"""Evaluator with contact-based fitness penalty."""

import math

import mujoco
import numpy as np
import numpy.typing as npt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body, Brick
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic, CpgNetworkStructure
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.scene import UUIDKey
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class Evaluator:
    """Evaluator with contact-based fitness: fitness = distance * (1 - non_leaf_contact_ratio)."""

    _terrain: object
    _cpg_network_structure: CpgNetworkStructure
    _body: Body
    _output_mapping: list[tuple[int, ActiveHinge]]
    _contact_mode: str
    last_distances: list[float]
    last_contact_ratios: list[float]

    @staticmethod
    def _is_leaf_brick(brick: Brick) -> bool:
        """Check if a brick is a leaf (has no children attached)."""
        # Check all attachment points (bricks have 3: front, left, right)
        if brick.front is not None:
            return False
        if brick.left is not None:
            return False
        if brick.right is not None:
            return False
        return True

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        cpg_network_structure: CpgNetworkStructure,
        body: Body,
        output_mapping: list[tuple[int, ActiveHinge]],
        contact_mode: str = "counting",
    ) -> None:
        """
        Initialize this object.

        :param headless: Not used (kept for compatibility).
        :param num_simulators: Not used (kept for compatibility).
        :param cpg_network_structure: Cpg structure for the brain.
        :param body: Modular body of the robot.
        :param output_mapping: Mapping between active hinges and cpg indices.
        :param contact_mode: Contact calculation mode - "counting" or "binary".
        """
        if contact_mode not in ["counting", "binary"]:
            raise ValueError(f"contact_mode must be 'counting' or 'binary', got: {contact_mode}")

        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body
        self._output_mapping = output_mapping
        self._contact_mode = contact_mode
        self.last_distances = []
        self.last_contact_ratios = []

    def evaluate(
        self,
        solutions: list[npt.NDArray[np.float_]],
    ) -> npt.NDArray[np.float_]:
        """
        Evaluate robots with contact-based fitness penalty.

        Fitness = distance * (1 - non_leaf_contact_ratio)

        :param solutions: Solutions to evaluate.
        :returns: Fitnesses.
        """
        fitnesses = []
        self.last_distances = []
        self.last_contact_ratios = []

        for params in solutions:
            # Create robot
            robot = ModularRobot(
                body=self._body,
                brain=BrainCpgNetworkStatic.uniform_from_params(
                    params=params,
                    cpg_network_structure=self._cpg_network_structure,
                    initial_state_uniform=math.sqrt(2) * 0.5,
                    output_mapping=self._output_mapping,
                ),
            )

            # Create scene
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)

            # Convert to simulation scene
            sim_scene, robot_mapping = scene.to_simulation_scene()

            # Get parameters
            batch_params = make_standard_batch_parameters()

            # Convert to MuJoCo model
            model, abstraction_mapping = scene_to_model(
                scene=sim_scene,
                simulation_timestep=batch_params.simulation_timestep,
                cast_shadows=False,
                fast_sim=True,
            )

            # Initialize MuJoCo data
            data = mujoco.MjData(model)

            # Get robot body ID
            multi_body_system = robot_mapping[UUIDKey(robot)]
            robot_body_id = abstraction_mapping.multi_body_system[
                UUIDKey(multi_body_system)
            ].id

            # Identify terrain geometries
            terrain_geom_ids = set()
            for i in range(model.ngeom):
                body_id = model.geom_bodyid[i]
                if body_id <= 1:
                    terrain_geom_ids.add(i)

            # Identify leaf brick bodies using MuJoCo body hierarchy
            # A body is a leaf if no other bodies have it as a parent
            leaf_brick_body_ids = set()

            # Build parent-child relationships
            children_count = {}
            for i in range(model.nbody):
                children_count[i] = 0

            for i in range(model.nbody):
                parent_id = model.body_parentid[i]
                if parent_id >= 0:
                    children_count[parent_id] += 1

            # Find leaf bodies: robot bodies with no children (end effectors/feet)
            for i in range(model.nbody):
                # Leaf = robot body (not world/terrain) with no children
                if i > 1 and children_count[i] == 0:
                    leaf_brick_body_ids.add(i)

            # Collect all robot body IDs
            all_robot_body_ids = set()
            for i in range(model.ngeom):
                body_id = model.geom_bodyid[i]
                geom_id = i
                # Robot geometries are those not in terrain
                if geom_id not in terrain_geom_ids and body_id > 1:
                    all_robot_body_ids.add(body_id)

            # Non-leaf bodies: all robot bodies except leaf bricks
            non_leaf_bodies = all_robot_body_ids - leaf_brick_body_ids

            # Contact tracking (per-step unique body contacts)
            steps = 0
            non_leaf_touch_sum = 0
            steps_with_contact = 0  # For binary mode

            # Initialize
            mujoco.mj_forward(model, data)
            initial_pos = data.xpos[robot_body_id].copy()

            # Simulation parameters
            control_step = 1.0 / batch_params.control_frequency
            simulation_time = batch_params.simulation_time

            last_control_time = 0.0

            # Create control interface
            control_interface = ControlInterfaceImpl(data, abstraction_mapping)

            # Simulation loop
            while data.time < simulation_time:
                # Control
                if data.time >= last_control_time + control_step:
                    last_control_time = math.floor(data.time / control_step) * control_step

                    simulation_state = SimulationStateImpl(
                        data=data,
                        abstraction_to_mujoco_mapping=abstraction_mapping,
                        camera_views={},
                    )

                    sim_scene.handler.handle(
                        simulation_state, control_interface, control_step
                    )

                # Step simulation
                mujoco.mj_step(model, data)

                steps += 1

                # Collect unique robot bodies that touched terrain this step
                touching_bodies = set()
                for i in range(data.ncon):
                    contact = data.contact[i]
                    geom1 = int(contact.geom1)
                    geom2 = int(contact.geom2)

                    robot_geom = None
                    if geom1 in terrain_geom_ids and geom2 not in terrain_geom_ids:
                        robot_geom = geom2
                    elif geom2 in terrain_geom_ids and geom1 not in terrain_geom_ids:
                        robot_geom = geom1

                    if robot_geom is not None:
                        body_id = int(model.geom_bodyid[robot_geom])
                        touching_bodies.add(body_id)

                # Count contacts based on mode
                touching_non_leaf = len(touching_bodies & non_leaf_bodies)
                if self._contact_mode == "counting":
                    # Counting mode: count how many non-leaf bodies are touching
                    non_leaf_touch_sum += touching_non_leaf
                else:  # binary
                    # Binary mode: just track if ANY non-leaf body is touching
                    if touching_non_leaf > 0:
                        steps_with_contact += 1

            # Get final position
            final_pos = data.xpos[robot_body_id].copy()

            # Calculate distance
            distance = float(
                np.sqrt(
                    (final_pos[0] - initial_pos[0]) ** 2
                    + (final_pos[1] - initial_pos[1]) ** 2
                )
            )

            # Calculate non-leaf contact ratio based on mode
            if steps > 0:
                if self._contact_mode == "counting":
                    # Counting: average fraction of non-leaf bodies touching per timestep
                    non_leaf_ratio = non_leaf_touch_sum / (len(non_leaf_bodies) * steps)
                else:  # binary
                    # Binary: fraction of timesteps with ANY non-leaf contact
                    non_leaf_ratio = steps_with_contact / steps
            else:
                non_leaf_ratio = 0.0

            # Apply multiplicative penalty: fitness = distance * (1 - non_leaf_ratio)
            fitness = distance * (1.0 - non_leaf_ratio**0.5)**2

            fitnesses.append(fitness)
            self.last_distances.append(distance)
            self.last_contact_ratios.append(non_leaf_ratio)

        return np.array(fitnesses)
