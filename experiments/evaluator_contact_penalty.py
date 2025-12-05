"""Evaluator with contact-based fitness penalty."""

import math
from collections import defaultdict

import mujoco
import numpy as np
import numpy.typing as npt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
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
    last_distances: list[float]
    last_contact_ratios: list[float]

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
        cpg_network_structure: CpgNetworkStructure,
        body: Body,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        """
        Initialize this object.

        :param headless: Not used (kept for compatibility).
        :param num_simulators: Not used (kept for compatibility).
        :param cpg_network_structure: Cpg structure for the brain.
        :param body: Modular body of the robot.
        :param output_mapping: Mapping between active hinges and cpg indices.
        """
        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body
        self._output_mapping = output_mapping
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

        # Non-leaf bodies: Core (2) + ActiveHinges (3, 5, 7, 9)
        non_leaf_bodies = {2, 3, 5, 7, 9}

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

            # Contact tracking
            contact_data = defaultdict(int)

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

                # Collect contacts
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
                        contact_data[body_id] += 1

            # Get final position
            final_pos = data.xpos[robot_body_id].copy()

            # Calculate distance
            distance = float(
                np.sqrt(
                    (final_pos[0] - initial_pos[0]) ** 2
                    + (final_pos[1] - initial_pos[1]) ** 2
                )
            )

            # Calculate non-leaf contact ratio
            total_contacts = sum(contact_data.values())
            non_leaf_contacts = sum(
                contact_data.get(body_id, 0) for body_id in non_leaf_bodies
            )

            if total_contacts > 0:
                non_leaf_ratio = non_leaf_contacts / total_contacts
            else:
                non_leaf_ratio = 0.0

            # Apply multiplicative penalty: fitness = distance * (1 - non_leaf_ratio)
            fitness = distance * (1.0 - non_leaf_ratio**0.5)**2

            fitnesses.append(fitness)
            self.last_distances.append(distance)
            self.last_contact_ratios.append(non_leaf_ratio)

        return np.array(fitnesses)