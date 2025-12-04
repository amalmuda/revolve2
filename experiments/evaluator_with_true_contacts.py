"""Evaluator with true MuJoCo contact detection."""

import math
from collections import defaultdict

import mujoco
import numpy as np
import numpy.typing as npt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic, CpgNetworkStructure
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.scene import Pose, UUIDKey
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class EvaluatorWithTrueContacts:
    """Evaluator that tracks true ground contacts using MuJoCo contact data."""

    _terrain: object
    _cpg_network_structure: CpgNetworkStructure
    _body: Body
    _output_mapping: list[tuple[int, ActiveHinge]]

    def __init__(
        self,
        cpg_network_structure: CpgNetworkStructure,
        body: Body,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        """
        Initialize this object.

        :param cpg_network_structure: Cpg structure for the brain.
        :param body: Modular body of the robot.
        :param output_mapping: A mapping between active hinges and the index of their corresponding cpg in the cpg network structure.
        """
        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body
        self._output_mapping = output_mapping

    def evaluate_with_true_contacts(
        self,
        solutions: list[npt.NDArray[np.float_]],
    ) -> tuple[npt.NDArray[np.float_], list[dict]]:
        """
        Evaluate robots and return true ground contact information.

        :param solutions: Solutions to evaluate.
        :returns: Tuple of (fitnesses, contact_info_list).
        """
        results = []

        # Process each solution individually for contact tracking
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

            # Get simulation parameters
            batch_params = make_standard_batch_parameters()

            # Convert scene to MuJoCo model
            model, abstraction_mapping = scene_to_model(
                scene=sim_scene,
                simulation_timestep=batch_params.simulation_timestep,
                cast_shadows=False,
                fast_sim=True,
            )

            # Initialize MuJoCo data
            data = mujoco.MjData(model)

            # Get the multi-body system for our robot
            multi_body_system = robot_mapping[UUIDKey(robot)]
            robot_body_id = abstraction_mapping.multi_body_system[
                UUIDKey(multi_body_system)
            ].id

            # Identify terrain geometries (first body is terrain typically)
            terrain_geom_ids = set()
            for i in range(model.ngeom):
                body_id = model.geom_bodyid[i]
                # Body 0 is world, body 1 is terrain
                if body_id <= 1:
                    terrain_geom_ids.add(i)

            # Run simulation and collect contact data
            contact_data = defaultdict(int)  # body_id -> count of contacts
            total_steps = 0

            # Get initial position for fitness
            mujoco.mj_forward(model, data)
            initial_pos = data.xpos[robot_body_id].copy()

            control_step = 1.0 / batch_params.control_frequency
            simulation_time = batch_params.simulation_time
            simulation_timestep = batch_params.simulation_timestep

            last_control_time = 0.0
            time = 0.0

            # Note: Brain control is not implemented in this simplified version
            # The robot will be passive, but contact detection will still work

            while time < simulation_time:
                # Step simulation
                mujoco.mj_step(model, data)
                time += simulation_timestep
                total_steps += 1

                # Collect contact information
                for i in range(data.ncon):
                    contact = data.contact[i]
                    geom1 = int(contact.geom1)
                    geom2 = int(contact.geom2)

                    # Check if this is a robot-terrain contact
                    robot_geom = None
                    if geom1 in terrain_geom_ids and geom2 not in terrain_geom_ids:
                        robot_geom = geom2
                    elif geom2 in terrain_geom_ids and geom1 not in terrain_geom_ids:
                        robot_geom = geom1

                    if robot_geom is not None:
                        body_id = int(model.geom_bodyid[robot_geom])
                        contact_data[body_id] += 1

            # Get final position for fitness
            final_pos = data.xpos[robot_body_id].copy()

            # Calculate fitness (xy displacement)
            fitness = float(np.sqrt(
                (final_pos[0] - initial_pos[0]) ** 2
                + (final_pos[1] - initial_pos[1]) ** 2
            ))

            # Compile contact statistics
            total_contacts = sum(contact_data.values())
            num_bodies_with_contact = len(contact_data)

            contact_info = {
                'total_contacts': total_contacts,
                'avg_contacts_per_step': total_contacts / total_steps if total_steps > 0 else 0,
                'num_bodies_with_contact': num_bodies_with_contact,
                'total_steps': total_steps,
                'body_contact_counts': dict(contact_data),
                'body_contact_ratios': {
                    body_id: count / total_steps
                    for body_id, count in contact_data.items()
                },
            }

            results.append((fitness, contact_info))

        # Separate fitnesses and contact info
        fitnesses = np.array([r[0] for r in results])
        contact_infos = [r[1] for r in results]

        return fitnesses, contact_infos