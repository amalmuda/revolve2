"""Evaluator class."""

import math

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic, CpgNetworkStructure
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import fitness_functions, terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class Evaluator:
    """Provides evaluation of robots."""

    _simulator: LocalSimulator
    _terrain: Terrain
    _cpg_network_structure: CpgNetworkStructure
    _body: Body
    _output_mapping: list[tuple[int, ActiveHinge]]

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

        :param headless: `headless` parameter for the physics simulator.
        :param num_simulators: `num_simulators` parameter for the physics simulator.
        :param cpg_network_structure: Cpg structure for the brain.
        :param body: Modular body of the robot.
        :param output_mapping: A mapping between active hinges and the index of their corresponding cpg in the cpg network structure.
        """
        self._simulator = LocalSimulator(
            headless=headless, num_simulators=num_simulators
        )
        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body
        self._output_mapping = output_mapping

    def evaluate(
        self,
        solutions: list[npt.NDArray[np.float_]],
    ) -> npt.NDArray[np.float_]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param solutions: Solutions to evaluate.
        :returns: Fitnesses of the solutions.
        """
        # Create robots from the brain parameters.
        robots = [
            ModularRobot(
                body=self._body,
                brain=BrainCpgNetworkStatic.uniform_from_params(
                    params=params,
                    cpg_network_structure=self._cpg_network_structure,
                    initial_state_uniform=math.sqrt(2) * 0.5,
                    output_mapping=self._output_mapping,
                ),
            )
            for params in solutions
        ]

        # Create the scenes.
        scenes = []
        for robot in robots:
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)
            scenes.append(scene)

        # Simulate all scenes.
        scene_states = simulate_scenes(
            simulator=self._simulator,
            batch_parameters=make_standard_batch_parameters(),
            scenes=scenes,
        )

        # Calculate the xy displacements.
        xy_displacements = [
            fitness_functions.xy_displacement(
                states[0].get_modular_robot_simulation_state(robot),
                states[-1].get_modular_robot_simulation_state(robot),
            )
            for robot, states in zip(robots, scene_states)
        ]

        return np.array(xy_displacements)

    def evaluate_with_contact_info(
        self,
        solutions: list[npt.NDArray[np.float_]],
        ground_threshold: float = 0.05,
    ) -> tuple[npt.NDArray[np.float_], list[dict]]:
        """
        Evaluate multiple robots and return fitness along with ground contact information.

        :param solutions: Solutions to evaluate.
        :param ground_threshold: Z-coordinate threshold for considering a body touching ground (in meters).
        :returns: Tuple of (fitnesses, contact_info_list) where contact_info_list contains dicts with contact statistics.
        """
        # Create robots from the brain parameters.
        robots = [
            ModularRobot(
                body=self._body,
                brain=BrainCpgNetworkStatic.uniform_from_params(
                    params=params,
                    cpg_network_structure=self._cpg_network_structure,
                    initial_state_uniform=math.sqrt(2) * 0.5,
                    output_mapping=self._output_mapping,
                ),
            )
            for params in solutions
        ]

        # Create the scenes.
        scenes = []
        for robot in robots:
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)
            scenes.append(scene)

        # Simulate all scenes.
        scene_states = simulate_scenes(
            simulator=self._simulator,
            batch_parameters=make_standard_batch_parameters(),
            scenes=scenes,
        )

        # Calculate the xy displacements and contact info.
        xy_displacements = []
        contact_info_list = []

        for robot, states in zip(robots, scene_states):
            # Calculate fitness
            xy_displacement = fitness_functions.xy_displacement(
                states[0].get_modular_robot_simulation_state(robot),
                states[-1].get_modular_robot_simulation_state(robot),
            )
            xy_displacements.append(xy_displacement)

            # Calculate ground contact statistics across all sampled states
            # We use the robot's core height as a proxy for ground contact
            # Lower height = more ground contact
            core_heights = []
            for state in states:
                robot_state = state.get_modular_robot_simulation_state(robot)
                pose = robot_state.get_pose()
                core_heights.append(pose.position.z)

            # Compile contact statistics
            # Check how many states have core below threshold
            states_in_contact = sum(1 for h in core_heights if h < ground_threshold)

            contact_info = {
                'avg_core_height': np.mean(core_heights),
                'min_core_height': np.min(core_heights),
                'max_core_height': np.max(core_heights),
                'states_in_contact': states_in_contact,
                'contact_ratio': states_in_contact / len(core_heights) if len(core_heights) > 0 else 0,
                'total_states': len(core_heights),
            }
            contact_info_list.append(contact_info)

        return np.array(xy_displacements), contact_info_list