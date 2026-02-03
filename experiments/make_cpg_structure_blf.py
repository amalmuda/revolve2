"""
BLF-Based CPG Network Structure for Revolve2.

Creates a CPG network structure using Body/Limb Finder (BLF) coupling rules
instead of the default neighbor-based coupling.

The BLF rules create bio-inspired connections:
- Spine ↔ Spine: fully coupled
- Hip ↔ Hip: fully coupled
- Hip → nearest Spine: coupled (if spine exists)
- Knee → its Hip: coupled
- Ankle → its Knee: coupled
"""

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import (
    CpgNetworkStructure,
    BrainCpgInstance,
)
from revolve2.modular_robot.brain.cpg._cpg_network_structure import Cpg, CpgPair
from revolve2.modular_robot.brain import Brain, BrainInstance

from blf_analyzer import BLFResult, BodyLimbFinder, JointType, analyze_body


def active_hinges_to_cpg_network_structure_blf(
    active_hinges: list[ActiveHinge],
    blf_result: BLFResult,
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create a CPG network structure based on BLF coupling rules.

    :param active_hinges: The active hinges in the robot.
    :param blf_result: The BLF analysis result containing coupling matrix.
    :returns: The created structure and a mapping between state indices and active hinges.
    """
    n = len(active_hinges)
    cpgs = CpgNetworkStructure.make_cpgs(n)
    connections: set[CpgPair] = set()

    # Use BLF coupling matrix to determine connections
    coupling_matrix = blf_result.coupling_matrix

    for i in range(n):
        for j in range(i + 1, n):  # Only upper triangle to avoid duplicates
            if coupling_matrix[i, j] > 0:
                connections.add(CpgPair(cpgs[i], cpgs[j]))

    cpg_network_structure = CpgNetworkStructure(cpgs, connections)

    return cpg_network_structure, [
        mapping for mapping in zip(cpg_network_structure.output_indices, active_hinges)
    ]


class BrainCpgNetworkBLF(Brain):
    """
    CPG brain that uses BLF-based coupling topology.

    Uses the Revolve2 CPG implementation (X' = WX with RK45 integration)
    but with coupling structure determined by Body/Limb Finder.
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        """
        Initialize the BLF CPG brain.

        :param initial_state: Initial state of the CPG network.
        :param weight_matrix: Weight matrix for integration.
        :param output_mapping: Mapping from state indices to active hinges.
        """
        self._initial_state = initial_state
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping

    def make_instance(self) -> BrainInstance:
        """Create a brain instance for simulation."""
        return BrainCpgInstance(
            initial_state=self._initial_state,
            weight_matrix=self._weight_matrix,
            output_mapping=self._output_mapping,
        )

    @classmethod
    def from_body(
        cls,
        body: Body,
        internal_weights: list[float] | float | None = None,
        external_weights: dict[tuple[int, int], float] | float | None = None,
        initial_state_value: float = 0.707,
    ) -> tuple["BrainCpgNetworkBLF", BLFResult, CpgNetworkStructure]:
        """
        Create a BLF CPG brain from a robot body.

        :param body: The robot body to analyze.
        :param internal_weights: Internal weights for each CPG (oscillation strength).
            Can be a single float (same for all), a list, or None for defaults.
        :param external_weights: External weights for connections.
            Can be a dict mapping (i,j) pairs to weights, a single float, or None.
        :param initial_state_value: Initial state value for the CPG.
        :returns: Tuple of (brain, blf_result, cpg_structure).
        """
        # Analyze body
        blf_result = analyze_body(body)
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n = len(active_hinges)

        # Create CPG structure using BLF
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(
            active_hinges, blf_result
        )

        # Set up internal weights (per-CPG oscillation strength)
        if internal_weights is None:
            # Default: use joint-type specific weights
            internal_weights_dict = {}
            for joint_info in blf_result.joints:
                cpg = cpg_structure.cpgs[joint_info.index]
                # Spine and hip have stronger internal coupling
                if joint_info.joint_type in [JointType.SPINE, JointType.HIP]:
                    internal_weights_dict[cpg] = 1.0
                else:
                    internal_weights_dict[cpg] = 0.8
        elif isinstance(internal_weights, (int, float)):
            internal_weights_dict = {cpg: float(internal_weights) for cpg in cpg_structure.cpgs}
        else:
            internal_weights_dict = {cpg: w for cpg, w in zip(cpg_structure.cpgs, internal_weights)}

        # Set up external weights (coupling between CPGs)
        if external_weights is None:
            # Default: use BLF-based coupling with default strength
            external_weights_dict = {pair: 1.0 for pair in cpg_structure.connections}
        elif isinstance(external_weights, (int, float)):
            external_weights_dict = {pair: float(external_weights) for pair in cpg_structure.connections}
        else:
            external_weights_dict = {}
            for pair in cpg_structure.connections:
                i, j = pair.cpg_index_lowest.index, pair.cpg_index_highest.index
                if (i, j) in external_weights:
                    external_weights_dict[pair] = external_weights[(i, j)]
                elif (j, i) in external_weights:
                    external_weights_dict[pair] = external_weights[(j, i)]
                else:
                    external_weights_dict[pair] = 1.0

        # Create weight matrix
        weight_matrix = cpg_structure.make_connection_weights_matrix(
            internal_weights_dict, external_weights_dict
        )

        # Create initial state
        initial_state = cpg_structure.make_uniform_state(initial_state_value)

        return cls(
            initial_state=initial_state,
            weight_matrix=weight_matrix,
            output_mapping=output_mapping,
        ), blf_result, cpg_structure

    @classmethod
    def from_parameters(
        cls,
        body: Body,
        params: npt.NDArray[np.float64] | list[float],
        initial_state_value: float = 0.707,
    ) -> "BrainCpgNetworkBLF":
        """
        Create from flat parameter array (for optimization).

        Parameter format: [internal_0, ..., internal_n-1, external_0, ..., external_m-1]
        where n = num_cpgs and m = num_connections from BLF.

        Total params = n + num_blf_connections

        :param body: The robot body.
        :param params: Flat array of parameters.
        :param initial_state_value: Initial state value.
        :returns: The created brain.
        """
        blf_result = analyze_body(body)
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n = len(active_hinges)

        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(
            active_hinges, blf_result
        )

        num_connections = len(cpg_structure.connections)
        expected = n + num_connections

        if len(params) != expected:
            raise ValueError(
                f"Expected {expected} params ({n} internal + {num_connections} external), "
                f"got {len(params)}"
            )

        # Split params into internal and external weights
        internal_params = list(params[:n])
        external_params = list(params[n:])

        internal_weights_dict = {
            cpg: w for cpg, w in zip(cpg_structure.cpgs, internal_params)
        }
        external_weights_dict = {
            pair: w for pair, w in zip(cpg_structure.connections, external_params)
        }

        weight_matrix = cpg_structure.make_connection_weights_matrix(
            internal_weights_dict, external_weights_dict
        )

        initial_state = cpg_structure.make_uniform_state(initial_state_value)

        return cls(
            initial_state=initial_state,
            weight_matrix=weight_matrix,
            output_mapping=output_mapping,
        )


def get_num_cpg_blf_params(body: Body) -> int:
    """
    Get the number of parameters for BLF CPG optimization.

    :param body: The robot body.
    :returns: Number of parameters (internal + external weights).
    """
    blf_result = analyze_body(body)
    active_hinges = body.find_modules_of_type(ActiveHinge)

    cpg_structure, _ = active_hinges_to_cpg_network_structure_blf(
        active_hinges, blf_result
    )

    return cpg_structure.num_connections  # internal + external


def print_cpg_structure_comparison(body: Body) -> None:
    """
    Print comparison of BLF vs neighbor-based CPG structures.

    :param body: The robot body to analyze.
    """
    from revolve2.modular_robot.brain.cpg import (
        active_hinges_to_cpg_network_structure_neighbor,
    )

    active_hinges = body.find_modules_of_type(ActiveHinge)
    blf_result = analyze_body(body)

    # BLF structure
    blf_structure, _ = active_hinges_to_cpg_network_structure_blf(active_hinges, blf_result)

    # Neighbor structure
    neighbor_structure, _ = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    print("=" * 60)
    print("CPG Structure Comparison")
    print("=" * 60)
    print(f"\nNumber of CPGs: {len(active_hinges)}")
    print(f"\nBLF-based connections: {len(blf_structure.connections)}")
    print(f"  Params needed: {blf_structure.num_connections}")
    blf_pairs = sorted([(p.cpg_index_lowest.index, p.cpg_index_highest.index)
                        for p in blf_structure.connections])
    print(f"  Connections: {blf_pairs}")

    print(f"\nNeighbor-based connections: {len(neighbor_structure.connections)}")
    print(f"  Params needed: {neighbor_structure.num_connections}")
    neighbor_pairs = sorted([(p.cpg_index_lowest.index, p.cpg_index_highest.index)
                             for p in neighbor_structure.connections])
    print(f"  Connections: {neighbor_pairs}")


# =============================================================================
# Demo
# =============================================================================

if __name__ == "__main__":
    import math
    from revolve2.modular_robot import ModularRobot
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards import modular_robots_v1, modular_robots_v2
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    print("=" * 60)
    print("BLF Revolve2 CPG Demo")
    print("=" * 60)

    # Test with spider
    print("\n--- SPIDER ---")
    spider_body = modular_robots_v1.get("spider")
    print_cpg_structure_comparison(spider_body)

    spider_brain, spider_blf, spider_cpg = BrainCpgNetworkBLF.from_body(spider_body)
    spider_blf.print_summary()

    print(f"\nWeight matrix shape: {spider_brain._weight_matrix.shape}")
    print(f"Number of params needed: {get_num_cpg_blf_params(spider_body)}")

    spider_robot = ModularRobot(body=spider_body, brain=spider_brain)

    simulator = LocalSimulator(headless=True, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(spider_robot)

    print("\nRunning simulation...")
    states = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)
    final_state = states[-1]
    pose = final_state.get_modular_robot_simulation_state(spider_robot).get_pose()
    dist = math.sqrt(pose.position.x**2 + pose.position.y**2)
    print(f"Spider distance: {dist:.3f} m")

    # Test with gecko
    print("\n--- GECKO ---")
    gecko_body = modular_robots_v2.get("gecko")
    print_cpg_structure_comparison(gecko_body)

    gecko_brain, gecko_blf, gecko_cpg = BrainCpgNetworkBLF.from_body(gecko_body)
    gecko_blf.print_summary()

    print(f"\nWeight matrix shape: {gecko_brain._weight_matrix.shape}")
    print(f"Number of params needed: {get_num_cpg_blf_params(gecko_body)}")

    gecko_robot = ModularRobot(body=gecko_body, brain=gecko_brain)

    scene2 = ModularRobotScene(terrain=flat())
    scene2.add_robot(gecko_robot)

    print("\nRunning simulation...")
    states2 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene2)
    final_state2 = states2[-1]
    pose2 = final_state2.get_modular_robot_simulation_state(gecko_robot).get_pose()
    dist2 = math.sqrt(pose2.position.x**2 + pose2.position.y**2)
    print(f"Gecko distance: {dist2:.3f} m")
