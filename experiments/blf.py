"""
Body/Limb Finder (BLF) for Revolve2 modular robots.

Adapted from:
"Automatic generation of reduced CPG control networks for locomotion of
arbitrary modular robot structures" by Bonardi et al. (2014, EPFL)

Modified for tree-structured bodies in Revolve2:
- Core is always body (rule 1).
- Brick clusters with >2 hinges directly touching them are body (rule 2).
  This replaces Bonardi's articulation-point approach with a simpler rule
  that identifies junction points where multiple branches meet.
- Modules on the path between body regions are body (rule 3).
- All body hinges are spine (no locking, unlike Bonardi who activates only 1).
- First hinge in each limb = hip, all remaining = knee (no ankle distinction).
- Spine coupling: all-to-all. Hip coupling: all-to-all + nearest spine(s).
  Knee coupling: chain within limb (each connects to previous hinge).

See blf_rules.md for the full rule description.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import TYPE_CHECKING

from revolve2.modular_robot.body.base import ActiveHinge, Body, Core
from revolve2.modular_robot.body._module import Module
from revolve2.modular_robot.brain.cpg._cpg_network_structure import (
    Cpg,
    CpgNetworkStructure,
    CpgPair,
)

if TYPE_CHECKING:
    pass


class JointType(Enum):
    """Types of joints identified by BLF."""

    SPINE = auto()
    HIP = auto()
    KNEE = auto()
    UNCLASSIFIED = auto()


class PartType(Enum):
    """Classification of robot parts."""

    BODY = auto()
    LIMB = auto()
    CORE = auto()


@dataclass
class Node:
    """A node in the robot graph representing a module."""

    module: Module
    index: int
    neighbors: list[int] = field(default_factory=list)
    part_type: PartType = PartType.LIMB
    joint_type: JointType = JointType.UNCLASSIFIED
    limb_id: int = -1  # Which limb this node belongs to (-1 = body)
    distance_from_body: int = -1  # Distance from body (for limb nodes)



@dataclass
class LimbSignature:
    """Signature of a limb for symmetry detection."""

    length: int  # Number of modules in limb
    active_hinge_count: int  # Number of active hinges
    joint_sequence: tuple[JointType, ...]  # Sequence of joint types from body to tip
    module_types: tuple[str, ...]  # Sequence of module type names


@dataclass
class BLFResult:
    """Result of Body/Limb Finder analysis."""

    nodes: list[Node]
    body_nodes: list[int]  # Indices of body nodes
    limbs: list[list[int]]  # List of limbs, each limb is a list of node indices
    articulations: dict[int, JointType]  # Node index -> joint type
    active_hinge_nodes: list[int]  # Indices of nodes that are ActiveHinges
    symmetric_groups: list[list[int]] = field(default_factory=list)  # Groups of symmetric limbs


class BodyLimbFinder:
    """
    Body/Limb Finder algorithm implementation.

    Analyzes a modular robot structure to identify:
    - Body vs limb modules
    - Joint types (spine, hip, knee)
    - Symmetries between limbs
    """

    def __init__(self, body: Body) -> None:
        """
        Initialize the BLF with a robot body.

        :param body: The robot body to analyze.
        """
        self._body = body
        self._nodes: list[Node] = []
        self._module_to_index: dict[Module, int] = {}
        self._adjacency: list[list[int]] = []

    def analyze(self) -> BLFResult:
        """
        Run the full BLF analysis.

        Rules (adapted from Bonardi et al. 2014 for tree-structured bodies):
        1. Core is always body.
        2. Brick clusters with more than 2 hinges directly touching them are body
           (they are junction points where multiple limbs/branches meet).
        3. Modules on the path between body regions are body.
        4. Everything else is a limb.
        5. Body hinges = spine (all-to-all coupling).
        6. First hinge in limb = hip (all-to-all + nearest spine).
        7. Remaining hinges in limb = knee (chained to previous hinge).

        :returns: The BLF analysis result.
        """
        # Step 1: Build graph from robot
        self._build_graph()

        # Step 2: Identify body and limbs using brick-cluster hinge-count rule
        body_nodes, limbs = self._find_body_and_limbs()

        # Step 4: Classify joints
        articulations = self._classify_joints(body_nodes, limbs)

        # Get active hinge nodes
        active_hinge_nodes = [
            node.index
            for node in self._nodes
            if isinstance(node.module, ActiveHinge)
        ]

        # Step 5: Find symmetric limbs
        symmetric_groups = self._find_symmetric_limbs(limbs)

        return BLFResult(
            nodes=self._nodes,
            body_nodes=body_nodes,
            limbs=limbs,
            articulations=articulations,
            active_hinge_nodes=active_hinge_nodes,
            symmetric_groups=symmetric_groups,
        )

    def _build_graph(self) -> None:
        """Build graph representation from robot body."""
        # BFS to traverse all modules
        queue: list[Module] = [self._body.core]
        visited: set[Module] = set()

        while queue:
            module = queue.pop(0)
            if module in visited:
                continue
            visited.add(module)

            # Create node for this module
            idx = len(self._nodes)
            node = Node(module=module, index=idx)

            # Mark core specially
            if isinstance(module, Core):
                node.part_type = PartType.CORE

            self._nodes.append(node)
            self._module_to_index[module] = idx
            self._adjacency.append([])

            # Add children to queue
            for child in module.children.values():
                queue.append(child)

        # Build adjacency list (edges between parent-child)
        for node in self._nodes:
            module = node.module
            # Add edge to parent
            if module.parent is not None and module.parent in self._module_to_index:
                parent_idx = self._module_to_index[module.parent]
                if parent_idx not in node.neighbors:
                    node.neighbors.append(parent_idx)
                if node.index not in self._nodes[parent_idx].neighbors:
                    self._nodes[parent_idx].neighbors.append(node.index)

    def _find_body_and_limbs(self) -> tuple[list[int], list[list[int]]]:
        """
        Identify body and limbs using brick-cluster hinge-count rules.

        Rules:
        1. Core is always body.
        2. A connected group of non-hinge modules (brick cluster) is body
           if more than 2 active hinges are directly attached to it.
           This identifies junction points between multiple limbs/branches.
        3. All modules on the shortest path between two body regions are body.
           This captures spine hinges and intermediate bricks.
        4. Everything not classified as body is a limb.

        :returns: Tuple of (body node indices, list of limb node lists).
        """
        n = len(self._nodes)

        # Rule 1: Core is always body
        body_nodes: list[int] = []
        core_idx = 0  # Core is always first node
        body_nodes.append(core_idx)
        self._nodes[core_idx].part_type = PartType.BODY

        # Rule 2: Find brick clusters with more than 2 hinges touching them.
        # A brick cluster is a connected group of non-hinge modules (excluding core).
        # If >2 hinges are direct neighbors of the cluster, it's a junction -> body.
        visited_for_clusters: set[int] = set()
        for start_idx in range(n):
            if start_idx in visited_for_clusters:
                continue
            node = self._nodes[start_idx]
            # Skip hinges and core (core is already body)
            if isinstance(node.module, ActiveHinge) or isinstance(node.module, Core):
                continue

            # BFS to find connected brick cluster
            cluster: list[int] = []
            cluster_queue = [start_idx]
            while cluster_queue:
                idx = cluster_queue.pop(0)
                if idx in visited_for_clusters:
                    continue
                cur_node = self._nodes[idx]
                if isinstance(cur_node.module, ActiveHinge) or isinstance(cur_node.module, Core):
                    continue
                visited_for_clusters.add(idx)
                cluster.append(idx)
                for neighbor in cur_node.neighbors:
                    if neighbor not in visited_for_clusters:
                        cluster_queue.append(neighbor)

            if not cluster:
                continue

            # Count how many hinges directly touch this cluster
            cluster_set = set(cluster)
            touching_hinges: set[int] = set()
            for cidx in cluster:
                for neighbor in self._nodes[cidx].neighbors:
                    if isinstance(self._nodes[neighbor].module, ActiveHinge):
                        touching_hinges.add(neighbor)

            # More than 2 hinges = junction point = body
            if len(touching_hinges) > 2:
                for cidx in cluster:
                    if cidx not in body_nodes:
                        body_nodes.append(cidx)
                        self._nodes[cidx].part_type = PartType.BODY

        # Rule 3: Include all modules on shortest paths between body regions.
        # This captures spine hinges and bricks connecting the core to
        # distant body brick clusters.
        if len(body_nodes) > 1:
            body_set = set(body_nodes)
            for i_b in range(len(body_nodes)):
                for j_b in range(i_b + 1, len(body_nodes)):
                    path = self._find_shortest_path(
                        body_nodes[i_b], body_nodes[j_b]
                    )
                    for node_idx in path:
                        if node_idx not in body_set:
                            body_set.add(node_idx)
                            body_nodes.append(node_idx)
                            self._nodes[node_idx].part_type = PartType.BODY

        # Find limbs: connected components after removing body
        limbs: list[list[int]] = []
        visited = [False] * n
        for idx in body_nodes:
            visited[idx] = True

        limb_id = 0
        for start_idx in range(n):
            if visited[start_idx]:
                continue

            # BFS to find this limb
            limb: list[int] = []
            queue = [start_idx]
            while queue:
                idx = queue.pop(0)
                if visited[idx]:
                    continue
                visited[idx] = True
                limb.append(idx)
                self._nodes[idx].part_type = PartType.LIMB
                self._nodes[idx].limb_id = limb_id

                for neighbor in self._nodes[idx].neighbors:
                    if not visited[neighbor] and neighbor not in body_nodes:
                        queue.append(neighbor)

            if limb:
                limbs.append(limb)
                limb_id += 1

        # Calculate distance from body for limb nodes
        for limb in limbs:
            self._calculate_distances_from_body(limb, body_nodes)

        return body_nodes, limbs

    def _find_shortest_path(self, src: int, dst: int) -> list[int]:
        """
        Find shortest path between two nodes using BFS.

        :param src: Source node index.
        :param dst: Destination node index.
        :returns: List of node indices on the path (inclusive of src and dst).
        """
        from collections import deque

        parent: dict[int, int] = {src: -1}
        queue: deque[int] = deque([src])

        while queue:
            current = queue.popleft()
            if current == dst:
                # Reconstruct path
                path = []
                node = dst
                while node != -1:
                    path.append(node)
                    node = parent[node]
                return path[::-1]
            for neighbor in self._nodes[current].neighbors:
                if neighbor not in parent:
                    parent[neighbor] = current
                    queue.append(neighbor)

        return []  # No path found (shouldn't happen in connected graph)

    def _calculate_distances_from_body(
        self, limb: list[int], body_nodes: list[int]
    ) -> None:
        """
        Calculate distance from body for each node in a limb.

        :param limb: List of node indices in the limb.
        :param body_nodes: List of body node indices.
        """
        # Find the node(s) in limb that connect to body
        body_set = set(body_nodes)
        limb_set = set(limb)

        # BFS from body-adjacent nodes
        queue: list[tuple[int, int]] = []  # (node_idx, distance)
        visited: set[int] = set()

        for node_idx in limb:
            for neighbor in self._nodes[node_idx].neighbors:
                if neighbor in body_set:
                    queue.append((node_idx, 0))
                    break

        while queue:
            idx, dist = queue.pop(0)
            if idx in visited:
                continue
            visited.add(idx)
            self._nodes[idx].distance_from_body = dist

            for neighbor in self._nodes[idx].neighbors:
                if neighbor in limb_set and neighbor not in visited:
                    queue.append((neighbor, dist + 1))

    def _classify_joints(
        self, body_nodes: list[int], limbs: list[list[int]]
    ) -> dict[int, JointType]:
        """
        Classify active hinges by their role in the robot.

        Rules:
        - Spine: any active hinge classified as body (connects body regions).
        - Hip: the first (closest to body) active hinge in each limb.
        - Knee: all remaining active hinges after the hip in a limb.
          No ankle distinction — coupling within a limb is always a chain
          (each knee connects to the previous hinge), so the label doesn't
          change the coupling structure.

        :param body_nodes: List of body node indices.
        :param limbs: List of limbs (each limb is list of node indices).
        :returns: Dictionary mapping node index to joint type.
        """
        articulations: dict[int, JointType] = {}

        # All body hinges are spine
        for idx in body_nodes:
            if isinstance(self._nodes[idx].module, ActiveHinge):
                articulations[idx] = JointType.SPINE
                self._nodes[idx].joint_type = JointType.SPINE

        # Classify limb joints: first = hip, rest = knee
        for limb in limbs:
            active_hinges_in_limb = [
                idx for idx in limb if isinstance(self._nodes[idx].module, ActiveHinge)
            ]

            if not active_hinges_in_limb:
                continue

            # Sort by distance from body
            active_hinges_in_limb.sort(
                key=lambda x: self._nodes[x].distance_from_body
            )

            # First ActiveHinge closest to body = HIP
            hip_idx = active_hinges_in_limb[0]
            articulations[hip_idx] = JointType.HIP
            self._nodes[hip_idx].joint_type = JointType.HIP

            # All remaining = KNEE (no ankle distinction)
            for idx in active_hinges_in_limb[1:]:
                articulations[idx] = JointType.KNEE
                self._nodes[idx].joint_type = JointType.KNEE

        return articulations

    def _get_limb_signature(self, limb: list[int]) -> LimbSignature:
        """
        Get the signature of a limb for symmetry comparison.

        :param limb: List of node indices in the limb.
        :returns: The limb signature.
        """
        # Sort limb nodes by distance from body
        sorted_limb = sorted(limb, key=lambda x: self._nodes[x].distance_from_body)

        # Get joint sequence
        joint_sequence = tuple(
            self._nodes[idx].joint_type
            for idx in sorted_limb
            if isinstance(self._nodes[idx].module, ActiveHinge)
        )

        # Get module type sequence
        module_types = tuple(type(self._nodes[idx].module).__name__ for idx in sorted_limb)

        # Count active hinges
        active_hinge_count = sum(
            1 for idx in limb if isinstance(self._nodes[idx].module, ActiveHinge)
        )

        return LimbSignature(
            length=len(limb),
            active_hinge_count=active_hinge_count,
            joint_sequence=joint_sequence,
            module_types=module_types,
        )

    def _find_symmetric_limbs(self, limbs: list[list[int]]) -> list[list[int]]:
        """
        Find groups of symmetric limbs based on their signatures.

        Two limbs are symmetric if they have identical signatures
        (same length, same module types, same joint sequence).

        :param limbs: List of limbs.
        :returns: List of symmetric groups (each group is a list of limb indices).
        """
        if not limbs:
            return []

        # Compute signature for each limb
        signatures = [self._get_limb_signature(limb) for limb in limbs]

        # Group limbs by signature
        signature_to_limbs: dict[tuple, list[int]] = {}
        for limb_idx, sig in enumerate(signatures):
            # Create hashable key from signature
            key = (sig.length, sig.active_hinge_count, sig.joint_sequence, sig.module_types)
            if key not in signature_to_limbs:
                signature_to_limbs[key] = []
            signature_to_limbs[key].append(limb_idx)

        # Return groups with more than one limb (actual symmetry)
        symmetric_groups = [
            group for group in signature_to_limbs.values() if len(group) > 1
        ]

        return symmetric_groups


class BLFCpgNetworkGenerator:
    """
    Generate reduced CPG networks based on BLF analysis.

    Creates CPG coupling patterns inspired by vertebrate locomotion:
    - Spine oscillators are fully coupled (all-to-all)
    - Hip oscillators are fully coupled (all-to-all) and connected to nearest spine(s)
    - Knee oscillators are chained within their limb (each to previous hinge)
    """

    def __init__(self, blf_result: BLFResult, use_symmetry: bool = False) -> None:
        """
        Initialize the generator.

        :param blf_result: Result from BLF analysis.
        :param use_symmetry: Whether to use symmetry constraints (BLF-SYM).
        """
        self._result = blf_result
        self._use_symmetry = use_symmetry
        self._active_hinges: list[ActiveHinge] = []
        self._hinge_to_cpg: dict[int, Cpg] = {}
        self._node_to_cpg: dict[int, Cpg] = {}
        self._cpg_to_node: dict[int, int] = {}
        self._symmetric_cpg_groups: list[list[int]] = []  # Groups of CPGs that share params

    def generate(self) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
        """
        Generate a reduced CPG network structure.

        :returns: Tuple of (CPG network structure, mapping of CPG indices to hinges).
        """
        # Get all ActiveHinges with their classifications
        classified_hinges: dict[JointType, list[int]] = {
            JointType.SPINE: [],
            JointType.HIP: [],
            JointType.KNEE: [],
        }

        for idx in self._result.active_hinge_nodes:
            node = self._result.nodes[idx]
            jtype = node.joint_type
            if jtype in classified_hinges:
                classified_hinges[jtype].append(idx)

        # Create CPGs for each classified hinge
        cpg_list: list[Cpg] = []
        node_to_cpg: dict[int, Cpg] = {}

        for jtype in [JointType.SPINE, JointType.HIP, JointType.KNEE]:
            for node_idx in classified_hinges[jtype]:
                cpg = Cpg(len(cpg_list))
                cpg_list.append(cpg)
                node_to_cpg[node_idx] = cpg
                self._active_hinges.append(self._result.nodes[node_idx].module)

        # Build connections based on bio-inspired rules
        connections: set[CpgPair] = set()

        # 1. Spine oscillators fully coupled
        spine_cpgs = [node_to_cpg[idx] for idx in classified_hinges[JointType.SPINE]]
        for i, cpg1 in enumerate(spine_cpgs):
            for cpg2 in spine_cpgs[i + 1 :]:
                connections.add(CpgPair(cpg1, cpg2))

        # 2. Hip oscillators fully coupled
        hip_cpgs = [node_to_cpg[idx] for idx in classified_hinges[JointType.HIP]]
        for i, cpg1 in enumerate(hip_cpgs):
            for cpg2 in hip_cpgs[i + 1 :]:
                connections.add(CpgPair(cpg1, cpg2))

        # 3. Hip connected to nearest spine (BFS through graph).
        #    If a hip is equidistant to multiple spines, connect to all of them.
        if spine_cpgs:
            from collections import deque
            spine_node_set = set(classified_hinges[JointType.SPINE])
            for hip_idx in classified_hinges[JointType.HIP]:
                hip_cpg = node_to_cpg[hip_idx]
                # BFS from hip to find ALL nearest spine nodes at the same distance
                bfs_queue = deque([(hip_idx, 0)])
                bfs_visited = {hip_idx}
                nearest_spines: list[int] = []
                nearest_dist: int | None = None
                while bfs_queue:
                    current, dist = bfs_queue.popleft()
                    # If we already found nearest spines and current is further, stop
                    if nearest_dist is not None and dist > nearest_dist:
                        break
                    if current in spine_node_set:
                        nearest_spines.append(current)
                        nearest_dist = dist
                    for neighbor in self._result.nodes[current].neighbors:
                        if neighbor not in bfs_visited:
                            bfs_visited.add(neighbor)
                            bfs_queue.append((neighbor, dist + 1))
                for spine_idx in nearest_spines:
                    connections.add(CpgPair(hip_cpg, node_to_cpg[spine_idx]))

        # 4. Knee coupling: chain within each limb.
        #    Each knee connects to the previous hinge (closer to body) in the
        #    same limb, forming: hip <- knee1 <- knee2 <- knee3 ...
        for limb in self._result.limbs:
            hinges_in_limb = [
                idx for idx in limb
                if isinstance(self._result.nodes[idx].module, ActiveHinge)
            ]
            if len(hinges_in_limb) < 2:
                continue
            # Sort by distance from body (hip first, then knees in order)
            hinges_in_limb.sort(
                key=lambda x: self._result.nodes[x].distance_from_body
            )
            # Chain: each hinge connects to the one before it
            for i in range(1, len(hinges_in_limb)):
                connections.add(CpgPair(
                    node_to_cpg[hinges_in_limb[i]],
                    node_to_cpg[hinges_in_limb[i - 1]],
                ))

        structure = CpgNetworkStructure(cpg_list, connections)

        # Build mapping
        mapping = [
            (cpg.index, self._active_hinges[cpg.index]) for cpg in cpg_list
        ]

        # Store for later use
        self._node_to_cpg = node_to_cpg
        self._cpg_to_node = {cpg.index: node_idx for node_idx, cpg in node_to_cpg.items()}

        # Find symmetric CPG groups if using symmetry
        if self._use_symmetry:
            self._symmetric_cpg_groups = self._find_symmetric_cpg_groups(
                classified_hinges, node_to_cpg
            )

        return structure, mapping

    def _find_symmetric_cpg_groups(
        self,
        classified_hinges: dict[JointType, list[int]],
        node_to_cpg: dict[int, Cpg],
    ) -> list[list[int]]:
        """
        Find groups of CPGs that should share amplitude/offset parameters.

        Based on limb symmetry: symmetric limbs have CPGs that share parameters.

        :returns: List of groups, each group is a list of CPG indices.
        """
        groups: list[list[int]] = []

        for sym_group in self._result.symmetric_groups:
            # For each joint type, group the CPGs from symmetric limbs
            for jtype in [JointType.HIP, JointType.KNEE]:
                cpg_group: list[int] = []
                for limb_idx in sym_group:
                    limb = self._result.limbs[limb_idx]
                    for node_idx in limb:
                        node = self._result.nodes[node_idx]
                        if node.joint_type == jtype and node_idx in node_to_cpg:
                            cpg_group.append(node_to_cpg[node_idx].index)
                if len(cpg_group) > 1:
                    groups.append(cpg_group)

        return groups

    def get_symmetric_groups(self) -> list[list[int]]:
        """
        Get groups of CPGs that should share amplitude/offset parameters.

        :returns: List of groups, each group is a list of CPG indices.
        """
        return self._symmetric_cpg_groups

    def get_num_unique_params(self) -> int:
        """
        Get number of unique parameters when using symmetry.

        Symmetric CPGs share amplitude and offset, but not phase.

        :returns: Number of unique parameters.
        """
        num_cpgs = len(self._active_hinges)

        if not self._use_symmetry or not self._symmetric_cpg_groups:
            # No symmetry: 2 params per CPG (amplitude, offset)
            return num_cpgs * 2

        # With symmetry: shared amplitude/offset for symmetric groups
        # Count how many CPGs are in symmetric groups
        cpgs_in_groups = set()
        for group in self._symmetric_cpg_groups:
            cpgs_in_groups.update(group)

        # Non-symmetric CPGs: 2 params each
        non_symmetric = num_cpgs - len(cpgs_in_groups)
        params = non_symmetric * 2

        # Symmetric groups: 2 params per group (shared amplitude/offset)
        params += len(self._symmetric_cpg_groups) * 2

        return params

    def get_reduced_parameter_info(
        self,
        num_connections: int,
        external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
    ) -> dict:
        """
        Get info for reduced parameter optimization with symmetry.

        Returns information needed to:
        1. Evolve only unique parameters
        2. Expand unique params back to full params

        Following the EPFL paper (Bonardi et al.):
        - Symmetric limbs share AMPLITUDE and OFFSET (internal weights)
        - Phase shifts (external coupling weights) remain INDEPENDENT

        "the corresponding oscillators share the same amplitude and the same
        offset (the phase shift remaining open to avoid restricting the
        possible gait patterns)" - Section III.C

        For spider (4 symmetric legs, each with HIP+KNEE):
        - 2 unique internal params (1 hip amplitude, 1 knee amplitude)
        - 10 independent external params (all couplings independent for gait flexibility)
        = 12 total unique params instead of 18

        :param num_connections: Total number of connections in full network.
        :param external_weight_bounds: Bounds for external coupling weights.
        :returns: Dict with reduction info.
        """
        num_cpgs = len(self._active_hinges)
        num_external = num_connections - num_cpgs

        if not self._use_symmetry or not self._symmetric_cpg_groups:
            # No reduction - return full params info
            return {
                "num_unique_params": num_connections,
                "unique_internal": num_cpgs,
                "unique_external": num_external,
                "internal_expansion_map": list(range(num_cpgs)),
                "external_expansion_map": list(range(num_external)),
                "lower_bounds": None,  # Use full bounds
                "upper_bounds": None,
            }

        # Build internal param reduction (amplitude per joint type in symmetric groups)
        # Symmetric limbs share amplitude - this IS what the paper does
        unique_internal_params: list[int] = []  # CPG indices for unique params
        internal_expansion_map: list[int] = []  # For each CPG, which unique param to use

        # Track which CPGs are in symmetric groups
        cpg_to_group: dict[int, int] = {}
        for group_idx, group in enumerate(self._symmetric_cpg_groups):
            for cpg_idx in group:
                cpg_to_group[cpg_idx] = group_idx

        # For each CPG, either it's unique or part of a symmetric group
        group_representative: dict[int, int] = {}  # group_idx -> representative cpg
        for cpg_idx in range(num_cpgs):
            if cpg_idx in cpg_to_group:
                group_idx = cpg_to_group[cpg_idx]
                if group_idx not in group_representative:
                    # First CPG in this group becomes representative
                    group_representative[group_idx] = len(unique_internal_params)
                    unique_internal_params.append(cpg_idx)
                internal_expansion_map.append(group_representative[group_idx])
            else:
                # Not in any symmetric group - unique param
                internal_expansion_map.append(len(unique_internal_params))
                unique_internal_params.append(cpg_idx)

        # IMPORTANT: External weights (coupling/phase) are NOT reduced!
        # The paper explicitly keeps phase shifts independent to allow different gaits:
        # "the phase shift remaining open to avoid restricting the possible gait patterns"
        # This allows trot, bound, walk, etc. which require different phase relationships.
        external_expansion_map = list(range(num_external))  # Identity map - no reduction

        # Build bounds for unique params
        lower_bounds: list[float] = []
        upper_bounds: list[float] = []

        # Internal bounds (uniform [-1, 1] — same as CMA-ES weight bounds)
        for cpg_idx in unique_internal_params:
            lower_bounds.append(external_weight_bounds[0])
            upper_bounds.append(external_weight_bounds[1])

        # External bounds (uniform) - all connections independent
        for _ in range(num_external):
            lower_bounds.append(external_weight_bounds[0])
            upper_bounds.append(external_weight_bounds[1])

        return {
            "num_unique_params": len(unique_internal_params) + num_external,
            "unique_internal": len(unique_internal_params),
            "unique_external": num_external,  # All external params are unique
            "internal_expansion_map": internal_expansion_map,
            "external_expansion_map": external_expansion_map,
            "lower_bounds": lower_bounds,
            "upper_bounds": upper_bounds,
        }

    def expand_reduced_params(
        self,
        reduced_params: list[float],
        reduction_info: dict,
    ) -> list[float]:
        """
        Expand reduced (unique) parameters to full parameter vector.

        :param reduced_params: The reduced parameter vector.
        :param reduction_info: Info from get_reduced_parameter_info().
        :returns: Full parameter vector.
        """
        unique_internal = reduction_info["unique_internal"]
        internal_map = reduction_info["internal_expansion_map"]
        external_map = reduction_info["external_expansion_map"]

        # Split reduced params into internal and external
        reduced_internal = reduced_params[:unique_internal]
        reduced_external = reduced_params[unique_internal:]

        # Expand internal params
        full_internal = [reduced_internal[idx] for idx in internal_map]

        # Expand external params
        full_external = [reduced_external[idx] for idx in external_map]

        return full_internal + full_external


def analyze_robot(body: Body) -> BLFResult:
    """
    Convenience function to analyze a robot body.

    :param body: The robot body.
    :returns: BLF analysis result.
    """
    finder = BodyLimbFinder(body)
    return finder.analyze()


def get_blf_parameter_bounds(
    body: Body,
    use_symmetry: bool = False,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> tuple[list[float], list[float]]:
    """
    Get per-parameter bounds for BLF-based CPG optimization.

    Uses joint-type specific amplitude bounds from the paper:
    - SPINE: [0, 2π/3] rad
    - HIP: [0, π/2] rad
    - KNEE: [0, π/6] rad

    External coupling weights use uniform bounds.

    When use_symmetry=True, returns REDUCED bounds (only unique params).

    :param body: The robot body.
    :param use_symmetry: Whether to use BLF-SYM mode (reduced params).
    :param external_weight_bounds: Bounds for external coupling weights.
    :returns: Tuple of (lower_bounds, upper_bounds) lists.
    """
    structure, mapping, result, generator = generate_blf_cpg_network(body, use_symmetry)

    num_cpgs = len(mapping)
    num_external = structure.num_connections - num_cpgs

    if use_symmetry:
        # Return reduced bounds for unique parameters only
        info = generator.get_reduced_parameter_info(
            num_connections=structure.num_connections,
            external_weight_bounds=external_weight_bounds,
        )
        return info["lower_bounds"], info["upper_bounds"]
    else:
        # Uniform bounds for all params (internal + external)
        lower = [external_weight_bounds[0]] * structure.num_connections
        upper = [external_weight_bounds[1]] * structure.num_connections
        return lower, upper


def get_blf_symmetry_expansion_info(
    body: Body,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> dict:
    """
    Get symmetry expansion info for BLF-SYM.

    Used to expand reduced parameters back to full parameters during evaluation.

    :param body: The robot body.
    :param external_weight_bounds: Bounds for external coupling weights.
    :returns: Dict with expansion info.
    """
    structure, mapping, result, generator = generate_blf_cpg_network(body, use_symmetry=True)
    return generator.get_reduced_parameter_info(
        num_connections=structure.num_connections,
        external_weight_bounds=external_weight_bounds,
    )


def expand_blf_sym_params(
    reduced_params: list[float],
    body: Body,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> list[float]:
    """
    Expand reduced BLF-SYM parameters to full parameter vector.

    :param reduced_params: The reduced parameter vector.
    :param body: The robot body.
    :param external_weight_bounds: Bounds for external coupling weights.
    :returns: Full parameter vector.
    """
    structure, mapping, result, generator = generate_blf_cpg_network(body, use_symmetry=True)
    info = generator.get_reduced_parameter_info(
        num_connections=structure.num_connections,
        external_weight_bounds=external_weight_bounds,
    )
    return generator.expand_reduced_params(reduced_params, info)


def generate_blf_cpg_network(
    body: Body,
    use_symmetry: bool = False,
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]], BLFResult, BLFCpgNetworkGenerator]:
    """
    Generate a BLF-based reduced CPG network for a robot.

    :param body: The robot body.
    :param use_symmetry: Whether to use symmetry constraints (BLF-SYM).
    :returns: Tuple of (CPG structure, hinge mapping, BLF result, generator).
    """
    result = analyze_robot(body)
    generator = BLFCpgNetworkGenerator(result, use_symmetry=use_symmetry)
    structure, mapping = generator.generate()
    return structure, mapping, result, generator


def print_blf_analysis(result: BLFResult) -> None:
    """
    Print a human-readable summary of BLF analysis.

    :param result: The BLF result to print.
    """
    print("=" * 50)
    print("BLF Analysis Result")
    print("=" * 50)

    print(f"\nTotal modules: {len(result.nodes)}")
    print(f"Body modules: {len(result.body_nodes)}")
    print(f"Number of limbs: {len(result.limbs)}")
    print(f"Total ActiveHinges: {len(result.active_hinge_nodes)}")

    print("\n--- Body Nodes ---")
    for idx in result.body_nodes:
        node = result.nodes[idx]
        module_type = type(node.module).__name__
        joint = node.joint_type.name if node.joint_type != JointType.UNCLASSIFIED else "-"
        print(f"  [{idx}] {module_type} (joint: {joint})")

    print("\n--- Limbs ---")
    for i, limb in enumerate(result.limbs):
        print(f"\n  Limb {i}:")
        for idx in limb:
            node = result.nodes[idx]
            module_type = type(node.module).__name__
            joint = (
                node.joint_type.name if node.joint_type != JointType.UNCLASSIFIED else "-"
            )
            dist = node.distance_from_body
            print(f"    [{idx}] {module_type} (dist={dist}, joint={joint})")

    print("\n--- Joint Classification ---")
    for jtype in [JointType.SPINE, JointType.HIP, JointType.KNEE]:
        joints = [idx for idx, jt in result.articulations.items() if jt == jtype]
        print(f"  {jtype.name}: {joints}")

    print("\n--- Symmetric Limb Groups ---")
    if result.symmetric_groups:
        for i, group in enumerate(result.symmetric_groups):
            print(f"  Group {i}: Limbs {group}")
    else:
        print("  No symmetric limbs found")

    print("=" * 50)


def compare_network_sizes(body: Body) -> dict:
    """
    Compare parameter counts between full neighbor network, BLF, and BLF-SYM.

    :param body: The robot body.
    :returns: Dictionary with comparison stats.
    """
    from revolve2.modular_robot.brain.cpg._make_cpg_network_structure_neighbor import (
        active_hinges_to_cpg_network_structure_neighbor,
    )

    # Get all active hinges
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Full neighbor network
    full_structure, _ = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    full_params = full_structure.num_connections

    # BLF network (without symmetry)
    blf_structure, _, result, gen = generate_blf_cpg_network(body, use_symmetry=False)
    blf_params = blf_structure.num_connections

    # BLF-SYM network (with symmetry)
    blf_sym_structure, _, result_sym, gen_sym = generate_blf_cpg_network(body, use_symmetry=True)
    blf_sym_unique_params = gen_sym.get_num_unique_params()
    # Total BLF-SYM params = unique amplitude/offset + all phase connections
    blf_sym_params = blf_sym_unique_params + len(blf_sym_structure.connections)

    return {
        "num_hinges": len(active_hinges),
        "full_cpgs": full_structure.num_cpgs,
        "full_connections": len(full_structure.connections),
        "full_params": full_params,
        "blf_cpgs": blf_structure.num_cpgs,
        "blf_connections": len(blf_structure.connections),
        "blf_params": blf_params,
        "blf_sym_params": blf_sym_params,
        "blf_sym_unique_amp_off": blf_sym_unique_params,
        "num_symmetric_groups": len(result_sym.symmetric_groups),
        "num_limbs": len(result.limbs),
        "joint_counts": {
            "spine": len([i for i, j in result.articulations.items() if j == JointType.SPINE]),
            "hip": len([i for i, j in result.articulations.items() if j == JointType.HIP]),
            "knee": len([i for i, j in result.articulations.items() if j == JointType.KNEE]),
        },
    }


def print_comparison_table(robots: dict[str, Body]) -> None:
    """
    Print a comparison table of full vs BLF vs BLF-SYM networks.

    :param robots: Dictionary of robot name -> Body.
    """
    print("\n" + "=" * 95)
    print("Network Size Comparison: Full Neighbor vs BLF vs BLF-SYM")
    print("=" * 95)
    print(f"{'Robot':<12} {'Hinges':>6} {'Full':>8} {'BLF':>8} {'BLF-SYM':>8} {'SymGrps':>7} {'Limbs':>6} {'S/H/K':>10}")
    print("-" * 95)

    for name, body in robots.items():
        stats = compare_network_sizes(body)
        jc = stats["joint_counts"]
        shk = f"{jc['spine']}/{jc['hip']}/{jc['knee']}"
        print(
            f"{name:<12} {stats['num_hinges']:>6} "
            f"{stats['full_params']:>8} {stats['blf_params']:>8} "
            f"{stats['blf_sym_params']:>8} {stats['num_symmetric_groups']:>7} "
            f"{stats['num_limbs']:>6} {shk:>10}"
        )

    print("=" * 95)
    print("S/H/K = Spine/Hip/Knee joint counts | SymGrps = Symmetric limb groups")


# Test function
if __name__ == "__main__":
    from revolve2.standards.modular_robots_v1 import (
        spider_v1,
        gecko_v1,
        tripod_v1,
        ant_v1,
        salamander_v1,
        snake_v1,
    )

    # Test on multiple robots
    robots = {
        "spider": spider_v1(),
        "gecko": gecko_v1(),
        "tripod": tripod_v1(),
        "ant": ant_v1(),
        "salamander": salamander_v1(),
        "snake": snake_v1(),
    }

    # Print comparison table
    print_comparison_table(robots)

    # Detailed analysis for spider
    print("\n" + "=" * 60)
    print("Detailed BLF Analysis: Spider")
    print("=" * 60)
    result = analyze_robot(robots["spider"])
    print_blf_analysis(result)

    # Show BLF-SYM details
    structure, mapping, _, gen = generate_blf_cpg_network(robots["spider"], use_symmetry=True)
    print(f"\nCPG Network: {structure.num_cpgs} CPGs, {len(structure.connections)} connections")

    print("\nCPG Joint Types:")
    for i in range(len(mapping)):
        node_idx = gen._cpg_to_node.get(i)
        jtype = result.nodes[node_idx].joint_type.name if node_idx else "?"
        print(f"  CPG {i}: {jtype}")

    print("\nSymmetric CPG Groups (share amplitude/offset):")
    for i, group in enumerate(gen.get_symmetric_groups()):
        print(f"  Group {i}: CPGs {group}")

    print(f"\nUnique amplitude/offset params: {gen.get_num_unique_params()}")
    print(f"Total BLF-SYM params: {gen.get_num_unique_params() + len(structure.connections)}")

    # Detailed analysis for gecko
    print("\n" + "=" * 60)
    print("Detailed BLF Analysis: Gecko")
    print("=" * 60)
    result = analyze_robot(robots["gecko"])
    print_blf_analysis(result)
