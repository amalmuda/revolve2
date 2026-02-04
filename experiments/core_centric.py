"""
Core-Centric Algorithm for Body/Limb Detection in Revolve2 Modular Robots.

This is an alternative to the BLF algorithm from the Bonardi et al. paper,
designed specifically for Revolve2's tree-structured robots (no cycles).

Key differences from BLF:
- No bi-connected component detection (trees have no cycles)
- Body detection based on Core and Bricks with 3+ hinges
- Simpler tree-based path finding for spine detection

Supports 4 experimental conditions from Bonardi paper:
- FO (Fully Open): All joints equal, wide bounds [0, π], no symmetry
- CC (Core-Centric): Joint classification, tight bounds by type, no symmetry
- CC-SYM: Joint classification, tight bounds, with symmetry
- SYM: No classification (all equal), wide bounds [0, π], with symmetry
"""

from enum import Enum
from dataclasses import dataclass, field
import math
from typing import Optional

from revolve2.modular_robot.body.base import Body, Brick, ActiveHinge, Core
from revolve2.modular_robot.body._module import Module


class ExperimentalCondition(Enum):
    """
    Experimental conditions from Bonardi et al. paper.

    FO: Fully Open - all joints equal, wide bounds, no symmetry
    CC: Core-Centric - joint classification, tight bounds, no symmetry
    CC_SYM: Core-Centric + Symmetry - classification + symmetry
    SYM: Symmetry only - no classification, wide bounds, with symmetry
    """
    FO = "fully_open"
    CC = "core_centric"
    CC_SYM = "core_centric_symmetry"
    SYM = "symmetry_only"


class JointType(Enum):
    """Classification of joints in the robot structure."""
    SPINE = "spine"
    HIP = "hip"
    KNEE = "knee"
    ANKLE = "ankle"
    LOCKED = "locked"
    UNCLASSIFIED = "unclassified"


class OptimizationMode(Enum):
    """
    Optimization mode as described in Bonardi et al. paper.

    FULLY_OPEN: All joints have full amplitude range [0, π]
    BLF: Body/Limb Finder - constrained bounds based on joint type
    """
    FULLY_OPEN = "fully_open"
    BLF = "blf"


@dataclass
class AmplitudeBounds:
    """Amplitude bounds for each joint type (in radians)."""

    # BLF-style constrained bounds (from Bonardi paper Table I)
    BLF_BOUNDS = {
        JointType.SPINE: (0.0, 2 * math.pi / 3),  # 0 to 2π/3
        JointType.HIP: (0.0, math.pi / 2),         # 0 to π/2
        JointType.KNEE: (0.0, math.pi / 6),        # 0 to π/6
        JointType.ANKLE: (0.0, math.pi / 6),       # 0 to π/6
        JointType.LOCKED: (0.0, 0.0),              # Fixed at 0
        JointType.UNCLASSIFIED: (0.0, math.pi),    # Default
    }

    # Fully Open bounds - all joints same range
    FO_BOUNDS = (0.0, math.pi)  # [0, π] for all joints

    @classmethod
    def get_bounds(
        cls,
        joint_type: JointType,
        mode: OptimizationMode = OptimizationMode.BLF
    ) -> tuple[float, float]:
        """
        Get amplitude bounds for a joint type.

        :param joint_type: The joint type.
        :param mode: FULLY_OPEN or BLF mode.
        :returns: (min_amplitude, max_amplitude) in radians.
        """
        if mode == OptimizationMode.FULLY_OPEN:
            # All joints get the same full range [0, π]
            return cls.FO_BOUNDS
        else:
            # BLF mode: constrained based on joint type
            return cls.BLF_BOUNDS.get(joint_type, (0.0, math.pi))


@dataclass
class LimbSignature:
    """
    Signature of a limb for symmetry detection.

    Two limbs are symmetric if they have the same signature.
    """
    num_hinges: int
    module_types: tuple[str, ...]  # Sequence of module type names
    attachment_indices: tuple[int, ...]  # Attachment face indices used

    def __hash__(self):
        return hash((self.num_hinges, self.module_types, self.attachment_indices))

    def __eq__(self, other):
        if not isinstance(other, LimbSignature):
            return False
        return (self.num_hinges == other.num_hinges and
                self.module_types == other.module_types and
                self.attachment_indices == other.attachment_indices)


@dataclass
class Limb:
    """Represents a limb in the robot structure."""
    hinges: list[ActiveHinge] = field(default_factory=list)
    signature: Optional[LimbSignature] = None
    symmetry_group: int = -1  # Index of symmetry group (-1 = no symmetry)

    def __len__(self) -> int:
        return len(self.hinges)


@dataclass
class SymmetryGroup:
    """
    A group of symmetric limbs.

    Symmetric limbs share amplitude and offset parameters,
    but have independent phase offsets.
    """
    limbs: list[Limb] = field(default_factory=list)
    signature: Optional[LimbSignature] = None

    def __len__(self) -> int:
        return len(self.limbs)


@dataclass
class CoreCentricResult:
    """Result of the Core-Centric analysis."""
    body_modules: list[Module] = field(default_factory=list)
    spine_hinges: list[ActiveHinge] = field(default_factory=list)
    limbs: list[Limb] = field(default_factory=list)
    joint_types: dict[ActiveHinge, JointType] = field(default_factory=dict)
    all_hinges: list[ActiveHinge] = field(default_factory=list)
    symmetry_groups: list[SymmetryGroup] = field(default_factory=list)


class CoreCentricAnalyzer:
    """
    Core-Centric algorithm for body/limb detection.

    Algorithm:
    1. Body Module Detection:
       - Core is ALWAYS a body module
       - Any Brick with 3+ directly connected Hinges is also a body module

    2. Spine Detection:
       - Find all paths between pairs of body modules
       - Every Hinge on these paths is classified as "spine"

    3. Limb Classification:
       - A "limb" is a chain of non-spine Hinges starting from a body module
       - H[1] = hip, H[n] = ankle, H[(n+1)//2] = knee, others = locked
    """

    def __init__(self, body: Body):
        """
        Initialize the analyzer.

        :param body: The robot body to analyze.
        """
        self._body = body
        self._core = body.core

    def analyze(self) -> CoreCentricResult:
        """
        Run the Core-Centric analysis.

        :returns: The analysis result.
        """
        result = CoreCentricResult()

        # Get all hinges
        result.all_hinges = self._body.find_modules_of_type(ActiveHinge)

        # Step 1: Detect body modules
        result.body_modules = self._detect_body_modules()

        # Step 2: Detect spine hinges
        result.spine_hinges = self._detect_spine_hinges(result.body_modules)

        # Step 3: Detect and classify limbs
        result.limbs = self._detect_limbs(result.body_modules, result.spine_hinges)

        # Step 4: Classify all joints
        result.joint_types = self._classify_joints(
            result.spine_hinges,
            result.limbs,
            result.all_hinges
        )

        # Step 5: Detect symmetry groups
        result.symmetry_groups = self._detect_symmetry_groups(result.limbs)

        return result

    def _detect_body_modules(self) -> list[Module]:
        """
        Detect body modules.

        Rules:
        - Core is ALWAYS a body module
        - Any Brick with 3+ directly connected Hinges is also a body module

        :returns: List of body modules.
        """
        body_modules: list[Module] = [self._core]

        # Find all bricks
        bricks = self._body.find_modules_of_type(Brick)

        for brick in bricks:
            # Count directly connected hinges
            hinge_count = self._count_direct_hinges(brick)
            if hinge_count >= 3:
                body_modules.append(brick)

        return body_modules

    def _count_direct_hinges(self, module: Module) -> int:
        """
        Count how many ActiveHinges are directly connected to a module.

        "Directly connected" means:
        - Immediate children that are ActiveHinges
        - The parent if it's an ActiveHinge

        According to BLF rules: "Any brick with 3+ hinges connected → also body"
        This includes connections in ALL directions (parent + children).

        :param module: The module to check.
        :returns: Number of directly connected hinges (children + parent).
        """
        count = 0

        # Count child hinges
        for child in module.children.values():
            if isinstance(child, ActiveHinge):
                count += 1

        # Count parent hinge (if parent is a hinge)
        if module.parent is not None and isinstance(module.parent, ActiveHinge):
            count += 1

        return count

    def _detect_spine_hinges(self, body_modules: list[Module]) -> list[ActiveHinge]:
        """
        Detect spine hinges (hinges on paths between body modules).

        In a tree, there's exactly one path between any two nodes.

        :param body_modules: List of body modules.
        :returns: List of spine hinges.
        """
        if len(body_modules) < 2:
            return []

        spine_hinges: set[ActiveHinge] = set()

        # Find paths between all pairs of body modules
        for i, module1 in enumerate(body_modules):
            for module2 in body_modules[i+1:]:
                path = self._find_path(module1, module2)
                for node in path:
                    if isinstance(node, ActiveHinge):
                        spine_hinges.add(node)

        return list(spine_hinges)

    def _find_path(self, start: Module, end: Module) -> list[Module]:
        """
        Find the path between two modules in the tree.

        Uses LCA (Lowest Common Ancestor) approach.

        :param start: Start module.
        :param end: End module.
        :returns: List of modules on the path (excluding start and end).
        """
        # Get path from start to root
        path_to_root_start: list[Module] = []
        current = start
        while current is not None:
            path_to_root_start.append(current)
            current = current.parent

        # Get path from end to root
        path_to_root_end: list[Module] = []
        current = end
        while current is not None:
            path_to_root_end.append(current)
            current = current.parent

        # Find LCA (first common ancestor)
        start_set = set(id(m) for m in path_to_root_start)
        lca = None
        lca_index_end = 0
        for i, module in enumerate(path_to_root_end):
            if id(module) in start_set:
                lca = module
                lca_index_end = i
                break

        if lca is None:
            return []  # No common ancestor (shouldn't happen in valid tree)

        # Find LCA index in start path
        lca_index_start = 0
        for i, module in enumerate(path_to_root_start):
            if id(module) == id(lca):
                lca_index_start = i
                break

        # Build path: start -> LCA -> end (excluding start, end, and LCA)
        path: list[Module] = []

        # Add nodes from start to LCA (excluding start and LCA)
        for i in range(1, lca_index_start):
            path.append(path_to_root_start[i])

        # Add LCA if it's a hinge (it connects the two sides)
        if isinstance(lca, ActiveHinge):
            path.append(lca)

        # Add nodes from LCA to end (excluding LCA and end), in reverse
        for i in range(lca_index_end - 1, 0, -1):
            path.append(path_to_root_end[i])

        return path

    def _detect_limbs(
        self,
        body_modules: list[Module],
        spine_hinges: list[ActiveHinge]
    ) -> list[Limb]:
        """
        Detect limbs (chains of non-spine hinges extending from body modules).

        This traces limbs from:
        1. Body modules (direct hinge children that aren't spine)
        2. Any module in the tree that has multiple hinge branches

        :param body_modules: List of body modules.
        :param spine_hinges: List of spine hinges.
        :returns: List of limbs.
        """
        spine_set = set(id(h) for h in spine_hinges)
        body_set = set(id(m) for m in body_modules)
        limbs: list[Limb] = []
        visited_hinges: set[int] = set()

        # For each body module, find limbs extending from it
        for body_module in body_modules:
            # Check all children
            for child in body_module.children.values():
                if isinstance(child, ActiveHinge) and id(child) not in spine_set:
                    # This is the start of a limb
                    limb = self._trace_limb(child, spine_set, body_set, visited_hinges)
                    if len(limb.hinges) > 0:
                        limbs.append(limb)

        # Also find limbs branching off from non-body modules (e.g., bricks in the middle)
        # This handles complex structures like salamander where branches occur at bricks
        self._find_branch_limbs(self._core, spine_set, body_set, visited_hinges, limbs)

        return limbs

    def _find_branch_limbs(
        self,
        module: Module,
        spine_set: set[int],
        body_set: set[int],
        visited: set[int],
        limbs: list[Limb]
    ) -> None:
        """
        Recursively find limbs that branch off from modules in the tree.

        :param module: Current module being examined.
        :param spine_set: Set of spine hinge IDs.
        :param body_set: Set of body module IDs.
        :param visited: Set of visited hinge IDs.
        :param limbs: List to append found limbs to.
        """
        # Get hinge children that aren't spine and haven't been visited
        hinge_children = [
            c for c in module.children.values()
            if isinstance(c, ActiveHinge)
            and id(c) not in spine_set
            and id(c) not in visited
        ]

        # If this module has multiple unvisited non-spine hinge children,
        # each one starts a new limb
        for hinge in hinge_children:
            limb = self._trace_limb(hinge, spine_set, body_set, visited)
            if len(limb.hinges) > 0:
                limbs.append(limb)

        # Recurse into all children
        for child in module.children.values():
            self._find_branch_limbs(child, spine_set, body_set, visited, limbs)

    def _trace_limb(
        self,
        start_hinge: ActiveHinge,
        spine_set: set[int],
        body_set: set[int],
        visited: set[int]
    ) -> Limb:
        """
        Trace a limb from a starting hinge going outward (away from body).

        This follows a single chain until it hits a branch, spine, or body module.

        :param start_hinge: The first hinge in the limb.
        :param spine_set: Set of spine hinge IDs.
        :param body_set: Set of body module IDs.
        :param visited: Set of visited hinge IDs.
        :returns: The traced limb.
        """
        limb = Limb()
        current: Module | None = start_hinge

        while current is not None:
            if isinstance(current, ActiveHinge):
                if id(current) in spine_set:
                    break  # Hit spine, stop
                if id(current) in visited:
                    break  # Already visited
                visited.add(id(current))
                limb.hinges.append(current)
            elif id(current) in body_set:
                break  # Hit another body module, stop

            # Move to next module - follow single chain
            # If there are multiple hinge children, stop (branch point)
            next_module = None
            hinge_children = [c for c in current.children.values() if isinstance(c, ActiveHinge)]
            non_hinge_children = [c for c in current.children.values() if not isinstance(c, ActiveHinge)]

            if len(hinge_children) == 1:
                # Single hinge child - continue the limb
                next_module = hinge_children[0]
            elif len(hinge_children) == 0 and len(non_hinge_children) == 1:
                # Single non-hinge child (brick) - continue through it
                next_module = non_hinge_children[0]
            elif len(hinge_children) == 0 and len(non_hinge_children) == 0:
                # No children - end of limb
                break
            else:
                # Multiple children or complex branching - stop this limb
                # The branches will be traced separately
                break

            current = next_module

        return limb

    def _classify_joints(
        self,
        spine_hinges: list[ActiveHinge],
        limbs: list[Limb],
        all_hinges: list[ActiveHinge]
    ) -> dict[ActiveHinge, JointType]:
        """
        Classify all joints.

        Spine hinges -> SPINE
        Limb hinges -> based on position:
            n=1: H1=hip
            n=2: H1=hip, H2=ankle
            n=3: H1=hip, H2=knee, H3=ankle
            n=4: H1=hip, H2=knee, H3=locked, H4=ankle
            etc.

        :param spine_hinges: List of spine hinges.
        :param limbs: List of limbs.
        :param all_hinges: All hinges in the robot.
        :returns: Dictionary mapping hinges to joint types.
        """
        joint_types: dict[ActiveHinge, JointType] = {}

        # Classify spine hinges
        for hinge in spine_hinges:
            joint_types[hinge] = JointType.SPINE

        # Classify limb hinges
        for limb in limbs:
            n = len(limb.hinges)
            if n == 0:
                continue

            classifications = self._get_limb_classifications(n)
            for i, hinge in enumerate(limb.hinges):
                if hinge not in joint_types:  # Don't override spine
                    joint_types[hinge] = classifications[i]

        # Mark any remaining hinges as unclassified
        for hinge in all_hinges:
            if hinge not in joint_types:
                joint_types[hinge] = JointType.UNCLASSIFIED

        return joint_types

    def _get_limb_classifications(self, n: int) -> list[JointType]:
        """
        Get joint classifications for a limb of length n.

        | n | H1 | H2 | H3 | H4 | H5 | H6 | H7 |
        |---|----|----|----|----|----|----|---|
        | 1 | hip | | | | | | |
        | 2 | hip | ankle | | | | | |
        | 3 | hip | knee | ankle | | | | |
        | 4 | hip | knee | LOCK | ankle | | | |
        | 5 | hip | LOCK | knee | LOCK | ankle | | |
        | 6 | hip | LOCK | knee | LOCK | LOCK | ankle | |
        | 7 | hip | LOCK | LOCK | knee | LOCK | LOCK | ankle |

        :param n: Number of hinges in the limb.
        :returns: List of joint types for each position.
        """
        if n <= 0:
            return []

        classifications = [JointType.LOCKED] * n

        # H[0] is always hip (index 0)
        classifications[0] = JointType.HIP

        if n >= 2:
            # H[n-1] is always ankle (last index)
            classifications[n - 1] = JointType.ANKLE

        if n >= 3:
            # H[(n+1)//2 - 1] is knee (middle, 0-indexed)
            # For n=3: (3+1)//2 - 1 = 1 (H2)
            # For n=4: (4+1)//2 - 1 = 1 (H2)
            # For n=5: (5+1)//2 - 1 = 2 (H3)
            # For n=6: (6+1)//2 - 1 = 2 (H3)
            # For n=7: (7+1)//2 - 1 = 3 (H4)
            knee_index = (n + 1) // 2 - 1
            classifications[knee_index] = JointType.KNEE

        return classifications

    def _compute_limb_signature(self, limb: Limb) -> LimbSignature:
        """
        Compute a signature for a limb that can be used for symmetry detection.

        Two limbs are symmetric if they have the same signature:
        - Same number of hinges
        - Same sequence of module types along the limb
        - Same INTERNAL attachment faces (excludes attachment to body module)

        For symmetry detection, we DON'T include the first attachment index
        (how the limb attaches to the body module) because that differs
        based on which side of the body the limb is on. What matters is
        the internal structure of the limb itself.

        :param limb: The limb to compute signature for.
        :returns: The limb signature.
        """
        if len(limb.hinges) == 0:
            return LimbSignature(0, (), ())

        module_types: list[str] = []
        attachment_indices: list[int] = []

        # Traverse the limb and record module types and INTERNAL attachment indices
        for i, hinge in enumerate(limb.hinges):
            module_types.append(type(hinge).__name__)

            # For the first hinge, DON'T record parent attachment (varies by position)
            # Only record internal attachments within the limb
            if i > 0:
                # Record the attachment index used to connect this hinge
                if hinge.parent is not None:
                    # Find which slot of parent this hinge is attached to
                    for slot_idx, child in hinge.parent.children.items():
                        if child is hinge:
                            attachment_indices.append(slot_idx)
                            break
                    else:
                        attachment_indices.append(-1)  # Not found (shouldn't happen)

            # Also traverse through non-hinge children (bricks) between hinges
            if i < len(limb.hinges) - 1:
                # Find intermediate modules between this hinge and next
                current = hinge
                next_hinge = limb.hinges[i + 1]

                # Traverse from current to next_hinge
                found_direct = False
                while current is not None and not found_direct:
                    found_path = False
                    for slot_idx, child in current.children.items():
                        if child is next_hinge:
                            # Direct connection - done with this segment
                            found_direct = True
                            break
                        elif isinstance(child, Brick):
                            # Check if next_hinge is descendant of this brick
                            if self._is_descendant(child, next_hinge):
                                module_types.append(type(child).__name__)
                                attachment_indices.append(slot_idx)
                                current = child
                                found_path = True
                                break
                    if not found_path and not found_direct:
                        break  # No path found

        return LimbSignature(
            num_hinges=len(limb.hinges),
            module_types=tuple(module_types),
            attachment_indices=tuple(attachment_indices)
        )

    def _is_descendant(self, ancestor: Module, target: Module) -> bool:
        """Check if target is a descendant of ancestor."""
        if ancestor is target:
            return True
        for child in ancestor.children.values():
            if self._is_descendant(child, target):
                return True
        return False

    def _detect_symmetry_groups(self, limbs: list[Limb]) -> list[SymmetryGroup]:
        """
        Detect symmetric limbs and group them.

        Two limbs are SYMMETRIC if:
        - Same number of hinges
        - Same sequence of module types along the limb
        - Same attachment faces used at each connection

        For symmetric limbs:
        - SHARE: amplitude and offset values
        - INDEPENDENT: phase offsets (to allow different gait patterns)

        :param limbs: List of limbs to analyze.
        :returns: List of symmetry groups.
        """
        if not limbs:
            return []

        # Compute signatures for all limbs
        for limb in limbs:
            limb.signature = self._compute_limb_signature(limb)

        # Group limbs by signature
        signature_to_limbs: dict[LimbSignature, list[Limb]] = {}
        for limb in limbs:
            if limb.signature not in signature_to_limbs:
                signature_to_limbs[limb.signature] = []
            signature_to_limbs[limb.signature].append(limb)

        # Create symmetry groups
        symmetry_groups: list[SymmetryGroup] = []
        group_idx = 0

        for signature, group_limbs in signature_to_limbs.items():
            if len(group_limbs) >= 2:
                # Multiple limbs with same signature -> symmetric group
                group = SymmetryGroup(limbs=group_limbs, signature=signature)
                for limb in group_limbs:
                    limb.symmetry_group = group_idx
                symmetry_groups.append(group)
                group_idx += 1
            else:
                # Single limb - not symmetric, but still track it
                group = SymmetryGroup(limbs=group_limbs, signature=signature)
                group_limbs[0].symmetry_group = group_idx
                symmetry_groups.append(group)
                group_idx += 1

        return symmetry_groups


def analyze_robot(body: Body) -> CoreCentricResult:
    """
    Convenience function to analyze a robot body.

    :param body: The robot body to analyze.
    :returns: The analysis result.
    """
    analyzer = CoreCentricAnalyzer(body)
    return analyzer.analyze()


def print_analysis(result: CoreCentricResult) -> None:
    """
    Print a human-readable summary of the analysis.

    :param result: The analysis result.
    """
    print("=" * 60)
    print("Core-Centric Analysis Result")
    print("=" * 60)

    print(f"\nBody modules: {len(result.body_modules)}")
    for i, module in enumerate(result.body_modules):
        print(f"  [{i}] {type(module).__name__}")

    print(f"\nSpine hinges: {len(result.spine_hinges)}")

    print(f"\nLimbs: {len(result.limbs)}")
    for i, limb in enumerate(result.limbs):
        sym_str = f" (symmetry group {limb.symmetry_group})" if limb.symmetry_group >= 0 else ""
        print(f"  Limb {i}: {len(limb)} hinges{sym_str}")

    print(f"\nSymmetry groups: {len(result.symmetry_groups)}")
    for i, group in enumerate(result.symmetry_groups):
        limb_indices = [result.limbs.index(l) for l in group.limbs]
        print(f"  Group {i}: {len(group)} limbs (limbs {limb_indices})")

    print(f"\nJoint classification summary:")
    counts = {}
    for jtype in JointType:
        counts[jtype] = sum(1 for jt in result.joint_types.values() if jt == jtype)
        if counts[jtype] > 0:
            bounds = AmplitudeBounds.get_bounds(jtype)
            print(f"  {jtype.name}: {counts[jtype]} (amplitude: {bounds[0]:.2f} to {bounds[1]:.2f} rad)")

    print("=" * 60)


# ============================================================================
# Parameter Count Functions for All 4 Conditions
# ============================================================================

def get_condition_parameter_info(
    result: CoreCentricResult,
    condition: ExperimentalCondition = ExperimentalCondition.CC,
) -> dict:
    """
    Get parameter count and info for a given experimental condition.

    :param result: The Core-Centric analysis result.
    :param condition: The experimental condition.
    :returns: Dictionary with parameter info.
    """
    n_hinges = len(result.all_hinges)
    n_limbs = len(result.limbs)
    n_sym_groups = len(result.symmetry_groups)

    # Count hinges by type (for CC and CC-SYM)
    type_counts = {jt: 0 for jt in JointType}
    for jt in result.joint_types.values():
        type_counts[jt] += 1

    # Count actuated hinges (exclude LOCKED)
    n_actuated = n_hinges - type_counts[JointType.LOCKED]

    info = {
        "condition": condition.value,
        "total_hinges": n_hinges,
        "total_limbs": n_limbs,
        "symmetry_groups": n_sym_groups,
    }

    if condition == ExperimentalCondition.FO:
        # Fully Open: all hinges, no classification, no symmetry
        # Each hinge has amplitude and offset
        # Couplings: neighbor-based (approximate)
        n_couplings = max(0, n_hinges - 1)  # Chain approximation
        info.update({
            "amplitude_params": n_hinges,
            "offset_params": n_hinges,
            "coupling_params": n_couplings,
            "total_params": 2 * n_hinges + n_couplings,
            "bounds": "wide [0, π]",
            "symmetry_used": False,
        })

    elif condition == ExperimentalCondition.CC:
        # Core-Centric: classification, tight bounds, no symmetry
        # Only actuated joints (no LOCKED)
        # Couplings: bio-inspired network
        n_spine = type_counts[JointType.SPINE]
        n_hip = type_counts[JointType.HIP]
        n_knee = type_counts[JointType.KNEE]
        n_ankle = type_counts[JointType.ANKLE]

        # Spine-spine + hip-hip + hip-spine + knee-hip + ankle-knee couplings
        n_spine_couplings = n_spine * (n_spine - 1) // 2
        n_hip_couplings = n_hip * (n_hip - 1) // 2
        n_hip_spine_couplings = n_hip if n_spine > 0 else 0
        n_knee_hip = n_knee  # Each knee couples to its hip
        n_ankle_knee = n_ankle  # Each ankle couples to its knee (or hip)

        n_couplings = (n_spine_couplings + n_hip_couplings +
                       n_hip_spine_couplings + n_knee_hip + n_ankle_knee)

        info.update({
            "amplitude_params": n_actuated,
            "offset_params": n_actuated,
            "coupling_params": n_couplings,
            "total_params": 2 * n_actuated + n_couplings,
            "bounds": "tight (by joint type)",
            "symmetry_used": False,
            "spine_count": n_spine,
            "hip_count": n_hip,
            "knee_count": n_knee,
            "ankle_count": n_ankle,
            "locked_count": type_counts[JointType.LOCKED],
        })

    elif condition == ExperimentalCondition.CC_SYM:
        # Core-Centric + Symmetry: classification + symmetry
        # Symmetric limbs share amplitude/offset, independent phase

        # Count unique parameters needed
        # Spine: all independent (usually 0-1 for most robots)
        n_spine = type_counts[JointType.SPINE]

        # For limbs: one set of (amp, offset) per symmetric group
        # Count actuated joints per limb type in each symmetry group
        unique_amplitude_params = n_spine  # Spine joints
        unique_offset_params = n_spine

        for group in result.symmetry_groups:
            if len(group.limbs) > 0:
                # Get one representative limb
                rep_limb = group.limbs[0]
                for hinge in rep_limb.hinges:
                    jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
                    if jtype != JointType.LOCKED:
                        unique_amplitude_params += 1
                        unique_offset_params += 1

        # Couplings: same structure as CC but may be fewer with symmetry
        # However, phase offsets remain independent for each hinge
        # For now, approximate couplings similar to CC
        n_spine_couplings = n_spine * (n_spine - 1) // 2
        n_hip = type_counts[JointType.HIP]
        n_hip_couplings = n_hip * (n_hip - 1) // 2
        n_hip_spine_couplings = n_hip if n_spine > 0 else 0
        n_knee = type_counts[JointType.KNEE]
        n_ankle = type_counts[JointType.ANKLE]
        n_knee_hip = n_knee
        n_ankle_knee = n_ankle

        n_couplings = (n_spine_couplings + n_hip_couplings +
                       n_hip_spine_couplings + n_knee_hip + n_ankle_knee)

        info.update({
            "amplitude_params": unique_amplitude_params,
            "offset_params": unique_offset_params,
            "coupling_params": n_couplings,
            "total_params": unique_amplitude_params + unique_offset_params + n_couplings,
            "bounds": "tight (by joint type)",
            "symmetry_used": True,
            "spine_count": n_spine,
            "hip_count": n_hip,
            "knee_count": n_knee,
            "ankle_count": n_ankle,
        })

    elif condition == ExperimentalCondition.SYM:
        # Symmetry only: no classification, wide bounds, with symmetry
        # All hinges in symmetric limbs share params

        # Count unique hinges needed
        unique_hinges = len(result.spine_hinges)  # Spine hinges independent

        for group in result.symmetry_groups:
            if len(group.limbs) > 0:
                # One set of params per position in the limb
                unique_hinges += len(group.limbs[0].hinges)

        n_couplings = max(0, n_hinges - 1)  # Full network

        info.update({
            "amplitude_params": unique_hinges,
            "offset_params": unique_hinges,
            "coupling_params": n_couplings,
            "total_params": 2 * unique_hinges + n_couplings,
            "bounds": "wide [0, π]",
            "symmetry_used": True,
        })

    return info


def print_condition_comparison(result: CoreCentricResult) -> None:
    """Print parameter counts for all 4 experimental conditions."""
    print("\n" + "=" * 70)
    print("PARAMETER COMPARISON - ALL 4 CONDITIONS")
    print("=" * 70)

    for condition in ExperimentalCondition:
        info = get_condition_parameter_info(result, condition)
        print(f"\n{condition.name} ({info['bounds']})")
        print(f"  Symmetry: {'Yes' if info['symmetry_used'] else 'No'}")
        print(f"  Amplitude params: {info['amplitude_params']}")
        print(f"  Offset params: {info['offset_params']}")
        print(f"  Coupling params: {info['coupling_params']}")
        print(f"  TOTAL: {info['total_params']}")

    print("\n" + "=" * 70)


# ============================================================================
# Symmetry Expansion Functions
# ============================================================================

def get_cc_sym_expansion_info(body: Body) -> dict:
    """
    Get information needed to expand CC-SYM reduced parameters to full parameters.

    This is used during evolution:
    1. CMA-ES optimizes reduced (symmetric) parameters
    2. Before simulation, expand to full parameters using this mapping

    The Revolve2 CPG structure has:
    - One internal weight per CPG (not separate amplitude/offset)
    - External weights for coupling between CPGs

    For CC-SYM:
    - Symmetric limbs share internal weights
    - External weights (couplings) remain independent for gait flexibility

    :param body: The robot body.
    :returns: Dictionary with expansion info.
    """
    result = analyze_robot(body)

    # Get the CPG structure to know the number of connections
    structure, hinge_mapping, _ = generate_core_centric_cpg_structure(body)
    num_cpgs = structure.num_cpgs
    num_external = len(structure.connections)

    # Build mapping from unique internal param index to full internal param indices
    # internal_expansion_map[cpg_idx] = unique_internal_param_idx

    internal_expansion_map: list[int] = []  # For each CPG, which unique internal param to use
    hinge_order: list[ActiveHinge] = []  # Order of hinges matching CPG order
    unique_internal_bounds: list[tuple[float, float]] = []  # Bounds for each unique internal param

    unique_internal_idx = 0

    # Map hinges to their CPG indices
    hinge_to_cpg_idx: dict[ActiveHinge, int] = {}
    for cpg_idx, hinge in hinge_mapping:
        hinge_to_cpg_idx[hinge] = cpg_idx
        hinge_order.append(hinge)

    # Track which unique param each hinge uses
    hinge_to_unique: dict[ActiveHinge, int] = {}

    # First: spine hinges (each unique)
    for hinge in result.spine_hinges:
        jtype = result.joint_types.get(hinge, JointType.SPINE)
        if jtype != JointType.LOCKED and hinge in hinge_to_cpg_idx:
            hinge_to_unique[hinge] = unique_internal_idx
            bounds = AmplitudeBounds.get_bounds(jtype, OptimizationMode.BLF)
            unique_internal_bounds.append(bounds)
            unique_internal_idx += 1

    # Group representative hinge to unique param index mapping
    group_hinge_to_unique: dict[int, dict[int, int]] = {}  # group_idx -> {position -> unique_idx}

    # Then: limb hinges (shared within symmetry groups)
    for group_idx, group in enumerate(result.symmetry_groups):
        if len(group.limbs) == 0:
            continue

        # Use first limb as representative
        rep_limb = group.limbs[0]
        group_hinge_to_unique[group_idx] = {}

        # Assign unique params to representative limb positions
        for pos, hinge in enumerate(rep_limb.hinges):
            jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
            if jtype != JointType.LOCKED and hinge in hinge_to_cpg_idx:
                group_hinge_to_unique[group_idx][pos] = unique_internal_idx
                bounds = AmplitudeBounds.get_bounds(jtype, OptimizationMode.BLF)
                unique_internal_bounds.append(bounds)
                unique_internal_idx += 1

        # Map all limbs in group to these unique params
        for limb in group.limbs:
            for pos, hinge in enumerate(limb.hinges):
                jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
                if jtype != JointType.LOCKED and hinge in hinge_to_cpg_idx:
                    unique_param_idx = group_hinge_to_unique[group_idx].get(pos, -1)
                    hinge_to_unique[hinge] = unique_param_idx

    # Build the expansion map in CPG order
    for hinge in hinge_order:
        if hinge in hinge_to_unique:
            internal_expansion_map.append(hinge_to_unique[hinge])
        else:
            # Hinge not in any group - shouldn't happen but handle gracefully
            internal_expansion_map.append(0)

    return {
        "num_unique_internal": unique_internal_idx,
        "num_external": num_external,
        "num_unique_params": unique_internal_idx + num_external,  # Total reduced params
        "internal_expansion_map": internal_expansion_map,
        "unique_internal_bounds": unique_internal_bounds,
        "hinge_order": hinge_order,
        "num_cpgs": num_cpgs,
        "result": result,
    }


def expand_cc_sym_params(
    reduced_params: list[float],
    body: Body,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> list[float]:
    """
    Expand reduced CC-SYM parameters to full parameters.

    The Revolve2 CPG structure expects: [internal_weights..., external_weights...]
    Where internal_weights has one value per CPG (not separate amp/off).

    In CC-SYM:
    - Symmetric limbs share internal weights
    - External weights (couplings) are independent

    :param reduced_params: The reduced parameter vector [unique_internal..., external...].
    :param body: The robot body.
    :param external_weight_bounds: Bounds for coupling weights (not used, kept for API compatibility).
    :returns: The full parameter vector [full_internal..., external...].
    """
    info = get_cc_sym_expansion_info(body)

    num_unique_internal = info["num_unique_internal"]
    internal_expansion_map = info["internal_expansion_map"]

    # Reduced params: [unique_internal..., external...]
    unique_internal = reduced_params[:num_unique_internal]
    external = reduced_params[num_unique_internal:]

    # Expand internal weights using the expansion map
    full_internal = [unique_internal[idx] for idx in internal_expansion_map]

    return full_internal + list(external)


def get_cc_sym_parameter_bounds(
    body: Body,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> tuple[list[float], list[float]]:
    """
    Get parameter bounds for CC-SYM optimization (reduced parameters).

    Returns bounds for: [unique_internal_weights..., external_weights...]
    Internal weights have joint-type specific bounds.
    External weights use uniform bounds.

    :param body: The robot body.
    :param external_weight_bounds: Bounds for coupling weights.
    :returns: Tuple of (lower_bounds, upper_bounds).
    """
    info = get_cc_sym_expansion_info(body)

    lower_bounds: list[float] = []
    upper_bounds: list[float] = []

    # Internal weight bounds (from unique_internal_bounds computed in expansion info)
    for lo, hi in info["unique_internal_bounds"]:
        lower_bounds.append(lo)
        upper_bounds.append(hi)

    # External weight bounds (uniform for all couplings)
    num_external = info["num_external"]
    for _ in range(num_external):
        lower_bounds.append(external_weight_bounds[0])
        upper_bounds.append(external_weight_bounds[1])

    return lower_bounds, upper_bounds


# ============================================================================
# CPG Network Generation (from Bonardi et al. paper)
# ============================================================================

@dataclass
class CPGOscillator:
    """
    Represents a single CPG oscillator controlling one joint.

    Each oscillator has:
    - amplitude: how far the joint moves
    - offset: the center position
    - phase connections to other oscillators
    """
    hinge: ActiveHinge
    joint_type: JointType
    index: int  # Unique index in the network
    limb_index: int | None = None  # Which limb this belongs to (None for spine)

    @property
    def amplitude_bounds(self) -> tuple[float, float]:
        """Get amplitude bounds based on joint type."""
        return AmplitudeBounds.get_bounds(self.joint_type, OptimizationMode.BLF)


@dataclass
class CPGCoupling:
    """Represents a coupling (connection) between two oscillators."""
    from_oscillator: int  # Index of source oscillator
    to_oscillator: int    # Index of target oscillator

    def __hash__(self):
        # Unordered pair - (a,b) == (b,a)
        return hash(frozenset([self.from_oscillator, self.to_oscillator]))

    def __eq__(self, other):
        if not isinstance(other, CPGCoupling):
            return False
        return {self.from_oscillator, self.to_oscillator} == {other.from_oscillator, other.to_oscillator}


@dataclass
class CPGNetwork:
    """
    Represents a complete CPG network for locomotion control.

    This is the reduced network topology from the Bonardi paper.
    """
    oscillators: list[CPGOscillator] = field(default_factory=list)
    couplings: set[CPGCoupling] = field(default_factory=set)

    # Grouped oscillators by type for easy access
    spine_oscillators: list[CPGOscillator] = field(default_factory=list)
    hip_oscillators: list[CPGOscillator] = field(default_factory=list)
    knee_oscillators: list[CPGOscillator] = field(default_factory=list)
    ankle_oscillators: list[CPGOscillator] = field(default_factory=list)

    # Mapping from hinge to oscillator
    hinge_to_oscillator: dict[ActiveHinge, CPGOscillator] = field(default_factory=dict)

    def get_parameter_count(self) -> dict[str, int]:
        """
        Get the number of parameters to optimize.

        Returns dict with:
        - amplitudes: number of amplitude parameters
        - offsets: number of offset parameters
        - phases: number of phase lag parameters
        - total: total parameter count
        """
        n_oscillators = len(self.oscillators)
        n_couplings = len(self.couplings)

        return {
            "amplitudes": n_oscillators,
            "offsets": n_oscillators,
            "phases": n_couplings,
            "total": 2 * n_oscillators + n_couplings
        }

    def get_fully_open_parameter_count(self, total_hinges: int) -> dict[str, int]:
        """
        Get parameter count for fully open network (for comparison).

        In FO, each hinge has an oscillator coupled to physical neighbors.
        Approximation: each joint coupled to ~1.5 neighbors on average.
        """
        # FO: all hinges have oscillators
        n_oscillators = total_hinges
        # Approximate couplings (chain structure)
        n_couplings = total_hinges - 1  # Minimum for connected graph

        return {
            "amplitudes": n_oscillators,
            "offsets": n_oscillators,
            "phases": n_couplings,
            "total": 2 * n_oscillators + n_couplings
        }


def generate_cpg_network(
    result: CoreCentricResult,
    mode: OptimizationMode = OptimizationMode.BLF
) -> CPGNetwork:
    """
    Generate a CPG network from the Core-Centric analysis result.

    For BLF mode, follows Bonardi et al. paper:
    - Only actuated joints (spine, hip, knee, ankle) have oscillators
    - LOCKED joints have no oscillator
    - Coupling rules:
        - Spine oscillators: fully coupled to each other
        - Hip oscillators: fully coupled to each other + coupled to nearest spine
        - Knee oscillators: coupled only to their hip
        - Ankle oscillators: coupled only to their knee

    For FULLY_OPEN mode:
    - All joints have oscillators
    - Coupled to physical neighbors

    :param result: The Core-Centric analysis result.
    :param mode: FULLY_OPEN or BLF optimization mode.
    :returns: The generated CPG network.
    """
    network = CPGNetwork()
    oscillator_index = 0

    if mode == OptimizationMode.FULLY_OPEN:
        # Fully open: all hinges get oscillators, coupled to neighbors
        return _generate_fully_open_network(result)

    # BLF mode: reduced network

    # 1. Create oscillators for spine joints
    for hinge in result.spine_hinges:
        jtype = result.joint_types.get(hinge, JointType.SPINE)
        osc = CPGOscillator(
            hinge=hinge,
            joint_type=jtype,
            index=oscillator_index,
            limb_index=None
        )
        network.oscillators.append(osc)
        network.spine_oscillators.append(osc)
        network.hinge_to_oscillator[hinge] = osc
        oscillator_index += 1

    # 2. Create oscillators for limb joints (only actuated: hip, knee, ankle)
    for limb_idx, limb in enumerate(result.limbs):
        limb_hip = None
        limb_knee = None

        for hinge in limb.hinges:
            jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)

            # Skip LOCKED joints - they don't get oscillators
            if jtype == JointType.LOCKED:
                continue

            osc = CPGOscillator(
                hinge=hinge,
                joint_type=jtype,
                index=oscillator_index,
                limb_index=limb_idx
            )
            network.oscillators.append(osc)
            network.hinge_to_oscillator[hinge] = osc

            if jtype == JointType.HIP:
                network.hip_oscillators.append(osc)
                limb_hip = osc
            elif jtype == JointType.KNEE:
                network.knee_oscillators.append(osc)
                limb_knee = osc
            elif jtype == JointType.ANKLE:
                network.ankle_oscillators.append(osc)
                # Ankle couples to knee (or hip if no knee)
                if limb_knee is not None:
                    network.couplings.add(CPGCoupling(osc.index, limb_knee.index))
                elif limb_hip is not None:
                    network.couplings.add(CPGCoupling(osc.index, limb_hip.index))

            oscillator_index += 1

        # Knee couples to hip
        if limb_knee is not None and limb_hip is not None:
            network.couplings.add(CPGCoupling(limb_knee.index, limb_hip.index))

    # 3. Spine oscillators are fully coupled
    for i, osc1 in enumerate(network.spine_oscillators):
        for osc2 in network.spine_oscillators[i+1:]:
            network.couplings.add(CPGCoupling(osc1.index, osc2.index))

    # 4. Hip oscillators are fully coupled
    for i, osc1 in enumerate(network.hip_oscillators):
        for osc2 in network.hip_oscillators[i+1:]:
            network.couplings.add(CPGCoupling(osc1.index, osc2.index))

    # 5. Hip oscillators couple to nearest spine
    if network.spine_oscillators:
        # For simplicity, couple all hips to the first spine oscillator
        # (In more complex robots, would find the nearest spine)
        spine_osc = network.spine_oscillators[0]
        for hip_osc in network.hip_oscillators:
            network.couplings.add(CPGCoupling(hip_osc.index, spine_osc.index))

    return network


def _generate_fully_open_network(result: CoreCentricResult) -> CPGNetwork:
    """Generate a fully open CPG network where all joints have oscillators."""
    network = CPGNetwork()

    # All hinges get oscillators with UNCLASSIFIED type (full range)
    for idx, hinge in enumerate(result.all_hinges):
        osc = CPGOscillator(
            hinge=hinge,
            joint_type=JointType.UNCLASSIFIED,  # Full range [0, π]
            index=idx,
            limb_index=None
        )
        network.oscillators.append(osc)
        network.hinge_to_oscillator[hinge] = osc

    # Couple to physical neighbors (parent-child relationships)
    for hinge in result.all_hinges:
        osc = network.hinge_to_oscillator[hinge]

        # Check children
        for child in hinge.children.values():
            if isinstance(child, ActiveHinge) and child in network.hinge_to_oscillator:
                child_osc = network.hinge_to_oscillator[child]
                network.couplings.add(CPGCoupling(osc.index, child_osc.index))
            # Check if child has hinge children
            if hasattr(child, 'children'):
                for grandchild in child.children.values():
                    if isinstance(grandchild, ActiveHinge) and grandchild in network.hinge_to_oscillator:
                        grandchild_osc = network.hinge_to_oscillator[grandchild]
                        network.couplings.add(CPGCoupling(osc.index, grandchild_osc.index))

    return network


def print_cpg_network(network: CPGNetwork, mode: OptimizationMode = OptimizationMode.BLF) -> None:
    """Print a human-readable summary of the CPG network."""
    print("=" * 60)
    print(f"CPG Network ({mode.value} mode)")
    print("=" * 60)

    print(f"\nOscillators: {len(network.oscillators)}")
    print(f"  Spine: {len(network.spine_oscillators)}")
    print(f"  Hip: {len(network.hip_oscillators)}")
    print(f"  Knee: {len(network.knee_oscillators)}")
    print(f"  Ankle: {len(network.ankle_oscillators)}")

    print(f"\nCouplings: {len(network.couplings)}")

    params = network.get_parameter_count()
    print(f"\nParameters to optimize:")
    print(f"  Amplitudes: {params['amplitudes']}")
    print(f"  Offsets: {params['offsets']}")
    print(f"  Phase lags: {params['phases']}")
    print(f"  TOTAL: {params['total']}")

    print("\n--- Oscillator Details ---")
    for osc in network.oscillators:
        bounds = osc.amplitude_bounds
        limb_str = f"limb {osc.limb_index}" if osc.limb_index is not None else "spine"
        print(f"  [{osc.index}] {osc.joint_type.name:8} ({limb_str}) "
              f"amplitude: [0, {bounds[1]:.2f}]")

    print("\n--- Coupling Topology ---")
    for coupling in sorted(network.couplings, key=lambda c: (c.from_oscillator, c.to_oscillator)):
        osc1 = network.oscillators[coupling.from_oscillator]
        osc2 = network.oscillators[coupling.to_oscillator]
        print(f"  {osc1.joint_type.name}[{osc1.index}] <-> {osc2.joint_type.name}[{osc2.index}]")

    print("=" * 60)


# ============================================================================
# Revolve2 CPG Structure Integration
# ============================================================================

from revolve2.modular_robot.brain.cpg._cpg_network_structure import (
    Cpg,
    CpgNetworkStructure,
    CpgPair,
)


def generate_core_centric_cpg_structure(
    body: Body,
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]], "CoreCentricResult"]:
    """
    Generate a Revolve2-compatible CPG network structure using Core-Centric algorithm.

    This creates a reduced CPG topology following Bonardi et al. paper rules:
    - Spine oscillators: fully coupled
    - Hip oscillators: fully coupled + connected to nearest spine
    - Knee oscillators: coupled to their hip
    - Ankle oscillators: coupled to their knee (or hip if no knee)
    - LOCKED joints: excluded (no oscillator)

    :param body: The robot body.
    :returns: Tuple of (CPG structure, hinge mapping, analysis result).
    """
    # Run Core-Centric analysis
    result = analyze_robot(body)

    # Get classified hinges that should have CPGs
    classified_hinges: dict[JointType, list[ActiveHinge]] = {
        JointType.SPINE: [],
        JointType.HIP: [],
        JointType.KNEE: [],
        JointType.ANKLE: [],
    }

    for hinge in result.all_hinges:
        jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
        if jtype in classified_hinges:
            classified_hinges[jtype].append(hinge)

    # Create CPGs for each classified hinge (excluding LOCKED)
    cpg_list: list[Cpg] = []
    hinge_to_cpg: dict[ActiveHinge, Cpg] = {}
    active_hinges: list[ActiveHinge] = []

    for jtype in [JointType.SPINE, JointType.HIP, JointType.KNEE, JointType.ANKLE]:
        for hinge in classified_hinges[jtype]:
            cpg = Cpg(len(cpg_list))
            cpg_list.append(cpg)
            hinge_to_cpg[hinge] = cpg
            active_hinges.append(hinge)

    # Build connections based on bio-inspired rules
    connections: set[CpgPair] = set()

    # 1. Spine oscillators fully coupled
    spine_cpgs = [hinge_to_cpg[h] for h in classified_hinges[JointType.SPINE]]
    for i, cpg1 in enumerate(spine_cpgs):
        for cpg2 in spine_cpgs[i + 1:]:
            connections.add(CpgPair(cpg1, cpg2))

    # 2. Hip oscillators fully coupled
    hip_cpgs = [hinge_to_cpg[h] for h in classified_hinges[JointType.HIP]]
    for i, cpg1 in enumerate(hip_cpgs):
        for cpg2 in hip_cpgs[i + 1:]:
            connections.add(CpgPair(cpg1, cpg2))

    # 3. Hip connected to nearest spine
    if spine_cpgs:
        for hip_hinge in classified_hinges[JointType.HIP]:
            hip_cpg = hinge_to_cpg[hip_hinge]
            # Connect to first spine (simplification)
            connections.add(CpgPair(hip_cpg, spine_cpgs[0]))

    # 4. Knee connected to corresponding hip (same limb)
    for limb in result.limbs:
        limb_hip = None
        limb_knee = None
        limb_ankle = None

        for hinge in limb.hinges:
            jtype = result.joint_types.get(hinge)
            if jtype == JointType.HIP:
                limb_hip = hinge
            elif jtype == JointType.KNEE:
                limb_knee = hinge
            elif jtype == JointType.ANKLE:
                limb_ankle = hinge

        # Knee -> Hip coupling
        if limb_knee is not None and limb_hip is not None:
            connections.add(CpgPair(hinge_to_cpg[limb_knee], hinge_to_cpg[limb_hip]))

        # Ankle -> Knee (or Hip if no knee) coupling
        if limb_ankle is not None:
            if limb_knee is not None:
                connections.add(CpgPair(hinge_to_cpg[limb_ankle], hinge_to_cpg[limb_knee]))
            elif limb_hip is not None:
                connections.add(CpgPair(hinge_to_cpg[limb_ankle], hinge_to_cpg[limb_hip]))

    structure = CpgNetworkStructure(cpg_list, connections)

    # Build mapping (index, hinge) like other functions return
    mapping = [(cpg.index, active_hinges[cpg.index]) for cpg in cpg_list]

    return structure, mapping, result


def get_core_centric_parameter_bounds(
    body: Body,
    external_weight_bounds: tuple[float, float] = (-1.0, 1.0),
) -> tuple[list[float], list[float]]:
    """
    Get per-parameter bounds for Core-Centric CPG optimization.

    Uses joint-type specific amplitude bounds from Bonardi paper Table I:
    - SPINE: [0, 2π/3] rad
    - HIP: [0, π/2] rad
    - KNEE/ANKLE: [0, π/6] rad

    External coupling weights use uniform bounds.

    :param body: The robot body.
    :param external_weight_bounds: Bounds for external coupling weights.
    :returns: Tuple of (lower_bounds, upper_bounds) lists.
    """
    structure, mapping, result = generate_core_centric_cpg_structure(body)

    num_cpgs = structure.num_cpgs
    num_external = len(structure.connections)

    lower_bounds: list[float] = []
    upper_bounds: list[float] = []

    # Internal weights (amplitude) - joint-type specific
    for _, hinge in mapping:
        jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
        lo, hi = AmplitudeBounds.get_bounds(jtype, OptimizationMode.BLF)
        lower_bounds.append(lo)
        upper_bounds.append(hi)

    # External weights (coupling) - uniform bounds
    for _ in range(num_external):
        lower_bounds.append(external_weight_bounds[0])
        upper_bounds.append(external_weight_bounds[1])

    return lower_bounds, upper_bounds


def active_hinges_to_cpg_network_structure_core_centric(
    active_hinges: list[ActiveHinge],
    body: Body,
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create a CPG structure using Core-Centric coupling (compatible with contact_detection.py).

    This is a convenience wrapper that returns the same format as other
    active_hinges_to_cpg_network_structure_* functions.

    :param active_hinges: The active hinges (ignored, uses body instead).
    :param body: The robot body.
    :returns: Tuple of (CPG structure, output mapping).
    """
    structure, mapping, _ = generate_core_centric_cpg_structure(body)
    return structure, mapping


# ============================================================================
# Test Cases
# ============================================================================

def _create_test_spider() -> Body:
    """Create a simple spider: Core + 4 single-hinge legs."""
    from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1

    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)
    return body


def _create_test_spider_with_feet() -> Body:
    """Create spider with 2-hinge legs: Core + 4 legs (hip + ankle)."""
    from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1

    body = BodyV1()

    # Front leg
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.front.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)

    # Back leg
    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.back.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(0.0)

    # Left leg
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)

    # Right leg
    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)

    return body


def _create_test_gecko() -> Body:
    """Create gecko: Core + Brick with 3 hinges attached (becomes body module)."""
    from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1

    body = BodyV1()

    # Front legs from core
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)

    # Connect to brick (body extension)
    body.core_v1.right = ActiveHingeV1(0.0)  # This is spine
    body.core_v1.right.attachment = BrickV1(0.0)

    # Brick has 3 hinges attached -> becomes body module
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)  # spine continuation or limb
    body.core_v1.right.attachment.left = ActiveHingeV1(0.0)   # leg
    body.core_v1.right.attachment.right = ActiveHingeV1(0.0)  # leg

    return body


def _create_test_centipede() -> Body:
    """Create centipede: 3 body modules in chain with legs."""
    from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1

    body = BodyV1()

    # Core has front/back legs
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)

    # First spine segment to body brick 1
    body.core_v1.right = ActiveHingeV1(0.0)  # spine
    body.core_v1.right.attachment = BrickV1(0.0)

    # Body brick 1 has 3 hinges -> body module
    body.core_v1.right.attachment.left = ActiveHingeV1(0.0)   # leg
    body.core_v1.right.attachment.right = ActiveHingeV1(0.0)  # leg
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)  # spine to next body
    body.core_v1.right.attachment.front.attachment = BrickV1(0.0)

    # Body brick 2 has 3 hinges -> body module
    body.core_v1.right.attachment.front.attachment.left = ActiveHingeV1(0.0)   # leg
    body.core_v1.right.attachment.front.attachment.right = ActiveHingeV1(0.0)  # leg
    body.core_v1.right.attachment.front.attachment.front = ActiveHingeV1(0.0)  # tail

    return body


def _create_test_long_limb(n: int) -> Body:
    """Create robot with a single limb of n hinges."""
    from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1

    body = BodyV1()

    if n == 0:
        return body

    # Create chain of hinges
    current = ActiveHingeV1(0.0)
    body.core_v1.front = current

    for _ in range(n - 1):
        brick = BrickV1(0.0)
        current.attachment = brick
        next_hinge = ActiveHingeV1(0.0)
        brick.front = next_hinge
        current = next_hinge

    return body


def run_tests() -> None:
    """Run all test cases."""
    print("\n" + "=" * 70)
    print("CORE-CENTRIC ALGORITHM TEST SUITE")
    print("=" * 70)

    # Test 1: Simple spider
    print("\n--- Test 1: Simple Spider (4 single-hinge legs) ---")
    spider = _create_test_spider()
    result = analyze_robot(spider)
    print_analysis(result)
    assert len(result.body_modules) == 1, "Spider should have 1 body module (Core)"
    assert len(result.spine_hinges) == 0, "Spider should have 0 spine hinges"
    assert len(result.limbs) == 4, "Spider should have 4 limbs"
    hip_count = sum(1 for jt in result.joint_types.values() if jt == JointType.HIP)
    assert hip_count == 4, f"Spider should have 4 hips, got {hip_count}"
    print("✓ Test 1 PASSED")

    # Test 2: Spider with feet (2-hinge legs)
    print("\n--- Test 2: Spider with Feet (4 two-hinge legs) ---")
    spider_feet = _create_test_spider_with_feet()
    result = analyze_robot(spider_feet)
    print_analysis(result)
    assert len(result.body_modules) == 1, "Spider should have 1 body module"
    assert len(result.limbs) == 4, "Spider should have 4 limbs"
    hip_count = sum(1 for jt in result.joint_types.values() if jt == JointType.HIP)
    ankle_count = sum(1 for jt in result.joint_types.values() if jt == JointType.ANKLE)
    assert hip_count == 4, f"Should have 4 hips, got {hip_count}"
    assert ankle_count == 4, f"Should have 4 ankles, got {ankle_count}"
    print("✓ Test 2 PASSED")

    # Test 3: Gecko (body extension)
    print("\n--- Test 3: Gecko (Core + Brick with 3 hinges) ---")
    gecko = _create_test_gecko()
    result = analyze_robot(gecko)
    print_analysis(result)
    assert len(result.body_modules) == 2, f"Gecko should have 2 body modules, got {len(result.body_modules)}"
    spine_count = sum(1 for jt in result.joint_types.values() if jt == JointType.SPINE)
    assert spine_count >= 1, f"Gecko should have at least 1 spine, got {spine_count}"
    print("✓ Test 3 PASSED")

    # Test 4: Centipede (3 body modules)
    print("\n--- Test 4: Centipede (chain of body modules) ---")
    centipede = _create_test_centipede()
    result = analyze_robot(centipede)
    print_analysis(result)
    assert len(result.body_modules) == 3, f"Centipede should have 3 body modules, got {len(result.body_modules)}"
    spine_count = sum(1 for jt in result.joint_types.values() if jt == JointType.SPINE)
    assert spine_count >= 2, f"Centipede should have at least 2 spine hinges, got {spine_count}"
    print("✓ Test 4 PASSED")

    # Test 5-11: Long limbs (n=1 to n=7)
    print("\n--- Tests 5-11: Long Limbs (n=1 to n=7) ---")
    expected_classifications = {
        1: {JointType.HIP: 1},
        2: {JointType.HIP: 1, JointType.ANKLE: 1},
        3: {JointType.HIP: 1, JointType.KNEE: 1, JointType.ANKLE: 1},
        4: {JointType.HIP: 1, JointType.KNEE: 1, JointType.LOCKED: 1, JointType.ANKLE: 1},
        5: {JointType.HIP: 1, JointType.KNEE: 1, JointType.LOCKED: 2, JointType.ANKLE: 1},
        6: {JointType.HIP: 1, JointType.KNEE: 1, JointType.LOCKED: 3, JointType.ANKLE: 1},
        7: {JointType.HIP: 1, JointType.KNEE: 1, JointType.LOCKED: 4, JointType.ANKLE: 1},
    }

    for n in range(1, 8):
        long_limb = _create_test_long_limb(n)
        result = analyze_robot(long_limb)

        # Count joint types
        counts = {}
        for jt in result.joint_types.values():
            counts[jt] = counts.get(jt, 0) + 1

        expected = expected_classifications[n]
        success = True
        for jt, expected_count in expected.items():
            actual = counts.get(jt, 0)
            if actual != expected_count:
                print(f"  n={n}: FAILED - Expected {jt.name}={expected_count}, got {actual}")
                success = False

        if success:
            print(f"  n={n}: ✓ PASSED - {counts}")

    # Test 12: Core only
    print("\n--- Test 12: Core Only (no hinges) ---")
    from revolve2.modular_robot.body.v1 import BodyV1
    core_only = BodyV1()
    result = analyze_robot(core_only)
    print_analysis(result)
    assert len(result.body_modules) == 1, "Should have 1 body module (Core)"
    assert len(result.all_hinges) == 0, "Should have 0 hinges"
    print("✓ Test 12 PASSED")

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    run_tests()
