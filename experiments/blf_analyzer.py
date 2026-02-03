"""Body/Limb Finder (BLF) for Revolve2 modular robots.

Analyzes robot structure to classify joints as spine, hip, knee, or ankle,
and generates coupling matrices based on bio-inspired rules.

BLF Rules (adapted for Revolve2):
- Body detection:
  - Core brick → always body
  - Any brick with 3+ hinges connected → also body
- Spine: Hinge between two body bricks
- Limb: Chain branching from body (not spine)
- Hip: First hinge in limb (touches body)
- Knee: Second hinge in limb
- Ankle: Third hinge in limb

Coupling Rules:
- Spine ↔ Spine: fully coupled
- Hip ↔ Hip: fully coupled
- Hip → nearest Spine: coupled (if spine exists)
- Knee → its Hip: coupled
- Ankle → its Knee: coupled
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import numpy as np

from revolve2.modular_robot.body.base import ActiveHinge, Body, Brick, Core
from revolve2.modular_robot.body import Module

# Try to import V2 specific modules
try:
    from revolve2.modular_robot.body.v2._attachment_face_core_v2 import AttachmentFaceCoreV2
    HAS_V2 = True
except ImportError:
    HAS_V2 = False
    AttachmentFaceCoreV2 = None


class JointType(Enum):
    """Classification of a joint in the robot structure."""
    SPINE = "spine"
    HIP = "hip"
    KNEE = "knee"
    ANKLE = "ankle"
    UNCLASSIFIED = "unclassified"


@dataclass
class JointInfo:
    """Information about a classified joint."""
    hinge: ActiveHinge
    index: int
    joint_type: JointType
    limb_id: Optional[int] = None  # Which limb this joint belongs to (for hip/knee/ankle)

    def __repr__(self):
        if self.limb_id is not None:
            return f"Joint {self.index}: {self.joint_type.value} (limb {self.limb_id})"
        return f"Joint {self.index}: {self.joint_type.value}"


@dataclass
class BLFResult:
    """Result of Body/Limb Finder analysis."""
    joints: list[JointInfo]
    body_modules: list[Module]
    limbs: list[list[ActiveHinge]]  # List of limbs, each limb is a list of hinges
    coupling_matrix: np.ndarray

    def get_joints_by_type(self, joint_type: JointType) -> list[JointInfo]:
        """Get all joints of a specific type."""
        return [j for j in self.joints if j.joint_type == joint_type]

    def get_spine_indices(self) -> list[int]:
        return [j.index for j in self.joints if j.joint_type == JointType.SPINE]

    def get_hip_indices(self) -> list[int]:
        return [j.index for j in self.joints if j.joint_type == JointType.HIP]

    def get_knee_indices(self) -> list[int]:
        return [j.index for j in self.joints if j.joint_type == JointType.KNEE]

    def get_ankle_indices(self) -> list[int]:
        return [j.index for j in self.joints if j.joint_type == JointType.ANKLE]

    def print_summary(self):
        """Print a summary of the BLF analysis."""
        print("=" * 50)
        print("BLF Analysis Summary")
        print("=" * 50)
        print(f"Total joints: {len(self.joints)}")
        print(f"  Spine joints: {len(self.get_spine_indices())} - indices: {self.get_spine_indices()}")
        print(f"  Hip joints:   {len(self.get_hip_indices())} - indices: {self.get_hip_indices()}")
        print(f"  Knee joints:  {len(self.get_knee_indices())} - indices: {self.get_knee_indices()}")
        print(f"  Ankle joints: {len(self.get_ankle_indices())} - indices: {self.get_ankle_indices()}")
        print(f"Number of limbs: {len(self.limbs)}")
        print(f"Body modules: {len(self.body_modules)}")
        print("\nJoint details:")
        for joint in self.joints:
            print(f"  {joint}")
        print("\nCoupling matrix:")
        print(self.coupling_matrix)


class BodyLimbFinder:
    """Analyzes robot structure to classify joints and generate coupling."""

    def __init__(self, body: Body):
        self.body = body
        self._body_modules: set[Module] = set()
        self._hinges: list[ActiveHinge] = []
        self._hinge_to_index: dict[ActiveHinge, int] = {}
        self._joint_infos: list[JointInfo] = []
        self._limbs: list[list[ActiveHinge]] = []

    def analyze(self) -> BLFResult:
        """Perform BLF analysis on the robot body."""
        # Step 1: Get all hinges in order
        self._hinges = self.body.find_modules_of_type(ActiveHinge)
        self._hinge_to_index = {h: i for i, h in enumerate(self._hinges)}

        # Step 2: Identify body modules
        self._identify_body_modules()

        # Step 3: Classify each hinge
        self._classify_hinges()

        # Step 4: Generate coupling matrix
        coupling_matrix = self._generate_coupling_matrix()

        return BLFResult(
            joints=self._joint_infos,
            body_modules=list(self._body_modules),
            limbs=self._limbs,
            coupling_matrix=coupling_matrix,
        )

    def _identify_body_modules(self):
        """Identify which modules are part of the body."""
        # Core is always body
        self._body_modules.add(self.body.core)

        # AttachmentFaceCoreV2 (if present) is part of body
        all_modules = self._get_all_modules(self.body.core)
        for module in all_modules:
            # AttachmentFaces are part of the body (they're extensions of the core)
            if HAS_V2 and AttachmentFaceCoreV2 is not None and isinstance(module, AttachmentFaceCoreV2):
                self._body_modules.add(module)

        # Find bricks with 3+ hinges connected (branching points)
        for module in all_modules:
            if isinstance(module, (Core, Brick)):
                # Count hinges connected to this module (as children or parent)
                hinge_count = self._count_connected_hinges(module)
                if hinge_count >= 3:
                    self._body_modules.add(module)

        # Also mark bricks that are in a chain between body modules as body
        # This handles spine segments
        self._mark_spine_bricks()

    def _get_all_modules(self, start: Module) -> list[Module]:
        """Get all modules in the robot tree."""
        modules = [start]
        for child in start.children.values():
            modules.extend(self._get_all_modules(child))
        return modules

    def _count_connected_hinges(self, module: Module) -> int:
        """Count how many hinges are directly connected to a module."""
        count = 0
        # Check children
        for child in module.children.values():
            if isinstance(child, ActiveHinge):
                count += 1
        # Check parent
        if module.parent is not None and isinstance(module.parent, ActiveHinge):
            count += 1
        return count

    def _mark_spine_bricks(self):
        """Mark bricks that are part of the spine (between body modules).

        A brick is part of the spine if:
        - It has 3+ hinge connections (branching point), OR
        - It is on a chain between the core and another body branching point
        """
        all_modules = self._get_all_modules(self.body.core)

        # First pass: Mark branching points (3+ connections)
        for module in all_modules:
            if isinstance(module, Brick):
                hinge_count = self._count_connected_hinges(module)
                if hinge_count >= 3:
                    self._body_modules.add(module)

        # Second pass: Mark linear spine segments (bricks between body modules)
        # A brick with 2 connections is a spine segment if it's on a path from
        # the core to another body module (branching point)
        changed = True
        while changed:
            changed = False
            for module in all_modules:
                if isinstance(module, Brick) and module not in self._body_modules:
                    hinge_count = self._count_connected_hinges(module)
                    if hinge_count == 2:
                        # Check if this brick connects body modules on both sides
                        if self._brick_connects_body_modules(module):
                            self._body_modules.add(module)
                            changed = True

    def _brick_connects_body_modules(self, brick: Brick) -> bool:
        """Check if a brick is on a path between two body modules."""
        parent_has_body = self._has_body_ancestor(brick)
        child_has_body = self._has_body_descendant(brick)
        return parent_has_body and child_has_body

    def _has_body_ancestor(self, module: Module) -> bool:
        """Check if there's a body module in the ancestor chain."""
        current = module.parent
        while current is not None:
            if current in self._body_modules or self._is_attachment_face(current):
                return True
            if isinstance(current, Core):
                return True
            current = current.parent
        return False

    def _has_body_descendant(self, module: Module) -> bool:
        """Check if there's a body module in the descendant tree."""
        for child in module.children.values():
            if child in self._body_modules:
                return True
            if isinstance(child, ActiveHinge):
                hinge_child = child.children.get(ActiveHinge.ATTACHMENT)
                if hinge_child is not None:
                    if hinge_child in self._body_modules:
                        return True
                    if isinstance(hinge_child, Brick) and self._has_body_descendant(hinge_child):
                        return True
        return False

    def _get_nearest_body_ancestor(self, module: Module) -> Optional[Module]:
        """Find the nearest body module ancestor."""
        current = module.parent
        while current is not None:
            if current in self._body_modules:
                return current
            current = current.parent
        return None

    def _is_body_module(self, module: Module) -> bool:
        """Check if a module is part of the body."""
        if module in self._body_modules:
            return True
        # AttachmentFaces are implicitly body modules (part of core)
        if HAS_V2 and AttachmentFaceCoreV2 is not None:
            if type(module).__name__ == 'AttachmentFaceCoreV2':
                return True
        return False

    def _is_attachment_face(self, module: Module) -> bool:
        """Check if module is an AttachmentFaceCoreV2."""
        if HAS_V2 and AttachmentFaceCoreV2 is not None:
            return type(module).__name__ == 'AttachmentFaceCoreV2'
        return False

    def _get_effective_parent(self, hinge: ActiveHinge) -> Optional[Module]:
        """Get the effective parent module, skipping AttachmentFaces to find Core/Brick."""
        parent = hinge.parent
        # Skip through AttachmentFaces to find the real body module (Core or Brick)
        while parent is not None:
            if isinstance(parent, (Core, Brick)):
                return parent
            if self._is_attachment_face(parent):
                # AttachmentFace - keep going to find Core
                parent = parent.parent
            else:
                # Some other module type
                return parent
        return None

    def _get_parent_body_or_hinge(self, hinge: ActiveHinge) -> Optional[Module]:
        """Get the body module or hinge that this hinge is connected to (towards core)."""
        return hinge.parent

    def _get_child_module(self, hinge: ActiveHinge) -> Optional[Module]:
        """Get the module attached to this hinge (away from core)."""
        return hinge.children.get(ActiveHinge.ATTACHMENT)

    def _classify_hinges(self):
        """Classify all hinges as spine, hip, knee, or ankle."""
        visited = set()
        limb_id = 0

        for hinge in self._hinges:
            if hinge in visited:
                continue

            idx = self._hinge_to_index[hinge]
            # Get effective parent (skipping AttachmentFaces)
            effective_parent = self._get_effective_parent(hinge)
            child = self._get_child_module(hinge)

            # Check if parent is a body module
            parent_is_body = effective_parent is not None and self._is_body_module(effective_parent)
            # For parent directly attached to hinge (including AttachmentFaces)
            direct_parent_is_body = hinge.parent is not None and (
                self._is_body_module(hinge.parent) or self._is_attachment_face(hinge.parent)
            )

            # Check if child is a body module
            child_is_body = child is not None and self._is_body_module(child)

            if (parent_is_body or direct_parent_is_body) and child_is_body:
                # This is a spine joint (between two body modules)
                self._joint_infos.append(JointInfo(
                    hinge=hinge,
                    index=idx,
                    joint_type=JointType.SPINE,
                ))
                visited.add(hinge)
            elif (parent_is_body or direct_parent_is_body) and not child_is_body:
                # This is the start of a limb - hip joint
                limb_hinges = self._trace_limb(hinge)
                self._limbs.append(limb_hinges)

                for i, limb_hinge in enumerate(limb_hinges):
                    limb_idx = self._hinge_to_index[limb_hinge]
                    if i == 0:
                        joint_type = JointType.HIP
                    elif i == 1:
                        joint_type = JointType.KNEE
                    elif i == 2:
                        joint_type = JointType.ANKLE
                    else:
                        joint_type = JointType.UNCLASSIFIED

                    self._joint_infos.append(JointInfo(
                        hinge=limb_hinge,
                        index=limb_idx,
                        joint_type=joint_type,
                        limb_id=limb_id,
                    ))
                    visited.add(limb_hinge)

                limb_id += 1

        # Sort joint infos by index
        self._joint_infos.sort(key=lambda j: j.index)

    def _trace_limb(self, start_hinge: ActiveHinge) -> list[ActiveHinge]:
        """Trace a limb from its hip to its end, collecting all hinges."""
        limb_hinges = [start_hinge]
        current = self._get_child_module(start_hinge)

        while current is not None:
            if isinstance(current, ActiveHinge):
                limb_hinges.append(current)
                current = self._get_child_module(current)
            elif isinstance(current, (Brick, Core)):
                # Check if this brick leads to another hinge
                next_hinge = None
                for child in current.children.values():
                    if isinstance(child, ActiveHinge):
                        next_hinge = child
                        break
                if next_hinge is not None:
                    limb_hinges.append(next_hinge)
                    current = self._get_child_module(next_hinge)
                else:
                    break
            else:
                break

        return limb_hinges

    def _generate_coupling_matrix(self) -> np.ndarray:
        """Generate coupling matrix based on BLF rules."""
        n = len(self._hinges)
        coupling = np.zeros((n, n), dtype=float)

        # Get indices by type
        spine_indices = [j.index for j in self._joint_infos if j.joint_type == JointType.SPINE]
        hip_indices = [j.index for j in self._joint_infos if j.joint_type == JointType.HIP]
        knee_indices = [j.index for j in self._joint_infos if j.joint_type == JointType.KNEE]
        ankle_indices = [j.index for j in self._joint_infos if j.joint_type == JointType.ANKLE]

        # Rule 1: Spine ↔ Spine: fully coupled
        for i in spine_indices:
            for j in spine_indices:
                if i != j:
                    coupling[i, j] = 1.0

        # Rule 2: Hip ↔ Hip: fully coupled
        for i in hip_indices:
            for j in hip_indices:
                if i != j:
                    coupling[i, j] = 1.0

        # Rule 3: Hip → nearest Spine: coupled (if spine exists)
        if spine_indices:
            for hip_joint in self._joint_infos:
                if hip_joint.joint_type == JointType.HIP:
                    # Find nearest spine (for now, couple to all spines)
                    # In a more sophisticated version, we'd find the geometrically nearest
                    for spine_idx in spine_indices:
                        coupling[hip_joint.index, spine_idx] = 1.0
                        coupling[spine_idx, hip_joint.index] = 1.0

        # Rule 4: Knee → its Hip: coupled
        for knee_joint in self._joint_infos:
            if knee_joint.joint_type == JointType.KNEE:
                # Find the hip in the same limb
                for hip_joint in self._joint_infos:
                    if hip_joint.joint_type == JointType.HIP and hip_joint.limb_id == knee_joint.limb_id:
                        coupling[knee_joint.index, hip_joint.index] = 1.0
                        coupling[hip_joint.index, knee_joint.index] = 1.0
                        break

        # Rule 5: Ankle → its Knee: coupled
        for ankle_joint in self._joint_infos:
            if ankle_joint.joint_type == JointType.ANKLE:
                # Find the knee in the same limb
                for knee_joint in self._joint_infos:
                    if knee_joint.joint_type == JointType.KNEE and knee_joint.limb_id == ankle_joint.limb_id:
                        coupling[ankle_joint.index, knee_joint.index] = 1.0
                        coupling[knee_joint.index, ankle_joint.index] = 1.0
                        break

        return coupling


def analyze_body(body: Body) -> BLFResult:
    """Convenience function to analyze a robot body."""
    blf = BodyLimbFinder(body)
    return blf.analyze()


if __name__ == "__main__":
    # Test with spider and gecko
    from revolve2.standards import modular_robots_v1, modular_robots_v2

    print("\n" + "=" * 60)
    print("SPIDER v1 Analysis")
    print("=" * 60)
    spider = modular_robots_v1.get("spider")
    spider_result = analyze_body(spider)
    spider_result.print_summary()

    print("\n" + "=" * 60)
    print("GECKO v2 Analysis")
    print("=" * 60)
    gecko = modular_robots_v2.get("gecko")
    gecko_result = analyze_body(gecko)
    gecko_result.print_summary()
