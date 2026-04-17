"""Walk each of the 9 thesis robots and print a structural description.

For each robot: tree dump with hinge orientation, BLF classification, module counts.
"""
import math
from revolve2.modular_robot.body.base import ActiveHinge, Brick, Core
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BrickV1
from revolve2.standards import modular_robots_v1

from blf import analyze_robot, JointType


ROBOTS = ["spider", "babyb", "gecko", "babya", "ant", "queen", "park", "insect", "snake"]


def describe_hinge_axis(rot: float) -> str:
    """Interpret hinge rotation parameter (X-axis euler)."""
    # normalize to [-pi, pi]
    r = ((rot + math.pi) % (2 * math.pi)) - math.pi
    snap = round(r / (math.pi / 2))
    if snap == 0:
        return "bend-V (child up/down)"
    if snap in (1, -1):
        return "bend-H (child L/R)"
    if snap == 2:
        return "bend-V-flip"
    return f"rot={r:.2f}"


def get_x_rot(module) -> float:
    """Extract the X-axis euler angle from a module's orientation quaternion."""
    q = module.orientation
    # quaternion -> euler (xyz)
    w, x, y, z = q.w, q.x, q.y, q.z
    # rotation about x axis
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    return math.atan2(sinr, cosr)


def walk_module(module, depth=0, face_name="core", lines=None):
    if lines is None:
        lines = []
    prefix = "  " * depth
    tname = type(module).__name__
    if isinstance(module, ActiveHingeV1):
        rot = get_x_rot(module)
        axis = describe_hinge_axis(rot)
        lines.append(f"{prefix}{face_name}: HINGE [{axis}]")
    elif isinstance(module, BrickV1):
        lines.append(f"{prefix}{face_name}: BRICK")
    else:
        lines.append(f"{prefix}{face_name}: {tname}")

    # walk children - different module types have different face names
    faces = []
    for attr in ("front", "back", "left", "right", "attachment"):
        if hasattr(module, attr):
            try:
                child = getattr(module, attr)
                if child is not None:
                    faces.append((attr, child))
            except AttributeError:
                pass
    for fname, child in faces:
        walk_module(child, depth + 1, fname, lines)
    return lines


def describe_robot(name: str):
    print("=" * 70)
    print(f"ROBOT: {name.upper()}")
    print("=" * 70)
    body = modular_robots_v1.get(name)

    # BLF classification
    result = analyze_robot(body)
    spines = [i for i, j in result.articulations.items() if j == JointType.SPINE]
    hips = [i for i, j in result.articulations.items() if j == JointType.HIP]
    knees = [i for i, j in result.articulations.items() if j == JointType.KNEE]

    hinges = body.find_modules_of_type(ActiveHinge)
    bricks = body.find_modules_of_type(Brick)
    print(f"Modules: {len(result.nodes)}  Hinges: {len(hinges)}  Bricks: {len(bricks)}")
    print(f"BLF: {len(spines)} spine, {len(hips)} hips, {len(knees)} knees, {len(result.limbs)} limbs")

    # Limb signatures (shape of each limb)
    for i, limb in enumerate(result.limbs):
        joints_in_limb = []
        for idx in limb:
            node = result.nodes[idx]
            if isinstance(node.module, ActiveHinge):
                jt = result.articulations.get(idx, JointType.UNCLASSIFIED)
                joints_in_limb.append(jt.name[0])  # S/H/K
        sig = "".join(joints_in_limb) if joints_in_limb else "(no hinges)"
        print(f"  Limb {i}: {len(limb)} modules, hinges=[{sig}]")

    # Tree dump
    print("\nTree:")
    for line in walk_module(body.core_v1, 0, "CORE"):
        print("  " + line)
    print()


for r in ROBOTS:
    describe_robot(r)
