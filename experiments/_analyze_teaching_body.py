"""Run BLF on the teaching body and dump per-module classifications."""
from revolve2.modular_robot.body.base import ActiveHinge, Brick, Core

from teaching_body import teaching_body_v1
from blf import analyze_robot, PartType, JointType


def describe(module):
    return type(module).__name__.replace("V1", "")


def main():
    body = teaching_body_v1()
    result = analyze_robot(body)

    print(f"Total modules: {len(result.nodes)}")
    print(f"Body nodes:    {len(result.body_nodes)}")
    print(f"Limbs:         {len(result.limbs)}")
    print()
    print(f"{'idx':>4} {'module':>12} {'role':>6} {'joint':>8}  neighbours")
    for n in result.nodes:
        jt = n.joint_type.name if n.joint_type != JointType.UNCLASSIFIED else "-"
        pt = n.part_type.name
        nbs = ",".join(str(x) for x in n.neighbors)
        print(f"{n.index:>4d} {describe(n.module):>12s} {pt:>6s} {jt:>8s}  [{nbs}]")

    print()
    print("--- Rule verification ---")
    # Rule 1: Core is body
    core_ok = any(isinstance(n.module, Core) and n.part_type == PartType.BODY
                  for n in result.nodes)
    print(f"Rule 1 (core = body)                     : {'OK' if core_ok else 'FAIL'}")

    # Rule 1 (new, 'core-attached bricks are body'): NOT in current blf.py implementation
    # but Rule 3 catches this brick via the path between core and the cluster.
    core_bricks_body = [n for n in result.nodes
                        if isinstance(n.module, Brick) and n.part_type == PartType.BODY]
    print(f"Rule 1/3 (>=1 brick has role=body)       : {'OK' if core_bricks_body else 'FAIL'}  ({len(core_bricks_body)} bricks)")

    # Rule 2: cluster with >2 touching hinges -> body
    # Our cluster is {JA, JB} both bricks; both should be body.
    print(f"Rule 2 (brick cluster body, >=2 bricks)  : {'OK' if len(core_bricks_body) >= 2 else 'FAIL'}")

    # Rule 3: path spine hinge classified spine via rule 5.
    spines = [n for n in result.nodes if n.joint_type == JointType.SPINE]
    print(f"Rule 3+5 (>=1 spine hinge)               : {'OK' if spines else 'FAIL'}  ({len(spines)} spine hinges)")

    # Rule 5: hips and knees
    hips = [n for n in result.nodes if n.joint_type == JointType.HIP]
    knees = [n for n in result.nodes if n.joint_type == JointType.KNEE]
    print(f"Rule 5 (>=1 hip, >=1 knee)               : {'OK' if hips and knees else 'FAIL'}  ({len(hips)} hip(s), {len(knees)} knee(s))")

    # Rule 6: foot. NOT implemented in blf.py -- cannot verify from BLF output.
    print(f"Rule 6 (foot classification)             : SKIP -- not implemented in blf.py")


if __name__ == "__main__":
    main()
