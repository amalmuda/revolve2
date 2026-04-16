"""Test BLF on robots with branching spines."""
import numpy as np
import math
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from blf import BodyLimbFinder, JointType, BLFCpgNetworkGenerator, print_blf_analysis


def y_spine_robot():
    """
    Y-shaped spine: core -> 2-hinge spine -> junction brick that branches
    into two spine arms, each with legs.

    Core
      |
     [H] spine1 (90 deg)
      |
     [B]
      |
     [H] spine2 (90 deg)
      |
     [B] <-- junction: 3 hinges touch it (spine2 + left_branch + right_branch) = body
    /   \
  [H]   [H]  spine branches (90 deg each)
   |     |
  [B]   [B]  <-- sub-junctions: 3 hinges each (branch + 2 legs) = body
  / \   / \
[H][H] [H][H]  legs (0 deg)
 |  |   |  |
[B][B] [B][B]  feet
    """
    body = BodyV1()
    # Spine from core
    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)  # spine1
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(np.pi / 2.0)  # spine2
    body.core_v1.back.attachment.front.attachment = BrickV1(-np.pi / 2.0)  # junction

    # Left branch
    junction = body.core_v1.back.attachment.front.attachment
    junction.left = ActiveHingeV1(np.pi / 2.0)  # left branch spine
    junction.left.attachment = BrickV1(-np.pi / 2.0)  # left sub-junction
    junction.left.attachment.left = ActiveHingeV1(0.0)  # left leg 1
    junction.left.attachment.left.attachment = BrickV1(0.0)
    junction.left.attachment.front = ActiveHingeV1(0.0)  # left leg 2
    junction.left.attachment.front.attachment = BrickV1(0.0)

    # Right branch
    junction.right = ActiveHingeV1(np.pi / 2.0)  # right branch spine
    junction.right.attachment = BrickV1(-np.pi / 2.0)  # right sub-junction
    junction.right.attachment.right = ActiveHingeV1(0.0)  # right leg 1
    junction.right.attachment.right.attachment = BrickV1(0.0)
    junction.right.attachment.front = ActiveHingeV1(0.0)  # right leg 2
    junction.right.attachment.front.attachment = BrickV1(0.0)

    # Front legs from core
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)

    return body


def t_spine_robot():
    """
    T-shaped spine: core branches into 3 spine arms directly.
    Each arm ends in a junction with 2 legs.

         [legs]
           |
    [legs]-CORE-[legs]
           |
         [legs]
    but with a hinge+brick between core and each leg junction.
    """
    body = BodyV1()

    # Left arm
    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)  # left spine
    body.core_v1.left.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.left.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.front.attachment = BrickV1(0.0)

    # Right arm
    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)  # right spine
    body.core_v1.right.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.right.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.right.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.front.attachment = BrickV1(0.0)

    # Back arm
    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)  # back spine
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.right.attachment = BrickV1(0.0)

    # Front arm
    body.core_v1.front = ActiveHingeV1(np.pi / 2.0)  # front spine
    body.core_v1.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.front.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.front.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.front.attachment.right.attachment = BrickV1(0.0)

    return body


def analyze_and_print(name, body):
    hinges = body.find_modules_of_type(ActiveHinge)
    result = BodyLimbFinder(body).analyze()
    gen = BLFCpgNetworkGenerator(result)
    structure, mapping = gen.generate()

    spine = sum(1 for jt in result.articulations.values() if jt == JointType.SPINE)
    hips = sum(1 for jt in result.articulations.values() if jt == JointType.HIP)
    knees = sum(1 for jt in result.articulations.values() if jt == JointType.KNEE)

    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"{'='*60}")
    print(f"Hinges: {len(hinges)} | Spine: {spine} | Hips: {hips} | Knees: {knees}")
    print(f"Body nodes: {len(result.body_nodes)} | Limbs: {len(result.limbs)}")
    print(f"CPG: {structure.num_cpgs} internal + {len(structure.connections)} coupling = {structure.num_connections} params")
    print()
    print_blf_analysis(result)

    # Show coupling details
    print("\nCoupling connections:")
    for pair in sorted(structure.connections, key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index)):
        lo = pair.cpg_index_lowest.index
        hi = pair.cpg_index_highest.index
        lo_node = gen._cpg_to_node.get(lo)
        hi_node = gen._cpg_to_node.get(hi)
        lo_type = result.nodes[lo_node].joint_type.name if lo_node else "?"
        hi_type = result.nodes[hi_node].joint_type.name if hi_node else "?"
        print(f"  CPG {lo}({lo_type}) -- CPG {hi}({hi_type})")


def visualize(name, body):
    """Open in MuJoCo viewer."""
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.simulation.scene import Color, Pose
    from revolve2.simulation.scene.geometry import GeometryPlane
    from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
    from revolve2.simulation.scene.vector2 import Vector2
    from revolve2.simulation.simulator import BatchParameters

    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
    params = np.zeros(cpg.num_connections)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params, cpg_network_structure=cpg,
        initial_state_uniform=0.0, output_mapping=mapping,
    )
    robot = ModularRobot(body=body, brain=brain)
    terrain = Terrain(
        static_geometry=[GeometryPlane(
            pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
            texture=Texture(
                base_color=Color(200, 200, 200, 255),
                primary_color=Color(220, 220, 220, 255),
                secondary_color=Color(80, 80, 80, 255),
                map_type=MapType.MAP2D,
                reference=TextureReference(builtin="checker"),
                repeat=(50, 50),
            ),
        )],
        friction=1.0,
    )
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    print(f"\nOpening viewer: {name} (static, {len(hinges)} hinges)")
    simulator = LocalSimulator(headless=False, num_simulators=1)
    batch_params = BatchParameters(
        simulation_time=120, sampling_frequency=1,
        simulation_timestep=0.001, control_frequency=50,
    )
    simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)


if __name__ == "__main__":
    import sys

    # Analyze both
    y_body = y_spine_robot()
    t_body = t_spine_robot()

    analyze_and_print("Y-SPINE (branches at junction)", y_body)
    analyze_and_print("T-SPINE (branches at core)", t_body)

    # Visualize if requested
    if "--viz" in sys.argv:
        which = sys.argv[sys.argv.index("--viz") + 1] if len(sys.argv) > sys.argv.index("--viz") + 1 else "y"
        if which == "y":
            visualize("Y-SPINE", y_body)
        elif which == "t":
            visualize("T-SPINE", t_body)
        else:
            visualize("Y-SPINE", y_body)
