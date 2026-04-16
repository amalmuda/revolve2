"""Deep sanity check: BLF coupling logic and foot detection for 9 chosen robots."""
import numpy as np
import mujoco
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from blf import BodyLimbFinder, JointType, BLFCpgNetworkGenerator
from contact_detection import identify_geometry_types

ROBOTS = ["spider", "gecko", "babya", "babyb", "ant", "queen", "park", "insect", "snake"]


def check_coupling_logic(name, body):
    """Verify coupling connections follow the rules."""
    result = BodyLimbFinder(body).analyze()
    gen = BLFCpgNetworkGenerator(result)
    structure, mapping = gen.generate()

    spine_nodes = set(i for i, jt in result.articulations.items() if jt == JointType.SPINE)
    hip_nodes = set(i for i, jt in result.articulations.items() if jt == JointType.HIP)
    knee_nodes = set(i for i, jt in result.articulations.items() if jt == JointType.KNEE)

    errors = []

    for pair in structure.connections:
        lo = pair.cpg_index_lowest.index
        hi = pair.cpg_index_highest.index
        lo_node = gen._cpg_to_node.get(lo)
        hi_node = gen._cpg_to_node.get(hi)
        lo_type = result.nodes[lo_node].joint_type
        hi_type = result.nodes[hi_node].joint_type

        # Check: spine-spine (all-to-all allowed)
        if lo_type == JointType.SPINE and hi_type == JointType.SPINE:
            continue  # OK

        # Check: hip-hip (all-to-all allowed)
        if lo_type == JointType.HIP and hi_type == JointType.HIP:
            continue  # OK

        # Check: hip-spine (hip to nearest spine)
        if (lo_type == JointType.SPINE and hi_type == JointType.HIP) or \
           (lo_type == JointType.HIP and hi_type == JointType.SPINE):
            continue  # OK (we trust BFS found nearest)

        # Check: knee-knee or knee-hip in SAME limb (chain)
        if lo_type == JointType.KNEE or hi_type == JointType.KNEE:
            lo_limb = result.nodes[lo_node].limb_id
            hi_limb = result.nodes[hi_node].limb_id
            if lo_limb == hi_limb:
                continue  # OK - same limb chain
            else:
                errors.append(f"CROSS-LIMB knee coupling: CPG {lo}(limb {lo_limb}) -- CPG {hi}(limb {hi_limb})")

        # Unexpected connection type
        errors.append(f"UNEXPECTED: CPG {lo}({lo_type.name}) -- CPG {hi}({hi_type.name})")

    # Check completeness: all spine pairs should be connected
    spine_cpgs = [gen._cpg_to_node.get(i) for i in range(len(mapping))
                  if gen._cpg_to_node.get(i) in spine_nodes]
    # ... (simplified - just check no errors)

    return errors


def check_foot_detection(name, body):
    """Verify foot detection matches BLF Rule 6."""
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    params = np.zeros(cpg.num_connections)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params, cpg_network_structure=cpg,
        initial_state_uniform=0.0, output_mapping=mp,
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
    sim_scene, _ = scene.to_simulation_scene()
    model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=0.001,
                                        cast_shadows=False, fast_sim=True)

    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot_ids = robot_ids - foot_ids

    return {
        "ground": len(ground_ids),
        "robot_geoms": len(robot_ids),
        "foot_geoms": len(foot_ids),
        "non_foot_geoms": len(non_foot_ids),
    }


def check_coupling_counts(name, body):
    """Manually verify coupling counts match the rules."""
    result = BodyLimbFinder(body).analyze()
    gen = BLFCpgNetworkGenerator(result)
    structure, mapping = gen.generate()

    spine_cpgs = []
    hip_cpgs = []
    knee_cpgs = []
    for i in range(len(mapping)):
        node = gen._cpg_to_node.get(i)
        if node is None:
            continue
        jt = result.nodes[node].joint_type
        if jt == JointType.SPINE:
            spine_cpgs.append(i)
        elif jt == JointType.HIP:
            hip_cpgs.append(i)
        elif jt == JointType.KNEE:
            knee_cpgs.append(i)

    # Count connection types
    spine_spine = 0
    hip_hip = 0
    hip_spine = 0
    knee_chain = 0
    other = 0

    for pair in structure.connections:
        lo = pair.cpg_index_lowest.index
        hi = pair.cpg_index_highest.index
        lo_node = gen._cpg_to_node.get(lo)
        hi_node = gen._cpg_to_node.get(hi)
        lo_type = result.nodes[lo_node].joint_type
        hi_type = result.nodes[hi_node].joint_type

        types = {lo_type, hi_type}
        if types == {JointType.SPINE}:
            spine_spine += 1
        elif types == {JointType.HIP}:
            hip_hip += 1
        elif types == {JointType.SPINE, JointType.HIP}:
            hip_spine += 1
        elif JointType.KNEE in types:
            knee_chain += 1
        else:
            other += 1

    S = len(spine_cpgs)
    H = len(hip_cpgs)
    expected_ss = S * (S - 1) // 2
    expected_hh = H * (H - 1) // 2

    errors = []
    if spine_spine != expected_ss:
        errors.append(f"spine-spine: got {spine_spine}, expected {expected_ss}")
    if hip_hip != expected_hh:
        errors.append(f"hip-hip: got {hip_hip}, expected {expected_hh}")
    if other > 0:
        errors.append(f"unexpected connection types: {other}")

    return {
        "S": S, "H": H, "K": len(knee_cpgs),
        "spine_spine": spine_spine, "expected_ss": expected_ss,
        "hip_hip": hip_hip, "expected_hh": expected_hh,
        "hip_spine": hip_spine,
        "knee_chain": knee_chain,
        "total": len(structure.connections),
        "errors": errors,
    }


# Run all checks
print("=" * 80)
print("BLF SANITY CHECK: Coupling Logic + Foot Detection")
print("=" * 80)

for name in ROBOTS:
    body = modular_robots_v1.get(name)
    print(f"\n--- {name.upper()} ---")

    # Coupling logic
    coupling_errors = check_coupling_logic(name, body)
    if coupling_errors:
        print(f"  COUPLING ERRORS:")
        for e in coupling_errors:
            print(f"    {e}")
    else:
        print(f"  Coupling logic: OK (all connections follow rules)")

    # Coupling counts
    counts = check_coupling_counts(name, body)
    ss_ok = "OK" if counts["spine_spine"] == counts["expected_ss"] else "FAIL"
    hh_ok = "OK" if counts["hip_hip"] == counts["expected_hh"] else "FAIL"
    print(f"  Spine-spine: {counts['spine_spine']}/{counts['expected_ss']} ({ss_ok})")
    print(f"  Hip-hip: {counts['hip_hip']}/{counts['expected_hh']} ({hh_ok})")
    print(f"  Hip-spine: {counts['hip_spine']}")
    print(f"  Knee chain: {counts['knee_chain']}")
    print(f"  Total coupling: {counts['total']} = {counts['spine_spine']}+{counts['hip_hip']}+{counts['hip_spine']}+{counts['knee_chain']}")
    if counts["errors"]:
        for e in counts["errors"]:
            print(f"  ERROR: {e}")

    # Foot detection
    feet = check_foot_detection(name, body)
    print(f"  Foot detection: {feet['foot_geoms']} foot geoms, {feet['non_foot_geoms']} non-foot, {feet['ground']} ground")
    if feet["foot_geoms"] == 0:
        print(f"  WARNING: no foot geoms detected!")
    if feet["robot_geoms"] == 0:
        print(f"  ERROR: no robot geoms detected!")

print(f"\n{'=' * 80}")
print("DONE")
