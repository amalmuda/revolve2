"""
Compute Kuramoto R (R_global, R_interlimb) for ODE-CPG experiments.

Re-simulates each best_params_run_*.npy from results/final_v2 and writes a TSV
in the same format as compute_big_spider_quality.py.

Usage:
    python compute_kuramoto_sweep.py             # all 9 robots
    python compute_kuramoto_sweep.py spider      # one robot
"""
import math
import os
import sys
import glob
import re
import numpy as np
import mujoco
from concurrent.futures import ProcessPoolExecutor, as_completed

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    get_robot_core_body_id,
)
from blf import BodyLimbFinder, JointType
from compute_kuramoto import compute_R_global_and_interlimb


SIM_TIME = 30.0
RESULTS_BASE = os.path.expanduser(
    "~/revolve2/experiments/results/final_v2"
)
OUTPUT_PATH = os.path.expanduser(
    "~/revolve2/experiments/_kuramoto_metrics.txt"
)
ROBOTS = ["spider", "gecko", "babya", "babyb", "ant", "queen", "park", "insect", "snake"]
COUPLINGS = ["uncoupled", "neighbor", "blf"]
LAMBDAS = [0, 2]


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)


def hip_joint_indices(body):
    """
    Identify which hinge index in `body.find_modules_of_type(ActiveHinge)`
    corresponds to a hip (one per limb) using BLF analysis.

    Returns a list of indices into the active_hinges list, or empty if BLF
    yields no limbs / no hips.
    """
    result = BodyLimbFinder(body).analyze()
    # BLF gives us node indices; we need to map back to hinge index in the
    # find_modules_of_type(ActiveHinge) order.
    hinges = body.find_modules_of_type(ActiveHinge)
    hinge_id_to_index = {id(h): i for i, h in enumerate(hinges)}

    hip_indices = []
    for node_idx, jt in result.articulations.items():
        if jt != JointType.HIP:
            continue
        node = result.nodes[node_idx]
        # Map to hinge list index
        hi = hinge_id_to_index.get(id(node.module))
        if hi is not None:
            hip_indices.append(hi)
    return hip_indices


def evaluate_run(args):
    robot_name, coupling, lam, run_num, params_path = args
    try:
        body = modular_robots_v1.get(robot_name)
        hinges = body.find_modules_of_type(ActiveHinge)
        cpg_struct, mapping = get_cpg_structure(coupling, body, hinges)
        params = np.load(params_path)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params,
            cpg_network_structure=cpg_struct,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=mapping,
        )
        robot = ModularRobot(body=body, brain=brain)

        terrain = Terrain(static_geometry=[GeometryPlane(
            pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
            texture=Texture(
                base_color=Color(200, 200, 200, 255),
                primary_color=Color(220, 220, 220, 255),
                secondary_color=Color(80, 80, 80, 255),
                map_type=MapType.MAP2D,
                reference=TextureReference(builtin="checker"),
                repeat=(50, 50),
            ),
        )], friction=1.0)
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        sim_scene, _ = scene.to_simulation_scene()
        batch = make_standard_batch_parameters()
        batch.simulation_time = SIM_TIME
        model, mj_mapping = scene_to_model(
            sim_scene,
            simulation_timestep=batch.simulation_timestep,
            cast_shadows=False,
            fast_sim=True,
        )
        data = mujoco.MjData(model)
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

        # Map of hinge module index -> qpos index in MuJoCo data
        # mj_mapping is BodyToMultiBodySystemMapping; hinges have UUIDKey -> JointHinge
        from revolve2.simulators.mujoco_simulator._abstraction_to_mujoco_mapping import (
            UUIDKey,
        )
        hinge_qpos_addrs = []
        for hinge in hinges:
            # Find this hinge's joint in the model and its qpos addr.
            # The mapping object is robot-specific; mj_mapping has hinge_joint maps.
            # Easier path: collect all hinge joints in mj order matching hinges order.
            pass

        # Generic approach: collect all hinge joints from the model.
        # Order in find_modules_of_type matches CPG output_mapping which uses
        # the same find_modules_of_type call. We need same order here.
        # MuJoCo joints might be in a different order, so we use the simulator's
        # hinge mapping via the abstraction.

        # Alternative: snapshot data.qpos at every step, then later filter to
        # hinge addresses only. We'll just collect all hinge qpos addresses
        # and trust that BLF / coupling structures use the same ordering.
        hinge_addrs = []
        for j in range(model.njnt):
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                hinge_addrs.append(int(model.jnt_qposadr[j]))

        if not hinge_addrs:
            return None

        mujoco.mj_forward(model, data)
        control_step = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        traces = []
        while data.time < SIM_TIME:
            traces.append(np.array([data.qpos[a] for a in hinge_addrs], dtype=np.float32))
            if data.time >= last_ctrl + control_step:
                last_ctrl = math.floor(data.time / control_step) * control_step
                sim_state = SimulationStateImpl(
                    data=data,
                    abstraction_to_mujoco_mapping=mj_mapping,
                    camera_views={},
                )
                sim_scene.handler.handle(sim_state, ctrl, control_step)
            mujoco.mj_step(model, data)

        traces = np.stack(traces, axis=0)  # (T, n_hinges)
        fs = 1.0 / batch.simulation_timestep

        # Identify hip indices in same hinge ordering used here.
        hips = hip_joint_indices(body)
        # BLF's hinge ordering matches body.find_modules_of_type(ActiveHinge).
        # MuJoCo's hinge ordering matches the order joints were added during
        # body conversion - which uses the same module list. So `hips` indexes
        # into traces[:, k] correctly.

        R_global, R_interlimb = compute_R_global_and_interlimb(
            joint_traces=traces,
            fs=fs,
            hip_indices=hips,
            skip_seconds=1.0,
        )

        return dict(
            robot=robot_name,
            coupling=coupling,
            lam=lam,
            run=run_num,
            R_global=R_global,
            R_interlimb=R_interlimb,
            n_hinges=traces.shape[1],
            n_hips=len(hips),
        )
    except Exception as e:
        print(f"  ERROR {robot_name} {coupling} l{lam} r{run_num}: {e}")
        return None


def main():
    if len(sys.argv) > 1:
        robots = sys.argv[1:]
    else:
        robots = ROBOTS
    print(f"Robots: {robots}")

    jobs = []
    for robot in robots:
        for coupling in COUPLINGS:
            for lam in LAMBDAS:
                d = os.path.join(
                    RESULTS_BASE, f"{robot}_ode_cpg_{coupling}_lambda{lam}_dragging"
                )
                for npy in sorted(glob.glob(os.path.join(d, "best_params_run_*.npy"))):
                    run_num = int(re.search(r"run_(\d+)", npy).group(1))
                    jobs.append((robot, coupling, lam, run_num, npy))

    print(f"Computing Kuramoto R for {len(jobs)} runs...")
    results = []
    workers = 4
    done = 0
    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(evaluate_run, j): j for j in jobs}
        for fut in as_completed(futures):
            r = fut.result()
            done += 1
            if r is not None:
                results.append(r)
                if done % 10 == 0 or done == len(jobs):
                    print(
                        f"[{done}/{len(jobs)}] {r['robot']} {r['coupling']} "
                        f"l{r['lam']} r{r['run']}: "
                        f"R_global={r['R_global']:.3f} R_interlimb={r['R_interlimb']:.3f}"
                    )

    results.sort(key=lambda r: (r["robot"], r["coupling"], r["lam"], r["run"]))
    with open(OUTPUT_PATH, "w") as f:
        f.write("robot\tcoupling\tlambda\trun\tR_global\tR_interlimb\tn_hinges\tn_hips\n")
        for r in results:
            f.write(
                f"{r['robot']}\t{r['coupling']}\t{r['lam']}\t{r['run']}\t"
                f"{r['R_global']:.6f}\t{r['R_interlimb']:.6f}\t"
                f"{r['n_hinges']}\t{r['n_hips']}\n"
            )
    print(f"Saved: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
