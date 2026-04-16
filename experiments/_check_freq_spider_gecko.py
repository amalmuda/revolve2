"""Check frequencies for spider and gecko (fully connected topology)."""
import math
import os
import glob
import re
import numpy as np
import mujoco
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import defaultdict

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
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

from contact_detection import active_hinges_to_cpg_network_structure_fully_connected

SIM_TIME = 10.0


def _zero_cross_freq(signal, fs):
    sig = signal - signal.mean()
    if np.allclose(sig, 0):
        return 0.0
    zc = np.where(np.diff(np.signbit(sig)))[0]
    if len(zc) < 2:
        return 0.0
    period_samples = np.mean(np.diff(zc)) * 2
    return fs / period_samples


def run_one(args):
    robot_name, lam, run_num, params_path = args
    try:
        body = modular_robots_v1.get(robot_name)
        hinges = body.find_modules_of_type(ActiveHinge)
        cpg, mp = active_hinges_to_cpg_network_structure_fully_connected(hinges)
        params = np.load(params_path)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params, cpg_network_structure=cpg,
            initial_state_uniform=math.sqrt(2) * 0.5, output_mapping=mp,
        )
        robot_obj = ModularRobot(body=body, brain=brain)
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
        scene.add_robot(robot_obj)
        sim_scene, _ = scene.to_simulation_scene()
        batch = make_standard_batch_parameters()
        batch.simulation_time = SIM_TIME
        model, mapping = scene_to_model(
            sim_scene, simulation_timestep=batch.simulation_timestep,
            cast_shadows=False, fast_sim=True,
        )
        data = mujoco.MjData(model)
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mapping)

        hinge_addrs = [int(model.jnt_qposadr[j]) for j in range(model.njnt)
                       if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
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
                    data=data, abstraction_to_mujoco_mapping=mapping, camera_views={},
                )
                sim_scene.handler.handle(sim_state, ctrl, control_step)
            mujoco.mj_step(model, data)
        traces = np.stack(traces, axis=0)
        fs = 1.0 / batch.simulation_timestep
        traces = traces[int(fs):]  # skip first 1s

        freqs = [_zero_cross_freq(traces[:, c], fs) for c in range(traces.shape[1])]
        freqs = [f for f in freqs if f > 0.01]
        if not freqs:
            return None
        return dict(robot=robot_name, lam=lam, run=run_num,
                    mean_freq=float(np.mean(freqs)),
                    min_freq=float(np.min(freqs)),
                    max_freq=float(np.max(freqs)))
    except Exception as e:
        print(f"ERR {robot_name} lam={lam} run={run_num}: {e}")
        return None


def main():
    base = os.path.expanduser("~/masteroppgave/revolve2/experiments/results/fully_connected_experiments")
    jobs = []
    for robot in ["spider", "gecko"]:
        for lam in [0, 1, 2, 3]:
            exp_dir = os.path.join(base, f"{robot}_ode_cpg_fully_connected_lambda{lam}_dragging")
            for npy in sorted(glob.glob(os.path.join(exp_dir, "best_params_run_*.npy")))[:5]:
                run_num = int(re.search(r"run_(\d+)", npy).group(1))
                jobs.append((robot, lam, run_num, npy))

    print(f"Computing frequencies for {len(jobs)} runs...")
    results = []
    with ProcessPoolExecutor(max_workers=4) as ex:
        futures = [ex.submit(run_one, j) for j in jobs]
        for fut in as_completed(futures):
            r = fut.result()
            if r:
                results.append(r)
    print(f"Done: {len(results)} successful.\n")

    agg = defaultdict(list)
    for r in results:
        agg[(r['robot'], r['lam'])].append(r)
    print(f"{'robot':<8}{'lam':<5}{'n':<4}{'f_mean':<9}{'f_min':<9}{'f_max':<9}")
    print("-" * 45)
    for (robot, l), rs in sorted(agg.items()):
        mean_f = np.mean([r['mean_freq'] for r in rs])
        min_f = np.mean([r['min_freq'] for r in rs])
        max_f = np.mean([r['max_freq'] for r in rs])
        print(f"{robot:<8}{l:<5}{len(rs):<4}{mean_f:<9.3f}{min_f:<9.3f}{max_f:<9.3f}")


if __name__ == "__main__":
    main()
