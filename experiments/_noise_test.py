"""Spider phase-noise robustness test: BLF vs phi/uncoupled.

Adds Gaussian noise to phases every control step. Measures distance/EM/drag
at increasing noise levels. Saves PDF with two plots: dist-vs-sigma and EM-vs-sigma.
"""
import os, sys, math, csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
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
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from bonardi_brain import (
    BrainBonardi, BrainBonardiInstance, bonardi_structure_from_cpg_structure,
)


# Monkey-patch BrainBonardiInstance.control to inject phase noise. We'll wrap
# the original method so that after the RK4 step, we add Gaussian noise to phases.
_original_control = BrainBonardiInstance.control
_NOISE_SIGMA = 0.0
_NOISE_RNG = np.random.RandomState(0)


def _noisy_control(self, dt, sensor_state, control_interface):
    _original_control(self, dt, sensor_state, control_interface)
    if _NOISE_SIGMA > 0:
        n = self._phi.size
        self._phi = self._phi + _NOISE_SIGMA * _NOISE_RNG.randn(n)
        self._phi = (self._phi + np.pi) % (2.0 * np.pi) - np.pi


BrainBonardiInstance.control = _noisy_control


def run(robot_name, coupling, evolve_phi0, params_path, sim_time, sigma, trial_seed):
    global _NOISE_SIGMA, _NOISE_RNG
    _NOISE_SIGMA = sigma
    _NOISE_RNG = np.random.RandomState(trial_seed)

    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    ks = bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0)
    params = np.load(params_path)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks, output_mapping=mp,
        nu_hz=0.5, w=1.0,
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
    batch.simulation_time = sim_time
    model, mj_mapping = scene_to_model(
        sim_scene, simulation_timestep=batch.simulation_timestep,
        cast_shadows=False, fast_sim=True,
    )
    data = mujoco.MjData(model)
    core = get_robot_core_body_id(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)
    init_pos = data.xpos[core].copy()
    prev_x, prev_y = init_pos[0], init_pos[1]
    path = 0.0
    total = 0
    drag = 0
    while data.time < sim_time:
        total += 1
        contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
        if any(rg in non_foot for rg, *_ in contacts):
            drag += 1
        if data.time >= last_ctrl + cstep:
            last_ctrl = math.floor(data.time / cstep) * cstep
            ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(ss, ctrl, cstep)
            cx, cy = data.xpos[core][0], data.xpos[core][1]
            path += math.sqrt((cx - prev_x) ** 2 + (cy - prev_y) ** 2)
            prev_x, prev_y = cx, cy
        mujoco.mj_step(model, data)
    fp = data.xpos[core].copy()
    disp = math.sqrt((fp[0] - init_pos[0]) ** 2 + (fp[1] - init_pos[1]) ** 2)
    em = disp / path if path > 1e-6 else 0.0
    return float(disp), float(em), 100.0 * drag / total


CASES = [
    ("BLF",       "blf",       False,
     os.path.expanduser("~/fox_results/spider/spider_bonardi_base_blf_lambda0_nu0.5_w1.0/best_params_run_5.npy")),
    ("phi/unc",   "uncoupled", True,
     os.path.expanduser("~/fox_results/spider/spider_bonardi_phi_uncoupled_lambda0_nu0.5_w1.0/best_params_run_3.npy")),
]
SIGMAS = [0.0, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6]
N_TRIALS = 5
SIM_TIME = 30.0

rows = []
for label, coupling, phi0, fp in CASES:
    for sigma in SIGMAS:
        for trial in range(N_TRIALS):
            disp, em, drag = run("spider", coupling, phi0, fp, SIM_TIME, sigma, trial)
            rows.append((label, sigma, trial, disp, em, drag))
            print(f"{label:10} sigma={sigma:.2f} trial={trial} dist={disp:.2f} EM={em:.3f} drag={drag:.1f}%")

CSV = os.path.expanduser("~/noise_test_spider.csv")
with open(CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["controller", "sigma", "trial", "distance", "em", "drag_pct"])
    w.writerows(rows)
print("wrote", CSV)

# Aggregate and plot
import pandas as pd
df = pd.read_csv(CSV)

OUT = os.path.expanduser("~/noise_test_spider.pdf")
with PdfPages(OUT) as pdf:
    for metric, ylabel, title in [
        ("distance", "Distance (m)", "Distance vs phase noise (spider, 30 s sim)"),
        ("em",       "EM",            "EM vs phase noise"),
        ("drag_pct", "Dragging (%)",  "Dragging vs phase noise"),
    ]:
        fig, ax = plt.subplots(figsize=(8, 5))
        for label in df.controller.unique():
            sub = df[df.controller == label]
            agg = sub.groupby("sigma")[metric].agg(["mean", "std"]).reset_index()
            ax.errorbar(agg["sigma"], agg["mean"], yerr=agg["std"],
                        marker="o", capsize=3, label=label, lw=1.5)
        ax.set_xlabel("Phase noise sigma (rad / control step)", fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.legend()
        ax.grid(alpha=0.3)
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)

print("wrote", OUT)
