"""5-degree slope robustness test: BLF vs phi/uncoupled.

Implementation: keep the floor flat, tilt gravity. With +x being uphill, gravity
gets a -x component of g*sin(theta). Equivalent to walking on an incline.

Tests at 0, 2.5, 5, 7.5, 10 degrees of uphill grade.
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
from bonardi_brain import BrainBonardi, bonardi_structure_from_cpg_structure


def run(robot_name, coupling, evolve_phi0, params_path, sim_time, slope_deg):
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

    # Tilt gravity such that robot's natural walking direction (-x) becomes uphill.
    # That requires gravity to have a +x component (pulling robot back toward +x = downhill).
    theta = math.radians(slope_deg)
    g = 9.81
    model.opt.gravity[0] = +g * math.sin(theta)
    model.opt.gravity[1] = 0.0
    model.opt.gravity[2] = -g * math.cos(theta)

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
    # Use signed x-displacement: uphill progress (+x) vs downhill backslide (-x)
    dx = fp[0] - init_pos[0]
    dy = fp[1] - init_pos[1]
    disp = math.sqrt(dx * dx + dy * dy)
    em = disp / path if path > 1e-6 else 0.0
    return float(disp), float(dx), float(em), 100.0 * drag / total


CASES = [
    ("BLF",      "blf",       False,
     os.path.expanduser("~/fox_results/spider/spider_bonardi_base_blf_lambda0_nu0.5_w1.0/best_params_run_5.npy")),
    ("neighbor", "neighbor",  False,
     os.path.expanduser("~/fox_results/spider/spider_bonardi_base_neighbor_lambda0_nu0.5_w1.0/best_params_run_3.npy")),
    ("phi/unc",  "uncoupled", True,
     os.path.expanduser("~/fox_results/spider/spider_bonardi_phi_uncoupled_lambda0_nu0.5_w1.0/best_params_run_3.npy")),
]
SLOPES = [0.0, 2.5, 5.0, 7.5, 10.0]
SIM_TIME = 30.0

rows = []
for label, coupling, phi0, fp in CASES:
    for slope in SLOPES:
        disp, dx, em, drag = run("spider", coupling, phi0, fp, SIM_TIME, slope)
        rows.append((label, slope, disp, dx, em, drag))
        print(f"{label:10} slope={slope:4.1f}deg  dist={disp:5.2f}m  dx={dx:+5.2f}m  EM={em:.3f}  drag={drag:.1f}%")

CSV = os.path.expanduser("~/slope_test_spider.csv")
with open(CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["controller", "slope_deg", "distance", "dx_uphill", "em", "drag_pct"])
    w.writerows(rows)
print("wrote", CSV)

# Plot
import pandas as pd
df = pd.read_csv(CSV)
OUT = os.path.expanduser("~/slope_test_spider.pdf")
with PdfPages(OUT) as pdf:
    for metric, ylabel, title in [
        ("dx_uphill", "Uphill displacement (m)", "Uphill progress vs slope"),
        ("distance",  "Total displacement (m)",  "Total displacement vs slope"),
        ("em",        "EM",                       "EM vs slope"),
        ("drag_pct",  "Dragging (%)",             "Dragging vs slope"),
    ]:
        fig, ax = plt.subplots(figsize=(8, 5))
        for label in df.controller.unique():
            sub = df[df.controller == label]
            ax.plot(sub["slope_deg"], sub[metric], marker="o", label=label, lw=1.5)
        ax.set_xlabel("Slope (degrees uphill)", fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.axhline(0, color="gray", lw=0.5, ls="--")
        ax.legend()
        ax.grid(alpha=0.3)
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)
print("wrote", OUT)
