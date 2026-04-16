"""Look at evolved coupling weights from the gecko_spider runs."""
import os
import glob
import re
import numpy as np
from collections import defaultdict

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)

# How many internal vs coupling params per (robot, coupling)
def get_split(robot_name, coupling):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, _ = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "blf":
        cpg, _ = active_hinges_to_cpg_network_structure_blf(hinges, body)
    else:
        cpg, _ = active_hinges_to_cpg_network_structure_neighbor(hinges)
    n_int = cpg.num_cpgs
    n_coup = len(cpg.connections)
    return n_int, n_coup


base = os.path.expanduser("~/masteroppgave/revolve2/experiments/results/final_experiments")
robot = "gecko_spider"

print(f"{'coupling':<12}{'lam':<5}{'n_int':<6}{'n_coup':<7}{'int_mean':<10}{'int_std':<9}{'int_abs':<9}{'coup_mean':<11}{'coup_std':<10}{'coup_abs':<9}")
print("-" * 100)

for coupling in ["uncoupled", "neighbor", "blf"]:
    n_int, n_coup = get_split(robot, coupling)
    for lam in [0, 1, 2, 3]:
        d = os.path.join(base, f"gecko_spider_ode_cpg_{coupling}_lambda{lam}_dragging")
        files = sorted(glob.glob(os.path.join(d, "best_params_run_*.npy")))[:30]
        if not files:
            continue
        all_int = []
        all_coup = []
        for f in files:
            params = np.load(f)
            internal = params[:n_int]
            coupling_w = params[n_int:n_int + n_coup] if n_coup > 0 else np.array([])
            all_int.append(internal)
            if n_coup > 0:
                all_coup.append(coupling_w)
        all_int = np.concatenate(all_int)
        if all_coup:
            all_coup = np.concatenate(all_coup)
            cm = float(np.mean(all_coup))
            cs = float(np.std(all_coup))
            ca = float(np.mean(np.abs(all_coup)))
        else:
            cm = cs = ca = 0.0
        im = float(np.mean(all_int))
        std = float(np.std(all_int))
        ia = float(np.mean(np.abs(all_int)))
        print(f"{coupling:<12}{lam:<5}{n_int:<6}{n_coup:<7}{im:<10.3f}{std:<9.3f}{ia:<9.3f}{cm:<11.3f}{cs:<10.3f}{ca:<9.3f}")
