"""Quick frequency comparison: spider bounds [-1,1] vs [-2,2]."""
import os, math, numpy as np
import sys
sys.path.insert(0, os.path.expanduser("~/revolve2/experiments"))
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)


def get_struct(cpl, body, hinges):
    if cpl == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if cpl == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    return active_hinges_to_cpg_network_structure_neighbor(hinges)


def integrate(W, x0, dt, n):
    x = x0.copy()
    h = np.zeros((n, len(x)))
    for i in range(n):
        h[i] = x
        a1 = W @ x
        a2 = W @ (x + dt / 2 * a1)
        a3 = W @ (x + dt / 2 * a2)
        a4 = W @ (x + dt * a3)
        x = x + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        x = np.clip(x, -1, 1)
    return h


def freq_from_signal(s, dt):
    crossings = 0
    for i in range(1, len(s)):
        if s[i - 1] < 0 and s[i] >= 0:
            crossings += 1
    duration = len(s) * dt
    return crossings / duration


def get_freqs(base_dir):
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    n_h = len(hinges)
    sim_time = 30.0
    dt = 1.0 / 20
    n_steps = int(sim_time / dt)
    res = {}
    for cpl in ["uncoupled", "neighbor", "blf"]:
        for lam in [0, 1, 2, 3]:
            d = os.path.join(base_dir, "spider_ode_cpg_%s_lambda%d_dragging" % (cpl, lam))
            if not os.path.isdir(d):
                continue
            cpg, _ = get_struct(cpl, body, hinges)
            x0 = cpg.make_uniform_state(math.sqrt(2) * 0.5)
            run_freqs = []
            for r in range(1, 31):
                npy = os.path.join(d, "best_params_run_%d.npy" % r)
                if not os.path.exists(npy):
                    continue
                params = np.load(npy)
                W = cpg.make_connection_weights_matrix_from_params(list(params))
                hist = integrate(W, x0, dt, n_steps)
                outputs = hist[:, :n_h]
                freqs = [freq_from_signal(outputs[:, h], dt) for h in range(n_h)]
                run_freqs.append(np.mean(freqs))
            if run_freqs:
                res[(cpl, lam)] = (np.mean(run_freqs), np.std(run_freqs))
    return res


p11 = get_freqs(os.path.expanduser("~/masteroppgave/revolve2/experiments/results/bounds_compare/pop50_local"))
p22 = get_freqs(os.path.expanduser("~/masteroppgave/revolve2/experiments/results/bounds_compare/bounds2_local"))

print("=== SPIDER Frequency (Hz) - bounds[-1,1] vs bounds[-2,2] ===")
print("%-12s %3s   %14s   %14s   %s" % ("Coupling", "lam", "Freq[-1,1]", "Freq[-2,2]", "change"))
print("-" * 65)
for cpl in ["uncoupled", "neighbor", "blf"]:
    label = {"uncoupled": "No coupling", "neighbor": "Neighbour", "blf": "Structured"}[cpl]
    for lam in [0, 1, 2, 3]:
        k = (cpl, lam)
        if k in p11 and k in p22:
            f1, s1 = p11[k]
            f2, s2 = p22[k]
            ch = ((f2 - f1) / f1) * 100 if f1 > 0 else 0
            print("%-12s  %d    %5.2f +- %4.2f   %5.2f +- %4.2f   %+.0f%%" % (label, lam, f1, s1, f2, s2, ch))
