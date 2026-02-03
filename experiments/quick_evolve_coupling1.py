"""Quick 15-gen evolution for coupling=1 to get decent params for visualization."""

import math
import numpy as np
from concurrent.futures import ProcessPoolExecutor, as_completed
import cma

from run_fixed_coupling_1 import evaluate_fixed_coupling_params, COUPLING_STRENGTH
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1

def main():
    print(f"\nQuick evolution for coupling={COUPLING_STRENGTH}")

    body = modular_robots_v1.get("spider")
    n_hinges = len(body.find_modules_of_type(ActiveHinge))

    # Initial mean
    initial_mean = []
    for _ in range(n_hinges):
        initial_mean.extend([0.5, 0.0, 0.0])  # amp, phase, offset
    initial_mean.append(2.0)  # frequency

    # Bounds
    lower_bounds = []
    upper_bounds = []
    for _ in range(n_hinges):
        lower_bounds.extend([0.0, -math.pi, -0.5])
        upper_bounds.extend([1.0, math.pi, 0.5])
    lower_bounds.append(0.5)
    upper_bounds.append(3.0)

    options = cma.CMAOptions()
    options.set("seed", 42)
    options.set("bounds", [lower_bounds, upper_bounds])

    opt = cma.CMAEvolutionStrategy(initial_mean, 0.5, options)

    best_fitness = float("-inf")
    best_params = None

    for gen in range(15):
        solutions = opt.ask()

        results = []
        with ProcessPoolExecutor(max_workers=4) as executor:
            futures = {executor.submit(evaluate_fixed_coupling_params, np.array(s)): i
                      for i, s in enumerate(solutions)}
            results = [None] * len(solutions)
            for future in as_completed(futures):
                idx = futures[future]
                results[idx] = future.result()

        fitnesses = [r.fitness for r in results]
        opt.tell(solutions, [-f for f in fitnesses])

        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])

        print(f"Gen {gen+1:2d}: best={max(fitnesses):.2f}m, overall={best_fitness:.2f}m")

    # Save for visualization
    np.savez(
        "quick_coupling1_params.npz",
        best_params=best_params,
        best_fitness=best_fitness,
        coupling_strength=COUPLING_STRENGTH,
    )

    print(f"\nBest: {best_fitness:.2f}m")
    print(f"Params saved to quick_coupling1_params.npz")

    # Print params for copy-paste
    print("\n# Best parameters:")
    print("params = np.array([")
    for i in range(n_hinges):
        print(f"    {best_params[3*i]:.3f}, {best_params[3*i+1]:+.3f}, {best_params[3*i+2]:+.3f},  # Hinge {i}")
    print("])")
    print(f"frequency = {best_params[-1]:.3f}")

if __name__ == "__main__":
    main()
