"""
Evaluator for evolutionary optimization with contact detection.
"""

import math
import sys
from pathlib import Path

# Ensure imports work from any directory
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import numpy.typing as npt
from concurrent.futures import ProcessPoolExecutor, as_completed

from contact_detection import simulate_with_contact_detection, LocomotionMetrics
import evolution_config as config


def calculate_fitness(
    distance: float,
    contact: float,
    lambda_penalty: float,
    fitness_formula: str = "exponential",
) -> float:
    """
    Calculate fitness using the specified formula.

    Formulas:
        - "exponential": distance * exp(-λ * contact)
        - "power": distance * (1 - contact)^λ
        - "linear": distance * (1 - λ * contact)

    Args:
        distance: XY displacement in meters
        contact: Contact metric (0-1)
        lambda_penalty: Penalty weight (λ)
        fitness_formula: Which formula to use

    Returns:
        Fitness value (can be negative for linear formula with high contact)
    """
    # Clamp contact to [0, 1] for numerical safety
    contact = max(0.0, min(1.0, contact))

    if fitness_formula == "exponential":
        return distance * math.exp(-lambda_penalty * contact)
    elif fitness_formula == "power":
        # (1 - contact)^λ: when λ=0, result is always 1 (pure distance)
        return distance * math.pow(1 - contact, lambda_penalty)
    elif fitness_formula == "linear":
        return distance * (1 - lambda_penalty * contact)
    else:
        raise ValueError(f"Unknown fitness formula: {fitness_formula}")


def evaluate_single(
    params: npt.NDArray[np.float64],
    robot_name: str,
    simulation_time: int,
    contact_metric: str,
    lambda_penalty: float,
    terrain_friction: float = 1.0,
    fitness_formula: str = "exponential",
    warmup_time: float = 0.0,
    no_coupling: bool = False,
) -> tuple[float, LocomotionMetrics]:
    """
    Evaluate a single robot with given CPG parameters.

    Returns fitness and full metrics.
    """
    try:
        tracker, metrics = simulate_with_contact_detection(
            robot_name=robot_name,
            simulation_time=simulation_time,
            verbose=False,
            cpg_params=params.tolist(),
            headless=True,
            terrain_friction=terrain_friction,
            warmup_time=warmup_time,
            no_coupling=no_coupling,
        )

        # Select contact metric
        if contact_metric.lower() == "m1":
            contact = metrics.contact_metric_1
        elif contact_metric.lower() == "m2":
            contact = metrics.contact_metric_2
        else:
            raise ValueError(f"Unknown contact metric: {contact_metric}")

        # Calculate fitness using selected formula
        fitness = calculate_fitness(distance=metrics.distance, contact=contact,
                                    lambda_penalty=lambda_penalty, fitness_formula=fitness_formula)

        return fitness, metrics

    except Exception as e:
        print(f"Evaluation error: {e}")
        # Return very negative fitness so crashed robots don't get selected as "best"
        # when using linear formula that can produce negative values
        return -1e9, None


def _evaluate_wrapper(args: tuple) -> tuple[int, float, LocomotionMetrics]:
    """Wrapper for parallel evaluation."""
    idx, params, robot_name, sim_time, contact_metric, lambda_penalty, terrain_friction, fitness_formula, warmup_time, no_coupling = args
    fitness, metrics = evaluate_single(
        params, robot_name, sim_time, contact_metric, lambda_penalty, terrain_friction, fitness_formula, warmup_time, no_coupling
    )
    return idx, fitness, metrics


class Evaluator:
    """Evaluator class for CMA-ES optimization with contact detection."""

    def __init__(
        self,
        robot_name: str = None,
        simulation_time: int = None,
        contact_metric: str = None,
        lambda_penalty: float = None,
        num_workers: int = None,
        terrain_friction: float = 1.0,
        fitness_formula: str = "exponential",
        warmup_time: float = None,
        no_coupling: bool = False,
    ):
        """
        Initialize evaluator with configuration.

        Parameters are loaded from config file if not provided.

        Args:
            fitness_formula: "exponential", "power", or "linear"
            warmup_time: Time in seconds for robot to settle before measuring metrics.
            no_coupling: If True, use only internal CPG weights (no external coupling).
        """
        self.robot_name = robot_name or config.ROBOT_NAME
        self.simulation_time = simulation_time or config.SIMULATION_TIME
        self.contact_metric = contact_metric or config.CONTACT_METRIC
        self.lambda_penalty = lambda_penalty if lambda_penalty is not None else config.LAMBDA_PENALTY
        self.num_workers = num_workers or config.NUM_SIMULATORS
        self.terrain_friction = terrain_friction
        self.fitness_formula = fitness_formula
        self.warmup_time = warmup_time if warmup_time is not None else config.WARMUP_TIME
        self.no_coupling = no_coupling

        # Store all metrics from last evaluation for analysis
        self.last_metrics: list[LocomotionMetrics] = []

    def evaluate(
        self,
        solutions: list[npt.NDArray[np.float64]],
    ) -> npt.NDArray[np.float64]:
        """
        Evaluate multiple solutions (CPG parameter sets).

        Args:
            solutions: List of CPG parameter arrays

        Returns:
            Array of fitness values
        """
        fitnesses = np.zeros(len(solutions))
        self.last_metrics = [None] * len(solutions)

        if self.num_workers <= 1:
            # Sequential evaluation
            for i, params in enumerate(solutions):
                fitness, metrics = evaluate_single(
                    params,
                    self.robot_name,
                    self.simulation_time,
                    self.contact_metric,
                    self.lambda_penalty,
                    self.terrain_friction,
                    self.fitness_formula,
                    self.warmup_time,
                    self.no_coupling,
                )
                fitnesses[i] = fitness
                self.last_metrics[i] = metrics
        else:
            # Parallel evaluation using ProcessPoolExecutor
            args_list = [
                (i, params, self.robot_name, self.simulation_time,
                 self.contact_metric, self.lambda_penalty, self.terrain_friction, self.fitness_formula, self.warmup_time, self.no_coupling)
                for i, params in enumerate(solutions)
            ]

            with ProcessPoolExecutor(max_workers=self.num_workers) as executor:
                futures = [executor.submit(_evaluate_wrapper, args) for args in args_list]

                for future in as_completed(futures):
                    idx, fitness, metrics = future.result()
                    fitnesses[idx] = fitness
                    self.last_metrics[idx] = metrics

        return fitnesses

    def get_best_metrics(self, fitnesses: npt.NDArray[np.float64]) -> LocomotionMetrics | None:
        """Get metrics for the best individual in last evaluation."""
        if not self.last_metrics:
            return None
        best_idx = np.argmax(fitnesses)
        return self.last_metrics[best_idx]

    def get_stats(self, fitnesses: npt.NDArray[np.float64]) -> dict:
        """Get statistics from last evaluation."""
        best_metrics = self.get_best_metrics(fitnesses)

        stats = {
            "fitness_mean": float(np.mean(fitnesses)),
            "fitness_std": float(np.std(fitnesses)),
            "fitness_max": float(np.max(fitnesses)),
            "fitness_min": float(np.min(fitnesses)),
        }

        if best_metrics:
            stats.update({
                "best_distance": best_metrics.distance,
                "best_contact_m1": best_metrics.contact_metric_1,
                "best_contact_m2": best_metrics.contact_metric_2,
                "best_cot": best_metrics.cost_of_transport,
                "best_straightness": best_metrics.straightness,
                "best_stability": best_metrics.stability_combined,
            })

        return stats


def fitness_function(
    distance: float,
    contact: float,
    lambda_penalty: float = 0.5,
    fitness_formula: str = "exponential",
) -> float:
    """
    Calculate fitness from distance and contact.

    Convenience wrapper around calculate_fitness().

    Args:
        distance: XY displacement in meters
        contact: Contact metric (0-1)
        lambda_penalty: Penalty weight for contact
        fitness_formula: "exponential", "power", or "linear"

    Returns:
        Fitness value
    """
    return calculate_fitness(distance, contact, lambda_penalty, fitness_formula)
