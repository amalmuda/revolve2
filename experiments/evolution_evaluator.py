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
from blf import expand_blf_sym_params, get_blf_symmetry_expansion_info
from core_centric import expand_cc_sym_params, get_cc_sym_expansion_info
from revolve2.standards import modular_robots_v1


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
    cross_leg_coupling: bool = False,
    diagonal_coupling: bool = False,
    full_cross_leg_coupling: bool = False,
    fully_connected_coupling: bool = False,
    blf_coupling: bool = False,
    blf_symmetry: bool = False,
    core_centric_coupling: bool = False,
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
            cross_leg_coupling=cross_leg_coupling,
            diagonal_coupling=diagonal_coupling,
            full_cross_leg_coupling=full_cross_leg_coupling,
            fully_connected_coupling=fully_connected_coupling,
            blf_coupling=blf_coupling,
            blf_symmetry=blf_symmetry,
            core_centric_coupling=core_centric_coupling,
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
    idx, params, robot_name, sim_time, contact_metric, lambda_penalty, terrain_friction, fitness_formula, warmup_time, no_coupling, cross_leg_coupling, diagonal_coupling, full_cross_leg_coupling, fully_connected_coupling, blf_coupling, blf_symmetry, core_centric_coupling = args
    fitness, metrics = evaluate_single(
        params, robot_name, sim_time, contact_metric, lambda_penalty, terrain_friction, fitness_formula, warmup_time, no_coupling, cross_leg_coupling, diagonal_coupling, full_cross_leg_coupling, fully_connected_coupling, blf_coupling, blf_symmetry, core_centric_coupling
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
        cross_leg_coupling: bool = False,
        diagonal_coupling: bool = False,
        full_cross_leg_coupling: bool = False,
        fully_connected_coupling: bool = False,
        blf_coupling: bool = False,
        blf_symmetry: bool = False,
        core_centric_coupling: bool = False,
        core_centric_symmetry: bool = False,
        param_bounds: tuple[float, float] = (-1.0, 1.0),
    ):
        """
        Initialize evaluator with configuration.

        Parameters are loaded from config file if not provided.

        Args:
            fitness_formula: "exponential", "power", or "linear"
            warmup_time: Time in seconds for robot to settle before measuring metrics.
            no_coupling: If True, use only internal CPG weights (no external coupling).
            cross_leg_coupling: If True, use cross-leg coupling (neighbor + shoulder-to-shoulder).
            diagonal_coupling: If True, use diagonal coupling (neighbor + true diagonal leg connections).
            full_cross_leg_coupling: If True, use full cross-leg coupling (complete graph among all 4 leg hinges).
            fully_connected_coupling: If True, every hinge connected to every other hinge (complete graph, 36 params for spider).
            blf_coupling: If True, use BLF (Body/Limb Finder) coupling from EPFL paper.
            blf_symmetry: If True, use BLF-SYM (symmetric limbs share parameters).
            core_centric_coupling: If True, use Core-Centric coupling (bio-inspired reduced network).
            core_centric_symmetry: If True, use CC-SYM (Core-Centric with symmetry).
            param_bounds: Bounds for external coupling weights (used for BLF-SYM/CC-SYM expansion).
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
        self.cross_leg_coupling = cross_leg_coupling
        self.diagonal_coupling = diagonal_coupling
        self.full_cross_leg_coupling = full_cross_leg_coupling
        self.fully_connected_coupling = fully_connected_coupling
        self.blf_coupling = blf_coupling
        self.blf_symmetry = blf_symmetry
        self.core_centric_coupling = core_centric_coupling
        self.core_centric_symmetry = core_centric_symmetry
        self.param_bounds = param_bounds

        # Store all metrics from last evaluation for analysis
        self.last_metrics: list[LocomotionMetrics] = []

        # Pre-compute BLF-SYM expansion info if needed
        self.blf_sym_expansion_info = None
        if blf_coupling and blf_symmetry:
            body = modular_robots_v1.get(self.robot_name)
            self.blf_sym_expansion_info = get_blf_symmetry_expansion_info(
                body, external_weight_bounds=param_bounds
            )

        # Pre-compute CC-SYM expansion info if needed
        self.cc_sym_expansion_info = None
        if core_centric_symmetry:
            body = modular_robots_v1.get(self.robot_name)
            self.cc_sym_expansion_info = get_cc_sym_expansion_info(body)

    def _expand_params_if_needed(self, params: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """Expand reduced BLF-SYM or CC-SYM params to full params if needed."""
        if self.cc_sym_expansion_info is not None:
            body = modular_robots_v1.get(self.robot_name)
            full_params = expand_cc_sym_params(
                params.tolist(), body, external_weight_bounds=self.param_bounds
            )
            return np.array(full_params, dtype=np.float64)
        elif self.blf_sym_expansion_info is not None:
            body = modular_robots_v1.get(self.robot_name)
            full_params = expand_blf_sym_params(
                params.tolist(), body, external_weight_bounds=self.param_bounds
            )
            return np.array(full_params, dtype=np.float64)
        return params

    def evaluate(
        self,
        solutions: list[npt.NDArray[np.float64]],
    ) -> npt.NDArray[np.float64]:
        """
        Evaluate multiple solutions (CPG parameter sets).

        Args:
            solutions: List of CPG parameter arrays (reduced if BLF-SYM)

        Returns:
            Array of fitness values
        """
        fitnesses = np.zeros(len(solutions))
        self.last_metrics = [None] * len(solutions)

        # Expand reduced params if using BLF-SYM
        expanded_solutions = [self._expand_params_if_needed(params) for params in solutions]

        if self.num_workers <= 1:
            # Sequential evaluation
            for i, params in enumerate(expanded_solutions):
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
                    self.cross_leg_coupling,
                    self.diagonal_coupling,
                    self.full_cross_leg_coupling,
                    self.fully_connected_coupling,
                    self.blf_coupling,
                    False,  # Don't pass blf_symmetry to simulation - we already expanded
                    self.core_centric_coupling,
                )
                fitnesses[i] = fitness
                self.last_metrics[i] = metrics
        else:
            # Parallel evaluation using ProcessPoolExecutor
            args_list = [
                (i, params, self.robot_name, self.simulation_time,
                 self.contact_metric, self.lambda_penalty, self.terrain_friction, self.fitness_formula, self.warmup_time, self.no_coupling, self.cross_leg_coupling, self.diagonal_coupling, self.full_cross_leg_coupling, self.fully_connected_coupling, self.blf_coupling, False, self.core_centric_coupling)  # Don't pass blf_symmetry
                for i, params in enumerate(expanded_solutions)
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
