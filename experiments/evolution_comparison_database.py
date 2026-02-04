"""
Database schema for CPG comparison experiments.

Stores experiments comparing:
- Controllers: ode_cpg vs sine
- Coupling modes: uncoupled, neighbor, blf
- Robots: spider, gecko
- Penalty: lambda=0, lambda=1
"""

from __future__ import annotations
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
import sqlalchemy
import sqlalchemy.orm as orm

from revolve2.experimentation.database import HasId


class Base(orm.MappedAsDataclass, orm.DeclarativeBase):
    """Base class for all SQLAlchemy models."""
    pass


class ComparisonExperiment(Base, HasId):
    """Experiment configuration for comparison experiments."""

    __tablename__ = "comparison_experiment"

    # Robot configuration
    robot_name: orm.Mapped[str] = orm.mapped_column(nullable=False)

    # Controller type: "ode_cpg" or "sine"
    controller_type: orm.Mapped[str] = orm.mapped_column(nullable=False)

    # Coupling mode: "uncoupled", "neighbor", "blf"
    coupling_mode: orm.Mapped[str] = orm.mapped_column(nullable=False)

    # Contact penalty lambda (0 = no penalty, 1 = penalty)
    lambda_penalty: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # Simulation settings
    simulation_time: orm.Mapped[float] = orm.mapped_column(nullable=False)
    num_generations: orm.Mapped[int] = orm.mapped_column(nullable=False)
    population_size: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Parameter bounds
    param_bounds_min: orm.Mapped[float] = orm.mapped_column(nullable=False)
    param_bounds_max: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # Random seed
    rng_seed: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Run number (for multiple seeds)
    run_number: orm.Mapped[int] = orm.mapped_column(nullable=False, default=1)

    # Penalty type: "dragging", "height", "min_height", "combined"
    penalty_type: orm.Mapped[str | None] = orm.mapped_column(nullable=True, default="dragging")

    # Controller-specific settings
    frequency: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=1.0)
    coupling_strength: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=0.5)

    # Number of parameters
    num_parameters: orm.Mapped[int | None] = orm.mapped_column(nullable=True, default=None)

    # Number of hinges
    num_hinges: orm.Mapped[int | None] = orm.mapped_column(nullable=True, default=None)


@dataclass
class ComparisonGenotype(Base, HasId):
    """CPG/Sine parameters genotype."""

    __tablename__ = "comparison_genotype"

    # Serialized parameters (semicolon-separated)
    serialized_parameters: orm.Mapped[str] = orm.mapped_column(
        "serialized_parameters", nullable=False
    )

    @property
    def parameters(self) -> npt.NDArray[np.float64]:
        """Get parameters as numpy array."""
        if not self.serialized_parameters or not self.serialized_parameters.strip():
            return np.array([], dtype=np.float64)
        try:
            return np.array(
                [float(x) for x in self.serialized_parameters.split(";")]
            )
        except ValueError as e:
            raise ValueError(f"Invalid serialized_parameters format: {self.serialized_parameters}") from e

    @classmethod
    def from_parameters(cls, params: npt.NDArray[np.float64]) -> ComparisonGenotype:
        """Create genotype from numpy array."""
        serialized = ";".join(str(x) for x in params)
        return cls(serialized_parameters=serialized)


@dataclass
class ComparisonIndividual(Base, HasId):
    """An individual in a population with full metrics."""

    __tablename__ = "comparison_individual"

    # Foreign keys
    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("comparison_population.id"), nullable=False, init=False
    )
    genotype_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("comparison_genotype.id"), nullable=False, init=False
    )

    # Relationships
    genotype: orm.Mapped[ComparisonGenotype] = orm.relationship()

    # Index in population
    population_index: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Fitness value
    fitness: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # Core metrics
    distance: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    dragging: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)  # M1
    cost_of_transport: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Final position
    final_x: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    final_y: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Core height metrics
    avg_core_height: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    min_core_height: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)


@dataclass
class ComparisonPopulation(Base, HasId):
    """A population of individuals."""

    __tablename__ = "comparison_population"

    # Individuals in this population
    individuals: orm.Mapped[list[ComparisonIndividual]] = orm.relationship(
        default_factory=list
    )


@dataclass
class ComparisonGeneration(Base, HasId):
    """A single generation in the evolution."""

    __tablename__ = "comparison_generation"

    # Foreign keys
    experiment_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("comparison_experiment.id"), nullable=False, init=False
    )
    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("comparison_population.id"), nullable=False, init=False
    )

    # Relationships
    experiment: orm.Mapped[ComparisonExperiment] = orm.relationship()
    population: orm.Mapped[ComparisonPopulation] = orm.relationship()

    # Generation number
    generation_index: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Fitness statistics
    fitness_mean: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_std: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_max: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_min: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # Distance statistics
    distance_mean: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    distance_max: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Dragging (M1) statistics
    dragging_mean: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # CoT statistics
    cot_mean: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Core height statistics
    height_mean: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Best ever (cumulative)
    best_ever_fitness: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    best_ever_distance: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)

    # Generation time
    time_seconds: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
