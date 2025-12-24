"""
Database schema for evolution experiments with contact metrics.
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


class Experiment(Base, HasId):
    """Experiment configuration and metadata."""

    __tablename__ = "experiment"

    # Experiment name/description
    name: orm.Mapped[str] = orm.mapped_column(nullable=False)

    # Configuration parameters
    robot_name: orm.Mapped[str] = orm.mapped_column(nullable=False)
    contact_metric: orm.Mapped[str] = orm.mapped_column(nullable=False)
    lambda_penalty: orm.Mapped[float] = orm.mapped_column(nullable=False)
    simulation_time: orm.Mapped[int] = orm.mapped_column(nullable=False)
    num_generations: orm.Mapped[int] = orm.mapped_column(nullable=False)
    initial_std: orm.Mapped[float] = orm.mapped_column(nullable=False)
    param_bounds_min: orm.Mapped[float] = orm.mapped_column(nullable=False)
    param_bounds_max: orm.Mapped[float] = orm.mapped_column(nullable=False)
    # Random seed for reproducibility
    rng_seed: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Optional notes
    notes: orm.Mapped[str | None] = orm.mapped_column(nullable=True, default=None)

    # Terrain friction (default 1.0)
    terrain_friction: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=1.0)

    # Fitness formula used ("linear", "power", "exponential")
    fitness_formula: orm.Mapped[str | None] = orm.mapped_column(nullable=True, default="exponential")

    # Warmup time in seconds (metrics only recorded after warmup)
    warmup_time: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=0.0)


@dataclass
class Genotype(Base, HasId):
    """CPG parameters genotype."""

    __tablename__ = "genotype"

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
    def from_parameters(cls, params: npt.NDArray[np.float64]) -> Genotype:
        """Create genotype from numpy array."""
        serialized = ";".join(str(x) for x in params)
        return cls(serialized_parameters=serialized)


@dataclass
class Individual(Base, HasId):
    """An individual in a population with full metrics."""

    __tablename__ = "individual"

    # Foreign keys
    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("population.id"), nullable=False, init=False
    )
    genotype_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("genotype.id"), nullable=False, init=False
    )

    # Relationships
    genotype: orm.Mapped[Genotype] = orm.relationship()

    # Index in population
    population_index: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Fitness value
    fitness: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # All metrics (for analysis)
    distance: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    contact_m1: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    contact_m2: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    cost_of_transport: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    stability: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    straightness: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    energy: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    path_length: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)


@dataclass
class Population(Base, HasId):
    """A population of individuals."""

    __tablename__ = "population"

    # Individuals in this population
    individuals: orm.Mapped[list[Individual]] = orm.relationship(
        default_factory=list
    )


@dataclass
class Generation(Base, HasId):
    """A single generation in the evolution."""

    __tablename__ = "generation"

    # Foreign keys
    experiment_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("experiment.id"), nullable=False, init=False
    )
    population_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("population.id"), nullable=False, init=False
    )

    # Relationships
    experiment: orm.Mapped[Experiment] = orm.relationship()
    population: orm.Mapped[Population] = orm.relationship()

    # Generation number
    generation_index: orm.Mapped[int] = orm.mapped_column(nullable=False)

    # Aggregated statistics for quick analysis
    fitness_mean: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_std: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_max: orm.Mapped[float] = orm.mapped_column(nullable=False)
    fitness_min: orm.Mapped[float] = orm.mapped_column(nullable=False)

    # Best individual metrics
    best_distance: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    best_contact_m1: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
    best_cot: orm.Mapped[float | None] = orm.mapped_column(nullable=True, default=None)
