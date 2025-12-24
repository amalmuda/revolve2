"""
Evolution Configuration File
============================
Modify these parameters to customize your evolutionary run.
"""

# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

# Robot to evolve: "spider", "gecko", "ant", "salamander", "snake", "turtle",
#                  "babya", "babyb", "blokky", "garrix", "insect", "linkin",
#                  "longleg", "penguin", "pentapod", "queen", "squarish",
#                  "stingray", "tinlicker", "ww", "zappa", "park"
ROBOT_NAME: str = "spider"

# =============================================================================
# FITNESS FUNCTION CONFIGURATION
# =============================================================================

# Contact metric to use in fitness function:
#   "m1" = Percentage of timesteps with any body (non-foot) contact
#   "m2" = Percentage of timesteps with body contact OR foot slip
CONTACT_METRIC: str = "m1"

# Penalty weight (lambda) for contact in fitness function
# Formula depends on --fitness-formula argument in evolve.py:
#   linear: fitness = distance * (1 - λ * contact)
#   power:  fitness = distance * (1 - contact)^λ
#   exponential: fitness = distance * exp(-λ * contact)
LAMBDA_PENALTY: float = 0.5

# =============================================================================
# SIMULATION CONFIGURATION
# =============================================================================

# Simulation time per evaluation (seconds)
SIMULATION_TIME: int = 30

# Warmup time (seconds) - robot settles before metrics are recorded
# Set to 0 for no warmup
WARMUP_TIME: float = 0.0

# Number of parallel simulators (set based on CPU cores)
NUM_SIMULATORS: int = 11

# =============================================================================
# CMA-ES CONFIGURATION
# =============================================================================

# Number of generations to evolve
NUM_GENERATIONS: int = 500

# Population size (number of individuals per generation)
# If None, CMA-ES will auto-calculate based on parameter count
POPULATION_SIZE: int | None = None

# Parameter bounds for CPG parameters
# CPG params typically in range [-1, 1]
PARAM_BOUNDS: tuple[float, float] = (-1.0, 1.0)

# Initial standard deviation for parameter sampling
# Rule of thumb: std = 1/4 of parameter range
# Set to None to auto-calculate from bounds, or set manually
# Examples: bounds=[-1,1] -> std=0.5, bounds=[-2,2] -> std=1.0
INITIAL_STD: float | None = None  # Auto: (max - min) / 4

# =============================================================================
# DATABASE & LOGGING CONFIGURATION
# =============================================================================

# Enable database logging (saves all generations to SQLite)
USE_DATABASE: bool = True

# Database file path (relative to Experiments folder)
DATABASE_FILE: str = "evolution_results.sqlite"

# Print progress every N generations
LOG_EVERY: int = 1

# =============================================================================
# REPRODUCIBILITY
# =============================================================================

# Random seed (set to specific value for reproducibility, or None for random)
SEED: int | None = None

# =============================================================================
# EXPERIMENT METADATA
# =============================================================================

# Name/description for this experiment run
# Set to None to auto-generate: "{robot}_{metric}_lambda{λ}"
EXPERIMENT_NAME: str | None = None

# Additional notes (stored in database)
# Set to None to auto-generate based on parameters
EXPERIMENT_NOTES: str | None = None
