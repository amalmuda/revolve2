"""Configuration parameters for this experiment."""

from revolve2.standards.modular_robots_v1 import spider_v1

NUM_SIMULATORS = 11
INITIAL_STD = 2.5
NUM_GENERATIONS = 500
BOUNDS = [-4.0, 4.0]
BODY = spider_v1()

# Contact penalty mode: "counting" or "binary"
# - "counting": Penalizes based on HOW MANY non-leaf bodies are dragging
# - "binary": Penalizes based on WHETHER any non-leaf body is dragging (not how many)
CONTACT_MODE = "binary"
