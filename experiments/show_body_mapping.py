"""Show which MuJoCo body IDs correspond to which robot modules."""

import config
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import ActiveHinge, Core, Brick
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic, active_hinges_to_cpg_network_structure_neighbor
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.scene import UUIDKey
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
import numpy as np

# Setup robot
active_hinges = config.BODY.find_modules_of_type(ActiveHinge)
cpg_network_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

# Create a dummy robot
robot = ModularRobot(
    body=config.BODY,
    brain=BrainCpgNetworkStatic.uniform_from_params(
        params=np.zeros(cpg_network_structure.num_connections),
        cpg_network_structure=cpg_network_structure,
        initial_state_uniform=0.5,
        output_mapping=output_mapping,
    ),
)

# Create scene
scene = ModularRobotScene(terrain=terrains.flat())
scene.add_robot(robot)

# Convert to simulation scene
sim_scene, robot_mapping = scene.to_simulation_scene()

# Convert to MuJoCo model
batch_params = make_standard_batch_parameters()
model, abstraction_mapping = scene_to_model(
    scene=sim_scene,
    simulation_timestep=batch_params.simulation_timestep,
    cast_shadows=False,
    fast_sim=True,
)

# Get multi-body system
multi_body_system = robot_mapping[UUIDKey(robot)]

# Get all modules
all_modules = config.BODY.find_modules_of_type(Module)
core_modules = config.BODY.find_modules_of_type(Core)
hinges = config.BODY.find_modules_of_type(ActiveHinge)
bricks = config.BODY.find_modules_of_type(Brick)

print("Robot Module Summary:")
print(f"  Total modules: {len(all_modules)}")
print(f"  Core modules: {len(core_modules)}")
print(f"  Active hinges: {len(hinges)}")
print(f"  Bricks: {len(bricks)}")

print(f"\nMuJoCo Model Info:")
print(f"  Total bodies: {model.nbody}")
print(f"  Total geometries: {model.ngeom}")

# Show body names
print(f"\nMuJoCo Body Names:")
for i in range(model.nbody):
    body_name = model.body(i).name
    print(f"  Body {i}: {body_name}")

# Try to match body IDs to module types based on naming
print(f"\nBody Type Mapping (inferred from names):")
for i in range(model.nbody):
    body_name = model.body(i).name
    if "world" in body_name.lower() or body_name == "":
        module_type = "World"
    elif "terrain" in body_name.lower() or "floor" in body_name.lower():
        module_type = "Terrain"
    elif "core" in body_name.lower():
        module_type = "Core"
    elif "brick" in body_name.lower():
        module_type = "Brick"
    elif "activehinge" in body_name.lower() or "hinge" in body_name.lower():
        module_type = "ActiveHinge"
    else:
        module_type = "Unknown"

    print(f"  Body {i} ({body_name}): {module_type}")
