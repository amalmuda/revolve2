"""Render insect body offscreen and save PNGs (top + iso)."""
import mujoco
import numpy as np
from PIL import Image

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model


body = modular_robots_v1.get("insect")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=np.zeros(cpg.num_connections),
    cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mapping,
)
robot = ModularRobot(body=body, brain=brain)

terrain = Terrain(static_geometry=[GeometryPlane(
    pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
    texture=Texture(
        base_color=Color(200, 200, 200, 255),
        primary_color=Color(220, 220, 220, 255),
        secondary_color=Color(80, 80, 80, 255),
        map_type=MapType.MAP2D,
        reference=TextureReference(builtin="checker"),
        repeat=(50, 50),
    ),
)], friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose())
sim_scene, _ = scene.to_simulation_scene()

model, _ = scene_to_model(sim_scene, simulation_timestep=0.004, cast_shadows=True, fast_sim=False)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat = np.array([0.0, 0.0, 0.05])
cam.distance = 1.6
cam.azimuth = 90
cam.elevation = -89
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera=cam)
Image.fromarray(renderer.render()).save("/tmp/insect_top.png")
print("saved /tmp/insect_top.png")

cam.elevation = -25
cam.azimuth = 135
renderer.update_scene(data, camera=cam)
Image.fromarray(renderer.render()).save("/tmp/insect_iso.png")
print("saved /tmp/insect_iso.png")

cam.elevation = 0
cam.azimuth = 90
renderer.update_scene(data, camera=cam)
Image.fromarray(renderer.render()).save("/tmp/insect_side.png")
print("saved /tmp/insect_side.png")

print("\nHinges: %d" % len(hinges))
print("Hinge IDs/names in mujoco:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name and "hinge" in name.lower():
        axis = model.jnt_axis[i]
        pos = data.xanchor[i]
        print("  %s axis=(% .2f,% .2f,% .2f) pos=(% .2f,% .2f,% .2f)" %
              (name, axis[0], axis[1], axis[2], pos[0], pos[1], pos[2]))
