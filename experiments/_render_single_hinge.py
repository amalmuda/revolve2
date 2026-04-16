"""Render just the hinge part, multiple angles."""
import numpy as np
import mujoco
from PIL import Image
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

WIDTH = 800
HEIGHT = 800

# Core with hinge on each side so the hinge is in the center
body = BodyV1()
body.core_v1.back = ActiveHingeV1(0.0)
body.core_v1.back.attachment = BrickV1(0.0)

hinges = body.find_modules_of_type(ActiveHinge)
cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
params = np.zeros(cpg.num_connections)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=params, cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mapping,
)
robot = ModularRobot(body=body, brain=brain)

terrain = Terrain(
    static_geometry=[GeometryPlane(
        pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
        texture=Texture(
            base_color=Color(240, 240, 240, 255),
            primary_color=Color(245, 245, 245, 255),
            secondary_color=Color(220, 220, 220, 255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"),
            repeat=(50, 50),
        ),
    )],
    friction=1.0,
)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot)
sim_scene, _ = scene.to_simulation_scene()

model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=0.001,
                                    cast_shadows=True, fast_sim=False)
model.vis.global_.offwidth = WIDTH
model.vis.global_.offheight = HEIGHT

# Make core and brick semi-transparent by changing their rgba
for i in range(model.ngeom):
    if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
        continue
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name is None:
        continue
    size = model.geom_size[i]
    vol = size[0] * size[1] * size[2]
    # The hinge servo parts are the small white boxes
    # Core is the big red one, brick is the big blue one
    # Make core and brick invisible (alpha=0)
    if vol > 0.000005:  # big boxes = core or brick
        model.geom_rgba[i] = [0, 0, 0, 0]  # transparent

data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

# Render multiple angles
angles = [
    ("front", 180, -20, 0.10),
    ("side", 90, -20, 0.10),
    ("angle", 135, -25, 0.12),
    ("top", 90, -80, 0.10),
]

# Find hinge center
hinge_positions = []
for i in range(model.ngeom):
    if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
        continue
    if model.geom_rgba[i][3] > 0:  # visible = hinge parts
        hinge_positions.append(data.geom_xpos[i].copy())

if hinge_positions:
    center = np.mean(hinge_positions, axis=0)
else:
    center = np.array([0.03, 0, 0.02])

for name, az, el, dist in angles:
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = center
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()
    img = Image.fromarray(pixels)
    img.save(f"hinge_{name}.png")
    print(f"Saved: hinge_{name}.png")
