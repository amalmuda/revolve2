"""View an evolved Hopf robot WITH torque limits applied during viewing."""
import math
import sys
import numpy as np
import mujoco
import mujoco.viewer

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor


ROBOT = sys.argv[1] if len(sys.argv) > 1 else "babyb"
PARAMS = sys.argv[2] if len(sys.argv) > 2 else "hopf_babyb_blf_xy_TORQUELIMITED_best.npy"
COUPLING = sys.argv[3] if len(sys.argv) > 3 else "blf"
HZ = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
TORQUE_LIMIT = float(sys.argv[5]) if len(sys.argv) > 5 else 0.948013269


body = modular_robots_v1.get(ROBOT)
hinges = body.find_modules_of_type(ActiveHinge)
if COUPLING == "blf":
    cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
elif COUPLING == "uncoupled":
    cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
else:
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
hopf_struct = hopf_structure_from_cpg_structure(cpg)

params = np.load(PARAMS)
brain = BrainHopfStatic.from_params(
    params=params, network_structure=hopf_struct,
    output_mapping=mp, omega=2 * math.pi * HZ,
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
scene.add_robot(robot)
sim_scene, _ = scene.to_simulation_scene()

batch = make_standard_batch_parameters()
model, mj_mapping = scene_to_model(
    sim_scene, simulation_timestep=batch.simulation_timestep,
    cast_shadows=False, fast_sim=False,
)

# *** TORQUE LIMIT PATCH ***
model.actuator_forcerange[:, 0] = -TORQUE_LIMIT
model.actuator_forcerange[:, 1] = +TORQUE_LIMIT
model.actuator_forcelimited[:] = 1

data = mujoco.MjData(model)
ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
ctrl_step = 1.0 / batch.control_frequency
last_ctrl = 0.0

print(f"Viewing {ROBOT.upper()} Hopf {COUPLING} @ {HZ} Hz | torque limit: {TORQUE_LIMIT} Nm")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = data.time
        if data.time >= last_ctrl + ctrl_step:
            last_ctrl = math.floor(data.time / ctrl_step) * ctrl_step
            sim_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mj_mapping,
                camera_views={},
            )
            sim_scene.handler.handle(sim_state, ctrl, ctrl_step)
        mujoco.mj_step(model, data)
        viewer.sync()
