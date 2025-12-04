"""Evaluator with true contact detection and visualization."""

import math
from collections import defaultdict

import mujoco
import numpy as np
import numpy.typing as npt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic, CpgNetworkStructure
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.scene import UUIDKey
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._render_backend import RenderBackend
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator.viewers import CustomMujocoViewer
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


class EvaluatorWithContactsViz:
    """Evaluator with true contact detection and visualization support."""

    _terrain: object
    _cpg_network_structure: CpgNetworkStructure
    _body: Body
    _output_mapping: list[tuple[int, ActiveHinge]]

    def __init__(
        self,
        cpg_network_structure: CpgNetworkStructure,
        body: Body,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        """Initialize this evaluator."""
        self._terrain = terrains.flat()
        self._cpg_network_structure = cpg_network_structure
        self._body = body
        self._output_mapping = output_mapping

    def evaluate_with_contacts(
        self,
        solutions: list[npt.NDArray[np.float_]],
        headless: bool = True,
    ) -> tuple[npt.NDArray[np.float_], list[dict]]:
        """
        Evaluate robots with contact detection.

        :param solutions: Robot brain parameters.
        :param headless: Whether to show visualization.
        :returns: Fitnesses and contact information.
        """
        results = []

        for params in solutions:
            # Create robot
            robot = ModularRobot(
                body=self._body,
                brain=BrainCpgNetworkStatic.uniform_from_params(
                    params=params,
                    cpg_network_structure=self._cpg_network_structure,
                    initial_state_uniform=math.sqrt(2) * 0.5,
                    output_mapping=self._output_mapping,
                ),
            )

            # Create scene
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)

            # Convert to simulation scene
            sim_scene, robot_mapping = scene.to_simulation_scene()

            # Get parameters
            batch_params = make_standard_batch_parameters()

            # Convert to MuJoCo model
            model, abstraction_mapping = scene_to_model(
                scene=sim_scene,
                simulation_timestep=batch_params.simulation_timestep,
                cast_shadows=False,
                fast_sim=True,
            )

            # Initialize MuJoCo data
            data = mujoco.MjData(model)

            # Create viewer if not headless
            viewer = None
            if not headless:
                viewer = CustomMujocoViewer(
                    model,
                    data,
                    width=None,
                    height=None,
                    backend=RenderBackend.GLFW,
                    start_paused=False,
                    render_every_frame=False,
                    hide_menus=False,
                )

            # Get robot body ID
            multi_body_system = robot_mapping[UUIDKey(robot)]
            robot_body_id = abstraction_mapping.multi_body_system[
                UUIDKey(multi_body_system)
            ].id

            # Identify terrain geometries
            terrain_geom_ids = set()
            for i in range(model.ngeom):
                body_id = model.geom_bodyid[i]
                if body_id <= 1:
                    terrain_geom_ids.add(i)

            # Contact tracking
            contact_data = defaultdict(int)
            total_steps = 0

            # Initialize
            mujoco.mj_forward(model, data)
            initial_pos = data.xpos[robot_body_id].copy()

            # Simulation parameters
            control_step = 1.0 / batch_params.control_frequency
            simulation_time = batch_params.simulation_time
            simulation_timestep = batch_params.simulation_timestep

            last_control_time = 0.0

            # Create control interface
            control_interface = ControlInterfaceImpl(data, abstraction_mapping)

            # Simulation loop
            while data.time < simulation_time:
                # Control
                if data.time >= last_control_time + control_step:
                    last_control_time = math.floor(data.time / control_step) * control_step

                    simulation_state = SimulationStateImpl(
                        data=data,
                        abstraction_to_mujoco_mapping=abstraction_mapping,
                        camera_views={},
                    )

                    sim_scene.handler.handle(
                        simulation_state, control_interface, control_step
                    )

                # Step simulation
                mujoco.mj_step(model, data)
                total_steps += 1

                # Collect contacts
                for i in range(data.ncon):
                    contact = data.contact[i]
                    geom1 = int(contact.geom1)
                    geom2 = int(contact.geom2)

                    robot_geom = None
                    if geom1 in terrain_geom_ids and geom2 not in terrain_geom_ids:
                        robot_geom = geom2
                    elif geom2 in terrain_geom_ids and geom1 not in terrain_geom_ids:
                        robot_geom = geom1

                    if robot_geom is not None:
                        body_id = int(model.geom_bodyid[robot_geom])
                        contact_data[body_id] += 1

                # Render
                if viewer is not None:
                    status = viewer.render()
                    if status == -1:  # Window closed
                        break

            # Get final position
            final_pos = data.xpos[robot_body_id].copy()

            # Calculate fitness
            fitness = float(
                np.sqrt(
                    (final_pos[0] - initial_pos[0]) ** 2
                    + (final_pos[1] - initial_pos[1]) ** 2
                )
            )

            # Compile contact stats
            total_contacts = sum(contact_data.values())
            num_bodies_with_contact = len(contact_data)

            contact_info = {
                "total_contacts": total_contacts,
                "avg_contacts_per_step": (
                    total_contacts / total_steps if total_steps > 0 else 0
                ),
                "num_bodies_with_contact": num_bodies_with_contact,
                "total_steps": total_steps,
                "body_contact_counts": dict(contact_data),
                "body_contact_ratios": {
                    body_id: count / total_steps for body_id, count in contact_data.items()
                },
            }

            results.append((fitness, contact_info))

        fitnesses = np.array([r[0] for r in results])
        contact_infos = [r[1] for r in results]

        return fitnesses, contact_infos
