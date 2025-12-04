"""Custom simulation handler for tracking ground contacts."""

from revolve2.modular_robot_simulation import ModularRobotSimulationHandler
from revolve2.simulation.scene import SimulationState, ControlInterface


class ContactTrackingHandler(ModularRobotSimulationHandler):
    """Handler that tracks ground contact information during simulation."""

    def __init__(self):
        """Initialize the contact tracking handler."""
        super().__init__()
        self.contact_history = []
        self._mujoco_data = None
        self._mujoco_model = None
        self._terrain_geom_ids = []

    def set_mujoco_refs(self, model, data, terrain_geom_ids):
        """
        Set references to MuJoCo model and data.

        :param model: MuJoCo model.
        :param data: MuJoCo data.
        :param terrain_geom_ids: List of geometry IDs belonging to terrain.
        """
        self._mujoco_model = model
        self._mujoco_data = data
        self._terrain_geom_ids = terrain_geom_ids

    def handle(
        self,
        state: SimulationState,
        control: ControlInterface,
        dt: float,
    ) -> None:
        """
        Handle a simulation step and collect contact data.

        :param state: Current simulation state.
        :param control: Control interface.
        :param dt: Time delta.
        """
        # Call parent to handle normal modular robot control
        super().handle(state, control, dt)

        # Collect contact information if MuJoCo data is available
        if self._mujoco_data is not None and self._mujoco_model is not None:
            contacts = []

            # Iterate through all active contacts
            for i in range(self._mujoco_data.ncon):
                contact = self._mujoco_data.contact[i]
                geom1 = contact.geom1
                geom2 = contact.geom2

                # Check if this is a contact with the terrain
                robot_geom = None
                if geom1 in self._terrain_geom_ids:
                    robot_geom = geom2
                elif geom2 in self._terrain_geom_ids:
                    robot_geom = geom1

                if robot_geom is not None:
                    # Get body ID for this geometry
                    body_id = self._mujoco_model.geom_bodyid[robot_geom]

                    contacts.append({
                        'body_id': int(body_id),
                        'geom_id': int(robot_geom),
                        'position': contact.pos.copy(),
                        'normal': contact.frame[:3].copy(),
                        'dist': float(contact.dist),
                    })

            self.contact_history.append(contacts)