"""Locomotion metrics for modular robots.

This module provides functionality to measure various locomotion metrics:
- Contact detection (feet vs non-feet ground contacts)
- Distance (displacement from start to end)
- Cost of Transport (energy efficiency)
- Stability (height and orientation variance)
- Straightness (path efficiency)
"""

import logging
import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkNeighborRandom,
    BrainCpgNetworkStatic,
    CpgNetworkStructure,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot.brain.cpg._cpg_network_structure import CpgPair
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._render_backend import RenderBackend
from revolve2.simulators.mujoco_simulator.viewers import CustomMujocoViewer
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards import modular_robots_v1
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed


def get_feet(body: Body) -> list[Module]:
    """
    Get all feet (leaf modules) of a robot body.

    Feet are defined as modules with no children (leaf nodes in the tree).

    :param body: The robot body.
    :returns: List of leaf modules (feet).
    """
    feet: list[Module] = []

    def find_leaves(module: Module) -> None:
        if len(module.children) == 0:
            feet.append(module)
        else:
            for child in module.children.values():
                find_leaves(child)

    find_leaves(body.core)
    return feet


def get_all_modules(body: Body) -> list[Module]:
    """
    Get all modules in the robot body (including core).

    :param body: The robot body.
    :returns: List of all modules.
    """
    modules: list[Module] = []

    def collect_modules(module: Module) -> None:
        modules.append(module)
        for child in module.children.values():
            collect_modules(child)

    collect_modules(body.core)
    return modules


@dataclass
class ContactInfo:
    """Information about a detected contact."""
    time: float
    geom1_name: str
    geom2_name: str
    contact_force: float
    position: tuple[float, float, float]


@dataclass
class ContactTracker:
    """Tracks contacts during simulation."""
    ground_contacts: list[ContactInfo] = field(default_factory=list)
    non_foot_ground_contacts: list[ContactInfo] = field(default_factory=list)
    foot_geom_ids: set[int] = field(default_factory=set)
    ground_geom_ids: set[int] = field(default_factory=set)
    all_robot_geom_ids: set[int] = field(default_factory=set)
    non_foot_geom_ids: set[int] = field(default_factory=set)
    # Per-timestep tracking for metrics
    timesteps_with_non_foot_contact: int = 0  # M1: body contact
    timesteps_with_bad_contact: int = 0  # M2: body contact OR foot slip
    total_timesteps: int = 0
    # Foot slip tracking
    foot_positions_prev: dict = field(default_factory=dict)  # geom_id -> (x, y, z)
    foot_slip_threshold: float = 0.1  # velocity threshold for slip detection (m/s)
    # Per-foot slip tracking
    foot_slip_counts: dict = field(default_factory=dict)  # geom_id -> slip count
    foot_contact_counts: dict = field(default_factory=dict)  # geom_id -> contact count
    foot_geom_names: dict = field(default_factory=dict)  # geom_id -> name


@dataclass
class ContactMetrics:
    """Calculated contact metrics for fitness evaluation.

    All metrics are penalties (lower = better, 0 = perfect).
    Range: 0.0 to 1.0 for both metrics.
    """
    # Metric 1: % of timesteps with any non-foot (body) contact
    # (0.0 = no body contact ever, 1.0 = body contact every timestep)
    metric_1: float = 0.0
    # Metric 2: % of timesteps with bad contact (body contact OR foot slip)
    # Foot slip = foot touching ground while moving (velocity > threshold)
    # (0.0 = perfect gait, 1.0 = bad contact every timestep)
    metric_2: float = 0.0
    # Number of non-foot modules (for reference)
    num_non_foot_modules: int = 0


@dataclass
class LocomotionTracker:
    """Tracks locomotion data during simulation for metric calculation."""
    # Position tracking
    start_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    end_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    previous_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    total_path_length: float = 0.0

    # Stability tracking (lists of per-timestep values)
    heights: list[float] = field(default_factory=list)
    roll_angles: list[float] = field(default_factory=list)
    pitch_angles: list[float] = field(default_factory=list)

    # Energy tracking for CoT
    total_energy: float = 0.0
    robot_mass: float = 0.0

    # Simulation info
    simulation_timestep: float = 0.001
    total_timesteps: int = 0


@dataclass
class LocomotionMetrics:
    """All locomotion metrics for analysis.

    Includes both fitness-related and analysis-only metrics.
    """
    # === FITNESS METRICS ===
    # Distance: straight-line displacement from start to end (meters)
    # Higher = better
    distance: float = 0.0

    # Contact penalties (from ContactMetrics)
    # Lower = better, range 0-1
    contact_metric_1: float = 0.0  # % timesteps with any body (non-foot) contact
    contact_metric_2: float = 0.0  # % timesteps with body contact OR foot slip

    # === ANALYSIS METRICS ===
    # Cost of Transport: energy / (mass * gravity * distance)
    # Lower = more efficient, dimensionless
    # Returns infinity if distance is 0
    cost_of_transport: float = 0.0

    # Stability: combined measure of height and orientation variance
    # Lower = more stable
    stability_height_variance: float = 0.0  # Variance in body height (m²)
    stability_roll_variance: float = 0.0    # Variance in roll angle (rad²)
    stability_pitch_variance: float = 0.0   # Variance in pitch angle (rad²)
    stability_combined: float = 0.0         # Combined stability score

    # Straightness: final_displacement / total_path_length
    # Higher = straighter path, range 0-1
    # 1.0 = perfectly straight, <1.0 = wandering
    straightness: float = 0.0

    # Reference values
    total_path_length: float = 0.0
    total_energy: float = 0.0
    robot_mass: float = 0.0
    simulation_time: float = 0.0
    num_non_foot_modules: int = 0


def calculate_contact_metrics(tracker: ContactTracker) -> ContactMetrics:
    """
    Calculate all contact metrics from a ContactTracker.

    :param tracker: The contact tracker with simulation data.
    :returns: ContactMetrics with all calculated values.
    """
    metrics = ContactMetrics()

    if tracker.total_timesteps == 0:
        return metrics

    # Store number of non-foot modules for reference
    metrics.num_non_foot_modules = len(tracker.non_foot_geom_ids)

    # Metric 1: % timesteps with any non-foot (body) contact
    metrics.metric_1 = (
        tracker.timesteps_with_non_foot_contact / tracker.total_timesteps
    )

    # Metric 2: % timesteps with bad contact (body contact OR foot slip)
    metrics.metric_2 = (
        tracker.timesteps_with_bad_contact / tracker.total_timesteps
    )

    return metrics


def quaternion_to_euler(quat: np.ndarray) -> tuple[float, float, float]:
    """
    Convert quaternion to euler angles (roll, pitch, yaw).

    MuJoCo uses [w, x, y, z] quaternion format.

    :param quat: Quaternion array [w, x, y, z].
    :returns: Tuple of (roll, pitch, yaw) in radians.
    """
    w, x, y, z = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def get_robot_core_body_id(model: mujoco.MjModel) -> int | None:
    """
    Find the body ID of the robot core.

    The core body is identified as an 'mbs' body without '_link' in its name,
    and with non-zero mass (to skip placeholder bodies like 'mbs0/').

    :param model: The MuJoCo model.
    :returns: Body ID of the core, or None if not found.
    """
    # Look for mbs body without _link and with mass (skip placeholders)
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and "mbs" in body_name and "_link" not in body_name:
            # Skip placeholder bodies with zero mass
            if model.body_mass[i] > 0:
                return i

    # Fallback: return first mbs body with mass
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and "mbs" in body_name and model.body_mass[i] > 0:
            return i
    return None


def get_robot_mass(model: mujoco.MjModel) -> float:
    """
    Calculate total mass of the robot.

    :param model: The MuJoCo model.
    :returns: Total robot mass in kg.
    """
    total_mass = 0.0
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and "mbs" in body_name:
            total_mass += model.body_mass[i]
    return total_mass


def calculate_actuator_energy(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    timestep: float,
) -> float:
    """
    Calculate energy used by actuators in one timestep.

    Energy = sum(|torque * angular_velocity|) * timestep

    :param model: The MuJoCo model.
    :param data: The MuJoCo data.
    :param timestep: Simulation timestep in seconds.
    :returns: Energy used in this timestep (Joules).
    """
    energy = 0.0

    # Get actuator forces and corresponding joint velocities
    for i in range(model.nu):  # nu = number of actuators
        # Get the joint that this actuator controls
        joint_id = model.actuator_trnid[i, 0]

        # Get actuator force (torque for rotational joints)
        torque = data.actuator_force[i]

        # Get joint velocity
        # For hinge joints, qvel index corresponds to joint dof
        qvel_idx = model.jnt_dofadr[joint_id]
        angular_velocity = data.qvel[qvel_idx]

        # Power = |torque * angular_velocity|
        # Energy = Power * dt
        energy += abs(torque * angular_velocity) * timestep

    return energy


def calculate_locomotion_metrics(
    loco_tracker: LocomotionTracker,
    contact_tracker: ContactTracker,
    simulation_time: float,
) -> LocomotionMetrics:
    """
    Calculate all locomotion metrics from trackers.

    :param loco_tracker: The locomotion tracker with position/energy data.
    :param contact_tracker: The contact tracker with contact data.
    :param simulation_time: Total simulation time in seconds.
    :returns: LocomotionMetrics with all calculated values.
    """
    metrics = LocomotionMetrics()
    metrics.simulation_time = simulation_time

    # === Contact Metrics ===
    contact_metrics = calculate_contact_metrics(contact_tracker)
    metrics.contact_metric_1 = contact_metrics.metric_1
    metrics.contact_metric_2 = contact_metrics.metric_2
    metrics.num_non_foot_modules = contact_metrics.num_non_foot_modules

    # === Distance (XY displacement) ===
    dx = loco_tracker.end_position[0] - loco_tracker.start_position[0]
    dy = loco_tracker.end_position[1] - loco_tracker.start_position[1]
    metrics.distance = math.sqrt(dx * dx + dy * dy)

    # === Straightness ===
    metrics.total_path_length = loco_tracker.total_path_length
    if loco_tracker.total_path_length > 0:
        metrics.straightness = metrics.distance / loco_tracker.total_path_length
        # Clamp to [0, 1] in case of numerical issues
        metrics.straightness = min(1.0, max(0.0, metrics.straightness))
    else:
        metrics.straightness = 0.0

    # === Stability ===
    if loco_tracker.heights:
        metrics.stability_height_variance = float(np.var(loco_tracker.heights))
    if loco_tracker.roll_angles:
        metrics.stability_roll_variance = float(np.var(loco_tracker.roll_angles))
    if loco_tracker.pitch_angles:
        metrics.stability_pitch_variance = float(np.var(loco_tracker.pitch_angles))

    # Combined stability: sum of normalized variances
    # Using standard deviations for more interpretable values
    height_std = math.sqrt(metrics.stability_height_variance)
    roll_std = math.sqrt(metrics.stability_roll_variance)
    pitch_std = math.sqrt(metrics.stability_pitch_variance)
    # Combined score (lower = more stable)
    metrics.stability_combined = height_std + roll_std + pitch_std

    # === Cost of Transport ===
    metrics.total_energy = loco_tracker.total_energy
    metrics.robot_mass = loco_tracker.robot_mass
    GRAVITY = 9.81

    if metrics.distance > 0.001 and loco_tracker.robot_mass > 0:
        metrics.cost_of_transport = (
            loco_tracker.total_energy
            / (loco_tracker.robot_mass * GRAVITY * metrics.distance)
        )
    else:
        metrics.cost_of_transport = float('inf')

    return metrics


def identify_geometry_types(model: mujoco.MjModel) -> tuple[set[int], set[int], set[int]]:
    """
    Identify ground, robot, and foot geometries in the MuJoCo model.

    Ground geometries are planes or heightmaps.
    Robot geometries are identified by 'mbs' in their body name.
    Foot geometries are bodies that have a hinge in their ancestor chain
    (i.e. part of a limb) AND have no hinge descendant (i.e. everything
    after the last hinge in a chain). This means a blob of bricks hanging
    off the final hinge is entirely considered foot.

    :param model: The MuJoCo model.
    :returns: Tuple of (ground_geom_ids, robot_geom_ids, foot_geom_ids).
    """
    ground_geom_ids = set()
    robot_geom_ids = set()
    foot_geom_ids = set()

    # Identify robot bodies and build parent-child relationships
    robot_body_ids = set()
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and "mbs" in body_name:
            robot_body_ids.add(i)

    # Find which bodies are parents (have children)
    parent_bodies = set()
    for i in range(model.nbody):
        parent_id = model.body_parentid[i]
        if parent_id in robot_body_ids:
            parent_bodies.add(parent_id)

    # Leaf bodies = robot bodies that are NOT parents of any other body
    leaf_body_ids = robot_body_ids - parent_bodies

    # Find which robot bodies have a hinge joint (i.e. are active hinges)
    bodies_with_hinge = set()
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            bodies_with_hinge.add(model.jnt_bodyid[j])

    def has_hinge_in_chain(body_id):
        """Walk up the body tree (including self); return True if any body has a hinge."""
        cur = body_id
        while cur != 0 and cur in robot_body_ids:
            if cur in bodies_with_hinge:
                return True
            cur = model.body_parentid[cur]
        return False

    # Mark bodies that have a hinge DESCENDANT (not self)
    # For each hinge body, walk up and mark all ancestors
    has_hinge_below = set()
    for hinge_body in bodies_with_hinge:
        cur = model.body_parentid[hinge_body]
        while cur != 0 and cur in robot_body_ids:
            has_hinge_below.add(cur)
            cur = model.body_parentid[cur]

    # Foot = any robot body that:
    # 1. Has a hinge in its chain (self or ancestor) — it's part of a limb, not body
    # 2. Has NO hinge descendant — it's after the last joint (the foot segment)
    foot_body_ids = set()
    for b in robot_body_ids:
        if has_hinge_in_chain(b) and b not in has_hinge_below:
            foot_body_ids.add(b)

    # Identify geometries
    for i in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        geom_type = model.geom_type[i]
        geom_body_id = model.geom_bodyid[i]

        # Planes are typically ground
        if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            ground_geom_ids.add(i)
        # Heightfields are also ground
        elif geom_type == mujoco.mjtGeom.mjGEOM_HFIELD:
            ground_geom_ids.add(i)
        # Check if it's a robot geometry (part of multi-body system)
        elif geom_name and "mbs" in geom_name:
            robot_geom_ids.add(i)
            # Foot geometries: on bodies after the last hinge (no hinge descendant)
            if geom_body_id in foot_body_ids:
                foot_geom_ids.add(i)

    return ground_geom_ids, robot_geom_ids, foot_geom_ids


def get_contacts_with_ground(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ground_geom_ids: set[int],
    robot_geom_ids: set[int],
) -> list[tuple[int, int, float, tuple[float, float, float]]]:
    """
    Get all contacts between robot and ground.

    :param model: The MuJoCo model.
    :param data: The MuJoCo data.
    :param ground_geom_ids: Set of ground geometry IDs.
    :param robot_geom_ids: Set of robot geometry IDs.
    :returns: List of (robot_geom_id, ground_geom_id, force, position) tuples.
    """
    contacts = []

    for i in range(data.ncon):
        contact = data.contact[i]
        geom1, geom2 = contact.geom1, contact.geom2

        # Check if one is ground and one is robot
        robot_geom = None
        ground_geom = None

        if geom1 in ground_geom_ids and geom2 in robot_geom_ids:
            ground_geom, robot_geom = geom1, geom2
        elif geom2 in ground_geom_ids and geom1 in robot_geom_ids:
            ground_geom, robot_geom = geom2, geom1

        if robot_geom is not None and ground_geom is not None:
            # Get contact force magnitude
            force = np.linalg.norm(contact.frame[:3])
            position = tuple(contact.pos)
            contacts.append((robot_geom, ground_geom, force, position))

    return contacts


def active_hinges_to_cpg_network_structure_internal_only(
    active_hinges: list[ActiveHinge],
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create a CPG structure with ONLY internal weights (no external coupling).

    Each hinge gets its own independent oscillator with no connections to other hinges.

    :param active_hinges: The active hinges to base the structure on.
    :returns: The created structure and a mapping between state indices and active hinges.
    """
    cpgs = CpgNetworkStructure.make_cpgs(len(active_hinges))
    # Empty connections set = no external coupling
    empty_connections: set = set()
    cpg_network_structure = CpgNetworkStructure(cpgs, empty_connections)
    output_mapping = list(zip(cpg_network_structure.output_indices, active_hinges))
    return cpg_network_structure, output_mapping


def active_hinges_to_cpg_network_structure_fully_connected(
    active_hinges: list[ActiveHinge],
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create a CPG structure where every oscillator is coupled to every other.

    This is the all-to-all topology: N internal weights + C(N,2) coupling weights.

    :param active_hinges: The active hinges to base the structure on.
    :returns: The created structure and a mapping between state indices and active hinges.
    """
    cpgs = CpgNetworkStructure.make_cpgs(len(active_hinges))
    connections: set[CpgPair] = set()
    for i in range(len(cpgs)):
        for j in range(i + 1, len(cpgs)):
            connections.add(CpgPair(cpgs[i], cpgs[j]))
    cpg_network_structure = CpgNetworkStructure(cpgs, connections)
    output_mapping = list(zip(cpg_network_structure.output_indices, active_hinges))
    return cpg_network_structure, output_mapping


def active_hinges_to_cpg_network_structure_blf(
    active_hinges: list[ActiveHinge],
    body: Body,
    use_symmetry: bool = False,
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create a CPG structure using BLF (Body/Limb Finder) from the EPFL paper.

    BLF automatically detects body vs limb modules, classifies joints (spine, hip, knee),
    and creates bio-inspired coupling:
    - Spine oscillators are fully coupled (all-to-all)
    - Hip oscillators are fully coupled (all-to-all) and connected to nearest spine(s)
    - Knee oscillators are chained within their limb (each to previous hinge)

    :param active_hinges: The active hinges to base the structure on.
    :param body: The robot body.
    :param use_symmetry: Whether to use BLF-SYM (symmetric limbs share parameters).
    :returns: The created structure and a mapping between state indices and active hinges.
    """
    from blf import generate_blf_cpg_network

    structure, mapping, result, generator = generate_blf_cpg_network(body, use_symmetry=use_symmetry)

    # The BLF mapping might have different order than input active_hinges
    # We need to return a compatible mapping
    return structure, mapping


def simulate_with_contact_detection(
    robot_name: str = "spider",
    simulation_time: int = 10,
    verbose: bool = True,
    cpg_params: list[float] | None = None,
    headless: bool = False,
    terrain_friction: float = 1.0,
    warmup_time: float = 0.0,
    cast_shadows: bool = True,
    track_camera: bool = True,
    slip_threshold: float = 0.1,
    no_coupling: bool = False,
    blf_coupling: bool = False,
    blf_symmetry: bool = False,
) -> tuple[ContactTracker, LocomotionMetrics]:
    """
    Run simulation with full locomotion metric tracking.

    Tracks: contact, distance, CoT, stability, straightness.

    :param robot_name: Name of the robot to simulate.
    :param simulation_time: Duration of simulation in seconds (total, including warmup).
    :param verbose: Whether to print contact events and metrics.
    :param cpg_params: Optional list of CPG parameters. If None, random params are used.
    :param headless: If True, run without viewer.
    :param terrain_friction: Ground friction coefficient.
    :param warmup_time: Time in seconds to let robot settle before measuring metrics.
                        Metrics are only recorded after warmup period.
    :param cast_shadows: Whether to render shadows (only affects viewer mode).
    :param track_camera: Whether camera should follow the robot (only affects viewer mode).
    :param slip_threshold: Velocity threshold (m/s) for foot slip detection.
    :param no_coupling: If True, use only internal CPG weights (no external coupling between hinges).
    :param blf_coupling: If True, use BLF (Body/Limb Finder) bio-inspired coupling.
    :param blf_symmetry: If True, use BLF-SYM (symmetric limbs share parameters).
    :returns: Tuple of (ContactTracker, LocomotionMetrics).
    """
    # Validate warmup_time
    if warmup_time >= simulation_time:
        raise ValueError(
            f"warmup_time ({warmup_time}s) must be less than simulation_time ({simulation_time}s)"
        )

    # Only setup logging for interactive runs, not worker processes
    if not headless:
        setup_logging()

    # Create robot - following the CMA-ES example pattern
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Choose CPG structure based on coupling mode
    if blf_coupling:
        (cpg_network_structure, output_mapping) = active_hinges_to_cpg_network_structure_blf(active_hinges, body, use_symmetry=blf_symmetry)
    elif no_coupling:
        (cpg_network_structure, output_mapping) = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    else:
        (cpg_network_structure, output_mapping) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    if cpg_params is not None:
        # Use provided CPG parameters
        expected_params = cpg_network_structure.num_connections
        if len(cpg_params) != expected_params:
            raise ValueError(f"Expected {expected_params} CPG params, got {len(cpg_params)}")

        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=np.array(cpg_params),
            cpg_network_structure=cpg_network_structure,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=output_mapping,
        )
        if verbose:
            print(f"Using provided CPG parameters ({len(cpg_params)} values)")
    else:
        # Use random CPG parameters
        rng = make_rng_time_seed()
        brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
        if verbose:
            print("Using random CPG parameters")

    robot = ModularRobot(body, brain)

    # Get feet and all modules for reference
    feet = get_feet(body)
    all_modules = get_all_modules(body)
    non_feet = [m for m in all_modules if m not in feet]

    if verbose:
        print(f"\nRobot: {robot_name}")
        print(f"Total modules: {len(all_modules)}")
        print(f"Feet (leaf modules): {len(feet)}")
        print(f"Non-feet modules: {len(non_feet)}")

    # Create scene with ground
    terrain = Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(),
                mass=0.0,
                size=Vector2([20.0, 20.0]),
                texture=Texture(
                    base_color=Color(200, 200, 200, 255),
                    primary_color=Color(220, 220, 220, 255),
                    secondary_color=Color(80, 80, 80, 255),
                    map_type=MapType.MAP2D,
                    reference=TextureReference(builtin="checker"),
                    repeat=(50, 50),
                ),
            )
        ],
        friction=terrain_friction,
    )

    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    # Convert scene to simulation scene (this sets up the handler with the brain)
    batch_params = make_standard_batch_parameters()
    simulation_scene, robot_to_mbs_mapping = scene.to_simulation_scene()

    if verbose:
        print(f"\nStarting simulation for {simulation_time}s...")
        if headless:
            print("Running headless simulation with contact tracking...\n")
        else:
            print("Running with viewer and contact tracking...\n")

    # Create model and data for simulation with contact tracking
    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=cast_shadows and not headless,
        fast_sim=headless,
    )
    data = mujoco.MjData(model)

    # Identify geometry types
    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids
    # Set terrain friction on ground geometries
    for geom_id in ground_geom_ids:
        model.geom_friction[geom_id] = [terrain_friction, 0.005, 0.0001]

    # Get robot core body ID for position/orientation tracking
    core_body_id = get_robot_core_body_id(model)
    if core_body_id is None:
        raise RuntimeError("Could not find robot core body in MuJoCo model")

    if verbose:
        print(f"Ground geometries: {len(ground_geom_ids)}")
        print(f"Robot geometries: {len(robot_geom_ids)}")
        print(f"  Foot geometries: {len(foot_geom_ids)}")
        print(f"  Non-foot geometries: {len(non_foot_geom_ids)}")
        if warmup_time > 0:
            print(f"Warmup period: {warmup_time}s (metrics recorded after)")

    # Create contact tracker
    contact_tracker = ContactTracker()
    contact_tracker.ground_geom_ids = ground_geom_ids
    contact_tracker.all_robot_geom_ids = robot_geom_ids
    contact_tracker.foot_geom_ids = foot_geom_ids
    contact_tracker.non_foot_geom_ids = non_foot_geom_ids
    contact_tracker.foot_slip_threshold = slip_threshold

    # Create locomotion tracker
    loco_tracker = LocomotionTracker()
    loco_tracker.robot_mass = get_robot_mass(model)
    loco_tracker.simulation_timestep = batch_params.simulation_timestep

    # Track if we've passed warmup (for resetting start position)
    warmup_complete = False

    # Create control interface
    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)

    # Set up viewer if not headless
    viewer = None
    if not headless:
        viewer = CustomMujocoViewer(
            model,
            data,
            backend=RenderBackend.GLFW,
            start_paused=False,
            render_every_frame=False,
            hide_menus=False,
        )

    # Track contacts during simulation
    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0
    logged_modules: set[int] = set()

    mujoco.mj_forward(model, data)

    # Initialize locomotion tracking with starting position
    start_pos = data.xpos[core_body_id].copy()
    loco_tracker.start_position = tuple(start_pos)
    loco_tracker.previous_position = tuple(start_pos)

    while (time := data.time) < simulation_time:
        # Check if warmup period is complete
        in_warmup = time < warmup_time

        # When warmup completes, reset start position for distance measurement
        if not in_warmup and not warmup_complete:
            warmup_complete = True
            current_pos = data.xpos[core_body_id].copy()
            loco_tracker.start_position = tuple(current_pos)
            loco_tracker.previous_position = tuple(current_pos)
            loco_tracker.total_path_length = 0.0
            loco_tracker.total_energy = 0.0
            loco_tracker.heights = []
            loco_tracker.roll_angles = []
            loco_tracker.pitch_angles = []
            if verbose and warmup_time > 0:
                print(f"[{time:.2f}s] Warmup complete, starting metrics recording")

        # Only count metrics after warmup
        if not in_warmup:
            contact_tracker.total_timesteps += 1
            loco_tracker.total_timesteps += 1

        # === Track robot position and orientation ===
        current_pos = data.xpos[core_body_id].copy()
        current_quat = data.xquat[core_body_id].copy()

        # Only track locomotion metrics after warmup
        if not in_warmup:
            # Update path length (XY distance from previous position)
            dx = current_pos[0] - loco_tracker.previous_position[0]
            dy = current_pos[1] - loco_tracker.previous_position[1]
            step_distance = math.sqrt(dx * dx + dy * dy)
            loco_tracker.total_path_length += step_distance
            loco_tracker.previous_position = tuple(current_pos)

            # Track height (z-position)
            loco_tracker.heights.append(float(current_pos[2]))

            # Track orientation (roll, pitch)
            roll, pitch, yaw = quaternion_to_euler(current_quat)
            loco_tracker.roll_angles.append(roll)
            loco_tracker.pitch_angles.append(pitch)

            # === Track energy consumption ===
            energy = calculate_actuator_energy(model, data, batch_params.simulation_timestep)
            loco_tracker.total_energy += energy

        # === Track contacts (only after warmup) ===
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        non_foot_modules_this_timestep: set[int] = set()
        foot_slip_this_timestep = False

        for robot_geom, ground_geom, force, position in contacts:
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, robot_geom)

            # Only record contact info after warmup
            if not in_warmup:
                contact_info = ContactInfo(
                    time=time,
                    geom1_name=geom_name or f"geom_{robot_geom}",
                    geom2_name="ground",
                    contact_force=force,
                    position=position,
                )
                contact_tracker.ground_contacts.append(contact_info)

            is_foot_contact = robot_geom in foot_geom_ids
            if not is_foot_contact:
                # Non-foot (body) contact
                if not in_warmup:
                    contact_tracker.non_foot_ground_contacts.append(contact_info)
                if verbose and robot_geom not in logged_modules:
                    print(f"[{time:.2f}s] NON-FOOT CONTACT: {geom_name}")
                    logged_modules.add(robot_geom)
                non_foot_modules_this_timestep.add(robot_geom)
            else:
                # Foot contact - check for slip
                # Get foot geom position (use contact position)
                foot_pos = np.array(position[:2])  # XY position only

                # Track per-foot contact count
                if robot_geom not in contact_tracker.foot_contact_counts:
                    contact_tracker.foot_contact_counts[robot_geom] = 0
                    contact_tracker.foot_slip_counts[robot_geom] = 0
                    contact_tracker.foot_geom_names[robot_geom] = geom_name
                contact_tracker.foot_contact_counts[robot_geom] += 1

                if robot_geom in contact_tracker.foot_positions_prev:
                    # Calculate velocity (displacement / timestep)
                    prev_pos = contact_tracker.foot_positions_prev[robot_geom]
                    displacement = np.linalg.norm(foot_pos - prev_pos)
                    velocity = displacement / batch_params.simulation_timestep

                    # Check for slip (velocity > threshold)
                    if velocity > contact_tracker.foot_slip_threshold:
                        foot_slip_this_timestep = True
                        contact_tracker.foot_slip_counts[robot_geom] += 1
                        if verbose and robot_geom not in logged_modules:
                            print(f"[{time:.2f}s] FOOT SLIP: {geom_name} (vel={velocity:.4f} m/s)")
                            logged_modules.add(robot_geom)

                # Update previous position for next timestep
                contact_tracker.foot_positions_prev[robot_geom] = foot_pos

        # Only track contact metrics after warmup
        if not in_warmup:
            has_body_contact = len(non_foot_modules_this_timestep) > 0
            if has_body_contact:
                contact_tracker.timesteps_with_non_foot_contact += 1

            # M2: bad contact = body contact OR foot slip
            if has_body_contact or foot_slip_this_timestep:
                contact_tracker.timesteps_with_bad_contact += 1

        # Control step
        if time >= last_control_time + control_step:
            last_control_time = math.floor(time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)

        mujoco.mj_step(model, data)

        # Render if viewer is active
        if viewer is not None:
            # Update camera to follow robot if tracking enabled
            if track_camera:
                cam = viewer._viewer_backend.cam
                # Follow robot position
                cam.lookat[0] = current_pos[0]
                cam.lookat[1] = current_pos[1]
                cam.lookat[2] = current_pos[2] + 0.05  # Slightly above robot center

            status = viewer.render()
            if status == -1:  # Window closed
                if verbose:
                    print(f"\nViewer closed at {time:.2f}s (simulation continues...)")
                break

    # Record final position
    loco_tracker.end_position = tuple(data.xpos[core_body_id])

    # Close viewer if it was open
    if viewer is not None:
        viewer.close_viewer()

    # Calculate all locomotion metrics using measurement time (excluding warmup)
    measurement_time = simulation_time - warmup_time
    metrics = calculate_locomotion_metrics(loco_tracker, contact_tracker, measurement_time)

    if verbose:
        print(f"\n{'='*60}")
        print(f"LOCOMOTION METRICS SUMMARY")
        print(f"{'='*60}")

        print(f"\n--- Simulation Info ---")
        print(f"Total timesteps (measured): {contact_tracker.total_timesteps}")
        print(f"Simulation time: {simulation_time}s (warmup: {warmup_time}s, measured: {measurement_time}s)")
        print(f"Robot mass: {metrics.robot_mass:.4f} kg")

        print(f"\n--- FITNESS METRICS ---")
        print(f"Distance (XY displacement): {metrics.distance:.4f} m")
        print(f"Contact M1 (body contact): {metrics.contact_metric_1:.4f} ({metrics.contact_metric_1*100:.2f}%)")
        print(f"Contact M2 (body contact OR foot slip): {metrics.contact_metric_2:.4f} ({metrics.contact_metric_2*100:.2f}%)")

        print(f"\n--- ANALYSIS METRICS ---")
        print(f"Cost of Transport: {metrics.cost_of_transport:.4f}")
        print(f"Straightness: {metrics.straightness:.4f} ({metrics.straightness*100:.2f}%)")
        print(f"Total path length: {metrics.total_path_length:.4f} m")
        print(f"Total energy: {metrics.total_energy:.4f} J")

        print(f"\n--- STABILITY ---")
        print(f"Height variance: {metrics.stability_height_variance:.6f} m²")
        print(f"Roll variance: {metrics.stability_roll_variance:.6f} rad²")
        print(f"Pitch variance: {metrics.stability_pitch_variance:.6f} rad²")
        print(f"Combined stability: {metrics.stability_combined:.6f} (lower=better)")

        print(f"\n--- Contact Summary ---")
        print(f"Total ground contacts: {len(contact_tracker.ground_contacts)}")
        foot_contacts = len(contact_tracker.ground_contacts) - len(contact_tracker.non_foot_ground_contacts)
        print(f"  Foot contacts: {foot_contacts}")
        print(f"  Non-foot contacts: {len(contact_tracker.non_foot_ground_contacts)}")
        print(f"  Non-foot modules: {metrics.num_non_foot_modules}")
        print(f"  Timesteps with body contact: {contact_tracker.timesteps_with_non_foot_contact}")
        print(f"  Timesteps with bad contact (body OR slip): {contact_tracker.timesteps_with_bad_contact}")

    return contact_tracker, metrics


def main() -> None:
    """Run locomotion metrics demo."""
    print("Locomotion Metrics for Modular Robots")
    print("=" * 40)

    # Preset CPG parameters for spider (from previous evolution)
    SPIDER_CPG_PARAMS = [
        0.97560332, -0.98052375, 0.70084728, -0.06435287,
        0.69862624, -0.05989215, -0.99993381, 0.99979145,
        0.99602341, 0.64765969, 0.72665882, -0.90461619
    ]

    AVAILABLE_ROBOTS = [
        "spider", "gecko", "ant", "salamander", "snake", "turtle",
        "babya", "babyb", "blokky", "garrix", "insect", "linkin",
        "longleg", "penguin", "pentapod", "queen", "squarish",
        "stingray", "tinlicker", "ww", "zappa", "park"
    ]

    print("\nAvailable robots:")
    for i, name in enumerate(AVAILABLE_ROBOTS, 1):
        print(f"  {i:2}. {name}")

    print("\nEnter robot name (or number), or press Enter for 'spider':")
    choice = input("> ").strip()

    if not choice:
        robot_name = "spider"
    elif choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(AVAILABLE_ROBOTS):
            robot_name = AVAILABLE_ROBOTS[idx]
        else:
            robot_name = "spider"
    elif choice.lower() in AVAILABLE_ROBOTS:
        robot_name = choice.lower()
    else:
        robot_name = "spider"

    print(f"\nSimulation time in seconds (default 10):")
    time_input = input("> ").strip()
    sim_time = int(time_input) if time_input.isdigit() else 10

    # Ask about CPG parameters
    cpg_params = None
    if robot_name == "spider":
        print("\nUse preset spider CPG parameters? (y/n, default y):")
        use_preset = input("> ").strip().lower()
        if use_preset != "n":
            cpg_params = SPIDER_CPG_PARAMS

    contact_tracker, metrics = simulate_with_contact_detection(
        robot_name=robot_name,
        simulation_time=sim_time,
        verbose=True,
        cpg_params=cpg_params,
    )

    # Print fitness calculation example (power formula - used in experiments)
    print(f"\n{'='*60}")
    print("FITNESS CALCULATION (power: distance * (1 - contact)^λ)")
    print(f"{'='*60}")
    for lambda_val in [0.0, 1.0, 2.0, 5.0]:
        fitness_m1 = metrics.distance * math.pow(1 - metrics.contact_metric_1, lambda_val)
        fitness_m2 = metrics.distance * math.pow(1 - metrics.contact_metric_2, lambda_val)
        print(f"\nλ = {lambda_val}:")
        print(f"  Using M1: fitness = {metrics.distance:.4f} × (1 - {metrics.contact_metric_1:.4f})^{lambda_val} = {fitness_m1:.4f}")
        print(f"  Using M2: fitness = {metrics.distance:.4f} × (1 - {metrics.contact_metric_2:.4f})^{lambda_val} = {fitness_m2:.4f}")


if __name__ == "__main__":
    main()


