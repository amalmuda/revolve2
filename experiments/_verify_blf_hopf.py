"""Verify BLF coupling is correctly used when building a Hopf brain."""
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1
from blf import BodyLimbFinder, JointType
from contact_detection import active_hinges_to_cpg_network_structure_blf
from hopf_brain import (
    hopf_structure_from_cpg_structure,
    BrainHopfStatic,
    BrainHopfPolarStatic,
)
import numpy as np

for robot_name in ["spider", "gecko"]:
    print(f"=== {robot_name.upper()} ===")
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)

    # Run BLF to get CPG structure
    result = BodyLimbFinder(body).analyze()
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)

    spine = [i for i, jt in result.articulations.items() if jt == JointType.SPINE]
    hips = [i for i, jt in result.articulations.items() if jt == JointType.HIP]
    knees = [i for i, jt in result.articulations.items() if jt == JointType.KNEE]
    print(f"Hinges: {len(hinges)}, Spine: {len(spine)}, Hips: {len(hips)}, Knees: {len(knees)}")
    print(f"BLF CPG: {cpg_struct.num_cpgs} oscillators, {len(cpg_struct.connections)} connections")

    # Convert to Hopf network structure
    hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)
    print(f"Hopf struct: {hopf_struct.num_oscillators} oscillators, {hopf_struct.num_connections} connections")

    # Verify same topology
    assert hopf_struct.num_oscillators == cpg_struct.num_cpgs
    assert len(hopf_struct.connections) == len(cpg_struct.connections)
    assert set(hopf_struct.connections) == set(cpg_struct.connections)
    print("  Topology matches ODE-CPG BLF structure: YES")

    # Build a Hopf brain (cartesian)
    n_cart = hopf_struct.num_oscillators + hopf_struct.num_connections
    params_cart = np.full(n_cart, 0.5)
    brain_cart = BrainHopfStatic.from_params(
        params=params_cart, network_structure=hopf_struct,
        output_mapping=mapping, omega=6.28,
    )
    print(f"  Cartesian Hopf brain: {n_cart} params")

    # Build a Hopf brain (polar)
    n_polar = hopf_struct.num_oscillators + 2 * hopf_struct.num_connections
    params_polar = np.concatenate([
        np.full(hopf_struct.num_oscillators, 0.5),
        np.full(hopf_struct.num_connections, 0.5),
        np.full(hopf_struct.num_connections, 0.0),
    ])
    brain_polar = BrainHopfPolarStatic.from_params(
        params=params_polar, network_structure=hopf_struct,
        output_mapping=mapping, omega=6.28,
    )
    print(f"  Polar Hopf brain: {n_polar} params")

    # Verify coupling matrices match the BLF connection set
    from contact_detection import active_hinges_to_cpg_network_structure_blf as _blf
    # Spot-check: for each BLF pair, non-zero entry in Hopf's coupling matrix
    K = brain_cart._K
    missing = 0
    for pair in cpg_struct.connections:
        i = pair.cpg_index_lowest.index
        j = pair.cpg_index_highest.index
        if K[i, j] == 0 or K[j, i] == 0:
            missing += 1
    print(f"  Cartesian coupling matrix: {missing} BLF pairs missing (expected 0)")

    W = brain_polar._W
    missing = 0
    for pair in cpg_struct.connections:
        i = pair.cpg_index_lowest.index
        j = pair.cpg_index_highest.index
        if W[i, j] == 0 or W[j, i] == 0:
            missing += 1
    print(f"  Polar weight matrix: {missing} BLF pairs missing (expected 0)")
    print()
