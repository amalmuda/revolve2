"""Sanity check counts for the 9 chosen robots."""
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from blf import BodyLimbFinder, JointType, BLFCpgNetworkGenerator
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)

robots = ["spider", "gecko", "babya", "babyb", "ant", "queen", "park", "insect", "snake"]

header = f"{'Robot':<10} {'Hinges':>6} {'Spine':>5} {'Hips':>4} {'Chain':>5} {'Limbs':>5} {'Sym':>3} | {'Uncoup':>6} {'Neigh':>5} {'BLF':>5} | Check"
print(header)
print("-" * len(header))

for name in robots:
    body = modular_robots_v1.get(name)
    hinges = body.find_modules_of_type(ActiveHinge)

    # BLF
    result = BodyLimbFinder(body).analyze()
    gen = BLFCpgNetworkGenerator(result)
    structure, mapping = gen.generate()

    spine = sum(1 for jt in result.articulations.values() if jt == JointType.SPINE)
    hips = sum(1 for jt in result.articulations.values() if jt == JointType.HIP)
    knees = sum(1 for jt in result.articulations.values() if jt == JointType.KNEE)
    blf_params = structure.num_connections

    # Uncoupled
    uc_struct, _ = active_hinges_to_cpg_network_structure_internal_only(hinges)
    uc_params = uc_struct.num_connections

    # Neighbor
    nb_struct, _ = active_hinges_to_cpg_network_structure_neighbor(hinges)
    nb_params = nb_struct.num_connections

    # Verify
    total_check = spine + hips + knees
    ok = "OK" if total_check == len(hinges) else f"FAIL ({total_check}!={len(hinges)})"

    print(f"{name:<10} {len(hinges):>6} {spine:>5} {hips:>4} {knees:>5} {len(result.limbs):>5} {len(result.symmetric_groups):>3} | {uc_params:>6} {nb_params:>5} {blf_params:>5} | {ok}")
