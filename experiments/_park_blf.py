"""Full BLF analysis of park morphology."""
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot.body.base import ActiveHinge, Core
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from blf import BodyLimbFinder, JointType, BLFCpgNetworkGenerator
from contact_detection import active_hinges_to_cpg_network_structure_internal_only

body = modular_robots_v1.get("park")
hinges = body.find_modules_of_type(ActiveHinge)
result = BodyLimbFinder(body).analyze()
gen = BLFCpgNetworkGenerator(result)
structure, mapping = gen.generate()

spine = [(i, result.nodes[i]) for i, jt in result.articulations.items() if jt == JointType.SPINE]
hips = [(i, result.nodes[i]) for i, jt in result.articulations.items() if jt == JointType.HIP]
knees = [(i, result.nodes[i]) for i, jt in result.articulations.items() if jt == JointType.KNEE]

# Foot detection (Rule 6)
def has_hinge_ancestor(mod):
    p = mod.parent
    while p is not None:
        if isinstance(p, ActiveHinge):
            return True
        p = p.parent
    return False

def has_hinge_descendant(mod):
    for child in mod.children.values():
        if isinstance(child, ActiveHinge):
            return True
        if has_hinge_descendant(child):
            return True
    return False

all_modules = []
queue = [body.core]
visited = set()
while queue:
    m = queue.pop(0)
    if id(m) in visited:
        continue
    visited.add(id(m))
    all_modules.append(m)
    for c in m.children.values():
        queue.append(c)

foot_modules = []
for m in all_modules:
    is_foot = (has_hinge_ancestor(m) or isinstance(m, ActiveHinge)) and not has_hinge_descendant(m)
    if is_foot:
        foot_modules.append(m)

print("## 1. Body Part Counts")
print()
print(f"- Total active hinges: {len(hinges)}")
print(f"- Spine joints: {len(spine)}")
print(f"- Hips: {len(hips)}")
print(f"- Knees: {len(knees)}")
print(f"- Foot modules: {len(foot_modules)}")

print()
print("## 2. Per-Limb Breakdown")
print()
print("| Limb | Modules | Hinges | Breakdown |")
print("|------|---------|--------|-----------|")
for li, limb in enumerate(result.limbs):
    sorted_limb = sorted(limb, key=lambda x: result.nodes[x].distance_from_body)
    limb_hinges = []
    for idx in sorted_limb:
        n = result.nodes[idx]
        if isinstance(n.module, ActiveHinge):
            limb_hinges.append(n.joint_type.name + "(node " + str(idx) + ")")
    total_mods = len(limb)
    total_h = len(limb_hinges)
    breakdown = ", ".join(limb_hinges) if limb_hinges else "no hinges"
    print(f"| {li} | {total_mods} | {total_h} | {breakdown} |")

print()
print("## 3. Spine Structure")
print()
for idx, node in spine:
    neighbors = node.neighbors
    neighbor_strs = []
    for nb in neighbors:
        nb_node = result.nodes[nb]
        mtype = type(nb_node.module).__name__.replace("V1", "")
        part = nb_node.part_type.name
        neighbor_strs.append("node " + str(nb) + " (" + mtype + ", " + part + ")")
    print("- Spine node " + str(idx) + ": neighbors = " + ", ".join(neighbor_strs))

# Check parallel vs series
spine_parents = {}
for idx, node in spine:
    parent = node.module.parent
    if parent is not None:
        parent_idx = None
        for ni, nd in enumerate(result.nodes):
            if nd.module is parent:
                parent_idx = ni
                break
        if parent_idx is not None:
            spine_parents.setdefault(parent_idx, []).append(idx)

print()
parallel = False
for parent_idx, children in spine_parents.items():
    if len(children) > 1:
        print("- Spine joints " + str(children) + " share parent node " + str(parent_idx) + " (PARALLEL)")
        parallel = True
if not parallel:
    print("- All spine joints are in series (no parallel siblings)")

print()
print("## 4. Body Partition")
print()
print("**Body modules:**")
print()
print("| Node | Type | Joint |")
print("|------|------|-------|")
for idx in result.body_nodes:
    n = result.nodes[idx]
    mtype = type(n.module).__name__.replace("V1", "")
    jt = n.joint_type.name if n.joint_type != JointType.UNCLASSIFIED else "-"
    print("| " + str(idx) + " | " + mtype + " | " + jt + " |")

print()
print("**Brick clusters triggering Rule 2 (>2 touching hinges):**")
print()
visited_cl = set()
for start_idx in range(len(result.nodes)):
    if start_idx in visited_cl:
        continue
    node = result.nodes[start_idx]
    if isinstance(node.module, ActiveHinge) or isinstance(node.module, Core):
        continue
    cluster = []
    cq = [start_idx]
    while cq:
        ci = cq.pop(0)
        if ci in visited_cl:
            continue
        cn = result.nodes[ci]
        if isinstance(cn.module, ActiveHinge) or isinstance(cn.module, Core):
            continue
        visited_cl.add(ci)
        cluster.append(ci)
        for nb in cn.neighbors:
            if nb not in visited_cl:
                cq.append(nb)
    if not cluster:
        continue
    touching = set()
    for ci in cluster:
        for nb in result.nodes[ci].neighbors:
            if isinstance(result.nodes[nb].module, ActiveHinge):
                touching.add(nb)
    triggered = len(touching) > 2
    status = "BODY (Rule 2)" if triggered else "not body"
    print("- Cluster nodes " + str(cluster) + " (" + str(len(cluster)) + " bricks): " + str(len(touching)) + " touching hinges -> " + status)

print()
print("## 5. Coupling Counts")
print()

uc_struct, _ = active_hinges_to_cpg_network_structure_internal_only(hinges)
nb_struct, _ = active_hinges_to_cpg_network_structure_neighbor(hinges)

ss = hh = hs = kc = 0
for pair in structure.connections:
    lo = pair.cpg_index_lowest.index
    hi = pair.cpg_index_highest.index
    lo_node = gen._cpg_to_node.get(lo)
    hi_node = gen._cpg_to_node.get(hi)
    lo_type = result.nodes[lo_node].joint_type
    hi_type = result.nodes[hi_node].joint_type
    types = {lo_type, hi_type}
    if types == {JointType.SPINE}:
        ss += 1
    elif types == {JointType.HIP}:
        hh += 1
    elif types == {JointType.SPINE, JointType.HIP}:
        hs += 1
    elif JointType.KNEE in types:
        kc += 1

print("| Topology | Internal | Coupling | Total params |")
print("|----------|----------|----------|--------------|")
print("| Uncoupled | " + str(uc_struct.num_cpgs) + " | " + str(len(uc_struct.connections)) + " | " + str(uc_struct.num_connections) + " |")
print("| Neighbour | " + str(nb_struct.num_cpgs) + " | " + str(len(nb_struct.connections)) + " | " + str(nb_struct.num_connections) + " |")
print("| Structured (BLF) | " + str(structure.num_cpgs) + " | " + str(len(structure.connections)) + " | " + str(structure.num_connections) + " |")
print()
print("BLF coupling breakdown: " + str(ss) + " spine-spine + " + str(hh) + " hip-hip + " + str(hs) + " hip-spine + " + str(kc) + " knee-chain = " + str(len(structure.connections)))
