"""Analyze all 22 original robots with the new BLF rules."""
import math
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot.body.base import ActiveHinge, Core
from blf import BodyLimbFinder, JointType, BLFCpgNetworkGenerator

ROBOTS = [
    "spider", "gecko", "babya", "babyb", "ant", "salamander",
    "blokky", "park", "garrix", "insect", "linkin", "longleg",
    "penguin", "pentapod", "queen", "squarish", "snake", "stingray",
    "tinlicker", "turtle", "ww", "zappa",
]

for name in ROBOTS:
    body = modular_robots_v1.get(name)
    hinges = body.find_modules_of_type(ActiveHinge)
    result = BodyLimbFinder(body).analyze()

    # Count modules via BFS
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
    n_core = sum(1 for m in all_modules if isinstance(m, Core))
    n_hinge = len(hinges)
    n_brick = len(all_modules) - n_core - n_hinge

    # BLF joint counts
    spine = [i for i, jt in result.articulations.items() if jt == JointType.SPINE]
    hips = [i for i, jt in result.articulations.items() if jt == JointType.HIP]
    knees = [i for i, jt in result.articulations.items() if jt == JointType.KNEE]

    # Hinge orientations (path order via BFS)
    hinge_info = []
    q2 = [(body.core, "core")]
    v2 = set()
    while q2:
        mod, path = q2.pop(0)
        if id(mod) in v2:
            continue
        v2.add(id(mod))
        if isinstance(mod, ActiveHinge):
            q = mod.orientation
            angle = round(math.degrees(2 * math.atan2(float(q.x), float(q.w))))
            hinge_info.append((path, angle))
        for slot, child in mod.children.items():
            sn = ""
            if hasattr(mod, "FRONT") and slot == mod.FRONT: sn = "front"
            elif hasattr(mod, "RIGHT") and slot == mod.RIGHT: sn = "right"
            elif hasattr(mod, "BACK") and slot == mod.BACK: sn = "back"
            elif hasattr(mod, "LEFT") and slot == mod.LEFT: sn = "left"
            elif hasattr(mod, "ATTACHMENT") and slot == mod.ATTACHMENT: sn = "att"
            else: sn = f"s{slot}"
            q2.append((child, f"{path}.{sn}"))

    # CPG network
    gen = BLFCpgNetworkGenerator(result)
    structure, mapping = gen.generate()
    n_connections = len(structure.connections)
    n_internal = structure.num_cpgs
    n_external = n_connections - n_internal

    # Brick cluster analysis (Rule 2)
    visited_cl = set()
    clusters_info = []
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
        clusters_info.append((len(cluster), len(touching), len(touching) > 2))

    # Body node details
    body_detail = []
    for idx in result.body_nodes:
        n = result.nodes[idx]
        mtype = type(n.module).__name__.replace("V1", "")
        jt = n.joint_type.name if n.joint_type != JointType.UNCLASSIFIED else "-"
        body_detail.append(f"{mtype}({jt})")

    print(f"=== {name.upper()} ===")
    print(f"Modules: {len(all_modules)} (1 core, {n_hinge} hinges, {n_brick} bricks)")
    print(f"Body nodes ({len(result.body_nodes)}): {', '.join(body_detail)}")
    print(f"Brick clusters: {clusters_info}")
    print(f"Limbs: {len(result.limbs)}")
    print(f"Joints: {len(spine)} spine, {len(hips)} hip, {len(knees)} knee")
    print(f"Sym groups: {len(result.symmetric_groups)}")
    print(f"CPG: {n_internal} internal + {n_external} coupling = {n_connections} params")
    print(f"Hinges:")
    for path, angle in hinge_info:
        # Find node index and joint type
        idx = None
        for ni, nd in enumerate(result.nodes):
            if isinstance(nd.module, ActiveHinge):
                # Match by checking the path... approximate by order
                pass
        print(f"  {path}: {angle} deg")
    print(f"Limb details:")
    for li, limb in enumerate(result.limbs):
        sorted_limb = sorted(limb, key=lambda x: result.nodes[x].distance_from_body)
        parts = []
        for idx in sorted_limb:
            n = result.nodes[idx]
            if isinstance(n.module, ActiveHinge):
                parts.append(f"H({n.joint_type.name})")
            else:
                parts.append(type(n.module).__name__.replace("V1", ""))
        print(f"  Limb {li}: {' -> '.join(parts)}")
    print()
