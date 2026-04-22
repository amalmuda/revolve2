"""Verify coupling connections are correct for BLF rules."""
import sys
sys.path.insert(0, ".")
from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.body.base import ActiveHinge, Core
from revolve2.modular_robot.body._right_angles import RightAngles
from blf import analyze_robot, JointType
from contact_detection import active_hinges_to_cpg_network_structure_blf

R0 = RightAngles.DEG_0
R90 = RightAngles.DEG_90

total_ok = 0
total_fail = 0

def print_coupling(name, body):
    global total_ok, total_fail
    result = analyze_robot(body)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)

    hinge_to_node = {}
    for n in result.nodes:
        if isinstance(n.module, ActiveHinge):
            hinge_to_node[id(n.module)] = n

    cpg_to_info = {}
    for cpg_idx, hinge_mod in mapping:
        node = hinge_to_node[id(hinge_mod)]
        cpg_to_info[cpg_idx] = (node.index, node.joint_type.name, node.limb_id)

    print("=== %s ===" % name)
    print("Hinges: %d, Connections: %d, Total params: %d" % (
        len(hinges), len(cpg_struct.connections), cpg_struct.num_connections))
    print()

    print("CPG -> Hinge classification:")
    for cpg_idx in sorted(cpg_to_info.keys()):
        nidx, jt, lid = cpg_to_info[cpg_idx]
        loc = "limb%d" % lid if lid >= 0 else "body"
        print("  CPG%d = M%d %s (%s)" % (cpg_idx, nidx, jt, loc))
    print()

    spine_spine = []
    hip_hip = []
    hip_spine = []
    knee_chain = []
    other = []

    for conn in cpg_struct.connections:
        i, j = conn.cpg_index_highest.index, conn.cpg_index_lowest.index
        ni, jti, li = cpg_to_info[i]
        nj, jtj, lj = cpg_to_info[j]
        label = "  CPG%d(M%d %s) <-> CPG%d(M%d %s)" % (i, ni, jti, j, nj, jtj)

        if jti == "SPINE" and jtj == "SPINE":
            spine_spine.append(label + "  [spine-spine]")
        elif jti == "HIP" and jtj == "HIP":
            hip_hip.append(label + "  [hip-hip]")
        elif (jti == "HIP" and jtj == "SPINE") or (jti == "SPINE" and jtj == "HIP"):
            hip_spine.append(label + "  [hip-spine]")
        elif li == lj and li >= 0:
            knee_chain.append(label + "  [limb%d chain]" % li)
        else:
            other.append(label + "  [???]")

    for group, gname in [(spine_spine, "Spine-Spine (all-to-all)"),
                          (hip_hip, "Hip-Hip (all-to-all)"),
                          (hip_spine, "Hip-Spine (nearest)"),
                          (knee_chain, "Knee chain (within limb)"),
                          (other, "UNEXPECTED")]:
        if group:
            print("  %s:" % gname)
            for line in group:
                print("  %s" % line)

    # Verify rules
    S = sum(1 for _, jt, _ in cpg_to_info.values() if jt == "SPINE")
    H = sum(1 for _, jt, _ in cpg_to_info.values() if jt == "HIP")
    K = sum(1 for _, jt, _ in cpg_to_info.values() if jt == "KNEE")
    exp_ss = S * (S - 1) // 2
    exp_hh = H * (H - 1) // 2

    ok = True
    if len(spine_spine) != exp_ss:
        print("  ERROR: spine-spine=%d expected=%d" % (len(spine_spine), exp_ss)); ok = False
    if len(hip_hip) != exp_hh:
        print("  ERROR: hip-hip=%d expected=%d" % (len(hip_hip), exp_hh)); ok = False
    if other:
        print("  ERROR: %d unexpected connections" % len(other)); ok = False
    if S > 0:
        hips_with_spine = set()
        for conn in cpg_struct.connections:
            i, j = conn.cpg_index_highest.index, conn.cpg_index_lowest.index
            jti = cpg_to_info[i][1]; jtj = cpg_to_info[j][1]
            if jti == "HIP" and jtj == "SPINE": hips_with_spine.add(i)
            if jtj == "HIP" and jti == "SPINE": hips_with_spine.add(j)
        if len(hips_with_spine) != H:
            print("  ERROR: %d/%d hips connected to spine" % (len(hips_with_spine), H)); ok = False

    # Verify knee chains: each knee connects to exactly 1 previous hinge in same limb
    for cpg_idx in sorted(cpg_to_info.keys()):
        nidx, jt, lid = cpg_to_info[cpg_idx]
        if jt != "KNEE":
            continue
        same_limb_conns = 0
        for conn in cpg_struct.connections:
            i, j = conn.cpg_index_highest.index, conn.cpg_index_lowest.index
            if i == cpg_idx or j == cpg_idx:
                other_cpg = j if i == cpg_idx else i
                other_lid = cpg_to_info[other_cpg][2]
                if other_lid == lid:
                    same_limb_conns += 1
        if same_limb_conns != 1:
            print("  ERROR: knee CPG%d has %d same-limb connections (expected 1)" % (cpg_idx, same_limb_conns))
            ok = False

    status = "ALL CORRECT" if ok else "ISSUES FOUND"
    print("  Result: %s" % status)
    print()
    if ok:
        total_ok += 1
    else:
        total_fail += 1


# === Custom blobs ===

# Web blob
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br1 = BrickV1(R0); h1.attachment = br1
h2 = ActiveHingeV1(R0); br1.front = h2
br2 = BrickV1(R0); h2.attachment = br2
h3 = ActiveHingeV1(R90); br1.left = h3
br3 = BrickV1(R0); h3.attachment = br3
h4 = ActiveHingeV1(R90); br1.right = h4
br4 = BrickV1(R0); h4.attachment = br4
h5 = ActiveHingeV1(R0); br2.front = h5
h6 = ActiveHingeV1(R90); br2.left = h6
h7 = ActiveHingeV1(R90); br3.front = h7
h8 = ActiveHingeV1(R0); br4.front = h8
print_coupling("WEB BLOB", b)

# Chain with branches
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br1 = BrickV1(R0); h1.attachment = br1
h2 = ActiveHingeV1(R0); br1.front = h2
br2 = BrickV1(R0); h2.attachment = br2
h3 = ActiveHingeV1(R0); br2.front = h3
br3 = BrickV1(R0); h3.attachment = br3
h4 = ActiveHingeV1(R90); br2.left = h4
br4 = BrickV1(R0); h4.attachment = br4
h5 = ActiveHingeV1(R0); br4.front = h5
h6 = ActiveHingeV1(R90); br1.left = h6
print_coupling("CHAIN WITH BRANCHES", b)

# Centipede
b = BodyV1(); cur = b.core
for i in range(5):
    br = BrickV1(R0); cur.front = br
    br.left = ActiveHingeV1(R90); br.right = ActiveHingeV1(R90)
    cur = br
print_coupling("CENTIPEDE", b)

# Standard robots
from revolve2.standards.modular_robots_v1 import get
print_coupling("SPIDER", get("spider"))
print_coupling("GECKO", get("gecko"))
print_coupling("QUEEN", get("queen"))
print_coupling("ZAPPA", get("zappa"))
print_coupling("SALAMANDER", get("salamander"))

print("=" * 60)
print("TOTAL: %d correct, %d issues" % (total_ok, total_fail))
print("=" * 60)
