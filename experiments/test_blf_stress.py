"""Stress test for BLF rules on many weird robot configurations."""
import sys, math
sys.path.insert(0, ".")
import numpy as np
from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.body._right_angles import RightAngles
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from blf import analyze_robot, JointType
from contact_detection import active_hinges_to_cpg_network_structure_blf

R0 = RightAngles.DEG_0
R90 = RightAngles.DEG_90

passed = 0
failed = 0

def test(name, body, expect_S=None, expect_H=None, expect_K=None):
    global passed, failed
    try:
        result = analyze_robot(body)
        hinges = body.find_modules_of_type(ActiveHinge)
        assert result.nodes[0].part_type.name == "BODY", "core not BODY"
        S = sum(1 for n in result.nodes if n.joint_type == JointType.SPINE)
        H = sum(1 for n in result.nodes if n.joint_type == JointType.HIP)
        K = sum(1 for n in result.nodes if n.joint_type == JointType.KNEE)
        uncl = sum(1 for n in result.nodes if isinstance(n.module, ActiveHinge) and n.joint_type == JointType.UNCLASSIFIED)
        assert uncl == 0, f"{uncl} unclassified"
        assert S + H + K == len(hinges), f"S+H+K={S+H+K} != {len(hinges)}"
        if expect_S is not None: assert S == expect_S, f"spine {S} != expected {expect_S}"
        if expect_H is not None: assert H == expect_H, f"hip {H} != expected {expect_H}"
        if expect_K is not None: assert K == expect_K, f"knee {K} != expected {expect_K}"
        if len(hinges) > 0:
            cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
            params = np.zeros(cpg.num_connections)
            brain = BrainCpgNetworkStatic.uniform_from_params(
                params=params, cpg_network_structure=cpg,
                initial_state_uniform=math.sqrt(2)*0.5, output_mapping=mp)
            robot = ModularRobot(body=body, brain=brain)
        print(f"  PASS  {name:45s} h={len(hinges):2d} S={S} H={H} K={K} limbs={len(result.limbs)}")
        passed += 1
    except Exception as e:
        print(f"  FAIL  {name:45s} {e}")
        failed += 1


print("=" * 80)
print("STRESS TESTS: WEIRD ROBOT CONFIGURATIONS")
print("=" * 80)

# 1. Core only
b = BodyV1()
test("core_only", b, 0, 0, 0)

# 2. Core + 1 hinge
b = BodyV1(); b.core.front = ActiveHingeV1(R0)
test("core_1hinge", b, 0, 1, 0)

# 3. Core + 1 brick (no hinges)
b = BodyV1(); b.core.front = BrickV1(R0)
test("core_1brick", b, 0, 0, 0)

# 4. Long snake: 8 hinges chained
b = BodyV1(); cur = b.core
for i in range(8):
    h = ActiveHingeV1(R0); cur.front = h; cur = h
test("snake_8h", b, 0, 1, 7)

# 5. Star: 4 hinges directly on core
b = BodyV1()
b.core.front = ActiveHingeV1(R0); b.core.back = ActiveHingeV1(R0)
b.core.left = ActiveHingeV1(R90); b.core.right = ActiveHingeV1(R90)
test("star_4h", b, 0, 4, 0)

# 6. One deep leg: h-b-h-b-h-b-h
b = BodyV1(); cur = b.core
for i in range(4):
    h = ActiveHingeV1(R0); cur.front = h
    br = BrickV1(R0); h.attachment = br; cur = br
test("deep_leg_4h", b, 0, 1, 3)

# 7. T-junction: core -> h -> brick with 3 hinges branching
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br = BrickV1(R0); h1.attachment = br
br.front = ActiveHingeV1(R0); br.left = ActiveHingeV1(R90); br.right = ActiveHingeV1(R90)
test("t_junction_3h_on_brick", b)

# 8. Y-shape: core -> h -> brick -> 2 hinges (brick has 3 connections: parent h + 2 child h)
# But only 2 HINGE neighbors (the 2 children). Parent is also a hinge though!
# So: parent hinge + 2 child hinges = 3 hinges touching. Should this brick be body?
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br = BrickV1(R0); h1.attachment = br
br.left = ActiveHingeV1(R90); br.right = ActiveHingeV1(R90)
# brick has h1 (parent) + 2 child hinges = 3 hinges touching -> body!
# That means h1 is between core(body) and brick(body) -> spine
test("y_shape_3h_touching", b)

# 9. Big blob: 4 bricks, 4 hinges branching off
b = BodyV1()
b1 = BrickV1(R0); b.core.front = b1
b2 = BrickV1(R0); b1.front = b2
b3 = BrickV1(R0); b2.left = b3
b4 = BrickV1(R0); b2.right = b4
b1.left = ActiveHingeV1(R90)
b3.front = ActiveHingeV1(R0)
b4.front = ActiveHingeV1(R0)
b2.front = ActiveHingeV1(R0)
test("brick_blob_4h", b)

# 10. Two clusters connected by 2 spine hinges
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br1 = BrickV1(R0); h1.attachment = br1
br1.front = ActiveHingeV1(R0); br1.left = ActiveHingeV1(R90); br1.right = ActiveHingeV1(R90)
h2 = ActiveHingeV1(R0); b.core.back = h2
br2 = BrickV1(R0); h2.attachment = br2
br2.front = ActiveHingeV1(R0); br2.left = ActiveHingeV1(R90); br2.right = ActiveHingeV1(R90)
test("two_clusters_2spine", b)

# 11. 4 legs x 3 hinges
b = BodyV1()
for slot in ["front", "back", "left", "right"]:
    h1 = ActiveHingeV1(R90 if slot in ["left","right"] else R0)
    setattr(b.core, slot, h1)
    br1 = BrickV1(R0); h1.attachment = br1
    h2 = ActiveHingeV1(R0); br1.front = h2
    br2 = BrickV1(R0); h2.attachment = br2
    h3 = ActiveHingeV1(R0); br2.front = h3
test("4x3_legs", b, 0, 4, 8)

# 12. Centipede: 5 body segments with 2 legs each
b = BodyV1(); cur = b.core
for i in range(5):
    br = BrickV1(R0); cur.front = br
    br.left = ActiveHingeV1(R90); br.right = ActiveHingeV1(R90)
    cur = br
test("centipede_5seg", b)

# 13. Hinge-on-hinge (no bricks between)
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
h2 = ActiveHingeV1(R90); h1.attachment = h2
h3 = ActiveHingeV1(R0); b.core.left = h3
test("hinge_on_hinge", b)

# 14. Brick with exactly 2 hinges (NOT body)
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
br = BrickV1(R0); h1.attachment = br
h2 = ActiveHingeV1(R0); br.front = h2
result = analyze_robot(b)
brick_is_body = False
for n in result.nodes:
    if isinstance(n.module, BrickV1):
        if n.part_type.name == "BODY":
            brick_is_body = True
if brick_is_body:
    print(f"  FAIL  {'brick_2h_NOT_body':45s} brick falsely classified as body!"); failed += 1
else:
    print(f"  PASS  {'brick_2h_NOT_body':45s} brick correctly NOT body"); passed += 1

# 15. Hinge with brick blob foot
b = BodyV1()
h = ActiveHingeV1(R0); b.core.front = h
br1 = BrickV1(R0); h.attachment = br1
br2 = BrickV1(R0); br1.front = br2
br3 = BrickV1(R0); br1.left = br3
test("hinge_brick_blob_foot", b, 0, 1, 0)

# 16. Asymmetric: 1h leg + 4h leg
b = BodyV1()
b.core.front = ActiveHingeV1(R0)
h1 = ActiveHingeV1(R0); b.core.back = h1
br = BrickV1(R0); h1.attachment = br
h2 = ActiveHingeV1(R0); br.front = h2
br2 = BrickV1(R0); h2.attachment = br2
h3 = ActiveHingeV1(R0); br2.front = h3
br3 = BrickV1(R0); h3.attachment = br3
h4 = ActiveHingeV1(R0); br3.front = h4
test("asym_1h_4h", b, 0, 2, 3)

# 17. Core -> brick -> hinge (brick is body-adjacent, not a foot)
b = BodyV1()
br = BrickV1(R0); b.core.front = br
h = ActiveHingeV1(R0); br.front = h
test("core_brick_hinge", b, 0, 1, 0)

# 18. Long spine connecting 2 clusters
b = BodyV1()
bc1 = BrickV1(R0); b.core.front = bc1
bc1.left = ActiveHingeV1(R90); bc1.right = ActiveHingeV1(R90); bc1.front = ActiveHingeV1(R0)
h1 = ActiveHingeV1(R0); b.core.back = h1
br_mid = BrickV1(R0); h1.attachment = br_mid
h2 = ActiveHingeV1(R0); br_mid.front = h2
br_mid2 = BrickV1(R0); h2.attachment = br_mid2
h3 = ActiveHingeV1(R0); br_mid2.front = h3
bc2 = BrickV1(R0); h3.attachment = bc2
bc2.front = ActiveHingeV1(R0); bc2.left = ActiveHingeV1(R90); bc2.right = ActiveHingeV1(R90)
test("long_spine_2clusters", b)

# 19. Hexapod-style: spine + 3 pairs of legs
b = BodyV1()
b.core.left = ActiveHingeV1(R90); b.core.right = ActiveHingeV1(R90)
h_sp = ActiveHingeV1(R90); b.core.back = h_sp
br = BrickV1(R0); h_sp.attachment = br
br.left = ActiveHingeV1(R90); br.right = ActiveHingeV1(R90)
h_sp2 = ActiveHingeV1(R90); br.front = h_sp2
br2 = BrickV1(R0); h_sp2.attachment = br2
br2.left = ActiveHingeV1(R90); br2.right = ActiveHingeV1(R90)
test("hexapod_6legs", b)

# 20. Long chain 6h (no branching)
b = BodyV1(); cur = b.core
for i in range(6):
    h = ActiveHingeV1(R0 if i%2==0 else R90)
    cur.front = h
    br = BrickV1(R0); h.attachment = br; cur = br
test("long_chain_6h", b, 0, 1, 5)

# 21. Core with only bricks, no hinges, lots of bricks
b = BodyV1()
b.core.front = BrickV1(R0)
b.core.front.front = BrickV1(R0)
b.core.front.left = BrickV1(R0)
b.core.back = BrickV1(R0)
test("core_only_bricks", b, 0, 0, 0)

# 22. Single hinge with nothing after it (bare hinge = foot)
b = BodyV1(); b.core.front = ActiveHingeV1(R0)
test("bare_hinge_foot", b, 0, 1, 0)

# 23. 2 hinges directly chained (h->h, no brick)
b = BodyV1()
h1 = ActiveHingeV1(R0); b.core.front = h1
h2 = ActiveHingeV1(R0); h1.attachment = h2
test("2h_chained", b, 0, 1, 1)

# 24. Massive: 4 clusters with 3 spine hinges between each
b = BodyV1()
# cluster 1 front
bc1 = BrickV1(R0); b.core.front = bc1
bc1.front = ActiveHingeV1(R0); bc1.left = ActiveHingeV1(R90); bc1.right = ActiveHingeV1(R90)
# 3 spines going back
h1 = ActiveHingeV1(R90); b.core.back = h1
br1 = BrickV1(R0); h1.attachment = br1
h2 = ActiveHingeV1(R90); br1.front = h2
br2 = BrickV1(R0); h2.attachment = br2
h3 = ActiveHingeV1(R90); br2.front = h3
# cluster 2 at end
bc2 = BrickV1(R0); h3.attachment = bc2
bc2.front = ActiveHingeV1(R0); bc2.left = ActiveHingeV1(R90); bc2.right = ActiveHingeV1(R90)
test("2clusters_3spine", b)

# === ALL 31 REVOLVE2 ROBOTS ===
print()
print("=" * 80)
print("ALL REVOLVE2 V1 ROBOTS")
print("=" * 80)
from revolve2.standards import modular_robots_v1
names = [n[:-3] for n in dir(modular_robots_v1) if n.endswith('_v1') and callable(getattr(modular_robots_v1, n))]
for name in sorted(names):
    try:
        body = getattr(modular_robots_v1, name + '_v1')()
        test(f"revolve2_{name}", body)
    except Exception as e:
        print(f"  FAIL  revolve2_{name:40s} {e}"); failed += 1

print()
print("=" * 80)
print(f"TOTAL: {passed} passed, {failed} failed")
print("=" * 80)
