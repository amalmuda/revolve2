"""
Deep verification tests for the Core-Centric algorithm.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from core_centric import analyze_robot, CoreCentricAnalyzer, JointType


def run_deep_checks():
    print("=" * 70)
    print("DEEP CODE ANALYSIS - EDGE CASE TESTS")
    print("=" * 70)

    errors = []

    # ==========================================================================
    # TEST 1: LCA Path Finding - Same module
    # ==========================================================================
    print("\n--- Test 1: LCA - Same module as start and end ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    analyzer = CoreCentricAnalyzer(body)
    path = analyzer._find_path(body.core_v1, body.core_v1)
    print(f"Path from Core to Core: {len(path)} nodes")
    if len(path) != 0:
        errors.append("LCA: Path from module to itself should be empty")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 2: LCA Path Finding - Parent to child
    # ==========================================================================
    print("\n--- Test 2: LCA - Parent to immediate child ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    analyzer = CoreCentricAnalyzer(body)
    hinge = body.core_v1.front
    path = analyzer._find_path(body.core_v1, hinge)
    print(f"Path from Core to Hinge: {len(path)} nodes, types: {[type(n).__name__ for n in path]}")
    # Path should be empty (excludes start and end)
    if len(path) != 0:
        errors.append(f"LCA: Path from parent to immediate child should be empty, got {len(path)}")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 3: LCA Path Finding - Grandparent to grandchild
    # ==========================================================================
    print("\n--- Test 3: LCA - Core to grandchild hinge ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.front.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)
    analyzer = CoreCentricAnalyzer(body)
    grandchild = body.core_v1.front.attachment.front
    path = analyzer._find_path(body.core_v1, grandchild)
    print(f"Path from Core to grandchild: {len(path)} nodes, types: {[type(n).__name__ for n in path]}")
    # Path should include the hinge and brick in between
    # Core -> Hinge -> Brick -> grandchild, so path should be [Hinge, Brick]
    if len(path) != 2:
        errors.append(f"LCA: Path Core->grandchild should have 2 intermediate nodes, got {len(path)}")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 4: LCA Path Finding - Two siblings
    # ==========================================================================
    print("\n--- Test 4: LCA - Two sibling hinges ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)
    analyzer = CoreCentricAnalyzer(body)
    path = analyzer._find_path(body.core_v1.front, body.core_v1.back)
    print(f"Path between siblings: {len(path)} nodes, types: {[type(n).__name__ for n in path]}")
    # Siblings share Core as LCA, path should be empty because Core is not a hinge
    if len(path) != 0:
        errors.append(f"LCA: Path between siblings (LCA=Core) should be empty, got {len(path)}")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 5: Limb detection - duplicate prevention
    # ==========================================================================
    print("\n--- Test 5: Limb duplication check ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)
    result = analyze_robot(body)
    # Should have exactly 4 limbs, not duplicates
    print(f"Limbs found: {len(result.limbs)}, total hinges in limbs: {sum(len(l) for l in result.limbs)}")
    if len(result.limbs) != 4:
        errors.append(f"Limb duplication: Expected 4 limbs, got {len(result.limbs)}")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 6: Deep nested structure (potential stack overflow)
    # ==========================================================================
    print("\n--- Test 6: Deep nested structure (50 levels) ---")
    body = BodyV1()
    current_module = ActiveHingeV1(0.0)
    body.core_v1.front = current_module
    for i in range(50):
        brick = BrickV1(0.0)
        current_module.attachment = brick
        hinge = ActiveHingeV1(0.0)
        brick.front = hinge
        current_module = hinge

    try:
        result = analyze_robot(body)
        print(f"Total hinges: {len(result.all_hinges)}, Limbs: {len(result.limbs)}")
        if len(result.all_hinges) == 51:  # 1 initial + 50 in loop
            print("PASS")
        else:
            errors.append(f"Deep structure: Expected 51 hinges, got {len(result.all_hinges)}")
            print("FAIL")
    except RecursionError as e:
        errors.append(f"Deep structure: RecursionError - {e}")
        print("FAIL - RecursionError")

    # ==========================================================================
    # TEST 7: Empty body (no children on core)
    # ==========================================================================
    print("\n--- Test 7: Empty body (core only) ---")
    body = BodyV1()
    result = analyze_robot(body)
    print(f"Body modules: {len(result.body_modules)}, Hinges: {len(result.all_hinges)}")
    if len(result.body_modules) != 1 or len(result.all_hinges) != 0:
        errors.append("Empty body: Should have 1 body module and 0 hinges")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 8: All hinges classified (no UNCLASSIFIED)
    # ==========================================================================
    print("\n--- Test 8: All hinges classified check ---")
    from revolve2.standards.modular_robots_v1 import all as all_robots
    all_bodies = all_robots()
    unclassified_found = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)
        unclassified = [h for h, jt in result.joint_types.items() if jt == JointType.UNCLASSIFIED]
        if unclassified:
            errors.append(f"Robot {i} has {len(unclassified)} UNCLASSIFIED hinges")
            unclassified_found = True
            print(f"Robot {i}: FAIL - {len(unclassified)} unclassified")

    if not unclassified_found:
        print(f"All {len(all_bodies)} robots: PASS - no unclassified hinges")

    # ==========================================================================
    # TEST 9: Hinge count consistency
    # ==========================================================================
    print("\n--- Test 9: Hinge count consistency ---")
    inconsistent = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)

        # Count hinges in limbs
        limb_hinges = set()
        for limb in result.limbs:
            for h in limb.hinges:
                limb_hinges.add(id(h))

        # Count spine hinges
        spine_ids = set(id(h) for h in result.spine_hinges)

        # Check overlap (spine and limb should not overlap)
        overlap = limb_hinges & spine_ids
        if overlap:
            errors.append(f"Robot {i}: {len(overlap)} hinges are both spine AND in limbs")
            inconsistent = True

        # Check coverage
        all_ids = set(id(h) for h in result.all_hinges)
        covered_ids = limb_hinges | spine_ids
        uncovered = all_ids - covered_ids
        if uncovered:
            errors.append(f"Robot {i}: {len(uncovered)} hinges not in spine or limbs")
            inconsistent = True

    if not inconsistent:
        print(f"All {len(all_bodies)} robots: PASS - consistent hinge counts")

    # ==========================================================================
    # TEST 10: Joint type dictionary uses correct hinge objects
    # ==========================================================================
    print("\n--- Test 10: Joint type dictionary consistency ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)
    result = analyze_robot(body)
    # Verify all hinges in joint_types are actual hinges from the body
    all_hinge_ids = set(id(h) for h in result.all_hinges)
    joint_type_ids = set(id(h) for h in result.joint_types.keys())
    if all_hinge_ids != joint_type_ids:
        errors.append("Joint type dict doesn't match all_hinges")
        print("FAIL")
    else:
        print("PASS")

    # ==========================================================================
    # TEST 11: Verify spine hinges are actually on paths between body modules
    # ==========================================================================
    print("\n--- Test 11: Spine verification for body module paths ---")
    # Create a structure with 2 body modules and verify spine is correct
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)  # should be spine
    body.core_v1.front.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)  # limb
    body.core_v1.front.attachment.left = ActiveHingeV1(0.0)   # limb
    body.core_v1.front.attachment.right = ActiveHingeV1(0.0)  # limb
    # Now brick has 3 hinge children -> body module

    result = analyze_robot(body)
    print(f"Body modules: {len(result.body_modules)}, Spine hinges: {len(result.spine_hinges)}")

    # The hinge connecting Core to Brick should be spine
    spine_hinge = body.core_v1.front
    if spine_hinge in result.spine_hinges:
        print("PASS - connecting hinge is spine")
    else:
        errors.append("Spine verification: connecting hinge should be spine")
        print("FAIL")

    # ==========================================================================
    # TEST 12: Multiple body modules with complex spine
    # ==========================================================================
    print("\n--- Test 12: Complex spine with 3 body modules ---")
    body = BodyV1()
    # Core has limbs
    body.core_v1.front = ActiveHingeV1(0.0)  # limb
    body.core_v1.back = ActiveHingeV1(0.0)   # limb

    # Spine to first body brick
    body.core_v1.left = ActiveHingeV1(0.0)  # spine
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)  # spine to next
    body.core_v1.left.attachment.left = ActiveHingeV1(0.0)   # limb
    body.core_v1.left.attachment.right = ActiveHingeV1(0.0)  # limb

    # Second body brick
    body.core_v1.left.attachment.front.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front.attachment.front = ActiveHingeV1(0.0)  # limb
    body.core_v1.left.attachment.front.attachment.left = ActiveHingeV1(0.0)   # limb
    body.core_v1.left.attachment.front.attachment.right = ActiveHingeV1(0.0)  # limb

    result = analyze_robot(body)
    print(f"Body modules: {len(result.body_modules)}, Spine: {len(result.spine_hinges)}")

    if len(result.body_modules) == 3 and len(result.spine_hinges) == 2:
        print("PASS")
    else:
        errors.append(f"Complex spine: Expected 3 body, 2 spine; got {len(result.body_modules)} body, {len(result.spine_hinges)} spine")
        print("FAIL")

    # ==========================================================================
    # TEST 13: Limb ordering (hip should be closest to body)
    # ==========================================================================
    print("\n--- Test 13: Limb ordering verification ---")
    body = BodyV1()
    h1 = ActiveHingeV1(0.0)  # hip
    body.core_v1.front = h1
    h1.attachment = BrickV1(0.0)
    h2 = ActiveHingeV1(0.0)  # ankle
    h1.attachment.front = h2

    result = analyze_robot(body)

    # Verify h1 is HIP and h2 is ANKLE
    if result.joint_types.get(h1) == JointType.HIP and result.joint_types.get(h2) == JointType.ANKLE:
        print("PASS - correct ordering (h1=HIP, h2=ANKLE)")
    else:
        errors.append(f"Limb ordering: h1={result.joint_types.get(h1)}, h2={result.joint_types.get(h2)}")
        print("FAIL")

    # ==========================================================================
    # TEST 14: Very long limb (n=20) classification
    # ==========================================================================
    print("\n--- Test 14: Very long limb (n=20) ---")
    body = BodyV1()
    current = ActiveHingeV1(0.0)
    body.core_v1.front = current
    for _ in range(19):
        brick = BrickV1(0.0)
        current.attachment = brick
        hinge = ActiveHingeV1(0.0)
        brick.front = hinge
        current = hinge

    result = analyze_robot(body)
    types = {}
    for jt in result.joint_types.values():
        types[jt.name] = types.get(jt.name, 0) + 1

    print(f"n=20 limb: {types}")
    # Should have 1 HIP, 1 KNEE, 1 ANKLE, 17 LOCKED
    if types.get("HIP", 0) == 1 and types.get("KNEE", 0) == 1 and types.get("ANKLE", 0) == 1 and types.get("LOCKED", 0) == 17:
        print("PASS")
    else:
        errors.append(f"Long limb n=20: wrong classification {types}")
        print("FAIL")

    # ==========================================================================
    # TEST 15: Brick chain (bricks between hinges)
    # ==========================================================================
    print("\n--- Test 15: Multiple bricks between hinges ---")
    body = BodyV1()
    h1 = ActiveHingeV1(0.0)
    body.core_v1.front = h1

    # h1 -> brick -> brick -> h2
    b1 = BrickV1(0.0)
    h1.attachment = b1
    b2 = BrickV1(0.0)
    b1.front = b2
    h2 = ActiveHingeV1(0.0)
    b2.front = h2

    result = analyze_robot(body)
    print(f"Hinges: {len(result.all_hinges)}, Limbs: {len(result.limbs)}")

    # Both hinges should be in the same limb
    if len(result.limbs) == 1 and len(result.limbs[0]) == 2:
        print("PASS - both hinges in same limb")
    else:
        errors.append(f"Brick chain: Expected 1 limb with 2 hinges, got {len(result.limbs)} limbs")
        print("FAIL")

    # ==========================================================================
    # TEST 16: Limb branching off spine
    # ==========================================================================
    print("\n--- Test 16: Limb branching off spine hinge ---")
    # Core -> spine_hinge -> Brick (body) with limbs
    # What if spine_hinge itself has a child that's not on the path to body?
    body = BodyV1()
    spine_h = ActiveHingeV1(0.0)
    body.core_v1.front = spine_h
    spine_h.attachment = BrickV1(0.0)

    # Make brick a body module (3 hinge children)
    spine_h.attachment.front = ActiveHingeV1(0.0)
    spine_h.attachment.left = ActiveHingeV1(0.0)
    spine_h.attachment.right = ActiveHingeV1(0.0)

    result = analyze_robot(body)
    print(f"Body: {len(result.body_modules)}, Spine: {len(result.spine_hinges)}, Limbs: {len(result.limbs)}")

    # spine_h should be spine, brick's 3 children should be limbs
    if spine_h in result.spine_hinges:
        print("PASS - spine hinge correctly identified")
    else:
        errors.append("Limb off spine: spine hinge not identified")
        print("FAIL")

    # ==========================================================================
    # TEST 17: Amplitude bounds correctness
    # ==========================================================================
    print("\n--- Test 17: Amplitude bounds verification ---")
    from core_centric import AmplitudeBounds
    import math

    expected_bounds = {
        JointType.SPINE: (0.0, 2 * math.pi / 3),
        JointType.HIP: (0.0, math.pi / 2),
        JointType.KNEE: (0.0, math.pi / 6),
        JointType.ANKLE: (0.0, math.pi / 6),
        JointType.LOCKED: (0.0, 0.0),
    }

    bounds_ok = True
    for jt, expected in expected_bounds.items():
        actual = AmplitudeBounds.get_bounds(jt)
        if abs(actual[0] - expected[0]) > 0.001 or abs(actual[1] - expected[1]) > 0.001:
            errors.append(f"Amplitude bounds for {jt}: expected {expected}, got {actual}")
            bounds_ok = False

    if bounds_ok:
        print("PASS - all amplitude bounds correct")
    else:
        print("FAIL")

    # ==========================================================================
    # TEST 18: Stress test - wide robot (many limbs)
    # ==========================================================================
    print("\n--- Test 18: Wide robot (Core with many limbs) ---")
    # Core has 4 attachment points, each with a chain
    body = BodyV1()
    for direction in ['front', 'back', 'left', 'right']:
        h = ActiveHingeV1(0.0)
        setattr(body.core_v1, direction, h)
        h.attachment = BrickV1(0.0)
        h.attachment.front = ActiveHingeV1(0.0)
        h.attachment.left = ActiveHingeV1(0.0)
        h.attachment.right = ActiveHingeV1(0.0)

    result = analyze_robot(body)
    print(f"Hinges: {len(result.all_hinges)}, Limbs: {len(result.limbs)}")

    # 4 directions * 4 hinges each = 16 hinges
    # But each brick has 3 children, so 4 * (1 + 3) = 16 hinges total
    # Each limb from core: 1 hip + branch at brick -> 3 more limbs each
    # So 4 initial + 4*3 = 16 limbs? No, wait...
    # Actually: 4 limbs from core, each with 1 hinge, then brick has 3 children
    # So each of those 4 limbs has the first hinge, then branches into 3
    # That means 4 + 4*3 = 16 total limbs? Let me think...
    # Actually the first hinge is part of one limb, then branches at brick
    # So we get 4 limbs of length 1, then 12 limbs of length 1 from the bricks
    # Total: 4*1 + 4*3*1 = 4 + 12 = 16 limbs with 16 hinges

    if len(result.all_hinges) == 16:
        print(f"PASS - correct hinge count: {len(result.all_hinges)}")
    else:
        errors.append(f"Wide robot: Expected 16 hinges, got {len(result.all_hinges)}")
        print("FAIL")

    # ==========================================================================
    # TEST 19: Same hinge shouldn't appear in multiple limbs
    # ==========================================================================
    print("\n--- Test 19: No hinge in multiple limbs ---")
    # Check all 23 standard robots
    duplicate_found = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)
        all_limb_hinges = []
        for limb in result.limbs:
            all_limb_hinges.extend([id(h) for h in limb.hinges])

        if len(all_limb_hinges) != len(set(all_limb_hinges)):
            errors.append(f"Robot {i}: Same hinge appears in multiple limbs")
            duplicate_found = True

    if not duplicate_found:
        print(f"All {len(all_bodies)} robots: PASS - no duplicate hinges in limbs")

    # ==========================================================================
    # TEST 20: Verify total hinges = spine + limb hinges
    # ==========================================================================
    print("\n--- Test 20: Total = spine + limb hinges ---")
    mismatch_found = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)
        spine_count = len(result.spine_hinges)
        limb_count = sum(len(limb) for limb in result.limbs)
        total = len(result.all_hinges)

        if spine_count + limb_count != total:
            errors.append(f"Robot {i}: spine({spine_count}) + limbs({limb_count}) != total({total})")
            mismatch_found = True

    if not mismatch_found:
        print(f"All {len(all_bodies)} robots: PASS - spine + limbs = total")

    # ==========================================================================
    # TEST 21: Verify knee is between hip and ankle
    # ==========================================================================
    print("\n--- Test 21: Knee between hip and ankle ---")
    # For limbs with knee, verify order: hip -> ... -> knee -> ... -> ankle
    body = BodyV1()
    h1 = ActiveHingeV1(0.0)  # hip
    body.core_v1.front = h1
    h1.attachment = BrickV1(0.0)
    h2 = ActiveHingeV1(0.0)  # knee
    h1.attachment.front = h2
    h2.attachment = BrickV1(0.0)
    h3 = ActiveHingeV1(0.0)  # ankle
    h2.attachment.front = h3

    result = analyze_robot(body)

    if (result.joint_types.get(h1) == JointType.HIP and
        result.joint_types.get(h2) == JointType.KNEE and
        result.joint_types.get(h3) == JointType.ANKLE):
        print("PASS - correct order: HIP -> KNEE -> ANKLE")
    else:
        errors.append(f"Order wrong: h1={result.joint_types.get(h1)}, h2={result.joint_types.get(h2)}, h3={result.joint_types.get(h3)}")
        print("FAIL")

    # ==========================================================================
    # TEST 22: Spine doesn't contain body modules
    # ==========================================================================
    print("\n--- Test 22: Spine contains only hinges ---")
    spine_not_hinge = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)
        for s in result.spine_hinges:
            if not isinstance(s, ActiveHingeV1):
                errors.append(f"Robot {i}: spine contains non-hinge: {type(s)}")
                spine_not_hinge = True

    if not spine_not_hinge:
        print(f"All {len(all_bodies)} robots: PASS - spine contains only hinges")

    # ==========================================================================
    # TEST 23: Body modules are Core or Brick
    # ==========================================================================
    print("\n--- Test 23: Body modules are Core or Brick ---")
    from revolve2.modular_robot.body.v1 import CoreV1
    wrong_body = False
    for i, body in enumerate(all_bodies):
        result = analyze_robot(body)
        for m in result.body_modules:
            if not isinstance(m, (CoreV1, BrickV1)):
                errors.append(f"Robot {i}: body module is {type(m)}, should be Core or Brick")
                wrong_body = True

    if not wrong_body:
        print(f"All {len(all_bodies)} robots: PASS - body modules are Core/Brick")

    # ==========================================================================
    # TEST 24: Deterministic results
    # ==========================================================================
    print("\n--- Test 24: Deterministic results ---")
    body = BodyV1()
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)

    results = [analyze_robot(body) for _ in range(5)]
    all_same = True
    for r in results[1:]:
        if len(r.limbs) != len(results[0].limbs):
            all_same = False
        if len(r.spine_hinges) != len(results[0].spine_hinges):
            all_same = False

    if all_same:
        print("PASS - results are deterministic")
    else:
        errors.append("Non-deterministic results")
        print("FAIL")

    # ==========================================================================
    # SUMMARY
    # ==========================================================================
    print("\n" + "=" * 70)
    if errors:
        print(f"FOUND {len(errors)} ERRORS:")
        for e in errors:
            print(f"  - {e}")
    else:
        print("ALL EDGE CASE TESTS PASSED")
    print("=" * 70)

    return len(errors) == 0


if __name__ == "__main__":
    success = run_deep_checks()
    exit(0 if success else 1)
