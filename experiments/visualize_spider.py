"""
Visualize the Spider robot with Core-Centric algorithm and CPG network.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from core_centric import (
    analyze_robot, JointType, AmplitudeBounds, OptimizationMode,
    generate_cpg_network, print_cpg_network, CPGNetwork
)
import math


def create_spider() -> BodyV1:
    """Create spider with 2-hinge legs: Core + 4 legs (hip + ankle)."""
    body = BodyV1()

    # Front leg
    body.core_v1.front = ActiveHingeV1(0.0)
    body.core_v1.front.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)

    # Back leg
    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.back.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(0.0)

    # Left leg
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)

    # Right leg
    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)

    return body


def visualize_cpg_network(network: CPGNetwork) -> None:
    """Visualize the CPG network topology as ASCII art."""
    print("\n" + "=" * 70)
    print("CPG NETWORK TOPOLOGY (BLF)")
    print("=" * 70)

    # Group oscillators by type
    hips = network.hip_oscillators
    ankles = network.ankle_oscillators

    if len(hips) == 4 and len(ankles) == 4:
        # Spider layout
        print("""
                    HIP-HIP Couplings (fully connected)
                          +-----+-----+
                          |     |     |
                    +-----+-----+-----+-----+
                    |                       |
               [HIP 0]---[HIP 1]---[HIP 2]---[HIP 3]
                  |         |         |         |
                  |         |         |         | (ankle->hip only)
                  v         v         v         v
              [ANK 0]   [ANK 1]   [ANK 2]   [ANK 3]

        Legend:
          [HIP n] = Hip oscillator (amplitude [0, π/2])
          [ANK n] = Ankle oscillator (amplitude [0, π/6])
          ------- = Bidirectional phase coupling
        """)

    # Print actual coupling matrix
    print("\nCoupling Matrix:")
    n = len(network.oscillators)
    header = "     " + " ".join(f"{i:3}" for i in range(n))
    print(header)
    print("     " + "-" * (4 * n))

    coupling_set = {(c.from_oscillator, c.to_oscillator) for c in network.couplings}
    coupling_set.update({(c.to_oscillator, c.from_oscillator) for c in network.couplings})

    for i, osc in enumerate(network.oscillators):
        row = f"{i:3} |"
        for j in range(n):
            if i == j:
                row += "  . "
            elif (i, j) in coupling_set:
                row += "  X "
            else:
                row += "  - "
        row += f"  ({osc.joint_type.name[:3]})"
        print(row)


def visualize_spider():
    """Visualize the spider robot with joint classifications and CPG network."""

    spider = create_spider()
    result = analyze_robot(spider)

    print("=" * 70)
    print("SPIDER ROBOT - Core-Centric + CPG Network Analysis")
    print("=" * 70)

    # ASCII robot visualization
    print("""
                        ANKLE               ANKLE
                          o                   o
                          |                   |
                         HIP                 HIP
                          \\                 /
                           \\               /
                            \\             /
                    ANKLE----HIP--[CORE]--HIP----ANKLE
                            /             \\
                           /               \\
                          /                 \\
                         HIP                 HIP
                          |                   |
                          o                   o
                        ANKLE               ANKLE
    """)

    print("\n" + "=" * 70)
    print("JOINT CLASSIFICATION")
    print("=" * 70)

    print(f"\nTotal hinges: {len(result.all_hinges)}")
    print(f"Body modules: {len(result.body_modules)}")
    print(f"Spine hinges: {len(result.spine_hinges)}")
    print(f"Number of limbs: {len(result.limbs)}")

    # Generate CPG networks for both modes
    blf_network = generate_cpg_network(result, OptimizationMode.BLF)
    fo_network = generate_cpg_network(result, OptimizationMode.FULLY_OPEN)

    # Print BLF network details
    print_cpg_network(blf_network, OptimizationMode.BLF)

    # Visualize CPG topology
    visualize_cpg_network(blf_network)

    # Compare parameter counts
    print("\n" + "=" * 70)
    print("PARAMETER COUNT COMPARISON")
    print("=" * 70)

    blf_params = blf_network.get_parameter_count()
    fo_params = fo_network.get_parameter_count()

    print(f"\n{'Parameter':<15} {'FO':<10} {'BLF':<10} {'Reduction':<10}")
    print("-" * 45)
    print(f"{'Amplitudes':<15} {fo_params['amplitudes']:<10} {blf_params['amplitudes']:<10} "
          f"{(1 - blf_params['amplitudes']/fo_params['amplitudes'])*100:.1f}%")
    print(f"{'Offsets':<15} {fo_params['offsets']:<10} {blf_params['offsets']:<10} "
          f"{(1 - blf_params['offsets']/fo_params['offsets'])*100:.1f}%")
    print(f"{'Phase lags':<15} {fo_params['phases']:<10} {blf_params['phases']:<10} "
          f"{(1 - blf_params['phases']/max(1, fo_params['phases']))*100:.1f}%")
    print("-" * 45)
    print(f"{'TOTAL':<15} {fo_params['total']:<10} {blf_params['total']:<10} "
          f"{(1 - blf_params['total']/fo_params['total'])*100:.1f}%")

    # Amplitude bounds comparison
    print("\n" + "=" * 70)
    print("AMPLITUDE BOUNDS COMPARISON")
    print("=" * 70)

    print(f"\n{'Joint Type':<12} {'FO Bounds':<20} {'BLF Bounds':<20}")
    print("-" * 52)
    for jt in [JointType.HIP, JointType.ANKLE, JointType.KNEE, JointType.SPINE]:
        fo_bounds = AmplitudeBounds.get_bounds(jt, OptimizationMode.FULLY_OPEN)
        blf_bounds = AmplitudeBounds.get_bounds(jt, OptimizationMode.BLF)
        print(f"{jt.name:<12} [0, {math.degrees(fo_bounds[1]):>5.1f}°]"
              f"           [0, {math.degrees(blf_bounds[1]):>5.1f}°]")

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"""
    The BLF approach reduces:
    - Parameter count: {fo_params['total']} -> {blf_params['total']} ({(1-blf_params['total']/fo_params['total'])*100:.1f}% reduction)
    - Hip amplitude: [0, 180°] -> [0, 90°]
    - Ankle amplitude: [0, 180°] -> [0, 30°]

    Coupling topology:
    - FO: All joints coupled to physical neighbors
    - BLF: Hips fully coupled, ankles only to their hip
    """)


def create_long_limb_robot() -> BodyV1:
    """Create a robot with longer limbs (4 hinges each) to show LOCKED joints."""
    body = BodyV1()

    # Front leg: 4 hinges
    h1 = ActiveHingeV1(0.0)
    body.core_v1.front = h1
    h1.attachment = BrickV1(0.0)
    h2 = ActiveHingeV1(0.0)
    h1.attachment.front = h2
    h2.attachment = BrickV1(0.0)
    h3 = ActiveHingeV1(0.0)
    h2.attachment.front = h3
    h3.attachment = BrickV1(0.0)
    h4 = ActiveHingeV1(0.0)
    h3.attachment.front = h4

    # Back leg: 4 hinges
    h1 = ActiveHingeV1(0.0)
    body.core_v1.back = h1
    h1.attachment = BrickV1(0.0)
    h2 = ActiveHingeV1(0.0)
    h1.attachment.front = h2
    h2.attachment = BrickV1(0.0)
    h3 = ActiveHingeV1(0.0)
    h2.attachment.front = h3
    h3.attachment = BrickV1(0.0)
    h4 = ActiveHingeV1(0.0)
    h3.attachment.front = h4

    return body


def visualize_long_limb():
    """Show parameter reduction with longer limbs (LOCKED joints)."""
    print("\n" + "=" * 70)
    print("LONG LIMB ROBOT - 2 legs with 4 hinges each")
    print("=" * 70)

    robot = create_long_limb_robot()
    result = analyze_robot(robot)

    print(f"\nTotal hinges: {len(result.all_hinges)}")
    print(f"Limbs: {len(result.limbs)}")

    print("\nJoint classification per limb (4 hinges):")
    print("  H1 = HIP    (actuated)")
    print("  H2 = KNEE   (actuated)")
    print("  H3 = LOCKED (NOT actuated)")
    print("  H4 = ANKLE  (actuated)")

    # Generate networks
    blf_network = generate_cpg_network(result, OptimizationMode.BLF)
    fo_network = generate_cpg_network(result, OptimizationMode.FULLY_OPEN)

    blf_params = blf_network.get_parameter_count()
    fo_params = fo_network.get_parameter_count()

    print(f"\n{'Mode':<10} {'Oscillators':<15} {'Couplings':<15} {'TOTAL params':<15}")
    print("-" * 55)
    print(f"{'FO':<10} {fo_params['amplitudes']:<15} {fo_params['phases']:<15} {fo_params['total']:<15}")
    print(f"{'BLF':<10} {blf_params['amplitudes']:<15} {blf_params['phases']:<15} {blf_params['total']:<15}")
    print(f"\nReduction: {(1 - blf_params['total']/fo_params['total'])*100:.1f}%")

    # Show which joints are actuated vs locked
    print("\n--- BLF Oscillators (LOCKED joints excluded) ---")
    for osc in blf_network.oscillators:
        print(f"  [{osc.index}] {osc.joint_type.name}")


if __name__ == "__main__":
    visualize_spider()
    visualize_long_limb()
