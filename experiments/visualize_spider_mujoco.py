"""
Visualize the Spider robot in MuJoCo with Core-Centric joint classification.

This script:
1. Loads the Spider robot from standard robots
2. Analyzes it with the Core-Centric algorithm
3. Visualizes it in MuJoCo simulator using the experiments framework
"""

import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from contact_detection import simulate_with_contact_detection
from revolve2.standards import modular_robots_v1

from core_centric import (
    analyze_robot, JointType, AmplitudeBounds, OptimizationMode,
    generate_cpg_network
)


def main():
    """Main function to visualize Spider in MuJoCo."""
    print("=" * 70)
    print("SPIDER ROBOT - MuJoCo Visualization with Core-Centric Analysis")
    print("=" * 70)

    # 1. Get the spider body from standard robots
    print("\n1. Loading Spider robot body...")
    body = modular_robots_v1.get("spider")

    # 2. Analyze with Core-Centric algorithm
    print("\n2. Running Core-Centric analysis...")
    result = analyze_robot(body)

    print(f"   Total hinges: {len(result.all_hinges)}")
    print(f"   Body modules: {len(result.body_modules)}")
    print(f"   Spine hinges: {len(result.spine_hinges)}")
    print(f"   Limbs: {len(result.limbs)}")

    print("\n   Joint Classifications:")
    for hinge in result.all_hinges:
        jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
        bounds = AmplitudeBounds.get_bounds(jtype, OptimizationMode.BLF)
        print(f"     - {jtype.name}: amplitude [0, {math.degrees(bounds[1]):.0f}]")

    # 3. Generate CPG network info
    print("\n3. CPG Network (Core-Centric/BLF mode):")
    blf_network = generate_cpg_network(result, OptimizationMode.BLF)
    params = blf_network.get_parameter_count()
    print(f"   Oscillators: {params['amplitudes']}")
    print(f"   Couplings: {params['phases']}")
    print(f"   Total parameters: {params['total']}")

    # 4. Run MuJoCo simulation with viewer
    print("\n4. Starting MuJoCo simulation...")
    print("   (Close window to exit)")
    print("   Controls:")
    print("     - Mouse drag: Rotate view")
    print("     - Scroll: Zoom")
    print("     - Space: Pause/Resume")

    # Use simulate_with_contact_detection for visualization
    # This uses random CPG parameters for initial visualization
    tracker, metrics = simulate_with_contact_detection(
        robot_name="spider",
        simulation_time=30,
        verbose=True,
        cpg_params=None,  # Random CPG parameters
        headless=False,   # Show viewer
        cast_shadows=False,
        warmup_time=0.0,
    )

    print("\n" + "=" * 70)
    print("Simulation complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
