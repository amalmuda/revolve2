"""Top-down schematic of the spider showing hip joints and their swing planes."""
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc


CORE = 0.089
BRICK = 0.063
SPACER = 0.02

directions = {
    "front": (0, 1),
    "back":  (0, -1),
    "left":  (-1, 0),
    "right": (1, 0),
}

fig, ax = plt.subplots(figsize=(8, 8))

# Core
ax.add_patch(Rectangle((-CORE/2, -CORE/2), CORE, CORE,
                       facecolor="#e66", edgecolor="black", linewidth=2, zorder=2))
ax.text(0, 0, "CORE", ha="center", va="center", fontsize=10, weight="bold", zorder=3)

for name, (dx, dy) in directions.items():
    r0 = CORE / 2 + SPACER

    # Hip hinge
    hip_c = r0 + BRICK / 2
    hip_x, hip_y = hip_c * dx, hip_c * dy
    ax.add_patch(Rectangle((hip_x - BRICK*0.4, hip_y - BRICK*0.4), BRICK*0.8, BRICK*0.8,
                           facecolor="#ffd700", edgecolor="black", linewidth=1.5, zorder=3))
    ax.text(hip_x, hip_y, "HIP", ha="center", va="center", fontsize=7, weight="bold", zorder=4)

    # Brick after hip
    b_c = hip_c + BRICK / 2 + SPACER + BRICK / 2
    bx, by = b_c * dx, b_c * dy
    ax.add_patch(Rectangle((bx - BRICK/2, by - BRICK/2), BRICK, BRICK,
                           facecolor="#66e", edgecolor="black", linewidth=1, zorder=2))

    # Knee (gray, swings vertically — not visible in top-down view)
    k_c = b_c + BRICK / 2 + SPACER + BRICK / 2
    kx, ky = k_c * dx, k_c * dy
    ax.add_patch(Rectangle((kx - BRICK*0.4, ky - BRICK*0.4), BRICK*0.8, BRICK*0.8,
                           facecolor="#ddd", edgecolor="black", linewidth=1,
                           linestyle="--", zorder=3))
    ax.text(kx, ky, "knee\n(U/D)", ha="center", va="center", fontsize=6, color="#555", zorder=4)

    # Foot
    f_c = k_c + BRICK / 2 + SPACER + BRICK / 2
    fx, fy = f_c * dx, f_c * dy
    ax.add_patch(Rectangle((fx - BRICK/2, fy - BRICK/2), BRICK, BRICK,
                           facecolor="#66e", edgecolor="black", linewidth=1, zorder=2))

    # Swing arc at the hip (visible range ±60°, in the XY plane)
    leg_angle = math.degrees(math.atan2(dy, dx))
    arc_r = 0.13
    ax.add_patch(Arc((hip_x, hip_y), arc_r*2, arc_r*2,
                     angle=leg_angle, theta1=-60, theta2=60,
                     color="darkorange", linewidth=2.5, zorder=5))
    # Arrow heads at both ends
    for sign in (-1, 1):
        end_th = math.radians(leg_angle + sign * 60)
        ex = hip_x + arc_r * math.cos(end_th)
        ey = hip_y + arc_r * math.sin(end_th)
        tang_th = math.radians(leg_angle + sign * 60 + sign * 90)
        tx = 0.012 * math.cos(tang_th)
        ty = 0.012 * math.sin(tang_th)
        ax.annotate("", xy=(ex + tx, ey + ty), xytext=(ex, ey),
                    arrowprops=dict(arrowstyle="->", color="darkorange", lw=2.5), zorder=5)

ax.text(0.17, 0.28,
        "HIP axis: VERTICAL (⊥ page)\n→ leg swings in XY plane\norange arc = ±60° range\nknee axis: horizontal\n→ foot lifts U/D (hidden top-down)",
        fontsize=9,
        bbox=dict(facecolor="white", edgecolor="gray", alpha=0.9))

ax.set_xlim(-0.35, 0.35)
ax.set_ylim(-0.35, 0.35)
ax.set_aspect("equal")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Spider — hips (yellow) swing legs horizontally; knees (gray) lift feet")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("spider_hips.png", dpi=120, bbox_inches="tight")
print("Saved: spider_hips.png")
