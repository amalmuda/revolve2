"""Render a candidate stepping terrain: grid of small raised boxes."""
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa


# Stepping terrain parameters
TERRAIN = (20.0, 20.0)        # 20 m x 20 m
BOX_SIZE = (0.10, 0.10)       # 10 cm x 10 cm box footprint
BOX_HEIGHT = 0.05             # 5 cm tall
SPACING = 0.30                # 30 cm grid spacing (centre-to-centre)
JITTER = 0.05                 # +-5 cm random offset for each box (avoids straight aisles)


def box_positions():
    rng = np.random.RandomState(0)
    xs = np.arange(-TERRAIN[0]/2 + 0.5, TERRAIN[0]/2 - 0.5, SPACING)
    ys = np.arange(-TERRAIN[1]/2 + 0.5, TERRAIN[1]/2 - 0.5, SPACING)
    coords = []
    for x in xs:
        for y in ys:
            dx = (rng.rand() - 0.5) * 2 * JITTER
            dy = (rng.rand() - 0.5) * 2 * JITTER
            coords.append((x + dx, y + dy))
    return coords


fig = plt.figure(figsize=(14, 6))

# Top-down
ax1 = fig.add_subplot(1, 2, 1)
for x, y in box_positions():
    rect = patches.Rectangle((x - BOX_SIZE[0]/2, y - BOX_SIZE[1]/2),
                              BOX_SIZE[0], BOX_SIZE[1],
                              facecolor="#69b3a2", edgecolor="#2e6049", linewidth=0.3)
    ax1.add_patch(rect)
ax1.set_xlim(-TERRAIN[0]/2, TERRAIN[0]/2)
ax1.set_ylim(-TERRAIN[1]/2, TERRAIN[1]/2)
ax1.set_aspect("equal")
ax1.set_xlabel("X (m)", fontsize=12)
ax1.set_ylabel("Y (m)", fontsize=12)
ax1.set_title(f"Top-down view (boxes {BOX_SIZE[0]*100:.0f} cm wide, "
              f"{BOX_HEIGHT*100:.0f} cm tall, ~{SPACING*100:.0f} cm spacing)",
              fontsize=12, fontweight="bold")
ax1.plot(0, 0, marker="x", markersize=15, mew=3, color="red", zorder=10)
ax1.annotate("robot spawn", xy=(0, 0), xytext=(1.5, 1.0),
             fontsize=11, color="red",
             arrowprops=dict(arrowstyle="->", color="red"))
ax1.set_facecolor("#f5f0e6")  # ground colour

# 3D
ax2 = fig.add_subplot(1, 2, 2, projection="3d")
positions = box_positions()
# Limit to a smaller area for clearer 3D view
view_size = 4.0
visible = [(x, y) for x, y in positions if abs(x) < view_size and abs(y) < view_size]

def make_box_verts(x, y, sx, sy, sz):
    x0, x1 = x - sx/2, x + sx/2
    y0, y1 = y - sy/2, y + sy/2
    z0, z1 = 0, sz
    return [
        [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],  # bottom
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],  # top
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
        [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
    ]

for x, y in visible:
    verts = make_box_verts(x, y, BOX_SIZE[0], BOX_SIZE[1], BOX_HEIGHT)
    poly = Poly3DCollection(verts, facecolor="#69b3a2", edgecolor="#2e6049",
                             linewidth=0.4, alpha=0.95)
    ax2.add_collection3d(poly)

ax2.set_xlim(-view_size, view_size)
ax2.set_ylim(-view_size, view_size)
ax2.set_zlim(0, 0.2)
ax2.set_xlabel("X (m)", fontsize=11)
ax2.set_ylabel("Y (m)", fontsize=11)
ax2.set_zlabel("Height (m)", fontsize=11)
ax2.set_title(f"3D zoom (-{view_size:g} to +{view_size:g} m)", fontsize=12, fontweight="bold")
ax2.view_init(elev=25, azim=-55)

fig.suptitle("Candidate stepping terrain (regular grid of low boxes with small jitter)",
              fontsize=14, fontweight="bold", y=1.0)
fig.tight_layout()
OUT = os.path.expanduser("~/stepping_terrain.pdf")
plt.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
