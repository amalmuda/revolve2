"""Render the rugged terrain as top-down heatmap and isometric 3D surface."""
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from revolve2.standards.terrains import rugged_heightmap


# Match the slurm run config exactly
size = (20.0, 20.0)
num_edges = (500, 500)
heights = rugged_heightmap(size=size, num_edges=num_edges, density=1.5)
# Scale to 5 cm max bump
amax = np.max(np.abs(heights))
heights_m = heights / amax * 0.05  # absolute meters

print(f"Heightmap stats: min={heights_m.min():.3f} m, max={heights_m.max():.3f} m,"
      f" mean={heights_m.mean():.4f} m, std={heights_m.std():.4f} m")

OUT = os.path.expanduser("~/rugged_terrain.pdf")

fig = plt.figure(figsize=(14, 6))

# Top-down heatmap
ax1 = fig.add_subplot(1, 2, 1)
extent = [-size[0]/2, size[0]/2, -size[1]/2, size[1]/2]
im = ax1.imshow(heights_m * 100, extent=extent, origin="lower",
                cmap="terrain", interpolation="bilinear")
ax1.set_xlabel("X (m)", fontsize=12)
ax1.set_ylabel("Y (m)", fontsize=12)
ax1.set_title("Top-down heatmap (20 m × 20 m, max bump 5 cm)",
              fontsize=13, fontweight="bold")
cbar = fig.colorbar(im, ax=ax1, fraction=0.046, pad=0.04)
cbar.set_label("Height (cm)", fontsize=11)
# Mark robot spawn at origin
ax1.plot(0, 0, marker="x", markersize=15, mew=3, color="red", zorder=10)
ax1.annotate("robot spawn", xy=(0, 0), xytext=(1.5, 1.0),
             fontsize=11, color="red",
             arrowprops=dict(arrowstyle="->", color="red"))

# Isometric 3D
ax2 = fig.add_subplot(1, 2, 2, projection="3d")
# Subsample for plotting speed (500x500 is way too many vertices for 3D)
step = 20
xs = np.linspace(-size[0]/2, size[0]/2, num_edges[0])[::step]
ys = np.linspace(-size[1]/2, size[1]/2, num_edges[1])[::step]
X, Y = np.meshgrid(xs, ys)
Z = heights_m[::step, ::step] * 100  # cm
surf = ax2.plot_surface(X, Y, Z, cmap="terrain",
                        rstride=1, cstride=1,
                        linewidth=0.1, antialiased=True, alpha=0.95)
ax2.set_xlabel("X (m)", fontsize=11)
ax2.set_ylabel("Y (m)", fontsize=11)
ax2.set_zlabel("Height (cm)", fontsize=11)
ax2.set_title("3D surface (Z exaggerated for visibility)",
              fontsize=13, fontweight="bold")
ax2.view_init(elev=35, azim=-60)
# Force z-axis to show the small height range clearly
ax2.set_zlim(-6, 6)

fig.suptitle(
    "Rugged terrain (Perlin noise heightmap, density=1.5, 500×500 grid)",
    fontsize=15, fontweight="bold", y=1.02,
)
fig.tight_layout()
plt.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
