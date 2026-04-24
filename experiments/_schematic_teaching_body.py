"""2D top-down schematic of the teaching body for Chapter 3 figures.

Walks the body tree from the core outward, placing each module in the 2D
plane based on the attachment face used. Draws with matplotlib; no MuJoCo.

Convention:
  - Core's "front" points DOWN in the figure (+y direction in data, but
    we flip the y axis for display so front ends up at the bottom like
    the MuJoCo top-down render).
  - "left" rotates CCW from the current facing direction.
  - "right" rotates CW.
  - "back" is 180 degrees from the facing direction.
  - A hinge's "attachment" face continues in the current facing direction.
"""
import os
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

from revolve2.modular_robot.body.base import ActiveHinge, Brick, Core

from teaching_body import teaching_body_v1


# Colors roughly matching Figure 3.2
CORE_COLOR  = (0.92, 0.18, 0.18)
BRICK_COLOR = (0.20, 0.30, 0.85)
HINGE_COLOR = (0.80, 0.80, 0.80)
EDGE_COLOR  = "black"

MODULE_SIZE = 1.0
STEP = 1.0          # center-to-center spacing (abuts neighbors)

# Face attribute names to check on each module type
CORE_FACES = ("front", "back", "left", "right")
BRICK_FACES = ("front", "left", "right")
HINGE_FACES = ("attachment",)


def classify(module) -> str:
    if isinstance(module, Core):
        return "core"
    if isinstance(module, Brick):
        return "brick"
    return "hinge"


def rotate_ccw(v):
    return (-v[1], v[0])


def rotate_cw(v):
    return (v[1], -v[0])


def child_direction(parent_dir, face: str):
    if face == "front":
        return parent_dir
    if face == "back":
        return (-parent_dir[0], -parent_dir[1])
    if face == "left":
        return rotate_ccw(parent_dir)
    if face == "right":
        return rotate_cw(parent_dir)
    if face == "attachment":
        return parent_dir
    raise ValueError(face)


def get_child(module, face):
    return getattr(module, face, None)


def walk(module, pos, facing, records):
    records.append((module, pos, facing))
    faces = CORE_FACES if isinstance(module, Core) else (
        BRICK_FACES if isinstance(module, Brick) else HINGE_FACES
    )
    for f in faces:
        child = get_child(module, f)
        if child is None:
            continue
        cdir = child_direction(facing, f)
        cpos = (pos[0] + cdir[0] * STEP, pos[1] + cdir[1] * STEP)
        walk(child, cpos, cdir, records)


def draw_module(ax, kind, pos, facing):
    x, y = pos
    s = MODULE_SIZE
    color = {"core": CORE_COLOR, "brick": BRICK_COLOR, "hinge": HINGE_COLOR}[kind]
    rect = Rectangle((x - s / 2, y - s / 2), s, s,
                     facecolor=color, edgecolor=EDGE_COLOR, linewidth=1.4, zorder=2)
    ax.add_patch(rect)
    if kind == "hinge":
        # axis line perpendicular to the hinge's facing direction
        perp = rotate_ccw(facing)
        half = s * 0.42
        ax.add_line(Line2D(
            [x - perp[0] * half, x + perp[0] * half],
            [y - perp[1] * half, y + perp[1] * half],
            color="#333", linewidth=2.0, solid_capstyle="round", zorder=3,
        ))


def main():
    body = teaching_body_v1()
    records = []
    # Core's "front" points +y in the figure; we'll flip the axis at render time
    walk(body.core_v1, pos=(0.0, 0.0), facing=(0.0, 1.0), records=records)

    # Set up plot
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.set_aspect("equal")

    # Compute bounds
    xs = [r[1][0] for r in records]
    ys = [r[1][1] for r in records]
    pad = 1.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)

    for module, pos, facing in records:
        draw_module(ax, classify(module), pos, facing)

    ax.set_axis_off()
    # Flip y so the core's "front" direction (downward in MuJoCo top view) points DOWN here
    ax.invert_yaxis()

    # Legend
    legend_handles = [
        Rectangle((0, 0), 1, 1, facecolor=CORE_COLOR, edgecolor=EDGE_COLOR, linewidth=1.2),
        Rectangle((0, 0), 1, 1, facecolor=BRICK_COLOR, edgecolor=EDGE_COLOR, linewidth=1.2),
        Rectangle((0, 0), 1, 1, facecolor=HINGE_COLOR, edgecolor=EDGE_COLOR, linewidth=1.2),
    ]
    ax.legend(legend_handles, ["Core", "Brick", "Active hinge"],
              loc="upper right", frameon=True, facecolor="white", edgecolor="#888",
              fontsize=10)

    fig.tight_layout()

    out_pdf = "/home/abdullah/masteroppgave/master_thesis/figures/teaching_body_schematic.pdf"
    out_png = "/home/abdullah/masteroppgave/master_thesis/figures/teaching_body_schematic.png"
    fig.savefig(out_pdf, bbox_inches="tight", transparent=True)
    fig.savefig(out_png, bbox_inches="tight", dpi=200, transparent=True)
    print(f"Saved: {out_pdf}")
    print(f"Saved: {out_png}")

    counts = {"core": 0, "brick": 0, "hinge": 0}
    for module, _, _ in records:
        counts[classify(module)] += 1
    print(f"Module counts: {counts}  total={sum(counts.values())}")


if __name__ == "__main__":
    main()
