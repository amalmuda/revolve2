"""Render Revolve2 V1 module types (core, brick, active hinge) for Figure 3.2.

Each module is described as a minimal MuJoCo XML with the same colored
geometry boxes Revolve2 uses internally. Rendered offscreen at a fixed
camera angle and lighting; combined into one PNG with labels underneath.

Camera/lighting settings (so other figures can match):
  - distance:  0.20 m (from look-at)
  - azimuth:   -45 deg
  - elevation: -25 deg
  - look-at:   module's geometric center
  - light:     diffuse white directional light from above and slightly
               in front, with ambient fill from below
  - background: light grey (RGB 245,245,245)
  - resolution: 600x600 per panel
"""
import os
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont


# ===== module dimensions (taken directly from base/_core.py, _brick_v1.py, _active_hinge_v1.py)
CORE_BBOX  = (0.089, 0.089, 0.0603)
BRICK_BBOX = (0.0629, 0.0629, 0.0603)
HINGE_FRAME_BBOX  = (0.018, 0.053, 0.0165891)
HINGE_SERVO1_BBOX = (0.0583, 0.0512, 0.020)
HINGE_SERVO2_BBOX = (0.002, 0.053, 0.053)
HINGE_FRAME_OFFSET = 0.04525
HINGE_SERVO_OFFSET = 0.0299

# Revolve2 colors
CORE_RGBA  = "1.0 0.196 0.196 1"   # red
BRICK_RGBA = "0.196 0.196 1.0 1"   # blue
HINGE_RGBA = "1.0 1.0 1.0 1"       # white

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
TMP_DIR = os.path.join(OUT_DIR, "_module_renders")
os.makedirs(TMP_DIR, exist_ok=True)
FIG_OUT = "/home/abdullah/masteroppgave/master_thesis/figures/v1_modules.png"

PANEL_W = 600
PANEL_H = 600
LABEL_H = 60


def make_xml(geoms_xml: str) -> str:
    """Wrap module-geom XML in a renderable MJCF document with checker floor."""
    return f"""
<mujoco>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.5 0.5 0.5" specular="0.05 0.05 0.05"/>
    <map zfar="30" znear="0.005"/>
    <quality shadowsize="4096"/>
    <global offwidth="{PANEL_W}" offheight="{PANEL_H}"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="flat" rgb1="0.94 0.94 0.94" rgb2="0.94 0.94 0.94" width="32" height="32"/>
    <texture name="floor_tex" type="2d" builtin="checker" mark="cross" rgb1="0.86 0.86 0.86" rgb2="0.78 0.78 0.78" markrgb="0.31 0.31 0.31" width="300" height="300"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="6 6" reflectance="0.0" shininess="0.0"/>
  </asset>
  <worldbody>
    <light pos="0.4 -0.3 0.6" dir="-0.5 0.4 -1" diffuse="0.85 0.85 0.85" specular="0.1 0.1 0.1" castshadow="true"/>
    <light pos="-0.3 0.3 0.4" dir="0.4 -0.4 -1" diffuse="0.35 0.35 0.35" specular="0 0 0" castshadow="false"/>
    <geom name="floor" type="plane" pos="0 0 0" size="0.5 0.5 0.01" material="floor_mat"/>
    {geoms_xml}
  </worldbody>
</mujoco>
"""


def core_xml() -> str:
    sx, sy, sz = CORE_BBOX
    z = sz / 2  # sit on the floor
    return make_xml(f'<geom type="box" size="{sx/2} {sy/2} {sz/2}" pos="0 0 {z}" rgba="{CORE_RGBA}"/>')


def brick_xml() -> str:
    sx, sy, sz = BRICK_BBOX
    z = sz / 2
    return make_xml(f'<geom type="box" size="{sx/2} {sy/2} {sz/2}" pos="0 0 {z}" rgba="{BRICK_RGBA}"/>')


def hinge_xml() -> str:
    # Reproduce the assembly the ActiveHingeBuilder creates with slot at origin.
    # Frame at (frame_bbox[0]/2, 0, 0). Servo body at (frame_offset/2 + servo_offset, 0, 0).
    # Servo2 at servo_body + (servo1_bbox[0]/2 + servo2_bbox[0]/2, 0, 0).
    # Each piece is lifted by its own half-height so its bottom face sits
    # on the floor (matches how the parts would rest on a table).
    fx, fy, fz = HINGE_FRAME_BBOX
    s1x, s1y, s1z = HINGE_SERVO1_BBOX
    s2x, s2y, s2z = HINGE_SERVO2_BBOX

    frame_x = fx / 2
    servo1_x = HINGE_FRAME_OFFSET / 2 + HINGE_SERVO_OFFSET
    servo2_x = servo1_x + s1x / 2 + s2x / 2

    geoms = f"""
    <geom type="box" size="{fx/2} {fy/2} {fz/2}" pos="{frame_x} 0 {fz/2}" rgba="{HINGE_RGBA}"/>
    <geom type="box" size="{s1x/2} {s1y/2} {s1z/2}" pos="{servo1_x} 0 {s1z/2}" rgba="{HINGE_RGBA}"/>
    <geom type="box" size="{s2x/2} {s2y/2} {s2z/2}" pos="{servo2_x} 0 {s2z/2}" rgba="{HINGE_RGBA}"/>
"""
    return make_xml(geoms)


def render_module(xml: str, out_png: str, look_at=(0, 0, 0), distance=0.20,
                   azimuth=-45.0, elevation=-25.0):
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = look_at
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    r = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)
    try:
        r.update_scene(data, camera=cam)
        img = r.render()
    finally:
        if hasattr(r, "close"):
            r.close()
    Image.fromarray(img).save(out_png)
    return out_png


def composite(panels: list[tuple[str, str]], out_path: str):
    """Combine labelled panels into one image."""
    images = [Image.open(p) for _, p in panels]
    canvas_w = sum(im.width for im in images)
    canvas_h = images[0].height + LABEL_H
    canvas = Image.new("RGB", (canvas_w, canvas_h), (255, 255, 255))

    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 28)
    except Exception:
        font = ImageFont.load_default()

    x = 0
    draw = ImageDraw.Draw(canvas)
    for (label, _), im in zip(panels, images):
        canvas.paste(im, (x, 0))
        bbox = draw.textbbox((0, 0), label, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        tx = x + (im.width - tw) // 2
        ty = im.height + (LABEL_H - th) // 2
        draw.text((tx, ty), label, fill=(0, 0, 0), font=font)
        x += im.width

    canvas.save(out_path)
    print(f"Saved: {out_path}  ({canvas.size})")


def main():
    panels = []
    for label, xml_fn, fname, lookat in [
        ("Core",         core_xml,  "core.png",  (0, 0, 0.025)),
        ("Brick",        brick_xml, "brick.png", (0, 0, 0.025)),
        ("Active hinge", hinge_xml, "hinge.png", (0.04, 0, 0.025)),
    ]:
        out = os.path.join(TMP_DIR, fname)
        render_module(xml_fn(), out, look_at=lookat)
        panels.append((label, out))
        print(f"Rendered {label} -> {out}")

    composite(panels, FIG_OUT)


if __name__ == "__main__":
    main()
