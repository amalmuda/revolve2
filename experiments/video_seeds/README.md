# Video seeds — comparison pairs

Four evolved controllers picked for visualisation. All flat terrain, revolve2 `BrainCpgNetworkStatic` with BLF coupling, 300 generations, pop=25, seed sims for 30s.

| file                              | robot  | fitness | run | distance | drag    |
|-----------------------------------|--------|---------|-----|----------|---------|
| spider_f1_run20_drag67.npy        | spider | f1      | 20  | 3.35 m   | 67.46 % |
| spider_f2_run6_drag20.npy         | spider | f2      | 6   | 3.28 m   | 20.30 % |
| gecko_f1_run12_drag29.npy         | gecko  | f1      | 12  | 5.09 m   | 29.24 % |
| gecko_f2_run21_drag29.npy         | gecko  | f2      | 21  | 4.72 m   | 29.23 % |

Pairs:
- **Spider pair** — same robot, very different drag (67% vs 20%) at similar distance.
- **Gecko pair** — same robot, near-identical drag (~29%) under different fitness; F1 walks slightly further.

## How to view

Interactive viewer with tracking camera:
```
python _view_evolved_tracking.py video_seeds/spider_f1_run20_drag67.npy --robot spider --sim-time 30
```

Standard revolve2 viewer (no tracking, but full GUI controls):
```
python _view_evolved.py video_seeds/spider_f1_run20_drag67.npy --robot spider --terrain flat --sim-time 30
```

Render to mp4 (requires `ffmpeg` in PATH and Pillow):
```
python _render_evolved_video.py video_seeds/spider_f1_run20_drag67.npy spider_f1_run20.mp4 \
    --robot spider --sim-time 10 --fps 30
```

## Tracking-camera parameters

Defaults that worked well in preview:
- `--distance 2.5` (m from robot)
- `--cam-height 0.3` (lookat offset above ground)
- `--elev -20` (degrees, looking slightly down)
- `--azim 135` (degrees)

Tune per-robot if needed (gecko is longer, spider is wider).
