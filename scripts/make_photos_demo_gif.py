"""Demo GIF: a folder of photos -> splat -> mesh (gsplat_photos output).

Each frame shows, for one registered view (in name order): the input photo,
the trained splat rendered from the recovered pose, and the extracted mesh
from the same pose.

usage:
  gsplat_eval --ply <run>/scene.ply --data <run> --eval-every 1 --save-dir <run>/renders
  python scripts/make_photos_demo_gif.py --run <gsplat_photos --out dir> \\
      --out docs/assets/photos_to_mesh.gif [--caption "..."]

Needs numpy, Pillow and open3d (mesh rendering).
"""
import argparse
import os

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--run', required=True, help='gsplat_photos --out dir (images/, sparse/0, mesh.ply, renders/)')
ap.add_argument('--out', required=True)
ap.add_argument('--panel-width', type=int, default=320)
ap.add_argument('--every', type=int, default=1, help='use every n-th view')
ap.add_argument('--fps', type=float, default=8.0)
ap.add_argument('--caption', default='')
args = ap.parse_args()
RUN = args.run


def read_model(root):
    cam = [l.split() for l in open(f'{root}/sparse/0/cameras.txt') if l.strip() and not l.startswith('#')][0]
    w, h = int(cam[2]), int(cam[3])
    fx, fy, cx, cy = map(float, cam[4:8])
    lines = [l for l in open(f'{root}/sparse/0/images.txt') if l.strip() and not l.startswith('#')]
    views = []
    for l in lines:
        f = l.split()
        if len(f) < 10:
            continue
        qw, qx, qy, qz, tx, ty, tz = map(float, f[1:8])
        r = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                      [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                      [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        ext = np.eye(4)
        ext[:3, :3] = r
        ext[:3, 3] = [tx, ty, tz]
        views.append((f[9], ext))
    views.sort()
    return (w, h, fx, fy, cx, cy), views


(w, h, fx, fy, cx, cy), views = read_model(RUN)
views = views[::args.every]

# Mesh renders from every view in one window (re-creating hidden windows
# renders blank frames in some open3d builds).
mesh = o3d.io.read_triangle_mesh(f'{RUN}/mesh.ply')
mesh.compute_vertex_normals()
mesh.paint_uniform_color([0.75, 0.75, 0.75])
vis = o3d.visualization.Visualizer()
vis.create_window(width=w, height=h, visible=False)
vis.add_geometry(mesh)
opt = vis.get_render_option()
opt.background_color = np.array([1.0, 1.0, 1.0])
opt.light_on = True
ctrl = vis.get_view_control()
intr = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, w / 2 - 0.5, h / 2 - 0.5)
mesh_frames = {}
for name, ext in views:
    p = o3d.camera.PinholeCameraParameters()
    p.intrinsic, p.extrinsic = intr, ext
    ctrl.convert_from_pinhole_camera_parameters(p, allow_arbitrary=True)
    vis.poll_events()
    vis.update_renderer()
    mesh_frames[name] = Image.fromarray((np.asarray(vis.capture_screen_float_buffer(True)) * 255).astype(np.uint8))
vis.destroy_window()

pw = args.panel_width
ph = round(pw * h / w)
try:
    font = ImageFont.truetype('arial.ttf', 15)
except OSError:
    font = ImageFont.load_default()
titles = ['input photo', '3D Gaussian splat', 'extracted mesh']
bar = 24
cap = 22 if args.caption else 0
frames = []
for name, _ in views:
    stem = os.path.splitext(name)[0]
    panels = [
        Image.open(f'{RUN}/images/{name}').convert('RGB'),
        Image.open(f'{RUN}/renders/{stem}.png').convert('RGB'),
        mesh_frames[name],
    ]
    canvas = Image.new('RGB', (3 * pw + 4, ph + bar + cap), 'white')
    d = ImageDraw.Draw(canvas)
    for k, (img, t) in enumerate(zip(panels, titles)):
        canvas.paste(img.resize((pw, ph), Image.LANCZOS), (k * (pw + 2), bar))
        d.text((k * (pw + 2) + 6, 4), t, fill=(20, 20, 20), font=font)
    if args.caption:
        d.text((6, bar + ph + 3), args.caption, fill=(60, 60, 60), font=font)
    frames.append(canvas.quantize(colors=192, method=Image.Quantize.MEDIANCUT))
frames[0].save(args.out, save_all=True, append_images=frames[1:],
               duration=round(1000 / args.fps), loop=0, optimize=True)
print(f'{len(frames)} frames -> {args.out} ({os.path.getsize(args.out) / 1e6:.1f} MB)')
