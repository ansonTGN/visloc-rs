# Splat + mesh web viewer

`index.html` is a single-file WebGL2 viewer for the output of
`gsplat_photos` (or any trained splat with a COLMAP model):

- **Splat mode:** instanced quads, EWA-projected covariance (with the same
  0.3 px blur as the wgpu renderer), front-to-back blending, and a depth
  sort in a Web Worker.
- **Mesh mode:** the extracted mesh, either in its vertex colours or
  headlight-shaded.
- **Controls:** orbit, pan and zoom; WASD/QE to fly; the arrow keys (or the
  buttons) jump to the viewpoints of the input photos.

Put these files next to `index.html`:

```bash
cargo run --release -p visloc-gsplat-core --example gsplat_export_web -- \
  --ply <run>/scene.ply --out docs/viewer/scene.splat --max 400000 \
  --cameras <run>/sparse/0/images.txt        # also writes scene.cameras.json
cp <run>/mesh.ply docs/viewer/mesh.ply       # optional; decimate large meshes
python -m http.server -d docs/viewer 8000    # then open http://localhost:8000
```

Each binary file may instead be served as base64 text (`scene.splat.txt`,
`mesh.ply.txt`) on hosts that only serve text.
