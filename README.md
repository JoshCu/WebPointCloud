# WebPointCloud

Client-side `.ply` viewer. Drag and drop (or open) one or more `.ply` files; nothing is uploaded.

- Point clouds (RGB if present, else colored by height)
- Meshes (faces → shaded mesh)
- 3D Gaussian Splatting `.ply` (shown as colored points)
- ASCII and binary PLY

Controls: orbit with left drag, pan with right drag, zoom with wheel. **Z-up** flips files that are Z-up, **Fit** re-frames the view.

## Run locally

```bash
python3 -m http.server
```

Then open http://localhost:8000. (Any static server works; ES modules don't load from `file://`.)

## GitHub Pages

Settings → Pages → Deploy from a branch → `main` / `/ (root)`.
