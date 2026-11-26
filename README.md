# WebPointCloud

Client-side `.ply` viewer. Drag and drop (or open) one or more `.ply` files; nothing is uploaded.

- Point clouds (RGB if present, else colored by height)
- Meshes (faces → shaded mesh)
- 3D Gaussian Splatting `.ply` (shown as colored points)
- ASCII and binary PLY

## Merging scans from different spots

1. Drop all the `.ply` files at once.
2. **Auto-align**: each file is aligned to the ones before it (the first stays put). Takes a few seconds per file.
3. Tick **Color by file** to check the fit.
4. If one is wrong (low overlap ⚠), pick it under **Move…**, drag it roughly into place (toggle **Translate/Rotate**), then **Refine**.
5. **Save merged** downloads everything as one `merged.ply`.

Alignment assumes the scans share an up axis (true for phone/LiDAR scans) and solves for rotation about it plus translation. Scans that share only a bare wall or two may need step 4.

Controls: orbit with left drag, pan with right drag, zoom with wheel. **Z-up** flips files that are Z-up, **Fit** re-frames the view.

## Run locally

```bash
python3 -m http.server
```

Then open http://localhost:8000. (Any static server works; ES modules don't load from `file://`.)

## GitHub Pages

Settings → Pages → Deploy from a branch → `main` / `/ (root)`.
