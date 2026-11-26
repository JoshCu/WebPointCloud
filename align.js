// Gravity-aware registration of two point clouds (yaw + translation, 4 DOF).
// Phone/LiDAR room scans share a vertical axis, so only rotation about "up" is unknown.
//
// 1. Coarse: rasterize wall-height points into top-down density images and find the
//    best yaw + shift by FFT cross-correlation over a sweep of angles.
// 2. Fine: 4-DOF ICP from the best few candidates; keep the one with the most overlap.
// `refine` runs the same search limited to near the current pose (for hand-placed clouds).

const VOXEL = 0.04;        // downsample size (m)
const GRID = 256;          // correlation image size (pixels)
const ANGLE_STEP = 2;      // degrees
const CANDIDATES = 8;
const ICP_POINTS = 30000;

// ── point prep ─────────────────────────────────────────

// Split xyz into horizontal (a, b) + height (h) arrays, voxel-downsampled.
function toPlane(xyz, up) {
    const [ia, ib] = up === 2 ? [0, 1] : [0, 2];
    const seen = new Set(), a = [], b = [], h = [];
    for (let i = 0; i < xyz.length; i += 3) {
        const pa = xyz[i + ia], pb = xyz[i + ib], ph = xyz[i + up];
        if (!Number.isFinite(pa + pb + ph)) continue;
        const key = `${Math.floor(pa / VOXEL)},${Math.floor(pb / VOXEL)},${Math.floor(ph / VOXEL)}`;
        if (seen.has(key)) continue;
        seen.add(key);
        a.push(pa); b.push(pb); h.push(ph);
    }
    return { a: Float64Array.from(a), b: Float64Array.from(b), h: Float64Array.from(h), n: a.length };
}

const mean = arr => arr.reduce((s, v) => s + v, 0) / arr.length;

function percentile(arr, p) {
    const s = Float64Array.from(arr).sort();
    return s[Math.min(s.length - 1, Math.floor(p * s.length))];
}

function radius(c, ca, cb) {
    let r = 0;
    for (let i = 0; i < c.n; i++) r = Math.max(r, Math.hypot(c.a[i] - ca, c.b[i] - cb));
    return r;
}

// Points between floor and ceiling. Walls and furniture carry the layout; floor and
// ceiling match almost anywhere horizontally, so they'd only blur the search.
function wallBand(c) {
    const lo = percentile(c.h, 0.02), hi = percentile(c.h, 0.98);
    const pad = Math.min(0.4, (hi - lo) * 0.15);
    const idx = [];
    for (let i = 0; i < c.n; i++) if (c.h[i] > lo + pad && c.h[i] < hi - pad) idx.push(i);
    if (idx.length < 100) return c;
    const pick = k => Float64Array.from(idx, i => c[k][i]);
    return { a: pick('a'), b: pick('b'), h: pick('h'), n: idx.length };
}

const cellKey = (x, y, z) => ((x + 10000) * 20000 + (y + 10000)) * 20000 + (z + 10000);

// ── FFT (radix-2, in place) ────────────────────────────

function fft1(re, im, off, stride, n, inv) {
    for (let i = 1, j = 0; i < n; i++) {
        let bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            const p = off + i * stride, q = off + j * stride;
            [re[p], re[q]] = [re[q], re[p]];
            [im[p], im[q]] = [im[q], im[p]];
        }
    }
    for (let len = 2; len <= n; len <<= 1) {
        const ang = (inv ? 2 : -2) * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
        for (let i = 0; i < n; i += len) {
            let cr = 1, ci = 0;
            for (let k = 0; k < len / 2; k++) {
                const p = off + (i + k) * stride, q = off + (i + k + len / 2) * stride;
                const tr = re[q] * cr - im[q] * ci, ti = re[q] * ci + im[q] * cr;
                re[q] = re[p] - tr; im[q] = im[p] - ti;
                re[p] += tr; im[p] += ti;
                [cr, ci] = [cr * wr - ci * wi, cr * wi + ci * wr];
            }
        }
    }
}

function fft2(re, im, n, inv) {
    for (let r = 0; r < n; r++) fft1(re, im, r * n, 1, n, inv);
    for (let c = 0; c < n; c++) fft1(re, im, c, n, n, inv);
}

// sqrt(point count) per cell; (a, b) already relative to the image centre.
function rasterize(as, bs, cell) {
    const img = new Float64Array(GRID * GRID);
    for (let i = 0; i < as.length; i++) {
        const x = Math.floor(as[i] / cell) + GRID / 2, y = Math.floor(bs[i] / cell) + GRID / 2;
        if (x >= 0 && y >= 0 && x < GRID && y < GRID) img[y * GRID + x]++;
    }
    for (let i = 0; i < img.length; i++) img[i] = Math.sqrt(img[i]);
    return img;
}

// ── coarse search ──────────────────────────────────────

function coarse(tgt, src, angles, maxShift) {
    const ta = mean(tgt.a), tb = mean(tgt.b), sa = mean(src.a), sb = mean(src.b);
    const cell = 2 * (radius(tgt, ta, tb) + radius(src, sa, sb)) / GRID;

    const tRe = rasterize(tgt.a.map(v => v - ta), tgt.b.map(v => v - tb), cell);
    const tIm = new Float64Array(GRID * GRID);
    fft2(tRe, tIm, GRID, false);

    const best = [];
    for (const deg of angles) {
        const th = deg * Math.PI / 180, c = Math.cos(th), s = Math.sin(th);
        const ra = new Float64Array(src.n), rb = new Float64Array(src.n);
        for (let i = 0; i < src.n; i++) {
            const x = src.a[i] - sa, y = src.b[i] - sb;
            ra[i] = c * x - s * y; rb[i] = s * x + c * y;
        }
        const re = rasterize(ra, rb, cell), im = new Float64Array(GRID * GRID);
        fft2(re, im, GRID, false);
        // T · conj(S)  →  corr[d] = Σ T[x + d] S[x]
        for (let k = 0; k < re.length; k++) {
            const r = tRe[k] * re[k] + tIm[k] * im[k], j = tIm[k] * re[k] - tRe[k] * im[k];
            re[k] = r; im[k] = j;
        }
        fft2(re, im, GRID, true);
        const wrap = v => (v >= GRID / 2 ? v - GRID : v);
        let peak = -Infinity, dx = 0, dy = 0;
        for (let k = 0; k < re.length; k++) {
            if (re[k] <= peak) continue;
            const x = wrap(k % GRID) * cell, y = wrap(Math.floor(k / GRID)) * cell;
            // pose of the source centroid after the shift, relative to where it is now
            if (maxShift < Infinity && Math.hypot(ta + x - sa, tb + y - sb) > maxShift) continue;
            peak = re[k]; dx = x; dy = y;
        }
        // pose: p' = R(th)(p - cs) + ct + d
        best.push({ th, score: peak, ta: ta + dx - (c * sa - s * sb), tb: tb + dy - (s * sa + c * sb) });
    }
    best.sort((x, y) => y.score - x.score);
    return { cands: best.slice(0, CANDIDATES), cell };
}

// ── ICP (yaw + translation) ────────────────────────────

function buildGrid(c, size) {
    const g = new Map();
    for (let i = 0; i < c.n; i++) {
        const k = cellKey(Math.floor(c.a[i] / size), Math.floor(c.b[i] / size), Math.floor(c.h[i] / size));
        (g.get(k) || g.set(k, []).get(k)).push(i);
    }
    return g;
}

function nearest(tgt, grid, size, a, b, h, maxD) {
    const x = Math.floor(a / size), y = Math.floor(b / size), z = Math.floor(h / size);
    let bestD = maxD * maxD, best = -1;
    for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (let k = -1; k <= 1; k++) {
        for (const p of grid.get(cellKey(x + i, y + j, z + k)) || []) {
            const d = (tgt.a[p] - a) ** 2 + (tgt.b[p] - b) ** 2 + (tgt.h[p] - h) ** 2;
            if (d < bestD) { bestD = d; best = p; }
        }
    }
    return [best, bestD];
}

function subsample(c, max) {
    if (c.n <= max) return c;
    const step = c.n / max, pick = k => Float64Array.from({ length: max }, (_, i) => c[k][Math.floor(i * step)]);
    return { a: pick('a'), b: pick('b'), h: pick('h'), n: max };
}

function icp(tgt, grids, src, pose, startD, endD, iters) {
    let { th, ta, tb, th_h } = pose;
    for (let it = 0; it <= iters; it++) {
        const maxD = Math.max(endD, startD * Math.pow(endD / startD, it / (iters * 0.7)));
        const { grid, size } = grids.find(g => g.size >= maxD) || grids[grids.length - 1];
        const c = Math.cos(th), s = Math.sin(th);
        let n = 0, sq = 0, qa = 0, qb = 0, pa = 0, pb = 0, dh = 0;
        const pairs = [];
        for (let i = 0; i < src.n; i++) {
            const a = c * src.a[i] - s * src.b[i] + ta, b = s * src.a[i] + c * src.b[i] + tb, h = src.h[i] + th_h;
            const [j, d2] = nearest(tgt, grid, size, a, b, h, maxD);
            if (j < 0) continue;
            pairs.push(a, b, tgt.a[j], tgt.b[j]);
            qa += a; qb += b; pa += tgt.a[j]; pb += tgt.b[j]; dh += tgt.h[j] - h; sq += d2; n++;
        }
        if (it === iters || n < 10) return { th, ta, tb, th_h, fitness: n / src.n, rmse: Math.sqrt(sq / Math.max(n, 1)) };
        qa /= n; qb /= n; pa /= n; pb /= n;
        let num = 0, den = 0;
        for (let k = 0; k < pairs.length; k += 4) {
            const x = pairs[k] - qa, y = pairs[k + 1] - qb, u = pairs[k + 2] - pa, v = pairs[k + 3] - pb;
            num += x * v - y * u; den += x * u + y * v;
        }
        const d = Math.atan2(num, den), dc = Math.cos(d), ds = Math.sin(d);
        // compose: new = R(d)(R(th)p + t) + (pc - R(d)qc)
        [ta, tb] = [dc * ta - ds * tb + pa - (dc * qa - ds * qb), ds * ta + dc * tb + pb - (ds * qa + dc * qb)];
        th += d; th_h += dh / n;
    }
}

// ── public ─────────────────────────────────────────────

function toMatrix(p, up) {
    const [ia, ib] = up === 2 ? [0, 1] : [0, 2];
    const m = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    const c = Math.cos(p.th), s = Math.sin(p.th);
    m[ia][ia] = c; m[ia][ib] = -s; m[ib][ia] = s; m[ib][ib] = c;
    m[ia][3] = p.ta; m[ib][3] = p.tb; m[up][3] = p.th_h;
    return { matrix: m.flat(), fitness: p.fitness, rmse: p.rmse };
}

function register(target, source, up, angles, maxShift) {
    const tgtAll = toPlane(target, up), srcAll = toPlane(source, up);
    const th_h = percentile(tgtAll.h, 0.02) - percentile(srcAll.h, 0.02);  // match floors
    const tgt = wallBand(tgtAll), src = wallBand(srcAll);

    const { cands, cell } = coarse(tgt, src, angles, maxShift);
    const startD = 3 * cell;
    const grids = [0.05, 0.15, 0.5, startD].sort((x, y) => x - y)
        .map(size => ({ size, grid: buildGrid(tgt, size) }));

    // Quick ICP on every candidate, then a full-resolution pass on the winner.
    // Rooms are near-rectangular, so wrong 90°/180° fits also match walls loosely;
    // a tight inlier threshold is what separates them.
    const quick = subsample(src, ICP_POINTS / 6);
    let best = null;
    for (const c of cands) {
        const r = icp(tgt, grids, quick, { ...c, th_h }, startD, 0.04, 20);
        if (!best || r.fitness > best.fitness) best = r;
    }
    return toMatrix(icp(tgt, grids, subsample(src, ICP_POINTS), best, 0.15, 0.03, 30), up);
}

const range = (from, to, step) => Array.from({ length: Math.floor((to - from) / step) + 1 }, (_, i) => from + i * step);

/**
 * Align `source` onto `target` (Float32Array xyz) from any starting pose.
 * `up` = 1 (Y-up) or 2 (Z-up). Returns a row-major 4×4 matrix, fitness (0–1), rmse (m).
 */
export const align = (target, source, up = 1) =>
    register(target, source, up, range(0, 360 - ANGLE_STEP, ANGLE_STEP), Infinity);

/** Same, but only searches near the current pose (±12°, ±1 m): for clouds placed by hand. */
export const refine = (target, source, up = 1) =>
    register(target, source, up, range(-12, 12, 1).map(d => (d + 360) % 360), 1);
