import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { PLYLoader } from 'three/addons/PLYLoader.js';
import { PLYExporter } from 'three/addons/PLYExporter.js';
import { TransformControls } from 'three/addons/TransformControls.js';

const $ = id => document.getElementById(id);

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(devicePixelRatio);
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 2.5));

const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 10000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const root = new THREE.Group();  // rotated for Z-up files
scene.add(root);

// Gizmo for placing a cloud by hand before "Refine". World space, so rotation is about the view's up.
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.space = 'world';
gizmo.addEventListener('dragging-changed', e => { controls.enabled = !e.value; });
scene.add(gizmo);
let pointSize = 2;

const resize = () => {
    renderer.setSize(innerWidth, innerHeight);
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
};
addEventListener('resize', resize);
resize();

renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

// ── PLY → Object3D ─────────────────────────────────────

const loader = new PLYLoader();
// 3D Gaussian Splatting PLYs: pull SH DC color + opacity so they render as colored points.
loader.setCustomPropertyNameMapping({ dc: ['f_dc_0', 'f_dc_1', 'f_dc_2'], opacity: ['opacity'] });

const sigmoid = x => 1 / (1 + Math.exp(-x));

function gaussianToPoints(geo) {
    const pos = geo.attributes.position.array, dc = geo.attributes.dc.array, op = geo.attributes.opacity.array;
    const keepPos = [], keepCol = [];
    for (let i = 0; i < op.length; i++) {
        if (sigmoid(op[i]) < 0.2) continue;  // drop near-transparent "fog"
        keepPos.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
        for (let c = 0; c < 3; c++) keepCol.push(Math.min(1, Math.max(0, 0.5 + 0.28209479 * dc[i * 3 + c])));
    }
    const out = new THREE.BufferGeometry();
    out.setAttribute('position', new THREE.Float32BufferAttribute(keepPos, 3));
    out.setAttribute('color', new THREE.Float32BufferAttribute(keepCol, 3));
    return out;
}

function heightColors(geo) {
    geo.computeBoundingBox();
    const { min, max } = geo.boundingBox;
    const pos = geo.attributes.position.array, col = new Float32Array(pos.length), c = new THREE.Color();
    for (let i = 0; i < pos.length; i += 3) {
        c.setHSL(0.66 * (1 - (pos[i + 1] - min.y) / (max.y - min.y || 1)), 0.8, 0.5);
        col.set([c.r, c.g, c.b], i);
    }
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
}

function toObject(geo) {
    // PLYLoader fills mapped properties with NaN when the file lacks them.
    if (geo.attributes.opacity && !isNaN(geo.attributes.opacity.array[0])) geo = gaussianToPoints(geo);
    geo.deleteAttribute('dc');
    geo.deleteAttribute('opacity');
    const hasColor = !!geo.attributes.color;
    if (geo.index) {
        if (!geo.attributes.normal) geo.computeVertexNormals();
        return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
            vertexColors: hasColor, color: hasColor ? 0xffffff : 0xbbbbbb, side: THREE.DoubleSide,
        }));
    }
    if (!hasColor) heightColors(geo);
    return new THREE.Points(geo, new THREE.PointsMaterial({ size: pointSize, sizeAttenuation: false, vertexColors: true }));
}

// ── Loading ────────────────────────────────────────────

async function loadFiles(files) {
    files = [...files].filter(f => /\.ply$/i.test(f.name));
    if (!files.length) return;
    $('info').textContent = 'Loading…';
    gizmo.detach();
    root.clear();
    let points = 0;
    for (const f of files) {
        try {
            const obj = toObject(loader.parse(await f.arrayBuffer()));
            obj.name = f.name;
            root.add(obj);
            points += obj.geometry.attributes.position.count;
        } catch (e) {
            alert(`${f.name}: ${e.message}`);
        }
    }
    // Center everything on the combined bounds (keeps float32 precision for big coordinates),
    // then put each cloud's origin at its own center so the gizmo rotates it in place.
    const center = new THREE.Box3().setFromObject(root).getCenter(new THREE.Vector3());
    for (const o of root.children) {
        o.geometry.translate(-center.x, -center.y, -center.z);
        o.geometry.computeBoundingBox();
        const own = o.geometry.boundingBox.getCenter(new THREE.Vector3());
        o.geometry.translate(-own.x, -own.y, -own.z);
        o.position.copy(own);
    }
    $('move').innerHTML = '<option value="">Move…</option>' +
        root.children.map((o, i) => `<option value="${i}">${o.name.replace(/</g, '&lt;')}</option>`).join('');
    tint($('tint').checked);
    $('info').textContent = `${root.children.length} file(s), ${points.toLocaleString()} points`;
    fit();
}

function fit() {
    const box = new THREE.Box3().setFromObject(root);
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3()), r = box.getSize(new THREE.Vector3()).length() / 2;
    controls.target.copy(c);
    camera.position.copy(c).add(new THREE.Vector3(1, 0.8, 1).normalize().multiplyScalar(r * 1.8));
    camera.near = r / 1000; camera.far = r * 100;
    camera.updateProjectionMatrix();
}

// ── Alignment ──────────────────────────────────────────

const worker = new Worker('./align-worker.js', { type: 'module' });
const runWorker = msg => new Promise((resolve, reject) => {
    worker.onmessage = e => (e.data.error ? reject(new Error(e.data.error)) : resolve(e.data));
    worker.postMessage(msg, [msg.target.buffer, msg.source.buffer]);
});

// Cloud's points in root space (its current placement), at most `max` of them.
function cloudPoints(objs, max = 300000) {
    const total = objs.reduce((n, o) => n + o.geometry.attributes.position.count, 0);
    const step = Math.max(1, Math.ceil(total / max)), out = [], v = new THREE.Vector3();
    for (const o of objs) {
        o.updateMatrix();
        const pos = o.geometry.attributes.position;
        for (let i = 0; i < pos.count; i += step) {
            v.fromBufferAttribute(pos, i).applyMatrix4(o.matrix);
            out.push(v.x, v.y, v.z);
        }
    }
    return Float32Array.from(out);
}

// 'align': search all rotations. 'refine': only near where each cloud is now.
// Each cloud is aligned to everything before it; with one picked under "Move", just that one.
async function runAlign(mode) {
    const objs = root.children;
    if (objs.length < 2) return void ($('info').textContent = 'Load 2+ files to align');
    const picked = objs[$('move').value];
    const jobs = picked && mode === 'refine'
        ? [[picked, objs.filter(o => o !== picked)]]
        : objs.slice(1).map((o, i) => [o, objs.slice(0, i + 1)]);
    const report = [];
    document.body.classList.add('busy');
    try {
        for (const [i, [obj, targets]] of jobs.entries()) {
            $('info').textContent = `Aligning ${obj.name} (${i + 1}/${jobs.length})…`;
            const r = await runWorker({
                mode, up: root.rotation.x ? 2 : 1, target: cloudPoints(targets), source: cloudPoints([obj]),
            });
            obj.updateMatrix();
            obj.matrix.premultiply(new THREE.Matrix4().set(...r.matrix));
            obj.matrix.decompose(obj.position, obj.quaternion, obj.scale);
            report.push(`${obj.name}: ${Math.round(r.fitness * 100)}% overlap${r.fitness < 0.15 ? ' ⚠' : ''}`);
        }
        $('info').textContent = report.join(' · ');
        if (report.some(s => s.endsWith('⚠'))) {
            $('info').textContent += ' — ⚠ may be wrong: place it roughly with Move, then Refine';
        }
    } catch (e) {
        $('info').textContent = `Align failed: ${e.message}`;
    } finally {
        document.body.classList.remove('busy');
    }
}

// Solid color per file, to judge how well clouds line up.
function tint(on) {
    root.children.forEach((o, i) => {
        const m = o.material;
        m.userData.orig ??= { vc: m.vertexColors, color: m.color.getHex() };
        m.vertexColors = on ? false : m.userData.orig.vc;
        if (on) m.color.setHSL((i * 0.618) % 1, 0.8, 0.55); else m.color.setHex(m.userData.orig.color);
        m.needsUpdate = true;
    });
}

function saveMerged() {
    if (!root.children.length) return;
    // Export in the files' own frame, not the Z-up display rotation.
    const rx = root.rotation.x;
    root.rotation.x = 0;
    root.updateMatrixWorld(true);
    new PLYExporter().parse(root, data => {
        const a = document.createElement('a');
        a.href = URL.createObjectURL(new Blob([data]));
        a.download = 'merged.ply';
        a.click();
        URL.revokeObjectURL(a.href);
    }, { binary: true });
    root.rotation.x = rx;
    root.updateMatrixWorld(true);
}

// ── UI ─────────────────────────────────────────────────

$('file').onchange = e => loadFiles(e.target.files);
$('fit').onclick = fit;
$('zup').onclick = () => { root.rotation.x = root.rotation.x ? 0 : -Math.PI / 2; fit(); };
$('align').onclick = () => runAlign('align');
$('refine').onclick = () => runAlign('refine');
$('tint').onchange = e => tint(e.target.checked);
$('save').onclick = saveMerged;
$('move').onchange = e => {
    const o = root.children[e.target.value];
    o ? gizmo.attach(o) : gizmo.detach();
};
$('rotate').onclick = e => {
    const rotate = gizmo.mode === 'translate';
    gizmo.setMode(rotate ? 'rotate' : 'translate');
    gizmo.showX = gizmo.showZ = !rotate;  // rotate only about the vertical axis
    e.target.textContent = rotate ? 'Rotate' : 'Translate';
};
$('size').oninput = e => {
    pointSize = +e.target.value;
    root.traverse(o => { if (o.isPoints) o.material.size = pointSize; });
};

let drag = 0;
addEventListener('dragenter', e => { e.preventDefault(); drag++; document.body.classList.add('dragging'); });
addEventListener('dragleave', () => { if (--drag <= 0) { drag = 0; document.body.classList.remove('dragging'); } });
addEventListener('dragover', e => e.preventDefault());
addEventListener('drop', e => {
    e.preventDefault();
    drag = 0;
    document.body.classList.remove('dragging');
    $('hint').hidden = true;
    loadFiles(e.dataTransfer.files);
});
$('file').addEventListener('change', () => { $('hint').hidden = true; });
