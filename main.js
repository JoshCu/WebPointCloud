import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { PLYLoader } from 'three/addons/PLYLoader.js';

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
    // Center everything on the combined bounds (keeps float32 precision for big coordinates).
    const box = new THREE.Box3().setFromObject(root);
    const center = box.getCenter(new THREE.Vector3());
    for (const o of root.children) o.geometry.translate(-center.x, -center.y, -center.z);
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

// ── UI ─────────────────────────────────────────────────

$('file').onchange = e => loadFiles(e.target.files);
$('fit').onclick = fit;
$('zup').onclick = () => { root.rotation.x = root.rotation.x ? 0 : -Math.PI / 2; fit(); };
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
