// ------------------ 預設控制器策略 ------------------
const DEFAULT_CONTROLLER_CODE = `// 無人機自主避障控制器 (高密度雷達與盲區抑制)
class StudentController {
    constructor(goal_pos) {
        this.goal = goal_pos;

        // ★ 核心運動與感知參數 (可自行微調)
        this.speed = 10.0;              // 最高巡航速度 (m/s，建議 6~14)
        this.cruise_altitude = 8.0;     // 基準巡航高度 (m)
        this.sensorRange = 10.0;        // 雷達探測半徑 (m)
        this.repulsePower = 12.0;       // 避障排斥力強度 (數值越大閃越開)

        // 脫困狀態機參數
        this.history = [];
        this.escapeTimer = 0.0;
        this.escapeDir = [0, 0, 0];
    }

    get_velocity(current_pos, raycast_func, dt) {
        const [cx, cy, cz] = current_pos;
        const [gx, gy, gz] = this.goal;

        // 1. 卡死判定 (過去 1.2 秒位移小於 0.3m)
        this.history.push([cx, cz]);
        if (this.history.length > 35) this.history.shift();

        let isStuck = false;
        if (this.history.length === 35 && this.escapeTimer <= 0) {
            const p0 = this.history[0];
            const disp = Math.hypot(cx - p0[0], cz - p0[1]);
            if (disp < 0.3) isStuck = true;
        }

        // 2. 觸發脫困：後退 + 側向切開
        if (isStuck) {
            this.escapeTimer = 1.6;
            const dx = gx - cx, dz = gz - cz;
            const len = Math.hypot(dx, dz) || 1.0;
            const bx = -dx / len, bz = -dz / len;
            
            const distLeft = raycast_func(current_pos, [-bz, 0, bx], 12.0);
            const distRight = raycast_func(current_pos, [bz, 0, -bx], 12.0);
            const side = distLeft > distRight ? [-bz, bx] : [bz, -bx];

            this.escapeDir = [bx * 0.5 + side[0] * 0.8, 0, bz * 0.5 + side[1] * 0.8];
        }

        // 3. 執行脫困
        if (this.escapeTimer > 0) {
            this.escapeTimer -= dt;
            this.history = [];
            const len = Math.hypot(this.escapeDir[0], this.escapeDir[2]) || 1.0;
            return [
                (this.escapeDir[0] / len) * (this.speed * 0.8),
                (this.cruise_altitude - cy) * 2.0,
                (this.escapeDir[2] / len) * (this.speed * 0.8)
            ];
        }

        // 4. 16 向高解析度雷達 (消弭盲區)
        const dx = gx - cx, dz = gz - cz;
        const distGoal = Math.hypot(dx, dz) || 1.0;
        let fx = (dx / distGoal) * 2.0;
        let fz = (dz / distGoal) * 2.0;

        const numRays = 16;
        for (let i = 0; i < numRays; i++) {
            const angle = (Math.PI * 2 / numRays) * i;
            const rx = Math.cos(angle), rz = Math.sin(angle);
            const dist = raycast_func(current_pos, [rx, 0, rz], this.sensorRange);
            if (dist < this.sensorRange) {
                const rep = (1.0 / Math.max(dist, 0.1) - 1.0 / this.sensorRange) * this.repulsePower;
                fx -= rx * rep;
                fz -= rz * rep;
            }
        }

        const hLen = Math.hypot(fx, fz) || 1.0;
        const vx = (fx / hLen) * this.speed;
        const vz = (fz / hLen) * this.speed;

        // 垂直定高與終點進場降落
        const dist3D = Math.hypot(gx - cx, gy - cy, gz - cz);
        const targetY = dist3D < 14.0 ? gy : this.cruise_altitude;
        const vy = (targetY - cy) * 2.2;

        return [vx, vy, vz];
    }
}`;

// 載入 localStorage 暫存或預設代碼
const codeTextarea = document.getElementById('controller-code');
const savedCode = localStorage.getItem('drone_student_code');
codeTextarea.value = savedCode ? savedCode : DEFAULT_CONTROLLER_CODE;

let saveTimeout = null;
codeTextarea.addEventListener('input', () => {
    localStorage.setItem('drone_student_code', codeTextarea.value);
    const hint = document.getElementById('save-hint');
    hint.style.display = 'inline';
    clearTimeout(saveTimeout);
    saveTimeout = setTimeout(() => hint.style.display = 'none', 1500);
});

// ------------------ 3D 視覺化場景初始化 ------------------
const viewer = document.getElementById('viewer');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c0d10);

const camera = new THREE.PerspectiveCamera(45, viewer.clientWidth / viewer.clientHeight, 1, 1000);
camera.position.set(50, 120, -50);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(viewer.clientWidth, viewer.clientHeight);
viewer.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(50, 0, 50);
controls.mouseButtons = {
    LEFT: THREE.MOUSE.PAN,
    RIGHT: THREE.MOUSE.ROTATE
};
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.maxPolarAngle = Math.PI / 2 - 0.05;

// 光照系統
const ambientLight = new THREE.AmbientLight(0xffffff, 0.75);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.65);
dirLight.position.set(60, 100, 40);
scene.add(dirLight);

// 地面格線與外圍電子圍欄籠
const gridHelper = new THREE.GridHelper(100, 50, 0x444455, 0x222233);
gridHelper.position.set(50, 0, 50);
scene.add(gridHelper);

const fenceGeo = new THREE.BoxGeometry(106, 26, 106);
const fenceWire = new THREE.WireframeGeometry(fenceGeo);
const fenceLine = new THREE.LineSegments(
    fenceWire,
    new THREE.LineBasicMaterial({ color: 0x004466, transparent: true, opacity: 0.35 })
);
fenceLine.position.set(50, 13, 50);
scene.add(fenceLine);

const cityGroup = new THREE.Group();
scene.add(cityGroup);

// 無人機 (實體球體，半徑 0.25m)
const DRONE_RADIUS = 0.25;
const droneMesh = new THREE.Mesh(
    new THREE.SphereGeometry(DRONE_RADIUS, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xff3355 })
);
scene.add(droneMesh);

// 航跡線段
let trailPositions = [];
let trailLine = null;

function clearTrail() {
    if (trailLine) scene.remove(trailLine);
    trailPositions = [];
    const geom = new THREE.BufferGeometry();
    const mat = new THREE.LineBasicMaterial({ 
        color: 0xffaa00, 
        linewidth: 2,
        depthTest: true
    });
    trailLine = new THREE.Line(geom, mat);
    trailLine.frustumCulled = false;
    scene.add(trailLine);
}

function appendTrail(x, y, z) {
    trailPositions.push(x, y, z);
    trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(trailPositions, 3));
    trailLine.geometry.attributes.position.needsUpdate = true;
    trailLine.geometry.computeBoundingSphere();
}

// 終點殘餘直線雷射導引線
let laserLine = null;
function showDirectLaser(fromPt, toPt) {
    if (laserLine) scene.remove(laserLine);
    const geom = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...fromPt),
        new THREE.Vector3(...toPt)
    ]);
    const mat = new THREE.LineDashedMaterial({
        color: 0xff1493,
        dashSize: 1.2,
        gapSize: 0.6,
        linewidth: 2
    });
    laserLine = new THREE.Line(geom, mat);
    laserLine.frustumCulled = false;
    laserLine.computeLineDistances();
    scene.add(laserLine);
}

function clearLaser() {
    if (laserLine) {
        scene.remove(laserLine);
        laserLine = null;
    }
}

// 撞擊點速度向量視覺化箭頭
let crashArrow = null;
function showCrashVector(pos, vel) {
    if (crashArrow) scene.remove(crashArrow);
    const dir = new THREE.Vector3(vel[0], vel[1], vel[2]);
    const len = dir.length();
    if (len > 0.01) {
        dir.normalize();
        crashArrow = new THREE.ArrowHelper(dir, new THREE.Vector3(...pos), Math.min(len * 0.35, 3.5), 0xffff00, 0.8, 0.4);
        scene.add(crashArrow);
    }
}

function clearCrashVector() {
    if (crashArrow) {
        scene.remove(crashArrow);
        crashArrow = null;
    }
}

window.addEventListener('resize', () => {
    camera.aspect = viewer.clientWidth / viewer.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(viewer.clientWidth, viewer.clientHeight);
});

// ------------------ 視角切換控制系統 ------------------
let cameraMode = 'orbit';
const camOrbitBtn = document.getElementById('cam-orbit-btn');
const camFollowBtn = document.getElementById('cam-follow-btn');
let activeVelocity = [1.0, 0.0, 1.0];

function setCameraMode(mode) {
    cameraMode = mode;
    if (mode === 'orbit') {
        camOrbitBtn.classList.add('active');
        camFollowBtn.classList.remove('active');
        controls.enabled = true;
        camera.position.set(50, 120, -50);
        controls.target.set(50, 0, 50);
        controls.update();
    } else {
        camFollowBtn.classList.add('active');
        camOrbitBtn.classList.remove('active');
        controls.enabled = false;
        
        // 切換時立即拉近相機
        const dp = droneMesh.position;
        let vx = activeVelocity[0], vz = activeVelocity[2];
        let vLen = Math.hypot(vx, vz) || 1.0;
        camera.position.set(dp.x - (vx / vLen) * 5.0, dp.y + 2.5, dp.z - (vz / vLen) * 5.0);
        camera.lookAt(dp.x, dp.y + 0.5, dp.z);
    }
}

camOrbitBtn.addEventListener('click', () => setCameraMode('orbit'));
camFollowBtn.addEventListener('click', () => setCameraMode('follow'));

function animate() {
    requestAnimationFrame(animate);

    if (cameraMode === 'orbit') {
        controls.update();
    } else if (cameraMode === 'follow') {
        const dp = droneMesh.position;
        
        let vx = activeVelocity[0];
        let vz = activeVelocity[2];
        let vLen = Math.hypot(vx, vz);
        
        if (vLen < 0.1) {
            vx = 1.0;
            vz = 1.0;
            vLen = Math.SQRT2;
        }

        const nx = vx / vLen;
        const nz = vz / vLen;

        // 目標機載視角：位於後方 5.0m、高 2.5m
        const targetCamX = dp.x - nx * 5.0;
        const targetCamY = dp.y + 2.5;
        const targetCamZ = dp.z - nz * 5.0;

        // 平滑跟隨內插
        camera.position.x += (targetCamX - camera.position.x) * 0.15;
        camera.position.y += (targetCamY - camera.position.y) * 0.15;
        camera.position.z += (targetCamZ - camera.position.z) * 0.15;

        // 注視無人機前方 2.5m
        camera.lookAt(dp.x + nx * 2.5, dp.y + 0.5, dp.z + nz * 2.5);
    }

    renderer.render(scene, camera);
}
animate();

// ------------------ 隨機地圖與灰色實體建築群 ------------------
function Mulberry32(seed) {
    return function() {
        var t = seed += 0x6D2B79F5;
        t = Math.imul(t ^ t >>> 15, t | 1);
        t ^= t + Math.imul(t ^ t >>> 7, t | 61);
        return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
}

const START_POS = [0, 6.25, 0];
const GOAL_POS = [100, 14.25, 100];
const BASE_TOTAL_DIST = Math.hypot(
    GOAL_POS[0] - START_POS[0],
    GOAL_POS[1] - START_POS[1],
    GOAL_POS[2] - START_POS[2]
);
const GEOFENCE = { xmin: -3.0, xmax: 103.0, zmin: -3.0, zmax: 103.0, ymax: 25.0 };
let buildings = [];

function buildCity(seed) {
    while (cityGroup.children.length > 0) {
        cityGroup.remove(cityGroup.children[0]);
    }
    buildings = [];

    const rng = Mulberry32(seed);

    function addSolidBuilding(bx, bz, bw, bd, bh, colorHex, isSpecial = false) {
        const geo = new THREE.BoxGeometry(bw, bh, bd);
        
        const solidMat = new THREE.MeshLambertMaterial({ 
            color: isSpecial ? colorHex : 0x2e323b 
        });
        const solidMesh = new THREE.Mesh(geo, solidMat);
        solidMesh.position.set(bx, bh / 2, bz);
        cityGroup.add(solidMesh);

        const edgesGeo = new THREE.EdgesGeometry(geo);
        const edgesMat = new THREE.LineBasicMaterial({ 
            color: isSpecial ? colorHex : 0x5a6375, 
            transparent: true, 
            opacity: 0.65 
        });
        const wireLine = new THREE.LineSegments(edgesGeo, edgesMat);
        solidMesh.add(wireLine);

        buildings.push({
            xmin: bx - bw / 2, xmax: bx + bw / 2,
            zmin: bz - bd / 2, zmax: bz + bd / 2,
            height: bh,
            mesh: solidMesh
        });
    }

    // 起點大樓 (高度 6.0) 與 綠色圓盤
    addSolidBuilding(0, 0, 2.0, 2.0, 6.0, 0x00ff88, true);
    const startPad = new THREE.Mesh(
        new THREE.CylinderGeometry(0.8, 0.8, 0.05, 16),
        new THREE.MeshBasicMaterial({ color: 0x00ff88 })
    );
    startPad.position.set(0, 6.02, 0);
    cityGroup.add(startPad);

    // 終點大樓 (高度 14.0) 與 金色圓盤
    addSolidBuilding(100, 100, 2.0, 2.0, 14.0, 0xffd700, true);
    const goalPad = new THREE.Mesh(
        new THREE.CylinderGeometry(0.8, 0.8, 0.05, 16),
        new THREE.MeshBasicMaterial({ color: 0xffd700 })
    );
    goalPad.position.set(100, 14.02, 100);
    cityGroup.add(goalPad);

    // 隨機大樓群
    const streetBuffer = 3.5;
    let placed = 0, attempts = 0;
    while (placed < 90 && attempts < 2500) {
        attempts++;
        const bw = 4.0 + rng() * 4.0;
        const bd = 4.0 + rng() * 4.0;
        const bh = 8.0 + rng() * 14.0;
        const bx = 5.0 + rng() * 90.0;
        const bz = 5.0 + rng() * 90.0;

        const xmin = bx - bw / 2, xmax = bx + bw / 2;
        const zmin = bz - bd / 2, zmax = bz + bd / 2;

        if (Math.hypot(bx, bz) < 7.0 || Math.hypot(bx - 100, bz - 100) < 7.0) continue;

        let overlap = false;
        for (let b of buildings) {
            if (!(xmax + streetBuffer < b.xmin || xmin - streetBuffer > b.xmax ||
                  zmax + streetBuffer < b.zmin || zmin - streetBuffer > b.zmax)) {
                overlap = true;
                break;
            }
        }
        if (overlap) continue;

        addSolidBuilding(bx, bz, bw, bd, bh, 0x2e323b, false);
        placed++;
    }
}

// 球體實體碰撞檢測 (Sphere-Box AABB + 邊界)
function checkSphereCollision(pos) {
    const [x, y, z] = pos;
    const r = DRONE_RADIUS;

    if (x - r < GEOFENCE.xmin || x + r > GEOFENCE.xmax ||
        z - r < GEOFENCE.zmin || z + r > GEOFENCE.zmax) {
        return { hit: true, reason: "碰觸電子圍欄邊界 (Geofence Breach)" };
    }
    if (y + r > GEOFENCE.ymax) {
        return { hit: true, reason: "超出
