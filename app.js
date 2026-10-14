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

// 嚴格防呆載入：若為空字串或不存在，一律使用 DEFAULT_CONTROLLER_CODE
const codeTextarea = document.getElementById('controller-code');
let savedCode = "";
try {
    savedCode = localStorage.getItem('drone_student_code') || "";
} catch (e) {
    savedCode = "";
}

if (!savedCode || savedCode.trim().length === 0) {
    codeTextarea.value = DEFAULT_CONTROLLER_CODE;
} else {
    codeTextarea.value = savedCode;
}

let saveTimeout = null;
codeTextarea.addEventListener('input', () => {
    try {
        localStorage.setItem('drone_student_code', codeTextarea.value);
    } catch (e) {}
    const hint = document.getElementById('save-hint');
    if (hint) {
        hint.style.display = 'inline';
        clearTimeout(saveTimeout);
        saveTimeout = setTimeout(() => hint.style.display = 'none', 1500);
    }
});

// ------------------ 3D 視覺化場景初始化 ------------------
const viewer = document.getElementById('viewer');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c0d10);

const camera = new THREE.PerspectiveCamera(45, (viewer.clientWidth || 800) / (viewer.clientHeight || 600), 1, 1000);
camera.position.set(50, 120, -50);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(viewer.clientWidth || 800, viewer.clientHeight || 600);
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

// 無人機 (實體球體，半徑 0.25m，亮青色 0x00ffff)
const DRONE_RADIUS = 0.25;
const droneMesh = new THREE.Mesh(
    new THREE.SphereGeometry(DRONE_RADIUS, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x00ffff })
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
    if (!viewer.clientWidth || !viewer.clientHeight) return;
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
        if (camOrbitBtn) camOrbitBtn.classList.add('active');
        if (camFollowBtn) camFollowBtn.classList.remove('active');
        controls.enabled = true;
        camera.position.set(50, 120, -50);
        controls.target.set(50, 0, 50);
        controls.update();
    } else {
        if (camFollowBtn) camFollowBtn.classList.add('active');
        if (camOrbitBtn) camOrbitBtn.classList.remove('active');
        controls.enabled = false;
        
        const dp = droneMesh.position;
        let vx = activeVelocity[0], vz = activeVelocity[2];
        let vLen = Math.hypot(vx, vz) || 1.0;
        camera.position.set(dp.x - (vx / vLen) * 5.0, dp.y + 2.5, dp.z - (vz / vLen) * 5.0);
        camera.lookAt(dp.x, dp.y + 0.5, dp.z);
    }
}

if (camOrbitBtn) camOrbitBtn.addEventListener('click', () => setCameraMode('orbit'));
if (camFollowBtn) camFollowBtn.addEventListener('click', () => setCameraMode('follow'));

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

        const targetCamX = dp.x - nx * 5.0;
        const targetCamY = dp.y + 2.5;
        const targetCamZ = dp.z - nz * 5.0;

        camera.position.x += (targetCamX - camera.position.x) * 0.15;
        camera.position.y += (targetCamY - camera.position.y) * 0.15;
        camera.position.z += (targetCamZ - camera.position.z) * 0.15;

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
        return { hit: true, reason: "超出空域限高 25m" };
    }
    if (y - r < 0) {
        return { hit: true, reason: "機體墜落地面" };
    }

    for (let b of buildings) {
        const cx = Math.max(b.xmin, Math.min(x, b.xmax));
        const cy = Math.max(0, Math.min(y, b.height));
        const cz = Math.max(b.zmin, Math.min(z, b.zmax));

        const distSq = (x - cx)**2 + (y - cy)**2 + (z - cz)**2;
        if (distSq <= (r * 0.96)**2) {
            if (b.mesh) {
                b.mesh.material.color.setHex(0xff0044);
            }
            return { 
                hit: true, 
                reason: "撞擊建築物外牆",
                building: b,
                contactPoint: [cx, cy, cz]
            };
        }
    }
    return { hit: false };
}

// 連續路徑細分檢測 (CCD)
function checkContinuousPath(p1, p2, step = 0.03) {
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
    const dist = Math.hypot(dx, dy, dz);
    if (dist === 0) return checkSphereCollision(p1);

    const steps = Math.ceil(dist / step);
    for (let i = 1; i <= steps; i++) {
        const t = i / steps;
        const checkPos = [p1[0] + dx * t, p1[1] + dy * t, p1[2] + dz * t];
        const res = checkSphereCollision(checkPos);
        if (res.hit) {
            return { 
                hit: true, 
                pos: checkPos, 
                reason: res.reason,
                building: res.building,
                contactPoint: res.contactPoint
            };
        }
    }
    return { hit: false };
}

// 光達淨距探測
function raycastSafe(origin, direction, maxDist = 15.0) {
    const dLen = Math.hypot(direction[0], direction[1], direction[2]);
    if (dLen === 0) return maxDist;
    const nx = direction[0] / dLen, ny = direction[1] / dLen, nz = direction[2] / dLen;

    let d = 0.1;
    const step = 0.2;
    while (d < maxDist) {
        d += step;
        const p = [origin[0] + nx * d, origin[1] + ny * d, origin[2] + nz * d];
        if (checkSphereCollision(p).hit) {
            return Math.max(0, d - DRONE_RADIUS);
        }
    }
    return maxDist;
}

// ------------------ 黑盒子深度診斷 ------------------
function diagnoseCrashReason(currentPos, vel, hitResult) {
    if (!hitResult.building) {
        return hitResult.reason;
    }

    const [vx, vy, vz] = vel;
    const speed = Math.hypot(vx, vy, vz);
    const [cx, cy, cz] = currentPos;
    const [cpx, cpy, cpz] = hitResult.contactPoint;

    const toWallX = cpx - cx;
    const toWallZ = cpz - cz;
    const wallDist = Math.hypot(toWallX, toWallZ);

    const dot = (vx * toWallX + vz * toWallZ) / ((speed * wallDist) || 1);

    if (speed >= 8.5 && dot > 0.7) {
        return `高速直衝煞停不及 (速度 ${speed.toFixed(1)}m/s，避障推力爆發距離不足)`;
    }
    if (dot < 0.4 && speed > 2.0) {
        return `轉向/側切盲區擦撞 (橫移速度過大，側翼無足夠雷達射線探測)`;
    }
    return `雷達盲區切入 (障礙物邊角落入射線間隙，未及時產生斥力)`;
}

// ------------------ 黑盒子日誌系統 (FDR) ------------------
let flightLogs = [];
let flightEvents = [];
let flightSummary = null;

function addFlightEvent(timeSec, text) {
    flightEvents.push(`[${timeSec.toFixed(2)}s] ${text}`);
}

function updateScoreboard(pos, status) {
    const [x, y, z] = pos;
    const remain3D = Math.hypot(GOAL_POS[0] - x, GOAL_POS[1] - y, GOAL_POS[2] - z);
    const remainHoriz = Math.hypot(GOAL_POS[0] - x, GOAL_POS[2] - z);
    const progress = Math.max(0, Math.min(100, ((BASE_TOTAL_DIST - remain3D) / BASE_TOTAL_DIST) * 100));

    const hudDist = document.getElementById('hud-dist');
    if (hudDist) hudDist.innerText = remain3D.toFixed(1);

    const scoreRemain = document.getElementById('score-remain-dist');
    if (scoreRemain) scoreRemain.innerText = `${remain3D.toFixed(2)} m`;

    const scoreHoriz = document.getElementById('score-horiz-dist');
    if (scoreHoriz) scoreHoriz.innerText = `${remainHoriz.toFixed(2)} m`;

    const scoreProg = document.getElementById('score-progress');
    if (scoreProg) scoreProg.innerText = `${progress.toFixed(1)} %`;

    const scoreStat = document.getElementById('score-status');
    if (scoreStat) scoreStat.innerText = status;

    return remain3D;
}

function resetScene() {
    if (simInterval) clearInterval(simInterval);
    clearLaser();
    clearCrashVector();
    const seed = parseInt(document.getElementById('map-seed').value) || 5655;
    buildCity(seed);
    clearTrail();
    droneMesh.position.set(...START_POS);
    activeVelocity = [1.0, 0.0, 1.0];

    const statusEl = document.getElementById('flight-status');
    if (statusEl) {
        statusEl.innerText = "準備就緒";
        statusEl.style.color = "#bbb";
    }

    const timeEl = document.getElementById('flight-time');
    if (timeEl) timeEl.innerText = "0.00";

    const altEl = document.getElementById('flight-alt');
    if (altEl) altEl.innerText = START_POS[1].toFixed(2);

    const logBtn = document.getElementById('download-log-btn');
    if (logBtn) logBtn.disabled = true;

    updateScoreboard(START_POS, "待命出發");

    if (cameraMode === 'follow') {
        const dp = droneMesh.position;
        camera.position.set(dp.x - 4.0, dp.y + 2.5, dp.z - 4.0);
        camera.lookAt(dp.x, dp.y + 0.5, dp.z);
    }
}

// ------------------ 事件綁定 ------------------
document.getElementById('random-seed-btn').addEventListener('click', () => {
    document.getElementById('map-seed').value = Math.floor(1000 + Math.random() * 9000);
    resetScene();
});
document.getElementById('apply-seed-btn').addEventListener('click', resetScene);

const modal = document.getElementById('prompt-modal');
document.getElementById('open-prompt-modal-btn').addEventListener('click', () => modal.style.display = 'flex');
document.getElementById('close-modal-btn').addEventListener('click', () => modal.style.display = 'none');
document.getElementById('copy-prompt-btn').addEventListener('click', () => {
    navigator.clipboard.writeText(document.getElementById('prompt-text').innerText).then(() => {
        const btn = document.getElementById('copy-prompt-btn');
        btn.innerText = "✅ 提示詞已複製！";
        setTimeout(() => btn.innerText = "📋 複製提示詞", 2000);
    });
});

// 下載黑盒子日誌
document.getElementById('download-log-btn').addEventListener('click', () => {
    if (!flightSummary) return;

    let content = `=======================================================================\n`;
    content += `         無人機黑盒子飛行紀錄與失事分析報告 (Flight Data Recorder)         \n`;
    content += `=======================================================================\n`;
    content += `產出時間             : ${new Date().toLocaleString()}\n`;
    content += `地圖種子碼 (Seed)    : ${flightSummary.seed}\n`;
    content += `任務總飛行時間       : ${flightSummary.duration} 秒\n`;
    content += `最終任務狀態         : ${flightSummary.status}\n`;
    content += `最後座標位置 [X,Y,Z] : [${flightSummary.finalPos.join(', ')}]\n`;
    content += `直達終點 3D 剩餘距離 : ${flightSummary.remainDist} 公尺\n`;
    content += `航程探索達成率       : ${flightSummary.progress} %\n\n`;

    content += `-----------------------------------------------------------------------\n`;
    content += `【一、關鍵事件時序紀錄 (Flight Event Log)】\n`;
    content += `-----------------------------------------------------------------------\n`;
    flightEvents.forEach(evt => {
        content += `${evt}\n`;
    });

    content += `\n-----------------------------------------------------------------------\n`;
    content += `【二、遙測數據時序抽樣 (每 0.2 秒)】\n`;
    content += `時間(s) | 當前座標 [X, Y, Z]       | 速度向量 [Vx, Vy, Vz]    | 速率(m/s)\n`;
    content += `-----------------------------------------------------------------------\n`;
    flightLogs.forEach(row => {
        const t = row.time.padEnd(7);
        const p = `[${row.pos.join(', ')}]`.padEnd(24);
        const v = `[${row.vel.join(', ')}]`.padEnd(24);
        content += `${t} | ${p} | ${v} | ${row.speed}\n`;
    });

    content += `\n=======================================================================\n`;
    content += `【三、AI 策略調優諮詢提示詞 (可直接貼給 ChatGPT / Claude)】\n`;
    content += `=======================================================================\n`;
    content += `學生提問模板：\n`;
    content += `「這是我在無人機 3D 避障模擬器中的黑盒子飛行報告。我的無人機在地圖種子 ${flightSummary.seed} 中，\n`;
    content += `飛行到 ${flightSummary.duration} 秒時於座標 [${flightSummary.finalPos.join(', ')}] 發生：\n`;
    content += `【${flightSummary.status}】，最後距離終點還差 ${flightSummary.remainDist} 公尺。\n`;
    content += `請根據這份黑盒子事件時序與速度數據，幫我深入診斷我的演算法邏輯盲點，並提供升級修改建議。」\n`;
    content += `=======================================================================\n`;

    const blob = new Blob([content], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `drone_fdr_seed${flightSummary.seed}_${Date.now()}.txt`;
    a.click();
    URL.revokeObjectURL(url);
});

// ------------------ 模擬執行迴圈 ------------------
let simInterval = null;

document.getElementById('run-btn').addEventListener('click', () => {
    resetScene();

    let controllerInstance = null;
    try {
        const code = document.getElementById('controller-code').value;
        try {
            localStorage.setItem('drone_student_code', code);
        } catch (e) {}
        const factory = new Function(`${code}; return new StudentController([${GOAL_POS.join(',')}]);`);
        controllerInstance = factory();
    } catch (err) {
        alert("控制器代碼語法錯誤: " + err.message);
        return;
    }

    flightLogs = [];
    flightEvents = [];
    flightSummary = null;
    addFlightEvent(0.0, `無人機從起點停機坪 (0.00, 6.25, 0.00) 啟動起飛。`);

    let currentPos = [...START_POS];
    droneMesh.position.set(...currentPos);
    appendTrail(...currentPos);

    const statusEl = document.getElementById('flight-status');
    const timeEl = document.getElementById('flight-time');
    const altEl = document.getElementById('flight-alt');

    statusEl.innerText = "飛行中...";
    statusEl.style.color = "#00d2ff";

    let timer = 0.0;
    const dt = 0.033;
    let sampleCounter = 0;

    simInterval = setInterval(() => {
        timer += dt;
        sampleCounter++;
        timeEl.innerText = timer.toFixed(2);
        altEl.innerText = currentPos[1].toFixed(2);
        updateScoreboard(currentPos, "飛行中");

        let vel = [0, 0, 0];
        try {
            vel = controllerInstance.get_velocity(
                currentPos,
                (orig, dir, maxD) => raycastSafe(orig, dir, maxD),
                dt
            );
        } catch (err) {
            clearInterval(simInterval);
            statusEl.innerText = "執行期錯誤";
            statusEl.style.color = "#ff4444";
            addFlightEvent(timer, `控制器腳本觸發未捕捉之例外錯誤: ${err.message}`);
            finalizeFlight(currentPos, timer, `執行期例外錯誤: ${err.message}`);
            alert("控制器執行錯誤: " + err.message);
            return;
        }

        activeVelocity = vel;

        if (sampleCounter % 6 === 0) {
            flightLogs.push({
                time: timer.toFixed(2),
                pos: currentPos.map(v => v.toFixed(2)),
                vel: vel.map(v => v.toFixed(2)),
                speed: Math.hypot(vel[0], vel[1], vel[2]).toFixed(2)
            });
        }

        const nextPos = [
            currentPos[0] + vel[0] * dt,
            currentPos[1] + vel[1] * dt,
            currentPos[2] + vel[2] * dt
        ];

        const collision = checkContinuousPath(currentPos, nextPos);
        if (collision.hit) {
            clearInterval(simInterval);
            const crashPos = collision.pos || nextPos;
            droneMesh.position.set(...crashPos);
            appendTrail(...crashPos);

            const detailedReason = diagnoseCrashReason(currentPos, vel, collision);

            statusEl.innerText = `墜毀：${detailedReason}`;
            statusEl.style.color = "#ff3366";

            showCrashVector(crashPos, vel);
            showDirectLaser(crashPos, GOAL_POS);
            updateScoreboard(crashPos, `墜毀：${detailedReason}`);

            addFlightEvent(timer, `碰撞失事！原因: ${detailedReason}，接觸面座標: [${crashPos.map(v=>v.toFixed(2)).join(', ')}]`);
            finalizeFlight(crashPos, timer, `墜毀：${detailedReason}`);
            return;
        }

        currentPos = nextPos;
        droneMesh.position.set(...currentPos);
        appendTrail(...currentPos);

        const distToGoal = Math.hypot(
            currentPos[0] - GOAL_POS[0],
            currentPos[1] - GOAL_POS[1],
            currentPos[2] - GOAL_POS[2]
        );

        if (distToGoal < 1.2) {
            clearInterval(simInterval);
            statusEl.innerText = "🎉 成功抵達終點！";
            statusEl.style.color = "#00ff88";
            updateScoreboard(GOAL_POS, "成功抵達 (完賽)");
            addFlightEvent(timer, `🎉 成功降落終點金色停機坪！全航程耗時 ${timer.toFixed(2)} 秒。`);
            finalizeFlight(GOAL_POS, timer, "成功抵達 (完賽)");
            return;
        }

        // 任務超時判定 (200 秒)
        if (timer > 200.0) {
            clearInterval(simInterval);
            statusEl.innerText = "任務超時 (200s)";
            statusEl.style.color = "#ffaa00";
            showDirectLaser(currentPos, GOAL_POS);
            updateScoreboard(currentPos, "超時終止");
            addFlightEvent(timer, `達到任務安全上限 (200秒)，強制安全滯空停止。`);
            finalizeFlight(currentPos, timer, "任務超時 (200s)");
            return;
        }
    }, 33);
});

function finalizeFlight(finalPos, duration, statusText) {
    const remain3D = Math.hypot(GOAL_POS[0] - finalPos[0], GOAL_POS[1] - finalPos[1], GOAL_POS[2] - finalPos[2]);
    const progress = Math.max(0, Math.min(100, ((BASE_TOTAL_DIST - remain3D) / BASE_TOTAL_DIST) * 100));

    flightSummary = {
        seed: document.getElementById('map-seed').value,
        duration: duration.toFixed(2),
        finalPos: finalPos.map(v => v.toFixed(2)),
        status: statusText,
        remainDist: remain3D.toFixed(2),
        progress: progress.toFixed(1)
    };

    const logBtn = document.getElementById('download-log-btn');
    if (logBtn) logBtn.disabled = false;
}

// 啟動場景
resetScene();
