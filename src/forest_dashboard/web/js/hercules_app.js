/**
 * HERCULES Autonomous Forest UGV Mission Control Frontend Application
 * Three.js 3D WebGL Simulation + 2D Radar Costmap + Live Telemetry & Perception AI
 */

// ==============================================================================
// State Variables
// ==============================================================================
let telemetry = {
    robot: { x: 0.0, y: 0.0, yaw_deg: 0.0, linear_v: 0.0, angular_w: 0.0, battery_pct: 94.0, motor_status: 'NORMAL' },
    slam: { status: 'ACTIVE', features: 412, keyframes: 86, drift_m: 0.12, est_x: 0.0, est_y: 0.0, est_yaw_deg: 0.0, trajectory: [] },
    perception: { fps: 58.4, latency_ms: 17.2, traversable_pct: 65.4, unknown_pct: 20.2, risk_pct: 14.4, status: 'ACTIVE', obstacles: [] },
    planner: { state: 'INITIALIZING', dist_to_goal_m: 20.3, path_length_m: 21.8, replans: 1, speed: 0.85, start: [0, 0], goal: [20, 3.5], global_path: [], local_path: [] },
    system: { system_name: 'AUTONOMOUS UGV', mission_name: 'FOREST RECONNAISSANCE', gps_mode: 'DISABLED (Vision-Only)', localization_mode: 'VISUAL SLAM: ACTIVE' },
    geodesic: { forest_sector: 'Mudumalai Deep Forest Reserve', current_lat: 11.562300, current_lng: 76.534200, altitude_m: 920.0 },
    pi_camera: { model: 'Raspberry Pi Camera Module V3 (Sony IMX708)', fps: 30.0, iso: 200, exposure_time: '1/120s' },
    events: []
};

let cameraViewMode = 'overlay'; // 'raw', 'overlay', 'mask'
let activeViewport = '3D';       // '3D', 'GMAPS', 'FPV'
let simCamMode = 'FOLLOW';      // 'FOLLOW', 'TOP', 'FREE', 'FPV'
let terrainRiskEnabled = true;
let hudOverlayVisible = true;

// Google Maps Satellite Components
let gmap = null;
let gmapUgvMarker = null;
let gmapStartMarker = null;
let gmapGoalMarker = null;
let gmapTrajectoryPolyline = null;
let gmapPathCoords = [];

// Three.js 3D Scene Components
let scene, camera, renderer;
let ugvModel, ugvWheels = [];
let plannedPathLine, localPathLine, trajectoryLine;
let terrainMesh;

// 2D Map Canvas
let mapCanvas, mapCtx;


// ==============================================================================
// Initialization
// ==============================================================================
window.addEventListener('DOMContentLoaded', () => {
    initThreeJsWorld();
    initGoogleMaps();
    init2DMap();
    setupEventListeners();
    setupKeyboardTeleop();

    // Telemetry Polling Loop (20 Hz)
    setInterval(fetchTelemetry, 50);

    // Animation Render Loop
    requestAnimationFrame(renderLoop);
});

// ==============================================================================
// Google Maps Satellite Reconnaissance Initialization
// ==============================================================================
function initGoogleMaps() {
    if (typeof google === 'undefined' || !google.maps) {
        console.warn('Google Maps API not yet loaded. Retrying in 1s...');
        setTimeout(initGoogleMaps, 1000);
        return;
    }

    const mapElement = document.getElementById('gmap');
    if (!mapElement) return;

    // Center on Mudumalai Deep Forest & Tiger Reserve (Nilgiri Biosphere)
    const baseOrigin = { lat: 11.562300, lng: 76.534200 };
    const goalLocation = { lat: 11.562472, lng: 76.534262 };

    gmap = new google.maps.Map(mapElement, {
        center: baseOrigin,
        zoom: 19,
        mapTypeId: 'hybrid',
        tilt: 45,
        heading: 30,
        mapTypeControl: true,
        streetViewControl: false,
        rotateControl: true,
        fullscreenControl: true,
        styles: [
            { elementType: 'labels.text.fill', stylers: [{ color: '#746855' }] }
        ]
    });

    // Start Point A Marker
    gmapStartMarker = new google.maps.Marker({
        position: baseOrigin,
        map: gmap,
        title: 'Start Reconnaissance Point A (0.0m, 0.0m)',
        icon: {
            path: google.maps.SymbolPath.CIRCLE,
            scale: 7,
            fillColor: '#00ff88',
            fillOpacity: 1,
            strokeColor: '#ffffff',
            strokeWeight: 2
        }
    });

    // Goal Point B Marker
    gmapGoalMarker = new google.maps.Marker({
        position: goalLocation,
        map: gmap,
        title: 'Goal Point B Rendezvous (20.0m, 3.5m)',
        icon: {
            path: google.maps.SymbolPath.CIRCLE,
            scale: 8,
            fillColor: '#00f0ff',
            fillOpacity: 1,
            strokeColor: '#ffffff',
            strokeWeight: 2
        }
    });

    // Live UGV Marker with Directional Vector
    gmapUgvMarker = new google.maps.Marker({
        position: baseOrigin,
        map: gmap,
        title: 'HERCULES UGV Live Position',
        icon: {
            path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            scale: 6,
            fillColor: '#ffb700',
            fillOpacity: 1,
            strokeColor: '#000000',
            strokeWeight: 2,
            rotation: 0
        }
    });

    // Live Visual SLAM Trajectory Path on Satellite Imagery
    gmapTrajectoryPolyline = new google.maps.Polyline({
        path: [baseOrigin],
        geodesic: true,
        strokeColor: '#00e699',
        strokeOpacity: 0.9,
        strokeWeight: 4,
        map: gmap
    });
}


// ==============================================================================
// Three.js 3D Simulation Viewport
// ==============================================================================
function initThreeJsWorld() {
    const container = document.getElementById('threejs-container');
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060a0f);
    scene.fog = new THREE.FogExp2(0x060a0f, 0.025);

    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 500);
    camera.position.set(-6, 8, 12);

    renderer = new THREE.WebGLRenderer({ antialias: true, canvas: document.getElementById('threejs-canvas') });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xddeeff, 0.6);
    scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xfff5e6, 1.2);
    sunLight.position.set(20, 40, 20);
    sunLight.castShadow = true;
    scene.add(sunLight);

    // Build Forest Environment
    buildTerrain();
    buildTreesAndRocks();
    buildStartAndGoalMarkers();

    // Build 3D UGV Robot Model
    buildUgvRobot();

    // Build Path & Trajectory Lines
    buildPathLines();

    // Window Resize Handler
    window.addEventListener('resize', () => {
        const w = container.clientWidth;
        const h = container.clientHeight;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    });
}

function buildTerrain() {
    // 60m x 40m Terrain Grid
    const geo = new THREE.PlaneGeometry(60, 40, 60, 40);
    const pos = geo.attributes.position;
    for (let i = 0; i < pos.count; i++) {
        const x = pos.getX(i);
        const y = pos.getY(i);
        // Add subtle natural terrain undulating height
        const z = Math.sin(x * 0.15) * Math.cos(y * 0.15) * 0.45;
        pos.setZ(i, z);
    }
    geo.computeVertexNormals();

    const mat = new THREE.MeshLambertMaterial({ color: 0x182a1b, wireframe: false });
    terrainMesh = new THREE.Mesh(geo, mat);
    terrainMesh.rotation.x = -Math.PI / 2;
    terrainMesh.position.set(10, 0, 0);
    scene.add(terrainMesh);

    // Trail Corridor Ribbon
    const trailCurve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(0, 0.05, 0),
        new THREE.Vector3(5, 0.05, 0.2),
        new THREE.Vector3(10, 0.05, 0.8),
        new THREE.Vector3(15, 0.05, 2.2),
        new THREE.Vector3(20, 0.05, 3.5),
        new THREE.Vector3(25, 0.05, 3.8)
    ]);
    const trailGeo = new THREE.TubeGeometry(trailCurve, 40, 1.4, 8, false);
    const trailMat = new THREE.MeshLambertMaterial({ color: 0x5a4a35 });
    const trailMesh = new THREE.Mesh(trailGeo, trailMat);
    scene.add(trailMesh);
}

function buildTreesAndRocks() {
    // Pine Trees
    const trunkMat = new THREE.MeshLambertMaterial({ color: 0x3d2b1f });
    const leavesMat = new THREE.MeshLambertMaterial({ color: 0x0f4d1e });

    const treePositions = [
        [3, -3.5], [6, 4.0], [9, -4.2], [12, 5.0], [15, -3.8], [18, 6.0], [22, -4.5],
        [2, 3.8], [7, -4.8], [11, 4.5], [14, -5.2], [17, 4.2], [21, 5.5], [24, -3.2]
    ];

    treePositions.forEach(([tx, ty]) => {
        const trunkGeo = new THREE.CylinderGeometry(0.25, 0.35, 3.5, 8);
        const trunk = new THREE.Mesh(trunkGeo, trunkMat);
        trunk.position.set(tx, 1.75, ty);

        const leavesGeo = new THREE.ConeGeometry(1.6, 5.0, 8);
        const leaves = new THREE.Mesh(leavesGeo, leavesMat);
        leaves.position.set(tx, 4.8, ty);

        scene.add(trunk);
        scene.add(leaves);
    });

    // Boulder Formation Hazard at X=10m
    const rockMat = new THREE.MeshLambertMaterial({ color: 0x4a525d });
    const rockGeo1 = new THREE.DodecahedronGeometry(0.8, 1);
    const rock1 = new THREE.Mesh(rockGeo1, rockMat);
    rock1.position.set(10.0, 0.5, 1.4);
    rock1.scale.set(1.2, 0.9, 1.1);
    scene.add(rock1);

    const rockGeo2 = new THREE.DodecahedronGeometry(0.6, 1);
    const rock2 = new THREE.Mesh(rockGeo2, rockMat);
    rock2.position.set(10.5, 0.4, 2.2);
    scene.add(rock2);
}

function buildStartAndGoalMarkers() {
    // Start Beacon (Point A)
    const startGeo = new THREE.RingGeometry(0.8, 1.1, 24);
    const startMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, side: THREE.DoubleSide });
    const startMesh = new THREE.Mesh(startGeo, startMat);
    startMesh.rotation.x = Math.PI / 2;
    startMesh.position.set(0, 0.08, 0);
    scene.add(startMesh);

    // Goal Beacon (Point B)
    const goalGeo = new THREE.CylinderGeometry(0.1, 0.8, 2.5, 16);
    const goalMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, wireframe: true });
    const goalMesh = new THREE.Mesh(goalGeo, goalMat);
    goalMesh.position.set(20.0, 1.25, 3.5);
    scene.add(goalMesh);
}

function buildUgvRobot() {
    ugvModel = new THREE.Group();

    // Chassis Box
    const bodyGeo = new THREE.BoxGeometry(0.9, 0.28, 0.55);
    const bodyMat = new THREE.MeshLambertMaterial({ color: 0x1f2937 });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.position.y = 0.25;
    ugvModel.add(body);

    // High-visibility top cover
    const topGeo = new THREE.BoxGeometry(0.7, 0.06, 0.45);
    const topMat = new THREE.MeshLambertMaterial({ color: 0x00f0ff });
    const top = new THREE.Mesh(topGeo, topMat);
    top.position.set(-0.05, 0.42, 0);
    ugvModel.add(top);

    // Forward Camera Mast & Raspberry Pi Camera Module V3 Assembly
    const mastGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.30, 8);
    const mastMat = new THREE.MeshLambertMaterial({ color: 0x475569 });
    const mast = new THREE.Mesh(mastGeo, mastMat);
    mast.position.set(0.32, 0.52, 0);
    ugvModel.add(mast);

    // Raspberry Pi Camera V3 Mount Bracket (Black Anodized)
    const piMountGeo = new THREE.BoxGeometry(0.04, 0.08, 0.08);
    const piMountMat = new THREE.MeshLambertMaterial({ color: 0x0f172a });
    const piMount = new THREE.Mesh(piMountGeo, piMountMat);
    piMount.position.set(0.34, 0.68, 0);
    ugvModel.add(piMount);

    // Raspberry Pi Camera V3 PCB (Sony IMX708 Sensor)
    const piPcbGeo = new THREE.BoxGeometry(0.015, 0.06, 0.06);
    const piPcbMat = new THREE.MeshLambertMaterial({ color: 0x14532d }); // Dark Green PCB
    const piPcb = new THREE.Mesh(piPcbGeo, piPcbMat);
    piPcb.position.set(0.36, 0.68, 0);
    ugvModel.add(piPcb);

    // Sony IMX708 Autofocus Lens Barrel
    const piLensGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.03, 16);
    const piLensMat = new THREE.MeshLambertMaterial({ color: 0xf59e0b }); // Golden-brass barrel
    const piLens = new THREE.Mesh(piLensGeo, piLensMat);
    piLens.rotation.z = Math.PI / 2;
    piLens.position.set(0.38, 0.68, 0);
    ugvModel.add(piLens);

    // Optical Glass Element
    const glassGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.005, 16);
    const glassMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
    const glass = new THREE.Mesh(glassGeo, glassMat);
    glass.rotation.z = Math.PI / 2;
    glass.position.set(0.396, 0.68, 0);
    ugvModel.add(glass);

    // Flexible White/Blue CSI Ribbon Cable
    const ribbonGeo = new THREE.BoxGeometry(0.20, 0.002, 0.035);
    const ribbonMat = new THREE.MeshLambertMaterial({ color: 0x3b82f6 }); // Blue/white flex cable
    const ribbon = new THREE.Mesh(ribbonGeo, ribbonMat);
    ribbon.position.set(0.18, 0.58, 0);
    ribbon.rotation.z = -0.4;
    ugvModel.add(ribbon);

    // Google Gemini Neural Cognitive Brain Core (Glowing Purple/Cyan Sphere & Compute Deck)
    const geminiBaseGeo = new THREE.BoxGeometry(0.32, 0.08, 0.28);
    const geminiBaseMat = new THREE.MeshLambertMaterial({ color: 0x1a1528 });
    const geminiBase = new THREE.Mesh(geminiBaseGeo, geminiBaseMat);
    geminiBase.position.set(-0.05, 0.44, 0);
    ugvModel.add(geminiBase);

    const geminiCoreGeo = new THREE.SphereGeometry(0.10, 16, 16);
    const geminiCoreMat = new THREE.MeshBasicMaterial({ color: 0xb05cff, wireframe: false });
    const geminiCore = new THREE.Mesh(geminiCoreGeo, geminiCoreMat);
    geminiCore.position.set(-0.05, 0.52, 0);
    ugvModel.add(geminiCore);

    // Gemini Neural Antenna
    const geminiAntGeo = new THREE.CylinderGeometry(0.01, 0.01, 0.16, 8);
    const geminiAntMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
    const geminiAnt = new THREE.Mesh(geminiAntGeo, geminiAntMat);
    geminiAnt.position.set(-0.16, 0.56, 0.10);
    ugvModel.add(geminiAnt);

    // 4 Wheels
    const wheelGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.12, 16);
    const wheelMat = new THREE.MeshLambertMaterial({ color: 0x111827 });
    const wheelOffsets = [
        [0.28, 0.18, 0.32],
        [0.28, 0.18, -0.32],
        [-0.28, 0.18, 0.32],
        [-0.28, 0.18, -0.32]
    ];

    wheelOffsets.forEach(([wx, wy, wz]) => {
        const wheel = new THREE.Mesh(wheelGeo, wheelMat);
        wheel.rotation.x = Math.PI / 2;
        wheel.position.set(wx, wy, wz);
        ugvWheels.push(wheel);
        ugvModel.add(wheel);
    });

    scene.add(ugvModel);
}

function buildPathLines() {
    // Global A* Path Line
    const globalGeo = new THREE.BufferGeometry();
    const globalMat = new THREE.LineBasicMaterial({ color: 0x00f0ff, linewidth: 3 });
    plannedPathLine = new THREE.Line(globalGeo, globalMat);
    scene.add(plannedPathLine);

    // Local Reactive Path Line
    const localGeo = new THREE.BufferGeometry();
    const localMat = new THREE.LineBasicMaterial({ color: 0xffb700, linewidth: 4 });
    localPathLine = new THREE.Line(localGeo, localMat);
    scene.add(localPathLine);

    // Trajectory History Line
    const trajGeo = new THREE.BufferGeometry();
    const trajMat = new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 });
    trajectoryLine = new THREE.Line(trajGeo, trajMat);
    scene.add(trajectoryLine);
}


// ==============================================================================
// 2D Tactical Radar & Traversability Risk Map
// ==============================================================================
function init2DMap() {
    mapCanvas = document.getElementById('map-canvas');
    mapCtx = mapCanvas.getContext('2d');
    resizeMapCanvas();
    window.addEventListener('resize', resizeMapCanvas);
}

function resizeMapCanvas() {
    if (!mapCanvas) return;
    mapCanvas.width = mapCanvas.parentElement.clientWidth;
    mapCanvas.height = mapCanvas.parentElement.clientHeight;
}

function draw2DMap() {
    if (!mapCtx) return;
    const w = mapCanvas.width;
    const h = mapCanvas.height;

    mapCtx.fillStyle = '#070a0e';
    mapCtx.fillRect(0, 0, w, h);

    // Coordinate Mapping: World X [0 to 24m] -> Canvas X [30 to w-30]
    // World Y [-6 to 6m] -> Canvas Y [h-30 to 30]
    const scaleX = (w - 60) / 24.0;
    const scaleY = (h - 60) / 12.0;

    const toCanvasX = (wx) => 30 + wx * scaleX;
    const toCanvasY = (wy) => (h / 2) - wy * scaleY;

    // Grid Lines
    mapCtx.strokeStyle = '#141d2b';
    mapCtx.lineWidth = 1;
    for (let x = 0; x <= 24; x += 4) {
        mapCtx.beginPath();
        mapCtx.moveTo(toCanvasX(x), 0);
        mapCtx.lineTo(toCanvasX(x), h);
        mapCtx.stroke();
    }
    for (let y = -6; y <= 6; y += 2) {
        mapCtx.beginPath();
        mapCtx.moveTo(0, toCanvasY(y));
        mapCtx.lineTo(w, toCanvasY(y));
        mapCtx.stroke();
    }

    // Terrain Traversability Risk Map Layer
    if (terrainRiskEnabled) {
        // Safe Trail Corridor (Green)
        mapCtx.fillStyle = 'rgba(0, 255, 136, 0.12)';
        mapCtx.beginPath();
        mapCtx.moveTo(toCanvasX(0), toCanvasY(-1.2));
        mapCtx.lineTo(toCanvasX(10), toCanvasY(-0.4));
        mapCtx.lineTo(toCanvasX(20), toCanvasY(2.2));
        mapCtx.lineTo(toCanvasX(20), toCanvasY(4.8));
        mapCtx.lineTo(toCanvasX(10), toCanvasY(2.2));
        mapCtx.lineTo(toCanvasX(0), toCanvasY(1.2));
        mapCtx.closePath();
        mapCtx.fill();

        // High-Risk Boulder Zone (Red)
        mapCtx.fillStyle = 'rgba(255, 42, 85, 0.28)';
        mapCtx.beginPath();
        mapCtx.arc(toCanvasX(10.0), toCanvasY(1.4), 16, 0, Math.PI * 2);
        mapCtx.fill();
        mapCtx.strokeStyle = '#ff2a55';
        mapCtx.stroke();
    }

    // Planned Global A* Path (Cyan)
    if (telemetry.planner.global_path && telemetry.planner.global_path.length > 1) {
        mapCtx.strokeStyle = '#00f0ff';
        mapCtx.lineWidth = 2;
        mapCtx.setLineDash([4, 4]);
        mapCtx.beginPath();
        telemetry.planner.global_path.forEach(([px, py], i) => {
            const cx = toCanvasX(px);
            const cy = toCanvasY(py);
            if (i === 0) mapCtx.moveTo(cx, cy);
            else mapCtx.lineTo(cx, cy);
        });
        mapCtx.stroke();
        mapCtx.setLineDash([]);
    }

    // Trajectory History (Emerald)
    if (telemetry.slam.trajectory && telemetry.slam.trajectory.length > 1) {
        mapCtx.strokeStyle = '#00ff88';
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        telemetry.slam.trajectory.forEach(([tx, ty], i) => {
            const cx = toCanvasX(tx);
            const cy = toCanvasY(ty);
            if (i === 0) mapCtx.moveTo(cx, cy);
            else mapCtx.lineTo(cx, cy);
        });
        mapCtx.stroke();
    }

    // Start Marker A
    mapCtx.fillStyle = '#00ff88';
    mapCtx.beginPath();
    mapCtx.arc(toCanvasX(0), toCanvasY(0), 6, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.font = '10px JetBrains Mono';
    mapCtx.fillText('START A', toCanvasX(0) - 18, toCanvasY(0) + 16);

    // Goal Marker B
    mapCtx.fillStyle = '#00f0ff';
    mapCtx.beginPath();
    mapCtx.arc(toCanvasX(20.0), toCanvasY(3.5), 7, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.fillText('GOAL B', toCanvasX(20.0) - 16, toCanvasY(3.5) - 12);

    // UGV Icon with Heading Vector
    const rx = toCanvasX(telemetry.robot.x);
    const ry = toCanvasY(telemetry.robot.y);
    const yawRad = (telemetry.robot.yaw_deg * Math.PI) / 180.0;

    mapCtx.save();
    mapCtx.translate(rx, ry);
    mapCtx.rotate(-yawRad); // Invert for canvas coordinate system

    // UGV Body
    mapCtx.fillStyle = '#ffb700';
    mapCtx.fillRect(-10, -6, 20, 12);
    mapCtx.strokeStyle = '#fff';
    mapCtx.lineWidth = 1.5;
    mapCtx.strokeRect(-10, -6, 20, 12);

    // Forward Arrow
    mapCtx.fillStyle = '#ff2a55';
    mapCtx.beginPath();
    mapCtx.moveTo(10, 0);
    mapCtx.lineTo(16, 0);
    mapCtx.lineTo(12, -4);
    mapCtx.lineTo(16, 0);
    mapCtx.lineTo(12, 4);
    mapCtx.stroke();

    mapCtx.restore();
}


// ==============================================================================
// Telemetry & UI Updates
// ==============================================================================
function fetchTelemetry() {
    fetch('/api/telemetry')
        .then(res => res.json())
        .then(data => {
            telemetry = data;
            updateDomElements();
            update3DScene();
            updateGoogleMaps();
        })
        .catch(() => {});
}

function updateDomElements() {
    // Top Bar Status
    if (document.getElementById('stat-state')) document.getElementById('stat-state').textContent = telemetry.planner.state;
    if (document.getElementById('stat-picam')) document.getElementById('stat-picam').textContent = telemetry.perception.fps > 0 ? 'STREAMING' : 'STANDBY';

    // Robot Telemetry
    if (document.getElementById('tel-x')) document.getElementById('tel-x').textContent = telemetry.robot.x.toFixed(2) + ' m';
    if (document.getElementById('tel-y')) document.getElementById('tel-y').textContent = telemetry.robot.y.toFixed(2) + ' m';
    if (document.getElementById('tel-yaw')) document.getElementById('tel-yaw').textContent = telemetry.robot.yaw_deg.toFixed(1) + '°';
    if (document.getElementById('tel-lin-v')) document.getElementById('tel-lin-v').textContent = telemetry.robot.linear_v.toFixed(2) + ' m/s';
    if (document.getElementById('tel-ang-w')) document.getElementById('tel-ang-w').textContent = telemetry.robot.angular_w.toFixed(2) + ' rad/s';
    if (document.getElementById('tel-battery')) document.getElementById('tel-battery').textContent = telemetry.robot.battery_pct.toFixed(0) + '%';

    // Real-World Google Maps Geodesic Coordinates
    if (telemetry.geodesic) {
        const geo = telemetry.geodesic;
        if (document.getElementById('geo-lat')) document.getElementById('geo-lat').textContent = (geo.current_lat || 11.562300).toFixed(7) + '° N';
        if (document.getElementById('geo-lng')) document.getElementById('geo-lng').textContent = (geo.current_lng || 76.534200).toFixed(7) + '° E';
        if (document.getElementById('geo-alt')) document.getElementById('geo-alt').textContent = (geo.altitude_m || 920.0).toFixed(1);
        if (document.getElementById('gmap-live-coords')) {
            document.getElementById('gmap-live-coords').textContent = `${(geo.current_lat || 11.562300).toFixed(7)}° N, ${(geo.current_lng || 76.534200).toFixed(7)}° E`;
        }
    }

    // SLAM Telemetry
    if (document.getElementById('slam-feat')) document.getElementById('slam-feat').textContent = telemetry.slam.features;
    if (document.getElementById('slam-kf')) document.getElementById('slam-kf').textContent = telemetry.slam.keyframes;
    if (document.getElementById('slam-drift')) document.getElementById('slam-drift').textContent = telemetry.slam.drift_m.toFixed(3) + ' m';

    // AI Perception
    if (document.getElementById('ai-fps')) document.getElementById('ai-fps').textContent = telemetry.perception.fps.toFixed(1);
    if (document.getElementById('ai-trav')) document.getElementById('ai-trav').textContent = telemetry.perception.traversable_pct.toFixed(1) + '%';
    if (document.getElementById('ai-risk')) document.getElementById('ai-risk').textContent = telemetry.perception.risk_pct.toFixed(1) + '%';

    // Raspberry Pi Camera Tactical HUD Overlays
    const yawDeg = Math.round((telemetry.robot.yaw_deg + 360) % 360);
    const hdgStr = String(yawDeg).padStart(3, '0');
    if (document.getElementById('hud-heading-data')) {
        document.getElementById('hud-heading-data').textContent = `HDG: ${hdgStr}° | PITCH: -7.0°`;
    }
    if (document.getElementById('fpv-compass')) {
        document.getElementById('fpv-compass').textContent = `◀ W · · ${hdgStr}° · · E ▶`;
    }

    // Google Gemini Multimodal VLA Brain Telemetry
    if (telemetry.gemini) {
        const gem = telemetry.gemini;
        if (document.getElementById('gemini-obs')) {
            document.getElementById('gemini-obs').textContent = gem.scene_description || 'Analyzing Pi-Cam scene...';
        }
        if (document.getElementById('gemini-reasoning')) {
            document.getElementById('gemini-reasoning').textContent = gem.tactical_spatial_reasoning || 'Processing...';
        }
        if (document.getElementById('fpv-gemini-text')) {
            document.getElementById('fpv-gemini-text').textContent = gem.tactical_spatial_reasoning || 'Clear trail ahead.';
        }
        if (document.getElementById('hud-gemini-badge')) {
            document.getElementById('hud-gemini-badge').textContent = `GEMINI: ${gem.action_decision || 'FOLLOW_TRAIL'}`;
        }
        if (document.getElementById('gemini-action-pill')) {
            const actionEl = document.getElementById('gemini-action-pill');
            actionEl.textContent = gem.action_decision || 'FOLLOW_TRAIL';
            if (gem.action_decision === 'BYPASS_LEFT' || gem.action_decision === 'BYPASS_RIGHT') {
                actionEl.style.color = 'var(--amber)';
            } else if (gem.action_decision === 'EMERGENCY_STOP') {
                actionEl.style.color = 'var(--crimson)';
            } else {
                actionEl.style.color = 'var(--emerald)';
            }
        }
        if (document.getElementById('gemini-latency')) {
            document.getElementById('gemini-latency').textContent = (gem.latency_ms || 18).toFixed(1) + ' ms';
        }
        if (document.getElementById('gemini-engine-tag')) {
            document.getElementById('gemini-engine-tag').textContent = gem.engine ? gem.engine.toUpperCase() : 'GEMINI 3.6 FLASH / VLA';
        }
    }

    // Obstacle Event Alert Banner
    const banner = document.getElementById('event-banner');
    if (banner) {
        if (telemetry.planner.state === 'AVOID_OBSTACLE') {
            banner.className = 'alert-banner hazard';
            banner.innerHTML = '⚠ HAZARD DETECTED on trail — Dynamically replanning bypass route!';
            banner.style.display = 'flex';
        } else if (telemetry.planner.state === 'GOAL_REACHED') {
            banner.className = 'alert-banner success';
            banner.innerHTML = '✓ MISSION ACCOMPLISHED: Goal Point B safely reached in Mudumalai Forest!';
            banner.style.display = 'flex';
        } else {
            banner.style.display = 'none';
        }
    }

    // Log Stream
    const logBox = document.getElementById('log-stream');
    if (logBox && telemetry.events) {
        logBox.innerHTML = telemetry.events.map(ev => `
            <div class="log-entry ${ev.level}">
                <span class="time">[${ev.time}]</span> ${ev.msg}
            </div>
        `).join('');
    }
}

function updateGoogleMaps() {
    if (!gmap || !gmapUgvMarker || !telemetry.geodesic) return;

    const lat = telemetry.geodesic.current_lat || 11.562300;
    const lng = telemetry.geodesic.current_lng || 76.534200;
    const currentPos = new google.maps.LatLng(lat, lng);

    // Update UGV marker position and heading rotation
    gmapUgvMarker.setPosition(currentPos);
    const icon = gmapUgvMarker.getIcon();
    if (icon) {
        icon.rotation = Math.round(telemetry.robot.yaw_deg);
        gmapUgvMarker.setIcon(icon);
    }

    // Append to live trajectory polyline on satellite map
    const path = gmapTrajectoryPolyline.getPath();
    const lastPos = path.getLength() > 0 ? path.getAt(path.getLength() - 1) : null;
    if (!lastPos || google.maps.geometry.spherical.computeDistanceBetween(lastPos, currentPos) > 0.3) {
        path.push(currentPos);
    }

    if (activeViewport === 'GMAPS') {
        gmap.panTo(currentPos);
    }
}

function update3DScene() {
    if (!ugvModel) return;

    // Smoothly interpolate UGV pose in 3D
    ugvModel.position.x = telemetry.robot.x;
    ugvModel.position.z = telemetry.robot.y;
    ugvModel.rotation.y = -(telemetry.robot.yaw_deg * Math.PI) / 180.0;

    // Update Path Lines
    if (telemetry.planner.global_path.length > 0) {
        const pts = telemetry.planner.global_path.map(([px, py]) => new THREE.Vector3(px, 0.12, py));
        plannedPathLine.geometry.setFromPoints(pts);
    }
    if (telemetry.slam.trajectory.length > 0) {
        const pts = telemetry.slam.trajectory.map(([tx, ty]) => new THREE.Vector3(tx, 0.08, ty));
        trajectoryLine.geometry.setFromPoints(pts);
    }

    // Camera Modes
    if (simCamMode === 'FOLLOW') {
        const rad = -(telemetry.robot.yaw_deg * Math.PI) / 180.0;
        camera.position.x = telemetry.robot.x - Math.cos(rad) * 4.5;
        camera.position.z = telemetry.robot.y - Math.sin(rad) * 4.5;
        camera.position.y = 2.8;
        camera.lookAt(telemetry.robot.x + 2.0, 0.5, telemetry.robot.y);
    } else if (simCamMode === 'TOP') {
        camera.position.set(10.0, 24.0, 1.75);
        camera.lookAt(10.0, 0, 1.75);
    } else if (simCamMode === 'FPV') {
        const rad = -(telemetry.robot.yaw_deg * Math.PI) / 180.0;
        camera.position.x = telemetry.robot.x + 0.35;
        camera.position.z = telemetry.robot.y;
        camera.position.y = 0.75;
        camera.lookAt(telemetry.robot.x + Math.cos(rad) * 8.0, 0.5, telemetry.robot.y + Math.sin(rad) * 8.0);
    }
}


// ==============================================================================
// Render Loop
// ==============================================================================
function renderLoop() {
    requestAnimationFrame(renderLoop);
    if (activeViewport === '3D' && renderer && scene && camera) {
        renderer.render(scene, camera);
    }
    draw2DMap();
}


// ==============================================================================
// Event Listeners & Buttons
// ==============================================================================
function setupEventListeners() {
    // Multi-Viewport Tabs Switcher (3D vs Google Maps vs Robot Eyes FPV)
    document.querySelectorAll('.btn-tab').forEach(tab => {
        tab.addEventListener('click', (e) => {
            document.querySelectorAll('.btn-tab').forEach(t => t.classList.remove('active'));
            e.target.classList.add('active');
            activeViewport = e.target.dataset.view;

            const threeContainer = document.getElementById('threejs-container');
            const gmapsContainer = document.getElementById('google-maps-container');
            const fpvContainer = document.getElementById('robot-fpv-container');
            const tag = document.getElementById('view-mode-tag');

            threeContainer.style.display = 'none';
            gmapsContainer.style.display = 'none';
            fpvContainer.style.display = 'none';

            if (activeViewport === '3D') {
                threeContainer.style.display = 'block';
                if (tag) tag.textContent = 'GAZEBO 3D DIGITAL TWIN';
                // Trigger resize for Three.js
                if (renderer && camera) {
                    const w = threeContainer.clientWidth;
                    const h = threeContainer.clientHeight;
                    camera.aspect = w / h;
                    camera.updateProjectionMatrix();
                    renderer.setSize(w, h);
                }
            } else if (activeViewport === 'GMAPS') {
                gmapsContainer.style.display = 'block';
                if (tag) tag.textContent = 'MUDUMALAI SATELLITE RECON';
                if (gmap) {
                    google.maps.event.trigger(gmap, 'resize');
                    if (telemetry.geodesic) {
                        gmap.setCenter({ lat: telemetry.geodesic.current_lat, lng: telemetry.geodesic.current_lng });
                    }
                }
            } else if (activeViewport === 'FPV') {
                fpvContainer.style.display = 'block';
                if (tag) tag.textContent = 'ROBOT FIRST PERSON EYE (PI-CAM V3)';
            }
        });
    });

    // Camera Stream Mode Buttons
    const camImg = document.getElementById('camera-stream-img');
    const fpvFullscreenImg = document.getElementById('fpv-fullscreen-stream');
    document.querySelectorAll('.btn-mode').forEach(btn => {
        btn.addEventListener('click', (e) => {
            if (e.target.id === 'btn-toggle-hud') return;
            document.querySelectorAll('.btn-mode').forEach(b => {
                if (b.id !== 'btn-toggle-hud') b.classList.remove('active');
            });
            e.target.classList.add('active');
            const mode = e.target.dataset.mode;
            let src = '/stream/overlay';
            if (mode === 'raw') src = '/stream/raw';
            else if (mode === 'mask') src = '/stream/mask';
            if (camImg) camImg.src = src;
            if (fpvFullscreenImg) fpvFullscreenImg.src = src;
        });
    });

    // Tactical HUD Overlay Toggle
    const hudBtn = document.getElementById('btn-toggle-hud');
    const hudReticle = document.getElementById('camera-hud-reticle');
    if (hudBtn && hudReticle) {
        hudBtn.addEventListener('click', () => {
            hudOverlayVisible = !hudOverlayVisible;
            hudReticle.style.display = hudOverlayVisible ? 'block' : 'none';
            hudBtn.classList.toggle('active', hudOverlayVisible);
        });
    }

    // 3D Viewport Camera View Buttons
    document.querySelectorAll('.btn-cam').forEach(btn => {
        btn.addEventListener('click', (e) => {
            document.querySelectorAll('.btn-cam').forEach(b => b.classList.remove('active'));
            e.target.classList.add('active');
            simCamMode = e.target.dataset.cam;
        });
    });

    // Terrain Risk Toggle
    const riskBtn = document.getElementById('btn-toggle-risk');
    if (riskBtn) {
        riskBtn.addEventListener('click', () => {
            terrainRiskEnabled = !terrainRiskEnabled;
            riskBtn.classList.toggle('active', terrainRiskEnabled);
        });
    }

    // Gemini API Key Input
    const keyBtn = document.getElementById('btn-save-key');
    const keyInput = document.getElementById('gemini-key-input');
    if (keyBtn && keyInput) {
        keyBtn.addEventListener('click', () => {
            const api_key = keyInput.value.trim();
            if (!api_key) return;
            keyBtn.textContent = 'CONNECTING...';
            fetch('/api/gemini/key', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ api_key })
            })
            .then(res => res.json())
            .then(() => {
                keyBtn.textContent = 'CONNECTED ✓';
                keyBtn.classList.remove('primary');
                keyBtn.classList.add('success');
                keyInput.value = '';
            })
            .catch(() => {
                keyBtn.textContent = 'RETRY';
            });
        });
    }

    // Mission Control Buttons
    document.querySelectorAll('.btn-ctrl').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const action = e.target.dataset.action;
            fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action })
            });
        });
    });
}


// ==============================================================================
// Keyboard Teleoperation (W, A, S, D / Arrows)
// ==============================================================================
function setupKeyboardTeleop() {
    window.addEventListener('keydown', (e) => {
        let v = 0.0, w = 0.0;
        if (e.key === 'w' || e.key === 'ArrowUp') v = 0.4;
        else if (e.key === 's' || e.key === 'ArrowDown') v = -0.3;
        else if (e.key === 'a' || e.key === 'ArrowLeft') w = 0.8;
        else if (e.key === 'd' || e.key === 'ArrowRight') w = -0.8;
        else return;

        fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action: 'teleop', v, w })
        });
    });
}
