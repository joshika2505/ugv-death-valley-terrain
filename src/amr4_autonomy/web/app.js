
const canvas = document.getElementById('terrainCanvas');
const ctx = canvas.getContext('2d');

let state = {
    start_a: {x: 0.0, y: 0.0, z: 2.64},
    destination_b: {x: 15.0, y: 15.0, z: 0.0},
    distance_ab: 21.21,
    distance_remaining: 21.21,
    path_status: 'Ready',
    robot_speed: 0.0,
    robot_pose: {x: 0.0, y: 0.0, yaw: 0.0},
    robot_pitch: 0.0,
    robot_roll: 0.0,
    min_clearance: 5.0,
    terrain_class: 'Safe',
    waypoints: []
};

let clickMode = 'NONE'; // 'SET_A', 'SET_B', 'NONE'

// World coordinates range [-75, 75]
const WORLD_MIN = -75.0;
const WORLD_MAX = 75.0;

function worldToCanvas(wx, wy) {
    const cx = ((wx - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) * canvas.width;
    const cy = canvas.height - (((wy - WORLD_MIN) / (WORLD_MAX - WORLD_MIN)) * canvas.height);
    return {x: cx, y: cy};
}

function canvasToWorld(cx, cy) {
    const wx = WORLD_MIN + (cx / canvas.width) * (WORLD_MAX - WORLD_MIN);
    const wy = WORLD_MIN + ((canvas.height - cy) / canvas.height) * (WORLD_MAX - WORLD_MIN);
    return {x: wx, y: wy};
}

function drawTerrainCanvas() {
    ctx.fillStyle = '#12161c';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Grid lines
    ctx.strokeStyle = '#1e242c';
    ctx.lineWidth = 1;
    for (let i = 0; i <= canvas.width; i += 40) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
    }

    // Canyon boundary contours (mock visualization of terrain)
    ctx.strokeStyle = '#3a3020';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(canvas.width/2, canvas.height/2, 180, 0, Math.PI * 2);
    ctx.stroke();

    // Draw Planned Path
    if (state.waypoints && state.waypoints.length > 1) {
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 3;
        ctx.beginPath();
        const p0 = worldToCanvas(state.waypoints[0].x, state.waypoints[0].y);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < state.waypoints.length; i++) {
            const pt = worldToCanvas(state.waypoints[i].x, state.waypoints[i].y);
            ctx.lineTo(pt.x, pt.y);
        }
        ctx.stroke();

        // Waypoint dots
        ctx.fillStyle = 'rgba(0, 255, 136, 0.4)';
        for (let w of state.waypoints) {
            const pt = worldToCanvas(w.x, w.y);
            ctx.beginPath();
            ctx.arc(pt.x, pt.y, 3, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    // Draw Point A (GREEN)
    const ptA = worldToCanvas(state.start_a.x, state.start_a.y);
    ctx.fillStyle = '#00ff88';
    ctx.beginPath();
    ctx.arc(ptA.x, ptA.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#00ff88';
    ctx.font = 'bold 12px Orbitron';
    ctx.fillText('A (START)', ptA.x + 12, ptA.y + 4);

    // Draw Point B (RED)
    const ptB = worldToCanvas(state.destination_b.x, state.destination_b.y);
    ctx.fillStyle = '#ff3366';
    ctx.beginPath();
    ctx.arc(ptB.x, ptB.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#ff3366';
    ctx.font = 'bold 12px Orbitron';
    ctx.fillText('B (DEST)', ptB.x + 12, ptB.y + 4);

    // Draw Robot (CYAN Tri-Arrow)
    const rPos = worldToCanvas(state.robot_pose.x, state.robot_pose.y);
    ctx.save();
    ctx.translate(rPos.x, rPos.y);
    ctx.rotate(-state.robot_pose.yaw);
    ctx.fillStyle = '#00d2ff';
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-8, -8);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-8, 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
}

canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const worldCoord = canvasToWorld(cx, cy);

    if (clickMode === 'SET_A') {
        fetch('/api/set_point_a', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({x: worldCoord.x, y: worldCoord.y})
        }).then(() => {
            clickMode = 'NONE';
            document.getElementById('btnSetA').style.borderColor = '';
            fetchStatus();
        });
    } else if (clickMode === 'SET_B') {
        fetch('/api/set_point_b', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({x: worldCoord.x, y: worldCoord.y})
        }).then(() => {
            clickMode = 'NONE';
            document.getElementById('btnSetB').style.borderColor = '';
            fetchStatus();
        });
    }
});

document.getElementById('btnSetA').onclick = () => {
    clickMode = 'SET_A';
    document.getElementById('btnSetA').style.borderColor = '#00ff88';
};

document.getElementById('btnSetB').onclick = () => {
    clickMode = 'SET_B';
    document.getElementById('btnSetB').style.borderColor = '#ff3366';
};

document.getElementById('btnApplyCoords').onclick = () => {
    const ax = parseFloat(document.getElementById('input-ax').value);
    const ay = parseFloat(document.getElementById('input-ay').value);
    const bx = parseFloat(document.getElementById('input-bx').value);
    const by = parseFloat(document.getElementById('input-by').value);

    Promise.all([
        fetch('/api/set_point_a', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({x: ax, y: ay})
        }),
        fetch('/api/set_point_b', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({x: bx, y: by})
        })
    ]).then(() => {
        return fetch('/api/plan_path', {method: 'POST'});
    }).then(() => fetchStatus());
};

document.getElementById('btnPlan').onclick = () => {
    fetch('/api/plan_path', {method: 'POST'}).then(() => fetchStatus());
};

document.getElementById('btnStart').onclick = () => {
    fetch('/api/start_navigation', {method: 'POST'}).then(() => fetchStatus());
};

document.getElementById('btnStop').onclick = () => {
    fetch('/api/stop', {method: 'POST'}).then(() => fetchStatus());
};

document.getElementById('btnReset').onclick = () => {
    fetch('/api/reset', {method: 'POST'}).then(() => fetchStatus());
};

function fetchStatus() {
    fetch('/api/status')
        .then(res => res.json())
        .then(data => {
            state = data;
            if (document.activeElement !== document.getElementById('input-ax')) {
                document.getElementById('input-ax').value = data.start_a.x.toFixed(2);
            }
            if (document.activeElement !== document.getElementById('input-ay')) {
                document.getElementById('input-ay').value = data.start_a.y.toFixed(2);
            }
            document.getElementById('val-az').innerText = data.start_a.z.toFixed(2);

            if (document.activeElement !== document.getElementById('input-bx')) {
                document.getElementById('input-bx').value = data.destination_b.x.toFixed(2);
            }
            if (document.activeElement !== document.getElementById('input-by')) {
                document.getElementById('input-by').value = data.destination_b.y.toFixed(2);
            }
            document.getElementById('val-bz').innerText = data.destination_b.z.toFixed(2);

            document.getElementById('val-dist-ab').innerText = data.distance_ab.toFixed(2) + ' m';
            document.getElementById('val-dist-rem').innerText = data.distance_remaining.toFixed(2) + ' m';
            document.getElementById('val-speed').innerText = data.robot_speed.toFixed(2) + ' m/s';
            document.getElementById('val-terrain').innerText = data.terrain_class;
            document.getElementById('val-pitch-roll').innerText = data.robot_pitch.toFixed(1) + '° / ' + data.robot_roll.toFixed(1) + '°';
            document.getElementById('val-clearance').innerText = data.min_clearance > 10 ? '> 5.0 m' : data.min_clearance.toFixed(2) + ' m';
            document.getElementById('mission-badge').innerText = data.path_status.toUpperCase();

            const arrivalBanner = document.getElementById('arrival-banner');
            if (data.path_status === 'Completed') {
                arrivalBanner.classList.remove('hidden');
            } else {
                arrivalBanner.classList.add('hidden');
            }

            drawTerrainCanvas();
        })
        .catch(() => {});
}

// Refresh status every 200ms
setInterval(fetchStatus, 200);

// Refresh camera frame every 500ms
setInterval(() => {
    const img = document.getElementById('robotCamera');
    img.src = '/api/camera_frame?t=' + Date.now();
}, 500);

drawTerrainCanvas();
