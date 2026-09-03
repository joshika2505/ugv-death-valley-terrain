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
    stability_status: 'NORMAL',
    waypoints: []
};

let clickMode = 'NONE'; // 'SET_A', 'SET_B', 'NONE'

// World coordinates range [-50, 50] matching Death Valley Basin
const WORLD_MIN = -50.0;
const WORLD_MAX = 50.0;

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
    ctx.fillStyle = '#0e131a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 1. Topographic Canyon Basin (Traversable Green Valley Floor)
    ctx.fillStyle = 'rgba(28, 44, 32, 0.75)';
    ctx.beginPath();
    ctx.ellipse(canvas.width/2, canvas.height/2, 230, 190, 0, 0, Math.PI * 2);
    ctx.fill();

    // Canyon Boundary Ring (The Circle)
    ctx.strokeStyle = '#4a3b22';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.ellipse(canvas.width/2, canvas.height/2, 230, 190, 0, 0, Math.PI * 2);
    ctx.stroke();

    // Secondary Gentle Pass
    ctx.fillStyle = 'rgba(38, 54, 40, 0.4)';
    ctx.beginPath();
    ctx.ellipse(canvas.width/2 - 50, canvas.height/2 + 30, 140, 90, 0.3, 0, Math.PI * 2);
    ctx.fill();

    // Outer Mountain Ridge Contours
    ctx.strokeStyle = 'rgba(180, 100, 45, 0.3)';
    ctx.lineWidth = 1.2;
    for (let r = 80; r <= 220; r += 45) {
        ctx.beginPath();
        ctx.ellipse(canvas.width/2, canvas.height/2, r, r * 0.85, 0, 0, Math.PI * 2);
        ctx.stroke();
    }

    // 2. Coordinate Grid Lines & Metric Ticks
    ctx.strokeStyle = '#1a222c';
    ctx.lineWidth = 1;
    ctx.font = '9px Orbitron';
    ctx.fillStyle = '#4a5568';
    for (let wx = -40; wx <= 40; wx += 20) {
        const pt = worldToCanvas(wx, 0);
        ctx.beginPath(); ctx.moveTo(pt.x, 0); ctx.lineTo(pt.x, canvas.height); ctx.stroke();
        ctx.fillText(wx + 'm', pt.x + 2, canvas.height - 4);
    }
    for (let wy = -40; wy <= 40; wy += 20) {
        const pt = worldToCanvas(0, wy);
        ctx.beginPath(); ctx.moveTo(0, pt.y); ctx.lineTo(canvas.width, pt.y); ctx.stroke();
        ctx.fillText(wy + 'm', 4, pt.y - 2);
    }

    // 3. Draw Planned 3D Path (Inside the terrain area)
    if (state.waypoints && state.waypoints.length > 1) {
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 3.5;
        ctx.beginPath();
        let first = true;
        for (let w of state.waypoints) {
            // Keep points within map visualization
            const pt = worldToCanvas(w.x, w.y);
            if (first) {
                ctx.moveTo(pt.x, pt.y);
                first = false;
            } else {
                ctx.lineTo(pt.x, pt.y);
            }
        }
        ctx.stroke();

        // Waypoint breadcrumb dots
        ctx.fillStyle = 'rgba(0, 255, 136, 0.6)';
        for (let w of state.waypoints) {
            const pt = worldToCanvas(w.x, w.y);
            ctx.beginPath();
            ctx.arc(pt.x, pt.y, 3.5, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    // 4. Draw Point A (GREEN START)
    const ptA = worldToCanvas(state.start_a.x, state.start_a.y);
    ctx.fillStyle = '#00ff88';
    ctx.beginPath(); ctx.arc(ptA.x, ptA.y, 8, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = '#00ff88'; ctx.font = 'bold 12px Orbitron';
    ctx.fillText('A (START)', ptA.x + 12, ptA.y + 4);

    // 5. Draw Point B (RED DESTINATION)
    const ptB = worldToCanvas(state.destination_b.x, state.destination_b.y);
    ctx.fillStyle = '#ff3366';
    ctx.beginPath(); ctx.arc(ptB.x, ptB.y, 8, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = '#ff3366'; ctx.font = 'bold 12px Orbitron';
    ctx.fillText('B (DEST)', ptB.x + 12, ptB.y + 4);

    // 6. Draw Robot (CYAN Tracked Platform)
    // Clamp visualization to canvas area
    const clampedRx = Math.max(WORLD_MIN, Math.min(WORLD_MAX, state.robot_pose.x));
    const clampedRy = Math.max(WORLD_MIN, Math.min(WORLD_MAX, state.robot_pose.y));
    const rPos = worldToCanvas(clampedRx, clampedRy);
    
    ctx.save();
    ctx.translate(rPos.x, rPos.y);
    ctx.rotate(-state.robot_pose.yaw);
    ctx.fillStyle = '#00d2ff';
    ctx.beginPath();
    ctx.moveTo(14, 0);
    ctx.lineTo(-9, -9);
    ctx.lineTo(-5, 0);
    ctx.lineTo(-9, 9);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();
}

// Exact Screen-to-Canvas Ratio Click Handling
canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const cx = (e.clientX - rect.left) * scaleX;
    const cy = (e.clientY - rect.top) * scaleY;
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

// Live Hover Coordinates Tracking
canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const cx = (e.clientX - rect.left) * scaleX;
    const cy = (e.clientY - rect.top) * scaleY;
    const wc = canvasToWorld(cx, cy);
    const hoverEl = document.getElementById('hover-coord');
    if (hoverEl) {
        hoverEl.innerText = 'Cursor: X=' + wc.x.toFixed(1) + 'm, Y=' + wc.y.toFixed(1) + 'm';
    }
});

document.getElementById('btnSetA').onclick = () => {
    clickMode = 'SET_A';
    document.getElementById('btnSetA').style.borderColor = '#00ff88';
    document.getElementById('btnSetB').style.borderColor = '';
};

document.getElementById('btnSetB').onclick = () => {
    clickMode = 'SET_B';
    document.getElementById('btnSetB').style.borderColor = '#ff3366';
    document.getElementById('btnSetA').style.borderColor = '';
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
        return fetch('/api/plan_path', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        });
    }).then(() => fetchStatus());
};

document.getElementById('btnPlan').onclick = () => {
    fetch('/api/plan_path', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

document.getElementById('btnAltPlan').onclick = () => {
    fetch('/api/plan_alternative_path', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

const quickAltBtn = document.getElementById('btnQuickAlt');
if (quickAltBtn) {
    quickAltBtn.onclick = () => {
        fetch('/api/plan_alternative_path', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        })
        .then(() => fetch('/api/start_navigation', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: '{}'
        }))
        .then(() => fetchStatus());
    };
}

document.getElementById('btnStart').onclick = () => {
    fetch('/api/start_navigation', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

document.getElementById('btnStop').onclick = () => {
    fetch('/api/stop', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

document.getElementById('btnReset').onclick = () => {
    fetch('/api/reset', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

document.getElementById('btnRecover').onclick = () => {
    fetch('/api/recover_robot', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: '{}'
    }).then(() => fetchStatus());
};

// Continuous Camera Frame Updater
const camImg = document.getElementById('robotCamera');
let lastCamUpdate = 0;
function updateCameraFeed() {
    const now = Date.now();
    if (now - lastCamUpdate > 100) {
        lastCamUpdate = now;
        const tempImg = new Image();
        tempImg.onload = () => {
            if (camImg) camImg.src = tempImg.src;
        };
        tempImg.src = '/api/camera_frame?t=' + now;
    }
}
setInterval(updateCameraFeed, 120);

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
            
            const stabEl = document.getElementById('val-stability');
            if (stabEl) {
                const stab = data.stability_status || 'NORMAL';
                stabEl.innerText = stab;
                if (stab === 'CRITICAL_FLIPPED' || data.path_status === 'Ridge_Blocked' || data.path_status === 'Reversing') {
                    stabEl.style.color = '#ff3366';
                } else if (stab === 'ANTI_TIP_ACTIVE' || stab === 'CLIMBING_ELEVATION' || data.path_status === 'Terrain_Scanning') {
                    stabEl.style.color = '#ffbb00';
                } else {
                    stabEl.style.color = '#00d2ff';
                }
            }

            document.getElementById('val-clearance').innerText = data.min_clearance > 10 ? '> 5.0 m' : data.min_clearance.toFixed(2) + ' m';
            document.getElementById('mission-badge').innerText = data.path_status.toUpperCase();

            // Ridge Block Warning Banner
            const ridgeAlert = document.getElementById('ridge-alert');
            if (ridgeAlert) {
                if (data.path_status === 'Ridge_Blocked' || data.path_status === 'Reversing') {
                    ridgeAlert.classList.remove('hidden');
                } else {
                    ridgeAlert.classList.add('hidden');
                }
            }

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

drawTerrainCanvas();
