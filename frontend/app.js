/**
 * ==============================================================================
 * AMR FLEET NAVIGATOR - MINIMALIST CLIENT APPLICATION
 * Features: 2D/3D Map Visualizer, Task Manager, Heatmap Analytics, English Logs
 * ==============================================================================
 */

// --- GLOBAL CONSTANTS & WAREHOUSE DEFINITION (demo_warehouse.yaml) ---
const WAREHOUSE_CONFIG = {
  width_m: 45.0,
  height_m: 60.0,
  resolution_m: 0.5,
  origin_x: -22.5,
  origin_y: -30.0,
  
  // 4 Charging bay pads at south center
  charging_pads: [
    { id: 'PAD_1', x: -3.6, y: -28.5, label: 'Pad 1 (Fast Inductive)' },
    { id: 'PAD_2', x: -1.2, y: -28.5, label: 'Pad 2 (Fast Inductive)' },
    { id: 'PAD_3', x: 1.2, y: -28.5, label: 'Pad 3 (Fast Inductive)' },
    { id: 'PAD_4', x: 3.6, y: -28.5, label: 'Pad 4 (Fast Inductive)' },
  ],

  // Corridors
  corridors: [
    { id: 'corridor_west', x: -9.0, width: 2.2, orientation: 'V' },
    { id: 'corridor_east', x: 9.0, width: 2.2, orientation: 'V' },
    { id: 'corridor_south', y: -10.0, height: 2.2, orientation: 'H' },
    { id: 'corridor_north', y: 10.0, height: 2.2, orientation: 'H' },
  ],

  shelves: []
};

// Generate warehouse racks matching demo_warehouse.yaml
function buildWarehouseShelves() {
  const x_zones = {
    west: [-19.99, -15.67, -11.35],
    center: [-2.5363, 2.5363],
    east: [11.35, 15.67, 19.99]
  };
  const y_zones = {
    south: [-28.79, -24.72, -20.65, -16.58, -12.51],
    middle: [-7.12, -3.05, 1.01, 5.08],
    north: [12.51, 16.58, 20.65, 24.72, 28.79]
  };

  let count = 1;
  for (const [zx, xList] of Object.entries(x_zones)) {
    for (const x of xList) {
      for (const [zy, yList] of Object.entries(y_zones)) {
        if (zx === 'center' && zy === 'south') continue; // Charging area
        for (const y of yList) {
          const zoneTag = `${zx[0].toUpperCase()}-${count.toString().padStart(2, '0')}`;
          WAREHOUSE_CONFIG.shelves.push({
            id: `Rack ${zoneTag}`,
            code: `RACK_${count++}`,
            zone: `${zx.toUpperCase()}_${zy.toUpperCase()}`,
            x: x,
            y: y,
            w: 3.92,
            h: 1.10,
            depth_3d: 2.2,
            itemType: ['Industrial Bearings', 'Servo Motors', 'Sensors', 'Hydraulic Valves', 'Pneumatics'][count % 5]
          });
        }
      }
    }
  }
}
buildWarehouseShelves();

// --- STATE MANAGEMENT ---
const APP_STATE = {
  activeTab: 'dashboard',
  viewMode: '2D', // '2D' or '3D'
  simSpeed: 1.0,
  isPaused: false,
  isEStopped: false,
  showTrails: true,
  showThoughts: true,
  selectedRobotId: 'robot_1',
  hoveredRobot: null,
  
  // Heatmap tracking grid
  heatmapGrid: Array(60).fill(0).map(() => Array(45).fill(0)),

  robots: [
    {
      id: 'robot_1',
      name: 'ROBOT_1',
      namespace: '/robot_1',
      color: '#2563eb', // Uber Blue
      x: -15.67,
      y: -5.0,
      theta: Math.PI / 2,
      speed: 0.85,
      battery: 84.0,
      isCharging: false,
      state: 'EN_ROUTE_PICKUP',
      task: {
        orderId: 'ORD-104',
        location: 'Rack W-04 (West Zone)',
        item: 'SKU-4921: Industrial Servo Bearing',
        etaSeconds: 42,
        demand: 8
      },
      path: [
        { x: -15.67, y: -5.0 },
        { x: -9.0, y: -5.0 },
        { x: -9.0, y: 16.58 },
        { x: -15.67, y: 16.58 }
      ],
      pathIdx: 0,
      thought: 'Bidding $8.20 for Task W-04'
    },
    {
      id: 'robot_2',
      name: 'ROBOT_2',
      namespace: '/robot_2',
      color: '#10b981', // Emerald
      x: -1.2,
      y: -28.5,
      theta: Math.PI / 2,
      speed: 0.0,
      battery: 98.0,
      isCharging: true,
      state: 'CHARGING_DOCKED',
      task: null,
      path: [],
      pathIdx: 0,
      thought: 'Inductive Fast Charge: 98%'
    },
    {
      id: 'robot_3',
      name: 'ROBOT_3',
      namespace: '/robot_3',
      color: '#8b5cf6', // Purple
      x: 9.0,
      y: 0.0,
      theta: Math.PI / 2,
      speed: 0.70,
      battery: 68.5,
      isCharging: false,
      state: 'CORRIDOR_MUTEX_WAIT',
      task: {
        orderId: 'ORD-108',
        location: 'Rack E-09 (East Zone)',
        item: 'SKU-7729: Hydraulic Manifold',
        etaSeconds: 75,
        demand: 4
      },
      path: [
        { x: 9.0, y: 0.0 },
        { x: 9.0, y: 20.65 },
        { x: 15.67, y: 20.65 }
      ],
      pathIdx: 0,
      thought: 'Requesting Corridor East (Lock #42)'
    },
    {
      id: 'robot_4',
      name: 'ROBOT_4',
      namespace: '/robot_4',
      color: '#f59e0b', // Amber
      x: 15.67,
      y: -20.65,
      theta: -Math.PI / 2,
      speed: 0.78,
      battery: 52.0,
      isCharging: false,
      state: 'EN_ROUTE_DROPOFF',
      task: {
        orderId: 'ORD-099',
        location: 'Staging Outbound Bay 1',
        item: 'SKU-1120: Micro Stepper Controller',
        etaSeconds: 28,
        demand: 12
      },
      path: [
        { x: 15.67, y: -20.65 },
        { x: 9.0, y: -20.65 },
        { x: 9.0, y: -28.5 },
        { x: 3.6, y: -28.5 }
      ],
      pathIdx: 0,
      thought: 'En Route to Outbound Staging'
    }
  ],

  obstacles: [
    { x: 0.0, y: 8.0, w: 2.0, h: 2.0, reason: 'Temporary Pallet Obstruction' }
  ],

  // TASK TABLE STORE (Directly inspired by Reference 1)
  tasks: [
    { id: 0, orderId: 'ORD-100', location: 'Adarsh Palm Staging', taskType: 'Starting Location', assignedAmr: 'None', sta: '-', ata: '-', std: '09:50', atd: '09:05', demand: 0, delay: '-00:45', status: 'Started', priority: 'Standard (P3)' },
    { id: 1, orderId: 'ORD-101', location: 'Rack W-04 (West Zone)', taskType: 'Pickup', assignedAmr: 'robot_1', sta: '09:59', ata: '09:07', std: '10:01', atd: '09:07', demand: 12, delay: '-00:54', status: 'Done', priority: 'High (P2)' },
    { id: 2, orderId: 'ORD-102', location: 'Rack C-06 (Center Zone)', taskType: 'Pickup', assignedAmr: 'robot_2', sta: '10:04', ata: '09:30', std: '10:07', atd: '09:30', demand: 8, delay: '-00:37', status: 'Done', priority: 'Standard (P3)' },
    { id: 3, orderId: 'ORD-103', location: 'Rack E-09 (East Zone)', taskType: 'Pickup', assignedAmr: 'robot_3', sta: '10:12', ata: '09:31', std: '10:14', atd: '09:31', demand: 6, delay: '-00:43', status: 'Done', priority: 'High (P2)' },
    { id: 4, orderId: 'ORD-104', location: 'Rack W-18 (West Zone)', taskType: 'Pickup', assignedAmr: 'robot_1', sta: '10:18', ata: '-', std: '10:22', atd: '-', demand: 8, delay: '-00:15', status: 'En Route', priority: 'Urgent (P1)' },
    { id: 5, orderId: 'ORD-105', location: 'Rack E-22 (East Zone)', taskType: 'Dropoff', assignedAmr: 'robot_4', sta: '10:25', ata: '-', std: '10:28', atd: '-', demand: 10, delay: '+00:05', status: 'En Route', priority: 'High (P2)' },
    { id: 6, orderId: 'ORD-106', location: 'Rack C-14 (Center Zone)', taskType: 'Transfer', assignedAmr: 'Unassigned', sta: '10:35', ata: '-', std: '10:40', atd: '-', demand: 4, delay: '00:00', status: 'Pending', priority: 'Standard (P3)' }
  ],

  // READABLE ENGLISH LOGS STORE
  logs: [
    { time: '10:18:22', robot: 'ROBOT_1', category: 'TASK_ALLOCATION', description: 'Won decentralized auction for Order ORD-104 (Rack W-18) with optimal path cost 8.20.', status: 'Success' },
    { time: '10:18:25', robot: 'ROBOT_3', category: 'CORRIDOR_MUTEX', description: 'Requested exclusive access lock for Corridor East. Granted via Lamport timestamp order.', status: 'Active' },
    { time: '10:18:30', robot: 'ROBOT_2', category: 'BATTERY_POWER', description: 'Docked at Inductive Charging Station Pad 2. Fast charging at 2.5% per minute.', status: 'Charging' },
    { time: '10:18:44', robot: 'ROBOT_4', category: 'PATH_ROUTING', description: 'WHCA* reservation completed without time-space conflicts across 16 planning steps.', status: 'Optimal' },
    { time: '10:19:02', robot: 'FLEET', category: 'SAFETY_ALERT', description: 'Temporary pallet detected in Central Aisle. Dynamic costmap updated across all peers.', status: 'Warning' }
  ]
};

// --- LOGGING UTILITY (HUMAN READABLE ENGLISH) ---
function addSystemLog(robotName, category, description, status = 'Info') {
  const timeStr = new Date().toLocaleTimeString('en-US', { hour12: false });
  APP_STATE.logs.unshift({ time: timeStr, robot: robotName, category, description, status });
  if (APP_STATE.logs.length > 120) APP_STATE.logs.pop();
  renderLogsTable();
  renderRecentLogsDashboard();
}

// --- TAB ROUTING ---
function initNavigation() {
  const tabs = document.querySelectorAll('.nav-tab');
  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const pageId = tab.dataset.page;
      tabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');

      document.querySelectorAll('.page-view').forEach(p => p.classList.remove('active'));
      const targetPage = document.getElementById(`page-${pageId}`);
      if (targetPage) targetPage.classList.add('active');
      APP_STATE.activeTab = pageId;

      // Resize canvas on tab switch
      setTimeout(() => {
        resizeActiveCanvases();
      }, 50);
    });
  });

  // Link from dashboard to logs
  const btnAllLogs = document.getElementById('btn-view-all-logs');
  if (btnAllLogs) {
    btnAllLogs.addEventListener('click', () => {
      document.querySelector('[data-page="logs"]').click();
    });
  }

  // 2D / 3D Toggle Buttons (Sync Dashboard & Full Map)
  const btnDash2D = document.getElementById('dash-btn-2d');
  const btnDash3D = document.getElementById('dash-btn-3d');
  const btnMap2D = document.getElementById('map-btn-2d');
  const btnMap3D = document.getElementById('map-btn-3d');

  function setViewMode(mode) {
    APP_STATE.viewMode = mode;
    [btnDash2D, btnMap2D].forEach(b => b && b.classList.toggle('active', mode === '2D'));
    [btnDash3D, btnMap3D].forEach(b => b && b.classList.toggle('active', mode === '3D'));
  }

  if (btnDash2D) btnDash2D.addEventListener('click', () => setViewMode('2D'));
  if (btnDash3D) btnDash3D.addEventListener('click', () => setViewMode('3D'));
  if (btnMap2D) btnMap2D.addEventListener('click', () => setViewMode('2D'));
  if (btnMap3D) btnMap3D.addEventListener('click', () => setViewMode('3D'));

  // Sim speed toggles
  document.querySelectorAll('.speed-toggle').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.speed-toggle').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      APP_STATE.simSpeed = parseFloat(btn.dataset.speed);
    });
  });

  // Pause toggle
  const pauseBtn = document.getElementById('global-pause-btn');
  pauseBtn.addEventListener('click', () => {
    APP_STATE.isPaused = !APP_STATE.isPaused;
    pauseBtn.textContent = APP_STATE.isPaused ? '▶' : '⏸';
  });

  // E-Stop toggle
  const estopBtn = document.getElementById('global-estop-btn');
  estopBtn.addEventListener('click', () => {
    APP_STATE.isEStopped = !APP_STATE.isEStopped;
    if (APP_STATE.isEStopped) {
      estopBtn.style.background = '#dc2626';
      estopBtn.style.color = '#fff';
      estopBtn.innerHTML = '<span>⚡</span> Resume';
      addSystemLog('FLEET', 'SAFETY_ALERT', 'Global emergency stop triggered by supervisor. All AMRs braking.', 'Emergency');
    } else {
      estopBtn.style.background = '#fee2e2';
      estopBtn.style.color = '#991b1b';
      estopBtn.innerHTML = '<span>🛑</span> E-Stop';
      addSystemLog('FLEET', 'SAFETY_ALERT', 'Emergency stop released. Resuming standard autonomous routing.', 'Info');
    }
  });

  // Trail / Bubble toggles
  const trailBtn = document.getElementById('dash-toggle-trails');
  if (trailBtn) {
    trailBtn.addEventListener('click', () => {
      APP_STATE.showTrails = !APP_STATE.showTrails;
      trailBtn.classList.toggle('active', APP_STATE.showTrails);
      trailBtn.textContent = `Trails: ${APP_STATE.showTrails ? 'ON' : 'OFF'}`;
    });
  }

  const thoughtBtn = document.getElementById('dash-toggle-bubbles');
  if (thoughtBtn) {
    thoughtBtn.addEventListener('click', () => {
      APP_STATE.showThoughts = !APP_STATE.showThoughts;
      thoughtBtn.classList.toggle('active', APP_STATE.showThoughts);
      thoughtBtn.textContent = `Thoughts: ${APP_STATE.showThoughts ? 'ON' : 'OFF'}`;
    });
  }
}

// --- CANVAS RENDERING (2D & 3D ISOMETRIC ENGINE) ---
const dashCanvas = document.getElementById('dashWarehouseCanvas');
const fullCanvas = document.getElementById('fullWarehouseCanvas');
const heatmapCanvas = document.getElementById('statsHeatmapCanvas');

let dashCtx = dashCanvas ? dashCanvas.getContext('2d') : null;
let fullCtx = fullCanvas ? fullCanvas.getContext('2d') : null;
let heatCtx = heatmapCanvas ? heatmapCanvas.getContext('2d') : null;

function resizeActiveCanvases() {
  const containerDash = document.getElementById('dash-canvas-container');
  if (containerDash && dashCanvas) {
    dashCanvas.width = containerDash.clientWidth * window.devicePixelRatio;
    dashCanvas.height = containerDash.clientHeight * window.devicePixelRatio;
    dashCtx = dashCanvas.getContext('2d');
    dashCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
  }

  const containerFull = document.getElementById('fullmap-canvas-container');
  if (containerFull && fullCanvas) {
    fullCanvas.width = containerFull.clientWidth * window.devicePixelRatio;
    fullCanvas.height = containerFull.clientHeight * window.devicePixelRatio;
    fullCtx = fullCanvas.getContext('2d');
    fullCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
  }

  const containerHeat = heatmapCanvas ? heatmapCanvas.parentElement : null;
  if (containerHeat && heatmapCanvas) {
    heatmapCanvas.width = containerHeat.clientWidth * window.devicePixelRatio;
    heatmapCanvas.height = containerHeat.clientHeight * window.devicePixelRatio;
    heatCtx = heatmapCanvas.getContext('2d');
    heatCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
    renderStaticHeatmap();
  }
}
window.addEventListener('resize', resizeActiveCanvases);

// Projection: World (Meters) -> Screen (Pixels)
function projectWorld(wx, wy, wz, cWidth, cHeight, mode = '2D') {
  if (mode === '2D') {
    const scale = Math.min(cWidth / (WAREHOUSE_CONFIG.width_m + 6), cHeight / (WAREHOUSE_CONFIG.height_m + 6));
    const sx = cWidth / 2 + (wx * scale);
    const sy = cHeight / 2 - (wy * scale);
    return { x: sx, y: sy, scale };
  } else {
    // 3D Isometric Projection (Matching Reference 2)
    const isoScale = Math.min(cWidth / 68, cHeight / 68) * 0.95;
    const isoAngle = Math.PI / 6; // 30 degrees
    
    // Isometric transformation: x_iso = (x - y) * cos(30), y_iso = (x + y) * sin(30) - z
    const isoX = (wx - wy * 0.8) * Math.cos(isoAngle);
    const isoY = (wx + wy * 0.8) * Math.sin(isoAngle) - (wz * 1.6);
    
    const sx = cWidth / 2 + (isoX * isoScale);
    const sy = cHeight / 2 + (isoY * isoScale) + 30;
    return { x: sx, y: sy, scale: isoScale };
  }
}

// Render Warehouse Scene
function drawWarehouseScene(ctx, cWidth, cHeight, mode) {
  ctx.clearRect(0, 0, cWidth, cHeight);

  // Background Floor
  ctx.fillStyle = mode === '3D' ? '#e2e8f0' : '#f8fafc';
  ctx.fillRect(0, 0, cWidth, cHeight);

  // 1. Corridors
  if (mode === '2D') {
    ctx.fillStyle = '#f1f5f9';
    for (const c of WAREHOUSE_CONFIG.corridors) {
      if (c.orientation === 'V') {
        const p1 = projectWorld(c.x - c.width / 2, WAREHOUSE_CONFIG.height_m / 2, 0, cWidth, cHeight, mode);
        const p2 = projectWorld(c.x + c.width / 2, -WAREHOUSE_CONFIG.height_m / 2, 0, cWidth, cHeight, mode);
        ctx.fillRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
      } else {
        const p1 = projectWorld(-WAREHOUSE_CONFIG.width_m / 2, c.y + c.height / 2, 0, cWidth, cHeight, mode);
        const p2 = projectWorld(WAREHOUSE_CONFIG.width_m / 2, c.y - c.height / 2, 0, cWidth, cHeight, mode);
        ctx.fillRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
      }
    }
  }

  // 2. Charging Bay Pads
  for (const pad of WAREHOUSE_CONFIG.charging_pads) {
    const p = projectWorld(pad.x, pad.y, 0, cWidth, cHeight, mode);
    if (mode === '2D') {
      const pw = 2.4 * p.scale;
      const ph = 1.8 * p.scale;
      ctx.fillStyle = '#ecfdf5';
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.roundRect(p.x - pw / 2, p.y - ph / 2, pw, ph, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#065f46';
      ctx.font = 'bold 9px Inter';
      ctx.textAlign = 'center';
      ctx.fillText(`⚡ ${pad.id}`, p.x, p.y + 3);
    } else {
      // 3D Charging Pad Plate
      ctx.fillStyle = '#a7f3d0';
      ctx.strokeStyle = '#059669';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  // 3. Shelves / Racks
  // Sort shelves back-to-front for 3D depth
  const shelvesSorted = mode === '3D' 
    ? [...WAREHOUSE_CONFIG.shelves].sort((a, b) => (a.x + a.y) - (b.x + b.y))
    : WAREHOUSE_CONFIG.shelves;

  for (const s of shelvesSorted) {
    if (mode === '2D') {
      const p = projectWorld(s.x, s.y, 0, cWidth, cHeight, mode);
      const sw = s.w * p.scale;
      const sh = s.h * p.scale;
      ctx.fillStyle = '#e2e8f0';
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(p.x - sw / 2, p.y - sh / 2, sw, sh, 3);
      ctx.fill();
      ctx.stroke();

      // Divider slots
      ctx.strokeStyle = '#94a3b8';
      ctx.beginPath();
      ctx.moveTo(p.x - sw / 4, p.y - sh / 2);
      ctx.lineTo(p.x - sw / 4, p.y + sh / 2);
      ctx.moveTo(p.x + sw / 4, p.y - sh / 2);
      ctx.lineTo(p.x + sw / 4, p.y + sh / 2);
      ctx.stroke();
    } else {
      // 3D Isometric Extruded Shelf Block (Reference 2 style)
      const pBase = projectWorld(s.x, s.y, 0, cWidth, cHeight, mode);
      const pTop = projectWorld(s.x, s.y, s.depth_3d, cWidth, cHeight, mode);
      
      const bW = 18;
      const bH = 10;
      const bHeight = pBase.y - pTop.y;

      // Front Face
      ctx.fillStyle = '#cbd5e1';
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.rect(pTop.x - bW / 2, pTop.y, bW, bHeight);
      ctx.fill();
      ctx.stroke();

      // Top Face
      ctx.fillStyle = '#e2e8f0';
      ctx.beginPath();
      ctx.rect(pTop.x - bW / 2, pTop.y - bH, bW, bH);
      ctx.fill();
      ctx.stroke();
    }
  }

  // 4. Dynamic Obstacles
  for (const obs of APP_STATE.obstacles) {
    const p = projectWorld(obs.x, obs.y, 0, cWidth, cHeight, mode);
    ctx.fillStyle = '#fef2f2';
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(p.x - 14, p.y - 14, 28, 28, 4);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#dc2626';
    ctx.font = 'bold 10px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('⚠️', p.x, p.y + 4);
  }

  // 5. Navigation Path Ribbons (Uber Blue / Crisp Glow)
  if (APP_STATE.showTrails) {
    for (const bot of APP_STATE.robots) {
      if (!bot.path || bot.path.length <= 1) continue;
      
      ctx.strokeStyle = bot.id === APP_STATE.selectedRobotId ? '#2563eb' : '#60a5fa';
      ctx.lineWidth = bot.id === APP_STATE.selectedRobotId ? 3.5 : 2;
      ctx.beginPath();

      const pStart = projectWorld(bot.x, bot.y, 0, cWidth, cHeight, mode);
      ctx.moveTo(pStart.x, pStart.y);

      for (let i = bot.pathIdx; i < bot.path.length; i++) {
        const pNode = projectWorld(bot.path[i].x, bot.path[i].y, 0, cWidth, cHeight, mode);
        ctx.lineTo(pNode.x, pNode.y);
      }
      ctx.stroke();

      // Target Waypoint Pin
      if (bot.path.length > 0) {
        const dest = bot.path[bot.path.length - 1];
        const pDest = projectWorld(dest.x, dest.y, 0, cWidth, cHeight, mode);
        
        ctx.fillStyle = '#2563eb';
        ctx.beginPath();
        ctx.arc(pDest.x, pDest.y, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }
  }

  // 6. AMRs (Robots)
  for (const bot of APP_STATE.robots) {
    const p = projectWorld(bot.x, bot.y, mode === '3D' ? 0.4 : 0, cWidth, cHeight, mode);
    const isSelected = bot.id === APP_STATE.selectedRobotId;

    // Outer Selection Halo
    if (isSelected) {
      ctx.strokeStyle = 'rgba(37, 99, 235, 0.4)';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 16, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Robot Circle Sprite (Uber Style navigation dot)
    ctx.fillStyle = bot.color;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Direction Heading Cone
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-bot.theta);
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(6, 0);
    ctx.lineTo(2, -3);
    ctx.lineTo(2, 3);
    ctx.fill();
    ctx.restore();

    // Robot Label
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 10px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(bot.name, p.x, p.y + 20);

    // Thought Bubble
    if (APP_STATE.showThoughts && bot.thought) {
      drawMinimalThoughtBubble(ctx, p.x, p.y - 18, bot.thought);
    }
  }
}

function drawMinimalThoughtBubble(ctx, x, y, text) {
  ctx.font = '600 10px Inter';
  const tw = ctx.measureText(text).width;
  const bw = tw + 14;
  const bh = 20;

  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#cbd5e1';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(x - bw / 2, y - bh, bw, bh, 4);
  ctx.fill();
  ctx.stroke();

  // Down pointer
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(x - 3, y);
  ctx.lineTo(x + 3, y);
  ctx.lineTo(x, y + 4);
  ctx.fill();

  ctx.fillStyle = '#334155';
  ctx.textAlign = 'center';
  ctx.fillText(text, x, y - 6);
}

// --- SIMULATION STEP ENGINE ---
let lastAnimTime = performance.now();

function updateSimulationEngine(dt) {
  if (APP_STATE.isPaused || APP_STATE.isEStopped) return;

  for (const bot of APP_STATE.robots) {
    // Battery Decay / Charge
    if (bot.isCharging) {
      bot.battery = Math.min(100, bot.battery + dt * 2.0 * APP_STATE.simSpeed);
    } else {
      bot.battery = Math.max(0, bot.battery - dt * 0.12 * APP_STATE.simSpeed);
    }

    // Waypoint Navigation
    if (bot.path && bot.path.length > 0 && bot.pathIdx < bot.path.length) {
      const targetWp = bot.path[bot.pathIdx];
      const dx = targetWp.x - bot.x;
      const dy = targetWp.y - bot.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist < 0.2) {
        bot.pathIdx++;
        if (bot.pathIdx >= bot.path.length) {
          // Finished route! Cycle task
          handleTaskCompletion(bot);
        }
      } else {
        const angle = Math.atan2(dy, dx);
        bot.theta = angle;
        const step = bot.speed * dt * APP_STATE.simSpeed;
        bot.x += Math.cos(angle) * Math.min(step, dist);
        bot.y += Math.sin(angle) * Math.min(step, dist);

        // Update heatmap grid
        const hx = Math.min(44, Math.max(0, Math.floor(bot.x + 22.5)));
        const hy = Math.min(59, Math.max(0, Math.floor(bot.y + 30.0)));
        APP_STATE.heatmapGrid[hy][hx] += dt * 0.5;
      }
    }
  }

  updateUberDirectionCard();
}

function handleTaskCompletion(bot) {
  if (bot.state === 'EN_ROUTE_PICKUP') {
    bot.state = 'PICKUP_COMPLETE';
    bot.thought = 'Loaded Item: Moving to Staging';
    addSystemLog(bot.name, 'TASK_ALLOCATION', `Item retrieved at ${bot.task?.location || 'Aisle'}. Proceeding to delivery bay.`, 'Success');

    setTimeout(() => {
      bot.state = 'EN_ROUTE_DROPOFF';
      bot.path = generateNavPath(bot.x, bot.y, 9.0, -28.5);
      bot.pathIdx = 0;
    }, 2000 / APP_STATE.simSpeed);

  } else if (bot.state === 'EN_ROUTE_DROPOFF') {
    bot.state = 'IDLE';
    bot.thought = 'Order Delivered: Ready for Next Auction';
    addSystemLog(bot.name, 'TASK_ALLOCATION', `Delivered ${bot.task?.item || 'Goods'} at destination staging. Score verified.`, 'Success');

    // Pick new random shelf
    setTimeout(() => {
      const randomShelf = WAREHOUSE_CONFIG.shelves[Math.floor(Math.random() * WAREHOUSE_CONFIG.shelves.length)];
      bot.state = 'EN_ROUTE_PICKUP';
      bot.task = {
        orderId: `ORD-${Math.floor(100 + Math.random() * 50)}`,
        location: randomShelf.id,
        item: `SKU-${Math.floor(1000 + Math.random() * 9000)}: ${randomShelf.itemType}`,
        etaSeconds: 45,
        demand: Math.floor(2 + Math.random() * 10)
      };
      bot.path = generateNavPath(bot.x, bot.y, randomShelf.x, randomShelf.y);
      bot.pathIdx = 0;
      bot.thought = `Bidding on ${randomShelf.id}`;
      addSystemLog(bot.name, 'TASK_ALLOCATION', `Won CBBA auction for ${randomShelf.id} (${bot.task.item}).`, 'Success');
    }, 2500 / APP_STATE.simSpeed);
  }
}

function generateNavPath(sx, sy, gx, gy) {
  const vx = sx < 0 ? -9.0 : 9.0;
  const targetVx = gx < 0 ? -9.0 : 9.0;
  const hy = sy < 0 ? -10.0 : 10.0;

  return [
    { x: sx, y: sy },
    { x: vx, y: sy },
    { x: vx, y: hy },
    { x: targetVx, y: hy },
    { x: targetVx, y: gy },
    { x: gx, y: gy }
  ];
}

// --- MAIN ANIMATION LOOP ---
function animLoop(timestamp) {
  const dt = (timestamp - lastAnimTime) / 1000;
  lastAnimTime = timestamp;

  updateSimulationEngine(dt);

  // Render active canvases
  if (dashCtx && dashCanvas) {
    const rect = dashCanvas.getBoundingClientRect();
    drawWarehouseScene(dashCtx, rect.width, rect.height, APP_STATE.viewMode);
  }

  if (fullCtx && fullCanvas && APP_STATE.activeTab === 'map-view') {
    const rect = fullCanvas.getBoundingClientRect();
    drawWarehouseScene(fullCtx, rect.width, rect.height, APP_STATE.viewMode);
  }

  requestAnimationFrame(animLoop);
}

// --- UBER DIRECTION CARD & HOVER TOOLTIP ---
function updateUberDirectionCard() {
  const activeBot = APP_STATE.robots.find(r => r.id === APP_STATE.selectedRobotId) || APP_STATE.robots[0];
  const nameElem = document.getElementById('dir-amr-name');
  const destElem = document.getElementById('dir-task-dest');
  const itemElem = document.getElementById('dir-item-name');
  const etaElem = document.getElementById('dir-eta');
  const speedElem = document.getElementById('dir-speed');
  const battElem = document.getElementById('dir-battery');

  if (!nameElem) return;

  nameElem.textContent = `${activeBot.name} (${activeBot.state.replace(/_/g, ' ')})`;
  destElem.textContent = activeBot.task ? `Target: ${activeBot.task.location}` : 'Target: Inductive Charging Pad';
  itemElem.textContent = activeBot.task ? activeBot.task.item : 'No active payload';
  etaElem.textContent = `${Math.max(5, Math.floor(30 + Math.sin(Date.now() / 1000) * 10))}s`;
  speedElem.textContent = `${activeBot.speed.toFixed(2)} m/s`;
  battElem.textContent = `${Math.round(activeBot.battery)}%`;
}

// Hover over AMR detection
function setupCanvasMouseListeners(canvasElem, isFullMap = false) {
  if (!canvasElem) return;
  const tooltip = document.getElementById('robot-hover-card');

  canvasElem.addEventListener('mousemove', (e) => {
    const rect = canvasElem.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    let foundBot = null;
    for (const bot of APP_STATE.robots) {
      const p = projectWorld(bot.x, bot.y, 0, rect.width, rect.height, APP_STATE.viewMode);
      const dist = Math.hypot(sx - p.x, sy - p.y);
      if (dist < 18) {
        foundBot = bot;
        break;
      }
    }

    if (foundBot && tooltip) {
      tooltip.style.display = 'flex';
      tooltip.style.left = `${sx + 14}px`;
      tooltip.style.top = `${sy - 20}px`;
      tooltip.innerHTML = `
        <div style="font-weight: 800; font-size: 12px; color: #0f172a;">${foundBot.name}</div>
        <div style="color: #64748b;">Battery: <strong style="color: #10b981;">${Math.round(foundBot.battery)}%</strong></div>
        <div style="color: #64748b;">Speed: <strong>${foundBot.speed} m/s</strong></div>
        <div style="color: #64748b;">Task: <strong>${foundBot.task ? foundBot.task.location : 'Docked'}</strong></div>
        <div style="color: #64748b;">Coords: <code>X:${foundBot.x.toFixed(1)}m, Y:${foundBot.y.toFixed(1)}m</code></div>
      `;
    } else if (tooltip) {
      tooltip.style.display = 'none';
    }

    // Update coordinate pill
    const coordPill = document.getElementById('fullmap-coords');
    if (coordPill && isFullMap) {
      const scale = Math.min(rect.width / 51, rect.height / 66);
      const wx = (sx - rect.width / 2) / scale;
      const wy = (rect.height / 2 - sy) / scale;
      coordPill.textContent = `X: ${wx.toFixed(2)}m | Y: ${wy.toFixed(2)}m`;
    }
  });

  // Click on robot -> Open Inspector Drawer/Modal
  canvasElem.addEventListener('click', (e) => {
    const rect = canvasElem.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    for (const bot of APP_STATE.robots) {
      const p = projectWorld(bot.x, bot.y, 0, rect.width, rect.height, APP_STATE.viewMode);
      if (Math.hypot(sx - p.x, sy - p.y) < 18) {
        APP_STATE.selectedRobotId = bot.id;
        openRobotInspectorModal(bot);
        break;
      }
    }
  });
}

// --- ROBOT INSPECTION MODAL ---
function openRobotInspectorModal(bot) {
  const modal = document.getElementById('robot-inspector-modal');
  if (!modal) return;

  document.getElementById('modal-robot-title').textContent = `${bot.name} Activity & Telemetry Inspector`;
  document.getElementById('insp-namespace').textContent = bot.namespace;
  document.getElementById('insp-coords').textContent = `X: ${bot.x.toFixed(2)}m, Y: ${bot.y.toFixed(2)}m (Heading: ${(bot.theta * 180 / Math.PI).toFixed(0)}°)`;
  document.getElementById('insp-speed').textContent = `${bot.speed.toFixed(2)} m/s`;
  document.getElementById('insp-battery').textContent = `${Math.round(bot.battery)}% (${bot.isCharging ? 'Charging' : 'Discharging'})`;
  document.getElementById('insp-task').textContent = bot.task ? `${bot.task.orderId} (${bot.task.location})` : 'Idle / Docked';
  
  // Filter logs for this specific robot
  const robotLogs = APP_STATE.logs.filter(l => l.robot === bot.name);
  const streamElem = document.getElementById('insp-logs-stream');
  streamElem.innerHTML = robotLogs.map(l => `
    <div style="background: #fff; padding: 6px 8px; border-radius: 4px; border: 1px solid #e2e8f0;">
      <div style="display: flex; justify-content: space-between; color: #64748b; font-size: 10px;">
        <span>[${l.time}] <strong>${l.category}</strong></span>
        <span style="color: #10b981; font-weight: 700;">${l.status}</span>
      </div>
      <div style="color: #0f172a; margin-top: 2px;">${l.description}</div>
    </div>
  `).join('');

  modal.style.display = 'flex';
}

document.getElementById('btn-close-inspector')?.addEventListener('click', () => {
  document.getElementById('robot-inspector-modal').style.display = 'none';
});

// --- RENDER TASK MANAGEMENT TABLE (REFERENCE 1 STYLE) ---
function renderTasksTable() {
  const tbody = document.getElementById('tasks-table-body');
  if (!tbody) return;

  const search = document.getElementById('task-search-input')?.value.toLowerCase() || '';
  const filterStatus = document.getElementById('task-filter-status')?.value || 'ALL';
  const filterPriority = document.getElementById('task-filter-priority')?.value || 'ALL';

  const filtered = APP_STATE.tasks.filter(t => {
    const matchesSearch = t.orderId.toLowerCase().includes(search) || t.location.toLowerCase().includes(search) || t.assignedAmr.toLowerCase().includes(search);
    const matchesStatus = filterStatus === 'ALL' || t.status === filterStatus;
    const matchesPriority = filterPriority === 'ALL' || t.priority === filterPriority;
    return matchesSearch && matchesStatus && matchesPriority;
  });

  tbody.innerHTML = filtered.map(t => {
    const statusClass = t.status === 'Started' || t.status === 'Done' ? 'tag-done' : t.status === 'En Route' ? 'tag-enroute' : 'tag-pending';
    const delayClass = t.delay.startsWith('-') ? 'delay-negative' : t.delay === '00:00' ? '' : 'delay-positive';

    return `
      <tr>
        <td><span class="table-link">${t.orderId}</span></td>
        <td><strong>${t.location}</strong></td>
        <td>${t.taskType}</td>
        <td><code>${t.assignedAmr}</code></td>
        <td>${t.sta}</td>
        <td>${t.ata}</td>
        <td>${t.std}</td>
        <td>${t.atd}</td>
        <td><strong>${t.demand}</strong></td>
        <td class="${delayClass}">${t.delay}</td>
        <td><span style="font-weight: 700; color: ${t.priority.includes('P1') ? '#dc2626' : '#2563eb'}">${t.priority}</span></td>
        <td><span class="status-tag ${statusClass}">${t.status}</span></td>
        <td>
          <button class="btn-outline btn-sm" onclick="moveTaskToTop(${t.id})" title="Move to Top (Highest Urgency)">⬆ Priority</button>
        </td>
      </tr>
    `;
  }).join('');
}

window.moveTaskToTop = function(id) {
  const idx = APP_STATE.tasks.findIndex(t => t.id === id);
  if (idx > -1) {
    const item = APP_STATE.tasks.splice(idx, 1)[0];
    item.priority = 'Urgent (P1)';
    APP_STATE.tasks.unshift(item);
    renderTasksTable();
    addSystemLog('SUPERVISOR', 'TASK_ALLOCATION', `Task ${item.orderId} prioritized to top of auction queue by operator.`, 'Success');
  }
};

// --- RENDER LOGS TABLE (HUMAN READABLE ENGLISH) ---
function renderLogsTable() {
  const tbody = document.getElementById('logs-table-body');
  if (!tbody) return;

  const search = document.getElementById('log-search-input')?.value.toLowerCase() || '';
  const robotFilter = document.getElementById('log-filter-robot')?.value || 'ALL';
  const catFilter = document.getElementById('log-filter-category')?.value || 'ALL';

  const filtered = APP_STATE.logs.filter(l => {
    const matchesSearch = l.description.toLowerCase().includes(search) || l.robot.toLowerCase().includes(search);
    const matchesRobot = robotFilter === 'ALL' || l.robot.toLowerCase() === robotFilter.toLowerCase();
    const matchesCat = catFilter === 'ALL' || l.category === catFilter;
    return matchesSearch && matchesRobot && matchesCat;
  });

  tbody.innerHTML = filtered.map(l => `
    <tr>
      <td><span style="font-family: var(--font-mono); color: #64748b; font-size: 11px;">${l.time}</span></td>
      <td><strong>${l.robot}</strong></td>
      <td><span class="step-badge">${l.category}</span></td>
      <td>${l.description}</td>
      <td><span class="status-tag ${l.status === 'Emergency' ? 'tag-pending' : 'tag-done'}">${l.status}</span></td>
    </tr>
  `).join('');
}

function renderRecentLogsDashboard() {
  const container = document.getElementById('dash-recent-logs');
  if (!container) return;

  container.innerHTML = APP_STATE.logs.slice(0, 4).map(l => `
    <div class="event-snippet">
      <span class="event-time">[${l.time}]</span>
      <span class="event-text"><strong>${l.robot}:</strong> ${l.description}</span>
    </div>
  `).join('');
}

// --- RENDER SIDEBAR AMR CARDS ---
function renderSidebarAmrCards() {
  const container = document.getElementById('amr-cards-list');
  if (!container) return;

  container.innerHTML = APP_STATE.robots.map(bot => {
    const isSelected = bot.id === APP_STATE.selectedRobotId;
    return `
      <div class="amr-card-item ${isSelected ? 'selected' : ''}" onclick="selectAmr('${bot.id}')">
        <div class="amr-card-head">
          <span class="amr-name-tag">
            <span class="amr-color-dot" style="background: ${bot.color};"></span>
            ${bot.name}
          </span>
          <span class="amr-battery-val" style="color: ${bot.battery > 50 ? '#10b981' : '#f59e0b'}">
            ${bot.isCharging ? '⚡ ' : ''}${Math.round(bot.battery)}%
          </span>
        </div>
        <div class="amr-card-body">
          <span>State: <strong>${bot.state.replace(/_/g, ' ')}</strong></span>
          <span>Speed: <strong>${bot.speed} m/s</strong></span>
        </div>
      </div>
    `;
  }).join('');
}

window.selectAmr = function(id) {
  APP_STATE.selectedRobotId = id;
  renderSidebarAmrCards();
  updateUberDirectionCard();
};

// --- RENDER TRAFFIC HEATMAP (REFERENCE 3 STYLE) ---
function renderStaticHeatmap() {
  if (!heatCtx || !heatmapCanvas) return;
  const rect = heatmapCanvas.getBoundingClientRect();
  heatCtx.clearRect(0, 0, rect.width, rect.height);

  // Background warehouse floor
  heatCtx.fillStyle = '#f8fafc';
  heatCtx.fillRect(0, 0, rect.width, rect.height);

  // Draw Shelves in Heatmap
  for (const s of WAREHOUSE_CONFIG.shelves) {
    const p = projectWorld(s.x, s.y, 0, rect.width, rect.height, '2D');
    heatCtx.fillStyle = '#e2e8f0';
    heatCtx.fillRect(p.x - 12, p.y - 4, 24, 8);
  }

  // Draw Heatmap Blob Clusters (Corridor intersections)
  const hotPoints = [
    { x: -9.0, y: -10.0, intensity: 0.9 },
    { x: 9.0, y: -10.0, intensity: 0.8 },
    { x: -9.0, y: 10.0, intensity: 0.7 },
    { x: 9.0, y: 10.0, intensity: 0.85 },
    { x: 0.0, y: -28.5, intensity: 0.95 },
    { x: -15.67, y: 0.0, intensity: 0.5 },
    { x: 15.67, y: 0.0, intensity: 0.6 }
  ];

  for (const hp of hotPoints) {
    const p = projectWorld(hp.x, hp.y, 0, rect.width, rect.height, '2D');
    const rad = 45 * hp.intensity;
    const grad = heatCtx.createRadialGradient(p.x, p.y, 4, p.x, p.y, rad);
    grad.addColorStop(0, 'rgba(239, 68, 68, 0.6)');
    grad.addColorStop(0.5, 'rgba(251, 146, 60, 0.4)');
    grad.addColorStop(1, 'rgba(254, 240, 138, 0)');

    heatCtx.fillStyle = grad;
    heatCtx.beginPath();
    heatCtx.arc(p.x, p.y, rad, 0, Math.PI * 2);
    heatCtx.fill();
  }
}

// --- CREATE TASK MODAL ACTIONS ---
function setupTaskModal() {
  const modal = document.getElementById('create-task-modal');
  const btnOpen = document.getElementById('btn-create-task-modal');
  const btnClose = document.getElementById('btn-close-create-task');
  const btnCancel = document.getElementById('btn-cancel-create-task');
  const btnSubmit = document.getElementById('btn-submit-create-task');

  if (btnOpen) btnOpen.addEventListener('click', () => modal.style.display = 'flex');
  if (btnClose) btnClose.addEventListener('click', () => modal.style.display = 'none');
  if (btnCancel) btnCancel.addEventListener('click', () => modal.style.display = 'none');

  if (btnSubmit) {
    btnSubmit.addEventListener('click', () => {
      const location = document.getElementById('newtask-location').value;
      const taskType = document.getElementById('newtask-type').value;
      const item = document.getElementById('newtask-item').value;
      const demand = parseInt(document.getElementById('newtask-demand').value) || 5;
      const priority = document.getElementById('newtask-priority').value;

      const newId = APP_STATE.tasks.length;
      const newOrder = {
        id: newId,
        orderId: `ORD-${107 + newId}`,
        location,
        taskType: taskType.charAt(0).toUpperCase() + taskType.slice(1),
        assignedAmr: 'Auctioning...',
        sta: '10:45',
        ata: '-',
        std: '10:50',
        atd: '-',
        demand,
        delay: '00:00',
        status: 'Started',
        priority
      };

      if (priority.includes('P1')) {
        APP_STATE.tasks.unshift(newOrder);
      } else {
        APP_STATE.tasks.push(newOrder);
      }

      renderTasksTable();
      addSystemLog('SUPERVISOR', 'TASK_ALLOCATION', `New order ${newOrder.orderId} dispatched to ${location} (${priority}).`, 'Success');
      modal.style.display = 'none';
    });
  }

  // Task search and filter event listeners
  document.getElementById('task-search-input')?.addEventListener('input', renderTasksTable);
  document.getElementById('task-filter-status')?.addEventListener('change', renderTasksTable);
  document.getElementById('task-filter-priority')?.addEventListener('change', renderTasksTable);

  // Log filter event listeners
  document.getElementById('log-search-input')?.addEventListener('input', renderLogsTable);
  document.getElementById('log-filter-robot')?.addEventListener('change', renderLogsTable);
  document.getElementById('log-filter-category')?.addEventListener('change', renderLogsTable);
}

// --- BOOTSTRAP INITIALIZATION ---
window.addEventListener('DOMContentLoaded', () => {
  initNavigation();
  resizeActiveCanvases();
  setupCanvasMouseListeners(dashCanvas, false);
  setupCanvasMouseListeners(fullCanvas, true);
  setupTaskModal();

  renderTasksTable();
  renderLogsTable();
  renderRecentLogsDashboard();
  renderSidebarAmrCards();

  requestAnimationFrame(animLoop);
});
