/**
 * ============================================================================
 * CUADERNO DE LABORATORIO BALÍSTICO 3D (Three.js r128)
 * Entorno Diurno Natural + Misil de Pruebas en Livery Naranja/Crema + RK4
 * ============================================================================
 */

(function () {
  'use strict';

  const state = {
    v0: 65.0,
    angleDeg: 45.0,
    azimuthDeg: 0.0,
    y0: 10.0,
    mass: 45.0,
    gravity: 9.81,
    useDrag: false,
    cd: 0.25,
    area: 0.08,
    rho: 1.225,
    windX: 0.0,
    targetDist: 440.0,
    missileScale: 2.6,

    showVecV: true,
    showVecComp: true,
    showVecA: true,
    showIdealCurve: true,
    autoLoop: true,
    simSpeed: 1.0,
    cameraMode: 'follow',
    chartMode: 'yx',
    soundEnabled: true,

    isPlaying: true,
    isPaused: false,
    currentTime: 0.0,
    loopCooldown: 0.0,
    trajectory: [],
    idealTrajectory: [],
    idealMetrics: { hMax: 0, range: 0, tFlight: 0, impactSpeed: 0 },
    metrics: { hMax: 0, range: 0, tFlight: 0, impactSpeed: 0 }
  };

  // --- Motor de Sonido Sintetizado ---
  class SoundEngine {
    constructor() {
      this.ctx = null;
    }

    init() {
      if (!this.ctx) {
        const AudioCtx = window.AudioContext || window.webkitAudioContext;
        if (AudioCtx) this.ctx = new AudioCtx();
      }
      if (this.ctx && this.ctx.state === 'suspended') {
        this.ctx.resume();
      }
    }

    playLaunch() {
      if (!state.soundEnabled) return;
      try {
        this.init();
        if (!this.ctx) return;
        const now = this.ctx.currentTime;
        const bufferSize = this.ctx.sampleRate * 0.85;
        const buffer = this.ctx.createBuffer(1, bufferSize, this.ctx.sampleRate);
        const data = buffer.getChannelData(0);
        for (let i = 0; i < bufferSize; i++) {
          data[i] = (Math.random() * 2 - 1) * Math.exp(-i / (this.ctx.sampleRate * 0.4));
        }
        const noise = this.ctx.createBufferSource();
        noise.buffer = buffer;

        const filter = this.ctx.createBiquadFilter();
        filter.type = 'bandpass';
        filter.frequency.setValueAtTime(280, now);
        filter.frequency.exponentialRampToValueAtTime(95, now + 0.8);

        const gain = this.ctx.createGain();
        gain.gain.setValueAtTime(0.2, now);
        gain.gain.exponentialRampToValueAtTime(0.01, now + 0.82);

        noise.connect(filter);
        filter.connect(gain);
        gain.connect(this.ctx.destination);
        noise.start(now);
      } catch (e) {}
    }

    playExplosion(isBullseye) {
      if (!state.soundEnabled) return;
      try {
        this.init();
        if (!this.ctx) return;
        const now = this.ctx.currentTime;
        const osc = this.ctx.createOscillator();
        const gain = this.ctx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(isBullseye ? 130 : 88, now);
        osc.frequency.exponentialRampToValueAtTime(22, now + 0.55);

        gain.gain.setValueAtTime(0.24, now);
        gain.gain.exponentialRampToValueAtTime(0.005, now + 0.58);

        osc.connect(gain);
        gain.connect(this.ctx.destination);
        osc.start(now);
        osc.stop(now + 0.6);
      } catch (e) {}
    }
  }

  const sound = new SoundEngine();

  // --- Integrador Numérico Runge-Kutta de 4to Orden (RK4) ---
  function computeTrajectoryPoints(useAerodynamicDrag) {
    const dt = 0.016;
    const theta = (state.angleDeg * Math.PI) / 180;
    const phi = (state.azimuthDeg * Math.PI) / 180;

    const v0Horiz = state.v0 * Math.cos(theta);
    let vx = v0Horiz * Math.cos(phi);
    let vy = state.v0 * Math.sin(theta);
    let vz = v0Horiz * Math.sin(phi);

    let x = 0;
    let y = state.y0;
    let z = 0;
    let t = 0;

    const points = [];
    let hMax = y;

    const kDrag = useAerodynamicDrag
      ? (0.5 * state.rho * state.cd * state.area) / state.mass
      : 0;
    const windX = useAerodynamicDrag ? state.windX : 0;

    function accel(vxCurr, vyCurr, vzCurr) {
      if (!useAerodynamicDrag) {
        return { ax: 0, ay: -state.gravity, az: 0 };
      }
      const rvx = vxCurr - windX;
      const rvy = vyCurr;
      const rvz = vzCurr;
      const vRel = Math.sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
      return {
        ax: -kDrag * vRel * rvx,
        ay: -state.gravity - kDrag * vRel * rvy,
        az: -kDrag * vRel * rvz
      };
    }

    const a0 = accel(vx, vy, vz);
    points.push({ t, x, y, z, vx, vy, vz, ax: a0.ax, ay: a0.ay, az: a0.az });

    for (let step = 0; step < 25000; step++) {
      const k1v = accel(vx, vy, vz);
      const k1x = { dx: vx, dy: vy, dz: vz };

      const vx2 = vx + 0.5 * dt * k1v.ax;
      const vy2 = vy + 0.5 * dt * k1v.ay;
      const vz2 = vz + 0.5 * dt * k1v.az;
      const k2v = accel(vx2, vy2, vz2);
      const k2x = { dx: vx2, dy: vy2, dz: vz2 };

      const vx3 = vx + 0.5 * dt * k2v.ax;
      const vy3 = vy + 0.5 * dt * k2v.ay;
      const vz3 = vz + 0.5 * dt * k2v.az;
      const k3v = accel(vx3, vy3, vz3);
      const k3x = { dx: vx3, dy: vy3, dz: vz3 };

      const vx4 = vx + dt * k3v.ax;
      const vy4 = vy + dt * k3v.ay;
      const vz4 = vz + dt * k3v.az;
      const k4v = accel(vx4, vy4, vz4);
      const k4x = { dx: vx4, dy: vy4, dz: vz4 };

      const nextX = x + (dt / 6) * (k1x.dx + 2 * k2x.dx + 2 * k3x.dx + k4x.dx);
      const nextY = y + (dt / 6) * (k1x.dy + 2 * k2x.dy + 2 * k3x.dy + k4x.dy);
      const nextZ = z + (dt / 6) * (k1x.dz + 2 * k2x.dz + 2 * k3x.dz + k4x.dz);

      const nextVx = vx + (dt / 6) * (k1v.ax + 2 * k2v.ax + 2 * k3v.ax + k4v.ax);
      const nextVy = vy + (dt / 6) * (k1v.ay + 2 * k2v.ay + 2 * k3v.ay + k4v.ay);
      const nextVz = vz + (dt / 6) * (k1v.az + 2 * k2v.az + 2 * k3v.az + k4v.az);
      const nextT = t + dt;

      if (nextY <= 0 && nextVy < 0) {
        const frac = y / (y - nextY);
        x = x + frac * (nextX - x);
        y = 0;
        z = z + frac * (nextZ - z);
        vx = vx + frac * (nextVx - vx);
        vy = vy + frac * (nextVy - vy);
        vz = vz + frac * (nextVz - vz);
        t = t + frac * dt;
        const af = accel(vx, vy, vz);
        points.push({ t, x, y, z, vx, vy, vz, ax: af.ax, ay: af.ay, az: af.az });
        break;
      }

      x = nextX;
      y = nextY;
      z = nextZ;
      vx = nextVx;
      vy = nextVy;
      vz = nextVz;
      t = nextT;

      if (y > hMax) hMax = y;
      const aCurr = accel(vx, vy, vz);
      points.push({ t, x, y, z, vx, vy, vz, ax: aCurr.ax, ay: aCurr.ay, az: aCurr.az });
    }

    const last = points[points.length - 1];
    const range = Math.sqrt(last.x * last.x + last.z * last.z);
    const impactSpeed = Math.sqrt(last.vx * last.vx + last.vy * last.vy + last.vz * last.vz);

    return {
      points,
      metrics: { hMax, range, tFlight: last.t, impactSpeed }
    };
  }

  function sampleTrajectoryAtTime(t) {
    const pts = state.trajectory;
    if (!pts || pts.length === 0) {
      return { t: 0, x: 0, y: state.y0, z: 0, vx: state.v0, vy: 0, vz: 0, ax: 0, ay: -state.gravity, az: 0 };
    }
    if (t <= 0) return pts[0];
    const last = pts[pts.length - 1];
    if (t >= last.t) return last;

    const dt = 0.016;
    const idx = Math.min(pts.length - 2, Math.max(0, Math.floor(t / dt)));
    const p1 = pts[idx];
    const p2 = pts[idx + 1];
    const span = p2.t - p1.t || 1e-6;
    const alpha = Math.min(1, Math.max(0, (t - p1.t) / span));

    return {
      t,
      x: p1.x + alpha * (p2.x - p1.x),
      y: p1.y + alpha * (p2.y - p1.y),
      z: p1.z + alpha * (p2.z - p1.z),
      vx: p1.vx + alpha * (p2.vx - p1.vx),
      vy: p1.vy + alpha * (p2.vy - p1.vy),
      vz: p1.vz + alpha * (p2.vz - p1.vz),
      ax: p1.ax + alpha * (p2.ax - p1.ax),
      ay: p1.ay + alpha * (p2.ay - p1.ay),
      az: p1.az + alpha * (p2.az - p1.az)
    };
  }

  // --- Escena 3D Diurna Natural (Campo de Pruebas Universitario) ---
  const container = document.getElementById('canvas-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xd6ecfa);
  scene.fog = new THREE.FogExp2(0xd6ecfa, 0.00038);

  const camera = new THREE.PerspectiveCamera(46, window.innerWidth / window.innerHeight, 0.5, 5000);

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  const orbit = {
    target: new THREE.Vector3(180, 45, 0),
    spherical: {
      radius: 260,
      phi: Math.PI / 2.3,
      theta: -Math.PI / 7
    },
    followTheta: 0.42,
    followPhi: 1.24,
    followDist: 56,
    sideDist: 340,
    sidePanX: 0,
    sidePanY: 0,
    isDragging: false,
    isPanning: false,
    prevX: 0,
    prevY: 0,
    pinchStartDist: 0,
    twoFingerMidX: 0,
    twoFingerMidY: 0
  };

  function getActiveZoomDistance() {
    if (state.cameraMode === 'follow') return orbit.followDist;
    if (state.cameraMode === 'side') return orbit.sideDist;
    return orbit.spherical.radius;
  }

  function syncZoomUI() {
    const d = getActiveZoomDistance();
    const slider = document.getElementById('slider-zoom');
    const readout = document.getElementById('val-zoom-readout');
    if (slider) slider.value = Math.round(Math.max(15, Math.min(750, d)));
    if (readout) {
      const pct = Math.round((120 / Math.max(15, d)) * 100);
      readout.textContent = Math.min(600, Math.max(15, pct)) + '%';
    }
  }

  function setActiveZoomDistance(newDist) {
    const clamped = Math.max(14, Math.min(1600, newDist));
    if (state.cameraMode === 'follow') {
      orbit.followDist = clamped;
    } else if (state.cameraMode === 'side') {
      orbit.sideDist = clamped;
    } else {
      orbit.spherical.radius = clamped;
      updateOrbitCamera();
    }
    syncZoomUI();
    applyMissileState(sampleTrajectoryAtTime(state.currentTime));
  }

  function zoomByFactor(factor) {
    setActiveZoomDistance(getActiveZoomDistance() * factor);
  }

  function updateOrbitCamera() {
    if (state.cameraMode !== 'orbit') return;
    const sinPhi = Math.sin(orbit.spherical.phi);
    const x = orbit.target.x + orbit.spherical.radius * sinPhi * Math.sin(orbit.spherical.theta);
    const y = Math.max(3, orbit.target.y + orbit.spherical.radius * Math.cos(orbit.spherical.phi));
    const z = orbit.target.z + orbit.spherical.radius * sinPhi * Math.cos(orbit.spherical.theta);
    camera.position.set(x, y, z);
    camera.lookAt(orbit.target);
  }

  renderer.domElement.addEventListener('pointerdown', (e) => {
    orbit.isDragging = true;
    orbit.isPanning = e.button === 2 || e.shiftKey;
    orbit.prevX = e.clientX;
    orbit.prevY = e.clientY;
  });

  window.addEventListener('pointermove', (e) => {
    if (!orbit.isDragging) return;
    const dx = e.clientX - orbit.prevX;
    const dy = e.clientY - orbit.prevY;
    orbit.prevX = e.clientX;
    orbit.prevY = e.clientY;

    if (state.cameraMode === 'follow') {
      orbit.followTheta -= dx * 0.0065;
      orbit.followPhi = Math.max(0.2, Math.min(Math.PI / 2 + 0.2, orbit.followPhi - dy * 0.0055));
      return;
    }

    if (state.cameraMode === 'side') {
      const panScale = orbit.sideDist * 0.0016;
      orbit.sidePanX -= dx * panScale;
      orbit.sidePanY = Math.max(-40, orbit.sidePanY + dy * panScale);
      return;
    }

    if (state.cameraMode !== 'orbit') return;
    if (orbit.isPanning) {
      const panScale = orbit.spherical.radius * 0.0015;
      orbit.target.x -= dx * panScale;
      orbit.target.y = Math.max(0, orbit.target.y + dy * panScale);
    } else {
      orbit.spherical.theta -= dx * 0.0055;
      orbit.spherical.phi = Math.max(0.12, Math.min(Math.PI / 2 - 0.02, orbit.spherical.phi - dy * 0.0055));
    }
    updateOrbitCamera();
  });

  window.addEventListener('pointerup', () => {
    orbit.isDragging = false;
  });

  renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

  // Soporte completo de Pinch-to-Zoom (2 dedos) y desplazamiento en pantallas táctiles móviles
  renderer.domElement.addEventListener('touchstart', (e) => {
    if (e.touches.length === 2) {
      orbit.isDragging = false;
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      orbit.pinchStartDist = Math.hypot(dx, dy);
      orbit.twoFingerMidX = (e.touches[0].clientX + e.touches[1].clientX) * 0.5;
      orbit.twoFingerMidY = (e.touches[0].clientY + e.touches[1].clientY) * 0.5;
    }
  }, { passive: false });

  renderer.domElement.addEventListener('touchmove', (e) => {
    if (e.touches.length === 2) {
      e.preventDefault();
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const dist = Math.hypot(dx, dy);
      if (orbit.pinchStartDist > 10 && dist > 10) {
        const ratio = orbit.pinchStartDist / dist;
        zoomByFactor(ratio);
        orbit.pinchStartDist = dist;
      }
      const midX = (e.touches[0].clientX + e.touches[1].clientX) * 0.5;
      const midY = (e.touches[0].clientY + e.touches[1].clientY) * 0.5;
      const panDx = midX - orbit.twoFingerMidX;
      const panDy = midY - orbit.twoFingerMidY;
      orbit.twoFingerMidX = midX;
      orbit.twoFingerMidY = midY;
      if (state.cameraMode === 'orbit') {
        const scale = orbit.spherical.radius * 0.0015;
        orbit.target.x -= panDx * scale;
        orbit.target.y = Math.max(0, orbit.target.y + panDy * scale);
        updateOrbitCamera();
      } else if (state.cameraMode === 'side') {
        const scale = orbit.sideDist * 0.0015;
        orbit.sidePanX -= panDx * scale;
        orbit.sidePanY = Math.max(-40, orbit.sidePanY + panDy * scale);
      }
    }
  }, { passive: false });

  renderer.domElement.addEventListener('wheel', (e) => {
    e.preventDefault();
    zoomByFactor(1 + e.deltaY * 0.0011);
  }, { passive: false });

  // --- Iluminación Solar Cálida Natural ---
  const hemiLight = new THREE.HemisphereLight(0xffffff, 0xc7bea9, 0.95);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xfffbeb, 1.25);
  dirLight.position.set(220, 480, 260);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  const dCam = 550;
  dirLight.shadow.camera.left = -dCam;
  dirLight.shadow.camera.right = dCam;
  dirLight.shadow.camera.top = dCam;
  dirLight.shadow.camera.bottom = -dCam;
  scene.add(dirLight);

  // --- Terreno Natural de Campo de Pruebas + Nubes 3D Suaves ---
  const groundGeo = new THREE.PlaneGeometry(4000, 4000);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0xd8d0c1,
    roughness: 0.92,
    metalness: 0.04
  });
  const groundMesh = new THREE.Mesh(groundGeo, groundMat);
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);

  // Cuadrícula topográfica suave color tierra/oliva
  const gridHelper = new THREE.GridHelper(2400, 80, 0x8c7f6c, 0xc2b8a3);
  gridHelper.position.set(500, 0.06, 0);
  scene.add(gridHelper);

  // Colinas lejanas decorativas en el horizonte para dar calidez orgánica
  const hillMat = new THREE.MeshStandardMaterial({ color: 0xb7c4b1, roughness: 0.95 });
  [
    { x: 300, z: -480, r: 240, h: 95 },
    { x: 750, z: -520, r: 320, h: 130 },
    { x: -150, z: -450, r: 210, h: 80 },
    { x: 550, z: 520, r: 290, h: 105 }
  ].forEach((h) => {
    const hill = new THREE.Mesh(new THREE.ConeGeometry(h.r, h.h, 24), hillMat);
    hill.position.set(h.x, h.h / 2 - 5, h.z);
    scene.add(hill);
  });

  // Nubes 3D blancas suaves en el cielo
  const cloudMat = new THREE.MeshLambertMaterial({ color: 0xffffff, transparent: true, opacity: 0.85 });
  for (let c = 0; c < 14; c++) {
    const cloudGroup = new THREE.Group();
    const nBlobs = 3 + (c % 3);
    for (let b = 0; b < nBlobs; b++) {
      const blob = new THREE.Mesh(new THREE.SphereGeometry(14 + (b % 2) * 6, 12, 12), cloudMat);
      blob.position.set(b * 12, (b % 2) * 3, (b % 3) * 4);
      blob.scale.set(1.4, 0.65, 1.0);
      cloudGroup.add(blob);
    }
    cloudGroup.position.set(-100 + c * 115, 165 + (c % 4) * 35, (c % 2 === 0 ? -1 : 1) * (140 + (c * 23) % 180));
    scene.add(cloudGroup);
  }

  // Carteles de madera/marfil de distancia cada 100 m
  function createTextSprite(text) {
    const c = document.createElement('canvas');
    c.width = 256;
    c.height = 88;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#fbf8f3';
    ctx.strokeStyle = '#b84a27';
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.roundRect(10, 10, 236, 68, 12);
    ctx.fill();
    ctx.stroke();

    ctx.font = 'bold 32px Georgia, serif';
    ctx.fillStyle = '#231f1c';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 44);

    const tex = new THREE.CanvasTexture(c);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
    sprite.scale.set(16, 5.5, 1);
    return sprite;
  }

  for (let dist = 100; dist <= 1100; dist += 100) {
    const marker = createTextSprite(dist + ' m');
    marker.position.set(dist, 4.2, 26);
    scene.add(marker);
  }

  // --- Plataforma de Lanzamiento de Laboratorio (Verde Oliva / Bronce) ---
  const launcherGroup = new THREE.Group();
  scene.add(launcherGroup);

  const pedestalMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(3.8, 5.2, 1, 24),
    new THREE.MeshStandardMaterial({ color: 0x5c5349, metalness: 0.4, roughness: 0.6 })
  );
  pedestalMesh.castShadow = true;
  launcherGroup.add(pedestalMesh);

  const turretPivot = new THREE.Group();
  launcherGroup.add(turretPivot);

  const baseDome = new THREE.Mesh(
    new THREE.SphereGeometry(3.4, 24, 16, 0, Math.PI * 2, 0, Math.PI / 2),
    new THREE.MeshStandardMaterial({ color: 0x235347, metalness: 0.45, roughness: 0.45 })
  );
  turretPivot.add(baseDome);

  const railArm = new THREE.Mesh(
    new THREE.BoxGeometry(12, 0.85, 1.8),
    new THREE.MeshStandardMaterial({ color: 0xb84a27, metalness: 0.35, roughness: 0.45 })
  );
  railArm.position.set(4.0, 1.5, 0);
  turretPivot.add(railArm);

  // --- COHETE / MISIL BALÍSTICO DE INVESTIGACIÓN EN 3D (Naranja Internacional + Marfil + Carbón) ---
  function createHeroMissile3D() {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.rotation.x = Math.PI / 2;
    root.add(model);

    const fusCanvas = document.createElement('canvas');
    fusCanvas.width = 1024;
    fusCanvas.height = 1024;
    const fctx = fusCanvas.getContext('2d');

    // Cuerpo marfil cálido de cohete de sondeo universitario
    fctx.fillStyle = '#fbf8f3';
    fctx.fillRect(0, 0, 1024, 1024);

    // Patrón ajedrezado clásico de telemetría óptica (Blanco/Carbón y Terracota)
    const sqSize = 64;
    for (let i = 0; i < 16; i++) {
      fctx.fillStyle = i % 2 === 0 ? '#231f1c' : '#fbf8f3';
      fctx.fillRect(i * sqSize, 110, sqSize, 68);
      fctx.fillStyle = i % 2 === 1 ? '#b84a27' : '#fbf8f3';
      fctx.fillRect(i * sqSize, 178, sqSize, 46);
    }

    // Franjas de alta visibilidad Naranja Internacional y Verde Bosque
    fctx.fillStyle = '#235347';
    fctx.fillRect(0, 730, 1024, 120);
    fctx.fillStyle = '#b84a27';
    fctx.fillRect(0, 470, 1024, 54);

    // Remaches
    fctx.fillStyle = '#786e60';
    for (let x = 32; x < 1024; x += 48) {
      fctx.beginPath();
      fctx.arc(x, 330, 6, 0, Math.PI * 2);
      fctx.arc(x, 660, 6, 0, Math.PI * 2);
      fctx.fill();
    }

    fctx.font = 'bold 56px Georgia, serif';
    fctx.fillStyle = '#231f1c';
    fctx.textAlign = 'center';
    fctx.fillText('SONDA BALÍSTICA 3D', 512, 420);
    fctx.fillStyle = '#235347';
    fctx.fillText('FÍSICA — RK4', 512, 610);

    const fuselageTex = new THREE.CanvasTexture(fusCanvas);

    const noseMat = new THREE.MeshStandardMaterial({
      color: 0xc84b28,
      metalness: 0.35,
      roughness: 0.28
    });
    const bodyMat = new THREE.MeshStandardMaterial({
      map: fuselageTex,
      metalness: 0.25,
      roughness: 0.35
    });
    const darkMetalMat = new THREE.MeshStandardMaterial({
      color: 0x231f1c,
      metalness: 0.65,
      roughness: 0.32
    });
    const finTrimMat = new THREE.MeshStandardMaterial({
      color: 0x235347,
      metalness: 0.4,
      roughness: 0.35
    });

    const bodyHeight = 9.5;
    const bodyRadius = 0.98;
    const bodyMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(bodyRadius, bodyRadius, bodyHeight, 36),
      bodyMat
    );
    bodyMesh.castShadow = true;
    model.add(bodyMesh);

    [-3.2, -0.5, 2.4, 4.5].forEach((ry) => {
      const ring = new THREE.Mesh(
        new THREE.TorusGeometry(bodyRadius + 0.035, 0.055, 14, 36),
        darkMetalMat
      );
      ring.rotation.x = Math.PI / 2;
      ring.position.y = ry;
      model.add(ring);
    });

    const noseHeight = 3.8;
    const noseMesh = new THREE.Mesh(
      new THREE.ConeGeometry(bodyRadius, noseHeight, 36),
      noseMat
    );
    noseMesh.position.y = bodyHeight / 2 + noseHeight / 2;
    noseMesh.castShadow = true;
    model.add(noseMesh);

    const pitotSpike = new THREE.Mesh(
      new THREE.CylinderGeometry(0.03, 0.09, 1.4, 16),
      new THREE.MeshStandardMaterial({ color: 0x231f1c, metalness: 0.85, roughness: 0.2 })
    );
    pitotSpike.position.y = bodyHeight / 2 + noseHeight + 0.65;
    model.add(pitotSpike);

    const finsGroup = new THREE.Group();
    model.add(finsGroup);

    const tailFinShape = new THREE.Shape();
    tailFinShape.moveTo(0, 0);
    tailFinShape.lineTo(2.6, -1.3);
    tailFinShape.lineTo(2.6, -3.1);
    tailFinShape.lineTo(0, -2.7);
    tailFinShape.closePath();

    const tailFinGeo = new THREE.ExtrudeGeometry(tailFinShape, {
      depth: 0.14,
      bevelEnabled: true,
      bevelSegments: 2,
      steps: 1,
      bevelSize: 0.04,
      bevelThickness: 0.04
    });
    tailFinGeo.center();

    for (let i = 0; i < 4; i++) {
      const pivot = new THREE.Group();
      pivot.rotation.y = (i * Math.PI) / 2;

      const fin = new THREE.Mesh(tailFinGeo, darkMetalMat);
      fin.position.set(bodyRadius + 1.15, -bodyHeight / 2 + 1.55, 0);
      fin.castShadow = true;
      pivot.add(fin);

      const tipLight = new THREE.Mesh(
        new THREE.SphereGeometry(0.14, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0xb84a27 })
      );
      tipLight.position.set(bodyRadius + 2.45, -bodyHeight / 2 + 0.6, 0);
      pivot.add(tipLight);

      finsGroup.add(pivot);
    }

    const canardShape = new THREE.Shape();
    canardShape.moveTo(0, 0);
    canardShape.lineTo(1.25, -0.65);
    canardShape.lineTo(1.25, -1.35);
    canardShape.lineTo(0, -1.15);
    canardShape.closePath();

    const canardGeo = new THREE.ExtrudeGeometry(canardShape, {
      depth: 0.1,
      bevelEnabled: false
    });
    canardGeo.center();

    for (let i = 0; i < 4; i++) {
      const cPivot = new THREE.Group();
      cPivot.rotation.y = (i * Math.PI) / 2;
      const canard = new THREE.Mesh(canardGeo, finTrimMat);
      canard.position.set(bodyRadius + 0.55, bodyHeight / 2 - 1.25, 0);
      cPivot.add(canard);
      finsGroup.add(cPivot);
    }

    const nozzleMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(bodyRadius * 0.62, bodyRadius * 0.92, 1.5, 28),
      darkMetalMat
    );
    nozzleMesh.position.y = -bodyHeight / 2 - 0.7;
    model.add(nozzleMesh);

    const flameGroup = new THREE.Group();
    flameGroup.position.y = -bodyHeight / 2 - 1.4;
    model.add(flameGroup);

    const outerFlame = new THREE.Mesh(
      new THREE.ConeGeometry(0.92, 5.8, 24),
      new THREE.MeshBasicMaterial({
        color: 0xea580c,
        transparent: true,
        opacity: 0.88
      })
    );
    outerFlame.rotation.x = Math.PI;
    outerFlame.position.y = -2.9;
    flameGroup.add(outerFlame);

    const midFlame = new THREE.Mesh(
      new THREE.ConeGeometry(0.62, 4.0, 20),
      new THREE.MeshBasicMaterial({
        color: 0xfacc15,
        transparent: true,
        opacity: 0.94
      })
    );
    midFlame.rotation.x = Math.PI;
    midFlame.position.y = -2.0;
    flameGroup.add(midFlame);

    const innerCore = new THREE.Mesh(
      new THREE.ConeGeometry(0.38, 2.4, 16),
      new THREE.MeshBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.98
      })
    );
    innerCore.rotation.x = Math.PI;
    innerCore.position.y = -1.2;
    flameGroup.add(innerCore);

    const thrusterLight = new THREE.PointLight(0xf97316, 2.4, 55);
    thrusterLight.position.set(0, -2.5, 0);
    flameGroup.add(thrusterLight);

    return {
      root,
      model,
      finsGroup,
      flameGroup,
      thrusterLight
    };
  }

  const missile = createHeroMissile3D();
  scene.add(missile.root);

  const groundShadowMarker = new THREE.Mesh(
    new THREE.RingGeometry(1.5, 3.8, 32),
    new THREE.MeshBasicMaterial({ color: 0x235347, side: THREE.DoubleSide, transparent: true, opacity: 0.65 })
  );
  groundShadowMarker.rotation.x = -Math.PI / 2;
  groundShadowMarker.position.y = 0.15;
  scene.add(groundShadowMarker);

  // --- Diana de Tiro en el Campo ---
  const targetGroup = new THREE.Group();
  scene.add(targetGroup);

  const ringColors = [0xb84a27, 0xfbf8f3, 0xb84a27, 0x235347];
  const ringRadii = [18, 12.5, 7, 2.5];
  ringRadii.forEach((r, idx) => {
    const disc = new THREE.Mesh(
      new THREE.CircleGeometry(r, 40),
      new THREE.MeshBasicMaterial({ color: ringColors[idx], side: THREE.DoubleSide })
    );
    disc.rotation.x = -Math.PI / 2;
    disc.position.y = 0.08 + idx * 0.02;
    targetGroup.add(disc);
  });

  const explosionGroup = new THREE.Group();
  scene.add(explosionGroup);

  const fireballMesh = new THREE.Mesh(
    new THREE.SphereGeometry(1, 24, 24),
    new THREE.MeshBasicMaterial({ color: 0xd9481f, transparent: true, opacity: 0 })
  );
  explosionGroup.add(fireballMesh);

  const shockwaveRing = new THREE.Mesh(
    new THREE.RingGeometry(1.2, 2.8, 40),
    new THREE.MeshBasicMaterial({ color: 0xb84a27, side: THREE.DoubleSide, transparent: true, opacity: 0 })
  );
  shockwaveRing.rotation.x = -Math.PI / 2;
  shockwaveRing.position.y = 0.4;
  explosionGroup.add(shockwaveRing);
  let shockwaveTimer = 0;

  // --- Vectores Físicos con Colores Académicos Contrastados ---
  const vecOrigin = new THREE.Vector3();
  const arrowV = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), vecOrigin, 20, 0x235347, 4.5, 2.4);
  const arrowVx = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), vecOrigin, 16, 0x0284c7, 3.6, 1.9);
  const arrowVy = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), vecOrigin, 16, 0xd97706, 3.6, 1.9);
  const arrowA = new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), vecOrigin, 14, 0xb84a27, 3.6, 1.9);

  scene.add(arrowV);
  scene.add(arrowVx);
  scene.add(arrowVy);
  scene.add(arrowA);

  let realCurveMesh = null;
  let idealCurveLine = null;

  function rebuildTrajectoryLines() {
    if (realCurveMesh) {
      scene.remove(realCurveMesh);
      realCurveMesh.geometry.dispose();
    }
    if (idealCurveLine) {
      scene.remove(idealCurveLine);
      idealCurveLine.geometry.dispose();
    }

    const idealPts = state.idealTrajectory.map((p) => new THREE.Vector3(p.x, p.y, p.z));
    const idealGeo = new THREE.BufferGeometry().setFromPoints(idealPts);
    const idealMat = new THREE.LineDashedMaterial({
      color: 0x5c5349,
      dashSize: 5,
      gapSize: 3.5,
      transparent: true,
      opacity: state.useDrag ? 0.75 : 0.4
    });
    idealCurveLine = new THREE.Line(idealGeo, idealMat);
    idealCurveLine.computeLineDistances();
    idealCurveLine.visible = state.showIdealCurve;
    scene.add(idealCurveLine);

    const realPts = state.trajectory.map((p) => new THREE.Vector3(p.x, p.y, p.z));
    if (realPts.length >= 2) {
      const curve = new THREE.CatmullRomCurve3(realPts);
      const tubeGeo = new THREE.TubeGeometry(curve, 120, 0.42, 8, false);
      const tubeMat = new THREE.MeshBasicMaterial({
        color: state.useDrag ? 0xb84a27 : 0x235347,
        transparent: true,
        opacity: 0.65
      });
      realCurveMesh = new THREE.Mesh(tubeGeo, tubeMat);
      scene.add(realCurveMesh);
    }
  }

  // --- Estela de Humo Visible en Cielo Diurno ---
  const MAX_SMOKE = 300;
  const smokeParticles = [];
  const smokeGeo = new THREE.SphereGeometry(1.0, 10, 10);

  for (let i = 0; i < MAX_SMOKE; i++) {
    const mat = new THREE.MeshBasicMaterial({
      color: 0x5c5349,
      transparent: true,
      opacity: 0
    });
    const mesh = new THREE.Mesh(smokeGeo, mat);
    mesh.visible = false;
    scene.add(mesh);
    smokeParticles.push({
      mesh,
      active: false,
      age: 0,
      maxAge: 1.5,
      vx: 0,
      vy: 0,
      vz: 0
    });
  }
  let smokeIdx = 0;

  function spawnSmokeParticle(pos, dir, sc) {
    const p = smokeParticles[smokeIdx];
    smokeIdx = (smokeIdx + 1) % MAX_SMOKE;
    p.active = true;
    p.age = 0;
    p.maxAge = 1.0 + Math.random() * 0.85;
    p.mesh.visible = true;
    const tailOffset = 6.2 * sc;
    p.mesh.position.set(
      pos.x - dir.x * tailOffset + (Math.random() - 0.5) * 0.9,
      Math.max(0.3, pos.y - dir.y * tailOffset + (Math.random() - 0.5) * 0.9),
      pos.z - dir.z * tailOffset + (Math.random() - 0.5) * 0.9
    );
    p.mesh.scale.setScalar(0.55 * sc);
    p.mesh.material.color.setHex(0xea580c);
    p.mesh.material.opacity = 0.78;
    p.vx = -dir.x * 6 + (Math.random() - 0.5) * 3.0;
    p.vy = -dir.y * 6 + Math.random() * 2.5;
    p.vz = -dir.z * 6 + (Math.random() - 0.5) * 3.0;
  }

  function updateSmokeParticles(dt) {
    for (let i = 0; i < MAX_SMOKE; i++) {
      const p = smokeParticles[i];
      if (!p.active) continue;
      p.age += dt;
      if (p.age >= p.maxAge) {
        p.active = false;
        p.mesh.visible = false;
        continue;
      }
      const progress = p.age / p.maxAge;
      p.mesh.position.x += p.vx * dt;
      p.mesh.position.y += p.vy * dt;
      p.mesh.position.z += p.vz * dt;
      p.mesh.scale.setScalar((0.6 + progress * 2.5) * (state.missileScale * 0.55));
      if (progress < 0.22) {
        p.mesh.material.color.setHex(0xea580c);
      } else {
        p.mesh.material.color.setHex(0x6b6257);
      }
      p.mesh.material.opacity = (1 - progress) * 0.45;
    }
  }

  function clearSmokeParticles() {
    for (let i = 0; i < MAX_SMOKE; i++) {
      smokeParticles[i].active = false;
      smokeParticles[i].mesh.visible = false;
    }
  }

  function applyMissileState(sample) {
    const sc = state.missileScale;
    missile.root.scale.setScalar(sc);
    missile.root.position.set(sample.x, Math.max(1.2, sample.y), sample.z);

    const speed = Math.sqrt(sample.vx * sample.vx + sample.vy * sample.vy + sample.vz * sample.vz) || 1;
    const dir = new THREE.Vector3(sample.vx / speed, sample.vy / speed, sample.vz / speed);

    const lookTarget = new THREE.Vector3(
      missile.root.position.x + dir.x * 20,
      missile.root.position.y + dir.y * 20,
      missile.root.position.z + dir.z * 20
    );
    missile.root.lookAt(lookTarget);

    const inFlight = sample.t < state.metrics.tFlight;
    missile.flameGroup.visible = inFlight;
    if (state.isPlaying && !state.isPaused && inFlight) {
      missile.finsGroup.rotation.y += 0.18;
      const flicker = 0.85 + Math.random() * 0.4;
      missile.flameGroup.scale.set(1, flicker, 1);
      missile.thrusterLight.intensity = 2.0 + Math.random() * 1.2;
      spawnSmokeParticle(missile.root.position, dir, sc);
      spawnSmokeParticle(missile.root.position, dir, sc);
    }

    groundShadowMarker.position.set(sample.x, 0.15, sample.z);

    const pos = missile.root.position;
    const vScale = 0.28 * Math.max(1, sc * 0.65);

    arrowV.visible = state.showVecV;
    if (state.showVecV) {
      arrowV.position.copy(pos);
      arrowV.setDirection(dir);
      arrowV.setLength(Math.max(8, speed * vScale), 4.2, 2.2);
    }

    const vHoriz = Math.sqrt(sample.vx * sample.vx + sample.vz * sample.vz);
    arrowVx.visible = state.showVecComp && vHoriz > 0.2;
    if (arrowVx.visible) {
      arrowVx.position.copy(pos);
      const dirH = new THREE.Vector3(sample.vx, 0, sample.vz).normalize();
      arrowVx.setDirection(dirH);
      arrowVx.setLength(Math.max(6, vHoriz * vScale), 3.4, 1.8);
    }

    arrowVy.visible = state.showVecComp && Math.abs(sample.vy) > 0.2;
    if (arrowVy.visible) {
      arrowVy.position.copy(pos);
      const dirY = new THREE.Vector3(0, sample.vy >= 0 ? 1 : -1, 0);
      arrowVy.setDirection(dirY);
      arrowVy.setLength(Math.max(6, Math.abs(sample.vy) * vScale), 3.4, 1.8);
    }

    const aMag = Math.sqrt(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
    arrowA.visible = state.showVecA && aMag > 0.1;
    if (arrowA.visible) {
      arrowA.position.copy(pos);
      const dirA = new THREE.Vector3(sample.ax, sample.ay, sample.az).normalize();
      arrowA.setDirection(dirA);
      arrowA.setLength(Math.max(7, aMag * 1.3 * Math.max(1, sc * 0.55)), 3.4, 1.8);
    }

    if (state.cameraMode === 'follow') {
      const dist = orbit.followDist;
      const sinP = Math.sin(orbit.followPhi);
      const offsetX = -dir.x * dist * 0.65 + dist * sinP * Math.sin(orbit.followTheta);
      const offsetY = Math.max(6, dist * Math.cos(orbit.followPhi) - dir.y * 8);
      const offsetZ = dist * sinP * Math.cos(orbit.followTheta);

      const desiredPos = new THREE.Vector3(
        pos.x + offsetX,
        Math.max(4, pos.y + offsetY),
        pos.z + offsetZ
      );
      camera.position.lerp(desiredPos, 0.18);
      camera.lookAt(pos.clone().add(dir.clone().multiplyScalar(4)));
    } else if (state.cameraMode === 'side') {
      const midX = Math.max(110, state.metrics.range * 0.5) + orbit.sidePanX;
      const lookY = Math.max(25, state.metrics.hMax * 0.38) + orbit.sidePanY;
      camera.position.set(midX, lookY + 12, orbit.sideDist);
      camera.lookAt(new THREE.Vector3(midX, lookY, 0));
    } else if (state.cameraMode === 'fpv') {
      const nosePos = pos.clone().add(dir.clone().multiplyScalar(7.5 * sc)).add(new THREE.Vector3(0, 2.2 * sc, 0));
      camera.position.copy(nosePos);
      camera.lookAt(pos.clone().add(dir.clone().multiplyScalar(80)));
    }
  }

  function updateLauncherAndTargetGeometry() {
    const h = Math.max(0.5, state.y0);
    pedestalMesh.scale.set(1, h, 1);
    pedestalMesh.position.y = h / 2;
    turretPivot.position.y = h;

    const theta = (state.angleDeg * Math.PI) / 180;
    const phi = (state.azimuthDeg * Math.PI) / 180;
    turretPivot.rotation.y = -phi;
    railArm.rotation.z = theta;

    targetGroup.position.set(state.targetDist, 0, 0);
  }

  // --- Gráfica sobre Papel Milimetrado de Laboratorio ---
  const chartCanvas = document.getElementById('telemetry-chart');
  const chartCtx = chartCanvas.getContext('2d');

  function drawTelemetryChart(currentSample) {
    const rect = chartCanvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    const dpr = window.devicePixelRatio || 1;
    if (chartCanvas.width !== Math.floor(rect.width * dpr) || chartCanvas.height !== Math.floor(rect.height * dpr)) {
      chartCanvas.width = Math.floor(rect.width * dpr);
      chartCanvas.height = Math.floor(rect.height * dpr);
    }

    const ctx = chartCtx;
    ctx.save();
    ctx.scale(dpr, dpr);
    const W = rect.width;
    const H = rect.height;
    ctx.clearRect(0, 0, W, H);

    // Cuadrícula sutil de papel milimetrado
    ctx.strokeStyle = 'rgba(184, 74, 39, 0.1)';
    ctx.lineWidth = 1;
    for (let gx = 0; gx < W; gx += 20) {
      ctx.beginPath();
      ctx.moveTo(gx, 0);
      ctx.lineTo(gx, H);
      ctx.stroke();
    }
    for (let gy = 0; gy < H; gy += 20) {
      ctx.beginPath();
      ctx.moveTo(0, gy);
      ctx.lineTo(W, gy);
      ctx.stroke();
    }

    const padL = 30;
    const padR = 10;
    const padT = 12;
    const padB = 20;
    const plotW = W - padL - padR;
    const plotH = H - padT - padB;

    ctx.strokeStyle = '#5c5349';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(padL, padT);
    ctx.lineTo(padL, H - padB);
    ctx.lineTo(W - padR, H - padB);
    ctx.stroke();

    const pts = state.trajectory;
    if (!pts || pts.length < 2) {
      ctx.restore();
      return;
    }

    if (state.chartMode === 'yx') {
      const maxX = Math.max(10, state.metrics.range, state.idealTrajectory[state.idealTrajectory.length - 1].x);
      const maxY = Math.max(10, state.metrics.hMax * 1.15);

      ctx.strokeStyle = '#235347';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      pts.forEach((p, idx) => {
        if (p.t > currentSample.t) return;
        const px = padL + (p.x / maxX) * plotW;
        const py = H - padB - (p.y / maxY) * plotH;
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();

      const cx = padL + (currentSample.x / maxX) * plotW;
      const cy = H - padB - (currentSample.y / maxY) * plotH;
      ctx.fillStyle = '#b84a27';
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, Math.PI * 2);
      ctx.fill();
    } else if (state.chartMode === 'vt') {
      const maxT = Math.max(0.5, state.metrics.tFlight);
      const maxV = Math.max(20, state.v0 * 1.2);

      ctx.strokeStyle = '#235347';
      ctx.lineWidth = 2;
      ctx.beginPath();
      pts.forEach((p, idx) => {
        if (p.t > currentSample.t) return;
        const px = padL + (p.t / maxT) * plotW;
        const py = H - padB - (Math.abs(p.vx) / maxV) * plotH;
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();

      ctx.strokeStyle = '#b84a27';
      ctx.beginPath();
      pts.forEach((p, idx) => {
        if (p.t > currentSample.t) return;
        const px = padL + (p.t / maxT) * plotW;
        const py = H - padB - (Math.abs(p.vy) / maxV) * plotH;
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    } else {
      const maxT = Math.max(0.5, state.metrics.tFlight);
      const p0 = pts[0];
      const eTotal0 = 0.5 * state.mass * state.v0 * state.v0 + state.mass * state.gravity * p0.y;
      const maxE = Math.max(100, eTotal0 * 1.1);

      const drawEnergyCurve = (color, fn) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        pts.forEach((p, idx) => {
          if (p.t > currentSample.t) return;
          const px = padL + (p.t / maxT) * plotW;
          const py = H - padB - (fn(p) / maxE) * plotH;
          if (idx === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        });
        ctx.stroke();
      };

      drawEnergyCurve('#235347', (p) => 0.5 * state.mass * (p.vx * p.vx + p.vy * p.vy + p.vz * p.vz));
      drawEnergyCurve('#b47b1c', (p) => state.mass * state.gravity * p.y);
      drawEnergyCurve('#b84a27', (p) => 0.5 * state.mass * (p.vx * p.vx + p.vy * p.vy + p.vz * p.vz) + state.mass * state.gravity * p.y);
    }

    ctx.restore();
  }

  function updateTelemetryUI(sample) {
    const speed = Math.sqrt(sample.vx * sample.vx + sample.vy * sample.vy + sample.vz * sample.vz);
    const vHoriz = Math.sqrt(sample.vx * sample.vx + sample.vz * sample.vz);
    const pitchDeg = (Math.atan2(sample.vy, vHoriz) * 180) / Math.PI;
    const ek = (0.5 * state.mass * speed * speed) / 1000;
    const ep = (state.mass * state.gravity * sample.y) / 1000;

    document.getElementById('tele-t').textContent = sample.t.toFixed(2) + ' s';
    document.getElementById('tele-v').textContent = speed.toFixed(2) + ' m/s';
    document.getElementById('tele-x').textContent = sample.x.toFixed(2) + ' m';
    document.getElementById('tele-y').textContent = sample.y.toFixed(2) + ' m';
    document.getElementById('tele-vx').textContent = 'v_x = ' + sample.vx.toFixed(2) + ' m/s';
    document.getElementById('tele-vy').textContent = 'v_y = ' + sample.vy.toFixed(2) + ' m/s';
    document.getElementById('tele-pitch').textContent = (pitchDeg >= 0 ? '+' : '') + pitchDeg.toFixed(1) + '°';
    document.getElementById('tele-ek').textContent = ek.toFixed(2) + ' kJ';
    document.getElementById('tele-ep').textContent = ep.toFixed(2) + ' kJ';

    const phasePill = document.getElementById('flight-phase-pill');
    if (sample.t >= state.metrics.tFlight) {
      phasePill.textContent = 'IMPACTO EN SUELO';
      phasePill.className = 'text-[10px] font-bold px-2 py-0.5 rounded-md bg-[#f9ebe5] text-[#b84a27] border border-[#e7bfb1]';
    } else if (sample.vy >= 0) {
      phasePill.textContent = '↗ ASCENSO (v_y > 0)';
      phasePill.className = 'text-[10px] font-bold px-2 py-0.5 rounded-md bg-[#e6f0ec] text-[#235347] border border-[#b7d4c8]';
    } else {
      phasePill.textContent = '↘ DESCENSO (v_y < 0)';
      phasePill.className = 'text-[10px] font-bold px-2 py-0.5 rounded-md bg-[#fdf3dc] text-[#b47b1c] border border-[#e8cf96]';
    }

    const pct = state.metrics.tFlight > 0
      ? Math.min(100, Math.round((sample.t / state.metrics.tFlight) * 100))
      : 0;
    document.getElementById('slider-timeline').value = Math.round((sample.t / (state.metrics.tFlight || 1)) * 1000);
    document.getElementById('val-timeline-pct').textContent = pct + '%';

    drawTelemetryChart(sample);
  }

  function recalculateSimulation() {
    const idealRes = computeTrajectoryPoints(false);
    state.idealTrajectory = idealRes.points;
    state.idealMetrics = idealRes.metrics;

    const realRes = state.useDrag ? computeTrajectoryPoints(true) : idealRes;
    state.trajectory = realRes.points;
    state.metrics = realRes.metrics;

    document.getElementById('metric-hmax').textContent = state.metrics.hMax.toFixed(1) + ' m';
    document.getElementById('metric-range').textContent = state.metrics.range.toFixed(1) + ' m';
    document.getElementById('metric-tof').textContent = state.metrics.tFlight.toFixed(2) + ' s';

    // Actualizar tabla comparativa del modal de Informe
    const idH = state.idealMetrics.hMax;
    const idR = state.idealMetrics.range;
    const idT = state.idealMetrics.tFlight;
    const reH = state.metrics.hMax;
    const reR = state.metrics.range;
    const reT = state.metrics.tFlight;

    document.getElementById('rep-ideal-hmax').textContent = idH.toFixed(2) + ' m';
    document.getElementById('rep-real-hmax').textContent = reH.toFixed(2) + ' m';
    document.getElementById('rep-diff-hmax').textContent = state.useDrag
      ? (((reH - idH) / idH) * 100).toFixed(1) + '% por arrastre'
      : '0.00% (Validado)';

    document.getElementById('rep-ideal-range').textContent = idR.toFixed(2) + ' m';
    document.getElementById('rep-real-range').textContent = reR.toFixed(2) + ' m';
    document.getElementById('rep-diff-range').textContent = state.useDrag
      ? (((reR - idR) / idR) * 100).toFixed(1) + '% por arrastre'
      : '0.00% (Validado)';

    document.getElementById('rep-ideal-tof').textContent = idT.toFixed(2) + ' s';
    document.getElementById('rep-real-tof').textContent = reT.toFixed(2) + ' s';
    document.getElementById('rep-diff-tof').textContent = state.useDrag
      ? (((reT - idT) / idT) * 100).toFixed(1) + '% por arrastre'
      : '0.00% (Validado)';

    const theta = (state.angleDeg * Math.PI) / 180;
    const vx0 = (state.v0 * Math.cos(theta)).toFixed(2);
    const vy0 = (state.v0 * Math.sin(theta)).toFixed(2);
    const halfG = (0.5 * state.gravity).toFixed(2);

    document.getElementById('eq-x').textContent = state.useDrag
      ? `x(t): EDO RK4 con C_d=${state.cd.toFixed(2)}`
      : `x(t) = ${vx0} · t  [MRU]`;
    document.getElementById('eq-y').textContent = `y(t) = ${state.y0} + ${vy0}·t - ${halfG}·t²  [MRUV]`;

    document.getElementById('motion-type-badge').textContent = state.useDrag
      ? 'Balística Exterior (Arrastre RK4)'
      : 'Movimiento Parabólico (MRU + MRUV)';

    updateLauncherAndTargetGeometry();
    rebuildTrajectoryLines();

    if (state.currentTime > state.metrics.tFlight) {
      state.currentTime = 0;
    }
    const sample = sampleTrajectoryAtTime(state.currentTime);
    applyMissileState(sample);
    updateTelemetryUI(sample);
  }

  function evaluateImpact() {
    const last = state.trajectory[state.trajectory.length - 1];
    const dx = last.x - state.targetDist;
    const dz = last.z;
    const distError = Math.sqrt(dx * dx + dz * dz);
    const banner = document.getElementById('impact-banner');
    const msg = document.getElementById('impact-message');
    const err = document.getElementById('impact-error');

    banner.classList.remove('hidden');
    explosionGroup.position.set(last.x, 0, last.z);
    shockwaveTimer = 1.0;

    if (distError <= 18.0) {
      msg.textContent = '🎯 ¡DIANA! Impacto directo en el objetivo';
      err.textContent = 'Desviación: ' + distError.toFixed(1) + ' m';
      sound.playExplosion(true);
    } else {
      msg.textContent = '📍 Impacto registrado en campo de pruebas';
      err.textContent = 'A ' + distError.toFixed(1) + ' m de la diana';
      sound.playExplosion(false);
    }
  }

  function startNewLaunch() {
    clearSmokeParticles();
    document.getElementById('impact-banner').classList.add('hidden');
    state.currentTime = 0;
    state.loopCooldown = 0;
    state.isPlaying = true;
    state.isPaused = false;
    document.getElementById('pause-text').textContent = '⏸ Pausa';
    sound.playLaunch();
  }

  function bindControls() {
    const syncAngle = (val) => {
      state.angleDeg = parseFloat(val);
      document.getElementById('slider-angle').value = state.angleDeg;
      document.getElementById('quick-slider-angle').value = state.angleDeg;
      document.getElementById('val-angle').textContent = state.angleDeg.toFixed(1) + '°';
      document.getElementById('quick-val-angle').textContent = Math.round(state.angleDeg) + '°';
      recalculateSimulation();
    };

    const syncV0 = (val) => {
      state.v0 = parseFloat(val);
      document.getElementById('slider-v0').value = state.v0;
      document.getElementById('quick-slider-v0').value = state.v0;
      document.getElementById('val-v0').textContent = state.v0.toFixed(1) + ' m/s';
      document.getElementById('quick-val-v0').textContent = Math.round(state.v0) + ' m/s';
      recalculateSimulation();
    };

    document.getElementById('slider-angle').addEventListener('input', (e) => syncAngle(e.target.value));
    document.getElementById('quick-slider-angle').addEventListener('input', (e) => syncAngle(e.target.value));
    document.getElementById('slider-v0').addEventListener('input', (e) => syncV0(e.target.value));
    document.getElementById('quick-slider-v0').addEventListener('input', (e) => syncV0(e.target.value));

    const bindSlider = (id, valId, prop, unit, decimals = 1) => {
      const el = document.getElementById(id);
      const valEl = document.getElementById(valId);
      el.addEventListener('input', () => {
        state[prop] = parseFloat(el.value);
        valEl.textContent = state[prop].toFixed(decimals) + unit;
        recalculateSimulation();
      });
    };

    bindSlider('slider-azimuth', 'val-azimuth', 'azimuthDeg', '°', 1);
    bindSlider('slider-y0', 'val-y0', 'y0', ' m', 1);
    bindSlider('slider-mass', 'val-mass', 'mass', ' kg', 1);
    bindSlider('slider-missile-scale', 'val-missile-scale', 'missileScale', 'x', 1);
    bindSlider('slider-gravity', 'val-gravity', 'gravity', ' m/s²', 2);
    bindSlider('slider-cd', 'val-cd', 'cd', '', 2);
    bindSlider('slider-wind', 'val-wind', 'windX', ' m/s', 1);
    bindSlider('slider-target', 'val-target', 'targetDist', ' m', 0);

    document.getElementById('chk-loop').addEventListener('change', (e) => {
      state.autoLoop = e.target.checked;
    });

    document.querySelectorAll('.gravity-preset').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.gravity-preset').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        const gVal = parseFloat(btn.getAttribute('data-g'));
        state.gravity = gVal;
        document.getElementById('slider-gravity').value = gVal;
        document.getElementById('val-gravity').textContent = gVal.toFixed(2) + ' m/s²';
        recalculateSimulation();
      });
    });

    const chkDrag = document.getElementById('chk-drag');
    const dragControls = document.getElementById('drag-controls');
    const dragPill = document.getElementById('drag-status-pill');
    chkDrag.addEventListener('change', () => {
      state.useDrag = chkDrag.checked;
      if (state.useDrag) {
        dragControls.classList.remove('opacity-50', 'pointer-events-none');
        dragPill.textContent = 'Arrastre RK4';
        dragPill.className = 'text-[10px] px-2 py-0.5 rounded bg-[#f9ebe5] text-[#b84a27] font-semibold';
      } else {
        dragControls.classList.add('opacity-50', 'pointer-events-none');
        dragPill.textContent = 'Ideal (Vacío)';
        dragPill.className = 'text-[10px] px-2 py-0.5 rounded bg-[#e6dfd3] text-[#5c5349] font-semibold';
      }
      recalculateSimulation();
    });

    document.getElementById('chk-vec-v').addEventListener('change', (e) => {
      state.showVecV = e.target.checked;
      applyMissileState(sampleTrajectoryAtTime(state.currentTime));
    });
    document.getElementById('chk-vec-comp').addEventListener('change', (e) => {
      state.showVecComp = e.target.checked;
      applyMissileState(sampleTrajectoryAtTime(state.currentTime));
    });
    document.getElementById('chk-vec-a').addEventListener('change', (e) => {
      state.showVecA = e.target.checked;
      applyMissileState(sampleTrajectoryAtTime(state.currentTime));
    });
    document.getElementById('chk-ideal-curve').addEventListener('change', (e) => {
      state.showIdealCurve = e.target.checked;
      if (idealCurveLine) idealCurveLine.visible = state.showIdealCurve;
    });

    document.querySelectorAll('.speed-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.speed-btn').forEach((b) => b.classList.remove('active', 'cam-btn'));
        btn.classList.add('active', 'cam-btn');
        state.simSpeed = parseFloat(btn.getAttribute('data-speed'));
      });
    });

    document.getElementById('btn-launch').addEventListener('click', startNewLaunch);

    document.getElementById('btn-pause').addEventListener('click', () => {
      state.isPaused = !state.isPaused;
      document.getElementById('pause-text').textContent = state.isPaused ? '▶ Reanudar' : '⏸ Pausa';
    });

    document.getElementById('btn-reset').addEventListener('click', () => {
      state.isPlaying = false;
      state.isPaused = false;
      state.currentTime = 0;
      document.getElementById('pause-text').textContent = '⏸ Pausa';
      document.getElementById('impact-banner').classList.add('hidden');
      clearSmokeParticles();
      const s = sampleTrajectoryAtTime(0);
      applyMissileState(s);
      updateTelemetryUI(s);
    });

    document.getElementById('btn-clear-trails').addEventListener('click', clearSmokeParticles);

    const timelineSlider = document.getElementById('slider-timeline');
    timelineSlider.addEventListener('input', () => {
      const frac = parseFloat(timelineSlider.value) / 1000;
      state.currentTime = frac * state.metrics.tFlight;
      state.isPaused = true;
      document.getElementById('pause-text').textContent = '▶ Reanudar';
      const s = sampleTrajectoryAtTime(state.currentTime);
      applyMissileState(s);
      updateTelemetryUI(s);
    });

    const camMap = {
      'btn-cam-orbit': 'orbit',
      'btn-cam-follow': 'follow',
      'btn-cam-side': 'side',
      'btn-cam-fpv': 'fpv'
    };
    Object.keys(camMap).forEach((btnId) => {
      document.getElementById(btnId).addEventListener('click', () => {
        Object.keys(camMap).forEach((id) => document.getElementById(id).classList.remove('active'));
        document.getElementById(btnId).classList.add('active');
        state.cameraMode = camMap[btnId];
        if (state.cameraMode === 'orbit') updateOrbitCamera();
        else applyMissileState(sampleTrajectoryAtTime(state.currentTime));
      });
    });

    const chartMap = {
      'btn-chart-yx': 'yx',
      'btn-chart-vt': 'vt',
      'btn-chart-et': 'et'
    };
    Object.keys(chartMap).forEach((btnId) => {
      document.getElementById(btnId).addEventListener('click', () => {
        Object.keys(chartMap).forEach((id) => document.getElementById(id).classList.remove('active'));
        document.getElementById(btnId).classList.add('active');
        state.chartMode = chartMap[btnId];
        drawTelemetryChart(sampleTrajectoryAtTime(state.currentTime));
      });
    });

    
    const btnSound = document.getElementById('btn-toggle-sound');
    btnSound.addEventListener('click', () => {
      state.soundEnabled = !state.soundEnabled;
      btnSound.textContent = state.soundEnabled ? '🔊' : '🔇';
    });

    const leftBody = document.getElementById('left-panel-body');
    const btnLeft = document.getElementById('btn-toggle-left');
    btnLeft.textContent = 'Abrir ▾';
    btnLeft.addEventListener('click', () => {
      const isHidden = leftBody.classList.toggle('hidden');
      leftBody.classList.toggle('flex', !isHidden);
      btnLeft.textContent = isHidden ? 'Abrir ▾' : 'Ocultar ▴';
    });

    const rightBody = document.getElementById('right-panel-body');
    const btnRight = document.getElementById('btn-toggle-right');
    btnRight.textContent = 'Abrir ▾';
    btnRight.addEventListener('click', () => {
      const isHidden = rightBody.classList.toggle('hidden');
      rightBody.classList.toggle('flex', !isHidden);
      btnRight.textContent = isHidden ? 'Abrir ▾' : 'Ocultar ▴';
      if (!isHidden) {
        setTimeout(() => drawTelemetryChart(sampleTrajectoryAtTime(state.currentTime)), 30);
      }
    });

    // Controles de Zoom en Pantalla (Botones + / -, Slider, Vista de Cerca y Encuadrar Todo)
    let zoomHoldInterval = null;
    const startContinuousZoom = (factor) => {
      zoomByFactor(factor);
      if (zoomHoldInterval) clearInterval(zoomHoldInterval);
      zoomHoldInterval = setInterval(() => zoomByFactor(factor), 45);
    };
    const stopContinuousZoom = () => {
      if (zoomHoldInterval) {
        clearInterval(zoomHoldInterval);
        zoomHoldInterval = null;
      }
    };

    const btnZoomIn = document.getElementById('btn-zoom-in');
    const btnZoomOut = document.getElementById('btn-zoom-out');

    btnZoomIn.addEventListener('pointerdown', () => startContinuousZoom(0.86));
    btnZoomOut.addEventListener('pointerdown', () => startContinuousZoom(1.16));
    window.addEventListener('pointerup', stopContinuousZoom);
    window.addEventListener('pointercancel', stopContinuousZoom);

    document.getElementById('slider-zoom').addEventListener('input', (e) => {
      setActiveZoomDistance(parseFloat(e.target.value));
    });

    document.getElementById('btn-zoom-close').addEventListener('click', () => {
      // Acercar directamente al misil en 3D
      document.querySelectorAll('.cam-btn').forEach((b) => {
        if (b.id && b.id.startsWith('btn-cam-')) b.classList.remove('active');
      });
      document.getElementById('btn-cam-follow').classList.add('active');
      state.cameraMode = 'follow';
      setActiveZoomDistance(30);
    });

    document.getElementById('btn-zoom-fit').addEventListener('click', () => {
      // Alejar la cámara para encuadrar toda la trayectoria parabólica completa
      const fullRange = Math.max(180, state.metrics.range);
      if (state.cameraMode === 'follow') {
        setActiveZoomDistance(Math.min(480, fullRange * 0.72));
      } else if (state.cameraMode === 'side') {
        orbit.sidePanX = 0;
        orbit.sidePanY = 0;
        setActiveZoomDistance(Math.max(260, fullRange * 0.92));
      } else {
        orbit.target.set(fullRange * 0.48, Math.max(30, state.metrics.hMax * 0.4), 0);
        setActiveZoomDistance(Math.max(260, fullRange * 0.92));
      }
    });

    
  }

  function createZipBlob(files) {
    const encoder = new TextEncoder();
    const crcTable = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) {
        c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      }
      crcTable[n] = c >>> 0;
    }

    function crc32(bytes) {
      let crc = 0xffffffff;
      for (let i = 0; i < bytes.length; i++) {
        crc = crcTable[(crc ^ bytes[i]) & 0xff] ^ (crc >>> 8);
      }
      return (crc ^ 0xffffffff) >>> 0;
    }

    const localChunks = [];
    const centralChunks = [];
    let offset = 0;

    files.forEach((f) => {
      const nameBytes = encoder.encode(f.name);
      const dataBytes = encoder.encode(f.content);
      const crc = crc32(dataBytes);

      const localHeader = new Uint8Array(30 + nameBytes.length);
      const lv = new DataView(localHeader.buffer);
      lv.setUint32(0, 0x04034b50, true);
      lv.setUint16(4, 20, true);
      lv.setUint16(6, 0, true);
      lv.setUint16(8, 0, true);
      lv.setUint16(10, 0, true);
      lv.setUint16(12, 0, true);
      lv.setUint32(14, crc, true);
      lv.setUint32(18, dataBytes.length, true);
      lv.setUint32(22, dataBytes.length, true);
      lv.setUint16(26, nameBytes.length, true);
      lv.setUint16(28, 0, true);
      localHeader.set(nameBytes, 30);

      localChunks.push(localHeader, dataBytes);

      const centralHeader = new Uint8Array(46 + nameBytes.length);
      const cv = new DataView(centralHeader.buffer);
      cv.setUint32(0, 0x02014b50, true);
      cv.setUint16(4, 20, true);
      cv.setUint16(6, 20, true);
      cv.setUint16(8, 0, true);
      cv.setUint16(10, 0, true);
      cv.setUint16(12, 0, true);
      cv.setUint16(14, 0, true);
      cv.setUint32(16, crc, true);
      cv.setUint32(20, dataBytes.length, true);
      cv.setUint32(24, dataBytes.length, true);
      cv.setUint16(28, nameBytes.length, true);
      cv.setUint16(30, 0, true);
      cv.setUint16(32, 0, true);
      cv.setUint16(34, 0, true);
      cv.setUint16(36, 0, true);
      cv.setUint32(38, 0, true);
      cv.setUint32(42, offset, true);
      centralHeader.set(nameBytes, 46);

      centralChunks.push(centralHeader);
      offset += localHeader.length + dataBytes.length;
    });

    const centralSize = centralChunks.reduce((acc, c) => acc + c.length, 0);
    const eocd = new Uint8Array(22);
    const ev = new DataView(eocd.buffer);
    ev.setUint32(0, 0x06054b50, true);
    ev.setUint16(4, 0, true);
    ev.setUint16(6, 0, true);
    ev.setUint16(8, files.length, true);
    ev.setUint16(10, files.length, true);
    ev.setUint32(12, centralSize, true);
    ev.setUint32(16, offset, true);
    ev.setUint16(20, 0, true);

    return new Blob([...localChunks, ...centralChunks, eocd], { type: 'application/zip' });
  }

  function downloadProjectZipInBrowser() {
    const fullHtml = '<!DOCTYPE html>\n' + document.documentElement.outerHTML;
    const readme = [
      '# Práctica de Laboratorio: Simulación Balística 3D (RK4)',
      '',
      'Interfaz de Cuaderno de Laboratorio adaptada a la Estructura General del Informe de Física Computacional.',
      'Abre `simulacion_proyectil_3d.html` en cualquier navegador web.'
    ].join('\n');

    const blob = createZipBlob([
      { name: 'simulacion_proyectil_3d.html', content: fullHtml },
      { name: 'README.md', content: readme }
    ]);

    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'simulacion_proyectil_3d.zip';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(() => URL.revokeObjectURL(url), 1500);
  }

  let lastFrameTime = performance.now();

  function animate(now) {
    requestAnimationFrame(animate);

    const dt = Math.min(0.05, (now - lastFrameTime) / 1000);
    lastFrameTime = now;

    if (state.isPlaying && !state.isPaused) {
      if (state.loopCooldown > 0) {
        state.loopCooldown -= dt;
        if (state.loopCooldown <= 0 && state.autoLoop) {
          startNewLaunch();
        }
      } else {
        state.currentTime += dt * state.simSpeed;
        if (state.currentTime >= state.metrics.tFlight) {
          state.currentTime = state.metrics.tFlight;
          evaluateImpact();
          if (state.autoLoop) {
            state.loopCooldown = 1.6;
          } else {
            state.isPlaying = false;
          }
        }
        const sample = sampleTrajectoryAtTime(state.currentTime);
        applyMissileState(sample);
        updateTelemetryUI(sample);
      }
    }

    if (shockwaveTimer > 0) {
      shockwaveTimer = Math.max(0, shockwaveTimer - dt * 1.15);
      const progress = 1 - shockwaveTimer;
      shockwaveRing.scale.setScalar(1 + progress * 22);
      shockwaveRing.material.opacity = shockwaveTimer * 0.9;

      fireballMesh.scale.setScalar(1 + progress * 14);
      fireballMesh.material.opacity = shockwaveTimer * 0.75;
    } else {
      fireballMesh.material.opacity = 0;
      shockwaveRing.material.opacity = 0;
    }

    updateSmokeParticles(dt);
    renderer.render(scene, camera);
  }

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    drawTelemetryChart(sampleTrajectoryAtTime(state.currentTime));
  });

  window.addEventListener('load', () => {
    bindControls();
    recalculateSimulation();
    camera.position.set(-25, 22, 48);
    state.isPlaying = true;
    state.currentTime = 0;
    requestAnimationFrame(animate);
  });
})();
