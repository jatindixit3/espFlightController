// Main UI glue. Talks to the FC only through FcApi (fc-api.js) - never builds
// raw MSP frames here.

const msp = new MspClient();
const api = new FcApi(msp);

const state = {
    activeTab: 'setup',
    misc: null,
    pid: null,
    rates: null,
    modes: null,
    motorTestEnabled: false,
    motorTestValues: [0, 0, 0, 0],
    pollTimer: null,
    motorTestTimer: null,
};

const MODE_NAMES = ['Arm', 'Angle', 'Horizon', 'Blackbox'];

function $(id) { return document.getElementById(id); }

// ---------------------------------------------------------------------------
// Connection
// ---------------------------------------------------------------------------

$('connectBtn').addEventListener('click', async () => {
    if (msp.isConnected()) {
        await disconnectFromFc();
        return;
    }
    try {
        await msp.connect();
        msp.onDisconnect = () => disconnectFromFc(true);
        const id = await api.identify();
        $('fcIdentify').textContent = id;
        $('connectBtn').textContent = 'Disconnect';
        startPolling();
    } catch (e) {
        alert('Connection failed: ' + e.message);
    }
});

async function disconnectFromFc(alreadyDropped) {
    stopPolling();
    if (!alreadyDropped) await msp.disconnect();
    $('connectBtn').textContent = 'Connect';
    $('fcIdentify').textContent = '';
    $('armedBadge').classList.add('hidden');
    $('failsafeBadge').classList.add('hidden');
    $('calBadge').classList.add('hidden');
    $('batteryBadge').classList.add('hidden');
}

// ---------------------------------------------------------------------------
// Tabs
// ---------------------------------------------------------------------------

document.querySelectorAll('.tab-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn').forEach((b) => b.classList.remove('active'));
        document.querySelectorAll('.tab-panel').forEach((p) => p.classList.remove('active'));
        btn.classList.add('active');
        $('tab-' + btn.dataset.tab).classList.add('active');
        state.activeTab = btn.dataset.tab;
    });
});

// ---------------------------------------------------------------------------
// Polling loop (status always; tab-specific extras only while relevant tab is active)
// ---------------------------------------------------------------------------

function startPolling() {
    stopPolling();
    state.pollTimer = setInterval(pollTick, 200);
}

function stopPolling() {
    if (state.pollTimer) clearInterval(state.pollTimer);
    state.pollTimer = null;
    if (state.motorTestTimer) clearInterval(state.motorTestTimer);
    state.motorTestTimer = null;
}

async function pollTick() {
    if (!msp.isConnected()) return;
    try {
        const status = await api.getStatus();
        updateStatusBadges(status);

        if (state.activeTab === 'setup') {
            const [att, imu] = await Promise.all([api.getAttitude(), api.getRawImu()]);
            updateAttitude(att);
            updateRawImu(imu);
        } else if (state.activeTab === 'receiver') {
            const channels = await api.getRc();
            updateChannelBars(channels);
        } else if (state.activeTab === 'motors') {
            const telemetry = await api.getEscTelemetry();
            updateEscTelemetryTable(telemetry);
        } else if (state.activeTab === 'modes') {
            const channels = await api.getRc();
            updateModesLiveValues(channels);
        }
    } catch (e) {
        console.error('Poll error', e);
    }
}

function updateStatusBadges(status) {
    $('armedBadge').classList.toggle('hidden', !status.armed);
    $('failsafeBadge').classList.toggle('hidden', !status.failsafe);
    $('calBadge').classList.toggle('hidden', status.gyroCalibrated);
    const battBadge = $('batteryBadge');
    if (status.batteryVoltage > 0) {
        battBadge.textContent = status.batteryVoltage.toFixed(2) + 'V / ' + status.totalCurrent.toFixed(1) + 'A';
        battBadge.classList.remove('hidden');
    } else {
        battBadge.classList.add('hidden');
    }
}

// ---------------------------------------------------------------------------
// Setup tab
// ---------------------------------------------------------------------------

function updateAttitude(att) {
    $('attRoll').textContent = att.roll.toFixed(1);
    $('attPitch').textContent = att.pitch.toFixed(1);
    $('attYaw').textContent = att.yaw.toFixed(1);
    drawAttitude(att.roll, att.pitch);
}

function updateRawImu(imu) {
    $('gyroX').textContent = imu.gyroDegS[0].toFixed(1);
    $('gyroY').textContent = imu.gyroDegS[1].toFixed(1);
    $('gyroZ').textContent = imu.gyroDegS[2].toFixed(1);
    $('accelX').textContent = imu.accelG[0].toFixed(2);
    $('accelY').textContent = imu.accelG[1].toFixed(2);
    $('accelZ').textContent = imu.accelG[2].toFixed(2);
}

function drawAttitude(rollDeg, pitchDeg) {
    const canvas = $('attitudeCanvas');
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    ctx.save();
    ctx.translate(w / 2, h / 2);
    ctx.rotate(-rollDeg * Math.PI / 180);

    const pitchOffset = pitchDeg * 2.5; // px per degree, purely visual scale

    // Sky
    ctx.fillStyle = '#2a5fb0';
    ctx.fillRect(-w, -h + pitchOffset, w * 2, h * 2);
    // Ground
    ctx.fillStyle = '#5a3d1f';
    ctx.fillRect(-w, pitchOffset, w * 2, h * 2);
    // Horizon line
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-w, pitchOffset);
    ctx.lineTo(w, pitchOffset);
    ctx.stroke();
    ctx.restore();

    // Fixed aircraft symbol
    ctx.strokeStyle = '#facc15';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(w / 2 - 40, h / 2);
    ctx.lineTo(w / 2 - 10, h / 2);
    ctx.moveTo(w / 2 + 10, h / 2);
    ctx.lineTo(w / 2 + 40, h / 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(w / 2, h / 2, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#facc15';
    ctx.fill();
}

$('calibrateBtn').addEventListener('click', async () => {
    if (!msp.isConnected()) { alert('Connect first'); return; }
    $('calibrateBtn').disabled = true;
    $('calibrateStatus').textContent = 'Calibrating - hold the quad still... (up to ~12s)';
    try {
        const ok = await api.calibrateGyro();
        $('calibrateStatus').textContent = ok
            ? 'Calibration succeeded.'
            : 'Calibration failed - the board moved (or was never still). Try again on a flat, stable surface.';
    } catch (e) {
        $('calibrateStatus').textContent = 'Calibration request failed: ' + e.message;
    } finally {
        $('calibrateBtn').disabled = false;
    }
});

// ---------------------------------------------------------------------------
// Shared MISC load/save (used by Configuration, Receiver, Failsafe, Blackbox tabs)
// ---------------------------------------------------------------------------

async function loadMisc() {
    state.misc = await api.getMisc();
    const m = state.misc;
    $('alignRoll').value = m.boardAlignRollDeg;
    $('alignPitch').value = m.boardAlignPitchDeg;
    $('alignYaw').value = m.boardAlignYawDeg;
    $('motorIdle').value = m.motorIdlePercent;
    $('polePairs').value = m.motorPolePairs;
    $('cellOverride').value = m.batteryCellOverride;
    $('battWarn').value = m.batteryWarnV;
    $('battCrit').value = m.batteryCritV;
    $('rxMin').value = m.rxMinUs;
    $('rxMid').value = m.rxMidUs;
    $('rxMax').value = m.rxMaxUs;
    $('failsafeTimeoutMs').value = m.failsafeTimeoutMs;
    $('blackboxRateDivider').value = m.blackboxRateDivider;
}

async function writeMisc() {
    const m = state.misc || {};
    m.boardAlignRollDeg = parseFloat($('alignRoll').value) || 0;
    m.boardAlignPitchDeg = parseFloat($('alignPitch').value) || 0;
    m.boardAlignYawDeg = parseFloat($('alignYaw').value) || 0;
    m.motorIdlePercent = parseFloat($('motorIdle').value) || 0;
    m.motorPolePairs = parseInt($('polePairs').value) || 0;
    m.batteryCellOverride = parseInt($('cellOverride').value) || 0;
    m.batteryWarnV = parseFloat($('battWarn').value) || 0;
    m.batteryCritV = parseFloat($('battCrit').value) || 0;
    m.rxMinUs = parseInt($('rxMin').value) || 0;
    m.rxMidUs = parseInt($('rxMid').value) || 0;
    m.rxMaxUs = parseInt($('rxMax').value) || 0;
    m.failsafeTimeoutMs = parseInt($('failsafeTimeoutMs').value) || 0;
    m.blackboxRateDivider = parseInt($('blackboxRateDivider').value) || 1;
    state.misc = m;
    await api.setMisc(m);
}

// ---------------------------------------------------------------------------
// PID tab
// ---------------------------------------------------------------------------

async function loadPid() {
    state.pid = await api.getPid();
    for (let i = 0; i < 3; i++) {
        $(`pid-${i}-P`).value = state.pid.axes[i].P;
        $(`pid-${i}-I`).value = state.pid.axes[i].I;
        $(`pid-${i}-D`).value = state.pid.axes[i].D;
        $(`pid-${i}-FF`).value = state.pid.axes[i].FF;
    }
    $('dtermLpf').value = state.pid.dtermLowpassHz;
    $('gyroLpf').value = state.pid.gyroLowpassHz;
    $('rpmFilterEnabled').checked = state.pid.rpmFilterEnabled;
    $('levelGainP').value = state.pid.levelGainP;
    $('horizonTiltEffect').value = state.pid.horizonTiltEffect;
    $('maxAngleDeg').value = state.pid.maxAngleDeg;
}

async function writePid() {
    const p = state.pid || { axes: [{}, {}, {}] };
    for (let i = 0; i < 3; i++) {
        p.axes[i] = {
            P: parseFloat($(`pid-${i}-P`).value) || 0,
            I: parseFloat($(`pid-${i}-I`).value) || 0,
            D: parseFloat($(`pid-${i}-D`).value) || 0,
            FF: parseFloat($(`pid-${i}-FF`).value) || 0,
        };
    }
    p.dtermLowpassHz = parseFloat($('dtermLpf').value) || 0;
    p.gyroLowpassHz = parseFloat($('gyroLpf').value) || 0;
    p.rpmFilterEnabled = $('rpmFilterEnabled').checked;
    p.levelGainP = parseFloat($('levelGainP').value) || 0;
    p.horizonTiltEffect = parseFloat($('horizonTiltEffect').value) || 0;
    p.maxAngleDeg = parseFloat($('maxAngleDeg').value) || 0;
    state.pid = p;
    await api.setPid(p);
}

// ---------------------------------------------------------------------------
// Rates tab
// ---------------------------------------------------------------------------

async function loadRates() {
    state.rates = await api.getRates();
    for (let i = 0; i < 3; i++) {
        $(`rate-${i}-rcRate`).value = state.rates[i].rcRate;
        $(`rate-${i}-superRate`).value = state.rates[i].superRate;
        $(`rate-${i}-expo`).value = state.rates[i].expo;
    }
    drawRatesCurve();
}

async function writeRates() {
    const rates = [];
    for (let i = 0; i < 3; i++) {
        rates.push({
            rcRate: parseFloat($(`rate-${i}-rcRate`).value) || 0,
            superRate: parseFloat($(`rate-${i}-superRate`).value) || 0,
            expo: parseFloat($(`rate-${i}-expo`).value) || 0,
        });
    }
    state.rates = rates;
    await api.setRates(rates);
    drawRatesCurve();
}

function applyRatesCurve(stick, rp) {
    const expoApplied = stick * (1 - rp.expo) + stick * stick * stick * rp.expo;
    let angleRate = 200 * rp.rcRate * expoApplied;
    if (rp.superRate > 0) {
        const absExpo = Math.abs(expoApplied);
        const factor = 1 / Math.max(1 - absExpo * rp.superRate, 0.01);
        angleRate *= factor;
    }
    return angleRate;
}

function drawRatesCurve() {
    const canvas = $('ratesCanvas');
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    if (!state.rates) return;
    const rp = state.rates[0];

    ctx.strokeStyle = '#333';
    ctx.beginPath();
    ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
    ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h);
    ctx.stroke();

    ctx.strokeStyle = '#3b82f6';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const maxRate = 1000; // deg/s scale for the plot
    for (let px = 0; px <= w; px++) {
        const stick = (px / w) * 2 - 1;
        const rate = applyRatesCurve(stick, rp);
        const py = h / 2 - (rate / maxRate) * (h / 2);
        if (px === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke();
}

// ---------------------------------------------------------------------------
// Receiver tab
// ---------------------------------------------------------------------------

function ensureChannelBars() {
    const container = $('channelBars');
    if (container.children.length === 16) return;
    container.innerHTML = '';
    for (let i = 0; i < 16; i++) {
        const row = document.createElement('div');
        row.className = 'channel-bar-row';
        const label = document.createElement('div');
        label.className = 'channel-bar-label';
        label.textContent = i < 4 ? ['Roll', 'Pitch', 'Throttle', 'Yaw'][i] : `AUX${i - 3}`;
        const track = document.createElement('div');
        track.className = 'channel-bar-track';
        const fill = document.createElement('div');
        fill.className = 'channel-bar-fill';
        fill.id = `chFill${i}`;
        track.appendChild(fill);
        const value = document.createElement('div');
        value.id = `chValue${i}`;
        value.style.width = '50px';
        row.appendChild(label);
        row.appendChild(track);
        row.appendChild(value);
        container.appendChild(row);
    }
}
ensureChannelBars();

function updateChannelBars(channels) {
    for (let i = 0; i < 16; i++) {
        const us = channels[i];
        const pct = Math.max(0, Math.min(100, ((us - 988) / (2012 - 988)) * 100));
        $(`chFill${i}`).style.width = pct + '%';
        $(`chValue${i}`).textContent = us + 'us';
    }
}

// ---------------------------------------------------------------------------
// Modes tab
// ---------------------------------------------------------------------------

function renderModesTable() {
    const table = $('modesTable');
    table.querySelectorAll('tr.mode-row').forEach((r) => r.remove());
    if (!state.modes) return;
    for (let i = 0; i < 4; i++) {
        const mode = state.modes[i];
        const row = document.createElement('tr');
        row.className = 'mode-row';

        const nameCell = document.createElement('td');
        nameCell.textContent = MODE_NAMES[i];

        const chCell = document.createElement('td');
        const chSelect = document.createElement('select');
        const noneOpt = document.createElement('option');
        noneOpt.value = -1; noneOpt.textContent = 'None';
        chSelect.appendChild(noneOpt);
        for (let a = 0; a < 12; a++) {
            const opt = document.createElement('option');
            opt.value = a; opt.textContent = `AUX${a + 1}`;
            chSelect.appendChild(opt);
        }
        chSelect.value = mode.channel;
        chSelect.id = `mode-${i}-channel`;
        chCell.appendChild(chSelect);

        const startCell = document.createElement('td');
        const startInput = document.createElement('input');
        startInput.type = 'number'; startInput.step = 1; startInput.value = mode.rangeStartUs;
        startInput.id = `mode-${i}-start`;
        startCell.appendChild(startInput);

        const endCell = document.createElement('td');
        const endInput = document.createElement('input');
        endInput.type = 'number'; endInput.step = 1; endInput.value = mode.rangeEndUs;
        endInput.id = `mode-${i}-end`;
        endCell.appendChild(endInput);

        const liveCell = document.createElement('td');
        liveCell.id = `mode-${i}-live`;
        liveCell.textContent = '-';

        row.appendChild(nameCell);
        row.appendChild(chCell);
        row.appendChild(startCell);
        row.appendChild(endCell);
        row.appendChild(liveCell);
        table.appendChild(row);
    }
}

function updateModesLiveValues(channels) {
    if (!state.modes) return;
    for (let i = 0; i < 4; i++) {
        const chSelect = $(`mode-${i}-channel`);
        if (!chSelect) continue;
        const auxIdx = parseInt(chSelect.value);
        const liveCell = $(`mode-${i}-live`);
        if (auxIdx < 0) { liveCell.textContent = '-'; continue; }
        const us = channels[4 + auxIdx];
        liveCell.textContent = us !== undefined ? us + 'us' : '-';
    }
}

async function loadModesTab() {
    state.modes = await api.getModes();
    renderModesTable();
}

async function writeModesTab() {
    const modes = [];
    for (let i = 0; i < 4; i++) {
        modes.push({
            channel: parseInt($(`mode-${i}-channel`).value),
            rangeStartUs: parseInt($(`mode-${i}-start`).value) || 0,
            rangeEndUs: parseInt($(`mode-${i}-end`).value) || 0,
        });
    }
    state.modes = modes;
    await api.setModes(modes);
}

// ---------------------------------------------------------------------------
// Motors tab
// ---------------------------------------------------------------------------

function ensureMotorSliders() {
    const container = $('motorSliders');
    if (container.children.length === 4) return;
    container.innerHTML = '';
    for (let i = 0; i < 4; i++) {
        const row = document.createElement('div');
        row.className = 'motor-slider-row';
        const label = document.createElement('div');
        label.textContent = `M${i + 1}`;
        const slider = document.createElement('input');
        slider.type = 'range'; slider.min = 0; slider.max = 100; slider.value = 0;
        slider.id = `motorSlider${i}`;
        slider.disabled = true;
        const value = document.createElement('div');
        value.id = `motorSliderValue${i}`;
        value.style.width = '40px';
        value.textContent = '0%';
        slider.addEventListener('input', () => {
            value.textContent = slider.value + '%';
            const pct = parseInt(slider.value);
            state.motorTestValues[i] = pct === 0 ? 0 : Math.round(48 + (pct / 100) * (2047 - 48));
        });
        row.appendChild(label);
        row.appendChild(slider);
        row.appendChild(value);
        container.appendChild(row);
    }
}
ensureMotorSliders();

$('motorTestArm').addEventListener('change', (e) => {
    state.motorTestEnabled = e.target.checked;
    for (let i = 0; i < 4; i++) $(`motorSlider${i}`).disabled = !state.motorTestEnabled;

    if (state.motorTestEnabled) {
        state.motorTestTimer = setInterval(async () => {
            if (!msp.isConnected()) return;
            try { await api.motorTest(state.motorTestValues); } catch (e) { /* ignore transient errors */ }
        }, 200);
    } else {
        if (state.motorTestTimer) clearInterval(state.motorTestTimer);
        state.motorTestTimer = null;
        state.motorTestValues = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
            $(`motorSlider${i}`).value = 0;
            $(`motorSliderValue${i}`).textContent = '0%';
        }
        if (msp.isConnected()) api.motorTest([0, 0, 0, 0]).catch(() => {});
    }
});

function updateEscTelemetryTable(telemetry) {
    const table = $('escTelemetryTable');
    table.querySelectorAll('tr.esc-row').forEach((r) => r.remove());
    telemetry.forEach((t, i) => {
        const row = document.createElement('tr');
        row.className = 'esc-row';
        row.innerHTML = `<td>M${i + 1}</td><td>${t.tempC}&deg;C</td><td>${t.voltage.toFixed(2)}V</td>` +
            `<td>${t.current.toFixed(2)}A</td><td>${t.eRpm}</td><td>${t.lastUpdateMs ? 'live' : 'no data'}</td>`;
        table.appendChild(row);
    });
}

// ---------------------------------------------------------------------------
// Blackbox tab
// ---------------------------------------------------------------------------

async function loadBlackboxInfo() {
    const info = await api.getBlackboxInfo();
    $('blackboxInfo').textContent = `Write offset: ${info.writeOffset} bytes / partition size: ${info.partitionSize} bytes`;
    return info;
}

$('downloadBlackboxBtn').addEventListener('click', async () => {
    if (!msp.isConnected()) { alert('Connect first'); return; }
    const info = await loadBlackboxInfo();
    const chunkSize = 240;
    const total = info.writeOffset;
    if (total === 0) { alert('No blackbox data logged yet.'); return; }

    const allBytes = new Uint8Array(total);
    let offset = 0;
    $('blackboxProgress').textContent = 'Downloading...';
    while (offset < total) {
        const len = Math.min(chunkSize, total - offset);
        const chunk = await api.readBlackboxChunk(offset, len);
        allBytes.set(chunk, offset);
        offset += chunk.length;
        $('blackboxProgress').textContent = `Downloading... ${offset}/${total} bytes`;
        if (chunk.length === 0) break; // avoid infinite loop on read failure
    }

    const csv = decodeBlackboxToCsv(allBytes);
    const blob = new Blob([csv], { type: 'text/csv' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `blackbox_${Date.now()}.csv`;
    a.click();
    $('blackboxProgress').textContent = 'Download complete.';
});

function decodeBlackboxToCsv(bytes) {
    const MAGIC = 0x424C4B58;
    // Field layout must match firmware/include/blackbox.h BlackboxFrame exactly.
    const RECORD_SIZE = 4 + 4 + 12 + 12 + 12 + 8 + 4 + 1 + 1;
    const view = new DataView(bytes.buffer);
    const rows = ['timestampMs,gyroX,gyroY,gyroZ,stickRoll,stickPitch,stickYaw,pidRoll,pidPitch,pidYaw,m1,m2,m3,m4,batteryVoltage,armed,mode'];

    for (let offset = 0; offset + RECORD_SIZE <= bytes.length; offset++) {
        if (view.getUint32(offset, true) !== MAGIC) continue;
        let p = offset + 4;
        const timestampMs = view.getUint32(p, true); p += 4;
        const gyro = [0, 1, 2].map(() => { const v = view.getFloat32(p, true); p += 4; return v; });
        const stick = [0, 1, 2].map(() => { const v = view.getFloat32(p, true); p += 4; return v; });
        const pidOut = [0, 1, 2].map(() => { const v = view.getFloat32(p, true); p += 4; return v; });
        const motors = [0, 1, 2, 3].map(() => { const v = view.getUint16(p, true); p += 2; return v; });
        const battV = view.getFloat32(p, true); p += 4;
        const armed = view.getUint8(p); p += 1;
        const mode = view.getUint8(p); p += 1;

        rows.push([timestampMs, ...gyro, ...stick, ...pidOut, ...motors, battV, armed, mode].join(','));
        offset += RECORD_SIZE - 1; // -1 because the outer loop also increments by 1
    }
    return rows.join('\n');
}

$('eraseBlackboxBtn').addEventListener('click', async () => {
    if (!msp.isConnected()) { alert('Connect first'); return; }
    if (!confirm('Erase the blackbox log? This cannot be undone.')) return;
    await api.eraseBlackbox();
    await loadBlackboxInfo();
});

// ---------------------------------------------------------------------------
// CLI (read-only dump) tab
// ---------------------------------------------------------------------------

$('dumpBtn').addEventListener('click', async () => {
    if (!msp.isConnected()) { alert('Connect first'); return; }
    const [pid, rates, modes, misc] = await Promise.all([api.getPid(), api.getRates(), api.getModes(), api.getMisc()]);
    const lines = [];
    lines.push('# ESP32 FC settings dump');
    lines.push('# axis order: roll, pitch, yaw');
    pid.axes.forEach((a, i) => lines.push(`set pid_${i}_P = ${a.P}, pid_${i}_I = ${a.I}, pid_${i}_D = ${a.D}, pid_${i}_FF = ${a.FF}`));
    lines.push(`set dterm_lowpass_hz = ${pid.dtermLowpassHz}`);
    lines.push(`set gyro_lowpass_hz = ${pid.gyroLowpassHz}`);
    lines.push(`set rpm_filter_enabled = ${pid.rpmFilterEnabled}`);
    lines.push(`set level_gain_p = ${pid.levelGainP}`);
    lines.push(`set horizon_tilt_effect = ${pid.horizonTiltEffect}`);
    lines.push(`set max_angle_deg = ${pid.maxAngleDeg}`);
    rates.forEach((r, i) => lines.push(`set rate_${i}_rcRate = ${r.rcRate}, rate_${i}_superRate = ${r.superRate}, rate_${i}_expo = ${r.expo}`));
    modes.forEach((m, i) => lines.push(`set mode_${MODE_NAMES[i]} = aux${m.channel}, ${m.rangeStartUs}-${m.rangeEndUs}`));
    Object.entries(misc).forEach(([k, v]) => lines.push(`set ${k} = ${v}`));
    $('dumpOutput').value = lines.join('\n');
});

$('copyDumpBtn').addEventListener('click', () => {
    $('dumpOutput').select();
    document.execCommand('copy');
});

// ---------------------------------------------------------------------------
// Generic Read/Write/Save button wiring
// ---------------------------------------------------------------------------

const TAB_HANDLERS = {
    configuration: { load: loadMisc, write: writeMisc },
    pid: { load: loadPid, write: writePid },
    rates: { load: loadRates, write: writeRates },
    receiver: { load: loadMisc, write: writeMisc },
    modes: { load: loadModesTab, write: writeModesTab },
    failsafe: { load: loadMisc, write: writeMisc },
    blackbox: { load: async () => { await loadMisc(); await loadBlackboxInfo(); }, write: writeMisc },
};

document.querySelectorAll('.load-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
        if (!msp.isConnected()) { alert('Connect first'); return; }
        const handler = TAB_HANDLERS[btn.dataset.target];
        try { await handler.load(); } catch (e) { alert('Read failed: ' + e.message); }
    });
});

document.querySelectorAll('.write-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
        if (!msp.isConnected()) { alert('Connect first'); return; }
        const handler = TAB_HANDLERS[btn.dataset.target];
        try { await handler.write(); } catch (e) { alert('Write failed: ' + e.message); }
    });
});

document.querySelectorAll('.save-btn').forEach((btn) => {
    btn.addEventListener('click', async () => {
        if (!msp.isConnected()) { alert('Connect first'); return; }
        try {
            await api.saveSettings();
            alert('Saved to flash.');
        } catch (e) {
            alert('Save failed: ' + e.message);
        }
    });
});
