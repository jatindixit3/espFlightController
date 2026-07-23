// MSP-style binary protocol client over the Web Serial API.
// Wire format and command IDs must match firmware/src/msp.cpp exactly -
// see docs/PROTOCOL.md for the authoritative reference.

const MSP = {
    IDENTIFY: 100,
    STATUS: 101,
    RAW_IMU: 102,
    ATTITUDE: 103,
    RC: 104,
    MOTOR: 105,
    PID: 106,
    SET_PID: 107,
    RATES: 108,
    SET_RATES: 109,
    MODES: 110,
    SET_MODES: 111,
    MISC: 112,
    SET_MISC: 113,
    CALIBRATE_GYRO: 114,
    MOTOR_TEST: 115,
    ESC_TELEMETRY: 116,
    BLACKBOX_INFO: 117,
    BLACKBOX_READ: 118,
    BLACKBOX_ERASE: 119,
    SAVE_SETTINGS: 120,
    RESET_DEFAULTS: 121,
    REBOOT: 122,
    SET_MOTOR_DIRECTION: 123,
};

class BinWriter {
    constructor() { this.bytes = []; }
    u8(v) { this.bytes.push(v & 0xFF); }
    i8(v) { this.bytes.push(v & 0xFF); }
    u16(v) {
        const b = new Uint8Array(2);
        new DataView(b.buffer).setUint16(0, v, true);
        this.bytes.push(b[0], b[1]);
    }
    u32(v) {
        const b = new Uint8Array(4);
        new DataView(b.buffer).setUint32(0, v >>> 0, true);
        this.bytes.push(b[0], b[1], b[2], b[3]);
    }
    f32(v) {
        const b = new Uint8Array(4);
        new DataView(b.buffer).setFloat32(0, v, true);
        this.bytes.push(b[0], b[1], b[2], b[3]);
    }
    toUint8Array() { return new Uint8Array(this.bytes); }
}

class BinReader {
    constructor(bytes) {
        this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
        this.pos = 0;
    }
    u8() { const v = this.view.getUint8(this.pos); this.pos += 1; return v; }
    i8() { const v = this.view.getInt8(this.pos); this.pos += 1; return v; }
    u16() { const v = this.view.getUint16(this.pos, true); this.pos += 2; return v; }
    u32() { const v = this.view.getUint32(this.pos, true); this.pos += 4; return v; }
    f32() { const v = this.view.getFloat32(this.pos, true); this.pos += 4; return v; }
    remainingBytes() { return this.view.byteLength - this.pos; }
}

class MspClient {
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.connected = false;
        this.pendingRequests = [];
        this._resetParser();
        this.onDisconnect = null;
    }

    _resetParser() {
        this.parseState = 'IDLE';
        this.expectedSize = 0;
        this.cmd = 0;
        this.payload = [];
        this.checksum = 0;
    }

    isConnected() { return this.connected; }

    async connect() {
        if (!('serial' in navigator)) {
            throw new Error('Web Serial API not available - use Chrome or Edge, and serve this page over http(s) or localhost, not a plain double-clicked file.');
        }
        this.port = await navigator.serial.requestPort();
        await this.port.open({ baudRate: 115200 });
        this.writer = this.port.writable.getWriter();
        this.connected = true;
        this._resetParser();
        this._readLoop();
    }

    async disconnect() {
        this.connected = false;
        for (const req of this.pendingRequests) {
            clearTimeout(req.timeoutId);
            req.reject(new Error('Disconnected'));
        }
        this.pendingRequests = [];
        try { await this.reader?.cancel(); } catch (e) { /* ignore */ }
        try { this.writer?.releaseLock(); } catch (e) { /* ignore */ }
        try { await this.port?.close(); } catch (e) { /* ignore */ }
        this.port = null;
    }

    async _readLoop() {
        this.reader = this.port.readable.getReader();
        try {
            while (this.connected) {
                const { value, done } = await this.reader.read();
                if (done) break;
                if (value) {
                    for (let i = 0; i < value.length; i++) this._feedByte(value[i]);
                }
            }
        } catch (e) {
            console.error('Serial read loop ended:', e);
        } finally {
            try { this.reader.releaseLock(); } catch (e) { /* ignore */ }
            if (this.connected) {
                // Port dropped unexpectedly (unplugged, etc).
                this.connected = false;
                if (this.onDisconnect) this.onDisconnect();
            }
        }
    }

    _feedByte(b) {
        switch (this.parseState) {
            case 'IDLE':
                this.parseState = (b === 0x24 /* $ */) ? 'DOLLAR' : 'IDLE';
                break;
            case 'DOLLAR':
                this.parseState = (b === 0x4D /* M */) ? 'M' : 'IDLE';
                break;
            case 'M':
                this.parseState = (b === 0x3E /* > */) ? 'DIR' : 'IDLE';
                break;
            case 'DIR':
                this.expectedSize = b;
                this.checksum = b;
                this.parseState = 'SIZE';
                break;
            case 'SIZE':
                this.cmd = b;
                this.checksum ^= b;
                this.payload = [];
                this.parseState = (this.expectedSize === 0) ? 'PAYLOAD_DONE' : 'PAYLOAD';
                break;
            case 'PAYLOAD':
                this.payload.push(b);
                this.checksum ^= b;
                if (this.payload.length >= this.expectedSize) this.parseState = 'PAYLOAD_DONE';
                break;
            case 'PAYLOAD_DONE':
                if (b === (this.checksum & 0xFF)) {
                    this._handleFrame(this.cmd, new Uint8Array(this.payload));
                }
                this.parseState = 'IDLE';
                break;
        }
    }

    _handleFrame(cmd, payload) {
        const idx = this.pendingRequests.findIndex((r) => r.cmd === cmd);
        if (idx >= 0) {
            const req = this.pendingRequests.splice(idx, 1)[0];
            clearTimeout(req.timeoutId);
            req.resolve(payload);
        }
        // Unmatched/late frame - dropped silently.
    }

    async sendCommand(cmd, payload = new Uint8Array(0), timeoutMs = 1500) {
        if (!this.connected) throw new Error('Not connected');
        const size = payload.length;
        let checksum = (size ^ cmd) & 0xFF;
        for (let i = 0; i < payload.length; i++) checksum ^= payload[i];

        const frame = new Uint8Array(5 + size);
        frame[0] = 0x24; frame[1] = 0x4D; frame[2] = 0x3C; // '$','M','<'
        frame[3] = size;
        frame[4] = cmd;
        frame.set(payload, 5);
        frame[5 + size] = checksum & 0xFF;

        return new Promise((resolve, reject) => {
            const entry = { cmd, resolve, reject, timeoutId: null };
            entry.timeoutId = setTimeout(() => {
                const idx = this.pendingRequests.indexOf(entry);
                if (idx >= 0) this.pendingRequests.splice(idx, 1);
                reject(new Error(`MSP command ${cmd} timed out`));
            }, timeoutMs);
            this.pendingRequests.push(entry);
            this.writer.write(frame).catch((e) => {
                const idx = this.pendingRequests.indexOf(entry);
                if (idx >= 0) this.pendingRequests.splice(idx, 1);
                clearTimeout(entry.timeoutId);
                reject(e);
            });
        });
    }
}
