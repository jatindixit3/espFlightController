// Typed high-level API wrapping MspClient - one method per command, matching
// docs/PROTOCOL.md exactly. UI code (app.js) should only ever talk to this,
// never build raw MSP frames itself.

class FcApi {
    constructor(mspClient) {
        this.msp = mspClient;
    }

    async identify() {
        const p = await this.msp.sendCommand(MSP.IDENTIFY);
        return new TextDecoder().decode(p);
    }

    async getStatus() {
        const r = new BinReader(await this.msp.sendCommand(MSP.STATUS));
        return {
            armed: r.u8() !== 0,
            failsafe: r.u8() !== 0,
            mode: r.u8(),
            gyroCalibrated: r.u8() !== 0,
            batteryVoltage: r.u16() / 100.0,
            totalCurrent: r.u16() / 100.0,
        };
    }

    async getRawImu() {
        const r = new BinReader(await this.msp.sendCommand(MSP.RAW_IMU));
        return {
            gyroDegS: [r.f32(), r.f32(), r.f32()],
            accelG: [r.f32(), r.f32(), r.f32()],
        };
    }

    async getAttitude() {
        const r = new BinReader(await this.msp.sendCommand(MSP.ATTITUDE));
        return { roll: r.f32(), pitch: r.f32(), yaw: r.f32() };
    }

    async getRc() {
        const r = new BinReader(await this.msp.sendCommand(MSP.RC));
        const channels = [];
        for (let i = 0; i < 16; i++) channels.push(r.u16());
        return channels;
    }

    async getPid() {
        const r = new BinReader(await this.msp.sendCommand(MSP.PID));
        const axes = [];
        for (let i = 0; i < 3; i++) axes.push({ P: r.f32(), I: r.f32(), D: r.f32(), FF: r.f32() });
        return {
            axes,
            dtermLowpassHz: r.f32(),
            gyroLowpassHz: r.f32(),
            rpmFilterEnabled: r.u8() !== 0,
            levelGainP: r.f32(),
            horizonTiltEffect: r.f32(),
            maxAngleDeg: r.f32(),
        };
    }

    async setPid(pid) {
        const w = new BinWriter();
        for (const axis of pid.axes) { w.f32(axis.P); w.f32(axis.I); w.f32(axis.D); w.f32(axis.FF); }
        w.f32(pid.dtermLowpassHz);
        w.f32(pid.gyroLowpassHz);
        w.u8(pid.rpmFilterEnabled ? 1 : 0);
        w.f32(pid.levelGainP);
        w.f32(pid.horizonTiltEffect);
        w.f32(pid.maxAngleDeg);
        await this.msp.sendCommand(MSP.SET_PID, w.toUint8Array());
    }

    async getRates() {
        const r = new BinReader(await this.msp.sendCommand(MSP.RATES));
        const axes = [];
        for (let i = 0; i < 3; i++) axes.push({ rcRate: r.f32(), superRate: r.f32(), expo: r.f32() });
        return axes;
    }

    async setRates(axes) {
        const w = new BinWriter();
        for (const a of axes) { w.f32(a.rcRate); w.f32(a.superRate); w.f32(a.expo); }
        await this.msp.sendCommand(MSP.SET_RATES, w.toUint8Array());
    }

    async getModes() {
        const r = new BinReader(await this.msp.sendCommand(MSP.MODES));
        const modes = [];
        for (let i = 0; i < 4; i++) modes.push({ channel: r.i8(), rangeStartUs: r.u16(), rangeEndUs: r.u16() });
        return modes; // index 0=ARM, 1=ANGLE, 2=HORIZON, 3=BLACKBOX
    }

    async setModes(modes) {
        const w = new BinWriter();
        for (const m of modes) { w.i8(m.channel); w.u16(m.rangeStartUs); w.u16(m.rangeEndUs); }
        await this.msp.sendCommand(MSP.SET_MODES, w.toUint8Array());
    }

    async getMisc() {
        const r = new BinReader(await this.msp.sendCommand(MSP.MISC));
        return {
            rxMinUs: r.u16(), rxMidUs: r.u16(), rxMaxUs: r.u16(),
            motorIdlePercent: r.f32(),
            batteryCellOverride: r.i8(),
            batteryWarnV: r.f32(),
            batteryCritV: r.f32(),
            blackboxRateDivider: r.u8(),
            motorPolePairs: r.u8(),
            failsafeTimeoutMs: r.u32(),
            boardAlignRollDeg: r.f32(),
            boardAlignPitchDeg: r.f32(),
            boardAlignYawDeg: r.f32(),
            bidirDshotEnabled: r.u8() !== 0,
        };
    }

    async setMisc(m) {
        const w = new BinWriter();
        w.u16(m.rxMinUs); w.u16(m.rxMidUs); w.u16(m.rxMaxUs);
        w.f32(m.motorIdlePercent);
        w.i8(m.batteryCellOverride);
        w.f32(m.batteryWarnV);
        w.f32(m.batteryCritV);
        w.u8(m.blackboxRateDivider);
        w.u8(m.motorPolePairs);
        w.u32(m.failsafeTimeoutMs);
        w.f32(m.boardAlignRollDeg);
        w.f32(m.boardAlignPitchDeg);
        w.f32(m.boardAlignYawDeg);
        w.u8(m.bidirDshotEnabled ? 1 : 0);
        await this.msp.sendCommand(MSP.SET_MISC, w.toUint8Array());
    }

    // Calibration blocks the FC for up to ~12s - use a long timeout.
    async calibrateGyro() {
        const r = new BinReader(await this.msp.sendCommand(MSP.CALIBRATE_GYRO, new Uint8Array(0), 15000));
        return r.u8() !== 0;
    }

    async motorTest(throttles) {
        const w = new BinWriter();
        for (const t of throttles) w.u16(t);
        await this.msp.sendCommand(MSP.MOTOR_TEST, w.toUint8Array());
    }

    async getEscTelemetry() {
        const r = new BinReader(await this.msp.sendCommand(MSP.ESC_TELEMETRY));
        const motors = [];
        for (let i = 0; i < 4; i++) {
            motors.push({
                tempC: r.u8(),
                voltage: r.f32(),
                current: r.f32(),
                consumptionMah: r.u16(),
                eRpm: r.u32(),
                lastUpdateMs: r.u32(),
                bidirErpm: r.u32(),
                bidirLastUpdateMs: r.u32(),
            });
        }
        return motors;
    }

    async getBlackboxInfo() {
        const r = new BinReader(await this.msp.sendCommand(MSP.BLACKBOX_INFO));
        return { writeOffset: r.u32(), partitionSize: r.u32() };
    }

    async readBlackboxChunk(offset, length) {
        const w = new BinWriter();
        w.u32(offset);
        w.u16(length);
        const payload = await this.msp.sendCommand(MSP.BLACKBOX_READ, w.toUint8Array());
        return payload; // raw bytes, Uint8Array
    }

    async eraseBlackbox() {
        await this.msp.sendCommand(MSP.BLACKBOX_ERASE);
    }

    async saveSettings() {
        await this.msp.sendCommand(MSP.SAVE_SETTINGS);
    }

    async resetDefaults() {
        await this.msp.sendCommand(MSP.RESET_DEFAULTS);
    }

    async reboot() {
        await this.msp.sendCommand(MSP.REBOOT).catch(() => {});
    }
}
