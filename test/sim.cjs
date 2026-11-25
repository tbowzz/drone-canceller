// Offline simulation of the worklet + page logic against a synthetic car.
// node test/sim.cjs [--tones 2] [--delay 0.2317] [--gainAfterCal 1] [--seconds 60] [--sr 48000]
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const DSP = require('../dsp.js');

const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, arr) => {
  if (a.startsWith('--')) acc.push([a.slice(2), Number(arr[i + 1])]);
  return acc;
}, []));
const SR = args.sr || 48000;
const DELAY = args.delay ?? 0.2317;
const TONES = args.tones || 2;
const SECONDS = args.seconds || 60;
const GAIN_AFTER = args.gainAfterCal ?? 1;
const CAL_DRONE = (args.calDrone ?? 1) !== 0;
const DRIFT = args.drift ?? 3; // Hz of slow pitch wobble

// --- load worklet in a sandbox ---
let Proc = null;
const outbox = [];
const ctx = {
  sampleRate: SR,
  AudioWorkletProcessor: class { constructor() { this.port = { postMessage: (m) => outbox.push(m), onmessage: null }; } },
  registerProcessor: (_n, c) => { Proc = c; },
  console,
};
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'anc-worklet.js'), 'utf8'), ctx);
const proc = new Proc();
const send = (m) => proc.port.onmessage({ data: m });

// --- synthetic car ---
let rngState = 12345;
const rand = () => { rngState = (rngState * 1664525 + 1013904223) >>> 0; return rngState / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(rand() + 1e-12)) * Math.cos(2 * Math.PI * rand());
const D = Math.round(DELAY * SR);
const dl = new Float32Array(D + 1); let dlPos = 0;
// 2nd-order high-pass at 60 Hz (speaker roll-off) → adds frequency-dependent phase
function biquadHP(fc, q) {
  const w = 2 * Math.PI * fc / SR, al = Math.sin(w) / (2 * q), cw = Math.cos(w);
  const b0 = (1 + cw) / 2, b1 = -(1 + cw), b2 = (1 + cw) / 2, a0 = 1 + al, a1 = -2 * cw, a2 = 1 - al;
  const B = [b0 / a0, b1 / a0, b2 / a0], A = [a1 / a0, a2 / a0];
  let x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  return (x) => { const y = B[0] * x + B[1] * x1 + B[2] * x2 - A[0] * y1 - A[1] * y2; x2 = x1; x1 = x; y2 = y1; y1 = y; return y; };
}
const hp = biquadHP(60, 0.707);
let pathGain = 0.5;
let lpNoise = 0;
const lpA = 1 - Math.exp(-2 * Math.PI * 100 / SR);
let droneTheta = 0, t = 0;
function droneFreq(time) { return 92 + DRIFT * Math.sin(2 * Math.PI * time / 40); }
function micSample(yOut, droneOn) {
  dl[dlPos] = yOut; dlPos = (dlPos + 1) % dl.length;
  const delayed = dl[dlPos];
  const speaker = pathGain * hp(delayed);
  const f = droneFreq(t);
  droneTheta += 2 * Math.PI * f / SR;
  const drone = droneOn ? 0.2 * Math.sin(droneTheta + 0.7) + 0.07 * Math.sin(2 * droneTheta + 2.1) : 0;
  lpNoise += lpA * (gauss() * 0.35 - lpNoise);
  const noise = lpNoise + 0.01 * gauss();
  t += 1 / SR;
  return { mic: speaker + drone + noise, drone, theta: droneTheta };
}

// --- run helpers ---
const B = 128;
let lastOut = new Float32Array(B);
function runBlock(droneOn, record) {
  const mic = new Float32Array(B);
  const info = [];
  for (let j = 0; j < B; j++) {
    const s = micSample(lastOut[j], droneOn);
    mic[j] = s.mic;
    info.push(s);
  }
  const out0 = new Float32Array(B), out1 = new Float32Array(B);
  proc.process([[mic]], [[out0], [out1]]);
  lastOut = out0;
  if (record) record(mic, out0, out1, info);
}

// --- calibration (drone on, like calibrating while idling) ---
const sweep = DSP.makeSweep(SR);
const recLen = sweep.length + Math.round(DSP.CAL.tailSeconds * SR);
const recs = [];
for (let p = 0; p < (args.passes || DSP.CAL.passes); p++) {
  send({ type: 'calibrate', sweep, recLength: recLen });
  while (true) {
    runBlock(CAL_DRONE);
    const i = outbox.findIndex((m) => m.type === 'calRec');
    if (i >= 0) { recs.push(outbox.splice(i, 1)[0].rec); break; }
  }
  for (let k = 0; k < Math.round(DSP.CAL.gapSeconds * SR / B); k++) runBlock(CAL_DRONE);
}
const cal = DSP.analyzeCalibration(sweep, recs, SR);

// true S(f): delay + HP + gain
function trueS(f) {
  const w = 2 * Math.PI * f / SR;
  const z1 = [Math.cos(-w), Math.sin(-w)], z2 = [Math.cos(-2 * w), Math.sin(-2 * w)];
  const wc = 2 * Math.PI * 60 / SR, al = Math.sin(wc) / (2 * 0.707), cw = Math.cos(wc);
  const b = [(1 + cw) / 2, -(1 + cw), (1 + cw) / 2], a = [1 + al, -2 * cw, 1 - al];
  const num = [b[0] + b[1] * z1[0] + b[2] * z2[0], b[1] * z1[1] + b[2] * z2[1]];
  const den = [a[0] + a[1] * z1[0] + a[2] * z2[0], a[1] * z1[1] + a[2] * z2[1]];
  const dd = den[0] ** 2 + den[1] ** 2;
  const h = [(num[0] * den[0] + num[1] * den[1]) / dd, (num[1] * den[0] - num[0] * den[1]) / dd];
  const ang = -2 * Math.PI * f * (D + B) / SR; // + one audio block of I/O latency
  const dly = [Math.cos(ang), Math.sin(ang)];
  return [0.5 * (h[0] * dly[0] - h[1] * dly[1]), 0.5 * (h[0] * dly[1] + h[1] * dly[0])];
}
function estS(f) {
  const g = (f - cal.f0) / cal.df, i = Math.floor(g), u = g - i;
  const rr = cal.re[i] + (cal.re[i + 1] - cal.re[i]) * u, ri = cal.im[i] + (cal.im[i + 1] - cal.im[i]) * u;
  const ang = -2 * Math.PI * f * cal.tau;
  return [rr * Math.cos(ang) - ri * Math.sin(ang), rr * Math.sin(ang) + ri * Math.cos(ang)];
}
console.log(`calibration: tau=${(cal.tau * 1000).toFixed(2)} ms (true ${((D + B) / SR * 1000).toFixed(2)}), coarse=${(cal.tauCoarse * 1000).toFixed(2)} ms, quality=${cal.quality.toFixed(3)}, level=${cal.level.toFixed(3)}`);
for (const f of [50, 92, 184, 250]) {
  const a = trueS(f), b = estS(f);
  const phErr = Math.atan2(b[1] * a[0] - b[0] * a[1], b[0] * a[0] + b[1] * a[1]) * 180 / Math.PI;
  console.log(`  S(${f} Hz): phase error ${phErr.toFixed(1)}°, gain ratio ${(Math.hypot(...b) / Math.hypot(...a)).toFixed(2)}`);
}

// --- run cancellation with the page's tracker ---
pathGain *= GAIN_AFTER;
send({ type: 'setCal', cal });
send({ type: 'params', adaptSeconds: args.adapt || 2, maxOut: 0.9, hold: false, trimDb: 0, trimDeg: 0 });
send({ type: 'run', on: true });
const tracker = new DSP.ToneTracker();
const FFT_N = 32768;
const ring = new Float32Array(FFT_N); let ringPos = 0;
const smooth = new Float64Array(FFT_N / 2);
const win = new Float64Array(FFT_N);
for (let i = 0; i < FFT_N; i++) win[i] = 0.42 - 0.5 * Math.cos(2 * Math.PI * i / FFT_N) + 0.08 * Math.cos(4 * Math.PI * i / FFT_N);
let lastStats = null;
const blocksPerTrack = Math.round(0.25 * SR / B);
const totalBlocks = Math.round(SECONDS * SR / B);
const measureFrom = totalBlocks - Math.round(20 * SR / B);
// residual vs drone at the drone's fundamental and harmonic, via true phase demod
const acc = { e1: [0, 0], d1: [0, 0], e2: [0, 0], d2: [0, 0], n: 0 };
const lp = 1 - Math.exp(-2 * Math.PI * 1 / SR);
const pw = { e1: 0, d1: 0, e2: 0, d2: 0 };
const timeline = [];
for (let blk = 0; blk < totalBlocks; blk++) {
  runBlock(true, (mic, out0, out1, info) => {
    for (let j = 0; j < B; j++) { ring[ringPos] = out1[j]; ringPos = (ringPos + 1) % FFT_N; }
    for (let j = 0; j < B; j++) {
      const th = info[j].theta, e = mic[j], d = info[j].drone;
      for (const [h, ek, dk] of [[1, 'e1', 'd1'], [2, 'e2', 'd2']]) {
        const c = Math.cos(h * th), s = Math.sin(h * th);
        acc[ek][0] += lp * (e * c - acc[ek][0]); acc[ek][1] += lp * (-e * s - acc[ek][1]);
        acc[dk][0] += lp * (d * c - acc[dk][0]); acc[dk][1] += lp * (-d * s - acc[dk][1]);
      }
    }
  });
  while (outbox.length) {
    const m = outbox.shift();
    if (m.type === 'stats') lastStats = m;
    if (m.type === 'maxed') console.log(`  maxed warning tone ${m.i} at ${m.f.toFixed(1)} Hz, t=${(blk * B / SR).toFixed(1)}s`);
    if (m.type === 'diverged') console.log(`  diverged tone ${m.i} at ${m.f.toFixed(1)} Hz, t=${(blk * B / SR).toFixed(1)}s`);
  }
  if (blk >= measureFrom) {
    for (const k of ['e1', 'd1', 'e2', 'd2']) pw[k] += acc[k][0] ** 2 + acc[k][1] ** 2;
  }
  if (blk % blocksPerTrack === 0) {
    const re = new Float64Array(FFT_N), im = new Float64Array(FFT_N);
    for (let i = 0; i < FFT_N; i++) re[i] = ring[(ringPos + i) % FFT_N] * win[i];
    DSP.fft(re, im, false);
    const db = new Float32Array(FFT_N / 2);
    for (let k = 0; k < FFT_N / 2; k++) {
      const mag = Math.hypot(re[k], im[k]) / FFT_N;
      smooth[k] = 0.5 * smooth[k] + 0.5 * mag;
      db[k] = 20 * Math.log10(smooth[k] + 1e-12);
    }
    const peaks = DSP.findPeaks(db, SR / FFT_N, 30, 250, TONES, 8, 10);
    const oscF = lastStats ? lastStats.tones.map((x) => x.f) : null;
    for (const msg of tracker.update(peaks, TONES, oscF)) send({ type: 'tone', ...msg });
    if (blk % (blocksPerTrack * (args.every || 20)) === 0 && lastStats) {
      if (args.debug) { const h = proc.tones[0]; timeline.push(`   tone0 step=${h.step.toFixed(2)} f=${h.f.toFixed(2)} rate=${h.fRate.toFixed(3)}`); }
      timeline.push(`t=${(blk * B / SR).toFixed(0).padStart(2)}s drone=${droneFreq(t).toFixed(2)}Hz ` +
        lastStats.tones.filter((x) => x.active).map((x) => `[${x.f.toFixed(2)}Hz amp ${x.amp.toFixed(3)} red ${(x.dDb - x.eDb).toFixed(1)}dB meas ${x.baseDb === null ? "-" : (x.baseDb - x.eDb).toFixed(1)}dB]`).join(' '));
    }
  }
}
console.log(timeline.join('\n'));
const red = (e, d) => (10 * Math.log10(pw[d] / pw[e])).toFixed(1);
console.log(`measured reduction at mic over last 20 s: fundamental ${red('e1', 'd1')} dB, 2nd harmonic ${red('e2', 'd2')} dB`);
