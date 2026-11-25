// Re-runs calibration analysis on a recording saved from the page ("Save recording").
// node test/analyze-wav.cjs drone-cal-48000-3pass.wav
'use strict';
const fs = require('fs');
const DSP = require('../dsp.js');
const b = fs.readFileSync(process.argv[2]);
const sr = b.readUInt32LE(24);
let off = 12, dataOff = -1, dataLen = 0;
while (off < b.length) { const id = b.toString('ascii', off, off + 4), len = b.readUInt32LE(off + 4); if (id === 'data') { dataOff = off + 8; dataLen = len; break; } off += 8 + len; }
const n = dataLen / 2, x = new Float32Array(n);
for (let i = 0; i < n; i++) x[i] = b.readInt16LE(dataOff + 2 * i) / 32767;
const sweep = DSP.makeSweep(sr);
const recLen = sweep.length + Math.round(DSP.CAL.tailSeconds * sr);
const recs = [];
for (let p = 0; (p + 1) * recLen <= n; p++) recs.push(x.subarray(p * recLen, (p + 1) * recLen));
const r = DSP.analyzeCalibration(sweep, recs, sr);
const db = (a, i0, i1) => { let e = 0; for (let i = i0; i < i1; i++) e += a[i] * a[i]; return (10 * Math.log10(e / (i1 - i0) + 1e-20)).toFixed(1); };
console.log(`${sr} Hz, ${recs.length} passes; delay ${(r.tau * 1000).toFixed(1)} ms, clarity ${r.snrDb} dB, agree ${r.quality.toFixed(3)}, level ${r.level.toExponential(2)}`);
recs.forEach((rec, i) => console.log(`pass ${i + 1}: ${JSON.stringify(r.passes[i])} mic ${db(rec, Math.round(0.3 * sr), sweep.length)} dBFS during sweep vs ${db(rec, recLen - Math.round(0.6 * sr), recLen)} after; peak ${rec.reduce((m, v) => Math.max(m, Math.abs(v)), 0).toFixed(3)}`));
