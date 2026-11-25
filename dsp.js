// Signal processing shared by the page and the offline simulation:
// FFT, calibration sweep, secondary-path analysis, peak picking, tone tracking.
(function (root) {
  'use strict';
  const TWO_PI = 2 * Math.PI;

  // In-place iterative radix-2 complex FFT. inverse=true scales by 1/n.
  function fft(re, im, inverse) {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        let t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (let len = 2; len <= n; len <<= 1) {
      const half = len >> 1;
      const ang = (inverse ? TWO_PI : -TWO_PI) / len;
      const wlr = Math.cos(ang), wli = Math.sin(ang);
      for (let i = 0; i < n; i += len) {
        let wr = 1, wi = 0;
        for (let j = 0; j < half; j++) {
          const a = i + j, b = a + half;
          const vr = re[b] * wr - im[b] * wi;
          const vi = re[b] * wi + im[b] * wr;
          re[b] = re[a] - vr; im[b] = im[a] - vi;
          re[a] += vr; im[a] += vi;
          const nwr = wr * wlr - wi * wli;
          wi = wr * wli + wi * wlr;
          wr = nwr;
        }
      }
    }
    if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
  }

  // Exponential sine sweep with short fades. More energy per Hz at low frequencies,
  // which is where the drone lives.
  function makeSweep(sr, opts) {
    const o = Object.assign({ f1: 20, f2: 2000, seconds: 3, amp: 0.3, fade: 0.05 }, opts);
    const f2 = Math.min(o.f2, 0.45 * sr);
    const n = Math.round(o.seconds * sr);
    const L = o.seconds / Math.log(f2 / o.f1);
    const K = TWO_PI * o.f1 * L;
    const nf = Math.round(o.fade * sr);
    const x = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const t = i / sr;
      let env = 1;
      if (i < nf) env = 0.5 - 0.5 * Math.cos(Math.PI * i / nf);
      else if (i > n - nf) env = 0.5 - 0.5 * Math.cos(Math.PI * (n - i) / nf);
      x[i] = o.amp * env * Math.sin(K * (Math.exp(t / L) - 1));
    }
    return x;
  }

  // Measures the path from the app's output to the mic (Bluetooth + stereo + cabin)
  // from one or more recordings of the sweep. Returns a delay plus a smooth
  // frequency-response table, so S(f) = table(f) * exp(-i 2π f tau).
  function analyzeCalibration(sweep, recs, sr) {
    const len = recs[0].length;
    let N = 1;
    while (N < len) N <<= 1;
    const binHz = sr / N;
    const kmax = Math.min(N / 2, Math.ceil(2100 / binHz));

    const xr = new Float64Array(N), xi = new Float64Array(N);
    xr.set(sweep);
    fft(xr, xi, false);
    let maxX = 0;
    for (let k = 0; k <= kmax; k++) maxX = Math.max(maxX, xr[k] * xr[k] + xi[k] * xi[k]);
    const eps = 1e-4 * maxX;

    // Band taper 12–20 Hz up, 1.5–2 kHz down, so the impulse response doesn't ring.
    const fTop = Math.min(2000, 0.44 * sr);
    const band = (f) => {
      if (f < 12 || f > fTop) return 0;
      if (f < 20) return Math.pow(Math.sin(0.5 * Math.PI * (f - 12) / 8), 2);
      if (f > fTop - 500) return Math.pow(Math.cos(0.5 * Math.PI * (f - (fTop - 500)) / 500), 2);
      return 1;
    };

    // Per pass: one-sided (analytic) deconvolved spectrum.
    const passes = recs.map((rec) => {
      const yr = new Float64Array(N), yi = new Float64Array(N);
      yr.set(rec);
      fft(yr, yi, false);
      const hr = new Float64Array(N), hi = new Float64Array(N);
      for (let k = 1; k <= kmax; k++) {
        const w = band(k * binHz);
        if (!w) continue;
        const d = xr[k] * xr[k] + xi[k] * xi[k] + eps;
        hr[k] = w * (yr[k] * xr[k] + yi[k] * xi[k]) / d;
        hi[k] = w * (yi[k] * xr[k] - yr[k] * xi[k]) / d;
      }
      fft(hr, hi, true); // → complex impulse response
      return { hr, hi };
    });

    // Each pass's arrival: envelope peak of its impulse response, and how far that peak
    // stands above the rest of the response (SNR). Bluetooth can shift the delay by a few
    // milliseconds between passes, so each pass is aligned on its own peak.
    const searchMax = Math.min(N - 2, Math.round(Math.max(0.2, (len - sweep.length) / sr - 0.1) * sr));
    for (const p of passes) {
      const envs = new Float64Array(searchMax);
      let ip = 1, best = -1;
      for (let i = 1; i < searchMax; i++) {
        const m = Math.hypot(p.hr[i], p.hi[i]);
        envs[i] = m;
        if (m > best) { best = m; ip = i; }
      }
      const sorted = Array.from(envs.subarray(1)).sort((x, y) => x - y);
      p.peak = ip;
      p.snrDb = 20 * Math.log10(best / (sorted[sorted.length >> 1] + 1e-30));
    }
    const order = passes.map((p) => p.peak).sort((x, y) => x - y);
    const i0 = order[order.length >> 1];

    // Keep only 30 ms before to 250 ms after each arrival, moved to the common delay.
    // Noise and the drone are spread over the whole recording; the stereo's response is not.
    const wa = Math.round(0.03 * sr), wb = Math.round(0.25 * sr), wt = Math.round(0.01 * sr);
    const Hp = passes.map((p) => {
      const r = new Float64Array(N), im_ = new Float64Array(N);
      for (let d = -wa; d <= wb; d++) {
        const src = ((p.peak + d) % N + N) % N;
        const dst = ((i0 + d) % N + N) % N;
        let w = 1;
        if (d < -wa + wt) w = 0.5 - 0.5 * Math.cos(Math.PI * (d + wa) / wt);
        else if (d > wb - wt) w = 0.5 - 0.5 * Math.cos(Math.PI * (wb - d) / wt);
        r[dst] = p.hr[src] * w; im_[dst] = p.hi[src] * w;
      }
      return { r, im_ };
    });
    // Sub-sample coarse delay from the aligned average envelope.
    const env = (i) => {
      let a = 0, b = 0;
      for (const h of Hp) { a += h.r[i]; b += h.im_[i]; }
      return Math.hypot(a, b);
    };
    const m0 = env(i0 - 1), m1 = env(i0), m2 = env(i0 + 1);
    const den = m0 - 2 * m1 + m2;
    const tau0 = (i0 + (den !== 0 ? 0.5 * (m0 - m2) / den : 0)) / sr;
    for (const h of Hp) { fft(h.r, h.im_, false); h.hr = h.r; h.hi = h.im_; }
    const P = Hp.length;
    const Hr = new Float64Array(kmax + 1), Hi = new Float64Array(kmax + 1);
    for (const h of Hp) for (let k = 0; k <= kmax; k++) { Hr[k] += h.hr[k] / P; Hi[k] += h.hi[k] / P; }

    // Noise per bin from the spread between passes, smoothed over ±2 Hz.
    const rawVar = new Float64Array(kmax + 1);
    for (let k = 0; k <= kmax; k++) {
      let v = 0;
      for (const h of Hp) v += Math.pow(h.hr[k] - Hr[k], 2) + Math.pow(h.hi[k] - Hi[k], 2);
      rawVar[k] = P > 1 ? v / (P - 1) / P : 0;
    }
    const half = Math.max(1, Math.round(2 / binHz));
    const noiseVar = new Float64Array(kmax + 1);
    let sumMag = 0, cntMag = 0;
    for (let k = 0; k <= kmax; k++) {
      let s = 0, c = 0;
      for (let d = -half; d <= half; d++) { const kk = k + d; if (kk >= 0 && kk <= kmax) { s += rawVar[kk]; c++; } }
      noiseVar[k] = s / c;
      const f = k * binHz;
      if (f >= 40 && f <= 300) { sumMag += Hr[k] * Hr[k] + Hi[k] * Hi[k]; cntMag++; }
    }
    const varFloor = 1e-4 * (sumMag / Math.max(1, cntMag)) + 1e-20;
    const weight = (k) => 1 / (noiseVar[k] + varFloor);

    // Refine with the phase slope in the drone band (40–300 Hz), where speaker and
    // subwoofer delay can differ from the broadband figure.
    const rot = (k, tau) => {
      const ang = TWO_PI * k * binHz * tau;
      const c = Math.cos(ang), s = Math.sin(ang);
      return [Hr[k] * c - Hi[k] * s, Hr[k] * s + Hi[k] * c];
    };
    let accR = 0, accI = 0;
    const kLo = Math.ceil(40 / binHz), kHi = Math.min(kmax - 1, Math.floor(300 / binHz));
    let prev = rot(kLo, tau0);
    for (let k = kLo + 1; k <= kHi; k++) {
      const cur = rot(k, tau0);
      const w = Math.min(weight(k), weight(k - 1));
      accR += w * (cur[0] * prev[0] + cur[1] * prev[1]);
      accI += w * (cur[1] * prev[0] - cur[0] * prev[1]);
      prev = cur;
    }
    const slope = Math.atan2(accI, accR) / binHz; // rad per Hz
    const tau = tau0 - slope / TWO_PI;

    // Smooth table of the delay-free response, 10–500 Hz in 1 Hz steps, noise-weighted
    // so bins swamped by the drone are filled from their neighbours.
    const f0 = 10, df = 1, n = 491, sigma = 2;
    const re = new Array(n), im = new Array(n);
    const kSpan = Math.ceil(3 * sigma / binHz);
    for (let g = 0; g < n; g++) {
      const fc = f0 + g * df;
      const kc = Math.round(fc / binHz);
      let sr_ = 0, si_ = 0, sw = 0;
      for (let k = Math.max(1, kc - kSpan); k <= Math.min(kmax, kc + kSpan); k++) {
        const x = (k * binHz - fc) / sigma;
        const w = Math.exp(-0.5 * x * x) * weight(k);
        const r = rot(k, tau);
        sr_ += w * r[0]; si_ += w * r[1]; sw += w;
      }
      re[g] = sw ? sr_ / sw : 0;
      im[g] = sw ? si_ / sw : 0;
    }

    // Agreement between passes over the drone band (1 = identical), noise-weighted.
    let quality = 1;
    if (P > 1) {
      let cr = 0, ci = 0, p1 = 0, p2 = 0;
      const a = Hp[0], b = Hp[1];
      for (let k = kLo; k <= kHi; k++) {
        const w = weight(k);
        cr += w * (a.hr[k] * b.hr[k] + a.hi[k] * b.hi[k]);
        ci += w * (a.hi[k] * b.hr[k] - a.hr[k] * b.hi[k]);
        p1 += w * (a.hr[k] * a.hr[k] + a.hi[k] * a.hi[k]);
        p2 += w * (b.hr[k] * b.hr[k] + b.hi[k] * b.hi[k]);
      }
      quality = Math.hypot(cr, ci) / Math.sqrt(p1 * p2 + 1e-30);
    }

    const mags = [];
    for (let g = 0; g < n; g++) { const fc = f0 + g * df; if (fc >= 40 && fc <= 300) mags.push(Math.hypot(re[g], im[g])); }
    mags.sort((x, y) => x - y);
    const level = mags[mags.length >> 1] || 0;

    const passInfo = passes.map((p) => ({ delayMs: +(1000 * p.peak / sr).toFixed(1), snrDb: +p.snrDb.toFixed(1) }));
    const snrs = passInfo.map((p) => p.snrDb).sort((x, y) => x - y);
    return {
      tau, tauCoarse: tau0, f0, df, re, im, quality, level, minMag2: Math.pow(0.1 * level, 2), sampleRate: sr,
      passes: passInfo, snrDb: snrs[snrs.length >> 1],
    };
  }

  // Strongest narrow peaks in a dB spectrum between fmin and fmax.
  function findPeaks(db, binHz, fmin, fmax, maxN, minSep, promDb) {
    const i0 = Math.max(3, Math.ceil(fmin / binHz));
    const i1 = Math.min(db.length - 4, Math.floor(fmax / binHz));
    if (i1 <= i0) return [];
    const band = [];
    for (let i = i0; i <= i1; i++) if (Number.isFinite(db[i])) band.push(db[i]);
    if (band.length < 5) return [];
    band.sort((a, b) => a - b);
    const floor = band[band.length >> 1];
    const cands = [];
    for (let i = i0; i <= i1; i++) {
      const v = db[i];
      if (!Number.isFinite(v) || v - floor < promDb) continue;
      if (!(v > db[i - 1] && v >= db[i + 1])) continue;
      let isMax = true;
      for (let d = 2; d <= 3; d++) if (db[i - d] > v || db[i + d] > v) { isMax = false; break; }
      if (!isMax) continue;
      const a = db[i - 1], c = db[i + 1];
      const den = a - 2 * v + c;
      const p = Number.isFinite(den) && den !== 0 ? Math.max(-0.5, Math.min(0.5, 0.5 * (a - c) / den)) : 0;
      cands.push({ f: (i + p) * binHz, level: v - 0.25 * (a - c) * p, prominence: v - floor });
    }
    cands.sort((x, y) => y.level - x.level);
    const out = [];
    for (const c of cands) {
      if (out.every((o) => Math.abs(o.f - c.f) >= minSep)) out.push(c);
      if (out.length >= maxN) break;
    }
    return out;
  }

  // Keeps up to 3 tones locked to spectral peaks. Returns messages for the worklet.
  class ToneTracker {
    constructor() { this.reset(); }
    reset() { this.t = [0, 1, 2].map(() => ({ active: false, f: 0, miss: 0 })); }
    update(peaks, nTones, oscF) {
      const msgs = [];
      const used = new Set();
      for (let i = 0; i < 3; i++) {
        const t = this.t[i];
        if (i >= nTones) {
          if (t.active) { t.active = false; msgs.push({ i, active: false }); }
          continue;
        }
        if (!t.active) continue;
        const cur = (oscF && oscF[i]) || t.f;
        let best = -1, bd = Infinity;
        peaks.forEach((p, k) => {
          if (used.has(k)) return;
          const d = Math.abs(p.f - cur);
          if (d < bd) { bd = d; best = k; }
        });
        if (best >= 0 && bd < 3) {
          used.add(best);
          t.miss = 0;
          if (bd > 1.0) { t.f = peaks[best].f; msgs.push({ i, f: t.f, resetFine: true }); }
          else t.f = cur;
        } else if (++t.miss > 6) {
          t.active = false;
          msgs.push({ i, active: false });
        }
      }
      for (let i = 0; i < nTones; i++) {
        const t = this.t[i];
        if (t.active) continue;
        const k = peaks.findIndex((p, idx) => !used.has(idx) &&
          this.t.every((o) => !o.active || Math.abs(o.f - p.f) >= 3));
        if (k < 0) break;
        used.add(k);
        t.active = true; t.f = peaks[k].f; t.miss = 0;
        msgs.push({ i, active: true, f: t.f, reset: true, resetFine: true });
      }
      return msgs;
    }
  }

  // Calibration recipe shared by the page and the simulation.
  const CAL = { passes: 3, gapSeconds: 0.4, tailSeconds: 1.5 };

  const api = { fft, makeSweep, analyzeCalibration, findPeaks, ToneTracker, CAL };
  root.DSP = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
