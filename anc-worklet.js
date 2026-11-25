// Audio-thread processor. Input: mic. Output 0: anti-noise to the speakers.
// Output 1: estimate of what the mic would hear without the anti-noise (for display
// and frequency tracking).
//
// Each tone is a sine at the drone frequency with complex weight W. The mic signal is
// demodulated against the sine as it was one stereo delay ago (the copy now arriving
// at the mic), giving the residual E. W is nudged against E using the calibrated
// output-to-mic response R(f), so W settles at -D/R and the drone cancels at the mic.
// A frequency-locked loop that also learns the rate of pitch change keeps the sine
// arriving at the mic on the drone's pitch while the car speeds up or slows down.
// If the stereo is turned up after calibration, W overshoots and oscillates; a step-size
// controller detects that and slows adaptation until it settles.
'use strict';

const TWO_PI = 2 * Math.PI;
const MAX_TONES = 3;
const FLL_KP = 1.5;          // 1/s
const FLL_KI = 1.0;          // 1/s²
const RING_SECONDS = 1.6;    // longest stereo delay supported
const LEAK_SECONDS = 60;
const STEP_CHECK_SECONDS = 0.4;

class AncProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.fs = sampleRate;
    this.mode = 'idle';
    this.cal = null;
    this.adaptSeconds = 1.2;
    this.maxOut = 0.5;
    this.hold = false;
    this.trimDb = 0;
    this.trimDeg = 0;
    this.a = 1 - Math.exp(-TWO_PI * 2 / this.fs); // 2 Hz demodulation low-pass, two stages
    this.ringLen = Math.ceil(RING_SECONDS * this.fs);
    this.ringPos = 0;
    this.nd = 0;
    this.tones = [];
    for (let i = 0; i < MAX_TONES; i++) this.tones.push(this.newTone());
    this.blocks = 0;
    this.reportBlocks = Math.max(1, Math.round(0.1 * this.fs / 128));
    this.fllBlocks = Math.max(1, Math.round(0.08 * this.fs / 128));
    this.stepBlocks = Math.max(1, Math.round(STEP_CHECK_SECONDS * this.fs / 128));
    this.port.onmessage = (e) => this.onMessage(e.data);
  }

  newTone() {
    return {
      f: 100, fFine: 0, fRate: 0, phase: 0, dphi: 0, active: false, live: false,
      ring: new Float64Array(this.ringLen),
      wr: 0, wi: 0, z1r: 0, z1i: 0, z2r: 0, z2i: 0,
      sr: 0, si: 0, ur: 0, ui: 0, k: 0, ok: false, adapt: false, gate: false,
      eP: 0, dP: 0, louder: 0, prevDr: 0, prevDi: 0, fllCount: 0,
      step: 1, stepCount: 0, lastWr: 0, lastWi: 0, dWr: 0, dWi: 0,
      age: 0, baseP: 0, baseAcc: 0, baseN: 0, maxed: 0, maxedSent: false,
    };
  }

  resetTone(t) {
    t.wr = t.wi = 0;
    t.z1r = t.z1i = t.z2r = t.z2i = 0;
    t.eP = t.dP = 0;
    t.louder = 0;
    t.prevDr = t.prevDi = 0;
    t.stepCount = 0;
    t.lastWr = t.lastWi = t.dWr = t.dWi = 0;
    t.age = 0; t.baseP = 0; t.baseAcc = 0; t.baseN = 0;
    t.maxed = 0; t.maxedSent = false;
  }

  onMessage(m) {
    switch (m.type) {
      case 'params': {
        if (this.hold && !m.hold) {
          // Releasing hold keeps what was tuned by ear: fold the trims into W.
          const g = Math.pow(10, this.trimDb / 20), th = this.trimDeg * Math.PI / 180;
          const gr = g * Math.cos(th), gi = g * Math.sin(th);
          for (const t of this.tones) {
            const wr = t.wr * gr - t.wi * gi, wi = t.wr * gi + t.wi * gr;
            t.wr = wr; t.wi = wi;
          }
        }
        this.adaptSeconds = m.adaptSeconds;
        this.maxOut = m.maxOut;
        this.hold = m.hold;
        this.trimDb = m.hold ? m.trimDb : 0;
        this.trimDeg = m.hold ? m.trimDeg : 0;
        break;
      }
      case 'setCal': {
        const c = m.cal;
        this.cal = {
          tau: c.tau, f0: c.f0, df: c.df, n: c.re.length, minMag2: c.minMag2,
          re: Float64Array.from(c.re), im: Float64Array.from(c.im),
        };
        this.nd = Math.min(this.ringLen - 1, Math.max(0, Math.round(c.tau * this.fs)));
        for (const t of this.tones) { this.resetTone(t); t.active = false; t.step = 1; }
        break;
      }
      case 'run':
        this.mode = m.on ? 'run' : 'idle';
        if (!m.on) for (const t of this.tones) { this.resetTone(t); t.active = false; t.fFine = 0; t.fRate = 0; }
        break;
      case 'tone': {
        const t = this.tones[m.i];
        if (!t) break;
        if (m.reset) this.resetTone(t);
        if (m.resetFine) { t.fFine = 0; t.fRate = 0; }
        if (typeof m.f === 'number') t.f = m.f;
        if (typeof m.active === 'boolean') t.active = m.active;
        break;
      }
      case 'calibrate':
        this.sweep = m.sweep;
        this.rec = new Float32Array(m.recLength);
        this.calPos = 0;
        this.mode = 'cal';
        break;
    }
  }

  // Output-to-mic response at f, excluding the pure delay (handled by the phase ring).
  rHat(f) {
    const c = this.cal;
    let x = (f - c.f0) / c.df;
    if (x < 0) x = 0;
    if (x > c.n - 1.001) x = c.n - 1.001;
    const i = x | 0, u = x - i;
    return [c.re[i] + (c.re[i + 1] - c.re[i]) * u, c.im[i] + (c.im[i + 1] - c.im[i]) * u];
  }

  process(inputs, outputs) {
    const input = inputs[0];
    const mic = input && input.length ? input[0] : null;
    const out = outputs[0][0];
    const dOut = outputs[1] && outputs[1][0];
    const n = out.length;

    if (this.mode === 'cal') {
      for (let j = 0; j < n; j++) {
        const p = this.calPos;
        out[j] = p < this.sweep.length ? this.sweep[p] : 0;
        const e = mic ? mic[j] : 0;
        if (p < this.rec.length) this.rec[p] = e;
        if (dOut) dOut[j] = e;
        this.calPos++;
      }
      if (this.calPos >= this.rec.length) {
        this.port.postMessage({ type: 'calRec', rec: this.rec }, [this.rec.buffer]);
        this.rec = null;
        this.mode = 'idle';
      }
      return true;
    }

    if (this.mode !== 'run' || !this.cal) {
      out.fill(0);
      if (dOut) for (let j = 0; j < n; j++) dOut[j] = mic ? mic[j] : 0;
      return true;
    }

    const fs = this.fs, a = this.a, k0 = 1 / (this.adaptSeconds * fs);
    let gr = 1, gi = 0;
    if (this.hold) {
      const g = Math.pow(10, this.trimDb / 20), th = this.trimDeg * Math.PI / 180;
      gr = g * Math.cos(th); gi = g * Math.sin(th);
    }
    const fade = Math.exp(-1 / (0.4 * fs));
    const T = this.tones;
    for (const t of T) {
      const f = t.f + t.fFine;
      const s = this.rHat(f);
      t.sr = s[0]; t.si = s[1];
      const m2 = s[0] * s[0] + s[1] * s[1];
      t.ok = m2 > this.cal.minMag2;
      t.ur = t.ok ? s[0] / m2 : 0;
      t.ui = t.ok ? -s[1] / m2 : 0;
      t.dphi = TWO_PI * f / fs;
      t.gate = t.active && t.ok;
      // No anti-noise until the tone's baseline drone level has been measured.
      t.adapt = t.gate && !this.hold && t.baseP > 0;
      t.k = k0 * t.step;
      t.live = t.gate || t.wr * t.wr + t.wi * t.wi > 1e-12;
    }

    const RL = this.ringLen, nd = this.nd;
    for (let j = 0; j < n; j++) {
      const e = mic ? mic[j] : 0;
      let y = 0, pred = 0;
      const wp = this.ringPos;
      const rp = wp >= nd ? wp - nd : wp - nd + RL;
      for (let q = 0; q < T.length; q++) {
        const t = T[q];
        let ph = t.phase + t.dphi;
        if (ph >= TWO_PI) ph -= TWO_PI;
        t.phase = ph;
        t.ring[wp] = ph;
        if (!t.live) continue;
        const owr = t.wr * gr - t.wi * gi, owi = t.wr * gi + t.wi * gr;
        y += owr * Math.cos(ph) - owi * Math.sin(ph);
        // The copy of the sine now reaching the mic left the speakers nd samples ago.
        const pd = t.ring[rp];
        const c = Math.cos(pd), s = Math.sin(pd);
        const pr = t.sr * owr - t.si * owi, pim = t.sr * owi + t.si * owr;
        pred += pr * c - pim * s;
        t.z1r += a * (e * c - t.z1r); t.z1i += a * (-e * s - t.z1i);
        t.z2r += a * (t.z1r - t.z2r); t.z2i += a * (t.z1i - t.z2i);
        if (t.adapt) {
          const er = 2 * t.z2r, ei = 2 * t.z2i;
          t.wr -= t.k * (t.ur * er - t.ui * ei);
          t.wi -= t.k * (t.ur * ei + t.ui * er);
        } else if (!t.gate) {
          t.wr *= fade; t.wi *= fade;
        }
      }
      out[j] = y > 0.98 ? 0.98 : y < -0.98 ? -0.98 : y;
      if (dOut) dOut[j] = e - pred;
      this.ringPos = wp + 1 === RL ? 0 : wp + 1;
    }

    this.afterBlock(n, gr, gi);
    return true;
  }

  afterBlock(n, gr, gi) {
    const dt = n / this.fs;
    const b = 1 - Math.exp(-dt / 0.5);
    const leak = Math.exp(-dt / LEAK_SECONDS);
    let total = 0;
    for (const t of this.tones) {
      const m = Math.hypot(t.wr, t.wi);
      if (m > this.maxOut) { t.wr *= this.maxOut / m; t.wi *= this.maxOut / m; }
      t.wr *= leak; t.wi *= leak;
      total += Math.hypot(t.wr, t.wi);
    }
    if (total > 0.95) for (const t of this.tones) { t.wr *= 0.95 / total; t.wi *= 0.95 / total; }

    for (let q = 0; q < this.tones.length; q++) {
      const t = this.tones[q];
      if (!t.gate) continue;
      const er = 2 * t.z2r, ei = 2 * t.z2i;
      const owr = t.wr * gr - t.wi * gi, owi = t.wr * gi + t.wi * gr;
      const dr = er - (t.sr * owr - t.si * owi);
      const di = ei - (t.sr * owi + t.si * owr);
      t.eP += b * (er * er + ei * ei - t.eP);
      t.dP += b * (dr * dr + di * di - t.dP);

      // Baseline: the drone level at the mic with no anti-noise, measured once the
      // low-pass has settled. Adaptation waits for it.
      t.age += dt;
      const b0 = 0.5, b1 = 0.5 + Math.max(0.25, this.nd / this.fs);
      if (t.age >= b0 && t.age < b1) { t.baseAcc += er * er + ei * ei; t.baseN++; }
      else if (t.age >= b1 && !t.baseP && t.baseN) { t.baseP = t.baseAcc / t.baseN + 1e-20; t.eP = t.baseP; }

      // Frequency-locked loop: the estimated drone, seen against the sine now arriving
      // at the mic, rotates at the pitch error. A PI loop on that error learns both the
      // pitch and its rate of change, so the sine leads the drone by one stereo delay.
      t.fFine += t.fRate * dt;
      if (++t.fllCount >= this.fllBlocks) {
        t.fllCount = 0;
        const span = this.fllBlocks * dt;
        const m0 = t.prevDr * t.prevDr + t.prevDi * t.prevDi;
        if (t.dP > 1e-9 && m0 > 1e-12) {
          const err = Math.atan2(di * t.prevDr - dr * t.prevDi, dr * t.prevDr + di * t.prevDi) / (TWO_PI * span);
          t.fRate = Math.max(-2, Math.min(2, t.fRate + FLL_KI * err * span));
          t.fFine += FLL_KP * err * span;
        }
        t.f = Math.max(15, Math.min(480, t.f + t.fFine));
        t.fFine = 0;
        t.prevDr = dr; t.prevDi = di;
      }

      // Step-size control: W moving back and forth between successive 0.4 s windows
      // means it is overshooting (stereo louder than at calibration), so slow down;
      // moving steadily in one direction means it is tracking, so speed back up.
      if (!this.hold && ++t.stepCount >= this.stepBlocks) {
        t.stepCount = 0;
        const dWr = t.wr - t.lastWr, dWi = t.wi - t.lastWi;
        const m1 = Math.hypot(dWr, dWi), m2 = Math.hypot(t.dWr, t.dWi);
        const sig = 0.03 * Math.hypot(t.wr, t.wi) + 1e-4;
        if (m1 > sig && m2 > sig) {
          const corr = (dWr * t.dWr + dWi * t.dWi) / (m1 * m2);
          if (corr < -0.3) t.step = Math.max(0.1, t.step * 0.75);
          else if (corr > 0.3) t.step = Math.min(1, t.step * 1.1);
        }
        t.dWr = dWr; t.dWi = dWi;
        t.lastWr = t.wr; t.lastWi = t.wi;
      }

      // Safety: if the tone has made things louder than the measured baseline for 3 s,
      // drop it and start over.
      const ref = t.baseP || t.dP;
      if (!this.hold && Math.hypot(t.wr, t.wi) > 1e-3 && t.eP > 2 * ref) t.louder += dt;
      else t.louder = Math.max(0, t.louder - dt);
      if (t.louder > 3) {
        this.port.postMessage({ type: 'diverged', i: q, f: t.f + t.fFine });
        this.resetTone(t);
        t.step = Math.max(0.1, t.step * 0.5);
      }

      // Output pinned at the limit without a real reduction: the stereo is too quiet
      // or the phone isn't hearing it.
      if (Math.hypot(t.wr, t.wi) >= 0.97 * this.maxOut && t.baseP && t.eP > 0.6 * t.baseP) t.maxed += dt;
      else t.maxed = 0;
      if (t.maxed > 5 && !t.maxedSent) {
        t.maxedSent = true;
        this.port.postMessage({ type: 'maxed', i: q, f: t.f + t.fFine });
      }
    }

    if (++this.blocks >= this.reportBlocks) {
      this.blocks = 0;
      this.port.postMessage({
        type: 'stats',
        tones: this.tones.map((t) => ({
          active: t.active, ok: t.ok, f: t.f + t.fFine, amp: Math.hypot(t.wr, t.wi), step: t.step,
          eDb: 10 * Math.log10(t.eP + 1e-20), dDb: 10 * Math.log10(t.dP + 1e-20),
          baseDb: t.baseP ? 10 * Math.log10(t.baseP) : null,
        })),
      });
    }
  }
}

registerProcessor('anc', AncProcessor);
