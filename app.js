'use strict';
(function () {
  const $ = (id) => document.getElementById(id);
  const CAL_KEY = 'droneCanceller.cal.v1';
  const SET_KEY = 'droneCanceller.settings.v1';
  const DEFAULTS = { tones: 2, fmin: 30, fmax: 250, adapt: 1.2, maxOut: 0.5 };

  const store = {
    get(key) { try { return JSON.parse(localStorage.getItem(key)); } catch { return null; } },
    set(key, v) { try { localStorage.setItem(key, JSON.stringify(v)); } catch { /* private mode */ } },
  };

  const settings = Object.assign({}, DEFAULTS, store.get(SET_KEY) || {});
  const trims = { hold: false, trimDeg: 0, trimDb: 0 };

  let ctx = null, stream = null, node = null, micSrc = null, anMic = null, anD = null, sink = null;
  let running = false, calibrating = false, wakeLock = null;
  let cal = null, lastStats = null, trackTimer = null, raf = 0, pendingCal = null, note = null;
  const tracker = new DSP.ToneTracker();
  let specMic = null, specD = null;

  // ---------- UI helpers ----------
  function setStatus(text, warn) {
    const el = $('status');
    el.textContent = text;
    el.classList.toggle('warn', !!warn);
  }

  function renderCal() {
    const el = $('calInfo');
    if (!cal) { el.textContent = 'Not calibrated.'; el.classList.remove('warn'); return; }
    const when = new Date(cal.date).toLocaleString([], { month: 'short', day: 'numeric', hour: 'numeric', minute: '2-digit' });
    const agree = Math.round(cal.quality * 100);
    let text = `Stereo delay ${Math.round(cal.tau * 1000)} ms · passes agree ${agree}% · ${when}`;
    const warn = cal.quality < 0.85;
    if (warn) text += '. Low agreement: recalibrate somewhere quieter or with the stereo louder.';
    el.textContent = text;
    el.classList.toggle('warn', warn);
  }

  function renderSettings() {
    for (const b of $('toneCount').querySelectorAll('button')) b.setAttribute('aria-pressed', String(Number(b.dataset.n) === settings.tones));
    $('fmin').value = settings.fmin;
    $('fmax').value = settings.fmax;
    $('adapt').value = settings.adapt;
    $('adaptVal').textContent = `${Number(settings.adapt).toFixed(1)} s`;
    $('maxOut').value = settings.maxOut;
    $('maxOutVal').textContent = `${Math.round(settings.maxOut * 100)}%`;
    $('trimDegVal').textContent = `${trims.trimDeg > 0 ? '+' : ''}${trims.trimDeg}°`;
    $('trimDbVal').textContent = `${trims.trimDb > 0 ? '+' : ''}${trims.trimDb} dB`;
    $('trims').hidden = !trims.hold;
  }

  function sendParams() {
    if (!node) return;
    node.port.postMessage({ type: 'params', adaptSeconds: settings.adapt, maxOut: settings.maxOut, ...trims });
  }

  function saveSettings() { store.set(SET_KEY, settings); renderSettings(); sendParams(); }

  function renderTones() {
    const box = $('tones');
    const list = lastStats ? lastStats.tones.filter((t) => t.active) : [];
    if (!running || calibrating || !list.length) {
      const msg = !running ? 'None yet.' : calibrating ? 'Paused while calibrating.' : `Listening for a drone between ${settings.fmin} and ${settings.fmax} Hz.`;
      box.innerHTML = `<p class="empty">${msg}</p>`;
      return;
    }
    const rows = list.map((t) => {
      let redText = 'measuring…';
      if (!t.ok) redText = 'stereo can’t play it';
      else if (t.baseDb !== null) {
        const red = t.baseDb - t.eDb;
        redText = `${Math.abs(red).toFixed(0)} dB ${red >= 0 ? 'quieter' : 'louder'}`;
      }
      return `<tr><td>${t.f.toFixed(1)} Hz</td><td class="num">${redText}</td><td class="num">${Math.round(t.amp * 100)}%</td></tr>`;
    }).join('');
    box.innerHTML = `<table><thead><tr><th>Frequency</th><th class="num">At the mic</th><th class="num">Output</th></tr></thead><tbody>${rows}</tbody></table>`;
  }

  function runningStatus() {
    if (!running || calibrating) return;
    if (note && Date.now() < note.until) { setStatus(note.text, true); return; }
    const list = lastStats ? lastStats.tones.filter((t) => t.active && t.ok) : [];
    if (!list.length) { setStatus(`Listening for a drone between ${settings.fmin} and ${settings.fmax} Hz.`); return; }
    const best = list.reduce((a, b) => ((b.baseDb ?? -999) > (a.baseDb ?? -999) ? b : a));
    const red = best.baseDb !== null ? best.baseDb - best.eDb : 0;
    setStatus(red >= 1 ? `Cancelling ${best.f.toFixed(1)} Hz: ${red.toFixed(0)} dB quieter at the phone.` : `Locking on to ${best.f.toFixed(1)} Hz…`);
  }

  // ---------- spectrum ----------
  function draw() {
    raf = requestAnimationFrame(draw);
    if (!anMic || document.hidden) return;
    const cv = $('spec');
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    const g = cv.getContext('2d');
    const css = getComputedStyle(document.documentElement);
    g.clearRect(0, 0, w, h);
    anMic.getFloatFrequencyData(specMic);
    anD.getFloatFrequencyData(specD);
    const binHz = ctx.sampleRate / anMic.fftSize;
    const f0 = Math.max(10, settings.fmin - 10), f1 = settings.fmax + 30;
    const k0 = Math.floor(f0 / binHz), k1 = Math.ceil(f1 / binHz);
    let top = -Infinity;
    for (let k = k0; k <= k1; k++) top = Math.max(top, specD[k], specMic[k]);
    if (!Number.isFinite(top)) return;
    draw.top = draw.top === undefined ? top : Math.max(top, draw.top - 0.3);
    const yTop = draw.top + 4, yBot = yTop - 60;
    const X = (f) => ((f - f0) / (f1 - f0)) * w;
    const Y = (db) => Math.min(h, Math.max(0, ((yTop - db) / (yTop - yBot)) * h));

    g.strokeStyle = css.getPropertyValue('--grid');
    g.fillStyle = css.getPropertyValue('--muted');
    g.lineWidth = 1;
    g.font = `${11 * dpr}px -apple-system, sans-serif`;
    const step = f1 - f0 > 200 ? 50 : 25;
    for (let f = Math.ceil(f0 / step) * step; f <= f1; f += step) {
      g.beginPath(); g.moveTo(X(f), 0); g.lineTo(X(f), h); g.stroke();
      g.fillText(`${f}`, X(f) + 3 * dpr, h - 4 * dpr);
    }
    const line = (arr, color) => {
      g.strokeStyle = color; g.lineWidth = 2 * dpr; g.beginPath();
      for (let k = k0; k <= k1; k++) {
        const x = X(k * binHz), y = Y(Number.isFinite(arr[k]) ? arr[k] : yBot);
        if (k === k0) g.moveTo(x, y); else g.lineTo(x, y);
      }
      g.stroke();
    };
    line(specD, css.getPropertyValue('--without'));
    line(specMic, css.getPropertyValue('--accent'));
    if (lastStats) {
      g.fillStyle = css.getPropertyValue('--text');
      for (const t of lastStats.tones) {
        if (!t.active) continue;
        const x = X(t.f);
        g.beginPath(); g.moveTo(x - 5 * dpr, 0); g.lineTo(x + 5 * dpr, 0); g.lineTo(x, 7 * dpr); g.fill();
      }
    }
  }

  // ---------- tracking ----------
  function track() {
    if (!running || calibrating || !anD) return;
    anD.getFloatFrequencyData(specD);
    const binHz = ctx.sampleRate / anD.fftSize;
    const peaks = DSP.findPeaks(specD, binHz, settings.fmin, settings.fmax, settings.tones, 8, 10);
    const oscF = lastStats ? lastStats.tones.map((t) => t.f) : null;
    for (const m of tracker.update(peaks, settings.tones, oscF)) node.port.postMessage({ type: 'tone', ...m });
    renderTones();
    runningStatus();
  }

  // ---------- calibration ----------
  function recordPass(sweep, recLength) {
    return new Promise((resolve) => {
      pendingCal = resolve;
      node.port.postMessage({ type: 'calibrate', sweep, recLength });
    });
  }

  async function calibrate() {
    calibrating = true;
    $('recal').disabled = true;
    node.port.postMessage({ type: 'run', on: false });
    tracker.reset();
    lastStats = null;
    renderTones();
    try {
      const sr = ctx.sampleRate;
      const sweep = DSP.makeSweep(sr);
      const recLength = sweep.length + Math.round(DSP.CAL.tailSeconds * sr);
      const recs = [];
      for (let p = 0; p < DSP.CAL.passes; p++) {
        setStatus(`Calibrating: sweep ${p + 1} of ${DSP.CAL.passes}. Keep the cabin quiet.`);
        recs.push(await recordPass(sweep, recLength));
        await new Promise((r) => setTimeout(r, DSP.CAL.gapSeconds * 1000));
        if (!running) return false;
      }
      setStatus('Calibrating: measuring…');
      await new Promise((r) => setTimeout(r, 30));
      const result = DSP.analyzeCalibration(sweep, recs, sr);
      if (result.quality < 0.5 || !(result.level > 0)) {
        setStatus('Couldn’t hear the sweep clearly. Turn the stereo up, check the phone is playing through it, and tap Recalibrate.', true);
        return false;
      }
      cal = Object.assign(result, { date: Date.now() });
      store.set(CAL_KEY, cal);
      node.port.postMessage({ type: 'setCal', cal });
      renderCal();
      return true;
    } finally {
      calibrating = false;
      $('recal').disabled = !running;
      renderTones();
    }
  }

  async function recalibrate() {
    if (!running || calibrating) return;
    if (await calibrate()) {
      sendParams();
      node.port.postMessage({ type: 'run', on: true });
      runningStatus();
    }
  }

  // ---------- start / stop ----------
  function onWorkletMessage(e) {
    const m = e.data;
    if (m.type === 'stats') lastStats = m;
    else if (m.type === 'calRec' && pendingCal) { const r = pendingCal; pendingCal = null; r(m.rec); }
    else if (m.type === 'maxed') {
      note = { text: `Anti-noise for ${m.f.toFixed(0)} Hz is at its limit with little effect. Turn the stereo up or raise Max anti-noise level in Settings.`, until: Date.now() + 8000 };
    } else if (m.type === 'diverged') {
      note = { text: `The ${m.f.toFixed(0)} Hz tone was making it louder, so it restarted. Recalibrate if this keeps happening.`, until: Date.now() + 6000 };
    }
  }

  async function waitRunning() {
    if (ctx.state === 'running') return;
    try { await ctx.resume(); } catch { /* needs a tap */ }
    if (ctx.state === 'running') return;
    setStatus('Tap Start again to allow audio.', true);
    throw new Error('audio-blocked');
  }

  async function requestWakeLock() {
    try { if ('wakeLock' in navigator) wakeLock = await navigator.wakeLock.request('screen'); } catch { wakeLock = null; }
  }

  async function start() {
    const btn = $('start');
    btn.disabled = true;
    setStatus('Starting…');
    try {
      try { if (navigator.audioSession) navigator.audioSession.type = 'play-and-record'; } catch { /* not supported */ }
      ctx = new (window.AudioContext || window.webkitAudioContext)({ latencyHint: 'interactive' });
      ctx.resume();
      stream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false, channelCount: 1 },
      });
      const micTrack = stream.getAudioTracks()[0];
      const micRate = micTrack.getSettings && micTrack.getSettings().sampleRate;
      if (micRate && micRate !== ctx.sampleRate) {
        // A context running at a different rate from the mic can distort on iOS.
        await ctx.close();
        ctx = new (window.AudioContext || window.webkitAudioContext)({ latencyHint: 'interactive', sampleRate: micRate });
      }
      await waitRunning();
      await ctx.audioWorklet.addModule('anc-worklet.js');

      micSrc = ctx.createMediaStreamSource(stream);
      node = new AudioWorkletNode(ctx, 'anc', {
        numberOfInputs: 1, numberOfOutputs: 2, outputChannelCount: [1, 1],
        channelCount: 1, channelCountMode: 'explicit',
      });
      node.port.onmessage = onWorkletMessage;
      micSrc.connect(node);
      node.connect(ctx.destination, 0);
      anMic = ctx.createAnalyser();
      anD = ctx.createAnalyser();
      for (const an of [anMic, anD]) { an.fftSize = 32768; an.smoothingTimeConstant = 0.5; }
      micSrc.connect(anMic);
      node.connect(anD, 1);
      sink = ctx.createGain();
      sink.gain.value = 0;
      anMic.connect(sink); anD.connect(sink); sink.connect(ctx.destination);
      specMic = new Float32Array(anMic.frequencyBinCount);
      specD = new Float32Array(anD.frequencyBinCount);
      micTrack.addEventListener('ended', () => { if (running) stop('The mic was interrupted. Tap Start to resume.'); });

      running = true;
      btn.textContent = 'Stop';
      btn.classList.add('stop');
      btn.disabled = false;
      $('recal').disabled = false;
      requestWakeLock();
      raf = requestAnimationFrame(draw);
      sendParams();

      const saved = store.get(CAL_KEY);
      if (saved && saved.sampleRate === ctx.sampleRate) {
        cal = saved;
        node.port.postMessage({ type: 'setCal', cal });
        renderCal();
      } else if (!(await calibrate())) {
        return;
      }
      if (!running) return;
      node.port.postMessage({ type: 'run', on: true });
      trackTimer = setInterval(track, 250);
      runningStatus();
    } catch (err) {
      const msg = err && err.name === 'NotAllowedError'
        ? 'Microphone access was blocked. Allow it for this page in Settings, then tap Start.'
        : err && err.message === 'audio-blocked' ? null : `Couldn’t start audio: ${err && err.message ? err.message : err}`;
      stop(msg);
    }
  }

  function stop(message) {
    running = false;
    calibrating = false;
    clearInterval(trackTimer); trackTimer = null;
    cancelAnimationFrame(raf);
    if (pendingCal) { const r = pendingCal; pendingCal = null; r(new Float32Array(1)); }
    try { node && node.port.postMessage({ type: 'run', on: false }); } catch { /* closed */ }
    if (stream) stream.getTracks().forEach((t) => t.stop());
    if (ctx) ctx.close().catch(() => {});
    if (wakeLock) wakeLock.release().catch(() => {});
    ctx = stream = node = micSrc = anMic = anD = sink = wakeLock = null;
    lastStats = null;
    tracker.reset();
    const btn = $('start');
    btn.textContent = 'Start';
    btn.classList.remove('stop');
    btn.disabled = false;
    $('recal').disabled = true;
    renderTones();
    if (message !== null) setStatus(message || 'Stopped.', !!message);
  }

  // ---------- wiring ----------
  $('start').addEventListener('click', () => (running ? stop() : start()));
  $('recal').addEventListener('click', recalibrate);
  $('toneCount').addEventListener('click', (e) => {
    const n = Number(e.target.dataset && e.target.dataset.n);
    if (n) { settings.tones = n; saveSettings(); }
  });
  const numField = (id, key, lo, hi) => $(id).addEventListener('change', () => {
    const v = Math.round(Number($(id).value));
    if (Number.isFinite(v)) settings[key] = Math.min(hi, Math.max(lo, v));
    if (settings.fmax - settings.fmin < 20) settings.fmax = settings.fmin + 20;
    saveSettings();
  });
  numField('fmin', 'fmin', 15, 400);
  numField('fmax', 'fmax', 35, 480);
  $('adapt').addEventListener('input', () => { settings.adapt = Number($('adapt').value); saveSettings(); });
  $('maxOut').addEventListener('input', () => { settings.maxOut = Number($('maxOut').value); saveSettings(); });
  $('hold').addEventListener('change', () => {
    trims.hold = $('hold').checked;
    if (!trims.hold) { sendParams(); trims.trimDeg = 0; trims.trimDb = 0; $('trimDeg').value = 0; $('trimDb').value = 0; }
    renderSettings(); sendParams();
  });
  $('trimDeg').addEventListener('input', () => { trims.trimDeg = Number($('trimDeg').value); renderSettings(); sendParams(); });
  $('trimDb').addEventListener('input', () => { trims.trimDb = Number($('trimDb').value); renderSettings(); sendParams(); });

  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && running) {
      if (ctx && ctx.state !== 'running') ctx.resume().catch(() => {});
      requestWakeLock();
    }
  });

  cal = store.get(CAL_KEY);
  renderCal();
  renderSettings();
  renderTones();
})();
