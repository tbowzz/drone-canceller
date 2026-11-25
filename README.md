# Drone Canceller

Proof-of-concept web app that quiets a steady drivetrain drone in a car. The phone's mic
finds the drone's pitch and the app plays a matching anti-noise tone through the car
stereo (Bluetooth or CarPlay), adjusting it until the drone gets quieter at the phone.

- `index.html`, `app.js`: page, calibration flow, frequency tracking, spectrum display.
- `anc-worklet.js`: audio-thread canceller (per-tone adaptive weight, delay-compensated
  demodulation, frequency-locked loop, step-size control, safety resets).
- `dsp.js`: FFT, calibration sweep and analysis, peak picking, tone tracker.
- `test/sim.cjs`: offline simulation against a synthetic car, e.g. `node test/sim.cjs --tones 2 --drift 1`
  (flags are listed at the top of the file).

Calibration plays three 3-second sweeps and measures the stereo's delay and response;
it's stored per sample rate in the browser.
