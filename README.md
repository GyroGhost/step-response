# GyroGhost Step Response

Free PID step response and log comparison for Betaflight blackbox logs. It runs fully in your browser, so your logs never leave your device.

**Open the tool:** https://gyroghost.github.io/step-response/

## What it does

- Reads `.BBL` / `.BFL` blackbox logs directly (every flight inside a file shows up as its own log)
- Plots the step response for Roll, Pitch and Yaw, one big chart per axis, with a ±sd band
- Overlays as many logs as you want, each in its own color, so you can compare before and after a tune change
- Shows Peak, Overshoot, Delay, Rise and Settling for every axis, with a quick read (Good / Overshoot / Soft)
- Pulls the PID, D min, Feedforward and TPA values from the log header and highlights what changed between logs
- Lets you trim a log (start / end second) to cut out takeoff and landing
- Separate view for normal stick moves (< 500°/s) and fast moves (> 500°/s)
- Feedforward check: measures how far the gyro trails your stick on fast moves (lag) and how far it overshoots when a move ends, then tells you if FF looks low, high or good (Betaflight 4.x logs)
- Checks every log for logging artefacts (missing time, corrupt frames, very low logging rate) and warns you, so you don't tune off a broken log
- English and Arabic UI

## How it works

The step response is estimated with Wiener deconvolution over 1 s windows of setpoint and gyro, the same method as [Plasmatree's PID-Analyzer](https://github.com/Plasmatree/PID-Analyzer). Windows with almost no stick input (< 20°/s) or a response that never settles near 1 are dropped. The curve is the mean of the windows that are left, and the shaded band is ±1 standard deviation, like PIDtoolbox. N is the number of windows used.

A dashed curve means N < 15, so there wasn't much stick input on that axis. Flights with too little input on every axis are hidden when you load them.

Logs from Betaflight 4.x use the logged `setpoint`. Older logs without it get the setpoint rebuilt from gyro + P-term, like PID-Analyzer does.

**Feedforward:** only fast stick moves are used (setpoint changing faster than 3000°/s²). Lag is the time shift that best lines the gyro up with the setpoint. Overshoot is the median of how far the gyro goes past the new rate in the 60 ms after a fast move ends. High lag means FF is low. Lag near 0 or below with overshoot means FF is high. Overshoot with normal lag usually points at P or D. It needs the real logged setpoint, so it only runs on Betaflight 4.x logs. The thresholds are a starting point, compare logs before and after a change.

**Log health:** if the blackbox can't write fast enough, frames get dropped and the gyro trace gets fake straight-line jumps across the gap. The tool finds those holes in the log's timestamps (anything longer than 2.5× the normal frame interval and at least 1.5 ms), counts corrupt frames, and leaves out every 1 s window that touches a gap. More than 0.5% missing or 0.2% corrupt shows a warning, more than 5% or 2% marks the log as not reliable. Fix it by erasing the flash, lowering the blackbox rate, or using a faster SD card.

**Tip:** fly with real stick moves on every axis. A hover-only log gives a flat, noisy curve.

## Build

The page is a single `index.html`. To rebuild it:

```sh
npm install
npm run fetch-parser   # clones Betaflight Blackbox Explorer at a pinned commit
npm run build          # writes index.html
```

- `src/page.src.html` – page, styles and UI
- `src/core.js` – log loading and step response math
- `src/example.json` – the example curves shown on first load

## Credits & license

- Log decoding uses the parser from [Betaflight Blackbox Explorer](https://github.com/betaflight/blackbox-log-viewer) (GPL-3.0)
- Step response method from [PID-Analyzer](https://github.com/Plasmatree/PID-Analyzer) by Plasmatree
- Example logs are PID-Analyzer's public sample logs

Licensed under GPL-3.0. See [LICENSE](LICENSE).
