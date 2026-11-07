# GyroGhost Step Response

Free PID step response and log comparison for Betaflight blackbox logs. It runs fully in your browser, so your logs never leave your device.

**Open the tool:** https://gyroghost.github.io/step-response/

## What it does

- Reads `.BBL` / `.BFL` blackbox logs directly (every flight inside a file shows up as its own log)
- Plots the step response for Roll, Pitch and Yaw
- Overlays as many logs as you want, each in its own color, so you can compare before and after a tune change
- Shows Peak, Overshoot, Delay, Rise and Settling for every axis, with a quick read (Good / Overshoot / Soft)
- Pulls the PID, D min, Feedforward and TPA values from the log header and highlights what changed between logs
- Lets you trim a log (start / end second) to cut out takeoff and landing
- Separate view for normal stick moves (< 500°/s) and fast moves (> 500°/s)
- English and Arabic UI

## How it works

The step response is estimated with Wiener deconvolution over 1 s windows of setpoint and gyro, the same method as [Plasmatree's PID-Analyzer](https://github.com/Plasmatree/PID-Analyzer). Windows with almost no stick input (< 20°/s) are ignored, and the result is a weighted average of all the remaining windows.

On PID-Analyzer's own sample log (`good_tune.BBL`) the roll curve matches the PID-Analyzer reference within 0.003.

Logs from Betaflight 4.x use the logged `setpoint`. Older logs without it get the setpoint rebuilt from gyro + P-term, like PID-Analyzer does.

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

---

## بالعربي

أداة مجانية تطلع لك الـ Step Response لكل محور من لوقات البلاك بوكس حق Betaflight، وتقارن بين أكثر من لوق فوق بعض. تشتغل بالمتصفح، واللوق ما يطلع من جهازك.

افتح الأداة، واسحب ملفات BBL عليها، وقارن قبل وبعد التيون.
