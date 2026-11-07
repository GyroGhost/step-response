// GyroGhost Step Response core — log loading + step response (Wiener deconvolution, PID-Analyzer method)
(function (root) {
  "use strict";
  const Parser = () => root.BBParser;

  // ---------- log loading ----------
  function findLogStarts(bytes) {
    const marker = Parser().prototype.FLIGHT_LOG_START_MARKER;
    const starts = [];
    outer: for (let i = 0; i <= bytes.length - marker.length; i++) {
      if (bytes[i] !== marker[0]) continue;
      for (let j = 1; j < marker.length; j++) if (bytes[i + j] !== marker[j]) continue outer;
      starts.push(i);
      i += marker.length - 1;
    }
    return starts;
  }

  function loadLogs(bytes) {
    const starts = findLogStarts(bytes);
    const logs = [];
    for (let k = 0; k < starts.length; k++) {
      const s = starts[k], e = k + 1 < starts.length ? starts[k + 1] : bytes.length;
      try { logs.push(parseOne(bytes, s, e, k)); }
      catch (err) { logs.push({ index: k, error: String(err && err.message || err) }); }
    }
    return logs;
  }

  function parseOne(bytes, start, end, index) {
    const P = Parser();
    const parser = new P(bytes);
    parser.parseHeader(start, end);
    const sc = parser.sysConfig;
    const idx = parser.frameDefs.I.nameToIndex;
    const need = (n) => (idx[n] === undefined ? -1 : idx[n]);
    const fTime = need("time");
    const fGyro = [0, 1, 2].map((a) => need("gyroADC[" + a + "]"));
    const fSet = [0, 1, 2].map((a) => need("setpoint[" + a + "]"));
    const fP = [0, 1, 2].map((a) => need("axisP[" + a + "]"));
    const fThr = need("rcCommand[3]");
    if (fTime < 0 || fGyro[0] < 0) throw new Error("Log has no gyro data");
    const hasSet = fSet[0] >= 0;
    if (!hasSet && fP[0] < 0) throw new Error("Log has no setpoint or P-term fields");

    let cap = 1 << 16, n = 0;
    let t = new Float64Array(cap);
    let g = [0, 1, 2].map(() => new Float32Array(cap));
    let s = [0, 1, 2].map(() => new Float32Array(cap));
    let th = new Float32Array(cap);
    const grow = () => {
      cap *= 2;
      const nt = new Float64Array(cap); nt.set(t); t = nt;
      g = g.map((a) => { const b = new Float32Array(cap); b.set(a); return b; });
      s = s.map((a) => { const b = new Float32Array(cap); b.set(a); return b; });
      const nth = new Float32Array(cap); nth.set(th); th = nth;
    };
    const scale = sc.blackbox_high_resolution > 0 ? 10 : 1;
    const pGain = [sc.rollPID, sc.pitchPID, sc.yawPID].map((p) => (p && p[0]) || 1);
    let lastT = -Infinity;
    parser.onFrameReady = function (valid, frame, type) {
      if (!valid || !frame || (type !== "I" && type !== "P")) return;
      const tt = frame[fTime];
      if (!(tt > lastT)) return; // drop non-monotonic
      lastT = tt;
      if (n >= cap) grow();
      t[n] = tt;
      for (let a = 0; a < 3; a++) {
        const gy = frame[fGyro[a]] / scale;
        g[a][n] = gy;
        s[a][n] = hasSet ? frame[fSet[a]] / scale : gy + frame[fP[a]] / (0.032029 * pGain[a]);
      }
      th[n] = fThr >= 0 ? frame[fThr] : 0;
      n++;
    };
    parser.parseLogData(false);
    if (n < 2000) throw new Error("Log too short (" + n + " frames)");

    const hdr = {
      craft: sc["Craft name"] || "",
      firmware: [sc["Firmware revision"] || sc.firmwareVersion || ""].join(""),
      firmwareVersion: sc.firmwareVersion,
      roll: sc.rollPID, pitch: sc.pitchPID, yaw: sc.yawPID,
      dMin: sc.d_min, ff: sc.ff_weight,
      looptime: sc.looptime, pDenom: sc.pid_process_denom,
      tpaRate: sc.tpa_rate, tpaBreakpoint: sc.tpa_breakpoint,
      dterm_lpf1: sc.dterm_lpf1_static_hz, gyro_lpf1: sc.gyro_lowpass_hz,
      simplified: {
        master: sc.simplified_master_multiplier, pd_ratio: sc.simplified_pi_gain,
        d: sc.simplified_d_gain, ff: sc.simplified_feedforward_gain,
      },
      setpointSource: hasSet ? "setpoint" : "reconstructed (gyro + P-term)",
    };
    return {
      index, header: hdr,
      time: t.subarray(0, n), gyro: g.map((a) => a.subarray(0, n)),
      input: s.map((a) => a.subarray(0, n)), throttle: th.subarray(0, n),
      duration: (t[n - 1] - t[0]) / 1e6,
    };
  }

  // ---------- FFT (radix-2, in place) ----------
  function fft(re, im, inverse) {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { let x = re[i]; re[i] = re[j]; re[j] = x; x = im[i]; im[i] = im[j]; im[j] = x; }
    }
    for (let len = 2; len <= n; len <<= 1) {
      const ang = (2 * Math.PI / len) * (inverse ? 1 : -1);
      const wr = Math.cos(ang), wi = Math.sin(ang);
      const half = len >> 1;
      for (let i = 0; i < n; i += len) {
        let cr = 1, ci = 0;
        for (let k = 0; k < half; k++) {
          const a = i + k, b = a + half;
          const xr = re[b] * cr - im[b] * ci, xi = re[b] * ci + im[b] * cr;
          re[b] = re[a] - xr; im[b] = im[a] - xi;
          re[a] += xr; im[a] += xi;
          const ncr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = ncr;
        }
      }
    }
    if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
  }

  // gaussian_filter1d (reflect mode default in scipy for sn; constant mode for hist)
  function gaussKernel(sigma) {
    const r = Math.max(1, Math.round(4 * sigma));
    const k = new Float64Array(2 * r + 1);
    let sum = 0;
    for (let i = -r; i <= r; i++) { k[i + r] = Math.exp(-0.5 * (i / sigma) ** 2); sum += k[i + r]; }
    for (let i = 0; i < k.length; i++) k[i] /= sum;
    return { k, r };
  }
  function gaussFilter(arr, sigma, mode) {
    const { k, r } = gaussKernel(sigma);
    const n = arr.length, out = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let acc = 0;
      for (let j = -r; j <= r; j++) {
        let p = i + j;
        if (p < 0 || p >= n) {
          if (mode === "constant") continue;
          p = p < 0 ? -p - 1 : 2 * n - p - 1; // reflect
          if (p < 0 || p >= n) continue;
        }
        acc += arr[p] * k[j + r];
      }
      out[i] = acc;
    }
    return out;
  }
  const toMask = (a) => {
    let mn = Infinity, mx = -Infinity;
    for (const v of a) { if (v < mn) mn = v; if (v > mx) mx = v; }
    const d = mx - mn || 1;
    return a.map((v) => (v - mn) / d);
  };

  // ---------- step response ----------
  const CFG = { framelen: 1.0, resplen: 0.5, cutfreq: 25, superpos: 16, threshold: 500, minInput: 20, maxRate: 4000 };

  // resample all channels onto a uniform grid (like PID-Analyzer equalize)
  function equalize(log, t0, t1) {
    const t = log.time;
    let i0 = 0, i1 = t.length - 1;
    if (t0 != null) while (i0 < i1 && (t[i0] - t[0]) / 1e6 < t0) i0++;
    if (t1 != null) while (i1 > i0 && (t[i1] - t[0]) / 1e6 > t1) i1--;
    const span = (t[i1] - t[i0]) / 1e6;
    const medDt = span / (i1 - i0);
    const dt = Math.max(medDt, 1 / CFG.maxRate);
    const n = Math.floor(span / dt);
    const out = { dt, n, gyro: [], input: [] };
    for (let a = 0; a < 3; a++) { out.gyro.push(new Float64Array(n)); out.input.push(new Float64Array(n)); }
    let j = i0;
    for (let i = 0; i < n; i++) {
      const tt = t[i0] + i * dt * 1e6;
      while (j < i1 - 1 && t[j + 1] < tt) j++;
      const f = (tt - t[j]) / ((t[j + 1] - t[j]) || 1);
      for (let a = 0; a < 3; a++) {
        out.gyro[a][i] = log.gyro[a][j] + (log.gyro[a][j + 1] - log.gyro[a][j]) * f;
        out.input[a][i] = log.input[a][j] + (log.input[a][j + 1] - log.input[a][j]) * f;
      }
    }
    return out;
  }

  function snWeights(N, dt) {
    // PID-Analyzer: sn = 10*(1 - smoothed mask(f >= cutfreq)), denominator term = 1/sn
    const freq = new Float64Array(N);
    for (let i = 0; i < N; i++) freq[i] = Math.abs((i <= N / 2 - 1 ? i : i - N) / (N * dt));
    // np.fft.fftfreq for even N: [0..N/2-1, -N/2..-1]
    let clipped = freq.map((f) => Math.min(Math.max(f, CFG.cutfreq - 1e-9), CFG.cutfreq));
    let sn = toMask(clipped);
    let lenLpf = 0; for (const v of sn) lenLpf += 1 - v;
    sn = toMask(gaussFilter(sn, lenLpf / 6, "reflect"));
    const inv = new Float64Array(N);
    for (let i = 0; i < N; i++) inv[i] = 1 / (10 * (-sn[i] + 1 + 1e-9));
    return inv;
  }

  function axisResponse(inp, out, dt) {
    const flen = Math.floor(CFG.framelen / dt);
    const rlen = Math.floor(CFG.resplen / dt);
    const shift = Math.floor(flen / CFG.superpos);
    const wins = Math.floor(inp.length / shift) - CFG.superpos;
    const pad = 1024 - (flen % 1024);
    let N = flen + pad;
    // round N up to power of two for radix-2 FFT
    let P2 = 1; while (P2 < N) P2 <<= 1; N = P2;
    const han = new Float64Array(flen);
    for (let i = 0; i < flen; i++) han[i] = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (flen - 1));
    const invSn = snWeights(N, dt);
    const responses = [], maxIn = [];
    const hr = new Float64Array(N), hi = new Float64Array(N), gr = new Float64Array(N), gi = new Float64Array(N);
    for (let w = 0; w < wins; w++) {
      const o = w * shift;
      if (o + flen > inp.length) break;
      hr.fill(0); hi.fill(0); gr.fill(0); gi.fill(0);
      let mx = 0;
      for (let i = 0; i < flen; i++) {
        const x = inp[o + i] * han[i];
        hr[i] = x; gr[i] = out[o + i] * han[i];
        const ax = Math.abs(x); if (ax > mx) mx = ax;
      }
      maxIn.push(mx);
      if (mx <= CFG.minInput) { responses.push(null); continue; }
      fft(hr, hi, false); fft(gr, gi, false);
      for (let k = 0; k < N; k++) {
        const den = hr[k] * hr[k] + hi[k] * hi[k] + invSn[k];
        // G * conj(H) / den
        const re = (gr[k] * hr[k] + gi[k] * hi[k]) / den;
        const im = (gi[k] * hr[k] - gr[k] * hi[k]) / den;
        gr[k] = re; gi[k] = im;
      }
      fft(gr, gi, true);
      const r = new Float32Array(rlen);
      let acc = 0;
      for (let i = 0; i < rlen; i++) { acc += gr[i]; r[i] = acc; }
      responses.push(r);
    }
    return { responses, maxIn, rlen, dt };
  }

  // PID-Analyzer weighted_mode_avr: 2D histogram -> vertical gaussian -> column-normalised -> weighted avg (w^2)
  function weightedModeAvr(responses, weights, rlen) {
    const vmin = -1.5, vmax = 3.5, vb = 1000;
    const binW = (vmax - vmin) / vb;
    const hist = new Float64Array(rlen * vb); // [t][v]
    let total = 0;
    for (let w = 0; w < responses.length; w++) {
      const r = responses[w], wt = weights[w];
      if (!r || !wt) continue;
      for (let i = 0; i < rlen; i++) {
        const v = r[i];
        if (v < vmin || v > vmax) continue;
        let b = Math.floor((v - vmin) / binW); if (b >= vb) b = vb - 1;
        hist[i * vb + b] += wt; total += wt;
      }
    }
    const avr = new Float64Array(rlen);
    if (!total) return null;
    const { k, r: kr } = gaussKernel(7);
    const col = new Float64Array(vb);
    for (let i = 0; i < rlen; i++) {
      let mx = 0;
      for (let b = 0; b < vb; b++) {
        let acc = 0;
        for (let j = -kr; j <= kr; j++) { const p = b + j; if (p >= 0 && p < vb) acc += hist[i * vb + p] * k[j + kr]; }
        col[b] = acc; if (acc > mx) mx = acc;
      }
      let num = 0, den = 0;
      if (mx > 0) for (let b = 0; b < vb; b++) {
        const h = col[b] / mx, w2 = h * h;
        num += (vmin + (b * (vmax - vmin)) / (vb - 1)) * w2; den += w2;
      }
      avr[i] = den ? num / den : 0;
    }
    return avr;
  }

  function metrics(resp, dt) {
    const n = resp.length;
    const t = (i) => i * dt * 1000;
    // steady state: mean over 200-500 ms
    let ss = 0, c = 0;
    for (let i = Math.floor(0.2 / dt); i < n; i++) { ss += resp[i]; c++; }
    ss = c ? ss / c : 1;
    let peak = -Infinity, peakI = 0;
    for (let i = 0; i < Math.min(n, Math.floor(0.15 / dt)); i++) if (resp[i] > peak) { peak = resp[i]; peakI = i; }
    const cross = (lvl) => {
      for (let i = 1; i < n; i++) if (resp[i] >= lvl) {
        const f = (lvl - resp[i - 1]) / ((resp[i] - resp[i - 1]) || 1);
        return t(i - 1 + f);
      }
      return null;
    };
    const t10 = cross(0.1 * ss), t50 = cross(0.5 * ss), t90 = cross(0.9 * ss);
    let settle = null;
    const band = 0.05 * Math.abs(ss);
    for (let i = n - 1; i >= 0; i--) if (Math.abs(resp[i] - ss) > band) { settle = t(Math.min(i + 1, n - 1)); break; }
    return {
      peak, peakTime: t(peakI), steady: ss,
      overshoot: ss ? (peak / ss - 1) * 100 : 0,
      delay: t50, rise: t10 != null && t90 != null ? t90 - t10 : null, settling: settle,
    };
  }

  function analyze(log, opts) {
    opts = opts || {};
    const eq = equalize(log, opts.start, opts.end);
    const axes = [];
    for (let a = 0; a < 3; a++) {
      const { responses, maxIn, rlen, dt } = axisResponse(eq.input[a], eq.gyro[a], eq.dt);
      const used = maxIn.filter((m) => m > CFG.minInput).length;
      const low = maxIn.map((m) => (m > CFG.minInput && m <= CFG.threshold ? 1 : 0));
      const all = maxIn.map((m) => (m > CFG.minInput ? 1 : 0));
      const hiW = maxIn.map((m) => (m > CFG.threshold ? 1 : 0));
      const hiCount = hiW.reduce((x, y) => x + y, 0);
      const resp = weightedModeAvr(responses, low.reduce((x, y) => x + y, 0) >= 10 ? low : all, rlen);
      const respHigh = hiCount >= 10 ? weightedModeAvr(responses, hiW, rlen) : null;
      axes.push({
        resp, respHigh, dt, windows: used, total: maxIn.length,
        m: resp ? metrics(resp, dt) : null,
      });
    }
    return { axes, dt: eq.dt, rate: Math.round(1 / eq.dt) };
  }

  root.GGStep = { loadLogs, analyze, CFG };
})(typeof window !== "undefined" ? window : globalThis);
