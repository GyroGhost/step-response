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
    const fF = [0, 1, 2].map((a) => need("axisF[" + a + "]"));
    const hasTerms = fF[0] >= 0 && fP[0] >= 0;
    const fThr = need("rcCommand[3]");
    if (fTime < 0 || fGyro[0] < 0) throw new Error("Log has no gyro data");
    const hasSet = fSet[0] >= 0;
    if (!hasSet && fP[0] < 0) throw new Error("Log has no setpoint or P-term fields");

    let cap = 1 << 16, n = 0;
    let t = new Float64Array(cap);
    let g = [0, 1, 2].map(() => new Float32Array(cap));
    let s = [0, 1, 2].map(() => new Float32Array(cap));
    let th = new Float32Array(cap);
    let pt = hasTerms ? [0, 1, 2].map(() => new Float32Array(cap)) : null;
    let ft = hasTerms ? [0, 1, 2].map(() => new Float32Array(cap)) : null;
    const grow = () => {
      cap *= 2;
      const nt = new Float64Array(cap); nt.set(t); t = nt;
      g = g.map((a) => { const b = new Float32Array(cap); b.set(a); return b; });
      s = s.map((a) => { const b = new Float32Array(cap); b.set(a); return b; });
      const nth = new Float32Array(cap); nth.set(th); th = nth;
      const g2 = (arr) => arr && arr.map((a) => { const b = new Float32Array(cap); b.set(a); return b; });
      pt = g2(pt); ft = g2(ft);
    };
    const scale = sc.blackbox_high_resolution > 0 ? 10 : 1;
    const pGain = [sc.rollPID, sc.pitchPID, sc.yawPID].map((p) => (p && p[0]) || 1);
    let lastT = -Infinity, invalid = 0, backwards = 0;
    parser.onFrameReady = function (valid, frame, type) {
      if (type !== "I" && type !== "P") return;
      if (!valid || !frame) { invalid++; return; }
      const tt = frame[fTime];
      if (!(tt > lastT)) { backwards++; return; } // drop non-monotonic
      lastT = tt;
      if (n >= cap) grow();
      t[n] = tt;
      for (let a = 0; a < 3; a++) {
        const gy = frame[fGyro[a]] / scale;
        g[a][n] = gy;
        s[a][n] = hasSet ? frame[fSet[a]] / scale : gy + frame[fP[a]] / (0.032029 * pGain[a]);
        if (hasTerms) { pt[a][n] = frame[fP[a]]; ft[a][n] = fF[a] >= 0 ? frame[fF[a]] : 0; }
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
    const time = t.subarray(0, n);
    const health = logHealth(time, parser.stats, invalid, backwards);
    return {
      index, header: hdr,
      time, gyro: g.map((a) => a.subarray(0, n)),
      input: s.map((a) => a.subarray(0, n)), throttle: th.subarray(0, n),
      pterm: pt && pt.map((a) => a.subarray(0, n)), fterm: ft && ft.map((a) => a.subarray(0, n)),
      duration: (t[n - 1] - t[0]) / 1e6,
      health, gapThr: health.gapThr,
    };
  }

  // ---------- log health (logging artefacts) ----------
  // Dropped or corrupt frames leave holes in the time axis. Plotting tools join the points on both
  // sides with a straight line, which looks like a jagged gyro trace that never happened.
  const HEALTH = { gapFactor: 2.5, minGapUs: 1500, warnLost: 0.5, badLost: 5, warnCorrupt: 0.2, badCorrupt: 2, minRate: 500 };

  function logHealth(time, stats, invalid, backwards) {
    const n = time.length;
    // median frame interval from an even sample of the diffs
    const step = Math.max(1, Math.floor((n - 1) / 20000));
    const sample = [];
    for (let i = 1; i < n; i += step) sample.push(time[i] - time[i - 1]);
    sample.sort((a, b) => a - b);
    const med = sample[sample.length >> 1] || 1;
    // ignore sub-1.5 ms jitter on fast logs; it doesn't bend the step response
    const gapThr = Math.max(med * HEALTH.gapFactor, HEALTH.minGapUs);
    let gaps = 0, lost = 0, longest = 0;
    for (let i = 1; i < n; i++) {
      const d = time[i] - time[i - 1];
      if (d > gapThr) { gaps++; lost += d - med; if (d > longest) longest = d; }
    }
    const span = time[n - 1] - time[0] || 1;
    let corrupt = (stats && stats.totalCorruptFrames) || 0;
    if (stats && stats.frame) for (const k of ["I", "P"]) if (stats.frame[k]) corrupt += stats.frame[k].desyncCount || 0;
    corrupt += invalid + backwards;
    const lostPct = (lost / span) * 100;
    const corruptPct = (corrupt / (n + corrupt)) * 100;
    const rate = Math.round(1e6 / med);
    const reasons = [];
    if (lostPct >= HEALTH.warnLost) reasons.push("gaps");
    if (corruptPct >= HEALTH.warnCorrupt) reasons.push("corrupt");
    if (rate < HEALTH.minRate) reasons.push("lowRate");
    const level = lostPct >= HEALTH.badLost || corruptPct >= HEALTH.badCorrupt ? "bad" : reasons.length ? "warn" : "ok";
    return {
      level, reasons, rate, frames: n, gaps, corrupt,
      lostPct: +lostPct.toFixed(2), corruptPct: +corruptPct.toFixed(2),
      longestGapMs: +(longest / 1000).toFixed(1), gapThr,
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
    const out = { dt, n, gyro: [], input: [], bad: new Uint8Array(n) };
    for (let a = 0; a < 3; a++) { out.gyro.push(new Float64Array(n)); out.input.push(new Float64Array(n)); }
    const terms = !!(log.pterm && log.fterm);
    if (terms) { out.pterm = [0, 1, 2].map(() => new Float64Array(n)); out.fterm = [0, 1, 2].map(() => new Float64Array(n)); }
    const gapThr = log.gapThr || Infinity;
    let j = i0;
    for (let i = 0; i < n; i++) {
      const tt = t[i0] + i * dt * 1e6;
      while (j < i1 - 1 && t[j + 1] < tt) j++;
      if (t[j + 1] - t[j] > gapThr) out.bad[i] = 1; // interpolated across missing frames
      const f = (tt - t[j]) / ((t[j + 1] - t[j]) || 1);
      for (let a = 0; a < 3; a++) {
        out.gyro[a][i] = log.gyro[a][j] + (log.gyro[a][j + 1] - log.gyro[a][j]) * f;
        out.input[a][i] = log.input[a][j] + (log.input[a][j + 1] - log.input[a][j]) * f;
        if (terms) {
          out.pterm[a][i] = log.pterm[a][j] + (log.pterm[a][j + 1] - log.pterm[a][j]) * f;
          out.fterm[a][i] = log.fterm[a][j] + (log.fterm[a][j + 1] - log.fterm[a][j]) * f;
        }
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

  function axisResponse(inp, out, dt, bad) {
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
    // prefix count of gap samples so each window can be checked in O(1)
    const badSum = new Uint32Array(inp.length + 1);
    if (bad) for (let i = 0; i < inp.length; i++) badSum[i + 1] = badSum[i] + bad[i];
    let gapWin = 0;
    const hr = new Float64Array(N), hi = new Float64Array(N), gr = new Float64Array(N), gi = new Float64Array(N);
    for (let w = 0; w < wins; w++) {
      const o = w * shift;
      if (o + flen > inp.length) break;
      if (badSum[o + flen] - badSum[o] > 0) { maxIn.push(0); responses.push(null); gapWin++; continue; }
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
    return { responses, maxIn, rlen, dt, gapWin };
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
    let gapWin = 0;
    for (let a = 0; a < 3; a++) {
      const { responses, maxIn, rlen, dt, gapWin: gw } = axisResponse(eq.input[a], eq.gyro[a], eq.dt, eq.bad);
      gapWin = gw;
      // quality gate (PIDtoolbox style): keep windows whose response settles near 1
      const s0 = Math.floor(0.2 / dt);
      const ok = responses.map((r) => {
        if (!r) return false;
        let ss = 0, mx = 0;
        for (let i = s0; i < rlen; i++) ss += r[i];
        ss /= rlen - s0;
        for (let i = 0; i < rlen; i++) if (Math.abs(r[i]) > mx) mx = Math.abs(r[i]);
        return ss > 0.5 && ss < 1.5 && mx < 3;
      });
      const pick = (inRange) => {
        const idx = [];
        for (let w = 0; w < responses.length; w++) if (ok[w] && inRange(maxIn[w])) idx.push(w);
        return idx;
      };
      const meanSd = (idx) => {
        if (idx.length < 3) return null;
        const mean = new Float64Array(rlen), sd = new Float64Array(rlen);
        for (const w of idx) { const r = responses[w]; for (let i = 0; i < rlen; i++) mean[i] += r[i]; }
        for (let i = 0; i < rlen; i++) mean[i] /= idx.length;
        for (const w of idx) { const r = responses[w]; for (let i = 0; i < rlen; i++) sd[i] += (r[i] - mean[i]) ** 2; }
        for (let i = 0; i < rlen; i++) sd[i] = Math.sqrt(sd[i] / idx.length);
        return { mean, sd, n: idx.length };
      };
      let low = meanSd(pick((m) => m > CFG.minInput && m <= CFG.threshold));
      if (!low) low = meanSd(pick((m) => m > CFG.minInput));
      const high = meanSd(pick((m) => m > CFG.threshold));
      axes.push({
        resp: low && low.mean, sd: low && low.sd, n: low ? low.n : 0,
        respHigh: high && high.mean, sdHigh: high && high.sd, nHigh: high ? high.n : 0,
        dt, windows: low ? low.n : 0, total: maxIn.length,
        m: low ? metrics(low.mean, dt) : null,
        mHigh: high ? metrics(high.mean, dt) : null,
        ff: log.header.setpointSource === "setpoint" ? ffAxis(eq.input[a], eq.gyro[a], eq.dt, eq.bad, eq.pterm && eq.pterm[a], eq.fterm && eq.fterm[a]) : { verdict: "noSetpoint" },
      });
    }
    return { axes, dt: eq.dt, rate: Math.round(1 / eq.dt), gapWin };
  }

  // ---------- feedforward check ----------
  // Looks only at fast stick moves. Lag = how far the gyro trails the setpoint (best time shift).
  // Overshoot = how far the gyro goes past the setpoint right after a fast move reaches its new rate.
  const FF = { minSlope: 3000, minRate: 30, minPlateau: 200, maxLagMs: 40, minLagMs: -10, lagLow: 3, lagHigh: 12, over: 12, minEvents: 5, minFastS: 0.25, minTrack: 0.4, moveSlope: 500, ffDom: 0.5, pDom: 0.3 };
  function shareOnly(sp, ds, k, n, dt, bad, pt, ft) {
    if (!pt || !ft) return null;
    let sf = 0, sp2 = 0, m = 0;
    for (let i = k; i < n - k; i++) { if (Math.abs(ds[i]) < FF.moveSlope || (bad && bad[i])) continue; sf += Math.abs(ft[i]); sp2 += Math.abs(pt[i]); m++; }
    return m * dt >= FF.minFastS && sf + sp2 > 0 ? +(sf / (sf + sp2)).toFixed(2) : null;
  }

  function ffAxis(sp, gy, dt, bad, pt, ft) {
    const n = sp.length, ms = (x) => Math.max(1, Math.round(x / 1000 / dt));
    const k = ms(5), lagMin = -ms(-FF.minLagMs), lagMax = ms(FF.maxLagMs);
    const ds = new Float64Array(n);
    for (let i = k; i < n - k; i++) ds[i] = (sp[i + k] - sp[i - k]) / (2 * k * dt);
    // bad sample near i?  prefix sum over gap marks
    const bs = new Uint32Array(n + 1);
    for (let i = 0; i < n; i++) bs[i + 1] = bs[i] + (bad ? bad[i] : 0);
    const clean = (i0, i1) => i0 >= 0 && i1 < n && bs[i1 + 1] - bs[i0] === 0;
    const fast = new Uint8Array(n);
    let nFast = 0;
    for (let i = k; i < n - k; i++) {
      if (Math.abs(ds[i]) > FF.minSlope && Math.abs(sp[i]) > FF.minRate && clean(i + lagMin, i + lagMax)) { fast[i] = 1; nFast++; }
    }
    if (nFast * dt < FF.minFastS) return { verdict: "none", fastS: +(nFast * dt).toFixed(2), events: 0, share: shareOnly(sp, ds, k, n, dt, bad, pt, ft) };
    // lag: time shift that best lines the gyro up with the setpoint during fast moves
    let best = Infinity, lag = 0;
    for (let L = lagMin; L <= lagMax; L++) {
      let e = 0;
      for (let i = 0; i < n; i++) if (fast[i]) { const d = gy[i + L] - sp[i]; e += d * d; }
      if (e < best) { best = e; lag = L; }
    }
    // does the gyro follow the stick at all? (least-squares gain gyro/setpoint over fast moves)
    let gs = 0, ss = 0;
    for (let i = 0; i < n; i++) if (fast[i]) { gs += gy[i + lag] * sp[i]; ss += sp[i] * sp[i]; }
    const track = ss ? gs / ss : 0;
    // who pushes during stick moves: FF share of |P| + |FF| while the stick is moving (any speed)
    const share = shareOnly(sp, ds, k, n, dt, bad, pt, ft);
    const lagPinned = lag === lagMin || lag === lagMax; // best shift sits on the search limit: no real match
    // overshoot after each fast move away from zero that settles on a new rate
    const overs = [];
    const hold = ms(20), look = ms(60);
    for (let i = 1; i < n; i++) {
      if (!(fast[i - 1] && !fast[i])) continue;
      const e = i - 1;
      if (Math.sign(ds[e]) !== Math.sign(sp[e])) continue; // only moves that speed the craft up
      const P = sp[Math.min(n - 1, e + hold)];
      if (Math.abs(P) < FF.minPlateau || Math.abs(P - sp[e]) > 0.1 * Math.abs(P) || !clean(e, e + look)) continue;
      let mx = -Infinity;
      for (let j = e; j <= Math.min(n - 1, e + look); j++) mx = Math.max(mx, Math.sign(P) * gy[j]);
      const o = (mx / Math.abs(P) - 1) * 100;
      if (o > -50) overs.push(o); // gyro never got near the new rate: not an overshoot sample
    }
    overs.sort((a, b) => a - b);
    const over = overs.length ? overs[overs.length >> 1] : null;
    const lagMs = +(lag * dt * 1000).toFixed(1);
    let verdict;
    if (track < FF.minTrack) verdict = "noTrack";
    else if (lagPinned) verdict = "unclear";
    else if (overs.length < FF.minEvents) verdict = lagMs > FF.lagHigh ? "low" : "few";
    else if (lagMs < 0 || (over > FF.over && lagMs <= FF.lagLow)) verdict = "high"; // gyro ahead of the stick
    else if (lagMs > FF.lagHigh) verdict = "low";
    else if (over > FF.over) verdict = share == null ? "over" : share >= FF.ffDom ? "overFF" : share <= FF.pDom ? "over" : "overMix";
    else verdict = "good";
    return { verdict, lagMs, track: +track.toFixed(2), share: share == null ? null : +share.toFixed(2), overshoot: over == null ? null : +over.toFixed(1), events: overs.length, fastS: +(nFast * dt).toFixed(2) };
  }

  root.GGStep = { loadLogs, analyze, CFG, HEALTH, FF, ffAxis };
})(typeof window !== "undefined" ? window : globalThis);
