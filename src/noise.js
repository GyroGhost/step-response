// GyroGhost Noise Lab core — throttle vs frequency noise maps and motor RPM harmonics from eRPM
(function (root) {
  "use strict";

  const CFG = {
    thrBins: 50,        // 2% throttle per row
    binHz: 4,           // target frequency resolution
    refLo: 10, refHi: 50, // band used to line up D-term input and output levels
    peakMinHz: 80,      // ignore the low end (stick moves, propwash) when looking for noise peaks
    harmTol: 0.12,      // a peak within ±12% of a harmonic counts as motor noise
    rangeDb: 45,        // colour range below the top of each pair
  };

  const median = (arr) => {
    if (!arr.length) return NaN;
    const s = Float64Array.from(arr).sort();
    const m = s.length >> 1;
    return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
  };
  const nextPow2 = (v) => { let n = 64; while (n < v) n <<= 1; return n; };

  function analyze(log, opts) {
    opts = opts || {};
    const fft = root.GGStep.fft;
    const time = log.time, n = time.length;
    const rate = log.health.rate;
    const fs = rate, dt = 1 / fs;
    const N = nextPow2(fs / CFG.binHz), hop = N >> 1, F = N >> 1;
    const t0 = time[0];
    let i0 = 0, i1 = n;
    if (opts.start != null) while (i0 < n && (time[i0] - t0) / 1e6 < opts.start) i0++;
    if (opts.end != null) { i1 = i0; while (i1 < n && (time[i1] - t0) / 1e6 <= opts.end) i1++; }

    const ex = log.extra || { gyroUnf: [], dterm: [], erpm: [] };
    const raw = log.raw || {};
    const hasUnf = ex.gyroUnf.some(Boolean);
    const poles = parseInt(raw.motor_poles, 10) || null;
    const erpm = (ex.erpm || []).filter(Boolean);
    const hasErpm = erpm.length > 0 && !!poles;
    const rpmH = raw.rpm_filter_harmonics != null ? parseInt(raw.rpm_filter_harmonics, 10) : null;
    const rpmMin = raw.rpm_filter_min_hz != null ? parseInt(raw.rpm_filter_min_hz, 10) : null;
    const dLpf = raw.dterm_lpf1_static_hz != null ? parseInt(raw.dterm_lpf1_static_hz, 10) : null;

    // D-term input = rate of change of the filtered gyro, before the D-term filters
    const dIn = log.gyro.map((g) => {
      const d = new Float32Array(n);
      for (let i = 1; i < n; i++) d[i] = (g[i] - g[i - 1]) * fs;
      return d;
    });
    const dOut = ex.dterm.map((d) => (d && d.some((v) => v !== 0) ? d : null));

    const sigs = {
      gyroUnf: ex.gyroUnf.map((a) => a || null),
      gyro: log.gyro,
      dIn: dIn,
      dOut: dOut,
    };
    const keys = Object.keys(sigs);
    const TB = CFG.thrBins;
    const acc = {};
    keys.forEach((k) => { acc[k] = sigs[k].map((a) => (a ? new Float64Array(TB * F) : null)); });
    const total = {}; keys.forEach((k) => { total[k] = sigs[k].map((a) => (a ? new Float64Array(F) : null)); });
    const cnt = new Uint32Array(TB);
    const motorByBin = Array.from({ length: TB }, () => []);

    const win = new Float32Array(N);
    let wsum = 0;
    for (let i = 0; i < N; i++) { win[i] = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (N - 1)); wsum += win[i] * win[i]; }
    const re = new Float64Array(N), im = new Float64Array(N);
    const gapThr = log.gapThr || Infinity;
    let used = 0, skipped = 0;

    for (let s = i0; s + N <= i1; s += hop) {
      let gap = false, thr = 0;
      for (let i = s + 1; i < s + N; i++) if (time[i] - time[i - 1] > gapThr) { gap = true; break; }
      if (gap) { skipped++; continue; }
      for (let i = s; i < s + N; i++) thr += log.throttle[i];
      thr /= N;
      const pct = Math.max(0, Math.min(99.999, (thr - 1000) / 10));
      const b = Math.floor((pct / 100) * TB);
      cnt[b]++; used++;
      if (hasErpm) {
        const hz = erpm.map((e) => { let m = 0; for (let i = s; i < s + N; i++) m += e[i]; return ((m / N) * 100) / 60 / (poles / 2); });
        motorByBin[b].push(median(hz));
      }
      for (const k of keys) {
        for (let a = 0; a < 3; a++) {
          const x = sigs[k][a];
          if (!x) continue;
          let mean = 0;
          for (let i = 0; i < N; i++) mean += x[s + i];
          mean /= N;
          for (let i = 0; i < N; i++) { re[i] = (x[s + i] - mean) * win[i]; im[i] = 0; }
          fft(re, im, false);
          const A = acc[k][a], T = total[k][a], off = b * F;
          for (let f = 0; f < F; f++) { const p = (re[f] * re[f] + im[f] * im[f]) / wsum; A[off + f] += p; T[f] += p; }
        }
      }
    }

    // line up D-term input with output at low frequency (where the D filters pass almost everything),
    // so both maps share one colour scale and the difference is what the filters removed
    const fRes = fs / N;
    const band = [Math.max(1, Math.round(CFG.refLo / fRes)), Math.max(2, Math.round(CFG.refHi / fRes))];
    const dGain = [0, 1, 2].map((a) => {
      if (!total.dIn[a] || !total.dOut[a]) return null;
      let pi = 0, po = 0;
      for (let f = band[0]; f <= band[1]; f++) { pi += total.dIn[a][f]; po += total.dOut[a][f]; }
      return pi > 0 ? po / pi : null;
    });
    for (let a = 0; a < 3; a++) if (dGain[a] && acc.dIn[a]) {
      const A = acc.dIn[a], T = total.dIn[a];
      for (let i = 0; i < A.length; i++) A[i] *= dGain[a];
      for (let f = 0; f < F; f++) T[f] *= dGain[a];
    }

    // mean power per cell, in dB
    const maps = {};
    for (const k of keys) {
      maps[k] = acc[k].map((A) => {
        if (!A) return null;
        const out = new Float32Array(TB * F).fill(NaN);
        for (let b = 0; b < TB; b++) {
          if (!cnt[b]) continue;
          for (let f = 0; f < F; f++) out[b * F + f] = 10 * Math.log10(A[b * F + f] / cnt[b] + 1e-12);
        }
        // short flights leave empty throttle rows between used ones: bridge gaps of up to 3 rows
        let prev = -1;
        for (let b = 0; b < TB; b++) {
          if (!cnt[b]) continue;
          if (prev >= 0 && b - prev > 1 && b - prev <= 4)
            for (let g = prev + 1; g < b; g++) {
              const w = (g - prev) / (b - prev);
              for (let f = 0; f < F; f++) out[g * F + f] = out[prev * F + f] * (1 - w) + out[b * F + f] * w;
            }
          prev = b;
        }
        return out;
      });
    }
    const spectra = {};
    for (const k of keys) spectra[k] = total[k].map((T) => (T && used ? Float32Array.from(T, (p) => 10 * Math.log10(p / used + 1e-12)) : null));

    const motorHz = new Float32Array(TB).fill(NaN);
    if (hasErpm) for (let b = 0; b < TB; b++) if (motorByBin[b].length) motorHz[b] = median(motorByBin[b]);
    // same gap bridging as the maps, so the lines stay continuous
    for (let b = 0, prev = -1; b < TB; b++) {
      if (!isFinite(motorHz[b])) continue;
      if (prev >= 0 && b - prev > 1 && b - prev <= 4) for (let g = prev + 1; g < b; g++) motorHz[g] = motorHz[prev] + ((motorHz[b] - motorHz[prev]) * (g - prev)) / (b - prev);
      prev = b;
    }
    const motorRange = (() => {
      if (!hasErpm) return null;
      const all = [].concat(...motorByBin).sort((x, y) => x - y);
      if (!all.length) return null;
      return [all[Math.floor(all.length * 0.1)], all[Math.floor(all.length * 0.9)]];
    })();

    const findings = [0, 1, 2].map((a) => findAxis(a, maps, spectra, motorHz, F, fRes, cnt, hasUnf));

    return {
      fs, N, F, fRes, thrBins: TB, counts: cnt, used, skipped,
      maps, motorHz, motorRange,
      info: { hasUnf, hasErpm, motors: erpm.length, poles, rpmH, rpmMin, dLpf, hasD: dOut.map(Boolean), dGain },
      findings,
    };
  }

  // Strongest noise peak on an axis: where it sits, whether it follows the motors, how much the filters took off
  function findAxis(a, maps, spectra, motorHz, F, fRes, cnt, hasUnf) {
    const pre = hasUnf ? spectra.gyroUnf[a] : spectra.gyro[a];
    const post = spectra.gyro[a];
    if (!pre) return null;
    const lo = Math.ceil(CFG.peakMinHz / fRes);
    if (lo >= F - 2) return null;
    let fp = lo;
    for (let f = lo; f < F - 1; f++) if (pre[f] > pre[fp]) fp = f;
    // floor = median level above lo
    const floor = median(Array.from(pre.subarray(lo)));
    const prom = pre[fp] - floor;
    const res = { hz: Math.round(fp * fRes), prom: +prom.toFixed(1), cut: hasUnf ? +(pre[fp] - post[fp]).toFixed(1) : null, kind: "none" };
    if (prom < 6) { res.kind = "clean"; return res; }
    // throttle row where this frequency is loudest
    const map = hasUnf ? maps.gyroUnf[a] : maps.gyro[a];
    let bb = -1, bv = -Infinity;
    for (let b = 0; b < cnt.length; b++) if (cnt[b] >= 3 && map[b * F + fp] > bv) { bv = map[b * F + fp]; bb = b; }
    res.thr = bb >= 0 ? Math.round(((bb + 0.5) / cnt.length) * 100) : null;
    // does the noise move with throttle? compare peak frequency in low vs high throttle rows
    if (!motorHz.some(isFinite) || bb < 0 || !isFinite(motorHz[bb])) { res.kind = "unknown"; return res; }
    res.kind = "other";
    for (let h = 1; h <= 4; h++) {
      const hf = motorHz[bb] * h;
      if (Math.abs(fp * fRes - hf) <= hf * CFG.harmTol) { res.kind = "motor"; res.harm = h; break; }
    }
    return res;
  }

  root.GGNoise = { analyze, CFG };
})(typeof window !== "undefined" ? window : globalThis);
