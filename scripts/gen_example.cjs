// Regenerates src/example.json from two logs.
// usage: PARSER=path/to/parser.bundle.js node scripts/gen_example.cjs stock_tune.BFL good_tune.BBL
globalThis.window = globalThis;
const fs = require("fs");
const path = require("path");
eval(fs.readFileSync(process.env.PARSER || "parser.bundle.js", "utf8"));
eval(fs.readFileSync(path.join(__dirname, "../src/core.js"), "utf8"));

const ds = (a, dt) => {
  if (!a) return null;
  const o = [];
  for (let ms = 0; ms <= 500; ms++) o.push(+a[Math.min(a.length - 1, Math.round(ms / 1000 / dt))].toFixed(4));
  return o;
};
const out = [];
for (const f of process.argv.slice(2)) {
  const l = GGStep.loadLogs(new Uint8Array(fs.readFileSync(f))).filter((x) => !x.error)[0];
  const r = GGStep.analyze(l);
  out.push({
    name: path.basename(f), logIndex: l.index, header: l.header, duration: l.duration, rate: r.rate,
    health: (({ gapThr, ...h }) => h)(l.health), gapWin: r.gapWin, totalWin: r.axes[0].total,
    axes: r.axes.map((a) => ({
      resp: ds(a.resp, a.dt), sd: ds(a.sd, a.dt), respHigh: ds(a.respHigh, a.dt), sdHigh: ds(a.sdHigh, a.dt),
      n: a.n, nHigh: a.nHigh, windows: a.windows, total: a.total, m: a.m, mHigh: a.mHigh, ff: a.ff,
    })),
  });
}
fs.writeFileSync(path.join(__dirname, "../src/example.json"), JSON.stringify(out));
console.log("example.json written");
