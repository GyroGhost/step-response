globalThis.window=globalThis;const fs=require('fs');
// usage: node scripts/gen_example.js (needs dist parser bundle; see README)
eval(fs.readFileSync(process.env.PARSER||'parser.bundle.js','utf8'));eval(fs.readFileSync(__dirname+'/../src/core.js','utf8'));
const out=[];
for (const [f,name] of [['/tmp/pida/stock_tune.BFL','stock_tune.BFL'],['/tmp/pida/good_tune.BBL','good_tune.BBL']]) {
  const l=GGStep.loadLogs(new Uint8Array(fs.readFileSync(f))).filter(x=>!x.error)[0];
  const r=GGStep.analyze(l);
  const ds=(a,dt)=>{ if(!a) return null; const o=[]; for(let ms=0;ms<=500;ms++){const i=Math.min(a.length-1,Math.round(ms/1000/dt)); o.push(+a[i].toFixed(4));} return o;};
  out.push({name,logIndex:l.index,header:l.header,duration:l.duration,rate:r.rate,
    axes:r.axes.map(a=>({resp:ds(a.resp,a.dt),respHigh:ds(a.respHigh,a.dt),windows:a.windows,total:a.total,m:a.m}))});
}
fs.writeFileSync('example.json',JSON.stringify(out));
console.log(fs.statSync('example.json').size);
