// Builds index.html: bundles the Blackbox Explorer parser and inlines everything into one page.
import * as esbuild from "esbuild";
import fs from "fs";
import path from "path";

const root = path.resolve(path.dirname(new URL(import.meta.url).pathname), "..");
const bbsrc = path.join(root, "vendor/blackbox-log-viewer/src");
if (!fs.existsSync(bbsrc)) {
  console.error("Parser source missing. Run: npm run fetch-parser");
  process.exit(1);
}

const out = await esbuild.build({
  stdin: {
    contents: `import { FlightLogParser } from ${JSON.stringify(path.join(bbsrc, "flightlog_parser.js"))};\nwindow.BBParser = FlightLogParser;`,
    resolveDir: root,
  },
  bundle: true, format: "iife", minify: true, write: false, legalComments: "none",
  plugins: [{
    name: "stub-presenter",
    setup(b) {
      // The field presenter pulls in UI/store code we don't need; the parser only calls one no-op on it.
      b.onResolve({ filter: /flightlog_fields_presenter/ }, () => ({ path: path.join(root, "src/presenter_stub.js") }));
    },
  }],
});
const parser = out.outputFiles[0].text;
const core = fs.readFileSync(path.join(root, "src/core.js"), "utf8");
const example = fs.readFileSync(path.join(root, "src/example.json"), "utf8");
for (const [n, s] of [["parser", parser], ["core", core], ["example", example]]) {
  if (/<\/script/i.test(s)) throw new Error(n + " contains </script");
}
let page = fs.readFileSync(path.join(root, "src/page.src.html"), "utf8");
page = page.replace("/*PARSER*/", () => parser).replace("/*CORE*/", () => core).replace("/*EXAMPLE*/", () => example);
page = "<!doctype html>\n<html>\n<head>\n<meta charset=\"utf-8\">\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n</head>\n<body>\n" + page + "\n</body>\n</html>\n";
fs.writeFileSync(path.join(root, "index.html"), page);
console.log("index.html written (" + (page.length / 1024).toFixed(0) + " KB)");
