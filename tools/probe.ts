/**
 * Dev harness: reconstruct a gcode and compare against the original mesh.
 *
 * Not part of the MCP surface; run with `npm run bench -- <gcode> [reference]`.
 */
import { existsSync, statSync } from "node:fs";
import { basename } from "node:path";

import { bboxOf, objectsInUse, parseFile } from "../src/parse.js";
import { buildMesh, round, type BuildOptions } from "../src/mesh.js";
import { compareMeshes, loadMesh, meshReport } from "../src/meshio.js";
import { writeFileSync } from "node:fs";

async function main() {
  const [gcode, reference, ...rest] = process.argv.slice(2);
  if (!gcode) {
    console.error("usage: npm run bench -- <gcode> [reference.stl|3mf] [--layer-step N] [--mode M]");
    process.exit(1);
  }
  const flags: Record<string, string> = {};
  for (let i = 0; i < rest.length; i += 2) {
    flags[rest[i].replace(/^--/, "")] = rest[i + 1];
  }

  const t0 = Date.now();
  const p = await parseFile(gcode);
  const tParse = Date.now() - t0;

  const b = bboxOf(p);
  console.log(
    `parse: ${tParse}ms  layers=${p.layers.length}/${p.declaredLayers ?? "?"}  ` +
      `segs=${p.layers.reduce((s, l) => s + l.segs.length, 0)}  objects=${objectsInUse(p).join(",")}`
  );
  console.log(
    `bbox : ${[b[3] - b[0], b[4] - b[1], b[5] - b[2]].map((v) => round(v, 2)).join(" x ")} mm`
  );

  const opts: BuildOptions = {
    mode: (flags.mode as BuildOptions["mode"]) ?? "shell",
    layerStep: flags["layer-step"] ? parseInt(flags["layer-step"], 10) : 1,
    weldTol: flags["weld-tol"] ? parseFloat(flags["weld-tol"]) : 0.01,
  };

  const t1 = Date.now();
  const { mesh, report } = buildMesh(p, opts);
  const tBuild = Date.now() - t1;

  const v = report.validation;
  console.log(
    `build: ${tBuild}ms  tris=${v.triangles}  vol=${v.signedVolumeCm3} cm3  ` +
      `boundary=${v.boundaryEdges}  nonmanifold=${v.nonmanifoldEdges}  watertight=${v.watertight}`
  );
  console.log(
    `       layers_used=${report.layersUsed}/${report.layersTotal}  islands=${report.islands}  ` +
      `loops=${report.loops}  skipped=${report.skippedLayers}  collapsed=${report.weld.collapsedTriangles}`
  );

  if (flags.out) {
    writeFileSync(flags.out, mesh.toStlBinary());
    console.log(`wrote ${flags.out} (${statSync(flags.out).size} bytes)`);
  }

  if (reference && existsSync(reference)) {
    const t2 = Date.now();
    const ref = loadMesh(reference);
    const target = loadMesh(flags.out ?? reference);
    console.log(
      `ref  : ${basename(reference)}  ${meshReport(ref, "reference").triangles} tris  ` +
        `${meshReport(ref, "reference").volumeMm3} mm3`
    );
    const cmp = compareMeshes(ref, target, Number(flags.samples ?? 40000));
    console.log(`cmp  : ${Date.now() - t2}ms`);
    console.log(
      `       dist mean=${cmp.distanceMm.mean} rms=${cmp.distanceMm.rms} ` +
        `p95=${cmp.distanceMm.p95} max=${cmp.distanceMm.max} mm`
    );
    console.log(
      `       size d=${cmp.sizeMm.delta.join("/")} mm  ` +
        `vol ${cmp.volumeMm3.a}->${cmp.volumeMm3.b} (${cmp.volumeMm3.deltaPct}%)`
    );
    if (cmp.warning) console.log(`       warn: ${cmp.warning}`);
  }
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});