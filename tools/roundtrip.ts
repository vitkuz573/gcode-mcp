/**
 * Closed-loop verification: reconstruct a model from gcode, re-slice the
 * reconstruction, and compare that gcode against the original.
 *
 * This is the real test of a reconstruction. Comparing meshes directly rewards
 * a result that merely has the right bounding box; re-slicing asks the hard
 * question "would this print the same part?", because a wrong contour changes
 * the toolpath, the layer count and the material budget.
 */
import { execFileSync } from "node:child_process";
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { basename, dirname, extname, join, resolve } from "node:path";
import { inflateRawSync } from "node:zlib";

import { parseFile } from "../src/parse.js";
import { buildMesh } from "../src/mesh.js";

/**
 * Work directory.
 *
 * Must sit on the Windows filesystem: OrcaSlicer cannot read /home, and reports
 * a missing input as a bare exit code with no diagnostic. Note that `~` in WSL
 * is /home/<user>, not C:\Users\<user>, so the path is spelled out.
 */
const WORK = resolve(
  process.env.BENCH_WORK ?? "/mnt/c/Users/vitaly/AppData/Local/Temp/opencode/gcode-corpus"
);
const SLICER_DIST = resolve((process.env.SLICER_DIST ?? "~/slicer-mcp/dist").replace("~", process.env.HOME ?? "~"));

function gcodeFrom3mf(path: string): Buffer | null {
  const buf = readFileSync(path);
  let eocd = -1;
  for (let i = buf.length - 22; i >= 0 && i > buf.length - 66000; i--) {
    if (buf.readUInt32LE(i) === 0x06054b50) {
      eocd = i;
      break;
    }
  }
  if (eocd < 0) return null;
  const count = buf.readUInt16LE(eocd + 10);
  let off = buf.readUInt32LE(eocd + 16);
  for (let i = 0; i < count; i++) {
    if (buf.readUInt32LE(off) !== 0x02014b50) break;
    const method = buf.readUInt16LE(off + 10);
    const compSize = buf.readUInt32LE(off + 20);
    const nameLen = buf.readUInt16LE(off + 28);
    const extraLen = buf.readUInt16LE(off + 30);
    const commentLen = buf.readUInt16LE(off + 32);
    const localOff = buf.readUInt32LE(off + 42);
    const name = buf.toString("utf8", off + 46, off + 46 + nameLen);
    if (/\.gcode$/i.test(name)) {
      const lName = buf.readUInt16LE(localOff + 26);
      const lExtra = buf.readUInt16LE(localOff + 28);
      const start = localOff + 30 + lName + lExtra;
      const raw = buf.subarray(start, start + compSize);
      return method === 0 ? Buffer.from(raw) : inflateRawSync(raw);
    }
    off += 46 + nameLen + extraLen + commentLen;
  }
  return null;
}

/** WSL path -> Windows path, for the slicer process. */
function toWin(p: string): string {
  if (/^[A-Za-z]:\\/.test(p)) return p;
  if (p.startsWith("/mnt/c/")) return "C:\\" + p.slice(7).replace(/\//g, "\\");
  if (p.startsWith("/")) return "\\\\wsl.localhost\\Ubuntu" + p.replace(/\//g, "\\");
  return p;
}

function sliceGcode(model: string, out3mf: string): string {
  const script = `
    const { configFromEnv, sliceModel } = await import(${JSON.stringify(`${SLICER_DIST}/orca.js`)});
    const r = await sliceModel(configFromEnv(), {
      model: ${JSON.stringify(toWin(model))},
      output: ${JSON.stringify(toWin(out3mf))},
      thumbnail: false,
      overrides: { retraction_distances_when_cut: 10 },
    });
    console.log(JSON.stringify({ ok: r.ok, tail: r.logTail.slice(-400) }));
  `;
  return execFileSync("node", ["--input-type=module", "-e", script], {
    encoding: "utf8",
    maxBuffer: 1024 * 1024 * 64,
  });
}

async function main() {
  const models = process.argv.slice(2).filter((a) => !a.startsWith("--"));
  if (models.length === 0) {
    console.error("usage: node roundtrip.js <model.stl> ...");
    process.exit(1);
  }
  mkdirSync(WORK, { recursive: true });

  for (const model of models) {
    const name = basename(model, extname(model)).replace(/[^a-zA-Z0-9._-]/g, "_");
    console.log(`\n=== ${name} ===`);
    try {
      // 1. Slice the original -> gcode A.
      const a3mf = join(WORK, `${name}.A.3mf`);
      const sliceA = JSON.parse(sliceGcode(resolve(model), a3mf));
      if (!sliceA.ok) throw new Error(`slice A failed: ${sliceA.tail}`);
      const gcodeA = gcodeFrom3mf(a3mf);
      if (!gcodeA) throw new Error("no gcode in A");
      const pathA = join(WORK, `${name}.A.gcode`);
      writeFileSync(pathA, gcodeA);

      // 2. Reconstruct from gcode A.
      const parsedA = await parseFile(pathA);
      const { mesh, report } = buildMesh(parsedA, { mode: "shell", weldTol: 0, mergeWalls: false });
      const stl = join(WORK, `${name}.recon.stl`);
      writeFileSync(stl, mesh.toStlBinary());
      console.log(
        `  reconstruct: tris=${report.validation.triangles} bnd=${report.validation.boundaryEdges} ` +
          `nm=${report.validation.nonmanifoldEdges} watertight=${report.validation.watertight}`
      );

      // 3. Re-slice the reconstruction -> gcode B.
      const b3mf = join(WORK, `${name}.B.3mf`);
      const sliceB = JSON.parse(sliceGcode(stl, b3mf));
      if (!sliceB.ok) throw new Error(`slice B failed: ${sliceB.tail}`);
      const gcodeB = gcodeFrom3mf(b3mf);
      if (!gcodeB) throw new Error("no gcode in B");
      const pathB = join(WORK, `${name}.B.gcode`);
      writeFileSync(pathB, gcodeB);

      // 4. Compare the two gcodes.
      const parsedB = await parseFile(pathB);
      const segA = parsedA.layers.reduce((s, l) => s + l.segs.length, 0);
      const segB = parsedB.layers.reduce((s, l) => s + l.segs.length, 0);
      const lenA = Object.values(parsedA.featureLen).reduce((s, v) => s + v, 0);
      const lenB = Object.values(parsedB.featureLen).reduce((s, v) => s + v, 0);
      const boxA = bbox(parsedA);
      const boxB = bbox(parsedB);
      const pct = (a: number, b: number) => (a > 0 ? ((b - a) / a) * 100 : 0);

      console.log(
        `  layers   A=${parsedA.layers.length} B=${parsedB.layers.length} ` +
          `(${pct(parsedA.layers.length, parsedB.layers.length).toFixed(2)}%)`
      );
      console.log(
        `  segments A=${segA} B=${segB} (${pct(segA, segB).toFixed(2)}%)  ` +
          `extrusion A=${lenA.toFixed(0)}mm B=${lenB.toFixed(0)}mm (${pct(lenA, lenB).toFixed(2)}%)`
      );
      // The slicer may re-orient the part on the plate, so compare dimensions
      // as a sorted set rather than axis by axis.
      const sizeA = [boxA[3] - boxA[0], boxA[4] - boxA[1], boxA[5] - boxA[2]].sort((x, y) => x - y);
      const sizeB = [boxB[3] - boxB[0], boxB[4] - boxB[1], boxB[5] - boxB[2]].sort((x, y) => x - y);
      console.log(
        `  size (sorted) A=${sizeA.map((v) => v.toFixed(2)).join(" x ")}  ` +
          `B=${sizeB.map((v) => v.toFixed(2)).join(" x ")}  ` +
          `delta=${sizeB.map((v, k) => (v - sizeA[k]).toFixed(2)).join(" / ")}`
      );
      const fa = normCounts(parsedA);
      const fb = normCounts(parsedB);
      const keys = [...new Set([...Object.keys(fa), ...Object.keys(fb)])].sort();
      const drift = keys
        .map((k) => ({ k, a: fa[k] ?? 0, b: fb[k] ?? 0, d: pct(fa[k] ?? 0, fb[k] ?? 0) }))
        .filter((r) => Math.abs(r.d) > 5)
        .sort((a, b) => Math.abs(b.d) - Math.abs(a.d));
      if (drift.length === 0) {
        console.log("  features: no drift > 5%");
      } else {
        console.log("  feature drift:");
        for (const r of drift.slice(0, 8)) {
          console.log(`    ${r.k.padEnd(24)} A=${r.a} B=${r.b} (${r.d.toFixed(1)}%)`);
        }
      }
    } catch (e) {
      console.log(`  FAILED: ${e instanceof Error ? e.message : String(e)}`);
    }
  }
}

function normCounts(p: ReturnType<typeof parseFile> extends Promise<infer T> ? T : never) {
  return Object.fromEntries(
    Object.entries(p.featureCounts).map(([k, v]) => [k.toLowerCase(), v])
  );
}

function bbox(p: { layers: Array<{ segs: Array<{ x0: number; x1: number; y0: number; y1: number; z0: number; z1: number }> }> }) {
  let a = [Infinity, Infinity, Infinity];
  let b = [-Infinity, -Infinity, -Infinity];
  for (const l of p.layers) {
    for (const s of l.segs) {
      a = [Math.min(a[0], s.x0, s.x1), Math.min(a[1], s.y0, s.y1), Math.min(a[2], s.z0, s.z1)];
      b = [Math.max(b[0], s.x0, s.x1), Math.max(b[1], s.y0, s.y1), Math.max(b[2], s.z0, s.z1)];
    }
  }
  return [a[0], a[1], a[2], b[0], b[1], b[2]];
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});