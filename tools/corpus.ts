/**
 * Batch harness: slice each model, reconstruct from the gcode, compare against
 * the original mesh.
 *
 * Calls slicer-mcp's compiled slicer module directly (same code path the MCP
 * tool uses) so a whole corpus runs unattended, then prints one line per model
 * plus a summary of what still fails.
 */
import { execFileSync } from "node:child_process";
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { basename, dirname, extname, join, resolve } from "node:path";
import { inflateRawSync } from "node:zlib";

import { parseFile } from "../src/parse.js";
import { buildMesh, type BuildOptions } from "../src/mesh.js";
import { compareMeshes, load3mf, loadStl } from "../src/meshio.js";

const WORK = resolve(
  process.env.BENCH_WORK ?? "/mnt/c/Users/vitaly/AppData/Local/Temp/opencode/gcode-corpus"
);
const SLICER_DIST = resolve((process.env.SLICER_DIST ?? "~/slicer-mcp/dist").replace("~", process.env.HOME ?? "~"));

interface Case {
  name: string;
  model: string;
  layerHeight?: number;
}

/** Extract the first .gcode entry from a 3mf archive. */
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

async function main() {
  const argv = process.argv.slice(2);
  const opts: Record<string, string> = {};
  const models: string[] = [];
  for (let i = 0; i < argv.length; i++) {
    if (argv[i].startsWith("--")) opts[argv[i].slice(2)] = argv[i + 1] ?? "";
    else models.push(argv[i]);
  }
  if (models.length === 0) {
    console.error("usage: node corpus.js [--mode shell] [--layer-height 0.2] <model.stl> ...");
    process.exit(1);
  }
  const mode = (opts.mode ?? "shell") as BuildOptions["mode"];
  const layerHeight = opts["layer-height"] ? parseFloat(opts["layer-height"]) : undefined;

  const { configFromEnv, sliceModel } = await import(`${SLICER_DIST}/orca.js`);
  const cases: Case[] = models.map((m) => ({
    name: basename(m, extname(m)).replace(/[^a-zA-Z0-9._-]/g, "_"),
    model: resolve(m),
    layerHeight,
  }));

  mkdirSync(WORK, { recursive: true });
  const rows: Array<Record<string, unknown>> = [];

  for (const c of cases) {
    const row: Record<string, unknown> = { name: c.name, model: c.model };
    const out3mf = join(WORK, `${c.name}.3mf`);
    const gcodePath = join(WORK, `${c.name}.gcode`);
    try {
      const t0 = Date.now();
      const req: Record<string, unknown> = {
        model: c.model,
        output: out3mf,
        thumbnail: false,
      };
      // The bundled Generic PETG filament leaves retraction_distances_when_cut
      // at 0 while the machine preset constrains it to [10, 18], which the
      // Orca CLI rejects before slicing. Satisfy the constraint.
      const ov: Record<string, unknown> = { retraction_distances_when_cut: 10 };
      if (c.layerHeight) ov.layer_height = c.layerHeight;
      req.overrides = ov;
      const res = await sliceModel(configFromEnv(), req as never);
      row.sliceMs = Date.now() - t0;
      if (!res.ok || !existsSync(out3mf)) {
        throw new Error(`slice failed: ${res.logTail.slice(-200)}`);
      }

      const gcode = gcodeFrom3mf(out3mf);
      if (!gcode) throw new Error("no .gcode inside the sliced 3mf");
      writeFileSync(gcodePath, gcode);

      const t1 = Date.now();
      const parsed = await parseFile(gcodePath);
      const { mesh, report } = buildMesh(parsed, { mode, weldTol: 0, mergeWalls: false });
      row.buildMs = Date.now() - t1;
      row.layers = report.layersUsed;
      row.skipped = report.skippedLayers;
      row.tris = report.validation.triangles;
      row.boundary = report.validation.boundaryEdges;
      row.nonmanifold = report.validation.nonmanifoldEdges;
      row.watertight = report.validation.watertight;
      row.wind = report.validation.consistentWinding;
      row.note = report.notes[0];

      const stlPath = join(WORK, `${c.name}.recon.stl`);
      writeFileSync(stlPath, mesh.toStlBinary());

      const ref =
        extname(c.model).toLowerCase() === ".stl" ? loadStl(c.model) : load3mf(c.model);
      row.refTris = ref.nTris;
      const cmp = compareMeshes(ref, loadStl(stlPath), 20000);
      row.dMean = cmp.distanceMm.mean;
      row.dP95 = cmp.distanceMm.p95;
      row.dMax = cmp.distanceMm.max;
      row.volDeltaPct = cmp.volumeMm3.deltaPct;
      row.sizeDelta = cmp.sizeMm.delta.join("/");
      if (cmp.warning) row.warn = cmp.warning;
      row.status = report.validation.watertight
        ? "watertight"
        : report.validation.boundaryEdges === 0
          ? "closed"
          : "OPEN";
    } catch (e) {
      row.status = "FAILED";
      row.error = e instanceof Error ? e.message : String(e);
    }
    rows.push(row);
    console.log(
      `${String(row.status).padEnd(10)} ${c.name.padEnd(26)} ` +
        `L=${row.layers ?? "-"} tris=${row.tris ?? "-"} bnd=${row.boundary ?? "-"} ` +
        `nm=${row.nonmanifold ?? "-"} dMean=${row.dMean ?? "-"} dP95=${row.dP95 ?? "-"} ` +
        `vol%=${row.volDeltaPct ?? "-"} ${row.sliceMs ?? 0}/${row.buildMs ?? 0}ms` +
        (row.error ? ` ERR=${String(row.error).slice(0, 90)}` : "")
    );
  }

  console.log("\n--- summary ---");
  const count = (s: string) => rows.filter((r) => r.status === s).length;
  console.log(
    `watertight=${count("watertight")} closed=${count("closed")} open=${count("OPEN")} ` +
      `failed=${count("FAILED")} total=${rows.length}`
  );
  writeFileSync(join(WORK, "report.json"), JSON.stringify(rows, null, 2));
  console.log(`report: ${join(WORK, "report.json")}`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});