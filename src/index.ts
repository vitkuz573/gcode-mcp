#!/usr/bin/env node
/**
 * gcode-mcp: reconstruct printable 3D models from sliced gcode.
 *
 * Inspect a slice, pull the model back out of it, and check the result against
 * the original mesh. Pure TypeScript: no native deps, no Python.
 *
 * Works with OrcaSlicer / AnycubicSlicerNext / PrusaSlicer / Cura gcode.
 */
import { McpServer } from "@modelcontextprotocol/server";
import { StdioServerTransport } from "@modelcontextprotocol/server/stdio";
import { existsSync, mkdirSync, statSync, writeFileSync } from "node:fs";
import { basename, dirname, extname, join, resolve } from "node:path";
import * as z from "zod";

import { bboxOf, objectsInUse, parseFile, type ParsedPrint } from "./parse.js";
import { buildMesh, round, type BuildOptions } from "./mesh.js";
import { compareMeshes, loadMesh, meshReport } from "./meshio.js";
import { Canvas } from "./png.js";

const server = new McpServer(
  { name: "gcode-mcp", version: "0.1.0" },
  { capabilities: { tools: {} } }
);

function asText(obj: unknown) {
  return { content: [{ type: "text" as const, text: JSON.stringify(obj, null, 2) }] };
}

function errText(e: unknown) {
  const msg = e instanceof Error ? e.message : String(e);
  return { content: [{ type: "text" as const, text: `Error: ${msg}` }], isError: true };
}

function requireFile(path: string): string {
  const p = resolve(path);
  if (!existsSync(p)) throw new Error(`file not found: ${path}`);
  return p;
}

/** Expand a Windows path (C:\...) into a WSL path when running under WSL. */
function normalizePath(input: string): string {
  const win = /^([A-Za-z]):\\(.*)$/.exec(input);
  if (win && process.platform === "linux") {
    const candidate = `/mnt/${win[1].toLowerCase()}/${win[2].replace(/\\/g, "/")}`;
    if (existsSync(candidate)) return candidate;
  }
  return input;
}

function defaultOut(gcode: string, suffix: string): string {
  const dir = dirname(resolve(gcode));
  const base = basename(gcode, extname(gcode));
  return join(dir, `${base}_${suffix}`);
}

function featureHistogram(p: ParsedPrint): Record<string, number> {
  return Object.fromEntries(
    Object.entries(p.featureCounts).sort((a, b) => b[1] - a[1])
  );
}

function featureLengths(p: ParsedPrint): Record<string, number> {
  return Object.fromEntries(
    Object.entries(p.featureLen)
      .sort((a, b) => b[1] - a[1])
      .map(([k, v]) => [k, round(v, 1)])
  );
}

function summarize(p: ParsedPrint): Record<string, unknown> {
  const b = bboxOf(p);
  return {
    file: p.path,
    sizeBytes: statSync(p.path).size,
    slicer: p.slicer,
    lines: p.totalLines,
    moves: p.moveCount,
    declaredLayers: p.declaredLayers,
    parsedLayers: p.layers.length,
    layersWithExtrusion: p.layers.filter((l) => l.segs.length > 0).length,
    extrusionSegments: p.layers.reduce((s, l) => s + l.segs.length, 0),
    objects: objectsInUse(p),
    featureCounts: featureHistogram(p),
    featureTotalMm: featureLengths(p),
    bbox: {
      x: [round(b[0], 2), round(b[3], 2)],
      y: [round(b[1], 2), round(b[4], 2)],
      z: [round(b[2], 2), round(b[5], 2)],
      size: [round(b[3] - b[0], 2), round(b[4] - b[1], 2), round(b[5] - b[2], 2)],
    },
    hasThumbnail: p.thumbnail !== null,
    thumbnailSize: p.thumbnailSize,
    keySettings: keySettings(p),
    warnings: p.warnings,
  };
}

const SETTING_KEYS = new Set([
  "layer_height",
  "first_layer_height",
  "nozzle_diameter",
  "filament_type",
  "temperature",
  "first_layer_temperature",
  "bed_temperature",
  "first_layer_bed_temperature",
  "material_used",
  "material_used_g",
  "estimated_printing_time",
  "perimeters",
  "wall_loops",
  "top_shell_layers",
  "bottom_shell_layers",
  "fill_density",
  "fill_pattern",
  "sparse_infill_density",
  "support_material",
  "support_enable",
  "support_type",
  "printer_model",
  "nozzle_type",
  "spiral_vase_mode",
]);

function keySettings(p: ParsedPrint): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [k, v] of Object.entries(p.settings)) {
    const lk = k.trim().toLowerCase();
    if (SETTING_KEYS.has(lk)) out[lk] = v.trim();
  }
  return out;
}

// ---------------------------------------------------------------- info

server.registerTool(
  "gcode_info",
  {
    description:
      "Inspect a sliced gcode file: slicer and version, layer counts, objects, extrusion feature histogram, bounding box, embedded thumbnail, and the original slice settings. Read-only.",
    inputSchema: {
      gcode: z.string().describe("Path to .gcode (Windows C:\\... or WSL /home/...)"),
      maxLayers: z
        .number()
        .int()
        .positive()
        .optional()
        .describe("Parse only the first N layers (fast preview on huge files)"),
    },
    annotations: { readOnlyHint: true },
  },
  async (args) => {
    try {
      const a = args as { gcode: string; maxLayers?: number };
      const path = requireFile(normalizePath(a.gcode));
      const p = await parseFile(path, { maxLayers: a.maxLayers });
      return asText(summarize(p));
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- thumbnail

server.registerTool(
  "gcode_thumbnail",
  {
    description:
      "Extract the thumbnail image embedded in a gcode header (base64 PNG block) and write it to a .png.",
    inputSchema: {
      gcode: z.string(),
      out: z.string().optional().describe("Output PNG path; defaults next to the gcode"),
    },
  },
  async (args) => {
    try {
      const a = args as { gcode: string; out?: string };
      const path = requireFile(normalizePath(a.gcode));
      const p = await parseFile(path, { maxLayers: 1 });
      if (!p.thumbnail) {
        return asText({
          ok: false,
          reason: "no embedded thumbnail in gcode",
          hint: "run gcode_preview to render one from the toolpath instead",
        });
      }
      const out = a.out ? resolve(normalizePath(a.out)) : `${defaultOut(path, "thumb")}.png`;
      writeFileSync(out, p.thumbnail);
      return asText({
        ok: true,
        out,
        bytes: p.thumbnail.length,
        size: p.thumbnailSize,
      });
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- to_model

server.registerTool(
  "gcode_to_model",
  {
    description:
      "Reconstruct a 3D model (STL/3MF) from sliced gcode by tracing per-layer toolpath contours into islands and meshing them. mode=shell keeps only part-forming features (perimeters, top/bottom, solid infill) and drops supports and sparse infill; solid adds sparse infill; outer keeps only the outer wall; all keeps everything including supports.",
    inputSchema: {
      gcode: z.string(),
      mode: z
        .enum(["shell", "solid", "outer", "no_support", "all"])
        .default("shell")
        .describe("Which extrusion features to reconstruct"),
      format: z.enum(["stl", "3mf", "both"]).default("stl"),
      out: z.string().optional().describe("Output path without extension"),
      objects: z
        .string()
        .optional()
        .describe("Comma-separated object names to keep (from gcode_info)"),
      weldTol: z
        .number()
        .min(0)
        .default(0)
        .describe(
          "Vertex weld tolerance in mm. 0 (default) keeps the shell closed; " +
            "small values fuse nearby contours and can open the mesh."
        ),
      splitNonManifold: z
        .boolean()
        .default(false)
        .describe("Give surplus faces their own vertex copies to clear non-manifold edges"),
      snapGrid: z
        .number()
        .min(0)
        .default(0)
        .describe("Snap contour points to a global XY lattice of this size in mm"),
      simplify: z
        .number()
        .min(0)
        .default(0)
        .describe("Douglas-Peucker tolerance in mm for contour simplification"),
      capMode: z
        .enum(["all", "column"])
        .default("all")
        .describe("all: cap every layer. column: cap only exposed faces (fewer internal faces)"),
      layerStep: z
        .number()
        .int()
        .positive()
        .default(1)
        .describe("Use every Nth layer. >1 is a fast low-res preview; scale Z with layerHeight"),
      layerHeight: z
        .number()
        .positive()
        .optional()
        .describe("Layer height in mm; required to keep layerStep previews true-scale in Z"),
      eps: z.number().positive().default(0.02).describe("Contour endpoint snap distance in mm"),
      minArea: z.number().min(0).default(0.05).describe("Discard loops smaller than this mm^2"),
    },
  },
  async (args) => {
    try {
      const a = args as Record<string, unknown>;
      const path = requireFile(normalizePath(String(a.gcode)));
      const p = await parseFile(path);

      let objects: string[] | undefined;
      if (a.objects) {
        objects = String(a.objects)
          .split(",")
          .map((s) => s.trim())
          .filter(Boolean);
        const have = new Set(objectsInUse(p));
        const missing = objects.filter((o) => !have.has(o));
        if (missing.length) {
          return asText({ ok: false, error: `objects not found: ${missing}`, available: [...have] });
        }
      }

      const layerStep = Number(a.layerStep ?? 1);
      const layerHeight = a.layerHeight ? Number(a.layerHeight) : undefined;
      if (layerStep > 1 && layerHeight) {
        // Keep the preview proportional: sampled layers are spaced layerStep
        // apart, so scale the extrusion height to match.
        for (const l of p.layers) {
          l.height *= layerStep;
          l.bottom = l.z - l.height;
          l.top = l.z;
        }
      }

      const opts: BuildOptions = {
        mode: (a.mode as BuildOptions["mode"]) ?? "shell",
        objects,
        eps: a.eps !== undefined ? Number(a.eps) : 0.02,
        minArea: a.minArea !== undefined ? Number(a.minArea) : 0.05,
        simplify: a.simplify !== undefined ? Number(a.simplify) : 0,
        snapGrid: a.snapGrid !== undefined ? Number(a.snapGrid) : 0,
        layerStep,
        weldTol: a.weldTol !== undefined ? Number(a.weldTol) : 0,
        capMode: (a.capMode as "all" | "column") ?? "all",
        splitNonManifold: a.splitNonManifold === true,
      };

      const { mesh, report } = buildMesh(p, opts);
      if (mesh.tris.length === 0) {
        return asText({ ok: false, error: "no geometry produced", report });
      }

      const outBase = a.out ? resolve(normalizePath(String(a.out))) : defaultOut(path, opts.mode!);
      mkdirSync(dirname(outBase), { recursive: true });

      const written: string[] = [];
      const bytes: Record<string, number> = {};
      const format = String(a.format ?? "stl");
      if (format === "stl" || format === "both") {
        const f = `${outBase}.stl`;
        writeFileSync(f, mesh.toStlBinary());
        written.push(f);
        bytes[basename(f)] = statSync(f).size;
      }
      if (format === "3mf" || format === "both") {
        const f = `${outBase}.3mf`;
        writeFileSync(f, mesh.to3mf());
        written.push(f);
        bytes[basename(f)] = statSync(f).size;
      }

      return asText({
        ok: true,
        written,
        format,
        bytes,
        report,
        validation: report.validation,
      });
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- verify

server.registerTool(
  "gcode_verify",
  {
    description:
      "Validate a mesh (STL/3MF) and optionally compare it against a reference mesh: watertightness, manifold edges, volume, area, bbox, and symmetric nearest-surface distances (mean/RMS/p50/p95/p99/max). Use it to check a reconstructed model against the original.",
    inputSchema: {
      mesh: z.string().describe("STL or 3MF to inspect"),
      reference: z.string().optional().describe("Reference STL/3MF to compare against"),
      samples: z.number().int().positive().default(40000).describe("Surface samples per mesh"),
      maxRadius: z
        .number()
        .positive()
        .default(6)
        .describe("Max nearest-surface search radius in mm"),
    },
    annotations: { readOnlyHint: true },
  },
  async (args) => {
    try {
      const a = args as Record<string, unknown>;
      const target = loadMesh(requireFile(normalizePath(String(a.mesh))));
      const out: Record<string, unknown> = { target: meshReport(target, "target") };
      if (a.reference) {
        const ref = loadMesh(requireFile(normalizePath(String(a.reference))));
        out.reference = meshReport(ref, "reference");
        out.comparison = compareMeshes(
          ref,
          target,
          a.samples ? Number(a.samples) : 40000,
          a.maxRadius ? Number(a.maxRadius) : 6
        );
      }
      return asText(out);
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- preview

const FEATURE_COLORS: Record<string, [number, number, number]> = {
  "outer wall": [220, 40, 40],
  "inner wall": [255, 140, 0],
  "top surface": [40, 160, 60],
  "bottom surface": [0, 200, 200],
  "internal solid infill": [150, 90, 220],
  "overhang wall": [230, 200, 40],
  bridge: [120, 200, 240],
  "internal bridge": [90, 170, 230],
  "sparse infill": [170, 170, 170],
  "gap infill": [110, 110, 110],
  support: [60, 60, 60],
  "support interface": [90, 90, 110],
  custom: [200, 120, 200],
};

function pickLayers(count: number, spec: string): number[] {
  if (!spec || spec === "auto") {
    const picks = new Set([0, Math.floor(count / 4), Math.floor(count / 2), Math.floor((3 * count) / 4), count - 1]);
    return [...picks].filter((i) => i >= 0 && i < count).sort((a, b) => a - b);
  }
  const out = new Set<number>();
  for (const part of spec.split(",")) {
    const t = part.trim();
    if (!t) continue;
    // "start-end" or "start-end-step". With only one number, scan upward from 0.
    const m = /^(\d*)(?:-(\d+))?(?:-(\d+))?$/.exec(t);
    if (!m || (!m[1] && !m[2])) continue;
    const start = m[1] ? parseInt(m[1], 10) : 0;
    const end = m[2] ? parseInt(m[2], 10) : start;
    const step = m[3] ? parseInt(m[3], 10) : 1;
    if (step <= 0) continue;
    // A start above the end counts down, which is the useful reading for a
    // top-down sweep; otherwise walk upward from 0.
    if (m[1] && m[2] && start > end) {
      for (let i = start; i >= end; i -= step) out.add(i);
    } else {
      for (let i = start; i <= end; i += step) out.add(i);
    }
  }
  return [...out].filter((i) => i >= 0 && i < count).sort((a, b) => a - b);
}

server.registerTool(
  "gcode_preview",
  {
    description:
      "Render a PNG contact sheet of selected layers, colour-coded by extrusion feature. Use it to confirm a gcode parses correctly before reconstructing.",
    inputSchema: {
      gcode: z.string(),
      layers: z
        .string()
        .default("auto")
        .describe("Comma list of layer indices, ranges like 0-100-10, or 'auto'"),
      mode: z
        .string()
        .optional()
        .describe("Comma list of features to draw, e.g. 'outer wall,inner wall'"),
      width: z.number().int().positive().default(520).describe("Tile width in px"),
      cols: z.number().int().positive().default(5),
      out: z.string().optional(),
    },
  },
  async (args) => {
    try {
      const a = args as Record<string, unknown>;
      const path = requireFile(normalizePath(String(a.gcode)));
      const p = await parseFile(path);
      const layers = p.layers.filter((l) => l.segs.length > 0);
      if (layers.length === 0) return asText({ ok: false, error: "no extrusion found" });

      const picks = pickLayers(layers.length, String(a.layers ?? "auto")).map((i) => layers[i]);
      const b = bboxOf(p);
      const margin = 5;
      const wmm = b[3] - b[0] + 2 * margin;
      const hmm = b[4] - b[1] + 2 * margin;
      const tileW = a.width ? Number(a.width) : 520;
      const scale = tileW / Math.max(wmm, hmm);
      const W = Math.max(1, Math.round(wmm * scale));
      const H = Math.max(1, Math.round(hmm * scale));
      const cols = Math.min(picks.length, a.cols ? Number(a.cols) : 5);
      const rows = Math.ceil(picks.length / cols);
      const sheet = new Canvas(cols * W, rows * H, [255, 255, 255]);

      const only = a.mode
        ? new Set(
            String(a.mode)
              .split(",")
              .map((s) => s.trim().toLowerCase())
              .filter(Boolean)
          )
        : null;

      for (let i = 0; i < picks.length; i++) {
        const layer = picks[i];
        const ox = (i % cols) * W;
        const oy = Math.floor(i / cols) * H;
        for (const s of layer.segs) {
          const key = s.feat.trim().toLowerCase();
          if (only && !only.has(key)) continue;
          const c = FEATURE_COLORS[key] ?? [150, 150, 150];
          const x0 = ox + Math.round((s.x0 - b[0] + margin) * scale);
          const y0 = oy + Math.round(H - (s.y1 - b[1] + margin) * scale);
          const x1 = ox + Math.round((s.x1 - b[0] + margin) * scale);
          const y1 = oy + Math.round(H - (s.y0 - b[1] + margin) * scale);
          sheet.line(x0, y0, x1, y1, c, 1);
        }
        sheet.fillRect(ox + 4, oy + 4, 96, 12, [255, 255, 255]);
        sheet.text(ox + 6, oy + 6, `Z${layer.z.toFixed(2)} N${layer.segs.length}`, [0, 0, 0], 1);
      }

      const out = a.out ? resolve(normalizePath(String(a.out))) : `${defaultOut(path, "layers")}.png`;
      writeFileSync(out, sheet.toPng());
      return asText({
        ok: true,
        out,
        size: [sheet.width, sheet.height],
        layersDrawn: picks.map((l) => round(l.z, 3)),
        palette: FEATURE_COLORS,
      });
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- settings

server.registerTool(
  "gcode_settings",
  {
    description:
      "Extract the original slice parameters embedded in a gcode header/config block (layer height, nozzle, filament, temperatures, densities, support settings). Read-only.",
    inputSchema: { gcode: z.string() },
    annotations: { readOnlyHint: true },
  },
  async (args) => {
    try {
      const a = args as { gcode: string };
      const path = requireFile(normalizePath(a.gcode));
      const p = await parseFile(path, { maxLayers: 1 });
      return asText({
        file: p.path,
        slicer: p.slicer,
        flat: Object.fromEntries(Object.entries(p.settings).sort()),
        sections: Object.fromEntries(
          Object.entries(p.settingsSections).map(([k, v]) => [k, Object.fromEntries(Object.entries(v).sort())])
        ),
        key: keySettings(p),
        count: Object.keys(p.settings).length,
      });
    } catch (e) {
      return errText(e);
    }
  }
);

// ---------------------------------------------------------------- compare

server.registerTool(
  "gcode_compare",
  {
    description:
      "Diff two sliced gcode files: layer counts, extrusion totals, per-feature counts, bounding size deltas, and the slice settings that differ. Useful to check how a re-slice changed the job.",
    inputSchema: {
      a: z.string().describe("Baseline gcode"),
      b: z.string().describe("Candidate gcode"),
    },
    annotations: { readOnlyHint: true },
  },
  async (args) => {
    try {
      const a = args as { a: string; b: string };
      const pa = await parseFile(requireFile(normalizePath(a.a)));
      const pb = await parseFile(requireFile(normalizePath(a.b)));
      return asText(comparePrints(pa, pb));
    } catch (e) {
      return errText(e);
    }
  }
);

function comparePrints(a: ParsedPrint, b: ParsedPrint): Record<string, unknown> {
  const ba = bboxOf(a);
  const bb = bboxOf(b);
  const fa = Object.values(a.featureLen).reduce((s, v) => s + v, 0);
  const fb = Object.values(b.featureLen).reduce((s, v) => s + v, 0);
  const sizeA = [round(ba[3] - ba[0], 2), round(ba[4] - ba[1], 2), round(ba[5] - ba[2], 2)];
  const sizeB = [round(bb[3] - bb[0], 2), round(bb[4] - bb[1], 2), round(bb[5] - bb[2], 2)];
  const norm = (p: ParsedPrint) =>
    Object.fromEntries(Object.entries(p.featureCounts).map(([k, v]) => [k.toLowerCase(), v]));
  const na = norm(a);
  const nb = norm(b);
  const keys = [...new Set([...Object.keys(na), ...Object.keys(nb)])].sort();
  const ka = keySettings(a);
  const kb = keySettings(b);
  const settingKeys = [...new Set([...Object.keys(ka), ...Object.keys(kb)])].sort();

  return {
    a: {
      file: a.path,
      slicer: a.slicer,
      layers: a.layers.length,
      segments: a.layers.reduce((s, l) => s + l.segs.length, 0),
      extrusionMm: round(fa, 1),
      sizeMm: sizeA,
      settings: ka,
    },
    b: {
      file: b.path,
      slicer: b.slicer,
      layers: b.layers.length,
      segments: b.layers.reduce((s, l) => s + l.segs.length, 0),
      extrusionMm: round(fb, 1),
      sizeMm: sizeB,
      settings: kb,
    },
    delta: {
      layers: b.layers.length - a.layers.length,
      segments:
        b.layers.reduce((s, l) => s + l.segs.length, 0) -
        a.layers.reduce((s, l) => s + l.segs.length, 0),
      extrusionMm: round(fb - fa, 1),
      extrusionPct: fa > 0 ? round(((fb - fa) / fa) * 100, 2) : null,
      sizeMm: sizeB.map((v, i) => round(v - sizeA[i], 2)),
      sizePct: sizeB.map((v, i) => (sizeA[i] > 0 ? round(((v - sizeA[i]) / sizeA[i]) * 100, 3) : null)),
    },
    features: keys.map((k) => ({ feature: k, a: na[k] ?? 0, b: nb[k] ?? 0, delta: (nb[k] ?? 0) - (na[k] ?? 0) })),
    settingsDiff: settingKeys
      .filter((k) => ka[k] !== kb[k])
      .map((k) => ({ key: k, a: ka[k] ?? null, b: kb[k] ?? null })),
    warnings: { a: a.warnings, b: b.warnings },
  };
}

async function main() {
  const transport = new StdioServerTransport();
  await server.connect(transport);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});