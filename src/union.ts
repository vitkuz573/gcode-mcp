/**
 * Bridge to the shapely-backed union helper.
 *
 * The mesh builder needs the exact boundary of the union of stacked prisms.
 * Per-layer meshing cannot produce it: two overlapping layers put a top cap and
 * a bottom cap in one plane, which is an interior face, and duplicate the wall
 * between them. Computing the boundary properly is a 2D polygon boolean, so it
 * goes to `tools/union.py` - the same split slicer-mcp uses for its Python
 * leaf helpers. If shapely is missing the caller falls back to per-layer meshing.
 */
import { existsSync } from "node:fs";
import { spawnSync } from "node:child_process";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import type { Island, Pt } from "./geometry.js";

const here = dirname(fileURLToPath(import.meta.url));

/**
 * Locate tools/union.py.
 *
 * The compiled output can sit at dist/ or at dist-tools/src/ depending on which
 * tsconfig built it, so walk up until the script appears rather than assuming
 * one layout.
 */
function findScript(): string | null {
  let dir = here;
  for (let i = 0; i < 6; i++) {
    const p = join(dir, "tools", "union.py");
    if (existsSync(p)) return p;
    const parent = dirname(dir);
    if (parent === dir) break;
    dir = parent;
  }
  return null;
}

const SCRIPT = findScript();

/** Repository root, used to find the project venv. */
function findRoot(): string {
  let dir = here;
  for (let i = 0; i < 6; i++) {
    if (existsSync(join(dir, "package.json"))) return dir;
    const parent = dirname(dir);
    if (parent === dir) break;
    dir = parent;
  }
  return here;
}

const ROOT = findRoot();

/** Python that has shapely. Prefer the project venv, then the system one. */
function pythonBin(): string | null {
  const venv = join(ROOT, ".venv", "bin", "python");
  if (existsSync(venv)) return venv;
  for (const c of ["/usr/bin/python3", process.env.GCODE_MCP_PYTHON]) {
    if (c && existsSync(c)) return c;
  }
  return null;
}

let shapelyOk: boolean | null = null;

export function unionAvailable(): boolean {
  if (shapelyOk !== null) return shapelyOk;
  const py = pythonBin();
  if (!py || !SCRIPT) {
    shapelyOk = false;
    return false;
  }
  const probe = spawnSync(py, ["-c", "import shapely"], { encoding: "utf8" });
  shapelyOk = probe.status === 0;
  return shapelyOk;
}

export interface UnionRing {
  exterior: Pt[];
  holes: Pt[][];
}

export interface UnionLayer {
  z0: number;
  z1: number;
  /**
   * Every boundary ring of the layer's footprint: exteriors counter-clockwise,
   * holes clockwise.
   *
   * A side wall is a strip rather than an area, so these are plain rings rather
   * than polygons. The winding is what puts the quad normal on the void side -
   * for an exterior and for a hole that means opposite directions.
   */
  side: Pt[][];
  capBottom: UnionRing[];
  capTop: UnionRing[];
}

export interface UnionResult {
  layers: UnionLayer[];
}

/**
 * Exact boundary of the union of the per-layer prisms.
 *
 * Returns null when shapely is unavailable, so the caller can fall back.
 */
export function unionPrisms(
  layers: Array<{ z0: number; z1: number; islands: Island[] }>
): UnionResult | null {
  if (layers.length === 0) return null;
  const py = pythonBin();
  if (!py || !unionAvailable()) return null;

  const payload = {
    layers: layers.map((l) => ({
      z0: l.z0,
      z1: l.z1,
      islands: l.islands.map((i) => ({
        outer: i.outer.pts,
        holes: i.holes.map((h) => h.pts),
      })),
    })),
  };

  if (!SCRIPT) return null;
  const res = spawnSync(py, [SCRIPT], {
    input: JSON.stringify(payload),
    encoding: "utf8",
    maxBuffer: 1024 * 1024 * 512,
  });
  if (res.status !== 0 || !res.stdout) return null;
  let parsed: { ok: boolean; layers?: UnionLayer[]; error?: string };
  try {
    parsed = JSON.parse(res.stdout);
  } catch {
    return null;
  }
  if (!parsed.ok || !parsed.layers) return null;
  return { layers: parsed.layers };
}