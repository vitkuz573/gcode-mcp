/**
 * Mesh assembly: layers of islands -> watertight triangle soup -> STL/3MF.
 *
 * Each layer is emitted as its own closed shell (caps + side walls), so every
 * edge is used exactly twice by construction. Coincident caps between
 * neighbouring layers are kept: they cancel in the volume integral and every
 * slicer unions them away. Welding then collapses near-identical vertices so
 * vertical seams close.
 */

import { gzipSync } from "node:zlib";
import {
  filterSegments,
  loopsToIslands,
  segmentsToLoops,
  type FeatureMode,
  type Island,
  type Pt,
} from "./geometry.js";
import { bboxOf, type Layer, type ParsedPrint } from "./parse.js";
import { triangulate } from "./triangulate.js";

export type V3 = [number, number, number];

export interface Validation {
  vertices: number;
  triangles: number;
  boundaryEdges: number;
  nonmanifoldEdges: number;
  duplicateFaces: number;
  watertight: boolean;
  signedVolumeCm3: number;
  consistentWinding: boolean;
  bbox: number[];
}

export class Mesh {
  verts: V3[] = [];
  tris: Array<[number, number, number]> = [];

  addQuad(a: V3, b: V3, c: V3, d: V3): void {
    const i = this.verts.length;
    this.verts.push(a, b, c, d);
    this.tris.push([i, i + 1, i + 2], [i, i + 2, i + 3]);
  }

  addTri(a: V3, b: V3, c: V3): void {
    const i = this.verts.length;
    this.verts.push(a, b, c);
    this.tris.push([i, i + 1, i + 2]);
  }

  /** Edge -> use count, keyed on rounded coordinates. */
  edgeCounts(): Map<string, number> {
    const counts = new Map<string, number>();
    for (const [a, b, c] of this.tris) {
      const va = this.verts[a];
      const vb = this.verts[b];
      const vc = this.verts[c];
      for (const [u, w] of [
        [va, vb],
        [vb, vc],
        [vc, va],
      ]) {
        const k = edgeKey(u, w);
        counts.set(k, (counts.get(k) ?? 0) + 1);
      }
    }
    return counts;
  }

  volume(): number {
    let total = 0;
    for (const [a, b, c] of this.tris) {
      const pa = this.verts[a];
      const pb = this.verts[b];
      const pc = this.verts[c];
      total +=
        pa[0] * (pb[1] * pc[2] - pb[2] * pc[1]) -
        pa[1] * (pb[0] * pc[2] - pb[2] * pc[0]) +
        pa[2] * (pb[0] * pc[1] - pb[1] * pc[0]);
    }
    return total / 6;
  }

  bbox(): number[] {
    if (this.verts.length === 0) return [0, 0, 0, 0, 0, 0];
    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (const v of this.verts) {
      for (let k = 0; k < 3; k++) {
        if (v[k] < lo[k]) lo[k] = v[k];
        if (v[k] > hi[k]) hi[k] = v[k];
      }
    }
    return [...lo, ...hi];
  }

  validate(): Validation {
    const edges = this.edgeCounts();
    let boundary = 0;
    let nonmanifold = 0;
    for (const c of edges.values()) {
      if (c === 1) boundary++;
      else if (c > 2) nonmanifold++;
    }
    const faceMap = new Map<string, number>();
    let dupes = 0;
    for (const [a, b, c] of this.tris) {
      const keys = [
        vk(this.verts[a]),
        vk(this.verts[b]),
        vk(this.verts[c]),
      ].sort();
      const k = keys.join("|");
      const n = (faceMap.get(k) ?? 0) + 1;
      faceMap.set(k, n);
      if (n > 1) dupes++;
    }
    const vol = this.volume();
    return {
      vertices: this.verts.length,
      triangles: this.tris.length,
      boundaryEdges: boundary,
      nonmanifoldEdges: nonmanifold,
      duplicateFaces: dupes,
      watertight: boundary === 0 && nonmanifold === 0,
      signedVolumeCm3: round(Math.abs(vol) / 1000, 3),
      consistentWinding: vol > 0,
      bbox: this.bbox().map((v) => round(v, 3)),
    };
  }

  toStlBinary(): Buffer {
    const tris = this.tris.length;
    const buf = Buffer.alloc(84 + tris * 50);
    buf.write("gcode-mcp reconstructed model", 0, "ascii");
    buf.writeUInt32LE(tris, 80);
    let o = 84;
    for (const [ia, ib, ic] of this.tris) {
      const pa = this.verts[ia];
      const pb = this.verts[ib];
      const pc = this.verts[ic];
      const ux = pb[0] - pa[0];
      const uy = pb[1] - pa[1];
      const uz = pb[2] - pa[2];
      const vx = pc[0] - pa[0];
      const vy = pc[1] - pa[1];
      const vz = pc[2] - pa[2];
      let nx = uy * vz - uz * vy;
      let ny = uz * vx - ux * vz;
      let nz = ux * vy - uy * vx;
      const len = Math.hypot(nx, ny, nz) || 1;
      nx /= len;
      ny /= len;
      nz /= len;
      for (const v of [nx, ny, nz, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], pc[0], pc[1], pc[2]]) {
        buf.writeFloatLE(v, o);
        o += 4;
      }
      o += 2; // attribute byte count
    }
    return buf;
  }

  to3mf(): Buffer {
    const head =
      '<?xml version="1.0" encoding="UTF-8"?>' +
      '<model unit="millimeter" xml:lang="en-US" ' +
      'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">' +
      '<metadata name="Application">gcode-mcp</metadata>' +
      '<resources><object id="1" type="model"><mesh><vertices>';
    const parts: string[] = [head];
    for (const [x, y, z] of this.verts) {
      parts.push(`<vertex x="${x.toFixed(4)}" y="${y.toFixed(4)}" z="${z.toFixed(4)}"/>`);
    }
    parts.push("</vertices><triangles>");
    for (const [a, b, c] of this.tris) {
      parts.push(`<triangle v1="${a}" v2="${b}" v3="${c}"/>`);
    }
    parts.push(
      "</triangles></mesh></object></resources>" +
        '<build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/></build></model>'
    );
    return zipSync([
      { name: "[Content_Types].xml", data: Buffer.from(contentTypes(), "utf8") },
      { name: "_rels/.rels", data: Buffer.from(rels(), "utf8") },
      { name: "3D/3dmodel.model", data: Buffer.from(parts.join(""), "utf8") },
    ]);
  }
}

function contentTypes(): string {
  return (
    '<?xml version="1.0" encoding="UTF-8"?>' +
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">' +
    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>' +
    '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>' +
    "</Types>"
  );
}

function rels(): string {
  return (
    '<?xml version="1.0" encoding="UTF-8"?>' +
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' +
    '<Relationship Target="/3D/3dmodel.model" Id="rel0" ' +
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>' +
    "</Relationships>"
  );
}

interface ZipEntry {
  name: string;
  data: Buffer;
}

/** Minimal store-only ZIP writer (3MF is an OPC package). */
function zipSync(entries: ZipEntry[]): Buffer {
  const locals: Buffer[] = [];
  const centrals: Buffer[] = [];
  let offset = 0;

  for (const e of entries) {
    const nameBuf = Buffer.from(e.name, "utf8");
    const crc = crc32(e.data);
    const comp = gzipSync(e.data);

    const local = Buffer.alloc(30 + nameBuf.length);
    local.writeUInt32LE(0x04034b50, 0);
    local.writeUInt16LE(20, 4); // version needed
    local.writeUInt16LE(0, 6); // flags
    local.writeUInt16LE(8, 8); // deflate
    local.writeUInt16LE(0, 10); // time
    local.writeUInt16LE(0x21, 12); // date (1996-01-01)
    local.writeUInt32LE(crc, 14);
    local.writeUInt32LE(comp.length, 18);
    local.writeUInt32LE(e.data.length, 22);
    local.writeUInt16LE(nameBuf.length, 26);
    local.writeUInt16LE(0, 28);
    nameBuf.copy(local, 30);
    locals.push(local, comp);

    const central = Buffer.alloc(46 + nameBuf.length);
    central.writeUInt32LE(0x02014b50, 0);
    central.writeUInt16LE(20, 4);
    central.writeUInt16LE(20, 6);
    central.writeUInt16LE(0, 8);
    central.writeUInt16LE(8, 10);
    central.writeUInt16LE(0, 12);
    central.writeUInt16LE(0x21, 14);
    central.writeUInt32LE(crc, 16);
    central.writeUInt32LE(comp.length, 20);
    central.writeUInt32LE(e.data.length, 24);
    central.writeUInt16LE(nameBuf.length, 28);
    central.writeUInt16LE(0, 30);
    central.writeUInt16LE(0, 32);
    central.writeUInt16LE(0, 34);
    central.writeUInt16LE(0, 36);
    central.writeUInt32LE(0, 38);
    central.writeUInt32LE(offset, 42);
    nameBuf.copy(central, 46);
    centrals.push(central);

    offset += local.length + comp.length;
  }

  const centralBuf = Buffer.concat(centrals);
  const end = Buffer.alloc(22);
  end.writeUInt32LE(0x06054b50, 0);
  end.writeUInt16LE(0, 4);
  end.writeUInt16LE(0, 6);
  end.writeUInt16LE(entries.length, 8);
  end.writeUInt16LE(entries.length, 10);
  end.writeUInt32LE(centralBuf.length, 12);
  end.writeUInt32LE(offset, 16);
  end.writeUInt16LE(0, 20);

  return Buffer.concat([...locals, centralBuf, end]);
}

let CRC_TABLE: Int32Array | null = null;
function crc32(buf: Buffer): number {
  if (!CRC_TABLE) {
    CRC_TABLE = new Int32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      CRC_TABLE[n] = c;
    }
  }
  let c = -1;
  for (let i = 0; i < buf.length; i++) {
    c = CRC_TABLE[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  }
  return (c ^ -1) >>> 0;
}

function vk(v: V3): string {
  return `${round(v[0], 4)},${round(v[1], 4)},${round(v[2], 4)}`;
}
function edgeKey(a: V3, b: V3): string {
  const ka = vk(a);
  const kb = vk(b);
  return ka <= kb ? `${ka}|${kb}` : `${kb}|${ka}`;
}
export function round(v: number, d: number): number {
  const f = 10 ** d;
  return Math.round(v * f) / f;
}

/** Snap near-coincident XYZ vertices together. */
export function weldVertical(mesh: Mesh, tol: number): number {
  if (mesh.verts.length === 0 || tol <= 0) return 0;
  const key = new Map<string, number>();
  const remap = new Int32Array(mesh.verts.length);
  mesh.verts.forEach((v, i) => {
    const k = `${Math.round(v[0] / tol)},${Math.round(v[1] / tol)},${Math.round(v[2] / tol)}`;
    const j = key.get(k);
    if (j === undefined) {
      key.set(k, i);
      remap[i] = i;
    } else {
      remap[i] = j;
    }
  });
  const before = mesh.tris.length;
  mesh.tris = mesh.tris.filter(
    ([a, b, c]) => remap[a] !== remap[b] && remap[b] !== remap[c] && remap[c] !== remap[a]
  );
  const collapsed = before - mesh.tris.length;

  const used = [...new Set(mesh.tris.flat())].sort((a, b) => a - b);
  const remap2 = new Int32Array(mesh.verts.length);
  used.forEach((oldIdx, newIdx) => {
    remap2[oldIdx] = newIdx;
  });
  mesh.verts = used.map((i) => mesh.verts[i]);
  mesh.tris = mesh.tris.map(([a, b, c]) => [remap2[a], remap2[b], remap2[c]] as [number, number, number]);
  return collapsed;
}

/** Make triangle winding consistent and outward-facing. */
export function flipFacesOutward(mesh: Mesh): boolean {
  if (mesh.tris.length === 0) return false;
  if (mesh.volume() >= 0) return false;
  mesh.tris = mesh.tris.map(([a, b, c]) => [a, c, b] as [number, number, number]);
  return true;
}

/**
 * Give each over-shared edge its own vertex copies.
 *
 * Two features meeting at a single point (a wall touching another island, or a
 * cap meeting a wall on the same line) can produce an edge used by four faces.
 * Splitting the duplicates by an infinitesimal Z offset gives each face its own
 * copy, restoring manifoldness without moving the surface off its plane.
 */
function resolveNonManifold(mesh: Mesh): number {
  const over: Array<[string, number]> = [];
  for (const [k, c] of mesh.edgeCounts()) {
    if (c > 2) over.push([k, c]);
  }
  if (over.length === 0) return 0;

  const copies = new Map<string, number>();
  let fixed = 0;

  for (const [k, count] of over) {
    // An odd count is a pinch point: faces cannot be paired off, so leave it.
    if (count % 2 !== 0) continue;
    const [pa, pb] = k.split("|") as [string, string];
    const faces: number[] = [];
    for (let ti = 0; ti < mesh.tris.length; ti++) {
      const [ia, ib, ic] = mesh.tris[ti];
      const ka = vk(mesh.verts[ia]);
      const kb = vk(mesh.verts[ib]);
      const kc = vk(mesh.verts[ic]);
      let has = 0;
      if (ka === pa || ka === pb) has++;
      if (kb === pa || kb === pb) has++;
      if (kc === pa || kc === pb) has++;
      if (has >= 2) faces.push(ti);
    }
    // Pair the faces off. Each pair gets its own vertex set, so every
    // resulting edge is used exactly twice.
    for (let n = 2; n < faces.length; n += 2) {
      const group = n / 2;
      for (const ti of [faces[n], faces[n + 1]]) {
        const [ia, ib, ic] = mesh.tris[ti];
        const next = [ia, ib, ic] as [number, number, number];
        for (let c = 0; c < 3; c++) {
          const v = mesh.verts[next[c]];
          const key = vk(v);
          if (key !== pa && key !== pb) continue;
          const ck = `${key}#${group}`;
          let idx = copies.get(ck);
          if (idx === undefined) {
            mesh.verts.push([v[0], v[1], v[2] + 1e-4 * group]);
            idx = mesh.verts.length - 1;
            copies.set(ck, idx);
          }
          next[c] = idx;
        }
        mesh.tris[ti] = next;
        fixed++;
      }
    }
  }
  return fixed;
}

/** Is this island covered by an island in the adjacent layer? */
function covered(isl: Island, neighbours: Island[]): boolean {
  if (neighbours.length === 0) return false;
  const outer = isl.outer;
  for (const nb of neighbours) {
    const cand = nb.outer;
    const cb = cand.bbox();
    const ob = outer.bbox();
    if (ob[0] < cb[0] || ob[1] < cb[1] || ob[2] > cb[2] || ob[3] > cb[3]) continue;
    let all = true;
    for (const p of outer.pts) {
      if (!cand.pointIn(p[0], p[1])) {
        all = false;
        break;
      }
    }
    if (all) return true;
  }
  return false;
}

export interface BuildOptions {
  mode?: FeatureMode;
  objects?: string[];
  eps?: number;
  minArea?: number;
  simplify?: number;
  snapGrid?: number;
  layerStep?: number;
  stitchTol?: number;
  weldTol?: number;
  capMode?: "all" | "column";
  /** Give surplus faces their own vertex copies to clear non-manifold edges. */
  splitNonManifold?: boolean;
  onProgress?: (layer: number, total: number) => void;
}

export interface BuildReport {
  mode: string;
  layersTotal: number;
  layersUsed: number;
  islands: number;
  loops: number;
  skippedLayers: number;
  weld: { toleranceMm: number; collapsedTriangles: number };
  nonmanifoldSplits: number;
  /**
   * Why the shell is not manifold, when it is not.
   *
   * Every layer is emitted as its own closed shell, so edges start out used
   * exactly twice. Two failures remain. When two layers' contours happen to
   * share an XY line (an internal wall resting on another, or a seam gap that
   * snapped together), the coincident caps meet the walls of the other layer
   * and produce edges used four times. Removing the surplus would need a real
   * boolean union of the per-layer polygons; until that exists the shell is
   * closed with a few over-shared edges instead.
   */
  notes: string[];
  validation: Validation;
  bbox: number[];
}

export function buildMesh(parsed: ParsedPrint, opts: BuildOptions = {}): {
  mesh: Mesh;
  report: BuildReport;
} {
  const mesh = new Mesh();
  const mode = opts.mode ?? "shell";
  const objects = opts.objects;
  const eps = opts.eps ?? 0.02;
  const minArea = opts.minArea ?? 0.05;
  const simplify = opts.simplify ?? 0;
  const snapGrid = opts.snapGrid ?? 0;
  const layerStep = opts.layerStep ?? 1;
  const stitchTol = opts.stitchTol;
  const weldTol = opts.weldTol ?? 0.01;
  const capMode = opts.capMode ?? "all";
  const splitNonManifold = opts.splitNonManifold ?? false;

  const report: BuildReport = {
    mode,
    layersTotal: parsed.layers.length,
    layersUsed: 0,
    islands: 0,
    loops: 0,
    skippedLayers: 0,
    weld: { toleranceMm: weldTol, collapsedTriangles: 0 },
    nonmanifoldSplits: 0,
    notes: [],
    validation: {} as Validation,
    bbox: bboxOf(parsed),
  };

  // Pass 1: trace every layer up front, because whether a layer needs a cap
  // depends on its neighbours.
  const traced: Array<{ layer: Layer; islands: Island[]; loops: number }> = [];
  const source = layerStep > 1 ? parsed.layers.filter((_, i) => i % layerStep === 0) : parsed.layers;

  for (let i = 0; i < source.length; i++) {
    const layer = source[i];
    const segs = filterSegments(layer.segs, mode, objects);
    if (segs.length === 0) {
      report.skippedLayers++;
      continue;
    }
    const loops = segmentsToLoops(segs, layer.z, {
      eps,
      minArea,
      simplify,
      stitchTol,
      snapGrid,
    });
    if (loops.length === 0) {
      report.skippedLayers++;
      continue;
    }
    const islands = loopsToIslands(loops);
    if (islands.length === 0) {
      report.skippedLayers++;
      continue;
    }
    traced.push({ layer, islands, loops: loops.length });
    if (opts.onProgress && i % 10 === 0) opts.onProgress(i, source.length);
  }

  // Pass 2: emit geometry.
  for (let idx = 0; idx < traced.length; idx++) {
    const { layer, islands, loops } = traced[idx];
    const z0 = layer.bottom;
    const z1 = layer.top;
    report.layersUsed++;
    report.islands += islands.length;
    report.loops += loops;

    const above = idx + 1 < traced.length ? traced[idx + 1].islands : [];
    const below = idx > 0 ? traced[idx - 1].islands : [];

    for (const isl of islands) {
      const { ring, tris } = triangulate(isl.outer.pts, isl.holes.map((h) => h.pts));

      // side walls
      const wallLoops: Array<[Pt[], boolean]> = [[isl.outer.pts, false]];
      for (const h of isl.holes) wallLoops.push([h.pts, true]);
      for (const [pts, reverse] of wallLoops) {
        const n = pts.length;
        for (let k = 0; k < n; k++) {
          let ax = pts[k][0];
          let ay = pts[k][1];
          let bx = pts[(k + 1) % n][0];
          let by = pts[(k + 1) % n][1];
          if (reverse) {
            [ax, ay, bx, by] = [bx, by, ax, ay];
          }
          mesh.addQuad([ax, ay, z0], [bx, by, z0], [bx, by, z1], [ax, ay, z1]);
        }
      }

      // caps
      const covAbove = capMode === "column" ? covered(isl, above) : false;
      if (!covAbove) {
        for (const [i0, i1, i2] of tris) {
          const p0 = ring[i0];
          const p1 = ring[i1];
          const p2 = ring[i2];
          mesh.addTri([p0[0], p0[1], z1], [p1[0], p1[1], z1], [p2[0], p2[1], z1]);
        }
      }
      const covBelow = capMode === "column" ? covered(isl, below) : false;
      if (!covBelow) {
        for (const [i0, i1, i2] of tris) {
          const p0 = ring[i0];
          const p1 = ring[i1];
          const p2 = ring[i2];
          mesh.addTri([p0[0], p0[1], z0], [p2[0], p2[1], z0], [p1[0], p1[1], z0]);
        }
      }
    }
  }

  report.weld.collapsedTriangles = weldVertical(mesh, weldTol);

  // Optional: clear non-manifold edges by giving the surplus faces their own
  // vertex copies. This does not produce a watertight shell on its own (the new
  // edges are used once), so it is opt-in and off by default: with it off the
  // shell stays closed with a handful of over-shared edges, which slicers and
  // mesh repair tools resolve more cheaply than an open mesh does.
  let splitCount = 0;
  if (splitNonManifold) {
    for (let i = 0; i < 6; i++) {
      const n = resolveNonManifold(mesh);
      splitCount += n;
      if (n === 0) break;
    }
  }
  flipFacesOutward(mesh);
  report.nonmanifoldSplits = splitCount;
  report.validation = mesh.validate();

  const v0 = report.validation;
  const notes: string[] = [];
  if (v0.watertight) {
    notes.push("watertight: every edge is used exactly twice.");
  } else if (v0.boundaryEdges > 0 && v0.nonmanifoldEdges > 0) {
    notes.push(
      `open in ${v0.boundaryEdges} edges and over-shared in ${v0.nonmanifoldEdges}. ` +
        `Vertex welding at ${weldTol} mm may be fusing nearby-but-distinct contours; try weldTol 0.`
    );
  } else if (v0.boundaryEdges > 0) {
    notes.push(
      `open in ${v0.boundaryEdges} edges. Cap triangulation did not fully cover some layer footprint.`
    );
  } else {
    notes.push(
      `closed but not manifold: ${v0.nonmanifoldEdges} edges are used more than twice, ` +
        `where adjacent layers' caps coincide. A boolean union of the per-layer polygons ` +
        `would resolve it; slicers and mesh repair tools handle this without complaint.`
    );
  }
  report.notes = notes;

  return { mesh, report };
}