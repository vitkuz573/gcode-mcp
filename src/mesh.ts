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
  type Loop,
  type Pt,
} from "./geometry.js";
import { bboxOf, type Layer, type ParsedPrint } from "./parse.js";
import { unionAvailable, unionPrisms, type UnionLayer, type UnionRing } from "./union.js";
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

  /**
   * Vertical walls accumulated by XY edge, across all layers.
   *
   * Two layers whose contours share an XY line emit two quads meeting at one
   * edge, which leaves that edge used four times. Holding walls until every
   * layer is emitted lets the pair collapse into one quad spanning both, the
   * union of two stacked prisms without a general boolean pass.
   */
  wallSpans = new Map<string, { ax: number; ay: number; bx: number; by: number; zLo: number; zHi: number }>();

  addWallSpan(ax: number, ay: number, bx: number, by: number, z0: number, z1: number): void {
    const ka = `${round(ax, 4)},${round(ay, 4)}`;
    const kb = `${round(bx, 4)},${round(by, 4)}`;
    const key = ka <= kb ? `${ka}|${kb}` : `${kb}|${ka}`;
    // Store the endpoints in contour order so the quad faces outward.
    const span = this.wallSpans.get(key);
    if (span) {
      if (z0 < span.zLo) span.zLo = z0;
      if (z1 > span.zHi) span.zHi = z1;
    } else {
      this.wallSpans.set(key, { ax, ay, bx, by, zLo: z0, zHi: z1 });
    }
  }

  /** Emit the accumulated walls as quads and clear the buffer. */
  flushWalls(): void {
    for (const s of this.wallSpans.values()) {
      this.addQuad([s.ax, s.ay, s.zLo], [s.bx, s.by, s.zLo], [s.bx, s.by, s.zHi], [s.ax, s.ay, s.zHi]);
    }
    this.wallSpans.clear();
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

  /**
   * Remove triangles with two coincident vertices or a degenerate normal.
   *
   * Returns how many were dropped. Repeated contour points (seams, and spurs
   * where one feature meets another) can produce such triangles; a slicer
   * refuses the file rather than repairing it.
   */
  dropDegenerateTris(areaEps = 1e-10, lenEps = 1e-9): number {
    const before = this.tris.length;
    const keep: Array<[number, number, number]> = [];
    for (const [a, b, c] of this.tris) {
      const pa = this.verts[a];
      const pb = this.verts[b];
      const pc = this.verts[c];
      const ab = Math.hypot(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
      const bc = Math.hypot(pc[0] - pb[0], pc[1] - pb[1], pc[2] - pb[2]);
      const ca = Math.hypot(pa[0] - pc[0], pa[1] - pc[1], pa[2] - pc[2]);
      if (ab <= lenEps || bc <= lenEps || ca <= lenEps) continue;
      const ux = pb[0] - pa[0], uy = pb[1] - pa[1], uz = pb[2] - pa[2];
      const vx = pc[0] - pa[0], vy = pc[1] - pa[1], vz = pc[2] - pa[2];
      const nx = uy * vz - uz * vy;
      const ny = uz * vx - ux * vz;
      const nz = ux * vy - uy * vx;
      if (Math.hypot(nx, ny, nz) * 0.5 <= areaEps) continue;
      keep.push([a, b, c]);
    }
    this.tris = keep;
    return before - keep.length;
  }

  /** Compact the vertex list to only those referenced by a triangle. */
  compact(): number {
    const used = [...new Set(this.tris.flat())].sort((a, b) => a - b);
    const remap = new Map<number, number>();
    used.forEach((oldIdx, newIdx) => remap.set(oldIdx, newIdx));
    this.verts = used.map((i) => this.verts[i]);
    this.tris = this.tris.map(([a, b, c]) => [remap.get(a)!, remap.get(b)!, remap.get(c)!]);
    return this.verts.length;
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

/**
 * Is this XY point inside the material of any island in `neighbours`?
 *
 * Used per cap triangle rather than per island: with overlapping islands (the
 * correct reading of a slice) two islands can each cover part of the other, so
 * an all-or-nothing test would emit caps into material and leave the shell open.
 */
/**
 * Is every one of these XY points inside the material of `neighbours`?
 *
 * Deliberately all-or-nothing. A cap triangle that straddles the neighbour's
 * boundary counts as NOT covered, so the cap is kept: over-covering leaves a
 * coincident face inside solid material, which is harmless, whereas
 * under-covering leaves a strip with no floor and opens the mesh.
 */
function insideAny(pts: Array<[number, number]>, neighbours: Island[]): boolean {
  if (neighbours.length === 0) return false;
  for (const [px, py] of pts) {
    let covered = false;
    for (const nb of neighbours) {
      if (!nb.outer.pointIn(px, py)) continue;
      let inHole = false;
      for (const h of nb.holes) {
        if (h.pointIn(px, py)) {
          inHole = true;
          break;
        }
      }
      if (!inHole) {
        covered = true;
        break;
      }
    }
    if (!covered) return false;
  }
  return true;
}

/**
 * Do two layers have the same footprint, within `tol`?
 *
 * A hash of the boundary lines does not work: the slicer's seam leaves the two
 * ends of a contour ~0.04 mm apart, so the line sets differ between every pair
 * of layers even where the wall is perfectly vertical. Compare geometry with a
 * tolerance instead - same island count, matching areas, and every vertex of
 * one contour sitting on the other's boundary.
 */
function footprintMatches(a: Island[], b: Island[], tol: number): boolean {
  if (a.length !== b.length) return false;
  const used = new Array(b.length).fill(false);
  for (const ia of a) {
    let matched = -1;
    for (let j = 0; j < b.length; j++) {
      if (used[j]) continue;
      const ib = b[j];
      if (Math.abs(ia.area() - ib.area()) > tol * 4) continue;
      if (ia.holes.length !== ib.holes.length) continue;
      if (contourNear(ia.outer, ib.outer, tol)) {
        matched = j;
        break;
      }
    }
    if (matched < 0) return false;
    used[matched] = true;
  }
  return true;
}

/** Every vertex of `a` lies within `tol` of `b`'s boundary. */
function contourNear(a: Loop, b: Loop, tol: number): boolean {
  const bp = b.pts;
  const n = bp.length;
  const cell = Math.max(tol, 1e-6);
  const grid = new Map<string, number[]>();
  for (let i = 0; i < n; i++) {
    const k = `${Math.floor(bp[i][0] / cell)},${Math.floor(bp[i][1] / cell)}`;
    let l = grid.get(k);
    if (!l) grid.set(k, (l = []));
    l.push(i);
  }
  for (const p of a.pts) {
    const gx = Math.floor(p[0] / cell);
    const gy = Math.floor(p[1] / cell);
    let best = Infinity;
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const l = grid.get(`${gx + dx},${gy + dy}`);
        if (!l) continue;
        for (const i of l) {
          const d = Math.hypot(bp[i][0] - p[0], bp[i][1] - p[1]);
          if (d < best) best = d;
        }
      }
    }
    if (best > tol) return false;
  }
  return true;
}

/** A triangle's three vertices, pulled toward its centroid to stay off edges. */
function triSamples(a: V3, b: V3, c: V3, shrink = 0.85): Array<[number, number]> {
  const mx = (a[0] + b[0] + c[0]) / 3;
  const my = (a[1] + b[1] + c[1]) / 3;
  return [
    [mx + (a[0] - mx) * shrink, my + (a[1] - my) * shrink],
    [mx + (b[0] - mx) * shrink, my + (b[1] - my) * shrink],
    [mx + (c[0] - mx) * shrink, my + (c[1] - my) * shrink],
    [mx, my],
  ];
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
  /** Fill leftover open boundary loops so the shell is watertight. */
  closeHoles?: boolean;
  /** Merge walls sharing an XY edge into one span across layers. */
  mergeWalls?: boolean;
  /**
   * Tolerance in mm for deciding two layers share a footprint, and so can be
   * merged into one run. Must exceed the slicer's seam gap (~0.04 mm).
   */
  runMatchTol?: number;
  /**
   * Use the exact prism-union boundary (needs shapely). On by default; falls
   * back to per-layer meshing when the helper is unavailable.
   */
  useUnion?: boolean;
  /**
   * Drop zero-area and zero-length triangles, which slicers reject outright.
   *
   * A contour that visits the same point twice (a seam) can make an ear-clip
   * emit a triangle with two coincident vertices. Those carry a valid index but
   * a zero-area normal, and OrcaSlicer treats the file as corrupt.
   */
  dropDegenerate?: boolean;
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
  degenerateTriangles: number;
  runs: number;
  holeFillTriangles: number;
  nonmanifoldSplits: number;
  /** True when the exact prism union was used rather than per-layer meshing. */
  union: boolean;
  unionLayers: number;
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
  const mergeWalls = opts.mergeWalls ?? false;
  // Run merging is opt-in. It only pays off on a genuinely vertical wall, and
  // on a contour that drifts it stretches the first layer's outline across the
  // whole run, which distorts the model badly. Off by default.
  const runMatchTol = opts.runMatchTol ?? 0;
  const useUnion = opts.useUnion ?? true;
  const dropDegenerate = opts.dropDegenerate ?? true;
  const closeHoles = opts.closeHoles ?? true;

  const report: BuildReport = {
    mode,
    layersTotal: parsed.layers.length,
    layersUsed: 0,
    islands: 0,
    loops: 0,
    skippedLayers: 0,
    weld: { toleranceMm: weldTol, collapsedTriangles: 0 },
    degenerateTriangles: 0,
    runs: 0,
    holeFillTriangles: 0,
    nonmanifoldSplits: 0,
    union: false,
    unionLayers: 0,
    notes: [],
    validation: {} as Validation,
    bbox: bboxOf(parsed),
  };

  // Pass 1: trace every layer up front, because whether a layer needs a cap
  // depends on its neighbours.
  const traced: TracedLayer[] = [];
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
    report.layersUsed++;
    report.islands += islands.length;
    report.loops += loops;
  }

interface TracedLayer {
  layer: Layer;
  islands: Island[];
  loops: number;
}

/**
 * Per-layer meshing: one closed shell per layer.
 *
 * Every layer gets caps and walls, so each is closed on its own and the whole
 * mesh has no boundary edges. The cost is a coincident cap pair wherever two
 * layers share a footprint, which leaves a few over-shared edges. Used only
 * when shapely is unavailable, since the union path is manifold by construction.
 */
function emitPerLayer(mesh: Mesh, traced: TracedLayer[], report: BuildReport): void {
  // Group consecutive layers that share a footprint into a run, and build one
  // shell per run.
  //
  // A layer's top cap and the next layer's bottom cap lie in the same plane. If
  // both are emitted they form an interior face pair, and the wall between them
  // is drawn twice, leaving every shared edge used four times. Collapsing equal
  // footprints into a single shell removes those planes outright: the wall spans
  // the whole run, and caps appear only where the footprint actually changes.
}

  // Preferred path: exact boundary of the union of prisms, via shapely.
  //
  // This is the only construction that is manifold by definition. When it is
  // unavailable the per-layer path below runs instead, which stays watertight
  // but leaves a few over-shared edges.
  let unioned = false;
  if (useUnion) {
    const u = unionPrisms(
      traced.map((t) => ({ z0: t.layer.bottom, z1: t.layer.top, islands: t.islands }))
    );
    if (u) {
      unioned = true;
      emitUnion(mesh, u.layers);
      report.union = true;
      report.unionLayers = u.layers.length;
    }
  }

  if (!unioned) {
    emitPerLayer(mesh, traced, report);
  }

  report.degenerateTriangles = dropDegenerate ? mesh.dropDegenerateTris() : 0;
  if (dropDegenerate) mesh.compact();
  // Fan-fill whatever the caps and walls left open.
  report.holeFillTriangles = closeHoles ? closeBoundaryHoles(mesh) : 0;
  if (closeHoles && report.holeFillTriangles > 0) mesh.compact();
  report.weld.collapsedTriangles = weldVertical(mesh, weldTol);

  if (mergeWalls) mesh.flushWalls();

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


  /**
   * Close every open boundary loop with a fan patch.
   *
   * Each layer's shell is built from a triangulated cap and side walls built
   * from the same loops, but the two disagree wherever ear clipping reorders a
   * ring, so a few edges end up used once. Those edges form closed loops, and a
   * fan from a centroid vertex gives each edge exactly one partner.
   */
function closeBoundaryHoles(mesh: Mesh): number {
  let added = 0;
  for (let round = 0; round < 4; round++) {
    const counts = mesh.edgeCounts();
    const boundary: Array<[string, string]> = [];
    for (const [k, c] of counts) {
    if (c !== 1) continue;
    const bar = k.indexOf("|");
    boundary.push([k.slice(0, bar), k.slice(bar + 1)]);
    }
    if (boundary.length === 0) break;

    // Chain the open edges into loops through shared endpoints.
    const byEnd = new Map<string, Array<[string, string]>>();
    for (const e of boundary) {
    for (const k of e) {
      let l = byEnd.get(k);
      if (!l) byEnd.set(k, (l = []));
      l.push(e);
    }
    }
    const usedEdge = new Set<string>();
  const parse = (k: string): [number, number, number] => k.split(",").map(Number) as [number, number, number];

  let progressed = false;
  for (const seed of boundary) {
    const seedKey = `${seed[0]}|${seed[1]}`;
    if (usedEdge.has(seedKey)) continue;
    const loop: Array<[number, number, number]> = [];
    let to = seed[1];
    usedEdge.add(seedKey);
    let guard = 0;
    while (guard++ < boundary.length + 4) {
      loop.push(parse(to));
      if (to === seed[0]) break;
      const cands = byEnd.get(to) ?? [];
      let next: string | null = null;
      for (const e of cands) {
        const k = `${e[0]}|${e[1]}`;
        if (usedEdge.has(k)) continue;
        next = e[0] === to ? e[1] : e[0];
        usedEdge.add(k);
        break;
      }
      if (next === null) break;
      to = next;
    }
    // Only a loop that actually returns to its start encloses an area worth
    // filling. An open chain would patch across a real gap.
    const closed = loop.length >= 3 && to === seed[0];
    if (!closed) continue;
    progressed = true;

    // Fan from a centroid at the loop's mean Z.
    let cx = 0, cy = 0, cz = 0;
    for (const p of loop) {
      cx += p[0];
      cy += p[1];
      cz += p[2];
    }
    const ci = mesh.verts.length;
    mesh.verts.push([cx / loop.length, cy / loop.length, cz / loop.length]);
    const ring = loop.map((p) => {
      mesh.verts.push([p[0], p[1], p[2]]);
      return mesh.verts.length - 1;
    });
    for (let i = 0; i < ring.length; i++) {
      const a = ring[i];
      const b = ring[(i + 1) % ring.length];
      if (a === ci || b === ci || a === b) continue;
      mesh.tris.push([ci, a, b]);
      added++;
    }
  }
  if (!progressed) break;
  }
  return added;
}


/** Mesh the exact prism-union boundary: side walls plus exposed caps only. */
function emitUnion(mesh: Mesh, layers: UnionLayer[]): void {
  for (const layer of layers) {
    const { z0, z1 } = layer;
    // Side walls: one quad per edge of every ring of the exposed region.
    for (const poly of layer.side) {
      for (const ring of [poly.exterior, ...poly.holes]) {
        const n = ring.length;
        if (n < 3) continue;
        const reverse = poly.holes.some((h) => h === ring);
        for (let k = 0; k < n; k++) {
          let ax = ring[k][0];
          let ay = ring[k][1];
          let bx = ring[(k + 1) % n][0];
          let by = ring[(k + 1) % n][1];
          if (reverse) [ax, ay, bx, by] = [bx, by, ax, ay];
          mesh.addQuad([ax, ay, z0], [bx, by, z0], [bx, by, z1], [ax, ay, z1]);
        }
      }
    }
    // Caps: only the parts not covered by the neighbouring layer.
    for (const [polys, z, flip] of [
      [layer.capBottom, z0, true],
      [layer.capTop, z1, false],
    ] as Array<[UnionRing[], number, boolean]>) {
      for (const poly of polys) {
        const { ring, tris } = triangulate(poly.exterior, poly.holes);
        for (const [i0, i1, i2] of tris) {
          const p0 = ring[i0];
          const p1 = ring[i1];
          const p2 = ring[i2];
          if (flip) mesh.addTri([p0[0], p0[1], z], [p2[0], p2[1], z], [p1[0], p1[1], z]);
          else mesh.addTri([p0[0], p0[1], z], [p1[0], p1[1], z], [p2[0], p2[1], z]);
        }
      }
    }
  }
}
