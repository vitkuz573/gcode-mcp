/**
 * Mesh IO and comparison: STL / 3MF load, stats, symmetric surface distance.
 */

import { closeSync, existsSync, openSync, readFileSync, readSync, statSync, writeFileSync } from "node:fs";
import { inflateRawSync } from "node:zlib";
import { Mesh, round } from "./mesh.js";

export interface MeshData {
  verts: Float64Array; // 3 * n
  tris: Uint32Array; // 3 * m
  nVerts: number;
  nTris: number;
  source: string;
}

export function loadStl(path: string): MeshData {
  const buf = readFileSync(path);
  if (buf.length >= 84) {
    const n = buf.readUInt32LE(80);
    if (84 + n * 50 === buf.length) {
      const verts = new Float64Array(n * 9);
      const tris = new Uint32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const o = 84 + i * 50 + 12;
        for (let v = 0; v < 3; v++) {
          for (let c = 0; c < 3; c++) {
            verts[i * 9 + v * 3 + c] = buf.readFloatLE(o + (v * 3 + c) * 4);
          }
          tris[i * 3 + v] = i * 3 + v;
        }
      }
      return { verts, tris, nVerts: n * 3, nTris: n, source: path };
    }
  }
  // ASCII fallback.
  const txt = buf.toString("utf8");
  const vRe = /vertex\s+(\S+)\s+(\S+)\s+(\S+)/g;
  const vs: number[] = [];
  let m: RegExpExecArray | null;
  while ((m = vRe.exec(txt))) vs.push(parseFloat(m[1]), parseFloat(m[2]), parseFloat(m[3]));
  const fRe = /facet\s+normal[^\n]*\n\s*outer\s+loop[^\n]*\n([\s\S]*?)endloop/g;
  const ts: number[] = [];
  while ((m = fRe.exec(txt))) {
    const idx: number[] = [];
    const iRe = /vertex\s+(\d+)/g;
    let k: RegExpExecArray | null;
    while ((k = iRe.exec(m[1]))) idx.push(parseInt(k[1], 10) - 1);
    for (let i = 1; i < idx.length - 1; i++) ts.push(idx[0], idx[i], idx[i + 1]);
  }
  return {
    verts: Float64Array.from(vs),
    tris: Uint32Array.from(ts),
    nVerts: vs.length / 3,
    nTris: ts.length / 3,
    source: path,
  };
}

/** Read one entry out of a ZIP (store or deflate). */
function zipRead(path: string, entryName: string): Buffer | null {
  const buf = readFileSync(path);
  // Locate the central directory.
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
    if (name === entryName) {
      const lNameLen = buf.readUInt16LE(localOff + 26);
      const lExtraLen = buf.readUInt16LE(localOff + 28);
      const start = localOff + 30 + lNameLen + lExtraLen;
      const raw = buf.subarray(start, start + compSize);
      return method === 0 ? Buffer.from(raw) : inflateRawSync(raw);
    }
    off += 46 + nameLen + extraLen + commentLen;
  }
  return null;
}

export function load3mf(path: string): MeshData {
  // Prefer the largest object part: bulk geometry lives in one file.
  const names = listZipEntries(path)
    .filter((n) => n.toLowerCase().endsWith(".model"))
    .sort((a, b) => zipEntrySize(path, b) - zipEntrySize(path, a));

  const vs: number[] = [];
  const ts: number[] = [];
  for (const name of names) {
    const blob = zipRead(path, name);
    if (!blob) continue;
    const text = blob.toString("utf8");
    const mv = /<vertex\s+x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"/g;
    const base = vs.length / 3;
    let m: RegExpExecArray | null;
    let count = 0;
    while ((m = mv.exec(text))) {
      vs.push(parseFloat(m[1]), parseFloat(m[2]), parseFloat(m[3]));
      count++;
    }
    const mt = /<triangle\s+v1="(\d+)"\s+v2="(\d+)"\s+v3="(\d+)"/g;
    while ((m = mt.exec(text))) {
      ts.push(parseInt(m[1], 10) + base, parseInt(m[2], 10) + base, parseInt(m[3], 10) + base);
    }
    if (count > 0) break;
  }
  return {
    verts: Float64Array.from(vs),
    tris: Uint32Array.from(ts),
    nVerts: vs.length / 3,
    nTris: ts.length / 3,
    source: path,
  };
}

function listZipEntries(path: string): string[] {
  const buf = readFileSync(path);
  let eocd = -1;
  for (let i = buf.length - 22; i >= 0 && i > buf.length - 66000; i--) {
    if (buf.readUInt32LE(i) === 0x06054b50) {
      eocd = i;
      break;
    }
  }
  if (eocd < 0) return [];
  const count = buf.readUInt16LE(eocd + 10);
  const out: string[] = [];
  let off = buf.readUInt32LE(eocd + 16);
  for (let i = 0; i < count; i++) {
    if (buf.readUInt32LE(off) !== 0x02014b50) break;
    const nameLen = buf.readUInt16LE(off + 28);
    const extraLen = buf.readUInt16LE(off + 30);
    const commentLen = buf.readUInt16LE(off + 32);
    out.push(buf.toString("utf8", off + 46, off + 46 + nameLen));
    off += 46 + nameLen + extraLen + commentLen;
  }
  return out;
}

function zipEntrySize(path: string, name: string): number {
  const buf = readFileSync(path);
  let eocd = -1;
  for (let i = buf.length - 22; i >= 0 && i > buf.length - 66000; i--) {
    if (buf.readUInt32LE(i) === 0x06054b50) {
      eocd = i;
      break;
    }
  }
  if (eocd < 0) return 0;
  const count = buf.readUInt16LE(eocd + 10);
  let off = buf.readUInt32LE(eocd + 16);
  for (let i = 0; i < count; i++) {
    const nameLen = buf.readUInt16LE(off + 28);
    const extraLen = buf.readUInt16LE(off + 30);
    const commentLen = buf.readUInt16LE(off + 32);
    const nm = buf.toString("utf8", off + 46, off + 46 + nameLen);
    if (nm === name) return buf.readUInt32LE(off + 24);
    off += 46 + nameLen + extraLen + commentLen;
  }
  return 0;
}

export function loadMesh(path: string): MeshData {
  const ext = path.toLowerCase().slice(path.lastIndexOf("."));
  if (ext === ".stl") return loadStl(path);
  if (ext === ".3mf" || ext === ".3mfb") return load3mf(path);
  if (ext === ".step" || ext === ".stp") {
    throw new Error("STEP is not a mesh; slice it first (use slicer-mcp)");
  }
  const head = readFileSync(path).subarray(0, 2);
  if (head[0] === 0x50 && head[1] === 0x4b) return load3mf(path);
  return loadStl(path);
}

export function bboxOf(m: MeshData): number[] {
  if (m.nVerts === 0) return [0, 0, 0, 0, 0, 0];
  const lo = [Infinity, Infinity, Infinity];
  const hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < m.nVerts; i++) {
    for (let k = 0; k < 3; k++) {
      const v = m.verts[i * 3 + k];
      if (v < lo[k]) lo[k] = v;
      if (v > hi[k]) hi[k] = v;
    }
  }
  return [...lo, ...hi];
}

export function sizeOf(m: MeshData): number[] {
  const b = bboxOf(m);
  return [b[3] - b[0], b[4] - b[1], b[5] - b[2]];
}

export function volumeOf(m: MeshData): number {
  let total = 0;
  for (let i = 0; i < m.nTris; i++) {
    const a = m.tris[i * 3] * 3;
    const b = m.tris[i * 3 + 1] * 3;
    const c = m.tris[i * 3 + 2] * 3;
    const ax = m.verts[a];
    const ay = m.verts[a + 1];
    const az = m.verts[a + 2];
    const bx = m.verts[b];
    const by = m.verts[b + 1];
    const bz = m.verts[b + 2];
    const cx = m.verts[c];
    const cy = m.verts[c + 1];
    const cz = m.verts[c + 2];
    total +=
      ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
  }
  return total / 6;
}

export function areaOf(m: MeshData): number {
  let total = 0;
  for (let i = 0; i < m.nTris; i++) {
    const a = m.tris[i * 3] * 3;
    const b = m.tris[i * 3 + 1] * 3;
    const c = m.tris[i * 3 + 2] * 3;
    const ux = m.verts[b] - m.verts[a];
    const uy = m.verts[b + 1] - m.verts[a + 1];
    const uz = m.verts[b + 2] - m.verts[a + 2];
    const vx = m.verts[c] - m.verts[a];
    const vy = m.verts[c + 1] - m.verts[a + 1];
    const vz = m.verts[c + 2] - m.verts[a + 2];
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    total += Math.hypot(nx, ny, nz);
  }
  return total * 0.5;
}

/** Weld by rounding, then count edges. */
export function edgeStats(m: MeshData): {
  boundaryEdges: number;
  nonmanifoldEdges: number;
  watertight: boolean;
} {
  if (m.nTris === 0) return { boundaryEdges: 0, nonmanifoldEdges: 0, watertight: false };
  const ids = new Map<string, number>();
  const vid = (i: number): number => {
    const k = `${round(m.verts[i * 3], 5)},${round(m.verts[i * 3 + 1], 5)},${round(
      m.verts[i * 3 + 2],
      5
    )}`;
    let v = ids.get(k);
    if (v === undefined) {
      v = ids.size;
      ids.set(k, v);
    }
    return v;
  };
  const counts = new Map<number, number>();
  const key = (a: number, b: number): number => (a <= b ? a * 4194304 + b : b * 4194304 + a);
  for (let i = 0; i < m.nTris; i++) {
    const a = vid(m.tris[i * 3]);
    const b = vid(m.tris[i * 3 + 1]);
    const c = vid(m.tris[i * 3 + 2]);
    for (const [u, v] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const k = key(u, v);
      counts.set(k, (counts.get(k) ?? 0) + 1);
    }
  }
  let boundary = 0;
  let nonmanifold = 0;
  for (const c of counts.values()) {
    if (c === 1) boundary++;
    else if (c > 2) nonmanifold++;
  }
  return { boundaryEdges: boundary, nonmanifoldEdges: nonmanifold, watertight: boundary === 0 && nonmanifold === 0 };
}

/** Deterministic barycentric surface sampling. */
export function sampleSurface(m: MeshData, n: number, seed = 12345): Float64Array {
  if (m.nTris === 0) return new Float64Array(0);
  let s = seed >>> 0;
  const rnd = (): number => {
    // xorshift32 for reproducibility across platforms
    s ^= s << 13;
    s >>>= 0;
    s ^= s >>> 17;
    s ^= s << 5;
    s >>>= 0;
    return s / 4294967296;
  };

  const areas = new Float64Array(m.nTris);
  let total = 0;
  for (let i = 0; i < m.nTris; i++) {
    const a = m.tris[i * 3] * 3;
    const b = m.tris[i * 3 + 1] * 3;
    const c = m.tris[i * 3 + 2] * 3;
    const ux = m.verts[b] - m.verts[a];
    const uy = m.verts[b + 1] - m.verts[a + 1];
    const uz = m.verts[b + 2] - m.verts[a + 2];
    const vx = m.verts[c] - m.verts[a];
    const vy = m.verts[c + 1] - m.verts[a + 1];
    const vz = m.verts[c + 2] - m.verts[a + 2];
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    areas[i] = Math.hypot(nx, ny, nz) * 0.5;
    total += areas[i];
  }

  const cdf = new Float64Array(m.nTris);
  let acc = 0;
  for (let i = 0; i < m.nTris; i++) {
    acc += total > 0 ? areas[i] / total : 1 / m.nTris;
    cdf[i] = acc;
  }

  const out = new Float64Array(n * 3);
  for (let k = 0; k < n; k++) {
    const target = rnd();
    let lo = 0;
    let hi = m.nTris - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (cdf[mid] < target) lo = mid + 1;
      else hi = mid;
    }
    const i = lo;
    const a = m.tris[i * 3] * 3;
    const b = m.tris[i * 3 + 1] * 3;
    const c = m.tris[i * 3 + 2] * 3;
    let u = rnd();
    let w = rnd();
    if (u + w > 1) {
      u = 1 - u;
      w = 1 - w;
    }
    for (let c2 = 0; c2 < 3; c2++) {
      const pa = m.verts[a + c2];
      const pb = m.verts[b + c2];
      const pc = m.verts[c + c2];
      out[k * 3 + c2] = pa + u * (pb - pa) + w * (pc - pa);
    }
  }
  // Mix in some raw vertices, which sharp features often lack.
  const extra = Math.floor(n / 4);
  for (let k = 0; k < extra; k++) {
    const vi = Math.floor(rnd() * m.nVerts);
    for (let c2 = 0; c2 < 3; c2++) {
      out[(n - extra + k) * 3 + c2] = m.verts[vi * 3 + c2];
    }
  }
  return out;
}

/** Nearest-target distance for each query point, via a spatial hash. */
function nnDist(query: Float64Array, nq: number, target: Float64Array, nt: number, maxRadius: number): Float64Array {
  const cell = Math.max(maxRadius, 1e-3);
  const buckets = new Map<string, number[]>();
  const tk = new Int32Array(nt * 3);
  for (let i = 0; i < nt; i++) {
    const cx = Math.floor(target[i * 3] / cell);
    const cy = Math.floor(target[i * 3 + 1] / cell);
    const cz = Math.floor(target[i * 3 + 2] / cell);
    tk[i * 3] = cx;
    tk[i * 3 + 1] = cy;
    tk[i * 3 + 2] = cz;
    const k = `${cx},${cy},${cz}`;
    let arr = buckets.get(k);
    if (!arr) buckets.set(k, (arr = []));
    arr.push(i);
  }

  const best = new Float64Array(nq).fill(Infinity);
  for (let q = 0; q < nq; q++) {
    const px = query[q * 3];
    const py = query[q * 3 + 1];
    const pz = query[q * 3 + 2];
    const bx = Math.floor(px / cell);
    const by = Math.floor(py / cell);
    const bz = Math.floor(pz / cell);
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (let dz = -1; dz <= 1; dz++) {
          const arr = buckets.get(`${bx + dx},${by + dy},${bz + dz}`);
          if (!arr) continue;
          for (const i of arr) {
            const d = Math.hypot(
              target[i * 3] - px,
              target[i * 3 + 1] - py,
              target[i * 3 + 2] - pz
            );
            if (d < best[q]) best[q] = d;
          }
        }
      }
    }
  }
  return best;
}

export interface Comparison {
  samples: number;
  distanceMm: Record<string, number>;
  aToB: { mean: number; p95: number };
  bToA: { mean: number; p95: number };
  sizeMm: { a: number[]; b: number[]; delta: number[] };
  volumeMm3: { a: number; b: number; deltaPct: number | null };
  areaMm2: { a: number; b: number; deltaPct: number | null };
  triangles: { a: number; b: number };
  warning?: string;
}

export function compareMeshes(
  a: MeshData,
  b: MeshData,
  samples = 40000,
  maxRadius = 6,
  seed = 7
): Comparison {
  // The reference is the original part, centred on its own origin; the
  // reconstruction sits on the build plate at the slicer's placement. Align
  // their bounding-box centres before measuring, otherwise every sample reports
  // a distance equal to the plate offset.
  const align = (m: MeshData): MeshData => {
    const b = bboxOf(m);
    const cx = (b[0] + b[3]) / 2;
    const cy = (b[1] + b[4]) / 2;
    const cz = (b[2] + b[5]) / 2;
    const v = new Float64Array(m.verts);
    for (let i = 0; i < m.nVerts; i++) {
      v[i * 3] -= cx;
      v[i * 3 + 1] -= cy;
      v[i * 3 + 2] -= cz;
    }
    return { ...m, verts: v };
  };

  const pa = sampleSurface(align(a), samples, seed);
  const pb = sampleSurface(align(b), samples, seed + 1);
  const dAb = nnDist(pa, samples, pb, samples, maxRadius);
  const dBa = nnDist(pb, samples, pa, samples, maxRadius);

  const all: number[] = [];
  let missing = 0;
  const push = (arr: Float64Array) => {
    for (let i = 0; i < arr.length; i++) {
      if (Number.isFinite(arr[i])) all.push(arr[i]);
      else missing++;
    }
  };
  push(dAb);
  push(dBa);
  if (all.length === 0) {
    return { samples: 0 } as unknown as Comparison;
  }
  all.sort((x, y) => x - y);
  const pct = (p: number): number => all[Math.min(all.length - 1, Math.floor((p / 100) * all.length))];
  const mean = all.reduce((s, v) => s + v, 0) / all.length;
  const rms = Math.sqrt(all.reduce((s, v) => s + v * v, 0) / all.length);

  const stat = (arr: Float64Array) => {
    const f = [...arr].filter(Number.isFinite).sort((x, y) => x - y);
    return {
      mean: round(f.reduce((s, v) => s + v, 0) / f.length, 4),
      p95: round(f[Math.min(f.length - 1, Math.floor(0.95 * f.length))], 4),
    };
  };

  const sa = sizeOf(a);
  const sb = sizeOf(b);
  const va = Math.abs(volumeOf(a));
  const vb = Math.abs(volumeOf(b));
  const aa = areaOf(a);
  const ab = areaOf(b);

  const out: Comparison = {
    samples: all.length,
    distanceMm: {
      mean: round(mean, 4),
      rms: round(rms, 4),
      p50: round(pct(50), 4),
      p95: round(pct(95), 4),
      p99: round(pct(99), 4),
      max: round(all[all.length - 1], 4),
    },
    aToB: stat(dAb),
    bToA: stat(dBa),
    sizeMm: {
      a: sa.map((v) => round(v, 3)),
      b: sb.map((v) => round(v, 3)),
      delta: sb.map((v, i) => round(v - sa[i], 3)),
    },
    volumeMm3: {
      a: round(va, 2),
      b: round(vb, 2),
      deltaPct: va > 0 ? round(((vb - va) / va) * 100, 3) : null,
    },
    areaMm2: {
      a: round(aa, 2),
      b: round(ab, 2),
      deltaPct: aa > 0 ? round(((ab - aa) / aa) * 100, 3) : null,
    },
    triangles: { a: a.nTris, b: b.nTris },
  };
  if (missing > 0) {
    out.warning = `${missing} samples had no match within ${maxRadius} mm`;
  }
  return out;
}

export function meshReport(m: MeshData, label: string): Record<string, unknown> {
  const es = edgeStats(m);
  const b = bboxOf(m);
  return {
    label,
    file: m.source,
    vertices: m.nVerts,
    triangles: m.nTris,
    bboxMin: b.slice(0, 3).map((v) => round(v, 3)),
    bboxMax: b.slice(3).map((v) => round(v, 3)),
    sizeMm: sizeOf(m).map((v) => round(v, 3)),
    volumeMm3: round(Math.abs(volumeOf(m)), 2),
    signedVolumeMm3: round(volumeOf(m), 2),
    areaMm2: round(areaOf(m), 2),
    watertight: es.watertight,
    boundaryEdges: es.boundaryEdges,
    nonmanifoldEdges: es.nonmanifoldEdges,
  };
}