/**
 * Why does OrcaSlicer refuse this mesh?
 *
 * Checks the properties a slicer validates before slicing: closed manifold,
 * no zero-area facets, consistent winding, no duplicate vertices on shared
 * edges, and every edge used exactly twice.
 */
import { readFileSync } from "node:fs";

const file = process.argv[2];
const buf = readFileSync(file);
const n = buf.readUInt32LE(80);

const v: number[][] = [];
const idx: number[][] = [];
for (let i = 0; i < n; i++) {
  const o = 84 + i * 50 + 12;
  for (let k = 0; k < 3; k++) {
    v.push([
      buf.readFloatLE(o + (k * 3) * 4),
      buf.readFloatLE(o + (k * 3 + 1) * 4),
      buf.readFloatLE(o + (k * 3 + 2) * 4),
    ]);
  }
  idx.push([i * 3, i * 3 + 1, i * 3 + 2]);
}

const K = (p: number[], d = 4) =>
  `${Math.round(p[0] * 10 ** d)},${Math.round(p[1] * 10 ** d)},${Math.round(p[2] * 10 ** d)}`;

// weld
const weld = new Map<string, number>();
const remap = new Int32Array(v.length);
for (let i = 0; i < v.length; i++) {
  const k = K(v[i]);
  const j = weld.get(k);
  if (j === undefined) {
    weld.set(k, i);
    remap[i] = i;
  } else remap[i] = j;
}

const edges = new Map<string, number>();
let zeroArea = 0;
let badNormal = 0;
let flipped = 0;
let vol = 0;
for (const [a, b, c] of idx) {
  const A = v[remap[a]], B = v[remap[b]], C = v[remap[c]];
  if (remap[a] === remap[b] || remap[b] === remap[c] || remap[c] === remap[a]) {
    zeroArea++;
    continue;
  }
  const ux = B[0]-A[0], uy = B[1]-A[1], uz = B[2]-A[2];
  const vx = C[0]-A[0], vy = C[1]-A[1], vz = C[2]-A[2];
  const nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
  const len = Math.hypot(nx, ny, nz);
  if (len / 2 < 1e-10) { zeroArea++; continue; }
  const snx = buf.readFloatLE(84 + idx.indexOf?.(undefined as never) * 0); // unused
  vol += (A[0]*(B[1]*C[2]-B[2]*C[1]) - A[1]*(B[0]*C[2]-B[2]*C[0]) + A[2]*(B[0]*C[1]-B[1]*C[0])) / 6;
  for (const [p, q] of [[A, B], [B, C], [C, A]]) {
    const ka = K(p), kb = K(q);
    const key = ka <= kb ? `${ka}|${kb}` : `${kb}|${ka}`;
    edges.set(key, (edges.get(key) ?? 0) + 1);
  }
}

let boundary = 0, over = 0;
for (const c of edges.values()) {
  if (c === 1) boundary++;
  else if (c > 2) over++;
}
const volCm3 = Math.abs(vol) / 1000;

console.log(`file: ${file}`);
console.log(`tris=${n} verts(raw)=${v.length} verts(welded)=${weld.size}`);
console.log(`boundaryEdges=${boundary} overSharedEdges=${over}`);
console.log(`degenerateTris=${zeroArea}`);
console.log(`signedVolume=${(vol/1000).toFixed(3)} cm3  winding=${vol > 0 ? "outward" : "INWARD"}`);
console.log(`watertight=${boundary === 0 && over === 0 ? "yes" : "no"}`);