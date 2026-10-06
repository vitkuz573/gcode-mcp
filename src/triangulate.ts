/**
 * Triangulation of polygons with holes: ear clipping with hole bridging.
 */

import type { Pt } from "./geometry.js";

export function signedArea(ring: Pt[]): number {
  let a = 0;
  const n = ring.length;
  for (let i = 0; i < n; i++) {
    const [x0, y0] = ring[i];
    const [x1, y1] = ring[(i + 1) % n];
    a += x0 * y1 - x1 * y0;
  }
  return a * 0.5;
}

function pointInTriangle(p: Pt, a: Pt, b: Pt, c: Pt): boolean {
  const d1 = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
  const d2 = (c[0] - b[0]) * (p[1] - b[1]) - (c[1] - b[1]) * (p[0] - b[0]);
  const d3 = (a[0] - c[0]) * (p[1] - c[1]) - (a[1] - c[1]) * (p[0] - c[0]);
  const neg = d1 < -1e-12 || d2 < -1e-12 || d3 < -1e-12;
  const pos = d1 > 1e-12 || d2 > 1e-12 || d3 > 1e-12;
  return !(neg && pos);
}

function inRing(ring: Pt[], p: Pt): boolean {
  let inside = false;
  const n = ring.length;
  let j = n - 1;
  for (let i = 0; i < n; i++) {
    const [xi, yi] = ring[i];
    const [xj, yj] = ring[j];
    if (yi > p[1] !== yj > p[1]) {
      const t = (p[1] - yi) / (yj - yi);
      if (p[0] < xi + t * (xj - xi)) inside = !inside;
    }
    j = i;
  }
  return inside;
}

const ccw = (r: Pt[]): Pt[] => (signedArea(r) > 0 ? r : [...r].reverse());
const cw = (r: Pt[]): Pt[] => (signedArea(r) < 0 ? r : [...r].reverse());

function dist(a: Pt, b: Pt): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

/** Merge hole rings into the outer ring via bridge edges (duplicated verts). */
function bridgeHoles(outer: Pt[], holes: Pt[][]): Pt[] {
  if (holes.length === 0) return ccw(outer);
  let ring = ccw(outer);
  const hs = holes.map(cw).sort((a, b) => Math.max(...b.map((p) => p[0])) - Math.max(...a.map((p) => p[0])));

  for (const hole of hs) {
    let hmaxI = 0;
    for (let i = 1; i < hole.length; i++) if (hole[i][0] > hole[hmaxI][0]) hmaxI = i;
    const [hx, hy] = hole[hmaxI];

    let best: number | null = null;
    let bestD = Infinity;
    for (let i = 0; i < ring.length; i++) {
      const [ox, oy] = ring[i];
      if (ox < hx) continue;
      const d = (ox - hx) ** 2 + (oy - hy) ** 2;
      if (d < bestD && !inRing(hole, ring[i])) {
        bestD = d;
        best = i;
      }
    }
    if (best === null) {
      best = 0;
      let bd = Infinity;
      for (let i = 0; i < ring.length; i++) {
        const d = (ring[i][0] - hx) ** 2 + (ring[i][1] - hy) ** 2;
        if (d < bd) {
          bd = d;
          best = i;
        }
      }
    }

    const merged: Pt[] = [];
    for (let i = 0; i < ring.length; i++) {
      merged.push(ring[i]);
      if (i === best) {
        merged.push(hole[hmaxI]);
        for (let step = 1; step < hole.length; step++) {
          merged.push(hole[(hmaxI + step) % hole.length]);
        }
        merged.push(hole[hmaxI]);
        merged.push(ring[i]);
      }
    }
    ring = merged;
  }
  return ring;
}

/**
 * Collapse repeated coordinates (adjacent, or anywhere in the ring).
 *
 * A contour can legitimately pass through one coordinate twice: a seam, or a
 * spur where the outer wall meets another feature. Ear clipping cannot
 * represent that, so the copy it fails to reference shows up later as an open
 * boundary edge. Reducing to one occurrence keeps caps and side walls built
 * from the same vertex set.
 *
 * Call this on each ring BEFORE hole bridging, never on the merged ring:
 * bridging works by duplicating its two bridge vertices, so a dedupe after it
 * deletes the bridge and turns the hole into a pinch that ear clipping fills
 * with the wrong area.
 */
export function dedupeRing(ring: Pt[], tol = 1e-4): Pt[] {
  const seen = new Set<string>();
  const out: Pt[] = [];
  for (const p of ring) {
    const k = `${Math.round(p[0] / tol)},${Math.round(p[1] / tol)}`;
    if (seen.has(k)) continue;
    seen.add(k);
    out.push(p);
  }
  while (
    out.length > 2 &&
    Math.abs(out[0][0] - out[out.length - 1][0]) <= tol &&
    Math.abs(out[0][1] - out[out.length - 1][1]) <= tol
  ) {
    out.pop();
  }
  return out;
}

/** Drop collinear/duplicate vertices from the working index list. */
function dropDegenerate(idx: number[], ring: Pt[]): number {
  let removed = 0;
  let changed = true;
  while (changed && idx.length > 3) {
    changed = false;
    const m = idx.length;
    for (let k = 0; k < m; k++) {
      const a = ring[idx[(k - 1 + m) % m]];
      const b = ring[idx[k]];
      const c = ring[idx[(k + 1) % m]];
      const cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
      const d = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (Math.abs(cross) <= 1e-12 || d <= 1e-12) {
        idx.splice(k, 1);
        removed++;
        changed = true;
        break;
      }
    }
  }
  return removed;
}

const same = (a: Pt, b: Pt): boolean => Math.abs(a[0] - b[0]) <= 1e-9 && Math.abs(a[1] - b[1]) <= 1e-9;

function earClip(input: Pt[]): { ring: Pt[]; tris: Array<[number, number, number]> } {
  let ring = input;
  let n = ring.length;
  if (n < 3) return { ring, tris: [] };
  if (signedArea(ring) < 0) ring = [...ring].reverse();

  const idx = [...Array(n).keys()];
  const tris: Array<[number, number, number]> = [];
  let guard = 0;
  const limit = n * n + 64;

  while (idx.length > 3 && guard < limit) {
    guard++;
    const m = idx.length;
    let clipped = false;
    for (let k = 0; k < m; k++) {
      const i0 = idx[(k - 1 + m) % m];
      const i1 = idx[k];
      const i2 = idx[(k + 1) % m];
      const a = ring[i0];
      const b = ring[i1];
      const c = ring[i2];
      const cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
      if (cross <= 1e-14) continue;
      let ok = true;
      for (const j of idx) {
        if (j === i0 || j === i1 || j === i2) continue;
        const p = ring[j];
        // Compare coordinates, not indices: hole bridging duplicates its two
        // bridge vertices on purpose, and a seam repeats one point. Such a
        // vertex lies ON the triangle's boundary and must not veto the ear.
        if (same(p, a) || same(p, b) || same(p, c)) continue;
        if (pointInTriangle(p, a, b, c)) {
          ok = false;
          break;
        }
      }
      if (!ok) continue;
      tris.push([i0, i1, i2]);
      idx.splice(k, 1);
      clipped = true;
      break;
    }
    if (!clipped) {
      if (dropDegenerate(idx, ring) > 0) continue;
      break;
    }
  }
  if (idx.length === 3) tris.push([idx[0], idx[1], idx[2]]);
  return { ring, tris };
}

/** Triangulate a polygon with holes. Triangles index into the returned ring. */
export function triangulate(outer: Pt[], holes: Pt[][] = []): {
  ring: Pt[];
  tris: Array<[number, number, number]>;
} {
  // Clean each ring on its own, then bridge. See dedupeRing for why the order
  // matters: a dedupe after bridging would remove the bridge itself.
  const o = dedupeRing(outer);
  const hs = holes.map((h) => dedupeRing(h)).filter((h) => h.length >= 3);
  const { ring, tris } = earClip(bridgeHoles(o, hs));
  return { ring, tris };
}