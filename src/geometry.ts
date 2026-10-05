/**
 * Geometry: extrusion segments -> closed 2D loops -> islands with holes.
 *
 * Slicers leave a small seam gap (typically ~0.04 mm) where a contour starts
 * and ends, so exact endpoint matching alone never closes a loop. The approach
 * is: build an edge graph on a snapped lattice, extract maximal open chains,
 * then stitch chain ends that fall within `stitchTol`.
 */

import { SHELL_FEATURES, SPARSE_FEATURES, SUPPORT_FEATURES, type Seg } from "./parse.js";

export type Pt = [number, number];

export class Loop {
  pts: Pt[];
  z: number;

  constructor(pts: Pt[], z: number) {
    this.pts = pts;
    this.z = z;
  }

  get n(): number {
    return this.pts.length;
  }

  signedArea(): number {
    let a = 0;
    const p = this.pts;
    const m = p.length;
    for (let k = 0; k < m; k++) {
      const [x0, y0] = p[k];
      const [x1, y1] = p[(k + 1) % m];
      a += x0 * y1 - x1 * y0;
    }
    return a * 0.5;
  }

  area(): number {
    return Math.abs(this.signedArea());
  }

  perimeter(): number {
    let s = 0;
    const p = this.pts;
    const m = p.length;
    for (let k = 0; k < m; k++) {
      const a = p[k];
      const b = p[(k + 1) % m];
      s += Math.hypot(b[0] - a[0], b[1] - a[1]);
    }
    return s;
  }

  centroid(): Pt {
    const a = this.signedArea();
    const p = this.pts;
    const m = p.length;
    if (Math.abs(a) < 1e-12) {
      let sx = 0;
      let sy = 0;
      for (const q of p) {
        sx += q[0];
        sy += q[1];
      }
      return [sx / m, sy / m];
    }
    let cx = 0;
    let cy = 0;
    for (let k = 0; k < m; k++) {
      const [x0, y0] = p[k];
      const [x1, y1] = p[(k + 1) % m];
      const cr = x0 * y1 - x1 * y0;
      cx += (x0 + x1) * cr;
      cy += (y0 + y1) * cr;
    }
    return [cx / (6 * a), cy / (6 * a)];
  }

  bbox(): [number, number, number, number] {
    let x0 = Infinity;
    let y0 = Infinity;
    let x1 = -Infinity;
    let y1 = -Infinity;
    for (const [x, y] of this.pts) {
      if (x < x0) x0 = x;
      if (x > x1) x1 = x;
      if (y < y0) y0 = y;
      if (y > y1) y1 = y;
    }
    return [x0, y0, x1, y1];
  }

  /** Even-odd ray cast. */
  pointIn(px: number, py: number): boolean {
    const p = this.pts;
    const m = p.length;
    let inside = false;
    let j = m - 1;
    for (let i = 0; i < m; i++) {
      const xi = p[i][0];
      const yi = p[i][1];
      const xj = p[j][0];
      const yj = p[j][1];
      if (yi > py !== yj > py) {
        const t = (py - yi) / (yj - yi);
        if (px < xi + t * (xj - xi)) inside = !inside;
      }
      j = i;
    }
    return inside;
  }
}

export class Island {
  outer: Loop;
  holes: Loop[];
  z: number;

  constructor(outer: Loop, holes: Loop[], z: number) {
    this.outer = outer;
    this.holes = holes;
    this.z = z;
  }

  area(): number {
    return this.outer.area() - this.holes.reduce((s, h) => s + h.area(), 0);
  }
}

type Key = number; // lattice cell index encoded as a single number pair string
type CellKey = string;

function cellKey(ix: number, iy: number): CellKey {
  return `${ix},${iy}`;
}

/** Snap a contour point onto a global XY lattice. */
export function snapXY(x: number, y: number, grid: number): Pt {
  if (grid <= 0) return [x, y];
  return [Math.round(x / grid) * grid, Math.round(y / grid) * grid];
}

/** Collapse points that repeat the same coordinate anywhere in the ring. */
export function dedupeRing(pts: Pt[], tol = 1e-4): Pt[] {
  const seen = new Set<string>();
  const out: Pt[] = [];
  for (const p of pts) {
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

/** Ramer-Douglas-Peucker on a closed ring. */
function rdp(ring: Pt[], eps: number): Pt[] {
  const n = ring.length;
  if (n < 4 || eps <= 0) return ring;

  const reduce = (seq: Pt[]): Pt[] => {
    if (seq.length < 3) return seq;
    const [ax, ay] = seq[0];
    const [bx, by] = seq[seq.length - 1];
    const dx = bx - ax;
    const dy = by - ay;
    const norm = Math.hypot(dx, dy);
    let dmax = 0;
    let idx = 0;
    for (let i = 1; i < seq.length - 1; i++) {
      const [px, py] = seq[i];
      const d =
        norm < 1e-12
          ? Math.hypot(px - ax, py - ay)
          : Math.abs(dy * px - dx * py + bx * ay - by * ax) / norm;
      if (d > dmax) {
        dmax = d;
        idx = i;
      }
    }
    if (dmax > eps) {
      return [...reduce(seq.slice(0, idx + 1)).slice(0, -1), ...reduce(seq.slice(idx))];
    }
    return [
      [ax, ay],
      [bx, by],
    ];
  };

  let iB = 0;
  let best = -1;
  for (let i = 0; i < n; i++) {
    const d = Math.hypot(ring[i][0] - ring[0][0], ring[i][1] - ring[0][1]);
    if (d > best) {
      best = d;
      iB = i;
    }
  }
  const fwd = reduce(ring.slice(0, iB + 1)).slice(0, -1);
  const rev = reduce([...ring.slice(iB), ring[0]]).slice(0, -1);
  const out = [...fwd, ...rev];
  return out.length >= 3 ? out : ring;
}

type Edge = [CellKey, CellKey];

function otherEnd(edge: Edge, node: CellKey): CellKey {
  return edge[0] === node ? edge[1] : edge[0];
}

function dist(a: Pt, b: Pt): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

export interface LoopOptions {
  eps?: number;
  minArea?: number;
  simplify?: number;
  stitchTol?: number;
  snapGrid?: number;
}

/** Trace closed loops from one layer's extrusion segments. */
export function segmentsToLoops(segs: Seg[], z: number, opts: LoopOptions = {}): Loop[] {
  if (segs.length === 0) return [];
  const eps = opts.eps ?? 0.02;
  const minArea = opts.minArea ?? 0.05;
  const simplify = opts.simplify ?? 0;
  const snapGrid = opts.snapGrid ?? 0;
  const stitchTol = opts.stitchTol ?? Math.max(0.09, eps * 4);

  const raw = new Map<CellKey, Pt>();
  const adj = new Map<CellKey, Edge[]>();
  const edges = new Set<string>();

  const node = (px: number, py: number): CellKey => {
    const [sx, sy] = snapXY(px, py, snapGrid);
    const k = cellKey(Math.round(sx / eps), Math.round(sy / eps));
    if (!raw.has(k)) raw.set(k, [sx, sy]);
    return k;
  };

  for (const s of segs) {
    if (s.length <= 1e-9) continue;
    const a = node(s.x0, s.y0);
    const b = node(s.x1, s.y1);
    if (a === b) continue;
    const ek: Edge = a <= b ? [a, b] : [b, a];
    const sig = `${ek[0]}|${ek[1]}`;
    if (edges.has(sig)) continue;
    edges.add(sig);
    let list = adj.get(ek[0]);
    if (!list) adj.set(ek[0], (list = []));
    list.push(ek);
    list = adj.get(ek[1]);
    if (!list) adj.set(ek[1], (list = []));
    list.push(ek);
  }

  // ---- extract maximal chains ----
  // Parallel arrays instead of a Set of stringified edges: at ~2500 segments
  // per layer with heavy branching, rebuilding string keys on every candidate
  // scan dominates runtime (measured 107s for a single layer).
  const sigList = [...edges];
  const nEdges = sigList.length;
  const edgeA: CellKey[] = new Array(nEdges);
  const edgeB: CellKey[] = new Array(nEdges);
  for (let i = 0; i < nEdges; i++) {
    const parts = sigList[i].split("|") as [CellKey, CellKey];
    edgeA[i] = parts[0];
    edgeB[i] = parts[1];
  }
  const edgeUsed = new Uint8Array(nEdges);
  const adjList = new Map<CellKey, number[]>();
  for (let i = 0; i < nEdges; i++) {
    let l = adjList.get(edgeA[i]);
    if (!l) adjList.set(edgeA[i], (l = []));
    l.push(i);
    l = adjList.get(edgeB[i]);
    if (!l) adjList.set(edgeB[i], (l = []));
    l.push(i);
  }
  const otherEndIdx = (ea: CellKey, eb: CellKey, node: CellKey): CellKey =>
    ea === node ? eb : ea;

  /** Pick the continuation at a vertex: unused, not backwards, straightest. */
  const pick = (nodeKey: CellKey, prevDir: [number, number] | null, avoid: CellKey): number => {
    const list = adjList.get(nodeKey);
    if (!list) return -1;
    let first = -1;
    let count = 0;
    for (const ei of list) {
      if (edgeUsed[ei]) continue;
      if (first < 0) first = ei;
      count++;
    }
    if (count === 0) return -1;
    if (count === 1 || prevDir === null) return first;
    const [vx, vy] = prevDir;
    const here = raw.get(nodeKey)!;
    let best = first;
    let bestScore = -Infinity;
    for (const ei of list) {
      if (edgeUsed[ei]) continue;
      const nxt = otherEndIdx(edgeA[ei], edgeB[ei], nodeKey);
      if (nxt === avoid) continue;
      const there = raw.get(nxt)!;
      const wx = there[0] - here[0];
      const wy = there[1] - here[1];
      const n1 = Math.hypot(vx, vy);
      const n2 = Math.hypot(wx, wy);
      if (n1 < 1e-12 || n2 < 1e-12) continue;
      const score = (vx * wx + vy * wy) / (n1 * n2);
      if (score > bestScore) {
        bestScore = score;
        best = ei;
      }
    }
    return best;
  };

  const chains: CellKey[][] = [];
  for (let seed = 0; seed < nEdges; seed++) {
    if (edgeUsed[seed]) continue;
    edgeUsed[seed] = 1;
    const nodes: CellKey[] = [edgeA[seed], edgeB[seed]];

    // grow forward
    let node = nodes[1];
    let here = raw.get(nodes[0])!;
    let there = raw.get(nodes[1])!;
    let prevDir: [number, number] = [there[0] - here[0], there[1] - here[1]];
    for (;;) {
      const ei = pick(node, prevDir, nodes[nodes.length - 2]);
      if (ei < 0) break;
      edgeUsed[ei] = 1;
      const nxt = otherEndIdx(edgeA[ei], edgeB[ei], node);
      if (nxt === nodes[0]) {
        nodes.push(nxt);
        break;
      }
      here = raw.get(node)!;
      there = raw.get(nxt)!;
      prevDir = [there[0] - here[0], there[1] - here[1]];
      nodes.push(nxt);
      node = nxt;
    }

    // grow backward
    if (nodes.length > 1) {
      node = nodes[0];
      here = raw.get(nodes[0])!;
      there = raw.get(nodes[1])!;
      prevDir = [here[0] - there[0], here[1] - there[1]];
      for (;;) {
        const ei = pick(node, prevDir, nodes[1]);
        if (ei < 0) break;
        edgeUsed[ei] = 1;
        const nxt = otherEndIdx(edgeA[ei], edgeB[ei], node);
        if (nxt === nodes[nodes.length - 1]) {
          nodes.unshift(nxt);
          break;
        }
        here = raw.get(node)!;
        there = raw.get(nxt)!;
        prevDir = [there[0] - here[0], there[1] - here[1]];
        nodes.unshift(nxt);
        node = nxt;
      }
    }
    chains.push(nodes);
  }

  // ---- stitch open ends ----
  const isClosed = (c: CellKey[]): boolean => {
    if (c.length < 2) return true;
    if (c[0] === c[c.length - 1]) return true;
    return dist(raw.get(c[0])!, raw.get(c[c.length - 1])!) <= stitchTol;
  };

  let items = chains.filter((c) => c.length >= 2);
  let merged = true;
  while (merged) {
    merged = false;
    let best: [number, number, number, number] | null = null;
    let bestD = stitchTol;
    for (let i = 0; i < items.length; i++) {
      if (isClosed(items[i])) continue;
      for (let j = i + 1; j < items.length; j++) {
        if (isClosed(items[j])) continue;
        const ci = items[i];
        const cj = items[j];
        for (const ai of [0, ci.length - 1]) {
          for (const aj of [0, cj.length - 1]) {
            const ni = ci[ai];
            const nj = cj[aj];
            if (ni === nj) continue;
            const d = dist(raw.get(ni)!, raw.get(nj)!);
            if (d <= bestD) {
              bestD = d;
              best = [i, j, ai, aj];
            }
          }
        }
      }
    }
    if (best) {
      const [i, j, ai, aj] = best;
      const ci = items[i];
      const cj = items[j];
      const head = ai === 0 ? [...ci].reverse() : ci;
      const tail = aj === 0 ? cj : [...cj].reverse();
      const joined = [...head, ...tail];
      items = items.filter((_, k) => k !== i && k !== j);
      items.push(joined);
      merged = true;
    }
  }

  const loops: Loop[] = [];
  for (const c of items) {
    if (c.length < 3) continue;
    const a = c[0];
    const b = c[c.length - 1];
    if (a !== b && dist(raw.get(a)!, raw.get(b)!) > stitchTol) continue;
    const nodeIds = a === b ? c.slice(0, -1) : c;
    if (nodeIds.length < 3) continue;
    let pts = nodeIds.map((k) => raw.get(k)! as Pt);
    pts = dedupeRing(pts);
    if (pts.length < 3) continue;
    if (simplify > 0) {
      pts = dedupeRing(rdp(pts, simplify));
      if (pts.length < 3) continue;
    }
    const lp = new Loop(pts, z);
    if (lp.area() < minArea) continue;
    loops.push(lp);
  }
  return loops;
}

/**
 * Classify loops into islands, optionally treating contained loops as holes.
 *
 * Nesting alone cannot tell a hole from a second island: an outer wall and the
 * inner wall one wall-thickness inside it form two concentric contours with
 * solid material between them, which looks exactly like an outer loop with a
 * hole. Picking wrong either punches a void into a solid wall or seals a real
 * cavity shut, so `allowHoles` defaults to false and every contour becomes its
 * own island. Overlapping islands are the correct reading of a slice: the
 * material between two walls is solid, and letting the solids overlap matches
 * that.
 */
export function loopsToIslands(loops: Loop[], allowHoles = true): Island[] {
  if (loops.length === 0) return [];
  const items = loops.slice();
  for (const lp of items) {
    if (lp.signedArea() < 0) lp.pts.reverse();
  }
  // Bigger loops first, so any container is examined before what it contains.
  items.sort((a, b) => b.area() - a.area());

  const islands: Island[] = [];
  const holes: Loop[][] = [];
  for (const lp of items) {
    const [cx, cy] = lp.centroid();
    let placed = false;
    if (allowHoles) {
      for (let k = 0; k < islands.length; k++) {
        if (islands[k].outer.pointIn(cx, cy)) {
          holes[k].push(lp);
          placed = true;
          break;
        }
      }
    }
    if (!placed) {
      const isl = new Island(lp, [], lp.z);
      islands.push(isl);
      // Share the same array, otherwise holes pushed below never reach the island.
      holes.push(isl.holes);
    }
  }
  return islands;
}

export type FeatureMode =
  | "shell"
  | "solid"
  | "no_support"
  | "outer"
  | "support"
  | "all";

/** Select which features participate in the reconstruction. */
export function filterSegments(
  segs: Seg[],
  mode: FeatureMode = "shell",
  objects?: string[]
): Seg[] {
  let allow: Set<string> | null;
  switch (mode) {
    case "shell":
      allow = SHELL_FEATURES;
      break;
    case "solid":
    case "no_support": {
      allow = new Set([...SHELL_FEATURES, ...SPARSE_FEATURES]);
      break;
    }
    case "outer":
      allow = new Set(["outer wall"]);
      break;
    case "support":
      allow = SUPPORT_FEATURES;
      break;
    case "all":
      allow = null;
      break;
  }
  const objSet = objects && objects.length ? new Set(objects) : null;
  const out: Seg[] = [];
  for (const s of segs) {
    if (allow !== null && !allow.has(s.feat.trim().toLowerCase())) continue;
    if (objSet && !objSet.has(s.obj)) continue;
    out.push(s);
  }
  return out;
}