#!/usr/bin/env python3
"""Exact union of stacked layer prisms, per interface.

Every layer of a slice is a flat prism: its footprint swept from the layer's
bottom to its top. The printable solid is the union of those prisms, and its
boundary is not what per-layer meshing produces - two layers whose footprints
overlap put a top cap and a bottom cap in the same plane, which is an interior
face, and draw the wall between them twice.

The z ranges tile: layer i's top is layer i+1's bottom. That fixes the boundary
exactly. At a height strictly inside layer i the only material present is F_i,
so the cross-section there is F_i and the lateral surface over that layer's
height is the WHOLE of dF_i. Only the horizontal faces are trimmed, by the
neighbours above and below:

    side_i   = dF_i                                (whole boundary, full height)
    capTop_i = F_i - F_{i+1}                       (up-facing at z1)
    capBot_i = F_i - F_{i-1}                       (down-facing at z0)

Trimming `side` by the neighbours instead is wrong twice over. It walls fewer
curves than the real lateral surface, leaving the shell open; and the slivers it
walls fragment into closed rings that sweep phantom volume.

Coordinates are snapped to a SNAP grid before any polygon is built. Slicers
leave a ~0.04 mm seam where a contour starts and ends, so two layers of a
vertical wall differ by a hair and their difference shatters into slivers.
Snapping makes a repeated contour land on identical vertices, so successive
layers produce byte-identical rings and the difference is exactly empty. Doing
the arithmetic in integer lattice units also makes GEOS exact, which keeps
neighbouring footprints free of the hairline slivers that fragment.

A cap's boundary is assembled from both footprints' outlines and switches from
one to the other wherever they cross, so it needs vertices that neither outline
has. Every ring is therefore split at the crossings on the planes it touches,
which is what makes each cap edge land exactly on a wall edge.

Reads JSON on stdin, writes JSON on stdout. Rings are returned as
{"exterior": [[x, y], ...], "holes": [[[x, y], ...], ...]}. `side` is a flat
list of rings instead, exteriors counter-clockwise and holes clockwise, because
a side wall is a strip rather than an area, and the winding is what puts its
normal on the void side.
"""

from __future__ import annotations

import bisect
import json
import sys

try:
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
except ImportError:  # pragma: no cover
    json.dump({"ok": False, "error": "shapely is not installed"}, sys.stdout)
    sys.exit(2)

# Snap grid, in mm. Coarser than the slicer's ~0.04 mm seam so that a contour
# repeated on consecutive layers lands on the same lattice point.
SNAP = 0.05

# Two z values closer than this are the same plane. Generous on purpose: the
# parser derives each layer's bottom from its own Z and height, so a shared
# plane drifts by ~1e-6 mm per hundred layers. This is still 200x smaller than
# the layer height, so a genuine gap is never mistaken for a seam.
Z_EPS = 1e-3

# Regions smaller than this are contour noise, not material.
MIN_AREA = 1e-6

# How far a ring edge may be from a split point and still count as on it, in
# lattice units. A hundredth of a cell: enough to absorb GEOS's rounding, far
# too little to pull in a point that belongs to a different curve.
SPLIT_TOL = 1e-6


def _to_lattice(pts):
    """Round a ring onto the integer SNAP lattice, dropping repeats."""
    out = []
    for p in pts:
        try:
            c = (int(round(p[0] / SNAP)), int(round(p[1] / SNAP)))
        except (TypeError, ValueError):
            continue
        if out and out[-1] == c:
            continue
        out.append(c)
    while len(out) > 2 and out[0] == out[-1]:
        out.pop()
    return out


def _to_mm(c):
    return [round(c[0] * SNAP, 4), round(c[1] * SNAP, 4)]


def _ring(pts):
    return [_to_mm(c) for c in pts]


def _clean(geom):
    """Repair an invalid geometry and drop empties, or return None."""
    if geom is None or geom.is_empty:
        return None
    if not geom.is_valid:
        geom = geom.buffer(0)
        if geom is None or geom.is_empty:
            return None
    if geom.area < MIN_AREA:
        return None
    return geom


def _signed_area(ring):
    a = 0
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return a * 0.5


def _geom_rings(geom):
    """Every ring of a polygon set, in lattice units: exteriors CCW, holes CW."""
    out = []
    if geom is None or geom.is_empty:
        return out
    for g in getattr(geom, "geoms", [geom]):
        if g.geom_type != "Polygon" or g.is_empty or g.area < MIN_AREA:
            continue
        for k, r in enumerate([g.exterior, *g.interiors]):
            c = list(r.coords)
            if len(c) > 1 and c[0] == c[-1]:
                c = c[:-1]
            if len(c) < 3:
                continue
            want_ccw = k == 0
            if (_signed_area(c) > 0) != want_ccw:
                c.reverse()
            out.append(c)
    return out


def _side_rings(geom):
    """Every boundary ring of a footprint: exteriors CCW, holes CW.

    The mesher builds a quad from consecutive ring vertices sweeping z0 -> z1,
    whose normal is (dy, -dx) of the ring direction. That points away from the
    material for a counter-clockwise exterior, and into the material for a
    counter-clockwise hole - so a hole has to run the other way round. Getting
    this wrong makes a void add material instead of subtracting it: a tube of
    outer radius R and inner r, height h, then measures pi*(3R^2-r^2)/3*h
    instead of pi*(R^2-r^2)*h.
    """
    return _geom_rings(geom)


def _emit(geom, split=None):
    """Flatten a geometry into polygons with exterior + hole rings.

    `split` is a list of points to insert wherever one falls inside a ring edge,
    so a cap can be given the same vertices as the walls it meets.
    """
    out = []
    if geom is None or geom.is_empty:
        return out
    for g in getattr(geom, "geoms", [geom]):
        if g.is_empty or g.area < MIN_AREA:
            continue
        polys = [g] if g.geom_type == "Polygon" else [p for p in getattr(g, "geoms", [])]
        for poly in polys:
            if poly.is_empty or poly.area < MIN_AREA:
                continue
            ext = list(poly.exterior.coords)
            if len(ext) > 1 and ext[0] == ext[-1]:
                ext = ext[:-1]
            holes = []
            for ring in poly.interiors:
                h = list(ring.coords)
                if len(h) > 1 and h[0] == h[-1]:
                    h = h[:-1]
                if len(h) >= 3:
                    holes.append(h)
            if len(ext) < 3:
                continue
            rings = [ext] + holes
            if split:
                rings = _split_rings(rings, split)
            ext = rings[0]
            holes = [_ring(r) for r in rings[1:] if len(r) >= 3]
            if len(ext) >= 3:
                out.append({"exterior": _ring(ext), "holes": holes})
    return out


def _footprint(layer):
    """Layer islands -> one shapely geometry, in lattice units."""
    parts = []
    for isl in layer.get("islands", []):
        ext = _to_lattice(isl.get("outer") or [])
        if len(ext) < 3:
            continue
        holes = [_to_lattice(h) for h in (isl.get("holes") or [])]
        holes = [h for h in holes if len(h) >= 3]
        try:
            p = Polygon(ext, holes)
        except Exception:
            continue
        if p.is_empty or p.area < MIN_AREA:
            continue
        if not p.is_valid:
            p = p.buffer(0)
            if p.is_empty or p.area < MIN_AREA:
                continue
        parts.append(p)
    if not parts:
        return None
    return _clean(parts[0] if len(parts) == 1 else unary_union(parts))


def _on_segment(px, py, ax, ay, bx, by, tol2):
    """Is (px, py) strictly inside segment a-b, within tol2 squared?"""
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    if l2 <= 0.0:
        return None
    t = ((px - ax) * dx + (py - ay) * dy) / l2
    if t <= 0.0 or t >= 1.0:
        return None
    ex = ax + t * dx - px
    ey = ay + t * dy - py
    if ex * ex + ey * ey > tol2:
        return None
    return t


def _split_rings(rings, pts, tol=None):
    """Insert the given points wherever they fall inside a ring edge.

    A cap's boundary is assembled from BOTH footprints' outlines, and switches
    from one to the other wherever they cross. Those crossings are vertices of
    the cap that neither outline has, so a wall edge spanning one would not line
    up with the cap and the shell would stay open. Inserting each crossing into
    both outlines makes every cap edge land on a wall edge.

    The tolerance is SPLIT_TOL, not a hair's breadth: GEOS evaluates a crossing
    to full double precision, and a point that is mathematically on an edge can
    land a hair to either side of it. Testing with a margin keeps every ring's
    idea of "does this point lie on that edge" the same, which is the whole
    point - the rings have to agree exactly.

    Candidate points come from a list sorted by x and narrowed to the edge's own
    x range, which keeps this near-linear: wall edges are ~0.4 mm long.
    """
    tol = SPLIT_TOL
    if not pts or not rings:
        return rings
    tol2 = tol * tol
    order = sorted(range(len(pts)), key=lambda k: pts[k][0])
    sx = [pts[k][0] for k in order]
    sp = [pts[k] for k in order]
    out = []
    for ring in rings:
        n = len(ring)
        if n < 2:
            out.append(ring)
            continue
        acc = []
        for j in range(n):
            ax, ay = ring[j]
            bx, by = ring[(j + 1) % n]
            acc.append((ax, ay))
            lo = bisect.bisect_left(sx, min(ax, bx) - tol)
            hi = bisect.bisect_right(sx, max(ax, bx) + tol)
            hits = []
            for k in range(lo, hi):
                px, py = sp[k]
                if ay == by:
                    if abs(py - ay) > tol:
                        continue
                t = _on_segment(px, py, ax, ay, bx, by, tol2)
                if t is not None:
                    hits.append((t, px, py))
            if hits:
                hits.sort()
                last = (ax, ay)
                for _, px, py in hits:
                    if (px, py) != last:
                        acc.append((px, py))
                        last = (px, py)
        out.append(acc)
    return out


def main() -> None:
    data = json.load(sys.stdin)
    layers = data.get("layers", [])
    n = len(layers)

    polys = [_footprint(layer) for layer in layers]

    # Which pairs are genuinely stacked? layerStep > 1 and sparse models leave
    # gaps in Z, and across a gap the footprint above is not a neighbour: the
    # layer is fully exposed at that plane and needs a whole cap there.
    def neighbour(i, step):
        j = i + step
        if j < 0 or j >= n or polys[j] is None:
            return None
        # The neighbour above must start exactly where this layer ends.
        touch = (
            abs(layers[j]["z0"] - layers[i]["z1"]) <= Z_EPS
            if step > 0
            else abs(layers[i]["z0"] - layers[j]["z1"]) <= Z_EPS
        )
        return polys[j] if touch else None

    def cap_geom(cur, other):
        """The exposed part of `cur` on the neighbour's side of the interface."""
        if other is None:
            return cur
        # An identical footprint leaves nothing exposed, and skipping it keeps
        # the two walls' end vertices the same on both sides of the plane.
        if other.equals(cur):
            return None
        return _clean(cur.difference(other))

    caps_b = [None] * n
    caps_t = [None] * n
    sides = [None] * n
    for i in range(n):
        cur = polys[i]
        if cur is None:
            continue
        sides[i] = _side_rings(cur)
        caps_b[i] = cap_geom(cur, neighbour(i, -1))
        caps_t[i] = cap_geom(cur, neighbour(i, +1))

    # Vertices present on each interface plane, still in lattice units. The top
    # cap of the layer below and the bottom cap of the layer above share one.
    # Both the walls and the caps are split by this set, so the two describe the
    # same curve with the same segmentation and every edge pairs up.
    def plane_points(geoms):
        pts = []
        for g in geoms:
            for r in _geom_rings(g):
                pts.extend(r)
        return pts

    plane_pts = [None] * (n + 1)
    plane_pts[0] = plane_points([caps_b[0], polys[0]])
    for i in range(1, n):
        plane_pts[i] = plane_points([caps_t[i - 1], caps_b[i], polys[i - 1], polys[i]])
    plane_pts[n] = plane_points([caps_t[n - 1], polys[n - 1]])

    def near(*idxs):
        pts = []
        for k in idxs:
            if 0 <= k <= n:
                pts.extend(plane_pts[k])
        return pts

    out = []
    for i in range(n):
        entry = {"z0": layers[i]["z0"], "z1": layers[i]["z1"]}
        if polys[i] is None:
            out.append({**entry, "side": [], "capBottom": [], "capTop": []})
            continue
        # One wall ring spans the whole layer, so its vertices serve BOTH
        # interface planes. The caps it meets bring their own windows: the cap
        # above a ring is split with one plane more to its right than the ring
        # is, so the ring has to be split with that plane too or the two stop
        # lining up. Matching both caps therefore means the union of their two
        # three-plane windows.
        out.append(
            {
                **entry,
                "side": [_ring(r) for r in _split_rings(sides[i], near(i - 1, i, i + 1, i + 2))],
                "capBottom": _emit(caps_b[i], near(i - 1, i, i + 1)),
                "capTop": _emit(caps_t[i], near(i, i + 1, i + 2)),
            }
        )

    json.dump({"ok": True, "layers": out}, sys.stdout)


if __name__ == "__main__":
    main()
