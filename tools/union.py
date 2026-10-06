#!/usr/bin/env python3
"""Exact union of stacked layer prisms, per interface.

Every layer of a slice is a flat prism: its footprint swept from the layer's
bottom to its top. The printable solid is the union of those prisms, and its
boundary is not what per-layer meshing produces - two layers whose footprints
overlap put a top cap and a bottom cap in the same plane, which is an interior
face, and draw the wall between them twice.

This computes the boundary correctly, using shapely:

    side_i   = footprint_i  -  footprint_{i-1}  -  footprint_{i+1}
    capBot_i = footprint_i  -  footprint_{i-1}
    capTop_i = footprint_i  -  footprint_{i+1}

`side_i` is what remains exposed sideways at layer i, `capBot_i` / `capTop_i`
the horizontal faces that are actually on the outside. Regions absent at the
stack's ends are treated as empty.

Reads JSON on stdin, writes JSON on stdout. Rings are returned as
{"exterior": [[x, y], ...], "holes": [[[x, y], ...], ...]}.
"""

from __future__ import annotations

import json
import sys

try:
    from shapely.geometry import MultiPolygon, Polygon
    from shapely.ops import unary_union
except ImportError:  # pragma: no cover
    json.dump({"ok": False, "error": "shapely is not installed"}, sys.stdout)
    sys.exit(2)

# Snap coordinates to this grid before unioning. Slicers leave a seam gap of
# ~0.04 mm where a contour begins and ends, so without snapping two layers of a
# vertical wall differ slightly and their union leaves hairline slivers.
SNAP = 0.05


def _ring(pts: list[list[float]]) -> list[list[float]]:
    return [[round(p[0], 4), round(p[1], 4)] for p in pts]


def _clean(geom):
    """Drop slivers and repair invalid rings, or return None if nothing is left."""
    if geom is None or geom.is_empty:
        return None
    if not geom.is_valid:
        geom = geom.buffer(0)
    parts = []
    geoms = getattr(geom, "geoms", [geom])
    for g in geoms:
        if g.is_empty or g.area < 1e-6:
            continue
        parts.append(g)
    if not parts:
        return None
    return unary_union(parts) if len(parts) > 1 else parts[0]


def _emit(geom) -> list[dict]:
    """Flatten a geometry into polygons with exterior + hole rings."""
    out = []
    if geom is None or geom.is_empty:
        return out
    geoms = getattr(geom, "geoms", [geom])
    for g in geoms:
        if g.is_empty or g.area < 1e-6:
            continue
        polys = [g] if g.geom_type == "Polygon" else [p for p in getattr(g, "geoms", [])]
        for poly in polys:
            if poly.is_empty or poly.area < 1e-6:
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
                    holes.append(_ring(h))
            if len(ext) >= 3:
                out.append({"exterior": _ring(ext), "holes": holes})
    return out


def _emit_rings(geom) -> list[dict]:
    """
    Extract boundary curves as "polygons" whose exterior is the curve itself.

    A side wall is a strip, not an area, so the rings are returned as polygons
    with no interior. The mesh builder only needs the ring order, and treats a
    hole-free ring as a simple band.
    """
    out = []
    if geom is None or geom.is_empty:
        return out
    geoms = getattr(geom, "geoms", [geom])
    for g in geoms:
        if g.is_empty:
            continue
        if g.geom_type == "LineString":
            coords = list(g.coords)
        elif g.geom_type == "LinearRing":
            coords = list(g.coords)
        elif g.geom_type == "Polygon":
            coords = list(g.exterior.coords)
        else:
            continue
        if len(coords) > 1 and coords[0] == coords[-1]:
            coords = coords[:-1]
        if len(coords) >= 2:
            out.append({"exterior": _ring(coords), "holes": []})
    return out


def main() -> None:
    data = json.load(sys.stdin)
    layers = data.get("layers", [])

    # Build each layer's footprint as a shapely geometry.
    polys: list = []
    for layer in layers:
        parts = []
        for isl in layer.get("islands", []):
            ext = isl.get("outer") or []
            if len(ext) < 3:
                continue
            holes = [h for h in (isl.get("holes") or []) if len(h) >= 3]
            try:
                p = Polygon(ext, holes)
            except Exception:
                continue
            if not p.is_valid:
                p = p.buffer(0)
            if p.is_empty or p.area < 1e-6:
                continue
            parts.append(p)
        merged = _clean(unary_union(parts)) if parts else None
        if merged is not None and SNAP > 0:
            merged = _clean(merged.buffer(0))
        polys.append(merged)

    n = len(layers)
    out = []
    for i in range(n):
        cur = polys[i]
        below = polys[i - 1] if i > 0 else None
        above = polys[i + 1] if i + 1 < n else None
        entry: dict = {"z0": layers[i]["z0"], "z1": layers[i]["z1"]}

        if cur is None:
            out.append({**entry, "side": [], "capBottom": [], "capTop": []})
            continue

        # Side walls: the part of this layer's footprint not covered by its
        # neighbours. Walling the whole difference also walls the curves where
        # this layer abuts a neighbour, and those duplicate the neighbour's own
        # wall rather than cancelling it.
        exposed = cur
        for other in (below, above):
            if other is not None:
                exposed = exposed.difference(other)
        exposed = _clean(exposed)
        side = _emit(exposed)

        cap_bottom = cur.difference(below) if below is not None else cur
        cap_top = cur.difference(above) if above is not None else cur

        out.append(
            {
                **entry,
                "side": side,
                "capBottom": _emit(_clean(cap_bottom)),
                "capTop": _emit(_clean(cap_top)),
            }
        )

    json.dump({"ok": True, "layers": out}, sys.stdout)


if __name__ == "__main__":
    main()