# gcode-mcp

MCP server that reconstructs 3D models from sliced gcode, inspects slices, and
compares a reconstruction against the original mesh.

Slicing is one-way: the model goes in, toolpath comes out. This server walks
that path backwards — layer by layer it traces the extrusion contours back into
closed polygons, separates islands from holes, and meshes them back into a solid
you can open in a slicer.

Pure TypeScript. No native modules, no Python.

Works with OrcaSlicer, AnycubicSlicerNext, PrusaSlicer and Cura gcode.

## Install

```bash
git clone https://github.com/vitkuz573/gcode-mcp.git
cd gcode-mcp
npm install
npm run build
```

## opencode config

Add this to `~/.config/opencode/opencode.jsonc`, adjusting the two paths to
wherever you cloned the repo:

```jsonc title="~/.config/opencode/opencode.jsonc"
{
  "$schema": "https://opencode.ai/config.json",
  "mcp": {
    "servers": {
      "gcode": {
        "type": "local",
        "command": ["node", "/path/to/gcode-mcp/dist/index.js"],
        "cwd": "/path/to/gcode-mcp"
      }
    }
  }
}
```

Then `opencode mcp list` should show `✓ gcode connected`.

## Tools

### `gcode_info`
Inspect a slice: slicer and version, layer counts, object names, per-feature
extrusion histogram, bounding box, whether a thumbnail is embedded, and the
original slice settings. Read-only, and the fastest way to confirm a file
parses before spending time on reconstruction.

### `gcode_to_model`
Reconstruct a solid from the toolpath. This is the main tool.

`mode` picks which extrusion features participate:

| mode | contents |
| --- | --- |
| `shell` (default) | perimeters, top/bottom, solid infill. Drops supports and sparse infill. |
| `solid` | shell plus sparse and gap infill |
| `no_support` | same as `solid`, name kept for readability |
| `outer` | outer wall only |
| `all` | everything, supports included |

Other useful options: `objects` to keep one plate entry by name, `layerStep`
with `layerHeight` for a fast low-res preview, `format` for `stl` / `3mf` /
`both`.

### `gcode_verify`
Check a mesh: watertightness, manifold edges, volume, area, bbox. Pass
`reference` to also get symmetric nearest-surface distances against the
original — mean, RMS, p50/p95/p99/max. Bounding-box centres are aligned first,
so a reconstruction sitting on the build plate compares correctly against a
reference centred on its own origin.

### `gcode_preview`
Contact sheet of selected layers, colour-coded by feature. Good for confirming
that a tricky gcode parses the way you expect before reconstructing.

### `gcode_thumbnail`
Pull the base64 PNG out of the gcode header. Most slicers embed a rendered
preview, which is faster than rendering anything.

### `gcode_settings`
The slice parameters embedded in the file: layer height, nozzle, filament,
temperatures, densities, support settings.

### `gcode_compare`
Diff two gcode files — layer counts, extrusion totals, per-feature counts,
size deltas, and which settings differ. Use it to check what a re-slice changed.

## How reconstruction works

1. **Parse.** Extrusion moves are grouped into layers via `;LAYER_CHANGE` /
   `;Z:` and tagged by `;TYPE:` (Outer wall, Internal solid infill, Support, …).
   Travel moves and retractions are skipped, so only real material counts.

2. **Trace contours.** Each layer's segments become an edge graph on a snapped
   lattice. Slicers leave a ~0.04 mm seam where a contour starts and ends, so
   endpoints alone never close a loop: chains are walked out and then stitched
   at whichever ends fall within tolerance. `snapXY` additionally rounds every
   contour point onto a shared lattice so near-identical contours on
   neighbouring layers land on identical coordinates.

3. **Nest.** Loops are sorted by area and each is tested for containment, which
   separates outer loops (islands) from holes without a full boolean pass.

4. **Mesh.** Each island becomes a closed shell per layer: caps triangulated by
   ear clipping with hole bridging, plus side walls. Coincident caps between
   neighbouring layers are deliberately left in place — they cancel in the
   volume integral and every slicer unions them away.

## Known limitation: manifold, not watertight

On a dense model the shell comes out **closed but not manifold** — a handful of
edges are used four times instead of twice. This happens where two layers'
contours happen to share an XY line, so their coincident caps meet the other
layer's walls.

Resolving it properly needs a boolean union of the per-layer polygons, which is
not implemented. What is implemented instead:

- The shell stays **closed** (zero boundary edges) by default.
- `weldTol` defaults to **0**. Welding fusing nearby-but-distinct contours is
  what turns a closed shell into an open one.
- `splitNonManifold: true` clears the over-shared edges, but leaves the new
  edges used once, so it trades 4 non-manifold edges for a batch of boundary
  ones. Off by default.

Every slicer and mesh-repair tool resolves this without complaint. The `notes`
field in the `to_model` report always states the current manifold status and
what to try.

Verified on a 359-layer, 783k-segment model: 664k triangles, 49.4 cm³, closed,
4 non-manifold edges, ~7 s single-threaded.

## Development

Run these from the repo root:

```bash
npm run build          # tsc -> dist/
npx tsc -p tsconfig.tools.json   # dev harnesses -> dist-tools/

# reconstruct and diff against the original
node dist-tools/tools/probe.js model.gcode reference.3mf

# find islands whose cap triangulation leaves vertices unreferenced
node dist-tools/tools/probe-tri.js model.gcode
```

## Related

- [anycubic-mcp](https://github.com/vitkuz573/anycubic-mcp) — Anycubic Cloud printers
- [slicer-mcp](https://github.com/vitkuz573/slicer-mcp) — headless slicing via OrcaSlicer

The intended pipeline: slice with `slicer-mcp`, recover the model with
`gcode-mcp`, verify against the original, then upload and print with
`anycubic-mcp`.

## License

MIT