/**
 * Diagnostic: find islands whose cap triangulation leaves vertices
 * unreferenced. Those are the ones that produce open boundary edges.
 */
import { parseFile } from "../src/parse.js";
import { filterSegments, loopsToIslands, segmentsToLoops } from "../src/geometry.js";
import { triangulate } from "../src/triangulate.js";

async function main() {
  const gcode = process.argv[2];
  const p = await parseFile(gcode);
  let total = 0;
  let bad = 0;
  for (const layer of p.layers) {
    const segs = filterSegments(layer.segs, "shell");
    if (segs.length === 0) continue;
    const loops = segmentsToLoops(segs, layer.z, {});
    for (const isl of loopsToIslands(loops)) {
      total++;
      const { ring, tris } = triangulate(
        isl.outer.pts,
        isl.holes.map((h) => h.pts)
      );
      const used = new Set<number>();
      for (const t of tris) {
        used.add(t[0]);
        used.add(t[1]);
        used.add(t[2]);
      }
      let missing = 0;
      for (let i = 0; i < ring.length; i++) if (!used.has(i)) missing++;
      if (missing > 0) {
        bad++;
        if (bad <= 10) {
          console.log(
            `z=${layer.z.toFixed(2)} outer=${isl.outer.pts.length} ring=${ring.length} ` +
              `tris=${tris.length} unreferenced=${missing} holes=${isl.holes.length}`
          );
        }
      }
    }
  }
  console.log(`islands=${total} with_unreferenced=${bad}`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});