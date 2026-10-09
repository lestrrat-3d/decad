# decad file layout: internal packages, core

Internal packages for arithmetic, records, geometry and mesh construction.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/proof/` | Exact dyadic arithmetic, rational intervals and float rounding. |
| `internal/measurement/` | Bounded reading types. See API §5.3, §6. |
| `internal/pair/` | Shared contact relation and reading types. |
| `internal/pair/box/` | Exact box paths, contact, oriented, sphere-box and cylinder-box proofs, patches, clips and witnesses. |
| `internal/pair/sphere/` | Exact sphere-pair relation, gap and bounded response witness. See contact-geometry §4.3. |
| `internal/pair/planar/` | Planar solid relations, gaps, patches, support faces, and convexity. |
| `internal/sweeppath/` | Pair paths, validation, rounded poses, and motion travel bounds. |
| `internal/planarsnapshot/` | Exact prism and held-mesh snapshots for planar contact. See multibody §9.1. |
| `internal/placedruling/` | Placed cylinder staging, support and relation proofs. See contact-geometry §4.5. |
| `internal/facetproof/` | Faceted shell audits and proofs. See `docs/evaluator-design.md` §9. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/selectorquery/` | Selector matching, query rendering, and residual counts. See API §9. |
| `internal/surfacegeom/` | Sealed face and edge variants and placement transforms. See API §6.1, surface §6. |
| `internal/reportvocab/` | Contact/sweep carriers, enums and pair memo; Verify, Motion, Linkage and JointBox reports and survey diagnostics. |
| `internal/extent/` | Extent variants, unit bounds and stop levels. See API §8.1, evaluator §5/§6. |
| `internal/patchchain/` | `Body.Patch` chain partition, plane proof, orientation, segment record and area bounds. See surface §5.2. |
| `internal/sectionaudit/` | Rewrite audit order, signed area, cutback, crossing, contact and nesting checks. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks, coalescing, and analytic survey and modify loops over recorded segments. |
| `internal/classbgeom/` | Class-B boolean geometry. See general-boolean §3 B, §5. |
| `internal/brepgeom/` / `internal/stackedbrep/` | BRep geometry, measurements, extents, face surveys and topology. See general-boolean §4–§5. |
| `internal/throughshell/` | Through-cut recognition, strips and rims. See modify-general §3. |
| `internal/capedge/` | Convex cap-edge cutter admission and quarter-cylinder profile. See vertex-blend §2. |
| `internal/prismshell/` | Prism side-opening sections, slab levels and stack faces. See shell-opening §3–§5. |
| `internal/revolveaxis/` | Axis input, walks, side/contact gates, snap and section charges, and extent readings. See evaluator §6. |
| `internal/revolveshell/` | Effective-meridian survey, removed-run checks, and shell wall sections and displacement. See modify-reach §9.3. |
| `internal/revolvemass/` | Revolve moments and centroid bounds. |
| `internal/revolveangle/` | Exact angle denotations and certified sweep extremes. See evaluator §6 and sweep §3. |
| `internal/sweeparc/` | Exact circle carriers, rational vectors, axis lines and angle bounds. See sweep §3. |
| `internal/sweepmitre/` | Mitred profile gates, exact sections and measurements. See sweep §16. |
| `internal/coil/` | Coil stations, axis coordinates, moments and closed forms. See helix §5, §7. |
| `internal/coilshell/` | Builds the held coil shell and mesh proofs. See helix §5, §8. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames. See sweep §3.2. |
| `internal/momentinput/` | Profile records, moments, walk cache and coordinate envelopes. See evaluator §4 and spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Recorded-loop and band chording, mesh topology, bounds and audits. See tessellation-design. |
| `internal/partialband/` | Selected cap-edge fillet band sample layouts and open boundary polylines. See loop-fillet §7.1. |
| `internal/stationbound/` | Bounds circular chord stations and offset cap stations against their exact recorded parameters. |
| `internal/loftmesh/` | Loft and chain pairing gates, stations, assembly, mass sums and mesh proofs. |
| `internal/revolvemesh/` | Revolve rings, cells, caps, construction and area proofs. |
| `internal/revolveplan/` | Revolve mesh walk resolution, coordinate ceilings and chord counts. See tessellation §8. |
| `internal/revolvesampling/` | Revolve meridian junctions, stations, samples and section readings. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, cell area and volume bounds. See tessellation §8–§11. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over recorded `Point2`; indexed triangles and chording refusals. |
