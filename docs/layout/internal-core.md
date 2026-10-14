# decad file layout: internal packages, core

Internal packages for arithmetic, records, geometry and mesh construction.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/proof/` | Exact dyadic arithmetic, rational intervals and float rounding. |
| `internal/measurement/` | Private bounded reading carriers, `ExactnessOf` and the `FiniteValues` gate. Public types live in `measurement.go`. See API §5.3, §6. |
| `internal/pair/` | Shared contact relation and reading types. |
| `internal/pair/box/` | Exact box paths, contact, oriented, sphere-box and cylinder-box proofs, patches, clips and witnesses. |
| `internal/pair/sphere/` | Exact sphere-pair relation, gap and bounded response witness. See contact-geometry §4.3. |
| `internal/pair/planar/` | Planar solid relations, gaps, patches, support faces, and convexity. |
| `internal/sweeppath/` | Pair paths, validation, rounded poses, and motion travel bounds. |
| `internal/planarsnapshot/` | Planar contact snapshots, cached records and pose gates. See multibody §9.1–§9.2. |
| `internal/placedruling/` | Placed cylinder staging, support and relation proofs. See contact-geometry §4.5. |
| `internal/facetproof/` | Faceted shell audits, placement, similarity and revolve sector-separation proofs, sector seams and angular gaps. See `docs/evaluator-design.md` §9. |
| `internal/pointcone/` | Point-loft cone crossing samples, far-cap subdivision and contact split, cap and triangle crossing bounds, and sector cap paths, subdivision, merged angles and root-edge order, and the strict-side offset of a cone meridian (`offset.go`). See loft-point and loft-face-sections designs. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/selectorquery/` | Selector matching, query rendering, residual counts, and face-role indexes. See API §9. |
| `internal/provenance/` | Producer identity, feature-role reference and the `RoleBody`/`RoleCapStart`/`RoleCapEnd` names the topology nodes hold; root `FeatureRef` is the public form. See evaluator §3. |
| `internal/surfacegeom/` | Private face and edge geometry carriers. Public surface and curve variants live in `topology.go`; `Edge.Curve` and `Face.Surface` convert the node's curve and surface to the public variant. See API §6.1, surface §6. |
| `internal/surfaceplace/` | Rigid placement of face surfaces and edge curves, and the placed circle bound. See surface §6. |
| `internal/reportvocab/` | Contact and sweep enums; Verify reports, motion verification records, survey diagnostics, and pair, interference and sheet outcomes. |
| `internal/extent/` | Unit bounds and stop levels for extents. See API §8.1, evaluator §5/§6. |
| `internal/patchchain/` | `PatchEdges` chain partition, plane proof, orientation, segment record and area bounds. See surface §5.2. |
| `internal/sectionaudit/` | Rewrite audit order, signed area, cutback, crossing, contact and nesting checks. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks, coalescing, junction vertices, rim convexity, and analytic survey and modify loops over recorded segments. |
| `internal/classbgeom/` | Class-B boolean geometry and shifted tool records. See general-boolean §3 B, §5. |
| `internal/brepgeom/` / `internal/stackedbrep/` | BRep geometry, measurements, extents, face surveys, middle-rim audit and topology. See general-boolean §4–§5. |
| `internal/throughshell/` | Through-cut recognition, strips and rims. See modify-general §3. |
| `internal/capedge/` | Convex cap-edge cutter admission, quarter-cylinder profile, chamfer wedge and the P8 top front edge test. See vertex-blend §2. |
| `internal/prismshell/` | Prism side-opening sections, shell wall bands, boss rectangles and bands, slab levels and stack faces. See shell-opening §3–§5, modify-reach §9.2. |
| `internal/revolveaxis/` | Axis frames, walks, side/contact gates, snap and section charges, extent readings, the simple-revolution radial proof, and wall and cap denotations. See evaluator §6. |
| `internal/revolveshell/` | Effective-meridian survey, removed-run and side-segment role checks, and axis, side and off-axis shell wall sections and displacement. See modify-reach §9.3. |
| `internal/revolvemass/` | Revolve moments and centroid bounds. |
| `internal/revolveangle/` | Exact angle denotations and certified sweep extremes. See evaluator §6 and sweep §3. |
| `internal/sweeparc/` | Exact circle carriers, rational vectors, axis lines, plane gates and angle bounds. See sweep §3. |
| `internal/sweepinput/` | Recorded Sweep `Path`, exact path records, tangent gates, path validation, recorded section gates and closed-path class gates. See sweep §3, §5, §15.4. |
| `internal/sweepmitre/` | Mitred profile gates, exact sections, placement, rounding, triangle assembly and measurements. See sweep §16. |
| `internal/coil/` | Coil magnitude, profile side and pitch gates, preflight gates, stations, axis coordinates, moments and closed forms. See helix §4, §5, §7. |
| `internal/coilshell/` | Builds the held coil shell, mesh proofs and wall-area enclosures. See helix §5, §7, §8. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames over recorded line and arc spans, and frames along a recorded path. See sweep §3.2. |
| `internal/prismrecord/` | Plain data of the prism record and its plane-local lifts, point bounds, lift factor, centroid envelope, placement operand, cap frames, mass record, exact box, section-plane test and the modify exact-section gate; root `prismPayload` embeds it. See evaluator §5, §8. |
| `internal/momentinput/` | Profile records, exact record comparison, rectangle bounds, axis-aligned rectangle footprints, moments, loop enclosed areas and first moments, walk cache and coordinate envelopes. See evaluator §4 and spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Recorded-loop and band chording, section loop and walk clearance refusals, mesh topology and joined-mesh orientation, bounds and audits. See tessellation-design. |
| `internal/partialband/` | Selected cap-edge fillet band sample layouts and open boundary polylines, and exact partial-chamfer quad gates and mass. See loop-fillet §7.1, modify-general §4.3a. |
| `internal/stationbound/` | Bounds circular chord stations, offset cap stations and chain-sweep arc stations against their exact recorded parameters. |
| `internal/loftmesh/` | Loft and chain pairing gates, stations, assembly, mass sums, projected cap proof, body reading finiteness gate, mirrored-cylinder bridge terms and reserve, cap chamfer source bound, split-ring and collar grids, triangle clearance and middle-collar proofs, and grouped contact scans. |
| `internal/smoothloft/` | Three-section homothety, independent rectangles with exact height curves for Section, varying curves, and four-section rounded rectangles. See loft-sections design. |
| `internal/loftfillet/` | Loft fit/circle root-fillet candidate, tangent-root certificate, connector and carrier departures, section contact audits and circle-bore containment. See loft §17–§18. |
| `internal/revolvemesh/` | Revolve rings, cells, caps, cap frames, construction and area proofs, quarter-turn cap-rim blend stations and bounds, and sector-bore cap fans, station steps and projected patches. |
| `internal/revolveplan/` | Revolve mesh walk resolution, coordinate ceilings and chord counts. See tessellation §8. |
| `internal/revolvesampling/` | Revolve meridian junctions, stations, samples and section readings. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, cell area and volume bounds, and chain-arc source points and area density. See tessellation §8–§11, sweep §15.9. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over recorded `Point2`; indexed triangles and chording refusals. |
| `internal/loftcylinder/` | Exact cap XY audits, source circle and connector gates, ring distances and occupancy, cap chamfer clearance, triangles, band strips, compaction and audits, triangle radii, root radius gaps, and radial and wedge gates of the loft/cylinder Union. See loft-cylinder-boolean design. |
| `internal/topo/` | Node data the root topology handles hold by value (Vertex, Edge, CoEdge, Loop, Face, FaceShell, Lump, Body, Document, Mesh), with their links and the document node's identity minting and liveness reads; `Payload`, which every feature payload implements, and `Env`. See evaluator §3 and §8. |
