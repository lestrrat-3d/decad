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
| `internal/facetproof/` | Faceted shell audits, placement, similarity and revolve sector-separation proofs, sector seams and angular gaps, and the faceted centroid allowance. See `docs/evaluator-design.md` §9. |
| `internal/facetedrecord/` | The faceted payload, its source groups and records, its placement, and the body builder over nodes: patch faces, topology, shells and voids, readings and bounds. Root `facetedPayload`, `facetGroup` and the record names alias them. See evaluator §8, §9. |
| `internal/pointcone/` | Point-loft cone crossing samples, far-cap subdivision and contact split, cap and triangle crossing bounds, and sector cap paths, subdivision, merged angles and root-edge order, and the strict-side offset of a cone meridian (`offset.go`). See loft-point and loft-face-sections designs. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/selectorquery/` | Selector matching, query rendering, residual counts, and face-role indexes. See API §9. |
| `internal/provenance/` | Producer identity, feature-role reference and the `RoleBody`/`RoleCapStart`/`RoleCapEnd` names the topology nodes hold; root `FeatureRef` is the public form. See evaluator §3. |
| `internal/surfacegeom/` | Private face and edge geometry carriers. Public surface and curve variants live in `topology.go`; `Edge.Curve` and `Face.Surface` convert the node's curve and surface to the public variant. See API §6.1, surface §6. |
| `internal/surfaceplace/` | Rigid placement of face surfaces and edge curves, and the placed circle bound. See surface §6. |
| `internal/reportvocab/` | Contact and sweep enums; the contact report over topology nodes (request, reason, feature, point, manifold); the sweep request; Verify reports, motion verification records, survey diagnostics, and pair, interference and sheet outcomes. See contact-geometry §6, contact-sweep §7. |
| `internal/extent/` | Unit bounds and stop levels for extents. See API §8.1, evaluator §5/§6. |
| `internal/patchchain/` | The `PatchProfile` and `PatchEdges` payloads and their builders over nodes; chain partition, plane proof, orientation, segment record, area bounds and Rule P. See surface §5.1-§5.2. |
| `internal/sectionaudit/` | Rewrite audit order, signed area, cutback, crossing, contact and nesting checks. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks, coalescing, junction vertices, rim convexity, and analytic survey and modify loops over recorded segments. |
| `internal/classbgeom/` | Class B: the entry over two payloads (gates B1-B8, through reach, result record), the crossing reach, face geometry and shifted tool records. See general-boolean §3 B, §5. |
| `internal/crossbore/` | Cross-bore mouth chamfer arithmetic: station plan and feet, outer-ring station errors, the dual-interval proof of the quarter-turn base cells and the mirror map from whole-turn cells. See cross-cylinder-bore-chamfer design. |
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
| `internal/sweepmitre/` | The mitred sweep payload and its builder over nodes: profile gates, exact sections, placement, rounding, triangle assembly, Table BM topology and measurements. Root `mitredSweepPayload` aliases it. See sweep §16, evaluator §8. |
| `internal/sweeprecord/` | The sweep and chain sweep records, their span payloads and builders over nodes (`BuildComposite`, `BuildArc`, `BuildClosedCyclic`, `ReplayChainWork`, `Placed`, finishing roles). Root `sweepPayload` and `chainSweepPayload` alias it. See sweep §3, §15, evaluator §8. |
| `internal/coil/` | Coil magnitude, profile side and pitch gates, preflight gates, stations, axis coordinates, moments and closed forms. See helix §4, §5, §7. |
| `internal/coilshell/` | The coil payload and its builder over nodes: the held shell, Table CB topology, Table CM readings, mesh proofs and wall-area enclosures. Root `coilPayload` aliases it. See helix §5, §7, §8, evaluator §8. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames over recorded line and arc spans, and frames along a recorded path. See sweep §3.2. |
| `internal/prismrecord/` | The prism record and its builder over nodes (`Build`, `Placed`, extents, map charge); plane-local lifts, point bounds, lift factor, centroid envelope, placement operand, cap frames, mass record, exact box, section-plane test, exact-section gate. Root `prismPayload` aliases it. See evaluator §5, §8. |
| `internal/momentinput/` | Profile records, exact record comparison, rectangle bounds, axis-aligned rectangle footprints, moments, loop enclosed areas and first moments, walk cache and coordinate envelopes. See evaluator §4 and spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Recorded-loop and band chording, section loop and walk clearance refusals, mesh topology and joined-mesh orientation, bounds and audits. See tessellation-design. |
| `internal/partialband/` | Selected cap-edge fillet band sample layouts and open boundary polylines, and exact partial-chamfer quad gates and mass. See loop-fillet §7.1, modify-general §4.3a. |
| `internal/stationbound/` | Bounds circular chord stations, offset cap stations and chain-sweep arc stations against their exact recorded parameters. |
| `internal/loftrecord/` | The loft and chain loft payloads and their builders over nodes: records, evaluation, placement, flat-triangle topology, certified circular-hole walls and bore proofs, mass readings. Root `loftPayload` and `chainLoftPayload` alias it. See loft §5, §7, §16, §18, evaluator §8. |
| `internal/breprecord/` | The brep and all-edge chamfer payloads and their node builders: prism and stack face views, audit, topology, route L and shell bands, measurements, bevels, placement. Root `brepPayload` and `allEdgeChamferPayload` alias them. See general-boolean §4, modify-general §4, evaluator §8. |
| `internal/capblend/` | The cap-loop chamfer payload and its builder over nodes: setbacks, gates, the mixed offset, band patches, cap edges (also draft and brep walls), departure bounds, mass readings, receiver refusal, admission, witness prisms. Root `capBlendPayload` aliases it. See modify-reach §8.3–§8.4, evaluator §8. |
| `internal/loftmesh/` | Loft and chain pairing gates, stations, assembly, mass sums, projected cap proof, body reading finiteness gate, mirrored-cylinder bridge terms and reserve, cap chamfer source bound, split-ring and collar grids, triangle clearance and middle-collar proofs, and grouped contact scans. |
| `internal/smoothloft/` | Three-section homothety and independent rectangles with exact height curves for Section; varying curves and four-section rounded rectangles. See loft-sections design. |
| `internal/loftfillet/` | Loft fit/circle root-fillet candidate, tangent-root certificate, connector and carrier departures, section contact audits and circle-bore containment. See loft §17–§18. |
| `internal/revolvemesh/` | Revolve rings, cells, caps, cap frames, construction and area proofs, quarter-turn cap-rim blend stations and bounds, and sector-bore cap fans, station steps and projected patches. |
| `internal/revolveplan/` | The revolve and chain revolve records and their builders over nodes (`Build`, `BuildChain`, `Placed`, sweep and extent bounds); revolve mesh walk resolution, coordinate ceilings and chord counts; `ResolveAxisSide`. Root `revolvePayload` aliases it. See evaluator §6, §8, tessellation §8. |
| `internal/revolvesampling/` | Revolve meridian junctions, stations, samples and section readings. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, cell area and volume bounds, and chain-arc source points and area density. See tessellation §8–§11, sweep §15.9. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over recorded `Point2`; indexed triangles and chording refusals. |
| `internal/loftcylinder/` | Exact cap XY audits, source circle and connector gates, ring distances and occupancy, cap chamfer clearance, triangles, band strips, compaction and audits, triangle radii, root radius gaps, and radial and wedge gates of the loft/cylinder Union. See loft-cylinder-boolean design. |
| `internal/topo/` | Node data the root topology handles hold by value (Vertex, Edge, CoEdge, Loop, Face, FaceShell, Lump, Body, Document, Mesh), with their links and the document's minting and liveness; `Payload`, `SimpleProver`, `Env` and the builders' `Alloc`; `topotest` allocates bare nodes. See evaluator §3, §8. |
| `internal/topobuild/` | Topology assembly builders share over nodes: face-loop attachment, open shells, connected lumps, node literals via `Alloc`, body, operand and coedge walkers, blend roles, loft-family helpers, face normals, finiteness gates, mesh facet writes and closed-mesh audit. See evaluator §3, §5, surface §2.2. |
