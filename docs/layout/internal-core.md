# decad file layout: internal packages, core

Internal packages for arithmetic, records, geometry and mesh construction.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/proof/` | Exact dyadic arithmetic, rational intervals and float rounding. |
| `internal/measurement/` | Private bounded reading carriers, exact inflated-box separation, `ExactnessOf` and `FiniteValues`. Public types live in `measurement.go`. See API §5.3, §6. |
| `internal/pair/` | Shared contact relation, reason and reading types; a scalar reading publishes as a measurement. |
| `internal/pair/box/` | Exact box paths and oriented-box translation, contact, oriented, sphere-box and cylinder-box proofs, interval separation, patches, clips, and witness and normal readings with their measurements. |
| `internal/pair/sphere/` | Exact sphere-pair relation, gap and bounded response witness. See contact-geometry §4.3. |
| `internal/pair/planar/` | Planar solid relations, gaps, patches, support faces, and convexity. |
| `internal/planarsnapshot/` | Planar contact snapshots, cached records and pose gates. See multibody §9.1–§9.2. |
| `internal/placedruling/` | Placed cylinder staging, support and relation proofs. See contact-geometry §4.5. |
| `internal/facetproof/` | Faceted shell audits, placement, similarity and revolve sector-separation proofs, sector seams and angular gaps, and the faceted centroid allowance. See `docs/evaluator-design.md` §9. |
| `internal/facetedrecord/` | Faceted payloads, point-loft source authentication, slab and cone-trim builders, bounds, caps, centroids, similarity proofs, placement, and node building. Root aliases its records. See evaluator §8–§9 and loft-point design. |
| `internal/pointcone/` | Point-loft cone crossing samples, far-cap subdivision and contact split, cap and triangle crossing bounds, and sector cap paths, subdivision, merged angles and root-edge order, and the strict-side offset of a cone meridian (`offset.go`). See loft-point and loft-face-sections designs. |
| `internal/pointloft/` | Builds the point-section Loft apex fan and proves its source stations, boundary, area and volume. See loft-point design. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/selectorquery/` | Selector matching, query rendering, residual counts, and face-role indexes. See API §9. |
| `internal/provenance/` | Producer identity, feature-role reference and the `RoleBody`/`RoleCapStart`/`RoleCapEnd` names the topology nodes hold; root `FeatureRef` is the public form. See evaluator §3. |
| `internal/surfacegeom/` | Private face and edge geometry carriers and Verify's diagnostic plane key. Public variants live in `topology.go`; `Edge.Curve` and `Face.Surface` adapt the nodes. See API §6.1, surface §6. |
| `internal/surfaceplace/` | Rigid placement of face surfaces and edge curves, and the placed circle bound. See surface §6. |
| `internal/reportvocab/` | Contact and sweep enums; contact reports, pair-reason mapping and planar feature mapping over topology nodes; the sweep request; Verify reports, boundary audits, motion records, survey diagnostics, and pair, interference and sheet outcomes. See contact-geometry §6, contact-sweep §7. |
| `internal/extent/` | Unit bounds and stop levels for extents. See API §8.1, evaluator §5/§6. |
| `internal/patchchain/` | Patch payloads and node builders; chain partition, exact and shared-level plane proofs, orientation, segment records, area bounds, Rule P and circular-prism admission. See surface §5.1-§5.2 and §10. |
| `internal/patchmesh/` | Builds PatchProfile and PatchEdges sheet mesh nodes with face and area bounds. See surface §10. |
| `internal/sectionaudit/` | Rewrite audit order, signed area, cutback, crossing, contact, nesting and detailed error rendering. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks, coalescing, junction vertices, rim convexity, and analytic survey and modify loops over recorded segments. |
| `internal/classbgeom/` | Class B: the entry over two payloads (gates B1-B8, through reach, result record), the crossing reach, face geometry and shifted tool records. See general-boolean §3 B, §5. |
| `internal/crossbore/` | Builds the held bore-mouth mesh, proves its cells and computes boundary, volume and area bounds from source records. See cross-cylinder-bore-chamfer design. |
| `internal/brepgeom/` / `internal/stackedbrep/` | BRep geometry, topology and shared endpoint matching. See general-boolean §4–§5. |
| `internal/brepwallmesh/` | Chords BRep swept walls and checks clearance between their mesh loops. See general-boolean §4.4 and tessellation §5. |
| `internal/brepbandmesh/` | Chords route L bands and the shell bands, imposes their counts on the walls beside them, places their rings, emits their strips, and states their motion, volume terms and occupied-volume admission. See modify-general DG3/DG4, §4.3a. |
| `internal/brepfilletmesh/` | Places full and selected BRep fillet rings, emits patch triangles, and computes their area and motion bounds. See loop-fillet §7.1. |
| `internal/stackedunion/` | Builds A1 stacked Union slabs, private scenes, and BRep faces after root admission. See general-boolean §3 A1. |
| `internal/throughshell/` | Through-cut record recognition, section and cavity construction, strips, rim assembly, and audits. See modify-general §3. |
| `internal/stackedshell/` | Admits and records the exact stacked shells (nested rectangular boss, circular boss, rectangular blind pocket, coaxial counterbore) from a two-slab record and its face view; `breprecord` builds the result. See modify-general §3.1a-§3.1c, §3.1e. |
| `internal/capedge/` | Convex cap-edge cutter admission, quarter-cylinder profile, chamfer wedge and the P8 top front edge test. See vertex-blend §2. |
| `internal/prismshell/` | Prism cap-shell gates with tube, band and closed-shell records; side sections, wall bands, nested boss mesh and proof, closed stack reversal, slab levels and stack faces. See modify §8, shell-opening §3–§5, shell-faceted-boss design. |
| `internal/prismrecord/` | The prism record and its builder over nodes (`Build`, `Placed`, extents, map charge); plane-local lifts, point bounds, lift factor, centroid envelope, placement operand, cap frames, mass record, exact box, section-plane test, exact-section gate. Root `prismPayload` aliases it. See evaluator §5, §8. |
| `internal/gatewitness/` | Selects payload witness prisms and their displacement for Verify's fallback diameter; root reads their stations. See verification §3. |
| `internal/prismtess/` | Builds a prism payload's chorded mesh node and its face, area and occupied-volume bounds. See tessellation §5. |
| `internal/cuptess/` | Builds a cup payload's chorded mesh node and its face, area and occupied-volume bounds. See tessellation §6. |
| `internal/stitchtess/` | Restates Stitch's held planar triangles and admitted Revolve-backed curved sheets as mesh nodes, and yields a stitched body's vertex bounds. See tessellation §2 and surface §10. |
| `internal/momentinput/` | Profile records, exact comparison, area falsifiers, rectangle bounds, moments, walk cache and coordinate envelopes. See evaluator §1/§4 and spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Recorded-loop and band chording, cap-band motion and slab volume, BRep wall strip triangulation, section loop and walk clearance refusals, mesh topology and joined-mesh orientation, chain-sweep proof readings, bounds and audits. See tessellation-design. |
| `internal/partialband/` | Selected cap-edge fillet band sample layouts and open boundary polylines, and exact partial-chamfer quad gates and mass. See loop-fillet §7.1, modify-general §4.3a. |
| `internal/stationbound/` | Bounds circular chord stations, offset cap stations and chain-sweep arc stations against their exact recorded parameters. |
| `internal/loftrecord/` | The loft and chain loft payloads and their builders over nodes: records, evaluation, placement, flat-triangle topology, certified circular-hole walls and bore proofs, mass readings. Root `loftPayload` and `chainLoftPayload` alias it. See loft §5, §7, §16, §18, evaluator §8. |
| `internal/breprecord/` | BRep records, route E gates and rewrite, reference-face identity, all-edge chamfer records and node builders; root payloads alias them. See general-boolean §4, brep-modify §5, modify-general §4, evaluator §8. |
| `internal/alledgechamfermesh/` | Builds the all-edge Chamfer mesh and its face, area and occupied-volume bounds. See modify-general §4.3b and tessellation §2. |
| `internal/breploop/` | Route L/V selected-edge matching, loop decisions, asymmetric assignments, record topology, reach, contours and rewrite over one budget, and the partial-loop fillet and chamfer rewrites. Root binds public selectors and face references. See modify-general §4 and §4.3a. |
| `internal/capblend/` | Chamfer payload/build: setbacks, draft bounds, offsets, patches, cap edges, departure/mass bounds, refusal, admission, witness prisms, mesh-loop chording/motion, the shared band emitter and audited mesh assembly. Root `capBlendPayload` alias. See modify-reach §8, tessellation-reach §7. |
| `internal/loftmesh/` | Loft and chain mesh assembly, point-plane and LoftFaces cap layouts and proofs, stations, bounds, split and collar grids, triangle clearance, and contact scans. See loft-point and tessellation designs. |
| `internal/smoothloft/` | Three-section homothety, constant curved-section admission and independent rectangles with exact height curves for Section; varying curves and four-section rounded rectangles. See loft-sections design. |
| `internal/loftfillet/` | Loft fit/circle root-fillet candidate, tangent-root certificate, connector and carrier departures, section contact audits and circle-bore containment. See loft §17–§18. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over recorded `Point2`; indexed triangles and chording refusals. |
| `internal/loftcylinder/` | Cap and source-circle audits, ring occupancy, mirrored blank, pair placements, Join and bore seams, upper tooth shifts, cap Chamfer admission and proved builds, and the circular Union's tooth admission, separation, joined shell and bounds. See loft-cylinder-boolean and loft-mirror-boolean designs. |
| `internal/loftcontact/` | Varying Loft/prism carrier, hull, bounded exact arithmetic, source partition and paired cap certificates, the output shell with its exact caps, side walls, source audit, embedding and volume, quarter-Loft held seams and pair separation, and cap, interval and wall records. See general-boolean. |
| `internal/topo/` | Node data held by value in root topology handles (Vertex, Edge, CoEdge, Loop, Face, FaceShell, Lump, Body, Document, Mesh), links, document minting/liveness, and Body's sweep-radius memo; `Payload`, `SimpleProver`, `Env`, builders' `Alloc`; `topotest` allocates bare nodes. See evaluator §3, §8. |
| `internal/topobuild/` | Topology assembly over nodes: faces, edges, shells, lumps, loft helpers, normals, mesh writes, and solid and sheet audits. See evaluator §3, §5 and surface §2.2. |
