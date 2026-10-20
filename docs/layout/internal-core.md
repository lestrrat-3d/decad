# decad file layout: internal packages, core

Internal packages for arithmetic, records, geometry and mesh construction.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/payloadclass/` | The hook through which `decadtest.PayloadClass` reads a body's payload class. See faceted-source-retention §10. |
| `internal/proof/` | Exact dyadic arithmetic, rational intervals and float rounding. |
| `internal/measurement/` | Private bounded reading carriers, exact inflated-box separation, `ExactnessOf` and `FiniteValues`. Public types live in `measurement.go`. See API §5.3, §6. |
| `internal/pair/` | Shared contact relation, reason and reading types; a scalar reading publishes as a measurement. |
| `internal/pair/box/` | Exact box paths and oriented-box translation, contact, oriented, sphere-box and cylinder-box proofs, interval separation, patches, clips, and witness and normal readings with their measurements. |
| `internal/contactbox/` | Source-box and oriented-box admission over body nodes, with their exact classification and the face, point and normal manifolds they publish into a contact report: axis, contained, clipped and oblique patches. See contact-geometry §4. |
| `internal/contactround/` | Source sphere and cylinder admission over body nodes and their contact classification against boxes, spheres and oriented boxes. See contact-sweep §4–§5. |
| `internal/planarpose/` | Planar admission of a body at a query pose: snapshots by payload, the held-mesh arm through a mesher, the convexity cache and the exact pair classification published through `internal/planarband/`. See multibody §9–§10. |
| `internal/contactruling/` | Placed ruling contact of a source cylinder against an exact planar body: side faces, the ruling set, normals and the manifolds published into a contact report. See contact-geometry §4.5. |
| `internal/contactfaceted/` | Exact faceted support-face proof, bounded strict separation from a source-box floor, and the floor contact classification. See contact-geometry §4. |
| `internal/pair/sphere/` | Exact sphere-pair relation, gap and bounded response witness. See contact-geometry §4.3. |
| `internal/pair/planar/` | Planar solid relations, gaps, patches, support faces, and convexity. |
| `internal/planarsnapshot/` | Planar contact snapshots, cached records, pose gates and placement. See multibody §9.1–§9.2. |
| `internal/placedruling/` | Placed cylinder staging, support and relation proofs. See contact-geometry §4.5. |
| `internal/facetproof/` | Faceted shell audits, placement, similarity and revolve sector-separation proofs, sector seams and angular gaps, and the faceted centroid allowance. See `docs/evaluator-design.md` §9. |
| `internal/facetedrecord/` | Faceted payloads, point-loft source authentication, slab and cone-trim builders, bounds, caps, centroids, similarity proofs, placement, node building, mesh restatement and copy separation proofs. Root aliases its records. See evaluator §8–§9, loft-point design and faceted-vertex-bounds §3.7. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/selectorquery/` | Selector matching on node views, query rendering, residual counts, face-role indexes. See API §9. |
| `internal/modifyselect/` | Modify's selected-edge context and corner match. See modify §6. |
| `internal/provenance/` | Producer identity, feature-role reference and the `RoleBody`/`RoleCapStart`/`RoleCapEnd` names the topology nodes hold; root `FeatureRef` is the public form. See evaluator §3. |
| `internal/surfacegeom/` | Private face and edge geometry carriers and Verify's diagnostic plane key. Public variants live in `topology.go`; `Edge.Curve` and `Face.Surface` adapt the nodes. See API §6.1, surface §6. |
| `internal/surfaceplace/` | Rigid placement of face surfaces and edge curves, and the placed circle bound. See surface §6. |
| `internal/reportvocab/` | Contact and sweep enums; contact reports, pair-reason mapping and planar feature mapping over topology nodes; the sweep request; Verify reports, boundary audits, motion records, survey diagnostics, and pair, interference and sheet outcomes. See contact-geometry §6, contact-sweep §7. |
| `internal/extent/` | Unit bounds, `ToFace`, `ToFaceAngular` and through-all stops for extents. See API §8.1, evaluator §5/§6. |
| `internal/patchchain/` | Patch payloads and node builders; `ChainsFromEdges`, chain partition, exact and shared-level plane proofs, orientation, segment records, area bounds, Rule P and circular-prism admission. See surface §5.1-§5.2 and §10. |
| `internal/patchmesh/` | Builds PatchProfile and PatchEdges sheet mesh nodes with face and area bounds. See surface §10. |
| `internal/sectionaudit/` | Rewrite audit order, signed area, cutback, crossing, contact, nesting and detailed error rendering. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks, coalescing, junction vertices, rim convexity, and analytic survey and modify loops over recorded segments. |
| `internal/classbgeom/` | Class B: the entry over two payloads (gates B1-B8, through reach, result record), the crossing reach, face geometry and shifted tool records. See general-boolean §3 B, §5. |
| `internal/crossbore/` | Recognizes the perpendicular cylinder Cut, admits the bore-mouth chamfer, builds the held mesh, proves its cells and computes boundary, volume and area bounds. See cross-cylinder-bore-chamfer design. |
| `internal/brepgeom/` / `internal/stackedbrep/` | BRep geometry, topology and shared endpoint matching. See general-boolean §4–§5. |
| `internal/brepwallmesh/` | Chords BRep swept walls and checks clearance between their mesh loops. See general-boolean §4.4 and tessellation §5. |
| `internal/brepbandmesh/` | Meshes a brep body: walls, planar faces, closure audits and the occupied-volume sum. Chords route L and shell bands, imposes their counts on the walls beside them, places and emits them, and states their motion, volume terms and admission. See general-boolean §4.4, modify-general DG3/DG4, §4.3a. |
| `internal/brepfilletmesh/` | Places full and selected BRep fillet rings, emits patch triangles, and computes their area and motion bounds. See loop-fillet §7.1. |
| `internal/throughshell/` | Through-cut record recognition, section and cavity construction, strips, rim assembly, audits, and the shell route over them. See modify-general §3. |
| `internal/capedge/` | Convex cap-edge cutter admission, quarter-cylinder profile, chamfer wedge and the P8 top front edge test. See vertex-blend §2. |
| `internal/gatewitness/` | Selects payload witness prisms and their displacement for Verify's fallback diameter; root reads their stations. See verification §3. |
| `internal/cuptess/` | Builds a cup payload's chorded mesh node and its face, area and occupied-volume bounds. See tessellation §6. |
| `internal/stitchtess/` | Restates Stitch's held planar triangles and admitted Revolve-backed curved sheets as mesh nodes, and yields a stitched body's vertex bounds. See tessellation §2 and surface §10. |
| `internal/momentinput/` | Profile records, exact comparison, area falsifiers, rectangle bounds, moments, walk cache and coordinate envelopes. See evaluator §1/§4 and spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Recorded-loop and band chording, cap-band motion and slab volume, BRep wall strip triangulation, section loop and walk clearance refusals, mesh topology and joined-mesh orientation, chain-sweep proof readings, bounds and audits. See tessellation-design. |
| `internal/tessaudit/` | Names the payload classes whose tessellation runs a facet-contact audit that a lower verification level may decline. See tessellation-design. |
| `internal/verifiedmesh/` | Names the one tessellation internal builders borrow from the root: a body's mesh at VerifyAll. See tessellation-design. |
| `internal/partialband/` | Selected cap-edge fillet band sample layouts and open boundary polylines, and exact partial-chamfer quad gates and mass. See loop-fillet §7.1, modify-general §4.3a. |
| `internal/stationbound/` | Bounds circular chord stations, offset cap stations and chain-sweep arc stations against their exact recorded parameters. |
| `internal/breprecord/` | BRep records, route E gates and rewrite, reference-face identity, all-edge chamfer records, their admission and node builders; root payloads alias them. See general-boolean §4, brep-modify §5, modify-general §4, evaluator §8. |
| `internal/alledgechamfermesh/` | Builds the all-edge Chamfer mesh and its face, area and occupied-volume bounds. See modify-general §4.3b and tessellation §2. |
| `internal/breploop/` | Route L/V selected-edge matching, loop decisions, asymmetric assignments, record topology, reach, contours and rewrite over one budget, and the partial-loop fillet and chamfer rewrites. Root binds public selectors and face references. See modify-general §4 and §4.3a. |
| `internal/capblend/` | Chamfer payload/build: setbacks, draft bounds, offsets, patches, cap edges, departure/mass bounds, refusal, admission, witness prisms, mesh-loop chording/motion, the shared band emitter and audited mesh assembly. Root `capBlendPayload` alias. See modify-reach §8, tessellation-reach §7. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over recorded `Point2`; indexed triangles and chording refusals. |
| `internal/topo/` | Node data held by value in root topology handles (Vertex, Edge, CoEdge, Loop, Face, FaceShell, Lump, Body, Document, Mesh), links, document minting/liveness, and Body's sweep-radius memo; `Payload`, `SimpleProver`, `Env`, builders' `Alloc`; `topotest` allocates bare nodes. See evaluator §3, §8. |
| `internal/topobuild/` | Topology assembly over nodes: faces and their roles, edges, shells, lumps, loft helpers, normals, mesh writes and vertex bounds, and solid and sheet audits. See evaluator §3, §5 and surface §2.2. |
