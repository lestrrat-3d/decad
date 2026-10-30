# decad file layout: internal packages, checks

Internal packages for records, bounds, surveys, sweeps and clearance.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/chainrecord/` | The chain ribbon record, with denoted sweep levels, and its builder over nodes (`Build`, `Placed`, `Prism` view, walk capture, bounds walks); builds one wall face per recorded segment of every walk and restates the walls as a mesh. Root `chainPayload` aliases it. See surface §13.4, evaluator §8. |
| `internal/draftrecord/` | Draft and two-sided draft records and their builder over nodes (`Build`, `BuildTwoSided`, `FromPrism`, `Extrude`, gates `Receiver`, `NeutralCap`, `KeptWalls`, `Placed`, band view, mesh, measurements, bounded meridians). Root `draftPayload` aliases it. See draft §5–§10.6, evaluator §8. |
| `internal/twistsweep/` | The twisted sweep payloads and their builder over nodes, with polygon admission, stations, Loft cell sewing, wall areas, offset source, cup mesh and the faceted shell body. Root `twistedSweepPayload` and `compositeTwistedSweepPayload` alias it. See sweep §17, evaluator §8. |
| `internal/sectionclip/` | Exact section records for prism, stack slab and Loft (parallel, D-shaped or line-and-arc through-bore, smooth rectangle, homothetic); oblique clips, cap polygons and circle-line trim; cardinal sphere, cone and full Revolve meridian sections; rational plane arithmetic. See section design. |
| `internal/sectionhistory/` | Exact section of a mesh Boolean result as the private-scene 2D operation of its two operands' sections of whole lines, arcs and circles, read in signed axis permutations of the plane, one profile per region with its holes. See section design. |
| `internal/sectioncert/` | Private bounded-section Sketch provenance, guarded Extrude recording, and `RecordProfile`, the Sketch-profile recording seam. See section design, sketch-seam §1–§2. |
| `internal/exactrotation/` | Exactly orthogonal rational rotation recorded at Copy time: scale-safe quaternion construction, exact translation, structural X-axis test, and the reject-only gap against r3's float rotation. See file comments. |
| `internal/proofbound/` | Certified bounds, intervals, work budgets and trig. See file comments. |
| `internal/prooflineage/` | Compact source-volume, centroid, and immutable contained-Cut V/P/Q evidence retained for certified quantity calculations. See operation-history-design and contained-cut-moments-design. |
| `internal/compositemesh/` | Plans shared composite Sweep stations, maps reduced span rings and source roles, and assembles a transient mesh node with its numeric audits. See sweep Table D. |
| `internal/compositesweep/` | Composite Sweep limits, span assembly (cap pairing, rim sewing, cap removal), boundary and vertex-link audits, bounded and body measurements, separation, body-extent and cross-span facet audits, rim-join certificate, side-role and cap-role rewrites. See sweep §7, §9. |
| `internal/diameter/` | Lower-bound diameter of held witness points, including vertex, axis-extreme and prism station witnesses and the free-form prism section arm. See verification §3. |
| `internal/gatediameter/` | Selects non-faceted Verify gate diameter and pair point witnesses over payload records; lower-bound arithmetic stays in `internal/diameter/`. See verification §3. |
| `internal/tolerance/` | Relative tolerance comparisons, body reference formulas, and reading diagnostics. See verification §2-§3. |
| `internal/verifyoption/` | Verify options. See verification §2. |
| `internal/featureoption/` | Profile-fed feature option records and validation. See evaluator §5–§6, sweep §2/§16, loft §2. |
| `internal/modifyoption/` | Fillet, Chamfer and Shell option codecs and side-role segment reading. See modify-reach §2. |
| `internal/tangentchain/` | Ordered tangent-chain walk and analytic continuation decisions over topology nodes. See modify-reach §5. |
| `internal/orderedwork/` | Runs independent jobs and returns results in input order. |
| `internal/hwfault/` | Test guard that skips on Windows the tests that provoke and recover a hardware fault, which corrupts the heap there under Go 1.27. |
| `internal/circularbounds/` | Circular endpoints, lengths, area, moment and arc-box bounds over neutral records, and certified circular bore rings. |
| `internal/surfacetrim/` | Prism-family `Trim`, `Extend` and `Split` pair gates, the S1 family and S7 section-displacement readings, `Extend`'s free-end widening for both families, and revolve `Split`. See surface-intersection §2.1, §3.2. |
| `internal/mirrorjoin/` | Exact line admission, reflection, prism and stacked record rewrites, splice and joined-section audit. See mirror-pattern §5. |
| `internal/patternrecord/` | Pattern spec gates, instance motion, record mapping and the exact plane map with its charged region move. See mirror-pattern §4.3, §6.2 and axial-pattern-join. |
| `internal/patternassembly/` | Pattern frame-keeping instances and disjoint group planning over recorded payloads. See mirror-pattern §4.3, §6. |
| `internal/massmoment/` | Rational moments, inertia, prism composition, translation enclosure, and axis placement charges. See dynamic-mass §2, contained-cut-moments-design, and exact-copy-moments-design. |
| `internal/capcontour/` | Cap contour displacement, shell offset intervals, and edge and arc bounds. See modify-reach §8.3-§8.4. |
| `internal/offset2d/` | Offset carriers, proofs, profile offsets and corner loops, rewrite and offset audits, offset displacement, and the draft near and far section stages. See modify §5–§9, shell-opening §3–§5, draft §2, §7. |
| `internal/capband/` | Cap-band loop walks, offset joins, contour, loop boundary, patch, locus, volume, first-moment and centroid bounds, and cap-loop chamfer refusals. See modify-reach §8.3–§8.4. |
| `internal/filletband/` | Loop-fillet closed forms and reference-frame band mass. See loop-fillet §5. |
| `internal/decaderr/` | The sentinel error values `errors.go` re-exports, so internal packages can return them. |
| `internal/parambind/` | The ledger of parameter reads behind `Document.Bindings`. See param-binding §3. |
| `internal/holeplan/` | Plans `Hole`: places the axis on the face plane, gates sizes, and returns the bore, counterbore and countersink tool shapes. See hole design. |
| `internal/ribplan/` | Plans `Rib`: the rib outline from a line, thickness and overrun, the box-disjoint test and the no-added-volume test. See rib design. |
| `internal/embossplan/` | Plans `Emboss`: the sketch-versus-face plane test, the two tool sides and the no-removed-volume test. See emboss design. |
| `internal/thickenaxis/` | Holds certified section pairs, checks offsets, ribbons and interval clearance, and builds the circle and axis offset sections of a prism sheet. See surface §16, §17. |
| `internal/thickenbuild/` | Dispatches Thicken and Offset by sheet payload, computes its amounts and certified sections, and builds the solids from admitted sheets. See surface §16. |
| `internal/motionbound/` | Motion variants, exact parameters, poses, box bounds, pair certificates, record radii, overlap transfer and sweeps. |
| `internal/motionrun/` | Static mover/stationary pair validity, swept-box exclusion and sheet classification over topology nodes. See motion-check §2–§6. |
| `internal/motionoption/` | Motion options. See motion-check §3. |
| `internal/planarband/` | Planar contact report from the exact relation: manifold witnesses and order, support band, the positive-displacement relation with its charged manifold, and planar solids of prism and faceted payloads. See multibody §9.3–§9.6, §10.4–§10.5. |
| `internal/radiusmemo/` | Body rotational sweep-radius keys and bounded memo readings. See contact-sweep §6–§7. |
| `internal/ringmemo/` | Bounded first-write memo that evicts its oldest key once full; backs the pair report and sweep replay memos. |
| `internal/spherepath/` | Sphere path gaps, face corridors, point transfer, track bounds, brackets and pair replay. See contact-sweep §4–§6. |
| `internal/linkagebound/` | Link bounds and projections. See linkage §5, §15. |
| `internal/loopscene/` | Closed-loop sketch scenes. See linkage §15.2. |
| `internal/linkagebound/loopchain/` | Sketch loop enclosures. See linkage §15. |
| `internal/polynomial/` | Exact polynomial arithmetic and root brackets, and exact Bernstein rows: values, signs, splits, restrictions, products and integer-row chains. |
| `internal/loftface/` | LoftFaces exact cap maps, band gates, root isolation, source cells, vertical-trace box bounds, cone-tangent end map gates, and the glued tangent-cap prism and its clearance gate. See loft-face-sections design. |
| `internal/freeform/` | Free-form curve proofs. |
| `internal/meshbool/` | Pair chord inputs, mesh boolean contact, the tangency gate and its source-pair roles, subdivision, stitching, embedding, exact similarity images, audits, and staging and rim refusals. See evaluator §9. |
| `internal/booloperand/` | Per-operand mesh boolean readings from the topology node: the pair chord tolerance and each operand's share of it, the restating held floor, and the payload-class occupied-volume precheck. See evaluator §9. |
| `internal/booleandispatch/` | The Boolean route order over nodes: LoftFaces, cylinder, sector, cone, prism, stacked, class B and common-axis routes (`Analytic` route prefix), then the mesh pipeline, and the chain regroup executor; and the read-only analytic Intersect. See evaluator §9, prism-boolean §14. |
| `internal/booleaneval/` | The mesh-boolean pipeline: both operand meshes, proximity and spiral-join gates, the exact mesh boolean and stitch, the proof bounds, the faceted payload, the audited volume reading, and `View`, one solid restated alone. See evaluator §9, brep-modify §2.1. |
| `internal/pointquery/` | Exact point-to-mesh distance and nearest point, the body and face distance and closest-point readings, the occupied-volume transfer proof, exact prism and line-only A7 Cut classifiers, and their Boolean-history combination. See point-containment design. |
| `internal/facetedtopology/` | Chains audited mesh face boundaries, proves axial chain directions, selects loops, resolves source-group faces and matches cap patches. See evaluator §9. |
| `internal/clearance/` | Clearance geometry, edge carriers, shell witnesses and cell sums. See clearance design. |
| `internal/clearance/facepair/` | Face-pair cells. See clearance §4. |
| `internal/clearance/curvecells/` | Face-edge and edge-edge cells. See clearance §4. |
| `internal/clearance/tier/` | Vertex cells and ruling proofs. See clearance §3/§6. |
| `internal/clearance/spine/` | Point, line and circle spine cell pairs. See clearance §4. |
| `internal/stitchflux/` | Stitch exact revolve tag admission, face flux and mass proofs. See surface §6.4. |
| `internal/stitchrecord/` | Stitch/Unstitch payloads and node builders, weld/topology/flux proofs, one-face copies, and sibling-revolve source identity for tessellation. Root aliases its payloads. See surface §6, §10 and evaluator §8. |
| `internal/stitchweld/` | Stitch welds, topology and bounds. See surface §6.2–6.4. |
| `internal/denotation/` | Level and curve identity certificates. See surface §5.2, §6.2. |
| `internal/surfacenormal/` | Exact enclosures and error bounds for analytic face normals. See `normal_bound.go`. |
| `internal/edgewedge/` | The material-wedge reading behind `Edge.Wedge` and the `Reflex`/`Salient` predicates, and the selector edge view that carries it. See API §6.1, §9. |
