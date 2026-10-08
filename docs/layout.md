# decad file layout

Every row names what a file owns and where its detail lives. Read "Layout
rows" below before editing one.
`CLAUDE.md` points here from its "Read before you write" table.

## Layout rows

**Layout rows are pointers, not summaries.** A row states what the file owns
in one or two sentences and names where the detail lives, within 300
characters. NEVER grow a row to record an invariant, a derivation, a sign
convention or a refusal — that belongs in the owning design doc or the
function's own doc comment, and a row that restates it drifts from the code.
`claude_md_layout_test.go` enforces these rules over every line of this file
and `CLAUDE.md`, refusing any line it cannot classify. Its `parseAgentDoc`
doc comment owns the complete rule list — heading spellings, the one
`## Layout` heading, declared tables — and its file-level comment owns what
the rules leave to the byte budget.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `docs/api-design.md` | Public API contract for modeling, features, selectors, and verification. |
| `docs/sketch-seam-design.md` | `sketch` recording: `TExact`, `CurveSegment`, and `ErrUnrecordableProfile`. |
| `docs/verification-design.md` | `Verify` reports, statuses, interference costs, deadlines, `WithTolerance`, and noise floor. |
| `docs/payload-verification-design.md` | Per-payload proofs, boundary certificates, bounded validity/clearance/survey algorithms, and tests. |
| `docs/evaluator-design.md` | Evaluator topology, payloads, mass properties, feature builds, and mesh booleans. |
| `docs/tessellation-design.md` | Tessellation: shared curve samples, manifold proofs, source faces, boundary certificates, and boolean handoff. |
| `docs/clearance-design.md` | Pair disjointness and bounded gap proofs. |
| `docs/interference-design.md` | Read-only pair overlap and bounded volume proofs. |
| `docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, and build audit. |
| `docs/spline-design.md` | Free-form kinds, exactness tiers, refusals, Tier A moments, work budget, proven brackets, and reach. |
| `docs/modify-reach-design.md` | Modify reach: tangent chains, asymmetric chamfers, cap-loop blends, shell reach and staging. |
| `docs/brep-modify-design.md` | Modify ops on brep and stacked receivers: prism recognition (route P), axis-parallel edge blends (route E), Tables RB/EB/SB/BB/DB. |
| `docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `docs/sweep-design.md` | `Path`/`Sweep` transport, refusals, topology, measurements, `SweepChain`, mitred sweeps (§16), and reach. |
| `docs/prism-boolean-design.md` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms: entry gate, private `sketch` scene, displacement bounds. |
| `docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `docs/mirror-pattern-design.md` | `Mirrored`/`MirroredCopy`, the exact mirror join, `PatternCopies`/`Patterned` and the prism group. |
| `docs/general-boolean-design.md` | Boolean classes past the prism pair: stacked union, reflected and multi-region operands, perpendicular pairs, `brepPayload`. |
| `docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `docs/faceted-vertex-bounds-design.md` | Per-vertex displacement bounds on faceted bodies and their boolean composition. |
| `docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps: entry gate, private `sketch` scene, and cut bounds. |
| `docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `docs/motion-check-design.md` | `VerifyMotion`: motions, the per-pose proof, the interval certificate, `MotionReport`. |
| `docs/linkage-check-design.md` | `VerifyLinkage` and `VerifyJointBox`: links, joints, drives, boxes, contacts, loops, the chain travel bound. |
| `docs/collision-dynamics-design.md` | Pair contact/sweep in decad and rigid response in `dynamics`. |
| `docs/contact-geometry-design.md` | Pair relation and contact manifold proofs. |
| `docs/contact-sweep-design.md` | Two-body continuous sweep and first-contact brackets. |
| `docs/dynamic-mass-design.md` | Bounded mass and inertia for rigid dynamics. |
| `docs/rigid-dynamics-design.md` | Rigid-body steps, impulses, and reports. |
| `docs/multibody-dynamics-design.md` | N-body dynamics, islands, `Timeline`. |
| `docs/step-export-design.md` | Export package entry points and the AP214 faceted writer contract. |
| `docs/3mf-export-design.md` | The 3MF writer's package parts, mesh mapping, and error contract. |

### User guides

| Path | Responsibility |
|---|---|
| `docs/collision-v1-support.md` | Certified collision paths, response limits, and refusal outcomes. |
| `docs/missing-features.md` | Index of what decad refuses or lacks, each row pointing at its owner. |

### Seam and records

| Path | Responsibility |
|---|---|
| `doc.go` | Package doc: scope, the evaluator support-and-refusal map, and the layering contract (`decad -> sketch -> r3 -> units`). |
| `errors.go` | The core §12 sentinel errors (values from `internal/decaderr/`) and the typed `BooleanError` with its failure `Code`. See `docs/api-design.md` §12, §8. |
| `measurement.go` | Bounded reading types. See `docs/api-design.md` §5.3, §6. |
| `identity.go` | Private document-local producer identities and the shared zero-vector predicate. |
| `record.go` | `ProfileRecord` and public record aliases. See `docs/sketch-seam-design.md` §2. |
| `seam.go` | `RecordProfile`/`RecordChain` adapters. See sketch-seam §1–§2. |
| `path.go` | The immutable spatial `Path` and its sealed `LineTo` / `ArcThrough` segment vocabulary. See `docs/sweep-design.md` §2–§3. |
| `extent.go` | Public extent aliases and normalization. See API §8.1. |
| `selector.go` | Selectors: `EdgeQuery`/`FaceQuery`, predicates and cardinality; a failure is a `SelectionError`. See `docs/api-design.md` §9. |
| `selection_error.go` | `SelectionError` and the shared `*Query.String()` rendering. See `docs/api-design.md` §9. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | Moment readings and record checks. See evaluator §4. |
| `mass_properties.go` / `mass_properties_rotated.go` | Prism mass. See `docs/dynamic-mass-design.md` §2–§3. |
| `mass_properties_sphere.go` / `mass_properties_revolved_cylinder.go` | Sphere and cylinder mass. See `docs/dynamic-mass-design.md`. |
| `mass_properties_revolve.go` | Revolve mass. See `docs/multibody-dynamics-design.md` §8.6. |
| `mass_properties_sweep.go` / `mass_properties_cup.go` | Sweep and cup mass. See `docs/multibody-dynamics-design.md` §8. |
| `mass_properties_faceted.go` / `mass_properties_mesh.go` | Mass read off verified meshes. See `docs/dynamic-mass-design.md`. |
| `moments_circular.go` | Adapts circle and arc records to `internal/circularbounds/`. |
| `spline_bezier.go` | Charges sketch reconstruction and adapts `internal/splinebezier/`. See `docs/spline-design.md` §5.1. |
| `spline_fit.go` | Adapts fit-spline conversion from `internal/splinebezier/`. See `docs/spline-design.md` §5.1.2. |
| `spline_moments.go` | Adapts free-form moments. See spline §5.1. |

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | Topology: `Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`→`Vertex`; aliases for `internal/surfacegeom/`. See `docs/evaluator-design.md` §3. |
| `document.go` | `Document`: live body set, commit, `Remove`, liveness gates, placement and duplication. See evaluator §8. |
| `mirror.go` | The sealed `MirrorPlane` vocabulary and `Mirrored`/`MirroredCopy` over `Placed`/`PlacedCopy`. See `docs/mirror-pattern-design.md` §4. |
| `mirror_join.go` | `WithJoin` adapters and section audit. See mirror-pattern §5. |
| `pattern.go` | Pattern entry points and payload adapters. See mirror-pattern §6. |
| `surface.go` | `WithSurfaceResult`, sheet refusal, shared shell/lump helpers, free-edge chain counts. See surface §2-§4, §7, §11. |
| `patch.go` | Builds a single planar face from a recorded profile. See surface §5.1. |
| `thicken.go` | `Body.Thicken`. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_axis.go` | Assembles thicken sections. See surface §16. |
| `offset.go` | `Body.Offset` builds a second sheet at a stated normal distance, leaving the receiver live. See surface §17. |
| `patch_body.go` | `Body.Patch` topology adapter and face build. See surface §5.2. |
| `denotation.go` | The shared-denotation certificate's two halves: a minted plane identity (LEVEL) and a minted curve/point identity (CURVE). See `docs/surface-design.md` §5.2, §6.2. |
| `stitch_weld.go` | Adapts Stitch topology to Table J. See surface §6.2. |
| `stitch.go` | `Stitch` evaluator and topology adapter. See surface §6.4. |
| `stitch_flux.go` | Stitch curved-face mass adapter. See surface §6.4. |
| `unstitch.go` | `Unstitch` sheet split and placement. See surface §6.5. |
| `extrude.go` | `Document.Extrude`, `WithTaper`, and linear-extent resolution into a `linearSweep`. See evaluator §5. |
| `sweep.go` | `Document.Sweep`/`SweepChain`, path gates, and span payloads. See `docs/sweep-design.md` and `docs/surface-design.md` §4. |
| `sweep_arc.go` | Adapts `internal/sweeparc/` to the one-span `ArcThrough` reduction and Revolve build. See sweep §3. |
| `sweep_composite.go` | Composite Sweep join topology, its boundary audit and a surface result's cap omission. See `docs/sweep-design.md` PR 4 and surface §4. |
| `sweep_composite_measure.go` | Composite Sweep span replay and combined body measurements. See `docs/sweep-design.md` PR 4. |
| `sweep_audit.go` | Composite Sweep adjacent-span and remote-span separation proofs. See `docs/sweep-design.md` §7. |
| `sweep_mitre.go` | Mitred options, entry adapters, payload, placement and restatement. See sweep §16. |
| `sweep_mitre_build.go` | Adapts mitred construction, rounding and SM8 audit. See sweep §16.3–§16.4. |
| `sweep_mitre_body.go` | Builds Table BM's topology and publishes §16.6's readings. See `docs/sweep-design.md` §16.5–§16.6. |
| `sweep_transport.go` | Validates the path and adapts spans to `internal/sweeptransport/`. See sweep §3.2. |
| `prism_payload.go` | `prismPayload`, its coordinate readings and envelopes. See `docs/evaluator-design.md` §5, `docs/prism-boolean-design.md` §7. |
| `prism_build.go` | `evalPrismContext`, caps, and side faces with displacement bounds. See `docs/evaluator-design.md` §5. |
| `segment_walk.go` | Caches profile walks for extrude, revolve and loft. |
| `segment_walk_adapters.go` | Adapts root walk callers to `internal/boundarywalk/`. |
| `prism_extent.go` | Prism extent readings, directional reach and box, each a bounded interval. See `docs/evaluator-design.md` §5. |
| `revolve.go` | `Document.Revolve`: `Axis` variants, options, angular extents. See evaluator §6. |
| `revolve_blend.go` | Fillet/Chamfer of revolve meridian junctions. See modify-reach §7. |
| `revolve_axis.go` | Resolves the axis, classifies walls and checks contact. Uses `internal/revolveaxis/`. See evaluator §6. |
| `revolve_build.go` | Builds a revolve's body, solid or (`WithSurfaceResult`) sheet, and its measurements. See evaluator §6, `docs/surface-design.md` §4. |
| `revolve_extent.go` | Adapts revolve extents over `internal/revolveaxis/` and `internal/revolveangle/`. See evaluator §6. |
| `revolve_denotation.go` | Payload adapters for `internal/revolveangle/` proofs. See evaluator §6 and sweep §3. |
| `stops.go` | Body-relative stop resolution for `ToFace`/`ToFaceAngular`/`ThroughAll`/`ThroughAllSide`. See evaluator §5/§6/§11 and the file's doc comments. |
| `loft.go` | Loft entry points, chain ribbons, and option parsing. See loft §2/§4/§10/§16. |
| `loft_build.go` | Loft payload, evaluation, placement, and the `tessellateLoft` adapter. See `docs/loft-design.md` §5, §8, §12 and `docs/surface-design.md` §4. |
| `loft_pairing.go` | Root adapters for Table P's record gates and station pairs. |
| `loft_stations.go` | Sets loft chord targets and station caps. See `docs/loft-design.md` §5.2. |
| `loft_topology.go` | Adapts loft assembly and builds `Body` topology. See loft §5.1, §7. |
| `loft_moments.go` | Loft mass adapters and chord proofs. See loft §8, §12. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | Fillet section rewrite and build. Owns `cornerBlend` for Chamfer. See modify §6. |
| `chamfer.go` | `Body.Chamfer` over `cornerBlend`; cap loops route to `capblend.go`. See `docs/modify-design.md` §7. |
| `fillet_audit.go` | Fillet, Chamfer and Shell audit orchestration over `internal/sectionaudit/`. See modify §5. |
| `shell.go` | `Body.Shell` offsets a prism into a tube or cup. See modify §8. |
| `shell_offset.go` | Shell section record, audit and displacement proof. See modify §7–§9. |
| `shell_cup.go` | `cupPayload` and `evalCup`: the two-prism body a one-cap `Shell` builds, with its mass properties and roles. See `docs/modify-design.md` §9, §12 D6. |
| `brep_modify.go` / `brep_modify_prism.go` | Brep and stacked modify receivers: dispatch, SB1/SB2, route P. See `docs/brep-modify-design.md`. |
| `brep_modify_edge.go` | Route E: Table EB, restatement, the end-face blends, trims, blend faces and closure. See brep-modify §5. |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Complete-cap-loop chamfer: `capBlendPayload`, selection classification, build gates. See modify-reach §8.3/§4. |
| `capblend_geom.go` | `buildCapBand`: trimmed side walls, cap faces, Plane/Cone band patches. See modify-reach §8.3. |
| `capblend_contour.go` | Bounds cap contour displacement and length through `internal/capcontour/`. See modify-reach §8.3-§8.4. |
| `capblend_centroid.go` | Cap-blend first moments and bounds. See modify-reach §8.4. |
| `capblend_moments.go` | `evalCapBlendContext`: the cap-blend body and its area/volume. See modify-reach §8.4. |
| `capblend_survey.go` | Cap-blend undercut and minimum-radius surveys. See modify-reach Table DX (DX7/DX8). |
| `capblend_normal.go` | Reads band-patch tags and placed frames for DX7's normal model. |
| `capblend_departure.go` | Reads band-patch tags and built edges for departure bounds. |
| `capblend_admit.go` | Adapts cap-band occupied-volume admission. See tessellation-reach §7. |

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | `Document.Verify` orchestration and ordered report assembly. See `docs/verification-design.md` §1–§3 and the file's doc comment. |
| `verify_pairs.go` | `Verify`'s bounded pair workers and ordered outcomes. See `docs/interference-design.md` §2, §5.3 and §7.2. |
| `report.go` | Public aliases and body/pair diagnostics. See verification §1. |
| `verify_tolerance.go` | Adapts `internal/tolerance/` to `Verify` readings and diagnostics. See verification §2-§3. |
| `verify_gate.go` | Verify's payload diameter adapters. See verification §3. |
| `verify_result.go` | Report records and accessors; `verify_publish.go` builds them. |
| `verify_publish.go` | Builds `Verify` reports from private survey results. See `docs/verification-design.md`. |
| `clearance.go` | The pair kernel: `clearancePair` proves one pair's relation and gap; `sheetSolidPair` decides a sheet pair. See `docs/clearance-design.md` §1-§3/§6. |
| `clearance_box.go` | Certifies unplaced axis-aligned box prisms and bounds their gap from exact box planes ahead of the kernel. |
| `clearance_planar.go` | The exact planar pair arm for mitred sweeps and faceted results: interference design §3.2. |
| `contact_pair.go` / `contact_pair_memo.go` | Pair gates, reports, and memo. Box classification lives in `internal/pair/box/`. See `docs/contact-geometry-design.md`. |
| `contact_box.go` | Source-box admission and public face/measurement mapping. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_pair.go` | Planar admission, convexity and the bands. See `docs/multibody-dynamics-design.md` §9–§10. |
| `contact_faceted_manifold.go` / `contact_faceted_patch.go` | Planar manifold faces and witnesses. See multibody §9. |
| `contact_faceted_support.go` | Exact faceted support-face proof and bounded strict separation. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_sweep.go` | Faceted floor sweeps: exact support face, or clearance by swept boxes. See `docs/contact-sweep-design.md`. |
| `contact_oriented_box.go` | Rotated boxes. See `docs/contact-geometry-design.md` §4. |
| `contact_oriented_patch.go` | Publishes oblique box patches from `internal/pair/box/` geometry. See contact geometry §4. |
| `contact_clipped_patch.go` | Publishes horizontal box patches from exact polygon clips. See contact geometry §4. |
| `contact_sphere.go` / `contact_sphere_sweep.go` | Sphere-box contact and sweep. See contact-sweep §4–§5. |
| `contact_cylinder.go` / `contact_cylinder_sweep.go` | Cylinder contact and sweep. See contact-sweep §4–§5. |
| `contact_analytic_manifold.go` | Ruling contact adapters. See contact-geometry §4.5. |
| `contact_sphere_oriented.go` / `contact_sphere_oriented_sweep.go` | Rotated sphere-box path. See the contact designs. |
| `contact_sphere_pair.go` / `contact_sphere_pair_sweep.go` | Sphere-pair contact and sweep. See contact-sweep §4–§5. |
| `contact_sweep_replay.go` | Affine and rotating replay adapters. See contact-sweep §6. |
| `clearance_cells.go` | Pruned cell walk and face-pair adapter. See clearance §3–§5. |
| `clearance_tiers.go` | Tier adapters and vertex budget. See clearance §3/§6. |
| `clearance_geom.go` | `bodyGeom`: each body's clearance faces, edges and nesting test over `internal/clearance/`'s carriers. See `docs/clearance-design.md` §2–§3. |
| `survey.go` | Adapts analytic payloads to wall, undercut and radius readers. See verification §6. |
| `survey_undercut.go` | `listVerdict`, the surveys' per-list fold of `internal/survey2d/`'s three-valued undercut reader. |
| `interference.go` | The pairwise overlap measurement behind `Verify`. See `docs/interference-design.md` §4-§8 and the file's doc comment. |
| `motion.go` / `motion_verify.go` | Public motion aliases, options, reports, and pose checks. See motion-check §2–§6. |
| `motion_bound.go` | Swept-box and corner readings over `internal/motionbound/`. See its doc comment. |
| `linkage.go` / `linkage_verify.go` | `VerifyLinkage`. See linkage design. |
| `linkage_box.go` | `VerifyJointBox`. See `docs/linkage-check-design.md`. |
| `linkage_loop.go` | Closed loops and `Schedule`. See `docs/linkage-check-design.md` §15. |
| `linkage_bound.go` | Linkage reach and projection adapters. See linkage §5.2, §5.8. |
| `contact_sweep.go` | Pair paths, sweeps, and tracks. See `docs/contact-sweep-design.md`. |
| `contact_sweep_rotation.go` / `contact_sweep_faceted.go` | Rotating drift over source boxes or exact planar bodies. See contact-sweep design. |
| `contact_sweep_memo.go` | Adapts `internal/sweepmemo/` to sweep paths and body radius readings. See contact-sweep §7. |
| `contact_sweep_band.go` / `contact_sweep_rolling.go` | Planar and rolling sweep adapters. See multibody-dynamics §10.2–§10.6, §10.8. |
| `swept_box.go` | `Document.SweptBox`: an exact whole-path box. See `docs/multibody-dynamics-design.md` §4.2. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` surface over the mesh-boolean evaluator and the typed `BooleanError` mapping. See `docs/evaluator-design.md` §9. |
| `prism_boolean.go` | Analytic Union/Cut/Intersect of co-directional coplanar or offset-plane prisms. See `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Clean-nesting scene adapters. See prism-boolean §4.2. |
| `prism_boolean_blind.go` | Blind and spanning Cuts via the whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `stacked_prism.go` | Stacked slabs, walls and measurements. See `docs/stacked-prism-design.md`. |
| `prism_group.go` | Prism-group `Cut` tools and disjoint `Union` results. See general-boolean A5. |
| `stacked_union.go` / `stacked_union_brep.go` | A1 `Union`: slabs or a brep. See general-boolean A1. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing resolution. See prism-boolean §4.2. |
| `prism_overlap.go` | Prism-boolean §4.5's overlap-area reading for `Verify`'s interference path. See its doc comment. |
| `brep_payload.go` / `brep_measure.go` | BRep face views, topology and readings. See general-boolean §4. |
| `classb.go` | Class-B admission adapters. See general-boolean §3 B. |
| `classb_crossing.go` / `classb_canonical.go` | Class-B crossing and keyed vertices. See general-boolean §5, §10. |
| `surface_trim.go` | `Trim`/`Extend`/`Split` gates and payload adapters. See surface-intersection §2–§3. |
| `boolean_mesh.go` | `prepBoolMeshContext` prepares an operand's mesh for `internal/meshbool/`'s pipeline. See `docs/evaluator-design.md` §9. |
| `boolean_body.go` | Builds faceted bodies and measurements from an audited mesh; adapts `internal/facetedtopology/`. See evaluator §9. |

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, prism/cup chording and dispatch. See tessellation design. |
| `tessellate_stacked.go` | Meshes stacked slabs with shared chords and volume proof. See `docs/stacked-prism-design.md` §5. |
| `tessellate_brep.go` | Meshes a brep body and proves its occupied volume. See general-boolean §4.4. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and what a mesh publishes about its own proofs. See `docs/tessellation-design.md` §1. |
| `tessellate_revolve.go` | Assembles revolve cells and caps. See tessellation §8–§10. |
| `tessellate_revolve_proof.go` | Wires `internal/revolvemesh/` audits. |
| `tessellate_revolve_arc.go` | Builds circular meridian stations with `internal/revolvemesh/` bounds. |
| `tessellate_revolve_volume.go` | Adapts the revolve volume proof. See tessellation §11. |
| `tessellate_station.go` | `chordStationBound`: the proven enclosure gap of one interior chord station on a circular walk. See the file's doc comment. |
| `tessellate_stitch.go` | Stitch mesh adapters. See tessellation §2 and surface §10.1. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh off its wall topology. `docs/surface-design.md` §13.4. |
| `tessellate_capblend.go` | `tessellateCapBlend`, the cap-loop chamfer mesh over `internal/tessellation/` rings. See `docs/tessellation-reach-design.md` §7. |
| `triangulate.go` | Maps cap points and expected chording errors between `Point2` and `internal/triangulation/`; `cross2` serves root mesh clearance. |
| `export/` | STL, OBJ, and 3MF mesh writers and the analytic/faceted AP214 writer. See `docs/step-export-design.md` and `docs/3mf-export-design.md`. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks). Never `package main`. |
| `dynamics/` | Rigid-body worlds and their scheduled step. See `docs/multibody-dynamics-design.md`. |
| `apitest/` | Tests of the exported API alone. See `apitest/doc.go`. |
| `decadtest/` | The public test kit: comparison helpers over bounded readings, bodies, reports and surveys, plus sketch-to-body fixtures. See `decadtest/doc.go`. |
| `internal/proof/` | Exact dyadic arithmetic, rational intervals and float rounding. |
| `internal/pair/` | Shared contact relation and reading types. |
| `internal/pair/box/` | Exact box paths, contact, oriented and sphere-box proofs, patches, clips and witnesses. |
| `internal/pair/planar/` | Planar solid relations, gaps, patches, support faces, and convexity. |
| `internal/planarsnapshot/` | Exact prism and held-mesh snapshots for planar contact. See multibody §9.1. |
| `internal/placedruling/` | Placed cylinder support proofs. See contact-geometry §4.5. |
| `internal/facetproof/` | Faceted shell audits, placement, restatement and bounds. See `docs/evaluator-design.md` §9. |
| `internal/sectionrecord/` | Curve records and validation. |
| `internal/surfacegeom/` | Sealed face and edge geometry variants. See `docs/api-design.md` §6.1. |
| `internal/reportvocab/` | Verification outcome and diagnostic enums. See verification §1. |
| `internal/extent/` | Sealed linear and angular extent variants. See API §8.1. |
| `internal/patchchain/` | `Body.Patch` chain partition, plane proof, orientation and segment record. See surface §5.2. |
| `internal/sectionaudit/` | Signed area, crossing, contact and nesting checks for rewritten sections. See modify §5. |
| `internal/sketchrecord/` | Sketch snapshot authentication, edge conversion, and join checks. See sketch-seam §2. |
| `internal/splinebezier/` | Exact Bézier conversion of recorded splines. See `docs/spline-design.md` §5.1. |
| `internal/boundarywalk/` | Bounded walks and coalescing over recorded segments. |
| `internal/classbgeom/` | Class-B boxes, gates, through reach, crossing scenes and edge splitting. See general-boolean §3 B, §5. |
| `internal/brepgeom/` / `internal/stackedbrep/` | BRep joins, frames, restatement and stacked records. See general-boolean §4. |
| `internal/revolveaxis/` | Axis input, walks, side/contact gates, snap charges and extent bounds. See evaluator §6. |
| `internal/revolveangle/` | Exact angle denotations and certified sweep extremes. See evaluator §6 and sweep §3. |
| `internal/sweeparc/` | Exact circle carriers, rational vectors, axis lines and angle bounds. See sweep §3. |
| `internal/sweepmitre/` | Mitred profile gates, exact sections and measurements. See sweep §16. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames. See sweep §3.2. |
| `internal/momentinput/` | Validates moment inputs. See spline §5.2. |
| `internal/momentline/` | Computes line moments. See evaluator §4. |
| `internal/momentregion/` | Accumulates record moments. See evaluator §4. |
| `internal/tessellation/` | Chords, prism/cup topology, mesh bounds, cap-blend rings and audits. |
| `internal/loftmesh/` | Loft pairing, stations, assembly, mass sums, mesh proofs and restatement. |
| `internal/revolvemesh/` | Revolve rings, construction and proofs. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, cell area and volume bounds. See tessellation §8–§11. |
| `internal/triangulation/` | Cap hole bridging and ear clipping; indexed triangles, chording refusals. |
| `internal/proofbound/` | Certified bounds, intervals, work budgets and trig. See file comments. |
| `internal/diameter/` | Lower-bound diameter of held witness points. See verification §3. |
| `internal/tolerance/` | Relative tolerance comparisons and body reference formulas. See verification §2-§3. |
| `internal/prismextent/` | Prism extremes and bounds. See evaluator §5. |
| `internal/prismplacement/` | Exact prism axis shifts and relative placement. See prism-boolean §3. |
| `internal/circularbounds/` | Circular endpoints, lengths, area, and moment bounds over neutral records. |
| `internal/prismcells/` | Prism scenes, cells, surface cuts, charges and trim walks. See prism-boolean §4. |
| `internal/mirrorjoin/` | Exact line admission, reflection and record splice. See mirror-pattern §5. |
| `internal/patternrecord/` | Instance motion and record mapping. See mirror-pattern §6.2. |
| `internal/massmoment/` | Rational mass moments and inertia. See dynamic-mass §2–§3. |
| `internal/capcontour/` | Cap contour and shell offset intervals, and cap edge and arc length bounds. See modify-reach §8.3-§8.4. |
| `internal/offset2d/` | Held offset carriers, intersections and section joins. See modify §6–§7. |
| `internal/capband/` | Cap-band admission, departure and moment proofs. See modify-reach §8.3–§8.4. |
| `internal/decaderr/` | The sentinel error values `errors.go` re-exports, so internal packages can return them. |
| `internal/survey2d/` | 2D disk geometry, prism readers, section walks, and Bézier carriers. See verification §6. |
| `internal/thickenaxis/` | Certifies offsets, ribbons and interval clearance. See surface §16. |
| `internal/revolvesurvey/` | Revolve meridian wall and concave-radius readers. See verification §6. |
| `internal/motionbound/` | Motion variants, validation, exact parameters, poses and sweeps. |
| `internal/planarsweep/` | Plane selection, motion, vertex rates, depth and rolling bounds. |
| `internal/sweepdeparture/` | Exact source-box departure proofs. See multibody §10.2. |
| `internal/sweepmemo/` | Sweep memo tables. See contact-sweep §6–§7. |
| `internal/spherepath/` | Sphere path gaps and brackets. See contact-sweep §4–§5. |
| `internal/linkagebound/` | Link reach, layers, spans, projections and loops. See linkage §5, §15. |
| `internal/linkagebound/loopchain/` | Sketch enclosure chains and zero-pose checks. See linkage §15. |
| `internal/polynomial/` | Exact polynomial arithmetic and root brackets. |
| `internal/freeform/` | Free-form curve proofs. |
| `internal/meshbool/` | Mesh boolean contact, subdivision, stitching, embedding, audits. See evaluator §9. |
| `internal/facetedtopology/` | Chains audited mesh face boundaries and selects loops. See evaluator §9. |
| `internal/clearance/` | Clearance geometry, cell sums, coplanar trim classification and degeneracy checks. See clearance design. |
| `internal/clearance/facepair/` | Face-pair cells. See clearance §4. |
| `internal/clearance/curvecells/` | Face-edge and edge-edge cells. See clearance §4. |
| `internal/clearance/tier/` | Vertex cells and ruling proofs. See clearance §3/§6. |
| `internal/clearance/spine/` | Point, line and circle spine cell pairs. See clearance §4. |
| `internal/stitchflux/` | Stitch bounded scalars and flux. See surface §6.4. |
| `internal/stitchweld/` | Stitch welds, topology and bounds. See surface §6.2–6.4. |
| `internal/surfacenormal/` | Exact enclosures and error bounds for analytic face normals. See `normal_bound.go`. |
| `_gallery/` | Nested module for README images, landing and linkage clips, dynamics scenes; excludes SolidLens. See `main.go`. |
| `_shardgen/` | Nested module: packs root and `apitest` tests into cost-balanced race shards; `_` hides it from root tools. See `main.go` doc comment. |
| `.github/workflows/` | `ci.yml`: lint, tests, tidy, vulnerability; `codeql.yml`: CodeQL; `test-shards*.txt`: race shards. |
