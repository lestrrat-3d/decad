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
| `docs/modify-reach-design.md` | The modify extension: tangent-chain expansion, asymmetric chamfers, cap-loop blends, allowed shells, proof gates, payloads and staging. |
| `docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `docs/sweep-design.md` | `Path`/`Sweep` transport, refusals, topology, measurements, `SweepChain`, mitred sweeps (§16), and reach. |
| `docs/prism-boolean-design.md` | The analytic `Union`/`Cut`/`Intersect` reduction over co-directional coplanar or offset-plane prisms: reject-only entry gate, private `sketch` scene, displacement bounds. |
| `docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `docs/faceted-vertex-bounds-design.md` | Per-vertex displacement bounds on faceted bodies: the bound model, boolean composition, readings, the chain-depth gate and the PR plan. |
| `docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps: entry gate, private `sketch` scene, and cut bounds. |
| `docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `docs/motion-check-design.md` | `Document.VerifyMotion`: the `Motion` set, the per-pose pair proof, the interval certificate, and `MotionReport`. |
| `docs/linkage-check-design.md` | `Document.VerifyLinkage`: links, joints, drives, joint contacts, the chain travel bound. |
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

### Seam and records

| Path | Responsibility |
|---|---|
| `doc.go` | Package doc: scope, the evaluator support-and-refusal map, and the layering contract (`decad -> sketch -> r3 -> units`). |
| `errors.go` | The core §12 sentinel errors (values from `internal/decaderr/`) and the typed `BooleanError`, whose `Code` classifies failures wrapping `ErrBooleanFailed` or `ErrUnsupported`. See `docs/api-design.md` §12, §8. |
| `measurement.go` | The bounded-result shapes: `Exactness`, `Measurement`, `VecMeasurement`, `Box`. See `docs/api-design.md` §5.3, §6. |
| `identity.go` | Private document-local producer identities and the shared zero-vector predicate. |
| `record.go` | Sketch profile, chain, loop and curve records; NURBS validation. See `docs/sketch-seam-design.md` §2. |
| `seam.go` | `RecordProfile`/`RecordChain`, `TExact` admission and record checks. See `docs/sketch-seam-design.md` §1, §7. |
| `path.go` | The immutable spatial `Path` and its sealed `LineTo` / `ArcThrough` segment vocabulary. See `docs/sweep-design.md` §2–§3. |
| `extent.go` | Linear and angular extent types; `ToFace`/`ToFaceAngular` references. See `docs/api-design.md` §8.1. |
| `selector.go` | Selectors: `EdgeQuery`/`FaceQuery`, predicate conjunction and `Exactly`/`AtLeast` cardinality over live topology; a failure returns a `SelectionError`. See `docs/api-design.md` §9. |
| `selection_error.go` | `SelectionError` (wraps `ErrNoMatch`/`ErrCardinality`) and the canonical `*Query.String()` rendering it and a verification `Diagnostic` both reuse. See `docs/api-design.md` §9. |
| `codec_error.go` | Internal path-aware validation errors for structural curve records. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | The mass-property engine (evaluator §4): closed-form Green's-theorem boundary integrals for `Area`, `Centroid`, `SecondMoments`, per region. See `docs/spline-design.md` §5.2. |
| `mass_properties.go` / `mass_properties_rotated.go` | Prism mass. See `docs/dynamic-mass-design.md`. |
| `mass_properties_sphere.go` / `mass_properties_revolved_cylinder.go` | Sphere and cylinder mass. See `docs/dynamic-mass-design.md`. |
| `mass_properties_revolve.go` | Revolve mass. See `docs/multibody-dynamics-design.md` §8.6. |
| `mass_properties_sweep.go` / `mass_properties_cup.go` | Sweep and cup mass. See `docs/multibody-dynamics-design.md` §8. |
| `mass_properties_faceted.go` / `mass_properties_mesh.go` | Mass read off verified meshes. See `docs/dynamic-mass-design.md`. |
| `moments_circular.go` | Maps recorded circle and arc segments to `internal/circularmoments/` for exact rational enclosures. |
| `spline_bezier.go` | Builds a recorded free-form curve's exact Bézier spans and reconstruction over `internal/freeform/`'s reduction and work budget. See `docs/spline-design.md` §5.1. |
| `spline_fit.go` | Converts a recorded `FitSplineSeg` into Bézier spans over `internal/freeform/`'s fit reduction. See `docs/spline-design.md` §5.1.2. |
| `spline_moments.go` | `addFreeformTo` folds `internal/freeform/`'s exact span moments into a region's integrals. See `docs/spline-design.md` §5.1. |

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | The topology model: `Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`→`Vertex`, plus sealed `Surface`/`Curve` variant sets. See the types' own doc comments and `docs/evaluator-design.md` §3. |
| `normal_bound.go` | The proof behind every `Face.NormalAt` bound, per surface arm. See the file's doc comment. |
| `document.go` | `Document`, its guarded live body set, commit, `Remove`, identity and liveness gates; body placement and duplication. See its doc comments and evaluator §8. |
| `surface.go` | `WithSurfaceResult`, sheet refusal, and shared shell/lump helpers. `freeChainCountsByFace` counts a sheet's free-edge chains. See surface §2-§4, §7, §11. |
| `patch.go` | Builds a single planar face from a recorded profile. See surface §5.1. |
| `thicken.go` | `Body.Thicken` grows an admitted sheet into a solid. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_axis.go` | Certifies exact axis-parallel offsets and interval separation. See surface §16.2. |
| `offset.go` | `Body.Offset` builds a second sheet at a stated normal distance, leaving the receiver live. See surface §17. |
| `patch_body.go` | `Body.Patch`: splits a free-edge selection into closed chains, proves each planar and fills it with a face. See `docs/surface-design.md` §5.2. |
| `denotation.go` | The shared-denotation certificate's two halves: a minted plane identity (LEVEL) and a minted curve/point identity (CURVE). See `docs/surface-design.md` §5.2, §6.2. |
| `stitch_weld.go` | Table J's admission: the shared vertex table and which free edge pairs weld. See `docs/surface-design.md` §6.2. |
| `stitch.go` | `Stitch`: rebuilds fresh topology over the weld plan, derives orientation, and decides Table C's outcome and measurements. See `docs/surface-design.md` §6. |
| `stitch_flux.go` | The per-surface flux integral for a curved closed boundary: Rule S, the hoisted vertex-link audit call, and the `Plane`/`Cylinder` volume and centroid arms. See `docs/surface-design.md` §6.4. |
| `unstitch.go` | `Unstitch`: splits a body into one free single-face sheet per face, reusing `stitch.go`'s placement machinery per face. See `docs/surface-design.md` §6.5. |
| `extrude.go` | `Document.Extrude`: the public entry point, `WithTaper`, and linear-extent resolution into a `linearSweep`. See `docs/evaluator-design.md` §5 and the file's doc comment. |
| `sweep.go` | `Document.Sweep`/`SweepChain`, path gates, and span payloads. See `docs/sweep-design.md` and `docs/surface-design.md` §4. |
| `sweep_arc.go` | The one-span `ArcThrough` reduction: exact circumcircle and tangent gates, bounded axis/angle publication and Revolve reuse. See `docs/sweep-design.md` PR 3. |
| `sweep_composite.go` | Composite Sweep shared join topology, its manifold-with-boundary audit, and the outer-cap omission a surface result takes. See `docs/sweep-design.md` PR 4, `docs/surface-design.md` §4. |
| `sweep_composite_measure.go` | Composite Sweep span replay and combined body measurements. See `docs/sweep-design.md` PR 4. |
| `sweep_audit.go` | Composite Sweep adjacent-span and remote-span separation proofs. See `docs/sweep-design.md` §7. |
| `sweep_mitre.go` | `WithMitredJoins`/`WithSectionScale`, Table SM's entry gates, the mitred payload, placement and restatement. See `docs/sweep-design.md` §16. |
| `sweep_mitre_build.go` | §16.3's exact mitred construction, rounding, orientation and SM8 audit. See `docs/sweep-design.md` §16.3–§16.4. |
| `sweep_mitre_body.go` | Table BM's triangles and topology, and §16.6's readings. See `docs/sweep-design.md` §16.5–§16.6. |
| `sweep_transport.go` | Rotation-minimizing endpoint-frame transport over exact path records, with rational enclosures of each held frame. See `docs/sweep-design.md` §3.2. |
| `prism_payload.go` | `prismPayload` and its coordinate readings: a world point, its proven bound, and the coordinate envelopes later bounds charge against. See `docs/evaluator-design.md` §5, `docs/prism-boolean-design.md` §7. |
| `prism_build.go` | `evalPrismContext`, caps, and side faces with displacement bounds. See `docs/evaluator-design.md` §5. |
| `segment_walk.go` | Builds the profile-boundary walks extrude, revolve and loft read; a kind with no stated bound refuses. See the file's doc comment. |
| `prism_extent.go` | A finished prism's extent readings, reach along a direction and the containing box, each a bounded interval charging the frame, section and axial terms. See `docs/evaluator-design.md` §5. |
| `revolve.go` | `Document.Revolve` (evaluator §6): the sealed `Axis` vocabulary, `EdgeAxis` and `WithSurfaceResult` parsing, angular-extent resolution. Axis, build and extent readings: the other `revolve_*.go` files. |
| `revolve_axis.go` | Resolves the axis into the sketch plane and decides what the profile may do around it: `axisLine2`, `axisFrame`, `wallKind`, and the contact gates. See `docs/evaluator-design.md` §6. |
| `revolve_build.go` | Builds a revolve's body, solid or (`WithSurfaceResult`) sheet, and its measurements. See evaluator §6, `docs/surface-design.md` §4. |
| `revolve_extent.go` | A finished revolve's extent readings: each extreme is a swept extreme, bracketed by `sweepExtremeBounds` rather than read off a boundary vertex. See `docs/evaluator-design.md` §6. |
| `revolve_denotation.go` | `angleDenotation`/`sweepDenotation`: exact or certified-interval angles and their sweep/trig bounds. See `docs/evaluator-design.md` §6, `docs/sweep-design.md` §3. |
| `stops.go` | Body-relative stop resolution for `ToFace`/`ToFaceAngular`/`ThroughAll`/`ThroughAllSide`. See evaluator §5/§6/§11 and the file's doc comments. |
| `loft.go` | `Document.Loft` and `LoftChain`: the entry points over `loft_build.go`'s evaluator, the chain ribbon build, and `WithSurfaceResult` parsing. See `docs/loft-design.md` §2/§4/§10/§16. |
| `loft_build.go` | Loft payload, evaluation, placement, and the `tessellateLoft` adapter. See `docs/loft-design.md` §5, §8, §12 and `docs/surface-design.md` §4. |
| `loft_pairing.go` | `docs/loft-design.md` Table P: which from-segment walls to which to-segment. A pair the table does not decide is refused, never matched to the nearest. See §5, §5.1. |
| `loft_stations.go` | Places the stations a loft's wall chords run between and proves each chain's departure from its curve, under one shared chord target and a station cap. See `docs/loft-design.md` §5.2. |
| `loft_topology.go` | Assembles the paired stations into the flat-triangle solid the payload holds, and builds the `Body` topology over it. See `docs/loft-design.md` §5.1, §7 and the file's doc comment. |
| `loft_moments.go` | `docs/loft-design.md` §8's mass-property engine: `loftMassAccumulator`, an exact-rational tetrahedron sum publishing Volume/Centroid/Bounds/Area. See §8, §12. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Body.Fillet` rewrites a straight prism's section with a tangent arc at each selected corner and rebuilds through `evalPrism`. It owns the `cornerBlend` Chamfer reuses. See `docs/modify-design.md` §6. |
| `chamfer.go` | `Body.Chamfer` bevels a straight prism's lateral corners with a chord between setback feet, sharing `cornerBlend` with `fillet.go`; a cap-loop selection routes to `capblend.go`. See `docs/modify-design.md` §7. |
| `fillet_audit.go` | Fillet, Chamfer and Shell section audits. See `docs/modify-design.md` §5. |
| `shell.go` | `Body.Shell` offsets a prism into a tube or cup. See modify §8. |
| `shell_offset.go` | The exact per-feature section offset (`P ⊖ t` / `P ⊕ t`) behind `Shell`, the §5 audit wrapper run on it, and a cup's offset displacement proof. See `docs/modify-design.md` §7-§9. |
| `shell_cup.go` | `cupPayload` and `evalCup`: the two-co-directional-prism body a one-cap `Shell` builds, with Exact mass properties and roles. See `docs/modify-design.md` §9; clearance stays staged (§12 D6). |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Builds the complete-cap-loop chamfer: `capBlendPayload` plus the selection classification and build gates in `buildCapBlend`. See `docs/modify-reach-design.md` §8.3/§4. |
| `capblend_geom.go` | Builds the `capBlendPayload` topology in `buildCapBand`: trimmed side walls, cap faces, and Plane/Cone band patches. See `docs/modify-reach-design.md` §8.3. |
| `capblend_contour.go` | Maps cap-blend joins and shell offsets to `internal/capcontour/`'s interval bounds. See `docs/modify-reach-design.md` §8.3-§8.4. |
| `capblend_centroid.go` | Assembles cap-blend first moments and bounds from slab, disk and `internal/cappatch/` patch terms. See `docs/modify-reach-design.md` §8.4. |
| `capblend_moments.go` | `evalCapBlendContext` builds the cap-blend body and assembles bounded area/volume readings over `internal/cappatch/`. See `docs/modify-reach-design.md` §8.4. |
| `capblend_survey.go` | The cap-blend payload's undercut and minimum-radius surveys, per patch and over the receiver's unchanged profile. See `docs/modify-reach-design.md` §12 Table DX (DX7/DX8). |
| `capblend_normal.go` | The certified half of DX7's circular-patch reading: a band patch's own exact normal-component model, enclosed over rational intervals. See the file's doc comment. |
| `capblend_departure.go` | Bounds built band-patch departure from its published surface. See its doc comment. |
| `capblend_admit.go` | Decides by exact rational tests whether `docs/tessellation-reach-design.md` §7's occupied-volume proof covers a cap-blend payload. |

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | `Document.Verify`: resolves the options, verifies each body via `verify_publish.go`, and folds the pair outcomes `verify_pairs.go` proves into the report in pair order. See `docs/verification-design.md` §1-§3 and the file's doc comment. |
| `verify_pairs.go` | `Verify`'s pair walk: lists the pairs box separation does not finish and proves them on a bounded worker pool, returning outcomes and the first error in pair order. See `docs/interference-design.md` §2, §5.3 and §7.2. |
| `report.go` | `Verify`'s report vocabulary: `Status`, `ReadingKind`, `DiagnosticCode`, `SurveyKind`, `DiagnosticPair`, `Diagnostic`, `Interference`, `Clearance`. Types only. See `docs/verification-design.md` §1-§3. |
| `verify_tolerance.go` | `Verify`'s tolerance gate: readings against the caller's relative tolerance, with a `Diagnostic` for each miss. See `docs/verification-design.md` §2-§3. |
| `verify_gate.go` | Proves the reference diameter the tolerance gate is anchored on, per payload. With no provable diameter it withholds the gate rather than guess. See `docs/verification-design.md` §3. |
| `verify_result.go` | Types `Verify`'s report is written in: `Report`, `BodyReport`, and every per-survey result record. Types and `Passed`/`ForBody` only; `verify_publish.go` builds the values. |
| `verify_publish.go` | Builds `Verify` reports from private survey results. See `docs/verification-design.md`. |
| `clearance.go` | The pair kernel: `clearancePair` proves one pair's four-way relation and, when disjoint, a proven gap interval. `sheetSolidPair` decides a sheet pair too. See `docs/clearance-design.md` §1-§3/§6. |
| `clearance_box.go` | Certifies unplaced axis-aligned box prisms and bounds their gap from exact box planes ahead of the kernel. |
| `clearance_planar.go` | The exact planar pair arm for mitred sweeps and faceted results: interference design §3.2. |
| `contact_pair.go` / `contact_pair_memo.go` | Pair gates, reports, each body's report memo. Box classification lives in `internal/pair/`. See `docs/contact-geometry-design.md`. |
| `contact_box.go` | Source-box admission and public face/measurement mapping. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_pair.go` | Planar admission, convexity and the bands. See `docs/multibody-dynamics-design.md` §9–§10. |
| `contact_faceted_manifold.go` / `contact_faceted_patch.go` | Planar manifold faces and witnesses. See multibody §9. |
| `contact_faceted_support.go` | Exact faceted support-face proof and bounded strict separation. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_sweep.go` | Faceted floor sweeps: exact support face, or clearance by swept boxes. See `docs/contact-sweep-design.md`. |
| `contact_oriented_box.go` | Rotated boxes. See `docs/contact-geometry-design.md` §4. |
| `contact_oriented_patch.go` | Exact co-oriented oblique face patch and bounded witnesses. See `docs/contact-geometry-design.md` §4. |
| `contact_clipped_patch.go` | Exact horizontal clip of a rotated source-box face on an axis-aligned one. See `docs/contact-geometry-design.md` §4. |
| `contact_sphere.go` / `contact_sphere_sweep.go` | Axis sphere-box path, including centered rotating sphere drift. See the contact designs. |
| `contact_cylinder.go` / `contact_cylinder_sweep.go` | Source-cylinder axial and circular-sidewall contact and sweeps. See the contact designs. |
| `contact_analytic_manifold.go` | Ruling contacts: clearance certificates and placed poses. See `docs/contact-geometry-design.md` §4.5. |
| `contact_sphere_oriented.go` / `contact_sphere_oriented_sweep.go` | Rotated sphere-box path. See the contact designs. |
| `contact_sphere_pair.go` / `contact_sphere_pair_sweep.go` | Sphere-pair contact and sweep. See the contact designs. |
| `contact_sweep_replay.go` | Cached affine and rotating sweep replay, and each report's replay memo. See `docs/contact-sweep-design.md` §6. |
| `clearance_cells.go` | Face-interior candidates and the pruned, box-sorted cell walk. See `docs/clearance-design.md` §3–§5. |
| `clearance_tiers.go` | The §3 curve and vertex tiers and the §6 ruling certificates. See `docs/clearance-design.md` §3/§4/§6. |
| `clearance_geom.go` | `bodyGeom`: each body's clearance faces, edges and nesting test over `internal/clearance/`'s carriers. See `docs/clearance-design.md` §2–§3. |
| `survey.go` | The analytic wall, undercut, and min-radius surveys on prism, revolve, and cup payloads. An undecided answer reads `Suspect`, never a silent pass. See `docs/verification-design.md` §6. |
| `survey_undercut.go` | `listVerdict`, the surveys' per-list fold of `internal/survey2d/`'s three-valued undercut reader. |
| `interference.go` | The pairwise overlap measurement behind `Verify`. See `docs/interference-design.md` §4-§8 and the file's doc comment. |
| `motion.go` / `motion_verify.go` | `Motion`, `MotionReport`, and the engine shared with `VerifyLinkage`. See `docs/motion-check-design.md`. |
| `motion_bound.go` | Swept-box and corner readings over `internal/motionbound/`. See its doc comment. |
| `linkage.go` / `linkage_verify.go` | `Document.VerifyLinkage`. See `docs/linkage-check-design.md`. |
| `linkage_bound.go` | The chain travel bound. See its doc comment. |
| `contact_sweep.go` | Pair paths, sweeps, and tracks. See `docs/contact-sweep-design.md`. |
| `contact_sweep_rotation.go` / `contact_sweep_faceted.go` | Rotating drift sweep, over source boxes or exact planar bodies. See `docs/contact-sweep-design.md`. |
| `contact_sweep_memo.go` | Sweep run and sweep radius memos. See `docs/contact-sweep-design.md` §7. |
| `contact_sweep_band.go` / `contact_sweep_rolling.go` | Departure and band tracks, planar and rolling. See `docs/multibody-dynamics-design.md` §10.2–§10.6, §10.8. |
| `swept_box.go` | `Document.SweptBox`: an exact whole-path box. See `docs/multibody-dynamics-design.md` §4.2. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` surface over the mesh-boolean evaluator and the typed `BooleanError` mapping. See the file's doc comment and `docs/evaluator-design.md` §9. |
| `prism_boolean.go` | The analytic Union/Cut/Intersect reduction over co-directional coplanar or offset-plane prisms, ahead of the mesh path. See the file's doc comment and `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Cut/Intersect's clean-nesting structural match: the whole-loop tag-map search resolving a clean bore/nested pair. See `docs/prism-boolean-design.md` §4.2. |
| `prism_boolean_blind.go` | Admits blind and spanning Cuts through sketch's whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `stacked_prism.go` | Builds and audits stacked slabs, walls and measurements. See `docs/stacked-prism-design.md`. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing sub-case: per-operand cell classification and `mergePrismCells`. See `docs/prism-boolean-design.md` §4.2. |
| `prism_overlap.go` | `docs/prism-boolean-design.md` §4.5's overlap-area reading, read-only for `Verify`'s interference path alone. See the file's doc comment. |
| `surface_trim.go` | `Trim`/`Extend`/`Split` gates. See surface-intersection §2–§3. |
| `boolean_mesh.go` | `prepBoolMeshContext` prepares an operand's mesh for `internal/meshbool/`'s pipeline. See `docs/evaluator-design.md` §9. |
| `boolean_body.go` | Builds a `facetedPayload` into a `Body`: face/loop/edge topology from the stitched mesh, measurements integrated exactly with composed bounds. See the file's doc comment and `docs/evaluator-design.md` §9. |

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh` and `Body.Tessellate`: the proof record, loop chording, payload dispatch, and the root side of the sheet audit (`requireSheetMesh`, `liftTessellationError`). See `docs/tessellation-design.md`. |
| `tessellate_stacked.go` | Meshes stacked slabs with shared chords and volume proof. See `docs/stacked-prism-design.md` §5. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and what a mesh publishes about its own proofs. See `docs/tessellation-design.md` §1. |
| `tessellate_revolve.go` | `tessellateRevolve`: the tolerance split, the meridian and angular chordings, and the rings, cells, poles and partial caps a revolve builds from them. See the file's doc comment. |
| `tessellate_revolve_proof.go` | Revolve mesh audit wiring over `internal/tessellation/`'s revolve proofs. |
| `tessellate_revolve_arc.go` | `revolveArcStation`: a CIRCULAR generator's meridian stations; its `Ecell` and cap area live in `internal/tessellation/`. |
| `tessellate_revolve_volume.go` | Revolve mesh occupied-volume proof. See the file's doc comment. |
| `tessellate_station.go` | `chordStationBound`: the proven enclosure gap of one interior chord station on a circular walk. See the file's doc comment. |
| `tessellate_stitch.go` | Restates planar stitched triangles or reuses a revolve sheet's curved mesh. See `docs/tessellation-design.md` §2 and `docs/surface-design.md` §10.1. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh off its wall topology. `docs/surface-design.md` §13.4. |
| `tessellate_capblend.go` | `tessellateCapBlend`: the cap-loop chamfer mesh, one chord count per wall walk shared three ways. See `docs/tessellation-reach-design.md` §7. |
| `triangulate.go` | Maps cap points and expected chording errors between `Point2` and `internal/triangulation/`; `cross2` serves root mesh clearance. |
| `export/` | STL, OBJ, and 3MF mesh writers and the analytic/faceted AP214 writer. See `docs/step-export-design.md` and `docs/3mf-export-design.md`. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks). Never `package main`. |
| `dynamics/` | Rigid-body worlds and their scheduled step. See `docs/multibody-dynamics-design.md`. |
| `apitest/` | Tests of the exported API alone. See `apitest/doc.go`. |
| `decadtest/` | The public test kit: comparison helpers over decad's three bounded readings, bodies, reports and surveys, plus sketch-to-body fixtures. Standard `testing` only, never testify. See `decadtest/doc.go`. |
| `internal/proof/` | Exact dyadic arithmetic, rational interval operations, the shared-denominator intervals of the island certificate, float rounding bounds, and their arithmetic tests. |
| `internal/pair/` | Exact source-box and planar solid relations, gaps, face patches and convexity. |
| `internal/tessellation/` | Mesh audits, the loft crossing audit, revolve mesh proofs and their float pre-test, and the loft exact restatement over neutral triangle data. |
| `internal/triangulation/` | Cap hole bridging and ear clipping over plane-local points; returns indexed triangles and marks chording refusals. |
| `internal/proofbound/` | Bounded scalars, faceted measurement bounds, the work budget, certified trig/`atan`/π enclosures and exact rational helpers. See each file's doc comment. |
| `internal/circularmoments/` | Exact rational area, length, endpoint and moment enclosures over neutral circle and arc records. |
| `internal/capcontour/` | Interval enclosures for cap contour points, offset carrier intersections and miter locus speed. See `docs/modify-reach-design.md` §8.3-§8.4. |
| `internal/cappatch/` | Cap-band patch flux, area and first-moment proofs over neutral patch geometry. See `docs/modify-reach-design.md` §8.4. |
| `internal/decaderr/` | The sentinel error values `errors.go` re-exports, so internal packages can return them. |
| `internal/survey2d/` | The 2D inscribed-disk kernel, undercut reader, walk type and interval vectors behind the surveys. See `docs/verification-design.md` §6. |
| `internal/motionbound/` | Exact motion parameters, poses and interval travel bounds behind `VerifyMotion`, and the `RadianSinCos` memo. See `docs/motion-check-design.md`. |
| `internal/freeform/` | Exact free-form arithmetic: Bézier reduction and work budget, arc length, extremes, sagitta stations, convexity, moments and the Sturm/Lipschitz bracket engine. See `docs/spline-design.md`. |
| `internal/meshbool/` | The exact-predicate mesh-boolean pipeline: contact classification and batches, facet subdivision, stitching, the closed-mesh audit, the rounding that keeps a held mesh embedded, and near-contact witnesses. See `docs/evaluator-design.md` §9. |
| `internal/clearance/` | Clearance carriers, angle and line windows, 2D trim regions, ray crossings, boxes, spine and ruling helpers, and the degeneracy oracle. See `docs/clearance-design.md`. |
| `_gallery/` | Own nested module for README images, landing clip, dynamics scenes and linkage clip; keeps SolidLens out of the library. See `main.go`. |
| `_shardgen/` | Own nested module: packs root and `apitest` tests into cost-balanced race shards; the `_` prefix hides it from root-module tools. See its `main.go` doc comment. |
| `.github/workflows/` | `ci.yml` runs lint, tests, tidy and vulnerability checks; race shards run `race-binary`'s root and `apitest` binaries. `codeql.yml`. `test-shards*.txt` assign each test a shard. |
