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
`claude_md_layout_test.go` enforces every mechanical rule above, over this
file and `CLAUDE.md` both, and, because a guard measures only the text it
reads, it classifies the WHOLE of each file rather than the Layout section
alone: a line it reads as a heading or as part of a table, without being the
one spelling declared for it, fails the test rather than being skipped, since
a skipped line escapes the cap and the path check both. So headings here are
ATX (`## Heading`), this file carries exactly one `## Layout` heading,
`CLAUDE.md` carries none, and a table outside that section exists only if the
guard declares it — adding one to either file means declaring it there first.
That guard's `parseAgentDoc` doc comment owns the complete rule list, and the
file-level comment above it owns the two things the rules deliberately leave
to the byte budget.

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
| `docs/modify-reach-design.md` | The approved modify extension: tangent-chain expansion, asymmetric chamfers, cap-loop blends, allowed shells, proof gates, payload topology and staging. |
| `docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `docs/sweep-design.md` | `Path`/`Sweep` transport, refusals, topology, measurements, 3D-sketch boundary, `SweepChain` pairing, and reach. |
| `docs/prism-boolean-design.md` | The analytic `Union`/`Cut`/`Intersect` reduction over co-directional coplanar or offset-plane prisms: reject-only entry gate, private `sketch` scene, displacement bounds. |
| `docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps: entry gate, private `sketch` scene, and cut bounds. |
| `docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `docs/motion-check-design.md` | `Document.VerifyMotion`: the `Motion` set, the per-pose pair proof, the interval certificate, and `MotionReport`. |
| `docs/collision-dynamics-design.md` | Pair contact/sweep in decad and rigid response in `dynamics`. |
| `docs/contact-geometry-design.md` | Pair relation and contact manifold proofs. |
| `docs/contact-sweep-design.md` | Two-body continuous sweep and first-contact brackets. |
| `docs/dynamic-mass-design.md` | Bounded mass and inertia for rigid dynamics. |
| `docs/rigid-dynamics-design.md` | Rigid-body steps, impulses, and reports. |
| `docs/step-export-design.md` | Export package entry points and the AP214 faceted writer contract. |
| `docs/3mf-export-design.md` | The 3MF writer's package parts, mesh mapping, and error contract. |

### Seam and records

| Path | Responsibility |
|---|---|
| `doc.go` | Package doc: scope, the evaluator support-and-refusal map, and the layering contract (`decad -> sketch -> r3 -> units`). |
| `errors.go` | The core §12 sentinel errors, plus the H2 typed `BooleanError`, whose `Code` classifies failures wrapping `ErrBooleanFailed` or `ErrUnsupported`. See `docs/api-design.md` §12, §8. |
| `measurement.go` | The bounded-result shapes: `Exactness`, `Measurement`, `VecMeasurement`, `Box`. See `docs/api-design.md` §5.3, §6. |
| `identity.go` | Private document-local producer identities, the boolean evaluator's operation kind, and the shared zero-vector predicate. |
| `record.go` | Sketch profile, chain, loop and curve records; NURBS validation. See `docs/sketch-seam-design.md` §2. |
| `seam.go` | `RecordProfile`/`RecordChain`, `TExact` admission and record checks. See `docs/sketch-seam-design.md` §1, §7. |
| `path.go` | The immutable spatial `Path` and its sealed `LineTo` / `ArcThrough` segment vocabulary. See `docs/sweep-design.md` §2–§3. |
| `extent.go` | Linear and angular extent types; `ToFace`/`ToFaceAngular` references. See `docs/api-design.md` §8.1. |
| `selector.go` | Selectors: `EdgeQuery`/`FaceQuery`, predicate conjunction and `Exactly`/`AtLeast` cardinality, resolved by filtering live topology; a failure returns a `SelectionError`. See `docs/api-design.md` §9. |
| `selection_error.go` | `SelectionError` (wraps `ErrNoMatch`/`ErrCardinality`) and the canonical `*Query.String()` rendering it and a verification `Diagnostic` both reuse. See `docs/api-design.md` §9. |
| `codec_error.go` | Internal path-aware validation errors for structural curve records. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | The mass-property engine (evaluator §4): closed-form Green's-theorem boundary integrals for `Area`, `Centroid`, `SecondMoments`, per region. See `docs/spline-design.md` §5.2. |
| `mass_properties.go` / `mass_properties_sphere.go` | Mass and inertia for prisms and source spheres. See `docs/dynamic-mass-design.md`. |
| `mass_properties_revolved_cylinder.go` | Mass and inertia for full source cylinders made by revolving an axis-incident rectangle. See `docs/dynamic-mass-design.md`. |
| `moments_trig.go` | `moments.go`'s certified sine/cosine primitive: `turnSinCosInterval` proves an enclosure of sin/cos of an exact rational turn without ever comparing against π. See this file's own doc comment. |
| `bounded.go` | The bounded-scalar vocabulary: a float64 carried beside a proven bound on its own error, its arithmetic, and the three-valued admission readers. See the file's doc comment. |
| `dyadic.go` | The exact BINARY-SCALED arithmetic every proof over held float64 coordinates is carried in: `dyadic`, a mantissa times a power of two, and `dyV3`, its vector. See the file's doc comment. |
| `rat_interval.go` | The exact rational interval arithmetic every certified reading is proven in, plus the `atan`/`atan2` and π enclosures no single rational can state. See the file's doc comment. |
| `moments_circular.go` | Exact rational arc/circle integration for `moments.go`'s boundary sums and `revolve_build.go`'s axis moment. See the file's doc comment. |
| `spline_bezier.go` | The exact reduction of `docs/spline-design.md` §5.1: a recorded free-form curve to piecewise polynomial Bézier control points over `big.Rat`, with no rounding. Owns the §5.2 work-budget charges. |
| `spline_length.go` | Bounded free-form arc length. See `docs/spline-design.md` §6.1. |
| `spline_extreme.go` | `docs/spline-design.md` §6.2's Tier A directional-extreme bracket, reducing to `clearance_poly.go`'s root engine. See the file's doc comment. |
| `spline_fit.go` | `docs/spline-design.md` §5.1.2's fit-spline reduction: converts a recorded `FitSplineSeg` into the same `bezierSpan` chain the other Tier A kinds produce. See the file's doc comment. |
| `spline_moments.go` | The exact integration of `docs/spline-design.md` §5.1 over Bézier spans, reusing `clearance_poly.go`'s `ratPoly`. `addFreeform` feeds `moments.go`'s region-level rational accumulator. |
| `spline_sagitta.go` | `docs/spline-design.md` §6.2.1's chord-sagitta bounds and the shared dyadic station generator built on them. See the file's doc comments. |
| `spline_convexity.go` | `docs/spline-design.md` §6.5: proves a free-form wall edge's single curvature sign from its Bernstein certificate, or refuses. Owns Table R row R19's refusal. See the file's doc comment. |

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | The topology model: `Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`→`Vertex`, plus sealed `Surface`/`Curve` variant sets. See the types' own doc comments and `docs/evaluator-design.md` §3. |
| `normal_bound.go` | The proof behind every `Face.NormalAt` bound: rational-interval enclosures of each arm's exact unit normal, and the radian sine/cosine enclosure the `Cone` arm needs. See the file's doc comment. |
| `document.go` | `Document`, commit, identity and liveness gates; body placement and duplication. See its doc comments and evaluator §8. |
| `surface.go` | `WithSurfaceResult`, sheet refusal, and shared shell/lump helpers. See surface §2-§4, §7, §11. |
| `patch.go` | Builds a single planar face from a recorded profile. See surface §5.1. |
| `thicken.go` | `Body.Thicken` grows an admitted sheet into a solid. See surface §16. |
| `thicken_prism.go` | Builds the certified wall of a prism sheet. See surface §16.2. |
| `thicken_axis.go` | Certifies exact axis-parallel offsets and interval separation. See surface §16.2. |
| `offset.go` | `Body.Offset` builds a second sheet at a stated normal distance, leaving the receiver live. See surface §17. |
| `patch_body.go` | `Body.Patch`: partitions a free-edge selection into closed chains, proves each planar via `dyadic.go` or a shared level token, and fills each with its own face. See `docs/surface-design.md` §5.2. |
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
| `sweep_transport.go` | Rotation-minimizing endpoint-frame transport over exact path records, with rational enclosures of each held frame. See `docs/sweep-design.md` §3.2. |
| `prism_payload.go` | `prismPayload` and its coordinate readings: a world point, its proven bound, and the coordinate envelopes later bounds charge against. See `docs/evaluator-design.md` §5, `docs/prism-boolean-design.md` §7. |
| `prism_build.go` | `evalPrismContext`, caps, and side faces with displacement bounds. See `docs/evaluator-design.md` §5. |
| `segment_walk.go` | The profile-boundary walk extrude, revolve and loft read a `CurveSegment` through: `segmentWalk`, `profileWalks` and the per-kind builders; a kind with no stated bound refuses. See the file's doc comment. |
| `prism_extent.go` | A finished prism's extent readings, reach along a direction and the containing box, each a bounded interval charging the frame, section and axial terms. See `docs/evaluator-design.md` §5. |
| `revolve.go` | `Document.Revolve` (evaluator §6): the sealed `Axis` vocabulary, `EdgeAxis` and `WithSurfaceResult` parsing, angular-extent resolution. Axis, build and extent readings: the other `revolve_*.go` files. |
| `revolve_axis.go` | Resolves the axis into the sketch plane and decides what the profile may do around it: `axisLine2`, `axisFrame`, `wallKind`, and the contact gates. See `docs/evaluator-design.md` §6. |
| `revolve_build.go` | Builds a revolve's body, solid or (`WithSurfaceResult`) sheet, and its measurements. See evaluator §6, `docs/surface-design.md` §4. |
| `revolve_extent.go` | A finished revolve's extent readings: each extreme is a swept extreme, bracketed by `sweepExtremeBounds` rather than read off a boundary vertex. See `docs/evaluator-design.md` §6. |
| `revolve_denotation.go` | `angleDenotation`/`sweepDenotation`: exact stated angles or certified derived-angle intervals, their endpoint displacement and the sweep/trig bounds on them. See `docs/evaluator-design.md` §6, `docs/sweep-design.md` §3. |
| `stops.go` | Body-relative stop resolution for `ToFace`/`ToFaceAngular`/`ThroughAll`/`ThroughAllSide`. See evaluator §5/§6/§11 and the file's doc comments. |
| `loft.go` | `Document.Loft` and `LoftChain`: the entry points over `loft_build.go`'s evaluator, the chain ribbon build, and `WithSurfaceResult` parsing. See `docs/loft-design.md` §2/§4/§10/§16. |
| `loft_build.go` | Loft payload, evaluation, and placement. See `docs/loft-design.md` §5, §8, §12 and `docs/surface-design.md` §4. |
| `loft_pairing.go` | `docs/loft-design.md` Table P: which from-segment walls to which to-segment. A pair the table does not decide is refused outright, never matched to the nearest one. See §5, §5.1. |
| `loft_stations.go` | Places the stations a loft's wall chords run between and proves each chain's departure from its curve, under one shared chord target and a station cap. See `docs/loft-design.md` §5.2. |
| `loft_topology.go` | Assembles the paired stations into the flat-triangle solid the payload holds, and builds the `Body` topology over it. See `docs/loft-design.md` §5.1, §7 and the file's doc comment. |
| `loft_audit.go` | `loftCrossingAudit` proves the assembled triangles manifold and watertight. See `docs/loft-design.md` §6. |
| `loft_moments.go` | `docs/loft-design.md` §8's mass-property engine: `loftMassAccumulator`, an exact-rational tetrahedron sum over the assembled triangle set, publishing Volume/Centroid/Bounds/Area. See §8, §12. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Body.Fillet` rewrites a straight prism's section into a tangent arc at each selected corner and rebuilds through `evalPrism`. It also owns the `cornerBlend` machinery Chamfer reuses. See `docs/modify-design.md` §6. |
| `chamfer.go` | `Body.Chamfer` bevels a straight prism's lateral corners with a chord between setback feet, sharing `cornerBlend` with `fillet.go`; a cap-loop selection routes to `capblend.go`. See `docs/modify-design.md` §7. |
| `fillet_audit.go` | Fillet, Chamfer and Shell section audits. See `docs/modify-design.md` §5. |
| `shell.go` | `Body.Shell` offsets a prism into a tube or cup. See modify §8. |
| `shell_offset.go` | The exact per-feature section offset (`P ⊖ t` / `P ⊕ t`) behind `Shell`, plus the §5 audit wrapper run on the offset section. See `docs/modify-design.md` §7-§8. |
| `shell_cup.go` | `cupPayload` and `evalCup`: the two-co-directional-prism body a one-cap `Shell` builds, with Exact mass properties and roles. See `docs/modify-design.md` §9; clearance stays staged (§12 D6). |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Builds the complete-cap-loop chamfer: `capBlendPayload` plus the selection classification and build gates in `buildCapBlend`. See `docs/modify-reach-design.md` §8.3/§4. |
| `capblend_geom.go` | Builds the `capBlendPayload` topology in `buildCapBand`: trimmed side walls, cap faces, and Plane/Cone band patches. See `docs/modify-reach-design.md` §8.3. |
| `capblend_contour.go` | Proves the cap contour's displacement bound every cap-level reading charges, plus a miter ruling's own locus-speed bound. See `docs/modify-reach-design.md` §8.3-§8.4. |
| `capblend_centroid.go` | Closed-form first moments for the cap-blend centroid: exact-rational Plane patch moments, a Fourier sum for Cone patches, and a bounding-box ceiling on the result. See `docs/modify-reach-design.md` §8.4. |
| `capblend_moments.go` | `evalCapBlendContext` builds the cap-blend body and its bounded area/volume/centroid by closed-form per-patch integrals. See `docs/modify-reach-design.md` §8.4. |
| `capblend_survey.go` | The cap-blend payload's undercut and minimum-radius surveys, per patch and over the receiver's unchanged profile. See `docs/modify-reach-design.md` §12 Table DX (DX7/DX8). |
| `capblend_normal.go` | The certified half of DX7's circular-patch reading: a band patch's own exact normal-component model, enclosed over rational intervals. See the file's doc comment. |
| `capblend_departure.go` | Bounds built band-patch departure from its published surface. See its doc comment. |
| `capblend_admit.go` | Decides by exact rational tests whether `docs/tessellation-reach-design.md` §7's occupied-volume proof covers a cap-blend payload. |

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | `Document.Verify`: resolves the options, verifies each body via `verify_publish.go`, and partitions body pairs for interference. See `docs/verification-design.md` §1-§3 and the file's doc comment. |
| `report.go` | `Verify`'s report vocabulary: `Status`, `ReadingKind`, `DiagnosticCode`, `SurveyKind`, `DiagnosticPair`, `Diagnostic`, `Interference`, `Clearance`. Types only. See `docs/verification-design.md` §1-§3. |
| `verify_tolerance.go` | `Verify`'s tolerance gate: which readings meet the caller's relative tolerance against the body's reference, and a `Diagnostic` for each that does not. See `docs/verification-design.md` §2-§3. |
| `verify_gate.go` | Proves the reference diameter the tolerance gate is anchored on, per payload. With no provable diameter it withholds the gate rather than guess. See `docs/verification-design.md` §3. |
| `verify_result.go` | The result vocabulary `Verify`'s report is written in: `Report`, `BodyReport`, and every per-survey result record. Types and `Passed`/`ForBody` only; `verify_publish.go` builds the values. |
| `verify_publish.go` | Builds `Verify` reports from private survey results. See `docs/verification-design.md`. |
| `clearance.go` | The pair kernel: `clearancePair` proves one pair's four-way relation and, when disjoint, a proven gap interval. `sheetSolidPair` decides a sheet pair too. See `docs/clearance-design.md` §1-§3/§6. |
| `clearance_box.go` | Certifies unplaced axis-aligned rectangular prisms and bounds their gap directly from exact box planes before the general pair kernel. |
| `contact_pair.go` | Pair gates and verdict. See `docs/contact-geometry-design.md`. |
| `contact_box.go` | Box manifold. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_support.go` | Exact rectangular support-face proof for a zero-bound faceted solid and floor contact/gap classification. See `docs/contact-geometry-design.md` §4. |
| `contact_faceted_sweep.go` | Affine vertical sweep of a certified faceted lower face against a source-box floor. See `docs/contact-sweep-design.md`. |
| `contact_oriented_box.go` | Rotated boxes. See `docs/contact-geometry-design.md` §4. |
| `contact_oriented_patch.go` | Exact co-oriented oblique face patch, source faces, and bounded witnesses. See `docs/contact-geometry-design.md` §4. |
| `contact_clipped_patch.go` | Exact horizontal clipping of one rotated source-box face against an axis-aligned face. See `docs/contact-geometry-design.md` §4. |
| `contact_sphere.go` / `contact_sphere_sweep.go` | Axis sphere-box path. See the contact designs. |
| `contact_cylinder.go` / `contact_cylinder_sweep.go` | Source-cylinder axial clearance. See the contact designs. |
| `contact_sphere_oriented.go` / `contact_sphere_oriented_sweep.go` | Rotated sphere-box path. See the contact designs. |
| `contact_sphere_pair.go` / `contact_sphere_pair_sweep.go` | Sphere-pair contact and sweep. See the contact designs. |
| `contact_sweep_replay.go` | Cached affine and rotating sweep replay. See `docs/contact-sweep-design.md` §6. |
| `clearance_degen.go` | Degeneracy tests. See `docs/clearance-design.md` §4/§5. |
| `clearance_cells.go` | Face-interior candidates. See `docs/clearance-design.md` §3/§4. |
| `clearance_tiers.go` | The curve and vertex tiers of §3: face-edge, edge-edge and vertex cells over §4's curve-tier table; constant-distance families emit only on the oracle's `degYes`. See `docs/clearance-design.md` §3/§4. |
| `clearance_geom.go` | Boundary carriers and nesting rays for clearance. See `docs/clearance-design.md` §2–§3. |
| `clearance_poly.go` | The certified-bracket machinery of §4/§5: Sturm sequences over exact rationals isolate stationarity polynomials, then a proven Lipschitz bound brackets each critical value. See `docs/clearance-design.md` §4/§5. |
| `survey.go` | The analytic wall, undercut, and min-radius surveys on prism, revolve, and cup payloads. An undecided answer reads `Suspect`, never a silent pass. See `docs/verification-design.md` §6. |
| `survey_undercut.go` | The exact three-valued receiver-face undercut reader `prismUndercuts`/`cupUndercuts`/`capBlendUndercuts` share, decided over the rationals, no float allowance. See the file's doc comment. |
| `survey2d.go` | The 2D closed-form inscribed-disk kernel behind the wall survey, shared with the modify section audit via `elemOf`; exact candidates for line/arc boundaries. See `docs/verification-design.md` §6. |
| `budget.go` | `workBudget`, the shared bounded work counter read-only and pre-commit audit phases poll via `step`/`err`. It holds closures, never a stored `context.Context`. See `docs/interference-design.md` §7.2. |
| `interference.go` | The pairwise overlap measurement behind `Verify`. See `docs/interference-design.md` §4-§8 and the file's doc comment. |
| `motion.go` / `motion_verify.go` | The `Motion` set, its options and `MotionReport`; `Document.VerifyMotion`'s swept-box exclusion, transient poses and interval certificate. See `docs/motion-check-design.md`. |
| `motion_bound.go` | Exact motion bounds. See its doc comment. |
| `contact_sweep.go` | Pair paths, sweeps, and tracks. See `docs/contact-sweep-design.md`. |
| `contact_sweep_rotation.go` | Rotating drift sweep. See `docs/contact-sweep-design.md`. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` surface over the mesh-boolean evaluator and the typed `BooleanError` mapping. See the file's doc comment and `docs/evaluator-design.md` §9. |
| `boolean_parallel.go` | The bounded ordered contact-classification batches `facesNearMiss` and `meshBoolean` share; workers classify uncached facet pairs into indexed slots, memo access and aggregation stay serial. |
| `prism_boolean.go` | The analytic Union/Cut/Intersect reduction over co-directional coplanar or offset-plane prisms, ahead of the mesh path. See the file's doc comment and `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Cut/Intersect's clean-nesting structural match (§4.2): the whole-loop tag-map search resolving a clean bore/nested pair. See the file's doc comment and `docs/prism-boolean-design.md`. |
| `prism_boolean_blind.go` | Admits blind and spanning Cuts through sketch's whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `stacked_prism.go` | Builds and audits stacked slabs, walls and measurements. See `docs/stacked-prism-design.md`. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing sub-case (§4.2): edge-orientation propagation classifies each cell per operand; `mergePrismCells` assembles the selection. See the file's doc comment and `docs/prism-boolean-design.md`. |
| `prism_overlap.go` | `docs/prism-boolean-design.md` §4.5's overlap-area reading, read-only for `Verify`'s interference path alone. See the file's doc comment. |
| `surface_trim.go` | `Trim`/`Extend`/`Split` gates. See surface-intersection §2–§3. |
| `boolean_mesh.go` | The exact-predicate mesh-boolean pipeline: contact classification, subdivision, stitching, and the closed-mesh audit. See the file's doc comment and `docs/evaluator-design.md` §9. |
| `boolean_cut.go` | Per-facet exact subdivision along contact segments into classified regions, in rational 2D on the facet's own plane. See the file's doc comment. |
| `boolean_exact.go` | The exact-arithmetic kernel behind the mesh boolean: adaptive orient3d, rational predicates, and the reject-only pre-filters. See each filter's own doc comment. |
| `boolean_body.go` | Builds a `facetedPayload` into a `Body`: face/loop/edge topology from the stitched mesh, measurements integrated exactly with composed bounds. See the file's doc comment and `docs/evaluator-design.md` §9. |
| `bounds.go` | Faceted measurement error bounds. See its doc comment and `docs/prism-boolean-design.md` §7. |

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh` and `Body.Tessellate`: the proof record and which level publishes it, the shared loop chording, the dispatch to each payload path. See the file's doc comment and `docs/tessellation-design.md`. |
| `tessellate_stacked.go` | Meshes stacked slabs with shared chords and volume proof. See `docs/stacked-prism-design.md` §5. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and what a mesh publishes about its own proofs. See `docs/tessellation-design.md` §1. |
| `tessellate_revolve.go` | `tessellateRevolve`: the tolerance split, the meridian and angular chordings, and the rings, cells, poles and partial caps a revolve builds from them. See the file's doc comment. |
| `tessellate_revolve_proof.go` | Revolve mesh proofs and audits. See the file's doc comment. |
| `tessellate_revolve_arc.go` | What a CIRCULAR revolve generator needs: its meridian stations, its `Ecell` by certified subdivision, and its cap segment area. See the file's doc comment. |
| `tessellate_revolve_volume.go` | Revolve mesh occupied-volume proof. See the file's doc comment. |
| `tessellate_station.go` | `chordStationBound`: the proven enclosure gap of one interior chord station on a circular walk. See the file's doc comment. |
| `tessellate_sheet.go` | `requireSheetMesh` and `requireSheetVertexLinks`: the sheet mesh's manifold-with-boundary and vertex-link audits. See `docs/tessellation-design.md` §1.2. |
| `tessellate_loft.go` | `tessellateLoft`: the exact restatement of a `loftPayload`'s held triangle set and proof record. See the file's doc comments. |
| `tessellate_stitch.go` | Restates planar stitched triangles or reuses a revolve sheet's curved mesh. See `docs/tessellation-design.md` §2 and `docs/surface-design.md` §10.1. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh off its wall topology. `docs/surface-design.md` §13.4. |
| `tessellate_capblend.go` | `tessellateCapBlend`: the cap-loop chamfer mesh, one chord count per wall walk shared three ways. See `docs/tessellation-reach-design.md` §7. |
| `triangulate.go` | The cap triangulator behind `Tessellate`: hole bridging plus reflex-blocked ear clipping, correct for non-convex outlines with holes. See the file's doc comment. |
| `export/` | STL, OBJ, and 3MF mesh writers and the analytic/faceted AP214 writer. See `docs/step-export-design.md` and `docs/3mf-export-design.md`. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks) that double as living documentation. Never `package main`. |
| `dynamics/` | Rigid-body state and response, including fixed-box edge impulses in `fixed_offcenter.go`. See `docs/rigid-dynamics-design.md`. |
| `decadtest/` | The public test kit: comparison helpers over decad's three bounded readings, bodies, reports and surveys, plus sketch-to-body fixtures. Standard `testing` only, never testify. See `decadtest/doc.go`. |
| `_gallery/` | Own nested module for README stills, animated hero and landing clip; keeps SolidLens out of the library. See `main.go`. |
| `_shardgen/` | Own nested module, keeping tooling out of the library's: packs the root package's tests into cost-balanced race shards. The `_` prefix hides it from root-module tools. See its `main.go` doc comment. |
| `.github/workflows/` | `ci.yml` runs lint, tests, tidy and vulnerability checks; root race shards depend on `race-binary`. `codeql.yml`. `test-shards.txt` beside it records which shard runs each root test. |
