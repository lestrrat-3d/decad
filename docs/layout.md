# decad file layout

Every row names what a file owns and where its detail lives. A row is a
POINTER, never a summary — read "Layout rows" below before editing one.
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
| `docs/api-design.md` | The core public API contract: recipe/evaluator split, forward-compat invariants, and the feature, selector and verification surface. Points at every companion design. |
| `docs/sketch-seam-design.md` | The recording contract at the `sketch` seam: the trim contract (`TExact`), the `CurveSegment` recording IR, and `ErrUnrecordableProfile`. |
| `docs/recipe-replay-design.md` | The stored-recipe contract: strict versioned encoding, validation and liveness, error precedence, resource limits, and whole-recipe atomic evaluation. |
| `docs/verification-design.md` | How `Verify` judges every bounded result: the report and its statuses, interference cost and deadlines, `WithTolerance`, and the diameter-anchored noise floor. |
| `docs/payload-verification-design.md` | How `Verify` covers each evaluator payload: per-payload proofs, boundary certificates, bounded validity/clearance/survey algorithms, and required tests. |
| `docs/evaluator-design.md` | The v1 evaluator: the evaluate-from-the-record rule, topology and provenance roles, mass properties, per-feature build tables, staging via `ErrUnsupported`, and the mesh boolean. |
| `docs/tessellation-design.md` | The tessellation contract for every payload: shared curve samples, manifold proofs, source faces, boundary certificates, and boolean handoff. |
| `docs/clearance-design.md` | The clearance kernel: stationarity-tier candidate enumeration, the face-pair distance table, the disjointness proof, and how rows feed the report. |
| `docs/interference-design.md` | Non-mutating pairwise overlap proof: the four-way pair relation, containment volume reuse, read-only mesh intersection, cancellation, and refusal rules. |
| `docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` in four normative tables (receiver, refusals, result, consumers), plus the section-rewrite reduction, the exact offset, and the build-time audit. |
| `docs/spline-design.md` | The free-form kinds: per-kind exactness tiers, refusals and their sentinels, exact rational Tier A moments and their work budget, proven brackets, and reach per capability. |
| `docs/modify-reach-design.md` | The approved modify extension: tangent-chain expansion, asymmetric chamfers, cap-loop blends, allowed shells, proof gates, payload topology and staging. |
| `docs/loft-design.md` | The count-free `Loft` design in four normative tables (pairing, refusals, result, consumers), its exact-rational mass properties, and the wall-crossing audit. |
| `docs/prism-boolean-design.md` | The analytic reduction for `Union`/`Cut`/`Intersect` over co-directional coplanar prisms: the reject-only entry gate, the private `sketch` scene, and section/axial displacement bounds. |
| `docs/tessellation-reach-design.md` | The tessellation reach plan: the loft restatement, free-form prism chording, revolve T2–T4 and the cap-loop chamfer tessellator, each with its cells, proof terms, refusals and tests. |

### Seam, records and recipes

| Path | Responsibility |
|---|---|
| `doc.go` | Package doc: scope, the evaluator support-and-refusal map, and the layering contract (`decad -> sketch -> r3 -> units`). |
| `errors.go` | The core §12 sentinel error vocabulary, plus the H2 typed `BooleanError` (`Op`/`Inputs`/`Code`) whose `BooleanErrorCode` set wraps `ErrBooleanFailed` or `ErrUnsupported`. See `docs/api-design.md` §12, §8. |
| `measurement.go` | The bounded-result shapes: `Exactness`, `Measurement`, `VecMeasurement`, `Box`. See `docs/api-design.md` §5.3, §6. |
| `record.go` | The recording IR: `PlaneRecord`, `ProfileRecord`, `LoopRecord`, and the ten sealed `CurveSegment` variants with their tagged codec. NURBS validation rules are documented on their own functions. See `docs/sketch-seam-design.md` §2. |
| `seam.go` | The seam conversion `RecordProfile(s, p)`: admits, authenticates and records a profile, then applies the `TExact` admission gate and the reject-only range and loop-closure falsifiers. See `docs/sketch-seam-design.md` §1, §7. |
| `extent.go` | The extent vocabulary: the sealed linear `Extent`/`SideExtent` and angular `AngularExtent`/`SideAngular` tiers, deliberately disjoint, each with a tagged codec. `ToFace`/`ToFaceAngular` record their body as a `StepRef`. See `docs/api-design.md` §8.1. |
| `recipe.go` / `recipe_wire.go` | The Recipe IR, Step's wire codec, and the strict/versioned root codec returning a path-aware `RecipeError`. Placement and per-op field presence are enforced on both wire directions. See `docs/recipe-replay-design.md` §2, §3.2, §6. |
| `recipe_decode.go` | The bounded JSON preflight scanner behind `Recipe`'s decoder: charges every array element and string byte against the fixed resource-limit ceilings before typed allocation. See `docs/recipe-replay-design.md` §7. |
| `selector.go` | The selector vocabulary: `EdgeQuery`/`FaceQuery`, predicate conjunction plus `Exactly`/`AtLeast` cardinality, and their tagged codec. Resolution is a filter pipeline over live topology; a failing resolution returns a `SelectionError`. See `docs/api-design.md` §9. |
| `selection_error.go` | `SelectionError` (wraps `ErrNoMatch`/`ErrCardinality`) and the canonical `*Query.String()` rendering it and a verification `Diagnostic` both reuse. See `docs/api-design.md` §9. |
| `codec_error.go` | The path-aware codec error machinery behind every recorded-step decode: builds the JSON-style path a `RecipeError` reports. See `docs/recipe-replay-design.md` §6. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | The mass-property engine (evaluator §4): closed-form Green's-theorem boundary integrals for `Area`, `Centroid`, `SecondMoments`, accumulated per region; Tier A free-form terms route through one record-level work preflight. See `docs/spline-design.md` §5.2. |
| `moments_trig.go` | `moments.go`'s certified sine/cosine primitive: `turnSinCosInterval` proves an enclosure of sin/cos of an exact rational turn without ever comparing against π. See this file's own doc comment. |
| `bounded.go` | The bounded-scalar vocabulary: a float64 carried beside a proven bound on its own error, its arithmetic, and the three-valued admission readers. Every operation charges its own rounding on top of the operand bounds. See the file's own doc comment. |
| `rat_interval.go` | The exact rational interval arithmetic every certified reading is proven in, plus the `atan`/`atan2` and π enclosures no single rational can state. See the file's own doc comment. |
| `moments_circular.go` | Integrates a recorded arc or circle in exact rational intervals for `moments.go`'s boundary sums. A record that does not determine the term exactly withholds it rather than approximating it. See the file's own doc comment. |
| `spline_bezier.go` | The exact reduction of `docs/spline-design.md` §5.1: converts a recorded free-form curve to piecewise polynomial Bézier control points over `big.Rat`, with no rounding. Owns the §5.2 work-budget charges. |
| `spline_length.go` | `docs/spline-design.md` §6.1: brackets a free-form curve's arc length between its chord and control polygon, narrowed by exact dyadic de Casteljau bisection to a fixed depth. |
| `spline_extreme.go` | `docs/spline-design.md` §6.2: brackets a Tier A free-form span's directional extreme over `gu·u + gv·v`, reducing to `clearance_poly.go`'s certified root engine via the Bernstein convex-hull property. Owns Table R row R18's refusal. See the file's own doc comment. |
| `spline_fit.go` | `docs/spline-design.md` §5.1.2's fit-spline reduction: converts a recorded `FitSplineSeg` into the same `bezierSpan` chain the other Tier A kinds produce, over its own closed form rather than knot insertion. See the file's own doc comment. |
| `spline_moments.go` | The exact integration of `docs/spline-design.md` §5.1 over Bézier spans, reusing `clearance_poly.go`'s `ratPoly`. `addFreeform` feeds `moments.go`'s region-level rational accumulator. |
| `spline_sagitta.go` | `docs/spline-design.md` §6.2.1's chord-sagitta bounds and the shared dyadic station generator built on them. Each bound's derivation, refusal and work charge lives on its own function's doc comment. See the file's own doc comments. |
| `spline_convexity.go` | `docs/spline-design.md` §6.5: proves a free-form wall edge's single curvature sign from the curvature numerator's Bernstein coefficients, subdividing to a fixed depth, or refuses. Owns Table R row R19's refusal. See the file's own doc comment. |

### Features

| Path | Responsibility |
|---|---|
| `topology.go` | The topology model (evaluator §3): `Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`→`Vertex`, plus sealed `Surface`/`Curve` variant sets. Convexity, exactness rules and immutability are on the types' own doc comments; see `docs/evaluator-design.md` §3. |
| `normal_bound.go` | The proof behind the bound every `Face.NormalAt` arm publishes: rational-interval enclosures of each arm's own exact unit normal, and the radian sine/cosine enclosure the `Cone` arm needs. See the file's own doc comment. |
| `document.go` | `Document` (`New`/`Bodies`/`Recipe`), its atomic commit tail, and retire/liveness gates. `Body.Placed`/`Duplicate`/`PlacedCopy` re-evaluate the payload under a composed motion; see their doc comments and `docs/evaluator-design.md` §8. |
| `extrude.go` | `Document.Extrude` (evaluator §5): the public entry point, `WithTaper`, and linear-extent resolution into a `linearSweep`. The payload, the build and the extent readings each have their own `prism_*.go` file. See `docs/evaluator-design.md` §5. |
| `prism_payload.go` | `prismPayload` and the coordinate readings taken off it: a world point, its proven bound, and the profile coordinate envelopes later bounds are charged against. See `docs/evaluator-design.md` §5, `docs/prism-boolean-design.md` §7. |
| `prism_build.go` | Builds a straight extrude's body from its payload: `evalPrismContext`, the caps, and `buildLoopSidesAs`'s per-loop side walk. Each face carries the displacement its own surface was built from. See `docs/evaluator-design.md` §5. |
| `segment_walk.go` | The package's profile-boundary walk: `segmentWalk`, `profileWalks`, and the per-kind builders extrude, revolve and loft all read a recorded `CurveSegment` through. A kind with no stated bound refuses. See the file's own doc comment. |
| `prism_extent.go` | The extent readings asked of a finished prism: reach along a direction, and the containing box. Every answer is a bounded interval charging the frame, section and axial terms. See `docs/evaluator-design.md` §5. |
| `revolve.go` | `Document.Revolve` (evaluator §6): the sealed `Axis` vocabulary with its codec, `EdgeAxis` resolution, and angular-extent resolution. The axis frame, the build and the extent readings each have their own `revolve_*.go` file. See `docs/evaluator-design.md` §6. |
| `revolve_axis.go` | Resolves the axis into the sketch plane and decides what the profile may do around it: `axisLine2`, `axisFrame`, `wallKind` classification, and the contact gates. See `docs/evaluator-design.md` §6. |
| `revolve_build.go` | Builds a revolve's body: the wall surface each segment sweeps, the caps a partial sweep closes with, the poles and seams a full sweep joins, and the measurements published. See `docs/evaluator-design.md` §6. |
| `revolve_extent.go` | The extent readings asked of a finished revolve. An extreme is a swept extreme, bracketed by `sweepExtremeBounds` rather than read off a boundary vertex. See `docs/evaluator-design.md` §6. |
| `stops.go` | Body-relative stop resolution for `ToFace`/`ToFaceAngular`/`ThroughAll`/`ThroughAllSide` (evaluator §5/§6/§11, core §8.1/§6.2): each stop body resolves at the call and records as a `StepRef`, never consumed. See doc comments on `resolveToFace`/`resolveThroughAll`/`resolveToFaceAngular`. |
| `loft.go` | `docs/loft-design.md` PR 1b: `Document.Loft`/`LoftContext`, the public entry point over `loft_build.go`'s evaluator. Owns gates S9–S11 and S4's arity half; the step commits only after `evalLoft` succeeds. See doc comments; `docs/loft-design.md` §2/§4/§10. |
| `loft_build.go` | `docs/loft-design.md` PR 1a/2a: `loftPayload`, `loftMeshProof`, `evalLoft` and `placed` — the gates it owns, the placement re-lift and its `delta`, and the four measurements. Pairing, stations and topology each have their own `loft_*.go` file. See `docs/loft-design.md` §5, §8, §12. |
| `loft_pairing.go` | `docs/loft-design.md` Table P: which from-segment walls to which to-segment. A pair the table does not decide is refused outright, never matched to the nearest one. See §5, §5.1. |
| `loft_stations.go` | Places the stations a loft's wall chords run between and proves each chain's departure from the curve it approximates, under one shared chord target and a station cap. See `docs/loft-design.md` §5.2. |
| `loft_topology.go` | Assembles the paired stations into the flat-triangle solid the payload holds, and builds the `Body` topology over it. The triangles are what is built; the curved solid is reached through the proven displacement. See `docs/loft-design.md` §5.1, §7. |
| `loft_audit.go` | The build-time crossing audit of `docs/loft-design.md` §6: `loftCrossingAudit` proves the assembled triangle set manifold and watertight, reusing `boolean_exact.go` and `boolean_mesh.go`'s `triTriClassify` unchanged. Gate order is S6, S8, S7. |
| `loft_moments.go` | `docs/loft-design.md` §8's mass-property engine: `loftMassAccumulator`, an exact-rational tetrahedron sum over the assembled triangle set, publishing Volume/Centroid/Bounds/Area under the payload's own placement displacement. See §8, §12. |

### Modify

| Path | Responsibility |
|---|---|
| `fillet.go` | `Body.Fillet` rewrites a straight prism's section into a tangent arc at each selected corner and rebuilds through `evalPrism`. It also defines the shared `cornerBlend` machinery Chamfer reuses. See `docs/modify-design.md` §6. |
| `chamfer.go` | `Body.Chamfer` bevels a straight prism's lateral corners with a chord between setback feet, sharing `cornerBlend` machinery with `fillet.go`. A cap-loop selection instead routes to `capblend.go`. See `docs/modify-design.md` §7. |
| `fillet_audit.go` | The shared §5 audit of a modify op's rewritten section — orientation, self-consuming trim, crossing/contact, and nesting — run by Fillet and Chamfer, and reused by Shell's offset audit. See `docs/modify-design.md` §5. |
| `shell.go` | `Body.Shell` removes a prism's cap faces and offsets the section into a wall of thickness `t`, building a tube (both caps) or a `cupPayload` (one cap). See `docs/modify-design.md` §8. |
| `shell_offset.go` | The exact per-feature section offset (`P ⊖ t` / `P ⊕ t`) behind `Shell`, plus the §5 audit wrapper run on the offset section. See `docs/modify-design.md` §7-§8. |
| `shell_cup.go` | `cupPayload` and `evalCup`: the two-co-directional-prism body a one-cap `Shell` builds, with Exact mass properties and roles. See `docs/modify-design.md` §9; clearance stays staged (§12 D6). |

### Cap-loop chamfer

| Path | Responsibility |
|---|---|
| `capblend.go` | Builds the complete-cap-loop chamfer: `capBlendPayload` plus the selection classification and build gates in `buildCapBlend`. Gate order and sentinels are documented per function; see `docs/modify-reach-design.md` §8.3/§4. |
| `capblend_geom.go` | Builds the `capBlendPayload` topology in `buildCapBand`: trimmed side walls, cap faces, and Plane/Cone band patches, each stamped with its own readings. See the per-function doc comments and `docs/modify-reach-design.md` §8.3. |
| `capblend_contour.go` | Proves the cap contour's displacement bound every cap-level reading charges, plus a miter ruling's own locus-speed bound. See the file's own doc comments and `docs/modify-reach-design.md` §8.3-§8.4. |
| `capblend_centroid.go` | Computes closed-form first moments for the cap-blend payload's centroid: exact-rational Plane patch moments, a Fourier sum for Cone patches, and a bounding-box ceiling on the result. See `docs/modify-reach-design.md` §8.4. |
| `capblend_moments.go` | `evalCapBlendContext` builds the cap-blend body and its bounded area/volume/centroid by closed-form per-patch integrals. See `docs/modify-reach-design.md` §8.4. |
| `capblend_survey.go` | The cap-blend payload's undercut and minimum-radius surveys, per patch and over the receiver's unchanged profile. See the file's own doc comments and `docs/modify-reach-design.md` §12 Table DX (DX7/DX8). |
| `capblend_normal.go` | The certified half of DX7's circular-patch reading: the rational-interval enclosure of a band patch's own exact normal-component model, and the proven enclosure of a harmonic form's extremes over one window. See the file's own doc comment. |
| `capblend_departure.go` | The proven bound on how far a band patch's BUILT ruled surface points away from the surface it publishes, measured in world space from the published corners, curves and tag. See the file's own doc comment. |

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | Implements `Document.Verify`: resolves the options, verifies each body, and partitions body pairs into disjoint, overlapping, or undecided. The report vocabulary, the tolerance gate and the gate diameter each have their own file. See `docs/verification-design.md` §1-§3. |
| `report.go` | The vocabulary `Verify`'s report is written in: `Status`, `ReadingKind`, `DiagnosticCode`, `Diagnostic`, and the `Report`/`BodyReport` shapes. Types only; `verify.go` builds them. See `docs/verification-design.md` §1-§3. |
| `verify_tolerance.go` | `Verify`'s tolerance gate: which readings are trustworthy at the caller's relative tolerance, and a `Diagnostic` for each that is not. Every comparison is against a reference the body supplies. See `docs/verification-design.md` §2-§3. |
| `verify_gate.go` | Proves the reference diameter the tolerance gate is anchored on, per payload. A payload with no provable diameter withholds the gate rather than anchoring it on a guess. See `docs/verification-design.md` §3. |
| `clearance.go` | The pair kernel: `clearancePair` proves one pair's four-way relation and, when disjoint, a proven gap interval. The coplanar-plane certificate runs first and short-circuits later checks. See `docs/clearance-design.md` §1/§2/§6. |
| `clearance_degen.go` | The degeneracy oracle every cell asks before emitting a constant/`Exact` candidate: three-valued `degYes`/`degNo`/`degUnknown`, `degYes` only from exact rational arithmetic, never a tolerance. See `docs/clearance-design.md` §4/§5. |
| `clearance_cells.go` | The §3 candidate sink and §4 face-interior table: enumerates stationarity tiers per face pair, folds admission into contributions, and reduces offset-surface pairs to spine-pair criticals. See `docs/clearance-design.md` §3/§4. |
| `clearance_tiers.go` | The curve and vertex tiers of §3: face-edge, edge-edge, and vertex cells over §4's curve-tier table. Constant-distance families emit only on the degeneracy oracle's `degYes`. See `docs/clearance-design.md` §3/§4. |
| `clearance_geom.go` | The kernel's boundary model: builds trimmed carrier faces, edges and vertices from a body's payload and runs the §2 nesting ray casts, closed form except a Sturm-certified torus quartic. See `docs/clearance-design.md` §2/§3. |
| `clearance_poly.go` | The certified-bracket machinery of §4/§5: isolates stationarity polynomials by Sturm sequences over exact rationals, then brackets each critical value by a proven Lipschitz bound. See `docs/clearance-design.md` §4/§5. |
| `survey.go` | The analytic wall, undercut, and min-radius surveys on prism, revolve, and cup payloads. An undecided answer reads `Suspect`, never a silent pass. See `docs/verification-design.md` §6. |
| `survey_undercut.go` | The exact three-valued receiver-face undercut reader `prismUndercuts`/`cupUndercuts`/`capBlendUndercuts` share: `decidePull`, `wallNormalDecision` and `capNormalDecision`, decided over the rationals with no float allowance. `revolveUndercuts` is not converted. |
| `survey2d.go` | The 2D closed-form inscribed-disk kernel behind the wall survey, shared with the modify section audit via `elemOf`. Its candidate set is exact for line/arc boundaries. See `docs/verification-design.md` §6. |
| `budget.go` | `workBudget`, the shared bounded work counter read-only and pre-commit audit phases poll via `step`/`err`. It holds closures, never a stored `context.Context`. See `docs/interference-design.md` §7.2. |
| `interference.go` | The pairwise overlap measurement behind `Verify`: containment and equality certificates reuse an operand's volume; every other supported pair runs read-only `OpIntersect` under the positive-volume gate. See `docs/interference-design.md` §4-§8. |

### Booleans

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` surface over the mesh-boolean evaluator and the typed `BooleanError` mapping; tries the prism-boolean analytic reduction first, and owns `Verify`'s read-only `OpIntersect` twin of it. See `Union`'s doc comment and `docs/evaluator-design.md` §9. |
| `prism_boolean.go` | The analytic Union/Cut/Intersect reduction over co-directional coplanar prisms: shared G1-G4 admission, work-budget cap, scene construction, and the shared merge/chain machinery, dispatched ahead of the mesh path. See the file's own doc comment and `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Cut/Intersect's clean-nesting structural match (§4.2's "clean" sub-case): the whole-loop tag-map search resolving a clean bore/nested pair, falling to the crossing sub-case when unresolved. See the file's own doc comment and `docs/prism-boolean-design.md`. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing sub-case (§4.2): edge-orientation propagation classifies each arrangement cell per operand; `mergePrismCells` assembles the selected set. See the file's own doc comment and `docs/prism-boolean-design.md`. |
| `prism_overlap.go` | `docs/prism-boolean-design.md` §4.5's overlap-area reading: measures a coplanar prism pair's overlap cell by cell and publishes the charged sum as a volume, with no assembled section and no body. Read-only, for `Verify`'s interference path alone. See the file's own doc comment. |
| `boolean_mesh.go` | The exact-predicate mesh-boolean pipeline: contact classification, subdivision, stitching, and the closed-mesh audit. See the file's own doc comment and `docs/evaluator-design.md` §9. |
| `boolean_cut.go` | Per-facet exact subdivision along contact segments into classified regions, in rational 2D on the facet's own plane. See the file's own doc comment. |
| `boolean_exact.go` | The exact-arithmetic kernel behind the mesh boolean: adaptive orient3d, rational predicates, and the reject-only segment and facet-pair pre-filters. Their admission derivations live in `segAdmissionRadius2`'s, `tooFar`'s and `triTriMissesFilter`'s own doc comments. |
| `boolean_body.go` | Builds a `facetedPayload` into a `Body`: face/loop/edge topology from the stitched mesh, and measurements integrated exactly with composed proven bounds. See the file's own doc comment and `docs/evaluator-design.md` §9. |
| `bounds.go` | The single owner of every proven error bound a faceted measurement reports, one helper per mechanism, including `docs/prism-boolean-design.md` §7's section-displacement terms. See the file's own doc comment. |

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh` and `Body.Tessellate`: the proof record, the shared loop chording, and the dispatch to each payload path. See the file's own doc comment and `docs/tessellation-design.md`. |
| `tessellate_revolve.go` | `tessellateRevolve`: the tolerance split, the meridian and angular chordings, and the rings, cells, poles and partial caps a revolve builds from them. See the file's own doc comment. |
| `tessellate_revolve_proof.go` | The revolve mesh's proofs: certified angular trig, both coordinate stages, the facet-pair and vertex-link audits, and a straight cell's `Ecell`. See the file's own doc comment. |
| `tessellate_revolve_arc.go` | What a CIRCULAR revolve generator needs: its meridian stations, its `Ecell` by certified subdivision, and its cap segment area. See the file's own doc comment. |
| `tessellate_revolve_volume.go` | The revolve mesh's occupied-volume proof: the meridian first-moment allowance, the per-cell angular homotopy integral and the two coordinate-stage allowances. See the file's own doc comment. |
| `tessellate_station.go` | `chordStationBound`: the proven enclosure gap of one interior chord station on a circular walk. See the file's own doc comment. |
| `tessellate_loft.go` | `tessellateLoft`: the exact restatement of a `loftPayload`'s held triangle set and proof record. See the file's own doc comments. |
| `tessellate_capblend.go` | `tessellateCapBlend`: the export-only cap-loop chamfer mesh, one chord count per wall walk shared three ways. See `docs/tessellation-reach-design.md` §7. |
| `triangulate.go` | The cap triangulator behind `Tessellate`: hole bridging plus reflex-blocked ear clipping, correct for non-convex outlines with holes. See the file's own doc comment. |
| `export.go` | `Body.STL`/`Body.OBJ`: deterministic writers over `Tessellate`, with `WithChordTolerance`'s documented default. See the file's own doc comment. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks) that double as living documentation. Never `package main`. |
| `_gallery/` | Own nested module, keeping SolidLens out of the library's dependencies: renders every README image under `docs/images`. The `_` prefix hides it from every root-module tool. See its `main.go` doc comment. |
| `.github/workflows/` | `ci.yml` (lint → test/tidy/govulncheck), `codeql.yml`. |
