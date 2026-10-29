# Whole-sphere and ring STEP implementation tasks

Status: Unexecuted implementation plan. Source baseline: `5ed2045c6`.
This plan is non-authoritative. [Closed-surface STEP design](../closed-surface-step-design.md) wins on every
geometry, admission and encoding decision. [Shared execution rules](../exact-arithmetic-task-plans.md) own
worktree setup, evidence, Git, CI and escalation. Do not implement this plan while a required gate is blocked.

## Navigation

This table is navigation, not authority.

| Question | Section |
|---|---|
| What must exist before editing the writer? | Task 1 — Establish the independent consumer |
| How are real input bodies constructed? | Task 2 — Freeze the two public fixtures |
| How does export read complete-surface proof? | Task 3 — Add the sphere coverage bridge |
| What must the sphere writer emit? | Task 4 — Build the sphere candidate |
| When may the sphere arm become normal behavior? | Task 5 — Transfer the real sphere |
| How does the torus prove complete coverage? | Task 6 — Add the ring coverage reader |
| What is the torus seam graph? | Task 7 — Build and transfer the ring candidate |
| Which regressions and metadata finish the work? | Tasks 8–10 |

## Starting conditions

- Read repository `CLAUDE.md`, [layout](../layout.md), [testing](../testing.md) and
  [STEP owner](../step-export-design.md), then the owning design linked above.
- Read `~/.claude/docs/go.md` before Go edits and the other pre-reads required by `CLAUDE.md`.
- Preserve the five proposal designs and unrelated work. This plan authorizes no implementation by itself.
- Use existing `export.NewSTEPFile`, `export.STEP`, `Body.Tessellate`, `Face.NormalAt` and AP214 constructors.
- Do not change `go.mod`, the `step` module, imported geometry or Decad's public API.
- Start with the sphere. Do not prepare a torus test matrix before one real sphere transfers successfully.
- Do not use `ReadSTEP` as the independent consumer. Its documented sphere/torus restrictions exclude these outputs.

## Task 1 — Establish the independent consumer

Complete this gate before editing analytic admission. It is currently BLOCKED on a usable local consumer.
The design investigation found no OCCT/OCP/FreeCAD runtime. The planning environment has `python3` and `c++`,
but neither proves an OCCT installation or usable STEP reader.

1. Check the implementation machine for an existing local reader:

   ```sh
   command -v DRAWEXE FreeCAD freecad python3 c++
   pkg-config --modversion opencascade
   ```

   A missing command is expected detection output. Save it; do not treat it as a code failure.
2. Prefer the design's pinned OCCT `V7_9_0` consumer. An already installed different reader/version requires
   a recorded reviewer decision before substituting it for the compatibility gate.
3. Require a supplied C++ harness bundle with `check_step.cxx`, `CMakeLists.txt`, `reader-config.json` and
   `SHA256SUMS` under `.tmp/closed-step-reader/`. The coordinating agent supplies/reviews that bundle against
   OCCT `V7_9_0` headers. Do not substitute a Python binding or choose another consumer on your own.
4. Require the harness contract below. Proposed executable name: `.tmp/closed-step-reader/build/check-step`.
   This executable does not exist in the baseline. The commands below become runnable only after it exists.
5. Record the exact reader build/revision, harness source hash, linked runtime version and transfer options in
   `.tmp/closed-step-reader/version.txt`. Source-code inspection is not an executed compatibility result.

### Consumer harness contract

Implement or obtain one local executable with these proposed arguments:

```text
check-step --input INPUT.step --expect box|sphere|ring --report REPORT.json
```

The harness must read the candidate bytes from Decad, transfer STEP into the reader's native BRep and return
nonzero on parse, transfer, geometry, topology or validation failure. Do not hand-write a STEP test input.
Its report must include all the following independently measured fields:

| Field | Required meaning |
|---|---|
| `reader_version`, `options` | Actual runtime revision and all transfer/healing settings |
| `read_ok`, `transfer_ok` | STEP parsing and native BRep transfer both succeeded |
| `roots`, `solids`, `shells`, `faces` | Root transfer count and unique native topology counts |
| `closed`, `valid` | Native closure and geometric/topological validity checks |
| `surface_kinds` | Underlying native analytic surface types, not STEP text tokens |
| `signed_volume_mm3`, `centroid_mm` | Native mass integration in the file's declared length unit |
| `warnings`, `healing_changes` | Transfer diagnostics and changes made by optional healing |
| `source_sha256` | Hash of the exact Decad-produced file transferred |

Use these native APIs in the supplied C++ harness; its pinned headers must confirm their signatures before build:

- `STEPControl_Reader::ReadFile`, `NbRootsForTransfer`, `TransferRoots`, `OneShape`, `PrintCheckLoad` and
  `PrintCheckTransfer` read, transfer and report diagnostics. Require `IFSelect_RetDone` and a non-null result.
  The [official reader reference](https://occt3d.com/dev/doc/refman/html/class_s_t_e_p_control___reader.html)
  documents that sequence; its current reference is not a substitute for the pinned headers.
- `TopExp::MapShapes` and `TopTools_IndexedMapOfShape` count unique solids, shells and faces. Preserve separate
  edge-use orientation inspection for the ring. The [modeling-data guide](https://occt3d.com/dev/doc/overview/html/occt_user_guides__modeling_data.html)
  documents shape maps.
- `BRepCheck_Analyzer(shape).IsValid()` checks native geometry/topology. Use `BRep_Tool::IsClosed(shell)` for
  free-boundary closure; do not trust the general shape's stored Closed flag alone. See the
  [analyzer reference](https://www.occt3d.com/dev/doc/refman/html/class_b_rep_check___analyzer.html) and
  [BRep_Tool reference](https://www.occt3d.com/dev/doc/refman/html/class_b_rep___tool.html).
- `BRepGProp::VolumeProperties` fills `GProp_GProps`; `Mass()` and `CentreOfMass()` provide signed volume and
  centroid for unit density. Use the analytic integration path, not triangulation. The
  [mass-property reference](https://occt3d.com/dev/doc/refman/html/class_b_rep_g_prop.html) describes these operations.
- `BRepAdaptor_Surface(face).GetType()` identifies reconstructed sphere/torus surfaces. Read the native
  carrier parameters as well; an analytic type name alone does not establish the expected radius/centre.

The bundle's `reader-config.json` must declare the OCCT prefix/config directory, compiler, link libraries,
length unit, transfer/healing options and reader numeric comparison rule. The harness must not invoke an
additional `ShapeFix` pass. Record transfer's own healing behavior/diagnostics instead of claiming it did none.
If that behavior cannot be reported or may have changed coverage, return it for reviewer analysis.

The supplied `CMakeLists.txt` must request OCCT 7.9.0 from that exact config directory, build only `check-step`
and link the supplied installation's STEP, BRep, geometry and property toolkits. No root `go.mod` edit is needed.
After the bundle and CMake/runtime are supplied, use these proposed build commands:

```sh
sha256sum -c .tmp/closed-step-reader/SHA256SUMS
cmake -S .tmp/closed-step-reader -B .tmp/closed-step-reader/build -DOpenCASCADE_DIR=SUPPLIED_CONFIG_DIRECTORY
cmake --build .tmp/closed-step-reader/build --target check-step
.tmp/closed-step-reader/build/check-step --help
```

Replace `SUPPLIED_CONFIG_DIRECTORY` only with the bundle's recorded path. Do not search for and link a different
system installation after version detection fails. A compile error or changed signature goes back to the
coordinating agent with the bounded error excerpt; do not guess wrapper APIs or add library fallbacks.

Completion evidence: Version record, reviewed harness source/build recipe or supplied binary hash, and its
successful invocation/help exist. Task 2 supplies the first real Decad file for harness wiring validation.
Failure response: If the runtime/harness cannot be provisioned, write the missing package/version and harness
contract to the task checkpoint. STOP writer implementation; report the blocked prerequisite to the reviewer.
Do not enable an arm based solely on Part 21 parsing or source inspection.

## Task 2 — Freeze the two public fixtures

Add proposed external-package fixtures to `export/step_analytic_closed_test.go`, next to existing export tests.
Reuse `header()` from `export/step_test.go`; it supplies a fixed timestamp and mandatory header fields.
Use test helpers with `t.Helper()`, `t.Context()` and bounded `require` assertions.

Sphere fixture, equivalent to `apitest/revolve_test.go`'s `semicircleSketch`:

1. `w := sketch.NewWorld()`; `s, err := w.CreateSketch(w.XY())`; check `err`.
2. Create `(0,0)`, `(10,0)` and `(5,0)` points. Fix `(0,0)`.
3. `s.CreateLine(start, end)`; `s.CreateArc(center, end, start)`; solve and require one profile.
4. Revolve using `decad.ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1,0,0)}` and
   `decad.FullRevolution{}`. The axis type is declared in current `revolve.go`.
5. Require one face, zero edges, zero face loops and a `decad.Sphere` tag with centre `(5,0,0)` and radius 5 mm.

Ring fixture, equivalent to `TestRevolveTorus` in that file:

1. Create a new XY Sketch, a fixed centre `(0,10)` and a radius-3 circle.
2. Solve, then Revolve its sole profile about the same world +X axis for a full revolution.
3. Require one face, no edges/loops and a regular ring tag with major/minor radii 10/3 mm.

For each fixture record source `Volume()` and `Centroid()` intervals before export. The exact expectations are
`500*pi/3` mm³ at `(5,0,0)` for the sphere and `180*pi²` mm³ at `(0,0,0)` for the ring.
Use outward arithmetic or an independently high-precision enclosure, not `float64 == exact mathematical pi`.
For reader floating integration, record its independently justified numeric tolerance separately from Decad's
source bound. Require agreement within the combined source interval and reader integration tolerance.
Never use that tolerance to admit a carrier, seam point or topology identity.

Add proposed `TestClosedSTEPConsumerFixture` with `box`, `sphere` and `ring` subtests. Write files only when
`CLOSED_STEP_EVIDENCE_DIR` supplies an absolute evidence directory; skip with a clear message otherwise.
Reject a relative supplied path rather than writing under the export package's test directory. It may emit box and sphere
or ring via `NewSTEPFile` and `step.File.Marshal`; it must not replace the normal correctness tests.
The sphere and ring must always come from the public Sketch/Revolve pipeline above.
The box subtest reuses `box(t, false)` from `step_test.go`, which makes the real 2 × 3 × 4 mm public Extrude.
Transfer `box.step` through the harness before proceeding to Task 3. Require one valid closed solid, positive
volume 24 mm³ and centroid `(1,1.5,2)` under the recorded numeric comparison rule. This checks harness wiring,
not closed-sphere compatibility; unexplained geometric healing fails the gate.

Completion evidence: The box passes the real writer/reader wiring gate; baseline sphere/ring construction and
current faceted export succeed; source scalar intervals are recorded. Current sphere/ring planar fallback is
expected and is not a failed implementation assertion.
Failure response: If the public fixture itself fails or misses the narrow source class, STOP and repair the
fixture description with reviewer input. Do not bypass the producer with hand-built topology.

## Task 3 — Add the sphere coverage bridge

Do not infer full sphere coverage from `Face.Surface()`, zero loop count or successful tessellation.
Add these proposed files; names/signatures below are new, not baseline APIs:

```go
// internal/closedstep/coverage.go; no import of decad or export.
type Kind uint8 // constants Absent, Sphere, Ring
type Coverage struct {
    Kind Kind
    Center, Axis, Reference r3.Vec
    Radius, Major, Minor units.Value
}
var Reader func(context.Context, any, any) (Coverage, bool, error)
func Read(context.Context, any, any) (Coverage, bool, error)
```

`Read` propagates cancellation and otherwise declines when `Reader` is nil. It returns no body pointers,
topology nodes, mutable rationals, document cache or caller-owned slice. The record is call-local.
Keep the zero `Coverage` plus `ok == false` as an unsupported-route result, not a public error.

Add a thin root adapter in proposed `step_closed_surface.go`, following `face_departure.go`'s init pattern:

1. Install the bridge once at package initialization, not per export call.
2. Type-check the arguments as `*Body` and `*Face`; reject nil/foreign handles and require the face to belong
   to the current body's actual face set.
3. Read the current `revolveplan.Payload`; enforce every common admission gate under the owning design's
   "Initial admission". Use `rp.Ax.Snap == (revolveaxis.SnapAllow{})` for the four-field snap check.
4. Require identity `Xform`, one lump/outer shell/face, zero edges/loops, solid/non-sheet and finite geometry.
5. Require exact angular one-turn denotation. `pointquery.fullTurn` is private; do not import it. Add a small
   private predicate beside the proposed coverage reader with the same explicit rational comparison:
   both `Span` nil, all four `Rad`/`Turn` pointers present, equal Rad and exact Turn difference 1.
6. Call existing `contactround.SphereRecord(&body.n)`. Require proof success and `proof.Face == &face.n`.
7. Require `!facedeparture.TagDeparts(&face.n)` and exact proof-centre/radius equality with the current tag.
8. Use `Dyadic.Float64()`'s exactness result and existing proof vector components to convert proof data.
   Any inexact or nonfinite conversion declines; do not round it into the accepted record.
9. Call `rp.ProvesSimple(ctx)` and propagate `ctx.Err()` if cancellation made that boolean return false.
10. Return sphere coverage with world +Z/+X axis/reference. Do not admit a nonidentity placement or retained tag.

The adapter reads private handles, so its file comment must state why root ownership is necessary.
Place numeric helper code that needs only plain data in `internal/closedstep`; keep root glue thin.
Do not add an exported Decad method for this internal export bridge.

Completion evidence: New bridge tests decline absent/foreign input and propagate cancellation; root coverage
tests admit the real Task 2 sphere and reject altered/placed/unproven records. Compare booleans and scalar data.
Failure response: If a proven source cannot pass tag/proof equality, STOP the new arm. Do not weaken
`TagDeparts`, `SphereRecord`, axis bounds or exact point conversion to make the fixture pass.

## Task 4 — Build the sphere candidate

Add proposed `export/step_analytic_closed.go`. Reuse `fileBuilder.add` and `analyticSTEPBuilder` placement helpers.
Choose names `admitClosedSTEP(ctx, body)`, `addClosedSphereFace` and later `addClosedRingFace`; all are new.
Admission returns validated call-local coverage and synthetic points before allocating face entities.

1. Retain `NewSTEPFile`'s existing solid, `Tessellate(VerifyBoundary)`, `checkMesh`, shell and void gates.
2. Attempt closed admission only after the existing body-wide undeparted-tag check.
3. Form the north pole exactly from `C + r*Z`. Use `r3` for held coordinate operations and `internal/proof`
   dyadic operations for the equality check. Require every exact component to equal the held candidate.
4. Call `face.NormalAt(P)`. Require success, zero bound and outward +Z agreement. Do not normalize a rounded
   residual to manufacture the expected normal or evaluate a longitude at the pole.
5. Emit the following graph with deterministic file-local reference allocation:

   ```text
   point = CARTESIAN_POINT('', P)
   vertex = VERTEX_POINT('', point)
   loop = VERTEX_LOOP('', vertex)
   bound = FACE_OUTER_BOUND('', loop, .T.)
   placement = AXIS2_PLACEMENT_3D('', C, +Z, +X)
   surface = SPHERICAL_SURFACE('', placement, r)
   face = ADVANCED_FACE('', (bound), surface, outwardSense)
   ```

6. The pinned step module has no `ap214.VertexLoop`. Use exactly:

   ```go
   step.Entity{Name: "VERTEX_LOOP", Parameters: []step.Value{step.String(""), vertexRef}}
   ```

   Existing `ap214.VertexPoint`, `FaceOuterBound`, `SphericalSurface`, `AdvancedFace` and `ClosedShell`
   constructors are available. Do not add a dependency change for one convenience constructor.
7. Route admitted closed surfaces before `patchWalks` and before `addSphereFace`/`addTorusFace` index a first
   loop. Keep `supportsAnalyticSpherePatch`, `supportsAnalyticTorus` and `isWalkedPatch` unchanged.
8. Ensure admission and generation use the same coverage decision. Thread an optional admission record through
   `assembleSTEPFile` and `addAnalyticFaces`, rather than checking a global cache or trusting a second loose test.
9. On unsupported admission return to the existing whole-body faceted path. On cancellation or real build error,
   return the error; never publish a partially allocated analytic candidate or mix it with faceted faces.

Keep the candidate arm test-only with a local overlay until Task 5 passes. The overlay must call the actual
`NewSTEPFile` pipeline on the real fixture. A manually assembled STEP file does not satisfy this gate.

Completion evidence: Decode the sphere's entity references and prove one spherical face, one vertex loop,
one bound and no edge curves. Assert the pole equals `(5,0,5)` exactly and source counts stay unchanged.
Failure response: Unexpected patch-walk or empty-loop access means dispatch ordering is wrong. Fix ordering;
do not add dummy source loops or disable the mesh gate.

## Task 5 — Transfer the real sphere

Run this first proposed end-to-end command only after Tasks 1–4 exist:

```sh
CLOSED_STEP_EVIDENCE_DIR="$PWD/.tmp/closed-step-e2e" go test ./export -run '^TestClosedSTEPConsumerFixture/sphere$' -count=1
.tmp/closed-step-reader/build/check-step --input .tmp/closed-step-e2e/sphere.step --expect sphere --report .tmp/closed-step-e2e/sphere-reader.json
```

Require one transferred solid, one closed valid shell, one native spherical face, positive signed volume,
the expected centroid and mass agreement under Task 2's documented comparison rule. Require the report hash
to match the generated file. No optional healing may alter coverage or geometry without a separate review.
Save exact commands, hashes, reader version and scalar results. Do not count the baseline planar fallback as
success: the consumer must reconstruct the candidate analytic sphere.

Completion evidence: Real producer output passed through the real candidate writer and independent reader.
Only now enable ordinary sphere admission and add proposed `TestNewSTEPFileWholeSphereAnalytic`.
Failure response: STOP expansion. Report read/transfer/validity diagnostics, bounded topology counts and file hash.
If the reader rejects `VERTEX_LOOP`, request an encoding design revision; do not invent seam topology here.

## Task 6 — Add the ring coverage reader

Begin only after sphere end-to-end success. Add proposed `internal/revolveplan/closed_carrier.go`:

```go
func (rp Payload) CompleteRing(ctx context.Context) (CompleteRingRecord, bool, error)
```

`CompleteRingRecord` is a new plain exact-data record. Store `proof.DyV3` centre, cardinal `r3.Vec` axis and
`proof.Dyadic` major/minor radii; expose fresh/value data only. This is internal computational data, not a new
public measurement API. Reject unsupported data rather than rounding it into the record.

1. Apply the common Full/denotation/identity/zero-departure gates from Task 3.
2. Require one whole `[0,1]` `sectionrecord.CircleSeg`, no holes, finite positive recorded radius.
3. Resolve it exactly as `buildRevolveLoop` does, using `ResolveLoopWithFreeform`, `rp.ChargedWalk`,
   `rp.Ax.SnapTol` and `rp.Ax.RadialLower`. Require `SingleClosed`, one walk and `Kinds[0] == WallTorus`.
4. Read the original radius with `extent.ExactConversion(circle.Radius, units.Millimeter)` from
   `internal/extent/conversion.go`; reject nil/nonpositive and non-dyadic results with `proof.DyOfRat`.
   Compare resolved centre, minor radius and axis distance against original data using exact expressions.
   A held `hypot` or `.In()` result alone proves no equality.
5. Require exact positive axis distance greater than exact radius and `rp.ProvesSimple(ctx)`.
6. Compute the world centre and axis through existing `r3`/exact mapping machinery. Admit only a cardinal axis.
7. Root coverage adapter validates the result against the current face's exact Torus tag and returns bridge data.
8. Pick the first positive perpendicular coordinate reference in X/Y/Z order. No trigonometric basis is needed.

For step 4, lift every stored scalar with `proof.DyOf` and use dyadic arithmetic for
`du = circle.Center.U-AU`, `dv = circle.Center.V-AV`, `z = du*DU+dv*DV`, `rho = dv*DU-du*DV`.
Require exact `z == walk.CU`, `rho == walk.CV`, recorded radius `== walk.Radius`, and `rho > radius > 0`.
Use `proof.DvAdd`/`DvScale` for the exact source-frame expressions
`anchor = origin+U*AU+V*AV`, `axis = U*DU+V*DV`, `centre = anchor+axis*z`.
Compare the held `rp.Basis()` anchor/axis and proposed centre against those exact vectors, then require the
axis to equal a signed coordinate basis vector. These equalities reject rounding; they never repair a walk.
Use the resulting exact `rho` as Major and the exact original radius as Minor, then compare their exact values
with the current Torus tag before returning a bridge record.

Completion evidence: Real ring fixture authenticates; horn/spindle/partial/placed/unknown-source cases decline.
Failure response: If exact circle-to-resolved-walk equality cannot be expressed with current records, STOP and
record the missing proof field. Do not accept equal rounded radii, normalize a nearly cardinal axis or broaden scope.

## Task 7 — Build and transfer the ring candidate

Use Task 4's checked exact-to-held point conversion for `T = C + R*E` and `P = C + (R+r)*E`.
Also require exact representability of scalar `R+r`. Use `r3` for cross products and all held vector arithmetic.

1. Emit one `TOROIDAL_SURFACE` at `C`, axis `A`, reference `E`, radii `R,r`.
2. Emit the outer-equator circle at `C`, axis `A`, reference `E`, radius `R+r`.
3. Emit the tube-meridian circle at `T`, axis `E.Cross(A)`, reference `E`, radius `r`.
4. Emit one shared `VERTEX_POINT(P)` and two `EDGE_CURVE`s, both with that vertex as both endpoints.
5. Emit four `ORIENTED_EDGE`s in order `parallel true`, `meridian true`, `parallel false`, `meridian false`.
6. Emit one edge loop, one outer bound and one advanced face; reuse the ordinary shell/solid assembly.
7. Validate outward sense at `P` against `E` with zero normal bound. Never reverse one seam use independently.

For the +X fixture, `E=+Y`, `T=(0,10,0)`, `P=(0,13,0)`, and meridian axis `E.Cross(A)=-Z`.
Require the two seam IDs each appear once in either orientation. Require unique topology counts
`V=1`, `E=2`, `F=1`, hence Euler characteristic zero. These are structural tests, not consumer acceptance.

Run the Task 5 fixture/reader commands with `ring` substituted for `sphere`. Require native toroidal geometry,
one valid closed solid, positive `180*pi²` volume and origin centroid under the documented comparison rule.
Completion evidence: This second real producer/writer/reader chain passes with recorded input hash and settings.
Failure response: If the reader needs `SEAM_CURVE`/`PCURVE`, STOP ring work and report that precise requirement.
Keep ring fallback. Do not manufacture parameter curves or repair reader behavior within this plan.

## Task 8 — Add bounded regressions

Use proposed `export/step_analytic_closed_test.go` tests. All names in this table are NEW.

| Test | Required assertions |
|---|---|
| `TestNewSTEPFileWholeSphereAnalytic` | Authenticate real sphere; decode complete vertex-loop graph; correct pole/surface/sense |
| `TestNewSTEPFileWholeRingAnalytic` | Authenticate real ring; decode both circles, paired edges, orientation and Euler count |
| `TestClosedSTEPUnsupportedSourceFallsBack` | Placed/reflected/retained-only/unknown/partial sources use whole-body facets |
| `TestClosedSTEPExactPointAdmission` | Inexact pole, tube centre or `R+r` declines; no tolerance-based admission |
| `TestClosedSTEPDeterministicAndReadOnly` | Fixed-header byte equality; original source face/edge/loop counts unchanged |
| `TestClosedSTEPContextAndWriterErrors` | Cancellation propagates; invalid header leaves destination unchanged |

Keep test graphs small. Compare typed scalar entity parameters and individual reference IDs; never pass an
entire body/file graph to a deep-diff formatter. Verify exact carrier refusal with direct admission tests where
a public producer cannot construct the deliberately invalid case.

Retain existing patch, departure, cavity and multi-lump tests. Do not rewrite their expectations for unrelated
bounded-face classes. Export-package test additions do not require root/apitest shard registration.
If root coverage tests add a root test name, register it in `.github/test-shards.txt` as instructed by testing.md.

## Task 9 — Run focused verification

These current baseline commands were verified during planning; save output under the task evidence directory:

```sh
go test ./export -run '^(TestNewSTEPFileBoxTopology|TestAnalyticSpherePatchesFilletedCube|TestAnalyticSTEPRefusesADepartedTag|TestWriteDeterministicAndUntouchedOnInvalidHeader)$' -count=1
```

After adding the proposed code/tests, run:

```sh
go test ./internal/closedstep ./internal/revolveplan -run 'Test.*(Closed|CompleteRing)' -count=1
go test ./export -run '^(TestNewSTEPFileWhole(Sphere|Ring)Analytic|TestClosedSTEP.*)$' -count=1
go test -race ./export -run '^(TestNewSTEPFileWhole(Sphere|Ring)Analytic|TestClosedSTEP.*)$' -count=1
go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1
```

For each focused command require actual matching tests; an empty match is not completion.
Task 1's independent consumer reports remain mandatory even when all Go tests pass.
Run repository-required lint and shard-name checks under the shared rules before any later push.
Do not spend a local full-suite run replacing remote CI.

## Task 10 — Update owners and hand off

Update `step-export-design.md`, the matching `missing-features.md` row and `docs/api.md` to state only the
classes actually enabled after reader acceptance. Update this proposal's status when its admitted class ships.
Add layout rows for new root/internal source ownership under the applicable existing layout part files.
Do not claim reflected/placed spheres, horn/spindle rings or parameter-curve seams shipped if they stayed fallback.

Final acceptance requires both source-coverage proof and native reader acceptance, exact synthetic admission,
unchanged old patch behavior, cancellation/determinism tests, registered files and actual remote CI results.
A sphere-only implementation is complete only as a sphere slice; report the ring as blocked if its gate failed.
Report paths, enabled class, reader revision/results, focused checks and any remaining prerequisite to the parent.
