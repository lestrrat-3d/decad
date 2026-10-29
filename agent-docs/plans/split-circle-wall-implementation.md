# Split-circle wall survey implementation tasks

Status: Unexecuted implementation plan. Source baseline: `5ed2045c6`.
The [design](../split-circle-wall-design.md) owns geometry and admission; this plan is non-authoritative.
Follow [shared execution rules](../exact-arithmetic-task-plans.md) before changing code.
All helper names and signatures marked proposed below are new; existing symbols are named explicitly.

## Navigation

This router is navigation, not authority.

| Question | Section |
|---|---|
| What must be read and preserved? | Task 1 — establish scope and baseline |
| Which real shape proves the change? | Task 2 — create the first public regression |
| How is exact circle identity computed? | Task 3 — implement source-arc identity |
| How do identities reach the reader? | Task 4 — build and consume carrier IDs |
| What blocks expanding the tests? | Task 5 — pass the first real consumer checkpoint |
| Which negative cases matter? | Task 6 — extend focused coverage |
| What finishes the change? | Task 7 — update owners and verify |

## Task 1 — establish scope and baseline

Dependencies: None. Produce a narrow file-change list before edits.

Read `CLAUDE.md`, `agent-docs/testing.md`, `~/.claude/docs/go.md`, and these owners:

- `agent-docs/split-circle-wall-design.md`, all sections.
- `agent-docs/verification-design.md`, wall-survey contract and split-circle restriction.
- `agent-docs/layout/internal-surveys.md`, package responsibility rows.

Read these current implementations and tests:

| File | Existing symbols to inspect |
|---|---|
| `internal/wallsurvey/record.go` | `RevolveWall`, `RevolveLoops`, `RevolveLoopsPlane` |
| `internal/revolvesurvey/survey.go` | `WallReading`, `OnAxisCentre` |
| `internal/survey2d/wall_candidates.go` | `ArcArcCands` |
| `internal/survey2d/wall_geometry.go` | `SurveyElem.Carrier` contract |
| `internal/revolveaxis/frame.go` | `Frame`, `Walk`, direction bounds |
| `internal/sectionrecord/record.go`, `validate.go` | `ArcSeg`, `NormalizeSegment` |
| `internal/proof/distance.go`, `exact_key.go` | `DySquaredDistance3`, `Dyadic.AppendKey` |
| `apitest/survey_sphere_test.go` | `sphereMeridianSketch`, `sphereWallReport`, existing sphere tests |

Run the following existing tests; save output under `.tmp/`:

```sh
go test ./internal/wallsurvey ./internal/revolvesurvey ./internal/survey2d \
  -run '^(TestRevolveLoopsCancellationIsBounded|TestOnAxisCentre|TestArcArcCandsSkipsCarrierPair)$' -count=1
```

Record these observed facts in the implementation notes:

- `RevolveWall` declines a displaced section before resolving loops.
- `WallReading` assigns a new carrier to each circular walk and its mirror.
- `ArcArcCands` already skips equal nonzero carriers with equal material sides.
- `RevolveLoopsPlane` preserves original source indices in each `SideWalk.Segs`.
- There is no `ArcSeg.Partial` field. Whole ranges are exactly `0→1` or `1→0`.

Completion evidence: Existing focused tests pass and the named symbols match this plan.
Stop if baseline tests fail or the source/data contract differs; report the mismatch before adapting the algorithm.

## Task 2 — create the first public regression

Dependencies: Task 1. Add only the split solid ball fixture initially.

In existing `apitest/survey_sphere_test.go`, add proposed test `TestWallSplitSolidBallReadsDiameter`.
Keep package `apitest_test`. Reuse its real Sketch construction helpers:

```go
s, p := sphereMeridianSketch(t,
    [][2]float64{{10, 0}, {0, 10}, {-10, 0}},
    map[int][2]float64{0: {0, 0}, 1: {0, 0}}, nil)
report := sphereWallReport(t, s, p, uAxis, 1.5)
```

The helper fixes every source point, creates two actual quarter arcs plus the diameter line, solves Sketch,
and returns the single profile. `sphereWallReport` calls public `Revolve(FullRevolution{})` and `Verify`.
Do not substitute a constructed `SideWalk` or handwritten payload for this regression.

Require the proposed final result:

1. Require one body, `ScalarMeasured`, and a non-nil `Wall.Minimum`.
2. Convert `Minimum.Value` and `Minimum.Bound` to millimetres with `.In(units.Millimeter)`.
3. Use existing `requireContains(t, big.NewRat(20, 1), value, bound)`.
4. Require the bound at most `1e-9` mm, matching the existing diameter regression's enclosure cap.
5. Require report status `Sound` against the 1.5 mm tool.

Register the proposed test name in shard 10 of `.github/test-shards-apitest.txt` when adding it.
Run just that test before implementation. Current expected result is a failure at `ScalarMeasured`:
the baseline reports `ScalarUndecided` and `Suspect`. Save this bounded failure as regression evidence.
Do not weaken the final assertion to make this run green.

Completion evidence: The real producer reaches the current undecided wall result.
Stop if Sketch, Revolve or Verify fails earlier, or if the baseline already measures 20 mm.
Do not add the shell or a fixture matrix before Task 5 succeeds.

## Task 3 — implement source-arc identity

Dependencies: Task 2 reproduced the intended gap.
Add the following proposed private helper in `internal/wallsurvey/record.go`:

```go
func wholeArcCarrierKey(segment sectionrecord.CurveSegment) (string, bool)
```

The key must describe original plane-local source data; it must not read transformed walk radii or centres.
Use these exact existing operations in this order:

1. Call `sectionrecord.NormalizeSegment(segment)` to accept value/pointer records consistently.
2. If normalization fails, or the result is not `sectionrecord.ArcSeg`, return `"", false`.
3. Accept only `(TStart == 0 && TEnd == 1) || (TStart == 1 && TEnd == 0)`.
4. Call `proof.DyOf` on both `Center.U` and `Center.V`; a false result rejects the key.
5. Compute source start radius squared with `proof.DySquaredDistance3`:
   arguments are `Start.U, Start.V, 0, Center.U, Center.V, 0`.
6. Compute the end radius squared with the same helper and `End`.
7. Require both calls to succeed, the start square's `Sign() > 0`, and `proof.DyCmp(start2, end2) == 0`.
8. Append the three exact values to one byte slice in order: centre U, centre V, start radius squared.
   Use each value's existing `AppendKey` method, then convert the completed slice to `string`.

`AppendKey` is self-delimiting and canonical; do not create a decimal key or concatenate ambiguous byte fields.
Using z=0 with the existing 3D squared-distance helper avoids a new coordinate calculation.
Do not use `math.Hypot`, `RadiusBound == 0`, sampled residuals or angular tolerance as identity proof.

Add proposed direct helper tests in `internal/wallsurvey/record_carrier_test.go`.
Use package `wallsurvey` only for these private-helper tests; the existing external tests remain unchanged.
Direct private access is needed to assert identity decisions separately from the wall kernel.

| New helper test case | Required assertion |
|---|---|
| Two radius-10 quadrants at one centre | Equal nonempty keys |
| Same record with range `1→0` | Same key as `0→1` |
| Value and pointer `ArcSeg` | Same key |
| Different centres or different squared radii | Different keys |
| Radius-1 arc and arc with offsets `e=2^-30` | Different keys despite equal rounded `math.Hypot` radii |
| Unequal start/end squared radii, zero radius, NaN/Inf coordinates | No key |
| Range `0.25→0.75`, whole `CircleSeg`, line or free-form segment | No key |

For the equal-hypot case, use centre `(0,0)` with endpoints `(1,0),(0,1)` for the first arc;
use `(1,e),(e,1)` for the second. The exact squared radii are `1` and `1+2^-60`.
Their keys must differ even though both derived float radii round to 1.

Completion evidence: Proposed helper tests pass and compare only strings, booleans and scalar fields.
Stop if existing proof helpers cannot express the key; do not introduce a tolerance-based replacement.

## Task 4 — build and consume carrier IDs

Dependencies: Task 3. Keep grouping state local to one wall query.
Add proposed private helper in `internal/wallsurvey/record.go`:

```go
func revolveWallCarriers(
    budget *proofbound.WorkBudget, profile momentinput.Profile,
    axis revolveaxis.Frame, loops [][]survey2d.SideWalk,
) ([][]int, error)
```

Use `nil` to mean no shared grouping, and zero at a table entry to mean this walk has no shared ID.
The helper is called only for a full Revolve after the existing `SectionDelta == 0` gate.
Return nil without grouping unless both direction bounds are zero and `(DU,DV)` is one of
`(1,0),(-1,0),(0,1),(0,-1)` exactly. Preserve existing anchor and walk-centre proof requirements.

Build IDs with one `map[string]int`, using first-visit order and positive dense IDs starting at 1:

1. Build the source loop list as outer followed by holes, matching `RevolveLoopsPlane`.
2. Require the source loop count to equal `len(loops)`; otherwise return an internal consistency error.
3. Allocate one output row with exactly the corresponding side-walk count.
4. Poll with `survey2d.WallBudgetErr` before the phase and use `WallBudgetStep` for each loop and walk visit.
5. Only inspect a walk further if `revolvesurvey.OnAxisCentre(w)` is true.
6. Require nonempty `w.Segs`; bounds-check each index against that source loop before reading it.
   An invalid index is an internal consistency error, not permission to assign a group.
7. Charge every source visit with `WallBudgetStep`, then call `wholeArcCarrierKey`.
8. Require every referenced segment to yield one identical key. Unsupported or mixed keys leave the entry zero.
9. Look up that key once in the map; charge this lookup/group decision with `WallBudgetStep`.
10. Reuse its ID or insert the next positive ID, then store it at `[loopIndex][walkIndex]`.

No nested scan over previously seen carriers is allowed. The key map compares complete canonical strings.
All grouping work is linear in visited source segments and walks, apart from hash-table operations.
The same map covers holes; the kernel independently distinguishes material sides.

Change the existing internal reader signature to the following proposed form:

```go
func WallReading(
    budget *proofbound.WorkBudget, loops [][]survey2d.SideWalk,
    carriers [][]int, classify AxisClassifier,
    full bool, phi0, phi1, angularDelta, alpha float64,
) (survey2d.WallReading, error)
```

In `WallReading`, validate a non-nil table before consuming it:

- Require matching loop and row dimensions and nonnegative IDs.
- Meter this scan through the same budget and find the maximum supplied ID.
- Return `fmt.Errorf("revolve wall carrier table: ...")` on malformed data; never truncate a row silently.
  Use the same error prefix for invalid source indices. Tests require `ErrorContains` with that bounded prefix.
- Start fallback allocation at the maximum, so every fresh fallback ID is greater than all supplied IDs.

Add proposed private `nextCarrierID(shared int, last *int) int` in `internal/revolvesurvey/survey.go`:
return `shared` when positive; otherwise increment `*last` and return it.
Its caller invokes it only for an already proven `OnAxisCentre` walk.
In the existing `full`/`OnAxisCentre` branch, select the positive supplied ID when present.
For a zero entry or nil table, increment the fallback allocator and use that fresh ID.
Assign the selected ID to the walk element and its mirror, exactly where the current pair ID is assigned.
Non-on-axis elements keep zero carrier. Partial sweeps keep their existing path and receive a nil table.
Change range loops to retain loop/walk indices when indexing the supplied table.

Change `RevolveWall` to build the table after resolving loops and pass it to the reader.
Propagate any error. There is one current production `WallReading` caller, in `internal/wallsurvey/record.go`;
repeat `rg -n '\.WallReading\(' --glob '*.go'` after the edit to catch any newly added callers.

Preserve every coordinate, angle, material side, radius bound, vertex, pinch check and kernel equation.
Do not modify `RadiusAggregate`, clearance's `RevolveCarrierInput` or shared loop resolution behavior.

Completion evidence: Both packages compile; direct ID tests prove no fallback/shared collision.
Stop on an ID mismatch or cancellation conversion to an undecided result; repair that local plumbing first.

## Task 5 — pass the first real consumer checkpoint

Dependencies: Task 4. Run only `TestWallSplitSolidBallReadsDiameter` from Task 2.

```sh
go test ./apitest -run '^TestWallSplitSolidBallReadsDiameter$' -count=1
```

Require all proposed final assertions to pass through real Sketch → Revolve → Verify.
Confirm that changing only the carrier assignment caused the new measured reading.
Do not force `Exact` or a zero bound; the existing candidate arithmetic may still round.

If the result stays undecided, inspect bounded fields: source indices, eligibility booleans, allocated IDs,
and the final wall outcome. Compare against the design's admission gates.
Stop before adding more fixtures if correct same-carrier IDs do not resolve this real case.
Report the remaining kernel or producer condition and obtain a revised design; do not change candidate equations.

Completion evidence: The one real split ball now reports `Sound` and its interval encloses 20 mm.
Only this checkpoint authorizes Task 6's test expansion within this implementation plan.

## Task 6 — extend focused coverage

Dependencies: Task 5 passed. Extend the public tests in `apitest/survey_sphere_test.go`.
Add proposed `TestWallSplitHollowSphereReadsThickness` using this exact source geometry:

```go
s, p := sphereMeridianSketch(t,
    [][2]float64{{10, 0}, {0, 10}, {-10, 0}, {-8, 0}, {0, 8}, {8, 0}},
    map[int][2]float64{0: {0, 0}, 1: {0, 0}, 3: {0, 0}, 4: {0, 0}},
    map[int]struct{}{3: {}, 4: {}})
```

The final map makes the inner two arcs clockwise. Revolve fully about existing `uAxis`.
Require a measured wall interval containing exactly 2 mm with a bound at most `1e-9` mm.
Require `Sound` for tool 1.5 mm and `Violating` plus `DiagWallTooThin` for tool 2.5 mm.
Extend the solid test with tool 25 mm and the same violating diagnostic.

Add source-order and flat-boundary variants as subtests under these two new public test names:

- Cyclically rotate the point list and remap arc indices; this preserves geometry while changing entity order.
- Split the existing spherical zone's outer arc at `(0,10)`; preserve its flats at u=±6 and require enclosure of 12 mm.
- Keep all original recorded spans. Do not construct a full circle to make the missing-interval case disappear.

Extend proposed `record_carrier_test.go` with these direct table checks:

| Case | Required behavior |
|---|---|
| Same exact carrier split across two walks | Same positive ID |
| Inner and outer skins | Different positive IDs |
| Shared carrier visits in different loops | Same ID; material side remains separate |
| One walk names mixed carriers or unsupported source | Zero ID for that walk |
| Non-cardinal or uncertain direction; off-axis/uncertain centre | No shared admission |
| Empty source list or bad source index | Internal consistency error |
| Pre-canceled budget or injected mid-visit error | Original error propagates |

For mid-visit cancellation, use existing `proofbound.WorkBudget`'s injectable `StepFn` and `ErrFn`.
Return `context.Canceled` from the chosen bounded step and require `ErrorIs`; do not allocate a large profile.
Verify nil-budget compatibility through `survey2d.WallBudgetStep`, which already accepts nil.

In `internal/revolvesurvey/survey_test.go`, add proposed `TestWallReadingCarrierTable`:
cover nil table, all-zero table, malformed dimensions, negative IDs and a mixture of shared/fallback entries.
Compare nil/all-zero results with bounded scalar fields, not a deep graph comparison.
Add proposed `internal/revolvesurvey/survey_carrier_test.go` in package `revolvesurvey` to test `nextCarrierID`.
Start `last=2`; supplied IDs `1,0,2,0` must produce `1,3,2,4` and leave `last=4`.
This direct private test asserts the shared/fallback collision invariant without exporting a test hook.
Keep existing `TestArcArcCandsSkipsCarrierPair` unchanged; it already checks opposite sides and distinct radii.

Run these focused commands after the last code edit:

```sh
go test ./internal/wallsurvey ./internal/revolvesurvey ./internal/survey2d -count=1
go test ./apitest -run '^TestWall(Split|HollowSphere|ThinSphereShell|SolidBall|Hemisphere|SphericalZone|Sphere)' -count=1
go test -race ./internal/wallsurvey ./internal/revolvesurvey -count=1
go test -race ./apitest -run '^TestWall(Split|SphereCentre|SphereTilted|ReflexSweep)' -count=1
```

Keep existing `TestWallSphereCentreOffAxisUndecided` and `TestWallSphereTiltedAxisUndecided` undecided.
Restore the pre-change per-walk ID assignment temporarily, run only the two new public tests, and require failure.
Restore the implemented assignment before final verification. This mutation is small and serial; keep scalar diagnostics.

Completion evidence: Positive intervals and negative admissions match this table in plain and race runs.
Stop on any changed existing negative outcome, unexpected timeout or resource failure.

## Task 7 — update owners and verify

Dependencies: Task 6 passed. Update current-state documentation only for the class actually implemented.

1. Update `agent-docs/verification-design.md`'s wall-survey section to describe shared exact source-arc carriers.
2. Narrow the split-circle row in `agent-docs/missing-features.md`; keep uncertain/unsupported cases listed.
3. Update `agent-docs/split-circle-wall-design.md`'s status to the implemented scope and retain remaining limitations.
4. Check `docs/api.md` and `docs/data-structures.md`; change only descriptions that claim this old restriction.
5. Update the existing package pointer in `agent-docs/layout/internal-surveys.md` only if its responsibility changed.
6. Register both new `apitest` test names in shard 10; subtests need no separate manifest entry.

Run the repository's focused manifest and layout checks:

```sh
go test . ./apitest -run '^TestCI' -count=1
go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1
```

Use the shared execution rules for required lint, PR checks and CI reporting.
Do not run the full suite locally merely to repeat CI.

Acceptance requires all of the following:

- Real split ball/shell outputs enclose 20/2 mm and distinguish passing/failing tool widths.
- Exact keys distinguish the equal-rounded-radius counterexample.
- Missing proof keeps the legacy per-walk fallback and preserves old uncertain-axis outcomes.
- Shared IDs never collide with fallback IDs and never remove opposite-material candidates.
- Span data, topology, radius bounds and candidate equations are unchanged.
- Cancellation propagates and no mutable carrier table survives the query.
- Public test manifests, owner docs and required focused checks agree with the shipped scope.

No cross-repository change or additional module is required. Stop if implementation requires either;
report the prerequisite separately instead of adding it to this change.
