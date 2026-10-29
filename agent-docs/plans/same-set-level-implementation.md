# SameSet exact-level implementation tasks

Status: Unexecuted task plan. Source inspection uses revision `5ed2045c6`.
The [SameSet design](../same-set-level-design.md) owns semantics and wins over this plan.
Algorithm and fixture details below are non-authoritative execution aids.
Follow [shared execution rules](../exact-arithmetic-task-plans.md) before starting.

## Navigation

This table is navigation, not authority.

| Question | Section |
|---|---|
| What may change? | Scope and file map |
| How do I establish the real failure? | Task 1: Add one public regression |
| What exact predicate do I add? | Task 2: Add the level predicate |
| Where does the prism consume it? | Task 3: Gate prism identity and run the first checkpoint |
| How do I cover cups and every endpoint? | Task 4: Gate cup identity and complete internal coverage |
| What does the complete public matrix assert? | Task 5: Expand the public regression |
| What documentation and checks finish the work? | Task 6: Update contracts and verify acceptance |
| When must I stop? | Stop conditions and completion record |

## Scope and file map

Repair only set-identity admission. Do not change public APIs, producers, profile equality,
placements, volume calculations, tolerance values, or fallback algorithms.

| Existing file | Edit or read |
|---|---|
| `internal/prismplacement/level.go` | Add the proposed shared level predicate beside `DenotedLevel`. |
| `internal/prismplacement/level_test.go` | Add predicate cases using the existing denotation constructors. |
| `internal/prismrecord/same_set.go` | Gate both exact endpoint pairs before comparing profiles. |
| `internal/prismrecord/same_set_test.go` | Extend `TestSameSet`; reuse `squareProfile`. |
| `internal/cupwall/payload.go` | Gate the three levels in `View.SameSet`. |
| `internal/cupwall/same_set_test.go` | Extend `TestViewSameSet`; reuse `squareOf`. |
| `internal/verifyrun/interference.go` | Read `analyticBodiesEqual` and `MeasuredInterference`; leave production code unchanged. |
| `apitest/interference_test.go` | Add proposed `TestVerifySameSetDenotedLevels` and positive controls. |
| `.github/test-shards-apitest.txt` | Register each newly added public test under `testing.md`. |
| `agent-docs/interference-design.md` | Update `Coincident analytic bodies` when the fix passes. |

A new private-consumer test file is proposed at `internal/verifyrun/interference_test.go`.
Do not add a new production file for this small helper.

Read the existing `DenotedLevel` contract, `SameSet` methods and their tests before editing.
Read `apitest/clearance_test.go` for the public Sketch/Extrude fixture style.
Read `TestVerifyCoincidentAnalyticBodiesUseStableFirstVolume` in `apitest/interference_test.go`.
Do not copy its body-slice equality helper into new tests; compare membership by count and pointer identity.

## Task 1: Add one public regression

Create only the millimetre-first, solid-prism case first. It must use real public operations.
Keep the assertion helper local to the test concern until a second caller needs it.

### Task 1.1: Construct the two solids

In proposed `TestVerifySameSetDenotedLevels`, add subtest `prism/mm-first`:

1. Create `doc := decad.New()`.
2. Build a fixed 1 × 1 Sketch rectangle with coordinates `(0,0)` through `(1,1)` on world XY.
3. Create the first prism with `Distance{D: units.Millimeters(5.08), Dir: decad.Along}`.
4. Create an independent, identical Sketch and prism with `units.Inches(0.2)`.
5. Use `sketch.NewWorld`, `CreateSketch`, `CreateRectangle`, `Fix(rect.A)` and `Solve(t.Context())`.
6. Require exactly one profile after solving before indexing `Profiles()[0]`.
7. Use `decad.Extrude(t.Context(), doc, s, s.Profiles()[0], distance)` and check each error.
8. Keep both returned body pointers for pair-identity assertions.
9. Call `doc.Verify(t.Context())` and require no error.

Do not replace either prism with a hand-built internal payload in this public regression.
Do not import `internal/extent` into `apitest`; the expected value must be independent of production conversion.

### Task 1.2: Build the rational oracle

Use `math/big` over the actual binary64 inputs, not decimal rational `1/5` or `127/25`:

```go
inch := new(big.Rat).Mul(
    new(big.Rat).SetFloat64(0.2),
    new(big.Rat).SetFloat64(25.4),
)
millimetre := new(big.Rat).SetFloat64(5.08)
require.Negative(t, inch.Cmp(millimetre))
want := new(big.Rat).Set(inch)
```

For this 1 × 1 profile, `want` is the true intersection volume in mm³.
Both prisms have the same held height, but their denoted heights differ.

For a measured overlap row, compute these exact scalars:

```go
value := new(big.Rat).SetFloat64(row.Volume.Value.Base())
bound := new(big.Rat).SetFloat64(row.Volume.Bound.Base())
residual := new(big.Rat).Sub(value, want)
residual.Abs(residual)
covered := residual.Cmp(bound) <= 0
```

Check finite `Value.Base()` and finite nonnegative `Bound.Base()` before these conversions.
For a positive-infinite bound, require the pair's existing `DiagMeasurementBeyondTolerance` diagnostic
and treat the interval as containing the finite oracle; do not call `SetFloat64` on infinity.
Reject a NaN or negative bound. A coarse but sound fallback is permitted by the design.
Report only subtest name, scalar values, bound and `covered`; never format a body or the full report.

### Task 1.3: Assert a sound report

For the two-body document, require at most one overlap row.
If present, require its `A` and `B` pointers to match document creation order and require `covered`.
The true overlap is not representable as binary64, so `Exact` with a zero bound cannot pass this oracle.

If the overlap row is absent, require `report.Status == decad.Suspect` and a pair-specific unresolved diagnostic.
Inspect `Diagnostic.Pair` before comparing its `A` and `B` pointers.
The existing cause-code set is `DiagUndecidedInterference`, `DiagUndecidedPair`, `DiagUnsupportedPairContact`,
`DiagUnsupportedPairPayload` and `DiagUnsupportedPairPipeline`.
Require `ReadingNone` and `Suspect` on the matching diagnostic.
Resolve the actual emitted code through `boundaryaudit.UndecidedPairReason` during Task 3 and pin it for the fixture.
Do not accept an arbitrary diagnostic or a measured `DiagInterference` as an unresolved outcome.
Never accept an empty row list, `Sound`, or an unrelated body diagnostic as proof of a safe fallback.

Record whether `Verify` preserves both live body pointers in the same order.
Compare `len(doc.Bodies())` and individual pointer identities; do not deep-compare body graphs.

### Task 1.4: Run the baseline once

```sh
go test ./apitest -run '^TestVerifySameSetDenotedLevels$/prism/mm-first$' -count=1
```

The proposed new test should fail on the unmodified source because the measured interval misses `want`.
The design's known baseline is `Value = 5.08`, bound zero, `Exact`.
This expected baseline failure is evidence, not a reason to alter the oracle or repair a fallback.
If the fixture errors, changes payload class unexpectedly, or already passes for a different reason, stop.
Record the source revision and scalar evidence for the parent to reconcile before continuing.

Completion evidence: a real Extrude→Verify regression reaches the known bad interval.

## Task 2: Add the level predicate

Add this proposed function to `internal/prismplacement/level.go`:

```go
func SameDenotedLevel(
    aHeld, aDelta float64, aDenoted LevelDenotation,
    bHeld, bDelta float64, bDenoted LevelDenotation,
) bool
```

Implement exactly this decision order:

1. Call `DenotedLevel(aHeld, aDelta, aDenoted)`.
2. Return false if the result is nil.
3. Call `DenotedLevel(bHeld, bDelta, bDenoted)`.
4. Return false if the result is nil.
5. Return whether the two rationals compare equal with `Cmp`.

Do not reimplement record validation, compare private pointers, mutate either input record, or use an epsilon.
Do not add a `math/big` operation on a nil result.
The existing reader already copies its rational and detects a displaced stale held value or bound.

Add proposed `TestSameDenotedLevel` in the existing external-package test file.
Use `DenoteLevel`, not struct literals with inaccessible fields, to create evidence.

| Case | Fixture | Expected predicate |
|---|---|---|
| Equal undisplaced values | Both `(7, 0, zero record)` | True |
| Unequal undisplaced values | Held 7 versus 8 | False |
| Equal displaced denotations | `inch`, same held float, records at bounds `1e-15` and `2e-15` | True |
| One held float, unequal exact values | Recorded `inch` versus undisplaced `millimetre` | False |
| Unknown displaced value | Positive delta and zero record | False |
| Stale held value | Change held with `math.Nextafter` after recording | False |
| Stale displacement | Change delta from `1e-15` to `2e-15` without recording again | False |
| Non-finite held value | NaN or either infinity, zero delta and zero record | False |
| Ownership | Call repeatedly, then read both original denotations | Both compare equal to their original rationals |

Exercise false cases in both operand orders. The predicate is symmetric.
Keep `TestDenotedLevel` unchanged except shared setup that directly avoids duplication.

```sh
go test ./internal/prismplacement -run '^(TestDenotedLevel|TestSameDenotedLevel)$' -count=1
```

Completion evidence: all predicate cases pass without changing existing reader semantics.
If an existing reader contract differs from the design, stop; do not widen `DenotedLevel` in this task.

## Task 3: Gate prism identity and run the first checkpoint

Import `internal/prismplacement` into `internal/prismrecord/same_set.go`.
Keep its existing frame, held-level, placement and section-displacement checks in their current order.
Before `momentinput.ExactProfileEqual`, add one predicate call per endpoint:

- `Z0`, `Z0Delta`, `Z0Denoted` against the corresponding fields of `other`.
- `Z1`, `Z1Delta`, `Z1Denoted` against the corresponding fields of `other`.

Return `(false, nil)` if either call fails. Do not return an error for missing proof.
Keep the existing budget and profile-error behavior on the admitted path.

Extend `TestSameSet` with independent tests for both endpoints using existing `base` and `squareProfile`.
Do not change both endpoints in one test; each endpoint must independently protect admission.
Set one held endpoint to the common rounded inch height, then vary its exact denotation or validity.
For the other endpoint, keep an ordinary equal, undisplaced value.
For the lower-endpoint test, choose the other end above the changed level so the interval remains ordered.

Require unequal, unknown and stale denotations to decline in both operand orders.
Require equal denotations at different valid bounds to pass.
Preserve existing section-displacement and `spent` budget-error tests.

Run the internal test, then the first real public case:

```sh
go test ./internal/prismrecord -run '^TestSameSet$' -count=1
go test ./apitest -run '^TestVerifySameSetDenotedLevels$/prism/mm-first$' -count=1
```

Handle the public outcome explicitly:

| Outcome | Action |
|---|---|
| A measured interval contains the oracle | Record value, bound and exactness; continue. |
| No row, `Suspect`, correct pair's existing unresolved cause | Pin that cause in the regression; continue. |
| Wrong interval, `Sound` without a row, unexpected error or failure elsewhere | Stop expansion and report the consumer path. |

Read `internal/boundaryaudit`'s `UndecidedPairReason` if a fallback cause needs identification.
Do not change `MeasuredInterference`, its fallback order, bounds or tests to force a measured result.

Completion evidence: the real first producer-to-consumer case is sound after the narrow prism change.

## Task 4: Gate cup identity and complete internal coverage

`internal/cupwall/payload.go` already imports `prismplacement`; reuse that import.
Inside `View.SameSet`, preserve cheap comparisons and the offset-displacement refusal.
Before the outer profile comparison, require the predicate for these three independent tuples:

- `ZOpen`, `ZOpenDelta`, `ZOpenDenoted`.
- `ZOuter`, `ZOuterDelta`, `ZOuterDenoted`.
- `ZCav`, `ZCavDelta`, `ZCavDenoted`.

Return `(false, nil)` for any failed tuple and leave both profile comparisons unchanged.
Extend `TestViewSameSet` using `squareOf` and its existing frame/placement setup.
For each changed tuple, keep the other two equal and choose ordered open/cavity/outer levels.
Require unknown, stale and unequal denotations to decline in both orders.
Require equal valid denotations with different bounds to pass.
Preserve offset, profile and spent-budget tests.

Add proposed private `TestAnalyticBodiesEqualDenotedLevels` in `internal/verifyrun/interference_test.go`.
Use `package verifyrun` only because this test exercises the unexported eligibility function.
Build minimal `topo.Body` nodes with prism payloads or with actual `cupwall.Payload` values and call
`analyticBodiesEqual` directly. A `cupwall.View` is not one of this consumer's accepted payload types.
Construct each cup payload through `view.Payload(t.Context())` and check its error before storing it on a node.
For that payload conversion, use exact outer rectangle `[0,10] × [0,10]` and strictly nested cavity
`[2,8] × [2,8]`, an identity XY frame/placement, `ZOuter = 0`, `ZCav = 1`, `ZOpen = 4`.
Create their four constant line records directly in the internal test. Do not reuse origin-sharing outer/cavity
profiles from an equality-only fixture: this conversion must build a valid shell band.
Assert only its boolean/error result for equal and unequal denotations.
This unit test may use small recorded squares; Task 5 owns the real public construction boundary.

```sh
go test ./internal/cupwall -run '^TestViewSameSet$' -count=1
go test ./internal/verifyrun -run '^TestAnalyticBodiesEqualDenotedLevels$' -count=1
```

Completion evidence: all five endpoint positions are independently covered, including consumer eligibility.

## Task 5: Expand the public regression

Only after Task 3 passes its first real checkpoint, add the remaining cases to the same proposed public test:

| Subtest | Creation order | Construction | Exact overlap oracle |
|---|---|---|---|
| `prism/mm-first` | mm, inch | 1 × 1 rectangular prisms | `inch` |
| `prism/inch-first` | inch, mm | Same prisms | `inch` |
| `cup/mm-first` | mm, inch | 3 × 3 prisms shelled inward by 1 mm, top removed | `8*inch + 1` |
| `cup/inch-first` | inch, mm | Same cups | `8*inch + 1` |

For each cup, call public `decad.Shell` on that prism with
`decad.Faces(decad.FaceCreatedBy(decad.CapEnd(prism)))` and `units.Millimeters(1)`.
Keep the returned cup pointer; the shell consumes its prism. Query the two live cups with `doc.Verify`.
The cavity is 1 × 1 and its floor is 1 mm, giving `9h - (h - 1) = 8h + 1`.
Build `want` using fresh rational receivers, leaving the shared `inch` oracle unchanged.
Use the same measured-or-explicitly-undecided assertion from the first checkpoint.

Add positive controls for two mm prisms, two inch prisms, two mm cups and two inch cups.
For these equal denotations, require one overlap row and compare its value, bound and exactness with the first
body's `Volume()` reading. Preserve document pair order with pointer assertions.
Do not require zero bound for inch controls; their original measurement already carries conversion error.

Run the new test and the existing stable-first-volume control:

```sh
go test ./apitest -run '^(TestVerifySameSetDenotedLevels|TestVerifyCoincidentAnalyticBodiesUseStableFirstVolume)$' -count=1
```

Completion evidence: all four unequal cases are sound, equal cases retain measurement reuse, and live membership is unchanged.

## Task 6: Update contracts and verify acceptance

Update `Coincident analytic bodies` in `agent-docs/interference-design.md` to name exact denoted endpoint checks.
Update the two `SameSet` doc comments to state their endpoint proof requirement.
Update proposal status in `same-set-level-design.md` only after its implementation and focused checks pass.
Do not add a missing-feature row for a corrected internal soundness defect or claim broader equality normalization.

Register the proposed public test in `.github/test-shards-apitest.txt` under the repository test-scope rules.
No root-package test name is added, so `.github/test-shards.txt` needs no entry for this plan.
No new root production file is proposed, so no new root-file layout row is required.

Run the following focused packages after the final edit, saving outputs under `.tmp/`:

```sh
go test ./internal/prismplacement ./internal/prismrecord ./internal/cupwall ./internal/verifyrun
go test ./apitest -run '^(TestVerifySameSetDenotedLevels|TestVerifyCoincidentAnalyticBodiesUseStableFirstVolume)$' -count=1
go test . ./apitest -run '^TestCI'
go test . -run '^TestCLAUDEMDLayoutStaysCompact$'
```

Use the shared execution rules for required lint, publication and remote CI.
Do not report the local focused commands as a full-suite result.

## Stop conditions and completion record

Stop if a valid acceptance fixture unexpectedly lacks its required denotation or differs from the documented defect,
or withholding the bad certificate uncovers a fallback failure outside this scope.
Keep the narrowed admission fix; record the failed command, scalar mismatch and exact consumer path for review.
Do not silently widen this plan to another numerical mechanism.

Before handoff, record changed files, focused commands/results, the four public outcomes, positive-control results,
and any remote CI result actually obtained. State any undecided fallback explicitly.
Completion requires no public measured overlap outside its exact oracle interval and no equality admission from
an unknown, stale or unequal endpoint denotation.
