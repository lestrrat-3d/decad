# Exact levels in set-identity certificates

Status: Proposal; unimplemented.

## Scope

Require equal denoted sweep levels before `prismrecord.Payload.SameSet` or
`cupwall.View.SameSet` certifies equal sets. Repair the certificate used by
`Verify`'s overlap-volume shortcut. Do not change public APIs, level producers,
profiles, placement rules or the overlap algorithms used after this shortcut
declines.

This proposal owns the additional level check. The shipped certificate and
volume-consumer contract remain in
[Coincident analytic bodies](interference-design.md#41-coincident-analytic-bodies).
Level-denotation semantics remain in
[Record](stacked-prism-design.md#21-record).

## Observed behavior

The following observations apply to source revision `5ed2045c6`.

| Component | Observed behavior |
|---|---|
| `internal/prismrecord/same_set.go:15` | Compares held `Z0` and `Z1`, frame and placement; rejects section displacement; compares profiles. It never reads axial displacements or denotations. |
| `internal/cupwall/payload.go:393` | Compares held open, outer and cavity levels, frame and placement; rejects offset displacement; compares both profiles. It never reads level displacements or denotations. |
| `internal/verifyrun/interference.go:42` | Calls these methods without an additional axial-level gate. |
| `internal/verifyrun/interference.go:84` | Reads strict containment first. Otherwise, successful set equality returns operand A's existing volume unchanged. |
| `internal/verifyrun/pairs.go:190` | Publishes the returned measurement and applies its tolerance gate. It does not re-establish the set identity or widen the measurement. |

Producers already record the missing information:

- `extrude.go:245` records the held ends, their displacements and
  `prismplacement.DenoteLevel` records in each prism payload.
- `internal/cupwall/payload.go:322` reads all three levels and their denotations
  from the cup's stored slabs.
- `internal/prismplacement/level.go:36` supplies `DenotedLevel`. An undisplaced
  finite level denotes its held float. A displaced level requires a record
  matching both its held float and bound. Missing or stale records return nil.
- `internal/prismplacement/level_range.go` already requires singleton equal
  rational ranges for `LevelRange.SameLevel`. Equal held floats alone are not
  an equality certificate there.

## Verified public counterexamples

The scratch diagnostic `.tmp/same_set_probe_test.go` constructs real public
`Extrude` and `Shell` outputs and passes them to `Document.Verify`.
It computes expected overlap as an exact rational and compares it with the
published scalar interval. It does not replace any producer with a fixture.

Let `h` be the exact rational product of the binary64 inputs `0.2` and `25.4`,
and let `m` be the exact binary64 value held by `units.Millimeters(5.08)`.
`extent.ExactConversion` gives the same `h` for `units.Inches(0.2)`.
Both hold the same float after conversion, but `h < m`.

| Public construction | True overlap in mm³ | Millimetre body created first | Inch body created first |
|---|---|---|---|
| Two coincident-base 1 × 1 mm prisms, heights `m` and `h` | `h` | Publishes `5.08`, `Exact`, bound 0; misses the true overlap by about `7.32747e-17`. | Publishes the same value with bound `7.3274719625260446e-17`, covering the overlap. |
| Two 3 × 3 mm open cups | `8h + 1` | Publishes `41.64`, `Exact`, bound 0; misses by about `5.86198e-16`. | Publishes the same value with bound `5.8619775700208377e-16`, covering the overlap. |

For the first row, the exact positive discrepancy is
`185773484629033 / 2535301200456458802993406410752`.
For the second, it is eight times that discrepancy.
Construct each cup from a prism of height `m` or `h`, shelled inward 1 mm with
its top cap removed. Both prisms have their bases at the origin.
The cup formula follows from outer area 9, cavity area 1 and a 1 mm floor:
`9h - (h - 1) = 8h + 1`.

The public overlap measurements are unsound in the millimetre-first order.
Both solids really overlap; this evidence does not establish a wrong
`Interfering` verdict. Reusing operand A's own bound happens to cover the
difference in the reverse order and does not prove the equality shortcut valid.

Evidence command:

```sh
go test -v .tmp/same_set_probe_test.go
```

The diagnostic passed four cases. Its scalar output is stored in
`.tmp/same-set-public-probe.log`. Passing means the diagnostic ran and printed
the existing behavior; it does not mean the proposed repair is implemented.

## Proposed level predicate

Add one internal helper beside `DenotedLevel` in
`internal/prismplacement/level.go`:

`SameDenotedLevel(aHeld, aDelta, aDenoted, bHeld, bDelta, bDenoted) bool`.

The signature uses the existing float and `LevelDenotation` field types;
the notation above specifies argument order rather than Go source syntax.

The helper must:

1. Read both exact values through `DenotedLevel`.
2. Return false if either reader returns nil.
3. Return true only when the two rationals compare equal.
4. Leave caller-owned denotations unchanged.

Do not compare `LevelDenotation` values structurally. Their private rational
pointers and recorded bounds do not determine rational equality.
Do not accept equal deltas, overlapping ranges, or a small difference as proof.
Do not add a new denotation representation or duplicate the stale-record check.

The helper is a sufficient equality predicate, not an inequality classifier.
False means the shortcut lacks proof; it does not assert that sets differ.

## Proposed payload checks

Keep the existing cheap held-field, frame and placement comparisons. This
proposal does not normalize alternate held representations of one exact set.

After those checks and the existing section/offset-displacement refusal:

- A prism must pass `SameDenotedLevel` independently for `Z0` and `Z1`.
- A cup must pass it independently for `ZOpen`, `ZOuter` and `ZCav`.
- A failed level check returns `(false, nil)` before profile comparison.
- Matching levels proceed to the existing budgeted profile comparison.
- Profile cancellation and budget errors propagate unchanged.

There are at most three endpoint comparisons per call. Rational values are
fresh copies from the existing reader. The helper adds no cache or shared
mutable state. Keep the existing profile walk as the unbounded budgeted work;
do not introduce a per-vertex pass or a separate geometric solver.

Equal denotations may pass even when their displacement bounds differ.
Those bounds describe the held representation's departure and do not change
the exact level the producer proved. A displaced level without a valid
denotation must decline even when compared with an identically stored record.

## Proof and fallback

For an admitted prism pair, exact profile equality and zero section
displacement prove one planar region. Equal denoted endpoint pairs prove one
axial interval. Equal frames and placements map their Cartesian products
onto one world set. Operand A's volume therefore measures the intersection.

For an admitted cup pair, the same argument applies separately to the outer
prism and cavity prism. The three equal denoted planes give identical outer
and cavity intervals. Existing zero-offset and profile gates prove equal
regions, so subtracting the cavity gives identical solids.

When proof fails, `MeasuredInterference` continues to its existing analytic
intersection, overlap-area or mesh path. Those paths may measure a bounded
overlap or return an undecided result. Do not repair the shortcut by copying
the larger bound, selecting the smaller float volume or checking a tolerance.
None of those operations establishes set equality.

## Verification plan

Add bounded scalar tests to the existing test files.

| Test owner | Required cases |
|---|---|
| `internal/prismplacement/level_test.go` | Equal undisplaced values; unequal rationals with one held float; equal rationals with different valid bounds; absent displaced denotation; stale held or bound; non-finite held value. |
| `internal/prismrecord/same_set_test.go` | Exercise each endpoint independently, both operand orders; equal valid denotations pass; missing, stale or unequal denotations decline; existing section and cancellation cases still apply. |
| `internal/cupwall/same_set_test.go` | Exercise each of the three planes independently and both operand orders; preserve profile, offset and cancellation coverage. |
| `apitest/interference_test.go` | Construct the two public counterexamples above in both creation orders; compare every emitted overlap interval against the exact rational oracle. |

For the public regressions, require either a measured interval containing the
exact overlap or the existing explicit undecided outcome. Never require a
particular fallback to succeed just to repair the equality certificate.
If an overlap row is absent, require its pair's undecided diagnostic and
`Suspect` status; absence alone is insufficient. If a row exists, its interval
must cover the oracle and it must not claim `Exact` with zero bound.

Keep positive controls for two equal millimetre bodies and two equal inch
bodies. Equal exact levels must retain the shortcut and reuse operand A's
measurement, including its existing nonzero bound when applicable.
Use a small internal consumer test to distinguish shortcut admission from a
fallback that coincidentally publishes the same numbers.

Compare rationals before formatting. Failure messages report only level
names, scalar readings and the comparison result; never dump bodies or meshes.

Focused implementation commands:

```sh
go test ./internal/prismplacement/ -run 'Test.*Level'
go test ./internal/prismrecord/ -run '^TestSameSet$'
go test ./internal/cupwall/ -run '^TestViewSameSet$'
go test ./apitest/ -run '^TestVerifySameSetDenotedLevels$'
```

The last name is proposed. Register any added root or `apitest` test in its
shard manifest and follow [Test scope](testing.md#test-scope).
Remote CI owns the full suite.

## Implementation slice

Deliver one small correctness PR:

1. Add the shared level predicate and its tests.
2. Apply it to both `SameSet` readers and extend their existing tests.
3. Add the public producer-to-consumer regressions and a narrow shortcut test.
4. Update the owner certificate section to require exact denoted endpoints.
5. Run focused tests and repository-required lint before pushing Go changes.

This design introduces no dependency change or cross-repository work.
The exact fallback outcomes after withholding each bad identity certificate
are unmeasured here. Settle them during implementation with the four public
fixtures before promising that every repaired pair retains a measured row.

## Implementation task plan

Follow the [ordered implementation tasks](plans/same-set-level-implementation.md).
This design owns the proposed contract; the plan supplies execution steps.
