# Decadtest Design

`decadtest` is the package a test author calls instead of reading decad's struct definitions.
Every helper takes `testing.TB` first, calls `tb.Helper()`, stops the test with `tb.Fatalf` on
failure, and prints the geometric quantity, the reading decad gave, and the value the author
expected. It applies decad's comparison rules where a `testify/require` call would compare two
floats. Companion to `docs/api-design.md` (the surface it wraps, "core §N") and
`docs/verification-design.md` (the report its verification helpers read, "verification §N").

## 1. Scope and non-goals

The kit covers the calls a decad test repeats: reading a body's volume, area, centroid and
bounds and comparing each against a known answer; checking that a body is a manifold solid
with the expected face kinds; running `Verify` and asserting on its report, its per-body
records, its diagnostics and its pair rows; reading the wall, concave-radius and undercut
surveys; and building the one-plate, one-block bodies most tests start from. It serves decad's
own `_test` packages and the gear generator's `proof/` module on the same terms.

It does not prove sketches (constraint closure, DOF, ambiguity), which is `sketch`'s verdict
and the gear repository's `proofkit` job. It does not run case tables. It never reads a mesh:
`Body.Tessellate`, `STL` and `OBJ` stay uncovered so that no test grows a dependency on
triangle structure (`docs/api-design.md` §3 invariant 1). It does not wrap `errors.Is`,
`SelectEdges`/`SelectFaces` or recipe round-trips: each of those is already one `require`
line, and the kit's job is the comparison that is NOT one line today.

The kit's usage record is its doc comments and its own tests, not an `examples/` entry. An
`Example*` function has no `testing.TB` to hand a helper, so an example would have to build
a recording TB before it could demonstrate a single call.

## 2. Placement and module

**`decadtest/` is a package of the root module, `github.com/lestrrat-3d/decad/decadtest`.**

Three facts decide it:

- The gear generator imports decad by pseudo-version in `proof/go.mod`, and its CI checks out
  decad at that pin and refuses any other revision. A nested module would need a second pin,
  a second `go get`, and a second revision check in `proof/run.sh` and `3d-proof.yml`. In the
  root module the existing pin covers the kit, and a decad bump that changes a helper's
  contract fails the proofs in the same PR that raises the pin, which is the gear repository's
  stated rule for engine bumps.
- The `_`-prefix rule (`CLAUDE.md`, "Tooling lives in its own nested module") exists for two
  things: keeping SolidLens out of the library's dependency graph, and hiding generators from
  `go list ./...`. The Go tool ignores every `_`-prefixed directory, so a `_decadtest` could
  not be imported by anyone. The rule is about tooling nobody imports; a package whose whole
  purpose is to be imported falls outside it. This design says so rather than assuming it.
- The kit adds no dependency. It imports `decad`, `units`, `r3`, `sketch` (for the fixtures),
  `lestrrat-go/option/v3` (for its options) and the standard library. All five are already in
  the root `go.mod`.

**`decadtest` is production code for the testify rule.** It is a non-`_test` package that
`go build ./...` compiles and that an importer's non-test code could link. Importing
`testify/require` from it would make testify a non-test dependency of the root module, which
the approved-modules list forbids. So the kit fails through `tb.Fatalf` alone and formats its
own messages; only its own `decadtest_test` files use `require`. This also means golangci-lint
runs its full production ruleset on the kit (no `_test.go` exclusions), which is the right
level of scrutiny for code every test trusts.

One row joins `docs/layout.md`'s Repository table, beside `examples/`:

| Path | Responsibility |
|---|---|
| `decadtest/` | The public test kit: comparison helpers over decad's three bounded readings, bodies, reports and surveys, plus the sketch-to-body fixtures. Standard `testing` only, never testify. See `decadtest/doc.go`. |

CI needs one edit. On Ubuntu the root package runs in race shards and every package outside
the root runs on the runner that carries a `packages:` operand, so `./decadtest/` joins
`./examples/` in that operand in `.github/workflows/ci.yml`. `ci_workflow_test.go` fails with
"runs in no race shard" until it does. The non-Linux leg runs `go test ./...` and reaches the
package with no edit, and the root-package shards are unaffected either way.

## 3. The comparison model

decad returns exactly three bounded shapes — `Measurement`, `VecMeasurement`, `Box` — and
every other value a test asserts on is a decided predicate, a count, an enum, or a record built
from those three. The kit therefore needs one comparison rule, stated once, and applied three
times.

### 3.1 A measurement is a proven interval, and the test author's number is a second claim

A `Measurement{Value: V, Bound: B}` is decad's claim that the true quantity lies in
`[V − B, V + B]`. The author's `want` is a second claim, `W ± s`, where `s` is the error of the
author's OWN oracle — a closed-form formula evaluated in float64, a hand computation, a value
read from another body. The two claims are consistent exactly when

```
|V − W| ≤ B + s          (all four terms in base units of the same Kind)
```

That is the whole rule, and every scalar helper in the kit applies it. Its consequences are
the design decisions:

- **`B` is read from the measurement, never supplied by the test.** No helper takes an
  expected bound. A bound differs between amd64 and arm64 (FMA), so a test that pinned one
  would be architecture-specific; a test that reads it is not. The only equality ever asserted
  on a bound is zero, and only through `Exactly()` (below), because `Exact` is a proof, not a
  float.
- **`s` is about the author, never about decad.** It defaults to `1e-12 × |W|` — the rounding
  of a float64 oracle across a handful of operations, roughly fifty ulps — and is overridden
  with `Within(units.Value)` (absolute, the measurement's own Kind) or
  `WithinRel(units.Value)` (Dimensionless, a fraction of `|W|`). A default of zero would fail
  the most common real call in the gear proofs, `math.Pi*r*r*h` against a circular prism whose
  own bound is a rational π enclosure far tighter than the oracle's float rounding. A fixed
  absolute default is Kind-blind and scale-blind, which `docs/verification-design.md` §2
  already rules out for the verifier's gate. A relative default is scale-invariant and states
  exactly one assumption, which the doc comment names.
- **Exactness is metadata, not a second gate.** `Exact` means `B = 0`, so the rule reduces to
  `|V − W| ≤ s`, and the author's slack still applies: an Exact reading is the truth, and the
  author's float oracle can still be off by rounding. A test that wants to assert exactness
  passes `Exactly()`, which additionally requires `Exactness == Exact` and `B == 0`, and fails
  with "reading is Approximate" when it is not. The kit also falsifies decad's own invariant
  for free: an `Exact` reading with a nonzero bound fails every comparison, whatever the
  values.
- **Kind is checked before magnitude.** `want` is a `units.Value`; if its `Kind` is not the
  measurement's, the helper fails with the two Kinds named and never compares numbers. This is
  why tolerances and expectations are `units.Value` and not `float64`: `Within(units.Millimeters(1e-6))`
  handed to a volume reading is a test bug the helper reports, where a bare `1e-6` would
  silently compare mm against mm³. `units.Value.Equal(o, tol)` does the arithmetic — it is
  exact in rationals where float64 would lose the difference, and it already refuses a Kind
  mismatch — and the kit passes it `tol = B + s` in base units. The one `float64` inside the
  kit is that argument, which never crosses the public API.
- **Comparing two readings** uses the same rule with both bounds: `Agree(a, b)` holds when
  `|Va − Vb| ≤ Ba + Bb + s`. This is what "the patterned tooth has the seed's volume" means
  when both are measured.
- **Precision is asserted as a ceiling with headroom, never a value.** `BoundAtMost(bound,
  limit)` checks `B ≤ limit` for a `limit` of the same Kind. The doc comment tells the author
  to leave orders of magnitude between the ceiling and any bound they have observed. A
  relative form (`B ≤ rel × |V|`) is deliberately not offered: the verifier's gate anchors
  `rel` on a per-body reference with a noise floor the kit cannot derive, and a test that wants
  that judgement should ask `Verify` for it.

### 3.2 Vectors

`VecMeasurement{Value: V, Bound: B}` is a ball of radius `B` around `V`, so the rule is
`|V − W|₂ ≤ B + s`, with `W` an `r3.Vec` (the §5.2 coordinate carve-out) and the distance
taken by `r3`. `B.Kind()` is Length for a position and Dimensionless for a direction; the kit
reads it from the measurement, prints it, and compares `B.Base()` against a millimetre
distance for a position or a unit-vector deviation for a direction. Default `s` is
`1e-12 × |W|₂`; `Within` takes a Length for a position and a Dimensionless `units.Value`
(`units.Scalar`) for a direction, and the Kind check catches the wrong one. `Exactly()` applies as above.

### 3.3 Boxes

`Box{Min, Max, Bound: B}` is checked per corner coordinate: each of the six components is
within `B + s` of the expected one, with `s` defaulting to `1e-12` of the box's own diagonal
length. `Encloses(box, p)` is the other useful reading — `p` lies inside the box grown by `B` —
and it is what "the tooth reaches the tip radius" actually asserts.

### 3.4 Everything else is a decided answer

`IsSolid`, face/edge/lump counts, `Exactness`, `Status`, every outcome enum, and
`Diagnostic.Code` are answers, not approximations (`docs/api-design.md` §6). The kit compares
them with `==` and spends its effort on the failure message: a wrong `Status` prints every
diagnostic in the report, a wrong outcome prints the survey's own local diagnostics, and a body
is always named by its recipe step (`body[2] (step 3 union)`) rather than a pointer.

## 4. API surface

All helpers: `tb testing.TB` first, `tb.Helper()`, `tb.Fatalf` on failure, nil arguments
rejected at the exported boundary with a message naming the argument. `what` is the label the
message opens with. Options are `lestrrat-go/option/v3` values of type `decadtest.Option`.

### Options

| Signature | Purpose |
|---|---|
| `Within(v units.Value) Option` | Absolute oracle slack `s`, same Kind as the reading. |
| `WithinRel(rel units.Value) Option` | Relative oracle slack, `s = rel × |W|`; `rel` Dimensionless. |
| `Exactly() Option` | Also require `Exactness == Exact` and a zero bound. |

### Readings — the comparison model on the three shapes

| Signature | Purpose |
|---|---|
| `Measures(tb, what string, got decad.Measurement, want units.Value, opts ...Option)` | `|V−W| ≤ B+s` after a Kind check. Works on any `Measurement`: body, face area, edge length, gap, overlap. |
| `MeasuresVec(tb, what string, got decad.VecMeasurement, want r3.Vec, opts ...Option)` | Ball rule for a position or direction: centroid, vertex position, face normal. |
| `BoundsAre(tb, what string, got decad.Box, min, max r3.Vec, opts ...Option)` | Six-coordinate rule on a box. |
| `Encloses(tb, what string, got decad.Box, p r3.Vec)` | `p` lies within the box grown by its bound. |
| `Agree(tb, what string, a, b decad.Measurement, opts ...Option)` | Two readings' proven intervals meet: `|Va−Vb| ≤ Ba+Bb+s`. |
| `BoundAtMost(tb, what string, bound, limit units.Value)` | A bound stays under a stated ceiling; takes the `Bound` field of any shape. |

### Bodies — fetch and compare

| Signature | Purpose |
|---|---|
| `Volume(tb, body *decad.Body, want units.Value, opts ...Option)` | `body.Volume()` with the error handled, then `Measures`. |
| `Area(tb, body *decad.Body, want units.Value, opts ...Option)` | Same for `Area()`. |
| `Centroid(tb, body *decad.Body, want r3.Vec, opts ...Option)` | Same for `Centroid()`, through `MeasuresVec`. |
| `Bounds(tb, body *decad.Body, min, max r3.Vec, opts ...Option)` | Same for `Bounds()`, through `BoundsAre`. |
| `Manifold(tb, body *decad.Body)` | Every edge bounds exactly two faces; every face has exactly one outer loop. |
| `SurfaceKinds(tb, body *decad.Body, want map[decad.SurfaceKind]int)` | Face count per surface kind equals `want`; a kind absent from `want` must be absent from the body. |

### Verification — reports, bodies, diagnostics, pairs

| Signature | Purpose |
|---|---|
| `Verify(tb, doc *decad.Document, opts ...decad.VerifyOption) *decad.Report` | `doc.Verify(tb.Context(), opts...)` with the error handled. |
| `Sound(tb, doc *decad.Document, opts ...decad.VerifyOption) *decad.Report` | `Verify` and require `Passed()`; on failure prints every diagnostic. |
| `Status(tb, report *decad.Report, want decad.Status)` | Report-level status, with the diagnostic list on mismatch. |
| `BodyReport(tb, report *decad.Report, body *decad.Body) *decad.BodyReport` | `report.ForBody` with the error handled and the body named. |
| `Valid(tb, report *decad.Report, body *decad.Body) *decad.BodyReport` | `Validity.Outcome == ValidityValid`, one lump, no voids; tolerance diagnostics do not fail it. |
| `Diagnosed(tb, report *decad.Report, code decad.DiagnosticCode) []decad.Diagnostic` | At least one diagnostic carries `code`; returns them for field checks. |
| `OnlyDiagnostics(tb, report *decad.Report, allowed ...decad.DiagnosticCode)` | Every diagnostic's code is in `allowed`; no arguments means none at all. |
| `Clearance(tb, report *decad.Report, a, b *decad.Body, want units.Value, opts ...Option)` | Finds the pair's `Clearance` row (either order) and `Measures` its `Gap`. |
| `Interference(tb, report *decad.Report, a, b *decad.Body, want units.Value, opts ...Option)` | Finds the pair's `Interference` row and `Measures` its `Volume`. |

### Surveys — per-body results

| Signature | Purpose |
|---|---|
| `WallMinimum(tb, br *decad.BodyReport, want units.Value, opts ...Option)` | `Wall.Outcome == ScalarMeasured` and `Measures` on `Wall.Minimum`. |
| `ConcaveRadius(tb, br *decad.BodyReport, want units.Value, opts ...Option)` | Same on `ConcaveRadius`. |
| `UndercutFaces(tb, br *decad.BodyReport, n int) []*decad.Face` | `Undercut.Coverage == CoverageComplete` and exactly `n` opposing faces; returns them. |

### Fixtures — the sketch-to-body path

| Signature | Purpose |
|---|---|
| `Sketch(tb) *sketch.Sketch` | An empty sketch on a fresh world's XY plane. |
| `Region(tb, s *sketch.Sketch) *sketch.Profile` | Solves `s` and returns its single valid closed region; fails on any other count. |
| `Prism(tb, doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, height units.Value) *decad.Body` | `doc.Extrude` by `Distance{D: height, Dir: Along}` with the error handled. |
| `Block(tb, doc *decad.Document, x0, y0, x1, y1 float64, height units.Value) *decad.Body` | Rectangle on XY (coordinates in mm, the §5.2 carve-out) extruded by `height`: the fixture most tests start from. |

`Block`'s four coordinates are millimetres under `docs/api-design.md` §5.2's coordinate
carve-out, and the rectangle is grounded at corner A (`x0, y0`) so the sketch it builds
solves fully constrained.

That is 27 helpers and 3 options.

### Deliberately uncovered in v1

- **Counts** (`len(body.Faces())`, `Topology.Lumps`) and **enum fields** (`Exactness`,
  outcomes) beyond the helpers above: `require.Equal`/`require.Len` is already one line and
  prints both values. `SurfaceKinds` exists because the loop it replaces is not.
- **Reading getters** (`ReadVolume` returning a `Measurement`): `body.Volume()` plus
  `require.NoError` is two lines, and a getter that returned `.Base()` would be a bare float
  crossing a public API. `Agree` covers the measured-against-measured case that made the gear
  proofs write `volumeOf`.
- **Per-entity helpers** (`FaceArea`, `EdgeLength`, `VertexAt`, `NormalAt`): each returns one
  of the three shapes, so `Measures`/`MeasuresVec` already apply directly.
- **Selectors**: `Edges(...).Exactly(n).SelectEdges(body)` already asserts cardinality through
  its error; the kit would add only `NoError`.
- **Errors**: `require.ErrorIs`/`ErrorAs` handle sentinels, `BooleanError`, `SelectionError`
  and `RecipeError` today.
- **Meshes and export**: never, for invariant 1.
- **Relative precision on vectors and boxes**: needs the owning body's diameter; ask `Verify`.
- **Sketch proving**: `sketch`'s verdict, the gear repository's `proofkit` job.

## 5. Failure messages

Format: `<what>: <what the reading is> ... <what was expected>`, then one indented line saying
by how much it missed and what was allowed. Readings render through `units.Value.String()` and
`r3.Vec`'s `%v`, so the numbers are the ones the author would print themselves. A body is named
`body[i] (step N op)` from `Report.Bodies` order or `Document.Bodies()` order, plus
`Body.Origin().Step` and the recipe step's `Op`.

Scalar miss (a chorded tooth compared against the unchorded oracle):

```
tooth volume: reading 26.3140891 mm^3 ± 0 mm^3 (Exact) does not enclose expected 26.3187345 mm^3
  off by 0.0046454 mm^3; allowed bound 0 mm^3 + slack 2.6e-11 mm^3 = 2.6e-11 mm^3
  (slack is the default 1e-12 relative; state your oracle's error with Within or WithinRel)
```

Kind mismatch (a millimetre slack handed to a volume):

```
bored Gear Body volume: Within slack is a length (mm) but the reading is a volume (mm^3)
```

Sound failing, with the bodies named:

```
verify: report is Suspect, want Sound; 2 diagnostic(s):
  [1] body[0] (step 3 union) measurement_beyond_tolerance area: 1256.63706 mm^2 ± 0.0213 mm^2 (Approximate), required bound ≤ 0.00125664 mm^2
  [2] pair (body[0] (step 3 union), body[1] (step 4 extrude)) undecided_pair: the partition proof resolved neither way
```

Box miss (only the offending coordinate is called out):

```
bored Gear Body bounds: reading min {-7 -7 0} max {7 7 3.5} ± 0 mm (Exact); want min {-7 -7 0} max {7 7 3}
  max.Z off by 0.5 mm; allowed bound 0 mm + slack 1.1e-11 mm
```

## 6. Supersession of proofkit and proofkit3d

The current `proofkit3d` is written against a decad older than HEAD (`Report.Trustworthy`,
`BodyReport.Solid/Watertight/Manifold`), so it does not compile against the decad that will
ship `decadtest`; the migration lands in the same PR as the pin bump the gear repository
requires for any engine change. `proofkit` (sketch side) compiles as is.

**Fate legend.** *moves*: the behaviour is provided by `decadtest` and the gear symbol is
deleted or becomes a one-line call. *stays*: gear- or proof-methodology-specific; unchanged.
*drops*: redundant after the move; deleted.

### proofkit (sketch proofs)

| Symbol | Fate | Why |
|---|---|---|
| `Case`, `Build`, `Run`, `RunParallel`, `ParallelGroup` | stays | A case table of Fusion dialog values over a sketch; nothing in it touches decad. |
| `ExpectedFailure`, `ExpectedFailureCase`, `RunWithExpectedFailures` | stays | Sketch-verdict matching (`sketch.Status`, DOF, `errors.Is` on the reason). |
| `RequireSound(t, s)` and `detail`/`constraintName`/`pointName` | stays | Judges `sketch.VerificationReport`; decad never re-derives a 2D answer and neither does its test kit. |
| `Step`, `Unmodelled` | stays | Logging and skip conventions of the proof methodology; shared with the 3D side. |
| `NewSketch(t)` | moves → `decadtest.Sketch` | Identical body. `proofkit.NewSketch` becomes `return decadtest.Sketch(t)` for one release, then drops. |

### proofkit3d (solid proofs)

| Symbol | Fate | Why |
|---|---|---|
| `Case`, `Build`, `Assert`, `Gate`, `ParallelGroup` | stays | The build/gate/assert case runner is proof methodology (bare-float dialog params, one document per case, at-least-one-completed). See open question 1. |
| `Run`, `RunSolid`, `RunSolidParallel`, `RunWithGate`, `RunWithGateParallel`, `runCases` | stays | Same reason; the runners lose nothing and shrink because the gates below become one-liners. |
| `Unmodelled` | drops | Duplicate of `proofkit.Unmodelled`; call sites switch import. |
| `RequireSound(t, doc, bodies)` | moves → `decadtest.Sound` | Nil-body check stays in the gate wrapper (it is about `Build`'s contract); `Sound` does the verify, the `Passed` check and the diagnostic dump. |
| `RequireSolid(t, doc, bodies)` | moves → `Verify` + `Valid` per body + `OnlyDiagnostics(DiagMeasurementBeyondTolerance)` | The kit provides the three checks; the gear-side gate keeps its policy that only Area/Centroid readings may be beyond tolerance, as a five-line filter over the `Diagnosed` slice. |
| `BodyReport(t, doc, body)` | moves → `decadtest.Verify` + `decadtest.BodyReport` | The re-verify-per-call semantics become explicit at the call site: one `Verify`, then lookups. |
| `bodyReportFrom` | drops | Is `Report.ForBody` now. |

### Per-gear duplicates

| Symbol | Where | Fate | Why |
|---|---|---|---|
| `newSketch`, `onlyRegion` | spurgear, cycloidal | moves → `Sketch`, `Region` | Same code in two packages. |
| `extrudeSection` | spurgear, cycloidal | moves → `Prism` | Same code; the params map stays at the call site. |
| `volumeOf` (returns `float64`) | spurgear, cycloidal | moves → `Volume`/`Agree` at each call site | The comparisons it fed become `Volume(t, b, want, WithinRel(...))` or `Agree(t, "patterned vs seed", ...)`. Where a value is still summed into an oracle, the site reads `body.Volume()` itself. |
| `bgVolume` (returns value and bound) | bevelgear | moves → `Volume` with `WithinRel` | Its callers already do `max(bound, rel*want)`, which is the kit's rule. |
| `bgClose`, `bgCloseTB` | bevelgear | stays | Labelled float comparison for gear geometry (cone slopes, half-angles, stations); not a decad reading. |
| `bgReadRings`, `bgReadCone`, `centroidAngle`, `angleGap`, `patternTurn` | bevelgear, spurgear | stays | Gear geometry read off topology; the kit has no opinion. |
| `mm` | spurgear | stays | Trivial. |
| `separatedBlocks`, `separatedBlocksIn` | proofkit3d tests | drops | Replaced by `decadtest.Block` in the runner's own tests. |

The two packages export 26 symbols between them. Four move into `decadtest`: `proofkit.NewSketch`,
and `proofkit3d`'s `RequireSound`, `RequireSolid` and `BodyReport`. One drops,
`proofkit3d.Unmodelled`, whose call sites switch to `proofkit`'s copy. The other 21 stay. Of the
gear-local helpers the two proof packages do not export, nine move and seven stay.

### What a migrated call site looks like

```go
// before
if got := volumeOf(t, bodies[0], "Gear Body"); math.Abs(got-want) > 1e-6*want {
    t.Errorf("Gear Body volume %.6f mm3, want the root disc's %.6f mm3", got, want)
}
// after
decadtest.Volume(t, bodies[0], units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-6)))
```

```go
// before (proofkit3d.RequireSolid)
report, err := doc.Verify(t.Context()) ... for _, d := range report.Diagnostics { ... } ...
// after
report := decadtest.Verify(t, doc)
decadtest.OnlyDiagnostics(t, report, decad.DiagMeasurementBeyondTolerance)
for _, body := range bodies {
    decadtest.Valid(t, report, body)
}
```

## 7. Testing the kit

The kit is production code with a test suite in `decadtest_test` (testify allowed there).
Every helper gets a pair of tests, and the pair is the deliverable:

- **A positive test on real geometry.** `Block` builds a 100×60×10 plate; the test asserts
  `Volume` at `60000 mm³` with `Exactly()`, `Area` at `15200 mm²`, `Centroid` at `{50 30 5}`,
  `Bounds`, `Manifold`, `SurfaceKinds{KindPlane: 6}`, and `Sound`. A boolean fixture (two
  overlapping blocks through `decad.Union`) exercises the Approximate path so the bound is
  real and nonzero on both architectures. Numbers come from decad, never from a hand-written
  `Measurement` literal, except in the one place below.
- **A negative test that shows the helper fail.** Each helper is called through a recording
  `testing.TB` — a struct embedding `testing.TB` whose `Fatalf`/`Errorf` capture the message
  and whose `FailNow` calls `runtime.Goexit`, run on its own goroutine — and the test asserts
  the captured message contains the phrase the design pins ("does not enclose", "is a Length
  (mm) but the reading is a Volume (mm^3)", "report is Suspect, want Sound"). A helper with no
  red test does not ship. This is the house rule that a fixture must be shown to fail before it
  is trusted, applied to the fixtures' own maker.
- **Bound literals appear only in the kit's tests, and only to test the kit.** `BoundAtMost`
  and the `Exact`-with-nonzero-bound falsifier are exercised with constructed
  `decad.Measurement{Value, Exactness, Bound}` values, because the point is the comparison, not
  decad's geometry. No test pins a bound decad produced.
- **Message goldens use exactly representable inputs** (`60000`, `10.5`, `{50 30 5}`) so the
  rendered text is identical on amd64 and arm64.
- **Kind and nil contracts** each get one red test: wrong-Kind `want`, wrong-Kind `Within`,
  nil body, nil report, a body absent from the report.
- **Fixtures are proven end to end**: `Block`'s test measures the body decad built and checks
  the recipe holds one `OpExtrude` step, so the real producer's output has passed through the
  real consumer before any other test relies on it.
- `go vet ./...` and `golangci-lint run` cover the package as production code.
