# Tessellation Reach Design

Implementation plan for extending `Body.Tessellate` from the three payload classes it dispatches today
(`facetedPayload`, `cupPayload`, `prismPayload` with an analytic-walled section) to every payload the
evaluator builds. Companion to `docs/tessellation-design.md` ("tess §N"), which stays the normative owner of
the mesh contract, the proof record, the shared chording rules, and the revolve theory (tess §§8–11). This
document owns three things tess does not: the increment-by-increment implementation plan for tess §13's
T2–T4 and T6, the extruded free-form chording tess §12 hands to `docs/spline-design.md` §10 P5, and the
cap-loop chamfer tessellator (`capBlendPayload`), which has no tess row at all. Every geometric claim tess
already states is cited, never restated.

Router (navigation, not authority):

| Your question | Section |
|---|---|
| what tessellates today, and what the proof record actually holds | §1 |
| in what order the four refusing payloads land, and why | §2 |
| what `Mesh` must carry before any restatement is boolean-sound | §3 (R0) |
| loft restatement: where the triangle set lives, what the proof terms are | §4 (R1) |
| free-form prism chording inside `chordLoop` | §5 (R2) |
| revolve T2/T3/T4: which tess paragraph each function discharges | §6 (R3–R5) |
| cap-loop chamfer: proof-record row, cells, refusals | §7 (R6) |
| recorded decisions this design changes | §8 |
| open questions | §9 |
| the task list an implementer executes | §10 |

## 1. Current state

`tessellateContext` (`tessellate.go`) dispatches on `facetedPayload`, `cupPayload`, `prismPayload`,
`loftPayload` (R1) and `revolvePayload` (§6, R3), and refuses every other with `ErrUnsupported`. A
free-form-walled `prismPayload` reaches the prism path and chords there: `chordLoop` switches on `walkKind`
and gives a `walkFreeform` walk its own dyadic station chain (§5, R2). A `revolvePayload` meshes through
`tessellate_revolve.go` for every ANALYTIC generator, line and circular alike (T2, T3); a free-form
meridian generator refuses (T5).

`Mesh` carries tess §2's proof record complete, populated by all five of those paths (§3, R0):

| tess §2 proof | Held on `Mesh` | Consumer |
|---|---|---|
| `sourceBound(face)` | `faceBound`, one entry per source face | `facesOfMesh` (`boolean.go`), for the hidden-tangency pre-pass |
| `areaSlack` | yes, the analytic terms plus a per-facet coordinate allowance | boolean area composition |
| `volSymDiff` + `symDiffOK` | yes; `symDiffOK` true for prism, cup, loft, revolve and faceted, and FALSE for the cap-loop chamfer | `operandSymDiff` (`boolean.go`), which refuses a mesh carrying no proof |
| `deltaStore` (tess §5) | charged per vertex into `faceBound`, `areaSlack` and `volSymDiff` | — |

Every restatement below publishes into that record, and no consumer infers a term the mesh did not state. A
`Plane` face with `Line3` edges publishes zero only where its own polygon and its own stored coordinates are
both proved exact, so a placed loft's planar facets cannot enter the hidden-tangency pre-pass at 0 while
sitting `delta` off their true position. `bound * heldArea` is nowhere a substitute for the occupied-volume
proof: for a chorded loft that proof composes `sweptVolumeAllow(delta, perturbedAreaUpper)` with the four-leg
`chordedBoundaryVolumeAllow` (tess §2's row), and for a revolve tess §11 forbids the product outright. A
payload whose occupied-volume proof has not landed returns a mesh with `symDiffOK` false, which serves export
while the boolean refuses the operand.

## 2. Increments

| # | tess/spline label | Lands | Reason for its place |
|---|---|---|---|
| **R0** | tess T1 completion | `Mesh` carries `faceBound`, `volSymDiff`, `symDiffOK`; prism/cup/faceted populate them; boolean reads them | every later row publishes into this record; without it R1 is boolean-unsound (§1) |
| **R1** | tess T6 | `loftPayload` exact restatement + boolean admission | cheapest reach: the triangle set already exists; unblocks loft booleans and interference, the north-star oracle |
| **R2** | spline §10 P5 | free-form arm of `chordLoop`: dyadic station chain over `spline_sagitta.go` | rides the existing prism path; spline Table C orders export/booleans/interference for free-form walls "before revolve" |
| **R3** | tess T2 | revolve, line generators only: cylinder/cone/plane/axis, poles, partial caps, both coordinate proofs, `Ecell`, export | first revolve export; largest single increment, so it follows the two cheap ones |
| **R4** | tess T3 | revolve circular generators: sphere/torus, axis-to-axis minimum, intra-loop tube clearance, circular `Ecell` | extends R3's cells; same files |
| **R5** | tess T4 | `Mmeridian` + certified `Icell`; `symDiffOK` true; revolve admitted to booleans | the last revolve proof; heaviest proof engineering |
| **R6** | new: T7 | `capBlendPayload` tessellator, export-only (`symDiffOK == false`) | depends on R3's cone/pole cells and R1's twist term; last because no other increment waits on it |

R6 depends on R3 only, so it may be scheduled anywhere after R3. R5 and R6 are independent.

Each increment ships with its own geometry tests (§10). No increment returns a mesh before its row's
boundary and area proofs are met (tess preamble).

## 3. R0 — the proof record on `Mesh`

### Responsibility

Make `Mesh` the single carrier of tess §2's private proofs, and make the boolean read them instead of
inferring them.

### Interface (private)

```go
type Mesh struct {
    vertices  []r3.Vec
    triangles [][3]int
    source    []*Face
    bound     float64            // max over faceBound
    faceBound map[*Face]float64  // tess §2 sourceBound(face), every source face present
    areaSlack float64
    volSymDiff float64           // tess §2; meaningful only when symDiffOK
    symDiffOK  bool
}
```

Rules:

- `bound == max(faceBound)` for every payload class. A source face missing from `faceBound` is an evaluator
  invariant failure (`ErrBooleanFailed` in the boolean, `ErrDegenerate` from `Tessellate`), never a zero.
- `facesOfMesh` reads `m.faceBound[f]` and `faceChordDelta` is deleted. For prism/cup the published value
  is never below what `faceChordDelta` returns today (a planar face with straight edges publishes
  `deltaStore + sectionDelta + deltaAxial(face)`, which is at least `sectionDisplacementOf`).
- `operandSymDiff` returns `m.volSymDiff` when `m.symDiffOK`, else `ErrUnsupported` (a staging refusal,
  routed through `expectedBooleanForOperand(booleanExpectedStaging, …)` like a tessellation refusal).
  `tessellateFaceted` sets `volSymDiff = fp.volSymDiff, symDiffOK = true`.

### Prism and cup population

Per tess §5/§6, with the terms the current code already computes plus the two it does not:

| Term | Source |
|---|---|
| `deltaTrim(face)` | wall: its walk's `chordCount` sagitta; cap/floor/rim: max sagitta over the loops bounding that patch (`chordedLoop.maxSag`) |
| `deltaSection` | `pp.sectionDelta` (0 for a cup) |
| `deltaAxial(face)` | `z0Delta` for `capStart`, `z1Delta` for `capEnd`, `max` for a wall; cup floors/rims read `cp.zDelta` per level |
| `deltaStore(face)` | NEW. Two mechanisms, max over the face's vertices: (a) a computed circular sample's own gap from the certified enclosure of the point it denotes — `circularWalkEndBound`'s mechanism (`extrude.go`) applied at interior fraction `k/n` through `turnSinCosInterval` for a `CircleSeg` and `radSinCosSpan` over `atan2Interval` for an `ArcSeg` (`moments.go`, `moments_trig.go`), carried by `walkEndBoundAllow`; (b) `exactPrismPointRound(pp, u, v, z, held)` (`extrude.go`) for the frame/placement write. Both are zero for a recorded line vertex under an axis-aligned identity payload |

`faceBound(f) = upRound(deltaTrim + deltaStore + deltaSection + deltaAxial)`. `bound` stays the conservative
composition tess §5 already permits (largest sagitta + payload-wide axial max + `deltaStore` max).

`areaSlack` adds `perturbedTriangleAreaAllow(a, b, c, deltaStore)` per triangle (tess §5's
`Rarea_triangle`) after the existing analytic terms.

`volSymDiff` per tess §5/§6:

```text
Mprism_analytic = |z1-z0| * Σ_walks segs(walk)          // walkAreaSlack's own `segs` term, once per walk
Msection        = sectionDisplacementArea(δs, walks, perimeterUpper) * |z1-z0|
Maxial          = z0Delta*capAreaUpper + z1Delta*capAreaUpper
Mstore          = sweptVolumeAllow(deltaStore, perturbedAreaUpper(verts, tris, deltaStore))
volSymDiff      = upRound(Mprism_analytic + Msection + Maxial + Mstore)
```

`capAreaUpper` is the region area upper bound the payload already publishes (`body.area`'s cap share) or the
chorded polygon area plus `areaSlack`. The cup composes `hO*E(O) + hC*E(C)` (tess §6) in place of the first
term. `symDiffOK = true` for both.

`walkAreaSlack` is split so `segs` (the exact `Σ a_c` per walk) is reachable on its own; the padding factor
stays.

### Failure behavior

A non-finite `deltaStore`, `faceBound`, or `volSymDiff` refuses `ErrUnsupported` (tess §12). `chordStationBound`
answering `+Inf` for an underivable enclosure is the same refusal.

### Tests (R0)

- Plate with a circular hole (`holedPlateBody`, `tessellate_test.go`): every wall face of the hole publishes
  `faceBound >= chordSagitta(r, 2π, n)`; each cap publishes at least that; every straight outer wall publishes
  exactly 0 under identity placement.
- The same plate under a rotated placement: every `faceBound > 0`, and `Bound` exceeds the unplaced `Bound`
  by at most `radius3D(16·ulp(coordMax))` (`rigidRoundAllow`'s own scale).
- tess §14 "Check prism/cup `volSymDiff` against exact circular-segment examples": a full cylinder of radius
  `r`, height `h`, `n` chords: `volSymDiff >= h * (π r² − n r² sin(2π/n)/2)` and at most twice it.
- A boolean over two prisms now reads `volSymDiff`: assert the result's `Volume().Bound` equals the sum of
  the two operands' `volSymDiff` plus the weld term, bit for bit, by reconstructing it in an internal test.

## 4. R1 — `loftPayload` exact restatement (tess T6)

### Where the triangle set lives

`loftPayload` (`loft_build.go`) holds `verts`, `tris`, `walls`. `tris[:walls]` are the wall triangles in
`loftAssembly.cell`/`side` order; `tris[walls:walls+capStartCount]` are `capStart`'s, the rest `capEnd`'s.
`capStartCount`, `cell` and `side` live on `loftAssembly` only. `matchedDelta` and
`loftChordedAllow.maxTwistOffsetUpper` are locals of `evalLoft`; the payload's own doc comment says
`matchedDelta` is deliberately not stored because nothing stored needed it. The restatement needs it, so the
payload gains the COMPOSED proof terms, regenerated on every `placed` (which re-runs `evalLoft`), never the
raw `matchedDelta`:

```go
// loftMeshProof is tess §2's loftPayload row, composed once by evalLoft.
type loftMeshProof struct {
    facetDeparture float64 // absSumUpper(matchedDelta, maxTwistOffsetUpper)
    areaSlack      float64
    volSymDiff     float64
}
```

`loftPayload` gains `capStartCount int`, `cell [][2]int`, `side []uint8`, `proof loftMeshProof`. All four are
copied from `loftAssembly` / composed at the end of `evalLoft`; `placed` already nils the triangle fields and
re-evaluates, so nothing stored can disagree with the records.

### Composition (in `evalLoft`, after `mass.chorded` is set)

| Term | Composition | Owner of each input |
|---|---|---|
| `facetDeparture` | `absSumUpper(chordCellDeltaUpper(sectionMatchedDelta, a.delta), chorded.maxTwistOffsetUpper)` — computed UNCONDITIONALLY, so a `LineSeg`-only placed loft publishes `a.delta` (loft §5.2's facet-departure row: "`matchedDelta` reduces to `delta`") | loft §5.2 `matchedDelta`, `maxTwistOffsetUpper` rows |
| `areaSlack` | `upRound(mass.perturbAreaSum + chorded.twistAreaAllow + chorded.areaExcess + chorded.capAreaExcess)` | tess §2's loft row: the per-triangle perturbation sum, the wall's three legs (`cellTwistAreaAllow` held-to-bilinear, `cellChordCurveAreaAllow` + `cellStationShiftAreaAllow` already summed in `areaExcess`), and the two caps' `capAreaAllow` |
| `volSymDiff` | `absSumUpper(sweptVolumeAllow(a.delta, perturbedAreaUpper(verts, tris, a.delta)), chordedBoundaryVolumeAllow(matchedDelta, chorded.wallAreaUpper, chorded.twistVolumeUpper, chorded.capVolumeUpper, chorded.seamAllow))` | tess §2's loft row; the FOUR-leg helper (`bounds.go:chordedBoundaryVolumeAllow`), not the three-leg residual `Volume` uses, because the mesh holds the uncorrected triangles |

`loftChordedAllow` gains `twistAreaAllow`, the sum of `cellTwistAreaAllow(vLo, vHi, wLo, wHi)` over the same
chorded cells `computeLoftChordedAllow` already walks (gated on `p.matchedDelta[j] > 0`, never on kind).

### `tessellateLoft(ctx, b, lp, chord)`

1. Build `byRole` from `b.Faces()` as the prism does. For `k < walls`: face of role
   `side(cell[k][0], cell[k][1], side[k])`. Then `capStartCount` triangles → `capStart`, rest → `capEnd`.
   A missing role is `ErrDegenerate` (tess §4).
2. Copy `verts`, `tris` (fresh slices — `Mesh` accessors already copy, but the payload's slices must not
   alias the mesh's). No chording, no retriangulation, no reflection flip: loft §5's whole-shell orientation
   step already made every triangle outward, and `placed` re-runs it, so tess §4's "reflected placement
   reverses once" rule is discharged by the payload. Assert the tetrahedron sum positive in the audit below.
3. `faceBound[f] = proof.facetDeparture` for every face; `bound = facetDeparture`; `areaSlack`, `volSymDiff`
   from `proof`; `symDiffOK = true`.
4. `requireClosedMesh`, then the signed-volume audit. Either failing is `ErrUnsupported` (tess §12) — it can
   only mean the payload's own §6 audit was bypassed.

No tolerance refusal: the tessellation adds no chording of its own, and the whole `Bound` is inherited
payload displacement, which tess §1's Tolerance row lets ride above `tol` (the same standing as a prism's
`deltaAxial`). `defaultChordTolerance` is unchanged for a loft.

### Boolean effect

Through R0, a loft whose `facetDeparture == 0` enters the existing all-planar zero-bound path; every other
loft is an ordinary positive-bound all-planar operand via `rimDelta` (loft Table D D2). `Verify`'s read-only
`OpIntersect` follows.

### Tests (R1)

tess §14's loft bullet in full, plus:

- `TestLoftTessellateStaged` (`loft_test.go`) becomes `TestLoftTessellate`: mesh non-nil; triangle count
  `== 2·Σstations + capTris`; `requireWatertight`; `meshVolume(mesh)` within `body.Volume().Bound()` of
  `body.Volume().Value`; every `SourceFaces()` entry is one of `b.Faces()`; the mesh vertex set equals the
  body's `Vertices()` positions exactly.
- Unplaced pinned `LineSeg`-only loft: `Bound() == 0`, and `Union(loft, prism)` reports an `Exact` volume
  equal to the analytic sum minus overlap.
- Placed `LineSeg`-only loft: `Bound() == delta` (read `Bounds().Bound` from the body, which equals
  `absSumUpper(delta, 0)`), and `Cut(prism, loft)` succeeds with a positive `Volume().Bound`.
- Chorded `ArcSeg` pair loft: `Bound()` strictly greater than `Bounds().Bound()` whenever
  `maxTwistOffsetUpper > 0` (assert through a helical-tooth fixture, `loft_helical_tooth_test.go`).
- Byte-identical `STL` on two calls.

## 5. R2 — free-form prism chording (spline §10 P5)

### Responsibility

Give `chordLoop` a `walkFreeform` arm so an extruded Tier A free-form wall chords, with a proven sagitta,
station displacement and area slack, on the existing prism path. Nothing else in `tessellateContext`
changes: `requireLoopClearance`, `triangulate2DContext`, `earClip` and the wall/cap emission already consume
2D samples.

### Machinery

`spline_sagitta.go` is the machinery. It owns `dyadicSpanSagittaUpper` (spline §6.2.1's control-point-to-
chord-SEGMENT bound), `dyadicSpan.split` (`spline_length.go`, exact midpoint de Casteljau), `spanSpeedUpper`
(a proven arc-length upper bound), and `sagittaStationWalk`, a two-sided measure-then-bisect walker built for
the loft's paired chains. R2 adds the ONE-sided sibling:

```go
// chainStations chords one Tier A span chain at target sagitta: measure each dyadic cell's
// sagitta, bisect what misses, accept what fits. spline §6.2.1; the single-chain twin of
// pairStations, sharing its cap, its charge discipline and its termination argument.
func chainStations(spans []bezierSpan, target float64, work *freeformWork) (freeformChain, error)

type freeformChain struct {
    stations    []ratPoint // exact points ON the curve, one per cell boundary, chain start included, end excluded
    end         ratPoint   // the excluded final boundary, copied so it aliases no input span
    sagitta     float64    // measured post-subdivision MAXIMUM over cells (never a sum)
    cellArcUpper []float64 // spanSpeedUpper of each accepted cell's own sub-span
    cellChordLower []float64 // proven lower bound on each accepted cell's chord length (chargedRatSqrtDown)
}
```

Implementation: `sagittaStationWalk` measures, accepts and bisects over a SLICE of sides (`[]dyadicSpan`),
and both `pairStations` (2 sides) and `chainStations` (1 side) drive it. What each consumer owes per accepted
cell — the pair's matched delta, the chain's arc/chord bracket — is a `stationCellReader`, the one place the
two differ. The cap (`maxChordsPerWalk`, `errTooManyChords`, spline R8) and the R7 work refusal stay where
they are.

### The `chordLoop` arm

For a `walkFreeform` walk `w` (`w.spans`, `w.reversed`):

1. `chain := chainStations(w.spans, chord, work)`; a reversed walk emits stations from the chain's end
   toward its start (the walk's own start is the chain's last cell boundary, so the emitted list is the
   reversed station list with the chain's END point prepended and its START point dropped).
2. Each station rounds ONCE into `Point2` (`ratPoint` → `float64` per component). `stationRound` for that
   station is `radius2D(rationalFloatError(u), rationalFloatError(v))` — loft §5.2's `stationRound`
   free-form arm, cited. The walk's `maxSag` is `chain.sagitta`; its `stationRound` max joins `deltaStore`
   (§3) on every face the loop touches.
3. `faceOf[j]` is `side(li, w.segs[0])` for every station of the walk — one `NURBSSurface` face per free-form
   segment (evaluator §5's table), and `coalesceWalks` never merges a free-form walk.
4. `areaSlack` per cell `k`: wall `(cellArcUpper[k] − cellChordLower[k]) * h`; caps
   `2 * sectionDisplacementArea(chain.sagitta, 1, cellArcUpper[k])` (the region between a curve within `s`
   of its chord SEGMENT and that chord lies in the `s`-neighbourhood of the segment, which a cell can cross
   at an inflection, so the two-sided tube is the bound, not a one-sided circular segment).
5. `volSymDiff` (R0's `Mprism_analytic`) adds `sectionDisplacementArea(chain.sagitta, 1, cellArcUpper[k]) * h`
   per cell.

Why the SET distance suffices here where the loft needs a parameter-matched one: tess §1's `Bound` is a
two-sided Hausdorff bound, the wall is the ruled sweep of the 2D chord along the plane normal through an
orthonormal frame, so the 2D set distance between chord and curve IS the 3D set distance between held wall
strip and `NURBSSurface` patch. `areaSlack` reads `spanSpeedUpper`, an arc-length bound, and needs no matched
parameter either.

### Refusals

| Condition | Result | Raised by |
|---|---|---|
| chain needs more than `maxChordsPerWalk` cells | `ErrUnsupported` (`errTooManyChords`, spline R8) | `chainStations` at the split |
| exact-rational work exceeds the record-level counter | `ErrUnsupported` (spline R7) | `freeformWork.step` inside each primitive |
| a cell's sagitta or speed enclosure is non-finite | `ErrUnsupported` | `chargedRatSqrtUp` / `spanSpeedUpper` |
| a station's `Point2` rounding gap is non-finite | `ErrUnsupported` | the arm, before emission |
| the chorded loop self-intersects or pinches | `ErrDegenerate` via `tessellationExpectedError` | `earClip`, `requireLoopClearance` — unchanged |

A rational (Tier B/C) walk never reaches the arm: spline R10 refuses it at `Extrude`.

### Tests (R2)

- The spline §6.2.1 overshoot net `(0,0), (−3,0.01), (4,0.01), (1,0)` closed by a `LineSeg`, extruded 5 mm:
  watertight; every wall vertex lies on the Bézier evaluated in `big.Rat` at the station's dyadic parameter
  to within one `Point2` rounding; dense sampling of the curve finds no point farther than `Bound()` from the
  mesh (a falsifier); the chording component of `Bound()` is `<= tol`.
- Chord count non-increasing as `tol` doubles; a `tol` no bisection can reach refuses `ErrUnsupported`
  before any vertex is allocated. Which of the two refusals binds is fixture-dependent and NOT asserted: the
  exact-rational work counter (spline R7) is charged per bisection and is reached long before one curve's
  chord cap (R8) is, on every fixture this package builds.
- A 15-point involute `FitSplineSeg` prism (`extrude_test.go`'s `involuteFlankSketch`): byte-identical `STL`
  and `OBJ` on two calls. On the fit-spline arch: `Cut(prism, box)` where the box lies strictly inside
  reports `Volume` within `Bound` of `prism.Volume − box.Volume`; two overlapping free-form prisms make
  `Verify` decide their pair with a proven `Interference` row.
- `extrude_freeform_test.go`'s tessellation, export, boolean and interference refusal assertions flip to
  success; its clearance and survey refusals stand, since neither reads the mesh.

## 6. R3–R5 — revolve (tess T2/T3/T4)

tess §§8–11 are the theory; this section maps each paragraph to code. No new geometry is stated here.

### Files

| File | Owns |
|---|---|
| `tessellate_revolve.go` | `tessellateRevolve`: walk resolution, the axis-incidence and section gates, the angular count, rings, poles, cells, partial caps, orientation, and the assembled mesh's own audits (tess §8, §9) |
| `tessellate_revolve_proof.go` | The angular sequence's certified trig, `deltaC`/`deltaR`, the tolerance split, the one facet-pair audit that carries both coordinate homotopies, the vertex-link audit, and `Ecell` (tess §8, §9, §10) |
| `tessellate_revolve_volume.go` (R5) | `Mmeridian`, per-cell `Icell`, `Mconstruct`, `Mround`, `volSymDiff_revolve` (tess §11) |

### Shared with the builder

`buildRevolveLoop` (`revolve.go`) resolves each loop's walks (`walkOf` → `requireAnalyticWalk` →
`rp.ax.walk` → `coalesceWalksContext` → `ax.classify`). Extract that prefix into
`revolveLoopWalks(ctx, rp, loop, work, what) (revolveWalks, error)`, whose result carries the coalesced axis
walks, their `wallKind`s, `singleClosed`, and the same walks still in PLANE-local coordinates indexed by
recorded segment. Both the builder and the tessellator call it, so the mesh reads exactly the walks the body
was built from (tess §3: "read the evaluator's payload, NEVER live sketch input"), and the proof reads the
recorded plane point the axis re-expression consumed rather than the re-expressed floats, which
`axisFrame.walk` states no axial bound for and SNAPS a near-axis radial one to zero. The evaluator §6
axis-incidence audit runs from the tessellator as `requireRevolveAxisIncidence` (tess §9's "also run").
`revolveBasis` (`rp.basis()`) gives `a3, w, e0, e1`; the tessellator evaluates `X(z, ρ, φ)` UNPLACED in
binary64, stores, then applies `rp.xform` once (tess §8's two stages) — it does not use `rp.point`, which
composes both in one expression.

### R3 (T2) — what each piece discharges

| tess paragraph | Function | Notes |
|---|---|---|
| §8 profile → `(z, ρ)`, `rhoMax`, `zAbsMax`, `coordMax` | `revolveExtents(walks)` | endpoints plus cardinal points inside a circular walk's interval; non-positive `rhoMax` is an invariant failure |
| §8 `deltaC` | `revolveIdealBasis` + `revolveMeridianEnclosure` + `revolveIdealPoint`, measured per vertex | `ratInterval` arithmetic over the whole path — the axis basis, the meridian `(z, ρ)` with its endpoint bound and its axis SNAP, and `X` itself; the angular `sin`/`cos` are `turnSinCosInterval` for a full turn from zero and `radSinCosInterval`/`radSinCosSpan` otherwise, and the mesh STORES the float nearest each enclosure's midpoint rather than calling `math.Sincos`, which is what makes the trig gap a construction fact (`revolveTrigGapPrior`) instead of a library assumption; final gap per vertex via `intervalFloatError`, max over vertices, `radius3D` |
| §8 `deltaR` | `min(rigidRoundAllow(coordMax + deltaC, translationMax), exactRigidPointRound per vertex)` | the second term measures the same displacement exactly, so it is 0 under an exact identity and tighter elsewhere; `exactPrismPointRound`'s own mechanism |
| §8 budget order | `revolveConstructionPrior` + `rigidRoundAllow` before the count, the measured pair after | §8 splits the tolerance BEFORE the counts, and the count decides how many angles there are, so the split spends count-INDEPENDENT ceilings: the meridian gap (per sample, not per angle), the stored trig's own ceiling, and the evaluation's ulps at `coordMax`. The measured `deltaC`/`deltaR` are checked against them and refuse on a violation, so the a-priori figure is held to account rather than trusted |
| §8 tolerance split | `revolveBudget(tol, deltaC, deltaR)` | `available = downRound(downRound(tol − deltaC − deltaR))`; `<= 0` refuses; meridian gets `available/2`; angular gets `available − deltaM` |
| §3/§8 counts | `chordCount` for each circular meridian walk (R4) and for the global angular sequence with `radius = rhoMax`, `sweep = |φ1−φ0|` or `2π`, `closed = full` | the downward-then-upward correction and the cap are `chordCount`'s own; R3 sections have no circular meridian walk, so `deltaM == 0` |
| §9 section proof | `revolveSectionPoints` + `requireLoopClearance` over the `(z, ρ)` samples, run for a full and a partial sweep alike before any cell is formed (line-only sections: the homotopy is the identity, so the endpoint checks are the proof) | R4 adds the intra-loop tube check |
| §9 rings + poles | `tessellateRevolve`'s ring loop, read back through `revMeridian.at` | `ρ > 0` → `nPhi` (full) or `nPhi+1` (partial) vertices; `ρ == 0` → ONE vertex, answered for every angle and for both caps, so the interning is structural rather than a lookup |
| §9 cells | `emitRevolveCell` | both off axis → planar quad, fixed `(m_k, φ_l) → (m_{k+1}, φ_{l+1})` diagonal; one on axis → fan; both on axis → nothing only for `wallAxis`, else refuse "erased generator" |
| §9 partial caps | `emitRevolveCaps`: `triangulate2DContext` over the `(z, ρ)` samples | the `(u, v) → (z, ρ)` map is a ROTATION for either axis side, so the recorded loop's own sense survives it and no index order is reversed; the `(z, ρ)` frame's normal is the sweep-velocity direction, so the END cap takes the triangulation as it stands and the start cap reverses it. Pole vertices are ordinary 2D samples mapping to the interned vertex, so the on-axis line's edge is shared by both caps |
| §4 orientation | walk sense × sweep sense, then reflection | rule: with material on the walk's left in `(z, ρ)` and a right-handed sweep about `w`, `∂X/∂t × ∂X/∂φ` is ρ times the outward in-plane normal, so cell triangles are `(m_k,l), (m_{k+1},l), (m_{k+1},l+1)` and `(m_k,l), (m_{k+1},l+1), (m_k,l+1)`; the axis SIDE needs no negation of its own (see the caps row); negate for `rp.reflected()`; then the signed-volume audit |
| §9 endpoint + homotopy audits | `revolveContactAudit(budget, verts, tris, deltaC + deltaR)` | ONE pass, at the final stored coordinates, against the COMBINED displacement. Every mesh on either homotopy is a vertex-wise displacement of the stored one by at most that (the placement between the two stages is an exact isometry), and every predicate the classification consults is multilinear in the vertices, so a stored reading exceeding its own perturbation allowance fixes the sign for the whole family. A pair sharing nothing is proven apart by an exact separating axis with the same margin; a pair sharing a vertex or an edge is proven to meet ONLY there by a boundary plane built as a polynomial in the pair's own corners, so the plane keeps containing the shared feature identically. Facet-pair count preflighted against `maxFacetPairTestsPerCall` (tess §3), as `loft_audit.go` does |
| §9 link audit | `requireVertexLinks(mesh)` | every link one connected degree-two cycle; a pinched pole is `ErrUnsupported` |
| §10.1 bounds | `faceBound`: wall `deltaM + deltaPhi + deltaC + deltaR`; partial cap `deltaM + deltaC + deltaR`; exact-line cap `deltaC + deltaR` | `bound` = max |
| §10.2 `Ecell` | `revolveCellAreaSlack` / `revolveFanAreaSlack` over `absLinearIntegral` | tess §15's choice for T2, recorded there: COMPLETE SIGN DECOMPOSITION in closed form, with no root isolation at all. For a straight generator `Jtrue = L·dφ·ρ(t)` does not depend on `u` and `Jheld` is constant on each half of the domain the fixed diagonal cuts, so the difference is LINEAR in `t`, its single zero is an exact rational quotient, and each sign-fixed piece integrates through its own primitive. `clearance_poly.go`'s Sturm engine is not reached, and `cos dφ`/`sin dφ` enter only through the ideal triangle's own area — never inside a root isolation — so the widening §9's open question worried about cannot lose a sign. One evaluation answers for every angular interval: interval `l`'s ideal samples are the exact rotation of interval 0's, and a rotation is an isometry |
| §10.2 caps + coordinate stages | circular-segment areas (R4); `perturbedTriangleAreaAllow(·, deltaC + deltaR)` per facet | a straight generator's cap trim is exact, so a cap contributes no chording area at all; the combined displacement covers the ideal, stored-unplaced and placed triangles alike. Summed upward, after `Ecell` |
| §3 ceilings | `revolvePreflightFacets`: facet count, cumulative facet work AND the pair audit's own `F·(F−1)/2`, all with checked unsigned arithmetic before any slice | charging the pair ceiling here rather than at the audit is strictly earlier than §3 asks; it is the binding one, so a revolve mesh carries at most 4000 facets |

R3 refuses a section carrying any circular walk with `ErrUnsupported` ("circular meridian generators are
T3"); R4 retires that refusal. Its mesh carries `symDiffOK == false`, so the boolean refuses a revolve
operand through R0's `operandSymDiff` and asks the same question of the payload class first
(`requireVolumeProvingPayload`) rather than paying for a mesh no boolean may consume; R5 retires both arms
together, leaving the cap-loop chamfer as the only payload class on that list.
`export.go`'s two doc comments that name a revolve as un-exportable are narrowed to the circular-generator
case, and `doc.go`'s support map with them.

### R4 (T3)

- `chordCount` on every circular meridian walk; `deltaM` is the largest proven sagitta (tess §8 step 3).
- Axis-to-axis minimum: a circular walk with both ends on the axis takes `n >= 2` and must produce an
  off-axis interior sample; `chordCount`'s `nMin` becomes a parameter (`3` closed, `2` axis-to-axis, `1`
  else). A circular walk with positive interior `ρ` that chords to an axis-only polyline refines, then
  refuses.
- Intra-loop tube clearance: `requireWalkClearance(ctx, pts, loopIdx, walkSag)` — every non-adjacent walk
  pair within one loop clears `sag_i + sag_j + floor`, the same gate `requireLoopClearance` runs across loops,
  with the same `tessellationExpectedError`. This plus the endpoint checks is tess §9's `Hm` proof.
- Refinement: first failing meridian walk in payload order, rebuild, re-audit; angular failures increment
  the one global count.
- `Ecell` for sphere/torus cells: certified interval subdivision under a fixed budget (tess §15's choice for
  T3), each accepted interval an outward-rounded absolute integral bound.
- Ring collapse detection before and after placement (tess §9): `ErrUnsupported`, never merged into a pole.

### R5 (T4)

`tessellate_revolve_volume.go` owns all four of tess §11's stages; tess §15 records the `Icell` choice and
its derivation, and is the authority on both.

- `Mmeridian = sweepAngle * Σ_c |∫_{S_c} ρ dA|`: each circular walk's slivers charged their own proven total
  area (`chordSegmentArea`) times a proven upper bound on `ρ` over them, summed as absolute values.
- `Icell` per cell: BM's meridian is already a polyline, so tess §11's straight-generator homotopy answers for
  a sphere and a torus too. The integral separates into an exact rational meridian factor and one angular
  factor that depends on `dφ` alone, proven ONCE per mesh by certified subdivision (tess §15).
- `Mconstruct`, `Mround` via `sweptVolumeAllow` over `perturbedAreaUpper` at the composed displacement, which
  covers every surface on both stages' paths.
- `symDiffOK = true`; `pairChordTolerance` raises the pair's tolerance above `deltaC + deltaR` of either
  revolve operand (`coordDisplacementOf` over `resolveRevolve`) the way it raises it above a section
  displacement today.

### Refusal mapping (tess §12)

| tess §12 row | Raised by |
|---|---|
| non-positive `available` | `revolveBudget` |
| inverse underflow / unrepresentable ceiling / cap | `chordCount` (unchanged) |
| on-axis incidence malformed | `requireRevolveAxisIncidence` (`ErrDegenerate`) |
| positive-radius ring collapse; erased generator | `revolveMeridianSamples`, `tessellateRevolve`'s own cell loop |
| meridian simplicity/nesting/clearance | `requireLoopClearance`, `requireWalkClearance` after refinement exhausts |
| non-adjacent facets intersect; homotopy sign not fixed | `revolveContactAudit` |
| directed-edge / link / zero area | `requireClosedMesh`, `requireVertexLinks` |
| a payload class with no occupied-volume proof in a boolean | `requireVolumeProvingPayload` before the mesh, `operandSymDiff` (R0) after it |

### Tests

tess §14 assigns them; the owning increment is: R3 — full cylinder, partial cone, planar annulus, poles and
apexes (one interned vertex, fans only, one link cycle), partial on-axis line (both caps share one axis
edge), reflected placement, holes, several shells, large-coordinate `deltaC` and non-identity `deltaR`
charged separately, the facet/work ceilings hit exactly and one beyond, byte-identical export; R4 — full
sphere, spherical band, ring torus, concave torus wall, the radius-1 full-turn `n = 122` threshold, the
`2r` and 5 mm budgets, the two-semicircle tangent disk refused `ErrDegenerate` before tessellation, the
narrow-bulge hole below and above the summed tubes, the inner-torus sign-changing `Ecell`; R5 — the interval
integrator on fixed-sign and adversarial cells, revolve×prism and revolve×revolve booleans with a hidden
tangency refusal and a shallow crossing whose rim amplification refuses, the property fuzz across tolerance
scales. `revolve_property_test.go` already tessellates opportunistically; after R3 its watertightness
invariant becomes mandatory for line-only fixtures, after R4 for all.

## 7. R6 — cap-loop chamfer (`capBlendPayload`), tess T7

### Verdict

It earns an export-only increment, last. Every term its proof record needs is either an existing published
term (`band.delta`, `g.levelDelta`, `capBandLevel`, `capPatchWindowSkew`, `cellTwistOffsetUpper`,
`chordSagitta`, `miterLocusSpeedUpper`) or a one-step derivation from one, stated below. What does NOT
exist is a proven occupied-volume homotopy from the held facets to the exact offset family, so the boolean
stays refused (`symDiffOK == false`), which tess §2 permits for an export-only increment. DX3's "a strip whose
densities disagree is not watertight" is answered by sharing ONE count per wall walk across the side wall,
the band patch and the cap contour (below).

### Geometry the payload states

`capBlendPayload` (`capblend.go`) holds the receiver's `profile`, `frame`, `z0/z1` with their axial
displacements, `d` with `dDelta`, the selected `startLoops`/`endLoops`, and `patches []capPatch` — every
`chamferCap(cap, loop, k)` role beside its plane-local `capPatchGeom` (`capblend_geom.go`): a `Plane` patch's
four corners, or a `Cone` patch's centre, `sideRadius`/`capRadius`, side window `th0..th1`, cap window
`capTh0..capTh1`, `sideZ`/`capZ`, `sweepCCW`, `wholeTurn`, `contourAllow`, `levelDelta`, `capThAllow`. Each
band's cap contour displacement `band.delta` is `capBandResult.delta`; it must also be stored per band on the
payload (`bandDelta map[bandKey]float64`) for the tessellator to read.

Faces the body holds (`evalCapBlendContext`, `capblend_moments.go`): per loop, trimmed side walls over
`[zLo, zHi]` where a chamfered end pulls its level in by `d` (`buildLoopSidesAs` over `prismLike`); per
chamfered `(loop, cap)`, one band of patches; two cap faces whose boundary is the offset contour for a
chamfered loop and the original loop otherwise (the `mixed` profile of `mixedOffsetProfile`).

### Chording — one count per wall walk, shared three ways

For each loop, resolve its walks as `chordLoop` does. For a circular walk `w` chamfered on cap `c`:

```text
n(w) = max(chordCount(w, budget), chordCount(capArc(w, c), budget))   // over each chamfered cap c
```

where `capArc` is the offset arc at `capRadius` over `capTh0..capTh1`. `n(w)` chords the ORIGINAL loop at
every level the wall reaches (the side wall's own ring at `zLo`/`zHi`, which is the band's side directrix at
`sideZ`), and the cap arc over its own window at the same `n(w)`, sample `k` at fraction `k/n` of each
window. A `Plane` patch needs no count. A reflex corner's connector arc (an apex patch, `sideRadius == 0`)
chords with its own `chordCount`. A `wholeTurn` patch has one closed walk, `n >= 3`, both rings at the same
azimuths.

Rings: side ring per loop per level (shared with the trimmed side wall exactly as the cup shares its rings);
cap contour ring per chamfered `(loop, cap)` at `capZ`, built from the cap arcs' samples joined at the miter
feet (`capWallFoot`) and apex arcs. The cap face at `capZ` triangulates through `triangulate2DContext` over
the contour rings of chamfered loops and the original rings of unchamfered ones, after `requireLoopClearance`
over all of them.

### Cells

| Patch | Cells | Departure of a held facet from the DENOTED surface |
|---|---|---|
| `Plane` | one quad (`sideA, sideB, capA, capB` — two parallel segments, coplanar), two triangles | `band.delta + levelDelta + deltaAxial + deltaStore` — a line offset is affine, so the quad IS the denoted family (modify-reach §8.3) |
| `Cone`, windows differ | `n(w)` quads between side sample `k` and cap sample `k`, fixed diagonal, two triangles each | `twist + sagitta + skewGap + locusGap + capRadiusRound + band.delta + levelDelta + deltaAxial + deltaStore` |
| `Cone`, `wholeTurn` or tangent join | same, but same azimuths both rings → planar quads (tess §9's cone cell) | as above with `twist == 0`, `skewGap == 0`, `locusGap == 0` |
| apex (`sideRadius == 0`) | fan from the interned corner vertex at `sideZ` to the connector arc's samples | `sagitta(cap arc) + band.delta + levelDelta + deltaAxial + deltaStore` |
| trimmed side wall, cap face | the prism's own cells | the prism's own terms (§3), the cap face's `deltaTrim` read over the contour ring |

Term derivations (this section owns them; each is a positional twin of a term modify-reach §8.3 already
publishes in another dimension):

- `twist = cellTwistOffsetUpper(vLo, vHi, wLo, wHi)` — held pair to bilinear patch (loft §5.2's
  `maxTwistOffsetUpper` row, per cell).
- `sagitta = max(chordSagitta(sideRadius, th1−th0, n), chordSagitta(capRadius, capTh1−capTh0, n))` —
  bilinear patch between the two chords to the ruled patch between the two arcs: at matched fraction each
  chord point is within its own uniform-angle sagitta of the arc point (loft §5.2's matched paragraph: "a
  circular arc's uniform-angle matched departure equals its sagitta"), and a convex combination of two
  displacements is bounded by their max.
- `skewGap = productUpper(capRadius, capPatchWindowSkew(g))` — ruled-between-arcs to the denoted cone: the
  ruling from side point at azimuth `θ` to cap point at `θ + σ` differs from the cone's generator through
  the side point (which reaches the cap circle at `θ`) by `s·|P_cap(θ+σ) − P_cap(θ)| <= capRadius·σ`, and
  the generator point lies on the cone. Zero where the windows coincide, the same condition under which
  modify-reach §8.3 zeroes the skew half of the normal departure.
- `locusGap = sqrt(L² − c²)/2` with `L = dc·sqrt(speedUpper² + (axialSpan/dc)²)`, `c` the built ruling's
  chord — the boundary ruling (tagged `Line3`) to the conic miter locus it stands for: a curve of length `L`
  between endpoints `c` apart lies inside the ellipse with those foci and major axis `L`, whose semi-minor
  axis is `sqrt(L²−c²)/2`. `speedUpper` is `miterLocusSpeedUpper` (`capblend_contour.go`), the same input
  `chordLocusLengthAllow` reads. Zero at a line-line miter and every reflex foot (both loci affine). Charged
  on BOTH patches sharing the ruling.
- `capRadiusRound = addRoundError(r, ∓d, capRadius)` — the held cap directrix radius against the exact
  offset radius `ivExactOffsetRadius` states.
- `band.delta`, `levelDelta`, `deltaAxial` (`capBandLevel`), `deltaStore` (§3's mechanism over `prismLike`).

`faceBound(patch) = upRound(Σ terms)`; `bound = max`. `areaSlack` per patch: `perturbedTriangleAreaAllow`
over the `twist + sagitta + skewGap + locusGap` displacement plus `contourAllow + bandLevelAreaAllow` the
payload already publishes per patch; the trimmed walls and caps charge the prism's own terms.

### Refusals

| Condition | Result |
|---|---|
| any term above non-finite (`miterLocusSpeedUpper` answering `false`, `capPatchWindowSkew` non-finite) | `ErrUnsupported` |
| a chamfered loop's contour ring and its original ring at the other cap cannot clear each other's sagitta tubes | `ErrDegenerate` via `tessellationExpectedError` (`requireLoopClearance`) |
| `n(w)` exceeds `maxChordsPerWalk` | `ErrUnsupported` (`errTooManyChords`) |
| directed-edge, link, or zero-area audit fails | `ErrUnsupported` |
| a `chamferCap(...)` role or a `side(i,j)` role resolves to no face | `ErrDegenerate` |
| cap blend in a boolean | `ErrUnsupported` through R0's `operandSymDiff` (export-only) |

### Tests (R6)

- Square plate chamfered on one cap loop (all `Plane` patches, unplaced): `Bound() == 0` when
  `band.delta == 0` and the levels are exact; watertight; `meshVolume` equals `body.Volume().Value` exactly.
- Disk chamfered on a cap loop (`wholeTurn` `Cone` patch): quads planar (assert the four corners' coplanarity
  in `big.Rat`), `Bound()` equals the larger of the two rings' `chordSagitta` plus the level terms, and dense
  sampling of the frustum finds no point past `Bound()`.
- Rounded rectangle chamfered on a cap loop (miter corners, windows differ): `Bound() > sagitta` by at least
  `capRadius·skew`; `requireWatertight`; every `chamferCap` face appears in `SourceFaces()`.
- Reflex (notched) profile: the apex fan has one interned vertex and one link cycle.
- A cap-blend body in `Union` refuses `ErrUnsupported`; `Verify` reads the pair `Suspect`.
- `capblend_test.go`'s tessellation staging assertion flips to success.

## 8. Recorded decisions this design changes

Stated separately, per `CLAUDE.md`'s rule. Everything else here extends a design; these change one.

| Doc | Section | Sentence today | Change | Reason |
|---|---|---|---|---|
| `docs/modify-reach-design.md` | §12 Table DX, row DX4, `capBlendPayload` cell | "available once DX3 exists" | "available once DX3's occupied-volume proof lands (`docs/tessellation-reach-design.md` §7); export-only tessellation does not admit it" | tess §2 makes boolean admission conditional on `volSymDiff`, which R6 does not prove |

The `loftPayload` doc comment's "deliberately NOT a `loftPayload` field" paragraph is a code comment, not a
design decision; R1 rewrites it to say the payload stores the COMPOSED proof terms and why that keeps the
"nothing stored can disagree with the records" property.

## 9. Open questions

- **R6 occupied-volume proof.** Decide between (a) a per-cell certified interval integral over the explicit
  homotopy held-facet → bilinear → ruled → cone, tess §11's `Icell` shape, and (b) restricting boolean
  admission to bands whose every patch is `Plane`, `wholeTurn`, or tangent-joined (where the chain collapses
  to vertex motion and `sweptVolumeAllow(faceBound, perturbedAreaUpper)` is already a proof). Choose (b)
  first if any cap-blend boolean is wanted before (a) is engineered; (a) is the complete answer.

## 10. Tasks

Ordered. Each is independently reviewable. "Pattern" names the file whose existing shape the task copies.

### R0 — proof record

1. **Files:** `tessellate.go`. **What:** add `faceBound`, `volSymDiff`, `symDiffOK` to `Mesh`; make
   `tessellateFaceted` fill them from `facetedPayload` (`faceBound[f] = fp.meshBound` per face, or a tighter
   per-face certificate where the payload holds one). **Tests:** `tessellate_internal_test.go` asserts the
   three fields on a faceted restatement.
2. **Files:** `tessellate.go`. **What:** split `walkAreaSlack` into `walkWallSlack` and `walkSegmentArea`
   (the exact `Σ a_c`), keep the composed helper. **Depends on:** none. **Tests:** existing `areaSlack`
   assertions unchanged; new internal test pins `walkSegmentArea` on a quarter circle.
3. **Files:** `tessellate.go`, new `tessellate_station.go`. **What:**
   `chordStationBound(w segmentWalk, k, n int) walkEndBound` for a circular walk's interior sample —
   `circularWalkEndBound`'s mechanism at fraction `k/n` (`turnSinCosInterval` for `CircleSeg`,
   `radSinCosSpan` over `atan2Interval` for `ArcSeg`). **Pattern:**
   `extrude.go`'s `circularWalkEndBound`, `moments.go`'s `circularEndpointInterval`. **Tests:** internal:
   a quarter-turn `CircleSeg` sample at `k/n = 1/2` publishes a bound within 4 ulps of `r`; an `ArcSeg` sample
   publishes a finite positive bound; a non-derivable enclosure answers `+Inf`.
4. **Files:** `tessellate.go`. **What:** `deltaStore` for prism and cup — max over emitted vertices of
   `walkEndBoundAllow(chordStationBound)` and `exactPrismPointRound` (`extrude.go`); per-face `faceBound`
   per §3's table; `perturbedTriangleAreaAllow` per triangle into `areaSlack`. **Depends on:** 1, 3.
   **Tests:** `tessellate_test.go`: §3's plate/rotated-plate assertions; `Bound()` under identity axis-aligned
   placement is unchanged from today's value for `holedPlateBody`.
5. **Files:** `tessellate.go`. **What:** `volSymDiff` per §3's prism/cup formulas; `symDiffOK = true`.
   **Depends on:** 2, 4. **Tests:** the cylinder circular-segment example (§3).
6. **Files:** `boolean.go`. **What:** `facesOfMesh` reads `m.faceBound`; delete `faceChordDelta` and
   `sectionDisplacementOf` if unreferenced; `operandSymDiff` returns `(float64, error)` reading
   `m.volSymDiff` under `m.symDiffOK`, else a staging `ErrUnsupported`. **Depends on:** 1, 4, 5.
   **Tests:** `boolean_internal_test.go` reconstructs a prism×prism result bound from the two `volSymDiff`s;
   `prism_boolean_displacement_test.go`'s existing refusal fixtures still refuse.

### R1 — loft

7. **Files:** `loft_moments.go`. **What:** `loftChordedAllow.twistAreaAllow`, accumulated in
   `computeLoftChordedAllow`'s chorded-cell loop via `cellTwistAreaAllow`. **Tests:**
   `loft_moments_internal_test.go`: zero for a `LineSeg`-only build, positive for an `ArcSeg` pair with a
   twisted cell, exactly the per-cell sum.
8. **Files:** `loft_build.go`. **What:** `loftMeshProof`; `loftPayload` gains `capStartCount`, `cell`, `side`,
   `proof`; `evalLoft` composes §4's three terms after `mass.chorded` and stores them; rewrite the
   `sectionDelta` doc comment paragraph §8 names. **Depends on:** 7. **Tests:** internal: `proof.facetDeparture
   == delta` for a placed `LineSeg`-only loft; `== 0` unplaced pinned; `> sectionDelta` for a chorded pair
   with twist.
9. **Files:** `tessellate.go`, new `tessellate_loft.go`. **What:** `tessellateLoft` per §4; dispatch in
   `tessellateContext`; update `Body.Tessellate`'s doc comment. **Pattern:** `tessellateFaceted`.
   **Depends on:** 6, 8. **Tests:** §4's list in `loft_test.go` and `export_test.go`.
10. **Files:** `docs/loft-design.md` §9 Table D rows D1/D2 (status cells become current state), `doc.go`'s
    support map. **Depends on:** 9.

### R2 — free-form prism

11. **Files:** `spline_sagitta.go`. **What:** generalise `sagittaStationWalk` to a slice of sides; add
    `chainStations` and `freeformChain` per §5; `pairStations` keeps its signature. **Tests:**
    `spline_sagitta` internal tests: `chainStations` on one side of an existing `pairStations` fixture
    returns the same station parameters that side had; the cap refuses at the same count; metering test
    (`spline_sagitta_metering_internal_test.go`) still passes.
12. **Files:** `tessellate.go`. **What:** the `walkFreeform` arm of `chordLoop` per §5 (stations, reversal,
    rounding, `faceOf`, `areaSlack`, `volSymDiff` terms); remove the `requireAnalyticWalk` call there.
    **Depends on:** 4, 5, 11. **Tests:** §5's list in `extrude_freeform_test.go` and `export_test.go`.
13. **Files:** `docs/spline-design.md` Table C rows (`Tessellate`, booleans, interference read current
    state), `doc.go`. **Depends on:** 12.

### R3 — revolve line generators

14. **Files:** `revolve.go`. **What:** extract `revolveLoopWalks` from `buildRevolveLoop`, returning the axis
    walks, their kinds, `singleClosed` and the plane-local walks the proof reads. **Tests:** existing revolve
    tests unchanged.
15. **Files:** new `tessellate_revolve.go`. **What:** `revolveMeridianSamples`, `requireRevolveAxisIncidence`,
    `revolveExtents`, `revolvePreflightFacets`, rings, poles, `emitRevolveCell`, `emitRevolveCaps`,
    orientation; dispatch; refuse circular walks. **Pattern:** `tessellateCup` for ring sharing and cap
    emission. **Depends on:** 6, 14. **Tests:** R3's list in new `tessellate_revolve_test.go`; export byte
    identity there too.
16. **Files:** new `tessellate_revolve_proof.go`. **What:** `revolveAngularSequence` and its certified trig,
    `deltaC` (`revolveIdealBasis`/`revolveMeridianEnclosure`/`revolveIdealPoint`), `deltaR`
    (`exactRigidPointRound`), `revolveBudget`, `revolveContactAudit`, `requireVertexLinks`, `Ecell` by sign
    decomposition, coordinate-stage area terms. **Pattern:** `loft_audit.go` for the pair audit;
    `capblend_contour.go` for `ratInterval`. **Depends on:** 15. **Tests:** internal: `absLinearIntegral`
    against a dense reference; `Ecell` against the closed form on a cylinder cell and a pole fan; `deltaR`
    zero under identity and positive under a rotation; a pinched link and a crossing pair both refuse.
17. **Files:** `export.go` doc comments, `tessellate.go`'s `Body.Tessellate` doc, `doc.go`, `boolean.go` and
    `errors.go`'s "cannot tessellate" wording. **Depends on:** 15.

### R4 — revolve circular generators

18. **Files:** `tessellate.go`, `tessellate_revolve.go`. **What:** `chordCount` gains `nMin`; circular
    meridian chording; axis-to-axis minimum; `requireWalkClearance`; deterministic refinement loop; ring
    collapse detection. **Depends on:** 16. **Tests:** R4's fixtures.
19. **Files:** `tessellate_revolve_proof.go`. **What:** `Ecell` by certified subdivision for sphere/torus
    cells; circular-segment cap areas. **Depends on:** 18. **Tests:** the inner-torus sign-changing cell.

### R5 — revolve booleans

20. **Files:** new `tessellate_revolve_volume.go`. **What:** `Mmeridian`, `Icell`, `Mconstruct`, `Mround`,
    `volSymDiff_revolve`; `symDiffOK = true`. **Depends on:** 19. **Tests:** R5's integrator and boolean
    fixtures. Every boolean fixture sweeps a QUARTER or HALF turn: a full turn's angular count makes its
    facet-pair audit the most expensive thing in the suite, and the geometry it would add is already covered
    by the export tests.
21. **Files:** `boolean.go`, `tessellate_revolve.go`. **What:** extract `resolveRevolve` — the
    count-independent half of `planRevolve` — so `pairChordTolerance` can read `deltaC + deltaR` for a
    revolve operand without a tolerance to plan against, and raise above it. **Depends on:** 20.
    **Tests:** a small revolve at a large coordinate is meshed at the raised tolerance and refused at the
    diameter-derived one.
22. **Files:** `docs/tessellation-design.md` §11's last paragraph and §12's last row (current state),
    `doc.go`. **Depends on:** 20.

### R6 — cap-loop chamfer

23. **Files:** `capblend.go`, `capblend_geom.go`. **What:** store each band's `delta` on the payload
    (`bandDelta`), keyed by `(loop, cap)`. **Tests:** internal: the stored value equals `capBandResult.delta`.
24. **Files:** new `tessellate_capblend.go`. **What:** shared count `n(w)`, rings, cap contour rings, patch
    cells, apex fans, cap faces, the §7 term table, refusals; dispatch. **Pattern:** `tessellateCup` for
    rings, `tessellate_revolve.go` for cone cells and fans. **Depends on:** 9 (twist helper use), 15, 23.
    **Tests:** §7's list in new `tessellate_capblend_test.go`.
25. **Files:** `docs/modify-reach-design.md` §12 DX3/DX4 cells (§8's change), `doc.go`. **Depends on:** 24.
