# Tessellation Reach Design

Design of how `Body.Tessellate` reaches the payload classes beyond `facetedPayload`, `cupPayload` and an
analytic-walled `prismPayload`. Companion to `agent-docs/tessellation-design.md` ("tess §N"), which stays the normative owner of
the mesh contract, the proof record, the shared chording rules, and the revolve theory (tess §§8–11). This
document owns three things tess does not: the increments R0-R6 that implement tess §13's T2–T4 and T6, the extruded free-form chording tess §12 hands to `agent-docs/spline-design.md` §10 P5, and the
cap-loop chamfer tessellator (`capBlendPayload`), which has no tess row at all. Every geometric claim tess
already states is cited, never restated.

Router (navigation, not authority):

| Your question | Section |
|---|---|
| what tessellates, and what the proof record actually holds | §1 |
| the increments and what each adds | §2 |
| what `Mesh` carries so a restatement is boolean-sound | §3 (R0) |
| loft restatement: where the triangle set lives, what the proof terms are | §4 (R1) |
| free-form prism chording inside `chordLoop` | §5 (R2) |
| revolve T2/T3/T4: which tess paragraph each function discharges | §6 (R3–R5) |
| cap-loop chamfer: proof-record row, cells, refusals | §7 (R6) |
| recorded decisions this design changes | §8 |
| open questions | §9 |
| where each increment's code lives | §10 |

## 1. Current state

`tessellateContext` (`tessellate.go`) hands the body to `tessdispatch.Body` (`internal/tessdispatch/dispatch.go`),
which dispatches on the payload class and refuses an unlisted class with `ErrUnsupported`. The classes this
document covers are `facetedPayload`, `cupPayload`, `prismPayload`, `loftPayload` (R1), `revolvePayload` (§6, R3-R5)
and `capBlendPayload` (§7, R6). A free-form-walled `prismPayload` chords on the prism path: `tessellation.ChordLoop`
(`internal/tessellation/loop_chord.go`) switches on the walk kind and gives a `survey2d.WalkFreeform` walk its own
dyadic station chain (§5, R2). `revolvetess.TessellateRevolve` meshes every ANALYTIC generator, line and circular
alike (T2, T3), and a Tier A free-form meridian with proven axis clearance (§6).

`Mesh` carries tess §2's proof record complete, populated by all five of those paths (§3, R0):

| tess §2 proof | Held on `Mesh` | Consumer |
|---|---|---|
| `sourceBound(face)` | `faceBound`, one entry per source face | `facesOfMesh` (`internal/meshbool/tangency_gate.go`), for the hidden-tangency pre-pass |
| `areaSlack` | yes, the analytic terms plus a per-facet coordinate allowance | boolean area composition |
| `volSymDiff` + `symDiffOK` | yes; `symDiffOK` is true for solid prism, cup, loft, revolve and faceted paths and for a cap-loop chamfer whose every band §7 admits, and false for a loft sheet and for a cap-loop chamfer carrying a band §7 does not admit | `meshbool.OperandSymDiff` (`internal/meshbool/operand_prep.go`), which refuses a mesh carrying no proof |
| `deltaStore` (tess §5) | charged per vertex into `faceBound`, `areaSlack` and `volSymDiff` | — |

Every restatement below publishes into that record, and no consumer infers a term the mesh did not state. A
`Plane` face with `Line3` edges publishes zero only where its own polygon and its own stored coordinates are
both proved exact, so a placed loft's planar facets cannot enter the hidden-tangency pre-pass at 0 while
sitting `delta` off their true position. `bound * heldArea` is nowhere a substitute for the occupied-volume
proof: for a chorded loft that proof composes `SweptVolumeAllow(delta, proofbound.PerturbedAreaUpper)` with `wallLeg`,
`skirtLeg` and `twistVolumeUpper` (tess §2's row), and for a revolve tess §11 forbids the product outright. A
payload whose occupied-volume proof has not landed returns a mesh with `symDiffOK` false, which serves export
while the boolean refuses the operand.

## 2. Increments

| # | tess/spline label | Lands | Reason for its place |
|---|---|---|---|
| **R0** | tess T1 completion | `Mesh` carries `faceBound`, `volSymDiff`, `symDiffOK`; prism/cup/faceted populate them; boolean reads them | every later row publishes into this record (§1) |
| **R1** | tess T6 | `loftPayload` exact restatement + boolean admission | the payload already holds the triangle set; admits loft booleans and interference |
| **R2** | spline §10 P5 | free-form arm of `chordLoop`: dyadic station chain over `internal/freeform/spline_sagitta.go` | runs on the prism path; spline Table C orders free-form export, booleans and interference "before revolve" |
| **R3** | tess T2 | revolve, line generators only: cylinder/cone/plane/axis, poles, partial caps, both coordinate proofs, `Ecell`, export | first revolve export |
| **R4** | tess T3 | revolve circular generators: sphere/torus, axis-to-axis minimum, intra-loop tube clearance, circular `Ecell` | extends R3's cells |
| **R5** | tess T4 | `Mmeridian` + certified `Icell`; `symDiffOK` true; revolve admitted to booleans | the revolve occupied-volume proof |
| **R6** | new: T7 | `capBlendPayload` tessellator: export for every band; `symDiffOK == true` for a band of line-line miters, exactly tangent joins and whole turns, and for a draft band's charged near-tangent joins (§7), `false` for every other band | reuses R3's cone/pole cells and R1's twist term |

R6 depends on R3 only. R5 and R6 are independent.

No increment returns a mesh before its row's boundary and area proofs are met (tess preamble).

## 3. R0 — the proof record on `Mesh`

### Responsibility

Make `Mesh` the single carrier of tess §2's private proofs, and make the boolean read them instead of
inferring them.

### Interface (private)

`topo.Mesh` (`internal/topo/mesh.go`) carries the record as exported fields (`FaceBound`, `VolSymDiff`,
`SymDiffOK`). This sketch uses the proof's own names:

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
- `facesOfMesh` reads `m.faceBound[f]` and no separate per-face chord-delta helper exists. For prism/cup a planar face with straight edges publishes
  `deltaStore + sectionDelta + deltaAxial(face)`, which is at least `booloperand.sectionDisplacement`.
- `meshbool.OperandSymDiff` returns `m.n.VolSymDiff` when `m.n.SymDiffOK`, else `ErrUnsupported` (a staging refusal,
  routed through `expectedBooleanForOperand(meshbool.BooleanExpectedStaging, …)` like a tessellation refusal).
  `facetedrecord.Restate` sets `volSymDiff = fp.VolSymDiff, symDiffOK = true`.

### Prism and cup population

Per tess §5/§6, with the terms the current code already computes plus the two it does not:

| Term | Source |
|---|---|
| `deltaTrim(face)` | wall: its walk's `chordCount` sagitta; cap/floor/rim: max sagitta over the loops bounding that patch (`chordedLoop.maxSag`) |
| `deltaSection` | `pp.SectionDelta` (0 for a cup) |
| `deltaAxial(face)` | `z0Delta` for `capStart`, `z1Delta` for `capEnd`, `max` for a wall; cup floors/rims read `cp.zDelta` per level |
| `deltaStore(face)` | Two mechanisms, max over the face's vertices: (a) a computed circular sample's own gap from the certified enclosure of the point it denotes — `circularWalkEndBound`'s mechanism (`extrude.go`) applied at interior fraction `k/n` through `turnSinCosInterval` for a `CircleSeg` and `radSinCosSpan` over `atan2Interval` for an `ArcSeg` (`internal/momentregion`, `internal/proofbound/turn_trig.go`), carried by `proofbound.WalkEndBoundAllow`; (b) `prismrecord.Payload.PointRound(pp, u, v, z, held)` (`extrude.go`) for the frame/placement write. Both are zero for a recorded line vertex under an axis-aligned identity payload |

`faceBound(f) = upRound(deltaTrim + deltaStore + deltaSection + deltaAxial)`. `bound` stays the conservative
composition tess §5 already permits (largest sagitta + payload-wide axial max + `deltaStore` max).

`areaSlack` adds `proofbound.PerturbedTriangleAreaAllow(a, b, c, deltaStore)` per triangle (tess §5's
`Rarea_triangle`) after the existing analytic terms.

`volSymDiff` per tess §5/§6:

```text
Mprism_analytic = |z1-z0| * Σ_walks segs(walk)          // WalkSegmentArea, once per walk
Msection        = sectionDisplacementArea(δs, walks, perimeterUpper) * |z1-z0|
Maxial          = z0Delta*capAreaUpper + z1Delta*capAreaUpper
Mstore          = proofbound.SweptVolumeAllow(deltaStore, proofbound.PerturbedAreaUpper(verts, tris, deltaStore))
volSymDiff      = upRound(Mprism_analytic + Msection + Maxial + Mstore)
```

`capAreaUpper` is the region area upper bound the payload already publishes (`body.area`'s cap share) or the
chorded polygon area plus `areaSlack`. The cup composes `hO*E(O) + hC*E(C)` (tess §6) in place of the first
term. `symDiffOK = true` for both.

`tessellation.WalkWallSlack` (the wall half) and `tessellation.WalkSegmentArea` (the exact `Σ a_c` per walk) are separate functions, so `segs` is reachable on its own.

### Failure behavior

A non-finite `deltaStore`, `faceBound`, or `volSymDiff` refuses `ErrUnsupported` (tess §12). `stationbound.ChordStationBound`
answering `+Inf` for an underivable enclosure is the same refusal.

### Tests (R0)

- Plate with a circular hole (`holedPlateBody`, `apitest/tessellate_test.go`): every wall face of the hole publishes
  `faceBound >= chordSagitta(r, 2π, n)`; each cap publishes at least that; every straight outer wall publishes
  exactly 0 under identity placement.
- The same plate under a rotated placement: every `faceBound > 0`, and `Bound` exceeds the unplaced `Bound`
  by at most `radius3D(16·ulp(coordMax))` (`RigidRoundAllow`'s own scale).
- tess §14 "Check prism/cup `volSymDiff` against exact circular-segment examples": a full cylinder of radius
  `r`, height `h`, `n` chords: `volSymDiff >= h * (π r² − n r² sin(2π/n)/2)` and at most twice it.
- A boolean over two prisms reads `volSymDiff`: assert the result's `Volume().Bound` equals the sum of
  the two operands' `volSymDiff` plus the weld term, bit for bit, by reconstructing it in an internal test.

## 4. R1 — `loftPayload` exact restatement (tess T6)

### Where the triangle set lives

`loftPayload` (`loftrecord.Payload`) holds `verts`, `tris`, `walls`. `tris[:walls]` are the wall triangles in
`loftAssembly.cell`/`side` order; `tris[walls:walls+capStartCount]` are `capStart`'s, the rest `capEnd`'s.
`capStartCount`, `cell` and `side` live on `loftAssembly` only. `matchedDelta` and
`loftChordedAllow.maxTwistOffsetUpper` are locals of `loftrecord.Build`. The payload stores the COMPOSED proof
terms, regenerated on every `Placed` (which re-runs `loftrecord.Build`), never the raw `matchedDelta`, so nothing
stored can disagree with the records:

```go
// loftmesh.MeshProof is tess §2's loftPayload row, composed once by loftrecord.Build.
type MeshProof struct {
    FacetDeparture float64 // proofbound.AbsSumUpper(matchedDelta, maxTwistOffsetUpper)
    AreaSlack      float64
    VolSymDiff     float64
}
```

`loftPayload` also holds `capStartCount int`, `cell [][2]int`, `side []uint8` and `proof loftmesh.MeshProof`, all
copied from `loftAssembly` or composed at the end of `loftrecord.Build`. `Placed` nils the triangle fields and
re-evaluates.

### Composition (in `loftrecord.Build`, after `mass.chorded` is set)

| Term | Composition | Owner of each input |
|---|---|---|
| `facetDeparture` | `proofbound.AbsSumUpper(loftmesh.ChordCellDeltaUpper(sectionMatchedDelta, a.delta), chorded.maxTwistOffsetUpper)` — computed UNCONDITIONALLY, so a `LineSeg`-only placed loft publishes `a.delta` (loft §5.2's facet-departure row: "`matchedDelta` reduces to `delta`") | loft §5.2 `matchedDelta`, `maxTwistOffsetUpper` rows |
| `areaSlack` | `upRound(mass.perturbAreaSum + chorded.twistAreaAllow + chorded.areaExcess + chorded.capAreaExcess)` | tess §2's loft row: the per-triangle perturbation sum, the wall's three legs (`cellTwistAreaAllow` held-to-bilinear, `cellChordCurveAreaAllow` + `cellStationShiftAreaAllow` already summed in `areaExcess`), and the two caps' `capAreaAllow` |
| `volSymDiff` | `proofbound.AbsSumUpper(proofbound.SweptVolumeAllow(a.delta, proofbound.PerturbedAreaUpper(verts, tris, a.delta)), chorded.wallLeg, chorded.twistVolumeUpper, chorded.skirtLeg)` | tess §2's loft row; the twist measure joins `Volume`'s legs because the mesh holds the uncorrected triangles (`agent-docs/loft-gear-bounds-design.md` §2) |

`loftChordedAllow` carries `TwistAreaAllow`, the sum of `cellTwistAreaAllow(vLo, vHi, wLo, wHi)` over the same
chorded cells `computeLoftChordedAllow` already walks (gated on `p.matchedDelta[j] > 0`, never on kind).

### `loftrecord.Tessellate` and `internal/loftmesh.RestateLoft`

`loftrecord.Tessellate` (`internal/loftrecord/tessellate.go`) is the adapter: it numbers the body's face nodes in order, maps each provenance
role to its face number, and hands `RestateLoft` the payload's triangle set, split and proof record. It
maps the returned face numbers back to face nodes and calls `topobuild.SetFaceBound`. `RestateLoft` does steps 1-4.

1. Build the role-to-face map from `b.Faces()` as the prism does. For `k < walls`: face of role
   `side(cell[k][0], cell[k][1], side[k])`. For a solid, `capStartCount` triangles → `capStart`,
   rest → `capEnd`. A missing role is `ErrDegenerate` (tess §4).
2. Copy `verts` and the selected triangle range (fresh slices — `Mesh` accessors already copy, but payload slices must not
   alias the mesh's). No chording, no retriangulation, no reflection flip: loft §5's whole-shell orientation
   step already made every triangle outward, and `Placed` re-runs it, so tess §4's "reflected placement
   reverses once" rule is discharged by the payload. Assert the tetrahedron sum positive in the audit below.
3. `faceBound[f] = proof.FacetDeparture` for every face; `bound = FacetDeparture`; `AreaSlack` from `proof`.
   A solid also publishes `volSymDiff` and `symDiffOK = true`.
4. A solid runs `RequireClosedMesh` and the signed-volume audit. Either failing is `ErrUnsupported` (tess §12).

A surface-result sheet copies only `tris[:walls]`, using the recorded split
rather than a geometric cap test. Its live `side(i,j,k)` faces supply sources;
the absent cap roles are never looked up. It runs `RequireSheetBoundary` and
`RequireSheetVertexLinks`, and keeps `symDiffOK` false. The payload's
`areaSlack` includes nonnegative cap allowances and remains conservative over
the retained wall triangles. Its open triangle set cannot use the signed-volume
audit, because that sum depends on the anchor without caps.

No tolerance refusal: the tessellation adds no chording of its own, and the whole `Bound` is inherited
payload displacement, which tess §1's Tolerance row lets ride above `tol` (the same standing as a prism's
`deltaAxial`). `defaultChordTolerance` is unchanged for a loft.

### Boolean effect

Through R0, a loft whose `facetDeparture == 0` enters the existing all-planar zero-bound path; every other
loft is an ordinary positive-bound all-planar operand via the per-pair rim bound (loft Table D D2). `Verify`'s read-only
`OpIntersect` follows.

### Tests (R1)

tess §14's loft bullet in full, plus:

- `TestLoftTessellate` (`apitest/tessellate_loft_test.go`) checks: mesh non-nil; triangle count
  `== 2·Σstations + capTris`; `requireWatertight`; `meshVolume(mesh)` within `body.Volume().Bound()` of
  `body.Volume().Value`; every `SourceFaces()` entry is one of `b.Faces()`; the mesh vertex set equals the
  body's `Vertices()` positions exactly.
- Unplaced pinned `LineSeg`-only loft: `Bound() == 0`, and `Union(loft, prism)` reports an `Exact` volume
  equal to the analytic sum minus overlap.
- Transformed `LineSeg`-only loft: `Bound() == delta` (read `Bounds().Bound` from the body, which equals
  `proofbound.AbsSumUpper(delta, 0)`), and `Cut(prism, loft)` succeeds with a positive `Volume().Bound`.
- Chorded `ArcSeg` pair loft: `Bound()` strictly greater than `Bounds().Bound()` whenever
  `maxTwistOffsetUpper > 0` (assert through a helical-tooth fixture).
- Byte-identical `STL` on two calls.

## 5. R2 — free-form prism chording (spline §10 P5)

### Responsibility

`tessellation.ChordLoop` has a `survey2d.WalkFreeform` arm so an extruded Tier A free-form wall chords, with a proven sagitta,
station displacement and area slack, on the existing prism path. Nothing else in `tessellateContext`
changes: `tessellation.RequireLoopClearance`, `triangulate2DContext`, `earClip` and the wall/cap emission already consume
2D samples.

### Machinery

`internal/freeform/spline_sagitta.go` and `internal/freeform/spline_stations.go` are the machinery. They own `dyadicSpanSagittaUpper` (spline §6.2.1's control-point-to-
chord-SEGMENT bound), `dyadicSpan.split` (`internal/freeform/spline_length.go`, exact midpoint de Casteljau), `spanSpeedUpper`
(a proven arc-length upper bound), and `freeform.SagittaStationWalk`, a two-sided measure-then-bisect walker built for
the loft's paired chains. `freeform.ChainStations` is the ONE-sided sibling:

```go
// ChainStations chords one Tier A span chain at target sagitta: measure each dyadic cell's
// sagitta, bisect what misses, accept what fits. spline §6.2.1; the single-chain twin of
// freeform.PairStations, sharing its cap, its charge discipline and its termination argument.
func ChainStations(spans []BezierSpan, target float64, work *FreeformWork) (FreeformChain, error)

type FreeformChain struct {
    stations    []ratPoint // exact points ON the curve, one per cell boundary, chain start included, end excluded
    end         ratPoint   // the excluded final boundary, copied so it aliases no input span
    sagitta     float64    // measured post-subdivision MAXIMUM over cells (never a sum)
    cellArcUpper []float64 // spanSpeedUpper of each accepted cell's own sub-span
    cellChordLower []float64 // proven lower bound on each accepted cell's chord length (chargedRatSqrtDown)
}
```

`freeform.SagittaStationWalk` measures, accepts and bisects over a SLICE of sides (`[]dyadicSpan`),
and both `freeform.PairStations` (2 sides) and `freeform.ChainStations` (1 side) drive it. What each consumer owes per accepted
cell — the pair's matched delta, the chain's arc/chord bracket — is a `freeform.StationCellReader`, the one place the
two differ. The cap (`freeform.MaxChordsPerWalk`, `freeform.ErrTooManyChords`, spline R8) and the R7 work refusal stay where
they are.

### The `ChordLoop` arm

For a `survey2d.WalkFreeform` walk `w` (`w.spans`, `w.reversed`):

1. `chain := freeform.ChainStations(w.spans, chord, work)`; a reversed walk emits stations from the chain's end
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
| chain needs more than `freeform.MaxChordsPerWalk` cells | `ErrUnsupported` (`freeform.ErrTooManyChords`, spline R8) | `freeform.ChainStations` at the split |
| exact-rational work exceeds the record-level counter | `ErrUnsupported` (spline R7) | `FreeformWork.Step` inside each primitive |
| a cell's sagitta or speed enclosure is non-finite | `ErrUnsupported` | `chargedRatSqrtUp` / `spanSpeedUpper` |
| a station's `Point2` rounding gap is non-finite | `ErrUnsupported` | the arm, before emission |
| the chorded loop self-intersects or pinches | `ErrDegenerate` via `tessellation.ExpectedError` | `triangulation.EarClip`, `tessellation.RequireLoopClearance` |

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
- A 15-point involute `FitSplineSeg` prism (`apitest/extrude_test.go`'s `involuteFlankSketch`): byte-identical `STL`
  and `OBJ` on two calls. On the fit-spline arch: `Cut(prism, box)` where the box lies strictly inside
  reports `Volume` within `Bound` of `prism.Volume − box.Volume`; two overlapping free-form prisms make
  `Verify` decide their pair with a proven `Interference` row.
- `apitest/extrude_freeform_test.go` expects success from tessellation, export, boolean and interference, and
  refusal from clearance and survey, since neither reads the mesh.

## 6. R3–R5 — revolve (tess T2/T3/T4)

tess §§8–11 are the theory; this section maps each paragraph to code. No new geometry is stated here.

### Files

| File | Owns |
|---|---|
| `internal/revolvetess/revolve.go` | `revolvetess.TessellateRevolve`: section gates, cell and cap assembly, orientation, and the assembled mesh's own audits (tess §8, §9) |
| `internal/revolveplan/` | Walk resolution, axis-incidence gate, coordinate ceilings, meridian and angular counts, and count refinement (tess §8, §9) |
| `internal/revolvesampling/meridian.go` and `section.go` | Certified meridian junctions, circular stations and section readings (tess §8–§9) |
| `internal/revolvemesh/revolve_ring.go` and `internal/revolvemesh/tessellate_revolve.go` | Ring, cell and cap emission, indices, area charge, and construction and placement rounding measurements (tess §8–§10) |
| `internal/revolvemesh/revolve_proof.go` | Certified angular samples and axis basis (tess §8) |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, `Ecell`, `Mmeridian`, and `volSymDiff_revolve` composition (tess §8–§11) |

### Shared with the builder

`buildRevolveLoop` (`internal/revolveplan/build.go`) and the tessellator's plan (`internal/revolveplan/plan.go`) both
resolve a loop's walks through `revolveaxis.ResolveLoopWithFreeform` (`internal/revolveaxis/recorded_walks.go`),
whose `ResolvedWalks` carries the coalesced axis walks, their kinds, `SingleClosed`, and the same walks still in
PLANE-local coordinates indexed by recorded segment. Because both call it, so the mesh reads exactly the walks the body
was built from (tess §3: "read the evaluator's payload, NEVER live sketch input"), and the proof reads the
recorded plane point the axis re-expression consumed rather than the re-expressed floats, which
`axisFrame.walk` states no axial bound for and SNAPS a near-axis radial one to zero. The evaluator §6
axis-incidence audit runs from the tessellator as `revolvesampling.RequireAxisIncidence` (tess §9's "also run").
`revolveplan.Payload.Basis()` gives `a3, w, e0, e1`; the tessellator evaluates `X(z, ρ, φ)` UNPLACED in
binary64, stores, then applies `rp.xform` once (tess §8's two stages) — it does not use `rp.point`, which
composes both in one expression.

### R3 (T2) — what each piece discharges

| tess paragraph | Function | Notes |
|---|---|---|
| §8 profile → `(z, ρ)`, `rhoMax`, `zAbsMax`, `coordMax` | `revolveproof.Extents(walks)` | endpoints plus cardinal points inside a circular walk's interval; non-positive `rhoMax` is an invariant failure |
| §8 `deltaC` | `revolvemesh.IdealBasis`, `revolveMeridianEnclosure`, `revolveIdealPoint` | Certified intervals enclose each unplaced vertex; `deltaC` bounds its stored float coordinate. |
| §8 `deltaR` | `min(RigidRoundAllow(coordMax + deltaC, translationMax), revolvemesh.ExactRigidPointRound per vertex)` | the second term measures the same displacement exactly, so it is 0 under an exact identity and tighter elsewhere; `prismrecord.Payload.PointRound`'s own mechanism |
| §8 budget order | `revolvemesh.RevolveConstructionPrior` + `RigidRoundAllow` before the count, the measured pair after | §8 splits the tolerance BEFORE the counts, and the count decides how many angles there are, so the split spends count-INDEPENDENT ceilings: the meridian gap (per sample, not per angle), the stored trig's own ceiling, and the evaluation's ulps at `coordMax`. The measured `deltaC`/`deltaR` are checked against them and refuse on a violation, so the a-priori figure is held to account rather than trusted |
| §8 tolerance split | `revolvemesh.RevolveBudget(tol, deltaC, deltaR)` | `available = downRound(downRound(tol − deltaC − deltaR))`; `<= 0` refuses; meridian gets `available/2`; angular gets `available − deltaM` |
| §3/§8 counts | `tessellation.ChordCount` for each circular meridian walk (R4) and for the global angular sequence with `radius = rhoMax`, `sweep = |φ1−φ0|` or `2π`, `closed = full` | the downward-then-upward correction and the cap are `ChordCount`'s own; a section with no circular meridian walk has `deltaM == 0` |
| §9 section proof | `revolvesampling.SectionPoints` + `tessellation.RequireLoopClearance` over the `(z, ρ)` samples, run for a full and a partial sweep alike before any cell is formed (line-only sections: the homotopy is the identity, so the endpoint checks are the proof) | R4 adds the intra-loop tube check |
| §9 rings + poles | `revolvetess.TessellateRevolve`'s ring loop, read back through `RevMeridian.At` | `ρ > 0` → `nPhi` (full) or `nPhi+1` (partial) vertices; `ρ == 0` → ONE vertex, answered for every angle and for both caps, so the interning is structural rather than a lookup |
| §9 cells | `revolvemesh.EmitCellTriangles` | both off axis → planar quad, fixed `(m_k, φ_l) → (m_{k+1}, φ_{l+1})` diagonal; one on axis → fan; both on axis → nothing only for `wallAxis`, else refuse "erased generator" |
| §9 partial caps | `revolvemesh.EmitCapTriangles`: `triangulation.Triangulate` over the `(z, ρ)` samples | the `(u, v) → (z, ρ)` map is a ROTATION for either axis side, so the recorded loop's own sense survives it and no index order is reversed; the `(z, ρ)` frame's normal is the sweep-velocity direction, so the END cap takes the triangulation as it stands and the start cap reverses it. Pole vertices are ordinary 2D samples mapping to the interned vertex, so the on-axis line's edge is shared by both caps |
| §4 orientation | walk sense × sweep sense, then reflection | rule: with material on the walk's left in `(z, ρ)` and a right-handed sweep about `w`, `∂X/∂t × ∂X/∂φ` is ρ times the outward in-plane normal, so cell triangles are `(m_k,l), (m_{k+1},l), (m_{k+1},l+1)` and `(m_k,l), (m_{k+1},l+1), (m_k,l+1)`; the axis SIDE needs no negation of its own (see the caps row); negate for `rp.reflected()`; then the signed-volume audit |
| §9 positive facet area | `revolvemesh.RequireRevolveFacetAreas(budget, verts, tris, deltaC + deltaR)` | tess §1's Geometry row, linear in the facets, run at EVERY verification level and never inside the pair audit below. It also builds the `revolveAuditTri` data that audit consumes, so the two share one pass over the facets and one work budget |
| §9 endpoint + homotopy audits | `RevolveContactAuditBounded(budget, data, tris, deltaC + deltaR)`, at `VerifyBoundary` and above | ONE pass, at the final stored coordinates, against the COMBINED displacement. Every mesh on either homotopy is a vertex-wise displacement of the stored one by at most that (the placement between the two stages is an exact isometry), and every predicate the classification consults is multilinear in the vertices, so a stored reading exceeding its own perturbation allowance fixes the sign for the whole family. A pair sharing nothing is proven apart by an exact separating axis with the same margin; a pair sharing a vertex or an edge is proven to meet ONLY there by a boundary plane built as a polynomial in the pair's own corners, so the plane keeps containing the shared feature identically. Candidate-sweep visits and exact pair tests share the cumulative `proofbound.MaxFacetPairTestsPerCall` budget (tess §3) |
| §9 link audit | `tessellation.RequireVertexLinks` | every link one connected degree-two cycle; a pinched pole is `ErrUnsupported` |
| §10.1 bounds | `faceBound`: wall `deltaM + deltaPhi + deltaC + deltaR`; partial cap `deltaM + deltaC + deltaR`; exact-line cap `deltaC + deltaR` | `bound` = max |
| §10.2 `Ecell` | `revolvemesh.RevolveCellAreaSlack` / `revolvemesh.RevolveFanAreaSlack` over `revolvemesh.AbsLinearIntegral` | tess §15's choice for T2, recorded there: COMPLETE SIGN DECOMPOSITION in closed form, with no root isolation at all. For a straight generator `Jtrue = L·dφ·ρ(t)` does not depend on `u` and `Jheld` is constant on each half of the domain the fixed diagonal cuts, so the difference is LINEAR in `t`, its single zero is an exact rational quotient, and each sign-fixed piece integrates through its own primitive. `internal/polynomial/sturm.go`'s Sturm engine is not reached, and `cos dφ`/`sin dφ` enter only through the ideal triangle's own area — never inside a root isolation — so the widening §9's open question worried about cannot lose a sign. One evaluation answers for every angular interval: interval `l`'s ideal samples are the exact rotation of interval 0's, and a rotation is an isometry |
| §10.2 caps + coordinate stages | circular-segment areas (R4); `proofbound.PerturbedTriangleAreaAllow(·, deltaC + deltaR)` per facet | a straight generator's cap trim is exact, so a cap contributes no chording area at all; the combined displacement covers the ideal, stored-unplaced and placed triangles alike. Summed upward, after `Ecell` |
| §3 ceilings | `revolveproof.PreflightFacets`: facet count and cumulative facet work with checked unsigned arithmetic before any slice; bounded contact audit charges candidate sweeps and exact pairs afterward | `maxFacetsPerMesh` (`internal/revolveproof/budget.go`) binds at 65_536 facets. `maxFacetWorkPerCall` (262_144) counts every assembled facet across retries. At `VerifyBoundary` and above, candidate-sweep visits and exact pair predicates share the cumulative `proofbound.MaxFacetPairTestsPerCall` ceiling (8,000,000 steps); `VerifyNone` runs neither and is charged no pair work |

`deltaC` uses rational intervals for the axis basis, the meridian `(z, ρ)`
with its endpoint bound and axis snap, and the ideal point `X`. The global
angular sequence uses exact turn intervals for a full turn from zero and
certified radian intervals otherwise. It stores the float nearest each
enclosure's midpoint rather than calling `math.Sincos`. The largest per-vertex
gap uses `proofbound.IntervalFloatError` and the three-dimensional radius.

A Tier A free-form meridian generator with proven axis clearance meshes through certified dyadic stations
(`internal/revolvetess/revolve.go`); a station chain that reaches the axis refuses `ErrUnsupported`. The mesh carries `symDiffOK == true` for an
analytic revolve (R5), so the boolean admits it, and `booloperand.RequireVolumeProof` asks the same question of the
payload class before any mesh is built. The cap-loop chamfer (§7) is the other payload class that question
covers.

### R4 (T3)

- `tessellation.ChordCount` on every circular meridian walk; `deltaM` is the largest proven sagitta (tess §8 step 3).
- Axis-to-axis minimum: a circular walk with both ends on the axis takes `n >= 2` and must produce an
  off-axis interior sample; `ChordCount`'s `nMin` is a parameter (`3` closed, `2` axis-to-axis, `1`
  else). A circular walk with positive interior `ρ` that chords to an axis-only polyline refines, then
  refuses.
- Intra-loop tube clearance: `tessellation.RequireWalkClearance` (`internal/tessellation/section_clearance_error.go`) — every non-adjacent walk
  pair within one loop clears `sag_i + sag_j + floor`, the same gate `tessellation.RequireLoopClearance` runs across loops,
  with the same `tessellation.ExpectedError`. This plus the endpoint checks is tess §9's `Hm` proof.
- Refinement: first failing meridian walk in payload order, rebuild, re-audit; angular failures increment
  the one global count.
- `Ecell` for sphere/torus cells: certified interval subdivision under a fixed budget (tess §15's choice for
  T3), each accepted interval an outward-rounded absolute integral bound.
- Ring collapse detection before and after placement (tess §9): `ErrUnsupported`, never merged into a pole.

### R5 (T4)

`internal/revolveproof/volume.go` computes the §11 bounds (`SymDiff`); tess §15 records the `Icell` choice and
its derivation, and is the authority on both.

- `Mmeridian = sweepAngle * Σ_c |∫_{S_c} ρ dA|`: each circular walk's slivers charged their own proven total
  area (`chordSegmentArea`) times a proven upper bound on `ρ` over them, summed as absolute values.
- `Icell` per cell: BM's meridian is already a polyline, so tess §11's straight-generator homotopy answers for
  a sphere and a torus too. The integral separates into an exact rational meridian factor and one angular
  factor that depends on `dφ` alone, proven ONCE per mesh by certified subdivision (tess §15).
- `Mconstruct`, `Mround` via `proofbound.SweptVolumeAllow` over `proofbound.PerturbedAreaUpper` at the composed displacement, which
  covers every surface on both stages' paths.
- `symDiffOK = true`; `booloperand.PairChord` raises the pair's tolerance above `deltaC + deltaR` of either
  revolve operand (`booloperand.coordDisplacement` over `revolvetess.ResolveRevolve`), as it does for a section
  displacement.

### Refusal mapping (tess §12)

| tess §12 row | Raised by |
|---|---|
| non-positive `available` | `revolvemesh.RevolveBudget` |
| inverse underflow / unrepresentable ceiling / cap | `chordCount` (unchanged) |
| on-axis incidence malformed | `revolvesampling.RequireAxisIncidence` (`ErrDegenerate`) |
| positive-radius ring collapse; erased generator | `revolvesampling.MeridianSamples`, `revolvetess.TessellateRevolve`'s own cell loop |
| meridian simplicity/nesting/clearance | `tessellation.RequireLoopClearance`, `tessellation.RequireWalkClearance` after refinement exhausts |
| non-adjacent facets intersect; homotopy sign not fixed | `revolvemesh.RevolveContactAuditBounded` |
| directed-edge / link / zero area | `tessellation.RequireClosedMesh`, `tessellation.RequireVertexLinks` |
| a payload class with no occupied-volume proof in a boolean | `booloperand.RequireVolumeProof` before the mesh, `meshbool.OperandSymDiff` (R0) after it |

### Tests

tess §14 assigns them. By increment: R3 — full cylinder, partial cone, planar annulus, poles and
apexes (one interned vertex, fans only, one link cycle), partial on-axis line (both caps share one axis
edge), reflected placement, holes, several shells, large-coordinate `deltaC` and non-identity `deltaR`
charged separately, the facet/work ceilings hit exactly and one beyond, byte-identical export; R4 — full
sphere, spherical band, ring torus, concave torus wall, the radius-1 full-turn `n = 122` threshold, the
`2r` and 5 mm budgets, the two-semicircle tangent disk refused `ErrDegenerate` before tessellation, the
narrow-bulge hole below and above the summed tubes, the inner-torus sign-changing `Ecell`; R5 — the interval
integrator on fixed-sign and adversarial cells, revolve×prism and revolve×revolve booleans with a hidden
tangency refusal and a shallow crossing whose rim amplification refuses, the property fuzz across tolerance
scales. `apitest/revolve_property_test.go` tessellates opportunistically.

## 7. R6 — cap-loop chamfer (`capBlendPayload`), tess T7

### Verdict

R6 gives every band an export mesh and gives the bands whose held cells
reproduce the exact offset family slice by slice a boolean mesh ("Occupied volume" below). Every term its proof record
needs is either an existing published term (`band.delta`, `g.levelDelta`, `capBandLevel`,
`capblend.PatchWindowSkew`, `cellTwistOffsetUpper`, `chordSagitta`, `capcontour.MiterLocusSpeedUpper`) or a one-step
derivation from one, stated below. The occupied-volume proof is complete for a band whose every corner is a
line-line miter or an exactly tangent (G1) join, and for the cornerless whole turn: there the band's true
section at every axial fraction is the exact offset section, and the held cells' section is its chord polygon
at fixed azimuths, so the proof is the prism's own two legs — a slice-wise circular-segment integral and a
vertex-motion `proofbound.SweptVolumeAllow`. A draft or cap-loop chamfer band's near-tangent join adds a third, its corner
charge, and a chamfer band's arc whose recorded end is off its start's circle adds a fourth, its restatement
charge (Admission below, §7.1). A prism cap-loop chamfer band's reflex corner adds a fifth, its connector's
vertex motion and chord term ("Reflex connector" below). Every other band — a circular wall at a genuine miter —
keeps `symDiffOK == false`, which tess §2 permits for an export-only mesh. DX3's "a strip whose densities
disagree is not watertight" is answered by sharing ONE count per wall walk across the side wall, the band
patch and the cap contour (below).

### Geometry the payload states

`capBlendPayload` (`capblend.Payload`, `internal/capblend/payload.go`) holds the receiver's `profile`, `frame`, `z0/z1` with their axial
displacements, each chamfered cap's two setbacks (`capSetback`: `dc` across the cap and `ds` down the side,
each with its conversion rounding `dcDelta`, `dsDelta`; `agent-docs/modify-design.md` Part R §8.3.1), the selected
`startLoops`/`endLoops`, and `Patches []capblend.Patch` — every
`chamferCap(cap, loop, k)` role beside its plane-local `capband.Patch` (`internal/capband/patch.go`): a `Plane` patch's
four corners, or a `Cone` patch's centre, `sideRadius`/`capRadius`, side window `th0..th1`, cap window
`capTh0..capTh1`, `sideZ`/`capZ`, `sweepCCW`, `wholeTurn`, `contourAllow`, `levelDelta`, `capThAllow`. Each
band's cap contour displacement `band.delta` is `capblend.BandResult.Delta`; the payload stores it per band
(`BandDelta map[BandKey]float64`, `internal/capblend/payload.go`) for the tessellator to read.

Below, `d` names the setback of whichever axis a term measures. An in-plane term — the cap contour, an
offset radius `R ∓ d`, a connector arc of radius `d`, a G1 foot `P + s·d·n` — reads the cap's `dc`, and its
rounding `dcDelta` reaches it through the band's contour displacement and, for a circular wall's cap sample
bounded against the offset circle at `dc`, as a radial term of its own. An axial
term — a level pulled in by `d`, the band height `dUpper = d + dDelta`, a miter locus's axial span — reads
the cap's `ds` and `dsDelta`. A loop chamfered on both caps takes one pair on both, so a per-loop count or
gap serves either cap.

Faces the body holds (`capblend.Build`, `internal/capblend/build.go`): per loop, trimmed side walls over
`[zLo, zHi]` where a chamfered end pulls its level in by `d` (`prismrecord.BuildLoopSidesAs` over `prismLike`); per
chamfered `(loop, cap)`, one band of patches; two cap faces whose boundary is the offset contour for a
chamfered loop and the original loop otherwise (the `mixed` profile of `capblend.MixedOffsetProfile`).

### Chording — one count per wall walk, shared three ways

A route L band on a brep body (`agent-docs/modify-design.md` Part G Table DG's
DG3) is chorded by the same loop chording over the receiver's loop in its
face's frame. The brep tessellator hands the wall beside each walk the
band's side-ring points as its own samples, so the three share one count and
one set of vertices.

For each loop, resolve its walks as `tessellation.ChordLoop` does. For a circular walk `w` chamfered on cap `c`:

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
feet (`capblend.WallFoot`) and apex arcs. The cap face at `capZ` triangulates through `triangulate2DContext` over
the contour rings of chamfered loops and the original rings of unchamfered ones, after `tessellation.RequireLoopClearance`
over all of them.

### Cells

| Patch | Cells | Departure of a held facet from the DENOTED surface |
|---|---|---|
| `Plane` | one quad (`sideA, sideB, capA, capB` — two parallel segments, coplanar), two triangles | `band.delta + levelDelta + deltaAxial + deltaStore` — a line offset is affine, so the quad IS the denoted family (modify-reach §8.3) |
| `Cone`, proven skew positive | `n(w)` quads between side sample `k` and cap sample `k`, fixed diagonal, two triangles each | `twist + sagitta + skewGap + locusGap + capRadiusRound + band.delta + levelDelta + deltaAxial + deltaStore` |
| `Cone`, proven skew zero (`wholeTurn` or a join whose held ends lie on one ray) | same, but same azimuths both rings → planar quads (tess §9's cone cell) | as above with `twist == 0` and `skewGap == 0`; `locusGap` is zero at a G1 or reflex corner |
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
- `skewGap = productUpper(capRadius, capblend.PatchWindowSkew(g))` — ruled-between-arcs to the denoted cone: the
  ruling from side point at azimuth `θ` to cap point at `θ + σ` differs from the cone's generator through
  the side point (which reaches the cap circle at `θ`) by `s·|P_cap(θ+σ) − P_cap(θ)| <= capRadius·σ`, and
  the generator point lies on the cone. `σ` is linear between the two corners, so the larger corner skew
  bounds it everywhere. `capblend.PatchWindowSkew` is the larger of the patch's two PROVEN corner skews
  (modify-reach §8.4's `CornerSkewUpper`), never a difference of its held float windows, which can fall
  below the exact corner angle. The same number gates `twist`: a patch whose proven skew is positive
  charges it, even where the skew is rounding-level. Zero where both held ends of each corner lie on one
  ray from the centre, the same condition under which modify-reach §8.3 zeroes the skew half of the normal
  departure.
- `locusGap = sqrt(L² − c²)/2` with `L = dc·sqrt(speedUpper² + (axialSpan/dc)²)`, `axialSpan` the stated
  side setback (at most `|ds| + dsDelta`) and `L` summed over 32 offset sub-ranges that share their ends —
  the boundary ruling (tagged `Line3`) to the conic miter locus it stands for: a curve of length `L`
  between endpoints `c*` apart lies inside the ellipse with those foci and major axis `L`, whose semi-minor
  axis is `sqrt(L²−c*²)/2`. That axis only grows as the focal distance shrinks, so `c` is a proven LOWER
  bound on `c*`, the distance from the denoted corner to the denoted foot the stated `ds` along the sweep:
  the held chord from the corner to the held foot at the held `ds`, its square exact and its root rounded
  down, less `band.delta`, the corner's bound and `dsDelta`. The corner's bound reaches the points both
  neighbouring walks' records denote there (`boundarywalk.JunctionStartBound`), which adds an arc's radial
  residual at its natural `t = 1` end, where the record holds `End` off Start's radius. The held chord alone is not one,
  since the held foot and `ds` can sit farther apart than the denoted ones. A millimetre setback on a
  recorded section with exact feet subtracts nothing. `speedUpper` is `capcontour.MiterLocusSpeedUpper` (`internal/capcontour`), the same input
  `chordLocusLengthAllow` reads. Zero at a line-line miter, every reflex foot and every G1 join (modify §7's
  dead-zone rule; all three loci affine). Charged on BOTH patches sharing the ruling.
- `capRadiusRound = addRoundError(r, ∓d, capRadius)` — the held cap directrix radius against `r ∓ d`
  taken exactly from the walk's held radius `r`. `band.delta` charges the gap from `r` to the radius
  the record denotes (the walk's `RadiusBound`), since `capcontour.Displacement` reads every offset
  radius through `capcontour.OffsetCircleRadius`.
- `band.delta`, `levelDelta`, `deltaAxial` (`capBandLevel`), `deltaStore` (§3's mechanism over `prismLike`).

`faceBound(patch) = upRound(Σ terms)`; `bound = max`. `areaSlack` per patch: `proofbound.PerturbedTriangleAreaAllow`
over the `twist + sagitta + skewGap + locusGap` displacement plus `contourAllow + bandLevelAreaAllow` the
payload already publishes per patch; the trimmed walls and caps charge the prism's own terms.

### Occupied volume — `volSymDiff` for an admitted band

**Admission** (`capblend.OccupiedVolumeAdmission`, `internal/capblend/admit.go`) is decided from the record alone and
read by both consumers that must agree: `booloperand.RequireVolumeProof` before any mesh is
built, and `tessellateCapBlend` when it publishes. Every loop of the payload must be either

- a single closed `CircleSeg` walk (the whole turn), or
- a loop whose every walk is a natural-bounded (`TStart`, `TEnd` ∈ {0, 1}) `LineSeg` or `ArcSeg`, whose
  every `ArcSeg` satisfies `|End − Center|² == |Start − Center|²` exactly over the rationals (so the denoted
  arc passes through both recorded endpoints and the pinned junction vertex lies on it) or, on a cap-loop
  chamfer band, is read about its restated centre (§7.1), and whose every
  corner join is either (i) a line-line miter — both walks straight — or (ii) an exactly G1 join: the two
  walks' recorded junction points are the same rational point, and their exact tangent directions there have
  a zero cross product and a positive dot product, where a line's direction is `end − start` and an arc's is
  its walk sense times `rot90(P − Center)`.

A draft band (`capblend.Payload.Draft`) or a cap-loop chamfer band also admits (iii) a near-tangent join: a
`LineSeg` and an `ArcSeg`, or two
`ArcSeg`s, with one recorded junction point whose exact tangents run the same way, turn by `sin²θ ≤ 2⁻⁶⁴`, and move
by one amount, each moved circle clear of the corner (`capband.NearTangentCorner`), and each wall at it proven to
survive every level (`capband.NearTangentWallRefusal`). At two arcs B1's corner is the leaving arc's radial foot, so
the arriving arc's last cell is not a planar trapezoid; the charge covers that cell's twist beside the sliver. Its
proof adds the corner charge `capblend.NearTangentVolume` to `volSymDiff` below; `agent-docs/modify-design.md` Part D
§9.1.1 states and proves it. A chamfer band moves every walk by its cap's `dc`, so the one-amount condition
holds; its shell offset meets a convex sliver at the same corner and closes a reflex sliver with a connector arc,
and §9.1.1's "On a cap-loop chamfer band" shows the same charge covers both (`capblend.Payload.cornerClass`).
Neither a fillet band's view (`capblend.Payload.Fillet`) nor a loft-source band charges it here, and a loop
chamfered on both caps with two different setbacks takes the exact class. A route L chamfer band of a brep
body (`brepbandmesh`) composes its own terms without the corner or restatement charge, so its admission
(`capblend.ExactOccupiedVolumeAdmission`) keeps the exact class. A route L fillet band takes no admission
here: `brepfilletmesh` meshes it and charges its own near-tangent joins (`agent-docs/modify-design.md` Part L
§5.5).

A reflex corner (`cornerJoin.arc`) is admitted for a prism cap-loop chamfer band only (`capband.OccupiedVolumeAdmission`'s
`reflex` parameter, which `capblend` passes as `restate`, so it is true exactly when `Payload.cornerClass` returns the
chamfer class). A draft, fillet or loft-source band, a loop chamfered on both caps with two setbacks, and a route L
brep band (`ExactOccupiedVolumeAdmission`) keep the refusal. Every test is exact rational arithmetic over the record's
own floats; none is a tolerance, so the predicate classifies the denoted geometry and never admits on a
residual. A band it does not prove is export-only, and a boolean over it is the staging `ErrUnsupported`
routed through `meshbool.BooleanExpectedVolumeProof`, naming the loop and the corner.

**Why an admitted band reduces to two legs.** Write `B0` for the body the record denotes, `M` for the held
mesh, and `B1` for the IDEAL polyhedron with `M`'s triangle index set and exact vertex positions: every
side-ring vertex at the exact fraction `k/n` of its walk's exact window, on the exact curve, at the exact
level; every cap-ring vertex of a circular wall at the exact fraction `k/n` of the SIDE window on the exact
offset circle `Center + (R ∓ d)·e(θ_k)`; every line-line foot at the exact offset corner point; all at the
exact cap level. For an admitted loop the exact offset family is `P ⊖ s·d` with every circular wall keeping
its own window — a G1 join's offset foot is `P + s·d·n` on the shared normal, so its azimuth about either
centre never moves — and every corner-foot locus affine in `s` (a line-line miter is the intersection of two
offset lines; a G1 foot rides the shared normal). Each band cell of `B1` is a planar trapezoid, because the
matched side and cap chords are parallel, so its section at axial fraction `s` is the straight segment
between `(1−s)·side_k + s·cap_k = Center + (R ∓ s·d)·e(θ_k)` for a circular wall, and the exact offset line
between its exact feet for a straight one. `B1`'s band section is therefore the chord polygon of the true
section `P ⊖ s·d` at the SAME exact azimuths, with every vertex ON the true boundary; over the trimmed
range it is the chord polygon of `P` itself, and at each cap it is the cap triangulation of that polygon.
Both bodies are unions of their level sections, so

```text
volume(B1 △ B0) = ∫ area(S1(z) △ S0(z)) dz <= Mchord
Mchord = Σ_loops [ hTrimUpper(loop) · Σ_{circular walks} tessellation.WalkSegmentArea(w, n(w))
                 + Σ_{chamfered caps} dUpper · Σ_{circular walks} tessellation.WalkSegmentArea(w at radius max(R, R ∓ d), n(w)) ]
hTrimUpper = |zHi − zLo| + zLo.bound + zHi.bound        dUpper = d + dDelta
```

plus the reflex connector's term (below) when the loop has one.

because a chord polygon whose vertices lie on the curve differs from the region it chords by the union of
its circular segments — overlapping segments only make the union smaller than the sum — and a segment's
area `ρ²(Δθ − sin Δθ)/2` is monotone in `ρ`, so the larger of the two radii bounds every level of the band
over the one shared window.

**Reflex connector.** At a reflex corner of the material (a hole's convex corner, or an outer loop's notch) the
band's section at axial fraction `s` is the loop offset by `s·d`, and the offset closes the corner with an arc of
radius `s·d` about the junction `V`. The band patch there is the apex cone (`sideRadius == 0`), meshed as a fan from
the side-ring junction vertex (at `sideZ`) to the connector's cap stations (at `capZ`). Put `B1`'s vertices as
follows. The apex is the denoted `V`, already the side ring's `B1` vertex. The end stations `PA` and `PB` are the
exact wall feet `V + d·n̂_prev` and `V + d·n̂_cur`, already the adjacent walls' `B1` vertices. Interior station `k`
is the radial projection of the held station onto the denoted circle of radius `d` about `V`. A fan triangle
`(V, e_k, e_{k+1})` cut at fraction `s` is the segment from `V + s(e_k − V)` to `V + s(e_{k+1} − V)`, whose ends
both lie on the true arc of radius `s·d`. `B1`'s section is therefore still the chord polygon of the true section
with every vertex on the true boundary, so the two legs cover the fan:

- *Chord term.* Over the band height each fan chord omits one circular segment. A chord `c` on a circle of radius
  `ρ` omits at most `π³c³/(96ρ)`, because `Δ − sin Δ ≤ Δ³/6` and `Δ = 2·asin(c/2ρ) ≤ πc/(2ρ)`. Down the cone `c`
  and `ρ` both scale by `s`, so the area scales as `s²` and the cap level bounds every slice. The term is
  `Σ_chamfered caps bandHeight[cap] · Σ_cells 32·c_k³/(96·(d − dDelta))`, with `32 ≥ π³` and `c_k` the held chord
  plus both ends' motion (`tessellation.CapBlendConnectorVolume`).
- *Vertex motion.* The rows above. The gap from an interior station to its radial projection is at most its gap to
  any point of the circle, including the held circle station `capBound` measures, moved by at most the junction's
  own bound and the setback's rounding.

The trimmed wall meets the corner sharply at every level, so the trimmed term is unchanged, and §7.1's restatement
charge already counts a junction's normal cone in `Perim_L`, so restated arcs need nothing more.

*The order check.* The chord-polygon claim holds only when the projected stations run in order inside the arc's
window, so `tessellation.CapBlendCapMotion` gives a connector `+Inf` motion (a non-finite term, so the boolean
refuses `ErrUnsupported`, tess §12) unless this passes. About the held junction `V`, with `u_k = p_k − V` for the
stations `PA = p_0, …, p_m = PB` and `s = ±1` the turn sign of the recorded sweep `ArcTh1 − ArcTh0`, require
`s·cross(u_0, u_m) > 0`, and for every interior `k`, `s·cross(u_0, u_k) > 0` and `s·cross(u_k, u_m) > 0`, and for
every `k`, `s·cross(u_k, u_{k+1}) > 0`. The first says the window is under half a turn and the other three that
every station lies inside it and the stations are strictly increasing. Each test is evaluated exactly over the
held floats (`big.Rat`), and it passes only when `s·cross` exceeds the change the operands' own displacements can
make to it: for vectors `u`, `w` displaced by at most `e_u`, `e_w`, that is `e_u·|w| + e_w·|u| + e_u·e_w`. An
interior station's displacement is the junction's bound (the angle of `p_k` about the denoted `V'` is all the
projection reads), and `PA`, `PB` carry `band.delta` beside it, because they stand for the exact feet. A chamfer band's restatement shift (§7.1, `capband.OffRadiusLoop`) is added to every operand's displacement, since the restated section moves each station and the junction by at most that shift. The check
can only refuse: a held loop is admitted only with margin, never on a residual, so a degenerate window (a cusp,
a station crossing a neighbour) refuses and never composes. The held sample angles run monotonically by
construction (`circleStation`), so the check fails in practice only for a hand-built or degenerate record.

`volume(M △ B1)` is vertex motion: `M` and `B1` share a triangle index set and differ vertex by vertex, so
`proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(M, motionMax))` (internal/proofbound/bounds.go) bounds it, with `motionMax`
the largest per-vertex displacement:

| Vertex | Per-vertex motion (each term already proven; summed through `proofbound.AbsSumUpper`) |
|---|---|
| side ring, level `L` ∈ {`zLo`, `zHi`} | `proofbound.WalkEndBoundAllow(sideBound)` — `stationbound.ChordStationBound` at `k/n` for an interior station, the walk's own `startBound` at a junction — plus `prismrecord.Payload.PointRound` plus `L.bound` |
| cap ring, station `k` of a circular walk (`k = 0` is the foot verbatim) | `proofbound.WalkEndBoundAllow(stationbound.CapOffsetStationBound(seg, k, n, ∓d))` — `circularEndpointInterval`'s enclosure with the exact offset `∓d` (`capcontour.CapWallRadiusOffset`) added to the record's radius `R`, read at the exact fraction `k/n` of the SIDE window — plus `prismrecord.Payload.PointRound` plus `capBandLevel(capZ).bound` |
| cap ring, a straight walk's foot preceded by a circular walk | the same enclosure on the PRECEDING walk at `k = n`, plus the same two terms |
| cap ring, a line-line miter foot | `band.delta` (`capContourDelta`'s enclosure of the exact miter point) plus the same two terms |
| cap ring, a straight wall's first station when its start join is a reflex connector | `band.delta`: the station is the wall's own exact foot `PB`, not the previous arc's end |
| cap ring, connector end stations `PA`, `PB` | `band.delta` (`capcontour.Displacement`'s `Arc` arm encloses the exact feet `V + d·n̂`) plus the same two terms |
| cap ring, connector interior station `k` | `proofbound.WalkEndBoundAllow(capBound[k])` (held station against the held circle) plus `proofbound.WalkEndBoundAllow(sideBound[V])` (the junction's bound) plus `dcDelta`, plus the same two terms; `+Inf` unless the order check below passes |

The per-vertex figure is a `motion` array beside `store`: `store` keeps publishing `sourceBound` and
`areaSlack` exactly as above, because the two answer different questions — a cap station's Hausdorff gap
from the held offset circle, versus its displacement from the exact-fraction point the slice argument pairs
it with. Then

```text
volSymDiff = upRound(Mchord + proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(M, motionMax)) [+ Ncorner] [+ Noff])
symDiffOK  = true
```

where `Ncorner` is a draft or chamfer band's near-tangent corner charge (`agent-docs/modify-design.md` Part D §9.1.1),
and `Noff`, with the shift `motionMax` adds, is §7.1's restatement charge; each is present only when it is
positive, so every other band's sum is unchanged.

A non-finite term refuses (tess §12). The cap-blend path runs no facet-contact audit
(`payloadAuditsFacetContact`), so `boundaryOK` is true at every level and `meshbool.OperandSymDiff` reads the proof
at `VerifyAll`. The payload's own `Volume()` bound is a different composition (`capband.BandVolume`) and is not
read here.

**What is not admitted, and why.** A circular wall meeting a genuine (non-G1) miter has its cap window
trimmed inside its side window, its true foot locus is a conic, and the ruled cells' sections are no longer
chord polygons of the true offset sections: the sliver at the corner is covered by none of the terms above
and needs §9's per-cell certified integral. It keeps `symDiffOK == false`. A reflex corner of a route L brep band,
a draft band or a fillet band also stays refused: their mesh proofs charge no connector term.

The matching axial loft's zero-or-one-bore, both-outer-cap chamfer is a narrow
exception (`capblend.ProveLoftBandVolume`, called from `capBlendMesh` in `internal/tessdispatch/dispatch.go`; `agent-docs/loft-design.md` §18). Its band
denotes the loft's certified held cap polygon, not the fit-spline offset.
Every outer walk is straight, and any unchanged bore is one whole circle.
For this payload, the proof locates the reflex connector's cap-level vertex
indices and replaces their unbounded point motion with the full occupied
volume of two enclosing cap slabs. The ordinary chord and finite vertex
motion terms cover the rest of the polygon band. The source loft's global
`VolSymDiff` is added for comparison with its original spline flanks. This
payload publishes `symDiffOK` only when every term is finite, and the Boolean
precheck admits the same payload. The general cap-band admission above admits reflex corners
for the prism cap-loop chamfer only, and refuses them in other receivers.

### 7.0 Displaced loops

`capblend.TessellateDisplacedMesh` builds a `capblend.DisplacedPayload` (carrier-record-design §8.7) through the core `TessellateMesh` uses, with `D = max LoopDelta`. It reserves `D` from the chord budget before any chord. A side face of loop `l` adds `LoopDelta[l]` to its facet bound and each cap adds `D`. Per displaced loop the area slack takes `SectionDisplacementLength(LoopDelta[l], n)` times the wall height for its walls and `SectionDisplacementArea(LoopDelta[l], n, P')` for each cap, `P'` the chorded perimeter. Where the occupied-volume proof is built, the volume term adds `SectionDisplacementArea` times `|Z1 − Z0|` plus both level displacements. `booloperand.RequireVolumeProof` reads the base payload's admission.

### 7.1 Off-radius arc ends — restatement

An `ArcSeg` denotes Start's radius at End's angle, and the loop closes the junction from that point to the
next walk's start with a straight chord (`agent-docs/evaluator-design.md` §4). A sketch solve places tangent
points by rounding, so an ordinary sketched outline records corner arcs whose `End` sits a few ulps off the
circle `Start` states (the preflight refuses more than 1024 ulps). The exact-radius test above refuses every
such arc. A cap-loop chamfer band instead **restates** each one (`capband.OffRadiusShift`,
`internal/capband/restate.go`):

```text
C′ = C + μ·(End − Start),   μ = (|End − C|² − |Start − C|²) / (2·|End − Start|²)
```

`C′` is the point of the two ends' perpendicular bisector nearest the recorded centre `C`, a rational, and
both recorded ends lie on the circle about it exactly. Write `X` for the section the record denotes and `X′`
for the section with every off-radius arc read about its `C′`. `X′` has no junction step, so the admission
reads it: every corner of a restated arc is tested about `C′`, as exactly G1 or as a near-tangent join, every
wall survival gate reads `C′`, and a restated arc must be its walk's only segment (every arc is one today; only
exactly collinear lines coalesce). Let `B1′` be the ideal polyhedron of `X′` and `B0′` the chamfer of `X′`. Then

```text
volume(M △ B0) <= volume(M △ B1′) + volume(B1′ △ B0′) + volume(B0′ △ B0)
```

**The shift.** For `c = |C − C′| = |r_E² − r_S²| / (2·|End − Start|)`, the point `X` denotes at fraction `φ` of
its window and in-plane offset `τ`, `C + (r − ιτ)·e(θ_S + φ·W)`, and the point `X′` denotes there,
`C′ + (r′ − ιτ)·e(θ′_S + φ·W′)`, are at most

```text
Δ = 2c + (R_hi + T)·(π/2)·c / r′_lo
```

apart for `|τ| <= T`: `|r − r′| <= c` since both radii reach `Start`, and each end's angle about `C` and about `C′`
differs by at most `(π/2)·c/r′`, because the sine of the angle at `Start` between `Start − C` and `Start − C′` is
`|(Start − C) × (C − C′)| / (r·r′) <= c/r′` (at `End` likewise), so the angle at every fraction does too. The bound
refuses (`ok` false, the admission's restatement refusal) unless `2c < r′_lo` and `π·c/r′_lo` is below
`|End − Start|/R_hi`, the least window that chord subtends from either side, so both windows run the same way
round. `Δ` is zero for an arc whose ends lie on one circle.

**Three charges** (`capblend.OffRadiusMotion`, `capblend.OffRadiusVolume`), with `H` the body's height and
`Δ_L` a loop's largest shift:

- **Vertex motion.** Every station of `B1′` lies within `Δ` of the matching station of `B1`, which the held
  mesh's motion bound is measured against, so `motionMax` adds the largest `Δ`. `NearTangentVolume`'s `λ` adds
  twice the loop's `Δ_L` beside twice its motion.
- **Chord segments.** `Mchord` integrates the circular segments of `X`'s walks. Each chord segment of a restated
  arc of `X′` lies inside the matching segment of `X` grown by `Δ`, a convex set whose area grows by at most its
  perimeter times `Δ` plus `πΔ²` (Steiner). Its segments' perimeters sum to at most twice the arc's length at its
  largest radius, `4π·(R_hi + T)`, so each restated arc adds `H·(4π·(R_hi + T)·Δ + n·π·Δ²)`, with `n` the loop's
  side-ring sample count bounding its chord count.
- **The two denoted bodies.** For each level's in-plane offset `t`, let `Ω` and `Ω′` be the loop's region in `X`
  and `X′`, and `E` the offset that loop takes (the erosion of an outer loop's region, the dilation of a hole's).
  `Ω △ Ω′` lies in the set `K` the segments from each point of `X`'s arc to its `X′` point sweep, with the
  junction chord within `|End − P(1)| <= Δ` of `End`; every point of `K` is within `Δ_L` of both `∂Ω` and
  `∂Ω′`. A point `x` in `E_t(Ω)` but not `E_t(Ω′)` has a point `y ∈ K` within `t` of it, so its signed distance
  `s′(x)` to `∂Ω′` is at most `t + Δ_L`, and no point of `Ω′`'s complement within `t − Δ_L`, since that point would
  lie in `K` within `Δ_L` of `∂Ω` and so within `t` of `x`; the other way round is the same. So both differences
  lie in `{t − Δ_L <= s′ <= t + Δ_L}`, whose area is `∫ H¹({s′ = τ}) dτ` by the coarea formula, the signed
  distance having unit gradient almost everywhere. A level set `{s′ = τ}` lies in the union of each walk's
  parallel at `|τ|` over its own window and an arc of radius `|τ|` about each junction within its normal cone,
  under a half turn, so its length is at most
  `Perim_L = Σ_lines L + Σ_arcs 2π·(R + T + Δ_L) + Σ_junctions π·(T + Δ_L)` (`capband.OffRadiusLoop`). The
  other loops are the same in both sections, so the section difference lies in the loop's own, and each level
  adds at most `2·Δ_L·Perim_L`.

```text
Noff = H · Σ_loops [ 2·Δ_L·Perim_L + Σ_{restated arcs} (4π·(R_hi + T)·Δ + n·π·Δ²) ]
```

Lemma 3 assumes `X′` stays a simple loop at the same clearance from the other loops as `X`, so the build's own
loop gates read for both. Nothing here proves it: the window gate bounds `Δ` only below the chord, so a very short
restated arc beside another loop is the case it leaves open.

A loop with no off-radius arc adds nothing. A complete-loop fillet reads the same restatement, shift and
level-set perimeter for its strip (`agent-docs/modify-design.md` Part L §5.6). The 20 mm plate of the apitest below (two R4.5 corners and an R22
end arc, 8 mm thick, 0.5 mm chamfer) restates two arcs with `Δ ≈ 1.5·10⁻¹⁵` mm and `Perim ≈ 274` mm, so its
`Noff` is near `7·10⁻¹²` mm³.

### Refusals

| Condition | Result |
|---|---|
| any term above non-finite (`capcontour.MiterLocusSpeedUpper` answering `false`, `capblend.PatchWindowSkew` non-finite) | `ErrUnsupported` |
| a chamfered loop's contour ring and its original ring at the other cap cannot clear each other's sagitta tubes | `ErrDegenerate` via `tessellation.ExpectedError` (`tessellation.RequireLoopClearance`) |
| `n(w)` exceeds `freeform.MaxChordsPerWalk` | `ErrUnsupported` (`freeform.ErrTooManyChords`) |
| directed-edge, link, or zero-area audit fails | `ErrUnsupported` |
| a `chamferCap(...)` role or a `side(i,j)` role resolves to no face | `ErrDegenerate` |
| a band the occupied-volume admission above does not prove, in a boolean or in `Verify`'s read-only intersection | `ErrUnsupported` naming the loop and corner, through `booloperand.RequireVolumeProof` before the mesh and R0's `meshbool.OperandSymDiff` after it; `Verify` reads the pair `Suspect` (`DiagUnsupportedPairPayload`) |

### Tests (R6)

- Square plate chamfered on one cap loop (all `Plane` patches, unplaced): `Bound() == 0` when
  `band.delta == 0` and the levels are exact; watertight; `meshVolume` equals `body.Volume().Value` exactly.
- Disk chamfered on a cap loop (`wholeTurn` `Cone` patch): quads planar (assert the four corners' coplanarity
  in `big.Rat`), `Bound()` equals the larger of the two rings' `chordSagitta` plus the level terms, and dense
  sampling of the frustum finds no point past `Bound()`.
- Rounded rectangle chamfered on a cap loop (miter corners, windows differ): `Bound() > sagitta` by at least
  `capRadius·skew`; `requireWatertight`; every `chamferCap` face appears in `SourceFaces()`.
- Reflex (notched) profile: the apex fan has one interned vertex and one link cycle.
- Square plate chamfered on one cap loop (all `Plane`, unplaced, `volSymDiff == 0`): `Intersect` with a box
  straddling one straight side over the band reports a bounded volume containing the closed form
  `w · (∫ over the flat part + ∫ (sideZ + (edge − x) − zBox) dx over the band)`, and `Cut` reports the
  plate's volume minus it; `Union` reports its corresponding sum. Triangle intersections can introduce rational
  vertices that do not round exactly to float64, so the result is `Approximate` even when this operand's
  `volSymDiff` is zero.
- The drilled flange (plate, analytic `Cut` of a bore, 12 mm `Fillet` of the vertical edges, 1 mm cap-loop
  chamfer): `Cut` by a bolt cylinder clear of the band reports `V_flange − π r² h` within the published
  bound, where `V_flange = 16·(6528 − (4 − π)·144 − 324π) − (116 + 30π)`; the bound is below a stated
  ceiling and positive; the result is `Approximate`.
- A chamfered pin as the TOOL of a `Cut`: the plate's volume minus the cylinder it removes, within bound.
- A band with a genuine miter at a circular wall (a circular-segment section: one chord, one arc): every
  boolean refuses `ErrUnsupported`, `errors.As` finds no `*BooleanError`, and the message names the loop and
  corner; `Verify` reads an overlapping pair `Suspect` with `DiagUnsupportedPairPayload`.
- Reflex corners (`apitest/capblend_reflex_test.go`). `TestCapBlendReflexDBoreCut`: a circle R18.1 plate with a D
  bore (arc r4.1, flat at y = 3.6) and three r1.3 holes, 5 mm thick, every start-cap loop chamfered 0.5 mm. The
  chamfered volume equals its closed form, a box cut clear of every band and a second cut on that result each
  equal the closed form less the box within the result's own published bound (positive, under 0.6 mm³,
  `Approximate`), the result tessellates watertight at 0.01 mm, and `Cut` plus `Intersect` of a box across the D's
  corner band add back to the chamfered volume within the two bounds. `TestCapBlendReflexLBodyBoolean`: the same
  for the 3 mm chamfered L section across its line-line reflex corner. Unit tests:
  `internal/tessellation/capblend_connector_test.go` (finite connector motion, `+Inf` when connectors are off and
  for each way the order check fails, the reflex-preceded wall reading `band.delta`, and a connector chord term
  at least the true omitted segments), `internal/capband/reflex_test.go` (admission by the `reflex` flag, and
  `NearTangentCharges` skipping a flagged corner). Dropping the connector term or the interior-station motion
  does not move a published bound out of its apitest tolerance: the connector's share of the D plate's bound
  is about 0.005 mm³ per corner against 0.5 mm³ of chord shortfall, so no fixture here leaves its bound
  without it. The unit tests pin each piece directly.
- `Verify` on an overlapping pair of admitted cap-blend bodies publishes an `Interference` row.
- The tangent-arc plate (§7.1, `apitest/capblend_tangent_arcs_test.go`): both cap loops chamfered, then a box
  cut clear of both bands removes exactly the 48 mm³ it overlaps within the published bound. In
  `internal/capband/restate_test.go`: the chamfer class admits the plate's recorded loop while the exact and
  draft classes refuse its off-radius arc; a restated corner turning past the near-tangent class refuses by
  name, which reading corners about the recorded centres would admit; `OffRadiusShift` covers a 256-bit
  sampled reference and refuses an arc too short to restate.
- Each leg of `volSymDiff` is shown to fail: deleting `Mchord`'s band term, its trimmed term, the cap
  `motion` enclosure, the side `L.bound` term or `prismrecord.Payload.PointRound` makes a named fixture's result
  volume fall outside its published bound, recorded in the test file.

## 8. Recorded decisions this design changes

This design extends tess and the evaluator, and changes one recorded decision. `agent-docs/modify-design.md`
Part R §12 Table DX, row DX4, `capBlendPayload` cell, states that boolean admission follows the occupied-volume
proof of §7: it admits the bands §7 proves and keeps `ErrUnsupported` for the rest. Tess §2 makes boolean admission
conditional on `volSymDiff`, which is why export-only tessellation does not admit a band.

## 9. Open questions

- **R6 occupied volume for the bands §7 does not admit.** §7's slice-wise proof admits a band of line-line
  miters, exactly G1 joins, whole turns and a prism chamfer's reflex corners. A circular wall at a genuine miter needs a per-cell certified
  interval integral over the explicit homotopy held-facet → bilinear → ruled → cone, tess §11's `Icell`
  shape — the complete answer.

## 10. Tasks

Where each increment's code lives. Payload dispatch for all of them is `internal/tessdispatch/dispatch.go`.

### R0 — proof record

- `topo.Mesh` fields `FaceBound`, `VolSymDiff`, `SymDiffOK`: `internal/topo/mesh.go`. `topobuild.SetFaceBound`:
  `internal/topobuild/mesh.go`.
- Faceted restatement: `internal/facetedrecord/restate.go`.
- Prism and cup `deltaStore`, `faceBound`, `areaSlack` and `volSymDiff`: `internal/prismtess`, `internal/cuptess/cup.go`.
  The area terms are `tessellation.WalkWallSlack` and `tessellation.WalkSegmentArea` (`internal/tessellation/chord.go`).
- `stationbound.ChordStationBound`: `internal/stationbound/station.go`.
- Boolean readers: `facesOfMesh` (`internal/meshbool/tangency_gate.go`) and `meshbool.OperandSymDiff`
  (`internal/meshbool/operand_prep.go`).

### R1 — loft

- `loftmesh.MeshProof` (`internal/loftmesh/mass_proof.go`), composed in `loftrecord.Build`, and
  `TwistAreaAllow` (`internal/loftmesh/loft_chord_allow.go`, `loft_moments.go`).
- `loftrecord.Tessellate` (`internal/loftrecord/tessellate.go`) and `loftmesh.RestateLoft`
  (`internal/loftmesh/loft.go`).
- Status cells of loft §9 Table D rows D1/D2 state current behaviour.

### R2 — free-form prism

- `freeform.ChainStations`, `freeform.PairStations` and `freeform.SagittaStationWalk`:
  `internal/freeform/spline_stations.go`.
- The free-form arm of `tessellation.ChordLoop`: `internal/tessellation/loop_chord.go`.
- Spline Table C rows read current behaviour.

### R3 — revolve line generators

- Walk resolution: `revolveaxis.ResolveLoopWithFreeform`. Mesh assembly: `revolvetess.TessellateRevolve`
  (`internal/revolvetess/revolve.go`). Sampling and the axis gate: `internal/revolvesampling`. Extents and budgets:
  `internal/revolveproof`.
- Angular sequence, `deltaC`, `deltaR`, `RevolveBudget`, `RevolveContactAudit`, `Ecell`:
  `internal/revolvemesh` (`revolve_proof.go`, `revolve_construction.go`, `revolve_contact_audit.go`,
  `revolve_cell_area.go`).
- The pair audit follows `internal/loftmesh/loft_audit.go`.

### R4 — revolve circular generators

- `tessellation.ChordCount` takes `nMin`; `tessellation.RequireWalkClearance`; refinement loop and ring collapse
  detection in `revolvetess.TessellateRevolve`; certified-subdivision `Ecell` in `internal/revolvemesh`.

### R5 — revolve booleans

- `revolveproof.SymDiff` (`internal/revolveproof/volume.go`) composes `Mmeridian`, `Icell`, `Mconstruct` and `Mround`.
- `revolvetess.ResolveRevolve` gives `booloperand.PairChord` the count-independent `deltaC + deltaR`.
- Every boolean fixture sweeps a quarter or half turn: a full turn's angular count makes its facet-pair audit the
  most expensive step in the suite, and export tests already cover that geometry.

### R6 — cap-loop chamfer

- `capblend.TessellateMesh` (`internal/capblend/mesh_build.go`), `mesh_loop.go`, `mesh_band.go`, and
  `capBlendMesh` in `internal/tessdispatch/dispatch.go`. `capblend.Payload.BandDelta` holds each band's `delta`.
- Admission: `capblend.OccupiedVolumeAdmission` (`internal/capblend/admit.go`).
