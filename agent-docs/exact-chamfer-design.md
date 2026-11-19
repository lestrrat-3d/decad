# Exact Chamfer Design

Exact setback points for `Chamfer` of a straight edge between two retained planes of a mesh Boolean result
(retained-edge-blend R1): root cause (§1), required behavior (§2), approach (§3), construction (§4), bounds and
soundness (§5), performance (§6), tests (§7), risks (§8), open questions (§9), where it lives (§10).

Amends `agent-docs/modify-design.md` Part E §5.0 step 1, §5.2 and §6.3 (Claim K). Companion to
`agent-docs/coplanar-contact-design.md` (exact faces are what its §3.1 admits) and
`agent-docs/faceted-vertex-bounds-design.md` §2 (the corner model the bounds use). Adds no public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| Where does the 0.0201 bound come from? | §1 |
| What must hold on the exact path? | §2 |
| Why exact feet and not a rotated face or a snapped point? | §3 |
| How is a foot computed and bounded? | §4 |
| Why is the result sound, exact or not? | §5 |
| Does the dovetail pass? | §2, §9 |
| Where does each part live? | §10 |

## Assumptions

- "Exact face" is coplanar-contact §3.1's reading: every vertex of the face has `VertexBound` 0, so its `FaceBound`
  is 0. No new exactness notion.
- The true solid a chamfer denotes is the ideal one of retained-edge-blend §4.1: setback lines at in-plane distance
  `tA`, `tB` from the true edge on the true planes, joined by a plane. Feet are not redefined to fit floats.
- The true plane of a retained planar face is the plane through any three non-collinear vertices of it whose
  `VertexBound` is 0 (faceted-vertex-bounds §2: such a vertex IS a true point of that face). The face's tag is not
  used for exact geometry, because a tag frame's normal is a float `u×v` and need not contain those corners.
- The chamfer face's published `Plane` tag keeps today's float construction (`lineTag`). No bound of this design
  reads it. §9 Q3 asks whether to charge its rounding.
- Clearance gates keep their margins (`w.tau + slack`). Only vertex bounds and volume terms change.
- Fillets, R2–R7, chains, mitre runs and corner routes keep today's behavior.
- Tests use minimal prism fixtures. The consumer dovetail runs outside this repository (§7.4).

## 1. Problem and root cause

The gallery dovetail chamfers the two lip edges of a socket made by `Cut` (0.5 mm, edges along the slide axis, the
flank tilted 10° from square). The `τ` path (§4.1's fallback) gives the chamfered socket a volume bound of
`0.020087`, against `2.0e-12` for the socket `Cut` alone.

The lip edges take `retainedblend.Chamfer` → `rewriteLineEdge` (R1). The bound has one cause:

1. **Feet bounded by the fillet's grid tolerance.** `newWork` sets `w.tau = max(2⁻¹⁰·size, 64·ulp·scale)`
   (`internal/retainedblend/rewrite.go`), the chord tolerance of a fillet's cylinder rows. `rewriteLineEdge` gives
   every row vertex the bound `w.tau + slack` (`internal/retainedblend/line_rewrite.go`, `w.vertex(setbackPoint(...),
   w.tau+slack)`). A chamfer has two straight rows and no chord, yet each of its four feet gets
   `2⁻¹¹ + 2.27e-13 = 4.8828125e-4` mm. Measured: every new vertex reads `±0.0004882812502273737`.
2. **Claim K multiplies it by the strip.** `measureChange` charges `ε·(area(strip) + area(blend))` with `ε` the
   strip's largest vertex bound: `4.8828e-4 × (10.2846 + 10.2846) = 0.010044` per edge, `0.020087` for two. That is
   the whole volume bound.
3. **Face bounds follow.** The eight faces holding a foot (two lip tops, two flanks, two chamfer faces, two end faces)
   read area bounds `0.016–0.036` (`FaceAreaGeom`: `δ_f × perimeter`; chamfer face
   `4.8828e-4 × 33.2856 = 0.016254`), and their `FaceBound` is `4.88e-4`, so coplanar-contact §3.1 refuses every pair
   that touches them.
4. **Feet are computed in floats.** `setbackPoint(e₀ + wA·tA, d̂, g)` normalizes normals and divides by `d̂·n_G`. A lip
   top foot that is exactly `x = −5.5` comes out `−5.500000000000001`. retained-edge-blend §4 and §5.2 already require
   "exact rational arithmetic … rounded once" and "a vertex position bound is the rounding of that computation"; the
   code does neither for R1.

The receiver's corners are exact enough for an exact construction: `TestRetainedLineChamferExact` asserts every vertex
of the `Cut` socket has bound 0 before it chamfers. §4.1's carrier rule therefore finds zero-bound carriers.

Two further facts bound what any Chamfer change can reach:

- **Tilted feet are irrational.** A foot on a tilted face is `V + tA·u/|u|` with `|u|²` a dyadic rational that is not a
  square, so it has no float. Float points on the dovetail flank's line through `(−5, 0)` and `(−6.05796188425079, 6)`
  are spaced by the whole edge: none lies between `z = 0` and `z = 6`. The flank, the chamfer face and the end faces
  that hold a tilted foot keep a rounding-level bound (about `2e-16` mm) under any float representation.
- **The tail half fails by a different route.** The tail is an exact prism chamfered by the section rewrite
  (`offset2d.setbackFoot`), whose record denotes its float feet (modify §5 convention, `SectionDelta` 0). The trimmed
  flank therefore runs through the rounded foot, which lies strictly outside the original flank line (exact
  orientation test). The chamfered tail against an unchamfered socket reads `Suspect`, "proven to overlap"; the
  unchamfered pair reads `Sound`.

**Out of scope:** the prism section-rewrite chamfer (tail, §9 Q2); contact classification of faces with a
rounding-level bound (§9 Q1); fillets; R2–R7, chain, mitre and corner routes (§9 Q4); the chamfer face's tag (§9 Q3);
the proximity gate.

## 2. Correct behavior and success criteria

- **E1.** On the exact path (§4.1: zero-bound carriers found, no oblique end), an R1 chamfer foot whose exact point is
  a float is held at that float with `VertexBound` 0.
- **E2.** Every other foot on the exact path is held at a float within rounding of its exact point, with `VertexBound`
  a proven upper bound on the distance (rounding level: `√3` times half an ulp of the coordinates plus the `2⁻²⁰⁰`
  enclosure, below `2e-14` mm at coordinates under 100 mm).
- **E3.** A face rebuilt by an R1 chamfer whose vertices all have bound 0 has `FaceBound` 0, and the chamfer adds
  nothing to `VolSymDiff` when every foot is exact.
- **E4.** The dovetail socket's lip chamfer reads a volume bound below `1e-11` (the fallback reads `0.020087`), and its lip top
  feet (`x = ±5.5`) carry bound 0.
- **E5.** An R1 chamfer of a square lip on a mesh `Cut` socket, paired with an exact flush prism, reads `Sound`,
  `Intersect` returns `BooleanEmpty`, and `Union` volume equals the sum of the halves. No test pins E5 yet (§7.3).
- **E6.** No foot claims less than its proven distance from the ideal foot. An edge with no zero-bound carrier, or
  with an oblique end, keeps today's `w.tau + slack` path.
- **Not reached by this design:** the dovetail pair `Sound`. Its flank feet are irrational (§1), and its tail flank is
  rotated by the prism route. It needs §9 Q1 and Q2.

## 3. Approach

Compute each R1 chamfer foot exactly from the receiver's zero-bound corners, as the intersection of the setback line
with the end face's true plane, with one square root enclosed by `proofbound.SqrtFixed`. Hold the nearest float and
its proven residual (`proofbound.BoxMidStation`). A float foot gets bound 0. The trimmed faces keep their planes;
nothing rotates. Drop `τ` from chamfer feet only. On the R1 chamfer's four rebuilt faces only, let the slab term read
the corners' bounds when that is smaller than the tag distance (faceted-vertex-bounds §2), so faces whose corners are
all exact charge 0.

**Alternatives considered:**

- **Drop `τ`, keep the float computation with `64·ulp·scale`.** Rejected: `setbackPoint` divides by `d̂·n_G` down to
  `10⁻³` and normalizes twice, so `64·ulp` of scale is not a proven bound once `τ` stops hiding it; it never reaches 0
  (probe: `9.5e-12`, still `Suspect`).
- **Snap feet to a float grid and claim zero.** Unsound: a snapped tilted foot is off its plane (probe: plane slab
  charge `6.6e-8`, pair `undecided`).
- **Rotate the trimmed face through the float foot** (the prism convention). Makes an exact polyhedron but moves a face
  off its plane; the chamfered tail shows the result: a held overlap against a flush partner.
- **Chamfer by a polyhedral cutter `Cut`.** The cutter's corners round the same way, and the `Cut` adds rim bounds of
  its own, so the feet get no more exact.

## 4. Construction

### 4.1 Carriers

For edge `e` from `V₀` to `V₁` (held vertices `l.ends[i].v`), faces `A`, `B`, end faces `G₀`, `G₁` (`fG[i]` in
`rewriteLineEdge`), choose the carriers after `locate` and before the setback points:

- `cA_i` = the first vertex of `runA.chain[i]`, walking out from `V_i`, with `w.beta == 0` that is also a vertex of a
  loop of `fG[i]` (`indexIn` over `fG[i].loops`). `cB_i` likewise from `runB.chain[i]`. Every chain vertex is a vertex
  of `A`'s (or `B`'s) loop by construction (`walk` over that loop), so `cA_i` is a held vertex of both `A` and `G_i`.
- The exact path runs when the spec is a chamfer (`!spec.fillet`), `V₀` and `V₁` have `w.beta == 0`, all four of
  `cA₀, cA₁, cB₀, cB₁` exist, and neither end is oblique (`l.ends[i].tilt == 0`), no plane of `A`, `B`, `G₀`, `G₁` states a displacement
  (`AxialDelta` is 0; §9 Q5), and `V_i` is a vertex of a loop of `G_i`. Otherwise the feet take today's
  path unchanged (E6). An oblique end's tag states a tilt, and this design does not derive its plane.

All arithmetic is in `big.Rat` on the held floats. With `E = V₁ − V₀`:

- `perp(X) = (E·E)·X − (X·E)·E`, the part of `X` square to the edge, scaled by `E·E` so no division enters. Its
  components are dyadic rationals.
- `uA = ±2^m·perp(cA₀ − V₀)`, with `m` the integer that makes every component an integer and at least one odd. The
  sign makes `uA · wA > 0`, `wA` being `rewriteLineEdge`'s float way into `A` read as a rational. `wA` is within
  rounding of the true inward unit direction, so the true `|cos|` between them is 1; the exact path falls back unless
  `(uA·wA)² ≥ ¾·(uA·uA)·(wA·wA)` holds exactly (a falsify-only check: a disagreement refuses, agreement admits nothing
  the carriers did not already prove). `uB` likewise from `cB₀` and `wB`. A zero `uA` or `uB` falls back.
- `N_i = (cA_i − V_i) × (cB_i − V_i)`, the normal of `G_i`. A zero `N_i` or `N_i·E == 0` falls back.

### 4.2 Feet

The setback line on `A` is `V₀ + kA·uA + λ·E` with `kA = tA/|uA| = tA·√qA/qA`, `qA = uA·uA`. Its meeting with `G_i`:

```text
P_A,i = V_i + kA · wA_i,      wA_i = uA − ((N_i·uA)/(N_i·E))·E
```

`V_i` lies on `G_i`, so the `E` coefficient that reaches `G_i` from `V₀` is exactly the one reaching `V_i`. `P_B,i`
likewise with `tB`, `uB`, `qB`. Coordinates: `[P]_c = [V_i]_c + [wA_i]_c·(tA/qA)·√qA`.

`qA` is an integer, so `√qA` is rational exactly when `qA` is a perfect square (`big.Int.Sqrt`, then compare the
square). A perfect square gives the point interval `[√qA, √qA]`; otherwise `proofbound.SqrtFixed(qA)` gives a
`2⁻²⁰⁰`-wide enclosure (`SqrtFixed` alone also returns a point for a perfect square; the explicit test is optional).
Each coordinate's interval is `IntervalAdd(PointInterval(V_i,c), IntervalScale(sqrtIv, wA_i,c·tA/qA))`.
`proofbound.BoxMidStation` of the three intervals returns the float nearest each interval's midpoint and the bound
`Radius3D(max over coordinates of IntervalFloatError)`, which reads both ends of every interval and so covers every
point of the box, the held float possibly lying just outside a box narrower than an ulp. It is 0 exactly when every
interval is a single float.

### 4.3 Wiring

In `rewriteLineEdge`, after the chain-direction check and before the setback loop, call `w.chamferCarriers` and, when
it succeeds, `exactChamferFeet`. On the exact path, row 0 at end `i` takes `P_A,i` and row 1 takes
`P_B,i` with their bounds, in place of `setbackPoint(p, l.d, end.g)` and `w.tau+slack`. Everything that reads
`w.verts` afterwards (gates, loops, `lineAreas`, strip, `measureChange`) reads the exact-held feet. `tA`, `tB` are
`spec.planeSetback()` and `spec.wallSetback()` after `spec.assign`, as today; `blend` already refuses a nonzero
conversion delta, so both are the exact requested distances.

### 4.4 Plane slab term

`planeFacetVolume` charges each new facet `t` of a re-triangulated plane face `(h_t + δ) · area(t)`. On the four faces
`rewriteLineEdge` re-triangulates for a chamfer on the exact path, charge `min(h_t + δ, βmax_t) · area(t)` instead,
`βmax_t` the largest `w.beta` of `t`'s corners. Every other call keeps `(h_t + δ)`.

- Both bound the facet's distance from the true plane: `h_t + δ` through the tag (Claim K), `βmax_t` through the
  corner model (each corner lies within its bound of a true point of the face's plane, so the facet lies within the
  largest of them).
- The corner bounds read must be final. `rewriteLineEdge` writes no `w.beta` entry after its `retriangulate` calls
  (no assignment to `w.beta` exists in `line_rewrite.go`; `build` and `compact` copy it), so the bounds the slab term
  reads are the ones the result publishes. Other routes raise bounds after `retriangulate` (`rewrite.go`,
  `arc_rewrite.go`, `chain_rewrite.go`, `cyl_rewrite.go`), so they stay on `h_t + δ`.
- Mechanism: a `cornerSlab bool` field on `work`, set by `rewriteLineEdge` on the exact path before its
  `retriangulate` loop. `chargePlaneFace` passes `w.beta` when it is set and `nil` otherwise; `planeFacetVolume`
  reads `βmax_t` only from a non-nil `beta`.

### 4.5 Files touched

| File | Change |
|---|---|
| `internal/retainedblend/line_exact.go` | `chamferFeetInput`, `lineFoot`, `exactChamferFeet`, `chamferCarriers` |
| `internal/retainedblend/line_rewrite.go` | `rewriteLineEdge` calls `exactChamferFeet` for chamfers |
| `internal/retainedblend/plane_charge.go` | `planeFacetVolume` takes optional corner bounds; `chargePlaneFace` passes them |
| `internal/retainedblend/rewrite.go` | `work.cornerSlab` field |
| `agent-docs/modify-design.md` Part E | §5.0 step 1, §5.2, §6.3 point to this doc |

New unexported symbols (no exported ones):

```go
// lineFoot is one held setback point of an R1 chamfer and its proven distance from the exact point.
type lineFoot struct {
	p     r3.Vec
	bound float64
}

// chamferFeetInput is the plain data exactChamferFeet reads; every point is a held vertex with bound 0, chosen
// by §4.1 (onA[i], onB[i] are vertices of both their side face and G_i).
type chamferFeetInput struct {
	v        [2]r3.Vec // edge ends V₀, V₁
	onA, onB [2]r3.Vec // cA_i, cB_i
	intoA    r3.Vec    // wA: float sense of the way into A
	intoB    r3.Vec    // wB
	tA, tB   float64   // setbacks on A and B, exact millimetres
}

// exactChamferFeet returns feet[i][0] = P_A,i and feet[i][1] = P_B,i (§4.2). ok is false when a carrier is
// degenerate or the sense check fails (§4.1); the caller then keeps today's feet.
func exactChamferFeet(in chamferFeetInput) (feet [2][2]lineFoot, ok bool)

// chamferCarriers picks cA_i and cB_i (§4.1) from the runs and end-face loops, or ok false.
func (w *work) chamferCarriers(l line, runA, runB edgeRun, fG [2]faceLoops) (onA, onB [2]int, ok bool)
```

`planeFacetVolume(verts []r3.Vec, tris [][3]int, pl plane, delta float64)` gains a final `beta []float64` parameter:
nil keeps today's charge; non-nil (one entry per vertex of `verts`) enables §4.4's minimum. `chargePlaneFace` is the
only caller.

## 5. Bounds and soundness

**Claim X.** After an R1 chamfer on the exact path, every new foot `f` with held float `f̂` satisfies
`|f̂ − f*| ≤ β(f)`, `f*` the ideal foot of the true solid (§4.1 of retained-edge-blend), and `β(f) = 0` iff `f*` is a
float.

*Proof.* The six carrier vertices have bound 0, so each is the true point it stands for (faceted-vertex-bounds §2),
and a true point of every face whose held loop holds it. `V₀`, `V₁` are true corners on the true line `A ∩ B`.
`cA₀` is a vertex of `A`'s loop (it comes from `runA.chain`, walked over that loop) off the edge line (the gate "a
face leaves an edge end along the edge's own line" refuses otherwise, and §4.1 refuses a zero `uA`). The true face
`A` is planar (`retainedPlane`: `Plane` tag, `TagDeparture` 0), so its plane is the one through `V₀, V₁, cA₀`, and
`uA` is an exact direction in it square to the edge. Its sense is the side of the edge `A` lies on; the float `wA`
names that side, and §4.1's exact check refuses when the two disagree by more than 30°, which rounding cannot cause.
`G_i` is planar (`retainedPlane` holds for a non-oblique end; §4.1 excludes oblique ends), and `V_i`, `cA_i`, `cB_i`
are vertices of `G_i`'s loop (§4.1 checks `cA_i`, `cB_i` against `fG[i]`), so `N_i` is its exact normal. `P_A,i` is
then the exact meeting of the true setback line with the true `G_i`. The only inexact step is `√qA`, enclosed by
`SqrtFixed`; interval arithmetic carries it to each coordinate; `BoxMidStation`'s bound covers every point of the box,
which contains `f*`. The box is a single float point exactly when `f*` is a float. ∎

**Claim K (amended).** Claim K holds with `ε` the largest strip-vertex bound, now the feet's `β`, and, on the four
faces of an exact-path R1 chamfer, with the slab term `Σ_t min(h_t + δ_t, βmax_t)·area(t)` (§4.4), `βmax_t` read
from bounds that are final when read. Inside the region `K` the true blend face is the plane through the
two true setback lines; each strip facet's corners lie within `ε` of true points on it, so the facet lies within `ε`
of it (faceted-vertex-bounds §2). The gates of §6.2 are unchanged, so `K` and everything outside it are as before.

**Consequences.**

- All feet exact → `ε = 0`; every rebuilt face whose old corners were exact has `βmax = 0` and charges 0; the chamfer
  adds nothing to `VolSymDiff`; those faces have `FaceBound` 0 (E3).
- A tilted foot → `β` about one ulp of its coordinates; the faces holding it read `FaceBound` of that size, honestly
  (E2). Coplanar-contact §3.1 still refuses their contacts; nothing is admitted on a positive bound.
- Fallback (no zero-bound carrier, an oblique end, or a failed check) → today's `w.tau + slack` and slab term, today's
  proof.
- Area: `FaceAreaGeom` charges each face `δ_f × perimeter` from its own vertex bounds, which covers the in-plane move
  of a foot by `β` (retained-edge-blend §6.3, Area); `AreaSlack` keeps its `|Δheld − Δtrue|` and rounding terms.
- No check here admits anything: the exact path is construction from upstream zero-bound claims (CLAUDE.md
  "falsify-only"), and every gate keeps its margin.

## 6. Performance

- Added work per R1 chamfer edge: four feet, each a fixed number of `big.Rat` operations plus one `SqrtFixed`, and one
  exact orientation per new facet already paid by `planeFacetVolume`. The mesh rebuild (`facetedrecord.Build`)
  dominates.
- `BenchmarkRetainedLineChamfer` (`apitest/retained_blend_fillet_bench_test.go`) chamfers both lips of fixture S in one
  call.
- Smaller bounds can make later Booleans cheaper: the proximity pre-pass skips exact face pairs without a gather.

## 7. Test plan

Every test asserts computed geometry: positions, vertex bounds, volumes, statuses. Fixture S ("square-lip socket"): a
block `x ∈ [−10, 10]`, `y ∈ [0, 16]`, `z ∈ [0, 10]` sketched on XY and extruded along +Z, `Cut` by a prism sketched on
XZ (frame `U = +X`, `V = +Z`) through `(−5, −6), (4, −6), (6, 6), (−5, 6)` extruded `y ∈ [−1, 17]`. Its walls are
`x = −5` (square) and the slope-1/6 line through `(5, 0)` and `(6, 6)`; the different extrusion axes keep the `Cut` on
the mesh path. Tenon T: the prism through `(−5, 0), (5, 0), (6, 6), (−5, 6)` on the same plane, `y ∈ [0, 16]`.

### 7.1 Unit (`internal/retainedblend/line_exact_internal_test.go`)

- U1 square: `V = (−5,0,0), (−5,16,0)`, `onA = (−10,0,0), (−10,16,0)`, `onB = (−5,0,6), (−5,16,6)`, `t = 0.5`: feet
  `(−5.5, y, 0)` and `(−5, y, 0.5)` exactly, every bound 0.
- U2 Pythagorean: `V = (0,0,0), (0,0,16)`, `onA = (6,8,0), (6,8,16)`, `onB = (−8,6,0), (−8,6,16)`, `t = 1.25`:
  feet `(0.75, 1, z)` and `(−1, 0.75, z)` exactly, bound 0.
- U3 tilted: `V = (5,0,0), (5,16,0)`, `onA = (10,0,0), (10,16,0)`, `onB = (6,0,6), (6,16,6)`, `t = 0.5`: the foot on
  `B` has bound in `(0, 1e-15]`; the exact foot recomputed independently with `big.Float` at 512 bits lies within the
  bound of the held foot. The foot on `A`, `(5.5, y, 0)`, has bound 0.
- U4 oblique end plane: U1 with `onA[0] = (−8, 1, 0)`, `onB[0] = (−5, −2, 6)`, so `G₀` is `(x + 5) + 3y + z = 0`:
  `P_A,0 = (−5.5, 1/6, 0)` and `P_B,0 = (−5, −1/6, 0.5)`, not floats; `|f̂ − f*|` computed in `big.Rat` is `≤ bound`,
  and `bound > 0`. End 1 stays exact.
- U5 degenerate: `onA[0]` on the edge line → `ok == false`; `intoA` square to `uA` → `ok == false`.

### 7.2 Root package (`retained_line_chamfer_exact_test.go`, package `decad`; reads `mesh.n.VertexBound`)

- R1 square lip of S, `d = 0.5`: first assert every vertex of the `Cut` has bound 0; then every vertex of the result
  has bound 0, `Volume.Bound` equals the `Cut`'s, and the volume change is exactly `0.125 × 16 = 2`.
- R2 asymmetric square lips: S with both walls square (prism through `(−5, −6), (5, −6), (5, 6), (−5, 6)`), both lips
  chamfered with `d = 0.5`, other `0.25`, the block's start cap as reference (a face query on the lip bottoms alone
  matches the pocket ceiling too, which the option refuses): as R1, change `2 × 0.0625 × 16 = 2`.
- R3 tilted lip of S, `d = 0.5`: the two feet on the lip top, `(5.5, y, 0)`, have bound 0; the two feet on the
  slope-1/6 wall have bound in `(0, 1e-14]`; `Volume.Bound − Cut bound ≤ 1e-11`; each strip facet lies within its own
  bound of the exact chamfer plane, measured with U3's 512-bit `big.Float` feet (the plane is irrational).
- R4 variation: S with the slope-1/4 wall (prism through `(−5, −6), (3.5, −6), (6.5, 6), (−5, 6)`) and `d = 1.0` on
  the square lip: exact as R1 (module boundary: independent cases).
- R5 tilted exact planes: a block `x ∈ [−20, 20]`, `y ∈ [−10, 25]`, `z ∈ [0, 10]` sketched on XZ and
  extruded along +Y, `Cut` by the square prism through `(0, 0), (6, 8), (−2, 14), (−8, 6)` sketched on XY and
  extruded `z ∈ [−1, 11]`. Chamfer the hole's corner edge at `(0, 0)` (concave, a fill) with `d = 1.25`: feet
  `(0.75, 1, z)` and `(−1, 0.75, z)` with bound 0, volume change exactly `+0.5 × 1.25² × 10 = 7.8125`, and
  `Volume.Bound` at most `1e-13` above the `Cut`'s (the `Cut` itself leaves rational wall corners that are not floats,
  so the walls' facets charge their tag distance, not `0`; the corner minimum is covered by the unit cases). The walls' float tags need not contain their
  corners.

### 7.3 API contact (not implemented)

`apitest/retained_chamfer_contact_test.go` does not exist. It would pin:

- C1: S with the square lip chamfered `0.5`, and T, in one document: `Verify` → `Sound`, no diagnostics;
  `Intersect` → `BooleanError` with `Code == BooleanEmpty`; `Union` volume equals `vol(S') + vol(T)` with bound 0.
- C2: C1 with `d = 0.25`, S on R4's slope-1/4 wall, and T through `(−5, 0), (5, 0), (6.5, 6), (−5, 6)`.
- C3 known limit: S with the tilted lip chamfered `0.5`, and T: `Verify` → `Suspect` with
  `DiagUnsupportedPairContact`, flipping to `Sound` when §9 Q1 lands.

### 7.4 Consumer acceptance (gallery, not committed here)

- Stage repro: the "cut chamfered" bound is below `1e-11`, reachable with the exact path alone.
- Pair: tail and chamfered socket → `Verify` `Sound`, `Intersect` `BooleanEmpty`, `Union` volume equals the sum.
  Expected `Suspect` until §9 Q1 and Q2 land.

### 7.5 Tests that expect zero bound growth

`requireVolumeChange` and `requireAreaChange` (`apitest/faceted_retained_blend_test.go`) require
`growth >= 0`, because an R1 chamfer with exact feet grows the bound by 0; with growth 0, `InDelta` demands the exact
change. `TestFacetedRetainedBlendStraight/concave_edges_at_a_wall's_foot` changes volume by exactly `−10` with growth 0.

## 8. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| The dovetail stays `Suspect` after this lands | the consumer goal is not met | §2 and §9 state it; C3 pins the limit |
| A test pins a `τ`-sized chamfer bound | a test fails on exact feet | §7.5 |
| A bound raised after `retriangulate` read by §4.4 | slab term too small | §4.4 limits it to R1, which writes no bound after |
| No zero-bound carrier on a receiver | falls back to `τ` | §4.1 fallback; §7.4 reports the carriers' bounds |

## 9. Open questions

**Q1. Contact of faces that share an exact plane but carry a rounding-level bound** (needed for the dovetail pair).

- A. New design in coplanar-contact: admit a coplanar pair whose faces' true planes are proven equal from zero-bound
  corners and whose only positive bounds are irrational feet, classifying on the true planes. Gains: the dovetail and
  every tilted chamfer flush against a partner read `Sound`. Costs: soundness-critical boolean work; the mesh pass
  must classify facets within `β` of the plane, not in it. Size unknown; a design pass settles it.
- B. Exact algebraic coordinates (`ℚ(√q)`) for held vertices. Gains: every Fusion-style chamfer exact. Costs: touches
  every faceted reader; months.
- C. Accept `Suspect` for tilted chamfers in exact contact; the gallery keeps its own fit check
  (`features/dovetail/dovetail_test.go` notes Verify refuses exact halves). Gains: no work. Costs: the oracle cannot
  answer the dovetail's fit question.
- Recommend A, because it is the only option that answers the question at bounded scope.

**Q2. The prism section-rewrite chamfer rotates its trimmed side through the float foot** (the tail half).

- A. Charge the foot's residual from the original side line as `SectionDelta`. Gains: the tail flank stays on its
  plane within `β`; no fake overlap. Costs: every prism chamfer with a non-float foot becomes a displaced receiver,
  which `RequireExactSection` routes refuse for later modify ops and analytic Booleans; how many tests flip is
  unknown until tried.
- B. Record carriers for the chamfered prism (carrier-record §7.3's convention: carrier to the projected point, junction
  chord to the foot, `r_f` charged). Gains: carriers exact and readable by later analytic ops. Costs: a carrier-record
  producer for plain prism chamfers; large.
- C. Keep the convention. Gains: no regressions. Costs: chamfered prisms read "proven overlap" against flush exact
  partners.
- Recommend deciding Q1 first; if Q1 = A, take B, because A turns every tilted prism chamfer into a displaced
  receiver.

**Q3. The chamfer face's `Plane` tag** is a float frame that need not contain the exact feet.

- A. Keep it (this design). Gains: no change downstream. Costs: retained-edge-blend §5.1's Claim R holds only to
  rounding, as today.
- B. Publish `AxialDelta` = the proven largest distance of the four exact feet from the tag plane. Gains: Claim R
  exact. Costs: every R1 chamfer face becomes a displaced plane; readers that refuse displaced planes refuse it.
- Recommend A until a reader needs B.

**Q5. Faces that state a plane displacement** (`AxialDelta` above 0) keep today's feet and the τ bound.

- A. Keep the restriction. Gains: the existing pin `TestChamferChargesPlaneAxialDelta` (a stated
  displacement reaches the vertex bounds) holds. Costs: such chamfers keep the `τ`-sized volume bound.
- B. Drop it. The carriers are true points, so the exact feet do not read the tags. Gains: exact feet there too.
  Costs: that test's pin changes.
- Recommend A until a consumer needs B.

**Q4. Exact feet on the other planar strips** (R3 chamfer, straight entries of chamfer chains). Gains: the same
exactness there. Costs: one foot formula per route. Recommend after Q1, which decides whether exactness pays off.

## 10. Tasks

The exact feet, the corner slab term and their tests are implemented. The API contact tests (§7.3) and the gallery
acceptance run (§7.4) are not.

| Part | Location |
|---|---|
| Feet and carriers (§4.1, §4.2) | `lineFoot`, `chamferFeetInput`, `exactChamferFeet`, `chamferCarriers` in `internal/retainedblend/line_exact.go`; unit tests `TestExactChamferFeet` (§7.1) |
| Wiring (§4.3) | `rewriteLineEdge` in `internal/retainedblend/line_rewrite.go` |
| Slab term (§4.4) | `work.cornerSlab` (`rewrite.go`), the `beta` parameter of `planeFacetVolume` (`plane_charge.go`); `TestPlaneFacetVolumeCornerBounds` |
| Root tests (§7.2) | `TestRetainedLineChamferExact` in `retained_line_chamfer_exact_test.go` |
| Benchmark | `BenchmarkRetainedLineChamfer` in `apitest/retained_blend_fillet_bench_test.go` |
| Amended owners | `agent-docs/modify-design.md` Part E §5.0 step 1, §5.2, §6.3 |

A change to this construction passes the Module boundary gate (`agent-docs/module-boundary.md`): owning section §4,
input class "R1 chamfer of a straight edge between two retained planes of a mesh Boolean result", no new root adapter.
