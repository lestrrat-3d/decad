# Tessellation Design

Normative design for `Body.Tessellate`: the public mesh contract, the shared
chording rules for prism and cup payloads, faceted-body restatement, revolve
tessellation, and the private proofs the mesh boolean consumes. Companion to
`docs/api-design.md` (public surface, "core §N"),
`docs/evaluator-design.md` (analytic evaluator + boolean, "evaluator §N"), and
`docs/verification-design.md` (how bounded results are judged,
"verification §N"). `docs/modify-design.md` owns the cup payload; this document
owns how that payload is tessellated.

This design may land in increments (§13). A payload row that has not landed is
`ErrUnsupported`; no increment returns a mesh before that row's boundary and
area proofs are met. Boolean admission additionally requires the row's
occupied-volume proof.

## 1. Contract

`Mesh` is an output, NEVER a body representation (core §3 invariant 1). Its
public accessors remain:

```go
func (b *Body) Tessellate(tol units.Value) (*Mesh, error)

func (m *Mesh) Vertices() []r3.Vec
func (m *Mesh) Triangles() [][3]int
func (m *Mesh) SourceFaces() []*Face
func (m *Mesh) Bound() units.Value
```

Every successful mesh MUST satisfy all rows:

| Property | Requirement |
|---|---|
| **Geometry** | `Triangles` index `Vertices`; every triangle has positive area; every connected boundary component is a closed, consistently outward-oriented 2-manifold |
| **Deviation** | `Bound` is a two-sided boundary bound: every point of the analytic boundary is within `Bound` of the mesh, and every point of the mesh is within `Bound` of the analytic boundary |
| **Tolerance** | `Bound <= tol`; `Bound` is the largest proven displacement actually used, not the requested tolerance |
| **Provenance** | `len(SourceFaces()) == len(Triangles())`; entry `i` is the live body face whose patch triangle `i` approximates |
| **Sharing** | Two facets meeting along one analytic edge reuse the same vertex indices; coincident coordinates in separately allocated vertices do not satisfy this rule |
| **Determinism** | Equal payload + equal tolerance → equal vertex order, triangle order, source-face order, and export bytes |
| **Immutability** | Accessors return copies; callers cannot change the held mesh or its proof data |

The closed-mesh audit is mandatory for every payload. It counts directed edges:
each directed edge occurs once and its reverse occurs once. A payload with
several shells or lumps may produce several closed components; the audit is per
whole mesh and permits that. The audit is a safety net after construction, not a
replacement for shared sampling.

`tol` is a length magnitude. Wrong kind → `ErrUnitKind`; non-finite →
`ErrNotFinite`; negative → `ErrNegativeMagnitude`; zero → `ErrDegenerate`.
A retired body remains readable, so it may be tessellated. A nil body or a body
with no evaluator payload is rejected before dispatch.

## 2. Private proof record

The public surface exposes only `Bound`; the evaluator also keeps private proof
data on every `Mesh`:

| Proof | Meaning | Consumer |
|---|---|---|
| `sourceBound(face)` | two-sided deviation introduced while tessellating that current operand face; zero when a faceted face is already the held polygons | boolean hidden-tangency pre-pass |
| `areaSlack` | upper bound on `abs(Area(true boundary) - Area(mesh boundary))` | boolean result area bounds |
| `volSymDiff` + `symDiffOK` | upper bound on `volume(TrueBody △ MeshSolid)`, present only after that payload's occupied-volume proof lands | boolean operand error composition |

`sourceBound` and `areaSlack` are mandatory for every returned mesh.
`volSymDiff` is mandatory before a boolean may consume it; an export-only
increment may return a mesh with `symDiffOK == false`. For an analytic payload,
`Mesh.Bound()` is the maximum `sourceBound` over the body. A faceted restatement
copies its payload's composed boundary bound, while its current polygon faces
have zero `sourceBound`: they are the geometry the next boolean operates on.
A proof is rounded
up at every sum/product that could understate it. A non-finite proof is a
refusal, never an infinite bound.

`areaSlack` is computed as a sum of absolute per-patch differences. This is
deliberately stronger than taking the absolute difference of the two total
areas: an outer wall's deficit and a hole wall's excess may cancel in the total,
but both remain available to a later boolean.

`volSymDiff` is about occupied volume, not signed volume. A mesh can have the
right signed volume while losing material outside one wall and gaining the same
amount inside a hole. Cancellation is forbidden.

The payload table is normative:

| Payload | Geometry source | `Bound` | `areaSlack` | `volSymDiff` |
|---|---|---|---|---|
| `prismPayload` | one chording per recorded section loop, shared by walls + caps | max section-curve sagitta | wall chord deficits + both cap circular-segment deficits | section symmetric-difference allowance × sweep height (§5) |
| `cupPayload` | one chording per outer/cavity loop, shared by walls + floors + rims | max section-curve sagitta | absolute per-wall/per-planar-patch differences | outer-prism allowance + cavity-prism allowance (§6) |
| `revolvePayload` | one meridian chording + one global angular sequence | meridian displacement + angular displacement (§8) | absolute true-vs-held difference per wall cell + cap deficits (§10) | construction-homotopy allowance (§11) |
| `facetedPayload` | held polygons | payload's existing boundary bound | payload's composed slack | payload's composed symmetric-difference bound |

## 3. Shared curve chording

Prism, cup, and revolve tessellators read the evaluator's payload, NEVER live
sketch input. They use the same recorded-walk normalization:

1. Convert every `CurveSegment` to the evaluator's line/circle walk.
2. Coalesce consecutive pieces that the analytic body already merged into one
   face. Preserve all source segment indices on that walk.
3. Sample each coalesced walk exactly once.
4. Reuse those samples at every incident face.

A line contributes its start point; its end is the next walk's start. A circular
walk of radius `r` and parameter span `theta` uses `n` equal subarcs. The proven
sagitta is:

```text
s(r, theta, n) = 2 r sin²(abs(theta) / (4 n))
```

Choose the smallest `n` whose upward-rounded sagitta fits its budget. Use the
stable inverse `4 asin(sqrt(tol/(2r)))`, then walk `n` upward until the direct
sagitta test passes. A whole closed circle uses at least three chords. A
circular revolve generator whose two ends are on the axis uses at least two
meridian chords (§9).

One curve may use at most `maxChordsPerWalk` chords. The global revolve angular
sequence has the same cap. Check integer multiplication and allocation sizes
before building any slice. Overflow or a finer request than the caps admit →
`ErrUnsupported`.

A requested tolerance is an upper bound, not a density request. The tessellator
may refine beyond the first admissible `n` to prove topology, non-intersection,
or a finite private bound. It may NEVER coarsen. Refinement is deterministic:
refine the first failing meridian walk in payload order; an angular failure
increments the one global angular count; rebuild and re-audit.

## 4. Source faces and orientation

Build one `role -> *Face` map from the body's live topology. A coalesced wall
may carry several `side(i,j)` origins; all of them MUST resolve to the same face.
Missing or conflicting roles are an evaluator invariant failure and no mesh is
returned.

Assign sources by patch:

| Facet patch | `SourceFaces` entry |
|---|---|
| prism/cup side cell | face carrying that walk's `side(i,j)` / `shellSide(i,j)` role |
| revolve wall cell or pole fan | face carrying that generator's `side(i,j)` role |
| start/end cap | `capStart` / `capEnd` face |
| cup kept cap, pocket floor, rim band | the corresponding `capStart`, `shellCap`, or `rim(i)` face |
| faceted restatement | faceted body face recorded by the payload's facet group |

Walk order carries material on its left. Emit wall triangles from that order,
the sweep sense, and the source face's `reversed` bit. Start caps point against
the sweep; end caps point with it. A reflected placement reverses every final
triangle once. Run the signed-volume orientation audit after the directed-edge
closure audit; every non-void shell must be positive and every void shell
negative under the evaluator's convention.

## 5. Prism

Chord each section loop once. For every 2D sample allocate one vertex at `z0`
and one at `z1`. One section chord sweeps to a planar quad; split it along the
fixed `(bottom-start, top-end)` diagonal into two outward triangles. Both caps
use the same chorded polygon-with-holes and the same vertex indices. The start
cap reverses the shared 2D triangulation; the end cap does not.

Before cap triangulation, prove:

- every chorded loop is simple;
- the outer loop contains every hole;
- holes are pairwise exterior;
- non-adjacent loops clear each other by more than their summed sagitta bounds
  plus the scale-anchored float floor.

Failure triggers refinement. Exhausted refinement → `ErrUnsupported`.

For a circular subarc `c`, let `S_c` be the planar circular segment between the
arc and its chord and let `a_c = area(S_c)` (absolute). Then:

```text
volSymDiff_prism <= abs(z1-z0) * sum_c a_c
```

The sum is an upper bound even when two local slivers overlap. The topology
audit prevents a sliver from changing loop nesting.

`areaSlack` sums:

- `abs(arcLength - chordLength) * abs(z1-z0)` for every circular wall strip;
- `a_c` for each occurrence of that chord on a cap (twice on a prism).

Straight-only prisms therefore have zero boundary bound, zero area slack, and
zero analytic symmetric-difference allowance.

## 6. Cup

A cup is the outer prism `O` minus the cavity prism `C` over their own
intervals (modify §9). Chord every loop of `O` once and every reversed loop of
`C` once. Reuse each ring across its wall, floor, and half of its rim band.

The kept outer cap triangulates `O`; the pocket floor triangulates `C`; rim
`i` triangulates the band between loop `i` of `O` and loop `i` of `C`. The
polygon-with-holes triangulator receives the loop order appropriate to that
planar face, but it MUST index the already allocated ring vertices.

Run the §5 simplicity/nesting/clearance proof over all outer and cavity loops
together before triangulating a floor or rim. This includes tunnels and posts.

Let `E(P)` be the sum of absolute circular-segment areas for a chorded section
`P`, and `hO`, `hC` the outer and cavity interval lengths. The symmetric-
difference inequality for set subtraction gives:

```text
volSymDiff_cup <= hO * E(O) + hC * E(C)
```

Compute `areaSlack` patch by patch: wall arc/chord differences on each wall's
own height, plus absolute circular-segment deficits on the kept cap, pocket
floor, and every rim band. A segment appearing on two planar patches is charged
twice.

## 7. Faceted restatement

A `facetedPayload` has lost analytic identity. `Tessellate` therefore restates
its held vertices and polygons; it NEVER fits or refines them.

- Requested `tol < payload.meshBound` → `ErrUnsupported`.
- Otherwise return the held connectivity and held bound unchanged.
- Map every facet group to the corresponding live faceted face.
- Set every faceted face's `sourceBound` to zero; no new chording was done and
  the polygons are the current operand geometry.
- Copy `areaSlack` and `volSymDiff`; do not recompute either from the held mesh.

The copied `volSymDiff` remains relative to the true boolean result the payload
stands for. Treating the polygons as exact data does not erase that proof.

## 8. Revolve sampling

Read `revolvePayload` directly: `profile`, `frame`, oriented axis frame, sweep
interval `[phi0, phi1]`, `full`, and accumulated rigid transform. Convert the
recorded profile into axis coordinates `(z, rho)`, with `rho >= 0`, and use:

```text
X(z, rho, phi) = axisOrigin + z*w + rho*(cos(phi)*e0 + sin(phi)*e1)
```

The rigid placement is applied last and changes no geometric bound.

Compute `rhoMax` exactly from the payload's line/arc walks: endpoints plus
every circular cardinal point inside a walk's parameter interval. `rhoMax`
is global across outer and hole loops. Non-positive or non-finite `rhoMax` is
an invariant failure; no revolved solid exists entirely on the axis.

Split the tolerance in this order:

1. Give meridian curves `tol/2` as their initial budget.
2. Chord every coalesced meridian walk and record the largest actual sagitta
   `deltaM`.
3. Give angular chording `tol - deltaM`; this is positive because
   `deltaM <= tol/2`.
4. Choose one global angular count from `rhoMax` and that remaining budget.

For maximum angular step `dphi`, the angular displacement is:

```text
deltaPhi = 2 * rhoMax * sin²(dphi / 4)
```

Walk the angular count upward until the upward-rounded
`deltaM + deltaPhi <= tol`. `Mesh.Bound()` is that sum, not either budget.

A partial sweep has `nPhi + 1` angular samples and includes `phi0` and `phi1`
exactly. A full revolution has `nPhi` samples, uses cyclic indices, and does
not duplicate the seam ring. A full revolution uses at least three angular
steps. Every off-axis latitude in every loop uses this same angular sequence.

The global sequence is load-bearing:

- adjacent generator faces share their complete latitude edge;
- a full turn closes without a tolerance seam;
- partial caps use the exact wall vertices at `phi0` / `phi1`;
- one cell proof applies uniformly across all radii.

## 9. Revolve cells, poles, and caps

First form the intermediate surface obtained by revolving every meridian chord
exactly. Every chord is a straight generator and classifies as one of:

| Generator | Intermediate surface |
|---|---|
| constant `rho` | cylinder |
| constant `z` | plane |
| other off-axis line | cone whose extension meets the axis |
| line on the axis | no face |

For one angular interval, the four corners of a cylinder, plane, or cone cell
are coplanar. The two fixed-diagonal triangles therefore equal the angularly
chorded intermediate patch; the diagonal adds no displacement. This fact is
what makes §8's bound `deltaM + deltaPhi`, rather than an unproved diagonal or
twist term.

Represent one meridian sample as a ring:

- `rho > 0` → one vertex per angular sample;
- `rho == 0` → exactly one vertex, interned by its exact axis coordinate
  `(z, 0)` across every incident walk and both partial caps.

Never allocate an angular sequence at a pole/apex. For one cell:

| End rings | Facets |
|---|---|
| both off axis | one planar quad, two triangles per angular interval |
| exactly one on axis | one fan triangle per angular interval |
| both on axis | no wall only when the source walk is an axis line; otherwise refuse an erased generator |

A circular generator whose endpoints both lie on the axis (a sphere meridian)
MUST have at least two meridian chords and at least one proven off-axis interior
sample. More generally, a circular generator with positive interior `rho`
cannot chord to an axis-only polyline. Refine it; budget exhaustion →
`ErrUnsupported`.

Detect numerical ring collapse before emitting facets. A payload sample with
`rho > 0` whose angular vertices coincide, whose radius is not representable,
or whose fan has zero area is not an axis sample. Refuse it as
`ErrUnsupported`; NEVER merge it into a pole.

Partial caps use the §5 polygon-with-holes triangulation in the `(z, rho)`
plane, mapped at `phi0` and `phi1`. They reuse all meridian samples and all
pole vertices. An on-axis line emits no wall, but its one geometric edge is
shared by both caps. Full revolutions emit no caps.

After assembly, test every pair of non-adjacent facets with the boolean's exact
triangle predicates. Shared vertices and shared edges are the only admitted
contacts. A coarse sphere, spindle-like near-axis patch, or concave torus patch
that crosses another patch is refined before retry. No proof before the budget
is exhausted → `ErrUnsupported`.

## 10. Revolve boundary proofs

### 10.1 Two-sided displacement

Let `S` be one true wall patch, `S_M` the same patch with only its meridian
curve replaced by chords, and `T` the returned triangles.

- Rotating a meridian chord is an isometry at each `phi`, so
  `Hausdorff(S, S_M) <= deltaM`.
- Angular chording moves a point at radius `rho` by at most
  `2 rho sin²(dphi/4) <= deltaPhi`.
- §9 proves the angularly chorded `S_M` cell is exactly the two triangles.

Triangle inequality gives both directions:

```text
Hausdorff(S, T) <= deltaM + deltaPhi
```

For a partial cap, angular error is zero and the bound is `deltaM`. A planar
cap with only line edges has zero source bound. Take the maximum source-face
bound for `Mesh.Bound()`.

### 10.2 Area slack

Compute area slack per wall cell. If true generator subwalk `gamma` spans one
angular interval of width `hPhi`, its true patch area is:

```text
Atrue = hPhi * integral_gamma rho ds
```

The integral is closed form for line and circular walks; evaluator §6 already
uses it for analytic revolve face area. Let `Aheld` be the sum of the one or
two emitted triangle areas. Add upward-rounded `abs(Atrue - Aheld)` for the
cell. Pole fans use the same formula.

For each partial cap, add the absolute circular-segment area between every
meridian arc and its chords. Cap triangulation is exact for the chorded planar
region. Add float summation slack after the per-cell bounds; NEVER rely on
signed cancellation between cells, faces, or loops.

## 11. Symmetric-difference proof and booleans

`Mesh.Bound * held mesh area` is NOT the revolve operand bound. A two-sided
Hausdorff bound alone does not prove that product bounds occupied-volume
difference: a torus's inner and outer walls move in opposite material senses,
and a doubly-curved chord cell can gain material while another loses it. The
signed volume error may cancel while the symmetric difference does not.

The revolve mesh MUST carry a construction-homotopy proof. Define:

- `B0`: the analytic revolved body;
- `BM`: the body obtained by replacing each circular meridian subarc with its
  chord, then revolving that chorded section exactly;
- `BH`: the closed polyhedron returned by the angular chording.

Use the set triangle inequality:

```text
volume(B0 △ BH) <= volume(B0 △ BM) + volume(BM △ BH)
```

The meridian term has a closed-form upper bound. For circular-segment sliver
`S_c` between one meridian arc and chord:

```text
Mmeridian = sweepAngle * sum_c abs(integral_S_c rho dA)
```

`rho >= 0`; the first moment of a circular segment about the axis is closed
form. Summing absolute local moments forbids cancellation across outer and hole
loops.

The angular term is the remaining proof obligation. For each straight-
generator cell, use the explicit homotopy from the rotated patch to its chord
plane:

```text
H(lambda, t, u) = axisOrigin + z(t)*w + rho(t) * (
    (1-lambda)*e(phi0 + u*dphi) +
    lambda*((1-u)*e(phi0) + u*e(phi1)))
```

where `z(t)` and `rho(t)` are linear along the meridian chord. Prove a finite
upper bound on the absolute swept volume:

```text
Icell >= integral_[0,1]^3 abs(
    dH/dlambda dot (dH/dt cross dH/du)
) d(lambda,t,u)
```

The implementation may use a closed form when the triple product has fixed
sign or certified interval subdivision otherwise. Sampling is not a proof.
Every interval coefficient is taken from the payload's own floats exactly; an
exhausted subdivision budget leaves the cell unsupported.

The topology and non-adjacent-intersection audits in §§5–9 are antecedents:
they prove the homotopy does not silently change loop nesting or erase a
component. Assemble `H` over every wall cell with the same parameterization on
shared edges; partial caps stay fixed because both angular endpoints stay
fixed. Every intermediate boundary is therefore a closed 2-cycle. Any point
whose inside/outside membership changes is crossed by that boundary, so the
area formula counts it at least once; the sum of absolute swept volumes bounds
the symmetric difference. Then:

```text
volSymDiff_revolve = upRound(Mmeridian + sum_cells Icell + arithmeticSlack)
```

Until every cell has this finite proof, revolve `Tessellate` may serve export,
but the mesh boolean MUST reject the operand with `ErrUnsupported`. It MUST NOT
fall back to `Mesh.Bound * held area`.

Boolean composition then stays evaluator §9's:

1. Tessellate both operands at the evaluator's internal tolerance.
2. Require a complete `volSymDiff` proof from each mesh.
3. Use `sourceBound(face)` for the hidden-tangency pre-pass.
4. Run the exact-predicate mesh boolean.
5. Bound the result volume by `volSymDiffA + volSymDiffB` plus final weld
   rounding.
6. Bound new rim vertices by `(deltaA + deltaB)/sin(theta)`, using each mesh's
   global boundary bound; this remains separate from volume error.
7. Compose `areaSlackA + areaSlackB` plus area dropped by the final weld.

A faceted operand contributes its payload's already composed `volSymDiff`.
Prism, cup, and revolve operands contribute their mesh proofs. No analytic
operand is admitted through a generic `delta * area` shortcut.

## 12. Refusals

Refuse before returning any partial mesh:

| Condition | Result |
|---|---|
| invalid tolerance | core §12's kind/finite/sign sentinel; zero is `ErrDegenerate` |
| payload class not implemented | `ErrUnsupported` |
| faceted request finer than held bound | `ErrUnsupported` |
| meridian/angular/total facet budget exceeded or integer size overflow | `ErrUnsupported` |
| non-finite `rhoMax`, sagitta, area slack, source bound, or symmetric-difference allowance | `ErrUnsupported` unless it proves an impossible payload invariant |
| positive-radius ring numerically collapses; circular generator is erased | `ErrUnsupported` |
| chording cannot prove loop simplicity/nesting/clearance after refinement | `ErrUnsupported` |
| non-adjacent facets intersect after refinement | `ErrUnsupported` |
| directed-edge audit fails or a triangle has zero area | `ErrUnsupported`; a missing/conflicting source role is `ErrDegenerate` because the body topology contradicts its payload |
| revolve mesh has no finite construction-homotopy allowance when used by a boolean | boolean call returns `ErrUnsupported`; export remains available when §§8–10 pass |

NEVER snap, weld, drop a facet, round a near-axis ring onto the axis, or perturb a
sample to make an analytic mesh close. Refine or refuse.

## 13. Increments

| Increment | Lands | Stays staged |
|---|---|---|
| **T1** | common proof record + audits; prism/cup/faceted paths expressed by §§2–7 without changing their public API | revolve |
| **T2** | revolve line generators: cylinder/cone/plane cells, global angular sequence, partial caps, full-turn cycles, poles/apexes, two-sided bound, area slack, STL/OBJ | circular generators; revolve booleans |
| **T3** | circular meridian generators: sphere/torus cells, axis-to-axis minimum, non-adjacent-intersection refinement | revolve booleans |
| **T4** | meridian first-moment allowance + certified per-cell angular homotopy integral; finite `volSymDiff`; revolve admitted to booleans | density improvements |
| **T5** | deterministic local meridian refinement and global angular density improvements that preserve every earlier proof | free-form/NURBS generators |

Each increment ships its computed geometry tests with it. T2/T3 may export a
revolve because §§8–10 prove the mesh itself; they do not enter the boolean
until T4 proves occupied-volume error.

## 14. Test obligations

- Assert directed-edge closure, positive triangle area, outward winding, and
  `len(SourceFaces) == len(Triangles)` on every payload class.
- Assert byte-identical repeated STL/OBJ output.
- Assert `Bound <= tol` at threshold tolerances around every chord-count change.
- Sample true line/cylinder/cone/plane/sphere/torus patches densely only as a
  falsifier: any observed distance above `Bound` fails; passing samples never
  replace the proof.
- Cover a full cylinder, partial cone, planar annulus, full sphere, spherical
  band, ring torus, concave torus wall, reflected placement, holes, and several
  shells.
- Cover poles/apexes at each end and both ends; assert one interned vertex,
  fan triangles only, and no degenerate quad.
- Cover a partial on-axis line; assert both caps share one axis edge.
- Verify every wall/cap/rim triangle maps to the expected live source face,
  including a coalesced face with several origins.
- Check `areaSlack` against high-precision analytic area differences for every
  surface class; tests are falsifiers of the closed-form proof.
- Check prism/cup `volSymDiff` against exact circular-segment examples.
- Prove the T4 interval integrator encloses analytic fixed-sign cells and
  adversarial sign-changing cells; budget exhaustion MUST refuse.
- Exercise revolve×prism and revolve×revolve booleans after T4, including a
  hidden tangency refusal and a shallow crossing whose rim amplification
  refuses.
- Fuzz valid revolve payloads across tolerance scales; a result is either a
  mesh satisfying every invariant or `ErrUnsupported`, never a cracked mesh.

## 15. Open implementation choice

The behavior is fixed through T4. One implementation choice remains: evaluate
§11's absolute triple integral by a complete closed-form sign decomposition or
by certified interval subdivision. Either is admissible only when it proves the
same `Icell`; a heuristic quadrature is not. Choose during T4 from measured
complexity and proof tests, without changing the mesh or boolean contract.
