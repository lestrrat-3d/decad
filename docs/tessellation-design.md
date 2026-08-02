# Tessellation Design

Normative design for `Body.Tessellate` / `Body.TessellateContext`: the public
mesh contract, the shared chording rules for prism and cup payloads, exact loft
restatement, faceted-body restatement, revolve tessellation, and the private
proofs the mesh boolean consumes. Companion to
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
func (b *Body) TessellateContext(ctx context.Context, tol units.Value) (*Mesh, error)

func (m *Mesh) Vertices() []r3.Vec
func (m *Mesh) Triangles() [][3]int
func (m *Mesh) SourceFaces() []*Face
func (m *Mesh) Bound() units.Value
```

`TessellateContext` propagates `ctx` through every cancellable chording,
clearance, triangulation, and audit phase and returns `ctx.Err()` unchanged.
`Tessellate` calls it with `context.Background()` for compatibility. Both
produce the same deterministic mesh when not canceled.

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
| `sourceBound(face)` | two-sided displacement between the true trimmed operand face patch and the held facets used by the current boolean; analytic faces carry every current trim/chording, coordinate-construction, and placement displacement, and faceted faces carry inherited/composed boundary-certificate error | boolean hidden-tangency pre-pass |
| `areaSlack` | cut-stable upper bound on area error: the integral of absolute local true-vs-held area-density error under the certified correspondence, plus separate trim and coordinate-movement allowances | boolean result area bounds |
| `volSymDiff` + `symDiffOK` | upper bound on `volume(TrueBody △ MeshSolid)`, present only after that payload's occupied-volume proof lands | boolean operand error composition |

`sourceBound` and `areaSlack` are mandatory for every returned mesh.
`volSymDiff` is mandatory before a boolean may consume it; an export-only
increment may return a mesh with `symDiffOK == false`. `Mesh.Bound()` is the
maximum `sourceBound` over the body, for every payload class. For an analytic
payload, each source bound includes every displacement introduced by the
current tessellation, including trim chording and every proven coordinate-
construction and final-placement rounding allowance. A planar carrier does not
make a trimmed patch exact: zero requires proof that the held polygon equals the
true trimmed patch and that its stored coordinates add no displacement.
A faceted restatement copies each face's inherited boundary-certificate
displacement; when the payload carries only one global composed displacement
`Delta`, every faceted face uses `Delta`. It is zero only when the inherited
certificate proves that face exact. A proof is rounded up at every sum/product
that could understate it. A non-finite proof is a refusal, never an infinite
bound.

A `facetedPayload` boundary certificate MUST retain global composed `Delta`.
It MAY retain a tighter displacement per live faceted face. Every per-face value
MUST be no greater than `Delta` and cover every true patch mapped to that face.
Missing or incomplete per-face composition falls back to `Delta`; it NEVER
falls back to zero.

`areaSlack` is computed from non-cancelling local bounds. For each certified
true-to-held patch correspondence, integrate the absolute local area-Jacobian
difference, then add trim and coordinate-movement allowances separately. This
is stronger than taking an absolute difference after integrating a patch or a
whole boundary: opposite-sign error may cancel at either scale, but a later
boolean can retain only one sign.

`volSymDiff` is about occupied volume, not signed volume. A mesh can have the
right signed volume while losing material outside one wall and gaining the same
amount inside a hole. Cancellation is forbidden.

The payload table is normative:

| Payload | Geometry source | `sourceBound(face)` | `Bound` | `areaSlack` | `volSymDiff` |
|---|---|---|---|---|---|
| `prismPayload` | one chording per recorded section loop, shared by walls + caps | wall sagitta; each cap's maximum curved-trim sagitta; plus proven coordinate/placement rounding for either; zero only for an exact held trim with exact stored coordinates | max per-face source bound | non-cancelling wall error + both cap circular-segment deficits + coordinate-movement allowance | section symmetric-difference allowance × sweep height + coordinate swept allowance (§5) |
| `cupPayload` | one chording per outer/cavity loop, shared by walls + floors + rims | wall sagitta; each floor/rim patch's maximum curved-trim sagitta; plus proven coordinate/placement rounding for either; zero only for an exact held trim with exact stored coordinates | max per-face source bound | non-cancelling per-wall/per-planar-patch error + coordinate-movement allowance | outer-prism + cavity-prism allowances + coordinate swept allowance (§6) |
| `loftPayload` | exact wall and cap triangles already held by the payload | zero: every held facet is the payload's exact triangle for its source face | zero | zero | zero; `symDiffOK == true` |
| `revolvePayload` | one meridian chording + one global angular sequence, then final rigid placement | current meridian + angular displacement for that analytic patch, plus construction rounding `deltaC` and final-placement rounding `deltaR`; `deltaC + deltaR` for otherwise exact planar patches | max per-face source bound (§8) | integral of absolute local true-vs-held area-density error + cap deficits + construction/placement area allowances (§10) | meridian/angular + construction/placement homotopy allowances (§11) |
| `facetedPayload` | held polygons + inherited boundary certificate | inherited certified face displacement, or global composed `Delta` when no tighter face value exists | max per-face source bound | payload's composed slack | payload's composed symmetric-difference bound |

### `loftPayload` exact restatement

A `loftPayload` already holds the cap triangles from its polygon-with-holes
triangulation and the two exact wall triangles for every paired segment. Its
construction audit proves this fixed triangle boundary is closed, positively
oriented, and free of non-adjacent contact. `Tessellate` copies that triangle
connectivity, vertices, and source faces directly. It MUST NOT chord,
retriangulate, move, round, weld, or otherwise alter a loft facet.

The true boundary is therefore the held triangle boundary. Every
`sourceBound(face)`, `Bound`, and `areaSlack` is zero, and the identical
occupied volumes prove `volSymDiff == 0` with `symDiffOK == true`. The normal
closed-mesh and source-face audits still run. A successful loft mesh is thus
admitted to the mesh boolean as an all-planar zero-bound operand.

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

Choose the smallest `n` whose upward-rounded direct sagitta fits its positive
budget `b`. Let `nMin` be the walk minimum. First evaluate
`upRound(s(r, theta, nMin))`; when it is at most `b`, choose `nMin` without
evaluating an inverse. Otherwise clamp `q = b/(2r)` to `[0, 1]`, compute
`dTheta = 4 asin(sqrt(q))`, and seed the checked search with
`ceil(abs(theta)/dTheta)`, clamped to `nMin`. Decrement while `n-1` remains at
least `nMin` and its upward-rounded direct sagitta fits, then increment while
`n` does not fit. This downward-then-upward correction applies to every section,
meridian, and global angular inverse. If a positive `b` makes `q` or `dTheta`
underflow to zero, the quotient is non-finite, the checked ceiling cannot be
represented, or the resulting count exceeds its fixed cap, refuse with
`ErrUnsupported` before integer conversion or allocation. A whole closed circle
uses at least three chords. A circular revolve generator whose two ends are on
the axis uses at least two meridian chords (§9).

One curve may use at most `maxChordsPerWalk` chords. The global revolve angular
sequence has the same cap. The complete call also has these fixed ceilings:

```text
maxFacetsPerMesh         = 65_536
maxFacetWorkPerCall      = 262_144
maxFacetPairTestsPerCall = 8_000_000
```

`maxFacetWorkPerCall` counts every facet assembled across the initial attempt
and every refinement retry. `maxFacetPairTestsPerCall` counts every exact
facet-pair predicate invocation across all endpoint and placement-homotopy
audits. Neither counter resets during refinement.

Before building any slice, derive the candidate facet count from every walk's
chord count and the global angular count with checked integer arithmetic. Refuse
if that attempt exceeds `maxFacetsPerMesh`, or if adding it would exceed
`maxFacetWorkPerCall`. Before each all-pairs audit, compute the conservative
upper bound `F*(F-1)/2` with checked arithmetic and refuse if adding it would
exceed `maxFacetPairTestsPerCall`; adjacency may reduce actual tests but never
raises the admitted ceiling. Overflow or a finer request than any cap admits →
`ErrUnsupported`. No facet allocation or pair predicate starts before its
corresponding preflight passes.

A requested tolerance is an upper bound, not a density request. The tessellator
may refine beyond the first admissible `n` to prove topology, non-intersection,
or a finite private bound. It may NEVER coarsen. Refinement is deterministic:
refine the first failing meridian walk in payload order; an angular failure
increments the one global angular count; rebuild and re-audit.

## 4. Source faces and orientation

Build one `role -> *Face` map from the body's live topology. A coalesced wall
may carry several `side(i,j)` origins; all of them MUST resolve to the same face.
A Loft wall cell has the two distinct `side(i,j,0)` and `side(i,j,1)` faces
evaluator §3 defines, even when its two triangles are coplanar. No Loft wall
triangle coalesces with one in another cell, so a flat split rung keeps its two
source faces. Missing or conflicting roles are an evaluator invariant failure
and no mesh is returned.

Assign sources by patch:

| Facet patch | `SourceFaces` entry |
|---|---|
| prism/cup side cell | face carrying that walk's `side(i,j)` / `shellSide(i,j)` role |
| Loft wall triangle `k` | face carrying that cell's `side(i,j,k)` role |
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
- every non-adjacent walk pair, within one loop or across loops, clears by more
  than its summed sagitta bounds plus the scale-anchored float floor.

Failure triggers refinement. Exhausted refinement → `ErrUnsupported`.

For a chorded planar region `P`, define `deltaTrim(P)` as the maximum sagitta
of every curved walk on every loop that trims that patch. The §5 proof makes
the straight-line homotopy from each analytic trim to its chord disjoint from
all non-adjacent trims, so every point gained or lost by the planar patch lies
within `deltaTrim(P)` of the other patch. Let `deltaStore(face)` be the proven
three-dimensional displacement from coordinate construction and any placement
write for that face, evaluated with directed-rounding intervals over the same
coordinate pipeline. An unproved or non-finite allowance refuses. Each cap has:

```text
sourceBound(cap) = upRound(deltaTrim(cap) + deltaStore(cap))
```

Each wall uses its own walk sagitta plus `deltaStore(wall)`. A cap bound is zero
only when every trim is straight/exact and every held coordinate is proved
exact; being planar is not enough. `Mesh.Bound()` remains the maximum of these
complete face bounds.

Reserve the maximum `deltaStore(face)` from `tol`, rounding the subtraction
downward, before choosing section chord counts; a non-positive remainder
refuses. If any `deltaStore` is nonzero, certify the affine ideal-to-stored
vertex homotopy with the positive-area/contact rules of §9. Charge its area per
triangle with §10.2's `Rarea_triangle(deltaStore)` and its occupied-volume sweep
with §11's `sweptVolumeAllow` over a perturbed-area upper bound. A source bound
alone does not discharge either proof.

For a circular subarc `c`, let `S_c` be the planar circular segment between the
arc and its chord and let `a_c = area(S_c)` (absolute). Then:

```text
Mprism_analytic = abs(z1-z0) * sum_c a_c
```

The sum is an upper bound even when two local slivers overlap. The topology
audit prevents a sliver from changing loop nesting.

`areaSlack` sums:

- `abs(arcLength - chordLength) * abs(z1-z0)` for every circular wall strip;
- `a_c` for each occurrence of that chord on a cap (twice on a prism).

Add the coordinate-movement area allowance from above after these analytic
terms. With upward-rounded coordinate swept allowance `Mstore`, use
`volSymDiff_prism = upRound(Mprism_analytic + Mstore + arithmeticSlack)`;
signed volume cancellation is not admissible.

Straight-only prisms with exact held coordinates therefore have zero boundary
bound, zero area slack, and zero analytic symmetric-difference allowance.
Otherwise the relevant coordinate-construction/placement allowances remain.

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
For every kept cap, pocket floor, and individual rim band, compute
`deltaTrim(patch)` from every curved outer or hole trim that bounds that patch
and add its `deltaStore(patch)`. A rim is planar but is exact only when both of
its held trim sides equal their analytic trims and its stored coordinates are
exact. Wall bounds use their own sagitta plus `deltaStore(wall)`.

Let `E(P)` be the sum of absolute circular-segment areas for a chorded section
`P`, and `hO`, `hC` the outer and cavity interval lengths. The symmetric-
difference inequality for set subtraction gives:

```text
Mcup_analytic = hO * E(O) + hC * E(C)
```

Compute `areaSlack` patch by patch: wall arc/chord differences on each wall's
own height, plus absolute circular-segment deficits on the kept cap, pocket
floor, and every rim band. A segment appearing on two planar patches is charged
twice. Reserve the cup's maximum `deltaStore` from the chord budget and apply
§5's coordinate homotopy, area, and swept-volume rules over the complete cup
mesh. Add those allowances after the analytic patch and outer/cavity-prism
terms, and set
`volSymDiff_cup = upRound(Mcup_analytic + Mstore + arithmeticSlack)`.

## 7. Faceted restatement

A `facetedPayload` has lost analytic identity. `Tessellate` therefore restates
its held vertices and polygons; it NEVER fits or refines them.

- First populate every face's `sourceBound` from the complete inherited
  certificate, using global `Delta` for each missing or incompletely composed
  face, then set `facetedBound = max(sourceBound(face))`.
- Requested `tol < facetedBound` → `ErrUnsupported`.
- Otherwise return the held connectivity and `facetedBound`. Retain the
  certificate's global `Delta` privately for fallback and later composition;
  it is not a second public `Bound` rule.
- Map every facet group to the corresponding live faceted face.
- NEVER set `sourceBound` to zero merely because no new chording ran; zero
  requires a certificate that the true patch equals its held polygons.
- Copy `areaSlack` and `volSymDiff`; do not recompute either from the held mesh.

The copied `sourceBound`, `Bound`, and `volSymDiff` remain relative to the true
boolean result the payload stands for. Restating held polygons does not erase
the boundary displacement or occupied-volume proof.

## 8. Revolve sampling

Read `revolvePayload` directly: `profile`, `frame`, oriented axis frame, sweep
interval `[phi0, phi1]`, `full`, and accumulated rigid transform. Convert the
recorded profile into axis coordinates `(z, rho)`, with `rho >= 0`, and use:

```text
X(z, rho, phi) = axisOrigin + z*w + rho*(cos(phi)*e0 + sin(phi)*e1)
```

The formula defines the ideal unplaced samples. The implementation evaluates
and stores an unplaced binary64 mesh, then applies the accumulated rigid
placement once after assembly. Both coordinate stages need separate proofs:

- `deltaC` bounds the displacement from every ideal unplaced sample to its
  stored unplaced vertex;
- `deltaR` bounds the displacement from the exact rigid image of each stored
  unplaced vertex to the final stored vertex.

An identity placement that performs no second coordinate operation may have
`deltaR = 0`; it does not make `deltaC` zero.

Compute `rhoMax` and `zAbsMax` exactly from the payload's line/arc walks:
endpoints plus every circular cardinal point inside a walk's parameter
interval. Both are global across outer and hole loops. Non-positive or
non-finite `rhoMax`, or non-finite `zAbsMax`, is an invariant failure; no
revolved solid exists entirely on the axis.

Before choosing chord counts, compute an upward-rounded `coordMax` for every
ideal unplaced analytic-boundary coordinate:

```text
coordMax = max_j upRound(
    abs(axisOrigin_j) + zAbsMax*abs(w_j) +
    rhoMax*(abs(e0_j) + abs(e1_j)))
```

Compute `deltaC` with directed-rounding enclosures over the complete path from
the payload floats to a stored unplaced vertex: profile-to-axis meridian
evaluation, axis-basis construction, angular `sin`/`cos`, every scale/product
and vector addition in `X`, axis-origin addition, and the final binary64 write.
Transcendental and square-root operations require certified enclosures; an
undocumented library error assumption is not a proof. Take the upward-rounded
maximum three-dimensional displacement over every emitted vertex. A constant
or otherwise exactly representable special case may prove a tighter value,
including zero, but identity placement alone may not.

Let `translationMax` be the maximum absolute component of the accumulated
placement's translation. Use the evaluator's proven
`rigidRoundAllow(coordMax + deltaC, translationMax)` as `deltaR`; it covers the
final rotation/addition/write from the already stored unplaced coordinates.
An identity placement that performs no coordinate operation has `deltaR = 0`.
Non-finite `coordMax`, `translationMax`, `deltaC`, or `deltaR`, or failure to
isolate any construction enclosure → `ErrUnsupported`.

Split the tolerance in this order:

1. Compute `available = tol - deltaC - deltaR`, rounding downward;
   non-positive `available` refuses with `ErrUnsupported`.
2. Give meridian curves `available/2` as their initial budget.
3. Chord every coalesced meridian walk and record the largest actual sagitta
   `deltaM`.
4. Give angular chording `available - deltaM`; this is positive because
   `deltaM <= available/2`.
5. Choose one global angular count from `rhoMax` and that remaining budget.

For maximum angular step `dphi`, the angular displacement is:

```text
deltaPhi = 2 * rhoMax * sin²(dphi / 4)
```

Apply §3's downward-then-upward correction to the angular inverse until the
upward-rounded `deltaM + deltaPhi + deltaC + deltaR <= tol`, choosing the
smallest valid angular count. `Mesh.Bound()` is that complete sum, not any
individual budget.

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

Before forming any three-dimensional cell, prove the meridian section for every
full and partial sweep. Run §5's endpoint checks over all chorded outer and hole
loops: each loop is simple, the outer loop contains every hole, and holes are
pairwise exterior. Then certify the complete analytic-to-chord homotopy. For a
circular walk use its matched arc/chord parameterization

```text
Hm(lambda, t) = (1-lambda)*gamma(t) + lambda*chord(t)
```

and use the identity map for a line. Each moving point stays inside its walk's
upward-rounded sagitta tube. Exact section predicates MUST prove that every
non-adjacent pair of walks, including pairs from one loop, clears the two tubes
plus the scale-anchored float floor. Together with the endpoint containment
classification, these disjoint tubes prove simplicity, outer/hole containment,
pairwise hole exteriority, and component count throughout `Hm`. A static
chorded-loop test alone is insufficient. Refine the first failing walk in
deterministic walk order and rerun the whole section proof; an undecidable
predicate or exhausted refinement/work budget → `ErrUnsupported`.

Also run evaluator §6's exact axis-incidence audit on the recorded walks before
sampling. At each exact on-axis loop point, collect every incident walk end
across all loops. A manifold pole has exactly one off-axis walk end and one
on-axis `LineSeg` end from the same loop. A second off-axis sector, repeated
loop incidence, isolated endpoint tangency, or missing on-axis continuation is
`ErrDegenerate`. In particular, splitting a circle tangent to the axis into two
arcs does not turn its shared tangent endpoint into an admissible pole.

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
what makes §8's analytic chording term `deltaM + deltaPhi`, rather than an
unproved diagonal or twist term. Coordinate terms remain `deltaC + deltaR`.

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

Detect numerical ring collapse both before and after final placement. A payload
sample with `rho > 0` whose angular vertices coincide, whose radius is not
representable, or whose fan has zero area is not an axis sample. Refuse it as
`ErrUnsupported`; NEVER merge it into a pole.

Partial caps use the §5 polygon-with-holes triangulation in the `(z, rho)`
plane, mapped at `phi0` and `phi1`. They reuse all meridian samples and all
pole vertices. An on-axis line emits no wall, but its one geometric edge is
shared by both caps. Full revolutions emit no caps.

After unplaced assembly, preflight §3's cumulative pair-work budget. Audit both
the ideal-coordinate angularly chorded endpoint and the stored unplaced
endpoint: every facet is positive-area; adjacent facet interiors meet only on
their shared vertex/edge paths; every non-adjacent pair is disjoint. Shared
vertices and shared edges are the only admitted contacts. Isolate every ideal
endpoint predicate with certified coordinate enclosures from §8; the stored
endpoint uses the boolean's exact predicates over its binary64 values.

Then certify the affine construction homotopy from every ideal unplaced vertex
to its stored unplaced vertex. Run the same positive-area/contact/disjointness
proof over the whole closed interval. The moving predicates have fixed degree
in the homotopy parameter; evaluate their coefficients with certified interval
arithmetic and refine the §8 enclosures until every required sign and root is
isolated. A zero, unisolated sign, or contact triggers deterministic meridian or
angular refinement; no proof before the refinement/work budget is exhausted →
`ErrUnsupported`. This is the topology proof charged to `deltaC`.

Exact rigid placement preserves that result. After final coordinate storage,
run the endpoint audit again and certify a second affine homotopy from each
exact rigid image of the stored unplaced vertex to its final binary64 vertex.
Here the inputs are binary64 and the moving predicates use `math/big.Rat`
exactly. Apply the same full-interval rules; this is the proof charged to
`deltaR`. Sampling or checking only endpoints is never a proof. Preflight and
charge both homotopy audits to §3's cumulative pair-work budget before starting
them. Together with the meridian proof, they preserve loop nesting, shell and
component topology, and contact relations through every construction stage.

Before return, build the combinatorial link of every stored mesh vertex: each
incident triangle contributes the edge between its other two vertices. Every
link vertex MUST have degree two and the complete link MUST be one connected
cycle. More than one cycle at an interned pole is a pinched vertex even when the
directed-edge audit passes. A failed link audit is `ErrUnsupported`; it is the
construction safety net behind the profile-level `ErrDegenerate` refusal.

## 10. Revolve boundary proofs

### 10.1 Two-sided displacement

Let `S` be one true placed wall patch, `S_M` the same patch with only its
meridian curve replaced by chords, `T_E` the ideal-coordinate angularly chorded
triangles after exact rigid placement, `T_C` the exact rigid image of the stored
unplaced triangles, and `T` the final stored triangles.

- Rotating a meridian chord is an isometry at each `phi`, so
  `Hausdorff(S, S_M) <= deltaM`.
- Angular chording moves a point at radius `rho` by at most
  `2 rho sin²(dphi/4) <= deltaPhi`.
- §9 proves the angularly chorded `S_M` cell is exactly `T_E`.
- Construction rounding and exact placement give
  `Hausdorff(T_E, T_C) <= deltaC`.
- Final placement rounding gives `Hausdorff(T_C, T) <= deltaR`.

Triangle inequality gives both directions:

```text
Hausdorff(S, T) <= deltaM + deltaPhi + deltaC + deltaR
```

For a partial cap, angular error is zero and its curved-trim displacement is at
most `deltaM`, so the bound is `deltaM + deltaC + deltaR`. A planar cap with
only exact line trims has source bound `deltaC + deltaR`, which is zero only
when both coordinate stages prove zero. Take the maximum complete source-face
bound for `Mesh.Bound()`.

### 10.2 Area slack

Compute a cut-stable area allowance per wall cell. If true generator subwalk
`gamma` spans one angular interval of width `hPhi`, its completed true patch
area is:

```text
Atrue = hPhi * integral_gamma rho ds
```

The integral is closed form for line and circular walks; evaluator §6 already
uses it for analytic revolve face area. That completed integral is a check, not
the cell allowance, because local positive and negative errors can cancel.

Instead split the cell's common parameter domain `D` along the fixed triangle
diagonal. On each half let `Ftrue` parameterize the analytic meridian/angular
patch and let `Fheld` parameterize the corresponding ideal-coordinate triangle.
Define their non-negative area densities:

```text
Jtrue = length(dFtrue/dt cross dFtrue/du)
Jheld = length(dFheld/dt cross dFheld/du)
Ecell >= integral_D abs(Jtrue - Jheld) d(t,u)
```

Prove `Ecell` upward. Isolate every zero of `Jtrue - Jheld` with exact sign
tests over the payload floats and certified enclosures for the required
algebraic/transcendental values, then integrate each sign-fixed region in closed
form; certified interval subdivision is allowed where sign isolation does not
yield a closed form. Every accepted interval contributes an outward-rounded
absolute integral bound. Sampling is not a proof, and exhausting the shared
fixed certified-interval proof budget → `ErrUnsupported`. Pole fans use the
triangular parameter domains. Because the absolute value is inside the
integral, any corresponding subset retained by a later boolean is bounded by
the whole cell's `Ecell`; rim movement from the new boolean trim remains a
separate allowance.

For each partial cap, add the absolute circular-segment area between every
meridian arc and its chords. Cap triangulation is exact for the chorded planar
region. Then charge both coordinate stages per triangle. If its two edge
vectors before one movement stage are `a` and `b`, moving each vertex by at
most `delta` changes its area by at most:

```text
Rarea_triangle(delta) = delta*(length(a) + length(b)) + 2*delta^2
```

Apply `Rarea_triangle(deltaC)` to every ideal-coordinate wall/cap triangle and
`Rarea_triangle(deltaR)` to every stored-unplaced wall/cap triangle, summing
each stage upward. Add proven arithmetic slack for every length, product, and
sum. Add these construction/placement allowances after the `Ecell` and cap
bounds; NEVER rely on signed cancellation within or between cells, faces, or
loops.

## 11. Symmetric-difference proof and booleans

`Mesh.Bound * held mesh area` is NOT the revolve operand bound. A two-sided
Hausdorff bound alone does not prove that product bounds occupied-volume
difference: a torus's inner and outer walls move in opposite material senses,
and a doubly-curved chord cell can gain material while another loses it. The
signed volume error may cancel while the symmetric difference does not.

The revolve mesh MUST carry construction and placement homotopy proofs. Define:

- `B0`: the analytic revolved body after exact rigid placement;
- `BM`: the body obtained by replacing each circular meridian subarc with its
  chord, then revolving that chorded section and placing it exactly;
- `BH`: the ideal-coordinate angularly chorded closed polyhedron after exact
  rigid placement;
- `BC`: the exact rigid image of the stored unplaced closed polyhedron after
  coordinate-construction rounding;
- `BR`: the returned closed polyhedron after placement-coordinate rounding.

Use the set triangle inequality:

```text
volume(B0 △ BR) <= volume(B0 △ BM) + volume(BM △ BH)
    + volume(BH △ BC) + volume(BC △ BR)
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
Every payload float enters the coefficients exactly; every derived
algebraic/transcendental value uses a certified enclosure. An exhausted shared
fixed certified-interval proof budget leaves the cell unsupported.

The meridian simplicity/nesting/homotopy proof and the three-dimensional
contact audits in §9 are antecedents: they prove these homotopies do not
silently change loop nesting or erase a component. Assemble `H` over every wall
cell with the same parameterization on shared edges; partial caps stay fixed
because both angular endpoints stay fixed. Every intermediate boundary is
therefore a closed 2-cycle. Any point whose inside/outside membership changes
is crossed by that boundary, so the area formula counts it at least once; the
sum of absolute swept volumes bounds the angular-construction symmetric
difference.

The two coordinate homotopies in §9 also keep every intermediate boundary a
closed 2-cycle. Bound construction rounding with
`sweptVolumeAllow(deltaC, perturbedAreaUpper(BH, deltaC))`; call the
upward-rounded result `Mconstruct`. Bound final placement rounding with
`sweptVolumeAllow(deltaR, perturbedAreaUpper(BC, deltaR))`; call it `Mround`.
Each area argument covers its input surface and every surface on that stage's
path, including a facet that would otherwise flatten. Then:

```text
volSymDiff_revolve = upRound(
    Mmeridian + sum_cells Icell + Mconstruct + Mround + arithmeticSlack)
```

Until every cell has this finite proof, revolve `Tessellate` may serve export,
but the mesh boolean MUST reject the operand with `ErrUnsupported`. It MUST NOT
fall back to `Mesh.Bound * held area`.

Boolean composition then stays evaluator §9's:

1. Tessellate both operands at the evaluator's internal tolerance.
2. Require a complete `volSymDiff` proof from each mesh.
3. Use `sourceBound(face)` for the hidden-tangency pre-pass. For a faceted
   operand this is its inherited certified face displacement, or its global
   composed `Delta`, never an automatic zero for restated polygons.
4. For each face pair, upward-round `b = deltaA + deltaB`. When `b > 0`, admit a
   held-facet meet only when the facet sets interpenetrate by a signed depth
   strictly greater than `b` — then A's true surface (within deltaA of its
   facets) and B's (within deltaB of its) still cross even after each is pulled
   inward by its own bound, so the true crossing is proven. A pair whose held
   facets merely touch, overlap by at most `b`, or stay apart within `b` is
   undecidable — the chord error alone could open such a shallow meet from truly
   disjoint surfaces — so return `ErrUnsupported`. The penetration depth is the
   maximum signed penetration of the two facet sets under the exact predicates.
   A tangency without crossing has no positive-bound certificate and stays
   refused. Only a zero-bound pair may pass directly to held-facet predicates as
   exact geometry.
5. Run the exact-predicate mesh boolean.
6. Bound the result volume by `volSymDiffA + volSymDiffB` plus final weld
   rounding.
7. Bound new rim vertices by `(deltaA + deltaB)/sin(theta)`, using each mesh's
   global boundary bound; this remains separate from volume error.
8. Measure the final weld displacement from every exact stitched result vertex
   to its stored binary64 vertex. Upward-round its addition to every incident
   result face's boundary displacement.
9. Compose `areaSlackA + areaSlackB` plus area dropped by the final weld.

A faceted operand contributes its payload's already composed `volSymDiff`.
Prism, cup, loft, and revolve operands contribute their mesh proofs. No
analytic operand is admitted through a generic `delta * area` shortcut.

The hidden-tangency and occupied-volume proofs answer different questions.
`volSymDiff` bounds changed occupied volume; it does not prove that a displaced
true faceted patch cannot touch another operand. Every boolean generation MUST
therefore preserve both the faceted boundary certificate used by `sourceBound`
and the composed `volSymDiff`.

When building the result `facetedPayload`, first assign each result face a
pre-weld displacement covering every inherited `sourceBound` and every new-rim
displacement that can reach it. Let `deltaW(face)` be the upward-rounded maximum
exact-stitched-to-stored coordinate displacement of the welded vertices
incident to that face; zero is allowed only when every such coordinate is exact.
Set the face's complete displacement to
`upRound(preWeld(face) + deltaW(face))`. Set `boundaryCert.Delta` to the
upward-rounded maximum of those complete result-face values. If per-face
composition is incomplete, use `upRound(conservativePreWeldDelta + deltaW)`,
where `deltaW` is the global maximum weld displacement, for every result face
before taking that maximum. Thus every faceted operand `sourceBound`, including
a global-`Delta` fallback, and every final weld displacement flow into the next
result's `boundaryCert.Delta`. This makes the next boolean's hidden-tangency
pre-pass sound without reconstructing analytic identity.

## 12. Refusals

Refuse before returning any partial mesh:

| Condition | Result |
|---|---|
| invalid tolerance | core §12's kind/finite/sign sentinel; zero is `ErrDegenerate` |
| canceled `TessellateContext` | `ctx.Err()` unchanged; no partial mesh |
| payload class not implemented | `ErrUnsupported` |
| faceted request finer than the certified maximum face bound | `ErrUnsupported` |
| meridian/angular, per-mesh facet, cumulative facet-work, cumulative pair-test, or certified-interval proof budget exceeded; integer size overflow | `ErrUnsupported`, before the refused allocation/audit starts |
| non-finite `rhoMax`, `deltaC`, `deltaR`, sagitta, area slack, source bound, construction/placement allowance, or symmetric-difference allowance | `ErrUnsupported` unless it proves an impossible payload invariant |
| positive chording budget whose inverse underflows, cannot produce a represented checked count, or exceeds the owning chord cap | `ErrUnsupported` before integer conversion or allocation |
| recorded on-axis incidence is not exactly one off-axis walk end plus one on-axis line end from the same loop | `ErrDegenerate` |
| positive-radius ring numerically collapses; circular generator is erased | `ErrUnsupported` |
| chording cannot prove loop simplicity/nesting/clearance or preserve it through the analytic-to-chord meridian homotopy after refinement | `ErrUnsupported` |
| non-adjacent facets intersect after refinement | `ErrUnsupported` |
| coordinate construction or placement rounding cannot prove positive facets and unchanged contact/component topology over its affine homotopy | `ErrUnsupported` |
| directed-edge audit fails, a vertex link is not one connected cycle, or a triangle has zero area | `ErrUnsupported`; a missing/conflicting source role is `ErrDegenerate` because the body topology contradicts its payload |
| revolve mesh has no finite construction/placement-homotopy allowance when used by a boolean | boolean call returns `ErrUnsupported`; export remains available when §§8–10 pass |

NEVER snap, weld, drop a facet, round a near-axis ring onto the axis, or perturb a
sample to make an analytic mesh close. Refine or refuse.

## 13. Increments

| Increment | Lands | Stays staged |
|---|---|---|
| **T1** | common proof record + audits; prism/cup/faceted paths expressed by §§2–7 without changing their public API | revolve |
| **T2** | revolve line generators: cylinder/cone/plane cells, smallest-count correction, global angular sequence, partial caps, full-turn cycles, poles/apexes, axis-incidence + vertex-link manifold audits, meridian nesting/homotopy audit, construction/placement rounding proofs, two-sided bound, cut-stable area slack, STL/OBJ | circular generators; revolve booleans |
| **T3** | circular meridian generators: sphere/torus cells, axis-to-axis minimum, circular meridian nesting/homotopy audit, non-adjacent-intersection refinement, cut-stable circular-cell area proof | revolve booleans |
| **T4** | meridian first-moment allowance + certified per-cell angular homotopy integral; finite `volSymDiff`; revolve admitted to booleans | density improvements |
| **T5** | deterministic local meridian refinement and global angular density improvements that preserve every earlier proof | free-form/NURBS generators |
| **T6** | `loftPayload` exact restatement: source-face-preserving wall/cap triangle copy, zero proof record, and mesh-boolean admission | loft surveys and analytic pair clearance |

Each increment ships its computed geometry tests with it. T2/T3 may export a
revolve because §§8–10 prove the mesh itself; they do not enter the boolean
until T4 proves occupied-volume error.

## 14. Test obligations

- Assert directed-edge closure, positive triangle area, outward winding, and
  `len(SourceFaces) == len(Triangles)` on every payload class.
- Assert byte-identical repeated STL/OBJ output.
- Assert `Bound <= tol` and the smallest valid count at threshold tolerances on
  both sides of every chord-count change. Include the radius-1 full-turn
  `n = 122` threshold and each closed-walk/axis-to-axis minimum. For radius 1,
  cover budgets equal to and above `2r`, including 5 mm: choose the minimum
  count without evaluating an out-of-domain inverse. Exercise inverse
  underflow, unrepresentable ceiling, and cap overflow; each MUST refuse before
  conversion or allocation.
- At large coordinate magnitudes under identity placement, assert `deltaC` is
  nonzero when required and is charged to each source bound, `Bound`,
  `areaSlack`, and `volSymDiff`. Repeat under a nonidentity transform and charge
  `deltaR` separately. Refuse when their downward-rounded subtraction leaves no
  positive chording budget.
- Sample true line/cylinder/cone/plane/sphere/torus patches densely only as a
  falsifier: any observed distance above `Bound` fails; passing samples never
  replace the proof.
- Cover a full cylinder, partial cone, planar annulus, full sphere, spherical
  band, ring torus, concave torus wall, reflected placement, holes, and several
  shells.
- Cover poles/apexes at each end and both ends; assert one interned vertex,
  fan triangles only, no degenerate quad, and one connected link cycle. Encode a
  disk tangent to the axis as two semicircular `ArcSeg` walks sharing the
  tangent endpoint; `Revolve` MUST reject the two-sector horn as
  `ErrDegenerate` before tessellation.
- Cover a partial on-axis line; assert both caps share one axis edge.
- Verify every wall/cap/rim triangle maps to the expected live source face,
  including a coalesced face with several origins.
- Give prism/cup caps, floors, and rims circular trims; prove each planar
  `sourceBound` covers its trim sagitta and make a boolean contact lie strictly
  inside the omitted circular segment. Keep a zero-bound case whose planar
  polygon and stored coordinates are both proved exact.
- For full and partial revolves, put a hole in a narrow outer-loop bulge. Cover
  clearances below and above the summed sagitta tubes; assert deterministic
  refinement or refusal before any mesh/proof is returned, and acceptance only
  when simplicity, nesting, and the whole meridian homotopy are certified.
- Check `areaSlack` against high-precision local area differences for every
  surface class. Include an inner-torus cell whose Jacobian error changes sign
  and a downstream boolean retaining one sign lobe; the certified `Ecell` MUST
  bound the retained error without whole-cell cancellation.
- Check prism/cup `volSymDiff` against exact circular-segment examples.
- Cover an admitted `loftPayload`: every wall/cap triangle and source face is
  copied unchanged, `sourceBound`, `Bound`, `areaSlack`, and `volSymDiff` are
  zero, and a loft/prism boolean succeeds through the all-planar path.
- Prove the T4 interval integrator encloses analytic fixed-sign cells and
  adversarial sign-changing cells; budget exhaustion MUST refuse.
- Exercise revolve×prism and revolve×revolve booleans after T4, including a
  hidden tangency refusal and a shallow crossing whose rim amplification
  refuses.
- Exercise a second-generation faceted boolean whose held facets miss a contact
  inside inherited nonzero `Delta`; the hidden-tangency pre-pass MUST refuse.
- Place a body inside the curved hole of a larger annular operand, with a true
  positive gap smaller than the hole wall's `sourceBound`, so chording the
  concave wall inward makes the two held facet sets meet while the true bodies
  stay disjoint. The pre-pass MUST refuse that positive-bound pair; a held meet
  within `b` is never read as a true meet, and no union may return one held lump
  for the true two-lump result.
- Boolean two exact all-planar operands whose rational intersection vertices
  round inexactly at the final weld. Assert the affected face bounds and
  `boundaryCert.Delta` include the upward-rounded coordinate displacement, then
  use that result in a second boolean with contact inside the inherited weld
  allowance; the hidden-tangency pre-pass MUST refuse.
- Exercise a certificate-zero all-planar faceted operand; inherited
  `sourceBound == 0` MUST remain admissible.
- Fuzz valid revolve payloads across tolerance scales; a result is either a
  mesh satisfying every invariant or `ErrUnsupported`, never a cracked mesh.
- Hit each fixed facet/work ceiling exactly and one unit beyond it. Assert the
  over-budget call refuses before allocation or pair testing and repeated
  refinement never resets either cumulative counter.

## 15. Open implementation choice

The behavior is fixed through T4. The proof implementation may combine complete
sign decomposition with certified interval subdivision for §10.2's absolute
Jacobian-difference integral and §11's absolute triple integral. Either path is
admissible only when it proves the same outward-rounded `Ecell` or `Icell` under
the shared fixed subdivision budget; heuristic quadrature is not. Choose during
the owning increment from measured complexity and proof tests, without changing
the mesh or boolean contract.
