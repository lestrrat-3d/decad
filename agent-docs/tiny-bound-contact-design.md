# Tiny-Bound Contact Design

Exact contact classification for planar faces whose held vertices carry a rounding-level bound (about 1e-16 to
1e-15 mm) while the faces themselves lie on exact planes: root cause (§1), success criteria (§2), approach (§3), the
carrier record (§4), the carrier snap (§5), soundness (§6), planar simplification with carrier corners (§7), rational
chamfer feet (§8), the Verify, Intersect and Union paths (§9), performance (§10), tests (§11), risks (§12), open
questions (§13), size (§14), tasks (§15).

Answers `agent-docs/exact-chamfer-design.md` §9 Q1 and Q2. Companion to `agent-docs/coplanar-contact-design.md`
(whose exact classification this design feeds), `agent-docs/faceted-vertex-bounds-design.md` §2 (the corner model)
and `agent-docs/faceted-source-retention-design.md` §2–§3 (group records). Amends `agent-docs/modify-design.md` §7 and
`agent-docs/exact-chamfer-design.md` §2, §4.2, §5. Adds no public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| Why does a 1e-15 bound block flush contact today? | §1 |
| What does decad honestly know about such a face? | §1.2 |
| What is the rule, in one paragraph? | §3 |
| Where does an exact plane come from, and when is it dropped? | §4 |
| How does a rounded vertex become exact again? | §5 |
| Why can a real overlap never read as touching? | §6 |
| Why does the chamfered tail read "proven to overlap", and what fixes it? | §1.3, §8.1 |
| What changes in the R1 chamfer? | §8.2 |
| What does the implementer build, in what order? | §15 |

## Assumptions

- "Exact facet" and "exact face" keep coplanar-contact §3.1's meaning. This design makes more facets exact; it adds
  no new exactness notion to the classifier.
- The corner model (faceted-vertex-bounds §2) holds for every operand mesh: a held vertex `v` stands for a true point
  `v*` with `|v − v*| ≤ β(v)`, and for a planar true face each facet's true piece is the affine triangle on its corners'
  true points.
- Snapping runs only on the non-replay, unshifted, non-fixed-tool attempt of `booleaneval.evaluate` that carries no
  cone-trim certificate (`capFace == nil`). The bounded replay never snaps and declines what it cannot reproduce
  (§13 Q3).
- A face is "planar-true" (`planecarrier.PlanarTrue(f *topo.Face) bool`, Task 3) when `f.Carrier != nil`, or its
  `CapRecord` gives a plane under §4.2 rule 2, or `FlatPatch`, or its surface is `Plane` with `NormalBound == 0` and
  `Denoted == nil`. `HeldPlanar` does not count: it records the source tag only, and a cap-blend band patch is tagged
  `Plane` while its true surface is ruled. No other face is read as planar.
- "Carrier" in this doc means a face's exact plane (§4). It is unrelated to prismrecord's `CarrierRecord`,
  `ExactCarriers` and `CarrierExact` (carrier-record design).
- decad tests use minimal box and prism fixtures and name no consumer model (`agent-docs/testing.md` "Test scope").
  The dovetail check runs in the consumer repository (§11.5).
- No feature flag. Each task merges with the full suite green; Task 9 turns the snap on.

## 1. Problem and root cause

Two solids that meet face to face must verify `Sound`, `Intersect` must return `BooleanEmpty`, and `Union` must return
one solid whose volume is the sum, also when a contact face holds a vertex rounded by about 1e-16 mm. Today exact
coplanar classification admits a facet only when its `FacetBound` and its face's gate reading are zero
(coplanar-contact §3.1). A positive bound fails in two places:

1. **The proximity gate refuses the face pair.** `meshbool.RefuseUndecidableProximity` sums the two faces' readings
   (`FaceGateDelta`). A positive sum with facets that touch but do not cross deeper than the sum is refused
   (`TouchContactError{What: touchNear}`). `Verify` reports `DiagUnsupportedPairContact`; `Intersect` and `Union` get
   `BooleanUnsupportedContact` or fall into the gap retry (`CodeGapShiftApplied`).
2. **The held facet is off its true plane.** A rounded vertex lies up to `β` off the face's true plane. Exact
   predicates over held floats then see a sliver crossing or a sliver gap where the true faces coincide. No exact
   test on held data can separate "touching" from "overlapping by ≤ β".

### 1.1 Observed outcomes

The gallery dovetail (a tail and a socket meeting face to face, each plain or chamfered with today's float feet from
exact-chamfer §1; the dovetail check runs in the consumer repository, §11.5) reads as follows.

| Tail | Socket | `Verify` | `Intersect` | `Union` |
|---|---|---|---|---|
| plain | plain | `Sound` | `BooleanEmpty` | volume equals the sum, no diagnostic |
| chamfered | plain | `Suspect`, `undecided_interference` (proven overlap) | non-empty sliver | volume off the sum, 1 diagnostic |
| plain | chamfered | `Suspect`, `unsupported_pair_contact` | refused (gate) | volume off the sum, 1 diagnostic |
| chamfered | chamfered | `Suspect`, `unsupported_pair_contact` | refused (gate) | volume off the sum, 1 diagnostic |

A minimal mesh fixture shows the same refusal without chamfers: a pentagon prism `Cut` by a prism with a tilted wall
(§11 fixture K) holds rational corners rounded by about 1e-16 mm; `Intersect` against a flush partner is refused (gate)
on the tilted plane and on an axis plane.

### 1.2 What decad knows about a face with a rounding-level bound

| Information | Today | Source |
|---|---|---|
| Per-vertex distance bound `β(v)` | yes | `Payload.VertexBound`, `topo.Mesh.VertexBound` |
| `v*` lies on the true plane of every incident planar face | yes (corner model) | faceted-vertex-bounds §2 |
| Exact plane of a planar face | caps only (`topo.CapRecord` with `AxialDelta == 0` or `ZDenoted`), or a face with three non-collinear zero-bound vertices | `internal/topo/face.go` |
| Retained `Plane` tag as exact plane | no: a float frame; tilted flank tags miss their own zero-bound corners by rounding-level amounts | — |
| Origin of a positive bound (rounding vs displacement) | not recorded | — |
| Side of the true face relative to a held vertex | computable only once the exact plane is known; the true face lies ON its plane, so the side adds nothing | — |

The socket flank after the chamfer keeps two zero-bound vertices, both on one line, so no plane derives from it. The
missing information is an exact plane per face and a way to rebuild `v*`.

### 1.3 The chamfered tail

The tail is an exact prism chamfered by the section rewrite. `offset2d.RewriteLoop` re-emits the trimmed flank as
`LineSeg(c, f̂)` through the float foot `f̂` (modify §7, §5 float evaluation), and the record denotes it
(`SectionDelta` 0). `f̂` lies strictly outside the original flank line (exact-chamfer §1), so the denoted flank is
rotated, and the denoted tail truly overlaps a flush partner by a wedge about `1e-16` mm thick. "Proven to overlap" is
correct for what decad denotes; the denotation is wrong.

**Out of scope:** curved faces; vertices not pinned by three carriers (Rule B, §13 Q1); fillet feet on tilted walls
(§13 Q4); carriers through placements other than the identity (§13 Q5); replay parity (§13 Q3); the gap-retry
ladder; the analytic prism and history routes of `Verify`.

## 2. Success criteria

- **T1.** A planar face whose every vertex is the single meeting point of the exact planes of its incident faces is
  classified as an exact face in every unshifted mesh boolean, whatever its published `β`.
- **T2.** A pair whose contacts are T1 faces reads `Sound`, `Intersect` returns `BooleanEmpty`, and `Union` returns one
  lump with volume within its bound of the sum and no `CodeGapShiftApplied` (fixtures K, G of §11).
- **T3.** A pair overlapping by any positive depth between T1 faces never reads touching; an overlap whose exact volume
  exceeds its bound reads `Interfering` (§11 C2).
- **T4.** A face without a carrier, or with a vertex the carriers do not pin, keeps today's behavior exactly.
- **T5.** A prism or revolve section chamfer keeps each trimmed line wall on its own support line (§8.1).
- **T6.** An R1 chamfer on the exact path denotes a chamfer plane through exact rational data, and every foot is the
  meeting point of three carriers (§8.2).
- **T7.** The six consumer dovetail parameter sets meet T2 with the chamfers kept on both halves (§11.5).
- **T8.** Booleans with no snapped vertex stay within benchmark noise (§10).

## 3. Approach

**Rule A (the rule).** Every planar face carries, where a producer can prove it, an exact rational plane its true patch
lies in (its carrier, §4). Before a boolean classifies, decad replaces each rounded vertex whose incident faces are all
planar-true and whose carriers meet in exactly one point by that point (§5). That point is the vertex's true point, so
the vertex's claim drops to 0. A face whose vertices are all exact afterwards is held exactly, and the existing exact
classification (coplanar-contact §3, proximity gate at zero slack) decides touching versus overlap with no tolerance.
Any vertex the carriers do not pin keeps its bound, and its faces keep today's refusals.

Two producer changes make the dovetail's vertices pinned. **Q2-N**: a section chamfer re-emits a trimmed line walk as
its own source line narrowed to the foot, so the junction to the chamfer chord closes through the two lines' exact
crossing and the wall stays on its plane (§8.1). **D1**: the R1 chamfer denotes the chamfer plane through its float
feet and the edge direction, and its feet as that plane's crossings with the adjacent carriers (§8.2). Both denote a
setback within about one rounding unit of the requested one.

**Alternatives considered:**

- **Rule B: certify bounded contact without rebuilding exact points.** Admit coplanar pairs whose corners lie on a
  shared carrier within their bounds, guarded by a half-space certificate in the gate (§13 Q1 states it). Rejected as
  the first step: it is a second soundness-critical classifier; Rule A plus D1 covers the dovetail.
  Kept as the path if D1 is overridden.
- **Weaken the gate by a tolerance.** Rejected by direction: a true overlap within the tolerance would pass.
- **Read retained `Plane` tags as exact planes.** Unsound: tags are float frames that miss exact corners (§1.2).
- **Derive planes from zero-bound vertices only.** Too weak: the chamfered socket flank keeps two collinear zero-bound
  vertices.
- **Decide in the planar arm of `Verify`.** It holds dyadic coordinates, and a carrier meeting point is rational with
  an arbitrary denominator. It would also leave `Intersect` and `Union` unsolved.
- **Q2 alternatives** for the tail: §8.1.

## 4. The carrier record

### 4.1 Data and claim

```go
// PlaneCarrier is the exact plane {x : N·x = C} a planar face's true patch lies in
// (agent-docs/tiny-bound-contact-design.md §4.1). N is a primitive integer vector whose first nonzero
// component is positive, so equal planes compare equal field by field. Immutable once built.
type PlaneCarrier struct {
	N [3]*big.Int
	C *big.Rat
}
```

- `topo.Face` gains `Carrier *PlaneCarrier` (nil: none recorded).
- **Claim C.** A non-nil carrier states that every point of the face's true patch lies on the plane. A producer sets
  it only where it proves Claim C.
- The carrier is unoriented. Facing comes from the held facets, as today.

### 4.2 Reader

`planecarrier.Of(m *topo.Mesh, f *topo.Face, beta []float64) (topo.PlaneCarrier, bool)` returns, in order:

1. `f.Carrier` when non-nil;
2. the `CapRecord` plane when `f.CapRecord != nil` and either `ZDenoted != nil` (level `ZDenoted`) or
   `HasAxialDelta && AxialDelta == 0` (level `Z`), every value read as an exact rational (topo.CapRecord's own claim):
   with `B` the linear part of `Xform` and `t` its translation, the normal is `(B·U)×(B·V)` and the plane passes
   through `B·(O + level·N) + t` (as `capRecordNormal` in `internal/facetedrecord/place.go` reads it);
3. the plane through the first three non-collinear zero-bound vertices of `f`'s facets in `m`, when `f` is planar-true
   by another clause (each such vertex is a true point of the planar true face);
4. none.

Every consumer computes each face's carrier once, before any vertex moves, so rule 3 never reads a snapped vertex.

### 4.3 Producers

- **Prism lateral faces** (`prismrecord.Build`'s own `buildLoopSidesAs` calls only; the external `BuildLoopSides*`
  callers in `capblend`, `stackedrecord`, `revolvebore` and `loftrecord` set none). Set on the face of a coalesced line
  walk when `SectionDelta == 0`, every segment of the walk is a `LineSeg` (coalescing is exact, `boundarywalk`
  `exactlyCollinear`), and each junction at the walk's ends is one point or closes through the line–line corner
  (evaluator §4; `momentinput.JunctionCorner`'s classification). Carrier: the plane through `Xform∘Frame` of the walk's
  first `Start` and last `End` at `z0`, and its first `Start` at `z1`, in exact rationals. Claim C: the denoted segments
  and their corner legs lie on the support line, and the frame map is affine.
- **Mesh boolean results** (`booleaneval`). Every group whose operand face has a carrier and whose piece did not move
  keeps that carrier. Claim C: the true result patch is a piece of the true operand face (faceted-source-retention
  Claim R).
- **R1 chamfer, exact path** (`retainedblend`): faces `A`, `B`, `G_i` and the chamfer face, per §8.2.
- **Caps** need no producer: the reader takes their `CapRecord`.

### 4.4 Transfer and fail-closed binding

- `facetedrecord.Group` gains `Carrier *topo.PlaneCarrier`. `booleaneval.sourceGroup` sets it to
  `planecarrier.Of(operand mesh, f, β)` (all three rules of §4.2; Claim C holds for each), so §5 and §7 read the same
  carrier set. `shift_certificate_snapshot.go`'s group comparison compares `Carrier` with `planecarrier.Equal`
  (nil equals nil only).
- `facetedrecord.Payload` gains `CarrierMesh occupancy.Binding`: the held mesh the group carriers were proven for.
  `meshOfPayload` reads carriers only when `CarrierMesh.Matches(Verts, Tris)`; `AssembleBody` then sets each face's
  `Carrier` from its group. Any payload whose arrays were rebuilt (placement, similarity, every modify route not listed
  in §4.3) therefore assembles with no carriers. NEVER set `CarrierMesh` outside the §4.3 producers.
- `boolshift` (`moved_tag.go`) sets `Carrier = nil` on every group of a moved or grown piece.
- `facemerge`: a merged group keeps `Carrier` only when every merged group carries an equal carrier
  (`planecarrier.Equal`); else nil.
- The generic boolean sets `payload.CarrierMesh = occupancy.Bind(payload.Verts, payload.Tris)` on every result.

## 5. The carrier snap

### 5.1 Rule

`(*meshbool.BoolMesh).SnapToCarriers(m *topo.Mesh) (snapped int, charge SnapCharge, err error)` (§5.4 defines
`SnapCharge`):

1. **Carriers.** Read every source face's carrier once with `planecarrier.Of(m, f, bm.VertexBound)`, before any move.
2. **Candidates.** A vertex `v` with `β(v) > 0` is a candidate when every face around it is planar-true and
   `planecarrier.Meet` of the distinct carriers of those faces returns one point `p_v`: the normals span R³ and every
   plane contains `p_v`.
3. **Falsify-only check.** For every candidate, `|v − p_v| ≤ β(v)` in exact arithmetic (squared, rounded up). Any failure
   disproves a record or a bound: the whole operand falls back (`snapped = 0`). The check never admits anything.
4. **No partly moved facet.** Drop a candidate when a facet around it has a corner that is neither a candidate nor
   zero-bound. Repeat until the candidate set stops shrinking (it only shrinks). Every facet then has either all corners
   moved-or-exact ("moved facet") or no moved corner.
5. **Move.** Save copies of `Verts`, `Xverts`, `Norms`, `Fnorms`, `FnormsReady`, `Boxes`, `VertexBound`, `FacetBound`,
   `SlabBound` and `FaceSurface`. Copy `Verts` first (`PrepContext` aliases `m.Vertices`; NEVER write through). For each
   remaining candidate: `Xverts[v] = p_v`, `Verts[v] = p_v.Vec()`, `Snapped[v] = true`, `VertexBound[v] = 0`.
6. **Refresh.** For each moved facet: `FacetBound[t] = 0`, `SlabBound[t] = 0`, exact `Norms[t]` from `Xverts`, `Boxes[t]`
   from `Xverts` rounded outward, `FnormsReady[t] = false`. Then rebuild `bm.Parity` (`NewParityMesh` borrows `Verts`,
   `Xverts`, `Norms` and `Boxes`) and call `bm.setDepthBlocks()`. Facets with no moved corner keep every bound, so T4
   holds exactly.
7. **Face readings.** Allocate `FaceSurface` when nil. For a planar-true face with a moved corner: set
   `FaceSurface[f] = 0` when every facet of `f` is a moved facet or has `FacetBound == 0`, and `f` has no trim chords
   (`m.Planar() == nil` or `len(rec.Chords[f]) == 0`); otherwise delete `FaceSurface[f]` (a refined σ was proven for the
   old facets). `FaceGateDelta` and `SetExactFacets` then read the new state with no change to either.
8. **Validate, else fall back.** §5.2. On failure restore every saved array and return `snapped = 0`. The boolean then
   runs as today.

### 5.2 Validation of the moved operand

- Each moved facet's exact normal is nonzero and has a positive dot product with its pre-snap exact normal.
- No moved facet folds onto a facet sharing one of its edges: reuse the per-pair fold predicate of `HeldFolds`
  (`internal/meshbool/held_embedding.go`) over exact points.
- No moved facet properly intersects a non-adjacent facet whose box comes within `2·max β` of it (padded query on a box
  tree over the operand's boxes), by exact triangle predicates.

### 5.3 Float reads of snapped vertices

meshbool reads `BoolMesh.Verts`, `TriCorners`, `Fnorms`, `Boxes` and `ParityMesh.Verts` as if they were exact. Every
such read on the live path falls in one class:

| Class | Use | Required change |
|---|---|---|
| E | an exact coordinate (`proof.XptOf`, `proof.OrientSign` or `proof.DyVec` over floats) | read `Xverts` (or the `Xpt` form of the predicate) when any involved vertex is snapped |
| F | a reject-only float filter (`TriTriMissesFilter`, `TriTriDistanceExceeds`, `SegFilter`, box tests) | abstain (take the exact path) when any involved vertex is snapped |
| A | an approximation with a stated error (reach, diagnostics, boxes) | recompute from `Xverts` when any involved vertex is snapped |

Audit scope: every hit of
`grep -n '\.Verts\b\|TriCorners(\|\.Fnorms\b\|\.Boxes\b' internal/meshbool/*.go internal/booleaneval/*.go | grep -v _test`
outside the replay files (`brake_*.go`, `*_replay*.go`, `replay_*.go`), which never see a snapped operand. The live files are listed in Tasks 5 and 6. Each PR classifies every hit in its description.

**Poison build.** Under the build tag `snappoison` (file `internal/meshbool/snap_poison_on.go`; the default build
compiles `snap_poison_off.go`), step 5 stores `NaN` in `Verts[v]` for snapped vertices. The detection rule is equality:
every snap test asserts the same exact values with and without the tag. A missed class E/F/A read changes a result,
errors or panics under the tag. Commands are in Tasks 5, 6 and 9.

### 5.4 Downstream of the snap

- The stitch already rounds every exact point and measures its gap. A snapped operand vertex enters it with claim 0, so
  its result bound is its rounding gap.
- **Operand volume and area terms.** `symA`/`symB` (`OperandSymDiff`) and `ma.AreaSlack`/`mb.AreaSlack` were proven for
  the unsnapped mesh. For a snapped operand add, rounded up: to its symmetric-difference term,
  `Σ_moved t (2·m_t·area(t) + π·m_t²·perimeter(t) + (4/3)·π·m_t³)` with `m_t` the largest move of a corner of `t` (the
  straight-line move of each point stays within `m_t` of `t`, so the swept set lies in `t`'s `m_t`-offset, whose volume
  is that Steiner bound; then `|X Δ Z| ≤ |X Δ Y| + |Y Δ Z|`); to its area slack,
  `Σ_moved t proofbound.PerturbedTriangleAreaAllow(a, b, c, m_t)`. `SnapToCarriers` returns both terms
  (`type SnapCharge struct{ Vol, Area float64 }`).
- Occupancy: an attempt with `snapped > 0` on either operand tracks no occupancy (the `Bulge` reading was proven for
  the unsnapped held solid).
- `RefuseCoarseHeldContact` and the chain gate read the lowered `FacetBound`.

## 6. Soundness

**Claim S.** After §5, every moved facet IS its true piece, every face with `FaceSurface == 0` IS its true face, and
every other facet and face keeps its published bound unchanged.

*Proof.* Let `v` move to `p_v`. Each face `F` around `v` is planar-true, so the true piece of each facet of `F` at `v` is
the affine triangle on true corners (corner model) and lies in `F`'s true patch; hence `v*` lies on `F`'s carrier
(Claim C). So `v* ∈ ∩ carriers = {p_v}` and `v* = p_v`. A moved facet has every corner moved or zero-bound, so its held
corners equal its true corners and it equals its true piece (planar-true face). A facet with no moved corner did not
change. A face set to `FaceSurface == 0` has every facet equal to its true piece and no trim chord, so it is the union of
its true pieces, its true face. Closure is unchanged; §5.2 checks orientation, folds and proper intersections of every
moved facet. ∎

**Consequences.**

- The held operand after §5 satisfies every premise coplanar-contact §9 and interference §6.1 (P1–P3) read, with more
  facets exact and with §5.4's terms added to its volume and area bounds. Their proofs apply unchanged: an admitted
  coplanar pair is classified by exact predicates over true geometry, and an empty unshifted `Intersect` proves disjoint
  interiors.
- **Overlap of any depth is never touching (T3).** Between exact facets, an overlap of depth `d > 0` is an exact
  crossing or a same-facing coincident patch (coplanar-contact §9 points 4–5): `Intersect` is non-empty and §6's
  `V − E > 0` gate decides the row. Where a contact facet stays bounded, the gate refuses unless the crossing is proven
  deeper than the summed bound, as today. No pair moves from refused to touching except by becoming exact.
- **Touching only when the true solids cannot interpenetrate.** `Verify` reads touching only from an empty unshifted
  `Intersect` (interference §6.1) or from routes this design does not touch.
- Snapping is construction from upstream claims (Claim C and the corner model); its residual check only rejects
  (CLAUDE.md "falsify-only").

## 7. Planar simplification with carrier corners

coplanar-contact §7.1 step 3 keeps a candidate face only when every corner is a float. Extend it: a non-float corner
`p` is also accepted when the carriers of the distinct source ids around `p` meet in exactly `p`
(`planecarrier.Meet`). The face is re-triangulated on its corners as today; its corners round with a positive gap, so
its `FaceBound` is that gap, and the next boolean's snap restores them exactly. §7.2's argument is unchanged: the exact
held surface before rounding does not change and no nonzero claim is dropped.

- `meshbool.StitchOptions` gains `Carriers []*topo.PlaneCarrier`, indexed by source id (nil: none). booleaneval fills
  it from `groups[i].Carrier`, which §4.4 sets from `planecarrier.Of` (recorded, cap or derived plane), so §5 and §7
  read one carrier set.
- The replay stitch (`StitchFacetsReplayContact`, called from `booleaneval.evaluate`) gains a final
  `carriers []*topo.PlaneCarrier` parameter and applies the same rule, charging one work unit per corner check, so live
  and replay held meshes stay equal for every boolean both build.

Without this step, a mesh result with a non-float corner keeps artifact vertices on its faces; an artifact vertex lies
on one or two carriers only, so §5 cannot pin it and its faces stay bounded.

## 8. Rational chamfer feet

### 8.1 Section chamfer (prism and revolve), answering exact-chamfer Q2

**Rule Q2-N.** In `offset2d.RewriteLoop`, a line walk trimmed at a chamfer foot whose float position is not exactly on
the walk's support line (exact orientation ≠ 0) is re-emitted as a `LineSeg` on its source support: `Start` and `End`
are the source segment's (for a merged collinear walk, the first segment's `Start` and the last's `End`), and each
trimmed end's parameter is the float projection parameter of the foot. Untrimmed ends keep the source parameters. The
connector stays `LineSeg(f̂A, f̂B)`. The junction between the wall and the connector then closes through the two
supports' exact crossing (evaluator §4), which lies on the wall's support. Walks whose foot lies exactly on the support,
arc walks, and fillet connectors keep today's output bit for bit.

- **The junction vertex sits at the corner.** `boundarywalk.JunctionVertex` holds a junction's vertex at the next
  walk's start, with a bound covering the two walk ends but not the corner the closing path passes through. With Q2-N
  the float foot `f̂A` lies about `h/sin θ` from the corner, more than its bound, so the wall's true corner would be
  uncovered and §5.1 step 3 would reject the operand. Change `junctionVertexWithBounds` (and its replay twin
  `JunctionVertexAtReplay`): where both neighbours are `LineSeg`s and the junction closes through the provably near
  line–line corner (evaluator §4), hold the vertex at the corner's nearest float, with bound the exact per-component
  rounding gap. `momentinput` imports `boundarywalk`, so move `momentinput`'s line–line crossing (`lineCorner` and the
  1024-ratio test of `JunctionCorner`) into `boundarywalk` as `LineCorner` and have `momentinput` call it; never copy
  it. This also closes the same uncovered-corner gap on existing corner-closed junctions (union sections).
- decad computes the foot's exact orientation and projection parameter itself instead of asking `sketch`: one sign and
  one parameter per chamfer corner over decad's own recorded segment and its own float foot; a private scene per corner
  would cost a scene build for one sign (CLAUDE.md "Ask sketch", coplanar-contact §3's reason).
- `offset2d.Blend` gains `Chamfer bool`, set by `ChamferWithCarriers`.
- `RewriteLoop(budget, walks, blends, sources []sectionrecord.CurveSegment)`: `sources` is the loop's original segment
  list; `RewriteProfileBudget` passes it. The revolve meridian rewrite shares the function and takes the rule too.
- The record still denotes itself: `SectionDelta` stays 0, no reader refuses it, and §4.3's lateral producer gives the
  wall its original plane, the connector the plane through `f̂A`, `f̂B`, and every corner three carriers.
- Denotation: the chamfer whose plane passes through `f̂A`, `f̂B` (parallel to the sweep) and whose feet are its
  crossings with the source supports. The setback differs from `d` by at most the feet's rounding (about one ulp of the
  coordinates).

Options compared:

- **Q2-N (chosen).** Gains: the wall stays on its plane, `SectionDelta` stays 0, every corner is pinned, no new
  refusal. Costs: the setback is within about one ulp of `d`; tests pinning chamfer foot coordinates or zero foot bounds
  flip.
- **Q2-A: denote the ideal chamfer, charge the foot residual as `SectionDelta`.** Gains: the exact requested setback.
  Costs: a displaced receiver, so `RequireExactSection` refuses later prism modify ops; the ideal feet are irrational,
  so they are not pinned and need Rule B (§13 Q1).
- **Q2-B: carrier record per carrier-record §7.3.** Gains: carriers readable by later analytic ops. Costs: §7.3's
  exact-carrier corner joins the carrier to the float foot by a chord, which sticks out up to `r_f` past the wall when
  the foot lies outside the wall (the tail's does), so the tail still truly overlaps; §7.3's other-corner rule is Q2-A
  plus a record producer.
- **Q2-C: keep the convention.** Gains: nothing changes. Costs: chamfered prisms keep reading "proven overlap" against
  flush partners.

### 8.2 R1 chamfer, amending exact-chamfer (decision D1)

On exact-chamfer §4.1's exact path, after §4.2 computes the enclosures of the ideal feet at end 0:

1. `P̂_A,0`, `P̂_B,0` = their `BoxMidStation` floats.
2. Chamfer carrier `C = planecarrier.Through(P̂_A,0, P̂_B,0, P̂_A,0 + E)`, `E = V₁ − V₀` exact.
3. Each foot is `Meet(C, plane(A), plane(G_i))` or `Meet(C, plane(B), plane(G_i))`, with `plane(A)` through `V₀, V₁,
   cA₀`, `plane(B)` through `V₀, V₁, cB₀`, `plane(G_i)` through `V_i, cA_i, cB_i` (exact-chamfer §4.1's carriers).
   Held foot = `Vec()` of the exact point; its bound = the exact rounding gap (`Radius3D` of per-coordinate gaps,
   rounded up; 0 for a float).
4. Groups: `A`, `B`, `G_i` keep the receiver's carriers when valid (§4.4), else take the planes of step 3; the chamfer
   group takes `C`; untouched groups keep the receiver's valid carriers. `retainedblend` sets `CarrierMesh` only when
   every selected edge took the exact path; any other call sets none.

Claim X becomes: each held foot is within its bound of the meeting point it stands for, exactly. Claim K is unchanged in
form with these bounds. The ideal-feet enclosure of exact-chamfer §4.1–§4.2 (`exactChamferFeet`) stays in use for step 1.

## 9. Verify, Intersect and Union

- **Intersect** and **Union** run `booleaneval.evaluate`; the snap runs before `SetExactFacets` on the non-replay,
  unshifted, non-fixed attempt with `capFace == nil`, and adds `SnapCharge` to `symA`/`symB` and the area slack.
- **Verify** reaches the same evaluation through `MeasuredInterference` (interference §5). The planar arm
  (`verifyrun/clearance.go`) and the history routes are unchanged; with a positive held bound the planar arm stays
  undecided and the pair falls through to the read-only `Intersect`.
- **Cut** gains the same behavior; no Cut-specific code.

## 10. Performance

- Snap: per vertex with `β > 0`, one exact 3×3 solve on small integers; per snapped facet, one exact normal and box. The
  dovetail pair has under 40 such vertices.
- Class E/F reads of snapped facets take the exact path; facets with no snapped corner take today's float paths.
- §7's corner check: one `Meet` per non-float corner of a candidate face.
- Carriers: one pointer per face and group; three big integers and one rational per distinct plane.
- Benchmarks (Task 8, Task 9): `BenchmarkMeshBooleanCrossing` (no snap) within 5%; new `BenchmarkCarrierFlush` (fixture
  K vs G through `booleaneval.Evaluate`, `Intersect`) reported before and after. Record the consumer dovetail stage wall
  time before and after (§11.5).

## 11. Test plan

Every test asserts computed geometry. Fixtures (float coordinates, mm):

- **K**: pentagon `(0,0) (10,0) (10,16) (5,18) (0,16)` on XY extruded +Z by 10, `Cut` by the prism on XZ (frame
  `U = +X`, `V = +Z`, origin `y = −1`; its normal is −Y, so `decad.Against`) through `(−1,3) (11,4) (11,11) (−1,11)`
  extruded 20 toward +Y. Its tilted top meets
  `x = 0` and `x = 10` at `z = 37/12` and `47/12`, rational non-floats (held corners carry `0 < β` about 1e-16, §1.1).
- **F**: box `[x0,20]×[0,18]×[0,6]` (XY section, +Z), `x0 = 10` unless a case draws it elsewhere (never moved by
  `Transform`). **P2**: a fresh copy of K's tool prism. **G** = `Union(P2, F)`.
- **S**, **T**: exact-chamfer §7's slot block and tenon (slope-1/6 wall through `(5,0)`, `(6,6)`).

### 11.1 Unit

| ID | Test | Expectation |
|---|---|---|
| U1 | `TestPlaneCarrierThroughMeetEqual` (`internal/planecarrier`) | canonical form; collinear points refused; `Meet` of 3 independent planes = hand point; 2 planes or inconsistent 3 refused |
| U2 | `TestCarrierOfReader` | recorded > CapRecord > derived; `AxialDelta > 0` without `ZDenoted` gives none; a non-planar face and a face descending from a band patch give none |
| U3 | `TestPrismLateralCarriers` (`internal/prismrecord`) | every `LineSeg` wall of an exact prism carries the plane of its segment; a chord-closed junction's neighbours carry none; `SectionDelta > 0` gives none |
| U4 | `TestCarrierTransferAndDrop` (`internal/facetedrecord`, `internal/booleaneval`) | a Cut result's faces carry their operands' carriers; a placed copy and a shifted-retry piece carry none; `CarrierMesh` mismatch reads none |
| U5 | `TestSnapToCarriers` (`internal/meshbool`) | rational-box corner snapped to its exact point with claim 0; `m.Vertices` unchanged; `Parity` and depth blocks read the snapped points |
| U5b | `TestSnapSkips` | a vertex on two carriers, beside a non-planar or band-patch face, or on a facet with an unpinned bounded corner stays; a residual above `β` falls the operand back |
| U6 | `TestSnapFallback` | a forced zero normal, a flipped sliver, a fold over a shared edge and a proper intersection each restore every saved array |
| U7 | `TestRationalBoxBooleans` | boxes with corners at thirds, held as floats and snapped by a test helper: `Union`, `Intersect`, `Cut` exact volumes equal the rational hand values; same assertions pass with `-tags snappoison` |
| U8 | `TestSimplifyPlanarCarrierCorner` | a face with a non-float corner pinned by three carriers is simplified; without carriers it is untouched; live and replay equal |
| U9 | `TestRewriteLoopChamferOnSupport` (`internal/offset2d`) | slope-1/6 corner, prism section and revolve meridian: emitted wall support equals the source support and holds the wall–connector corner (exact); a square corner's output is bit-identical to the parent |
| U11 | `TestJunctionVertexAtCorner` (`internal/boundarywalk`) | a corner-closed line–line junction holds its vertex at the corner's nearest float with bound equal to its rounding gap; a shared-point junction is unchanged; live and replay equal |
| U10 | `TestRationalChamferFeet` (`internal/retainedblend`) | each foot lies exactly on `C`, its side plane and `G_i`; `|held − foot| ≤ bound`; `|dist(foot, edge line) − t| ≤ 4·bound` |

### 11.2 Contact (`apitest/carrier_contact_test.go`)

Each case runs in a default and an `ExactOnly` document. Before Task 9 merges, the implementer runs C1–C3 on the parent
commit and records today's outcomes in the PR.

| ID | Case | Expectation |
|---|---|---|
| C1a | K vs F (flush on `x = 10`) | `Intersect` `BooleanEmpty`; `Union` one lump, volume within bound of the sum, no `CodeGapShiftApplied`; `Verify` `Sound` |
| C1b | K vs P2 (flush on the tilted plane) | as C1a |
| C1c | K vs G (flush on both planes) | as C1a; precondition: K's and G's contact faces hold a vertex with `0 < β ≤ 1e-14` |
| C2a | F drawn at `x0 = 10 − 2⁻⁴⁰` (overlap) | `Verify` `Interfering`; `Intersect` volume with `Value − Bound > 0` |
| C2b | F drawn at `x0 = 10 − 2⁻⁴⁹`, the next float below 10 (overlap of about the old bound) | `Verify` not `Sound`; `Intersect` not `BooleanEmpty` |
| C2c | F drawn at `x0 = 10 + 2⁻⁴⁹·2` (gap) | `Verify` `Sound`; `Union` two lumps |
| C3 | K placed by a 30° rotation about z and back, vs F | today's outcome (carriers dropped, rule not applied) |
| P1 | 24 seeded K-like cuts (wall slope `a/b`, `a ∈ [1,5]`, `b ∈ [7,13]`) against G-like partners, half moved in by `2⁻⁴⁰` | flush: as C1a; moved: as C2a |

`Verify` may read `Sound` on C1 today through a history or half-space route (`verifyrun/csg.go`, `halfspace.go`);
the `Intersect` and `Union` assertions are the ones that change.

### 11.3 Repro stages (`apitest/carrier_chamfer_contact_test.go`)

| ID | Pair | Today (exact-chamfer §1) | Expectation |
|---|---|---|---|
| R1 | T with its wide slope corner chamfered `0.5` (Q2-N) vs S | `Suspect`, proven overlap | as C1a |
| R2 | T vs S with its slope lip chamfered `0.5` (D1) | `Suspect`, `unsupported_pair_contact` | as C1a; S's lip-top feet hold bound 0 |
| R3 | both chamfered | `Suspect` | as C1a |

### 11.4 Existing tests this change flips

- Tests asserting a zero bound or exact float coordinates on a section-chamfer foot of a tilted line walk (Q2-N), and
  exact-chamfer §7's R3/C3 expectations (D1). List each in the PR with old and new values.
- Tests asserting `BooleanUnsupportedContact`, `TouchCoplanar` or `CodeGapShiftApplied` on a pair whose faces become
  exact. Move each retry assertion onto a fixture without carriers (C3's rotated copy), never delete it
  (coplanar-contact §11.3 rule).

### 11.5 Consumer acceptance (not committed here)

In the gallery's dovetail package, with `go.mod` replacing decad by the merged commit: for all six parameter sets,
`Document.Verify` reads `Sound` with no diagnostic; `Intersect` returns `BooleanEmpty`; `Union` returns one lump with
volume within its bound of the sum, bound `≤ 1e-9`, no `CodeGapShiftApplied` on either half or the union; each half's
volume equals its unchamfered volume minus `2·ChamferCut()·Length` within its bound. Record stage wall times.

## 12. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| A class E/F/A read of `Verts` missed (§5.3) | the rounded point is classified while the snapped one is claimed: unsound | `snappoison` equality rule; exhaustive grep list per PR; fresh Opus review |
| Junction vertex moved to the corner (§8.1) | vertex coordinates and bounds change on every corner-closed prism junction | listed flips in Task 10; the old bound did not cover the corner |
| A stale carrier on a moved face | snap onto a wrong plane: unsound | `CarrierMesh` binding fails closed; only §4.3 producers mint; U4 |
| D1 and Q2-N change chamfer denotations | conflict with the exact-chamfer implementer; reviewers may read it as snapping | §13 Q1–Q2 decisions; Task 11 waits for exact-chamfer Task 5 |
| Q2-N test churn (prism and revolve chamfers) | large Task 10 diff | only walks whose float foot misses its support change |
| Snapped operand folds or self-intersects | wrong classification | no partly moved facet (§5.1 step 4); §5.2 checks with fallback |
| Union face merge in flight edits `facemerge/merge.go` | rebase conflict | one-line rule in Task 3 |
| Replay declines snapped booleans | a later re-tessellation or certificate refuses | §13 Q3 |
| Pre-existing: `retainedPlaneOf` slabs treat float tags as exact planes, which miss corners by ~4e-16 | a slab half-width may be short by that much | out of scope; flag for a separate fix |

## 13. Open questions

- **Q1. Chamfer denotation for R1 (D1).** D1 (designed): rational chamfer plane, feet pinned by three carriers. Gains:
  Rule A alone decides every flush contact on chamfered planar bodies. Costs: the denoted setback
  differs from the requested one by about one ulp of the coordinates; amends an in-flight design. D2: keep exact-chamfer's
  ideal (irrational) chamfer. Gains: the exact requested setback. Costs: the socket feet are not pinned, so the dovetail
  needs **Rule B**: snap a vertex onto the line or plane its carriers share keeping `β`; admit a coplanar facet pair whose
  corners all lie on the shared carrier when the gate certifies every near facet pair `(a, b)` by a carrier plane `Π`
  (each corner on `Π`, or off it by more than its bound on its own side; at every corner on `Π` the held star lies in its
  side's closed half-space with its in-plane facets facing the other side); extend interference §6.1 Step 4 with swept
  sets, each confined to its half-space; add in-plane rim bounds for clip points. It needs a second soundness proof. Recommend D1: smaller and Fusion-like (B-rep surfaces as exact planes, vertices derived).
- **Q2. Section chamfer (Q2-N vs Q2-A, Q2-B, Q2-C).** Gains and costs in §8.1. Recommend Q2-N.
- **Q3. Replay parity.** Decline (designed): the bounded replay does not snap, so it refuses where live snapping was
  needed. Gains: no replay work. Costs: a re-tessellation or certificate replay of such a pair refuses. Port the snap to
  the replay: gains parity, costs replay work. Decide after Task 9 shows whether any test or consumer replays these pairs.
- **Q4. Fillet feet on tilted walls.** A section fillet's foot rounds off the wall the same way. Decide between Q2-N's
  rule for line walks next to fillet arcs (small, same pattern) and leaving fillets as they are, after Task 10.
- **Q5. Carriers through placement.** Drop (designed). An exact image under translations and exact-rotation copies
  would keep flush contact decidable after `Transform`/`Copy`. Decide when a consumer needs it.

## 14. Size

| Part | Tasks |
|---|---|
| carrier record and transfer | 2–3 |
| snap core and float-read audit | 4–6 |
| simplification, wiring, contact tests | 7–9 |
| section chamfer (Q2-N) and junction corner | 10 |
| R1 chamfer (D1) | 11 |
| repro tests, docs, acceptance | 12–14 |

Task 10 can run beside Tasks 4–9. The largest unknown is the float-read audit (Tasks 5–6). The cheapest sound subset is
Tasks 2–9: it makes flush contact exact for every planar body whose corners are pinned (fixtures K, G) with no
denotation change; the dovetail also needs Tasks 10 and 11. Rule B instead of D1 adds a second classifier and its
soundness proof (§13 Q1).

## 15. Tasks

Each task merges on its own with `go test ./...`, `go vet ./...` and `golangci-lint run` green (CI runs the full suite;
run the focused command locally). A Sonnet agent implements each task; a fresh Opus agent reviews it against §6 before
merge. Every production PR carries the Module boundary gate evidence (`agent-docs/module-boundary.md`): owning section
of this doc; input class "planar faces with exact carriers whose vertices are pinned by three carriers, in mesh boolean
operands" (Tasks 2–9), "section chamfer of a line walk whose float foot misses its support" (Task 10), or "R1 chamfer on
the exact path" (Task 11); the focused cases; no new root adapter.

### Task 1: Land the design doc

- **Files:** `agent-docs/tiny-bound-contact-design.md`, `agent-docs/layout/design-docs.md`.
- **What:** Commit this doc and its layout row.
- **Depends on:** none.
- **Done when:** `go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1` passes.

### Task 2: Carrier type and exact plane helpers

- **Files:** `internal/topo/plane_carrier.go` (new), `internal/topo/face.go`, `internal/planecarrier/planecarrier.go`
  (new), `internal/planecarrier/planecarrier_test.go` (new), `agent-docs/layout/internal-contact.md`.
- **What:** §4.1's type and `Face.Carrier`. `planecarrier.Through(a, b, c proof.Xpt) (topo.PlaneCarrier, bool)`,
  `Contains(p topo.PlaneCarrier, x proof.Xpt) bool`, `Equal(p, q topo.PlaneCarrier) bool`,
  `Meet(planes []topo.PlaneCarrier) (proof.Xpt, bool)`, `OfCapRecord(f *topo.Face) (topo.PlaneCarrier, bool)`,
  `OfLateral(frame r3.Frame, xform r3.Transform, z0, z1 float64, start, end sectionrecord.Point2) (topo.PlaneCarrier, bool)`.
  No producer or reader yet.
- **Depends on:** 1.
- **Pattern refs:** `capRecordNormal` in `internal/facetedrecord/place.go` (exact CapRecord reading); `proof.Xcross`.
- **Tests:** U1.
- **Done when:** `go test ./internal/planecarrier/ ./internal/topo/ -count=1` passes.

### Task 3: Carrier producers, reader and transfer

- **Files:** `internal/prismrecord/build.go`, `internal/prismrecord/lateral_carrier_test.go` (new),
  `internal/planecarrier/reader.go` (new), `internal/planecarrier/reader_test.go` (new),
  `internal/facetedrecord/group.go`, `internal/facetedrecord/payload.go`, `internal/facetedrecord/build.go`,
  `internal/facetedrecord/assemble.go`, `internal/booleaneval/eval_mesh.go`, `internal/booleaneval/eval.go`,
  `internal/booleaneval/shift_certificate_snapshot.go`, `internal/booleaneval/carrier_transfer_test.go` (new),
  `internal/boolshift/moved_tag.go`, `internal/facemerge/merge.go`, `internal/facemerge/merge_test.go`.
- **What:** §4.2 reader as `planecarrier.Of` and the §Assumptions predicate `planecarrier.PlanarTrue`; §4.3 lateral
  producer; §4.4 group field (set from `planecarrier.Of`), `CarrierMesh` binding, assembly, minting by the generic
  boolean, snapshot comparison, drop on moved pieces, facemerge rule. `planecarrier` imports `topo`, `proof`, `r3`,
  `sectionrecord` and `momentinput` only.
- **Depends on:** 2.
- **Pattern refs:** `CapRecord` handling in `facetedrecord/source_tag.go`, `assemble.go` and `boolshift/moved_tag.go`;
  `Payload.SurfaceBounds` for the binding.
- **Tests:** U2, U3, U4; a facemerge case where equal carriers survive a merge and unequal ones drop.
- **Done when:** `go test ./internal/planecarrier/ ./internal/prismrecord/ ./internal/facetedrecord/ ./internal/booleaneval/ ./internal/facemerge/ -run 'Carrier' -count=1` passes.

### Task 4: Snap core

- **Files:** `internal/meshbool/carrier_snap.go` (new), `internal/meshbool/carrier_snap_test.go` (new),
  `internal/meshbool/boolean_mesh.go` (`Snapped []bool` field and doc comment),
  `internal/meshbool/snap_poison_on.go` (new, `//go:build snappoison`), `internal/meshbool/snap_poison_off.go` (new,
  `//go:build !snappoison`).
- **What:** §5.1 and §5.2 as `SnapToCarriers(m *topo.Mesh) (int, SnapCharge, error)` with
  `type SnapCharge struct{ Vol, Area float64 }` (§5.4), unwired. The poison files define one unexported constant read by
  step 5.
- **Depends on:** 3.
- **Pattern refs:** `SetExactFacets` (`internal/meshbool/face_exact.go`); `HeldFolds` pair test
  (`held_embedding.go`); box tree in `held_embedding_tree.go`; `setDepthBlocks`, `NewParityMesh`.
- **Tests:** U5, U5b, U6.
- **Done when:** `go test ./internal/meshbool/ -run 'TestSnap' -count=1` passes.

### Task 5: Snapped vertices in operand prep, contact and the gate

- **Files:** `internal/meshbool/operand_prep.go`, `internal/meshbool/boolean_mesh.go`, `internal/meshbool/boolean.go`,
  `internal/meshbool/boolean_pair_cuts.go`, `internal/meshbool/pair_cuts.go`, `internal/meshbool/boolean_parallel.go`,
  `internal/meshbool/contact_memo.go`, `internal/meshbool/depth_witness.go`, `internal/meshbool/depth_blocks.go`,
  `internal/meshbool/depth_shared.go`, `internal/meshbool/tangency_gate.go`, `internal/meshbool/plane_slab.go`,
  `internal/meshbool/bound_spread.go`, `internal/booleaneval/eval.go`, `internal/booleaneval/eval_mesh.go`,
  `internal/booleaneval/overlap_clip.go`, `internal/booleaneval/view.go`, `internal/meshbool/rational_box_test.go` (new).
- **What:** §5.3 for these files. No behavior change while nothing snaps.
- **Depends on:** 4.
- **Tests:** U7 for `Intersect` (rational boxes snapped by a test helper in `rational_box_test.go`).
- **Done when:** `go test ./internal/meshbool/ -count=1` and
  `go test -tags snappoison ./internal/meshbool/ -run 'TestRationalBox|TestSnap' -count=1` pass, and the PR classifies
  every audit hit in these files.

### Task 6: Snapped vertices in parity, cutting and the stitch

- **Files:** `internal/meshbool/parity_index.go`, `internal/meshbool/parity_mesh.go`,
  `internal/meshbool/parity_oblique.go`, `internal/meshbool/boolean_cut.go`,
  `internal/meshbool/boolean_cut_projection.go`, `internal/meshbool/cut_vertex_index.go`,
  `internal/meshbool/mesh_conform.go`, `internal/meshbool/mesh_stitch.go`, `internal/meshbool/held_embedding.go`,
  `internal/meshbool/held_embedding_tree.go`, `internal/meshbool/held_embedding_diagnostics.go`,
  `internal/meshbool/rational_box_test.go`.
- **What:** §5.3 for these files (a file whose hits all read other structs is listed as unchanged in the PR); the stitch
  takes a snapped vertex's exact point and claim.
- **Depends on:** 5.
- **Tests:** U7 complete (`Union`, `Cut`, `Intersect`).
- **Done when:** `go test ./internal/meshbool/ -count=1` and `go test -tags snappoison ./internal/meshbool/ -count=1`
  pass.

### Task 7: Planar simplification with carrier corners

- **Files:** `internal/meshbool/planar_simplify.go`, `internal/meshbool/mesh_stitch.go`,
  `internal/meshbool/brake_stitch_replay.go`, `internal/meshbool/planar_simplify_test.go`,
  `internal/meshbool/replay_simplify_test.go`, `internal/booleaneval/eval.go`.
- **What:** §7, live and replay; booleaneval passes `StitchOptions.Carriers` and the replay `carriers` argument.
- **Depends on:** 3.
- **Tests:** U8.
- **Done when:** `go test ./internal/meshbool/ -run 'TestSimplifyPlanar|TestReplaySimplify' -count=1` passes.

### Task 8: Benchmark baseline

- **Files:** `internal/booleaneval/carrier_bench_test.go` (new).
- **What:** `BenchmarkCarrierFlush` (K vs G, `Intersect` through `Evaluate`). It accepts today's refusal as a result
  and records it; it lands before Task 9 so Task 9's PR compares.
- **Depends on:** 3.
- **Done when:** `go test ./internal/booleaneval/ -run '^$' -bench 'CarrierFlush' -benchtime 5x -count=1` runs.

### Task 9: Turn on the snap

- **Files:** `internal/booleaneval/eval.go`, `apitest/carrier_contact_test.go` (new), `.github/test-shards-apitest.txt`,
  every test flipped per §11.4.
- **What:** call `SnapToCarriers` on both operands before `SetExactFacets` in the non-replay, unshifted, non-fixed
  branch when `capFace == nil`; add each operand's `SnapCharge.Vol` to its `OperandSymDiff` term and `SnapCharge.Area`
  to its area slack before `booleanProofBounds`; §5.4 occupancy guard; replay unchanged (declines).
- **Depends on:** 5, 6, 7, 8.
- **Tests:** C1–C3, P1; §11.4 flips.
- **Done when:** `go test ./apitest/ -run 'TestCarrierContact' -count=1`,
  `go test -tags snappoison ./apitest/ ./internal/booleaneval/ -run 'TestCarrierContact|Carrier' -count=1` and
  `go test . ./apitest/ -run '^TestCI' -count=1` pass, and the PR carries `benchstat` of `BenchmarkCarrierFlush` and
  `BenchmarkMeshBooleanCrossing` on Task 9's parent and on Task 9 (crossing within 5%).

### Task 10: Section chamfer feet on their supports (Q2-N)

- **Files:** `internal/offset2d/blend.go`, `internal/offset2d/profile_offset.go`,
  `internal/offset2d/blend_support_test.go` (new), `internal/boundarywalk/vertex_bounds.go`,
  `internal/boundarywalk/junction_replay.go`, `internal/boundarywalk/line_corner.go` (new),
  `internal/boundarywalk/vertex_bounds_test.go`, `internal/momentinput/junctions.go`,
  `apitest/section_chamfer_support_test.go` (new), `.github/test-shards-apitest.txt`, `agent-docs/modify-design.md`
  (§7: the rewrite paragraph), tests flipped per §11.4.
- **What:** §8.1, including the junction vertex at the corner and `boundarywalk.LineCorner`.
- **Depends on:** 3.
- **Pattern refs:** `OriginalSegment`; carrier-record §7.2 for walk-to-segment mapping (`SideWalk.Segs`).
- **Tests:** U9, U11; an apitest case: the chamfered slope prism's wall face carries its original plane, its volume
  equals the crossing polygon's exact area times the length within its bound, and a `Copy`, a `Transform` and a later
  `Fillet` of another lateral edge of it all succeed; a revolve with a chamfered slope meridian corner keeps that wall
  on its cone (independent case for the module boundary gate).
- **Done when:** `go test ./internal/offset2d/ ./internal/boundarywalk/ ./internal/momentinput/ -count=1` and
  `go test ./apitest/ . -run 'Chamfer|SectionChamferSupport' -count=1` pass.

### Task 11: Rational R1 chamfer feet (D1)

- **Files:** `internal/retainedblend/line_exact.go`, `internal/retainedblend/line_rewrite.go`,
  `internal/retainedblend/route.go` (its `facetedrecord.Build` call publishes the result's `Groups`; set `CarrierMesh`
  there), `internal/retainedblend/line_exact_internal_test.go`, `retained_line_chamfer_exact_test.go`,
  `agent-docs/exact-chamfer-design.md` (§2 E1–E2, §4.2, §5 Claim X).
- **What:** §8.2.
- **Depends on:** 3; exact-chamfer §10's slab term and feet, which create `line_exact.go` and both tests.
- **Tests:** U10; exact-chamfer R1–R5 updated to the rational feet.
- **Done when:** `go test ./internal/retainedblend/ . -run 'Chamfer' -count=1` passes.

### Task 12: Repro-stage contact tests

- **Files:** `apitest/carrier_chamfer_contact_test.go` (new), `.github/test-shards-apitest.txt`.
- **What:** R1–R3 (§11.3).
- **Depends on:** 9, 10, 11.
- **Done when:** `go test ./apitest/ -run 'TestCarrierChamferContact' -count=1` and `go test . ./apitest/ -run '^TestCI' -count=1` pass.

### Task 13: Owning docs

- **Files:** `agent-docs/coplanar-contact-design.md` (§3.1, §4: snapped faces are exact faces), `agent-docs/interference-design.md`
  (§5.2 and §6.1 P1: the held operand is the snapped one), `agent-docs/faceted-vertex-bounds-design.md` (§2.1: snap
  reading), `agent-docs/faceted-source-retention-design.md` (§3.1 table: `Carrier` row), `agent-docs/evaluator-design.md`
  (§9 one sentence), `agent-docs/missing-features.md` (bounded contact without carriers; fillet feet).
- **What:** state current behavior with pointers to this doc; no restatement.
- **Depends on:** 12.
- **Done when:** `go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1` passes.

### Task 14: Consumer acceptance

- **Files:** none in this repository.
- **What:** §11.5 in the gallery; report every assertion, the stage wall times, and each half's face bounds.
- **Depends on:** 12.
- **Done when:** all six parameter sets meet §11.5.
