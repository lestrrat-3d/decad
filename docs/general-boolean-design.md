# General Boolean Design

The next reach of exact `Union`/`Cut`/`Intersect` past the co-directional
prism pairs `docs/prism-boolean-design.md` admits: the co-directional shapes
that still refuse (a boss on a plate, a counterbore, a mirrored half, a
pattern of holes, a rotated tooth), and the first non-co-directional class —
two prisms whose sweeps are exactly perpendicular and whose every contact is a
line or a circle (a cross-drilled hole, a cross slot, a keyway). Companion to
`docs/evaluator-design.md` §9 (the mesh path, unchanged and still the
fallback), `docs/prism-boolean-design.md` ("prism-boolean §N": the gates,
scene, cell classification and displacement bounds this design reuses),
`docs/stacked-prism-design.md` ("stacked §N") and
`docs/mirror-pattern-design.md` ("mirror §N"). Core §N is
`docs/api-design.md`; seam §N is `docs/sketch-seam-design.md`. Every row of §2
was measured with the probe under `.tmp/probe/` at `8cc06f90`.

## 1. Problem

Outside prism-boolean's admitted class a boolean tessellates both operands
(evaluator §9) and ends in one of three places, each measured in §2:

1. **A coplanar contact refuses.** Two operands sharing a plane with positive
   common area are `BooleanUnsupportedContact`. This is the dominant failure:
   a boss standing on a face, a counterbore over a hole, a flange on a shaft,
   a tooth placed by rotation, a mirrored half beside its source, a tool that
   is a multi-lump body — every one of them shares a cap or a wall with its
   partner.
2. **The result is `Faceted`.** A cross-drilled hole, a slot, a keyway, a
   boss rooted inside a plate, a tee of cylinders all build, but as
   `facetedPayload`: `Fillet`/`Chamfer`/`Shell` refuse permanently
   (modify-reach SX9), the three surveys read `Suspect`, the analytic STEP
   path is gone, and the published bound is 0.5–2 mm³ against 1e-12 mm³ for
   the same part drawn as an analytic cut.
3. **Chains stop early.** A faceted face's bound is the largest over its
   facets, so a plate's side wall carrying one hole's rim is coarse over its
   whole 60 mm, and a third cross hole 20 mm from any rim refuses on the
   chain-depth gate (§2 S11).

A fourth limit sits beside them: a revolve with a spherical face cannot be
tessellated at the pair's chord tolerance at all (§2 S8), so a sphere never
reaches a boolean.

**Success criterion.** Every §3 class A shape builds a `prismPayload` or
`stackedPrismPayload` with prism-boolean §7's bounds; every class B shape
builds the analytic `brepPayload` of §4, whose faces are planes and
cylinders, whose edges are lines, circles and arcs, whose measurements carry
closed-form bounds, and which every consumer in §4.5 reads. `facetedPayload`
stays the result of every pair outside §3, unchanged.

## 2. Measured current behaviour

| Row | Scene | Path | Result |
|---|---|---|---|
| S1 | 40×20×20 box `Cut` by Ø6 cylinder along y through its middle | mesh | 7 `Faceted` faces, 15434.9 mm³ ± 1.12 (closed form 15434.51), `Fillet`: no analytic edge, wall `unavailable` |
| S2 | Ø30×20 cylinder `Union` Ø8 radial boss | mesh | 5 `Faceted` faces, bound 1.79 mm³ |
| S3 | full revolve (cylinder along x) `Union` box crossing it | mesh | 13 `Faceted` faces, bound 0.21 mm³ |
| S3b | the same revolve `Union` a coaxial disk sharing its end plane (flange) | mesh | `BooleanUnsupportedContact` |
| S4a | plate `Cut` through hole, both on XY | analytic | 7 faces, bound 8e-13 mm³ |
| S4b | then `Cut` Ø10 counterbore on the offset plane z = 7 over the hole | mesh | `BooleanUnsupportedContact` |
| S4c/d | counterbore first (blind, analytic), then the through hole inside it | mesh | `BooleanUnsupportedContact` |
| S5 | plate `Cut` by six `PlacedCopy`-translated cylinders | analytic | six results, bound 5e-10 mm³, `sectionDelta > 0`: `Fillet` refuses, wall `undecided` |
| S5b | the six holes each drawn in a sketch | analytic | bound 2e-12 mm³, `Fillet` OK |
| S6a | filleted box `Cut` by a cross cylinder | mesh | 11 `Faceted` faces, bound 1.71 mm³ |
| S6b | filleted box `Cut` by a same-plane cylinder | analytic | 11 faces, bound 3e-12 mm³ |
| S7 | two Ø10 cylinders crossing at right angles, `Union` and `Cut` | mesh | `Faceted`, bound 1.2–1.5 mm³ |
| S8 | sphere r = 10 `Cut` by a box, or by a Ø6 cylinder | mesh | `ErrUnsupported`: "more than 65536 facets in one revolve mesh" at the pair tolerance |
| S9 | box `Intersect` the same box rotated 45° about z | mesh | `BooleanUnsupportedContact` (coplanar caps); rotated 90°: "come within the chord tolerance without provably interpenetrating" |
| S10 | box `Cut` by a cylinder on a plane tilted 30° | mesh | 7 `Faceted` faces, bound 0.69 mm³ |
| S11 | 60×20×10 bar, three cross holes at x = 10, 30, 50 | mesh | holes 1–2 build (bounds 0.86, 1.72 mm³); hole 3 refuses: target bound 0.0034 mm where the pair meets, pair tolerance 0.0017 mm. Both 60 mm side walls carry that bound over their whole face; the box's other faces carry 0 |
| S12 | two disjoint boxes `Union` | mesh | `Faceted`, 2 lumps, `Exact` 1000 mm³, wall `unavailable` |
| S12c | two boxes touching at one vertical edge | mesh | `BooleanUnsupportedContact` |
| W1–W4 | two boxes sharing a wall (whole, partial, longer, 1 mm overlap with collinear walls) | analytic | RB1 `ErrUnsupported`: the arrangement reports an invalid region |
| W5 | two boxes overlapping with no collinear wall | analytic | 800 mm³ |
| W6 | box `Cut` by a box sharing its wall | mesh | `BooleanUnsupportedContact` |
| B1 | box `Cut` by a 10×10 square cross slot | mesh | 10 `Faceted` faces, 14000 mm³ ± 9e-12 (exact integral) |
| B2 | Ø20×40 rod `Cut` by a 4 mm keyway box | mesh | 7 `Faceted` faces, bound 2.0 mm³ |
| B3 | plate `Union` Ø10 boss standing on its top (z 10..25) | mesh | `BooleanUnsupportedContact` (held facets within tolerance) |
| B4 | plate `Union` the boss rooted inside (z 5..25) | mesh | 8 `Faceted` faces, bound 0.51 mm³, no analytic edge |
| B5 | plate `Union` a side pin whose end cap lies in the plate's face | mesh | `BooleanUnsupportedContact` |
| P1 | hub ∪ six teeth placed by `RotationAround` | analytic → mesh | tooth 2 `BooleanUnsupportedContact` (prism-boolean §3.4 reroute) |
| P2 | plate `Cut` by a four-lump faceted tool | mesh | `BooleanUnsupportedContact` |
| M2–M5, T1 | reflected operands (mirror §2) | mesh | coplanar refusal, or a `Faceted` result |

## 3. Admitted classes, in priority order

Priority follows how many ordinary machined or printed parts a class unlocks,
measured by which §2 rows it clears. Class A keeps the result a prism or a
stacked prism and extends prism-boolean's own machinery; class B is the
first new payload. Nothing here changes the mesh path, and a pair no class
admits takes it unchanged.

### Class A — co-directional prisms

| # | Shape | Clears | Result |
|---|---|---|---|
| A1 | `Union` with unequal sweep intervals: a boss standing on a plate, rooted inside it, a flange on a circle-prism shaft, a stack of blocks | B3, B4, S3b with the shaft drawn as a prism | `stackedPrismPayload`: a slab per distinct level, interface exposure by the clean-nesting match |
| A2 | a blind tool over an existing hole, or a through tool inside a blind one (counterbore, counterbored through hole) | S4b, S4d | stacked §7 stage 2 owns it; this design adds no mechanism and lists it for priority |
| A3 | two operands sharing a wall: collinear coincident line carriers (box beside box, a mirrored half beside its source) | W1–W4, W6, S12c, M2 | waits on `sketch` (§7); the mirror join (mirror §5) covers the symmetric case without it |
| A4 | a reflected operand, or two operands whose relative map is a reflection | M3, M4, M5, T1, T2 | prism-boolean's own paths over a reflected re-expression (§3.2) |
| A5 | a multi-region operand or result: a prism-group tool (N holes in one arrangement), a `Union` of disjoint footprints | S5 (through mirror §6), S12, P2 | one-slab multi-region `stackedPrismPayload` (mirror §6.3) |
| A6 | a split boundary under a non-identity re-expression, a prior section displacement or a walk charge | S9 (45° in plane), rotated and translated overlaps; not P1 (A6 below) | `prismPayload` carrying a certified crossing-sensitivity charge |

#### A1 — stacked union

Admission is prism-boolean's G1–G4 and G6 (hole-free operands until A5), G3
in either arm, and G5 read for `Union` as "the intervals overlap or touch"
(`z0_b' <= z1_a && z0_a <= z1_b'`, exact over `big.Rat`). The distinct levels
`{z0_a, z1_a, z0_b', z1_b'}` sorted exactly cut the union into at most three
slabs. In a slab both operands reach, the region is `Union`'s select-all
merge (prism-boolean §4.2, with its enclosed-void check) over the two
records, with its §6 audit and §7 displacement; where the clean-nesting
match below proves one region carries the other's outer whole as a hole,
that merge is the containing region, and
the slab keeps the containing operand's record verbatim. In a slab one
operand reaches, the region is that operand's own record verbatim; under a
non-identity re-expression, B's region is arranged alone in a private scene
of re-expressed entities and recorded from the one cell `sketch` returns. Each interface is where the two adjacent slabs' regions
differ; its exposed records are decided by the clean-nesting structural match
prism-boolean §4.2 already runs: the smaller region's outer must reproduce
whole as a hole of the larger region's cell, in which case the exposed
record is the larger region with that hole, and the smaller region is the
material on both sides. A boss whose footprint crosses the plate's outline
at the interface (the match finds `Partial` edges) is an unresolved topology
in prism-boolean §4.4's sense and takes the mesh path with no error: the
exposed floor would need the per-cell classification with a split boundary,
which A6 admits later. The mesh path builds a rooted crossing boss as a
`Faceted` body and refuses one standing on the plate's top as a coplanar
contact, as it did before this class. Stacked §2.2's I5 reads, for
a union-built stack, "every slab's outer loop equals the previous slab's or
is proven nested by the clean-nesting match", I6/I7 generalise to the
exposure records the match derives, and the implementation PR changes that
document on the same branch. The inner level a shifted operand contributes
rounds once and is charged by `rationalFloatError` into that interface's
`z0Delta`/`z1Delta`, exactly as stacked's blind cut charges its floor. A
stacked operand in a stacked union splits at every level of both.

A stacked union whose only interface is a boss on a plate has `Volume` the
exact rational sum `A_plate·h_plate + A_boss·h_boss`, `Exact` where both are.

#### A4 — the reflected re-expression

G2 is replaced by "the relative map's determinant decides nothing": operand
B's record is re-expressed into A's frame through the composed relative map
exactly as prism-boolean §4.1 does, and when that map is a reflection
(`Transform.IsReflection` on the composed map, read once) the mapped record
is re-wound before any entity is created:

- every loop's segment order is reversed;
- each `LineSeg` swaps `Start`/`End`; each `ArcSeg` becomes
  `{m(Center), m(End), m(Start)}` (mirror §5.2's table: the reflection turns
  the arc clockwise and the reversal turns it back); both keep their range,
  whose order still names the walk's sense;
- each `CircleSeg` keeps `CCW` and its range, with only its centre mapped:
  the reflection reverses the circle's winding and the reversal reverses it
  back, and `walkOf` refuses a `CCW` that contradicts the range order;
- a `LineSeg` recorded over a narrowed range enters as its walked endpoints
  (prism-boolean §7 `δ_walk`), since `1 − t` is not an exact float operation
  for a general `t`; a trimmed circular carrier is refused as §4.1 already
  refuses it;
- `prismcells.Origin.AuthoredReversed` is computed on the re-wound record, so
  the crossing sub-case's flag comparison reads the authored sense of the
  record the scene actually holds.

The shared-axis arm (`xformA == xformB`) admits two operands that are BOTH
reflected the same way with no re-expression at all (T1). The G3 reading of
the world normal uses `ApplyDir`, which a reflection maps correctly, so the
co-directional test is unchanged. `δ_reexpress` is the same single
rounding per coordinate; a reflection adds no term. Interference's read-only
twin (`evaluateAnalyticIntersect`) and the overlap-area reading share the
same gate and scene, so a reflected pair the analytic `Intersect` resolves is
measured instead of reaching the mesh path's `unsupported_pair_contact`.
A one-sided reflection is a nonidentity re-expression, so a reflected pair
whose outlines cross takes A6's crossing charge. M3 and T2 still refuse:
their outlines share collinear walls, which A3 waits on (§9).

#### A5 — multi-region operands and results

A prism-group tool (mirror §6.3) enters the scene as `Count` operand-B
loops, each tagged with its region index. `Cut`'s clean-nesting match then
requires the target's outer whole plus exactly `Count` new holes, each
reproducing one region's outer whole; any `Partial` edge takes the crossing
sub-case with the group's regions classified as B-material each. A
`Union` whose select-all survivors chain into several closed loops that are
pairwise disjoint (the structural disjointness read of mirror §6.3, on the
survivors) becomes a one-slab multi-region stacked payload instead of
"unresolved → mesh path" (prism-boolean §4.4). Prism-boolean §4.5's
overlap-area reading is unchanged.

Select-all keeps every bounded cell, which is the union only when no cell is
material of neither operand. Hole-free operands can still enclose such a
cell between them (two C shapes facing each other, or four bars in a ring),
so A5's `Union`, like every select-all merge (prism-boolean §4.2, A1's
slabs), first requires every cell to carry at least one boundary edge on
its operand's material side. A cell with none is an enclosed void, and the
pair takes the mesh path. A group operand needs
the partner's interval exactly (prism-boolean §3.2's `Union` row), and a
result whose survivors close into one loop is a prism. A group scene whose
cuts carry a displaced operand's displacement (a placed group, for one)
takes A6's crossing charge on both the `Cut` crossing sub-case and the
`Union` merge, and falls back to the mesh path where A6 does.

#### A6 — the crossing-sensitivity charge

An input displacement moves every cut the arrangement makes: two recorded
carriers crossing at `O` at an angle `θ`, whose denoted carriers sit within
`δ1` and `δ2` of them, meet at a point `P*` with
`|P* − O| ≤ (δ1 + δ2)/sin θ_low + min(δ1, δ2)`. The proof: take `Q1`, `Q2`
on the recorded carriers nearest `P*`, so `|Q1 − Q2| ≤ δ1 + δ2`; inside a
ball around `O` where every tangent of one carrier makes an angle of at
least `θ_low` (mod π) with every tangent of the other, `Q1 − O` and
`Q2 − O` are sums of each carrier's own tangents, so
`|Q1 − Q2| ≥ max(|Q1 − O|, |Q2 − O|)·sin θ_low`. This class charges that
distance instead of prism-boolean §3.4 rerouting the pair
(`prismcells.CrossingCharge`, `internal/prismcells/crossing.go`):

- a cut is an arranged vertex (an edge's walk end in its `Polyline`, matched
  by exact equality) where some incident edge's parameter is one the
  arrangement computed (a `Partial` edge's non-natural end, or any end of a
  `Partial` circle). Every pair of distinct entities incident at a cut is
  charged, whichever face walks them, so a pair adjacent only through the
  unbounded face, which no returned cell walks, is charged too. A vertex
  where only recorded vertices meet is not a cut (prism-boolean §4.4's
  known limit). A touch from outside is not arranged as a vertex at all:
  `sketch` returns the two outlines as separate cells, and every point a
  displacement moves across either boundary there lies within that
  operand's own displacement of the other's recorded wall, inside the
  section displacement's tube;
- each side's displacement is its operand's incoming term: `δ_A + δ_walkA`
  for A, `δ_B + δ_walkB + δ_reexpress` for B. Both zero charges nothing;
- `sin θ_low` is a certified lower bound on `|sin θ|` between any tangent of
  one recorded carrier and any tangent of the other at points within `ρ` of
  a junction point held exactly: a line side's exact rational point at its
  recorded parameter (within its `δ_cut` of `O`), or a circular walk's
  endpoint with its walk-end bound added. It is exact rational arithmetic
  with outward-rounded square roots: `|d1 × d2|/(|d1||d2|)` for two lines,
  `(|d·(p − C)|/|d| − ρ)/R` for a line and a circle (a circle's tangent is
  perpendicular to its radius), `(|a1 × a2| − ρ(|a1| + |a2|) − ρ²)/(R1 R2)`
  for two circles, and zero for a circle read over a ball wider than half
  its radius;
- `ρ` starts at the junction point's own error, then widens to hold twice
  the distance the first bound charges; the second bound is accepted only
  when the distance it charges fits that ball, and otherwise the crossing
  has no charge;
- the largest charge over every cut of every returned cell is `crossing`,
  and the section displacement becomes
  `up(max(δ_A + δ_walkA, δ_B + δ_walkB + δ_reexpress, crossing) + δ_cut)`;
- a cut with `sin θ_low` not above the dimensionless noise floor
  `ε = 1e-9` of verification §4 (`sectionaudit.ContactEps`) has no charge,
  nor does a bound that does not settle or an edge that does not record.
  Neither check admits anything. `sketch` itself declines to certify a
  line/line cut much shallower than `sin θ ≈ 1e-8` (`TExact = false`), so
  the floor is reached through tangent junctions, whose bound is zero;
- a pair whose cuts carry an input displacement never refuses on the
  analytic path: a crossing with no charge, and any merge, audit or
  recording failure after the charge (RB1–RB9), sends it to the mesh path
  with no error (`prismAmplifiedFallback`), as prism-boolean §4.4 does for
  every topology it leaves unresolved. Cancellation still propagates, and a
  pair that brings no displacement keeps its §9 refusals.

With A6 a rotated tooth whose root sits inside the hub builds analytically
with a bound of a few ulps times `1/sin θ`, `Approximate`; `Fillet` on that
result refuses (prism-boolean §13's displaced-receiver rule), as it does for
every cut-bearing merge. P1's own tooth does not build. Its root arc lies on
the hub's circle, so where the arc ends it meets the hub circle tangentially
(a coincident carrier, which `sketch` merges), and that junction's bound is
zero: the pair falls back to the mesh path. A chain of teeth also stops at the
second tooth on prism-boolean §4.1's trimmed-circular refusal, since the
first union trims the hub circle.

### Class B — perpendicular prism pairs with planar contacts

Both operands are prisms (or stacked prisms, or class-B results: §4), their
sweep directions are exactly perpendicular, and every face pair that can
meet is a plane against a plane, or a cylinder against a plane that is
parallel or perpendicular to the cylinder's axis. The intersection curves are
then lines and circles only. The result is the `brepPayload` of §4.

#### B.1 Entry gate (reject-only, silent fallback)

| # | Condition | Why exact |
|---|---|---|
| B1 | Both operands expose the face view of §4.1: `prismPayload`, `stackedPrismPayload`, or `brepPayload`. | structural |
| B2 | Every recorded segment of both is `LineSeg`/`CircleSeg`/`ArcSeg` (prism-boolean G4's reason). | `TExact` is whole-scene |
| B3 | The composed world normals are exactly perpendicular: `N_A · N_B == 0.0` over the stored `r3.Vec` floats (`ApplyDir` on each frame's `N()`). | a decision over stored floats, as G3 is; a near-perpendicular pair takes the mesh path |
| B4 | The relative frame map is a signed axis permutation: every dot product among `{U_A, V_A, N_A}` and `{U_B, V_B, N_B}` (world, through `ApplyDir`) is exactly `0`, `1` or `−1`. | keeps every trace line exact (§5); the datum planes XY/XZ/YZ and their offsets meet it; a general pair waits on A6's charge (§8) |
| B5 | Both operands carry `sectionDelta == 0`. | the traces and chords of §5 inherit no displacement to amplify |
| B6 | Every segment of each operand is parallel or perpendicular to the OTHER operand's normal, read in its own frame: with `n' = (N_other · U, N_other · V)` (exact by B4: a signed unit axis or zero), a `LineSeg` has `d · n' == 0` or `d × n' == 0` exactly over the recorded endpoints; a `CircleSeg`/`ArcSeg` has no direction and passes. | a line wall neither parallel nor perpendicular to the other sweep meets a cylinder in an ellipse (class C) |
| B7 | No curved wall of one operand comes within reach of a curved wall of the other: for every pair of circular segments across the operands, the two walls' bounding boxes (prism-extent, outward-rounded) are separated along some axis by an exact comparison. | cylinder × cylinder is a quartic; refused unless provably apart, which may refuse a valid pair |
| B8 | No cap or wall of one operand is coplanar with a face of the other (same plane, exact: the two planes' recorded normals bit-identical and their origins' difference exactly along it), unless the pair is also co-directional, which class A owns. | a coplanar contact across a perpendicular pair is an edge-on contact the per-face scenes do not classify |

Every miss takes the mesh path with no error. The arrangement cap
(`prismMaxArrangementSegments`) and cancellation are prism-boolean §10's,
charged per scene.

#### B.2 What is refused, and stays refused

| Shape | Why |
|---|---|
| cylinder × cylinder, non-coaxial (S2, S7, a boss on a round hub) | a quartic space curve; no exact record |
| cone, sphere, torus against anything (S8) | no line/circle curve class; S8 also refuses at the mesh path's facet cap |
| a plane oblique to a cylinder's axis (S10, a chamfered hole mouth cut sideways) | an ellipse: class C, after `sketch` certifies elliptical cuts and `Ellipse3` enters the curve vocabulary |
| sweeps neither parallel nor perpendicular (S10's 30° plane against a box cap) | its contacts are lines, but B4's exact traces do not exist; waits on A6 |
| free-form segments | seam §1's whole-scene gate |
| a sheet operand | core §13, unchanged |

## 4. The `brepPayload`

### 4.1 Record

```go
// brepFace is one analytically trimmed face.
type brepFace struct {
    frame   r3.Frame       // the face's plane (planar) or the sweep frame of its wall (swept)
    // exactly one of:
    region  *ProfileRecord // planar: one outer loop and holes in frame coordinates, material left of each walk
    outward bool           // planar: true when the outward normal is frame.N()
    wall    CurveSegment   // swept: a LineSeg/CircleSeg/ArcSeg in frame coordinates, material on its left
    z0, z1  float64        // swept: the sweep interval along frame.N(); planar: the level, z0 == z1
    z0Delta, z1Delta float64
    delta   float64        // the face's own section displacement (prism-boolean §7 terms)
    role    string
}

type brepPayload struct {
    faces []brepFace
    xform r3.Transform
}
```

A prism is a `brepPayload` with two planar faces and one swept face per
segment; a stacked prism is one with its columns and planar patches. The
face view of §B1 is this record, built on demand from either payload and
never stored for them. A class-B result stores it. Every face's frame is
stated in the payload's unplaced coordinates and `xform` places the whole
body, as every payload does.

Every face frame shares the first face's origin bit for bit and carries each
of its axes, or that axis negated, bit for bit, keeping handedness. The map
between two face frames is then a signed permutation, which moves every
coordinate and level exactly, and §4.2–§4.4 read every face in the first
face's frame, the reference frame. The datum planes XY, XZ and YZ meet this. A
record whose frames do not is `ErrUnsupported`.

### 4.2 Topology and roles

Faces are the record's. Edges: each planar face's loop segments and each
swept wall's two rim curves (the wall segment at `z0` and `z1`) and two
side lines (a wall's ends at the junction with the next face). Every edge is
shared by exactly two faces by construction: a planar face's segment is one
rim of exactly one swept wall, or one line of exactly one other planar face
(two planar faces meeting along a line), and the build identifies them by
exact record identity in reference coordinates (same coordinates, same
level). A result whose edges do not pair is `ErrUnsupported`
(`BooleanEvaluatorFailure`): the build proves closure by counting, as the
mesh audit does. A circular edge that bounds no swept wall, and a rim shared
by two swept walls, are `ErrUnsupported` too. `Edge.IsConvex` reads the walked
boundary as evaluator §3 states. A rim reads the wall it runs along: a
circular wall by its own turn, a straight wall by the role of the loop it
belongs to. The planar face sharing the rim states that role: its own loop's
role when it walks the rim the wall's way, the other role when it walks it
the opposite way, as a stacked floor walks a reversed hole. A side line
between two walls is a junction, convex when the walk turns left there. Any
other line reads its planar face's loop role. `Lumps` and `Shells` are derived
from face adjacency (`sheetLumps`). Roles are fresh under the boolean's producer identity:
`face(k)` for planar faces, `wall(k)` for swept, both indexed by the result
record, and `capStart`/`capEnd` are not minted (core §9: the helper returns a
reference matching nothing). `Face.Origins` carry no operand provenance
(prism-boolean §11's decision, same reasoning).

### 4.3 Measurements

Every quantity is a sum over faces of a closed form in that face's own
record, read in reference coordinates. Volume is `(1/3)∮ p·n dA` and the
first moment along reference axis `i` is `½∮ x_i²·n_i dA`. A planar face at
level `z` with outward sign `s` (`+1` when `outward`) has its normal on one
reference axis `k` with sign `σ`; a swept face over height `h = z1 − z0` has
its `u` and `v` on two reference axes with signs `σ_u`, `σ_v`. Each segment
contributes its Green's-theorem terms about the frame origin in the forms
evaluator §4 accumulates: `g = ½∫(u dv − v du)`, `mu = ½∫u² dv`,
`mv = −½∫v² du`.

| Quantity | Planar face | Swept wall |
|---|---|---|
| `Volume` | `(1/3)·s·z·A`, `A` the region area (evaluator §4) | `(1/3)·2·h·g` |
| first moment | `½·s·σ·z²·A` along axis `k` | `σ_u·h·mu` and `σ_v·h·mv` along the `u` and `v` axes |
| `Area` | `A` | `L·h` (the prism wall's own reading) |
| `Bounds` | the face's own extremes (prism-extent's per-segment analysis over the face's prism view) | |

`Centroid` is each moment over the volume, lifted through the reference frame
and the placement as a prism's is. The volume and moment sums run in exact
rational intervals: a line's terms and a line region's area are exact
rationals, a circular term is the held float widened by its proven bound, and
each published value is the sum rounded once.

Each face's `delta`, `z0Delta`, `z1Delta` enter as prism-boolean §7 enters
them for a prism. A planar face's area carries `2·δ·p + n·π·δ²`; a wall's
length carries the length displacement, and its height both level
displacements. The volume the denoted body can differ by is each wall's band
`2·δ·L + π·δ²` over its height plus both level displacements, and each planar
face's level displacement times its area; that volume times the coordinate
envelope bounds each moment. `Exactness` is `exactnessOf` on the composed
bound: `Exact` for an all-planar result with every coordinate recorded and
every level a float. The divergence-theorem sums are signed and may cancel in
VALUE, never in BOUND: every term's enclosure width is carried, so a
cancelling pair of faces widens the bound rather than narrowing it.

### 4.4 Tessellation and the occupied-volume proof

A planar face chords its loops and triangulates them through the cap path
(`triangulate.go`, tessellation §5), sharing every arc's chord samples with
the swept wall that owns the arc (tessellation §3). A swept wall is the
prism wall path over its own segment and interval. The mesh closes by
construction because every edge is shared by exactly two faces (§4.2) and
both chord it from one sample set; `internal/tessellation.RequireClosedMesh`
proves it. The largest `delta` is reserved from the chord budget, as a
prism's is. Each planar face proves its loops' clearance, and every two walls
sweeping along one reference axis over overlapping intervals, sharing no
endpoint, prove theirs, so no two chorded walls cross where the analytic ones
do not. `Bound` per face is the wall's sagitta plus its level displacement,
or the planar patch's bounding-loop sagitta plus its level displacement,
composed with `delta`. Occupied volume: the analytic body and the chorded
body differ only inside the circular-segment slivers of each circular wall,
swept over its height — `Σ_walls h · E_wall`, tessellation §5's prism term
applied per wall — plus each wall's section band over its height where
`delta > 0`, each planar face's level displacement times its area, and
`sweptVolumeAllow` over the store maximum. Planar faces chord nothing of their
own. The mesh therefore carries `volSymDiff`, and a class-B result is an
ordinary mesh-path operand and export input.

### 4.5 Consumers

| Consumer | Behaviour |
|---|---|
| `Union`/`Cut`/`Intersect` with a prism, stacked or brep partner | class B again over the face view (§5), so a cross-drilled plate takes a second cross hole and a coplanar blind cut alike; a pair outside B1–B8 takes the mesh path over §4.4's mesh |
| `Body.Placed` / `Duplicate` / `PlacedCopy` / `Mirrored` | re-lifts every face frame under the composed motion; a reflection flips `outward` and every wall's winding, exactly as `prismPayload.reflected()` does |
| `Fillet` / `Chamfer` / `Shell` | `ErrUnsupported` — modify-reach gains an RX row for the payload. This is STAGED, not SX9's permanent exclusion: the faces are analytic carriers with recorded trims, so a later design can rewrite a planar face's region and re-trim its walls on the same terms modify §2 rewrites a section |
| `ThroughAll` / `ToFace` stops | a planar face's level is its frame and `z0`; a directional extent reads the per-face extremes (§4.3), and refuses a record carrying a `delta` as a prism's does |
| `Verify` validity | by construction (§4.2); the structural audit runs |
| `Verify` tolerance gate | `gateWitnessPrism`'s reader over every body vertex and each swept face's prism-wall witnesses, shrunk by the largest `delta` plus axial term (verification §3) |
| `Verify` wall survey | staged `Suspect` (`DiagUnsupportedSurveyPayload`): the 2D spanning-disk reduction has no single section to read |
| `Verify` undercut, minimum radius | per face: exact normal ranges for planes and cylinders (DX7's reading), the tightest concave circular wall radius (DX8's) — both land with the payload |
| Clearance kernel | a `bodyGeom` arm adding each face's plane or cylinder carrier with its trims, as `addPrismFaces` does for a prism; undecidable cells stay `Suspect` |
| Interference | `analyticBodiesEqual` undecided; the read-only mesh intersection over §4.4 |
| Tessellate, STL/OBJ/3MF | §4.4 |
| STEP | the analytic writer where every edge is a `Line3` or a full `Circle3` and every cylindrical wall has two one-circle loops (`docs/step-export-design.md`): a cross-drilled box meets it; a keyway (arc rims) takes the faceted writer until the writer admits `Arc3` edges |
| Prism-boolean's class | a brep operand misses G1 and never enters class A; class B's face view covers the co-directional pair of planar faces (§5.2) |

## 5. The 2D answers, and the 3D computations

Every face of the result is decided by one of three private `sketch` scenes.
decad builds the scene from its own records, asks `sketch` to arrange it,
and selects cells through `prismcells.Classify`'s flag comparison
(prism-boolean §4.2) — it computes no crossing, no containment and no cut
parameter. The three shapes, for operand X against operand Y with `N_X ⟂
N_Y`:

| Scene | Built in | Holds | Answers |
|---|---|---|---|
| **Section scene** of X | X's sketch plane | X's loops, plus one line per face of Y that contains `N_X` (Y's caps and Y's walls parallel to `N_X`), traced in X's plane | which fragments of X's own walls survive (their `TStart`/`TEnd`), and the cells of X's section each of Y's columns passes through |
| **Perpendicular-face scene** of a planar face F of X with `n_F ∥ N_Y` | F's plane | F's loops, plus Y's whole section re-expressed into F's frame (a 2D rigid map, the identity or a signed permutation under B4) | F's new loops: prism-boolean's clean-nesting match or crossing sub-case, verbatim |
| **Parallel-face scene** of a planar face F of X with `n_F ⟂ N_Y` | F's plane | F's loops, plus one rectangle per chord of Y's section along F's trace line, `chord × [y0, y1]` along `N_Y` | F's new loops, by the same cell selection |

The chords of the parallel-face scene come from Y's section scene: F's trace
line is one of its lines, and the fragments `sketch` returns along it, with
`TExact` certified cuts, are the chords. A circular wall of X survives as
the arc fragments its section scene returns, each over the z-intervals that
Y's perpendicular faces bound (below), and each such patch is inside the
result when the cell of X's section it borders is selected and the level
interval lies inside Y's own `[y0, y1]` — both facts the scene and the exact
level comparison already decided.

The 3D computations decad performs, each with its bound:

| Computation | How | Bound | Reject-only check |
|---|---|---|---|
| Perpendicularity, B3 | `==` on the dot product of stored world normals | a decision | misses take the mesh path |
| Trace of a plane of Y in X's plane | two points of the plane through `Frame.ToLocal` into X's frame; under B4 every coordinate is a sum of signed stored floats, held exactly, and the trace is the line through them | zero under B4 (`rationalFloatError` of each coordinate is computed and must be `0`; a nonzero one refuses the pair in this increment) | — |
| Level of a perpendicular face of Y along `N_X` | the exact rational `p · N_X` of a recorded point `p` of that face (`big.Rat` over stored floats); the held float is rounded once | `rationalFloatError` into that patch's `z0Delta`/`z1Delta`, prism-boolean G5's own mechanism | interval comparisons are exact over `big.Rat` |
| Chord endpoints, surviving fragments | `sketch`'s certified cut parameters | `δ_cut` (prism-boolean §7's `cutDisplacementAllow`) | `TExact == false` refuses (`ErrUnrecordableProfile`); the seam's range falsifier rejects a disproven flag |
| Re-expression of Y's section into F's frame | one rigid 2D map per coordinate | zero under B4; `δ_reexpress` otherwise (§8) | — |
| Curved × curved separation, B7 | outward-rounded per-wall boxes compared exactly | a decision | refuses any overlap, including false overlaps |
| Closure of each face loop | the seam's junction falsifier on every assembled loop | — | rejects a contradicted junction (RB9) |
| Simplicity, orientation, nesting per face | modify §5's audit per planar face record | verification §4's floor | S7/S8/S9 refuse; none admits |
| Edge pairing of the result | exact record identity (§4.2) | — | an unpaired edge refuses |

No residual against a curve admits anything anywhere in this design; the
only residual-shaped quantities — the contact floor and the chord bounds —
refuse or are charged.

## 6. Interaction with mirror and pattern

- A prism-group tool (mirror §6.3) along a perpendicular sweep is class B
  with `Count` operand-Y loops in every scene: a row of cross holes through
  a bar is one section scene per operand and one perpendicular-face scene per
  wall, with `Count` holes each. The efficient N-hole case holds in both
  directions.
- A reflected operand in class B re-winds its record as A4 does before the
  face view is built; B4's permutation test reads `ApplyDir`, which a
  reflection maps correctly.
- A mirror join's result (mirror §5) is a prism and enters either class
  unchanged.

## 7. Upstream dependency

Class A3 needs `sketch` to resolve coincident collinear LINE carriers the way
it resolves coincident circular ones. The ask is filed at
`../sketch/.tmp/decad-handoff-coincident-line-carriers.md` with the §2 W rows
as evidence. This design does not block on it: A1, A2, A4–A6 and class B
stand without it, and the mirror join covers the symmetric half of the
shared-wall workload exactly. If `sketch` declines, the alternative is a
decad-side pre-resolution of collinear overlaps in exact rational
arithmetic before the scene is built (the mirror join's splice generalised
to two records); §10 records it as an open question.

## 8. Do not do this

- **Re-sweep a box as a prism along the tool's axis** so a cross hole becomes
  a clean-nesting cut. It works once and fails to compose: the result is a
  prism along y whose next cut along z finds no prism at all, and the face
  roles change under the caller.
- **Loosen B3/B4 to a tolerance.** A near-perpendicular pair's traces are
  then rounded lines, and a cut on a rounded line amplifies by `1/sin θ` with
  no charge; A6 is where that charge lives, and until it lands the pair takes
  the mesh path.
- **Derive a cylinder's θ-trim from `acos`.** The trim is the arc fragment
  `sketch` certifies in the section scene; no angle is ever computed.
- **Admit cylinder × cylinder at "small radius" or "far apart" by a
  distance.** B7 is an exact box separation or nothing.
- **Make `brepPayload` the stored form of every prism.** The face view is
  derived on demand; prisms keep `prismPayload` and every consumer that
  dispatches on it.
- **Read the per-face `Faceted` bound as a per-facet bound to extend chains.**
  The S11 refusal is correct for what the face publishes; the fix is an
  analytic result, not a looser gate.
- **Fall back to the mesh path after a scene has been built and refused**
  (prism-boolean §3.4's point of no return): a refusal past the gate is an
  error the caller branches on. A topology the scene leaves unresolved is not
  a refusal: prism-boolean §4.4 sends it to the mesh path, and A1's crossing
  interface is one. Nor is a pair whose cuts carry an input displacement
  (A6): it builds with its crossing charge or takes the mesh path.

## 9. Test plan

Every test asserts computed geometry with a closed-form expectation; bounds
are relations, never literals.

- **A1 boss on plate**: 40×40×10 plate on XY `Union` Ø10 boss on
  `CreateOffsetPlane(XY, 10)` extruded 15: no `Faceted` face, two slabs, one
  interface with one exposed floor record (plate outer, boss hole), volume
  within its bound of `16000 + π·25·15`, bound below 1e-9 mm³, 8 faces,
  `CapStart` one face, `CapEnd` one face. The rooted boss (offset 5,
  extruded 20) gives three slabs and the same volume. Two equal boxes stacked
  edge-to-edge give `Exact` volume and I5's equal-outer column of one wall
  per side.
- **A1 fallback**: a rooted boss whose footprint crosses the plate's outline
  takes the mesh path, a `Faceted` result whose volume bound contains the
  closed form, with the test asserting the interface match reports a
  `Partial` edge.
- **A4 reflected operand**: the L of mirror §2 mirrored across x = 30 and
  cut by a same-plane Ø3 cylinder inside its leg (M5) builds analytically,
  `Approximate`, volume within its bound of `1750 − π·1.5²·10`; the same
  image cut by a quarter-disc tool builds with the arc re-wound, volume
  within its bound of `4000 − 16π·10` for a mirrored 20×20 box; a box
  holding the mirrored L unions to the box's own volume through the
  select-all merge; T1's both-reflected cut builds through the shared-axis
  arm with `sectionDelta` exactly `0.0`; `Verify` on the mirrored L beside
  the cylinder reports one `Interference` row of `π·1.5²·10`. A scene test
  reads `prismcells.Classify` on a reflected box crossing a box and finds
  the exact areas 75, 25 and 75 for A-only, both and B-only. M3 (the L
  unioned with its image across x = 15) and T2 fall back to the mesh path's
  coplanar refusal: the two outlines share collinear walls at y = 0 and
  y = 5, which meet at a sine of zero, and `sketch` reports that arrangement
  as an invalid region, which A3 waits on.
- **A5 N-hole cut**: mirror §8's N-hole test, and the disjoint `Union` of
  S12 reporting two lumps with `Exact` 1000 mm³ and no `Faceted` face.
- **A6 crossing charge**: a Ø40 hub unioned with a 7×3 tooth rooted inside
  it and placed by `RotationAround` through 60° builds analytically within
  its bound of the closed form; P1's own tooth, its root arc on the hub
  circle, falls back to the mesh path, as does the same tooth cut from the
  hub by a taller tool; S9 (a box intersected
  with the same box rotated 45°) builds an octagonal prism whose bound
  contains the exact rational residual against the two recorded squares;
  the charge covers `(δ + δ)/sin θ` when both operands bring `δ`, the floor
  leaves a junction whose sine bound is not above `ε` uncharged, an outside
  touch is arranged with no cut, and an amplified pair's RB9 (fu141's
  overshooting quadrilateral crossed by a box) falls back while the same
  quadrilateral's uncut merge keeps refusing; the
  formerly rerouted fixtures (a nonidentity shallow crossing, a displaced
  chain, a walk charge, a placed `Verify` pair) build or measure with a
  section displacement covering the input displacement over the crossing
  sine.
- **B cross-drill**: S1 builds a `brepPayload` with 7 faces (6 planar, 1
  circular wall), volume within its bound of `16000 − 9π·20`, bound below
  1e-9 mm³, `Lumps` one, every edge on two faces, the two walls at y = 0 and
  y = 20 each carrying one circular hole loop, `Cylindrical()` one face,
  `Verify` `Sound` with no survey asked, undercut survey measured, STEP
  analytic path (every edge a line or full circle).
- **B slot**: B1 builds with `Exact` 14000 mm³ and 10 planar faces.
- **B keyway**: B2 builds with 7 faces (5 planar, 2 circular walls of the
  rod split by the key), volume within its bound of
  `π·100·40 − 40·(2·√96 + 100·asin(0.2))` (the rod's strip |x| ≤ 2 over its
  40 mm length) computed over certified enclosures, the arc walls' rims
  `Arc3`, STEP taking the faceted writer.
- **B chain**: S11's three cross holes build analytically, then a fourth
  overlapping the first refuses by B8 or B7 with `ErrUnsupported`, pinned.
- **B gate misses**: S10 (tilted plane), S7 (cylinder × cylinder), a 1-ulp
  non-perpendicular pair, a non-permutation frame pair, each take the mesh
  path with the §2 result unchanged; the non-perpendicular fixture asserts
  `N_A · N_B != 0` itself.
- **B with a hole breaking out**: a cross hole whose circle crosses the bar's
  top face: the perpendicular-face scenes report `Partial` edges, the
  crossing sub-case builds, the top cap gains a chord notch, volume within
  its bound of the closed form (box minus the cylinder clipped by the top
  plane: circular-segment integrals).
- **brep measurements**: a prism viewed as a brep reports volume, area,
  bounds and centroid value bit-identical to `prismPayload`'s for an
  all-line record whose readings are exact rationals rounded once, and
  within the composed bounds for a circular one. The brep's centroid bound is
  the exact error of that one rounding, so it is at or below the prism's.
- **Cancellation and the cap** as prism-boolean §15, per scene.

## 10. Open questions

- **A3 fallback if `sketch` declines collinear carriers.** Recommendation:
  wait for `sketch`; the mirror join covers the symmetric workload, and a
  decad-side overlap resolution would duplicate an arrangement rule the
  coincident-circle case already gets upstream.
- **Should B4 (signed-permutation frames) be relaxed before A6 lands?**
  Recommendation: no. A rounded trace has no charge without A6, and the
  datum planes cover the measured workload.
- **Should class B's roles mint `capStart`/`capEnd` when a result keeps an
  operand's caps whole?** Recommendation: no; prism-boolean §11's fresh-roles
  decision stands, and a cap kept whole is still a face of a new record.
- **Should the brep's planar-face scenes run the modify §5 audit per face
  (chosen) or once over a merged section?** Recommendation: per face; the
  faces lie in different planes and share no section.
- **Class C's ellipse.** Recommendation: file the `sketch` ask only when a
  consumer needs an oblique hole; the whole-scene `TExact` gate makes it a
  larger upstream change than a carrier rule.
- **S8's sphere facet cap.** Recommendation: separate task; raising the cap
  or deriving the boolean's tolerance per operand is a mesh-path change
  outside this design.

## 11. PR split

Each PR ships code and tests; this document ships with PR 1.

1. **A1 stacked union.** Overlapping/touching `Union` intervals, slab
   splitting, per-slab select-all merge, interface exposure by the
   clean-nesting match, stacked §2.2's union reading of I5–I7, §9's A1 tests.
2. **A4 reflected re-expression.** G2 replaced by the re-wound record,
   `AuthoredReversed` on the re-wound record, the shared-axis both-reflected
   case, the interference twin, §9's A4 tests. Depends on nothing.
3. **A5 multi-region.** The prism-group tool in `Cut`'s clean-nesting match
   and crossing sub-case, the disjoint `Union` to a one-slab group, with
   mirror §11 PR 4.
4. **A6 crossing-sensitivity charge.** Certified `sin θ_low`, the amplified
   fragment charge replacing §3.4's reroute, the noise-floor refusal, the
   rotated-tooth and S9 fixtures.
5. **`brepPayload` foundation.** The record, the face view of prism and
   stacked payloads, topology and roles, measurements, tessellation with the
   occupied-volume proof, placement, `Verify` validity and the tolerance
   gate; the prism round-trip tests.
6. **Class B `Cut`.** B1–B8, the three scenes, result assembly, S1/B1/B2
   fixtures and the gate-miss fixtures.
7. **Class B `Union`/`Intersect` and chaining.** The rooted boss (B4), a
   brep operand in both positions, the breaking-out hole, S11's chain.
8. **brep consumers.** Undercut and minimum-radius surveys, the clearance
   `bodyGeom` arm, STEP's analytic arm with `Arc3` edges, modify-reach's RX
   row.
9. **A3 shared walls.** Waiting on §7's `sketch` hand-off; lands the W1–W4
   and S12c fixtures as analytic results once the arrangement resolves
   collinear carriers.
