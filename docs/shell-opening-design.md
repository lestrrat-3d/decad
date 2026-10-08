# Shell Opening Design

How `Body.Shell` removes one proper connected run of a prism's side faces
(`docs/modify-reach-design.md` Table RX row RX1, Table BX row BX4 — "reach
§N" below), and how the same rim rule lifts the right-angle restriction on a
revolve side opening (reach §9.3.2). Companion to `docs/modify-design.md`
("modify §N"), which owns the call contract, the gate order, the section
offset and its §5 audit; to `docs/general-boolean-design.md` ("general-boolean
§N"), which owns the `brepPayload` the prism result is recorded on; and to
`docs/brep-modify-design.md` ("brep-modify §N"), which owns what a later
modify op does with that result.

Four tables are normative:

| Table | Owns | Section |
|---|---|---|
| **RO** | the rim at each end of the kept chain, per corner kind | §2 |
| **SO** | every refusal this document adds, with its sentinel and gate order | §5 |
| **BO** | result payloads, topology and roles | §6 |
| **DO** | what every downstream consumer reads | §7 |

## 1. Problem

Reach §9.2 builds a prism shell with a side opening on `stackedPrismPayload`
and closes the wall section with the exact normal segment of length `t` at
each end of the kept chain. The implementer of reach PR C found three defects,
each reproducible on an ordinary part:

1. On every box the normal segment at an end corner lies on the removed
   face's own line and overlaps it, so the exposed cavity floor is not
   "the removed run followed by the wall section's boundary": that record
   doubles back along the overlap.
2. The outer wall along a kept face changes loop record between the cap slab
   (`P`) and the wall slab (`W`), so one stack column cannot carry it as one
   face; and on a box the floor strip, the ceiling strip and the two end strips
   on the removed face's plane are one planar face with the opening as its
   hole, which a stack column cannot express either.
3. The normal segment follows the removed face only at a right angle. At an
   acute corner an inward normal segment leaves the receiver; at an obtuse one
   it stops short of the removed face.

Each defect is a consequence of one wrong premise: that the rim is a segment
perpendicular to the kept wall. The rim is the removed face's own surface,
cut by the kept wall's offset (§2). With that rule the floor, the walls and
the rims are a set of planar regions and swept pieces over a stack of three
levels, which `internal/stackedbrep` already restates into a `brepPayload`
with flush walls merged into one face (general-boolean §3 "A1 as a brep").
The prism result is therefore a `brepPayload`, not a `stackedPrismPayload`.

## 2. Table RO — the rim rule

### 2.1 Statement

Write `P` for the receiver's hole-free section, walked counter-clockwise with
the material on its left; `R` for the removed run, a proper connected run of
whole coalesced walks of `P`'s outer loop; `K` for the kept chain, the rest of
the loop, running from `vA` (where `R` ends) to `vB` (where `R` starts). Write
`s` for the sense, `+1` inward and `−1` outward, and `k'` for the carrier of a
kept walk `k` offset by `s·t`: the parallel line `s·t` to the walk's left, or
the concentric circle of radius `R_k − s·inside·t` (modify §8's table).

At each end of `K`, `v` is the end vertex, `k` the kept walk ending there and
`r` the removed walk adjacent to `v`. The **band** at `v` is the open strip
between `carrier(k)` and `k'`: the points at a signed distance strictly
between `0` and `s·t` from `carrier(k)` on the material side inward, or off it
outward.

**The rim at `v` is the piece of `carrier(r)` from `v` to `q`, where `q` is
the first point at which `carrier(r)`, walked from `v` in the direction that
enters the band, meets `k'`.** The cavity's boundary on `carrier(r)` begins at
`q`, and `k'` is trimmed at `q`. Nothing else is cut, and no segment is built
that lies on neither `carrier(r)` nor `k'`.

Exactly one direction from `v` enters the band. `r` leaves `v` with a unit
tangent `r̂`, and `k` arrives with a unit tangent `k̂`; the band lies on the
side of `carrier(k)` the normal `n̂ = s·(−k̂_v, k̂_u)` points to. Where
`r̂·n̂ > 0` the forward walk along `r` enters the band; where `r̂·n̂ < 0` the
backward walk does. Where `r̂·n̂ = 0` the two carriers are tangent at `v` and
no direction enters the band first: that end is refused (SO1).

The derivation is §2.2 and the corner-by-corner reading is §2.3. The rule is
the one reach §9.3.2 already applies to a right-angle revolve rim ("the rim is
the removed face's own cut through the wall"), stated for every corner.

### 2.2 Derivation

A shell removes faces and lines every kept face with a wall of thickness `t`.
The body is the receiver less its **inner body**, bounded by every kept face
moved `s·t` along its own normal and every removed face left where it is,
each surface extended until it meets its neighbours (modify §8's offset,
which is modify §7's corner rule in the section plane). A removed face is
moved by zero, so where it meets a kept face the inner body's edge is the
intersection of the kept face's offset surface with the removed face's own
surface. In the section plane that is `carrier(r) ∩ k'`.

The receiver's boundary at `v` is `k` then `r`. The wall behind `k` is the
band, and it ends where the inner body begins, on `carrier(r)`. Walking
`carrier(r)` from `v` into the band, the wall material lies on one side and
the inner body on the other until `k'` is reached at `q`; past `q` the curve
bounds the inner body alone. So the rim is `v → q`, the wall's section near `v`
is the band cut off by that piece, and the inner body's boundary runs along
`carrier(r)` from `q` onward.

The right-angle normal segment of reach §9.2 is this rule at one corner kind:
where `r̂ ⟂ k̂`, the piece of `carrier(r)` from `v` to `k'` is the
perpendicular of length `t`. At every other corner the two differ, and the
normal segment bounds a wrong solid: at an acute corner it leaves `P` (the
band's end lies outside the receiver), and at an obtuse corner the triangle
between the normal segment, `k'` and `carrier(r)` is wall material the
construction would omit, since it lies inside the band and outside the inner
body. Either is a wrong body, not a coarse one.

With `θ` the material's interior angle at `v` and `k` a line, `q` lies on
`carrier(r)` at the signed distance `t/sin θ` from `v` along `r̂`, and on `k'`
at `−t·cot θ` from `k`'s offset foot `v + s·t·n̂`. The signs read as follows.

### 2.3 Corner table

| Corner at `v` | `q` | Rim | Where the cut lands on `k'` |
|---|---|---|---|
| right (`θ = 90°` or `270°`) | the foot `v + s·t·n̂`, which lies on `carrier(r)` | perpendicular to `k`, length `t` | at the foot |
| acute (`θ < 90°`) | forward along `r`, at `t/sin θ > t` from `v` | along `r`'s span, longer than `t`; the wall ends in a wedge whose edge is `v` | short of the foot: `k'` is trimmed back by `t·cot θ` |
| obtuse (`90° < θ < 180°`) | forward along `r`, at `t/sin θ > t` from `v` | along `r`'s span, longer than `t` | past the foot: `k'` is extended by `t·|cot θ|` |
| reflex (`180° < θ < 360°`) | backward along `carrier(r)`, at `t/|sin θ|` from `v`, inside the material inward (outside it outward) | the extension of `r`'s carrier beyond `v`; its outward normal is opposite to `r`'s | short of the foot for `θ < 270°`, past it for `θ > 270°`; the exact extension of a straight `r` is one `LineSeg` from `q` through `v` |
| line `k`, arc `r` | the first crossing of the line `k'` by `r`'s circle, sweeping from `v` in the entering sense | an `ArcSeg` about `r`'s centre from `v` to `q` | as the angle between `k̂` and `r`'s tangent at `v` reads above |
| arc `k`, line `r` | the first crossing of the concentric circle `k'` by `r`'s line, walking from `v` in the entering direction | a `LineSeg` from `v` to `q` | the trimmed arc of `k'` ends at `q` |
| arc `k`, arc `r` | the first crossing of the concentric circle `k'` by `r`'s circle, sweeping from `v` in the entering sense | an `ArcSeg` about `r`'s centre | the trimmed arc of `k'` ends at `q` |
| smooth (`θ = 180°`: `r̂ = −k̂`, including a line–arc or arc–arc G1 join) or cusp (`r̂ = k̂`) | none: no direction enters the band first, and the wall's thickness would reach zero at `v` along the rim | refused, SO1 | — |

Three consequences the table makes visible:

- **A reflex rim is new geometry inside the material.** Inward, the band
  extends past `v` into the solid, and the rim is the piece of `carrier(r)`
  from `q` to `v` that lies there, with the wall on one side and the cavity on
  the other; the cavity reaches `r` through the region between the rim and
  `r`'s span. The 270° corner of an L prism is the case the normal segment
  happened to get right: `cot 270° = 0`, so the rim is the perpendicular.
- **A near-smooth reflex corner gives a long thin wedge.** As `θ → 180°` the
  cut `q` runs far back along `k'` (`|cot θ| → ∞`) and the wall tapers to zero
  thickness at `v` along a long rim. That is the shape the caller asked for by
  removing a face nearly tangent to a kept one; the body is built when every
  gate passes. The gates that bound it are SO2 (the cut beyond the far end of
  `r`) and S11a (the cut consuming `k'`), never a tolerance on `θ`.
- **A cut's existence is decided by closed form, never by sampling.** A line
  meets a line in zero or one point; a line meets a circle in zero, one or
  two; two circles in zero, one or two. The entering direction picks one root
  or finds none. A tangent root (one point) is the smooth or cusped case and
  is SO1.

### 2.4 The computation and its bound

`offset2d.OpeningJoin(k, r survey2d.SideWalk, atEnd bool, s, t, tol float64)
(Join, error)` is the one owner of the rim cut, beside `CornerJoin` and
`MirrorCornerJoin` in `internal/offset2d/section.go`'s family: `k` is the kept
walk and `r` the removed neighbour; `atEnd` says whether `v` is `k`'s end
(`K`'s end, `r` follows) or its start (`K`'s start, `r` precedes). It returns
`Join{VertU, VertV: v, M: q}` with `Arc` and `G1` false. `offset2d.RimSegment`
writes the rim segment from that join and `r`'s record: a `LineSeg`, or an
`ArcSeg` about `r`'s centre turning in the entering sense from `v` to `q`, and
the other way when the caller's loop walks it from `q` to `v`.

| Step | Rule |
|---|---|
| classify | `cross = k̂ × r̂`, `dot = k̂ · r̂` on the held unit tangents, with modify §8's dead zone: `|cross| ≤ tol` is SO1 (`ErrUnsupported`) whatever `dot` is — a smooth join and a cusp alike |
| entering direction | `r̂ · n̂ > 0` forward, else backward; `n̂` is `k`'s left unit normal at `v` times `s` |
| exact pairs | a straight `r` whose raw tangent at `v` has a float dot product of exactly zero with `k`'s: `q` is `k`'s offset foot `v + s·t·n̂` along `k`'s held unit normal, bit for bit what the right-angle open chain wrote before this rule. Two axis-aligned lines (`k` with `du = 0` or `dv = 0`, `r` on the other axis) are such a pair, and there the foot is the pair of levels — `r`'s constant coordinate, which `v` carries, and `k`'s offset level `v_coord + s·t` — read as `stackedbrep`'s plane–plane junction reads it, so the record and the engine hold one point. An axis-aligned line `r` through the centre of a circular `k`: `q` is the centre moved by `k'`'s radius along that axis |
| float solve | otherwise `Intersect(offsetCarrier(k, s, t), offsetCarrier(r, s, 0))` over all roots, choosing the first root reached from `v` in the entering direction: along a line by signed parameter; around a circle by the signed sweep from `v` in the entering sense. No root, or the first root reached lying on `carrier(k)` (the circle re-crosses `k` before `k'`), is SO1 |
| span | the forward cut must lie strictly inside `r`'s span: a line by parameter in `(0, 1)`, an arc by sweep strictly below `r`'s own; otherwise SO2. A backward cut lies off `r`'s span by construction and has no span test; the audits of §4.7 decide whether it crosses anything |
| consumption | `k'` trimmed at `q` must still advance along `k` (`WalkConsumed` with `q` as the trimmed end, modify §7's S11a reading); a consumed `k'` is S11a |

The cut's **displacement** is proven as modify §9 proves a cup's joins
(`offset2d.LoopReach`'s method, `capcontour`'s enclosures): the same closed
form is re-evaluated over rational intervals across the whole thickness
interval `[t − tDelta, t + tDelta]`, with `k`'s walk taken exactly and `r`'s
carrier exact at offset zero, the roots enclosed with outward-rounded square
roots, and the held `q` charged its enclosure's greatest reach.
`offset2d.OpeningReach` is that enclosure for one opening end: `k`'s offset
carrier and `r`'s own carrier are enclosed as `LoopReach` encloses a miter's,
every root is enclosed, and the denoted cut is the root the entering
direction reaches first — on a straight `r` the one whose advance from `v` is
certainly positive and certainly smallest, on a circular `r` the one the
orientation of `v` and the two roots places first around the circle. It never
takes the root nearest the held `q`. `offset2d.ChainReach(chain, line, end0,
end1, s, t, amount, tol)` is `LoopReach` for an open chain: interior corners
as `LoopReach` reads them, a mirror end against the widened axis line, and an
opening end (`ChainEnd` with its `Removed` walk) through `OpeningReach`. A
cut whose enclosure is the held float — the exact pairs over exact
coordinates and an exactly converted thickness — reaches zero. A root the
enclosures cannot place or order, or a reach the enclosure cannot bound, is
`ErrUnbounded` (SO4).

## 3. The three regions

`offsetOpenChain` (`shell_revolve.go`, moved to `shell_chain.go`) already
offsets an open chain with per-end joins. Its end kinds become an enumeration:
an **axis end** (`MirrorCornerJoin`, the revolve's), and an **opening end**
(`OpeningJoin` with the removed neighbour walk). It returns `K'` in `K`'s own
walk order from `qA` to `qB`, the join at each interior corner as modify §7
rules (miter, arc of radius `t` about the corner, G1 foot), and the two cut
points. Every walk of `K'` passes `WalkConsumed` (S11a) as `offsetOpenChain`
checks today.

With `R'` the removed run re-cut at both ends — `r_B` (the walk after `vB`)
starting at `qB` instead of `vB`, `r_A` (the walk before `vA`) ending at `qA`
instead of `vA`, each a whole `LineSeg` or `ArcSeg` on its own carrier, and
every other walk of `R` verbatim — the three regions are:

| Region | Loop, walked with the material on its left | Holds |
|---|---|---|
| `P` | the receiver's own outer loop | the cap material at a kept cap |
| `W` (wall section) | inward: `K`, rim `vB → qB`, `K'` reversed (`qB → qA`), rim `qA → vA`. Outward: `K'` (`qA → qB`), rim `qB → vB`, `K` reversed, rim `vA → qA` | the wall material over the cavity's height |
| `C` (cavity section) | inward: `K'` (`qA → qB`), then `R'` (`qB → … → qA`). Outward: `K` (`vA → vB`), then `R` verbatim — the original solid is the cavity | the exposed floor and ceiling of the cavity |

Inward `C ⊂ P` and `W = P \ C`; outward `P ⊂ O` where `O = W ∪ P` is the outer
region, the loop `K'` then `R'`. `R'` appears in `C` inward and in `O`
outward; the rims appear in `W` only. A reflex rim is a piece of `C`'s (`O`'s)
walk along `carrier(r)` and of `W`'s loop, so the record states it once per
region and the two agree by construction (§4.7 checks it).

Levels: with the receiver on `[z0, z1]`, inward the cavity spans
`[z0 + t, z1 − t]` with both caps kept, `[z0 + t, z1]` with the end cap
removed, `[z0, z1 − t]` with the start cap removed, `[z0, z1]` with both
removed; outward the caps extend to `z0 − t` and `z1 + t` where kept. Each
derived level carries its source end's displacement, the thickness conversion
and its own float sum's rounding, as `shellClosedPrism` derives its levels.

## 4. The brep construction

### 4.1 Slabs

The result is a stack of at most three slabs, one region each, inward:

| Slab | Interval | Region |
|---|---|---|
| floor (kept start cap only) | `[z0, z0 + t]` | `P` |
| wall | the cavity's interval | `W` |
| ceiling (kept end cap only) | `[z1 − t, z1]` | `P` |

Outward the cap slabs hold `O` on `[z0 − t, z0]` and `[z1, z1 + t]`, and the
wall slab holds `W` on `[z0, z1]`. Each interface exposes `C` once, facing
the cavity: up at the floor, down at the ceiling. The stack's bottom cap is
the lowest slab's region facing down and its top cap the highest facing up.

**Both caps removed is a prism.** The stack is then one slab over `W`, and the
result is `prismPayload{profile: W}` over the receiver's interval, frame and
placement, as a both-caps tube is (modify Table B, B2/B3): every consumer and
every further modify op already take it, its roles are the prism's own
(`side(0,j)`, `capStart`, `capEnd`), and the rims are two of its side walls.
Nothing below applies to it beyond §3's construction and §4.7's audit.

### 4.2 What the engine writes

`stackedbrep.Engine` takes the slabs' loops and levels and writes the faces
general-boolean §3 "A1 as a brep" states, with no scene: every loop here is
decad's own record, so `LoopOf` restates it directly, and the faces are:

- **horizontal faces**: the two caps and the exposed `C` at each interface,
  planar faces in the reference frame, each line unit split at the body's
  vertices at that level and each circle unit at every vertex on it;
- **axis-aligned straight walls**: one planar face per carrier plane and
  material side (`stackedBrepWallFaces`), the rectangles each slab's segment sweeps with
  their shared edges cancelled and the rest chained into loops;
- **oblique straight and circular walls**: swept pieces between consecutive
  vertices, the identical piece in consecutive slabs joined into one face, a
  vertex at the level between recorded as a side-line split.

What that produces on the removed face's carrier, the one place the three
slabs disagree:

| `r` | Pieces on `carrier(r)` | Faces |
|---|---|---|
| axis-aligned line, `|R| = 1`, both cuts forward | floor strip `r × [z0, z0+t]`, ceiling strip, two end columns `(v → q) × [z0, z1]` | one planar face whose loop is `r × [z0, z1]` and whose hole is the opening `(qB → qA) × cavity interval` |
| axis-aligned line, a reflex cut at one end | the strips and the far column form a U; the reflex rim `(q → v) × cavity interval` faces the other way | two planar faces on one plane: the U, and the rim rectangle with the opposite material side |
| axis-aligned line, `|R| ≥ 2` | a U per removed walk (no second column at a walk's interior end) | one planar face per removed walk and side |
| circular arc | the same pieces on one cylinder | the end columns are each one swept face over `[z0, z1]`, joined across the three slabs, with side-line splits at `q`'s two levels; the floor and ceiling strips are swept faces over the cap slabs alone |
| oblique line | the same pieces on one plane the record cannot hold as a planar face | four swept faces (two columns, two strips), coplanar, each edge paired by identity |

The circular case needs nothing beyond the engine's rules: `W`'s rim is an
`ArcSeg` about `r`'s centre, so it shares `r`'s carrier key, `circlePoints`
splits the cap slabs' `r` unit at `qA` and `qB`, and `stackedBrepSweptFaces` joins the
column pieces. The oblique case needs the cap slabs' record of `r` pre-split
at every forward cut (`vB → qB`, `qB → qA`, `qA → vA` as three collinear
`LineSeg`s), since an oblique line carrier is keyed by its recorded endpoints
and the engine refuses a vertex strictly inside one; `LoopOf` keeps collinear
units apart when their carriers differ, and the piece keys then match across
slabs.

### 4.3 Engine extensions

| Extension | Where | What |
|---|---|---|
| planar wall with holes | `brepgeom.StackedWallRegions(embed, loops)`, read over every loop of one wall key | the chained loops of one `(axis, level, side)` key are grouped: all counter-clockwise → one face each; exactly one counter-clockwise and the rest clockwise → one face `{Outer, Holes}`; anything else is a miss. The §5 audit's S9 then proves each hole nested in its outer, and refuses; it admits nothing |
| vertex events | `Engine.Event(p Point2, level int)` exported | the shell marks each end vertex `v` whose cut runs backward along `carrier(r)` (a reflex end inward, a convex end outward) at both cavity levels, so `C`'s walk along `carrier(r)` is split at `v` where the rim face and the floor strip meet (`CutsOnLine` reads events; a vertex whose junction pair does not change between slabs gets none on its own) |
| circle–circle junction (PR 5) | `Engine.junction` | two circles meeting where both walked ends are one recorded point take that point with the larger allowance; two circles whose walked ends differ still miss |

A miss anywhere in the engine (`brepgeom.ErrStackedWallMiss`) is SO5, never a
silent fallback: a shell has no mesh path to fall to. One miss every
rectilinear section can reach: an arc join of `K'` (modify §7's arc of radius
`t` about an outward convex or inward reflex interior corner of `K`) meets its
neighbouring offset lines tangentially, and `Engine.junction` decides a
line–circle crossing's side by its distance from the centre's coordinate,
which a tangency makes zero. A kept cap slab or an interface over such a join
is therefore SO5. With both caps removed the wall is BO1's prism, which takes
the arc.

### 4.4 Displacements

Every face carries one section displacement `delta`, the larger of the chain
reach's figure (`3 × ChainReach`, modify §9's three-reach argument over the
open chain and both cuts) and the engine's largest canonical-vertex
allowance (`Engine.Allow()`), as `stacked_union_brep.go` charges its faces.
Levels carry their own `z0Delta`/`z1Delta` (§3). A rectilinear section whose
offset levels and cuts are exact floats and whose thickness converts exactly
has `delta = 0`, and its volume is `Exact` (general-boolean §4.3). `stack`
is nil: the result is not an A1 operand (brep-modify DB11).

### 4.5 Topology and roles

The record is general-boolean §4.2's: every edge bounds exactly two faces by
`brepTopologyContext`'s count, and a record that fails it is `ErrUnsupported`
(SO5) — the proof that the three regions closed against each other. One lump,
one shell, `Shell.IsVoid()` false: the cavity reaches the outside through the
opening. Roles are the brep's, `face(k)` and `wall(k)` by result index,
minted fresh under the shell's producer identity; no `capStart`/`capEnd`, no
`rim`, no receiver role (modify §11, general-boolean §4.2). A caller names a
result face by geometry. Edge convexity reads general-boolean §4.2's walked
boundary: a coplanar junction between two swept pieces of one oblique plane
reads concave, as every zero turn does.

### 4.6 What `sketch` is asked

Nothing. `K'`, the rims and `C` exist in no sketch: they are decad's own
geometry synthesised from decad's own record, the class modify §5 names for
the offset and the corner blend, and decad proves them with the same
closed-form audit (§4.7) and the record's closure by counting. No upstream
claim exists to bless, no residual admits anything, and no scene is built: a
scene of `P` and `W` would place `W`'s cut vertices on the interior of `P`'s
walks, which `sketch` returns as `Partial` fragments whose `TExact` decad can
only reject (`docs/sketch-seam-design.md`).

### 4.7 Audit

In order, each refusing and none admitting:

1. S11a as `K'` is built (a consumed walk, a dropped circle); SO1–SO4 as each
   cut is computed.
2. Modify §5's audit on `W` and on `C` (`O` outward): S8 (orientation; a `C`
   with no area is SO3, the cavity does not exist), the crossing test
   (S11b), S9. Together the two loops hold every pair of a new segment with
   another: rims and `K'` against `K` in `W`, `K'` and the re-cut `R'`
   against each other in `C`; the pairs of `K` with `R` are the receiver's
   own.
3. Area identity, exact rational over the recorded floats:
   `area(P) = area(W) + area(C)` inward, `area(O) = area(W) + area(P)`
   outward; a sum that differs is SO5. An arc enters as its chord: every arc
   is a join of `K'`, which `W` and the offset region both walk, so its bulge
   stands on both sides of the identity and cancels.
4. The engine's own misses (SO5), modify §5's audit on every planar face the
   engine writes (as `stacked_union_brep.go` runs it), `falsifyBrepPayload`,
   `brepTopologyContext`'s closure count, and `evalBrepContext`'s positive
   volume (`ErrDegenerate`).

## 5. Table SO — refusals and gate order

Modify §1's test picks every sentinel: a body that does not exist is
`ErrDegenerate`; one this evaluator does not build is `ErrUnsupported`.

| SO | Call | Exists? | Sentinel |
|---|---|---|---|
| **SO1** | an end of `K` at a smooth or cusped corner (`|cross| ≤ tol`), or a cut whose carriers meet nowhere in the entering direction (a line `r` parallel to a line `k`, an arc `r` turning away from the band or re-crossing `carrier(k)` first) | the body has a zero-thickness wall edge at `v`, or no inner body closes at `v` without a surface this record does not hold | `ErrUnsupported` |
| **SO2** | a forward cut beyond the far end of `r` (`|r| ≤ t/sin θ` for lines; the sweep to `q` reaching `r`'s own) | yes; the inner body's boundary there runs on `r`'s neighbour's carrier, a trimmed-offset construction this evaluator does not build | `ErrUnsupported` |
| **SO3** | `C` encloses no area (S8 on `C`), or inward `h − k·t ≤ 0` for `k` kept caps (reach SX11) | no cavity | `ErrDegenerate` |
| **SO4** | a cut, a join or a level whose displacement the enclosure cannot bound (`offset2d.ErrUnbounded`) | yes; its readings would carry no bound | `ErrUnsupported` |
| **SO5** | the engine misses the record (`ErrStackedWallMiss`: two circles meeting at distinct walked ends, a crossing too near a circle's centre to key — an offset arc join's tangent junction among them (§4.3) — a wall plane whose loops are neither all outers nor one outer with holes), the area identity fails, or the closure count fails | yes; the record cannot be stated | `ErrUnsupported` |
| **SO6** | a side opening on a holed section; a run that is not one proper connected run of whole walks of the outer loop (a face of a hole loop, a run covering part of a coalesced walk, every side face) | yes | `ErrUnsupported` (reach SX8's text) |
| **SO7** | a side opening on a brep or stacked receiver through route P | yes; route P maps removed faces to the recognised prism's caps only | brep-modify SB3, unchanged |

Gate order, after modify §4's stage 1 and reach SX10:

| Stage | Gates |
|---|---|
| 2. receiver | S3; `requireExactSection`; the removed faces: caps by role, side faces by `side(0,j)`; SO6 |
| 3. levels | SO3's height half (inward, kept caps) |
| 4. chain | `offsetOpenChain` over `K`: S11a per walk, SO1/SO2/SO4 per end |
| 5. regions | §4.7 steps 2–3: S8 (SO3's section half), S11b, S9, the area identity |
| 6. record | both caps removed → `evalPrismContext` over `W`; else the engine, §4.7 step 4 |

No section limit (modify S10, S18) runs: the cavity opens through the removed
faces, so the inradius of `P` bounds nothing. A cavity that closes by
consuming walks reports S11a, as the revolve side opening does today; SO3's
section half fires only on a chain that survives and encloses nothing.

## 6. Table BO — results and roles

| BO | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BO1** | side run, both caps removed, either sense | `prismPayload` over `W` on `[z0, z1]` | modify B1's | `side(0,j)`, `capStart`/`capEnd` |
| **BO2** | side run, one or both caps kept, either sense | `brepPayload`, `stack` nil | one lump, one shell; the removed face's carrier as §4.2 states | `face(k)`/`wall(k)` |
| **BO3** | revolve side opening with a slanted or circular rim (§8) | `revolvePayload` over the wall region | reach BX6/BX7's | reach BX6/BX7's |

Reach Table BX row BX4 reads BO1/BO2; reach §9.3.2's rim restriction reads
BO3.

## 7. Table DO — downstream

A BO1 result is a prism and modify Table D covers it. A BO3 result is a
revolve and reach Table DX's revolve column covers it; where its wall carries
a nonzero `sectionDelta` (§8), every reading charges it as reach §9.3.1 and
`docs/surface-intersection-design.md` §7.2 state, the clearance, wall,
minimum-radius and undercut surveys read `Suspect`, and a second shell or a
junction blend refuses. A BO2 result is an
ordinary `brepPayload`, and general-boolean §4.5 reads it unchanged:

| DO | Consumer | BO2 |
|---|---|---|
| **DO1** | `Volume`, `Area`, `Centroid`, `Bounds` | general-boolean §4.3's face sums; `Exact` when every face is planar with every coordinate a float and `delta = 0` |
| **DO2** | `Verify` validity, tolerance gate | by construction; the brep's witness reader |
| **DO3** | `Tessellate`, STL/OBJ/3MF | general-boolean §4.4; a planar face with a hole chords through the cap path's hole bridging |
| **DO4** | mesh boolean | class B over the face view where it admits (an oblique rim misses B6); else the mesh path |
| **DO5** | `ThroughAll`/`ToFace` | the per-face extremes; refuses a record with `delta > 0`, as a prism's does |
| **DO6** | clearance | `addBrepFaces`; no model when `delta > 0` |
| **DO7** | undercut | per face: a planar face's one outward normal, a swept face's walk range |
| **DO8** | concave radius | the cavity's concave arcs are clockwise arcs with the material on their left and are read; undecided when `delta > 0` |
| **DO9** | wall survey | staged `Suspect` (`DiagUnsupportedSurveyPayload`), as for every brep |
| **DO10** | STEP | analytic where every wall is a plane or a cylinder (the oblique four-face plane writes as four planar faces) |
| **DO11** | `Fillet`/`Chamfer`/`Shell` on the result | brep-modify Tables RB/EB/SB over the record: RB1 needs `delta = 0`, so a rectilinear exact result takes route E on any axis-parallel straight edge (the opening's edges included: each hole corner is incident to three faces); route P applies only where the result reads as a prism along a reference axis, which a U-channel with kept caps does not; `Shell` again is SB10. A result with `delta > 0` is SB1 |
| **DO12** | `Placed`/`Mirrored`/`PatternCopies` | re-lifts every face frame; a reflection flips `outward` and every wall's winding |

## 8. The revolve's slanted rim

A revolve side opening (reach §9.3.2) closes each opening end by Table RO:
`offsetOpenChain`'s opening end calls `OpeningJoin`, and the wall
region's rim is the piece of `carrier(r)` from `v` to `q` — a `LineSeg`
sweeping a cone or a plane, or an `ArcSeg` about `r`'s centre sweeping a
torus or a sphere, all surfaces `evalRevolve` builds from any line/arc
meridian. The wall region's loop is §3's `W`, inward and outward alike, and
it passes the §5 audit (S8, S11b, S9) and the axis gates as today.

The rule carries over without change because the meridian plane is a section
plane: the kept walk's offset is modify §8's, the removed walk's carrier is
its own record, and the band and the entering direction read as in §2.1. Two
things differ from the prism and are stated here:

- the exact-pair rule keeps today's right-angle feet bit for bit (§2.4), so
  every body §9.3.2 builds today is unchanged;
- the displacement is the whole wall's, not a face's: reach §9.3.1's
  `openChainSectionDelta` encloses every interior miter, every axis join and
  each opening's rim cut (`OpeningReach`, §2.4) and publishes three times the
  largest reach as the revolve's `sectionDelta`, with `sectionWhole` set,
  only where it is nonzero. A cut whose enclosure is the held float — a
  right-angle rim on axis-aligned walks, the cone fixture's `(6.5, 4)` —
  adds nothing, so those bodies stay bit for bit undisplaced. The prism route
  charges the same reach per face (§4.4).

SO1 and SO2 are the revolve's opening-end refusals; an opening end adjacent to
the axis walk never arises (the run touches the axis walk, reach §9.3.2).

## 9. Required tests

Every test asserts computed geometry against a closed form; bounds are
relations (`|value − closed form| ≤ Bound`), never literals; every new bound
fixture is shown to fail first by deleting the leg it guards. Tests of the
exported API live in `apitest/shell_opening_test.go` and
`apitest/revolve_shell_side_test.go`; record-level tests in
`shell_opening_internal_test.go` and `internal/offset2d/opening_test.go`.

**U-channel** (PR 2). Box `x∈[0,40]`, `y∈[0,20]`, `z∈[0,10]`, `t = 2`, the
`x = 0` face removed:

- inward, both caps kept: cavity `(0,38)×(2,18)×(2,8)`, volume
  `8000 − 3648 = 4352`, area `2704 + 1768 = 4472`, centroid
  `(1417/68, 10, 5)`, 11 faces, the `x = 0` face one planar face with one
  hole `y∈[2,18]`, `z∈[2,8]`, `Exact`, `delta = 0`, `Shell.IsVoid()` false,
  one lump;
- the `z = 10` cap removed too: cavity height 8, volume `3136`;
- both caps removed: a `prismPayload` over `W`, volume `192·10 = 1920`,
  `capStart`/`capEnd` present;
- outward, both caps kept: `K'` rounds the two convex corners beyond
  `x = 40` to arcs of radius 2 tangent to their neighbours, SO5 (§4.3); with
  both caps removed the same wall is BO1's prism, volume
  `(208 − 2(4 − π))·10`;
- outward, both caps kept, the `x = 0`, `y = 20` and `x = 40` faces removed:
  the one kept wall `y = 0` cuts backward at `(0, −2)` and `(40, −2)`,
  `O = [0,40]×[−2,20]`, volume `880·14 − 8000 = 4320`, 10 faces, `Exact`;
- `t = 10` inward → SO3's height half; `t = 10` with both caps removed →
  S11a (the far wall's offset is consumed); `t = 5` with both caps removed
  builds, the far wall keeping 10 of its 20: volume `(800 − 350)·10 = 4500`;
  removing the `x = 0` and `x = 40` faces → SO6 (two runs); every side face →
  SO6; `Verify` `Sound`, STEP analytic, the mesh's occupied-volume proof
  covering `4352`.

**L prism opened on one leg** (PR 2). `P = (0,0),(30,0),(30,10),(10,10),
(10,30),(0,30)`, height 10, `t = 2`, the `x = 10` face (`(10,10)→(10,30)`)
removed, inward, both caps kept:

- the reflex end at `(10,10)`: `q = (10, 8)`, the rim `(10,8)→(10,10)` with
  outward normal `−x`; the convex end at `(10,30)`: `q = (10, 28)`;
- `C = (10,28),(2,28),(2,2),(28,2),(28,8),(10,8)`, area 316; volume
  `5000 − 316·6 = 3104`; area `2092 + 1148 = 3240`; 16 faces; the `x = 10`
  plane holds exactly two faces, a U with normal `+x` and the rim rectangle
  with normal `−x`; `Exact`;
- route E on the result: chamfer the outer edge along `z` at `(0,0)`,
  `d = 1`: `Exact` `3104 − 5`; `Shell` on the result → SB10.

**Acute corner** (PR 4). Triangle `(0,0),(12,0),(0,9)`, height 10,
`t = 1.5`, the `y = 0` leg removed, inward, both caps kept: the acute end at
`(12,0)` cuts at `q = (9.5, 0)`, the right-angle end at `(0,0)` at
`(1.5, 0)`; `C = (1.5,0),(9.5,0),(1.5,6)`, area 24; volume
`540 − 24·7 = 372`; the `y = 0` face a planar face with hole `x∈[1.5,9.5]`,
`z∈[1.5,8.5]`; the hypotenuse wall and its offset are swept faces; `delta`
encloses the offset's distance from the exact line `3x + 4y = 28.5`.

**Obtuse corner** (PR 4). Trapezoid `(0,0),(14,0),(11,4),(3,4)`, height 10,
`t = 1`, the `y = 4` side removed: cuts at `(4.25, 4)` and `(9.75, 4)`;
`C = (2,1),(12,1),(9.75,4),(4.25,4)`, area `23.25`; volume
`440 − 23.25·8 = 254`.

**Oblique removed face** (PR 4). The triangle above with the hypotenuse
removed, `t = 3`: cuts `(8, 3)` and `(3, 6.75)`; `C = (3,3),(8,3),(3,6.75)`,
area `9.375`; volume `540 − 9.375·4 = 502.5`; the hypotenuse plane holds four
swept faces whose shared edges pair; the cap loops hold the hypotenuse as
three collinear segments.

**Arcs** (PR 3). The D section — the semicircle of radius 5 about the origin
through `(5,0)` from `(0,−5)` to `(0,5)`, then the chord `x = 0` — height
10, both caps kept:

- chord removed, `t = 1`: `C` is the half-disc of radius 4, volume
  `125π − 8π·8 = 61π`, the `x = 0` face a planar face with hole `y∈[−4,4]`,
  `z∈[1,9]`, the cuts `(0, ±4)` exact;
- arc removed, `t = 3`: cuts `(3, ±4)`, the rims arcs about the origin from
  `(0,∓5)` to `(3,∓4)`, each column one swept face over `[0,10]` with
  side-line splits at `z = 3` and `z = 7`; `C` the circular segment beyond
  `x = 3`, area `25·acos(3/5) − 12`; volume `125π − 4·(25·acos(3/5) − 12)`
  within its bound;
- two consecutive arcs at an end corner → SO5 until PR 5, then builds.

**Revolve slanted rim** (PR 1). Meridian `(ρ,z) = (0,0),(8,0),(8,2),(5,6),
(0,6)`, a full turn, `t = 1.5`, the cone `(8,2)→(5,6)` and the top disc
removed: the cut `(6.5, 4)` on the cone's line, the rim a cone frustum between
radii 8 and 6.5; volume `300π − 172.125π = 1023π/8`. Outward at `t = 1.5` the
cone's line walked back from `(8,2)` reaches `ρ = 9.5` at `z = 0`, the end of
the corner arc about `(8,0)`, so the trimmed cylinder offset has no length
left (S11a); outward at `t = 0.75` the cut is `(8.75, 1)` and the wall's
`∫ρ dA` is `24 + 0.140625 + 6.28125 + 3.09375 + 1.125π`. The right-angle
fixtures of `TestRevolveShellSideOpening` unchanged, and `OpeningJoin`
reproducing the right-angle foot bit for bit; the top disc removed alone,
`t = 1` (the obtuse end at `(5,6)` against the kept cone): `q = (3.75, 6)`,
the cavity `(0,1),(7,1),(7,5/3),(3.75,6),(0,6)` a cylinder under a frustum,
volume `300π − 1455.0625π/9 = 19919π/144`. The cone `(ρ,z) = (0,0),(10,0),
(0,20)` without its base disk, `t = 1`: the acute end cuts the base plane at
`ρ = (20 − √5)/2`, the wall's `∫ρ dA` is `(8000 − L³)/24` with `L = 20 − √5`.
The bead — the chord `ρ = 6` from `z = 0` to `10` under the arc of radius 5
about `(z,ρ) = (5,6)` — without its arc, `t = 1`: the cuts `(5 ± √24, 7)`
inward and `(5 ± √24, 5)` outward, the rims torus bands about the arc's own
centre, the wall's `∫ρ dA` `6(√24 + 25·asin 0.2) ± 2(125 − 24√24)/3`. Each float
cut — the `5/3` miter, the `√5` acute cut, the bead's `√24` cuts — sweeps a
seam vertex whose position bound is nonzero and encloses the closed form,
while the cone fixture's exact `(6.5, 4)` cut and every right-angle rim on
axis-aligned walks publish a zero `sectionDelta`. The
cone shorter than its own cut (`t = 3.5`) → SO2; a ridge of two slants
filleted and the fillet removed, so both ends of the kept chain are smooth →
SO1.

**Record and refusal fixtures** (every PR). `OpeningJoin` on each row of
§2.3 against hand-derived points; the exact-pair rule reproducing the foot
bit for bit; `ChainReach`'s enclosure containing the exact rational cut for
the triangle and trapezoid families over a range of `t`; every refusal
leaving the receiver live and the document unchanged; `.github/test-shards.txt`
carrying every new name and `go test . ./apitest/ -run '^TestCI'` passing.

## 10. Do not do this

- **Close the wall with a normal segment.** It is Table RO at one corner kind
  and a wrong body at every other (§2.2).
- **Compute the rim with equal offsets and a rolling ball.** A removed face is
  offset by zero. The arc join of modify §8 belongs to two kept walks at a
  reflex corner; a rim is always the carrier intersection.
- **Pick the root nearest `v`.** `Intersect`'s nearest-root rule is a miter's;
  a strongly curved `r` can put the wrong root nearer. The entering direction
  picks the rim's root (§2.4).
- **Subtract a closed cavity.** A scene of `P` against a closed `C` would
  fabricate a face across the opening, and its cut vertices on `P`'s walks are
  `Partial` fragments decad can only reject (§4.6).
- **Record the ring on an oblique or circular carrier as one face.** The record
  holds planar faces on reference planes only and swept faces without holes;
  the pieces are separate faces whose edges pair (§4.2).
- **Let a stacked column carry a wall across the cap and wall slabs.** The loop
  record changes between them; the engine's per-carrier restatement is what
  joins the pieces into one face.
- **Fall back silently on an engine miss.** `stacked_union_brep.go` misses
  into the mesh path because a boolean has one; a shell has none and refuses
  with SO5.
- **Charge the cut only on the rim face.** The payload-wide `delta` is what
  every consumer reads; a cut vertex belongs to the rim, the cavity wall and
  the cavity floor alike.
- **Admit a cut because the residual against `carrier(r)` is small.** The cut
  is decad's own closed form; its displacement is enclosed, never measured.

## 11. Decided questions

- **Payload for the prism side opening**: `brepPayload` through
  `internal/stackedbrep`, as approved; `prismPayload` where both caps are
  removed, since the result then is a prism (§4.1).
- **Roles**: the brep's `face(k)`/`wall(k)` alone, as route E's results carry
  (brep-modify BB4). No shell-specific second role is minted: a caller selects
  by geometry, and no consumer reads a rim role.
- **Section limit**: none for a side opening; S11a and SO3 decide existence
  (§5).
- **Smooth end corners**: refused (SO1), not built as a zero-thickness edge.
- **Displacement**: charged on both routes: per face on the prism route
  (`brepFace.delta`), and on the revolve route as the wall's whole-section
  `sectionDelta`, published only where nonzero (§8, reach §9.3.1).
- **Side openings on brep and stacked receivers**: not in this design (SO7).
  Mapping a brep face to a segment of the recognised prism's section is a
  change to brep-modify §4.2 and ships, if at all, there.
- **Hand-offs**: none. No capability is needed from `sketch`, `r3` or `units`.

## 12. PR split

Each PR ships code, tests and the documentation updates for every refusal it
lifts: reach §14's increment table, `doc.go`'s support map,
`docs/missing-features.md`, the function doc comments, and the layout rows of
every new root file. This document ships with PR 1.

| PR | Lands | Files and functions | Proves | After |
|---|---|---|---|---|
| **1** (landed) | `offset2d.OpeningJoin` (§2.4) with its exact pairs and root choice; `offsetOpenChain` moved to `shell_chain.go` with an end-kind parameter; the revolve slanted rim (§8): `revolveShellSideWall` writes the rim from `OpeningJoin`, `requireOpeningRim` deleted; `offset2d.OpeningReach` charging each rim cut through `ChainReach` into the wall's `sectionDelta` | `internal/offset2d/opening.go`, `internal/offset2d/reach.go`, `shell_chain.go`, `shell_revolve.go`, `apitest/revolve_shell_side_test.go`, `internal/offset2d/opening_test.go`, `shell_chain_internal_test.go` | the revolve fixtures of §9; `OpeningJoin`'s corner rows; right-angle bodies bit for bit; float cuts charged, exact ones not | — |
| **2** (landed) | the prism side opening, rectilinear: `classifyRemovedFaces` (caps by role, sides by `side(0,j)`, SO6), the three regions and the §4.7 audit (`shell_opening.go`), the stack through the engine into a `brepPayload` and the both-caps prism (`shell_opening_brep.go`), the `delta` composition over `offset2d.ChainReach`, `brepgeom.StackedWallRegions` with holes, `Engine.Event`; every non-axis-aligned walk in `K` or `R` refused with SO5's sentinel until PRs 3–4 | `shell_opening.go`, `shell_opening_brep.go`, `shell.go` (S2 replaced by the dispatch), `internal/offset2d/reach.go`, `internal/brepgeom/stacked_wall.go`, `internal/stackedbrep/record.go`, `apitest/shell_opening_test.go`, `shell_opening_internal_test.go` | the U-channel and L fixtures of §9, every sense and cap variant, DO11's route E on the result | 1 |
| **3** | circular walks in `K` and `R`: line–arc and arc–line cuts through `ChainReach`'s circle enclosures; the refusal of PR 2 narrowed to oblique lines and arc–arc end corners | `shell_opening.go`, `internal/offset2d/opening.go`, `internal/offset2d/reach.go`, tests | the D fixtures of §9 | 2 |
| **4** | oblique straight walks in `K` and `R`: the cap slabs' pre-split record of `r` at forward cuts (§4.2), the acute, obtuse and slanted-reflex corners; the refusal of PR 2 narrowed to arc–arc end corners | `shell_opening.go`, `shell_opening_brep.go`, tests | the triangle and trapezoid fixtures of §9 | 2 |
| **5** | the engine's circle–circle junction at one recorded point (§4.3); arc–arc interior corners of `K` and arc–arc end corners | `internal/stackedbrep/record.go`, `shell_opening.go`, tests | two consecutive arcs at an end corner build; distinct walked ends still miss | 3 |

PRs 3 and 4 run in parallel: they share PR 2's files but touch disjoint
functions (the circle cases of `OpeningJoin`/`ChainReach` against the cap
pre-split and the oblique refusal), and each narrows PR 2's one refusal
message on its own branch. PR 5 follows 3.

Increment table — what still refuses after each PR:

| After | Still refused |
|---|---|
| 1 | every prism side opening (base S2) |
| 2 | a prism side opening with any circular or oblique walk (SO5's sentinel); an offset arc join under a kept cap (SO5, §4.3); the revolve rows of reach SX8 |
| 3 | oblique walks; arc–arc end corners; an offset arc join under a kept cap |
| 4 | arc–arc end corners and an offset arc join under a kept cap (SO5) |
| 5 | Table SO alone |
