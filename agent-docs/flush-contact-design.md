# Flush Contact Design

Admit a coplanar contact between two planar faces whose outlines are held within a bound, when both true planes are
proven equal exactly and the two outlines keep the same arrangement under every move their bounds allow: current
behavior (§1), required behavior and definitions (§2), approach (§3), the rule (§4), the face merge anchor (§5), the
chain of Booleans that makes and joins a flush cap (§6), soundness (§7), refusals that stay (§8), performance (§9),
tests (§10), documentation (§11), risks (§12), open questions (§13), tasks (§14).

Companion to `agent-docs/coplanar-contact-design.md` (§3.1 admission, §3.3 in-plane edges, §3.5 bounds, §4 positive
face bounds, §6 gate obligations), `agent-docs/union-face-merge-design.md` (§2, §4.2 rule 3, §8),
`agent-docs/faceted-vertex-bounds-design.md` (§2, §2.1, §3) and `agent-docs/interference-design.md` §5.2. Adds no
public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| Why does a flush cap with a round on it come out `Faceted` and off its plane? | §1 |
| Which coplanar pairs does this admit, exactly? | §2, §4 |
| Which held vertex stands for which true point? | §4.2 |
| What may move, and how far? Which edges are straight? | §4.3 |
| How is "same arrangement" decided? | §4.4 |
| Which mesh-pass sites change? | §4.6 |
| What backs the gate's deferral of a flush pair? | §4.7 |
| What bound does a point on a flush pair get? | §4.8 |
| How does the merged face prove its plane? | §5 |
| Why does the Cut that makes the cap need the same rule? | §6 |
| Why can this never admit a pair whose true contact differs? | §7 |
| Where does each part live? | §14 |

## Assumptions

- "Carrier-exact facet" is coplanar-contact §16.1's notion computed for every op: `SlabBound == 0`, an unplaced
  analytic plane the facet lies in exactly (`analyticPlaneFacet`), or a record naming the face's exact plane
  (`trueFacePlane`, §17). The placed analytic reading (`placedCarriers`, §17.2) is not used (§13 Q2). A face that
  carries a `CapRecord` is read against the plane its record denotes instead of its float tag (§4.1).
- A face is **planar** when its true face is provably a plane: its surface is a `surfacegeom.Plane` or
  `topo.Face.HeldPlanar` holds (a Faceted face descending only from planar sources); it states no axial displacement
  and no extent departure (`AxialDelta`, `ExtentDelta`) unless it states `FlatPatch`; and it is no wall of a chorded
  loft (faceted-source-retention §11 "Chorded loft walls": a `Plane` through three held vertices of a curved ruled
  wall). The loft test is one shared predicate, `booleaneval.chordedLoftWall(owner, face)`, which
  `chordedLoftPlane` also calls; `booleaneval` passes the whole planar test to `meshbool.AttachFlushFaces` as a
  function, because `loftrecord` imports `meshbool`. Flush reads count a facet as carrier-exact only when its face is
  planar in this sense. A Faceted face descending from a chorded loft wall carries `chordedLoftPlane`'s `AxialDelta`,
  so the displacement rule excludes it even where `HeldPlanar` is set.
- Vertex bounds mean what faceted-vertex-bounds §2 says; §4.2 states the correspondence this design reads from them.
- Shifted, grown and fixed-tool attempts admit nothing new (coplanar-contact Assumptions).
- decad tests use neutral minimal fixtures and name no consumer model (`agent-docs/testing.md` "Test scope").

## 1. Current behavior

| Pair | Today |
|---|---|
| host box `[-10,10]×[0,16]×[-10,0]` ∪ tool drawn on `y = 0` (a strip `x ∈ [-1,1]` up to `z = 6` from feet at `z = -2`, 8-chord quarter rounds of radius 0.7 at both roots), extruded 4 toward +y | mesh path, no shift; one face on `y = 0` (union-face-merge R5) |
| the same tool with its top closed by a true arc of radius 1 about `(0, 6)` | the gate defers the coplanar pair, the mesh pass refuses it (`TouchCoplanar`: the cap reads the arc's sagitta), the retry moves the tool: `CodeGapShiftApplied`, every tool face `Faceted`, the cap 0.0019 off `y = 0` |
| that arc tool built as a chain: a longer prism cut back flush to `y = 0` by a box, minus a block whose lower face lies flush, facing down, on the foot tops; then ∪ a host made by an analytic Union of two co-directional prisms, one with a sketch fillet | the end Cut's flush pair is refused the same way and takes a gap shift, so every tongue face carries at least 2.4e-6; the Union with the host is refused and shifted |

Findings:

- **The cause is the admission, not the tessellation.** Both planes are exact. coplanar-contact §3.1 admits a coplanar
  facet pair only when both facets carry bound zero and both faces read zero. A face with a true arc anywhere on its
  outline reads the arc's sagitta, so every facet pair of it is refused, however far the arc is from the contact. The
  gap retry (gap-tolerance §5.2) then moves the operand.
- **Bounds are coarser than the outline near the contact.** On the analytic host every vertex reads the fillet's
  sagitta (no per-vertex record, faceted-vertex-bounds §2.1). On the faceted tongue the far cap's arc sagitta reaches
  the far ends of the long straight edges that run to it, and earlier Cuts leave vertices with rounding bounds near
  `1e-14` on straight edges. A per-facet or per-vertex zero test fails on every one of these faces.
- **The merge cannot fix it afterwards.** A shifted result is never merged (union-face-merge §6), and a face whose
  every vertex carries a bound has no zero-bound facet to fix its plane (§4.2 rule 3 there).

## 2. Correct behavior and definitions

**Rule.** Two planar faces of different operands form a **flush pair** when both are carrier faces in one exact plane
and their outlines keep their arrangement (§4.4). The mesh pass admits every coplanar facet pair of a flush pair as it
admits an exact pair (§4.6), which backs the gate's deferral of the face pair (§4.7), and a flush `Union` keeps every retained
tag, takes no shift, and merges the two coplanar faces (§5).

| Term | Definition |
|---|---|
| Carrier face | A face of one operand whose every facet is carrier-exact and whose every corner lies in its first facet's plane, exactly (`carrierFace`). Its true face lies in that plane. |
| One plane | The three corners of the first facet of one carrier face lie in the first facet's plane of the other (`OrientSign` zero). Facing may be equal or opposite. |
| Outline edge | A held edge of the face whose twin facet belongs to another face of the same operand. |
| Corner vertex | An outline vertex where the outline passes from the boundary with one neighbour face to the boundary with another. |
| Straight edge | An outline edge whose face and twin face are both planar and whose twin face's true plane provably differs from the face's own (§4.3): its true counterpart lies on the line where the two planes meet, so it is a straight segment. |
| Line edge | A straight edge whose true counterpart lies on the held edge's own line: both endpoints have `β == 0`, or the twin face is a carrier face in a plane other than the face's own and both endpoints lie in it exactly (the true boundary lies in both exact planes, so on their exact line). |
| Corner radius `ρ(v)` | How far the true point of outline vertex `v` may lie from `v` (§4.3): `β(v)`, or a strip bound at a corner of a faceted face. |
| Moving part | An endpoint with `ρ > 0` (it may move by `ρ`), or a non-line edge with an endpoint `ρ > 0` (its points may move by the larger endpoint radius). |
| Reach `D(F)` | The largest `ρ` over the outline vertices of `F`; the true outline lies within `D(F)` of the held one. |
| Robust crossing | Two straight edges, one of each face, that cross and keep crossing under every move of their endpoints within their bounds (§4.4). |

**Out of scope:** curved coplanar or tangent contact; faces whose true planes agree only within a bound (coplanar-contact
§18 keeps that refusal); placed analytic bodies read through `placedCarriers` (§13 Q2); a refined surface bound
(`FaceSurface`) as a source of carrier exactness.

**Success criteria.**

- **S1.** The arc-topped tool flush with the host (§1 row 2) unions with no `CodeGapShiftApplied`, no `Faceted` face,
  exactly one face on `y = 0`, and the volume of the exact solid within its bound, in five exact placements.
- **S2.** The chain of §1 row 3 builds with no `CodeGapShiftApplied` at any step and one face on the flush plane.
- **S3.** A tool whose cap lies off the host's plane by `2^-40` or `10^-3` keeps two faces; a tool whose arc crosses the
  host's outline is refused as today (retry in a default document, `BooleanUnsupportedContact` under `ExactOnly`); a
  tool placed by an inexact turn keeps today's shifted result.
- **S4.** Every Boolean that succeeds today still succeeds; an exact pair's result does not change.
- **S5.** The bounded replay admits a subset of the pairs the live pass admits; a pair it does not admit is a decline.

## 3. Approach

Decide flushness once per face pair, from data both prepared operands already hold, before the gate runs:

1. Read carrier faces and their planes (§4.1).
2. Read each carrier face's outline: its edges, which are straight and which are line edges, and its vertex bounds
   (§4.2, §4.3).
3. For a pair of carrier faces of the two operands in one plane, decide whether the outlines keep their arrangement
   (§4.4), lazily, memoised per face pair.
4. Admit coplanar facet pairs of flush pairs in the mesh pass (§4.6), which backs the gate's coplanar deferral (§4.7),
   and give every point the coplanar pass places on a flush pair a bound that covers both outlines (§4.8).
5. Let the face merge fix a group's plane from an exact retained plane tag as well as from a zero-bound facet (§5).

**Reject-only.** Admission rests on exact plane equality (exact signs and zeros) and on proven lower bounds of
distances and orientations that exceed proven upper bounds of displacement, as the gate's far skip does. No residual
admits anything (CLAUDE.md "A decad-side check may only FALSIFY").

**Not a `sketch` question.** The test reads decad's own held edges and their proven bounds; `sketch` has no
padded-clearance or perturbation-robust crossing query, and a scene per face pair would cost a scene build per
examined pair for no gain in exactness. The reason is coplanar-contact §3's, which computes its clips itself.

**Alternatives considered.**

- *Faces whose planes agree within a bound.* Rejected by coplanar-contact §18.2: a tolerance answer.
- *Per-facet or per-vertex zero test.* Rejected: §1's findings show the real faces carry bounds that come from parts far
  from the contact.
- *Admit only when every moving part is far from the other FACE.* Rejected: a tongue's foot lies inside the host face,
  and the end Cut's foot-top edges cross the block. Only the other face's outline can change the arrangement.
- *Forbid every crossing of a moving part (clearance only).* Rejected: the foot-top edges of §6 step 2 run from an
  exact end to an end that reads the far cap's sagitta, and they cross the block's outline. The crossing is robust by
  millimetres; §4.4 admits it with its bound.
- *A planar record for the BRep tessellator.* Not needed: line edges cover straight rims between carrier faces (§13 Q1).
- *Prefer an unshifted result at another chord before a shifted one.* Not needed once the Cut's flush pair is admitted.

## 4. The rule

### 4.1 Carrier faces

`meshbool.FlushFaces` holds, per operand, each face's facets (by `BoolMesh.Src`), whether it is planar and a carrier
face, and its first facet. Carrier-exactness per facet comes from `carrierFacets(m, placed)`, factored out of
`SetSeats` (lines 46–63), which `SetSeats` then calls unchanged; flush reads pass `placed = nil` and also require the
facet's face to be planar (Assumptions), which keeps a chorded loft wall, a single held triangle on a curved surface,
from reading as a carrier face. A face is read lazily.

A face that carries a `CapRecord` (an analytic cap, or a Faceted group that retained one) is a carrier face only when
every facet corner also lies exactly in the plane its record denotes: `{B(O + u·U + v·V + z·N) + t}` with `B`, `t` the
record's `Xform`, every held value an exact rational, and `z` the denoted level `ZDenoted`, or the held level `Z` when
the face states a zero `AxialDelta` (`capRecordPlane`). The float `Plane` tag is a rounded re-expression of that
plane: where `O + Z·N` rounds, the tag and the held corners sit off the true cap while the face still states a zero
displacement against its record, so `analyticPlaneFacet` alone would read a cap the true face does not lie in. Such a
cap is no carrier face, and its coplanar contact keeps today's refusal and shift (§8). A face without a record keeps
the `carrierFacets` reading; `SetSeats` is unchanged.

### 4.2 Vertex correspondence

The rule reads one property of vertex bounds, stated here as an invariant of every producer it reads:

> **End correspondence.** Every held outline vertex `v` stands for one point `v*` of the true outline with
> `|v − v*| ≤ ρ(v)` (§4.3), and the held outline edge `(a, b)` stands for the true outline arc from `a*` to `b*`. At a corner
> vertex, `v*` is the true corner.

Why each producer keeps it:

- **Analytic payloads (prism, BRep, cup, and every chorded analytic mesh).** A vertex that ends a rim between two
  planar faces is a record vertex: a section vertex or a cap–wall corner, which the record states exactly (a sketch
  fillet's tangent point is one shared record point of the line and the arc). Its true point is that record point,
  displaced only by the coordinate store, section, axial and placement terms that its faces' bounds already charge,
  so `β(v) ≥ |v − v*|` under §2.1's reading. An interior chord sample is no end of a rim between two planar faces.
- **Faceted payloads (faceted-vertex-bounds §3).** A kept operand vertex keeps its point and bound (§3.1). A vertex
  placed on a held facet edge (§3.3), and every vertex in the middle of an outline chain along one neighbour face,
  stands for the affine point of that edge's true counterpart, within its `β`. A rim vertex at a corner is weaker:
  each of its two rim segments' true curves lies within that segment's tube width of the held segment (§3.2), and the
  vertex takes the larger width, but the true corner is where the two true curves meet, which can lie up to
  `(w₁ + w₂)/sin φ` away (`φ` the angle of the two held edges). §4.3 gives such corners that radius instead of `β`.
- **Placement and similarity** map `v*` with the body (faceted-vertex-bounds §4) and charge the rounding in `β`.

So the invariant holds with `ρ(v)` in place of `β(v)`, where §4.3 defines `ρ`. A producer that cannot state this does
not enter flush admission: its faces read as non-carrier. Every carrier face today comes from the producers above
(`carrierFacets` admits analytic planes, retained planes, and record planes). The analytic terms are the ones
tessellation-design §2's `sourceBound(face)` row charges ("every current trim/chording, coordinate-construction, and
placement displacement"), which §2.1's reading folds into `β`.

### 4.3 What may move

For a carrier face `F` in plane `P`, walk every facet of `F` and edge `k` with the operand's twin lookup (`TwinFacet`
live; `twinFacetReplay` in the bounded pass, charged as there). A missing twin is a broken mesh in the live pass
(`ErrBooleanFailed`) and a decline in the bounded pass. For an outline edge `(a, b)` with twin face `G`:

- it is **straight** when `F` and `G` are planar and `G`'s true plane provably differs from `P`: `G` is a carrier face
  with a corner of its first facet off `P` (exact sign), or `G`'s `Plane` surface has a unit normal farther than
  `NormalBound + TagDeparture`, padded by a relative `1e-9` and `1e-12`, from both `n` and `−n`, where `n` is `P`'s
  unit normal (a proven departure bound against a larger float distance, as the gate's far skip). A seam with a
  coplanar neighbour, or a `HeldPlanar` neighbour without such a tag, is not straight;
- it is a **line edge** when both endpoints have `β == 0`, or `G` is a carrier face, some corner of `G`'s first facet
  lies off `P`, and `a`, `b` lie in `G`'s first facet's plane (exact signs).

Each outline vertex `v` gets a radius `ρ(v)`:

- `0` when `β(v) == 0`;
- `β(v)` when the operand's vertex bounds are §2.1's reading (no per-vertex record: an analytic payload), or when both
  outline edges at `v` border the same neighbour face (a vertex in the middle of a chain);
- otherwise (a corner of a faceted face) `ρ(v) = √3·(w₁ + w₂)/sin φ_lo`, where `w_i` is `0` for a line edge and the
  larger endpoint `β` of edge `i` otherwise, and `sin φ_lo` is the projected exact cross product of the two edges'
  directions over their rounded-up projected lengths, rounded down; two line edges on different lines give `0`; a
  zero or non-finite `sin φ_lo` gives `+Inf`, which refuses every pair the face is in.

By §4.2, a line edge's true counterpart lies on its held line between `a*` and `b*`; a straight edge's is the segment
`[a*, b*]`; any other edge's lies within the larger endpoint radius of it (faceted-vertex-bounds §2: a chord's
endpoints carry its sagitta, by §2.1's reading for analytic meshes and by §3.1–§3.3 through each Boolean, and §4.8
keeps it for points this design places). `flushface` reads `RA`, `RB` as the endpoints' `ρ`. `D(F)` is the largest
`ρ`.

### 4.4 Same arrangement

`internal/flushface` decides on plain data (`Edge{A, B r3.Vec; RA, RB float64; Line, Straight bool}` per outline edge,
and the projection axis: the largest component of the plane's float normal). For faces `X` and `Y`, every moving part
`m` of `X`, with radius `r` (an endpoint's `ρ`, or a non-line edge's larger endpoint radius), is tested against every
outline edge `e` of `Y` whose box comes within `pad = upRound((r + D(Y))·(1 + 1e-9))` of `m`'s box:

1. Boxes that stay apart by more than `pad` on some axis are clear. The gap is the rounded difference stepped one float
   toward minus infinity, so it bounds the exact gap from below.
2. Otherwise `m` must be a non-line straight edge `s`, `e` must be straight, and `s`, `e` must cross robustly:
   projected to the plane that drops the axis, each orientation `orient(c, d, a)`, `orient(c, d, b)`, `orient(a, b, c)`,
   `orient(a, b, d)` (exact rationals) has the opposite sign of its partner, and its magnitude exceeds the largest
   change the endpoint moves can make. Orientation is bilinear, so moving `p`, `q`, `r` by `εp`, `εq`, `εr` changes
   `orient(p, q, r)` by at most `|q − p|·(εr + εp) + (εq + εp)·|r − p| + (εq + εp)·(εr + εp)` (lengths rounded up).
   Against a line edge only the tested point moves off its line. Sliding `c` and `d` along that line scales
   `orient(c, d, ·)` by a positive factor, or flips every such sign together if they swap, which leaves the
   opposite-sign test unchanged; the tested point's move changes it by at most `|q − p|·εr`.
3. Anything else is not clear.

The test runs the items of `X` against `Y`, then of `Y` against `X`; the pair is flush when every test passes. It also
returns `Cross`: the largest `(w_s + w_e)/sin θ · √3` over the robust crossings, where `w` is an edge's larger endpoint
radius `ρ` (zero for a line edge; `ρ ≥ β`, and a strip's half-width needs only the chord invariant of §4.3), `sin θ ≥ |cross(b − a, d − c)| / (|b − a|·|d − c|)` from the projected cross product
(rounded down), and `√3` covers the projection's largest shrink of a distance in the plane. The axis comes from a float
normal, so its exact component may fall a hair under `1/√3`; the relative `1e-9` of every pad covers that. It steps the caller's
budget once per box comparison and once per crossing test, and uses no meshbool helper (meshbool imports it).

### 4.5 Where flush pairs are computed

`booleaneval.evaluate` attaches one `FlushFaces` to both prepared operands right after `SetExactFacets`, on exactly
the attempts that call it: the unshifted, unfixed live attempt (charging `proofbound.NewWorkBudget(ctx)`) and the
unshifted, unfixed bounded replay (charging the replay meter: one unit per facet read for carrier faces and outlines,
one per exact sign, one per box or crossing test, and the records the memo holds). The memo is keyed by the unordered
pair of global source ids, so a gate call with the operands swapped reads the same verdict. A face pair is decided
only after a cheap exact check that the two faces' first facets share a plane.

`flushPair(mx, my, i, j) (bool, error)` reads the memo; `pairExact(ma, mb, i, j) (bool, error)` is
`isExact(i) && isExact(j)` or `flushPair`. Every caller propagates the error; an error never reads as "not flush".

### 4.6 Admission in the mesh pass

| Site | Live (`boolean_pair_cuts.go`) | Bounded (`pair_cuts.go`) | Rule |
|---|---|---|---|
| coplanar region pair (coplanar-contact §3.2) | `admitted :=` | `admitted :=` | `pairExact(i, j)` |
| coplanar point touch | `pointCoplanar` | `pointCoplanar` | `c.Coplanar && pairExact(i, j)` |
| in-plane edge of `ma` (§3.3) | `EdgeCoplanarBounded` branch | the same | today's exact rule, or `flushPair(adjacent, p.J)` with `adjacent` recorded with `p.J` in `cp.coplanarA` |
| in-plane edge of `mb` | the same | the same | today's exact rule, or `flushPair(p.I, adjacent)` recorded in `cp.coplanarA[p.I]` |
| isolated point (`seatPoint`, bounded `pair_cuts.go:781`) | unchanged | unchanged | `pairExact(i, j)` replaces the exact pair; a point between a wall facet and a flush face stays a refusal (§8) |

The in-plane edge case reads `adjacent`, the coplanar neighbour `EdgeSide` returns, because the edge's own facet is a
wall that does not lie in the plane: the wall's rim is an outline edge of the flush face, which §4.4 covers, and the
gate still examines the wall's surface off the plane. `Exact` and every reader of `isExact` alone (`seat.go` wall
exactness) are unchanged. A coplanar facet pair whose faces are not flush is refused as today.

### 4.7 Proximity gate

No gate code changes. The gather defers a face pair on its first coplanar region (interference-design §5.2), which
needs a backing: the mesh pass either classifies the pair exactly or refuses it. A face pair with a positive summed
reading was backed by the refusal; it is now backed by the refusal or, for a flush pair, by the flush admission (F5). A
flush pair whose held facets never overlap (a coplanar near miss) is examined by the depth test as today, and may be
refused there. The occupancy re-run (`composeOccupancy`) is unchanged: it already drops its record whenever the mesh
pass admitted a coplanar point (`Stats.Coplanar != nil`). Walls whose rims lie on a flush face keep entering the gate
and `RefuseCoarseHeldContact` as today.

### 4.8 Bounds of points the coplanar pass places

coplanar-contact §3.5 gives a clipped endpoint the bound of the facet it is cut into, which is zero for an exact pair.
On a flush pair the endpoint lies on a held edge of the partner facet and inside the facet it cuts, and the result's
boundary may follow either outline there. The coplanar pass records, for each clipped endpoint of a flush (non-exact)
pair `(ta, tb)`, the key with `max(FacetBound[ta], FacetBound[tb], Cross)`, keeping the largest over pairs. Keep side
gives a kept corner that is neither an operand corner nor a rim key `max(FacetBound[i], record[key])` (live
`keepSideWith` after `CutCornerBound`; bounded `cutCornerBoundReplay` through its reserved table). An exact pair
records nothing, so its results do not change.

## 5. Face merge anchor

union-face-merge §4.2 rule 3 fixes a group's plane from a facet whose three corners have `β == 0`. A group on a flush
pair can have no such facet. Rule 3 gains a second anchor:

- **Tag anchor.** The group is `Retained` with a `surfacegeom.Plane` surface, `NormalBound == 0`, no axial displacement
  (`!HasAxialDelta || AxialDelta == 0`), and every vertex of every facet lies exactly in the tag's plane (exact dyadic
  dot product with `U × V`, as `planeSlab`).

Such a tag is the face's true plane: `retainedPlaneOf` reads it so, and faceted-source-retention-design's Claim R is
why a retained tag without departure is the true surface. With every held facet in it, rule 4 holds and union-face-merge
§8 step 1 holds with the tag as the anchor. A zero-bound facet stays an anchor; a group with neither stays apart.

## 6. The chain that makes and joins a flush cap

1. **Cut-back.** A prism started early and cut back by a box whose face lies on the intended plane: the box face
   crosses the prism's walls, no coplanar pair. The new cap is the box's face (exact, retained tag).
2. **End Cut.** A block cut out of the tongue whose lower face lies flush on the foot tops, facing the other way. A foot
   top meets the foot's outer wall along a line edge, but its inner edge borders a polyline round's first chord face,
   a retained plane whose float tag is not exact, so that edge is straight but not a line edge; its far end reads the
   far cap's sagitta. That edge crosses the block's outline at right angles, millimetres from every end: a robust
   crossing, whose bound is the far end's `β` at most. The pair is flush; `Cut` keeps the target's copy of an
   opposite-facing patch (coplanar-contact §3.4) and the tongue takes no shift.
3. **Union with the host.** The host's straight rims are line edges between analytic carrier planes, so only its far
   corners move; its fillet chords lie far from the tongue. The tongue's round chords cross the host's top rim; their
   ends near it are exact, the rim is a line edge, so the crossings are robust. The tongue's moving parts (arc chords,
   far ends, rounding left by step 2) lie hundredths of a millimetre or more from the host's outline. The pair is
   flush, the Union takes no shift, and §5 merges the two faces.

Without step 2's admission every tongue face carries step 2's shift and loses its tag, so step 3 has no carrier face.

## 7. Soundness

Claim: a flush pair's held coplanar contact has the same arrangement as the true one, so classifying it on held facets
gives the true result within the bounds the result carries, and nothing the gate defers is hidden.

- **F1. One true plane.** Each face is a carrier face: every facet's true piece lies in its own plane (coplanar-contact
  §16.1, §17) and every corner lies in the first facet's plane exactly. The pair test puts the other face's corners in
  that plane exactly. Every decision is an exact sign.
- **F2. The true outlines.** By §4.2–§4.3, deform each held outline edge to its true counterpart: a line edge slides
  along its line, a straight edge moves its ends by at most their `ρ` and stays a segment, any other edge's points move
  by at most the larger endpoint bound. Parts of no moving part do not move.
- **F3. Same arrangement.** During the deformation, a moving part of `X` that §4.4 step 1 cleared stays farther than
  `r + D(Y) − r − D(Y) > 0` from `Y`'s moving outline, so it never meets it. A part step 2 admitted is a straight edge
  whose crossing with a straight edge of `Y` keeps all four orientation signs (the bilinear bound covers every
  endpoint move), so the two segments keep crossing exactly once and nothing else of them comes near (two segments meet
  at most once). Exact parts do not move. So the outlines never gain or lose a crossing or a touch, and the regions of
  the plane they bound keep their inside/outside relation. Every crossing or touch the exact predicates of
  coplanar-contact §3 decide is either between non-moving parts, where held equals true, or a robust crossing, whose
  true point lies within `Cross` of the held one (two strips of half-widths `w_s`, `w_e` meet within
  `(w_s + w_e)/sin θ`, and the projection shrinks a distance by at most `√3`).
- **F4. Bounds cover the rest.** The held result differs from the true one only inside moving parts and at robust
  crossings. Operand vertices keep their bound, and §4.8 gives every point the coplanar pass places at least both
  facets' bounds and `Cross`; `δ(t)` bounds every point of a facet.
- **F5. The deferral is backed.** The gate defers a face pair only where nothing a chord could hide exists between
  those two faces (interference-design §5.2). For a flush pair both true faces lie in one plane (F1), so their true
  contact is the overlap of their true regions, which F3 shows the held outlines carry and the mesh pass classifies.
  Every other face pair, and every flush pair the gather does not defer, is examined unchanged.
- **F6. The merge stays exact.** §5 fixes the true plane exactly. The two merged faces are truly adjacent along a
  true seam, which F3 gives (union-face-merge §8 step 3 assumes it); steps 2–8 there hold unchanged.

The bounded replay reads the same rule from the same fields; where its meshes carry coarser data it admits fewer
pairs, and a refusal there is a decline (S5).

## 8. Refusals that stay

| Case | Why |
|---|---|
| A coplanar pair whose faces are not both carrier faces in one exact plane | F1 |
| A cap whose held corners leave the plane its `CapRecord` denotes, such as a cap at a level that rounds | F1, §4.1 |
| A moving part within its radius plus the other reach of the other outline that is not a robust crossing (an arc that crosses or touches the other outline; a rounded vertex on the other outline; a near-parallel crossing) | F3 |
| A point contact between a wall facet and a flush face's partner that is not an exact or flush pair | the isolated-point rule keeps its refusal |
| Faces whose planes agree only within a bound, placed by an inexact turn | coplanar-contact §17.4, §18 |
| A placed analytic body (`placedCarriers`) | §13 Q2 |
| Shifted, grown, fixed-tool attempts | coplanar-contact Assumptions |

## 9. Performance

- Only coplanar region pairs of the mesh pass ask `flushPair`, after the exact pair test. A cheap exact check that the
  first facets share a plane comes first; a face read is linear in its facets; a pair test is linear in moving parts
  times the other outline's edges, after one outline-box test that clears every far part.
- Exact pairs never reach the memo in the mesh pass.
- `BenchmarkClear` (`internal/flushface`) measures a 1000-edge outline against a 1000-edge outline.

## 10. Tests

"Flush check": the result payload is `facetedrecord.Payload`, no `CodeGapShiftApplied`, no `Faceted` face, exactly one
face on the flush plane with one outer loop.

### 10.1 Unit tests

| ID | Package, function | Case | Expect |
|---|---|---|---|
| U1 | `internal/flushface`, `TestClearFarParts` | two square outlines 1 mm apart, every endpoint `β = 1e-3` | clear, `Cross == 0` |
| U2 | `TestClearNearVertex` | an endpoint with `β = 1e-14` on an edge of the other outline; then `1e-14 + D` farther than the pad | not clear; clear |
| U3 | `TestClearRobustCrossing` | a straight edge from `β = 0` to `β = 3e-4` crossing a line edge at right angles mid-length | clear, `Cross ≤ 3e-4·√3·(1+1e-9)` |
| U4 | `TestClearFragileCrossing` | the same crossing at a shallow angle so an orientation falls under its bound; a non-straight edge crossing | not clear; not clear |
| U5 | `TestClearNoMovingParts` | both outlines exact, crossing | clear |
| U6 | `TestClearBudget` | a cancelled context | the context error |
| U7 | `internal/facemerge`, `TestMergeTagAnchor` | base fixture with every vertex `β = 1e-6`, top groups retained planes at `z = 1`; then `NormalBound > 0`; then the tag at `z = 1 + 2^-40` | the top pair merges; then not; then not |
| U8 | `internal/meshbool`, `TestFlushOutline` | hand-built `topo.Mesh`/`BoolMesh` operands (a box; a box whose top is split into two coplanar faces; a box with one corner bound raised, with and without a per-vertex record) | an edge between carrier planes reads as a line edge; the seam between the coplanar top faces reads as not straight; a faceted corner's `ρ` equals `√3·(w₁ + w₂)/sin φ_lo` and an analytic one's equals `β`; reach is the largest `ρ` |
| U9 | `internal/meshbool`, `TestFlushRecordCorner` | a prism with a sketch fillet tangent to a straight side | the tangent vertex's `β` is at least its distance from the record point (red without §2.1's reading) |

### 10.2 Root tests (`flush_contact_test.go`, package `decad`)

| ID | Function | Case | Expect |
|---|---|---|---|
| R1 | `TestFlushContactArcCap` | §1 row 2 in the five placements of union-face-merge R5 | flush check; 14 faces; volume `3200 + 4·(A + π/2)`, `A` the profile's polygon area above `z = 0`, within the bound |
| R2 | `TestFlushContactArcCapExactOnly` | R1 identity in an `ExactOnly` document | succeeds, 14 faces |
| R3 | `TestFlushContactChain` | hull: R1's profile drawn on `y = -1`, extruded 9 toward +y, cut back by a box over `y ∈ [-3, 0]`; tongue: hull minus a block `x ∈ [-3,3]`, `y ∈ [3,3.5]`, `z ∈ [-0.5, 8]` (lower face flush on the foot tops at `z = -0.5`); host: Union of a prism drawn on `y = 0` (profile `x ∈ [-10,10]`, `z ∈ [-10,0]`, corner `(-10,-10)` rounded by a sketch fillet of radius 2) extruded 16 toward +y and a co-directional prism drawn on `y = 8` (profile `x ∈ [-10,10]`, `z ∈ [-10,-5]`) extruded 12 toward +y | no `CodeGapShiftApplied` on the tongue or the result; flush check on `y = 0`; volume equals `V(host) + V(tongue) − V(host ∩ tongue)` within the summed bounds (each read by decad) |
| R4 | `TestFlushContactKeepsApart` | R1's tool on `y = 2^-40` and on `y = 10^-3` | two faces on the two planes, no merged group, volume within bound |
| R5 | `TestFlushContactArcCrossingRefused` | tool profile a circle of radius 1 about `(0, 0)` drawn flush, so its chords cross the host's edge `z = 0` | default document: `CodeGapShiftApplied`; `ExactOnly`: `BooleanUnsupportedContact` |
| R6 | `TestFlushContactTurnedRefused` | R1's tool turned 37° about z and back (30° and back composes to the identity exactly) | `CodeGapShiftApplied`, no merged group |
| R7 | `TestFlushContactCutFlushEnd` | R3's end Cut alone; then the block 2^-40 higher | no shift and no face on `z = -0.5` inside `y ∈ [3, 3.5]`; then a thin slab remains (a face at `z = -0.5 + 2^-40`) |
| R8 | `TestFlushContactChordedLoft` | a chorded circular loft (two circles of radius 2, 3 apart along z) whose start cap states no displacement (`AxialDelta == 0`, `loftrecord.stateCapLevels`) and lies flush on a box face, the box overlapping the loft's foot (`z ∈ [0, 1]`) and its edge crossing the cap 0.5 from its centre | the cap reads as a carrier face and its rim edges as neither line nor straight (the walls are not planar), so the arc chords crossing the box edge refuse the pair: `CodeGapShiftApplied` as today; red without the `chordedLoftWall` exclusion (the pair is admitted) |
| R9 | `TestFlushContactClipBounds` | R1 identity | every result vertex the coplanar pass placed on the flush plane carries `β ≥` the tool cap facets' bound there; the exact R5 of union-face-merge keeps its bounds |
| R11 | `TestFlushContactRoundedCapLevelRefused` | `ExactOnly`: a square tool on the frame through `z = 1e9`, extruded `nextafter(2, 0)`, its end cap held at the blank Loft's upper cap level while its record denotes a level `2^-52` lower; `Cut` | `ErrUnsupported` (the cap is no carrier face, §4.1); red without the `CapRecord` rule (a hole through the cap) |
| R10 | `TestFlushContactReplay` | `booleaneval.EvaluateReplay(ctx, OpUnion, host.n, tool.n, 1, proofbound.NewReplayTrialBudget())` on R1 identity's operands | no error, `Shift == nil`, and the payload's facets on `y = 0` all in source groups of both operands' flush faces (the bounded pass admits the flush pair) |

### 10.3 API test (`apitest/flush_contact_test.go`)

| ID | Case | Expect |
|---|---|---|
| A1 | R1 identity through the public API | `Faces(Facing(-y)).Exactly(1)` is a `Plane`; no operation diagnostic; volume within bound |

### 10.4 Existing tests this changes

Tests that expect a gap shift, a `Faceted` face or a refusal for a flush pair of carrier faces flip to the flush
result; each flip is listed in the PR and must match §2. `TestIntersectSeat`'s seated `Union` stays refused: its two
bore rims coincide, so their chords are moving parts on each other's outline.

### 10.5 Consumer acceptance (gallery; not committed in decad)

With a `go.work` pointing at the decad checkout, the gallery's flag sliders build with no gap shift on the tongue's end
Cut or the tongue Union, and their stem chamfers build.

## 11. Documentation

| File | Change |
|---|---|
| `agent-docs/coplanar-contact-design.md` | §1 out-of-scope line, §4 "Positive face bound", §8 last bullets, §9 items 5–6: flush pairs per this design; §3.1 admission is `pairExact`; §3.3 the in-plane edge flush branch; §3.5 the flush bound; §6 the deferral's flush backing |
| `agent-docs/union-face-merge-design.md` | §2 "Plane-exact", §4.2 rule 3: the tag anchor; §7 "Exact-face reading"; §8 step 1; Q4 resolved |
| `agent-docs/interference-design.md` §5.2 | the deferral is backed by exact classification, a flush admission, or a refusal; Verify's measured interference reads flush pairs the same way |
| `agent-docs/faceted-vertex-bounds-design.md` §2 | the end correspondence of §4.2 as an invariant, with its producers |
| `internal/meshbool/tangency_gate.go` doc comment | the three backings |
| `agent-docs/missing-features.md` | the contact row narrows to pairs that are not flush |
| `docs/api.md` (lines 315–317) | under `ExactOnly`, a coplanar contact of faces with a positive bound is refused unless they are flush |
| `agent-docs/layout/design-docs.md`, `agent-docs/layout/booleans.md` | rows for this document and `internal/flushface/` |

## 12. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| A test or consumer relied on the shifted result of a flush pair | different face count, tags and diagnostics | §10.4; counts are not contract (`api-design.md` §3) |
| §4.8 raises bounds of points on flush pairs | a later gate reads a larger bound | only flush pairs, which today shift by far more |
| Live admits a pair the replay declines | a chain regroup or history replay declines | S5, R10 |
| Verify's measured interference now admits flush pairs | an `Intersect` of flush bodies reads the same as the Boolean | the same rule and proof (F5) |

## 13. Open questions

- **Q1. Planar record for the BRep tessellator?** A (chosen): no; line edges cover straight rims. B: record it as
  `prismtess` does; gains tighter vertex bounds next to fillets; costs a `brepbandmesh` change and its replay.
  Recommendation: A until a case needs it.
- **Q2. Read placed analytic bodies (`placedCarriers`) for flush pairs?** A (chosen): no; it runs only for a live
  `Intersect` and has no bounded reading. B: yes; gains quarter-turned analytic bodies; costs the replay reading.
  Recommendation: A.
- **Q3. Exact segment distance when boxes come within the pad but nothing crosses?** A (chosen): refuse. B: exact
  segment-to-segment distance; gains slanted moving parts near the other outline; costs exact arithmetic per pair.
  Recommendation: A until a case needs it.

## 14. Tasks

Each task is one PR, merged before the next, and passes the Module boundary gate: owning section of this design, input
class "coplanar planar faces of a mesh Boolean's two operands", no new root adapter.

### Task 1: Clearance kernel

- Files: `internal/flushface/clear.go`, `internal/flushface/clear_test.go`,
  `agent-docs/flush-contact-design.md`, `agent-docs/layout/design-docs.md`, `agent-docs/layout/booleans.md`.
- Work: §4.4. Tests U1–U6, `BenchmarkClear`.
- Depends on: nothing.
- Done when: `go test ./internal/flushface/` passes; `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` passes;
  `golangci-lint run ./...` is clean.

### Task 2: Face merge tag anchor

- Files: `internal/facemerge/merge.go`, `internal/facemerge/merge_test.go`, `agent-docs/union-face-merge-design.md`.
- Work: §5. Test U7.
- Depends on: Task 1 (for the design doc).
- Done when: `go test ./internal/facemerge/ && go test . -run 'TestUnionFaceMerge'` pass; lint clean.

### Task 3: Flush faces, live

- Files: `internal/booleaneval/chorded_loft_plane.go` (`chordedLoftWall`),
  `internal/meshbool/flush_faces.go` (new), `seat.go` (`carrierFacets`), `boolean_mesh.go` (`Flush` field,
  keep-side bound), `boolean_pair_cuts.go`, `coplanar_cover.go`, `tangency_gate.go` (doc comment);
  `internal/booleaneval/eval.go` (attach); root `flush_contact_test.go`; `apitest/flush_contact_test.go`;
  `.github/test-shards.txt`, `.github/test-shards-apitest.txt`; the §11 docs except the replay sentences.
- Work: §4.1–§4.8 for the live pass. Tests U8, U9, R1–R9, A1, and every §10.4 flip.
- Depends on: Tasks 1, 2.
- Done when: `go test ./internal/meshbool/ ./internal/booleaneval/ ./internal/facemerge/ ./internal/flushface/`
  pass; `go test . ./apitest/ -run 'FlushContact|UnionFaceMerge|Coplanar|Seat|Contact'` passes;
  `go test . ./apitest/ -run '^TestCI'` passes; lint clean; CI green.

### Task 4: Flush faces, bounded replay

- Files: `internal/meshbool/flush_faces.go` (replay reads), `pair_cuts.go`, `brake_boolean_replay.go`,
  `internal/booleaneval/eval.go` (replay attach); root `flush_contact_test.go` (R10).
- Work: §4.5's replay charges and `twinFacetReplay`; the bounded admission sites of §4.6 and `cutCornerBoundReplay`
  of §4.8.
- Depends on: Task 3.
- Done when: R10 passes; `go test . -run 'FlushContact|BooleanGroupHistory|Replay'` passes; lint clean; CI green.
