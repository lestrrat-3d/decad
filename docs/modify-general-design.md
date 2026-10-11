# Modify General Design

The next reach of `Body.Fillet`, `Body.Chamfer` and `Body.Shell` over
analytic boolean results: a `brepPayload` (`docs/general-boolean-design.md`
§4, "general-boolean §N" below) and a `stackedPrismPayload` through its face
view (`docs/stacked-prism-design.md`). Companion to
`docs/brep-modify-design.md` ("brep-modify §N"), whose routes P and E this
document extends with route S (shell) and route L (complete-loop chamfer),
and whose Table SB rows SB3, SB4, SB5 and SB10 it narrows. `docs/modify-design.md`
("modify §N") keeps the call contracts, the base gates and the §5 audit;
`docs/modify-reach-design.md` ("reach §N") keeps the cap-loop chamfer's
construction and proofs (§8.3, §8.4), which route L reuses.

Six tables are normative:

| Table | Owns | Section |
|---|---|---|
| **SV** | the reference-part survey: what refuses today, with its code | §1 |
| **TC** | the through-cut record route S recognises | §3.1 |
| **SG** | every refusal route S adds, with its sentinel and gate order | §3.4 |
| **LB** | the loop selection route L admits | §4.1 |
| **SL** | every refusal route L adds | §4.4 |
| **BG** / **DG** | result payloads and what every consumer reads | §5 |

## 1. Survey — Table SV

Nine reference parts, each built from operations decad ships today through
the public API, and the fillet, chamfer and shell requests an ordinary Fusion
part makes on them. Every cell was read off the live code with the probe under
`.tmp/probe-modify/` (not tracked). "builds" names the route that built it.

| Part | Receiver | Request | Today |
|---|---|---|---|
| **P1** cross-drilled bar: 40×20×20 box, Ø6 hole along y | brep (class B) | fillet the 4 edges along y | builds (route P) |
| | | fillet one edge along z | builds (route E) |
| | | fillet every straight edge | builds (route V; `docs/vertex-blend-design.md`) |
| | | chamfer every straight edge | SL1 |
| | | chamfer the 8 along x and z, the top and bottom loops | builds (route L) |
| | | fillet the top and bottom loops | builds (route L fillet arm) |
| | | chamfer a hole rim, or both | builds (route P cap loop) |
| | | fillet a hole rim | builds (route L fillet arm) |
| | | shell removing one y wall, or both | builds (route P cup, tube) |
| | | shell removing the top face, or both caps | builds (route S) |
| | | shell removing one x wall, or one y wall with the top or both caps | builds (route S, a wall run) |
| **P2** pocketed plate: 40×40×10, blind 20×10 pocket 5 deep | stacked | fillet the pocket's 4 vertical edges | builds (route E) |
| | | fillet the 4 floor edges, or the 4 mouth edges | builds (route L fillet arm) |
| | | chamfer the 4 mouth edges, or the plate's top loop | builds (route L) |
| | | fillet the plate's 4 vertical edges | builds (route E) |
| | | shell removing the top | builds (§3.1b, for exact separated rectangles) |
| | | shell removing the bottom | SG3 |
| **P3** round boss on a plate: 40×40×10 ∪ Ø10 boss 15 tall | stacked (A1) | fillet the boss root circle | builds (route L fillet arm) |
| | | fillet the boss top rim | builds (route L fillet arm) |
| | | chamfer the boss top rim, or the boss root | builds (route L) |
| | | fillet the plate's vertical edges | builds (route E) |
| | | shell removing the boss top | builds (§3.1c, for an exact nested circle) |
| | | shell removing the bottom | SG3 |
| **P4** drilled plate: 60×40×8 with four Ø5 holes in the sketch | prism | chamfer the hole mouths, the top loop, or both | builds (cap loop) |
| | | fillet the hole mouths, or the top loop | builds (route L fillet arm, RF3) |
| | | fillet the vertical edges; shell removing the top | builds |
| **P5** counterbored hole: Ø6 through, Ø10 counterbore 3 deep | stacked (A2) | shell removing the top | SG3 (the annular shoulder is a third planar face across the reference axis) |
| **P6** enclosure: 60×40×30 box, 20×10 port through both x walls | brep (class B) | chamfer the port mouth's 4 edges | builds (route P cap loop) |
| | | fillet the 4 vertical edges | builds (route E) |
| | | shell removing both x walls | builds (route P tube) |
| | | shell removing the top face | builds (route S) |
| | | shell removing one y wall, or a y wall and an x wall | builds (route S, a wall run) |
| **P6c** enclosure with a blind port into one x wall | brep (class B) | chamfer one port mouth edge | builds (route E) |
| | | chamfer the port mouth's 4 edges | builds (route L) |
| | | shell removing the top | SG3 (the port floor) |
| **P6b** enclosure shelled first, port cut after | faceted | any request | the cup is no analytic boolean operand; the cut took the mesh path |
| **P7** L bracket: L section 40×40, 8 thick, 30 tall, Ø6 hole along x through the upright leg | brep (class B) | fillet the inner corner edge, or the leg's two outer edges | builds (route E) |
| | | fillet the hole rims | builds (route L fillet arm) |
| | | chamfer a hole rim, or the top cap's loop (6 edges) | builds (route L) |
| | | shell removing the top | builds (route S) |
| | | shell removing the `y = 8` wall | SG5 (its end at the reflex corner) |
| **P8** rounded plate drilled across: 40×20×20 box, 4 vertical edges filleted r = 3, then Ø6 hole along y | brep (class B) | chamfer a hole rim, or the top cap's loop (4 lines, 4 arcs) | builds (route L) |
| | | fillet one top edge along x | SB7 (its end face is a fillet cylinder) |
| | | shell removing the top | builds (route S) |
| | | shell removing one y wall | shell-opening SO1 (the wall meets the fillets smoothly) |
| **P9** round rod with a cross hole | faceted | any request | SX9's class (`chamfers a straight prism or a revolve only`); cylinder × cylinder has no analytic class |

Ranked by the parts each refusal blocks:

| Rank | Refusal | Blocks | What the body needs |
|---|---|---|---|
| 1 | SB10 / SB3: shell of a nonprism brep or a noncap face | P1, P2, P3, P5, P6, P6c, P7, P8 | receiver erosion: planes and cylinders for a through cut (§3); spheres, tori or elliptical edges for a blind pocket or union |
| 2 | SL1: edges sharing a vertex outside complete loops | P1 | route V builds complete loops with independent edges; a partial fillet builds edges on one planar loop after straight-wall restatement (`docs/vertex-blend-design.md`) |
| 3 | a loop fillet: the curved-edge and cornered-loop fillets this survey found refused | P1, P2, P3, P4, P7 | `docs/loop-fillet-design.md`'s pipe band, which builds them |
| 4 | SB7: an edge ending on a curved face or a blend | P8 | the complete-loop fillet, which builds P8's top loop |
| 5 | a faceted receiver | P6b, P9 | an analytic boolean: a cup as a boolean operand, cylinder × cylinder; reach SX9 stays permanent |

Routes S and L below clear every brep and stacked row of ranks 1–2 whose
body is planes and cylinders along reference axes, meeting in lines, circles
and arcs; route L's fillet arm (`docs/loop-fillet-design.md`) clears ranks 3
and 4. Route V builds the sphere corners of an exact-radius three-edge
fillet (`docs/vertex-blend-design.md`).

## 2. The two increments

**Route S — shell of a through-cut record.** Erosion distributes over
intersection: for a body `A \ T₁ \ … \ Tₙ`, `(A \ ⋃Tᵢ) ⊖ t = (A ⊖ t) \ ⋃(Tᵢ ⊕ t)`
exactly, as sets. When `A` is a prism along a reference axis and every `Tᵢ`
is a prism along another reference axis passing through `A`, every piece is
a shape decad already builds: `A ⊖ t` is modify §8's section erosion over
the cup's interval, `Tᵢ ⊕ t` is the same offset outward, and the difference
is the class-B `Cut` general-boolean §3 ships. The cavity's faces are planes
and cylinders along reference axes — a dilated square port's corners are
quarter cylinders along the port's axis, which is modify §8's outward rule
applied to the tool's section — so the result is an ordinary `brepPayload`.
No new face kind, no new proof: the shelled body is the receiver's kept
faces, the cavity's faces reversed, and one rim region per removed face.

**Route L — chamfer of a complete loop on any planar face.** Reach §8.3's
cap-loop chamfer is local: a band between the loop offset into its face's
material (the cap contour) and the loop moved along the face's normal (the
side contour), a `Plane` patch per line walk, a `Cone` per circular walk, an
apex `Cone` per reflex connector. It needs the face to be planar and every
adjacent face to be swept along the face's normal, or to be a plane holding
the loop's edge between two lines along that normal — exactly route E's
(sw)/(pl) classes. Nothing in it needs the body to be a prism, so route L runs
reach §8.3's construction and proofs on a loop of any planar brep face and
attaches the band at body build, as `capBlendPayload` does.

Neither increment admits a residual, samples a surface or asks `sketch` a
question the shipped code does not already ask: route S's 2D work is modify
§8's exact offset and general-boolean §5's private scenes, route L's is
reach §8.3's. Every new gate below rejects; none admits.

## 3. Route S — shell of a through-cut record

### 3.1 Table TC — recognition

Route S reads the receiver's face record (`brepPayload`, or `brepOfStacked`
for a stacked receiver; SB1 and SB2 run first, as for every brep route) as a
prism `A` along reference axis `k` cut by through tools, or refuses (SG3).
Every test is an exact comparison of recorded floats and records; the
reference frame, embeds and `brepTopology` are brep-modify §4.1's.

| TC | Condition |
|---|---|
| TC1 | Route P's P1, P3 and P4 along `k`: exactly two planar faces across `k`, bottom facing `−k` and top `+k`, `zlo < zhi`, both regions one section `S` in the prism frame `F`. |
| TC2 | Every other face is one of: **(a)** a wall of `A`: route P's P2(a); **(b)** a pierced wall of `A`: a planar face with `Axis[2] = j ≠ k` whose outer loop is P2(b)'s rectangle and which holds zero or more hole loops; **(c)** a tool wall: a swept face with `Axis[2] = j ≠ k`, natural range, empty `side0`/`side1`, whose two levels are the levels of two (b) faces across `j`. |
| TC3 | Route P's P5 over the (a) walls and the (b) outer rectangles: each claims one segment of `S` with the right outward normal, and every segment of `S` is claimed once. |
| TC4 | Tools. In `brepTopology`, each (c) face's two rims pair with one hole-loop segment of a (b) face each; call them `W0` (at the lower level `L0`) and `W1` (at `L1`). A tool is one hole loop `H` of some `W0`: every segment of `H` pairs with a (c) face whose other rim pairs with a segment of one hole loop `H'` of one `W1`, and `H'` re-expressed into `W0`'s frame equals `H` as P4 compares loops. The tool is `prismPayload{profile: {Outer: reverse(H)}, frame: brepgeom.AxisFrame(ref, j), z0: L0, z1: L1}`, its section in `F_j`'s coordinates. |
| TC5 | Every hole loop of every (b) face belongs to exactly one tool, and every (c) face to exactly one tool. |
| TC6 | Every face has `delta = 0` (SB1) and every (c) face's two levels carry zero displacement; a (b) face whose level carries a displacement reads as no rectangle, as route P's P2(b) reader refuses it. |

A stacked union outside the admitted boss cases below fails TC1: three
planar faces across `k`. A blind pocket fails TC2: its floor is a third
planar face across `k`,
and its walls' levels are no wall's. A keyway, a boss crossing a plate's
outline, a hole breaking out of a face, a split side line and an oblique
wall fail TC2 or TC6. Each is SG3, which names the first face the table does
not take along the first axis whose caps TC1 reads, else the third planar
face across the first axis holding more than two, else axis 0's reason.

### 3.1a Exact rectangular-boss shell

Before route S's through-cut reading, `shellStackedBoss` admits one nested,
hole-free pair of axis-aligned rectangular slabs in the same frame. Both
sections and all levels must have zero displacement. The lower slab is the
plate, the upper slab is the boss, and the one removed face must be the boss
top. Inward thickness `t` must leave a positive floor, a positive lower
cavity wall, an upper cavity at least `2t` narrower in both directions, and
positive clearance between the boss and the plate's eroded outline. All new
offset coordinates must be exactly representable. Other stacks continue to
route S and keep SG3.

The result keeps the source's exterior faces except the boss top. Its cavity
has a plate inset over `[z0+t, z1-t]`, a horizontal ledge at `z1-t`, and a
boss inset over `[z1, z2]`. Four quarter-cylinder patches join the ledge's
boss-sized inner ring to the upper cavity walls. Adjacent patches meet on
four `Ellipse3` seams. The lower and upper rings each bound one ordinary
record face and one patch. The body build mints fresh `face(k)` and `wall(k)`
roles; each patch has a `filletLoop` role. No planar cap is inserted at the
transition.

For boss width `w`, depth `d` and thickness `t`, the transition's void volume
is `wdt - 2(w+d)t² + (w+d)πt²/2 + (20/3 - 2π)t³`. The shell volume subtracts
that transition, the plate-inset void below it and the boss-inset void above it from the two
source slabs. Rational interval arithmetic encloses `π`, volume and centroid.
The established fillet band's cylinder, ellipse and mesh-ring builders close
the patches. `Tessellate(VerifyAll)` charges the band's chorded volume,
surface departure and vertex motion before publishing its occupied-volume
bound. A later modify operation refuses this payload; class B and clearance
face views refuse it because they omit its patch faces. Mesh booleans use its
verified tessellation.

### 3.1b Exact rectangular blind-pocket shell

Before route S's through-cut reading, `shellBlindPocket` admits two slabs in
one frame: a hole-free axis-aligned outer rectangle below an interface, and
the same outer rectangle with one nested rectangular hole above it. The one
removed face is the top cap, and the sense is inward. Sections, levels and
thickness must have zero displacement. The pocket must leave more than `2t`
below its floor and between its edges and the outer rectangle; both
rectangles must remain wider than `2t`. Offset coordinates must be exactly
representable; otherwise the call
refuses before building a result.

The result keeps the source exterior and pocket faces except the top cap. It
adds the outer cavity's flat floor and four walls, the top outer rim, a pocket
rim around the rounded expansion, four straight and four cylindrical walls
along the expanded pocket, and a square underside at `zfloor-t`. The floor
transition uses the loop-fillet band's exact-radius arc collapse: four
quarter cylinders meet four sphere octants. The expanded pocket's rounded
rectangle is `offsetProfile(pocket, -1, t)`; its certified displacement must
be zero, and its fillet contour at `t` must reproduce the source square.

For a pocket of width `w`, depth `d`, axial depth `h`, and thickness `t`, the
expanded pocket's volume within the outer cavity is
`wd(h+t) + 2(w+d)th + πt²h + (w+d)πt²/2 + 2πt³/3`.
The shell volume is the source volume minus the eroded outer box's volume
plus that expanded-pocket volume. Rational intervals enclose `π` and all
moments; the body's occupied-volume proof uses the rounded vertical walls'
chord slivers and the fillet band's sphere/cylinder patch bounds. A later
modify operation refuses the payload. Class B and clearance face views
refuse it because its record omits the band patches; mesh booleans require
its verified tessellation.

### 3.1c Exact circular-boss shell

Before route S reads a through-cut, `shellRoundBoss` admits two exact slabs in
one frame: a hole-free axis-aligned rectangular plate and one whole-circle
boss strictly inside it. The top boss face must be the only removed face, and
the shell must be inward. Both sections, all levels, the circle radius and
the thickness must have zero displacement. The plate height must exceed
`2t`, the boss radius must exceed `t`, and the boss circle must lie strictly
inside the plate's inset rectangle. The offset levels, radius and rectangle
coordinates must be exactly represented. Other unions continue to SG3.

The result keeps the source exterior except the boss top. Its cavity has the
plate's inset rectangle over `[z0+t,z1-t]`, a ledge at `z1-t`, and the boss's
concentric inset circle over `[z1,z2]`. One LF7 quarter-torus patch joins
the ledge's circle of radius `r` to the inner cylinder of radius `r-t`.
The patch has two whole-circle edges and no seam. The ledge hole uses the
band's side-ring stations; the inner cylinder uses its cap-ring stations.
Both rings therefore share mesh vertices with their neighbouring faces.

At height `u` above `z1-t`, the transition void has radius
`R(u)=r-t+sqrt(t²-u²)`. Its volume is
`π[(r-t)²t+(r-t)πt²/2+2t³/3]`. Its first height moment above `z1-t` is
`π[(r-t)²t²/2+2(r-t)t³/3+t⁴/4]`. Rational intervals enclose both powers
of `π`, the full shell volume and centroid. `Tessellate(VerifyAll)` uses the
existing whole-circle fillet-band and face-chord volume proofs. Later modify,
class B, and clearance face views keep §3.1a's staging; a mesh boolean uses
the verified mesh.

### 3.2 Removed faces and the sense

The removed faces are read against the recognised record:

| Removed face | PR | Rule |
|---|---|---|
| a cap of `A` (TC1's bottom or top), one or both | S-1 | `A ⊖ t` keeps a removed cap's level and moves a kept one by `t`: modify §9's cup interval `[zlo + t·kept₀, zhi − t·kept₁]` |
| a wall of `A` (an (a) or (b) face), one proper connected run of whole walls of `S`'s outer loop, with or without either cap | S-2 | `A ⊖ t`'s section is `docs/shell-opening-design.md` §3's cavity section `C` over the kept chain, with that document's rim rule at each end (`sideOpeningRegions`); its SO1–SO6 refusals keep their codes, a set of walls that is no proper connected run among them (SO6). No section limit runs (shell-opening §5). Each removed wall must be a straight wall along a section axis, whose rim is a planar face, and neither end of the run may cut backward along the removed carrier (a reflex end, whose rim lies inside the material): SG5 |
| a tool wall, a tool's floor, a face of a hole of `S` | — | SG4 |
| two removed faces sharing an edge, other than the runs above | — | SG5 |

Only `Inward` builds. The dilation `(A \ T) ⊕ t` does not distribute over the
difference and rounds every convex edge of the receiver — a hole mouth's rim
into a torus, a box corner into a sphere — so `Outward` is SG1.
`WithNoOpenings` is SG2: a brep record's lumps are its connected face sets
(general-boolean §4.2), and a closed cavity's faces would read as a second
solid; the void shell `shellClosedPrism` builds lives on the stacked record
alone.

### 3.3 Construction

With `s = +1`, `t` the thickness in millimetres with its conversion bound
`tDelta` (`extent.MagnitudeInBounded`), and the receiver live:

1. **Gates on `S`.** Modify §8's shell gates on `A`'s section, in modify §4's
   order: S18, S10's section limit (`shellsurvey.SectionInradius`), S10's
   height limit per kept cap (`t < h` with one cap kept, `t < h/2` with both:
   reach SX11's reading), S11a as `offsetProfile(S, +1, t)` is built, then
   `auditOffsetSectionBudget` (S8, S11b, S9).
2. **Each tool dilated.** `Dᵢ = offsetProfile(Tᵢ.profile, −1, t)`: S11a for a
   dropped feature, then the same audit on `Dᵢ` against `Tᵢ`'s section. The
   dilation rounds the tool's convex corners into arcs of radius `t` and
   miters its reflex ones, modify §8's outward row, so a square port's cavity
   gets four quarter cylinders along the port's axis.
3. **TC7 — the strips beyond the pierced walls.** Beyond `W0` the dilated
   tool reaches into `{j ∈ [L0 − t, L0)}` over `Dᵢ`'s box, and beyond `W1`
   into `{j ∈ (L1, L1 + t]}`. That slab must hold no material of `A`: with
   `A` a prism along `k`, the question is 2D in `S`'s plane — the strip
   `[L0 − t, L0) × [m₀ − t, m₁ + t]` (`m` the third axis, `[m₀, m₁]` the
   tool section's `m`-extent) meets `S` only where some segment of `S` other
   than the pierced wall's own crosses it, since the strip touches that wall
   from outside. Every other segment's outward-rounded bounding box must be
   separated from the strip box along some axis by an exact comparison; a
   box that is not is SG6. The test is reject-only and may refuse a valid
   body (a far segment whose box happens to reach the strip).
4. **The cavity.** `A' = prismPayload{profile: S ⊖ t, frame: F, z0, z1}` on
   step 1's interval, with the moved level's displacement `z0Delta + tDelta +`
   the float sum's rounding, as `shellClosedPrism` derives it, and every face
   of its view carrying `offsetSectionDelta(S, +1, t, tDelta)` (modify §9's
   figure). `Tᵢ' = prismPayload{profile: Dᵢ, frame: F_j, z0: L0 − t, z1: L1 + t}`
   with `offsetSectionDelta(Tᵢ, −1, t, tDelta)`. The cavity is
   `Cut(…Cut(A', T₁'), …, Tₙ')` through class B's own entry over payloads
   (PR 0's `classBOfPayloads`, the body-free form of `tryClassB`): each cut
   must build a `brepPayload`; a pair class B does not admit — two dilated
   holes closer than `2t` (B7), a dilated tool reaching a cap of `A'` within
   `t` of a hole (the crossing reach or the mesh path) — is SG6, naming the
   tool. The private cuts build no `Body` and touch no document. Class B
   admits no section displacement (general-boolean B5), so the cuts run on
   the offsets' recorded floats with every displacement zero, and every
   cavity face then takes `δ`, the largest of the offsets' displacements
   above, as its section displacement, and the largest of `δ` and the cap
   levels' displacements on both its levels: an upper bound on each face's
   own. That charge `c` is in no record the cuts read, so each cut takes it
   as a parameter (`classBOfPayloads`'s `delta`; class B's own booleans pass
   zero) under two rules. First, every face box of `A'` (and of the cavity
   so far) and every tool box that B7 and the through reach compare grows by
   `c` on every side, and each through-reach slab's level band by `c`: a
   dilated tool recorded within `2c` of a cavity face meets it, and the cut
   refuses (SG6) where the denoted tool may cross the denoted face. Second,
   a positive `c` refuses the crossing reach (SG6): that reach computes its
   crossings from the zero-displacement records, and the crossing-angle
   charge (prism-boolean §3.4, A6) amplifies only a displacement the records
   carry, so a cut crossing at a grazing angle would publish vertices the
   flat `c` does not cover. A cut whose `c` is zero takes the crossing reach
   as class B does. Class B also needs every line exactly along a reference
   axis (B6), so an eroded polygon whose float miters drift off the axis — any convex
   corner at a thickness that is no exact float, such as 0.1 in — is SG6; a
   section whose corners are arcs erodes through G1 joins that keep each line
   on its axis.
5. **Assemble.** The result record is: every receiver face but the removed
   ones, verbatim; every cavity face reversed, except those lying in a
   removed face's plane; and one rim region per removed face `R`. Reversal
   flips a planar face's `outward` and walks a swept face's wall the other
   way (`reverseSegment`, extended to a `CircleSeg` by flipping `CCW` and
   swapping its range, with `side0`/`side1` exchanged), the rule
   `brepPayload.placed` applies under a reflection. The rim at `R` (level
   `L_R`, outward as `R`'s): the cavity faces in `R`'s plane are the planar
   cavity faces across `R`'s axis at level `L_R`, and at a removed wall the
   straight cavity walls along `k` on `R`'s carrier as the rectangles they
   sweep, re-expressed into `R`'s
   frame through `brepgeom.NewMap2`'s signed permutation (a reflecting map
   re-winds the loops, general-boolean A4). The rim's record is
   `{Outer: R.Outer, Holes: R.Holes'}` where `R.Holes'` holds, for each such
   cavity face `Q`, `reverse(Q.Outer)`; and for each hole `h` of `R` — a hole
   of `S`, since `R`'s region is `S` — the band `{Outer: reverse(Q.hole),
   Holes: [h]}`, where `Q.hole` is the one hole of some `Q` whose record
   equals `(S ⊖ t).Holes[i]`, `i` the index of `h` in `S`; a hole of `R` with
   no such partner, a `Q` hole that partners no `h`, or a `Q` holding a loop
   that is neither its outer nor such a hole, is SG7. The rim regions and
   bands carry the cavity's section displacement `δ` as their `delta`.

   A removed wall (S-2) is read the same way. A pierced wall is its own
   planar record. A wall along `k` is stated as the rectangle it sweeps over
   `[zlo, zhi]`, in the signed-permutation frame across its normal axis whose
   normal is its outward normal, with `sweep` along `k`. Each hole `h` of a
   pierced wall is a tool's loop, and its band partners the cavity hole equal
   to that tool's `Dᵢ`, joined as class B records it, carried into `R`'s
   frame and reversed. Two lines compare by their walked ends: a reversed
   loop states a line `{End, Start, 0 → 1}` where class B states
   `{Start, End, 1 → 0}`. A wall's rim region states the cavity's levels and
   `A`'s cap levels as in-plane coordinates, so it carries the largest of `δ`
   and every cavity level's displacement.

   Where a cavity face's outer loop runs along `R`'s own outer loop, the two
   rims of a wall run and a removed cap reach each other's edge, and
   `{Outer: R.Outer, Holes: …}` would hold a hole touching its outer loop. The
   rim is then stated by cancellation (`throughshell.RimRegions`): every
   axis-aligned line of `R.Outer` and of each reversed cavity outer loop is
   split at each line end of another loop lying strictly inside it, compared
   exactly; a piece of `R.Outer` and a cavity piece walking the same two ends
   the other way both drop; and the rest chain into loops, a piece continuing
   along its own loop where its successor survives and otherwise into the one
   surviving piece starting at its end whose predecessor dropped. One
   counter-clockwise loop with clockwise ones is one region with holes;
   counter-clockwise loops alone are one region each; an end with no or two
   continuations, a loop with no area, or two outer loops beside a hole is
   SG7. Every split point is a recorded vertex lying exactly on the line it
   splits, so nothing is solved or admitted. Where nothing cancels, the rim
   is the record above, bit for bit.
6. **Audit.** Every rim region and band runs modify §5's audit: S8, S7, S9.
7. **Closure.** `assignRoles`, `falsifyBrepPayload`, `brepTopologyContext`:
   an unpaired edge is `ErrUnsupported` (general-boolean §4.2's count), the
   proof that the receiver's kept faces, the cavity and the rims close.
   `evalBrepContext` builds and measures; a non-positive volume is
   `ErrDegenerate`. `commitModifyResult` retires the receiver.

Why the pieces pair by identity: a removed cap's region is `S` (TC1), the
cavity's cap there is `S ⊖ t` verbatim where no tool reaches the cap plane
(general-boolean B.3: a tool along `j ≠ k` meets only faces across `j`), and
its hole `i` is `(S ⊖ t).Holes[i]` as `offsetProfile` keeps loop order. A
tool that does reach the cap plane splits or notches that cap through the
crossing reach, which step 4 admits only where the cavity's charge is zero;
step 5 then reads the pieces as they come and step 6 proves the rim's
nesting, so a notched cavity cap builds where its loops still partner, and
refuses honestly where they do not.

### 3.4 Table SG — refusals and gate order

Modify §1's test picks every sentinel.

| SG | Call | Exists? | Sentinel |
|---|---|---|---|
| **SG1** | `WithShellSense(Outward)` on a brep or stacked receiver | yes; the dilation rounds the receiver's convex edges into tori and spheres this record does not hold | `ErrUnsupported` |
| **SG2** | `WithNoOpenings` on a brep or stacked receiver | yes; a brep record holds no void shell | `ErrUnsupported` (replaces SX16's text for this call) |
| **SG3** | the record is no through-cut record (Table TC) and is outside §3.1a–§3.1c: a blind port, a keyway, a crossing boss, a split or oblique wall, a displaced tool level | yes; its erosion may hold surfaces beyond the admitted cases | `ErrUnsupported`, naming the first face Table TC does not take |
| **SG4** | a removed face that is a tool wall, a tool floor or a hole wall of `S` | yes | `ErrUnsupported` |
| **SG5** | a removed face that names no face of the record; a removed wall that is no straight wall along a section axis (a fillet cylinder: its rim is no planar face); a wall run whose end cuts backward along the removed carrier (a reflex corner: the rim lies inside the material). A set of walls that is no proper connected run is shell-opening SO6 | yes | `ErrUnsupported` |
| **SG6** | a tool's dilation reaches material beyond its pierced wall (TC7), or a private class-B cut does not build a brep: two dilated tools within `2t`, a dilated tool reaching a cap of `A'`, an eroded miter off its axis, a dilated tool recorded within twice the cavity's charge of a cavity face, or a cut that takes the crossing reach while that charge is positive (§3.3 step 4) | yes | `ErrUnsupported`, naming the tool |
| **SG7** | a rim's loops do not partner the cavity's trace, or do not chain into one outer loop with holes or outer loops alone (step 5) | — (a falsifier) | `ErrUnsupported` |

Gate order for a brep or stacked `Shell`, after modify §4's stage 1 and
reach SX10:

| Stage | Gates |
|---|---|
| 2a. record | RB dispatch; SB2; SB1 |
| 2b. route P | brep-modify §4.2 unchanged: an axis whose prism takes the removed faces as caps builds the prism's own cup or tube |
| 2c. route S entry | SG1; SG2; Table TC (SG3); the removed faces against the record (SG4, SG5's face and straight-wall tests) |
| 3. existence | caps alone: S18; S10 (section and height limits); S11a on `S`. A wall run: `sideOpeningRegions` in shell-opening §5's stages 2–5 (SO6, SO3's height half, S11a and SO1, SO2, SO4 per end, S8, S11b and S9 on `W` and `C`, the area identity), then SG5's reflex-end test. Then S11a on each tool |
| 4. offset audit | S8, S11b, S9 on `S ⊖ t` (caps alone) and on each `Tᵢ ⊕ t`; TC7 (SG6) |
| 5. cavity | the private cuts (SG6) |
| 6. assembly | SG7; the rim audit S8, S7, S9 |
| 7. closure | `falsifyBrepPayload`, `brepTopologyContext`, `evalBrepContext` |

SB10 narrows to a record route S also refuses, and its text names SG3's
reason. SB3 narrows likewise: a record that reads as a prism along some axis
whose caps are not the removed faces is read again by Table TC along every
axis before it refuses.

## 4. Route L — chamfer of a complete loop on a planar face

### 4.1 Table LB — the admitted selection

Route L runs for `Chamfer` and `Fillet` on a brep or stacked receiver when
route P takes no axis and the selection is not route E's single straight
edges — every edge a straight line along a reference axis (EB1), no two
sharing a vertex (EB7). A `Fillet` builds its loops with the fillet arm of
`docs/loop-fillet-design.md`, whose band is a pipe where this section's is
ruled; Table LB, the record rewrite and SL1, SL2 hold for both. A record carrying
route L bands reads as no prism, so route P takes no axis of it. Each selected edge is matched to one
loop segment of one planar face of the record by lifting the segment's ends
(a circle: its centre and radius) through the reference frame and placement
and comparing within `1e-6`, as route E's `admitEdge` does; the match
identifies and admits no geometry.

| LB | Condition | Refusal |
|---|---|---|
| **LB1** | The selected edges are exactly the segments of one or more whole loops of planar faces: every segment of each touched loop is selected and no selected edge lies outside a touched loop. | SL1 |
| **LB2** | No two touched loops share an edge, and no touched loop lies on a face with `delta ≠ 0`. | SL1 / SB1 |
| **LB3** | Each segment of a touched loop on face `F` (normal axis `n`) pairs in `brepTopology` with a face that is **(sw)** a swept face along `n` whose rim is the segment, natural range, empty `side0`/`side1`; or **(pl)** a planar face across an axis `≠ n` holding the segment as a loop segment whose two neighbours are `LineSeg`s along `n` over their natural range. | SL2 |
| **LB4** | Every segment of the loop and of the adjacent faces' loops runs over its natural range, and no loop holds two consecutive segments on one carrier (route E's EB5). | SL2 |
| **LB5** | The side level `L_F ∓ d` lies strictly inside every (sw) face's interval and strictly short of every (pl) neighbour line's far end: the band reaches `d` along the wall and must not reach another vertex (reach SX7's `reach ≥ height`, read per adjacent face). | SX7 |
| **LB6** | The adjacent faces all leave `F` on one side: every (sw) face's interval and every (pl) neighbour line lie on `F`'s inner side (against `F`'s outward normal), or all on its outer side. | SL2 |

LB6 decides the band's sense. A loop whose walls descend into the body — a
cap's outer loop, a hole mouth, a pocket mouth, a port mouth — is reach
§8.3's chamfer and removes a wedge. A loop whose walls rise off the face — a
boss root — is modify §7's concave corner: the cap contour is still the loop
offset into `F`'s material (away from the boss), the side contour is the
loop moved up the boss, and the band fills the wedge between them. Both are
the one construction; only the side level's sign and the patches' outward
orientation differ.

### 4.2 Construction

For each touched loop `ℓ` of face `F`, `dc` is its setback across `F` and
`ds` is its setback along the faces beside `ℓ`. Without
`WithAsymmetricChamfer`, `dc = ds = d`. With the option on a brep or stacked
face view,
every edge of a touched loop must consistently name either `F` or the face
beside that edge as its reference. A reference to `F` sets `(dc, ds)` to
`(d, otherDistance)`; a side reference swaps them. Mixed assignments on one
loop are SX4. The public one-reference-face-per-edge gate catches this first
as SX3 when a query names both `F` and a side face. Touched loops on the same
face and material side must agree on their pair, because they share one
cap-blend view. Then:

1. **The cap contour.** `ℓ` offset `dc` into `F`'s material by reach §8.3's
   construction (`capLoopBoundary`): SX6 for a dropped carrier, SX13's radial
   half per circular wall, SX14 per corner. `F`'s region is rewritten with the
   contour in `ℓ`'s place, and the second offset at the top of `dc`'s
   conversion span runs as `auditCapBlendSetbackSpan` does.
2. **The side level.** `sideZ = L_F + σ·ds` with `σ = −1` for a loop whose
   walls descend into the body and `+1` for one whose walls rise (LB6), its
   displacement `levelDelta` the float sum's rounding plus `dsDelta`; SX13's
   axial half where `sideZ == L_F`.
3. **Trims.** Each (sw) face's level at `F` becomes `sideZ`, its level
   displacement `F`'s plus `levelDelta`. Each (pl) face's segment on `ℓ`
   moves to `sideZ` along `n`, its section displacement taking the same sum,
   its two neighbour lines shortened or lengthened to it, a connector-free
   blend at both corners whose cutback is `ds` (route E's (pl) rule); the face
   runs modify §5's audit (S8, S6, S7, S9).
4. **The face audit.** `F`'s rewritten region runs modify §5's audit (S8,
   S7 — the contour against `F`'s other loops, SX7's text — and S9).
5. **The band.** The record stores `loopBands`: `(face, loop, dc, ds, σ)`
   per touched loop, beside the rewritten faces; no face record holds a cone.
   At body build, after `evalBrepContext` has built every face and edge of
   the record, each band is attached by reach §8.3's `buildCapBand` run in
   `F`'s own frame (`prismLike` over `F.frame`, cap level `L_F`, side level
   `sideZ`, `matSign` from `σ` and `F.outward`), with both of its directrices
   handed in as the brep's own coedges: the cap contour's edges are `F`'s new
   loop coedges, the side contour's edges are the (sw) faces' rims at `sideZ`
   and the (pl) faces' moved segments. PR 0 refactors `buildCapBand` to take
   the cap-level coedges from its caller; the prism cap blend passes the
   ones it minted. Patches, their readings (`setPatchReadings`), their
   orientation (`fixPatchOrientation` against reach §8.3.1's reference
   `(ds·n̂_wall, σ·dc·n̂_F)`, which has a positive dot product with the true
   outward normal in both senses), their skews, locus spans and corner flux
   are reach §8.3's and §8.4's unchanged.
6. **Closure.** The record's faces pair by `brepTopologyContext` as before;
   the band's patches each bound two faces by construction (the contour edge
   on `F`, the side edge on the adjacent face, the corner rulings between
   neighbouring patches), and `attachFaceLoopsContext` proves every edge
   bounds exactly two faces over the whole body.

The contour displacement (reach §8.4, `capband.ContourDisplacement`) is
charged into `F.delta`, so every reading of `F` and every cap-level vertex
of the bands carries it, and a later modify op on a body whose contour was a
float solve refuses with SB1 — as it does for a cup. The side contour is the
receiver's own loop moved along `n`, so the faces beside it carry only the
side level's displacement (step 3). An axis-aligned
loop at a millimetre setback has an exact contour and keeps `delta = 0`, so a
second loop on the same body chamfers in a later call.

### 4.3 Measurements

`measureBrepContext` sums general-boolean §4.3's face terms over the
rewritten record, then adds each band's terms through reach §8.4's readers
(`readBandMass`): its volume flux and first moments about the plane-local
origin, which is the reference origin every face frame shares bit for bit
(`brepEmbeds`), each `Cone` patch's chord-versus-locus terms, the closure
slivers (`capBandClosure`), and `capband.LevelVolume` for the side level's
displacement. Those readers integrate the closed band region `B` — the
patches closed by the cap contour's disk at `L_F` and `ℓ`'s disk at `sideZ`
— so the patches' share is `B`'s flux and moments less the two disks', each
disk exact against `B`'s outward normal, signed `+1` where `B` is material
(an outer loop that descends, a hole that rises) and `−1` where it is void
(a hole that descends, an outer loop that rises). `F`'s area composes `sectionDisplacementArea` over the contour
displacement as a cap face does; each patch publishes its own bounded area.
A body whose bands are all `Plane` patches at exact coordinates reports
`Exact` where its true volume is a float; one holding a `Cone` is
`Approximate`. `Bounds` reads the record's faces alone: each patch is ruled
between its cap contour, a loop of `F`, and its side contour, a rim or
segment of a face beside it, so a linear functional over the patch is
extremized on those two directrices, which the record's faces hold within
their own displacements. The centroid divides the summed moments by the
volume and lifts through the reference frame.

### 4.4 Table SL — refusals

| SL | Call | Exists? | Sentinel |
|---|---|---|---|
| **SL1** | a partial loop outside one admissible straight-edge fillet chain, two loops sharing an edge after route V's partition, or loops mixed with single edges for a `Chamfer`; a selected chain on one planar-face loop takes the partial fillet route | yes | `ErrUnsupported` |
| **SL2** | an adjacent face outside LB3/LB4/LB6: a curved or oblique neighbour, a split side line, a neighbour whose own loop continues past the vertex on a curve, walls on both sides of `F` | yes | `ErrUnsupported` |
| **SL3** | retired: a `Fillet` of complete loops builds through `docs/loop-fillet-design.md`'s fillet arm or refuses with that document's Table SF | — | — |
| **SL4** | an asymmetric route L reference with no unambiguous record-face identity | yes | reach SX16 |

Reach SX6, SX7, SX12, SX13, SX14 and SX15 keep their meanings per band, and
base S6/S7/S8/S9 per rewritten face.

Gate order for a brep or stacked `Chamfer`, after modify §4's stage 1,
reach SX10, SB2, SB1 and route P (brep-modify §6's stages 2a–2b):

| Stage | Gates |
|---|---|
| 2c. entry | independent straight edges → route E (brep-modify §6); complete planar-face loops → route L: LB1, LB2; an asymmetric selection outside both routes is SX16 |

For a `Fillet`, route V also partitions complete loops and independent
straight edges before LB1/LB2. A selected straight-edge chain on part of
one planar-face loop, including a swept wall restated as a plane, takes the partial fillet route
(`docs/vertex-blend-design.md` §2).
| 3. topology | LB3, LB4, LB6 (SL2); LB5 (SX7) |
| 4. existence | SX6, SX13 per band as the contour is built |
| 5. audit | per band SX14, SX7, SX12 on the contour; per (pl) face S8, S6, S7, S9; per `F` S8, S7, S9 |
| 6. build | the record's closure; `buildCapBand` per band (SX15 as each patch is oriented) |

## 5. Tables BG and DG — results and downstream

| BG | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BG1** | route S shell | `brepPayload`, `stack` nil | the receiver's kept faces, the cavity's faces reversed, one rim per removed face; every edge on exactly two faces | `face(k)` / `wall(k)` by result index; no `capStart`/`capEnd` |
| **BG2** | route L chamfer | `brepPayload` with `loopBands`, `stack` nil | the rewritten record's faces plus the band patches | `face(k)` / `wall(k)`; each patch `chamferLoop(f,l,p)` for face `f`, loop `l`, patch `p` in its band's own order |
| **BG3** | rectangular-boss shell (§3.1a) | `brepPayload` with `bossShell`, `stack` nil | ordinary faces plus four cylinder patches and four ellipse seams | `face(k)` / `wall(k)`; the patches carry `filletLoop` roles |
| **BG4** | circular-boss shell (§3.1c) | `brepPayload` with `bossShell`, `stack` nil | ordinary faces plus one whole-turn torus patch | `face(k)` / `wall(k)`; the patch carries a `filletLoop` role |

A BG1 result is an ordinary brep: general-boolean §4.5 reads it unchanged.
A BG2 result is a brep whose body carries extra faces, and each consumer
below reads the bands where the plain brep reader would miss them:

| DG | Consumer | BG1 | BG2 |
|---|---|---|---|
| **DG1** | mass properties, `Bounds` | general-boolean §4.3 with the cavity and rim faces' `delta` | §4.3 |
| **DG2** | `Verify` validity, tolerance gate | by construction; `brepGateDiameter` reads every vertex | the same; the bands' vertices are body vertices |
| **DG3** | `Tessellate`, STL/OBJ/3MF | general-boolean §4.4 | §4.4 for the record's faces, with one count per band walk shared by the trimmed wall, the patch and the contour (`docs/tessellation-reach-design.md` §7's rule). `chordCapBlendLoop` chords the band's loop in `F`'s frame, and the wall beside a (sw) walk takes the band's side-ring points as its samples, carried into its own frame exactly (`brepImposeWall`), so the wall's rim at the side level and the band's side ring are one set of vertices; a straight wall needs no count. `emitCapBand` writes each band's strip and its departure terms. A band that rises off its face (`σ = +1`) turns its windings over. At `VerifyAll` the occupied-volume proof adds the band's chord-polygon volume (`CapBlendChordVolume` over the band's height alone) and swaps the vertex-motion allowance for the cap-blend twin's per-vertex motion array; a band DG4's rule does not admit leaves the mesh export-only |
| **DG4** | mesh boolean operand | class B over the face view where it admits, else the mesh path | the mesh path only; admitted under `capBlendOccupiedVolumeAdmission`'s rule per band (every corner a line–line miter, an exact G1 join or a whole turn), else `ErrUnsupported`; a hole or pocket mouth, whose corners are apex cones, is not admitted (`brepBandsOccupiedVolumeAdmission`, read by `requireVolumeProvingPayload`) |
| **DG5** | `ThroughAll`/`ToFace` | the per-face extremes; refuses at `delta > 0` as a prism does | the same, plus each band's extrema under the contour displacement |
| **DG6** | clearance | `addBrepFaces`; no model when any face carries `delta > 0` | no model: the pair reads `Suspect` unless its boxes decide it (reach DX6's staging) |
| **DG7** | undercut survey | per face | per face, and per patch through `Face.NormalAt` with reach DX7's three-valued rule and allowance |
| **DG8** | concave-radius survey | the cavity's dilated hole walls are read; undecided when any face carries `delta > 0` | undecided when any band holds a patch whose stamped departure is not exactly zero (reach DX8) |
| **DG9** | wall survey | staged `Suspect`, as for every brep. The wall behind every kept face is `t` by construction, but the material between two features the erosion left solid is not, and no reader proves which is which | staged `Suspect` |
| **DG10** | STEP | analytic: planes and cylinders | analytic where every patch is a `Plane`; a `Cone` sends the whole body to the faceted writer (`docs/step-export-design.md`) |
| **DG11** | a later `Fillet`/`Chamfer`/`Shell` | Tables RB/EB/SB/TC/LB over the record; RB1 needs `delta = 0`, so a shell whose offsets are exact floats takes route E or L again, and one whose offsets rounded is SB1 | the same over the rewritten record; an edge of a band patch is not in the record and matches nothing (route E's SB6, route L's SL1) |
| **DG12** | `Placed`, `Mirrored`, `PatternCopies` | re-lifts every face frame | the same; `loopBands` are re-attached by the re-evaluation, as `capBlendPayload.placed` re-derives its bands |

BG3's and BG4's consumer rules are stated in §3.1a and §3.1c.

## 6. What stays refused, and why

| Request | Code | Why not here |
|---|---|---|
| chamfers of edges sharing a vertex outside one loop, or fillets whose selected edges share no planar loop after straight-wall restatement | SB5 / SL1 | the mixed chamfer's corner plane has no reference-axis normal; route V and the partial fillet admit the cases in `docs/vertex-blend-design.md` |
| an edge ending on a blend or a curved face (P8) | SB7 | its honest form is the complete-loop fillet, `docs/loop-fillet-design.md` |
| shell of a blind pocket outside §3.1b, a blind port, a stacked union outside §3.1a or §3.1c, or a keyway (P2, P3, P6c) | SG3 | the remaining cases need a shape-specific exact transition and occupied-volume proof |
| an outward or closed shell of a brep | SG1 / SG2 | §3.2 |
| shell removing a curved wall (a fillet cylinder, P8), or a wall run ending at a reflex corner (P7's `y = 8` wall) | SG5 | the rim at a curved wall is a swept face less the cavity's trace, and a reflex end's rim lies inside the material along the removed carrier; the rim assembly states only planar regions on a removed face |
| any op on a faceted result (P5, P6b, P9) | reach SX9 | permanent (reach §11); the boolean is the owner |

## 7. Decided questions

- **Build the cavity through the shipped class-B `Cut`, not face by face.**
  The erosion algebra makes the cavity an ordinary boolean result; a direct
  3D assembly would re-derive general-boolean §5's scenes and §10's keyed
  crossings for one caller.
- **Charge route L's contour displacement into `F.delta`, not a per-loop
  field.** One reader path; it is zero for every axis-aligned loop at a
  millimetre setback, and a per-loop displacement can be added later without
  changing a published measurement.
- **A reflex corner of `F`'s region takes the cap-loop chamfer's apex cone.**
  Route L offsets `ℓ` by reach §8.3's construction, which closes every
  reflex corner with an arc of radius `d` about the corner; a hole's corners
  are reflex corners of the face around it, so a rectangular mouth's band
  carries four apex cones, as a prism's rectangular hole loop's does.
- **A removed wall run takes the shell-opening section, straight walls
  along section axes only.** `C` carries that document's rim rule and
  refusals unchanged. A curved wall's rim is a swept face less the cavity's
  trace, and a reflex end's rim lies inside the material; the rim assembly
  states neither (SG5).
- **State a touching rim by cancelling coincident pieces.** The rim is `R`
  less the cavity's trace in either case; cancellation over exact recorded
  vertices states it where the trace reaches `R`'s edge, and a private
  `sketch` scene would return the cut vertices as `Partial` fragments decad
  can only reject (shell-opening §4.6).
- **`Outward` and `WithNoOpenings` refuse.** §3.2; neither has a record.
- **Asymmetric route L chamfers use the existing two-distance band.** The
  reference face chooses `dc` and `ds` consistently per loop. A stacked
  receiver's face view still lacks the required public face identity (SL4).
- **Hand-offs: none.** No capability is needed from `sketch`, `r3` or
  `units`.

## 8. Do not do this

- **Build the cavity as a face offset with sharp reflex edges.** It is a
  different, wrong solid: the wall at a reflex corner would be thinner than
  `t` measured to the corner. decad's shell is modify §8's erosion, and the
  rounded reflex edges are the dilated tools' arcs.
- **Erode a stacked union slab by slab.** Erosion does not distribute over
  union; the boss base's two reflex edges meet in an elliptical edge no slab
  record states. SG3.
- **Admit a dilated blind tool by ignoring its rounded floor.** The cavity
  under a pocket floor is bounded by cylinders and spheres, not by the
  dilated section swept to the floor level. SG3.
- **Decide TC7 by distance from the hole to the wall's ends.** A U-shaped
  section's other arm can lie within `t` of the strip; the per-segment box
  test is the exact reject-only reading.
- **Let the private cut fall back to the mesh path.** A faceted cavity has
  no record to assemble; SG6 refuses.
- **Match a rim's holes to the cavity's by order.** The crossing reach can
  renumber a notched cap's loops; step 5 matches by record identity and SG7
  refuses a miss.
- **Store a cone in a `brepFace`.** The record's faces are planes and
  axis-aligned swept segments, and every reader of it relies on that; a band
  is attached at build from its loop and setbacks, as `capBlendPayload`'s is.
- **Sample a patch to decide its orientation where the reference dot is
  ambiguous.** Reach §8.3.1's reference has a positive dot product with the
  true normal for any positive setback pair in either sense; SX15 refuses
  where `NormalAt` cannot answer.
- **Patch a corner where a band meets a lateral blend with a triangle.** Its
  plane is in no reference frame; SL1.
- **Admit a loop fillet with a cylinder-only band and a sphere at each
  mitred corner.** The two cylinders meet along an ellipse and no sphere is
  swept there (`docs/loop-fillet-design.md` §2).

## 9. Required tests

Every test asserts computed geometry against a closed form, bounds as
relations, through the public booleans. Fixtures are §1's parts.

Route S (S-1):

- P1, shell removing the top, `t = 2`: a brep of 13 faces (the receiver's 6
  kept, the cavity's 6 reversed, one rim), volume within its bound of
  `5632 + 220π` (receiver `16000 − 180π` less the cavity
  `36·16·18 − 25π·16`), the rim one planar face at `z = 20` with the
  eroded rectangle as its hole, the cavity's hole wall a cylinder of radius 5
  along y, `Verify` `Sound`, the mesh's occupied-volume proof covering the
  same figure, STEP analytic.
- P6, shell removing the top, `t = 2`: 23 faces, volume within its bound of
  `21472 + 224π` (cavity `56·36·28 − 56·(320 + 4π)`), the cavity's port four
  planes and four quarter cylinders of radius 2 along x.
- P7, shell removing the top, `t = 2`: volume within its bound of
  `9552 + 56π` (cavity `28·(276 − π) − 4·25π`): the L's reflex corner
  erodes to a quarter cylinder of radius 2 along z, the dilated hole runs
  through the eroded leg's 4 mm.
- P8, shell removing the top, `t = 2`: volume within its bound of
  `4984 + 382π` (cavity `18·(572 + π) − 16·25π`).
- P1 with both caps removed: the rims at both levels, volume within its
  bound of `4480 + 220π` (`5632 + 220π − 36·16·2`: the cavity grows by the
  bottom slab, which the dilated hole does not reach); P1 placed by a
  translation reproducing the volume. A stacked receiver holds a blind
  interface, so Table TC refuses it unless §3.1a or §3.1b takes it.
- Bands: P1 with a Ø4 hole along z through `(8, 10)`, top removed: the rim
  and one band between the receiver's hole and the cavity's, volume
  `5632 + 428π`; both caps removed, `4480 + 460π`, the band's wall a lump of
  its own.
- Bound fixture, shown to fail first: P8 shelled at `units.Inches(0.1)`
  (P1 at that thickness is SG6, its float miters off their axes; P8's arc
  corners keep every line on its axis): every cavity vertex at
  `x = 40 − t` on `y = 3` and `y = 17` encloses its exact rational position,
  and deleting the cavity faces' `δ` (set to zero in step 4) turns those
  vertices red; the test records that leg. The volume encloses the exact
  rational closed form, unplaced and placed `10⁶` mm along x; its own
  rounding bound exceeds `δ`'s charge, so it does not see the leg.
- Refusals: P2 outside §3.1b and P6c → SG3 naming the pocket floor; P3 → SG3 naming the
  third planar face across z; a removed hole wall → SG4; a removed wall → SG5
  (until S-2); P1 with a second Ø6 hole along y 7 mm from the first → SG6
  (the dilated holes meet); P1 at 0.1 in → SG6; a U section with a 1 mm
  slot, drilled through one arm → SG6 through TC7; `Outward` → SG1;
  `WithNoOpenings` → SG2. A hole 1.5 mm under the top on a 2 mm shell builds
  through the crossing reach: the cavity's top splits in two, the rim holds
  both pieces, and the volume is `5632 + 220π − 16·(25·acos 0.9 − 4.5·√4.75)`.
  Two cuts refuse on the cavity's charge (§3.3 step 4's rules), SG6:
  the 40×20×20 box drilled R = 3.1 along y through `(20, ·, 15.90004)`,
  top removed at 1 mm, whose dilated hole notches the cap through the
  crossing reach with a positive charge (built, its notch vertex on `y = 1`
  misses its exact position by more than its bound); and the box
  `[32.5, 50] × [16.5, 20] × [0, 30]` drilled R = 2.69 along y through
  `(44.990000000000002, ·, 15)`, top removed at 1.16 mm, whose dilated hole
  is recorded apart from the eroded wall `x = fl(50 − 1.16)` while the
  denoted tube crosses `x = 50 − 1.16` (built, its area interval lies above
  the denoted area). Every refusal leaves the receiver live.

Route S (S-2):

- P6 with one y wall removed and both caps kept (a U-channel with its port
  through both x walls): `C = [2, 58] × [0, 38]`, the rim the wall's
  rectangle holding the opening `[2, 58] × [2, 28]` in `(x, z)`, whose four
  corners are the rim rule's right-angle cuts and the kept caps' levels;
  volume within its bound of `22592 + 224π`
  (`60000 − 2128·26 + 56·(320 + 4π)`). P6 with one x wall removed reads as a
  prism along x, so route P builds it as its own cup (`24272 + 232π`) before
  route S runs.
- P1 with one y wall and the top removed: `C = [2, 38] × [0, 18]`, volume
  `4336 + 270π` (`16000 − 180π − 648·18 + 450π`); the top's rim and the
  wall's rim are each a U of eight lines, and the wall's band runs between
  the cavity's hole (radius 5) and the receiver's (radius 3). With both caps
  removed, `3040 + 270π`, the wall's rim two rectangles beside the band.
- P6 with its y = 0 and x = 0 walls removed, a run of two: `21256 + 232π`
  (`C = [0, 58] × [0, 38]`). P1 with its x = 0 wall removed: `6272 + 220π`
  (`C = [0, 38] × [2, 18]` over `[2, 18]`).
- Each builds one lump that Verify reads `Sound`, with the mesh's
  occupied-volume proof covering the figure; through the public API P1's
  wall-and-top shell writes analytic STEP.
- Bound fixture, shown to fail first: P8 with only its two `y = 20` edges
  filleted, its `y = 0` wall removed at `units.Inches(0.1)` (both end cuts
  are right-angle exact pairs and the far corners erode through G1 joins):
  every cavity vertex at `x = 40 − t` on `y = 0` and `y = 17` encloses its
  exact rational position, and deleting `C`'s displacement turns them red;
  the volume encloses the exact closed form. The wall rim's level charge is
  pinned on a cavity built with a thickness conversion bound of `10⁻⁹` and
  no section displacement, since in every real fixture `δ` covers it.
- Refusals: P8's fillet cylinder and P7's `y = 8` wall → SG5; P8's y wall →
  SO1; P1's two x walls → SO6. Every refusal leaves the receiver live.

Route L (L-1):

- P2, chamfer the pocket mouth, `d = 1.5`: volume within its bound of
  `15000 − 135/2 − 9π/8` (`30d² + πd³/3`), four `Plane` and four apex `Cone`
  patches, the top face's hole loop four lines and four arcs of radius 1.5
  about the mouth's corners, each pocket wall's level at `z = 8.5`.
- P2, chamfer the plate's top loop, `d = 1.5`: `Exact` `15000 − 175.5`
  (`80d² − 4d³/3`), the plate's four walls trimmed through the (pl) rule.
- P1, chamfer the top and bottom loops in one call, `d = 1.5`: volume within
  its bound of `15739 − 180π`; both y walls (pl) and both x walls (sw)
  trimmed.
- P3, chamfer the boss top rim, `d = 1`: a `Cone`, volume within its bound of
  `16000 + 1111π/3`; chamfer the boss root, `d = 1`: a `Cone` that fills,
  volume within its bound of `16000 + 1141π/3`, `Edge.IsConvex` false on the
  root's trimmed circle, the plate top's hole of radius 6.
- P6c, chamfer the port mouth, `d = 1.5`: volume within its bound of
  `69400 − 135/2 − 9π/8`, the mouth's corners carrying apex cones as P2's.
- P7, chamfer the top loop, `d = 1`: an apex `Cone` at the reflex corner,
  volume within its bound of `17280 − 72π − (80 − (5 − π/4)/3)`.
- P8, chamfer a hole rim, `d = 1`: volume within its bound of
  `15280 − 10π/3`; chamfer the top loop, `d = 1`: four `Plane` and four
  `Cone` patches at G1 joins, volume within its bound of `15232 − 8π/3`.
- Bound fixture, shown to fail first: the trapezoid `(0, 0)`, `(100, 0)`,
  `(72, 45)`, `(28, 45)` extruded 10, with a blind pocket (a stacked
  receiver), whose top face's outer loop turns 58° at its base corners and
  whose slanted sides run along `(28, 45)`, of length 53, chamfered `d = 1`:
  every cap-level vertex's bound encloses its exact rational distance to the
  denoted contour corner; deleting the `F.delta` charge turns it red; the
  test records the leg. No float pair holds a 60° slope with a rational unit
  normal.
- Refusals: a partial loop outside the selected-chain fillet route → SL1; a chamfer of the top loop with
  one vertical edge → SL1; P1's top loop and its planar y = 0 wall's outer loop, which
  share an edge → SL1; P8's y = 0 wall's outer loop, whose top neighbour
  continues on a fillet arc → SL2; P8's top loop chamfered at `d = 3` → SX6
  (the fillet arcs' offsets vanish); the pocket mouth at `d = 5` → SX7 (the
  band reaches the floor); a loop on a face with `delta > 0` → SB1;
  P8's top loop with cap and side references → the two distinct bounded
  volumes of §4.2; a blind-pocket stack's outer edge and top loop with
  both reference choices → the three distinct analytic volumes of §4.2.

Route L (L-2):

- Every L-1 fixture's mesh is closed over every face of the body. P2's top
  loop (planes) meshes to exactly `15000 − 175.5` with a zero occupied-volume
  bound, and a `Cut` by a box over its corner leaves `14584.625`. P3's rim and
  root and P8's two chamfers are admitted: their mesh volumes lie within the
  published bound of the closed forms. P2's mouth, P6c's port mouth and P7's
  top loop hold apex cones at reflex corners, which DG4's rule does not admit:
  their meshes are export-only, within the chord times the curved faces' area
  of `15000 − 135/2 − 9π/8`, and a `Cut` over them refuses.
- P3's rim chamfer tessellates with one count on the cone's two circles;
  P8's loop chamfers export faceted STEP and P2's top loop analytic; `Verify`
  `Sound` on each.
- The undercut survey lists P3's rim cone, which faces up and out, under a
  pull along `−z` and clears the body along `+z`. The concave-radius survey
  decides P2's top loop (planes alone) as holding no concave feature, and
  reads P2's mouth and P3's root undecided: their cones carry a non-zero
  stamped departure. A clearance pair against P2's result reads `Suspect`
  where its boxes do not decide it.

## 10. PR split

Each PR ships code, tests and the documentation its lifted refusals touch:
this document's increment table, `doc.go`'s support map,
`docs/missing-features.md`'s Modify rows, `docs/brep-modify-design.md`'s
Table SB text for SB3/SB4/SB5/SB10, the functions' doc comments, a
`docs/layout/` row per new root file, and `.github/test-shards.txt`. This
document ships with PR 0.

| PR | Model | Lands | Files and functions | Proves | After |
|---|---|---|---|---|---|
| **0** | Sonnet, file-by-file | the hooks: `classBOfPayloads` (class B's entry over two payloads and one placement, `tryClassB` rewritten over it); `recognisePrism` split into `readPrismCaps` (P1, P3, P4) and the wall classifier, both exported to route S; `reverseSegment` over a `CircleSeg` and a `brepFace` reversal helper; `buildCapBand` taking its cap-level coedges from the caller; `modifyBrepReceiver`'s two new arms calling stubs that return SB10's and SX16's refusals | `classb.go`, `brep_modify_prism.go`, `brep_modify_edge.go`, `brep_payload.go`, `capblend_geom.go`, `capblend_moments.go`, `brep_modify.go` | every existing brep-modify, class-B and cap-blend fixture bit for bit | — |
| **S-1** | Opus, proof spec | route S with removed caps: Table TC, the gates, TC7, the private cuts, the rim assembly, SG1–SG7 | `brep_shell.go` (new: `readThroughCut`, `shellThroughCut`, `throughCutRims`), `brep_modify.go`, `shell.go` (doc comment), `brep_shell_internal_test.go`, `apitest/brep_shell_test.go` | §9's S-1 fixtures | 0 |
| **S-2** | Opus, proof spec | route S with a removed wall run: `sideOpeningRegions`'s cavity section as `A ⊖ t`'s section, the rim at a pierced wall as bands, SG5 narrowed | `brep_shell.go`, `brep_shell_rim.go`, `internal/throughshell/`, `shell_opening.go`, tests | §9's S-2 fixtures | S-1 |
| **L-1** | Opus, proof spec | route L: Table LB, the record rewrite and trims, `loopBands` on `brepPayload`, the band attached at build, measurements, SL1–SL4; tessellation, clearance, the surveys and STEP refuse a body with `loopBands` until L-2 (`ErrUnsupported`, naming L-2) | `brep_modify_loop.go` (new: `brepLoopRoute`, `admitLoops`, `rewriteLoopFaces`), `brep_loop_band.go` (new: the `buildCapBand` adapter and `readBandMass` sums), `brep_payload.go`, `brep_measure.go`, `chamfer.go` (doc comment), `brep_modify_loop_internal_test.go`, `apitest/brep_loop_chamfer_test.go` | §9's L-1 fixtures | 0 |
| **L-2** | Sonnet, file-by-file: each consumer copies its cap-blend twin | DG3 (`tessellateBrep` with imposed samples and `emitCapBand`), DG4 (`capBlendOccupiedVolumeAdmission` per band in `requireVolumeProvingPayload`), DG6 (`addBrepFaces` returns no model), DG7/DG8 (`brepUndercuts`, `brepMinRadius` reading the patch faces through `capblend_survey.go`'s readers), DG10 (the STEP writer's cone fallback, which needed no change), DG12 | `tessellate_brep.go`, `tessellate_brep_band.go` (new), `tessellate_capblend.go`, `boolean.go`, `brep_loop_band.go`, `brep_measure.go`, `brep_payload.go`, `survey.go`, tests | §9's L-2 fixtures | L-1 |

S-1 and L-1 run in parallel after PR 0: they add disjoint files and touch
`brep_modify.go` only through the arms PR 0 placed. S-2 follows S-1; L-2
follows L-1 and runs in parallel with S-2.

Increment table — what still refuses after each PR:

| After | Still refused |
|---|---|
| 0 | everything Table SB refuses today |
| S-1 (landed) | a removed wall (SG5); every shell of §6; every route L loop |
| S-2 (landed) | every shell of §6; every route L loop |
| L-1 (landed) | every consumer of a route L body but mass properties, `Verify`'s structural audit and gate, placement and later modify ops (L-2); §6 |
| L-2 (landed) | §6; a banded body's mesh boolean where a band holds an apex cone (DG4) |
