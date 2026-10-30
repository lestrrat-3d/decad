# Brep Modify Design

How `Fillet`, `Chamfer` and `Shell` take an analytically
trimmed boolean result — a `brepPayload` (`agent-docs/general-boolean-design.md` §4,
"general-boolean §N" below) — and, through its face view, a
`stackedPrismPayload` (`agent-docs/stacked-prism-design.md`). Companion to
`agent-docs/modify-design.md` ("modify §N"), which owns the three call contracts,
the base gates, the section rewrite and its §5 audit, and to
`agent-docs/modify-reach-design.md` ("reach §N"), whose RX3/RX7 rows and SX10/SX16
refusals this document narrows. Nothing here changes a prism receiver's
behaviour, and a `facetedPayload` receiver keeps reach SX9.

Five tables are normative:

| Table | Owns | Section |
|---|---|---|
| **RB** | which boolean receivers a modify op takes, and through which route | §3 |
| **EB** | which edges route E admits, and what each adjacent face must be | §5.1 |
| **SB** | every refusal this document adds, with its sentinel, and the gate order | §6 |
| **BB** | result payloads, topology and roles | §7 |
| **DB** | what every downstream consumer reads | §8 |

## 1. Problem

`Fillet`, `Chamfer` and `Shell` refuse every boolean result (reach RX6/RX7,
`agent-docs/missing-features.md`'s first row). "Cut a hole, then round or chamfer
the edges" fails on most machined parts. Reach §11 names the only substitute: a
proof over the operands, taken before the boolean retires them.

The brep record makes this fixable without a general kernel. Every face of a
`brepPayload` is a planar region or one line/arc/circle segment swept along an
axis, every face frame is a signed permutation of one reference frame
(general-boolean §4.1), and every planar region is a `ProfileRecord` — the same
record modify §2 rewrites. So a modify op on a brep is one of two things:

- **Route P — the brep is a prism in disguise.** A cross-drilled box is a
  plate-with-a-hole extruded along the hole's axis; a keyway rod and a cross
  slot are prisms along the tool. The op recognises the prism from the
  record and runs the shipped prism op on it. Every prism result follows:
  lateral fillets, complete cap-loop chamfers (the hole-mouth deburr), tubes
  and cups.
- **Route E — an axis-parallel straight edge.** The edge's two end faces are
  planar faces across the edge's axis. Their corner rewrites build the blend
  only when their material wedge agrees with the wedge between the adjacent
  faces at the edge. The blend surface is one more swept face. The result
  stays a `brepPayload`.

Scenes (general-boolean §2, §9) and what each route unlocks:

| Scene | Body | Route P (axis) | Route E |
|---|---|---|---|
| S1 cross-drilled box | brep | y: 4 lateral edges; hole rims (chamfer); shell | the 8 straight edges along x and z |
| B1 square cross hole | brep | y: 4 outer and 4 hole edges; shell | edges along x and z |
| S11 three cross holes | brep | y: lateral edges, 6 rims; shell | x/z edges |
| S11 + hole down the caps | brep | none | every straight edge |
| B2 keyway, D-flat | brep, `delta > 0` | x: lateral edges whose selection blends every crossing corner (§4.4) | SB1 |
| A1 flush boss, T-face | brep | none | boss edges, plate edges |
| A1 crossing round boss | brep, `delta > 0` | SB1 | SB1 |
| A1 L bracket (axis-aligned crossing prisms) | brep, `delta = 0` | y: the symmetric chamfer of an edge along y; shell | two-distance and angle chamfers (asymmetric requests take E/L) |
| blind rectangular pocket | stacked → face view | none | pocket corners (concave), floor edges |
| counterbore | stacked → face view | none | rims as complete loops (`agent-docs/loop-fillet-design.md`) |
| boss on plate (stacked) | stacked → face view | none | boss edges; root circle as a complete loop (`agent-docs/loop-fillet-design.md`) |

A complete-loop fillet with independent straight edges builds through route V
(`agent-docs/vertex-blend-design.md`). A curved rim or a complete loop without
single edges uses route L (`agent-docs/loop-fillet-design.md`).

## 2. Routes and dispatch

Each op keeps modify §4's stage 1 (S17, the sheet refusal, options, S15, S13 or
S14, S16) and reach SX10 for a `capBlendPayload`. Where the ops today cast to
`prismPayload` and refuse a brep with SX16, they call `brepmodify.Receiver`
(`internal/brepmodify/prism_route.go`):

```
receiver payload           → record handed to the brep route
prismPayload               → prism path, or route E face view for one straight cap edge
brepPayload                → itself
stackedPrismPayload        → breprecord.OfStacked(sp); its refusal is SB2
cupPayload, revolve, loft… → unchanged (S3, RX2, …)
facetedPayload             → SX9, unchanged
```

The brep route: SB1, except §4.4's displaced reading; then route P for every
reference axis in order; then
route E or `agent-docs/modify-general-design.md`'s route L (Fillet/Chamfer), or
route S (Shell; modify-general §3), whose refusal leads with SB3 or SB10
where the record reads as no through-cut record either. Route E takes single
straight edges along reference axes, no two sharing a vertex; route L takes
every other Fillet or Chamfer selection (modify-general §4.1). A record
carrying route L chamfer bands reads as no prism. Route P is tried first because its
result is a `prismPayload`, which every consumer and every further modify op
already takes. The two routes never build the same edge differently: a lateral
edge of the recognised prism and the same edge under route E are the same
cylinder or plane; route P merely records the body as the prism it is.
An asymmetric chamfer on a brep skips route P and reaches route E's
reference-face mapping (§5.1) or route L's complete-loop mapping
(`agent-docs/modify-general-design.md` §4.2). A selection outside those routes
remains SX16.

### 2.1 A stacked shell's rounded cavity: the faceted view

A `brepPayload` that carries a boss shell or pocket shell band
(`breprecord.Payload` `BossShell` or `PocketShell`; modify-general §3.1a–§3.1c,
§3.1e) has no exact modify route: no route above rewrites its cylinder,
sphere or torus band patches. A `Fillet` or `Chamfer` of it runs, before the
brep route, on the body's faceted view (`shellViewBlend`,
`internal/brepmodify/shell_view.go`):

1. `booleaneval.View` tessellates the body at `VerifyAll`, at the chord the
   pair `(b, b)` derives, and gives each face of the body one group, which
   `facetedrecord.SourceTag` fills exactly as a mesh Boolean fills an
   operand face's group. Planes keep their exact plane. A band patch keeps its
   cylinder, sphere or torus where `SourceTag` admits the face, and is
   `Faceted` otherwise. The view's vertex bounds, volume allowance and area
   slack are the tessellation's own; nothing is cut, welded or moved.
2. Each selected edge maps to the one view edge between the images of its two
   faces whose two ends equal its own ends exactly. An edge with no such view
   edge, or with several, leaves the selection unmapped.
3. `retainedblend.Fillet` or `retainedblend.Chamfer`
   (`agent-docs/retained-edge-blend-design.md`) builds the result from the view.
   Its strip check (SR7) reads every other facet of the view, the cavity
   walls and band facets included, as an obstacle, so a blend that would
   break through the wall into the cavity refuses.

Only a mapped selection that the retained blend recognizes and builds
succeeds. A selection the retained blend recognizes but cannot build refuses
with the retained blend's own row (Table SR of
`agent-docs/retained-edge-blend-design.md`). On the box below, a cavity
vertical or floor edge refuses with "an end face's corner does not match the
edge's convexity", the opening's inner rim with "a convex edge ends where a
face is reflex", and an edge at the band's ring with "an edge end meets a face
that is not a retained plane". An unmapped selection, or one the retained
blend does not recognize, falls through to SB11. A placed shell whose held
vertices do not equal its view's vertices exactly maps nothing: the box shell
translated by 12.7 along x, or turned 30° or 90° about z by
`r3.RotationAround`, refuses with SB11. A `Shell` that removes faces of such a
body is SB11 without a view, and a closed one is modify-general SG2. The result
is a `facetedPayload`: its volume, area and box bounds grow by the view's
band chords, and every later reading, modify op and Boolean reads its held
mesh. The view's root band takes the ring count of loop-fillet §7.1, which
covers each cell's twist, so for the 60×40×20 box with a 20×20×10 boss
shelled `t = 2` and filleted at `r = 3` the volume bound is about ±2.4 mm³ and
the area bound about 10 mm², both below `Verify`'s default relative limits,
and `Verify` reads the result `Sound`.

For the box above, open at the boss top, `Fillet` of the four outer box
verticals at `r = 3` leaves `√2·t − (√2−1)·r = 1.586` mm of wall on each
corner's diagonal and removes `4·20·9·(1 − π/4)` mm³. At `r = 7`, past
`√2·t/(√2−1) = 6.83`, the strip meets the cavity and SR7 refuses.

## 3. Table RB — receivers

| RB | Receiver | `Fillet` / `Chamfer` | `Shell` |
|---|---|---|---|
| **RB1** | `brepPayload`, `sectionDelta() == 0` | route P, else route E (Table EB) or route L (modify-general §4); an asymmetric chamfer skips P | route P, else route S (modify-general §3) |
| **RB2** | `stackedPrismPayload` whose `breprecord.OfStacked` succeeds (one region per slab, not a group) | as RB1 over the face view; asymmetric references use the source-face role at each record index | as RB1 |
| **RB3** | `brepPayload` or stacked receiver with a section displacement | route P's displaced reading (§4.4) for a lateral Fillet or symmetric Chamfer that passes Table PD; otherwise SB1 | SB1 |
| **RB4** | `facetedPayload` | reach SX9 (permanent) | reach SX9 |
| **RB5** | `brepPayload` carrying a boss or pocket shell band | the faceted view's retained edge blend (§2.1), else SB11 | SB11; a closed shell is modify-general SG2 |

RB1's zero-displacement rule is prism-boolean §13's displaced-receiver rule,
the one `prismrecord.Payload.RequireExactSection` enforces for a prism: a rewrite of a record that
only denotes its body to within a displacement has no proven displacement of
its own. Level displacements (`z0Delta`/`z1Delta`) pass through untouched;
no construction below moves a level.

The receiver is keyed on its payload, not its history: a transformed brep reads as
the brep it was placed from.

## 4. Route P — the prism in disguise

### 4.1 Recognition

For reference axis `k` (the reference frame is `bp.faces[0].frame`; every face
has an `Embed`), the body reads as a prism along `+k` when all of P1–P5 hold.
Every test is an exact comparison of recorded floats and records; nothing is
sampled or solved.

| P | Condition |
|---|---|
| P1 | Exactly two planar faces have `Embed.Axis[2] == k`. Call the one whose outward normal is `−k` **bottom** and the other **top**; their reference levels (`Sign[2]·z0`) satisfy `zlo < zhi`. |
| P2 | Every other face is one of: **(a)** a swept face with `Axis[2] == k`, both levels in reference coordinates exactly `{zlo, zhi}`, empty `side0`/`side1`; **(b)** a planar face with `Axis[2] != k` whose region is one loop of four `LineSeg`s over natural ranges, each along one of its in-plane axes, whose reference `k`-coordinates are exactly `zlo` and `zhi` (two segments at each); **(c)** a swept face with `Axis[2] != k`, empty splits, whose wall is a natural-range `LineSeg` along one in-plane axis with reference `k`-coordinates exactly `zlo` and `zhi` at its two ends. (b) and (c) state one rectangle two ways; (c) is read by restating it as (b) (§5.2). |
| P3 | The prism frame `F` is top's frame when its `Sign[2] == +1`, else bottom's when its `Sign[2] == +1`, else `brepgeom.AxisFrame(ref, k)`: the right-handed signed permutation with `N = +k`, `U` the next axis, `V` the one after. |
| P4 | top's region re-expressed into `F` (§5.4's map; a reflecting map re-winds the loops as general-boolean A4 states) is the section `S`. bottom's region re-expressed the same way equals `S`: equal outer loops, and each hole of one equals exactly one hole of the other. Two loops are equal when one's segment records, in order, are the other's from some starting segment: a boolean states the two caps of one section from different first segments. `S` keeps top's hole order. |
| P5 | Every P2(a) wall's segment re-expressed into `F`, re-wound as a one-segment loop under a reflecting map, equals, as a directed walk, one segment of `S` (a whole circle by centre, radius and sense). Every P2(b) rectangle projects along `k` onto `F`'s plane as one `LineSeg` equal, as a set, to one segment of `S`, and the rectangle's outward normal mapped into `F` is that segment's right-hand unit normal (exact for an axis-aligned segment). Each segment of `S` is claimed exactly once. |

P5 is a reject-only cross-check of what the receiver's own pairing already
implies (general-boolean §4.2: a planar loop segment pairs with one rim or one
other planar segment); it catches a wall recorded with its material on the
wrong side and nothing else.

A P2(b) or P2(c) face's levels are section coordinates of the prism, and
RB1 admits no section displacement, so such a face whose level carries a
displacement (`z0Delta`, `z1Delta`) reads as no rectangle.

A4 carries a line or arc over a narrowed range under a reflection only to
within a rounding (its walked endpoints), so a region or wall that holds one
and needs a reflecting map reads as no prism along that axis.

The recognised prism is
`prismPayload{profile: S, frame: F, z0: zlo, z1: zhi, xform: bp.xform}` in
`F`'s coordinates, with `z0Delta`/`z1Delta` the largest level displacement any
face states at that level, and `sectionDelta` zero (RB1; §4.4's displaced
reading carries the record's). It is never stored;
it is the op's receiver for the rest of the call.

### 4.2 Selection and dispatch

Axes are tried in order `0, 1, 2`. For each axis that reads as a prism the
selection is classified against that prism exactly as the prism op classifies
it today — `modifyselect.MatchCorner` for lateral edges, `brepmodify.ClassifyChamferSelection`
for complete cap loops, `classifyRemovedCaps` for a shell's openings — with one
change: the two helpers that find caps by `capStart`/`capEnd` roles take the
two cap `*Face`s as arguments instead (for a prism body, the faces those roles
name; here, `facesByRole(b)[bp.faces[top].role]` and bottom's). The first axis
whose classification admits the selection is taken, and the call continues in
the prism op from modify §4's stage 2 with the recognised prism as `pp`. Every
later gate, construction, payload and role is the prism's own. A stacked
receiver's body faces carry the stacked body's roles, not its face view's,
so its route P names no cap face: a selection on its caps classifies as no
cap edge, and its removed faces as no caps.

When some axis reads as a prism but none admits the selection, a Fillet or
Chamfer falls to route E, and a Shell to route S (modify-general §3), which
refuses with the prism path's own S2 (SB3) where the record reads as no
through-cut record either. When no axis reads as a prism, a Shell takes
route S, which refuses with SB10 where the record reads as no through-cut
record.

`modifyselect.MatchCorner` compares the selected edge's vertex positions with
`pp.Point(...)` within `1e-6`; the recognised prism lifts through `F` and
`bp.xform`, the same placement the brep's vertices were lifted through, so the
match holds.

### 4.3 Why this is not general-boolean §8's "re-sweep"

General-boolean §8 forbids a boolean from re-sweeping a box along the tool's
axis: the roles would change under the caller and the next cut would find no
prism. Neither objection reaches a modify op. Its result mints fresh roles
anyway (modify §11), and the prism it returns composes as any prism does: a
next perpendicular cut takes class B and a co-directional one class A.

### 4.4 A displaced class-B receiver: carriers, not vertices

A class-B crossing result (general-boolean §3 B.3: a D-flat or keyway on a
shaft) carries a section displacement `δ = bp.SectionDelta() > 0`: every
cylinder–plane crossing is one float from the keyed crossing table, within its
proven charge of the true crossing of the operand's own circle. Route P still
reads such a record as a prism (P1–P5 are identity comparisons on the
recorded data). This section is the one reading of a displaced receiver that
builds: a symmetric `Fillet` or `Chamfer` of lateral edges. Every other
displaced call is SB1.

**Claim X (where a class-B section is displaced).** Let `S` be a route-P
section of a class-B crossing record and `S*` the section the record denotes.
`S` and `S*` have the same walks in the same order, on the same carriers. Each
line lies exactly on its plane (a P2(b) rectangle, whose level carries no
displacement), and each arc's centre is its cylinder's exact centre. They
differ only at junctions that involve a circular walk: there a junction point
of `S` lies within `δ` of the one in `S*`, and an arc's recorded radius lies
within `δ` of its cylinder's radius. A line–line junction meets two exact
levels and is exact.

**The record carries the exact cylinder.** `breprecord.Face.WallCarrier`
(`brepgeom.CircleCarrier`: a centre in the face's frame and a radius in
millimetres) is the operand circle a class-B crossing build's cylinder piece
lies on (general-boolean §4.1). Only a `CircleSeg` source sets it, so a wall
on an arc-walled operand records none. `brepgeom.PrismWallSet.Claims` returns,
per section segment, the face that claims it under P5 and that wall's carrier
mapped into `F`.

**Dispatch.** `Receiver` sends a displaced record to `displacedReceiver`. A
Shell, an asymmetric chamfer, an op with no corner construction, or a record
with route L bands is SB1. Otherwise route P runs over axes 0, 1, 2; a
reading takes PD2 and then the op's own classification, and the first that
passes is the route, with the prism's `SectionDelta` set to `δ` and the
carriers in `Route.Carriers`. No admitting reading is SB1 (with PD1's or PD2's
reason when some axis read as a prism); routes E, L, V and S never run. A
prism-path consumer that does not take the displaced arm still refuses through
`prismrecord.Payload.RequireExactSection`.

**Construction** (`internal/brepmodify/displaced_blend.go`). The Fillet and
Chamfer arms match corners as the prism path does, then PD3 and PD4, then
copy the corner loops with every circular walk on its carrier: the exact
radius, and the length that radius gives the walk's sweep. Centres are
already exact and lines are untouched. Each selected corner is built by
`offset2d.FilletWithRoots` (which also returns the cross product, both offset
roots and the arc-sense dot it decided on) or `offset2d.Chamfer` on those
walks. `offset2d.RewriteProfileBudget` rewrites the section from the carrier
walks; the audit runs S8, S6, S7 and S9 with PD6's widened cutbacks; the build
is `prismrecord.Build` with `SectionDelta` 0 for a fillet and `δ_out` for a
chamfer (PD7).

**Claim D (fillet).** Under PD1–PD6 the fillet of `S` equals the fillet of
`S*` up to the float evaluation of the same closed forms from the same exact
inputs, which modify §5 takes as every prism fillet's definition, so its
`SectionDelta` is 0. The offsets are those of the exact carriers. The displaced
corner enters only decisions: the turn's sign (S4 and the offset side; an arc
tangent at a point within `δ` of the true corner turns by at most
`θ = asin(δ/R) ≤ δ/(R − δ)`, held by PD5a), which root is nearest the corner
(PD5b), the connector's sense (PD5c), and S6's cutback comparison (PD6). By
PD4 every junction that involves a circular walk is a selected corner, so the
rewrite re-emits every circular walk from foot to foot about its exact centre
and no displaced point survives.

**Claim C (chamfer).** A chamfer's feet are set back along the carriers from
the recorded corner. A line foot moves by at most `δ`, and an arc foot by at
most `R·asin(δ/R) ≤ R·δ/(R − δ)` (a rotation about the exact centre). So the
record differs from the chamfer of `S*` by at most
`δ_out = max(δ, max over circular walks at chamfered corners of R·δ/(R − δ))`,
computed exactly and rounded up once, beyond the rounding the prism chamfer
also takes. The result states `δ_out`, so a later modify op refuses it as any
displaced prism. The setback's conversion bound is not charged, as on the
prism path.

Table PD — a PD1–PD4 or PD5 refusal is `ErrUnsupported` with SB1's text,
then the row and its reason (`... (brep-modify SB1); its displaced reading
(brep-modify §4.4) refuses: ... (brep-modify PDn)`). Stage 2a's op checks (a
Shell, an asymmetric chamfer, an op with no corner construction, a record with
loop bands) return SB1's text alone, with no row. PD6's refusal is modify S6,
and PD7 is a charge:

| PD | Condition (exact comparisons of recorded data unless stated) |
|---|---|
| **PD1** | Every selected edge is a lateral edge of the reading. A selection the op's classification refuses, a cap edge or a complete cap loop refuses with this row. (A Shell, an asymmetric chamfer or a record with loop bands never reaches the reading: SB1 alone.) |
| **PD2** | Every circular segment of `S` is claimed by a P2(a) wall whose `WallCarrier.Radius > 0`, and every line by a P2(b) planar rectangle, never a P2(c) restated wall; a circular walk covers one segment. |
| **PD3** | Each such segment's centre equals its carrier's bit for bit, `2δ < R`, and each recorded endpoint `P` satisfies `(R − δ)² ≤ |P − C|² ≤ (R + δ)²` over `big.Rat` (P5 makes the wall's endpoints the same points). A whole circle's radius equals the carrier's bit for bit. |
| **PD4** | Every corner at which a circular walk meets another walk is selected; the refusal names the corner's section point. |
| **PD5a** | At a selected corner with a circular walk: `|cross| > tol + θ_A + θ_B + 2⁻⁵⁰`, `θ = δ/(R − δ)` rounded up per circular walk, 0 for a line. |
| **PD5b** | Fillet only: two distinct offset roots' distances to the recorded corner differ by more than `2δ` plus `2⁻⁴⁸` of their sum. One root, or a double root, decides nothing. |
| **PD5c** | Fillet only: `|rot90(f_A − O) · t_A| > r·θ_A + 2⁻⁵⁰`. |
| **PD6** | S6 sees each cutback at a corner with a circular walk widened by `2δ'`: `δ' = δ` on a line walk, `R·δ/(R − δ)` rounded up on a circular one. A failure is modify S6. |
| **PD7** | Chamfer only: the result's `SectionDelta` is `δ_out` (a charge, never a refusal). |

Gate order of the displaced reading (stage numbers as in §6):

| Stage | Gates |
|---|---|
| 2a. record | RB dispatch; SB2; for `δ > 0`, a Shell, an asymmetric chamfer, an op with no corner construction or loop bands is SB1 alone |
| 2b. route P | P1–P5 per axis with the claim map; PD2; the op's classification; no admitting reading is SB1 |
| 3. arm | PD1 (a chamfer's cap edge or cap loop); corner matching; PD3; PD4 |
| 4. per corner | S4; S5 on the exact radius (fillet); PD5a; PD5b, PD5c (fillet) |
| 5. audit | S8, S6 with PD6's widening, S7, S9 on the rewritten section |
| 6. build | `prismrecord.Build`, `SectionDelta` 0 (fillet) or `δ_out` (chamfer) |

Existence gates keep their sentinels and come first: S4 and S5 are
`ErrDegenerate` (a `Fillet` of radius 10 on the D-flat has `R − r = 0`).

A stepped shaft's flat reads as no prism, so it stays SB1. A crossing on an
arc-walled operand records no carrier (PD2). A selection that leaves a
crossing corner unblended (PD4) would keep a displaced junction the record has
no per-vertex charge for.

## 5. Route E — axis-parallel straight edges

### 5.1 Table EB — the admitted edge

Route E runs for `Fillet` and `Chamfer` over the selected edge set `E`. Each
edge `e` must satisfy EB1–EB6 and the set must satisfy EB7.

| EB | Condition | Refusal |
|---|---|---|
| **EB1** | `e.Curve()` is `Line3` and its two vertices differ in exactly one reference coordinate, the edge's axis `a`. | SB4 |
| **EB2** | Each vertex `V` of `e` is incident to exactly three edges of the receiver's `brepTopology` (edges whose owner use starts or ends at `V`'s reference coordinates), and `e` is not a piece of a split side line (`side0`/`side1` of its swept face empty). | SB6 |
| **EB3** | At each vertex the third face `G` (the face using both other edges at `V`, not adjacent to `e`) is, after §5.2's restatement, a planar face with `Embed.Axis[2] == a`. `G0` is the end face at `e`'s lower `a`-coordinate, `G1` at its upper. | SB7 |
| **EB4** | Each face adjacent to `e`, after restatement, is one of: **(sw)** a swept face with `Axis[2] == a` whose side line is `e`; **(pl)** a planar face with `Axis[2] != a` that holds `e` as one loop segment whose two neighbouring segments are `LineSeg`s at constant `a`-coordinate. | SB8 |
| **EB5** | Every segment of `G0`, `G1` and the adjacent faces is over its natural range (`LineSeg` 0→1 or 1→0, `ArcSeg` likewise, `CircleSeg` whole), and no loop of those faces holds two consecutive segments on one carrier (collinear lines; co-circular arcs of one centre, radius and sense). | SB8 |
| **EB6** | `G0`'s two loop segments at `V0` are the traces of the two adjacent faces — each pairs, in the receiver's topology, with that face's rim (sw) or with its segment next to `e` (pl) — and they make a corner there (modify S4 otherwise). `G1` likewise at `V1`. | SB8 / S4 |
| **EB7** | No two edges of `E` share a vertex. | SB5 |
| **EB8** | For a Fillet or Chamfer, the corner in each end face, or its complement when the other end face lies on its outer side, has the same convex or reflex turn as the solid between the two faces beside `e`. | SB8 |

EB5's second half exists because the corner rewrite re-emits a coalesced walk as
one segment (`rewriteLoop`), and a vertex another face placed inside that walk
would vanish from the record; the result would then fail to pair and refuse
honestly, but late and with the wrong reason.

EB1 reads a selected edge in the record by lifting each recorded line edge's
two ends through the reference frame and the placement and matching them to
the selected edge's vertices within `1e-6`, as `modifyselect.MatchCorner` does for a
prism. The match identifies the edge and admits no geometry; a selected edge
that matches no recorded edge, or several, is SB6.

For an asymmetric chamfer, route P is skipped. Route E admits an independent
straight edge of a `brepPayload` or stacked face view under the same EB rows.
The option's resolved reference face identifies one of the edge's two public
adjacent faces by its record index. A brep result names that face `face(i)` or
`wall(i)`; restatement preserves `i`. A stacked face view carries one source
role per record face: `slab(start).region(region).side(loop,segment)` for a
wall column, `capStart` or `capEnd` for a cap, and `floor(k,e)` or
`ceiling(k,e)` for an exposed interface. `breprecord.joinStacked` changes only
segment endpoints, so it keeps column and segment indices. A missing or
ambiguous source role is SX16.
EB6 then pairs each end-face walk with one adjacent record face. The walk
paired with the reference face takes the positional distance; the other takes
`otherDistance`. A missing or ambiguous public-to-record face index is SX16.
Route L maps complete-loop references to its cap face or each neighbouring
record face (`agent-docs/modify-general-design.md` §4.2), using the same source-role
mapping for a stacked receiver.

### 5.2 Restatement of a straight wall as a planar face

A swept face whose wall is a `LineSeg` is the rectangle it sweeps, and the
record may state it either way. Route E restates one as a planar face wherever
it needs the face as an end face (EB3) or the face holds `e` as a rim (an
adjacent face with `Axis[2] != a`). Restatement runs in two passes over the
whole call: the first pass visits every selected edge, collects the faces it
needs restated and refuses where one cannot be (SB7 for an end face, SB8 for a
rim-adjacent one); the second pass classifies every edge against the restated
record. A wall that is one edge's end face and another edge's side face is
therefore one planar face for both.

`brepgeom.Restate(face, ref)`: the wall is a `LineSeg` over its natural range,
axis-aligned in its frame (exactly one of `du`, `dv` is zero), with empty
`side0`/`side1` and neither level displaced (`z0Delta`, `z1Delta` zero); an
oblique, split or displaced wall cannot be restated. A level of the swept
face is a section coordinate of the planar one, and RB1 holds section
displacement at zero, so a displaced level has no place in the restated
record; route P's P2(c) refuses the same wall for the same reason. The four corners
`(Start, z0), (End, z0), (End, z1), (Start, z1)` are mapped to reference
coordinates (`Embed.Canon`), the constant reference axis and its level give the
plane, and `brepgeom.PlanarFrame(ref, axis, sign)` is the right-handed frame
whose `N` is the wall's outward normal — the same frame `StackedWallFrame`
builds for `internal/stackedbrep/faces.go`'s wall faces. The region is the one loop
through the four corners in the wall's own boundary order — rim at `z0`
forward, side line up, rim at `z1` backward, side line down — the order
`brepgeom.Build` already walks, which is counter-clockwise from outside; the
face is `outward: true`, its level and section displacements the wall's
section displacement, and it records the wall's own frame normal as its
`sweep`, so a line it shares with another wall of that sweep reads the turn
(general-boolean §4.2). `Restate` refuses a rectangle that turns clockwise in
the new frame, a reject-only check of the orientation reading. Every
coordinate is a recorded float moved by a signed permutation, so the restated
face pairs with every neighbour by identity exactly as the swept one did: the
second pass reads the restated record's topology again, its closure proving
the pairing, and matches every selected edge in it again, since the record
numbers its edge uses afresh.

### 5.3 The construction

For each admitted edge `e` with axis `a`, adjacent faces `F1`, `F2` and end
faces `G0`, `G1`:

1. **Corner blend in `G0`.** `G0`'s region is walked into `cornerLoop`s as
   `offset2d.ProfileCornerLoopsBudget` does for a section; the corner at `V0` is the walk
   junction whose point maps to `V0`'s reference coordinates (`brepgeom.BrepCornerAt`,
   an exact comparison, replacing `modifyselect.MatchCorner`'s lifted-point match).
   `computeFillet(loop, corner, r)` or `computeChamfer(loop, corner, dA, dB)` runs
   unchanged: S4 for a smooth or cusped junction, S5 for a fillet whose offsets
   never meet, then the feet and connector. The blend is assigned to carriers,
   not to walk order: `fF1` is the foot on the walk that pairs with `F1`,
   `fF2` on `F2`'s, and the centre `c` (fillet only).
2. **The same blend in `G1`.** The blend is recomputed on `G1`'s corner at
   `V1`. Mapped to reference coordinates with the `a` coordinate dropped, its
   centre and two feet must equal `G0`'s bit for bit; a disagreement is SB9.
   The agreement is expected, not assumed: the operations `computeFillet` and
   `computeChamfer` perform on coordinates — differences, sums, products by
   `±1`, `hypot`, the offset solve — commute with a signed permutation of the
   inputs, and the two end faces hold the same two carriers; SB9 refuses
   wherever that expectation fails. (The `Atan2`-based cutbacks may differ by
   rounding; each face's audit reads its own.) `G1`'s rewrite takes its own
   recomputed blend, so its connector is already walked in `G1`'s loop sense;
   nothing is transported from `G0` into `G1`'s record. Steps 4 and 5 read
   `G0`'s feet and centre.
   Before a route E Fillet or Chamfer rewrites a face, EB8 compares each corner's turn
   with the interior material wedge. The adjacent faces' outward normals
   `n0` and `n1` and the first face's oriented solid-boundary tangent `t` make the material
   turn `sign((n0 × n1) · t)`: positive is convex, negative is reflex. An inward
   planar face reverses its recorded loop tangent for this test. A turn
   within `1e-9` of tangent is undecided and refused. `Edge.IsConvex()` is a
   walked-boundary reading, so it does not make this decision. A single edge
   that fails EB8 may instead take route L's selected planar-loop band; a
   failed band leaves the call refused.
3. **Rewrite `G0` and `G1`.** `rewriteProfileBudget` over the face's
   `cornerLoop`s and its `blendAt` map, as the prism does. A face may carry
   several corners from several edges of `E`.
4. **Trim `F1`, `F2`.** (sw): the wall's endpoint at `e` becomes the foot
   `fF`, re-emitted by `walkSegment` in the face's frame (`fF` mapped through
   the face's embed); its levels and splits are unchanged. (pl): `e`'s segment
   becomes the `LineSeg` between `fF` at `zlo` and `fF` at `zhi`, and its two
   neighbouring segments end at those points instead of at `V0`/`V1`. The foot
   is the same float in every face that uses it; no face recomputes it.
5. **The blend face.** One swept `brepFace` in `G0`'s frame: `wall` the
   connector in that frame, `z0`/`z1` the two end levels in that frame's
   coordinates (`G0`'s own level and `G1`'s reference level through `G0`'s
   embed, ascending), `z0Delta`/`z1Delta` those levels' displacements,
   `delta` zero, `blend` `"fillet"` or `"chamfer"`. Its wall walks with the
   solid's material on its left. `G0`'s connector already walks with `G0`'s
   region on its left, and near `V0` the solid between the two end faces is
   bounded by `F1` and `F2` alone, so its section there is `G0`'s region
   when `G1` lies on `G0`'s inner side (against `G0`'s outward normal) and
   the rest of the plane when `G1` lies on `G0`'s outer side. The wall is
   `G0`'s connector in the first case and the connector reversed in the
   second. The same two feet and the same circle serve both end faces
   whatever their own corner's convexity — a boss top's convex corner and a
   floor's reflex corner at one boss edge compute one centre, since modify
   §6's rule offsets into the material at a convex corner and away from it at
   a reflex one. `e.IsConvex()` cannot decide the walk sense: a line two
   planar faces of different sweeps share reads its convexity from a loop's
   role (general-boolean §4.2), which a concave edge need not match.
6. **Audit.** Every rewritten planar face runs modify §5's audit on its final
   record: S8 (orientation), S6 (every walk's claims from both ends sum
   strictly below its length — a corner's cutback and a (pl) neighbour's
   shortening are both claims), S7 (crossing or contact), S9 (nesting). Every
   trimmed (sw) wall runs S6 over its two ends. A claim is accounted once per
   walk end whichever edge made it.
7. **Assemble.** The result record is the receiver's faces in order, restated
   and rewritten in place, with every blend face appended, `stack` nil,
   `assignRoles`. `breprecord.Falsify` and `breprecord.TopologyContext` run: an edge
   that does not pair is `ErrUnsupported` (general-boolean §4.2's closure by
   counting — the proof that the rewrite closed). `breprecord.Build` builds and
   measures the body; a non-positive volume is its own `ErrDegenerate`.
   `Document.commit` retires the receiver, as every modify op does.

Every pairwise interaction of two admitted blends is caught by step 6 or step
7: two blends on one wall or one planar segment meet in S6; two blends whose
rims cross or touch meet in S7 inside the end face that holds both rims; and a
rewrite that leaves any edge unmatched fails the closure count.

### 5.4 Maps between faces, and the 2D answers

Every coordinate route E emits is a recorded float, or a float the corner
blend computed once, carried between faces by signed permutations of the
reference axes (`Embed.Canon` then `Embed.Local`, each exact). The 2D map
between two faces with normals on one axis is the composition restricted to
the two other axes; it reflects when its permutation sign times its two signs
is `−1`. `brepgeom.MapSegment(map, seg)` applies it to a record: a `LineSeg`
maps its endpoints; an `ArcSeg` maps centre and endpoints and, under a
reflecting map, swaps `Start`/`End` and reads its range as `1 − t`; a
`CircleSeg` maps its centre and, under a reflecting map, flips `CCW` and reads
its range as `1 − t`. On a natural range `1 − t` swaps `TStart`/`TEnd`, which
is general-boolean A4's re-winding rule applied to one segment.

Route E asks `sketch` for nothing. The rounded corner, the chord and the
trimmed wall exist in no sketch; they are decad's own geometry synthesised from
decad's own records, the class modify §5 already draws for the prism rewrite,
and decad proves them with the same exact closed-form audit. No upstream claim
exists to bless, and no residual admits anything: the only tolerance in the
construction is `filletTol`'s S4/S5 degeneracy floor, which refuses.

## 6. Table SB — refusals and gate order

Modify §1's test picks every sentinel: a body that does not exist is
`ErrDegenerate`; one this evaluator cannot build is `ErrUnsupported`.

| SB | Call | Exists? | Sentinel |
|---|---|---|---|
| **SB1** | a brep or stacked receiver with `sectionDelta() != 0` outside §4.4 | yes; its rewrite has no proven displacement | `ErrUnsupported`, naming the displacement as `prismrecord.Payload.RequireExactSection` does |
| **SB2** | a stacked receiver `breprecord.OfStacked` refuses (a prism group, several regions in one slab; a stack enclosing a cavity, a closed shell) | yes | that call's `ErrUnsupported` |
| **SB3** | route P reads a prism along some axis, a Shell's removed faces are not its caps, no other axis admits them, and route S reads no through-cut record along any axis (modify-general Table TC) | yes | modify S2, with SG3's reason |
| **SB4** | route E reading an edge that is not a straight line along a reference axis; only single straight edges reach route E, and every other Fillet or Chamfer selection takes route L (modify-general §4, `agent-docs/loop-fillet-design.md`), which builds a hole rim, a boss root or a cornered loop or refuses with Table SL or SF | — (a falsifier) | `ErrUnsupported` |
| **SB5** | selected single straight edges share a vertex outside route V's complete-loop or selected-chain fillet (`agent-docs/vertex-blend-design.md` §2); a mixed `Chamfer` still takes SL1 | yes | `ErrUnsupported` (SL1's text) |
| **SB6** | an edge vertex with other than three incident edges, or an edge that is one piece of a split side line | yes | `ErrUnsupported` |
| **SB7** | the third face at an edge vertex cannot be restated as a plane across its axis | yes outside P8's exact front top edge (§4.3c); complete-loop fillets have their own route | `ErrUnsupported` |
| **SB8** | an adjacent face outside EB4/EB5, or a Fillet or Chamfer whose end-face corners disagree with the interior material wedge (EB8) or whose wedge turn is undecided | yes | `ErrUnsupported` |
| **SB9** | the two end faces' blends disagree in reference coordinates | — (a falsifier) | `ErrUnsupported` |
| **SB10** | Shell of a brep that reads as a prism along no axis and as no through-cut record (modify-general Table TC) | yes; the three-dimensional offset puts a sphere at a reflex vertex, a torus around a reflex circle and an elliptical edge where two reflex edges meet, none of which this record holds | `ErrUnsupported`, with SG3's reason |
| **SB11** | RB5: a `Shell` that removes faces, or a `Fillet` or `Chamfer` whose selection §2.1 leaves unmapped or the retained blend does not recognize; a recognized selection the retained blend cannot build takes its own SR row instead | yes for a thin enough second shell or a small enough blend | `ErrUnsupported`: "this evaluator does not modify a stacked shell's rounded cavity" |

SB7 includes a cylinder, a plane along the edge's axis, an oblique, split or
level-displaced wall, an earlier blend, and a trimmed wall ending on a loop
band patch. Only modify-general §4.3c's exact P8 edge bypasses this gate.

When the third face is curved and the selected edge ends at a junction that
`tangentchain.Continues` proves tangent (an exact `DegYes`) to an unselected
edge, the refusal names that junction and the workaround: "the selected edge
ends at the tangent junction `x,y,z` whose continuation is not selected;
`WithTangentChain` selects the whole chain (brep-modify SB7)", the mesh route's
SM2 wording. An undecided or non-tangent junction keeps the curved-face text.

Base S4, S5, S6, S7, S8, S9 keep modify §4's meanings per end face and per
trimmed wall. Reach SX16 is replaced: a brep receiver either builds here or
refuses with one of the rows above.

Gate order for a brep receiver, after modify §4's stage 1 and reach SX10.
The exact P8 front top Fillet of modify-general §4.3c is tested before the
record gate; a mismatch falls through to the table unchanged:

| Stage | Gates |
|---|---|
| 2a. record | a Fillet or Chamfer of an RB5 receiver takes §2.1's view first; RB dispatch; SB11; SB2; SB1, except a displaced receiver that takes §4.4's reading (its own gate order) |
| 2b. route P | P1–P5 per axis; the prism path's own stage 2 onward where an axis admits; a shell with no admitting axis takes route S at 2c, and SB3 where a prism read and route S reads none |
| 2c. route E entry | route S (Shell; modify-general §3.4); independent straight edges take route E; a `Fillet` of complete loops with independent straight edges takes route V (`agent-docs/vertex-blend-design.md` §2); other selections take route L or refuse; EB1/SB4 and EB7/SB5 apply to route E's set |
| 3. edge topology | EB2/SB6; the restatement passes (§5.2): EB3/SB7, then EB4/SB8; EB5/SB8; EB6 |
| 4. construction | per edge: S4, S5 in `G0`; the recomputation in `G1`, SB9; EB8/SB8 before rewriting |
| 5. audit | per planar face: S8, S6, S7, S9; per trimmed wall: S6 |
| 6. closure | `breprecord.Falsify`, `breprecord.TopologyContext`, `breprecord.Build` |

The existence question is asked first wherever two gates meet one input: S4
and S5 (stage 4) decide whether the blend exists before any trim is measured
(S6, stage 5), and S8 leads the audit, as modify §4 orders them.

## 7. Table BB — results and roles

| BB | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BB1** | route P fillet/chamfer of lateral edges | `prismPayload` over the rewritten `S` | modify B1 | modify B1's: `side(i,j)`, `capStart`/`capEnd`, `fillet(i,j)`/`chamfer(i,j)` |
| **BB2** | route P complete cap-loop chamfer | `capBlendPayload` | reach BX3 | reach BX3's |
| **BB3** | route P shell | `prismPayload` tube or `cupPayload` | modify B2–B6 | modify B2–B6's |
| **BB4** | route E fillet/chamfer | `brepPayload`, `stack` nil | the receiver's faces, rewritten and restated in place, plus one swept face per edge; every edge on exactly two faces | `face(k)`/`wall(k)` by result index; each blend face also carries `fillet(k)`/`chamfer(k)` for its own `k` |

The blend role rides on `brepFace.blend`, so a placement re-mints it
(`brepPayload.Placed` re-evaluates the record). No face carries a receiver role
or an operand's provenance (modify §11, general-boolean §4.2). A BB4 result is
a brep like any other: route E admits it again under Table EB, and SB7 refuses
an edge that now ends on a blend face.

## 8. Table DB — downstream

Route P results are the prism, cap-blend and cup payloads modify Table D and
reach Table DX already cover. A BB4 result is an ordinary `brepPayload`, and
general-boolean §4.5 reads it unchanged:

| DB | Consumer | BB4 |
|---|---|---|
| **DB1** | mass properties, `Bounds` | general-boolean §4.3: the blend face's `ArcSeg` or `LineSeg` wall enters `brepSegmentIntegrals` with its proven bound; an all-planar result with every coordinate a float is `Exact` |
| **DB2** | `Verify` validity, tolerance gate | by construction and the brep's witness reader |
| **DB3** | `Tessellate`, STL/OBJ/3MF | general-boolean §4.4: the blend wall chords its arc once and shares the samples with both end faces |
| **DB4** | mesh boolean, class B operand | a fillet result passes B2 and B6 (an arc has no direction); a chamfer's oblique wall misses B6 and takes the mesh path |
| **DB5** | `ThroughAll`/`ToFace` | the per-face extremes; a brep with zero displacement answers |
| **DB6** | clearance | `clearance.BuildBrepBodyCarriers` adds the blend's cylinder or plane carrier |
| **DB7** | undercut survey | per face; the blend wall's normal range from its walk |
| **DB8** | concave-radius survey | a concave fillet is a clockwise arc walked with the material on its left and is read; a convex fillet is not a concave feature |
| **DB9** | wall survey | staged `Suspect`, as for every brep |
| **DB10** | STEP | analytic: a fillet wall is a partial cylinder of two arcs and two lines |
| **DB11** | a co-directional `Union` | `stack` is nil, so the result is not an A1 operand; class B or the mesh path |

## 9. Required tests

Every test asserts computed geometry against a closed form; bounds are
relations, never literals. Receivers are built through the public booleans
(`Cut`/`Union`) so the tests exercise the real records.

Fixtures: **S1** is the 40×20×20 box (`x∈[0,40]`, `y∈[0,20]`, `z∈[0,20]`)
cut by the Ø6 cylinder along `y` at `(x, z) = (20, 10)`, volume
`16000 − 180π`. **B1** is the same box cut by the 10×10 square hole along `y`
at `x∈[15,25]`, `z∈[5,15]`, `Exact` 14000. **Pocket** is the 40×40×10 plate on
XY with the 20×10 blind pocket `x∈[10,30]`, `y∈[15,25]` from `z = 10` to
`z = 5` (stacked, `Exact` 15000). **Boss** is general-boolean §9's flush
corner boss (plate `x, y ∈ [−20, 20]`, `z∈[0,10]`; boss `x∈[10,20]`,
`y∈[−20,−10]`, `z∈[10,25]`; `Exact` 17500).

Route P:

- S1, fillet the four edges along `y`, `r = 2`: a `prismPayload`, 11 faces,
  volume within its bound of `15680 − 100π` (section `784 − 5π` over 20), four
  `fillet(0,j)` roles, `capStart`/`capEnd` present.
- S1, chamfer the hole rim at `y = 0`, `d = 1`: a `capBlendPayload`, volume
  within its bound of `16000 − 550π/3` (the removed ring `10π/3`), one `Cone`
  face; the undercut survey lists the cone under a pull along `+y`, and the
  concave-radius survey reads the whole-turn band undecided, as reach DX8
  states.
- S1, shell removing the `y = 20` face, `t = 2`: a cup, volume within its
  bound of `5632 + 270π` (`20·(800 − 9π) − 18·(576 − 25π)`); removing the
  `x = 0` face instead falls past route P to route S, which opens that wall
  (`agent-docs/modify-general-design.md` §3.2): `6272 + 220π`.
- B1, chamfer the square hole's four edges along `y`, `d = 2`: concave (the
  hole loop's corners fill), `Exact` `14000 + 4·2·20 = 14160`; the hole's
  four lateral edges share no vertex, and S6 admits `2 + 2 < 10` on each wall.
- A recognised prism's `S` equals top's region re-expressed into `F` bit for
  bit, and P5 refuses a hand-built record whose one wall is walked against its
  material.
- A hand-built two-hole S1 whose `y = 0` face lists its holes in the other
  order: a chamfer of that face's first-listed hole rim bands that hole.

Route E:

- S1, fillet the edge along `z` at `(x, y) = (0, 0)`, `r = 2`: a `brepPayload`
  of 8 faces, volume within its bound of `15920 − 160π`, the `y = 0` face's
  `x = 0` segment at `x = 2`, both caps carrying the arc about `(2, 2)` with
  feet `(0, 2)` and `(2, 0)`, one `fillet(k)` role on the new wall.
- B1, chamfer the edge along `z` at `(0, 0)`, `d = 2`: `Exact` `13960`.
- Pocket, fillet its four vertical edges, `r = 2`, in one call: concave, 15
  faces, volume within its bound of `15080 − 20π`; the concave-radius survey
  reads 2 mm; the top cap's hole loop and the floor's outer loop each carry
  four arcs about the same four centres.
- Pocket, fillet the two floor edges along `x` (`y = 15`, `y = 25`, `z = 5`),
  `r = 1`, in one call: both pocket `y`-walls and both `x`-walls are restated,
  13 faces, volume within its bound of `15040 − 10π`; all four floor edges in
  one call are the floor's complete loop, which route L's fillet arm builds
  (`agent-docs/loop-fillet-design.md` §8).
- Boss, chamfer the boss's inner vertical edge at `(10, −10)`, `d = 2`:
  `Exact` `17470`, 10 faces; the floor's reflex corner and the boss top's
  convex corner carry the one chord between `(10, −12)` and `(12, −10)`.
  Its front vertical edge at `(10, −20)` → SB9: the plate top's convex
  corner there faces away from the boss, and its feet `(8, −20)`,
  `(10, −18)` disagree with the boss top's.
- An L-shaped boss flush on the plate's corner (`(10, −20)`, `(20, −20)`,
  `(20, −10)`, `(15, −10)`, `(15, −15)`, `(10, −15)`, `z∈[10,25]`), fillet
  the inner vertical edge at `(15, −15)`, `r = 1`: concave between two planar
  walls, and `Edge.IsConvex()` reads it concave; volume within its bound of
  `17140 − 15π/4`.
- S1, chamfer the edge along `x` at `(y, z) = (0, 0)`, `d = 2`: the `x = 0`
  and `x = 40` walls are restated, volume within its bound of
  `16000 − 180π − 80`.
- Chaining: chamfer S1's edge along `z` at `(0, 0)`, then chamfer the edge
  along `x` at `(y, z) = (0, 0)` on the result → SB7. Filleting all twelve
  straight edges of S1 takes route V (`agent-docs/vertex-blend-design.md`).
- Consumers on the S1 fillet result: `Verify` `Sound` at the default
  tolerance, the mesh's occupied-volume proof covering `15920 − 160π`, STEP's
  analytic arm (the fillet wall two arcs and two lines), the undercut survey
  listing the fillet wall under a pull along `(1, 1, 0)` and not along `z`,
  and transformed by a translation reproducing volume and area.

Displaced reading (§4.4; `apitest/revolve_shaft_class_b_blend_test.go`,
`internal/brepmodify/displaced_blend_test.go`):

- D-flat (r = 10, L = 40, flat at z = 7), `Fillet` r = 1 and r = 2 of both
  flat edges: 3 planes and 3 cylinders, the volume within its bound of
  `40·(100π − segment − 2·A_c)` with the fillet corner `A_c` in closed form,
  `VerifyAll`; the Revolve and Extrude twins agree bit for bit, and r = 2
  agrees with the Transform-placed tool's mesh-path blend within both bounds.
- D-flat `Chamfer` d = 1: within its bound of the closed form, and a further
  `Fillet` of its lateral edges refuses with the prism's displaced-receiver
  text (PD7). The r = 1 fillet result takes a d = 0.5 cap-loop chamfer to
  the closed form `(d²/2)·(P − 2π·d/3)` removed, `P` the filleted section's
  perimeter; the keyway's two top edges, and all four lateral edges, fillet at
  r = 0.5 to their closed forms; a translated D-flat fillets to the same
  volume.
- Refusals: one flat edge, and the keyway's floor edges alone (PD4); the
  `x = 40` cap loop's chamfer (PD1); an asymmetric chamfer and a Shell (SB1);
  `Fillet` r = 10 (S5).
- A record whose crossings sit 1e-4 off (displacement 2e-4) fillets to the
  real record's volume within both bounds with `SectionDelta` 0, and chamfers
  with `SectionDelta ≥ 2e-4`; a cleared carrier is PD2; a carrier radius of
  10.5 or a centre one ulp off is PD3; PD3's half-radius leg, PD5a–PD5c and
  PD6 hold on crafted numbers.

Axis-aligned A1 crossing:

- The L bracket (60×40×8 base united with an 8×40×50 upright; `Exact` 32640)
  records `SectionDelta` 0 and a `Stack.Delta` of `AbsSumUpper(cut, 0)`, about
  one ulp above `CutDisplacementAllow(60)`. Its heel edge takes a two-distance
  chamfer 3/5 (`Exact` 32640 − 300) and an equal chamfer 3 (`Exact` 32460),
  and its concave inner corner a `ChamferAngle` 3 at 30°, whose volume agrees
  with 32640 + 60√3 to a relative 1e-12. An inward Shell, t = 2, with the
  y = 0 end open reads 17592 + 38π (the wall of the 816 mm² L section over
  40 mm less the 38 mm cavity of area 396 − π). Each result is `Sound`.
- A round boss crossing a plate's side records a `SectionDelta` at least
  `CutDisplacementAllow` of that side: a line-circle vertex carries its cut
  end's charge itself.

Refusals:

- general-boolean §9's crossing round boss → SB1 naming the displacement; a
  prism-group stacked receiver → SB2; S1's hole rims
  selected → SB4 (Pocket cut by a Ø4 through hole takes the mesh path, and
  its result is reach SX9's); Pocket shell → SB10; S1 chamfered at `d = 25`
  along its `z` edge at the origin → S6 in the end face; S1 by hand with its
  `x = 0` wall's side line at `y = 0` split at `z = 10` (the `y = 0` face's
  segment split to match): its edge along `x` at the origin → SB7, its edge
  along `y` there → SB8.
- Every refusal leaves the receiver live and the document unchanged.
- `.github/test-shards.txt`
  carries every new name, and `go test . ./apitest/ -run '^TestCI'` passes.

## 10. Do not do this

- **Offset a displaced record's own vertices.** An offset solve from
  displaced data amplifies the displacement by the corner geometry. §4.4
  offsets the exact carriers and lets a displaced corner decide signs only,
  each with a margin; every other displaced receiver is SB1, as
  `prismrecord.Payload.RequireExactSection` refuses for a prism.
- **Compute the blend in one end face and copy it to the other without
  recomputing.** SB9's recomputation is the one falsifier that proves the two
  faces hold the same carriers; copying would build a closed body on a wrong
  face.
- **Recompute a foot per face.** Every face takes the one float the corner
  blend produced; a second evaluation can differ by rounding and the edges
  stop pairing.
- **Read a face region's corner convexity as the solid edge's.** A floor's
  reflex corner and a boss top's convex corner belong to one convex edge; the
  blend face's walk sense reads `G0`'s outward side against `G1`'s (§5.3 step
  5) and the corner rewrite reads each face's own loop. `Edge.IsConvex()` is
  not the solid's convexity for a rim, or for a line two planar faces of
  different sweeps share.
- **Patch a vertex where two independent route E blends meet.** SB5 still
  refuses a selection outside one planar-face loop. A selected chain on
  one such loop uses a band; route V builds a sphere where a complete-loop
  fillet meets a straight-edge fillet (`agent-docs/vertex-blend-design.md`).
- **Build a hole-rim chamfer on a brep face as a planar region.** A cone band
  is not a region of any plane; the brep needs a band face kind, which is a
  design of its own (reach §8.3 is the construction to port).
- **Offset a brep face by face for Shell.** The three-dimensional erosion moves
  each face's trace by the dihedral at its edge, not by `t` in its own plane,
  and fills reflex edges with cylinders and tori; a per-face 2D offset is a
  different, wrong solid. Route S (modify-general §3) builds the erosion of a
  through-cut record; SB10 refuses the rest.
- **Let a selector's `Exactly(n)` count change under restatement.** Selection
  is resolved against the receiver before any face is restated.

## 11. Decided questions

- **Faceted receivers**: never. Reach §11's reasoning stands; the mesh is one
  evaluator's decomposition, not a carrier.
- **A prism receiver's straight cap edge through its face view**: route E
  takes a single selected edge for Fillet or Chamfer when Table EB admits it.
  Modify Table S (S1) keeps other partial cap selections staged.
- **Route P before route E**: yes; a `prismPayload` result reaches more
  consumers and further modify ops.
- **Shell of a non-prism brep**: route S of `agent-docs/modify-general-design.md`
  for a prism cut by through tools; SB10 for the rest.
- **Hand-offs**: none. No capability is needed from `sketch`, `r3` or `units`.

## 12. PR split

Each PR ships code, tests and the documentation updates for every refusal it
lifts (the increment table below marked landed, `doc.go`'s support map,
`agent-docs/missing-features.md`, the function doc comments). This document ships
with PR 0.

| PR | Lands | Files and functions | Proves | After |
|---|---|---|---|---|
| **0** | the dispatch: `modifyBrepReceiver` in `brep_modify.go` replacing `requireNotBrepReceiver` in `fillet.go`, `chamfer.go`, `shell.go`; the stacked receiver's `breprecord.OfStacked` entry (SB2); SB1; every other brep call still refuses with reach SX16's text | `brep_modify.go`, `internal/breprecord/convert.go`, `fillet.go`, `chamfer.go`, `shell.go` | SB1, SB2 fixtures; a stacked receiver reaches SB1 rather than SX10 | — |
| **1** | route P: `recognisePrism` (P1–P5), `brepgeom.AxisFrame`, the re-expression into `F`, cap-face arguments on `classifyChamferSelection` and `classifyRemovedCaps`, SB3 | `brep_modify.go`, `capblend.go`, `shell.go`, `internal/brepgeom/` | §9's five route P fixtures | 0 |
| **2a** | route E without restatement: Table EB, `brepgeom.BrepCornerAt`, the corner blend in both end faces and SB9, (sw)/(pl) trims, claims and the per-face audit, the blend face and `brepFace.blend`, `brepgeom.MapSegment`, assembly and closure | `brep_modify_edge.go`, `internal/breprecord/convert.go`, `fillet_audit.go`, `internal/brepgeom/` | S1 `z`-edge fillet, B1 `z`-edge chamfer, Pocket vertical fillets, Boss chamfer, the chaining and SB4/SB5 refusals, the consumer and placement fixtures | 0 |
| **2b** | restatement: `brepgeom.Restate`, `brepgeom.PlanarFrame`, the whole-call restatement set, EB3/EB4 over restated faces | `brep_modify_edge.go`, `internal/brepgeom/` | Pocket floor-edge fillets, S1 `x`-edge chamfer, SB8 on a split wall | 2a |
| **2c** | route E's asymmetric reference-face mapping; bypass route P for this option and refuse route L | `brep_modify.go`, `brep_modify_edge.go`, `brep_modify_loop.go`, `chamfer.go` | public boolean body: both reference faces on a vertical edge, restated end faces on a horizontal edge, and route L refusal | 2b |
| **2d** | route L's asymmetric cap/side assignment and two-distance band | `brep_modify_loop.go`, `tessellate_brep.go` | public rounded, cross-drilled plate: both reference choices, analytic volume, verified mesh | 2c |
| **2e** | preserve stacked source-face roles in its brep view for asymmetric route E/L references | `internal/breprecord/convert.go`, `brep_modify.go`, `chamfer.go` | public blind-pocket plate: outer edge and both top-loop reference choices, analytic volumes, verified meshes | 2d |
| **3** | §4.4: `breprecord.Face.WallCarrier` from the crossing build, the claim map, `displacedReceiver`, the Fillet and Chamfer arms, Table PD | `internal/brepgeom/`, `internal/breprecord/payload.go`, `internal/classbgeom/`, `internal/offset2d/blend.go`, `internal/brepmodify/` (`displaced_blend.go`, `prism_route.go`, `fillet.go`, `chamfer.go`) | §9's displaced-reading fixtures | 1 |

PR 1 and PR 2a run in parallel: they share only PR 0's dispatch and add
disjoint files. PR 2b follows 2a.

Increment table — what still refuses after each PR:

| After | Still refused |
|---|---|
| 0 (landed) | every brep and stacked modify op except SB1/SB2 (reach SX16's text) |
| 1 (landed) | every non-prism brep; every route E edge |
| 2a (landed) | edges whose end or rim-adjacent face is a swept straight wall (SB7/SB8 until 2b) |
| 2b (landed) | Table SB alone |
| 2c (landed) | asymmetric route P/L chamfers and stacked receivers (SX16) |
| 2d (landed) | asymmetric stacked receivers and brep selections outside routes E/L (SX16) |
| 2e | brep or stacked asymmetric selections outside routes E/L (SX16) |
| 3 (landed) | displaced receivers outside §4.4 (SB1): a stepped shaft, a partial crossing selection, an asymmetric chamfer, a cap edge or loop, a Shell, an arc-walled operand |
