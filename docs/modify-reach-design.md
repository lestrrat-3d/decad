# Modify Reach Design

How the evaluator extends `Fillet`, `Chamfer`, and `Shell` beyond the
straight-prism cases in `docs/modify-design.md`.

This document owns only the extension. `docs/modify-design.md` still owns the
shared call contract, gate order, section audit, current prism corner rewrite,
and base result tables. The rules here add receiver/target rows without
weakening any base rule.

Four tables are normative:

| Table | Owns | Section |
|---|---|---|
| **RX** | additional receiver + target classes | §3 |
| **SX** | extension refusals + gate order | §4 |
| **BX** | extension result payloads, topology, roles | §10 |
| **DX** | downstream behavior + staged questions | §12 |

## 1. Scope

The extension adds only shapes reducible to decad's recorded line/arc regions
or to a finite set of named analytic patches:

- prism cap-loop fillets and chamfers;
- prism shell side openings and closed shells;
- revolve junction-edge fillets and chamfers;
- full-revolve shells, plus partial-revolve shells with both angular caps open;
- exact tangent-chain expansion over analytic topology;
- two-distance chamfers with an explicit reference face.

The extension does **not** add a general B-rep kernel:

- no partial cap-edge chain with a free endpoint;
- no mixed lateral-edge + cap-edge blend in one call;
- no variable-radius fillet;
- no topology-changing offset;
- no faceted receiver (a brep or stacked receiver is
  `docs/brep-modify-design.md`'s);
- no partial-revolve shell that keeps an angular cap.

Each excluded body exists. Table SX stages it with `ErrUnsupported` at the
modify call. No excluded shape is approximated, clipped, or deferred into
`Verify`.

## 2. Public options

The three method signatures stay unchanged. The extension adds these options:

```go
// FilletChamferOption is accepted by both Fillet and Chamfer.
type FilletChamferOption interface {
    FilletOption
    ChamferOption
}

// WithTangentChain expands each selected seed across proven G1 continuations.
func WithTangentChain() FilletChamferOption

// WithAsymmetricChamfer applies the positional distance on reference and
// otherDistance on the other face adjacent to each selected edge.
func WithAsymmetricChamfer(
    reference FaceSelector,
    otherDistance units.Value,
) ChamferOption

// WithNoOpenings asks Shell to keep every face and build a closed hollow body.
// It is the only call form in which Shell accepts a nil face selector.
func WithNoOpenings() ShellOption
```

The option constructors copy their selector inputs at the call boundary. The
selectors resolve against the receiver during evaluation; resolved faces and
topology indices are never retained.

Each call decodes its options into a private record (`modify_options.go`):
`filletOpts{TangentChain}`, `chamferOpts{TangentChain, Asymmetric}` and
`shellOpts{Sense, NoOpenings}`. A shell with openings states a removed-face
query and a thickness; a shell with no openings states no selector, a
thickness and `shellOpts{NoOpenings: true}`.

An option repeated with the same intent counts once: `WithTangentChain`,
`WithNoOpenings`, and one `WithShellSense` sense given twice. Two
`WithShellSense` options naming different senses, and a second
`WithAsymmetricChamfer`, name two intents and are SX1. The other distance
passes the positional distance's magnitude gates as it is decoded: base S15
for a wrong `Kind`, a non-finite or a negative value, and `ErrDegenerate` for
zero, since §6 requires both distances strictly positive.

Cardinality on the seed edge query applies **before** tangent expansion. This
keeps `Exactly(n)` an assertion about what the caller named, not about an
evaluator-dependent number of continuation edges.

`WithNoOpenings` and a non-nil face selector conflict: SX1. A nil selector
without `WithNoOpenings` remains base S16 (`errNilSelector`).

## 3. Table RX — receivers + targets

Base Table R still admits the shipped straight-prism cases. RX adds these rows:

| RX | Receiver | Fillet / Chamfer | Shell |
|---|---|---|---|
| **RX1** | `prismPayload` | base lateral junctions; OR every geometric edge of one or more complete cap loops. Never both classes in one call | base cap openings, with BX8 replacing base S12; OR, for a hole-free section, one proper connected run of outer side faces with any cap openings; OR, for a hole-free section, `WithNoOpenings` |
| **RX2** | `revolvePayload` | off-axis swept meridian junctions only: a full-turn latitude `Circle3` or partial-turn junction `Arc3` | full turn: one proper connected run of generated side faces, or `WithNoOpenings`; partial turn: both angular caps MUST be removed, with an optional proper connected side-face run |
| **RX3** | `stackedPrismPayload` | through its face view, `docs/brep-modify-design.md` Table RB; a view that refuses is SX10 | the same |
| **RX4** | `capBlendPayload` | SX10 | SX10 |
| **RX5** | `cupPayload`: a two-slab `stackedPrismPayload` record beside the shell morphology (§9.1) | base S3 | base S3 |
| **RX6** | `facetedPayload`, including zero-bound all-planar boolean output | SX9 | SX9 |
| **RX7** | `brepPayload` (`docs/general-boolean-design.md` §4) | `docs/brep-modify-design.md` Table RB; outside it, SX16 | the same |

Definitions:

- **complete cap loop**: every edge in one `Loop` of one prism cap face;
- **geometric edge**: a whole `Circle3` has no endpoint corner; its seam vertex
  does not make it a partial chain;
- **proper side-face run**: non-empty, connected in boundary-walk order, and
  not every side face;
- **swept meridian junction**: edge produced by one corner between consecutive
  walks of the recorded meridian profile;
- **angular cap**: `capStart` / `capEnd` of a partial revolve.

Selections spanning two complete cap loops are allowed. Selection spanning the
same loop on both prism caps is allowed only when the two blend bands pass the
SX7 separation gate.

The receiver is still keyed on payload, not history. Placement preserves every
RX class.

## 4. Table SX — refusals + order

Base S1–S17 keep their meanings. Where RX admits a former S1/S2/S3 case, the
more specific SX row replaces that base refusal.

| SX | Call | Existence | Sentinel |
|---|---|---|---|
| **SX1** | `WithNoOpenings` with a non-nil selector, repeated contradictory option, or malformed option payload | no single intent | `ErrDegenerate` |
| **SX2** | tangent continuation is branch-ambiguous or the analytic oracle cannot decide G1 continuity | evaluator cannot know which chain caller named | `ErrUnsupported` |
| **SX3** | asymmetric reference is nil, invalid, or does not identify exactly one adjacent face per expanded edge | invalid selector / cardinality | existing selector error; otherwise `ErrCardinality` |
| **SX4** | blend selection mixes lateral/revolve junctions with prism cap edges, selects only part of a cap loop, or gives one cap loop mixed asymmetric face assignments | body exists; endpoint/setback transition not built | `ErrUnsupported` |
| **SX5** | selected revolve edge is not a swept meridian junction | body exists; cap-edge/general rolling blend not built | `ErrUnsupported` |
| **SX6** | cap-loop offset loses a carrier, reaches an empty circular offset, or has no regular radius-`r` envelope | no regular requested blend | `ErrDegenerate` |
| **SX7** | cap-loop center paths cross/touch non-adjacent paths, a patch self-intersects, two cap bands meet, or trims need merging | body exists under trimming/merge kernel | `ErrUnsupported` |
| **SX8** | shell side/no-opening extension is used on a holed prism section; a non-empty side selection is not one proper outer-loop run; offset changes topology; partial revolve keeps either angular cap; revolve meridian is holed or meets the axis along more than one walk; a revolve wall's offset reaches the axis | body exists outside extension | `ErrUnsupported` |
| **SX9** | any modify op on `facetedPayload` | body exists; analytic carrier + stable topology absent | `ErrUnsupported` |
| **SX10** | another modify op on `capBlendPayload`, or on a `stackedPrismPayload` whose face view refuses (`docs/brep-modify-design.md` SB2) | body exists; compound feature composition not built | `ErrUnsupported` |
| **SX11** | inward closed/side-opening prism shell leaves axial cavity height `h - k*t <= 0`, where `k` is kept cap count; or section cavity is empty | no cavity | `ErrDegenerate` |
| **SX12** | cap chamfer ruled patches intersect away from shared boundaries or cannot be certified disjoint | body exists under trim kernel | `ErrUnsupported` |
| **SX13** | a cap-loop chamfer whose setback rounds away against the level it displaces: the cap contour's offset radius rounds back onto a circular wall's own radius (`R -/+ d == R`), or the band's side level rounds back onto its own cap level (`z1 - d == z1` on the end cap, `z0 + d == z0` on the start cap) | body exists; its taper is real but finer than float64 names at that radius or at that sweep level, so the band's patches cannot be told from a cylinder or from the cap plane | `ErrUnsupported` |
| **SX14** | a cap-loop chamfer whose denoted contour corner cannot be enclosed: the two offset carriers' interval intersection is unbounded, or the exact carriers do not meet where the float solve found a root. A G1 join (modify §7's dead-zone rule) intersects no carriers — its corner is the shared-normal foot, enclosed as a reflex corner's feet are — so SX14 never fires on one. Also a circular band patch whose corner skew between its side and cap directrices cannot be enclosed below a quarter turn (§8.4) | body exists; its offset corner is real and this evaluator cannot state where it is, so no cap-level coordinate there can publish a proven displacement, or no area bound holds for the ruled patch at that skew | `ErrUnsupported` |
| **SX15** | a cap-loop chamfer whose band patch's outward orientation cannot be certified: the patch's own `Face.NormalAt` refuses at the build's orientation sample point | body exists and its patches are real; the evaluator cannot evaluate its own orientation sample on this patch, so it cannot state which side of the patch is outward | `ErrUnsupported` |
| **SX16** | a modify op on a `brepPayload`, or on a stacked receiver through its face view, outside `docs/brep-modify-design.md` Tables RB/EB | body exists; that document's Table SB names the row | `ErrUnsupported` |

Gate order:

| Stage | Gates |
|---|---|
| 1. call | base S17; option decode; SX1; base S15/S13-or-S14; seed selector S16 unless no-openings |
| 2. expansion | resolve seed; expand tangent chain; SX2 |
| 3. reference | resolve asymmetric reference; SX3 |
| 4. receiver/target | base R + RX; SX4/SX5/SX8/SX9/SX10/SX16 |
| 5. existence | base S4/S5/S18/S10; SX6/SX11 |
| 6. constructed-geometry audit | base S8/S6/S7/S9/S11; SX7/SX12/SX13/SX14/SX15 |
| 7. payload | BX8 holds the `1 + k` bands where base S12 stood |

The existence-first rule remains load-bearing. SX6 precedes SX7: an empty
offset means no regular rolling-ball surface; intersecting valid patches mean
the surface exists but this evaluator cannot trim it.

SX13's RADIAL half sits with SX7/SX12 in stage 6 and after SX6 for the same
reason, on the same offset radius: SX6 is the offset that reaches the centre and
leaves nothing (the existence question), while SX13 is the offset that leaves a
circle this evaluator cannot tell from the one it started with. It is decided as
each band patch's carrier is constructed — not from `d` alone, since the same `d`
is perfectly representable against a smaller wall in the same section.

SX13 covers the AXIAL direction on the same terms. The band has two directrices
and the setback displaces both: `d` in the plane, which is the radial half above,
and `d` along the sweep, which carries the original loop from the cap level to
the side level. A tall enough sweep makes `d` fall under the float64 spacing of
that level too, and `z1 - d` rounds back onto `z1` (or `z0 + d` onto `z0`).
Every patch of the band is then emitted flat IN the cap plane, so a `Plane`
patch — which carries its normal with no bound — asserts a 45° taper the solid
does not have, and the DX7 undercut survey reads that assertion off the surface
and answers about a shape the caller never asked for. Substituting it is a wrong
answer, not a coarse one, exactly as in the radial case. The volume the collapsed
level reports is not what refuses the call: it is the correctly rounded volume of
the true chamfered solid, and its bound honestly charges the collapsed level. The
axial half is a fact about the sweep interval and the setback alone, so it is
decided once per chamfered cap; the radial half stays per circular wall.

SX7 is the neighbouring axial gate and the two do not overlap. SX7 refuses a
setback so LARGE beside the sweep that the band reaches or passes the far end
(`reach >= height`); SX13 refuses one so SMALL beside the sweep's own coordinates
that the level it displaces does not move at all. A tall prism with a tiny
setback clears SX7 by an enormous margin and is precisely the shape SX13 catches.

SX7's own place in the order matters for the same reason SX13's does, and it is
the one place two rows really can fire on one input. A setback that empties the
cap contour is SX6 at any sweep height, and a short enough sweep also satisfies
SX7's `reach >= height`. Since SX6 and SX7 make OPPOSITE existence claims, the
stage order is what keeps that one nonexistent body from reporting two
sentinels: the offset is built first and SX6 decides it, and only a contour that
exists reaches the band-reach test. Deciding the reach from the sweep interval
alone, ahead of the offset, lets the sweep height pick the sentinel for a body
whose non-existence has nothing to do with the sweep.

SX14 is not about the setback's size at all. It is about the corner's own
conditioning: the miter is a float solve over unit directions, so its result
sits some distance from the point the offset denotes, and §8.4 requires every
cap-level reading to publish that distance. Where the two offset carriers are so
nearly parallel that no bounded box holds the denoted corner, there is no such
distance to publish and the call refuses rather than name one.

SX15 is decided as each patch is constructed, like SX13's radial half: the
patch's own built surface is sampled at a point on it, and where that sample's
`NormalAt` cannot answer, the sign is a build-time question this evaluator
cannot finish, so it refuses rather than publish a patch whose outward side was
never checked.

## 5. Tangent-chain expansion

`WithTangentChain` expands every seed edge by a fixed-point walk over endpoint
vertices. Expansion never changes the seed selector's cardinality result.

At endpoint `v` of edge `e`, candidate `c` continues the chain only when all
tests are proven:

1. `c != e`, and `c` is incident to `v`.
2. Tangent rays of `e` and `c` pointing away from `v` are exactly opposite.
3. Two faces adjacent to `e` map one-to-one to two faces adjacent to `c`.
4. Each mapped face pair has the same outward normal at `v`.
5. Curves and surfaces are analytic variants supported by the oracle.

Use unnormalised analytic directions:

- line tangent: endpoint difference;
- circle/arc tangent: `axis × (point-center)` with endpoint sense;
- plane normal: frame normal;
- cylinder/cone/sphere/torus normal: carrier numerator before normalisation.

Feed collinearity and equality to the exact-float degeneracy oracle. A
tolerance may prove `no`; it may NEVER prove `yes`. `unknown` is SX2, not a
chain stop, because silently stopping under-fillets the caller's rule.

Continuation outcomes at one endpoint:

| Proven candidates | Result |
|---|---|
| 0, with no unknown | stop |
| 1, with no unknown | add edge and continue |
| >1, or any unknown | SX2 |

Body edge order breaks no tie; a branch is never chosen. Expansion across a
cycle stops when the next edge is already in the expanded set. Final edge order
is body order, so replay and role assignment stay deterministic.

Faceted curves/surfaces never enter the oracle. They route to SX9 before
expansion.

### 5.1 Implementation

`tangent_chain.go` runs this section; `Body.Fillet` and `Body.Chamfer` call it
right after the seed query resolves. The oracle is the clearance kernel's
`Oracle.ParallelExact` (`internal/clearance/oracle.go`): an exact zero cross
product over the held floats proves "parallel", a cross above `ClrAngTol`
relative to the two lengths disproves it, and the band between is undecided.
Opposite rays and equal normals additionally need the sign of the exact dot
product, which decides "no" by itself wherever it has the wrong sign.

A plane's frame normal, a cylinder's radial offset (scaled by the held axis's
own squared length, so a not exactly unit axis keeps the direction exact) and
a sphere's offset from its centre are exact over the held floats. A cone's or
a torus's normal needs a square root or a trigonometric value, so the oracle
reads its float normal and that answer decides "no" only, never "yes".

Test 3 tries both one-to-one maps of the two face pairs, and a continuation
is proven when either map proves both normal pairs equal. A face shared by
both edges maps to itself. An edge with other than two adjacent faces is
undecided.

A full circle has no endpoint, so it neither expands nor continues another
edge.

## 6. Asymmetric chamfer

Without `WithAsymmetricChamfer`, setback remains equal on both adjacent faces.

With it:

- positional `d` is setback across the reference face;
- `otherDistance` is setback across the other adjacent face;
- both are length magnitudes and strictly positive;
- reference resolves after tangent expansion;
- each expanded edge MUST have exactly one adjacent face in the resolved set;
- every resolved reference face MUST touch at least one expanded edge.

Extra/missing/dual adjacency is SX3. This avoids defining “first side” from a
traversal-dependent coedge direction.

On a prism or a revolve receiver, each adjacent wall carries `side(i,j)`
roles naming the recorded segments it is built from, and the reference face's
roles pick the walk that takes `d`: the arriving or the leaving walk of the
coalesced corner walk. A reference face whose roles name segments of both
walks, or of neither, is `ErrUnsupported`.
`docs/brep-modify-design.md` states no asymmetric setback for either brep
route, so the option on a brep or stacked receiver is SX16.

For a prism lateral edge or revolve junction, adjacent faces map to arriving
and leaving walks of the section/meridian. Set each foot back by its assigned
arc length and connect the feet by the existing chord. Base S6 audits the two
different cutbacks independently.

For a complete prism cap loop, reference normally names the cap face:

- cap-face distance offsets the cap contact contour in the section plane;
- side-face distance moves the side contact contour axially into the prism.

A reference query may instead name one side face per edge. The per-edge
one-adjacent-face rule keeps that spelling unambiguous. One complete loop MUST
use one assignment throughout: cap referenced for every edge, or side
referenced for every edge. A mixed assignment gives adjacent patches different
axial setbacks and is SX4. SX3 runs first and always catches it: a cap face
borders every edge of its loops, so a reference naming it for one edge names
it for all, and a side face named beside it gives that edge two reference
faces. §8.3.1 builds the two-distance cap-loop band.

## 7. Revolve junction rewrite

A revolve's meridian profile has the same corner-to-edge mapping as a prism's
section:

| Meridian record | Revolve topology |
|---|---|
| walk | side face of revolution |
| off-axis corner | latitude circle on a full turn; angular arc on a partial turn |
| on-axis corner | one vertex, no selectable edge |

Map every selected RX2 edge to `(loop, corner)` of the axis-local coalesced
walk. Match the stored analytic edge against the junction record; do not use a
topology index.

Reuse the base `cornerBlend` rewrite and audit:

- fillet inserts a tangent meridian arc;
- chamfer inserts a meridian chord;
- asymmetric chamfer assigns its two cutbacks by adjacent face roles;
- S4/S5/S8/S6/S7/S9 keep their base meanings.

Run the revolve axis gates again on the rewritten profile. A new interior axis
contact is `ErrDegenerate`; a representable but staged spindle branch is
`ErrUnsupported`, matching the base revolve contract.

Build through `evalRevolve` with the receiver's frame, oriented axis, angular
interval, and placement. Result remains `revolvePayload`:

- fillet arc → `Sphere` when its center lies on axis, otherwise `Torus`;
- chamfer chord → `Cylinder`, `Plane`, or `Cone` by existing classification;
- full and partial turns use existing topology builders;
- measurements carry existing revolve-integral bounds.

Blend face carries `side(i,j)` plus `fillet(i,j)` / `chamfer(i,j)` in the
result record's index space.

### 7.1 Implementation

`revolve_blend.go` runs this section; `Body.Fillet` and `Body.Chamfer` route a
`revolvePayload` receiver to it after stage 1 and SX10/SX16.

**Matching.** The build stamps every junction edge from
`revolvePayload.junctionCircle`: centre on the axis at the junction's `z`,
the placed sweep axis, radius `ρ`. A selected edge is a junction exactly when
its curve kind is the sweep's (`Circle3` full, `Arc3` partial) and its centre,
axis and radius equal one junction circle's bit for bit, recomputed from the
same payload. A cap edge's circle lies in a cap plane, and an on-axis or cap
line is a `Line3`, so neither matches: SX5. Equality is exact because the two
numbers come from one function over one payload; no tolerance chooses a
junction.

**Corner coordinates.** The payload records the meridian plane-locally, and
the rewrite is rigid-invariant, so the corner rewrite runs on the plane-local
coalesced walk (`profileCornerLoopsBudget`), the one a prism's section uses.
The axis-local junction maps to it through the recorded segment its leaving
walk starts at. Where the two coalescings disagree about that segment, the
call refuses with `ErrUnsupported` rather than picking a neighbouring corner.

**Axis gates.** `revolveBlendAxis` reruns `resolveAxisSide` — side gate,
axis-contact audit, snap allowances — on the rewritten meridian about the
receiver's oriented axis, and the build takes the axis frame it returns,
`radialProof` included. A region the gate puts on the far side is
`ErrDegenerate`. A line-line blend cannot create new axis contact: its new
piece lies in the triangle of its corner and two tangent feet, all strictly
off the axis (S6 refuses a foot that reaches a wall's on-axis end). The gate
still runs for every call, and a concave fillet whose arc centre falls across
the axis reaches it as the spindle refusal (`ErrUnsupported`).

**Payload.** The result is a `revolvePayload` with the receiver's frame,
oriented axis, angular interval, denotation and placement, and the rewritten
profile. Its `blendSegs`/`blendKind` fields carry the blend roles, so
`Placed` and `Duplicate` re-mint them; a path that replaces the profile
clears them. Measurements, topology, tessellation and the surveys are the
revolve's own (Table DX).

**Asymmetric chamfer.** The junction's two adjacent walls are swept walks of
the meridian, and each carries the `side(i,j)` roles of the recorded segments
it sweeps. §6's role reading picks which plane-local walk takes `d`, and
`computeChamfer` sets each walk back by its own distance.

## 8. Complete prism cap-loop blends

Cap-loop support requires a complete loop. This removes the free-end setback
patch that makes a partial edge chain a general B-rep problem.

### 8.1 Common record

`capBlendPayload` holds:

```text
receiver prism record + placement
selected cap/loop set
operation + distances
material-side offset loops
analytic patch records + trim domains
```

It is private evaluator data. Public calls state only selectors, values, and
options.

Each selected cap loop is offset on the material side by the cap-face setback.
Use the existing exact line/arc offset construction and profile audit. Unselected
loops stay unchanged.

### 8.2 Fillet

For radius `r`, intersect two offset carriers:

- cap plane offset `r` into material;
- adjacent side carrier offset `r` into material.

Their intersection in the offset cap plane is the rolling-ball center path.
Each center-path piece is analytic:

| Center path | Blend patch |
|---|---|
| line | `Cylinder`, radius `r` |
| circle/arc | `Torus`, major = path radius, minor = `r` |
| zero-length miter between non-tangent pieces | trimmed `Sphere`, radius `r` |

An offset connector arc at a reflex section corner is a circular center-path
piece and therefore a torus patch. A miter point carries the spherical
normal-cone patch joining its neighboring tubes. This is the cap-edge vertex
blend; no guessed setback surface is used.

Trim each patch between exact contact traces on cap and side carriers. Replace
the selected cap loop with its cap contact trace. Trim each adjacent side face
to its side contact trace. Canonicalize tangent patch joins; keep non-tangent
joins as topology edges.

Regularity gates:

- every material-side carrier offset exists: SX6;
- every circular center path has positive regular tube reach over its trim;
- offset loops preserve orientation, simplicity, and nesting;
- non-adjacent center paths stay strictly farther than `2r` unless their
  adjacency owns the shared spherical patch;
- every center path stays strictly farther than `r` from every unselected
  boundary carrier it does not belong to;
- blend bands from opposite caps do not meet.

Failure of the first two is SX6. Cross/contact/merge is SX7.

### 8.3 Chamfer

Let `dc` be setback across cap and `ds` setback down side. Equal chamfer uses
`dc = ds = d`; asymmetric reference assigns them per §6.

Build two contact contours:

- cap contour: selected loop offset `dc` into material;
- side contour: original loop moved axially `ds` into material.

Join corresponding analytic pieces:

| Boundary piece | Chamfer patch |
|---|---|
| line ↔ parallel line | `Plane` |
| concentric circle/arc ↔ circle/arc | `Cone` (a cylinder only at equal radii) |
| offset reflex connector ↔ original corner | trimmed `Cone` with corner apex |

"A cylinder only at equal radii" is exact equality and nothing looser. Two radii
that merely round close still name a cone, and the surface built for them must
be one: the taper is what DX7 reads off that surface, and a tolerance deciding
the kind would answer a whole scale of legitimate chamfers — a large radius with
a small setback — with a shape of different geometry. Where the offset radius is
not merely close to the wall's but bit-identical to it, the requested taper has
no float64 representation at that radius and the call is SX13.

The side contour is subject to the same rule along the sweep. Its level is the
cap level moved axially by `ds`, and where that sum is bit-identical to the cap
level the two contours share one level: every patch of the band comes out flat in
the cap plane, a `Plane` patch asserting a taper of 45° that the emitted geometry
does not have, and DX7 reads the assertion. Bit-identical is again exact equality
and nothing looser — a level that merely rounds close still separates the two
contours and still builds — and the refusal is SX13 for the same reason the
radial one is: the requested band exists and only float64 cannot name its side
level at that sweep coordinate.

Adjacent miter patches meet on their common analytic edge and need no extra
vertex face. The feature's contract is the exact offset family: at axial
fraction `s`, the denoted miter locus is the parallel section offset by
`s*dc`. A straight wall's `Plane` patch reaches it exactly — offsetting a
line is affine in the offset amount, so ruling the wall between its
side-level segment and its cap-level (offset) one reproduces the line offset
by `s*dc` at every `s`. A circular wall's `Cone` patch does not: the build
RULES it, with straight `Line3` rulings between the side-level directrix
(its own `th0`/`th1` sweep) and the trimmed cap-level directrix
(`capTh0`/`capTh1`, generally narrower at a non-tangential corner), so it
meets the exact offset family only at `s=0` and `s=1` and chords the true
curve strictly between them. That residual is bounded, never ignored:
erosion by an increasing offset is monotone, so the true swept flux is
sandwiched between the ordinary cone-sector flux read at the wide (side)
window and the narrow (cap) one. Both reference fluxes are read about the
arc's own axis at the side level, where the cone's flux density `R0·r(z)`
never changes sign; about the plane-local origin the density changes sign,
the sandwich fails, and the flux difference grows with the section's distance
from that origin. The ruled patch's own point-for-point
departure from the wide cone is bounded in closed form from the two windows'
angular skew. The band's volume sums every patch's flux about the plane-local
origin, and moving a patch's flux from the arc's axis to that origin adds the
axis point times the patch's vector area, which depends only on the patch's
boundary. The true and built patches end on different curves at each mitered
corner: the curved corner-foot locus and the straight ruling. Their vector
areas differ by the thin sliver between the two curves, which the patch
across the ruling carries with the opposite sign. Split at the corner vertex
`v`, the `Cone` patch's share is `(ds/dc)·|(v − c) × W|`, where
`W = ∫₀^dc (P(t) − Q(t)) dt` integrates the in-plane gap between the locus
`P` and the ruling `Q` ridden at the same offset rate. A `Plane` neighbour's
share is zero, because the sliver lies in its plane. Where a line meets a
circle the locus is a parabola and `|W| = dc³·Δ1²/(6·(y0 + y1)³)` in closed
form, with the line's distance from the centre as the moment arm. Between two
circles, `|W|` is at most `dc²/4` times the diagonal of the hull of the locus
velocity enclosures over the 32 offset sub-ranges. The share is zero at a
reflex foot, a G1 join and a whole turn, whose loci are straight, and a corner
whose share cannot be bounded makes the volume bound unbounded. The built
volume's own error holds none of this flux: it cancels between the two
patches a ruling joins. It is owed because each `Cone` patch's bound is taken
about its own axis. `ChordLocusVolumeAllow` composes the three terms into one
proven volume bound. The skew it reads is the larger of the patch's two proven corner
skews (§8.4's `CornerSkewUpper`), the exact angle between each corner's held
side end and held cap end. The difference of the two held windows is not used:
each end of it is a float `Atan2`, and over a 600-sector sweep drawn away from
the sketch origin that difference fell below the exact corner angle on 120 of
568 patches. The term rounds every operation outward: the flux difference is
taken exactly, `√(R0·R1)` through `RatSqrtUp`, and `sin(Φ/2)` at the top of its
certified enclosure. A skew that is not finite answers an unbounded volume. The
residual, and its bound, are exactly zero wherever both proven skews are: an
apex patch, and a join or whole turn whose two directrix ends lie on one ray
from the centre. A tangent join is modify §7's G1 row: its cap-level foot is
`v + dc·n̂` and its ruling runs from `v` to that foot, so the ruling IS the
denoted corner locus `v + s·dc·n̂` — affine in `s`, exactly as a reflex foot is
— and the two denoted windows coincide because the foot sits on the circular
wall's own radial through `v`. The held foot is a float point, so its proven
skew is zero only where it lands on that radial exactly, as it does on a wall
drawn on an axis; elsewhere the term charges the rounding-level skew.

That residual is not only a quantity: it is a difference of KIND. A straight
ruled surface between two arcs sweeping different windows has negative
Gaussian curvature everywhere between them, so no cone is it, and its own
normal TURNS along a single ruling — which a cone's, constant along every
ruling, never does. The patch keeps the `Cone` tag, because the offset family
the feature denotes really is that cone sector and the taper is what DX7 asks
about, but the tag is then a bounded STAND-IN for the surface as well as for
the measurement, and every reading taken off it owes the departure its own
term:

- `Face.NormalAt` on a band patch reports the tagged surface's own normal with a
  proven surface-departure bound. The bound is measured in WORLD space from the
  numbers the body itself publishes — the directrices' own `Arc3` centres, axes
  and radii or their own straight endpoints, the rulings' own endpoint vertices,
  and the tag's own frame, origin, axis, radius and half angle — in exact
  rational arithmetic, and `internal/capband/departure.go` owns the derivation. Two
  independent things separate the built surface from the tag and the one bound
  covers both. The two windows' SKEW is the first, and it is the CIRCULAR
  patch's alone: a non-tangential corner trims the cap directrix narrower than
  the side one sweeps, and the two directrices then differ in azimuth along every
  ruling. The PLACEMENT's own rounding is the second, it belongs to both patch
  kinds, and it is not a plane-local quantity at all: every world coordinate the
  build emits is a rounded image of what it denotes, and the roundings are
  independent, so the built rulings stop being generators of the published cone,
  the directrices' centres leave its axis, and the fourth corner of a flat
  patch's quad leaves the `Plane` fixed through the other three. That second part
  is nonzero on a band whose windows coincide exactly and on every flat patch,
  and it grows with the distance from the world origin to the patch and shrinks
  with the patch's own size — so a small band placed far out shows it orders past
  any reading's own arithmetic bound. A bound derived from the plane-local
  windows alone is identical placed or not, so its zero on a tangent join, an
  apex patch, a whole turn or a straight wall is an ASSERTION rather than a
  measurement, and it omits a direction difference the built surface has.
  `Face.NormalAt` separately composes its arithmetic proof
  (`normal_bound.go`).
- DX7 widens its own window reading by that bound. A point proven to oppose
  lists the patch; only an all-clear needs every point to clear. For the tagged
  normal-component range `[mn, mx]` and allowance `allow`, it lists when
  `mn + allow < 0 && mx - allow > -1`, clears when
  `mn - allow >= 0` (perpendicular included) or `mx + allow <= -1`
  (antiparallel included), and is undecided otherwise — the same rule
  `docs/verification-design.md` §6 states, at both ends. An undecided patch
  does not remove other patches already proven to oppose. Every point of the patch
  carries an azimuth inside the window, which is what makes each proof about
  the patch rather than about the cone. `allow` is TWO terms, and the departure
  is only one of them: DX7 reads each patch's normal through `Face.NormalAt`,
  and a reading so taken departs from the patch's own exact normal model three
  ways — the arm's own arithmetic (`normal_bound.go`), the displacement of the
  point the survey computed to sample at from the azimuth that reading is then
  used as, and the rounded spacing between those azimuths, neither of the last
  two being anything a reading's own bound speaks about. So a circular patch
  charges the WHOLE distance from its recovered coefficients to the model
  enclosed exactly from the tag's and the placed frame's own held numbers
  (`internal/capband/normal_model.go`), which covers all three at once and
  estimates no
  mechanism separately, beside that model's own proven departure from a single
  harmonic. Its window is then read through a proven enclosure of the recovered
  form's own extremes rather than a float evaluation of them, and each
  extreme's remaining enclosure width is charged as well. A flat patch is one
  reading under the bound that reading publishes, and that bound carries its
  own departure term the same way. Dropping any of it would
  decide against a direction the face never claimed — a pull the reading cannot
  separate from the patch's own tangent would be answered with the proven
  all-clear or a listed violation, and be right only by rounding luck. So an
  outright decision needs BOTH terms proven zero, and neither a whole turn nor a
  flat patch is exempt.
- DX8 does not answer for a band holding a mitered patch at all. Its reduction
  to the receiver's own section rests on every patch being flat or a cone
  sector, and the ruled patch is neither. DX8 also does not answer for a band
  holding any patch whose proven departure from the surface it publishes is not
  exactly zero — the mitered patch is one case of that; a whole-turn `Cone`
  patch is another, since its own held half-angle only encloses a cosine and
  sine rather than fixing them exactly; and every patch of a band built under a
  placement is a third, since the placement's own independent rounding of every
  emitted coordinate is never zero.
- A band patch's corner RULING owes its own boundary the same accounting. The
  build tags every corner ruling `Line3`, and that tag is the same kind of
  bounded stand-in the `Cone` tag is: the denoted corner-foot locus of the
  exact offset family is straight only where BOTH carriers meeting at the
  corner are lines, and is a conic wherever either one is circular — a line's
  offset moves affinely in the offset amount, so two lines' offsets cross
  along a path affine in it too, while a circle's offset FOOT does not move
  affinely along the circle even though its offset radius shrinks linearly.
  `Edge.Length()` on such a ruling reports the built chord, and its `Bound`
  covers the chord-versus-locus excess beside its own arithmetic. The excess
  is ONE-SIDED — a chord never exceeds the curve it subtends — so the term
  only ever widens the bound upward, never the reported value, and it is
  exactly zero at a line-line miter, at every reflex foot and at every G1
  join, because the locus is affine in the offset amount there (a reflex foot
  rides one wall's own offset carrier alone, whatever that wall's kind, and a
  G1 join's foot is `v + s·dc·n̂` by construction). A corner whose locus
  enclosure this evaluator cannot build refuses through `ErrUnsupported`
  rather than publish an understated bound — the same rule `Edge.Length`'s own
  doc comment (`topology.go`) already states for a boolean rim on a curved
  source.

A coinciding window — every tangent join, apex patch and whole turn — buys back
the SKEW half of the departure and nothing else. DX8's reduction is a claim
about the patch the build ASSEMBLED, not merely about its tag, so it needs BOTH
halves of the departure proven exactly zero: the coinciding window buys back
only the skew half, and the placement's own independent rounding of every
emitted coordinate leaves the other half in place on any placed band. So a
coinciding window alone no longer answers DX8. Only a patch whose own stamped
departure (`capblend_geom.go`'s `f.normalBound`, derived in
`internal/capband/departure.go`) is an exact zero does — an axis-aligned `Plane` patch
of an unplaced band reaches that, and nothing else does. `Face.NormalAt` and DX7
already read that same stamp; DX8 now reads it too, rather than assuming a
coinciding window buys back what only a zero stamp proves. A later PR could win
the answer back for a placed or whole-turn band through a proven curvature
bound over the built ruled patch's own held numbers, rather than through its
tag; this PR does not implement that route.

SX12 audits the exact offset family, not the ruled patch the body builds. It
runs the existing line/arc offset audit on the section offset by the full
setback `d` — the family's own `s=1` member — and certifies every
`s ∈ [0,1]` from that single check by the offset distance's own
monotonicity: a crossing anywhere in the family occurs no later than it
occurs at the full offset, so disjointness at `s=1` implies disjointness
throughout. Auditing one surface while measuring another is sound because
the two ask different questions of the same family: SX12 proves the swept
region the offset family denotes is well-formed — a fact about that family
alone, independent of which surface later reports its volume — while the
ruled `Cone` patch is a separate, proven-bounded stand-in, for the surface
readings above as much as for area/volume/moment measurement. A sample or
residual never admits disjointness.

#### 8.3.1 Two distances

`WithAsymmetricChamfer` on a complete cap loop sets `dc` and `ds` apart. The
construction above is unchanged; only the two numbers it reads differ.

**What the reference picks.** §6's resolution pairs every selected edge with
one reference face, and a cap-loop edge borders exactly two faces: its cap and
one side wall. The pick is made per chamfered cap:

| Reference face of the cap's edges | `dc` (across the cap) | `ds` (down the side) |
|---|---|---|
| the cap face itself | positional `d` | `otherDistance` |
| the side wall beside each edge | `otherDistance` | positional `d` |

Swapping the reference face swaps the two setbacks and nothing else. The two
caps of one call may pick differently: a hole's mouth on the start cap
referenced through its own side walls and the outer rim on the end cap
referenced through the end cap face build two bands with swapped setbacks.

**Refusals.** The payload holds one setback pair per chamfered cap. A cap
whose edges reference the cap face for some edges and a side wall for others
is SX4, and so is a loop chamfered on both caps whose two caps pick
differently, since that loop's two bands would offset its cap contour by two
different `dc`. SX3 already refuses both before stage 4 (§6), so SX4 here is a
second check over the resolved pairs and never the first answer. The other
existence and audit gates read the component they are about:

- SX6, SX12, SX14 and SX13's radial half read `dc`, the offset of the cap
  contour. SX6 and the stage-6 audit also read the top of `dc`'s span (below);
- SX7 reads `ds`: a loop's bands reach `ds(start) + ds(end)` along the sweep,
  and reaching the height is refused;
- SX13's axial half reads `ds`: `z0 + ds == z0` or `z1 - ds == z1` refuses.

A `dc` that empties the cap contour therefore answers SX6 (`ErrDegenerate`)
whatever `ds` is, since SX6 runs first; a `dc` whose contour exists but
crosses itself answers the audit's SX7/SX12 (`ErrUnsupported`) once the
band-reach test has passed. A `ds` that reaches the far end answers SX7
whatever `dc` is. A `dc` or `ds` too small to move its own coordinate answers
SX13 on that axis alone.

**Patches.** Each patch keeps its kind:

- a straight wall's patch is the `Plane` through the side-level segment and
  the cap-level segment offset `dc`, tilted `atan(dc/ds)` from the wall;
- a circular wall's patch is the ruled `Cone` between radius `R` at the side
  level and `R ∓ dc` at the cap level, half angle `atan(dc/ds)` about the
  wall's axis;
- a reflex corner's apex patch is the `Cone` from the original corner at the
  side level to the connector arc of radius `dc` at the cap level, half angle
  `atan(dc/ds)`.

**Bounds.** Every bound above is already stated in terms of the patch's own
held numbers — its two radii, its two levels, its two windows and its built
rulings — or of the cap contour's offset, so each keeps its derivation and
reads the right component:

- the cap contour displacement (§8.4), the cap-level vertex and edge bounds,
  the reflex connector arc's length bound, the cap face area's displacement
  term, and every offset-radius rounding read `dc` and its own
  unit-conversion rounding: the contour is the section offset by `dc`, and
  `ds` does not enter it;
- the side level `capZ ± ds`, its rounding plus `ds`'s own unit-conversion
  rounding (`levelDelta`), the straight-slab levels, the axial extent terms
  and the tessellator's band height read `ds`: the side contour is the
  original loop moved by `ds`, and `dc` does not enter it;
- the miter locus enclosure parametrises the corner foot by the in-plane
  offset `t ∈ [0, dc]` and moves it `t·ds/dc` along the sweep, so its length
  bound sums the in-plane speed against the axial rate `ds/dc`. The rise it
  reads is the stated `ds`, at most `|ds| + dsDelta`, never the held difference
  of the two float levels, which a tall sweep rounds below `ds`. Each
  sub-range starts at the float the previous one ends at, so the 32 sub-ranges
  cover `[0, dc]` with no gap, and each sub-range's axial rise
  `ds·width/dc` is rounded up;
- the ruled patch's chord-versus-locus volume term, its departure from the
  `Cone` tag, the normal model DX7 reads, and the area brackets read the
  patch's radii, levels and windows. Erosion stays monotone in the offset
  amount whatever rate the side contour moves at, so the sandwich between the
  wide and narrow windows still encloses the true flux.

Each bound encloses for the same reason it encloses at `dc = ds`: none of the
derivations equates the two setbacks. That includes the `Cone` area's
corner-skew term (§8.4), which reads the two corner skews, the two radii and
the two levels, and so both setbacks. The orientation sample references keep
a positive dot product with the true outward normal for any positive pair:
a wall patch's normal is `(ds·n̂_wall, ∓dc·ẑ)` against the reference
`(n̂_wall, ∓ẑ)`, and an apex patch's is `(−ds·r̂, ∓dc·ẑ)` against
`(−r̂, ∓ẑ)`, so each dot product is `dc + ds > 0`.

A setback stated in a unit other than millimetres reaches the band as a
rescaled float, some rounding away from the quantity the caller stated
(`magnitudeInBounded`). The side level charges `ds`'s rounding as `levelDelta`.
The cap contour charges `dc`'s rounding `dcDelta` through its displacement
(§8.4): the enclosures are taken over every offset amount in
`[dc − dcDelta, dc + dcDelta]`. The reflex connector arc's length is bracketed
over the same span, and the miter locus is enclosed out to `dc + dcDelta` at
an axial rate read against `dc − dcDelta`. A circular wall's cap arc charges
its sweep times the largest gap between the held radius and any radius
`R ∓ t` the span holds. That gap is nonzero even for a millimetre setback
wherever `R ∓ dc` is not a float64, and the arc's turn term, which moves its
two ends along the held circle, does not cover it.

SX6 and the stage-6 audit (SX7/SX12, S8/S9) reject at one offset and certify
every smaller one: SX12's monotonicity argument above runs one way. So
`buildCapBlend` runs them a second time on the section offset by the top of
the span, the smallest float at or above `dc + dcDelta`
(`auditCapBlendSetbackSpan`). A section that passes at `dc` and fails there is
`ErrUnsupported` whichever sentinel the second run raised, since the stated
setback lies somewhere in the span and this evaluator cannot decide which
side of the failure it is on. The contact floor (`1e-9` of the section's
diameter) is about 10⁸ times a real conversion's rounding, so the second run
changes the answer only where a corner's miter foot moves faster than that
ratio per unit of offset; it is run rather than argued away because that
speed is `1/sin(α/2)` at a corner of angle `α` and has no bound.

The equal-setback band charges `d`'s rounding the same way. A millimetre
setback converts exactly, its span is the single point `dc`, the second audit
does not run, and every reading is the one `dc` alone gives.

With no option, or with `otherDistance` equal to `d` in the same unit, the
band reads `dc = ds = d` and builds the same body bit for bit.

### 8.4 Measurements + tessellation

`capBlendPayload` owns analytic patches. Report each exactly representable
result as `Exact`; report every float/transcendental result with a proven bound.

Compute area, signed volume, and first moments by closed-form surface integrals
over each trimmed patch. Parameter domains are line/circle intervals, tube
angles, and spherical normal polygons; all integrands reduce to polynomials and
trigonometric endpoint terms. NEVER use quadrature to claim Exact.

A trigonometric endpoint term is ENCLOSED, never trusted from `math`. A `Cone`
patch's volume flux and first moments are evaluated over exact rationals with
the sine and cosine of each held float angle read through the certified radian
enclosure (`internal/proofbound/interval_trig.go`'s `RadSinCosInterval`, over
`internal/proofbound/turn_trig.go`'s series). The held value is the midpoint of
that enclosure at the held parameters. The magnitude envelope
(`conservativeValueError`) stands only where no enclosure can be built: a
non-finite coordinate. A `Cone` patch is never `Exact`: the enclosure always
has width.

**The held numbers.** The parameters the closed forms read are not all the
values the band's closed surface needs. Each window end `th0`, `th1`, `capTh0`,
`capTh1` is a float `Atan2`, an `ArcSeg` wall's side radius is the
`math.Hypot` of its recorded `Start`, and the cap radius is the float offset
`R ∓ dc`. The two closing disks are the side record's own region and the cap
face's recorded loop, so a patch integrated at the held floats does not meet
them, and a patch's flux is not translation invariant: the gap is read
against the centre's distance from the plane-local origin. A slot whose
half-turn ends sit `10⁶` mm up the `v` axis published a volume `6.11e-08` mm³
from its exact value under a `5.72e-08` mm³ bound, and an L at the sketch
origin missed by `7.854e-13` against `7.852e-13`. So each patch carries a
`capband.HeldAllow`, a proven bound on `|held − reference|` for each of those
six numbers:

| Held number | Reference | Allowance |
|---|---|---|
| side `th0`, `th1` | exact angle of the point the record denotes at that walk end | `AngleAllow`: `Atan2Interval` of the held end, widened by `(π/2)·b/ρ` for the walk's own end bound `b` |
| cap `capTh0`, `capTh1` | exact angle of the held cap vertex | `AngleAllow` with no reach |
| side radius | the record's `|Start − Center|` | the walk's `RadiusBound`; zero for a `CircleSeg` |
| cap radius | the cap face arc's radius, read through one foot (`offset2d.ArcSegment`) | `RadiusAllow`: either foot's exact distance from the centre |

An apex patch's side directrix is the corner itself, a radius of exactly zero,
so no term reads its side angles and they carry no allowance. A whole turn
reads its windows as exactly `2π` from the rational bracket of `π`, never the
held `fl(2π·T)` difference, and its cap face records the circle at the held
cap radius, so only its side radius has an allowance. The flux and first
moments are enclosed a second time with every held number boxed by its
allowance: each sine and cosine widened by its angle's allowance (both are
1-Lipschitz), the ruled-angle and phase integrals by the larger of their two
ends' phase allowances (the phase is linear between them), and the moment
coefficients over interval arithmetic (`internal/capband/moment.go`'s
`ivRing`). The published bound is that box's reach from the held value, so
the value is unchanged and the bound covers the patch at its references. The
chord-versus-locus term's two reference sectors (§8.3) take their own
window's allowances on both directrices. The area's frustum sector charges
the side radius allowance as `αc·e·(2·(R0+R1) + |H| + e)`. Its corner-skew
term is not monotone in the side radius, because the slant shrinks as `R0`
grows toward `R1`, so it reads `R0 + e` and the held slant plus `e` (the
slant is 1-Lipschitz in `R0`) and bounds the term over the whole allowance.

At the references every patch meets both disks along its directrices, and two
patches meet along a corner ruling only to within a gap: the held ends two
walks meet at need not be one float, a walk end names its denoted point only
to within its end bound, an `ArcSeg`'s `End` need not lie at its `Start`'s
radius, and a cap arc is recorded through one foot while the other sits at its
own distance. `capBandClosure` (`internal/capband/closure.go`) sums those gaps per
corner. Two rulings at most `gap` apart and at most `slant` long bound a
sliver of area at most `(slant + gap)·gap`, whose flux is at most that area
times `|P|` over the band and whose first moment is at most that area times
`|P|²/2`. A straight wall whose ends carry a bound integrates its held side edge
while the side face records the denoted one, which leaves a flat strip of area
at most `(L + bS + bE)·(bS + bE)` at the side level, read at that level's `|z|`
(`z²/2` for the axial moment), with each corner's `gap²` at both levels. Every
closure term is zero for a section whose walk ends are recorded, whose arcs end
at their start radius and whose circular patches' cap feet lie at one exact
distance from the centre, so an axis-aligned section's band charges nothing
here and its `Plane` patches' exact readings are unchanged.

Compute bounds from patch boundary extrema plus interior stationary points.
An unisolated stationary family is `ErrUnsupported` at build, not a loose Exact
box.

**The cap contour's displacement.** A cap-blend band has two directrices and
only one of them is recorded. The side contour is the receiver's own loop held
at its own `(u, v)`, so its coordinates are the record's. The CAP contour is
not: every corner of it comes out of the float offset solve — a line/line,
line/circle or circle/circle intersection over directions divided by a hypot —
so it is a COMPUTED coordinate sitting some distance from the point the offset
denotes. Derive that distance ONCE per band and route every cap-level reading
through it. A zero bound there publishes an `Exact` the solve never had; an
infinite one, beside a finite value, bounds nothing.

Derive it as an ENCLOSURE, never as an error model: re-evaluate the same closed
forms over rational intervals with the recorded coordinates taken exactly and
outward-rounded square roots, and report the enclosure's greatest reach from the
float point the build holds. A straight wall's carrier, foot and frame run
along the difference of its two enclosed endpoints, never along the walk's
held tangent, which is that difference rounded to float64. A circular wall's
foot steps along the radius from its recorded centre to its enclosed walk end,
never along the held tangent, which is a `math.Sincos` at a computed angle.
The offset amount is itself an interval: the setback the caller stated lies
within its own unit-conversion rounding of the float `dc` (§8.3.1), so every
carrier, foot and radius is enclosed over that whole span. Interval arithmetic
is inclusion-monotonic, so the box holds the denoted point whatever the
platform's `sqrt` and `hypot` did, and nothing in the derivation assumes an ulp contract. Where no bounded box exists
the call is SX14. A G1 join's corner is not a carrier intersection: its
denoted point is `v + dc·n̂` for the leaving wall's exact unit normal, and the
enclosure is the hull of the two shared-normal feet (the arriving wall's and
the leaving wall's, the same enclosure a reflex corner's two feet take), read
against the held foot. The hull is what charges the dead zone: the two normals
differ by at most the classification's own `|cross|`, so a join classified G1
with a residual turn is displaced by at most what the hull spans.

The readings that carry it: every cap-level vertex `bound`; every cap-level
edge length — the corner-to-apex slants, a wall's own cap edge, a reflex
corner's connector arc, and a whole circle's circumference — beside that edge's
own evaluation error; and the payload's directional extent, weighted by how much
of the direction lies in the plane. The contour's cap level separately retains
the receiver end's axial displacement, so an axial direction reads that term
even though it reads none of the contour displacement.
`Bounds` reports the same figure as the box's own `Bound`, per candidate: a
contour that loses the extremization contributes nothing, so an UNPLACED plate
whose world-axis extremes are all recorded coordinates still reports an `Exact`
box. That contour term is not the whole exactness test. The same reading also
charges the frame and placement's own rounding of the coefficients it
decomposes the direction into, and the rounding of its own summation of those
terms with the extremized candidate (`docs/evaluator-design.md` §5), so a
PLACED plate is `Exact` only where both of those terms are zero as well —
which a translation satisfies only where its own endpoint recombinations are
representable too, not merely where the offset is. Translating a 10 mm extreme
by 2 keeps `10 + 2` representable and reads `Exact`, while the equally
representable offset `0.1` makes `10 + 0.1` unrepresentable and reads
`Approximate`. A rotation fails the coefficient term outright.

Tessellation chords each shared boundary once. For a two-parameter tube patch,
choose parameter counts so the sum of path sagitta and minor-circle sagitta is
`<= tol`. Reuse the same samples on every adjacent patch. Mesh remains
watertight and carries the proven maximum sum.

The chamfer's cap contour is a COMPUTED offset (a float line/circle solve),
not a recorded one, so every reading built from it carries that offset's own
proven displacement (delta) beside its arithmetic bound, exactly as a
recorded 2D section carries its own displacement one layer down
(`docs/prism-boolean-design.md` §7's `sectionDelta` identity). Four readings
need it, each composing a different existing bound for a different reason:

- the chamfered cap FACE AREA (the offset loop's own enclosed area, feeding
  both `capStart`/`capEnd`'s `Face.Area()` and `Body.Area()`) composes
  `sectionDisplacementArea(delta, walks, perimeterUpper)` — the same 2D
  set-displacement identity a recorded section's own area composes, since the
  offset loop IS a 2D boundary known only to within delta of the one it
  denotes;
- the BAND VOLUME composes `sweptVolumeAllow(delta, areaUpper)` exactly ONCE,
  after the flux sum, with `areaUpper` the band's own patch area plus the
  cap-level disk area it closes on — never inside the disk area's own bound or
  inside a patch's own flux term, because the flux integral already reads the
  SAME displaced cap-level coordinates the disk does, and composing the term
  in both places would charge it twice;
- each BAND PATCH's own area (one ruled quad between a side-level chord and a
  cap-level chord displaced by delta) composes
  `bandPatchAreaAllow(delta, chordUpper, slantUpper)` — a ruled quad's area is
  its chord length times its slant distance to first order, so the chord's own
  length can move by `sectionDisplacementLength(delta, 1)` and the slant can
  move by delta, each read against the OTHER factor's own held magnitude;
- the BODY'S FIRST MOMENT (the centroid's own numerator, `Body.Centroid()`)
  composes `sweptMomentAllow(delta, areaUpper, coordUpper)` exactly ONCE per
  band, after the flux sum — the same "composed once" rule the band volume
  follows, and for the same reason: the symmetric difference between the band
  the build holds and the one the offset denotes has volume at most
  `sweptVolumeAllow(delta, areaUpper)`, and every point of that difference
  lies within `coordUpper` of the plane-local origin, so the moment it can
  carry is at most that volume times `coordUpper`.

All four helpers are zero wherever delta is zero (an axis-aligned section's
exact miters), which is what keeps an all-Plane cap loop's Exact volume and
its exact-rational centroid unchanged in that case.

**A `Cone` patch's area.** The patch publishes the frustum-sector area
`A₀ = (αc/2)·(R0+R1)·L`, `L = √(ΔR² + H²)`, at its cap sweep `αc`. The patch
the build holds is ruled between two windows that differ at a mitered corner:
`P(u,t) = (1−t)·S(u) + t·C(u)` over `[0,1]²`, the side end `S(u)` at radius `R0`
and angle `θs0 + u·αs`, the cap end `C(u)` at radius `R1` and angle
`θc0 + u·αc`, and the two levels `H` apart. Its area differs from `A₀` by at
most

```text
|A − A₀| ≤ L·(R0·(Φs+Φe)/2 + αc·R1·Φ²/4) + R1·Φ²·(αs·R0 + αc·R1)/4 + H·αc·R1·Φ/2
```

where `Φs` and `Φe` bound the corner skews `φ = θc − θs` at the window's two
ends and `Φ = max(Φs, Φe)`. The derivation:

- `φ(u)` is linear in `u`, so `|φ(u)| ≤ Φ` everywhere, and
  `αs − αc = φ(0) − φ(1)`, so `|αs − αc| ≤ Φs + Φe`.
- In the frame turned to `θs(u)` the integrand is `|V|`, with
  `V = (b·H, −a·H, a·R1·sin φ − b·(R1·cos φ − R0))`,
  `a = −t·αc·R1·sin φ` and `b = (1−t)·αs·R0 + t·αc·R1·cos φ`. `A₀`'s integrand
  is `|G|` with `G = (b₀·H, 0, −b₀·ΔR)` and `b₀ = αc·((1−t)·R0 + t·R1)`.
- The difference regroups exactly as `V − G = (b−b₀)·(H, 0, −ΔR) + (0, −a·H, 0)
  + (0, 0, ((1−t)·αs·R0·R1 − t·αc·R1²)·(1 − cos φ))`, with
  `b − b₀ = (1−t)·R0·(αs − αc) − t·αc·R1·(1 − cos φ)`.
- `||V| − |G|| ≤ |V − G|`, `|sin φ| ≤ Φ` and `1 − cos φ ≤ Φ²/2`; integrating
  over `t` (each of `t` and `1 − t` integrates to `1/2`) gives the bound.

`αc` is read at `|held| + capThAllow` and `αs` at `αc + Φs + Φe`. Each corner
skew is the angle between the two ends' directions from the wall's centre: both
ends and the centre are float64s, so the directions are exact rationals, and
the angle is the `Atan2Interval` enclosure of their cross and dot products. The
patch's ruled correspondence takes the skew on the branch its held windows
name, and that branch is the principal one wherever both the enclosure and the
held skew lie inside `(−π/2, π/2)`, since any other branch sits more than `π`
from the held skew. A corner outside that range is `ErrUnsupported` (SX14):
the band exists and this evaluator proves no area bound for its ruled patch.
The sum is formed over rationals with the one square root rounded up, so the
bound assumes no ulp contract. The term compares the ruled patch the build
holds with `A₀`. The cap contour's and the side level's displacements from the
denoted patch are the separate `contourAllow` and `levelDelta` terms below.

The term is zero where both corner skews are. An apex patch's side directrix
is the corner point itself, the same point at every angle, so it pairs with
the cap arc at the arc's own angle and both skews are zero. A whole turn pairs
its two circles at their seams and keeps that pairing all the way round, so
both skews are the seams' one angle, zero when both seams lie on one ray.

Every other reader of a patch's skew reads the same two proven numbers
(`capPatchWindowSkew`, the larger of them): the volume's chord-versus-locus
term (§8.3), the mesh's `skewGap` and the gate on its per-cell twist term
(`docs/tessellation-reach-design.md` §7), and DX8's undecided gate. None reads
a difference of the held windows `th0`, `th1`, `capTh0`, `capTh1`.

A band's SIDE level is displaced too, and by a different mechanism, so it is a
separate term with its own helper. `sideZ` is the single float sum
`capZ + matSign*ds`, so the whole side directrix translates rigidly by that
sum's own rounding (`levelDelta`) rather than moving point by point the way a
solved contour does. Every reading built on that level charges it: a slant
edge's own length, the band volume, and each BAND PATCH's own area, which
composes `bandLevelAreaAllow(levelDelta, directrixSumUpper)` — under a rigid
translation of one directrix a patch's area moves at the rate of its two
directrix lengths, which is the Plane arm's two chords and the Cone arm's two
frustum arcs alike. A patch bound that reads the held level as an exact input
bounds only the patch the build HOLDS, not the one the chamfer denotes, and
the gap is whole square millimetres wherever the sweep is large enough to
round that sum. The term is zero wherever the sum is exact, which is the
ordinary sweep and setback.

**First moments and the centroid.** Compute the body's own first moment
`M = ∫ p dV` in the payload's plane-local `(u, v, z)` coordinates, decomposed
the same way the volume already is: a signed slab term per loop (that loop's
own first moments, from a signed sibling of the region-area integral, times
its straight height, with the z component the elementary
`A·h·(zLo+zHi)/2`) plus a band term per chamfered cap (the divergence theorem
with `F = (u²/2, 0, 0)`, `(0, v²/2, 0)`, `(0, 0, z²/2)` over the SAME closed
band-plus-disks sub-solid the volume integrates — the two flat disks
contribute to the z component only, since their outward normal is `±ẑ`). A
flat `Plane` patch's own first moment is exact rational, the same
`(x_a²+x_b²+x_c²+x_a·x_b+x_b·x_c+x_c·x_a)/24` triangle identity one degree
higher than the tetrahedron identity the volume uses; a `Cone`/apex/
whole-turn patch's is a closed-form Fourier sum over a finite set of phases
`k·θS+m·θC` (`|k|+|m| <= 3`) whose coefficients are exact rationals in the
patch's own held floats, each phase's integral `cos(mid)·sinc(width/2)` (and
the sine analogue) enclosed through the same certified radian enclosure the
volume's own eccentric origin term and ruled cross term take; the bound is
the reach, from that held value, of the same sum over the patch's held
numbers boxed by their allowances (the held numbers paragraph above), and
never a magnitude envelope of the coefficients. The whole-turn window
collapses to the `k+m = 0` terms, whose phases are identically zero, so the
held value is an exact rational in the held floats and the bound is the
reach of those terms' coefficients with both swept angles read as `2π` — the
moment's own analogue of the volume's zero-valued eccentric origin term
there. Each ruled `Cone` patch also charges the first moment for its
chord-versus-locus gap (§8.3). The region between the built solid and the
denoted one has at most the volume term's own measure (its flux divided by
3). Every point of it lies within the band's coordinate envelope (the
original loop, the cap boundary widened by the contour displacement, and both
levels) plus the patch's radial gap `|R0 − R1|`, so each moment component's
bound grows by that volume times that reach (`capband.ChordLocusVolume`).
The centroid divides the summed first moment by the body's own volume and
lifts the plane-local quotient to world through the same frame/placement lift a prism centroid
uses, with the geometric safety-net bound (the true centroid lies within the
body's own `Bounds` box) standing as a `math.Min` ceiling on the formula
answer, never the whole bound.

## 9. Shell reach

### 9.1 `stackedPrismPayload`

General prism shells are a finite axial stack of exact line/arc regions. One
axial slab may contain several disconnected regions. `docs/stacked-prism-design.md`
owns the payload: its record (`prismSlab`, `prismSlabInterface`,
`stackedPrismPayload`), its invariants, its body build, its measurements, its
tessellation and what every consumer does with it. The analytic blind `Cut`
builds it over slabs of one region each and interfaces whose exposed material
is each exclusive hole's own interior. The rules below are the shell cases
this section adds on top of that record: the cup and BX8 build slabs of
several regions.

Slab intervals are ordered, have positive height, and have disjoint interiors.
Consecutive intervals meet at exactly one axial plane. Region interiors within
one slab are pairwise disjoint. Material is the union of every region prism.

At each shared plane, the shell construction records a certified partition into
exposed lower material and exposed upper material; the material on both sides
is the narrower region itself and is not stored. Every region on the narrower
side is proven contained in its paired region on the wider side; any relation
outside that subset/equality form is SX8. The payload builder therefore does
not hide a general planar Boolean.

Builder rules:

- build side faces from every slab region;
- cancel the certified coincident material at each slab interface;
- emit each exposed planar difference region once;
- join coincident side patches with equal carriers + orientation;
- connect two region prisms across a shared plane only when their certified
  intersection has positive area; boundary-only contact does not connect them;
- emit one `Lump` per connected component and one void `Shell` per enclosed
  cavity;
- mint roles from slab + region + result-record indices.

Mass properties are bounded sums of all region-prism integrals. Bounds compose
from the slab-region bounds. Tessellation chords a section curve once per
shared carrier and triangulates exposed planar differences.

A cup's record is this payload. A cup over a section with `k` holes is one
floor slab containing `P` inward or `Q` outward, plus one wall slab containing
the outer band and `k` hole-lining bands as separate regions. Every wall region
has positive-area overlap with the floor region, so the component graph proves
exactly one lump. The interface partition — `docs/stacked-prism-design.md`
§2.2's lining reading — emits the remaining cavity-floor face once, and records
it as the cavity region itself.

The cup keeps its own payload type, `cupPayload`, which holds that stacked
record beside the shell morphology payload verification §4.1 reads: the
thickness, its conversion displacement, the offset region's proven
displacement and the sense. The stacked build makes the body under the cup's
own roles (modify Table B, B5/B6) and face order, so public topology is the
cup's. Every cup consumer — the wall, undercut and minimum-radius surveys,
interference, the tessellator, mass properties, the tolerance gate — reads a
view of the cup that the record re-derives from its slabs, and answers as
before. A distinct type keeps every reader that dispatches on
`stackedPrismPayload` — the analytic booleans, the mirror join, a pattern, the
brep face view — from treating a cup as a boolean-built stack: RX5 stays base
S3.

The offset region's loops — the cavity's inward, the outer region's outward —
carry the cup's offset displacement column by column, not as the payload-wide
`sectionDelta`, which stays zero: the receiver's own loops are its exact
section, and charging them the offset's displacement would widen every reading
of the receiver's own walls. A migrated cup's readings agree with the
two-prism formulas `A_O·h_O − A_C·h_C` within the published bounds, which are
the same order and often tighter.

### 9.2 Side-opening section

Scope: hole-free prism section and one proper connected selected run on its
outer loop.

The kept side faces form one open boundary chain `K`. Build the wall section
directly; do not subtract a closed cavity that would fabricate a face over the
opening.

Inward wall section walk:

1. walk original kept chain `K`;
2. connect its end to the material-side offset of `K` by the exact normal
   segment of length `t`;
3. walk the offset chain in reverse;
4. connect back to `K` by the other exact normal segment.

Outward uses the outward offset chain as the outer walk and original `K` in
reverse. Audit the closed wall section with base §5. Offset drop/merge remains
S11/SX8.

The selected removed run appears nowhere in the wall-section boundary, so no
side face is emitted across the opening.

Cap slabs:

| Sense | kept start cap | middle | kept end cap |
|---|---|---|---|
| inward | `P` on thickness `t` | wall section | `P` on thickness `t` |
| outward | outer offset `Q` extending `t` below | wall section | `Q` extending `t` above |

Omit a cap slab when that cap is selected as an opening. Inward cavity height
is `h - k*t`, where `k` is kept cap count; reaching zero is SX11.

`WithNoOpenings` uses both cap slabs and a closed middle wall section: `P \ Q`
inward, `Q \ P` outward. It produces one outer shell plus one void shell.
`Shell.IsVoid()` is true only on the inner shell.

`shellClosedPrism` (`shell.go`) builds it as three slabs: inward `P` on
`[z0, z0 + t]`, the band `{P, reverse(Q)}` on `[z0 + t, z1 − t]` and `P` on
`[z1 − t, z1]`; outward `Q` on `[z0 − t, z0]`, `{Q, reverse(P)}` on `[z0, z1]`
and `Q` on `[z1, z1 + t]`. Both interfaces are monotone
(`docs/stacked-prism-design.md` §2.2): the band's one hole is upper-only at the
first and lower-only at the second, and each exposes the cavity region (`Q`
inward, `P` outward) as the cavity's floor and ceiling. Each derived level
carries its source end's displacement, the thickness conversion and its own
float sum's rounding, as a cup's floor level does, and the offset loops'
proven displacement is the stack's `sectionDelta`. The gates run in the order
S18, S10's section limit, SX11 (`t` below `h/2`, inward), S11a, the §5 audit.
The cavity's walls, floor and ceiling form a connected face set touching no
outer wall and no end cap, which the stacked build reads as the void shell of
the one outer lump. Roles are the stack's own: outer walls
`slab(0).region(0).side(0,j)`, cavity walls `slab(1).region(0).side(1,j)`,
`capStart`/`capEnd`, the floor `floor(0,0)` and the ceiling `ceiling(1,0)`.
A stack enclosing a cavity has no brep face view, so a later modify op on the
closed shell is `docs/brep-modify-design.md` SB2.

**The side opening is `docs/shell-opening-design.md`'s.** The normal segment
above is that document's rim rule at a right angle only (its §2.2), and the
cap slabs and the middle slab hold different loop records along `K`, which no
stack column carries as one face (its §1). That document owns the rim at
every corner kind (Table RO), the three regions, the `brepPayload` the result
is recorded on through `internal/stackedbrep`, its refusals (Table SO), its
consumers (Table DO) and its PR split. A prism side opening builds where
every section walk is a line or a circular arc (`shell_opening.go`), and
its increment table lists what still refuses. A revolve side opening (§9.3.2)
builds with that document's rim at every line and arc corner (its §8).

For cap-only removal from a holed section, build the wall as one slab with
`1 + k` regions: the band between the paired outer loops first, followed by one
band between each paired hole loop in `ProfileRecord` order. The base
S18/S10/S11/§5 gates prove those bands are regular and pairwise disjoint before
payload construction. The slab is a prism group (`docs/stacked-prism-design.md`
§2.2) whose disjointness is that audit's, not `provePrismRegionsDisjoint`'s: a
hole lining lies inside the outer band's own hole, which a scene of outers alone
would read as nesting. The offset loops sit within the offset displacement of
the offset the thickness denotes, and the group carries it as its
`sectionDelta`. The resulting `1 + k` connected components lift base S12
without admitting holed side-opening or no-opening shells.

### 9.3 Revolve shell

Use the same line/arc offset on an **effective meridian**. A walk on the
revolve axis emits no face and MUST NOT grow a wall:

- profile strictly off axis → effective meridian is recorded region `P`;
- profile with an on-axis walk → reflect `P` across axis, union the two exact
  halves, run offset/open-chain construction on that symmetric region, then
  restrict result to the non-negative radial half-plane.

Build the symmetric union by cancelling each on-axis walk against its reflected
reverse and stitching the remaining line/arc walks at their shared endpoints.
Do not invoke a sampled/general 2D union.

Reflect selected generated-side walks with the region. The axis walk is never
selectable and never part of kept wall chain. This is the same symmetry rule
the full-revolve wall survey uses; offsetting raw `P` would fabricate a tube
around the axis of a solid cylinder.

Full turn:

- side opening: build one open-chain wall region as §9.2, then full-revolve it;
- no openings: build `P \ Q` inward or `Q \ P` outward, then full-revolve it.

Partial turn:

- both angular caps MUST be selected;
- optional generated-side opening follows the same open-chain rule;
- revolve the resulting wall region over the unchanged angular interval;
- `evalRevolve`'s two cap faces are the rim bands at the openings.

A partial turn that keeps an angular cap needs a plane offset by constant
distance. That plane is not another constant-angle radial plane: sweep-angle
reduction gives radius-dependent distance. SX8 stages it; NEVER approximate it
by changing `phi0` / `phi1`.

Receiver meridian profile MUST be hole-free for this extension. Hole-carrying
or topology-changing offsets remain SX8.

#### 9.3.1 Construction

`shell_revolve.go` builds the partial turn without a side opening and the
full turn under `WithNoOpenings`, from one wall region. It never materialises
the mirror half. The effective offset cut back to `ρ ≥ 0` is the
offset of the kept chain `K` alone, the recorded meridian less its on-axis walk
`A`. Interior corners of `K` take modify §7's join. Each end of `K` takes the
join of the corner `K` makes with its own mirror image there, read from the walk
and the axis line alone:

| Corner `K` makes with its mirror | Cut-back join |
|---|---|
| miter | `K`'s offset carrier met with the axis line, where the mirror carrier meets it |
| G1 (`K` meets the axis at a right angle) | `K`'s own offset foot, taken as the axis point at distance `t` from the corner, so a foot stepped along a float normal (an arc's tangent read through its angle) cannot land a rounding off the axis |
| arc | the arc about the corner from `K`'s offset foot to the axis point at distance `t` from the corner, which is the whole arc's midpoint |

The mirror tangent `K`'s own tangent reflects to only classifies the corner by
modify §7's rule. The cut-back offset `Q` therefore starts at `qB` and ends at
`qE`, both on the axis, and closes along it. `A` runs from `E`, where `K`
arrives, to `B`, where it leaves. The wall region is one loop:

- inward: `K`, the axis from `E` to `qE`, `Q` backward, the axis from `qB` to
  `B`;
- outward: `Q`, the axis from `qE` to `E`, `K` backward, the axis from `B` to
  `qB`.

The four axis points lie on `A`'s line in one order: `E, qE, qB, B` inward and
`qE, E, B, qB` outward. An offset whose ends land out of that order has crossed
its own mirror image on the axis, which is S11b. `Q` closed along the axis then
faces modify §5's audit (S8, S11b, S9). The axis walk is never part of `K`, so it
grows no wall. A meridian strictly off the axis is its own effective meridian,
and its wall region is the tube section of modify Table B: `{P, Q}` inward and
`{Q, P}` outward.

S10's section limit reads the effective meridian's inradius. With an on-axis
walk, the survey reads `K`'s walks and their mirrors in axis coordinates, where
the mirror is `ρ ↦ −ρ` exactly. A solid cylinder of radius `R` and height `H`
keeps a cavity below `min(R, H/2)`, not below the half-section's own inradius.

The wall region then passes the revolve axis gates (evaluator §6) about the
receiver's oriented axis. Where the gate finds the region across or touching
the axis in a form it refuses, the offset has reached the axis and would meet
its own mirror image. That is SX8 (`ErrUnsupported`), not the base call's
`ErrDegenerate`: the shelled body exists. An outward wall off a meridian closer
to the axis than `t` is the usual case.

The result is a `revolvePayload` over the wall region with the receiver's frame,
axis, angular interval, denotation and placement, built by `evalRevolve`. The
wall's offset coordinates are float cuts: a slanted walk's foot, a miter of a
slanted carrier with the axis (the cone apex's `20 − √5`), a connector arc's
foot. The wall therefore carries the proven distance from its record to the
wall it denotes as its `sectionDelta`, with `sectionWhole` set, and every
reading charges it (`docs/surface-intersection-design.md` §7.2). The figure is
modify §9's `offsetSectionDelta` argument: three times the largest reach of a
recorded join point from its rational enclosure, over every denoted thickness
within the thickness's own conversion bound. A wall off the axis reads the
prism cup's `offsetSectionDelta` over the meridian's closed offset unchanged.
A wall with an axis end or an opening end reads `offset2d.ChainReach` over the
open chain `K`: an interior corner by `LoopReach`'s own enclosures, an opening
end by its rim cut (`offset2d.OpeningReach`, `docs/shell-opening-design.md`
§2.4), and an axis end by the join it builds against
the receiver's axis line widened by its four proven bounds — the offset
carrier met with that line for a miter, the line point `t` from the corner
for a G1 end (charged the hull with the walk's own foot), and both for an arc.
`K` and the axis points it leaves from are the receiver's own record and move
by nothing. The figure is published only where it is nonzero, so a
right-angle or exact-level shell — every enclosure a single point equal to the
float the build holds — keeps a zero `sectionDelta`, `sectionWhole` false, and
reads bit for bit as an undisplaced revolve. A shell whose wall carries a
displacement refuses a second shell and a junction blend, since each rewrites
the recorded meridian.

A full turn sweeps that same wall region a whole turn, and evaluator §6's shell
rule splits the result into an outer shell and one void shell. Off the axis the
wall region's offset loop is a hole, which sweeps the toroidal cavity wall. With
an on-axis walk the wall region is one loop meeting the axis along two walks,
`E`–`qE` and `qB`–`B`. Its two runs, `K` and `Q`, sweep the two closed surfaces,
and the run whose axis ends bracket the other's is the outer one: `K` inward,
`Q` outward. The receiver's own surfaces are therefore the outer shell inward
and the void shell outward. Only S10's section limit bounds the thickness, as
for a partial turn, since a full turn keeps no angular floor.

Stage 4's revolve gates run in this order: the receiver's section-displacement
guard (`requireExactRevolveSection`), a holed meridian (SX8), a kept angular
cap (SX8), more than one on-axis walk (SX8), then a removed side run that is
not one proper connected run of whole walks or that leaves two kept chains
(SX8, §9.3.2). `WithNoOpenings` on a partial turn or on a holed meridian is SX8 before the shell is routed here.

#### 9.3.2 Side opening

`revolveShellSideWall` (`shell_revolve.go`) builds the side opening of a full
turn, and of a partial turn beside both removed angular caps. The removed
faces name the recorded segments they sweep through their `side(0,j)` roles,
and they must cover whole plane-local walks. The removed walks form one
proper connected run. On a meridian with an on-axis walk the run must touch
the axis walk at one end of the chain the axis walk leaves: a run between two
kept walks would leave two wall regions, which one revolve record does not
hold.

The kept chain `K` runs from the walk after the run to the walk before it,
skipping the axis walk. Each end of `K` is an axis end or an opening end. An
axis end takes §9.3.1's mirror join, so the offset ends on the axis, and its
axis point must land on the axis walk on the material side (S11b). An opening
end takes `docs/shell-opening-design.md` Table RO's rim: the removed
neighbour walk's own carrier from `K`'s end to its cut with `K'`
(`offset2d.OpeningJoin`, `shell_chain.go`'s opening end), a `LineSeg` or an
`ArcSeg` about the removed walk's centre. The wall walks `K`, the rim at
`K`'s end, `K'` backward and the rim at `K`'s start, inward; and `K'`, the
rim back to `K`'s end, `K` backward and the rim out to `K`'s start, outward. The wall faces the §5 audit (S8, S11b, S9) and the axis
gates before it is swept, and the result's roles are its own `side(i,j)`.

An opening end at a smooth or cusped corner, or one whose removed carrier
never reaches `K'` before leaving the wall's band, is that document's SO1; a
cut in the removed walk's span direction at or past its far end is its SO2.
Both are `ErrUnsupported`. A right-angle end, where the two walks' tangents
have a float dot product of exactly zero, keeps the offset foot it has always
written, so its wall is the normal segment of length `t`. Every cut is charged
by §9.3.1's displacement, the rim cut through its own enclosure. A side
opening runs no section limit: its cavity opens through the removed faces, so
the open chain's own S11a drop gate and the wall's §5 audit decide it.

## 10. Table BX — results + roles

| BX | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BX1** | prism lateral fillet/chamfer | base `prismPayload` | base B1 | base roles |
| **BX2** | revolve junction fillet/chamfer | `revolvePayload` over rewritten meridian | existing full/partial revolve topology | `side(i,j)` + blend role on inserted wall |
| **BX3** | complete prism cap-loop fillet/chamfer | `capBlendPayload` | trimmed cap/sides + analytic blend patches | result side/cap roles; `filletCap(c,l,p)` / `chamferCap(c,l,p)` per patch |
| **BX4** | prism shell with side opening | `docs/shell-opening-design.md` Table BO: `prismPayload` over the wall section with both caps removed (BO1), else `brepPayload` (BO2) | one lump, one shell, no void | BO1: `side(0,j)`, `capStart`/`capEnd`; BO2: `face(k)`/`wall(k)` |
| **BX5** | prism `WithNoOpenings` | `stackedPrismPayload` | one lump; outer + void shell | outer/inner/result-slab roles |
| **BX6** | full-revolve shell | `revolvePayload` over wall region | evaluator §6's full-turn shells: one lump, an outer shell and one void shell | result `side(i,j)` roles |
| **BX7** | partial-revolve shell with both caps open | `revolvePayload` over wall region | one shell with two rim-band caps | result sides + `capStart` / `capEnd` |
| **BX8** | prism cap-only shell with both caps removed from a section with `k ≥ 1` holes | `stackedPrismPayload` with one slab and `1 + k` regions | `1 + k` lumps: outer wall band, then one band lining each hole | `slab(0).region(m).side(i,j)` plus exposed rim roles |

`c` is cap role (`start`/`end`), `l` loop order, and `p` deterministic patch
order in the result's own `capBlendPayload`. No role names a receiver index.
For stacked payloads, `k` is axial slab order and `m` is region order within
that slab. The outer-wall region precedes hole-wall regions, which retain
`ProfileRecord` order.

Do not copy ancestor `FeatureRef`s onto result faces. A consuming modify step
owns the result. Multi-role faces carry only roles minted by that step. This
keeps base §11's record-index rule and resolves ancestor provenance in favor of
the shipped behavior.

## 11. Exactness + proof discipline

RX1/RX2 outputs are analytic. Report `Exact` only when the result is proved
exactly representable. Carry proven outward bounds for computed floats; no shape
sampling enters the answer.

Proof rules:

- accept tangency only from exact analytic identities;
- accept trim membership only from closed-form parameter ranges;
- accept loop simplicity/nesting only through the existing line/arc audit;
- accept patch separation only through exact carrier intersection + exact trim
  exclusion;
- use the diameter-anchored floor only to **refuse** near contact;
- NEVER use a residual or tessellation to admit an Exact body.

A build-time question has no `Suspect`. Any proof the builder cannot finish is
`ErrUnsupported`.

`facetedPayload` stays excluded even when `meshBound == 0`. Its groups retain
provenance + a planar flag, not analytic carrier/trim intent. Modifying its held
polygons would make one evaluator's decomposition part of the operation's
meaning, while an exact kernel would modify a different B-rep. SX9 is therefore a
permanent limit of this evaluator reach, not an unfinished zero-bound shortcut.

**What SX9 leaves a caller proving.** Because SX9 never lifts, a caller whose
part fillets or chamfers a faceted boolean result cannot reach the modified
solid here at all, and waiting is not one of the options (an analytic
`brepPayload` result is `docs/brep-modify-design.md`'s and is admitted there).
What replaces it is a proof of the
op's INPUTS rather than of its output: resolve the selector against the
boolean body and assert the edge set the step would collect, then assert the
material the op would leave — the wall or radius the requested size implies —
against the analytic bodies that went INTO the boolean, which are analytic
operands — a prism payload, a tube among them, or a cup payload — and answer
every survey from that payload. That reading has to be taken before the operands
are consumed, since a boolean retires them. It proves a weaker claim than the
built solid would, and a caller choosing it should say which claim it is: the op's
inputs are well formed and the material it removes is affordable, not that the
modified body exists and is sound.

Taking the reading off the operands is what makes the substitute reachable at
all. The wall, undercut and minimum-radius surveys each answer from an analytic
payload, so the same question asked on the boolean RESULT is undecided today and
reads `Suspect` — the faceted surveys that would answer it are
`docs/payload-verification-design.md`'s design, not shipped behaviour — while
asked on an operand carrying an analytic payload, a prism or a cup, it is
answered outright. An operand that is itself a boolean result carries a
`facetedPayload` and leaves the same question undecided, so the substitute
reaches only the analytic bodies at the start of a chain.

## 12. Table DX — downstream

| DX | Consumer | Revolve rewrite | `capBlendPayload` | `stackedPrismPayload` |
|---|---|---|---|---|
| **DX1** | mass properties / bounds | existing bounded path | bounded analytic patch integrals | bounded slab-region sums |
| **DX2** | topology / structural Verify | existing builder | payload builder | slab-region union builder |
| **DX3** | `Tessellate` / STL / OBJ | existing revolve tessellator | patch tessellator: one count per wall walk shared by the side wall, the band patch and the cap contour (`docs/tessellation-reach-design.md` §7) | required slab-region tessellator |
| **DX4** | mesh boolean | existing revolve operand path, with its own facet ceilings; no blend-specific gate | admitted for a band whose every corner is a line-line miter or an exactly G1 join, or a whole turn (`docs/tessellation-reach-design.md` §7); a circular wall at a genuine miter or a reflex corner stays `ErrUnsupported` | available once DX3 exists |
| **DX5** | `ThroughAll` directional extent | existing | analytic patch extrema, published beside the displacement a computed cap contour and the inherited axial levels give them (§8.4), and beside the frame, placement and endpoint-summation rounding every extent reading of this payload carries (evaluator §5); the stop charges that displacement to the level it resolves and refuses only where it straddles the sketch plane (evaluator §5) | union of slab-region extents |
| **DX6** | clearance | existing revolve boundary reader | add trimmed patch faces to boundary model; undecidable cells stay `Suspect`; staged for the cap-loop chamfer, whose pairs read `Suspect` unless boxes already decide them | union exposed slab faces; never include cancelled interfaces |
| **DX7** | undercut | existing revolve survey | bounded normal ranges per patch, each widened by the whole distance its own `Face.NormalAt` readings can sit from the patch's exactly enclosed normal model and by that patch's own proven departure from the surface it publishes (§8.3), a circular patch's window read through a proven enclosure rather than a float evaluation; a proven opposing point lists its patch, and a remaining straddle is undecided without removing another proven listing. The receiver's own unchanged walls and caps are not patches, and are read through the SAME three-valued rule, with the same undecided outcome — no reader may treat the receiver half as exempt | exact normal ranges per exposed face |
| **DX8** | minimum radius | existing meridian survey | minimum concave principal radius over sphere/torus/cylinder/cone patches; undecided unless every patch is proven to be exactly the surface it publishes (zero departure, §8.3) — a mitered ruled patch is one case of that | section arcs + exposed rim geometry |
| **DX9** | minimum wall thickness | existing revolve rewrite survey | staged: `DiagUnsupportedSurveyPayload`, `Survey` `SurveyWall`, `Suspect` | staged: `DiagUnsupportedSurveyPayload`, `Survey` `SurveyWall`, `Suspect` |

DX9 is a deliberate evaluator limit. A cap blend and a stacked shell are not
one constant section at one height. The existing 2D spanning-disk proof does
not decide them. The modify call still builds an exact solid; only an
explicitly asked wall survey reports the staged refusal, published as an
explicit unsupported-payload dispatch rather than a generic undecided result
(verification design §1.1). No open implementation claim remains.

Clearance may also return undecided for surface cells its certified kernel does
not solve. This is the existing `Verify` contract, not a modify-build refusal.

DX6 is staged for the cap-loop chamfer, and §14's rule that a PR may leave a DX
question staged only where this table says so is what that cell now says. It
remains required for the cap-loop fillet.

The chamfer's DX3 reading is a mesh. A patch tessellator must chord the
cap-level offset boundary and the side-level original boundary into one strip,
and the two may need different sample densities; a strip whose densities
disagree is not watertight, and a mesh that is not watertight is a wrong answer
rather than a coarse one. The tessellator answers that with ONE count per wall
walk, shared by the side wall, the band patch and the cap contour, and charges
each patch its own positional departure; the design is
`docs/tessellation-reach-design.md` §7. DX4 reads that mesh's occupied-volume
proof where §7 states one — a band of line-line miters, exactly G1 joins and
whole turns — and every boolean refuses the operand for any other band, which
serves export alone.

The chamfer's DX6 reading is `Suspect` for any pair its bounding boxes do not
already decide, which is the same staging the cup payload took before its own
clearance model landed. A trimmed patch face admitted into the certified kernel
without a proof of its trim yields a false disjointness certificate, and a false
certificate is worse than an undecided pair: `Verify` would report a clearance
the geometry does not have.

## 13. Required tests

Every implementation PR MUST add geometry assertions, not run-only coverage.

### Options

- option nil/duplicate/conflict gates;
- `WithNoOpenings` nil-selector exception and non-nil conflict;
- tangent-chain seed cardinality before expansion;
- asymmetric nested selector deep-copy;
- repeated calls select the same seeds, expansion, reference faces, and result;
- failed call leaves the body live and the document unchanged.

### Tangent chain

- line↔arc and arc↔arc proven G1 continuation;
- same tangent but non-tangent face sheets → stop;
- branch → SX2;
- near-tangent exact `no` → stop;
- oracle `unknown` → SX2;
- closed cycle visits each edge once;
- deterministic body-order result.

### Asymmetric chamfer

- different feet measured along both adjacent walks;
- reference face swaps the two distances;
- one reference face per edge over multi-edge selection;
- missing/dual/extra reference → SX3;
- independent overrun on either side → base S6;
- the option preserves the reference selector and other distance exactly.

### Revolve

- full-turn latitude fillet creates expected torus/sphere and exact radius;
- partial-turn junction arc produces same meridian rewrite;
- chamfer line classifies to cylinder/plane/cone as expected;
- bounded volume/area/centroid against rewritten-profile integrals;
- cap-edge selection → SX5;
- axis contact and spindle gates preserve base sentinel;
- roles and selectors survive placement/replay.

### Cap loops

- whole circle rim fillet has no seam vertex patch;
- polygon loop produces cylinders + spherical miter patches;
- reflex line/arc loop produces torus/cone vertex patch;
- partial loop and mixed cap/lateral selection → SX4;
- mixed cap/side asymmetric assignment on one loop → SX4;
- a two-distance chamfer of a box's cap loop and of a circular rim matches the
  closed-form volume (`ds·(LW − (L+W)·dc + 4/3·dc²)` and the frustum
  `π·ds/3·(R² + R·(R−dc) + (R−dc)²)`), and swapping the reference face swaps
  `dc` and `ds`;
- a two-distance chamfer with `otherDistance == d` is bit-identical to the
  equal one;
- a two-distance overrun across the cap → SX6, down the side → SX7, and a
  setback too small for its own axis → SX13 on that axis;
- a two-distance band's side level carries `otherDistance`'s conversion
  rounding where the side wall is referenced, and a miter ruling next to a
  circular wall encloses its locus with `ds ≠ dc`;
- opposite cap bands meeting → SX7;
- carrier collapse → SX6, including where the same call ALSO satisfies SX7's
  `reach >= height`, so the sweep height alone can never pick the sentinel;
- non-adjacent patch crossing/touch → SX7/SX12;
- every cap-level edge reports a finite length and a finite bound, and a
  `LongerThan` query no longer matches a slant edge on an infinity;
- a miter ruling adjacent to a CIRCULAR wall publishes a length bound that
  ENCLOSES its own denoted locus length, read over a family of setbacks, while
  a ruling between two straight walls keeps its arithmetic-only bound;
- a cap-level vertex's bound ENCLOSES its distance to the denoted contour
  point, taken over exact rationals from a section whose offset has a closed
  form (the 12-9-15 right triangle, whose feet are `(t, t)`, `(12 - 3t, t)` and
  `(t, 9 - 2t)` exactly);
- a body whose world-axis extremes are recorded coordinates reports an `Exact`
  `Bounds` box, and one tilted so an axis reads both plane and sweep reports the
  contour's own displacement instead;
- a PLACED cap blend whose world-axis extremes are all images of recorded
  coordinates reports an `Approximate` `Bounds` box with a positive bound, and
  that box widened by its own bound still encloses the exact rational image of
  the receiver's base loop, the band's side-level directrix and the inset cap
  contour;
- a chamfer band over a circular wall whose radius dwarfs `d` still carries a
  `Cone`, its taper still reaches DX7, and volume and area are unmoved by the
  kind decision; an offset radius identical to the wall's → SX13;
- a mitered patch's own `NormalAt` bound ENCLOSES its distance to the ruled
  surface's own normal, sampled across the whole patch and over a family of
  setbacks up to the widest the offset admits;
- a TANGENT-join band, whose two windows coincide, still carries a departure
  bound that ENCLOSES its own built ruled surface's distance from the surface it
  publishes, read at every corner of every patch from held coordinates alone.
  For a `Cone` patch that is the built tangent plane's own normal, and the
  published ruling against the published cone's generator through the same
  corner; for a straight wall's `Plane` patch it is the built quad's own four
  corner normals, taken over exact rationals against the tag the build fixed
  through three of those corners. The same band is read unplaced, rotated about
  the world origin, and rotated far out, and both the measured departure and the
  published bound grow by orders across those rows for either patch kind: a bound
  derived from the plane-local windows would be unmoved and zero in all three.
  The section is drawn at the sketch origin and carried out by the PLACEMENT,
  never drawn at large sketch coordinates, so no arrangement weld is left a
  handful of ulps of margin for a platform to land either side of;
- a body whose ruled patch opposes a pull its published `Cone` does not is NOT
  passed by DX7 — the answer is undecided, never the proven all-clear — while
  an ordinary setback's band is still cleared outright under one pull and
  still listed as a proven undercut under the opposite one;
- a pull that DX7's own reading cannot separate from a patch's tangent, on a
  band with no window skew at all, is undecided rather than answered:
  covered on the whole-turn `Cone` patch, whose arm's float cosine and sine of
  the held half angle leave the minimum component inside the bound the patch
  publishes, and on a flat patch read from one sample, against a pull
  perpendicular to the direction that reading names;
- a circular patch's reported range holds every component the patch takes, read
  independently across its whole window, on a band about the frame origin and on
  a small band placed far from it — where each sampled point's own azimuth
  displacement runs orders past the readings' own bounds, and the allowance is
  proportionately larger for it while the band about the origin pays nothing
  extra;
- the window enclosure brackets the form's own peak and trough wherever the
  window reaches them, and no azimuth of the window takes a value that
  enclosure excludes — the stationary point included, which is where a float
  evaluation of the same range escapes;
- an unmitered band whose patches carry an exactly zero departure still reports
  the proven absence of a concave feature, and the same band placed away from
  the origin is undecided;
- a chamfer band under a sweep whose height dwarfs `d` still separates its two
  levels; a side level identical to its own cap level → SX13, and the receiver
  and document stay untouched;
- every topology edge has exactly two adjacent faces;
- every patch `Face` reports its own area, and no float-computed one is `Exact`;
- an all-`Plane` cap-loop band whose true volume is a float64 reports it `Exact`
  with a zero bound, and a band carrying a `Cone` patch reports `Approximate`;
- the centroid bound encloses the true centroid, tested on a box far wider than
  it is tall — the shape whose farthest corner is neither `Min` nor `Max`;
- a cap-loop chamfer on a tangent-filleted plate (every circular wall a
  partial turn) reads `Sound` under `Document.Verify` at the default tolerance,
  with a volume bound and a centroid bound orders below the material the
  chamfer removed, drawn at the sketch origin and drawn a thousand millimetres
  from it alike — a bound that grows with the arc centres' distance from the
  plane-local origin is the magnitude envelope, not the enclosure;
- the same plate's centroid matches the erosion family's own closed form, and
  a `Cone` patch's exact-rational Fourier coefficients agree with their float
  reference term by term;
- bounded mass properties from independent closed forms;
- shared-curve tessellation is watertight and bound `<= tol`;
- selected/unselected hole loops retain correct nesting.

### Shell

- one side opening emits no face over removed chain;
- one/both/no cap openings produce correct slab intervals;
- inward `h-k*t` boundary → SX11;
- outward slabs extend by exactly `t` at kept caps;
- `WithNoOpenings` produces outer + void shell;
- slab interface faces cancel exactly;
- bounded volume/area/centroid equal slab-region sums;
- a migrated cup with `k` holes has `1 + k` wall regions, all joined through
  one floor region into exactly one lump;
- both-caps cap-only shell with `k` holes produces exactly `1 + k` regions,
  lumps, and rim sets in stable outer-then-hole order;
- side run disconnected/all-side, or side/no-opening on a holed section → SX8;
- full-revolve side opening and closed shell;
- axis-touching cylinder/sphere shell has no fabricated axis wall;
- partial revolve builds only with both angular caps selected;
- partial kept cap → SX8;
- tessellation watertight across slab interfaces and between all BX8 rim faces;
- asked wall reading is `Suspect`, never absent or fabricated.

### Faceted receivers

- positive-bound boolean receiver → SX9;
- zero-bound all-planar boolean receiver → SX9;
- refusal leaves operands live and the document unchanged.

## 14. Implementation order

| PR | Lands | Still staged |
|---|---|---|
| **A** (landed) | option records; tangent expansion; asymmetric chamfer of prism lateral edges and revolve junctions; `WithNoOpenings` accepted and refused per receiver | cap/shell reach; the asymmetric chamfer of a brep or stacked receiver (SX16); all SX9/SX10 |
| **B** (landed) | revolve junction rewrite + roles + surveys | cap loops; shell reach |
| **C1** (landed) | multi-region `stackedPrismPayload` (the lining reading); cups recorded on it; base S12 lifted through BX8 | closed + side-opening prism shell; revolve side opening; cap loops |
| **C2** (landed) | closed prism shell (BX5): the void-shell stack, its tessellation | side-opening prism shell (BX4, `docs/shell-opening-design.md`); revolve side opening; cap loops |
| **C3** (landed) | revolve shell side opening, full and partial turn, right-angle rims (§9.3.2) | a slanted rim; cap loops |
| **C4** (landed) | the side opening of `docs/shell-opening-design.md` §12, six PRs: the rim rule and the revolve's slanted rim (PR 1, landed), the rectilinear prism side opening as a `brepPayload` (PR 2, landed), circular walks (PR 3, landed), oblique walks (PR 4, landed), arc–arc corners and oblique arc joins (PR 5, landed), a kept arc extended past its end (PR 6, landed) | per that document's increment table; cap loops |
| **D** (partial) | partial-turn revolve shell with both angular caps removed and no side opening (BX7); full-turn closed shell under `WithNoOpenings` (BX6), §9.3.1 | a side opening, full or partial turn (C3 lands it); cap loops |
| **E** | `capBlendPayload`; complete cap-loop chamfer at an equal setback and at two distances (§8.3.1); analytic integrals | complete cap-loop fillet; DX4 admission for a mitered circular wall or a reflex corner; DX6 clearance model; partial cap chains; mixed edge classes; faceted receivers |

Each PR lands its result payload, structural topology, measurement path, and
tests together. A PR may leave a DX question staged only where
Table DX explicitly says `Suspect` or `ErrUnsupported`.

No implementation PR changes SX9. Brep and stacked receivers are
`docs/brep-modify-design.md`'s, not another row in this extension.

## Implementation notes

### capblend_contour.go

The cap-loop chamfer's cap contour displacement (§8.4) and
`docs/prism-boolean-design.md` §7's `sectionDelta` are the same idea applied to
two different constructions, and they are independent terms with separate
owners: `internal/capcontour/` derives the cap contour's own displacement;
`capblend_contour.go` maps it to the cap-band construction and refusal, and
the cap-blend build/measurement code (`capblend_geom.go`, `capblend.go`,
`capblend_moments.go`) carries it into every reading that needs it. No
cap-blend reading composes `sectionDelta`, and no `sectionDelta` consumer reads
the cap contour's displacement. `capBlendPayload` separately preserves its
receiver's per-end axial displacement and the selected-end setback rounding.
`capblend_contour.go` also states each circular patch's held allowances
(`capWallHeldAllow`, `capApexHeldAllow`). `internal/capband/closure.go` bounds
each band's closure slivers (`capBandClosure`), which `capblend_moments.go`
and `capblend_centroid.go` charge beside the patch integrals (§8.4's held
numbers paragraph).
