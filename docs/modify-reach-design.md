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
- no faceted/boolean receiver;
- no partial-revolve shell that keeps an angular cap.

Each excluded body exists. Table SX stages it with `ErrUnsupported` at the
modify call. No excluded shape is approximated, clipped, or deferred into
`Verify`.

## 2. Public options + recipe

The three method signatures stay unchanged. The extension fills the option
records already reserved by core §6.2.

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

The recorded option values are:

```go
type FilletOpts struct {
    TangentChain bool `json:"tangent_chain,omitempty"`
}

type AsymmetricChamferOpts struct {
    Reference FaceSelector `json:"reference"`
    Other     units.Value  `json:"other"`
}

type ChamferOpts struct {
    TangentChain bool                     `json:"tangent_chain,omitempty"`
    Asymmetric  *AsymmetricChamferOpts    `json:"asymmetric,omitempty"`
}

type ShellOpts struct {
    Sense      ShellSense `json:"sense"`
    NoOpenings bool       `json:"no_openings,omitempty"`
}
```

`AsymmetricChamferOpts.Reference` is encoded through the existing sealed
selector codec and deep-copied on call, `Recipe()`, marshal, and unmarshal.
It resolves against the receiver during evaluation. It never records resolved
faces or topology indices.

Step fields remain:

| Op | `Selectors` | `Values` | `Opts` |
|---|---|---|---|
| Fillet | seed edge query | radius | `FilletOpts` |
| Chamfer | seed edge query | positional distance | `ChamferOpts` |
| Shell with openings | removed-face query | thickness | `ShellOpts` |
| Shell with no openings | empty | thickness | `ShellOpts{NoOpenings:true}` |

Cardinality on the seed edge query applies **before** tangent expansion. This
keeps `Exactly(n)` an assertion about what the caller named, not about an
evaluator-dependent number of continuation edges.

`WithNoOpenings` and a non-nil face selector conflict: SX1. A nil selector
without `WithNoOpenings` remains base S16 (`errNilSelector`).

## 3. Table RX — receivers + targets

Base Table R still admits the shipped straight-prism cases. RX adds these rows:

| RX | Receiver | Fillet / Chamfer | Shell |
|---|---|---|---|
| **RX1** | `prismPayload` | base lateral junctions; OR every geometric edge of one or more complete cap loops. Never both classes in one call | base cap openings, with BX8 replacing base S12 after the stacked payload lands; OR, for a hole-free section, one proper connected run of outer side faces with any cap openings; OR, for a hole-free section, `WithNoOpenings` |
| **RX2** | `revolvePayload` | off-axis swept meridian junctions only: a full-turn latitude `Circle3` or partial-turn junction `Arc3` | full turn: one proper connected run of generated side faces, or `WithNoOpenings`; partial turn: both angular caps MUST be removed, with an optional proper connected side-face run |
| **RX3** | `stackedPrismPayload` | SX10 | SX10 |
| **RX4** | `capBlendPayload` | SX10 | SX10 |
| **RX5** | `cupPayload` during migration to `stackedPrismPayload` | base S3 | base S3 |
| **RX6** | `facetedPayload`, including zero-bound all-planar boolean output | SX9 | SX9 |

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
| **SX8** | shell side/no-opening extension is used on a holed prism section; a non-empty side selection is not one proper outer-loop run; offset changes topology; partial revolve keeps either angular cap | body exists outside extension | `ErrUnsupported` |
| **SX9** | any modify op on `facetedPayload` | body exists; analytic carrier + stable topology absent | `ErrUnsupported` |
| **SX10** | another modify op on `stackedPrismPayload` or `capBlendPayload` | body exists; compound feature composition not built | `ErrUnsupported` |
| **SX11** | inward closed/side-opening prism shell leaves axial cavity height `h - k*t <= 0`, where `k` is kept cap count; or section cavity is empty | no cavity | `ErrDegenerate` |
| **SX12** | cap chamfer ruled patches intersect away from shared boundaries or cannot be certified disjoint | body exists under trim kernel | `ErrUnsupported` |
| **SX13** | a cap-loop chamfer whose setback rounds away against the level it displaces: the cap contour's offset radius rounds back onto a circular wall's own radius (`R -/+ d == R`), or the band's side level rounds back onto its own cap level (`z1 - d == z1` on the end cap, `z0 + d == z0` on the start cap) | body exists; its taper is real but finer than float64 names at that radius or at that sweep level, so the band's patches cannot be told from a cylinder or from the cap plane | `ErrUnsupported` |
| **SX14** | a cap-loop chamfer whose denoted contour corner cannot be enclosed: the two offset carriers' interval intersection is unbounded, or the exact carriers do not meet where the float solve found a root | body exists; its offset corner is real and this evaluator cannot state where it is, so no cap-level coordinate there can publish a proven displacement | `ErrUnsupported` |
| **SX15** | a cap-loop chamfer whose band patch's outward orientation cannot be certified: the patch's own `Face.NormalAt` refuses at the build's orientation sample point | body exists and its patches are real; the evaluator cannot evaluate its own orientation sample on this patch, so it cannot state which side of the patch is outward | `ErrUnsupported` |

Gate order:

| Stage | Gates |
|---|---|
| 1. call | base S17/S15/S13-or-S14; option decode; SX1; seed selector S16 unless no-openings |
| 2. expansion | resolve seed; expand tangent chain; SX2 |
| 3. reference | resolve asymmetric reference; SX3 |
| 4. receiver/target | base R + RX; SX4/SX5/SX8/SX9/SX10 |
| 5. existence | base S4/S5/S18/S10; SX6/SX11 |
| 6. constructed-geometry audit | base S8/S6/S7/S9/S11; SX7/SX12/SX13/SX14/SX15 |
| 7. payload | base S12 until `stackedPrismPayload` lands; BX8 handles that exact case afterward |

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
axial setbacks and is SX4.

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

It is evaluator data, never recipe data. Recipe records only selector, values,
and options.

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
window and the narrow (cap) one, and the ruled patch's own point-for-point
departure from the wide cone is bounded in closed form from the two windows'
angular skew; `chordLocusVolumeAllow` composes both terms into one proven
volume bound. The residual, and its bound, are exactly zero wherever the two
windows already coincide: a tangent join, an apex patch, and a whole turn.

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
  rational arithmetic, and `capblend_departure.go` owns the derivation. Two
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
  `mn - allow > 0`, and is undecided otherwise. An undecided patch does not
  remove other patches already proven to oppose. Every point of the patch
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
  (`capblend_normal.go`), which covers all three at once and estimates no
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

A coinciding window — every tangent join, apex patch and whole turn — buys back
the SKEW half of the departure and nothing else. DX8's reduction is a claim
about the patch the build ASSEMBLED, not merely about its tag, so it needs BOTH
halves of the departure proven exactly zero: the coinciding window buys back
only the skew half, and the placement's own independent rounding of every
emitted coordinate leaves the other half in place on any placed band. So a
coinciding window alone no longer answers DX8. Only a patch whose own stamped
departure (`capblend_geom.go`'s `f.normalBound`, derived in
`capblend_departure.go`) is an exact zero does — an axis-aligned `Plane` patch
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

### 8.4 Measurements + tessellation

`capBlendPayload` owns analytic patches. Report each exactly representable
result as `Exact`; report every float/transcendental result with a proven bound.

Compute area, signed volume, and first moments by closed-form surface integrals
over each trimmed patch. Parameter domains are line/circle intervals, tube
angles, and spherical normal polygons; all integrands reduce to polynomials and
trigonometric endpoint terms. NEVER use quadrature to claim Exact.

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
float point the build holds. Interval arithmetic is inclusion-monotonic, so the
box holds the denoted point whatever the platform's `sqrt` and `hypot` did, and
nothing in the derivation assumes an ulp contract. Where no bounded box exists
the call is SX14.

The readings that carry it: every cap-level vertex `bound`; every cap-level
edge length — the corner-to-apex slants, a wall's own cap edge, a reflex
corner's connector arc, and a whole circle's circumference — beside that edge's
own evaluation error; and the payload's directional extent, weighted by how much
of the direction lies in the plane. The contour's cap level separately retains
the receiver end's axial displacement, so an axial direction reads that term
even though it reads none of the contour displacement.
`Bounds` reports the same figure as the box's own `Bound`, per candidate: a
contour that loses the extremization contributes nothing, so a plate whose
world-axis extremes are all recorded coordinates still reports an `Exact` box.

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

A band's SIDE level is displaced too, and by a different mechanism, so it is a
separate term with its own helper. `sideZ` is the single float sum
`capZ + matSign*d`, so the whole side directrix translates rigidly by that
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
`k·θS+m·θC` (`|k|+|m| <= 3`), bounded by the SAME structural-envelope
discipline (`|cos|`, `|sin|`, `|sincHalf|` never exceed 1) the volume's own
cross term already uses, with the whole-turn window collapsing to two terms
computed with no trigonometric call at all — the moment's own analogue of the
volume's zero-valued eccentric origin term there. The centroid divides the
summed first moment by the body's own volume and lifts the plane-local
quotient to world through the same frame/placement lift a prism centroid
uses, with the geometric safety-net bound (the true centroid lies within the
body's own `Bounds` box) standing as a `math.Min` ceiling on the formula
answer, never the whole bound.

## 9. Shell reach

### 9.1 `stackedPrismPayload`

General prism shells are a finite axial stack of exact line/arc regions. One
axial slab may contain several disconnected regions:

```go
type prismSlab struct {
    Regions []ProfileRecord
    Z0, Z1  float64 // evaluator coordinates, not public measurements
}

type prismSlabInterface struct {
    Shared       []ProfileRecord // material on both sides; cancelled
    LowerExposed []ProfileRecord // outward normal points toward +Z
    UpperExposed []ProfileRecord // outward normal points toward -Z
}

type stackedPrismPayload struct {
    Slabs      []prismSlab
    Interfaces []prismSlabInterface // exactly len(Slabs)-1
    Frame      r3.Frame
    Xform      r3.Transform
}
```

Slab intervals are ordered, have positive height, and have disjoint interiors.
Consecutive intervals meet at exactly one axial plane. Region interiors within
one slab are pairwise disjoint. Material is the union of every region prism.

This payload is evaluator-private and is built only from the shell cases in
this section. At each shared plane, the shell construction records a certified
partition into coincident material, exposed lower material, and exposed upper
material. Every region on the narrower side is proven contained in its paired
region on the wider side; any relation outside that subset/equality form is
SX8. The payload builder therefore does not hide a general planar Boolean.

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

Existing `cupPayload` migrates to this payload before RX1 side/no-opening shell
lands. The migration changes evaluator storage only, not recipe or public
topology. A cup over a section with `k` holes becomes one floor slab containing
`P` inward or `Q` outward, plus one wall slab containing the outer band and `k`
hole-lining bands as separate regions. Every wall region has positive-area
overlap with the floor region, so the component graph still proves exactly one
lump. The interface partition emits the remaining cavity-floor face once.

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

For cap-only removal from a holed section, build the wall as one slab with
`1 + k` regions: the band between the paired outer loops first, followed by one
band between each paired hole loop in `ProfileRecord` order. The base
S18/S10/S11/§5 gates prove those bands are regular and pairwise disjoint before
payload construction. The resulting `1 + k` connected components lift base S12
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

## 10. Table BX — results + roles

| BX | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BX1** | prism lateral fillet/chamfer | base `prismPayload` | base B1 | base roles |
| **BX2** | revolve junction fillet/chamfer | `revolvePayload` over rewritten meridian | existing full/partial revolve topology | `side(i,j)` + blend role on inserted wall |
| **BX3** | complete prism cap-loop fillet/chamfer | `capBlendPayload` | trimmed cap/sides + analytic blend patches | result side/cap roles; `filletCap(c,l,p)` / `chamferCap(c,l,p)` per patch |
| **BX4** | prism shell with side opening | `stackedPrismPayload` | one connected outer shell under RX scope | `slab(k).region(m).side(i,j)`, exposed cap/rim roles, `shellSide(i,j)` |
| **BX5** | prism `WithNoOpenings` | `stackedPrismPayload` | one lump; outer + void shell | outer/inner/result-slab roles |
| **BX6** | full-revolve shell | `revolvePayload` over wall region | existing full-revolve shell rules | result `side(i,j)` roles |
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
polygons would record one evaluator's decomposition as the meaning of a recipe
whose exact-kernel replay modifies a different B-rep. SX9 is therefore a
permanent limit of this evaluator reach, not an unfinished zero-bound shortcut.

**What SX9 leaves a caller proving.** Because SX9 never lifts, a caller whose
part fillets or chamfers a boolean result cannot reach the modified solid here at
all, and waiting is not one of the options. What replaces it is a proof of the
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
| **DX3** | `Tessellate` / STL / OBJ | waits on revolve tessellator; feature itself still builds | required patch tessellator; staged for the cap-loop chamfer, whose asked reading is `ErrUnsupported` | required slab-region tessellator |
| **DX4** | mesh boolean | available once DX3 exists | available once DX3 exists | available once DX3 exists |
| **DX5** | `ThroughAll` directional extent | existing | analytic patch extrema; a direction whose extreme carries computed cap-contour or inherited axial displacement is `ErrUnsupported`, since a stop reads this coordinate as exact and has no bound to widen (§8.4) | union of slab-region extents |
| **DX6** | clearance | existing revolve boundary reader | add trimmed patch faces to boundary model; undecidable cells stay `Suspect`; staged for the cap-loop chamfer, whose pairs read `Suspect` unless boxes already decide them | union exposed slab faces; never include cancelled interfaces |
| **DX7** | undercut | existing revolve survey | bounded normal ranges per patch, each widened by the whole distance its own `Face.NormalAt` readings can sit from the patch's exactly enclosed normal model and by that patch's own proven departure from the surface it publishes (§8.3), a circular patch's window read through a proven enclosure rather than a float evaluation; a proven opposing point lists its patch, and a remaining straddle is undecided without removing another proven listing | exact normal ranges per exposed face |
| **DX8** | minimum radius | existing meridian survey | minimum concave principal radius over sphere/torus/cylinder/cone patches; undecided unless every patch is proven to be exactly the surface it publishes (zero departure, §8.3) — a mitered ruled patch is one case of that | section arcs + exposed rim geometry |
| **DX9** | minimum wall thickness | existing revolve rewrite survey | staged: asked reading is `Suspect` | staged: asked reading is `Suspect` |

DX9 is a deliberate evaluator limit. A cap blend and a stacked shell are not
one constant section at one height. The existing 2D spanning-disk proof does
not decide them. The modify call still builds an exact solid; only an explicitly
asked wall survey is undecided. No open implementation claim remains.

Clearance may also return undecided for surface cells its certified kernel does
not solve. This is the existing `Verify` contract, not a modify-build refusal.

DX3 and DX6 are staged for the cap-loop chamfer, and §14's rule that a PR may
leave a DX question staged only where this table says so is what these two cells
now say. Both remain required for the cap-loop fillet.

The chamfer's DX3 reading is `ErrUnsupported` through the unknown-payload path.
A patch tessellator must chord the cap-level offset boundary and the side-level
original boundary into one strip, and the two may need different sample
densities; a strip whose densities disagree is not watertight, and a mesh that
is not watertight is a wrong answer rather than a coarse one.

The chamfer's DX6 reading is `Suspect` for any pair its bounding boxes do not
already decide, which is the same staging the cup payload took before its own
clearance model landed. A trimmed patch face admitted into the certified kernel
without a proof of its trim yields a false disjointness certificate, and a false
certificate is worse than an undecided pair: `Verify` would report a clearance
the geometry does not have.

## 13. Required tests

Every implementation PR MUST add geometry assertions, not run-only coverage.

### Options + recipe

- option nil/duplicate/conflict gates;
- `WithNoOpenings` nil-selector exception and non-nil conflict;
- tangent-chain seed cardinality before expansion;
- asymmetric nested selector deep-copy + JSON round trip;
- recipe replay selects same seeds, expansion, reference faces, and result;
- failed call leaves body live and recipe unchanged.

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
- recipe encodes reference + other distance exactly.

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
- opposite cap bands meeting → SX7;
- carrier collapse → SX6, including where the same call ALSO satisfies SX7's
  `reach >= height`, so the sweep height alone can never pick the sentinel;
- non-adjacent patch crossing/touch → SX7/SX12;
- every cap-level edge reports a finite length and a finite bound, and a
  `LongerThan` query no longer matches a slant edge on an infinity;
- a cap-level vertex's bound ENCLOSES its distance to the denoted contour
  point, taken over exact rationals from a section whose offset has a closed
  form (the 12-9-15 right triangle, whose feet are `(t, t)`, `(12 - 3t, t)` and
  `(t, 9 - 2t)` exactly);
- a body whose world-axis extremes are recorded coordinates reports an `Exact`
  `Bounds` box, and one tilted so an axis reads both plane and sweep reports the
  contour's own displacement instead;
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
  and recipe stay untouched;
- every topology edge has exactly two adjacent faces;
- every patch `Face` reports its own area, and no float-computed one is `Exact`;
- an all-`Plane` cap-loop band whose true volume is a float64 reports it `Exact`
  with a zero bound, and a band carrying a `Cone` patch reports `Approximate`;
- the centroid bound encloses the true centroid, tested on a box far wider than
  it is tall — the shape whose farthest corner is neither `Min` nor `Max`;
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
- refusal leaves operands live and recipe unchanged.

## 14. Implementation order

| PR | Lands | Still staged |
|---|---|---|
| **A** | option records/codecs; tangent expansion; asymmetric prism chamfer | revolve/cap/shell reach; all SX9/SX10 |
| **B** | revolve junction rewrite + roles + surveys | cap loops; shell reach; DX3 until revolve tessellation lands |
| **C** | multi-region `stackedPrismPayload`; migrate cups; lift base S12 through BX8; closed + side-opening prism shell; tessellation/clearance cases | cap loops; revolve shell |
| **D** | full/partial allowed revolve shell | cap loops |
| **E** | `capBlendPayload`; complete cap-loop chamfer; analytic integrals | complete cap-loop fillet; DX3 patch tessellation; DX6 clearance model; partial cap chains; mixed edge classes; faceted receivers |

Each PR lands its result payload, structural topology, measurement path, recipe
round trip, and tests together. A PR may leave a DX question staged only where
Table DX explicitly says `Suspect` or `ErrUnsupported`.

No implementation PR changes SX9. Supporting boolean receivers requires a
separate carrier-preserving B-rep design, not another row in this extension.

## Implementation notes

### capblend_contour.go

The cap-loop chamfer's cap contour displacement (§8.4) and
`docs/prism-boolean-design.md` §7's `sectionDelta` are the same idea applied to
two different constructions, and they are independent terms with separate
owners: `capblend_contour.go` derives the cap contour's own displacement, and
the cap-blend build/measurement code (`capblend_geom.go`, `capblend.go`,
`capblend_moments.go`) carries it into every reading that needs it. No
cap-blend reading composes `sectionDelta`, and no `sectionDelta` consumer reads
the cap contour's displacement. `capBlendPayload` separately preserves its
receiver's per-end axial displacement and the selected-end setback rounding.
