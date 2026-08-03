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

Gate order:

| Stage | Gates |
|---|---|
| 1. call | base S17/S15/S13-or-S14; option decode; SX1; seed selector S16 unless no-openings |
| 2. expansion | resolve seed; expand tangent chain; SX2 |
| 3. reference | resolve asymmetric reference; SX3 |
| 4. receiver/target | base R + RX; SX4/SX5/SX8/SX9/SX10 |
| 5. existence | base S4/S5/S18/S10; SX6/SX11 |
| 6. constructed-geometry audit | base S8/S6/S7/S9/S11; SX7/SX12 |
| 7. payload | base S12 until `stackedPrismPayload` lands; BX8 handles that exact case afterward |

The existence-first rule remains load-bearing. SX6 precedes SX7: an empty
offset means no regular rolling-ball surface; intersecting valid patches mean
the surface exists but this evaluator cannot trim it.

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

Adjacent miter patches meet on their common analytic edge and need no extra
vertex face. At axial fraction `s`, every patch intersects the parallel section
as the exact loop offset by `s*dc`. Audit the whole `s ∈ [0,1]` family with
certified interval subdivision over `s` and the two line/arc parameters. A cell
whose distance lower bound is positive is excluded; an exact admitted root is
an intersection; budget exhaustion is undecided. Proven intersection and
undecided both route to SX12. A sample or residual never admits disjointness.

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

Tessellation chords each shared boundary once. For a two-parameter tube patch,
choose parameter counts so the sum of path sagitta and minor-circle sagitta is
`<= tol`. Reuse the same samples on every adjacent patch. Mesh remains
watertight and carries the proven maximum sum.

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
| **DX5** | `ThroughAll` directional extent | existing | analytic patch extrema | union of slab-region extents |
| **DX6** | clearance | existing revolve boundary reader | add trimmed patch faces to boundary model; undecidable cells stay `Suspect`; staged for the cap-loop chamfer, whose pairs read `Suspect` unless boxes already decide them | union exposed slab faces; never include cancelled interfaces |
| **DX7** | undercut | existing revolve survey | exact normal ranges per patch | exact normal ranges per exposed face |
| **DX8** | minimum radius | existing meridian survey | minimum concave principal radius over sphere/torus/cylinder/cone patches | section arcs + exposed rim geometry |
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
- carrier collapse → SX6;
- non-adjacent patch crossing/touch → SX7/SX12;
- every topology edge has exactly two adjacent faces;
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
| **E** | `capBlendPayload`; complete cap-loop fillet/chamfer; analytic integrals + tessellation + clearance model | partial cap chains; mixed edge classes; faceted receivers |

Each PR lands its result payload, structural topology, measurement path, recipe
round trip, and tests together. A PR may leave a DX question staged only where
Table DX explicitly says `Suspect` or `ErrUnsupported`.

No implementation PR changes SX9. Supporting boolean receivers requires a
separate carrier-preserving B-rep design, not another row in this extension.
