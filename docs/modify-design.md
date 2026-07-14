# Modify Design

How the v1 evaluator builds the three modify ops — `Body.Fillet`,
`Body.Chamfer`, `Body.Shell`.

Four tables are normative, and each states its facts **once**:

| Table | States | Section |
|---|---|---|
| **R** — the receiver | which body a modify op accepts, keyed on the **payload class** | §3 |
| **S** — the refusals | every refusal, with the §1 existence test that picks its sentinel, and the order the gates run in | §4 |
| **B** — the result | op × removed faces × sense × section connectivity → payload class, **lump count**, faces and their roles | §9 |
| **D** — downstream | one row per consumer, and whether it reads the payload only or reads roles | §12 |

Every other section cites a row of one of them and restates none of it.
Around the tables: what a modify op owes and how it refuses (§1), the reduction
that makes a straight prism's modify ops exact (§2), where the 2D work is
allowed to live (§5), the fillet (§6), the chamfer (§7), the shell (§8), how
the results stay Exact (§10), the recipe, the provenance and the replay (§11),
the increment plan (§13) and the open questions (§14).

Companion to `docs/api-design.md`, which owns the three signatures, the
selector contract and the retire rule ("core §N"), and to
`docs/evaluator-design.md`, whose §11 row 5 this document is ("evaluator §N").
`docs/verification-design.md` owns how the results are judged
("verification §N"). Nothing here changes those contracts.

## 1. What a modify op owes, and how it refuses

A modify op consumes one body and returns another (core §6). It has exactly
two honest outcomes, and the whole of this document is the line between them:

- **the body the caller asked for, built exactly** — analytic faces, Exact
  measurements, a boundary valid by construction; or
- **an error at the call**, naming which of the two things went wrong.

There is no third outcome. A blend clipped back because the radius did not fit,
an offset whose self-overlap was quietly trimmed away, a corner patched with a
surface nobody can name — each is a body the caller did not ask for, wearing
the Exact measurements of one they did. That is core §1's confidently-wrong
failure, produced by the very operation an agent reaches for last, on the model
it is about to commit to.

**The sentinel follows the intent, and the test is whether the body exists.**
Evaluator §2 stages what the evaluator cannot build; core §12 rejects what
cannot be built at all. The two sentinels are not interchangeable, and one
question separates them:

| The caller asked for | Sentinel |
|---|---|
| a body that **does not exist** — no such solid, at that radius or that thickness, under any evaluator | `ErrDegenerate` |
| a body that **exists** and this evaluator cannot build | `ErrUnsupported` |

Table S (§4) answers that question for every refusal, once, and fixes the order
the gates are asked in — which the answer above depends on: a gate that fires
early on a body that *does* exist would hand the caller `ErrUnsupported` for a
body that does not. No other section picks a sentinel or an order; they cite an
S row.

Both sentinels are returned from the call, before anything is recorded: a
failed evaluation leaves the recipe and the document untouched (evaluator §8),
so a refused modify op retires nothing and the caller still holds a live body.
Neither is ever deferred into a `Verify` reading — `Verify` judges bodies the
document holds, and a refused call produced none. The staging split of
evaluator §11 is exactly this: an intent the evaluator cannot **build** is an
error at the call; only a question it cannot **answer** on a body it did build
reads `Suspect`. This increment has two such questions, both on the cup, and
Table D (§12) names them: its own `MinWallThickness` (D1) and its clearance
against another body — the latter wherever the clearance kernel is invoked on
the pair, a box-disjoint cup pair with no clearance asked staying `Sound` by
the box test (D6).

## 2. The reduction: a prism's modify ops are its section's

A straight prism is a 2D region swept along a straight line. Every face, edge
and vertex of it is the sweep of a feature of the recorded section
(evaluator §5), and the correspondence is total:

| In the section | In the prism |
|---|---|
| a boundary segment | a side face (`Plane` for a `LineSeg`, `Cylinder` for a circular one) |
| a corner between two segments | a **lateral edge** — a `Line3` parallel to the sweep, running cap to cap |
| the region itself | the two cap faces |

**So a modify op that touches only lateral edges and caps is a rewrite of the
section, and the modified body is the prism over the rewritten section.** A
fillet of a lateral edge is the section's corner rounded by an arc of radius
`r`; a chamfer of one is the corner cut off by a chord; an inward shell is the
section eroded by the thickness. Each rewrite maps a line/arc region to a
line/arc region — the offset of a line is a line, of a circle a concentric
circle, and a corner closes with an arc or a miter — so the result is again a
`ProfileRecord` in the increment-1 vocabulary, and the body is again built by
`evalPrism` from it.

A prism's recorded section is line and arc segments and nothing else: a
free-form segment kind does not build as an extrude (evaluator §5), so no
prism carries one, and a modify op needs no gate against a shape that cannot
reach it.

That is the whole design, and everything else follows from it:

- **exactness is inherited, not re-argued.** The mass-property engine
  (evaluator §4) integrates the rewritten region in closed form, so volume,
  area and centroid stay Exact with a zero bound (§10);
- **validity is by construction, exactly as evaluator §10 claims it.** A prism
  over a simple closed line/arc region is watertight and manifold structurally.
  What changes is who proves the region simple: for an extrude it is `sketch`;
  for a modify op it is decad's own audit of its own construction (§5), run
  before anything is built;
- **the ops compose**, and Table R (§3) says so in the only way that cannot
  drift: a filleted prism, a chamfered prism and a tube are all one payload
  class, so what accepts one accepts all three.

**The reduction is the evaluator's, never the recipe's.** The `Step` records
"fillet these edges at 2 mm" — a selector and a radius (core §6.2) — and never
the rewritten section. An exact kernel replays the same step and builds the
blend as a true rolling-ball surface against the true faces; it produces the
same body by a different route, which is precisely what core §2 promises a
second evaluator will do. A recipe that recorded the rewritten section would
have recorded this evaluator's *method* as the caller's *intent*, and vN would
inherit a 2D reduction it does not need.

The one thing the reduction cannot express is a result in **more than one
piece**: a `ProfileRecord` is one outer loop and its holes, so a `prismPayload`
is one connected region swept once. The lump count is therefore a load-bearing
fact of every result, and Table B (§9) carries it as a column.

## 3. Table R — the receiver

The receiver is admitted on **what its payload is**, never on the op history
that produced it. `Body.Placed` re-evaluates a payload under a composed motion
and yields the same class (evaluator §8), so a placed body reads exactly as
the body it was placed from. The receiver must also be **live**: a modify op on
a retired body is S17, by core §6's retire rule.

| R | Receiver payload | `Fillet` / `Chamfer` | `Shell` |
|---|---|---|---|
| **R1** | `prismPayload` — an extrude, a filleted body, a chamfered body, a tube (B2/B3), or any of these `Placed` | **builds**; every selected edge must be lateral (else S1) | **builds**; the removed faces must be caps (else S2) |
| **R2** | `cupPayload` — a one-cap shell (B5/B6) | S3 | S3 |
| **R3** | `revolvePayload` | S3 | S3 |
| **R4** | no payload — a body this evaluator did not build (a `Faceted` boolean output) | S3 | S3 |

A full-circle loop is a single closed wall with **no** lateral edge at all
(evaluator §5 emits no seam), so a cylinder has no edge a fillet can name but
its cap rims — and a query naming one is S1, a query naming nothing is S16.
That is the honest reading of the class, not an oversight: the blend of a cap
edge is the vertex-blend problem (§6), and this increment does not solve it.

## 4. Table S — the refusals

Every refusal in this document, once. The middle column is §1's test; the
sentinel follows from it and from nothing else.

| S | The call asked for | Does that body exist? | Sentinel |
|---|---|---|---|
| **S1** | a fillet or chamfer of an edge that is not a **lateral** edge of the receiver — a cap edge (a `Line3`, `Arc3` or `Circle3` in a cap plane) | yes | `ErrUnsupported` — the vertex-blend problem (§6) |
| **S2** | a shell that removes a **side wall** | yes | `ErrUnsupported` — the cavity is then the offset of an open chain, closed against the removed wall's own surface; a different 2D machine |
| **S3** | a modify op on a receiver whose payload class is not `prismPayload` (R2–R4) | yes | `ErrUnsupported` |
| **S4** | a blend or bevel of a corner whose two segments meet **smoothly** (tangent) or in a **cusp** (anti-tangent) | no — there is no corner to blend | `ErrDegenerate` |
| **S5** | a blend at a corner where **no blend surface of that radius exists**: the two carriers' material-side offsets never meet (parallel lines, concentric circles), or a circular carrier with the material **inside** it has `R − r ≤ 0` — its `r`-offset into the material is empty (`r > R`) or the circle's own centre, whose foot on the arc is not unique (`r = R`) | no | `ErrDegenerate` (§6) |
| **S6** | a cutback that reaches or passes the far end of a walk — including a far end another corner **in the same call** claims | yes — the blend surface is pinned by its own corner's offsets, and merging the two (or trimming each against what it runs into) builds the body | `ErrUnsupported` (§6) |
| **S7** | a rewrite whose loops **cross** — a loop crossing itself, or two loops of the section crossing each other | yes — a resolving kernel trims the pieces against each other | `ErrUnsupported` (§5) |
| **S8** | a rewritten loop that has turned **inside out** — its signed area has changed sign | no — the modification consumed the region | `ErrDegenerate` (§5) |
| **S9** | a rewrite whose loops do not cross but whose **nesting the audit cannot decide** (§5) | this evaluator cannot tell | `ErrUnsupported` — it declines rather than guess |
| **S10** | an **inward** thickness that leaves **no cavity** — at or beyond the section's **inradius**, or, where a cap is **kept** (B5), at or beyond the sweep's **height**, which that cap's floor consumes | no — the cavity is empty; the wall has eaten the part, across the section or along the sweep. The two limits are independent, and an **outward** thickness has neither: a dilation of a non-empty region is never empty, and an outward floor *adds* height below the kept cap instead of eating it. A **both-caps** shell keeps no cap, so it grows no floor and its cavity runs the whole sweep: only the inradius limit can fire on B2/B4 | `ErrDegenerate` (§8) |
| **S11** | a shell whose **exact offset changes the section's feature set** — a loop merged, split or dropped (a slot or a gap narrower than `2t` inward; a hole narrower than `2t` outward), or a **segment** dropped (a circular segment with the material inside it and `R ≤ t` inward: its offset radius `R − t` reaches zero or goes negative, the arc vanishes and its neighbours miter) | yes | `ErrUnsupported` — this evaluator's offset is per-feature and topology-preserving; resolving either needs a trimmed-offset kernel it does not have (§8) |
| **S12** | a **both-caps** shell of a **holed** section — the wall is one band around the outer loop plus one band lining each hole: `1 + k` lumps (B4) | yes | `ErrUnsupported` — a `prismPayload` holds one region, and this evaluator has no multi-lump payload (§9, §14) |
| **S13** | a **zero radius** or a **zero distance** — a body identical to the one the caller already holds | it exists, and it is the receiver: a question with one answer and no content, exactly as `Verify`'s zero tool is (verification §2) | `ErrDegenerate` |
| **S14** | a **zero thickness** shell | no — a face is removed and the wall is `P \ P`: the empty region, no solid at all | `ErrDegenerate` |
| **S15** | a magnitude of the wrong `Kind`, a non-finite one, or a negative one | — | `ErrUnitKind` / `ErrNotFinite` / `ErrNegativeMagnitude` (core §12 names all three by role) |
| **S16** | a selector that matches nothing — loudly (core §9): a query asserting no cardinality, `ErrNoMatch`; a failed `Exactly(n)` / `AtLeast(n)`, `ErrCardinality`, zero matches included (core §12) | — | `ErrNoMatch` / `ErrCardinality` |
| **S17** | a modify op on a retired receiver | — | `ErrRetiredBody` |

Selection happens against the live receiver, before the build, and every gate
in this table runs before a single face is made.

**The order the gates run in.** A refusal is truthful only if the question it
answers could be asked at all, so the gates run in the order their inputs come
into existence — and where two of them can be asked on the *same* inputs, the
**existence** question (§1) is asked first, because a body that does not exist
must never be reported as one this evaluator merely cannot build. The order is
the same for every op:

| Stage | Asks | Gates, in order |
|---|---|---|
| **1 — the pre-gates** | is this a call at all? Decided before any geometry | S17 (a live receiver), S15 (a magnitude of the right `Kind`, finite and non-negative), S13 / S14 (a non-zero one), S16 (a selector that matches) |
| **2 — the receiver and its targets** | is this body one a modify op takes, and is what the query named a thing it can act on? | S3 (Table R's payload class), then S1 (every selected edge is lateral) / S2 (every removed face is a cap) |
| **3 — the construction's own gates** | does the rewrite the caller asked for exist, feature by feature? | fillet / chamfer: S4 (there is a corner), then S5 (a blend of that radius exists — fillet only). Shell: S10 (the cavity is non-empty — inward only: the eroded section, and the height a kept cap's floor leaves), then S11 (no feature the offset would drop) |
| **4 — the §5 audit of the rewritten profile** | do the pieces bound a simple, correctly nested region? | S8 (orientation — the existence question, so a consumed region never reads `ErrUnsupported`), then S6 (no walk consumed by its own corners — an offset mints none, §8), then S7 (no crossing; for a **shell** a crossing is S11, §8), then S9 (nesting, which is decidable only once no two loops cross) |
| **5 — what the result can be held as** | the region is proven; can a payload hold it? | S12 (a both-caps shell of a holed section is `1 + k` lumps) |

Each stage needs the one before it, and that is what fixes the order rather than
taste: there is no cutback to measure until the blend centre exists (S5), no
offset loop to orient until the cavity exists (S10) and keeps its features
(S11), and no lump count to take until the offset bounds a proven region. **S12
is therefore last** — an inward both-caps shell of a holed section at or beyond
the inradius is S10, and one whose offset merges two loops is S11; neither
reaches the count (B4).

Stage 1 applies to every op and to every row of Table B (§9), and so does S3;
S2 is decided by the removed faces each shell row is keyed on. Table B's
Refusals column therefore names what a row's own geometry refuses, in this
order.

## 5. Where the 2D work lives, and what proves it

The section rewrite computes tangent points, offset curves and their crossings —
2D geometry, in a repository whose first hard rule is **never re-derive a 2D
answer**. The line is the one evaluator §4 already draws for the mass
properties, and it is drawn by *whose claim it is*:

- **`sketch` decides the sketch**: what closes, what is valid, where the
  caller's entities cross, what a trim is. decad consumes those answers and
  never recomputes one. That is unchanged and untouched here.
- **decad owns its own records.** A `ProfileRecord` is decad's own geometry
  (seam §2), and the rounded corner, the bevel chord and the offset wall exist
  in **no sketch at all** — no entity was ever drawn there, so there is no
  upstream answer to consume, and reaching back for one is impossible in any
  case: the evaluator evaluates from the record, and a replay holds no sketch
  (evaluator §1). The rewrite is new geometry decad synthesizes from decad's
  own data, in the same class as the boundary integrals of evaluator §4 and the
  inscribed-disk kernel of `survey2d.go`.

**The falsify-only rule is not in tension with the audit, because there is no
upstream claim to bless.** That rule governs admission of what `sketch` hands
over — a `Partial` fragment's `TExact`, which decad may disprove and may never
certify (seam design). The rewritten section is not handed over by anyone; decad
constructed it, so decad owns its validity, and it proves it with **exact,
closed-form** tests rather than a residual.

**The audit is a property of a rewritten profile, not of the op that rewrote
it.** A fillet's rounded section, a chamfer's beveled one and a shell's offset
one are the same kind of thing — a `ProfileRecord` decad synthesized from its own
data — so every one of them passes the audit below before anything is built, and
there is no rewrite that skips it. Where the op that produced the rewrite has a
row of its own for what a test catches, that row is the one the refusal cites;
the shell has one, and §8 says which.

**The rewrite is admitted only when the loops it produces are proven simple and
correctly nested.** Four tests, in the order §4's precedence fixes, all in
closed form over decad's own line and arc segments:

1. **Orientation preserved.** A loop whose signed area has changed sign has
   turned itself inside out — the modification consumed the region — and is
   **S8**. It is asked **first** because it is the audit's one existence
   question (§1), and its answer is defined whatever the pieces do to each
   other: the signed area of a closed chain is a boundary integral, and it is
   there to be read before any piece has been tested against any other. A
   region a rewrite has consumed exists under no evaluator, so it must never
   leave by one of the three staging exits below.
2. **No self-consuming trim.** A segment trimmed past its own other end is
   **S6** — the corners at its two ends have claimed the whole walk between them
   (§6): the pieces the rewrite produced must be resolved against each other
   before they bound anything. It precedes the crossing test because a walk its
   own corners have eaten is not yet a piece an intersection against it would
   mean anything on.
3. **No crossing.** Every pair of segments within a rewritten loop, and every
   pair drawn from two loops of the section, is tested for intersection —
   line×line, line×circle, circle×circle, the same closed forms the clearance
   kernel's 2D reduction and `survey2d.go`'s boundary walks use. A crossing is
   **S7** — and, on a shell's offset, **S11** (§8).
4. **Nesting preserved.** Once no two loops cross, each loop lies wholly inside
   or wholly outside every other, so nesting is decided by classifying **one
   point** of each loop against each other loop. Containment is *not* a crossing
   test and is not free: the classifier is the ray-parity walk with direction
   retries that `survey2d.go` already runs, and it admits an **undecided**
   outcome. A build-time audit has no `Suspect` to fall back on, so an undecided
   containment is **S9** — the evaluator declines. The audit passes only when the
   outer loop is proven to contain each hole and the holes are proven mutually
   disjoint.

A residual proves nothing and admits nothing; a crossing or containment test on
exactly represented line and arc segments is a **decided** fact of decad's own
data, and its verdict is the same under every evaluator. It is what makes the
build's "valid by construction" claim (evaluator §10) survive a modify op: the
region is proven simple before the prism over it is built, so no unproven body
is ever made, and `Verify` reads the result exactly as it reads an extrude's.

## 6. Fillet

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
```

`r` is a magnitude, gated at the call like every other (S15), and a zero `r` is
S13.

The rolling-ball blend of a lateral edge is a **cylinder**: the ball of radius
`r` rolling in the corner sweeps its center along a straight line parallel to
the sweep direction, and the surface it envelops is the cylinder of radius `r`
about that line, trimmed to the sweep interval. In the section, that cylinder is
one arc.

**The blend center is pinned by the material, not by the edge's convexity flag.**
Let a corner join walk `A` (arriving) to walk `B` (leaving) at the corner point.
The center is the point at distance `r` from both carriers, **on the material
side of each**, and each carrier's material-side offset is a curve of the same
kind:

| Carrier | Its `r`-offset into the material |
|---|---|
| a line | the parallel line, `r` into the material |
| a circle of radius `R`, material inside it | the concentric circle of radius `R − r` (`R − r ≤ 0` is S5) |
| a circle of radius `R`, material outside it | the concentric circle of radius `R + r` |

Which side the material is on is a **decided** fact of the record — the loop's
winding and the segment's own sense — never a guess. The center is the
intersection of the two offset carriers nearest the corner; the tangent points
are its feet on the two carriers. Line×line, line×circle and circle×circle are
closed form with at most two roots, and the root is chosen by the corner it
belongs to, not by proximity to a sample. No intersection means no blend of that
radius exists (S5), and a corner whose two carriers meet **smoothly** or in a
**cusp** is no corner at all — S4, decided before any of this is computed.

`Edge.IsConvex` is therefore not an input to the construction at all; it is what
the caller *selected* with (`Convex()` / `Concave()`, core §9). A convex corner
and a concave corner take the same construction, and differ only in which way
the arc winds — the convex corner rounds material off, the concave one fills
material in, and both are exact.

**The cutback gate refuses; it never clips.** The blend consumes an arc length
`aA` of `A` and `aB` of `B`, measured along each carrier from the corner. Each
must fit **strictly** inside the length that walk still has, after the
modification at its *other* end has taken its own cutback — two adjacent
corners of a short wall are filleted in the same call, and they claim the wall
from both ends. A cutback that reaches or passes the far end is S6, and S6's
existence answer is what makes it the staging sentinel: the blend cylinder
itself is unharmed — it is pinned by its own corner's offsets, and the wall's
*length* never entered its equations. What this evaluator will not do is clip
the blend, slide it, or shrink `r` to fit: those are all the same failure — a
body the caller did not ask for, with a radius they did not name.

**The rewrite.** `A` is trimmed to its tangent foot, `B` is trimmed to its own,
and an `ArcSeg` of radius `r` about the center joins them, wound so the loop
keeps its orientation. The result goes through the §5 audit and then through
`evalPrism`, which produces, with no special case:

| Entity | What it is |
|---|---|
| the blend face | `Cylinder{Origin: the center lifted to the section plane, Axis: the sweep direction, Radius: r}`, trimmed to the sweep interval |
| its two tangent edges | `Line3`, each shared with a trimmed side face — a **smooth** junction: the surfaces meet tangentially |
| its two cap edges | `Arc3`, one per cap, each shared with the cap face |
| the old lateral edge | gone; the old corner vertices are replaced by the arcs' endpoints |

Its roles are B1's.

**The prism build reads the walk, and the fillet depends on that — it does not
introduce it.** Two readings there are taken from the **walk's own sense**, never
from the loop's **role** (an outer loop, or a hole):

- a circular wall's **face orientation** — a circular walk that runs clockwise
  in the plane frame has its material *outside* the cylinder, so the face is
  reversed; counter-clockwise, inside, and it is not;
- **`Edge.IsConvex`**, the walked-boundary convexity a `Convex()` / `Concave()`
  query filters on (core §9) — a **lateral** edge, where two walls meet, reads
  the turn the walk makes there; a circular wall's **rim** edges read that wall's
  own sense, so a clockwise round is concave whatever loop carries it.

The role would be a wrong proxy for both wherever an **outer loop carries a
clockwise circular walk**, and that is not a shape only a fillet can make: the
seam records an arc's walk sense in the segment's own range (`TStart` > `TEnd`
says the walk runs against the curve, `seam.go`), so a plain sketch produces one —
a plate with a semicircular bite taken out of one edge walks that arc clockwise
on its outer loop. The rules are therefore the prism build's own, and this
increment inherits them rather than shipping them: a concave round — a clockwise
circular walk on an outer loop — is the first thing a fillet emits, so the
walk-sense rules are its prerequisite, not its deliverable (§13).

**The corner problem is excluded, not fudged.** Where two blends meet at a
shared vertex — a lateral edge's blend running into a cap edge's blend — the
rolling ball has no single surface to sweep: the two cylinders meet in a curve
that is not on either, and the patch that closes the corner (a vertex blend, a
setback) is neither a cylinder nor a sphere in general. It is not in the shipped
surface set, it is not exactly derivable from the record, and an evaluator that
invented one would be guessing at the very corner an agent asked it to check.
So the class is drawn where the problem does not arise: **a lateral edge's blend
terminates on the two caps, and two lateral blends never share a vertex** —
distinct lateral edges are disjoint, and each blend runs cap to cap, so there is
no vertex at which two of them meet and no patch that would close one. A cap
edge is S1, and the vertex blend stays an open question (§14), not a surface
this evaluator makes up.

Two lateral blends can still **interfere**, and interference is refused, never
patched. Two corners of one wall claim it from both ends: S6. Two corners that
share no wall at all — opposite ends of a thin neck, two corners of one loop
that are not adjacent, a corner of the outer loop and a corner of a hole across
a thin section — can have their rewritten pieces cross without either
overrunning a walk, and the §5 audit catches those: S7. Both fire before a face
is made, and neither produces a corner needing a surface nobody can name.

`FilletOpts` carries nothing in this increment, so a fillet `Step`'s `Opts` is
nil (core §6.2: nil when the op takes none). A variable-radius or setback option
lands in it, with the struct, when it ships.

## 7. Chamfer

```go
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
```

The same edge class, the same gates, one simpler surface. **The bevel of a
lateral edge is a plane**: in the section, the corner is cut off by a straight
chord, and the chord swept along the sweep direction is a planar face.

**The setback is measured along the adjacent boundary curve**, `d` from the
corner along each — a length along a `LineSeg`, an **arc length** along a
circular one. On a straight prism that is exactly the geodesic setback from the
edge across the adjacent face, because the boundary curve *is* that face's
cross-section: the two readings coincide, so the definition is unambiguous
rather than merely convenient. Equal distance both ways is the whole of v1: an
asymmetric chamfer — two distances, or a distance and an angle — is an option
that has not shipped, so it is not an option a caller can pass, and nothing is
silently narrowed (core §8.1: an option that cannot be recorded does not ship).

The rewrite trims both walks back by `d` and joins the feet with a `LineSeg`.
The gates are the fillet's, in §4's order: S15 for a magnitude that is not a
valid length and S13 for a zero `d`; S1 for a cap edge; S4 for a smooth or
cusped corner; then the §5 audit — S8, S6 for a setback that reaches or passes
the far end of a walk, S7, S9. A corner with a circular neighbour
builds: the chord from a point on a line to a point on an arc, or from arc to
arc, is still a `LineSeg`, and the bevel face is still a `Plane` — a chamfer
against a cylindrical wall meets it in a straight ruling, because both are
parallel to the sweep. (S5 has no chamfer case: a chord exists between any two
distinct feet.)

A convex corner's chamfer cuts material away; a concave corner's fills material
in. Both build, from the same construction, for the same reason the fillet's
two cases do. The result is B1.

`ChamferOpts` carries nothing this increment, so a chamfer `Step`'s `Opts` is
nil.

## 8. Shell

```go
func (b *Body) Shell(sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
```

`sel` names the faces to **remove** — the openings. What remains of the solid is
a wall of the given thickness behind every face that was *kept*. On a prism, with
the removed faces restricted to caps (S2), the whole construction is the
section's own offset.

**Inward is the default sense, and outward is enumerated, never signed.** The
thickness is a magnitude, so it carries no sign (core §8.1's rule, applied
here): the sense is a `ShellSense` — `Inward` (the wall grows into the original
solid; the outer skin does not move) or `Outward` (the wall grows off it; the
original solid becomes the cavity) — set by `WithShellSense`, recorded in
`ShellOpts`, and defaulting to `Inward`, which is what "shell this box" means
everywhere it is said. `ShellOpts` is the one `StepOpts` variant this increment
fills, and its `Sense` encodes as a named text token, exactly as `Direction`
does. The thickness passes the magnitude gates before either question below is
asked: a wrong `Kind`, a non-finite or a negative one is S15, and a zero one is
S14.

**The section offset is exact, and it is closed in the recorded vocabulary.**
Write `P` for the section. The inward offset (the erosion) `P ⊖ t` is bounded by:

| Feature of `P` | Its offset |
|---|---|
| a line segment | the parallel line, `t` into the material |
| a circular segment, material inside | the concentric circle of radius `R − t` (`R ≤ t` drops the segment: S11) |
| a circular segment, material outside — a hole wall, a concave round | the concentric circle of radius `R + t` |
| a **convex** corner | a miter: the two offset curves meet, and the corner stays sharp |
| a **reflex** corner | an **arc of radius `t` centered on the corner point** — the nearest boundary feature there is the corner itself, so the erosion's boundary is at distance exactly `t` from it |

Every piece is a line or an arc, so `P ⊖ t` is a `ProfileRecord`. The outward
offset `P ⊕ t` is the same table with the two corner rules exchanged (a convex
corner rounds, a reflex one miters) and the radii moved the other way. Both are
exact.

**Three gates, and they are different questions — asked in that order (§4),
because each needs the one before it to have passed.**

- **Does the body exist?** This is the inward sense's question, and the **cavity**
  answers it. The cavity is a region swept along an interval, so it is empty when
  either of them is — and the thickness can empty either one, which is why S10
  carries **two independent limits**:
  - the **section** limit: `P ⊖ t` is non-empty exactly when `t` is strictly less
    than the section's **inradius** — the radius of its largest inscribed disk,
    which `survey2d.go` already computes **exactly** as part of the wall survey.
    The reading that refuses is the same one that answers `MinWallThickness`;
  - the **height** limit: the wall behind a **kept** cap is a floor `t` thick, so
    the cavity is swept over `[z0 + t, z1]` (B5) and is non-empty exactly when `t`
    is strictly less than the sweep's height `h`. A wide, shallow section clears
    the inradius at a thickness that still eats its whole depth, and no test on
    the offset section would ever see that: `P ⊖ t` is there, and perfectly
    valid — it is the interval under it that has gone.

  Either limit, reached or passed, is S10. The **height** one cannot fire on a
  **both-caps** shell: it keeps no cap, grows no floor, and sweeps its cavity over
  the whole of `[z0, z1]` (B2/B4), so the section limit is the only one that
  reaches those rows. Nothing below can be asked until this gate passes: there is
  no offset section to inspect until the offset section is there, and no wall to
  build around a cavity that has no room to exist.
- **Can this evaluator build the offset?** An offset that changes the section's
  feature set is S11 — a segment the offset would drop, caught as the offset is
  constructed, and a loop it would merge or split, caught by the audit below.
  Staged, not denied: refusing costs the caller a shell decad could in principle
  build; producing one costs them a part that is wrong where they cannot see it —
  the same principle evaluator §12 states for the tapered extrude.
- **Can a payload hold the result?** Only now — with an offset that exists and
  bounds a proven region with `P`'s own feature set — is the wall's **lump count**
  a question with an answer, and a result in more than one piece is S12 (B4).
  Staged for the same reason, and **last** for this one.

**The offset section is a rewrite, so it faces the §5 audit like any other**, and
the audit runs between the second gate and the third. Two of its refusals are the
shell's exactly as they stand: an offset loop that has turned inside out is
**S8**, and an offset whose nesting the containment classifier cannot decide is
**S9**, which the evaluator declines rather than guess. The crossing test is the
one the shell claims for itself: **a crossing of offset loops is S11, not S7**,
because a merge is the expected outcome of an offset — two walls closing on each
other at `2t` *is* the feature-set change S11 names — so the shell's own row owns
the event and S7 never fires on an offset. The trim test cannot fire at all: it
tests a cutback, and an offset mints none — a segment the offset consumes is a
dropped feature, which is S11 again.

**Which section is the outside, and which is the cavity, is what the sense
decides.** Inward, the outer boundary is `P` and the cavity is `P ⊖ t`; outward,
the outer boundary is `P ⊕ t` and the cavity is `P`, because the wall grew off
the original solid and the original solid is what it now encloses. The bodies
that come out — their payload class, their **lump count**, their faces and their
roles — are Table B (§9), and this section states none of it a second time.

**A shelled body has no void, and `Shell.IsVoid()` is false on it.** An opening
is what a removed face *is*, so the cavity's skin reaches the outside through
the rim, and the inner and outer skins are one connected shell — a cup and a
tube alike. A hollow **closed** body — the one shape whose inner skin is a
genuine void shell — would be a shell that removes *no* face, and the selector
vocabulary cannot ask for it at all: S16 makes an empty match an error, and a
cardinality assertion takes a positive count. It is **unspellable**, not
staged: there is no call to refuse, and how it should be spelled is an open
question (§14).

## 9. Table B — the result

One row per (op × removed faces × sense × section connectivity). `k` is the
number of holes in the receiver's section `P`; hole-free means `k = 0`. `[z0, z1]`
is the receiver's sweep interval, and the removed cap is taken as the top
(`z1`) — a removed bottom is its mirror. `Q` is the offset section: `P ⊖ t`
inward, `P ⊕ t` outward.

| B | Op | Removed | Sense | Section | Payload | Lumps | Faces (roles) | Refusals |
|---|---|---|---|---|---|---|---|---|
| **B1** | `Fillet` / `Chamfer` | — | — | any (`k ≥ 0`) | `prismPayload` over the **rewritten** section, same frame, same `[z0, z1]` | **1** | side walls `side(i,j)` over the rewritten record, two caps `capStart` / `capEnd`. The blend cylinder / bevel plane **is** one of those walls, and carries a **second** role `fillet(i,j)` / `chamfer(i,j)` naming the same `(loop, segment)` of the rewritten record | S1, S4, S5 (**a fillet only** — S5 is a condition on the two carriers' `r`-offsets, which only the blend computes; a chamfer's chord exists between any two distinct feet, §7), then the §5 audit: S8, S6, S7, S9 |
| **B2** | `Shell` | both caps | `Inward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: P, Holes: [Q]}`, on `[z0, z1]` | **1** | outer walls `side(0,j)`, cavity walls `side(1,j)`, and the two **rim annuli** — the caps of that prism — `capStart` / `capEnd` | S10 (its **section** limit only — no cap is kept, so no floor eats the sweep), S11, then the §5 audit: S8, S9 |
| **B3** | `Shell` | both caps | `Outward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: Q, Holes: [P]}`, on `[z0, z1]` — no cap is kept, so no material is added along the sweep | **1** | as B2 | S11, then the §5 audit: S8, S9 (no S10 — an outward thickness has no limit) |
| **B4** | `Shell` | both caps | either | holed (`k ≥ 1`) | — | **1 + k** — a band around the outer loop, plus one band lining each hole, pairwise disjoint | — | S10 (**`Inward` only**, and its **section** limit only — B2's reason), S11, the §5 audit's S8 and S9 — every one of them decided on the offset section, and so reached before the count is — then, and only then, **S12** |
| **B5** | `Shell` | one cap | `Inward` | any (`k ≥ 0`) | a **cup**: `cupPayload` — the outer prism over `P` on `[z0, z1]` and the cavity prism over `Q = P ⊖ t` on `[z0 + t, z1]`, an interval S10's **height** limit is what proves non-empty. The kept cap does not move; the floor is `t` of the original material | **1** — every wall band hangs off the floor slab | outer walls `side(i,j)`, the kept cap `capStart`, the **rims** `rim(i)` — the removed cap's plane trimmed to the band between loop `i` of `P` and loop `i` of `Q`, one face per loop (`1 + k` of them) — cavity walls `shellSide(i,j)`, cavity cap `shellCap` | S10 (**both** its limits — this is the one row whose floor eats the sweep), S11, then the §5 audit: S8, S9 (no S12 — one cap is kept, and every band hangs off the floor it leaves) |
| **B6** | `Shell` | one cap | `Outward` | any (`k ≥ 0`) | a **cup**: `cupPayload` — the outer prism over `Q = P ⊕ t` on `[z0 − t, z1]` and the cavity prism over `P` on `[z0, z1]`. The original solid *is* the cavity; the floor is `t` of new material below the kept cap | **1** | as B5 | S11, then the §5 audit: S8, S9 (no S10, no S12, for B3's and B5's reasons) |

**The Refusals column names what a row's own geometry refuses, in the order §4
fixes** — so a row is read left to right, and the first gate that fires is the
one the caller sees. Two groups are decided before a row is reached at all and
are therefore not repeated in it: §4's stage-1 pre-gates — S13 / S14 (a zero
magnitude), S15 (an invalid one), S16 (a selector that matches nothing) and S17
(a retired receiver) — and the receiver gate S3, which Table R (§3) owns. S2 is
likewise absent from every shell row, because the removed faces are what each
row is **keyed on**: a call that removes a side wall has left S2 before any row
claims it. S1 has no such key to hide behind — B1 is keyed on the op, not on the
edge class — so it opens B1's cell.

Every role above indexes the record of the payload **the result holds** — never
the receiver's (§11). B2/B3's tube is a `prismPayload`, which is why Table R
admits it as a receiver in R1 and why nothing downstream needs a new case for
it (§12).

`cupPayload` is the one new payload this increment introduces: two
co-directional prisms over the same plane — the outer region on its interval,
the cavity region on its own — plus the accumulated rigid placement, which is
what `Body.Placed` re-evaluates (evaluator §8) and what every measurement reads
(§10). Every edge of B1–B3, B5 and B6 bounds exactly two faces, so each body is
manifold and watertight by the same structural argument the prism enjoys
(evaluator §10), on regions the §5 audit has already proven simple.

## 10. Mass properties, and why they stay Exact

**A modify op introduces no bound.** Its result is one prism, or two, over
regions the mass-property engine (evaluator §4) integrates in closed form —
`LineSeg`, `CircleSeg` and `ArcSeg` walks are exactly the kinds it already
integrates, and the arcs a fillet and an offset add are those kinds. So every
quantity is `Exact` with a zero `Bound`, and the verification gate passes it at
any tolerance (verification §5). Write `A_X` for the area of region `X`, and
`h` for the receiver's own sweep length, the magnitude of `z1 − z0`. Every prism
then takes **the length of its own interval** — a cup is two prisms, and they
are not the same height:

| Prism | Its interval | Its height |
|---|---|---|
| the receiver, and B1's / B2's / B3's single prism | `[z0, z1]` | `h` |
| a cup's **outer** prism — B5 over `P` on `[z0, z1]`, B6 over `Q` on `[z0 − t, z1]` | its own | `h_o` = `h` inward, **`h + t`** outward — an outward floor is `t` of new material below the kept cap |
| a cup's **cavity** prism — B5 over `Q` on `[z0 + t, z1]`, B6 over `P` on `[z0, z1]` | its own | `h_c` = **`h − t`** inward — the kept cap's floor takes `t` off it, and S10 is what proves the remainder positive — and `h` outward |

| Quantity | B1 — a filleted / chamfered prism | B2 / B3 — a tube | B5 / B6 — a cup |
|---|---|---|---|
| `Volume` | `A · h` on the rewritten section | `(A_outer − A_cavity) · h` — the tube's section is the outer loop less its holes, which is what the engine integrates, and both loops are swept over the one interval | `A_outer · h_o − A_cavity · h_c` — the outer prism less the cavity prism, **each on its own interval**: inward `A_P · h − A_Q · (h − t)`, outward `A_Q · (h + t) − A_P · h` |
| `Area` | caps + Σ (segment length · h); an arc's length is `rθ`, exact | rim annuli + Σ (segment length · h) over both loops | Σ (outer segment length · `h_o`) + Σ (cavity segment length · `h_c`) — each wall band over the interval of the prism it belongs to — plus the kept cap (`A_outer`), the rim bands (`A_outer − A_cavity` in total: the removed cap's plane, less the opening) and the cavity cap (`A_cavity`) |
| `Centroid` | the rewritten region's centroid, lifted to the interval's signed midpoint | the section's centroid (holes subtract, as the engine already does), lifted likewise | each region's centroid lifted to the midpoint of **its own** interval, the two combined with the cavity's mass subtracted: `(A_outer · h_o · c_outer − A_cavity · h_c · c_cavity) / (A_outer · h_o − A_cavity · h_c)` |
| `Bounds` | per-segment analytic extremes over the interval | the same | the outer prism's — in both senses the cavity lies within it, `Q ⊂ P` on the shorter interval inward, `P ⊂ Q` on the shorter one outward |

Each is a difference or a sum of quantities the engine already produces exactly;
none is sampled, and none is fitted. The `Exactness` a modify op reports is
therefore the one it inherited, and there is no path by which a fillet of an
Exact body yields an Approximate one.

**The rewritten section is the body's truth, in exactly the sense a recorded one
is.** A tangent foot is the root of a closed-form equation, computed once, in
floating point — as is every coordinate the seam records and every vertex an
extrude places. `Exact` means the number is the analytic answer and no
approximation was made of the *shape*; it does not mean the arithmetic was
performed in infinite precision, which is a property no evaluator has and which
verification §4 already accounts for, in the only place it can be accounted for:
the coordinates.

## 11. The recipe, provenance, and replay

**The `Step`.** Each op appends one step (core §6.2), and each depends on and
**consumes** the receiver — the receiver is retired from the document and the
result registered, by the uniform rule of core §6:

| Field | Fillet | Chamfer | Shell |
|---|---|---|---|
| `Op` | `OpFillet` | `OpChamfer` | `OpShell` |
| `Inputs` | `[the receiver's StepRef]` | same | same |
| `Selectors` | `[the edge query]` | `[the edge query]` | `[the face query]` |
| `Values` | `[r]` | `[d]` | `[thickness]` |
| `Opts` | nil | nil | `ShellOpts{Sense}` |

Everything else — `Profile`, `Plane`, `Extent`, `Angular`, `Axis`, `Placement`
— is absent, and the wire codec omits it, exactly as the shipped `Step` codec
omits the fields an op does not key.

**The selector is recorded unresolved, and deep-copied.** The query is a value
(core §9), and the step stores a **clone** of it — the same discipline
`extent.go` and `selector.go` already keep: no caller-owned pointer survives into
a recorded step, and none escapes `Recipe()`. The step never records the edges
or faces the query resolved to; that would be the topology index invariant #3
forbids, one level down.

**Replay is deterministic because resolution is.** Selector resolution is a
filter over `Body.Edges()` / `Body.Faces()` in the body's own deterministic
order (evaluator §7), the body being resolved against is itself rebuilt from its
own step, and every gate in §3–§9 is a closed-form test on that geometry. A
replay therefore selects the same edges, computes the same tangent feet, and
builds the same body — which is what makes a recipe the deliverable core §2 says
it is.

**A role is an index into the record, so the result's roles index the result's
record.** `FeatureRef` is a producing `StepRef` plus a role, and the role of a
side face is `side(i, j)` — loop `i`, segment `j` **of the payload the body
holds** (evaluator §3). A modify op rewrites the section: segments are trimmed,
inserted and renumbered, so a role inherited from the receiver would name a
segment of a record this body no longer has. This increment therefore does what
the shipped evaluator already does when it re-evaluates a payload: **every face
of the result carries roles of the modify step alone**, in the result's own index
space, and Table B lists them. There is no re-parenting problem because there is
no inheritance: a role is minted from the record it labels.

Two consequences, both load-bearing, and neither is a workaround:

- **the role-keyed consumers keep working.** The undercut survey and the
  tessellator map a payload walk to the face built from it by looking the role
  up on the body itself — the payload's `(i, j)` against the body's own roles
  (Table D). Because both sides are the result's index space, the lookup hits on
  a filleted, chamfered or tube-shelled body exactly as it does on an extrude,
  with no new code and no new undecided case. An inherited role would have
  broken precisely this: it names the *receiver's* step, and the same role
  *string* would then label two different segments.
- **`FaceCreatedBy` of an earlier step selects nothing on a modified body.** A
  `FeatureRef` is a step **and** a role, so `FaceCreatedBy(FeatureRef{Step: the
  fillet's step, Role: "fillet(0,3)"})` names that blend face, and the extrude's
  refs name faces of the extrude's body — which the modify op consumed. Selecting
  the walls of a modified body is done on what they **are** (`Planar()`,
  `Cylindrical()`, `NormalTo`, `ParallelTo`) or on the modify step's own roles.
  Whether a consuming op should additionally carry its ancestor's refs is core
  §9's question, and it is open (§14).

The body's own `Origin()` is the modify step, role `"body"`. Roles derive from
the record and the deterministic walk order, so a replay reproduces every one of
them (evaluator §3).

## 12. Table D — downstream

One row per consumer. The **Reads** column is the whole point: a payload-only
consumer cannot notice a modify op, and a role-keyed one can only work because
§11 puts the roles in the payload's own index space. A consumer that instead
switches on the payload **kind** — the tessellator, the clearance kernel —
notices a new payload class only once it grows a case for it, and stages it
until then; a consumer that reads a body's live **topology** or a selected
**face** — the body-relative stops' `ToFace` — sees any body regardless of its
payload class.

| D | Consumer | Reads | B1 — filleted / chamfered | B2 / B3 — a tube | B5 / B6 — a cup |
|---|---|---|---|---|---|
| **D1** | `prismWall` + `survey2d` (`MinWallThickness`) | the payload only | works unchanged: the rewritten section is a section, the height is the receiver's, the reading is Exact | works unchanged: a tube **is** a prism over an annular section | **undecided.** The kernel decides one section swept at one height; a cup is two sections over adjoining intervals, so the survey does not decide it and the reading is `Suspect` — §1's sanctioned path, never a silent pass (§14) |
| **D2** | `prismUndercuts` | the payload **and the roles** — it looks each payload walk's face up by `side(i,j)` on the body's own step | works unchanged (§11): every wall of the result, blends included, carries its `side(i,j)` role in the result's index space | works unchanged | a cup reading lands with the cup payload: the same per-face exact normal ranges over the faces of B5/B6, mapped by their roles |
| **D3** | `prismMinRadius` | the payload only | works unchanged: a fillet of a **concave** edge is a concave arc of the section, and its radius is read; a fillet of a convex edge adds a convex cylinder, which is not a concave feature and rightly does not appear | works unchanged: the cavity loop's walls are read like any hole wall | a cup reading lands with the payload: the same walk over the outer and the cavity section. The sharp concave edge where the wall meets the floor carries no radius — the survey reads faces' principal radii, and a spec about the *edges* is one no option states (verification §2) |
| **D4** | `Tessellate` → `STL` / `OBJ` | the payload **and the roles** — it chords the payload's curves and names the face each belongs to by role | works unchanged: blend cylinders chord through the same per-curve machinery, shared by every face meeting the curve, so the mesh stays watertight by construction | works unchanged | extends with the cup payload, in the shell's own increment: the faces are the same analytic variants, and a rim band is a polygon-with-holes the shipped cap triangulator already builds |
| **D5** | Body-relative stops (`ToFace` / `ToFaceAngular`; `ThroughAll` / `ThroughAllSide`) | two stop kinds read differently: `ToFace` / `ToFaceAngular` read **topology + a selector + a surface** — a live stop body, its face resolved by the selector, and that face's plane; `ThroughAll` / `ThroughAllSide` read **the payload's directional extent** (`extentAlong`) | works unchanged | works unchanged | `ToFace` reads the cup's faces like any body's; `ThroughAll` reads its outer prism's extent — the cup's own `extentAlong` — the cavity being interior |
| **D6** | Clearance (`docs/clearance-design.md`) | **the payload and the topology** — it builds its boundary model by switching on the payload **kind** (`prismPayload` / `revolvePayload`), then reads the exact edges, vertices and shells and each body's payload extent | a first-class operand — a `prismPayload`, which its switch builds | a first-class operand — a tube is a `prismPayload` too | **staged.** The kernel's payload switch has no `cupPayload` case, so a cup has no boundary model and the kernel returns undecided on any pair with it. But the pair partition proves box-disjoint pairs cheaply first and reaches for the kernel only where it is needed — a pair the box test cannot settle, or a `WithClearances` gap request — so a cup pair reads `Suspect` exactly there, until clearance learns the payload (§14), never a silent pass. A box-disjoint cup pair with no clearance asked stays `Sound`, proven by the box test with the kernel never invoked |
| **D7** | The mesh boolean (evaluator §9) | the tessellation | takes these bodies as it takes any other | as any other | as any other, once D4 covers it |
| **D8** | `Verify` — the structural audit and the tolerance gate | the topology and the measurements | `Sound` on the same terms an extrude's body is: valid by construction (§5), Exact at any tolerance (§10) | the same | the same, with two `Suspect` exceptions on a cup: its `MinWallThickness` (D1) and its clearance against another body wherever the clearance kernel is invoked on the pair — a `WithClearances` gap request, or a pair the box test cannot settle (D6); a box-disjoint cup pair with no clearance asked stays `Sound` |

Two readings verification §6 asks about are worth stating because a modify op is
what makes them arise, and neither needs §6 relaxed:

- a **chamfer** meets its neighbours at dihedrals of `π` minus the bevel's half
  turn — far beyond any legal draft allowance on a right-angled edge — so it
  reads as an *edge, not a wall*, which is precisely what verification §6 says a
  chamfer is;
- a **fillet**'s junctions are tangent, and the material's interior angle there
  is `π`: a smooth continuation, not the closing wedge of verification §6's knife
  edge (whose interior dihedral goes to *zero*). So a fillet mints no spurious
  zero wall.

## 13. Increments

PR-level staging inside evaluator increment 5. Everything not yet landed is
`ErrUnsupported` **at the call** — never a `Verify` reading, and never a body:

| PR | Lands | Still `ErrUnsupported` after it |
|---|---|---|
| 1 | the section rewrite and its §5 audit, `Fillet` on lateral edges (line/line, line/arc, arc/arc corners), B1's roles, the `Step` wiring | `Chamfer`, `Shell` (S3 for their receivers is unchanged); every cap edge (S1); every non-prism receiver (S3) |
| 2 | `Chamfer`, equal distance | `Shell`; the asymmetric chamfer (it is not spellable — no option carries it) |
| 3 | `Shell`: cap removal, the exact erosion and dilation, the §5 topology gates, the tube (B2/B3), the `cupPayload` (B5/B6), and D2/D3/D4 extended to the cup | side-wall removal (S2); the topology-changing offset (S11); the both-caps shell of a holed section (S12) |

After PR 3, two asked questions on a body this evaluator builds are still
undecided, both on the cup: its `MinWallThickness`, which reads `Suspect` (D1),
and its clearance against another body, which reads `Suspect` wherever the
clearance kernel is invoked on the pair — the kernel has no `cupPayload` case
(D6). A box-disjoint cup pair with no clearance asked is still proven `Sound`
by the box test.

The walk-sense rules of §6 — a circular wall's face orientation and the
walked-boundary `Edge.IsConvex` — are a **prerequisite**, not a deliverable. They
belong to the prism build, which reads them on profiles a fillet has nothing to
do with; PR 1 builds on them and carries none of them.

## 14. Open questions

- **The vertex blend.** A cap-edge fillet (S1), and therefore any blend chain
  that turns a corner, needs a surface for the vertex where three blends meet.
  It is not a cylinder, not a sphere, and not exactly derivable from the section
  — which is why §6 excludes it rather than approximating it. What it should be
  (a setback patch? a spherical corner where the three radii agree, and
  `ErrUnsupported` where they do not?) is undecided, and it is the single
  largest thing standing between this increment and general edge chains.
- **Blend merging.** Two corners of one short wall, filleted in one call so that
  their cutbacks overlap, is S6: the two blend surfaces exist, and a kernel that
  merges them into one — or trims each against what it runs into — builds the
  body. Whether this evaluator should grow that resolution step, or whether a
  caller who asks for it is better served by the refusal, is undecided.
- **The topology-changing offset.** S11 — a shell whose exact offset merges two
  loops (a slot narrower than twice the wall) or drops a segment. Resolving it
  needs a trimmed-offset kernel — the same machinery the tapered extrude wants
  (evaluator §12), which suggests one kernel, one increment, and both callers.
- **The multi-lump result.** S12 — a both-caps shell of a holed section is
  `1 + k` disjoint bands (B4), and no payload in this evaluator holds a
  disconnected region: a `ProfileRecord` is one outer loop and its holes, and a
  `prismPayload` holds one. A multi-region payload (a list of `ProfileRecord`s,
  one lump each, sharing a frame and an interval) would build it, and would also
  be what a boolean that splits a body needs (evaluator §9). Whether that payload
  is worth its cost, and which increment owns it, is undecided; until it exists,
  the refusal stands.
- **A cup's wall reading.** D1 — the shipped 2D kernel decides one section swept
  at one height. A cup is a stack of two sections over adjoining intervals (a
  floor slab, then a wall band), and its spanning balls are not the balls of
  either section alone. Extending the kernel to a stacked section is the missing
  piece; until it lands, the question reads `Suspect`.
- **Clearance on a cup.** D6 — the clearance kernel
  (`docs/clearance-design.md`) builds its boundary model by switching on the
  payload kind, and it has no `cupPayload` case, so the kernel returns undecided
  on any pair involving a cup. The kernel is invoked on a pair only where it is
  needed — a pair the box-disjointness test cannot settle, or a `WithClearances`
  gap request — so a cup pair reads `Suspect` exactly there; a box-disjoint cup
  pair with no clearance asked stays `Sound`, proven by the cheap box test with
  the kernel never invoked. Teaching the kernel to read a cup's two-prism
  boundary — the same analytic faces a prism carries, over two intervals — is the
  missing piece; until it lands, an invoked cup pair is staged, exactly as the
  wall reading is.
- **The no-opening shell.** A hollow closed body — the only shape with a genuine
  `IsVoid` shell — is a shell that removes no face, and the selector vocabulary
  has no way to ask for it: core §9 makes an empty match an error (S16), and a
  cardinality assertion takes a positive count, so a *satisfied* assertion of
  zero is not a spelling the contract offers. Asking for it therefore means adding
  something — a nullary `WithNoOpenings()` option is the obvious candidate — and
  nothing is chosen. Admitting a zero-count assertion instead would be a change to
  the selector contract, which is core §9's to make, not this document's.
- **Ancestor provenance across a consuming op.** §11 mints every role of a result
  from the record it labels, so no face of a modified body carries the extrude's
  refs, and `FaceCreatedBy` of an earlier step selects nothing on it. A face can
  hold several refs (evaluator §3 unions them on a canonicalization merge), so
  carrying the ancestor's `(step, role)` **alongside** the result's own is
  expressible — but it is a change to what `FaceCreatedBy` promises across a
  consuming op, and that promise is core §9's to make.
- **The asymmetric chamfer**, and the **variable-radius fillet**: both are
  options with nowhere to land until `ChamferOpts` and `FilletOpts` carry a
  field, and both are recordable when they do. Neither is in v1.
- **Modify ops on a revolve.** A revolve's meridian section stands to its body
  exactly as a prism's section stands to its own — a latitude-circle edge is a
  meridian corner, and rounding that corner sweeps a **torus**, which is in the
  shipped surface set. The reduction of §2 looks like it generalizes intact. It
  is not designed here (R3 refuses today), and whether it is increment 5's or a
  later one's is open.
