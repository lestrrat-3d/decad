# Modify Design

How the v1 evaluator builds the first straight-prism cases of the three modify
ops — `Body.Fillet`, `Body.Chamfer`, `Body.Shell`.

`docs/modify-reach-design.md` owns the approved extension to revolve
junctions, complete prism cap loops, side-opening/closed shells, tangent-chain
selection, and asymmetric chamfers. This document remains the shared
foundation: call semantics, base gates, section rewrite/audit, and shipped
payloads.

Four tables are normative, and each states its facts **once**:

| Table | States | Section |
|---|---|---|
| **R** — the receiver | which body a modify op accepts, keyed on the **payload class** | §3 |
| **S** — the refusals | every refusal, with the §1 existence test that picks its sentinel, and the order the gates run in | §4 |
| **B** — the result | op × removed faces × sense × section connectivity → payload class, **lump count**, faces and their roles | §9 |
| **D** — downstream | one row per consumer, and whether it reads the payload only or reads roles | §12 |

Every other section cites a row of one of them and restates none of it.
Around the tables: what a modify op owes and how it refuses (§1), the reduction
that keeps the analytic shape and propagates bounded mass results, with
Exactness only when the proven bound is zero (§2, §10), where the 2D work is
allowed to live (§5), the fillet (§6), the chamfer (§7), the shell (§8), the
recipe, the provenance and the replay (§11), the increment plan (§13) and the
reach decisions (§14).

Companion to `docs/api-design.md`, which owns the three operation signatures
and their context-aware forms, the selector contract and the retire rule
("core §N"), and to
`docs/evaluator-design.md`, whose §11 row 5 this document is ("evaluator §N").
`docs/verification-design.md` owns how the results are judged
("verification §N"). `docs/tessellation-design.md` owns the mesh construction
and proof bounds behind Table D's tessellation row.
`docs/payload-verification-design.md` owns the exact cup verification
follow-on. Nothing here changes those contracts.

## 1. What a modify op owes, and how it refuses

A modify op consumes one body and returns another (core §6). It has exactly
two honest outcomes, and the whole of this document is the line between them:

- **the body the caller asked for, built analytically** — analytic faces,
  bounded measurements, a boundary valid by construction; or
- **an error at the call**, naming which of the two things went wrong.

There is no third outcome. A blend clipped back because the radius did not fit,
an offset whose self-overlap was quietly trimmed away, a corner patched with a
surface nobody can name — each is a body the caller did not ask for, wearing
the measurements of one they did. That is core §1's confidently-wrong
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
reads `Suspect`. Table D (§12) names the remaining staged cup question: its
clearance against another body (D6). The exact `MinWallThickness` theorem (D1)
is implemented. The clearance proof and implementation order live in payload
verification §3; until that row lands, the existing `Suspect` staging remains.

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

That exactness is what needs the rewritten walks to be line and arc segments,
and it is the whole of the requirement. A prism's section may also carry a
free-form walk, so each modify op does need a gate against a free-form carrier.
`docs/spline-design.md` owns those gates: Table C there gives what a free-form
section builds, Table R the modify refusals and their sentinels, and §4.1 both
their derivation from this reduction and the analytic-corner slice that
survives it.

That is the whole design, and everything else follows from it:

- **bounds are inherited, not re-argued.** The mass-property engine
  (evaluator §4) integrates the rewritten region in closed form, so volume,
  area and centroid carry the engine's representability and error bounds (§10);
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
| **R1** | `prismPayload` — an extrude, a filleted body, a chamfered body, a tube (B2/B3), or any of these `Placed` | **builds** for lateral edges here; reach RX1 adds complete cap loops | **builds** for cap removal here; reach RX1 adds the B4 multi-lump result and side/no-opening cases |
| **R2** | `cupPayload` — a one-cap shell (B5/B6) | S3 | S3 |
| **R3** | `revolvePayload` | reach RX2 for swept meridian junctions; otherwise S3/SX5 | reach RX2; otherwise S3/SX8 |
| **R4** | `facetedPayload` — a boolean output | reach SX9 | reach SX9 |
| **R5** | reach payload (`stackedPrismPayload` / `capBlendPayload`) | reach SX10 | reach SX10 |

A full-circle loop is a single closed wall with **no** lateral edge at all
(evaluator §5 emits no seam), so a cylinder has no edge the corner rewrite can
name but its cap rims — and a query naming one is S1 for a `Fillet`, a query
naming nothing is S16. That is the honest reading of the class, not an
oversight: the rolling blend of a cap edge is the vertex-blend problem (§6), and
this increment does not solve it. A `Chamfer` of that same rim leaves the corner
rewrite instead of refusing, because a cylinder's rim **is** one complete cap
loop and reach §8.3 builds it (RX1's second class).

## 4. Table S — the refusals

Every refusal in this document, once. The middle column is §1's test; the
sentinel follows from it and from nothing else.

| S | The call asked for | Does that body exist? | Sentinel |
|---|---|---|---|
| **S1** | in the base increment, a fillet or chamfer of an edge that is not a **lateral** edge — reach RX1 replaces this only for complete prism cap loops | yes | `ErrUnsupported` — partial-loop endpoint transition not built |
| **S2** | in the base increment, a shell that removes a **side wall** — reach RX1 replaces this only for its one proper outer-loop run | yes | `ErrUnsupported` — general open-chain offset not built |
| **S3** | a modify op on a receiver/target not admitted by Table R or reach Table RX | yes | `ErrUnsupported` |
| **S4** | a blend or bevel of a corner whose two segments meet **smoothly** (tangent) or in a **cusp** (anti-tangent) | no — there is no corner to blend | `ErrDegenerate` |
| **S5** | a blend at a corner where **no blend surface of that radius exists**: the two carriers' material-side offsets never meet (parallel lines, concentric circles), or a circular carrier with the material **inside** it has `R − r ≤ 0` — its `r`-offset into the material is empty (`r > R`) or the circle's own centre, whose foot on the arc is not unique (`r = R`) | no | `ErrDegenerate` (§6) |
| **S6** | a cutback that reaches or passes the far end of a walk — including a far end another corner **in the same call** claims | yes — the blend surface is pinned by its own corner's offsets, and merging the two (or trimming each against what it runs into) builds the body | `ErrUnsupported` (§6) |
| **S7** | a rewrite whose loops **cross or make boundary contact** — a loop crossing or touching itself, or two loops of the section crossing or touching each other (a tangency, a shared boundary point, a pinch) | yes — a resolving kernel trims the pieces against each other, and a boundary touch is the limiting case of a crossing: the solid exists under a trimmed-offset kernel this evaluator lacks | `ErrUnsupported` (§5) |
| **S8** | a rewritten loop that has turned **inside out** — its signed area has changed sign | no — the modification consumed the region | `ErrDegenerate` (§5) |
| **S9** | a rewrite whose loops neither cross nor make boundary contact but whose **nesting the audit cannot decide** (§5) | this evaluator cannot tell | `ErrUnsupported` — it declines rather than guess |
| **S10** | an **inward** thickness that leaves **no cavity** — at or beyond the section's **inradius**, or, where a cap is **kept** (B5), at or beyond the sweep's **height**, which that cap's floor consumes | no — the cavity is empty; the wall has eaten the part, across the section or along the sweep. The two limits are independent, and an **outward** thickness has neither: a dilation of a non-empty region is never empty, and an outward floor *adds* height below the kept cap instead of eating it. A **both-caps** shell keeps no cap, so it grows no floor and its cavity runs the whole sweep: only the inradius limit can fire on B2/B4 | `ErrDegenerate` (§8) |
| **S11** | a shell whose **exact offset changes the section's feature set**, `ErrUnsupported` in either of two shapes at two points in the order. **S11a — a feature the offset drops**, caught **as the offset is built**: a **segment** dropped (a circular segment with the material inside it and `R ≤ t` inward: its offset radius `R − t` reaches zero or goes negative, the arc vanishes and its neighbours miter), or a **loop** dropped (a hole narrower than `2t` outward, whose erosion is empty). It is **antecedent to the §5 audit** — a dropped feature leaves no constructed section to audit — so it precedes S8. **S11b — a loop the offset merges or splits**, caught **by the §5 audit's crossing test** (§5 test 3, in S7's slot between S8 and S9): a slot or gap narrower than `2t` inward, whose two offset walls cross — or, at exactly `2t`, touch. A merge is the expected outcome of an offset, so the shell owns that event and S7 never fires on an offset (§8). | yes | `ErrUnsupported` — this evaluator's offset is per-feature and topology-preserving; resolving either needs a trimmed-offset kernel it does not have (§8) |
| **S12** | in the base increment, a **both-caps** shell of a **holed** section — the wall is one band around the outer loop plus one band lining each hole: `1 + k` lumps (B4); reach BX8 replaces this after the multi-region stacked payload lands | yes | `ErrUnsupported` — a base `prismPayload` holds one region, and the base evaluator has no multi-lump payload (§9, §14) |
| **S13** | a **zero radius** or a **zero distance** — a body identical to the one the caller already holds | it exists, and it is the receiver: a question with one answer and no content, exactly as `Verify`'s zero tool is (verification §2) | `ErrDegenerate` |
| **S14** | a **zero thickness** shell | no — a face is removed and the wall is `P \ P`: the empty region, no solid at all | `ErrDegenerate` |
| **S15** | a magnitude of the wrong `Kind`, a non-finite one, or a negative one | — | `ErrUnitKind` / `ErrNotFinite` / `ErrNegativeMagnitude` (core §12 names all three by role) |
| **S16** | a selector that matches nothing — loudly (core §9): a query asserting no cardinality, `ErrNoMatch`; a failed `Exactly(n)` / `AtLeast(n)`, `ErrCardinality`, zero matches included (core §12) | — | `ErrNoMatch` / `ErrCardinality` |
| **S17** | a modify op on a retired receiver | — | `ErrRetiredBody` |
| **S18** | an **inward** shell whose section-inradius survey has more than **1,048,576 candidate-family visits** under checked arithmetic, whose count overflows, or whose one shared **1,048,576-visit** generation + validation budget is exhausted | this evaluator cannot decide whether the cavity exists within its fixed work limit | `ErrUnsupported` (§8) |

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
| **2 — the receiver and its targets** | is this body one a modify op takes, and is what the query named a thing it can act on? | S3 (Table R's payload class), then S1 (every selected edge is lateral — or, for a `Chamfer`, every geometric edge of one or more complete cap loops, which leaves this route for reach §8.3 and takes reach Table SX's gates from here on) / S2 (every removed face is a cap) |
| **3 — the construction's own gates** | does the rewrite the caller asked for exist, feature by feature? | fillet / chamfer: S4 (there is a corner), then S5 (a blend of that radius exists — fillet only). Shell: S18 (the inward section survey fits its fixed work limit), S10 (the cavity is non-empty — inward only: the eroded section, and the height a kept cap's floor leaves), then S11a (no feature the offset drops as it is built) |
| **4 — the §5 audit of the rewritten profile** | do the pieces bound a simple, correctly nested region? | S8 (orientation — the existence question, so a consumed region never reads `ErrUnsupported`), then S6 (no walk consumed by its own corners — an offset mints none, §8), then S7 (no crossing and no boundary contact; for a **shell** either is S11b, §8), then S9 (nesting, which is decidable only once no two loops cross or touch) |
| **5 — what the result can be held as** | the region is proven; can a payload hold it? | S12 (a both-caps shell of a holed section is `1 + k` lumps) |

Each stage needs the one before it, and that is what fixes the order rather than
taste: there is no cutback to measure until the blend centre exists (S5), no
offset loop to orient until the evaluator can decide the cavity (S18), the
cavity exists (S10), and the offset keeps its features
(S11a), and no lump count to take until the offset bounds a proven region. **S12
is therefore last** — an inward both-caps shell of a holed section at or beyond
the survey limit is S18, one at or beyond the inradius is S10, and one whose
offset merges two loops is S11b; none reaches the count (B4).

**S11a precedes S8 without contravening the existence-before-buildability rule.**
That rule governs two gates asked on **one constructed section** — where both
could fire on the same section, the existence question (S8) wins. S11a is
different: it is **antecedent**, firing *while the offset is being built*, and a
dropped feature leaves **no constructed section** for S8 to audit at all. So S11a
and S8 are never asked on the same inputs; S11a running first is the stage order
(a section must be built before it can be audited), not a buildability gate
jumping ahead of an existence one. The audit's own crossing test — **S11b** — is
the one that sits inside the audit, after S8, exactly where §5's order puts it.

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

**Every modify audit is cancellable with bounded work.** `FilletContext`,
`ChamferContext`, and `ShellContext` create one `workBudget` for their
pre-commit cancellation path. Fillet and Chamfer share it through profile
walks, segment-pair crossing/contact tests, hole-pair tests, and each
ray-boundary containment scan. Shell also shares it through exact offset
construction before that audit. The budget polls at phase boundaries and after
at most `workPollInterval` candidate operations. A cancelled operation returns
`ctx.Err()` before commit, leaving the receiver live and the recipe and
document unchanged. The original `Fillet`, `Chamfer`, and `Shell` methods
remain source-compatible wrappers using `context.Background()`. The `cupWall`
morphology recheck shares its budget across profile validation and integration,
offset construction, and the same audit through `Document.Verify`'s context.

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
3. **No crossing, and no boundary contact.** Every pair of segments within a
   rewritten loop, and every pair drawn from two loops of the section, is tested
   for intersection — line×line, line×circle, circle×circle, the same closed
   forms the clearance kernel's 2D reduction and `survey2d.go`'s boundary walks
   use. A transverse crossing is **S7**; so is a mere **boundary contact** — a
   tangency, a shared boundary point, a pinch. A touch is the limiting case of a
   crossing, so it takes the same sentinel: the loops provably meet but bound no
   simple region until a kernel this evaluator lacks trims them against each
   other. On a shell's offset either is **S11b** (§8).

   **Contact is declared at a scale-anchored floor, reject-only.** Two
   non-adjacent segments are in boundary contact when the minimum distance
   between them is within `δ = ε·D` — `ε = 1e-9`, the **same** constant
   verification §4 fixes for its diameter-anchored noise floor `δ = ε·D` (the one
   decad already uses elsewhere, as `verify.go`'s clearance-gap tolerance gate
   `appendClearance`), not a new constant; and `D` the rewritten section's
   (u, v) bounding-box diagonal, decad's standard reading of the §3 diameter (it
   is ≥ the true diameter, so the floor overstates `δ` slightly, the conservative
   direction — it refuses a hair more, never a hair less). The test admits in one
   direction only: `segMinDist(pair) ≤ δ` ⇒ the two loops are indistinguishable
   from a pinch ⇒ **REFUSE** (the S7 family → `ErrUnsupported`); a gap comfortably
   above `δ` ⇒ **BUILD**. A positive gap below the floor is **not** built — it is
   below decad's resolution, and refusing it loudly is the sound conservative
   behaviour, never a silently trimmed body. The threshold **scales with the
   section**; it is never a fixed absolute constant.
4. **Nesting preserved.** Once no two loops cross and none make boundary contact,
   each loop lies wholly inside or wholly outside every other, so nesting is
   decided by classifying **one point** of each loop against each other loop.
   Both conditions are load-bearing: two loops enclose disjoint interiors only
   when their boundaries neither cross **nor** merely touch (Jordan), so a shared
   boundary point — a tangency or a pinch — leaves them not cleanly nested even
   with no crossing, and test 3 has already refused it (S7). Containment is *not*
   a crossing test and is not free: the classifier is the ray-parity walk with
   direction retries that `survey2d.go` already runs, and it admits an
   **undecided** outcome. A build-time audit has no `Suspect` to fall back on, so
   an undecided containment is **S9** — the evaluator declines. The audit passes
   only when the outer loop is proven to contain each hole and the holes are
   proven mutually disjoint.

A residual proves nothing and admits nothing; a crossing, boundary-contact or
containment test on exactly represented line and arc segments is a **decided**
fact of decad's own data, and its verdict is the same under every evaluator. It is what makes the
build's "valid by construction" claim (evaluator §10) survive a modify op: the
region is proven simple before the prism over it is built, so no unproven body
is ever made, and `Verify` reads the result exactly as it reads an extrude's.

## 6. Fillet

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
func (b *Body) FilletContext(ctx context.Context, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
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

**The base increment excludes the corner problem.** Where a cap-edge chain ends
at a vertex, its rolling blend needs an endpoint transition this section rewrite
does not carry. So the base class is drawn where the problem does not arise:
**a lateral edge's blend terminates on the two caps, and two lateral blends
never share a vertex** —
distinct lateral edges are disjoint, and each blend runs cap to cap, so there is
no vertex at which two of them meet and no patch that would close one. A cap
edge is base S1. Reach §8 admits only a **complete** cap loop: its closed
material-side center path gives named cylinder/torus patches and spherical
miter patches, while every partial chain remains SX4.

Two lateral blends can still **interfere**, and interference is refused, never
patched. Two corners of one wall claim it from both ends: S6. Two corners that
share no wall at all — opposite ends of a thin neck, two corners of one loop
that are not adjacent, a corner of the outer loop and a corner of a hole across
a thin section — can have their rewritten pieces cross or come into boundary
contact without either overrunning a walk, and the §5 audit catches those: S7.
Both fire before a face is made, and neither produces a corner needing a surface
nobody can name.

`FilletOpts` carries nothing in this increment, so a fillet `Step`'s `Opts` is
nil (core §6.2: nil when the op takes none). A variable-radius or setback option
lands in it, with the struct, when it ships.

## 7. Chamfer

```go
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
func (b *Body) ChamferContext(ctx context.Context, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
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
valid length and S13 for a zero `d`; S1 for a cap edge **the corner rewrite is
asked to take** — a partial cap chain, or a cap edge mixed with lateral ones,
which reach RX1 renumbers SX4; S4 for a smooth or
cusped corner; then the §5 audit — S8, S6 for a setback that reaches or passes
the far end of a walk, S7, S9. A selection covering every geometric edge of one
or more **complete** cap loops is not this route at all: it leaves the corner
rewrite entirely and builds through reach §8.3's cap-loop chamfer, whose own
refusals are reach Table SX's. A corner with a circular neighbour
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
func (b *Body) ShellContext(ctx context.Context, sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
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
does. The feature call materializes that default as explicit
`ShellOpts{Sense: Inward}`; a stored shell-options object with no `sense` does
not request the default and is malformed. The thickness passes the magnitude
gates before either question below is asked: a wrong `Kind`, a non-finite or a
negative one is S15, and a zero one is S14.

**The section offset is exact, and it is closed in the recorded vocabulary.**
Write `P` for the section. The inward offset (the erosion) `P ⊖ t` is bounded by:

| Feature of `P` | Its offset |
|---|---|
| a line segment | the parallel line, `t` into the material |
| a circular segment, material inside | the concentric circle of radius `R − t` (`R ≤ t` drops the segment: S11a) |
| a circular segment, material outside — a hole wall, a concave round | the concentric circle of radius `R + t` |
| a **convex** corner | a miter: the two offset curves meet, and the corner stays sharp |
| a **reflex** corner | an **arc of radius `t` centered on the corner point** — the nearest boundary feature there is the corner itself, so the erosion's boundary is at distance exactly `t` from it |

Every piece is a line or an arc, so `P ⊖ t` is a `ProfileRecord`. The outward
offset `P ⊕ t` is the same table with the two corner rules exchanged (a convex
corner rounds, a reflex one miters) and the radii moved the other way. Both are
exact.

**Three gates, and they are different questions — asked in that order (§4),
because each needs the one before it to have passed.**

- **Can this evaluator decide whether the cavity exists within fixed work?**
  Before an inward inradius survey starts, count concentric-scan,
  element/vertex pair, and Apollonius-triple candidate-family visits with
  checked arithmetic.
  Require at most **1,048,576 visits**. Stream every emitted disk directly into
  validation, and charge generation plus every whole-boundary validation visit
  to one shared **1,048,576-visit budget**. Count overflow, a preflight count
  above the limit, or runtime exhaustion is S18 (`ErrUnsupported`). Outward
  shelling does not need an inradius survey, so S18 does not reach it.
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
  feature set is S11, in two shapes at two points in the order: **S11a**, a
  feature the offset **drops** — a segment, or a loop whose offset is empty —
  caught **as the offset is constructed**, so it is antecedent to the audit and
  precedes S8; and **S11b**, a loop the offset **merges or splits**, caught **by
  the audit's crossing test below** (in S7's slot, after S8 and before S9).
  Staged, not denied: refusing costs the caller a shell decad could in principle
  build; producing one costs them a part that is wrong where they cannot see it —
  the same principle evaluator §12 states for the tapered extrude.
- **Can a payload hold the result?** Only now — with an offset that exists and
  bounds a proven region with `P`'s own feature set — is the wall's **lump count**
  a question with an answer, and a result in more than one piece is S12 (B4).
  Staged for the same reason, and **last** for this one.

**The offset section is a rewrite, so it faces the §5 audit like any other**, and
the audit runs between the second gate and the third, in its own order (§4): an
offset loop that has turned inside out is **S8**, asked first as the audit's
existence test; then the crossing test — **a crossing or boundary contact of
offset loops is S11b, not S7**, because a merge is the expected outcome of an
offset (two walls closing on each other at `2t` *is* the feature-set change S11b
names, and touching at exactly `2t` is its limiting case), so the shell's own row
owns the event and S7 never fires on an offset; then nesting — an offset whose
nesting the containment classifier cannot decide is **S9**, which the evaluator
declines rather than guess. The trim test cannot fire at all: it tests a cutback,
and an offset mints none — a segment the offset consumes is a dropped feature,
which is **S11a**, caught at construction (the gate above), not here.

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
cardinality assertion takes a positive count. It is **unspellable** through the
removal selector, not staged: there is no face to name. `WithNoOpenings()`
spells it instead — the one nil-selector shell (§14).

## 9. Table B — the result

One row per (op × removed faces × sense × section connectivity). `k` is the
number of holes in the receiver's section `P`; hole-free means `k = 0`. `[z0, z1]`
is the receiver's sweep interval, and the removed cap is taken as the top
(`z1`) — a removed bottom is its mirror. `Q` is the offset section: `P ⊖ t`
inward, `P ⊕ t` outward.

| B | Op | Removed | Sense | Section | Payload | Lumps | Faces (roles) | Refusals |
|---|---|---|---|---|---|---|---|---|
| **B1** | `Fillet` / `Chamfer` | — | — | any (`k ≥ 0`) | `prismPayload` over the **rewritten** section, same frame, same `[z0, z1]` | **1** | side walls `side(i,j)` over the rewritten record, two caps `capStart` / `capEnd`. The blend cylinder / bevel plane **is** one of those walls, and carries a **second** role `fillet(i,j)` / `chamfer(i,j)` naming the same `(loop, segment)` of the rewritten record | S1, S4, S5 (**a fillet only** — S5 is a condition on the two carriers' `r`-offsets, which only the blend computes; a chamfer's chord exists between any two distinct feet, §7), then the §5 audit: S8, S6, S7, S9 |
| **B2** | `Shell` | both caps | `Inward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: P, Holes: [Q]}`, on `[z0, z1]` | **1** | outer walls `side(0,j)`, cavity walls `side(1,j)`, and the two **rim annuli** — the caps of that prism — `capStart` / `capEnd` | S18, S10 (its **section** limit only — no cap is kept, so no floor eats the sweep), S11a, then the §5 audit: S8, S11b, S9 |
| **B3** | `Shell` | both caps | `Outward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: Q, Holes: [P]}`, on `[z0, z1]` — no cap is kept, so no material is added along the sweep | **1** | as B2 | S11a, then the §5 audit: S8, S11b, S9 (no S18 or S10 — outward shelling has no inradius survey or thickness limit) |
| **B4** | `Shell` | both caps | either | holed (`k ≥ 1`) | — | **1 + k** — a band around the outer loop, plus one band lining each hole, pairwise disjoint | — | S18 and S10 (**`Inward` only**, with S10's **section** limit only — B2's reason), S11a, the §5 audit's S8, S11b and S9 — every one of them decided on the offset section, and so reached before the count is — then, and only then, **S12** |
| **B5** | `Shell` | one cap | `Inward` | any (`k ≥ 0`) | a **cup**: `cupPayload` — the outer prism over `P` on `[z0, z1]` and the cavity prism over `Q = P ⊖ t` on `[z0 + t, z1]`, an interval S10's **height** limit is what proves non-empty. The kept cap does not move; the floor is `t` of the original material | **1** — every wall band hangs off the floor slab | outer walls `side(i,j)`, the kept cap `capStart`, the **rims** `rim(i)` — the removed cap's plane trimmed to the band between loop `i` of `P` and loop `i` of `Q`, one face per loop (`1 + k` of them) — cavity walls `shellSide(i,j)`, cavity cap `shellCap` | S18, S10 (**both** its limits — this is the one row whose floor eats the sweep), S11a, then the §5 audit: S8, S11b, S9 (no S12 — one cap is kept, and every band hangs off the floor it leaves) |
| **B6** | `Shell` | one cap | `Outward` | any (`k ≥ 0`) | a **cup**: `cupPayload` — the outer prism over `Q = P ⊕ t` on `[z0 − t, z1]` and the cavity prism over `P` on `[z0, z1]`. The original solid *is* the cavity; the floor is `t` of new material below the kept cap | **1** | as B5 | S11a, then the §5 audit: S8, S11b, S9 (no S18, S10, or S12, for B3's and B5's reasons) |

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
the cavity region on its own — plus the private shell-thickness/sense morphology
certificate of payload verification §3.1 and the accumulated rigid placement,
which is what `Body.Placed` re-evaluates (evaluator §8) and what every measurement
reads (§10). Every edge of B1–B3, B5 and B6 bounds exactly two faces, so each body is
manifold and watertight by the same structural argument the prism enjoys
(evaluator §10), on regions the §5 audit has already proven simple.

## 10. Mass properties, and how their bounds compose

**A modify op carries the analytic engine's bounds.** Its result is one prism, or two, over
regions the mass-property engine (evaluator §4) integrates in closed form —
`LineSeg`, `CircleSeg` and `ArcSeg` walks are exactly the kinds it already
integrates, and the arcs a fillet and an offset add are those kinds. An exactly
representable result is `Exact`; every rounded result is `Approximate` with a
proven `Bound` and passes verification only at a sufficient tolerance
(verification §5). Write `A_X` for the area of region `X`, and
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
| `Area` | caps + Σ (segment length · h); an arc's length is `rθ` with its evaluation bound | rim annuli + Σ (segment length · h) over both loops | Σ (outer segment length · `h_o`) + Σ (cavity segment length · `h_c`) — each wall band over the interval of the prism it belongs to — plus the kept cap (`A_outer`), the rim bands (`A_outer − A_cavity` in total: the removed cap's plane, less the opening) and the cavity cap (`A_cavity`) |
| `Centroid` | the rewritten region's centroid, lifted to the interval's signed midpoint | the section's centroid (holes subtract, as the engine already does), lifted likewise | each region's centroid lifted to the midpoint of **its own** interval, the two combined with the cavity's mass subtracted: `(A_outer · h_o · c_outer − A_cavity · h_c · c_cavity) / (A_outer · h_o − A_cavity · h_c)` |
| `Bounds` | per-segment analytic extremes over the interval | the same | the outer prism's — in both senses the cavity lies within it, `Q ⊂ P` on the shorter interval inward, `P ⊂ Q` on the shorter one outward |

Each is a difference or a sum of bounded quantities the engine already produces;
none is sampled, and none is fitted. The modify op propagates both source and
arithmetic bounds through those formulas.

**The rewritten section is the body's truth, in exactly the sense a recorded one
is.** A tangent foot is the root of a closed-form equation, computed once, in
floating point — as is every coordinate the seam records and every vertex an
extrude places. The represented *shape* remains analytic. Measurement `Exact`
means the reported binary number is proved exactly representable; otherwise its
floating-point evaluation is `Approximate` with a proven outward bound.

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
  §9's question; consuming modify results do not inherit ancestor face roles
  (§14).

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
| **D1** | `prismWall` + `survey2d` (`MinWallThickness`) | the payload only | works unchanged: the rewritten section is a section, the height is the receiver's, the reading is Exact | works unchanged: a tube **is** a prism over an annular section | payload verification §4's morphology recheck returns an exact reading: zero for an allowance-qualified material pinch, otherwise the exact shell thickness `t`; a failed recheck reads `Suspect` |
| **D2** | `prismUndercuts` | the payload **and the roles** — it looks each payload walk's face up by `side(i,j)` on the body's own step | works unchanged (§11): every wall of the result, blends included, carries its `side(i,j)` role in the result's index space | works unchanged | a cup reading lands with the cup payload: the same per-face exact normal ranges over the faces of B5/B6, mapped by their roles |
| **D3** | `prismMinRadius` | the payload only | works unchanged: a fillet of a **concave** edge is a concave arc of the section, and its radius is read; a fillet of a convex edge adds a convex cylinder, which is not a concave feature and rightly does not appear | works unchanged: the cavity loop's walls are read like any hole wall | a cup reading lands with the payload: the same walk over the outer and the cavity section. The sharp concave edge where the wall meets the floor carries no radius — the survey reads faces' principal radii, and a spec about the *edges* is one no option states (verification §2) |
| **D4** | `Tessellate` → `STL` / `OBJ` | the payload **and the roles** — `docs/tessellation-design.md` owns the chording, source-face map, and proof bounds | works by tessellation design §§3–5: blend cylinders are ordinary circular section walks | works by the same prism path | works by tessellation design §6: every outer/cavity loop is shared by its walls, floors, and rim band |
| **D5** | Body-relative stops (`ToFace` / `ToFaceAngular`; `ThroughAll` / `ThroughAllSide`) | two stop kinds read differently: `ToFace` / `ToFaceAngular` read **topology + a selector + a surface** — a live stop body, its face resolved by the selector, and that face's plane; `ThroughAll` / `ThroughAllSide` read **the payload's directional extent** (`extentAlong`) | works unchanged | works unchanged | `ToFace` reads the cup's faces like any body's; `ThroughAll` reads its outer prism's extent — the cup's own `extentAlong` — the cavity being interior |
| **D6** | Clearance (`docs/clearance-design.md`) | **the payload and the topology** — it builds its boundary model by switching on the payload **kind** (`prismPayload` / `revolvePayload`), then reads the exact edges, vertices and shells and each body's payload extent | a first-class operand — a `prismPayload`, which its switch builds | a first-class operand — a tube is a `prismPayload` too | payload verification §3 adapts the exact outer/cavity skins, three axial planes, rim bands, topology witnesses, and outer extent to the analytic kernel; staged as `Suspect` when the kernel is needed until the cup-boundary stage lands, while a box-disjoint pair with no gap request remains proven |
| **D7** | The mesh boolean (evaluator §9) | the tessellation | takes these bodies as it takes any other | as any other | as any other, once D4 covers it |
| **D8** | `Verify` — the structural audit and the tolerance gate | the topology and the measurements | valid by construction (§5); numerical bounds pass only at a sufficient tolerance (§10) | the same | D1's exact wall answer is available; mass bounds still pass only at a sufficient tolerance; a pair that requires D6 remains `Suspect` |

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

Reach PRs A–E follow these base PRs. `docs/modify-reach-design.md` §14 owns
their order; RX/SX/BX/DX own every additional build and staging result.

D1's cup-wall theorem is implemented. D6 remains staged in the implementation;
payload verification §13 names its approved cup-boundary stage. Until that stage
lands, an invoked cup clearance pair reads `Suspect`; a box-disjoint cup pair
with no clearance asked is still proven `Sound` by the box test.

The walk-sense rules of §6 — a circular wall's face orientation and the
walked-boundary `Edge.IsConvex` — are a **prerequisite**, not a deliverable. They
belong to the prism build, which reads them on profiles a fillet has nothing to
do with; PR 1 builds on them and carries none of them.

## 14. Reach decisions

`docs/modify-reach-design.md` resolves the former reach questions:

- cap-edge support requires a complete prism cap loop; line/circle center paths
  produce cylinder/torus patches, and miter points produce trimmed sphere
  patches;
- patch/offset merging remains a deliberate `ErrUnsupported` limit;
- topology-changing offsets remain a deliberate `ErrUnsupported` limit;
- `stackedPrismPayload` owns multi-region axial slabs, replaces `cupPayload`,
  and lifts S12 by holding the B4 outer band plus every hole-lining band;
- minimum wall on cap-blend/stacked payloads deliberately reads `Suspect` when
  asked;
- clearance reads exposed analytic faces of the new payloads and may return its
  existing undecided pair result — reach DX6, whose reader is staged for the
  cap-loop chamfer: boxes may still decide a pair, and every other requested
  pair reads `Suspect` until that reader lands;
- `WithNoOpenings()` spells a closed shell and is the only nil-selector shell;
- consuming modify results do not inherit ancestor face roles;
- `WithAsymmetricChamfer` carries a reference face + second distance;
- `WithTangentChain` expands only through proven analytic G1 continuations;
- revolve meridian junctions reuse the exact section rewrite;
- variable-radius fillets, partial cap chains, mixed edge classes, and all
  faceted receivers stay explicitly unsupported.

The extension's PR order and required tests are reach §§13–14. No reach choice
in this document remains open.

Free-form reach is `docs/spline-design.md`'s, not this document's or the
extension's: Table C there owns what a free-form section builds, Table R the
modify refusals it earns, and §4.1 the analytic-corner slice (§2).
