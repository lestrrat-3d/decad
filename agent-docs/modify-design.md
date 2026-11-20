# Modify Design

How the v1 evaluator builds the first straight-prism cases of the three modify
ops — `Fillet`, `Chamfer`, `Shell`.

Part R owns the approved extension to revolve
junctions, complete prism cap loops, side-opening/closed shells, tangent-chain
selection, and asymmetric chamfers. This document remains the shared
foundation: call semantics, base gates, section rewrite/audit, and shipped
payloads.

## Parts

This file holds 14 parts. A section number with no part prefix is a section of Part F.
`Part X §N` is section N of part X. Short names such as `reach §3` in comments and design docs
mean the same part. The parts keep their own section numbers.

| Part | Former document | Short name | Owns |
|---|---|---|---|
| Part F | the unprefixed sections of this file | `modify` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, build audit |
| Part R | Modify reach | `modify-reach`, `reach` | Tangent chains, asymmetric chamfers, cap-loop blends, shell reach and staging |
| Part G | Modify general | `modify-general` | Shell of a through-cut brep (S), complete-loop chamfers (L); Tables TC/SG/LB/SL |
| Part B | Brep modify | `brep-modify` | Modify ops on brep and stacked receivers; Tables RB/EB/SB/BB/DB |
| Part L | Loop fillet | `loop-fillet` | Route L's fillet arm: pipe bands on a planar face's loop |
| Part V | Three-edge vertex blends | `vertex-blend` | Three-edge fillets, sphere patches, route V, exact-radius refusals |
| Part A | Edge blend variants | `blend-variants` | Chord-length fillet, angle chamfer and the oblique boss fillet |
| Part E | Retained edge blend | `retained-edge-blend` | `Fillet` and `Chamfer` of an edge between two retained faces of a mesh Boolean result |
| Part M | Retained modify reach | `retained-modify-reach` | Modify and placement reach on retained mesh Boolean results |
| Part O | Shell opening | `shell-opening` | Shell side openings on a prism and a revolve; Tables RO/SO/BO/DO |
| Part S | Shell variants | `shell-variants` | `Shell` of a faceted body with bosses and of a twisted sweep |
| Part D | Draft | `draft` | `WithTaper` and `Draft`: the sharp offset family; Tables RD/SD/BD/DD |
| Part T | Tangent side notch | `tangent-notch` | Blind side-notch `Cut` meeting the outer wall tangentially, and its floor-arc fillet |
| Part X | Perpendicular cylinder bore-mouth chamfer | `cross-cylinder-bore-chamfer` | Source-backed perpendicular cylinder bore-mouth chamfer and mesh proof |

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
commit and provenance rules (§11), what each base step builds (§13) and the
reach decisions (§14).

Companion to `agent-docs/api-design.md`, which owns the three operation signatures
and their context-aware forms, the selector contract and the retire rule
("core §N"), and to
`agent-docs/evaluator-design.md`, whose §11 row 5 this document is ("evaluator §N").
`agent-docs/verification-design.md` owns how the results are judged
("verification §N"). `agent-docs/tessellation-design.md` owns the mesh construction
and proof bounds behind Table D's tessellation row.
`agent-docs/payload-verification-design.md` owns the exact cup verification
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

Both sentinels are returned from the call before commit: a failed evaluation
leaves the document untouched (evaluator §8),
so a refused modify op retires nothing and the caller still holds a live body.
Neither is ever deferred into a `Verify` reading — `Verify` judges bodies the
document holds, and a refused call produced none. The staging split of
evaluator §11 is exactly this: an intent the evaluator cannot **build** is an
error at the call; only a question it cannot **answer** on a body it did build
reads `Suspect`. Table D (§12) names the remaining staged cup question: its
clearance against another body (D6). The exact wall-thickness theorem (D1)
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
`agent-docs/spline-design.md` owns those gates: Table C there gives what a free-form
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

**The reduction is private evaluator data.** The public call states "fillet
these edges at 2 mm" with a selector and radius; it never exposes the rewritten
section. A later exact kernel can build the same intent as a true rolling-ball
surface without inheriting this evaluator's 2D reduction.

The one thing the reduction cannot express is a result in **more than one
piece**: a `ProfileRecord` is one outer loop and its holes, so a `prismPayload`
is one connected region swept once. The lump count is therefore a load-bearing
fact of every result, and Table B (§9) carries it as a column.

## 3. Table R — the receiver

The receiver is admitted on **what its payload is**, never on the op history
that produced it. `Transform` re-evaluates a payload under a composed motion
and yields the same class (evaluator §8), so a placed body reads exactly as
the body it was placed from. The receiver must also be **live**: a modify op on
a retired body is S17, by core §6's retire rule.

| R | Receiver payload | `Fillet` / `Chamfer` | `Shell` |
|---|---|---|---|
| **R1** | `prismPayload` — an extrude, a filleted body, a chamfered body, a tube (B2/B3), or any of these after `Transform` | **builds** for lateral edges here; reach RX1 adds complete cap loops; route E or the bounded cutter adds one straight cap edge for either blend; route V adds mixed complete-loop fillets (Part V) | **builds** for cap removal here, the B4 multi-lump result through reach BX8; reach RX1 adds side/no-opening cases |
| **R2** | `cupPayload` — a one-cap shell (B5/B6), recorded as a two-slab `stackedPrismPayload` (reach §9.1) | S3 | S3 |
| **R3** | `revolvePayload` | reach RX2 for swept meridian junctions and one bounded quarter-turn radial cap rim for either fillet or chamfer; otherwise S3/SX5 | reach RX2; otherwise S2/SX8 |
| **R4** | `facetedPayload` — a boolean output | reach RX6, otherwise SX9 | One authenticated nested rectangular-boss Union top cap builds a bounded cup (Part S); otherwise reach SX9. |
| **R5** | `capBlendPayload` | reach SX10 | reach SX10 |
| **R6** | `brepPayload`, or `stackedPrismPayload` through its face view | Part B Table RB | Table RB |
| **R7** | `loftPayload` with matching, hole-free, axially translated sections | `Fillet` on recorded analytic corners (loft §17); `Chamfer` is S3 | S3 |
| **R8** | One-sided positive-taper full-circle `draftPayload`, near section at start, no holes or kept walls | Far-rim `Fillet` or symmetric `Chamfer` builds per draft §10.4–§10.5; otherwise S3 | Inward far-cap removal builds draft §10.6's cup; otherwise S3 |
| **R9** | One straight solid profile `sweepPayload` (§6–§7) | Lateral `Fillet` and equal-setback `Chamfer` use the exact prism reduction; one straight cap edge of either op takes route E. | Cap or side openings; closed Shell if hole-free. |
| **R10** | One straight `twistedSweepPayload` with a smooth symmetric convex Line/Arc section | S3 | Inward top-cap Shell builds a bounded faceted cup. See Part S. |

R9 includes rigid placement and only a zero-twist, one-span `LineTo` path.
One straight cap-edge Fillet or Chamfer uses the stored prism's face view and
route E's edge matching, material-side check, face rewrite and closure proof.
Fillet also checks the public cap and wall roles before that match. Other cap
selections, mixed selections and asymmetric setbacks refuse. Shell maps each
selected `side(0,0,j)` face to segment `j`
of the stored exact prism before the prism side-opening build. R10 requires
the origin XY plane, an origin-to-positive-Z path, an exact thickness, and
the top-cap opening. R10 evaluates these source conditions in the local
frame before its audited rigid placement. Other Sweep kinds are S3.

A full-circle loop is a single closed wall with **no** lateral edge at all
(evaluator §5 emits no seam), so a cylinder has no edge the corner rewrite can
name but its cap rims. A complete rim takes route L's loop fillet; a query
naming nothing is S16. A `Chamfer` of that rim takes reach §8.3.

## 4. Table S — the refusals

Every refusal in this document, once. The middle column is §1's test; the
sentinel follows from it and from nothing else.

| S | The call asked for | Does that body exist? | Sentinel |
|---|---|---|---|
| **S1** | in the base increment, a fillet or chamfer of an edge that is not a **lateral** edge; route E or the bounded cutter adds one straight prism cap edge and route V adds mixed complete-loop fillets (Part V) | yes | `ErrUnsupported` — remaining partial-loop endpoint transitions are not built |
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
| **S12** | a **both-caps** shell of a **holed** section — the wall is one band around the outer loop plus one band lining each hole: `1 + k` lumps (B4). Reach BX8 builds it as a `stackedPrismPayload` prism group, so S12 refuses no call | yes | none: reach BX8 holds the `1 + k` bands (§9, §14) |
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
| **5 — what the result can be held as** | the region is proven; can a payload hold it? | S12's slot: a both-caps shell of a holed section is `1 + k` lumps, which reach BX8 holds |

Each stage needs the one before it, and that is what fixes the order rather than
taste: there is no cutback to measure until the blend centre exists (S5), no
offset loop to orient until the evaluator can decide the cavity (S18), the
cavity exists (S10), and the offset keeps its features
(S11a), and no lump count to take until the offset bounds a proven region. **The
bands are therefore built last** — an inward both-caps shell of a holed section
at or beyond the survey limit is S18, one at or beyond the inradius is S10, and
one whose offset merges two loops is S11b; none reaches the bands (B4).

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
2D geometry, in a repository whose first hard rule is **ask `sketch` for 2D
answers by default**. The line is the one evaluator §4 already draws for the mass
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
  inscribed-disk kernel of `internal/survey2d/wall_kernel.go`.

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

`internal/sectionaudit/` computes signed area, crossing, contact and nesting.
The root audit orders those checks with each modify op's own cutback rules.

**Every modify audit is cancellable with bounded work.** `Fillet`,
`Chamfer`, and `Shell` create one `proofbound.WorkBudget` for their
pre-commit cancellation path. Fillet and Chamfer share it through profile
walks, segment-pair crossing/contact tests, hole-pair tests, and each
ray-boundary containment scan. Shell also shares it through exact offset
construction before that audit. The budget polls at phase boundaries and after
at most `proofbound.WorkPollInterval` candidate operations. A cancelled operation returns
`ctx.Err()` before commit, leaving the receiver live and the document
unchanged. The `cupWall`
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
   forms the clearance kernel's 2D reduction and `internal/survey2d/wall_kernel.go`'s boundary walks
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
   direction retries that `internal/survey2d/wall_kernel.go` already runs, and it admits an
   **undecided** outcome. A build-time audit has no `Suspect` to fall back on, so
   an undecided containment is **S9** — the evaluator declines. The audit passes
   only when the outer loop is proven to contain each hole and the holes are
   proven mutually disjoint. Once no two loops cross or touch, every point of a
   loop classifies the whole loop, so `sectionaudit.Nesting` tries the loop's
   first walk start and reads a later start only where the ray test cannot
   decide the earlier one (a start on another loop's carrier circle, as a
   window's start on a concentric window's circle). Every candidate is a point
   of the loop, so the verdict is the same whichever start decides it
   (`agent-docs/prism-boolean-design.md` Part K §6.5).

A residual proves nothing and admits nothing; a crossing, boundary-contact or
containment test on exactly represented line and arc segments is a **decided**
fact of decad's own data, and its verdict is the same under every evaluator. It is what makes the
build's "valid by construction" claim (evaluator §10) survive a modify op: the
region is proven simple before the prism over it is built, so no unproven body
is ever made, and `Verify` reads the result exactly as it reads an extrude's.

## 6. Fillet

```go
func Fillet(ctx context.Context, b *Body, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
```

`r` is a magnitude, gated at the call like every other (S15), and a zero `r` is
S13.

For R9's straight Sweep, `Fillet` resolves the selector on the published Sweep
edges, then matches each lateral edge to the stored prism section corner. One
straight cap edge takes the stored prism's brep face view through route E. The
public edge must border one cap and one `side(0,loop,segment)` wall; route E
matches it to one record edge and checks both end-face corners against the
interior material wedge before rewriting them. Other cap selections are S1.
The lateral section rewrite and audit build B1's `prismPayload` over the same
frame and interval, including the Sweep's endpoint displacement bound. Arc,
composite, mitred and twisted Sweep payloads do not take either route.

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
on its outer loop. The rules therefore belong to the prism build, and the fillet relies on
them: a concave round — a clockwise circular walk on an outer loop — is the first
thing a fillet emits (§13).

**The base increment excludes the corner problem.** Where a cap-edge chain ends
at a vertex, its rolling blend needs an endpoint transition this section rewrite
does not carry. So the base class is drawn where the problem does not arise:
**a lateral edge's blend terminates on the two caps, and two lateral blends
never share a vertex** —
distinct lateral edges are disjoint, and each blend runs cap to cap, so there is
no vertex at which two of them meet and no patch that would close one. A cap
edge is base S1. Reach §8 admits only a **complete** cap loop: its closed
material-side center path gives named cylinder/torus patches; LF9 adds a
sphere where a previously filleted lateral edge's arc collapses. Every
remaining partial chain stays SX4.

Two lateral blends can still **interfere**, and interference is refused, never
patched. Two corners of one wall claim it from both ends: S6. Two corners that
share no wall at all — opposite ends of a thin neck, two corners of one loop
that are not adjacent, a corner of the outer loop and a corner of a hole across
a thin section — can have their rewritten pieces cross or come into boundary
contact without either overrunning a walk, and the §5 audit catches those: S7.
Both fire before a face is made, and neither produces a corner needing a surface
nobody can name.

A fillet's one option is reach §5's `WithTangentChain`. There is no
variable-radius or setback option.

## 7. Chamfer

```go
func Chamfer(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
```

For R9's straight Sweep, `Chamfer` resolves the selector on the published
Sweep edges and maps each selected lateral edge to the stored prism corner.
Equal setbacks use the ordinary section rewrite, audit and B1 build, carrying
the same frame, interval, endpoint displacement and placement. The resulting
`prismPayload` takes later R1 operations. A cap edge, a mixed cap/lateral
selection or `WithAsymmetricChamfer` is `ErrUnsupported`; other Sweep kinds
remain S3. `WithTangentChain` retains its usual selector expansion.

The same edge class, the same gates, one simpler surface. **The bevel of a
lateral edge is a plane**: in the section, the corner is cut off by a straight
chord, and the chord swept along the sweep direction is a planar face.

**The setback is measured along the adjacent boundary curve**, `d` from the
corner along each — a length along a `LineSeg`, an **arc length** along a
circular one. On a straight prism that is exactly the geodesic setback from the
edge across the adjacent face, because the boundary curve *is* that face's
cross-section: the two readings coincide, so the definition is unambiguous
rather than merely convenient. The setback is equal both ways unless the call
passes reach §6's `WithAsymmetricChamfer`, which names the face that takes
`d` and a second distance for the other face. A distance and an angle is no
option.

The rewrite trims both walks back by `d` and joins the feet with a `LineSeg`.
The gates are the fillet's, in §4's order: S15 for a magnitude that is not a
valid length and S13 for a zero `d`; S1 for a cap edge **the corner rewrite is
asked to take** — a partial cap chain of at least two edges, or a cap edge mixed with lateral ones,
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

A chamfer takes reach §5's `WithTangentChain` and reach §6's
`WithAsymmetricChamfer`.

## 8. Shell

```go
func Shell(ctx context.Context, b *Body, sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
```

`sel` names the faces to **remove** — the openings. What remains of the solid is
a wall of the given thickness behind every face that was *kept*. On a prism, with
the removed faces restricted to caps (S2), the whole construction is the
section's own offset.

**Inward is the default sense, and outward is enumerated, never signed.** The
thickness is a magnitude, so it carries no sign (core §8.1's rule, applied
here): the sense is a `ShellSense` — `Inward` (the wall grows into the original
solid; the outer skin does not move) or `Outward` (the wall grows off it; the
original solid becomes the cavity) — set by `WithShellSense`, decoded into
the call's `internal/modifyoption.ShellConfig` record, and defaulting to `Inward`, which is
what "shell this box" means everywhere it is said. Two `WithShellSense`
options naming different senses are reach SX1. The thickness passes the magnitude
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
| a **G1 join** — the two walks leave and arrive along the same direction (a fillet's tangent foot, a slot's line–semicircle seam) | **the corner moved `t` along the shared left normal**: `v + t·n̂`, with `n̂` the leaving walk's own unit normal at `v`. The two offset carriers are tangent there, so this is their one common point, and intersecting them would solve for a double root the float discriminant cannot hold at zero |

Every piece is a line or an arc, so `P ⊖ t` is a `ProfileRecord`. The outward
offset `P ⊕ t` is the same table with the two corner rules exchanged (a convex
corner rounds, a reflex one miters) and the radii moved the other way; a G1
join is the same point either way, `v + s·t·n̂`. Both are exact as shapes.
The section the build records is their float evaluation at the thickness
held in millimetres, so a cup carries a proven bound on how far that record
sits from the offset the caller's thickness denotes (§9).

**Which row a corner takes is decided on the held unit tangents, with one dead
zone.** Write `a` for the arriving walk's unit leaving tangent and `b` for the
leaving walk's unit arriving tangent, both normalised from the walk's own held
components, and `cross = a × b`, `dot = a · b`. A corner is a **G1 join** when
`|cross| ≤ 1e-9` and `dot > 0`; it is an arc or a miter by the sign of `cross`
otherwise. A corner with `|cross| ≤ 1e-9` and `dot ≤ 0` is a cusp and stays on
the miter row, which refuses it as S11 when its carriers do not meet. The dead
zone is the construction's own rule, not an admission on a residual: it decides
which of two closed forms is built, never whether a body exists, and every gate
below — the dropped-walk test, S8, S11b, S9 — runs on the section it builds.
Within the dead zone the three rows name points at most `t·|a − b| + R·|cross|`
apart (a corner turning by under `1e-9` moves its miter or its connector arc by
under that), and the G1 row is the one whose point is well-conditioned in the
held floats. The sibling exact predicate `capband.JoinIsG1` (exact rational zero
cross over the record) is an admission test for the boolean proof and implies
this rule; this rule never replaces it.

**Three gates, and they are different questions — asked in that order (§4),
because each needs the one before it to have passed.**

- **Can this evaluator decide whether the cavity exists within fixed work?**
  Before an inward inradius survey starts, count concentric-scan,
  element/vertex pair, and Apollonius-triple candidate-family visits with
  checked arithmetic.
  Require at most **1,048,576 visits**. After that count, an exact contained
  disk may prove the requested thickness fits without finding the full
  inradius. Otherwise, stream every emitted disk directly into validation,
  and charge generation plus every whole-boundary validation visit to one
  shared **1,048,576-visit budget**. Count overflow, a preflight count above
  the limit, or runtime exhaustion is S18 (`ErrUnsupported`). Outward shelling
  does not need an inradius survey, so S18 does not reach it.
- **Does the body exist?** This is the inward sense's question, and the **cavity**
  answers it. The cavity is a region swept along an interval, so it is empty when
  either of them is — and the thickness can empty either one, which is why S10
  carries **two independent limits**:
  - the **section** limit: `P ⊖ t` is non-empty exactly when `t` is strictly less
    than the section's **inradius** — the radius of its largest inscribed disk,
    which `internal/survey2d/wall_kernel.go` computes as part of the wall survey. A contained disk
    whose radius exceeds the requested thickness plus the rounding margin
    proves the success side without the full survey. A failure still uses
    the inradius reading that answers `Wall.Minimum`;
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
  a question with an answer, and a result in more than one piece is the
  `1 + k` bands reach BX8 holds (B4), built **last** for this reason.

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

**A shelled body with a removed face has no void, and `FaceShell.IsVoid()` is false
on it.** An opening
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
| **B2** | `Shell` | both caps | `Inward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: P, Holes: [Q]}`, on `[z0, z1]` with the receiver's level denotations | **1** | outer walls `side(0,j)`, cavity walls `side(1,j)`, and the two **rim annuli** — the caps of that prism — `capStart` / `capEnd` | S18, S10 (its **section** limit only — no cap is kept, so no floor eats the sweep), S11a, then the §5 audit: S8, S11b, S9 |
| **B3** | `Shell` | both caps | `Outward` | hole-free | a **tube**: `prismPayload` whose section is `{Outer: Q, Holes: [P]}`, on `[z0, z1]` with the receiver's level denotations — no cap is kept, so no material is added along the sweep | **1** | as B2 | S11a, then the §5 audit: S8, S11b, S9 (no S18 or S10 — outward shelling has no inradius survey or thickness limit) |
| **B4** | `Shell` | both caps | either | holed (`k ≥ 1`) | a `stackedPrismPayload` prism group: one slab on `[z0, z1]` holding the `1 + k` bands as regions (reach BX8) | **1 + k** — a band around the outer loop, plus one band lining each hole, pairwise disjoint | per band `m` (the outer band first, then the hole linings in record order): walls `slab(0).region(m).side(i,j)` and two rims `capStart` / `capEnd` | S18 and S10 (**`Inward` only**, with S10's **section** limit only — B2's reason), S11a, the §5 audit's S8, S11b and S9 — every one of them decided on the offset section — then the bands build |
| **B5** | `Shell` | one cap | `Inward` | any (`k ≥ 0`) | a **cup**: `cupPayload` — the outer prism over `P` on `[z0, z1]` less the cavity prism over `Q = P ⊖ t` on `[z0 + t, z1]`, recorded as a floor slab over `P` on `[z0, z0 + t]` under a slab of the `1 + k` wall bands on `[z0 + t, z1]` (reach §9.1), an interval S10's **height** limit is what proves non-empty. The kept cap does not move; the floor is `t` of the original material | **1** — every wall band hangs off the floor slab | outer walls `side(i,j)`, the kept cap `capStart`, the **rims** `rim(i)` — the removed cap's plane trimmed to the band between loop `i` of `P` and loop `i` of `Q`, one face per loop (`1 + k` of them) — cavity walls `shellSide(i,j)`, cavity cap `shellCap` | S18, S10 (**both** its limits — this is the one row whose floor eats the sweep), S11a, then the §5 audit: S8, S11b, S9 (no S12 — one cap is kept, and every band hangs off the floor it leaves) |
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

`cupPayload` is the payload of B5 and B6: two
co-directional prisms over the same plane — the outer region on its interval,
the cavity region on its own — plus the private shell-thickness/sense morphology
certificate of payload verification §3.1 and the accumulated rigid placement,
which is what `Transform` re-evaluates (evaluator §8) and what every measurement
reads (§10). Every edge of B1–B3, B5 and B6 bounds exactly two faces, so each body is
manifold and watertight by the same structural argument the prism enjoys
(evaluator §10), on regions the §5 audit has already proven simple.

**A cup records its offset region within a proven displacement of the one it
denotes.** The denoted offset is `P ⊖ t*` (inward) or `P ⊕ t*` (outward), `t*`
the caller's thickness in exact millimetres. The recorded region is that
offset's float evaluation at the held thickness, so its corners can land a
few units in the last place off the denoted ones (a `0.1 in` wall puts one
cavity corner at `2.539999999999999`). `cupPayload.OffsetDelta` is the proven
upper bound on how far any recorded boundary point of the offset region — the
cavity inward, the outer region outward — sits from the denoted boundary; the
receiver's own region carries none. It is derived once, at the shell, as an
enclosure (the method §8.4 of Part R states for a cap
contour): §7's closed forms are re-evaluated over rational intervals across
the whole thickness interval `extent.MagnitudeInBounded` proves, with the receiver's
walk endpoints, centres and bracketed radii taken exactly and outward-rounded
square roots, and each recorded join is charged its enclosure's greatest reach
from the held point. A recorded line stays within the larger of its two end
reaches of the denoted line, and a recorded arc within three times the larger,
so the recorded figure is three times the largest reach. A corner the evaluator cannot enclose is `ErrUnsupported`. The
displacement plays `prismPayload.SectionDelta`'s role for that one region
(`agent-docs/prism-boolean-design.md` §7): the offset region's prism carries it as its
section displacement, so its walls, `Bounds`, the mass path, the through-all
extent and the verification gate's witness all charge it; the cup charges it
itself into the region's area and first moments and into the rim bands; the
mesh charges it on every face the region bounds, in its area slack and in its
occupied-volume proof; the concave-radius survey widens by it; and two equal
records carrying it are not a set-equality certificate. The shell-wall reading
needs none: it reads `t*`, which the thickness's own conversion bound already
covers.

**A cup's levels denote exact rationals as a prism's caps do**
(`agent-docs/prism-boolean-design.md` Part S §2.1). The open end and the kept end keep
the receiver end's denotation. The derived floor level, held as the float sum
of that end and the held thickness, denotes the end's rational plus or minus
`t*` (`prismshell.CapShellInput.StepLevel`). The exact construction makes
that claim true: the level's displacement covers the end's own, the
thickness conversion and the sum's rounding. `DenoteLevel` only checks it and
can only reject it. A `ToFace` stop on the cup's floor, kept cap
or rim records it, and an end that denotes nothing leaves its floor denoting
nothing.

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
arithmetic bounds through those formulas; a cup's offset region adds its
offset displacement (§9) as one more source bound.

**A fillet's or chamfer's rewritten section is the body's truth, in exactly the
sense a recorded one is.** A tangent foot is the root of a closed-form equation, computed once, in
floating point — as is every coordinate the seam records and every vertex an
extrude places. The represented *shape* remains analytic. Measurement `Exact`
means the reported binary number is proved exactly representable; otherwise its
floating-point evaluation is `Approximate` with a proven outward bound.

## 11. Commit and provenance

Each operation consumes the receiver: it resolves its selector against the
live body, builds and audits the result, then atomically retires the receiver
and registers the result. No caller-owned selector pointer survives the call,
and resolved topology pointers are never exposed as durable identity.

**A role is an index into the payload, so the result's roles index the result's
payload.** `FeatureRef` carries a private producer identity plus a role, and the role of a
side face is `side(i, j)` — loop `i`, segment `j` **of the payload the body
holds** (evaluator §3). A modify op rewrites the section: segments are trimmed,
inserted and renumbered, so a role inherited from the receiver would name a
segment of a record this body no longer has. The evaluator
therefore does what it already does when it re-evaluates a payload: **every face
of the result carries roles of the modify operation alone**, in the result's own index
space, and Table B lists them. There is no re-parenting problem because there is
no inheritance: a role is minted from the payload it labels.

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
| **D1** | `wallsurvey.PrismWall` + `survey2d` (`Wall`) | the payload only | works unchanged: the rewritten section is a section, the height is the receiver's; the reading is `Exact` only where it is proven exactly representable — a material pinch or a sweep height with no axial displacement — and otherwise carries the proven bound `survey2d`'s own candidate arithmetic derives | works unchanged: a tube **is** a prism over an annular section | payload verification §4's morphology recheck returns a reading: zero for an allowance-qualified material pinch, otherwise the shell thickness `t` — `Exact` only when converting `t` to millimetres was itself exact, otherwise carrying that conversion's own proven displacement; a failed recheck reads `Suspect` |
| **D2** | `analyticundercut.Prism` | the payload **and the roles** — it looks each payload walk's face up by `side(i,j)` on the body's own step | works unchanged (§11): every wall of the result, blends included, carries its `side(i,j)` role in the result's index space | works unchanged | `analyticundercut.Cup` reads the cup payload: the same per-face exact normal ranges over the faces of B5/B6, mapped by their roles |
| **D3** | `radiussurvey.Prism` | the payload only | works unchanged: a fillet of a **concave** edge is a concave arc of the section, and its radius is read; a fillet of a convex edge adds a convex cylinder, which is not a concave feature and rightly does not appear | works unchanged: the cavity loop's walls are read like any hole wall | a cup reading lands with the payload: the same walk over the outer and the cavity section. The sharp concave edge where the wall meets the floor carries no radius — the survey reads faces' principal radii, and a spec about the *edges* is one no option states (verification §2) |
| **D4** | `Tessellate` → `STL` / `OBJ` | the payload **and the roles** — `agent-docs/tessellation-design.md` owns the chording, source-face map, and proof bounds | works by tessellation design §§3–5: blend cylinders are ordinary circular section walks | works by the same prism path | works by tessellation design §6: every outer/cavity loop is shared by its walls, floors, and rim band |
| **D5** | Body-relative stops (`ToFace` / `ToFaceAngular`; `ThroughAll` / `ThroughAllSide`) | two stop kinds read differently: `ToFace` / `ToFaceAngular` read **topology + a selector + a surface** — a live stop body, its face resolved by the selector, and that face's plane; `ThroughAll` / `ThroughAllSide` read **the payload's directional extent** (`ExtentAlong`) | works unchanged | works unchanged | `ToFace` reads the cup's faces like any body's; `ThroughAll` reads its outer prism's extent — the cup's own `ExtentAlong` — the cavity being interior |
| **D6** | Clearance (`agent-docs/clearance-design.md`) | **the payload and the topology** — it builds its boundary model by switching on the payload **kind** (`prismPayload` / `revolvePayload`), then reads the exact edges, vertices and shells and each body's payload extent | a first-class operand — a `prismPayload`, which its switch builds | a first-class operand — a tube is a `prismPayload` too | payload verification §3 adapts the exact outer/cavity skins, three axial planes, rim bands, topology witnesses, and outer extent to the analytic kernel; staged as `Suspect` when the kernel is needed until the cup-boundary stage lands, while a box-disjoint pair with no gap request remains proven |
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

The base operations build in three steps inside evaluator increment 5. Anything a step does not build is
`ErrUnsupported` **at the call** — never a `Verify` reading, and never a body:

| Step | Builds | `ErrUnsupported` here |
|---|---|---|
| 1 | the section rewrite and its §5 audit, `Fillet` on lateral edges (line/line, line/arc, arc/arc corners), B1's roles, the `Step` wiring | every cap edge (S1); every non-prism receiver (S3) |
| 2 | `Chamfer`, equal distance | the asymmetric chamfer (reach §6) |
| 3 | `Shell`: cap removal, the exact erosion and dilation, the §5 topology gates, the tube (B2/B3), the `cupPayload` (B5/B6), and D2/D3/D4 extended to the cup | side-wall removal (S2); the topology-changing offset (S11) |

Part R §14 owns the reach order; RX/SX/BX/DX own every build and refusal beyond these steps, including
the both-caps shell of a holed section (S12, built by BX8).

D1's cup-wall theorem is implemented. D6 remains staged in the implementation;
payload verification §13 names its approved cup-boundary stage. Until that stage
lands, an invoked cup clearance pair reads `Suspect`; a box-disjoint cup pair
with no clearance asked is still proven `Sound` by the box test.

The walk-sense rules of §6 — a circular wall's face orientation and the
walked-boundary `Edge.IsConvex` — belong to the prism build, which reads them on
profiles a fillet has nothing to do with. Step 1 relies on them and carries none of them.

## 14. Reach decisions

Part R resolves the former reach questions:

- cap-loop blends use complete prism cap loops; line/circle center paths
  produce cylinder/torus patches, and miter points produce trimmed sphere
  patches; route E admits one straight cap edge;
- patch/offset merging remains a deliberate `ErrUnsupported` limit;
- topology-changing offsets remain a deliberate `ErrUnsupported` limit;
- `stackedPrismPayload` owns multi-region axial slabs, holds the cup's record
  beside its shell morphology, and lifts S12 by holding the B4 outer band plus
  every hole-lining band;
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
- variable-radius fillets, unsupported partial cap chains, mixed edge classes,
  and faceted receivers outside explicit source-backed routes stay unsupported.

The reach order and required tests are reach §§13–14. No reach choice
in this document remains open.

Free-form reach is `agent-docs/spline-design.md`'s, not this document's or the
extension's: Table C there owns what a free-form section builds, Table R the
modify refusals it earns, and §4.1 the analytic-corner slice (§2).

## Part R — Modify reach

How the evaluator extends `Fillet`, `Chamfer`, and `Shell` beyond the
straight-prism cases in Part F.

This part owns only the extension. Part F still owns the
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

### 1. Scope

The extension adds only shapes reducible to decad's recorded line/arc regions
or to a finite set of named analytic patches:

- prism cap-loop fillets and chamfers;
- prism shell side openings and closed shells;
- revolve junction-edge fillets and chamfers;
- full-revolve shells, plus partial-revolve shells with both angular caps open;
- exact tangent-chain expansion over analytic topology;
- two-distance chamfers with an explicit reference face.

The extension does **not** add a general B-rep kernel:

- no partial cap-loop `Chamfer` beyond modify-general §4.3a's straight chains;
- no mixed lateral-edge + cap-edge blend in one call;
- no variable-radius fillet;
- no topology-changing offset;
- no general faceted receiver (the isolated rectangular-prism Union edge blends
  below and certified cap routes are bounded exceptions; a brep or stacked
  receiver is Part B's);
- no partial-revolve shell that keeps an angular cap.

Each excluded body exists. Table SX stages it with `ErrUnsupported` at the
modify call. No excluded shape is approximated, clipped, or deferred into
`Verify`.

### 2. Public options

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

Each call decodes its options into a private record (`internal/modifyoption/`):
`FilletConfig{TangentChain}`, `ChamferConfig{TangentChain, Asymmetric}` and
`ShellConfig{Sense, NoOpenings, FaceThickness}`. A shell with openings states a removed-face
query and a thickness; a shell with no openings states no selector, a
thickness and `ShellConfig{NoOpenings: true}`.

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

### 3. Table RX — receivers + targets

Base Table R still admits the shipped straight-prism cases. RX adds these rows:

| RX | Receiver | Fillet / Chamfer | Shell |
|---|---|---|---|
| **RX1** | `prismPayload` | base lateral junctions; one straight cap edge through route E or the bounded cutter; OR every geometric edge of one or more complete cap loops. Route V admits both classes in one `Fillet` call where its partition holds (Part V) | base cap openings, with BX8 replacing base S12; OR, for a hole-free section, one proper connected run of outer side faces with any cap openings; OR `WithNoOpenings`, in either sense |
| **RX2** | `revolvePayload` | off-axis swept meridian junctions: a full-turn latitude `Circle3` or partial-turn junction `Arc3`; one bounded quarter-turn rectangular cap rim (§7.2) | full turn: one proper connected run of generated side faces, or `WithNoOpenings`; partial turn: both angular caps MUST be removed, with an optional proper connected side-face run |
| **RX3** | `stackedPrismPayload` | through its face view, Part B Table RB; a view that refuses is SX10 | the same, plus `WithNoOpenings` for a nested two-slab rectangular boss (modify-general §3.1a) |
| **RX4** | `capBlendPayload` | SX10 | SX10 |
| **RX5** | `cupPayload`: a two-slab `stackedPrismPayload` record beside the shell morphology (§9.1) | base S3 | base S3 |
| **RX6** | `facetedPayload`, including zero-bound all-planar boolean output | certified loft/cylinder outer cap `Chamfer`, including mirrored bore results, and the outer cap `Chamfer` of a hole-free mirrored Loft join (loft §19.1), one cross-cylinder bore-mouth `Chamfer`, or §10.1's isolated rectangular-prism Union edge `Fillet` or equal-setback `Chamfer`, or Part E's closed circular rim between a retained plane and a coaxial retained cylinder or a retained sphere, straight edge between two retained planes that ends on retained planes, or straight edge between a retained plane and a parallel retained cylinder that ends on planes normal to its axis, for `Fillet` or equal-setback `Chamfer`; otherwise SX9 | SX9 |
| **RX7** | `brepPayload` (`agent-docs/prism-boolean-design.md` Part G §4) | Part B Table RB; outside it, SX16 | the same |
| **RX8** | `loftPayload` | `Fillet` on analytic corners of axial sections whose segments match in kind, each section rounded on its own (loft §17); equal-distance `Chamfer` on both complete held outer cap loops of a matching axial loft with zero holes or one certified circular bore (loft §18), or of a loft whose sections differ, with zero holes or one whole-circle bore (loft §19); other chamfers are base S3 | base S3 |
| **RX9** | `draftPayload` with one positive-taper whole circular wall and its near section at the start level | draft §10.4–§10.5 | Removing its far cap inward builds draft §10.6's cup; other calls are base S3 |

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

### 4. Table SX — refusals + order

Base S1–S17 keep their meanings. Where RX admits a former S1/S2/S3 case, the
more specific SX row replaces that base refusal.

| SX | Call | Existence | Sentinel |
|---|---|---|---|
| **SX1** | `WithNoOpenings` with a non-nil selector, repeated contradictory option, or malformed option payload | no single intent | `ErrDegenerate` |
| **SX2** | tangent continuation is branch-ambiguous or the analytic oracle cannot decide G1 continuity | evaluator cannot know which chain caller named | `ErrUnsupported` |
| **SX3** | asymmetric reference is nil, invalid, or does not identify exactly one adjacent face per expanded edge | invalid selector / cardinality | existing selector error; otherwise `ErrCardinality` |
| **SX4** | a partial prism cap-loop `Chamfer` of at least two edges, a mixed lateral/cap `Chamfer`, a mixed revolve/cap selection, or one prism or brep loop with mixed asymmetric face assignments; route E admits one straight cap edge and route V admits complete cap loops with independent straight edges for `Fillet` | body exists; endpoint/setback transition not built | `ErrUnsupported` |
| **SX5** | selected revolve edge is neither a swept meridian junction nor §7.2's single quarter-turn rectangular cap rim | body exists; other cap-edge/general rolling blends are not built | `ErrUnsupported` |
| **SX6** | cap-loop offset loses a carrier, reaches an empty circular offset, or has no regular radius-`r` envelope; LF9's exact-radius inward arc with straight G1 neighbours is the fillet exception (Part V) | no regular requested blend | `ErrDegenerate` |
| **SX7** | cap-loop center paths cross/touch non-adjacent paths, a patch self-intersects, two cap bands meet, or trims need merging | body exists under trimming/merge kernel | `ErrUnsupported` |
| **SX8** | a side opening on a holed prism section; a non-empty side selection is not one proper outer-loop run; offset changes topology; partial revolve keeps either angular cap; revolve meridian is holed or meets the axis along more than one walk; a revolve wall's offset reaches the axis | body exists outside extension | `ErrUnsupported` |
| **SX9** | any modify op on `facetedPayload` outside RX6's certified classes and the closed inward `Shell` of a convex planar mesh Boolean result (`agent-docs/trim-graph-design.md`) | body exists; no admitted source and topology certificate for the selected modification | `ErrUnsupported` |
| **SX10** | another modify op on `capBlendPayload` or `capblend.DisplacedPayload`, or on a `stackedPrismPayload` whose face view refuses (Part B SB2) | body exists; compound feature composition not built | `ErrUnsupported` |
| **SX11** | inward closed/side-opening prism shell leaves axial cavity height `h - k*t <= 0`, where `k` is kept cap count; or section cavity is empty | no cavity | `ErrDegenerate` |
| **SX12** | cap chamfer ruled patches intersect away from shared boundaries or cannot be certified disjoint | body exists under trim kernel | `ErrUnsupported` |
| **SX13** | a cap-loop chamfer whose setback rounds away against the level it displaces: the cap contour's offset radius rounds back onto a circular wall's own radius (`R -/+ d == R`), or the band's side level rounds back onto its own cap level (`z1 - d == z1` on the end cap, `z0 + d == z0` on the start cap) | body exists; its taper is real but finer than float64 names at that radius or at that sweep level, so the band's patches cannot be told from a cylinder or from the cap plane | `ErrUnsupported` |
| **SX14** | a cap-loop chamfer whose denoted contour corner cannot be enclosed: the two offset carriers' interval intersection is unbounded, or the exact carriers do not meet where the float solve found a root. A G1 join (modify §7's dead-zone rule) intersects no carriers — its corner is the shared-normal foot, enclosed as a reflex corner's feet are — so SX14 never fires on one. Also a circular band patch whose corner skew between its side and cap directrices cannot be enclosed below a quarter turn (§8.4). Also a miter corner at a circular wall whose corner-foot locus cannot be bounded (`capband.MiterLocusUpper`): a tangent cusp, or a corner that turns past the G1 dead zone by less than the arc's own radius enclosure resolves | body exists; its offset corner is real and this evaluator cannot state where it is, so no cap-level coordinate there can publish a proven displacement, or no area bound holds for the ruled patch at that skew | `ErrUnsupported` |
| **SX15** | a cap-loop chamfer whose band patch's outward orientation cannot be certified: the patch's own `Face.NormalAt` refuses at the build's orientation sample point | body exists and its patches are real; the evaluator cannot evaluate its own orientation sample on this patch, so it cannot state which side of the patch is outward | `ErrUnsupported` |
| **SX16** | a modify op on a `brepPayload`, or on a stacked receiver through its face view, outside Part B Tables RB/EB and Part G Table LB | body exists; those documents' refusal tables name the row | `ErrUnsupported` |

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

The existence-first rule holds. SX6 precedes SX7: an empty
offset means no regular rolling-ball surface; intersecting valid patches mean
the surface exists but this evaluator cannot trim it.

SX13's RADIAL half sits with SX7/SX12 in stage 6 and after SX6 for the same
reason, on the same offset radius: SX6 is the offset that reaches the centre and
leaves nothing outside LF9's sphere case, while SX13 is the offset that leaves a
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
distance to publish and the call refuses rather than name one. The same row
covers the path the corner foot takes as the offset grows (§8.3): the slant
ruling's length and the circular patch's chord-locus term both bound it, and a
corner whose carriers meet in a double root at the corner, or in a root pair
closer than the radius enclosure resolves, has no such bound.
`capblend.RequireCornerLoci` decides it after the other stage-6 audits and
before the body is assembled, so the refusal names the corner.

SX15 is decided as each patch is constructed, like SX13's radial half: the
patch's own built surface is sampled at a point on it, and where that sample's
`NormalAt` cannot answer, the sign is a build-time question this evaluator
cannot finish, so it refuses rather than publish a patch whose outward side was
never checked.

### 5. Tangent-chain expansion

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
expansion, except a mesh Boolean result whose seeds are retained lines and arcs between retained plane and cylinder
tags: its junctions are decided by `retainedblend.Tangent` (`retained-modify-reach §5.2, §5.3`).

#### 5.1 Implementation

`internal/tangentchain/geometry.go` decides continuations;
`brep_modify_partial.go` adapts handles, selectors and refusals. `Fillet` and `Chamfer` call it
right after the seed query resolves. `Chamfer` also calls it without `WithTangentChain`, as the target CAD's chamfer
extends its edges along tangent chains: an expansion that grows the selection replaces it, and one that refuses (SX2,
SX9) leaves the selection as named. The oracle is the clearance kernel's
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

### 6. Asymmetric chamfer

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
Part B §5 admits the option for independent straight
edges of a `brepPayload` or stacked face view through route E, and complete
loops of planar faces through route L (Part G §4.2).
The stacked face view carries the public source-face role for each record
index. Route P does not map a reference face to its construction, so an
asymmetric chamfer skips it. Selections outside routes E and L remain SX16.

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

### 7. Revolve junction rewrite

A revolve's meridian profile has the same corner-to-edge mapping as a prism's
section:

| Meridian record | Revolve topology |
|---|---|
| walk | side face of revolution |
| off-axis corner | latitude circle on a full turn; angular arc on a partial turn |
| on-axis corner | one vertex, no selectable edge |

For a meridian-junction selection, map every selected RX2 edge to
`(loop, corner)` of the axis-local coalesced walk. Match the stored analytic
edge against the junction record; do not use a topology index. §7.2 covers
one angular-cap radial rim separately.

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

#### 7.1 Implementation

`internal/brepmodify/revolve_blend.go` runs this section; `Fillet` and `Chamfer` route a
`revolvePayload` receiver to it after stage 1 and SX10/SX16. Both operations
try §7.2's source-specific radial rim first.

**Matching.** The build stamps every junction edge from
`revolvePayload.JunctionCircle`: centre on the axis at the junction's `z`,
the placed sweep axis, radius `ρ`. A selected edge is a junction exactly when
its curve kind is the sweep's (`Circle3` full, `Arc3` partial) and its centre,
axis and radius equal one junction circle's bit for bit, recomputed from the
same payload. A cap edge's circle lies in a cap plane, and an on-axis or cap
line is a `Line3`, so neither matches this junction route: SX5 outside §7.2.
Equality is exact because the two
numbers come from one function over one payload; no tolerance chooses a
junction.

**Corner coordinates.** The payload records the meridian plane-locally, and
the rewrite is rigid-invariant, so the corner rewrite runs on the plane-local
coalesced walk (`offset2d.ProfileCornerLoopsBudget`), the one a prism's section uses.
The axis-local junction maps to it through the recorded segment its leaving
walk starts at. Where the two coalescings disagree about that segment, the
call refuses with `ErrUnsupported` rather than picking a neighbouring corner.

**Axis gates.** `revolveBlendAxis` reruns `revolveplan.ResolveAxisSide` — side gate,
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
`Transform` and `Copy` re-mint them; a path that replaces the profile
clears them. Measurements, topology, tessellation and the surveys are the
revolve's own (Table DX).

**Asymmetric chamfer.** The junction's two adjacent walls are swept walks of
the meridian, and each carries the `side(i,j)` roles of the recorded segments
it sweeps. §6's role reading picks which plane-local walk takes `d`, and
`computeChamfer` sets each walk back by its own distance.

#### 7.2 One quarter-turn angular-cap radial-rim blend

`internal/brepmodify/rim_revolve.go` admits an unplaced solid Revolve of one exact
XY-plane rectangle `(xlo,0)`–`(xhi,R)`, swept from zero to a denoted exact
quarter turn about +X. The four natural-range lines must follow that order;
the source has no holes, section displacement, previous blend, or draft
rewrite. `internal/revolveplan/rim_source.go` limits `|xlo|,|xhi| ≤ 16` and `1 ≤ xhi−xlo,R ≤ 16`, and the exact millimetre
fillet radius or equal chamfer setback satisfies `0.125 ≤ a ≤ 4` and
`2a < min(xhi−xlo,R)`. An asymmetric chamfer remains SX5. One
selected `Line3` must run from `(xhi,0,0)` to `(xhi,R,0)`. A different
orientation, angle, placement, source, edge, or magnitude takes SX5. The
result is faceted and does not admit another modify op.

For `z ∈ [0,R]`, the true result has `0 ≤ y ≤ H(z) = √(R²−z²)` and
`xlo ≤ x ≤ f(z)`. On `[0,a]`, the fillet has
`f(z) = xhi−a+√(2az−z²)` and the chamfer has `f(z) = xhi−a+z`; above `a`,
both have `f(z) = xhi`. The fillet's quarter-cylinder and the chamfer's
plane each meet the source outer cylinder along an analytic intersection
curve. At `y=0`, the fillet meets the other angular cap along a quarter-circle
and the chamfer meets it along a line. Both forms of `f`, and `H`, are concave,
so chords through inward-rounded stations lie inside the true region.

The build uses 128 blend and 512 outer-wall intervals. Every station's `z`
is read as an exact dyadic. `RatSqrtDown` puts the fillet's held `x` and both
blends' held `y` at or inside their exact radicals. The chamfer's held `x`
is the downward-rounded exact affine value. `RatSqrtUp` encloses the radical
values. Adjacent station
rectangles share vertices; their four ruled walls, first angular cap and
collapsed last rim form a closed mesh. `RequireVertexLinks` and the faceted
mesh audit reject a broken held boundary. The five retained source faces
keep their origins; the new wall carries `fillet(0,1)` or `chamfer(0,1)`.

For each z interval, let `df` be the next station's upper `f` minus the
current held `f`, and `dh` the current upper `H` minus the next held `H`.
Monotonicity gives `0 ≤ f−F ≤ df` and `0 ≤ H−G ≤ dh`, where `F,G` are the
held linear walls. Every true boundary point and its held counterpart are
within `df+dh`; the maximum becomes each vertex's boundary certificate.
The call refuses if it reaches 0.1 mm. The held solid is inside the true
one, so its occupied-volume difference is at most
`Σ Δz [R·df + (xhi−xlo)·dh]`, summed in exact rational arithmetic and
rounded upward once. The faceted builder integrates held volume and first
moments exactly and uses this difference with the box diagonal to bound
centroid displacement. Each true face area is below the sum
`2R²+4R(xhi−xlo)+2aR`; `AreaSlack` is the larger of that and the audited
held mesh area upper bound. This bounds each source face's area difference.

### 8. Complete prism cap-loop blends

Cap-loop support requires a complete loop. This removes the free-end setback
patch that makes a partial edge chain a general B-rep problem.

#### 8.1 Common record

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

#### 8.2 Fillet

For radius `r`, intersect two offset carriers:

- cap plane offset `r` into material;
- adjacent side carrier offset `r` into material.

Their intersection in the offset cap plane is the rolling-ball center path.
Part L builds this band as route L's fillet arm (Table
LF) on a prism's cap loop (its RF3) and on a brep's planar-face loop:

| Center path | Blend patch |
|---|---|
| line | `Cylinder`, radius `r` |
| circle/arc | `Torus`, major = path radius, minor = `r` |
| one centre point after an exact-radius inward arc collapses (LF9) | `Sphere`, radius `r`, with a cap pole |
| a reflex corner's connector arc | horn `Torus`, major = minor = `r`, about the corner |
| a line–line miter | no patch: the two cylinders meet along an `Ellipse3` |
| a regular line–circle or circle–circle miter, not tangent | `FilletMiter3` joins the two pipes; corner allowances bound mass, area and mesh deviation |

The rolling ball never rests at a miter point, so no sphere is swept there:
the two tubes meet along the ellipse in the corner's bisector plane, and a
sphere octant would leave a flat shelf of area `r²(1 − π/4)` at the side
level (loop-fillet §2). The sphere is the vertex blend of three blended
edges (Part V §1).

Trim each patch between exact contact traces on cap and side carriers. Replace
the selected cap loop with its cap contact trace. Trim each adjacent side face
to its side contact trace. Canonicalize tangent patch joins; keep non-tangent
joins as topology edges.

Regularity gates:

- every material-side carrier offset exists, except LF9's admitted point: SX6;
- every circular center path has positive regular tube reach over its trim;
- offset loops preserve orientation, simplicity, and nesting;
- non-adjacent center paths stay strictly farther than `2r`;
- every center path stays strictly farther than `r` from every unselected
  boundary carrier it does not belong to;
- blend bands from opposite caps do not meet.

Failure of the first two is SX6. Cross/contact/merge is SX7.

#### 8.3 Chamfer

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
curve strictly between them. That residual is bounded, never ignored. At
each height the denoted surface is the cone over a window whose two ends
are the corner-foot loci's azimuths, and each locus's azimuth is monotone in
the offset amount: for a line meeting the circle, the cosine of the azimuth
against the line's normal is `(α + σ·t)/(R + ρ·t)`, whose derivative has a
fixed sign, and for two circles the cosine of the angle at this centre has
a derivative of fixed sign because `R1 ∓ R2` is constant; in both cases the
azimuth stays on one side of its reference line while the two carriers do
not touch, which the corner's sliver bound requires before it charges
anything. So each end of the window runs between the side window's end and
the cap window's end. A corner can trim the cap window inside the side
window or extend it past, and a patch can do one at each end, so the true
swept flux is sandwiched between the ordinary cone-sector flux `W` read over
the union of the two windows and `N` read over their intersection; with
both corners trimming they are the side and cap windows. Both reference
fluxes are read about the
arc's own axis at the side level, where the cone's flux density `R0·r(z)`
never changes sign; about the plane-local origin the density changes sign,
the sandwich fails, and the flux difference grows with the section's distance
from that origin. The same monotone azimuth tightens the sandwich. Over
each of 32 offset sub-ranges a locus's angle from its corner's ray lies
between the angles of the corner feet at the sub-range's two ends, which
`capband.CornerLocusSpans` encloses, so the denoted window's width is
enclosed at every height to within the loci's movement over one sub-range,
and integrating the flux density against those widths encloses the denoted
flux within about a thirtieth of `|W − N|`. A reflex foot and a G1 join run
along the wall's own radial, an angle of zero. The term keeps the
intersection of that enclosure and the sandwich; a setback with conversion
rounding, whose offset-to-height map is not exact, keeps the sandwich alone.
The built ruled patch's own flux `B` about the same point is enclosed by the
same certified closed form, with no claim about where the ruled surface
sits, and the about-axis gap is at most the larger distance from `B`'s
enclosure to either end of the denoted enclosure. The gap is never
taken as `|W − B|` through a displacement-times-area bound: the wide sector
and the built patch end on different rulings, and the corner triangle
between them carries a flux of about `½·R0·R1·H·Φ` per corner, which no
bound on how far interior points move accounts for. The band's volume sums every patch's flux about the plane-local
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
circles, `W = ∫₀^dc (dc/2 − t)·P'(t) dt` is enclosed over 128 offset
sub-ranges: each contributes `(dc/2 − mid)` times the difference of the
enclosed feet at its two ends, plus at most `width²/4` times its velocity
box's half-width, and the moment crosses that enclosure with the exact arm
`v − c`. The error falls as the square of the sub-range count, and the
lens-shaped test corners are charged within a few percent of their exact share. The
share is zero at a reflex foot, a G1 join and a whole turn, whose loci are
straight, and a corner whose share cannot be bounded makes the volume bound
unbounded. The built volume's own error holds none of this flux: it cancels
between the two patches a ruling joins. It is owed because each `Cone`
patch's bound is taken about its own axis. `ChordLocusVolumeAllow` composes
the three terms into one proven volume bound.

That bound covers the signed volume gap, which can be smaller than the
volume of the region between the two solids when the built patch lies
outside the denoted surface in one place and inside it in another. The first
moment (§8.4) needs the region's volume, so it reads `ChordLocusRegionAllow`
instead. Its proof works along horizontal rays out from the arc's axis and
reads no point on the axis, so the half-ulp anchoring of the reference
fluxes' levels does not enter it. Each patch's difference of the two solids
is closed by joining the corner-foot locus to the built ruling with a
straight segment at each height, and on each ray the winding number of that
closed surface is nonzero only between its nearest and farthest crossing.
Between the two corner wedges only the cone, at `r(z)`, and the built patch
cross the ray. The built patch's level curve sweeps the azimuth
monotonically and covers that range, and its radius sits at most
`max(R0, R1)·Φ²/4` inside `r(z)` (`ChordLocusBuiltDeficitUpper`), `Φ` the
larger corner skew, since `r² − |B|² ≤ R0·R1·sin²(Φ/2)`. Inside a corner
wedge every crossing, of the cone, the built patch or the joining segments,
lies within `(3/8)·max(R0, R1)·Φ²` inside `r(z)`
(`ChordLocusCornerDeficitUpper`). So the whole region lies in a shell under
the cone. Its cross-section per unit angle is a difference of half squared
radii, and the built patch's dip,
`r² − |B|² = 4·v·(1−v)·R0·R1·sin²((θC − θS)/2)`, integrates over the height
to at most `max(R0, R1)²·Φ²/12` per unit angle,
over the union of the two windows. Each corner's sliver adds `r²·δ²/8` only
over its own angle `δ(v)` between the locus and the built ruling, which the
corner locus spans enclose span by span. The shell's volume is at most
`H·max(R0, R1)²·((w + s0 + s1)·Φ²/12 + Σ ∫ δ³ dv/8)`, with `w` the narrower
window's width (`ChordLocusShellUpper`): second order in the skew. The region term charges that shell and nothing else: the
corner slivers lie in the shell. A patch whose corner sliver could not be bounded answers an
unbounded region, since its locus may fold back. The proof reads the
corner-foot locus as the carrier root nearest the corner: the two roots
mirror each other across the line through the two centres, or across the
perpendicular from the centre to a line, and the locus stays on the
corner's side of it until the carriers touch, which the sliver bound
refuses. Both terms compare the built and denoted surfaces between the
same two held levels. The held side level sits up to `capband.BandLevelDelta`
from the denoted one, and that move is the body's, not the patch's: the
slab and the band meet at the held level, so the body changes only where a
vertical line leaves the band toward the cap, by at most the larger of the
side loop's and the cap contour's areas times the move
(`capband.LevelVolume`), which the band's volume and first-moment bounds
charge. The band volume charges it there alone: its side disk carries only
the cap level's inherited displacement. The skew the
region term reads is the larger of the patch's two proven corner
skews (§8.4's `CornerSkewUpper`), the exact angle between each corner's held
side end and held cap end; the volume term reads it only to test for zero.
The difference of the two held windows is not used:
each end of it is a float `Atan2`, and over a sweep of sectors drawn away from
the sketch origin that difference can fall below the exact corner angle. Both terms round every operation outward: the flux sums and
differences are taken exactly and rounded once, and every product is rounded
up. A skew that is not finite answers an unbounded volume. The
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
  (`internal/surfacenormal/normal_bound.go`).
- The same stamp adds the turn between the built patch and the wall its records
  denote (`capband.DenotedNormalAllow`). The cap directrix is the offset the
  build solved, held within the contour displacement `δ` of the denoted one in
  the plane and within the cap level's own displacement along the axis, and the
  side level is held within its rounding. A flat patch's tag passes through its
  side edge and one cap corner, and moving that corner by `e` turns the normal
  by at most `2e/h`, `h` the axial height. A circular patch is coaxial with its
  denoted cone, and its half angle moves by at most
  `(e_r + |Δr|·e_z/h)/h`; its held side and cap radii widen `e_r` by their own
  allowances. The term charges `2·(δ + δ_z·max(1, |Δr|/h))/h` with `h` at the
  bottom of its span. It is zero only where every one of those displacements
  is. Without it, a small chamfer on a square drawn far from the origin (`apitest/capblend_bounds_test.go`) publishes
  a bound far below the walls' distance from the exact 45° normal.
- A flat patch also states `topo.Face.FlatPatch` while its stamp is below the cap: it denotes the plane through its
  straight side directrix and the parallel cap directrix over the window its one loop bounds, so a mesh Boolean
  bounds where its true patch lies from its vertices' bounds, its edges' lengths and the stamp, and keeps its tag
  as an oblique plane (faceted-source-retention §3.2, A2, Claim F). A circular patch states none.
- DX7 widens its own window reading by that bound. A point proven to oppose
  lists the patch; only an all-clear needs every point to clear. For the tagged
  normal-component range `[mn, mx]` and allowance `allow`, it lists when
  `mn + allow < 0 && mx - allow > -1`, clears when
  `mn - allow >= 0` (perpendicular included) or `mx + allow <= -1`
  (antiparallel included), and is undecided otherwise — the same rule
  `agent-docs/verification-design.md` §6 states, at both ends. An undecided patch
  does not remove other patches already proven to oppose. Every point of the patch
  carries an azimuth inside the window, which is what makes each proof about
  the patch rather than about the cone. `allow` is TWO terms, and the departure
  is only one of them: DX7 reads each patch's normal through `Face.NormalAt`,
  and a reading so taken departs from the patch's own exact normal model three
  ways — the arm's own arithmetic (`internal/surfacenormal/normal_bound.go`), the displacement of the
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
departure (`internal/capblend/band.go`'s `f.n.NormalBound`, derived in
`internal/capband/departure.go`, plus `capband.DenotedNormalAllow`) is an exact
zero does — an axis-aligned `Plane` patch of an unplaced band whose cap contour
and levels are held exactly reaches that, and nothing else does. `Face.NormalAt` and DX7
already read that same stamp; DX8 reads it too, rather than assuming a
coinciding window buys back what only a zero stamp proves. A proven curvature
bound over the built ruled patch's own held numbers could answer DX8 for a placed
or whole-turn band without the tag; that route is not implemented.

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

##### 8.3.1 Two distances

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
(`MagnitudeInBounded`). The side level charges `ds`'s rounding as `levelDelta`.
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
(`capblend.AuditSetbackSpan`). A section that passes at `dc` and fails there is
`ErrUnsupported` whichever sentinel the second run raised, since the stated
setback lies somewhere in the span and this evaluator cannot decide which
side of the failure it is on. The contact floor (`1e-9` of the section's
diameter) is many orders of magnitude above a real conversion's rounding, so the second run
changes the answer only where a corner's miter foot moves faster than that
ratio per unit of offset; it is run rather than argued away because that
speed is `1/sin(α/2)` at a corner of angle `α` and has no bound.

The equal-setback band charges `d`'s rounding the same way. A millimetre
setback converts exactly, its span is the single point `dc`, the second audit
does not run, and every reading is the one `dc` alone gives.

With no option, or with `otherDistance` equal to `d` in the same unit, the
band reads `dc = ds = d` and builds the same body bit for bit.

##### 8.3.2 Whole runs of a cap loop

A `Chamfer` that selects part of a cap loop's edges, which routes E and L above refuse (SX4), takes this route when the
selection is a set of whole runs, on a straight prism or on a faceted body (a mesh Boolean result) whose face is a
retained plane. A run is a maximal chain of selected edges, lines and circular arcs, that continue one
another tangentially. Every junction between a selected edge and an unselected one is a sharp corner, so no run stops
inside a tangent chain. `WithTangentChain` makes such a set from one seed edge. A loop selected whole stays with the loop
routes. A prism's cap edge always meets its wall at a right angle, so this route does not read `Edge.Convex`, which on a
prism is only the rim's two-dimensional turn: a loop with clockwise (concave) arc walls outside the selection is read
like any other.

Every wall beside a chain is a plane normal to the cap or a cylinder with its axis normal to it. A chain is one maximal
set of selected segments, so segments that meet at a sharp corner mitre inside one cutter, as the whole-loop chamfer
mitres them. The cutter is the region within `t` of the chain, inside the cap, above the bevel plane that leaves the wall
at depth `dc` and the cap at `dc`. A moved arc is swept between its own end points, which a mitre moves off the source
radials. A moved arc whose end points the mitres have turned past each other is `ErrUnsupported`. Each end of a chain is
read on its own, with `over` the cutter's overshoot above the cap and outside the wall (the margin) and
`reach = dc + 2 over` how far the cutter reaches into the material at its top plus that margin; one `reach` serves the
extension and the probe of its end:

- A convex end turned by 45 degrees or more reaches past the end, so the bevel ends on the wall beside the next edge,
  where it runs out onto that wall over `dc` times the cotangent of the turn. The cutter reaches
  `max(dc, 1.5 reach cot(turn))`. A line run's cutter reaches along the run's tangent. An arc run's cutter follows its
  own circle, moved with each ring, past the end: the same angle and the same chord count (read at the largest ring's
  radius) on every ring, so the rings pair vertex for vertex, and the bevel runs out as the cone itself does. The angle
  is the first one at which the cutter's end face, the radial segment between the ring moved out by `over` and the ring
  moved by `dc`, lies `over` past the next wall's line on its void side, where the cutter crosses void alone, solved in
  closed form from the two radii. Both ends of one arc stop there, so they do not reach each other across a slot. An arc
  whose bevel circle at the cap does not get `over` past that line within a half turn, a convex arc of radius `R` with
  `R - dc` below the next wall's distance from its centre plus `over`, has no run-out onto the wall (the cutter would
  carve a sector of the wall's material and leave a step) and is `ErrUnsupported`. A turn under 90 degrees needs the next
  edge at least `1.5 reach / sin(turn)` long. The run-out removes `dc^3 cot(turn)/6` beyond the perpendicular end's wedge (a
  negative amount past 90 degrees).
- A concave end turned by up to 90 degrees beside a line stops the bevel on the next face's plane, carried back over
  the run's band. The cutter loses a sector of a disc about the corner, bounded by the next wall's line carried back
  over the band and by the bisector of the corner's empty wedge, so the bevel tapers toward the corner's vertical
  edge, shortens at the cap by `dc / sin(turn)` along that plane, and leaves no square end. A triangle coplanar with
  the next face closes the bevel's end; the mesh result holds it as a face of its own beside the next face, and
  nothing is removed on the neighbour's side. A turn of exactly 90 degrees stops the bevel on the corner's
  perpendicular. The result removes `dc^3 cot(turn)/6` less than the perpendicular end's wedge. The sector's radius is
  `S = 1.5 (reach/sin(turn) + dc + over)`, local to the corner. The cutter lies within `reach` of its run, so a point
  of the chain whose band of that width meets the sector is refused unless it lies in the smooth stretch leaving the
  corner within `reach / sin(turn)` of it along the chain, where the taper is. That refuses a stretch past the first
  sharp corner, a smooth stretch that doubles back, and an arc that curls onto the next wall's plane carried back. For
  an arc at the corner the clip stops at the foot of the next wall's line, so its radius is `min(S, R sin(turn))`,
  where `R sin(turn)` is the distance from the corner along that line to the foot of the perpendicular from the arc's
  centre. Past the foot the line turns back toward the arc, and the clip does not reach it. The plane must also meet
  the bevel's circle at the cap (radius `R - dc`), that is `R - dc` must not be below the plane's distance from the
  centre; otherwise it cannot cap the chamfer and the run is refused. Where `S` is the smaller radius the clip keeps
  it, and the plane's second crossing must lie farther than `S + reach` from the corner. Without more, the clipped
  cutter lies flush with the next wall along the corner's vertical edge. The mesh Boolean cannot classify that touch,
  and its gap shift (gap-tolerance §5.2) moves the cutter outward off the next wall's plane. The moved clip face then
  crosses the run's wall at the corner's turn, and beside an arc the rim there divides the wall's chord bound by
  `sin(turn)`, which is almost six times at a turn of 10 degrees. So on a prism whose document's gap
  tolerance is at least 0.004 mm (`partialCornerRelief`), the clip also holds a polygon inscribed in the circle of
  that radius about the corner, one chord per sixteenth of a turn over the angles the sector leaves out. The cutter
  then stays at least `0.004 min(cos(turn/2), cos(c/2))` mm from the next wall, `c` the angle each chord spans, more
  than the two faces' bounds, and the Cut needs no shift. The bevel stops short of the corner's edge and keeps
  material within 0.004 mm of it, at most `0.004^2 turn dc / 2` mm3 per corner (`turn` in radians), 6.3e-6 mm3 at a
  right angle and a setback of 0.5 mm, for a line run or an arc that curves toward the material (an arc of radius `R`
  curving away keeps up to `0.004^3 dc / (6 R)` more), and a setback under about `0.004 sin(turn)` loses its whole
  taper to it. That relieved chamfer is the result's true solid, a use of the gap tolerance like the shifted retry's
  (gap-tolerance §5.4): its bounds, tags and edges describe it, and no charge is added for the relief, so its bounds
  do not cover the chamfer without it. Against the chamfer without the relief, the edge where the run's wall meets the
  next wall runs up to the cap instead of stopping at the bevel, and the relief adds faces and edges about `dc` tall
  and at most 0.004 mm wide. A document whose gap tolerance is under 0.004 mm, an exact-only one included, and a
  faceted receiver take no relief; their Cut takes the gap shift where the tolerance allows it.

Any other end is `ErrUnsupported`: a convex end turned by less than 45 degrees, a concave end turned by more than 90
degrees, a concave end beside an arc whose next wall's plane misses the bevel, a run-out longer than the next edge, an
arc run whose extension turns more than a half turn, and a chain that comes back into the end sector. So is a chain
whose extension would meet material of the cap, within `reach` of the run's end (the region past the end, less the
run-out zone beside the next wall, which is the strip a line end reaches or the annular sector an arc end reaches, is
tested exactly against the loop's boundary, read within 1e-6 mm of its arcs: no boundary edge may have a stretch in
it, and where none does, one point of each cell of the region must lie in void, so a feature of any thickness is
seen), or whose margin strip beside the run (the `over` outside its wall, except within a concave end's sector and
near a corner inside the chain) holds material of another wall of the loop outside a concave end's sector, or whose
offset empties an arc. The overshoot list retries a smaller `over` after such a refusal, so a slot narrower than
`over` is cut with a smaller margin when that clears it.

A hole loop of the cap is read the same way as an outer loop: its edges chain, mitre and end like any other, and a
reflex corner of the material inside a chain (a hole's corner) mitres by removing `dc^3/3` more than the two wedges.

On a faceted body the run lies in any retained planar face, and a face that holds the most selected edges is read, and a selection that face does not read is declined.
The selected edges and the edge beside each end must be exact lines or arcs (`Line3`, `Arc3`), convex, with a wall that
is a retained plane normal to the face or a retained cylinder whose axis is normal to it; the edge beside an end
that carries no exact curve is `ErrUnsupported`. Every other edge of the loop is read for the probes' polygon alone, as
its curve, or as the chain of held boundary vertices between its end points when it has no exact one (a `FacetedCurve`),
and needs no wall. A loop selected whole stays with the loop routes.

The polygon holds the face's own material, so on a faceted body it cannot see a feature that stands in the cutter's
band from another face. Every held facet of a face other than the run's own faces must miss the cutter
(`internal/brepmodify/partial_loop_clear.go`). The own faces are the loop's face, both faces of each selected edge and
both faces of the edge beside each end, where the bevel runs out onto the next wall or stops on its plane. The test is
exact and refuses only: no such facet may touch a facet of the cutter (the cutter after any end clip, so the sector a
clip removes is not tested), by meshbool's exact triangle classification on the held floats; a facet within its
vertices' bounds plus the cutter's bound of a cutter facet refuses; a facet that crosses nothing may lie wholly inside
the cutter, so one corner of each is placed by the exact parity of a ray through the cutter's mesh, and a corner inside,
on its boundary or undecided refuses. A facet with collinear corners near the cutter refuses. A feature of another face
in the band is therefore `ErrUnsupported`: a pin standing in a slot beside the run, a lug through a wall, or a blind
cavity whose wall comes within the bevel's reach.

The cutter is a polyhedron held exactly as a mesh solid, with arc chords within 0.0002 mm of their circle, and the
result is a Boolean Cut: faceted, with the Cut's own readings. A chain with two rings (a top and a bottom) holds the
bevel exactly except where it holds a sharp corner beside an arc. There the offset's corner moves along a conic with
the setback, and an arc's cells twist between rings, so a straight rule between two rings leaves the bevel. Such a
chain steps through `L` layers of rings, `L` stepping up from 2 by one to the first value at which `layerDeparture` is
within 0.0002 mm, and refuses past 32. A value that puts an inner ring within a quarter of a layer of the wall's
level, `dc` below the cap, is passed over (`ringNearWall`). A ring at that level holds the run on the wall itself, so
the cells below it touch the wall along the ring, outside the material. Near a concave end the end clip's Boolean
leaves those cells as a face of their own that touches the wall without crossing it, the Cut refuses the touch, and
its gap shift moves the clip face across the run's wall at the corner's turn. With an overshoot of half the setback
the wall's level lies six sevenths of the way down, so every seventh value puts a ring on it, and the values beside
those are passed over too (6 to 8, 13 to 15, 20 to 22 and 27 to 29); the overshoots of 0.35 and 0.7 pass over other
runs, and none passes over 32. A quarter layer is about 0.01 mm at the counts chamfers take, several times the gate's
slack of about 0.001 to 0.002 mm, and a shortfall only brings back the shift. Every chain with layers takes the rule,
whatever its ends. A column is the path of one ring point as the setback changes. Only a sharp corner beside an arc,
and the samples of an arc that ends at one, move off a straight line; every other column is straight, and two
neighbouring straight columns bound a planar cell, which the two end rings hold exactly. So only a column that bends
and the columns beside it carry the rings between: a cell between two such columns takes `2L` triangles, a cell
between one of them and a column of two vertices is a fan of `L + 1`, and every other cell is the two triangles of the
two-ring cutter. The bound is the same, since the cells that keep every ring are the ones `layerDeparture` bounds.
`layerDeparture` is closed form, the largest over the corners and layers of two terms. The first is the corner path's
bound `B^2 dt^2 / (32 v^3)` for a line and an arc (`v` the corner's distance along the line from the foot of the arc's
centre, `B` the rate of `v^2` per unit of setback, which is constant) and `|4ac - b^2| dt^2 / (32 y^3)` for two arcs
(`y^2 = a + b t + c t^2` the squared distance from the line of centres), each at its smallest `v` or `y` over the
layer. The second is the twist, with `dphi` the turn of the corner about the arc's centre over the layer: a cell's
diagonal spans that turn and the arc's chord step together, so its sag is `rho (step + dphi)^2 / 8`, and the twist
charges `rho dphi^2 / 8 + 2 sqrt(0.0002 rho dphi^2 / 8)`, the part of it the sagitta does not already hold. A corner
that reaches the arc's foot, or an arc-arc corner that changes sides of the line of centres in a layer, is
`ErrUnsupported`. The cutter's bound is 0.0002 mm plus that departure. On a concave arc wall (material outside the
circle) the faceted result reads more volume per arc run than its exact value, so a test of such a run reads a section or
allows that excess. A concave end's
clip is a second mesh Boolean; without a relief its gap shift leaves a sliver on a straight edge, and with one
the relief keeps what it states above. `internal/brepmodify/partial_loop_chamfer.go` owns it. It runs after every
exact route has declined, and only for a symmetric chamfer.

#### 8.4 Measurements + tessellation

**Displaced receiver (`capblend.DisplacedPayload`).** A cap-loop chamfer of exact loops of a prism whose recorded section is displaced from the one it denotes stores a `DisplacedPayload{Base, LoopDelta}` (carrier-record-design §8). `LoopDelta[l]` is the receiver record's `h_loop` of loop `l`, zero on every selected loop. `capblend.BuildDisplaced` charges each displaced loop's walls, rim vertices, wall lengths, enclosed area, in-plane first moments, both cap areas and the box through the same builder as `Build`. `ExtentBoundedAlong` adds `LoopDelta[l]` times the in-plane norm of the direction to that loop's candidates and widens the placement envelope by the largest entry. `ExtentAlong` refuses with `ErrUnsupported`, so a through-all stop with the body live refuses. A mesh reserves `max LoopDelta` from the chord budget, adds each loop's bound to its facet bounds, and charges the tube area to area slack and occupied volume (tessellation-reach §7). The type is not `Payload`: readers that do not name it give their answer for an unknown payload class.

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
against the centre's distance from the plane-local origin. Far from the origin a
published bound can then miss the exact value. So each patch carries a
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
window's allowances on both directrices, and the built patch it reads keeps
all of its own. The area's frustum sector charges
the side radius allowance as `αc·e·(2·(R0+R1) + |H| + e)`. Its corner-skew
term is not monotone in the side radius, because the slant shrinks as `R0`
grows toward `R1`, so it reads `R0 + e` and the held slant plus `e` (the
slant is 1-Lipschitz in `R0`) and bounds the term over the whole allowance.

At the references every patch meets both disks along its directrices, and two
patches meet along a corner ruling only to within a gap: the held ends two
walks meet at need not be one float, a walk end names its denoted point only
to within its end bound, an `ArcSeg`'s `End` need not lie at its `Start`'s
radius, and a cap arc is recorded through one foot while the other sits at its
own distance. `capband.Closure` (`internal/capband/closure.go`) sums those gaps per
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
A circular wall's carrier, offset radius and corner-foot locus start from
every radius its record denotes: the walk's held radius widened by its `RadiusBound`, since an
`ArcSeg` walk holds the `math.Hypot` of `Start − Center`.
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
terms with the extremized candidate (`agent-docs/evaluator-design.md` §5), so a
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
(`agent-docs/prism-boolean-design.md` §7's `sectionDelta` identity). Four readings
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
  `BandPatchAreaAllow(delta, chordUpper, slantUpper)` — a ruled quad's area is
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
(`capblend.PatchWindowSkew`, the larger of them): the chord-versus-locus volume and
region terms (§8.3), the mesh's `skewGap` and the gate on its per-cell twist term
(`agent-docs/tessellation-reach-design.md` §7), and DX8's undecided gate. None reads
a difference of the held windows `th0`, `th1`, `capTh0`, `capTh1`.

A band's SIDE level is displaced too, and by a different mechanism, so it is a
separate term with its own helper. `sideZ` is the single float sum
`capZ + matSign*ds`, so the whole side directrix translates rigidly by that
sum's own rounding (`levelDelta`) rather than moving point by point the way a
solved contour does. Every reading built on that level charges it: a slant
edge's own length, the band volume and first moment (`capband.LevelVolume`:
the body moves only where a vertical line leaves the band toward the cap,
over at most the larger of the side loop's and cap contour's areas), and
each BAND PATCH's own area, which
composes `BandLevelAreaAllow(levelDelta, directrixSumUpper)` — under a rigid
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
denoted one has at most the region term's measure (`ChordLocusRegionAllow`
divided by 3). Every point of it lies within the band's coordinate envelope (the
original loop, the cap boundary widened by the contour displacement, and both
levels, each arc read over the angle it sweeps rather than its whole circle:
`capband.CoordUpper`) plus the patch's radial gap `|R0 − R1|`, so each moment component's
bound grows by that volume times that reach (`capband.ChordLocusVolume`).
The centroid divides the summed first moment by the body's own volume and
lifts the plane-local quotient to world through the same frame/placement lift a prism centroid
uses, with the geometric safety-net bound (the true centroid lies within the
body's own `Bounds` box) standing as a `math.Min` ceiling on the formula
answer, never the whole bound.

#### 8.5 Free-form rim chamfer

A selection covering the whole outer loop of one or both caps of a straight prism whose section holds fit
splines cannot take the exact cap-loop band above: its contour offset is not representable (spline-design R5).
`Chamfer` builds it instead as a Boolean `Cut` of the receiver by a cutter, which is the only inexact chamfer
here, and which `brepmodify.FreeformRimChamfer` (`internal/brepmodify/freeform_rim.go`) states as follows.

| Condition | Answer |
|---|---|
| receiver is a plain `Extrude` prism (no placement transform, no section or level displacement) whose outer loop holds a fit spline | the route below |
| selection is every edge of a cap's outer loop, for one or both caps; equal setback; no `WithAsymmetricChamfer` | the route below |
| any other receiver or selection | the ordinary routes and their refusals (SX4, the free-form refusal of `offset2d`) |
| the loop holds a segment that is not a whole fit spline or whole line | `ErrUnsupported` |
| a joint between two segments turns by more than 10° (`rimoffset.MaxJointAngle`) | `ErrUnsupported`, naming the corner |
| the setback, plus its overshoot, reaches a radius of curvature of the outline | `ErrDegenerate` |
| the setback reaches the far end (`d ≥ h`, or `2d ≥ h` with both rims) | `ErrUnsupported`, as SX7 |
| the offset splines depart from the exact offset by more than `rimoffset.MaxDeviationRatio · d` | `ErrUnsupported` |

Per selected rim, with setback `d`, overshoot `o` and the outline's offset by `u` written `C(u)` (positive into
the material):

1. `internal/rimoffset` builds `C(d + o)` and `C(−o)`. A line moves exactly. A fit spline is sampled in
   the chord-length parameter of its recorded fit points, every one to a few spans apart, and the new fit
   spline passes through the sample moved along the curve's left normal, with the joints between segments moved
   along the bisector of their two normals so the loop closes. The samples lie off the recorded fit points on
   purpose: a cutter whose rulings cross the receiver's mesh edges exactly sits within rounding of them, and the
   Boolean refuses the result as folded (evaluator §9).
2. The cutter is the `Loft` of two rings. Each ring lies between a circle about the outline's box and an
   offset of the outline. The first ring is at the level `o` beyond the cap and holds `C(d + o)`; the second is at
   the level `o` beyond the bevel's far edge, `d + o` from the first ring's level along the sweep, and holds
   `C(−o)`. The lofted wall between the two outlines is the bevel: it runs at 45° and crosses the cap's level at
   `C(d)` and the receiver's side wall at `d` below the cap. The circle makes the loft chord at a coarse target,
   which the loft's station cap needs, and its radius, `scale` times the outline's farthest point from the box's
   middle, sets the pair diameter the Boolean's chord tolerance follows. With both rims selected, the second
   cutter lofts the first one's two rings at the other cap. Its records match the first cutter's bit for bit, so it
   takes the first cutter's area falsifiers and pairing from `loftrecord.PairingReuse`. It runs again only the
   coplanar-plane gate (loft S5) and the build steps that read the planes.
3. One `Cut` removes it, and with both rims selected one `Cut` removes both: the two cutters lie at the two ends
   of the sweep and touch nothing, so their `Union` is one tool that holds both. Two `Cut`s would cross the second
   rim's wall facets after the first had left them carrying its rim bound. Each rim crosses the wall at 45°, which
   amplifies the bound it meets, and the wall's facets run the whole sweep, so the second rim would read a larger
   bound than the first. One `Cut` meets fresh wall facets at every rim and holds the one-rim bound. The cut's
   result is held as a mesh, so its readings and bounds are the Boolean's own.

Which `o` and `scale` build depends on the shape. `rimAttempts` tries `(0.5d, 1.5)`, then `(0.35d, 1.5)` for a
Boolean that refuses a cutter close to the receiver's edges, then circles of 2 and 3 times for a loft whose
station cap or work ceiling a tight ring exceeds; a refusal of the cutter or of the Cut tries the next, and a
refusal of the offset does not. A larger circle holds the body coarser: a Cut's chord tolerance follows the
pair's diameter, and the cutter's circle sets that diameter.

**Exactness.** The route proves nothing about the chamfer being the ideal one. `rimoffset.OffsetLoop` measures,
at the middle of each step between samples, the distance from the exact offset to the offset spline, and refuses past
`MaxDeviationRatio · d`. A small measurement proves nothing, so the check can only refuse, never admit (CLAUDE.md,
hard rules). The body that results is exact for what it is, the receiver minus a
cutter. What it departs from is the ideal bevel, and the caller owes the same statement it owes for a chorded
section (spline-design §4): a proof over this body proves a chamfer within the stated ratio of the one intended.

The result carries no chamfer role and no band record, so a further modify op on it is the refusal of any Boolean
result, and `Verify` reads it as a faceted body.

### 9. Shell reach

#### 9.1 `stackedPrismPayload`

General prism shells are a finite axial stack of exact line/arc regions. One
axial slab may contain several disconnected regions. `agent-docs/prism-boolean-design.md` Part S
owns the payload: its internal `stackedrecord.Slab` and
`stackedrecord.Interface` records, the root `stackedPrismPayload`, its
invariants, its body build, its measurements, its
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
- emit one `Lump` per connected component and one void `FaceShell` per enclosed
  cavity;
- mint roles from slab + region + result-record indices.

Mass properties are bounded sums of all region-prism integrals. Bounds compose
from the slab-region bounds. Tessellation chords a section curve once per
shared carrier and triangulates exposed planar differences.

A cup's record is this payload. A cup over a section with `k` holes is one
floor slab containing `P` inward or `Q` outward, plus one wall slab containing
the outer band and `k` hole-lining bands as separate regions. Every wall region
has positive-area overlap with the floor region, so the component graph proves
exactly one lump. The interface partition — `agent-docs/prism-boolean-design.md` Part S
§2.2's lining reading — emits the remaining cavity-floor face once, and records
it as the cavity region itself.

The cup keeps its own payload type, `cupwall.Payload` (`internal/cupwall/payload.go`), which holds that stacked
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

#### 9.2 Side-opening section

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
`FaceShell.IsVoid()` is true only on the inner shell.

`prismshell.BuildClosed` (`internal/prismshell/cap_shell.go`) builds it as
three slabs: inward `P` on `[z0, z0 + t]`, the band `{P, reverse(Q)}` on `[z0 + t, z1 − t]` and `P` on
`[z1 − t, z1]`; outward `Q` on `[z0 − t, z0]`, `{Q, reverse(P)}` on `[z0, z1]`
and `Q` on `[z1, z1 + t]`. Both interfaces are monotone
(`agent-docs/prism-boolean-design.md` Part S §2.2): the band's one hole is upper-only at the
first and lower-only at the second, and each exposes the cavity region (`Q`
inward, `P` outward) as the cavity's floor and ceiling. Each derived level
carries its source end's displacement, the thickness conversion and its own
float sum's rounding, and denotes the source end's rational moved by the
exact thickness, as a cup's floor level does (modify §9). The end caps keep
the receiver's denotations, so a `ToFace` stop on either cap, the floor or
the ceiling records its exact level. The offset loops'
proven displacement is the stack's `sectionDelta`. The gates run in the order
S18, S10's section limit, SX11 (`t` below `h/2`, inward), S11a, the §5 audit.
The cavity's walls, floor and ceiling form a connected face set touching no
outer wall and no end cap, which the stacked build reads as the void shell of
the one outer lump. Roles are the stack's own: outer walls
`slab(0).region(0).side(0,j)`, cavity walls `slab(1).region(0).side(1,j)`,
`capStart`/`capEnd`, the floor `floor(0,0)` and the ceiling `ceiling(1,0)`.

A section with `k >= 1` holes builds in either sense, with `O` the outer
region and `C` the cavity region (`P` and `Q` inward, `Q` and `P` outward).
Its middle slab holds the `1 + k` wall bands `ShellWallBands` lists
(`{O.Outer, reverse(C.Outer)}`, then per hole `i` the lining
`{reverse(C.Holes[i]), O.Holes[i]}`); each interface takes stacked-prism
§2.2's lining reading, and each exposes `C`, a ring with `k` holes, as the
cavity's floor or ceiling. The cavity wraps every lining, so its void shell
holds `C`'s outer walls, each lining's outer cylinder or wall, the floor and
the ceiling. A lining's outer column starts and ends at an interface and faces
the cavity, so the record carries `ClosedShell` (stacked-prism §2.4) and that
column anchors no outer shell. The built body must read as one lump of one
outer shell around one void shell; any other reading is `ErrUnsupported`.
The volume is the slab sum: `area(P)·h − area(Q)·(h − 2t)` inward and
`area(Q)·(h + 2t) − area(P)·h` outward. A hole the erosion makes meet the
inset outer loop changes the cavity's topology and is the offset audit's
refusal (S11).

**The outward closed shell of a holed section.** The denoted solid is
`(Q × [z0 − t, z1 + t]) \ (P × (z0, z1))` with `Q = P ⊕ t`, base §8's
outward offset: the cap slabs B6 grows below a kept cap, on both ends. The
record above denotes it, by these steps:

1. `Q` is the per-loop offset. Write `E` for the open exterior of `P`'s
   outer loop and `H_i` for the open region of hole `i`. The complement of
   `P` is the disjoint union of the open sets `E` and `H_i`, and an open disk
   of radius `t` is connected, so it lies in that complement exactly when it
   lies in one of them. Hence `P ⊕ t`'s complement is
   `(E ⊖ t) ∪ ⋃ (H_i ⊖ t)`: the outer loop dilates on its own, and each hole
   erodes on its own, its walls moving `t` into the hole. `OffsetProfile`
   offsets each loop in its own walk sense with `s = −1`, which is exactly this
   split: a convex corner of the material rounds with an arc of radius `t`,
   whether it sits on the outer loop or is a hole's reflex corner, and a
   reflex corner of the material miters (base §7). Each loop's per-feature
   offset is that loop's true offset while it keeps its feature set, which
   S11a (at construction) and the §5 audit's S8, S11b and S9 (after it)
   decide on the built section, the same gates B4 and B6 run on an outward
   holed section.
2. A hole that the offset empties drops a feature: a circle whose radius `r`
   is at most `t` (`OffsetRadius`), or a polygonal hole narrower than `2t`,
   whose trimmed walks run backward (`WalkConsumed`). Either is S11a,
   `offset2d.ErrDrop` (`ErrUnsupported`). The body exists — the hole fills
   in, and the cavity, the whole of `P`, keeps its hole — but its outer shell
   loses the through hole, a feature set this record does not hold.
3. Outward, holes cannot meet each other or the outer loop. `H_i ⊖ t ⊂ H_i`,
   and `E ⊖ t ⊂ E`, so the offset loops lie in pairwise disjoint regions that
   `P`'s own loops separate. Two outer-loop walls that close on each other
   across a notch narrower than `2t` are the one merge the dilation makes. A
   notch whose walls the trim runs backward is S11a (`WalkConsumed`) as the
   offset is built; one that survives the trim is the audit's crossing test,
   S11b. A hole's own erosion can still change its topology: a hole whose
   narrow waist is under `2t` wide splits into two (a dumbbell), its offset
   walk crossing itself at the waist, and that is S11b too.
4. `Q \ P` is the `1 + k` bands. `Q \ P = (D \ D_P) ∪ ⋃ (H_i \ (H_i ⊖ t))`,
   with `D` and `D_P` the closed regions inside `Q`'s and `P`'s outer loops:
   the outer band `{Q.Outer, reverse(P.Outer)}` and, per hole, the lining
   `{reverse(P.Holes[i]), Q.Holes[i]}`, pairwise disjoint by step 3.
5. The interfaces read. At each interface the cap side holds `Q`, one region
   with `k` holes, and the wall side the `1 + k` bands: the first band's outer
   is `Q.Outer` and band `m >= 1` has `Q`'s hole `m − 1` as its one hole
   (lining I5); the exposed record is `P` (I6, I7).
6. The faces close as the denoted boundary. `Q`'s outer loop and each
   `Q.Holes[i]` are one column over all three slabs (the body's outer wall and
   its through-hole walls); `P`'s outer loop and each lining's outer
   `reverse(P.Holes[i])` are columns of the middle slab alone, and with the
   floor and ceiling exposing `P` they bound `P × [z0, z1]`, the cavity. The
   lining columns start and end at an interface, so `ClosedShell` keeps them
   out of the exterior anchors, and the lump reading must be one outer shell
   around one void shell (`requireOneCavity`); tessellation's per-shell
   winding audit falsifies a reading that disagrees with the facets.

The offset loops are the float evaluation of these shapes at the held
thickness, and the stack carries their proven displacement as its
`sectionDelta` (base §9), so a curved or rounded record is `Sound` with a
bound, not `Exact`. No gate is new: S11a, the §5 audit and
`requireOneCavity` are the ones the inward record and B6 already run.
A stack enclosing a cavity has no brep face view, so a later modify op on the
closed shell is Part B SB2.

**The side opening is Part O's.** The normal segment
above is that document's rim rule at a right angle only (its §2.2), and the
cap slabs and the middle slab hold different loop records along `K`, which no
stack column carries as one face (its §1). That document owns the rim at
every corner kind (Table RO), the three regions, the `brepPayload` the result
is recorded on through `internal/stackedbrep`, its refusals (Table SO), its
consumers (Table DO) and its PR split. A prism side opening builds where
every section walk is a line or a circular arc (`internal/brepmodify/shell.go`), and
its increment table lists what still refuses. A revolve side opening (§9.3.2)
builds with that document's rim at every line and arc corner (its §8).

For cap-only removal from a holed section, build the wall as one slab with
`1 + k` regions: the band between the paired outer loops first, followed by one
band between each paired hole loop in `ProfileRecord` order. The base
S18/S10/S11/§5 gates prove those bands are regular and pairwise disjoint before
payload construction. The slab is a prism group (`agent-docs/prism-boolean-design.md` Part S
§2.2) whose disjointness is that audit's, not a scene-of-outers disjointness proof's: a
hole lining lies inside the outer band's own hole, which a scene of outers alone
would read as nesting. The offset loops sit within the offset displacement of
the offset the thickness denotes, and the group carries it as its
`sectionDelta`. The resulting `1 + k` connected components lift base S12
without admitting holed side-opening shells.

#### 9.3 Revolve shell

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

##### 9.3.1 Construction

`revolveshell.Shell` (`internal/revolveshell/shell.go`) builds the partial turn without a side opening and the
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
reading charges it (`agent-docs/surface-intersection-design.md` §7.2). The figure is
modify §9's `offset2d.OffsetSectionDelta` argument: three times the largest reach of a
recorded join point from its rational enclosure, over every denoted thickness
within the thickness's own conversion bound. A wall off the axis reads the
prism cup's `offset2d.OffsetSectionDelta` over the meridian's closed offset unchanged.
A wall with an axis end or an opening end reads `offset2d.ChainReach` over the
open chain `K`: an interior corner by `LoopReach`'s own enclosures, an opening
end by its rim cut (`offset2d.OpeningReach`, Part O
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
guard (`revolveplan.RequireExactSection`), a holed meridian (SX8), a kept angular
cap (SX8), more than one on-axis walk (SX8), then a removed side run that is
not one proper connected run of whole walks or that leaves two kept chains
(SX8, §9.3.2). `WithNoOpenings` on a partial turn or on a holed meridian is SX8 before the shell is routed here.

##### 9.3.2 Side opening

`revolveshell.SideWall` (`internal/revolveshell/wall.go`) builds the side opening of a full
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
end takes Part O Table RO's rim: the removed
neighbour walk's own carrier from `K`'s end to its cut with `K'`
(`offset2d.OpeningJoin`, `internal/offset2d/opening.go`), a `LineSeg` or an
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

### 10. Table BX — results + roles

| BX | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BX1** | prism lateral fillet/chamfer | base `prismPayload` | base B1 | base roles |
| **BX2** | revolve junction fillet/chamfer | `revolvePayload` over rewritten meridian | existing full/partial revolve topology | `side(i,j)` + blend role on inserted wall |
| **BX3** | complete prism cap-loop fillet/chamfer | `capBlendPayload` | trimmed cap/sides + analytic blend patches | result side/cap roles; `filletCap(c,l,p)` / `chamferCap(c,l,p)` per patch |
| **BX4** | prism shell with side opening | Part O Table BO: `prismPayload` over the wall section with both caps removed (BO1), else `brepPayload` (BO2) | one lump, one shell, no void | BO1: `side(0,j)`, `capStart`/`capEnd`; BO2: `face(k)`/`wall(k)` |
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
the shipped behavior. Section 10.1's rebuilt Boolean is a separate faceted
route: its source face groups retain their authenticated origins, as the
ordinary Boolean evaluator does.

#### 10.1 Isolated outer edge of a faceted prism Union

A mesh Union of two real prism sources retains those immutable sources until
the Union is placed or otherwise rebuilt. `Fillet` or equal-setback `Chamfer`
accepts one selected
faceted edge only when its zero-bound endpoints equal one complete lateral
edge of a source's exact, unplaced, axis-aligned rectangular prism. The two
faces next to the selected edge must carry exactly the two planar source-wall
roles next to that source edge. The source's section and both sweep levels
must have zero displacement. The other source is a prism with certified
bounds; it may be placed or rotated.

The radius or setback has zero conversion displacement and is smaller than
half each rectangle side. The existing section rewrite and audit create the
blend. A fillet additionally checks that its connector is the exact inward
quarter circle: its centre is one radius inside each side, its feet lie one
radius along the source walls, and its directed arc traverses that quadrant.
A chamfer checks an exact whole line between feet one setback along those
walls, directed through the same inward corner. A centre or foot that cannot
be represented exactly makes the corresponding route refuse. These checks
prove the rewritten source lies inside the original prism and differs only
in the corner square swept through the prism's full height.

The corner-square box is rounded outward from exact rational coordinate
sums. It must be strictly separate from the other source's true body box
after that box's stated bound. Thus the removed material does not intersect
the other source, and the modified Union equals the Union of the rewritten
source with the unchanged second prism. The removed volume is
`height × radius² × (1 − π/4)` for a fillet or
`height × setback² / 2` for a chamfer; the other source cannot replenish it.
The evaluator builds the source blend without committing it, then reruns its
ordinary Boolean evaluator.
That evaluator supplies the final one-lump boundary, occupied-volume bound,
measurements, and `VerifyAll` mesh proof. One document commit retires only
the original Union. A failed selection, non-isolated radius band, displaced
section, or placed Union falls through to the ordinary receiver-class S3
refusal, preserving its selected-edge context.

### 11. Exactness + proof discipline

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

`facetedPayload` stays excluded even when `MeshBound == 0`, except for RX6's
authenticated routes. Its groups retain provenance and a planar flag, not
general analytic carrier or trim intent. The loft/cylinder cap route reads
the immutable Union or Bore Cut source record and requires both complete
outer cap loops. It denotes one rational polyhedral cutter built from offset
samples of the certified held cap polygons. That same cutter applies to the
true source and held joined solid. The inset sampling bound charges deviation
from the requested held-polygon offset, and the published boundary and volume
bounds charge rounding of the final mesh. No smooth-spline offset is denoted.
The mirrored-bore case rewrites one exterior cap on its audited lower half,
reflects that result, and removes the two matching middle annuli. It proves
the cap, bore wall, and seam separately. For a near-axis tooth placement,
the upper cap proof checks split rings against the translated original teeth
and adds the inherited bridge reach and swept-volume charge.
The perpendicular cylinder Cut retains two full source circles for a
separate one-mouth chamfer (Part X).
Section 10.1 admits one isolated rectangular-prism Union outer-edge fillet or
equal-setback chamfer.
The nested rectangular-boss Shell route accepts a complete boss top cap from
one authenticated prism Union and rebuilds a bounded faceted cup
(Part S). Other unrecognized faceted cases remain SX9.

**What SX9 leaves a caller proving.** Outside RX6, a caller whose
part fillets or chamfers a faceted boolean result cannot reach the modified
solid here at all, and waiting is not one of the options (an analytic
`brepPayload` result is Part B's and is admitted there).
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
reads `Suspect` (the faceted surveys that would answer it are
`agent-docs/payload-verification-design.md`'s design and are not implemented) while
asked on an operand carrying an analytic payload, a prism or a cup, it is
answered outright. An operand that is itself a boolean result carries a
`facetedPayload` and leaves the same question undecided, so the substitute
reaches only the analytic bodies at the start of a chain.

### 12. Table DX — downstream

| DX | Consumer | Revolve rewrite | `capBlendPayload` | `stackedPrismPayload` |
|---|---|---|---|---|
| **DX1** | mass properties / bounds | existing bounded path | bounded analytic patch integrals | bounded slab-region sums |
| **DX2** | topology / structural Verify | existing builder | payload builder | slab-region union builder |
| **DX3** | `Tessellate` / STL / OBJ | existing revolve tessellator | patch tessellator: one count per wall walk shared by the side wall, the band patch and the cap contour (`agent-docs/tessellation-reach-design.md` §7) | required slab-region tessellator |
| **DX4** | mesh boolean | existing revolve operand path, with its own facet ceilings; no blend-specific gate | admitted for a band whose every corner is a line-line miter or an exactly G1 join, or a whole turn (`agent-docs/tessellation-reach-design.md` §7); a circular wall at a genuine miter or a reflex corner stays `ErrUnsupported` | available once DX3 exists |
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
(verification design §1.1). A `capblend.DisplacedPayload` is staged the same way in DX7 to DX9: the surveys' payload switches have no arm for it, so each answers `DiagUnsupportedSurveyPayload` naming `displacedCapBlendPayload` and the body reads `Suspect`. No open implementation claim remains.

Clearance may also return undecided for surface cells its certified kernel does
not solve. This is the existing `Verify` contract, not a modify-build refusal.

DX6 is staged for the cap-loop chamfer, as the table says (§14). The cap-loop fillet builds today as a `brepPayload`
(reach RF3), so its clearance reading is Table DG6's, not DX6.

The chamfer's DX3 reading is a mesh. A patch tessellator must chord the
cap-level offset boundary and the side-level original boundary into one strip,
and the two may need different sample densities; a strip whose densities
disagree is not watertight, and a mesh that is not watertight is a wrong answer
rather than a coarse one. The tessellator answers that with ONE count per wall
walk, shared by the side wall, the band patch and the cap contour, and charges
each patch its own positional departure; the design is
`agent-docs/tessellation-reach-design.md` §7. DX4 reads that mesh's occupied-volume
proof where §7 states one — a band of line-line miters, exactly G1 joins and
whole turns — and every boolean refuses the operand for any other band, which
serves export alone.

The chamfer's DX6 reading is `Suspect` for any pair its bounding boxes do not
already decide. A trimmed patch face admitted into the certified kernel
without a proof of its trim yields a false disjointness certificate, and a false
certificate is worse than an undecided pair: `Verify` would report a clearance
the geometry does not have.

### 13. Required tests

Every implementation PR MUST add geometry assertions, not run-only coverage.

#### Options

- option nil/duplicate/conflict gates;
- `WithNoOpenings` nil-selector exception and non-nil conflict;
- tangent-chain seed cardinality before expansion;
- asymmetric nested selector deep-copy;
- repeated calls select the same seeds, expansion, reference faces, and result;
- failed call leaves the body live and the document unchanged.

#### Tangent chain

- line↔arc and arc↔arc proven G1 continuation;
- same tangent but non-tangent face sheets → stop;
- branch → SX2;
- near-tangent exact `no` → stop;
- oracle `unknown` → SX2;
- closed cycle visits each edge once;
- deterministic body-order result.

#### Asymmetric chamfer

- different feet measured along both adjacent walks;
- reference face swaps the two distances;
- one reference face per edge over multi-edge selection;
- missing/dual/extra reference → SX3;
- independent overrun on either side → base S6;
- the option preserves the reference selector and other distance exactly.

#### Revolve

- full-turn latitude fillet creates expected torus/sphere and exact radius;
- partial-turn junction arc produces same meridian rewrite;
- chamfer line classifies to cylinder/plane/cone as expected;
- bounded volume/area/centroid against rewritten-profile integrals;
- cap-edge selection → SX5;
- axis contact and spindle gates preserve base sentinel;
- roles and selectors survive placement/replay.

#### Cap loops

- whole circle rim fillet has no seam vertex patch;
- polygon loop produces cylinders + spherical miter patches;
- reflex line/arc loop produces torus/cone vertex patch;
- partial cap-loop `Chamfer` of at least two edges and mixed cap/lateral selection → SX4;
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
- a band patch's `NormalAt` bound ENCLOSES its distance from the normal of the
  wall its records denote, read in 400-bit arithmetic: a 0.1 mm chamfer on a
  20 mm square and on a 20 mm disk, each drawn at `v = 10⁶`, whose cap contour
  is held an ulp of `10⁶` off the denoted one;
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

#### Shell

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
- side run disconnected/all-side, or a side opening on a holed section → SX8;
- no-opening on a holed section builds inward and outward; an outward one
  whose offset empties a hole → S11a;
- full-revolve side opening and closed shell;
- axis-touching cylinder/sphere shell has no fabricated axis wall;
- partial revolve builds only with both angular caps selected;
- partial kept cap → SX8;
- tessellation watertight across slab interfaces and between all BX8 rim faces;
- asked wall reading is `Suspect`, never absent or fabricated.

#### Faceted receivers

- positive-bound boolean receiver → SX9;
- zero-bound all-planar boolean receiver → SX9;
- refusal leaves operands live and the document unchanged.

### 14. Implementation order

| PR | Lands | Still staged |
|---|---|---|
| **A** | option records; tangent expansion; asymmetric chamfer of prism lateral edges and revolve junctions; `WithNoOpenings` accepted and refused per receiver | brep and stacked asymmetric selections outside routes E/L (SX16); all SX9/SX10 |
| **B** | revolve junction rewrite + roles + surveys | none |
| **C1** | multi-region `stackedPrismPayload` (the lining reading); cups recorded on it; base S12 lifted through BX8 | side-opening prism shell beyond C4's rectilinear case |
| **C2** | closed prism shell (BX5): the void-shell stack, its tessellation | side-opening prism shell beyond C4's rectilinear case (BX4, Part O) |
| **C3** | revolve shell side opening, full and partial turn, right-angle rims (§9.3.2) | none |
| **C4** | the side opening of Part O §12: the rim rule and the revolve's slanted rim, the rectilinear prism side opening as a `brepPayload`, circular walks, oblique walks, arc–arc corners and oblique arc joins, a kept arc extended past its end | per that document's increment table |
| **D** | partial-turn revolve shell with both angular caps removed and no side opening (BX7); full-turn closed shell under `WithNoOpenings` (BX6), §9.3.1 | none |
| **E** | `capBlendPayload`; complete cap-loop chamfer at an equal setback and at two distances (§8.3.1); analytic integrals | DX4 admission for a mitered circular wall or a reflex corner; DX6 clearance model; partial cap-loop chamfers beyond modify-general §4.3a; mixed edge classes; faceted receivers |

Each PR lands its result payload, structural topology, measurement path, and
tests together. A PR may leave a DX question staged only where
Table DX explicitly says `Suspect` or `ErrUnsupported`.

The authenticated loft/cylinder cap and perpendicular cross-bore routes are
RX6's SX9 exceptions. Brep and stacked receivers are
Part B's, not another row in this extension.

### Implementation notes

#### internal/capblend contour

The cap-loop chamfer's cap contour displacement (§8.4) and
`agent-docs/prism-boolean-design.md` §7's `sectionDelta` are the same idea applied to
two different constructions, and they are independent terms with separate
owners: `internal/capcontour/` derives the cap contour's own displacement;
`internal/capblend` maps it to the cap-band construction and refusal, and its
build/measurement code (`band.go`, `payload.go`, `build.go`, `measure.go`)
carries it into every reading that needs it. No
cap-blend reading composes `sectionDelta`, and no `sectionDelta` consumer reads
the cap contour's displacement. `capBlendPayload` separately preserves its
receiver's per-end axial displacement and the selected-end setback rounding.
Each circular patch's held allowances are `capband.HeldAllow` (`internal/capband/patch.go`). `internal/capband/closure.go` bounds
each band's closure slivers (`capband.Closure`), which
`internal/capband/band_mass.go` charges beside the patch integrals (§8.4's held
numbers paragraph).

## Part G — Modify general

The next reach of `Fillet`, `Chamfer` and `Shell` over
analytic boolean results: a `brepPayload` (`agent-docs/prism-boolean-design.md` Part G
§4, "general-boolean §N" below) and a `stackedPrismPayload` through its face
view (`agent-docs/prism-boolean-design.md` Part S). Companion to
Part B ("brep-modify §N"), whose routes P and E this
document extends with route S (shell) and route L (complete-loop chamfer),
and whose Table SB rows SB3, SB4, SB5 and SB10 it narrows. Part F
("modify §N") keeps the call contracts, the base gates and the §5 audit;
Part R ("reach §N") keeps the cap-loop chamfer's
construction and proofs (§8.3, §8.4), which route L reuses.

Seven tables are normative:

| Table | Owns | Section |
|---|---|---|
| **SV** | the reference-part survey: what refuses today, with its code | §1 |
| **TC** | the through-cut record route S recognises | §3.1 |
| **SG** | every refusal route S adds, with its sentinel and gate order | §3.4 |
| **LB** | the loop selection route L admits | §4.1 |
| **SL** | every refusal route L adds | §4.4 |
| **BG** / **DG** | result payloads and what every consumer reads | §5 |

### 1. Survey — Table SV

Nine reference parts, each built from operations decad ships today through
the public API, and the fillet, chamfer and shell requests an ordinary Fusion
part makes on them. "builds" names the route that built it.

| Part | Receiver | Request | Today |
|---|---|---|---|
| **P1** cross-drilled bar: 40×20×20 box, Ø6 hole along y | brep (class B) | fillet the 4 edges along y | builds (route P) |
| | | fillet one edge along z | builds (route E) |
| | | fillet every straight edge | builds (route V; Part V) |
| | | chamfer every straight edge | builds (§4.3b) |
| | | chamfer the 8 along x and z, the top and bottom loops | builds (route L) |
| | | fillet the top and bottom loops | builds (route L fillet arm) |
| | | chamfer a hole rim, or both | builds (route P cap loop) |
| | | fillet a hole rim | builds (route L fillet arm) |
| | | shell removing one y wall, or both | builds (route P cup, tube) |
| | | shell removing the top face, or both caps | builds (route S) |
| | | shell removing one x wall, or one y wall with the top or both caps | builds (route S, a wall run) |
| | | closed shell (`WithNoOpenings`), inward or outward | builds (route P's prism reading along y, reach BX5; §3.2) |
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
| **P5** counterbored hole: Ø6 through, Ø10 counterbore 3 deep | stacked (A2) | shell removing the counterbore end | builds (§3.1e, for exact coaxial circles in a rectangular plate) |
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
| | | fillet the front top edge along x at r = 1 | builds as a bounded faceted cut (§4.3c) |
| | | shell removing the top | builds (route S) |
| | | shell removing one y wall | shell-opening SO1 (the wall meets the fillets smoothly) |
| **P9** round rod with a cross hole | faceted | chamfer both hole mouths, one setback | builds (retained-edge-blend R7, §4.9) |
| | | fillet both hole mouths | builds (retained-edge-blend R7 fillet, §4.10) |
| | | any other request | SX9's class (`chamfers a straight prism or a revolve only` for a chamfer); cylinder × cylinder has no analytic class |

Ranked by the parts each refusal blocks:

| Rank | Refusal | Blocks | What the body needs |
|---|---|---|---|
| 1 | SB10 / SB3: shell of a nonprism brep or a noncap face | P2/P3 from their opposite cap, P6c | receiver erosion with the missing rounded transitions and an occupied-volume proof |
| 2 | SL1: edges sharing a vertex outside complete loops | other mixed-edge parts | route V builds complete loops with independent edges; §4.3b builds P1's all-edge chamfer; a partial fillet builds edges on one planar loop after straight-wall restatement (Part V) |
| 3 | a loop fillet: the curved-edge and cornered-loop fillets this survey found refused | P1, P2, P3, P4, P7 | Part L's pipe band, which builds them |
| 4 | SB7: an edge ending on a curved face or a blend | P8 outside §4.3c | the complete-loop fillet builds P8's top loop; §4.3c builds its one exact front top edge |
| 5 | a faceted receiver | P6b, P9 outside its hole-mouth chamfer | an analytic boolean: a cup as a boolean operand, cylinder × cylinder; reach SX9 stays for them |

Routes S and L below clear every brep and stacked row of ranks 1–2 whose
body is planes and cylinders along reference axes, meeting in lines, circles
and arcs; route L's fillet arm (Part L) clears ranks 3
and 4. Route V builds the sphere corners of an exact-radius three-edge
fillet (Part V).

### 2. The two increments

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

### 3. Route S — shell of a through-cut record

#### 3.1 Table TC — recognition

Route S reads the receiver's face record (`brepPayload`, or `breprecord.OfStacked`
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

Table TC ends at TC6. TC7 is the construction check on the strips beyond the pierced walls (§3.3 step 3), not a
recognition condition.

A stacked union outside the admitted boss cases below fails TC1: three
planar faces across `k`. A blind pocket fails TC2: its floor is a third
planar face across `k`,
and its walls' levels are no wall's. A keyway, a boss crossing a plate's
outline, a hole breaking out of a face, a split side line and an oblique
wall fail TC2 or TC6. Each is SG3, which names the first face the table does
not take along the first axis whose caps TC1 reads, else the third planar
face across the first axis holding more than two, else axis 0's reason.

#### 3.1a Exact rectangular-boss shell

Before route S's through-cut reading, `stackedshell.Build` tries the stacked
shells in this order: the rectangular boss (this section), the circular boss
(§3.1c), the blind pocket (§3.1b) and the counterbore (§3.1e). Each builder reads only the
stacked record, its face view and `stackedshell.Call`, in which root states
the sense, the thickness and whether the one removed face is the receiver's
cap-end or cap-start face. The rectangular-boss builder admits one nested,
hole-free pair of axis-aligned rectangular slabs in the same frame. Both
sections and all levels must have zero displacement. The lower slab is the
plate, the upper slab is the boss for the top-open and bottom-open calls.
`WithNoOpenings` (`stackedshell.Closed`) accepts that order and the reverse
order, where the lower slab is narrower.
The reverse order negates both slab intervals and each section's V coordinate,
flips the frame's V axis exactly, and rebuilds the exposed interface. It then
takes the same closed construction in a right-handed frame. If `r3.NewFrame`
changes any held U, V or N component while flipping V, the reverse route
refuses because the source-to-result map would need a displacement bound.
Inward thickness `t` must leave a plate thicker than `2t`, a positive lower
cavity wall, an upper cavity at least `2t` narrower in both directions, and, for a
closed or bottom-open shell, a positive cavity below the boss roof. It must also leave
positive clearance between the boss and the plate's eroded outline. All new
offset coordinates must be exactly representable. Other stacks continue to
route S and keep SG3.

With no opening, the source's complete outer boundary remains. The cavity
ends at `z2 − t`, where its ceiling faces into the void. Its lower inset,
ledge and four quarter-cylinder root patches match the top-open construction.
The completed cavity forms one void shell inside the one outer shell; the
plate floor, boss roof and root clearances prove they do not meet.

The bottom-open result removes the plate's cap-start face and keeps the boss
roof. Its cavity ends at `z2 − t` under a ceiling, as the closed result's does,
and the lower inset's walls run from the plate bottom `z0` to the ledge at
`z1 − t`. One planar annulus at `z0` joins the plate outline to the lower inset
and replaces the cavity floor, so the body is a single shell. The plate must
still be thicker than `2t`, which is stricter than the geometry strictly
needs and is kept to share the closed arm's exactness gate. `BossShellBand.CavityBottomZ` is the lower
cavity's bottom level, `z0 + t` above a kept floor and `z0` here; the measure
reads the lower cavity over `[CavityBottomZ, z1 − t]`. For the same dimensions
and thickness, the bottom-open volume is the closed volume less the lower
inset's area times `t`. Of the bosses and pockets, only the rectangular boss
opens at the plate bottom; the circular boss and the blind pocket keep SG3
there. The counterbore (§3.1e) opens at `CapStart` when its large bore is at
the bottom.

The top-open result keeps the source's exterior faces except the boss top.
Its cavity has a plate inset over `[z0+t, z1-t]`, a horizontal ledge at `z1-t`, and a
boss inset over `[z1, z2]`. Four quarter-cylinder patches join the ledge's
boss-sized inner ring to the upper cavity walls. The root edge, the boss
outline at `z1`, is a concave edge of the source, so modify §8's reflex-corner
row applies to it: the cavity keeps distance `t` from the root edge. Each
patch therefore lies on the cylinder of radius `t` whose axis is one source
root edge, on the quarter below `z1` and inside the boss outline; the solid
lies inside the cylinder. A patch runs from the ledge's ring, straight below
its axis at `z1-t`, to the upper inset's ring at `z1`. Adjacent patches meet on
four `Ellipse3` seams, the quarter ellipses the bisector plane of a root corner
cuts from both cylinders: centre on the root corner at `z1`, semi-axes `t√2`
and `t`. The lower and upper rings each bound one ordinary
record face and one patch. The body build mints fresh `face(k)` and `wall(k)`
roles; each patch has a `filletLoop` role. No planar cap is inserted at the
transition.

The closed result keeps the boss's complete top, ends the boss inset at
`z2 − t`, and caps that inset with one cavity ceiling. The source's exterior
faces form one connected shell. The derived cavity floor, inset walls,
ledge, four cylinder patches and ceiling form one other closed face group.
`breprecord.Build` marks that group void after confirming that all source
faces occupy the exterior group. For the same dimensions and thickness,
the closed volume adds the boss inset's area times `t` to the top-open
volume; its first axial moment adds that volume at height `z2 − t/2`.

For boss width `w`, depth `d` and thickness `t`, the transition's void at
`s = z1 − z`, `0 ≤ s ≤ t`, is the boss rectangle inset by `sqrt(t² − s²)` on
every side, with sharp corners. Its volume is `wdt − (w+d)πt²/2 + 8t³/3`, and
its first height moment above `z1 − t` is
`wdt²/2 + (2/3 − π/2)(w+d)t³ + 5t⁴/3`. The shell volume subtracts
that transition, the plate-inset void below it and the boss-inset void above it from the two
source slabs. Rational interval arithmetic encloses `π`, volume and centroid.
`breprecord`'s `attachBossShellBand` builds the four cylinder patches and
their seams on the two rings' edges. A patch's area is `tπL/2 − 2t²` for a root
edge of length `L`. The fillet band's mesh-ring builder places the interior
rings with the two rings exchanged (`brepfilletmesh.Input.RootAxis`), so each
ring turns about the root edge. Its ring count covers each cell's `|T|/4`
twist bound as well as the sagitta, and each cell is charged the smaller of
that bound and its height bound (loop-fillet §7.1). The band's cells are
planar, so the mesh `Bound` stays within the requested tolerance, and
a tighter tolerance raises the ring count until it reaches the cap
`tessellation.MaxFilletRings`. `Tessellate(VerifyAll)` charges the band's chorded volume,
surface departure and vertex motion before publishing its occupied-volume
bound. A later `Fillet` or `Chamfer` runs on the body's faceted view, and a
selection that maps to the view and that the retained edge blend recognizes
builds a faceted result; a later `Shell` refuses without a view
(Part B §2.1, RB5). Class B and clearance
face views refuse it because they omit its patch faces. Mesh booleans use its
verified tessellation.

#### 3.1b Exact rectangular blind-pocket shell

Before route S's through-cut reading, `stackedshell.Build` admits two slabs in
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
rectangle is `offset2d.OffsetProfile(pocket, -1, t)`; its certified displacement must
be zero, and its fillet contour at `t` must reproduce the source square.

For a pocket of width `w`, depth `d`, axial depth `h`, and thickness `t`, the
expanded pocket's volume within the outer cavity is
`wd(h+t) + 2(w+d)th + πt²h + (w+d)πt²/2 + 2πt³/3`.
The shell volume is the source volume minus the eroded outer box's volume
plus that expanded-pocket volume. Rational intervals enclose `π` and all
moments; the body's occupied-volume proof uses the rounded vertical walls'
chord slivers and the fillet band's sphere/cylinder patch bounds. A later
`Fillet` or `Chamfer` takes §3.1a's faceted view, and a later `Shell` refuses
(brep-modify §2.1). Class B and clearance face views
refuse it because its record omits the band patches; mesh booleans require
its verified tessellation.

#### 3.1c Exact circular-boss shell

Before route S reads a through-cut, `stackedshell.Build` admits two exact slabs in
one frame: a hole-free axis-aligned rectangular plate and one whole-circle
boss strictly inside it. The top boss face must be the only removed face, and
the shell must be inward. Both sections, all levels, the circle radius and
the thickness must have zero displacement. The plate height must exceed
`2t`, the boss radius must exceed `t`, and the boss circle must lie strictly
inside the plate's inset rectangle. The offset levels, radius and rectangle
coordinates must be exactly represented. Other unions continue to SG3.

The result keeps the source exterior except the boss top. Its cavity has the
plate's inset rectangle over `[z0+t,z1-t]`, a ledge at `z1-t`, and the boss's
concentric inset circle over `[z1,z2]`. One quarter-torus patch joins
the ledge's circle of radius `r` to the inner cylinder of radius `r-t`. As in
§3.1a, the cavity keeps distance `t` from the root circle, so the torus's tube
centre is the root circle itself: major radius `r` at `z1`, minor radius `t`,
the quarter below `z1` and inside the root. Its area is `π²rt − 2πt²`.
The patch has two whole-circle edges and no seam. The ledge hole uses the
band's side-ring stations; the inner cylinder uses its cap-ring stations.
Both rings therefore share mesh vertices with their neighbouring faces.

At height `u` above `z1-t`, the transition void has radius
`R(u)=r-sqrt(t²-(t-u)²)`. Its volume is
`π[r²t-rπt²/2+2t³/3]`. Its first height moment above `z1-t` is
`π[r²t²/2+(2/3-π/2)rt³+5t⁴/12]`. Rational intervals enclose both powers
of `π`, the full shell volume and centroid. `Tessellate(VerifyAll)` uses the
existing whole-circle fillet-band and face-chord volume proofs, with the
mesh rings turned about the root circle as §3.1a's are. Later modify,
class B, and clearance face views keep §3.1a's staging; a mesh boolean uses
the verified mesh.

#### 3.1d Exact circular through-wall opening

Before route S classifies removed faces, a seven-face through-cut record may
remove its one cylindrical tool wall. Admit only one exact whole-circle tool
through an axis-aligned rectangular box, with no other holes or curved outer
walls. The cylinder must be the sole removed face. Require inward sense,
exact source sections and levels, exact thickness conversion, and exactly
represented inset rectangle and axial levels. The original circle must lie
strictly inside the rectangle inset by `t`; both axial end layers and the
middle layer must have positive height. Other tool-wall removals remain SG4.

The result is three stacked sections along the tool axis. Each end layer has
the original rectangle and the original circular hole, over thickness `t`.
The middle layer has the original rectangle and its inset rectangle as a
hole. At each interface, one planar patch occupies the inset rectangle less
the original circle. The two cylindrical end-layer walls are the rims of the
removed source cylinder. Both exterior hole mouths keep their original
radius. The strict circle-in-inset-rectangle comparison proves each interface
ring has no crossing or contact. `stackedrecord.Falsify` checks the loop and
level identities; stacked mass and `Tessellate(VerifyAll)` carry the analytic
area, volume, boundary, and occupied-volume proofs.

For box section area `A`, inset area `A'`, through length `h`, source hole
radius `r`, and thickness `t`, the shell volume is
`2t(A−πr²) + (h−2t)(A−A')`. For the 40×20×20 reference bar with a
radius-3 hole and `t=1`, this is `3688−18π` mm³.

#### 3.1e Exact coaxial counterbore shell

Before the through-cut reader, `stackedshell.Build` accepts two stacked Cut
slabs with one common axis-aligned rectangular outer loop and one whole-circle
hole in each slab. The circles must have exactly the same recorded center.
The larger circle must lie at the sole removed cap, on either end of the
plate; the two Cut orders give the same reading. The source sections, levels,
circle radii and thickness must have zero displacement. The far slab must be
taller than `2t`, the shoulder width must exceed `t`, and the larger circle
expanded by `t` must lie strictly inside the outer rectangle inset by `t`.
All offset levels, radii, rectangle coordinates and circle extents must be
exactly represented. Other counterbores keep SG3.

The result keeps the source faces except the opening cap. Its cavity has an
inset rectangular floor with a hole of radius `r_s+t` at the kept end, an
inset outer wall to the opening, and a lining of radius `r_s+t` along the
smaller bore. At distance `t` before the shoulder, an annular underside runs
from `r_s+t` to the source larger radius `r_l`. One whole-turn quarter torus
joins that ring to the radius-`r_l+t` lining at the shoulder. The opening has
two annular rims, one along the outside rectangle and one around the larger
bore. The torus uses the blind-pocket shell's band reader with one whole
circle; its two rings share mesh vertices with the underside and lining.

Let `h_f` be the far slab height, `h_n` the opening-side slab height, and
`H=h_f+h_n`. Let `A` and `A'` be the outer and inset rectangle areas. The
source volume is `AH−π(r_s²h_f+r_l²h_n)`. The outer cavity has volume
`A'(H−t)`. The bore obstacle left within that cavity has volume

```text
π[(r_s+t)²(h_f−2t) + (r_l+t)²h_n
  + r_l²t + r_lπt²/2 + 2t³/3].
```

The shell volume is source minus outer cavity plus that obstacle. In the
coordinate from the kept end toward the opening, the transition's first
moment is `h_f V_transition − π(r_l²t²/2 + 2r_lt³/3 + t⁴/4)`.
Rational intervals enclose both powers of `π`, the volume and all centroid
coordinates. `Tessellate(VerifyAll)` uses the whole-circle fillet-band proof;
the band chords at one quarter of the requested tolerance so its azimuth,
meridian and cell-twist departures fit the requested bound. The result has
the same later-modify, class-B and clearance restrictions as §3.1b.

#### 3.2 Removed faces and the sense

The removed faces are read against the recognised record:

| Removed face | PR | Rule |
|---|---|---|
| a cap of `A` (TC1's bottom or top), one or both | S-1 | `A ⊖ t` keeps a removed cap's level and moves a kept one by `t`: modify §9's cup interval `[zlo + t·kept₀, zhi − t·kept₁]` |
| a wall of `A` (an (a) or (b) face), one proper connected run of whole walls of `S`'s outer loop, with or without either cap | S-2 | `A ⊖ t`'s section is Part O §3's cavity section `C` over the kept chain, with that document's rim rule at each end (`prismshell.SideOpeningRegions`); its SO1–SO6 refusals keep their codes, a set of walls that is no proper connected run among them (SO6). No section limit runs (shell-opening §5). Each removed wall must be a straight wall along a section axis, whose rim is a planar face, and neither end of the run may cut backward along the removed carrier (a reflex end, whose rim lies inside the material): SG5 |
| a tool wall outside §3.1d, a tool's floor, a face of a hole of `S` | — | SG4 |
| two removed faces sharing an edge, other than the runs above | — | SG5 |

A shell that removes faces (route S) builds `Inward` only. The dilation
`(A \ T) ⊕ t` does not distribute over the difference and rounds every convex
edge of the receiver — a hole mouth's rim into a torus, a box corner into a
sphere — so an outward one is SG1. A closed shell removes no face and is not
route S: it runs first, in either sense, through the cases below.
`WithNoOpenings` takes §3.1a's closed rectangular-boss case, and a closed
shell of a brep record that route P reads as a prism along some reference
axis (brep-modify §4.1); the shell is the prism's own closed shell (reach
BX5), a holed section's included. No face is removed, so no cap
classification picks the axis. Inward, route P takes the first axis that
reads: the erosion keeps every convex edge sharp and rounds only reflex
ones, and a record that reads along several axes is a box, whose erosion is
the same box along each, so every reading denotes the same solid. Outward,
the shell rounds the convex edges along the reading axis and keeps the cap
edges sharp (modify §7, §8), so two readings of a box denote different
solids: the record must read along exactly one axis, and one that reads along
more is SG2 (`closedShellPrismReading`). P1, the drilled bar, reads as a
prism along its hole's axis only, and its closed shell is the three-slab
stack whose cavity rings the hole's lining inward, or is P1 itself outward,
the lining then growing into the hole (reach §9.2). It is SG2 on other brep or
stacked receivers: a brep record's lumps are its connected face sets
(general-boolean §4.2), and a closed cavity's faces would read as a second
solid without a nesting proof. The void shell `prismshell.BuildClosed` builds
lives on the stacked record alone.

#### 3.3 Construction

With `s = +1`, `t` the thickness in millimetres with its conversion bound
`tDelta` (`extent.MagnitudeInBounded`), and the receiver live:

1. **Gates on `S`.** Modify §8's shell gates on `A`'s section, in modify §4's
   order: S18, S10's section limit (`shellsurvey.SectionInradius`), S10's
   height limit per kept cap (`t < h` with one cap kept, `t < h/2` with both:
   reach SX11's reading), S11a as `offset2d.OffsetProfile(S, +1, t)` is built, then
   `offset2d.AuditOffsetSectionBudget` (S8, S11b, S9).
2. **Each tool dilated.** `Dᵢ = offset2d.OffsetProfile(Tᵢ.profile, −1, t)`: S11a for a
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
   the float sum's rounding, as `prismshell.BuildClosed` derives it, and every face
   of its view carrying `offset2d.OffsetSectionDelta(S, +1, t, tDelta)` (modify §9's
   figure). `Tᵢ' = prismPayload{profile: Dᵢ, frame: F_j, z0: L0 − t, z1: L1 + t}`
   with `offset2d.OffsetSectionDelta(Tᵢ, −1, t, tDelta)`. The cavity is
   `Cut(…Cut(A', T₁'), …, Tₙ')` through class B's own entry over payloads
   (`classbgeom.OfPayloads`, the body-free form of class B's route): each cut
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
   as a parameter (`classbgeom.OfPayloads`'s `delta`; class B's own booleans pass
   zero) under two rules. First, every face box of `A'` (and of the cavity
   so far) and every tool box that B7 and the through reach compare grows by
   `c` on every side, and each through-reach slab's level band by `c`: a
   dilated tool recorded within `2c` of a cavity face meets it, and the cut
   refuses (SG6) where the denoted tool may cross the denoted face. Second,
   a positive `c` refuses the crossing reach (SG6): that reach computes its
   crossings from the zero-displacement records, and the crossing-angle
   charge (prism-boolean §3.4, A6) amplifies only a displacement the records
   carry, so a cut crossing at a grazing angle would publish vertices the
   flat `c` does not cover. A cut whose `c` is zero lifts B8 and takes the
   flush reach, the sweep walk and the crossing reach as class B does
   (general-boolean §3 "Flush cap contact", "Pin across several faces"). Class B also needs every line exactly along a reference
   axis (B6), so an eroded polygon whose float miters drift off the axis — any convex
   corner at a thickness that is no exact float, such as 0.1 in — is SG6; a
   section whose corners are arcs erodes through G1 joins that keep each line
   on its axis.
5. **Assemble.** The result record is: every receiver face but the removed
   ones, verbatim; every cavity face reversed, except those lying in a
   removed face's plane; and one rim region per removed face `R`. Reversal
   flips a planar face's `outward` and walks a swept face's wall the other
   way (`brepgeom.ReverseSegment`, extended to a `CircleSeg` by flipping `CCW` and
   swapping its range, with `side0`/`side1` exchanged), the rule
   `brepPayload.Placed` applies under a reflection. The rim at `R` (level
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
7. **Closure.** `assignRoles`, `breprecord.Falsify`, `breprecord.TopologyContext`:
   an unpaired edge is `ErrUnsupported` (general-boolean §4.2's count), the
   proof that the receiver's kept faces, the cavity and the rims close.
   `breprecord.Build` builds and measures; a non-positive volume is
   `ErrDegenerate`. `commitModifyResult` retires the receiver.

Why the pieces pair by identity: a removed cap's region is `S` (TC1), the
cavity's cap there is `S ⊖ t` verbatim where no tool reaches the cap plane
(general-boolean B.3: a tool along `j ≠ k` meets only faces across `j`), and
its hole `i` is `(S ⊖ t).Holes[i]` as `offset2d.OffsetProfile` keeps loop order. A
tool that does reach the cap plane splits or notches that cap through the
crossing reach, which step 4 admits only where the cavity's charge is zero;
step 5 then reads the pieces as they come and step 6 proves the rim's
nesting, so a notched cavity cap builds where its loops still partner, and
refuses honestly where they do not.

#### 3.4 Table SG — refusals and gate order

Modify §1's test picks every sentinel.

| SG | Call | Exists? | Sentinel |
|---|---|---|---|
| **SG1** | `WithShellSense(Outward)` with removed faces on a brep or stacked receiver (an outward closed shell through §3.2's prism reading builds, and so does an outward shell whose removed faces are the caps of a prism the record reads as: route P runs before route S and checks only which caps are removed) | yes; the dilation rounds the receiver's convex edges into tori and spheres this record does not hold | `ErrUnsupported` |
| **SG2** | `WithNoOpenings` on a brep or stacked receiver outside §3.1a's nested rectangular boss and outside a shell of a brep record route P reads as a prism (§3.2); an outward one of a record that reads as a prism along more than one axis | yes; no other brep record carries a certified void shell | `ErrUnsupported` (replaces SX16's text for this call) |
| **SG3** | the record is no through-cut record (Table TC) and is outside §3.1a–§3.1e: a blind port, a keyway, a crossing boss, a split or oblique wall, a displaced tool level | yes; its erosion may hold surfaces beyond the admitted cases | `ErrUnsupported`, naming the first face Table TC does not take |
| **SG4** | a removed face that is a tool wall outside §3.1d, a tool floor or a hole wall of `S` | yes | `ErrUnsupported` |
| **SG5** | a removed face that names no face of the record; a removed wall that is no straight wall along a section axis (a fillet cylinder: its rim is no planar face); a wall run whose end cuts backward along the removed carrier (a reflex corner: the rim lies inside the material). A set of walls that is no proper connected run is shell-opening SO6 | yes | `ErrUnsupported` |
| **SG6** | a tool's dilation reaches material beyond its pierced wall (TC7), or a private class-B cut does not build a brep: two dilated tools within `2t`, a dilated tool reaching a cap of `A'`, an eroded miter off its axis, a dilated tool recorded within twice the cavity's charge of a cavity face, or a cut that takes the crossing reach while that charge is positive (§3.3 step 4) | yes | `ErrUnsupported`, naming the tool |
| **SG7** | a rim's loops do not partner the cavity's trace, or do not chain into one outer loop with holes or outer loops alone (step 5) | — (a falsifier) | `ErrUnsupported` |

Gate order for a brep or stacked `Shell`, after modify §4's stage 1 and
reach SX10:

| Stage | Gates |
|---|---|
| 2a. record | RB dispatch; SB2; SB1 |
| 2b. route P | brep-modify §4.2 unchanged: an axis whose prism takes the removed faces as caps builds the prism's own cup or tube |
| 2c. route S entry | `WithNoOpenings` first: §3.1a's closed-boss case, a closed shell through route P's prism reading (§3.2), or SG2; then SG1; Table TC (SG3); §3.1d's exact circular-wall case; the remaining removed faces against the record (SG4, SG5's face and straight-wall tests) |
| 3. existence | caps alone: S18; S10 (section and height limits); S11a on `S`. A wall run: `prismshell.SideOpeningRegions` in shell-opening §5's stages 2–5 (SO6, SO3's height half, S11a and SO1, SO2, SO4 per end, S8, S11b and S9 on `W` and `C`, the area identity), then SG5's reflex-end test. Then S11a on each tool |
| 4. offset audit | S8, S11b, S9 on `S ⊖ t` (caps alone) and on each `Tᵢ ⊕ t`; TC7 (SG6) |
| 5. cavity | the private cuts (SG6) |
| 6. assembly | SG7; the rim audit S8, S7, S9 |
| 7. closure | `breprecord.Falsify`, `breprecord.TopologyContext`, `breprecord.Build` |

SB10 narrows to a record route S also refuses, and its text names SG3's
reason. SB3 narrows likewise: a record that reads as a prism along some axis
whose caps are not the removed faces is read again by Table TC along every
axis before it refuses.

### 4. Route L — chamfer of a complete loop on a planar face

#### 4.1 Table LB — the admitted selection

Route L runs for `Chamfer` and `Fillet` on a brep or stacked receiver when
route P takes no axis and the selection is not route E's admitted independent
straight edges. A single straight Fillet or Chamfer edge whose end-face corner
differs from the interior material wedge (brep-modify EB8) takes route L when
it is part of a planar-face loop. A `Fillet` builds its loops with the fillet arm of
Part L, whose band is a pipe where this section's is
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

#### 4.2 Construction

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
   construction (`capband.LoopBoundary`): SX6 for a dropped carrier, SX13's radial
   half per circular wall, SX14 per corner. `F`'s region is rewritten with the
   contour in `ℓ`'s place, and the second offset at the top of `dc`'s
   conversion span runs as `capblend.AuditSetbackSpan` does.
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
   At body build, after `breprecord.Build` has built every face and edge of
   the record, each band is attached by reach §8.3's `capblend.BuildBand` run in
   `F`'s own frame (`prismLike` over `F.frame`, cap level `L_F`, side level
   `sideZ`, `matSign` from `σ` and `F.outward`), with both of its directrices
   handed in as the brep's own coedges: the cap contour's edges are `F`'s new
   loop coedges, the side contour's edges are the (sw) faces' rims at `sideZ`
   and the (pl) faces' moved segments. `capblend.BuildBand` takes the cap-level
   coedges from its caller; the prism cap blend passes the ones it minted. Patches, their readings (`setPatchReadings`), their
   orientation (`capblend.FixPatchOrientation` against reach §8.3.1's reference
   `(ds·n̂_wall, σ·dc·n̂_F)`, which has a positive dot product with the true
   outward normal in both senses), their skews, locus spans and corner flux
   are reach §8.3's and §8.4's unchanged.
6. **Closure.** The record's faces pair by `breprecord.TopologyContext` as before;
   the band's patches each bound two faces by construction (the contour edge
   on `F`, the side edge on the adjacent face, the corner rulings between
   neighbouring patches), and `topobuild.AttachFaceLoops` proves every edge
   bounds exactly two faces over the whole body.

The contour displacement (reach §8.4, `capband.ContourDisplacement`) is
charged into `F.delta`, so every reading of `F` and every cap-level vertex
of the bands carries it, and a later modify op on a body whose contour was a
float solve refuses with SB1 — as it does for a cup. The side contour is the
receiver's own loop moved along `n`, so the faces beside it carry only the
side level's displacement (step 3). An axis-aligned
loop at a millimetre setback has an exact contour and keeps `delta = 0`, so a
second loop on the same body chamfers in a later call.

#### 4.3 Measurements

`breprecord`'s `measureBody` sums general-boolean §4.3's face terms over the
rewritten record, then adds each band's terms through reach §8.4's readers
(`capblend.ReadBandMass`): its volume flux and first moments about the plane-local
origin, which is the reference origin every face frame shares bit for bit
(`breprecord.Embeds`), each `Cone` patch's chord-versus-locus terms, the closure
slivers (`capband.Closure`), and `capband.LevelVolume` for the side level's
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

#### 4.3a Selected straight-edge chamfers

A symmetric `Chamfer` takes a selected-walk band for one or two adjacent straight
edges on a rising inner loop (a boss root), or exactly two adjacent straight
edges on a descending outer loop. The loop has at least four walks, and each end
of the selected run meets an unselected straight walk. The two-edge run has
a sharp common corner. Route E's three-edge, end-face and restatement gates
run on every selected edge. Route L's neighbouring-face, reach, contour and
section audits run on the selected walks. Each selected wall ends at the
band's side level; the planar face keeps the contour with only the selected
walks offset. A material-wedge mismatch bars route E before the single-edge
band is tried.

The one or two band patches are planar quadrilaterals between the side and
cap walks. Two selected walks meet at the exact intersection of their shifted
line carriers; the reflex connector arc used by a fillet is not part of a
chamfer. Two patches share an edge that joins their original corner at the
side level to the offset corner on the face. At each free end, the end face gets a straight
terminal segment between its side and cap feet. The builder refuses if a
terminal foot, any shared corner, patch plane, or both triangles' common
orientation cannot be proved from the recorded lines and setbacks. Exact
rational polygon integrals contribute the patches' volume and first moments.
The patch normal and those signed integrals follow the planar face's outward
direction for both the removed wedge and the filled boss root. The mesh uses
those same four vertices per patch; `VerifyAll` checks its closed boundary
and occupied volume. Curved, longer and disjoint chains continue to SL1.

`internal/breploop/partial.go` holds the selection gate, the restatement
check, the record rewrite and the end-face terminals for this chamfer and
for the selected-chain fillet: a terminal line for the chamfer and a
terminal arc for the fillet. `internal/brepmodify/partial_route.go` passes the
selected edges to route E's preparation and builds the body.
`internal/partialchamfer/` checks each patch's exact plane, winding and
setback, and supplies its bounded area and exact mass integrals to
`internal/breprecord/partial.go`.

#### 4.3b Simultaneous outer-edge chamfer of a box

The symmetric `Chamfer` of all twelve straight outer edges of P1 has a
bounded box-with-one-bore construction. Admission requires six zero-bound
axis-aligned rectangular planes, one through-y cylinder and its two matching
circle rims, no displaced record faces, and the selected set of all twelve
outer box edges.

The same twelve edges of a box with no bore take the same construction
without the bore (`breprecord.TryBareBoxAllEdgeChamfer`). The receiver is a
straight prism with a hole-free rectangular section, or a six-face record with
no loop bands, and it MUST read as six zero-bound axis-aligned planes whose
corners are the box's eight corners. The record's `Radius` is zero, so it has
eighteen planes and no bore wall, rims or hole loops. The volume formula below
holds with the bore term zero, and the result is `Exact` for a setback that
meets the exact-coordinate gate. A placement may turn any box axis onto any
other, where the bored box must keep its bore along y. The circular bore must remain strictly inside the inset
x and z faces. The setback and every resulting polygon corner and bore seam
must equal their rational coordinate expressions as `float64` values. Thus
`d = 1` and `d = 0.5` build on P1, while `d = 0.1` refuses rather than
publishing rounded corners as exact.

Six inset box planes and twelve six-sided bevel planes meet at eight shared
three-plane corners. Each corner lies halfway along every setback. The bore
wall and its two circular rim edges remain unchanged. The result has 19
outward source faces and two incident faces at every edge. Bounded scalar
operations hold each face area, edge length, volume, and centroid. If box
side lengths are `Lx`, `Ly`, and `Lz`, setback is `d`, and bore radius is
`r`, the volume is `Lx Ly Lz − πr² Ly − 2d²(Lx + Ly + Lz) + 6d³`.
The admitted centered bore and twelve symmetric wedges put the centroid at
the box midpoint.

The mesh uses the analytic polygon vertices directly and one set of chord
stations on both bore rims and the cylinder. `VerifyAll` audits the closed,
positive, crossing-free mesh and bounds occupied-volume difference by the
circle chord slivers times bore length plus the station-position allowance.
`Transform` and `Copy` re-evaluate the same record under a signed-axis
isometry when the transformed bore still runs along y. The placed box
endpoints and bore center must equal their rational images under the
transform's exact signed basis and recorded translation; §4.3b's corner
gate then runs on the placed record. Each face keeps its source role under
the axis and side mapping. The analytic readings are recomputed from the
placed coordinates, and tessellation uses those same coordinates. A rounded
coordinate or an oblique transform returns `ErrUnsupported` without changing
the document.

#### 4.3c One rounded cap edge on a cross-drilled plate

P8's front top edge from `(3, 0, 20)` to `(37, 0, 20)` ends on two radius-3
outer cylinders, so route E's third-face plane test (SB7) cannot build its
fillet. For radii 0.25, 0.5, 0.75, and 1 mm, `Fillet` first compares every
field of the source BRep record with the exact P8 record: a 40×20×20 plate
rounded along its four vertical corners, with one radius-3 bore centered at `(x, z) = (20, 10)`
through y. The source must have identity placement and zero displacements.
The selected edge must have those exact endpoints. A different radius, bore,
corner, placement, or face record falls through to the existing routes and
their refusals.

The admitted quarter-cylinder cutter is swept along x from −22 to 62. Its
local axes are `(0, −1, 0)` and `(0, 0, 1)` about `(20, 0, 20)`. The exact
source coordinates prove the bore stops at z = 13 below the cutter's z ≥ 19
minimum, and the back at y = 20 is beyond its y ≤ 1 reach. The
boolean cut then proves contact with the actual rounded end walls, closes
the resulting faceted mesh, and holds its occupied-volume difference. This
route uses 1/64 of the ordinary pair chord tolerance so the faceted area
and bounds also meet default `Document.Verify` limits. The source is retired
only after the proof succeeds. The source volume is 15280 mm³. For radii
0.25, 0.5, 0.75, and 1 mm, independent circular-segment integration bounds
the respective result volumes to 15279.5300–15279.5301,
15278.0976–15278.0978, 15275.6815–15275.6818, and
15272.2670–15272.2672 mm³. The real public Sketch→Fillet→Cut→Fillet path
tessellates at 0.1 mm with `VerifyAll` and reports Sound under default
`Document.Verify`.

#### 4.4 Table SL — refusals

| SL | Call | Exists? | Sentinel |
|---|---|---|---|
| **SL1** | a partial loop outside one admissible straight-edge fillet chain, §4.3a's one or two inner-root or two outer-edge chamfer, or §4.3b's box all-edge chamfer; two loops sharing an edge after route V's partition; or loops mixed with single edges for a `Chamfer` | yes | `ErrUnsupported` |
| **SL2** | an adjacent face outside LB3/LB4/LB6: a curved or oblique neighbour, a split side line, a neighbour whose own loop continues past the vertex on a curve, walls on both sides of `F` | yes | `ErrUnsupported` |
| **SL3** | retired: a `Fillet` of complete loops builds through Part L's fillet arm or refuses with that document's Table SF | — | — |
| **SL4** | an asymmetric route L reference with no unambiguous record-face identity | yes | reach SX16 |

Reach SX6, SX7, SX12, SX13, SX14 and SX15 keep their meanings per band, and
base S6/S7/S8/S9 per rewritten face.

Gate order for a brep or stacked `Chamfer`: after modify §4's stage 1,
§4.3b tries the exact all-edge box. Other selections then reach SX10,
SB2, SB1 and route P (brep-modify §6's stages 2a–2b):

| Stage | Gates |
|---|---|
| 2c. entry | independent straight edges → route E (brep-modify §6); complete planar-face loops → route L: LB1, LB2; an asymmetric selection outside both routes is SX16 |
| 3. topology | LB3, LB4, LB6 (SL2); LB5 (SX7) |
| 4. existence | SX6, SX13 per band as the contour is built |
| 5. audit | per band SX14, SX7, SX12 on the contour; per (pl) face S8, S6, S7, S9; per `F` S8, S7, S9 |
| 6. build | the record's closure; `capblend.BuildBand` per band (SX15 as each patch is oriented) |

For a `Fillet`, §4.3c tries the exact P8 cap edge before the brep receiver.
Route V also partitions complete loops and independent
straight edges before LB1/LB2. A selected straight-edge chain or one EB8
single edge on part of one planar-face loop, including a swept wall restated
as a plane, takes the partial fillet route
(Part V §2).

### 5. Tables BG and DG — results and downstream

| BG | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BG1** | route S shell | `brepPayload`, `stack` nil | the receiver's kept faces, the cavity's faces reversed, one rim per removed face; every edge on exactly two faces | `face(k)` / `wall(k)` by result index; no `capStart`/`capEnd` |
| **BG2** | route L chamfer | `brepPayload` with `loopBands`, `stack` nil | the rewritten record's faces plus the band patches | `face(k)` / `wall(k)`; each patch `chamferLoop(f,l,p)` for face `f`, loop `l`, patch `p` in its band's own order |
| **BG3** | rectangular-boss shell (§3.1a) | `brepPayload` with `bossShell`, `stack` nil | ordinary faces plus four cylinder patches and four ellipse seams; the closed arm groups the cavity as one void shell | `face(k)` / `wall(k)`; the patches carry `filletLoop` roles |
| **BG4** | circular-boss shell (§3.1c) | `brepPayload` with `bossShell`, `stack` nil | ordinary faces plus one whole-turn torus patch | `face(k)` / `wall(k)`; the patch carries a `filletLoop` role |
| **BG5** | box all-edge chamfer (§4.3b) | `allEdgeChamferPayload` (`breprecord.AllEdgeChamferPayload`) | six inset box planes, twelve bevel planes, one bore cylinder or none | `face(k)` / `chamfer(axis,side,side)` / `wall(6)` |
| **BG6** | coaxial counterbore shell (§3.1e) | `brepPayload` with `pocketShell`, `stack` nil | retained source faces, two cavity linings and floors, two opening rims, one whole-turn torus patch | `face(k)` / `wall(k)`; the patch carries a `filletLoop` role |

A BG1 result is an ordinary brep: general-boolean §4.5 reads it unchanged.
A BG2 result is a brep whose body carries extra faces, and each consumer
below reads the bands where the plain brep reader would miss them:

| DG | Consumer | BG1 | BG2 |
|---|---|---|---|
| **DG1** | mass properties, `Bounds` | general-boolean §4.3 with the cavity and rim faces' `delta` | §4.3 |
| **DG2** | `Verify` validity, tolerance gate | by construction; `brepGateDiameter` reads every vertex | the same; the bands' vertices are body vertices |
| **DG3** | `Tessellate`, STL/OBJ/3MF | general-boolean §4.4 | §4.4 for the record's faces, with one count per band walk shared by the trimmed wall, the patch and the contour (`agent-docs/tessellation-reach-design.md` §7's rule). `capblend.ChordLoop` chords the band's loop in `F`'s frame (`internal/brepbandmesh`), and the wall beside a (sw) walk takes the band's side-ring points as its samples, carried into its own frame exactly (`tessellation.ImposeBandWall`), so the wall's rim at the side level and the band's side ring are one set of vertices; a straight wall needs no count. `capblend.EmitBand` writes each band's strip and its departure terms. A band that rises off its face (`σ = +1`) turns its windings over. At `VerifyAll` the occupied-volume proof adds the band's chord-polygon volume (`CapBlendChordVolume` over the band's height alone) and swaps the vertex-motion allowance for the cap-blend twin's per-vertex motion array; a band DG4's rule does not admit leaves the mesh export-only |
| **DG4** | mesh boolean operand | class B over the face view where it admits, else the mesh path | the mesh path only; admitted under `capblend.OccupiedVolumeAdmission`'s rule per band (every corner a line–line miter, an exact G1 join or a whole turn), else `ErrUnsupported`; a prism cap-loop chamfer also admits a reflex corner (`agent-docs/tessellation-reach-design.md` §7, "Reflex connector"), but a route L band does not: a hole or pocket mouth, whose corners are apex cones, is not admitted (`brepbandmesh.OccupiedVolumeAdmission`, read by `booloperand.RequireVolumeProof`) |
| **DG5** | `ThroughAll`/`ToFace` | the per-face extremes; refuses at `delta > 0` as a prism does | the same, plus each band's extrema under the contour displacement |
| **DG6** | clearance | `clearance.BuildBrepBodyCarriers`; no model when any face carries `delta > 0` | no model: the pair reads `Suspect` unless its boxes decide it (reach DX6's staging) |
| **DG7** | undercut survey | per face | per face, and per patch through `Face.NormalAt` with reach DX7's three-valued rule and allowance |
| **DG8** | concave-radius survey | the cavity's dilated hole walls are read; undecided when any face carries `delta > 0` | undecided when any band holds a patch whose stamped departure is not exactly zero (reach DX8) |
| **DG9** | wall survey | staged `Suspect`, as for every brep. The wall behind every kept face is `t` by construction, but the material between two features the erosion left solid is not, and no reader proves which is which | staged `Suspect` |
| **DG10** | STEP | analytic: planes and cylinders | analytic where every patch is a `Plane` whose stamped departure is exactly zero; a corner cone patch, convex or reflex, states a nonzero departure from its tag and sends the whole body to the faceted writer, as does a cone with one circle loop (`agent-docs/step-export-design.md`) |
| **DG11** | a later `Fillet`/`Chamfer`/`Shell` | Tables RB/EB/SB/TC/LB over the record; RB1 needs `delta = 0`, so a shell whose offsets are exact floats takes route E or L again, and one whose offsets rounded is SB1 | the same over the rewritten record; an edge of a band patch is not in the record and matches nothing (route E's SB6, route L's SL1) |
| **DG12** | `Transform`, `Mirrored`, `PatternCopies` | re-lifts every face frame | the same; `loopBands` are re-attached by the re-evaluation, as `capBlendPayload.Placed` re-derives its bands |

BG3's and BG4's consumer rules are stated in §3.1a and §3.1c.
BG6 reads §3.1b's downstream gates and §3.1e's proof.
BG5's measurement, tessellation, and placement rules are stated in §4.3b.
Other record-based consumers keep their existing payload gates.

### 6. What stays refused, and why

| Request | Code | Why not here |
|---|---|---|
| chamfers of edges sharing a vertex outside one loop, or fillets whose selected edges share no planar loop after straight-wall restatement | SB5 / SL1 | the mixed chamfer's corner plane has no reference-axis normal; route V and the partial fillet admit the cases in Part V |
| an edge ending on a blend or a curved face outside P8's exact front top edge (§4.3c) | SB7 | the complete-loop fillet builds P8's top loop (Part L); other partial edges need a source and cutter-contact proof |
| shell of a blind pocket outside §3.1b, a counterbore outside §3.1e, a blind port, a stacked union outside §3.1a or §3.1c, or a keyway (P2, P3, P6c) | SG3 | the remaining cases need a shape-specific exact transition and occupied-volume proof |
| an outward shell, or a closed shell outside §3.1a and outside a brep record that reads as a prism (along one axis only, outward) | SG1 / SG2 | §3.2 |
| shell removing a curved outer wall (a fillet cylinder, P8), or a wall run ending at a reflex corner (P7's `y = 8` wall) | SG5 | the rim at a curved outer wall is a swept face less the cavity's trace, and a reflex end's rim lies inside the material along the removed carrier; the rim assembly states only planar regions on a removed face |
| the surveyed modify requests on faceted P6b, and on P9 other than the equal-setback chamfer of its hole mouths (retained-edge-blend R7, §4.9) | reach SX9 | these requests have no retained source route; a complete tilted plate-to-cylinder junction has the separate bounded `Fillet` in Part A |

### 7. Decided questions

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
- **`Outward` refuses on route S, and `WithNoOpenings` builds only through a prism reading.** §3.2; an outward route S shell and a closed shell of a record that reads as no prism have no record. An outward closed shell takes the prism reading only when it is the record's one reading.
- **Asymmetric route L chamfers use the existing two-distance band.** The
  reference face chooses `dc` and `ds` consistently per loop. A stacked
  receiver's face view still lacks the required public face identity (SL4).
- **Hand-offs: none.** No capability is needed from `sketch`, `r3` or
  `units`.

### 8. Do not do this

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
  swept there (Part L §2).

### 9. Required tests

Every test asserts computed geometry against a closed form, bounds as
relations, through the public booleans. Fixtures are §1's parts.

Route S (§3.1d): P1 with its radius-3 cylinder removed at `t=1` has 14
analytic faces, two radius-3 cylinder rims at `y=[0,1]` and `[19,20]`,
volume within its bound of `3688−18π`, `Verify` `Sound`, and a
`Tessellate(VerifyAll)` boundary and occupied-volume proof. A thickness that
puts the port outside the inset rectangle refuses SG4 without retiring P1.

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
  third planar face across z; a removed hole wall outside §3.1d → SG4; a removed fillet cylinder → SG5; P1 with a second Ø6 hole along y 7 mm from the first → SG6
  (the dilated holes meet); P1 at 0.1 in → SG6; a U section with a 1 mm
  slot, drilled through one arm → SG6 through TC7; `Outward` → SG1;
  `WithNoOpenings` builds P1's closed shell through its prism reading along y
  in either sense, and outward on a brep box that reads along three axes →
  SG2. A hole 1.5 mm under the top on a 2 mm shell builds
  through the crossing reach: the cavity's top splits in two, the rim holds
  both pieces, and the volume is `5632 + 220π − 16·(25·acos 0.9 − 4.5·√4.75)`.
  Two cuts refuse on the cavity's charge (§3.3 step 4's rules), SG6 (the two inputs below are worked examples that no
  test pins):
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
  P8's top-loop chamfer and its hole-rim chamfer export faceted STEP, the
  top loop's corner cones being band patches whose tags state a departure
  (`agent-docs/step-export-design.md`), and P2's top loop analytic; `Verify`
  `Sound` on each.
- The undercut survey lists P3's rim cone, which faces up and out, under a
  pull along `−z` and clears the body along `+z`. The concave-radius survey
  decides P2's top loop (planes alone) as holding no concave feature, and
  reads P2's mouth and P3's root undecided: their cones carry a non-zero
  stamped departure. A clearance pair against P2's result reads `Suspect`
  where its boxes do not decide it.

### 10. PR split

Route S lives in `internal/throughshell/` (`ShellCut` is its entry) and the stacked builders in `internal/stackedshell/`.
Route L lives in `internal/brepmodify/loop_route.go` (`brepLoopRoute`, `admitLoops`) and `internal/breploop/record.go`
(`rewriteLoopFaces`); its band mesh is `internal/brepbandmesh/`. §9's S-1, S-2, L-1 and L-2 name fixture groups: S-1 is
route S with removed caps, S-2 a removed wall run, L-1 route L's construction and L-2 its downstream consumers (DG3,
DG4, DG6, DG7, DG8, DG10, DG12). All four have shipped. Still refused: §6, and a banded body's mesh boolean where a
band holds an apex cone (DG4).

## Part B — Brep modify

How `Fillet`, `Chamfer` and `Shell` take an analytically
trimmed boolean result — a `brepPayload` (`agent-docs/prism-boolean-design.md` Part G §4,
"general-boolean §N" below) — and, through its face view, a
`stackedPrismPayload` (`agent-docs/prism-boolean-design.md` Part S). Companion to
Part F ("modify §N"), which owns the three call contracts,
the base gates, the section rewrite and its §5 audit, and to
Part R ("reach §N"), whose RX3/RX7 rows and SX10/SX16
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

### 1. Problem

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
| counterbore | stacked → face view | none | rims as complete loops (Part L) |
| boss on plate (stacked) | stacked → face view | none | boss edges; root circle as a complete loop (Part L) |

A complete-loop fillet with independent straight edges builds through route V
(Part V). A curved rim or a complete loop without
single edges uses route L (Part L).

### 2. Routes and dispatch

Each op keeps modify §4's stage 1 (S17, the sheet refusal, options, S15, S13 or
S14, S16) and reach SX10 for a `capBlendPayload`. Where an op would otherwise cast to
`prismPayload`, it calls `brepmodify.Receiver` (`internal/brepmodify/prism_route.go`):

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
route E or Part G's route L (Fillet/Chamfer), or
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
(Part G §4.2). A selection outside those routes
remains SX16.

#### 2.1 A stacked shell's rounded cavity: the faceted view

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
   (Part E) builds the result from the view.
   Its strip check (SR7) reads every other facet of the view, the cavity
   walls and band facets included, as an obstacle, so a blend that would
   break through the wall into the cavity refuses.

Only a mapped selection that the retained blend recognizes and builds
succeeds. A selection the retained blend recognizes but cannot build refuses
with the retained blend's own row (Table SR of
Part E). On the box below, a cavity
vertical or floor edge refuses with "an end face's corner does not match the
edge's convexity", and an edge at the band's ring with "an edge end meets a face
that is not a retained plane". The opening's inner rim builds: the annular top
is reflex at the rim's ends, and the inner walls across them take the blend's
end caps. An unmapped selection, or one the retained
blend does not recognize, falls through to SB11. A placed shell whose held
vertices do not equal its view's vertices exactly maps nothing: the box shell
translated by 12.7 along x, or turned 30° or 90° about z by
`r3.RotationAround`, refuses with SB11. A `Shell` that removes faces of such a
body is SB11 without a view, and a closed one is modify-general SG2. The result
is a `facetedPayload`: its volume, area and box bounds grow by the view's
band chords, and every later reading, modify op and Boolean reads its held
mesh. The view's root band takes the ring count and the cell charge of
loop-fillet §7.1, so for the 60×40×20 box with a 20×20×10 boss
shelled `t = 2` and filleted at `r = 3` the volume and area bounds stay below
`Verify`'s default relative limits, and `Verify` reads the result `Sound`
(`apitest/shell_then_blend_test.go`).

For the box above, open at the boss top, `Fillet` of the four outer box
verticals at `r = 3` leaves `√2·t − (√2−1)·r = 1.586` mm of wall on each
corner's diagonal and removes `4·20·9·(1 − π/4)` mm³. At `r = 7`, past
`√2·t/(√2−1) = 6.83`, the strip meets the cavity and SR7 refuses.

#### 2.2 A displaced prism: the faceted view with exact carriers

A complete cap-loop `Chamfer` of exact loops of a receiver with a valid carrier record reaches this view only after `brepmodify.CarrierCapLoopChamfer` declines or refuses (carrier-record-design §8.1): a selected loop with a nonzero entry, a partial or mixed selection, a gate, offset audit or transfer-condition refusal, and SX6's drop all fall through to it, and only cancellation is returned.

A `prismPayload` with a section displacement refuses the corner rewrite
(`prismrecord.Payload.RequireExactSection`), since the rewrite reads the
displaced coordinates. The usual source is a `Cut` by rotated pattern copies:
each copy records its rotated section to the nearest float and charges that
rounding as its section displacement (mirror-pattern §4.3), and the `Cut`
carries it into the result (prism-boolean §7). The rounding is real: the
denoted holes have irrational coordinates, so no float record states them.
The target's own loops still enter the result verbatim, and the `Cut` lists
them as loops with exact carriers (prism-boolean §7, "Exact carriers").

A `Chamfer` that the corner rewrite refuses this way runs on the body's
faceted view (`exactCarrierViewBlend`,
`internal/brepmodify/exact_carrier_view.go`), by §2.1's three steps with one
change to step 1. The view keeps a retained tag on each cap and on each side
face whose loop has exact carriers (`prismrecord.Payload.CarrierExact`). A
side face of any other loop, or a face whose roles name no single loop, keeps
no tag: its recorded surface lies only within the displacement of the true
one, so its facets are plain obstacles. The tessellation charges the section
displacement into every facet bound (tessellation §2, §5), so the strip check
(SR7) clears the true holes, not only the recorded ones. The retained blend
rewrites only faces with an exact tag, so a selection on a displaced loop is
not recognized and keeps the `RequireExactSection` refusal. A receiver whose
carrier record is valid (`agent-docs/prism-boolean-design.md` Part K §3) first tries
the lateral carrier route (§7 there); when Table CB refuses a `Chamfer`, the
call falls through to this view exactly as for a receiver with no record. The
result is a
`facetedPayload`, as in §2.1. For the 3 mm rounded-triangle plate of
`apitest/chamfer_pattern_socket_test.go` with three pattern-copied sockets, a
0.5 mm chamfer of both outlines in one call builds.

#### 2.3 A stacked or slab-cut BRep: the faceted view after the receivers

A `Fillet` of a `brepPayload` that keeps its slabs (`Stack`: an A1 union,
general-boolean §3 "A1 as a brep") or that a blind Cut built through the same
build (`SlabCut`: general-boolean §3 A7, A9) first takes every route of
Table RB. When those refuse, the call runs the retained edge blend on the
body's faceted view (`stackedViewBlend`, `internal/brepmodify/stacked_view.go`)
by §2.1's steps. A selection the blend does not recognize keeps the receivers'
refusal; a recognized selection the blend cannot build takes the blend's own
refusal. So every selection the receivers build keeps its result, and a
section displacement (a pocket crossing a round hole) or a partial loop
route L declines reaches the blend.

A planar record face (`face(k)`: a cap, an interface face, a wall on an
axis-aligned plane) lies on its recorded plane and keeps its retained tag. A
swept wall (`wall(k)`: a circular or oblique wall) is rewritten between
canonical vertices: an arc takes its radius from its first vertex and an
oblique plane passes through its walked ends, so it lies only within the
record's section displacement of the wall it denotes, and a holed union's
merged circle carrier lies `CarrierCharge` from it. Where either is nonzero,
a circular wall keeps its `Cylinder` tag and states `w = max(SectionDelta,
CarrierCharge)` as the tag's `AxialDelta` (`stackedViewCharge`,
`internal/brepmodify/stacked_view_charge.go`): its circle lies within `w` of
the circle it denotes all the way round, since the record's bound covers their
centre offset plus radius difference (general-boolean §3, Width band). The retained blend charges that
wall in a closed rim and refuses it elsewhere (retained-edge-blend §6.1,
"Displaced planes and walls"), and its result keeps each charged wall with its charge. An
oblique swept plane keeps its `Plane` tag with its tilt as `TagDeparture` and
its offset over the face as `ExtentDelta` (`stackedViewTilt`,
`agent-docs/charged-window-chamfer-design.md` §2). Every other swept wall, an
oblique plane that fails that section's gates among them, stays in the view
untagged, a plain faceted obstacle as in §2.2. A boundary's vertex
allowance is charged into the facets' bounds. A refusal raised while building
the view, before the blend reads the selection, keeps the receivers' refusal.
The result is a `facetedPayload`. `apitest/stacked_brep_view_fillet_test.go`
fillets a plate edge of an A7 result and of an A1 union this way and pins the
selections that reach the blend, and
`apitest/stacked_brep_view_charged_wall_test.go` fillets a round hole's rim on a
charged holed union.

### 3. Table RB — receivers

| RB | Receiver | `Fillet` / `Chamfer` | `Shell` |
|---|---|---|---|
| **RB1** | `brepPayload`, `sectionDelta() == 0` | route P, else route E (Table EB) or route L (modify-general §4); an asymmetric chamfer skips P | route P, else route S (modify-general §3) |
| **RB2** | `stackedPrismPayload` whose `breprecord.OfStacked` succeeds (one region per slab, not a group) | as RB1 over the face view; asymmetric references use the source-face role at each record index | as RB1 |
| **RB3** | `brepPayload` or stacked receiver with a section displacement | route P's displaced reading (§4.4) for a lateral Fillet or symmetric Chamfer that passes Table PD; otherwise SB1 | SB1 |
| **RB4** | `facetedPayload` | reach SX9 (permanent) | reach SX9 |
| **RB5** | `brepPayload` carrying a boss or pocket shell band | the faceted view's retained edge blend (§2.1), else SB11 | SB11; a closed shell is modify-general SG2 |
| **RB6** | `prismPayload` with a section displacement | With a valid `CarrierRecord` of the receiver body (`prismrecord.CarrierRecordOf`) and a lateral selection (carrier-record-design §7): the carrier route; a Table CB refusal is `Fillet`'s `ErrUnsupported`, and `Chamfer` falls through. Otherwise `Chamfer`: the faceted view's retained edge blend over loops with exact carriers (§2.2; a loop is exact when `ExactCarriers` lists it or the body's own record holds every entry of it at zero), else `RequireExactSection`'s refusal; `Fillet`: that refusal. A `Chamfer` that selects every edge of complete cap loops, each exact in that record, builds a `capblend.DisplacedPayload` (carrier-record-design §8.1); every refusal of that route but cancellation falls through to the rows above | that refusal |

RB1's zero-displacement rule is prism-boolean §13's displaced-receiver rule,
the one `prismrecord.Payload.RequireExactSection` enforces for a prism: a rewrite of a record that
only denotes its body to within a displacement has no proven displacement of
its own. Level displacements (`z0Delta`/`z1Delta`) pass through untouched;
no construction below moves a level.

The receiver is keyed on its payload, not its history: a transformed brep reads as
the brep it was placed from.

### 4. Route P — the prism in disguise

#### 4.1 Recognition

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

#### 4.2 Selection and dispatch

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

#### 4.3 Why this is not general-boolean §8's "re-sweep"

General-boolean §8 forbids a boolean from re-sweeping a box along the tool's
axis: the roles would change under the caller and the next cut would find no
prism. Neither objection reaches a modify op. Its result mints fresh roles
anyway (modify §11), and the prism it returns composes as any prism does: a
next perpendicular cut takes class B and a co-directional one class A.

#### 4.4 A displaced class-B receiver: carriers, not vertices

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

### 5. Route E — axis-parallel straight edges

#### 5.1 Table EB — the admitted edge

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
record face (Part G §4.2), using the same source-role
mapping for a stacked receiver.

#### 5.2 Restatement of a straight wall as a planar face

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

#### 5.3 The construction

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
   The agreement is expected, not assumed. Each end face reads its corner in
   fixed roles: a fillet takes the walk beside `F1` (the edge's first
   adjacent face) as the arriving walk, reversing both walks where the face's
   loop runs the other way round the corner, and every straight carrier
   passes through the corner itself (`offset2d.FilletAtCorner`). The two end
   faces then hold the same two carriers in the same roles, and the
   operations the construction performs on coordinates — differences, sums,
   products, `hypot`, the offset solve — commute with negating them, also
   inside a fused multiply-add; under a reflection the corner's cross
   product, the side the offsets take and each line's left normal change
   sign together. Where the map between the faces also exchanges the two
   coordinates, the agreement is bit for bit only without fused
   multiply-adds, since Go may fuse `a·b − c·d` (on arm64, for one) and the
   exchange moves which product is fused; a disagreement there refuses SB9
   and never admits a wrong blend. A chamfer's feet
   are set back from the corner along each walk separately, so it needs no
   fixed roles. SB9 refuses wherever the expectation fails. Read in each
   face's own loop order through each walk's far end instead, the line
   solve's base point and the order of its operands differ between the faces,
   and the feet part by an ulp on an ordinary box. (The `Atan2`-based cutbacks
   may differ by rounding; each face's audit reads its own.) `G1`'s rewrite takes its own
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

#### 5.4 Maps between faces, and the 2D answers

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

### 6. Table SB — refusals and gate order

Modify §1's test picks every sentinel: a body that does not exist is
`ErrDegenerate`; one this evaluator cannot build is `ErrUnsupported`.

| SB | Call | Exists? | Sentinel |
|---|---|---|---|
| **SB1** | a brep or stacked receiver with `sectionDelta() != 0` outside §4.4 | yes; its rewrite has no proven displacement | `ErrUnsupported`, naming the displacement as `prismrecord.Payload.RequireExactSection` does |
| **SB2** | a stacked receiver `breprecord.OfStacked` refuses (a prism group, several regions in one slab; a stack enclosing a cavity, a closed shell) | yes | that call's `ErrUnsupported` |
| **SB3** | route P reads a prism along some axis, a Shell's removed faces are not its caps, no other axis admits them, and route S reads no through-cut record along any axis (modify-general Table TC) | yes | modify S2, with SG3's reason |
| **SB4** | route E reading an edge that is not a straight line along a reference axis; only single straight edges reach route E, and every other Fillet or Chamfer selection takes route L (modify-general §4, Part L), which builds a hole rim, a boss root or a cornered loop or refuses with Table SL or SF | — (a falsifier) | `ErrUnsupported` |
| **SB5** | selected single straight edges share a vertex outside route V's complete-loop or selected-chain fillet (Part V §2); a mixed `Chamfer` still takes SL1 | yes | `ErrUnsupported` (SL1's text) |
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
SM2 wording. An undecided or non-tangent junction keeps the curved-face text. `Chamfer` extends such a selection
along its chain first (modify-reach §5.1), so it reaches this text only when that expansion is undecided.

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
| 2c. route E entry | route S (Shell; modify-general §3.4); independent straight edges take route E; a `Fillet` of complete loops with independent straight edges takes route V (Part V §2); other selections take route L or refuse; EB1/SB4 and EB7/SB5 apply to route E's set |
| 3. edge topology | EB2/SB6; the restatement passes (§5.2): EB3/SB7, then EB4/SB8; EB5/SB8; EB6 |
| 4. construction | per edge: S4, S5 in `G0`; the recomputation in `G1`, SB9; EB8/SB8 before rewriting |
| 5. audit | per planar face: S8, S6, S7, S9; per trimmed wall: S6 |
| 6. closure | `breprecord.Falsify`, `breprecord.TopologyContext`, `breprecord.Build` |

The existence question is asked first wherever two gates meet one input: S4
and S5 (stage 4) decide whether the blend exists before any trim is measured
(S6, stage 5), and S8 leads the audit, as modify §4 orders them.

### 7. Table BB — results and roles

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

### 8. Table DB — downstream

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

### 9. Required tests

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
  (Part G §3.2): `6272 + 220π`.
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
  of 8 faces, volume within its bound of `15920 + 20π`, the `y = 0` face's
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
  (Part L §8).
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
- A 7×10 box sketched on `y = 0` (and one on `y = 9.2` at `z ∈ [42.5, 60.5]`)
  and extruded 3.6 along `y`, each edge along `z` filleted alone at
  `r = 0.3` (`apitest/brep_route_e_fillet_end_faces_test.go`): every edge
  builds, the floor's and top's feet agreeing bit for bit, the volume within
  its bound of the box less `ℓ·r²(1 − π/4)`. Read in each face's own loop
  order through the walks' far ends, the edges at the sketch plane refuse
  SB9; `internal/offset2d/blend_test.go` pins the fixed-role reading against a
  reflected corner whose walk runs to another far end.
- Trapezoid bosses on the 40×40 plate, each vertical edge filleted alone at
  `r = 0.3` (`TestBrepRouteEFilletObliqueBossEdges`): every edge builds.
  The walls beside these edges are oblique, so their carriers are not
  axis-aligned, and read with each face's own walk arriving some edges
  refuse SB9.
- S1, chamfer the edge along `x` at `(y, z) = (0, 0)`, `d = 2`: the `x = 0`
  and `x = 40` walls are restated, volume within its bound of
  `16000 − 180π − 80`.
- Chaining: chamfer S1's edge along `z` at `(0, 0)`, then chamfer the edge
  along `x` at `(y, z) = (0, 0)` on the result → SB7. Filleting all twelve
  straight edges of S1 takes route V (Part V).
- Consumers on the S1 fillet result: its volume reads within its bound of `15920 + 20π`
  (`TestBrepSingleYCapEdgeAfterRestatement`, the edge along `y` at `(x, z) = (0, 20)`; the edge along `z` removes the
  same `20·r²(1 − π/4)`). No test runs `Verify`, STEP export, the undercut survey or a translation on this single-edge
  result; those consumers are covered on loop fillets (`TestVertexBlendThreeEdgeOpenChain` for `Verify`,
  `TestBrepLoopFilletPublicSTEP`, `TestBrepLoopFilletUndercutReadsEachPipePatch`).

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

### 10. Do not do this

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
  fillet meets a straight-edge fillet (Part V).
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

### 11. Decided questions

- **Faceted receivers**: never. Reach §11's reasoning stands; the mesh is one
  evaluator's decomposition, not a carrier.
- **A prism receiver's straight cap edge through its face view**: route E
  takes a single selected edge for Fillet or Chamfer when Table EB admits it.
  Modify Table S (S1) keeps other partial cap selections staged.
- **Route P before route E**: yes; a `prismPayload` result reaches more
  consumers and further modify ops.
- **Shell of a non-prism brep**: route S of Part G
  for a prism cut by through tools; SB10 for the rest.
- **Hand-offs**: none. No capability is needed from `sketch`, `r3` or `units`.
- **An edge that ends on a sloped end face**: route E keeps SB7. The record
  holds an oblique plane only as a swept wall, so the end cap, an ellipse
  arc in that plane, and the blend face cut at an angle have no face this
  record holds. A body that a mesh Boolean built takes the retained edge
  blend instead, whose straight route ends rows on any retained plane and
  takes the end cap where a face beside the edge is reflex
  (Part E §5.0).

### 12. PR split

The brep route is implemented in `internal/brepmodify/`. Each piece below ships with the tests of §9;
`doc.go`'s support map and `agent-docs/missing-features.md` state what it lifted.

| Piece | Code |
|---|---|
| Receiver dispatch, SB1, SB2 | `brepmodify.Receiver` (`prism_route.go`); the stacked entry is `breprecord.OfStacked` |
| Route P (P1–P5, SB3) | `prism_route.go`; `brepgeom.AxisFrame` |
| Route E (Table EB, SB9, blend face) | `edge_route.go`; `brepgeom.BrepCornerAt`, `brepgeom.MapSegment` |
| Restatement (§5.2) | `brepgeom.Restate`, `brepgeom.PlanarFrame` |
| Asymmetric reference-face mapping (§5.1) | `asymmetric.go`; route L's side in `loop_route.go` |
| Displaced reading (§4.4, Table PD) | `displaced_blend.go`; `breprecord.Face.WallCarrier` |
| Faceted views (§2.1–§2.3) | `shell_view.go`, `exact_carrier_view.go`, `stacked_view.go` |

Still refused: the selections Table SB lists, a `facetedPayload` receiver (reach SX9), and an asymmetric selection
that route E and route L both decline (reach SX16).

## Part L — Loop fillet

`Fillet` of a complete loop of a planar face: a prism's cap loop, a
brep or stacked result's planar-face loop, a hole rim or boss root, and the
honest form of a top edge ending on a fillet cylinder (brep-modify SB7). Companion to
Part G ("modify-general §N"), whose route L this
document extends with a fillet arm, and to Part R
("reach §N"), whose §8.2 corner rule it replaces. Part F
("modify §N") keeps the call contract and the §5 audit;
Part B ("brep-modify §N") keeps the brep receiver's
gate order.

Five tables are normative:

| Table | Owns | Section |
|---|---|---|
| **LF** | the result surface and edge kind per corner class of the loop | §2 |
| **RF** | receivers, and how each reaches route L | §3 |
| **SF** | every refusal the fillet arm adds, with its sentinel and gate order | §6 |
| **BF** / **DF** | result payload and what every consumer reads | §7 |
| **CF** | the strip coefficients every measurement reads | §5.1 |

### 1. The one idea: route L's band with a circular height map

Route L's chamfer band (modify-general §4.2) is the exact offset family of
the loop `ℓ` on face `F`: at axial fraction `s` from the side level toward
`F`, the body's section is `F`'s region with `ℓ` offset by `s·d` into `F`'s
material. The fillet of the same loop at radius `r` is the SAME family under
a different height map. A ball of radius `r` touching `F` and a wall beside
`ℓ` has its centre `r` below `F` (toward the walls) and `r` from the wall
into `F`'s material; the surface it sweeps, read at distance `h` from the
side level `sideZ = L_F ∓ r` toward `F`, is the loop offset by

```text
t(h) = r − √(r² − h²),   0 ≤ h ≤ r,   t(0) = 0 (the side contour, ℓ itself),  t(r) = r (the cap contour)
```

Every level curve of the fillet band is an exact 2D offset of `ℓ`: the lines
move along their inward normals, a circular wall's offset is the concentric
circle, a reflex corner of `F`'s region closes with the connector arc of
radius `t` about the corner, and a convex corner's two offsets cross at the
miter foot. The chamfer uses the linear map `h = t·ds/dc`; the fillet uses
the circular one. So the fillet arm reuses, unchanged:

- the cap contour at `t = r` (`capband.LoopBoundary`, modify-general §4.2 step 1),
  its displacement (`capband.ContourDisplacement`) charged into `F.delta`;
- the side contour, `ℓ` moved `r` along `F`'s normal (step 2), with
  `levelDelta`;
- the trims of the (sw) and (pl) faces beside the loop (step 3), the face
  audit (step 4), the record's closure (step 6);
- Table LB's admission (LB1–LB6) and the gates SX6, SX7 (as `requireBandReach`
  sums it per wall piece, so two bands on one wall from both ends refuse
  where `r₁ + r₂ ≥ h`), SX12, SX13 and SX14, each on the offset at `r`.
  SX12's argument holds under any monotone height map: a crossing anywhere
  in the family occurs no later than at the full offset.

What changes is the surface between the two directrices, the curve of a
convex corner's miter, the measurements, and the mesh.

### 2. Table LF — surfaces and edges per corner class

The loop's walks are resolved as route L resolves them
(`capblend.OneLoopCornerLoop`, `capband.OffsetJoins`). Each walk and each corner of `ℓ`
yields one patch or one edge:

| LF | Piece of `ℓ` | Patch (face) | Trimmed by |
|---|---|---|---|
| **LF1** | a straight walk of length `ℓᵢ` | `Cylinder{Origin: the wall's line offset r into F's material, at sideZ; Axis: the walk direction; Radius: r}`, the quarter from the contact line on `F` (the contour segment, at `L_F`) to the contact line on the wall (`ℓᵢ` at `sideZ`) | its two end curves (LF4–LF6) |
| **LF2** | a circular walk, radius `R > r`, centre `C`, material inside the circle | `Torus{Center: C at sideZ; Axis: F's normal; Major: R − r; Minor: r}`, the quarter of the tube from its top (radius `R − r` at `L_F`) to its outer equator (radius `R` at `sideZ`). `R − r < r` is a spindle torus whose outer quarter is the patch and never meets its axis | LF5/LF6 ends, or none on a whole turn |
| **LF3** | a circular walk, material outside the circle (a hole rim) | `Torus{Center: C; Axis: F's normal; Major: R + r; Minor: r}`, the quarter from the top (radius `R + r` at `L_F`) to the inner equator (radius `R` at `sideZ`) | the same |
| **LF4** | a convex corner where two straight walks meet at interior angle `θ` (material side) | no patch: the two cylinders meet along their intersection | the edge is a planar quarter ellipse in the corner's bisector plane, `Ellipse3{Center: the ball centre at the corner, c = corner + (r / sin(θ/2))·b̂ at sideZ; Axis: the bisector plane's normal; Major: b̂; SemiMajor: r / sin(θ/2); SemiMinor: r}`, from the corner vertex at `sideZ` to the contour corner at `L_F`; `b̂` is the in-plane bisector into the material |
| **LF5** | a G1 join (a line tangent to an arc, or two tangent arcs), route L's dead-zone rule | no patch: the two pipes share a meridian | `Arc3`, the quarter circle of radius `r` in the plane through the join normal to `ℓ`, from the join vertex at `sideZ` to the contour foot `v + r·n̂` at `L_F` |
| **LF5′** | a near-tangent join: a line and an arc, or two arcs, that the offset reads as G1 while their exact tangents turn by a sliver (§5.5) | LF5's two pipes, meeting at the G1 foot the contour holds | LF5's `Arc3` meridian on that foot; §5.5 charges the sliver in every reading |
| **LF6** | a reflex corner of `F`'s region with turn `ψ` (a hole or pocket mouth's corner, a square boss root's corner) | `Torus{Center: the corner at sideZ; Axis: F's normal; Major: r; Minor: r}`, the horn torus whose tube runs from the connector arc of radius `r` at `L_F` to the apex on its own axis at `sideZ`, over azimuth `ψ`; both joins to the neighbouring patches are G1 | two LF5 `Arc3` meridians, at the connector arc's two ends `pA`, `pB`, each in the plane normal, at the corner, to the walk whose wall it bounds: `pA`'s Axis is the corner tangent of the walk before the corner, `pB`'s that of the walk after it, which for a straight walk is the walk's own direction |
| **LF7** | a whole circle | LF2 or LF3 over `2π`; two `Circle3` edges and no seam | — |
| **LF8** | a sharp convex corner involving a circular walk | no patch: the two analytic pipes meet along their intersection | `FilletMiter3` follows the proven offset-foot locus from the side vertex to the cap foot; SF1 refuses a fold or an unbounded locus |
| **LF9** | inward open arc with both recorded endpoints exactly at `r` and straight G1 neighbours | sphere from side arc to cap pole, bounded by two LF5 meridians (Part V §1) | three arcs |

The sphere does not appear at a sharp convex corner. Reach §8.2 states a trimmed `Sphere` at a
zero-length miter; this document replaces that row (§9). At a convex corner
of one loop, the ball touching `F` and wall `A` reaches the centre `c` where
it also touches wall `B`, and from `c` it rolls along `B`. The ball never
rests at `c`, so no spherical patch is swept; the material the two blends
remove is the union of the two per-wall wedges, whose boundary inside the
body is cylinder `A` where it lies inside cylinder `B` and the converse,
meeting along the ellipse of LF4. A sphere octant at `c` would remove more:
its lower boundary arc lies in the plane `z = sideZ` on the circle of radius
`r` about `c`'s projection, and runs from `(r, 0)` to `(0, r)` in corner
coordinates, through `(r − r/√2, r − r/√2)`, a point strictly inside both
walls. Below that arc the body would keep its full corner square at
`z = sideZ` and above it only the quarter disk, so a flat shelf of area
`r²(1 − π/4)` would be exposed at the side level. No CAD kernel produces that
surface for two blended edges meeting at an unblended third: Fusion's fillet
of a box's top four edges shows the two blends meeting along a curved seam
and no third face. The sphere octant appears after the lateral edge is
filleted too; route V builds it through LF9 (Part V).

At height `h` the offset corner of two lines lies on the bisector at distance
`t(h)/sin(θ/2)` from the corner, so the miter curve is
`{corner + (t/sin(θ/2))·b̂ + h·n̂ : t = r − √(r² − h²)}`, which satisfies
`((u − r/sin(θ/2))·sin(θ/2))² + h² = r²` in bisector coordinates `(u, h)`:
the ellipse of LF4, which is also the section of either cylinder by the
bisector plane. A line–circle or circle–circle convex corner's foot locus is
a conic in the plane (reach §8.3), and lifted by `t(h)` it is a space curve.
`FilletMiter3` names that edge without replacing it with a chord or a NURBS.
LF8 admits a regular offset locus. Its mass and patch area readings widen
the polynomial strip result by certified corner allowances, and its mesh
places each seam station on the offset intersection. The allowances may
make the default verification tolerance report `Suspect`.

`Ellipse3` is a sealed `Curve` variant (api §4): `type Ellipse3 struct { Center, Axis, Major r3.Vec; SemiMajor,
SemiMinor units.Value }`, the arc swept counter-clockwise about `Axis` from
the start vertex to the end vertex, as `Arc3` is. `Major` is the unit
direction of the semi-major axis. Its parameters are exact for the curve they
name (api §4); the curve sits within `F.delta` of the one the fillet denotes,
since `c` is the offset corner's float solve. Every production switch on
`Curve` carries a default (`geometry.go`'s rule), so `Ellipse3` has an arm only
where the default would be wrong: `surfaceplace/placement.go` (map `Center`,
`Axis`, `Major`), `modifyselect.EdgeContext` (render centre and semi-axes),
`brepEdgeMatches` (match nothing: a band edge is in no record); the
default is right in
`stops.go`, `internal/clearance/pairkernel/gate.go`, `internal/clearance/pairkernel/geom.go` and `selector.go`
which read an unknown curve through its two vertices or refuse it. `Edge.Length`
of an LF4 edge is the quarter-ellipse arc length
`∫₀^{π/2} √(b² + (a² − b²)·sin²φ) dφ`, `a = r/sin(θ/2)`, `b = r`, enclosed by
the monotone integrand's lower and upper Riemann sums over 64 equal
sub-ranges; the held value is the midpoint and the bound the half-width.

Edge convexity: the contact edges on `F` and on the walls, and every LF5
meridian, are tangent junctions and carry the band's sense (`IsConvex` true
on a band that removes material, false on one that fills); an LF4 ellipse
carries the loop corner's own 2D turn, the sign the receiver's vertical edge
there carried.

### 3. Table RF — receivers

| RF | Receiver | Path to route L |
|---|---|---|
| **RF1** | `brepPayload` | brep-modify §6 stages 2a–2b, then route L (modify-general §4.4 stage 2c); the fillet arm replaces SL3 |
| **RF2** | `stackedPrismPayload` | the same through `breprecord.OfStacked` (SB2 first) |
| **RF3** | `prismPayload` whose selection holds a cap edge | `fillet.go`: where `modifyselect.MatchCorner` finds no lateral corner for a selected edge, the call takes `breprecord.OfPrism(pp)` and dispatches route E, L or V by the selection (Part V §2). The result is a brep body; every-lateral selections keep the prism path and B1's roles |
| **RF4** | `revolvePayload` cap edge | SX5 unchanged: the cap's adjacent faces are revolve surfaces, not faces swept along the cap normal (LB3 fails), and the blend of a planar loop whose walls are revolved surfaces is a pipe about a general planar curve that no level set of an offset family states |
| **RF5** | `capBlendPayload`, other draft, sweep, loft, faceted | SX10, modify S3, SX9 unchanged. A positive-taper full-circle draft's far outer cap rim takes draft §10.4 before S3 |

RF3's result is a `brepPayload`, not a `capBlendPayload`: the fillet patches
and their readers (§5, §7) live once, on the band record, and the prism cap
chamfer's payload is not given a second patch family. A prism cap-loop fillet
therefore returns `face(k)`/`wall(k)` roles with `filletLoop(f,l,p)` patches,
where its chamfer returns BX3's `capStart`/`side(i,j)`/`chamferCap`. §9.

Selection: Table LB unchanged. A `Fillet` whose matched edges are whole loops
of planar faces, of any segment count, takes the fillet arm; a single whole
circle takes it too (SB4). Route E keeps every selection of
single straight edges along axes (`singleStraightEdges`) whose endpoint-face
corners agree with the interior material wedge (brep-modify EB8). One straight
edge that fails EB8 takes the selected planar-loop band if that edge belongs
to a proper subset of a planar face's loop. The selected face and loop stay
fixed when restatement turns the wall beside the edge into another planar
face. If no partial band admits the edge, EB8 refuses the call.

### 4. Construction

For each admitted loop `ℓ` of face `F` (normal axis `n`, level `L_F`), radius
`r` with its conversion bound `rDelta` (`capSetback{dc: r, ds: r, dcDelta:
rDelta, dsDelta: rDelta}`), sense `σ` from LB6:

1. **Record rewrite.** Modify-general §4.2 steps 1–4 and 6 verbatim with
   `d = r`: the cap contour, `F`'s region rewritten, `sideZ = L_F + σ·r`
   (`brepLoopBand.sideLevel`), the (sw)/(pl) trims and audits. The cap
   contour's feet come from `filletband.OffsetJoins`, which holds a G1 join's
   foot and a connector arc's two feet at the exact point the record denotes
   wherever a float64 holds that point: the recorded corner moved `r` along a
   rational unit normal, evaluated in exact rational arithmetic
   (`capcontour.ExactJoinFoot`), and taken at a G1 join only when both walks'
   feet are that one point; that agreement is a choice, not a soundness gate,
   since the displacement bound covers either foot. Elsewhere the foot is `offset2d.CornerJoin`'s
   float step along the held tangent, which for a circular walk is a
   `math.Sincos` at a computed angle; the contour's displacement bound covers
   it either way. The LF5 and LF6 meridians stand on these feet (step 4), so
   where the feet are exact a meridian's `Center` and both its vertices lie
   in its own plane exactly, as DF10's analytic writer requires: P8's
   tangent join at `(0, 3)` holds its foot at `(2, 3)`, which the float step
   puts at `(2, 2.9999999999999996)` on amd64. `offset2d.PartialFilletContour`
   applies the same rule (`offset2d.ExactCornerFeet`) to its selected pairs,
   so a partial fillet's contour and meridians stand on the same feet.
2. **Corner classes.** Each corner of `ℓ`'s coalesced walk is classified from
   the record's own floats as `capband.OccupiedVolumeAdmission` classifies
   them: a reflex corner (`joins[i].arc`) is LF6; a line–line miter is LF4;
   a join `capband.RestatedJoinIsG1` proves tangent is LF5; a join the
   offset reads as G1 that `capband.RestatedNearTangentCorner` admits, every
   wall beside it passing `capband.NearTangentWallRefusal`, is LF5′ (§5.5);
   a whole turn is LF7; any other corner is SF1. Every off-radius arc is read
   about its restated centre (§5.6). The classification is exact rational
   arithmetic on recorded coordinates and admits nothing on a residual.
3. **Band record.** `brepLoopBand` carries `kind` (`"chamfer"` | `"fillet"`);
   the record stores `(face, loop, orig, setback, sigma, kind)` as route L
   does. No face record holds a cylinder with an in-plane axis, a torus or an
   ellipse (modify-general §8).
4. **Build.** After `breprecord.Build` has built the record's faces and edges,
   `attachFilletBand` (`internal/breprecord/fillet.go`) runs in `F`'s frame over
   `brepLoopBand.view`, with the cap contour's coedges supplied as route L
   supplies them and the side contour's coedges the trimmed walls' rims at
   `sideZ`. Per walk `i` it mints the LF1–LF3 patch face whose loop is: the
   side contact edge (shared), the end curve at the corner after `i`, the cap
   contact edge reversed (shared), the end curve at the corner before `i`
   reversed. Per LF6 corner it mints the horn-torus face: the connector arc
   at `L_F` (shared with `F`), the `pB` meridian, the apex vertex at `sideZ`
   (the loop corner's own vertex, `sideVertexAt`), the `pA` meridian. Every
   end curve is minted once and shared by the two patches it bounds. Vertices
   are route L's: contour corners and connector ends at `L_F` with
   `F.delta`'s bound, loop corners at `sideZ` with `levelDelta`.
5. **Roles.** `filletLoop(f,l,p)` for face `f`, loop `l`, patch `p` in the
   band's own order: walk `i`'s patch, then the LF6 patch at the corner after
   walk `i`. A patch face's `Surface()` is its LF tag; `normalBound` is zero,
   because the tag's surface is the built surface and the quarter window is
   structural (§7 DF7); placement rounding of the tag's direction is
   `Face.NormalAt`'s own arithmetic term (`normal_bound.go`), as on a prism's
   cylinder wall.
6. **Closure.** `topobuild.AttachFaceLoops` proves every edge bounds exactly two
   faces, as for a chamfer band.

Where the walls rise off `F` (`σ = +1`: a boss root, a pocket floor's outer
loop) the band fills the concave corner: the same rewrite with `sideZ` on
`F`'s outer side, and the patches' outward normals pointing into the void
between `F` and the wall. The ball's centre is then `r` above `F` and `r`
from the wall, so at height `h'` above `F` the fill extends
`r − √(2rh' − h'²)` from the wall, which is `t(r − h')`: read from `sideZ`
toward `F`, the offset is `t(h)` in both senses, and every formula below
measures `h` that way.

### 5. Measurements

#### 5.1 Table CF — the strip coefficients

Let `S(t)` be the strip between `ℓ` and `ℓ` offset `t` into `F`'s material,
`0 ≤ t ≤ r`, in `F`'s plane-local `(u, v)`. With LF4–LF7 corners, its area
and first moment are polynomials in `t`:

```text
A(t) = a₁·t + a₂·t²
M(t) = ∫_{S(t)} (u, v) dA = m₁·t + m₂·t² + m₃·t³      (m₁, m₂, m₃ ∈ ℝ²)
```

because each of those corners' foot loci is affine in `t` (a line–line
miter moves along the bisector at rate `1/sin(θ/2)`, a G1 foot along the
shared normal, a reflex foot along each wall's own normal; reach §8.3). The
coefficients sum over the loop's pieces. A straight walk from `P` to `Q`
has length `ℓᵢ`, unit tangent `τ`, inward unit normal `ν` (into `F`'s
material), and end rates `κ₀` at `P`, `κ₁` at `Q`, where `κ = cot(θ/2)` at
an LF4 corner of interior angle `θ` and `κ = 0` at an LF5, LF5′ or LF6 corner; its
strip piece is the trapezoid `{P + s·τ + w·ν : 0 ≤ w ≤ t, κ₀·w ≤ s ≤ ℓᵢ −
κ₁·w}`. A circular walk has centre `C`, radius `R`, sweep `β > 0`, and
`E = (sin α₁ − sin α₀, cos α₀ − cos α₁)` over its angular interval
`[α₀, α₁]` taken increasing. A reflex corner at `V` with turn `ψ` spans the
angular interval `[γ₀, γ₁]` between the arriving walk's inward normal and the
leaving walk's, `E_V` likewise.

| CF | Piece | `a₁` | `a₂` | `m₁` | `m₂` | `m₃` |
|---|---|---|---|---|---|---|
| **CF1** | straight walk | `ℓᵢ` | `−(κ₀ + κ₁)/2` | `ℓᵢ·P + (ℓᵢ²/2)·τ` | `−(κ₀+κ₁)/2·P − (ℓᵢ·κ₁/2)·τ + (ℓᵢ/2)·ν` | `((κ₁² − κ₀²)/6)·τ − ((κ₀+κ₁)/3)·ν` |
| **CF2** | circular walk, material inside | `β·R` | `−β/2` | `β·R·C + R²·E` | `−(β/2)·C − R·E` | `E/3` |
| **CF3** | circular walk, material outside | `β·R` | `+β/2` | `β·R·C + R²·E` | `+(β/2)·C + R·E` | `E/3` |
| **CF4** | reflex corner (LF6) | 0 | `ψ/2` | 0 | `(ψ/2)·V` | `E_V/3` |
| **CF5** | LF4 corner, LF5 or LF5′ join | 0 | 0 (its share is in `κ`) | 0 | 0 | 0 |

At an LF8 corner, these coefficients describe the untrimmed walk pieces.
The true strip differs only near the shared corner. A proven disk contains
that difference at every offset: the miter foot's speed bounds its travel,
and a circular connector either stays on a certified short arc or uses its
entire offset circle as a fallback. `CurvedMiterMassAllowance` widens the
strip's volume and moments by that disk's integrals.

Derivations. CF1: `M = ∫₀^t [P·Λ(w) + τ·((ℓᵢ − κ₁w)² − κ₀²w²)/2 + ν·w·Λ(w)] dw`
with `Λ(w) = ℓᵢ − (κ₀+κ₁)w`; the kite two walls overlap in at an LF4 corner
has area `t²·cot(θ/2)`, split as one triangle `κ·t²/2` per wall. CF2:
`∫_{R−t}^{R} ρ² dρ = R²t − Rt² + t³/3`; CF3: `∫_R^{R+t} ρ² dρ = R²t + Rt² +
t³/3`. CF4: the sector of radius `t` fills the gap the two wall strips leave.
The check on route L's own fixtures: P2's top loop has `a₁ = 160, a₂ = −4`
and its mouth `a₁ = 60, a₂ = π`, so the chamfer's `∫₀^d A(s) ds = a₁d²/2 +
a₂d³/3` reproduces `80d² − 4d³/3` and `30d² + πd³/3` (modify-general §9).

Every coefficient is enclosed in rational interval arithmetic
(`internal/proof`) over the recorded coordinates with outward-rounded square
roots: `ℓᵢ`, `τ`, `ν` from `Q − P`; `κ = cot(θ/2) = (1 + cos θ)/sin θ` from
the exact dot and cross products of the two walls' directions divided by the
enclosed lengths; `R` and `E` from the recorded centre and ends (`sin α =
(P_v − C_v)/R`); `β` and `ψ` through `Atan2Interval`. An axis-aligned loop of
lines at recorded coordinates has every coefficient an exact rational.

#### 5.2 The height integrals

```text
J_k = ∫₀^r t(h)^k dh:   J₁ = r²·(1 − π/4)     J₂ = r³·(5/3 − π/2)     J₃ = r⁴·(3 − 15π/16)
H_k = ∫₀^r h·t(h)^k dh: H₁ = r³/6             H₂ = r⁴/12
```

By `h = r·sin a`, `t = r·(1 − cos a)`, `dh = r·cos a·da` over `[0, π/2]`:
`J₁` from `∫(cos a − cos²a) = 1 − π/4`, `J₂` from `∫(cos a − 2cos²a + cos³a)
= 1 − π/2 + 2/3`, `J₃` from `∫(cos a − 3cos²a + 3cos³a − cos⁴a) = 1 − 3π/4 +
2 − 3π/16`; `H₁ = r³/2 − r³/3`, `H₂ = 3r⁴/4 − 2r⁴/3`. `π` enters through
`proofbound.TwoPiInterval`'s rational bracket; `r` is the interval
`[r − rDelta, r + rDelta]`, so a radius stated in inches charges its
conversion here and nowhere else in the volume.

#### 5.3 Volume, first moment, area

```text
V_strip  = a₁·J₁ + a₂·J₂                                        (the wedge the band removes or fills)
M_strip  = (m₁·J₁ + m₂·J₂ + m₃·J₃,  sideZ·V_strip − m·(a₁·H₁ + a₂·H₂))   (in-plane, axial; m = matSign)
V_body   = V_receiver + σ·V_strip
M_body   = M_receiver + σ·M_strip
```

`V_receiver` and `M_receiver` are general-boolean §4.3's face terms over the
record RESTORED: `F` read over its region with `orig` in the contour's place,
each face beside the loop read over its untrimmed level `L_F` instead of
`sideZ` (`brepPayload.filletRestored`, a view), every other face as it is.
`breprecord`'s `measureBody` therefore sums the restored faces, which carry no
contour displacement (SB1 holds `delta = 0` on the receiver), then adds
`σ·V_strip` and `σ·M_strip` per fillet band. The patches' own flux is never
integrated: `Σ_patches flux = V_body − Σ_rewritten faces = (F's strip term)
+ Σ(wall strips over [sideZ, L_F]) + σ·V_strip`, and the two strip terms are
exactly the restored-minus-rewritten difference. The centroid divides
`M_body` by `V_body` and lifts through the reference frame and placement as a
chamfered brep's does, with `capband.CentroidGeometryBound`'s ceiling.

Patch areas, the perimeter of the level loop being `a₁ + 2a₂·t` (the
derivative of `A(F ⊖ t)`), integrate over the quarter arc:

```text
Σ_patches area = a₁·(π/2)·r + a₂·(π − 2)·r²
per LF1 patch:  (π/2)·r·ℓᵢ − (κ₀ + κ₁)·r²·(π/2 − 1)
per LF2/LF3:    β·r·(R·π/2 ∓ r·(π/2 − 1))
per LF6:        ψ·r²·(π/2 − 1)
```

`Body.Area()` sums the rewritten faces (`F` with `sectionDisplacementArea`
over `F.delta`, the walls trimmed) and the patches. Each patch publishes its
own area with the bound of its coefficients' enclosure.

Exactness: `π` appears in every `J_k` and every patch area, so a loop
fillet's volume, area and centroid are never `Exact`. LF8 corner allowances
can be much wider than rounding bounds and can exceed Verify's default
tolerance. `F.delta` does not enter the volume or the first moment: the strip
model reads the receiver's recorded loop, which the fillet denotes, and the
rewritten faces enter only through `Body.Area()` and the per-face readings.

#### 5.4 Extents and `Bounds`

A fillet patch bulges past the hull of its two directrices (a quarter
cylinder's farthest point along an oblique direction is interior), so
`brepPayload.ExtentAlong` adds each band's patches. Along unit direction `g`
in `F`'s frame, with `A` the component of `g` along the direction from
`sideZ` toward `F` (`−m·ẑ`) and `B` its component along the patch's in-plane
outward normal:

- LF1: `f(φ) = g_τ·s_end(φ) + r·(A·cos φ + B·sin φ)` over `φ ∈ [0, π/2]` with
  `s_end(φ) = ℓᵢ − κ·r·(1 − sin φ)` at the end the sign of `g_τ` selects,
  a sinusoid in `φ` whose extreme is an endpoint or the interior stationary
  point; the result is `Exact` only at an endpoint that is a recorded
  vertex.
- LF2/LF3/LF6: `f(φ, a) = (R ∓ r·(1 − sin φ))·ĝ_⊥·ρ̂(a) + r·A·cos φ` with the
  azimuth extreme `|g_⊥|` where `g_⊥`'s direction lies in the patch's
  azimuth window and the window's end otherwise, then the same sinusoid in
  `φ`.

Every stationary point is isolated (a sinusoid has one per quarter), so no
`ErrUnsupported` arises for the baseline patches here. LF8 widens the
maximum by the changed corner disk and uses existing boundary vertices as
lower witnesses. `ThroughAll`/`ToFace` read the same extents.

#### 5.5 LF5′ — near-tangent joins

A sketch solve places tangent points by rounding, so an outline drawn with
tangent constraints or `sketch.CreateFillet` records line–arc and arc–arc
joins whose exact tangents turn by a sliver θ. The offset's held-tangent rule
(`offset2d.CornerJoin` within `offset2d.ShellTol`) reads such a join as G1,
and the contour holds one foot there; `capband.JoinIsG1` refuses it.

**The class.** Corner `k` is LF5′ when the offset reads it as G1,
`capband.RestatedJoinIsG1` does not prove it exactly tangent, and
`capband.RestatedNearTangentCorner` admits it with both walks moving by `r`
over `[0, T]`, `T = r + rDelta`: the six conditions of
Part D §9.1.1 (a line and an arc or two arcs, one
recorded junction point, exact tangents running the same way,
`sin²θ ≤ 2⁻⁶⁴`, one amount, each moved circle clear of the corner), with
every off-radius arc read about its restated centre (§5.6). The same corner
is also read about the recorded centres through the recorded junction, as
the LF5 test reads it; that reading's class must admit it too, and its
reach is `ρ_rec`, zero at a join exactly tangent there. Every wall with an
end whose `ρ′` or `ρ_rec` is positive must pass
`capband.NearTangentWallRefusal` with every walk at `r`, read with restated
centres for `ρ′` and with the recorded ones for `ρ_rec`. Its other end must
not be LF8 or LF6, whose true end that gate does not enclose. Every test is exact rational arithmetic over
the record's floats. `filletband.CornerClass` returns the corner's
`ρ′ = T·sinUp·(1 + S)` and `ρ_rec`; `filletband.Loop.Rho` and `RhoRec` hold
them.

**Geometry.** On a section with no radial step (§5.7), at offset
`t ∈ [0, T]` write `v` for the corner and `f_p`, `f_c` for the arriving and
leaving walks' radial feet `v + t·n`. Lemmas 1 and 1′ give
`|f_p − f_c| ≤ ρ` (`2t·sin(θ/2) ≤ ρ` for a line and an arc). At a convex
sliver the true corner `m(t)` lies within `ρ` of the arc's foot (a line and an
arc) or of both feet (two arcs), so within `2ρ` of each foot. At a reflex
sliver the offset closes the gap with the connector arc of radius `t` about
`v` from `f_p` to `f_c`, whose every point lies within `ρ` of both feet.

**Build.** LF5′ is built as LF5: one `Arc3` meridian on the contour's G1
foot, the two patches meeting there, `κ = 0` in Table CF, and no horn patch,
so the record's faces, edges and roles are LF5's. Every reading below states
that structure's value and widens it by a proven upper bound rounded up from
a rational.

**Strip.** On the restated section X′ (§5.6), which has no step, the strip
pieces of Table CF and the true strip at offset `t` differ by one closed chain
at the corner: `v → f_p`, along the arriving carrier to `m`, along the
leaving carrier to `f_c`, `→ v` at a convex sliver, and `v → f_p`, along the
connector arc to `f_c`, `→ v` at a reflex one. The rest of both boundaries
coincide, since every wall survives. The first chain encloses at most the
triangle `v f_p f_c` (`½·t·ρ`), the triangle `f_p m f_c` (`ρ²`), and each
carrier arc's segment against its chord: the chord is at most `ρ`, below the
moved radius, so the arc stays under a sixth of a turn and its segment under
`πρ²/4` (Thales). The second encloses the sector `t²θ/2 ≤ (π/4)·t·ρ`, since
`θ ≤ π·sin(θ/2)`. So `|A′_CF(t) − A′(t)| ≤ t·ρ′ + 3ρ′²`, and the difference
lies within `t + 2ρ′` of `v`. Over the quarter (`t ≤ r_hi`,
`filletband.NearTangentMassAllowance`):

```text
ΔV        ≤ r_hi·ρ′·(r_hi + 3ρ′)
ΔM_u      ≤ ΔV·(|v_u| + e_v + r_hi + 2ρ′)        (ΔM_v likewise)
ΔM_axial  ≤ ΔV·(|L_F| + L_F.delta + r_hi)
```

with `e_v` the corner's recorded end bound. `filletband.MassOf` widens both
ends of the volume and every moment by them, as it does at LF8. §5.6 charges
the difference between X′ and the recorded section.

**Every other reading** is §5.7's with `ρ̃ = max(ρ′, ρ_rec)`: on a section
with no step it reaches `2ρ̃` (extents, cap contour, mesh), changes each
level curve's length by at most `2ρ̃` (patch areas), turns the normals by
`filletband.NearTangentNormalTurn` (undercut survey, `Face.NormalBound`), and
lengthens the seam by at most `(π/2)·r_hi·γ²/4`.

**Unchanged readings.** Clearance (DF6) stays `Suspect`. STEP (DF10) writes
a patch whose `Face.NormalBound` §5.7 sets as faceted, since
`facedeparture` reads that bound as a tag departure. A partial fillet charges
none of §5.5–§5.7's terms, so a selection that meets a corner with a
near-tangent reach in either reading, a radial step, or a restated arc on
either side refuses (`filletband.Loop.Charged`, SL2).

The rounded hexagon of §8 (corner circle 2.51, corner rounds 0.8, fillet 0.6)
has LF5′ joins of both signs, each with a `ρ′` at rounding level, far below
any tessellation tolerance.

#### 5.6 Off-radius arcs — restatement

An `ArcSeg` denotes Start's radius up to End's angle and a radial step from
that point to End (`agent-docs/evaluator-design.md` §4), and a sketch solve
leaves its End a few ulps off the circle its Start states (most of the rounded
hexagon's corner arcs). Table CF reads such an arc as the record denotes it,
the annular sector about the recorded centre `C` at Start's radius up to
End's angle, so the strip proof restates the loop as
`agent-docs/tessellation-reach-design.md` §7.1 does: each off-radius arc is
read about the centre `C′` of its ends' perpendicular bisector nearest `C`, on
which both recorded ends lie. Write X for the recorded section and X′ for the
restated one, which has no step.

**Reading.** Every corner is read about the restated centres: a join the
offset reads as G1 through `capband.RestatedJoinIsG1` and
`capband.RestatedNearTangentCorner` (§5.5), and every wall's survival gate
with `restate` set. At a reflex corner beside an off-radius arc the exact turn
is read about both centres, and two signs that disagree refuse. A sharp convex
corner beside an off-radius arc (LF8 there) refuses: the curved-miter
allowance reads the recorded centre. Each off-radius arc must be its walk's
only recorded segment, and `capband.OffRadiusShift` must bound it at reach
`T`, giving its shift `Δ_a`; `filletband.Loop.Restate` keeps
`capband.OffRadiusLoop`'s largest shift `Δ`, level-set perimeter `P` and
largest radius `R_hi`, and how many LF6 corners sit beside a restated arc.
This charges an exactly tangent (LF5) corner beside an off-radius arc too.

**The strip.** Write `A`, `A′` for the strip areas of X and X′ at offset `t`
and `A_CF`, `A′_CF` for Table CF's pieces of each. Then
`|A_CF − A| ≤ |A_CF − A′_CF| + |A′_CF − A′| + |A′ − A|`:

- `|A′ − A|`: the strip is the loop's region less its offset. §7.1's coarea
  bound states that each of the two regions and the two offsets differ by at
  most `2Δ·P` at every level up to `T`, so `|A′ − A| ≤ 4Δ·P`.
- `|A′_CF − A′|`: X′ has no step, so its corners are the classes above; only
  an LF5′ corner adds §5.5's term.
- `|A_CF − A′_CF|`: Table CF's pieces differ only where an arc is read. A
  restated arc's piece `S` is the image of `(φ, τ) ∈ [0, 1] × [0, t]`, and its
  restated piece `S′` is the image of the same rectangle with every point within
  `Δ_a` (`capband.OffRadiusShift`). A point of one image and not the other is
  swept by the straight homotopy between the two images of the rectangle's
  boundary, so `S △ S′` lies in the `Δ_a`-tube of `∂S`, of area at most
  `2Δ_a·|∂S| + πΔ_a²`, with `|∂S| ≤ 4π(R_hi + T) + 2T`. An LF6 sector beside a
  restated arc reads that arc's normal at the corner, which turns by at most
  `(π/2)·c/r′ ≤ Δ_a/T` (§7.1's angle bound), so the sector moves by at most
  `t²·Δ_a/(2T) ≤ T·Δ_a`. Lines are not restated, so a straight piece and every
  `κ` stay as they are.

Over the quarter (`filletband.RestateMassAllowance`), with `n` restated arcs
and `m` LF6 corners beside one:

```text
ΔV_R    ≤ r_hi·[4Δ·P + n·(2Δ·(4π(R_hi + T) + 2T) + πΔ²) + m·T·Δ]
ΔM_R,u  ≤ ΔV_R·(c_L + T + Δ)        (ΔM_R,v likewise)
ΔM_R,ax ≤ ΔV_R·(|L_F| + L_F.delta + r_hi)
```

with `c_L` bounding the loop's plane-local coordinates
(`filletband.WalkCoordUpper`). `filletband.MassOf` widens the volume and
every moment by them. §7.1's Lemma 3 assumes X′ stays a simple loop at the
same clearance from the other loops, and so does this bound; the window gate
of `OffRadiusShift` bounds `Δ` only below the chord. That premise is inherited
from §7.1 and is not proven here.

The restated section serves the strip alone. The patches, the contour and
the mesh are the record's, built about `C`, and §5.7 charges each of them
against the true band of X directly.

#### 5.7 Local bounds at a corner

At corner `k` write `v` for the recorded junction, and `J` for the record's
radial step there: the segment from `v` to each neighbouring arc's own end
point on the circle its Start states. `δ_k`, the sum of the two arcs'
`||v − C| − |Start − C||` (`filletband.cornerGap`, each read as
`|r_v² − r_S²|/(r_v + r_S)` with both roots rounded down), bounds how far `J`
reaches from `v`. `δ_k` is zero at a corner whose arcs end on their circles,
and a corner whose `δ_k` reaches `T/4` refuses (SF1). The cone term below
needs that gate: with `δ < T/4` and `θ ≤ π·ρ̃/(2T)`,
`(t + δ)·θ ≤ (5π/8)·ρ̃ < 2ρ̃`.
Write `ρ̃ = max(ρ′, ρ_rec)` (zero away from LF5′ and recorded near-tangent
joins). The recorded reading sees each arc's tangent at `v` about `C`, which
is the arc's own tangent at its end on the circle, since that end lies on the
ray from `C` through `v`. So the turn θ between the two walks' end tangents
satisfies `θ ≤ π·ρ̃/(2T)`. The band's surface at offset `t` is the level
curve `{d_X = t}` lifted to height `h(t)`.

**Where the true level curve leaves the stated one.** Take a true level
point `x` and its nearest point `y` on X.

- If `y` lies inside a walk, then `x` lies on that walk's moved carrier in the
  walk's window, which is on the stated patch.
- If `y` lies on `J` or at a walk's end, then `t − δ ≤ |x − v| ≤ t + δ`. In a
  walk's window the depth below that walk lies in `[t, t + 2δ]`: depth is
  1-Lipschitz, and `J` lies within `2δ` of the walk's own end point. So `x`
  is within `2δ` of the moved carrier.
- In the corner's normal cone (the wedge of angle θ at a G1-read join, the
  reflex cone at LF6), `x` lies within `δ + (t + δ)·θ` of the stated corner
  point at that level. That point is the G1 foot `v + t·n`, or the LF6
  connector of radius `t` about `v`.

A stated point that differs from the true curve lies on its moved carrier,
within `t + δ + ρ̃` of `v` (§5.5 bounds the convex sliver's overlap), while the
carrier passes at least `t − δ` from `v`. On a line, or on a circle that holds
its material inside (moved radius `R − t`, curving away from `v`), the piece's
chord is at most `2·√((2t + ρ̃)(2δ + ρ̃)) ≤ 4·√((T + δ + ρ̃)(δ + ρ̃))`. On a
circle that holds its material outside (a hole rim's arc, moved radius
`R + t`, curving round `v`), the power of `v` about the moved circle scales
the squared chord by at most `(R + t)/R_v`. So the chord carries the factor
`√k`, `k = (R_hi + T)/R_lo`, read from the recorded centre to both recorded
ends (`stepCurvature`), and `k = 1` otherwise. The piece's arc, under a sixth
of a turn, is at most `stepRoot = 2π·√(k·(T + δ + ρ̃)(δ + ρ̃))`, and it starts
at a point the two curves share. With `δ = 0`, §5.5's `2ρ` bounds it instead.
So the two curves are within

```text
σ_k = 2ρ̃ + 3δ + stepRoot·[δ > 0]                (filletband.Loop.LocalReach)
```

of each other, in both directions, at every level.

- **Extents.** `filletband.WidenNearTangentExtent` widens both ends of §5.4's
  enclosure by the largest `σ_k` times the direction's planar length.
- **Cap contour.** `filletband.LoopContourDelta` adds the largest `σ_k` to
  `F.delta`. The cap vertex, `F`'s area and mesh, and every meridian on the
  contour read it there.
- **Mesh.** `brepfilletmesh` adds `σ_k` to corner `k`'s seam gap. Each patch
  beside the corner adds it to its stated distance (§7.1), and so does an LF6
  horn at the corner.

**Level length and patch area.** A cylinder's or torus's area element, and
that of the true band, which is a distance level set with unit gradient, is
its level length times `r·dφ`. Outside the disk of radius `t + δ + ρ̃` about
`v` the two level curves coincide. Inside it, the stated and true pieces that
differ consist of the following:

- the stated carrier pieces above, each at most `stepRoot`, two in all;
- the true pieces in the walks' windows: the moved carrier's part that stays
  true lies inside the stated piece, and the arcs of radius `t` about points
  of `J` that replace the rest span no more than the stated piece's chord, so
  each window adds at most another `stepRoot`;
- in the cone, a curve star-shaped about `v`, with radius between `t` and
  `t + δ`, against the stated connector of radius `t`;
- the straight sides of `J`'s offset, `2δ` in all.

In the cone the two curves differ in length by at most `πδ`, plus the
radius's variation, at most `4δ`. With §5.5's `2ρ̃` for the sliver, each level
changes by at most

```text
L_k = 2ρ̃ + (4·stepRoot + 10δ)·[δ > 0]          (filletband.Loop.LevelLength)
```

Each patch beside the corner widens its area by `(π/2)·r_hi·L_k`
(`filletband.LevelAreaAllowance`). At `δ = 0` this is §5.5's convex sliver
term `π·r·ρ`, which also covers a reflex sliver's absent horn
`θ·r²·(π/2 − 1) ≤ (π/2)(π/2 − 1)·r·ρ`.

**Normals.** At a corner with no step whose `ρ̃` is positive, the true band's
normals stray from the normal a patch beside the corner states at the same
point by at most `filletband.NearTangentNormalTurn` read at `ρ̃`. A stated face
that runs past the true corner carries the neighbour's surface there, so the
tilt is the turn between the two walls, `2·sin(θ/2) ≤ ρ/T ≤ ρ/r_lo`, plus
the rotation of each circular carrier's normal over the stretch between its
radial foot and the true corner or the other foot. That stretch is a chord
of at most `ρ` on a moved circle of radius at least `R_lo`, and a chord bounds
the unit-normal distance it turns through, so each adds `ρ/R_lo`. The charge
is `ρ/r_lo + Σ ρ/R_lo` over the circular walks. At a reflex sliver the
horn's normals lie between the two walls', inside the same sum. Beside a step,
the true band wraps round the step's own corner at levels near `δ`. There
its normals run from the wall's to the step's, a quarter turn plus θ, so the
turn charged is `3/2 > √2` (`filletband.Loop.NormalTurn`).

- **Undercut survey (DF7).** `survey2d.WidenPullBracket` widens each adjacent
  patch's bracket, an LF6 horn's included, by the turn times
  `|Du·pull| + |Dv·pull|`. A pull along `F`'s normal moves nothing.
- **Patch normal bound.** Each patch beside such a corner sets
  `Face.NormalBound` to the turn after its orientation is read. The
  concave-radius survey (DF8) therefore reads such a body undecided
  (`brepMinRadius`'s guard).

**Seam length.** At an LF5 or LF5′ corner the true seam lifts the in-plane
path of the corner's true point. That point lies on both moved carriers, so
`u₁·m′ = u₂·m′ = 1` for their inward normals. `m′` then runs along their
bisector with `|m′| = 1/cos(γ/2)`, where γ is the angle between the normals:
`γ ≤ θ + Σ (π/2)·ρ̃/R_lo` over the circular carriers, and
`θ ≤ (π/2)·ρ̃/r_lo`. The seam's speed `r·√(|m′|²·sin²φ + cos²φ)` is at most
`r·|m′|` and at least `r`, so its length lies in
`[(π/2)·r, (π/2)·r·(1 + γ²/4)]`. At a step, the boundaries of `J`'s region
advance monotonically in the lateral direction. The circle of radius `t`
about a point of `J` at depth `d` meets the moved carrier at angle α with
`1 − cos α = d(2t − d)/(2(R − t)(R − d))`, which increases with `t`. So each
boundary adds at most twice `σ_k`. `filletband.Loop.MeridianExcess` sums
these, and every meridian at the corner widens its `LengthBound` by them.

#### 5.8 Reference numbers

The rounded hexagon's allowances are rounding-sized. Its near-tangent strip
allowance (§5.5) and its restatement allowance (§5.6, dominated by `4Δ·P`)
are tiny volumes. Its largest radial step is a few ulps, and `stepRoot` is a
square root of that step, so it is far larger than the step but still small.
Every local reach and each patch's area widening stays far below a
tessellation tolerance.
`TestFilletLoopNearTangentJoins` asserts the volume's published bound is
under 0.01 mm³.

### 6. Table SF — refusals and gate order

Modify §1's test picks every sentinel.

| SF | Call | Exists? | Sentinel |
|---|---|---|---|
| **SF1** | a corner whose exact turn disagrees with the offset contour's join; an LF8 corner whose offset locus folds or cannot be bounded; a G1-read join outside LF5′ in either reading (a turn past `sin²θ ≤ 2⁻⁶⁴`, a moved circle within its reach, a wall not proven to survive, a wall whose other end is LF8 or LF6); a sharp convex corner beside an off-radius arc; a reflex corner whose turn changes sign on restatement; a radial step of a quarter of `r + rDelta` or more; an off-radius arc `capband.OffRadiusShift` does not bound or that shares its walk (§5.5–§5.6) | no regular miter can be proved | `ErrUnsupported`, naming the corner and the rule's reason |
| **SF2** | `Face.NormalAt` at an LF6 apex vertex, which lies on the horn torus's own axis | the point exists; the surface normal there does not | `ErrDegenerate` (the survey samples interior points only) |

Everything else is an existing row: SL1 (a partial loop outside the
selected-chain route in Part V),
SL2 (LB3/LB4/LB6), SB1 (`delta ≠ 0`), SX6
(`R − r < 0` on an LF2 wall, a dropped carrier), SX7 (band reach, two bands
on one wall), SX12/SX14 (the contour at `r`), SX13 (`R ∓ r == R` or
`L_F ∓ r == L_F`), SX5 (revolve), SX9/SX10, S3. SL3 is retired: a fillet of
complete loops of planar faces builds here or refuses with a row above.

Gate order for a `Fillet` on a brep, stacked, or (RF3) prism receiver, after
modify §4's stage 1 and reach SX10:

| Stage | Gates |
|---|---|
| 2a. record | RB dispatch; SB2; SB1; RF3's `breprecord.OfPrism` |
| 2b. route P | brep-modify §6 unchanged |
| 2c. entry | single straight edges → route E; complete loops with independent straight edges → route V; otherwise LB1, LB2 (SL1) |
| 3. topology | LB3, LB4, LB6 (SL2); LB5 (SX7) |
| 4. existence | SX6, SX13 per band as the contour at `r` is built; SF1 per corner |
| 5. audit | per band SX14, SX7, SX12 on the contour; per (pl) face S8, S6, S7, S9; per `F` S8, S7, S9 |
| 6. build | the record's closure; `attachFilletBand` per band |

### 7. Tables BF and DF — results and downstream

| BF | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BF1** | route L or V fillet, any RF1–RF3 receiver | `brepPayload` with `loopBands` of kind `"fillet"`, `stack` nil | the rewritten record's faces plus one LF1–LF3 or LF9 patch per walk and one LF6 patch per reflex corner; every edge on exactly two faces | `face(k)` / `wall(k)`; each patch `filletLoop(f,l,p)` |

| DF | Consumer | BF1 |
|---|---|---|
| **DF1** | mass properties, area | §5.3 |
| **DF2** | `Bounds`, `ThroughAll`/`ToFace` | §5.4 |
| **DF3** | `Verify` validity, tolerance gate | by construction; the bands' vertices are body vertices; `brepGateDiameter` reads every vertex |
| **DF4** | `Tessellate`, STL/OBJ/3MF | §7.1 |
| **DF5** | mesh boolean operand | admitted for every band: every ring of §7.1 lies on the denoted surface up to `F.delta` and interpolation rounding, so the slice-wise argument of tessellation-reach §7 holds with the per-vertex store as the vertex-motion allowance and §7.1's sagitta terms as the chord term; an LF5′ corner or a radial step adds its local reach to the distance of the patches beside it (§5.7). `capband.OccupiedVolumeAdmission`'s reflex refusal does not apply: the LF6 fan's stations are the connector arc's azimuths at every ring |
| **DF6** | clearance | no model: the pair reads `Suspect` unless its boxes decide it (DG6); no model of the LF1 cylinder carrier, whose axis is in-plane, exists |
| **DF7** | undercut survey | per patch through `Face.NormalAt`, a patch beside an LF5′ corner or a radial step widened by §5.7's normal turn: the normal of an LF1 patch sweeps the quarter from `n̂_F` to the wall's outward normal, an LF2/LF3 patch the quarter at every azimuth of its window, an LF6 patch the quarter at every azimuth of `ψ` short of the apex; the component range along the pull is the closed-form range of `A·cos φ + B·sin φ` over the window, read with reach DX7's three-valued rule and its arithmetic allowance alone, since the stamped departure is zero |
| **DF8** | concave-radius survey | undecided for a band with a patch beside an LF5′ corner or a radial step, whose `NormalBound` §5.7 sets; otherwise a band with `σ = +1` (a fill) contributes `r` exactly, the tag's `Minor`/`Radius`, which a placement leaves unchanged; a band with `σ = −1` is convex in its tube direction and contributes nothing; an LF6 patch's other principal radius is `ρ/sin φ`, which vanishes at the apex where the receiver's own sharp reflex edge already reads as `brepMinRadius` reads it today (a sharp corner enters no aggregate), so it adds nothing. `brepMinRadius` drops its `capblend.PatchWindowSkew`/`normalBound` refusal for fillet bands, keeps it for chamfer bands |
| **DF9** | wall survey | staged `Suspect`, as for every brep |
| **DF10** | STEP | faceted for a band with a patch whose `NormalBound` §5.7 sets; otherwise analytic: `supportsAnalyticSTEP` takes a whole-turn torus (LF7), a four-arc torus patch (LF2/LF3) whose arcs each hold both their vertices in their own plane, a horn torus patch (LF6, a three-edge loop closing on the axis at its apex vertex) whose vertices are `Exact`, and a straight-walk cylinder patch closed by `Ellipse3` edges (`agent-docs/step-export-design.md`). `Body.Tessellate` admits a banded body, so `export.STEP` reaches the writer |
| **DF11** | a later `Fillet`/`Chamfer`/`Shell` | DG11: Tables RB/EB/SB/TC/LB over the rewritten record; a patch's edge is in no record and matches nothing (SL1); a trimmed wall's rim at `sideZ` ends on a patch, so route E's SB7 refuses an edge there; a record carrying bands reads as no prism (route P and Table TC refuse); a second loop fillet or chamfer on another loop appends to `loopBands`, provided the first left `delta = 0` |
| **DF12** | `Transform`, `Mirrored`, `PatternCopies` | re-lifts every face frame; `loopBands` re-attach on re-evaluation; a reflection flips `outward` and the ellipse's `Axis` sense with every winding (general-boolean A4) |

#### 7.1 Tessellation

A fillet band is meshed as `n_φ` strips between `n_φ + 1` rings. Ring `0` is
the side contour (the trimmed wall's rim at `sideZ`, shared), ring `n_φ` the
cap contour (`F`'s new loop, shared with `F`'s triangulation). Ring `k` is
the loop offset by `t_k = r·(1 − cos φ_k)` at height `h_k = r·sin φ_k` from
`sideZ` toward `F`, `φ_k = k·(π/2)/n_φ`. Ordinary ring vertices interpolate
affinely between the side and cap rings at matched azimuths:

```text
ring_k[j] = side[j] + (t_k / r)·(cap[j] − side[j])   in (u, v), at z = sideZ − m·h_k
```

This is exact outside LF8 corners: a line's offset is affine in `t`, a
circle's foot moves along its radius, a line–line miter corner moves along
the bisector, a G1 foot along the shared normal, and a reflex connector's
ring is the arc of radius `t_k` about the apex (`side[j]` is the apex for
every `j` of the connector). So every ring vertex lies on the denoted
surface within `F.delta·t_k/r ≤ F.delta` plus the interpolation's rounding,
which the vertex store carries. At LF8, each seam station uses the proven
offset-foot intersection at `t_k`; its chord gap is bounded by the seam's
global speed and the meridian step. At LF5′, and at a corner beside an
off-radius arc, the column stays on the G1 foot's or recorded join's affine
ring, and its seam gap is the corner's local reach (§5.7).

Counts: one count per walk shared by the trimmed wall, every ring and `F`'s
contour (`capblend.ChordLoop`, with `n(w) = max(chordCount(w),
chordCount(capArc(w)))` so the larger of the side and cap radii sets the
in-plane sagitta); `n_φ = ⌈(π/2) / (2·acos(1 − tol_φ/r))⌉` with `tol_φ` half
the chord budget, at least 2 and at most 4096
(`tessellation.MaxFilletRings`). The boss shell's root band (modify-general
§3.1a) raises `n_φ` to `⌈s·π/(4·c)⌉`, with `c` the whole chord budget
(`2·tol_φ`), within the same cap, where `s` is the
largest plan distance between the two corners' cap-minus-side offsets over
the band's cells (`brepfilletmesh.TwistRingCount`). Ring `k` sits at
`side + (cap − side)·g(φ_k)` with one `g` for every sample, so a cell's twist
vector is that offset difference times `Δg ≤ Δφ`, and the raised count keeps
`|T|/4` alone within the other half of the chord budget. Every other fillet
band takes the sagitta count alone, and every band's cells are charged as
the cell charge below states. An LF4 corner's ring vertices
are the ellipse's chord chain, shared by both cylinders; an LF5 join's are the
quarter circle's. Each strip between rings `k` and `k+1` is a quad per
azimuth interval with a fixed diagonal; along a straight walk the quad is a
planar trapezoid (two parallel chords).

Occupied-volume proof (general-boolean §4.4's structure, per band): the
true surface and the strip mesh correspond by `(azimuth, φ)`, and every
mesh point lies within `s_φ + s_ring + store` of its surface point, where
`s_φ = r·(1 − cos(Δφ/2))` is the quarter-arc chord sagitta, `s_ring` the
in-plane chord sagitta of the ring's arc at the larger radius (zero on a
straight walk), and `store` the ring vertex's bound. The band's symmetric
difference is at most `Σ_strips area(strip)·(s_φ + s_ring + store)`, summed
into `volSymDiff` beside the record faces' own terms. The strip's quads
are triangulated, so each cell's charge (below) joins `s_φ + s_ring`;
`store` enters through the per-vertex motion that
`SweptVolumeAllow` reads, where an interior ring vertex carries its two ends'
motion plus its own interpolation error (the interval error of `1 − cos φ_k`
times the end-to-end span, the exact rounding of the interpolation, and the
error of its level). A reflex connector's cap sample carries its station
bound, the radius's conversion bound and `F.delta`. The area slack a patch
adds is the measured gap between its held facets' area, enclosed in exact
rational arithmetic, and the patch's published area with its bound. The `Bound` a patch
face publishes on the mesh is the same sum per patch. Both terms are zero
at `n_φ → ∞` only, so a banded mesh is never `Exact`.

##### Cell charge

The charge is a Hausdorff bound between a cell's two held triangles `L` and
the bilinear patch `Q(s, u)` through the cell's four held corners: `vLo`,
`vHi` on ring `k` at the cell's two columns, `wLo`, `wHi` on ring `k+1`. The
surface-to-`Q` leg is `s_φ + s_ring`, and the vertex motion from the ideal
corners to the held ones is the `store` leg, so the charge answers only for
`Q` against `L` over the coordinates the mesh stores. `Rings.Emit` reads them
from the mesh's vertex array after every ring is placed, and the brep
tessellator moves no vertex afterwards (a reflection reverses windings
only). A patch takes the largest charge over its cells; a cell whose two
ring-`k` corners are one vertex (the LF6 apex) is a single triangle, which
is its own bilinear patch, and charges nothing. The charge is the
smaller of two upper bounds on the same distance, which is again an upper
bound:

1. **Twist bound** `|T|/4` with `T = vLo − vHi − wLo + wHi`
   (`proofbound.CellTwistOffsetUpper`). At equal parameters `L` and `Q`
   differ by a multiple of `T` of size at most `1/4`. It holds for every
   cell. It also counts movement within the cell's own plane: the box's
   straight-walk cell from one miter to the next has `|T| = 2·r·Δg`, whose
   `|T|/4` alone exceeds `tol` at the sagitta count (`2·3·0.309/4 ≈ 0.46` mm
   for `r = 3` at `tol = 0.1`, where 5 rings make the last strip's
   `Δg = cos(0.4π) = 0.309`).
2. **Height bound** (`proofbound.CellHeightOffsetUpper`). Let `Π` be the
   plane through `vLo`, `vHi`, `wLo`, `n = (vHi − vLo) × (wLo − vLo)`, and
   `h(p) = n̂·(p − vLo)`. The precondition is that the cell projected onto
   `Π` is a strictly convex quadrilateral: `n·(in × out) > 0` at each corner
   of the boundary `vLo → vHi → wHi → wLo`, with `in` and `out` the edges
   arriving at and leaving the corner. Projection along `n̂` does not change
   `n·(x × y)`, so these signs are the projected turns. Each is decided in
   exact integer arithmetic over the held floats, and a zero `n` or any
   sign `≤ 0` keeps the twist bound. Under the precondition:
   - `∂Q/∂s × ∂Q/∂u` is affine in `(s, u)` (its `s·u` term is `T' × T' =
     0`), and its `n` component at the unit square's four corners is the
     four corner turns, so it is positive on the whole square. The
     projected patch is a local homeomorphism whose boundary runs once
     around the convex quadrilateral `K`, hence a homeomorphism onto `K`.
   - Either diagonal splits a strictly convex `K` into two triangles that
     tile it, so the projected triangle pair is `K` too.
   - `Q` and `L` are therefore graphs over `K` with the same boundary. `h`
     is 0 at three corners and `H = h(wHi)` at the fourth, `h∘Q(s, u) =
     s·u·H`, and a triangle interpolates its corners' heights, so both
     graphs lie between heights 0 and `H`.
   - The two points of `Q` and `L` above one point of `K` are at most `|H|`
     apart. Pairing them is a homeomorphism that is the identity on the
     cell's boundary, so it glues with the neighbouring cells' pairings
     into one map of the band, as the parameter correspondence does.

   The bound is `|H| = |n·(wHi − vLo)| / |n|`, the distance of `wHi` from
   `Π`: `H²` as an exact rational over the held floats, rounded up through
   `RatSqrtUp`. No tolerance enters. A cell that fails the precondition is
   charged the twist bound; a near-planar residual never admits anything.

Every ordinary cell of a fillet band is a planar trapezoid at the ideal
corners: on a straight walk ring `k`'s chord runs along the walk's offset
line, on a circular walk between two azimuths at radius `ρ_k`, and on a
reflex connector between two azimuths of the arc of radius `t_k` about the
apex, so rings `k` and `k+1` give parallel chords run the same way. The held
corners carry rounding, so `H` is not zero but a rounding-level distance, and
the charge drops from `|T|/4` to that. An LF8 seam's ring positions follow
offset-carrier intersections and need not give parallel chords; there the
smaller bound is whichever the cell's corners give. `Emit` skips the height
reading for a cell whose `|T|/4` is at most the patch's running maximum,
since the smaller bound could not raise it.

##### Patch Bound against tol

A patch's `Bound` is `s_φ + s_ring + charge + seam gaps`, composed with
`F.delta`, the level displacement, the axial term and the vertex store.
`n_φ` holds `s_φ` within `tol/2`, and the charge is at rounding level, so a
band whose patches are all straight walks between line–line miters, on a
record with no displacement, publishes `Bound ≤ tol`
(`TestBrepLoopFilletTessellateWithinTolerance` asserts it with 5 and 14 rings
at `tol` 0.1 and 0.01 on the box's top loop at `r = 3`). There
`s_φ = r·(1 − cos(Δφ/2))` is 0.037 mm at 5 rings and 0.0047 mm at 14. `capblend.ChordLoop`
spends the whole chord budget on the in-plane walk count `n(w)`, so `s_ring`
on a circular walk, and the connector arc's sagitta at a reflex corner,
reach up to `tol`, and such a patch can read up to about `1.5·tol`. By the same
formulas, at `tol
= 0.01` the round boss's root loop filleted at `r = 1` has `s_φ` 0.0048 +
`s_ring` 0.0098 = 0.0146 mm, the pocket mouth and the L prism's top loop at
`r = 1.5` have `s_φ` 0.0046 + connector sagitta 0.0094 = 0.0141 mm, and the
rounded plate's top loop at `r = 2` has `s_φ` 0.0043 + `s_ring` 0.0093 =
0.0135 mm. At `tol = 0.1` only the round boss root exceeds `tol`, at 0.034 +
0.091 = 0.126 mm.

### 8. Reference fixtures

Every closed form below is `V_receiver + σ·(a₁·J₁ + a₂·J₂)` with Table CF's
coefficients, `J₁ = r²(1 − π/4)`, `J₂ = r³(5/3 − π/2)`. Each test asserts the
published volume's interval encloses it (`big.Rat` with a `π` bracket), the
patch count and kinds, the edge kinds, and the roles, through the public
API; a bound is asserted as a relation, never a literal.

| Part | Call | `a₁` | `a₂` | `σ` | Volume |
|---|---|---|---|---|---|
| **P1** 40×20×20, Ø6 along y | top loop, `r = 2` | 120 | −4 | −1 | `16000 − 180π − (1280/3 − 104π) = 46720/3 − 76π`; four `Cylinder` patches, four `Ellipse3` edges with `SemiMajor 2√2`, `SemiMinor 2`, centres `(2, 2, 18)` and its images; both x walls (sw) and both y walls (pl) trimmed to `z = 18` |
| | top and bottom loops, one call | 2 × above | | | `45440/3 + 28π` |
| | hole rim, `r = 1` (one y wall's hole loop, R = 3) | `6π` | `+π` | −1 | `16000 − 180π − ((23/3)π − 2π²) = 16000 − (563/3)π + 2π²`; one whole-turn `Torus{Axis: ŷ, Major: 4, Minor: 1}`, two `Circle3` edges: radius 4 on the wall face, radius 3 inside the hole at `y = 1` |
| **P2** 40×40×10, blind 20×10 pocket 5 deep (stacked) | pocket mouth, `r = 1.5` | 60 | `+π` (four LF6) | −1 | `15000 − (135 − 225π/8 − 27π²/16) = 14865 + 225π/8 + 27π²/16`; four `Cylinder` and four horn `Torus{Major: 1.5, Minor: 1.5}` patches, eight `Arc3` meridians, every pocket wall's level `z = 8.5` |
| | plate top loop, `r = 1.5` | 160 | −4 | −1 | `15000 − (337.5 − 333π/4) = 14662.5 + 333π/4` |
| | both in one call | | | | `14527.5 + 891π/8 + 27π²/16`; S7 passes (the contours sit 7 mm apart) |
| | pocket floor loop, `r = 1.5` (a fill) | 60 | −4 | +1 | `15000 + 112.5 − 27π`; `sideZ = 6.5`; four concave cylinders meeting along `Ellipse3` edges whose `IsConvex` is false; DF8 reads `1.5` exactly |
| **P3** 40×40×10 ∪ Ø10 boss 15 tall (A1) | boss root, `r = 1` (a fill) | `10π` | `+π` | +1 | `16000 + 375π + (35/3)π − 3π² = 16000 + (1160/3)π − 3π²`; Pappus check `2π[5(1 − π/4) + (5/3 − π/2)/2]`; `Torus{Center: (20,20,10), Major: 6, Minor: 1}`, cap circle radius 6 on the plate, side circle radius 5 at `z = 11` |
| | boss rim, `r = 1` | `10π` | `−π` | −1 | `16000 + 375π − ((25/3)π − 2π²) = 16000 + (1100/3)π + 2π²`; `Torus{Major: 4, Minor: 1}`, the boss top shrunk to radius 4 |
| | both | | | | `16000 + (1135/3)π − π²` |
| **P4** 60×40×8, four Ø5 holes (prism, RF3) | four hole mouths, `r = 1` | `5π` each | `+π` each | −1 | `19200 − 200π − 4((20/3)π − (7/4)π²) = 19200 − (680/3)π + 7π²`; four whole-turn tori `Major 3.5`; the result is a brep body (BF1) |
| | top loop, `r = 1` | 200 | −4 | −1 | `19200 − 200π − (580/3 − 48π) = 57020/3 − 152π` |
| **P6c** 60×40×30, blind 20×10 port 13 deep into one x wall | port mouth, `r = 1.5` | 60 | `+π` | −1 | `69400 − 135 + 225π/8 + 27π²/16` (P2's mouth on a wall face) |
| **P7** L bracket, Ø6 along x through the upright leg | top loop, `r = 1` | 160 | `−5 + π/4` | −1 | `17280 − 72π − (455/3 − (445/12)π − π²/8) = 51385/3 − (419/12)π + π²/8`; five `Ellipse3` edges and one horn torus |
| | one hole rim, `r = 1` (R = 3, on the upright leg's `x = 0` face) | `6π` | `+π` | −1 | `17280 − 72π − ((23/3)π − 2π²) = 17280 − (239/3)π + 2π²`; `Torus{Center: the hole centre on the face, Axis: x̂, Major: 4, Minor: 1}` |
| **P8** 40×20×20, four vertical edges filleted `r = 3`, Ø6 along y | top loop, `r = 2` | `96 + 6π` | `−π` (four LF2 at `β = π/2`) | −1 | `15280 − (384 − (256/3)π − 2π²) = 14896 + (256/3)π + 2π²`; four `Cylinder` and four `Torus{Major: 1, Minor: 2}` (spindle) patches joined by eight `Arc3` quarter meridians, no ellipse |

Bound fixture: the trapezoid `(0, 0)`, `(100, 0)`,
`(72, 45)`, `(28, 45)` extruded 10 with a blind 20×20 pocket 5 deep (a
stacked receiver of 30400, route L's own fixture), top loop filleted `r = 1`.
`filletband`'s `TestCoefficientsOfTheTrapezoid` asserts `κ`, `a₁` and `a₂`
below and checks `A(t)` and `M(t)` against the exact offset polygon at `t = 1`
and `t = 3`. Its slanted sides have
length 53 and `cos θ = 28/53`, `sin θ = 45/53` at the base corners, so
`κ = cot(θ/2) = 9/5` there and `tan(θ/2) = 5/9` at the top corners:
`a₁ = 250`, `a₂ = −212/45`, and the volume `V_receiver − (250(1 − π/4) −
(212/45)(5/3 − π/2))` is an exact rational-`π` closed form. No public-API
test fillets this trapezoid end to end. The checks that fixture needs are
these. Every cap-level vertex's bound must enclose its exact rational
contour corner, and deleting the `F.delta` charge must turn the corners red
(route L's mechanism). The centroid of the body placed along every axis must
enclose the placed exact centroid, and deleting the frame-lift rounding term of
the centroid must turn it red (reach §8.4's placed-plate mechanism). The
centroid's x is exactly 50, which lifts to `10⁶ + 50` with no rounding
for a placement of `10⁶` mm, so a placement along x alone leaves the term
unexercised. The volume's own bound is the enclosures' reach alone and cannot
be made to fail by deleting one term.

Near-tangent and off-radius fixtures (§5.5–§5.7).
`apitest/fillet_near_tangent_loop_test.go` covers the following cases:

- It fillets the start cap loop of a rounded hexagon (corner circle 2.51,
  `sketch.CreateFillet` corner rounds 0.8, 3 tall) by 0.6 and encloses its
  Pappus closed form; most of its arcs are off-radius.
- It fillets the same loop by 0.025 in, an inexact conversion.
- It fillets the R4.5/R22 tangent-arc plate's start loop by 0.5, with arc–arc
  joins and off-radius arcs.
- It fillets the hexagon as a plate's hole rim (σ = −1, material outside).
- It fillets the hexagon as a united boss's root (σ = +1).
- It cuts a cap-chamfered 12 × 10 × 4 arm with the filleted hexagon prism,
  removing its closed-form volume below the arm's top within both bodies'
  bounds.

Each volume test encloses Table CF's closed form, and each closes its mesh.
`internal/filletband/near_tangent_test.go` and `junction_test.go` cover the
following:

- They classify a slot's near-tangent joins as LF5′ with
  `capband.NearTangentCorner`'s `ρ`.
- They read a stepped loop's gap against a 256-bit `||v| − 5|`.
- They sample the true level curves of that loop, stepping both into and out
  of the material, against `LocalReach` and `LevelLength`, and the width of
  the notch a step cuts beside a radius-5 arc and a radius-0.03 hole-rim arc
  against `stepRoot`.
- They compare a restated arc's Table CF piece against §5.6's tube, and
  enclose Lemma 1's seam excess, integrated in 256-bit arithmetic, by
  `MeridianExcess`.
- They pin `RestateMassAllowance`, `MassOf` and the normal turns.
- They refuse a turn past `sin²θ ≤ 2⁻⁶⁴`, a 2⁻⁴⁰ rad arc between near-tangent
  walls, a curved miter beside an off-radius arc, a restated corner past the
  class, and an LF5′ wall ending at a reflex corner.

Refusals: a partial loop outside the selected-chain route → SL1;
P8's top loop at `r = 4` → SX6; the P2 mouth at `r = 5` → SX7;
a partial revolve's cap loop → SX5; a stacked
receiver whose section carries a displacement → SB1. Every refusal leaves the
receiver live. A full revolve's circles are meridian junctions, which the
revolve route rounds (reach §7), so no full-turn revolve fixture refuses here.

### 9. Decided questions

- **No sphere at a convex loop corner; the two pipes meet along an ellipse.**
  §2 states the derivation: the rolling ball never rests at the corner, and
  the octant would expose a shelf. Reach §8.2's miter row ("zero-length
  miter → trimmed `Sphere`") and modify-general §6's first two rows and SL3
  state this; an LF9 sphere needs the third edge's
  fillet (Part V).
- **An `Ellipse3` curve variant, rather than a `Line3`, `NURBSCurve` or
  `FacetedCurve` stand-in.** The miter is an exact conic; a `Line3` tag would
  be a first-order lie (sagitta up to `0.2·r`), a `NURBSCurve` would hold an
  irrational weight, and a `FacetedCurve` says the body holds chords where it
  holds a cylinder (api §4).
- **Admit a regular non-tangent convex corner at a circular walk (LF8).**
  `FilletMiter3` names its pipe intersection. Certified corner allowances
  enclose the difference from the polynomial strip and patch area formulas.
- **Measure through the receiver's restored faces plus the strip integral,
  not through patch flux.** §5.3: no new surface integral, no `π`-phase
  Fourier sums, no chord-versus-locus term, and no `F.delta` in the volume.
- **A prism receiver's loop fillet returns a brep body.** RF3: one band
  record, one reader set; its roles are BF1's, not B1's or BX3's. A caller
  reading `capStart` after a cap-loop fillet reads `face(k)` instead.
- **Curved miter rings follow the offset intersection.** Other rings
  interpolate affinely between the side and cap contours; LF8 seam stations
  solve the actual offset locus at each meridian angle.
- **STEP is analytic where DF10 admits it and faceted elsewhere.** The `step`
  module's `ap214.ToroidalSurface` and `ap214.Ellipse` carry the tori and
  ellipse edges (§12).
- **Restored faces keep the rewritten record's displacements.**
  `breprecord`'s `measureBody` reads each face a fillet band rewrote restored to the
  receiver's level and region (`filletRestored`), but with the level and
  section displacements the rewritten record states, which are never less
  than the receiver's: sound, and zero for every fixture of §8.
- **A near-tangent join is LF5′, built as LF5 and charged, not LF8 and not
  a residual.** LF8's corner disk has radius near `r` and would swamp the
  bound; LF5′ reads `capband.NearTangentCorner`'s exact class and charges
  the sliver's own reach `ρ` in every reading (§5.5).
- **The strip restates off-radius arcs; every other reading charges the
  step locally.** Restatement reuses tessellation-reach §7.1's accepted
  bounds for the volume and moments, which integrate over the whole loop
  (§5.6). The patches, contour and mesh are the record's, built about the
  recorded centre, so each is charged against the true band of the record at
  the corner where its step sits (§5.7). A curved miter beside an off-radius
  arc refuses rather than widen LF8's allowance.
- **SF1 also refuses a corner whose exact class disagrees with the
  contour's join.** The class is the exact turn of the recorded walks; a
  right turn the cap contour does not close with a connector arc, or a left
  or tangent turn it reads otherwise, is no corner of Table LF.
- **`Bounds` reads the band's extents too.** §5.4's extents are added to the
  box as to the through-all extent, since a patch bulges past its
  directrices along an oblique world axis under a placement.
- **The surveys read the patches in closed form.** The undercut survey
  decides each patch from the range of `cos φ·A + sin φ·B(θ)`, with `A` from
  `F`'s frame normal and `B` from the inward normal of `F`'s region along the
  patch's walk or through a reflex corner's turn (Table DF's DF7). Both are
  nondecreasing in `A` and `B`, so their ends bound the whole patch, and the
  three-valued rule of reach DX7 reads them with no departure term.
- **No new 2D question.** The arm asks `sketch` for nothing beyond route L's own offset.

### 10. Do not do this

- **Put a sphere octant at a convex corner.** §2: a shelf of area
  `r²(1 − π/4)` at the side level. An LF9 sphere needs the third edge's
  fillet (Part V).
- **Tag the miter edge `Line3` with a length bound, as the chamfer's ruling
  is.** The chamfer's ruling departs from its conic locus at second order;
  the fillet's miter departs from a chord at first order.
- **Rule the patch between its two directrices and tag it `Cylinder`.** The
  chamfer's `Cone` stand-in needed reach §8.3's skew, departure and locus
  terms; the fillet builds the exact pipe and owes none of them.
- **Integrate patch flux with trigonometric endpoint terms.** §5.3's strip
  model is polynomial in `t`; the only transcendental is `π`.
- **Compute `V_body` from the rewritten record's faces plus a closed band
  region.** The restored record plus `σ·V_strip` is the same number with no
  displacement charge and no disk bookkeeping.
- **Solve each tessellation ring's entire offset contour separately.**
  §7.1's affine interpolation still handles ordinary corners; LF8 solves
  only the curved miter's offset intersection.
- **Tag a line–circle miter as a chord.** The pipe intersection is a space
  curve; `FilletMiter3` keeps its analytic identity and carries a chord bound
  only when tessellated.
- **Read `F.delta` into the volume bound.** The strip model reads the
  recorded loop; `F.delta` belongs to `F`'s own area and the cap-level
  vertices.
- **Decide a tangent join by comparing float tangents.**
  `capband.JoinIsG1` uses exact rational arithmetic on the recorded segments.

### 11. PR split

The fillet arm is complete; no later PR is planned. Its code lives in `internal/filletband/` (Table CF readings),
`internal/breprecord/` (`attachFilletBand`, the restored-face sum, `ExtentAlong`),
`internal/brepfilletmesh/` and `internal/brepbandmesh/` (§7.1 rings), `internal/survey2d/` and
`internal/verifyrun/brep.go` (DF7/DF8 surveys), and `export/step_analytic.go` with
`export/step_analytic_torus.go` (DF10). Still refused: SF1, SB5 (single edges sharing a vertex outside
route V), SX5 (revolve cap edges), SL1 (partial loops outside the selected-chain route), the clearance
model (DF6, `Suspect`), and variable-radius fillets (no entry point).

### 12. Hand-off

DF10's analytic STEP uses `ap214.ToroidalSurface(id, name, position, majorRadius, minorRadius)` and
`ap214.Ellipse(id, name, position, semiAxis1, semiAxis2)` from `github.com/lestrrat-3d/step`.
## Part V — Three-edge vertex blends

`Fillet` admits a complete planar-face loop together with independent
straight edges that end on that loop. The straight-edge rewrite runs first;
the loop rewrite then fillets its new arc segments. A circular wall whose
recorded radius equals the requested fillet radius produces a sphere patch at
the shared vertex.

This part narrows the selection refusals in
Part B Table SB,
Part G Table SL, and
Part F Table S. It adds LF9 to
Part L Table LF. The ordinary route E, route L, and
loop-fillet gates still apply to each step.

### 1. Geometry and record

For an inward circular walk of radius `R`, the loop fillet's rolling-ball
centre follows a circle of radius `R-r`. At `R=r`, the circle is one point:
the walk's recorded centre `C` at the side level. The resulting patch is a
`Sphere` of radius `r`, from the side arc at the ball's equator to the
single pole at the cap level. Its two other edges are the neighbouring
straight walks' quarter-circle meridians. A right-angle box corner gives
one sphere octant.

The radius comparison first checks the held `float64` readings, then checks
in rational arithmetic that both pinned endpoint-to-centre distances squared
equal `r²`. It does not turn a near-equal arc into a sphere. Making a
sphere from a different radius would leave a gap along the existing side arc.

An LF9 walk must be open, inward, and exactly tangent (G1) to a straight
walk on each side. Both offset joins use the recorded centre `C`; the
collapsed arc contributes no cap-contour segment. The two straight offset
segments meet at `C`, which is the pole's planar coordinate. Chamfer and
ordinary offsets still reject a dropped circular carrier.

| Class | Loop walk or corner | Result |
|---|---|---|
| LF9 | inward open arc, both recorded endpoints exactly at `r`, straight G1 neighbours | sphere patch with one side arc and two meridians |
| LF2 | inward arc, `R > r` with regular cap contour | torus patch |
| LF3 | outward arc | torus patch with major radius `R+r` |
| LF4 | sharp convex corner between two selected loop lines | two cylinders meet along an ellipse |
| LF5 | exactly tangent join | one shared meridian |

The strip measurements use the existing loop-fillet coefficient integral.
For LF9's arc sweep `β`, its coefficients are `a₁=βr` and `a₂=-β/2`.
The receiver already includes the straight-edge fillets before the loop
strip is subtracted or added. The sphere patch has area `βr²`.

### 2. Selection and build

Route V partitions a fillet selection into complete planar-face loops and
independent straight edges along reference axes. For a box-like selection
with several possible complete loops, it first tries each reference axis as
the straight-edge set; the remaining edges must form complete, non-sharing
loops. Otherwise it assigns each edge covered by one complete loop to the
loop set and checks the remaining straight edges for shared vertices.
An open selection on one planar-face loop forms one or more maximal chains
of selected walks. Each selected walk's cap offset is `r`; every unselected
walk stays at offset zero. Table LB's LB3, LB4 and LB6 read the faces beside
the selected walks alone: they must all descend into the body or all rise
off it, and that sense is the band's. The faces beside unselected walks keep
their geometry, so they are not read and may lie on either side of the face.
A block standing on a plate flush with the plate's side is the case: the
plate top's loop runs down the plate's side and up the block's, and the
block's root chain rises. Route E reads an end-face corner only at a chain's
two ends; the band builds every corner inside the chain from the cap
contour, so the third face at an interior vertex, the next selected wall,
is not read as an end face. A corner between two selected convex straight
walks uses LF4's miter and ellipse. A reflex corner between selected walks
keeps the fillet connector arc and its horn-torus patch. A selected-to-
unselected corner uses the sharp intersection of the moved and fixed
carriers and ends the pipe on an arc in the unselected wall's plane.
The selected wall ends at the band's side level. The unselected wall keeps
its original reach and replaces its corner with that terminal arc. A chain
with only one walk is route E's existing single-edge case.

A single cap edge of a strictly convex, straight-sided prism has a second
path when the edge or either terminal wall is oblique. A quarter-cylinder
cutter extends beyond both cap and side planes, then the mesh boolean trims
it against the prism's actual walls. This keeps the curved terminal cut on
an oblique wall without restating that wall as an axis plane. The result is
`Faceted`, with the boolean's mesh and measurement bounds. The admission
requires one selected edge, no holes or recorded displacement, more than
four radii of cap height and clearance from other section vertices, and
terminal wall directions whose trim travels less than four radii along the
selected edge. A failed contact proof leaves the receiver live.

A single-edge chamfer under the same convexity and clearance gates cuts a
straight wedge instead. Its cutter has a diagonal from the cap setback foot
to the side setback foot and extends beyond both adjacent faces. An
asymmetric reference face assigns those two distances before construction.
The mesh boolean trims the wedge at oblique terminal walls and publishes its
faceted result with a volume bound. A contact refusal leaves the receiver
live.

Two selected straight edges can meet on a swept wall whose loop is absent
from the initial planar-face record. Route E restates that wall as a planar
rectangle before route L looks for a partial loop. The partial-loop band then
closes the shared corner on the restated face. The two-edge corner of a box
at one top and one vertical edge uses this path.

Three selected edges at one box corner first fillet the edge across the top
cap. The two shortened top edges and their new arc then form one selected
partial loop. The arc's exact radius equals the requested radius, so its cap
walk collapses to the arc centre. Route L builds a sphere patch between that
pole and the arc at the band's side level. Only the far ends of the two top
lines need terminal arcs; their ends beside the new arc meet the sphere.

The band record keeps the original complete loop, a selected-walk mask,
and the cap-contour segment for each selected walk and reflex connector.
An exact-radius selected arc has no cap-contour segment and records `-1`.
Only selected cap and side edges and reflex connector arcs are open in the
face record. Its terminal arcs are also open and each pairs with one band
patch. At a convex join between selected walks the ellipse pairs the two
band patches. The cap face,
trimmed walls and terminal walls remain ordinary brep faces. The partial
band's measured strip sums only selected walks; a selected-to-unselected
straight join uses the signed end rate `-cot(θ)` of the offset line against
the unselected line. Its mesh shares the terminal arc's samples with the
unselected planar wall and the cap and side samples with their faces. The
terminal arc is pinned to the cap contour's endpoint, including at a hole
mouth where the cap contour moves away from the hole.

The straight-edge set runs through route E (`brepmodify.BlendEdges`). A prism is
read through `breprecord.OfPrism` first; a single straight prism cap edge also
uses route E for a chamfer. Its two end-face corners take straight chords,
the cap and side faces end at the chords' feet, and a planar bevel closes
them. Route E keeps face and loop indices. Route L reads those
indices on the rewritten brep, including arcs route E added to the loops. Route L records its loop bands and builds one brep result. The
public `Fillet` call commits only that final result. A single-edge
`Chamfer` commits route E's result.

`capband.PartialFilletContourDelta` drops the collapsed arc from the cap contour
and joins its straight neighbours at the pole. `attachFilletBand` creates a
three-edge sphere face: the side arc, the trailing meridian in reverse, and
the leading meridian. The two neighbouring cylinder faces share those
meridians. A complete circular walk still takes LF7's torus path.

### 3. Meshing and proof

The band chords each wall walk once. Its side, interior, and cap rings keep
matching sample counts. LF9's cap samples all lie at the pole, so the planar
cap triangulator removes consecutive duplicate samples from its own ring.
The band retains them to form the triangular sphere strips. The mesh still
passes the vertex-link and facet-area audits.
The partial-loop mesh uses the swept arc wall's chord count for the sphere
side, so its samples meet that wall at the same vertices.

`CapOffsetStationBound` bounds the ideal cap stations for the occupied
volume proof. At an exactly collapsed radius, every station is exactly the
recorded centre; `circularbounds.OffsetEndpointInterval` returns that point
interval directly. The other displacement and volume terms remain the
loop-fillet terms.

### 4. Refusals

| Input | Result |
|---|---|
| loops sharing an edge after the partition | SL1, `ErrUnsupported` |
| selected straight edges sharing a vertex with no common planar-face loop after straight-wall restatement | SL1 or SB5, `ErrUnsupported` |
| a partial loop whose selected walls lie on both sides of the face | SL2, `ErrUnsupported` |
| a single oblique prism cap edge outside the convex cutter admission, or a cutter whose boolean contact cannot be proved | route E's refusal or the boolean's `ErrUnsupported` |
| an LF9 candidate with a circular neighbour or a non-G1 join | SF1, `ErrUnsupported` |
| held radius `r` but either recorded endpoint has a different exact distance to the centre | `ErrUnsupported` |
| recorded inward arc radius slightly above `r` but its residual is within `shellTol` | `ErrUnsupported`, naming the unequal radii |
| inward arc radius below `r`, or another offset feature consumed | SX6, `ErrDegenerate` |
| an exact-radius whole circle | existing LF7 path; it does not use LF9 |
| mixed loop and single-edge `Chamfer` | existing SL1 refusal |

A trapezoid with slanted sides demonstrates the near-equal refusal: one
rewritten arc has a held radius a few rounding steps above the `1 mm` fillet
radius, while another has an endpoint slightly off its exact radius-one
circle. The record does not define exact spheres there.

### 5. Integration fixtures

| Receiver and selection | Expected result |
|---|---|
| 40×20×20 box, all 12 edges, `r=2` | 8 spheres; volume `14848 + 848π/3` |
| same box, two adjacent top edges, `r=2` | one ellipse seam; volume `47320/3 + 56π` |
| same box, three adjacent top edges, `r=2` | three cylinders, two ellipse seams, closed mesh |
| 40×20×10 plate, 10×6×5 block flush with its side, three root edges, `r=1` | three cylinders, two horn tori; volume `8322 − 14π/3 − π²/4` |
| same plate, block flush in its corner, two root edges, `r=1` | two cylinders, one horn torus; volume `8316 − 43π/12 − π²/8` |
| pocket mouth, three of four edges, `r=1.5` | three cylinders, two horn tori, closed mesh |
| same box, top loop and 4 vertical edges | 4 spheres; volume `15264 + 544π/3` |
| same box, top loop and 2 vertical edges | 2 spheres and 2 ellipse seams |
| same box, one straight top edge | route E brep; volume `15840 + 40π` |
| trapezoid prism, one oblique top edge | faceted quarter-cylinder cut; volume bounds enclose `32400 − 53(1 − π/4)` |
| 40×20×10 box, one straight top edge, `d=2` | route E brep; volume `7920` |
| same box, one top edge, cap/side setbacks `2` and `3` | route E brep; volume `7880` |
| trapezoid prism, one oblique top or bottom edge, `d=1` | faceted wedge cut; volume bounds enclose `32400 − 53/2` |
| same trapezoid, one oblique top edge, cap/side setbacks `1` and `1.5` | faceted wedge cut; volume bounds enclose `32400 − 53·0.75` |
| trapezoid prism, horizontal top edge beside oblique walls | faceted quarter-cylinder cut; volume bounds enclose `32400 − 44(1 − π/4) − (28/45)(5/3 − π/2)` |
| 40×40×10 plate with 20×10×5 pocket, floor loop and 4 vertical edges, `r=1.5` | 4 spheres; volume `15153 - 297π/8` |
| cross-drilled 40×20×20 bar, all 12 straight edges, `r=2` | 8 spheres; volume `14848 + 308π/3` |
| rounded plate P8, top loop over radius-3 side arcs, `r=3` | 4 spheres; volume `14416 + 207π`; tessellates |

The volume column is the hand derivation for each row. Tests assert the volume of the rows `14848 + 848π/3`
(`apitest/brep_loop_fillet_test.go`), `47320/3 + 56π` (same file), `8322 − 14π/3 − π²/4` and `8316 − 43π/12 − π²/8`
(`apitest/brep_partial_root_fillet_test.go`), `7920` (`apitest/chamfer_test.go`), `7880`
(`apitest/modify_options_test.go`) and the two trapezoid wedge rows `32400 − 53/2` (`apitest/chamfer_test.go`) and
`32400 − 53·0.75` (`apitest/modify_options_test.go`). The other volume figures (`15264 + 544π/3`, `15840 + 40π`,
`15153 − 297π/8`, `14848 + 308π/3`, `14416 + 207π` and the trapezoid quarter-cylinder rows) are derived values that no
test asserts. The tests also assert closed topology, bounded volume readings, patch kinds, and the cap mesh for the
radius-equality cases. The near-equal trapezoid case of section 4 asserts a typed refusal and leaves the receiver
live.

## Part A — Edge blend variants

One part per edge blend variant. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N".

### Chord fillet

`FilletChord` rounds an edge with an arc stated by its chord, the straight line between the arc's two tangent edges.
It is a feature-level entry point over `Fillet`: it computes the radius and adds no geometry kernel. The fillet
routes are Part F §6 and Part R; this document owns the
conversion.

#### 1. Contract

```go
func FilletChord(ctx context.Context, b *Body, sel EdgeSelector, chord units.Value) (*Body, error)
```

| Decision | Rule |
|---|---|
| C1 | Where the two faces meet at the angle φ across the corner the arc turns π − φ, so its radius is `chord/(2·cos(φ/2))`, which is `chord/√2` on a right angle. |
| C2 | `FilletChord` calls `Fillet(ctx, b, sel, radius)`. Receivers, routes, history entry (`OperationFillet`) and refusals are `Fillet`'s. |
| C3 | `chord` is a positive length. Zero is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. |
| C4 | Each selected edge has exactly two faces, both `Plane`, and is convex or concave. φ is the interior angle of a convex edge and the open wedge between the faces of a concave one, which the round fills. Any other edge is `ErrUnsupported`. |
| C5 | A nil context or body is `ErrDegenerate`. A selector `Fillet` would refuse is passed to `Fillet` for its own error. |
| C6 | `Fillet` takes one radius, so every selected edge MUST give the same one within a relative 1e-12, which equal dihedrals do. A selection that mixes angles is `ErrUnsupported`: take the angles in separate calls. |

#### 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes: the angle
between them is `atan2(|n₁ × n₂|, n₁ · n₂)`, accurate near a flat edge and near a knife edge as well as at a right angle,
and φ is π less that. It is data of the recorded planes, not a test of a claim, so it blesses nothing: the radius is a
float function of those planes, and the result is `Fillet`'s result for a radius of that value. `ChamferAngle` reads its
dihedral the same way (Part A, Angle chamfer §2).

The same-radius test of C6 only groups edges. Two equal dihedrals read from different face pairs differ in rounding,
far below 1e-12, and the one radius used differs from the other edge's by less than that, far below the bound of the
result `Fillet` publishes.

`WithTangentChain` is not offered: the chain would add edges C4 has not checked.

#### 3. Exactness

The radius is a float, so a result is as exact as `Fillet`'s for a radius of that value. The tests compare volumes
against `(1 − π/4)·(chord²/2)·height` on a right angle within a relative `1e-9`, and against `R²(cot(φ/2) − (π − φ)/2)·height`
on the 120 degree edges of a hexagonal prism.

#### 4. Not covered

- A curved face beside a straight or free-form edge, and a torus: the angle there varies along the edge.
- `WithTangentChain`.
- Setback corners and curvature-continuous blends, which have no entry point.

#### 5. Edges that are not right angled

The hexagonal prism's six vertical edges meet at 120 degrees. `FilletChord` of one gives the radius `chord`, and the
tests compare the volume with the area `R²(cot 60° − 30°)` a fillet of that radius removes from the corner. All six
together give one radius and are one call. A right-isosceles triangular prism's 45, 45 and 90 degree edges together are
`ErrUnsupported`, and the L-shaped prism's reflex edge is a concave edge: its faces turn 90 degrees, so a chord of `2√2` is a radius of 2 and the round adds `(1 − π/4)·r²·height` to the corner.

#### 6. A circular edge between a plane and a surface of revolution

An edge that is a `Circle3` or an `Arc3` may lie between a plane and a `Cylinder`, `Cone` or `Sphere`, or between two of
those: such a circle lies on a surface of revolution about its own axis, so it meets the other face at one angle all round.
The rim of a rod, the mouth of a bore and the foot of a boss are such edges.

| Decision | Rule |
|---|---|
| C7 | φ is read from the two faces' outward normals at the edge's start, as `planarDihedral` reads it for planes. The radius is `chord/(2·cos(φ/2))` as C1 says, which is `chord/√2` for the right angle between a cap and a wall. |
| C8 | A curved face beside a straight or free-form edge, and a `Torus` anywhere, is `ErrUnsupported`: the angle there varies along the edge. Two faces that meet tangentially have φ of π and are refused. |
| C9 | The result is `Fillet`'s for that radius, so its refusals apply. The tests compare a rod's two rims with `∫ 2π(R−x)(r−√(r²−(r−x)²))dx` over the round. |

### Angle chamfer

`ChamferAngle` bevels an edge by a distance along a reference face and an angle between the bevel and that face. It
is a feature-level entry point over `Chamfer` and `WithAsymmetricChamfer`: it computes the second setback and adds no
geometry kernel. The setback model is Part R §6; this document owns the conversion.

#### 1. Contract

```go
func ChamferAngle(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, reference FaceSelector,
    angle units.Value) (*Body, error)
```

| Decision | Rule |
|---|---|
| A1 | The bevel leaves the reference face `d` from the edge and makes `angle` with that face. It meets the other face `d·sin(angle)/sin(φ + angle)` from the edge, where φ is the interior angle between the two faces. That is `d·tan(angle)` on a right angle. |
| A2 | `ChamferAngle` calls `Chamfer(ctx, b, sel, d, WithAsymmetricChamfer(reference, other))`. Receivers, routes, history entry (`OperationChamfer`) and refusals are `Chamfer`'s. |
| A3 | `angle` is strictly between 0 and a half turn. Zero or a half turn or more is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. An edge whose φ makes `φ + angle` reach a half turn has a bevel that never meets the other face: `ErrDegenerate`. |
| A4 | `d` is a positive length under `Chamfer`'s own gates. |
| A5 | Each selected edge has exactly two faces, both `Plane`, and is convex or concave. A concave edge's φ is the open wedge the faces make in the air, which the bevel fills, so the same formula holds. Any other edge is `ErrUnsupported`. |
| A6 | A nil context, body or reference is `ErrDegenerate`. A selector `Chamfer` would refuse is passed to `Chamfer` for its own error. |
| A7 | `WithAsymmetricChamfer` takes one second distance, so every selected edge MUST give the same one within a relative 1e-12, which equal dihedrals do. A selection that mixes angles is `ErrUnsupported`: take the angles in separate calls. |

#### 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes, as
Part A, Chord fillet §2 states: the angle between them is `atan2(|n₁ × n₂|, n₁ · n₂)` and φ is π less
that. The second distance is a float function of those recorded planes and blesses no claim about them, so a result
is as exact as `Chamfer`'s asymmetric result for a distance of that value. The same-distance test of A7 only groups
edges, and the one distance used differs from another edge's by less than a relative 1e-12.

`WithTangentChain` is not offered: the chain would add edges A5 has not checked.

#### 3. Exactness

The second setback is a float, so a result is as exact as `Chamfer`'s asymmetric result for a distance of that value.
The tests compare volumes against `0.5·d·other·sin(φ)·height` within a relative `1e-9`, and they recover the bevel's
angle from the volume independently of the formula.

#### 4. Not covered

- A curved face beside a straight or free-form edge, and a torus: the angle there varies along the edge.
- The angle measured from the other face. Name the other face as `reference`.
- `WithTangentChain`.

#### 5. Edges that are not right angled

The hexagonal prism's 120 degree edges and a right-isosceles prism's 45 degree edge take the sine rule. A 120 degree
edge with a bevel angle of 60 degrees or more never meets the other face and is `ErrDegenerate`, and a bevel that
reaches past the face's own width is `Chamfer`'s refusal.

#### 5. A circular edge between a plane and a surface of revolution

`ChamferAngle` takes the edges `FilletChord` does in Chord fillet §6: a circular edge
between a plane and a `Cylinder`, `Cone` or `Sphere`, whose angle is one all round. φ is read as A1 says and the second
distance follows A1. A curved face beside a straight or free-form edge is `ErrUnsupported`. The tests compare a rod's rims
with the ring a triangle of legs `d` and `d·tan(angle)` sweeps about the axis.

### Oblique boss fillet

This design covers one complete junction between a rectangular plate's top
plane and a rooted circular cylinder whose axis rises obliquely. It extends
`agent-docs/prism-boolean-design.md` Part G without
changing either operation's public API.

#### Source and admission

The ordinary mesh `Union` remains responsible for finding contact and
proving the unmodified joined body. After a successful `Union`, it may retain
the two original prism records when one is an axis-aligned rectangular plate
and the other is a whole circular cylinder. The cylinder's near cap must lie
strictly below the plate's top, its far cap strictly above it, and its entire
projected intersection and requested fillet must lie inside the plate top.
The plate bottom and all four walls remain untouched. These inequalities
include the operands' recorded placement and section errors. A partial
junction, a side contact, a detached cap, or a tilted plate is not admitted.
The bounded implementation admits a 1–20° tilt, a cylinder length at least
its radius, radius and length at most 100 mm, and source coordinates within
10,000 mm. The recorded section and levels must have zero displacement.
The fillet radius is positive and below one quarter of the boss radius.

`Fillet` accepts exactly the one complete, closed junction edge between the
plate top and cylinder side. It rebuilds from the retained prisms; the mesh
edge is only a selection witness. Other edge selections keep their existing
refusal. The result is a faceted body with a dedicated blend face and source
provenance on the retained faces.

#### Geometry

Let `H` be the plate top, `b` the cylinder's near-cap center, `a` its unit
axis, `R` its radius, and `r` the fillet radius. For radial unit vector
`d(u)` perpendicular to `a`, a rolling-ball center is

```
C(u) = b + (R+r)d(u) + a (H+r-b.z-(R+r)d.z(u))/a.z.
```

Every `C` lies in the plane `z=H+r`. The plane contact is `P=C-r ez` and the
cylinder contact is `S=C-r d`. The blend patch is

```
F(u,v) = C(u) - r normalize((1-v)ez + v d(u)),  0<=v<=1.
```

Both contact curves are shared vertex-for-vertex with their neighbors. The
top plate is the square-to-`P` annulus; the cylinder side is the `S`-to-far
cap ruled band. The mesh also contains the original plate walls, bottom cap,
and boss far cap.

#### Proofs and refusal

The source placement and exact Sketch section bounds precede every mesh
claim. Rational intervals enclose every sampled sine, cosine, normal length,
and vertex position. The complete angular plan has gaps below 0.05 radians;
each blend interval is 1/16. For the admitted basis, `|q|>0.56` throughout
the blend. Second-derivative bounds on `F` charge angular interpolation,
blend interpolation, and the diagonal split into triangles. The near-unit
source radial basis adds at most `0.02r` where the blend meets the ruled
cylinder. The placed circle's two certified caps and linear ruling contribute
their source displacement, amplified by 16 through the plane crossing and
unit normal. The amplification covers at most `4ε/R` of radial-normal change
for a near-rigid affine image with cap displacement `ε`, `r/R<1/4`, plane
division by `a.z>0.94`, and normal division by `|q|>0.56`; the resulting
position change is below `16ε`. The resulting boundary bound must be below
0.1 mm.

The angularly ordered square and ellipse rings make a planar annulus; the
ruled cylinder side and rolling-ball band share their contact vertices.
An exact mesh audit checks closure, orientation and crossings. For occupied
volume, a full rectangular collar around the junction charges every possible
source-to-held change in the fillet zone. Outside that collar, a radial
annulus over the cylinder length and cap slabs enclose the remaining chord
and placement difference. The bound must be finite and leave a positive
lower volume. A clean mesh audit alone never supplies a source proof.

The output does not claim an analytic torus: its rolling-ball center follows
an ellipse, so a torus would be the wrong surface. Placement of the result
uses the existing faceted placement proof and drops the one-use source record.

## Part E — Retained edge blend

`Fillet` and `Chamfer` of an edge of a mesh Boolean result whose two faces
carry retained analytic surfaces (`agent-docs/faceted-source-retention-design.md`,
"retention §N" below). Companion to Part F ("modify §N"),
which owns the call contracts and base gates, and to
Part R ("reach §N"), whose SX9 row this document
narrows. Nothing here changes a prism, revolve or brep receiver.

### 1. Problem

Retention §1, row P4: a 40×20×20 box with four
filleted lateral edges, cross-drilled by a tool that a `Transform` placed, then
`Fillet` of the four x-parallel cap edges. The call returns `ErrUnsupported`
(reach SX9). The same box drilled by an unplaced tool is a `brepPayload`
(general-boolean class B) and takes the brep routes. A drilled plate whose
tool was placed, rotated or cut at an angle is the common machining shape, and
it is the shape the mesh path exists for.

Three things stop the call.

1. **No analytic edge.** Before retention an edge of a mesh result was a chord
   chain (`FacetedCurve`) between two `Faceted` faces. Retention §10 now
   publishes `Line3`, `Circle3` and `Arc3` for edges between two retained
   faces, and §10.5 adds `Ellipse3`.
2. **No blend by Boolean.** A rolling-ball blend can be written as a Cut by a
   cutter solid, but the cutter's blend face is tangent to both faces it cuts.
   The mesh Boolean refuses that contact (`an operand edge grazes along the
   other operand's facet`, evaluator §9), and it should.
3. **No record the brep routes read.** `brepPayload` keeps every frame a signed
   permutation of one reference frame (general-boolean §4.1). A rotated or
   oblique result has no such record, which is why it is a mesh result.

This design builds the blend as a local rewrite of the held mesh, in the way
the prism route rewrites a section (modify §2) and reach §10.1 rewrites a
Union's source. The rewrite replaces the facets near one edge by facets that
lie on exact surfaces computed from the two retained tags.

### 2. What the design reads

| Input | Source | Used for |
|---|---|---|
| The two retained tags `T1`, `T2` | retention §3 | the exact surfaces the blend is tangent to |
| The edge's exact curve | retention §10 | the line or circle the blend is measured from |
| Held facets and vertex bounds | retention §3.3, faceted-vertex-bounds §2 | what the rewrite cuts and keeps |
| The slab bound of a retained plane facet | retention §11 | how far a cut point on a held facet sits from the tag plane (§6.3) |

Tags with a stated departure (`NormalBound != 0`, a placed float image) are not
read: their plane or axis may differ from the true one, so an exact setback
computed from them has no proof (SR1). A tag moved by an exact translation
states a zero departure and is read: a `Transform` whose moved origin is the
exact rational sum, whether it moved the Boolean result or an analytic operand
before the Boolean (retention §3.5), and the tags of a piece the shifted Boolean retry
translates (`agent-docs/gap-tolerance-design.md` §5.2), whose snapped shift makes that
sum exact for ordinary coordinates (retention §3.4). Such a blend is built on
the moved solid `A∘(B+s)`. A positional displacement stated in `AxialDelta`
is read as a charge, not refused: on a plane by every route, and on a
cylinder wall by the closed rim alone (§6.1, "Displaced planes and walls").

A face with a denoted patch (every Revolve wall and end face, `topo.Face.Denoted`)
is read only where its tag IS the surface its record denotes, decided by
`stitchflux.TagIsDenoted` on recorded data by exact equality: every leaf of the
patch is a point, the axis frame is exactly orthonormal in rationals, and the
tag equals the denoted cylinder or plane exactly. Nothing is admitted on a
residual. Only a `Plane` or `Cylinder` tag is considered here: R1–R5 refuse a
`Cone`, `Sphere` or `Torus` tag before they ask `TagIsDenoted`. A widened patch,
a partial-sweep cap, a rotated placement and an off-axis sketch line fail the
test and stay SR1. An admitted face is read as a retained `Plane` or `Cylinder`
tag, and every other gate of the route (the edge class, the end faces, the
setback and clearance gates) applies to it as to any retained face.

A `Sphere` face is read only by R6, through its own exact-record rule (§4.5):
`TagIsDenoted`'s sphere arm reads the centre and no radius, and R6 needs both.

**Interior vertices of an end face.** A revolve's end disc is triangulated as a
fan around a centre vertex on the axis, so the held end face has a vertex that
no boundary loop reaches. An end face `G` may hold such a vertex only where it
lies exactly on `G`'s plane tag (rational test of `n·(x − o) = 0` on the held
normal, origin and vertex). The same holds for the plane face of an R2 rim and
the base plane of a chain. Each of these faces is rewritten by discarding all
its facets and triangulating it again from its boundary loops alone (§5), so
the dropped vertex reaches neither the result nor its bounds, and the
exact-plane test shows it never carried geometry off the plane. A vertex off
the plane by any amount, and any stray vertex on a wall face, still refuses
(SR5).

### 3. Edge classes

Table RR lists the edges a blend can be built on. The others are refused.

| RR | Edge | Faces | Blend surface (fillet / chamfer) | Staged |
|---|---|---|---|---|
| **R1** | `Line3`, convex or concave | two retained `Plane`s, not parallel, ending at three-face corners whose third face is a retained `Plane` | `Cylinder` / `Plane` | increment B2; equal or two-distance chamfer |
| **R2** | `Circle3`, closed, centre on the axis | retained `Plane` and coaxial retained `Cylinder` | `Torus` / `Cone` | increment B1; equal or two-distance chamfer |
| **R3** | `Line3` of retention E4 | retained `Plane` and a `Cylinder` whose axis lies in the plane, ending on retained planes normal to the axis | `Cylinder` / `Plane` | increment B3; equal or two-distance chamfer |
| **R4** | `Arc3` | as R2, open, ending on planes or cylinders parallel to the axis | as R2 | ends on two distinct retained planes, or (concave fillet only, the wall running on past the end) on retained cylinders parallel to the axis whose material lies outside them; equal or two-distance chamfer on planes; an arc whose ends meet one face is refused; a concave arc's wall may run on past an end plane and lie on both sides of the plane (§4.4); several arcs of one selection share a two-sided wall only when every selected edge is an open arc and their ends are cylinders; several plane-ended arcs on a two-sided wall keep the whole-face refusal |
| **R5** | `Line3` of retention E10 | two retained `Cylinder`s with parallel axes, ending on retained planes normal to the axes | `Cylinder` / `Plane` | reach §8; equal or two-distance chamfer |
| **R6** | `Circle3` of retention E5, closed | retained `Plane` and a retained `Sphere` whose tag is exactly its record (§4.5) | `Torus` / `Cone` | §4.5; equal or two-distance chamfer |
| **R7** | closed saddle, any held curve | a retained `Cylinder` shaft (material inside) and a retained `Cylinder` bore of radius `r` (material outside), unit coordinate axes on two coordinates meeting exactly, `2r² < R²` | (next row) / ruled band held `Faceted` | §4.9; equal-setback `Chamfer`, by a mesh `Cut` |
| **R7 fillet** | a closed saddle, the ball outside the bore: inside the larger cylinder at a convex edge (σ = (−1, +1), the R7 mouth), or outside both at a convex edge (two crossing bores) or a concave one (a boss's root on a rod) (σ = (+1, +1)) | two retained `Cylinder`s of radii `R > r` with R7's axes and ratio: R7's faces, both `Reversed`, or neither | pipe held `Faceted` / (row above) | §4.10; `Fillet` by a rewrite of both walls; `2ρ < R − r` inside the larger cylinder |

Refused, with the reason a caller can act on:

- **An oblique plane and cylinder (`Ellipse3`).** The surface that rolls a ball
  between them is a pipe surface about an ellipse offset. It is neither a
  `Cylinder`, `Cone`, `Torus` nor `Sphere`, and the only other surface the
  vocabulary has is `NURBSSurface`, an opaque marker with no geometry to carry
  a proof (spline §7). The rim keeps its published ellipse and the blend
  refuses (SR1).
- **Any `Cylinder`/`Cylinder` rim with crossing axes outside R7 and its fillet
  row (a pin's root inside a tube's bore; a ball inside the bore; a bore off the
  larger cylinder's axis or off the coordinate axes; `2r² ≥ R²`; a chamfer of
  the fillet row's σ = (+1, +1) patterns), an open (`Arc3`) plane-sphere rim, a
  `Cone` or `Torus` face.** The ball rolled between crossing cylinders sweeps a
  canal surface about a quartic, which the vocabulary cannot carry as a tag, and
  only the R7 fillet row's sign patterns have a proven faceted pipe (§4.10). No
  Table RR row; SR1.
- **A rim with a face that is not retained,** or whose edge is a chord chain
  because retention §10 declined it. SR1.

Both the convex and the concave edge are built. A concave fillet adds
material in the corner, a convex one removes it. The two share the clip and
strip construction below; only the side of the faces the strip lies on and
the sign of the blend face's orientation change.

### 4. The exact blend

All values are computed in exact rational arithmetic from the tags' float
parameters and rounded once, upward where a bound is wanted.

#### 4.1 R1, a straight edge between planes

Let `nA`, `nB` be the faces' outward unit normals (the frame normal, negated
for a `Reversed` face), `d̂ = (nA × nB) / |nA × nB|` the edge direction and
`φ ∈ (0, π)` the angle between the normals, `cos φ = nA · nB`. Let `e₀` be a
point of the edge line. The edge is convex when the face `A`'s inward
in-plane direction `uA` (the unit vector along `±(nA × d̂)` that points into
`A`'s held facets) satisfies `nB · uA < 0`; the held `Edge.Convex` flag must
agree or the call refuses (SR1). Let `s = −1` for a convex edge and `s = +1`
for a concave one.

| Quantity | Fillet radius `r` | Chamfer distances `dA`, `dB` |
|---|---|---|
| Setback on `A` and `B` | `t = r · tan(φ / 2)` | `dA`, `dB` |
| Setback line on `A` | the line at in-plane distance `t` from the edge, parallel to it, inside `A` | the same at `dA` |
| Setback line on `B` | the same at `t` | the same at `dB` |
| Blend surface | `Cylinder`, axis `d̂`, radius `r`, through `e₀ + s·b̂·r / cos(φ/2)` with `b̂ = (nA + nB) / |nA + nB|` | `Plane` through the two setback lines |
| Blend face orientation | `Reversed = false` convex, `true` concave | the plane's own normal, signed to point away from the material |

*Why.* A ball of radius `r` tangent to both planes has its centre on the line
parallel to the edge at distance `r / cos(φ/2)` along the bisector, and touches
each plane at distance `r · tan(φ/2)` from the edge. A 90° box edge has
`φ = π/2`, `t = r`, centre offset `r√2`, which is the familiar quarter circle.

The cylinder meets each face tangentially along its setback line, so the
construction needs the setback lines and nothing else of the cylinder. The
chamfer plane contains both setback lines, which are parallel.

#### 4.2 R2, a closed circular rim

Take the meridian half-plane `(ρ, z)` about the common axis, with the plane at
`z = 0`, the cylinder at `ρ = R` and the rim at `(R, 0)`. The ball sits on the
material side for a convex rim and the void side for a concave one, and on the
outside of the cylinder (`ρ > R`) or the inside.

| Rim | Convex or concave | Ball centre `(ρ, z)` | `Torus.Major` |
|---|---|---|---|
| hole mouth: material at `ρ > R`, `z < 0` | convex | `(R + r, −r)` | `R + r` |
| boss root: boss at `ρ < R`, `z > 0`, plate at `z < 0` | concave | `(R + r, +r)` | `R + r` |
| shaft end: material at `ρ < R`, `z < 0` | convex | `(R − r, −r)` | `R − r` |
| blind-hole floor: void at `ρ < R`, `z > −h` | concave | `(R − r, −h + r)` | `R − r` |

The torus has `Center` on the axis at the centre's height, `Minor = r`, and the
blend face is the quarter of it between the two tangent circles: radius
`R ± r` on the plane (setback `r` from the rim along the plane) and height `r`
on the cylinder (setback `r` along the wall). A chamfer with distances `dA`
along the plane and `dB` along the wall is the `Cone` through the circle of
radius `R + σ·dA` on the plane and the circle at distance `dB` along the wall.
`σ = +1` when the plane face lies at `ρ > R`, else `−1`.

A fillet that would need `R − r ≤ 0` (the ball cannot sit inside the
cylinder) is SR3.

#### 4.3 R3, a line between a plane and a parallel cylinder

Take the cross-section normal to the common direction `d̂` of the edge and the
cylinder's axis. The plane is a line `L` with outward normal `nA`; the
cylinder is a circle `C` of radius `R` about `o`, with outward normal `nB(x)`
radial (negated for a `Reversed` wall); the edge is their meeting `K`. The
corner is convex when the wall's material lies on the same side of `L` as the
plane's. The angle between the tangent of `C` at `K` and `L` must clear
`minSine`; a tangent meeting is not an edge.

**Fillet.** The ball touches `L` from the side `ball` (`−1` convex, `+1` concave)
along `nA`, so its centre lies on the line `L'` at distance `r` from `L`. It
touches `C` from the outside (`|c − o| = R + r`) when `κ = +1` and from the
inside (`|c − o| = R − r`) when `κ = −1`, where `κ = +1` exactly when the edge is
convex and the material lies outside `C`, or the edge is concave and the
material lies inside `C`. The centre is the point of `L' ∩ circle(o, R + κ·r)`
nearest `K`. The tangent points are `T_A = c − ball·r·nA` and the radial
projection `T_B` of `c` onto `C`. The unit vectors from the centre to them are
`a₀ = −ball·nA` and `a₁ = −κ·(c − o)/|c − o|`, `φ` is the angle between them, and
the blend arc runs between them through `φ`, as in R1. The surface is a
`Cylinder` of radius `r` and axis `d̂` through `c`.

**Chamfer.** The setback is `d` along `L` from `K` into the plane's face and arc
length `d` along `C` from `K` into the wall's face. The blend is the `Plane`
through the two generator lines.

The wall's setback line is the generator of the cylinder at `T_B`. The blend
removes the band of the wall whose angle about the axis, measured from `K` into
the wall, is below the angle of `T_B`, and, at an end where the wall extends
past the edge's corner (§5.0), which lies inside that end's plane `G`
(`internal/retainedblend/cyl_band.go`). Each held facet meets the band in a
convex polygon: the facet is clipped by `G`'s exact tag plane at each such
end, then by the half-plane that holds the axis and the generator. A vertex
is placed on a side of a plane only when it lies farther from the plane than its
bound plus the slack and `τ`; one within that distance is undecided there,
except the edge's corner, the wall's chain along `G`, and, on the arc and chain
routes, the vertices that bound a face whose retained plane tag is `G`'s tag
exactly, which lie on `G`'s plane by construction (below). Every band plane reads each held vertex, and a plane
that places it outside proves it outside the band: the vertex's readings on the
other planes are then never used, so a feature of the wall past an end plane,
such as a cross hole, and the run of cut vertices an earlier blend left along
`G` where the band's base plane or generator puts them outside, do not refuse,
while one between the end planes does. A facet whose three corners lie outside
one plane, or on or outside the generator, misses the band and is emitted
unchanged. The facet that meets the band is clipped plane by plane in a fixed
order, except that a plane that refuses on an undecided corner is deferred
until the other planes have been applied: one of them may drop that corner
from the polygon, and the deferred plane then never reads it. A plane still
refusing after every other plane is applied refuses the cut (SR5): "a vertex of
the wall lies on an end face's plane", or, where the undecided vertex lies on
or inside every other band plane, "an earlier cut on an end face's plane lies
within the blend's reach". On the arc and chain routes an undecided end plane is
read by sign first (below), and only a vertex that stays undecided refuses. No
record of which vertices an earlier blend made is kept: a vertex on `G` that no
other plane puts outside and that no face's tag puts on `G` is not read as lying
on it, and a cut vertex an earlier blend made on `G` is such a vertex.

A facet the band cuts is framed from its own plane to place the cut in two
dimensions (`keepOutside`), and the frame must wind as the facet does. Its
normal is the float cross product of two edges, accepted only when it is longer
than twice `proofbound.FacetAreaTermSlop` of the facet's corners: the float and
exact normals then make an angle under 90 degrees, so the frame's winding agrees
with the facet's exact winding. A shorter normal may be rounding noise of the
wrong sign, as the cross product of a long near-collinear facet is, and the facet refuses (SR5, "a wall facet has no plane"). The check only refuses.

On the arc and chain routes (an exact main plane; R3's angle-main band keeps the
rules above) the end plane's `on` set also holds every vertex of the boundary of
a face whose retained tag is `G`'s exactly (`band_provenance.go`). The tags are
equal when their normals are parallel and one origin lies on the other's plane,
both read on the tags' floats as rationals, and the face states no departure
(`TagDeparture`, `NormalBound` and `AxialDelta` zero, the tag its own record).
This is sound by retention Claim R, which puts every true point of a retained
face's patch on its tag, and by the vertex bound, which puts a boundary vertex's
true point in the closure of every face it bounds: that point lies on `G`, and the
vertex's bound already covers its distance from it. It charges nothing. A vertex
inside a face's fan, and a vertex of a face whose tag is one float step off, is
not admitted. Two consequences follow:

- A cut of a held edge whose two ends both lie on a band plane lies on it too:
  the point lies within its facet bound of the affine segment between the ends'
  true points, which lies on the plane, and the bound the cut carries covers
  that. The cut records both planes (`crossing`). When it is the main plane's cut
  along an end plane it is the band's corner there, and it is not a vertex of the
  wall's row. A polygon edge on a facet's own boundary whose two ends lie on an
  end plane joins that plane's run, as a chord inside a facet does.
- A held edge that receives such a cut vertex is split in every facet that keeps
  it, of any face and whether the cut emitted the facet unchanged or the mesh
  kept it: the facet is fanned from its opposite corner through the vertices on
  the edge (`splitSeams`, applied before the strip's clearance test and in the
  build). The vertex lies on the held edge within its bound, so the facet's
  region and claims do not change. Without the split the facet meets the strip
  across a T-junction.

**Sign reading with a charge.** When every remaining plane of a polygon refuses,
the clip applies the first remaining end plane of an exact-main band once more with
sign reading on (`clipBySign`); the final test of a held vertex's membership
(`inside`) does the same. A point that lies within its bound of the end plane
at an offset `v`, which no face's tag puts on the plane, is then put on the side
of `v`'s sign (on the plane when `v` is zero), and its bound is raised to
`c / s_lo` (a plane wall) or `β + c / s_lo` (a curved wall), rounded up, where
`c = β + |v| + slack` and `s_lo` is a proven lower bound on the sine of the angle
between the wall and `G` (`signRead`, `slantBound`). A held vertex's bound is raised in the work and reported through
`raised`, like the raised inner vertices of a curved wall; a cut point's own bound
is raised. The reading applies to end planes only and only to a band with an exact
main plane, so R3's band, the main plane, the further planes and the guard keep
their refusals. It is on only where reading without it refuses, so no clip that
builds without it changes. A cut of a held edge reads its two ends' sides first, and a
sign reading there raises an end that another plane may already have put outside; that
only widens the end's bound. The cut itself reads the ends' input bounds, never the
raises (below).

The side is a choice, not a claim, and no residual admits an input claim. The held
bound `β` says the vertex's true point lies within `β` of it; that holds as before.
The vertex lies within `|v|` plus the slack (the plane's own arithmetic and
displacement charge) of the plane the exact band is cut at, so its true point lies
within `c = β + |v| + slack` of that plane, on whichever side it is on. That is a
distance to the plane. The band's boundary on the wall is where the wall crosses
the plane, and when the wall meets `G` at a slant of sine `s` it lies `c / s` away
along the wall from the true point, not `c`. The path along the wall from the true
point to the plane stays within `c` of the plane, because its offset falls along it
while the sine stays above zero, so `s_lo(c)` bounds the sine over the whole path.
The published bound is the distance from the *held* point:

- **Plane wall: `c / s_lo`.** Split the held point's error into a part normal to
  the wall, at most `β`, and a part across the boundary line within the wall, at
  most `(a + β·cos θ) / s` for `a = |v| + slack`. Their root-sum-square is at most
  `√(β² + ((a + β·cos θ)/s)²) ≤ (β + a)/s = c/s`, because `2aβ·cos θ ≤ 2aβ`.
- **Curved wall: `β + c / s_lo`.** The path bends, so the triangle inequality is
  used: `β` to the true point and `c / s_lo` from it to the boundary. No curvature
  term is proven. (At sine 0.866 this is about 2.15 β where a plane gives 1.15 β.)

The flat counterexample (a kept vertex whose foot on the true wall lies on the band's
side) and the cylinder cases (foot on the band's side, the point inside and outside
the circle, the axis on either side of `G`, a bore wall) are tests
(`TestSignReadBoundCoversSlant`, `TestSlantBoundCoversCylinderExactly`). The input-bound read and the setback cap
are tested in `cyl_band_sign_internal_test.go` (`TestCutOfHeldEdgeReadsInputBounds`, `TestCutFacetBoundReadsInputBounds`,
`TestCutBoundCoversTrueSetbackLineAfterSignRead`, `TestSetbackBoundFormula`, `TestSetbackBoundCoversSetbackLine`,
`TestSignReadKeepsSlantBoundForHeldVertices`) and `wall_sine_internal_test.go` (`TestWallTilt`,
`TestSetbackBoundCoversKeptCylinder`, `TestRoutesSetTheWallLean`), and `cyl_band_facet_internal_test.go`
(`TestHeldVertexKeepsSlantBoundBesideTheSetbackPlane`, `TestCappedCutPointOnTheSetbackPlane`,
`TestOnlyCutPointsOnTheSetbackPlaneAreCapped`), with `TestCutTakesTheFacetsLargestInputBound` in the sign file.

`s_lo` (`wall_sine.go`) comes from the wall face's retained tag, which must state
no departure (`TagDeparture` and `AxialDelta` zero): Claim R then puts the true
wall on it, and its `NormalBound` lowers the sine by at most itself, since
`sin(θ − δ) ≥ sin θ − δ`.

- A plane wall meets `G` at the length of the cross product of the two unit
  normals, less its rounding.
- A cylinder wall whose axis, normalized, runs parallel to `G` (`|n·a| ≤ 64 ulp`;
  the axial term `|n·a|·(scale + |origin|)` and the rounding of `n·(origin − o)` are charged) meets it, at a point with unit radial direction `r`, at
  `√(1 − (r·n)²)`. A wall point within `reach` of `G` has `|r·n| ≤ (h + reach) / R`,
  `h` the axis's distance from `G`, because `n·(p − o) = n·(c − o) + R·(r·n)` when
  the axial term vanishes. This is `√(R² − (h + reach)²) / R`, the arc route's
  `√(R² − h²) / R` at reach zero (0.866 for a radius-3 wall with its axis 1.5 mm from `G`, as in
  `wall_sine_internal_test.go`). The axis may lie on either side
  of `G`, and a bore wall is the same surface.
- Any other wall, a cylinder with a tilted axis, a cone, a sphere and a wall whose
  tag states a departure, has no `s_lo`, and the reading refuses.

`reach` is a distance from `G`: `s_lo(reach)` holds at wall points within `reach`
of `G`. The reading takes `s_lo(c)`, valid on the path above, and refuses when it
is below `minSine`. What decides the result is
still what the gates check afterwards, and they only reject: the wall test, the
strip and taken-cap clearance and the nesting parity are unchanged, so a band that
overlaps an earlier cut or reaches another part of the wall refuses with their
text, not with an end plane's. A vertex exactly on the plane in floats but on no
face's tag reads as on the plane without being a corner or a run vertex, so a cut
whose corner it should be refuses at the setback corner.

**A cut reads input bounds.** A cut point of a held edge, and each facet's bound in `cutBand`, take
`max` of the corners' *input* bounds (`fp.VertexBound`, `work.inputBeta`), not the work's bounds as
a sign reading has raised them. A cut's claim is where it lies on its facet: the facet lies within
the input bounds of the true wall (vertex bounds §2), the cut lies on the plane `q` within the
rounding, and so the cut is within `max(β_in) + slack + τ` of the true line where the true wall meets
`q` (and the slant term of the crossing, where `q` is a slanted end plane). The raise of an end is
something else: the end's own distance to the band's boundary under the side chosen for it. It
is published on the end, and the output facets that touch the end take it through their corners'
`max`. A cut does not inherit it, because nothing the cut publishes depends on another point's side
choice. If the cut itself lies within its bound of another end plane, it is read by sign on its own
offset, once. Charges therefore never compound: a cut never takes `β + c/s` of an end that an earlier
reading raised and is then charged again. A vertex
the work made keeps the bound it was made with (`inputBeta` has no input record for it).

**The setback line caps a sign reading of a cut point on the setback plane.** A cut point that lies on the setback
plane (the band's main plane) and is read by sign against an end plane `G` publishes the smaller of `β + c/s_lo`
(above) and `β + (|v_m| + λ·β + slack)/√(1 − λ²)` (`setbackBound`). `v_m` is the point's offset from the setback
plane, `λ` is the lean of the wall's generators from that plane's normal (`wallTilt`, from the wall's tag;
`wallBand.leanOK` is false, and the cap off, where the tag gives none), and `β` is the cut's crossing bound.

*The vertex.* The wall above the setback plane is kept at every azimuth, inside `G` and past it, so the setback
line, where the true wall meets the setback plane, lies in the closure of the kept wall and in the true result.
Take the foot `x*` of the point `x` on the true wall: `|x − x*| ≤ β`, and `x − x*` runs along the wall's normal,
which leans from square to the plane's normal by `λ`, so the foot is at most `|v_m| + λβ + slack` from the plane.
A generator of the wall, leaning `λ` from the plane's normal, takes the foot to the line in at most
`(|v_m| + λβ + slack)/√(1 − λ²)`. The bound is a distance from the point to the true result and does not depend on
the slant of `G`. The tags give `λ` only where the wall is a plane or a cylinder that states no departure and no
axial delta: the arc route's wall is coaxial with the rim's plane (`newWallBand`) and the chain route's wall is
normal to its base within `10⁻⁹` (`chainEntry`), so `λ` is the tag's `NormalBound` plus a few ulp.

*The facets.* A vertex bound alone does not carry a facet: a facet publishes the largest bound of its corners, and
every point of it must lie within that of the true result. For a cut point on the setback plane the facets that
keep it are the kept wall above the setback plane when it is read inside `G`, and the strip's facets, whose other
row lies on the base plane and keeps the slant bound. A cut point read outside `G` is dropped from the clipped
polygon and becomes no output vertex. A held vertex is different: the facets that keep it run down the wall below
the setback plane, and their points inside the band lie farther from every true surface than the vertex does from
the setback line (a corner just outside `G` a few hundredths below the setback plane gave 1.09 times its facet
bound at sine 0.31). A held vertex therefore keeps `β + c/s_lo`, and so does a cut point on any other plane, whose
facets are not those. `TestHeldVertexKeepsSlantBoundBesideTheSetbackPlane` samples the facet of a held vertex
against the exact kept wall; `TestCappedCutPointOnTheSetbackPlane` and `TestOnlyCutPointsOnTheSetbackPlaneAreCapped`
pin the restriction.

The cap never lowers a bound below `β` and only replaces a larger one, for a point whose sign reading already
succeeded. It admits no input claim. A lower bound can let a later tolerance-gated call succeed, because the gates
read the published bounds; each such build still passes its own gates.

**The slant of the crossing bound.** A cut point on an end plane `G` publishes
`max(ends, facet) + slack + τ`, a distance to the plane. Where the true wall meets `G`
at sine `s` the true boundary lies about `β / s` away along the wall, so a slanted end
plane needs a slant term; a square one needs none. Each route marks its end planes
from its own gate (`bandEnd.slanted`), never from a number. The arc route's ends are
always slanted: the wall's cylinder is parallel to `G` (`|n·a| ≤ parallelTol`, the arc
reader's gate). A chain end is slanted only where the end is an arc. R3's ends are
normal to the cylinder's axis (`cyl.go`, `|n × a| ≤ 10⁻⁹`) and a chain's straight end
is normal to the chain edge, whose wall's plane contains it, so both meet the wall
square to `s ≥ √(1 − 10⁻¹⁸)`. The excess `b·(1/s − 1) ≤ 10⁻¹⁸·b` lies far inside `τ`
for a point on `G`, and inside the `2·slack` of `cutOffset` for a corner. The planes
of the main, rim, base and guard cross the wall square, because they are normal to
the axis or contain it. R1, R5 and R6 make no cut on a slanted end plane. The rim
routes do: the height cut of a trimming arc end or a chain arc end without `extendsW`
(`clipArcWall`, `clipChainWall`, the chain's seam cut) cuts the wall's boundary generator
along `G`. Both ends of that held edge lie on one Line3 edge of `G`, so their true points
lie on one generator, and the cut keeps the old bound by the straight-line argument of
the same-side rule below.

`cylSlant.reach(b, e)` (`wall_slant.go`) is a proven bound on the distance from a
point within `b` of the true wall and within `e` of `G` to the generator `L` where the
true wall crosses `G` on the point's side. The wall's tag must state no departure and no
axial delta, so Claim R puts the true wall on the tag's cylinder of radius `R`. Work in
the section normal to the axis, where `G` is a line at distance `h` from the axis,
`h ∈ [h⁻, h⁺]` (the tag's `|n·(c − o)|` with the rounding and axial charges `wallSine`
makes). The point `x′ = (X, Y)`, `Y` measured along `G`'s normal from the axis, has
`|Y − h| ≤ e`, so `|Y| ∈ [y⁻, y⁺]` for `y⁺ = h⁺ + e`, `y⁻ = max(0, h⁻ − e)`, and its
distance `ρ` from the axis has `|ρ − R| ≤ b`. Then `X² = ρ² − Y² ∈ [X⁻², X⁺²]` with
`X⁻ = √((R − b)² − y⁺²)` and `X⁺ = √((R + b)² − y⁻²)`. `X⁻ > 0` puts `x′` on one side of
the axis's foot, so the generator it is near is the one at `Z = √(R² − h²)` on that
side, `Z ∈ [Z⁻, Z⁺]` for `Z⁻ = √(R² − h⁺²)` and `Z⁺ = √(R² − h⁻²)`. Then
`|x′ − z′|² = (|X| − Z)² + (Y − h)² ≤ max(X⁺ − Z⁻, Z⁺ − X⁻)² + e²`, and `L` runs along
the axis through `z′`, so the distance from `x` to `L` is `|x′ − z′|`. `L` lies in the
closure of the kept wall. Every operation rounds outward, and `reach` refuses when
`R − b ≤ y⁺`. The end plane's own displacement from its tag (`plane.delta`, which a draft
or brep record states) moves the true generator, so it is charged into `h` (`wallSlant`)
and into `e` (`slantBound`): the bound does not rest on the band's slack covering it.
`wallSine`, and so the H2 reading, has the same omission and relies on that slack
(`planeCharge`, at least `4·delta/minSine`); `wallSine` does not charge it. The
crossing is a generator only up to `|n·a| ≤ parallelTol`, which the `|n·a|·far` term of
`h` charges per section. It is tight: the
worst ratio of the true distance to the published bound reaches 0.98 on the
adversarial cylinder rows (`TestCrossingBoundCoversCylinder`).

What a cut point publishes (`slantBound`), with `b = β + slack` for the held input
bound `β` (the ends' and the facet's largest) and `e = |v| + slack` for the point's offset
`v` from `G`:

- A cut point on a slanted end plane alone, a held edge's crossing: `max(old, reach(b, e))`,
  with `old = β + slack + τ`. A point whose old bound already covers `reach` keeps its
  bytes.
- A corner of `G` and another plane `P′` whose normal runs along `L` (`tilt`, the sine
  of the angle between the normal and the axis, at most `parallelTol`): the true corner
  is where `L` meets `P′`. With `δ` the point's offset from `L` in the section and `α`
  the axial offset, `n·(x − z) = v` gives `|α − t| ≤ (|v| + tilt·|δ|)/√(1 − tilt²)`, so
  the corner publishes `max(old, D_c)` for
  `D_c = reach + (|v| + slack + tilt·reach)/√(1 − tilt²)`. The main, rim and base
  planes qualify. A corner of `G` and a plane that does not qualify, which is the
  guard, or of two end planes, refuses: "the wall meets an end face's plane at a slant the
  tags cannot bound". A slanted end whose wall has no section (a tag that states a
  departure or an axial delta, or a wall that is not a cylinder parallel to `G`)
  refuses the same way.
- The band charge. The corner's `noteCut` delta is `max(facet, D_c − 2·slack)` in place of
  the facet bound, so `cutOffset(v, δ, e) = δ + 2e` covers `D_c` and the strip
  triangles at the corner charge the distance from the corner to its true point, not
  to the square-cut surrogate. The band's slack is the strip's.
- A cut of a held edge along `G` by another plane (the band's corner on a held edge):
  both ends lie on `G` by provenance, so their true points lie on `G ∩ wall`, which is
  one straight generator when both lie on one side of the axis's foot, and the segment
  between them then lies on it. The cut keeps `old` when the ends' coordinates along `G`
  from the axis exceed their bounds with the same sign (`sameSide`) and the cutting plane
  is normal to the axis (`tilt`, so it cuts the generator square); otherwise the ends
  may lie on different generators and the point publishes as a corner.

`max(old, ·)` never lowers a bound, so no claim that read `old` weakens, and every new
gate only rejects. The H2 reading (`signRead`) keeps its `β + c / s_lo` form. Its
input `β` for a cut point is its crossing bound, slant term included; a held vertex's
`β` is the held bound and does not depend on it.

The slant is tested in `cyl_band_slant_internal_test.go` and `apitest`:
`TestCrossingBoundCoversSlant` (a flat wall, edge and corner, sine 1 to 0.2) and
`TestCrossingBoundCoversCylinder` (the arc route's shape, band on either side, `h/R` 0.7 to
0.98, the setback plane off its place by its slack; the worst ratio of the true distance
to the published bound is at most 1 and at least 0.98) read every cut point against the
true walls the held bounds allow; dropping the slant at the held-edge crossing or the
corner fails them. `TestSectionBoundCoversTheCrossing` searches the section densely, and
fails when `e` leaves the hypotenuse or `X⁻` reads `y⁻`. `TestSlantBoundCorners` fails
without the offset's or the tilt's term, `TestCornerBandChargeCarriesSlant` without the
band charge's delta, `TestCornerOnHeldEdgeAlongSlantedPlane` without the same-side check
in either direction or without its margin, and `TestSlantRefusals` without the refusals.
`TestCrossingBoundCoversPlaneDelta` reads a cut point against the generator on a plane
displaced by `delta`, and fails without `delta` in the offset.
`TestSlantSquarePlanesKeepBytes` holds a square plane's bound at its old value.
`TestFacetedRetainedBlendArcSlantedEndBound` does the same through the arc route (a rib
of half-width 2.4 and 2.6 on a round of radius 3, sine 0.60 and 0.50) and fails without
the arc route's slant mark.
`TestFacetedRetainedBlendSlantedEndBound` builds the notched round's back root at sine
0.71, 0.60 and 0.53 and requires the mesh bound to reach the held bound over the sine; it
falls short at 0.60 and 0.53 without the slant. Two first-order partners of the
section's ranges (`X⁺` against `Z⁺`, the outer radius against the inner) cover each
other, so dropping one of them alone is not caught.

The facet keeps the rest, triangulated again with every point
where the band's boundary crosses a held facet edge and every corner of the
band inside it. A crossing point is read from the held edge's two vertices,
so the two facets beside the edge make the same point, and takes the larger of
their bounds and the bound of the facet it is cut from (§6.3) plus the slack
and `τ`; a corner of the band, where the generator meets
`G`'s plane, takes the largest bound of its facet's vertices plus the same.
The crossings on the generator and the band's corners, in order along the
axis, are the strip's last row.

#### 4.4 R4, an open arc

The surfaces are R2's: the same four rims, the same `κ`, torus or cone, with the
rim an arc of the circle from `V₀` to `V₁` and the faces ending on planes `G_i`
that are parallel to the axis. Each `G_i` meets the wall in a generator line and
the plane in a straight line, and the three meet at the end vertex.

The plane's setback curve is the arc of the circle of radius `ρ_A = R + κ·t`
between its points on the two end faces. A point `P_A,i` is where that circle
meets the line `A ∩ G_i`, taken on the side of the corner the chain leaves along
(a quadratic in the distance along the line). The wall's setback curve is the arc
at height `t_B` below the rim, and `P_B,i` is where it meets the generator
`B ∩ G_i`. The rows between them are the circles of the torus (or the straight
generators of the cone) at the tube angle `α_j`; row `j` runs from the azimuth
where its circle meets `G₀` to the one where it meets `G₁`, a solution of
`n_G · x = h_G` in the azimuth. The blend meets `G_i` in the curve those end
points trace, a section of a torus by a plane that is not a conic, so it stays a
chain of exact points. The rows' tube angles start evenly spaced at the
profile's own step and are refined until every chord of that chain in both end
planes lies within `τ` of the curve (§6.3), so they need not stay evenly spaced.
Each row's columns step by what its two gaps leave once each gap's sagitta
and the shift of its rows' ends on the `G`s are spent (`rowSteps`), so a strip
triangle's sagitta and azimuth term together stay within `τ`, a fillet's and a
chamfer's alike (§6.3, Strip triangles).
An open tangent chain whose arc ends lie on such a plane takes the same rows
(retained-modify-reach §5.5 step 5), once the walk decides the arc's end vertex.
The walk reads a held arc whose chain has short steps (retention E7 skips them),
so a pocket mouth whose 3 mm corners' axes lie 2.5 mm and 2.75 mm from the end plane
builds as well (`apitest/faceted_retained_blend_arc_end_plane_test.go`).

**A wall that runs on past an end plane.** A concave arc may end where the
cylinder continues beyond `G_i`, as the end face of a rib standing on a round
meets the round in an arc between the rib's two side faces: the round runs on
past each side face, and, where the rib is not, behind the end face, so the
whole wall face lies on both sides of the rim's plane. Two readings change
(`arc_band.go`):

- *The wall's side.* `readSide` first counts the whole wall face, as for every
  rim. Only when the face lies on both sides does an arc read the side from the
  wall's facets along the rim: each facet that holds a boundary edge of the wall
  face with both ends on the rim circle has a third vertex that lies on the
  wall's side when its height from the plane exceeds its bound plus the slack. A
  vertex that close to the plane decides nothing; at least one facet must
  decide, and every deciding one must agree, else SR5 with the whole-face text.
  A solid's wall cannot lie on both sides of the plane along its own boundary
  edge, so the true piece of such a facet lies on the wall's side near the rim,
  and the convexity check that follows stays the falsifier. The rim records
  `twoSided`; R2 and R6 read the whole face only and refuse it, and a convex
  arc keeps the refusal too. An arc reads the local side when it is the one edge
  selected, or when every selected edge is an open arc, so several arcs may share
  one wall where their ends are cylinders (a selection of several plane-ended arcs on a two-sided
  wall declines, as before); each arc's band is local to its own wedge, and a band that reaches
  another arc's cut refuses (SR5). A selection that mixes an arc with any other
  edge keeps the whole-face refusal. An arc of a tangent chain reads the local
  side too, and only a concave one (`readChainRim`).
- *The wall's band.* A wall that is two-sided, or extends at an end, is cut by
  the band of `cutBand` (`cyl_band.go`, shared with R3) instead
  of by height alone: the part of the wall between the plane `P` and the plane
  `P′` at `t_B` on the wall's side, inside the end plane of each end where the
  wall extends (the held vertices on the rim and on the wall's chain along
  `G_i` lie on their planes by construction), and on the arc's side of the
  guard, the plane through the axis normal to the arc's middle direction, which
  keeps the band off the far half of the cylinder. Each held facet meets the
  band in a convex polygon, and every crossing is made once per held edge and
  plane, so the facets on either side of an edge share the point. A vertex
  within its bound plus the slack and `τ` of a plane it is not on by
  construction, other than the rim's own, refuses (SR5); one strictly inside
  the band other than the rim's and the chain's, or a wall boundary chord that
  meets the band widened by its bound, the slack and `τ`, refuses (SR5): the
  blend may consume a feature of the wall it cannot see. Every wall that is one
  sided and trims at both ends keeps the whole-face height cut, so every case
  admitted before is built from the same vertices, in the same order. An arc
  that spans `π` or more refuses.

At an end where the wall extends the end face keeps its corner `V` and takes the
blend's end cap (§5.0).

**A chain's walls.** The same band cuts the walls of a concave tangent chain
(retained-modify-reach §5.5 step 5; `chain_band.go`): for each chain edge the
prism between `P` and `P′`, inside the end plane of each free end where the wall
extends. A junction has no plane, because the wall face ends at the seam it
shares with the next wall; both walls cut the seam's held edge once. There is no
guard: a wall facet on the far side of the axis inside the prism would hold a
vertex strictly inside the band or a boundary chord meeting it, and refuses
(SR5). The wall's boundary along the chain edge, its seams and its chain along
an extending end plane are exempt; every other chord must miss the band widened
by its bound, the slack and `τ`. The band's corner on an extending end plane is
the wall's last row at that end, and its run along the plane joins the end
face's loop.

The faces are rewritten as in R2 for the plane (a polygon on the setback arc)
and the wall (the cut at the setback height), and as in R1 for the ends.

**An end on a cylinder.** A concave fillet may end on a retained cylinder `G`
parallel to the axis whose material lies outside it, a window's or slot's corner
round, where the wall runs on past `G` as in the previous paragraphs. A round hole's
wall is not built: its chain along `G` runs more than `π`, where the chain's sense is not
read (the end refuses "a setback point lies behind an arc end"), and shorter chains have
not been tested.
This is the only end shape built at a cylinder: `A` trims at `V`, the wall
extends, and `G` takes the end cap. An end that trims the wall (X1) refuses, a
chamfer there refuses, and `G` with its material inside refuses. An arc with a
plane at one end and a cylinder at the other refuses, since no test covers it.
Write `c` for the point where `G`'s axis meets the rim's plane, `r_G` for `G`'s
radius, `d` for the in-plane distance from the rim's axis `O` to `c`, `R` for the
rim's radius and `θ_c` for `c`'s azimuth in the arc's signed angle `θ` (`arc_end_cyl.go`).

- *The root.* The rim's circle meets `G`'s circle at `θ_c ± α`, `cos α =
  (R² + d² − r_G²)/(2·R·d)`. The arc runs on material, outside `G`'s disk, so in
  the direction of growing `θ` it leaves the disk at `θ_c + α` and enters it at
  `θ_c − α`. End 0 is the root `θ_c + α` and end 1 the root `θ_c − α`, and `G`
  must be `Reversed`. The other root `V*` is the one the end is not at. The held
  vertex must lie within its bound, the slack and `e_V` of the root, else SR6.
  The distance to the held vertex never picks the root.
- *The charged crossing sine.* With `cos V = (R² + r_G² − d²)/(2·R·r_G)` and
  `err_c` the rounding of that quotient, `s_V = √(1 − (|cos V| + err_c)²) −
  (ε_G + ε_G³) − NB_G − NB_B`, `ε_G = |axis_G × n|` the tilt of `G` and `NB` the
  faces' normal bounds. `s_V > minSine` proves that the two circles cross in two
  distinct points (boss-root-overflow-design §5.3); at or below it the end
  refuses as a grazing end. `e_V = 64·ulp·S_far·max(1, S_far/r_G)/s_V +
  8·ulp·S_far` bounds the rounding of `V` and `V*`, `S_far` the sum of the body's
  largest coordinate and the lengths the end reads: `acos` amplifies the
  rounding of its argument by `1/sin α`, and `sin α ≥ r_G·s_V/d`.
  `TestCylEndRootBoundCoversRounding` measures `V` and `V*` against the exact roots in 200-bit
  arithmetic for `s_V` from 0.72 down to twice `minSine` and requires each under `e_V/4`.
- *The rows' ends.* A row of radius `ρ` ends at `θ_c + σ·α(ρ)`, `σ = +1` at end 0
  and `−1` at end 1, by the same side rule: the meeting of two circles in plan,
  with no choice by continuity. A row that does not cross `G`'s circle refuses.
- *The wall's band.* The band's end plane at a cylinder end is the radial plane
  `Π` through the axis and `V`, not `G`. On the true wall the band's boundary is
  the generator through `V`, which lies in `Π`, so `Π` cuts the same set on the
  true wall as `G` would, and it meets the wall square. The wall's chain along `G`
  and `V` are the `on` set of `Π`, and a chain vertex joins it only when (a) it is a
  vertex of a facet of the wall and of a facet of `G`, so its true point lies on
  the wall's meeting with `G`, the generators through `V` and `V*`, and (b) its
  distance from `V*`'s generator exceeds its bound plus the slack plus `e_V`, so
  the true point is not on the far generator and lies on `V`'s, in `Π` up to `Π`'s
  own rounding. The band's slack grows by `e_V` for that rounding (when `e_V > 0`, so a plane
  end's band is unchanged). A chain vertex that fails either check refuses SR5 ("the wall's chain
  along a cylinder end face does not lie on the end's generator"). Besides the chain, the
  held mesh may hold, beside `V`, boundary vertices of the wall that are the arc's own, `n₁, n₂, …`
  from `V` outward (a tessellation point of the boss's base circle that falls beside
  `V`). Such a vertex lying within `β + slack + τ` of `Π` has no decided side, and a side read
  from its held offset leaves the cut along `Π` short of `V`. It is read as on `Π`: a choice, as
  `signRead` makes (§4.3), charged by the same distance, the one from the held point to the band's
  boundary on the wall, by the `slantBound` charge (`β + c/s` on a curved wall). The cut along
  `Π` must then end at one of them, `n_j`, and `V, n₁ … n_j` are joined by the held edges of
  the wall's boundary (`linkNoise`); a held edge among them that a facet recorded as part of the
  cut but that the cut does not use is dropped. If the cut reaches `V` itself none is used.
- *The plane `A`.* `A`'s chain along `A ∩ G` is walked from `V` only across loop
  edges that are also edges of a `G` facet, and only while the vertex lies within
  its bound plus the slack of `G`'s circle in `A`'s plane, so it stops at the end
  of `G` and never runs on to the next face. The setback point `P_A` is where the
  row of radius `ρ_A` meets `G`; its position along the chain is the arc length
  `r_G·(ψ_c(p) − ψ_c(V))` about `G`'s axis, signed by the chain's last vertex, and
  a held vertex's uncertainty along it is `r_G·asin(min(1, β_v/r_G))`. `A`'s lost
  region has the arc of `G`'s circle from `V` to `P_A` for each side, area
  `½(c × (p₁ − p₀) + r_G²·Δψ)` in Green's theorem.
- *The cap.* `G` keeps its facets and gains the cap, the region between `A ∩ G`
  and the end curve, in `G`'s unrolled coordinates `x = σ_i·r_G·(ψ_c − ψ_c(V))`
  along `G`'s circle and `z` above `A`. The bottom chain is `V`, `A`'s kept chain
  to `P_A`, with a bottom point `b_j` on `G`'s circle in `A`'s plane under each
  row end, unless a held chain vertex lies within a quarter of the row gap's `x`
  of it. The top chain is `V`, the wall's run up to the band's corner, the corner,
  the row ends from the last to the first, and `P_A`. Each held facet of `G` that
  keeps a chain edge `(u, w)` receiving new points is split: removed, and its fan
  `(u, p₁, x), …, (p_k, w, x)` emitted from the opposite corner. A new point `p`
  takes the bound `δ(t) + distToSegment(p, u, w)` of the facet `t` it splits, and
  the feet must lie strictly inside `(u, w)` in increasing order, else SR5 ("a cap
  column falls on a vertex of the end face's top edge"). The two chains are zipped
  by `x`: the first triangle is `(V, b₁, t₁)`, the last `(b_{K−1}, P_A, t_{L−1})`,
  and between them the chain whose next vertex has the smaller `x` advances (the
  other on a tie or when the triangle is not positively oriented beyond its
  rounding); no advance leaves SR5 ("the cap on a cylinder end face folds"). Every triangle is
  positively oriented in `(x, z)`, so the triangles of a two-chain strip whose
  boundary is the simple cap polygon cover each of its points once (winding
  number) and nothing outside it. `G`'s turn at `V` is read from the wall's
  generator direction and `A`'s chain tangent, as an R3 extending end reads it, and
  must be convex, else the R4 end's refusals. The cap is simple (`capSimple` on the
  unrolled polygon) or the rows are spaced again (`cuspParam`, with the `x` distance
  read as `r_G·|ψ(s) − ψ(1)|`). The polygon closes at `n_j`, not `V`, where `V`'s neighbours were read as
  on `Π`: the vertices `n₁ … n_j` are not part of the cap's loop. The zip then starts at `n_j`
  and the closing triangles `(n_{i+1}, n_i, b₁)`, `n₀ = V`, join `V, n₁ … n_j` to the bottom
  chain's first vertex `b₁`, wound against the held facets that keep the path `V → n₁ → …` so the
  mesh closes (the cap must run it the other way, else SR5).

**A tangent end (C3).** Where `G` touches the wall at the arc end instead of
crossing it, the end is a corner of four faces: the plane, the wall, `G` below
the plane and a full-height side face `K` tangent to both. The route reads it
only with a proof from the records the faces came from (`TangentEnds`, which the
stacked view's fillet passes), builds on the ideal circle `G*` tangent to the
wall, cuts the wall by height to its seam with `K`, and runs the cap's top chain
up that seam (boss-root-overflow-design §5.5). Without a proof the four-face
corner refuses ("an edge end is not a corner of exactly three faces").

The arc's end values are computed decad-side, not by a `sketch` scene (the ask
sketch rule in `CLAUDE.md`). On a mesh result `V`, `V*`, the bottom points, the
crossing sine and the rows' ends come from retained tags, not from records a
sketch scene can read, and each row is a circle of the torus at a tube angle the
blend picks, not a recorded entity. The plane ends of R4 are computed decad-side
for the same reason.

A plane end keeps the root nearest its held vertex. The other root must lie
farther from the held vertex than the vertex's bound plus the slack plus
`64·ulp·S_far/q_end`, `q_end = √(R² − h²)/R` the sine at which the plane crosses
the rim's circle, else the end refuses ("an arc end lies between two meetings of
its tags"); this makes the distance choice the only possible one. The grazing
gate of a plane end `h + 64·ulp·S_far ≤ R·√(1 − minSine²)` charges the rounding
of `h`.

#### 4.5 R6, a plane-sphere rim

The edge is the closed `Circle3` of retention E5 between a retained `Plane`
(outward normal `n`, origin `o`) and a retained `Sphere` (centre `S`, radius
`R_s`). It is the rim a placed cut leaves on a revolved ball or dome.

**Exact record.** A `Sphere` face with a denoted patch (every Revolve sphere)
is read only where its tag is exactly the sphere its record states. Each test
compares recorded rationals by exact equality, and nothing is admitted on a
residual:

1. every leaf of the circular patch is a point and the axis frame is exactly
   orthonormal (`stitchflux.ExactRevolveOf`);
2. the recorded centre lies on the axis (`ρ = 0`), and the tag's centre is the
   axis point at the recorded `z` exactly;
3. the recorded radius is a point and equals the tag's radius exactly.

`stitchflux.TagIsDenoted` reads the first two and no radius, so the third is
this route's own. Each failure refuses SR1 with a message naming the leaf. A
rotated placement fails the first, a half-disk centre within the snap
tolerance of the axis fails the second, and a meridian whose recorded radius is
no float (an arc to `2.1` about `0.1`) fails the third. Retention §10.1 E12
applies the same three tests before it publishes the rim, so on a body built
by those three the rim already reads `FacetedCurve`, and `Unread` refuses it
SR1 as a tag not proven to be exactly its revolve record. A `Sphere` face with
no denoted patch is its own record, as for every other kind (§2).

**Rim.** Let `zs = n·(S − o)`. The rim's centre is `c = S − zs·n` and its radius
`ρ0 = √(R_s² − zs²)`. `ρ0 / R_s` is the sine of the angle at which the plane
crosses the sphere. A plane that misses or touches the sphere, or crosses it
below `minSine` (`10⁻³`), refuses SR5: a nearly tangent rim is no edge the
section can resolve. The published circle must be `(c, ρ0)` within slack (SR6).
The wall's side, the convexity and the side of the rim the plane face lies on
read as R2's: the held wall lies on one side of the plane, the edge's `Convex`
flag must agree, and the plane face lies outside the rim exactly when the rim's
convexity equals "the sphere's material is outside it" (`Reversed`).

**Section.** Take the meridian half-plane `(ρ, z)` about the axis through `S`
along `n`, with the plane at `z = 0`, the rim's point `K = (ρ0, 0)` and the
sphere the circle of radius `R_s` about `(0, zs)`. This is R3's line-circle
cross-section (§4.3): the plane's face runs from `K` along `±ρ`, toward the
plane face, and the wall along the circle's tangent at `K` toward the wall's
side. The same solve gives the fillet ball's centre `(ρc, zc)`, its tangent
points `T_A = (ρA, 0)` and `T_B = (ρB, zB)`, and the arc's rows. A chamfer's
setbacks are `dA` along the plane and the arc length `dB` along the meridian.
Revolving the section about the axis gives the blend: the `Torus` with
`Center = c + zc·n`, `Axis = n`, `Major = ρc` and `Minor = r` (`Reversed` for
a concave rim), or the `Cone` through the circles `(ρA, 0)` and `(ρB, zB)`,
whose `Origin` is the narrower circle.

**Size (SR3).** The rim refuses when no ball of radius `r` touches both faces,
when the plane's setback circle reaches the axis (`ρA ≤ τ + slack`), when
the sphere's setback runs past the pole (`ρB ≤ τ + slack`), and when the
tangent point or the arc is degenerate (R3's tests).

**Construction.** As R2 (§5). The plane's rim loop gives way to the polygon on
the circle `ρA`, and the wall is cut by the plane `z = zB`, which meets the
sphere in exactly the setback circle. A sphere's held facets carry vertices
inside the face. Such a vertex is the tessellation of a doubly curved surface,
not a feature of the face, so only the wall's boundary loops must clear the
band (§6.2). The cut removes an inner vertex with its facets or keeps it, by the
side of the cut plane its held position lies on. A kept inner vertex `v` whose
held height clears the cut plane by at most `β(v) + slack` may have its true
point in the removed zone, which is no longer boundary. Its bound becomes the
larger of `β(v)` and its distance to the blend face, plus slack: that distance
is the meridian distance from `v`'s image to the torus arc or the cone's
segment, and a point of the blend face is a true boundary point. The largest
such bound joins `ε` of §6.3. Where the band is thin against the wall's held
vertex bounds, the clipped facets can fold; the rebuilt mesh then fails its
audit (SR7).

**Exact areas.** The plane loses the annulus `π·|ρA² − ρ0²|` and the sphere the
zone `2π·R_s·|zB|` (Archimedes). The torus patch has area
`2π·r·(ρc·φ + r·∫a_ρ dα)` by Pappus, with `a(α)` the unit vector from the ball's
centre along the arc and `a_ρ` its radial part. The cone's frustum has
`π·(ρA + ρB)·|T_A T_B|`.

#### 4.7 A mitre run of R1 fillets

Straight R1 fillet edges of one radius that meet end to end on one base face `A`, at corners where `A` and the two
walls are mutually perpendicular, are blended as one run (`internal/retainedblend/line_mitre.go`). The edges form one
open chain or one closed loop, no more than two at a corner, every edge convex, and the walls are distinct.

At a joint `C` between the edges with walls `B_k` and `B_{k+1}`, the two cylinders meet in the plane through `C` with
normal `n(B_{k+1}) − n(B_k)`, which bisects the walls. Each edge's rows end on that plane (`setbackPoint`), so the two
strips share the vertices of their rows there and the strips' end triangles coincide. No end face is trimmed at a
joint. The mitre plane meets each edge at 45°, so every edge of the run takes its row count from the smallest
`|d̂ · n|` over all the run's mitre and end planes, as §5.0 step 1 does for one edge's two end planes: the chords in the
mitre plane stay within `τ`, and both strips at a joint have the same rows. The vertical edge under `C` loses its top
part, and the two walls meet at the strips' feet.

The base face's boundary gives way to a polygon of the rows' base points at the joints and at any free end. A closed
run replaces the whole outer loop; an open run replaces the stretch it covers. A free end trims its end face as a
single edge's does (§5), and an end face is trimmed by one end only. Each strip is its own blend group. The removed
and added areas are the sums of the single-edge terms, each with its mitred ends' end planes replaced by the mitre
plane, and the volume change is bounded from those sums.

Refusals (SR2): three edges at a corner, edges not on one base face, a corner whose faces are not mutually
perpendicular, both free ends of an open run on one face, and a boundary of the base face or of a wall within the
strips' reach.

#### 4.8 Corners of three perpendicular faces

Straight R1 fillet edges of one radius whose ends lie at corners of three mutually perpendicular retained planes are
blended together (`internal/retainedblend/corner_fillet.go`), however many edges the selection holds at each corner.
Every edge is convex and at least `2r` long.

Each corner is read from its three faces, with outward normals `n_0`, `n_1`, `n_2`; the corner's edge `k` runs along
`u_k = −n_k`. The strip of an edge ends on a plane that depends on how many selected edges reach the corner:

| Selected at the corner | End plane of each strip | Face `k` (which holds edges `i` and `j`) gets, in place of the corner |
|---|---|---|
| one, edge `k` | the plane of the end face, as a single edge's | the end row of edge `k`, an arc of centre `C + r·u_i + r·u_j` |
| two, `a` and `b` | the plane through the corner with normal `n_a − n_b` (§4.7) | `C + r·u_i + r·u_j` when `i` and `j` are both selected, else the point `r` along the unselected edge |
| three | the plane square to the edge `r` from the corner | `C + r·u_i + r·u_j` |

At a corner with three selected edges a sphere of radius `r` and centre `C + r·(u_0 + u_1 + u_2)` fills the gap. Its
patch is the octant between the three strips' end rows, gridded so that its sides are those rows' vertices. The
exact areas are those of the strips cut at their end planes, plus `3r²` removed from the three faces and `πr²/2` added
by the patch for every such corner. The chord tolerance is an eighth of a single edge's, so the many chord deficits stay
inside what `Verify` accepts for a box with all twelve edges rounded. A patch triangle lies farther from the sphere than
the rows' chords along its sides, so the rows are added until the patch's own grid stays within that tolerance
(§6.3, Strip triangles). Every strip takes the same row count, from the
smallest `|d̂ · n|` over all the strips' end planes (§5.0 step 1), so the chords on a two-edge corner's 45° plane stay
within the tolerance.

Refusals (SR2): a selected edge no longer than `2r`, a face edge at a corner no longer than `r`, a corner whose faces
are not mutually perpendicular retained planes, and a rebuilt face whose outline is not a simple polygon.

#### 4.9 R7, the saddle of a cross hole (Chamfer)

R7 is the closed mouth a perpendicular bore leaves in a turned shaft. It is
built by `internal/retainedblend/saddle.go` (recognition) and `saddle_tool.go`
(tool and cut), not by §5's rewrite: the chamfer face is a ruled band between
two space curves, which is no `Plane`, `Cone` or `Cylinder`, so the result holds
it as an untagged `Faceted` face whose chords carry a proven bound, as
Part X's band does. A coaxial cone
does not give equal setbacks: its setback along the bore wall varies by
`R − √(R² − r²)` around the mouth (0.46 mm for R = 10, r = 3).

**Recognition (tags only).** `looksLikeSaddle` admits a closed edge
(`Start == End`) between two faces with `RetainedTag`, `TagDeparture == 0` and
`tagIsRecord`, both `Cylinder` and of different radii: the larger is the shaft
(radius `R`), the smaller the bore (radius `r`). The edge's `Convex` flag must
be set and the ball's sides must be σ = (−1, +1) (§4.10), so the shaft is not
`Reversed` (material inside) and the bore is (material outside). Each axis is a unit
coordinate axis (one component ±1, two exactly 0), the two on different
coordinates, and the origins agree exactly on the third coordinate, so the axes
meet at right angles at a point `T` read from the tags. `2r² < R²` holds in
rationals. Any failure leaves the edge to `Unread`'s canal-surface reason (SR1).

**Pose.** A signed permutation and the translation `T` map the pair exactly
onto cross-cylinder-bore §"Source surface and band"'s pose with the bore axis
at `z = 0`: shaft axis local Z, bore axis local X, the third coordinate local
Y. The held seam loop is the boundary loop of the bore's facets that holds the
edge's start vertex. Every vertex of it must lie on the shaft's facets, within
its bound plus `64·ulp·scale` of both cylinders, and on one side of the shaft's
axis; that side becomes local `+X` (X and Y both flip for the far mouth). This
check only refuses.

**Size.** One setback `d` on both walls, `0 < d ≤ r/2`, converted exactly; an
asymmetric chamfer refuses. Several selected saddles are cut in one Boolean.

**Tool.** At each of `N = 1024` stations `θ_j = 2πj/N` the feet `O` (shaft)
and `B` (bore) are cross-cylinder-bore's, each `d` from the seam point `P` along
its unrolled cylinder. The tool section holds four vertices:

| Vertex | Local position |
|---|---|
| `O⁺` | `O + λ(O − B)`, `λ = 1/4`: the band run past the shaft wall |
| `A` | `O⁺` moved radially from the shaft's axis to `R + r`, same height |
| `K` | `(√(R² − r²cos²θ) + r, (r/2)cos θ, (r/2)sin θ)`: in the bore's extension outside the shaft |
| `B⁺` | `B − λ(O − B)`: the band run into the bore |

The band `B⁺O⁺` is split into `M = 8` rows across its length: each station
also holds the points `B⁺ + t_i·(O⁺ − B⁺)` at `t_i = i/M`, computed in float
from the held `O⁺` and `B⁺`. `M` is a power of two, so every `t_i` is a
dyadic, and no `t_i` is 1/6 or 5/6, where the exact band meets the bore and
the shaft (`λ = 1/4`). Quads join consecutive stations over the sides `O⁺A`,
`AK`, `KB⁺` and each row of the band, each quad split along the same
diagonal; the ring is wound outward by its signed volume. Its band is group
`chamfer(k)`, every other side one shared group.

**Proofs (each failure refuses `ErrUnsupported`).**

1. Band cells: `crossbore.ProveToolBases` runs the same eight interval sign checks
   as `CellExactProof` on the `N/4 + 1`
   quarter-turn cells widened by `2π/N` (the outer foot outside the bore, the
   bore foot inside the shaft, polar progress, no fold), and
   `crossbore.ProveToolBandWith` mirrors them over the whole turn as
   cross-cylinder-bore does; the local pose is exactly symmetric by
   construction. The base cells read only `R`, `r` and the setback `d`, never
   the pose, so within one chamfer call consecutive saddles of bit-equal `R`
   and `r` (the two mouths of one through bore) read one proof of them.
2. Chord bound `ε`: a point of a held band triangle pairs with the exact
   extended band `F(θ, t) = B⁺(θ) + t·D(θ)`, `D = O⁺ − B⁺`, at the same
   parameters. Take the quad between stations `θ_j`, `θ_j+1` (`h = 2π/N`) and
   rows `t_i`, `t_i+1` (`k = 1/M`), write `P(θ) = F(θ, t_i)` and
   `Q(θ) = F(θ, t_i+1)`, `u` the fraction of `h` and `s` of `k`, so that
   `F = (1 − s)·P + s·Q` there. On the triangle `(P_j, Q_j, Q_j+1)`, where
   `u ≤ s`, the held point with those exact corners is
   `(1 − s)·P_j + (s − u)·Q_j + u·Q_j+1`, and
   `F − x = (1 − s)·g_P + s·g_Q + (1 − s)·u·(ΔP − ΔQ)`, with `ΔP = P_j+1 − P_j`
   and `g_P(u) = P(θ_j + u·h) − P_j − u·ΔP` the chord's departure.
   Per coordinate `g_P = h·u(1 − u)·(m₁ − m₂)`, `m₁` and `m₂` the means of
   `P′` over the two parts of the cell, so `|g_P| ≤ h·u(1 − u)·W`, `W` the
   larger width of the enclosures of `O⁺′` and `B⁺′` over the cell (`P′` lies
   in `(1 − t_i)·[B⁺′] + t_i·[O⁺′]`, no wider). `ΔP − ΔQ = −k·h` times the
   mean of `D′ = (1 + 2λ)·(O′ − B′)`, at most `k·h·S` with `S` the largest
   `|D′|` over the cell. Since `(1 − s)·u ≤ (1 − u)·u`, each coordinate of
   `F − x` is at most `h·u(1 − u)·(W + S/M) ≤ (h/4)·(W + S/M)`. The other
   triangle, where `s ≤ u`, gives `s·(1 − u)·(ΔP − ΔQ)` in place of the last
   term, with the same bound, and so does either diagonal, so the bound holds
   on the mirror image of a quad. The enclosures come from cells over exactly
   `[θ_k, θ_k+1]` for the quarter turn's quads; every other quad is the mirror
   image of one of them, which keeps each coordinate's magnitude. `W` shrinks
   with `h` and `S/M` with the rows. Each held band point, on every row, is compared
   with a tight enclosure of `(1 − t_i)·B⁺ + t_i·O⁺` at its own exact angle,
   mapped to the world by the exact pose; the largest gap joins `ε`.
3. Band area: `|F_θ × F_t| = |w(t)|`, `w = (1 − t)·u + t·v` with
   `u = B⁺′ × D` and `v = O⁺′ × D`, is convex in `t`, so over a row
   `[a, b]` its integral lies between `(b − a)·|w((a + b)/2)|` (Jensen) and
   `(b − a)·(|w(a)| + |w(b)|)/2` (the chord above a convex function). Each
   quad of the quarter turn splits into 8 cells along θ, and each cell's
   enclosures `U` and `V` of `u` and `v` give `(1 − t)·U + t·V ∋ w(t)` at
   every θ of the cell; four quarters give each row's exact area interval. An
   interval enclosure over a cell is wider than the range it holds by a part
   proportional to the cell's width, so the split narrows the band's interval.
4. Outside: on the held vertices in exact rationals, every `O⁺A` and `AK` quad
   lies in one half-space `n·p ≥ (R + μ)|n|`, `n` horizontal, and every `K`
   and `B⁺` lies inside the bore shrunk to `r − μ'`, so the `KB⁺` quads do
   (the disc is convex). `μ` and `μ'` are the wall's largest held vertex bound
   plus `ε` plus slack.
5. Clearance: no facet of another face, and no segment of another boundary loop
   of either wall, grown by its held bounds plus slack, meets the tool's box.
   Inside the box the solid is then the region inside the shaft and outside the
   bore. It reads only the placed tool's box, so it runs first, before the band
   proof: a mouth that fails it and another proof refuses with its text.
6. Each ring passes the exact crossing audit (`loftmesh.LoftCrossingAuditBounded`)
   and the rings' boxes are apart.
7. The Cut passes the mesh Boolean's own audits; a shifted retry refuses.

**Claim S.** Let `S` be the true solid and `W*` its exact chamfer wedge, the
part of `S` on the seam's side of the exact band. Let `T*` be the held tool
with its band replaced by the exact extended band. By proofs 4 and 5, `T*` meets
`S` only in `W*`: its sides lie outside `S`, the exact band runs past `O` outside
the shaft (`B` is strictly inside it) and past `B` inside the bore (`O` is
outside the bore and `B⁺` inside it). So `S − T* = S − W*`, the chamfered solid.
The held tool lies within `ε` of `T*`. The tool states vertex bound `ε` on `O⁺`
and `B⁺` and 0 on `A` and `K`, `VolSymDiff = ε·(A_held + A_exact)` over the
band (Claim K's argument), and `AreaSlack` the sum over the rows of the gap
between each row's held area and its exact interval, plus a seal `2ε` times the
length of both band ends. The sum is at least the whole band's gap and bounds
the gap of every union of whole rows. The Boolean composes the result's bounds
from these and the body's own.

**Chain allowance.** The Cut meets only the two walls and the bands, whose
facets carry the walls' held bounds and `ε`. The chain-depth gate
(`agent-docs/faceted-vertex-bounds-design.md` §5) would refuse them past the
pair's chord tolerance; it only compares two proven numbers, so the cut runs
with the chain tolerance raised to the largest of those bounds, as §5's rewrite
publishes the walls' bounds on its cut vertices.

**Result.** The walls keep their `Cylinder` tags; each band is a `Faceted` face
with role `chamfer(k)`. On the R = 10, r = 3 shaft `Tessellate(…, VerifyAll)`
verifies the two-mouth result's boundary and volume. Each face's area term is its largest facet bound times its perimeter,
and the perimeter carries `proofbound.ChainLengthBound`'s `2δ` for each of the
rims' segments. The largest facet bound is a rim vertex's,
`(ε + β_wall)/sin θ` (faceted-vertex-bounds §3.2). The body's area bound sums
the face terms, the area bound carried from the Boolean, and the two tools' row gaps and seals.
`Verify` reads the d = 1.5 and d = 0.5 results Sound (`TestRetainedSaddleChamfer`). At r = 7, d = 0.5 the band
rims' length bounds keep the area bound past `Verify`'s relative tolerance, so that case does not claim Sound.

**Reach.** Built and checked: r/R = 0.3 (d = 0.5, 1.5) and 0.7 (d = 0.5) on a
Revolve shaft, and r/R = 0.3 (d = 1.5) on an Extrude shaft under each
assignment of the two axes to world coordinates and under a translation. Recognition stops at `1/√2`, where the
opening stops being one graph over the shaft's angle: a bore of radius 7.5 in the R = 10 shaft refuses SR1.

#### 4.10 R7 fillet, the rolling ball in a saddle

R7's saddle takes a `Fillet` where the ball lies outside the bore and either
inside the larger cylinder, the convex mouth a cross hole leaves in a shaft, or
outside it: the convex seam where two bores cross in a block, and the concave
root of a boss or pin on a rod. It is built by
`internal/retainedblend/saddle_fillet.go` (the rewrite), `internal/saddlecanal`
(the exact fillet and its cell proofs) and `saddle.go` (recognition), as a
rewrite of the held mesh near the mouth in the manner of §5. A Cut by a tool
would leave a crease where the true fillet meets each wall tangentially (§12).

**Recognition (tags only).** `saddleTags` reads the edge as §4.9 does, and
takes the larger cylinder (radius `R`) and the bore (radius `r`) by their radii.
Each wall gets the side σ of the rolling ball: −1 inside its cylinder, +1
outside. The ball lies in the material at a convex edge and in the void at a
concave one, and a face's material lies inside its cylinder unless the face is
`Reversed`. `saddleKey.filletBuilt` admits three patterns:

| σ | Edge | Faces | Example | The fillet |
|---|---|---|---|---|
| (−1, +1) | convex | the larger not `Reversed`, the bore `Reversed` | a cross hole's mouth in a shaft | removes material |
| (+1, +1) | convex | both `Reversed` | two bores crossing in a block | removes material |
| (+1, +1) | concave | neither `Reversed` | a boss on a rod, a pin joined through a shaft | adds material |

σ alone does not identify the first row: a concave pin root inside a tube's
bore has both `Reversed` flags and the `Convex` flag flipped from a mouth's, so
it reads σ = (−1, +1), and a blend there adds material. Its pipe is the
mouth's, but no fixture checks its walls, bands and winding, so it stays SR1
(B8), as does every pattern with the ball inside the bore. §4.9's chamfer reads only the first row
(`saddleKey.built`); a unit test checks that its rule admits exactly the edges
it admitted before σ existed, and another checks the fillet's three rows over
every pair of radii, `Reversed` flags and `Convex` flag. The pose and the seam
are read as in §4.9.

**Size (SR3).** With `R' = R + σ1·ρ` and `r' = r + ρ`, the ball's centre lies
on both offset cylinders, and they meet in one closed loop per mouth exactly
when `r' < R'`, decided in rationals. Inside the larger cylinder that is
`2ρ < R − r`; otherwise the call refuses `ErrDegenerate` "no ball of that radius
touches both walls of the saddle". For R = 10 and r = 3 the bound is ρ < 3.5.
Outside it `r' < R'` holds for every ball, since `r < R`; the reach ends at the
cell proofs, the window and the clearance.

**Fillet.** In the local pose (larger cylinder `x² + y² = R²` about z, bore
`y² + z² = r²` about x, mouth at x > 0) the centre curve is
`c(θ) = (√(R'² − r'²cos²θ), r'·cos θ, r'·sin θ)`. The ball touches the larger
wall along `a1 = −σ1·(c_x, c_y, 0)/R'` and the bore along
`a2 = −(0, cos θ, sin θ)`. Both are normal to `c′`, and the arc between them has
angle φ with `cos φ = σ1·r'·cos²θ / R'`. Inside the larger cylinder φ is
widest at θ = 0, `arccos(−r'/R')`; outside it at θ = ±π/2, π/2. The tangency
curves are `T1 = c + ρ·a1` on the larger wall, the curve
`(R'·y/|p_xy|)² + z² = r'²`, and `T2 = c + ρ·a2` on the bore, the curve
`x² + r'²y²/|p_yz|² = R'²`, which crosses each generator of the bore once at
the bore angle θ. T2 lies nearer the larger cylinder's axis than the seam
inside it and farther outside it. The fillet is the pipe
`F(θ, t) = c(θ) + ρ·w/|w|`. The direction `w` blends `a1` into the bisector
`b = (a1 + a2)/|a1 + a2|` over `t ∈ [0, 1/2]` and `b` into `a2` over
`[1/2, 1]`, each half as `(1 − τ)·from + τ·to`. Splitting at the bisector keeps
the parameter's angular speed near uniform. A linear blend over an angle α runs
`1/cos²(α/2)` times faster at its midpoint than at its ends, and each half spans
half the angle the single blend from `a1` to `a2` does.

**Stations and rows.** The strip has `N = 256` stations `θ_k = 2πk/N` and the
rows `t_j` that split each half of the widest arc φ* into equal angles no
larger than `2·arccos(1 − 2⁻¹⁰)`. The sagitta over that angle is `τ = 2⁻¹⁰·ρ`.
The rows are shared by every station: at R = 10, r = 3, ρ = 0.5 inside the
shaft there are 24 rows and the strip holds 12,288 facets per mouth; outside
the larger cylinder φ* = π/2 gives 18 rows.

**Walls.** The larger wall is cut out of a window `W`, a 64-gon in the (y, z)
projection. Its corners lie on `(R'/R)²y² + z² = s²` with `s·cos(π/64) =
r'·(1 + 1/32)`, so `W` holds T1 with a margin of `r'/32`. T1's ellipse holds the
seam's circle `y² + z² = r²` on either side: its semi-axes are `r' > r` and
`r'·R/R' > r`. Each side of `W` is a plane parallel to the bore's axis, so it
cuts every flat held facet of the larger wall along a straight segment: the clip
is exact. The part of a facet outside `W` runs along the facet's boundary
outside `W` and, from each point where that boundary enters `W`, clockwise along
`W`'s boundary through the corners inside the facet to the next crossing, which
must be an exit. Ear clipping in the projection triangulates each part. The cut
ring is the chain of those arcs around `W`, not a sort by angle. Two crossings
within `64·ulp·scale` of each other, across a held sliver, are one vertex. The
bore is cut by a plane `x = X_w` and keeps the part away from the seam: inside
the larger cylinder `X_w = √(R'² − r'²) − r'/32`, below T2's least x, keeping
`x < X_w`; outside it `X_w = R' + r'/32`, past T2's largest x, keeping
`x > X_w`. A held vertex within 10⁻⁶ mm of a cut, or a window corner as near a
held edge, turns the window (nine offsets of its corners) or moves the plane
(four places), else SR5. A cut along a curve of T1's own family would miss
facets: the larger wall's held facets are strips from the seam to the wall's
ends (on the R = 10 Revolve they run the shaft's 80 mm length), T1 runs almost along the
generators at its two ends, and such a curve crosses a strip between the
strip's vertices.

**Bands.** On each wall a zipper of triangles joins the held cut ring to the
ring of exact tangency points at the stations: on the larger wall both rings
are ordered by the angle of `((R'/R)·y, z)`, on the bore by the angle about its
axis. T1 sits at angle θ_k in the first order and T2 in the second. Each band
facet is wound toward the solid's outward side of its wall, away from the
wall's axis when the face is not `Reversed` and toward it when it is, and then
tested exactly, in rationals on the held floats (`bandFacing`). On the larger
wall its normal's component along the bore's axis (+x) must have the outward
side's sign, + for a face that is not `Reversed`, so its (y, z) projection keeps
one orientation. On the bore its normal's component along `(0, y_i, z_i)` at
each vertex must have that sign, so its radial projection onto the bore keeps
one orientation over the facet's angular span. With both rings running once
around, either test embeds the band. A zero-area facet fails the test and
refuses. A band facet lies within `d + R·Δ²/8` of its wall, `d` the largest
distance of its vertices from the wall (a cut vertex's bound, or the strip's
sample error for a tangency point) and Δ the spread of its vertices' angles
about the wall's axis widened by `2d/(R − d)`; the band's bound is the largest
over its facets.

**Strip.** Consecutive stations and rows join in quads of two triangles
`(k, j), (k + 1, j), (k + 1, j + 1)` and `(k, j), (k + 1, j + 1), (k, j + 1)`.
Their local normal runs along `F_θ × F_t`, whose component along `u` has the
sign of proof 1's Jacobian: + inside the larger cylinder and − outside it. The
solid's outward side is `+u` at a convex edge (the ball in the material) and
`−u` at a concave one. The quads are swapped when the product of the
Jacobian's sign, the edge's (+ convex, − concave) and the pose's hand is
negative. Rows 0 and m are the T1 and T2 rings the bands share. The fillet is
an untagged `Faceted` face with role `fillet(k)`, as §4.9's band is.

**Proofs (each failure refuses `ErrUnsupported`).**

1. Cells (`saddlecanal.Prove`): over each station cell and row cell, split in
   two along t, second-order interval jets of `F` in outward-rounded float
   arithmetic give mean-value enclosures about the cell's midpoint, from sine
   and cosine enclosures of the exact turns k/N and (2k + 1)/(2N). Three tests
   must hold:
   - `s_J·det[F_θ, F_t, u] > 0` with `s_J = −σ1` (`Canal.JacobianSign`), so the
     pipe is regular and wound as for a small ball: its envelope is the surface
     the ball sweeps. As ρ → 0 the determinant tends to
     `ρ·det[c′, u_t, u] = −ρ·ω·(c′·ê)`, ê the unit axis about which `u` turns
     from `a1` to `a2` at speed ω > 0. φ stays inside (0, π) at every θ, so
     `c′·ê` keeps one sign around the loop, and θ = 0 shows it: `c′ = r'·ẑ`,
     and the arc turns about −ẑ inside the larger cylinder (`a1` in the first
     quadrant of the (x, y) plane, `a2 = −ŷ`) and about +ẑ outside it (`a1` in
     the third). A cell of the other sign lies past a fold.
   - `∂Φ/∂θ > 0`, with Φ the angle of `(F_y, F_z)` about the bore's axis, so
     every row runs once around the bore. Row 0 is the ellipse
     `(r'·cos θ·R/R', r'·sin θ)` in that projection and row m the circle of
     radius r, both run counter-clockwise for either σ1.
   - `∂(Φ, G)/∂(θ, t) > 0`, with `G = σ1·(|F_xy| − R) − (|F_yz| − r)` the
     distance from the larger wall on the ball's side less the distance from
     the bore. G is negative on T1 (on the larger wall, outside the bore) and
     positive on T2 (on the bore, off the larger wall on the ball's side) for
     either σ1. At a fixed Φ, `G` then grows with `t`, so `(Φ, G)` determines
     `(θ, t)` and the strip is embedded. The radius about the bore's axis
     cannot replace `G`: its t-derivative vanishes on T2, where the fillet
     touches the bore.

   A cell too wide to decide a test splits up to 8 × 16. At R = 10, r = 3,
   ρ = 3.4 inside the shaft, where `r' < R'` holds, the pipe folds near
   θ = 0 and the first test refuses: "the saddle fillet's cell at station 0
   folds: the pipe is not regular there". Dropping the sign `s_J` refuses every
   outside fillet with that text; dropping σ1 from `G` refuses them "is not
   embedded".
2. Chord ε: a point of a held strip triangle pairs with `F` at the same
   parameters. Linear interpolation on a triangle of a cell of widths h and k
   errs by at most `|F_θθ|·h²/8 + |F_θt|·h·k/4 + |F_tt|·k²/8` per coordinate,
   with the second partials enclosed over the cell. Each held vertex is compared
   with the point enclosure at its own station and row. ε is the largest bound
   plus that sample error, both read as 3D distances.
3. Areas: the fillet's, by the cells' mean-value integrals. The larger wall
   loses `A(r', R') − A(r, R)` with `A(a, b) = ∫₀^{2π} R·a²·sin²θ /
   (b·√(1 − (a/b)²cos²θ)) dθ`, the area of the ellipse `(b/R)²y² + z² ≤ a²` of
   the (y, z) projection lifted onto the wall. The bore loses
   `∫₀^{2π} r·|√(R² − r²cos²θ) − √(R'² − r'²cos²θ)| dθ`; outside the larger
   cylinder `R'² − r'²cos²θ − (R² − r²cos²θ) ≥ 2ρ·(R − r) > 0`. Each integral is
   enclosed station cell by station cell.
4. The band facet tests above (`bandFacing`), each exact.
5. Clearance: `checkMouthClear` refuses when a facet of a face other than the
   two walls, or a segment of another boundary loop of either wall, comes
   within its bound plus slack of the box of the window, the bore's cut-off
   part from the seam to `X_w`, and the held strip's vertices grown by ε:
   "another face of the body comes within the saddle fillet's reach". The box
   holds the region the fillet changes on either side of the seam, so a feature
   in the void beside a boss root refuses as one in the material beside a
   mouth does. Inside that box the solid's boundary is the two walls alone.
6. `checkStripClear` with both bands as taken ranges, and the strip's facets
   against each other.
7. The held audit of the rebuilt mesh.

**Claim F.** Let `H` be the held solid with `VolSymDiff = v`, `H'` the rewritten
one and `S'` the true filleted solid. Outside the box of proof 5, `H'` equals `H`
and `S'` equals `S`: proof 5 keeps every other feature out of it. Inside it `S'`
is bounded by the larger wall over `W` less T1, the fillet, and the bore from T2
to `X_w`, with the material on the side the faces' `Reversed` flags state. `H'`
is bounded there by the larger wall's band, the strip and the bore band, each
within ε of its exact surface, where ε is the largest of the strip's and the two
bands' bounds and every new vertex states it. So `VolSymDiff` grows by
`ε·(A_strip + A_fillet + A_bands + A_bands*)`, the held and exact areas, plus
rounding, whether the fillet removes material or adds it. `A_bands*` is bounded
on the larger wall by the cut ring's projected area less T1's ellipse
`π·r'²·R/R'`, times `R/√(R² − Y²)`, Y the ring's largest `|y|`, and on the bore
by `2πr` times the gap from the cut to T2's farthest x: `R' − X_w` inside the
larger cylinder, `X_w − √(R'² − r'²)` outside it. `AreaSlack` grows as §6.3
states, with the exact change the fillet's area less the two walls' (proof 3),
read at the end of its interval farther from the held change.

**Loose bounds at large ρ.** The fillet's area enclosure sums each cell's
mean-value remainder, which grows with the cell's second partials, and the
first split stops at the coarsest cell that passes the three tests. Every cell
splits in two along t first: a row cell is wider in t than a station cell in
θ, so the t term of the remainder dominates, and halving it quarters that term
for more proof time. The enclosure widens with ρ, and `AreaSlack` grows with it. ε grows with ρ too, so the volume bound grows with it. Each bound is proven; only its tightness suffers,
and splitting every cell finer would buy it back at a multiple of the proof's
time.

**Result.** The walls keep their `Cylinder` tags; each fillet is a `Faceted`
face. On the R = 10, r = 3 shaft `Tessellate(…, VerifyAll)` verifies
the two-mouth result's boundary and volume at ρ = 0.5.

**Reach.** Built and checked inside the larger cylinder: a Revolve shaft with
r/R = 0.3 (ρ = 0.5, 1.5) and 0.7 (ρ = 0.5), a `Hole` through the shaft's wall
(ρ = 1.5), and an Extrude shaft (ρ = 1.5) under each assignment of the two axes
to world coordinates and under a translation. Outside it: a block 80 × 60 × 20
with a radius-5 `Hole` down through it crossed by a radius-3 blind `Hole`
(ρ = 0.5, both mouths), a radius-5 `Boss` on a radius-15 rod (ρ = 1), and a
radius-3 pin joined through the R = 10 Revolve shaft (ρ = 1.5, both roots, a
left-handed pose). Near the size limit inside the shaft the route refuses before
`2ρ = R − r`: at r = 3 the pipe folds at ρ = 3.4 (`TestRetainedSaddleFilletRefusals`);
a window that would reach past the side of the shaft (`s ≥ R'`) refuses (SR5). A boss whose top face
lies inside the bore's cut refuses at proof 5. The volume per mouth, from the
tube-coordinate integral of `apitest/retained_saddle_fillet_test.go`, is
1.635978 mm³ removed at ρ = 0.5, 7.317201 at ρ = 1.0 and 18.593228 at ρ = 1.5
for the R = 10, r = 3 mouth; 0.540755 removed for the block (R = 5, r = 3,
ρ = 0.5); 4.701377 added for the rod (R = 15, r = 5, ρ = 1); and 6.619077
added for the pin root (R = 10, r = 3, ρ = 1.5). A rolling-ball grid (a point is in the region
when it lies on the ball's side and farther than ρ from that side eroded by ρ)
agrees within 0.5 % for the mouth and 0.2 % for the outside patterns.

### 5. Construction

The rewrite touches the faces `A` and `B` of the edge and, for R1, the two end
faces `G₀` and `G₁`. Everything else in the held mesh is copied unchanged.

1. **Region.** Let `K` be the closed region the blend changes: for R1 the
   triangular prism of the corner between the two setback lines, cut by the
   planes of `G₀` and `G₁`; for R2 the annular wedge between the rim, the two
   setback circles and the blend torus or cone. §6's gates prove that nothing
   but `A`, `B`, and the end faces `G` meets `K`.
2. **Rewrite the faces.** The plane face's facets are replaced. Its boundary
   loops other than the rim are kept with their vertices, the rim loop gives way
   to a polygon of `N` vertices on the setback circle (R2) or a cut line (R1), and
   the region is triangulated again by ear clipping over those loops, with one
   bridge from each hole to the outer loop. The ear clipping uses every loop
   vertex, collinear ones included, so the face keeps the vertices its
   neighbours share. The ear clipping reads each vertex in world coordinates
   along the plane frame's axes, not relative to the rim. For a plane normal to
   a coordinate axis that reading is exact, so two distinct boundary vertices
   stay distinct however close a mesh Boolean left them; on a tilted plane far
   from the world origin the reading rounds. Two neighbouring loop vertices that
   read one point refuse (SR5). A face with a vertex off its boundary loops refuses
   (SR5), except a plane face's vertex exactly on its plane (§2), and the
   straight edge's own two faces' vertex exactly on that face's retained plane.
   The rewrite triangulates those faces again from their loops, so the point a
   mesh adds to fan a face is dropped. The wall's facets are cut by the setback
   plane: a facet wholly beyond it stays, one wholly before it goes, and a crossing facet is clipped, with one
   new vertex per cut mesh edge shared by the facets on either side. A new wall
   vertex lies on a held facet edge, so it takes the bound of the facet it is
   cut from, the largest of its three corners' (§6.3), plus `τ` and the
   rounding of the cut, and then what the strip's band triangles beside it
   need (§6.3, Strip triangles).
3. **Strip.** The blend face is a grid of rings. The first ring is the plane
   face's new polygon and the last is the wall's cut vertices, sorted by angle
   about the axis; vertices of equal float angle are ordered by their exact
   turn about the centre, then by index. A fillet has `m − 1` rings between them on the exact torus,
   with `m` the smallest count whose arc sagitta `r·(1 − cos(Δα/2))` is at most
   `τ/2`, with `τ = 2⁻¹⁰·r` (not below `64·2⁻⁵²·scale`). A chamfer has none.
   The plane's polygon steps so that its sagitta on the largest of the rows'
   circles is within `τ/2` for a fillet and `τ` for a chamfer, and every row
   takes its azimuths; an R6 rim keeps both at `τ` (§6.3, Strip triangles). Neighbouring
   rings join by walking both in angle and advancing the ring whose next vertex
   comes first, which gives the band of triangles whatever the two rings' vertex
   counts. Each triangle is wound to face the blend's analytic outward normal.
4. **Ends (R1).** At each end vertex `V`, the end face `G` meets the blend
   cylinder in an arc: a circle arc when `G` is normal to `d̂`, an `Ellipse3`
   arc otherwise, running from `P_A` (the setback line of `A` on `A ∩ G`)
   to `P_B`. For a chamfer it is the segment `P_A P_B`. The region of `G`
   inside the polygon `V, P_A, arc, P_B` is removed from `G`'s facets by
   the same ear clipping; the arc enters as the chords of the strip's rows.
5. **Build.** The new vertex and triangle lists, the new groups and the new
   bounds go through `facetedrecord.Build`, which derives the topology, the
   measurements and the held audit the way `Placed` does.

#### 5.0 R1, a straight edge

All four faces an R1 blend touches are planes, so each is rewritten the way the
plane face of R2 is: its boundary loops change, and the face is triangulated
again over the new loops. Let `e` run from `V₀` to `V₁`, with `d̂` its direction,
`G_i` the end face at `V_i`, `eA_i = A ∩ G_i` and `eB_i = B ∩ G_i`.

1. **Exact points.** `V_i` is the meeting of the planes `A`, `B` and `G_i`.
   The setback line `LA` on `A` is the line parallel to `d̂` at in-plane distance
   `tA` from `e`, and `LB` likewise. `P_A,i = LA ∩ G_i` and `P_B,i = LB ∩ G_i`.
   For a fillet, the arc rows `α_j = jφ/m` of the exact cylinder meet `G_i` at
   `q_j,i = p_j + s·d̂`, with `p_j` the point of the cross-section arc and `s`
   the offset along `d̂` that reaches `G_i`. Row 0 is `P_A` and row `m` is `P_B`;
   a chamfer has rows 0 and 1 only. A chamfer's feet follow
   `agent-docs/exact-chamfer-design.md` §4: where the receiver's corners allow it,
   each is computed exactly and held with its own proven bound, not `τ`. `m` is the smallest count whose arc
   sagitta `r·(1 − cos(Δα/2))` is at most `τ·c`, with `c` the smaller of
   `|d̂ · n_G0|` and `|d̂ · n_G1|` (`rowStep`): in `G_i` each row chord stands
   for an arc of the ellipse and its sagitta grows by up to `1/c`, so the
   chords in both end planes stay within `τ` (§6.3). The grazing-angle
   refusal, `c < 10⁻³`, keeps `m` at most 1,124 for any `φ < π`.
2. **Chains.** In `A`'s loop, `e` is the run of vertices from `V₀` to `V₁` that lie
   on the line of `e` within their bounds. `eA_i` is the run that leaves `V_i`
   on the line of `eA_i`. The setback point `P_A,i` falls between two vertices of
   that run, or between `V_i` and its first vertex. The run's vertices nearer
   `V_i` than `P_A,i` go; `P_A,i` takes their place.
3. **Loops.** `A`'s loop loses `V₀`, `V₁`, the run of `e` and the removed parts of
   `eA₀` and `eA₁`, and gains `P_A,₀` and `P_A,₁` joined by one straight edge.
   `B`'s loop changes the same way. `G_i`'s loop loses `V_i` and the removed parts
   of `eA_i` and `eB_i`, and gains `P_A,i`, the intermediate `q_j,i` and `P_B,i`,
   in the loop's own direction. Each face is then triangulated again.
4. **Strip.** Rows `j = 0 … m` each hold the two vertices `q_j,₀` and `q_j,₁`.
   Neighbouring rows join in one quadrilateral of two triangles, wound to face
   the blend's outward normal. The strip's boundary edges are the setback edges
   of `A` and `B` and the end curves of `G₀` and `G₁`, which are the vertices
   the four faces share with it.

**A face that is reflex at an end.** A concave edge can end where one of its two
faces wraps the corner, as a plate's top face does at the foot of a wall. That
face leaves the edge on the far side of its line: the setback line meets the
line of `A ∩ G` behind the corner, not along the chain. The face is not trimmed.
Its boundary runs on from the corner `V` to the setback point and along the
setback edge, so `V` stays in its loop. The end face `G` is convex at `V`, and the
blend's end cap lies in `G`'s plane beside it. `G` gains the cap: its loop keeps
`V`, takes the arc and the setback points next to it, and loses the removed part
of the other face's chain. Its rebuilt facets are new surface, so `checkStripClear`
tests them against every facet the result keeps outside them, as it does for a
convex edge's cap (below). The same holds at every end of a concave edge on the
plane–cylinder (R3), arc (R4) and cylinder–cylinder (R5) routes: the end face
gains the cap and is tested. Which way a face leaves the edge comes from the
loop: a face lies to the left of its own boundary, so the way into `A` is
`n_A × δ`, with `δ` the direction the loop runs along the edge.

On the R1 route a convex edge may end where one of its faces is reflex, as a
wall's side edge does where its front face runs on below a step, or a stem's
edge where its front face turns onto an arm's underside. `A` is trimmed: it
loses the quadrilateral `V₀ V₁ P_A,₁ P_A,₀`, and since `P_A` lies on the line of
`A ∩ G` behind the corner, its boundary runs from the setback edge through
`P_A` and `V` along that line, so `V` stays in its loop as above. `B` trims at
`V` as usual. `G` must be convex at `V`. The blend's end cap is the corner
region projected along `d̂` onto `G`'s plane. It lies beside `G`'s corner,
across `G`'s chain on `B`, and `G` takes it with the same loop as for a concave
edge. The cap is sound for this reason. Before the blend the corner prism next
to `V` is material, since it lies in the convex wedge between `A` and `B`. Near
`V` only `A`, `B` and `G` bound the solid. The cap lies in none of them: it is
outside `G`'s region and off the planes of `A` and `B`. So the solid continues
across the cap's plane, and once the corner prism is removed the cap is the
boundary between that material and the void. `checkStripClear` tests the cap's
rebuilt facets like any other cap. Every row of the strip must run forward
along `d̂` from its end on `G₀` to its end on `G₁` by more than its two
vertices' bounds and the slack, else SR5. Where a face runs on past a
corner, the rows end on `G`'s plane outside the edge's own span, and a steep
`G` brings a row's two ends together or past each other, which folds the strip
and crosses its end caps. The other routes keep refusing a convex edge
whose face is reflex at an end, except an R3 wall that extends past the corner.
An end where both faces are reflex refuses on every route.

**An end face that is reflex at the corner.** A convex edge whose two faces both
trim at `V` can end on a `G` that is reflex there, as a boss's side edge ends on
the plate's top face, which wraps the boss's footprint. The corner region of the
cross-section, projected along `d̂` onto `G`'s plane, lies on the far side of
`G`'s two chains from `G`: before the blend it is inside the material, after it
is the floor of the removed region. `G` takes it as the blend's end cap. Its
loop is rebuilt as in step 3, which now encloses the corner region, and its
rebuilt facets are new surface: `checkStripClear` tests them, as it tests an
open chain's taken cap (retained-modify-reach §5.6), against every facet the
result keeps outside them, and refuses on any contact. The SR5 reach check
already keeps the rest of `G`'s boundary off the cap. Only this R1 route takes
a cap from a reflex `G`; the plane–cylinder, arc, cylinder–cylinder and mitre
routes refuse a reflex `G` under a convex edge (SR5).

**A wall that extends past the corner (R3).** A convex plane–cylinder edge can
end where the cylinder runs on past the end plane, as a keyway's top edge does
in a turned shaft: the shaft's surface continues beyond the keyway's end wall
`G`. The wall's chain leaves `V` along the end circle away from the wall's side
of the edge, `G` is convex at `V`, and the plane `A` trims there. Only the
band of the wall between the two end planes goes (§4.3), so the wall keeps `V`
and its whole chain. Beyond `G` the corner region stays material, so `G` takes
the corner region of the cross-section as the blend's end cap: its loop loses
the removed part of `A`'s chain and runs from `P_A` along the blend's rows to
the wall's setback point in `G`'s plane, then along the wall's cut in that
plane back to `V`. Its rebuilt facets are new surface and are cleared by
`checkStripClear` as the R1 cap is. The cap's loop of rows, the wall's run and
`A ∩ G` must be a simple polygon in `G`'s plane (`capSimple`), else SR5: no two
of its edges that share no vertex touch, no two that do run back along each
other, and it turns at `P_A` and at the band's corner the way it runs. A
fillet's cap closes in a cusp at `T_B`, where the ball touches the circle, and
the wall's cut in `G`'s plane lies within its bound of the circle, inside or
outside it. Where the ball's gap to the circle at the profile's last row
before `T_B` is under that, the row lies beyond the wall's cut and the loop
crosses itself: on the R = 10 shaft whose keyway runs from x = 20.37 off the
end, the r = 0.3 fillet's last row lies nearer the circle than the cut's band corner
does (`faceted_retained_blend_keyway_test.go`). The route then spaces the rows again
(`clearCusp`): the last gap before `T_B` spans the tube angle `ψ` at which the
ball lies `need` from the circle, `need` the largest bound of the band's
corner and the wall's run in `G`'s plane at every end that takes a cap, plus
`τ` and the slack, and the rest of the arc keeps the profile's step. The ball's
gap at `ψ` short of `T_B` is `κ·(√(D² + r² − 2κ·D·r·cos ψ) − R)`, `D = R + κ·r`,
which grows with `ψ` over the arc, so every earlier row lies farther from the
circle still. The rows are spaced so only where the cap on the profile's own
rows fails `capSimple`, so a cap that is already simple keeps them, and
`capSimple` decides the cap on the new rows too. The wider gap's sagitta is
charged as §6.3, "The cusp's last gap", states. A fillet whose tangent point
on `A` lies no farther than `need` from the circle refuses SR5, "the fillet's
arc comes no more than … mm from the wall's circle". On that shaft r = 0.007
refuses in `solveSection`, with "the blend's
tangent point does not lie on the wall's face": `T_B` lies under `minSine`
radians about the axis from the edge. The route refuses a wall that extends beside a
concave edge, or beside a convex edge whose plane is reflex at the
same end (SR5). Which way the chain leaves `V` comes from its first vertex,
which must lie farther along the circle's tangent at `V` than its bound plus
the slack, else SR5; `A`'s chain takes its sense from its last vertex the same
way.

**A wall that extends past the corner (R4).** A concave arc's wall may run on
past an end plane `G_i` that is convex at `V`: the wall's chain along `G_i`
leaves `V` against the way the wall climbs from the plane, farther than its
bound plus the slack, and the plane `A` trims there. Which of the two the chain
does is read from its last vertex, not from a residual. Only the band between
the two end planes goes (§4.4), so the wall keeps `V` and its whole chain, and
its new boundary gains the cut along `G_i`'s plane from the band's corner to
`V`. Beyond `G_i` the corner region stays void, so `G_i` takes the corner
region of the torus (or cone) section its plane cuts as the blend's end cap:
its loop runs from `P_A` along the end curve to the band's corner, along the
wall's cut back to `V` and along `A ∩ G_i` to `P_A`. The cap's rebuilt facets
are new surface cleared by `checkStripClear`, and the cap must be a simple
polygon (`capSimple`, SR5) as for R3. A fillet's end curve nears the wall's cut
in `G_i`'s plane without touching it, `X(s) − X_R = κ·(√(ρ(s)² − h²) −
√(R² − h²))`, tangent at the wall's tangent point, so its cap closes in a cusp
exactly as R3's does; where the cap on the profile's own rows is not simple
the rows are spaced again (`cuspParam`, `widenRows`): the last row before the
wall's lies `need` from the generator, `need` the largest bound of the band's
corner and the wall's run in `G_i`'s plane plus `τ` and the slack, and the
rows below keep their certified spacing. A fillet whose end curve lies within
`need` of the generator at the plane's row refuses SR5. The widened gap's chord
on each end plane lies within `endChord` of the end curve (the smallest
`τ·2^k` that `certify` accepts), and each last-row vertex on an end plane
states that plus the bound of the wall's cut vertex at the chord's other end,
which moves the chord's points by at most that much, plus the slack. Convex
arcs, an arc whose plane or both faces extend, and an arc whose end face is
reflex at `V` keep SR5.

**`G`'s turn at `V`.** On the R1 route the turn is read from the tag lines
`A ∩ G` and `B ∩ G`, not from held vertices: each line's direction is the cross
of two retained normals, and its sense comes from the chain's last vertex,
which must lie farther along the line than its bound plus the slack, else SR5.
The turn is the sine between the two directions about `n_G`; within `minSine`
of zero it is undecided, SR5. A reflex `G` is therefore never read as convex,
which would skip the cap's clearance. Nothing places the last vertex of a
chain whose face extends past `V` far from `V`; where it lies within its bound
the sense is undecided and the edge refuses. R3 reads the turn the same way at
an end where its wall extends past `V`, from the line `A ∩ G` and the end
circle's tangent at `V`. The other routes, and R3 at an end where its wall
ends with the edge, read the turn from `G`'s loop vertices beside `V`.

**Exact areas.** The removed region of `A` is the quadrilateral
`V₀ V₁ P_A,₁ P_A,₀`, and `B`'s likewise. The removed region of `G_i` is the
projection along `d̂` of the corner region of the cross-section, area
`r²(tan(φ/2) − φ/2)`, so its area is that over `|d̂ · n_G|`. A chamfer removes
the triangle `V_i P_A,i P_B,i` from `G_i`. A `G_i` reflex at `V_i` gains the
same area instead, as does an R3 end face where the wall extends past `V_i`; on
R3 the corner region is the plane's line from `K` to `T_A`, the blend, and the
wall's circle back to `K` (`cylLine.cornerArea`). The cylinder patch has area
`r·ℓ_c·φ + r²·tan(φ/2)·μ·(a₀ + a₁)`, with `ℓ(x) = ℓ_c + μ·(x − c)` the length of
the generator through `x` between the two end planes, `c` the axis point and
`a₀`, `a₁` the unit vectors from `c` to the two tangent lines. The chamfer plane
is a trapezoid of area `½(|P_A,₀P_A,₁| + |P_B,₀P_B,₁|)·|T_B − T_A|`.

#### 5.1 Groups and tags

| Face | Group fields |
|---|---|
| `A`, `B`, `G` | the old group, with `Surface`, `Retained`, `Reversed`, `CapRecord` and `NormalBound` unchanged. The face's patch is smaller; its plane or cylinder is not |
| blend | a new group: `Surface` the exact `Cylinder`, `Plane`, `Torus` or `Cone` of §4, `Retained = true`, `Reversed` as §4 states, `NormalBound = 0`, `Origins` the role the step mints (`fillet(k)` or `chamfer(k)`, `k` the edge's ordinal in the call), `Source` a new `SourcePatch` naming the modify producer and `k` (`Blend` true, no parent). The old groups keep their `Source` pointers |

The blend tag satisfies retention Claim R: the true blend patch lies on the
surface by definition of the blend. The face's normal bound follows retention
§3.3 from its own `δ_f`, which is the strip's sagitta and rounding.
A face whose `2·δ_f / R` reaches 1 reads `Faceted` (retention A4); that is
not a refusal.

#### 5.2 Edges

The new edges are constructed, not discovered, so they publish their curve
without retention's falsifier: the two setback edges (`Line3` for R1,
`Circle3` for R2), the end arcs (`Arc3`, or `Ellipse3` for an oblique `G`), and
for a chamfer the end segments (`Line3`). Each is marked `RetainedCurve`.
Their vertices are the exact intersections of the setback lines with the end
planes or the end faces' own edges, so a vertex position bound is the rounding
of that computation. An R1 chamfer's feet state the proven distance of the held
point from the exact one (`agent-docs/exact-chamfer-design.md` §4.2), 0 when the
exact point is a float.

### 6. Gates and bounds

#### 6.1 Table SR — refusals

| SR | Call | Existence | Sentinel |
|---|---|---|---|
| **SR1** | an edge outside Table RR: a curve that is not `Line3` or `Circle3`, a face that is not `RetainedTag`, a face with `NormalBound != 0` (except R1's end plane, below), a cylinder that states an `AxialDelta` outside the closed rim, a tangent-ended arc and a closed chamfer chain (§6.1), or a denoted patch whose tag is not exactly the surface it denotes (for R6's sphere, its record's exact centre and radius, §4.5), or a `Convex` flag that disagrees with §4.1 | body exists; no admitted source for the selected edge | `ErrUnsupported` (reach SX9's text) |
| **SR2** | two selected edges share a vertex and are neither a mitre run (§4.7) nor corners of perpendicular faces (§4.8), or a selected edge continues tangentially into a neighbour | no transition built | `ErrUnsupported` |
| **SR3** | no blend of that size exists: `R − r ≤ 0` for an inside ball (§4.2), `φ` within 10⁻⁹ of 0 or π, a chamfer distance of zero or below, an R6 setback that reaches the axis or runs past the sphere's pole (§4.5), or an R7 fillet inside the larger cylinder with `2ρ ≥ R − r`, where no ball touches both walls (§4.10) | no regular blend | `ErrDegenerate` |
| **SR4** | the setback does not fit (§6.2) and the held bounds prove it does not | the blend consumes the face | `ErrDegenerate` |
| **SR5** | the setback might not fit and the held bounds cannot decide it, or a plane that crosses an R6 sphere below `minSine` (§4.5), or an end vertex of an R1 edge is not a three-face corner with a retained planar `G`, or a chain's sense or `G`'s turn at the vertex is undecided (§5.0), or `G` turns the wrong way: a concave edge needs a reflex corner of `G`, or a convex one where `A` or `B` is reflex at the vertex, and a convex edge outside the R1 route (plane–cylinder, arc, cylinder–cylinder, mitre run) needs a convex one; or both `A` and `B` turn reflex at the vertex, or a row of an R1 strip runs along `d̂` by no more than its bounds (§5.0), or a convex edge outside the R1 route meets a reflex face other than an R3 wall that extends past the vertex beside a plane that trims there (§5.0), or another part of `G` comes within the blend's reach of the vertex, or a vertex of an R3 wall lies within its bound of an end plane the band is clipped by, or of the band's generator, unless another band plane puts it outside the band or, on an arc or chain band, a face whose tag is that end plane's puts it on the plane or a charged sign reading puts it on a side (§4.3), or the end cap an R3 or an R4 end face takes is not a simple polygon, or an R3 fillet's tangent point on the plane lies no farther from the wall's circle than the wall's held cut may at the cap (§5.0), or an R4 fillet's end curve lies no farther from the wall's cut in an end plane than that cut may, or a vertex of an R4 wall that lies on both sides of its plane lies within its bound of the band's planes, or another part of such a wall, or a chord of its boundary, comes within the band's reach, or the wall's facets along the rim decide no side or disagree (§4.4), or an R4 arc's end on a cylinder that is not parallel to the axis, that is coaxial with the wall, that the wall crosses at a charged sine of `minSine` or less, whose material lies inside it, or past which the wall does not run, or a chain of the wall off the end's generator, or a cap column that falls on a vertex of the end face's top edge, or a cap that folds, or a vertex beside `V` whose cut along `Π` does not end at one of them, or whose boundary runs against the cap, or a plane at one end and a cylinder at the other, or a chamfer at a cylinder end, or rows that meet an end plane stating a displacement at a sine below `minSine` (§6.3), or an end that lies between two meetings of its tags (§4.4), or another feature comes within an R7 chamfer's or fillet's reach, or an R7 fillet's cell, window, cut ring or band test fails (§4.9, §4.10), or an arc's rows need more than 1,024 rows for their chords in an end plane to certify within `τ` (§6.3), or a corner's sphere patch needs more than 1,024 rows to stay within the corner route's tolerance, or a revolve strip's row needs more than 4,096 columns for its azimuth term's share of `τ` (§6.3, Strip triangles) | body exists; the evaluator cannot prove its construction | `ErrUnsupported` |
| **SR6** | the held mesh disagrees with the exact vertex of its three tags by more than the vertex bound plus slack (§6.3) | the record and the mesh do not describe one solid | `ErrUnsupported` |
| **SR7** | a facet the rewrite keeps outside the new strip meets a strip facet, or a facet of an end face that gains a cap, that it shares no vertex with (the strip clearance of retained-modify-reach §5.6, run before the build), or the rewritten mesh fails the held audit (watertight, shells nested with consistent signs; the audit tests no facet pair for intersection) | the result is invalid | `ErrDegenerate` when the audit proves a face inside out (modify S8), else `ErrUnsupported` |

**Naming the unread edge.** When no route recognizes the selection, the generic
fillet or chamfer refusal of a mesh Boolean result keeps its text and its
sentinel and appends `; no retained blend class reads selected edge[N]: <reason>`
(`Unread`, `internal/retainedblend/unread.go`). It names the first selected edge
that is in no class of Table RR, from the faces' tags and the edge's face count
only, never from held geometry. The reasons are a face with no retained surface
(faceted), a free-form surface, a face without a retained tag, a tag with a
stated departure or one not proven to be exactly its revolve record, two retained cylinders
whose axes are not parallel (the canal surface of SR1), a retained cone or
torus, and otherwise the two retained kinds the edge lies between. `Unread`
skips the saddles §4.9 reads, the patterns both blends build; the canal-surface
reason names the crossing rims outside R7. The fillet's σ = (+1, +1) saddles
(§4.10) reach `Unread` only in a selection that mixes them with an edge no
route reads, and it names them with the canal-surface reason too.

**An oblique end plane.** R1's end face `G` may be an oblique plane: a retained `Plane` tag whose `TagDeparture`
`θ` equals its `NormalBound` and that states `ExtentDelta` `E` with no `AxialDelta` or denoted patch (a stacked view's
oblique wall, charged-window-chamfer §2.2, or a chamfer band's flat patch a mesh Boolean kept, faceted-source-retention
Claim F). Over `G` the true plane lies within `E` of the tag and turns from it by an angle whose sine is at most `θ`.
R1 reads `E` as the plane's charge, as it reads a displaced plane's δ below, so the corner, the rows on `G` and every
bound of §6.3 carry `4E/minSine`, and `θ` as the end's tilt. A tilt above `0.5` refuses ("an edge end meets a face
whose tag turns more than 0.5 from its plane"). `ExtentDelta` holds over `G` alone, so an oblique `G` that takes
the blend's end cap, new surface beside its held face, refuses ("an oblique end plane that takes the blend's end
cap is not built"); one that gives the cap up builds. The cap's true region lies on the tilted true plane, whose
area exceeds its projection onto the tag by at most `θ²` of it for `θ ≤ 0.5` (`1/cos α − 1 ≤ sin²α` there), so the
area bound adds `θ²` times the cap's exact area plus its plane charge (`obliqueCapArea`, §6.3). `A` and `B` stay
exact retained planes, and the mitre and corner routes refuse an oblique end plane with SR1's text. This is the end a
Fusion chamfer meets where a ramp's top edge runs into a chamfered outline's flat band.

**Displaced planes and walls.** A retained `Plane` tag may state `AxialDelta` δ, the
proven distance of the true plane from the tag along its normal. A route never
reads the tag as exact past δ. Each reader adds `4δ/minSine` per plane it reads
(`planeCharge`) to its slack, which the vertex-agreement test of SR6 and the
bounds of §6.3 both carry. A tangent chain charges every plane of the held
body, summed in the order of the held faces. A negative or non-finite δ makes
the face no retained plane, so the edge is outside Table RR (SR1). The charge
widens the published bound or the route refuses; no residual admits a
displaced plane.

A retained `Cylinder` tag may state `AxialDelta` δ too: the true wall is a
cylinder parallel to the tag whose axis offset and radius difference sum to at
most δ. A stacked brep's faceted view states it on each swept circular wall
(brep-modify §2.3), and the blend's result and every later Boolean keep it
("Publishing a charged wall", below). The closed rim (R2) reads such a wall
(`internal/retainedblend/wall_charge.go`), and so does an open arc whose two ends
are proven tangent ends (§4.4, "A tangent end"), with the charge of
boss-root-overflow-design §5.5.6. The rim circle is the plane's
meeting with the cylinder, so a true wall within δ moves the circle, and the
torus or cone built about it, rigidly by at most δ. R2 adds `4δ/minSine`
(`cylCharge`, the plane rule's amplification) to the rim's slack, so SR6, the
clearance of §6.2 and every strip and cut vertex of §6.3 carry it. Every term of
the rim's exact area change is linear in the wall radius, with slopes 2πt_A for
the annulus, 2πt_B for the band, π²ρ for a torus and 2πL ≤ 2√2π·max(t_A, t_B)
for a cone. So the area bound adds `(π² + 4π)·max(ρ, t_A, t_B)·δ`, and the volume
bound multiplies the strip's largest vertex bound by the blend's exact area plus
that term. A closed chamfer tangent chain charges it too
(`agent-docs/charged-window-chamfer-design.md` §3). Every other reader refuses a
charged cylinder with its existing text (`tagIsRecord`), every other open arc
(R4) included ("an arc on a wall whose tag states a displacement is not built").

**Publishing a charged wall.** A blend result keeps every charged `Cylinder`
tag with its δ (`publishChargedWalls`), so a later `Fillet`, `Chamfer` or
Boolean reads the wall as it reads a displaced plane: R2 charges it and every
other route refuses it by the texts above. The charge is a statement about the
wall, not about the blend, so the blend that kept the wall leaves it unchanged.
A curved group that states a displacement and is no `Cylinder`, or whose δ is
negative or not finite, is untagged, with the area shortfall charged.

The view marks each wall it charges (`Group.ViewCharge`, copied to
`topo.Face.ViewCharge`), and the mark is the one difference between a wall
inside the blend and a published one:

- *Edge curves.* `facetedrecord.RetainedEdgeCurve` publishes a curve beside a
  marked wall for any finite δ ≥ 0, as R2 needs inside the view. Beside an
  unmarked charged wall it applies the displaced plane's rule: every curve
  declines when δ exceeds the chain's arithmetic allowance
  (`64·2⁻⁵²·max(1, |coordinate|)`, retention §10.1 E6), so a published
  `Circle3`, `Arc3` or `Line3` lies within that allowance of the tags' meeting, as
  a displaced plane's circle does. A larger δ leaves the rim `FacetedCurve`.
- *Clearing.* `publishChargedWalls` clears the mark on every kept wall and
  rebuilds the body, so its edges take the published rule. A body is rebuilt
  only when some group carried a mark or was untagged.

A published charged wall reaches these readers outside this package. Each
charges δ, refuses the wall, or is listed in `agent-docs/missing-features.md`
as reading the tag as the wall.

| Reader | With a charged `Cylinder` |
|---|---|
| `facetedrecord.SourceTag` (every mesh Boolean) | carries δ with the tag, as for a plane with no record. The true result patch is a piece of the true operand face, so it lies within δ of the tag. A negative or non-finite δ gives the group no tag. The shifted retry translates the tag and keeps δ (`boolshift`), since a translation moves the true wall with it |
| `facetedrecord` assembly, `NormalAt` | the held normal term is `HeldNormalTerm(T, 2·(δ_f + δ))`. A held point lies within `δ_f + δ` of the tag's radial line, and the true radius is at least `R − δ`, so the normal moves by at most `2(δ_f + δ)/(R − δ)`, which `4(δ_f + δ)/R` covers while the term stays below 1 (A4) |
| `facetedrecord.ExactGroupPlaced` (an exact rotation copy) | keeps δ with the moved tag: an exactly orthogonal map moves the true wall rigidly. The float image's own departure enters `NormalBound`, as for any placed tag |
| float placement (`placed.go`: `Copy`, `MirroredCopy`, `Pattern`, motion) | keeps δ, as for a plane with no record. A placement that rounds states a nonzero `TagDeparture`, which every positional reader refuses (E12, SR1) |
| `ScaledAt` (`point_plane_scale.go`) | scales δ by `|s|`, rounded up. The scaled tag adds only its rounding to `NormalBound` |
| `edgewedge` (`Edge.Wedge`, its selector) | reads `Undecided` at a charged wall |
| `RetainedEdgeCurve` | the published rule above |
| `retained_loop_chamfer.go`, `partial_loop_chamfer.go`, `trimgraph`, open and fillet tangent chains | refuse a nonzero δ |
| a closed chamfer tangent chain | charges δ (`agent-docs/charged-window-chamfer-design.md` §3) |
| `export/step_analytic.go` | `facedeparture.TagDeparts` reads δ as a departure, so the body takes the faceted writer |
| `parts` joint inference (`parts/faces.go` `addCylinder`) | reads a charged wall as an unread face (`facedeparture.Charged`, installed by the root package), so a joint on it is `Undecided` with `CauseUnreadFace`. Its clearance measurement would otherwise take the tag's radius and axis as the true wall's |
| `Face.Surface()` and the site builders that read it (`Hole`, `Boss`, `Emboss`, `Thread`, `planarDihedral`) | see the tag with no positional bound, as for a displaced plane or a departed tag (api-design §5, "Surface parameters carry no `Exactness`"). `planarDihedral` reads a design angle from it. `Hole`, `Boss`, `Emboss` and `Thread` take the tag's radius and axis as the true wall's, so a feature placed on a charged wall may sit up to its charge off it; `agent-docs/missing-features.md` lists that gap. `NormalAt` carries the charge |

Readers gated by payload class (contact geometry, clearance, gate diameter,
stitch, prism, brep and capblend routes) never receive a faceted body
(retention §4). `contactfaceted.AxisSupportAtPose` does receive one and reads
only `HeldPlanar` faces.

A closed chamfer tangent chain reads charged walls, a displaced or charged
base and an oblique wall's `ExtentDelta`, and charges them
(`agent-docs/charged-window-chamfer-design.md` §3). Any other chain does not:
the junction test reads a charged tag to name the gate a selected edge stops
at, so a proven tangent junction to an unselected edge refuses with SM2's text,
read on tags that state a displacement, an open chain refuses "an open tangent
chain on walls that state a displacement is not built", and a fillet chain "a
tangent chain fillet on walls that state a displacement is not built". A junction at a proven tangent end (§4.4) is no
chain junction, charged or not: the arc's continuation along `G` is convex while
the arc is concave, and the arc end takes the tangent end's reading
(boss-root-overflow-design §5.5.7).

A blend face built from displaced planes or walls may lie off its own tag by
the same charge. A chamfer plane states it as its `AxialDelta`. A fillet
cylinder, a rim torus or cone and a corner sphere built from a charged face are
not published as retained tags, so the face stays faceted; only a wall the view
charged states a cylinder's `AxialDelta`. With no charge every tag is published as built. A tangent
chain other than a closed chamfer chain (Part M §5.3) reads its tags as exact, so a
plane with any nonzero `AxialDelta` is no wall of it, as `facedeparture.TagDeparts` reads it.

Gate order: base gates of modify §4 stage 1, then SR1, SR2, SR3, SR6, SR5, SR4,
then the construction, then SR7. R7 runs its own order (§4.9): recognition,
the size, the seam loop, then per saddle the band proofs, the outside and
clearance proofs and the crossing audit, then the Cut. The R7 fillet's order
(§4.10) is recognition, the size, the seam loop, then per mouth the cell
proofs, the window and the bore cut, the clearance, the wall cuts, the bands,
the strip, `checkStripClear`, and the build's audit.

#### 6.2 Clearance

The strip must lie inside `A` and `B`. Each held boundary vertex is known to
within its bound `b_v`, and the held bound of a retained plane facet is the slab
bound of retention §11. The gate uses held data and published bounds and
nothing fitted.

- **R1.** For `A`, let `s(v)` be the signed in-plane distance of a boundary
  vertex `v` from the edge line, positive into `A`. Every boundary vertex of
  `A` other than the edge's two end vertices must satisfy
  `s(v) − b_v ≥ t + slack`. A boundary segment between two such vertices stays
  at `s ≥ t + slack`, so no boundary edge of `A` other than the two end edges
  enters the strip. The two end edges are `Line3` edges to the end faces and
  reach `s ≥ t` by the same inequality at their far vertices. The same holds
  for `B`. If some vertex has `s(v) + b_v < t` the strip provably leaves the
  face (SR4); if neither inequality holds, SR5.
- **R2.** On the plane, every boundary segment of every loop other than the rim
  must be at distance at least `R + σ·t + b + slack` from the axis, the way
  `σ` of §4.2 points. On the cylinder, every boundary vertex other than the
  rim's must lie at axial distance at least `t + b + slack` from the rim's
  plane.
- **R6.** The plane as R2's, out to `ρA`. On the sphere, every vertex of a
  boundary loop other than the rim's must clear the cut plane at `|zB|` by more
  than `b + τ + slack` on the wall's side. A vertex inside the face is no
  feature and is not gated; a kept one near the cut takes the bound of §4.5.

#### 6.3 Bounds of the result

A ring vertex on the plane face lies at the exact setback circle, so it is an
exact function of the tags. A facet states how far it lies from its true piece
only through its corners' bounds (faceted-vertex-bounds §2), so every strip
vertex takes the strip's own bound plus the rounding of its computation: `τ`,
on a strip whose triangles lie in the plane of two straight rows (R1, R3, the
cylinder–cylinder line, a mitre run and the corner route's strips) and on a
cone or torus alike, except an R6 fillet's rows, which state `2τ` (Strip
triangles, below). A straight edge's
rows meet `τ` in the end and mitre planes they end on as well as on the
cylinder, because their step is set by the most oblique of those planes, and an
arc's rows meet it in the planes its ends lie on, because their gaps are halved
until each chord there certifies within `τ` (Area, below). A wall cut vertex
takes the bound of the held facet it is cut from, the largest of its three
corners', plus `τ` and the rounding (§5 step 2): a curved wall's facet (R2,
R3, R4, R5, R6, a chain's arc entries) claims its true piece within that bound
only as a whole, and the cut edge's two ends alone do not carry the claim. On
a planar wall (a chain's straight entries) the two ends would suffice, since
their true points lie on the true plane and so does every point between them
(faceted-vertex-bounds §2); the chain reads the facet's bound there too. Beside a revolve strip a
cut vertex also takes what the band's triangles need (Strip triangles). An old
vertex keeps its bound.

**Claim K.** Let `H` be the held solid with `VolSymDiff = v`, `H'` the rewritten
held solid, and `S'` the true blended solid. Then
`vol(H' △ S') ≤ v + ε·(area(strip) + area(blend)) + Σ_t (h_t + δ_t)·area(t) + ρ`,
with `ε` the largest vertex bound of the strip's facets (Strip triangles; an R1
chamfer's feet carry their own proven bounds, `agent-docs/exact-chamfer-design.md`
§4.2 and Claim X), the
sum over every facet `t` of every plane face the route rebuilds by re-triangulating
it (`retriangulate`, §5; on every route, mitre runs and corners included), `h_t`
the largest exact distance of `t`'s corners from that face's tag plane, rounded
up, `δ_t` the face's `AxialDelta` plus its `ExtentDelta` (each zero when it states
none), and `ρ` the rounding of the new vertices times the area of the faces they
touch. On the four faces of an R1 chamfer on the exact path, `h_t + δ_t` is
replaced by the smaller of it and the largest bound of `t`'s corners, read from
bounds that are final when read (`agent-docs/exact-chamfer-design.md` §4.4): each
corner lies within its bound of a true point of the face's plane
(faceted-vertex-bounds §2), so the facet lies within the largest of them of it.

*Proof.* Outside `K` and the re-triangulated plane faces, `H'` equals `H` and
`S'` equals the true solid `S`, because the gates of §6.2 show that no other
face enters `K`. So the difference there lies inside `H △ S`. Let `Y` be the slab
between a re-triangulated face's new facets and its true plane. Then
`vol(H' △ S') ≤ v + vol(Y \ (H △ S))`: the slab between the face's old facets and
the true plane lies in `H △ S`, so only the new slab can add. A new facet `t`
lies within `h_t` of the tag plane and so within `h_t + δ_t` of the true plane,
and its slab has volume at most that times `area(t)`. This holds for a cap an end
face gains (new surface on the far side of `K`) as for any other facet of the
face, since the term reads its corners' actual distances, not their bounds, except on an exact R1
chamfer's four faces, where the smaller of that and the largest corner bound is read
(`agent-docs/exact-chamfer-design.md` §4.4). Each
blend charges a rebuilt face again, so `n` blends on one displaced face add its
`δ·area` `n` times; the published bound stays sound. Inside `K`, `S'` is the exact blended region and
`H'` is the polyhedron on the strip's chords; every strip facet lies within its
own bound, at most `ε`, of the blend face and the blend face within it of the
facets, so they differ by at most `ε` times the larger of the two areas, plus
the rounding of the grid's vertices. ∎

**Area.** Write `Δ_held` for the held area of the new facets less that of the
old facets the rewrite replaced, and `Δ_true` for the exact area of the blend
face less the exact area of the annulus and the wall band it removes. Both
bodies have the same area elsewhere, so the held total differs from the true one
by the old difference plus `Δ_held − Δ_true`. The old difference was proven
only with `G + AreaSlack`, where `G` sums every face's geometric term (the
smaller of its largest facet bound times its perimeter and its facets'
perturbation allowance, `facetedrecord.FaceAreaGeom`). The rebuilt body
charges its own sum `G'`, each term computed afresh from the face's new facets,
and a term can be smaller than before: a rewrite drops high-bound rim vertices
and whole facets, and the arc that replaces a corner shortens a face's
boundary. Write `G_g` and `G'_g` for the sums over the faces whose first facet
comes from source group `g`, in the old and the rebuilt body; group indices
carry over, since a rewrite only appends groups. The new `AreaSlack` is the old
one plus `|Δ_held − Δ_true|`, the rounding of the two area sums
(`proofbound.FacetAreaTermSlop` per facet), and the shortfall
`Σ_g max(0, G_g − G'_g)` over the old body's groups. Then
`G' + AreaSlack' ≥ G + AreaSlack + |Δ_held − Δ_true|`, which bounds the new
difference whatever any face's new term is. `G_g` is summed rounding up and
`G'_g` rounding down, and each positive difference is rounded up, so the
charged shortfall covers the exact one. `G'` exists only once the body is
built, so the shortfall joins `AreaSlack` and every area bound after the build
(`facetedrecord.ChargeAreaSlack`); `AreaSlack` enters no other reading.

The rebuilt body charges a rewritten face's new term itself, so `AreaSlack`
carries only what the face's term lost. Charging the face's whole old term
instead charges the same allowance a second time, beside its new term, and a
blend of `n` edges that each rebuild one large face then charges that face's
term `n` more times. A face that is the only face of its group in both bodies
keeps its own area bound at or above its old term plus the old `AreaSlack`.

A face's own area bound is not derived from its old one: its term is computed
afresh from its new facets (`agent-docs/faceted-vertex-bounds-design.md`
§4.2), and the shortfall keeps only the body total. A new boundary chord of
length `c` on a face, standing for an exact curve of the face whose sagitta
against it is `s`, leaves a segment of area at most `c·s` between them: the
segment lies in the chord's `c × s` rectangle, since the arc spans less than a
half circle. Both forms of the face's term charge at least `β·c` for that
chord, `β` the smaller bound of its two end vertices: the perimeter form's
`δ_f` is at least `β` and its perimeter holds `c`, and the facet beside the
chord charges `δ(t)·(|u| + |v|) ≥ β·c` by the triangle inequality. So the face's
own term covers its share of the chord error wherever `s ≤ β`. The rim ring
meets that (its step keeps the ring's sagitta within `τ`, and each ring vertex
has the strip's bound plus slack), and so do a straight edge's rows, both on the blend's
circle and where they end on an end face oblique to the edge. In the end plane
`G` the rows' chords are the cross-section's chords projected along the edge,
which stretches lengths by at most `1/|d·n_G|`, so a chord whose cross-section
sagitta is `s` has sagitta at most `s/|d·n_G|` in `G`. The step
`2·acos(1 − τ·c/r)`, with `c` the smallest `|d·n|` over the planes the rows end
on (§5.0 step 1, `rowStep`), keeps `s ≤ τ·c`, so the sagitta in `G` is at most
`τ` against each row vertex's bound `τ + slack`. A mitre plane (§4.7, §4.8) is
held to the same step: there no face takes the gap between the chord and the
ellipse, and a true point on the ellipse lies within the chord's sagitta of the
mesh only through that chord.

**The cusp's last gap.** An R3 fillet whose rows `clearCusp` spaces again
(§5.0, "A wall that extends past the corner") has one gap before `T_B`, of tube
angle `ψ`, wider than the profile's step, and its sagitta
`σ = r·(1 − cos(ψ/2))` can exceed the `τ` its rows' bounds state. Work in the
section normal to the edge: `P` is the last row's exact point, `C` the exact
`T_B`, and `C'` a cut vertex of the wall's cut row, the band's corner in an
end plane or a crossing of the setback generator. `C'` lies on the
generator's half-plane, within its bound `β(C')` of the true cylinder, whose
nearest point to it in that half-plane is `C`; so `|C' − C| ≤ ε` with
`ε ≤ β(C')`. The bound is built term by term (`cuspCharge`,
`internal/retainedblend/cyl_rewrite.go`):

1. A point of the chord `PC` at the fraction `ν` lies at distance
   `√(r²·cos²(ψ/2) + s²)` from the ball's centre, its offset from the chord's
   middle `s` with `s² = r²·sin²(ψ/2)·(1 − 4ν(1 − ν))`. Its gap to the ball's circle is
   `r·(1 − √(1 − x))` with `x = 4ν(1 − ν)·sin²(ψ/2)`, and
   `1 − √(1 − x) = x/(1 + √(1 − x)) ≤ x/(1 + cos(ψ/2))`, so the gap is at most
   `4ν(1 − ν)·σ`. The point at `ν` along `PC'` lies within `ν·ε` of it.
2. A point of the arc whose foot on the chord is at `ν` lies
   `√(r² − s²) − r·cos(ψ/2)` from the chord. With `√(1 − y) ≤ 1 − y/2` that is
   at most `4ν(1 − ν)·σ + σ²/(2r)`, and the foot lies within `ν·ε` of `PC'`.
3. `4σ·ν(1 − ν) + ν·ε` is largest at `ν = (4σ + ε)/(8σ)` when `ε < 4σ`, where
   it is `(4σ + ε)²/(16σ)`, and at `ν = 1` otherwise, where it is `ε`. So the
   segment `PC'` and the arc `PC` lie within `cuspCharge(σ, ε, r)`, that
   maximum plus `σ²/(2r)`, of each other, each step rounded up. The charge
   grows with `ε`, so `β(C')` may stand for `ε`.
4. The blend is a cylinder along the edge, so a point's distance from it is the
   distance of its section from the circle. A strip triangle of the last gap
   has its row vertices at `P`'s section and its one or two cut vertices
   within their bounds of `C`, so its section is a segment from `P` to a
   point within the larger of those bounds of `C`, and steps 1–3 bound it
   both ways. Every such triangle has a cut vertex, and every cut vertex takes
   `max(β, cuspCharge(σ, β, r) + slack)` (`chargeCusp`), so its corner maximum
   states the triangle.
5. R3's end planes are normal to the edge, so in each end plane the gap's
   chord is the section's chord from `P` to the cut row's end vertex. That
   vertex is charged by step 4, and the last row's vertex there takes
   `max(β, σ + slack)`, so both ends of the chord state its sagitta, as the
   chord term of Area below needs.

The ball's gap at `ψ` is about `D·r·ψ²/(2R)` and `σ` about `r·ψ²/8`, so `σ` is
about `need·R/(4D)`, a quarter of the wall's bound at the cusp. With `4σ`
near `ε` the charge stays near `ε`: on the R = 10 shaft the r = 0.3 fillet's
gap spans a wider `ψ` than the profile's step and its `σ` exceeds `τ`, yet every facet lies within its own bound
and `Mesh.Bound` (`TestRetainedBlendKeywayCuspBounds`). The rows keep the profile's count or one fewer.

An arc's rows (§4.4) end on planes `G` parallel to the axis at distance `h`
from it. The row at profile parameter `s`, the circle of radius `ρ(s)` at height
`z(s)`, meets `G` at the in-plane distance `X = √(ρ² − h²)` from the axis's
foot, so the end curve is `(X(s), z(s))`. The profile's own step does not bound
the sagitta of its chords: where `G` passes close to a row's circle, `X/ρ` is
small, and with the profile's rows a hole mouth's fillet strays about 4 times
`τ` from its chords at `X/ρ = 0.2` on the wall's row, past the published
`Bound` (§9). The rows are sized for the end curve instead (`arcRows`,
`internal/retainedblend/arc_section.go`):

1. Both coordinates are monotone in `s`, because `ρ` and `z` are. The curve's
   tangent makes the angle `ψ` with the `X` axis, `tan ψ = g(s)·X/ρ`, with
   `g = |dz/ds| / |dρ/ds|`: `tan α` on the torus, `d_B/d_A` on the cone.
2. On a piece `[t₀, t₁]` of the profile `g` does not decrease and `X/ρ` is
   monotone, so `ψ` lies in `[atan(g(t₀)·min X/ρ), atan(g(t₁)·max X/ρ)]`,
   inside `[0, π/2]`. `certify` widens that range by the angle allowance `ε_ψ`
   below, clamped to `[0, π/2]`.
3. A curve from `P` to `Q` whose velocity stays in the cone of two directions
   lies in `P + cone` and in `Q − cone`: the parallelogram with diagonal `PQ`
   and sides along the two directions. No point of the piece lies farther from
   the chord's line than the parallelogram's corners do.
4. Each parallelogram lies in the box whose diagonal is the chord, and every
   point of that box projects onto the chord, so the distance from the line is
   the distance from the chord.
5. `certify` accepts a piece whose corners' computed distance `d̂` satisfies
   `d̂ + δ ≤ τ`, with the margin `δ` below, and halves any other piece, up to
   4,096 pieces. A chord that does not certify on every end plane has its gap
   halved, and so is a gap across which the azimuth of the rows' ends on any
   `G` moves by more than half the step the gap's budget allows, a quarter on
   the band's gap (`arcRows`, Strip triangles). The rows start from the
   profile's own count, a fillet's arc sagitta within `τ/4` on the arc route
   (`arcProfileShare`) and `τ/2` on the chain route (`profileShare`), or two
   rows for a chamfer. The blend refuses (SR5) past 1,024 rows (`maxSectionRows`), and at once
   when `δ ≥ τ`, where no number of rows certifies. Across ordinary sizes of `R` and `r`, both signs of `z` and
   small `X/ρ`, the rows stay far below that limit.

Every chord on every end plane then lies within `τ` of its arc of the end curve,
which each row vertex's bound, at least `τ + slack`, states. The last chord ends at
the wall's cut vertex, whose larger bound covers that vertex's own offset from
the exact point. A halved gap keeps the cross-section's sagitta within `τ`. The
chain route's arc ends (retained-modify-reach §5.5 step 5) are sized the same
way, and every entry of the chain takes the same rows.

**Strip triangles.** Every strip triangle of a surface of revolution (R2, R4,
R6 and the arc entries of a tangent chain, on a torus or a cone) has its three
vertices on two neighbouring rows. Let `ṽ_i = F(s_i, φ_i)` be the exact point
a vertex `v_i` stands for, at profile parameter `s_i` and azimuth `φ_i`, with
meridian point `(ρ_i, z_i)`, and `e_i = |v_i − ṽ_i|`. For a row vertex `ṽ_i` is
the point of its row's circle at the azimuth the route computed for it, so
`e_i` is the rounding of placing it there.

*Claim S.* Every point `x = Σλ_i·v_i` of the triangle lies within
`max e_i + σ + ρ_g·(1 − cos(Δφ/2))` of the blend surface, where `σ` is the
profile's sagitta over the two rows' gap (`r·(1 − cos(Δα/2))` over a tube angle
gap `Δα`, 0 on a cone's straight profile), `Δφ` the largest angle between two of
the `φ_i`, and `ρ_g` the larger radius of the two rows.

*Proof.* Term by term:

1. `x̃ = Σλ_i·ṽ_i` lies within `Σλ_i·e_i ≤ max e_i` of `x`.
2. `x̃`'s height above the rim plane is `z̄ = Σλ_i·z_i`. Its horizontal part is
   `Σλ_i·ρ_i·u(φ_i)`, `u` the unit direction at an azimuth. Its length is at
   most `ρ̄ = Σλ_i·ρ_i`, and its component along the direction halfway between
   the two outermost azimuths is at least `ρ̄·cos(Δφ/2)`, since every `u(φ_i)`
   lies within `Δφ/2` of that direction. So `x̃`'s meridian point `(ρ_x̃, z̄)`
   lies within `ρ̄·(1 − cos(Δφ/2))` of `(ρ̄, z̄)`, and `ρ̄`, a convex
   combination of the two rows' radii, is at most `ρ_g`.
3. The `(ρ_i, z_i)` are two profile points, one per row, so `(ρ̄, z̄)` lies on
   the chord between them, and the chord lies within `σ` of the profile.
4. A surface of revolution is as far from a point as its profile is from the
   point's meridian image, so `x̃` lies within `σ + ρ_g·(1 − cos(Δφ/2))` of it,
   and `x` within that plus `max e_i`. ∎

*The other way.* Every point `P` of the patch between the two rows, at azimuth
`φ`, lies within the same sum, `e` aside, of the triangles, each term now read
from the triangles that meet `φ`'s half-plane. Write `ê = ρ_g·(1 − cos(Δφ/2))`
for the azimuth term, `β = Δα/2`, and work in that half-plane, where the
triangles between the two rows cut a path `γ` from row `j`'s polyline to row
`j + 1`'s:

- Where both rows take the same azimuths (a closed rim, a chain's meridian
  columns), the triangles between two columns form a planar trapezoid, and `γ`
  is the chord between the rows' profile points with `ρ` scaled by one factor
  `k ≥ cos(c/2)`, `c` the column step. Its point nearest `P` lies within
  `σ + (1 − k)·ρ_g ≤ σ + ê`.
- Otherwise let `L` be the line through `P` normal to the chord, and `g` the
  coordinate along the chord from row `j`'s profile point. `γ`'s end on row
  `j`'s polyline lies within `(1 − cos(c_j/2))·ρ_j` of that row's profile point,
  radially, `c_j` the row's column step and `ρ_j` its radius, and likewise on
  row `j + 1`. Where `P`'s own `g` lies farther than that from both chord ends,
  `L` separates `γ`'s ends and meets `γ` at a point within `ê` of the chord, by
  step 2, while `P` lies within `σ` of it on `L`: within `σ + ê`. Nearer row
  `j`'s end, `P` lies within `(1 − cos(c_j/2))·ρ_j / cos β` of that row's
  profile point, since the arc stays below the tangent at its end, which leaves
  the chord at the angle `β`, and so within `(1 + 1/cos β)·(1 − cos(c_j/2))·ρ_j`
  of `γ`'s end. Rows that end on a plane hold that within `τ` (`rowSteps`, step
  3 below).

So the exact strip's facets and its patch lie within `σ + ê` of each other
both ways, the near-end term aside, which the rows' steps hold within `τ`.

*Column limit.* A row takes at most 4,096 columns over a closed rim's turn or
an open row's span (`columnCount`, `maxStripColumns`). The column step shrinks
with `√(size/ρ)`, so a blend tiny against its rim would need more: on a closed
rim a fillet reaches the limit at a smaller `r/ρ_hi` than a chamfer, whose columns take all of `τ`, does at
`d/ρ_hi`. On the arc route the last row's step reaches it first (step 4 below), at a larger ratio: on the Ø8
bore's mouth whose wall row meets the end plane at the sine 0.4, the limit is reached; at the sine 0.2 the wall gate
("the held bounds cannot show that the blend leaves the wall") refuses first. Such a blend refuses
(SR5) rather than step coarser than its bound states.

*Closed rims and meridian columns split `τ` between the two terms.* The terms
add, so a fillet spends half of `τ` on each: its rows keep `σ ≤ τ/2`
(`profileShare`; the profile's step, §5 step 3), and its columns keep
`ê ≤ τ_a = τ/2` (`azimuthShare`). A chamfer's profile is straight, `σ = 0`, so
its columns take all of it, `τ_a = τ`. Splitting `τ` instead of stating `2τ`
keeps a later Boolean's chain-depth gate (api-design §8) admitting the blend
face: a hole mouth filleted with r = 1 and then cut through its torus refuses
there with `2τ`. The columns hold the azimuth term:

- A closed rim (R2, R6) gives every row the plane polygon's azimuths, whose step
  `c` keeps `ρ_hi·(1 − cos(c/2)) ≤ τ_a` (`ringCount` at the largest radius of
  any row; an R6 arc can bulge past the rim's radius, so its rows are read). A
  triangle between two such rows spans one column.
- A chain's arc entry with no free end steps its meridian columns the same way.

*Rows that end on a plane take a budget per gap.* A row that ends on an end
plane `G` (R4, and a chain's free end, whose columns fan out from the end
curve) runs between azimuths that differ from row to row, so its triangles
span the shift of the rows' ends as well as a column. Number the vertex rows
`0 … n − 1` from the plane's, the wall's cut row `n` after them. Write `w_j`
for row `j`'s span, `c_j` for its column step, and `s_g` for the larger move,
over the end planes, of the end azimuths between rows `g` and `g + 1`, read
from the azimuths the route places those ends at (`arcRowSteps`,
`fanColumns`). Term by term:

1. Each row's azimuth is linear in its own fraction `t` of the row, so two
   rows' azimuths at one `t` differ by at most `s_g`. The arc route joins two
   rows by whichever row's next vertex comes first in `t` (`zipOpen`). Its merge
   keeps both rows' current `t` at or below both rows' next, so a triangle's
   three `t` lie within one column of one of the two rows. Two vertices on one
   row are one column apart. A vertex on each row, at `t` and `t'`, lie at most
   `|t − t'|·min(w_g, w_{g+1}) + s_g` apart in azimuth, and `|t − t'|` is at
   most one column's fraction of either row, so that is at most one column of
   that row plus `s_g`. So `Δφ ≤ max(c_g, c_{g+1}) + s_g`. A chain's fan
   columns sit at one fraction of every row, so a triangle of a quad between two
   columns spans at most `Δf·max(w_g, w_{g+1}) + s_g`, the same sum with
   `c_j = Δf·w_j`. The fan takes as many columns as its most demanding row
   needs (`fanColumns`): at a boss's corner (`κ = −1`) the rows nearer the axis
   sweep far wider angles than the rim, at a smaller radius.
2. For each gap `g < n − 1` between two vertex rows, `rowSteps` holds both
   rows' steps to `C_g − s_g`, where `C_g = azimuthStep(τ − σ_g, ρ_g)`, `ρ_g` the
   larger radius of the two rows and `σ_g` the profile's sagitta over the gap.
   `C_g` is the largest spread whose term `ρ_g·(1 − cos(C_g/2)) =
   2·ρ_g·sin²(C_g/4)` stays within `b = (τ − σ_g)·(1 − 2⁻¹⁶)`, computed as
   `4·asin(√(b/(2ρ_g)))`. A step that is not positive, or not a number,
   refuses (SR5, `columnCount`). Then `Δφ ≤ C_g`, and Claim S puts the
   triangle within `max e_i + σ_g + (τ − σ_g) = max e_i + τ`.
3. Each row's step also keeps `(1 + 1/cos β_g)·ρ_j·(1 − cos(c_j/2)) ≤ τ` for
   both its gaps, `β_g` half the gap's tube angle and 0 on a cone, which bounds
   the patch near the row from the other side (The other way). That cap, not
   the gap's, binds wherever the shift is small and `σ_g` well under `τ/2`.
4. The band's gap, `n − 1`, is charged per triangle (The band beside the
   wall's cut, below), so no step bounds it. The last vertex row still keeps
   the step whose term at the band's larger radius is a quarter of the azimuth
   share, so that the band's triangles, which also span the held wall's facets,
   keep their charge near what the cut vertices already state; this is a choice
   of spacing, not a bound.
5. The rounding. A vertex's azimuth is computed from its row's ends and its
   fraction within `16π·u` of the value the bound reads, `u = 2⁻⁵³`, and the
   half-angle form puts the step within a few `u` of itself. `σ_g` as computed
   is within `2u·r` of the exact sagitta, `r` the profile's radius, which the row cap
   (`maxSectionRows`) keeps within a fixed multiple of `τ`. The rows refuse unless
   certify's margin `δ ≥ 64·ulp·ρ_g` is under `τ` (`sectionRows`), so
   `τ − σ_g ≥ τ/2` gives `C_g ≥ 2·10⁻⁷`, and the azimuths' rounding moves `Δφ`
   by at most `6·10⁻⁸` of `C_g` and the term by at most `1.2·10⁻⁷` of itself.
   That and the sagitta's `2u·r` lie inside the `2⁻¹⁶·(τ − σ_g) ≥ 2⁻¹⁷·τ`
   the step gives up (`stepCut`).

The rows' own spacing is a choice, not a bound: `arcRows` halves a gap while
its shift passes half of `C_g`, a quarter of the band's own allowance on the
band's gap, and the columns take what is left. A fillet's rows start from the
profile's sagitta `τ/4` on the arc route, where a finer profile buys coarser
columns, and `τ/2` on the chain route, whose other entries take the same rows
and step their meridian columns by the split above.

So a strip triangle between two exact rows lies within `τ` of the blend, and
every exact strip vertex takes `τ` plus slack, which holds `e_i`. An R6 rim
keeps its rows and its polygon at `τ` each, and its interior rows state `2τ`.
With each term at `τ/2`, the r = 0.2 lower rim of the radius 2 ball's slice
between z = −1.5 and −1 (`apitest/faceted_retained_blend_sphere_test.go`)
refuses SR7, "the held boundary does not close": its band and the sphere's
clipped facets share a directed edge. The same refusal comes at other slices
on either grid, r = 0.1 with the lower cut at −1.5 and r = 0.2 with it at −1.8
(`agent-docs/missing-features.md`), so keeping `τ` each only moves where that
weakness of the R6 construction shows. The `2τ` rows, and the band's slanted
cut offsets, put a later Boolean whose contact crosses the torus past the
chain-depth gate (`agent-docs/missing-features.md`). Its plane polygon states
`τ`, since every strip triangle has a corner on an interior row. The steps are computed in
floating point, as `rowStep` is; their rounding moves each term by a few units
in the last place of `τ`, inside slack.

*The band beside the wall's cut.* A triangle with a wall cut vertex `v` does not
have exact corners. `v` lies on an edge of a held wall facet `t`, which claims
its true piece within its facet bound `δ_t`, the largest bound of its three
corners (the cut edge's two ends alone do not carry that claim), so a point of
the true wall lies within `δ_t + e` of `v`, `e` the rounding of the cut. Its
surrogate `ṽ` is the setback circle's point at `v`'s azimuth (on a chain's
plane wall, the setback line's point beside it), a point of the blend's last
row:

- On a cylinder about the axis or a plane along the edge, the cut plane meets
  the wall square, so `|v − ṽ| ≤ δ_t + 2e`.
- On an R6 sphere of radius `R_s` about the height `z_S`, the cut plane `z_B`
  meets it at a slant. From `ρ² = |v − S|² − (z − z_S)²`,
  `|ρ_v − ρ_B| ≤ (δ'·(2R_s + δ') + e·(2|z_B − z_S| + e)) / (ρ_v + ρ_B)` with
  `δ' = δ_t + e`, and `|v − ṽ|` is that plus `e` (`cutOffset`).

Claim S applied to the surrogates puts the triangle within the larger offset of
its cut vertices, plus the last gap's `σ`, plus `ê_t = ρ_b·(1 − cos(Δφ_t/2))`
of its own vertices' azimuths, which come from the held wall and no step
bounds. `ρ_b` is at least the larger radius of the band's two rows, the last
row's and the setback circle's (Claim S, step 2): the arc and chain routes read
it there, a closed rim at the largest radius of any row. The other way, the surrogate band reaches every point of the patch
between the last row and the setback circle within `σ + ê_t` where `L` crosses
the triangle, or within `(1 + 1/cos β)·ê_t` beside a row's polyline, whose edge
belongs to a band triangle whose spread is at least that edge's; the real band
lies within the offsets of the surrogate one, vertex for vertex. The build
computes `offset + σ + (1 + 1/cos β)·ê_t` per band triangle, which covers both,
and raises each of its cut vertices to it plus slack (`chargeBand`); a chain's
straight entry has no azimuth term, since its surrogates lie on the setback
line, parallel to its last row. The wall's held facets beside the cut take the
raised bounds too, which only widens their own.

*A corner's sphere patch* (§4.8) is a grid of triangles whose corners lie on the
sphere of radius `r` about `C`. Every point `x` of such a triangle lies inside
the sphere, no nearer `C` than the triangle's plane, so `r − |x − C|` is at most
`r` less that plane's distance from `C`; the point of the sphere on the ray
from `C` through `x` lies that far from it, which bounds both directions. A
patch triangle reaches farther from the sphere than the rows' chords along its
sides: farther, by more than twice the sagitta of its side chords (`TestOctantDepartureCoversSamples`). The rows are
therefore added until the grid's own departure (`octantDeparture`, the grid's
shape at unit radius) times `r` is within the corner route's tolerance, and the
blend refuses (SR5) past 1,024 rows. Each patch vertex also takes its
triangles' computed departure plus slack (`chargeSpherePatch`).

**Rounding and tilt in the certificate.** `certify` runs in floating point, as
`rowStep` does. Write `u = 2⁻⁵³` for the unit roundoff and `ulp = 2u`, measure
every length from the rim's centre, and let `S = max ρ + h + Z`, `Z = max |z|`,
`ρ_lo = min ρ` and `q = min X/ρ` over the profile, all read at its two ends
since `ρ` and `z` are monotone. `ε_G = |n_G · axis|` is the end plane's tilt off
the axis, at most 10⁻⁹ (`readEnd`). The margin and the angle allowance are
(`endSection.margin`, `endSection.angle`):

    δ   = 32·ulp·S·(1 + 1/q) + 2·ε_G·Z·(1 + 1/q)
    ε_ψ = 4·ulp·(1/q + S/(ρ_lo·q²))

The length budget, term by term:

- `ρ` and `z` from `sin` and `cos`: within `4u·S` each.
- `X = √(ρ² − h²)`: `ρ² − h²` within `2ρ·4uS + 3uρ²`, and the square root
  divides that by `2X`, so `X` lies within `(6/q + 1)·u·S`. A computed point of
  the curve lies within `e = (6/q + 5)·u·S` of the exact one.
- The chord's line through two computed points moves by at most `2e` over the
  box, and the point measured against it by `e`: `3e`.
- The corners' own arithmetic (`sin`, `cos`, `hypot` on lengths at most `S`):
  `16u·S`. The distance formula: `4u·S`.
- The row vertex the route places on `G` with `ringAzimuths` and `atTheta`:
  `acos` near 1 magnifies its argument's rounding by `ρ/X = 1/q`, so the
  vertex's offset relative to the centre is within `(3/q + 4)·u·S`. Its
  rounding at the body's own scale is what `slack` charges, as for every row.
- The tilt: the route places each row's end as if `G` were parallel to the
  axis. On the tilted plane the row's circle meets `G` at an in-plane distance
  that differs by `ε_G·|z|` across and `ε_G·|z|·h/X ≤ ε_G·|z|/q` along the
  plane, within `2·ε_G·Z·(1 + 1/q)`.

The rounding terms sum to `3e + 20uS + (3/q + 4)uS = (21/q + 39)·u·S`, and
`(21/q + 39)·u·S ≤ 64·u·S·(1 + 1/q) = 32·ulp·S·(1 + 1/q)` for every `q ≤ 1`, so
`δ` covers them and the tilt.

The tangent angles do not enter `δ`. `q` is computed as `X/ρ`, with relative
error at most `δX/X + δρ/ρ + u ≤ (6/q² + 1/q + 4)·u·S/ρ_lo + u`, and
`ψ = atan(g·q)` moves by at most half that relative error, since
`x/(1 + x²) ≤ 1/2`. `g = tan(sπ/2)` from a rounded argument adds at most
`π·u/q`, since `g·q/((1 + g²q²)·sin x·cos x) = q/(cos² x + q²·sin² x) ≤ 1/q`.
With `atan` itself within `u`, the angle error is at most
`π·u/q + (3/q² + 1/(2q) + 2)·u·S/ρ_lo + 2u ≤ 8u·(1/q + S/(ρ_lo·q²)) = ε_ψ`,
because `S/ρ_lo ≥ 1` and `q ≤ 1`. Widening `[ψlo, ψhi]` by `ε_ψ` keeps the true
tangent range inside the one `certify` uses, so step 3's containment holds for
the exact curve.

A piece that certifies with `d̂ + δ ≤ τ` therefore has every exact point of the
end curve within `τ` of the chord between the placed vertices, and the vertex
bound `τ + slack` covers the rest. The budget is a hand count of IEEE
operations, not a machine proof. `TestEndSectionMarginCoversRounding` measures
two of its terms in 200-bit arithmetic: the rounding of `X = √(ρ² − h²)` and
of the placed row vertex, each against the exact point of `G` for the same
computed `ρ` and `z`, at every row and 64 points per gap, from a plane through
the axis down to `q = 10⁻⁶`. It finds each under `δ/4`. It does not measure the
rounding of `ρ` and `z` themselves, of the corners' arithmetic or of the tangent
angles. `TestEndSectionTiltCharge` measures the tilt term at `ε_G = 10⁻⁹` in
200-bit arithmetic. `δ` reaches `τ = 2⁻¹⁰·r` only for a blend tiny against its
rim, such as `r = 10⁻⁶` on a 1,000 mm bore whose end plane meets the wall's row
at `q = 10⁻³`, or for a tilted plane at `q` near 10⁻⁷, and then the rows refuse.

**An arc's rows on a cylinder end.** A row circle of radius `ρ(s)` at height `z(s)`
meets `G`'s circle at `P`, with `O` the rim's axis, `c` `G`'s axis and `d = |Oc|`
(§4.4). Write `ψ(s) ∈ (0, π)` for the angle at `c` between `c→O` and `c→P`,
`cos ψ = (d² + r_G² − ρ²)/(2·d·r_G)`. `G` unrolls to a plane, and the end curve is
`(u, v) = (−κ·r_G·ψ(s), side·z(s))`, both coordinates growing with `s`, since
`dψ/dρ = ρ/(d·r_G·sin ψ) > 0`. The certificate of §6.3 carries over with `q` in
place of `X/ρ`:

- With `q = sin∠OPC = d·sin ψ/ρ`, `du/ds = |dρ/ds|/q` and `dv/ds = |dz/ds|`, so the
  tangent angle obeys `tan ψ_t = g(s)·q(s)` with the same `g = |dz/ds|/|dρ/ds|`.
  `certify`'s parallelogram argument (steps 2 to 4) then reads `(u, v)` and this `q`.
- `cos∠OPC = (ρ² + r_G² − d²)/(2·ρ·r_G)` has the derivative
  `(ρ² + d² − r_G²)/(2·ρ²·r_G)` in `ρ`, so it is monotone in `s` while
  `ρ² + d² > r_G²` over the profile, and `endSection` refuses a profile where that
  fails ("the end curve folds back"). On a piece `q` lies in `[min(q₀, q₁), max(q₀, q₁)]`,
  or in `[min(q₀, q₁), 1]` where `cos∠OPC` changes sign in the piece (`qHi`).
- A geodesic on the cylinder is at least as long as the chord, so the unrolled
  distance is at least the 3D distance: a point of the true end curve within `d̂` of
  the unrolled chord lies within `d̂` of the cylinder's image `K` of that chord.
- The 3D chord between two placed row vertices is not `K` of the unrolled chord.
  At one fraction `λ` the heights agree, and in plan
  `|(1 − λ)·e^{iψ₀} + λ·e^{iψ₁} − e^{i((1−λ)ψ₀ + λψ₁)}| = |Σλ_j(e^{iδ_j} − 1 − iδ_j)| ≤ Δψ²/2`
  with `Σλ_jδ_j = 0` and `|e^{iδ} − 1 − iδ| ≤ δ²/2`, `|δ_j| ≤ Δψ`. Each gap therefore
  adds `curv = r_G·(Δψ + 2·ε_ψ)²/2`, `Δψ = |ψ(s₁) − ψ(s₀)|`, once per gap and not
  once per piece (`endSection.curv`, times `1 + 16·ulp` for its own rounding). A gap
  certifies when `d̂ + δ + curv ≤ τ`. `widenRows` and `endChord` reuse `certify`, so a
  widened gap's charge includes `curv`.

The margin and the angle allowance of a cylinder end (`cylEndSection`) are, with
`S = S_far`, `r_lo = min(ρ_lo, r_G)`, `q = min q` over the profile (read at its ends and
at 1 where `cos∠OPC` changes sign), `Z = max |z|` and `e_d = 16·ulp·S`:

    δ   = 64·ulp·S·(S/r_lo)·(1 + 1/q) + 2·ε_G·(Z + r_G)·(1 + 1/q)
    δ_c = (8·ulp·(ρ_hi² + r_G² + d²) + 2·d·e_d + e_d²)/(2·ρ_lo·r_G)
          + 4·ulp·S·(ρ_hi² + d²)/(2·ρ_lo²·r_G)
    ε_ψ = 2·(δ_c/q² + 6·ulp/q + 2·ulp)

The first term of `δ` covers `acos`'s `1/sin` amplification in `ψ` and in the row
vertex's azimuth (`sin φ_O = r_G·q/d`). The second covers `G`'s tilt: at height `z`
`G`'s circle is displaced in plan by at most `ε_G·|z|/√(1 − ε_G²)` and its section is
an ellipse within `r_G·ε_G²` of the circle, which moves the meeting along the row
circle by that over `q`. `δ_c` bounds the rounding of `cos∠OPC`, including the
rounding of `ρ` itself, so the computed `q` has relative error at most
`δ_c/q² + 2·ulp/q`. `ψ_t = atan(g·q)` moves by at most half that, since
`x/(1 + x²) ≤ 1/2`, `g` adds `π·ulp/q` as for a plane, `atan` itself `ulp`, and
`ε_ψ` doubles the sum. Where `ε_G` stands for an angle, `ε_G + ε_G³` is used.
`TestCylEndSectionMarginCoversRounding` measures, in 200-bit arithmetic at every
row and 64 points per gap, the rounding of `u` and of the placed row vertex against
the exact meeting, each under `δ/4`, and the computed `ψ_t` against the exact
tangent angle at the exact parameter `s`, under `ε_ψ/4`. `TestCylEndSectionTiltCharge`
measures the tilt term at `ε_G = 10⁻⁹`. `TestCylEndCertifyChargesCurvature` takes a
chord whose unrolled curve lies within `τ/2` of it and whose `curv` exceeds `4τ`,
requires `certify` to refuse it, accepts it once `τ` is raised by `curv`, and
accepts a short chord, with `curv` under `τ/4`, at `τ`.

**The cap on a cylinder end.** A cap triangle's point `Σλ_i·v_i` lies within `max e_i`
of `Σλ_i·ṽ_i`, the same combination of the true points on `G` (`e_i ≤ β_i`), and
that lies within `r_G·Δψ̃²/2` of the cylinder point `K(Σλ_i(ψ̃_i, z̃_i))`, by the
chord bound above, with `Δψ̃` the span of the triangle's true angles: at most the
held span plus `2·asin(min(1, β_max/r_G)) + 2·(ε_G + ε_G³)·Z/r_G`. The map
`λ ↦ K(Σλ_i(ψ̃_i, z̃_i))` is a bijection onto the triangle's true piece, so the bound
is two-sided. A triangle with an end-curve edge differs from the true cap by the
chord's sliver, which its row-end corners' `τ + slack` covers as on a plane cap.
Each cap triangle takes `max_i β_i + r_G·Δψ̃²/2` plus the slack, rounded up, as the
bound of its largest-bound corner. A vertex `n` beside `V` read as on `Π` (§4.4) is a rim
point, not a point of `G`: its true point lies within `β_in(n) + |n − V| + β_in(V)` of the true
`Ṽ`, which lies on `G`, `β_in` the bound the input mesh states and `|n − V|` the held distance. Its
bound in the cap is raised to that plus the slack, so the closing triangles `(n, V, b₁)`, whose
true piece is the arc of `A ∩ G` from `Ṽ` to `b̃₁`, of zero area (the fillet buries the rim point `n`, and its claim
moves to `Ṽ`), lie within their facet bound of that arc. Raising a held or shared vertex only widens
claims already made. The cap facets are new surface, cleared by `checkStripClear`
against every kept facet, the split ones included.

Claim K gains two terms for a cylinder end. The cap bounds the true solid `K`, so
`VolSymDiff` adds `ε_cap·(A_held,cap + A_true,cap)`, `ε_cap` the largest corner bound of
the cap's facets. Each held facet `t` of `G` split by a fan (§4.4) differs from its
fan by a region within `max_p dist(p, uw)` of `t`'s plane over `t`'s projection, so
`VolSymDiff` adds `Σ_t max_p dist(p, uw)·(area(t) + Σ area(fan of t))`, rounded up.
The held area change of the fan enters `measureChange`, because `t` is removed and the
fan emitted in the same pass. The closing triangles' sliver is charged through `ε_cap`, which
takes their corners' raised bounds.

**Volume.** The new `VolSymDiff` is the old one plus `ε·(A_strip + A_true)`, with
`ε` the largest vertex bound of the strip's facets, the band's raised cut
vertices included, and `A_strip`, `A_true` the held and exact areas of the
blend face (with the wall and plane area charges below added to `A_true`), plus
`Σ_t (h_t + δ_t)·area(t)` over the re-triangulated plane faces' facets (Claim K),
each term rounded up. Two surfaces within `ε` of each other
enclose at most `ε` times the larger area between them. A cap on a cylinder end
keeps its own term (The cap on a cylinder end).

**A displaced plane in the area bound.** Every exact area of `Δ_true` (the regions
`A` and `B` lose, the blend face, each end cap) is computed on the tags. When a
plane the route reads states `AxialDelta` δ (any plane it reads: `A`, `B`, an end
plane `G`, a mitre or corner face), the true pieces' boundaries lie within the
route's plane charge `c` (built from `planeCharge`, `4δ/minSine` per plane, as the
route builds its slack) of the tags' pieces, each in the
piece's own coordinates: a corner or rim moves by at most `2δ/det` per plane with
`det ≥ minSine` (R1's `intersect3` and `|d·n_G|` gates; R3's and R5's end planes
are normal to the axis), an R6 rim's radius `√(R_s² − z_s²)` moves by at most
`|z_s|·δ/R_rim ≤ δ/minSine` under its crossing gate, and an R4 or chain arc whose
end plane states a displacement refuses unless every row meets that plane at a
sine of `minSine` or more ("an arc's rows meet a displaced end plane below
`minSine`"), since a row at sine `q` moves its end by `δ/q`. A closed rim (R2)
only moves along its axis with its plane, so its areas do not change and it adds
nothing. An oblique
end plane (§6.1) adds, for its cap, `θ²·(A_cap + 2·P_cap·c + (π + Φ_cap)·c²)`, the tilt's area term. So the area
slack adds, per changed piece `g`, `2·P_g·c + (π + Φ_g)·c² + A_g·c/r_g` (the form of
boss-root-overflow §5.5.6), with `P_g` an upper bound on the piece's perimeter
from its exact curves, `Φ_g` its total turning (at most `2π` plus each arc's
span), `A_g` its exact area and `r_g` its smallest radius of curvature (no term
for a planar piece). It is added on top of the rim's or arc's wall charge. The
term reads the route's whole charge `c`, which also holds a charged wall's and,
on a chain, every held face's charge; charging more is safe. With no charge it is
zero.

The `Verify` and `VerifyAll` results come from the faceted audit of the rebuilt
body, not from these terms.

### 7. Receiver and dispatch

This section extends reach RX6's receiver row and narrows SX9.

| Receiver | `Fillet` / `Chamfer` |
|---|---|
| `facetedPayload` whose selected edges are all Table RR edges | this document's route, applied one edge at a time to the evolving mesh. Each step reads the previous step's result as an ordinary retained body; the selection resolves up front to `(face pair, curve)` records, not mesh indices |
| any other `facetedPayload` call | SX9, unchanged |

A `Chamfer` whose selected edges are all R7 saddles takes §4.9 instead: one Cut
by a tool with a ring per saddle. A `Fillet` whose selected edges are all
saddles of the R7 fillet row takes §4.10, one mouth at a time on the evolving
mesh.

The result is a `facetedPayload` with retained tags. A further `Fillet`,
`Chamfer` or Boolean takes it like any retained body, and a placement carries
its tags by retention §3.4. `Shell` stays SX9. An asymmetric chamfer takes the
reference-face option of modify §7: SX3 resolves one reference face per edge,
and each step finds it again on the evolving mesh by its retained tag
(Part M §4). A missing or ambiguous one
is reach SX3.

### 8. Readers

A blend result is a retained body, so every retention §4 and §10.3 reader row
applies unchanged. Two readers see something new: `FaceCreatedBy` finds the
blend face by its role, and the edges between the blend and `A` or `B` are
tangent junctions, the kind a tangent-chain selection expands (reach SX2).

### 9. Tests and the red leg each must show

Tests assert on computed geometry. Volumes are compared with the result's
published bound, never with a pinned literal.

| Test | Asserts | Red leg |
|---|---|---|
| Hole mouth in a placed-tool plate (R2 fillet, convex) | blend face is a `Torus` with `Major = R + r`, `Minor = r`; removed volume equals `π·r·(2Rr + r²) − 2π·((R + r) − 4r/(3π))·(π·r²/4)` within the bound; both setback edges are `Circle3` | build the cone for a fillet: the surface kind is wrong |
| Boss root fillet (R2 concave) | the volume grows by the same expression with the tangent circles at `R + r`; the blend face has `Reversed = true` | use the convex sign: the blend removes material |
| Shaft end fillet and chamfer (R2) | `Major = R − r`; `R − r ≤ 0` refuses with `ErrDegenerate` | drop SR3: the torus has a negative major radius |
| Straight 90° edge (R1 fillet and chamfer) | `Cylinder`, axis along the edge, removed volume `L·r²(1 − π/4)`; the chamfer removes `L·d²/2` | |
| 45° edge (R1 fillet) | setback `r·tan(φ/2)`; removed area per unit length `r²(tan(φ/2) − φ/2)` | use `r` for the setback at `φ ≠ π/2`: the 45° test fails |
| Plane to cylinder edge (R3): a groove's top edge, fillet and chamfer | the fillet's centre and tangent points are those of the cross-section; the volume change equals `L` times the area of the corner region, and the surface changes by the cylinder gained less the plane strip, the wall band and a corner of each end face | take `κ` the other way: the ball sits on the wrong side of the wall |
| Plane to cylinder edge at a boss's foot | the plate's top face is reflex at each end; the fill adds `L` times the corner region's area, and `VerifyAll` passes | |
| Keyway top edges in a turned shaft and a hub (`apitest/faceted_retained_blend_keyway_test.go`): an R = 10 shaft 80 mm long with a keyway `|y| ≤ 3` above z = 6.5 from x = 20 to 60, one from x = 20 running off the shaft's end, one from x = 20.37 to 60, and the 20–60 keyway with a Ø0.6 hole along z at x = 70 whose rim has a vertex on the r = 0.5 fillet's setback line; a hub with a bore r = 5 and a keyway `|y| ≤ 2` from x = 10 to 30 | the shaft runs on past each end wall, which is convex at the corner and takes the cap. Fillet r = 0.5 of both edges removes `2·40·A`, with `A` = 0.117968684 mm² the corner region (its closed form agrees with a sampled polygon), the run-off keyway `2·60·A`, Fillet r = 0.3 of the 20.37 keyway `2·39.63·A(0.3)`, Fillet r = 0.3 and 0.1 of a keyway from x = 20.37 running off the shaft's end, whose rows `clearCusp` spaces again, `2·59.63·A(r)` with `A(0.3)` = 0.040674574 and `A(0.1)` = 0.004336328 mm² (each agreeing with a sampled polygon), a point 0.05 and 0.01 mm inside the corner region lying on the cap at x = 20.37, both meshes passing `VerifyAll` and `Verify` reading both Sound, the holed shaft `2·40·A` as without the hole, and Chamfer d = 0.5 of one edge `40·A_c`; the hub's bore holds its material outside the cylinder, its ball lies in the hub and its region (agreeing with a sampled polygon) is the quadrilateral less the sector and less the bore's segment, removed over `2·20`; the area changes by the blend face less the wall's strip and the cylinder's band, plus a cap at each end inside the cylinder and less a corner at the shaft's end disc; a point 0.125 mm inside the corner region lies on each end wall and more than 0.1 mm off the disc; `VerifyAll` passes. A cavity crossing the cap refuses "the cap an end face takes meets …", a pin hole into the end wall within the blend's reach refuses SR5, the cross hole moved to x = 40, inside the band, refuses SR5 on the setback line, Fillet r = 0.007 of a keyway from x = 20.37 running off the shaft's end refuses "the blend's tangent point does not lie on the wall's face", and the top edge of a blind groove in a plate, whose plane is reflex at its ends, refuses SR5. Unit tests (`internal/retainedblend/line_rewrite_internal_test.go`, `cyl_rewrite_internal_test.go`) splice the rows, the wall's run and `V` into an end face's loop, refuse tag directions whose turn is undecided or reflex while the held vertices show a convex corner, refuse a plane's or wall's chain whose sense vertex lies within its bound of the corner, charge `A` against the strip and band with the sign of each end's kind (`cylRemovedArea`), refuse a cap loop that crosses itself or runs straight on at the band's corner, and space a shaft's and a hub's rows so the last row before `tb` lies `need` from the circle, every other gap within the profile's step, a need the rows already meet leaving them and one past the plane's tangent point refusing (`TestClearCusp`) | clip the wall by angle alone: "a wall facet reaches across half the cylinder"; read every wall vertex against the generator: the x = 70 hole refuses; drop the caps from the strip-clear check: the cavity builds; never space the rows again: the r = 0.3 and 0.1 run-off keyways refuse SR5 "the blend's end cap folds …"; space them by `τ` and the slack alone, without the wall's reach: r = 0.3 refuses the same; drop its crossing or its turn test: the matching unit case returns no error; drop the wall's run from the splice: the end face fails to triangulate; drop the reach check: the pin hole builds; drop the reflex-plane check: the groove builds; read the turn from held vertices: the unit cases return no error; drop either sense bound: its unit case returns no error; charge a cap where the wall extends as a corner given up: the area unit case fails |
| The keyway cusp's last gap (`retained_blend_facet_bound_test.go` `TestRetainedBlendKeywayCuspBounds`, `internal/retainedblend/cyl_rewrite_internal_test.go` `TestCuspChargeCoversSamples`): Fillet r = 0.3 and 0.1 of the keyway top edge at y = 3 from x = 20.37 running off the R = 10 shaft's end | every blend facet, sampled on a 12×12 barycentric grid against the exact cylinder of radius r about (3 + r, √((10 − r)² − (3 + r)²)), lies within its own bound and `Mesh.Bound`, and 12,139 exact points of each fillet (61 columns of 199 arc points), denser near `tb`, within the nearest blend facet's own bound; every facet of the end planes x = 20.37 and x = 80 near the corner, sampled the same way against the exact end face, whose boundary there runs up the wall, along the fillet's arc and along the circle, lies within its own bound and `Mesh.Bound`, and 2,001 exact points of the arc in each end plane within the nearest end facet's own bound. `TestCuspChargeCoversSamples` samples the segment from `P` to `C'`, moved `ε` from `C` in 16 directions, and the arc `PC` at chord half-angles 0.02, 0.07 and 0.3 on a circle of radius 0.3 and `ε` from 0 to 8 sagittas: each lies within `cuspCharge` of the other, and past `ε` wherever `ε` is under four sagittas | charge the cut row with its own bound alone (`cuspCharge` returning `ε`): `TestCuspChargeCoversSamples` fails at `ε` = 0; dropping the charge leaves both keyway fixtures within their facets' own bounds, since their cut vertices lie nearer the exact setback line than their own bounds state, so no keyway fixture shows that leg; the `σ²/(2r)` term is shown by no sample past rounding |
| Convex side edges of a boss at its foot (`apitest/faceted_retained_blend_boss_foot_test.go`): a 20×20×15 box boss on an 80×60×10 plate with a placed through hole; a 45° corner, its slant sketched so its tag states no departure (retention §3.5), on a boss sunk 1 mm into the plate because the shifted retry re-normalises a slanted frame and states a departure; a pillar between two plates. The square bosses stand on the plate and take the shifted retry | the plate's top face is reflex at each foot and takes the cap. Fillet r = 3 of the four sides removes `4·15·r²(1 − π/4)`, Chamfer d = 2 removes `4·15·d²/2`, the 45° corner `15·r²(tan(φ/2) − φ/2)` with `φ = 3π/4`, the pillar `15·r²(1 − π/4)` with a cap at each end; the area changes by the blend face less the two strips, less the corner each convex top end gives up, plus each cap a reflex end takes (top and foot cancel on the boss; the pillar gains two caps), and the bound's growth stays under twice the caps' area; `VerifyAll` passes on the box fillet. A cavity crossing the cap refuses "the cap an end face takes meets …", and a hole within the blend's reach on the plate's top refuses SR5. The wall edge of an L section whose front face is reflex at the step loses `20·r²(1 − π/4)`, and the plate's top takes the cap. A unit test (`internal/retainedblend/line_rewrite_internal_test.go`) reads `G`'s turn from tag directions whose sine is under `minSine` while the held vertices show a square corner: it refuses as undecided | build no cap: the boss refuses SR5; drop the cap's facets from the strip-clear check: the cavity builds; charge the cap as a corner `G` gives up: the bound's growth passes twice the caps; read the turn from `G`'s loop vertices beside `V`: the undecided unit case returns no error |
| Concave edge in an L section | the fill adds `L·r²(1 − π/4)`, the blend's normal points into the void, `VerifyAll` passes | build the convex blend: the normal points into the material |
| Convex edge whose front face is reflex at its top (`apitest/faceted_retained_blend_reflex_side_test.go`): an outline in the XZ plane extruded 6 mm along y, joined to a block that crosses its top so the Union takes the mesh path; its right side x = 10 rises from z = 0 to (10, 10), where the front face y = 0 turns onto a flat underside or onto one at 45° | Fillet r = 1 of the edge x = 10, y = 0 removes `10·A` under the flat underside and `A·(10 − c)` under the sloped one, `A` the corner region and `c` its centroid's offset from x = 10; the area changes by the cylinder less the two strips, less the floor's corner, plus the cap the underside takes (`A/|d·n|`), and the bound's growth stays under twice the cap; the corner (10, 0, 10) is no longer a vertex and `P_A` is; the end curve on the 45° underside lies within the mesh's bound; `VerifyAll` passes. A cavity crossing the cap refuses "the cap an end face takes meets …"; an underside rising at slope 10 or 12 from the corner meets the setback line at or below the floor and refuses "a row of the blend's strip is no longer than its bounds along the edge" | charge the cap as a corner the underside gives up: the bound's growth passes twice the cap; charge it as zero: the growth passes the cap (as for the L section's wall edge and a shell opening's inner rim); keep the cap out of the strip-clear check: the cavity case fails its held audit instead of naming the cap; drop the row check: the steep undersides refuse on the end face's reach instead |
| Concave edge at a wall's foot | the plate face is reflex at each foot corner; the fill adds `L·r²(1 − π/4)`, the wall's end faces grow by the caps, `VerifyAll` passes; two foot edges together and a chamfer build as well | keep `V` out of the extended end face's loop: the mesh fails its audit |
| Oblique end face, sketched at 45° about z and, separately, about y (no exact tool leans both ways) | the end arc is an `Ellipse3` with `SemiMinor = r` and `SemiMajor = r / |d̂ · nG|`; removed volume equals the section area times the length between the end planes measured through the section's centroid | treat `G` as normal to the edge: the volume is off by the centroid offset |
| Rows ending on oblique planes (`apitest/faceted_retained_blend_oblique_end_test.go`): the 45° end face above at r = 2, a two-edge mitre run at r = 3, and the 45° plane of a two-edge corner on the corner route at r = 2 | 1,001 samples of the true end curve, the ellipse the cylinder cuts in that plane, lie within the mesh's `Bound` of its facets. A unit test (`internal/retainedblend/line_rewrite_internal_test.go`) projects rows onto a plane with `|d̂ · n| = 1/2`: the chord centred on the stretch direction has sagitta `r·(1 − cos h)/c` there and no other chord's is larger; spaced for the cross-section alone that is `τ/c`, spaced by `rowStep` it is `τ` | space the rows by `τ` alone in any of the three routes: its curve lies past `Bound` from the mesh; drop `c` from `rowStep`: the unit case reads `τ/c` |
| Rows ending on an arc's offset end planes (`apitest/faceted_retained_blend_arc_end_plane_test.go`): a Ø8 bore whose axis lies `h` from the plate's face `x = 0` breaks out through it, fillet r = 4 with the wall's row meeting `x = 0` at `X/ρ` = 0.4 and 0.2 (arc route); the rounded box corner cut back so the plane's row meets the cut at `X/ρ` = 0.1, fillet r = 2.5 along the tangent chain (chain route); a pocket open through `x = 0` whose 3 mm corners' axes lie 2.5 mm and 2.75 mm from it, fillet r = 2 and r = 1 along the mouth's chain (chain route, hole side); the hole mouth chamfered with d = 1, 2 and 4 at `X/ρ` = 0.4 and 0.2 | 1,001 samples of each true end curve, taken from the torus, lie within the mesh's `Bound` of its facets; every facet of the chamfer, the band beside the wall's cut included, sampled on a barycentric grid against the cone's generator in closed form, lies within `Bound` of the cone. Unit tests (`internal/retainedblend/arc_section_internal_test.go`) pin `tan ψ = g·X/ρ` against finite differences of a sampler that bisects each row's circle against the plane; on a plane through the axis, where the end curve is the tube's circle, `certify` refuses a chord whose sagitta `r·(1 − cos(Δα/2))` is 1.01 `τ` and accepts one at 0.9 `τ`; a hole mouth's torus and cone at `X/ρ` = 0.2 and a boss's torus at 0.1 keep every sampled chord within `τ` with at most 96 rows, the rows that hold the ends' azimuth shift included, where the profile's own rows stray past it; a plane through the axis adds no rows; a tiny `τ`, and a fillet of 10⁻⁶ on a 1,000 mm bore whose margin `δ` passes `τ`, refuse past 1,024 rows; a plane the setback circle misses refuses; `TestEndSectionMarginCoversRounding` measures the rounding of `X` and of the placed row vertex in 200-bit arithmetic under `δ/4` down to `q = 10⁻⁶`; `TestStripRowStepsHoldTau` sizes the rows and the steps `rowSteps` gives a hole mouth's cone and torus at the sine 0.2 and a boss's torus at 0.1: every gap between two vertex rows keeps its sagitta plus the azimuth term of its larger step and its shift within `τ`, the worst gap within 10⁻⁴ of it, and every row keeps its near-end term within `τ`; `TestAzimuthStepHoldsBudget` keeps the step's term within its budget and within `2·stepCut` of it down to a budget of 10⁻¹⁴ of the radius; `TestEndSectionTiltCharge` measures a plane tilted 10⁻⁹ off the axis in 200-bit arithmetic within the margin's tilt term and refuses where that term passes `τ` | space the rows by the profile alone: the curves lie past `Bound` from the mesh and the unit cases stray past `τ`; drop the parallelogram's corners from `certify`: the 1.01 `τ` chord certifies; set `δ` to zero: the rounding test and the margin refusal fail; drop each gap's shift from `rowSteps`: the root test's hole mouth chamfer at d = 2, c = 0.4 lies past its facets' own bounds from the cone, and `TestStripRowStepsHoldTau` reads more than `τ` on the cone, the hole's torus and the boss's; read every gap at the strip's smallest radius: that test reads more than `τ` on all three; drop a gap's sagitta from its budget, or the cap near each row: that test fails, and no sampler here does: the hole mouth's sagitta and azimuth terms push opposite ways, and the other limit on each step still binds; drop the tilt term: the tilt test fails |
| Strip triangles (`apitest/faceted_retained_blend_strip_interior_test.go`, `retained_blend_facet_bound_test.go`): the rounded box corner cut back so the plane's row meets the cut at `q` = 0.4 and 0.2 for r = 2 and at 0.4, 0.2 and 0.1 for r = 2.5, filleted along the tangent chain (`κ = −1`); the hole mouth that breaks out through `x = 0`, filleted with d = 1, 2 and 4 at `X/ρ` = 0.4 and 0.2; bosses of radius 3 and 4 on the plate, their top rims filleted with r = 1 and 2.5, and of radius 1 at r = 0.01; a boss of radius 9 filleted at r = 0.004 and chamfered at d = 0.003, and an Ø18 bore's mouth filleted at r = 0.002; a groove's top edge (R3) and a bore pair's ridge (R5) filleted with r = 1; the cap of the radius 2 ball below z = −1, its rim filleted with r = 0.4; three edges at a mesh box's corner filleted with r = 1, 2 and 3 | every facet of each blend face, the band beside the wall's cut included, sampled on a 12×12 barycentric grid against the fixture's profile or sphere in closed form, lies within the mesh's `Bound`; the root test reads each blend facet's own bound, the largest vertex bound of its corners, and every sample of the boss rims at r = 1 and 2.5, the boss corner at r = 2.5 and `q` = 0.4 and its chain chamfer at d = 2.5 and `q` = 0.2, the hole mouth at d = 4 and 0.2 and its chamfer at d = 2 and 0.4, the sphere rim, the r = 2 corner, the radius 9 boss's chamfer at d = 0.003, the groove and the ridge lies within it. Every blend facet's normal agrees in sign with the exact surface's outward normal at its centroid, the smallest dot product logged. The radius 9 boss at r = 0.004 and the Ø18 mouth refuse past 4,096 columns; `TestColumnCountRefusesPastTheLimit` builds exactly 4,096 and refuses one more, and refuses a step that is zero, negative or not a number. `TestRetainedBlendStripPatchCovered` samples the exact torus of the hole mouth's fillet at d = 4 and 0.2 and of the boss corner's at r = 2.5 and 0.4 against the blend's facets: each point lies within the nearest facet's own bound of it. `TestOctantDepartureCoversSamples` samples the corner patch's grid at 4, 17 and 51 rows: no sample lies past `octantDeparture`, the farthest comes within a tenth of it, and it passes twice the side chords' sagitta | count a fan's columns from its wall row alone: the boss corner's chain chamfer lies past its facets' own bounds from the cone; drop both each gap's shift and the cap near each row from `rowSteps`: the hole mouth's fillet and its chamfer at d = 2 lie past their facets' own bounds from the torus and cone, the boss corner's chain chamfer does too, and a point of the hole mouth's torus lies past the nearest facet's own bound; read the band's azimuth term at the smaller of its two radii: no fixture here fails; give each of a closed rim fillet's two terms all of `τ`: the boss rims fail the root test, their facets past their own bounds, while `Bound` alone still covers them, held up by the band's cut vertices; drop the band's charge: the sphere rim's band lies past `Bound` from the torus; size the corner's rows by the strips alone and drop the patch's charge: the patch lies past `Bound`; size them so and keep the charge: both tests pass and the corner boxes' volume bounds pass 0.1 and 0.5 (`apitest/orthogonal_fillet_corner_test.go`); drop the shift from the rows' spacing (`arcRows`): the hole mouth's chamfers and the boss corner's chain fillets refuse with "the blend's strip has no column step left …", their rows' shifts past their gaps' budgets, and `TestStripRowStepsHoldTau` fails; state `2τ` on every fillet row instead of splitting `τ`: `TestFacetedSourcePatchBlendThenCut` refuses at the chain-depth gate; clamp the column count at 4,096 instead of refusing: the radius 9 boss and the Ø18 mouth build, their facets past their own bounds; read a curved wall's cut vertex from its edge's two ends alone: no fixture here fails, since their held facets' corners carry equal bounds |
| Grazing-angle and direction gates (`TestGatesRefuseNaN`, `internal/retainedblend/line_internal_test.go`) | each gate refuses a value that is not a number, as a tag whose normal is not a number produces: `shallow` (the straight edge's end, mitre and corner planes, and the cylinder–cylinder walls' meeting), `intersect3`, `grazes` (a plane–cylinder edge's end plane), `clears` (the cylinder–cylinder walls' ways out of a corner), `crossing` (a sphere rim's plane) and `cornerTurn` (a chain end's turn reads undecided) | write any of them as `c < minSine`, `≤` or `>` against the bound: NaN passes and its subtest fails |
| Clearance | a radius past the face width refuses `ErrDegenerate`; a radius within `b` of it refuses `ErrUnsupported` | drop the `− b_v` term of §6.2: the near case builds an invalid solid |
| Shared vertex / tangent chain | refuse SR2 | |
| Top loop of a mesh box, `Fillet` r = 2 | mitre run (§4.7): the volume is the box less the columns' removed height, the larger of the two walls' at each column, integrated on a 1000 × 1000 grid; the section 0.1 mm under the top has its corner on the plane x = y; `Verify` is `Sound` | delete the mitre and the corner volume is the sum of both strips |
| Two or three top edges of a mesh box in a row | a two-edge run matches the same integral; both free ends on one face, or three edges at a corner, refuse SR2 | |
| A pocket wall inside the strips' reach | refuse SR2 | |
| Oblique plane/cylinder rim | refuse SR1 and keep the `Ellipse3` of retention §10.5 | |
| Plane-sphere rim (R6): a Revolve ball cut by a placed box, fillet and chamfer | the removed volume equals the zone of the sphere down to `zB` less the solid under the torus (or the frustum), from the meridian's closed form; the area matches the cap's less the annulus and the zone plus the blend; the `Torus` has `Major = ρc` and `Minor = r`, the `Cone` starts at the narrower setback circle; `VerifyAll` passes | drop R6 from dispatch: SX9 |
| R6 two-distance chamfer, and both rims of a twice-cut ball | the plane takes `dA`, the sphere's meridian `dB`; the second rim is found again on the partly blended body | |
| R6 refusals | a snapped centre, a rounded radius and a rotated revolve keep the rim's chords (retention E12) and refuse SR1 as a tag not proven to be exactly its revolve record; a ball too large, a plane setback past the axis and a sphere setback past the pole refuse SR3; a grazing crossing refuses SR5 (unit test) | drop E12's denoted arm: each of the three publishes a `Circle3` rim; drop its sphere-radius test: the rounded radius does; delete an SR3 or SR5 check: its fixture builds |
| R6 inner wall vertices | a sphere zone that holds tessellation vertices builds | gate inner vertices as R2 gates the cylinder's: "the blend consumes the wall" |
| R6 kept inner vertex bound (unit test) | a vertex held `β` outside the sphere off a point inside the zone gets a bound that reaches the nearest point of the new boundary | keep its old bound: the bound falls short |
| R6 pocket and dome foot | `Cut(slab, ball)` (convex, plane outside the rim, material outside the sphere) and `Union(ball, slab)` (concave, sphere rising): fillet and chamfer change the volume and the area by the meridian's closed forms; the torus and cone parameters match and the blend's normal points away from the material | flip the chamfer normal's sphere term for an outside material: the cone's normal points into the material |
| Area slack carries the shortfall of the face terms | the fillet's area change lies within the bound's growth | drop the shortfall: the rebuilt bound grows by less than the change's error |
| Shortfall by group (`internal/retainedblend/route_internal_test.go`, `internal/facetedrecord/build_test.go`) | a group whose old terms exceed its new ones by an exact half charges at least that half to `AreaSlack` and the area bound, a group the new body lacks charges all its old terms, the lower sum stays at or below the exact sum, `ChargeAreaSlack` raises every face's and the body's bound by at least the charge, and a difference that is not a number, as of two infinite terms, charges `+Inf`, which the build refuses | drop the charge, the group the new body lacks, the lower sum's step down, a face's or the body's charge, or the `+Inf` charge: each test fails |
| Four boss side fillets, then a rim chamfer, on one plate (`apitest/faceted_retained_blend_area_charge_test.go`): the 20×20×15 box boss, a counterbored M6 hole and a placed round boss of radius 6 on an 80×60×10 plate, r = 2, then d = 1 | each volume and area equals the hand value within its bound; `Verify` reads `Sound` with the area within its tolerance | charge each rewritten face's whole old term: the plate's top charges its term once per side edge and the area bound passes its tolerance |
| Result is an ordinary retained body | a second blend on another edge, a `Cut` through the blended region and `Transform` all build; `NormalAt` on the blend face has a bound below 1 | store the blend with `Retained = false`: the chain loses the tag |
| New vertex bounds | every new vertex lies within its stated bound of its exact point | skip the rounding charge: the bound is zero and the check fails |
| Verify | `Verify` of a blended plate passes with no `Suspect` in watertightness | |
| R7 cross-hole chamfer: Revolve shaft R = 10, pin r = 3 through it, both mouths, d = 1.5 and 0.5; pin r = 7, d = 0.5 (`apitest/retained_saddle_chamfer_test.go`) | the volume falls by twice the wedge's divergence-theorem integral per mouth (`saddleWedgeVolume`) within both bodies' bounds, and at d = 1.5 that change exceeds the bounds; the area falls by twice the shaft's and the bore's strips less the band, from the same parametrisation, within both bodies' bounds, and at r = 3 that change exceeds the bounds and `Verify` reads the result Sound with the area within its tolerance; the walls stay `Cylinder`, the bands read `Faceted`; on the `VerifyAll` mesh the wedge's section centroid reads outside, farther than the mesh bound, and a point 0.3 mm past the band reads inside, at 8 stations of each mouth; an Extrude shaft at d = 1.5 under the three axis assignments and a translation does the same (`TestRetainedSaddleChamferExtrudeShaft`; under `-race` the translated case only, on one mouth). A unit test (`internal/crossbore/saddle_tool_test.go`, R = 10, r = 3, d = 1.5) checks that the chord bound covers sampled points of the held band triangles against the exact band at the same parameters, that each row's area interval holds a midpoint-rule area of the exact row and the band's interval is narrower than 1 % of its area, and that a moved inner row raises the sample error | cut with `d/2`: the d = 1.5 volume and the d = 0.5 centroids fail; drop the outer foot's rotation from the held stations: the sample comparison widens `ε` and proof 4 refuses; drop the mouth-side flip or permute the pose's axes: the race-mode Extrude case refuses; drop the twist term from the chord spread: the sampled triangles lie past the bound; charge the whole twist (`S` in place of `S/M`): `Verify` reads both r = 3 results Suspect; split no quad into area cells: `Verify` reads the d = 1.5 result Suspect and the band's interval is too wide; give every row the whole band's density: row 0's interval misses its area; compare only `O⁺` and `B⁺`: the moved inner row goes unread |
| R7 fillet: Revolve shaft R = 10, pin r = 3 through it, both mouths, ρ = 1.5 and 0.5; pin r = 7, ρ = 0.5; a `Hole` r = 3 through the shaft's wall, ρ = 1.5 (`apitest/retained_saddle_fillet_test.go`) | the volume falls by twice the tube-coordinate integral of §4.10, computed by Gauss–Legendre over exact ray exits (18.593228 mm³ per mouth at ρ = 1.5, 1.635978 at ρ = 0.5), within both bodies' bounds, and at ρ = 1.5 that change exceeds the bounds; the walls stay `Cylinder`, the fillets read `Faceted`; `VerifyAll` verifies the result; on its mesh a point midway between the arc and the edge along the arc's bisector reads outside, farther than the mesh bound, and a point 0.3·ρ inside the ball reads inside, at 8 stations of each mouth; an Extrude shaft at ρ = 1.5 under the three axis assignments and a translation does the same (`TestRetainedSaddleFilletExtrudeShaft`; under `-race` the translated case only, on one mouth). Unit tests (`internal/saddlecanal/proof_internal_test.go`, both sides of the larger cylinder; under `-race` one canal of each): the cell proofs' area intervals hold a midpoint-rule area of the fillet and Green's-theorem areas of both walls, the chord bound covers sampled held triangles, the rows split the widest arc, a moved held vertex raises the sample error, and ρ = 3.4 folds; `internal/retainedblend/saddle_fillet_internal_test.go` chains a cut ring through a merged sliver, reads a pose's hand, checks that the chamfer's recognition admits exactly the edges its rule before σ admitted, and that the fillet's admits its three patterns, over every pair of radii, `Reversed` flags and `Convex` flag; a second fillet of a rebuilt r = 3 shaft at ρ = 0.5 tessellates to the same vertices and triangles | fillet with ρ/2: the ρ = 1.5 volume and the ρ = 0.5 points fail; flip σ1 in the route: every build refuses, "the bore's cut leaves fewer than three vertices"; drop the strip's winding swap for a left-handed pose: the Revolve result fails its audit; drop the mouth-side flip: the race-mode Extrude case puts the fillet on the far mouth and its points fail; drop the Jacobian's sign or σ1 in `G`: the outside unit canals refuse, "folds" and "is not embedded" |
| R7 fillet outside both cylinders (`TestRetainedSaddleFilletOutside`): a block 80 × 60 × 20 with a radius-5 `Hole` down through it crossed by a radius-3 blind `Hole`, both mouths, ρ = 0.5 (convex, removes); a radius-5 `Boss` 8 high on a radius-15 rod, ρ = 1 (concave, adds); a radius-3 pin joined through the R = 10 Revolve shaft, both roots, ρ = 1.5 (concave, a left-handed pose) | the volume falls (block) or grows (rod, pin) by the tube-coordinate integral per mouth (0.540755, 4.701377 and 6.619077 mm³) within both bodies' bounds; `VerifyAll` verifies the result; on its mesh the point midway between the arc and the edge reads outside for the block and inside for the rod and pin, farther than the mesh bound from every facet, and the point 0.3·ρ inside the ball keeps its side, at 8 stations of each mouth; under `-race` the rod only | expect ρ/2 or σ1 = −1 in the test: all three fail, at the ball's points or the volume; flip σ1 in the route: every build refuses, "the bore's cut leaves fewer than three vertices" |
| R7 fillet refusals | `ρ = 3.5` (`2ρ = R − r`) refuses `ErrDegenerate` "no ball of that radius touches both walls of the saddle"; ρ = 3.4 refuses "folds: the pipe is not regular there"; a pin moved 0.5 off the shaft's axis, the pin's part inside the shaft (`Intersect`, σ = (−1, −1)) and a concave pin root inside a tube's bore (`Chamfer` and `Fillet`) refuse with the canal-surface reason; a flat milled beside a mouth refuses "another face of the body comes within the saddle fillet's reach", as does a boss 1 high on the radius-15 rod at ρ = 1, whose top face lies inside the bore's cut (`TestRetainedSaddleFilletRefusals`) | drop the route's existence test: ρ = 3.5 refuses at `saddlecanal.Prove`'s own, "the saddle canal's offset cylinders do not meet in a loop" (a unit test pins that site's text); drop the Convex flag from `saddleKey.built`: the tube root's `Chamfer` builds and removes material where a concave blend adds it, and both recognition unit tests fail; drop the Jacobian test: ρ = 3.4 refuses at the progress test instead; drop the clearance check: the flat refuses later, "a corner of the saddle fillet's window lies on no facet of the shaft wall", and the short boss "the bore's cut leaves fewer than three vertices"; admit every σ: the tube root's `Fillet` builds and the clipped pin refuses "a corner of the saddle fillet's window lies on no facet of the shaft wall"; drop the exact axis meeting: the moved pin refuses with "a held seam vertex is off the two cylinders" |
| R7 refusals | `d > r/2` refuses; a pin moved 0.5 off the shaft's axis, a pin with r/R = 0.75 and a pin joined through the shaft refuse with the canal-surface reason; a flat milled beside a mouth refuses "another face of the body comes within the saddle chamfer's reach"; `Chamfer` of a pin's roots and of a boss's root on a rod keeps the canal-surface reason (`apitest/blend_refusal_names_test.go`) | drop the clearance check: the flat builds; admit a bore that is not `Reversed`: the joined pin builds; drop the exact axis meeting: the moved pin refuses with "a held seam vertex is off the two cylinders"; drop `2r² < R²`: r/R = 0.75 builds |
| Arc whose wall extends past its end planes (`apitest/faceted_retained_blend_arc_extends_test.go`): a round bar r = 3 along x with a 6 × 2 box notched into its underside, a tube (bore r = 3, outer r = 6) with the same rib standing on its bore's underside, so the plane lies outside and inside the arc's circle, and a rod r = 5 standing beside a plate's corner, whose top meets it in an arc between the plate's two perpendicular faces; fillet r = 1 and chamfer 1 of each arc, a fillet r = 0.3 of the tube, r = 0.1 of the bar and r = 0.2 of the rod | the result is a solid and `VerifyAll` and `Verify` pass; the volume gains the corner region swept between the two end planes, `∫∫ ρ·Δφ(ρ)` over it, within the bound's growth; each end plane's face gains the cap, `∫∫ ρ/√(ρ² − h²)`, within the area bound's growth; the blend face is one torus with `Major = R + κr`; the bound grows by at most `2τ` past the held one at size 1, as a hole mouth's arc does | drop the local side: F1's text; cut the wall by height alone: the wall behind the end face and past the side faces is deleted |
| Arc whose wall extends, refusals (same file) | a 0.5 mm hole through the bar inside the band refuses "another part of the wall lies within the blend's reach of the edge"; r = 2.5 refuses "the blend consumes the plane beside the arc"; a 0.1 mm cube floating across the side face's plane inside the corner region and clear of the strip refuses "the cap an end face takes meets another facet of the body" | skip the wall gate: the hole is consumed unseen; clear the cap by the strip alone: the cube is accepted |
| R4 local side, band and cusp (`internal/retainedblend/rim_internal_test.go`, `arc_section_internal_test.go`) | the side comes from the facets along the rim that agree, undecided within the bound refuses, disagreeing facets refuse, a closed rim keeps the whole-face text; a chord through, within the widening of, and clear of a band; `cuspParam` puts the end curve `need` from the generator, `widenRows` keeps the earlier gaps certified and widens only the last, `endChord` bounds its chord | |
| R4 end on a cylinder (`internal/retainedblend/arc_end_cyl_internal_test.go`, `apitest/boss_root_cliff_test.go`) | `TestCylEndCrossingSine` and `TestCylEndSideRule`: a grazing crossing (R = 7.5, r_G = 2, d = 9.5 − 10⁻⁷) refuses and a transversal one reads, and the root is the side rule's at both ends; `TestCylEndRootBoundCoversRounding`, `TestCylEndSectionMarginCoversRounding` and `TestCylEndSectionTiltCharge` measure `e_V`, `δ`, `ε_ψ` and the tilt term in 200-bit arithmetic under a quarter of each bound; `TestCylEndCertifyChargesCurvature` refuses a chord whose `curv` alone exceeds `τ`; `TestCylEndCuspParam` puts the end curve `need` from the generator along `G`; `TestCapFacetsChargeCoversSagitta` (`arc_end_cyl_cap_internal_test.go`) checks sample points of cap triangles against the exact cylinder under their charged bound, `TestCapFacetsRefusesFoldingChain` refuses a zip with no positive move, `TestSplitChainEdge` refuses feet that are equal or off the edge and fans a facet with feet in order, and `TestCapFacetsNoiseVertexBesideV` closes a vertex beside `V` with the closing triangle, within its facet bound of the arc, one boundary loop and every other edge used once each way; `TestBossRootFilletSpokedHub`: a Ø15 boss on the spoked disc builds four `Torus` faces (`Major` 8, `Minor` 0.5), its volume equals the boss's without the fillet plus the fillet's wedge integral within the result's bound, and `ClosestPoint` of two probes at `r = 7.6`, `z = 5.02`, 0.6° either side of the window boundary, lies on the corner round's cylinder within its off-surface bound and the result's mesh bound, its `z` in `(5, 5.5)`, with its distance within the reported bound of the probe's exact distance to the round (`LocatePoint` cannot decide there: it needs `4r³` above the volume bound, `r` over 0.68 mm); Ø12 builds one torus; Ø13 and Ø14 refuse `ErrUnsupported`; `TestBossRootFilletRefusesArcEndsOnOneFace` and `TestBossRootFilletRefusesHalfCircleArc` refuse a root arc whose ends meet one face and one of π or more | skip the side rule: the fillet ends on the wrong root; drop `curv`: a chord of 4 `τ` certifies; drop the cap: the volume misses the wedge; drop the cap charge's sagitta: the sagitta test fails; read the vertex beside `V` by its held sign: the Ø15 build refuses "the wall's cut along an end face does not run from the setback line to the corner" |
| Close boundary vertices (`internal/retainedblend/rewrite_close_vertices_internal_test.go` `TestRetriangulateKeepsCloseBoundaryVertices`) | a plane face whose hole loop holds two vertices closer than the rounding of an offset frame re-triangulates with every loop edge kept and its area unchanged, for normals −z, +z and +x; two neighbouring vertices that differ only along the normal refuse | subtract the rim centre before projecting: the pair collides and ear clipping ends on a degenerate triangle |
| Wall ring order (`TestSortRingOrdersEqualAngles`) | two cut vertices of one float angle sort by their exact turn, the same for every input order, both senses and both normals | sort by float angle alone: the order follows the input order |
| Oblique end plane (`faceted_retained_blend_tilted_end_test.go`): a 20 × 60 plate with `r = 4.5` corners drawn with tangent and radius constraints, both cap outlines chamfered 0.5, cut to 8 beyond a 30° ramp; the same plate with every point fixed; the gallery-shaped arm (round pin end, `r = 4.5` corner arcs closed by an `R = 22` arc, all joins by constraint) | the cap chamfer's 45° flat patches beside the ramp's top edge keep `Plane` tags through the cut; the top edge's chamfer removes `A·(19 + 2·h̄)` with `A = 0.25·sin 150°/2`, `h̄ = 0.5·sin 30°/3`, within the bound, as much as the exact plate's within both bounds; `VerifyAll` passes; the arm's top edge then its outline, and both in one call, build. Unit tests: `FlatPatchExtent`'s value and every declined premise, `SourceTag` keeping a flat patch and declining any other tilted face, `obliqueCapArea` covering `1/cos α − 1` at the largest tilt | keep A2 strict, or read `G` through `retainedPlane` alone: "an edge end meets a face that is not a retained plane"; drop the perimeter term or the vertex bound from Claim F: the unit value fails; drop the tilt term: its unit test fails |

### 10. Files and fallout

| Area | Change |
|---|---|
| `internal/retainedblend/` | classification (Table RR), exact geometry (§4), clip, strip and end construction (§5), gates (§6); R6 in `sphere.go`, its exact-record rule in `denoted.go`; an R4 arc's end on a cylinder in `arc_end_cyl.go`, its band, chain, cap and split in `arc_end_cyl_cap.go`; R3's wall band in `cyl_band.go`, which also cuts an R4 arc's wall by the planes of `arc_band.go` and a concave chain's wall by those of `chain_band.go`; the strip triangles' bounds, the band's charge and the corner patch's in `strip_bound.go` |
| `internal/facetedrecord/` | an entry that builds a payload from new vertices, triangles, groups and bounds |
| `internal/retainedblend/saddle.go`, `saddle_tool.go`, `internal/crossbore/saddle_tool.go` | R7 (§4.9): recognition, tool, band proof, outside and clearance proofs, the Cut |
| `internal/brepmodify/chamfer.go` | calls `retainedblend.SaddleChamfer` with the hooks' mesher before `retainedblend.Chamfer` |
| `internal/retainedblend/saddle_fillet.go`, `internal/saddlecanal/` | R7 fillet (§4.10): the window and bore cuts, the bands, the strip and the bounds; the exact fillet, its rows and its cell proofs |
| `internal/brepmodify/fillet.go` | calls `retainedblend.SaddleFillet` before `retainedblend.Fillet` |
| `fillet.go`, `chamfer.go` | call the route before SX9 for a `facetedPayload`; thin wrappers only |
| Part R | RX6 row and SX9 text; the sentence in §11 about what SX9 leaves a caller proving |
| `agent-docs/missing-features.md` | the row for faceted `Fillet`/`Chamfer` |
| `agent-docs/faceted-source-retention-design.md` | this document's row in §9, §13 |
| `agent-docs/layout/design-docs.md` | this document's row |
| `apitest/`, `examples/` | the tests of §9; one executable example, a filleted hole mouth on a placed-tool plate |

### 11. Increments

Each stage below is built. Its gate is the §9 rows named.

| # | Built | Gate: rows that pass |
|---|---|---|
| B1 | R2 fillet and chamfer: classification, the four rims of §4.2, clip and strip, bounds, gates SR1–SR4 and SR6–SR7 | the hole-mouth, boss-root and shaft-end rows of §9 pass; `go test ./...` on CI |
| B2 | R1 fillet and equal-distance chamfer with planar end faces, SR5 | the 90° and 45° edge rows, the oblique-end row and the L-section concave row pass |
| B3 | R3, a line between a plane and a parallel cylinder, ending on planes normal to the axis | the groove, chamfer and boss-foot rows of §9 pass |
| B4 | R4, an open arc | as B1 for an arc with end faces |
| B5 | R6, a closed plane-sphere rim (§4.5) | the R6 rows of §9 pass |
| B6 | R7, the cross-hole saddle chamfer (§4.9) | the R7 rows of §9 pass |
| B7 | R7 fillet: the convex mouth (σ = (−1, +1)), and the ball outside both cylinders (σ = (+1, +1)) at a convex edge, two crossing bores, and at a concave one, a boss's root on a rod (§4.10) | the R7 fillet rows of §9 pass |
| B8 | the remaining sign patterns of crossing cylinders: a pin's root inside a tube's bore (σ = (−1, +1) at a concave edge) and every pattern with the ball inside the bore | a design for each pattern's walls and bands |

### 12. Rejected shortcuts

- **A Cut by a cutter solid.** The cutter's blend face is tangent to the faces
  it cuts, and the mesh Boolean refuses tangency. A slightly deeper cutter is
  not the fillet.
- **Promote the retained body to a `brepPayload`.** A brep keeps every frame a
  signed permutation of one reference frame. The bodies this route exists for
  are the ones that do not.
- **Fit a cylinder or torus to the held rim.** It blesses a surface from the
  polygons it should be checked against (CLAUDE.md). The tags come from the
  operands.
- **Admit a blend because the rebuilt volume matches.** A matching volume
  proves nothing about a blend that leaves its faces; the gates of §6.2 admit
  from published bounds, and the volume check can only reject.
- **Blend an oblique plane and cylinder as a `NURBSSurface`.** The surface
  carries no geometry a proof can read.
- **Chamfer a cross hole with a coaxial cone.** A countersink cone sets back
  unequally: along the bore wall its setback varies by `R − √(R² − r²)` around
  the mouth. R7 cuts the equal-setback ruled band instead (§4.9).
- **Fillet a cross hole by a Cut with a tool like §4.9's.** The tool's face
  would cut both walls at an angle. The true fillet meets each wall tangentially
  along T1 and T2, so the Cut leaves a crease the solid does not have.
- **Cut the shaft wall along a curve of T1's family.** The shaft's held facets
  are long strips, and such a curve crosses a strip between its vertices: a cut
  that reads the level at the vertices misses the strip, and a ring cut from the
  vertex levels lies up to a facet's width off the
  curve, far beyond the strip's first row. §4.10 cuts by planes instead.

### 13. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Edge classes in the first increment | R2 (closed circular rims), because they have no end vertices | R1: more common, but needs the end-face construction and SR5 |
| A blend whose clearance is undecided within bounds | refuse (`ErrUnsupported`) | build with a bound that charges the undecided margin: smaller refusal set, larger proof |
| Chord tolerance `τ = 2⁻¹⁰·r` for the blend grid | fixed ratio of the radius | follow the body's `MeshBound`: finer blend, many more facets |
| Several selected edges | one at a time on the evolving mesh; each later rim is found again by its circle | require disjoint regions `K` and build them together |
| `Shell` of a retained body | stays SX9 | a separate design |
| R7 tool stations | `N = 1024` (`crossbore.ToolStations`) | `N = 4096`: a smaller `ε`, but a rim vertex's bound falls only by the wall's share while the rims hold four times the segments, so their length bounds grow; more facets and time |
| R7 band rows and area cells | `M = 8` rows and 8 area cells per quad (`crossbore.ToolRows`, `ToolAreaCells`); `Verify` reads the R = 10, r = 3, d = 1.5 result Sound (`TestRetainedSaddleChamfer`, `apitest/retained_saddle_chamfer_test.go`; its r = 7 row does not assert Sound) | more rows or cells: a smaller `ε` or area bound for more facets and time; fewer stations with the same rows and cells: a larger bound that can read Suspect |
| R7 cut's chain allowance | raised to the walls' held bounds and `ε` (§4.9) | keep the document's: every R7 cut on a Boolean result refuses at the chain-depth gate |
| R7 ratio | `2r² < R²` | admit what the cell proofs pass: wider reach, but no checked fixture past r/R = 0.7 (the r = 7 rows of `apitest/retained_saddle_fillet_test.go`) |
| R7 fillet stations and rows | `N = 256` (`saddlecanal.Stations`), rows for `τ = 2⁻¹⁰·ρ` | `N = 1024` as §4.9: a smaller ε for four times the facets and more time |
| R7 fillet cell split along t | every cell splits in two along t (`rowSplit`): the fillet's area enclosure narrows (`saddlecanal.rowSplit` = 2) for more proof time | no split: less proof time, and a wider area bound that can pass `Verify`'s relative tolerance; or more splits: a narrower enclosure for more time |
| R7 fillet face | untagged `Faceted`, role `fillet(k)` | a sealed `Canal` surface variant (`agent-docs/trim-graph-design.md` increment 9): a tag a reader can query, a new public type |
| R7 fillet sign patterns | σ = (−1, +1) at a convex edge (the cross hole's mouth) and σ = (+1, +1) at either edge (crossing bores, a boss's root) | the patterns of B8 as well |

## Part M — Retained modify reach

What the modify operations and placements do with a mesh Boolean result whose faces carry retained analytic
surfaces (`agent-docs/faceted-source-retention-design.md`, "retention §N"). Companion to
Part E ("blend §N"), which owns Table RR, the blend construction and Table SR;
Part F ("modify §N"), which owns the call contracts and base gates; and
Part R ("reach §N"), which owns tangent-chain expansion (§5), the asymmetric
reference (§6) and SX9. This part adds receiver classes and refusal rows to those tables. It changes no
prism, revolve or brep receiver.

Navigation only; the sections own the content.

| Question | Section |
|---|---|
| What each call does on a mesh Boolean result today | §1.1 |
| Which gaps build and which stay refused | §3 |
| Two-distance chamfer of a retained edge | §4 |
| Fillet or chamfer of a rounded outline (line, arc, line …) | §5 |
| Chamfer of a tangent chain whose edges share no base plane | §15b |
| `Copy` with `WithCopyRotation` | §6 |
| `ScaledAt` | §7 |
| Edge between two parallel cylinders | §8 |
| Why `Shell`, `Draft`, `Offset`, many-body `Union`, cone and torus blends stay refused | §9 |
| Refusal rows and gate order | §10 |
| Tests and their red legs | §12 |
| Increments | §14 |

### 1. Probe

The fixture is retention §1's: a 40×20×20 box with its four z-parallel edges filleted at r=3, cut by a Ø8 tool
along y. The tool is unplaced (P1), translated by a `Transform` (P2), or translated and then rotated 20° about z
(P3). P1's Cut is a `brepPayload` (class B). P2 and P3 are mesh Boolean results whose faces read 6 `Plane` and
5 `Cylinder`, and whose edges publish `Line3`, `Arc3` and `Circle3` (retention §10). The **x-parallel edges** are
the four edges along x at `y ∈ {0, 20}`, `z ∈ {0, 20}`; the top front one runs from `(3,0,20)` to `(37,0,20)`.

#### 1.1 Table P — calls on a mesh Boolean result

P2 and P3 give the same result on every row.
Every refusal is `ErrUnsupported`; the quoted text follows `decad: not supported by the current evaluator:`.
"Kinds" counts the result's faces by `Face.Surface()` kind.

| Call on P2/P3 | Result | Kinds | Closed by, or why refused |
|---|---|---|---|
| `Hole` blind on the top face | builds | 7 `Plane`, 6 `Cylinder` | retention |
| `PatternCopies`, linear and circular | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Mirrored`, `MirroredCopy` across the +x face | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Transform`, translation or 20° about z | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Copy`, no option or `WithCopyPlacement` | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Copy` with `WithCopyRotation`, 20° about z | builds | 6 `Plane`, 5 `Cylinder` | GAP-A; M2 (§6) |
| `ScaledAt` 1.5 about `(20,10,0)` | builds | 6 `Plane`, 5 `Cylinder` | M5 (§7) |
| `Fillet` r=1 or `Chamfer` d=1 of one or all four x-parallel edges | refused (SM2) | — | GAP-E; §5.3, §15 |
| `Chamfer` of the four, `WithAsymmetricChamfer` 1 and 2 | refused (SM2) | — | as above |
| `Fillet` r=1 of the four, `WithTangentChain` | builds two chains | 6 `Plane`, 13 `Cylinder`, 8 `Torus` | M3 (§5) |
| `Chamfer` d=1 of the four, `WithTangentChain` | builds two chains | 14 `Plane`, 5 `Cylinder`, 8 `Cone` | M3 (§5) |
| `Chamfer` of the four, asymmetric, `WithTangentChain`, reference `Faces(Facing(+z)).Or(Facing(-z))` | builds two chains | 8 `Plane`, 8 `Cone` | M3 (§5.6) |
| `Chamfer` of the four, asymmetric, `WithTangentChain`, reference `Facing(+z)` | refused (text A) | — | §5.6 |
| `Shell` inward, top face removed | refused (text S) | — | GAP-B; §9.1 |
| `Draft` 2° of the walls about the bottom face | refused (SD23) | — | GAP-C; §9.2 |
| `Offset` | refused (text O) | — | §9.3 |
| `Union` of P2, a feature and its linear `PatternCopies` | refused (text J) | — | GAP-F; §9.4 |

The refusal texts are these.

- **SM2**: "the selected edge ends at the tangent junction 37,0,20 whose continuation is not selected;
  WithTangentChain selects the whole chain" (`internal/brepmodify/edge_route.go`).
- **Text A**: SX3, "names 0 of the faces adjacent to" the edge, "it must name exactly one" (reach §6), for the chain on the bottom plane.
- **Text S**: "this evaluator shells a straight prism or its one-line Sweep reduction only"
  (`internal/brepmodify/shell.go`).
- **SD23**: "this evaluator drafts a straight prism only (an extrude, its filleted or chamfered result, a tube, or
  any of these placed) (draft SD23)".
- **Text O**: "Offset requires a sheet body" (`offset.go`).
- **Text J**: "Union of three or more bodies needs a certified prism, axial or copied-tooth set; join them two at
  a time" (`internal/unionmany/routes.go`).

The asymmetric chain rows differ in the reference. `Faces(Facing(+z)).Or(Facing(-z))` holds both base planes, the
top and the bottom face. Reach SX3 admits it, because each expanded edge borders exactly one of the two. The four
seeds expand to two closed chains, one on each base plane, and each builds with its own base as reference (§5.6).
A bare `Facing(+z)` names no face adjacent to the edges of the bottom chain, so SX3 refuses with text A.

The probe calls neither the two-distance chamfer of a single edge (GAP-D, M1, §4) nor the parallel-cylinder blend
(M6, §8); §12's fixtures cover both.

On P1 the brep routes answer instead. A plain `Fillet` or `Chamfer` of an x-parallel edge refuses brep-modify SB7,
because the face across the edge's axis at `(3,0,20)` is the curved wall. All three `WithTangentChain` rows build,
the asymmetric one included. `Shell` builds (11 `Plane`, 10 `Cylinder`). `ScaledAt` refuses "scale needs an exact
line-section stack BRep".

#### 1.2 The x-parallel edge

At each end the x-parallel top edge continues tangentially into an `Arc3` of radius 3 between the top face and a
lateral blend cylinder, because the front face is tangent to that cylinder along the generator `x = 3, y = 0`. The
edge has no end face: it is one edge of the top face's closed outline of 4 lines and 4 arcs. The same outline on
the box before the Cut takes reach §8's complete cap-loop blend.

### 2. What the design reads

| Input | Source | Used for |
|---|---|---|
| Retained tags with `NormalBound == 0` and no denoted patch, or a revolve patch whose tag is exactly its denoted surface (blend §2, and blend §4.5 for a sphere) | retention §3 | every exact surface in §4, §5, §8 |
| Retained edge curves (`Line3`, `Arc3`, `Circle3`) | retention §10 | edge classes and chain topology |
| Held vertices, facets and vertex bounds | retention §3.3, faceted-vertex-bounds §2 | rewrite and clearance gates |
| `brepmodify.Asymmetric.Refs` | reach §6, `modify_options.go` `resolveAsymmetricReference` | which face takes which distance (§4) |
| `exactrotation.Rotation` (`Linear`, `Translation`) | `document.go` `copyUnder` | the exact image of a tag (§6) |
| Anchor and factor of `ScaledAt` | `point_plane_scale.go` | the exact similar image of a tag (§7) |

Faces with a stated departure, or a denoted patch whose tag is not exactly the surface it denotes, are not read by §4, §5 or §8, for blend §2's reason. A Revolve `Plane` or `Cylinder` face whose tag is exactly its denoted surface is read as a retained tag (blend §2), and a Revolve `Sphere` whose tag is exactly its record's centre and radius is read by the plane-sphere rim route R6 (blend §4.5).

### 3. Verdicts

| Gap | Verdict | Reason in one clause | Section |
|---|---|---|---|
| GAP-D | build | the blend construction is already two-distance; only the reference mapping is missing | §4 |
| GAP-E | build | at an exact tangent junction the two neighbouring blends share one exact cross-section (Claim J) | §5 |
| GAP-A | build | the exact rotation is exactly orthogonal, so the tag's image costs only its rounding | §6 |
| `ScaledAt` | build | a uniform scale keeps every normal, so the scaled tag costs only its origin and radius rounding | §7 |
| Cylinder/Cylinder, parallel axes | build | the blend is a `Cylinder` from a circle–circle cross-section, as blend §4.3 | §8 |
| GAP-B (`Shell`) | refuse | the cavity needs the trims between offset faces, which a mesh result does not record | §9.1 |
| GAP-C (`Draft`) | refuse | each drafted face must be re-trimmed against its neighbours and the tool, with the same missing trims | §9.2 |
| `Offset` of a solid | refuse | surface-design R37 refuses every solid; not a retention matter | §9.3 |
| GAP-F (many-body `Union`) | refuse here | the refusal is the many-body Union rule, which retention does not touch | §9.4 |
| Cylinder/Cylinder, crossing axes, `Fillet` outside blend R7's fillet row | refuse | the blend is a canal surface outside the surface vocabulary | §9.5 |
| Cylinder/Cylinder, a bore crossing a larger shaft at right angles, `Chamfer` | build (blend R7) | a proven band-shaped tool cuts the exact equal-setback wedge through the mesh Boolean | blend §4.9 |
| Cylinder/Cylinder, a bore crossing a larger cylinder at right angles, `Fillet` of the convex mouth, of two crossing bores' convex seam, or of a boss's or pin's concave root on a rod | build (blend R7 fillet) | both walls are cut by exact planes near the mouth and joined by a faceted pipe whose chord error is proven cell by cell; the pipe removes material at a convex edge and adds it at a concave one | blend §4.10 |
| `Cone`, `Torus` faces | refuse | the admitted sources carry denoted interval surfaces, which this route does not read | §9.5 |
| Plane/`Sphere` rim | build (blend R6) | the meridian blend is R3's line-circle section revolved, and the sphere's record reads as exact points | blend §4.5 |

### 4. Two-distance chamfer of a retained edge (GAP-D)

`retainedblend` already builds every Table RR chamfer from two setbacks: `blendSpec.dA` along the plane face (R2,
R3, R4) or face `A` (R1), and `blendSpec.dB` along the wall or face `B`. The equal chamfer sets both to `d`. This
section passes reach §6's two distances through instead.

#### 4.1 Mapping

SX3 has resolved, for each selected edge `e`, exactly one adjacent reference face `Refs[e]`. Let `F1`, `F2` be the
two faces in the order the class reader names them (`A`/`B` for R1, plane/wall for R2–R4).

| Reference | Setback on `F1` | Setback on `F2` |
|---|---|---|
| `Refs[e] == F1` | `D` (the positional distance) | `Other` |
| `Refs[e] == F2` | `Other` | `D` |

**Resolve each reference to a tag before the first step.** Edges after the first are found again on the evolving
mesh (blend §7), whose face pointers are new. Record, per target, the retained tag of its reference face. Each step
picks the face whose tag equals that tag bit for bit; the two faces of every RR edge carry distinct tags.

R3's chamfer reads `dA` for both setbacks (`internal/retainedblend/cyl_rewrite.go`, `solveSection`). It must read
`dA` along the plane line `L` and arc length `dB` along the wall circle, so `θ_T = dB / R`.

#### 4.2 Geometry and proof

No new claim. R1's chamfer plane runs through two parallel setback lines at in-plane distances `dA` and `dB`; R2
and R4 take the `Cone` through the circle `R + σ·dA` on the plane and the circle at distance `dB` along the wall
(blend §4.2); R3 takes the plane through the generator at distance `dA` along `L` and the one at arc length `dB`
along `C`. Blend §5's exact areas and Claim K already state every term for `dA ≠ dB`. Each face's clearance
(blend §6.2) uses that face's own setback.

`DDelta` and `OtherDelta` must both be zero, for blend §6.1's existing "converts exactly to millimetres" rule.
A tangent chain (§5) takes one assignment for the whole chain; SX3 forces it (§5.6).

### 5. Tangent chains on one base plane (GAP-E)

#### 5.1 Table RC — chain classes

A **chain** is a run of selected edges `e_1 … e_n` that all lie on one retained plane face `P` (the **base**),
where each `e_k` is an R1 edge between `P` and a retained plane wall `W_k`, or an R4 edge between `P` and a
retained cylinder wall `W_k` whose axis is normal to `P`, and consecutive edges meet at a **tangent junction**
(§5.2). A closed chamfer chain may also have walls and a base that state a positional charge
(`agent-docs/charged-window-chamfer-design.md` §3); every other chain reads only exact tags.

| RC | Chain | Ends | Built in |
|---|---|---|---|
| **C1** | closed: every edge of one loop of `P` | none | increment M3 |
| **C2** | open | each end at a three-face corner of a straight R1 edge, on an end plane `G` normal to the edge or meeting the wall square (§5.5 step 5, "An oblique end face"), where the wall is convex or continues past a convex `G`, the base is convex (or, for a concave chain, wraps the corner and `G` is convex) and `G` is convex or reflex (§5.5 step 5), or of a convex chain's R4 arc that blend §4.4 admits, on an end plane parallel to its axis and convex at the corner | increments M4, M4b, M8, M10 |

At a junction both walls are perpendicular to `P`: a cylinder wall's axis is normal to `P`, and a plane wall
tangent to it along a generator contains that generator. So every edge of a chain has dihedral angle `π/2`, and
the whole chain is convex or the whole chain is concave, and `n_g = s·m` at every junction. Here `m = n_P × d`
points into `P` along the walk direction `d` (a face lies to the left of its loop), and the chain's **side** `s`
is `−1` for a convex chain, whose blends cut into the material, and `+1` for a concave one, whose blends fill the
void. A line edge reads `s` from its wall's outward normal: `n_W = −m` gives `−1` and `n_W = +m` gives `+1`; the
two candidates are a unit vector apart, so the float test only classifies, and the edge's own convexity flag must
agree. An arc edge takes `s` from the rim's side (blend §4.2), which `readSide` ties to the edge's convexity. A
chain whose edges read different sides is SM3; at a junction test 3 of §5.2 makes the walls' normals along `g`
equal, so public input does not reach it.

#### 5.2 Tangent junction

Edges `e_k` and `e_{k+1}` meet at a tangent junction at vertex `v` when all of these hold.

1. Their faces are `{P, W_k}` and `{P, W_{k+1}}` with `W_k ≠ W_{k+1}`.
2. `W_k` and `W_{k+1}` are tangent along one line `g` normal to `P`, decided by an exact identity on the tags'
   floats read as rationals:
   - plane `(n, o)` and cylinder `(a, c, R)`: `a · n = 0` and `(n · (c − o))² = R²·(n · n)`;
   - cylinder `(a1, c1, R1)` and cylinder `(a2, c2, R2)`: `a1 × a2 = 0` and
     `|(c2 − c1)_⊥|² = (R1 ± R2)²`, with `_⊥` the part normal to `a1`, `+` for external and `−` for internal
     tangency;
   - two plane walls are never a junction;
   - every cylinder wall's axis is normal to `P` exactly: its cross product with `P`'s frame normal is zero.
3. The two walls' outward normals along `g` are equal: their exact dot product is positive (the sign test of
   reach §5.1).
4. `V = g ∩ P` is exact. `g` is the line parallel to the axis through the foot of the axis on the plane wall,
   `c − n·(n · (c − o)) / (n · n)`, or for two cylinders through `c1 + R1·(c2 − c1)_⊥ / (R1 ± R2)`. Both are
   rational once identity 2 holds, and `P` meets `g` in one rational point.
5. Falsifier: the held `v` lies within `bound(v) + slack` of `V` (blend §6.3's slack). A larger distance disproves
   the pairing and refuses (SR6). A smaller one proves nothing.

Identity 2 is the oracle of reach §5.1 applied to tag data: exact equality proves "tangent"; a difference above
`ClrAngTol` relative to the operands proves "not tangent"; anything between is undecided and is SX2. A tolerance
never proves "tangent", with one exception: the wall-to-axis distance of identity 2 may differ from the radius by
`tangentRoundoff` (64) ulps of the junction's coordinate scale, which is all that the rounding of the stored tags can
explain. A sketch cannot place a line and an arc exactly tangent in floats: its rounded arc centre, line plane and
radius differ by a few ulps, as in a rounded plate built from a sketch. Cross products
of axes and normals stay exact. A gap past round-off but inside the angular band is still undecided. The falsifier
(item 5) still holds the vertex to the junction point, and Claim J holds up to that round-off.

#### 5.3 Expansion

`expandTangentChain` (`brep_modify_partial.go`) expands a `facetedPayload` only when every seed is a retained line or
arc between two faces that §2 reads (`retainedblend.Chainable`); any other faceted seed set is SX9. The walk calls
a retained continuation test in place of `tangentchain.Continues`: §5.2 decides a candidate edge at `v`; an
incident edge whose faces fail §2, or whose curve is a chord chain, is undecided (SX2) unless its direction at `v` is
proven apart from the seed's. A line or arc is apart when reach §5's oracle reads the two rays away from `v` as not
parallel, or their exact dot product is non-negative (`tangentchain.Apart`). A chord chain, such as the rim an
earlier bevel's Cut left, is apart when every held chord from `v` that a facet of each of its two faces holds turns
from the reverse of the seed's ray by more than `asin(10⁻³)` plus the chord's own uncertainty, the angle whose sine
is `4(δ + β_v + β_w)/L` for the chain's bound `δ`, the chord's vertex bounds and its length `L`
(`tangentchain.HeldApart`). A tolerance proves "no" only, as reach §5 requires. At a junction the third incident edge
is the seam `{W_k, W_{k+1}}`; its face map sends `P` to a wall, test 4 of reach §5 fails, and it is "not a
continuation". Seeds whose chains overlap merge into one chain (reach §5's fixed point), so the four x-parallel seeds
of §1 give two closed chains.

`Chamfer` without `WithTangentChain` expands the selection the same way, as the target CAD's tangent chain does: a
selected edge that ends at a proven tangent junction takes its continuation, and its bevel has no free end there.
An expansion the walk cannot decide keeps the selection as named. `Fillet` without the option builds what the caller
named. A selection that names every edge of a chain builds the chain. A `Fillet` edge, or a `Chamfer` edge
whose expansion was undecided, that ends at a proven tangent junction whose continuation is not selected is SM2
(§10).

#### 5.4 Claim J

**Claim J.** Let `e_k` and `e_{k+1}` meet at a tangent junction `V` with base outward normal `n_P` and common wall
outward normal `n_g` along `g`. Let `Π` be the plane through `V` normal to `d_V = n_P × n_g`. Then the
cross-section of `e_k`'s blend by `Π` equals the cross-section of `e_{k+1}`'s blend by `Π`:

| Blend | Common section in `Π` |
|---|---|
| fillet of radius `r`, convex | the quarter circle of radius `r` about `c_J = V − r·n_P − r·n_g`, from `c_J + r·n_P` to `c_J + r·n_g` |
| fillet, concave | the quarter circle of radius `r` about `c_J = V + r·n_P + r·n_g`, from `c_J − r·n_P` to `c_J − r·n_g` |
| chamfer, setbacks `t_P` on `P`, `t_W` on the walls | the segment from `V − t_P·n_g` to `V − t_W·n_P` (convex; `+` for concave) |

With the chain's side `s` (§5.1), both fillet rows are `c_J = V + r·m + s·r·n_P`, because `n_g = s·m`; the
construction computes every junction row from that one form.

*Proof.* `Π` contains `n_P` and `n_g`, hence `g`. For an R1 edge, `d_V` is the edge direction, so `Π` is normal to
the blend cylinder's axis and cuts it in blend §4.1's circle: centre at `r/cos(π/4) = r√2` along the bisector, which
is `−r·n_P − r·n_W` with `n_W = n_g`. For an R4 edge, the meridian plane at `V`'s azimuth holds the axis (parallel
to `n_P`) and the radial direction at `V` (parallel to `n_g`), so it is `Π`. In it the torus is a circle of radius
`r` about the tube centre: radius `R − r` from the axis toward `V` when the material lies inside the cylinder
(`n_g` radial outward), `R + r` when it lies outside (`n_g` radial inward), at depth `r` below `P`. Both give
`V − r·n_g − r·n_P`. A concave edge puts the ball on the void side, which flips both signs in both readings. In
every case the two blends touch `P` and `g` at the feet of `c_J` on them, so both arcs share their end points. The
chamfer case reads the setback points the same way: blend §4.1's setback lines and blend §4.2's cone circles both
pass through `V ∓ t_P·n_g` and `V ∓ t_W·n_P`. For two cylinder walls, both meridian planes at `V` hold both axes
and `g`, so both are `Π`, and the argument repeats. ∎

Both blend surfaces contain the common section, and both tangent planes along it hold `d_V` and the section's
tangent, so the blends meet G1. Claim J needs identity 2 of §5.2 exactly; without it the two sections differ and no
shared row exists, which is why near tangency is SX2 and not a charge (§16 lists the alternative). A closed chamfer chain on charged walls keeps
identity 2 on the tags and bounds the true walls' departure from them by Claim J′
(`agent-docs/charged-window-chamfer-design.md` §3.2).

#### 5.5 Construction

The rewrite touches `P`, every wall `W_k`, every seam between consecutive walls, and for C2 the two end faces.

1. **Exact data.** Per junction `J`: `V_J`, `c_J`, and one row of section points `q_j(J)`, `j = 0 … m`, at tube
   angles `α_j = j·(π/2)/m`. `m` follows blend §5 step 3 for `τ = 2⁻¹⁰·r`; every chain edge has angle `π/2` and
   the same `r`, so every edge takes the same `m`. A chamfer has `m = 1`. **Compute each junction row once** and
   give the same vertex indices to both strips that meet there; `q_0(J) = P_J` lies on `P` and
   `q_m(J) = S_J = g ∩ P'` on the seam, with `P'` the plane parallel to `P` at height `t_W` on the chain's side
   `s`: into the material for a convex chain, into the void for a concave one. A fillet row is
   `V + m·r(1 − sin α_j) + s·n_P·r(1 − cos α_j)` and a chamfer row `V + m·t_P(1 − σ) + s·n_P·t_W·σ`.
2. **Strips.** An R1 edge's strip is blend §5.0 step 4 with each end row taken from its junction (or, for a C2 end,
   from blend §5.0 step 1). An R4 edge's strip is blend §4.4's grid with its two end columns taken from its
   junction rows. Rows are wound to face the blend's outward normal. A line fillet's cylinder has its axis through
   `p_0 + r·m + s·r·n_P`, faces away from the axis when convex and toward it when concave (tag reversed); a
   chamfer's plane faces `n_P + s·m`. An arc's torus or cone reads the rim's side as blend §4.2 does.
3. **Base face.** The chain's run in `P`'s loop gives way to the setback contour: an R1 edge's setback edge between
   its two end setback points, an R4 edge's ring-0 polygon. `P_J` is shared by both neighbours. `P` is then
   triangulated again by ear clipping over its loops (blend §5 step 2), with its other loops kept.
4. **Walls.** Each `W_k` loses the band between `P` and `P'`, measured as the height `s·n_P·(p − o_P)` on the
   chain's side: a plane wall by blend §5.0 step 2's chain cut at the
   setback line, a cylinder wall by blend §5 step 2's cut at the setback height. Each seam's held run is cut at
   `S_J`: the run's vertices on `P`'s side of `S_J` go and `S_J` takes their place.
5. **Ends (C2).** Blend §5.0 (R1) at the first and last edge, which must be straight with a retained plane wall and
   an end face `G` that is a retained plane normal to the edge or meeting the wall square ("An oblique end face",
   below). The end column is the junction row's construction with `V` the corner `P ∩ W ∩ G`, each row slid along
   the edge onto `G`; its rows lie in `G`, and its last vertex is the cut of `W` on `G ∩ W`, shared with the wall. `G` gives up the corner and the chain vertices nearer than the setbacks, and takes the column. `P`'s
   loop gives up the chain's run and the removed part of `G ∩ P` at each end. A first or last arc (R4) takes blend
   §4.4's end: its end column holds, per row `j` of the junction rows' tube angles, the point where the row's circle of
   the torus (or cone) meets `G`, a root of `n_G · x = h_G` in the azimuth nearest the previous row's (blend §4.4).
   The plane's setback point is row 0, on `G ∩ P`; the wall's is the cut of the generator `G ∩ W` at depth `t_W`. The
   edge's interior columns fan out from the end curve: row `j` of column `i` lies at the fraction `i / n_a` of the
   way from its azimuth on `G` to the azimuth of the other end's meridian, so every vertex lies on the torus. The end
   curve stays a chain of exact points (it is no conic). The rows of a fillet or a chamfer with an arc end are refined,
   as blend §4.4's are, until their chords follow the section `G` cuts in the torus or cone to within `τ`
   (blend §6.3), and every entry of the chain takes those rows. `n_a` is the most columns any row needs at the
   step its two gaps leave it once each gap's sagitta and the shift of the rows' ends on `G` are spent
   (`fanColumns`, `rowSteps`), not the rim's span: the rows nearer the axis sweep wider angles than the rim, at a
   smaller radius. That holds a fillet's and a chamfer's strip triangles within `τ` alike (blend §6.3, Strip
   triangles).

   At a straight end `P`, `W` and `G` meet at `V` (`cornerOf` admits exactly three faces), pairwise orthogonally
   where `G` is normal to the edge. The end is classified by the turn of each face's loop at `V` about the face's
   outward normal. The turn is read from the corner's three lines, not from the held vertices next to `V`, which
   can lie within their bounds of both lines: the chain edge's direction comes from its exact end points, and the
   directions along `G ∩ P` and `G ∩ W` point from `V` to the last vertex of `G`'s chain on each line, which the
   end clearance places beyond the setback by more than its bound. The loops' senses at `V` are topological: the
   walk follows `P`'s loop, the held mesh is oriented, so `G`'s loop runs the mesh edge from `V` to `P`'s
   neighbour on `G ∩ P` the other way, and `W`'s loop runs the chain edge the other way from `P`'s. An end whose
   `G` loop does not run that edge the other way, so that `G` would leave the first corner from `P`'s side or
   enter the last one toward it, refuses ("an end face's loop runs its edge with the base the same way as the
   base's loop"); an oriented held mesh never reaches it. A turn whose lines are too near parallel is undecided
   and refuses; at a corner of three orthogonal planes it reads `±1`.

   | Chain `s` | `P` at `V` | `W` at `V` | `G` at `V` | Case | `G`'s cap |
   |---|---|---|---|---|---|
   | −1 | convex | convex | convex | ordinary convex end | loses |
   | −1 | convex | convex | reflex | reflex foot: a ledge standing out of a wall | gains |
   | +1 | convex | convex | reflex | run-out: a slot leaving through a face | gains |
   | +1 | convex | convex | convex | blind end: a pocket floor ending at a sharp pocket corner | loses |
   | +1 | reflex | convex | convex | the base wraps the corner: a block's root ending at the block's sharp vertical edge | gains |
   | +1 | convex | reflex | convex | the wall continues past `G`: a floor running on behind a block's back face | gains |
   | −1 | convex | reflex | convex | the wall continues past `G`: a ledge's outline running into a step's riser or a ramp | gains |
   | −1 | reflex | any | any | the base wraps the corner of a convex chain | refuse (SM8) |
   | +1 | reflex | any | reflex | the base and `G` both wrap the corner | refuse (SM8) |
   | ±1 | any | reflex | reflex | the wall continues past a `G` that is reflex | refuse (SM8) |
   | +1 | reflex | reflex | any | the base and the wall both continue past `G` | refuse (SM8) |

   When a straight edge ends on a normal plane, the end section lies in the triangle formed
   by the corner and the two setback points. The ball about the corner with radius equal to
   the larger setback contains all three points and the entire triangle. Charge the margin
   and each boundary segment's endpoint bounds when checking the rest of the end face.

   The end column lies in `G`'s plane, in the quadrant between `G ∩ P` and `G ∩ W` that holds the blend's
   section. Where `G` is convex at `V` that quadrant is `G`'s own, so `G` gives the cap up; where `G` is reflex
   the quadrant lies outside `G`, so `G` takes it. The loop operation is one splice in both cases: `V` and the
   parts of `G`'s two chains inside the setbacks give way to the column, and because a face lies to the left of
   its loop the splice shrinks a convex corner and grows a reflex one. The exact area `G` loses is the cap,
   `A_f = r²(1 − π/4)` for a fillet or `t_P·t_W/2` for a chamfer, with sign `+` for a convex `G` and `−` for a
   reflex one. A cap `G` takes is new surface on the far side of `K_k`, and §5.6 tests it as it tests the strip.

   **A base that wraps the corner** (a concave chain on a plate, ending at a block's sharp vertical edge). `P`
   is then reflex at `V`, and its remaining region at `V` is the half plane on the far side of `G ∩ P` from the
   chain. The blend's strip ends on `G`'s plane, so `P` keeps `V`: nothing of `G`'s chain along `G ∩ P` is
   removed, and `P`'s new boundary runs from `V` along `G ∩ P`, away from `G`'s chain, to the setback point
   (the end column's first row), then along the offset contour. `readEndCut` reads the base's turn before the
   chains, from `P`'s neighbour of `V` on `G ∩ P` (its distance along that line must exceed its bound plus the
   margin, else the end keeps today's reading and refuses), and marks the end `wraps` only for a concave
   chain's straight end. `W` still trims, so `G`'s chain along `G ∩ wall` gives way below the wall's setback as
   at any end. `G` must be convex (a reflex `G` beside a wrapping base refuses, SM8) and takes the cap: the
   section lies in `G`'s plane on the side of the chain's wall away from `G`'s held face, so it is new surface
   beside `G`'s held vertical edge at `V`. `G` keeps `V`, drops its chain below the wall's setback, and gains the
   column after `V` where its loop reaches `V` from `P`'s side, or before `V` otherwise (`wrapLoop`). The exact
   area `G` loses is `−A_cap`. `P`'s contour, with `V` kept at each wrapping end, is tested as a path, or as a
   closed polygon when both ends lie on one `G` and the chain is the whole loop; it checks the end segment from
   `V` to the setback point as well. The base's clearance test (§5.6) already covers that segment, which lies
   within `t_P` of `V`. A convex chain keeps SM8 at a wrapping base: the blend cuts into the material there, and
   its section has no end face to lie in.

   **A wall that continues past the corner** (a concave chain whose floor runs on behind a block's back face, or
   whose round runs on past the block's front face; a convex chain along a ledge whose side walls run on up a
   step's riser or a ramp). `W` is then reflex at `V`: `G`'s chain along `G ∩ W` leaves `V` away from the wall's
   setback point on `G ∩ W`, which lies along `s·n_P` from `V` where `G` is normal to the edge (into the void for
   a concave chain, into the material for a convex one), and `readEndCut` reads that sign at every straight end
   and at a concave chain's arc end. `W` keeps `V` and all of `G`'s chain
   along `G ∩ W`, and loses only its part inside the band (below). Its new boundary gains the run where `G`'s plane
   cuts the band, from the corner `C` where the setback plane meets `G` back to `V`. `G` must be convex and takes
   the cap: the section lies in `G`'s plane beside `V`, away from `G`'s held face. `G` keeps `V`, drops its chain
   along `G ∩ P` below the base's setback, and its loop runs the end column to `C`, the band's run, then `V`
   (`rebuildEnds`); a straight end's last row and an arc end's are the setback plane's cut on `G ∩ W`, which is
   `C`. The wall's cut is the band, not the height cut, wherever the wall holds a vertex behind `P` or an end
   extends (`chain_band.go`, below).

   **The wall band.** A chain wall that lies on both sides of `P` (the notched round of an ear root: the round runs
   on behind the ear's side face and past its front face) or extends past an end plane is cut by the band of
   blend §4.4 (`cutBand`) with the exact planes `P′` (the setback plane, whose crossings are the strip's last row),
   `P` (the held vertices on the chain edge lie on it by construction) and the end plane `G` of each end where
   the wall extends. A junction needs no plane: the wall face ends at the seam it shares with the next wall, and
   both walls cut the seam's held edge once, at one vertex. Where every wall vertex lies on `P`'s side and inside
   every end plane, the height cut of `clipChainWall` runs, so every case built before keeps its vertices and
   triangles. A rim read from its facets along the arc (blend §4.4, `twoSided`) takes the band, and only a concave
   arc builds it.

   When both ends lie on one `G`, its loops are read once and both splices are applied in turn before `G` is
   triangulated once. Each end's run holds only the vertices nearer its corner than the setbacks, and the end
   clearance places the last vertex of each of its chains beyond them; where two runs would still share a vertex,
   the second splice finds its run broken and refuses ("a face boundary breaks a run into pieces").
   When the chain and the removed parts of `G ∩ P` are all of `P`'s loop (a tongue, a blind pocket floor), `P`'s
   new loop is the setback contour alone: its first and last points are the two setback points on `G ∩ P`, and
   its closing segment runs along `G ∩ P`; the contour is then tested as a closed polygon.

   **An oblique end face** (a ramp or a drafted end wall at a straight end). `G` need not be normal to the edge,
   but it must meet the wall square, `|n_G · n_W| ≤ 10⁻⁹`, else "an open tangent chain's end face does not meet
   its wall square"; its normal then lies in the plane of `d` and `n_P`, so `G` tilts only about its line
   `G ∩ P`, which stays normal to the edge. `c = |d·n_G|` below `minSine` refuses as grazing. A row point at
   offset `a` along `m` and height `b` on the chain's side slides along the edge onto `G`: from the corner into
   the chain by `−(a·m + b·s·n_P)·n_G / (d·n_G)`, with `d` pointing into the chain, so `m·n_G = 0` keeps the
   base's setback point at `t_P` along `G ∩ P` and the base's band a rectangle, while a row at height `b` runs
   `b·κ` longer than the edge, `κ = s·(n_P·n_G)/(d·n_G)`. The wall's setback point is the last row slid onto
   `G`; its signed distance along `G`'s chain on `G ∩ W` decides whether the wall trims (positive) or continues
   past `G` (negative), and the wall's cut meets `G` on the line `G ∩ W` (`findEndCut`). A fillet's rows step by
   `c` finer, as blend §5.0 step 1's do, so each row chord's sagitta stays within `τ` on `G`. The cap is the
   section's corner projected along the edge onto `G`, its area the section's over `c` (`capArea`), and it lies
   within `√2·max(t_P, |S − V|)` of `V`, `S` the wall's setback point on `G`, which the end face's reach test
   uses. A `G` normal to the edge keeps every reading above unchanged.

   An arc end is built for a convex chain on a `G` convex at the corner, and for a concave chain on a `G` convex at
   the corner (a blind end, `G` gives the cap up) or reflex there (a run-out, `G` takes the cap, as a slot floor
   does where it runs out through a face). The end column's rows take the profile of the rim's own side (the ball on
   the void side, `κ` as blend §4.2), and each row's root is the one nearest the rim's own end, which lies on the
   chain's side of `G`. A convex chain's arc end on a reflex `G` refuses ("an arc end of a tangent chain needs an end
   face convex at the corner", SM9). A concave chain's wall may continue past `G`
   and lie behind `P`, as a straight end's does (above); a convex chain's wall must end with the edge.
6. **Build.** `facetedrecord.Build`, as blend §5 step 5. Each chain edge gets its own blend group with role
   `fillet(k)` or `chamfer(k)`, `k` the edge's ordinal in the expanded selection.

New edges: each fillet junction row is an `Arc3` (centre `c_J`, radius `r`, axis `±d_V`) between a `Cylinder` and a
`Torus` blend, published by retention §10.1 E11 and marked `RetainedCurve` (blend §5.2). A chamfer junction keeps its
chord chain: no edge rule publishes the `Plane`/`Cone` section. Setback edges and wall cut edges follow
blend §5.2.

#### 5.6 Gates

All tests are exact rational predicates over held floats and published bounds, inflated by `b + slack` with `b`
the largest vertex bound of the tested chord. The inflation only refuses more.

- **Base band.** Edge `k`'s band in `P` is a rectangle (R1: between the edge line, its setback line and the planes
  `Π` or `G` at its ends) or an annular sector (R4: radii `R` and `R ∓ t_P` about the axis, between the two radial
  lines to its ends). Every held boundary chord of `P` that is not on the chain must miss every band. A rectangle
  test is affine. An annular test clips the chord to the sector's wedge (two half-planes, exact), then compares
  the clipped chord's exact minimum and maximum squared distance from the axis with the two squared radii. An R4
  edge whose span exceeds `π` has no convex wedge and is SM3.
- **Neighbours.** The vertices of edges `k − 1` and `k + 1` other than `V_J` are chords of `P` for edge `k`'s band
  test like any other. A G1 neighbour leaves `V_J` beyond `Π_J`, so it clears the band.
- **Walls.** Every held boundary vertex of every wall that lies on no chain edge and no seam has height at least
  `t_W + b + slack` from `P` on the chain's side `s`. Height above a plane is affine, so the chords between such
  vertices clear `P'` as well. A concave pocket shallower than `t_W` fails it (SR4). A wall that lies on both
  sides of `P`, or runs on past an end plane, has boundary legitimately behind `P` and past `G`, so a single
  R4 arc (blend §4.4) tests every boundary chord of the wall against the band between `P` and `P'`, inside the end
  planes it extends past and the guard, widened by `b + slack`: a chord meeting it refuses (SR5), and a vertex
  strictly inside it other than the rim's and the chain's refuses too. A chain's wall keeps the height test where it lies on `P`'s side and inside the end planes; a chain wall that
  lies on both sides of `P` or extends past an end plane takes the same band test, with the seams' chords exempt, and its vertices
  strictly inside the band other than the chain's own boundary and its seams refuse (`chain_band.go`).
- **Seams.** Each seam's far end lies at height at least `t_W + b + slack` from `P` on side `s`, so `S_J` cuts it.
- **Sizes.** Blend SR3 per edge: an R4 edge whose base lies inside its circle (`κ = −1`: a convex edge whose
  material lies inside its cylinder, or a concave pocket floor's arc) needs `R − r > 0` (fillet) or `R − t_P > 0`
  (chamfer).
- **Asymmetric.** `P` borders every edge of the chain, so a reference that names `P` names it for every edge. A
  reference naming walls names each edge's own wall. A mix gives some edge two reference faces, and SX3 refuses it
  first (reach §6's cap-loop argument); SM5 is the second check. Several chains of one selection (the top and
  bottom outline of a bored plate) each build with their own base as reference. A reference that names every base
  at once, such as `Facing(+z).Or(Facing(-z))`, resolves to one face per edge. A chain after the first is found
  again on the partly blended body, possibly directed opposite to its selected edges, and takes each edge's
  reference by the edge's two end points in either direction.

A chord inside a band deflated by `b + slack` proves the blend consumes another face (SR4). A chord that meets the
inflated band but not the deflated one is undecided (SR5). Every band of a chain lies in the prism over its base
band between `P` and `P'`, so disjoint base bands give disjoint regions `K_k`, adjacent ones sharing only `Π_J`.
A component that enters `K_k` without touching `P`, a wall or a seam either crosses `K_k`'s boundary or lies wholly
inside it. The held audit (SR7, `internal/facetproof/faceted.go`) does not see the first case: it checks that the
mesh closes and that its shells nest with consistent signs, reading one probe point per shell, and it tests no
facet for intersection. So the chain runs its own strip test before the build (`strip_clear.go`): every facet
the result keeps outside the strip, held or rebuilt, that shares no vertex with a strip facet must miss it, decided
by `meshbool.TriTriClassify`, the exact triangle/triangle classification, on the held floats. A contact refuses
with "the blend meets another facet of the body at" a point: an end of a point or segment contact, or a corner of
the other facet for a coplanar overlap, which carries no point; a facet whose corners are collinear refuses
with "the blend's clearance from a flat facet is undecided"; a classification that fails refuses with "the blend's
clearance from the rest of the body is undecided"; all three are `ErrUnsupported`. Float bounding boxes, whose
corners are exact, skip pairs that cannot meet, and a uniform grid over the strip facets' boxes finds the pairs
whose boxes meet without testing every pair. A coordinate's cell is clamped to the grid in floating point before
it becomes an integer, so the cell index never decreases as the coordinate grows, even for a query far outside.
A strip facet's vertices are all made by the construction (rows, setback points, wall cuts, end columns), so a
pair that shares a vertex with it is the construction's own seam: along the setback contour, along a wall cut, or
for an open chain along the end column that a rebuilt end face shares with the strip; the gates above and the end
checks already place those. A rebuilt end face that takes a cap also keeps `G`'s held boundary vertices, and ear
clipping can join one of them to the cap, so a held facet at such a vertex could share it and still cross the
cap. Its pairs are therefore read by kind of shared vertex. A pair that shares a vertex the construction made is
skipped: besides the strip, only the rebuilt base and walls carry such vertices, and their planes meet `G`'s along
`G ∩ P` and `G ∩ W`; the new `P` and walls lie within the held ones, which met the held `G` only on its boundary,
and the cap meets those lines only along its own edges. A pair that shares only held vertices is classified like
any other and passes only when the exact contact is a shared vertex or the edge between two shared vertices. A
held facet coplanar with `G` that shares an edge with a rebuilt `G` facet meets it in what the classification
reports as one coplanar region, so it is refused; the refusal can only be a false one, and no current fixture
reaches it. The rest of `K_k`'s boundary is the removed bands of `P` and the walls and, for an open chain, the
caps its end faces give up or take. No other facet of the held solid crosses a removed band or a cap a convex `G`
gives up, because each was part of the held surface. A cap a reflex `G` takes is new surface, and a held shell can
cross it: a solid floating in a run-out slot's void across `G`'s plane, or a cavity across `G`'s plane inside a
reflex foot's ledge. So the rebuilt facets of an end face that takes a cap are tested like the strip, against
every facet the result keeps outside them, and a contact refuses with "the cap an end face takes meets another
facet of the body at" a point (`ErrUnsupported`). A shell wholly inside `K_k` crosses nothing, so with no
crossings the audit's one-point parity reads its nesting correctly: a concave chain's `K_k` lies in the void, and a
positive shell inside it nests inside a positive shell and refuses; a convex chain's `K_k` lies in the material,
and a cavity inside it ends up contained by no shell, so the audit refuses its negative sign.

The routes that blend one edge (blend Table RR and R5 of §8) build through the same two steps. The rim routes (R2, R6)
and the mitre run of blend §4.7 have no end caps and run only the strip test, through `buildClear`. The straight-edge
(R1), plane–cylinder (R3), arc (R4) and cylinder–cylinder (R5) routes call `checkStripClear` with the rebuilt
facets of every end face that gains the blend's end cap, then `build`. An end face gains a cap in three cases: a
convex edge on a reflex `G` (R1), a convex R3 end whose wall runs past the corner, and every end of a concave edge,
whether `G` is reflex (a run-out, a notch fill) or convex beside a face that extends (a base that wraps, a wall's
foot). The tests use the same texts and the same skipped pairs as the chain, and the shared vertices also include the
trim of an end face along the strip's end row. The refusal is blend SR7, where for a chain it is SM5. Their region
`K` is bounded by the removed bands of the two faces and the end faces' cut corners, which the blend gates (blend
SR4, SR5) place. A convex end gives up a cut corner of held surface, so nothing held crosses it. A gained cap is new
surface, so the rebuilt `G` facets are tested as the chain's are. A cavity or solid crossing `K`'s boundary then
meets a strip or cap facet and refuses, and one wholly inside `K` fails the audit's parity. A cavity whose corner
reaches the removed corner of a plate's top edge, a hole mouth, a broken-out arc, a groove's edge or the ridge between
two bores builds without the strip test and refuses with it. A solid crossing the cap of a concave single edge's end
face (a base-wrap block's root with sharp corners, a run-out slot's floor edge, a rod's foot on a plate, the valley
between two bores, a concave arc that breaks out of two faces) builds without the cap test, a self-intersecting
body that the held audit passes when its probe point lies outside the fill, and refuses with it.

#### 5.7 Bounds

Claim K (blend §6.3) holds with `K = ∪ K_k`: outside it the rewritten mesh equals the held one; inside each `K_k`
the strip's facets leave the exact blend by at most `τ`, and the band beside the walls' cut by its cut vertices'
raised bounds (blend §6.3, Strip triangles). The shared junction rows add no term, because both neighbouring strips
use the same vertices, which lie on both exact blend surfaces (Claim J). On charged walls a junction vertex also takes
the junction's shift (`agent-docs/charged-window-chamfer-design.md` §3.3). A junction vertex takes `τ` plus the
rounding of its computation. `VolSymDiff` and `AreaSlack` add each edge's terms as blend §6.3
states them, and `AreaSlack` the shortfall of the chain's rebuilt body's geometric terms against the old body's, by
source group, once per chain (blend §6.3, Area). The exact areas are
blend §5.0's for an R1 edge with `μ = 0` (its end planes are normal to it) and blend §4.4's for an R4 edge over its
azimuth span. A straight end on an oblique `G` (§5.5 step 5) adds, to its entry's wall band `κ·t_W²/2`, to a
chamfer's bevel `κ·t_W·√(t_P² + t_W²)/2` and to a fillet's cylinder `κ·r²·(π/2 − 1)`, the integrals of `b·κ`
over the section's wall leg, bevel and arc (`obliqueEndAreas`); the base's band keeps its rectangle. An end face's cap enters the removed area with the sign of §5.5 step 5. A wrong sign moves the exact
change by four caps, which `AreaSlack = |Δ_held − Δ_true|` absorbs, so no published reading shows it; the sign has
its own unit test.

### 6. Exact-rotation `Copy` keeps its tags (GAP-A)

`WithCopyRotation` gives a faceted copy that denotes `Q·S + t_Q`, with `Q` an exactly orthogonal rational rotation
(`internal/exactrotation`). `ExactGroupPlaced` drops every group's `Denoted`, and every `Surface` and `CapRecord`
except those of a plane that `Q` maps onto itself (below), because they describe the float placement and not `Q`.

**Fixed planes.** A retained `Plane` group with no denoted patch and `NormalBound == 0` keeps its `Surface`,
`CapRecord`, `AxialDelta` and `HasAxialDelta` verbatim, at bound 0, when `Q` maps its plane onto itself: for the tag's
exact normal `m = U × V` (not the rounded `Frame.N()`), and for the `CapRecord` plane's normal `m = (B·U) × (B·V)` when there is a record, `Q·m = m`,
`Qᵀ·m = m` and `m·t_Q = 0`, all in rationals (`facetedrecord.PlaneFixedBy`). The image plane is then the source
plane, so the source's exact record and its axial bound denote the copy's plane. The check compares recorded
data and admits only when every equality holds. A rotation about z fixes the ±z caps. Every other plane takes the
construction below.

**Construction.** For any other group with `Retained == true`, no denoted patch, and `Surface` a `Plane`, `Cylinder`,
`Sphere` or `Torus`: set `Surface` to the float image of the tag under the float delta
(`surfaceplace.TransformSurface`) and `NormalBound` to `min(2, ε_F + add)`, where `ε_F` is the group's own
`NormalBound` and `add` is retention §3.4's charge computed against `Q`: the linear map is `Q.Linear()`, the
translation `Q.Translation()`, and `e = 0` because `Q` is exactly orthogonal. A plane takes `8a + P`; a curved tag
takes the held term of retention §3.3 at displacement `2·(P + a·D)`. `a` and `P` are the exact distances of the
moved frame vectors and origin from their images under `Q`, so they cover both the float rounding and the gap
between the float delta and `Q`. A retained `Cone` keeps the bound 2, as retention §3.4 states. Denoted patches,
non-retained structural tags and `CapRecord` are dropped.

**Proof.** The copy's true solid is `Q·S + t_Q`. By Claim R each true patch of the group lies on the source tag
`T`, so it lies on `Q·T + t_Q` exactly. The published tag is within `a` (directions) and `P` (origin) of that
surface, and `Q` maps normals to normals without stretch, so retention §3.4's derivation holds with `e = 0`. A
plane that `Q` fixes is handled by the fixed-plane rule above.

**Code.** `facetedrecord.PlacedTagDepartureBy(tag, moved, linear, translation, e, verts, reach)` takes the exact
rational map. `PlacedTagDeparture` is its wrapper for a float transform. `ExactGroupPlaced` calls
`PlacedTagDepartureBy` with `fp.Verts` and the `reach` that `PlacedWith` computes.

### 7. `ScaledAt` keeps curved tags

`ScaleImage` (`internal/facetedrecord/point_plane_scale.go`) scales each retained `Plane`, `Cylinder`, `Sphere`
and `Torus` tag through `facetedrecord.ScaleTag` (`scale_tag.go`). A positive uniform scale `x ↦ a + s·(x − a)`
maps each surface to a similar one with the same normal at corresponding points, so the only charge is rounding.

| Tag | Scaled tag | Charge `add` |
|---|---|---|
| `Plane` | origin `round(a + s·(o − a))`, frame from `r3.NewFrame` on the unchanged `U`, `V` | `8·α + P` |
| `Cylinder` | axis unchanged, origin as above, `Radius = round(s·R)` | held term of retention §3.3 at `2·(P + ρ)` |
| `Sphere` | centre as above, `Radius = round(s·R)` | the same |
| `Torus` | axis unchanged, centre as above, `Major` and `Minor` each rounded from `s` times the source value | the same |

`P` is the exact distance of the rounded origin from `a + s·(o − a)`, `ρ` the exact gap of the rounded radius
from `s` times the source radius (a `Torus` takes the sum of its two radii's gaps), and `α` the exact gap of `NewFrame`'s vectors from `U`, `V`. Each group's
`NormalBound` becomes `min(2, ε_F + add)`; `ε_F` is a bound on an angle, which a uniform scale preserves. `Cone`
and denoted groups still read `Faceted`: retention §3.4 states no cone rule, and the interval denotation has no
similarity image here. A retained curved group passes through the same A4 check at assembly as any other. A plane group that is not
retained keeps the bound 2.

### 8. Parallel cylinders (R5)

Two cylinders with parallel axes meet in straight generators. Two rows publish and blend them.

**E10** (retention §10.1). `Cylinder` and `Cylinder` with `|a1 × a2| ≤ 10⁻¹²` and axis distance `s` with
`|R1 − R2| < s < R1 + R2` within slack: `Line3`. Two generators exist, at the cross-section's circle–circle
intersections; either passes if E6 accepts it, as in E4.

**R5** (Table RR). A `Line3` of E10 ending on retained planes normal to the axes. In the cross-section normal to
the edge, the walls are circles `C1 (o1, R1)` and `C2 (o2, R2)` meeting at `K`. The angle between their tangents at
`K` must clear `minSine`.

- Fillet: the centre is the point of `circle(o1, R1 + κ1·r) ∩ circle(o2, R2 + κ2·r)` nearest `K`, with `κ_i`
  chosen per circle exactly as blend §4.3 chooses `κ`. The tangent points are the radial projections of the centre
  onto `C1` and `C2`. The blend is a `Cylinder` of radius `r` along the edge.
- Chamfer: arc length `dA` along `C1` and `dB` along `C2` from `K`; the blend is the `Plane` through the two
  generators.
- Each wall is cut as blend §4.3 cuts R3's wall, by the half-plane through its axis and its setback generator.
- Ends: blend §4.3's end rule.

The offset circles not meeting is SR3. Both ends must be corners of the same type as the edge (blend SR5), so
a valley between bosses that stand on a plate, with one convex and one reflex end, refuses. The construction reuses `cylLine.solveSection` with the line `L` replaced
by `C1`.

### 9. Refusals

#### 9.1 `Shell` (GAP-B)

An inward `Shell` removes the cavity `S ⊖_F t`: the erosion of `S` by `t`, open through the removed faces `F`
(modify §8's erosion rules). The tags give every offset surface exactly: a plane moved `t`, a cylinder at
`R ∓ t`, a sphere, a torus minor radius `∓ t`. The cavity also needs where those offsets meet: a miter at a convex
edge, a radius-`t` blend about a reflex edge, the faces that vanish when `t` exceeds them, and how a bore's offset
meets a wall's offset. A mesh result records none of it (retention §12, last bullet), so building the cavity from
tags is a general 3D erosion kernel.

Two subclasses were examined.

- **Every face a retained plane, and the body provably convex.** Convexity is proven when every held vertex off a
  face lies on the inner side of that face's plane by more than its bound. The cavity is then the intersection of
  the moved half-spaces, exact by rational vertex enumeration, and SX8's topology-change refusal applies when a
  face drops out. This is sound. It is not built: it is a new polyhedral kernel for bodies that mesh Booleans
  rarely produce.
- **Retained planes and cylinders** (the drilled rounded box of §1). The bore makes the body non-convex, and its
  reflex junctions need trims the record lacks. This design has no construction from tags alone.

`Shell` of a `facetedPayload` stays SX9 outside the source-backed routes (Part S).
§16 lists the operand-record route.

#### 9.2 `Draft` (GAP-C)

Drafting turns each selected wall's tag about its line on the neutral plane. The turned plane is computable
(a rational rotation, as `internal/exactrotation`). Each drafted face then needs new trims against its neighbours,
and a bore through a drafted wall meets it in a new curve, which re-meshes the bore wall. These are the same
missing trims as §9.1. `Draft` of a brep receiver is also SD23 (the probe's unplaced-tool Cut, a `brepPayload`),
so a mesh `Draft` would come after a brep one. `Draft` of a `facetedPayload` stays SD23. §16 lists the
operand-record route.

#### 9.3 `Offset` of a solid

`Offset` refuses every solid (surface-design R37, `agent-docs/missing-features.md` "Direct edits"). Retention
changes nothing about it.

#### 9.4 Many-body `Union` (GAP-F)

Not a retention matter. A `Union` of the target, a feature and its `PatternCopies` joins every operand in one
many-body `Union`. Its pairwise
fallback, `tryPrismUnionChain` (`internal/unionmany/routes.go`), admits prism and exact-Loft operands only; the contract is
`agent-docs/api-design.md`'s many-body `Union` paragraph. Each pair of a chain would be an ordinary mesh Boolean,
and retention §3.1 already carries tags through a second generation. Whether faceted operands may join that chain
belongs to that owner.

#### 9.5 Cylinder/Cylinder, `Cone` and `Torus` blends

- **Crossing cylinders.** The rolling ball's centre runs along the intersection of two offset cylinders, a
  quartic, and the blend is a canal surface about it. It is no `Cylinder`, `Cone`, `Torus` or `Sphere`, and
  `NURBSSurface` carries no geometry a proof can read (spline §7). The convex saddle a bore leaves in a larger
  shaft, with exact perpendicular coordinate axes that meet and `2r² < R²`, builds an equal-setback `Chamfer` as
  blend R7 (§4.9) and a `Fillet` as an untagged faceted pipe (blend §4.10). The same axes with the ball outside
  both cylinders, the convex seam of two crossing bores and the concave root of a boss or pin on a rod, build a
  `Fillet` too. Every other crossing pair (a pin's root inside a tube's bore, a ball inside the bore, a bore off
  the larger cylinder's axis or off the coordinate axes, `r/R ≥ 1/√2`, a `Chamfer` of two crossing bores or of a
  root, a two-distance chamfer) is SR1.
- **`Cone` and `Torus` faces.** In the meridian of a coaxial pair the blend is a `Torus`, so the geometry exists.
  The retained cones and tori that reach a mesh result come from revolves. Their tag is a float re-expression (the
  cone's half-angle is an `Atan2`, `internal/revolveplan/payload.go`) whose departure is stated only through the
  face's denoted interval surface (`topo.Face.Denoted`), which blend §2 reads only for a `Plane` or `Cylinder` tag
  equal to it, and blend §4.5 for a `Sphere` whose record's centre and radius are its tag's. A cone or torus that a blend
  built (blend §4.2) meets its neighbours tangentially, so it has no edge to blend, and a later cut through it
  gives a non-coaxial rim. SR1. §16 lists the denoted-surface route. The `Intersect` seat reading reads a revolve
  cone's rulings from that record, not its tag (coplanar-contact §16.2).

### 10. Table SM — added refusal rows

Rows narrow blend Table SR and reach SX2/SX9; sentinels follow those tables.

| SM | Call | Sentinel |
|---|---|---|
| **SM1** | a chain junction whose tangency §5.2 cannot decide | `ErrUnsupported` (reach SX2's text) |
| **SM2** | a selected edge ends at a proven tangent junction whose continuation is not selected and `WithTangentChain` is absent, on a `Fillet` or on a `Chamfer` whose expansion (§5.3) was undecided; the message names the junction point and `WithTangentChain` | `ErrUnsupported` |
| **SM3** | a chain whose edges share no base plane, other than an equal-setback `Chamfer` (§15b), a junction of two plane walls, a cylinder wall whose axis is not exactly normal to `P`, an R4 chain edge spanning more than `π`, or a chain that mixes convex and concave edges | `ErrUnsupported` |
| **SM4** | the chain gates of §5.6: proven overlap | `ErrDegenerate` (SR4) |
| **SM5** | the chain gates of §5.6: undecided; a strip facet, or a facet of an end face that takes a cap, that meets another facet of the body; or a chain whose resolved references mix `P` and walls | `ErrUnsupported` (SR5; SX3 first for the mix) |
| **SM6** | R5: the offset circles do not meet | `ErrDegenerate` (SR3) |
| **SM7** | R5: the walls meet at `K` below `minSine` | `ErrUnsupported` (SR5) |
| **SM8** | a straight end of an open chain whose wall is reflex at the corner and whose end face is reflex, or whose base is reflex too, or whose base is reflex and the chain is convex: "a chain end's base or wall is reflex at the corner"; a straight end face that is not normal to its edge and does not meet the wall square: "an open tangent chain's end face does not meet its wall square"; a concave chain's end whose base and `G` are both reflex: "a chain end's base and end face are both reflex at the corner"; a turn there too near straight to read: "a chain end's base or wall turn at the corner is undecided"; an end face whose loop runs its edge with the base the way the base's loop does (a guard: an oriented held mesh never reaches it) | `ErrUnsupported` |
| **SM9** | an open convex chain whose arc end lies on an end face reflex at the corner | `ErrUnsupported` |

Kept refusals: `Shell` (the `shell.go` text, SX9), `Draft` (SD23), `Offset` (R37), the many-body `Union` (text J), crossing cylinders outside blend R7's chamfer and fillet, `Cone`, `Torus` and oblique plane/cylinder rims
(SR1).

Gate order: modify §4 stage 1, reach stage 2 (expansion, SM1), stage 3 (SX3), then SR1, SM3, SM9, SR2 and SM2,
SR3 and SM6, SR6, SM8, SR5, SM5 and SM7, SR4 and SM4, the construction, SR7.

### 11. Readers

- A chain result is a retained body; every retention §4 and §10.3 row applies. Its junction edges join two blend
  faces tangentially. `FaceCreatedBy` finds each blend face by its `fillet(k)` or `chamfer(k)` role.
- An exact-rotation copy answers `NormalAt`, `Facing` and `Face.Surface()` from its tags. `MirrorFace` admits a
  plane only at bound 0, which a plane that `Q` fixes keeps. `ToFace` also needs
  a `CapRecord`, which only such a fixed plane keeps; every other plane's record is dropped.
  A retained blend on the copy refuses at SR1 unless the bound is 0. `ExactCopyMoments` and the motion engine's
  `ExactPlaced` reading are unchanged.
- A scaled body answers the same readers; `MirrorFace` admits a plane whose charge is 0.

### 12. Tests and the red leg each must show

Tests assert on computed geometry. Volumes compare with the result's published bound and normals with
`NormalAt`'s bound, never with a pinned bound literal: float bounds differ between amd64 and arm64. Each test is
shown red first by the change named. `A_f = r²(1 − π/4)` is a fillet's corner area and
`x̄ = r·(10 − 3π) / (3·(4 − π))` its centroid's distance from the sharp corner along either face.

| Test | Asserts | Red leg |
|---|---|---|
| R1 two-distance chamfer (placed-tool plate, one reference face per edge) | the bevel plane runs through the setback `D` on the reference face and `Other` on the other; removed `L·D·Other/2` | swap the §4.1 mapping: the distances swap |
| R2 two-distance chamfer, reference the plane | removed `π·dA·dB·(R + dA/3)`; the `Cone` leans by `atan(dA / dB)`, starts `dB` below the plane and has radius `R` there | use `D` on both faces: the half-angle and volume are wrong |
| R3 two-distance chamfer of a groove's top edge | the wall's setback generator lies at angle `dB / R` from `K` | keep `solveSection`'s `dA` for the wall: the angle reads `dA / R` |
| R4 two-distance chamfer of an arc | as R2 over the arc's span | as R2 |
| Two R1 edges, two reference faces, one call | each edge's setbacks match its own reference | resolve references by face pointer: the second step finds no reference |
| Chain expansion on P2 | a top-front seed with `WithTangentChain` expands to 4 `Line3` and 4 `Arc3` in body order | keep the faceted gate in `expandTangentChain`: SX9 |
| Closed chain fillet on P2, r = 1 (C1) | 4 `Cylinder`, 4 `Torus` (`Major = R − r`); removed `A_f·(96 + 2π·(R − x̄))`; junctions `Arc3` of radius `r`; `VerifyAll` passes | build each junction row per edge: rows differ in the last bits, SR7 refuses |
| Same, torus side | as above | take `κ` the other way: `Major = R + r` and the volume is off |
| Closed chain chamfer, equal and with reference `P` | removed `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))` | let the arcs take `t_W` along `P`: the two-distance volume is off |
| Two closed chains (top and bottom outline), reference naming both bases | each chain removes `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))`; the 8 corner faces are `Cone` of radius `R − t_P` and half-angle `atan(t_P / t_W)`, the 8 straight bevels `Plane` | a later chain loses its reference: `the reference face is not a face of the selected edge` |
| Four x-parallel seeds | two closed chains; removed volume twice the C1 row's | skip merging overlapping chains: the corners are claimed twice and SR2 refuses |
| One straight edge, no option | SM2 naming `WithTangentChain` and the junction point | drop SM2: the message is blend SR5's "not a retained plane" |
| All 8 edges named, no option | builds the C1 result; volume equal within both bounds | treat shared junction corners as SR2: refused |
| Open chain (C2): one lateral fillet, seed its top edge | chain of line, arc, line; removed `A_f·(L1 + L2) + (π/2)·(R − x̄)·A_f` | stop the walk at the junction and use the R1 end rule: refused as grazing |
| Open chain ending on an arc (C2, R4): the corner-rounded box trimmed by a plane through its arc; seed the arc | line and arc ending on `G`; removed volume `A_f·L + ∫∫ ρ·Δφ(ρ)` over the section's corner, `Δφ` the angle from `G` to the tangent point | start every row at the corner's azimuth: the end rows leave `G` (unit test) |
| Concave open chain ending on arcs (C2, R4, `faceted_retained_blend_concave_arc_end_test.go`): the slot `−2…30` with four `r = 3` corners, cut flush with its mouth face (floor `z = 5`, the arcs' centres 1 mm from `G`), fillet and chamfer of the floor; the same pocket with its end wall at `x = 10` (blind end); a boss root against a wall (`κ = +1`, blind end) | volume added `A_f·ΣL + ∫∫ ρ·Δφ(ρ)` with `Δφ = asin(1/ρ)` at each end arc and `π/2` at each quarter arc, the end face's area growing by two caps where the slot runs out and falling by two where the end is blind (`requireFaceAreaChange`, independent sums); `VerifyAll` passes; a box floating across the cap the slot's face takes refuses "the cap an end face takes meets another facet"; a blend circle that misses `G` refuses | keep the convex-only gate: the slot and the pocket refuse SM9; give a concave end's cap the wrong sign: the face areas fail; root the end rows on the far side: the volume fails |
| Concave closed chain: pocket floor (40×40×10 plate, 20×20 `r = 3` pocket, floor `z = 5`, `ΣL = 56`), fillet `r = 1` | fill `A_f·(ΣL + 2π·(R − x̄))`; 4 `Cylinder`, 4 `Torus` (`Major = R − r`); every blend's `NormalAt` points into the void; `VerifyAll` passes | restore the concave refusal: refused; convex rows: SR7; an unreversed cylinder tag: the normals point into the material |
| Concave closed chain chamfer, equal and with reference the floor | fill `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))` | the convex chamfer normal `n_P − m`: SR7; swap the setbacks: the two-distance fill is off |
| Concave closed chain: boss root (20×20 `r = 3` boss joined to the plate's top) | fill `A_f·(ΣL + 2π·(R + x̄))`; 4 `Torus` (`Major = R + r`) | take `κ` the other way in the arc tag: SR7 |
| Concave chain, all 8 edges named | builds without the option; the same fill | none |
| Chain strip clearance | a 2×1×1 box joined over the pocket floor with only its lower front edge in the fill, and a cavity under the cross-drilled box's top-front edge with only its upper front edge in the removed corner, both refuse SM5 "the blend meets another facet of the body" at a point with `x` in 15…17; unit tests check that the grid keeps every pair of meeting boxes (random, degenerate-axis, far, touching and boundary boxes) and that a crossing, a coplanar overlap and a flat facet refuse | drop the strip test: both build; report a strip vertex: the point lies outside the solid; visit one grid column, or convert the cell before clamping (amd64): the grid loses pairs; read a region contact's point: panic |
| Single-edge strip clearance | a cavity whose corner reaches the corner the blend removes refuses blend SR7 "the blend meets another facet of the body" at a point inside the cavity's `x` range, for a plate's top edge (R1, `r = 2`), a mitre run of two top edges, a hole mouth (R2), a broken-out arc, a groove's edge (R3) and the ridge between two bores (R5); a top-edge cavity at `y` 3.25…4.25, `z` 18.75…19.75 and one at `y` 1.99…4, `z` 16…19.995, whose corner lies inside the box of the strip's first facet but under its chord so the exact test runs on the cavity's facets, both build and remove `40·4·(1 − π/4)` | call `build` for one route in place of `buildClear`: that route's cavity builds; count every tested pair with a cavity vertex as a contact: the near cavity refuses and the far one still builds |
| Concave chain gates | a pocket 0.8 deep refuses SR4 "the blend consumes the wall"; `r = 3` refuses SR3; a cube floating inside the fill refuses SR7 by nesting parity while one clear of it builds; a chain of mixed sides refuses SM3 (unit test) | classify by `n_W = −m` only: every concave row refuses; measure the wall band on the convex side: the pocket refuses; drop the wall clearance: the shallow pocket refuses with another text; drop the mixed-side check: the unit test builds |
| Chain ends: reflex foot (wall `0…10 × 0…40 × 0…30` joined with a ledge `5…30 × 10…30 × 4…10` whose `x = 30` corners are `r = 3`), fillet `r = 1` | removed `A_f·(48 + π·(R − x̄))`; 3 `Cylinder`, 2 `Torus` (`Major = R − r`); the `x = 10` face's area grows by `2A_f` within its bounds and the face holds a point just inside each cap; `VerifyAll` passes | keep a reflex `G` refusing: "an end face's corner does not match the edge's convexity"; walk `G`'s loop the same way at both ends: "an end face leaves the chain's corner along neither of its lines" |
| Chain ends: run-out (the slot of `roundedPocket` leaving through `x = 0`), fillet and chamfer `1` | fill `A_f·(68 + π·(R − x̄))` and `(1/2)·(68 + π·(R − 1/3))`; the `x = 0` face grows by two caps and holds a point just inside each; blend normals point into the void | keep a reflex `G` refusing: refused; rebuild each end alone: the strip meets the duplicated `G` facets |
| Chain ends: blind end (pocket `10…30` with only its `x = 30` corners `r = 3`, floor `z = 5`), fillet `r = 1` | fill `A_f·(48 + π·(R − x̄))`; the `x = 10` wall shrinks by `2A_f` and lies more than 0.1 from a point just inside each cap | admit only a reflex `G` for a concave chain: refused; drop the whole-loop base branch: "a face boundary holds no run to replace" |
| Chain ends: tongue (40×20×20, `x = 40` corners `r = 3`, bored), fillet `r = 1`, both ends on `x = 0` | removed `A_f·(88 + π·(R − x̄))`; 3 `Cylinder`, 2 `Torus` (`Major = R − r`); the `x = 0` face shrinks by `2A_f` and gives up both caps | drop the whole-loop base branch, or rebuild each end alone |
| Chain ends: base wrap (plate `30×26×10` joined with a block `10…20 × 10…16 × 10…16`, `x = 20` corners `r = 2`), fillet `r = 1` and chamfer `1` of the root along `y = 10` through both corners | fill `A_f·(18 + (π/2)·2·(R + x̄))` and `(1/2)·(18 + π·(R + 1/3))`; 3 `Cylinder`, 2 `Torus` (`Major = R + r`); the `x = 10` face grows by `2A_f` (`2·(1/2)`) and holds a point just inside each cap; `VerifyAll` passes. With only one corner rounded the two ends lie on two faces: fill `A_f·(12 + (π/2)(R + x̄))`, the `x = 10` face grows by `A_f` | keep a wrapping base refusing: SM8; leave the cap out of the taken caps: the box across it builds; drop the kept corner from the base's contour: the build refuses |
| Chain ends: step and ramp (`faceted_retained_blend_chain_ends_test.go`, `faceted_retained_blend_ramp_end_test.go`): the step (block 40×20×30 with `x = 40` corners `r = 3`, cut to `z = 10` for `x > 10`), fillet `r = 1` of its top outline; a 20 mm arm 17 tall with a round end, cut to 8 beyond a 30° ramp, chamfer `0.5` and fillet `0.5` of the outer face's outline, against the same arm with an upright riser at the ramp's foot; the arm with both caps chamfered, its outline and the ramp's top edge in one call; the ramp turned 0.05 rad about z | the step removes `A_f·(68 + π·(R − x̄))` and its riser grows by `2A_f`, holding a point just inside each cap; the ramp chamfer removes `(t²/2)·(2L + π·(R − t/3)) − 2κ̄` with `κ̄ = (t²/2)·(t/3)·cot 30°`, exactly `2κ̄` less than the riser's to 1e-9, and the ramp grows by `2·(t²/2)/sin 30°`; the ramp fillet removes its closed form within the bound and the ramp grows by `2A_f/sin 30°`; the one call removes what two calls (top edge, then outline) remove; `VerifyAll` passes; the turned ramp refuses "an open tangent chain's end face does not meet its wall square" | keep the wall's turn convex-only: the step refuses SM8; keep `G` normal-only: every ramp case refuses; drop `obliqueEndAreas` or `capArea`'s cosine: the published area bound misses the ramp's change; keep open chains out of a mixed selection: the one call refuses "a selection that holds a tangent chain names only chain edges" |
| Chain end gates | the blind pocket's mouth (a convex chain) refuses SM8; a unit test refuses a wrapping base beside a reflex `G` and a wrapping base beside a wall that wraps; a ledge `8…30` with four corners refuses SM9 for its convex arc ends on the reflex wall; unit tests read convex, reflex and undecided turns, `G`'s turn at both ends of the tongue and the reflex foot, and the cap's sign | drop the base's turn: the mouth builds a flat-ended notch; drop the arc gate: the ledge builds; read `G`'s turn one way at both ends: the unit test fails; drop the cap's sign: the unit test fails |
| Taken cap clearance | a 0.1 box joined across `x = 10` beside the base-wrap block's corner refuses SM5 (the box moved clear builds), a second block standing in the base's setback band refuses SR4 "the blend consumes the base face"; a 0.1 box joined across `x = 0` in the run-out slot's void, and a 0.1 cavity cut across `x = 10` in the reflex foot's ledge corner, each at either corner, refuse SM5 "the cap an end face takes meets another facet of the body"; the box moved clear of the cap builds with the run-out's fill; unit tests refuse a facet that crosses a taken cap but not the strip, one that shares a held vertex with the cap and crosses it beyond, and one coplanar with the cap beside a shared edge, while facets meeting the cap only at a shared vertex or along a shared edge pass, and `checkEndSense` refuses an end face's loop sense that matches the base's | skip the taken caps: the box builds, and the cavity refuses SR7 "a result shell has inconsistent orientation"; skip every pair that shares a vertex: the held-vertex unit case builds; refuse every contact of a sharing pair, or every point contact: the run-out, its chamfer and the reflex foot refuse; drop the loop-sense test from `checkEndSense`: its unit test builds; drop the call from `rewriteChain`: nothing goes red, because no oriented held mesh reaches it |
| Single-edge taken cap clearance | a 0.1 box joined across `x = 10` (or `x = 20`) beside the base-wrap block's sharp corner, filleted as a single edge (R1, base wraps), a 0.1 box across `x = 0` in a straight run-out slot, a 0.1 box across `x = 10` on a rod lying on the plate (R3), a 0.1 × 0.1 × 1 box across `z = 15.001` beside the valley of two bores (R5) and a 0.1 box across `x = 0` beside a blind bore's floor arc that breaks out through two faces (R4) each refuse blend SR7 "the cap an end face takes meets another facet of the body at" a point on that plane; the same boxes moved clear build with the fill's volume and a mesh with no pair of crossing triangles; each crossing box has its probe point outside the fill, so the held audit alone does not refuse it; `TestLineEndFaceTurn` reports a gain for every end of a concave edge | revert `endFaceRun`'s `takes || !convex`: the R1 and run-out boxes build and the unit case fails; drop `gainsG` from the R3 end entries: the rod's box builds; drop the end ranges from `rewriteArc` or `rewriteCylCyl`, or call `buildClear`: the arc's or the valley's box builds |
| Concave chain on a notched round (C2, R4, the wall band, `faceted_retained_blend_notched_round_test.go`): the 40×20×20 block with its bottom long edges rounded `r = 3` and short ones `r = 2`, a pocket cut in its top, an 8×12×7 box ear unioned under it; fillet `r = 1` and chamfer 1 of the ear's side root (N1: the floor line to the back face `G0`, then the arc on the round to the front face `G1`), its back root over the end round (N2) and its side root on the round alone (N3) | the result is a solid and `VerifyAll` passes; volume added `A_f·L + ∫∫ ρ·Δφ(ρ)` with `Δφ = asin(h/ρ)`, `h` the distance of `G1` from the axis; `G0` grows by `A_f` and `G1` by `∫∫ ρ/√(ρ² − h²)`, each within `AreaSlack`; one blend `Cylinder` and one `Torus` (`Major = R + r`); the bound at most the held one plus `2τ`; the round's vertices outside the band's zone are the held ones exactly; no pair of crossing triangles. A 0.6 hole in the round inside the band and an end round whose band reaches the corner patch refuse "another part of the wall lies within the blend's reach of the edge"; a 0.15 box across `G1`'s cap refuses "the cap an end face takes meets another facet of the body"; unit tests read a wall that extends at a concave chain's end, refuse a base and a wall that both extend, a reflex `G` beside an extending wall, and give the cap's sign | keep the whole-face rim side: "the rim's wall does not lie on one side of its plane"; keep the height cut: the same text; drop the wall's turn from `checkEndTurns`: the unit case builds; drop `G`'s cap from `takesCap`: the face areas fail |
| Ears on one back plane (the notched round block with two 8×12 ears whose back faces share `y = 13.5`, their fronts at `y = 1.5` and `y = 2`, 10 apart; `faceted_retained_blend_notched_round_test.go`): fillet `r = 1` of one ear's two side roots in two calls, in either order, of all four roots in one call, of ear B after ear A, and of each root in its own call | each result is a solid and `VerifyAll` passes; volume removed is the sum of each root's N1 fill; one `Cylinder` and one `Torus` per root; the bound at most the held one plus `2τ` per root; the first blend's cut vertices on the back plane are still vertices of the result. Ears 1.5 apart on the back plane build one side root: the band ends 0.5 short of the other ear. The same root twice refuses `ErrNoMatch`; unit tests (`cyl_band_internal_test.go`) skip a facet outside one plane, clip a corner that another plane drops before the plane that cannot read it, read a corner no plane drops by sign, and refuse a corner two planes leave undecided | read the end planes before the others: the four-root and sequential cases refuse; drop the deferral: the unit case refuses |
| Ears on two shared planes (the same block with ears whose back faces share `y = 13.5` and front faces share `y = 1.5`, 10 apart; `faceted_retained_blend_notched_round_test.go`): fillet `r = 1` of one ear's side root, of the other ear's, of an outer one, of all four in one call, of two facing roots 3.5 apart in one call, of two facing roots in turn, in either order, and of one root of ears 1.5 apart on both planes; the same with the second ear's back face one float step off the back plane | each result is a solid and `VerifyAll` passes; volume removed is the sum of each root's N1 fill with `h = 1.5`; one `Cylinder` and one `Torus` per root; the bound at most the held one plus `2τ` per root, also where the band reads a vertex by sign (its charge is capped by the vertex's distance to the setback line). Refusals (`ErrUnsupported`): facing roots 1.5 apart whose bands overlap, in turn and in one call, facing roots 0.8 apart, and a 0.6 hole in the round inside the band, each with "another part of the wall lies within the blend's reach of the edge"; unit tests (`band_provenance_internal_test.go`) read two tags exactly (a step off, a tilt, a departure, a delta or a cylinder never match), admit the boundary vertices of equal-tag faces and not a fan's inside, cut a facet along an end plane between two vertices on it (the cuts, the corner, the run, no cut in the wall's row) and split a facet across such an edge | drop the provenance: the single roots refuse "a vertex of the wall lies on an end face's plane"; drop the corner or the run: the band refuses "does not run from the setback line to the corner"; drop the split: the strip meets the facet at the cut vertex; read a tag one step off as on the plane: the step-off case is no longer charged (the unit case `TestSignReadChargesHeldVertex` fails) |
| End plane read by sign (`TestFacetedRetainedBlendEndRoundChordRow`, `cyl_band_sign_internal_test.go`): N2's back root with the ear's outer side placed 0.0005 or 0.002 off a row of the end round's vertices | the result is a solid, `VerifyAll` passes, the volume change matches N2's fill with `h = 2 − x₀` within the published growth, and the bound grows by at most `2τ` plus four times the held bound and the offset over the round's slant sine. Unit tests put a held vertex and a cut point within their bound of an end plane at an offset `d` (inside, outside, a float step) and require the bound to grow to `(β + d + slack)` over the slant sine (`raised` too, for a held vertex), the route's slack to be charged, a crossing to take the ends' input bounds, not their charges, a cut point on the setback plane near the end plane to publish the setback bound and a held vertex to keep the slant bound, a slanted end plane (sine 1 to 0.2, the true wall either side of the held facet) to publish a bound that reaches the true boundary along the wall, a cylinder wall (the foot on the band's side, the point inside and outside the circle, the axis on either side) to publish `β + c/s_lo` at least the exact distance to the kept arc (`TestSlantBoundCoversCylinderExactly`), `wallSine` to read a plane, a cylinder (axis either side, a bore) and no other wall, a sine below `minSine` to refuse, a vertex that another plane drops or that a tag puts on the plane to stay uncharged, and an angle-main band, a main plane and a further plane to refuse as before | drop the held charge: `TestSignReadChargesHeldVertex`; drop the cut point's: `TestSignReadChargesCutPoint` and the 1.5-apart apitest; drop `raised`: the held case; drop the slack: `TestSignReadChargesSlack`; take the ends' charged bounds in a crossing: `TestCutOfHeldEdgeReadsInputBounds`, `TestCutFacetBoundReadsInputBounds`; drop a term of the setback bound or its wiring: `TestSetbackBoundFormula`, `TestRoutesSetTheWallLean`; drop the slant term: `TestSignReadBoundCoversSlant`, `TestSlantBoundCoversCylinderExactly`; accept an end plane with no proven sine: `TestSignReadRefusesBelowMinSine`; read by sign wherever a plane refuses: the byte hashes of the retained-blend builds change |
| Chain clearance | a pocket whose chord crosses a band while its vertices lie outside it refuses SM5; a hole deep inside a band refuses SM4 | test vertices only: the crossing chord builds and SR7 or the volume check fails |
| `internal/tangentchain` retained oracle | exactly tangent plane/cylinder tags prove "tangent"; tags a few float steps off prove it too (round-off); tags 1e-12 relative off are undecided; a clear miss is "not tangent" | widen round-off to the angular band: the 1e-12 case proves "tangent"; drop it: the one-step case is undecided |
| Exact-rotation copy of P3, 20° | 6 `Plane`, 5 `Cylinder`; `NormalAt` at an interior facet centroid within its bound plus 64 float steps of the rotated source normal; bounds below 1 | keep `ExactGroupPlaced`'s drop: `Faceted` |
| `facetedrecord` exact-copy charge | a plane whose image under a 20° `Q` rounds has `NormalBound ≥ 8a + P` recomputed in the test, and above 0 | charge 0: the relation fails |
| Exact-rotation copy of P3, 20°, ±z caps | bound 0; `ToFace` stops at exactly z = 20 and `MirrorFace` across the top cap mirrors to the exact z range; a tilted rotation still refuses | drop the fixed-plane arm in `ExactGroupPlaced`: both refuse |
| Cut of the 20° copy | untouched planes and cylinders keep their tags | clear `Retained` in `ExactGroupPlaced`: the Cut loses them |
| `ScaledAt` of P2 by 2 and by 1.1 | 6 `Plane`, 5 `Cylinder`; `Radius` is `2·R` at `s = 2` and within its rounding of `1.1·R`; `NormalAt` bounds below 1 | keep the drop: `Faceted`; keep the plane's 2: bound 2 |
| `facetedrecord` scale charge | a cylinder at `s = 1.1` has `NormalBound` at least the held term at `2·(P + ρ)` recomputed in the test | charge 0: the relation fails |
| R5: two overlapping parallel bores, placed-tool cuts | the bore–bore ridge is `Line3`; fillet and chamfer change the volume by `L` times the section's corner area; the fillet is a `Cylinder` of radius `r` through the ball's centre | flip `κ1`: the centre sits on the wrong side of `C1` |
| R5 refusals | a valley between bosses standing on a plate (ends of unlike corner type) and bores with crossing axes refuse with `ErrUnsupported` | drop the end-corner gate: the valley blend builds |
| Kept refusals on P2 | `Shell`, `Draft`, `Offset` and the many-body `Union` keep the texts of §10 | none; pins the refusals |

### 13. Files and fallout

| Area | Change |
|---|---|
| `internal/retainedblend/route.go` | `Chamfer` takes a two-distance spec with per-target reference tags (§4.1); group selected edges into chains; SM2, SM3 |
| `internal/retainedblend/line_rewrite.go`, `arc_rewrite.go`, `rewrite.go` | setbacks from the mapping; an end at a junction plane in place of an end face |
| `internal/retainedblend/cyl_rewrite.go` | the wall setback reads `dB` (§4.1) |
| `internal/retainedblend/chain.go`, `chain_rewrite.go` | Table RC, §5.2 junctions, Claim J rows, §5.5 construction, §5.6 gates |
| `internal/retainedblend/cyl2.go`, `cyl2_rewrite.go` | R5 |
| `internal/retainedblend/chain_cross.go`, `chain_cross_face.go`, `chain_cross_end.go`, `chain_cross_bevel.go` | §15b.1, the exact chamfer of a chain across planes |
| `internal/brepmodify/chamfer.go`, `asymmetric.go` | pass the resolved reference into `retainedblend.Chamfer` |
| `internal/tangentchain/` | the retained continuation test of §5.3 |
| `brep_modify_partial.go` | admit a retained `facetedPayload` and dispatch the retained test |
| `internal/facetedrecord/place.go`, `place_tag.go`, `placed.go` | §6 |
| `internal/facetedrecord/point_plane_scale.go`, `place_tag.go` | §7 |
| `internal/facetedrecord/edge_curve.go` | E10 |
| `document.go` | `WithCopyRotation`'s doc comment states the tags it keeps (§6) |
| Part E | Table RR: the two-distance chamfer, R5, a pointer to this document's chains; §7 |
| `agent-docs/faceted-source-retention-design.md` | §3.4 (exact rotation), §6 (`ScaledAt`), §10.1 (E10) |
| Part R | RX6 row and SX9 text; §5's sentence that faceted curves never enter the oracle |
| `agent-docs/missing-features.md` | the rows for faceted `Fillet`/`Chamfer` and for analytic readings of a `WithCopyRotation` copy |
| `apitest/` | the tests of §12; `apitest/faceted_retained_blend_test.go` holds the blend rows |
| `examples/` | one executable example: the rounded box's top rim filleted after a placed-tool Cut |
| `docs/api.md`, `docs/data-structures.md` | none: no exported name or signature changes, and their rows name no receiver class |

### 14. Increments

Each stage below is built. Its gate is the §12 rows named.

| # | Built | Gate: rows that pass |
|---|---|---|
| M1 | §4: two-distance chamfer on R1–R4 | the five two-distance rows of §12 pass; `go test ./...` on CI |
| M2 | §6: exact-rotation `Copy` keeps its tags (independent of M1) | the four exact-copy rows pass on CI |
| M3 | §5.2–§5.7 for C1: retained expansion, junctions, closed chains, SM1–SM5 | the expansion, C1, chamfer, seed-merge, SM2, all-8 and oracle rows pass, and the example's output is verified |
| M4 | C2: open chains with blend §5.0 straight ends; E11 for the fillet junction `Arc3` | the C2 row passes |
| M4b | C2: open chains whose first or last edge is a blend §4.4 arc | the arc-end rows pass |
| M5 | §7: `ScaledAt` keeps curved tags | the two scale rows pass |
| M6 | §8: E10 and R5 | the R5 row passes |
| M7 | C1 concave: the chain side `s` of §5.1 in the rows, wall bands and tags | the concave chain rows of §12 pass |
| M4c | a single concave R4 arc whose wall extends past its end planes, read by the local side of blend §4.4 | the `arc_extends` rows pass |
| M8 | C2 end corners: open concave chains, reflex feet and blind ends, the same-face end rebuild and the whole-loop base contour of §5.5 step 5; SM8, SM9 | the chain-end rows of §12 pass |
| M9 | C2 concave chains whose walls continue past their end planes or behind the base: the ear roots of a notched round, by the wall band of §5.5 step 5 | the `notched_round` rows pass |
| M10 | C2 straight ends on an oblique `G` that meets the wall square, and convex chains whose walls continue past `G` (§5.5 step 5); an open chamfer chain beside other edges in one selection (charged-window-chamfer §3.4) | the step and ramp rows pass |

Chains are built for closed and open outlines of lines and arcs, convex or concave.
`retainedblend.Tangent` decides each junction (`chain_tangent.go`), `planChains` groups and orders the selection
(`chain.go`) and `rewriteChain` builds one chain (`chain_rewrite.go`); the ends of an open chain live in
`chain_open.go`, an arc end in `chain_arc_end.go`, a wall that lies on both sides of the base or extends past an end
plane in `chain_band.go`, and the strip's clearance from the rest of the body, for every
route, in `strip_clear.go`. A closed chain has at least three edges and an open chain at least two. A straight end
whose wall continues past a reflex `G`, or whose base is reflex on a convex chain or beside a reflex `G`, a straight end on a face that is not normal to its edge and does not meet its wall square, an arc end of
a convex chain on a face reflex at the corner and an arc end on a face that is not parallel to the
axis are refused. The edge between a `Cylinder` and a `Torus` blend of a fillet chain is an `Arc3`
(retention §10.1 E11).

### 15. Rejected shortcuts

- **Admit near tangency by a tolerance.** Reach §5 lets a tolerance prove "no" only. Claim J needs equal sections,
  which near tangency does not give.
- **Blend each chain edge on the evolving mesh.** Blending one edge alone leaves a flat end cap in `Π_J`; the
  next edge's blend then removes that cap to zero area, which no end rule builds.
- **Expand a `Fillet`'s tangent chain without `WithTangentChain`.** Cardinality applies to what the caller named
  (reach §2). SM2 names the option instead. `Chamfer` expands it (§5.3), as the target CAD's chamfer does.
- **Re-derive the copy's or the scaled body's tag from its moved polygons.** It blesses a surface from the
  polygons it should be checked against (CLAUDE.md).
- **Scale or rotate a tag and set its bound to 0.** The origin and radius round; a zero bound would claim the
  rounded tag is the true surface.
- **Shell by a Cut with an inward-offset copy of the held mesh.** A vertex-normal offset of a mesh is not the
  erosion and carries no proof.
- **Rebuild the operands of a mesh Cut from tags and roles.** Payload §5.2 forbids rebuilding from roles, and tags
  carry no trims.

### 15a. Whole loops with sharp corners

The per-edge routes above build an edge whose ends meet planes, and a closed tangent chain (§5). A closed loop of
lines and arcs whose corners are not tangent, such as a window of two lines and two arcs in a cover, has an edge end
that meets a cylinder wall, which `line.go`, `arc.go` and `cyl.go` refuse ("an edge end meets a face that is not a
retained plane"). `brepmodify.RetainedLoopChamfer` (`internal/brepmodify/retained_loop_chamfer.go`) builds it when
`retainedblend.Chamfer` refuses with `ErrUnsupported` or declines:

- The selection is exactly whole boundary loops of retained planar faces of a faceted body. Each loop is a chain of
  lines and arcs, or one circle. Each wall beside it is a retained plane normal to the face or a retained cylinder
  with its axis normal to it, and every edge is convex. Anything else declines and the earlier refusal stands.
- Circle loops go first through the retained rim route, because a cutter of a large circle needs more chord cells than
  a loft allows. Their edges in the result are found again by their ends and curve kind.
- Each remaining loop is cut by a polyhedral cutter, held exactly as a mesh solid and cut with a mesh Boolean. At
  height `h` above the face the cutter's section is the loop moved `d + h` into the material with a mitre at each
  corner, so the bevel is 45° and meets the wall at depth `d`. The default cutter takes its sections at six equal steps from `d/2`
  above the face to `d/2` below the bevel's far end, and samples each arc so a chord lies within 0.002 mm of its
  circle. A face's outer loop takes the ring between a rectangle and the moved loop; a hole loop takes the region
  inside it. Between two steps the cutter is ruled, so at a sharp corner beside an arc the bevel patches meet along a
  straight ruling where the exact patches meet along a conic, and an arc's cells twist as the corner turns about its
  centre. Every cutter vertex carries the chord sagitta plus the layer departure the shared closed form proves
  (`chamferRun.layerDepartureWith`, the partial route's `layerDeparture` with the cutter's own sagitta, modify-reach
  §8.3.2), over the layers that meet the material: zero for a loop whose arcs meet their neighbours tangentially. The
  default cutter's bound can rise at sharp corners beside an arc and equals the chord sagitta on tangent loops. A
  lofted cutter of two sections is not used: its twisted arc cells carry a larger facet bound, which the Boolean's rim
  rule `(δ_A + δ_B)/sin θ` turns into a larger body bound; the polyhedral cutter leaves the body's bound at the
  value its target already had.
- The cutter above the face and beyond the bevel's far end removes no material: it keeps the caps clear of the face
  and the walls. Moving the loop on past its narrowest feature (a dart whose width is twice the reach, a convex round
  whose radius the reach passes) crosses the moved loop over itself. Each of the two overshoots is therefore halved,
  up to ten times, until the section there is a simple ring (no two non-adjacent chords meet); the section at the face
  and at the bevel's far end must be simple, and every section between them is checked too.
- A receiver held finer than that cutter (small rounds beside a small bore) refuses it at the Cut's chain-depth gate.
  Every attempt with the default cutter runs first, so a chamfer that builds with it keeps its geometry; then each
  attempt runs again with a cutter built within the receiver's own mesh bound `B` (`chaintol`, or the faceted body's
  `MeshBound`): the chord sagitta at most `B/2`, and the layers stepped up from six, to at most 256, until the layer
  departure is within `B/2`. A loop whose fine cutter equals its default one is cut with it, and a fine attempt whose
  every loop's cutter does is not run again. A cutter past the loft audit's
  triangle ceiling is refused before its rings are built.
- The setback reaching the far end of a wall is `ErrUnsupported`, and one that empties a loop or crosses it over
  itself (a dart narrower than twice the setback) is `ErrDegenerate`.
- The cut allows the chain-depth gate the body's own mesh bound (`chaintol` on the context, which the document's
  tolerance never lowers). The cutter meets only the face and the walls beside the loop, so the rims it makes compose
  from bounds those facets already carry. Without it the gate compares them with a chord tolerance that follows the
  pair's size and refuses a face whose facets reach a coarse circle elsewhere, such as a boss ring on the cover.

Part of a loop, as whole runs ending at sharp corners, takes `brepmodify.PartialLoopChamfer` after the routes above
refuse or decline it, on a faceted body whose face is a retained plane: see Part R §8.3.2.

The result is a Boolean Cut: faceted, with the Cut's own readings. Tests are in `apitest/chamfer_planar_loops_test.go`:
windows of a cover before and after a boss is joined (both faces, with the rim and bore in one call), refusals for a
setback past the walls and for part of a loop, and measured removed volume against `L·d²/2`.
`apitest/loop_chamfer_narrow_feature_test.go` chamfers a watch lever's fork, a mesh result, whose dart and corner rounds
the default cutter's overshoot crossed and whose fine mesh the default cutter's bound exceeded.

### 15b. Tangent chains across planes

The chain route of §5 builds a chain whose edges all border one base plane. A chain whose junctions have different
bases has none: a bar rounded about z at one end and about y at the other, whose rim runs round both caps and both
sides; a slot whose rim runs along its sides, round the bar's end on both sides and round the slot's end on the top
and bottom faces; a step whose floor's rim runs on round the step's round and up its wall. §5's base search finds no
face for them ("the chain's edges share no base plane", SM3). An equal-setback `Chamfer` of such a chain is rewritten
exactly on the held mesh by `rewriteCrossChain` (`internal/retainedblend/chain_cross.go`, `chain_cross_face.go`,
`chain_cross_end.go`, `chain_cross_bevel.go`; §15b.1), with or without `WithTangentChain`. When the rewrite refuses
with `ErrUnsupported`, `brepmodify.ChainCutterChamfer` cuts the chain (§15b.2). A fillet and a two-distance chamfer
keep SM3: a two-distance chamfer's setbacks belong to the base and the walls, which swap where the base changes.

#### 15b.1 Exact rewrite

Accepted input:

- The receiver is a solid mesh Boolean result. The call is a `Chamfer` with one setback `d` and no reference.
- Every chain edge is convex and reads exact tags: a line between two retained planes, or an arc between a retained
  plane and a cylinder about its normal (a convex rim, blend §4.2). No face of a chain edge states a charge.
- The edges group into chains whose junctions §5.2 proves tangent, and no face borders every edge of a chain (such a
  chain belongs to §5). Each junction proves its walls normal to its base, so both faces of every edge meet square.
- A chain is closed, or open with each end at a straight edge whose end vertex is a corner of exactly three faces:
  the edge's planes `A` and `B` and an end plane `G`, whose two edges at the corner are convex. `G` faces along the
  edge, `n_G · t > minSine` with `t` the edge's direction out of the chain. `G` is an exact plane, or one that states
  a charge: a displacement, or an oblique plane's tilt (at most `maxEndTilt`) and extent (blend §6.1, "An oblique end
  plane").

Construction:

1. **Rows.** Each junction `J`, with base `P_J` and walls meeting along `g`, gives one row: `a_J = V − d·n_g` on
   `P_J` and `b_J = V − d·n_P` on `g` (Claim J's chamfer row with `t_P = t_W = d`). The row is symmetric in the base
   and the walls, so an edge whose faces are base at one junction and wall at the other takes each row as it stands:
   its vertex on face `X` at `J` is `a_J` where `X = P_J` and `b_J` otherwise, which is `V − d·n` of the edge's other
   face in both cases. Each row is computed once and both edges at `J` share it.
2. **Bevels.** A line's bevel is the plane quadrilateral of its two end rows, and each of its faces loses a strip of
   width `d`. An arc's bevel joins its columns on the plane, at the setback circle and spaced as §5.5 step 2, directly
   to the cylinder's cut along the setback plane, which runs from `b_J` at one junction to `b_J` at the other, as a
   chain chamfer's strip on one base does. A separate row on the cylinder would put the cut of a held generator
   outside the row's chords, and the cylinder's facet would then meet the bevel. Each bevel is its own group with
   role `chamfer(k)`, tagged with its plane, or blend §4.2's cone. Both read only the chain's exact faces, so they
   are published with no charge.
3. **Plane faces.** Each plane face gives way along each run of consecutive chain edges on its loop. The run's held
   vertices, from its first chain point to its last, are replaced by the run's setback contour: the face's row vertex
   at each chain point and, where the face is an arc's plane, the arc's columns. At a run end that is a junction the
   face is a wall, and its seam with the next wall runs from `V` along `−n_P`: the seam's vertices nearer than `d` go
   and `b_J` takes their place. At a free end the face's edge with `G` loses its vertices nearer than the column
   point. The face is triangulated again over its loops (blend §5 step 2); a face whose runs remove its whole loop
   takes the contours alone.
4. **Cylinder faces.** Each cylinder face is cut at the setback plane of each arc it borders, one arc after the
   other: the round at a bar's end borders its top rim's arc and its bottom rim's. The seam at each end of the arc is
   the face's boundary edge on `g` that the plane crosses, and its cut is `b_J`, which the face across the seam takes
   too. An inner edge near `g`, such as a sliver's diagonal, is no seam.
5. **Ends.** The column at a free end `V` is the segment between `q_A` and `q_B`, the setback points `V − d·n_B` on
   `A` and `V − d·n_A` on `B` slid along the edge onto `G`; `q_X` lies on `X ∩ G`. `G` gives up the triangle `V`,
   `q_A`, `q_B`: its corner and the vertices of its chains along `A ∩ G` and `B ∩ G` nearer than the column points
   give way to the column, and `G` is triangulated again.
6. **Held vertices on a straight boundary.** A vertex inside a seam or inside an edge `X ∩ G`, bordering only that
   line's two faces, is no corner of either: it goes or stays by its held place along the line, which moves no part
   of the solid. One closer to the setback than the margin refuses. A corner of a face (a vertex of three faces) on
   such a line stays where it lies beyond the setback by four times its bound plus the margin, and otherwise refuses,
   except at a setback point exactly at a corner of `G`.

**A setback point exactly at a corner of `G`.** Where an open chain ends on an earlier bevel of the same setback (a
step's wall edges ending on the bevel of the top rim, a tongue's shoulder edges ending on the cap's bevel), the old
bevel `G` meets the wall `X` square and passes through the corner where `G`, `X` and the face `T` beyond meet, at
setback `d` from the chain's other face. `q_X` is then that corner: the column takes the held corner as its vertex on
`X`, the setback contour on `X` ends at it, `G`'s loop runs from the other column point to it, and nothing of `X ∩ G`
remains. The coincidence is decided by an exact identity on the tags' floats read as rationals: the point where `G`,
`X` and `T` meet equals `q_X` computed from the tags of `A`, `B` and `G` and from `d`. A near miss proves nothing and
refuses ("a setback point falls on a vertex of the end face's boundary"). The falsifier holds the held corner within
its bound plus the slack of that point. `T` is an exact plane or states a charge, as `G` may. When either does, the
tags meet exactly and the true planes lie within their charges of them, so the true corner and the true setback point
lie within twice the amplified charges (`planeCharge`) of each other, and the held corner's bound grows by that. A
mesh Cut cannot build this end: the old corner is held only within the earlier Cut's bound, so the Cut cannot tell
the new bevel's touch there from a crossing ("the operands' held facets come within the chord tolerance").

Gates, each `ErrUnsupported` unless stated:

- A junction's held vertex is not a corner of exactly its base and its two walls, or lies farther from `V` than its
  bound plus the slack.
- A run does not follow its face's loop, or a held vertex on a run borders a third face.
- A seam vertex inside the setback borders another face; a seam ends, or a corner of a wall lies, inside the
  setback; the bounds cannot place a vertex against the setback.
- A kept boundary chord of a plane face meets the region a run removes, the face's form of §5.6's base band: the
  run's exact points and its contour back, widened by the chord's vertex bounds, the margin and the sagitta of an
  arc's chords. A chord with an end inside the region consumes the face (SR4, `ErrDegenerate`), and one within the
  widening is undecided (SR5). A chord along a seam or along `X ∩ G` beyond the setback runs on in line with the
  region's edge and passes. A chord leaving a held corner the column takes must leave it outward of the region.
  Beyond such a corner the boundaries of `X` and of `G` run on along their edges with `T`, away from `G` and from
  `X`; a chord along those lines passes too, as a Cut leaves vertices inside them a few micrometres from the corner
  (a sketched stadium's cap edge with the shoulder). Either line that meets `G` or `X` too near parallel to tell
  its way refuses.
- The cylinder cut takes §5.6's wall gates: a vertex on the arc's plane off the arc refuses, and a vertex inside the
  band that lies on no seam refuses (SR4 or SR5).
- The strip test of §5.6 (`checkStripClear`) and the held audit (SR7).

Bounds follow §5.7. The exact areas are a line's rectangles `L·d` on each face and its bevel `L·d·√2`; an arc's
annular sector, wall band and cone over its span as §5.7 states them; and for an end edge, the quadrilaterals each
face loses, the bevel's quadrilateral and `G`'s triangle, each as the polygon of its exact points. A charged `G` adds
`planeAreaCharge` of the pieces that end on it, and an oblique `G` adds `obliqueCapArea` of its triangle, to the
area bound.

#### 15b.2 Cutter fallback

`brepmodify.ChainCutterChamfer` (`internal/brepmodify/chain_cutter_chamfer.go`) chamfers a chain across planes that
the rewrite refuses with `ErrUnsupported`, such as one with a charged wall. It takes every selected edge that is a
convex retained line or arc between two faces with retained plane or cylinder tags (`retainedblend.Chainable`), in
chains that share no vertex and whose open ends are straight edges at a convex corner of three faces. Its end face
`G` is a plane there: its plane tag, or its held facets at the vertex when it is faceted, all coplanar, and it faces
away from the chain, `n_G · t ≥ 10⁻³`.

Construction. At a point `p` of an edge, with outward normals `n_A` and `n_B` of its faces ordered so that
`n_A × n_B` runs along the walk, the body near `p` is `a, b ≤ 0` in the frame along `(n_A, n_B)` and the chamfer
removes `a + b ≥ −d`. The cutter's cross-section is the triangle `a ≤ o, b ≤ o, a + b ≥ −d` with `o = over/√2`: it
holds the removed wedge and reaches `o` past each face, so its other two sides lie outside the body. The section is
swept along the chain: a line takes its two end sections, which rule the wedge exactly; an arc takes sections at
chord points whose sagitta at radius `R + d + over` is at most `cutterSagitta` (0.002 mm), the cutter's bound. A
proven junction gives both edges the same section (Claim J's chamfer row), so the sweep is continuous and a closed
chain's cutter closes on itself. An open end carries its section straight on past the end vertex by
`reach + over`, `reach = d·max(0, n_G·n_A, n_G·n_B)/(n_G·t)`, so the end cap clears `G`'s plane over the whole
wedge. The corner is convex, so the body there lies inside all three planes and the cutter takes only the wedge.
The overshoot runs through `partialOvershoots` (0.5, 0.35, 0.7 of `d`) while the Cut refuses.

Gates, each `ErrUnsupported`: an open end on an arc, at a corner that is not convex or not of three faces, or on an
end face that is not a plane there or faces the chain; two chains that meet at a vertex, or whose cutters' boxes
meet; a held facet of any face other than the chain's faces, its end faces and their neighbours that meets or comes
within bounds of the cutter (`checkCutterClearOf`, the partial route's test); and the Cut's own refusals. Each cutter
is built and tested against the receiver before any Cut.

The result is a Boolean Cut: faceted, with the Cut's own readings.

Tests: `apitest/chamfer_tangent_chain_planes_test.go` chamfers the bar at two sizes from its caps' edges, from its
sides' arcs and from one seed under `WithTangentChain`; a step's floor rim open to the top face, and ending on an
earlier bevel of the top rim of the same setback (the column takes the old bevel's far corner) and of a wider one
(the wall's edge with the old bevel keeps its far part); a slot through the bar's rounded end, before and after the
bar's rim was chamfered; a slot, a tongue open to uncut caps and a tongue ending on the caps' earlier bevels through
a stadium link; tongues ending on the cap bevels of sketched stadiums of two lengths and of a sketched head with
filleted corners; and the link's tongue and fork faces in one call. Each checks the removed volume against Pappus
(with `d³/6` for each end on a 45° bevel that meets the wall square) within the readings' bounds, that no chain edge
survives, that every vertex the chamfer made lies `d` from a chain edge's line or circle, and that each line's bevel
is a new oblique plane and each arc's a new cone with no new faceted face. The ends on an earlier bevel also check
the column's two vertices. `internal/retainedblend/chain_cross_internal_test.go` checks the identity
(`columnAtCorner`): the same setback meets the old bevel's corner, and a wider bevel, a setback one float step off and
the other face's column do not; and the direction test for a chord that leaves a held corner. Red legs: without the
coincidence the ends on an earlier bevel of the same setback fall back to the cutter, whose Cut refuses them; with the
seam read from every edge near `g` the step open to the top face falls back to the cutter and its bevels are faceted;
with a row of its own on the cylinder an arc's bevel meets the cylinder's facets at a generator's cut and the strip
test refuses; without the line beyond a held corner on `X`, the sketched tongues fall back to the
cutter, whose Cut refuses them. No case reaches the line on `G`, which reads the same way.

### 16. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Chain junctions | exact tangency, or a gap of at most 64 ulps of the coordinate scale (§5.2) | a bounded near-tangency rule wider than round-off |
| `Shell` and `Draft` of a mesh Cut | refuse (§9.1, §9.2) | a source-backed route on an operand record |
| `Cone` and `Torus` rims | refuse (§9.5) | a coaxial revolution class read from denoted surfaces |
| R5 | built (M6) | drop it |

- **Chain junctions.** The default proof is short and matches reach §5. It admits a tangency gap of 64 ulps at
  most (the rounding of sketch-built tags) and refuses every part whose wall and fillet tags miss tangency by
  more, for example a box whose wall and fillet tags were rounded apart. The wider bounded rule admits those
  parts. It costs a new proof that the seam lies within `√(2R·δ)` of the foot and that the crease angle is at most
  `√(2δ/R)`, with `δ` the exact tangency defect, both charged into the junction rows.
- **`Shell` and `Draft`.** The mesh Cut would keep an immutable record of its operands `T` and `U`. Then
  `Shell(T − U) = (T − U) − ((T ⊖_F t) − (U ⊕ t))` holds for modify §8's erosion when `F` are faces of `T` and `U`
  crosses `T` by more than `t`, and `Draft(T − U) = Draft(T) − U` holds when every drafted face descends from `T`.
  The route gains shells and drafts of drilled parts. It costs a new record on every mesh Cut, a dilation of the
  tool, two more mesh Booleans per call (which may hit the coarse-held-contact refusal on curved faces), and a
  separate design.
- **`Cone` and `Torus`.** The coaxial class's meridian blend is a `Torus`, built from the denoted interval
  surfaces. It gains blends on revolved shafts after a placed cut. It costs an interval construction and a proof
  that two denoted walls share one axis. A revolved shaft's plane and cylinder faces blend after a placed cut where
  the other gates of the route hold (blend §2), and a revolved ball's plane-sphere rim blends by R6 (blend §4.5); the
  cone and torus classes wait.
- **R5.** Dropping R5 removes M6. Two parallel cylinders meeting in a transversal edge are rare in machined parts.

## Part O — Shell opening

How `Shell` removes one proper connected run of a prism's side faces
(Part R Table RX row RX1, Table BX row BX4 — "reach
§N" below), and how the same rim rule lifts the right-angle restriction on a
revolve side opening (reach §9.3.2). Companion to Part F
("modify §N"), which owns the call contract, the gate order, the section
offset and its §5 audit; to `agent-docs/prism-boolean-design.md` Part G ("general-boolean
§N"), which owns the `brepPayload` the prism result is recorded on; and to
Part B ("brep-modify §N"), which owns what a later
modify op does with that result.

A zero-twist one-line solid `Sweep` uses its stored exact prism for this
construction. Its selected `side(0,0,j)` face must name outer-loop segment
`j` before the same region and topology audits run.

Four tables are normative:

| Table | Owns | Section |
|---|---|---|
| **RO** | the rim at each end of the kept chain, per corner kind | §2 |
| **SO** | every refusal this document adds, with its sentinel and gate order | §5 |
| **BO** | result payloads, topology and roles | §6 |
| **DO** | what every downstream consumer reads | §7 |

### 1. Problem

Closing the wall section with the exact normal segment of length `t` at each
end of the kept chain (reach §9.2's construction on `stackedPrismPayload`) has
three defects, each reproducible on an ordinary part:

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

### 2. Table RO — the rim rule

#### 2.1 Statement

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

#### 2.2 Derivation

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

#### 2.3 Corner table

| Corner at `v` | `q` | Rim | Where the cut lands on `k'` |
|---|---|---|---|
| right (`θ = 90°` or `270°`) | the foot `v + s·t·n̂`, which lies on `carrier(r)` | perpendicular to `k`, length `t` | at the foot |
| acute (`θ < 90°`) | forward along `r`, at `t/sin θ > t` from `v` | along `r`'s span, longer than `t`; the wall ends in a wedge whose edge is `v` | short of the foot: `k'` is trimmed back by `t·cot θ` |
| obtuse (`90° < θ < 180°`) | forward along `r`, at `t/sin θ > t` from `v` | along `r`'s span, longer than `t` | past the foot: `k'` is extended by `t·|cot θ|` |
| reflex (`180° < θ < 360°`) | backward along `carrier(r)`, at `t/|sin θ|` from `v`, inside the material inward (outside it outward) | the extension of `r`'s carrier beyond `v`; its outward normal is opposite to `r`'s | short of the foot for `θ < 270°`, past it for `θ > 270°`; the exact extension of a straight `r` is one `LineSeg` from `q` through `v`, which the record states as two pieces split at `v` where `r` is oblique (§4.2) |
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

#### 2.4 The computation and its bound

`offset2d.OpeningJoin(k, r survey2d.SideWalk, atEnd bool, s, t, tol float64)
(Join, error)` (`internal/offset2d/opening.go`) is the one owner of the rim cut,
beside `CornerJoin` and `MirrorCornerJoin` (`internal/offset2d/section.go`): `k` is the kept
walk and `r` the removed neighbour; `atEnd` says whether `v` is `k`'s end
(`K`'s end, `r` follows) or its start (`K`'s start, `r` precedes). It returns
`Join{VertU, VertV: v, M: q}` with `Arc` and `G1` false. `offset2d.RimSegment`
writes the rim segment from that join and `r`'s record: a `LineSeg`, or an
`ArcSeg` about `r`'s centre turning in the entering sense from `v` to `q`, and
the other way when the caller's loop walks it from `q` to `v`. The revolve
(§8) records that segment; the prism side opening keeps its `LineSeg` and
states a rim on a circular `r` as a parameter range of `r`'s own record
instead (§4.2).

| Step | Rule |
|---|---|
| classify | `cross = k̂ × r̂`, `dot = k̂ · r̂` on the held unit tangents, with modify §8's dead zone: `|cross| ≤ tol` is SO1 (`ErrUnsupported`) whatever `dot` is — a smooth join and a cusp alike |
| entering direction | `r̂ · n̂ > 0` forward, else backward; `n̂` is `k`'s left unit normal at `v` times `s` |
| exact pairs | a straight `r` whose raw tangent at `v` has a float dot product of exactly zero with `k`'s: `q` is `k`'s offset foot `v + s·t·n̂` along `k`'s held unit normal, bit for bit what the right-angle open chain wrote before this rule. Two axis-aligned lines (`k` with `du = 0` or `dv = 0`, `r` on the other axis) are such a pair, and there the foot is the pair of levels — `r`'s constant coordinate, which `v` carries, and `k`'s offset level `v_coord + s·t` — read as `stackedbrep`'s plane–plane junction reads it, so the record and the engine hold one point. An axis-aligned line `r` through the centre of a circular `k`: `q` is the centre moved by `k'`'s radius along that axis |
| float solve | otherwise `Intersect(offsetCarrier(k, s, t), offsetCarrier(r, s, 0))` over all roots, choosing the first root reached from `v` in the entering direction: along a line by signed parameter; around a circle by the signed sweep from `v` in the entering sense. No root, or the first root reached lying on `carrier(k)` (the circle re-crosses `k` before `k'`), is SO1. A straight `r` along a section axis holds `v`'s coordinate across that axis bit for bit, since `q` lies on `r`'s carrier: the record states the cut on `r`'s own plane, and the reach charges the solved coordinate |
| span | the forward cut must lie strictly inside `r`'s span: a line by parameter in `(0, 1)`, an arc by sweep strictly below `r`'s own; otherwise SO2. A backward cut lies off `r`'s span by construction and has no span test; the audits of §4.7 decide whether it crosses anything |
| consumption | `k'` trimmed at `q` must still advance along `k` (`offset2d.OpenWalkConsumed` with `q` as the trimmed end, modify §7's S11a reading); a consumed `k'` is S11a. A cut past the foot extends `k'` beyond `k`'s own end: a line by any length, an arc by the angle from `k`'s end angle to `q` about its centre, read in the walk's sense and wrapped to `(−π, π]`. A reading strictly between `0` and `π/2` is an extension, since a straight rim from `v` to `q` inside the band subtends less than `π/2`. Such an arc is consumed only when its held sweep exceeds its source's sweep plus the extensions, or when that allowance reaches a full turn. Every other end reads as `WalkConsumed` reads it. The extended arc ends at the held `q`, which `OpeningReach` charges |

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

### 3. The three regions

`offset2d.OffsetOpenChain` (`internal/offset2d/open_chain.go`) offsets an
open chain with per-end joins. An end is one of two kinds: an **axis end** (`MirrorCornerJoin`, the revolve's), and an **opening end**
(`OpeningJoin` with the removed neighbour walk). It returns `K'` in `K`'s own
walk order from `qA` to `qB`, the join at each interior corner as modify §7
rules (miter, arc of radius `t` about the corner, G1 foot), and the two cut
points. Every walk of `K'` passes S11a's consumption test in
`OffsetOpenChain`: `offset2d.OpenWalkConsumed`, which lets the walk at an
opening end run past its own end to the cut (§2.4).

With `R'` the removed run re-cut at both ends — `r_B` (the walk after `vB`)
starting at `qB` instead of `vB`, `r_A` (the walk before `vA`) ending at `qA`
instead of `vA`, each a `LineSeg` on its own carrier or a parameter range of a
circular walk's own record (§4.2), and every other walk of `R` verbatim — the
three regions are:

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
and its own float sum's rounding, as `prismshell.BuildClosed` derives its levels.

### 4. The brep construction

#### 4.1 Slabs

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

#### 4.2 What the engine writes

`stackedbrep.Engine` takes the slabs' loops and levels and writes the faces
general-boolean §3 "A1 as a brep" states, with no scene: every loop here is
decad's own record, so `LoopOf` restates it directly, and the faces are:

- **horizontal faces**: the two caps and the exposed `C` at each interface,
  planar faces in the reference frame, each line unit split at the body's
  vertices at that level and each circle unit at every vertex on it;
- **axis-aligned straight walls**: one planar face per carrier plane and
  material side (`Engine.wallFaces`), the rectangles each slab's segment sweeps with
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

The circular case is stated in the same pieces as the oblique one below
(`P`'s arc split at each forward cut, `R'` at `v` where its cut runs
backward), and the engine merges consecutive pieces of one circle walked one
way into one unit, `circlePoints` splits the cap slabs' unit at `qA` and
`qB`, and `Engine.sweptFaces` joins the column pieces; the pieces serve
the area identity (§4.7), and each end vertex on a circular `r` is marked as
an oblique one is. The engine keys a circle by the centre and the radius an
`ArcSeg` reads from its `Start`, so every piece on `carrier(r)` — the rims and
the pieces of `R` and `R'` — is a **parameter range of `r`'s own record**:
the recorded segment's `Center`, `Start` and `End` (a `CircleSeg`'s `Center`
and `Radius`) verbatim, and `[tStart, tEnd]` in that segment's own
parameterisation, walked backward (`tStart > tEnd`) where the region walks
against `r`. Every piece then reads `r`'s radius bit for bit, whether its cut
is an exact point of the circle or a float solve such as the D section's
`(2, ±√21)`. A cut's parameter is `atan2` of `q` about the centre over the
segment's sweep, as the segment's own walk evaluates angles; a walk end `v`
takes the segment's own range end. A piece spanning several segments of a
coalesced walk is one range per segment it crosses, verbatim where it
crosses one whole. A backward cut lies off `r`'s span: its piece between `q`
and `v` is a range of the segment recorded at `v` where `q`'s parameter lies
in `[0, 1]` beyond `v`'s, and otherwise, where `v` is an `ArcSeg`'s own `Start`
or `End`, a range of that arc's complement — the same `Center` with `Start` and
`End` swapped, which sweeps every angle the arc does not and reads its radius
from the arc's `End`. The complement keys and denotes `r`'s circle only where
`End` lies exactly on the circle `Start` defines, compared over exact
rationals. A forward cut whose parameter does not land strictly inside a
segment's range, and a backward cut neither form names, is SO5, refused
before the engine runs. The point a range names at a cut's parameter is not
`q` itself; the engine never reads it (every junction on `carrier(r)` takes
the straight carrier's walked point, the cut `K'` states), and BO1's prism,
which publishes the regions as its section, charges it (§4.4).

The oblique case needs every region walking `carrier(r)` to state it in the
same pieces, since an oblique line carrier is keyed by its recorded endpoints and the engine refuses a vertex strictly inside one: `P`'s
record of `r` is pre-split at every forward cut (`vB → qB`, `qB → qA`,
`qA → vA` as three collinear `LineSeg`s), and `R'` is split at `v` where its
cut runs backward (`qB → vB`, then on along `r`). `LoopOf` keeps collinear
units apart when their carriers differ, and the piece keys then match across
slabs. The area identity (§4.7) reads `P` as recorded. Each oblique end
vertex `v` is also marked at both cavity levels (§4.3): the column `v → q`
is one swept face split at `q`'s two levels, and the mark splits the kept
wall's edge through `v` at the same levels, so the mesh of the two faces
closes piece for piece. An oblique end walk of `R` recorded as more than one
segment has no such common statement and is SO5.

#### 4.3 Engine extensions

| Extension | Where | What |
|---|---|---|
| planar wall with holes | `brepgeom.StackedWallRegions(embed, loops)`, read over every loop of one wall key | the chained loops of one `(axis, level, side)` key are grouped: all counter-clockwise → one face each; exactly one counter-clockwise and the rest clockwise → one face `{Outer, Holes}`; anything else is a miss. The §5 audit's S9 then proves each hole nested in its outer, and refuses; it admits nothing |
| vertex events | `Engine.Event(p Point2, level int)` exported | the shell marks each end vertex `v` whose cut runs backward along `carrier(r)` (a reflex end inward, a convex end outward), and each end vertex on an oblique `r`, at both cavity levels, so `C`'s walk along `carrier(r)` is split at `v` where the rim face and the floor strip meet, and the kept wall's edge through `v` splits where an oblique rim column's does (`CutsOnLine` reads events; a vertex whose junction pair does not change between slabs gets none on its own) |
| tangent line–circle junction | `Engine.junction`, `tangentFoot`, `recordedJunction` | a line crossing a circle is keyed by the side of the centre's foot its walked end lies on, read along the line, an oblique line as an axis-aligned one. An axis-aligned line whose level lies exactly the circle's radius from the centre's coordinate across it, read in exact rational arithmetic over the held floats, touches the circle at one point: the centre's coordinate along the line at the line's level, an exact point. Where the side test cannot decide a line–circle crossing (the walked end within its allowance band of the centre's foot), that point is the junction, keyed with no side and charged the walked end's allowance plus its distance from the point. Every arc join of `K'` (modify §7's arc of radius `t` about an outward convex or inward reflex interior corner) between axis-aligned walks meets its offset lines so, since the join's feet are the offset lines' own feet. Any other crossing the side test cannot decide — an oblique offset line meeting an arc join at its foot, or an axis-aligned one meeting an arc join that reads its radius off a float foot — stands where both walked ends are one recorded point, as two circles do (next row), and misses where they differ |
| circle–circle junction | `Engine.junction`, `recordedJunction` | two circles meeting where both walked ends are one recorded point take that point with the larger allowance, keyed by the side of the line through the centres it lies on, read in exact rational arithmetic; a later loop reaching that key at another point misses, so two crossings never share a vertex. Two circles whose walked ends differ still miss |

A miss anywhere in the engine (`brepgeom.ErrStackedWallMiss`) is SO5, never a
silent fallback: a shell has no mesh path to fall to. The junctions at one
recorded point decide nothing by a float sign: the record states each join of
`K'` — a miter, an arc join's foot, a rim cut — once, as the end of both
segments meeting there, and the engine takes it as recorded. How far that
point sits from the join it denotes is the chain reach's (§4.4): `cornerReach`
encloses an arc join's two feet and a miter's root, `OpeningReach` a rim cut.

#### 4.4 Displacements

Every face carries one section displacement `delta`, the larger of the chain
reach's figure (`3 × ChainReach`, modify §9's three-reach argument over the
open chain and both cuts) and the engine's largest canonical-vertex
allowance (`Engine.Allow()`), as `internal/stackedunion/brep.go` charges its faces.
BO1's prism carries the chain's figure plus three times the **cut gap**: the
largest distance between a rim's cut `q` and the point the rim's range names
at `q`'s parameter (§4.2), that point enclosed over rational intervals
(`boundarywalk.CircularWalkEndBound`). The range's end is within the reach
plus the gap of the cut the shell denotes, on `r`'s own circle, so modify
§9's three-reach argument covers it at three times their sum. The gap is
zero where every rim is a line; a BO2 stack never reads it, since the engine
writes every face from its canonical vertices.
Levels carry their own `z0Delta`/`z1Delta` (§3). A rectilinear section whose
offset levels and cuts are exact floats and whose thickness converts exactly
has `delta = 0`, and its volume is `Exact` (general-boolean §4.3). `stack`
is nil: the result is not an A1 operand (brep-modify DB11).

#### 4.5 Topology and roles

The record is general-boolean §4.2's: every edge bounds exactly two faces by
`breprecord.TopologyContext`'s count, and a record that fails it is `ErrUnsupported`
(SO5) — the proof that the three regions closed against each other. One lump,
one shell, `FaceShell.IsVoid()` false: the cavity reaches the outside through the
opening. Roles are the brep's, `face(k)` and `wall(k)` by result index,
minted fresh under the shell's producer identity; no `capStart`/`capEnd`, no
`rim`, no receiver role (modify §11, general-boolean §4.2). A caller names a
result face by geometry. Edge convexity reads general-boolean §4.2's walked
boundary: a coplanar junction between two swept pieces of one oblique plane
reads concave, as every zero turn does.

#### 4.6 What `sketch` is asked

Nothing. `K'`, the rims and `C` exist in no sketch: they are decad's own
geometry synthesised from decad's own record, the class modify §5 names for
the offset and the corner blend, and decad proves them with the same
closed-form audit (§4.7) and the record's closure by counting. No upstream
claim exists to bless, no residual admits anything, and no scene is built: a
scene of `P` and `W` would place `W`'s cut vertices on the interior of `P`'s
walks, which `sketch` returns as `Partial` fragments whose `TExact` decad can
only reject (`agent-docs/sketch-seam-design.md`).

#### 4.7 Audit

In order, each refusing and none admitting:

1. S11a as `K'` is built (a consumed walk, a dropped circle); SO1–SO4 as each
   cut is computed.
2. Modify §5's audit on `W` and on `C` (`O` outward): S8 (orientation; a `C`
   with no area is SO3, the cavity does not exist), the crossing test
   (S11b), S9. Together the two loops hold every pair of a new segment with
   another: rims and `K'` against `K` in `W`, `K'` and the re-cut `R'`
   against each other in `C`; the pairs of `K` with `R` are the receiver's
   own.
3. Area identity, exact rational over the walked floats:
   `area(P) = area(W) + area(C)` inward, `area(O) = area(W) + area(P)`
   outward, `P` read as recorded (§4.2); a sum that differs is SO5. An arc
   enters as its chord. Every arc stands on both sides piece by piece — a
   kept arc in `P` and `W`, an arc of `K'` in `W` and the offset region, a
   rim in `W` and in `P` or `R'`, every other piece of a removed arc in `P`
   and `R'` — so its bulge cancels. A piece on a removed arc is the same
   parameter range in every region that holds it (§4.2), so its walked ends
   are the same floats there, read forward or backward, and its chord terms
   cancel exactly although a cut's walked point is not `q`. The identity
   pairs the pieces; whether a cut lies on `carrier(r)` is the enclosure's
   question (§2.4), not this one's.
4. The engine's own misses (SO5), modify §5's audit on every planar face the
   engine writes (as `internal/stackedunion/brep.go` runs it), `breprecord.Falsify`,
   `breprecord.TopologyContext`'s closure count, and `breprecord.Build`'s positive
   volume (`ErrDegenerate`).

### 5. Table SO — refusals and gate order

Modify §1's test picks every sentinel: a body that does not exist is
`ErrDegenerate`; one this evaluator does not build is `ErrUnsupported`.

| SO | Call | Exists? | Sentinel |
|---|---|---|---|
| **SO1** | an end of `K` at a smooth or cusped corner (`|cross| ≤ tol`), or a cut whose carriers meet nowhere in the entering direction (a line `r` parallel to a line `k`, an arc `r` turning away from the band or re-crossing `carrier(k)` first) | the body has a zero-thickness wall edge at `v`, or no inner body closes at `v` without a surface this record does not hold | `ErrUnsupported` |
| **SO2** | a forward cut beyond the far end of `r` (`|r| ≤ t/sin θ` for lines; the sweep to `q` reaching `r`'s own) | yes; the inner body's boundary there runs on `r`'s neighbour's carrier, a trimmed-offset construction this evaluator does not build | `ErrUnsupported` |
| **SO3** | `C` encloses no area (S8 on `C`), or inward `h − k·t ≤ 0` for `k` kept caps (reach SX11) | no cavity | `ErrDegenerate` |
| **SO4** | a cut, a join or a level whose displacement the enclosure cannot bound (`offset2d.ErrUnbounded`) | yes; its readings would carry no bound | `ErrUnsupported` |
| **SO5** | the engine misses the record (`ErrStackedWallMiss`: two circles, or a line and a circle crossing too near the centre's foot to key, meeting at distinct walked ends or at a point another loop keyed apart (§4.3), a wall plane whose loops are neither all outers nor one outer with holes), an oblique end walk of `R` recorded as several collinear segments (§4.2), a rim cut on a removed arc that no range of the arc's record names — a forward cut whose parameter rounds onto a segment's range end, a backward cut behind an arc whose `End` lies off the circle its `Start` defines (§4.2) — the area identity fails, or the closure count fails | yes; the record cannot be stated | `ErrUnsupported` |
| **SO6** | a side opening on a holed section; a run that is not one proper connected run of whole walks of the outer loop (a face of a hole loop, a run covering part of a coalesced walk, every side face) | yes | `ErrUnsupported` (reach SX8's text) |
| **SO7** | a side opening on a brep or stacked receiver through route P | yes; route P maps removed faces to the recognised prism's caps only | brep-modify SB3, unchanged |

Gate order, after modify §4's stage 1 and reach SX10:

| Stage | Gates |
|---|---|
| 2. receiver | S3; `prismrecord.Payload.RequireExactSection`; the removed faces: caps by role, prism sides by `side(0,j)`, one-line Sweep sides by `side(0,0,j)` mapped to the stored prism segment; SO6 |
| 3. levels | SO3's height half (inward, kept caps) |
| 4. chain | `offset2d.OffsetOpenChain` over `K`: S11a per walk, SO1/SO2/SO4 per end; each cut on a removed arc placed in the arc's parameterisation (SO5) |
| 5. regions | §4.7 steps 2–3: S8 (SO3's section half), S11b, S9, the area identity |
| 6. record | both caps removed → `prismrecord.Build` over `W`; else the engine, §4.7 step 4 |

No section limit (modify S10, S18) runs: the cavity opens through the removed
faces, so the inradius of `P` bounds nothing. A cavity that closes by
consuming walks reports S11a, as the revolve side opening does today; SO3's
section half fires only on a chain that survives and encloses nothing. A kept
arc whose offset runs past its own end to the cut, Table RO's extension of
`k'` on a circular `k`, builds (§2.4's consumption row). Where that cut is a
float solve on a removed arc's circle and a cap is kept, the rim's range and
the extended arc end at two walked points, and the engine misses (SO5).

### 6. Table BO — results and roles

| BO | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BO1** | side run, both caps removed, either sense | `prismPayload` over `W` on `[z0, z1]` | modify B1's | `side(0,j)`, `capStart`/`capEnd` |
| **BO2** | side run, one or both caps kept, either sense | `brepPayload`, `stack` nil | one lump, one shell; the removed face's carrier as §4.2 states | `face(k)`/`wall(k)` |
| **BO3** | revolve side opening with a slanted or circular rim (§8) | `revolvePayload` over the wall region | reach BX6/BX7's | reach BX6/BX7's |

Reach Table BX row BX4 reads BO1/BO2; reach §9.3.2's rim restriction reads
BO3.

### 7. Table DO — downstream

A BO1 result is a prism and modify Table D covers it. A BO3 result is a
revolve and reach Table DX's revolve column covers it; where its wall carries
a nonzero `sectionDelta` (§8), every reading charges it as reach §9.3.1 and
`agent-docs/surface-intersection-design.md` §7.2 state, the clearance, wall,
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
| **DO6** | clearance | `clearance.BuildBrepBodyCarriers`; no model when `delta > 0` |
| **DO7** | undercut | per face: a planar face's one outward normal, a swept face's walk range |
| **DO8** | concave radius | the cavity's concave arcs are clockwise arcs with the material on their left and are read; undecided when `delta > 0` |
| **DO9** | wall survey | staged `Suspect` (`DiagUnsupportedSurveyPayload`), as for every brep |
| **DO10** | STEP | analytic where every wall is a plane or a cylinder (the oblique four-face plane writes as four planar faces) |
| **DO11** | `Fillet`/`Chamfer`/`Shell` on the result | brep-modify Tables RB/EB/SB over the record: RB1 needs `delta = 0`, so a rectilinear exact result takes route E on any axis-parallel straight edge (the opening's edges included: each hole corner is incident to three faces); route P applies only where the result reads as a prism along a reference axis, which a U-channel with kept caps does not; `Shell` again is SB10. A result with `delta > 0` is SB1 |
| **DO12** | `Transform`/`Mirrored`/`PatternCopies` | re-lifts every face frame; a reflection flips `outward` and every wall's winding |

### 8. The revolve's slanted rim

A revolve side opening (reach §9.3.2) closes each opening end by Table RO:
`OffsetOpenChain`'s opening end calls `OpeningJoin`, and the wall
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
  `revolveshell.ChainSectionDelta` encloses every interior miter, every axis join and
  each opening's rim cut (`OpeningReach`, §2.4) and publishes three times the
  largest reach as the revolve's `sectionDelta`, with `sectionWhole` set,
  only where it is nonzero. A cut whose enclosure is the held float — a
  right-angle rim on axis-aligned walks, the cone fixture's `(6.5, 4)` —
  adds nothing, so those bodies stay bit for bit undisplaced. The prism route
  charges the same reach per face (§4.4).

SO1 and SO2 are the revolve's opening-end refusals; an opening end adjacent to
the axis walk never arises (the run touches the axis walk, reach §9.3.2).

### 9. Required tests

Every test asserts computed geometry against a closed form; bounds are
relations (`|value − closed form| ≤ Bound`), never literals; every bound
fixture is shown to fail first by deleting the leg it guards. Tests of the
exported API live in `apitest/shell_opening_test.go` and
`apitest/revolve_shell_side_test.go`; record-level tests in
`internal/offset2d/opening_test.go` and `internal/stackedbrep/record_test.go`.

**U-channel**. Box `x∈[0,40]`, `y∈[0,20]`, `z∈[0,10]`, `t = 2`, the
`x = 0` face removed:

- inward, both caps kept: cavity `(0,38)×(2,18)×(2,8)`, volume
  `8000 − 3648 = 4352`, area `2704 + 1768 = 4472`, centroid
  `(1417/68, 10, 5)`, 11 faces, the `x = 0` face one planar face with one
  hole `y∈[2,18]`, `z∈[2,8]`, `Exact`, `delta = 0`, `FaceShell.IsVoid()` false,
  one lump;
- the `z = 10` cap removed too: cavity height 8, volume `3136`;
- both caps removed: a `prismPayload` over `W`, volume `192·10 = 1920`,
  `capStart`/`capEnd` present;
- outward, both caps kept: `K'` rounds the two convex corners beyond
  `x = 40` to arcs of radius 2 tangent to their neighbours (§4.3's tangent
  junction): `W` of area `208 − 2(4 − π)` over `[0, 10]` and `O` of area
  `1008 − 2(4 − π)` over the two cap slabs, volume `6000 + 28π` within its
  bound against π's bracket, 13 faces (two corner cylinders, the `x = 0`
  plane one face whose hole is the removed face); with both caps removed the
  same wall is BO1's prism, volume `(208 − 2(4 − π))·10`;
- outward, both caps kept, the `x = 0`, `y = 20` and `x = 40` faces removed:
  the one kept wall `y = 0` cuts backward at `(0, −2)` and `(40, −2)`,
  `O = [0,40]×[−2,20]`, volume `880·14 − 8000 = 4320`, 10 faces, `Exact`;
- `t = 10` inward → SO3's height half; `t = 10` with both caps removed →
  S11a (the far wall's offset is consumed); `t = 5` with both caps removed
  builds, the far wall keeping 10 of its 20: volume `(800 − 350)·10 = 4500`;
  removing the `x = 0` and `x = 40` faces → SO6 (two runs); every side face →
  SO6; `Verify` `Sound`, STEP analytic, the mesh's occupied-volume proof
  covering `4352`.

**L prism opened on one leg**. `P = (0,0),(30,0),(30,10),(10,10),
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

**Acute corner**. Triangle `(0,0),(12,0),(0,9)`, height 10,
`t = 1.5`, the `y = 0` leg removed, inward, both caps kept: the acute end at
`(12,0)` cuts at `q = (9.5, 0)`, the right-angle end at `(0,0)` at
`(1.5, 0)`; `C = (1.5,0),(9.5,0),(1.5,6)`, area 24; volume
`540 − 24·7 = 372`; the `y = 0` face a planar face with hole `x∈[1.5,9.5]`,
`z∈[1.5,8.5]`; the hypotenuse wall and its offset are swept faces; `delta`
encloses the offset's distance from the exact line `3x + 4y = 28.5`.

**Obtuse corner**. Trapezoid `(0,0),(14,0),(11,4),(3,4)`, height 10,
`t = 1`, the `y = 4` side removed: cuts at `(4.25, 4)` and `(9.75, 4)`;
`C = (2,1),(12,1),(9.75,4),(4.25,4)`, area `23.25`; volume
`440 − 23.25·8 = 254`.

**Oblique removed face**. The triangle above with the hypotenuse
removed, `t = 3`: cuts `(8, 3)` and `(3, 6.75)`; `C = (3,3),(8,3),(3,6.75)`,
area `9.375`; volume `540 − 9.375·4 = 502.5`; the hypotenuse plane holds four
swept faces whose shared edges pair; the cap loops hold the hypotenuse as
three collinear segments.

**Slanted reflex corner**. `P = (0,0),(30,0),(30,10),(14,10),(10,30),
(0,30)`, height 10, `t = 2`, the oblique face `(14,10)→(10,30)` removed: the
reflex end at `(14,10)` cuts backward at `(14.4, 8)`, the convex end at
`(10,30)` forward at `(10.4, 28)`; `C = (10.4,28),(2,28),(2,2),(28,2),(28,8),
(14.4,8),(14,10)`, area 364, `R'` split at `(14,10)`; volume
`5400 − 364·6 = 3216`; the removed face's carrier holds three swept faces
facing out and the reflex rim facing back. Outward, the triangle's hypotenuse
kept alone at `t = 1` cuts backward at `(41/3, 0)` and `(0, 41/4)`: volume
`2·1681/24 + 10·(1681/24 − 54)`. With both caps removed the acute fixture is
BO1's prism, volume `(54 − 24)·10`. A forward cut past an oblique removed
face's far end is SO2, and an oblique removed face recorded as two collinear
segments is SO5.

**Arcs**. The D section — the semicircle of radius 5 about the origin
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
- arc removed outward, `t = 3`: the offset `x = −3` meets the circle at
  `(−3, ±4)` walking it backward from each corner, `O` is the disc less the
  segment left of `x = −3`, volume `192 + 275π − 400·acos(3/5)`, 10 faces;
- arc removed, `t = 2`: the cuts `(2, ±√21)` are float solves; the section
  displacement is nonzero and encloses `√21`; the rims and the pieces of `R`
  and `R'` are ranges of the arc's own record, each rim's range end within
  the displacement plus the cut gap of the exact cut; with both caps removed
  the prism's volume `125π − 250·acos(2/5) + 20√21` lies within its bound,
  with the end cap kept `125π − 200·acos(2/5) + 16√21` (8 faces), with both
  kept `125π − 150·acos(2/5) + 12√21` (10 faces), each `Verify` `Sound` with
  a watertight, volume-verified mesh; outward at `t = 2` the cuts
  `(−2, ±√21)` lie behind both corners, the rims are ranges of the arc's
  complement, and the volume is `225π − 350·acos(2/5) + 28√21`;
- arc removed, `t = 3`, both caps removed: the prism's volume
  `120 + 125π − 250·acos(3/5)` within a bound that charges the cut gap of the
  exact cuts `(3, ±4)`, whose parameters are floats;
- the removed arc touching the kept chord at its end (a cusp) → SO1.

**Circle junctions**. Each 10 tall:

- the notch — the concave arc of radius 13 about the origin from `(0,13)` to
  `(13,0)` with the material outside it, the arc of radius 17 about `(−2,8)`
  from `(13,0)` to `(13,16)`, then `x = 13` to `(13,20)`, `y = 20` and
  `x = 0` — with every walk but the concave arc removed, inward, `t = 4`,
  both caps kept: the arc–arc end corner `(13,0)` cuts at `(15,8)`, the other
  end at `(0,17)`; with `β = atan(8/15)`, `P` has area `140 − 169π/4 + 289β`
  and `C` `156 − 72.25π + 289β`, volume `1088 − 278π + 2312β`, 14 faces;
  with both caps removed `W = 30π − 16`, volume `300π − 160`. At `t = 2` the
  cut `y = 4(1 + √239)/17` is a float solve whose displacement the section
  delta encloses; with a cap kept the rim, a range of the removed arc's
  record, and the offset arc end at two walked points, so the engine misses
  (SO5);
- the half lens — the arcs of radius 5 about `(0,0)` and `(6,0)` above the
  chord `y = 0` — without its chord, inward, `t = 1.25`: the interior arc–arc
  corner `(3,4)` miters at `(3, 2.25)`, the cuts `(3.75, 0)` and
  `(2.25, 0)`; with `A = acos(3/5)`, volume `355.46875A − 52.734375π −
  69.375` with both caps kept and `390.625A − 70.3125π − 52.5` with both
  removed;
- the triangle `(0,0),(12,0),(0,9)` without its `y = 0` leg, outward, `t = 1`,
  both caps kept: the corner `(0,9)` rounds to an arc of radius 1 meeting the
  slanted offset `3x + 4y = 41` at `(0.6, 9.8)` and `x = −1` at `(−1, 9)`, the
  arc reading its radius off the float foot; volume `404 + 6π − 6A`, 10
  faces;
- engine records: the lens's two crossings `(3, ±4)` two vertices; two arcs
  whose walked ends differ by an ulp miss; a second loop meeting the lens's
  circles at `(3, 4 + ulp)` misses; an oblique line touching a circle at its
  recorded point builds; an oblique chord's two crossings of its circle are
  two vertices.

**Arc extension**. Each 10 tall, outward, every walk but the concave
arc of radius 13 about the origin removed:

- the end section — that arc from `(0,13)` to `(5,12)`, then `x = 5` to
  `(5,20)`, `y = 20` and `x = 0` — at `t = 2.375`: the offset arc of radius
  `10.625` runs from the exact cut `(0, 10.625)` past the arc's end angle to
  the exact cut `(5, 9.375)`; with `β = atan(8/15)` and `A = atan(5/12)`,
  `P = 70 − 84.5A`, `O = 76.5625 − 56.4453125β`, volume
  `27475/64 − (426275/512)β + 845A` with both caps kept (10 faces) and
  `525/8 − (36125/64)β + 845A` with both removed, the displacement zero;
  revolved a full turn about the `x` axis, the same wall is a spherical band,
  volume `2π · 5(169 − 10.625²)/2`;
- the notch of the circle-junction fixtures at `t = 4`: the offset arc of
  radius 9 runs past `(13,0)` to the float cut `((35 + 8√38)/17,
  (−140 + 2√38)/17)` on the removed arc's complement; with both caps removed
  the prism's volume `220π − 520 + 20√38 + 405φ − 1445(β + ψ)`, `φ` and `ψ` the
  cut's angles about the two centres, lies within its bound, and the
  displacement encloses the cut; with a cap kept the engine misses (SO5);
- the end section with `x = 0` replaced by the slant `(−7,18) → (0,13)`: the
  slant's cut trims the offset arc to `59.8°`, past its extended end at
  `61.9°`, so the arc runs backward (S11a);
- every closed-shell, cup, tube, revolve and side-opening reading of the
  existing fixtures bit for bit unchanged.

**Revolve slanted rim**. Meridian `(ρ,z) = (0,0),(8,0),(8,2),(5,6),
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
and `.github/test-shards-apitest.txt` carrying every test name (`apitest/ci_shards_test.go`).

### 10. Do not do this

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
- **Fall back silently on an engine miss.** `internal/stackedunion/brep.go` misses
  into the mesh path because a boolean has one; a shell has none and refuses
  with SO5.
- **Charge the cut only on the rim face.** The payload-wide `delta` is what
  every consumer reads; a cut vertex belongs to the rim, the cavity wall and
  the cavity floor alike.
- **Admit a cut because the residual against `carrier(r)` is small.** The cut
  is decad's own closed form; its displacement is enclosed, never measured.

### 11. Decided questions

- **Payload for the prism side opening**: `brepPayload` through
  `internal/stackedbrep`; `prismPayload` where both caps are
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
- **Arc joins on oblique walks under a kept cap**: decided at one recorded
  point, the foot both segments end at, with its displacement the chain
  reach's (§4.3); no tangency test runs on the oblique line.
- **Hand-offs**: none. No capability is needed from `sketch`, `r3` or `units`.

### 12. PR split

All six increments of the side-opening design have shipped; Table SO lists
every remaining refusal. Where each piece lives:

| Piece | Where |
|---|---|
| `OpeningJoin` (§2.4) with its exact pairs and root choice, `OpeningReach`, `RimSegment`, `ChainReach`, `OffsetOpenChain`, `OpenWalkConsumed` (§2.4, §3) | `internal/offset2d/opening.go`, `reach.go`, `open_chain.go`, `section.go` |
| the revolve's slanted rim (§8) | `internal/revolveshell/wall.go`, `bounds.go` |
| the prism side opening: removed-face classification, the three regions, the §4.7 audit, BO1's prism | `internal/brepmodify/shell.go`, `internal/prismshell/section.go`, `side_opening_prism.go` |
| rim pieces on removed arcs, the cut gap and the area identity (§4.2, §4.7) | `internal/offset2d/opening_record.go` |
| the engine extensions (§4.3): `Engine.Event`, the tangent line–circle and circle–circle junctions | `internal/stackedbrep/record.go` |
| planar walls with holes | `internal/brepgeom/stacked_wall.go` (`StackedWallRegions`) |
| tests | `apitest/shell_opening_test.go`, `apitest/revolve_shell_side_test.go`, `internal/offset2d/opening_test.go`, `internal/stackedbrep/record_test.go` |

## Part S — Shell variants

One part per Shell route that has its own construction. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N". The general opening route is Part O.

### Faceted boss

This route shells the complete top cap of one ordinary two-prism mesh Union.
It covers a rectangular plate and a strictly nested rectangular boss whose
horizontal placement rotates and may carry the stored similarity scale. It
rebuilds a faceted cup from the Union's authenticated sources and held mesh.

#### Admission

The Union must retain its two original `prismPayload` sources and remain
unplaced. The plate is an exact positive XY rectangular prism with identity
placement. The boss is another exact positive XY rectangular prism. Its
placement has horizontal basis `(c,s),(-s,c)` and unchanged positive Z;
`sqrt(c²+s²)` is enclosed within `[0.99,1.01]`. No source section or level
displacement is admitted. The selected face must be exactly the boss's
`CapEnd`: one group of two triangles, four source corners, and no other
fragment of that cap. All four true boss corners, widened by their source
point bounds, must lie strictly inside the plate's inset by thickness `t`.

The boss bottom is exactly `t` below the plate top. The plate floor below it
and the boss's two horizontal dimensions must leave positive material after
the offset. All levels and offset coordinates used in the reconstruction
must be represented exactly where the route does not carry a numeric bound.
Other Union shapes and placed Union results keep the ordinary Shell refusal.

#### Cavity and held boundary

Write `L=sqrt(c²+s²)`, and let `W,H` be the boss's local rectangle widths.
The true boss width and height are `LW,LH`. A world-distance offset `q`
therefore becomes a local inset `q/L`. The cavity has a plate-inset floor at
`z_plate_bottom+t`, lower walls up to `z_boss_bottom`, a ledge from the plate
inset to the boss footprint, the rounded root band up to the plate top,
upper boss-inset walls, and a rim around the removed top cap.

The root edge, the boss outline at the plate top, is a concave edge of the
source, so the reflex-corner row of Part F §8 applies
to it: the cavity keeps distance `t` from the root edge. On the root
interval, with `s=z-z_boss_bottom`, the true world inset is

```text
q(s) = sqrt(t² - (t-s)²),       0 ≤ s ≤ t.
```

In the `(q,s)` plane this is the quarter circle of radius `t` centred on the
root edge at `(0,t)`. It leaves the ledge's boss-outline ring at `(0,0)`
tangent to the ledge and meets the upper inset walls at `(t,t)` tangent to
them. Each section is the boss rectangle inset by `q(s)` with sharp corners,
so adjacent sides meet on mitred diagonal seams.

The held band uses 16 stations at `w=k/16` with rational coordinates
`q=2tw/(1+w²)` and `s=2tw²/(1+w²)`. The square-root bracket for `L` and the
rational-to-float station error enter each new vertex's source point bound.
The planar curve `(q(w),s(w))` has second-derivative norm at most `4t`.
Linear interpolation at the same `w` across one station cell differs by at
most `t/(2·16²)`. Each rectangle corner moves in two inward coordinates,
so the payload charges `3t/(4·16²)` for the curved band. Parameter matching
allows Z to move near the horizontal tangent at `w=0`. The root volume
bound uses swept surface area rather than a same-Z inset difference. The
source Union mesh loses only the selected top cap triangles. Its other facets
and vertex bounds remain unchanged.

The held mesh is checked for positive facet area, exact facet contact,
directed-edge closure, components, and signed volume before the new body is
built. Strict source containment and the monotone nested rectangle sections
keep the new cavity apart from the exterior. A failed audit leaves the Union
live.

#### Occupied volume and readings

The true outer volume is plate area times plate height plus `L²WH` times the
boss height above the plate. The lower and upper cavity prisms have ordinary
rectangle volumes. Integrating the root rectangle area gives

```text
V_root = WHL²t - (W+H)Lπt²/2 + 8t³/3.
```

The code evaluates this expression as a rational interval with an outward
interval for `L` and `π`. Its held signed volume must lie inside the stated
occupied-volume allowance. The allowance adds the source Union's symmetric
difference bound, a root-surface sweep term below four times perimeter times
thickness times the chord and station gap, an upper-wall station term, and a
level-slab term. These
terms bound the actual symmetric difference; the final volume comparison is
an independent consistency gate. The faceted result carries the maximum
source and cavity boundary displacement. Its area slack includes both sides
of a conservative root-patch area bound.

### Twisted sweep

`Shell` admits an inward, one-cap opening on a one-line twisted solid whose
Sketch section is a smooth, centrally symmetric, strictly convex sequence of
whole Lines and Arcs. The path starts at the origin, follows positive Z, and
uses the origin XY profile plane. The opening must select `capEnd`. Thickness
must have an exact millimetre value, leave a positive floor, and pass the
bounded offset and mesh proofs below. The result has one outer wall, one inner
wall, a bottom cap, a cavity floor, and a top rim. Other twisted Sweep Shell
requests return `ErrUnsupported` without retiring the receiver.

#### Denoted offset

Let `p(s)` follow the section counterclockwise at unit speed, with tangent
`T`, inward normal `n = J T`, curvature `κ`, and `J` the quarter-turn. At
height `z`, the source wall is `R(kz)p(s)`, where `k = θ/H`. Put
`f = p·T`, `h = p·n`, `g = sqrt(1+k²f²)`, `a = t/g`, and `β = k²tf/g`.
Moving the source wall inward by its unit surface normal and then naming
the result by its own height gives the cavity contour
`q(s) = R(β)(p(s)+a n(s))`. The cavity wall is `R(kz)q(s)` for
`t ≤ z ≤ H`; its floor is at `z=t`. This curve remains denoted by the
authenticated Lines and Arcs. It is never replaced by the stored polygon.

On each source segment, interval arithmetic encloses `f`, `h`, `κ`, the
source length and the denoted angle. The derivative is
`q' = λ R(β)T`, with
`λ = 1-aκ-β'(h+a)` and `β' = k²t(1+κh)/g³`.
Admission requires positive lower bounds for `λ`, `κ+β'`, and the inward
support gap `a-R|β|`, where `R` encloses the source radius. It also requires
smooth source joins and positive contour area. These checks exclude a cusp,
a reversed offset, and contact with the outer contour in the admitted
class. Half-turn symmetry gives zero first moments for the cavity section.
The Green integral encloses its area by summing
`½ λ(-h-a) ds` over 64 bounded parts of each source edge. The true cup
volume lies in `H*A(P) - (H-t)*A(Q)`, with both areas enclosed by the source
certificates.

#### Held boundary and certificates

Each Line contributes eight contour cells and each Arc contributes sixteen.
The builder encloses every source and offset station with rational
trigonometric intervals. A second-derivative bound gives the offset's
maximum chord departure on each cell; the source Arc uses its circular
sagitta. Eight equal-height stations rotate paired outer and inner
contours. The held shell has two triangles per contour cell and axial cell
on each wall, a fan on the outside bottom and inside floor, and a paired
top annulus. It keeps the outer and inner wall as separate faces.

The wall boundary bound adds contour chord departure, angular interpolation
`R*step²/8`, the diagonal's `edge*step/4` displacement, and outward-rounded
station error. Planar caps and rim use their contour chord and vertex errors.
For each wall cell, a straight homotopy to its held triangles moves points
by at most its boundary bound. Its swept-volume allowance is that bound
times the sums of certified source and held contour lengths and axial
travel lengths. The build adds every nonnegative cell allowance. Planar
bottom, floor, and rim homotopies sweep zero occupied volume. The enclosed
true volume must be positive and lie within the resulting held-volume
allowance.

Each source wall cell and held triangle receives a non-cancelling area
allowance. The planar patches contribute at most `2*A(P)` of source area;
the held triangles contribute their outward area upper bounds. This coarse
sum remains valid when a later operation keeps only part of one face.
The builder audits held facet crossings, edge use, vertex links, and
orientation before it commits. `Tessellate(VerifyAll)` checks the stored
boundary and occupied-volume certificates at the requested tolerance.

`Copy` and `Transform` can place the source Sweep under a rigid motion. Shell
first certifies the offset cup in the Sweep's local profile and path frame.
The stored floating-point rotation has a small exact metric defect, so an
affine image of a local unit-normal offset is not exactly the world-normal
offset. The exact stored matrix supplies `e = ||LᵀL-I|| < 1/2`. Its normal
map differs from the local unit normal by at most `4e/(1-e)` and its tangent
map by at most `100e/(1-e)³`. The certificate compares the two cavity walls
at each height. It requires the height map to remain increasing, the new
contours to keep their positive radial winding, the outer-wall clearance to
remain positive, and the world-offset floor to stay between the two caps.
The floor's local height differs from `t` by at most `te/(1-e)`.

The maximum contour movement plus the moved floor bounds boundary distance.
The contour homotopy's swept area bounds each section's symmetric difference;
its height integral and the floor strip bound occupied-volume difference.
Separate absolute wall-density, floor, and rim terms bound area difference
without cancellation. These three charges augment the local cup proof before
the ordinary faceted placement path moves the held vertices, audits contact
and orientation, and adds its own rounding, area, and volume charges. A proof
refusal leaves the source Sweep and document unchanged. `capEnd` remains
selectable on the placed Sweep.

The public gate (`apitest/shell_twisted_sweep_test.go`) uses a solved 8×6 mm rounded rectangle with 1.5 mm
corners, a 5 mm path, 30° twist, and 1 mm inward Shell thickness. The cup's volume bound stays below 20 mm³ and its
volume lower bound stays positive. `Tessellate(0.1 mm, VerifyAll)` reports boundary and volume verified, with a
boundary bound below 0.1 mm and exactly 3,456 triangles: four Lines at 8 cells and four Arcs at 16 cells make 96
contour cells, which take two triangles on each of two walls over eight axial cells, plus 96 triangles on each of the
bottom fan, the floor fan and the top annulus. A 0.01 mm request refuses at the same fixed mesh. Starting-cap,
outward, and over-thick calls also refuse without changing the document.
The placed public gate applies a 23° rotation about `(1,2,3)` followed by a `(9,-4,7)` mm translation. Both `Copy`
and `Transform` Sweep sources select one `capEnd` and shell successfully. Their cups pass `VerifyAll` at 0.1 mm with
the unplaced cup's triangle count, a volume that matches it within 1e-9 mm³, a volume bound no smaller than its bound
and within 1e-8 mm³ of it, and a mesh bound within 1e-8 mm of its mesh bound.

## Part D — Draft

How decad builds a drafted body: a tapered extrude (`Extrude` with a nonzero
`WithTaper`) and a face draft of an existing prism (`Draft`). Both
produce a `draftPayload` for one-sided extents or a `twoSidedDraftPayload`
for extents that cross the sketch plane. Their walls lean at a stated angle
to the sweep axis so that a molded part releases along that axis.

Companion to `agent-docs/api-design.md` ("core §N"), `agent-docs/evaluator-design.md`
§5 (the prism this body generalises), Part F §5 and §8 (the
exact section offset and its audit, which this design reuses) and
Part R §8.3–§8.4 (the cap-loop chamfer band, whose
patch, flux and bound machinery this body is built from). Nothing here changes
those contracts.

Four tables are normative and each states its facts once:

| Table | States | Section |
|---|---|---|
| **RD** — the inputs | which profile, extent, operand and selection each entry point admits | §4 |
| **SD** — the refusals | every refusal, modify §1's existence test, its sentinel, and the gate order | §5 |
| **BD** — the result | the payload, its faces, edges and roles | §6 |
| **DD** — downstream | one row per consumer and what it does with a draft body | §9 |

Question router (navigation, not authority):

| Question | Section |
|---|---|
| What is a draft, and which offset family defines it | §1, §2 |
| Why the far section is decad's own geometry, not a `sketch` answer | §3 |
| What `WithTaper` admits | §4 Table RD |
| What `Draft` admits, and its signature | §4, §10 |
| Why a call refused, and in which order the gates run | §5 Table SD |
| What faces, edges and roles come out | §6 Table BD |
| How the body is built | §7 |
| How volume, area, centroid and bounds are computed and bounded | §8 |
| What tessellation, export, `Verify`, booleans and modify ops do | §9 Table DD, §9.1 |
| Tests an implementer must write | §11 |
| Rejected approaches | §12 |
| Decided questions | §13 |
| PR split and what each PR leaves refused | §14 |

### 1. Problem

A molded part needs every wall that runs along the pull direction to lean
away from the mold by a few degrees. A straight prism's walls run parallel to
the sweep, so a caller proving a molded part needs a tapered extrude and a
draft of the faces of a body that already exists, and `Verify`'s
pull-direction survey needs drafted walls to read. §14 states which of these
build today and `agent-docs/missing-features.md` ("Feature operations") lists the
rest.

The evaluator already builds exactly one drafted surface family: the cap-loop
chamfer band of modify-reach §8.3, which joins a cap loop at one level to its
exact offset at another level with `Plane` and `Cone` patches, and proves
every reading off it. A tapered extrude is that band with the operand's
straight part removed, so this design reuses the band's construction and
proofs and adds the far section's offset rule, the entry points and the
refusals.

### 2. The model: the sharp offset family

**A draft body is the sweep of a section family.** Write `P` for the recorded
section in its plane, `α` for the signed taper, `h` for the sweep height and
`e` for the unit sweep direction from the sketch plane toward the far end.
The section at axial distance `z ∈ [0, h]` from the sketch plane is

```text
P(z) = P offset by t(z) = z · tan α,   inward for t > 0, outward for t < 0
```

and the body is `{ p + z·e : z ∈ [0, h], p ∈ P(z) }`. The far section is
`Q = P(h)`, offset by `d = h · tan α`.

For a two-sided extent, each side starts at the same sketch section `P` and
uses its own travel direction away from the sketch plane. The negative slab
occupies `[z0, 0]`; the positive slab occupies `[0, z1]`. Each far section
uses that side's distance in `d = h · tan α`. Their common section is one
sewn rim, not an internal cap.

**The sign.** A positive `α` narrows the body with distance from the sketch
plane: every wall's carrier moves `t(z)` into the material. A negative `α`
widens it. `WithTaper` keeps its existing meaning (core §12: a signed
displacement, outside `ErrNegativeMagnitude`), and `Draft` (§10) reads
its angle the same way about its neutral plane.

**The offset is sharp.** `P(z)` is the section whose every wall carrier is
moved by `t(z)` and whose every corner is the intersection of its two moved
carriers:

| Feature of `P` | In `P(z)` |
|---|---|
| a line | the parallel line, `t(z)` into the material |
| a circular walk, material inside (a counter-clockwise round) | the concentric circle of radius `R − t(z)` |
| a circular walk, material outside (a hole wall, a concave round) | the concentric circle of radius `R + t(z)` |
| a corner between two lines, convex or reflex | the **miter**: the intersection of the two moved lines |
| a G1 join (modify §8's row: the two walks leave and arrive along one direction, decided on the held unit tangents with the same dead zone) | the corner moved along the shared left normal, `v + t(z)·n̂` |
| a whole circle | the concentric circle; no corner |

A reflex corner between two lines is a miter here, where modify §8's shell
offset inserts an arc. The two families answer two different questions: a
shell's wall must be `t` thick everywhere, so its reflex corner is the arc at
distance `t` from the corner point; a draft tilts each wall about its trace
and re-intersects the neighbours, so a reflex corner stays sharp, as the
molded part's does. §12 records the rejected alternative.

**The far section is a `ProfileRecord` in the recorded vocabulary.** Every
piece of the table is a line, an arc or a circle, so `Q` records as
`LineSeg`/`ArcSeg`/`CircleSeg` walks and every consumer that reads a recorded
section reads it. That closure is what the admitted profile class (Table RD)
protects: a corner where a circular walk meets its neighbour other than
tangentially has a miter locus that is a conic, not a line, and a free-form
walk's offset is not a free-form kind. Both refuse.

**Every corner locus is affine in `t`.** A miter between two moved lines moves
along a fixed ray at a fixed rate; a G1 foot moves along the shared normal; a
circle's radius moves linearly. So every wall between `P` and `Q` is ruled by
straight lines between corresponding points, and each wall is an exact
analytic surface:

| Wall of `P` | Surface | Normal |
|---|---|---|
| a line | `Plane` through the near segment and its far segment, leaning `α` from the sweep axis | `n̂·cos α + e·sin α`, with `n̂` the section's outward unit normal in the plane |
| a circular walk or whole circle | `Cone` about the axis through the centre along `e`, radius `R` at the sketch plane and `R ∓ d` at the far end, half angle `|α|` | the cone's own |

A circular wall's two directrices sweep **one** angular window, because both
of its corners are G1 feet (which lie on the wall's own radial through the
corner) or it has no corner. The ruled surface between two concentric arcs
over one window about one axis is the cone of revolution itself, so the
`Cone` tag names the surface built, with only the placement's rounding
between them (modify-reach §8.3's skew term is identically zero here).

**The lateral edges are `Line3`.** Each corner of `P` and its image in `Q` are
joined by the straight ruling the walls share. A miter ruling between two
`Plane` walls is their exact intersection line; a G1 ruling between a `Plane`
and a `Cone` is the line along which they are tangent.

### 3. What `sketch` is asked

Nothing beyond the recorded profile. `Q` is decad's own synthesized geometry
under modify §5's rule: the offset loop exists in no sketch the caller drew,
so there is no upstream answer to consume, and decad proves its validity with
the exact closed-form audit of `internal/sectionaudit/` (modify §5's four
tests) rather than with a residual.

The reason that rule applies here, stated once per CLAUDE.md's hard rule: at
decad's pin, `sketch.CreateOffset` offsets lines only (non-line entities are
skipped), places the copies through the floating Levenberg–Marquardt solve,
certifies nothing (`Enclose` refuses `Offset` constraints with
`ErrUncertifiedConstraint`) and checks neither collapse nor self-intersection.
No answer decad could consume exists upstream, and a certified one would still
leave the audit decad's, since the loop is decad's construction. No hand-off
file is written.

### 4. Table RD — the inputs

| RD | Entry point | Admits | Result |
|---|---|---|---|
| **RD1** | `Extrude` with nonzero `WithTaper` | analytic walks with G1 circular joins; `Distance`, `ToFace`, `ThroughAll`, `Symmetric` or `TwoSided`; `|α| < 90°`; no `WithSurfaceResult` | a one- or two-slab draft solid (Table BD) |
| **RD2** | `Draft` | a live `prismPayload` operand (an extrude, a filleted or chamfered body, a tube, or any of these after `Transform`) whose section carries no displacement; a `NeutralFace` naming one cap of the operand; a selection resolving to exactly the operand's complete wall set; `0 < |α| < 90°` | a `draftPayload` solid over the operand's section, the operand retired |
| **RD3** | `Draft` | as RD2 with a selection naming a **subset** of the walls, where every corner between a selected and an unselected wall is a line–line corner | a `draftPayload` whose far section moves the selected walls only (§10.2) |

Everything else is Table SD.

### 5. Table SD — the refusals

Each row states what was asked, modify §1's test (does the body exist under
any evaluator?) and the sentinel that follows from it.

| SD | The call asked for | Exists? | Sentinel |
|---|---|---|---|
| **SD1** | a taper that is not an angle, or not representable in radians | — | `ErrUnitKind` / `ErrNotFinite`, as for a straight prism |
| **SD2** | `|α| ≥ 90°`, or an angle whose certified cosine enclosure (`RadSinCosInterval`) reaches zero | no — a wall at a right angle to the axis sweeps nothing | `ErrDegenerate` |
| **SD3** | a free-form walk in the section | yes | `ErrUnsupported` — the offset of a Bézier span is not a recorded kind; §12 names the chorded alternative and rejects it |
| **SD4** | a corner where a circular walk meets its neighbour that the held-tangent rule classifies as a miter (not G1), or, in a subset draft, a G1 corner at a circular walk whose two walks move by different amounts (§10.2) | yes | `ErrUnsupported` — the junction is a conic; the ruled stand-in of modify-reach §8.3 is not implemented (§14) |
| **SD5** | a circular walk whose far radius collapses: `offset2d.OffsetRadius` reports `ok == false` (`R ≤ d` for a round with the material inside under a positive taper; a hole wall under a negative one) | yes — the cone's apex lies inside the sweep and the body past it has another topology | `ErrUnsupported` (modify S11a) |
| **SD6** | a far outer loop whose signed area has changed sign (modify S8), whose every walk the offset consumes (`offset2d.ErrLoopConsumed`) under a positive taper, or a far hole the audit decides lies outside the far outer loop (modify S9's decided row) | no — the taper consumed the region | `ErrDegenerate` |
| **SD7** | a far walk `offset2d.WalkConsumed` reports consumed — a line whose two miters have crossed — where some walk of its loop survives, or a hole every walk of which is consumed | yes | `ErrUnsupported` (modify S11a) |
| **SD8** | far loops that cross or make boundary contact at modify §5's scale-anchored floor (S7/S11b) — two walls closing on each other, a widened hole reaching the outer loop | yes | `ErrUnsupported` |
| **SD9** | far loops whose nesting the containment classifier cannot decide (modify S9) | undecidable here | `ErrUnsupported` |
| **SD10** | a far section that passes every audit at the held `d` and fails at the top of `d`'s span (§8.1; modify-reach §8.3.1's second audit) | undecidable here | `ErrUnsupported` |
| **SD11** | not used: `Symmetric` and `TwoSided` build one draft slab per side (BD4) | — | — |
| **SD12** | a nonzero taper with `WithSurfaceResult` | yes | `ErrUnsupported` (staged, §14) |
| **SD13** | a `d` that rounds to zero, a far vertex bit-identical to its near vertex, or a far radius bit-identical to its near radius (`capband.BandRadius`) | yes — float64 cannot name the far section at this scale | `ErrUnsupported` (modify-reach SX13) |
| **SD14** | a far corner whose displacement enclosure cannot be built (`offset2d.ErrUnbounded`) | yes | `ErrUnsupported` (SX14) |
| **SD15** | a corner whose two moved carriers have no intersection — a cusp (`|cross| ≤ tol`, `dot ≤ 0`) between two lines | yes | `ErrUnsupported` (`offset2d.ErrTopology`, modify S11's row) |
| **SD16** | a wall patch whose outward orientation `capblend.FixPatchOrientation` cannot certify | yes | `ErrUnsupported` (SX15) |
| **SD17** | `Draft` of a retired operand, or of a sheet | — | `ErrRetiredBody`; a sheet as `Fillet`'s `refuseSheetOperand` refuses it |
| **SD18** | `Draft` with a zero angle | it exists and is the operand (modify S13) | `ErrDegenerate` |
| **SD19** | a neutral selector resolving to zero or several faces; a curved neutral face; a `Faceted` neutral face | — | `ErrCardinality` (Expected "exactly 1"); `ErrDegenerate`; `ErrUnsupported` — `MirrorFace`'s own three rules |
| **SD20** | a `NeutralFrame`, or a `NeutralFace` that is not a cap of the operand | yes | `ErrUnsupported` (staged, §14) |
| **SD21** | a selected face that is no wall of the operand, or a selection that splits the recorded segments of one coalesced walk; an empty selection | yes | `ErrUnsupported`; `ErrNoMatch`/`ErrCardinality` as core §9 |
| **SD22** | a selected face parallel to the neutral plane — the other cap — or the neutral face itself | no — a face with no trace on the neutral plane has no line to tilt about | `ErrDegenerate` |
| **SD23** | `Draft` of an operand that is not a `prismPayload`, or whose section carries a displacement (`prismrecord.Payload.RequireExactSection`), or that is itself a draft body | yes | `ErrUnsupported` (modify S3; a second draft is SX10's composition rule) |

The sharp offset of a convex section past its collapse is the section
point-reflected through a centre, which keeps its walk sense, so S8 never sees
a square tapered past its apex; stage 4 reads that outer loop, every walk
consumed, as SD6. Two concentric far circles never cross: past the closure of
F7's ring the far hole lies outside the far outer loop, which is SD6 through
the audit's decided nesting row, not SD8. No recorded profile holds a cusp
between two lines, and a cusp at a circular walk is SD4 first, so Extrude never
reaches SD15; `internal/offset2d/sharp_test.go` pins it.

**A chain section never reaches this build.** `Extrude` over an open
`*sketch.Chain` returns `ErrUnsupported` for `WithTaper` at any angle, zero
included, after the chain's seam gates and before extent resolution
(`agent-docs/surface-design.md` §13.2), so none of Table SD applies to a chain.
`Extrude` threads its `ctx` through the extent resolution and both slab builds
of a profile with a nonzero taper; a canceled `ctx` returns `ctx.Err()` before
commit, unwrapped by the stage names the two-sided build adds to other errors.

**Gate order.** Each stage needs the one before it, and where two gates read
one constructed section the existence question is asked first (modify §4).

| Stage | Gates, in order |
|---|---|
| 1 — the pre-gates | `Extrude`: the seam gates of core §7, then SD1, SD2 (the seam gates run first, as they do for SD1 on a straight prism). `Draft`: SD1, SD2, SD17, SD18, SD19's cardinality |
| 2 — the operand and its inputs | `Draft`: SD23, SD20's `NeutralFrame` refusal, SD19's kind rules, SD20's cap test, SD22, SD21. `Extrude`: the extent's own validation (unchanged), SD12 |
| 3 — the section's kinds | SD3; then per corner SD4 and SD15 |
| 4 — the far section is built | the span of `d` (§8.1); SD5, SD7 and SD6's consumed outer loop as the walks are offset, then SD13 |
| 5 — the audit | SD6, then SD8, then SD9, over the far section at `d` |
| 6 — the span re-audit | SD10 |
| 7 — the patches | SD14, SD16 |

Every gate runs before a face is made, and a refused call leaves the
document unchanged (evaluator §8).

### 6. Table BD — the result

| BD | Entry | Payload | Lumps | Faces and roles | Edges |
|---|---|---|---|---|---|
| **BD1** | `Extrude` + taper | `draftPayload`: the near record `P`, the far record `Q` with its displacement, the frame, `[z0, z1]` with the end displacements, `α` with the span of `d`, the placement | 1 | `capStart` over the section at `z0`, `capEnd` over the section at `z1` (the prism's convention: `Along` puts `P` at `z0`, `Against` puts `P` at `z1`); one wall per walk of `P` with role `side(i, j)`: `Plane` for a line, `Cone` for a circular walk, a seamless `Cone` for a whole circle | cap rims `Line3`/`Arc3`/`Circle3` from each record; lateral edges `Line3`, convex by the walk's turn exactly as a prism's (evaluator §3) |
| **BD2** | `Draft` (RD2) | the same payload over the operand's record, under the operand's placement | 1 | as BD1, every role minted under the result's own producer (modify §11: a result's roles are its own) | as BD1 |
| **BD3** | `Draft` (RD3) | the same payload; `Q` moves the selected walls only | 1 | as BD2; an unselected wall keeps its vertical `Plane`/`Cylinder` | as BD1 |
| **BD4** | two-sided `Extrude` + taper | `twoSidedDraftPayload` with a negative and positive `draftPayload` | 1 | outer `capStart` and `capEnd`; walls `side(0,i,j)` below and `side(1,i,j)` above the sketch plane | one shared middle rim; the two slabs have no published middle cap |

A full-circle loop builds one `Cone` with two `Circle3` rims and no seam edge,
as a cylinder does today. Adjacent `Plane` walls whose two recorded segments
lie on one exact line canonicalize into one face by evaluator §3's rule, which
reads the record and not the surface.

### 7. Construction

`draftrecord.Build` (`internal/draftrecord/build.go`) builds the body from the
payload over topology nodes. The steps, in order:

1. **Resolve the walks.** `boundarywalk.WalkOf`, `RequireAnalyticWalk` (SD3)
   and `CoalesceWalksBudget` over every loop of `P`, as
   `capblend.OneLoopCornerLoop` does.
2. **Compute `d` and its span** (§8.1).
3. **Classify every corner** with `offset2d.CornerJoin`'s held-tangent rule.
   A corner at a circular walk that is not G1 is SD4.
4. **Offset every loop sharply.** `offset2d.BuildSharpLoop(budget, walks, s,
   d, tol)` is `BuildLoop` with one change: a corner the rule would send to
   the reflex-arc row takes the miter row instead. Its corner table is §2's,
   and it returns the joins beside the record for SD13's corner test. A
   missing miter root is SD15; a collapsed radius is SD5; a consumed walk is
   SD7, and a loop whose every walk is consumed is `offset2d.ErrLoopConsumed`
   (SD6 for the outer loop under a positive taper). The result is `Q` as
   `[]CurveSegment` per loop, assembled into a `ProfileRecord` with the same
   loop order as `P`.
5. **Audit `Q`** with `offset2d.AuditOffsetSectionBudget` (modify §5): SD6, SD8, SD9.
   Re-run steps 4–5 at the top of `d`'s span (`capblend.AuditSetbackSpan`'s
   rule): SD10.
6. **Enclose the far corners.** `capband.ContourDisplacement`
   (`capcontour.Displacement` over the sharp joins, across the span of `d`)
   gives each loop's far
   contour displacement, and the payload's `farDelta` is the largest; a
   corner it cannot enclose is SD14. `offset2d.SectionDeltaFromWalks` is not
   used: it re-derives the joins by the shell's rule, which closes a reflex
   corner with an arc the sharp offset never builds.
7. **Build the caps** with the prism's cap builder over `P` at its level and
   over `Q` at the far level. The far cap's vertices carry `farDelta` beside
   their frame-lift rounding (evaluator §5's `liftedVertex`), as the cap-blend
   cap does.
8. **Build the walls.** Per walk, one `capband.Patch`: a line gives a Plane
   patch with `SideA/B` the near segment and `CapA/B` the far one; a circular
   walk gives a Cone patch with `SideRadius = R`, `CapRadius = R ∓ d` and
   `CapTh0/CapTh1 = Th0/Th1` (one window). The surface is
   `capblend.PlaneFromThree` or `coneSurface(pl, cu, cv, R,
   R ∓ d, zNear, zFar)`; the rims are `arcEdge` / `wholeCircleEdge` / line
   edges; the rulings are `Line3` between corresponding corners; the
   orientation is certified by `capblend.FixPatchOrientation` (SD16). Each patch
   carries `ContourAllow = farDelta`, `LevelDelta` = the far end's axial
   displacement, and its `HeldAllow` as modify-reach §8.4's table states.
9. **Stamp roles and convexity**: `side(i, j)` per walk, `capStart`/`capEnd`,
   lateral edges convex by the walk's turn.
10. **Measure** (§8), then commit.

Steps 7–9 run through the cap-loop band itself. `draftPayload.Band()` is a
`capBlendPayload` view with every loop chamfered on the far cap, `dc = d`
across the cap and `ds = h` down the sweep, so the band's side level is the
sketch plane and no straight slab remains. Its `draft` flag makes
`capblend.BuildBand` (`internal/capblend/band.go`) take the sharp joins
(`offset2d.SharpJoinsBudget`), stamp `side(i, j)` roles, give rulings and far
rims the prism's convexity, and flip the axial half of the orientation
reference for a negative taper. The near rim is `draftrecord`'s `nearRim`: the bottom
rim `prismrecord.BuildLoopSidesAs` builds for a prism's wall, at the sketch plane.

For a crossing extent, `draftrecord.BuildTwoSided` builds the negative and positive
slabs independently with steps 1–9. It checks that their held sketch rims
match vertex for vertex, then removes both middle caps and reuses the same
rim edges and vertices. The negative slab's published face loops are reversed
so the two wall faces use each sewn edge in opposite directions. The
composite sweep boundary and adjacent-span audits check the resulting shell.
The two slab volumes and first moments add; the body area sums its published
faces, and its bounds enclose both slabs.

`Draft` reaches step 1 with the operand's record and placement after its
own gates (§10), and builds the identical body `Extrude` would build from the
same section and sweep.

#### 7.1 Vertex bounds

A near vertex is the prism's: its recorded coordinates, its junction bound
(`boundarywalk.JunctionVertex`) and its own frame-lift rounding. A far vertex
adds `farDelta` and the far level's axial displacement. Every reading that
takes a far coordinate charges both terms, and no reading reads the far level
as exact.

### 8. Measurements

The draft body is one closed sub-solid: the near disk, the far disk and the
wall band between them. Every reading reuses the cap-loop band's closed forms
over the patches of step 8, with the band's `dc = d`, `ds = h`, and no
operand slab.

| Reading | Closed form | Owner of the code |
|---|---|---|
| Volume | the divergence theorem in plane-local coordinates: the near disk's area at its level, the far disk's area at its level, and `Σ capband.RawFlux(patch)`, divided by 3 (`capband.BandVolume`). A Plane patch's flux is the exact rational tetrahedron identity; a Cone patch's is the closed-form polynomial-plus-trig flux over rationals with certified `sin`/`cos` enclosures | `internal/capband/band_mass.go` |
| Area | the two disks' region areas plus `Σ capband.AreaOf(patch)`: a Plane patch's two-triangle sum, a Cone patch's frustum sector `(αc/2)(R0 + R1)·L` with `L = √(ΔR² + H²)` | `internal/capband/area.go` |
| Centroid | the first moment by the same divergence theorem (`capband.BandMoment`), divided by the volume and lifted to world through the frame and placement; the geometric safety-net bound (the centroid lies within `Bounds`) as a ceiling | `internal/capband/band_mass.go`, `internal/capband/moment.go` |
| `Bounds` | per direction the extreme over both records' analytic extremes at their levels, each charged with its own displacement and the lift rounding; a placed body charges the placement's rounding as evaluator §5 states | `internal/prismrecord/extent.go`'s readers over both records |

**Bounds compose as modify-reach §8.4 states, with the same four helpers.**
The far cap's area composes `sectionDisplacementArea(farDelta, walks,
perimeterUpper)`; the volume composes `SweptVolumeAllow(farDelta, areaUpper)`
once after the flux sum; each patch's area composes `BandPatchAreaAllow` and
`BandLevelAreaAllow`; the first moment composes `SweptMomentAllow` once per
body. `capband.ClosureOf` charges the sliver at every corner whose held ends
are not provably one point, exactly as the band does. The chord-versus-locus
term (`ChordLocusVolumeAllow`) is read and is zero, because both windows of
every Cone patch coincide; the code keeps the call and asserts the zero
rather than omitting the term.

**The wall normal.** `Face.NormalAt` on a draft wall carries the same
denoted-wall term a chamfer band's patch does (modify-reach §8.3): the turn
between the built wall and the wall the taper denotes, since the far rim sits
within the contour displacement of the denoted one. A Plane wall's tag passes
through its exact near edge and one far corner, and moving that corner by `e`
turns the normal by at most `2|e|/h`; a Cone wall's half angle moves by at most
`(e_r + |Δr|·e_z/h)/h`. `capband.DenotedNormalAllow` charges
`2·(δ + δ_z·max(1, |Δr|/h))/h` with `h` at the bottom of its span, and
`setPatchReadings` (`internal/capblend/band.go`) adds it to every draft wall and chamfer
patch alike. `δ_z` is the far level's displacement beside the near level's
rounding, and no near-level displacement enters. Every admitted one-sided
extent puts its near end on the sketch plane at exactly zero with a zero
displacement; the far end carries the distance's conversion rounding or the
stop level's bound. The near edge is therefore the recorded section itself.
The far displacement stays in `δ_z` even though it would cancel
for a chamfer, whose two levels both follow one held cap level: here
`h = z1 − z0` is read from the held far level, so that displacement moves the
height the taper is measured over. Each two-sided slab also puts its near
level at the exact sketch plane; its far level carries its own displacement.

**Exactness.** A draft measurement is `Exact` only where every term of it is
exactly representable. The tangent of the taper is a certified enclosure of
positive width (§8.1), so `d` is never exact and every measurement of a draft
body carries a nonzero bound; an axis-aligned drafted box at the identity
placement reads its volume `Approximate` with a bound of the order of `d`'s
own enclosure width times the box's lateral area. That is the same status a
`Cone` patch has today ("never `Exact`: the enclosure always has width",
modify-reach §8.4) and nothing here weakens core §5.3: a zero bound is a
claim, and the claim is false for every drafted body.

**The closed forms the tests check against** (§11) are the same quantities
read the other way: with `A(z)` the area of `P(z)`, a quadratic in `z` because
every corner locus is affine and every circular radius linear, `Volume =
∫₀ʰ A(z) dz` and the axial first moment is `∫₀ʰ z·A(z) dz`. For a drafted
rectangle `a × b` that is `h·(ab − (a + b)·d + 4d²/3)`; for a drafted disk the
frustum `πh(R² + Rr + r²)/3` with `r = R − d`. The production path integrates
the patches, never these polynomials; §12 records why.

#### 8.1 The offset amount and its span

`d` is read from three held numbers, and every one of them rounds:

| Term | Source | Charged how |
|---|---|---|
| `α` in radians | `extent.MagnitudeInBounded` of the stated angle (degrees convert with rounding) | the conversion's own displacement, as a chamfer setback's `dcDelta` is |
| `tan α` | `RadSinCosInterval` over the radian enclosure: `tan = sin / cos` in rational interval arithmetic, `cos` bounded away from zero (SD2 otherwise) | the enclosure's width |
| `h` | `|z1 − z0|` with the ends' own axial displacements (`z0Delta`/`z1Delta`) | the far end's displacement |

`d` is the float product; `dDelta` is the reach of the exact rational product
of the three enclosures from it. `offset2d.OffsetAmount(s, d, dDelta)` then
spans every denoted amount, every corner enclosure (step 6) is taken over that
span, and the audit runs a second time at the span's top (SD10). A millimetre
sweep and a degree angle still carry a positive `dDelta`, from the tangent.

The far level is `zFar = ±h` as the extent resolved it; the draft adds no
level of its own, so `LevelDelta` is the extent's own axial displacement and
nothing else.

### 9. Table DD — downstream

| DD | Consumer | What it does with a draft body |
|---|---|---|
| **DD1** | `Body.Tessellate` | chords both records once with the shared curve samples (tessellation §3), rules each wall between its two rims with the cap-band strip emitter (`capblend.EmitBand` in `internal/capblend/mesh_band.go`, over `internal/tessellation`'s rings), triangulates both caps; manifold by construction; the volume proof is the band admission of `agent-docs/tessellation-reach-design.md` §7 with no slab (§9.1). A band the admission refuses meshes for export with no volume proof |
| **DD2** | `Union`/`Cut`/`Intersect` | the mesh path (evaluator §9) over DD1's mesh, admitted by `booloperand.RequireVolumeProof` through the same band admission; the analytic prism reduction's entry gate (`internal/prismboolean`) does not admit the class and takes the mesh path |
| **DD3** | STL / OBJ / 3MF | the mesh of DD1 |
| **DD4** | STEP | the faceted writer. A drafted polygon's tilted planes with `Line3` loops and a drafted circle's full conical wall pass the analytic face admission, but each drafted wall's tag is a rounding of the tilted surface and states a nonzero `NormalBound`, and a face that states a departure sends the whole body to the faceted writer (`agent-docs/step-export-design.md`) |
| **DD5** | `Verify` validity | built by construction after the §5 audit, as a prism is; `Built` and `Solid` read true |
| **DD6** | `Verify` tolerance reference | `gatediameter`'s body gate has an arm (`draftGateDiameter`) beside `pairkernel.BodyGateDiameter`: the lower-bound diameter over wall stations on the near section at the near level and on the recorded far section at the far level, each charged its gap plus the level displacement and `farDelta`, falling back to the held vertices with their bounds (verification §3). Without it every draft body would read `DiagToleranceReferenceUnavailable`, since its bounds are never zero |
| **DD7** | `Verify` undercut survey (`WithPullDirection`) | `internal/capblendsurvey` reads built face normals and decides each wall and cap as described below. |
| **DD8** | `Verify` wall and concave-radius surveys | `Suspect` with `DiagUnsupportedSurveyPayload` (not implemented). The available reduction: the thinnest wall between two drafted skins is the far section's 2D reading scaled by `cos α`, and the smallest concave radius of a hole wall is its radius at the narrower end; neither is implemented |
| **DD9** | `Verify` clearance (`WithClearances`) | `Suspect` (`boundaryaudit.PairUndecided`): `newBodyGeomBudget` has no carrier model for the class. The Plane/Cone carriers are not implemented |
| **DD10** | `Verify` interference | box-disjoint proofs from the body's `Bounds`; the read-only mesh intersection over DD1's mesh |
| **DD11** | `MassProperties` and `dynamics` | the verified-mesh path (`verifiedMeshMassProperties`) over DD1's mesh; `ErrUnsupported` where the band admission refuses |
| **DD12** | `Transform`, `Copy`, `Mirrored`, `MirroredCopy` | the payload implements `Transform()` and `Placed()`: a placement re-runs §7 under the composed motion, as a prism's does. `WithJoin` refuses (it rewrites a prism's or stacked prism's section) |
| **DD13** | `Patterned` | `placedInstance` and a boolean `Union` (the mesh path); the frame-keeping arm does not take the class |
| **DD14** | `Fillet`, `Chamfer`, `Shell` | A positive-taper, one-sided, full circular solid admits one complete far-cap outer rim fillet (§10.4), symmetric chamfer (§10.5), or inward shell with that cap removed (§10.6). Other selections and draft shapes remain modify S3, `ErrUnsupported` |
| **DD15** | `Draft` of a draft body | SD23 |
| **DD16** | `Thicken`, `Offset`, `Patch`, `Stitch`, `Trim`/`Extend`/`Split` | not reached: a draft body is a solid (SD12 refuses the sheet form) |
| **DD17** | Motion and linkage bounds | `internal/motionengine/recordradius.go` reads near and widened far section bounds. `internal/motionbound/record_radius.go` reads each segment's carrier and walk. |
| **DD18** | Selectors | `Planar()`, `FaceCreatedBy`, `CapStart`/`CapEnd` and `Walls(b)` (§10) select as on a prism. `Facing(v)` and `NormalTo(v)` match a drafted `Plane` wall only for its own tilted normal, since both require parallelism; a caller naming a drafted wall by direction passes that normal, or selects by role |

DD7's `capblendsurvey.DraftUndercuts` reads each built wall's `*topo.Face`
through `topobuild.NormalAt`. `capblendsurvey.PatchNormalRange` encloses its
normal component along the pull, and `capblendsurvey.PatchUndercuts` shares
that reading with cap-blend bodies (DX7). `survey2d.CapNormalDecision` reads
each cap; `survey2d.DecidePull` lists, clears or leaves a face undecided under
modify-reach §8.3's rule. A wall drafted at `α > 0` against a pull along `e`
reads `sin α > 0` and clears. A pull in a wall's tangent plane leaves it
undecided because the reading includes the taper's enclosure.

The two-sided payload composes DD1–DD6, DD10–DD13 and DD17 from its two
slabs. Its undercut survey is still `Suspect`: DD7's current face-role lookup
handles one slab only. Wall and concave-radius surveys remain staged for both
payloads (DD8).

#### 9.1 The mesh

`draftrecord.Tessellate` hands `draftPayload.Band()` to the cap-loop chamfer
tessellator with the payload's own wall geometry and far contour displacement
(`patches`, `bandDelta`, filled by the build), so the mesh charges the numbers
every reading of the body charged. Three things differ from a chamfer's mesh:

- no trimmed side wall is emitted, and the near cap's rim is written once and
  read by both the near cap and the walls (`capblend.DraftNearRing`);
- a wall patch carries `side(i, j)` and the band has no apex patch, since the
  corners are read by the sharp rule (`capBlendPayload.offsetJoins`), which the
  admission (`capblend.OccupiedVolumeAdmission`) reads too: a reflex line-line
  corner is a miter, affine in the amount, and is admitted;
- the slice-wise chord term integrates over the whole sweep, `|z1 − z0|` with
  both ends' displacements (`capblend.Payload.DraftBandHeightUpper`), not over a side setback.

A wall face's bound charges the far level's displacement and the band's, as a
chamfer patch's does, and no near-level term: the near end of every admitted
one-sided extent is the sketch plane, exact (§8). Each slab of a two-sided
extent has the same exact near level and its own bounded far level.

`draftrecord.TessellateTwoSided` meshes both slabs through that same
tessellator, drops the two middle caps, and identifies vertices with equal
held coordinates. The closed-mesh audit refuses a mismatch in the two middle
rings. The outer facets retain their source faces and displacement bounds;
area slack and occupied-volume difference bounds add over the two slabs.

The admission refuses a trimmed segment, an arc whose recorded end is off its
circle (a cap-loop chamfer restates that arc,
`agent-docs/tessellation-reach-design.md` §7.1; a draft does not), and a
circular wall whose join is G1 by the held-tangent rule but not exactly
tangent over the rationals, unless §9.1.1 admits that join. That mesh
serves export, and every boolean, interference and mass reading refuses it,
naming the loop, the corner's plane-local point and the two recorded segments
that meet there (or the one segment).

#### 9.1.1 Near-tangent joins

A corner the build reads as G1 that `capband.JoinIsG1` does not prove exactly
tangent turns by a sliver θ over the rationals. The sharp family's corner
there (§2) is `m(t)`, where the moved line meets the moved circle, or the two
moved circles meet, on the branch that starts at the recorded corner `v`; it
runs along a conic, not along the ruling the mesh holds. The build holds the
shared-normal foot, and
the body's own readings charge that gap through the far contour's G1 hull and
`capband.ClosureOf` (§8, §12). The mesh proof charges the sliver as a volume
term, never by admitting a small cross product: the term is a proven upper
bound, and a corner it does not cover refuses.

**The class** (`capband.NearTangentCorner`, exact rational arithmetic over
the record's floats). A complete-loop fillet reads the same class and Lemmas
1 and 1′ with its own charges (Part L §5.5). A
draft band, or a cap-loop chamfer band ("On a cap-loop chamfer band" below),
admits the corner when all six hold:

1. one walk is a `LineSeg` and the other an `ArcSeg`, or both are `ArcSeg`s;
2. the two recorded junction points are one rational point `v`;
3. the two exact walk tangents `a` and `b` (as `JoinIsG1` states them) have
   `a·b > 0`;
4. `S = (a × b)² / (|a|²·|b|²)`, the exact `sin²θ`, is at most `2⁻⁶⁴`, so
   `sin θ ≤ 2⁻³²`, under a quarter of the build's dead zone;
5. both walks move by one amount;
6. the moved circles stay clear of the corner, with `R = |v − C|` and `r_lo`
   the smallest moved radius any level reaches: for a line and an arc,
   `r_lo > ρ + R_hi·S`; for two arcs, `r_lo > ρ` on each circle.

Failing 2 or 3 keeps the exact-tangent refusal, and so does failing 1, which
no corner reaches: a corner of two lines is a miter. Failing 4 refuses "a
near-tangent join whose exact turn exceeds the admitted sin²θ ≤ 2⁻⁶⁴", 5 "a
near-tangent join whose two walls move by different amounts" (unreachable after SD4), and 6 "a
near-tangent join whose moved circle comes within the near-tangent corner
charge of the corner" for a line and an arc, "a near-tangent join of two
circular walls whose moved circles do not both clear the near-tangent corner
charge" for two arcs. The limit in 4 is a stated class, not a soundness step:
a join a sketch tangent constraint solves turns by rounding, orders below
`2⁻³²`, and a corner turning more was drawn as a corner and keeps the
refusal a genuine miter has.

**B1 at the corner.** At a line and an arc, write `n_ℓ` and `n_c` for the two
walks' unit left normals at `v`, `cos θ = n_ℓ·n_c`. The ideal polyhedron B1 of
`agent-docs/tessellation-reach-design.md` §7 puts the corner's cap vertex at
the circle's exact offset station at its window end
(`tessellation.CapBlendCapMotion`: a line's first cap sample after a circle is
bounded against that circle's last station, a circle's first sample against
its own first station). That is the radial foot `f(t) = v + t·n_c`, and its
ruling from `v` stays radial, so B1's corner at amount `t` is `f(t)` on the
moved circle. `stationbound.CapOffsetStationBound` measures the held vertex
against that exact station, so the vertex-motion term already charges the
held foot's distance from `f`. The circular wall's cells keep their planar
trapezoids. The line wall's one cell no longer has parallel near and cap
edges; B1 splits it along the mesh's diagonal, and its section at `t` is the
path `f₁ → q → f₂` with `q` the diagonal's point at that level.

At a corner of two arcs, walk 1 arriving and walk 2 leaving with unit left
normals `n₁`, `n₂` at `v` and `cos θ = n₁·n₂`, the corner's cap
vertex is walk 2's first station, the radial foot `f₂(t) = v + t·n₂` on walk
2's moved circle. Walk 1's own radial foot `f₁(t) = v + t·n₁`, on its moved
circle at its window end, is no vertex of B1. Walk 2's cells keep their
planar trapezoids. Walk 1's last cell joins side station `A` (the station
before `v`) and `v` to cap station `A'` and `f₂(d)`; its cap edge does not
parallel its side edge, so the cell is not planar, and B1 splits it along the
mesh's diagonal. Its section at `t` is the path `P → q → f₂`, where
`P = P(t)` is the station's point on walk 1's moved circle (affine in `t`,
since both `A` and `A'` sit at one azimuth about `C₁`), and `q` is the
diagonal's point. Had the cap vertex been `f₁(d)`, the cell would be planar,
its section the chord `P → f₁` and its diagonal's point some `q″` on that
chord. Either diagonal puts `q` at `q″` or `q″ + δ`, with
`δ = f₂(t) − f₁(t) = t·(n₂ − n₁)`, since moving one cap vertex by
`f₂(d) − f₁(d)` moves the level's point on a ruling from it by the level's
fraction of that.

**Lemma 1 (a line and an arc).** For every amount `t` of the sweep,
`|m(t) − f(t)| ≤ |t|·sinθ·√(1 + sin²θ)`, and `f(t)` lies `|t|·(1 − cos θ) ≤
|t|·S` from the moved line ℓ(t).

Let `u = v − C`, `R = |u|`, `ι = +1` for a counter-clockwise arc (material
inside) and `−1` otherwise, so the moved radius is `r(t) = R − ιt`, and `e`
the line's unit direction. A point `v + t·n_ℓ + x·e` of ℓ(t) is on the moved
circle exactly when `x² + 2βx + 2tκ = 0`, with `β = u·e` and
`κ = u·n_ℓ + ιR`. Since `u = −ιR·n_c`, `κ = ιR(1 − cos θ)` and
`|β| = R·sin θ`. The discriminant is
`Δ(t) = R(1 − cos θ)(2r(t) − R(1 − cos θ))`, positive because
`R(1 − cos θ) ≤ R_hi·S < r_lo` (condition 6), and the root that starts at
zero is `x = sgn(β)(√Δ − |β|)`. In the frame `(e, n_ℓ)`,
`m − f = t·(X − s_c, 1 − cos θ)` with `s_c = n_c·e = ±sin θ` and
`X = x/t = sgn(s_c)·g`, where
`g = 2R(1 − cos θ)/(√Δ + R·sin θ)` lies in `[0, 2 tan(θ/2)] ⊆ [0, 2 sin θ]`
because `cos θ > 0` (condition 3). So `|X − s_c| = |g − sin θ| ≤ sin θ`, and
`1 − cos θ = sin²θ/(1 + cos θ) ≤ S`. ∎

**Lemma 1′ (two arcs).** At a corner of two arcs, for every amount `t` of
the sweep, `|m(t) − f₂(t)|`, `|m(t) − f₁(t)|` and `|f₂(t) − f₁(t)|` are each
at most `2|t|·tan(θ/2) ≤ |t|·sinθ·(1 + S)`.

Write `n_i = −ι_i·e_i` with `e_i = (v − C_i)/R_i`, so `C_i = v + ι_i·R_i·n_i`,
the moved radius is `r_i(t) = R_i − ι_i·t`, and `cos θ = n₁·n₂ > 0`
(condition 3). The centres are `D` apart, where
`D² = R₁² + R₂² − 2ι₁ι₂·R₁R₂·cos θ`. That gives the four cases:

| `ι₁, ι₂` | Walks | Tangency | `D²` |
|---|---|---|---|
| `+1, +1` | both counter-clockwise (material inside both) | internal: one circle inside the other | `(R₁ − R₂)² + 2R₁R₂(1 − cos θ)` |
| `−1, −1` | both clockwise (two hole walls, or a concave pair) | internal | `(R₁ − R₂)² + 2R₁R₂(1 − cos θ)` |
| `+1, −1` | counter-clockwise into clockwise (an S bend) | external: the circles lie on either side of `v` | `(R₁ + R₂)² − 2R₁R₂(1 − cos θ)` |
| `−1, +1` | clockwise into counter-clockwise | external | `(R₁ + R₂)² − 2R₁R₂(1 − cos θ)` |

The proof below carries `ι₁, ι₂` as signs and covers all four at once.
`f₂(t) = C₂ + r₂·e₂`, so `f₂` lies on the moved circle 2. Write its points as
`C₂ + r₂(cos ψ·e₂ + sin ψ·τ₂)`, `τ₂ = rot90(e₂)`, and `w = C₂ − C₁`. Such a
point lies on the moved circle 1 exactly when
`a(1 − cos ψ) − b·sin ψ = q`, with
`a = w·e₂ = ι₁ι₂R₁cos θ − R₂`, `b = w·τ₂`, `|b| = R₁ sin θ`, and
`q = ι₁R₁t(1 − cos θ)/r₂`; the right side is `f₂`'s power about the moved
circle 1, `|f₂ − C₁|² − r₁² = 2ι₁R₁t(1 − cos θ)`, over `2r₂`. With
`u = tan(ψ/2)` that is `(2a − q)u² − 2bu − q = 0`, whose discriminant
(over four) simplifies, using `r₁ + ι₁t = R₁` and `r₂ + ι₂t = R₂`, to

```text
Δ′ = b² + q(2a − q) = R₁R₂(1 − cos θ)·(2r₁r₂ − (1 − cos θ)R₁R₂) / r₂²
```

Condition 6 makes `Δ′ > 0` at every level. Write `s = sin θ`. Since
`ρ ≥ T·s ≥ |t|·s` and `r_i > ρ`, `R_i = r_i + ι_i·t < r_i(1 + 1/s)`, so
`(1 − cos θ)R₁R₂ = s²R₁R₂/(1 + cos θ) < r₁r₂(1 + s)²/(1 + cos θ) ≤ 2r₁r₂`,
the last step because `(1 + s)² ≤ 2(1 + cos θ)` for `s ≤ 2⁻³²` (condition 4).
So the two moved circles meet in
two distinct points throughout,
and the root `u = −q/(b + sgn(b)·√Δ′)` is continuous in `t` and zero at
`t = 0`: it is the branch `m(t)` that starts at `v`. Then
`|u| ≤ |q|/|b| = |t|(1 − cos θ)/(r₂ sin θ) = |t|·tan(θ/2)/r₂`, and
`|m − f₂| = 2r₂|sin(ψ/2)| ≤ 2r₂|u| ≤ 2|t|·tan(θ/2)`. The same argument with
the two circles swapped (`f₁` on the moved circle 1, power `2ι₂R₂t(1 − cos θ)`
about the moved circle 2, the same `Δ′` times `r₂²/r₁²`) bounds
`|m − f₁|`; the two small roots are the same point, since both start at `v`
and the two meeting points never coincide. `|f₂ − f₁| = |t|·|n₂ − n₁| =
2|t|·sin(θ/2) ≤ 2|t|·tan(θ/2)`. Last, `2 tan(θ/2) = 2 sin θ/(1 + cos θ)` and
`(1 + S)(1 + cos θ) − 2 = cos θ(1 − cos θ)(2 + cos θ) ≥ 0`. ∎

The charge reads `ρ = T·sinUp·(1 + S)` and `ε = T·S`, where `T = |d| + dDelta`
bounds `|t|` at every level and `sinUp ≥ √S` is the upward rational root
(`proofbound.RatSqrtUp`); each is rational before one upward rounding. A
corner of two arcs reads the same `ρ` and no `ε`: no straight wall meets it.

**Lemma 2 (one slice).** At each level, `∂S1 − ∂S0` splits as a sum of
closed 1-chains:

- each B1 cell's chord against its own arc, the circular segments `Mchord`
  already bounds; at a corner of two arcs, walk 1's last chord is `P → f₁`,
  its matched window end;
- per near-tangent corner of a line and an arc, the chord from `f` to `m`
  closed by the moved circle's arc back to `f`;
- per straight walk with a near-tangent end, the path `f₁ → q → f₂ → m₂ → m₁
  → f₁`, where an exact end has `m = f`;
- per near-tangent corner of two arcs, the polygon `P → q → f₂ → f₁ → P`
  (walk 1's last cell against its chord), and the curvilinear triangle
  `m → f₁` along the moved circle 1, `f₁ → f₂` straight, `f₂ → m` along the
  moved circle 2.

At a corner of two arcs, `∂S1` runs `P → q → f₂` and `∂S0` runs from `P`
along circle 1 to `m` and along circle 2 to `f₂`; subtracting the chord
`P → f₁`'s segment and the polygon from that difference leaves exactly the
curvilinear triangle.

The split assumes every wall survives at every level, so each wall's true
edge runs from its start corner to its end corner in the walk's order; a wall
shorter than about `2ρ` could reverse and break it. The admission proves it
for every wall with a near-tangent end (`capband.NearTangentWallRefusal`).

- A straight wall: B1's two corner points move affinely between the near and
  far sections and each true corner lies within its `ρ` of B1's, so the true
  edge advances at every level when the wall's advance along its recorded
  direction exceeds `ρ_start + ρ_end` at both sections. The near advance is
  the recorded length, compared exactly; the far corners are enclosed over
  the span of `d` (the circle's radial foot at a circular neighbour, the
  moved carriers' miter at a straight one), and the enclosure's least advance
  is compared against the upward root of the recorded length. A wall that
  fails refuses "a straight wall at a near-tangent join whose advance at the
  near or far section does not exceed the near-tangent corner charge".
- A circular wall: each true end lies on the moved circle within its `ρ` of
  the recorded window's radial foot there (Lemma 1, or Lemma 1′ read from the
  wall's own circle), so it
  turns from that foot by at most `2·asin(ρ/2r) ≤ (π/2)·ρ/r_lo`. The arc survives
  when those two turns stay below the window `W`, which is at least `π` past
  a half turn and at least `|v₂ − v₁|/R_hi` within one (a chord `c` subtends
  `2·asin(c/2R) ≥ c/R`). The gate asks `(π/2)(ρ_start + ρ_end) < W_lo·r_lo`
  over the rationals, with `π/2` rounded up and `π` down. A wall that fails
  refuses "a circular wall at a near-tangent join whose window does not
  exceed the turn the near-tangent corner charge allows its ends".

The build's float consumed-wall check (`offset2d.WalkConsumed`, SD7, SD10)
runs first and proves nothing here. Winding numbers add over chains, so
`area(S1 △ S0)` is at most the sum of the areas those chains enclose. The
corner's arc starts at zero length and its
chord stays below `ρ < r_lo`, under `√2` times the moved radius, so it never
reaches a quarter turn; the segment it closes lies inside the disc on the
chord `fm` as diameter (Thales), of area at most `πρ²/4`. The five points of a
straight walk's path lie within `ε` of ℓ at that level: `m₁`, `m₂` lie on it,
and `f₁`, `f₂` and `q` lie at the level's fraction of their cap points'
distance, since the signed distance to ℓ is affine along each ruling and the
near points lie on the recorded line. Their projections onto ℓ span at most
`λ`, the loop's held bounding-box diagonal plus twice its largest vertex
motion plus `2ρ`, because each lies within `ρ` of the hull of its cell's four
exact vertices. A fan of three triangles from `f₁` covers that path, each
inside a `2ε × λ` rectangle, so it encloses at most `3ελ`.

At a corner of two arcs, split the polygon `P → q → f₂ → f₁` along `q → f₁`.
With `q = q″ + δ` (the other diagonal has `q = q″`, and its polygon is the
one triangle `q″ f₂ f₁`), the triangle `P q f₁` encloses
`½|δ × (f₁ − P)|`, because `q″ − P` runs along `f₁ − P`, and the triangle
`q f₂ f₁` encloses `½|(f₁ − q″) × δ|`. Since `q″` lies on the chord,
the polygon encloses at most `|δ|·|f₁ − P| ≤ ρλ`: `P` lies in the hull of its
cell's exact vertices and `f₁` within `ρ` of `f₂`, which does too, so
`|f₁ − P| ≤ λ`. The curvilinear triangle is the straight triangle
`m f₁ f₂`, at most `½|m − f₂|·|f₁ − f₂| ≤ ρ²/2`, plus the segment each moved
circle's arc closes against its chord. Each chord is at most `ρ` (Lemma 1′)
and below `r_lo` (condition 6), so as for the line and arc each arc stays
under a quarter turn and its segment under `πρ²/4`. The corner encloses at
most `ρλ + (1 + π)ρ²/2`.

**On a cap-loop chamfer band.** A chamfer's in-plane section at axial
fraction `s` is the erosion `P ⊖ s·d` (modify §7), so every walk moves by the
cap's one `d` and condition 5 holds. At a convex sliver the erosion's two
moved carriers meet at `m(t)`, the sharp family's corner, so Lemmas 1, 1′ and
2 hold as stated. At a reflex sliver the erosion keeps each moved carrier up
to its own radial foot and closes the gap with the connector arc of radius
`|t|` about `v`. Only the corner chains of Lemma 2 change:

- a line and an arc: the straight wall's true end is `m := f_ℓ = v + t·n_ℓ`,
  on ℓ(t) and `2|t|·sin(θ/2) ≤ ρ` from `f`. The corner chain is the chord
  `f → m` closed by the connector arc, which turns by θ, under a quarter turn,
  so Thales still bounds it by `πρ²/4`. The straight walk's path keeps its
  `3ελ`, since `m` lies on ℓ;
- two arcs: the polygon `P → q → f₂ → f₁ → P` is unchanged. The curvilinear
  triangle becomes the segment the connector arc closes against the chord
  `f₁ → f₂`, at most `ρ` long, so at most `πρ²/4 < 2.08ρ²`.

Each wall's true end still lies within its ρ of B1's corner, so the survival
gate stands. The charge below therefore covers a chamfer band as written. Its
height is the sum of the loop's chamfered caps' axial setbacks, not the
whole sweep: on the trimmed wall the corner is the recorded point, which lies
on both curves, so the chord polygon there has no corner term. A chamfer arc
whose recorded end is off its start's circle is read about its restated
centre first (`agent-docs/tessellation-reach-design.md` §7.1), and every
corner and wall test reads that centre.

**The charge.** Integrated over the sweep,

```text
Ncorner = hUpper · Σ_loops [ Σ_{line–arc corners} πρ²/4 + Σ_{straight walks with a near-tangent end} 3·ε·λ
                           + Σ_{two-arc corners} (ρ·λ + 2.08·ρ²) ]
volSymDiff = upRound(Mchord + sweptVolumeAllow(motionMax, …) + Ncorner)
```

with `hUpper` the draft's `DraftBandHeightUpper` (a chamfer loop's summed
band setbacks, `capblend.Payload.bandHeightUpper`), `2.08 > (1 + π)/2`, and a
straight walk's `ε` the larger of its two ends'. `capblend.NearTangentVolume` composes it from
`capband.NearTangentCharges`, which reads every corner as the admission does.
A band with no near-tangent join adds no term, so its `volSymDiff` is the sum
it was, and a band without a corner of two arcs sums only its line–arc
terms. For the near-tangent slot of §11 (F3 with its right centre `2⁻⁴⁰` mm off), `S ≈ 2⁻⁸⁰/25`, so `Ncorner` is tiny; the change a reader sees in
`volSymDiff` is the vertex-motion term, the held foot sitting about `d·sin θ` from B1's. The split disc of §11 (a radius-5 circle drawn as two half arcs, the second's centre `δ = 2⁻⁴⁸`
mm off) charges the `ρλ` and `2.08ρ²` terms per corner. `TestDraftNearTangentChargeCoversEveryTerm` pins the sum
term by term (`internal/draftrecord/near_tangent_arcs_test.go`).

### 10. `Draft`

```go
// NeutralPlane is the plane a draft tilts faces about. The set is sealed.
type NeutralPlane interface{ neutralPlane() }

// NeutralFace names a planar face of Body, selected and never pointed at
// (core §9), under MirrorFace's rules: exactly one face, planar, analytic.
type NeutralFace struct {
    Body *Body
    Face FaceSelector
}

// NeutralFrame names the plane through Frame's origin spanned by U and V.
// Not implemented: ErrUnsupported (SD20).
type NeutralFrame struct {
    Frame r3.Frame
}

func Draft(ctx context.Context, b *Body, sel FaceSelector, neutral NeutralPlane, angle units.Value, opts ...DraftOption) (*Body, error)

// Walls matches every face a sweep made: the side(i, j) roles of b's own
// producer. It is the sibling of CapStart and CapEnd.
func Walls(b *Body) FacePredicate
```

`Draft` is a modify operation on core §8's terms: it takes a context, resolves
`sel` against the live operand, returns a new body and retires the operand,
and leaves the document unchanged on any error, including `ctx.Err()`.
`angle` is a signed `units.Value` of `Kind` Angle (core §12 lists it with
`ToFace.Offset` and `WithTaper` as the signed displacements outside
`ErrNegativeMagnitude`); a positive angle narrows the body with distance from
the neutral plane. `DraftOption` is a sealed option tier that carries no option; it reserves
the signature and includes `option.Interface` in its method set.

#### 10.1 Resolution (RD2)

1. The neutral face resolves through `SelectFaces(neutral.Body)` under the
   implicit exactly-one rule (SD19). It must be `capStart` or `capEnd` of the
   operand (SD20).
2. `sel` resolves against the operand. The resolved set must not contain a
   cap (SD22), and every face of it must carry a `side(i, j)` role (SD21). The
   complete set of such faces is RD2; any other set is RD3 (§10.2).
3. The sweep direction `e` runs from the neutral cap toward the other cap.
   With the neutral cap at `z0`, `e` is the frame normal and the far end is
   `z1`; with the neutral cap at `z1`, `e` is its negation and the far end is
   `z0`. The section `P` is the operand's record, which lies in the frame's
   plane at level 0 and is the section of every level of a straight prism, so
   the neutral cap's section is `P` whichever cap is named.
4. `h` is the operand's height with its end displacements; `d = h · tan α`
   per §8.1.
5. §7 builds the body over `P`, `Q` and `[z0, z1]`; the far record sits at
   the far end. The result's roles are minted under the new producer.

A `NeutralFace` whose `Body` is not the operand passes SD19's cardinality and
kind rules on that body's face and is then SD20, so a curved neutral face is
`ErrDegenerate` wherever it lies. `Walls(b)` is a face predicate over the
`side(i, j)` roles of `b`'s own producer (`selectorquery.WallsKind`), and the
complete wall set is the operand's faces carrying such a role, fillet and
chamfer rounds included.

The body `Draft` builds equals the body `Extrude` would build from the same
sketch with `WithTaper(angle)` and a `Distance` extent toward the far end,
bit for bit when the operand is unplaced. §11 asserts it.

#### 10.2 The subset case (RD3)

A selection naming some walls moves only their carriers. The far section is
`offset2d.BuildSharpLoop` with a per-walk amount, `d` for a selected walk and
`0` for an unselected one, and §2's corner table read with those amounts: a
corner between two lines miters whatever the two amounts are; a G1 join
between a line and a circular walk, or between two circular walks, is a G1
foot only when both amounts are equal, and a mixed pair there is SD4 (the
moved carrier and the unmoved one are no longer tangent, and their junction
is a conic). A whole circle is selected alone or not at all. An unselected
wall builds as the prism's `Plane` or `Cylinder`, and the lateral edge between
a tilted wall and a vertical one is their exact intersection line. Every
reading of §8 is unchanged: a patch with a zero amount is a Plane or Cylinder
patch whose far directrix is its near one translated.

The payload records the unselected walls as `draftPayload.Kept`, the
`(loop, segment)` pairs read from their `side(i, j)` roles, nil when every
wall is selected; a placement re-runs §7 over the same set (DD12). Decided
details, each the reading the rule above implies:

- **Kept amounts carry no span.** A kept wall's amount is exactly zero, so its
  carrier enters the corner and contour proofs (`capcontour.AmountsDisplacement`,
  `capband.AmountsContourDisplacement`) as a point interval, while a moved
  walk keeps `d`'s span (§8.1). A kept circular wall's far radius is its own
  (`capband.WallRadius`), not `BandRadius`'s offset.
- **A corner between two kept walls stays put.** Its far point is the recorded
  corner itself, never a re-solved intersection of the two unmoved carriers;
  the contour proof still encloses the carriers' exact intersection and charges
  the gap. SD13 reads only corners and radii a moved walk touches.
- **The profile class is RD1's.** A non-G1 corner at a circular walk is SD4
  even when both of its walks are kept: the rule above admits a mixed corner
  only between lines and narrows nothing else.
- **Unequal amounts inside the G1 dead zone.** Two nearly collinear lines with
  different amounts take the miter, as the rule says; where their moved
  carriers do not meet the corner is SD15.
- **A walk is one wall.** The amounts are per coalesced walk. A selection that
  names some recorded segments of one walk and not others names no single wall
  to tilt and is SD21; a face built from one walk carries every segment's role,
  so a face selection never splits one.
- **Gate order.** The selection is read at stage 2 (SD22, SD21); the mixed
  corners are read with SD4 at stage 3, at a unit amount per moved walk.
- **The mesh (DD1).** The band tessellator reads each walk's own setback
  where it read the one setback: a kept circular wall's cap radius, its
  band patch's radius rounding, and its cap samples' motion against the
  recorded circle with no span. Nothing else in §9.1's proof changes, since a
  mixed line–line miter is still a corner locus affine in the amount.
- **The undercut survey (DD7).** A kept wall's patch is parallel to the
  sweep, so its normal component along a pull along `±e` is zero within its
  bound and the survey leaves it undecided; a moved wall reads as in a full
  draft.

#### 10.4 One circular far-rim fillet (DD14)

`Fillet` accepts the one complete far cap rim of a one-sided inward draft
whose source is a single full circular outer loop with no holes or kept walls.
The source sketch frame must have exact signed-axis basis vectors. The draft
payload authenticates the near circle, sweep levels, taper and far contour.
An arbitrary rigid placement of that body is retained by the result.

For near radius `R`, height `h`, inward amount `d`, slope `k = d/h`, far radius
`r = R−d`, and fillet radius `f`, the rounded meridian's arc centre is
`(z,ρ) = (h−f, r+kf−f√(1+k²))`. Its top contact is directly above the
centre by `f`; its side contact is
`(h−f+fk/√(1+k²), r+kf−f√(1+k²)+f/√(1+k²))`.
The five-segment meridian closes through the rotation axis and revolves a
top disk, toroidal fillet, conical side and bottom disk. The full-revolution
evaluator supplies topology, mass readings and occupied-volume proof. The
top and bottom disks also carry the draft-style `capEnd` and `capStart`
roles; the torus carries `fillet(0,2)`.

Bounded arithmetic encloses the stated radius, height, inward amount and
their derived centre and contact coordinates. The result's `sectionDelta`
charges the largest coordinate and frame-origin displacement by a factor of
64. With that charge below `f/16`, the per-coordinate error is below
`f/1024`. An endpoint ray changes by at most `2√2` times that error; the
resulting angle changes by less than four times the error divided by `f`.
Interpolating the unwrapped arc sweep of at most a quarter turn moves every
arc point by less than 16 times the error. Positive interval
clearances keep the arc outside the axis, on the cone, above the near cap and
inside the original near rim. Unproved clearances, other cap selections,
partial circles, holes, reverse taper and non-axis sketch frames remain
outside this bounded route.

#### 10.5 One circular far-rim chamfer (DD14)

`Chamfer` accepts the same source and rim as §10.4, with equal setbacks along
the cap plane and conical wall. `WithAsymmetricChamfer` remains outside this
route. The draft record and exact signed-axis sketch frame authenticate the
source. The result keeps its placement and axial cap roles.

For near radius `R`, height `h`, inward amount `d`, slope `k = d/h`, far radius
`r = R−d`, bevel setback `c`, and `n = √(1+k²)`, the cap contact is
`(z,ρ) = (h,r−c)`. The side contact is `(h−c/n,r+kc/n)`: its distance along
the slanted wall from the original rim is `c`. The meridian closes through
the axis with these two contacts, the near rim and two axial segments. Its
full revolution builds the bottom and top disks and the original and bevel
cones. The bevel cone carries `chamfer(0,2)`.

The operation requires positive interval clearance for `r−c`, `h−c/n` and
the side contact below the far level and inside the near rim. Bounded
arithmetic encloses source dimensions and both contacts. The largest
coordinate and frame-origin displacement is charged to the whole meridian's
`sectionDelta`; a line between held endpoints stays within their maximum
displacement. `evalRevolve` then supplies bounded mass and occupied-volume
proof. The removed volume has the independent closed form
`πc²/n · [r + c(k/n−1)/3]`. The far circle survives as a smaller cap rim,
and the new cone meets the source cone at one whole circular junction.

#### 10.6 One circular drafted cup (DD14)

`Shell` admits the same positive-taper full-circle source as §10.4 when the
one selected opening is its far cap and the sense is inward. The source must
have one near-start draft slab, no hole or kept wall, and an exact signed-axis
sketch frame. `WithNoOpenings`, outward shelling, another cap, a partial
circle and an additional region remain S3.

Let the source's near radius be `R`, height `h`, inward draft amount `d`,
`k = d/h`, and shell thickness `t`. The outer cone has radius `R−kz`.
The cavity floor is at `z=t`; its conical wall has radius
`R−kz−t√(1+k²)`. Its top opening therefore has radius
`R−d−t√(1+k²)`. The build requires positive interval lower bounds for
`h−t`, that opening radius, and the offset. A six-line meridian records the
axis segment below the floor, floor, cavity cone, top annulus, outer cone and
kept near cap. Its full revolution has one shell and five faces.

The near circle, far offset interval, far-contour displacement and source
frame come from the authenticated `draftPayload`. Bounded arithmetic encloses
the floor and both cone endpoints. The largest endpoint, frame-lift and
far-contour displacement receives a 64-fold whole-section charge; the
gate also requires that charge to be less than one sixteenth of the floor,
cavity and rim clearances. Each denoted meridian segment pairs with a held
line of the same kind by its affine parameter, so its interior displacement
does not exceed its endpoint displacement. `evalRevolve` applies its mass,
face, placement and occupied-volume proofs to the charged meridian. The
result adds `capStart`, `rim(0)`, `shellCap` and `shellSide(0,0)` roles to
its own revolved faces; placement re-mints them.

The independent remaining-volume formula is the difference of two conical
frusta. With `r=R−d`, `c0=R−kt−t√(1+k²)`, and
`c1=r−t√(1+k²)`, it is
`π/3·[h(R²+Rr+r²)−(h−t)(c0²+c0c1+c1²)]`.

### 11. Required tests

Every fixture asserts computed geometry against a closed form and asserts
that the closed form lies within the published bound. Closed forms are
written in the test from §8's polynomials, with `d = h · tan α` evaluated in
the test, never read back from the body. No bound literal is pinned
(amd64 and arm64 round FMA differently); a bound is asserted to cover the
closed form and to be below a stated relative ceiling. Each bound leg is
shown to fail first: delete the leg, run the fixture, record
the red run's failing assertion in the test's comment, and restore the leg.

**Extrude fixtures** (`apitest/extrude_taper_test.go`,
`internal/draftrecord/build_test.go`):

| Fixture | Section, sweep, taper | Asserts |
|---|---|---|
| F1 box | square `a = 20`, `h = 10`, `α = 5°`, `Along` | `Volume = h(a² − 2ad + 4d²/3)`; axial centroid `h(a²/2 − 4ad/3 + d²)/(a² − 2ad + 4d²/3)` above the sketch plane; far-cap vertices at `(±(a/2 − d), ±(a/2 − d), h)`; each wall `Planar()` with `NormalAt` equal to `(n̂·cos α, sin α)` within its bound; `Area = a² + (a − 2d)² + 4·(a − d)·√(h² + d²)`; 6 faces, 12 edges, 8 vertices; every measurement `Approximate` with a nonzero bound |
| F2 frustum | circle `R = 10`, `h = 10`, `α = 10°` | `Volume = πh(R² + Rr + r²)/3`, `r = R − d`; one `Cone` wall with `HalfAngle = α` and no seam edge; lateral area `π(R + r)√(h² + d²)`; axial centroid `h(R² + 2Rr + 3r²) / (4(R² + Rr + r²))` |
| F3 slot | two lines `ℓ = 30` apart joined by semicircles `R = 5` (G1 joins), `h = 8`, `α = 3°` | `Volume = h[2ℓ(R − d/2) + π(R² − Rd + d²/3)]`; two `Plane` and two `Cone` walls; the four G1 rulings are `Line3` |
| F4 L-section | `[0,20]²` minus `[10,20]²`, `h = 10`, `α = 5°` | `Volume = h(300 − 40d + 4d²/3)` (the sharp reflex miter: the notch square keeps its side); the reflex far vertex at `(10 − d, 10 − d, h)` |
| F5 flare | F1 with `α = −5°` | `Volume = h(a² + 2ad + 4d²/3)`; far vertices at `±(a/2 + d)` |
| F6 against | F1 with `Against` | the far cap at `z = −h` and `capStart`; volume as F1 |
| F7 ring | annulus `R = 10`, `r = 4`, `h = 10`, `α = 5°` | `Volume = πh[R² − r² − d(R + r)]`: the hole widens; the hole wall is a `Cone` opening toward the far end, its rims concave |
| F8 far origin | F1 and F3 on a sketch plane whose origin is `(10⁶ + 0.1, 0.1, 0)` and under a rotation placement; F3 drawn at `v = 10⁶` in the plane | every vertex's published bound covers the exact rational lift (the `apitest/vertex_frame_origin_test.go` pattern); F1's volume and area and F3's volume bounds cover their closed forms; the slot at `v = 10⁶` covers its volume, its centroid and each straight wall's denoted normal |
| F9 placed equivalence | F1 built, then `Transform` under a translation and a rotation | volume, area and centroid equal the unplaced readings within bounds; `Bounds` covers the placed box |
| F10 tangent span | F1 with `a = 2` | each far corner `a/2 − d`, a Sterbenz subtraction exact for the held `d`, lies within its bound of the exact corner the stated taper denotes |
| F11 stop extents | F1's section, with a planar stop at 30 mm and a 5 mm thick stop body | `ToFace` ends at +30 or −30 mm; `ThroughAll` ends at +35 or −35 mm; an offset in inches propagates its rounding; the far corner and `Volume = h(a² − 2ad + 4d²/3)` lie within their bounds |

**Refusal fixtures**, one per SD row that `Extrude` can reach, each asserting
`errors.Is` on the sentinel and that `Document.Bodies()` is unchanged: SD2
(`90°`), SD3 (a spline wall), SD4 (a semicircular bite meeting its lines at
right angles), SD5 (a radius-5 disk drafted `30°` over 10 mm, so `d ≥ R`), SD6 (F1 with `d ≥ a/2`,
and F7 with `d` closing the ring), SD7 (a thin rectangle whose short walls'
miters cross), SD8 (a plate whose off-centre hole widens across the outer
wall), SD12, SD13 (an angle of
`1e-14°` on a 1 mm sweep). SD15 is pinned in `internal/offset2d/sharp_test.go`
(§5). Beside them, a draft body's requests outside §10.4–§10.6 refuse it by
name (DD14), and `MirroredCopy` builds it (DD12).

**Circular shell fixtures** (`apitest/draft_circle_shell_test.go`): a
Sketch-produced radius-8 circle drafted at `5°` over 10 mm has its far cap
removed inward at `t=1`. Its one-shell result has a kept cap, top rim,
cavity cone and floor with their own roles. Its bounded volume covers
§10.6's frustum difference; its 0.1 mm `VerifyAll` mesh proves its boundary
and occupied volume. Placement preserves those roles and the volume.
Thickness that consumes the opening, a near-cap selection, outward sense,
negative taper and a holed section refuse.

**`Draft` fixtures** (`apitest/draft_test.go`):

| Fixture | Asserts |
|---|---|
| D1 equivalence | `Draft(ctx, b, Faces(Walls(b)), NeutralFace{b, Faces(FaceCreatedBy(CapStart(b)))}, 5°)` of F1's untapered box equals F1's body: same volume, area, centroid, vertex coordinates and face count, bit for bit |
| D2 other cap | the neutral face `CapEnd`: the far cap is `capStart`, at `z0`; volume as F1 |
| D3 negative | `−5°` widens: volume as F5 |
| D4 retire | the operand is retired; the result is live; the document holds one body |
| D5 refusals | SD17, SD18, SD19 (zero faces, two faces, a cylindrical neutral face), SD20 (`NeutralFrame`; a wall as the neutral face), SD21 (an empty selection), SD22 (the other cap selected), SD23 (a revolve; a drafted body drafted again) |

**PR 2 fixtures** (`apitest/draft_mesh_test.go`): F1–F7 tessellated at three
tolerances, each mesh closed and consistently wound, its boundary and volume
proofs published, every facet attributed to a face of the body, and
`Mesh.Bound` covering the chording it took; the same F1–F7 records meshed at
three tolerances through `draftrecord.Tessellate`, each held mesh embedded and
its exact volume within `volSymDiff` of the closed form, and each record's
DD17 radius at least every mesh vertex's L1 norm and at most twice the largest,
and F1's mesh holding its eight corners and two triangles on each of its six
faces (`internal/draftrecord/draft_fixtures_test.go`); `Union` of F1 with a straight
prism through it; STEP of F1 and of F2 faceted (`faceted decad solid`); `MassProperties` of F1 against the closed form.

**Near-tangent fixtures** (§9.1.1): F3's slot with its right centre moved
`2⁻⁴⁰` mm meshes with the volume proof at `VerifyAll`, and a `Cut` by a
square post covers `h[2ℓ(R − d/2) + π(R² − Rd + d²/3)] − 128` with
`ℓ = 30 + 2⁻⁴⁰` (`apitest/draft_mesh_test.go`); moved `5·2⁻³¹` mm, each join
turns past `sin θ = 2⁻³²` and the `Cut` refuses naming the corner and the limit. The
same records at `2⁻⁴⁰` and `5·2⁻³⁴` mm mesh at three tolerances with the held
volume within `volSymDiff` of that closed form; the exactly tangent slot
charges nothing, the `5·2⁻³⁴` slot charges a positive volume whose rate per
unit height covers Lemma 2's slice area at every eighth of the sweep, and its
`volSymDiff` exceeds the exact slot's (`internal/draftrecord/near_tangent_test.go`).
Lemma 1 is checked against a 256-bit reference of the true miter at amounts
of either sign, each refusal of §9.1.1 has a row, and the survival gate
admits a wedge whose unit wall survives, or whose `2⁻³⁰` mm wall outruns `ρ`,
and refuses it drafted until the far miter overruns the wall or with a
`2⁻³⁶` mm wall inside `ρ`; a riser that leaves a near-tangent quarter arc off
the bottom wall's axis refuses the same two ways at its start corner, and a
`2⁻⁴⁰` rad arc between two near-tangent walls refuses at a 1 mm draft, where
its true window reverses; the slot's right arc, past a half turn, is
admitted drafted to within `6.2e-10` mm of its radius and refused by the
half-turn branch at `6.0e-10` mm, where condition 6 still holds: condition 6
gives only `ρ_start + ρ_end < 2r_lo`, and `(π/2)·2 > 3`
(`internal/capband/near_tangent_test.go`); the two-sided slot of `2⁻⁴⁰` sews a
mesh that carries the proof (`internal/draftrecord/tessellate_test.go`).

**Two-arc near-tangent fixtures** (§9.1.1): the split disc, a radius-5
circle drawn as two half arcs whose lower centre sits `δ = 2⁻⁴⁸` mm (four
ulps of 5) above the upper one's, meshes with the volume proof at `VerifyAll`,
and a `Cut` by a square post covers `πh(25 − 5d + d²/3) − 2δh(5 − d/2) − 128`
(`apitest/draft_mesh_test.go`); moved `5·2⁻³¹` mm, each join turns past
`sin θ = 2⁻³²` and the `Cut` refuses naming the corner and the limit. The
same records at `2⁻⁴⁸` and `5·2⁻³⁴` mm mesh at three tolerances with the held
volume within `volSymDiff` of the closed form
`h[π(Q(5) + Q(R₂))/2 − 2δ(R₂ − d/2)]`, `Q(R) = R² − Rd + d²/3`,
`R₂ = √(25 + δ²)`; the exactly tangent disc charges nothing, the `5·2⁻³⁴`
disc charges a positive volume whose rate per unit height covers Lemma 2's
two-arc slice area (the twisted last cell and the curvilinear triangle) at
every eighth of the sweep, its `volSymDiff` exceeds the exact disc's, and it
is at least the 256-bit sum of the chord and motion terms and the charge;
on both the slot and the split disc at `5·2⁻³⁴`, the charge is at least the
256-bit sum of each term §9.1.1 names, read from `capband.NearTangentCharges`
and the loop meshes (`internal/draftrecord/near_tangent_arcs_test.go`). Lemma 1′ is checked
against a 256-bit reference of the two moved circles' meeting point in each
of its four cases at amounts of either sign; the limit, mixed-amount and
condition-6 refusals have rows, condition 6 for an inner counter-clockwise
circle moved inward, a clockwise circle moved outward and the arriving
circle; and a `2⁻⁴⁰` rad arc closed by a near-full arc that meets it near
tangent at both ends is admitted drafted `0.5` mm and refused by the
survival gate at `0.6` mm (`internal/capband/near_tangent_arcs_test.go`).

**PR 5 fixtures** (`apitest/draft_subset_test.go`): F1's box with its `+u`
wall drafted alone, `Volume = h·a(a − d/2)`, its area, centroid, far corners
and the kept walls' horizontal normals, and the same body placed; F4's L with
its notch wall `u = 10` drafted alone, `Volume = h(300 − 5d)`, the far reflex
corner `(10 − d, 10, h)`; F7's hole drafted alone, `Volume = πh(R² − r² − rd −
d²/3)`, the outer wall a `Cylinder` of radius `R`; F10's 2 mm square with one
wall drafted, each moved far corner within its bound; `Verify` on the subset
bodies (DD5, DD6); SD4 for one straight wall of F3's slot; the mixed-amount
corner rows in `internal/offset2d/sharp_test.go`; the box and ring subsets
meshed at three tolerances, closed, with the volume proof published, and each
mesh `Bound` no larger than its tapered extrude's; the one-wall, notch-wall and
hole-alone records meshed through `draftrecord.Tessellate`, each exact volume
within `volSymDiff` of its closed form (`internal/draftrecord/draft_fixtures_test.go`); the one-wall box under DD7, its drafted
wall listed along `−e` and cleared along `+e`, its kept walls undecided.

**PR 4 fixtures** (`apitest/draft_verify_test.go`): F1 under
`WithPullDirection(+e)` reads every wall clear and `Coverage` complete; under
`WithPullDirection(−e)` every wall is listed with `DiagUndercut`; F5 the
reverse; F3's `Cone` walls decided, not undecided; F7's hole wall clear along
`+e`, and a wider ring swept `Against` clear along its own `e`; F9's placed F1 and D1's `Draft` result read as F1 does along their own
sweep; a pull perpendicular to one wall's published normal leaves that wall
undecided.

**Fail-first legs**, each named in the test that owns it:

| Leg | Deleted | Goes red on |
|---|---|---|
| `farDelta` in the volume (`SweptVolumeAllow`) | the composition after the flux sum | F1, F6 and F8's placed F1 volumes |
| `dDelta` in the corner enclosures | the band's `dcDelta`, so the span is the point `d` | F10: every far corner reads `Exact` with a zero bound about `1.1e-16` from its exact point |
| the tangent enclosure | `RadTanSpan` replaced by the point `math.Tan` | `TestRadTanSpanEnclosesSeriesReference` against a 75-digit `big.Float` series reference |
| the closure sliver | `ClosureOf`'s result omitted | no fixture: F3's axis-aligned slot holds no sliver, and a slanted slot's slivers sit four orders below the far contour's volume term |
| the far cap's `sectionDisplacementArea` | omitted | F6's far cap area |
| the wall normal's denoted term (`capband.DenotedNormalAllow`) | omitted | F8's slot at `v = 10⁶`: a straight wall publishes a `2.2e-16` bound `3.3e-12` from its denoted normal |
| DD6's diameter arm | omitted | every F fixture under `Verify` (`DiagToleranceReferenceUnavailable`) |
| the mesh proof's chord term (`DraftBandHeightUpper`) | zeroed | F2, F3, F7 and the hole-alone subset: each mesh volume sits outside its `volSymDiff` |
| the mesh proof's vertex motion (`SweptVolumeAllow`) | omitted | F1, F4, F5, F6 and the one-wall and notch-wall subsets: `volSymDiff` misses the far corners' rounding |
| the far section in DD17's record radius | dropped from `MoverRecordRadius` | F5's flare: a far vertex lies past the radius |
| DD7's dispatch arm (`capblendsurvey.DraftUndercuts`) | removed from `runSurveys` | every PR 4 fixture: the survey reads the draft body as an unsupported payload |
| the wall reading's allowance (`capblendsurvey.PatchNormalRange`'s third result) | zeroed in `capblendsurvey.PatchUndercuts` | the tangent pull: the wall reads the sign of its own rounding and no `DiagUndecidedUndercut` is reported |
| a moved walk's span at a mixed corner | `capcontour.AmountsDisplacement` reads every span as its point amount | the subset F10: each moved far corner `1.06e-16` outside its bound |
| the mixed-amount refusal | `offset2d.SharpCornerJoin` reads a mixed circular corner as an equal one | the slot's SD4 fixture refuses as SD13 instead |
| a kept wall's setback in the mesh | the chord pass's cap radius, or the cap motion, reads the draft's `d` | the hole-alone subset: its `volSymDiff` grows past a tenth of the body's volume, so the proof bounds nothing |
| a kept wall's setback in the band's radius rounding | read against the draft's `d` | the hole-alone mesh `Bound` above F7's |
| the near-tangent arm of the admission | `capblend.OccupiedVolumeAdmission` passes no `SharpAmounts` | the near-tangent slot: no volume proof at any tolerance, and its `Cut` refuses |
| the near-tangent turn limit | condition 4 skipped | the `5·2⁻³¹` slot's `Cut` builds; the limit row of `capband` admits |
| the other near-tangent gates | condition 1, 5 or 6 skipped, or condition 6 read with every arc counter-clockwise | the matching `capband` refusal row admits, for condition 6 on a counter-clockwise arc moved inward and on a clockwise arc moved outward |
| the survival gate (`NearTangentWallRefusal`) | skipped in the admission | every wedge, riser and tiny-arc refusal row admits |
| the gate's circular branch | a circular wall read as surviving | the tiny-arc row admits |
| the gate's half-turn branch | an arc past a half turn read as surviving | the slot row at `6.0e-10` mm admits |
| the start corner's `ρ` in the gate | read as zero | the short riser admits |
| the circular neighbour's foot at a wall's start | read at the arc's start instead of its end | the overrun riser admits |
| Lemma 1's `ρ` and `ε` | each quartered | the 256-bit miter reference at an amount of 0.26 mm |
| the strip term `3ελ` | dropped from `NearTangentVolume` | the `5·2⁻³⁴` slot: the charge falls below the slice area from the first eighth of the sweep |
| the disc term `πρ²/4` | dropped | the `5·2⁻³⁴` slot's charge falls below the 256-bit sum of its terms (`TestDraftNearTangentChargeCoversEveryTerm`); no slice area pins it, since the segment it covers sits below `1e-33` mm² |
| `NearTangentVolume` in the proof sum | not appended | the `5·2⁻³⁴` slot: its `volSymDiff` falls below the 256-bit sum of the chord and motion terms and the charge; the `5·2⁻³⁴` split disc likewise |
| the two-arc arm of `NearTangentCorner` | a corner of two arcs read as no near-tangent join | the split disc: no volume proof at any tolerance, and its `2⁻⁴⁸` mesh at `VerifyAll` carries none |
| the turn limit at two arcs | condition 4 skipped | the `5·2⁻³¹` split disc's mesh carries the proof; the two-arc limit row admits |
| condition 6 for two arcs | skipped, read on the leaving circle only, or read with every arc counter-clockwise | the matching two-arc refusal row admits |
| the survival gate at two arcs | skipped in the admission | the `2⁻⁴⁰` rad arc drafted `0.6` mm admits |
| Lemma 1′'s `ρ` | quartered | the 256-bit reference in all four cases at an amount of `0.1575` mm |
| the twisted-cell term `ρλ` | dropped from `NearTangentVolume` | the `5·2⁻³⁴` split disc: the charge falls below the two-arc slice area from the first eighth of the sweep |
| the curvilinear-triangle term `2.08ρ²` | dropped | the `5·2⁻³⁴` split disc's charge falls about `2e-20` below the 256-bit sum of its terms (`TestDraftNearTangentChargeCoversEveryTerm`); no slice area pins it, since the triangle sits under `1.3e-21` mm² per corner |

### 12. Do not do this

- **Build the taper as a `Loft` between `P` and `Q`.** A loft walls a circular
  pair with chorded flat triangles and a section displacement, mints
  `side(i, j, k)` triangle roles, refuses every modify op and reads
  `Suspect` on every survey. A drafted box must be a first-class body with
  `Plane` walls and `side(i, j)` roles that DX7 can read.
- **Offset reflex corners with the shell's arc.** That is a wall-thickness
  feature (modify §8); a draft keeps a reflex corner sharp (§2). F4 fails
  under the arc rule.
- **Integrate `A(z)` as a polynomial for the production volume.** The
  polynomial's coefficients are the far section's own readings, which carry
  the same displacement; the patch flux already exists, is proven, and is
  what the tessellation proof is reconciled against. The polynomial is the
  test's cross-check (§8).
- **Admit a near-tangent corner as G1 on a small cross product and
  then publish the wall as exact.** The dead-zone rule decides which closed
  form is built; the held foot's hull and the closure sliver charge what it
  cost. Deleting either is the first fail-first leg.
- **Hold a non-G1 circular corner as a ruled `Cone` with a bounded skew.**
  That is the cap-loop chamfer's stand-in (modify-reach §8.3), with its
  departure terms, its DX8 refusal and its conic ruling bound. It is not
  built (§14).
- **Build `Symmetric` or `TwoSided` by offsetting both ends from the sketch
  plane and gluing two bands at level 0.** The seam at level 0 is an edge
  between two walls of different lean and a `stackedPrismPayload` holds no
  tapered slab. `TwoSidedPayload` holds the two draft slabs (§14).
- **Ask `sketch.CreateOffset` for `Q`.** Lines only, a float solve, no
  certificate, and no collapse or crossing check (§3).
- **Let `Draft` take a `nil` selector meaning "every wall".** A nil selector
  with a default is the implicit-target guess core §4 rejects; `Walls(b)`
  spells the intent.
- **Decide `Draft`'s sweep direction from a pull vector argument.** The
  neutral plane and the sign of the angle determine the body; a second
  direction argument can only agree or conflict.
- **Read `tan α` from `math.Tan` and publish a zero bound on an axis-aligned
  box.** The enclosure has width; the box is `Approximate` (§8).

### 13. Decided questions

- **One payload for both entry points**: `draftPayload`, over the operand's
  or the sketch's record. `Draft` is `Extrude` with a taper over an existing
  prism's section (§10.1), so the two share §7 entirely.
- **Build order**: the tapered extrude comes first (it has no selection or neutral
  plane to resolve), then tessellation, then `Draft` (§14).
- **Sign**: positive narrows away from the sketch plane or neutral plane
  (§2). `Verify`'s pull survey then reads a positive draft as clear along
  the sweep.
- **Sharp corners**: the miter at every line–line corner, convex or reflex
  (§2, §12).
- **Circular corners**: G1 joins only in this design; the ruled stand-in for
  a mitered circular corner is deferred (SD4, §14).
- **Near-tangent joins**: a line–arc or arc–arc join the build reads as G1
  and the record does not make exactly tangent meshes with the volume charge
  of §9.1.1 while `sin θ ≤ 2⁻³²`; a larger turn keeps the export-only mesh.
- **Free-form walks**: refused (SD3); the chorded alternative is a loft and
  is rejected (§12).
- **Exactness**: every draft measurement is `Approximate` (§8).
- **Extents**: `Distance`, `ToFace`, `ThroughAll`, `Symmetric` and `TwoSided`.
  A crossing extent uses two slabs with one sewn sketch rim (§7, §14).
- **Neutral plane**: a cap of the operand, through a `NeutralFace`; a
  `NeutralFrame` and an interior neutral level are deferred (SD20, §14).
- **Selection vocabulary**: `Walls(b)` is a `FacePredicate` beside
  `CapStart`/`CapEnd`; there is no nil-selector form.
- **Surveys**: undercut is decided off the patches' normal model (DD7); wall
  and concave-radius surveys are not built (DD8, §14).
- **Hand-offs**: none (§3).

### 14. PR split

All seven increments are built. The numbers name the increments that §11's tables cite. Documentation for every
refusal an increment lifts lives in this document's §11, `doc.go`'s support map, `agent-docs/missing-features.md`,
the function doc comments, and the layout rows of the root files.

| Increment | Provides | Files | Pinned by | Builds on |
|---|---|---|---|---|
| **1** | `Extrude` + `WithTaper` over Table RD1: `offset2d.BuildSharpLoop`; `draftPayload` with `Transform`/`Placed`; the build (§7); the measurement (§8 over `internal/capband`); `d` and its span (§8.1) with a certified tangent in `internal/proofbound`; `extrude.go` dispatches a nonzero taper to the draft build and refuses SD11/SD12; DD6's gate arm; DD12; the modify refusal messages naming the class | `internal/offset2d/sharp.go`, `internal/proofbound/interval_trig.go` (tangent), `internal/draftrecord/` (`payload.go`, `build.go`, `measure.go`), `draft.go`, `extrude.go`, `internal/clearance/pairkernel/gate.go`, `internal/gatediameter/body_gate.go`, `fillet.go`/`chamfer.go`/`shell.go` (messages), `apitest/extrude_taper_test.go`, `internal/draftrecord/build_test.go`, `internal/offset2d/sharp_test.go`, `examples/extrude_taper_example_test.go` | F1–F10 and the SD refusals of §11; the fail-first legs | — |
| **2** | tessellation and the mesh volume proof (DD1), which serves DD2, DD3, DD4, DD10, DD11, DD13, DD17 | `internal/draftrecord/tessellate.go`, `internal/tessdispatch/dispatch.go` (dispatch), `internal/capblend/admit.go` (§9.1's three differences), `internal/draftrecord/payload.go`/`build.go` (`Patches`, `BandDelta`), `internal/booloperand/volume_proof.go` (the admission arm), `internal/massdispatch/dispatch.go` (the mesh path needs no arm), `internal/motionengine/recordradius.go`, `apitest/draft_mesh_test.go`, `internal/draftrecord/tessellate_test.go` | the mesh fixtures | 1 |
| **3** | `Draft` over RD2: `NeutralPlane`, `NeutralFace`, `NeutralFrame` (refusing), `DraftOption`, `Walls(b)`; §10.1's resolution; SD17–SD23; core §8's pointer to this document and core §12's signed-displacement list | `draft.go`, `selector.go` (`Walls`), `internal/selectorquery/predicate.go`, `agent-docs/api-design.md` §8/§12, `apitest/draft_test.go`, `examples/draft_example_test.go` | D1–D5 | 1 |
| **4** | DD7: the undercut survey over draft bodies | `internal/capblendsurvey/draft.go`, `internal/capblendsurvey/undercut.go`, `internal/verifyrun/survey.go` (dispatch), `apitest/draft_verify_test.go` | the survey fixtures; the `−e` pull lists every wall | 2 |
| **5** | RD3: the subset draft (§10.2): per-walk amounts in `BuildSharpLoop`, the mixed-corner rule, SD21 narrowed | `internal/offset2d/sharp.go`, `internal/capcontour/displacement.go`, `internal/capband/`, `draft.go`, `internal/draftrecord/build.go`, `internal/draftrecord/payload.go`, `internal/draftrecord/tessellate.go` (the mesh's per-walk setbacks), `apitest/draft_subset_test.go` | one wall of F1's box drafted: `A(z) = a(a − z·tan α)`, so `Volume = h·a(a − d/2)`; the L with one notch wall drafted; a hole drafted alone (F7's cone with vertical outer walls) | 3 |
| **6** | RD1 with a one-sided `ToFace` or `ThroughAll`: `resolveLinearExtent` resolves the stop, then the existing single-slab draft builds with the stop's axial bound | `draft.go`, `apitest/extrude_taper_test.go` | the far-cap vertex and volume match the selected stop level within their bounds | 5 |
| **7** | RD1 with `Symmetric` or `TwoSided`: two slabs sewn, with bounded readings and meshes combined | `internal/draftrecord/two_sided.go`, `internal/draftrecord/tessellate.go`, `draft.go`, `internal/compositesweep/body_audit.go` | public volume, manifold, curved mesh and placement cases | 6 |

What still refuses:

- SD3 (free-form walk), SD4 (the mitered circular corner), SD12 (taper with a surface result), SD20 (`NeutralFrame`
  and the interior neutral level).
- The wall and concave-radius surveys (DD8) and the undercut survey of a two-sided body read `Suspect`.
- Clearance carriers (DD9).
- Shell, fillet and chamfer of a draft body outside §10.4–§10.6 (DD14).
- Booleans, interference and mass of a body the band admission refuses (§9.1).

The unscheduled reach, in the order a later design should take it: the interior neutral level, the mitered circular
corner as modify-reach §8.3's ruled stand-in, the wall and concave-radius surveys (DD8), and the clearance carriers
(DD9).

## Part T — Tangent side notch

A blind `Cut` whose tool meets the target's outer wall only at recorded vertices, tangentially, and the `Fillet` of
the concave arc it leaves on the notch floor. The gallery reports that Fusion 360 builds both (not checked here; §6
gate G0). Status: approved (option O1) and built. T1 is the `sketch` fix (sketch `9216dee`, pinned in `go.mod`). T2 is
general-boolean §3 A9 on A7's build: the cut builds as a BRep. T3 and T4 build the dart-root fillet as
boss-root-overflow §5.5's C3 construction. §2 and §3 record why both refused before T1 to T4.

Navigation (not authority): the failing case → §1; why the cut refuses → §2; why the fillet refuses → §3; the target
result → §4; soundness → §5; increments and size → §6; options → §7.

### 1. Problem

The decad-gallery swisslever lever (`models/swisslever/lever.go`, `buildLever`) is one extrude, one cut and one fillet.
Coordinates below are the lever sketch's, origin at the pallet arbor. The `z` intervals and the fillet radius are the
gallery model's; the fork's plan coordinates are the constants of `leverForkSketch`
(`apitest/side_notch_cut_test.go`), and `r_c`, `G`'s radius and the notch walls' end `(±1, 44)` follow from them.

| Body | Profile | Interval |
|---|---|---|
| target | lever outline: two slot holes (oblique lines), arm sides, three sketch fillets, at the fork end a corner round `K` (radius `r_c` = 0.66616, centre `(±2.33384, k_y)`), a gap round `G` (radius 0.53384, centre `(±1.13384, k_y)`) and the dart (sides `x = ±0.6`, tip arc) | `z` 3.30 to 9.80 |
| tool | notch: semicircle `B` (radius `r_n` = 1.66767, centre `(0, k_y)`, under the centre), two oblique walls from `(±r_n, k_y)` to `(±1, 44)`, top line | `z` 5.80 to 10.80 |
| fillet | the one concave circular edge, `B` on the floor `z` = 5.80, radius 0.6 | |

`k_y` = 38.874650636459869. `a = (r_n, k_y)` is the inner end of `K`, the outer end of `G` and the end of `B`. All
three arcs have their centres on `y = k_y`, so all three are vertical at `a`. `K` touches `B` from outside, `G` from
inside. The mirror vertex `a'` is the same.

The result is a valid solid. Above 5.80 the outline runs along `K` into `B` with a smooth (G1) joint at `a` and `a'`.
The floor at 5.80 is the dart top plus two crescents between `G` and `B`, each narrowing to a zero-width point at `a`.

Current behaviour of the cases the table names, with the tests that assert it:

| Variant | Behaviour |
|---|---|
| exact lever (the fork with the notch, no, one or two oblique slot holes) | `Cut` builds a `brepPayload` through general-boolean §3 A9, volume within its bound of the closed form, one `Reflex` edge (`TestSideNotchCutTangentFork`); the mesh path alone refuses the contact at `a` and `a'` with `meshbool.RefuseUndecidableProximity`. The notch floor's arc then takes a 0.5 fillet (`TestBossRootTangentFork`). No test builds the complete lever. |
| notch radius widened | `Cut` takes the mesh path or declines with `ErrUnsupported` (`TestSideNotchCutDeclinesObliqueCut`); no test fillets its floor arc. |
| exact lever as two extrudes (band S with the dart, band F with the notched outline) and `Union` | not asserted by a test |
| both of the above with the slot holes left out | the no-slot case of the first row is asserted (`TestSideNotchCutTangentFork`); the widened-notch case is not |
| notch radius widened, fillet skipped | not asserted by a test |

A chamfer wider than the corner round `K`'s radius cannot run along it, in decad or in Fusion, so a gallery edge finish
sized that wide refuses `ErrDegenerate`. The same holds for the exact lever.

### 2. The cut refusal

Three facts together give it.

#### 2.1 No analytic route admits a side notch

- prism-boolean §4.4 sends a `Cut` "whose tool ends inside the target and whose boundary crosses the target's (a side
  notch)" to the mesh path.
- general-boolean §3 A7 (`prismboolean.TryCrossingBlindCut`, the `stackedunion` BRep build) is the nearest route. It
  admits a target with exactly one hole, loops of axis-aligned lines, circles and arcs only, a pocket inside the outer
  wall, and a cut slab of exactly one outer and one hole (`ubBuild.cutSlabRegion`). The lever has two holes, oblique
  lines, and a tool that reaches the outer wall.

#### 2.2 The mesh path refuses the contact at `a`

The gate's refused face pairs are the target's corner round `K` (a `Cylinder`) against the tool's oblique wall (a
`Plane`), at both `a` and `a'`. The wall starts at `a`, where `K` is vertical, and leaves at atan(0.66767 / 5.12535) =
7.4° from vertical into the void (the wall runs from `(r_n, k_y)` to `(1, 44)`), so the true faces touch along the
generator through `a` and nowhere else. Their facets come within the chord slack without crossing deeper than it,
which is exactly the case the gate refuses (evaluator §9). The cylinder pairs `B`–`K` and `B`–`G` touch along the same line.

The shifted retry (gap-tolerance §5.2) does not open it:

- a translation must move the tool along `−x` to open `a` and along `+x` to open `a'`;
- growth (§5.2 item 5) runs only for coplanar refusals with zero slack, and a gate contact carries slack;
- growth would also notch both horns by the growth, a shape Fusion does not make and one that breaks the fillet (§3).

#### 2.3 `sketch` returns a wrong arrangement at `a`

A record route (§6) asks `sketch` for the cells of the band F scene. Before T1, the scene of the lever region and the
notch returned too few cells, and `prismcells.ClassifyRegions` refused: one cell's outer loop walks the notch arc in
both directions and the gap arcs and dart edges against their material side.

A `sketch`-only scene reproduces it: the fork end (arm sides, `K`, `G`, the dart, the closing line) and the notch
polygon, sharing `a` and `a'` exactly. The correct arrangement has 3 cells, and
`TestSideNotchCutTangentFork` pins two of their areas: 50.273480 mm² for the fork less the notch and 7.969198 mm² for
the overlap. Before T1 the arrangement was one cell holding the union, with some edges walked twice, reported `Valid`.
That is a `sketch` soundness defect, not a refusal. decad caught it only because `ClassifyRegions` met a membership
conflict. The defects in `geom/arrange.go` were:

- The consistency gate exempts an internal tangency.
- The shared-endpoint branch breaks before the vertex is registered for exact port order (`exactPortVerts`), so `a` is
  never a certified tangency vertex.
- `prepareExactPorts`' second door skips a same-ray tie even when the chord angles tie bit for bit.
  `G` and `B` leave `a` on one tangent ray, so curvature, which would order them, is never asked.
- Both arcs are densified with one angular step per turn, so their first chords leave `a` on one ray to rounding.
  `G`'s first chord is shorter than `B`'s and ends on it, so the chorded curves overlap.

T1 is the `sketch` fix: register a shared endpoint of two curved fragments on one tangent ray for exact port
order (tangent, then signed curvature); keep their chords from overlapping there; and flag the scene `Degenerate`
wherever the ports cannot be ordered (equal curvature, or a line and an arc on one ray) instead of returning a cell.

### 3. The fillet refusal

- Widened notch: `B`'s end `(r_n + 0.02, k_y)` lies inside the horn, and the oblique wall crosses `K` about 0.1 mm
  above `k_y` (the arcs `B` and `K` themselves do not meet: their circles cross at about `y = k_y ± 0.14`, each point on
  one arc only). `B` therefore ends at the oblique wall, as in the exact lever. The arc blend ends each row on the plane at
  each arc end (retained-edge-blend R4). That plane contains the oblique wall, which leaves `B`'s end toward `(1, 44)`, so it lies
  `(r_n + 0.02)·5.12535/√(5.12535² + 0.68767²)` = 1.6727 mm from `B`'s axis. The floor row's circle has radius
  `r_n + 0.02 − 0.6` = 1.0877 mm and never reaches it, so `arcRim.section` refuses. The shape is boss-root-overflow X1
  (an arc ending on a wall that rises from the floor). It says nothing about the exact case.
- Exact lever: never reached before T2, because the cut refused first. By construction it is boss-root-overflow's rule CO on an
  inner wall. The fillet ring lies inside `B` (torus major radius `r_n − ρ` = 1.06767). Its floor row crosses `G` at
  27.86° from each end of `B` (the circle of radius `r_n − ρ` about `(0, k_y)` meets `G`'s circle at `(±0.94392,
  k_y + 0.49894)`), where the crescent becomes narrower than `ρ` (a transversal crossing of a cliff
  cylinder, C2's construction on a floor row rather than at an arc end). From there to `a` the ring is trimmed by `G`
  extended upward, and at `a` the cap on `G` narrows to the point where `G` touches `B`. That is configuration C3, the
  class boss-root-overflow rates least likely to be built by Fusion, built by its §5.5 (T4).
- C3 here differs from the boss's: `B` does not continue below the floor. Below `a` lies `G`; beside it `K` runs the
  full height, tangent to both `B` and `G` along the generator through `a`. The cap on `G` therefore meets `K` along
  that generator from 5.80 to 6.40.
- The selected edge `B` meets the convex floor edge on `G` at `a`, tangentially. The tangent-junction refusal
  (missing-features "Correct refusals", SB7/SM2) concerns a blendable continuation. A convex continuation of a concave
  edge is not one, and an implementation must check that rule does not fire.

### 4. Target result

- Cut: the regularized `target − tool`. Its band S slab is the target region. Its band F slab is the target region
  less the notch: one outer loop, both slot holes. The floor is the band S region less the band F region (the dart top
  and the two crescents). No wall is split: `B` meets the target loop only at its recorded vertices `a` and `a'`.
- Faces at `a`: `K` over the full height; `G` from 3.30 to 5.80; `B` from 5.80 to 9.80; the floor with a cusp at `a`.
  `K`–`G` and `K`–`B` are G1 edges along the generator through `a`.
- Fillet: rule CO (boss-root-overflow §2.2) with `A` the floor, `B` the notch wall, `D` the annular sector
  `r_n − ρ ≤ r ≤ r_n` over the root arc's angular range (the semicircle; the full ring would also cover the dart top
  beyond `B`'s ends, where no wall stands), and `G` the cliff faces. The torus is inside `B`.

### 5. Soundness

CLAUDE.md governs every admission: a decad-side residual may only falsify.

| Decision | Proof | Never |
|---|---|---|
| the band F cells | `sketch`'s arrangement, after §2.3's fix, with `TExact` on every `Partial` edge and `ClassifyRegions` on recorded edge orientation | a point-in-region test in decad |
| `B` meets the target loop only at `a` and `a'` | `sketch`'s intersection query for every pair of a tool entity and a target entity with a curved member (the query `prismcells.ProveGroupApart` reads, prism-boolean §4.2), every reported point being the shared recorded endpoint (equal floats in both records; probe: `(1.6676711944252536, 38.874650636459869)` for `K`, `G` and `B`). Whole edges alone prove nothing: the arrangement neither cuts nor flags a clean tangency | a distance below a tolerance |
| `B`–`G` tangency at `a` for C3 | not the floor loop's joint `B → G`: that joint is a cusp (B arrives along `+y`, `G` leaves along `−y`), and `capband.JoinIsG1` and `NearTangentCorner` both refuse `dot ≤ 0`. Prove it through `K` at the one exact vertex: `JoinIsG1(K → G)` on the target outline (or `NearTangentCorner`'s near-tangent admission with its turn charged, boss-root-overflow §5.5.2) and `JoinIsG1(K → B)` on the result's band F outline; or add a reject-only cusp predicate (cross product zero, dot product negative) to `capband` | an angle residual below a tolerance |
| `K` meets the fillet prism `P` only along the generator through `a`, from 5.80 to 5.80 + ρ | boss-root-overflow A7 refuses any other face meeting `P`; T4 changes that gate for a side face G1 to both `B` and `G` at the arc end, proved by the row above | a clearance read from held facets |
| C2 crossing of the floor row with `G` | its charged sine above `minSine` (retained-edge-blend §4.4) | a sine without its charge |

The cut's result is exact where the loops are exact; arcs make its volume `Approximate` with a proven bound, as A7's
do. The mesh path's gate is unchanged: T2's route runs after the mesh path refuses, and every pair outside that route's
gate keeps the mesh path's refusal.

### 6. Increments

| # | Increment | Unlocks |
|---|---|---|
| T1 (built) | `sketch`: exact port order at a shared endpoint of two curved fragments on one tangent ray, no chord overlap there, and `Degenerate` where the ports cannot be ordered (§2.3); the densification change moves the golden scenes that depend on it | correct cells for tangent arcs sharing an endpoint; no silently wrong `Valid` cell |
| T2 (built) | general-boolean §3 A9: A7's BRep build, tried after the mesh path refuses, with A7's scope gates lifted (any number of target holes, oblique lines only where no scene cuts them, the tool across or touching the outer wall, one result cell with no pinch). A7's engine already splits walls at an interface on the outer ring, so no new geometry was needed | the lever body; side notches of line, circle and arc outlines |
| T3 (built) | brep-modify §2.3: the retained edge blend on the faceted view of a stacked or slab-cut BRep (`SlabCut`), after the receiver routes refuse; a holed union's circular walls carry its charge on their tags (boss-root-overflow P5). Replaces boss-root-overflow P3 | fillets on T2's results reach the retained edge blend |
| T4 (built) | C3 tangent ends (boss-root-overflow P6, §5.5) generalized to an inner wall and to §3's end topology (`G` below, `K` beside), the floor-row cliff crossing on an inner wall, and §5's A7 gate change | the dart-root fillet; the balance-wheel boss at Ø14 |
| T5 (no code) | the lever's later steps on T2's result: the D bore `Cut` (an A7 BRep admits no later analytic Boolean, general-boolean §3 A7, so it takes the mesh path) and the edge-finish chamfers over the `K`–`B` G1 joints | the finished lever: on T4's result the gallery lever's bore `Cut` and edge chamfers build |

The lever with its fillet needs T1 to T4; T5 needs no code.

Gate G0 of boss-root-overflow §2.4 applies to T4: record in Fusion the faces the lever fillet produces. A Fusion result
that keeps the edge or fails moves T4 to a refusal.

### 7. Options

O1 was chosen: build T1 to T5, so the lever builds as Fusion builds it. T4's Fusion outcome stays unconfirmed until G0
runs. The rejected options:

- O2: build T1, T2 and T5 and leave the dart root sharp. It loses the lever's 0.6 mm root round in decad and in the port.
- O3: build T4 as one generalized C3 increment with boss-root P6, so one construction serves both gallery models.
- O4: change the fork geometry so no tool face touches the target. `r_n + 0.02` cuts today, and the bore after it,
  but the fillet then refuses (X1, §3), and no tested geometry builds both.
- O5: build no lever test. No test in this repository builds the complete lever, and none is planned.

## Part X — Perpendicular cylinder bore-mouth chamfer

This route admits one equal-setback chamfer on the positive-X mouth of a
cross-drilled circular cylinder. The public producer is a `Cut` of two full
Sketch-circle extrusions. The `Cut` still uses the ordinary mesh Boolean and
keeps its existing contact proof. It adds a one-use source record only when
both prisms have the exact cardinal pose below. `Chamfer` rebuilds from those
source circles; its selected faceted edge identifies the mouth and supplies
no geometric samples to the new surface.

### Admission

The target cylinder is `x²+y²≤R², 0≤z≤H`, with an XY circle centered at the
origin. The tool removes `y²+(z-c)²≤r²`, with a YZ circle centered at `(0,c)`
and an X sweep extending more than `R+1 mm` either side. Both sections are
whole CCW circles with zero section and sweep-level displacement. The
authenticated circular source edges must carry finite curve bounds below
one-thousandth of their radii. Placement clears the source record.

The bounded class has `2≤R≤20 mm`, `0.25≤r≤R/4`, `2≤H≤40 mm`, and
`0<c-r<c+r<H`. One selected closed faceted edge must join the target and
tool wall origins at the positive-X mouth. Its held start lies at `x>R/2`.
The equal setback `d` is positive, at most `r/2`, and leaves both target caps
clear: `c-r-d>0` and `c+r+d<H`. The negative mouth, asymmetric setback,
changed pose, and other source or selection classes refuse.

### Source surface and band

For `θ` around the bore, set `C=cos θ`, `S=sin θ`,
`X=sqrt(R²-r²C²)`, and `Q=sqrt(R²-r²C⁴)`. The source intersection on the
positive side is `(X,rC,c+rS)`; the negative side uses `−X`.

Unroll the outer cylinder into `(R atan2(y,x),z)` and the bore into
`(x,rθ)`. Moving `d` along each developed surface's unit normal to the
intersection gives these feet:

- Outer foot: rotate `(X,rC)` around the Z axis by
  `δo=dCX/(RQ)`, then set `z=c+rS+dRS/Q`.
- Bore foot: set `x=X(1-d/Q)`, rotate `(rC,rS)` about the bore axis by
  `δb=dCS/Q`, then restore the Z offset `c`.

The chamfer face is the straight ruled band between corresponding feet.
Each foot is exactly `d` from the original seam on its own developed
cylinder. The complete remaining outer and bore walls join those feet; the
negative mouth and target caps stay on their source surfaces.

The outer foot's cylindrical angle increases with `C`. Its derivative has
the positive terms `r/X` and
`(d/R)[(R²−2r²C²)/(XQ)+2r²C⁴X/Q³]`. The class bound `r≤R/4` makes each
denominator positive and `R²−2r²C²>0`. The positive opening therefore stays
a single graph over the target cylinder's angle.

### Boundary and occupied volume

The positive mouth uses 4,096 whole-turn cells. Each cell evaluates the
source feet over outward rational sine, cosine, square-root and normal-angle
intervals. Each derivative product, quotient, square root and trig step rounds
its derivative enclosure outward onto a dyadic with 128 significant bits, which
only widens it; the coordinate enclosures are never rounded. It requires the
outer foot outside the bore, the bore foot inside the target, and an outward support dot product at the bore foot.
The first, mixed and last polar-cross-product coefficients are positive,
which makes every band cross-section advance around the bore. The two
Jacobian endpoint coefficients have the same strict sign, excluding a band
fold. Failed interval signs refuse the operation.

Only the 1,025 cells of the first quarter turn, `0≤θ≤π/2`, are proved;
every other cell is the exact mirror image of one of them. The admitted
pose makes both walls, `x²+y²=R²` and `y²+(z-c)²=r²`, exactly symmetric
under `y→−y` and `z−c→−(z−c)`, and the feet depend on `θ` only through
`C` and `S`. With `θ_j=2πj/4096`, cell `j` has a base cell `k` with
`θ_j=θ_k`, `π−θ_k`, `π+θ_k` or `2π−θ_k`, an identity of exact turns, so
`θ→π−θ` (`C→−C`) negates every Y coordinate, `θ→−θ` (`S→−S`) negates
every Z−c coordinate, and X keeps its value. The feet of cell `j` over its
whole angular range are the feet of cell `k` with those coordinates negated,
so negating base `k`'s enclosures, an exact rational step, encloses cell
`j`. Interval width and absolute bounds do not change under negation. Each
of the eight interval signs pairs coordinates or derivatives of equal
parity, since a derivative gains one more sign under each map, so it takes
the same value at both angles. Before any cell is proved, the record is
rebuilt from its two source bodies by admission's exact comparisons and must
equal itself; a pose that is symmetric only to rounding refuses. The
normal-angle sine and cosine enclosures widen from the lower end of their
angle interval, so the enclosures computed directly in the other quarters are
valid but not the mirror images of the quarter-turn ones; the mirrored ones
are the ones used.

The held mesh has the two target caps, outer wall with two openings, bore
wall, and one chamfer band. A full exact facet-contact and closed-shell
audit checks it. The audit proposes each cap fan as a projected XY patch. It
omits only the within-cap pairs of a cap whose patch proof holds, and
classifies every other pair exactly, through the row-batched route. A cap
whose proof fails keeps all its pairs. The row preflight charges the scan
ceiling differently from the pairwise route, so a record near that ceiling
can refuse on one route and not the other; every such refusal is still a refusal. At each angular cell, paired source and held wall
points use the same height or axial fraction. The band uses the same transverse
fraction. Three cell coordinate diagonals enclose the source-to-facet
departure, with separate charges for held sample rounding, source-circle
displacement, and outer/bore cylinder chord sagitta. The held sample charge
compares every cell's own held outer foot, bore foot and negative-mouth
vertex with that cell's enclosures, mirrored or not, reading Z as an exact
shift of the Z−c enclosure by `c`. Every outer-ring vertex
is compared with an outward trigonometric interval at its recorded angle;
this charges radius and angle-conversion rounding. Exact rational differences
between consecutive recorded angles, including the full-turn wrap, bound
each outer chord. The bore sagitta uses
the angular change *across* the bore wall as well as the station gap.
The resulting bound must be below 0.1 mm.

The source and held solids can differ freely within a box around the one
modified positive mouth. The box width is bounded by
`R−sqrt(R²−r²)+2d`, and its Y and Z spans by `2(r+d)`, enlarged for source
displacement. Each foot lies within Euclidean distance `d` of its seam
point because its developed-cylinder travel has length `d`; this contains
the entire band in the box. Outside that box, differences fit the complete outer
and bore radial chord annuli. Their volumes and the box volume give the
occupied-volume allowance. The held mesh's exact rational volume must
exceed this allowance. A source-area upper bound plus the held mesh area
bounds the surface-area difference. The held diameter plus boundary
departure enters the usual volume-based centroid allowance.

### Shared cell proof

`CellExactProof` reads only the radii `R`, `r` and the setback `d` as plain
numbers, with the bore axis at `z − c = 0`; `Feet` reads the record through
`FeetAt`, which takes the same numbers and `c`. The retained saddle chamfer
(Part E §4.9) maps a shaft and a
perpendicular bore read from exact retained tags onto this pose and proves its
tool's band with the same cell tests at its own station count
(`crossbore.ProveToolBases`, `crossbore.ProveToolBandWith`); its admission is that document's, not this one's.

### Public case

`apitest/cross_cylinder_bore_chamfer_test.go` builds the cut, chamfers the positive mouth, and checks one
`boreMouthChamfer` face. `Tessellate(0.1 mm, VerifyAll)` reports boundary and volume verified, the boundary bound is
below 0.1 mm, and the volume reading's lower bound is positive.

### Continuing a straight chamfer to a cylindrical support

A straight cap edge can remain between a stopped chamfer and a cylindrical
support tangent to the cap's outer profile. Chamfer at the previous setback
continues the bevel across the stop and tapers into the cylinder. It preserves
the cylinder's diameter and retains the seating face outside the extension.

The bounded class has a coordinate-aligned cap, a perpendicular planar side,
and a cylindrical support normal to the cap. The side and the support trim
must each be single-valued over a coordinate along the run. One end of the
selected edge has the existing chamfer stop. The other meets the convex
rounded end of the same native extruded profile. Its cylinder must lie inside
or coincide with the supporting cylinder. Both mirror senses are admissible.

Bound every candidate face's reach in the affected window before using its
retained surface. Bind native cap, side and support to their original
extrusions and authenticate the previous cutter's terminating support. A
third surface entering the window, an ambiguous stop, a missing native
record, or an uncertified material orientation declines the route.

Within the window below the cap the boss supplies the only boundary. Above
it the original cap can add material on crossing upward. An oriented true
cap point between the boss and side witnesses the extruded stock there.
A held point supplies this witness only if its full source displacement ball
stays within that connected region. Determine clearance from the actual
native records including placement and construction error.

Clip the held cap and side and remove the old stop wall. Extend the existing
cylinder trim to the new bevel plane at each shared edge. Triangulate the
bevel and its end from the resulting boundary. Charge the radial departure,
the stop's source displacement and coordinate rounding. Verify the closed
oriented shell and every crossing pair involving a changed facet before
publication. The result carries boundary, area and occupied-volume bounds.

The implemented chart uses world X along the run and a cap normal to Y.
A direct ruled Loft supplies the stopped cut. Its two original bounded
line-and-arc profiles, frames and alignment must match their construction
records. A cut-tool face must trace through exactly one subtraction.
Every stock face must trace through none. Moved tools decline this route.
The held edit must fit the same finite window used for the source proof.

The public regression joins a transverse web and a circular pad to an
extruded flange. It checks the original side and finished bevel heights at
the same transverse station, and verifies the closed shell and volume.
