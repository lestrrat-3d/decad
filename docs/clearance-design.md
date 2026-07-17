# Clearance Design

How the evaluator proves the pair partition beyond box separation, and how it
measures `Clearance.Gap`: the two claims a row asserts (§1), why boundaries
carry the whole problem and what they alone cannot decide (§2), the candidate
enumeration that makes a minimum provable (§3), the face-pair distance table
over the shipped surface set (§4), the interval machinery — witnesses,
conservative lower bounds, certified brackets, refinement (§5), touching
pairs (§6), how the results feed the report (§7), and the increment plan (§8).
Companion to `docs/verification-design.md`, which owns what a `Clearance` IS
and how its `Gap` is judged (references "verification §N"), and
`docs/evaluator-design.md`, which owns the evaluator's staging and the shipped
box-disjointness proofs ("evaluator §N") — this is the increment-3 design its
§11 row 3 names. `docs/payload-verification-design.md` owns the cup/faceted
boundary adapters and the proof-bound expansion around this analytic kernel.
`docs/interference-design.md` owns how a proven overlap is turned into a
non-mutating, bounded `Interference` row. Nothing here changes those
contracts.

## 1. The proof obligation

A `Clearance` row asserts two claims, and the kernel must prove both
(verification §1): **the pair's interiors are disjoint**, and **`Gap` is the
minimum distance between them** — the true minimum over everything the two
bodies are, never the smallest distance a search happened to visit. A search
that samples proves an upper bound; only an enumeration that provably covers
every place the minimum can live (§3), with a lower bound that provably holds
everywhere it does not look (§5), proves a minimum.

**The answer is an interval, and exactness is the interval's, not the winning
candidate's.** Every candidate distance the kernel computes is carried as a
proven interval `[lo, hi]` — degenerate (`[v, v]`) when the candidate is
closed-form, a certified bracket otherwise (§4/§5). The gap over a candidate
set is the minimum of intervals: `Gap ∈ [min lo_i, min hi_i]`. The row reports
`Value = (lo + hi)/2`, `Bound = (hi − lo)/2`, and `Exact` exactly when the
interval is a point — which requires not just a closed-form winner but every
bracketed rival proven to sit at or above it (`lo_j ≥` the winner's value). A
rival whose bracket straddles the winner keeps the answer honest-`Approximate`
at the bracket's width, never blessed by the winner's pedigree.

**A touching pair's answer is `Exact` zero or no answer at all.** Verification
§1 makes a touching pair's zero `Gap` a measured zero, and verification §5's
near-zero rule reads it: at the noise floor `Ref` collapses to `Quantum`, an
`Exact` zero passes on its own terms, and an `Approximate` zero must earn
trust with a vanishingly tight bound. The analytic kernel honors that shape by
construction: a contact it can certify is a closed-form fact and reports
`Exact` zero (§6); a contact it cannot certify yields no row and the pair
reads `Suspect` — a failed classification is never laundered into an
`Approximate` zero that would then fail the floor anyway.

And the standing rule of verification §1/§6, restated once because every
section below leans on it: **no fabricated rows**. An unproven PARTITION —
a pair decided neither overlapping nor disjoint — joins neither list and
makes the report `Suspect` unasked; an unproven GAP does so only when
`WithClearances()` asked for it, since no rung needs a gap the caller never
requested (§7). The shipped `verify.go` semantics this kernel extends,
never relaxes.

## 2. Boundaries carry the problem — and boundary clearance alone does not decide it

**Face-to-face suffices for the distance.** The closest points of two solids
with disjoint interiors lie on their boundaries: an interior point sits at the
center of an open ball of its own material, and moving within that ball toward
the other body shortens the distance, so no interior point can realize a
positive minimum. Hence `d(A, B) = d(∂A, ∂B)`, faces cover boundaries, and the
gap is the minimum over face pairs — which is what makes a face-pair table
(§4) the whole kernel.

**Boundary clearance alone proves the wrong thing.** `d(∂A, ∂B) = g > 0` says
the skins never meet; it does not say the interiors are disjoint — one solid
nested wholly inside the other has exactly the same signature. Disjointness is
`g > 0` **and nesting excluded**. Because the boundaries provenly do not meet,
each connected shell of one body lies wholly inside or wholly outside the
other, so one exact membership test per shell decides it: a witness point on
the shell (every shipped face can produce one — a vertex, a seam point, or
for the loop-less closed faces a point off stored data: a sphere's center
plus its radius along a fixed unit direction, a torus's center plus
`Major + Minor` along a fixed direction perpendicular to its axis), cast
along a ray, crossings counted against every face of the other body, trim
membership included — closed-form for plane, sphere, cylinder and cone
crossings, while a torus crossing is a quartic and takes the same certified
Sturm isolation discipline as the P4/P8 table cells: a root count is a proof
only when the isolation certifies it. A cast is
admissible only when every crossing is certified transversal and interior to
its face; a cast that grazes an edge, a vertex or a tangency retries the next
direction on a fixed ladder — deterministic, never random, so a replay
resolves identically (evaluator §8). Interference PR 1 MUST return any witness
strictly inside as proven nesting — `pairOverlapping`, never the same state as
a failed cast; all
witnesses mutually outside, with `g > 0`, is the disjointness proof. Preserve
inside / outside / undecided for the shipped single-lump analytic payloads, per
shell. `docs/interference-design.md` §4 owns which combination of these
per-shell classifications certifies strict full containment; a single shell's
reading never settles it. Multi-lump faceted bodies bypass this analytic model
and use read-only intersection.

**Payload adapters preserve this obligation.** A cup expands to its exact outer
skin, reversed cavity skin, kept cap, pocket floor, and rim bands, so the same
analytic tiers and exact nesting casts apply. A faceted payload expands every
held triangle to a trimmed plane and carries one body-level displacement
`delta`; after held candidate aggregation the true distance interval is widened
once by the two payload deltas. A nonzero widened interval that reaches zero is
undecided unless a separate exact contact certificate settles it. Payload
verification §3/§7 owns these adapters and their tests.

## 3. The candidate enumeration

The distance over a compact face pair attains its minimum at some `(p, q)`,
and first-order conditions constrain each side independently: `p` interior to
the face → the segment `pq` runs along the surface normal at `p`; `p` interior
to an edge → `pq ⊥` the edge tangent; `p` at a vertex → no condition. Three
cases a side, six unordered tiers, and the enumeration is **complete** — every
local minimum, the global one included, is a candidate in exactly one tier:

| tier | stationarity | solved as |
|---|---|---|
| face interior × face interior | `pq` along both normals | surface-pair critical (§4) |
| face interior × edge interior | normal one side, `⊥` tangent the other | surface–curve critical (§4) |
| edge interior × edge interior | `⊥` both tangents | curve–curve critical (§4) |
| face interior × vertex | foot of the point on the surface | point–surface, closed form for all five surfaces |
| edge interior × vertex | foot of the point on the curve | point–curve, closed form for all three curves |
| vertex × vertex | — | a distance |

The vertex tiers quantify over the face's STATIONARY POINTS, not only the
topology's vertices: a cone's apex is a surface singular point — the normal
is undefined there, so no interior first-order condition holds and a minimum
can sit exactly on the tip — and a full-revolution cone carries NO axis
vertex in the shipped topology (`Body.Vertices()` returns edge endpoints
only, and `Face.NormalAt` rejects the apex). The kernel therefore
synthesizes a vertex-like candidate at every trimmed cone face's apex, read
off the stored surface data, and runs it through the two vertex tiers like
any topological vertex. A face with no edges at all — a full sphere, a STANDARD full
torus (`Minor < Major`) — contributes no curve tier and, being smooth
everywhere, no synthesized point either. A spindle-branch torus patch
(`Minor ≥ Major`, §4's `BB` downgrade) can reach the axis, where — as at a
cone apex — the surface is singular with no normal and no latitude edge:
its axis-collapse points are synthesized as vertex-like candidates the same
way, and the `BB` meridian-domain search includes its domain endpoints, so
a closest point there is never missed.

**Admission is exact, and doubt only ever costs figures.** A tier's candidates
are computed on the unbounded carriers, and each is admitted only when both
feet lie within their faces' trims. Trim membership on the shipped faces is
closed form: an angular interval × a meridian/axial range on the revolution
faces, loop containment with exact line/arc crossing counts on the planar
ones. A candidate whose foot leaves the trims is discarded — its minimum then
lives on the face's boundary, and a lower tier holds it. That discard is the
projection rule: two parallel planar skins facing each other contribute their
face-interior plateau only where the trims overlap in projection; offset
laterally until the projections clear, the plateau is discarded and the
minimum falls to the edge and vertex tiers (the worked cubes of §7). A
**bracketed** candidate whose foot straddles a trim edge within its bracket is
kept for the lower bound and never counted toward exactness — admission doubt
may widen the interval, never unsound it.

## 4. The face-pair table

One reduction shrinks the table before it is written: **three of the five
surfaces are constant offsets of a spine** — a sphere is a point ⊕ its
radius, a cylinder is a line ⊕ its radius, a torus is its spine circle ⊕ its
minor radius — and the surface distance follows from the spine distance `d`
in three branches, because an offset surface has an inside as well as an
outside:

- **exterior** — `d > r₁ + r₂` STRICTLY, with `d` the MINIMUM spine
  distance: the gap is `d − r₁ − r₂`, the two skins facing each other
  across free space. Equality is a tangency at distance zero, and a zero
  row exists only through a §6 contact certificate — an equality here
  routes to §6, and an offset tangency outside §6's certified list stays
  `Suspect`;
- **nested** — one carrier provably inside the other. Containment is a claim
  about the WHOLE inner spine, so this branch reads `d_sup`, the supremum of
  inner-spine-to-outer-spine distance, never the minimum: nested holds when
  `d_sup + r_in < r_out` STRICTLY, and the gap is `r_out − d_sup − r_in`,
  read across the annular space; equality is an internal tangency and
  routes to §6 on the same terms as the exterior branch. `d_sup` is certified per spine pair: an inner POINT spine (a sphere
  inside anything) has `d_sup = d` trivially — the supremum over one point
  is that point's distance; concentric spheres, parallel cylinder axes and
  coaxial torus spines have constant spine distance, so `d_sup = d` there
  too; any other nested configuration — a non-coaxial circle spine inside
  another curved carrier — has no constant supremum, and the pair routes to
  the `BB` path or stays undecided rather than borrowing this branch. In
  the certified cases the branch is closed form:
  two coaxial cylinders of radii 10 and 5 have `d = 0` and a genuine 5 mm
  gap, the peg-in-hole clearance a subtraction-only rule would misread as
  "carriers meet". A minimum-distance test could NOT carry this branch: two
  perpendicular crossing cylinder axes with radii 10 and 5 also have
  `d = 0`, and their surfaces intersect — that pair has `d_sup = ∞` along
  the unbounded spines, fails the containment test, and falls through;
- **otherwise** the carriers intersect, touch, or admit no containment
  proof: no positive gap exists through this reduction, and the pair falls
  to the boundary tiers, the `BB` path, or §6.

The branch test reads exact spine geometry, so which branch holds is itself
a decided answer. The feet map along the joining segment — outward on the
exterior branch, inward across the annulus on the nested branch — spine foot
to surface foot, and admission (§3) reads the mapped feet. A plane is its
own problem, and the spine-offset reduction does not apply to a cone at all —
a cone's surface is not at constant distance from its axis, so it has no
spine to offset from — which is what makes the cone cells the genuinely
iterative ones.

The face-interior table over the shipped surface set (`CF` = closed form;
`P4`/`P8` = a certified bracket on the stationarity polynomial — degree 4 for
line–circle, degree 8 for circle–circle, through the offset reduction; `BB` =
branch-and-bound, §5):

| | `Plane` | `Cylinder` | `Cone` | `Sphere` | `Torus` |
|---|---|---|---|---|---|
| `Plane` | CF | CF | CF | CF | CF |
| `Cylinder` | | CF | BB | CF | P4 |
| `Cone` | | | BB | CF | BB |
| `Sphere` | | | | CF | CF |
| `Torus` | | | | | P8 |

The table is triangular because distance is symmetric — pairs are unordered,
enumerated `i < j` exactly as the shipped partition loop already does. Row by
row: the plane row is elementary — parallel plane/plane and axis-parallel
plane/cylinder carry a constant plateau, a plane/cone plateau exists exactly
when the plane parallels a ruling (`|n·axis| = sin α`, the plateau at the
apex's own plane distance), plane/sphere is the center's plane distance minus
the radius, plane/torus the spine circle's plane distance (a closed-form
amplitude extreme) minus the minor; a carrier the plane crosses has no
positive plateau — and where the crossing itself is admitted by BOTH trims
it is an interior zero-distance contact, routed to the contact/overlap
handling of §6/§7, never quietly to the boundary tiers (which would read a
positive clearance off the edges while the faces cross); only a crossing
the trims exclude leaves the face minimum to the boundary tiers. The sphere
column is the point column ⊕: point/line, point/plane, point/circle and the
meridian point/cone are all closed form. Cylinder/cylinder reads the three
offset branches of the reduction above — exterior `d − r₁ − r₂` off the
axes' common perpendicular (closed form for parallel and skew axes alike),
nested `r_out − d_sup − r_in` for parallel axes with the containment proof,
and the fall-through otherwise. The two torus polynomial cells are the spine problems:
cylinder/torus is line/circle, whose stationarity is a degree-4 polynomial;
torus/torus is circle/circle, degree 8. **A certified bracket is a proof, not
a hope**: the polynomial's coefficients are the evaluator's own floats, taken
exactly, and Sturm counts over `math/big.Rat` on those exact values isolate
every real root to an interval that cannot lie — the same adaptive-exactness
discipline as the boolean's sign tests (evaluator §9).

The curve tiers, same legend (`Arc3` shares `Circle3`'s carrier and differs
only in trim):

| pair | solved as |
|---|---|
| `Line3` × `Line3` | CF |
| `Line3` × `Circle3` | P4 |
| `Circle3` × `Circle3` | P8 |
| `Line3` × `Plane` / `Cylinder` / `Sphere` | CF |
| `Line3` × `Torus` | P4 — line × spine, ⊕ |
| `Line3` × `Cone` | BB, 1-variable |
| `Circle3` × `Plane` / `Sphere` | CF |
| `Circle3` × `Cylinder` | P4 — circle × axis, ⊕ |
| `Circle3` × `Torus` | P8 — circle × spine, ⊕ |
| `Circle3` × `Cone` | BB, 1-variable |
| a vertex × any surface or curve | CF |

**Every `BB` cell reduces to a one- or two-variable azimuth search whose
pointwise evaluation is itself a `CF` or `P4`/`P8` cell.** A cone is its
ruling family: cone × cylinder is a one-variable search over the cone's
azimuth with a closed-form line × cylinder at each point; cone × torus the
same with a `P4` per ruling; cone × cone a two-variable search over both
azimuths with closed-form line × line pointwise. So `BB` never iterates on
raw surface points — it subdivides a compact angular domain whose pointwise
answers are proven, which is what makes its bounds provable (§5).

Two upgrades and one downgrade close the table:

- **Coaxial pairs are closed form regardless of the table.** Two surfaces of
  revolution about the same axis reduce to the 2D meridian problem — the
  distance between their generating segments and arcs in the shared `(z, ρ)`
  half-plane, closed form pairwise — but ONLY where the two faces' angular
  trims overlap: the meridian problem quantifies over full revolutions, and
  two partial faces with identical generators but disjoint sweep intervals
  have a positive real gap the meridian reading would wrongly report as
  zero. The face-interior candidate is admitted when the sweep intervals
  overlap with positive interior measure (a decided closed-form test);
  endpoint-only touching is an angular-BOUNDARY configuration — §6 leaves
  edge and vertex contact undecided, and blessing it here would mint an
  `Exact` zero row §6 forbids — so it, like disjoint intervals, discards
  the candidate and the minimum falls to the angular boundary edges and
  vertices, exactly as a trim-departed foot does in §3. This upgrades every `BB` cell for the
  layout revolved bodies most often produce.
- **Co-directional prisms reduce to 2D.** Two prisms with parallel sweep
  directions have `Gap = hypot(dz, d₂)` where `dz` is the sweep-interval
  separation (zero when the intervals overlap) and `d₂` the 2D distance
  between the profile REGIONS — a CLEARANCE result only while at least one
  of the two is positive. `dz = 0` AND `d₂ = 0` together decide nothing by
  themselves — the double zero splits on WHY each is zero. `dz = 0` from
  endpoint-only sweep contact (the intervals touch, with no positive-length
  overlap) is cap-on-cap CONTACT whatever the profiles do in projection —
  the coplanar plane-pair certificate of §6 owns it, never §7. With
  positive-length sweep overlap, the split reads `d₂`: positive-area
  interior overlap or containment of the 2D regions means the solids share
  volume, the §7 proven-overlap path; boundary-only 2D contact (two extrudes
  sharing a side face) leaves the interiors disjoint and routes to the §6
  contact classification, whose certificates alone may bless the touching
  zero; and a `d₂` the 2D tests cannot classify is undecided — `Suspect` —
  never an `Exact` zero `Clearance` row minted from the shortcut itself. That is an operational two-step: `d₂` is the
  minimum over boundary pairs (line/arc pairwise, closed form) UNLESS the
  regions overlap or one contains the other — decided by an exact 2D
  point-in-region test on any boundary point of each — in which case `d₂` is
  zero. Boundary distance alone would overstate the gap for a profile nested
  inside another with no boundary crossing. Two extrudes off one sketch
  plane land here.
- **A torus face with `Minor ≥ Major` leaves the polynomial path.** The
  spine-offset foot map is faithful only while every surface point's nearest
  spine point is unique — a `Minor ≥ Major` (spindle-branch) tube wraps the
  axis and breaks that uniqueness, and the shipped topology can carry such a
  trimmed patch (an arc whose radius exceeds its center's axis distance,
  swept above the axis). Those faces take the `BB` path (the torus as its
  meridian-circle family) — a wider bound, never a wrong `Exact`.

## 5. Intervals: witnesses, conservative bounds, refinement

- **An upper bound is any admitted witness.** Evaluate any on-face point pair
  — subdomain centers, mapped feet — and `d(p, q) ≥ Gap` holds by definition.
  A witness must be admitted (on the trimmed face, not just the carrier) or
  it bounds nothing.
- **A lower bound must under-estimate, never over.** Each parameter subdomain
  gets a conservative enclosure from the payload's own extreme machinery (the
  directional extremes and angular-sweep extremes the prism and revolve
  bounds already compute), and enclosure distance ≤ true distance. The safe
  failure is a loose bound — it costs refinement, never soundness — the same
  one-sided discipline as verification §4's noise floor, which may sit too
  low but never too high.
- **Pruning reads the bounds it just proved.** A face pair or subdomain whose
  lower bound exceeds the current best upper bound cannot hold the minimum
  and is dropped; because the bound is conservative, pruning never discards
  the argmin. Body boxes prune first (the shipped `boxesDisjoint` machinery),
  then per-face enclosures, then subdomains.
- **Refinement stops at the caller's own gate.** Subdivision runs until
  `(hi − lo)/2 ≤ rel × max(lo, δ)` — the verification §2/§5 test evaluated
  conservatively from below — because figures past the gate change no
  verdict. A fixed, deterministic subdivision order and depth budget bound
  the work (`ctx` checked per pair, as the shipped loop does); on exhaustion
  the honest wide interval stands — a row exists only if `lo > 0`, and the
  gate reads a wide `Bound` `Suspect`. Never a tightened number, never a
  silent pass.
- **Held bounds subtract before anything is proven.** A body whose held
  boundary carries a nonzero proven bound (a `Faceted` body, increment 4)
  clears only what exceeds the summed bounds (evaluator §10): the pair's
  proven `lo` is the held-boundary `lo` minus both bodies' bounds, and the
  row's `Bound` folds them in. In increment 3 every body is feature-built and
  the held bounds are zero, so the subtraction is exact nothing.

## 6. Touching pairs

At a touching pair `lo` can never rise above zero, so the positive-gap proof
cannot run; the contact itself must carry the disjointness proof. **A pair is
proven touching when every zero-distance contact is a certified tangential
contact whose outward material normals strictly oppose, and the boundaries
provenly do not cross anywhere else.** Opposing outward normals — the face's
`reversed` folded in, so a hole wall's outward normal is its surface normal
negated — put the two materials on opposite sides of the shared tangent
plane, which clears the interiors locally at the contact; no crossing
elsewhere, plus the §2 witnesses outside, closes it globally. Aligned normals
at a contact are the opposite verdict: the materials sit on the same side —
a body pressed into material, or touching a wall from inside it — and the
pair is proven not disjoint (§7).

The certified contact types, each a closed-form fact of the shipped surfaces:
coplanar `Plane` × `Plane` with opposing normals and trims that overlap — the
stop-built extrude case, a body extruded to a stop on another body sharing
that plane cap against cap; `Sphere` × `Plane` at the foot point; `Sphere` ×
`Sphere` on the center line; `Plane` × `Cylinder` and `Plane` × `Cone` along
the tangent ruling; parallel external `Cylinder` × `Cylinder` along the
common ruling. Each certifies its whole contact set in closed form — a
point, a ruling segment, or for the coplanar cap-on-cap case a 2D REGION,
the positive-area overlap of the two trims — so the opposition test
quantifies over all of it, not over samples. The plane pair certifies only
when the trim overlap has positive area; trims meeting along an edge or at
a vertex alone are the undecided case below. Anything else at distance
zero — a curved-on-curved osculation, a contact through an edge or a
vertex, a contact set the kernel cannot describe in closed form — is
undecided: no row, `Suspect` (verification §6). The blessed answer is `Gap` `Exact` zero, and it passes
the near-zero gate on its own terms (§1).

## 7. Feeding the report

**The kernel serves two callers, and only one of them is opt-in.** The
partition is always owed: verification §1 computes `Interferences`
unconditionally and holds the pair partition to proof, and verification §6's
absence standard makes a pair-in-neither-list a claim a `Sound` report must
have proven — so the disjointness proof (`lo > 0` + nesting excluded, or a
§6 contact) runs for every undecided pair whether or not anyone asked. The
measurement is opt-in: `WithClearances()` — which takes nothing, because
`Gap` is a measurement and not a verdict (verification §2) — additionally
refines each proven-disjoint pair's interval to the §5 gate target and emits
the row. The split is exactly verification §2's line: the partition is a rung
the caller is owed unasked; no rung needs a gap the caller never asked for.

- **Rows exist only for proven-disjoint pairs** (verification §1). A
  box-proven pair (shipped, evaluator §10) is already partition-decided, but
  its row still needs the kernel: box separation proves disjointness, not the
  gap — the box distance is a lower bound, not a minimum.
- **Interference PR 1 makes overlap and uncertainty distinct.** `lo` that
  cannot clear zero with no certified contact, a budget-exhausted refinement
  whose `lo ≤ 0`, and an
  uncertified cast are `pairUndecided`. An admitted boundary crossing, an
  aligned-normal contact whose material sides prove shared interior, or a
  nesting witness is `pairOverlapping`. Neither emits a `Clearance` row. The
  overlap relation proceeds to the containment-volume or read-only intersection
  path of `docs/interference-design.md`; if neither bounds the complete overlap
  volume, the pair makes the report `Suspect` without losing the relation the
  analytic kernel already proved.
- **`WithClearances` asked with no pairs answers the empty list** — non-nil,
  the shipped `[]Clearance{}` — a document of zero or one proven solid has no
  pairs, and the empty list is that answer in the shape verification §1 gives
  every pair result: an answer, never an absence.
- **The gate reads pair geometry.** A `Gap` is a `Measurement` of Kind
  Length: `Ref = max(|Value|, Quantum)` with `Quantum = δ = ε × D`, `D` the
  **pair's** diameter exactly as verification §3 defines it — the greatest
  distance between two points drawn from either body. The evaluator READS
  that `D` from exact vertex pairs and per-face analytic support points, and
  the reading may understate the true diameter; verification §4 already
  admits exactly this (its diameter reading understates a curved body "by at
  most the chord error") because a floor's ingredients are magnitudes, not
  answers, and understating `D` lowers the floor — the only safe direction:
  it can demand more of an answer, never admit one. The definition is the
  parent's, untouched; only the reading is the evaluator's. A `Gap` beyond
  tolerance makes the report `Suspect` directly (verification §6, the last
  rung).

Worked, at the default `rel = 1e-3`:

| pair | closest features | kernel path | `Gap` | pair `D` | reads |
|---|---|---|---|---|---|
| 10 mm cube at origin; 10 mm cube at x∈[13,23], y∈[12,22] | two parallel vertical edges | facing-face plateau discarded (trims clear in projection); edge × edge CF | √13 ≈ 3.606 mm, `Exact` | 33.4 mm | passes |
| the same cubes stacked 2 mm apart | facing caps, trims overlap in projection | face × face plateau CF | 2 mm, `Exact` | 26.2 mm | passes |
| a stop-built stack sharing its cap plane | coplanar caps, opposing normals | §6 contact | 0 mm, `Exact` | — | passes on its own terms (verification §5) |
| two tori, parallel axes 30 mm apart, major 10, minor 2 | tube to tube | circle × circle P8, ⊕ | 6 mm ± 5e-10, `Approximate` | ≈54 mm | passes — 5e-10 ≤ 6e-3 |
| a cone face near a torus, budget out at `lo` = 0.5 mm | — | BB, coarse | 0.8 ± 0.3 mm | 40 mm | row stands, **`Suspect`** — 0.3 ≫ 8e-4 |

The last row is the honest coarse answer: the partition is decided — disjoint,
proven — and the measurement is not to the figures asked, so the row exists
and the gate says so.

## 8. Increments

PR-level staging inside evaluator increment 3, each staged answer reading per
evaluator §11's rule — a question the evaluator cannot answer is accepted and
reads `Suspect`, never an error, never a silent pass:

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 | the tier enumeration + exact admission, every CF cell, the P4/P8 certified brackets, the nesting exclusion, coplanar `Plane` × `Plane` contact, report wiring — rows, the empty list, the `Gap` gate, pair `D` | cone-involved pairs near contact (coarse enclosure interval only: proven disjoint with a wide honest row when even the coarse `lo` clears zero, undecided when it does not); every non-coplanar contact type |
| 2 | the `BB` refiner (the 1- and 2-variable azimuth searches; the `Minor ≥ Major` torus downgrade path), the coaxial and co-directional 2D reductions | non-coplanar contacts |
| 3 | the remaining §6 certified contact types | osculating and edge/vertex contacts (§9) |

Cup and faceted adapters land in payload verification §13. Until their stages
land, an invoked pair containing that payload remains `Suspect`; the analytic
kernel does not tessellate a cup or discard a faceted displacement bound.

## 9. Open questions

- **Contact breadth.** Osculation (equal-curvature touching), vertex and edge
  point contacts, and contact sets with mixed tangential/transversal pieces
  are all undecided by §6's set. Which of them real models produce often
  enough to certify?
- **A clearance witness.** Should `Clearance` also name the closest point
  pair (two `VecMeasurement`s)? The core §5.3 shapes allow it; verification
  §1 does not currently carry it.
- **A refinement budget knob.** The depth budget is fixed and deterministic;
  is a `VerifyOption` for it a spec the caller should be able to state?
