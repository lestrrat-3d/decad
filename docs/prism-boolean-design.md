# Prism Boolean Design

An analytic reduction for `Union`/`Cut`/`Intersect` over co-directional
coplanar prisms: the 3D boolean reduces to a 2D combination of the two
operands' recorded sections, routed entirely through `sketch` — decad selects
among the regions `sketch`'s arrangement returns and rebuilds a `prismPayload`
from the selection; it never computes a crossing, a cut parameter, or a region
membership of its own. Companion to `docs/evaluator-design.md` §9 (the
existing mesh boolean, unchanged, and still the fallback for everything this
design does not admit) and `docs/modify-design.md` §2 (the section-rewrite
reduction this design reuses and extends from one operand to two). References
of the form "core §N" are to `docs/api-design.md`; "modify §N" to
`docs/modify-design.md`; "seam §N" to `docs/sketch-seam-design.md`.

## 1. Problem

`Union`/`Cut`/`Intersect` tessellate both operands and intersect meshes
(evaluator-design §9). For a pair of straight prisms extruded from the same
plane — the normal shape of a combined part built from sketch-adjacent
features (a hub and its teeth, a boss and a plate) — that pipeline is
structurally worst-cased three ways, all traced in
`.tmp/boolean-redesign/README.md` and `.tmp/gear-generator-feedback/README.md`:

1. **No chaining.** The result's `meshBound` composes across operations and
   exceeds the chord tolerance the next pair derives from its own diameter, so
   a second boolean on the same lineage refuses (`ErrUnsupported`,
   "requested tolerance below the faceted body's minimum mesh bound").
2. **Coplanar contact refuses outright.** `triTriClassify`'s `contactRegion`
   is a tangency the exact chord predicates cannot classify
   (`errUnclassifiableContact`), and two bodies sharing an extrusion plane
   always have one.
3. **Analytic identity dies.** The result is `facetedPayload`: modify-reach
   SX9 refuses every modify op on it permanently, and `survey.go` refuses all
   three shape surveys.

Two upstream `sketch` changes since that report — commits `e0d5d49` ("resolve
coincident carriers into regions") and `18a947b` ("certify curve/curve
crossings as exact") — remove the two obstacles that previously ruled out
routing this work through `sketch` instead of building a second, decad-owned
2D intersection kernel: a circle×circle crossing now reports
`TExact = true` on every fragment (verified against
`sketch@f982746`, `.tmp/decad-2d-region-asks/probe`), and a tooth arc lying
exactly on a hub circle now arranges without the `Degenerate` flag into two
clean regions sharing that arc as one boundary, instead of one region
reporting the whole hub disk with the coincident boundary split arbitrarily
between the two. **This design routes the 2D combination through `sketch` on
that basis and does not bend the hard rule in CLAUDE.md**: "an intersection ...
ask `sketch`, consume its answer." decad builds a scene holding both operands'
recorded entities, asks `sketch` to arrange it, and selects among the returned
regions; §4 states precisely, for each op, how that selection is made without
decad computing a crossing or a containment test of its own — and names the
one sub-case (general-position holed union, PR3) where the honest answer is
"not yet," staged rather than smuggled in.

**Success criterion.** For the admitted class (§2): a chained boolean carries
no accumulated tessellation bound; a shared or crossing extrusion plane is the
ordinary case, not a refusal; the result is a first-class `prismPayload` that
Fillet/Chamfer/Shell, all three surveys, and the clearance kernel already
handle. §8 states exactly which of the three consequences above disappear for
the admitted class and which stand outside it.

## 2. Approach

**Chosen direction.** PR1 dispatches a reject-only admission gate from
`performBoolean` for `Union`, ahead of the shared `evaluateBoolean` mesh
pipeline. An admitted pair combines its two operands' `ProfileRecord`s through
a private `sketch` scene decad builds. It records the selected result regions
through the existing seam (`RecordProfile`/`recordEdge`), audits the assembly
with `modify §5`'s existing closed-form checks, and rebuilds through
`evalPrism`. Section 7 extends the result with one section-displacement bound
so the re-expressed operand's rounding reaches every measurement.
`Cut` and `Intersect` remain on the mesh path until later increments. A
non-admitted `Union` pair — wrong payload class, non-coplanar, a segment kind
outside the admitted set, an unequal z-interval for `Union`, a nonidentity
re-expression whose arranged boundary is split, or a topology this increment's
region resolution does not cover — takes the unchanged mesh path, with **zero
behavior change**: no new error, no new refusal text, nothing a caller not
making booleans this shape will ever observe.

**Alternatives considered:**

- **Decad builds its own 2D regularized-boolean kernel** (the investigation's
  "candidate B", predating the upstream `sketch` fixes) — rejected now that
  the two `sketch` asks it needed for the gear's normal case landed anyway;
  building a second intersection kernel duplicates work `sketch` already does
  and durably contradicts the hard rule rather than complying with it.
- **General exact analytic B-rep boolean** (arbitrary curved-surface pairs,
  algebraic root isolation) — the eventual vN destination (api-design §2.1),
  multi-quarter scope; this design's gate/audit/authentication pattern is its
  first honest increment, not a competing direction.
- **Mesh-path palliatives** (a caller-set chord tolerance, an N-ary `Union`) —
  removes only the chaining consequence, and only partially; the coplanar
  refusal — which blocks the gear workload *twice*, independent of chaining —
  stands untouched. Not pursued as the primary fix, though `WithPairTolerance`
  remains a defensible independent ask for genuinely general-position pairs
  outside this design's admitted class.
- **Keep the CSG tree, answer queries against it without evaluating** — breaks
  the promised B-rep-shaped surface (`Faces()`, selectors, stops) and, for the
  workload that exists, converges on this design's own 2D combination anyway
  to answer the wall/undercut surveys — see `.tmp/boolean-redesign/README.md`
  §3.D for the full argument. Not pursued.

## 3. The admitted class

### 3.1 Entry gate (silent fallback, never an error)

Checked before any `sketch` call, on the two operands' *current* payloads
(`prismPayload`, which includes a shell-built tube — modify §8's
`evalTube` builds one). Every row is a routing decision: failing it takes the
unchanged mesh path with no error surfaced, identical to today's behavior for
that pair.

| # | Condition | Why exact, not tolerant |
|---|---|---|
| G1 | Both operands' payload is `prismPayload`. | Structural — the class this design's reduction applies to (modify §2's same reduction, extended to two operands). |
| G2 | Neither operand's accumulated placement is a reflection (`!xform.IsReflection()`). | A reflected operand flips winding/arc sense through the combination; deferred rather than threading a sign correction through §4 for a case no current consumer needs. |
| G3 | The two operands' **composed world planes** (`xform ∘ frame`, core §5.2/§6.2's r3 vocabulary — `worldOrigin = xform.Apply(frame.Origin())`, `worldNormal = xform.ApplyDir(frame.N())`) are the same plane, exactly: `worldNormalA == worldNormalB` (Go `==` on the stored `r3.Vec` floats — component-wise exact equality, which treats `-0.0` and `0.0` as equal, §3.3; "co-directional" — the same outward sweep sense, not merely antiparallel) **and** `(worldOriginB − worldOriginA)·worldNormalA == 0.0` (an ordinary float64 dot product compared against the literal zero). | This is decad's own admission decision, not a question `sketch` answers, so CLAUDE.md's reject-only rule binds it directly: a residual test here would be an admission gate on a residual, which the hard rule forbids outright. Every quantity is read off the stored `r3.Vec` floats as-is (the `clearance_degen.go` discipline: exact arithmetic on the payload's own floats, never a re-derived angle) — never loosened to a tolerance. §3.3 covers what this excludes and why it is not fixed here. |
| G4 | Every segment of both operands' `ProfileRecord` (`Outer` and every loop of `Holes`) is a `LineSeg`, `CircleSeg`, or `ArcSeg`. | `geom.BoundaryEdge.TExact`'s own contract is a **whole-scene** gate (`sketch`'s `geom/region.go`): one `Ellipse`/`EllipticalArc`/`Conic`/`Spline`/`ClosedSpline`/`FitSpline`/`NURBS` anywhere in an arrangement makes every bound in it — including unrelated line/circle/arc edges — report `TExact = false`. A single free-form segment on either operand would silently blind the whole combination, not just its own edges, so the gate excludes the kind entirely rather than trying to admit "the free-form parts don't touch." Staged: §9's free-form row. |
| G5 | The z-interval relation the op needs (§3.2) holds, computed after re-expressing operand B's `[z0, z1]` onto operand A's normal axis: `z' = z + (originB − originA)·normalA` (an origin shift along an axis G3 already proved identical — ordinary float arithmetic, no rotation). | A shift, not a containment test — G3 already certified the shared axis; this is bookkeeping on it. The shift is only a comparison input for `Union` and `Cut`, whose result interval is one operand's own endpoints verbatim (§3.2); for `Intersect` a shifted endpoint can reach the result, and its rounding is the same rigid-shift mechanism §7's displacement term already carries. |
| G6 | `ProfileRecord.Holes` is empty wherever §4.2's selection rule for the op needs it: `Union` needs it on **both** operands, `Cut` needs it on the **tool** (the target's own holes are carried through unchanged). | `Union`'s rule (select every returned cell) is sound only when neither operand has a hole a returned cell could sit inside without touching either outer boundary; holed unions are §9's PR3 row. `Cut`'s clean-nesting match describes the removed tool as **one** new hole reproducing the tool's `Outer`, which is the tool's whole solid only while the tool is hole-free: a holed tool's solid is its `Outer` minus its own `Holes`, so the material standing inside each tool hole survives the cut as an island that one new hole does not describe. Those islands are disconnected lumps a single `ProfileRecord` cannot carry (§4.4's multi-lump row), so the pair falls back rather than building a body missing them. |

G1–G6 are the only conditions checked before touching `sketch`. **Passing
them is not admission** — see §3.4.

### 3.2 Per-operation shape

| Op | Required relation (after G5's shift) | Result z-interval |
|---|---|---|
| `Union(a, b)` | `[z0_a, z1_a] == [z0_b', z1_b']`, exactly | the common interval |
| `Cut(target, tool)` | `z0_tool' <= z0_target && z1_tool' >= z1_target` (tool spans target) | `[z0_target, z1_target]`, unchanged |
| `Intersect(a, b)` | `z0_a < z1_b' && z0_b' < z1_a` (intervals overlap) | `[max(z0_a, z0_b'), min(z1_a, z1_b')]` |

`Union`'s equality is exact float equality — not a tolerance — matching G3's
discipline: two teeth swept to visibly-the-same but not exactly equal heights
(e.g. built through two different construction paths) refuse to the mesh
path rather than being blessed as "close enough." `Cut`/`Intersect`'s
inequalities are ordinary comparisons on already-exact endpoint floats (no new
rounding, so no exactness risk in comparing them directly); a boundary case
(tool's cap exactly meets target's) is a valid span/overlap.

Every other relation (unequal-interval union, a cut whose tool does not fully
span the target, disjoint intervals for intersect) is **staged, not
refused**: it falls through the gate to the unchanged mesh path today, and
names the future payload that would carry it in §9's increments row (a
stacked-interval union is `stackedPrismPayload`, modify-reach §9.1; a
non-spanning cut is a `cupPayload`-shaped pocket).

### 3.3 The rotated-section problem

The consumer's teeth are placed copies arranged by a circular pattern, so
each tooth's section arrives at a different world rotation. Two probes
establish the following against `r3@2e6d6464` —
`.tmp/boolean-redesign/probe/` and, for the swept counts and the `Frame.N()`
reading, `r3`'s own `.tmp/decad-axis-exact-rotation-ask/probe/`:

- `r3.RotationAround` about `(0,0,1)` keeps the composed plane's normal
  **exactly** `(0,0,1)` (G3's exact `==` test passes) for only a minority
  of step counts. Swept over every count from 3 to 60, **48 of the 58 have at
  least one inexact step**, including every count from 19 upward and also 7,
  10, 13, 14, 16 and 17; the z-component picks up a nonzero float
  (`0.99999999999999989` at `n = 17`) and G3 correctly refuses the pair to
  the mesh path. The exact counts are the sparse case, not the rule, and the
  consumer's own range of 12 to 45 teeth sits almost entirely inside the
  inexact set. The cause is the general Rodrigues evaluation, whose
  `cos + fl(1 − cos)` term is exact only when the roundings happen to
  cooperate, so this is a missing guarantee rather than a defect.
- A hand-built basis through `r3.FromBasis` (literal `0.0` z-components,
  `U = (cos θ, sin θ, 0)`, `V = (−sin θ, cos θ, 0)`) stays exactly planar for
  every `n` probed, because the zero components are literal, not computed. It
  survives `Then` composition, and an on-axis pivot adds no new failure.
- `Frame.N()` is **itself** inexact for an in-plane rotated frame, returning
  an axis component one to two ulps off 1 and a `-0` in practice. G3 reads
  `xform.ApplyDir(frame.N())`, so `N()` is one of its inputs. Both operands
  of the consumer's union descend from ONE sketch plane, so both sides read
  the same `N()` and the rotation stays the only failing term; two operands
  whose frames were constructed independently do not have that protection,
  and G3 may refuse a genuinely coplanar pair. That refusal is sound under
  the reject-only rule and is not fixed by loosening the comparison.
- Because `-0` occurs, note that a bit-identical comparison and Go's `==` do
  not agree here: `==` treats `-0.0` and `0.0` as equal. G3 is specified as
  Go `==` on the stored floats, which is what the implementation must use.

**Decision: G3 stays exact, unconditionally — no tolerance is introduced to
paper over `RotationAround`'s inexactness.** A residual-based coplanarity
test would be exactly the admission-gate-on-a-residual the hard rule forbids;
loosening G3 would also loosen the axis identity §4.2's coordinate
re-expression and §4.3's coincident-carrier detection lean on. The practical
consequence is that a circular pattern built with `r3.RotationAround` alone
silently under-triggers this design at most step counts, falling back to the
unchanged, working mesh path. That is a missed optimization rather than a new
failure mode, but it is the common case and not the exception. A consumer can
use `FromBasis` to clear G3, but §3.4 still routes any re-expressed arrangement
with a split boundary to the mesh path. **The r3 upstream ask is filed** at
`r3/.tmp/decad-axis-exact-rotation-ask/`, requesting a rotation constructor
whose result leaves a plane perpendicular to its axis exactly perpendicular.
This design does not block on it: nothing here requires it, and nothing here
regresses without it.

### 3.4 What "admitted" means, precisely

Admission is **two-stage**, and the boundary between the stages is where a
silent fallback stops being available:

1. **Entry gate (§3.1–3.2).** Cheap, pre-`sketch`, structural. A miss here is
   never an error — the unchanged mesh path runs exactly as it does today.
2. **Bounded region resolution (§4).** Before `s.Profiles()` runs, the
   code-owned `prismUnionMaxArrangementSegments` cap bounds the pinned
   arrangers' tiny-segment pair work. Exceeding it is an `ErrUnsupported`
   refusal (§9), never an unbounded private worker. A pair within the cap
   builds the private scene, arranges it, and attempts to resolve a unique
   candidate result. **A pair whose topology this increment's resolution logic
   does not cover (§4.4) is treated exactly like a stage-1 gate miss: silent
   fallback, no error.** The same routing rule applies before a candidate is
   accepted when B's re-expression is nonidentity and `sketch` returns any
   `Partial` boundary edge: a coordinate error in B can move a transverse cut
   on A by that error divided by the crossing sine, and this increment carries
   no certified crossing-sensitivity bound. Every other capacity, arrangement,
   candidate-validity, or assembly-audit problem is a genuine refusal (§9's
   table), **never** a reroute to the mesh path. An admitted-then-failed pair
   does not silently become an `Approximate` mesh result whose exactness claim
   the caller never asked to downgrade to; it becomes an explicit
   `ErrUnsupported`/`ErrDegenerate` the caller can branch on, matching every
   other modify-op refusal in this codebase.

## 4. Design

### 4.1 Scene construction

For an admitted pair, decad builds one private `sketch.Sketch` (the same
`sketch.NewWorld().CreateSketch(...)` pattern `moments_validate.go`'s
`momentRecordScene` already uses for authentication):

- Operand A's frame is the reference (`target`'s, for `Cut`). Operand A's
  `ProfileRecord` entities are created verbatim — same `Point2` floats, zero
  new rounding.
- Operand B's `ProfileRecord` is **re-expressed into A's frame** before its
  entities are created, through the **composed relative map**: B's frame to
  world, B's placement, A's placement inverted and A's frame inverted are folded
  into ONE rigid transform (`r3.FromFrame` and `Transform.Then`, each inverse
  exact — the transpose, `r3.Transform`'s own contract (core §5.2) — and every
  step a dot product, never a solve), and each `Point2` is mapped once through
  it, dropping the resulting local z, which G3 already certified is the shared
  plane's own zero axis. Composing first is what keeps the rounding at the two
  operands' RELATIVE offset: walking each point out to world space and back
  rounds it at each operand's own WORLD magnitude instead, so a pair sitting
  1e16 mm from the origin — where one ulp is 2 mm — loses millimetres to a
  translation the two operands share and that cannot move them relative to each
  other at all. Composing proves nothing, though; it narrows the rounding
  without removing it, which is why §7 carries the displacement regardless. This
  is the **one and only** new rounding this design introduces: an ordinary
  rigid-transform coordinate computation, rounded once per coordinate, on
  operand B's segments only — and where B's frame and placement ARE A's own in
  the stored floats (component-wise `==`, G3's own comparison) the composed map
  is the identity, B's `Point2` fields are copied verbatim, and nothing is
  computed at all (§7's decidable zero case).
  Non-`Point2` fields (a `CircleSeg`'s `Radius`, every `CCW`, every `TStart`/
  `TEnd`) are magnitudes or parameters under a rigid, non-reflected
  (G2) map and carry over unchanged.
- Every created `sketch.Entity` is tagged, in a side map, with its origin:
  which operand (A or B) and which loop (`Outer` or `Holes[i]`) it came from,
  and the **authored orientation** decad recorded it with (`CCW`/`Reversed`
  as the segment's own record states — record.go's existing "outer loops CCW,
  holes CW" convention). No entity is deduplicated across operands before
  handing them to `sketch`: a coincident carrier (the tooth root arc and the
  hub circle) is created as **two separate entities** with numerically
  matching center/radius, exactly the probe C construction — `sketch`'s own
  coincident-carrier resolution (its `carriersIdentical` round-off band,
  `weldIdentEps = 1e-12` scale/radius-relative) decides whether they merge,
  and that decision is `sketch`'s to make, not decad's to second-guess or
  pre-empt. An ordinary rigid-transform rounding (a few ulps) clears that band
  by three-plus orders of magnitude on any geometry with a real coincident
  carrier by construction (`docs/coincident-carrier-resolution-design.md`'s
  own sizing: "a couple of ulps of `r`").

### 4.2 Region resolution

`s.Profiles()` on the built scene returns the arrangement's bounded cells —
the full planar-map decomposition (a lens plus two crescents for two
overlapping circles; a disk plus an annulus-with-hole for a nested pair) —
never a pre-merged union. decad selects among them per op. Selection reads
only facts `sketch` already computed (each cell's boundary edges, their
`SourceIndex`→entity trace via the tag map above, their `Reversed` flag,
their `Whole` flag, and `Profile.Valid`) — it is comparison and bookkeeping on
those facts, never a point-in-polygon test, a containment computation, or a
re-derived crossing parameter.

**`Union`, hole-free operands (G6): select every returned cell.** For two
hole-free operands, every bounded cell the arrangement of `{A's outer,
B's outer}` alone produces is, by construction, material of A, of B, or both
— there is no bounded cell that is material of neither (a hole would be the
only way to produce one, which G6 excludes). So the union's material is
*every* returned cell, unconditionally: no per-cell classification is needed
at all for this sub-case. Assemble by taking the multiset of every selected
cell's boundary edges and dropping any edge that appears on **two** selected
cells (matched by `Entity` identity plus its unordered `{TStart, TEnd}` pair
— a shared wall between two adjacent selected cells, walked in opposite
senses by each), keeping every edge that appears on exactly one (a wall
against the unbounded exterior — the survivor is the union's own outer
boundary). Chain the survivors into a closed directed walk (the same
directed-edge-loop-closure shape `boolean_body.go`'s face-patch construction
already performs for the mesh boolean's own loops); a chain that does not
close into exactly one simple loop — a disjoint pair of footprints producing
two separate lumps, which the single-outer-loop `ProfileRecord` cannot
represent, or a union that encloses an internal void — is not resolved by
this increment (§4.4).

**`Cut`/`Intersect`, "clean" sub-case: structural whole-loop match, no
per-cell classification at all.** When operand B's boundary does not touch
operand A's anywhere (`Cut`'s bore-through-a-solid-hub shape; a fully nested
`Intersect`), the arrangement leaves **both operands' original loops
completely unmodified** — every one of their edges reports `Whole = true`,
because nothing cut them. decad scans `s.Profiles()`'s results for the one
profile whose `Outer` structurally reproduces target/A's original `Outer`
verbatim (same entities, same order, every edge `Whole`) — for `Cut`, whose
`Holes` additionally reproduce target's original holes plus **one new hole**
that structurally reproduces tool/B's own `Outer` verbatim (also every edge
`Whole`; G6 keeps the tool hole-free, so that one hole is the tool's whole
solid and no material inside a tool hole is dropped); for `Intersect` with B
fully inside A, the match is simply B's own
disk cell, `Outer` reproducing B's original loop verbatim. A structural
match — entity identity, order, and `Whole`-ness, nothing geometric — is a
pure data comparison against decad's own tag map. **When a unique such
profile exists, it is not assembled at all: it is one of `s.Profiles()`'s own
results, verbatim, and is authenticated by handing it directly to the
existing public `RecordProfile(s, profile)` — the full seam (§5) applies
unmodified, no new authentication code.**

**`Cut`/`Intersect`, crossing sub-case (boundary contact, not clean nesting):
staged (§4.4, §9 PR3).** When operand B's boundary genuinely crosses
operand A's (a bore that pokes outside the hub, or a general-position
overlap), neither `Union`'s select-all rule (only sound because `Union`
wants *every* cell) nor the clean-nesting structural match (which requires
every edge `Whole`) applies. The general mechanism — stated here for
completeness, since the design must answer how region selection works in
general, even though increment 1/2 does not build it — is **edge-orientation
propagation**: for a cell touching a boundary edge sourced from operand X,
compare that edge's `Reversed` flag (in this cell's own walk) against X's
recorded authored orientation for the loop that entity came from
(record.go's "outer CCW, holes CW"); a match means the cell sits on X's
material side at that edge (X's authored convention already puts material on
the walk's left), a mismatch means X's void side. This is a **flag
comparison**, not a geometric test — the classification `sketch` already
encodes in which direction it walked a shared entity for this specific cell.
For a cell with a direct edge from both operands, both readings are
available and the four combinations (`A-only`, `B-only`, `both`, impossible-
for-two-simple-curves-`neither`) select `Union`'s "either", `Cut`'s "A and
not B", `Intersect`'s "both". For a cell touching only one operand directly
but adjoining a **coincident carrier** (the tooth-only cell of the gear's
shared-arc case, which the merged edge names under only one operand's entity)
— decad additionally identifies, from its **own** recorded segment data
(comparing `Point2`/radius fields for exact equality after §4.1's
re-expression — the same discipline `momentRecordScene`'s dedup key already
applies, an algebraic comparison of decad's own numbers, never a geometric
one), which of its own segments across the two operands describe the same
carrier, and derives the fixed relative orientation between the two operands'
authored senses for that shared carrier once, structurally, from their own
recorded windings — never from evaluating a point against either curve. A
cell this propagation cannot reach at all (isolated from every operand
boundary by an unclassified path) is unresolved and falls to §4.4.

### 4.3 Assembly output

Every resolution path (§4.2) ends with a candidate `ProfileRecord`: either
one `s.Profiles()` result taken verbatim (the clean-nesting match, no
assembly), or a merged set of surviving boundary edges chained into a closed
loop (`Union`'s select-all case, and the staged crossing case). Each surviving
edge is converted to a `CurveSegment` through the **existing**, unmodified
`recordEdge`/`falsifyRange` pair in seam.go (package-internal, so this new
code calls it directly) — every fragment's `TExact` admission and reject-only
range falsifier applies exactly as it does for a caller-drawn profile,
because it is the same check on the same kind of `sketch.BoundaryEdge`,
regardless of who authored the input curves it was cut from.

### 4.4 What stays outside the admitted class

| Shape | Where it stands |
|---|---|
| Non-coplanar, non-co-directional, reflected, or non-analytic-segment pairs | G1–G4, mesh path, unchanged |
| `Union` with unequal z-intervals | G5, mesh path; future `stackedPrismPayload` (modify-reach §9.1) |
| `Cut` whose tool does not span the target | G5, mesh path; future `cupPayload`-shaped pocket |
| `Intersect` with disjoint intervals | G5, mesh path (result is empty; unchanged `BooleanEmpty`) |
| `Union` with a holed operand | G6, mesh path; §9 PR3 |
| A nonidentity re-expression whose arranged boundary is split | §3.4 safety routing, mesh path; a future crossing-sensitivity proof may admit it |
| `Cut` with a holed tool | G6, mesh path; the surviving material inside each tool hole is a separate lump, so it waits on the multi-lump prism payload of the row below, not on PR3 |
| `Cut`/`Intersect` crossing sub-case (§4.2) | resolution unresolved (§3.4), mesh path; §9 PR3 |
| A disjoint-footprint `Union` (two separate lumps) | resolution fails to close one loop (§4.2), mesh path; a future multi-lump prism payload, not currently planned |
| A free-form (Tier A spline) segment that never touches the other operand | excluded by G4 today (whole-scene `TExact` gate, §3.1); worth revisiting once the two-operand scene construction is proven, since the segment itself would ride through untouched |

## 5. Authentication

The task this section answers: `RecordProfile`'s existing authentication
(seam §2.1) matches a caller-supplied `*sketch.Profile` against a **fresh**
`s.Profiles()` result on the caller's *own* sketch. decad's combined scene is
not the caller's sketch — it is one decad built. Authentication here is
**two separate claims**, each answered by an existing mechanism:

1. **Every individual segment is authentic.** Whether reached through the
   clean-nesting structural match or through `Union`'s merge, every recorded
   segment traces to a `sketch.BoundaryEdge` that is itself one of `s.
   Profiles()`'s own results, on the private scene decad just built and
   arranged. The `TExact` contract (seam §1) is a property of the
   *arrangement*, not of who supplied the input curves — `sketch` does not
   know or care that decad authored the scene rather than a caller. So the
   existing `recordEdge`/`falsifyRange` conversion (§4.3) provides full,
   unmodified authentication for this claim.
2. **The assembly is correct.** For the clean-nesting match this claim is
   free — the "assembly" is a single `s.Profiles()` result, unmodified, so
   claim 1 already covers it entirely. For `Union`'s merge, the *additional*
   fact needing proof is that dropping the shared edges and chaining the
   survivors produced a genuinely simple, correctly-wound, properly-nested
   loop — decad's own combinatorial assembly, not a `sketch` answer. This is
   the modify-design carve-out already in force for auditing a
   *decad-constructed* section (modify §4/§5's Table S, and the offset
   audit `shell_offset.go` already runs on ITS OWN constructed section with
   an empty blend map): §6 runs the closed-form crossing/orientation/nesting
   checks the same way, on the merged record, as the second, independent
   proof.

Nothing here computes a *new* geometric fact `sketch` has not already
certified; it re-checks decad's own combinatorial bookkeeping the same way
every modify op already re-checks its own rewrite.

## 6. Build-time audit (Union's merge only)

Reuses the modify §5 machinery verbatim — `crossingAuditBudget`,
`nestingAuditBudget`, and `loopSignedAreaBudget` (`fillet_audit.go`) already
take generic `[]segEntry`/`LoopRecord` shapes with no fillet-specific
coupling, so the merged `ProfileRecord` feeds them directly with an empty
blend map (`shell_offset.go`'s own precedent for "no cutback data, still run
the shared audit"). Order matches modify §4's:

| Step | Check | Sentinel |
|---|---|---|
| assembly | surviving edges chain into exactly one closed loop | not resolved (§4.4), mesh path — not a refusal |
| S8-equiv | assembled loop's signed area does not flip/collapse | `ErrDegenerate` |
| S7-equiv | no non-adjacent segment pair crosses or contacts within the diameter-anchored `contactFloor` band (verification §4's noise floor, reused unchanged) | `ErrUnsupported` |
| S9-equiv | outer loop provably contains every hole (both operands' original holes survive into the result unchanged, since `Union`'s admitted class is hole-free per G6 — this step is a no-op until §9 PR3 relaxes G6's union arm) | `ErrDegenerate` (decidably broken) / `ErrUnsupported` (undecidable) |

## 7. Exactness derivation

Every recorded field, after §4.1's re-expression, is one of:

- **Unchanged from operand A's own record** (A's segments are created
  verbatim into the scene — zero new rounding), or
- **A single rigid-transform recomputation of operand B's own recorded
  field** (§4.1 — one rounding per coordinate; `Intersect`'s shifted interval
  endpoint (G5) is the one other place this design rounds, and it rounds by the
  same rigid-shift mechanism), or
- **`sketch`'s own single-rounded, `TExact`-certified cut coordinate**, for a
  segment the arrangement actually split (recorded as a narrowed
  `TStart`/`TEnd` range on the entity's *own, unchanged* defining data —
  record.go's existing contract: a cut fragment never gets new `Center`/
  `Start`/`End` fields, only a narrower range over the same ones).

When the re-expression is the identity, operand A's fields and every
`sketch`-cut range carry no new rounding from this union. When the
re-expression is nonidentity, §3.4 routes any scene with a `Partial` boundary
edge to the mesh path before it records a fragment, so an analytic result has
no newly cut range whose position could amplify B's coordinate rounding.
Operand B's re-expressed coordinates carry the new allowance `δ_reexpress`.
Either input can already carry a section displacement from an earlier analytic
union, `δ_A` or `δ_B`. The rebuilt section therefore carries
`δ = max(δ_A, up(δ_B + δ_reexpress))`: A's coordinates are unchanged, while
B's prior displacement passes through the rigid map and accumulates the new
coordinate-rounding allowance. `up` rounds the positive sum outward.
It is **exactly zero in one decidable case**: both inputs carry zero displacement
and operand B's composed map into A's frame is the identity in the stored floats
(`frameB == frameA` and `xformB == xformA`, component-wise `==` — G3's own
comparison), where §4.1 copies B's `Point2` fields verbatim and computes nothing
at all. Two caller-drawn profiles on one sketch plane with no placement between
them are that case.

**The evaluator's current measurement path cannot carry `δ`, so this design
extends it.** `prismPayload` holds the section, the frame, the sweep interval,
the placement and its blend descriptors, and no coordinate-error term;
`evalPrism` derives area and volume from the profile integrals and the z
endpoints alone; and those integrals bound only the rounding of their own
arithmetic over the recorded floats, so a line-only record yields a zero area
bound and `exactnessOf` publishes `Exact`. `prismPointBound` carries a
coordinate bound for the centroid POINT only and speaks for no other reading.
Feeding an assembled record through that path unchanged would publish a
displacement-free measurement of a section that carries a displacement. The
extension is two pieces, each in the existing machinery's own shape:

- **`prismPayload` gains a section displacement bound** — the proven upper
  bound on how far any recorded boundary coordinate of the section sits from
  the section its construction denotes. It is zero for every payload built
  today (a plain extrude, a placement, and every modify rewrite record their
  own coordinates), and this design's assembly is the first construction that
  sets it to `δ`. Being a payload field, it re-evaluates with the payload,
  so a placement or copy of an analytically-combined body keeps it.
- **`bounds.go` gains the helpers for the mechanism**, under that file's own
  rule that each error mechanism has exactly one helper and no measurement site
  computes a bound inline. The mechanism's own reading is an AREA: the area a
  boundary displacement `δ` can move is covered by a tube of half-width `δ`
  about the recorded boundary — `2·δ·p + n·π·δ²` up-rounded, for a boundary of
  `n` walks whose proven length upper bound is `p` (a rectangle per walk and a
  disk per joint), which encloses the symmetric difference between the recorded
  section and any section whose coordinates lie within `δ` of it. Its second
  reading is a LENGTH, for the walls: `12·π·δ` per walk, which covers a straight
  walk (whose two moved ends give `2·δ` — `chainLengthBound`'s own reasoning)
  and a circular one alike (whose radius moves by up to `2·δ` and whose swept
  angle moves with its endpoints, so the `2·δ` figure alone understates it).
  `evalPrism` composes those two terms into area (the area displacement once per
  cap, plus the sweep height × the walls' own length displacement), volume (the
  height upper bound × the area displacement), centroid (`δ` in plane, entering
  `prismPointBound`'s existing source term) and `Box` (`δ` outward on every
  face); the same length displacement stands beside every walk's own edge length
  and side-face area, and `δ` beside every junction vertex. The sweep
  interval adds no term of its own for `Union` or `Cut`, whose result interval
  is one operand's own endpoints verbatim (§3.2); `Intersect` may take a
  shifted endpoint, whose G5 rounding is the same rigid-shift mechanism and
  rides in the same `δ`.

The result's `Exactness` is then the existing rule with that displacement as
its one added term:

- **`Exact`, zero bound**, when every surviving segment is a `LineSeg` **and**
  `δ == 0` — `moments.go`'s region-level exact rational accumulator with its
  single final rounding, over a section no coordinate of which was recomputed.
- **`Approximate` otherwise**, carrying the same per-mechanism proven bounds an
  ordinary prism already reports plus the displacement term: the accumulator
  retires the moment any `CircleSeg`/`ArcSeg` survives (no `π` is ever exact),
  and the displacement stands whenever `δ > 0`, whatever the segment kinds
  are.

Volume, Centroid and `Box` all read that same accumulator and that same
displacement term (evaluator §4, `moments.go`), so no separate derivation is
needed for each.

## 8. Consequences removed, and what stands outside the admitted class

| Consequence (§1) | Admitted class | Outside it |
|---|---|---|
| 1. No chaining | Removed. Result is `prismPayload`; no `meshBound` to compose, so no chord tolerance for the next pair to fall below. A chained boolean re-checks §3's gate on the new pair and carries the greater of A's incoming displacement and B's incoming displacement plus its new re-expression allowance (§7). | Unchanged — general-position or non-analytic pairs still degrade per evaluator §9. |
| 2. Coplanar contact refuses | Removed. Coplanar, co-directional contact is the admitted case's whole premise. | Unchanged — non-coplanar or non-prism coplanar contact (e.g. a prism against a revolve cap) stays on the mesh path. |
| 3. Analytic identity dies | Removed where `δ == 0`. Result is `prismPayload`: Fillet/Chamfer/Shell, all three surveys, and the clearance kernel already dispatch on payload class and need zero new code for it. Where `δ > 0`, §12's own rows stage the readings that have no place to put a displacement. | Unchanged for mesh-path results — `facetedPayload` still permanently refuses modify ops (modify-reach SX9) and all three surveys. |

## 9. Refusals

Entry-gate misses (§3.1's G1–G6) and unresolved region-topology (§4.4) are
**not refusals** — no error, silent fallback. The table below is exclusively
the stage-2 refusals (§3.4), each stated once with its
sentinel, decided by the existence test (modify §1: no such body exists is
`ErrDegenerate`; a body this evaluator cannot build is `ErrUnsupported`) and
whether it is permanent.

| Row | Condition | Sentinel | Permanent? |
|---|---|---|---|
| RB1 | A candidate region (or one the merge depends on) reports `Profile.Valid == false` — the arrangement is degenerate or self-intersecting where it reaches a needed cell | `ErrUnsupported` | No — a differently-shaped input to the same op may resolve |
| RB2 | `Union`'s surviving edges do not chain into exactly one simple closed loop after the assembly step commits (distinct from §4.4's *unresolved*: this is a resolved-but-broken assembly) | `ErrUnsupported` | No |
| RB3 | §6's S8-equivalent: assembled loop's signed area flips or collapses | `ErrDegenerate` | No — depends on operand geometry |
| RB4 | §6's S7-equivalent: a non-adjacent pair crosses, or contacts within the diameter-anchored noise floor | `ErrUnsupported` | No |
| RB5 | §6's S9-equivalent, decidably broken (a hole proven outside the outer loop or nested wrong) | `ErrDegenerate` | No |
| RB6 | §6's S9-equivalent, undecidable | `ErrUnsupported` | No |
| RB7 | The bounded work budget (§10) exhausts, or the private scene exceeds `prismUnionMaxArrangementSegments`, before resolution or the audit completes | `ErrUnsupported` | No — a coarser/simpler input may clear it |
| RB8 | `recordEdge`/`falsifyRange` rejects a surviving segment (`TExact` disproven on a merged edge — an internal `sketch` inconsistency, reported upstream per seam §3) | `ErrUnrecordableProfile` | No, but should not occur on a certified arrangement; a defensive check |

None of these rows is permanent in the modify-reach SX9 sense: every one is a
property of the specific pair's geometry, not a structural class exclusion —
a differently-authored model of the same intent can clear it.

## 10. Work budget and cancellation

Reuses `budget.go`'s existing `workBudget` (`newWorkBudget(ctx)`), the same
shared counter Fillet/Chamfer/Shell already thread through their own audits
(CLAUDE.md's "Modify audit cancellation"). PR1 opens one counter per
`tryPrismUnion` attempt and threads it through: the G4 segment scan, scene
construction (one charge per created entity), the §4.2 selection/merge walk
(one charge per candidate edge/cell touched, matching `crossingAuditBudget`'s
own per-pair charge shape), and §6's audit (its existing budget parameter,
unchanged). Polled at phase boundaries and at least every 256 candidates,
identical to the existing pattern. `prismUnionMaxArrangementSegments` is the
single code-owned pre-`Profiles` cap: it bounds the private arrangers'
tiny-segment pair work for the pinned line/circle/arc density. `sketch.Sketch.
Profiles` is synchronous and has no context parameter, so a capped private
scene runs in one worker while the caller selects its result against
`ctx.Done()`. Cancellation returns before that arrangement reaches the
document; its buffered result is discarded after the worker finishes.
`UnionContext`/`CutContext`/`IntersectContext` add no public cancellation
surface. Exhaustion is RB7 (§9), not silent truncation.

## 11. Topology, provenance, and roles

The result is built by `evalPrism` over the merged record, under the boolean
step's own `StepRef` — identical to how Fillet/Chamfer/Shell already build, and
the topology it builds is untouched by §7's added displacement term. Faces
therefore get **fresh** roles
(`side(i,j)`/`capStart`/`capEnd`) minted from the merged record's own segment
positions, per decad's already-settled role rule (modify §9/§12: "a role
indexes the record it labels ... never inherited"). **Decision: the analytic
path does not carry `Face.Origins()` provenance forward from either
operand** — overturning the earlier investigation's lean toward the mesh
boolean's behavior (`.tmp/boolean-redesign/README.md` §3.B, "Provenance").
Reason: this construction is a section rewrite in exactly modify-design's
sense (a section derived from a record, rebuilt through `evalPrism` under a
fresh step), not the mesh boolean's face-patch grouping over surviving
operand facets — the two constructions are not the same shape, so extending
the mesh boolean's *different* provenance rule to this one would be
inconsistent with decad's own settled precedent, not a continuation of it. A
caller that needs to re-select "the tooth's faces" after the union selects
by geometric predicate (`Circular`, `LongerThan`, `NormalTo`) rather than by
origin, exactly as it already must after a Fillet or Chamfer. Flagged in
§13 as a decision the user may want to overturn.

## 12. Downstream consumers

| Consumer | Effect |
|---|---|
| Tessellation | No new code — the result is an ordinary `prismPayload` and `docs/tessellation-design.md` §5's existing prism contract applies. §7's section displacement rides in that contract's own stored-coordinate rounding term (tessellation §5's prism row), so a mesh of an assembled body is `Exact`-trimmed only where `δ == 0`. |
| Clearance kernel | Unchanged where `δ == 0` — dispatches on payload class; `prismPayload` already has full analytic support (`clearance.go`'s coplanar `Plane`×`Plane` certificate, `offsetPair`, etc.). Where `δ > 0` the kernel builds no model for the body and the pair reads `Suspect`: every certificate it emits is an exact statement about the carriers it read, and a carrier the payload holds only within `δ` of the one it denotes cannot support one. Widening the kernel's own candidate intervals by `δ` is a separate piece of work, not this design's. |
| Interference (`Verify`) | **Still on the mesh path.** `interference.go`'s `measuredInterference` calls `evaluateBoolean(ctx, OpIntersect, ...)` directly after its containment and represented-set-equality certificates. The analytic dispatch is in `performBoolean`, which this PR implements only for `Union`, so an admitted coplanar-prism pair still reaches the existing read-only mesh intersection and may be coarse or `Suspect`. PR4 separately wires a read-only analytic `Intersect` path and its tests. `docs/interference-design.md` §5.2 records this PR1 boundary. |
| Surveys (wall/undercut/min-radius) | No new code — they dispatch on payload class, and support is immediate where `δ == 0`. The undercut reading is a normal-direction membership and is unaffected at any `δ`. The wall and min-radius readings are staged at `δ > 0` and answer undecided (`Suspect`, never a silent pass): each publishes a bare reading with no bound beside it, and the wall reading is not a quantity a displacement widens by a fixed amount anyway — its allowance-angle contact families (verification §6) can change membership under a boundary perturbation, so a proven displaced reading needs the survey's own theory extended, not a term added to a bound. |
| `Verify`'s structural/tolerance gates | Unchanged — `prismPayload` is valid by construction as always. |
| Export (STL/OBJ) | Unchanged — reads `Tessellate`'s output. |
| Recipe/replay | **No wire change.** The step still records the existing `OpUnion`/`OpCut`/`OpIntersect` + `Inputs` (`[a, b]` or `[target, tool]`), unmodified — recipe-replay-design §8's own contract already allows this: "A later evaluator MUST reproduce ... one produced body per step ... measurements valid under its own `Exactness`/`Bound`. It need not reproduce v1's internal payload." A replayed recipe simply builds via the analytic path wherever it now qualifies; nothing in §2 (wire envelope), §3 (validation), or §4 (references/liveness) changes. |

## 13. Decisions the user may want to overturn

- **§3.1 G3's exactness is unconditional** — no tolerance admits a
  near-but-not-exactly coplanar pair, even though `sketch`'s own carrier
  merge (§4.1) uses a round-off band internally. Loosening G3 would let a
  caller's placement rounding silently decide whether a pair routes through
  the analytic or mesh path with different exactness claims for
  visually-identical models; kept strict on the reject-only rule, at the cost
  of `RotationAround`'s known under-triggering (§3.3).
- **§11's provenance decision** — fresh roles only, no inherited
  `Face.Origins()`, overturning the investigation's lean toward the mesh
  boolean's behavior. Reason given in §11; the alternative (thread operand
  origins through the merge, tagging each surviving segment with its source
  operand's `FeatureRef`s) is a bounded, separable addition if a consumer
  needs it later — it does not change §4's selection logic, only what
  `addBlendRoles`-equivalent code stamps onto the built faces. **Accepted**: a
  `FaceCreatedBy` selector that survives the mesh boolean does not survive
  this one, and that is the agreed cost.
- **A nonzero `δ` stages the readings §12 names, and every modify op.** A
  modify op rewrites the receiver's recorded section (modify §2), and a blend
  centre or an offset miter amplifies a displacement by the corner geometry —
  by an unbounded factor at a shallow corner — so `Fillet`/`Chamfer`/`Shell`
  refuse a displaced receiver (`ErrUnsupported`) rather than publish a result
  whose bounds speak only for the record. Every one of these stagings is
  invisible at `δ == 0`, which covers every body a caller draws and every
  union whose sources and re-expression carry zero displacement.
- **§3.1 G6 restricts `Union` to hole-free operands and `Cut` to a hole-free
  tool.** The union restriction is increment 1/2's, deferring the general
  per-cell classification (§4.2's crossing sub-case) to PR3 even though it is
  fully specified here; chosen to ship the gear's actual workload (hole-free
  hub+teeth union, clean-nesting bore cut) without waiting on the harder
  general case. The tool restriction outlives PR3: a holed tool leaves a lump
  per tool hole, so it waits on the multi-lump prism payload §4.4 names, and a
  consumer that wants that cut analytically can express the same intent as one
  `Cut` per hole-free tool region.

## 14. Increments

1. **PR1 — gate + scene construction + `Union`'s select-all path.** G1–G6,
   §4.1's scene builder and re-expression, `Union`'s select-all/merge/chain,
   §6's audit reuse, §5's authentication, §7's exactness (the section
   displacement bound on `prismPayload`, its one `bounds.go` helper, and
   `evalPrism`'s composition of it), and `performBoolean`'s branch before
   `evaluateBoolean` to build via `evalPrism` instead of `buildFacetedBody`
   on admission.
   Tests: a two-box union sharing a cap plane (the "control" case from the
   consumer's report) builds analytically, with `Exact` volume where both boxes
   sit on one frame (§7's `δ == 0` case); the gear's tooth-on-hub
   shared-carrier union builds with the correct region set and an
   `Approximate` volume within the composed bound of the closed-form Pappus/
   Green's-theorem answer; a non-coplanar pair still takes the mesh path
   unchanged (regression).
2. **PR2 — `Cut`/`Intersect` clean-nesting sub-case.** The structural
   whole-loop match (§4.2), reusing PR1's scene infrastructure and §5's
   direct `RecordProfile` authentication (no new authentication code). Tests:
   a bore cut through a hole-free hub (the F1 workload) builds analytically
   with the correct hole added and the target's original outer loop
   byte-identical to its pre-cut record; an `Intersect` of a fully-nested pair
   returns the inner operand's own geometry.
3. **PR3 — general per-cell classification.** §4.2's edge-orientation
   propagation and coincident-carrier detection, relaxing G6's union arm
   (holed unions; its `Cut` tool arm stands, §13) and admitting the crossing
   sub-case for `Cut`/`Intersect`. Tests: two overlapping (not
   coincident-carrier) prisms union/cut/intersect correctly
   against the mesh path's own answer on the same pair (property test,
   volumes agree within the analytic path's tighter bound); a holed hub
   unions with a tooth correctly, holes preserved.
4. **PR4 — replay/interference wiring + docs.** Confirm recipe replay against
   a stored `OpUnion`/`OpCut`/`OpIntersect` step builds via the analytic path
   post-upgrade with no wire change (recipe-replay-design §8's contract,
   §12 above); wire `interference.go` to the read-only analytic `OpIntersect`
   path with a test
   asserting an exact overlap volume on an admitted coplanar-prism pair that
   meets §7's exactness conditions;
   `docs/interference-design.md` §5.2 gets a one-line pointer noting the
   prism case is superseded here. `docs/evaluator-design.md` §9 gets a
   pointer to this document (§ done alongside this design, see the CLAUDE.md
   diff).

## 15. Required tests

Every capability below asserts on computed geometry (coordinates, volumes,
areas, residuals), never merely "it ran" — CLAUDE.md's own rule.

- G1–G6 each have a dedicated fallback test: a pair that fails exactly that
  gate still builds via the (unchanged) mesh path, with the SAME error/result
  shape as before this design existed. G6 needs one per arm — a holed `Union`
  operand and a holed `Cut` tool — and the holed-tool test additionally
  asserts the mesh result keeps the material standing inside the tool's hole,
  which is the body the analytic path would have dropped.
- The rotated-tooth case (§3.3): a placement built via `RotationAround` at
  `n = 17` correctly falls back to the mesh path (G3 miss, no error). The
  same model built via a hand-constructed `FromBasis` placement clears G3 but
  falls back on §3.4's re-expressed split-boundary rule. The test must cover
  several counts from §3.3's inexact set rather than `n = 17` alone, since the
  inexact counts are the majority and a single-count test reads as though they
  were rare.
- Two operands whose frames were constructed independently but denote the
  same plane (§3.3's `Frame.N()` reading): the test records what G3 does with
  them. A refusal there is sound under the reject-only rule and is not
  repaired by loosening the comparison.
- Coincident-carrier union: assert the merged record's boundary uses the
  hub's own circle for the majority arc and the tooth's own root arc for the
  shared span (not a fabricated third geometry), and that the reported volume
  matches the closed-form sum of the two prisms' own volumes within the
  composed bound.
- Clean-nesting cut: assert the result's outer loop is byte-identical
  (`Point2` fields, not just area) to the target's own pre-cut outer loop,
  and the new hole's fields are byte-identical to the tool's own pre-cut
  outer loop (verbatim reproduction, §4.2's structural-match claim).
- Exactness, one test per §7 arm: a line-only merged section over operands
  that share a frame reports `Exact` volume with a zero bound; the same
  section over an operand B with a nonidentity re-expression but no split
  boundary reports `Approximate` with a bound the §7 displacement term alone
  explains (assert it scales with the placement's own magnitude, so a payload
  silently dropping the term fails);
  a merged section retaining a `CircleSeg`/`ArcSeg` reports `Approximate` with
  a bound composed from `moments.go`'s and `bounds.go`'s machinery, asserted
  against the closed-form answer.
- A nonidentity re-expression whose arranged profile contains a `Partial`
  boundary edge falls back before a fragment is recorded. The focused fixture
  must prove that the arrangement splits, then assert that `tryPrismUnion`
  returns `ok == false` without an analytic-resolution error.
- Downstream chaining: fillet a corner of an analytically-unioned body and
  read `MinWallThickness` on the result — both refuse today (SX9, all three
  surveys) on a mesh-path union of the same model, and both succeed here.
- A second boolean consuming the first's result as operand B under a
  nonidentity re-expression carries B's prior displacement plus its new
  allowance — assert `Exactness`/`Bound` on the second result follow §7,
  not accumulated tessellation tolerance.
- Cancellation: a canceled `ctx` mid-resolution returns `ctx.Err()` unchanged
  with the document and recipe untouched, matching the existing modify-op
  contract.
- Arrangement cap: two line-only prism records whose combined upper bound
  exceeds `prismUnionMaxArrangementSegments` refuse with `ErrUnsupported`
  before `s.Profiles()` runs.
- Replay: encode a recipe whose `OpUnion` step is an admitted pair, decode
  and evaluate it fresh, and assert the replayed body's `Exactness`/`Bound`
  match direct construction (recipe-replay-design §10.3's shape).
