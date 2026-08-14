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
handle wherever its section displacement is zero (§7). §8 states exactly which
of the three consequences above disappear for the admitted class, which stand
outside it, and where a nonzero displacement's cost per consumer is decided.

## 2. Approach

**Chosen direction.** PR1 dispatches a reject-only admission gate from
`performBoolean` for `Union`, ahead of the shared `evaluateBoolean` mesh
pipeline. An admitted pair combines its two operands' `ProfileRecord`s through
a private `sketch` scene decad builds. It records the selected result regions
through the existing seam (`RecordProfile`/`recordEdge`), audits the assembly
with `modify §5`'s existing closed-form checks, and rebuilds through
`evalPrism`. Section 7 extends the result with a section-displacement bound so
the re-expressed operand's rounding and the cut parameters `sketch` computes
for surviving fragments reach every measurement. It also preserves each sweep
end's axial displacement so every measurement keeps the bounds its operands
already proved.
`Cut` and `Intersect` remain on the mesh path until later increments. A
non-admitted `Union` pair — wrong payload class, non-coplanar, a segment kind
outside the admitted set, an unequal z-interval for `Union`, a nonidentity
re-expression or prior source displacement whose arranged boundary is split,
or a topology this increment's region resolution does not cover — takes the
unchanged mesh path, with **zero
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
   accepted when `sketch` returns any `Partial` boundary edge and either
   source carries a nonzero section displacement, either source carries a
   nonzero walk charge (§7's `δ_walk` — a consumed segment whose own recorded
   range narrows its natural domain), or B's re-expression is nonidentity:
   any of the three can move a transverse cut by its displacement divided by
   the crossing sine, and this increment carries no certified
   crossing-sensitivity bound. Every other capacity, arrangement,
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

- Operand A's frame is the reference (`target`'s, for `Cut`). Every entity —
  A's and B's alike — is created from its own segment's **walked** geometry
  (`walkOf`, extrude.go), never from the record's `Point2` fields directly:
  this is the same reduction `momentRecordScene` uses for a single record's
  own self-consistency check, and it is why a `Partial` fragment is recreated
  over its own WALKED portion rather than its source curve's whole extent
  (buildPrismScene's own doc comment). A segment whose recorded range is the
  entity's own full domain walks to the record's own coordinates verbatim —
  `lerp2` and `pinArcWalkEnds` both special-case the natural bounds `t=0`/
  `t=1` to return the record's own `Point2`, and a `CircleSeg` recorded over
  those same bounds walks the recorded centre and radius directly — so
  operand A's segments
  reach the scene with zero new rounding wherever every one of them is
  WHOLE. A segment recorded over a **narrowed** range instead evaluates the
  carrier at a **computed** parameter, so it enters the scene at an endpoint
  this boolean computed, not one the record states — charged as §7's
  `δ_walk`. A trimmed circular carrier (`ArcSeg`/`CircleSeg`) moves by more
  than its two endpoints — its rebuilt radius and sweep move too, since the
  scene's arc is built through cos/sin-computed points — so this increment
  refuses a pair carrying one before the scene is even built
  (`prismProfileHasTrimmedCircularSource`) rather than publish an
  under-charged bound for it; the routing is the same silent §3.4
  fallback as every other entry-gate miss.
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
  is the only new rounding this design commits on an operand's own INPUT
  coordinates: an ordinary rigid-transform coordinate computation, rounded once
  per coordinate, on operand B's segments only — and where B's frame and
  placement ARE A's own in the stored floats (component-wise `==`, G3's own
  comparison) the composed map is the identity, B's `Point2` fields are copied
  verbatim, and nothing is computed at all. It is **not** the only new rounding
  the design introduces: the cut parameters `sketch` computes for the surviving
  fragments are new rounded coordinates of their own, whatever the inputs
  carried, and §7 charges them separately.
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
(same entities, same order, every edge `Whole`) — for `Cut`, whose `Holes`
additionally reproduce target's original holes plus **one new hole** that
structurally reproduces tool/B's own `Outer` (also every edge `Whole`; G6
keeps the tool hole-free, so that one hole is the tool's whole solid and no
material inside a tool hole is dropped); for `Intersect` with B fully inside
A, the match is simply B's own disk cell, `Outer` reproducing B's original
loop. A structural match — entity identity, order, and `Whole`-ness, nothing
geometric — is a pure data comparison against decad's own tag map. **When a
unique such profile exists, it is not assembled at all: it is one of
`s.Profiles()`'s own results, and is authenticated by handing it directly to
the existing public `RecordProfile(s, profile)` — the full seam (§5) applies
unmodified, no new authentication code.**
This reproduction is **byte-identical to the matched operand's own pre-cut
record — same `Point2` floats — only where every one of that operand's own
consumed segments is itself WHOLE.** `Whole = true` in THIS arrangement says
only that nothing in THIS cut trimmed the scene's own entity further; it says
nothing about whether the segment ENTERED the scene at the record's own
coordinates or at a walked endpoint an EARLIER construction already computed
(§4.1, §7's `δ_walk`). A target whose own record already carries a
narrowed-range `LineSeg` — the recorded segment of a body that is itself a
prior analytic result — is walked to that narrowed endpoint before its
scene entity is even built, so this cut's own "verbatim" match reproduces
the SCENE entity's endpoint, not the target's original, wider one: a
`LineSeg{(1,0)→(11,0), t=[0,0.4]}` denoting corner `u = 5.0000000000000002`
returns from a clean-nesting cut as `LineSeg{(1,0)→(5,0), t=[0,1]}` — a
different, computed corner recorded as though it were exact. §7's `δ_walk`
is what charges that gap into the result's own `sectionDelta`.

**`Cut`/`Intersect`, crossing sub-case (boundary contact, not clean nesting):
implemented for the direct-edge case (`prism_boolean_crossing.go`); the
coincident-carrier extension below stays staged (§4.4).** When operand B's
boundary genuinely crosses operand A's (a bore that pokes outside the hub, or
a general-position overlap), neither `Union`'s select-all rule (only sound
because `Union` wants *every* cell) nor the clean-nesting structural match
(which requires every edge `Whole`) applies. The mechanism is
**edge-orientation propagation**: for a cell touching a boundary edge sourced
from operand X, compare that edge's `Reversed` flag (in this cell's own walk)
against X's recorded authored orientation for the loop that entity came from
(record.go's "outer CCW, holes CW"); a match means the cell sits on X's
material side at that edge (X's authored convention already puts material on
the walk's left), a mismatch means X's void side. This is a **flag
comparison**, not a geometric test — the classification `sketch` already
encodes in which direction it walked a shared entity for this specific cell.
`prismEntityOrigin.authoredReversed` (`prism_boolean.go`'s `buildPrismScene`)
records, once per created entity at scene-build time, whether that operand's
own recorded walk runs backwards relative to the entity's own natural
parameterization, so the comparison above is bookkeeping on a fact already
captured, never a fact `classifyPrismCells` derives from geometry. For a cell
with a direct edge from both operands, both readings are available and the
four combinations (`A-only`, `B-only`, `both`, impossible-for-two-simple-
curves-`neither`) select `Union`'s "either", `Cut`'s "A and not B",
`Intersect`'s "both". A cell with no direct edge from an operand takes that
operand's own classification from a neighboring cell reached by crossing an
edge that does NOT belong to that operand — crossing a non-operand edge
cannot move across that operand's own boundary, so membership carries over
unchanged; `classifyPrismCells` floods this from every directly-classified
cell.
For a cell touching only one operand directly but adjoining a **coincident
carrier** (the tooth-only cell of the gear's shared-arc case, which the
merged edge names under only one operand's entity) — decad would additionally
need to identify, from its **own** recorded segment data (comparing
`Point2`/radius fields for exact equality after §4.1's re-expression — the
same discipline `momentRecordScene`'s dedup key already applies, an algebraic
comparison of decad's own numbers, never a geometric one), which of its own
segments across the two operands describe the same carrier, and derive the
fixed relative orientation between the two operands' authored senses for that
shared carrier once, structurally, from their own recorded windings — never
from evaluating a point against either curve. This extension is not yet
built: a cell reachable only through a coincident carrier is unresolved today
(§4.4), the same outcome as any other cell the propagation above cannot
reach at all (isolated from every operand boundary by an unclassified path).

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
| A split arranged boundary with a nonzero source displacement or a nonidentity re-expression | §3.4 safety routing, mesh path; a future crossing-sensitivity proof may admit it |
| `Cut` with a holed tool | G6, mesh path; the surviving material inside each tool hole is a separate lump, so it waits on the multi-lump prism payload of the row below, not on PR3 |
| `Cut`/`Intersect` crossing sub-case reachable only through a coincident carrier named under one operand's own entity (§4.2's own further extension) | not yet built (`prism_boolean_crossing.go`), mesh path |
| A holed `Cut` target whose tool does not clear it via clean nesting (the crossing classifier is scoped to hole-free operands on both sides, §4.2) | mesh path; the clean-nesting path above still covers a holed target whose tool does not touch it |
| A `Cut`/`Intersect` selection covering two or more disjoint regions (a multi-region coplanar overlap) | no BODY is built: a `ProfileRecord` carries one outer loop, so resolution fails to close one (§4.2) and the pair takes the mesh path, waiting on a multi-lump prism payload that is not currently planned. `Verify`'s interference reading answers such a pair anyway, without a body, through §4.5's overlap-area reading |
| A disjoint-footprint `Union` (two separate lumps) | resolution fails to close one loop (§4.2), mesh path; a future multi-lump prism payload, not currently planned. The limitation is the public boolean's alone: the two operands share no interior, so `Verify` answers the pair through its disjoint/overlap partition (`docs/verification-design.md` §1) rather than through §4.5 |
| A free-form (Tier A spline) segment that never touches the other operand | excluded by G4 today (whole-scene `TExact` gate, §3.1); worth revisiting once the two-operand scene construction is proven, since the segment itself would ride through untouched |

### 4.5 The overlap-area reading: a volume, never a body

`Verify`'s interference reading needs exactly two things from a pair: the
verdict that the two solids provably overlap, and a bounded volume for that
overlap (`docs/interference-design.md` §1, §6). It does not need a `Body`.
Every limit §4.4 states for a multi-region `Cut`/`Intersect` is a limit on
ASSEMBLING one — a `ProfileRecord` carries a single outer loop, so a selection
covering two or more disjoint regions cannot become a section at all, and
`chainPrismUnionSurvivors` correctly reports it unresolved. This reading
answers the volume without assembling anything, so a pair whose 2D outlines
overlap in several disjoint regions — a pair of meshing gears, which always
engages several teeth at once — is measured rather than left undecided.

The reading runs only from the read-only interference path, never from
`performBoolean`. `Union`, `Cut` and `Intersect` keep the behaviour §4.4
states, multi-region results included: a public boolean owes its caller a body,
and this reading builds none. §13 records that asymmetry as a deliberate
decision.

**Entry.** Identical to `Intersect`'s own, and shared with it unchanged: G1–G4
(`admitPrismPairBudget`), the trimmed-circular refusal
(`prismProfileHasTrimmedCircularSource`), G6's hole-free arms, G5's Intersect
z-relation (§3.2), the arrangement cap, the re-expression, `buildPrismScene`,
§3.4's split-boundary reroute, `classifyPrismCells`, and `selectPrismCells`
under `Intersect`'s own `keep`. The reading begins where `mergePrismCells`
would have been called, and replaces only that tail.

**Selection.** The measured set is the arrangement's own bounded cells that
`classifyPrismCells` puts on BOTH operands' material sides. `sketch` returns
the arrangement's full planar decomposition (§4.2), whose cells have disjoint
interiors, so the selected set's total area is the sum of its cells' areas and
no cell is counted twice. That disjointness is `sketch`'s own answer about its
own arrangement, consumed as CLAUDE.md's carve-out allows — decad computes no
containment, no crossing and no membership of its own. The existing
reject-only structural checks inside `classifyPrismCells` are what falsify it:
a cell reporting `Valid == false`, a cell carrying its own hole, an edge
occurring on more than two cells, or an entity the scene did not create each
leave the whole reading unresolved.

**Per-cell measurement.** Each selected cell is already a closed directed walk
in `sketch`'s own order, so it is recorded through the existing
`recordEdge`/`edgeJoin`/`falsifyLoopJoins` sequence `mergePrismCells` already
uses — minus the count, drop and chain steps, which exist only to build one
loop out of many. §5's authentication applies with its claim 2 discharged for
free, exactly as it is for the clean-nesting match: every recorded edge is one
of `s.Profiles()`'s own results and nothing is assembled from them, so §6's
build-time audit has no assembly to re-prove and does not run.

The recorded cell becomes a `prismPayload` carrying the pair's own frame,
placement and §3.2 Intersect z-interval, that cell's own §7 section
displacement, and the pair's per-end axial displacement. Its volume is then
`evalPrism`'s, unchanged — the same region integrals, the same
`sectionDisplacementArea` charge over the cell's own perimeter, the same axial
terms, the same `Exactness` rule. No bound mechanism is restated at a second
site, which is what `bounds.go`'s one-helper-per-mechanism rule requires, and
no new helper is introduced.

<!-- "a volume, never a body" is scoped to ASSEMBLY and DELIVERY, not to
     allocation: this section's own wording is "a limit on ASSEMBLING one",
     "without assembling anything", "nothing is assembled from them", and "it
     assembles nothing". The paragraph above names `evalPrism` as the per-cell
     volume path outright, and `evalPrismContext` (extrude.go) does allocate a
     `Body` and read `body.volume` from it — that is the specified path, not a
     contradiction of it. What the reading never does is commit a `Step`,
     retire an operand, register a body in the `Document`, or return one to a
     caller; `docs/interference-design.md` §5 states that promise for this
     reading beside the existing analytic twin's. -->

**The sum.** The reading publishes the charged sum of the cells' volumes: the
value is their float sum, whose own accumulated rounding `bounds.go`'s
`exactSumRound` charges against the exact rational sum, and the bound is
`absSumUpper` over the cells' own bounds and that rounding. `exactnessOf` then
reads the summed bound, so the reading is `Exact` only where every cell is —
which a crossing selection never is, since a genuine crossing splits at least
one edge and §7's `δ_cut` is positive on every split fragment. A multi-region
overlap therefore reads `Approximate` over a bound whose whole content is the
cut parameters' own rounding.

**What the reading declines.** An empty selection — the exactly-tangent pair,
whose arrangement puts no cell on both operands' material sides — is
unresolved: the reading answers nothing and the pair falls back to the mesh
path, whose coplanar refusal leaves it undecided, unchanged. So is a selected
cell whose own section the region integrals refuse as degenerate or
unsupported. The reading never publishes a zero-volume overlap and never turns
a contact into a row; `docs/interference-design.md` §6's positive-volume gate
judges what it does publish, unchanged.

**Refusals.** The reading crosses no point of no return: it assembles nothing,
so §9's RB2–RB6 cannot arise, and every shape it does not cover is the silent
fallback above. It returns only the errors every path already shares — the
arrangement cap (RB7, `ErrUnsupported`) and the caller's own cancellation —
plus RB8/RB9 on a cell `recordEdge` rejects or a cell loop that does not join,
surfaced exactly as they are on the body path so a broken arrangement is never
hidden as an undecided pair.

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
   proof. Neither of those checks proves the merged loop's own **closure**.
   `chainPrismUnionSurvivors` resolves which survivor follows which in
   `sketch`'s own arranged polyline order — a fact about the arrangement, not
   about what the record states — and §6's crossing check skips adjacent
   segment pairs by construction, since an adjacent pair sharing a junction
   is expected to touch there. The merged loop's closure is instead proven on
   the RECORDED coordinates themselves, the same way `recordLoop` proves it
   for any caller-supplied loop: the seam's existing junction falsifier
   (seam §3, `edgeJoin`/`falsifyLoopJoins`) runs on the merged chain before
   the assembled `ProfileRecord` is returned.

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
| closure | the merged loop's own RECORDED coordinates join at every junction (seam §3's `edgeJoin`/`falsifyLoopJoins`, unmodified), run after the chain resolves — past §3.4's point of no return | `ErrUnrecordableProfile` (RB9), a refusal, not a reroute — the same treatment RB8 already gives a `recordEdge` rejection at this stage |
| S8-equiv | assembled loop's signed area does not flip/collapse | `ErrDegenerate` |
| S7-equiv | no non-adjacent segment pair crosses or contacts within the diameter-anchored `contactFloor` band (verification §4's noise floor, reused unchanged) | `ErrUnsupported` |
| S9-equiv | outer loop provably contains every hole (both operands' original holes survive into the result unchanged, since `Union`'s admitted class is hole-free per G6 — this step is a no-op until §9 PR3 relaxes G6's union arm) | `ErrDegenerate` (decidably broken) / `ErrUnsupported` (undecidable) |

## 7. Exactness derivation

Every recorded field, after §4.1's re-expression, is one of:

- **Unchanged from operand A's own record where every one of A's consumed
  segments is WHOLE** (a whole segment's walk restates the entity's own
  defining data exactly, §4.1 — zero new rounding), or
- **One of A's own segments' WALKED endpoint**, where that segment's own
  recorded range narrows the entity's own natural domain — a coordinate this
  boolean COMPUTED rather than one the record states, charged as `δ_walk`
  below, or
- **A single rigid-transform recomputation of operand B's own recorded
  field** (§4.1 — one rounding per coordinate; `Intersect`'s shifted interval
  endpoint (G5) is the one other place this design rounds, and it rounds by the
  same rigid-shift mechanism), composed with B's own walk charge the same way
  operand A's is, or
- **`sketch`'s own single-rounded, `TExact`-certified cut coordinate**, for a
  segment the arrangement actually split (recorded as a narrowed
  `TStart`/`TEnd` range on the entity's *own, unchanged* defining data —
  record.go's existing contract: a cut fragment never gets new `Center`/
  `Start`/`End` fields, only a narrower range over the same ones). It is a
  coordinate this union COMPUTED, so it is charged, as `δ_cut` below.

Four separate things can displace the rebuilt section, and the result carries
all four.

**The re-expression, `δ_reexpress`.** Operand B's re-expressed coordinates carry
it; operand A's are unchanged and carry nothing.

**An input's own prior displacement, `δ_A` or `δ_B`.** Either input can already
carry one from an earlier analytic union. A's passes through unchanged; B's
passes through the rigid map and accumulates `δ_reexpress` on top.

**The cut parameters, `δ_cut`.** A surviving `Partial` fragment records the
entity's own unchanged defining data and the narrowed `TStart`/`TEnd` range
`sketch` computed for THIS pair. That range is a **freshly rounded coordinate**:
it names a crossing whose true parameter `t*` is a real number `sketch` had to
round to a float, and every consumer that reads the fragment's endpoint
evaluates the carrier at the rounded `t`, not at `t*`. Nothing the two operands
carried says anything about it — a pair of exactly-drawn, unplaced boxes commits
this rounding as surely as any other pair does. So `δ_cut` stands on its own,
and an identity re-expression over two zero-displacement operands does **not**
send it to zero.

Its size is the parameter allowance times how fast the carrier moves under it.
The carrier is exact, so the endpoint can only slide ALONG it:
`|P(t) − P(t*)| ≤ |t − t*| · sup|dP/dt|`. `bounds.go`'s `cutParamUlps` states
the parameter allowance once — the quantitative reading decad gives `TExact`'s
"to machine precision" claim (`docs/sketch-seam-design.md` §1) — and
`cutDisplacementAllow` multiplies it by the carrier's own speed over its full
parameterisation: the chord for a line, `2πR` for a circle or an arc.
`δ_cut` is the largest such allowance over the surviving fragments, and zero
when every survivor is a whole edge.

**The walk charge, `δ_walk`.** `buildPrismScene` builds every entity from its
segment's own WALKED geometry (`walkOf`, §4.1), never from the record's
`Point2` fields directly. For a segment whose recorded range is the entity's
own full domain that walk restates the record's own coordinates exactly —
`lerp2`'s and `pinArcWalkEnds`' own natural-bound special cases, and a
`CircleSeg` recorded over those bounds walking the recorded centre and radius
directly — so a WHOLE segment charges nothing, whatever its kind. A `LineSeg`
recorded over a range NARROWER than that domain instead evaluates the carrier
at a computed parameter (`lerp2`'s general arm), so it enters the scene at an
endpoint this boolean itself computed, whatever the two operands' own prior
displacement was. It is the only kind that reaches this charge: a trimmed
circular carrier would enter through two `cos`/`sin`-computed points, moving
its rebuilt radius and sweep as well as its endpoints, and §4.1 refuses such a
pair before the scene is built rather than charge it (below). `bounds.go`'s
`walkEndpointAllow` states the allowance, at the magnitude of the operands
that walk's OWN arithmetic touches and never at the endpoint it produced:
`lerp2` computes
`fl(a + fl(t·fl(b−a)))` from the carrier's own recorded `Start` and `End` — at
most that, since a target free to fuse the multiply and the add commits one
rounding fewer — and that difference CANCELS, so a fragment near the plane
origin on a far-reaching carrier rounds by ulps of the CARRIER while its own
endpoint magnitude stays tiny. A trimmed `LineSeg` therefore charges the
envelope of its recorded `Start`/`End` coordinates (`walkChargeOf`'s
`lineWalkOperandUpper`), folded together with the walked endpoint so the
envelope stands whatever the recorded parameter is. `walkChargeOf` still
states a circular answer — `segmentWalk.coordUpper`, whose `|cu|+|cv|+r+r` L1
form already bounds the centre and radius a `cos`/`sin` walk works on — but
the §4.1 refusal means no boolean reaches that arm; it stands so a charge,
never a silent zero, is what any future widening of that refusal would meet.
Each operand owes the largest such allowance over its OWN
consumed segments — `δ_walkA` and `δ_walkB`, the `a` and `b` fields of
`prismSceneDelta`, each holding that operand's walk charge ALONE — and the
charge stands even when both operands carry zero displacement and the
re-expression is the identity — the same
independence `δ_cut` already has, one construction earlier: `δ_cut` charges
the crossing `sketch` computes for THIS pair, `δ_walk` charges an INPUT
segment's own narrowed range entering the scene at all, before any crossing
is asked about. A trimmed circular carrier (`ArcSeg`/`CircleSeg`) is refused
rather than charged (§4.1) — its rebuilt radius and sweep move too, which a
coordinate-envelope charge does not state — so a positive `δ_walk` is only
ever computed over a trimmed `LineSeg`. Every circular carrier that survives
§4.1 is WHOLE and therefore charges zero, whichever field holds which bound
(Reversed swaps them, so a WHOLE arc's `TStart`/`TEnd` are `{0, 1}` in either
order — see `walkChargeOf`'s own doc comment). WHOLE is read off the RECORDED
range for every kind, a `CircleSeg`
included, and never off the walk's own closed-ness: that flag is
decided within a tolerance of a full turn, and a decad-side tolerance that
can ACCEPT is the admission gate the reject-only rule forbids.

A pre-existing source displacement can additionally AMPLIFY at a cut, by
`δ/sin θ` for a crossing angle `θ` this design cannot bound below. Section 3.4
therefore routes any scene with a `Partial` boundary edge to the mesh path
before it records a fragment whenever either source carries a nonzero
section displacement, either source carries a nonzero `δ_walk`, or the
re-expression is nonidentity. That reroute is about amplifying an INPUT
uncertainty; it does nothing about the cut's own rounding, which is why
`δ_cut` is charged on the fragments the reroute admits.

The rebuilt section therefore carries

```
δ = up( max( up(δ_A + δ_walkA), up(δ_B + δ_walkB + δ_reexpress) ) + δ_cut )
```

where `up` rounds each positive sum outward. Each operand's own walk charge
enters on that operand's OWN side of the `max`, folded there with the prior
displacement it accompanies, and it appears nowhere else in the formula: it is
charged exactly once, and no term outside the `max` repeats it. The fold is
what the per-operand split buys — every other term in this formula already
keeps the two operands apart the same way, so a heavier walk charge on the
operand that does NOT win the `max` never silently drops out.

§4.5's overlap-area reading carries this same `δ` once per measured cell, with
`δ_cut` taken over that cell's OWN surviving fragments rather than over a
merged loop's. Nothing else in the formula changes for it, and the reading's
charged sum of the cells' volumes adds only that sum's own accumulated
rounding (`bounds.go`'s `exactSumRound`).

It is **exactly zero in one decidable case**: both inputs carry zero
displacement, operand B's composed map into A's frame is the identity in the
stored floats (`frameB == frameA` and `xformB == xformA`, component-wise `==` —
G3's own comparison), **every surviving edge is whole, AND every consumed
source segment is whole**. Two caller-drawn profiles on one sketch plane with
no placement between them meet the first two conditions; they meet the third
and fourth only where the merge cut nothing — one profile strictly containing
the other, or two footprints meeting along complete shared walls, drawn (not
inherited from an earlier trim) so every one of their own segments spans its
entity's full domain. A partial overlap, which splits at least two walls,
never reaches the zero case, and neither does a union whose result the caller
would call "obviously exact": the recorded walk closes only to within
`δ_cut`, and a zero bound over it would be a claim the evaluator cannot make.
Narrowing the zero case to require every CONSUMED segment whole, not only
every SURVIVING edge, is what closes the gap this task found: an OPERAND can
carry a segment recorded over a range narrower than its own entity's full
domain — the entity's own `sketch` arrangement trimmed it there, at THAT
segment's own recording, whether from an ordinary sketch cut (`RecordProfile`
on a caller's profile) or an earlier analytic merge — while this NEW cut's
own arrangement leaves that wall untouched (`Whole = true` here, an
arrangement-LOCAL fact about THIS scene alone). Recording that survivor
without `δ_walk` would re-express the operand's own narrowed range as though
it were the new scene entity's full, exact extent: `Whole` in the result
record says nothing about whether the coordinate it names is the operand's
own recorded one or one this boolean computed walking a narrower range to
reach it.

That `δ_cut` bound is about a genuinely cut junction's own rounding along an
exact carrier; it says nothing about a whole-to-whole junction the record states
**twice, differently** (RB9, §6, §9) — that is not a rounding this design
charges, it is a claim the assembly disproves, so the merge refuses instead
of publishing a section with a displacement no term here bounds.

Each sweep endpoint is A's recorded endpoint after G5 proves the two intervals
coincide, so its axial displacement is the per-end maximum of A and B's
incoming displacement. The G5 shift is exact after G3's stored-plane equality,
so it adds no new axial term.

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
- **`prismPayload` preserves both axial end bounds** — G5 gives one shared
  interval, so the result takes the greater `z0Delta` and `z1Delta` from its
  operands independently. These terms remain separate from `δ`: one displaces
  the section in its plane and the other displaces its sweep level.
- **`bounds.go` gains the helpers for the mechanism**, under that file's own
  rule that each error mechanism has exactly one helper and no measurement site
  computes a bound inline. `cutDisplacementAllow` owns the cut-parameter
  mechanism above, turning `cutParamUlps` and a carrier's own speed into the
  coordinate displacement `δ_cut` reads. `walkEndpointAllow` owns `δ_walk`'s
  own mechanism the same way, turning the envelope of the SOURCE operands a
  walk's own arithmetic touches into the coordinate displacement its computed
  endpoint owes; the caller supplies that envelope (above), and the
  helper never reads it off the answer the walk produced. The section
  displacement's own reading
  is an AREA: the area a
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

The result's `Exactness` is then the existing rule with these displacement
terms added:

- **`Exact`, zero bound**, when every surviving segment is a `LineSeg` **and**
  `δ == 0` and both axial end bounds are zero — `moments.go`'s region-level
  exact rational accumulator with its single final rounding, over a section and
  sweep interval no coordinate of which was recomputed.
  `δ == 0` requires every survivor to be a WHOLE edge, so this arm is reached
  by a merge that cut nothing: a contained footprint, or footprints meeting
  along complete shared walls. It is deliberately narrow — the alternative is a
  zero bound over a walk that closes only to within `δ_cut`.
- **`Approximate` otherwise**, carrying the same per-mechanism proven bounds an
  ordinary prism already reports plus the displacement terms: the accumulator
  retires the moment any `CircleSeg`/`ArcSeg` survives (no `π` is ever exact),
  and a nonzero section or axial displacement remains approximate, whatever the
  segment kinds are. Every partially overlapping pair lands here.

Volume, Centroid and `Box` all read that same accumulator and that same
displacement term (evaluator §4, `moments.go`), so no separate derivation is
needed for each.

## 8. Consequences removed, and what stands outside the admitted class

| Consequence (§1) | Admitted class | Outside it |
|---|---|---|
| 1. No chaining | Removed. Result is `prismPayload`; no `meshBound` to compose, so no chord tolerance for the next pair to fall below. A chained boolean re-checks §3's gate on the new pair, carries the greater incoming displacement plus its new re-expression and cut displacement, and retains the greater incoming axial displacement at each end (§7). | Unchanged — general-position or non-analytic pairs still degrade per evaluator §9. |
| 2. Coplanar contact refuses | Removed. Coplanar, co-directional contact is the admitted case's whole premise. | Unchanged — non-coplanar or non-prism coplanar contact (e.g. a prism against a revolve cap) stays on the mesh path. |
| 3. Analytic identity dies | Removed where `δ == 0` — a merge that cut nothing. Result is `prismPayload`: Fillet/Chamfer/Shell, all three surveys, and the clearance kernel already dispatch on payload class and need zero new code for it. Where `δ > 0` — which every cut-bearing merge is, §7 — the result is a `prismPayload` still, and every consumer's reach there is §12's own rows to state, `Verify`'s tolerance gate included; the reference that gate anchors against is verification design §3's to define. Restoring the reach §12 stages, for a displacement whose CARRIERS are exact, is a separate design change, not a consequence this design removes. | Unchanged for mesh-path results — `facetedPayload` still permanently refuses modify ops (modify-reach SX9) and all three surveys. |

## 9. Refusals

Entry-gate misses (§3.1's G1–G6) and unresolved region-topology (§4.4) are
**not refusals** — no error, silent fallback. The table below is exclusively
the stage-2 refusals (§3.4), each stated once with its
sentinel, decided by the existence test (modify §1: no such body exists is
`ErrDegenerate`; a body this evaluator cannot build is `ErrUnsupported`) and
whether it is permanent. §4.5's overlap-area reading assembles nothing, so only
RB1, RB7, RB8 and RB9 can arise on it; RB2–RB6 are properties of an assembly it
never performs.

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
| RB9 | The merged loop's recorded segments do not join (`falsifyLoopJoins`, seam §3) — a whole-to-whole junction the assembly restates as two segments' own defining coordinates, which the merge did not compute and so did not round to agreement. Not RB8's class: the mismatch is inherited from an operand's own record (typically one carrying `Partial` cut fragments), not an internal `sketch` inconsistency | `ErrUnrecordableProfile` | No — a differently-drawn operand may close |

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
`ctx.Done()`. On cancellation, the caller waits for that bounded arrangement
worker to finish, discards its result, and then returns `ctx.Err()` before the
arrangement reaches the document.
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
| Tessellation | The result is an ordinary `prismPayload` and `docs/tessellation-design.md` §5's prism contract applies, with §7's section displacement as a term of its own there: it displaces the analytic boundary a mesh approximates, so it does not ride in that contract's stored-coordinate rounding term, while the per-end axial displacement still does. Tessellation §5 reserves the section displacement from the requested tolerance before chording, refuses a tolerance it exhausts, and charges it to every face bound and to `areaSlack`, so chording plus that displacement stays within `tol`. The reservation covers no other term: the per-end axial displacement rides on top of it and can lift the published `Bound` above `tol`, which tessellation §1's Tolerance row allows. A mesh of an assembled body is `Exact`-trimmed only where every one of those displacements is zero. |
| `ThroughAll` / `ThroughAllSide` | The extent reading states its interval over the RECORDED section and does not carry §7's section displacement, so at `δ > 0` the stop returns `ErrUnsupported`: it has no stated displacement to charge to the level it resolves. At `δ == 0` it reads the extent as before, beside whatever displacement that reading publishes on its own account. |
| Clearance kernel | Unchanged where `δ == 0` — dispatches on payload class; `prismPayload` already has full analytic support (`clearance.go`'s coplanar `Plane`×`Plane` certificate, `offsetPair`, etc.). Where `δ > 0` the kernel builds no model for the body and the pair reads `Suspect`: every certificate it emits is an exact statement about the carriers it read, and a carrier the payload holds only within `δ` of the one it denotes cannot support one. Widening the kernel's own candidate intervals by `δ` is a separate piece of work, not this design's. |
| Interference (`Verify`) | **Wired to the analytic path (PR4), over two read-only entry points tried in order.** After its containment and represented-set-equality certificates, `interference.go`'s `measuredInterference` calls `evaluateAnalyticIntersect` (`boolean.go`) — a read-only twin of `performBoolean`'s analytic dispatch that runs `tryPrismBoolean`/`evalPrismContext` under a self-minted `StepRef` and never commits, so it consumes neither operand — and publishes the built payload's own volume. A pair that twin does not admit (`ok == false`) next reaches §4.5's overlap-area reading (PR5), which measures a selection covering any number of disjoint regions and publishes a volume with no body at all; the two-step order is what keeps every pair the twin already answers byte-identical. A pair neither admits falls back unchanged to `evaluateBoolean`'s read-only mesh intersection, exactly as before this design existed. Both analytic answers are still subject to §6's positive-volume gate (`docs/interference-design.md` §6). "Admitted" covers `Union`'s select-all path, `Cut`/`Intersect`'s clean-nesting sub-case, the crossing sub-case (PR3), and since PR5 a multi-region coplanar overlap. `docs/interference-design.md` §5.2 records the boundary this closes. |
| Surveys (wall/undercut/min-radius) | No new code — they dispatch on payload class, and support is immediate where `δ == 0`. The undercut reading is a normal-direction membership and is unaffected at any `δ`. The wall and min-radius readings publish the bound their own arithmetic proves at `δ == 0` — a candidate's own division or square root, never the section displacement — and are staged at `δ > 0`, answering undecided (`Suspect`, never a silent pass): `δ` is not a term either reading may absorb into that bound, because the wall reading is not a quantity a displacement widens by a fixed amount — its allowance-angle contact families (verification §6) can change membership under a boundary perturbation, so a proven displaced reading needs the survey's own theory extended, not a term added to a bound. |
| `Verify`'s structural/tolerance gates | Structurally unchanged — `prismPayload` is valid by construction as always. The TOLERANCE gate anchors each reading against a reference the body's own geometry supplies, and at `δ > 0` it cannot read that geometry through the clearance kernel's model, which the row above declines to build. It reads the body's OWN recorded section instead (`gateWitnessPrism`), whose witnesses go to the same shared reader every arm publishes through: `pointSetDiameterWithBudget` states the winning pair's own distance computed over exact rationals and rounded toward zero, never the float scan's norm. The gate then shrinks that witness maximum by twice the SUM of `δ` and the axial displacement, rounding the shrunken value toward zero as well (`lowerDiameterForDisplacement`), which verification design §3 proves a lower bound on the denoted body's own diameter. A displaced body is therefore judged on the same terms as any other: a reading passes when its bound meets the tolerance and reports `Suspect` with a stated `Required` threshold when it does not. |
| Export (STL/OBJ) | Reads `Tessellate`'s output. Its size-derived default tolerance is raised past `δ`, which tessellation reserves from the tolerance before chording, so a default export of an assembled body still writes its mesh rather than refusing. |
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
- **§4.5's overlap-area reading answers a multi-region pair for `Verify`
  alone; the public booleans keep refusing it.** `Union`/`Cut`/`Intersect` owe
  their caller a `Body`, and a body covering several disjoint lumps needs the
  multi-lump prism payload §4.4 still stages — changing `prismPayload`'s own
  contract would ripple through topology, tessellation, every survey, the
  clearance kernel, the modify ops and §4.1's equality certificate, which is
  the opposite of the smallest sound extension. `Verify` needs no body, so it
  is given the volume alone. The cost is a visible asymmetry: a pair `Verify`
  reports an `Interference` for is a pair `Intersect` still routes to the mesh
  path. Two alternatives were rejected. Extending `mergePrismCells` to emit
  several loops cannot work at all — a `ProfileRecord` has one `Outer`.
  Restating the volume composition inside the reading, rather than building one
  `prismPayload` per cell and reusing `evalPrism`, would put a second owner on
  `bounds.go`'s section-displacement and axial mechanisms, which that file's
  own rule forbids.
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
   on admission. A split boundary routes to the mesh path before recording
   when either source carries a section displacement or B's re-expression is
   nonidentity.
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
3. **PR3 — crossing sub-case for `Cut`/`Intersect`.** §4.2's edge-orientation
   propagation over hole-free operands (`prism_boolean_crossing.go`), reusing
   PR1's `mergePrismCells` chain/merge tail and §6's audit over its own
   per-op cell selection. Tests: two overlapping (not coincident-carrier)
   prisms cut/intersect correctly against the mesh path's own answer on the
   same pair (property test, volumes agree within the analytic path's
   tighter bound). Two pieces of the general mechanism (§4.2) remain: a cell
   reachable only through a coincident carrier is still unresolved (silent
   mesh-path fallback, not a refusal), and G6's `Union` arm still excludes a
   holed operand (its `Cut` tool arm stands regardless, §13) — a holed hub
   unions with a tooth correctly once that lands.
4. **PR4 — replay/interference wiring + docs.** A stored
   `OpUnion`/`OpCut`/`OpIntersect` step builds via the analytic path
   post-upgrade with no wire change (recipe-replay-design §8's contract,
   §12 above), pinned by a round-trip/replay test on an admitted coplanar
   `Union` step. `interference.go`'s `measuredInterference` reaches the same
   read-only analytic `OpIntersect` dispatch `performBoolean` uses
   (`evaluateAnalyticIntersect`, `boolean.go`) after its containment and
   represented-set-equality certificates, falling back unchanged to the mesh
   path when the analytic path does not admit the pair; a test asserts an
   exact overlap volume on an admitted coplanar-prism pair that meets §7's
   exactness conditions. `docs/interference-design.md` §5.2 carries a pointer
   noting the prism case is superseded here. `docs/evaluator-design.md` §9
   gets a pointer to this document (§ done alongside this design, see the
   CLAUDE.md diff).
5. **PR5 — §4.5's multi-region overlap-area reading.** The per-cell record,
   per-cell `prismPayload`, `evalPrism` volume and charged sum, entered after
   `evaluateAnalyticIntersect` returns `ok == false`, over the shared
   admission/scene/classification of PR1–PR3. Nothing in the public
   `Union`/`Cut`/`Intersect` surface changes, and no bound helper is added.
   Tests: §15's multi-region rows.

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
  outer loop (verbatim reproduction, §4.2's structural-match claim) — for a
  target and tool whose own consumed segments are whole, the only case this
  claim holds for (§4.2's own qualification).
- A trimmed source segment's walked endpoint is charged (task fu143): an
  operand whose OWN recorded profile carries a segment ranged narrower than
  its entity's full domain (a `LineSeg` fragment of a larger sketch-cut line,
  the ordinary shape a `sketch` arrangement produces for a rectangle split by
  another entity) reports a positive `sectionDelta` from `Union`, `Cut`'s
  clean-nesting match, and `Intersect`'s clean-nesting match alike, and each
  op's published volume bound contains the residual against the exactly
  computed union of the two operands' own DENOTED sections (`math/big.Rat`
  over the operands' own recorded floats, never a second float answer). The
  same fixture with every consumed segment whole publishes a `sectionDelta`
  of exactly `0.0` on all three ops, pinning that the new charge does not
  fire on the case §7 already kept exact.
- The walk charge survives `lerp2`'s CANCELLATION: over a table of trimmed
  `LineSeg` carriers whose `End − Start` cancels — the extreme one reaching
  `±1e12`, and a plain 200 mm carrier whose fragment sits on the sketch origin,
  which needs no extreme coordinate at all — `walkChargeOf`'s answer contains
  the EXACT rational residual of the endpoint `lerp2` walked to, at both
  recorded bounds, compared squared over `math/big.Rat` so no square root or
  float difference can flatter it. Each row also asserts its own premise: that
  charging the walked endpoint's own envelope (`segmentWalk.coordUpper`)
  instead of the carrier's would under-charge exactly on the rows that cancel
  and NOT on an ordinary uncancelling fragment, so a row that stops
  reproducing the shape fails rather than passing quietly.
- The trimmed-circular refusal reads the RECORDED range, not the walk's
  closed-ness: a `CircleSeg` whose `TEnd` is one ulp short of `1` — a range
  the walk's own tolerance still calls a closed turn, which the fixture must
  assert directly so the case cannot silently stop being the one under test —
  refuses all three ops (`ok == false`, no error), while the same pair with
  the bound recorded exactly resolves analytically, so the fallback is the
  trim's own doing.
- Exactness, one test per §7 arm: a line-only merged section over operands
  that share a frame AND whose merge cut nothing (a contained footprint)
  reports `Exact` volume with a zero bound; a partially overlapping pair of
  the same two operands reports `Approximate`, and the test computes the
  merged region's TRUE volume over `math/big.Rat` from the operands' own float
  coordinates and asserts the published bound contains the residual — the arm
  that proves `δ_cut` is charged rather than assumed away; the same
  section over an operand B with a nonidentity re-expression but no split
  boundary reports `Approximate` with a bound the §7 displacement term alone
  explains (assert it scales with the placement's own magnitude, so a payload
  silently dropping the term fails);
  a merged section retaining a `CircleSeg`/`ArcSeg` reports `Approximate` with
  a bound composed from `moments.go`'s and `bounds.go`'s machinery, asserted
  against the closed-form answer.
- An arranged profile containing a `Partial` boundary edge falls back before a
  fragment is recorded when either source carries a nonzero section
  displacement, either source carries a nonzero walk charge (`δ_walk`), or
  B's re-expression is nonidentity. The focused fixtures must prove the
  arrangement splits, including an identity second re-expression after a
  displaced first result, then assert that `tryPrismUnion` returns
  `ok == false` without an analytic-resolution error.
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
- §4.5's multi-region reading, on a U-shaped prism crossed by a bar whose two
  operands' outlines overlap in exactly two disjoint 12 mm² regions over a
  5 mm sweep: `Verify` reports one `Interference` row whose volume is
  120 mm³, `Approximate`, with a bound below 1e-9 mm³ — the cut parameters'
  own rounding and nothing else. The same fixture asserts the report carries
  no `unsupported_pair` diagnostic, which is what the mesh path emits for a
  coplanar pair, so a regression back to the mesh path fails rather than
  passing with a different number.
- The single-region reading is untouched: two coplanar boxes overlapping in
  one 25 mm² region over a 5 mm sweep still report 125 mm³ through
  `evaluateAnalyticIntersect`, with the same bound they report before §4.5
  exists — the assertion is on the volume and on the reading remaining
  `Approximate` at the same magnitude, pinning the two-step order.
- The exactly-tangent pair stays undecided: two coplanar prisms whose sections
  meet along a shared wall but enclose no common area report no
  `Interference` row and leave the report `Suspect`, with §4.5's reading
  answering nothing rather than a zero-volume row.
- A bored (faceted) pair stays undecided: G1 excludes a `facetedPayload`
  operand, so the reading is never entered and the pair reports exactly what
  it does today.
- Cell-sum soundness against an independent answer: for a multi-region fixture
  whose true overlap area is computable in closed form, assert the published
  volume interval contains the exact rational answer taken over the operands'
  own recorded floats (`math/big.Rat`), never a second float computation.
- Cancellation and the cap: a context cancelled during the reading returns
  `ctx.Err()` and leaves the document and both operands unchanged; a pair
  whose scene exceeds `prismMaxArrangementSegments` reports `Suspect` through
  the existing `ErrUnsupported` mapping, with no report-level error.

## Implementation notes

Every `sectionDelta` consumer below is a no-op at the zero every caller-drawn
payload carries. `extrude.go`'s `evalPrism` composition and `tessellate.go`'s
tolerance reservation, mesh-bound charge and area-slack charge are §7's own
subject and are not repeated here.

§12 already rules on four of the remaining consumers, and this section adds
only the call site each of its rows lands at. What the consumer does at
`δ > 0` is that row's to state, and is deliberately not restated here:
`survey.go`'s `prismWall` and `prismMinRadius` are §12's "Surveys
(wall/undercut/min-radius)" row, `extrude.go`'s `extentAlong` is its
"`ThroughAll` / `ThroughAllSide`" row, `clearance_geom.go`'s
`addPrismFaces` is its "Clearance kernel" row, and `verify.go`'s
`gateWitnessPrism` is its "`Verify`'s structural/tolerance gates" row.

No §12 row rules on the two remaining consumers, and this section owns them.
Each withholds its answer rather than measure the recorded section as the one
it denotes:

- `interference.go`'s `analyticBodiesEqual` withholds the equal-records
  set-identity certificate: two records being equal says nothing about two
  sets each record is only within its own `δ` of.
- `fillet.go`'s `requireExactSection` refuses the receiver for
  `Fillet`/`Chamfer`/`Shell` with `ErrUnsupported`, since a rewrite of that
  section has no proven displacement of its own.

### boolean.go

`boolean.go`'s `faceChordDelta` is one of the two consumers that do NOT
withhold: it charges a nonzero `sectionDelta` as a planar face's own chording
displacement, so the mesh path's tangency gate
(`refuseUndecidableProximity`) cannot read that face as held exactly. See
`faceChordDelta`'s own doc comment. The other is `verify.go`'s
`gateWitnessPrism` above, which charges `δ` as a shrink on the tolerance
gate's reference diameter.

### capblend_contour.go (boundary note)

The cap-loop chamfer's cap contour displacement (`docs/modify-reach-design.md`
§8.4) is a separate term with its own owner, derived by `capblend_contour.go`
and read by `capblend_geom.go`/`capblend.go`/`capblend_moments.go`. It is not
`sectionDelta`: nothing in this design's own consumers reads the cap contour
term, and nothing on the cap-loop chamfer path reads `sectionDelta`.
