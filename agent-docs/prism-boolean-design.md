# Prism Boolean Design

An analytic reduction for `Union`/`Cut`/`Intersect` over co-directional
prisms sketched on one plane, or on parallel planes offset along their shared
normal (§3.1 G3): the 3D boolean reduces to a 2D combination of the two
operands' recorded sections, routed entirely through `sketch` — decad selects
among the regions `sketch`'s arrangement returns and rebuilds a `prismPayload`
from the selection; it never computes a crossing, a cut parameter, or a region
membership of its own. Companion to `agent-docs/evaluator-design.md` §9 (the
existing mesh boolean, unchanged, and still the fallback for everything this
design does not admit) and `agent-docs/modify-design.md` §2 (the section-rewrite
reduction this design reuses and extends from one operand to two). References
of the form "core §N" are to `agent-docs/api-design.md`; "modify §N" to
`agent-docs/modify-design.md`; "seam §N" to `agent-docs/sketch-seam-design.md`.

## Parts

This file holds 9 parts. A section number with no part prefix is a section of Part B.
`Part X §N` is section N of part X. Short names such as `general §3` in comments and design docs
mean the same part. The parts keep their own section numbers.

| Part | Former document | Short name | Owns |
|---|---|---|---|
| Part B | the unprefixed sections of this file | `prism-boolean`, `prism` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms |
| Part G | General Boolean | `general-boolean`, `general` | Boolean classes past the prism pair, and `brepPayload` |
| Part R | Boolean chain regroup | `boolean-chain-regroup` | Rebuilding a refused two-body Boolean from the source bodies its history names |
| Part C | Boolean continuation | `boolean-continuation` | Source-expression continuation after an accepted geometry adjustment |
| Part S | Stacked prism | `stacked-prism`, `stacked` | Stacked slabs, walls, measurements, mesh and consumers |
| Part K | Carrier record | `carrier-record` | Carrier records of displaced prisms; producers, lateral blends, exact-loop cap chamfers |
| Part P | Pattern union | `pattern-union` | Joining prism and axial Loft pattern copies to a base through `Union` |
| Part U | Union fold | `union-fold` | The two-at-a-time fold of a many-body `Union` no certified route takes |
| Part A | Boolean of axial extrusions by the 2D Boolean of their sections | `axial-section-boolean` | Union of axial extrusions through sketch loop containment |

## 1. Problem

`Union`/`Cut`/`Intersect` tessellate both operands and intersect meshes
(evaluator-design §9). For a pair of straight prisms extruded from the same
plane — the normal shape of a combined part built from sketch-adjacent
features (a hub and its teeth, a boss and a plate) — that pipeline is
structurally worst-cased three ways:

1. **Limited chaining.** Each rim's bound composes across operations, and a
   later boolean on the same lineage whose contacts touch a rim coarser than
   the chord tolerance the pair derives from its own diameter refuses (a plain
   `ErrUnsupported` naming the operand and quoting the touched bound and the
   pair's chord tolerance — core §8, "The chain depth").
2. **Coplanar contact refuses outright.** `triTriClassify`'s `contactRegion`
   is a tangency the exact chord predicates cannot classify
   (`errUnclassifiableContact`), and two bodies sharing an extrusion plane
   always have one.
3. **Analytic identity dies.** The result is `facetedPayload`: modify-reach
   SX9 refuses every modify op on it permanently, and `internal/verifyrun/survey.go` refuses all
   three shape surveys.

`sketch` supplies the two answers that make a second, decad-owned 2D
intersection kernel unnecessary. A circle×circle crossing reports
`TExact = true` on every fragment. A tooth arc lying exactly on a hub circle
arranges without the `Degenerate` flag into two clean regions that share that arc
as one boundary.

**This design routes the 2D combination through `sketch`, as CLAUDE.md's hard rule
requires** ("an intersection ... ask `sketch`, consume its answer"). decad builds a
scene holding both operands' recorded entities, asks `sketch` to arrange it, and
selects among the returned regions. §4 states, for each op, how that selection is
made without decad computing a crossing or a containment test of its own. The one
sub-case without a route is a general-position holed union (§4.4, G6).

**Success criterion.** For the admitted class (§2): a chained boolean carries
no accumulated tessellation bound; a shared or crossing extrusion plane is the
ordinary case, not a refusal; the result is a first-class `prismPayload` that
Fillet/Chamfer/Shell, all three surveys, and the clearance kernel already
handle wherever its section displacement is zero (§7). §8 states exactly which
of the three consequences above disappear for the admitted class, which stand
outside it, and where a nonzero displacement's cost per consumer is decided.

## 2. Approach

**Chosen direction.** `performBoolean` dispatches a reject-only admission gate for
`Union`, `Cut` and `Intersect`, ahead of the shared `evaluateBoolean` mesh
pipeline. An admitted pair combines its two operands' `ProfileRecord`s through
a private `sketch` scene decad builds. It records the selected result regions
through the existing seam (`RecordProfile`/`recordEdge`), audits the assembly
with `modify §5`'s existing closed-form checks, and rebuilds through
`prismrecord.Build`. Section 7 extends the result with a section-displacement bound so
the re-expressed operand's rounding and the cut parameters `sketch` computes
for surviving fragments reach every measurement. It also preserves each sweep
end's axial displacement so every measurement keeps the bounds its operands
already proved.
A non-admitted pair takes the unchanged mesh path, with **zero behavior change**:
no new error, no new refusal text. Non-admission covers a wrong payload class,
planes that are neither the same nor a shared-axis offset (G3), a segment kind
outside the admitted set, an unequal `Union` z-interval that Part G §3 A1 does
not stack, and a topology the region resolution does not cover (§4.4).

**Alternatives considered:**

- **Decad builds its own 2D regularized-boolean kernel** — rejected because
  `sketch` already answers the two asks the gear's normal case needs;
  building a second intersection kernel duplicates work `sketch` already does,
  with no performance or correctness win over it that CLAUDE.md's "Ask
  `sketch` for 2D answers by default" rule would accept as the reason for a
  decad-side 2D answer. The coplanar contact patch of two exact planar faces
  (`agent-docs/multibody-dynamics-design.md` §9.4) is such a decad-side answer: a
  convex-polygon clip over rational vertices, with no arcs, circles or
  regions, and §9.4 states its reason; this design's section combination
  stays with `sketch`.
- **General exact analytic B-rep boolean** (arbitrary curved-surface pairs,
  algebraic root isolation) — the eventual vN destination (api-design §2.1).
  This design's gate/audit/authentication pattern is its first step, not a
  competing direction.
- **Mesh-path palliatives** (a caller-set chord tolerance) — remove only the
  chaining consequence, and only partially. The coplanar refusal stands untouched.
  Not pursued.
- **Keep the CSG tree, answer queries against it without evaluating** — breaks
  the promised B-rep-shaped surface (`Faces()`, selectors, stops) and, for the
  wall/undercut surveys, converges on this design's own 2D combination anyway.
  Not pursued.

## 3. The admitted class

### 3.1 Entry gate (silent fallback, never an error)

Checked before any `sketch` call, on the two operands' *current* payloads
(`prismPayload`, which includes a shell-built tube — modify §8's
`prismshell.BuildTube` builds one). Every row is a routing decision: failing it takes the
unchanged mesh path with no error surfaced, identical to the mesh path's behavior for
that pair.

| # | Condition | Why exact, not tolerant |
|---|---|---|
| G1 | Both operands' payload is `prismPayload`. | Structural — the class this design's reduction applies to (modify §2's same reduction, extended to two operands). |
| G2 | None: a reflected accumulated placement admits (Part G §3 A4). When the composed relative map of §4.1 is a reflection (`Transform.IsReflection`), operand B's record is re-wound before any entity is created; two operands under one reflected placement meet G3's shared-axis arm with no re-expression. | G3 reads the world normal through `ApplyDir`, which maps a reflection's sweep direction like any other. The re-winding restores the "outer CCW, holes CW" convention §4.2's flag comparison reads. |
| G3 | The composed world normals agree exactly, and the coplanar, shared-frame-axis, or placed-axis arm below holds. | Use stored floats and exact rational arithmetic for the axis arms; a rounded residual never admits a pair. |
| G4 | Every segment of both operands' `ProfileRecord` (`Outer` and every loop of `Holes`) is a `LineSeg`, `CircleSeg`, or `ArcSeg`. | `geom.BoundaryEdge.TExact`'s own contract is a **whole-scene** gate (`sketch`'s `geom/region.go`): one `Ellipse`/`EllipticalArc`/`Conic`/`Spline`/`ClosedSpline`/`FitSpline`/`NURBS` anywhere in an arrangement makes every bound in it — including unrelated line/circle/arc edges — report `TExact = false`. A single free-form segment on either operand would silently blind the whole combination, not just its own edges, so the gate excludes the kind entirely rather than trying to admit "the free-form parts don't touch." The free-form segment is not admitted (§4.4's last row). |
| G5 | Apply G3's exact rational axial shift to B's interval, then check §3.2's interval relation exactly for every level each held float, displacement and denotation allows. | A rounded float sum cannot decide whether a tool reaches a target cap, and neither can two held floats that each round a different level. |
| G6 | `Union` needs both operands hole-free. `Cut` needs the tool hole-free; the target may carry holes. `Intersect` needs both operands hole-free except for the one-hole clean-nesting arm below. | `Union`'s select-all rule would include a hole's void. A holed `Cut` tool can leave separate lumps that one `ProfileRecord` cannot carry. The admitted holed `Intersect` has one nested result profile, including its hole. |

G1–G6 are the only conditions checked before touching `sketch`. **Passing
them is not admission** — see §3.4.

#### G3 — exact sweep-axis arms

Read each composed world normal as `xform.ApplyDir(frame.N())`. Require Go
`==` on the two vectors, so `-0.0` equals `0.0` (§3.3), and require one of
these arms:

- **Coplanar:** `(worldOriginB − worldOriginA)·worldNormalA == 0.0`, with
  `worldOrigin = xform.Apply(frame.Origin())` and the ordinary float dot product.
- **Shared frame axis:** `xformA == xformB`, both frames' `U` and `V` agree
  componentwise, and the exact dyadic difference of their stored origins has
  zero exact cross product with the stored `frameA.N()`. The exact rational
  shift `s` satisfies `originB − originA = s·N`. `CreateOffsetPlane` on an
  axis-aligned base produces this arm; a tilted base whose `U` differs by one
  ulp does not.
- **Placed axis:** both frames' `U` and `V` agree componentwise, the two
  transforms' stored linear bases agree componentwise, and the exact dyadic
  difference of their composed origins has zero exact cross product with
  the exact linear image of the stored frame normal. The exact rational
  shift `s` satisfies `X_B(originB) − X_A(originA) = s·(R·N)`. This admits
  `Copy` along the common sweep axis without re-expressing either profile.
  This arm additionally requires zero section and axial displacement on both
  operands. `Union` admits exactly equal recorded profiles, compared under
  the caller's work budget, or the existing identical axis-aligned rectangle
  certificate. `Intersect` requires identical rectangle bounds; `Cut`
  requires the tool rectangle to lie strictly inside the target rectangle.
  These are sufficient profile certificates; every other placed-axis pair falls back
  to the mesh path. The shared-frame-axis arm has its own profile admission.
  `Verify`'s read-only Intersect twin (`booleandispatch.AnalyticIntersect`) runs the same admission, so a pair
  this arm admits is measured from the built intersection body's volume, under `Verify`'s positive-volume gate.

For both axis arms, B's denoted prism has A's exact section coordinates and
only its sweep interval moves by `s` (G5). The placed-axis arm does not rely
on a rounded `ApplyDir` result to prove collinearity: multiply the stored
transform basis and frame normal in dyadic arithmetic. The existing
same-transform arm likewise reads the stored frame origins exactly. An
in-plane translation, a different linear basis, or different frame `U`/`V`
takes the mesh path, except for the blind-Cut arm below. No residual tolerance admits a pair; this is decad's
own admission rule under `CLAUDE.md`'s reject-only rule.

After ordinary analytic admission misses, a blind `Cut` can use identical stored frames and equal finite,
invertible transform bases when the exact in-section translation is nonzero. An exact pure-axial miss keeps
the earlier Boolean route. `prismplacement.CutTranslation` solves the three local coefficients over exact dyadic rationals. It
rounds the two section offsets once and charges their error to the re-expression; the exact third coefficient
shifts the tool's held and denoted sweep levels. The blind-cut slab gates compare those shifted rational levels.
This arm does not change `Union`, `Intersect`, or the other Cut routes.

G5 obtains `s` by dividing the largest-magnitude nonzero component of G3's
exact composed-origin difference by the corresponding component of its exact
transformed axis vector `R·N` (`N` when the common transform basis is the
identity). The coplanar arm uses zero. `prismplacement.ShiftedInterval` adds `s` to each B
endpoint as a `big.Rat`; every §3.2 comparison is exact. `Union` and `Cut`
keep an operand's own endpoint. `Intersect` rounds a selected shifted B
endpoint once and charges `rationalFloatError` beside the displacement §7
selects. No rounded float sum admits a short tool.

`Intersect` has one additional G6 arm: exactly one operand may have exactly
one hole when the other is hole-free. Reorder this symmetric operation so the
hole-free operand is A, then require the clean-nesting proof in §4.2 to show
that A contains B's whole outer loop. The result must be one `sketch` profile
whose outer and hole reproduce B's loops whole. Any other holed intersection
falls back to the mesh path. This arm leaves the shared overlap-area reading
of §4.5 on its existing admission gate.

### 3.2 Per-operation shape

| Op | Required relation (after G5's shift) | Result z-interval |
|---|---|---|
| `Union(a, b)` | `[z0_a, z1_a] == [z0_b', z1_b']`, exactly | the common interval |
| `Union(a, b)`, stacked | the intervals differ and overlap or touch: `z0_b' <= z1_a && z0_a <= z1_b'`, with every pair of neighbouring levels of both operands strictly ordered across their ranges, or one shared denoted rational where two coincide | a `stackedPrismPayload` with one slab per pair of adjacent distinct levels; Part G §3 A1 |
| `Cut(target, tool)` | `z0_tool' <= z0_target && z1_tool' >= z1_target` (tool spans target) | `[z0_target, z1_target]`, unchanged; a clean-nesting cut keeps the target's denoted levels (stacked §2.1) |
| `Cut(target, tool)`, blind | `z0_target < z0_tool' < z1_target && z1_tool' >= z1_target` (a pocket from the top, floor at `z0_tool'`), or `z0_tool' <= z0_target && z0_target < z1_tool' < z1_target` (a pocket from the bottom, ceiling at `z1_tool'`) | a two-slab `stackedPrismPayload` (Part S): the target's own interval, split at the tool's inner end. Every inequality is read on the level ranges below over G5's shifted interval, an inner end whose held float rounds onto a cap declines, and the inner end is that exact rational rounded to the nearest float once, carrying the tool's own axial displacement plus `rationalFloatError` of the rounding — the same charge `Intersect`'s shifted endpoint takes; the slabs also record the level that end denotes when the tool's own level is known exactly (stacked §2.1) |
| `Cut(target, tool)`, stacked target | the tool spans the whole stack: `z0_tool' <= z0_first && z1_tool' >= z1_last` | the same slabs; each is cut or proven unchanged because the tool lies inside a hole |
| `Cut(target, tool)`, stacked target, blind | the tool opens at one outer cap and ends strictly inside a slab across the level ranges, or at an interface both slabs and the tool denote as one rational | an interior slab splits; each reached slab is cut or proven unchanged by its own scene; opposed holes at an interface require exact, separated sections, and exact overlapping ones build general-boolean §3 A8's BRep |
| `Intersect(a, b)` | `z0_a < z1_b' && z0_b' < z1_a` (intervals overlap) | `[max(z0_a, z0_b'), min(z1_a, z1_b')]`, each end charged the larger of both operands' displacements |

`Union`'s equality is exact float equality — not a tolerance — matching G3's
discipline: two teeth swept to visibly-the-same but not exactly equal heights
(e.g. built through two different construction paths) refuse to the mesh
path rather than being blessed as "close enough." All three relations are
`big.Rat` comparisons over the operands' own recorded floats and G5's exact
rational shift — no new rounding. So are the stacked and blind rows. An
offset plane 0.1 mm below a target and a tool 0.3 mm tall put B's cap at the
exact rational `fl(0.1) + fl(0.3)`, which is no float; G5 compares it as it
is, and only `Intersect`'s result rounds it, charged (§7).

Each relation in every row must hold for the levels the held floats denote,
not only for the floats. A level with zero displacement denotes its own float.
A displaced level denotes the rational stacked §2.1 records for it, or, with
no record, any value within its displacement of its float.
`prismplacement.LevelRange` reads each level as that exact range, lifted by
G5's shift, and an inequality is admitted only when it holds across both
ranges: the tool's lower level at or below the target's whatever their
displacements, and likewise the other ends. The blind rows read their outer
and inner ends this way (`stackedrecord.blindArmsOf`), as does the split of
a stacked target's slab (`rangeSplit`); the stacked `Union` row orders and
merges every level of both operands this way (`stackedrecord.UnionLevels`).
`Intersect`'s result end can be either operand's true end where the two
ranges overlap, so it takes the larger of both displacements.
`Union`'s equality needs equal held floats and one shared denoted rational at
each end. A cap meeting a cap is therefore a valid span only when both denote
one rational, as two undisplaced equal floats or two 0.2 in levels do, and
never an overlap. Equal held floats prove nothing on their own: a tool swept
to 0.2 in holds the float of a 5.08 mm plate top, denotes a level below it,
and so leaves a skin over its hole; that `Cut` takes the mesh path. Any
relation decided inside a displacement window without one shared denotation
declines to the mesh path the same way.

The two blind `Cut` rows require whole-loop scene matches. `prismcells.ResolveCut`
proves a tool disjoint from the target's existing holes. An enclosing tool
uses two cells: the target's outer with the tool and any outside holes, then
the tool's outer with precisely the holes it encloses. The cut slab keeps
only the outside holes plus the tool. The other slab keeps the original
section; stacked §2.3 shares walls where their loops equal. A blind tool
whose boundary crosses the target's, a tool strictly inside both ends (an
enclosed void), and a tool touching the target only at a cap fall through to
the mesh path. A blind tool on a stacked target splits the slab containing
its inner end and uses a whole-loop scene for every reached slab. A tool
ending at an exact existing interface reuses that level when the reached
slabs' scenes prove a clean cut or no change and the interface audit accepts
the resulting hole sets. `Intersect` with a
stacked operand takes the mesh path over that payload's tessellation;
`Union` with one follows Part G §3 A1.

Disjoint intervals (for `Union` and for `Intersect`) are **not
refused**: they fall through the gate to the unchanged mesh path.

### 3.3 The rotated-section problem

The consumer's teeth are placed copies arranged by a circular pattern, so
each tooth's section arrives at a different world rotation. Against `r3`:

- `r3.RotationAround` about `(0,0,1)` keeps the composed plane's normal
  **exactly** `(0,0,1)` (G3's exact `==` test passes) for only a minority
  of step counts. For most counts some step leaves the z-component a nonzero
  float, and G3 correctly refuses the pair to the mesh path. The cause is the
  general Rodrigues evaluation, whose `cos + fl(1 − cos)` term is exact only
  when the roundings happen to cooperate, so this is a missing guarantee
  rather than a defect.
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
takes the unchanged, working mesh path at most step counts. That is a missed
optimization, not a new failure mode. A consumer can use `FromBasis` to clear
G3, and the crossings its re-expression can move are then charged as §3.4
states. An `r3` rotation constructor that leaves a plane perpendicular to its
axis exactly perpendicular would remove the miss; nothing here depends on it.

### 3.4 What "admitted" means, precisely

Admission is **two-stage**, and the boundary between the stages is where a
silent fallback stops being available:

1. **Entry gate (§3.1–3.2).** Cheap, pre-`sketch`, structural. A miss here is
   never an error — the unchanged mesh path runs exactly as it does today.
2. **Bounded region resolution (§4).** Before `s.Profiles()` runs, the
   code-owned `prismcells.MaxArrangementSegments` cap bounds the pinned
   arrangers' tiny-segment pair work. Exceeding it is an `ErrUnsupported`
   refusal (§9), never an unbounded private worker. A pair within the cap
   builds the private scene, arranges it, and attempts to resolve a unique
   candidate result. **A pair whose topology the resolution logic
   does not cover (§4.4) is treated exactly like a stage-1 gate miss: silent
   fallback, no error.** When `sketch` returns any `Partial` boundary edge
   and either source carries a nonzero section displacement, either source
   carries a nonzero walk charge (§7's `δ_walk` — a consumed segment whose
   own recorded range narrows its natural domain), or B's re-expression is
   nonidentity, any of the three can move a transverse cut by its
   displacement divided by the crossing sine. Part G
   §3 A6 charges that movement at every cut (`prismcells.CrossingCharge`).
   Such a pair never refuses: a crossing whose proven sine is not above
   verification §4's dimensionless noise floor has no charge, and that, or
   any later merge, audit or recording failure, sends it to the mesh path
   with no error (`prismcells.AmplifiedFallback`), exactly as an unresolved
   topology does. `Transform` is the ordinary way a pair reaches the
   re-expression cause, and it reaches it through the ACCUMULATED placement
   rather than through the motion any one call received:
   `prismcells.NewReexpression` reports the identity exactly when G3's shared-axis
   arm holds — `pa.xform == pb.xform`, bit-identical `U`/`V`, and a frame-origin
   difference that is exactly a multiple of the shared `N` (a coplanar pair on
   one frame is that arm with `d = 0`) — and `Transform` composes its
   motion onto the transform its receiver already carries instead of replacing
   it. A sequence of motions composing back to the partner's own accumulated
   placement leaves that map the identity again, and the identity charges no
   re-expression term.
   Every other capacity, arrangement, candidate-validity, or assembly-audit
   problem is a genuine refusal (§9's table), **never** a reroute to the mesh
   path. An admitted-then-failed pair
   does not silently become an `Approximate` mesh result whose exactness claim
   the caller never asked to downgrade to; it becomes an explicit
   `ErrUnsupported`/`ErrDegenerate` the caller can branch on, matching every
   other modify-op refusal in this codebase.

## 4. Design

### 4.1 Scene construction

For an admitted pair, decad builds one private `sketch.Sketch` (the same
`sketch.NewWorld().CreateSketch(...)` pattern `internal/momentinput/reconstruct.go`'s
`momentRecordScene` already uses for authentication):

- Operand A's frame is the reference (`target`'s, for `Cut`). Every entity —
  A's and B's alike — is created from its own segment's **walked** geometry
  (`walkOf`, extrude.go), never from the record's `Point2` fields directly:
  this is the same reduction `momentRecordScene` uses for a single record's
  own self-consistency check, and it is why a `Partial` fragment is recreated
  over its own WALKED portion rather than its source curve's whole extent
  (prismcells.BuildScene's own doc comment). A segment whose recorded range is the
  entity's own full domain walks to the record's own coordinates verbatim —
  `momentregion.Lerp2` and `pinArcWalkEnds` both special-case the natural bounds `t=0`/
  `t=1` to return the record's own `Point2`, and a `CircleSeg` recorded over
  those same bounds walks the recorded centre and radius directly — so
  operand A's segments
  reach the scene with zero new rounding wherever every one of them is
  WHOLE. A segment recorded over a **narrowed** range instead evaluates the
  carrier at a **computed** parameter, so it enters the scene at an endpoint
  this boolean computed, not one the record states — charged as §7's
  `δ_walk`. A trimmed circular carrier (`ArcSeg`/`CircleSeg`) moves by more
  than its two endpoints — its rebuilt radius and sweep move too, since the
  scene's arc is built through cos/sin-computed points — so the gate
  refuses a pair carrying one before the scene is even built
  (`prismcells.ProfileHasTrimmedCircularSource`) rather than publish an
  under-charged bound for it; the routing is the same silent §3.4
  fallback as every other entry-gate miss. A `Cut` of a bore tool out of a
  union's merged outer loop, which holds trimmed `CircleSeg`s, therefore takes
  the mesh path. The bore `Cut` instead follows the root `Fillet` of that
  union: the fillet rewrite (`offset2d.OriginalSegment`) re-emits every walk
  of a filleted loop as a whole `LineSeg` or `ArcSeg` and clones every other
  loop, so the filleted part enters the `Cut` scene with whole segments, and
  the tool records its own circular segments whole (a D bore as a whole
  `ArcSeg` and a line). A clean-nesting `Cut` of a target with a valid
  carrier record keeps a record of its own
  (Part K §5.4).
- Operand B's `ProfileRecord` is **re-expressed into A's frame** before its
  entities are created, through the **composed relative map**: B's frame to
  world, B's placement, A's placement inverted and A's frame inverted are folded
  into ONE rigid transform (`r3.FromFrame` and `Transform.Then`, each inverse
  exact — the transpose, `r3.Transform`'s own contract (core §5.2) — and every
  step a dot product, never a solve), and each `Point2` is mapped once through
  it, dropping the resulting local z, which G3's coplanar arm already certified
  is the shared plane's own zero axis (the shared-axis arm never reaches this
  map: it is the identity below). Composing first is what keeps the rounding at the two
  operands' RELATIVE offset: walking each point out to world space and back
  rounds it at each operand's own WORLD magnitude instead, so a pair sitting
  1e16 mm from the origin — where one ulp is 2 mm — loses millimetres to a
  translation the two operands share and that cannot move them relative to each
  other at all. Composing proves nothing, though; it narrows the rounding
  without removing it, which is why §7 carries the displacement regardless. This
  is the only new rounding this design commits on an operand's own INPUT
  coordinates: an ordinary rigid-transform coordinate computation, rounded once
  per coordinate, on operand B's segments only — and where G3's shared-axis
  arm holds (B's placement and `U`/`V` ARE A's own in the stored floats, and
  B's origin sits on A's normal axis exactly — a pair drawn on one plane, or on
  a datum and its `CreateOffsetPlane`) the composed map is the identity, B's
  `Point2` fields are copied verbatim, and nothing is computed at all: B's
  denoted prism is A's frame swept over `[z0 + s, z1 + s]`, so its section
  needs no re-expression. It is **not** the only new rounding
  the design introduces: the cut parameters `sketch` computes for the surviving
  fragments are new rounded coordinates of their own, whatever the inputs
  carried, and §7 charges them separately.
  Non-`Point2` fields (a `CircleSeg`'s `Radius`, every `CCW`, every `TStart`/
  `TEnd`) are magnitudes or parameters under a proper rigid map and carry over
  unchanged; under a reflected map, B's record is re-wound instead
  (Part G §3 A4).
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
  carrier by construction (`sketch` `docs/coincident-carrier-resolution-design.md`'s
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

**`Union`, hole-free operands (G6): select every returned cell, once no
cell is a void.** Two hole-free operands can still enclose a bounded cell
that is material of neither: two C shapes facing each other with
overlapping tips, or a ring of overlapping bars, bound an empty cell between
them that neither operand's own loop encloses. Selecting every cell would
fill it. So the select-all path first requires every cell to carry at least
one boundary edge on its operand's material side — `Reversed` compared with
the authored sense, the crossing sub-case's own flag comparison
(`prismcells.CellsHaveNoVoid`). One material-side edge puts the cell inside that
operand, since membership is constant over a cell. A cell with none is
outside every operand with an edge on it, and a cell bounded by one
operand's edges alone would be a hole of that operand, which G6 excludes,
so such a cell is a void and the pair is unresolved (§4.4): it takes the
mesh path. The check only refuses. Once it passes, the union's material is
every returned cell, with no further per-cell classification. Assemble by taking the multiset of every selected
cell's boundary edges and dropping any edge that appears on **two** selected
cells (matched by `Entity` identity plus its unordered `{TStart, TEnd}` pair
— a shared wall between two adjacent selected cells, walked in opposite
senses by each), keeping every edge that appears on exactly one (a wall
against the unbounded exterior — the survivor is the union's own outer
boundary). Chain the survivors into a closed directed walk (the same
directed-edge-loop-closure shape `internal/facetedtopology/build.go`'s face-loop construction
already performs for the mesh boolean's own loops); a chain that does not
close into exactly one simple loop — a disjoint pair of footprints producing
two separate lumps, which the single-outer-loop `ProfileRecord` cannot
represent — is not resolved (§4.4).

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
material inside a tool hole is dropped); for `Intersect` with one operand fully inside the other
(B inside A, or A inside B when B has no hole), a proof cell must exist first: its outer loop reproduces the
enclosing operand's original `Outer` and its only hole reproduces the nested operand's original `Outer`. Exactly
one direction must match. With the proof cell found, the match is the nested operand's own cell: its `Outer` and
any admitted hole reproduce that operand's original loops. A structural match — entity identity, order, and `Whole`-ness, nothing
geometric — is a pure data comparison against decad's own tag map. **When a
unique such profile exists, it is not assembled at all: it is one of
`s.Profiles()`'s own results. `Cut` passes the matched profile through
`RecordProfileContext`; `Intersect` records the fresh, unchanged result through
`RecordArrangedProfileContext`. Both paths check each recorded edge's exact
range (§5).**
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

**A tangent contact is not clean nesting.** Sketch's arrangement neither cuts
nor flags a clean tangency: a tool circle or arc touching a target wall at one
interior point leaves both loops whole, so the match above succeeds. The cut
section would then hold two loops meeting at that point. The body pinches
there, and no cap mesh can separate the two loops. So every `Cut` match that
puts the tool's outer into the result as a hole — this one, stacked §7's
enclosing match (`prismcells.MatchEnclosingCut`) and general-boolean §3 A5's
group match (`prismcells.GroupCutMatch`) — also puts every pair of entities
from the two operands, or from two tool regions, with a curved member to
sketch's own intersection query, the query `prismcells.ProveGroupApart` reads
(stacked §2.2). The contact answer comes from that query
(`geom.LineCircleIntersections` and its circle and arc kin), not from the
arrangement's `TExact`. The query reports a point within its own band,
`1e-9·max(1, R)` with `R` the larger radius, and a line's point counts within
`1e-9` of the segment's parameter range. Any point it reports turns the match
into a miss, and the pair takes its next route, usually the mesh path. So a
near-tangent gap of `1e-9` mm or less (scaled by `R` past 1 mm) is refused as a
touch, and a larger gap keeps the exact route. The check only refuses: a
reported point never admits anything. On the mesh path a blind pocket builds an
`Approximate` faceted result. A tool through the whole target also takes the
mesh path, which may refuse the contact with `BooleanUnsupportedContact`: a
round tool tangent to a square hole or a box's side wall refuses there, while
one tangent inside a cylinder's wall builds a faceted result. Two lines go to
the line reading of `prismcells.RegionContact`, which these matches call for
both kinds and which `Union`'s groups and stacked interfaces read too (stacked
§2.2): sketch's `geom.LineLineIntersection` point, read along each segment
within `1e-9` of its parameter range. The arrangement splits two lines that
cross, two that share a span (a square pocket along a hole's wall) and two
corners that meet, so those touches never match whole. It does not split a
line where another line's endpoint touches its interior (a T-touch) in a scene
whose cells keep the two loops apart, as a `Union`'s can. In a `Cut` scene the
walk around the touching hole passes that point, so sketch cuts the wall there
and the match misses on the `Partial` edges first: blind, through, stacked and
enclosing T-touches with an apex on a hole wall or on the outer wall take the
mesh path before the line reading runs (`apitest/cut_tangent_contact_test.go`).
`Intersect`'s result and the no-op match of a tool inside an
existing hole (`prismcells.MatchCutNoOp`) are one operand's own unchanged
cell, whose loops meet nothing, so they take no query.

For the one-hole `Intersect` arm, the proof cell has A's whole outer loop
and B's whole outer loop as its only hole. A second cell must reproduce B's
whole outer and hole loops. Both cells must report `Valid`, and the result
cell alone becomes the payload's section. The proof cell excludes a disjoint
pair; the result cell preserves the void in B. The existing G3 plane gate,
G5 overlapping sweep intervals, work cap,
private-scene profile recording, and per-end bound selection apply unchanged.
The crossing classifier does not admit this arm.

**`Cut`/`Intersect`, crossing sub-case (boundary contact, not clean nesting):
implemented (`internal/prismcells/payload_resolve.go`), with the coincident-carrier
extension below.** When operand B's
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
`prismcells.Origin.AuthoredReversed` (`internal/prismcells/scene.go`)
records, once per created entity at scene-build time, whether that operand's
own recorded walk runs backwards relative to the entity's own natural
parameterization, so the comparison above is bookkeeping on a fact already
captured, never a fact `prismcells.Classify` derives from geometry. For a cell
with a direct edge from both operands, both readings are available and the
four combinations (`A-only`, `B-only`, `both`, impossible-for-two-simple-
curves-`neither`) select `Union`'s "either", `Cut`'s "A and not B",
`Intersect`'s "both". A cell with no direct edge from an operand takes that
operand's own classification from a neighboring cell reached by crossing an
edge that does NOT belong to that operand — crossing a non-operand edge
cannot move across that operand's own boundary, so membership carries over
unchanged; `prismcells.Classify` floods this from every directly-classified
cell.
For a cell touching only one operand directly but adjoining a **coincident
carrier** (the tooth-only cell of the gear's shared-arc case, which the
merged edge names under only one operand's entity), the shared span is a
boundary of BOTH operands. `prismcells.CoincidentEdges`
(`internal/prismcells/coincident.go`) reads which edges those are from
`sketch`'s own report, never from evaluating a point against either curve:
`sketch` withdraws the span from the losing entity as a window its fragments
do not cover, and the partner is the one entity of the other operand with
fragment ends at both window ends (Part G §3 A3).
The cell walking the span takes both operands' readings directly — its own
entity's flag comparison, and the partner's through the fixed relative
direction of the two entities — and `Classify` never propagates either
operand's membership across it. Read as an edge of the named operand alone,
the span is one the other operand's propagation may cross: the tooth-only
cell then inherits "inside the hub" from the hub's cell across the tooth's
own root arc, so `Intersect(hub, tooth)` would publish the whole tooth
(`TestPrismBooleanToothTouchingHubDoesNotOverlap`) and `Verify` would report the touching pair
`Interfering`. A
window no single partner explains leaves the scene unresolved (§4.4).

### 4.3 Assembly output

Every resolution path (§4.2) ends with a candidate `ProfileRecord`: either
one `s.Profiles()` result taken verbatim (the clean-nesting match, no
assembly), or a merged set of surviving boundary edges chained into a closed
loop (`Union`'s select-all case, and the crossing case). Each surviving
edge is converted to a `CurveSegment` through the **existing**, unmodified
`recordEdge`/`falsifyRange` pair in seam.go (package-internal, so this new
code calls it directly) — every fragment's `TExact` admission and reject-only
range falsifier applies exactly as it does for a caller-drawn profile,
because it is the same check on the same kind of `sketch.BoundaryEdge`,
regardless of who authored the input curves it was cut from.

### 4.4 What stays outside the admitted class

| Shape | Where it stands |
|---|---|
| Antiparallel, tilted, or non-analytic-segment pairs | G1–G4, mesh path, unchanged |
| A displaced pair whose operands' recorded vertices meet exactly, with no cut there | Known limit: §3.4's crossing charge reads cut vertices only, so a vertex where two recorded vertices meet is charged nothing, though the operands' displacements can open a crossing within `(δ_A + δ_B)/sin θ` of it; the result carries only §7's incoming terms there |
| A parallel-offset pair whose frames' `U`/`V` bits differ or whose transform bases differ | G3, mesh path; the blind-Cut translation arm requires identical frames and bases |
| A tool plane whose normal is genuinely reversed (a frame built with `V` flipped, so `N` opposes the target's), whichever way the tool is extruded | G3's co-directional requirement, mesh path. Admitting it needs a reflection of B's section (`V` reversed, arcs re-sensed) that §4's selection does not carry. A tool sketched on a same-normal offset plane above the target and extruded `Against` is not this case: `Extrude` keeps the sketch frame and records the negative interval `[−D, 0]`, so the shared-axis arm admits it with the exact shift (G3, G5) |
| `Union` with unequal z-intervals | overlapping or touching: a `stackedPrismPayload` (Part G §3 A1); disjoint: G5, mesh path |
| `Cut` whose tool ends inside the target and whose boundary crosses the target's (a side notch) | mesh path; where the mesh path refuses (a tool tangent to the outline at recorded vertices), general-boolean §3 A9's BRep |
| `Cut` whose tool lies strictly inside both target ends (an enclosed void) | G5, mesh path; a void shell is not a stage this design admits |
| Interface-aligned blind `Cut` whose displaced tool end or interface has no recorded denoted level, whose denoted levels differ, touching opposed holes, or section displacement | Equal denoted levels with monotone holes and proved separate opposed holes build, and overlapping opposed holes of axis-aligned lines, circles and arcs at an undisplaced level build general-boolean §3 A8's BRep; these cases take the mesh path (stacked §2.1, §7 stage 3) |
| `Cut` whose tool crosses an existing hole's boundary instead of enclosing it whole | no whole-loop match; a blind tool takes general-boolean §3 A7 (one hole), else the mesh path, and A9 where the mesh path refuses |
| `Intersect` with disjoint intervals | G5, mesh path (result is empty; unchanged `BooleanEmpty`) |
| `Union` with a holed operand | G6, mesh path; unequal intervals with a hole sharing a span with the other outline build as an A1 brep (general-boolean §3 "A1 as a brep", holed operands) |
| A split arranged boundary with a nonzero source displacement, a nonzero walk charge, or a nonidentity re-expression — any one of the three alone | §3.4 safety routing, mesh path; a future crossing-sensitivity proof may admit it |
| `Cut` with a holed tool | G6, mesh path; the surviving material inside each tool hole is a separate lump, so it waits on the multi-lump prism payload of the row below |
| `Intersect` with a holed operand outside §3.1's one-hole clean-nesting arm | G6, mesh path; the result may require several regions or a split boundary |
| A shared span `prismcells.CoincidentEdges` cannot attribute to one partner, or a cell walking a circular run past another cell's vertex | unresolved, mesh path (Part G §3 A3) |
| A displaced pair whose shared span lies inside or outside the result | mesh path: the true walls can part, and no recorded edge bounds the sliver (Part G §3 A3) |
| A holed `Cut` target whose tool does not clear it via clean nesting (the crossing classifier is scoped to hole-free operands on both sides, §4.2) | mesh path, except general-boolean §3 A7's blind Cut across one through hole, and A9's where the mesh path refuses; the clean-nesting path above still covers a holed target whose tool does not touch it |
| A `Cut` tool whose circle or arc touches a target wall at one point without crossing it (a round pocket tangent to a hole or to the outer wall) | mesh path through §4.2's tangent-contact query: a blind pocket builds a faceted result; a tool through the whole target takes the mesh path too, which may refuse it with `BooleanUnsupportedContact` |
| A `Cut`/`Intersect` selection covering two or more disjoint regions (a multi-region coplanar overlap) | no BODY is built: a `ProfileRecord` carries one outer loop, so resolution fails to close one (§4.2) and the pair takes the mesh path, waiting on a multi-lump prism payload that is not currently planned. `Verify`'s interference reading answers such a pair anyway, without a body, through §4.5's overlap-area reading |
| A disjoint-footprint `Union` (two separate lumps) | resolution fails to close one loop (§4.2); Part G §3 A5 builds a prism group when the loops are proven disjoint. `Verify` answers the pair through its disjoint/overlap partition (`agent-docs/verification-design.md` §1) rather than through §4.5 |
| A free-form (Tier A spline) segment that never touches the other operand | excluded by G4 (whole-scene `TExact` gate, §3.1) |

### 4.5 The overlap-area reading: a volume, never a body

`Verify`'s interference reading needs exactly two things from a pair: the
verdict that the two solids provably overlap, and a bounded volume for that
overlap (`agent-docs/interference-design.md` §1, §6). It does not need a `Body`.
Every limit §4.4 states for a multi-region `Cut`/`Intersect` is a limit on
ASSEMBLING one — a `ProfileRecord` carries a single outer loop, so a selection
covering two or more disjoint regions cannot become a section at all, and
`prismcells.ChainClosedSurvivors` correctly reports it unresolved. This reading
answers the volume without assembling anything, so a pair whose 2D outlines
overlap in several disjoint regions — a pair of meshing gears, which always
engages several teeth at once — is measured rather than left undecided.

The reading runs only from the read-only interference path, never from
`performBoolean`. `Union`, `Cut` and `Intersect` keep the behaviour §4.4
states, multi-region results included: a public boolean owes its caller a body,
and this reading builds none. §13 records that asymmetry as a deliberate
decision.

**Entry.** Identical to `Intersect`'s own, and shared with it unchanged: G1–G4
(`prismcells.AdmitPairBudget`), the trimmed-circular refusal
(`prismcells.ProfileHasTrimmedCircularSource`), G6's hole-free arms, G5's Intersect
z-relation (§3.2), the arrangement cap, the re-expression, `prismcells.BuildScene`,
`prismcells.Classify`, `prismcells.Select` under `Intersect`'s own `keep`,
and §3.4's crossing charge. The reading begins where `prismcells.Merge`
would have been called, and replaces only that tail.

**Selection.** The measured set is the arrangement's own bounded cells that
`prismcells.Classify` puts on BOTH operands' material sides. `sketch` returns
the arrangement's full planar decomposition (§4.2), whose cells have disjoint
interiors, so the selected set's total area is the sum of its cells' areas and
no cell is counted twice. That disjointness is `sketch`'s own answer about its
own arrangement, consumed as CLAUDE.md's carve-out allows — decad computes no
containment, no crossing and no membership of its own. The existing
reject-only structural checks inside `prismcells.Classify` are what falsify it:
a cell reporting `Valid == false`, a cell carrying its own hole, an edge
occurring on more than two cells, or an entity the scene did not create each
leave the whole reading unresolved.

**Per-cell measurement.** Each selected cell is already a closed directed walk
in `sketch`'s own order, so it is recorded through the existing
`recordEdge`/`edgeJoin`/`falsifyLoopJoins` sequence `prismcells.Merge` already
uses — minus the count, drop and chain steps, which exist only to build one
loop out of many. §5's authentication applies with its claim 2 discharged for
free, exactly as it is for the clean-nesting match: every recorded edge is one
of `s.Profiles()`'s own results and nothing is assembled from them, so §6's
build-time audit has no assembly to re-prove and does not run.

The recorded cell becomes a `prismPayload` carrying the pair's own frame,
placement and §3.2 Intersect z-interval, that cell's own §7 section
displacement, and the pair's per-end axial displacement. Its volume is then
`prismrecord.Build`'s, unchanged — the same region integrals, the same
`sectionDisplacementArea` charge over the cell's own perimeter, the same axial
terms, the same `Exactness` rule. No bound mechanism is restated at a second
site, which is what `internal/proofbound/bounds.go`'s one-helper-per-mechanism rule requires, and
no new helper is introduced.

<!-- "a volume, never a body" is scoped to ASSEMBLY and DELIVERY, not to
     allocation: this section's own wording is "a limit on ASSEMBLING one",
     "without assembling anything", "nothing is assembled from them", and "it
     assembles nothing". The paragraph above names `prismrecord.Build` as the per-cell
     volume path outright, and `prismrecord.Build` (extrude.go) does allocate a
     `Body` and read `body.volume` from it — that is the specified path, not a
     contradiction of it. What the reading never does is commit a `Step`,
     retire an operand, register a body in the `Document`, or return one to a
     caller; `agent-docs/interference-design.md` §5 states that promise for this
     reading beside the existing analytic twin's. -->

**The sum.** The reading publishes the charged sum of the cells' volumes: the
value is their float sum, whose own accumulated rounding `internal/proofbound/bounds.go`'s
`ExactSumRound` charges against the exact rational sum, and the bound is
`proofbound.AbsSumUpper` over the cells' own bounds and that rounding. `exactnessOf` then
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
unsupported. A genuinely overlapping pair whose arrangement splits a
boundary, where either operand carries a prior section displacement or walk
charge or B's re-expression is nonidentity, is measured: §3.4's crossing
charge enters each cell's displacement (§7), and a crossing too close to
tangent for that charge, or a cell such a pair cannot record, declines the
reading to the mesh path. The reading never publishes a
zero-volume overlap and never turns a contact into a row;
`agent-docs/interference-design.md` §6's positive-volume gate judges what it does
publish, unchanged.

**Refusals.** The reading crosses no point of no return: it assembles nothing,
so §9's RB2–RB6 cannot arise, and every shape it does not cover is the silent
fallback above. It returns only the errors every path already shares — the
arrangement cap (RB7, `ErrUnsupported`) and the caller's own cancellation —
plus RB8/RB9 on a cell `recordEdge` rejects or a cell loop that does not join,
surfaced exactly as they are on the body path so a broken arrangement is never
hidden as an undecided pair.

### 4.6 One-scene Union of three or more prisms

`Union(ctx, first, others...)` builds one section arrangement for three or more
same-frame, same-placement prisms whose sweep intervals G5 proves equal
(`prismplacement.UnionZIntervalMatches`: equal held floats and one shared
denoted rational at each end), and the result keeps that shared denotation.
Every
input section of the **line class** (`prismboolean.UnionMany`'s own gate) has
no holes, zero section displacement, and only whole `LineSeg` edges. Two input
edges on the same infinite line carrier are refused because this path has no
proof for their shared source span.

Every other set of prisms that meets U1 is the **extended class**
(Part K §5.3): whole `LineSeg`, `CircleSeg` and
`ArcSeg` sections, holed operands, and displaced operands whose own body
carries a valid carrier record (`prismrecord.CarrierRecordOf`; an operand with an
unowned record, or an arc whose Lemma H bound is infinite, is a miss). It runs one arrangement under the identity re-expression,
reads every arranged vertex where two operands meet through Lemma T, audits
the arrangement's stability under each operand's displacement (V1 to V3 and
carrier-record-design §6), and records the result's carriers. The extended
class never refuses. Every failure is a silent miss to the next `Union`
route (`unionmany.Routes`), as A6's `AmplifiedFallback` is for a pair, and
only cancellation returns an error. `StraightLoftUnionMany` reaches it too
when a converted set leaves the line class. A
circular prism joined to radial copies of one fitted section has a separate
certified path in Part P. Other multi-prism
classes enter `Union`'s certified two-body evaluator in source order after
the specialized multi-body gates miss. A pair refusal leaves every original
operand live.

Every source region enters `BuildSceneRegions` once, in the same plane. The
work cap applies to the sum of all input segments. `SplitRuns` resolves
shared stations, `CellsHaveNoVoid` refuses an enclosed void, and
`MergeLoops` keeps the exterior boundary; a boundary that does not chain into
closed loops, as where two loops meet at a shared corner, is a miss, and the
bodies join two at a time. Each resulting loop passes
`prismcells.AuditMergeSection`; multiple loops also pass
`prismcells.ProveGroupApart` (stacked §2.2). One loop builds a
`prismPayload`, while loops proven apart build one one-slab
`stackedPrismPayload`. Loops that touch at one point, such as a triangle's
apex on a square's side, which the arrangement leaves uncut, are no group:
the evaluator misses, and the bodies join two at a time, where the
touching pair takes the mesh path. The result carries
the maximum of the inputs' axial endpoint bounds and one cut-parameter
section charge. A group also carries its disjointness walk charge. One
document commit retires every input after result evaluation succeeds and
the context remains live.

### 4.7 Three-source prism pair retry

After the specialized set routes miss, a public three-body `Union(A, B, C)`
first runs its ordinary prism pair chain `(A ∪ B) ∪ C`. If and only if its
second pair refuses with `meshbool.CoarseHeldContactError` for operand zero,
retry `B ∪ C`, then `A ∪ (B ∪ C)`, through ordinary full dispatch. The source
eligibility checks in `tryPrismUnionChain` still apply, including its admitted
straight-source Lofts. Two-body calls, larger sets, first-pair failures,
other refusals and private pattern chains do not take this retry.

Associativity of regularized union proves the same set. Each alternate pair
keeps every ordinary boundary, occupied-volume, depth and refusal check.
Reject every warning from either alternate step, including a chain allowance;
no accepted geometry adjustment enters the identity. An ordinary miss returns
the original second-pair refusal. Cancellation and invariant failures surface.

The retry runs at most two full dispatches, each with its existing independent
limits. It has no shared whole-call work, record or byte meter. The preceding
failed fold also consumed its own per-pair allowances. Never describe these
as one whole-call allowance.

Reserve two private producer identities on success: the first belongs to
`Union(B, C)`, the second to `Union(A, BC)`. Each ordinary route carries its
source face ancestry. Publish only one operation with consumed inputs
`[A, B, C]` in caller order and the final body as its sole output. The failed
fold and the alternate intermediate remain private. Failure or cancellation
leaves every original live body, producer counter and history unchanged.
This route does not produce a mapped-path witness or relax the held-mesh gate.

The sequential prism fallback builds every intermediate body without
registering or retiring it. It assigns a distinct producer identity to each
pair result and reserves the full identity span when the final result commits.
It does not retry an error from the one-scene or specialized evaluator. Its
output may be analytic or faceted according to the pair evaluator; every pair
must pass its existing boundary and occupied-volume proof.

## 5. Authentication

For a disjoint whole-circle group, the source-clearance check may use exact
axis-aligned enclosing boxes. A circle uses its exact center and radius; a
line uses its endpoints; an arc uses a full carrier enclosed by an L1 radius.
The whole-arc allowance covers the held radius and pinned-end residual, and
the matched scene must reproduce its natural-order source record exactly.
Strict separation of expanded boxes proves a lower distance bound.
The expansion includes both recorded section displacements and the scene's
walk, reexpression, and crossing charges. Overlapping boxes refuse. This
check proves clearance only; Sketch's whole-loop arrangement still decides
section membership and assembly.

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
   `Intersect` passes the profile selected from its private scene's first
   `s.Profiles()` result to `RecordArrangedProfileContext` without changing
   the scene or profile. `Cut` uses `RecordProfileContext`, which also asks
   `sketch` for a fresh profile snapshot.
2. **The assembly is correct.** For the clean-nesting match this claim is
   free — the "assembly" is a single `s.Profiles()` result, unmodified, so
   claim 1 already covers it entirely. For `Union`'s merge, the *additional*
   fact needing proof is that dropping the shared edges and chaining the
   survivors produced a genuinely simple, correctly-wound, properly-nested
   loop — decad's own combinatorial assembly, not a `sketch` answer. This is
   the modify-design carve-out already in force for auditing a
   *decad-constructed* section (modify §4/§5's Table S, and the offset
   audit `offset2d.AuditOffsetSectionBudget` already runs on ITS OWN constructed section with
   an empty blend map): §6 runs the closed-form crossing/orientation/nesting
   checks the same way, on the merged record, as the second, independent
   proof. Neither of those checks proves the merged loop's own **closure**.
   `prismcells.ChainClosedSurvivors` resolves which survivor follows which in
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

Reuses the modify §5 machinery verbatim — `sectionaudit.LoopSignedArea`,
`sectionaudit.EntriesOf`, `sectionaudit.Crossing`, and `sectionaudit.Nesting`
take generic section records and walks with no fillet-specific coupling,
so the merged `ProfileRecord` feeds them directly with an empty
blend map (`offset2d.AuditOffsetSectionBudget`'s own precedent for "no cutback data, still run
the shared audit"). Order matches modify §4's:

| Step | Check | Sentinel |
|---|---|---|
| assembly | surviving edges chain into exactly one closed loop | not resolved (§4.4), mesh path — not a refusal |
| closure | the merged loop's own RECORDED coordinates join at every junction (seam §3's `edgeJoin`/`falsifyLoopJoins`, unmodified), run after the chain resolves — past §3.4's point of no return | `ErrUnrecordableProfile` (RB9), a refusal, not a reroute — the same treatment RB8 already gives a `recordEdge` rejection at this stage |
| S8-equiv | assembled loop's signed area does not flip/collapse | `ErrDegenerate` |
| S7-equiv | no non-adjacent segment pair crosses or contacts within the diameter-anchored `contactFloor` band (verification §4's noise floor, reused unchanged) | `ErrUnsupported` |
| S9-equiv | outer loop provably contains every hole (both operands' original holes survive into the result unchanged, since `Union`'s admitted class is hole-free per G6 — this step is a no-op while G6 keeps the union arm hole-free) | `ErrDegenerate` (decidably broken) / `ErrUnsupported` (undecidable) |

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
  endpoint (G5) is the one other place this design rounds, charged into that
  end's axial displacement below), composed with B's own walk charge the same
  way operand A's is, or
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
`|P(t) − P(t*)| ≤ |t − t*| · sup|dP/dt|`. `internal/proofbound/bounds.go`'s `CutParamUlps` states
the parameter allowance once — the quantitative reading decad gives `TExact`'s
"to machine precision" claim (`agent-docs/sketch-seam-design.md` §1) — and
`CutDisplacementAllow` multiplies it by the carrier's own speed over its full
parameterisation: the chord for a line, `2πR` for a circle or an arc.
`δ_cut` is the largest such allowance over the surviving fragments, and zero
when every survivor is a whole edge.

**The walk charge, `δ_walk`.** `prismcells.BuildScene` builds every entity from its
segment's own WALKED geometry (`walkOf`, §4.1), never from the record's
`Point2` fields directly. For a segment whose recorded range is the entity's
own full domain that walk restates the record's own coordinates exactly —
`momentregion.Lerp2`'s and `pinArcWalkEnds`' own natural-bound special cases, and a
`CircleSeg` recorded over those bounds walking the recorded centre and radius
directly — so a WHOLE segment charges nothing, whatever its kind. A `LineSeg`
recorded over a range NARROWER than that domain instead evaluates the carrier
at a computed parameter (`momentregion.Lerp2`'s general arm), so it enters the scene at an
endpoint this boolean itself computed, whatever the two operands' own prior
displacement was. It is the only kind that reaches this charge: a trimmed
circular carrier would enter through two `cos`/`sin`-computed points, moving
its rebuilt radius and sweep as well as its endpoints, and §4.1 refuses such a
pair before the scene is built rather than charge it (below). `internal/proofbound/bounds.go`'s
`WalkEndpointAllow` states the allowance, at the magnitude of the operands
that walk's OWN arithmetic touches and never at the endpoint it produced:
`momentregion.Lerp2` computes
`fl(a + fl(t·fl(b−a)))` from the carrier's own recorded `Start` and `End` — a
`float64()` conversion rounds the product before the add, so every target
commits all three roundings and walks the same point — and that difference
CANCELS, so a fragment near the plane
origin on a far-reaching carrier rounds by ulps of the CARRIER while its own
endpoint magnitude stays tiny. A trimmed `LineSeg` therefore charges the
envelope of its recorded `Start`/`End` coordinates (`prismcells.WalkChargeOf`'s
`lineWalkOperandUpper`), folded together with the walked endpoint so the
envelope stands whatever the recorded parameter is. `prismcells.WalkChargeOf` still
states a circular answer — `segmentWalk.coordUpper`, whose `|cu|+|cv|+r+r` L1
form already bounds the centre and radius a `cos`/`sin` walk works on — but
the §4.1 refusal means no boolean reaches that arm; it stands so a charge,
never a silent zero, is what any future widening of that refusal would meet.
Each operand owes the largest such allowance over its OWN
consumed segments — `δ_walkA` and `δ_walkB`, the `A` and `B` fields of
`prismcells.SceneDelta`, each holding that operand's walk charge ALONE — and the
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
order — see `prismcells.WalkChargeOf`'s own doc comment). WHOLE is read off the RECORDED
range for every kind, a `CircleSeg`
included, and never off the walk's own closed-ness: that flag is
decided within a tolerance of a full turn, and a decad-side tolerance that
can ACCEPT is the admission gate the reject-only rule forbids.

A pre-existing source displacement can additionally AMPLIFY at a cut: two
carriers crossing at an angle `θ`, displaced by `δ1` and `δ2`, meet up to
`(δ1 + δ2)/sin θ + min(δ1, δ2)` from where the recorded carriers meet.
Part G §3 A6 states that bound per cut over a
certified lower bound on `sin θ` and calls the largest one `crossing`
(`prismcells.CrossingCharge`); a span two operands share enters `crossing`
too, as its width bound (Part G §3 A3). It is about
amplifying an INPUT uncertainty;
it does nothing about the cut's own rounding, which is why `δ_cut` is
charged as well.

The rebuilt section therefore carries

```
δ = up( max( up(δ_A + δ_walkA), up(δ_B + δ_walkB + δ_reexpress), crossing ) + δ_cut )
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
rounding (`internal/proofbound/bounds.go`'s `ExactSumRound`).

It is **exactly zero in one decidable case**: both inputs carry zero
displacement, operand B's section map into A's frame is the identity under
G3's exact shared-axis or placed-axis arm (equal stored `U`/`V` and linear
bases, with a composed-origin difference exactly along the common axis),
**every surviving edge is whole, AND every consumed
source segment is whole**. Two caller-drawn profiles on one sketch plane with
no placement between them meet the first two conditions; they meet the third
and fourth only where the merge cut nothing — one profile strictly containing
the other, or two footprints meeting along complete shared walls, drawn (not
inherited from an earlier trim) so every one of their own segments spans its
entity's full domain. A partial overlap, which splits at least two walls,
never reaches the zero case, and neither does a union whose result the caller
would call "obviously exact": the recorded walk closes only to within
`δ_cut`, and a zero bound over it would be a claim the evaluator cannot make.
The zero case requires every CONSUMED segment whole, not only every SURVIVING edge, because an OPERAND can
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
incoming displacement. The G5 shift is an exact rational, so `Union` and `Cut`
add no new axial term. `Intersect` takes, at each end, the
winning operand's endpoint: the higher lower end and the lower upper end. That
end's displacement is the winner's own unless the loser's range, its held
level widened by its displacement, reaches past the winner's range (exact over
`big.Rat`); then the loser's true level may be the result, and the end takes
the larger of the two displacements, which covers either. A tie (exact
`big.Rat` equality) takes A's float with the larger of the two. When B's
shifted endpoint `z + s` wins it rounds to the nearest float once, and
`proof.RationalFloatError` over the exact rational charges that rounding into that
end's `z0Delta`/`z1Delta` beside the displacement. The rounding term is zero whenever `z + s` is a float
— every datum-plane offset by a whole number of millimetres — and positive
for an offset such as 0.1 mm under a 0.3 mm tool.

**The evaluator's current measurement path cannot carry `δ`, so this design
extends it.** `prismPayload` holds the section, the frame, the sweep interval,
the placement and its blend descriptors, and no coordinate-error term;
`prismrecord.Build` derives area and volume from the profile integrals and the z
endpoints alone; and those integrals bound only the rounding of their own
arithmetic over the recorded floats, so a line-only record yields a zero area
bound and `exactnessOf` publishes `Exact`. `prismrecord.Payload.PointBound` carries a
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
- **`internal/proofbound/bounds.go` gains the helpers for the mechanism**, under that file's own
  rule that each error mechanism has exactly one helper and no measurement site
  computes a bound inline. `CutDisplacementAllow` owns the cut-parameter
  mechanism above, turning `CutParamUlps` and a carrier's own speed into the
  coordinate displacement `δ_cut` reads. `WalkEndpointAllow` owns `δ_walk`'s
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
  `prismrecord.Build` composes those two terms into area (the area displacement once per
  cap, plus the sweep height × the walls' own length displacement), volume (the
  height upper bound × the area displacement), centroid (`δ` in plane, entering
  `prismrecord.Payload.PointBound`'s existing source term) and `Box` (`δ` outward on every
  face); the same length displacement stands beside every walk's own edge length
  and side-face area, and `δ` beside every junction vertex. The sweep
  interval adds no term of its own for `Union` or `Cut`, whose result interval
  is one operand's own endpoints verbatim (§3.2); `Intersect` may take a
  shifted endpoint, whose single rounding rides in that end's axial
  displacement (above), never in `δ`.

**Exact carriers.** `δ` is one bound for the whole section, but a `Cut` that
lands in §4.2's clean sub-case keeps the target's own loops whole and
verbatim; only the tool's loops carry its displacement. The result lists the
loops it keeps bit for bit from the target, wherever the target states their
carriers exactly — a target with zero displacement, or one whose own listing
names the loop — in `prismrecord.Payload.ExactCarriers`
(`prismrecord.ListExactCarriers`, read by `Payload.CarrierExact`). Each
listed segment's carrier is then a carrier of the denoted boundary: the
denoted result is the target less the true tool, so every boundary point it
takes from a listed loop lies on that loop's line or circle. The listing
refines no coordinate reading. A displaced tool loop may still reach a listed
loop within `δ`, so every vertex, length, area and box reading keeps `δ`;
only a reader of carriers, such as the faceted view's retained tags
(brep-modify §2.2), takes a listed loop as exact. The listing records the
`δ` it was made at, and `CarrierExact` answers false for every loop once the
payload's `δ` or the loop record differs. A copy that moves the record, such
as a pattern instance, clears it.

The result's `Exactness` is then the existing rule with these displacement
terms added:

- **`Exact`, zero bound**, when every surviving segment is a `LineSeg` **and**
  `δ == 0` and both axial end bounds are zero — `internal/momentregion`'s region-level
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
displacement term (evaluator §4, `internal/momentregion`), so no separate derivation is
needed for each.

## 8. Consequences removed, and what stands outside the admitted class

| Consequence (§1) | Admitted class | Outside it |
|---|---|---|
| 1. No chaining | Removed. Result is `prismPayload`; no `MeshBound` to compose, so no chord tolerance for the next pair to fall below. A chained boolean re-checks §3's gate on the new pair, carries the greater incoming displacement plus its new re-expression and cut displacement, and retains the greater incoming axial displacement at each end (§7). | Unchanged — general-position or non-analytic pairs still degrade per evaluator §9. |
| 2. Coplanar contact refuses | Removed. Coplanar, co-directional contact is the admitted case's whole premise. | Unchanged — non-coplanar or non-prism coplanar contact (e.g. a prism against a revolve cap) stays on the mesh path. |
| 3. Analytic identity dies | Removed where `δ == 0` — a merge that cut nothing. Result is `prismPayload`: Fillet/Chamfer/Shell, all three surveys, and the clearance kernel already dispatch on payload class and need zero new code for it. Where `δ > 0` — which every cut-bearing merge is, §7 — the result is a `prismPayload` still, and every consumer's reach there is §12's own rows to state, `Verify`'s tolerance gate included; the reference that gate anchors against is verification design §3's to define. A `Fillet` or `Chamfer` of a displaced result reaches its lateral corners where the result carries a valid carrier record (Part K §3, §7): the pair `Union` of two undisplaced operands, the extended one-scene `Union` (§4.6), a clean-nesting `Cut` and a frame-keeping pattern instance set one. Any other displacement keeps §12's rows, and so does a cap-loop selection unless every selected loop is exact in the record, which builds a `capblend.DisplacedPayload` (carrier-record-design §8). | Unchanged for mesh-path results — `facetedPayload` still permanently refuses modify ops (modify-reach SX9) and all three surveys. |

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
| RB7 | The bounded work budget (§10) exhausts, or the private scene exceeds `prismcells.MaxArrangementSegments`, before resolution or the audit completes | `ErrUnsupported` | No — a coarser/simpler input may clear it |
| RB8 | `recordEdge`/`falsifyRange` rejects a surviving segment (`TExact` disproven on a merged edge — an internal `sketch` inconsistency, reported upstream per seam §3) | `ErrUnrecordableProfile` | No, but should not occur on a certified arrangement; a defensive check |
| RB9 | The merged loop's recorded segments do not join (`falsifyLoopJoins`, seam §3) — a whole-to-whole junction the assembly restates as two segments' own defining coordinates, which the merge did not compute and so did not round to agreement. Not RB8's class: the mismatch is inherited from an operand's own record (typically one carrying `Partial` cut fragments), not an internal `sketch` inconsistency | `ErrUnrecordableProfile` | No — a differently-drawn operand may close |
| RB10 | A stacked result's slabs fail Part S §2.2's audit after every slab's scene resolved — the per-slab reproductions of one target loop differ, or an interface is not monotone | `ErrUnsupported` | No — the audit reads the records this pair produced |

None of these rows is permanent in the modify-reach SX9 sense: every one is a
property of the specific pair's geometry, not a structural class exclusion —
a differently-authored model of the same intent can clear it.

## 10. Work budget and cancellation

Reuses `internal/proofbound/budget.go`'s `proofbound.WorkBudget` (`NewWorkBudget(ctx)`), the same
shared counter Fillet/Chamfer/Shell already thread through their own audits
(`agent-docs/modify-design.md` §5). One counter is opened per analytic boolean
attempt and threaded through the G4 segment scan, scene construction (one
charge per created entity), the §4.2 selection/merge walk (one charge per
candidate edge or cell touched), and §6's audit (its budget parameter). It is
polled at phase boundaries and at least every 256 candidates. `prismcells.MaxArrangementSegments` is the
single code-owned pre-`Profiles` cap: it bounds the private arrangers'
tiny-segment pair work for the pinned line/circle/arc density. That pair work
is one `O(n^2)` pass with no broadphase, so the cap bounds a single
uninterruptible arrangement; the value in `internal/prismcells/work_cap.go` states what it
costs and what it admits. The screen `prismcells.RegionsWithinWorkCap` counts a
line as 1 and a circular segment as `prismcells.CircularSegmentCount`: 257 for
a whole circle, and for an arc `min(257, max(2, ⌈256·(turn + 2⁻⁴⁰)⌉ + 1))`
with `turn` the sweep as sketch's `wrapSweep` reads it (`math.Mod`), times a
narrowed range. The scene builders (`prismcells.BuildScene`,
`BuildSceneRegions`, `classbgeom.CreateNaturalSegment`) then recount every
entity they create with sketch's own densification over the entity's floats and
return `prismcells.WithWorkLimit` when the sum exceeds the cap. A caller whose
over-cap answer is a silent miss maps that `*WorkLimitError` to its miss;
every other caller returns it (Part K §4). `sketch.Sketch.Profiles` is synchronous and has no
context parameter, so a capped private
scene runs in one worker while the caller selects its result against
`ctx.Done()`. On cancellation, the caller waits for that bounded arrangement
worker to finish, discards its result, and then returns `ctx.Err()` before the
arrangement reaches the document.
`Union`/`Cut`/`Intersect` add no public cancellation surface beyond the context
they already take. Exhaustion is RB7 (§9), not silent truncation.

## 11. Topology, provenance, and roles

The result is built by `prismrecord.Build` over the merged record, under the boolean's
own private producer identity — identical to how Fillet/Chamfer/Shell build, and
the topology it builds is untouched by §7's added displacement term. Faces
get **fresh** roles (`side(i,j)`/`capStart`/`capEnd`) minted from the merged
record's own segment positions, per decad's settled role rule (modify §9/§12:
"a role indexes the record it labels"). Those fresh roles stay in
`topo.Face.Origins`, which tessellators and modify ops key on.

**A planar result face also inherits an operand face's roles when exact plane
identity proves it lies inside exactly one operand face.**
`internal/boolupstream` runs after every route `booleandispatch.Analytic` takes
and stores the inherited roles in `topo.Face.Upstream`, a field separate from
`Origins`. Public `Face.Origins()` returns both, and `FaceCreatedBy`,
`CreatedBy` and `OuterLoopOf` match either. The rule, per result face F:

1. F needs a `CapRecord` plane read exactly (`capplane.OfFace`): the record's
   denoted level `ZDenoted`, or its held level when F states a zero axial
   displacement. Any other F inherits nothing.
2. F's outward side s on that plane P fixes the side an operand face must
   bound F from: s for both operands of a `Union` or `Intersect` and for a
   `Cut`'s target, −s for a `Cut`'s tool. The result's boundary lies in the
   operands' boundaries, and an operand face on P with the other side cannot
   bound the result there.
3. Every face of both operands is placed against P exactly. A `CapRecord` face
   with a `Plane` tag whose record normal is not parallel to P's, or whose
   exact plane is another plane, is off P. A swept wall of a prism, stacked or
   brep record whose sweep direction crosses P (`capplane.Plane.Crosses`) meets
   P in a set of zero area, so it is off P. Any other face is undecided: a
   displaced cap with no denoted level, a `Faceted` or retained tag, a wall
   parallel to P, a band patch, a face with no `CapRecord` in any other
   payload class.
4. F inherits the origins, own and upstream, of operand face G only when no
   face is undecided and G is the one face of either operand on P with the
   side step 2 asks for. Two candidates, as at the floor two flush boxes share,
   leave F with nothing.

The table is keyed by F's own role names, so faces sharing one role list must
all inherit the same roles, or none of them does. It rides in the payload
(`Upstream` on the prism, stacked and brep records). Build stores the payload
without it, and only `Placed` re-attaches it, so a `Transform` or `Copy` keeps
the inherited roles while a modify op that derives a new record from the
payload carries none. A result of any other payload class (a Loft, revolve or
faceted route) inherits nothing from this pass.

**A face that descends from another keeps the two lists apart.** The mesh path
copies an operand face's `Origins` into the result face's `Origins` and its
`Upstream` into `Upstream` (`facetedrecord.Group.Upstream`), as do `ScaledAt`,
`Stitch`, `Unstitch` and `PatchEdges`. Every recognizer that matches a face by
its roles reads `Origins` alone, so an operand that inherited a role is
recognized exactly as one that did not: the nested-boss shell's opening
(`prismshell`, one own cap role), the prism-union fillet's walls, and the
oblique-boss fillet and cross-bore mouth, whose rebuilt faces copy the
operand faces' own roles only because they are modify results, which inherit
no Boolean roles. A many-body
`Union` that a one-scene, pattern or Loft route takes as a set inherits
nothing; its pair chain and the two-at-a-time fold run the pass at each step,
since each step goes through `booleandispatch.Body`. A caller that needs a
face this rule leaves unnamed selects by geometric predicate (`Facing`,
`NormalTo`, `Circular`).

## 12. Downstream consumers

| Consumer | Effect |
|---|---|
| Tessellation | The result is an ordinary `prismPayload` and `agent-docs/tessellation-design.md` §5's prism contract applies, with §7's section displacement as a term of its own there: it displaces the analytic boundary a mesh approximates, so it does not ride in that contract's stored-coordinate rounding term, while the per-end axial displacement still does. Tessellation §5 reserves the section displacement from the requested tolerance before chording, refuses a tolerance it exhausts, and charges it to every face bound and to `areaSlack`, so chording plus that displacement stays within `tol`. The reservation covers no other term: the per-end axial displacement rides on top of it and can lift the published `Bound` above `tol`, which tessellation §1's Tolerance row allows. A mesh of an assembled body is `Exact`-trimmed only where every one of those displacements is zero. |
| `ThroughAll` / `ThroughAllSide` | The extent reading states its interval over the RECORDED section and does not carry §7's section displacement, so at `δ > 0` the stop returns `ErrUnsupported`: it has no stated displacement to charge to the level it resolves. At `δ == 0` it reads the extent as before, beside whatever displacement that reading publishes on its own account. |
| Clearance kernel | Unchanged where `δ == 0` — dispatches on payload class; `prismPayload` already has full analytic support (`internal/clearance/pairkernel`'s coplanar `Plane`×`Plane` certificate, `offsetPair`, etc.). Where `δ > 0` the kernel builds no model for the body and the pair reads `Suspect`: every certificate it emits is an exact statement about the carriers it read, and a carrier the payload holds only within `δ` of the one it denotes cannot support one. Widening the kernel's own candidate intervals by `δ` is a separate piece of work, not this design's. |
| Interference (`Verify`) | **Reads the analytic path through two read-only entry points, tried in order.** After its containment and represented-set-equality certificates, `internal/verifyrun/interference.go`'s `MeasuredInterference` calls `booleandispatch.AnalyticIntersect` (`boolean.go`) — a read-only twin of `performBoolean`'s analytic dispatch that runs `prismboolean.Try`/`prismrecord.Build` under a temporary producer identity and never commits, so it consumes neither operand — and publishes the built payload's own volume. A pair that twin does not admit (`ok == false`) next reaches §4.5's overlap-area reading, which measures a selection covering any number of disjoint regions and publishes a volume with no body at all; the two-stage order keeps every pair the twin already answers byte-identical. A pair neither admits falls back unchanged to `evaluateBoolean`'s read-only mesh intersection. Both analytic answers are still subject to §6's positive-volume gate (`agent-docs/interference-design.md` §6). "Admitted" covers `Union`'s select-all path, `Cut`/`Intersect`'s clean-nesting sub-case, the crossing sub-case, and a multi-region coplanar overlap. `agent-docs/interference-design.md` §5.2 records the boundary this closes. |
| Surveys (wall/undercut/min-radius) | No new code — they dispatch on payload class, and support is immediate where `δ == 0`. The undercut reading is a normal-direction membership and is unaffected at any `δ`. The wall and min-radius readings publish the bound their own arithmetic proves at `δ == 0` — a candidate's own division or square root, never the section displacement — and are staged at `δ > 0`, answering undecided (`Suspect`, never a silent pass): `δ` is not a term either reading may absorb into that bound, because the wall reading is not a quantity a displacement widens by a fixed amount — its allowance-angle contact families (verification §6) can change membership under a boundary perturbation, so a proven displaced reading needs the survey's own theory extended, not a term added to a bound. |
| `Verify`'s structural/tolerance gates | Structurally unchanged — `prismPayload` is valid by construction as always. The TOLERANCE gate anchors each reading against a reference the body's own geometry supplies, and at `δ > 0` it cannot read that geometry through the clearance kernel's model, which the row above declines to build. It reads station witnesses on the body's OWN recorded section instead (`gatewitness.Prism`), each charged its own lift's gap, and hands them to the same shared reader every arm publishes through: `diameter.PointsWithBudget` states the winning pair's own distance computed over exact rationals and rounded toward zero, never the float scan's norm. The gate then shrinks that station maximum by twice the SUM of `δ`, the axial displacement and the widest station gap, rounding the shrunken value toward zero as well (`diameter.LowerForDisplacement`), which verification design §3 proves a lower bound on the denoted body's own diameter. A displaced body is therefore judged on the same terms as any other: a reading passes when its bound meets the tolerance and reports `Suspect` with a stated `Required` threshold when it does not. |
| `Fillet` / `Chamfer` | A result with `δ > 0` and no valid carrier record keeps `prismrecord.Payload.RequireExactSection`'s refusal (brep-modify RB6). A result with a valid record (Part K §3; the record is bound to the body that `prismrecord.BuildWithRecord` minted it for, `prismrecord.Build` and `Placed` drop it, and readers use `prismrecord.CarrierRecordOf`) takes the lateral carrier route for lateral edges (§7): a `Fillet` refusal of Table CB is `ErrUnsupported`, and a `Chamfer` refusal falls through to RB6's route. A cap-loop `Chamfer` whose selected loops are all exact in the record builds a `capblend.DisplacedPayload` (Part K §8); any other cap-loop `Chamfer` keeps RB6. |
| Export (STL/OBJ) | Reads `Tessellate`'s output. Its size-derived default tolerance is raised past `δ`, which tessellation reserves from the tolerance before chording, so a default export of an assembled body still writes its mesh rather than refusing. |
| Immediate calls | Public `Union`/`Cut`/`Intersect` signatures and consuming behavior stay unchanged. Qualified pairs use the analytic path internally; other pairs keep the mesh path. |

## 13. Decisions the user may want to overturn

- **§3.1 G3's exactness is unconditional** — no tolerance admits a
  near-but-not-exactly coplanar pair, even though `sketch`'s own carrier
  merge (§4.1) uses a round-off band internally. Loosening G3 would let a
  caller's placement rounding silently decide whether a pair routes through
  the analytic or mesh path with different exactness claims for
  visually-identical models; kept strict on the reject-only rule, at the cost
  of `RotationAround`'s known under-triggering (§3.3).
- **§3.1 G3's axis arms admit only equal stored `U`/`V` and transform bases
  with an exact composed-origin difference along the transformed normal.**
  `CreateOffsetPlane` on an axis-aligned datum and `Copy` along that
  axis both qualify. No orthonormality assumption on the stored frame enters
  the proof: the shared section coefficients are identical term for term.
  The blind-Cut translation arm above reads all three exact coefficients and never decides a cap relation from
  the rounded section translation. The
  coplanar arm keeps its float dot product against literal zero, while both
  axis arms use exact dyadic cross products over stored floats.
- **Opposite normals stay outside G3.** A tool sketched on a plane whose
  normal is genuinely reversed (a frame built with `V` flipped) sweeps the
  same solid as one sketched on the target's own orientation, but its
  section is mirrored relative to A's frame (`V` reversed, every arc's sense
  flipped), and §4's selection reads the recorded orientation as authored.
  Routing it to the mesh path is the sound choice until a reflection of B's
  record is specified. Extruding `Against` does not reverse a normal:
  `Extrude` keeps the sketch frame and records the negative interval
  `[−D, 0]`, so a tool sketched on `CreateOffsetPlane` above the target and
  extruded `Against` shares A's frame bits and is admitted by the shared-axis
  arm with the exact shift, its section copied verbatim.
- **§11's provenance rule is reject-only.** A result face inherits an
  operand's roles only where exact plane identity places every operand face,
  so a face on a plane parallel to an operand's sweep direction inherits
  nothing even where the operand's walls lie elsewhere: deciding a wall's
  plane would need its record segment lifted exactly, which the pass does not
  read. Extending step 3 that way adds reach without changing the rule.
- **A nonzero `δ` stages the readings §12 names, and every modify op.** A
  modify op rewrites the receiver's recorded section (modify §2), and a blend
  centre or an offset miter amplifies a displacement by the corner geometry —
  by an unbounded factor at a shallow corner — so `Fillet`/`Chamfer`/`Shell`
  refuse a displaced receiver (`ErrUnsupported`) rather than publish a result
  whose bounds speak only for the record. The corner rewrite keeps the rule
  for a prism receiver; a class-B brep receiver takes brep-modify §4.4, which
  blends from the record's exact carriers, and a prism `Chamfer` whose edges
  lie on loops with exact carriers (§7) takes brep-modify §2.2's faceted
  view. Every one of these stagings is
  invisible at `δ == 0`, which covers every body a caller draws and every
  union whose sources and re-expression carry zero displacement.
- **§4.5's overlap-area reading answers a multi-region pair for `Verify`
  alone; the public booleans keep refusing it.** `Union`/`Cut`/`Intersect` owe
  their caller a `Body`, and a body covering several disjoint lumps needs the
  multi-lump prism payload §4.4 still stages — changing `prismPayload`'s own
  contract would ripple through topology, tessellation, every survey, the
  clearance kernel, the modify ops and §4.1's equality certificate, which is
  the opposite of the smallest sound extension. `Verify` needs no body, so it
  is given the volume alone. The cost is a visible asymmetry: `Intersect` still routes to the mesh
  path a pair for which `Verify` reports an `Interference`. Two alternatives were rejected. Extending `prismcells.Merge` to emit
  several loops cannot work at all — a `ProfileRecord` has one `Outer`.
  Restating the volume composition inside the reading, rather than building one
  `prismPayload` per cell and reusing `prismrecord.Build`, would put a second owner on
  `internal/proofbound/bounds.go`'s section-displacement and axial mechanisms, which that file's
  own rule forbids.
- **§3.1 G6 restricts `Union` to hole-free operands and `Cut` to a hole-free
  tool.** The union restriction keeps the gear's workload (hole-free hub+teeth
  union, clean-nesting bore cut) off the harder per-cell classification
  (§4.2's crossing sub-case). The tool restriction is structural: a holed tool leaves a lump
  per tool hole, so it waits on the multi-lump prism payload §4.4 names, and a
  consumer that wants that cut analytically can express the same intent as one
  `Cut` per hole-free tool region.

## 14. Increments

All seven shipped. Each item names what exists and the tests that pin it; §15 lists the tests.

1. **PR1 — gate + scene construction + `Union`'s select-all path.** G1–G6,
   §4.1's scene builder and re-expression, `Union`'s select-all/merge/chain,
   §6's audit reuse, §5's authentication, §7's exactness (the section
   displacement bound on `prismPayload`, its helpers in `internal/proofbound/bounds.go`, and
   `prismrecord.Build`'s composition of it), and `performBoolean`'s branch before
   `evaluateBoolean` that builds via `prismrecord.Build` instead of `facetedrecord.Build`
   on admission. A split boundary routes to the mesh path before recording
   when either source carries a section displacement, either source carries a
   nonzero walk charge (§7's `δ_walk`), or B's re-expression is nonidentity —
   any one of the three alone (§3.4).
   Tests: a two-box union sharing a cap plane builds analytically, with `Exact`
   volume where both boxes sit on one frame (§7's `δ == 0` case); the gear's
   tooth-on-hub shared-carrier union builds with the correct region set and an
   `Approximate` volume within the composed bound of the closed-form Pappus/
   Green's-theorem answer; a non-coplanar pair still takes the mesh path
   unchanged.
2. **PR2 — `Cut`/`Intersect` clean-nesting sub-case.** The structural
   whole-loop match (§4.2) over PR1's scene infrastructure and §5's
   profile recording. Tests: a bore cut through a hole-free hub builds
   analytically with the correct hole added and the target's original outer loop
   byte-identical to its pre-cut record; an `Intersect` of a fully-nested pair
   returns the inner operand's own geometry.
3. **PR3 — crossing sub-case for `Cut`/`Intersect`.** §4.2's edge-orientation
   propagation over hole-free operands (`internal/prismcells/payload_resolve.go`), over
   PR1's `prismcells.Merge` chain/merge tail and §6's audit of its own
   per-op cell selection. Tests: two overlapping (not coincident-carrier)
   prisms cut/intersect correctly against the mesh path's own answer on the
   same pair (property test, volumes agree within the analytic path's
   tighter bound). A cell reachable only through a coincident carrier reads
   it through Part G §3 A3. G6's `Union` arm still
   excludes a holed operand; its `Cut` tool arm is structural (§13).
4. **PR4 — interference wiring.** Public boolean calls use the analytic
   path when a pair qualifies, pinned by direct construction tests on admitted
   coplanar pairs. `internal/verifyrun/interference.go`'s `MeasuredInterference` reaches the same
   read-only analytic intersection dispatch `performBoolean` uses
   (`booleandispatch.AnalyticIntersect`) after its containment and
   represented-set-equality certificates, falling back unchanged to the mesh
   path when the analytic path does not admit the pair; a test asserts an
   exact overlap volume on an admitted coplanar-prism pair that meets §7's
   exactness conditions. `agent-docs/interference-design.md` §5.2 and
   `agent-docs/evaluator-design.md` §9 point to this document.
5. **PR5 — §4.5's multi-region overlap-area reading.** The per-cell record,
   per-cell `prismPayload`, `prismrecord.Build` volume and charged sum, entered after
   `booleandispatch.AnalyticIntersect` returns `ok == false`, over the shared
   admission/scene/classification of PR1–PR3. It adds no bound helper and
   changes nothing in the public `Union`/`Cut`/`Intersect` surface.
   Tests: §15's multi-region rows.
6. **PR6 — G3's shared-axis arm.** A tool sketched on `CreateOffsetPlane` of
   the target's datum plane is admitted with an identity re-expression and
   G5's exact rational shift; `Intersect`'s shifted endpoint rounding is
   charged into the axial displacement. The mesh path's chain-depth refusal
   names the operand and states that a boolean takes no tolerance. Tests:
   §15's offset-plane rows.
7. **PR7 — the blind `Cut`.** It rests on G3's shared-axis offset-plane arm
   and the exact rational G5. §3.2's two blind rows over `prismcells.ResolveCut`'s
   clean-nesting match build a `stackedPrismPayload`, with its body,
   measurements and tessellation, and a spanning `Cut` on a stacked target
   runs the match once per slab. `prismboolean.TryBlindStackedCut`,
   `TryStackedBlindCut` and `TryStackedThroughCut` return the admitted
   `stackedPrismPayload`, and `performBoolean` builds it through
   `stackedrecord.Build`. Tests: Part S §8. Part S §7 owns the later stages.

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
  same model built via a hand-constructed `FromBasis` placement clears G3, and
  its root arc shares the hub circle as a span `Union` keeps inside its
  result while the placement displaces the tooth, which no charge covers
  (Part G §3 A3), so it falls back to the mesh
  path. The test
  must cover
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
- A trimmed source segment's walked endpoint is charged (§7's δ_walk). Two tests pin it:
  `TestReflectedRewindReversesLoopAndChargesTrimmedLine` (`internal/prismcells`) asserts that
  `prismcells.WalkChargeOf` is positive for a `LineSeg` ranged to `TEnd: 0.5` and that `RewindLoop` returns that
  same charge bit for bit, and `TestCleanNestingCutRecordNarrowedLines` asserts that a clean-nesting `Cut` over
  lines narrowed to `0.25..0.75` carries a walk allowance in its carrier record's entries. No test yet runs a
  trimmed-fragment operand through `Union`, `Cut` and `Intersect` and compares each published bound with an
  exact `math/big.Rat` residual, and none checks that the all-whole variant publishes a `sectionDelta` of exactly
  `0.0`. No test drives `WalkChargeOf` over lines whose `End − Start` cancels in `momentregion.Lerp2`, and none
  compares the line operand envelope against the walked endpoint's own `coordUpper`.
- The trimmed-circular refusal reads the RECORDED range, not the walk's closed-ness: a `CircleSeg` whose `TEnd` is
  one ulp short of `1` refuses all three ops (`ok == false`, no error), while the same pair with the bound
  recorded exactly resolves analytically. `ProfileHasTrimmedCircularSource` (`internal/prismcells/charges.go`)
  implements the refusal; no test pins it through a boolean. The only test with `TEnd: math.Nextafter(1, 0)` on a
  `CircleSeg` is `apitest` `TestRegionMomentsRejectMalformedTopology`'s `NearlyFullCircle` row, which asserts
  `decad.ErrDegenerate` from the moment readers.
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
  a bound composed from `internal/momentregion`'s and `internal/proofbound/bounds.go`'s machinery, asserted
  against the closed-form answer.
- An arranged profile containing a `Partial` boundary edge, where either
  source carries a nonzero section displacement, either source carries a
  nonzero walk charge (`δ_walk`), or B's re-expression is nonidentity, builds
  with §3.4's crossing charge in its section displacement. The focused
  fixtures must prove the arrangement splits, including an identity second
  re-expression after a displaced first result, then assert the published
  displacement covers the input displacement divided by the crossing sine.
- Downstream chaining: fillet a corner of an analytically-unioned body and
  read `Wall.Minimum` on the result — both refuse today (SX9, all three
  surveys) on a mesh-path union of the same model, and both succeed here.
- A second boolean consuming the first's result as operand B under a
  nonidentity re-expression carries B's prior displacement plus its new
  allowance — assert `Exactness`/`Bound` on the second result follow §7,
  not accumulated tessellation tolerance.
- Cancellation: a canceled `ctx` mid-resolution returns `ctx.Err()` unchanged
  with the document untouched, matching the existing modify-op
  contract.
- Arrangement cap: two line-only prism records whose combined upper bound
  exceeds `prismcells.MaxArrangementSegments` refuse with `ErrUnsupported`
  before `s.Profiles()` runs.
- Repeated direct construction of an admitted union produces matching
  `Exactness`, `Bound`, topology roles, and measurements.
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
  `booleandispatch.AnalyticIntersect`, with the same bound they report before §4.5
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
  whose scene exceeds `prismcells.MaxArrangementSegments` reports `Suspect` through
  the existing `ErrUnsupported` mapping, with no report-level error.
- G3's shared-axis arm, end to end: a 96×68×16 plate on XY cut by three
  cylinder tools sketched on `CreateOffsetPlane(XY, −16)` and extruded 48 mm
  builds three analytic results in a row (no `Faceted` face), each volume
  within 1e-6 mm³ of the closed form `104448 − π·18²·16 − 2·π·7²·16` at its
  step with a bound below 1e-9 mm³, a `sectionDelta` of exactly `0.0` and
  `Fillet` still available on the third result. The same tools on
  `CreateOffsetPlane(XY, 0)` and on XY itself give the same three volumes.
- The shared-axis arm's exclusions each take the mesh path: a tool whose
  offset plane is `CreatePlaneFromFrame` with an in-plane origin component;
  a tool sketched on an offset of a tilted base plane (the test asserts the
  two frames' `U` differ in the stored bits, so the fixture cannot silently
  stop being the one under test); a tool moved along the normal and sideways
  through `Transform`; and a tool sketched on a plane above the
  target whose `V` is flipped, so its normal opposes the target's. Each
  asserts a `Faceted` face on the result, or the mesh path's own refusal
  where that path refuses the pair.
- A tool sketched on `CreateOffsetPlane(XY, 32)` above the 16 mm plate and
  extruded `Against` 48 mm (the same z −16..32 solid) is admitted by the
  shared-axis arm: the result has no `Faceted` face, its volume is within
  1e-6 mm³ of `104448 − π·18²·16`, and its bound is below 1e-9 mm³.
- G5's exactness over the shift: a `Cut` whose tool cap meets the target cap
  exactly after the shift (offset −16, tool 32 mm tall under a 16 mm target,
  so `z1' == 16 == z1_target` exactly) is admitted, and a tool whose shifted
  cap is a hair short is not. The short fixture is a synthetic pair whose
  frame origins differ by `fl(0.4)` along `N` and whose tool `z1` is
  `fl(15.6)`: the FLOAT sum `15.6 + 0.4` is exactly `16` while the exact
  rational sum `fl(15.6) + fl(0.4)` is below `16`. The test asserts both
  premises itself, so a float-sum comparison would admit the pair and the
  `big.Rat` comparison refuses it (`prismplacement.CutZIntervalSpans == false`).
- `Intersect`'s shifted-endpoint charge: a 1 mm target on XY intersected with
  a 0.3 mm tool on `CreateOffsetPlane(XY, 0.1)` publishes `z0Delta == 0`
  (B's `z0' = fl(0.1)` is a float) and `z1Delta > 0` equal to
  `proof.RationalFloatError(fl(0.1) + fl(0.3), z1)` (no float), and the published
  volume bound contains the exact rational volume `area · (fl(0.3))`. The
  fixture is shown to fail with the charge deleted.
- The chain-depth refusal's wording: two tools transformed 16 mm down and
  0.125 mm sideways (the exact axis arms exclude the sideways offset)
  leave rims coarser than the next pair's chord tolerance. A third tool clear
  of them cuts; a pin placed on the second hole's rim is refused with a plain
  `ErrUnsupported` that is not a `BooleanError`, names the `target`, quotes the
  touched bound and the pair's chord tolerance, states that a boolean takes no
  tolerance, and contains no "retry with a tolerance". `Body.Tessellate` on a faceted body at
  a finer tolerance keeps its own "retry with a tolerance of at least" text.

## Implementation notes

Every `sectionDelta` consumer below is a no-op at the zero every caller-drawn
payload carries. `prismrecord.Build`'s composition (`internal/prismrecord/build.go`) and `tessellate.go`'s
tolerance reservation, mesh-bound charge and area-slack charge are §7's own
subject and are not repeated here.

§12 already rules on four of the remaining consumers, and this section adds
only the call site each of its rows lands at. What the consumer does at
`δ > 0` is that row's to state, and is deliberately not restated here:
`wallsurvey.PrismWall` and `radiussurvey.Prism` are §12's "Surveys
(wall/undercut/min-radius)" row, `internal/prismrecord/extent.go`'s `ExtentAlong` is its
"`ThroughAll` / `ThroughAllSide`" row, `internal/clearance`'s
`BuildPrismBodyCarriers` is its "Clearance kernel" row, and `internal/gatewitness`'s
`Prism` is its "`Verify`'s structural/tolerance gates" row.

No §12 row rules on the two remaining consumers, and this section owns them.
Each withholds its answer rather than measure the recorded section as the one
it denotes:

- `internal/verifyrun/interference.go`'s `analyticBodiesEqual` withholds the equal-records
  set-identity certificate: two records being equal says nothing about two
  sets each record is only within its own `δ` of.
- `prismrecord.Payload.RequireExactSection` refuses the receiver for
  `Fillet`/`Chamfer`/`Shell` with `ErrUnsupported`, since a rewrite of that
  section has no proven displacement of its own. A `Chamfer` it refuses
  first tries brep-modify §2.2's faceted view, which rewrites no recorded
  coordinate.

### internal/meshbool/tangency_gate.go

`meshbool`'s `facesOfMesh` is one of the two consumers that do NOT
withhold: it charges each face the displacement the operand's own mesh proved
for it (`agent-docs/tessellation-design.md` §2's `sourceBound`), and `tessellate.go`
composes a nonzero `sectionDelta` into every one of them, so the mesh path's
tangency gate (`meshbool.RefuseUndecidableProximity`) cannot read a face of a displaced
section as held exactly. See `facesOfMesh`'s own doc comment. The other is
`gatewitness.Prism` above, which charges `δ` as a shrink on the
tolerance gate's reference diameter.

### internal/capblend contour (boundary note)

The cap-loop chamfer's cap contour displacement (`agent-docs/modify-design.md` Part R
§8.4) is a separate term with its own owner, derived and read in
`internal/capblend` (`band.go`, `payload.go`, `build.go`). It is not
`sectionDelta`: nothing in this design's own consumers reads the cap contour
term, and nothing on the cap-loop chamfer path reads `sectionDelta`. A cap-loop chamfer of a displaced receiver instead carries each non-selected loop's `h_loop` as `capblend.DisplacedPayload.LoopDelta`, a third term charged beside the contour's (Part K §8.5).

## Part G — General Boolean

The next reach of exact `Union`/`Cut`/`Intersect` past the co-directional
prism pairs Part B admits: the co-directional shapes
that still refuse (some touching bosses, a rotated tooth), and the first
non-co-directional class —
two prisms whose sweeps are exactly perpendicular and whose every contact is a
line or a circle (a cross-drilled hole, a cross slot, a keyway). Companion to
`agent-docs/evaluator-design.md` §9 (the mesh path, unchanged and still the
fallback), Part B ("prism-boolean §N": the gates,
scene, cell classification and displacement bounds this design reuses),
Part S ("stacked §N") and
`agent-docs/mirror-pattern-design.md` ("mirror §N"). Core §N is
`agent-docs/api-design.md`; seam §N is `agent-docs/sketch-seam-design.md`. The rows of §2 describe
the mesh path's behaviour on the listed scenes before Part G, without measured figures.

### 1. Problem

Outside prism-boolean's admitted class a boolean tessellates both operands
(evaluator §9) and ends in one of three places, each measured in §2:

1. **A coplanar contact refuses.** Two operands sharing a plane with positive
   common area are `BooleanUnsupportedContact` under `ExactOnly`, and when
   no shift of the second operand opens the contact. This is the dominant
   failure: a boss standing on a face, a flange on a shaft,
   a tooth placed by rotation, a mirrored half beside its source, a tool that
   is a multi-lump body — every one of them shares a cap or a wall with its
   partner. A document with a gap tolerance runs the boolean again with the
   second operand moved by up to that tolerance
   (`agent-docs/gap-tolerance-design.md` §5.2), and the measurements below
   are of the exact path.
2. **The result is `Faceted`.** A cross-drilled hole, a slot, a keyway, a
   boss rooted inside a plate, a tee of cylinders all build, but as
   `facetedPayload`: most `Fillet`/`Chamfer`/`Shell` requests refuse
   (modify-reach SX9). A complete tilted plate-to-cylinder junction can take
   the bounded `Fillet` in `agent-docs/modify-design.md` Part A. The three
   surveys read `Suspect`, the analytic STEP
   path is gone, and the published volume bound is that of a faceted mesh, far looser than the closed-form bound
   of the same part drawn as an analytic cut.
3. **Chains stop early.** A faceted face's bound is the largest over its
   facets, so a plate's side wall carrying one hole's rim is coarse over its
   whole length, and a third cross hole refuses on the
   chain-depth gate (§2 S11).

A fourth limit sits beside them: a revolve with a spherical face cannot be
tessellated at the pair's chord tolerance at all (§2 S8), so a sphere never
reaches a boolean.

**Success criterion.** Every §3 class A shape builds a `prismPayload` or
`stackedPrismPayload` with prism-boolean §7's bounds; every class B shape
builds the analytic `brepPayload` of §4, whose faces are planes and
cylinders, whose edges are lines, circles and arcs, whose measurements carry
closed-form bounds, and which every consumer in §4.5 reads. `facetedPayload`
stays the result of pairs outside §3. The structural point-loft ×
coaxial-cone Cut/Intersect path is specified in `agent-docs/loft-design.md`
§20.4; it also uses `facetedPayload` but retains
its source surface tags.

### 2. Measured current behaviour

| Row | Scene | Path | Result |
|---|---|---|---|
| S1 | 40×20×20 box `Cut` by Ø6 cylinder along y through its middle | mesh | 7 `Faceted` faces, a mesh-bounded volume (closed form 15434.51), `Fillet`: no analytic edge, wall `unavailable` |
| S2 | Ø30×20 cylinder `Union` Ø8 radial boss | mesh | 5 `Faceted` faces, mesh-bounded volume |
| S3 | full revolve (cylinder along x) `Union` box crossing it | mesh | `Faceted` faces, mesh-bounded volume |
| S3b | the same revolve `Union` a coaxial disk sharing its end plane (flange) | mesh at the survey commit; common-axis Revolve when both inputs qualify | `BooleanUnsupportedContact` at the survey commit; analytic common-axis result now |
| S4a | plate `Cut` through hole, both on XY | analytic | 7 faces, closed-form volume bound |
| S4b | 20×20×10 plate with Ø4 through hole, then `Cut` Ø8 counterbore from z = 7 | analytic | 9 faces, volume 4000 − 76π mm³ within its published bound (`apitest/counterbore_test.go`) |
| S4c/d | the same counterbore first, then the Ø4 through hole inside it | analytic | the same 9 faces and bounded volume |
| S5 | plate `Cut` by six `Copy`-translated cylinders | analytic | at the survey commit: six results, `sectionDelta > 0`: `Fillet` refuses, wall `undecided`. Today the cut still records no carrier record, because §5.4 of Part K needs the identity re-expression (`prismcells.CleanCutRecord`) and a translated tool is not one. The `Fillet` refusal therefore stands. No test asserts this fixture; `TestChamferPlateAfterPatternSocketCut` pins the same refusal for the loops of rotated pattern sockets |
| S5b | the six holes each drawn in a sketch | analytic | closed-form bound, `Fillet` OK |
| S6a | filleted box `Cut` by a cross cylinder | mesh | `Faceted` faces, mesh-bounded volume |
| S6b | filleted box `Cut` by a same-plane cylinder | analytic | closed-form volume bound |
| S7 | two Ø10 cylinders crossing at right angles, `Union` and `Cut` | mesh | at the survey commit: `Faceted`, mesh-bounded volume. Not asserted by a test for equal diameters. `TestSectionMeshBooleanHistoryRefusesCutCurves` shows the mesh path for a Ø12 and a Ø8 cylinder crossing at right angles |
| S8 | sphere r = 10 `Cut` by a box, or by a Ø6 cylinder | mesh | `ErrUnsupported`: "more than 65536 facets in one revolve mesh" at the pair tolerance (`revolveproof` `maxFacetsPerMesh`) |
| S9 | box `Intersect` the same box rotated 45° about z | mesh at the survey commit; analytic with A6's charge | `BooleanUnsupportedContact` (coplanar caps) at the survey commit; an analytic octagonal prism with 10 faces and an `Approximate` volume now (`TestPrismIntersectRotatedBoxBuildsAnalyticOctagon`). Rotated 90° at the survey commit: "come within the chord tolerance without provably interpenetrating" |
| S10 | box `Cut` by a cylinder on a plane tilted 30° | mesh | 7 `Faceted` faces, mesh-bounded volume |
| S11 | 60×20×10 bar, three cross holes at x = 10, 30, 50 | mesh | holes 1–2 build; hole 3 refuses because the target's held mesh bound where the pair meets exceeds the pair tolerance. Both long side walls carry that bound over their whole face; the box's other faces carry 0 |
| S12 | two disjoint boxes `Union` | mesh at the survey commit; multi-region prism with A5 | `Faceted`, 2 lumps, `Exact` volume, wall `unavailable` at the survey commit; two lumps, `Exact` volume and no `Faceted` face now (§9, A5 N-hole cut) |
| S12c | two boxes touching at one vertical edge | mesh | `BooleanUnsupportedContact` |
| W1–W4 | two boxes sharing a wall (whole, partial, longer, 1 mm overlap with collinear walls) | analytic | RB1 `ErrUnsupported`: the arrangement reports an invalid region at the survey commit; one analytic prism now (A3, `TestPrismBooleanSharedWallBoxes`) |
| W5 | two boxes overlapping with no collinear wall | analytic | builds |
| W6 | box `Cut` by a box sharing its wall | mesh at the survey commit; analytic with A3 | `BooleanUnsupportedContact` at the survey commit; an analytic prism now (`TestPrismBooleanSharedWallBoxes`) |
| B1 | box `Cut` by a 10×10 square cross slot | mesh | 10 `Faceted` faces, exact-integral volume |
| B2 | Ø20×40 rod `Cut` by a 4 mm keyway box | mesh | 7 `Faceted` faces, mesh-bounded volume |
| B3 | plate `Union` Ø10 boss standing on its top (z 10..25) | mesh at the survey commit; A1 stacked prism | `BooleanUnsupportedContact` (held facets within tolerance) at the survey commit; an analytic stacked prism now (`TestStackedUnionBossOnPlate`) |
| B4 | plate `Union` the boss rooted inside (z 5..25) | mesh | 8 `Faceted` faces, mesh-bounded volume, no analytic edge |
| B5 | plate `Union` a side pin whose end cap lies in the plate's face | mesh at the survey commit; class B "Flush cap contact" when both are perpendicular datum-placed prisms | `BooleanUnsupportedContact` at the survey commit; analytic brep now |
| P1 | hub ∪ six teeth placed by `RotationAround` | analytic → mesh | tooth 2 `BooleanUnsupportedContact` (prism-boolean §3.4 reroute) at the survey commit. A tooth whose root arc lies on the hub circle still falls back to the mesh path and refuses with `ErrUnsupported`, "held facets come within the chord tolerance" (`TestPrismUnionToothOnHubCircleFallsBackAtTheTangentRoot`); a rotated tooth rooted inside the hub builds analytically (`TestPrismUnionRotatedToothBuildsAnalytic`) |
| P2 | plate `Cut` by a four-lump faceted tool | mesh | `BooleanUnsupportedContact` |
| M2–M5, T1 | reflected operands (mirror §2) | mesh | coplanar refusal, or a `Faceted` result |

### 3. Admitted classes, in priority order

Priority follows how many ordinary machined or printed parts a class unlocks,
measured by which §2 rows it clears. Class A keeps the result a prism or a
stacked prism and extends prism-boolean's own machinery; class B is the
first new payload. Nothing here changes the mesh path, and a pair no class
admits takes it unchanged.

#### Class A — co-directional prisms

| # | Shape | Clears | Result |
|---|---|---|---|
| A1 | `Union` with unequal sweep intervals: a boss standing on a plate, rooted inside it, a flange on a circle-prism shaft, a stack of blocks | B3, B4, S3b with the shaft drawn as a prism | `stackedPrismPayload`: a slab per distinct level, interface exposure by the clean-nesting match |
| A2 | a blind tool enclosing holes, or a through tool inside a blind hole | S4b, S4d | `stackedPrismPayload`: two scene cells prove the shoulder; a tool inside a hole leaves that slab unchanged (stacked §7 stage 2) |
| A3 | two operands sharing a wall: coincident line carriers (box beside box, a mirrored half beside its source), and the coincident arcs and circles `sketch` already resolved | W1–W4, W6, M3, T2 | `prismPayload` through prism-boolean's own paths, every shared span read from `sketch`'s report (A3 below); a reflected half beside its source (M2) joins through the mirror join (mirror §5) |
| A4 | a reflected operand, or two operands whose relative map is a reflection | M3, M4, M5, T1, T2 | prism-boolean's own paths over a reflected re-expression (§3.2) |
| A5 | a multi-region operand or result: a prism-group tool (N holes in one arrangement), a `Union` of disjoint footprints | S5 (through mirror §6), S12, P2 | one-slab multi-region `stackedPrismPayload` (mirror §6.3) |
| A6 | a split boundary under a non-identity re-expression, a prior section displacement or a walk charge | S9 (45° in plane), rotated and translated overlaps; not P1 (A6 below) | `prismPayload` carrying a certified crossing-sensitivity charge |
| A7 | one blind pocket crossing one through hole, opening at either cap, clipped to the target, every loop of axis-aligned lines, circles and arcs | a pocket that partly overlaps a through opening: a rectangle across a round hole or a slot, a round pocket across a square hole | `brepPayload` with split hole walls and a partial floor |
| A8 | a blind pocket ending exactly at an interface of a stack, overlapping in plan an opposed pocket that ends there, every exclusive hole a loop of axis-aligned lines, circles and arcs | pockets from opposite faces that meet at one level: rectangles, a round pocket across a rectangle, a slot, a counterbore drilled from both faces | `brepPayload` with a partial floor and ceiling |
| A9 | after the mesh path refuses: a blind tool crossing or touching the target's outer wall (a side notch), or crossing holes of a target with any number of them; loops of lines, circles and arcs, an oblique line only where no scene cuts it | a side notch tangent to the outline at recorded vertices (a watch lever's fork) | `brepPayload` from A7's build |

##### A1 — stacked union

Admission is prism-boolean's G1–G4 and G6 (hole-free operands until A5; a
holed operand goes to the brep build alone, under "Holed operands" below), G3
in any arm, and G5 read for `Union` as "the intervals overlap or touch"
(`z0_b' <= z1_a && z0_a <= z1_b'`, exact over `big.Rat`). The distinct levels
`{z0_a, z1_a, z0_b', z1_b'}` sorted exactly cut the union into at most three
slabs. Each level is also read as its exact range (prism-boolean §3.2): two
tied levels merge only when both denote one rational, and neighbouring
distinct levels must lie strictly in order across their ranges, so a 0.2 in
top and a block drawn at 5.08 mm, which hold one float, neither touch nor
order and take the mesh path. In a slab both operands reach, the region is
`Union`'s select-all merge (prism-boolean §4.2, with its enclosed-void check) over the two
records, with its §6 audit and §7 displacement; where the clean-nesting
match below proves one region carries the other's outer whole as a hole,
that merge is the containing region, and
the slab keeps the containing operand's record verbatim. In a slab one
operand reaches, the region is that operand's own record verbatim; under a
non-identity re-expression, B's region is arranged alone in a private scene
of re-expressed entities and recorded from the one cell `sketch` returns. Each interface is where the two adjacent slabs' regions
differ; its exposed records are decided by the clean-nesting structural match
prism-boolean §4.2 already runs: the smaller region's outer must reproduce
whole as a hole of the larger region's cell, in which case the exposed
record is the larger region with that hole, and the smaller region is the
material on both sides. A smaller outer touching the larger one at one
point, such as a boss circle tangent inside the plate's edge, still
reproduces whole, since the arrangement does not cut a tangency, and the
exposed face would pinch there. So a nesting match at an interface also
puts the two regions' entities to `prismcells.RegionContact` (stacked
§2.2), and any point it reports makes the match a miss: the pair tries the
brep build below and otherwise takes the mesh path, which meshes the boss
as one `Approximate` body. A slab both operands reach keeps a tangent
nesting, because the slab's region is the containing record alone. A boss whose footprint crosses the plate's outline,
or sits flush with it (a wall of each on one carrier), meets it at the
interface with `Partial` edges, which the match leaves unresolved. The 2D
answer is not what is missing: `Classify` resolves the interface scene's
cells, through A3's shared-span reading where the walls are flush, and the
cells on one side only are the exposed floor or ceiling. What the stacked
payload cannot state is the result's topology:

- an interface patch is bounded by whole column rings (stacked §3), while
  this floor's boundary takes part of the plate's ring and part of the
  boss's, so the rings would have to split at the vertices where the two
  outlines meet;
- a flush wall is one plane on both sides of the interface, across the
  plate's column and the boss's, and evaluator §3's canonicalization makes
  that one face, while a stacked wall is one column's segment swept over
  that column's own slabs.

Such a union is therefore stated as a `brepPayload` (§4) from the same
slabs, under "A1 as a brep" below, and a pair that build does not cover
takes the mesh path with no error, prism-boolean §4.4's unresolved
topology. So is a union whose slab both operands reach holds records that
stay apart, such as a second round boss standing beside the first on a
plate: the select-all merge leaves two loops, which a stacked slab's one
region cannot state. Stacked §2.2's I5 reads, for
a union-built stack, "every slab's outer loop equals the previous slab's or
is proven nested by the clean-nesting match", I6/I7 generalise to the
exposure records the match derives. The inner level a shifted operand contributes
rounds once and is charged by `rationalFloatError` into that interface's
`z0Delta`/`z1Delta`, exactly as stacked's blind cut charges its floor. A
union-built slab records the rational its operands denote at each level
(`stackedrecord.UnionLevel.Denoted`, stacked §2.1), so a later union tying
that level merges it, and so does the stack an A1 BRep keeps. A union-built
stack has several outer runs, so stacked §7's cut arms do not take it as a
target. A stacked operand in a stacked union splits at every level of both.

A stacked union whose only interface is a boss on a plate has `Volume` the
exact rational sum `A_plate·h_plate + A_boss·h_boss`, `Exact` where both are.

##### A1 as a brep — the flush or crossing interface

`internal/stackedunion/brep.go` takes over when an interface's clean-nesting match
is unresolved, or when the select-all merge of a slab both operands reach
does not close into one loop, with the slabs `prismboolean.TryStackedUnion`
admitted. The brep build then runs every slab's merge and every interface
again under its own checks below, and its miss takes the mesh path. It admits the pair
only where every carrier it records is exact: the identity re-expression
(B's records enter every scene verbatim; a re-expressed B would be recorded
once per scene, at floats no two scenes share), both operands'
`sectionDelta` zero (class B's B5: a straight wall becomes a planar face at
its recorded level, and a planar face states no band for a wall that
moved), and every scene's walk and crossing charge zero, except the shared
span width a holed union carries as a band (below). Every miss is silent.

**Operands.** An operand is a prism (one region over one interval), a
stacked prism (one region per slab), or a brep that keeps the slabs it was
built from (§4.1's `stack`: the A1 build's own result, with one or more
regions per slab, holed only where the holed-operand gate admitted them, and
the section displacement they carry). A
class-B brep keeps no stack and is not an operand here. The levels of both
operands cut the union into slabs, and every slab names the records
reaching it: each operand's regions in its slab there.

**Holed operands.** A union whose operand holds a region with holes builds
here only where one of those holes shares a span with the other operand's
outline, as sketch resolves it (A3); every other holed pair takes the mesh
path silently, whatever the build's refusal. A boss standing on a plate
across a window whose inner wall lies on the boss circle is the case: the
boss wall continues the window wall below the plate's face.

- *Pre-filter.* Before any scene, a hole segment of one operand and an outer
  segment of the other must lie on one carrier kind within a loose band
  (circles: centre distance plus radius difference at most `1e-9·max(1, r)`;
  lines: both hole endpoints that close to the outer line). It computes in
  decad for cost alone, sparing the arrangement for pairs that cannot share
  a span, and only rejects.
- *Cap.* Every scene of a build that passes the pre-filter arranges under
  twice `prismcells.MaxArrangementSegments` (`gatedArrangementSegments`, 8192).
- *Gate.* Every scene the build uses is arranged and charged first. Some
  scene must report a span between a hole entity of one operand and an
  outer entity of the other. A span between two records of one operand must
  have zero width, and a line span between the operands too.
- *Slab merge.* A slab both operands reach where a record has holes keeps
  the cells `ClassifyRegions` puts inside some record. Every other cell
  must carry an edge of a record's hole loop with the cell on that loop's
  void side, so it lies inside an operand's own hole; a cell outside every
  record without one is a new enclosed void and a miss. The kept cells merge
  (`MergeLoopsExactLineCuts`) into one counter-clockwise outer that takes
  every clockwise loop as a hole; several outers with a hole miss, several
  without one are proven apart as above. Each holed region is audited by
  ring rotation: the first rotation of its rings' starting segments, among
  at most eight, that modify §5's audit passes is kept. A rotation changes
  no geometry, only the loop point the nesting test classifies, and that
  test's disjointness proof makes every point classify alike.
- *One carrier.* The two record carriers of every circular span join one
  class whose canonical carrier is the least member; every loop reads its
  segments' carriers through that class, so a wall piece below the
  interface and the other operand's piece above it on the same circle join
  into one swept face or meet at a side-line split.
- *Span-end junction.* Where sketch puts a span's end at the named arc's
  own bound (its parameter exactly 0 or 1, on a whole reported edge), the
  arc's recorded endpoint is a span end of its class. A circle-circle
  junction whose two walked ends differ, one a natural end at such a corner
  and the other a cut on a carrier of the same class, stands at that corner.
  Its allowance is the larger walk allowance plus the outward L1 distance
  between the two walked ends: the cut end walks within its allowance of the
  point its circle denotes at sketch's certified cut. The published vertex
  is the recorded corner, a few ulps from sketch's own vertex position and
  inside that allowance. It rests on A3's one-carrier reading: without it,
  a tangent contact between a corner round and the boss circle could put
  the true vertex about `sqrt(2·ρ·gap)` away. A round sketch cuts has a cut
  end and never matches, so that pair takes the mesh path.
- *Width band.* With every walk charge and section displacement zero, the
  build's crossing charge is the largest span width (A3). Every face's
  displacement adds `3·D` plus the canonical radius's proven rounding, with
  `D = (largest class size − 1)·crossing` rounded up, and never less than
  the crossing itself: a piece about the canonical centre
  `c0` whose radius is read from a vertex on member `Cj` sits from its own
  member `Ck` within `|rj − rk| + |cj − ck| + 2·|ck − c0|` of its allowance. A
  canonical radius read from an arc carries the exact bound on its distance
  from every member segment's recorded radii. The stack carries the same.

**Scenes.** A slab both operands reach arranges every record reaching it,
operand A's as the scene's A regions and operand B's as its B regions, and
its regions are the select-all merge's loops (`prismcells.MergeLoopsExactLineCuts`): one
loop, or several when the records are apart in that slab, each an outer
walked counter-clockwise, with prism-boolean §4.2's enclosed-void check and
§6's audit per loop. Several loops are separate regions only when
`prismcells.ProveGroupApart` proves them apart (A5's reading: two tangent
boss circles close as two whole loops meeting at a point no loop names, and
miss), and that proof scene's walk charge joins the build's walk term. A slab one operand reaches keeps that operand's
records verbatim. Each interface arranges every record reaching the slab
below or the slab above it, again split by owner, once: a record reaching
both sides enters once and counts on both. Two records of one operand are one
record there only when their outer loops and their holes are equal exactly,
so two slabs that share an outer but not a hole enter as two. The scene is cached by its
record set, so a slab merge and an interface over the same records share
one arrangement and its cut points, and two scenes that cut one carrier
pair record the crossing once through the keyed table below.
Only this authenticated A1 BRep Union path, a common-axis full-Revolve
Boolean ("Common-axis full revolutions" below) and `Body.Section` of a mesh
Boolean result read from its history (`agent-docs/section-design.md`) can
re-express a complete all-line loop at certified exact line crossings; the
section refuses any loop that keeps a cut charge. Each crossing must have
one paired Sketch event, an exactly representable binary64 coordinate,
and strict source-range and walk-order checks. Any failed check keeps the
original fragments and cut charge. Ordinary Union, Cut, Intersect, and
class-B paths use `prismcells.MergeLoops` or `MergeWithChain` unchanged.
`prismcells.ClassifyRegions` names each cell's membership in every record
of the scene, a cell's holes read as its outer is — prism-boolean §4.2's flag comparison and propagation, run
per record rather than per operand, with a span two records share
(`prismcells.CoincidentEdgesRegions`, A3's reading with the partner taken
from any other record, of either operand) a boundary of both that neither
membership crosses. A cell whose membership in a record no propagation
reaches lies in a group of cells, joined by shared edges, that holds none of
that record's edges. A record ring enclosing such a group is the outer of a
cell that holds the group's outer ring as a hole, which joins the group, so
the group lies outside the record and the cell reads outside it: the top of
a lower block and the footprint of a taller one, standing apart on a plate,
share no edge in the scene above the lower block's top
(`prismcells.ClassifyRegionsApart`). The reading holds only where no two
groups touch: a walked vertex reached from two groups, an entity walked by
two, or a `RegionContact` point between entities of two groups (a block's
corner on another's, a block's edge tangent to a boss circle) is a contact
that would pinch the result, and the scene stays unresolved and misses.
Only a `Union`'s interface reads cells so, and only when some interface cell
lies inside a record of each operand. That guard narrows where the reading
is tried; it is not what makes the result correct, and it does not keep the
result in one piece. Two blocks that stand apart on no common plate share no
such cell and keep the mesh path, but an operand that is itself two pieces
can meet the other with one piece while its other piece floats clear: a
plate with a tower, and as the other operand two blocks over z 20 to 30, one
on the tower and one clear of everything, build an exact two-lump brep. That
result is correct, and it is accepted. A cell unreached in a record whose edges its group does hold
stays unresolved and misses.
A cell inside some record reaching one side and inside
none reaching the other is that side's exposed face, recorded from the cell
as `sketch` returned it (a floor faces up, a ceiling down). A `Union`'s
exposed cell may carry holes: the plate's floor round separate bosses is one
cell whose holes are the boss footprints, and becomes one planar face with
those holes, each ring restated through `LoopOf` as a slab cap's are. The
arrangement cuts no tangency, so a boss tangent inside the plate's outline
leaves a hole touching the outer at a point the cell never names, and two
tangent bosses come back as one hole ring through their tangency twice;
`prismcells.RingContact` reads a walked vertex starting two of the cell's
edges, an entity in two rings, and `RegionContact` between the rings'
entities, and any contact is a miss. A `Cut`'s holed interface cell misses. Two records of
one operand sharing a wall in one scene enter that scene as two regions and
read as A3 reads them; two records whose identical segment the scene
builder would make one entity read as unresolved and miss.

**No pinch at an interface.** A `Union`'s floors and ceilings must not meet
where the body would meet itself, even inside one connected piece: a U-shaped
bridge resting on a tower by one arm, whose other arm's corner meets the
tower's top corner, rests on the tower's top edge from outside, or is tangent
to a boss's top circle. Every cell still classifies and the record pairs every
edge, so `interfaceManifold` (`internal/stackedunion/brep.go`) reads the
interface scene and misses on any of these:

- a point where a record reaching only one side touches a record of the other
  side (a record reaching both is read against each) that is no scene vertex
  where an edge of each ends (`prismcells.ContactsCut`). The points are
  sketch's line, line-circle and circle queries'; a transverse crossing inside
  both entities is exempt, so a tangency or a point at an entity's end must be
  a vertex. A shared span's partner counts as ending where the span's edges
  end, so a window's corner round meeting the boss circle at a span's end
  passes;
- a vertex a floor and a ceiling both walk where edges of a record reaching
  both sides end, or of more than one record reaching only the slab below or
  only the slab above, a shared span's edge counting for both its records;
- at such a vertex, a floor or ceiling corner that does not turn from the
  below record's edge to the above record's, or that walks a shared span
  (`opposedManifold`).

A floor and a ceiling meet only where both sides' boundaries pass. With one
record per side there, each passing once, the material below is one sector and
the material above another; their four bounding edges alternate between the
records exactly at a transverse crossing, where the body is manifold, as at a
boss crossing the plate's outline. Every other order puts a floor or ceiling
corner between two edges of one record. The check only refuses.

**Vertices.** decad restates the records it holds and computes no 2D
answer. Consecutive fragments of one carrier — a circle cut at its seam, a
line run restated at other cells' vertices — are one unit. Each junction
between two units is one canonical vertex, as class B's §10 table makes
it: two axis-aligned lines meet at their exact levels, displacement zero;
a line crossing a circle takes the keyed table's one float for the crossing
(the line's walked point, its fixed coordinate exact, with the larger of that
point's cut and walk allowance and, at a cut, its proven distance from the
exact crossing, §5), keyed by the two carriers and the side of the circle's
centre the crossing lies on, the first loop to reach the key recording it,
operand records before cells so a recorded corner wins; a crossing within
twice its allowance of the centre's coordinate cannot name its side and
misses; two circles crossing have no exact record and miss. Every unit is
then rewritten between its two vertices, a line whole and a circular
fragment as an arc pinned there about its recorded centre.

**Faces.** Each level's horizontal faces — the bottom cap, the top cap,
and each interface's floors and ceilings — are planar faces in the
reference frame (operand A's), their line units split at every vertex of
the body at that level on the unit's carrier, their circle units at every
vertex on the circle. A vertex of the body at a level is a horizontal
face's loop vertex there, or a level where a slab region's junction starts,
ends or changes its two carriers. Every straight wall on an axis-aligned
carrier plane becomes one planar face per material side, in a frame whose
normal is the plane's outward axis and whose in-plane axes are two
reference axes (a right-handed signed permutation, so §4.1's frame rule
holds and every coordinate carries over exactly): each slab's segment on
the plane sweeps a rectangle whose horizontal edges are split at the body's
vertices at the two levels, the edges two rectangles share cancel, the
rest chain into the face's loops, and consecutive collinear pieces join
where their shared point is no vertex of the body. A loop that chains into
a clockwise ring, a point with two outgoing pieces, or a rectangle edge
with no partner is a miss. A flush wall is then one L- or T-shaped face
across both operands' slabs, as class B's crossing reach records a notched
wall. Every circular or oblique straight wall is a swept piece between its
vertices over its slab's interval, the identical piece in consecutive slabs
joined into one face; a vertex of the body at the level between, on either
of the piece's two ends, becomes a split of that side line (§4.1's
`side0`/`side1`, with that level's displacement), so the rooted round boss
crossing the plate's outline builds: its floor corner lies on the
overhanging piece's side line, which the record states as two edges. An
oblique line with a vertex strictly inside it misses.

**Displacements.** Every level carries its own `z0Delta`/`z1Delta`. Every
face carries one section displacement: the largest allowance of any canonical
vertex, and nothing else. No face coordinate reads a slab merge's cut
parameter. A junction is a canonical vertex, and `LoopOf` and the face writers
record only canonical points, recorded levels and whole lines. A vertex where
a line meets a circle or another oblique line reads a cut end, and
`stackedbrep`'s `ubEnds` adds that cut's charge to the vertex's own allowance,
with `crossingOffset` on top. A circle-circle junction (`recordedJunction`)
takes the larger `ubEnds` allowance of its two ends. So an axis-aligned
result's faces are exact rational data even when a merge falls back to
`CutDisplacementAllow` for a cut: the L bracket's base side is cut at the
upright's wall, an interior point of the merged edge and a corner only of the
other operand, so the two fragments meeting there are collinear. A modify op
may take such faces for the faces they denote. The stack is the one record
that keeps merged slab fragments verbatim, so `Stack.Delta` is §7's stacked
formula over the slab merges (their cut charge, except for A1's certified
exact line rewrite, since every incoming and walk term is zero by admission)
plus that vertex allowance. Every vertex lies on exact carriers — an
axis-aligned plane at its recorded level, a circle about its recorded centre
— within that distance of the crossing it denotes, so each planar region and
each pinned arc is within it of the face it denotes; a swept piece's band
and a planar face's area term read it as §4.3 states, and a planar face's
level moves by its level displacement alone. An all-planar result with float
levels is `Exact`. A nonzero `Stack.Delta` is read by `Body.Section` (through
its `SectionDelta`, `internal/sectionclip/reconstruct.go`) and `ScaledAt`
(`stack_brep_scale.go`), which refuse it, and by a further Union, whose
stacked operand then carries it as its `SectionDelta` and misses silently
(`internal/stackedunion/brep.go`).

**Audit and consumers.** Each planar face runs modify §5's audit, and the
record must pair every edge by §4.2's count; a record this build leaves
unpaired is an uncovered topology and a miss, not a refusal. The result is
an ordinary brep: §4.5's consumers read it, STEP writes it analytically
(two partial cylinders with arc rims for the crossing boss), and it keeps
its slabs as its `stack`, so a further co-directional `Union` on it — a
second boss on the plate — enters this build as an operand and the result
keeps the merged slabs as its own stack; its reach is the admission
above, so a result whose stack carries a section displacement (a crossing
round boss) or a merged slab's walked fragments is a miss as an operand.
A brep operand in a `Cut` or `Intersect`, or in a perpendicular pair, is
class B's (§4.5). Edge convexity is evaluator
§3's walked boundary (§4.2). Every wall face records the stack axis as its
sweep, so two walls meeting along a vertical line form a junction that reads
the turn: an L boss's reflex corner reads concave. A line a floor and a wall
share reads the owner's loop role, so the floor's edges along the boss's
walls read convex, as the stacked body's rims along a square boss do.

##### A1 face patch from perpendicular rectangular prisms

Two authenticated rectangular prisms can share part of one planar face even
when their original sweep axes differ. Before class B's B8 gate, Union admits
this pair when both profiles are complete axis-aligned four-line rectangles,
both sections and levels have zero displacement, the transforms are identical,
and each source axis is bit-identical to a signed axis of one exact orthonormal
reference frame. The source rectangles and sweep intervals define boxes in
that frame. Each projected box coordinate is evaluated as an exact dyadic
rational and must be representable as a float. Exactly one box-axis interval
ends where the other starts; the other two intervals must overlap with
positive length. The reference prism's sweep axis must be one of those
in-plane axes. This proves a positive-area common face patch, disjoint
interiors, and one connected solid without a sampled contact test.

The second box is restated as a rectangular prism in the first box's frame.
This restatement preserves its exact source solid because the signed-axis
permutation and every coordinate passed the rational check. Equal sweep
intervals use the existing co-directional prism Union. Unequal intervals use
A1's slabs and, for a partly shared wall, its BRep result. Both routes use
private Sketch scenes to classify the footprints and remove the shared face
patch. A1 also splits the exposed interface faces at the two levels. The
prism or BRep topology audit closes the result, and its volume reading equals
the sum of the disjoint source volumes. Their ordinary tessellators supply
closed-boundary and occupied-volume proofs at `VerifyAll`. If one reference
frame silently misses, the pair is tried in the other's frame, since that
changes the slab axis. An unequal-level A1 frame is tried before an equal-level
prism frame, so a planar box join keeps its exact BRep volume when available.
A nonrectangular, displaced, rounded-frame or edge-only pair misses this path
and reaches the unchanged later dispatch.

##### A4 — the reflected re-expression

G2 is replaced by "the relative map's determinant decides nothing": operand
B's record is re-expressed into A's frame through the composed relative map
exactly as prism-boolean §4.1 does, and when that map is a reflection
(`Transform.IsReflection` on the composed map, read once) the mapped record
is re-wound before any entity is created:

- every loop's segment order is reversed;
- each `LineSeg` swaps `Start`/`End`; each `ArcSeg` becomes
  `{m(Center), m(End), m(Start)}` (mirror §5.2's table: the reflection turns
  the arc clockwise and the reversal turns it back); both keep their range,
  whose order still names the walk's sense;
- each `CircleSeg` keeps `CCW` and its range, with only its centre mapped:
  the reflection reverses the circle's winding and the reversal reverses it
  back, and `walkOf` refuses a `CCW` that contradicts the range order;
- a `LineSeg` recorded over a narrowed range enters as its walked endpoints
  (prism-boolean §7 `δ_walk`), since `1 − t` is not an exact float operation
  for a general `t`; a trimmed circular carrier is refused as §4.1 already
  refuses it;
- `prismcells.Origin.AuthoredReversed` is computed on the re-wound record, so
  the crossing sub-case's flag comparison reads the authored sense of the
  record the scene actually holds.

The shared-axis arm (`xformA == xformB`) admits two operands that are BOTH
reflected the same way with no re-expression at all (T1). The G3 reading of
the world normal uses `ApplyDir`, which a reflection maps correctly, so the
co-directional test is unchanged. `δ_reexpress` is the same single
rounding per coordinate; a reflection adds no term. Interference's read-only
twin (`booleandispatch.AnalyticIntersect`) and the overlap-area reading share the
same gate and scene, so a reflected pair the analytic `Intersect` resolves is
measured instead of reaching the mesh path's `unsupported_pair_contact`.
A pair with equal transform bases whose composed origins differ only along
the sweep axis also needs no section re-expression; G3 computes that shift
exactly for a `Copy` operand. The placed-axis arm requires zero section and
axial displacement on both operands. `Union` admits equal recorded profiles
or the existing equal axis-aligned rectangle certificate. `Intersect`
requires equal rectangle bounds; `Cut` requires the tool's bounds to lie
strictly inside the target's bounds. Stacked Union still checks its own
intervals, slabs and interfaces.
Other placed-axis pairs keep the mesh route. The same-transform shared-axis arm
admits profiles by its own rule. `Verify` measures a pair the placed-axis arm
admits through `booleandispatch.AnalyticIntersect`, which runs the same admission, and then its positive-volume gate.
A one-sided reflection is a nonidentity re-expression, so a reflected pair
whose outlines cross takes A6's crossing charge, and one whose outlines share
a wall takes A3's reading (M3, T2).

##### A3 — shared walls

`sketch` resolves two operands' entities on one carrier (two lines, an arc and
a circle, or two arcs; `sketch`'s "Coincident line carriers") by emitting their shared span once, under the entity it names,
and withdrawing it from the other, the losing entity, as a window its
fragments do not cover. `prismcells.CoincidentEdges` reads every such span
from that report alone: the windows are the gaps in the losing entity's own
fragments, a window's ends are vertices `sketch` split both entities at, and
the partner is the one entity of the other operand, on the same kind of
carrier, with fragment ends at both. Two candidates make the scene
unresolved; none means the window is an open part no bounded cell uses. A
circle withdrawn whole takes the other operand's circle with the same
recorded centre and radius bit for bit, or the scene is unresolved. Nothing
is decided from geometry.

Each consumer reads the span as a boundary of both operands:

- **Classification.** The edge carries both operands' direct readings: its
  own entity's flag comparison, and the partner's, through the two entities'
  relative direction (two lines may oppose; arcs and circles both run
  counter-clockwise). Membership never propagates across it. Propagating
  across it, as an edge of the named operand alone, put the cell beyond the
  span inside the operand it only touches: a tooth whose root arc lies on a
  hub's circle `Intersect`ed into the whole tooth and `Verify` reported the
  touching pair `Interfering`. `ClassifySplit` reads it the same way, and
  `Trim` refuses a span it shares with the receiver (surface-intersection
  RS5).
- **Runs.** `sketch` reports a cell's boundary in runs: a cell walking a line
  straight through vertices that only bound cells on its other side reports
  one edge over the run. `prismcells.SplitRuns` restates every line run at
  the parameters other cells' edges of that line end at, with those edges'
  own `Polyline` vertices, so a shared span is one edge key in both cells it
  separates; a piece is `TExact` only where both of its bounds were. A
  circular run, which would need its own densified `Polyline`, leaves the
  scene unresolved.
- **Crossing charge.** The two entities of a span do not cross, so A6 skips
  that pair at its end vertices and charges every other pair there.
- **Displacement.** The result records the span on the named entity. The
  span's width bound is the gap between the two recorded entities — at the
  span's two ends, each end vertex's distance from both lines; for circular
  carriers, the centres' distance plus the radii's difference, an arc's
  radius read at both recorded endpoints — plus both incoming displacements,
  every square root rounded outward. It enters `crossing` in §7's formula
  and covers a span the result keeps as boundary. `sketch`'s identity gate
  decides the two recorded entities are one carrier (prism-boolean §4.1), so
  a span inside or outside the result leaves nothing between two walls. An
  incoming displacement does: the true walls can part by up to the width,
  leaving a sliver no recorded edge bounds. A displaced pair with any span
  that is not exactly one selected cell's boundary takes the mesh path.

A union of two boxes sharing a wall whole, drawn on one carrier, cuts nothing
and is `Exact`. A partial shared wall keeps the cut charge unless A1's entire
all-line loop passes the paired Sketch-event and exact crossing proof above.

##### A5 — multi-region operands and results

A prism-group tool (mirror §6.3) enters the scene as `Count` operand-B
loops, each tagged with its region index. `Cut`'s clean-nesting match then
requires the target's outer whole plus exactly `Count` new holes, each
reproducing one region's outer whole, with no curved contact between the
target and a region or between two regions (prism-boolean §4.2's
tangent-contact query); any `Partial` edge takes the crossing
sub-case with the group's regions classified as B-material each. A
`Union` whose select-all survivors chain into several closed loops that are
pairwise disjoint (the structural disjointness read of mirror §6.3, on the
survivors) becomes a one-slab multi-region stacked payload instead of
"unresolved → mesh path" (prism-boolean §4.4). Prism-boolean §4.5's
overlap-area reading is unchanged.

Select-all keeps every bounded cell, which is the union only when no cell is
material of neither operand. Hole-free operands can still enclose such a
cell between them (two C shapes facing each other, or four bars in a ring),
so A5's `Union`, like every select-all merge (prism-boolean §4.2, A1's
slabs), first requires every cell to carry at least one boundary edge on
its operand's material side. A cell with none is an enclosed void, and the
pair takes the mesh path. A group operand needs
the partner's interval exactly (prism-boolean §3.2's `Union` row), and a
result whose survivors close into one loop is a prism. A group scene whose
cuts carry a displaced operand's displacement (a placed group, for one)
takes A6's crossing charge on both the `Cut` crossing sub-case and the
`Union` merge, and falls back to the mesh path where A6 does.

A `Union`'s several loops are a group only when they are apart as well as
disjoint, since a body meeting itself at one point is not a manifold. The
arrangement cuts neither a tangency nor a line at another line's endpoint
lying on it, so a circle tangent to the plate's side, two tangent circles,
or a triangle's apex on a square's side close as separate whole loops. A5's
`Union` and prism-boolean §4.6's one-scene `Union` therefore prove their
loops with `prismcells.ProveGroupApart` (stacked §2.2): no vertex two loops
share, and no point sketch's intersection queries report between two
loops' entities. A touching pair takes the mesh path, which meshes it as
one `Approximate` body.

##### A6 — the crossing-sensitivity charge

An input displacement moves every cut the arrangement makes: two recorded
carriers crossing at `O` at an angle `θ`, whose denoted carriers sit within
`δ1` and `δ2` of them, meet at a point `P*` with
`|P* − O| ≤ (δ1 + δ2)/sin θ_low + min(δ1, δ2)`. The proof: take `Q1`, `Q2`
on the recorded carriers nearest `P*`, so `|Q1 − Q2| ≤ δ1 + δ2`; inside a
ball around `O` where every tangent of one carrier makes an angle of at
least `θ_low` (mod π) with every tangent of the other, `Q1 − O` and
`Q2 − O` are sums of each carrier's own tangents, so
`|Q1 − Q2| ≥ max(|Q1 − O|, |Q2 − O|)·sin θ_low`. This class charges that
distance instead of prism-boolean §3.4 rerouting the pair
(`prismcells.CrossingCharge`, `internal/prismcells/crossing.go`):

- a cut is an arranged vertex (an edge's walk end in its `Polyline`, matched
  by exact equality) where some incident edge's parameter is one the
  arrangement computed (a `Partial` edge's non-natural end, or any end of a
  `Partial` circle). Every pair of distinct entities incident at a cut is
  charged, whichever face walks them, so a pair adjacent only through the
  unbounded face, which no returned cell walks, is charged too. A vertex
  where only recorded vertices meet is not a cut (prism-boolean §4.4's
  known limit). A touch from outside is not arranged as a vertex at all:
  `sketch` returns the two outlines as separate cells, and every point a
  displacement moves across either boundary there lies within that
  operand's own displacement of the other's recorded wall, inside the
  section displacement's tube;
- each side's displacement is its operand's incoming term: `δ_A + δ_walkA`
  for A, `δ_B + δ_walkB + δ_reexpress` for B. Both zero charges nothing;
- `sin θ_low` is a certified lower bound on `|sin θ|` between any tangent of
  one recorded carrier and any tangent of the other at points within `ρ` of
  a junction point held exactly: a line side's exact rational point at its
  recorded parameter (within its `δ_cut` of `O`), or a circular walk's
  endpoint with its walk-end bound added. It is exact rational arithmetic
  with outward-rounded square roots: `|d1 × d2|/(|d1||d2|)` for two lines,
  `(|d·(p − C)|/|d| − ρ)/R` for a line and a circle (a circle's tangent is
  perpendicular to its radius), `(|a1 × a2| − ρ(|a1| + |a2|) − ρ²)/(R1 R2)`
  for two circles, and zero for a circle read over a ball wider than half
  its radius;
- `ρ` starts at the junction point's own error, then widens to hold twice
  the distance the first bound charges; the second bound is accepted only
  when the distance it charges fits that ball, and otherwise the crossing
  has no charge;
- the largest charge over every cut of every returned cell is `crossing`,
  and the section displacement becomes
  `up(max(δ_A + δ_walkA, δ_B + δ_walkB + δ_reexpress, crossing) + δ_cut)`;
- a cut with `sin θ_low` not above the dimensionless noise floor
  `ε = 1e-9` of verification §4 (`sectionaudit.ContactEps`) has no charge,
  nor does a bound that does not settle or an edge that does not record.
  Neither check admits anything. `sketch` itself declines to certify a
  line/line cut that is very shallow (`TExact = false`), so
  the floor is reached through tangent junctions, whose bound is zero;
- a pair whose cuts carry an input displacement never refuses on the
  analytic path: a crossing with no charge, and any merge, audit or
  recording failure after the charge (RB1–RB9), sends it to the mesh path
  with no error (`prismcells.AmplifiedFallback`), as prism-boolean §4.4 does for
  every topology it leaves unresolved. Cancellation still propagates, and a
  pair that brings no displacement keeps its §9 refusals.

A6's distance bound is also, in the carrier-record design's own readings, an
existence and uniqueness statement (Part K §6.1,
Lemma T). With
`q1 ≥ dist(p, γ1)`, `q2 ≥ dist(p, γ2)`, `L = (q1 + q2)/s + min(q1, q2)` and a
ball of radius `ρ ≤ R/2` for each circle where the tangent sine is at least
`s > 0`, `2L ≤ ρ` means the two carriers meet in `B(p, ρ/2)` at exactly one
point `c`, with `|c − p| ≤ L`, and cross there. Only the readings that design
makes (its vertex reading, ASA2 and CB2) carry that proof; an A6 reading of
`sin θ_low` elsewhere states the distance bound alone.

With A6 a rotated tooth whose root sits inside the hub builds analytically
with a bound of a few ulps times `1/sin θ`, `Approximate`. A union result that
carries a valid carrier record (carrier-record-design §5.2, §5.3) fillets its
lateral corners through the carrier route (§7). A cut-bearing merge without a
record, or without A1's exact line-junction proof, still refuses
(prism-boolean §13's displaced-receiver rule). P1's own tooth
does not build. Its root arc lies on
the hub's circle (a coincident carrier, which `sketch` merges), the rotation
displaces the tooth, and the shared arc lies inside the union, where A3 has
no charge for the two true walls parting: the pair falls back to the mesh
path. A chain of teeth also stops at the
second tooth on prism-boolean §4.1's trimmed-circular refusal, since the
first union trims the hub circle.

##### A7 — blind pocket across a through hole

`prismboolean.TryCrossingBlindCut` admits `Cut(target, tool)` when `target` is a
prism with exactly one through hole, `tool` is a hole-free prism, and every
segment of every section loop is an axis-aligned line, a circle or an arc
(`momentinput.CrossingBlindLoops`): a rectangle, a rectilinear polygon, a
round hole, a slot. An oblique line, an ellipse and a free-form segment miss.
The target and tool have zero section displacement. Their relative section map
is the identity.

`stackedrecord.CrossingBlindLevels` reads the two sweep intervals exactly. The
tool is clipped to the target's interval, so it takes one of two arms:

- The top arm: the tool starts strictly inside the target and ends at the
  target's far cap or beyond it.
- The bottom arm: the tool ends strictly inside the target and starts at the
  target's first cap or before it.

A tool that spans the whole interval, lies wholly inside it, matches it, is
disjoint from it, or touches it only at a cap continues to the ordinary boolean dispatch. A level may carry an axial
displacement (an inch-converted depth does). Two levels compare by their exact
rationals and order only when their gap exceeds the sum of both displacements.
Equal levels count as one when both are undisplaced. A tool end and a target
cap also count as one when both record a denoted level (stacked §2.1) and the
two rationals are equal, whatever their displacements and held floats: 0.2 in
reached by a `Distance` and by a `TwoSided` side, or by half a full-length
`Symmetric` 0.4 in, denotes one plane. The tool's denoted level is lifted
onto the target's axis by G5's exact shift first.
`stackedunion.OperandFromPayload` carries a prism's denotations onto its
slab for this comparison, and a through `Cut` that nests the hole inside the
target keeps the target's. Equal held floats never stand in for equal denoted
levels: 0.2 in and 5.08 mm hold one float and denote two planes, and a
displaced level with no recorded denotation matches nothing. A gap inside the
displacements, or equal levels that carry one without one shared denoted
level, orders nothing and the pair continues to the ordinary dispatch. A
displacement therefore never admits an order: it only has to be smaller than
the gap. A denotation never contradicts an order the margin proves, since
each lies within its own level's displacement. The tool is clipped to the
target, so a tool end that meets a cap reaches the slab beside it, and the
result takes the cap's held level and displacement. A displaced level reaches
the result's face levels, so the volume reads `Approximate` with a proven
bound, and a result with no displaced level reads `Exact`. The BRep stack A7
outputs records no denoted level. An A7 result whose two input profiles and
selected slab regions contain only lines, with zero input section
displacement, also carries its source prisms for point containment
(point-containment §9.1). These are not an A1 operand stack and do not admit a
later Boolean operation.

In the top arm the target's original section is the lower slab and the cut
section the upper; in the bottom arm the cut section is the lower slab and the
target's section the upper. For the reached slab, a
private Sketch scene requires `TExact` on every partial edge and classifies
every cell, including cells with a hole, by each source region's recorded edge
orientation. It retains cells inside the
target and outside the tool. `prismcells.MergeLoops` must return exactly one
counter-clockwise outer and one clockwise hole; the normal section audit
checks the merged record. The scene's shared-span bound, crossing charge and
cut-parameter displacement enter the same gates and bounds as A1's brep
build. A scene that cannot classify or close this shape misses.

The A1 `stackedbrep.Engine` records the two slab regions and the interface
cells. Its horizontal cap face can carry a hole, and its existing wall
assembly splits a partially shared hole wall at the floor. The generic BRep
face and topology checks run before publication; `Tessellate(VerifyAll)`
checks the occupied volume. A pocket crossing the body's outer boundary, a
second source hole, a displaced section, a rotated footprint, and later Cuts of
the result remain outside this bounded path; A9 takes the first two where the
mesh path refuses them.

Circles and arcs reach the same build with no new 2D answer. Each crossing of
a pocket line with a hole circle, or of a pocket circle or arc with a hole
line, is a cut `sketch` computed and certified with `TExact`. An uncertified
fragment misses before any cell is read. The engine places it as "A1 as a
brep" places a line crossing a circle: the line's walked point with its fixed
coordinate exact, keyed by the side of the circle's centre, and charged its
walk, cut and proven crossing distances. A floor arc and the split cylinder
wall are pinned between those vertices about the recorded centre. Any circle
or arc in the result makes its volume `Approximate` with a proven bound, even
when every level is exact and no line crosses a circle (a disc plate's rim).
Only a result whose loops are all lines, with exact levels, reads `Exact`. Two circles
or arcs crossing, a round pocket across a round hole, have no exact vertex
record: the engine misses unless both loops end at one recorded point, and the
pair takes the mesh path. A floor or ceiling cell with a hole, such as the
annulus a round pocket leaves around a smaller round hole, also misses in the
engine, so a counterbore keeps the stacked payload of A2 (stacked §7 stage 2).
A merged slab loop whose recorded junctions do not meet, such as a pocket
drawn as four lines overshooting its corners, is a miss here, not
prism-boolean's RB9 refusal: the pair continues to the later routes and ends
on the mesh path. A pocket edge tangent to the hole circle also misses.

##### A8 — opposed pockets overlapping at one level

`prismboolean.TryOpposedBlindCut` takes the stacked blind `Cut` that
`TryStackedBlindCut` leaves to the mesh path: the tool ends exactly at an
existing interface of a cut-built stack, and after the cut both sides of that
interface hold exclusive holes that `stackedrecord.SeparatedOpposed` does not
prove apart (Part S §2.2). Its admission is the
stacked arm's. The target is a stacked payload with one outer run and one
region per slab. The tool ends at an interface with zero level displacement.
Every reached slab passes its own whole-loop cut proof
(`stackedrecord.ApplyOpposedBlindCut`, which shares `CutSlabs`'s slab cut).
The result's section displacement is zero, and every exclusive hole at that
interface is a loop of lines along the section axes, circles and arcs
(`stackedrecord`'s `opposedLoop`). An oblique line, an ellipse and a
free-form segment miss. Every other interface keeps the exposed records
`stackedrecord.Derive` reads for it.

The opposed interface's faces come from one private Sketch scene over only the
exclusive holes' interiors: each lower-only hole, reversed, as an A region and
each upper-only hole, reversed, as a B region. Every partial edge must carry
`TExact`, the scene carries no walk charge, and
`prismcells.ClassifyRegions` must classify every cell. A cell inside
some B region and no A region is a floor: the lower slab's material under an
upper hole, facing up. A cell inside some A region and no B region is a
ceiling, facing down. A cell inside both is open through both slabs and has no
face. Sketch also merges two lines as one shared span when their carriers
differ within its identity band, so two more checks refuse. Every genuinely
cut line end must lie exactly on its own entity's carrier
(`prismcells.OnLineCarrier`, exact rational arithmetic over the held floats).
Where the displaced line's own endpoint ends the merged span, the coincident
reading records no span, and only this check sees the cut end off its
carrier. Read sampled ends only at genuinely cut bounds: whole edges and
uncut bounds use their authored endpoints, under sketch-seam §1's recording
and loop-closure rules. A circle's shared sampled junction may round off a
whole line's carrier without changing that line's authored endpoint.
Every shared span the reading does record must also pass
`prismcells.CoincidentReading.ExactLineCarriers`: both recorded lines lie on
precisely one line. A lower or upper hole's edge a hair off the
other side's edge (a 5e-13 mm offset in `TestCoincidentReadingRequiresExactFiniteLineContact`) misses, in either orientation. A span on a circle carrier,
an arc of one side lying on a circle of the other, has no exact line, so the
same reading misses it. Two crossing axis-aligned lines meet at their exact
levels, and with both checks such a vertex is exact. A line crossing a circle
or an arc is a cut `sketch` computed and certified with `TExact`, and the
engine places it as A7 does: the line's walked point with its fixed
coordinate exact, keyed by the side of the circle's centre, and charged its
walk, cut and proven crossing distances. The engine's largest vertex
allowance is the face displacement the BRep record states. Any circle or
arc in the result makes its volume `Approximate` with a proven bound; a
result whose loops are all lines reads `Exact`. The incoming displacements
are all zero, so A6 charges nothing.

A curve can meet the other side without a scene vertex. Among line, circle
and arc sources `sketch` never cuts a tangency (`agent-docs/sketch-seam-design.md`),
so a circle touching the other side's edge leaves no point for the touch
reading below. `prismcells.CurvedCrossingsCut` therefore puts every pair of
entities from opposite sides with a curved member to `sketch`'s own
intersection query. Two curves miss on any point it reports, so two
crossing circles miss as well as two tangent ones, and no exact vertex
record is needed for them. A line and a curve miss when the query reads
their carriers as tangent (one point) at a point on both. Every other
point the query reads on both must be an arranged vertex where an edge of
each ends, within `1e-9` relative to the coordinates and the radius. A
crossing the arrangement left uncut, or read as a tangency, has none and
misses. The check only refuses: a matched vertex admits nothing by itself.

Opposed holes that only touch have no manifold result. Along an edge, the
floor, the ceiling and both hole walls would share one edge. At a corner, the
body would meet itself at one point. `stackedunion.opposedManifold` reads
every point where a floor and a ceiling cell meet. Each of the two cells must
turn there from an edge of one side to an edge of the other, and neither edge
may be a span both sides share. That is a proper crossing of one lower and one
upper boundary. Each side's boundary passes a point at most once, so at
most four scene edges meet there, and after the curve reading above they
leave it in distinct directions unless two lines share a span. Every touch
where a floor meets a ceiling fails this reading: two touching corners, two
reflex corners whose outsides meet, or a shared span at its ends. A corner
of one side on the other side's circle from inside it, such as a
rectangle's corner on a round pocket that encloses it there, meets no floor
and ceiling pair: one side's void lies inside the other's at that point,
and the faces round it close one cycle, so it builds. The points and edges are Sketch's own,
compared by exact equality. The check only refuses. A corner touch also
leaves cells that classification cannot reach, which misses as well.

`stackedunion.OpposedCut` emits the record through the A1
`stackedbrep.Engine`. It adds the slab loops, both end caps, every other
interface's exposed records and the opposed interface's cells. The engine
splits the hole walls at the cells' vertices, and it joins coplanar hole walls
of the two slabs into one face. `breprecord.StackedRecord` runs the per-face
section audit and proves closure by counting. The result keeps no stack, so a
later `Cut` of it does not reach the stacked arms. Any miss, including a
touch, leaves the pair to the mesh path. That path refuses the coplanar
contact under `ExactOnly`. With a gap tolerance it builds an `Approximate`
result with the tool moved, where a shift opens the contact
(`agent-docs/gap-tolerance-design.md` §5.2). An exclusive hole with an
oblique line, an ellipse or a free-form segment, two curves of opposite sides
that meet, a displaced level or section, and a union-built or multi-region
stack stay outside this path. So does a side whose second exclusive hole
meets nothing of the other side: no shared edge carries membership to its
cell, which classification cannot reach.

##### A9 — side notch and pockets across holes

`prismboolean.TrySideNotchCut` admits `Cut(target, tool)` for a prism target
with any number of holes and a hole-free prism tool whose loops hold lines,
circles and arcs (`momentinput.LineArcLoops`), and runs A7's admission and
build otherwise unchanged: zero section displacement, the identity relative
map, and one of A7's two arms read by `stackedrecord.CrossingBlindLevels`.

**A9 runs after the mesh path.** `booleandispatch.BodyScaled` tries it only
when the mesh path has refused the `Cut` with `ErrUnsupported`, and returns the
mesh path's refusal when A9 misses or refuses on its own (a cancelled context
excepted). A pair any route builds today keeps that
result: an ordinary side notch stays a mesh result, whose `Body.Section` from
history and retained edge blends (a `Chamfer` of its `Reflex` edges) read it,
where the BRep's consumers do not yet. What A9 takes is the side notch the
mesh path cannot decide: a tool tangent to the outline at recorded vertices
(below), whose contact the proximity gate refuses. Its body takes the operand
roles `boolupstream.Attach` gives every analytic hit. Which payload such a
notch gets therefore follows the mesh path's contact gate: a change there (a
gap tolerance that opens the contact) moves the pair off A9, and
`TestSideNotchCutTangentFork` pins today's route.

A9 lifts A7's scope gates, not its soundness. A7's engine already states a
wall split at an interface on the outer ring as on a hole ring, and every
shape it cannot record misses. `stackedunion.Cut` takes A9's rules in its
`sideNotch` mode; A7 keeps its own (one outer and one hole). Three rules keep
A7's guarantees where its gates no longer stand:

- **No cut on an oblique line.** A7's build states a cut vertex exactly only on
  an axis-aligned line (its fixed coordinate exact) or on a circle. In a Cut
  build, any scene edge that is a `Partial` fragment of an oblique line misses
  (`ubBuild.scene`). An oblique record reaches the result only whole, at its
  recorded vertices: the lever's slot and frame lines, and a tool wall lying
  in the void.
- **One cell.** The reached slab's region is the cells inside the target and
  outside the tool. Two such cells share no edge, since a target edge has
  target material on one side only and a tool edge has the tool on one side;
  several of them are separate lumps or touch at a point, and miss
  (`ubBuild.cutSlabRegion`).
- **No pinch.** `prismcells.RingContact` over that one cell refuses two rings
  touching at a point the arrangement leaves uncut (a round pocket tangent to
  a hole). A ring touching itself there (a round notch tangent inside the outer
  wall) is refused by the section audit (`AuditMergeSection`), which every cut
  slab region runs. The cell may then carry any number of holes.

The last two rules are reject-only defenses that no current A9 fixture
reaches: the shapes they name (a notch across the whole width, a round pocket
tangent to a hole or to the wall from inside) build on the mesh path before A9
runs, and in direct probes of the build the slab merge and the section audit
decline them first.

**Tangent contact at recorded vertices.** A tool whose boundary meets the
target's only at recorded vertices of both, tangentially, builds: the
arrangement cuts nothing there, the vertex is one canonical vertex, and the
walls meeting at it (the target's corner round over both slabs, the gap round
below the floor, the tool's arc above it) are swept pieces joined at that
vertex. Where two arcs end at such a vertex along one tangent ray, sketch
certifies the vertex for exact port order (sketch's `certifySharedCuspEnds`),
without which the face walk lost the cells. The floor cell narrows to a
zero-angle point there. A line-only A9 result keeps its source prisms
(`PointQueryCut`) as A7's does, and point containment reads them for a target
with one hole.

`apitest/side_notch_cut_test.go` builds the watch lever's tangent fork notch
with no, one and two oblique slot holes, declines a notch whose oblique walls
cross the corner rounds, and pins the results the earlier routes keep.

#### Class B — perpendicular prism pairs with planar contacts

Both operands are prisms (or stacked prisms, or class-B results: §4), their
sweep directions are exactly perpendicular, and every face pair that can
meet is a plane against a plane, or a cylinder against a plane that is
parallel or perpendicular to the cylinder's axis. The intersection curves are
then lines and circles only. The result is the `brepPayload` of §4.

##### B.1 Entry gate (reject-only, silent fallback)

| # | Condition | Why exact |
|---|---|---|
| B1 | Both operands expose the face view of §4.1: `prismPayload`, `stackedPrismPayload`, or `brepPayload`. A `Cut` target may also be a Revolve read through its prism view or its stack of coaxial prisms (§3 "Revolved shaft as a prism"). | structural |
| B2 | Every recorded segment of both is `LineSeg`/`CircleSeg`/`ArcSeg` (prism-boolean G4's reason). | `TExact` is whole-scene |
| B3 | The composed world normals are exactly perpendicular: `N_A · N_B == 0.0` over the stored `r3.Vec` floats (`ApplyDir` on each frame's `N()`). | a decision over stored floats, as G3 is; a near-perpendicular pair takes the mesh path |
| B4 | The relative frame map is a signed axis permutation: every dot product among `{U_A, V_A, N_A}` and `{U_B, V_B, N_B}` (world, through `ApplyDir`) is exactly `0`, `1` or `−1`. | keeps every trace line exact (§5); the datum planes XY/XZ/YZ and their offsets meet it; a general pair waits on A6's charge (§8) |
| B5 | Both operands carry `sectionDelta == 0`. | the traces and chords of §5 inherit no displacement to amplify |
| B6 | Every segment of each operand is parallel or perpendicular to the OTHER operand's normal, read in its own frame: with `n' = (N_other · U, N_other · V)` (exact by B4: a signed unit axis or zero), a `LineSeg` has `d · n' == 0` or `d × n' == 0` exactly over the recorded endpoints; a `CircleSeg`/`ArcSeg` has no direction and passes. | a line wall neither parallel nor perpendicular to the other sweep meets a cylinder in an ellipse (class C) |
| B7 | No curved wall of one operand comes within reach of a curved wall of the other: for every pair of circular segments across the operands, the two walls' bounding boxes (prism-extent, outward-rounded) are separated along some axis by an exact comparison. | cylinder × cylinder is a quartic; refused unless provably apart, which may refuse a valid pair |
| B8 | No cap or wall of one operand is coplanar with a face of the other (same plane, exact: the two planes' recorded normals bit-identical and their origins' difference exactly along it), unless the pair is also co-directional, which class A owns. "Flush cap contact" below lifts B8 for a tool cap alone. | a coplanar contact across a perpendicular pair is an edge-on contact the per-face scenes do not classify |

Every miss takes the mesh path with no error. The arrangement cap
(`prismcells.MaxArrangementSegments`) and cancellation are prism-boolean §10's,
charged per scene.

##### B.2 What is refused, and stays refused

| Shape | Why |
|---|---|
| cylinder × cylinder, non-coaxial (S2, S7, a boss on a round hub) | a quartic space curve; no exact record |
| cone, sphere, torus against anything (S8) | no line/circle curve class; S8 also refuses at the mesh path's facet cap |
| a plane oblique to a cylinder's axis (S10, a chamfered hole mouth cut sideways) | an ellipse: class C, after `sketch` certifies elliptical cuts and `Ellipse3` enters the curve vocabulary |
| sweeps neither parallel nor perpendicular (S10's 30° plane against a box cap) | its contacts are lines, but B4's exact traces do not exist; waits on A6 |
| free-form segments | seam §1's whole-scene gate |
| a sheet operand | core §13, unchanged |

##### B.3 The built reach

`Cut`, `Union` and `Intersect` build class B for an operand X read through
its face view (a prism, a stacked prism whose slabs keep one outer loop and
one region, or a brep body) and a prism Y (`internal/classbgeom/entry.go`). `Cut` takes X as the
target; `Union` and `Intersect` try the operands both ways, so a brep or
stacked operand enters in either position. Beside B1–B8, the record of §4.1
needs one shared placement (`X.xform == Y.xform`), Y's axes carried bit for
bit as signed axes of X's reference frame (its first face's), and every
segment of both over its natural range, walked either way. Y's record is
moved into `g`, Y's axes at the reference origin, by the exact rational dot
of the origin difference with each axis. The pair is admitted only when
every moved coordinate and level is a float, so a datum plane and its
`CreateOffsetPlane` at a float distance meet it and a shift that would round
misses. B3 binds a prism or stacked X; a brep X has no single sweep and takes
Y along any reference axis, so a cross-drilled bar takes a hole down its own
caps.

Within that, the built reach is the through-nesting one, decided by exact
comparisons of recorded coordinates. Let d be the reference axis Y sweeps
along, and T Y's tube: its section's outward box (arcs by their radius
rounded outward) swept over its interval. The faces of X whose boxes,
widened by their level displacements, are not strictly apart from T are one
or two, each across d: a planar face whose normal lies on d, or a straight
wall at constant d. Each sits strictly inside Y's interval, both level
displacements as margin, and two of them bound X's material between them
(the lower outward −d, the upper +d). No other face of X meets T, so inside
Y's tube the only boundary of X is those faces' patches over Y's section,
and each face's perpendicular-face scene (§5) proves that patch lies inside
its region by the clean-nesting match. Over Y's tube X is then the slab
between two faces (through) or the side of one face its outward normal
points away from (rooted), so every result is Y's section swept over an
interval joined to X's faces:

| Op | Through (two faces) | Rooted (one face) |
|---|---|---|
| `Cut` | Y's walls reversed between the faces | Y's walls reversed over the inside part, and Y's section as the hole's floor |
| `Union` | Y's walls and caps over both outside parts | Y's walls and cap over the outside part |
| `Intersect` | Y's section between the faces: a prism | Y's section over the inside part: a prism |

Each slab face becomes a planar face in `g` carrying Y's section as a new
hole: a straight wall by the rectangle it sweeps, a planar face by its region
through the signed permutation between its frame and `g`, walked back
(`prismcells.RewindLoop`) where that map reverses the plane. No edge of either section
is cut, so the section and parallel-face scenes are not built. S1's
cross-drilled box, B1's cross slot, a blind cross hole, a pin rooted in a
wall, S11's chain of cross holes and a cross hole under a blind pocket are in
the reach.

A `Cut` of a prism target outside it takes the crossing reach
(`internal/classbgeom/crossing_cut.go`), whose scenes cut edges: B2's keyway, a hole breaking
out of a face, a tool crossing a wall. It needs both operands hole-free, every level
displacement zero (a positive charge from `OfPayloads` refuses it with `ErrUnsupported`), and a frame for faces normal to e that carries X's axes
bit for bit. Every planar face of either operand is decided by its own
scene (§5), every crossing a scene computes takes the one float §10's keyed
crossing table records for it, every edge is split at every vertex on it,
and each cylinder's pieces are read off the planar faces across its axis
(§5). A piece whose arc end another piece of the same frame meets at a level
strictly inside its interval splits that side edge there, so a flat opened in a
shaft's wall pairs edge by edge. A trace with more than one chord, a crossing too close to tangency to
key, a scene whose cells this reach does not cover, and a crossing result
with a hole each send the pair to the mesh path; a face that fails modify
§5's audit, or a result whose edges do not pair, refuses.

##### Flush cap contact

A pin whose end cap lies flat in a wall of the target (§2 B5), a blind
hole whose tool ends exactly in the target's face, a pin bridging the gap of
a U-shaped plate from wall to wall, and a through hole whose tool runs
exactly from one wall of the target to the opposite one share one or two
planes across a perpendicular pair, which B8 refuses. `classbgeom.OfPayloads` lifts B8 for
exactly this contact when the caller charges no displacement beyond the
records (`delta == 0`). Route S's shell cavity cuts pass their charge `c`
(modify-general §3.3 step 4), which is zero when no cavity level or tool
level carries a displacement; such a cut lifts B8 as class B's own booleans
do, and a positive charge keeps B8.
`CoplanarFaces` compares every target cap and wall plane with every tool
plane by exact equality of recorded axis and level. A target plane on a tool
wall's plane still misses. A target plane on a tool cap's plane marks that
cap, and a marked pair takes only the flush reach (`FlushReach`) and, where
it misses, the sweep walk of "Pin across several faces" below, never the
through or crossing reach:

- The tool's tube meets one or two target faces' boxes (the through reach's
  box test, unwidened), and each face lies across the tool's sweep.
- Each face sits at a marked cap's level, compared as recorded floats, two
  faces at the two distinct caps, and each face's level displacement and its
  cap's are both zero, so the face and the cap denote one plane. A marked cap
  whose plane holds only faces apart from the tube is not the contact; when a
  face meets the tool at no marked cap (a face crossing the tube strictly
  inside its interval), or two faces sit at one cap's level, the pair misses.
- No other face's box meets the tube, so the open tube holds no boundary of
  the target and the tool lies wholly on one side of it: outside when a
  face's outward normal points from its cap into the tool, inside otherwise.
  With two faces both must say the same side, or the pair misses; recorded
  outward flags alone decide it.

With one face the result is the rooted build of B.3 with one part of the
tool's interval empty. `Union` of a tool outside keeps the face with the
tool's section as a hole, the tool's walls over its whole interval and its
far cap. `Cut` of a tool inside keeps the face with that hole, the tool's
walls reversed over its whole interval and the hole's floor at the far cap.
`Intersect` of a tool inside is the tool's own prism. With two faces a tool
outside bridges a gap of the target whose two faces point at each other:
`Union` keeps both faces with the section as a hole and the tool's walls
over its whole interval, and no cap. A tool inside runs from face to face:
the through build of B.3 with both outside parts empty, so `Cut` keeps both
faces holed and the tool's walls reversed over its whole interval, and
`Intersect` is the tool's own prism. The other three pairs miss to the
mesh path. A `Union` of a tool inside is the target itself, which the mesh
path builds. A tool outside meets the target in its end disks alone. With
one face the mesh path decides it: its `Cut` leaves the target whole and
its `Intersect` is empty. With two faces the mesh path refuses its `Cut`
and `Intersect` with `BooleanUnsupportedContact`, because the held facets
of both disks come within the chord tolerance without provably crossing. Each
face's region takes the section as a hole through the same clean-nesting
match, so a section touching the face's boundary misses. Each hole's rim is
the wall's rim at the shared level, one circle bounding one swept wall, so
§4.2's refusal of a rim shared by two cylinder pieces does not arise. A
tool one ulp off the face, a cap level that rounds onto the face's
float (an inch length converted to 10 mm), and a face whose level carries a
displacement keep the mesh path. A pin bridging the gap and running on into
the far wall, by any length down to one ulp, meets that wall strictly inside
its interval, so the flush reach misses it and the sweep walk below builds
it; one stopping one ulp short of the far wall meets the near face alone and
builds as a pin standing on it.

##### Pin across several faces

A pin through both prongs of a U-shaped plate, a pin through a slotted
plate, and a pin bridging the U's gap and running on into the far prong meet
more than two faces of the target, or two faces in a pattern neither the
through reach nor the flush reach builds. When both of those miss and the
caller charges no displacement (`delta == 0`), class B walks the tool's
sweep (`WalkReach` in `internal/classbgeom/walk_reach.go`, built by
`buildClassBWalk` in `internal/classbgeom/walk_build.go`). Every comparison
is between recorded floats, and every gate misses silently to the mesh path:

| # | Condition |
|---|---|
| W1 | The tube (the through reach's box test, unwidened) meets at least one face's box, and every face whose box meets it lies across the tool's sweep (`Across`). |
| W2 | Every such face's level displacement is zero, and so are both of the tool's cap displacements. |
| W3 | In `g` the tool spans `[a, b]` with `a < b`, and the faces' levels `s_1 < … < s_N` are strictly increasing. |
| W4 | Consecutive faces point opposite ways along the sweep: `up_i ≠ up_{i+1}`, where `up_i` says face `i`'s outward normal is `g`'s `+N`. |
| W5 | Each face's region holds the tool's section by the clean-nesting match of its perpendicular-face scene (§5). |
| W6 | The operation changes the target in the tube and states one body: below. |

The box test places every level in `[a, b]`, so a level equal to `a` or `b`
is a flush cap, and only `s_1` or `s_N` can be one. Write `S` for the tool's
section (one outer loop and no hole, so its interior is connected),
`T = S × [a, b]` for the tube and `B` for its box.

The proof that W1–W5 decide the target's membership around the tube runs
in five steps.

1. A face whose box is strictly apart from `B` lies at a positive distance
   from it, so some open box `U` around `B` meets no face but `F_1 … F_N`.
   Each `F_i` lies across the sweep at level `s_i`, so inside `U` the
   target's boundary lies in the `N` planes `z = s_i`.
2. The planes cut `U` into `N + 1` open layers, each an open box holding no
   boundary point. Membership in the target is therefore constant on each
   layer: `x_0` below `s_1`, `x_k` between `s_k` and `s_{k+1}`, and `x_N`
   above `s_N`.
3. By W5, `S` lies strictly inside `F_i`'s region, so over `S` the face is a
   boundary patch of the target whose outward normal is `up_i`. The layer
   above it is outside exactly when `up_i`: `x_i = ¬up_i` and
   `x_{i−1} = up_i`. W4 is the condition that face `i` and face `i + 1`
   agree on the layer between them. Flags that break it describe no solid,
   and the pair misses. Together, `x_0 = up_1` and the layers alternate.
4. The tool's interval splits into `I_0 = [a, s_1]`, `I_k = [s_k, s_{k+1}]`
   and `I_N = [s_N, b]`. `I_0` is empty exactly when `a = s_1` and `I_N`
   exactly when `s_N = b`, a flush cap. Every interior interval is nonempty
   by W3.
5. Inside `T` the tool's membership is 1, so on `I_k` the result is
   `op(x_k, 1)`: 1 for `Union`, 0 for `Cut`, `x_k` for `Intersect`. Outside
   `T` it is 0, so the result is the target for `Union` and `Cut` and empty
   for `Intersect`.

The result's boundary inside `U` lies where the result differs across a
surface. For `Union` and `Cut` write `t` for the tube's value (1 and 0):

- Face `F_i` over `S` has the tube on both sides unless it is a flush cap.
  At a flush cap at `a` the side below lies outside `T` and holds `x_0`; at
  `b` the side above holds `x_N`. With `t` on both sides the patch leaves
  the boundary, and the face takes `S` as a hole strictly inside its region
  (W5). Where the two sides differ, the patch stays with the face's own
  sense: at `a` they are `x_0 = up_1` below and `t` above, which differ only
  when the material lies on the side `F_1`'s outward normal points away from.
  The face is kept whole, its record unchanged.
- The tool's cap at `a < s_1` has `x_0` below and `t` above. It is a face of
  `S` when they differ: outward `−N` for `Union` (`x_0 = 0`) and `+N` for
  `Cut` (`x_0 = 1`, the floor of a blind hole). The cap at `b > s_N` is the
  mirror case: outward `+N` for `Union` and `−N` for `Cut`.
- The tool's walls over `I_k` have `t` inside and `x_k` outside: `Union`
  keeps them where `x_k = 0`, and `Cut` keeps them reversed where `x_k = 1`.
- Outside `U` the result is the target, so every face apart from the tube is
  kept, and each `F_i` keeps its region outside `S`.

`Intersect` is the union of `S × I_k` over the nonempty `I_k` with `x_k = 1`.
Two such intervals never touch, because the layers alternate.

| Op | W6: builds when | Result |
|---|---|---|
| `Union` | some nonempty `I_k` has `x_k = 0` | faces apart from the tube; each `F_i` holed or kept whole; the walls over each outside `I_k`; the caps above |
| `Cut` | some nonempty `I_k` has `x_k = 1` | faces apart from the tube; each `F_i` holed or kept whole; the walls reversed over each inside `I_k`; the floors above |
| `Intersect` | exactly one nonempty `I_k` has `x_k = 1` | the prism `S × I_k` |

A `Union` or `Cut` that W6 refuses is the target itself, and an `Intersect`
with no inside interval meets the target in disks at most. The mesh path
decides those, as for the flush reach. An `Intersect` over two or more
inside intervals is several separate prisms, which this build does not state,
and it keeps the mesh path.

The edges pair. Under `Union` an interior face has an outside interval on one
side and an inside one on the other, by the alternation, so exactly one wall
piece ends at its hole's rim. A holed flush face at `a` has `x_0 = 1`, so
`x_1 = 0` and the wall over `I_1` ends at its rim. A cap at `a` has
`x_0 = 0` and the wall over `I_0` ends at it. `Cut` is the same with the
inside intervals. No two wall pieces meet, so §4.2's refusal of a rim shared
by two cylinder pieces does not arise.

On the U plate of "Flush cap contact" (500 mm² × 10 mm), a Ø6 pin from the near prong's
inner wall `x = 10` to `x = 35` has `x_0 = 1`, a flush low cap, `x_1 = 0`
over the gap, and `x_2 = 1` in the far prong. `Union` holes both walls and
adds the wall over the gap, 5000 + 180π. `Cut` keeps the near wall whole,
holes the far wall and drills a blind hole of depth 5, 5000 − 45π.
`Intersect` is the prism over `[30, 35]`, 45π. A tool cap level that carries
a displacement (an inch length), a section that crosses a face not across
the sweep (the plate's top), and an `Intersect` through two prongs keep the
mesh path. A pin ending exactly at a face is a flush cap, and pins one ulp
short or long of it meet one face fewer or one face strictly inside. All
three build.

Once W1–W4 hold, the face scenes of W5 run inside `buildClassBWalk`, as they
do in `buildClassB`. A scene that returns no clean nesting misses, but a
scene past its work cap and a scene error refuse the operation instead of
falling back to the mesh path.

Route S's shell cavity cuts reach the walk too, because they call
`OfPayloads` with a zero charge whenever no cavity or tool level is
displaced (modify-general §3.3 step 4). The walk's proof holds for any pair
of records that pass W1–W6, so such a cut builds the exact eroded prism less
the dilated tool. The cavity record then still passes Stage 7's
`breprecord.Build` checks (its falsification, its edge pairing and its
volume check), which reject a record that does not close.

#### Revolved shaft as a prism

A turned shaft is a stack of coaxial circular prisms. A full solid `Revolve`
whose meridian is one staircase with one side exactly on the axis denotes the
set of points whose axial coordinate lies in some step's span and whose
distance from the axis is at most that step's radius. The prisms whose
sections are the circles of those radii about the axis point, each swept over
its step's span along the axis, denote the same set together.
`revolveplan.Payload.CylinderSteps` builds those prism records, in order
along the axis, each one's `Z1` the next one's `Z0` bit for bit. A plain
turned shaft is the stack of one step, which `CylinderPrism` returns. A `Cut`
whose target is such a Revolve and whose tool is a prism enters class B: with
one step, the prism record is X (`classbgeom.CutPrismView`); with more, the
stack is X (`classbgeom.CutSteppedView`), both from `booleandispatch.classB`.

The view admits by exact comparison of recorded data only, and every miss is
silent, so the pair keeps the mesh path:

- The record is a full turn of a solid with no section displacement, no
  whole-section charge and no blend or draft roles.
- The axis runs along the plane's U or V: the recorded direction is
  `(±1, 0)` or `(0, ±1)`, and the anchor and direction bounds, the radial
  admission allowance and every snap allowance are zero.
- The meridian is one outer loop of whole `LineSeg`s, each joined end to
  start bit for bit, each along or across the axis and of nonzero length.
  Exactly one line, along the axis itself, spans `[lo, hi]`. The other lines
  along the axis, the rims, lie on one side of it; sorted by their start
  (then their end) along it, they tile `[lo, hi]` end to start bit for bit,
  and neighbouring rims differ. The lines across the axis, the risers, number
  one more than the rims.
- Each radius, a rim's across value less the axis's, is a float exactly (an
  exact rational difference) and exceeds the axis classification tolerance.
- `r3.NewFrame` rebuilds the prism frame from the plane's axes bit for bit:
  (V, N, U) for an axis on U, (N, U, V) for an axis on V. The frame shares the
  plane's origin, so each circle's centre is the axis anchor's across
  coordinate and every level is a recorded along value.

These conditions make the meridian the staircase. The lines along the axis
end at `2n + 2` distinct points for `n` rims, one line at each: the tiling
and the different neighbouring rims keep two rims from sharing an end, and
no rim lies on the axis. Two of those points share each along value. The
loop is a closed walk, so each point is the end of an even number of its
lines and needs an odd number of riser ends. The `n + 1` risers have
`2n + 2` ends, so each point ends exactly one riser and no riser ends
anywhere else, and a riser keeps its along value, so it joins the two points
at one along value: the riser at `lo` from the axis to the first rim, the one
at `hi` from the axis to the last, and one at each step boundary between the
two rims there. The at-least-one-rim condition follows from the closed walk
and the riser count; the code keeps it as a guard. The one-axis-line
condition does not follow: a zero-area spike out along the axis and back
adds two axis lines whose ends pair with each other, and without the
condition the last axis line read would set the span.

The prisms use the same frame vectors as the plane in permuted order and
state the same set in the frame's coordinates, so no lift differs and the
view carries no displacement. A tube (a meridian off the axis), any arc or
slanted meridian line and a staircase whose rims overhang have no view.

Beside B1–B8 and the crossing reach's own conditions, the view's `Cut`
misses unless every straight tool wall across the shaft's axis lies at a
level strictly outside the shaft's span. Under B6 every other tool line runs
along the axis, and under B7 every tool arc is apart from the whole shaft, so
inside the shaft the tool's boundary does not change along the axis. Every
surviving piece of a step's wall then runs from one of the step's bounding
planes (an end cap or a shoulder) to the other. A tool that ends inside the
span would cut the wall into pieces of different angular extent that share a
rim, which §4.2 refuses. For one step, `CutPrismView`'s gate keeps that tool
on the mesh path. For a stack, the stepped build misses when a tool wall
across the axis lies strictly inside the span (`BuildYCrossingFaces` under
its `XSlice` hook), and B8 misses one in an end plane. An Extrude target is
not gated this way and refuses there. The through reach is not tried: a
cylinder X has no face across d.

##### The stepped crossing build

A stack's entry gate runs on its envelope, the circle of the largest radius
swept over the whole span (`steppedEnvelope`): B1–B8 read only its end caps,
its circular wall and its span, and each bounds every step, so a pair the
envelope admits meets no step in a way the gate refuses. The envelope needs
two or more steps sharing one frame and placement, each one whole
counter-clockwise circle about one centre with no displacement, every span
positive, each step's `Z1` the next one's `Z0`, and neighbouring radii
different. The shoulders lie strictly inside the span, and every tool face
normal to the axis lies outside it, so no tool plane holds a shoulder.

The crossing reach then runs over the stack's own faces (`stepped_cut.go`,
with the hooks of `crossing_cut.go` and `face_build.go`). The wall of step k
is X carrier k, the end caps are −1 and −2 as for a prism, and the shoulder
at the start of step i is −(2+i), a plane across the axis at that step's
`Z0`:

- Each end cap is its end step's disk, and each shoulder is the annulus
  between the two steps' circles. The wider step's circle is its outer loop
  and the narrower one's its hole, and it faces away from the wider step.
  Each is decided by its own scene against the tool's chord in its plane
  swept between the tool's caps, as a prism's cap is.
- In the plane of each tool cap or tool wall along the axis, X's slice is
  the union of each step's chord swept over its span. Each step's chord is
  its circle's trace through `sketch`, with each end pinned in the keyed
  crossing table at the step's two bounding planes. The steps the plane meets
  must be consecutive, so the slice is one staircase loop: up the chords'
  high ends, each step's wall joined to the next by the shoulder between
  them, across the top step's bounding plane, and down the low ends. Steps
  apart from the plane on both sides of one that meets it would cut the slice
  into several loops, and the pair misses.
- A face whose box is apart from every loop of the tool's slice, by an exact
  comparison of outward segment boxes, keeps its region whole: a step the
  tool does not reach.
- A shoulder whose hole is box-apart from the tool's slice is decided by its
  outer loop's scene alone, and the hole joins the one face that scene keeps.
  The hole lies strictly inside the outer circle and apart from the slice,
  so its disk lies in the kept area and its circle touches none of its
  boundary; with more than one kept face (a slot through the wider step) the
  pair misses rather than place it. Otherwise the scene holds the hole too,
  and a cell with a hole is outside `prismcells.Classify`'s reach, so the
  pair misses.
- Each step's wall pieces are read off the planar faces across the axis
  (§5), between the step's two bounding planes, and each crossing takes the
  one float the keyed table records for its cylinder, plane and side, at
  every axial level.

The result is a class-B `brepPayload`, and `breprecord.Build`'s
falsification, edge pairing and volume check run on it.

##### Coaxial stepped through-bore

`CutSteppedView` also admits a prism tool whose extrusion runs along the
stepped shaft's axis and spans the shaft's full interval. The tool and shaft
must have zero displacement and a relative rigid map whose axes match by
signed permutation. That map must re-express every outline coordinate and
both axial levels exactly, including when the tool extrudes opposite the shaft
axis. The tool must have one natural, closed line/arc/circle outline and no
holes. Every segment's conservative exact box must fit strictly inside the
smallest step circle, so the bore cannot touch an outer cylinder or a
shoulder.

For each step, `prismcells.ResolveCut` asks Sketch to prove that the tool
outline becomes one new hole in the recorded circle, with zero scene charge.
The B-rep keeps the source outer cylinder for each step, the reversed bore
loop as its inner wall, the cut profiles on both end caps, and the unchanged
annulus at every shoulder. `breprecord.Build` checks the resulting face
pairing and volume. A blind tool, breakout, touching box, displaced pair,
incompatible frame, or unsupported curve misses this route and keeps the
existing fallback.

##### Coaxial linear Revolve through-bore

`Cut` admits a full solid Revolve with one whole-line meridian, one axis
edge, two radial ends and a monotone axial outer run. The outer run may
include slopes, cylinders and radial shoulders; at least one wall must slope.
The prism tool has one natural closed line/arc/circle section and no holes.
Both caps must coincide exactly with the Revolve ends. Both sweep signs are
accepted. Axis, section and level displacement, blends, draft roles, rounded
relative maps and insufficient radial clearance miss the route. Circular
segments use their complete carrier for the conservative clearance proof.

Sketch certifies the two nested cap regions. A fresh body keeps the Revolve
walls and source origins, adds two holed end faces and inward prism walls, and
checks edge incidence and vertex links. Its volume and mass moments subtract
the tool from the Revolve with bounded arithmetic. The tessellator builds
both source meshes and projects their cap vertices onto the common exact cap
planes for its proof. It replaces their caps with shared-rim annuli, audits
the source and combined facets under the projection displacement, and adds
the resulting area and swept-volume allowances to the source mesh bounds.
The facet audit uses a bounded spatial candidate scan.
The projection changes only the coordinate omitted by the cap's 2D chart, so
the virtual cap triangulation uses the same chart points and shared rim
indices as the held mesh. The projected source caps share the certified plane;
their nested sections make the virtual result the regularized source-mesh
difference. The source and result vertex homotopies preserve embedding under
the expanded facet audits. Mesh and projection allowances are converted from
world lengths to local radii through the frame's lower length factor.
Folded outer runs, cone-crossing tools and blind tools remain outside this
route.

The class-B result of a plain shaft is the `brepPayload` an Extrude shaft
gives: the full-length D-flat and keyway of
`apitest/revolve_shaft_class_b_test.go` measure the same value and bound bit
for bit as their Extrude twins. The crossing charges the flat's section
displacement. Brep-modify §4.4 fillets and chamfers the lateral edges of the
D-flat and keyway from the record's exact carriers when the selection blends
every crossing corner; the stepped flat, a partial selection and an
asymmetric chamfer keep SB1. `Thread` on the cut wall builds, to its Extrude
twin's volume. A shaft
or a tool placed by `Transform` carries its own placement, misses the
shared-placement condition of B.3 (`AdmitPair`) as it does for an Extrude
pair, and keeps the mesh path, where retained-edge-blend §2 blends those
edges.

#### Common-axis full revolutions

Two solid full-Revolve bodies with the same stored plane frame, placement,
axis anchor and direction share one rotational motion. For this bounded
class, `Union`, `Cut` and `Intersect` arrange their authenticated meridian
profiles in one private Sketch scene. Sketch decides each cell's membership
and certifies every retained cut parameter. The result revolves the selected
meridian cells through one full turn, so a shared axial cap or cylindrical
wall is classified in the source plane instead of by chorded mesh contact.

The admission requires one simple outer loop per operand of line, arc and
circle segments (a sphere is a half-disk of one arc and the axis line), no
meridian holes, no section displacement, a matching full-turn angle record,
and one resulting closed loop. Every input segment must have a distinct scene
entity so its source-face role is unambiguous. The existing prism-cell work
cap and cancellation apply. A result loop of lines only, whose every cut end
is a Sketch-certified line crossing at an exact binary64 point, is restated
as whole lines between those points (`prismcells.MergeWithChainExactLineCuts`,
the exact line-crossing restatement of §3 "A1 as a brep") and carries no section
displacement. A groove cut into a turned shaft is such a loop: its walls
cross the shaft's rim at the groove's own recorded coordinates, although the
cut parameters along the rim are rounded. A groove drawn flush with the rim
is not: each wall ends on the rim as a whole edge, so the rim's cut end
meets a recorded endpoint rather than a second cut, and the loop keeps the
charge (`apitest/revolve_groove_blend_test.go`,
`TestRevolveGrooveDisplacedMeridianRefuses`). Any other result cut through a
source line or arc carries the cut parameter's displacement through
`revolvePayload.SectionDelta`, and a blend or shell of it refuses
(`revolveplan.RequireExactSection`). A kept
wall retains its source face's origins; a coincident wall retains both source
faces' origins. Every shared span must lie on exactly the same recorded
carrier (`prismcells.CoincidentReading.ExactCarriers`). Sketch's identity band
alone cannot join a positive gap, so the reading only refuses:

- A span of two lines lies on precisely one line, with both ends inside both
  finite segments.
- A span of two arcs or circles (a dome on a ball of its radius, a ball on an
  equal ball, a half-torus on its torus) lies on one circle: both entities
  record the same centre, and every recorded value that defines either (a
  circle's radius, an arc's start and end) gives the same squared radius,
  compared as exact rationals over the held floats. A span end that is an
  entity's own endpoint (an arc's start or end, a circle's seam at one radius
  along +u) is read from that record, and two such readings must agree
  exactly. Sketch's reported vertex there is a sampled point that can lie off
  the circle (a circle's seam at parameter 1). An end both entities share as
  a cut is Sketch's reported vertex, read only as a direction from the centre.
  Each arc must hold the span counter-clockwise from its first end to its
  second, by exact cross and dot signs (the first end on the arc, the second on
  the arc's remainder after it). A circle withdrawn whole must have a circle
  partner.

Classification reads an admitted span as a boundary of both operands (A3),
so a dome `Intersect` a ball of its radius is the dome, their `Union` the
ball, and the ball `Cut` by the dome the other hemisphere. Two arcs Sketch
resolves onto one carrier that the reading falsifies (an arc centred 1e-13 mm
from the other's centre, or one whose endpoint is a cos/sin rounding off the
radius) continue to the mesh path. Two equal whole circles are not reached:
Sketch publishes no cell for the pair, and the scene misses. An arc withdrawn
into a window whose ends meet no partner's fragment ends (radii one ulp apart)
is not a shared span, and the exact-carrier reading never sees it. The cells
Sketch publishes without that arc are classified as in any other scene, and
the outcome depends on which operand holds which arc. A dome of radius 10
`Intersect` a ball one ulp larger, and a ball of radius 10 `Union` or `Cut` by
a dome one ulp larger, build a body whose published volume bound contains the
true volume. A ball one ulp larger than a radius-10 dome, under any of the
three operations, and a dome one ulp larger `Intersect` a radius-10 ball miss
and continue to the mesh path
(`apitest/revolve_boolean_shared_arc_test.go`,
`TestCommonAxisRevolveSharedArcSpanUlpRadii`).

An arc cut parameter is certified by Sketch (`TExact`) and its rounding is charged
through `CutDelta` in `MergeWithChain`. A cut arc keeps its source
face's origins, so a sphere or torus wall cut by a coaxial plane or cylinder stays
a sphere or torus. The pole
needs no new rule: the arc meets the axis only at its own recorded end, a
disk that touches that point is a point contact and is not promoted, and the
facet ceiling of the mesh path does not apply because the Boolean does not
mesh the sphere. An empty `Cut` or `Intersect` returns
`BooleanEmpty`.
An unresolved arrangement, a disconnected result, a different axis, a
free-form meridian, or a shared span the exact-carrier reading refuses
continues to the existing mesh path and its documented error behavior. An edge-only or point-only contact is not promoted to a
solid join.

### 4. The `brepPayload`

#### 4.1 Record

```go
// brepFace is one analytically trimmed face.
type brepFace struct {
    frame   r3.Frame       // the face's plane (planar) or the sweep frame of its wall (swept)
    // exactly one of:
    region  *profileRecord // planar: one outer loop and holes in frame coordinates, material left of each walk
    outward bool           // planar: true when the outward normal is frame.N()
    sweep   r3.Vec         // planar: the direction, along a reference axis, a restated straight wall sweeps; zero for a cap
    wall    CurveSegment   // swept: a LineSeg/CircleSeg/ArcSeg in frame coordinates, material on its left
    z0, z1  float64        // swept: the sweep interval along frame.N(); planar: the level, z0 == z1
    z0Delta, z1Delta float64
    side0, side1 []brepSplit // swept: the levels strictly inside (z0, z1) at which each side line is a vertex, ascending, each with its level displacement
    delta   float64        // the face's own section displacement (prism-boolean §7 terms)
    role    string
    wallCarrier circleCarrier // swept circular face of a class-B crossing build: its operand circle's centre and radius
}

type brepPayload struct {
    faces []brepFace
    xform r3.Transform
}
```

The code's types are `breprecord.Face` and `breprecord.Payload` (`internal/breprecord/payload.go`), with exported
field names. They add `Blend` (route E's fillet or chamfer face), `Stack`, `PointQueryCut`, `SlabCut`,
`CarrierCharge` and `LoopBands`, which other documents own.

A prism is a `brepPayload` with two planar faces and one swept face per
segment; a stacked prism is one with its columns and planar patches. The
face view of §B1 is this record, built on demand from either payload and
never stored for them. A class-B result stores it. Every face's frame is
stated in the payload's unplaced coordinates and `xform` places the whole
body, as every payload does. An A1 result additionally keeps its `stack`:
the slabs it was built from, each slab's regions in the reference frame,
holed where §3's holed-operand gate admitted them, with the section
displacement they carry, read by a further co-directional `Union` (§3 "A1
as a brep"), and by `Body.Section`, `ScaledAt` and point containment only
where that displacement is zero, and unchanged by a placement, which moves
`xform` alone.

`wallCarrier` is set by the class-B crossing build alone
(`internal/classbgeom/crossing_cut.go`), on each cylinder piece whose operand
wall is a `CircleSeg`: that circle's centre in the piece's frame and its
radius in millimetres, the value the crossing charges are taken against. The
recorded wall denotes that circle only to within `delta`, and every other face
leaves it zero. A reversal or an in-place trim keeps it, since the wall stays
on the same circle, and a placement moves `xform` alone. Only brep-modify
§4.4's displaced reading reads it, and its PD3 falsifies it against the
record.

A boolean's cut fragment records its carrier and a narrowed range, so two
fragments meeting at a cut walk to it at two different floats. The face view
joins each such junction first (`brepgeom.JoinLoop`): it takes the line's walked
point, whose fixed coordinate the lerp keeps exact, else the lexicographically
smaller one, so a loop and its reversal choose alike, and rewrites each
segment between its two junctions, a circular fragment as an arc pinned
there. The chosen point's distance from the junction joins the payload's
section displacement: its distance from the point its own segment denotes
there (`boundarywalk.DenotedEndBound`/`DenotedStartBound`, the walk's
rounding plus an arc's radial residual at its natural `t = 1` end), and,
where the two segments lie on different carriers, its proven distance from
their exact crossing (§5). Two fragments of one carrier name no crossing. A
nonzero displacement makes class B's B5 refuse the face view, and
brep-modify's SB1 refuse it outside brep-modify §4.4's displaced reading, so the join
changes a published measurement only where both terms are zero.

Every face frame shares the first face's origin bit for bit and carries each
of its axes, or that axis negated, bit for bit, keeping handedness. The map
between two face frames is then a signed permutation, which moves every
coordinate and level exactly, and §4.2–§4.4 read every face in the first
face's frame, the reference frame. The datum planes XY, XZ and YZ meet this. A
record whose frames do not is `ErrUnsupported`.

#### 4.2 Topology and roles

Faces are the record's. Edges: each planar face's loop segments and each
swept wall's two rim curves (the wall segment at `z0` and `z1`) and two
side lines (a wall's ends at the junction with the next face), each side
line one edge per piece between consecutive levels of `z0`, its splits and
`z1`, so a vertex another face puts on a side line is a vertex of the wall
too. Every edge is
shared by exactly two faces by construction: a planar face's segment is one
rim of exactly one swept wall, or one line of exactly one other planar face
(two planar faces meeting along a line), and the build identifies them by
exact record identity in reference coordinates (same coordinates, same
level). A result whose edges do not pair is `ErrUnsupported`
(`BooleanEvaluatorFailure`): the build proves closure by counting, as the
mesh audit does. A circular edge that bounds no swept wall, and a rim shared
by two swept walls, are `ErrUnsupported` too. `Edge.IsConvex` reads the walked
boundary as evaluator §3 states. A rim reads the wall it runs along: a
circular wall by its own turn, a straight wall by the role of the loop it
belongs to. The planar face sharing the rim states that role: its own loop's
role when it walks the rim the wall's way, the other role when it walks it
the opposite way, as a stacked floor walks a reversed hole.

A line two walls of one sweep share is a junction, convex when the walk turns
left there. A planar face that restates a straight wall records the axis that
wall sweeps along (`sweep`): an A1 wall face the stack axis, a class-B face on
a wall's carrier that operand's axis, a class-B slab and a wall route E
restates (brep-modify §5.2) its source wall's. The
junctions are a side line between two swept walls, a side line between a swept
wall and a planar face whose sweep is the wall's axis, and a line two planar
faces of one sweep share. Two swept walls compare their walk tangents. A
planar wall reads the same turn from its own loop: the direction it runs from
the line into the face (its frame normal crossed with the way its loop walks
the line) makes the junction convex when it points to the inner side of the
other wall. That wall's outward normal is a swept wall's tangent crossed with
its axis, or a planar face's own normal. Every vector there except a swept
wall's tangent is a signed reference axis, so the sign is that tangent
component's sign, exactly. A zero turn reads concave, as two swept walls'
zero tangent cross does. So an L boss's reflex corner, a slot's floor
corners and a pierced L prism's reflex corner read concave. Any other line
reads the owner planar face's loop role: outer convex, hole concave. That
covers a cap meeting a wall, and two planar faces of different sweeps.

Vertices are shared by exact reference coordinates. Each rim and loop-segment
use places a vertex at both ends of its walk, and the vertex publishes the
largest bound any use placing it states. A use states its face's `delta`,
its level's displacement, the vertex's own frame and placement lift rounding,
and the distance from its held walk end to the point its recorded segment
denotes there (`brepgeom.Use`'s `StartBound` and `EndBound`, evaluator §3).
That distance is the walk's own end bound, plus the arc's radial residual at
an arc's natural `t = 1` end, where the record holds `End` verbatim while
the arc denotes Start's radius at End's angle (evaluator §4). Every use
meeting at a vertex holds the same coordinates, so the largest bound reaches
every neighbour's denoted end. A vertex at a whole arc's `End` off Start's
radius therefore carries a bound in every result that keeps the arc: an A1
union, a class-B cut, a route-E rewrite, a shell side opening. A vertex where
natural line ends and an arc's `t = 0` Start meet at one recorded coordinate
stays `Exact`.

`Lumps` and `Shells` are derived from face adjacency (`topobuild.SheetLumps`). A closed
rectangular-boss shell uses its proven cavity clearance to group the source
outer faces as one shell and the disconnected cavity faces as one void shell
(modify-general §3.1a). Roles are fresh under the boolean's producer identity:
`face(k)` for planar faces, `wall(k)` for swept, both indexed by the result
record, and `capStart`/`capEnd` are not minted (core §9: the helper returns a
reference matching nothing). A planar face also inherits the roles of the one
operand face exact plane identity proves it lies inside (prism-boolean §11), so
`FaceCreatedBy(CapEnd(operand))` selects it; the stacked record of §3 A1 follows
the same rule.

#### 4.3 Measurements

Every quantity is a sum over faces of a closed form in that face's own
record, read in reference coordinates. Volume is `(1/3)∮ p·n dA` and the
first moment along reference axis `i` is `½∮ x_i²·n_i dA`. A planar face at
level `z` with outward sign `s` (`+1` when `outward`) has its normal on one
reference axis `k` with sign `σ`; a swept face over height `h = z1 − z0` has
its `u` and `v` on two reference axes with signs `σ_u`, `σ_v`. Each segment
contributes its Green's-theorem terms about the frame origin in the forms
evaluator §4 accumulates: `g = ½∫(u dv − v du)`, `mu = ½∫u² dv`,
`mv = −½∫v² du`.

| Quantity | Planar face | Swept wall |
|---|---|---|
| `Volume` | `(1/3)·s·z·A`, `A` the region area (evaluator §4) | `(1/3)·2·h·g` |
| first moment | `½·s·σ·z²·A` along axis `k` | `σ_u·h·mu` and `σ_v·h·mv` along the `u` and `v` axes |
| `Area` | `A` | `L·h` (the prism wall's own reading) |
| `Bounds` | the face's own extremes (prism-extent's per-segment analysis over the face's prism view) | |

`Centroid` is each moment over the volume, lifted through the reference frame
and the placement as a prism's is. The volume and moment sums run in exact
rational intervals: a line's terms and a line region's area are exact
rationals, a circular term is the held float widened by its proven bound, and
each published value is the sum rounded once. The walls need no junction
chord (evaluator §4): every wall segment is a natural-range line or an arc
pinned between vertices the record shares bit for bit, and a whole arc's
terms read its recorded `End`, so consecutive walls meet exactly. A planar
face's area is evaluator §4's region integral, junction charge included.

Each face's `delta`, `z0Delta`, `z1Delta` enter as prism-boolean §7 enters
them for a prism. A planar face's area carries `2·δ·p + n·π·δ²`; a wall's
length carries the length displacement, and its height both level
displacements. The volume the denoted body can differ by is each wall's band
`2·δ·L + π·δ²` over its height plus both level displacements, and each planar
face's level displacement times its area; that volume times the coordinate
envelope bounds each moment. `Exactness` is `exactnessOf` on the composed
bound: `Exact` for an all-planar result with every coordinate recorded and
every level a float. The divergence-theorem sums are signed and may cancel in
VALUE, never in BOUND: every term's enclosure width is carried, so a
cancelling pair of faces widens the bound rather than narrowing it.

#### 4.4 Tessellation and the occupied-volume proof

`internal/brepbandmesh.Tessellate` builds the mesh over the body's topology
nodes, resolving each record face to the face whose origins carry its role.
A planar face chords its loops and triangulates them through the cap path
(`internal/triangulation`, tessellation §5), sharing every arc's chord samples with
the swept wall that owns the arc (tessellation §3). A swept wall is the
prism wall path over its own segment and interval: each interior sample
holds a vertex at every split level of either side line, and each side
line's own column a vertex at its own splits alone, so each side piece's two
vertices are the wall's own and the face across a side line meets no vertex
the other side's splits put there. The strip between two columns climbs
whichever column's next vertex is lower. The mesh closes by
construction because every edge is shared by exactly two faces (§4.2) and
both chord it from one sample set; `internal/tessellation.RequireClosedMesh`
proves it. The largest `delta` is reserved from the chord budget, as a
prism's is. Each planar face proves its loops' clearance, and every two walls
sweeping along one reference axis over overlapping intervals, sharing no
endpoint, prove theirs, so no two chorded walls cross where the analytic ones
do not. `Bound` per face is the wall's sagitta plus its level displacement,
or the planar patch's bounding-loop sagitta plus its level displacement,
composed with `delta`. Occupied volume: the analytic body and the chorded
body differ only inside the circular-segment slivers of each circular wall,
swept over its height — `Σ_walls h · E_wall`, tessellation §5's prism term
applied per wall — plus each wall's section band over its height where
`delta > 0`, each planar face's level displacement times its area, and
`sweptVolumeAllow` over the store maximum. Planar faces chord nothing of their
own. A mesh vertex's store is the largest bound any use placing it states,
as §4.2's body vertex is: a swept wall's two end samples charge the distance
from the held walk end to the point the wall's record denotes there
(`boundarywalk.DenotedStartBound` and `DenotedEndBound`), which adds an arc's
radial residual at its natural `t = 1` end; a line edge no swept wall chords
charges its use's `StartBound` and `EndBound`. The mesh therefore carries `volSymDiff`, and a class-B result is an
ordinary mesh-path operand and export input.

#### 4.5 Consumers

| Consumer | Behaviour |
|---|---|
| `Union`/`Cut`/`Intersect` with a prism, stacked or brep partner | class B again over the face view (§5), so a cross-drilled plate takes a second cross hole and a coplanar blind cut alike; a co-directional `Union` on an A1 result reads its `stack` and is A1 again (§3); a pair outside both takes the mesh path over §4.4's mesh |
| `Transform` / `Copy` / `Mirrored` | re-lifts every face frame under the composed motion; a reflection flips `outward` and every wall's winding, exactly as `prismPayload.Reflected()` does |
| `ScaledAt` | An A1 stack-backed, line-only solid with zero section and level displacement becomes a certified faceted image; source roles and planar tags follow the scale (evaluator §8). |
| `Fillet` / `Chamfer` / `Shell` | `agent-docs/modify-design.md` Part B: a brep that reads as a prism along a reference axis takes the prism's own ops (route P); an axis-parallel straight edge takes the brep rewrite (route E) and the result is a `brepPayload`; every other call refuses by that document's Table SB |
| `ThroughAll` / `ToFace` stops | a planar face's level is its frame and `z0`, and the face carries its cap record (`topo.CapRecord`: that frame, the placement and `z0`); a directional extent reads the per-face extremes (§4.3), and refuses a record carrying a `delta` as a prism's does |
| `Verify` validity | by construction (§4.2); the structural audit runs |
| `Verify` tolerance gate | `brepGateDiameter` reads every body vertex and each face's stations (a swept face's wall at both levels, a planar face's region boundary), shrunk by the largest `delta` plus axial term plus the widest vertex bound or station gap (verification §3) |
| `Verify` wall survey | staged `Suspect` (`DiagUnsupportedSurveyPayload`): the 2D spanning-disk reduction has no single section to read |
| `Verify` undercut, minimum radius | per face over its own placed frame: a planar face's one outward normal and a swept face's wall-walk normal range, decided exactly as a prism's cap and side are (DX7's reading); the tightest radius over the swept faces whose wall is a circle or arc walked clockwise with the material on its left, each under its own proven radius bound (DX8's), undecided when any face carries a section displacement |
| Clearance kernel | `clearance.BuildBrepBodyCarriers` (`internal/clearance/body_carriers.go`, the brep arm of `pairkernel`'s carrier model) adds each face's plane or wall carrier with its trims through the prism builders `clearance.PrismPlaneCarrier` and `clearance.PrismWallCarrier`, each face over its own frame; no model when any face carries a section displacement or the record holds loop bands or a boss or pocket shell band; the model's displacement is the largest per-face frame and placement rounding and tilt plus the record's largest level displacement; undecidable cells stay `Suspect` |
| Interference | `analyticBodiesEqual` undecided; the read-only mesh intersection over §4.4 |
| Tessellate, STL/OBJ/3MF | §4.4 |
| STEP | the analytic writer where every edge is a `Line3`, an `Arc3` or a full `Circle3`, and every cylindrical wall is full (two one-circle loops) or partial (one loop of at least four edges, each an arc about its axis or a line along it, with both kinds present) (`agent-docs/step-export-design.md`): a cross-drilled box, a keyway (arc rims) and a wall with a split side line all meet it |
| Prism-boolean's class | a brep operand misses G1 and never enters class A; class B's face view covers the co-directional pair of planar faces (§5.2) |

### 5. The 2D answers, and the 3D computations

Every face of the result is decided by one of three private `sketch` scenes.
decad builds the scene from its own records, asks `sketch` to arrange it,
and selects cells through `prismcells.Classify`'s flag comparison
(prism-boolean §4.2) — it computes no crossing, no containment and no cut
parameter. The three shapes, for operand X against operand Y with `N_X ⟂
N_Y`:

| Scene | Built in | Holds | Answers |
|---|---|---|---|
| **Section scene** of X | X's sketch plane | X's loops, plus one line per face of Y that contains `N_X` (Y's caps and Y's walls parallel to `N_X`), traced in X's plane | which fragments of X's own walls survive (their `TStart`/`TEnd`), and the cells of X's section each of Y's columns passes through |
| **Perpendicular-face scene** of a planar face F of X with `n_F ∥ N_Y` | F's plane | F's loops, plus Y's whole section re-expressed into F's frame (a 2D rigid map, the identity or a signed permutation under B4) | F's new loops: prism-boolean's clean-nesting match or crossing sub-case, verbatim |
| **Parallel-face scene** of a planar face F of X with `n_F ⟂ N_Y` | F's plane | F's loops, plus one rectangle per chord of Y's section along F's trace line, `chord × [y0, y1]` along `N_Y` | F's new loops, by the same cell selection |

The chords of the parallel-face scene come from one private scene per trace:
the other operand's section and F's trace line, run past its section box.
The fragments of the line that bound a returned cell are the chords, with
`TExact` certified cuts; a trace with more than one chord misses. A face of X
keeps what lies outside Y (`Cut`'s selection), a face of Y what lies inside X
(`Intersect`'s), through prism-boolean's clean-nesting match or crossing
sub-case; a crossing result may close into several outer loops, each one
face.

Every junction a scene returns is replaced by one canonical vertex, keyed
by the three carriers that meet there. Three planes meet at their exact
levels. A cylinder and two planes meet at the keyed crossing of the cylinder
with the plane along its axis (§10), at the other plane's exact level. Each
segment is then rewritten between its two vertices — a line whole, a
circular fragment as an arc pinned there about its recorded centre — and
every face edge is split at every recorded vertex on it: a line at each
vertex lying on both its face and its carrier, an arc at the angle of each
vertex on its cylinder, read at the face's level. Two vertices of one
cylinder closer in angle than the split tolerance of 1e-9 rad (`tol` in `internal/classbgeom/split.go`) are not
ordered and miss.

The scene's own point at a junction is a line's end wherever a line meets
there, since the line keeps its fixed coordinate exact. Its displacement
starts from that end's bound against the point its record denotes
(`boundarywalk.DenotedEndBound` or `DenotedStartBound`). Consecutive fragments
of one cylinder are joined first, so a junction with no line names two
distinct cylinders and misses.

A keyed vertex's displacement is at least its own proven distance from the
crossing it names (`classbgeom.CrossingOffsetUpper`). The vertex lies on both
planes at their exact levels, so in the plane across the cylinder's axis it
sits on the along plane's trace at a free coordinate `f`, and the crossing on
its side of the centre `c` sits at `c ± √b` with
`b = R² − (level − c_along)²`. The distance is `|a − √b| = |a² − b|/(a + √b)`
with `a = |f − c|`, exact rationals over the recorded floats and `√b` rounded
down. decad states this 2D quantity itself because no `sketch` answer bounds
it: the scene placed the vertex through a recorded cut parameter, and `δ_cut`
charges that parameter `proofbound.CutParamUlps` (8) ulps of `[0, 1]` times the trace's speed, while far
from the plane origin the parameter is off by the coordinates' own rounding,
about `ulp(|f|)` over the trace length. A box cut 100 m from the origin by a
drill breaking out of its top places the vertex farther from the crossing than `δ_cut`
(`apitest/classb_junction_test.go`, `TestClassBCrossingVertexFarFromOrigin`), and every face pinned there
inherits that distance. The bound is a charge, never an admission: it widens the faces
that use the key and decides nothing.

The two other places that pin a walked point at a crossing charge the same
distance through `brepgeom.CrossingOffsetUpper`, which reads both segments'
carriers exactly: a line through its recorded `Start` and `End`, a circle's
recorded radius, an arc's squared `Start` radius. A1's keyed table (§3 "A1
as a brep") charges it to the key's displacement wherever either side of the
junction is a cut parameter; a recorded corner, both sides at their natural
ends, is the record's own point and keeps displacement zero. The face view's
join (`brepgeom.JoinLoop`, §4.1) charges it to the payload's section
displacement at every junction it moves. Two lines
cross at one exact rational point. A line `n·x = c` crosses a circle at
`a = ±√b` along it from the centre's foot, and the point is
`|n·p − c|/|n| + |a² − b|/(a + √b)` from the crossing on its side, with every
square root rounded the way that enlarges the bound. Two circles cross on
their radical line, `2(C₂ − C₁)·x = R₁² − R₂² + |C₂|² − |C₁|²`, which is
exact, so they take the line arm. A point whose foot lies within twice that
bound of the centre's foot names no side and is charged the far crossing too.
A pair with no crossing — parallel lines, concentric circles, a line missing
its circle — answers +Inf: the keyed table misses, and the join's
displacement is infinite, which the face view's consumers refuse. A vertex
exactly at its crossing is charged zero, so an exact record stays `Exact`.
Without this charge a Ø10 boss crossing a 40 mm plate's wall 1e6 mm from the
origin reads its A1 brep's area off the exact value by more than its bound
(`apitest/stacked_union_brep_far_test.go`, `TestStackedUnionCrossingBossFarFromOrigin`).

A circular wall's pieces are read off the planar faces across its axis: X's
caps and Y's walls along e for a cylinder of X, Y's caps and X's walls across
d for a cylinder of Y. Each arc those faces carry on the cylinder is a
fragment; the levels of the faces carrying one fragment, sorted along the
axis, pair up into the ends of its pieces, since each piece of a closed
surface ends on exactly one face at each end. An odd count misses. A piece of
X keeps X's walk; a piece of Y runs reversed, since Y's material leaves the
result.

The 3D computations decad performs, each with its bound:

| Computation | How | Bound | Reject-only check |
|---|---|---|---|
| Perpendicularity, B3 | `==` on the dot product of stored world normals | a decision | misses take the mesh path |
| Trace of a plane of Y in X's plane | two points of the plane through `Frame.ToLocal` into X's frame; under B4 every coordinate is a sum of signed stored floats, held exactly, and the trace is the line through them | zero under B4 (`rationalFloatError` of each coordinate is computed and must be `0`; a nonzero one refuses the pair) | — |
| Level of a perpendicular face of Y along `N_X` | the exact rational `p · N_X` of a recorded point `p` of that face (`big.Rat` over stored floats); the held float is rounded once | `rationalFloatError` into that patch's `z0Delta`/`z1Delta`, prism-boolean G5's own mechanism | interval comparisons are exact over `big.Rat` |
| Chord endpoints, surviving fragments | `sketch`'s certified cut parameters | `δ_cut` (prism-boolean §7's `proofbound.CutDisplacementAllow`) plus the walk's rounding | `TExact == false` refuses (`ErrUnrecordableProfile`); the seam's range falsifier rejects a disproven flag |
| Canonical vertex | three planes: their exact levels; a cylinder: the keyed table's one float (§10) | zero for three planes; the larger of the key's `δ_cut` plus walk rounding and the float's proven distance from the crossing (`classbgeom.CrossingOffsetUpper`), charged to every face that uses it | a crossing whose free coordinate lies within twice its displacement of the cylinder's centre is too close to tangency to key and misses |
| Re-expression of Y's section into F's frame | one rigid 2D map per coordinate | zero under B4; `δ_reexpress` otherwise (§8) | — |
| Curved × curved separation, B7 | outward-rounded per-wall boxes compared exactly | a decision | refuses any overlap, including false overlaps |
| Closure of each face loop | the seam's junction falsifier on every assembled loop | — | rejects a contradicted junction (RB9) |
| Simplicity, orientation, nesting per face | modify §5's audit per planar face record | verification §4's floor | S7/S8/S9 refuse; none admits |
| Edge pairing of the result | exact record identity of the canonical vertices (§4.2) | — | an unpaired edge refuses |

No residual against a curve admits anything anywhere in this design; the
only residual-shaped quantities — the contact floor and the chord bounds —
refuse or are charged.

### 6. Interaction with mirror and pattern

- A prism-group tool (mirror §6.3) along a perpendicular sweep is class B
  with `Count` operand-Y loops in every scene: a row of cross holes through
  a bar is one section scene per operand and one perpendicular-face scene per
  wall, with `Count` holes each. The efficient N-hole case holds in both
  directions.
- A reflected operand in class B re-winds its record as A4 does before the
  face view is built; B4's permutation test reads `ApplyDir`, which a
  reflection maps correctly.
- A mirror join's result (mirror §5) is a prism and enters either class
  unchanged.

### 7. Upstream dependency

Class A3 reads `sketch`'s resolution of coincident line carriers
(`sketch` `docs/coincident-carrier-resolution-design.md`, "Coincident line carriers"),
which resolves a line pair only where both lines close loops (its
shared-wall gate) and they are the same line at round-off (its identity
gate). A pair outside either gate stays an invalid region, and the boolean
refuses it as RB1 or takes the mesh path, as before.

### 8. Do not do this

- **Re-sweep a box as a prism along the tool's axis** so a cross hole becomes
  a clean-nesting cut. It works once and fails to compose: the result is a
  prism along y whose next cut along z finds no prism at all, and the face
  roles change under the caller.
- **Loosen B3/B4 to a tolerance.** A near-perpendicular pair's traces are
  then rounded lines, and a cut on a rounded line amplifies by `1/sin θ` with
  no charge; Class B has no such charge (A6's charge covers only the prism
  pair), so the pair takes the mesh path.
- **Derive a cylinder's θ-trim from `acos`.** The trim is the arc fragment
  `sketch` certifies in the section scene; no angle is ever computed.
- **Admit cylinder × cylinder at "small radius" or "far apart" by a
  distance.** B7 is an exact box separation or nothing.
- **Make `brepPayload` the stored form of every prism.** The face view is
  derived on demand; prisms keep `prismPayload` and every consumer that
  dispatches on it.
- **Read the per-face `Faceted` bound as a per-facet bound to extend chains.**
  The S11 refusal is correct for what the face publishes; the fix is an
  analytic result, not a looser gate.
- **Fall back to the mesh path after a scene has been built and refused**
  (prism-boolean §3.4's point of no return): a refusal past the gate is an
  error the caller branches on. A topology the scene leaves unresolved is not
  a refusal: prism-boolean §4.4 sends it to the mesh path, and A1's crossing
  interface is one. Nor is a pair whose cuts carry an input displacement
  (A6): it builds with its crossing charge or takes the mesh path.

### 9. Test plan

Every test asserts computed geometry with a closed-form expectation; bounds
are relations, never literals.

- **A1 boss on plate**: 40×40×10 plate on XY `Union` Ø10 boss on
  `CreateOffsetPlane(XY, 10)` extruded 15: no `Faceted` face, two slabs, one
  interface with one exposed floor record (plate outer, boss hole), volume
  within its bound of `16000 + π·25·15`, bound below 1e-9 mm³, 8 faces,
  `CapStart` one face, `CapEnd` one face. The rooted boss (offset 5,
  extruded 20) gives three slabs and the same volume. Two equal boxes stacked
  edge-to-edge give `Exact` volume and I5's equal-outer column of one wall
  per side.
- **A1 as a brep**: a 10 mm square boss standing flush in the plate's
  corner: the interface match reports a `Partial` edge, the scene's cells
  resolve through the shared-span reading, the plate-only cell is the
  floor's 1500 mm², and the union is a brep of 9 faces, 21 edges, `Exact`
  17500 mm³ and 5400 mm², whose walls x = 20 and y = −20 are each one
  L-shaped planar face of 550 mm², with a proven 5 mm clearance to a box
  off the shared wall and STEP's analytic arm (9 planes). The same boss
  flush along one edge: 10 faces, the wall x = 20 one T-shaped face,
  the same exact volume and area. The Ø10 boss standing on the plate's
  top with its centre 2 mm inside x = 20: 10 faces (two cylinder pieces, a
  floor, a ceiling, the top cap in two arcs), 20 edges, volume within its
  bound of `16000 + 375π`, area within its bound of
  `4800 + 150π + 2·(25·acos(0.4) − 2·√21)`, bound below 1e-9, mesh
  volume verified, `Sound`, STEP analytic with two partial cylinders; a
  further `Union` on the result takes the mesh path. The flush corner boss
  rooted at z = 5: 9 faces, the shared corner's vertical edge one 25 mm
  edge. The Ø10 boss rooted at z = 5 with its centre 2 mm inside x = 20:
  10 faces and 22 edges, the overhanging cylinder piece one face from z = 5 to 25 whose
  side lines split at z = 10, the plate's x = 20 wall notched under the
  overhang, volume within its bound of `16000 + 375π + 5·S`, area within
  its bound of `4800 + 150π + 100·acos(0.4) − 14·√21`, mesh volume
  verified, STEP analytic with two partial cylinders. A hand-built record
  whose side split names no vertex of a neighbour, or lies outside its
  interval, is `ErrUnsupported`/`ErrDegenerate`. Misses: an operand
  carrying a section displacement, and a boss drawn on a plane offset in
  the plate's plane.
- **A1 interface pinch**: a bridge over z 25..30 resting on a tower over a
  60 × 40 × 10 plate keeps the mesh path or refuses where it also touches the
  tower's top at a point: a U's bar corner on the tower's corner in either
  operand order, a peak's apex on the tower's edge from outside, a notch's
  apex on a wider tower's edge, or a bar's edge tangent to a round boss's top
  circle. Each twin moved 1 mm clear builds the brep, `Exact` and `Sound`
  for the straight bridges (`apitest/stacked_union_self_pinch_test.go`).
  A lower L and an upper L whose missing quadrants meet opposite each other
  at one point keep the mesh path; so do the same Ls each joined first by a
  wedge pointing at that point in its operand's other slab, in either order,
  where only the count of records with edges there reads the pinch.
- **A1 separate bosses**: a 100 × 60 × 10 plate with radius-6, 8 mm round
  bosses standing apart (`apitest/stacked_union_separate_bosses_test.go`):
  `PatternCopies` and `Union` of three at 30 mm, `Union` of two, three in
  either order or with a boss first, four, one under the plate, and one overhanging the
  edge each build a brep with no shift, volume within its bound of
  `60000 + n·288π`, one radius-6 wall about each boss's axis and `Sound`; a
  radius-1 fillet of the three bosses' six circles gives six tori and
  `60000 + 3·(288π + 4π·A·d)`, `A = 1 − π/4`, `d = (10 − 3π)/(3·(4 − π))`.
  Two bosses 8 mm apart merge into one stacked region; bosses 12 mm apart,
  and a boss tangent inside the plate's edge, take the mesh path with a
  shift. Internal reads show `ProveGroupApart` refusing the tangent slab
  loops and `RingContact` refusing the tangent floor.
- **A1 brep operand**: the flush corner boss result unioned with a second
  10 mm boss flush in the opposite corner, in either operand order: a brep
  of 12 faces, `Exact` 19000 mm³ and 6000 mm², every flush wall one L-shaped
  face of 550 mm², two top caps, one floor of 1400 mm², a stack of two slabs
  whose upper slab holds both bosses; then a third boss on the plate's edge,
  `Exact` 20500 mm³, whose interface arranges four records. A second boss
  overlapping the first in the upper slab merges into one region. A scene
  test reads `ClassifyRegions` over a plate and two flush bosses and finds
  the plate-only area 1400 and each boss's cell inside its own record only.
  Blocks of unequal heights standing apart on a 60 × 40 × 10 plate, two or
  three, in either order, build an `Exact` brep of the summed volume, and the
  lower block's root loop fillets on it; a bridge across two block tops
  builds exactly where it stands clear, and keeps the mesh path or refuses
  where its corner meets a block's top corner or its edge is tangent to a
  boss's top circle (`apitest/stacked_union_unequal_blocks_test.go`).
  Misses: a class-B brep operand (no stack), and a crossing round boss
  result as an operand (its stack carries a displacement).
- **A4 reflected operand**: the L of mirror §2 mirrored across x = 30 and
  cut by a same-plane Ø3 cylinder inside its leg (M5) builds analytically,
  `Approximate`, volume within its bound of `1750 − π·1.5²·10`; the same
  image cut by a quarter-disc tool builds with the arc re-wound, volume
  within its bound of `4000 − 16π·10` for a mirrored 20×20 box; a box
  holding the mirrored L unions to the box's own volume through the
  select-all merge; T1's both-reflected cut builds through the shared-axis
  arm with `sectionDelta` exactly `0.0`; `Verify` on the mirrored L beside
  the cylinder reports one `Interference` row of `π·1.5²·10`. A scene test
  reads `prismcells.Classify` on a reflected box crossing a box and finds
  the exact areas 75, 25 and 75 for A-only, both and B-only.
- **A3 shared walls**: W1–W4 union into one analytic prism of 2000, 1600,
  3000 and 1900 mm³, and `Cut` by the second box leaves 1000, 1000, 1000
  and 900 mm³ (W6). M3 (the L unioned with its image across x = 15, sharing
  the walls y = 0 and y = 5) builds 3000 mm³ with a positive section
  displacement, and T2 measures one `Interference` row of 500 mm³. A box
  beside its own mirror image across x = 10 takes the mesh path for
  `Union` (a displaced interior span) and builds the box for `Cut`. A wall
  leaning inside the identity band charges at least its exact
  distance into `Cut`'s section displacement. A tooth whose root arc lies on
  a hub's circle: `Cut` of the tooth by the hub leaves the tooth,
  `Intersect` builds no analytic body, `Verify` reports no interference, and
  `Union` stays analytic. A scene test reads `Classify` and `ClassifySplit`
  on a region outside a disk bounded by an arc on the disk's circle, and
  finds it outside the disk. A `Trim` tool sharing the sheet's wall refuses
  with RS5. `Patterned` pegs that overlap or share a wall join into 625 and
  500 mm³. S12c (two boxes meeting at one vertical edge) shares no span: its
  `Union` is two lumps along an edge, which no prism payload holds, and it
  keeps the mesh path's coplanar refusal.
- **A5 N-hole cut**: mirror §8's N-hole test, and the disjoint `Union` of
  S12 reporting two lumps with `Exact` 1000 mm³ and no `Faceted` face.
- **A5 and A1 touching loops**: a circle tangent to a plate's side, two
  tangent circles, a triangle's apex on a square's side (two bodies, and
  three through the one-scene `Union`), three line-only bodies two of
  which meet at a shared corner, a circle tangent to a group's
  plate, and a boss tangent inside a square or round plate's edge on its
  top each take the mesh path, mesh under `VerifyAll`, and carry a volume
  bound covering the closed form with its π terms. A circle tangent inside
  the plate at one level stays `Exact`, and loops with a gap stay a group
  (`apitest/union_tangent_contact_test.go`).
- **A6 crossing charge**: a Ø40 hub unioned with a 7×3 tooth rooted inside
  it and placed by `RotationAround` through 60° builds analytically within
  its bound of the closed form; P1's own tooth, its root arc on the hub
  circle, falls back to the mesh path, as does the same tooth cut from the
  hub by a taller tool; S9 (a box intersected
  with the same box rotated 45°) builds an octagonal prism whose bound
  contains the exact rational residual against the two recorded squares;
  the charge covers `(δ + δ)/sin θ` when both operands bring `δ`, the floor
  leaves a junction whose sine bound is not above `ε` uncharged, an outside
  touch is arranged with no cut, and an amplified pair's RB9 (fu141's
  overshooting quadrilateral crossed by a box) falls back while the same
  quadrilateral's uncut merge keeps refusing; the
  fixtures for a nonidentity shallow crossing, a displaced
  chain, a walk charge and a placed `Verify` pair build or measure with a
  section displacement covering the input displacement over the crossing
  sine.
- **B cross-drill**: S1 builds a `brepPayload` with 7 faces (6 planar, 1
  circular wall), volume within its bound of `16000 − 9π·20`, bound below
  1e-9 mm³, `Lumps` one, every edge on two faces, the two walls at y = 0 and
  y = 20 each carrying one circular hole loop, `Cylindrical()` one face,
  `Verify` `Sound` with no survey asked, undercut survey measured, STEP
  analytic path (every edge a line or full circle).
- **B slot**: B1 builds with `Exact` 14000 mm³ and 10 planar faces.
- **B keyway**: B2 builds through the crossing reach with 6 faces (two
  notched caps, the key's floor and two walls, one cylinder piece), volume
  within its bound of `π·100·40 − 40·(2·√96 + 100·asin(0.2))` (the rod's
  strip |x| ≤ 2 above its axis, over its 40 mm length), the mesh's
  occupied-volume proof covering the same closed form, and a volume bound
  covering each keyed wall's band `2·δ·L·h`.
- **B turned shaft**: a full Revolve of the rectangle x in [0, 40],
  y in [0, 10] about x, `Cut` by a box from z = 7 up drawn on an offset XY
  plane and spanning x in [−5, 45], builds a D-flat with one cylinder piece
  and three planes, volume within its bound of
  `40·(100π − (100·acos(0.7) − 7·√51))`; with the box narrowed to |y| ≤ 3 it
  builds a keyway with five planes, `40·(100π − (3·√91 + 100·asin(0.3) − 42))`.
  Each verifies its mesh at `VerifyAll` and matches its Extrude twin's value
  and bound bit for bit; a shaft about the sketch's V axis builds the same
  D-flat, and an Acme thread of pitch 8 on the D-flat's wall builds to its
  twin's volume. A tool ending inside the span (drawn either way along the
  axis), a tool wall in the end plane (missed by B8), a tilted tool, a
  `Transform`-placed tool and a tube each keep the mesh path. `Fillet` of the
  class-B flat's and keyway's long edges builds with no section displacement,
  and `Chamfer` of them builds with the crossings' displacement carried as
  `δ_out` (brep-modify §4.4, `apitest/revolve_shaft_class_b_blend_test.go`).
  `internal/revolveplan/cylinder_view_test.go` misses each view condition
  on its own.
- **B stepped shaft**: full Revolves of staircases of 20 mm steps about x,
  `Cut` by the z = 7 box of the turned shaft, build with no `Faceted` face,
  verify their mesh at `VerifyAll`, and hold their volume within its bound
  of the sum over steps of `20·(π·r² − A)`, where `A` is the step's cut
  area: `r²·acos(7/r) − 7·√(r² − 49)` for a flat, and
  `3·√(r² − 9) + r²·asin(3/r) − 42` for the |y| ≤ 3 keyway. The radii
  10/8 (flat and keyway across both steps), 10/6 (the narrow step whole, its
  circle a hole of the shoulder), 6/10 (a shoulder facing −x) and 8/10/9 each
  build (`apitest/revolve_stepped_shaft_class_b_test.go`). A flat meeting the
  10 and 9 steps of 10/6/9 but not the step between, a 1 mm slot splitting
  the wide step of 10/6, a slot y in [4, 6] from z = 5 beside the 6 step of
  10/6 (apart from that step but inside its shoulder hole's box, its
  faceted volume holding `20·(136π − A)` with `A` the r = 10 disc's area in
  the slot), a flat tangent to the 8 step of 10/8, a keyseat
  ending inside the span, a tool wall in the end plane and a
  `Transform`-placed tool keep the mesh path.
  `internal/classbgeom/stepped_cut_test.go` misses each envelope condition
  on its own, and `internal/revolveplan/cylinder_view_test.go` reads two- and
  three-step meridians and misses rims on both sides of the axis, an
  overhanging rim, equal neighbouring rims, slanted risers, an extra riser
  and a spike along the axis.
- **B coaxial stepped bore**: a five-step Revolve cut by a full-length D tool,
  plus two-step round, rectangular, translated and antiparallel D bores, builds
  without a Faceted face, has the expected removed volume, and passes
  `VerifyAll` (`apitest/revolve_stepped_shaft_bore_test.go`). A blind tool,
  a tool whose box leaves the narrowest circle, a displaced tool, a holed tool
  and a non-permutation section frame miss the exact route
  (`internal/classbgeom/stepped_bore_test.go`).
- **B blind cross hole**: a drill ending at y = 11 inside the box builds with
  8 faces and volume within its bound of `16000 − 9π·11`.
- **B tool crossing a wall**: a 10×10 slot ending inside the box and crossing
  its x = 40 wall removes `5·10·10.3` mm³ within its bound.
- **B exact offset**: a drill on a frame whose origin is shifted in plane
  builds when the shifted centre is a float (0.5 + 19.5) and takes the mesh
  path when it rounds (0.1 + 19.9).
- **B chain**: S11's three cross holes build analytically, 9 faces with two
  y walls of three holes each, `12000 − 240π`; a hole down the result's caps
  builds, `12000 − 280π`; a pin rooted in the bar's end wall unions with the
  brep in either position; a fourth hole overlapping the first misses B7 and
  takes the mesh path, pinned.
- **B gate misses**: S10 (tilted plane), S7 (cylinder × cylinder), a
  non-perpendicular pair, a non-permutation frame pair, a slanted
  tool wall (B6) and a tool wall in the target's cap plane (B8) each take the
  mesh path; the non-perpendicular fixture asserts `N_A · N_B != 0` itself.
- **B rooted and through, every op**: a Ø6 pin rooted in a 40×20×10 plate's
  wall x = 40 from x = 30 to 50: `Union` 8 faces, volume within its bound of
  `8000 + 90π`; `Cut` a blind hole of 8 faces, `8000 − 90π`; `Intersect` a
  prism of `90π`. The same pin from x = −5 to 45: `Union` 10 faces,
  `8000 + 90π`, and `Intersect` `360π`, each with either operand first.
- **B flush cap**: a Ø6 pin standing on a 40×20×10 plate's wall x = 40
  from x = 40 to 55 unions, with either operand first, into 8 faces,
  `8000 + 135π`; the pin from x = 30 to 40 cuts a blind hole of 8 faces,
  `8000 − 90π`, and intersects to its own prism, `90π`; a boss standing on a
  cross-drilled brep's top unions into 9 faces, `8000 − 225π`; a notched
  plate whose far wall shares the pin's other cap plane still cuts its blind
  hole. Pins drawn `Against` the YZ normal (an inside `Cut` from x = 40, an
  outside `Union` from x = 55) and pins on the wall x = 0 (an outside `Union`
  from x = −15, an inside `Cut` to x = 10) build the same 8-face results,
  `8000 ± 9π·L`. Each is non-`Faceted`, pairs every edge, verifies its mesh at
  `VerifyAll` and is `Sound`. A `Union` of the inside pin, a `Cut` of the
  outside pin, the notched plate's `Union`, a pin one ulp off the wall and a
  pin whose inch length rounds onto the wall keep the mesh path; an
  `Intersect` of the outside pin is `BooleanEmpty`; a displaced face level
  and a positive caller charge miss (`apitest/classb_flush_pin_test.go`,
  `internal/classbgeom/flush_reach_test.go`).
- **B flush caps on two faces**: a Ø6 pin bridging a U-shaped 10 mm plate's
  gap from its wall x = 10 to x = 30, drawn from either wall and with either
  operand first, unions into 11 faces, `5000 + 180π`; a pin from a
  40×20×10 plate's wall x = 0 to x = 40, drawn from either wall, cuts a
  through hole of 7 faces, `8000 − 360π`, and intersects with either operand
  first to its own prism, `360π`. Each is non-`Faceted`, pairs every edge,
  verifies its mesh at `VerifyAll` and is `Sound`. The bridging pin one ulp
  short builds the standing pin of 12 faces. A pin with its section crossing
  the plate's top and a `Union` of the through pin keep the mesh path; the
  bridging pin's `Cut` and `Intersect` refuse there with
  `BooleanUnsupportedContact`.
  Two faces whose outward flags disagree, two faces at one cap's level, a
  displaced face level and a displaced tool cap miss
  (`apitest/classb_bridging_pin_test.go`,
  `internal/classbgeom/flush_reach_test.go`).
- **B pin across several faces**: on the U plate, a Ø6 pin from x = 10 to
  35, drawn from either end, unions with either operand first into 11 faces,
  `5000 + 180π`, cuts into 12 faces, `5000 − 45π`, and intersects with either
  operand first to the prism over `[30, 35]`, `45π`; the pin one ulp past the
  far wall x = 30 unions into 11 faces and cuts a blind hole one ulp deep.
  The pin from x = 10 to the far outer wall x = 40 unions into 11 faces,
  `5000 + 180π`, cuts into 11 faces, `5000 − 90π`, and intersects to `90π`.
  One ulp short of x = 40 it cuts a blind hole of 12 faces and unions into
  11; one ulp long it cuts 11 faces, intersects to `90π` and unions into 13
  faces with a one-ulp stub past the wall. On a slotted 6500 mm³ plate a pin
  from x = −5 to 45 unions into 15 faces, `6500 + 180π`, and cuts into 12,
  `6500 − 270π`; the pin from x = 0 to 40, both caps flush, unions into 11
  faces, `6500 + 90π`, and cuts into 12, `6500 − 270π`. Each is
  non-`Faceted`, pairs every edge, verifies its mesh at `VerifyAll` and is
  `Sound`. The slotted plate's `Intersect` (two prisms), a far cap displaced
  by an inch length, and a section crossing the U plate's top keep the mesh
  path. Unit tests miss W1–W4 over hand-written face records (a face along
  the sweep, one level twice, two faces of one sense, a displaced face or
  tool cap, a backwards interval) and W5 over two faces too small for the
  section, and build a boss from a brep plate's bottom z = 0 through its top
  to z = 15, which a displaced top keeps off
  (`apitest/classb_multi_face_pin_test.go`,
  `internal/classbgeom/walk_reach_test.go`).
- **B stacked target**: a cross hole under a blind pocket builds a brep of
  `4000 − 400 − 80π`.
- **B with a hole breaking out**: a through cross hole whose circle crosses
  the box's top face: the perpendicular-face scenes report `Partial` edges,
  the crossing sub-case builds, the top cap splits in two, volume within its
  bound of `16000 − 20·(9π − 9·acos(2/3) + 2·√5)` (the disc below the top
  face), with the same mesh and charge checks as the keyway.
- **brep face view of a crossing-built prism**: a box less a corner bite,
  built through prism-boolean's crossing sub-case, joins its fragments'
  junctions and builds a brep within its bound of `8000 − 62.5π`.
- **brep measurements**: a prism viewed as a brep reports volume, area,
  bounds and centroid value bit-identical to `prismPayload`'s for an
  all-line record whose readings are exact rationals rounded once, and
  within the composed bounds for a circular one. The brep's centroid bound is
  the exact error of that one rounding, so it is at or below the prism's.
- **brep consumers**: S1, both hand-built and as the public class B `Cut`
  builds it: its pull survey hooks only the hole's wall under a pull along
  +z, nothing along the hole, and the x = 0 face, the floor and the hole's
  wall under a pull along (1, 0, 1); its concave-radius survey measures
  3 mm, a half disc's convex wall is no candidate, and a displaced record is
  undecided; its clearance rows read 5 mm to a box off its x = 40 wall and
  2 mm to a pin threaded through its hole; `Fillet`, `Chamfer` and `Shell`
  refuse with SX16. export writes the class B S1 and B1 results, a half
  disc, a notched plate and an L whose cap loop starts at its reflex corner
  analytically, every edge used once in each sense.
- **Cancellation and the cap** as prism-boolean §15, per scene.

### 10. Open questions

- **A3's interior span under a displacement (decided: the mesh path).** A
  displaced pair whose shared span lies inside or outside the result could
  instead charge the sliver as an area, `width · length` over the result's
  perimeter, but the section displacement is a boundary distance that also
  feeds the centroid and the walls' areas, and a sliver away from every
  recorded edge moves neither by a distance. A mirrored half beside its
  source joins through the mirror join, which carries no displacement.
- **Should B4 (signed-permutation frames) be relaxed?**
  Decided: no. Class B has no charge for a rounded trace, and the
  datum planes cover the workload.
- **Should class B's roles mint `capStart`/`capEnd` when a result keeps an
  operand's caps whole?** Recommendation: no; prism-boolean §11's fresh-roles
  decision stands, and a cap kept whole is still a face of a new record.
- **Should the brep's planar-face scenes run the modify §5 audit per face
  (chosen) or once over a merged section?** Recommendation: per face; the
  faces lie in different planes and share no section.
- **Class C's ellipse.** Recommendation: file the `sketch` ask only when a
  consumer needs an oblique hole; the whole-scene `TExact` gate makes it a
  larger upstream change than a carrier rule.
- **Crossing vertices across scenes (decided: a keyed table).** Two scenes
  that cut one carrier pair record the crossing at different floats: a circle of radius 10 at the
  origin against the line u = 2 records the crossing at a float that differs
  in the last places on the line and on the circle, and moving the line's ends
  records a third float on the line. §4.2's
  pairing by record identity therefore cannot join faces that different
  scenes cut. Options: key each crossing by the two carriers that make it
  and the side of the circle's centre it lies on, record one float per key
  for every face that meets it, and charge each face the cut displacement;
  or pre-split every carrier at the section scenes' crossings so the face
  scenes cut nothing. Decision: the keyed table. A crossing of a cylinder
  with a plane along its axis is keyed by the cylinder, the plane, and the
  side of the cylinder's centre it lies on along the remaining axis; the
  first scene to reach a key records its float and displacement, every later
  one takes them, and every face that uses the key carries that
  displacement. The displacement covers the float's own proven distance from
  the exact crossing (§5), not only the cut parameter's allowance. A recorded
  corner where a section's arc meets its line is the key's float with
  displacement zero. A crossing too close to tangency to decide its side
  misses to the mesh path; reject-only, it admits nothing.
- **S8's sphere facet cap.** Recommendation: separate task; raising the cap
  or deriving the boolean's tolerance per operand is a mesh-path change
  outside this design.

### 11. PR split

Each item is implemented and tested. The labels (PR 1, 1b, 2 ...) are the names other documents cite.

1. **A1 stacked union.** Overlapping/touching `Union` intervals, slab
   splitting, per-slab select-all merge, interface exposure by the
   clean-nesting match, stacked §2.2's union reading of I5–I7, §9's A1 tests.
1b. **A1 as a brep.** The flush and crossing interfaces as a `brepPayload`
   over the same slabs: cached interface scenes, cell classification,
   canonical vertices, merged planar walls, swept pieces, §9's A1 brep
   tests. Builds on PRs 5 and 8.
1c. **A1 brep operand.** The result's `stack`, several records per slab and
   per interface, `ClassifyRegions` and the region-aware span reading, the
   brep build's own slab merges, §9's A1 brep operand tests.
1d. **Side-line splits.** `brepFace.side0`/`side1`, one edge per piece in
   the topology, the wall mesh's split rows, STEP's partial wall of any
   edge count, the rooted round crossing boss.
2. **A4 reflected re-expression.** G2 replaced by the re-wound record,
   `AuthoredReversed` on the re-wound record, the shared-axis both-reflected
   case, the interference twin, §9's A4 tests. Depends on nothing.
3. **A5 multi-region.** The prism-group tool in `Cut`'s clean-nesting match
   and crossing sub-case, the disjoint `Union` to a one-slab group, with
   mirror §11 PR 4.
4. **A6 crossing-sensitivity charge.** Certified `sin θ_low`, the amplified
   fragment charge in place of §3.4's reroute, the noise-floor refusal, the
   rotated-tooth and S9 fixtures.
5. **`brepPayload` foundation.** The record, the face view of prism and
   stacked payloads, topology and roles, measurements, tessellation with the
   occupied-volume proof, placement, `Verify` validity and the tolerance
   gate; the prism round-trip tests.
6. **Class B `Cut`.** B1–B8, the through-nesting reach (§3 B.3), result
   assembly, S1/B1 fixtures, the exact offset and the gate-miss fixtures.
6b. **Class B crossing reach.** §10's keyed crossing table, the per-face
   scenes, vertex canonicalization and edge splitting, the cylinder pieces
   (§5), the face view's junction join (§4.1); B2, the blind cross hole, the
   hole breaking out and the tool crossing a wall, each with a closed-form
   volume fixture.
7. **Class B `Union`/`Intersect` and chaining.** The rooted boss (B4), a
   brep operand in both positions, the breaking-out hole, S11's chain.
8. **brep consumers.** Undercut and minimum-radius surveys, the clearance
   `bodyGeom` arm, STEP's analytic arm with `Arc3` edges, modify-reach's RX
   row.
9. **A3 shared walls.** The `sketch` pin with coincident line carriers, the
   shared-span reading (`prismcells.CoincidentEdges`) in `Classify`,
   `ClassifySplit` and `CrossingCharge`, line-run splitting, the span's width
   charge and the displaced-interior-span fallback, `Trim`'s RS5 refusal,
   and §9's A3 tests. S12c stays on the mesh path (§9).

### 12. Straight-source axial Loft Union

`prismboolean.TryAxialLoftUnion` accepts a prism and an authenticated two-section
Loft whose source records denote a straight prism. The gate reads the Loft's
recorded geometry, not its source identity, segment count, or radial position.
Two independently created Lofts can therefore join the same prism in separate
pair operations. A many-body call can arrange their certified private prism
records in one scene when each Loft overlaps the first prism over positive
area, and all inputs share the same whole-line, hole-free plane and interval.
Otherwise the many-body call uses the transient pair chain.

The Loft-to-prism source certificate requires all of these facts:

1. The Loft has a solid authenticated reconstruction and a source identity.
   It has no hole, blend, bore rewrite, curved chord, or construction
   displacement.
2. Both profiles contain the same full-domain line records in the same order.
   Every pairing offset is zero. Equal line source curves under that pairing
   generate the same planar wall at both sections and throughout the ruled
   interval.
3. Both section frames have identical in-plane axes.
   `prismplacement.SharedAxisOf` proves over exact rationals that their origins
   differ only along the common normal. A nonpositive interval declines.
   The prism sweep level is the rounded rational shift; `Z1Delta` encloses
   precisely that rounding. The source profile and base frame are unchanged.

The route creates private prism operands only after the source certificate.
The existing prism `Intersect` scene must resolve a positive-area shared
region for a pair, or for each Loft against the first prism in a many-body
scene; an edge-only tangent declines. The existing prism `Union` scene then
authenticates the same profiles through Sketch's exact fragments, selects the
material cells, audits the merged section, and publishes the normal computed
section and axial bounds. A miss falls through to the existing Boolean path.
An arranger refusal or exhausted work budget propagates; neither becomes a
small-residual admission. The old Loft/cylinder route and its bounds are
unchanged.

This reduction does not admit moving sections, twisted pairings, free-form
walls, or a helical tooth Loft. Such contacts need a separate
source-wall intersection and held-shell certificate before this route can
replace the structural Loft/cylinder path.

## Part R — Boolean chain regroup

A two-body `Union`/`Cut`/`Intersect` that refuses, and that has a mesh Boolean result as an operand, is
re-evaluated from the source bodies the operation history names, under a set identity that leaves the result set
unchanged. Companion to `agent-docs/operation-history-design.md` ("history §N"), `agent-docs/evaluator-design.md`
§9, `agent-docs/faceted-vertex-bounds-design.md` §5 ("vertex-bounds §5"), Part B
("prism-boolean §N") and Part G ("general-boolean §N"). Core §N is
`agent-docs/api-design.md`.

Navigation only; the named sections own their rules.

| Question | Section |
|---|---|
| What does this buy, honestly? | Value assessment |
| When does it run? | Trigger |
| Which chains, which identities, what proof? | Chain normal form; Admitted identities |
| In what order are the steps tried? | Plan |
| Where does the code sit? | Execution in booleandispatch; Root wrapper |
| How are retired source bodies reached? | Source reachability and retention |
| What does the result and its history look like? | Result identity and history |
| What must the tests prove? | Test plan |
| What does an implementer do, in order? | Tasks |

### Problem

An operand of `Union`/`Cut`/`Intersect` that is a mesh Boolean result (`facetedrecord.Payload`) restates its held
mesh. The next pair's local gate refuses a contact on held facets coarser than the pair's chord tolerance
(vertex-bounds §5, `meshbool.RefuseCoarseHeldContact`), and the mesh path refuses coplanar or grazing contact against
the held facets. The operation history (`operation_history.go`: `Document.OperationHistory`, `Producer`,
`Operation.Inputs`) records which bodies produced the operand.

Success criteria:

- A two-body Boolean that refuses with an error wrapping `ErrUnsupported`, and whose chain behind a mesh operand
  can finish under Plan, returns a body whose readings enclose the true quantities of the same set the
  sequential chain denotes.
- Every call that succeeds today returns the same body as today (no regroup runs).
- Every call regroup cannot rescue leaves the document unchanged and returns today's error, except two cases:
  cancellation during regroup returns `ctx.Err()`, and an invariant break inside a private step returns that
  error (Failure behavior).
- The fixtures in Test plan pass with assertions on computed volumes and coordinates.

**Out of scope:** the chain regroup does not run for `Union` of three or more bodies (`performUnionMany`).
Its one regroup is the three-source prism pair retry `booleandispatch.RegroupUnionThree` (prism-boolean §4.7),
a separate route. The regroup also leaves out regrouping before a call that succeeds
today, walking through `Transform`/`Copy`/modify producers, general chains mixing `Union` with `Cut`/`Intersect`, a public
option or diagnostic for regrouping, and `Verify`'s read-only evaluator.

Prism-boolean §4.7 owns the separate three-source variadic prism retry. It
reads the call's original operands and does not expand operation history.

### Value assessment

Regrouping cannot make an unsupported pair exact: a pair no analytic route admits stays a mesh step in every order.
The plan combines source bodies analytically before trying the remaining full dispatch steps (Plan). A mesh
step returns a `Faceted` result. Two remaining cut tools may be united by full dispatch before the final cut;
both dispatches retain their ordinary geometry, bound and refusal checks.

No missing-features row closes. Effect per row or claim:

- "Held mesh operand coarser than the pair tolerance after a Boolean chain": narrowed, not closed. A chain builds
  when the bounded Plan finishes. Chains whose remaining mesh-only steps exceed that plan still refuse:
  crossing-cylinder chains (general-boolean §3 B.2), the mitred-sweep branch tree and cone chains of
  vertex-bounds §6, and held primitives remain subject to those limits.
- "Contact on the mesh path other than a coplanar overlap, ...": narrowed for chained calls only. A coplanar contact
  between two source bodies that an analytic route joins (general-boolean §3 A3) no longer meets the held mesh. A
  coplanar contact that exists in the final set still refuses.
- "Multi-body Union ... a mesh Boolean result joins no such chain": not touched. `performUnionMany` keeps its
  refusal (`boolean_many.go`, "Union of three or more bodies needs ...").
- general-boolean §1 item 3 and §2 S11 (three cross holes): not needed. general-boolean §3 B.3 places S11's chain
  in class B's analytic reach.
- Bound tightening of chains that succeed today: none, by the Trigger decision.

The concrete gain is a set of ordinary refused chains: two overlapping parallel holes placed off the body's frame
(fixture F1), a straight hole that breaks into an earlier translated hole (F2), and an analytic intersect after a
mesh cut (F4). F3, a box beside a plate that already carries a boss under `ExactOnly`, is not shown to be a
refused chain. `TestRegroupPlansBuildByHand` shows that the sequential join of a copy of the plate and boss with the
box succeeds by the grid route (volume 12240). `TestBooleanRegroupDisjunction` runs the call without the copy and
asserts the same volume, a mesh-path watertight result and the regrouped history shape, but it does not assert that
the sequential call refuses first, so it does not show that the regroup is what builds F3.

### Assumptions

- Decad's Booleans are regularized: every result is a closed regular set or a refusal (core §8 refuses grazing,
  edge and point contacts on the mesh path rather than emitting lower-dimensional pieces). The proofs below rely on
  the Boolean algebra of regular closed sets.
- A body built by a public `Union`/`Cut`/`Intersect` denotes that operation over its recorded consumed inputs
  exactly, unless it carries an accepted adjustment (Eligibility of an expandable body). Its published bounds relate
  its held mesh to that exact set (vertex-bounds §2).
- An analytic result is admitted whatever its exactness: an `Approximate` prism result is a sound step, because its
  readings carry their bounds. `Exact` accepts it.
- Analytic routes are deterministic in their operands: a pair that missed misses again with the same two bodies.
  Producer identity (`topo.Env.Ref`) does not decide admission.
- `MaxLiterals = 6` bounds the work on the refusal path. No measurement chose it.
- The roles of faces a private analytic step mints are acceptable as fresh roles under that step's private
  producer, as in the many-body prism chain (`boolean_many.go` `tryPrismUnionChain`), beside the source roles
  prism-boolean §11's attribution adds.

### Approach

Run today's two-body path first. When it fails with an error wrapping `ErrUnsupported` and an operand is an
expandable mesh Boolean result, root reads producer records, `internal/boolchain` flattens the chain into a normal
form of source-body literals, and `internal/booleandispatch` evaluates a plan: every pair an analytic route admits
first, then the bounded full dispatch steps in Plan. Any miss, failure or ineligibility returns today's error
unchanged.
The public commit records the caller's call exactly as today and reserves one producer identity per private step.

Alternatives considered:

- Regroup before dispatch on every call with a mesh operand: rejected. It changes results and roles of calls that
  succeed today and costs analytic attempts on every chained call; per-vertex bounds already keep untouched
  geometry from compounding (vertex-bounds §5), so the gain on succeeding calls is small.
- Regroup before dispatch only when the plan is all analytic: rejected. It needs a mesh-only pair to vanish under
  reordering, which the fixtures surveyed never showed, and it costs analytic attempts on every chained call.
- Walk through `Transform`/`Copy` (a rigid motion distributes over Boolean operations): rejected.
  It needs placed leaves built inside the plan; nothing in the fixtures requires it.
- A public opt-in or opt-out option, or an `OpDiag` warning naming the regroup: rejected. It adds public API under
  core §6 review for behavior that only turns refusals into proven results.
- General distributive rewrites for mixed chains remain outside `boolchain`. The narrow target-side coarse-held
  `Cut` exception is specified in Paired Cut continuation.

### Design

#### Trigger

The separate Paired Cut continuation checks its narrower typed trigger before the chain regroup below.

Regroup runs inside `performBoolean` (`boolean.go`) only when all of these hold:

- `booleanBodyWith` returned an error `err` with `errors.Is(err, ErrUnsupported)`. This covers
  `BooleanUnsupportedContact` (it wraps `ErrUnsupported`, `newBooleanError`) and the plain staging refusal.
- `err` is not a context error.
- The call carries no `WithMatchingFaces` (`booleanConfig.matchingFaces` false). The option is a statement about
  the caller's operands, not about the plan's.
- `boolchain.Flatten` expanded at least one operand (Chain normal form).

Errors that never trigger: `BooleanEmpty`, `BooleanEvaluatorFailure` (an invariant break MUST surface), input
gate errors (`ErrDegenerate`, `ErrRetiredBody`, `ErrForeignBody`), the retryable coarse-chording `ErrDegenerate`,
and cancellation. `performUnionMany`, `booleanBody` (Patterned's private chain), `evaluateBoolean` (Verify) and
composite tools under `privateConstructionContext` never regroup.

#### Eligibility of an expandable body

`boolchain` expands body `n` only when all hold; any failure makes `n` a leaf. `boolchain.Expandable` implements
these rules, and point location reads the same function (`agent-docs/point-containment-design.md` §8):

- `n.Payload` is a `facetedrecord.Payload` value.
- The `Source` reports a public producer of kind `Cut`, `Union` or `Intersect` with `n` as its output.
- `n.Payload.(facetedrecord.Payload).PointCone == nil`. A point-cone trim may record `MatchingFaces`
  (`internal/facetedrecord/point.go`), which history does not record.
- Every `n.OperationWarnings` record is classified as moving no geometry. The only such record is a
  `CodeChainAllowanceApplied` with `Change == opdiagnostic.ChangeUnknown`: it relaxes a gate and moves nothing. Any
  other record makes `n` a leaf, including a code added later. A gap shift or embedding reach moved accepted geometry
  (`internal/booleaneval/eval.go` `Diagnostics`, `internal/meshbool/held_embedding_diagnostics.go`), so `n` does not
  denote the operation over its inputs.

A body whose producer is `Transform`, `Copy`, a modify operation, `Hole`/`WebPath`/`Emboss`/`Boss`, `Patterned`
or any non-Boolean kind is a leaf. An analytic (non-faceted) Boolean result is a leaf: it is already exact.
`boolchain.Expandable` states this rule; `Body.Section` applies the same rule before it sections a mesh Boolean
result from its two operands.

#### Chain normal form

`boolchain.Flatten(op, a, b, src)` returns a `Chain` or `ok == false`.

| Call | Form | Literal list start |
|---|---|---|
| `Cut(a, b)` | conjunction | `+a`, `−b` |
| `Intersect(a, b)` | conjunction | `+a`, `+b` |
| `Union(a, b)` | disjunction | `a`, `b` |

Expansion rules, applied depth-first, left to right, in each producer's recorded input order:

| Literal | Expandable producer | Replacement |
|---|---|---|
| positive `+n` | `Cut(t, u)` | `+t`, `−u` |
| positive `+n` | `Intersect(x, y)` | `+x`, `+y` |
| negative `−n` | `Union(x1 … xk)` | `−x1 … −xk` |
| disjunction item `n` | `Union(x1 … xk)` | `x1 … xk` |
| any other pairing | any | none; `n` stays a leaf |

Rules:

- `Cut` inputs are read by role (`target` positive, `tool` negative). `Union` and `Intersect` inputs are read in
  recorded order. Only `InputConsumed` inputs count.
- More than `MaxLiterals` (6) literals → `ok == false`. Stop walking once the limit is passed.
- No expansion at all → `ok == false`.
- A conjunction's first literal is positive by construction.
- Leaves are pairwise distinct bodies: a body is consumed at most once (`requireLive` at every consuming call), so
  no body is the input of two recorded producers.

#### Admitted identities

Notation: `∧`, `∨`, `¬` are meet, join and complement in the Boolean algebra of regular closed subsets of R³.
Regularized `Cut(A, B) = A ∧ ¬B`, `Intersect(A, B) = A ∧ B`, `Union(A, B) = A ∨ B`. The regular closed sets form a
Boolean algebra under these operations, so associativity, commutativity and De Morgan's laws hold exactly.

| # | Identity | Proof |
|---|---|---|
| I1 | `(A − B) − C = (A − C) − B` | `A ∧ ¬B ∧ ¬C`: commutativity and associativity of `∧`. |
| I2 | `(A − B) − C = A − (B ∪ C)` | `¬B ∧ ¬C = ¬(B ∨ C)`: De Morgan. |
| I3 | `(A ∩ B) − C = (A − C) ∩ B` | `A ∧ B ∧ ¬C`: commutativity and associativity of `∧`. |
| I4 | `(A ∪ B) ∪ C = (A ∪ C) ∪ B = A ∪ (B ∪ C)` | Commutativity and associativity of `∨`. |
| I5 | `(A ∩ B) ∩ C = (A ∩ C) ∩ B` | Commutativity and associativity of `∧`. |

Consequences the Plan relies on:

- Every conjunction chain equals `(∧ positives) ∧ ¬(∨ negatives)`. Any evaluation that starts from a positive
  literal and applies each remaining literal once, by `Intersect` for a positive and `Cut` for a negative, after
  merging any subset of negatives by `Union`, yields that element.
- Every disjunction chain equals `∨ items`, in any grouping.
- No intermediate of a conjunction plan is empty when the final set is non-empty: each intermediate is a meet of a
  subset of the literals, so it contains the final set. A disjunction intermediate contains a non-empty leaf.
- `Union(A, B)` with `A`, `B` disjoint is a two-lump regular set; nothing above assumes connectivity.

The chain plan admits no mixed identity: `(A ∪ B) − C ≠ A ∪ (B − C)` in general, so a `Union` result inside a conjunction's
positive side, or a `Cut`/`Intersect` result inside a disjunction or a negative literal, stays a leaf.

#### Paired Cut continuation

- Trigger only on `Cut` without `WithMatchingFaces`, after a `BooleanExpectedStaging` refusal carrying a
  target-side `CoarseHeldContactError`. Reject conflicting tags and canceled contexts.
- Authenticate public history and its identity seals. The target must be `Union(L,U)` with exactly two consumed
  sources, or `Cut(Union(L,U),H1)` with those same sources and the recorded first tool. Require expandable faceted
  Union and prior Cut payloads, including their warning and point-cone gates. Recheck history, live operands and
  producer identity under the commit lock.
- The regularized identities are `(L ∪ U) − H1 = (L − H1) ∪ (U − H1)` and
  `((L ∪ U) − H1) − H2 = ((L − H1) − H2) ∪ ((U − H1) − H2)`.
- The first plan calls `Cut(L,H1)` and `Cut(U,H1)` at chord scale `0.25`, then unions the results at scale `1`.
  The second calls `Cut(L,H1)` at `0.25`, cuts that result by `H2` at `0.125`, calls `Cut(U,H1)` at `0.0625`,
  cuts that result by `H2` at `0.125`, then unions both results at scale `1`.
- Each step starts a fresh ordinary pipeline. No shared replay meter or per-step work, record or byte counter is
  used. Reject every private warning. Apply the ordinary source, contact, boundary and occupied-volume gates.
- Verify the detached result at the original pair chord raised to its held floor. A finer consumer tolerance is a
  separate check; the held-floor result does not establish the original chord globally.
- Reserve three or five consecutive producer identities. Publish one `Cut` with the original target and tool only
  after the plan, verification, history recheck and cancellation check succeed. A declined plan falls through to
  the existing retry and regroup rules. A cancellation or evaluator invariant failure returns without a commit.

#### Plan

`boolchain.Run(ctx, chain, steps)` evaluates the chain through two callbacks:

```go
type Step func(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error)

type Steps struct {
    Exact Step // analytic routes only; ok=false on any miss or failure
    Full  Step // the complete route order, mesh pipeline included; ok=false on any failure
}
```

Contract for both callbacks: a non-nil error means cancellation or a surfaced invariant break (Execution in
booleandispatch) and MUST be returned by `Run` unchanged; every other failure is `ok == false`.

Conjunction:

1. `acc` = the first positive literal. `pending` = the remaining literals in walk order.
2. Absorb: scan `pending` in order; for literal `ℓ` call `Exact(opOf(ℓ), acc, ℓ.Body)` with `opOf(+) = OpIntersect`,
   `opOf(−) = OpCut`. On success set `acc` to the result, remove `ℓ`, restart the scan. Stop after a full scan
   absorbs nothing.
3. If `len(pending) >= 2` and every pending literal is negative: merge their bodies (step 6), replace them with the
   merged bodies as negatives, then Absorb again.
4. `len(pending) == 0` → return `acc`. `len(pending) == 1` → return `Full(opOf(ℓ), acc, ℓ.Body)`.
5. Exactly two negative literals remain → `tools = Full(OpUnion, pending[0].Body, pending[1].Body)`, then
   `Full(OpCut, acc, tools)`. Stop on either miss or error. Otherwise → `ok == false`.
6. Merge (shared with disjunction): over `items` in order, find the first pair `i < j` with
   `Exact(OpUnion, items[i], items[j])` ok; replace `items[i]` with the result, delete `items[j]`, restart. Stop
   when no pair merges.

Disjunction: Merge all items. One item → return it. Two items → return `Full(OpUnion, items[0], items[1])`.
Otherwise → `ok == false`.

Rules:

- Keep a miss set keyed by `(op, x, y)`; never call `Exact` twice on one key. Bodies are immutable and routes are
  deterministic (Assumptions), so a cached miss stays valid.
- `Full` runs at most twice per `Run`, only for step 5's two negative literals. Every other plan uses at most one.
- Three or more remaining literals decline without any `Full` call.
- Two remaining literals that are not both negative decline without any `Full` call.
- Check `ctx.Err()` before every callback.
- With `MaxLiterals = 6`, the number of `Exact` calls is small: the miss cache admits each `(op, x, y)` key once, and
  the keys come from at most six literal bodies and the bodies Merge and Absorb produce.

#### Execution in booleandispatch

`internal/booleandispatch/dispatch.go` splits `Body` into `Analytic` and a mesh tail:

```go
// Analytic runs every route Body tries before the mesh pipeline, in Body's order.
// ok=false with a nil error is a miss. A non-nil error is the route's refusal, already
// mapped through hooks.AsBooleanError where Body maps it today.
func Analytic(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body,
    hooks Hooks) (*topo.Body, bool, error)
```

`Body` runs `Analytic`, then the mesh tail (from `booleaneval.Evaluate` through the final
`ctx.Err()` check) when `Analytic` misses. `refuseSheetOperand` stays at its current position inside `Analytic`.

`internal/booleandispatch/regroup.go`:

```go
// Regroup evaluates chain's plan (boolchain.Run). Step k builds under producer env.Ref+k.
// It returns the result, the number of producer identities its steps used, and ok.
// A non-nil error is cancellation, or a step error for which surface reports true.
func Regroup(ctx context.Context, chain boolchain.Chain, env topo.Env, hooks Hooks,
    surface func(error) bool) (*topo.Body, provenance.Producer, bool, error)
```

`Regroup` supplies the callbacks:

| Callback | Body | Miss when |
|---|---|---|
| `Exact` | `Analytic(ctx, op, envAt(k), x, y, hooks)` | any error not surfaced; `ok == false`; result restates a held mesh; result has any `OperationWarnings` record |
| `Full` | `Body(ctx, op, envAt(k), x, y, hooks)` | any error not surfaced; result has any `OperationWarnings` record |

Each `Full` callback calls `Body` separately and enforces that dispatch's existing per-step limits. The two-step
exception can consume both per-step allowances. It has no shared whole-chain work, record or byte meter.

- `envAt(k)` is `env` with `Ref = env.Ref + k`, where `k` counts successful steps so far. A missed attempt does
  not advance `k`.
- A cancellation is `ctx.Err() != nil` after the callback returns. Return `ctx.Err()`.
- A step error with `surface(err) == true` is an invariant break. Return it unchanged; regroup stops. Root's
  `surface` reports true for a `*BooleanError` with `Code == BooleanEvaluatorFailure`, and for an error wrapping
  `ErrBooleanFailed` that is not a `*BooleanError` with `Code == BooleanEmpty`. Root owns the predicate because
  `BooleanError` is a root type.
- "Restates a held mesh" is `booloperand.HeldFloor(result)` reporting `true`: a faceted, mitred-sweep, coil or
  twisted-sweep payload. Such an `Exact` result is a miss: an accepted analytic step produces an operand without
  a held floor. The two-negative exception may pass its full union's held result into the final full cut. That
  cut retains every ordinary held-operand and mesh proof gate. A route ahead of the mesh tail that returns a held
  mesh is still reachable through `Full`.
- The returned span is the final `k`. The result was built under `env.Ref + k − 1`.
- `Regroup` never writes the document. `Analytic` and `Body` write nothing today (`Body`'s doc comment); keep it so.

Soundness under CLAUDE.md "A decad-side check may only FALSIFY": every check here is a routing decision that can
only send the call back to today's error or surface an invariant break. Each step's result is admitted by its own route's proofs, exactly as a
public call would be. The set identity is a theorem over the recorded inputs, not a geometric test. No residual,
sampled distance or bound comparison admits anything.

#### Root wrapper

Root file `boolean_regroup.go`. It stays in root because it reads `Document.producers`, which only root holds.

```go
// regroupBoolean tries the chain regroup for a refused two-body Boolean. ok=false means
// the caller returns cause unchanged. A non-nil error is cancellation or a surfaced
// invariant break. It stays in root because it reads Document.producers and maps
// BooleanError codes, both root-only.
func regroupBoolean(ctx context.Context, d *Document, op meshbool.OperationKind, a, b *Body,
    cfg booleanConfig, cause error) (*Body, producerID, bool, error)

// historyProducers implements boolchain.Source over d's operation records.
type historyProducers struct{ d *Document }

func (h historyProducers) BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool)
```

- `BooleanProducer` takes `d.n.LiveMu.RLock()`, reads `d.producers[bodyHandle(n)]`, unlocks, and maps
  `OperationCut`/`OperationUnion`/`OperationIntersect` to `meshbool.OpCut`/`OpUnion`/`OpIntersect`. It returns
  consumed input nodes: for `Cut`, the `target` then the `tool`; otherwise recorded order. Any other kind, or no
  record, is `ok == false`. `Operation` records are immutable, so no lock is held past the map read.
- `regroupBoolean` checks Trigger, calls `boolchain.Flatten(op, &a.n, &b.n, historyProducers{d})`, takes
  `d.evalContext(ctx)`, and calls `booleandispatch.Regroup(ectx, chain, envOf(documentNode(d), d.nextProducerID()),
  booleanHooks(cfg), regroupSurfaces)`. `regroupSurfaces(err error) bool` is the predicate Execution in
  booleandispatch defines.
- `booleanHooks(cfg booleanConfig) booleandispatch.Hooks` (`boolean.go`) supplies the root-owned hooks;
  `booleanBodyWith` and `regroupBoolean` both call it.

`performBoolean` calls `regroupBoolean` after the paired Cut continuation declines (Paired Cut continuation):

```go
body, err := booleanBodyWith(ctx, op, a, b, d.nextProducerID(), cfg)
span := producerID(1)
if err != nil {
    rescued, used, ok, rerr := regroupBoolean(ctx, d, op, a, b, cfg, err)
    if rerr != nil {
        return nil, rerr
    }
    if !ok {
        return nil, err
    }
    body, span = rescued, used
}
// containedCutQuantities, ctx check unchanged
// commitOperationSpanIn(ctx, nil, kind, operationInputsForBoolean(op, a, b), body, span, a, b)
```

`containedCutQuantities` runs on the rescued body: its relation reads the public `a` and `b`, and the
rescued body denotes `a − b` (Admitted identities).

#### Source reachability and retention

- Every public record holds strong `*Body` references to its inputs and outputs (history "Atomic publication and
  storage"), so every leaf of a chain stays reachable for the document's lifetime. Regroup adds no retention: the
  result payload stores no pointer to a leaf beyond what the building route already stores for any operand.
- A leaf is retired: the expanded producer consumed it. `booleandispatch.Body` and `Analytic` gate no liveness, and
  `Patterned` already runs `booleanBody` over unregistered bodies. Reading a retired body's geometry is not
  consumption (history "Public API proposal").
- Routes that require live operands miss on a retired leaf, which is a sound miss: `CertifyLoftQuarterSourceEnvelope`
  (`internal/loftdirect/quarter.go`), the held quarter pair (`internal/loftcontact/quarter_held_pair.go`) and copy
  lineage (`internal/facetedrecord/copy_proof.go` `liveIn`).
- A source body cannot be consumed twice or edited: bodies are immutable, and a modify operation returns a new body
  whose producer is not a Boolean, so it is a leaf. `Remove` cannot reach a leaf, which is already retired.
- A missing producer record (none today; any future eviction) makes that body a leaf, so regroup declines or plans
  without it. It never refuses for that reason.

#### Result identity and history

- Set: the result denotes the same regular set as the caller's call over its public operands (Admitted
  identities). Its readings are proven against that set by the routes that built it, so a test checks them against
  the closed-form value of that set.
- Representation: the result differs from any sequential result. Plan bounds its full dispatch steps; a result
  built by the mesh pipeline is `Faceted`.
- Public history: one record, exactly as today: kind `Cut`/`Union`/`Intersect`, the caller's `a` and `b` consumed
  with today's roles (`operationInputsForBoolean`), and the result as the only output. No record names a leaf or a
  private step. The chain is recoverable from `Producer(a)`/`Producer(b)`. `FaceSuccessors` stays
  `HistoryUnknown`, as for every Boolean route.
- Producer identities: the commit reserves the span `Regroup` returns (`commitOperationSpanIn`), as
  `tryPrismUnionChain` does. `Body.Origin` names the final step's producer.
- Roles: faces a private analytic step mints carry fresh roles under that step's private producer, which no
  caller can name, plus the source roles prism-boolean §11's attribution proves (each step runs
  `booleandispatch.Analytic`). Each mesh step keeps its operands' face origins, upstream roles included
  (evaluator §9). A face §11 leaves unattributed, such as one two sources share, names no source feature, so a
  selector naming a source feature's origin can match fewer faces than it would on a sequential result. Select
  such a face by geometric predicate.
- Diagnostics: the result carries no `OperationWarnings` (Execution in booleandispatch rejects any).
- Recognizers in the mesh tail (`obliqueboss.Recognize`, `facetedrecord.RecognizePrismUnionFillet`,
  `crossbore.RecognizeBore`, `loftfacestrim.RecordConeToe`) read the final step's operands. Their records are claims
  about the body built from those operands, so they hold.

#### Failure behavior

| Condition | Result |
|---|---|
| Trigger not met | Today's `err`, document unchanged. |
| `Flatten` declines (no expandable operand, ineligible body, mixed chain, more than 6 literals) | Today's `err`. |
| Plan cannot finish its remaining literals | Today's `err`. |
| A step fails with an error `surface` does not report, or a step result carries warnings | Today's `err`. |
| A step fails with an error `surface` reports (invariant break) | That error; no commit. |
| Context canceled during regroup | `ctx.Err()`; no commit. |
| Rescued body built | Commit as Root wrapper states. |

Every non-commit path leaves the live set, producer counter and history unchanged (evaluator §8). Level and curve
counters may advance, as for any failed build.

#### Public behavior

No exported symbol is added. A refused two-body Boolean may return a body; that body's selector roles follow
Result identity and history. Core §8 "The chain depth", evaluator §9's gate bullet, `docs/api.md`, the
`Union`/`Cut`/`Intersect` doc comments (`boolean.go`) and the two missing-features rows describe it.

### Performance

- Calls that succeed today pay nothing.
- A refused call with an expandable operand pays one history map read under `RLock` per examined body (the walk
  stops past `MaxLiterals` literals), one `Exact` attempt per distinct `(op, x, y)` key (Plan) and the bounded `Full` steps.
- An `Exact` miss on a structural gate costs a payload type switch; a miss after a private `sketch` scene costs
  that scene.
- No timing threshold is set. Repository rule: measure on CI, not locally.

### Test plan

Every fixture asserts computed geometry. Use `require`, `t.Context()`, and bounded diagnostics (scalars, counts,
handles); never dump topology. Helpers come from `apitest`: `retentionScene` (`faceted_source_retention_test.go`),
`slabYTool` (`faceted_plane_slab_test.go`), `boxBody`/`boxBodyAtZ` (`clearance_test.go`), `translated`,
`volumeMM`, `requireBodyWatertight` (`boolean_test.go`), `tookMeshPath` (`prism_boolean_test.go`).

#### Fixtures

All F fixtures use one `retentionScene` except F3. `box` = `sc.roundedBox(t)` (40×20×20, four vertical edges filleted
r = 3). `t1` = `sc.tool(t, 20)` (Ø8 along y through (20, ·, 10), placed by translation). Reference box volume
`Vbox = (800 − 4·(1 − π/4)·9)·20`.

Extra bodies:

- `t2 = slabYTool(t, sc, 22, 3)`: Ø6 along y through (22, ·, 10), placed as `t1` is.
- `t3`: circle r = 3 at (20, 10) on `sc.w.XY()`, extruded 20 `Along`.
- `C`: rectangle x 18..26, y 2..18 on `sc.w.XY()`, extruded 20 `Along`.
- F3 uses `decad.New(decad.ExactOnly())`: `plate = boxBody(t, doc, 0, 0, 40, 20, 10)`,
  `box2 = boxBody(t, doc, 40, 0, 60, 20, 10)`, `boss = translated(t, boxBodyAtZ(t, doc, 10, 8, 14, 12, 5, 20), 0.125, 0, 0)`.

| ID | Chain the caller writes | Regrouped plan | Reference volume, mm³ |
|---|---|---|---|
| F1 | `Cut(Cut(box, t1), t2)` | Absorb misses; Merge `Union(t1, t2)`; `Full Cut(box, U)` | `Vbox − 20·(25π − lens(4, 3, 2))` |
| F2 | `Cut(Cut(box, t1), t3)` | Absorb `Cut(box, t3)`; `Full Cut(·, t1)` | `Vbox − 320π − 180π + Vx` |
| F3 | `Union(Union(plate, boss), box2)` | Merge `Union(plate, box2)`; `Full Union(·, boss)` | `12000 + 4·4·15 = 12240` |
| F4 | `Intersect(Cut(box, t1), C)` | Absorb `Intersect(box, C)`; `Full Cut(·, t1)` | `2560 − 16·(32π/3 + 4√3)` |

Every Absorb or Merge step the Regrouped plan column names is analytic; every `Full` step takes the mesh path.

- `lens(r1, r2, d) = r1²·acos((d² + r1² − r2²)/(2·d·r1)) + r2²·acos((d² + r2² − r1²)/(2·d·r2)) −
  ½·√((−d + r1 + r2)(d + r1 − r2)(d − r1 + r2)(d + r1 + r2))`, as in `TestBooleanChainDepthGateReadsOnlyTheTouchedRims`.
- `Vx` is the intersection of `t1`'s and `t3`'s cylinders, axes crossing at (20, 10, 10):
  `Vx = 36·∫_{−π/2}^{π/2} cos²φ·√(16 − 9·sin²φ) dφ`. Evaluate by the trapezoid rule on N = 4096 and N = 2048 nodes
  (the integrand is smooth and π-periodic); require the two to agree within 1e-12 mm³ and use N = 4096.
- F1 also checks vertices: every result vertex with y = 0 or y = 20 (within 1e-9) whose distance from a rim's centre,
  (20, 10) with r = 4 or (22, 10) with r = 3, is within 0.05 of that rim's radius satisfies
  `|hypot − r| ≤ Vertex.Position().Bound`. The result has four such pairs: the two rim crossings on each of the two
  faces lie on both rims.

Assertions for every F fixture:

- The call returns a solid, one lump, watertight (`requireBodyWatertight`).
- `|Volume − reference| ≤ Volume.Bound + 1e-6` mm³ (the float slack `apitest/faceted_plane_slab_test.go` uses), and
  `Bound < 1e-3 · reference`.
- `tookMeshPath(result)` is true (the one mesh step is honest). `TestRegroupPlansBuildByHand` pins this premise on the hand-built final step.
- History: the last record has the caller's kind, the two public operands as consumed inputs with the roles
  `operationInputsForBoolean` gives (`target`/`tool` for `Cut`, `a`/`b` for `Intersect`, `first`/`others` for
  two-body `Union`), and outputs `[result]`. For each leaf behind an expanded operand (`box` and `t1` in F1, F2, F4;
  `plate` and `boss` in F3), `Consumers(leaf)` returns only the earlier Boolean that consumed it. `doc.Bodies()`
  contains `result` and neither public operand.

#### Decline fixtures

| ID | Setup | Why regroup declines | Assert |
|---|---|---|---|
| D1 | F1, but `Cut(Copy(first), t2)` | `Copy` producer: leaf, no expansion | `ErrUnsupported`, one diagnostic `OpDiagOperandBoundExceeded`, history length and `doc.Bodies()` unchanged |
| D2 | F1's `first`, then `Union(first, t2)` | `Cut` result inside a disjunction: leaf | today's error unchanged |
| D3 | F1 with `Cut(first, t2, decad.WithMatchingFaces())` | Trigger excludes the option | same as D1 |
| D4 | F1 with `t2` rotated 0.01 rad about the x axis through (22, 10, 10) | The full tool union builds, but the final full cut refuses | same as D1 |

#### Unit tests (`internal/boolchain`)

- `Flatten`: each expansion rule row; role-based `Cut` input reading; leaf on non-faceted payload, non-Boolean
  producer, `PointCone != nil`, gap-shift and embedding-reach warnings; expansion through a chain-allowance warning;
  a 7-literal chain declines; no expansion declines; mixed chains decline.
- `Run`: Absorb order and restart; Merge first-pair order and restart; full-step bounds per Plan; miss cache (no
  key repeated); two remaining negatives use a full union then cut in input order; a third remaining negative
  or two pending mixed literals decline; three disjunction items decline; a callback error returns unchanged;
  a pre-canceled context returns `ctx.Err()` before any callback. Fakes record calls; bodies are bare `topo.Body`
  values.

#### Existing tests this change flips

`TestChainToleranceAdmitsAContactOnACoarserRim` (`apitest/chain_tolerance_test.go`), the "a contact on a curved
rim still refuses" subtest of `TestFacetedPlaneSlab` (`apitest/faceted_plane_slab_test.go`) and the
sequential-premise subtests of `TestRegroupPlansBuildByHand` test the sequential refusal. Each calls `Copy` on the
chain's intermediate body, because a `Copy` ends the history chain and the final call is then the sequential gate.
Rule for any other test whose refusal regroup rescues: if its purpose is the gate or contact refusal itself,
insert the `Copy`; otherwise assert the rescued geometry against a closed form.

### Open questions

- A regrouped result carries no public warning, although its selector roles differ from a sequential result.
  Adding one (e.g. an `OpDiagChainRegrouped` code, which does not exist) would need a core §6 and
  operation-diagnostics review plus `docs/api.md` and `docs/data-structures.md` edits.

### Tasks

All of Part R is implemented. Each task below gives its owner and its pinning tests.

#### Task 1: Design doc and the hand-built plans

Owner: none (tests only). `TestRegroupPlansBuildByHand` (`apitest/boolean_regroup_plan_test.go`) has one subtest per
F fixture asserting: for F1, F2 and F4 the sequential chain's final call fails with
`errors.Is(err, decad.ErrUnsupported)`. F3's subtest instead joins a `Copy` of `Union(plate, boss)` with `box2`
sequentially, which succeeds by the grid route with the F3 volume, a mesh-path result and watertightness. Each plan
step marked analytic in Fixtures has `tookMeshPath == false`; the final `Full` step has `tookMeshPath == true`; the
final body meets the F assertions on volume, bound and watertightness. D1 and D4 subtests assert the sequential
refusal persists through the `Copy` and the tilted `t2'`, and that `Union(t1, t2')` takes the mesh path.

#### Task 2: `internal/boolchain` normal form and plan

Owner: `internal/boolchain/chain.go` and `plan.go` (Chain normal form, Plan). Tests: `TestFlatten*`
(`chain_test.go`) and `TestRun*` (`plan_test.go`).

#### Task 3: Split `booleandispatch.Body` into `Analytic` plus the mesh tail

Owner: `internal/booleandispatch/dispatch.go`. `TestBooleanDispatchAnalyticPrefix`
(`boolean_dispatch_split_test.go`) uses boxes x 0..10 and x 5..15 (y 0..10, z 0..10) on one XY frame:
`Analytic(OpUnion)` is ok with a `prismPayload` of volume 1500 mm³. It also rebuilds F1's `box` and `t1` with
`Extrude`, `Fillet` and `Transform` (root tests cannot use `apitest` helpers): `Analytic(OpCut)` misses with a
nil error, and `Body` returns a `facetedPayload` whose volume is within its bound plus 1e-6 of `Vbox − 320π`.

#### Task 4: Conjunction regroup end to end

Owners: the plan executor `internal/booleandispatch/regroup.go`, and the root wrapper in `boolean_regroup.go` and
`boolean.go` (`historyProducers`, `regroupSurfaces`, `booleanHooks`). Tests: `TestBooleanRegroupConjunction`
(`apitest/boolean_regroup_test.go`) and `TestRegroupSurfacesInvariantBreaks` (`boolean_regroup_test.go`), which checks
`regroupSurfaces` on a `BooleanEvaluatorFailure` (true), a `BooleanEmpty` (false), a `BooleanUnsupportedContact`
(false) and a bare `ErrBooleanFailed` (true).

#### Task 5: Disjunction regroup

`TestBooleanRegroupDisjunction` covers F3 and D2. Its F3 subtest calls `Union(Union(plate, boss), box2)` with no
`Copy` and asserts the F3 volume, the mesh-path watertight result and the history assertions of Fixtures. It does not
assert that the sequential call refuses beforehand. D2 asserts the refusal of `Union(first, t2)` over a `Cut` result.
The plan executor handles a disjunction; `regroupBoolean` has no disjunction gate.

The test names are in `.github/test-shards.txt` and `.github/test-shards-apitest.txt`. Layout rows:
`agent-docs/layout/booleans.md` (`boolean_regroup.go`), `verification.md` (`internal/boolchain/`),
`internal-checks.md` (`internal/booleandispatch/`). `TestCLAUDEMDLayoutStaysCompact` keeps each layout cell at
or under 300 characters.

## Part C — Boolean continuation

This part defines a read-only continuation of a refused Boolean through
certified source expressions. It adds no public operation or option. The module
boundary is owned by [Module boundary gate](module-boundary.md); public history
and computational evidence are owned by
[operation-history-design.md](operation-history-design.md).

This design is unimplemented. Existing part-named continuations do not establish
its admission or its supported input class.

### Accepted source expressions

MUST describe the accepted true solid, including every accepted geometry change.
The public call graph alone cannot supply that description.

| Expression | Required producer evidence |
|---|---|
| `Leaf(K)` | Immutable source-record token `K`, its true-solid contract, placement, bounds and source-face identity. |
| `Image(F,E)` | Creation-issued proof that the output denotes the bijective image `F(E)`, including placement and rounding charges. |
| `Boolean(op,E1,E2)` | Creation-issued proof that the accepted output denotes the regularized operation over these exact expressions. |

- MUST admit only finite, bounded regular solids whose ordinary builders and
  occupied-volume proofs accept them.
- MUST represent each geometry adjustment in the expression. A warning-free
  Boolean can use its retained operands under the existing chain-regroup
  eligibility contract. A shifted Boolean needs the accepted adjusted operand.
- MUST initially limit adjustment maps to one whole-solid translation or
  positive uniform similarity. Multi-component movement stays declined.
- MUST initially limit reconstructible analytic leaves to exact line/circular
  solid prisms whose complete records can be imaged with certified denotation
  and placement charges. Support is a geometry class, independent of a part.
- MUST keep an unsupported record as an opaque leaf. A held faceted leaf cannot
  be refined below its held floor; its ancestry does not waive that floor.
- MUST bound expression size, traversal, materialization and private evaluation
  by one finite continuation budget. The owning implementation must declare its
  numerical ceilings before enabling the route.

`F` denotes one exact mathematical map. Rounded image coordinates require the
existing displacement and denotation machinery. A rounded analytic record plus
a loose bound does not, by itself, prove that its true solid equals `F(E)`.
Preserve the source-image description or decline analytic materialization.

### Producer relation certificate

MUST issue an accepted-expression relation while the original evaluator knows
the exact operands and selected adjustment. The relation binds the result's
immutable source token, the operand tokens, the accepted map and the evaluator's
completed proof. Seal it with the actual public warning and producer identity
before publication.

`ShiftCertificate`, its capture seal and its private snapshot are evidence for
an accepted map. `AuthenticateStoredCutSnapshot` authenticates an original
translated cutter; `AuthenticateStoredUnionShrinkSnapshot` authenticates an
original one-component similarity. These records do not authorize substituting
an independently rebuilt body or selecting a different adjustment.

- MUST validate the exact relation required by this consumer. History identity,
  warning text, matching roles and overlapping volume intervals are insufficient.
- MUST decline when the producer has not issued the relation certificate.
- MUST reject contradictory stored evidence as an invariant error.
- MUST store compact immutable source descriptions and tokens in committed
  computational evidence, following operation-history's Computational evidence.
- MUST use body pointers and document history only in the transient root adapter.
- MUST revalidate the source snapshot, relation and public history under the
  existing commit lock before publishing the continuation.

### Set rewrites

MUST apply only the following equivalences over certified accepted expressions.
The symbols denote true solids; every constructed intermediate still passes its
ordinary boundary, contact, orientation and occupied-volume checks.

#### Cut across Union

For an accepted `U = X ∪ F(Y)`:

```text
U − Z = (X − Z) ∪ (F(Y) − Z)
```

A strict whole-solid proof that `X ∩ Z` is empty permits:

```text
U − Z = X ∪ (F(Y) − Z)
```

This is the reusable identity corresponding to the existing mother-hinge
history. The map `F` must be the original accepted map; it cannot be recomputed
from a fresh retry. Strict enclosure separation may prove the omitted branch.
Overlapping boxes, a sampled gap or a volume comparison cannot prove it.

#### Union after Cut

For an accepted `Q = X − G(Y)` and strict whole-solid separation of `G(Y)` and `Z`:

```text
Q ∪ Z = (X ∪ Z) − G(Y)
```

This is the reusable identity corresponding to the existing brake history.
The final Cut must use the exact accepted `G(Y)`, not an independently shifted
copy of nominal `Y`. The separation proof must read that accepted image.

#### Images of expressions

For a certified bijective rigid placement or positive uniform similarity `T`:

```text
T(X op Y) = T(X) op T(Y)
```

MUST compose certified maps and their charges through the expression. This
permits a placed compound operand to be rebuilt from its immutable sources.
The planner must not require a particular child count, slot-and-pin recipe,
construction name or original model coordinates.

### Admission and refusal

- MUST trigger only after the ordinary public Boolean returns its typed
  `ErrUnsupported` refusal carrying an unambiguous target-side
  `CoarseHeldContactError`. Conflicting nested tags decline.
- MUST decline `WithMatchingFaces`, sheets, point-cone payloads, unsupported
  geometry changes and suppressed/private public-history contexts.
- MUST authenticate every expanded source and adjustment before private geometry.
- MUST keep accepted operands immutable. Materialization builds detached nodes
  under reserved private producer identities.
- MUST run private Boolean steps through ordinary dispatch and its existing
  proofs. Reject every new geometry-change warning in the exact continuation.
- MUST return the original refusal when the expression, relation, materializer,
  rewrite or ordinary private step declines.
- MUST surface cancellation and evaluator invariant failures without publication.
- MUST publish one public operation with the caller's original inputs only
  after result proof, source revalidation and the final cancellation check.

### Bounds and lineage

- MUST carry inherited vertex, surface, area and symmetric-difference charges.
  Reconstruction changes no bound without a proof for that same quantity.
- MUST charge analytic image materialization through existing geometry and
  placement machinery. A finer tessellation cannot erase inherited movement.
- MUST keep source attribution from the actual image and Boolean builders.
  Compose every private step's face correspondence under the history contract.
- MUST remove private producer nodes from committed ancestry only through a
  validated map to the original public source faces.
- MUST preserve the caller's warning meanings and actual displacement charges.
  A warning from an earlier accepted operation is not a new allowance.
- MUST verify the final result at the original requested pair chord, subject to
  its proven held floor. A successful coarser VerifyAll check does not prove the
  original requested chord.

### Declined replacement behavior

MUST decline a plan that chooses a new adjustment while rebuilding an earlier
operation. The exact identities above preserve accepted geometry; they do not
authorize a different tolerant reading of an earlier source expression.

The existing mother continuation requires a new private Union adjustment. The
existing brake continuation can choose new growth while rebuilding an earlier
Cut. Neither behavior follows from the exact identities above. A shared volume
interval, bounded movement of one source mesh, or successful final mesh audit
does not establish equality with the caller's accepted operands.

To admit such a replacement, first design a reusable relation certificate for
`Candidate(E,E')`. Its producer must prove the changed true-set relation and the
boundary and symmetric-difference charges the enclosing operation needs, bind
those charges to its actual accepted geometry, and prove that its composition
is within the enclosing public operation's stated tolerance semantics. It must
cover arbitrary supported expressions, rather than one history recipe. Until
that rule and its real producer-to-consumer proof exist, return the original
refusal instead of reproducing the part-specific replacement.

### Real end-to-end cases

MUST use real public builders, accepted warnings and typed refusals. Never inject
a hand-built shift certificate, synthetic history record or fabricated refusal
to establish the production boundary.

| Case | Minimal construction and required proof |
|---|---|
| First Cut continuation | Build a small mesh-supported solid `X`, an overlapping circular prism `Y`, and a smaller through-tool `Z` strictly apart from `X`. Obtain a real one-component shrink from `Union(X,Y)`. Require its real public `Cut(U,Z)` to reach the typed held-contact refusal, then consume the captured accepted map through Cut across Union. |
| Independent geometry | Repeat with different radius-to-height ratios and a noncircular line-only prism source. The same admission code must run; no model name or fixture identity may enter it. |
| Independent placement | Move the complete construction by translation and by a non-axis-aligned rigid placement. Compare bounded measurements and composed source-face identity under the map. |
| First Union continuation | Build a small `Cut(X,Y)` whose real accepted certificate translates `Y`, then attach a simple prism `Z` strictly apart from the accepted cutter. Require a real ordinary Union refusal before Union after Cut retries it. |
| Compound image | Replace `Z` with a placed warning-free Boolean expression. Rebuild its fixed accepted expression through Images of expressions; vary its Boolean operation and leaf geometry. |
| Required declines | Break separation, omit or alter the accepted relation, exceed the expression budget, request an unproved image, or cause a new private geometry adjustment. Preserve the original refusal and every original live body. |

The first construction is a candidate minimal experiment, not an observed
passing fixture. If the real producer does not yield the required certificate
and refusal, or the exact private plan needs a new adjustment, stop expansion.
Report that boundary before selecting a larger model or adding a replacement.

For each accepted case, assert computed geometry against an independent
analytic reading or bounded section integral. Assert manifold and occupied-set
proofs, original-input immutability, public operation grouping, source-face
correspondence and absence of private ancestry. Test cancellation and exhausted
budgets with bounded diagnostics after the first real geometry case passes.

### Staged implementation

1. Specify and issue the producer relation certificate and the supported source
   image materializer. Reuse existing capture/snapshot proofs; decline every
   unproved branch. Keep these algorithms in internal packages.
2. Run First Cut continuation through the real producer, materializer, ordinary
   evaluator, commit and public readers. Stop if this one instance fails.
3. Add independent geometry, placement and refusal cases. Keep only the root
   history reader, handle adapter and atomic commit wrapper in root.
4. Add Union after Cut and then compound images through the same expression
   provider, planner, materializer and bound rules. Run one real case before
   expanding each stage.
5. Replace an existing part-named route only when its behavior follows from the
   accepted generic contract. Remove its recipe and dedicated bypasses; return
   the ordinary refusal for replacement behavior this design declines.
6. Design Candidate replacement separately if preserving those extra successes
   is required. Do not expand this exact-continuation contract by weakening its
   equality, warning or source-image admission rules.

## Part S — Stacked prism

The `stackedPrismPayload`: an axial stack of recorded sections over one sketch
plane, and the two constructions that build it — a blind `Cut` of a straight
prism by a same-plane prism tool whose sweep ends inside the target, and a
`Union` of two co-directional prisms or stacked prisms whose sweep intervals
overlap or touch without being equal (Part G §3 A1).
`agent-docs/modify-design.md` Part R §9.1 records the prism shells on it — the cup
and the both-caps shell of a holed section (§2.4). This document owns the
payload itself (its record, its invariants, how its body is built and measured,
how it tessellates, and what every consumer does with it). Part B owns the
boolean admission that produces one. References of the form "core §N" are to
`agent-docs/api-design.md`, "prism-boolean §N" to Part B,
"modify-reach §N" to `agent-docs/modify-design.md` Part R, "tessellation §N" to
`agent-docs/tessellation-design.md`.

The blind cut reads prism-boolean §3.1's G3 with its shared-axis offset-plane
arm and its exact rational G5 shift: a tool sketched on the target's datum
plane, or on a `CreateOffsetPlane` of it, is compared over `big.Rat`, and the
one level the tool contributes to the result (its inner end) is that exact
rational rounded to a float once, charged with `rationalFloatError`. Where the
tool's own level is known exactly, the slabs also record the rational that
inner end denotes (§2.1). For identical stored frames and equal finite,
invertible transform bases, the blind-Cut translation arm in prism-boolean
§3.1 runs after ordinary analytic admission misses when the exact in-section
translation is nonzero. It carries its exact axial coefficient through those
level comparisons; exact pure-axial misses keep the earlier Boolean route.

### 1. Problem

A `Cut` whose tool does not span the target misses prism-boolean §3.1's G5 and
takes the mesh path. Its result is a `facetedPayload`, so every later `Cut` on
the body misses G1 and takes the mesh path too, and the mesh path's chain depth
(core §8) refuses the second through hole after a blind bore:

```
decad: not supported by the current evaluator: requested tolerance 0.25 mm
is below the faceted body's minimum mesh bound 0.5 mm; ...
```

A plate with a blind hole is an ordinary part. The two natural ways to draw its
tool — sketched on the plate's top face pointing down, or on the plate's own
plane pointing up to the depth — both end in a plate face, which the mesh path
refuses outright as a coplanar contact. A blind hole therefore needs an analytic
result, and the result is not a prism: it has two different sections at two
different heights.

### 2. The payload

#### 2.1 Record

`internal/stackedrecord` owns `Slab` and `Interface`. A slab records its regions,
its axial interval `Z0`–`Z1`, the two levels' displacement bounds, and, where
the producer knows it, the exact rational each level denotes. An
interface records `LowerExposed` and `UpperExposed`, the material exposed on
one side of its plane. A slab holds one region, or several under §2.2's group
and lining readings. The root payload holds `[]stackedrecord.Slab` and
`[]stackedrecord.Interface` beside its frame, placement and section bound.
There is exactly one interface between each pair of adjacent slabs.

Material is the union of every slab's regions swept over that slab's interval.
The payload is evaluator-private, re-evaluates under `Transform` through the
same `topo.Payload` interface every other payload implements, and carries the
accumulated placement in `xform` exactly as `prismPayload` does.

`sectionDelta` is one bound for the whole payload: the largest section
displacement any slab's region carries (prism-boolean §7). Every reading that
charges a section displacement charges this one.

**Denoted levels.** A level with zero displacement denotes its own held
float. A displaced level denotes a rational the float only rounds:
`fl(x) · fl(Inch.Factor) / fl(Millimeter.Factor)` for an inch depth
(`extent.ExactConversion`), or `12 − fl(3.175)` for a 3.175 mm pocket below an
offset plane at 12. `prismplacement.LevelDenotation` records that rational
beside the held float and displacement it was proven against, on
`prismrecord.Payload` (`Z0Denoted`, `Z1Denoted`) and on `Slab`. `DenoteLevel`
refuses a rational farther from the held float than the displacement, and
`DenotedLevel` answers only for the same held float and displacement, so a
copy that rewrites a level without restating its denotation reads nothing
there. The zero value records nothing, and every producer that does not set
it behaves as an undisplaced-or-unknown level always did.

`Extrude` with a `Distance`, `Symmetric` or `DistanceSide` extent records
the exact rescale of the magnitude, halved for a full-length `Symmetric` and
negated for the side below the plane. A prism's caps carry their levels'
denotations in `topo.CapRecord.ZDenoted`. A `ToFace` stop on such a cap
records the cap's plane at its denoted level carried exactly onto the sketch
frame (`extent.PlaneStopAt`), plus the exact offset
(`extent.ToFaceDenotedLevel`). A `ThroughAll` stop records the far cap of
its farthest stop body when that body is a prism whose two caps are exact
level sets of the sketch frame, the sketch normal is exactly a unit vector,
and every other met body's far-end interval ends at or below the farthest
one's lower end (`extent.ThroughStops.Finish`). Each stop records through
`DenoteLevel`, so a rational farther than the stop's displacement from its
held level records nothing. The blind cut records its inner end as G5's exact
shift plus the tool's denoted level, and carries the target's own end
denotations onto the stack's ends. A `Transform` or `Copy` keeps them with the
rest of the record, and so does a through `Cut` whose tool nests inside a
prism target, since it keeps the target's levels. A prism `Union` over equal
intervals, a prism group's union and the one-scene many-operand union keep
the level their operands share, since G5 admits them only when every operand
denotes it. A stacked union records, at each of its levels, the rational
its operands denote there (`UnionLevel.Denoted`), on a stacked payload's
slabs and on an A1 BRep's stack. A stack's cap and interface faces carry
their slab level's denotation in `topo.CapRecord.ZDenoted`, so a `ToFace`
stop on a stack's cap or floor records it. A chain sheet records its sweep's
denoted levels as `Extrude` does, and `Split` pieces, a hole-free tube shell
and a prism pattern group keep the receiver's. A cup and a closed prism
shell keep the receiver's end denotations on their kept and open ends, and
each derived level (a cup's floor, a closed shell's cavity floor and
ceiling) denotes its source end's rational moved by the thickness's exact
rescale (`prismshell.CapShellInput.StepLevel`, modify §9). A mesh result's retained
plane face keeps its source face's whole `CapRecord`, `ZDenoted` included
(`facetedrecord.SourceTag`), so a stop on it records what the source face,
a stack's cap or floor among them, records. A stop on a BRep or revolve
face or on a faceted face with no retained plane, a `ThroughAll` stop on a
stack or with a competing far side, a chamfered end, a holed band shell,
the BRep stack of general-boolean
§3 A7, a trimmed ribbon and the prism `Thicken` builds from a chain ribbon
record none.

The denotations decide the interface arm, enter the split arm's level
ranges (§7 stage 3), order and merge the stacked union's levels
(`UnionLevels`, general-boolean §3 A1), enter the I4 audit (§2.2) and decide
when general-boolean §3 A7's tool end meets a target cap. They also decide
prism-boolean G5's span, match and overlap relations (prism-boolean §3.2),
where a stack's first and last slab ends stand for the stack's own ends,
and `Split` and `Extend`'s S6 span relation
(`agent-docs/surface-intersection-design.md` §2.1).
Measurements, tessellation and every bound read the held floats and their
displacements
as before, so a displaced level still reads `Approximate` (§4).

An interface stores no field for the material on both sides of its plane. That
material is the narrower side's own region and nothing emits a face for it, so a
stored copy would add an invariant for the audit below to re-check with no reader.

#### 2.2 Invariants

`stackedrecord.Falsify` checks every one of these before a body is built and
before a mesh is chorded. A record that fails one is refused — `ErrDegenerate`
for a record no stacked body matches, `ErrUnsupported` for a shape this
evaluator does not build — never repaired.

| # | Invariant | Sentinel |
|---|---|---|
| I1 | At least two slabs, or one slab holding two or more regions (a prism group, below). A one-slab stack with one region is a prism and is recorded as `prismPayload`. | `ErrUnsupported` |
| I2 | Each slab holds exactly one region, except a prism group's one slab, whose regions are proven pairwise apart, touching nowhere, when the group is built (`prismcells.ProveGroupApart`, below), and the narrow side of a lining interface (below). (A region enclosing no area is refused by the build's own integrals, `ErrDegenerate`.) | `ErrUnsupported` |
| I3 | Each slab has `z0 < z1`. | `ErrDegenerate` |
| I4 | Consecutive slabs meet: `slabs[i].z1 == slabs[i+1].z0` and `slabs[i].z1Delta == slabs[i+1].z0Delta`, both as stored floats. Where both sides record a denoted level, the two rationals are equal. One plane, one displacement, one denotation. | `ErrDegenerate` |
| I5 | Consecutive slabs carry one outer loop record (`loopRecordsEqual`), or their interface meets the union reading below. A clean-nesting cut never touches the outer loop, and a mirror join rewrites every slab's outer the same way, so a cut-built stack's outer wall runs the whole height. | `ErrUnsupported` |
| I6 | Hole sets are monotone, one new hole replaces enclosed old holes through §7's two-cell sketch proof, or the exclusive holes on both sides are pairwise separate through the private scene below. | `ErrUnsupported` |
| I7 | A monotone interface exposes each exclusive hole's interior; an enclosing interface exposes one patch with the new hole's reversed loop as outer and the enclosed old holes as holes; a separated opposed interface exposes every exclusive hole on its material side. | `ErrDegenerate` |

For a monotone interface, every hole of either region equals a hole on the
other side or occurs on that side alone. Only one side has exclusive holes.
Each exposed patch reverses one exclusive hole into an outer loop with no
holes. Reversal rebuilds segments from their walks; the audit accepts either
`outer == reverse(hole)` or `reverse(outer) == hole` for a shell offset loop.

For an enclosing interface, both sides have exclusive holes. One side has
exactly one new hole, and the other has one or more old holes it encloses.
The cut's two whole-loop sketch cells prove the nesting. The patch on the
old-hole side reverses the new hole for its outer and carries the old holes
inside it. Opposed holes that neither nest nor separate remain `ErrUnsupported`
here; a blind `Cut` that leaves them overlapping builds a BRep instead (§7
stage 3).

When both sides have exclusive holes that are pairwise separate, the interface
holds both `LowerExposed` and `UpperExposed`. A private `sketch` scene over the
interiors of every exclusive hole must return one valid whole cell per hole,
no two cells sharing a vertex, with no arrangement charge. Two holes meeting
at one corner would leave the body meeting itself at that point, so they are
not separate (`prismcells.ProveGroupApart`). A tangency is no vertex: the
arrangement neither cuts nor flags a clean tangent contact. So every pair of
entities from two holes with a curved member also goes to sketch's own
intersection query, and any point it reports, within its band, refuses. Two
tangent round pockets, or a round pocket tangent to a rectangular one, are not
separate. `stackedrecord.Derive` and `Falsify` run this proof;
the reached slabs' cut scenes already prove each hole lies inside its section
and clears holes on the same side, touching none of them (prism-boolean §4.2's
tangent-contact query). The lower side exposes each upper-only hole
as a floor, and the upper side exposes each lower-only hole as a ceiling.
Construction refuses this reading when the section has a nonzero displacement:
the scene proves separation of the recorded loops, while that displacement
could close their gap. A rigid placement re-evaluates the same local record.

The same proof decides a prism group's regions (I2) and a `Union`'s several
loops (general-boolean §3 A5). The arrangement does not cut a line at another
line's endpoint lying on it either, such as a triangle's apex on a square's
side, so `prismcells.RegionContact` also puts every two lines of two regions
to sketch's line query: `geom.LineLineIntersection` gives the carriers'
common point, and `geom.ClosestPointOnLine` reads its parameter along each
segment, within 1e-9 of [0, 1] (`lineContactBand`). Parallel carriers report no point; the
arrangement cuts collinear lines where they overlap or meet. A union-built
stack's interface reads `RegionContact` over its nesting match
(general-boolean §3 A1), so a boss outline touching the plate's edge at one
point is a miss there, and `Cut`'s clean-nesting, enclosing and group matches
read it too (prism-boolean §4.2). The A1 BRep's holed floor round separate
bosses reads it per ring through `prismcells.RingContact`, and its slab loops
through `prismcells.ProveGroupApart` (general-boolean §3 "A1 as a brep"). A vertex lying on another region's line whose two
edges leave that line within sketch's parallel band is
reported by neither line query; sketch's arrangement reports such a region
invalid, and the pair is refused before any contact reading runs
(`apitest/union_tangent_contact_test.go`).

A mirror join (`agent-docs/mirror-pattern-design.md` §5) builds a stacked payload by
rewriting every slab's region with one splice. The splice is a function of the
loop record alone, so equal loops in two slabs rewrite to equal loops: the
outer stays one record (I5) and every interface stays monotone (I6). The
join re-derives each interface's exposed records from the rewritten regions
(I7), and this audit checks all three again before the body is built. A
union-built stack, whose outer loop changes between slabs, is not joined.

A union-built stack reads I5–I7 per interface. Where the two outer loops are
one record, the rows above apply unchanged. Where they differ:

| # | Union reading | Sentinel |
|---|---|---|
| I5 | Both regions are hole-free, and the narrower outer is proven nested in the wider region by prism-boolean §4.2's clean-nesting match when the union is built: the wider region's cell carries the narrower outer, whole, as its one hole, and `prismcells.RegionContact` reports no point between the two outers | `ErrUnsupported` |
| I6 | Exactly one side records exposed material: `lowerExposed` when the lower region is the wider one, `upperExposed` when the upper is | `ErrDegenerate` |
| I7 | That side holds exactly one record, `{Outer: wider.Outer, Holes: [reverse(narrower.Outer)]}`. The audit re-derives it from the two regions and compares it record for record | `ErrDegenerate` |

The audit checks the records the union reading names. The nesting itself is
proven once, by the match that built the union, as I6's monotone holes are
proven by the cut that built them.

An interface where one side holds several regions takes the **lining
reading**. It is what a shell's floor and wall slabs meet at (§2.4): the wide
side holds one region `W` with `k` holes, and the narrow side holds the
`1 + k` bands that line `W`'s loops.

| # | Lining reading | Sentinel |
|---|---|---|
| I5 | Exactly one side holds one region `W`; the other holds `1 + k` regions, `k` the number of `W`'s holes. The first narrow region's outer is `W`'s outer and it has exactly one hole; narrow region `m >= 1` has exactly one hole, `W`'s hole `m - 1` | `ErrUnsupported` |
| I6 | Only `W`'s side records exposed material | `ErrDegenerate` |
| I7 | That side holds exactly one record: its outer is the first narrow region's hole reversed and its holes, in order, are every other narrow region's outer reversed, each reversal in either spelling | `ErrDegenerate` |

That every narrow region lies in `W` and that the narrow regions are pairwise
disjoint is proven when the shell builds them, by the offset section's audit
(`agent-docs/modify-design.md` §5); the audit here compares records. Each narrow
region's own loop — the first one's hole, every other one's outer — has no
equal loop on the wide side, so its column starts or ends at the interface,
and the one exposed patch reads those columns' rings there.

A **prism group** (`agent-docs/mirror-pattern-design.md` §6.3) is one slab holding
two or more regions, each a separate lump over the slab's interval. It has
no interface, so I3 is its only level rule and I4–I7 do not apply. The audit
checks the slab, the empty interface list and that every region has an outer
loop; that the regions are pairwise apart is proven when the group is
built, by `prismcells.ProveGroupApart`: a private scene of every region's
outer must return one valid cell per region, each reproducing that outer
with every edge whole and no hole, no two cells may share a vertex, and
`RegionContact` (above) must report no point between two regions. A
`Union` whose survivors close into several such loops builds one
(Part G §3 A5); loops that touch at one
point take the mesh path.

#### 2.3 Loop columns

A **column** is one loop record over a maximal run of consecutive slabs that
all carry it, in whichever region of each slab holds it (I5 and I6 make
"carry it" a `loopRecordsEqual` question). In a
cut-built stack the outer loop is one column over every slab. A union-built
stack has one outer column per run of slabs that share an outer record: a
boss's wall is one column over the slabs the boss alone reaches, and a plate's
wall is one column over the slabs the plate spans. A through hole is one
column over every slab. A pocket's wall is a column over the slabs the pocket
reaches, and a hole that vanishes and reappears is two columns.
`stackedrecord.Payload.Columns` derives the column list from the slabs alone, and the body
build and the tessellator both read it, so a wall is one face however many
slabs it crosses — evaluator §3's canonicalization rule — and both consumers
name the same face by the same role.

A column records its loop, the slab it starts in, the region holding the loop
there and the loop's index in that region, its sweep interval `[z0, z1]` (the first slab's `z0` to the last
slab's `z1`) and the two levels' displacements.

#### 2.4 Shell records

`Shell` (`agent-docs/modify-design.md` Part R §9) records two results on this
payload, with `O` the shell's outer region and `C` its cavity region: `P` and
its offset `Q` inward, `Q` and `P` outward. Its wall bands are the band between
`O`'s outer and `C`'s outer (`C`'s outer reversed as its one hole), then, per
hole `i`, the band between `C`'s hole `i` reversed (an outer) and `O`'s hole
`i`.

| Result | Record |
|---|---|
| both caps removed, `k >= 1` holes (reach BX8) | a prism group: one slab over the receiver's sweep holding the `1 + k` bands; `sectionDelta` is the offset's proven displacement |
| no opening, `k = 0` (reach BX5, `WithNoOpenings`) | three slabs: the cap region (`P` inward, `Q` outward) under the band `{O.Outer, reverse(C.Outer)}` under the cap region, both interfaces monotone, each exposing `C`; `sectionDelta` is the offset's proven displacement; `closedShell` is set; the cavity is a void shell (§3) |
| no opening, `k >= 1` (reach BX5) | three slabs: `O` under the `1 + k` bands under `O`, both interfaces lining readings, each exposing `C`, a region with `k` holes; `sectionDelta` is the offset's proven displacement; `closedShell` is set; the cavity is one void shell around every lining (§3). Outward, `C` is `P` itself and each lining lies between `P`'s hole and `Q`'s smaller one |
| one cap removed, any `k` (a cup, modify B5/B6) | a floor slab over `O` and a wall slab over the bands, meeting at a lining interface (`k >= 1`) or a monotone one (`k = 0`) whose one exposed record is `C` itself; `sectionDelta` is zero |

`closedShell` marks a closed shell's record. A lining band's outer loop is
`C`'s hole reversed, so its column starts at the floor interface, ends at the
ceiling interface and faces the cavity, not the exterior. The mark is on the
payload, not on a naming plan, so `Placed` and every record copied from this
one keep it. A stacked through cut cuts one region per slab and leaves a
pair whose reached slab holds several regions to another route
(`cutActiveSlabs`), so it never copies the record of a closed shell whose
wall slab holds several regions (`k >= 1`) while it reaches that slab. A
hole-free closed shell (`k = 0`) has one region per slab, so a through cut
copies its marked record and builds it exactly. The blind and opposed blind
cuts refuse a stack with any multi-region slab, reached or not
(`prismboolean.TryStackedBlindCut`, `TryOpposedBlindCut`).

A cup is not handed over as a bare `stackedPrismPayload`: `cupPayload` (`internal/cupwall`) holds
the record beside its shell morphology, and its consumers read the cup's own
view of it (reach §9.1). Its offset loops carry the offset displacement column
by column (§4).

### 3. Topology and roles

`stackedrecord.Build` builds the body under the boolean's own private producer
identity, after `stackedrecord.Falsify`. It builds through the payload's own
naming plan; a cup builds the same record through its own plan
(`stackedrecord.Plan`), which mints the cup's roles (modify Table B) and states each
column's displacement:

| Face | Built from | Surface | Role |
|---|---|---|---|
| wall | one column, through `prismrecord.BuildLoopSidesAs` over a `prismPayload` view of that column (the payload's frame, placement and `sectionDelta`; the column's interval and level displacements) | `Plane` or `Cylinder` per segment | `slab(k).region(r).side(i,j)` — `k` the slab the column starts in, `r` the region (0 outside a prism group), `i` the loop's index in that region, `j` the segment |
| bottom cap | the first slab's region | `Plane` at `slabs[0].z0`, outward `-N` | `capStart` |
| top cap | the last slab's region | `Plane` at the last `z1`, outward `+N` | `capEnd` |
| floor | `interfaces[i].lowerExposed[e]` | `Plane` at the interface level, outward `+N` | `floor(i,e)` |
| ceiling | `interfaces[i].upperExposed[e]` | `Plane` at the interface level, outward `-N` | `ceiling(i,e)` |

A union-built interface's one patch has two loops: its outer is the wider
side's outer column, read at that column's end on the interface, and its hole
is the narrower side's outer column, read at that column's start or end there.
`stackedrecord.Payload.InterfacePatches` lists every patch with its column rings, and the
body build and the tessellator both read that list.

Every planar face's loops are the coedges the column walls already placed at
that level, so every edge bounds exactly two faces: a column that starts in
slab 0 gives its bottom coedges to `capStart`, one that starts at an interface
gives them to that interface's floor; a column that ends in the last slab gives
its top coedges to `capEnd`, one that ends at an interface gives them to that
interface's ceiling. `stackedrecord.Build` derives the lump set from face adjacency:
each connected face set holding a region outer's wall or an end cap is one
lump's outer shell, and a connected set holding neither — hole walls closed
by a floor below and a ceiling above, a closed shell's cavity (§2.4) — is a
void shell of the one outer lump. On a record marked `closedShell`, no outer
column that starts and ends at an interface anchors a shell. In the closed
shell's own record those columns are exactly the hole linings, whose walls
face the cavity. A record copied from a marked one keeps the mark, and there
such a column need not be a lining; a set that reaches an end cap stays an
outer shell whatever its columns, and tessellation's per-shell winding audit
(`agent-docs/tessellation-design.md` §4) refuses any body whose lump reading
disagrees with its facets. The closed shell's own build requires one lump of
one outer shell around one void shell (`prismshell.BuildClosed`). Several
outer lumps beside a cavity would need a nesting proof and are
`ErrUnsupported`. A stage-1 body is one lump with one shell.

A prism group carries one bottom cap and one top cap per region, every one
under `capStart` or `capEnd`, so `CapStart(body)` selects one face per lump,
and `Lumps()` reports one lump per region. Columns, measurements and
tessellation run per region; loop clearance is proven over every ring of the
one slab, all regions together.

`capStart` and `capEnd` keep their prism names so `CapStart(body)` and
`CapEnd(body)` select them. Roles are fresh under the boolean's step and no
operand `FeatureRef` is carried forward (prism-boolean §11).

### 4. Measurements

Every quantity is a bounded sum of per-slab or per-column terms through
`internal/proofbound/bounded.go`'s arithmetic, so every float operation between a proven term and
the published value charges its own rounding.

| Quantity | Composition |
|---|---|
| `Volume` | `Σ_k A_k · h_k` — each slab's region area (with `sectionDisplacementArea(sectionDelta, walks_k, perimeter_k)` folded into its bound, as `prismrecord.Build` does) times its bounded height |
| `Area` | the first region's area (bottom cap) + the last region's area (top cap) + each exposed record's area (`momentinput.LoopEnclosedArea` of its outer loop) + `Σ_columns perimeter · height` |
| `Centroid` | `Σ_k m_k · c_k / Σ_k m_k`, with `m_k = A_k · h_k` and `c_k` the region's centroid lifted to the slab's bounded midpoint through `prismPayload.Point`; taken component by component in bounded arithmetic, each `c_k` component carrying `prismrecord.Payload.PointBound`'s radius as its bound, and the three component bounds folded into one radius through `proofbound.Radius3D`. `prismrecord.Payload.CentroidGeometryBound`, the largest over the outer runs, plus `sectionDelta`, caps the result as `prismrecord.Build`'s does |
| `Bounds` | the union of `prismrecord.Bounds` over each outer run: one outer-only prism per run of slabs sharing an outer record, over that run's interval with its end levels' displacements and the payload's `sectionDelta`, the largest run bound covering the union. A cut-built stack is one run, the outer-only prism on the full interval |
| `ExtentAlong` | the extreme over every outer run's reading |
| `AxialDelta` | the largest level displacement over every slab |

A column whose plan states a displacement (a cup's offset loop, §2.4) builds
its walls with that displacement added to `sectionDelta`, so its vertices,
lengths and face areas carry it; each region holding the loop adds
`sectionDisplacementArea` over that loop alone to its area and widens its
first moments by that area times the farthest coordinate the displaced
boundary reaches, so the centroid quotient covers it; each planar patch the
loop bounds adds the same area term; and every outer run whose outer loop is
displaced reads its box with that displacement. The geometric centroid cap is
then a ceiling (`math.Min`) on the covered formula answer. A zero column
displacement adds no term, so a boolean-built stack's readings are unchanged.

`Exactness` follows `exactnessOf` on each composed bound: a rectangular pocket in
a rectangular plate drawn on one plane reports `Exact` volume with a zero bound;
any circular wall, any nonzero level displacement or any nonzero `sectionDelta`
reports `Approximate`.

### 5. Tessellation

`tessellateStacked` (tessellation §5's prism contract, applied per column and
per planar patch):

- Reserve `sectionDelta` from the requested tolerance exactly as the prism
  arm does, through the same helper, and refuse a tolerance it exhausts.
- Chord each column's loop once (`chordLoop` over the column's height). Its
  samples get one vertex at the column's `z0` and one at its `z1`; its walls
  are the prism's outward quads. Chording per column is modify-reach §9.1's
  "once per shared carrier".
- Prove loop clearance (`tessellation.RequireLoopClearance`) **per slab**, over the rings of
  the columns that slab carries. Two pockets from opposite faces may overlap in
  plan while never sharing a slab; a whole-body clearance proof would refuse a
  valid body.
- Triangulate `capStart` from slab 0's rings at their `z0` (reversed, `-N`),
  `capEnd` from the last slab's rings at their `z1` (`+N`), each floor from its
  upper-only column's bottom ring with the hole's winding reversed (`+N`), and
  each ceiling from its lower-only column's top ring, reversed, emitted
  reversed (`-N`). A union-built patch triangulates the wider outer column's
  ring as its outer and the narrower outer column's ring, reversed, as its
  hole. Its two rings come from two slabs, so `tessellation.RequireLoopClearance` runs over
  the patch's own rings before it is triangulated. Every planar patch indexes the already allocated ring
  vertices, so the mesh closes by construction and `internal/tessellation.RequireClosedMesh` proves it.
  A separated opposed interface emits its floor and ceiling patches from
  different column rings at the same level; the private scene proves those
  planar patches do not overlap.
- Face bounds: a wall's largest sagitta plus the larger of its column's two
  level displacements; a planar patch's largest bounding-loop sagitta plus its
  own level's displacement; `tessellation.FaceBounds` adds every vertex's store term
  and `sectionDelta`.
- `areaSlack`: per column `wallSlack + 2 · capSlack` (every column ring bounds
  exactly two planar patches, one at each end); when `sectionDelta > 0`, one
  `sectionDisplacementArea` term per planar patch over that patch's own loops
  and one `sectionDisplacementLength · height` term per column; then
  `tessellation.StoreAreaAllow`.
- Occupied volume (`tessellation.SymDiff`): `Σ_k h_k · E_k`, with `E_k` the summed
  circular-segment area of the columns slab `k` carries; `Σ_k h_k ·
  sectionDisplacementArea(sectionDelta, walks_k, perimeter_k)`; each planar
  patch's level displacement times `tessellation.FaceAreaUpper` of that patch; and
  `sweptVolumeAllow` over the store maximum. The body is the disjoint union of
  its slab prisms and the mesh solid is the disjoint union of the chorded slab
  prisms over the same intervals, so the symmetric difference is at most the
  sum of the per-slab symmetric differences, each of which is tessellation §5's
  own prism bound.

The mesh carries the occupied-volume proof, so a stacked body is an ordinary
mesh-path boolean operand and an ordinary export input. `booloperand.PairChord`
reads its `sectionDelta` through `booloperand.sectionDisplacement`, as it reads a prism's.

### 6. Consumers

| Consumer | Behaviour |
|---|---|
| `Transform` / `Copy` | re-evaluates the payload under the composed motion |
| `ScaledAt` | Exact line-only solid sections with zero section and level displacement become a certified faceted image; source roles and planar tags follow the scale (evaluator §8). |
| later `Cut` with a prism tool | A spanning tool or a blind tool ending inside a slab or exactly at an interface builds when each reached slab holds one region and proves a clean cut or no change inside a hole; a reached slab of several regions leaves the pair to another route; opposed interface holes require exact, separated sections, and overlapping ones build a BRep (§7). |
| `Union` with a stacked operand | Part G §3 A1: every slab region hole-free, or holed under §3 "A1 as a brep"'s holed-operand gate (a brep result), the stack splits at every level of both operands; a prism-group operand over the partner's interval is A5's |
| `Cut` by a prism-group tool | Part G §3 A5 on a prism target: one arrangement for every lump |
| `Intersect` with a stacked operand | mesh path, over this payload's own tessellation |
| `Tessellate`, `export.STL` / `OBJ` / `STEP` | §5; a body with one shell meets STEP's one-shell rule, and a closed shell's void shell is STEP's refusal |
| `ThroughAll` / `ThroughAllSide` stops | `ExtentAlong` (§4), `ErrUnsupported` at `sectionDelta > 0` as for a prism |
| `ToFace` stops | read the selected face's own `AxialDelta` and cap record |
| `Verify` structural audit | every edge bounds two faces by construction (§3) |
| `Verify` tolerance gate | `gatewitness.Prisms` reads every outer run's prism over its own interval, whose wall stations are all points of the body, and shrinks their maximum by `sectionDelta + axialDelta` plus the widest station gap (`agent-docs/verification-design.md` §3) |
| `Verify` wall survey | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (modify-reach Table DX, DX9); a pocket floor is a wall the 2D spanning-disk proof does not read |
| `Verify` undercut and minimum-radius surveys | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (stage 4 adds DX7 face normals and DX8 `radiussurvey.Prism` over each loop) |
| `Verify` clearance | `newBodyGeomBudget` has no arm, so a pair the boxes do not separate reads `Suspect`; a box-disjoint pair is proven (stage 4 adds the exposed-face model, DX6) |
| `Verify` interference | `analyticBodiesEqual` answers undecided; the read-only mesh intersection reads §5's proof |
| `Fillet` / `Chamfer` / `Shell` | through the brep face view (modify-reach RX3); a prism group and a stack enclosing a cavity have none (brep-modify SB2). `internal/stackedshell` reads a two-slab stack with that view for the exact boss, blind-pocket and counterbore shells (`agent-docs/modify-design.md` Part G §3.1a-§3.1c, §3.1e) |
| `Thicken` / `Offset` / `Patch` / `Stitch` | not reachable: the body is a solid |

### 7. Stages

Stages 1 to 3 are implemented. Stage 4 is not.

1. **Blind cut and through cuts on the result.** The payload, `stackedrecord.Build`,
   `tessellateStacked` with its occupied-volume proof, prism-boolean §3.2's
   `Cut` rows for a blind tool on a prism target and a spanning tool on a stacked
   target, the `Verify` gate arm, `booloperand.sectionDisplacement`'s arm, the explicit
   stagings in §6, and an executable example. The gallery's landing clip cuts
   one blind hole per frame and renders the body, so the tessellator belongs
   to the same stage as the cut that produces the payload.
2. **Enclosing counterbore.** A blind tool around existing holes builds when
   sketch returns the target's outside cell with the tool as one hole and a
   second cell inside the tool with precisely the enclosed holes. The exposed
   floor or ceiling is `{Outer: reverse(tool), Holes: enclosed holes}`. A
   spanning tool inside a pre-existing blind hole leaves that slab unchanged
   when a private scene returns both the unchanged material cell and the
   annulus between the existing hole and the tool. A spanning cut on the other
   slabs then builds the same counterbore in the opposite construction order.
3. **Blind cut on a stacked target.** A tool ending strictly inside a slab
   splits it and proves a clean cut or no change inside a hole in every slab
   it reaches. The new interface and all earlier ones re-derive their exposed
   patches; the record audit checks them before evaluation. A tool ending
   at an existing interface uses that level when the tool end's denoted level
   equals the rational both slabs record there (§2.1), compared exactly; a
   zero displacement on both sides is the case where each rational is the held
   float itself. Equal held floats never stand in for equal denoted levels:
   0.2 in and 5.08 mm hold one float and denote two planes, so that pair
   declines, while 0.125 in and 3.175 mm denote one plane and share the
   interface whatever rounding the floor level carries. A tool end with a
   displacement and no recorded denotation, or an interface whose slabs record
   none or disagree, declines this arm. The split arm reads levels as
   prism-boolean §3.2's exact ranges (`prismplacement.LevelRange`): the tool
   end must lie strictly above the containing slab's lower level and
   strictly below its upper one across both ranges, a denoted level being a
   single rational and an undenoted one its held float widened by its
   displacement. The blind cut on a prism, and the outer end of either blind
   cut, read the target's caps the same way, so a tool end within its
   displacement of a cap or an interface declines rather than claiming an
   open pocket or a slab, and an inner end whose held float rounds onto a cap
   declines too. The reached slabs' private scenes
   plus the interface audit prove the new hole set is monotone. Opposed
   exclusive holes also build when the private scene above proves their
   separation without a charge and the section displacement is zero.

   A tool whose curved wall touches an existing hole, an enclosed hole or the
   outer wall at one point, without crossing it, matches no cut cell: the
   blind cut on a prism, the split and interface arms here and the spanning
   cut all decline it through prism-boolean §4.2's tangent-contact query, and
   it takes the mesh path.

   Opposed exclusive holes that overlap in plan at that interface build a
   BRep through general-boolean §3 A8. The reached slabs keep this stage's
   cut proofs. A private scene over only the exclusive holes gives the
   partial floor and ceiling as cells, which this payload's whole column
   rings cannot hold. Every exclusive hole is a loop of axis-aligned lines,
   circles and arcs: a round pocket across a rectangular one, a rectangle
   inside a round pocket, a slot across a rectangle, or a counterbore drilled
   from both faces. Opposed holes that only touch, along an edge, at a corner
   or at a tangency, have no manifold result and take the mesh path. So do
   two circles or arcs of opposite sides that meet, opposed holes at a
   displaced level, holes that carry a section displacement, and holes with
   an oblique line, an ellipse or a free-form segment.

   A blind tool crossing one existing through hole on a prism target, every
   loop of axis-aligned lines, circles and arcs (a rectangle across a round
   hole or a slot, a round pocket across a square hole), uses general-boolean
   §3 A7's BRep path. Its partial floor cannot be represented by this
   payload's whole column rings. A7 reads the tool's and the target's denoted
   levels (§2.1) where the tool ends at a target cap. A round pocket across a
   round hole, where two circles cross, takes the mesh path, and a pocket
   enclosing the hole keeps stage 2's payload.
4. **Surveys and clearance (not implemented).** DX7, DX8, DX6 and `analyticBodiesEqual` for the
   stacked payload; §6 states the current staged answers.

Not implemented: a blind tool crossing the target's boundary (a side notch),
whose per-slab walls split at the crossing; it takes the mesh path, and
general-boolean §3 A9 builds it as a BRep where the mesh path refuses.
`Fillet` and `Chamfer` of a stacked body build through the face view
(`agent-docs/modify-design.md` Part B), not through this payload (§6).

### 8. Required tests

Every test asserts on computed geometry. Bounds are asserted as relations and
ceilings, never as literals, because FMA rounding differs between amd64 and
arm64.

- A rectangular blind pocket in a rectangular plate, both drawn on one plane,
  from each end: the result is analytic (no `Faceted` face), has one lump and
  eleven faces, every edge bounds two faces, and its `Exact` volume equals
  the rational answer. Its centroid bound contains the rational answer;
  the Z coordinate is `Approximate` when that answer is not a float.
- A round blind bore: `Approximate` volume within the published bound of
  `A·h − π r² d`.
- Blind bore then two through bolt holes: all three build analytically, the
  volume lands within its bound of the closed form, and the face count is ten.
- A tool whose cap lies exactly in the target's far face builds (the mesh path
  refused it as a coplanar contact).
- A tool touching the target only at a cap (zero depth), a tool strictly inside
  both ends (an enclosed void), and a blind tool crossing the outer boundary
  each still take the mesh path with the mesh path's own result.
- A round blind pocket touching the square through hole's wall at one point,
  from either cap, on a prism or a stacked target, and one touching the outer
  wall or enclosing a round hole it touches, take the mesh path: the result
  meshes under `VerifyAll` and its volume bound covers the closed form. The
  same tools run through the whole target return `BooleanUnsupportedContact`,
  while a through tool tangent inside a cylinder's wall builds a faceted
  result whose bound covers `960π` and meshes.
  Square pockets touching the hole along its wall or at a corner mesh and
  carry the closed-form volume (`apitest/cut_tangent_contact_test.go`).
- Two blind tools ending at different levels build a three-slab analytic
  result from either face; the mesh and placement retain the volume proof.
- Two separate blind tools ending exactly at the same interface build an
  analytic stack when their reached slabs' scenes prove clean cuts and the
  interface stays monotone. Separate pockets from opposite faces ending at
  that interface build two distinct exposed faces when the private scene
  proves their holes separate, with analytic volume, verified watertight mesh,
  and placement. Opposed pockets touching along an edge or at a corner take
  the mesh path, which refuses them under `ExactOnly`. Overlapping opposed
  pockets build general-boolean §3 A8's BRep with an `Exact` volume, every
  edge on two faces, a verified watertight mesh and a `Sound` report. Where a
  plan loop holds a circle or an arc, the volume is `Approximate` and its
  bound covers the closed form with its π terms; a pocket tangent to the
  other side's edge, or two crossing circles, take the mesh path
  (`apitest/stacked_opposed_arc_test.go`).
- Denoted levels: on a plate 12 mm thick, two pockets beside each other cut
  from z = 12 by 3.175 mm and 0.125 in, in either order, by 0.2 in twice, or
  by 0.2 in against a full-length `Symmetric` 0.4 in or a `TwoSided` 0.2 in
  side, share one displaced floor level: the result is analytic with sixteen
  faces, one lump, an `Approximate` volume whose bound covers the rational
  answer, a `VerifyAll` mesh and a `Sound` report. A nested pocket ending on
  the outer pocket's floor leaves eleven faces. A level split at 0.2 in under
  a deeper pocket is reused by a third pocket at 0.2 in. 0.2 in beside
  5.08 mm takes the mesh path. Record tests decline a displaced tool end or
  interface with no denotation, an interface whose two sides record different
  rationals (and the audit refuses it), and a split whose denoted order
  contradicts the held one.
- Denoted levels carried by stacks: a `ToFace` stop on a blind-cut stack's
  top or a stacked union's top at 0.2 in cuts a 0.2 in plate into the analytic
  through hole, and a 5.08 mm plate takes the mesh path. A stacked union, and
  an A1 BRep union, that ties a second 0.2 in top builds analytically with
  the closed-form volume, and a 5.08 mm second top takes the mesh path. Record
  tests show the union's levels, the A1 BRep's stack and every stack cap and
  interface face carrying the rational
  (`apitest/stack_denoted_level_test.go`,
  `apitest/through_stop_denoted_level_test.go`).
- Denoted levels carried by shells: a `ToFace` stop on a cup's floor, kept
  cap or rim, opened at either end in either sense, or on a closed shell's
  cavity floor, ceiling or top cap, cuts a plate whose cap denotes the same
  rational (0.2 in, or 10 mm or 20 mm through a 0.2 in offset) into the
  analytic through hole. A stop holding the plate cap's float but denoting a
  plane just inside it (a 5.08 mm plate or thickness, or a −5.08 mm offset)
  takes the mesh path. Record tests read every derived level and slab end
  (`apitest/shell_floor_denoted_level_test.go`,
  `internal/cupwall/payload_test.go`, `internal/prismshell/cap_shell_test.go`).
- Level displacement, incoming: a tool whose inner end is a converted magnitude
  (`units.Inches`) publishes a positive axial delta on that end, asserted first
  on the tool so the fixture cannot silently stop exercising it, and the
  result's volume bound is at least that delta times the floor's area.
- Level displacement, rounding: a tool sketched on `CreateOffsetPlane(XY,
  0.1)` and extruded `Symmetric` 0.3 mm into a plate on XY puts its inner end at
  the exact rational `fl(0.1) + fl(0.3)`, which is no float; the result's
  ceiling level is that rational rounded once and its delta equals
  `rationalFloatError` of the rounding, asserted positive. A tool extruded
  `Distance` from an offset plane puts its inner end exactly on that plane
  (`0 + s`, a float), so its delta is exactly zero.
- Section displacement: a tool placed in its plane (nonidentity re-expression)
  yields `sectionDelta > 0`, an `Approximate` volume even for a rectangular
  pocket, and a bound containing the exact rational residual.
- Payload audits I1–I7, each on a hand-built payload, each refusing with the
  stated sentinel.
- Tessellation at `VerifyAll`: the mesh closes, `Bound` is at or below the
  requested tolerance plus the axial maximum, and a mesh-path `Cut` by a
  non-coplanar tool on the pocketed plate builds, so the occupied-volume proof
  is published.
- Export: STEP of the pocketed plate writes one shell.
- `Verify` on the pocketed plate is `Sound` with no survey asked; with the wall
  survey asked it reports `DiagUnsupportedSurveyPayload` and `Suspect`.
- `Fillet` of the pocketed plate's four vertical convex edges builds, and its volume reading covers `896 + 10π`
  (`TestBlindPocketFilletsItsOuterEdges`).
- `Transform` under a rotation keeps the volume, exactness and face count.
- Cancellation during the second slab's arrangement returns `ctx.Err()` and
  leaves both operands live.

## Part K — Carrier record

`Fillet` and `Chamfer` of a prism whose recorded section is displaced from the
section it denotes, and the producers that make such a prism carry the proof
those ops need. References of the form "prism-boolean §N" are to
Part B; "A6" and "A3" to
Part G §3; "modify §N" to
`agent-docs/modify-design.md`; "brep-modify §N" to
`agent-docs/modify-design.md` Part B; "reach §N" to
`agent-docs/modify-design.md` Part R; "mirror-pattern §N" to
`agent-docs/mirror-pattern-design.md`; "tessellation §N" to
`agent-docs/tessellation-design.md`.

Navigation only (each section owns its rules):

| Question | Section |
|---|---|
| What lands in which PR? | §2.1 |
| What does a carrier record claim? | §3.2 |
| How far does a displaced segment move, point for point? | §3.3 (Lemma H) |
| How is a scene counted against the arrangement cap? | §4 |
| Which constructions set a record? | §5 |
| How does the one-scene `Union` take circles, holes and displaced copies? | §5.3 |
| How does a clean-nesting `Cut` keep a record? | §5.4 |
| Why does a crossing exist, and why only one? | §6.1 (Lemma T) |
| How is an arrangement proven stable under displacement? | §6.3, §6.4 |
| How does a lateral `Fillet` or `Chamfer` of a displaced prism build? | §7 |
| What does a displaced blend's result denote? | §7.3 |
| What is the second increment? | §8 |
| How does a cap-loop `Chamfer` of exact loops dispatch on a recorded receiver? | §8.1 |
| Which `BuildChecked` gates read non-selected loops, and how do they transfer? | §8.3 |
| What does `capblend.DisplacedPayload` charge, and who declines it? | §8.4 to §8.8 |

### 1. Problem

Every prism modify op refuses a receiver with a nonzero `SectionDelta`
(`prismrecord.Payload.RequireExactSection`, prism-boolean §8 row 3, brep-modify
Table RB row RB6). Every analytic `Union` that cuts an edge carries
`δ_cut > 0` (prism-boolean §7). The input class this design serves is a disc
(a `CircleSeg`, with holes bounded by lines and arcs) united on its own sketch
plane with all-line teeth (one tooth and its frame-keeping `PatternCopies`
instances), then a concave `Fillet` of every root, then a `Cut` by a bore tool
lying inside the disc, then a `Chamfer` of the hole rims on both caps. The
motivating part has several windows (each an inner arc, an outer arc, two lines and
four sketch-fillet arcs), many teeth (a tooth and its copies), a fillet of every root and a
D bore; it stays in the consumer repository (`agent-docs/testing.md`, Test
scope). Without a carrier record:

1. A hole-free disc united with one tooth takes the pair route (prism-boolean
   G6 refuses a holed operand there) and records the disc as
   a trimmed `CircleSeg` and the flanks as narrowed `LineSeg`s. Its
   `SectionDelta` is cut-parameter rounding, so `Fillet` of
   the root edges refuses.
2. The disc united with the tooth and its copies misses every analytic route.
   Each copy is displaced (mirror-pattern §6.2 charges its rounding). The
   one-scene route (prism-boolean §4.6) takes only undisplaced, hole-free,
   whole-line prisms, and the pair fold stops once the disc circle is trimmed
   (prism-boolean §4.1). The set falls to the mesh path and its edges are
   `FacetedCurve`s.
3. A count of 256 densified segments per circle and per arc (the count before
   §4's per-entity rule) puts the wheel's union and its bore `Cut` after the
   fillet over `MaxArrangementSegments = 4096`.
4. The windows make the disc a holed operand, and the cap-loop chamfer
   (`capBlendPayload`, reach §8.3) has no displacement term.

The displacement in case 1 is cut ends sliding along exact carriers. In case 2
the copies' own carriers also moved by their motion's rounding. Every op stays
sound against the section each body DENOTES; this design never redefines a
denotation to fit a record.

### 2. Approach

A displaced prism may carry a **carrier record**: per segment and per junction,
a bound on how far the denoted section's carriers and junctions sit from the
recorded ones, with the denoted section proven to have the recorded section's
structure. A lateral `Fillet` or `Chamfer` reads it, builds from the recorded
carriers, holds a margin on every decision the displacement could flip, and
charges what it cannot remove. Constructions that can prove a record set one:
a frame-keeping pattern instance, the pair `Union` of two undisplaced operands,
an extended one-scene `Union`, a clean-nesting `Cut`, and the blends
themselves.

#### 2.1 Increments

**PR 1** (§3 to §7, §9, the PR-1 rows of §10): the carrier record and Lemma H;
the per-entity arrangement count (§4); the frame-keeping instance record; the
pair `Union` record; the extended one-scene `Union`; the clean-nesting `Cut`
record; the arrangement stability audit; the lateral carrier route of `Fillet`
and `Chamfer`; and `sectionaudit.Nesting`'s candidate points (§6.5). It builds
the wheel's union, its root fillet and its bore `Cut` with exact analytic
faces. A cap-loop `Chamfer` of a displaced receiver outside PR 2's route is refused by
`RequireExactSection`, or takes brep-modify §2.2's faceted view.

**PR 2** (§8 and the PR-2 rows of §10): the cap-loop `Chamfer` of exact loops
of a receiver with a valid record, built as a separate payload type
`capblend.DisplacedPayload` that every reader unaware of it declines. It
chamfers the wheel's window rims on both caps with exact analytic faces. PR 2
also measures a lateral blend's arc ends at their read ends (§7.7, "Arc read
ends" and "Seam snap"), and declines a mesh `Boolean`'s source tag on every
displaced wall of a `DisplacedPayload` (§8.7, rule DW; displaced prisms keep
their tags). Nothing in PR 1 depends on PR 2.

#### 2.2 Alternatives considered

- **Denote the record.** Treat a displaced union's recorded section as its own
  denotation, as a loft join's fillet does (pattern-union, Axial pattern join
  §4). That removes every claim the result makes about the operands the caller
  combined. Rejected.
- **Faceted fallback.** brep-modify §2.2's retained blend already chamfers
  exact-carrier loops of a displaced prism, but its result is a
  `facetedPayload`. Rejected: the part must keep exact analytic faces.
- **Faceted axial-section route** (local commit `a6c77827d8`): rejected by the
  user. Its test shapes are reused in §10.
- **One global carrier displacement.** A single value per payload makes every
  window loop non-exact as soon as one copy is displaced, and §8 needs exact
  loops. Rejected for per-segment values.

### 3. The carrier record

#### 3.1 Data

`prismrecord.Payload` gains `Carriers *CarrierRecord`:

```go
type CarrierRecord struct {
	SectionDelta float64             // the payload's SectionDelta when the record was made
	Profile      momentinput.Profile // a deep copy of the section the record describes
	Segment      [][]float64         // ε[l][j]: loop l (0 outer, i+1 hole i), segment j
	Junction     [][]float64         // δ[l][j]: the junction at the end of segment j of loop l
	owner        *topo.Body          // the one body the record was minted for (§3.4)
}
```

Every entry is finite and non-negative, and the shapes match the profile's
loops and segments. Junction `j` of loop `l` joins segment `j` to segment
`j+1` (mod the loop's length). Every producer stores `Profile` as a deep copy
(`sectionrecord.CloneLoopRecord` per loop), so no segment slice is shared with
the payload's own profile.

#### 3.2 The claim

**Recorded data.** Segment `j`'s recorded carrier is: for a `LineSeg`, the line
through its `Start` and `End`; for a `CircleSeg`, the circle of its `Center` and
`Radius`; for an `ArcSeg`, the circle of its `Center` and radius
`|Start − Center|`. Its **read ends** are the two points the segment DENOTES at
its ends (evaluator §4): the exact point at its recorded parameter on its
recorded carrier. For a whole `LineSeg` that is its `Start` and `End`; for a
whole `ArcSeg` it is `Start`, and the point at `Start`'s radius on `End`'s
bearing, not the recorded `End`. An `ArcSeg`'s **radial residual**
`ρ_j = | |End − Center| − |Start − Center| |` (outward-rounded square roots) is
the distance between that read end and the recorded `End`. A consumer that
reads the pinned `End` (as `pinArcWalkEnds` does) charges `ρ_j` itself, as
`loftmesh.ArcNaturalEndRadialUpper` does for the loft; a consumer that reads a
float walk charges that walk end's `proofbound.WalkEndBoundAllow`.

**Junction paths.** Where segment `j`'s read end and segment `j+1`'s read start
differ, the section closes the gap with evaluator §4's closing path: through
the two supporting lines' exact crossing where both neighbours are `LineSeg`s
and that corner is provably near, else the straight chord. A junction's
**path** is that closing path with its two ends, or the single shared point.

A valid record proves, for the payload's recorded section `S` and the section
`S*` its construction denotes:

- **CR1 — structure.** `S*` has the same loops, each with the same segments in
  the same order, of the same kinds (line where `S` has a line, circular where
  `S` has an arc or circle) and walk senses, joined by junction paths. Each
  circular arc of `S*` runs between its two ends the short way: each end's
  slide, measured as an angle on `S*`'s circle from the radial image of the
  matching read end (`C* + R*·(read end − C)/R`), is below `α_j` of Lemma H, and
  `2·α_j` is below both `σ_j` and `2π − σ_j`, with `σ_j` the recorded sweep.
- **CR2 — carriers.** A line of `S*` lies on the line through two points
  `Start*`, `End*` with `|Start − Start*| ≤ ε` and `|End − End*| ≤ ε`. A circular
  segment of `S*` lies on a circle `(C*, R*)` with `|C − C*| + |R − R*| ≤ ε`,
  where `(C, R)` is the recorded carrier.
- **CR3 — junctions.** `S`'s and `S*`'s junction paths between segments `j` and
  `j+1` have parametrizations over `[0, 1]` (each continuous and weakly
  monotone in arc length along its path, either possibly constant, so two such
  pairs chain through a shared path) that start at segment `j`'s ends and
  finish at segment `j+1`'s
  starts, with every pair of points at the same parameter within `δ[l][j]`.
  Where `S*`'s two segments meet at one point `J*` (on both of its carriers),
  this says every point of `S`'s path lies within `δ[l][j]` of `J*`. Where both
  paths are polylines (a point, a chord, or end–corner–end), matching their
  breakpoints and interpolating linearly between them gives such a pair, and
  then the largest breakpoint distance bounds `δ` (**path rule**: a chord
  matched to a corner path takes the chord point at the corner's arc-length
  fraction as its breakpoint).
- **CR4 — the bound.** Every entry, and every loop's `h_loop` (Lemma H), is at
  most `SectionDelta`. So every boundary point of `S*` lies within
  `SectionDelta` of the matched point of `S`, the displacement map
  `prismrecord.Build`'s section charges and `proofbound.SectionDisplacementArea`
  read.

With every entry zero the claim says `S*` is `S` as evaluator §4 denotes it,
closing paths included. So a payload with `SectionDelta == 0`, and a loop a
producer copies from such a payload unchanged, has a true zero record whatever
its arcs' radial residuals.

#### 3.3 Lemma H — the pointwise segment bound

For segment `j` with start junction entry `δ_s = δ[l][j−1]`, end junction entry
`δ_e = δ[l][j]`, carrier entry `ε` and recorded radius `R`, let
`δ_m = max(δ_s, δ_e)` and define `h[l][j]`:

| Segment | `h` | Condition (else `h = +∞`) |
|---|---|---|
| line | `max(δ_s, δ_e)` | none |
| whole circle | `ε` | none |
| arc (an `ArcSeg`, with `σ_j` its recorded sweep times `|TEnd − TStart|` when narrowed; or a narrowed `CircleSeg` with `σ_j = 2π·|TEnd − TStart|`) | `ε + 2·(ε + δ_m)` | `ε + δ_m ≤ 2·(R − ε)` and `2·α_j < min(σ_j, 2π − σ_j)` with `α_j = 2·(ε + δ_m)/(R − ε)` |

Every value is rounded up. The sweep test reads `σ_j` as a proven enclosure
(`circularbounds`' `Atan2Interval` enclosures of the two bearings, the same
branch correction a whole arc takes) and holds for every value in it. A
consumer that meets `h = +∞` misses or refuses as its own section says.

**Lemma H.** Under CR1–CR3 there is a homeomorphism `φ` from each recorded
segment (between its read ends) onto `S*`'s segment that takes the read start
and read end to `S*`'s two segment ends and moves every point by at most `h`.

Proof. A line: map `(1 − t)·A + t·B` to `(1 − t)·A* + t·B*`, where `A`, `B` are
the read ends and `A*`, `B*` `S*`'s segment ends; the distance is a convex
combination of `|A − A*| ≤ δ_s` and `|B − B*| ≤ δ_e` (CR3: both lie on the
junction paths). A whole circle: the radial map `P ↦ C* + R*·(P − C)/R` moves
`P` by at most `|C − C*| + |R − R*| ≤ ε`. An arc: both read ends lie on the
recorded circle. The radial map takes each read end to a point `J'` of `S*`'s
circle within `ε` of it, so `|J' − B*| ≤ ε + δ_m` for `S*`'s end `B*`. That chord
is at most `2R*` (first condition, with `R* ≥ R − ε`), so the shorter arc of
`S*`'s circle from `J'` to `B*` is at most `(π/2)·(ε + δ_m) ≤ 2·(ε + δ_m)` long,
an angle below `α_j`. CR1 says the end slides that short way. Map the point at
normalized angle `t` of the recorded arc to the point at normalized angle `t`
of `S*`'s arc: the radial part moves it by at most `ε`, and the angular part by
at most `R*` times the larger end slide, so by at most `2·(ε + δ_m)`.

**Loops.** Parametrize each segment of `S` and its image under `φ` by the same
parameter, and each junction path pair by CR3's parametrizations. Joined in loop
order, they parametrize each loop of `S` and of `S*` over one circle, `φ` on
every segment, with matched points within `h_loop[l]`, the largest `h` and `δ`
of the loop.

**Corollary W.** The straight-line homotopy `u ↦ (1 − t)·S(u) + t·S*(u)` between
the two loop parametrizations moves every point by at most `h_loop`. A point
farther than `h_loop` from a loop
has the same winding number about that loop and about its `S*` image; two loops
whose recorded distance exceeds the sum of their `h_loop` stay disjoint through
the homotopy, and each keeps its winding number about the other's points.

#### 3.4 Validity, staleness and readers

**The record fails closed.** A record is bound to the one body it was minted
for. `prismrecord.Build` always clears `Carriers`; only
`prismrecord.BuildWithRecord(ctx, env, p, rec)`, which the producers of §5
call, stores a deep copy of `rec` in the new body's payload with `owner` set to
that new body. `prismrecord.CarrierRecordOf(budget, body)` returns the record
and true only when:

- `body.Payload` is a `prismrecord.Payload` whose `Carriers.owner` is `body`;
- the payload's `SectionDelta` is bit-identical to the record's;
- the entries are well formed (finite, non-negative, at most `SectionDelta`,
  shaped like the profile) and every loop's `h_loop` is finite and at most
  `SectionDelta` (CR4);
- the payload's profile equals the record's copy loop for loop and segment for
  segment (`momentinput.ExactProfileEqual`).

A payload with `SectionDelta == 0` answers a record of zeros whatever body holds
it: it denotes its own record. Anything else answers false, which keeps today's
refusals. Every reader (§5.3's operands, §5.4's target, §7's receiver, §8's
receiver) reads through `CarrierRecordOf` on the body it was handed.

So no pass-through can carry a record. A construction that copies a payload into
its result (`prismcells.MatchCutNoOp` returning the target for a tool inside a
displaced hole, a contained `Union` returning an operand, a scale or mirror, or a
site that builds `&topo.Body{Payload: p}` directly) produces a body that is not
the record's owner, so its record claims nothing; no list of such sites needs
auditing. `Placed` re-evaluates through `prismrecord.Build` and so drops the
record too: a `Transform` or `Copy` of a recorded body carries none, and a
later blend of it refuses as today. `ExactCarriers` (prism-boolean §7) is
unchanged, and `Payload.CarrierExact` reads only it; a reader that needs a
record's exact loop asks `CarrierRecordOf(...).ExactLoop(l)`, which reports a
loop whose entries are all zero: `S*` then holds that loop of `S` unchanged,
point for point.

### 4. Arrangement cap

The pinned `sketch` (`go.mod`, `geom/arrange.go` `sampleParams`) densifies a line
to 1 segment, a whole circle to 256, and an arc to `max(2, ⌈256·sweep/(2π)⌉)`,
where `sweep = Arc.Sweep()` is `wrapSweep(atan2(Start − Center), atan2(End −
Center))` over the scene entity's own floats, a value in `(0, 2π]`. The cap
`MaxArrangementSegments = 4096` bounds the densified count of one scene, which
is what bounds sketch's quadratic pass.

**The screen.** `RegionsWithinWorkCap` counts a line as 1 and a circular segment
as `prismcells.CircularSegmentCount(seg)`:

- a whole `CircleSeg`: 256, sketch's own count for a closed circle, which reads
  no float sweep;
- an `ArcSeg`: `turn = sweep/(2π)` with
  `a0 = math.Atan2(Start.V − Center.V, Start.U − Center.U)`,
  `a1 = math.Atan2(End.V − Center.V, End.U − Center.U)`,
  `d = math.Mod(a1 − a0, 2π)`, `sweep = d` when `d > 0` else `d + 2π`: sketch's
  `wrapSweep` (`geom/geom.go`) on the record's floats, written with
  `math.Mod` as sketch writes it, not with a repeated `+2π` loop. A narrowed
  `ArcSeg` multiplies `turn` by `|TEnd − TStart|`. A narrowed `CircleSeg` takes
  `turn = |TEnd − TStart|`;
- the count is `min(257, max(2, ⌈256·(turn + 2⁻⁴⁰)⌉ + 1))`, and 257 for a
  non-finite or non-positive `turn` or one above 1.

For a whole `ArcSeg` that enters a scene unmapped (operand A, or any operand
under an identity re-expression), the scene builder creates
`CreateArc(Center, Start, End)` from the pinned walk ends (prism-boolean §4.1's
ascending-angle order puts `Start` first for either walk sense), so `sketch`
evaluates the same expressions on the same floats and the screen's count is at
least sketch's. A whole circle counts exactly sketch's 256.

**The authoritative count.** For a narrowed or mapped circular segment the
record's floats are not the floats `sketch` reads. So the scene builders
(`prismcells.BuildScene`, `prismcells.BuildSceneRegions`,
`classbgeom.CreateNaturalSegment`) sum, over the entities they create,
sketch's own densification computed from each entity's floats (the formula
above), and return `prismcells.WithWorkLimit(ErrUnsupported…)` when the sum
exceeds the cap. A caller whose own over-cap answer is a silent miss maps that
error (`errors.As` a `*prismcells.WorkLimitError`) to its miss; every other
caller returns it, as it returns its screen refusal today. The screen stays
first so a scene far over the cap is refused before any entity is built.

The per-entity count admits more scenes for every `RegionsWithinWorkCap`
caller (the stacked-union, section-history, within, pattern-assembly,
revolve-bore, revolve-plan, loft-direct, class-B, stacked-record, pair,
one-scene and group callers): a pair with arcs that used to stop at the cap
now reaches its route. A whole circle keeps its count of 256, so the
rectangular plate with fourteen circular holes and a circular tool still fits.

A set that the old per-circle count stopped at the cap (a silent miss to the mesh path, or a
`WorkLimitError` refusal) now runs its analytic route and can refuse with that
route's own typed error instead. A test covers one such caller: an arc-heavy pair
now reaches the pair route and answers that route's result or refusal.

### 5. Producers (PR 1)

Every producer below sets the record together with `SectionDelta`, sets
`SectionDelta` to at least every entry and every loop's `h_loop` (CR4), stores
a deep copy of the profile (§3.1), and sets no record when any arc of its result
has `h = +∞` (Lemma H); the extended `Union` (§5.3) misses instead, since its
proof needs every `h`. A construction not listed sets no record.

#### 5.1 Frame-keeping pattern instance

`patternassembly.FrameKeepingInstance` moves a prism's record by an in-plane
motion and charges the rounding into `SectionDelta` (mirror-pattern §4.3,
§6.2). The instance denotes the source section moved by the EXACT motion: a
rotation by exactly `i/Count` of a turn about the exact centre, or the exact
translation `t_i`. When the source has `SectionDelta == 0`, the instance gets a
record: every segment entry and every junction entry is the motion's charge
`δ_pattern` (mirror-pattern §6.2, the instance's `SectionDelta` before any raise
below), except the junction at an `ArcSeg`'s `End` in record-field order
(§7.7): for a counter-clockwise arc segment `j`'s end junction `j`, for a
clockwise one (`End` = walk start) its start junction `j − 1`. That junction
takes
`δ_End = up(3e + 4e·(R + 2e)/(R_E − 2e))` with `R` the recorded radius and
`R_E` a lower bound on the moved `|End − Center|`. The instance's
`SectionDelta` is raised to the largest entry and `h_loop` when either exceeds
the charge. An all-line source keeps
`prismrecord.UniformCarrierRecord(moved, SectionDelta)` (every line's `h` is its
larger junction entry). A source with a line–line junction whose two read ends
differ (two narrowed lines) gets no record: its closing path may run through the
supports' corner, which a motion's rounding moves by up to `e/sin θ` and whose
corner-or-chord choice it can flip.

Proof. Let `e` be the largest point charge `patternrecord.MoveRegion` takes. The
frame-keeping arm admits no trimmed arc or circle, so every circular segment is
whole. CR1: `MoveRegion` keeps every segment's kind, sense and range, and the
exact motion keeps each arc's sweep. CR2: a line's moved `Start` and `End` are
each within `e` of their exact images; a circle's centre moves by at most `e`
and its radius not at all; an arc's centre and `Start` each move by at most
`e`, so its radius `|Start − Center|` moves by at most `2e` and
`|ΔC| + |ΔR| ≤ 3e`, the arc charge `MoveRegion` takes. CR3: a read end that is
a recorded endpoint, or a narrowed line's point at a recorded parameter, lies
within `e` of its exact image. An arc's end read end is `C + R·u`, with `u` the
unit vector to `End`. Centre and radius move by at most `3e` together, and `u`
by at most `2·2e/(R_E − 2e)` (`End` and `Center` each move by at most `e`),
which gives `δ_End`. With no line–line junction of distinct read ends, every
junction path of the source and of the instance is a chord or a point, whose
ends move by at most these amounts, and the path rule (CR3) gives the entry.

#### 5.2 Pair `Union` of two undisplaced operands

The pair route's `Union` (`prismcells.ResolveAndBuildUnion`) sets a record when
both operands have `SectionDelta == 0`, the re-expression is the identity
(prism-boolean §3.4), every consumed segment is whole (both walk charges zero),
and the arrangement withdraws no span (`SceneDelta.HasSharedSpans` false: two
near-identical carriers that `sketch` merges are two walls of `S*`, which breaks
CR1). Every carrier entry is zero. A junction where both result segments end at
their natural bounds (an operand's own recorded point) takes zero; a junction
where the arrangement computed a parameter on either side takes the larger
`prismcells.CutDelta` of its two edges (`prismcells.ExactOperandsRecord`). A
trimmed arc's `h` is twice its larger junction entry, so the result's
`SectionDelta` is raised to `max_l h_loop[l]` when that exceeds prism-boolean
§7's value.

Proof. Every scene entity is built from the operand's own exact record data
(prism-boolean §4.1), and the operands denote their records. `sketch` arranged
those carriers; the arrangement's topology and its certified cut parameters are
`sketch`'s answer, which decad consumes (CLAUDE.md). So `S*` is the arrangement
of the same carriers with the same cells (CR1, CR2 with `ε = 0`). At an own
recorded point the result's junction path is the operand's own, which `S*`
keeps (zero). At a cut, `S*` passes through the true crossing `O`; each read end
is the exact point at its computed parameter, within its `δ_cut` of `O`, and the
closing path between them (a chord, or the supporting lines' corner, which is
`O` itself) stays within the larger `δ_cut` of `O` (CR3). The proof applies no
V1: with no displacement nothing moves, and it rests on `sketch`'s certified cut
parameters (`TExact`, which `sketchrecord.RecordEdge` requires) and topology.

#### 5.3 Extended one-scene `Union`

**Dispatch.** `prismboolean.UnionMany` keeps prism-boolean §4.6's class (whole
lines, hole-free, undisplaced: `lineClass`) with its behaviour and refusals.
Every other set is the **extended class** (`unionRegions`), which never refuses:
every failure is a silent miss and the set takes the next `Union` route
(`unionmany.Routes`), as A6's `AmplifiedFallback` does for a pair; cancellation
still returns its error. `StraightLoftUnionMany` converts its lofts and calls
`UnionMany`, so a converted set outside the line class reaches the extended
class too. `UnionMany` runs first in `unionmany.Routes`,
and the extended class takes no set a later route owns: the free-form routes
(`axialsection`) fail U2, the loft and revolve routes fail U1, and
`prismpattern.UnionMany`'s copied-prism join roots each copy on an arc of the
blank's own circle, which U5b misses (§10 pins that set still taking its
route).

**Admission, before the arrangement (U1–U5a), each a miss when it fails:**

| # | Condition |
|---|---|
| U1 | Every operand is a `prismPayload` with no `SurfaceResult`, on the first operand's frame and placement, with sweep intervals `prismplacement.UnionZIntervalMatches` proves equal. |
| U2 | Every segment of every loop is a `LineSeg`, `CircleSeg` or `ArcSeg` over its whole range (`prismcells.WholeSegmentRange`). |
| U3 | Every operand has `SectionDelta == 0` or a valid carrier record (`prismrecord.CarrierRecordOf` on its body, §3.4). |
| U4 | The set fits the arrangement cap (§4, screen and authoritative count). |
| U5a | No two operands' lines lie on one infinite line (`prismcells.SameLineCarrier`). |

**Arrangement.** Every operand enters `BuildSceneRegions` as one region, under
the identity, with zero walk charge (a nonzero charge is a miss). After
`ProfilesContext` and `SplitRuns`, these are misses:

- **U5b.** The arrangement withdraws a span (`prismcells.CoincidentEdgesRegions`
  reports one): two operands' near-identical carriers merged by `sketch`.
- **V1.** At an arranged vertex where entities of two operands meet, other than
  exactly two entities each passing through it at a parameter the arrangement
  computed on both of its edges (`cutAt`). A vertex where an operand's own
  recorded vertex meets another operand's entity could change its order under
  any displacement, and nothing charges it (prism-boolean §4.4's known limit).
- An unresolved cell (`prismcells.ClassifyRegions`), a crossing with no charge,
  or a stability audit failure (V2, below).

**Vertex reading** (`prismcells.ReadVertices`). Each V1 vertex `v` takes
`Δ_v = up(δ_cut(v) + err_v + L_v)`: `δ_cut(v)` the larger `CutDelta` of its two
edges, `P̂_v` A6's exact junction point (`junctionPoint`) with its error
`err_v`, and `L_v` Lemma T's distance for the two DENOTED source carriers about
`p = P̂_v` with `q = err_v + h` of each source segment (Lemma H). `L_v` is read
as §7.5 reads the centre: start with `ρ = 4·(q_e + q_f)`, read `s` over
`B(P̂_v, ρ + max ε)`, set `s* = s − θ_e − θ_f` (§6.1), and accept when
`s* > ContactEps` and `2L ≤ ρ`, else set `ρ = 4L` and retry, at most four times;
no acceptance is a crossing with no charge (a miss). When both source segments
have `h = 0`, `Δ_v = δ_cut(v)` (§5.2). `Δ_v` bounds how far each read end at `v`
lies from `S*`'s junction there.

**Stability (V2).** When any operand is displaced, §6's audit passes over every
pair of arranged edges from different operands of which at least one is
displaced. A pair from two undisplaced operands needs no audit: both carriers
are the recorded ones, and their crossings and topology are `sketch`'s (§5.2).

**Order (V3).** When any operand is displaced, along every source segment that
carries an arranged vertex with `L_v > 0` (an undisplaced disc circle cut by
displaced flanks included), with `h` that segment's own (possibly zero), each two
consecutive arranged vertices `v`, `w` (a segment's own read end counts as a
vertex with `L = err = 0`) satisfy the exact test
`|P̂_v − P̂_w| > 4·(L_v + 2·err_v + L_w + 2·err_w + 2h)`. Each denoted crossing
`c*_v` lies within `L_v + 2·err_v + h` of `φ(O_v)`, the Lemma H image of the
recorded crossing, so along the denoted segment (arc length at most `π/2` times
the chord) it lies within `(π/2)·(L_v + 2·err_v + h)` of `φ(O_v)`. `φ` keeps
order, and `φ(O_v)`, `φ(O_w)` lie more than `|P̂_v − P̂_w| − err_v − err_w − 2h`
apart, which the test makes larger than the two slides together. So the
denoted vertices keep their order, and every arranged edge maps to the piece
between its own two denoted ends. On a whole circle the vertices are cyclic, so
the test also pairs the last vertex with the first. V3 also covers the edges of
undisplaced operands that meet a displaced one: their cut vertices move by
`L_v`, and near a point where three operands' edges come close, V3 and ASA2's
apart balls together keep each vertex on its own piece.

**Selection and merge.** A cell is selected when it lies in at least one
operand. `prismcells.MergeLoopsWithChains` keeps every edge of the selected
cells counted once. Each merged loop's signed area is read as an enclosure: the
sum of `circularbounds.AreaInterval` over its segments about one anchor, widened
by the area its junction gaps can move (`internal/momentinput/junctions.go`'s
charge) and by `proofbound.SectionDisplacementArea(h_loop, walks, perimeter)`
for the result's own `h_loop`. Exactly one loop's enclosure is positive, and it
is the outer loop; every other loop's enclosure is negative and it is a hole.
An enclosure that holds zero, or more than one positive loop, is a miss.
`prismcells.AuditMergeSection` then runs S7 and S9 on the holed profile (its S8
compares only the outer loop, so hole orientation rests on this check). When any
operand is displaced, the merged profile must also pass CB7's certified contact
test (every non-adjacent pair `G > F`, with `4·max h_loop < F`), so S9's
candidate-point verdict (§6.5) rests on a certified gap; the float S7 inside
`AuditMergeSection` may still refuse.

**Charges.** The result's `SectionDelta` is the larger of
`up(max(max_i SectionDelta_i, max_v Δ_v) + δ_cut)`, prism-boolean §7's formula
with zero walk and re-expression terms, and `max_l h_loop[l]` of the result
record (CR4).

**Result record.** Each result segment takes its source segment's `ε`. A
junction where both result segments end at their natural bounds is a recorded
vertex of one operand and takes that operand's junction `δ`. A junction at a cut
vertex takes `Δ_v`. A whole circle's seam junction, where its two pieces meet,
is charged the circle's `ε`, and `err_v` includes `δ_cut`, so `Δ_v` counts
`δ_cut` twice, which is conservative.

**Claim U.** Under U1–U5b and V1–V3, `S*` is the union of the denoted operands
and satisfies CR1–CR3 with the entries above.

Proof. With no displaced operand the proof is §5.2's. Otherwise let `A` be the
arrangement of the recorded operand boundaries and `A*` that of the denoted
ones. Every operand satisfies CR1–CR3, so Lemma H maps each of its segments onto
the denoted one, moving points by at most `h`.

- *Vertices.* Each own vertex maps to the operand's denoted junction (CR3). At
  each cut vertex `v`, Lemma T (§6.1) gives exactly one crossing `c*` of the two
  denoted carriers in its ball, transversal, within `L_v` of `P̂_v`; §6.3's
  ASA2 keeps both source segments' own vertices out of the ball, so `c*` lies on
  the two denoted segments.
- *Edges.* Each arranged edge `e` (a piece of one source segment between two
  arranged vertices) maps to the piece `e*` of the denoted segment between the
  images of its end vertices, which V3 keeps in order, within `η_e` of §6.3
  pointwise. Lemma S (§6.4)
  shows two images from different operands meet exactly at the images of their
  shared cut vertices, once each. Images from one operand keep that operand's
  structure, since the denoted operand section is a valid section.
- *Rotation.* At an own vertex the cyclic order of edges is the operand's own.
  At a cut vertex the sign of the tangents' cross product is the same on `A` and
  `A*` (`s* > 0`), so the four half-edges keep their cyclic order.
- *Faces.* Each connected component of `A*` is therefore embedded as its
  component of `A`, with the same faces. Components not joined by a vertex stay
  more than `F − 2·max η > 0` apart through Corollary W's homotopy, so each keeps
  its winding number about the others, and the nesting of components is the
  same. Two loops of one operand nest in `A*` as in `A` because the denoted
  operand is a valid section with the recorded loop structure (CR1). Each face's
  membership in each operand is read across that operand's edges from their
  material side (`ClassifyRegions`), which the embedding keeps.

So the same cells are selected and `S*`'s merged loops are the images of `S`'s,
segment for segment (CR1, with each arc's end slide below `L_v` and Lemma H's
condition checked). Each result segment lies on its denoted source carrier
(CR2 with the source `ε`). Each junction is an own vertex, whose junction path
is the operand's own (the operand's `δ`), or a cut vertex, where both read ends
and the closing path between them lie within `δ_cut(v)` of the recorded crossing
and `c*` within `err_v + L_v` of that crossing (CR3 with `Δ_v`; the closing
path is a chord, or for two lines possibly the supports' corner, which is the
recorded crossing itself).
CR4 follows from the formula.

#### 5.4 Clean-nesting `Cut`

A clean-nesting `Cut` (prism-boolean §4.2, `prismcells.ResolveAndBuildCut`)
sets a record when the target body has a valid record (`CarrierRecordOf`; the
`Cut` dispatch reads it from the target body and hands it to the payload-level
resolver), the tool has `SectionDelta == 0`, the re-expression is the
identity, the tool's walk charge is zero (`sceneDelta.B == 0`), and §6's
stability audit passes over every pair of a tool segment and a target segment
with a nonzero `h`. All pairs are ASA1
pairs: a clean-nesting match shares no vertex. When any condition fails the
`Cut` builds as today, with no record.

**Mapping.** The matched profile is recorded from the private scene
(`RecordProfileContext`), which may order loops and start segments differently
from the target. The record therefore reads each result segment's entity tag
(`prismcells.ResolveCutWithTags`) for its target `(hole, segment)` or the tool's
outer loop:

- a result segment from a whole target segment records the entity's own data,
  which is the target segment's data, and takes its `ε` verbatim;
- a result segment from a narrowed target segment records a whole `LineSeg`
  through the scene's walked endpoints (prism-boolean §4.1 refuses a narrowed
  circular target first). With `a` each walked end's `WalkEndBoundAllow` and
  `δ` the target junction entry at that end, its `ε` is `max(δ + a)` over its
  two ends;
- a junction between two result segments of one target loop takes the target's
  junction `δ` (in either walk direction, `ownJunction`) plus `τ`, the path
  rule's distance (CR3) between the result's junction path and the target's
  there. Both paths are recorded data, each closed by the evaluator's own rule
  (`internal/momentinput` `lineCorner` and its 1024-chord test) computed
  exactly, so `τ` is an exact breakpoint distance; for two chords it is the
  larger walk allowance `a`. `S*`'s path enters only through the target's own
  CR3;
- every entry of the tool's loop is zero;
- the result's `SectionDelta` is raised to `max_l h_loop[l]` when that exceeds
  prism-boolean §7's value (CR4).

The tool must record its circular segments whole: a D bore drawn as a whole
`ArcSeg` and a line takes this route, while a tool holding a trimmed
`CircleSeg` is refused by `prismcells.ProfileHasTrimmedCircularSource`
(`internal/prismcells/analytic_entry.go`) and the `Cut` takes the mesh path.

Proof. `S*` is the denoted target less the exact tool. ASA1 keeps the tool more
than `F` from every displaced recorded target loop, and by Corollary W each such
loop moves by at most its `h_loop < F/4`, so the denoted target loops neither
meet the tool nor change their winding numbers about it. The result's
loops are the denoted target's loops plus the tool's loop, nested as recorded.
A re-recorded narrowed segment's two ends lie within `a` of the exact read ends,
which lie within `δ` of `S*`'s segment ends (CR2), and each result junction path
lies within `τ` of the target's, which lies within the target's `δ` of `S*`'s
(CR3, composing the two parametrizations).

**Order of operations.** The bore `Cut` follows the root `Fillet`. The union's
outer loop holds trimmed `CircleSeg`s, which prism-boolean §4.1 refuses
(`prismcells.ProfileHasTrimmedCircularSource`), so a bore `Cut` of the union
itself takes the mesh path. The fillet rewrite re-emits every walk of a filleted
loop as a whole `LineSeg` or `ArcSeg` (`offset2d.OriginalSegment`) and clones
every other loop, so the filleted part enters the `Cut` scene with whole
segments.

#### 5.5 Blends

§7.7 gives a lateral blend's result record.

### 6. Arrangement stability

#### 6.1 Lemma T — one transversal crossing

**Lemma T.** Let `γ1`, `γ2` each be a line or a circle, `p` a point,
`q1 ≥ dist(p, γ1)`, `q2 ≥ dist(p, γ2)`, and `s > 0` a lower bound on `|sin|` of
the angle between any tangent of `γ1` and any tangent of `γ2` at points of the
ball `B(p, ρ)`, with `ρ ≤ R/2` for each circle of radius `R`. Let
`L = (q1 + q2)/s + min(q1, q2)`. If `2L ≤ ρ`, then `γ1 ∩ γ2 ∩ B(p, ρ/2)` is
exactly one point `c`, `|c − p| ≤ L`, and the two curves cross there.

Proof. Say `q1 ≤ q2`, and let `Q1` be the point of `γ1` nearest `p`. Walk along
`γ1` from `Q1` by arc length `t` and let `g(t)` be the signed distance from
`γ1(t)` to `γ2`. While `γ1(t)` and its nearest point on `γ2` lie in `B(p, ρ)`,
`g` is differentiable with `|g′(t)| ≥ s`, and `g′` keeps one sign because the
tangent angle never reaches `0` or `π`. `|g(0)| ≤ q1 + q2`, so `g` has exactly one
zero with `|t| ≤ (q1 + q2)/s`, and every point walked lies within
`q1 + (q1 + q2)/s = L ≤ ρ/2` of `p`; `g` changes sign at `c`. Uniqueness: two
crossings `c ≠ c′` in `B(p, ρ/2)` are joined on each curve by an arc inside
`B(p, ρ)` (a line's segment, or a circle's shorter arc, since `ρ ≤ R/2`), and
by the mean value theorem each of those arcs has a tangent parallel to
`c′ − c`; the two tangents are then parallel, against `s > 0`.

decad computes these crossings, and V3's order, itself (the reason CLAUDE.md
asks the owning doc to state): they are statements about the DENOTED carriers,
whose data are exact images or bounds that `sketch` never sees, so there is no
`sketch` answer to consume. `sketch` still arranges the recorded carriers.

A6's distance bound is `L`; Lemma T adds existence and uniqueness. Every use
below reads `s` with `prismcells.sinLower` over the recorded carriers and
accepts only `s > sectionaudit.ContactEps`. `sinLower` answers 0 for a circle
read over a ball wider than half its radius (`internal/prismcells/crossing.go`),
so every accepted `s` meets `ρ ≤ R/2`.

**Denoted carriers.** For a denoted carrier, the tangent allowance `θ` is
`4ε/ℓ` for a line of recorded length `ℓ` (`|ΔEnd − ΔStart| ≤ 2ε` and
`|a/|a| − b/|b|| ≤ 2|a − b|/|a|`) and zero for a circle when `s` is read over the
ball widened by `ε`: the radial map takes each point of the denoted circle to a
point of the recorded circle within `ε` with the same tangent. So
`s* = s − Σθ` bounds the denoted carriers' separation.

#### 6.2 Certified gap lower bound

`sketch` answers no distance query between two arranged edges, so decad computes
the gap itself (the reason CLAUDE.md asks the owning doc to state). It does not
use `sectionaudit.segMinDist` as a lower bound: its crossing tests count only
crossings more than `segCrossEps = 1e-7` (in parameter) from a segment end, so a
true crossing near an end reads as a gap of about `1e-7·ℓ`, far above the
floor.

`sectionaudit.SegGapLower(a, b survey2d.SegmentWalk)` is a proven lower bound on
the distance between two closed walks, each read as exact data: its float ends,
centre and radius as exact rationals, an arc being the points at that radius
whose bearing lies in the walk's range from its start to its end in its sense.
An arc's two endpoints are the points at its radius on its float ends' bearings
(the float ends need not lie on the circle); each is enclosed by scaling the
end's offset from the centre by the outward-rounded ratio `R/|end − C|`, and
its distances are read from that enclosure.

1. **Meeting.** Two lines: exact orientation tests on the closed segments. A
   line and a circle, or two circles: decide whether the carriers meet by exact
   comparison of squared quantities; when they do, enclose each meeting point
   with outward-rounded square roots. The answer is 0 unless every meeting
   point is proven outside one of the two pieces (a line parameter outside
   `[0, 1]`, or a bearing outside the arc by exact orientation tests on every
   corner of the enclosure).
2. **Candidates.** Otherwise the answer is the smallest of these lower bounds:
   each endpoint's distance to the other piece; for a line and an arc, the gap
   along the perpendicular from the centre (its foot is rational) when the foot
   lies inside the segment and its bearing inside the arc; for two arcs, the
   two centre-line points of each circle (bearing along the rational direction
   `C2 − C1`), or the radial gap for concentric arcs whose ranges overlap. A
   point-to-segment distance is exact rational before one downward square root;
   a point-to-arc distance is `| |p − C| − R |` from outward-rounded enclosures
   when the bearing lies in the range, else the nearer endpoint.
3. **Undecided membership includes.** A candidate whose membership is not
   decided exactly is kept: it is the distance to a point of the extended
   curve, which can only lower the minimum.

The candidates are complete: for two disjoint pieces the distance is attained at
two points each of which is an endpoint or an interior point where the joining
segment is normal to its curve, which for lines and circles are the points
listed (`segMinDist` documents the same candidate set).

An arranged edge's walk `w_e` differs from its exact recorded edge by
`c_e`, Lemma H's `h` read with the walk's radius bound
(`SegmentWalk.RadiusBound`; zero for a `CircleSeg`) as `ε` and each end's
`WalkEndBoundAllow` as `δ`. Where an edge ends at an own vertex whose two read
ends differ, the junction path between them is boundary that no edge holds; its
exact length (both legs of a corner path, or the chord) is added to that edge's
`c_e`, so every `G` also covers it. The certified gap is
`G(e, f) = SegGapLower(w_e, w_f) − c_e − c_f`.

A scene can hold many edge pairs. Each pair first compares the exact
distance between the two walks' bounding boxes (each box from the walk's ends
and, for an arc, from the same endpoint enclosures `SegGapLower` uses and its
extreme points in range, widened by `c`): a box gap above
`F` is itself a proven lower bound and skips `SegGapLower`.

#### 6.3 The audit

The audit runs over arranged edges (`sketch`'s split edges, read from the
returned cells' boundary edges, each with its scene tag and recorded segment)
and over pairs from different operands of which at least one is displaced.

- **η.** An edge's `η_e` is Lemma H's `h` read for the edge itself, with its two
  end displacements in place of junction entries: an own-vertex end takes the
  operand's junction `δ`, a cut-vertex end takes `Δ_v` (§5.3). A line edge's
  `η_e` is its larger end displacement; a circular edge's is
  `ε + 2·(ε + larger end displacement)`, with `ε` its source segment's (read
  ends lie on the recorded circle, §3.2). A segment that the arrangement leaves
  whole (a clean-nesting `Cut`'s target and tool loops) is its own edge, and its
  `η` is its `h`.
- **Floor.** `F` is `sectionaudit.ContactEps` times the diagonal of the true
  bounding box of every operand segment (`sectionaudit.ContactFloor`). The audit
  first requires `4·max(η_e, h, c_e) < F`, over every edge, its walk allowance
  `c_e` (§6.2) and every source segment. Every `h`, `c`, `err_v` (at most `Δ_v`),
  `η`, and every circular carrier's `ε` (at most its `h`) is then below `F/4`; a
  line's `ε` enters only through `θ`.
- **ASA1 — apart.** Two edges that share no arranged vertex need
  `G(e, f) > F`.
- **ASA2 — crossing.** Two edges that share cut vertices need, at each shared
  vertex `v` (a chord and its arc share two): read `s0` over
  `B(P̂_v, 2F + err_v)` and set the trim length `ρ1 = 8F/s0`. Each walk is
  trimmed from its walked end `W_v` at `v` by arc length `ρ1` (a line shortened
  by `ρ1·(1 + 2⁻⁴⁰)`, an arc by the angle `(ρ1/R)·(1 + 2⁻⁴⁰)`). The trimmed end
  is recomputed in float; its error `κ_t` from the exact point at the trimmed
  parameter (a `WalkEndBoundAllow`-style bound) joins that walk's allowance `c`
  for every `G` read on the trimmed walk. The ball of `v` is `B_v = B(P̂_v, R_v)`
  with `R_v = ρ1·(1 + 2⁻⁴⁰) + max(w_v + c + κ_t)` over the two walks, `w_v` the
  exact distance `|W_v − P̂_v|` rounded up; it holds every exact edge point
  whose walk point the trim removes, over-trim included. Set Lemma T's
  radius `ρ_T = 2·(R_v + F)`, read `s1` over `B(P̂_v, ρ_T + F)`, and accept only
  when `s1 > ContactEps`, `5F/s1 ≤ ρ1`, and `s1 − θ_e − θ_f ≥ s1/2` (§6.1). Both
  source segments' own read ends lie farther than `R_v + F` from `P̂_v` (exact
  distance; a whole circle has none), and two shared vertices' balls are more
  than `F` apart. Two lines need nothing more. Otherwise require
  `G(e∖B, f) > F` and `G(e, f∖B) > F` over the trimmed walks; an empty remainder
  skips its test.
- **ASA3 — shared ends.** Two edges that meet at an own vertex of one operand
  belong to that operand and are not paired.

#### 6.4 Lemma S

**Lemma S.** If ASA1–ASA3 hold and each edge's denoted image lies within `η_e`
of it pointwise (Lemma H's map), two images of edges from different operands
meet only inside the balls of their shared cut vertices, and there exactly
once, transversally.

Proof. Let `η = max η < F/4`. An ASA1 pair's images are at least
`F − 2η > 0` apart. For an ASA2 pair, suppose `p*` on `e*` meets `q*` on `f*`;
their recorded preimages `p`, `q` satisfy `|p − q| ≤ 2η < F`. If `p` lay outside
every ball, its walk point (within `c` of it) would lie farther than the
trimmed length `ρ1·(1 + 2⁻⁴⁰)` from every walked end `W_v`, so it would survive
the trim and `G(e∖B, f) > F` would
be contradicted. So `p` lies in some `B_v`; likewise `q`, and since the balls
are more than `F` apart, in the same `B_v`. So `p*` lies within
`R_v + η < ρ_T/2` of `P̂_v`. Lemma T applies
about `P̂_v` with radius `ρ_T` (the ball read for `s1` is `ρ_T` widened by more
than every circle's `ε`), `q = err_v + h ≤ F/2` for each denoted carrier (`O`
lies on the recorded edge, within `err_v` of `P̂_v`, and Lemma H moves it by at
most `h`) and `s* ≥ s1/2`: `L ≤ 2F/s1 + F/2 ≤ 5F/(2·s1) ≤ ρ1/2`, so `2L ≤ ρ_T`.
The denoted carriers therefore meet in `B(P̂_v, ρ_T/2)` exactly once, which
covers every possible meeting `p*`. That crossing lies within `ρ1/2` of `P̂_v`,
while both segments' own read ends lie farther than `R_v + F` (their images
farther than `R_v`), so it lies on both denoted segments. For two
lines, `|p − q| ≥ max(|p − O|, |q − O|)·s1` holds along the whole lines
(constant tangents), so any meeting lies within `2η/s1 < ρ1/2` of `O`.

#### 6.5 Nesting candidate points

`sectionaudit.Nesting` classifies each loop by the first of its walk starts the
ray test decides: once S7 has proven no crossing or contact, every point of a
loop classifies the whole loop, and a later start is read only where the ray
test cannot decide an earlier one (a start on another loop's carrier circle, as
a window's start on a concentric window's circle). A single fixed start would
answer S9 for the wheel's union audit and for any modify audit of the windowed disc.

### 7. Lateral `Fillet` and `Chamfer` of a recorded receiver (PR 1)

#### 7.1 Dispatch

In the prism path of `Fillet` and `Chamfer` (`internal/brepmodify/fillet.go`,
`chamfer.go`), a receiver with a nonzero `SectionDelta` and a valid record
(`prismrecord.CarrierRecordOf` on the receiver body) takes the carrier route
(`brepmodify.CarrierLateralBlend`) before `RequireExactSection`, when every
selected edge matches a lateral corner (`modifyselect.MatchCorner`) and the
chamfer is symmetric. Any other selection (a cap edge, a cap loop,
`WithTangentChain` reaching a cap, an asymmetric chamfer) is not matched; a
`Chamfer` of complete cap loops then tries §8.1's route, and every other
selection keeps today's routes. A receiver with no valid record keeps today's
behaviour exactly.

- `Fillet`: a Table CB refusal is returned (`ErrUnsupported`).
- `Chamfer`: a Table CB refusal falls through to `RequireExactSection` and
  brep-modify §2.2's faceted view, exactly as for a receiver with no record, so
  no chamfer that builds today stops building. `ErrDegenerate` (S4, and S5 for
  a fillet) and cancellation are returned.

#### 7.2 Corner walks and carriers

The route resolves corner walks with `offset2d.ProfileCornerLoopsBudget`, which
merges exactly collinear line walks (`boundarywalk.ModifyLoopsBudget`;
`SideWalk.Segs` lists the merged segment indices). Each walk's entries map
through `Segs`: a merged walk whose segments or internal junctions carry any
nonzero entry, in any loop that has a selected corner, is refused (CB0), since
`S*` may turn where the record does not and `RewriteLoop` emits one segment per
walk. Otherwise its carrier entry is zero, its start junction is the entry
before its first segment and its end junction the entry of its last. Its
segments lie on one exact line (`exactlyCollinear`), and the carrier is read
from the segment next to the corner: the last for an arriving walk, the first
for a leaving one.

The construction reads carriers from the record, not from walks:
`offset2d.FilletWithRoots` and `offset2d.Chamfer` take, per walk at the
corner, a carrier override built from the recorded segment: a line through the
recorded `Start` with direction `End − Start` normalized, or a circle of the
recorded `Center` and radius (`Radius`, or `|Start − Center|` for an
`ArcSeg`). `offset2d.carrierOf`'s walk-start anchor (for a narrowed flank, the
rounded `Lerp2` cut point) is not read on this route. The turn (S4, CB1) reads
the carriers' tangents at the walked corner from the same data: a line's
normalized `End − Start`, a circle's `rot90` of the normalized
`corner − Center` in the walk's sense. Chamfer feet are set back from the walked
corner point; §7.5 charges the corner's read position.

#### 7.3 What the result denotes

The result denotes `S*_out`: `S*` with each selected corner replaced as follows.

- **Exact-carrier corner** (`ε_A = ε_B = 0`). The connector is the float
  evaluation of offset2d's closed forms on the recorded carrier data, which `S`
  and `S*` share: the arc `ArcSeg(ĉ, f̂A, f̂B)` as recorded. This is modify §5's
  convention, which brep-modify §4.4's Claim D already takes as every prism
  fillet's definition. A chamfer at a corner whose `δ_J` is also zero likewise
  takes today's float chord `LineSeg(f̂A, f̂B)`. `S*`'s two carriers end at the
  points `p_A`, `p_B` nearest the float feet, and each joins the connector by
  the chord from `p` to its foot (a junction path, §3.2).
- **Any other corner.** The exact blend of `S*`: the radius-`r` circle tangent to
  `S*`'s two carriers on the material side, centre `c*` the denoted offsets'
  crossing on the branch the construction took, feet the tangent points; or a
  chamfer whose foot on `A` lies at arc length `d` back from the end of `S*`'s
  segment `A`, and whose foot on `B` at arc length `d` forward from the start of
  `S*`'s segment `B`, joined by a chord. Each foot is measured from its own
  segment's end; where `S*` closes the corner with a junction path, there is no
  single corner point.

Exactly these entries rest on the convention and are zero: the connector's `ε`
at an exact-carrier fillet corner and at an all-zero chamfer corner. Each foot
there is charged its exact residual from its recorded carrier,
`r_f = |f̂ − p|` (rational for a line; `| |f̂ − C| − R |` from outward-rounded
enclosures for a circle), since the re-emitted wall ends on `S*`'s carrier and
the float foot does not lie on it. Every other entry charges the float
construction against the exact one (§7.5).

#### 7.4 Corners on exact carriers (Claim D′)

**Claim D′.** At a selected corner with `ε_A = ε_B = 0`, under Table CB, the
fillet of `S` at that corner is `S*_out`'s at the matching corner. Its arc
carries no displacement, and each foot carries only its residual `r_f`.

Proof. `S*` has the same two carriers (CR2 with `ε = 0`), and §7.2 builds the
offsets from them alone. The construction reads the corner point only in four
decisions: the turn's sign (S4 and the offset side), which offset root is
nearest the corner, the connector's sense, and S6's cutback comparison. Every
point of `S*`'s junction path there (both of its segment ends included) lies
within `δ_J` of the matched point of the read path (CR3), and the walked corner
within `a_J = WalkEndBoundAllow` of the read end. CB1, CB3, CB4 and CB5 make
each decision the same for every corner point within `δ_J + a_J`, so the
construction on `S*` takes the same branch and evaluates the same closed forms
on the same data.

#### 7.5 Corners with a displaced carrier or junction

Corner `J` joins walk `A` (arriving) to walk `B` (leaving), with entries `ε_A`,
`ε_B`, `δ_J`, fillet radius `r` or setback `d`. Every quantity is a float
rounded up by `proofbound` helpers, or exact over `big.Rat` rounded once.

**Offset displacement.** For a line of length `ℓ`, a point at parameter `s`
moves by at most `ε·(|1 − s| + |s|)`, and the unit normal by at most `4ε/ℓ`. Over
a ball of radius `ρ` about `ĉ`, whose projection falls `τ ≥ 0` outside the
recorded `[Start, End]`, the `r`-offset moves by at most
`e = ε·(1 + 2(ρ + τ)/ℓ) + 4rε/ℓ`. For a circle the offset circle's centre and
radius together move by at most `e = ε`.

**Centre (CB2).** Compute exactly the residuals `q_A`, `q_B` from `ĉ` to the
recorded offset carriers (outward-rounded square roots). Apply Lemma T to the
denoted offsets with `p = ĉ` and `q_i + e_i` as the two distances: start with
`ρ = 4·(q_A + q_B)` (at least `2⁻⁶⁰`); compute `e_A`, `e_B` at `ρ`; read `s` over
`B(ĉ, ρ + max ε)` and set `s* = s − θ_A − θ_B` (`θ` of §6.1). An offset
circle's radius `R ± r` is irrational for an `ArcSeg` (`R = |Start − Center|`),
so `s` is read by `prismcells.SinLowerBounds`, which takes each circle as its
exact centre and a radius enclosure `[R_lo, R_hi]`: it evaluates `sinLower`'s
bound at `R_hi` (every circle form falls as the radius grows) and its
half-radius test at `R_lo`. Then
`L = (q_A + e_A + q_B + e_B)/s* + min(q_A + e_A, q_B + e_B)`; accept when
`s* > ContactEps` and `2L ≤ ρ`, else set `ρ = 4L` and retry, at most four times.
On acceptance `c*` exists, is unique in `B(ĉ, ρ/2)`, and `D = |ĉ − c*| ≤ L`. The
float centre's own error (`|ĉ − c|`, with `c` the recorded offsets' exact
crossing) is inside `D`.

**Fillet feet.** With `ρ_f` the exact distance from the float foot `f̂` to the
exact foot of `ĉ` on the recorded carrier (rational for a line; radial and
tangential residuals from enclosures for a circle):

- line: `Δf = ρ_f + D + e + 4rε/ℓ` (projection is 1-Lipschitz, and the foot
  of `c*` on `S*`'s line is `c* − r·n*`);
- circle of recorded radius `R`, with `R_c` an exact lower bound on `|ĉ − C|`:
  `Δf = ρ_f + ε + (R + ε)·2(D + ε)/R_c`.

**Fillet arc.** `ε_arc = D + | |ArcSeg.Start − ĉ| − r |`, the second term from
outward-rounded enclosures; `ArcSeg.Start` is `f̂A` for a counter-clockwise arc
and `f̂B` for a clockwise one (§7.7).

**Chamfer feet.** Each foot is set back from its own walk's read end at the
corner (segment `A`'s read end, segment `B`'s read start), whose distance to
`S*`'s segment end there is at most `δ_J` (CR3). With `ρ_f` the exact distance
from `f̂` to the exact foot set back `d` from that read end along the recorded
carrier: a line foot
`Δf = ρ_f + δ_J + 4dε/ℓ`. A circle foot with `ε = 0` slides along the exact
circle by at most `R·δ_J/(R − δ_J)` (brep-modify §4.4's Claim C), so
`Δf = ρ_f + R·δ_J/(R − δ_J)`; with `ε > 0` and recorded radius `R ≥ 2ε` (else
CB0), `Δf = ρ_f + ε + (R + ε)·(π(δ_J + ε)/R + dε/(R(R − ε)))`.

#### 7.6 Decision margins (Table CB)

Each row only refuses: failing it refuses with `ErrUnsupported`, naming the row,
the corner and the numbers (§9). `a_J` is the walked corner's
`WalkEndBoundAllow`. `θ` at the corner is `4ε/ℓ` for a line and
`2(δ_J + a_J + ε)/(R − ε)` for a circle: the tangent at `S*`'s corner equals the
recorded circle's tangent at the radial image of that corner, which lies within
`δ_J + a_J + ε` of the walked corner.

| CB | Condition |
|---|---|
| CB0 | A displaced walk at the corner covers several recorded segments (§7.2), a carrier is not a finite line or circle, or a displaced circle has `R < 2ε`. |
| CB1 | S4 turn: `|cross| > tol + θ_A + θ_B + 2⁻⁴⁸` (brep-modify PD5a with these `θ`; `2⁻⁴⁸` covers the float tangents of §7.2). |
| CB2 | Fillet: §7.5's centre acceptance at the chosen root and, where a second distinct root exists, at that root; skipped when `ε_A = ε_B = 0`. |
| CB3 | Fillet root: of two distinct roots at distances `d₀ < d₁` from the walked corner, `d₁ − d₀ > 2(δ_J + a_J) + D₀ + D₁ + 2⁻⁴⁸·(d₀ + d₁)` (PD5b), with `D₀ = D₁ = 0` under Claim D′. |
| CB4 | Fillet sense: `|rot90(f̂A − ĉ)·t_A| > Δf_A + D + r·(θ_A + 2⁻⁴⁸)` (PD5c); under Claim D′, `D = 0` and `Δf_A = r_f`. |
| CB5 | S6: every selected corner's cutbacks are widened by `2·T(δ′)`, `δ′ = max(δ_J + a_J, Δf_A, Δf_B, ε)` (under Claim D′, `Δf = r_f`), and every unselected junction with `δ > 0` at the end of a walk a selected corner trims widens that walk by `2·T(δ + a)`; `T(x) = x` on a line walk and `R_lo·x/(R_lo − x)` on a circular walk with `R_lo = R − ε` (PD6). |
| CB6 | S5 with displacement: an inward circular carrier needs `R − r > tol + ε + D`. |
| CB7 | Audit transfer, with `h_loop[l]` Lemma H's loop bound of the result record and `F_out` the rewritten section's contact floor (`sectionaudit.ContactFloor`): `4·max_l h_loop[l] < F_out`; every pair of non-adjacent segments of the rewritten section, pairs between kept (cloned) loops and rewritten loops included, has certified gap `G > F_out` (§6.2, `SegGapLower` with the bounding-box prefilter); and every rewritten loop's `|signed area|` exceeds `proofbound.SectionDisplacementArea(h_loop[l], walks, perimeter)`. |

CB5 holds because S6 compares a walk's remaining length with the cutbacks that
claim it. On `S*` each foot or end lies within its `δ′` of the recorded one, and
its position along the walk moves by at most `T(δ′)`; the carrier's own move
shifts positions on a circle by at most `T(ε)`. CB7 proves contact-freedom of
the rewritten section with the certified gap, not with S7's float
`segMinDist`, so it does not rest on that test's near-end crossing gap (§6.2).
A pair certified farther apart than `F_out` stays more than `F_out/2` apart under
a pointwise move of at most `h_loop` each (Lemma H, Corollary W), so no contact
appears on `S*_out`, nesting does not change (§6.5's candidate-point argument
then rests on this certified verdict), and no loop's area can change sign. Each
`h_loop[l]` is at most `SectionDelta_out` (§7.7); the row reads each loop's own
bound. Modify §5's own S7, used by every other modify op, keeps its float test and
its near-end gap (`agent-docs/missing-features.md`).

#### 7.7 Result record

`offset2d.RewriteLoop` emits, per loop with a blend, one segment per walk and
then the connector of the corner at that walk's end; a loop without a blend is
cloned and keeps its entries. Per emitted segment:

- A walk with no blend at either end whose single source segment is whole is
  re-emitted with its own data (`OriginalSegment` reproduces the pinned ends);
  it keeps its `ε`.
- Any other walk is re-emitted as a whole segment through its two float ends.
  Each emitted end's displacement `d` is `Δf` at a foot of a displaced corner,
  `r_f` at a foot of a convention corner (§7.3), or, at an untouched end, the
  receiver's junction `δ` plus the exact distance from the emitted read end to
  the receiver's read end there. A line takes `ε = max(d_s, d_e)`.
- **Arcs in record-field order.** `offset2d.OriginalSegment` and the fillet
  connector both record through `offset2d.ArcSegment`, which stores a clockwise
  walk reversed: `Start` = walk end, `End` = walk start, `TStart, TEnd = 1, 0`.
  So every arc rule below reads the ends in the `ArcSeg`'s field order, not the
  walk's. Let `d_Start` and `d_End` be the displacements of the ends stored in
  `ArcSeg.Start` and `ArcSeg.End`.
  - A re-emitted circular walk keeps its recorded centre and takes
    `ε = 2·ε_src + d_Start` (its new radius is `|ArcSeg.Start − C|`, within
    `d_Start + ε_src` of `R*`, and its centre within `ε_src`).
  - A fillet arc takes `ε_arc = D + | |ArcSeg.Start − ĉ| − r |` (§7.5; zero under
    Claim D′).
  - An `ArcSeg`'s read end on `ArcSeg.End`'s side is `ArcSeg.Start`'s radius on
    `ArcSeg.End`'s bearing (§3.2), so that side's displacement is
    `d_End + ρ`, with `ρ` the new segment's radial residual. For a clockwise walk
    that side is the walk's start.
- A chamfer connector `LineSeg(f̂A, f̂B)` takes `max(Δf_A, Δf_B)` (zero at an
  all-zero corner).
- A junction at a selected corner's foot takes the larger of the two adjacent
  segments' displacements there (including any `ρ` above). Every untouched
  junction takes the receiver's junction `δ` plus `τ`, the path rule's exact
  distance (CR3) between the new junction path, built from the emitted read
  ends, and the receiver's path there, each closed by the evaluator's own rule
  as §5.4 states: for two chords `τ` is the larger read-end distance, and an
  untouched line–line junction whose emitted read ends differ may close through
  the supports' corner. Both paths are recorded data, so no margin on the
  1024-chord test is needed; `S*`'s path enters only through the receiver's
  CR3.

**Arc read ends.** Every displacement of an emitted `ArcSeg`'s end is measured at
its read end (§3.2), never at the pinned field. On `ArcSeg.Start`'s side the
read end is the `Start` field. On `ArcSeg.End`'s side it is `Start`'s radius on
`End`'s bearing, so:

- at an untouched end, the displacement is the receiver's junction `δ` plus the
  distance between the emitted read end and the receiver's read end there.
  Where the two share the centre and the `End` field (a re-emitted walk), that
  distance is `|R_emit − R_recv|`, rounded outward from enclosures of the two
  radii (`RatSqrtDown`/`RatSqrtUp` of `|Start − Center|²`, as `circularRadius`
  reads them); otherwise it is the distance between the two read ends'
  enclosures, rounded up. It is not the walk end's `WalkEndBoundAllow`, which is
  zero at a natural-bound end (`boundarywalk.WalkOf` pins it), and it is not the
  new segment's radial residual `ρ_new` alone: the receiver's own `ρ_recv` moves
  the receiver's read end as well;
- at a foot or connector end, the displacement is `Δf` (or `r_f`) plus `ρ_new`,
  as the bullet above states;
- a segment re-emitted with its own data (`unchanged`) and not snapped has the
  receiver's read ends and adds nothing.

**Seam snap.** Before the record is made, `closeSeams`
(`internal/brepmodify/carrier_blend.go`) moves a rewritten segment's walk start
onto the previous segment's walk end where the two differ only by a walk end's
rounding (a circle's seam reads `sin(2π)·R` at parameter 1). It rewrites only a
natural-bound field (`seamStartAt`). Its `seamMove` records the distance each
read end of the moved segment moved, on both sides, from enclosures rounded up:
`Read` at the walk start and `ReadEnd` at the walk end. Moving a natural
`ArcSeg`'s `Start` changes its radius, so its `End`-side read end moves too;
moving a reversed `ArcSeg`'s `End` (its walk start) moves the read end that
reads `Start`'s radius on the new bearing, which carries the old residual
`ρ_old` and the new one. `seamMove.Carrier` stays how far the rewritten field
moved. `carrierRewriteRecord` adds `Read` to the moved segment's start
displacement (`emitted.sd`, a connector's included), `ReadEnd` to its end
displacement (`emitted.ed`), and `Carrier` to its `ε`. `junctionEntry` returns
the receiver's `endJ` alone only when neither neighbour moved on that side.

"Arc read ends" measures every read-end distance on the emitted segment after
the snap, the segment the record describes, and the snap's moves (`Read`,
`ReadEnd`, `Carrier`) are still added on top. A snapped segment's move is then
counted twice: once inside the post-snap distance and once through `seamMove`.
That overcharges by at most the snap's own move, a few ulps, and is sound;
`carrierRewriteRecord` needs no pre-snap profile.

`SectionDelta_out` is the larger of the largest entry and `max_l h_loop[l]`
(CR4), rounded up, and zero when every entry is zero. Kept loops (cloned, not
rewritten) count with the receiver's entries, so a displaced kept loop keeps the
result displaced. The result is a `prismPayload` with that `SectionDelta`, the record (when
nonzero), and the receiver's frame, levels, level displacements and placement.

Proof. Claim D′ or §7.5 places every connector and foot of `S*_out`, and CR3
every untouched junction. Each emitted segment runs between two read ends
within the stated displacements of `S*_out`'s junctions, on a carrier within
the stated `ε` (CR2, CR3). CB1–CB7 make the rewrite of `S*` the same rewrite
(CR1), and each arc's Lemma H condition is checked before the record is set.

### 8. Second increment (PR 2): cap-loop `Chamfer` of exact loops

#### 8.1 Admission and dispatch

The route takes a `Chamfer` whose selection is every edge of one or more
complete cap loops (reach §8.3) of a prism receiver with a nonzero
`SectionDelta` and a valid carrier record, when every selected loop is exact.
The motivating input is §1's windowed disc after the root fillet and the bore
`Cut`: its window loops are the disc's own loops, which §5.3, §7.7 and §5.4
carry with all-zero entries, and its outer loop carries the teeth's
displacement.

In `internal/brepmodify/chamfer.go` the route sits in the block that holds §7's
lateral carrier route: after route P's displaced arm (`route.Carriers`) and
before `pp.RequireExactSection`, when `pp.SectionDelta != 0`, `route.Prism ==
nil` and the receiver is not a straight `Sweep` reduction. The block reads the
record once, with `prismrecord.CarrierRecordOf(budget, b)` on the receiver body
`b`, and hands it to both routes. A record that is missing, stale, or owned by
another body (a `Transform` or `Copy` of the receiver, §3.4) answers false, and
both routes are skipped. The lateral route runs first, unchanged, and keeps its
`asym == nil` condition. The cap-loop route (`brepmodify.CarrierCapLoopChamfer`)
runs when the lateral route did not match or returned a `*CarrierRefusal` (a
Table CB refusal, which a `Chamfer` lets through), for a symmetric or an
asymmetric chamfer (reach §8.3.1):

1. A single straight cap edge (`straightPrismCapEdge`) does not match.
2. `ClassifyChamferSelection(ctx, pp, CapsOf(b), sel, edges)` runs. An error
   other than cancellation, or a lateral answer, does not match.
3. Every loop in `startLoops ∪ endLoops` must satisfy `cr.ExactLoop(l)`. Any
   other selected loop does not match.
4. The setbacks are the exact path's: one `capblend.Setback` on both caps, or
   `asym.CapSetbacks`. An error other than cancellation does not match.
5. `LoopDelta[l] = cr.LoopH(l)` for every loop `l`, in the index space the
   record and `capblend.Payload.Loops` share (0 the outer loop, `i+1` hole `i`).
   It is zero on every selected loop, and finite and at most `SectionDelta` on
   every loop (CR4, which `CarrierRecordOf` checks).
6. `capblend.BuildCheckedDisplaced(ctx, env, pp, start, end, startLoops,
   endLoops, loopDelta)` runs `BuildChecked`'s gates with §8.3's transfer
   condition and builds §8.4's payload through the shared builder.

A route that does not match continues to `RequireExactSection` and brep-modify
§2.2's faceted view exactly as today. Of the route's own refusals only
cancellation is returned. Every other refusal falls through to the faceted
view: each `ErrUnsupported` gate, both offset audits (their verdicts speak about
`S`, not `S*`), §8.3's transfer condition, and SX6's `ErrDegenerate`
(`capband.ChamferDropError` of `capblend.MixedOffsetProfile`). SX6 reads only
selected loops, so it does say the denoted chamfer does not exist; it falls
through all the same, so the route changes no answer except by building an
exact body, and the faceted view gives that selection its own answer as today.
So no chamfer that builds today stops building, and `BuildCheckedDisplaced`
needs no marker to tell SX6 from the audits' S8/S9 `ErrDegenerate`.

#### 8.2 What the result denotes

Let `S` be the receiver's recorded section, `S*` the section it denotes, and
`B(Σ)` reach §8.3's cap-loop chamfer of a section `Σ` with the receiver's frame,
placement, levels and level displacements, the selected loops and the setbacks.
The result denotes `B(S*)`.

**Claim E.** Under §8.1 and §8.3's transfer condition, `B(S*)` passes every gate
`B(S)` passes. Its selected loops, with their bands, cap contours and trimmed
walls, are `B(S)`'s. Each other loop `l` of `B(S*)` is `S*`'s loop `l`, whose
walls run the whole sweep, and Lemma H's loop map moves every point of the
recorded loop `l` onto it by at most `LoopDelta[l]`.

Proof. A selected loop is exact, so `S*` holds it point for point (§3.4,
`ExactLoop`). Reach §8.3 builds a selected loop's band, cap contour and trimmed
wall from that loop, its setbacks and the two levels alone, which `B(S)` and
`B(S*)` share, so they are equal; `BandDelta`, the contour's own float
displacement, is charged as today. Every gate that reads only selected loops,
setbacks and levels (SX6, SX7's band reach, SX13 both halves, SX14) answers the
same on `S*`. A non-selected loop enters `B` only as a wall over `[Z0, Z1]`, as
a boundary of both caps, and in the two offset audits. Lemma H's "Loops"
paragraph (§3.3) parametrizes the recorded loop and `S*`'s loop over one circle
with matched points within `h_loop[l] = LoopDelta[l]`. §8.3 transfers the
audits. §8.5 to §8.7 charge `LoopDelta[l]` in every reading of loop `l`.

#### 8.3 Transfer of the gates

| Gate in `BuildChecked` | Reads non-selected loops | Transfer |
|---|---|---|
| `MixedOffsetProfile` (SX6) | resolves every loop's corner walks (`offset2d.ProfileCornerLoopsBudget`), which can only refuse, and clones the non-selected loops | none: a refusal falls through (§8.1) |
| SX7 band reach | no: levels and side setbacks | none |
| `offset2d.AuditOffsetSectionBudget(budget, pp.Profile, mixed)` | S8, S7/SX12 and S9 over every loop of `mixed` | the transfer condition on `mixed` |
| `AuditSetbackSpan` | its own audit of the span-top mixed profile, when a cross setback carries a conversion rounding | the transfer condition on that profile |
| `RequireLevelsSeparate` (SX13 axial) | no: levels only | none |
| `RequireCornerLoci` (SX14) | no: selected loops only | none |
| `Build`: `capband.BandRadius` (SX13 radial) and each band | no: selected loops only | none |

**Transfer condition** on a mixed profile `M`, run right after the audit of `M`
passes, with `F` the contact floor of `M` (`sectionaudit.ContactFloor` over
`sectionaudit.EntriesOf` of its loops):

- **T1.** `4·max_l LoopDelta[l] < F`.
- **T2.** Every two segments of `M` that share no vertex, at least one of them on
  a loop with a nonzero `LoopDelta`, have certified gap `G > F` (§6.2:
  `sectionaudit.SegGapLower` of their walks less both allowances `c_e`, after
  the bounding-box prefilter). Every segment of `M` has `4·c_e < F`. A segment
  of a selected loop's cap contour is computed, not recorded, so its `c_e` is
  §6.2's walk allowance plus that loop's contour displacement from
  `capblend.Payload.LoopContourDelta`. On `BuildChecked`'s profile the
  arguments are `(ctx, l, loop, Dc, DcDelta)` of the loop's setback, the number
  `Build` later stores as the loop's `BandDelta` and `LoopBandDelta` reads. On
  `AuditSetbackSpan`'s span-top profile they are `(ctx, l, loop, top.Dc,
  DcDelta)`, with `top` from `setbackSpanTop`. That profile is the held offset
  at `top.Dc`, and the family argument below covers every smaller offset, so a
  zero span would suffice; keeping `DcDelta` encloses the contour over a wider
  span of offsets and is conservative. The term
  bounds the contour `MixedOffsetProfile` builds: both it and
  `capband.LoopContourDelta` offset the loop by `offset2d.BuildLoop(walks, 1,
  d, offset2d.ShellTol)`, and `capband.ContourDisplacement` bounds every point
  of that construction against the exact offset. The two read their walks
  separately (`offset2d.ProfileCornerLoopsBudget` against `capband.LoopWalks`),
  so T2 checks that each selected loop's two walk lists name the same segments
  and refuses otherwise.
- **T3.** Every loop with a nonzero `LoopDelta` has an area enclosure
  `prismcells.LoopAreaEnclosure(budget, loop, LoopDelta[l])` that excludes zero.

T2 is `prismcells.CertifiedContact`'s test with a per-loop displacement:
`CertifiedContactLoops(budget, loops, hLoop, extra []float64)` checks T1
against the largest `hLoop`, adds `extra[l]` to the `c_e` of every segment of
loop `l`, and skips a pair whose two loops both carry zero `hLoop`. Its
`4·c_e < F` check runs on every segment, those of zero-displacement loops
included: whenever some loop is displaced, every segment of every other loop
enters a tested pair with it (two loops' segments share no vertex), so a
narrower check would skip no segment that a tested pair reads.
`CertifiedContact(budget, loops, hMax)` stays CB7's and §5.3's call, with every
loop at `hMax` and no extra. T3 calls `prismcells.LoopAreaEnclosure` directly,
as `carrierAuditTransfer`'s area loop does (`capblend` cannot import
`brepmodify`). A failure falls through (§8.1).

Proof. A pair from two exact loops is the same pair on `S*`, so the audit's
verdict holds as it does for an exact receiver; its float S7 limit is that of
every modify audit (`agent-docs/missing-features.md`). A pair with a displaced
loop is certified more than `F` apart, and each of its segments moves by at most
`LoopDelta < F/4` (Corollary W), so on `S*` it stays more than `F/2` apart: no
crossing or contact appears (S7, SX12), every loop keeps its winding number
about every other (S9), and S9's candidate-point verdict (§6.5) rests on this
certified gap. S8 compares each mixed loop's orientation with the receiver's;
a non-selected loop is the receiver's own loop, so S8 passes on it trivially.
T3's job is the build: `Build` orients each loop's walls and signs its slab and
cap terms from the recorded loop (outer loop positive, holes negative, walls
facing out of the material), and T3 proves `S*`'s loop has the same orientation,
so those signs and wall orientations hold for `B(S*)`. SX12 audits the whole
offset family by its member at the
full setback: a selected loop's offset at fraction `s ≤ 1` is the same on `S`
and `S*`, and its distance to any fixed feature does not grow with `s` (reach
§8.3). At `s = 1` its distance to a displaced loop of `S*` exceeds
`G − LoopDelta > 3F/4`, so no member meets that loop.

#### 8.4 The displaced payload

```go
type DisplacedPayload struct {
	Base      Payload   // the cap-loop chamfer record, as built today
	LoopDelta []float64 // per loop: the receiver record's h_loop, zero on every selected loop
}
```

**The type is the guard.** `Base` is a named field, never embedded, so no method
of `Payload` is promoted. `DisplacedPayload` has exactly these exported methods:
`Transform` and `Placed` (`topo.Payload`), `SurveyClass` (answering
`"displacedCapBlendPayload"`), `ExtentAlong` and `ExtentBoundedAlong` (§8.6). A
type switch or assertion on `capblend.Payload` never matches it, and each reader
not named in §8.8 gives its existing answer for a payload class it does not
know. An interface reader finds only those five methods: `extent.ThroughAll`
meets the refusing `ExtentAlong`, and no reader finds `AxialDelta`,
`DenotedCapLevels` or `ProvesSimple`. An unknown payload's existing answer is
not always a refusal: `booloperand.RequireVolumeProof`'s default arm admits it.
That reader fails closed only because §8.8 names it, and so does every reader
whose default answer would read the body as exact (`booleaneval.sourceGroup`,
rule DW).

**Construction.** `capblend.Build(ctx, env, cbp)` and `capblend.BuildDisplaced(ctx,
env, cbp, loopDelta)` share one builder. It refuses with `ErrUnsupported` a
`loopDelta` whose length is not the loop count, an entry that is negative, not
finite or nonzero on a selected loop, and a base with `Draft`, `Fillet` or
`LoftSource` set. With every entry zero it stores a plain `Payload`: `S*` then
equals `S` on every loop. Otherwise the body's payload is the
`DisplacedPayload`, whose `Base` carries `Patches` and `BandDelta` as today.
`LoopDelta` is stored once, and every reader charges the stored number, as every
reader of `BandDelta` does (`LoopBandDelta`).

**Placement.** `Placed` sets `Base.Xform` to the composed motion and rebuilds
through `BuildDisplaced` with the same `LoopDelta`. `LoopDelta` bounds a
plane-local displacement of the record, and the record moves with the body, so
a placement keeps it, as a prism's placement keeps `SectionDelta`. §3.4's
carrier record differs: `Placed` drops it because §7 and §8 admit a route on it,
while every reader of `LoopDelta` only charges it.

**Later modify ops.** `capblend.RequireNotReceiver` (SX10 in `Fillet`, `Chamfer`
and `Shell`) matches a `DisplacedPayload` as well as a `Payload`, so a later
modify op refuses with SX10's reason.

#### 8.5 Measurements

The shared builder charges each loop `l` with `LoopDelta[l] > 0` (never a
selected loop) as `prismrecord.Build` charges `SectionDelta`. With
`n_l = len(loop.Segments)`:

- **Walls, junction vertices and wall lengths.** That loop's
  `prismrecord.BuildLoopSidesAs` call takes `ppFor.SectionDelta = LoopDelta[l]`,
  which charges every walk length `SectionDisplacementLength(LoopDelta[l], 1)`,
  every rim vertex `LoopDelta[l]` and every side face area through its walk
  length.
- **Perimeter.** `P_l` is the upper end of the `loopLen` that call returns.
- **Enclosed area.** `A_l = proofbound.SectionDisplacementArea(LoopDelta[l], n_l,
  P_l)` is added to `loopArea.Bound` before the slab multiply, so `BoundedMul`
  carries it into the slab volume (times the straight height) and into the slab's
  axial moment (times the height and `zMid`).
- **In-plane first moments.** Before the height multiply, `loopMu.Bound` and
  `loopMv.Bound` each take `A_l·(M_l + LoopDelta[l])`, with `M_l` from
  `momentinput.CoordinateEnvelope(momentinput.Profile{Outer: loop}, ...)`.
- **Caps.** Each cap's area bound takes `A_l` once: a non-selected loop bounds
  both caps with its own boundary.
- **Box and centroid.** The box is §8.6's. The centroid's formula bound reads the
  charged moments and volume, and its geometry bound reads the charged box.

Every value is rounded up. Proof. The two sections' symmetric difference lies
in the union over `l` of the `LoopDelta[l]`-neighbourhoods of the recorded
loops (Corollary W: a point farther than `LoopDelta[l]` from loop `l` keeps its
winding number about it), which `SectionDisplacementArea` bounds per loop. A
first moment differs by at most the area of that difference times the largest
`|u|` or `|v|` in it, at most `M_l + LoopDelta[l]` (`M_l` bounds `|u| + |v|` on
the loop). A wall's walk ends and carrier lie within `LoopDelta[l]` of `S*`'s
(Lemma H), which `SectionDisplacementLength` and the rim vertex bounds charge.

#### 8.6 Extent and box

`DisplacedPayload.ExtentBoundedAlong(ctx, g, work)` is the base reading with,
on each loop with a nonzero `LoopDelta`, `ProductUpper(inPlane, LoopDelta[l])`
added to that loop's `planeAllow` in both of its `take` calls. `inPlane` is the
reading's own upward-rounded in-plane norm of `g`, the factor it already applies
to a contour's displacement. Contour candidates belong to selected loops and
take nothing. The `coordUpper` that the reading hands
`prismrecord.PlacementCoeffAllow` is widened by `max_l LoopDelta[l]`, since a
denoted coordinate can exceed the recorded envelope by that much.
`DisplacedPayload.ExtentAlong` refuses with `ErrUnsupported`, as
`prismrecord.Payload.ExtentAlong` refuses a nonzero `SectionDelta`, so a
through-all stop with this body live refuses as it does for a displaced prism
(prism-boolean §12). `capblend.Bounds` reads the displaced reading on each axis,
so each displaced loop's candidates carry their own `LoopDelta[l]` into the box
bound.

#### 8.7 Tessellation

`tessdispatch.Body` takes `capblend.DisplacedPayload` beside the plain payload,
through `capblend.TessellateDisplacedMesh`, which shares `TessellateMesh`'s core
with a `loopDelta` argument (nil for the plain payload). With `D = max_l
LoopDelta[l]`:

- **Reservation.** Before any chord, as `internal/prismtess` reserves
  `SectionDelta` (tessellation §5): `chord' =
  freeform.DownRound(freeform.DownRound(chord − D))`. `chord' ≤ 0` refuses with
  `ErrUnsupported` and prismtess's message; otherwise every walk is chorded
  against `chord'`.
- **Facet bounds.** The term each side face of loop `l` bumps into `faceExtra`
  (its sagitta plus the axial displacement) adds `LoopDelta[l]`, and each cap's
  term (trim, contour displacement and axial displacement) adds `D`, so
  `tessellation.ComposeCapBlendBounds` charges them into every facet bound there.
- **Area slack.** Per displaced loop, with `H` the wall height the wall slack
  reads (`|Z1 − Z0|` for a loop chamfered on neither cap), `P'_l` the upward sum
  of `Length + LengthBound` over the loop's chorded walks and `A'_l =
  SectionDisplacementArea(LoopDelta[l], n_l, P'_l)`: `ProductUpper(
  SectionDisplacementLength(LoopDelta[l], n_l), H)` for the walls and `A'_l` for
  each cap.
- **Occupied volume.** Where the proof is built (`proveVolume`, and
  `OccupiedVolumeAdmission(dp.Base)` admits), `terms` takes
  `Σ_l ProductUpper(A'_l, H_up)`, with `H_up` the sweep height `|Z1 − Z0|`
  rounded up plus `Z0Delta + Z1Delta`, rounded up. Where the admission refuses,
  `SymDiffOK` stays false, as today.

`booloperand.RequireVolumeProof` takes the plain payload's arm on `dp.Base`, so
it and `meshbool.OperandSymDiff` keep naming the same classes: a body the
admission passes composes in a mesh `Boolean`, and any other refuses before it
is meshed. `booloperand.sectionDisplacement`, which sets an operand's chord
floor (`ChordShare`), answers `max_l LoopDelta[l]` for a `DisplacedPayload`.

**Mesh Boolean source tags (rule DW, displaced walls).** A mesh `Boolean` copies
each operand face's surface onto its facets as a retained tag
(`booleaneval.sourceGroup`, `facetedrecord.SourceTag`), and `SourceTag` reads the
face alone. A side wall of a displaced loop has a recorded `Plane` or `Cylinder`
that lies only within the loop's bound of the denoted wall, both offset and
tilted (a line wall of length `ℓ` by up to about `2·b/ℓ`). Tagging it as
retained with `NormalBound` 0 breaks faceted-source-retention Claim R, and an
`AxialDelta` would not repair it: `facetedrecord` publishes the tag's normal
with no bound, and `MirrorFace` and the retained-blend plane model extrapolate
`AxialDelta` as a parallel offset.

Rule DW therefore declines the tag (no surface, not retained, no `AxialDelta`)
on every displaced wall of a `DisplacedPayload`, `Plane` and `Cylinder` alike: a
`side(l, j)` wall of a loop with `LoopDelta[l] > 0`, and any other non-cap face
whose roles name no single side loop and no band patch of a selected loop (the
band-patch roles are those `Base.Patches` lists). The rule is per loop: a
`DisplacedPayload` holds no per-segment record. Every other face keeps its tag.
The rule lives in `internal/displacedface` (`FaceLoop`), which both `brepmodify`
and `booleaneval` import (`brepmodify` imports `booleaneval`).

`sourceGroup` (defined in `internal/booleaneval/eval_mesh.go`) takes no context
or budget, so the decision is made once per operand where the operand's faces
are walked and handed to `sourceGroup` as a per-face lookup. It reads only the
payload's `LoopDelta` and the faces' roles. The rule runs at both `sourceGroup`
callers: the mesh `Boolean` evaluator's operand-face loop
(`internal/booleaneval/eval.go`) and `booleaneval.View` (`view.go`), the faceted
view the retained edge blend reads. A `DisplacedPayload` operand never reaches
the evaluator's replay path, since `booloperand.ChordShareReplay`'s default arm
refuses it, so the rule needs no replay metering.

**Displaced prisms keep today's tags.** A `prismrecord.Payload` with
`SectionDelta > 0` reaches `sourceGroup`, and its walls keep
their retained tags with zero departure, which Claim R does not support for a
wall whose carrier is displaced. Neither a per-loop nor a per-segment prism arm
exists. A per-segment arm would keep a wall on an exact carrier
(record entry `ε = 0`, the reasoning of prism-boolean §7's "Exact carriers") and
decline the rest, but a fillet's feet leave ulp-sized carrier entries on the
adjacent line walls of a faceted retained blend's operand, so it declines tags
that `TestFacetedRetainedBlendConcaveArcEnd` and two other faceted-retained-blend
apitests need. The prism half stays an open gap in
`agent-docs/missing-features.md`, in the row on displaced classes that reach
`sourceGroup` with uncharged tags, beside the face-level readers outside the
mesh `Boolean` that read a displaced wall as exact.

#### 8.8 Readers

| Reader | Answer for a `DisplacedPayload` |
|---|---|
| `tessdispatch.Body` | §8.7 |
| `booloperand.RequireVolumeProof` | the plain payload's arm on `Base` (§8.7) |
| `capblend.RequireNotReceiver` | SX10's refusal (§8.4) |
| `extent.ThroughAll` | `ExtentAlong` refuses with `ErrUnsupported` (§8.6) |
| `verifyrun` wall, undercut and minimum-radius surveys | the default arm: staged, `Suspect` |
| clearance kernel (`pairkernel.newBodyGeomBudget`, `payloadExtent`) | the default arm: no model, so a pair reads `Suspect` unless its boxes decide it |
| tolerance-gate witnesses (`gatewitness.Prisms`) | a new arm: `capblend.WitnessPrisms(dp.Base)` with displacement `AbsSumUpper(Base.AxialDelta(), max_l LoopDelta[l])`. Each witness point lies within that displacement of a point of the denoted body: its loop moves at most `LoopDelta[l]` in the plane, its level at most the axial term |
| tolerance-gate diameter (`gatediameter.DirectBodyGateDiameter`, then `FallbackGateDiameter`) | no direct arm; the clearance kernel builds no model, so `FallbackGateDiameter` reads the witness arm above |
| pair gate points (`gatediameter.BodyGatePoints`) | the default arm reads the witness arm above through `diameter.PrismGatePoints`, else the body's vertices with their published bounds, which carry `LoopDelta[l]` on every vertex of a displaced loop (§8.5) |
| `booloperand.sectionDisplacement` (an operand's chord floor) | `max_l LoopDelta[l]` (§8.7) |
| `booloperand.ChordShareReplay` (a replayed mesh `Boolean`) | the default arm: `ErrUnsupported` |
| mesh `Boolean` source tags and the faceted view (`booleaneval.sourceGroup`, called from the evaluator in `eval.go` and from `booleaneval.View`) | rule DW (§8.7): every wall of a loop with `LoopDelta[l] > 0`, `Plane` or `Cylinder`, keeps no tag; a displaced prism's walls keep today's tags (an open gap) |
| brep route (`brepmodify.brepModifyRecord`) | not a brep record: no route |
| `payloadclass.TypeName`, `verifypublish` | `SurveyClass` |

The packages that build `capblend.Payload` views internally (`breprecord`,
`breploop`, `brepbandmesh`, `brepfilletmesh`, `draftrecord`) never hold a
`DisplacedPayload`, and a draft view never builds one.

### 9. Refusals and fallbacks

| Where | Condition | Answer |
|---|---|---|
| §4 | the authoritative count exceeds the cap | `WithWorkLimit(ErrUnsupported)`, or the caller's silent miss |
| §5.3 | an extended-class set fails anything after U1 | silent miss to the next `Union` route |
| §5.4 | a record condition or the stability audit fails | the `Cut` builds as today, with no record |
| §7.1 | a receiver with a nonzero `SectionDelta` and no valid record | today's answer, unchanged |
| §7.6 | CB0–CB7 on `Fillet` | `ErrUnsupported`, naming Part K, the row, the corner's section point and the numbers |
| §7.6 | CB0–CB7 on `Chamfer` | today's route (`RequireExactSection`, brep-modify §2.2) |
| §7.4, §7.5 | S4 and S5 themselves | `ErrDegenerate`, before any CB row (modify §4) |
| §8.1 | a selection that is not complete cap loops, a selected loop with a nonzero entry, or no valid record | today's answer |
| §8.1, §8.3 | any refusal of the route other than cancellation: SX6, every gate, the offset audits and T1–T3 | today's route (`RequireExactSection`, brep-modify §2.2) |
| §8.4 | a malformed `LoopDelta`, or a base with `Draft`, `Fillet` or `LoftSource` | `ErrUnsupported` |
| §8.6 | `DisplacedPayload.ExtentAlong` | `ErrUnsupported` |
| §8.7 | a tolerance at or below the largest `LoopDelta` | `ErrUnsupported`, prismtess's message |

A carrier-route refusal is `ErrUnsupported` because the body exists and this
evaluator cannot prove it: a smaller displacement or a cleaner model builds.

### 10. Tests

PR 1, each beside the code it covers:

- `prismrecord`: `CarrierRecordOf` validity against a changed bit of
  `SectionDelta`, a changed segment, a stale shape, an `h_loop` above
  `SectionDelta`, and a record whose profile shares no slice with the payload;
  fail-closed ownership: `prismrecord.Build` of a recorded payload, a
  `&topo.Body{Payload: p}` copy and a `Placed` copy each answer no record;
  `ExactLoop`; Lemma H's `h` for a line, a circle, an arc and a narrowed circle, and
  `+∞` past its condition; a zero record of a windowed sketch section whose
  fillet arcs carry a radial residual of a few ulps stays valid (the read end is
  `Start`'s radius on `End`'s bearing).
- `prismcells` cap: `CircularSegmentCount` of a whole arc equals the builder's
  count from the same floats at sweeps near `0`, `π` and `2π`; the builder
  refuses a scene its recount puts over the cap; the fourteen-hole plate still
  fits; a wheel-like disc with windows and copied teeth fits.
- `prismcells` audit: Lemma T finds the crossing of two lines and of a line and a
  circle and refuses `2L > ρ`; `SegGapLower` answers 0 for two lines crossing
  `1e-8` of a length from an end (where `segMinDist` answers a positive gap) and
  stays at or below the exact distance of random line and arc pairs; the
  stability audit refuses a displaced edge `F/2` from another, refuses a shallow
  line–circle crossing that comes back within `F`, refuses a ball holding an own
  vertex, sizes `B_v` from the walked end's distance `w_v` (a walked end placed
  off `P̂_v` widens the ball), and passes the wheel-like teeth; V1, V2 and V3 each
  fail to refuse their scene when switched off; V3 pairs a whole circle's last
  vertex with the seam; an edge's `c_e` grows by the exact closing path of an
  own junction whose read ends differ; the stability prefilter's arc box holds
  the endpoint enclosures `SegGapLower` reads; the certified contact test
  refuses a web between two loops and a near-end crossing.
- `patternassembly`: a 17th-turn instance of a tooth carries a record whose
  entries equal its `SectionDelta`; an instance of a source with arcs carries
  `δ_End` at each arc end and a `SectionDelta` at least its `h_loop`; an
  instance of a source holding a clockwise arc (a hole-loop arc) carries
  `δ_End` at that arc's start junction `j − 1`, its `ArcSeg.End` in field order;
  an instance of a source with two narrowed lines meeting at distinct read ends
  carries none; an instance of a displaced source carries none.
- `prismcells` cap callers: an arc-heavy pair reaches the pair route and answers its result or its own typed
  refusal.
- `prismboolean`: the extended `Union` of a disc with two windows and three teeth
  (one rotated copy) keeps two holes and the teeth, matches the volume of the
  same part drawn in one sketch within both bounds, and gives the windows
  all-zero entries; a triangle with a valid record whose own vertex lies on the
  disc circle misses (V1; the guard-disabled demonstration is in `prismcells`);
  a displaced block crossing a square's side 5e-5 from its corner misses (V2)
  and builds with V2 switched off; a near-coincident copy line misses (sketch
  merges it first); two copy flanks crossing one segment
  so close that their denoted crossings could swap miss (V3), including on an
  undisplaced circle; a holed rectangular plate united with a block and its
  translation-pattern copies (not disc-shaped) builds with a record; a holed
  loft set reaches the extended class through `StraightLoftUnionMany`; a
  copied-prism circular join still takes `prismpattern.UnionMany`; the result's
  `SectionDelta` is at least every loop's `h_loop`.
- `prismcells` `Cut`: a clean-nesting `Cut` of a displaced recorded target whose
  matched profile starts its loops elsewhere maps every entry through the tags;
  a narrowed target flank gains its walk allowance; two narrowed target lines
  whose re-recorded junction closes through a corner take the exact path
  distance `τ`; a tool `F/2` from a displaced loop builds with no record; a tool
  with a trimmed `CircleSeg` takes the mesh path.
- `sectionaudit`: `Nesting` decides a loop whose first walk start lies on
  another loop's carrier circle.
- `brepmodify`: Claim D′ (case 1's two-root fillet builds, matches the same
  filleted outline drawn in one sketch within both bounds, and records zero arc
  entries and tiny positive foot entries, each the foot's residual `r_f`); a
  displaced-carrier corner charges a positive `D` below
  `1e-12` mm for a 17th-turn copy; each of CB1–CB7 refuses a constructed
  near-boundary case and is shown to fail by disabling it (CB2: a corner whose
  denoted offsets meet too shallowly for Lemma T; CB6: an inward circle with
  `R − r` just above `r`'s tolerance and below `ε + D`); a CB refusal of a
  `Chamfer` falls through to the faceted view; a lateral chamfer of a recorded
  receiver states its foot entries from §7.5; a lateral fillet of a hole-loop
  corner on a displaced receiver, whose clockwise arc walk is blended at one end
  only, records `ε = 2·ε_src + d` at the `ArcSeg.Start` end and adds `ρ` on the
  `ArcSeg.End` side (§7.7), and a deliberately walk-order computation is shown
  to undercharge it; CB7's certified contact test refuses a rewritten pair whose
  float `segMinDist` reads a near-end crossing as a gap.
- `prismcells` pass-throughs: `MatchCutNoOp` with a tool inside a displaced hole,
  and a contained `Union` returning an operand, each return a body with no
  record.

PR 1, `apitest` (public API only):

- Case 1 (a hole-free disc and one tooth) and case 2 of §1 build analytically:
  every edge is `Line3`, `Arc3` or
  `Circle3`, an edge query `ParallelTo(+Z)` with `Concave()` asserted
  `Exactly(2·teeth)` selects the roots, and `Fillet` builds.
- A wheel-like part: a disc with two windows (each an inner and outer arc, two
  lines and four sketch-fillet arcs), a tooth and two rotated `PatternCopies`,
  the `Union`, the root `Fillet` through `Concave().Exactly(6)`, and a D bore
  `Cut` whose tool records its arc as a whole `ArcSeg`, with every edge
  analytic, a `VerifyAll` mesh whose boundary and volume verify, and a volume
  that matches the same outline drawn in place within both bounds.
- The faceted-loft attempt's shapes (commit `a6c77827d8`) where they fit the
  admitted class.

PR 2, each beside the code it covers. The twin of a displaced build is the same
`capblend.Payload` built with every `LoopDelta` zero.

- `capblend` build: a holed square receiver whose outer loop takes a positive
  `LoopDelta` and whose exact hole is chamfered on both caps builds a
  `DisplacedPayload` storing that `LoopDelta` and zero on the hole. Against its
  twin it holds the same volume value; a volume bound larger by at least
  `A_0·(Z1 − Z0)`; each cap's area bound larger by at least `A_0`; an
  in-plane moment bound larger by at least `A_0·M_0·(Z1 − Z0)`; and at least
  `LoopDelta[0]` on every outer-wall rim vertex. An all-zero `LoopDelta` builds
  a plain `Payload`. A wrong length, a nonzero entry on a selected loop and a
  non-finite entry each refuse. `Placed` keeps `LoopDelta` and answers a
  `DisplacedPayload`.
- `capblend` extent: `ExtentAlong` refuses. `ExtentBoundedAlong` along an
  in-plane axis whose extreme the outer loop holds states a bound at least
  `LoopDelta[0]` above its twin's, and along an axis whose extreme only the
  chamfered contour holds states its twin's bound. The box bound is at least
  `LoopDelta[0]`.
- `capblend` transfer: T1, T2 and T3 each refuse a constructed case and are
  shown to matter by disabling them, as Table CB's rows are. T2's case is a
  displaced outer loop certified less than `F` from a chamfered hole's contour
  while the float audit passes, and a contour segment whose gap clears `F` only
  without its contour displacement in `c_e`. A near-end crossing that
  `segMinDist` reads as a gap is not reachable through `Chamfer`, because the
  offset audit refuses the crossing first; `prismcells` tests it on
  `CertifiedContact` directly. T3's case is a displaced loop whose
  segment pairs are all adjacent, so T2 tests none of them: a thin triangle of
  height `4·LoopDelta`, whose area enclosure holds zero.
- `capblend` tessellation: a tolerance at or below `max LoopDelta` refuses.
  Above it, a `VerifyAll` mesh closes, every outer-wall facet and both caps state
  a bound of at least `LoopDelta[0]`, and `VolSymDiff` is at least
  `A'_0·H_up` above its twin's.
- `prismcells`: `CertifiedContactLoops` passes a pair of two zero-displacement
  loops closer than `F` and refuses the same pair when one loop carries a
  displacement; it refuses a zero-displacement loop's segment whose `c_e` is at
  least `F/4`; `CertifiedContact` answers as before.
- `brepmodify` arc read ends: a lateral blend of a recorded receiver whose arc's
  `End` sits a few ulps off `Start`'s radius (`ρ_recv > 0`), blended at its
  `Start` end only, records at the untouched `End`-side junction an entry of at
  least `δ + |R_emit − R_recv|`, the exact distance between the two read ends,
  checked against exact rationals.
- `brepmodify` seam snap: a snap that moves a natural `ArcSeg`'s `Start`, and one
  that moves a reversed `ArcSeg`'s `End`, each record an entry on that side of at
  least the exact distance the read end moved (`ρ_old` included for the
  reversed arc).
- `booleaneval` rule DW. Each case calls the `booleaneval` evaluator directly,
  so the mesh path runs even where a public `Cut` would take an analytic route.
  A mesh `Cut` by a box of a `DisplacedPayload` gives every facet from a
  displaced loop's `Plane` or `Cylinder` wall a group with no tag, while walls of
  exact loops, caps and selected-loop band patches keep their tags; without the
  rule the displaced walls read retained with zero departure. A displaced
  prism's walls (a disc united with a tooth and a rotated copy) keep today's
  tags. `booleaneval.View` of the same bodies decides the same faces.
- No regression: `apitest/chamfer_pattern_socket_test.go`,
  `apitest/faceted_retained_blend_cylinders_gaps_test.go` (two exact cylinders
  unioned, mesh-cut, then a retained-tag fillet or chamfer of the valley) and
  `TestFacetedRetainedBlendConcaveArcEnd` pass unchanged.
- `booloperand`: `sectionDisplacement` answers `max LoopDelta` for the result,
  and `ChordShare`'s floor is at least twice it.
- `brepmodify` dispatch: a cap-loop `Chamfer` of an exact hole loop on both caps
  of a recorded receiver builds a `DisplacedPayload`, and so does an asymmetric
  one. Each of these keeps today's answer, compared with the same call with the
  route switched off: a selected loop with a nonzero entry, a partial or mixed
  selection, a `Placed` copy of the receiver (it carries no record), a T2
  failure, and an SX6 drop (a hole loop whose setback reaches its radius), each
  of the last two falling through to the faceted view. Cancellation is returned.
- §8.8's readers: `Chamfer`, `Fillet` and `Shell` of the result refuse with
  SX10; `ThroughAll` with the result live refuses; the three surveys answer
  staged; the clearance kernel builds no model; `gatewitness.Prisms` answers
  one witness prism per loop with displacement at least `max LoopDelta` plus
  the axial term, and the tolerance-gate diameter reads it through
  `FallbackGateDiameter`; `booloperand.RequireVolumeProof` agrees with the mesh's `SymDiffOK`,
  and a mesh `Cut` of the result by a box at `VerifyAll` composes.

PR 2, `apitest` (public API only): `TestCarrierRecordWheelUnionFilletAndBore`'s
part (two windows, a tooth and two rotated `PatternCopies`, the six roots
filleted, the whole-`ArcSeg` D bore cut), then a `Chamfer` of every edge of both
windows' cap loops on both caps. `decadtest.PayloadClass` answers
`decad.displacedCapBlendPayload`; every edge is analytic
(`requireAnalyticEdges`); the volume matches `wheelInPlace(t, 2, 3, true,
true)` chamfered the same way within both bounds; and a `VerifyAll` mesh
verifies its boundary and volume. Full consumer models stay out of decad's tests
(`agent-docs/testing.md`, Test scope).

### 11. Documents to update in the same change

Each document below states one effect of this design. A change to the effect changes both places.

PR 1:

- Part B: §4.1 (the bore `Cut` order), §4.6 (the extended class and its fallback), §8 row 3, §10 (the per-entity
  count and the builder's authoritative count), §12's modify row.
- Part G A6: the readings this document defines (§5.3's vertex reading, ASA2, CB2) prove a crossing's existence and
  uniqueness through Lemma T, and a recorded union result fillets. A6 keeps its own pair reading and does not claim
  that every A6 reading has Lemma T's form.
- `agent-docs/modify-design.md` Part B Table RB row RB6, and §2.2 (a CB refusal of
  `Chamfer` falls through to it).
- `agent-docs/modify-design.md` §5 test 4 (the nesting candidate points).
- `agent-docs/mirror-pattern-design.md` §4.3 (the instance's record).
- `agent-docs/missing-features.md`: the displaced-receiver refusal, and the entry that modify §5's S7 contact test
  reads float `segMinDist`, which reports a crossing within `segCrossEps` of a segment end as a positive gap (§6.2).
  Moving S7 to `SegGapLower` is not implemented.
- `agent-docs/layout/design-docs.md` row for this document, and rows for any new
  root `.go` file.

PR 2:

- `agent-docs/modify-design.md` Part R §8.3 (`capblend.DisplacedPayload`) and
  §8.4 (its measurements and tessellation).
- `agent-docs/tessellation-design.md` §5 (the `LoopDelta` reservation).
- Part B §8 row 3 (a displaced result with a valid
  record takes a cap-loop `Chamfer` of exact loops), §12: the `Fillet` / `Chamfer` row (a
  cap-loop `Chamfer` of exact loops builds a `DisplacedPayload`) and §8.8's
  readers, and "internal/capblend contour (boundary note)" (`LoopDelta` is the
  third term beside the contour displacement and `sectionDelta`).
- `agent-docs/modify-design.md` Part B Table RB row RB6 and §2.2 (a cap-loop
  `Chamfer` of exact loops of a recorded receiver reaches the faceted view only
  through §8.1's fall-through).
- `agent-docs/modify-design.md` Part R Table SX row SX10 (it also refuses a
  `DisplacedPayload` receiver) and Table DX's survey rows DX7 to DX9 (a
  `DisplacedPayload` is staged).
- `agent-docs/tessellation-reach-design.md` §7 (T7: the `LoopDelta` reservation,
  facet charges, area slack and volume term).
- `agent-docs/faceted-source-retention-design.md` §3 and §11 (rule DW declines
  the tag of every displaced wall of a `DisplacedPayload`, per loop, at both
  `sourceGroup` callers; a displaced prism's walls keep today's tags, an open
  gap).
- `agent-docs/missing-features.md`: the displaced-receiver modify row, whose "Today" cell names the cap-loop
  `Chamfer`'s answer. Its rows for chorded loft plane normals, other displaced classes' source tags, face readers
  of displaced walls and displaced-junction convexity are open.

### 12. Decisions the user may want to overturn

- **Copies stay displaced.** A frame-keeping instance denotes the source
  section under the exact motion (a rotation by exactly `i/Count` of a turn, or
  the exact translation; mirror-pattern §6.2), and its rounded record is
  displaced from that. Every reading of a part built from it stays
  `Approximate` and declines where prism-boolean §12 says a displaced prism
  declines (the clearance kernel and the surveys read `Suspect`). Treating the
  rounded record as the instance's denotation would make those parts exact, at
  the cost of every pattern's claim about its motion.
- **The stability audit and CB7 use a certified gap** (`SegGapLower`), not the
  modify audit's float `segMinDist`; so does the extended `Union`'s contact test
  and §8.2's transfer. Modify §5's own S7 keeps its float test until the
  follow-up `agent-docs/missing-features.md` lists.
- **A record is bound to its body.** `Placed` drops it, so a `Transform` or
  `Copy` of a recorded body cannot be blended analytically until the copy is
  rebuilt by a producer. Keeping it through placement would need `Placed` to see
  the source body, which `topo.Payload.Placed` does not take.
- **A `Chamfer` CB refusal falls through** to the faceted view; a `Fillet` CB
  refusal is returned.
- **An arc-bearing instance raises its `SectionDelta`** to its largest entry
  and `h_loop` (an arc-end junction entry `δ_End` of about `7e`, since `3e + 4e·(R + 2e)/(R_E − 2e)`
  is about `7e` when `R_E` is near `R`; and through Lemma H's arc row `ε + 2·(ε + δ_m)`, with `ε = 3e` the arc
  charge and `δ_m = 7e`, an `h` of about `23e`) so its
  record satisfies CR4. Withholding the record instead would keep today's value
  and stop every arc-bearing copy from filleting. An all-line copy, such as a
  tooth, keeps its `SectionDelta`.
- **Convention-corner feet are charged** their residual `r_f`, so case 1's
  fillet carries a `SectionDelta` of a few ulps instead of zero.
- **Every §8 refusal falls through** to the faceted view, SX6's `ErrDegenerate`
  included; only cancellation is returned. Returning §8's refusals would name its
  limit, at the cost of window chamfers that build faceted today refusing, and
  returning SX6 alone would change today's answer for a dropping hole loop.
- **`DisplacedPayload.ExtentAlong` refuses**, as a displaced prism's does, though
  its bounded reading already charges `LoopDelta`. The cost: a `ThroughAll` or
  `ThroughAllSide` extrude refuses while the chamfered part is live. Returning
  the bounded reading would let the stop charge it instead.
- **Rule DW declines a `DisplacedPayload`'s displaced walls** instead of charging
  them as an `AxialDelta`. A later retained edge blend or plane reading at such
  a wall then has no retained surface there and refuses; charging it would need
  a bounded tag normal that `facetedrecord` does not publish. The rule is per
  loop, which declines some exact-carrier walls of a displaced loop; a
  per-segment rule would need the payload to carry the receiver's carrier
  entries.
- **Displaced prisms keep their tags.** Declining them, per loop or
  per segment, would make three faceted-retained-blend apitests refuse
  (`TestFacetedRetainedBlendConcaveArcEnd` among them: a fillet's feet leave
  ulp-sized carrier entries on the adjacent line walls). The cost: a displaced
  prism's wall with a displaced carrier still enters a mesh `Boolean` as a
  retained tag with zero departure, which Claim R does not support. The remedy
  is a retained edge blend that accepts a bounded tag, so such a wall can be
  tagged with its displacement instead of being declined.
- **The tolerance gate reads witness prisms** (`capblend.WitnessPrisms` with the
  axial term plus `max LoopDelta`) rather than declining, so a displaced
  cap-loop chamfer gets a gate diameter, shrunk by twice that displacement.
- **The displaced cap-loop chamfer keeps a mesh volume proof.** Its tessellation
  charges the `LoopDelta` tube and `booloperand` admits it as it admits a plain
  cap-loop chamfer, so it composes in a mesh `Boolean` as a displaced prism
  does. Declining the proof would drop §8.7's volume term and the `booloperand`
  arm, and a later mesh `Cut` of the chamfered part would refuse.
- **Placement keeps `LoopDelta`**, while it drops §3.4's carrier record: no route
  admits on `LoopDelta`.
- **The transfer skips pairs of two exact loops.** Such a pair is the same on
  `S*`, and the float audit decides it as for an exact receiver. Certifying it
  too would close modify §5's S7 gap (`agent-docs/missing-features.md`) on this
  route alone, at the cost of refusing pairs the float audit admits.
- **The extended one-scene class never refuses.** A displaced or holed set it
  cannot prove takes the next route, ending at the mesh path or the pair fold, as
  A6 does for pairs. A caller who wants to know why reads the result's kind.

## Part P — Pattern union

One part per pattern-then-join route. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N".

### Prism pattern union

#### Admission

`Union(blank, section, copies...)` takes one centered circular XY prism and
two to 32 axial extrusions from one recorded section. All bodies share the
same positive interval starting at Z = 0. The blank has one circle and no
holes. Each section has no holes, no section or axial displacement, a final
centered inward root arc, and rigid radial `Copy` placement without
translation or reflection. Copies retain the first section's exact source
record identity. A separately drawn similar section does not enter this
path.

The root arc's denoted radius comes from its recorded start. Its recorded
end can differ slightly, so both endpoints must have squared radius
strictly below the blank's squared radius after multiplying by the copy's
exact dyadic XY scale squared. A tangent or outward root refuses this route
before a joined shell is built. A set with a free-form section is first tried
as a union of the sections (Part A §2),
which joins a root arc lying on the blank's rim and any number of copies. This
route takes such a set only when that one refuses, as for a root arc a
nanometre inside the blank, where `sketch` places no region of the arrangement
inside an operand. Every other boundary curve stays outside the blank:
lines and fitted-spline Bezier spans use a strict support line for all their
controls; centered arcs use their denoted radius. The two lines next to the
root start inside and end outside the circle, with a positive radial
derivative at the inner end. These tests prove each connector has one
circle crossing and the root arc enters the blank.

Each section's `Tessellate(VerifyAll)` mesh supplies its source-to-held
boundary and occupied-volume bounds. A candidate separating line through
the difference of two held root endpoint sums must leave every held vertex
of one section strictly beyond every held vertex of the other, after
charging both mesh bounds times the line's exact length. The boundary and
therefore each filled section lie in their separate halfplanes. Every pair
must pass. The root endpoint angle intervals must leave an ordered gap
between adjacent copies.

#### Joined shell

The build keeps each section's exterior wall facets and omits its inward
root wall. It connects consecutive root endpoints with a wall sampled on
the blank circle at steps no wider than 0.02 radians. Endpoint angles use
`Atan2Interval`, and interior points charge both their angle interpolation
and `RadSinCosInterval` rounding. The two caps triangulate the same ordered
exterior and circle-gap outline used by the walls.

`tessellation.OrientJoinedMesh` proves every edge pairs and the result has one orientable
component. The bounded exact triangle crossing audit checks the complete
shell. Each cap audit proves a simple counterclockwise outline containing
the axis and an exact tiling by its cap triangles. A separate exact
point-in-polygon and edge-contact check keeps every omitted held root chord
strictly inside both cap outlines, except for its two shared endpoints.
Any failed check returns `ErrUnsupported` and leaves all inputs live.

#### Boundary and occupied volume

For a connector from inner point `q` along step `d`, squared radius grows
at first derivative `2 q·d > 0`. Its unique circle crossing parameter is
at most `(R²−|q|²)/(2 q·d)`. The copy's exact XY scale enters both terms;
the crossing distance multiplies that parameter bound by the scaled step
length. Nonconnector held wall triangles must remain outside
`R + mesh.Bound()` by their exact minimum projected radius.

The seam band adds the source root-radius gap, both connector crossing
reaches, any held root-vertex excess, circle-chord sagitta, and ring-point
rounding. The joined boundary bound adds that band to the largest section
mesh bound and must be at most 0.1 mm. The complete ring and connector
changes lie in the symmetric annulus from `R−band` to `R+band`; admission
requires `band < R`. Its occupied volume is at most `4πR·band·height`.
The output's symmetric-difference bound adds that annulus volume to each
source section mesh's already verified occupied-volume bound. Its area
slack includes a perturbed area upper bound for the joined shell and upper
areas of the input faces. The two cap groups retain the blank's planar
surfaces and receive the result's own cap roles; other faces are faceted.

#### Axial edge selection

The authenticated inputs and their joined shell are one XY section swept
through a common Z interval. The result records this source proof. A faceted
edge chain receives a Z direction for `ParallelTo` only when every held
chord has exactly fixed X/Y and moves strictly in one Z direction. Bent,
stationary, reversed, and unproved faceted chains have no selector direction.
The edge remains a bounded `FacetedCurve`; this certificate does not make its
held XY position exact. A rigid placement drops the source-specific axial
selector certificate until that placed result has its own proof.

#### Local root Fillet

The joined body retains its authenticated circular blank, copied section
records, source meshes and audited shell. `Fillet` admits selected
source-root axial chains on that body. Other faceted chains and placed joins
refuse. The radius must have zero input displacement.

For each selected corner, exact dyadic connector and placement coordinates
define an outward line. Its material-side offset by the requested radius
meets the circle of radius `R+r` at the outward quadratic root. Rational
interval arithmetic brackets the center, the line foot and the circle foot.
The line foot must lie beyond the blank and before the next source vertex;
the circle foot must lie in a strictly bracketed shared-ring step. The bracket
may follow several consumed ring stations. Held station order and source
circle station order must both be certified, including ring-point rounding.
The entire tangent arc fits within `3r` of the source root. The tangent arc
must start on the fillet circle's near half: the line foot `P` lies on the
side of the circle facing the axis. Then nothing the fillet changes lies
farther from the axis than `P`. Squared radius grows along the connector
cutback toward `P`, the circular cutback stays at `R`, and the short arc from
the circle foot, the circle's point nearest the axis, grows in radius all the
way to `P`. The source control-hull certificate keeps every nonconnector wall
beyond the largest exact `|P|` bracket of the selected contacts. The two
connectors next to the root must also end beyond it. The complete fillet
circle reaches `R+2r`, but its far side is not part of the result, so a
radial root line shorter than `2r` still admits a fillet whose line foot lands
on it. Pinned source arcs charge their radial residual. The exact contact
center, line foot and source root define a narrow polar sector for the connector cutback, circular-wall cutback and short
fillet arc. The circle's tangent cone bounds the complete arc. Sectors of
selected contacts and all other copied connectors must be strictly separate
modulo a full turn. The connector belonging to each selected root is the only
exception. Sector overlap or an ambiguous angle refuses.

The shell moves each selected connector endpoint to its line foot, removes
the ring facet pairs before the circle foot, and moves the near endpoint of
the retained bracket pair to that foot. It removes the consumed stations
from both cap outlines, then joins the feet with a sampled circular wall.
Overlapping cuts and missing facet pairs refuse. The edge retains its bounded
faceted type. Endpoint angle intervals,
trigonometric intervals and chord sagitta bound every new arc station and
facet. The rebuilt shell must be connected and orientable with paired edges,
pass the full exact triangle-crossing audit, tile both caps exactly, and
keep the omitted source-root chords inside its caps.

`TestPrismPatternFilletPastShortRadialLine` rounds every root corner of 12-,
17- and 22-tooth involute patterns whose radial root line is shorter than
`2r`, and checks the volume gain against the analytic corner area. Their root
arcs lie 1 nm inside the blank, so the union of the sections refuses them and
this route joins them.
`TestPrismPatternFilletRefusesUnprovedCornerAndLargeRadius` refuses a radius
whose line foot lies past the root line's far end.

Unchanged source and ring walls keep the Union boundary bound. The new
boundary bound adds the arc station and sagitta allowance and must stay
within 0.1 mm. The occupied-volume certificate sums two complete `3r`
cylindrical disks per selected root over the old Union certificate, enlarging
the held disk by the old boundary and arc bounds. These disks may overlap;
the sum remains an upper bound. The area allowance charges both old and new
walls and caps in those disks. `Tessellate(VerifyAll)` checks the detached
result before Fillet commits and retires its input.

### Axial pattern join

`Union` of axial extrusions (Part A) takes operands that carry a rigid
placement, such as the copies `PatternCopies` and `Copy` make. One tooth, one rib or one lobe is modeled once,
patterned about the sweep axis, and joined to its base in one `Union`.

The rule names no shape. It applies to any axial operands whose placement keeps the sweep planes.

#### 1. What a placed operand is

An operand is a prism or a matching axial loft, as in axial-section-boolean §1, with any placement `Xform`. It denotes
its recorded section on its frame, mapped by `Xform` read as an exact rational affine map of the held floats. A copy of
a free-form section therefore denotes the rigid image the placement states, and its record is untouched.

A prism that carries a section displacement, and a loft that carries a bore, a blend, a construction delta or a surface
result, are still not operands.

#### 2. Placement onto the reference frame

`axialsection.Place` brings every operand that is not the reference onto the frame of the reference operand, the first
loft that carries no placement. A frame is orthonormal only to rounding, so `patternrecord.NewPlaneMap` reads the
reference's coordinates through the exact inverse of its Gram matrix, which is exact for a point in its plane, and keeps
the defects it cannot remove:

- The image of a source point can sit off the reference plane by `|S0 + S1·u + S2·v| / |n|`, with `n` the reference's
  plane normal. `MovePlaneRegion` bounds it over the section's extent and charges it.
- The map `A` need not be a similarity. A line and a fit spline map exactly under `A`. A circle or arc maps to an
  ellipse, which the charge bounds against the circle of the nearest similarity `S* = [[a, −b], [b, a]]`, whose scale
  lies in `[ScaleLo, ScaleHi]` and whose distance from `A` in the operator norm is `Defect`.
- Each end plane sits at a height from the reference's (`PlaneHeight`, `HeightGap`) that is charged in full.
- An end frame made by adding a float multiple of the normal to an origin sits a little off the sweep axis, so a loft's
  shift along its axis is the exact projection (`SweepShift`) and its lateral offset is charged.

A placement is refused (a miss) when the off-plane distance or a height gap or a lateral offset exceeds
`2⁻⁴⁴·(1 + extent + |translation|)` mm. A placement matrix and a frame are exact to a few float steps, far below that,
so a larger value names a tilted, lifted or sheared sweep. The limit only refuses: a value it admits is charged in
full.

#### 3. Re-recording and the charge

`patternrecord.MovePlaneRegion` maps every point of the section through the exact map and rounds it once to a float.
It returns the moved record and an upper bound `δ` on how far any point of that record sits from the exact image of the
source curve:

| Segment | Charge |
|---|---|
| Line | The larger endpoint rounding. |
| Circle | The centre rounding, `r·|λ − 1|` and twice `Defect·(|u|max + |v|max)`. |
| Arc | Three times the largest of its centre, start and end roundings (the argument of `offset2d.OffsetSectionDelta`), `2·ρ·|λ − 1|` for its largest radius `ρ`, and four times `Defect·(|u|max + |v|max)`. |
| Fit spline | The distance between the refit cubics and the image of the source cubics, below. |

A fit spline is a natural cubic spline over the sketch's float chord-length parameters. `geom.FitInterpolant.ExactSpans`
publishes each span's cubic with exact rational coefficients in the local parameter `u ∈ [0, 1]`, from the same code the
arrangement certificates use. The source spline's spans are mapped by `A` and `T` exactly. The moved spline is the
interpolant through the rounded image points, with its own exact spans. The two have the same span count, since the
moved points are the images of the source points to within rounding. For each matched span the difference of the two
cubics is a cubic in `u` with rational coefficients. Its Bernstein coefficients enclose it on `[0, 1]`, so the larger of
`√(Σ max|b|²)` over the two coordinates bounds the distance from each point of one curve to the point of the other at
the same `u`. That is an upper bound on the Hausdorff distance between the curves. It is a charge: it admits nothing,
and it widens every reading of the result.

The reference operand contributes `δ = 0`, and so does an operand on the reference's own frame. An operand's `Delta` is
its section's `δ` plus the off-plane distance, its two end-plane gaps and any `JoinDelta` it already carries. The result
adds each operand's lateral offset and the reference's, since the result keeps the reference's end frame.

An operand's `PlaneGap` bounds how far its true end planes sit from the recorded ones along the normal. `Place` adds to
it the larger of the two end-plane height gaps and the off-plane charge (`patternrecord.RegionNormalCharge`) over the
section's extent widened by the operand's own `Delta`, on top of any `PlaneGap` the operand inherits from a join. A prism
reference (`Operand.AsReference`) charges the gap between each raised frame's plane and the exact level plane into its
`PlaneGap`, not into `Delta`. The scale term of an arc is conservative: the three
pinned points of the image arc are the exact images, so the three-times argument already covers the arc.

#### 4. The result

`UnionMany` runs `Place` first, then the route of axial-section-boolean §2 over the moved sections. The loft it builds
carries `loftrecord.Payload.JoinDelta`, the largest `Delta` of its operands (an operand that is itself a join
contributes its own), and `loftrecord.Payload.JoinPlaneGap`, the largest `PlaneGap` plus that operand's lateral offset,
plus the reference's lateral offset (`JoinPlaneGap`). `loftrecord.Build` states it as the `AxialDelta` of every cap that
lies exactly on its section plane (loft-design §18); any other cap adds it to its vertex displacement. A loft that is
no join has a zero gap.

`loftrecord.Build` adds `JoinDelta` to the held-vertex displacement, the chord displacement and the matched
displacement of the join body, so every reading of that body encloses the ideal union of the placed operands. A body
rebuilt from a copy of the payload (`Fillet`, a through-bore `Cut`, a placement) keeps the term as extra slack and
denotes its own record: its readings make no claim about the operation applied to the ideal join. `ConstructionDelta`
is not used, because it records a fitted-spline fillet's departure from its ideal rewrite and the fillet refuses a
second one.

#### 5. Limits

- The placement must keep the sweep planes to the rounding of the placement (§2): a rotation about the sweep axis, a
  translation in the plane, or any composition, on any frame. A tilt, a lift off the plane or a differently sized
  sweep is a miss, and the Union takes its next route.
- `Fillet`, a through-bore `Cut` and a cap `Chamfer` of the join read the frame's exactness themselves: the loft fillet
  needs an end frame exactly on the sweep axis (`loftrecord.FilletStraight`), so a join on an oblique frame refuses a
  fillet.
- A set whose only loft is itself placed has no reference frame and is a miss.
- Segments of kind line, arc, circle and fit spline are placed. Other kinds are a miss.
- A tooth or lobe that only touches the base (a bottom arc resting on the circle with the flank ends on it) or misses it by
  a micron is a miss: it makes no region `sketch` can certify as one union. `apitest.TestPatternUnionRefusesLobesThatOnlyTouchTheBase`
  pins both.
- Copies whose free-form edges cross one another (two arched tops that overlap), or whose sides nearly coincide without
  sharing a whole edge, are a miss: `sketch` certifies a spline against a circle but not a spline against another spline
  or an arc. `apitest.TestPatternUnionRefusesCopiesItCannotCertify` pins both refusals against the 4 mm step that joins.
- The route is `Union` only.

#### 6. Tests

- `internal/patternrecord`: `MovePlaneRegion` refuses a tilt and a lift and accepts the identity, charges a 17th-turn
  turn about the normal of an oblique frame within 1e-12 and refuses a hundredth-radian tilt of it; the fit charge is below
  1e-13 for a 17th-turn rotation and is at least a deliberate 1e-3 displacement of one knot, so it is not vacuous; the
  circle charge is the radius scale deviation.
- `internal/axialsection`: a lobe placed by a 17th turn is re-recorded within a positive `δ` below 1e-12 and each fit
  point is the rotated source point; misses for a tilt, a lift and a placed reference; each height and plane guard fires
  alone (shown to fail by disabling it); a section of quarter-millimetre coordinates slid by whole millimetres is charged
  exactly zero, spline included (`TestPlaceChargesNothingWhenTheMapIsExact`), so the charge is nonzero only where a
  placement rounds. `TestPlaceChargesPlaneGapAlongTheNormalOnly`: `PlaneGap` stays 0 for a turn that keeps the z axis
  exact and is positive and below 1e-12 for one that rounds it, `JoinPlaneGap` adds both lateral offsets, and an
  inherited gap is kept. `TestAsReferenceChargesTheRaisedPlane`: a raised origin that rounds gets a positive gap.
- `apitest.TestAxialSectionUnionExactTurnsKeepExactCaps`: a join of quarter turns keeps `Exact` caps.
  `TestAxialSectionUnionCounterboreRimChamfers`: a blind counterbore cut into a join cap has a `Circle3` rim that
  `Chamfer` takes.
- `apitest.TestPatternUnionManyCopies` joins 64 and 128 lobes to a disc and checks the sum of the parts.
- `apitest.TestPatternedLobesJoinADisc`: one lobe patterned 8 and 12 times and joined to a disc has the volume
  of the same lobes drawn at their angles within both bounds, with a larger bound, and a `VerifyAll` mesh whose boundary
  and volume are verified. `TestPatternedLobesAboutATiltedAxisAreRefused`: `ErrUnsupported`.
  `TestPatternUnionOnAnObliqueFrame`: the same part on a frame no axis aligns with has the world-frame part's volume
  within both bounds, with a verified boundary and volume.

#### 7. Joining a pattern to a base

`PatternCopies` followed by `Union(target, feature, copies...)` joins a feature and its pattern to a base. The
join takes every route `Union` takes over those operands, including the two-at-a-time fold
(Part U), so every refusal of §5 and of Part A §3
is `Union`'s. A refused `Union` leaves `target`, `feature` and every copy live, and the caller removes the copies
with `Remove`. The history holds the `OperationPatternCopies` entry and the `Union`'s own entries.

Each copy of a loft feature is built before the `Union` reads it. The axial-section route reads only each
operand's payload, so the result is the same as for the same lobes drawn in place.

`Patterned` keeps its own rule (`agent-docs/mirror-pattern-design.md` §4.3, §6.1): it returns the instances of
one body as one body, takes no target, and refuses disjoint lumps.

#### 8. Features whose flanks cross the base

A feature whose fit-spline flanks start inside the base's circle and leave it joins the base the same way: the private
scene cuts each spline against the circle, and `sketch` certifies the crossings (`TExact` on every `Partial` fragment of
its pair certificate, `sketch/docs/pair-certificate-design.md`). The union records the circle's remainders as trimmed
`CircleSeg` fragments. A union `sketch` leaves uncertified is a miss, as in axial-section-boolean §3. The joined body
then takes a concave `Fillet` where each flank leaves the circle, a through-bore `Cut` and a cap `Chamfer`.

Tests: `apitest.TestPatternUnionFlanksCrossTheBase` joins 8 and 12 patterned lobes whose flanks cross the disc and
compares their volume with the same lobes drawn at their own angles.
`TestPatternUnionFlanksCrossTheBaseThenFilletBoreChamfer` takes 8 of them through the fillet, bore and chamfer.

## Part U — Union fold

`Union` of three or more bodies takes the certified routes of `boolean_many.go` as a set. When none of them takes the set,
it joins the bodies two at a time, left to right, instead of refusing. The fold is `unionFold` in `boolean_many.go`.

### 1. Rules

| Decision | Rule |
|---|---|
| U1 | The fold runs only when `performUnionMany` refuses with `ErrUnsupported` and the refusal is the one that says no certified route takes the set. Every other refusal, and every gate that runs before it (a repeated body, a retired body, a foreign body, a sheet), is returned as it was. |
| U2 | The bodies are joined in the order given: the first with the second, that result with the third, and so on. Each step is the two-body `Union`, so it takes that `Union`'s routes, its bounds and its refusals, and the result is a mesh Boolean's where no analytic route applies. |
| U3 | The chain is run on copies first, each retired when read, so a step that refuses leaves every body live and the document as it was, and the refusal returned is the many-body one. Only a chain that completes on the copies is run on the bodies. The copies leave history entries. |
| U4 | The result retires the bodies and every intermediate, and the history holds one `OperationUnion` for each step. |
| U5 | The volume of the result is that of the union to the exactness of the mesh steps, whatever the order. The tests compute it independently by integrating the union's cross-section. |
| U6 | A pattern joined to its base, `PatternCopies` then `Union(target, feature, copies...)`, takes the fold as any `Union` does. The tests join three overlapping round pins and three overlapping hexagonal pins, as tall as a slanted mesh plate and flush with both its faces, to the plate (volume 7500 within a bound under 0.1 for the hexagonal pins and under 5 for the round pins), and, under `ExactOnly`, keep the refusal with the plate, the pin and its copies live. |
| U8 | The first Union of a fold whose second operand is flush with the first's faces shrinks it inside (gap-tolerance design §5.2 item 6), so the next Union meets the first operand's own faces and not a translation's sliver; a round pin, held within its chord bound, shrinks past that bound and the result's bound states it. |
| U7 | A fold step meets contacts whose normal is the zero vector, from a sliver facet of the earlier result that is too thin to normalise. The shifted retry gives such a contact no direction to open along (`shiftOpens` skips it) and reaches its slack along the shift's own length (`shiftNeed`), where before it left no direction at all and ended the retry. |

### 2. Not covered

- A chain whose step refuses on its own geometry, such as coarse held meshes: the fold returns the many-body refusal.

## Part A — Boolean of axial extrusions by the 2D Boolean of their sections

A Union of bodies that are each an axial extrusion of one recorded section between the same two parallel planes is the
axial extrusion of the 2D Boolean of those sections. `internal/axialsection` builds that result: it asks `sketch` for the
2D answer and records it as a matching axial loft, so the loft routes (`agent-docs/loft-design.md` §17, §18) read it.

The rule names no shape. It applies to any such operands, whatever the sections hold.

### 1. Operands

`axialsection.OperandOf` reads a payload as an axial extrusion:

- A `prismPayload` with no section displacement, no sweep-level displacement, the identity placement, no blend and no
  holes.
- A matching axial `loftPayload` (loft-design §17): two identical recorded profiles with no holes, equal section axes,
  end planes that differ only along their shared normal (`prismplacement.SharedAxisOf`, exact), the identity placement,
  and no bore, blend, through-bore Cut or surface result. A loft's own chord displacement is not read: this route rebuilds
  from the recorded profiles, as the loft's own placement does.

A prism that carries a section displacement is not an operand, because the displacement states how far its record sits
from the solid it stands for and this route rebuilds from records.

Operands are admitted together only when every one lies between the same two planes: equal section axes and equal exact
offsets of both end planes along the normal. A loft records the planes. A set of prisms takes them from its first unplaced
prism (`Operand.AsReference`): that prism's frame raised along its normal by each sweep level, with the float rounding of
the raised origin charged as its lateral offset, and the offset of its raised plane along the normal charged as its
`PlaneGap`. Anything else is a miss, and the Union takes its next route.

### 2. Flow

1. Admit every operand (§1). A set none of whose sections holds a free-form segment is left to the analytic prism routes
   (Part B), which carry their own proofs for line, arc and circle sections.
2. Build one private `sketch` scene holding every operand's recorded outer loop: one entity per distinct recorded curve
   across all operands, every point fixed from the records. Only whole edges are admitted (a segment whose range is not the
   whole curve is a miss), and the kinds the scene builder creates are line, arc, circle and fit spline.
3. Declare one loop per operand (`Sketch.DeclareLoop`) with its entities in recorded order.
4. Read the arrangement's regions (`Sketch.Profiles`). A valid region that is `Inside` at least one loop belongs to the
   union. A region that is not `Inside` any loop and is `Unproven` for some loop leaves the question open, which is a miss.
5. `UnionProfiles` of the chosen regions. A union `sketch` declines to build, or one with a `Partial` edge that is not
   `TExact`, is a miss. This is decad's reject-only rule (CLAUDE.md): sketch's own certificate decides, and decad never
   repairs a fragment it reports inexact.
6. Record the union profile through the seam (`sectioncert.RecordProfile`) and build the loft of two identical sections
   between the operands' end planes with `loftrecord.Build`. The result is a matching axial loft whose own bounds are
   computed fresh.

The route runs for the many-operand Union (`Union(base, tools...)`) ahead of the other many-operand routes, and for a
two-operand Union ahead of the analytic prism routes and after the loft routes that need a recorded join.
A set of prisms with no loft runs this route (`axialsection.UnionManyPrisms`) after the loft and sector routes and ahead of
the prism-pattern route (Part P). Both routes can take a pattern of free-form sections joined
to a circular blank. The prism-pattern route admits one only when every placed copy's root arc rounds strictly inside the
blank, which a root arc on the blank's rim does or not by the last bit of each copy's rotation. This route decides by what
`sketch` certifies about the placed sections, not by the sign of a rounding residue, so a root arc on the rim joins here
at every tooth count. The prism-pattern route takes the sets this route refuses. A refusal here is a miss for the routes
after it. When they refuse too, their refusal is the answer and the Union tries no pair chain.

### 3. What is a miss, and what is an error

A miss returns `ok = false` with a nil error and the Union takes its next route, so a set this route cannot certify behaves
as it did before it existed. An error is a genuine failure of the build (a canceled context, a loft build refusal after the
section was certified).

Misses: fewer than two operands; an operand that is not an axial extrusion; operands on different planes; a hole in any
section; a trimmed segment in any section; a segment kind the scene does not create; a region `sketch` cannot place inside
or outside; a union `sketch` declines to build, such as two edges that share only part of a line (a flush rib); and a
fragment `sketch` reports inexact.

A trimmed segment is the one that matters in practice: the union of a disc and a lobe records the circle between them as a
fragment, so the result is not an operand of a second two-body Union. The many-operand Union joins all operands in one scene
and so avoids it. A `DeclareLoop` element that names an entity with a parameter range, which would admit chains, is not implemented.

### 4. Limits

- Placed copies. An operand that carries a placement (a `Copy` or `PatternCopies` instance of a prism or loft) is placed
  onto the reference frame and re-recorded with a charge, and a prism that carries a section displacement is not an
  operand (Part P). A placement that tilts the sweep planes is a miss.
- The loft fillet (§17) takes the corner this union leaves, a line or arc meeting a trimmed circle fragment, so a root
  fillet, then a bore and an outer-cap chamfer, apply to the result.
- Union only. Cut, Intersect and sections with holes are not implemented.
- The 2D union adds one certified `sketch` arrangement per Union.

### 5. Tests

- `axialsection` unit tests (no free-form segment needed at this level): a plate and a rib that crosses its top edge
  (outline segments and area), three rectangles forming an L and a T, flush rectangles (a miss), a disc and N lobes with fit
  splines on its circle for N = 8, 12 and 17 (area equals the sum of the parts, 6N outline segments), and the misses (one
  section, a hole, a trimmed segment).
- `apitest.TestAxialSectionUnionLobesOnADisc`: the joined volume against disc plus lobes within its bound, `VerifyAll`
  boundary and volume verified, for N = 8, 10, 12 and 17. `TestAxialSectionUnionPlateAndTongue`: a plate and a curved-top
  tongue, the overlap an exact rectangle. `TestAxialSectionUnionFinishes`: a bore Cut and an outer-cap Chamfer on the result.
- `TestRimToothPatternJoinsBySectionUnion`: patterns of 8, 11, 17, 18 and 27 involute teeth whose root arcs lie on the
  blank's rim, each thinned so that on amd64 every copy's root arc rounds strictly inside the blank, join through this
  route ahead of the prism-pattern route and take all 2N root fillets with a verified boundary and volume. At 27 teeth the
  fillet reaches the involute flank, which the prism-pattern fillet refuses. `TestRimToothPatternFilletVolumeBound`: the
  rounded 17-tooth pattern at 10 mm thick reads its volume within 0.5 mm³.
