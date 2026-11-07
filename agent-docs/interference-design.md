# Interference Design

How `Document.Verify` proves pairwise overlap and reports its volume without
changing the document: the pair relation (§1), stable report walk (§2), proof
paths (§3), full-containment certificate (§4), read-only boolean evaluation
(§5), positive-volume gate (§6), errors, cost and cancellation (§7), tolerance
gate (§8), evaluator coverage (§9), tests (§10), and increment plan (§11).
Companion to `agent-docs/verification-design.md`, which owns what an
`Interference` row means and how its `Volume` is judged; to
`agent-docs/clearance-design.md`, which owns analytic pair classification; and to
`agent-docs/evaluator-design.md`, which owns the mesh boolean and its proven bounds.
Nothing here adds a public API or modeling operation. Nothing here changes a
public boolean's consuming semantics.

## 1. Pair relation

Every unordered pair of proven solids has one evaluator relation. A pair
holding a `BodySheet` operand takes none of it: a sheet encloses no region, so
the interior relation below is not the question for it at all, and
`agent-docs/surface-design.md` §9.3 states its own decision procedure and report
vocabulary (crossing, contained, outside, undecided) in its place. §2 states
where that procedure sits in the report walk.

| Relation | Proven claim | Report effect |
|---|---|---|
| **disjoint** | interiors share no point | no `Interference` row; emit `Clearance` only when asked and measured |
| **touching** | interiors are disjoint and boundaries meet through a certified contact | no `Interference` row; emit `Exact` zero `Clearance` only when asked |
| **overlapping** | interiors share a non-empty open region | emit `Interference` only when overlap volume is also bounded; otherwise `Suspect` |
| **undecided** | none of the three claims is proven | no pair row; `Suspect` |

`boundaryaudit.PairOverlapping` MUST be distinct from `boundaryaudit.PairUndecided`. A nesting witness,
an admitted transversal boundary crossing, or a certified same-side contact
proves overlap even when the evaluator cannot yet measure it. Losing that fact
inside `boundaryaudit.PairUndecided` prevents the next proof stage from using it and makes
diagnostics lie about what remains unknown.

The relations concern **interiors**. Boundaries may touch while the pair stays
disjoint. A positive-volume `Interference` row is never emitted for contact
alone.

The pair result also records the strongest certificate already obtained:

```go
// internal/clearance/pairkernel
type Result struct {
    Verdict boundaryaudit.PairVerdict // undecided / disjoint / touching / overlapping
    // gap interval + exactness + pair diameter when disjoint or touching
    // containment classification when a strict nesting proof ran
}
```

The type stays internal; nothing public exposes it. The relation and its certificate MUST be
structural values, never inferred from an error string.

## 2. Stable, non-mutating report walk

`Verify` walks live bodies in `Document.Bodies()` order, keeps only proven
solids for pair work, and enumerates `i < j`. Both pair lists preserve that
order. No spatial index, worker completion order, or proof-path order may
reorder rows.

**A pair holding a `BodySheet` operand is reported, not dropped.** A sheet
encloses no region, so §1's interior relation is not the question, and dropping
the pair silently would let a sheet passing straight through a solid leave a
`Sound` report. The pair runs §3.1's box separation first: separated boxes
emit nothing. Boxes that MEET run `agent-docs/surface-design.md` §9.3's own decision
procedure, which owns the full report vocabulary that follows — a proven
crossing (`Interfering`, no row), a proven containment or separation with a
measured gap (`Sound`, a `Clearance` row under `WithClearances()`), or
`DiagUnsupportedPairSheet` naming the pair and no row when the procedure
cannot settle it, including every sheet-sheet pair.

For each pair:

1. Check `ctx`.
2. Use bounds-inflated box separation. If it proves disjoint interiors and
   clearances were not requested, finish the pair.
3. Run the analytic pair kernel when a gap is requested or box separation did
   not settle the partition.
4. Finish immediately on `boundaryaudit.PairDisjoint` or `boundaryaudit.PairTouching`; append a clearance
   row only when `WithClearances()` asked and the gap was measured.
5. For an undecided pair, an unadjusted mesh Cut result and an exact same-set
   analytic copy of its consumed tool prove non-overlap. Check both pair
   orientations using operation history and a bounded set comparison. If a
   clearance was requested, emit `DiagUndecidedClearance` and no clearance row:
   this proof does not measure a gap.
6. On `boundaryaudit.PairOverlapping`, try its containment-volume certificate first, then
   read-only intersection measurement when needed.
7. On `boundaryaudit.PairUndecided`, try read-only intersection measurement. A positive
   volume may prove overlap even when the analytic kernel could not classify
   the contact arrangement.
8. If neither volume path proves and bounds the overlap, mark the pair
   undecided for aggregation.

Steps 2–8 run on a bounded worker pool, one pair per task, sized like the
boolean's contact batches (`GOMAXPROCS`, at most 12). A pair's proof reads only
its two operands, immutable operation history, per-body caches that every caller fills with the same value,
and the chord §5.3 fixes for each body before the pool starts, so its outcome
does not depend on which pairs run before it or beside it. Each task writes its
outcome to its own slot, and `Verify` appends the slots in pair order. A pool
of more than one worker runs the contact batches of every boolean inside its
pairs on that pair's worker alone, so the pair proofs never run more
goroutines than the pool has workers. Letting each pair's batches start their
own workers beside the pool's put two layers of goroutines on the same cores:
on a 24-core machine the 20-segment revolved star took 40 s that way and 19 s
this way. No measured document got slower, down to a three-body star whose
three pairs leave most cores idle. One worker proving the pairs alone leaves the
batches their own workers. The worker count never changes a batch's result,
because each batch merges its slots in input order. When a
pair fails, no pair after it starts, every pair before it still runs, and
`Verify` returns the failure at the lowest pair index: the error a walk of the
pairs one at a time would stop on.

`Verify` MUST NOT call public `Intersect`. Public `Intersect` retires both
operands, advances provenance, and registers a result. Verification is
non-mutating and must leave all model state unchanged:

- `Document.Bodies()` membership and order;
- every body's live/retired state;
- the next private producer identity.

## 3. Proof paths

The paths run cheapest and strongest first. A later path may add a measurement;
it may never weaken an earlier relation.

### 3.1 Box separation

Bounds-inflated boxes with disjoint interiors prove the bodies' interiors
disjoint. Touching boxes are included: box contact cannot prove body contact,
but it does prove no positive-volume overlap.

Box separation does not measure the true gap. Under `WithClearances()`, the
analytic clearance kernel still runs.

### 3.2 Analytic pair kernel

The clearance kernel owns boundary distance, certified contact, boundary
crossing, and nesting classification. It returns all four §1 relations:

- positive boundary clearance plus nesting excluded → `boundaryaudit.PairDisjoint`;
- a certified opposite-material contact with no other crossing →
  `boundaryaudit.PairTouching`;
- an admitted transversal crossing, certified same-side contact, or nesting
  witness → `boundaryaudit.PairOverlapping`;
- insufficient trim, contact, or containment proof → `boundaryaudit.PairUndecided`.

Boundary distance alone MUST NOT turn nesting into disjointness. A nesting cast
must preserve three outcomes — outside, inside, undecided — instead of returning
one `bool` that merges inside with failed classification.

**The exact planar arm** (`internal/verifyrun/clearance.go`) answers a pair the analytic
kernel leaves `boundaryaudit.PairUndecided` when one operand is a payload that kernel has
no carrier model for and the exact planar relation can read: a
`mitredSweepPayload`, a `facetedPayload` or a `coilPayload`, each read off
its own held triangle set with that set's displacement `δ`, against a prism,
a stitched solid, or (a mitred sweep or a faceted result only) another of
the two; a coil is served against a prism or a stitched solid alone, read exactly off its own record — the same
admission `Document.ContactPair` uses (`planarpose.SolidAtPose`,
`internal/planarpose/pose.go`; `agent-docs/multibody-dynamics-design.md` §9.1, §10.4).
Every predicate there is an exact sign over dyadic coordinates, and the band
rule of §10.4 turns the held relation into the partition's: a held gap whose
lower end clears the summed `δ` is `boundaryaudit.PairDisjoint` with the interval widened
by `δ` outward; a held touch is `boundaryaudit.PairTouching` only at `δ = 0`; a held
overlap is `boundaryaudit.PairOverlapping` at `δ = 0` or through a vertex proven deeper
than `δ` (`pair.PlanarDeepVertex`); anything else stays `boundaryaudit.PairUndecided`. The
arm only ever adds a verdict: it never replaces one the analytic kernel
reached. A pair of prisms or stitched solids keeps the analytic kernel's
answer, a cup is never tessellated for verification
(`agent-docs/payload-verification-design.md` §1), and a loft, a composite sweep
and a cap-loop chamfer keep their own designs' staging.

**The coaxial separator** (`internal/verifyrun/coaxial.go`) answers a pair both
arms leave `boundaryaudit.PairUndecided` when one circular cylinder separates
the interiors: one body lies in the closed solid cylinder of axis `L` and
radius `r`, the other misses its open interior. The interiors are then
disjoint whatever the boundaries share, so a shaft pressed into a bore of its
own radius, whose coincident faces no held mesh can decide, is proven
non-overlapping. Each side is read from records alone, in exact rationals:

- a prism whose section is held exactly lies inside the cylinder over its
  outer loop when that loop is one whole circle, and outside the cylinder over
  any hole that is one whole circle;
- a mesh Boolean result whose history denotes its operation exactly
  (`boolchain.Expandable`) inherits it: a Cut lies where its target lies, an
  Intersect where either operand lies, a Union where both lie;
- a Cut result also lies outside a cylinder that is its tool's, or a Union
  tool operand's, when that operand is a prism over one whole circle with
  exact levels and the target lies within those levels along the cylinder's
  axis. The target's reach along the axial functional g (Lᵀg = e₃ for the
  tool's lift) is read exactly, as the end-cap separator reads support. Every
  point of the target inside the cylinder then lies in the closed tool, so the
  Cut's interior misses the cylinder's open interior. A plate bored through
  by a tool on the shaft's own lift is decided this way;
- a solid Revolve the meridian separator reads (below) lies outside a
  cylinder on an exactly orthonormal lift when its lifted axis is the
  cylinder's axis line and its least radius, scaled by its lift across the
  axis over its sweep, reaches `r` (`internal/verifyrun/coaxial_reach.go`,
  "The Revolve reach" below). A label bonded flush on a yarn pack, placed at
  a rounded angle, is decided this way.

The coaxial walk reads Boolean results as the history walk does, any number
of operands included. Two cylinders match only with equal radii and either
the same axis line under
lifts that are exactly orthonormal, or one affine lift (frame times placement,
exactly equal linear parts) that carries one section circle onto the other.
A lift that is not exactly orthonormal makes the denoted set an affine image
of a round cylinder, which still separates when both bodies share it. Two
cylinders on frames that are exactly orthonormal under one placement, bit for
bit, are also compared before that placement: it is one affine bijection on
both sets. Bodies
lifted by different inexact rotations, or by one rotation composed with a
rounded translation, denote sets whose boundaries may cross by rounding, and
stay undecided.
The certificate proves no gap, so under `WithClearances()` the pair keeps an
unmeasured-clearance diagnostic, as a Cut result against its own tool does.

**The Revolve reach.** A Revolve's denoted point at axial `z`, radius `ρ ≥ 0`
and angle `φ` is `A + z·D + ρ·(cos φ·a₁ + sin φ·a₂)`: `A` the lifted axis
anchor, `D` the lifted axis direction `L·W`, and `a₁ = L·E₀`,
`a₂ = L·(W × E₀)` the lifted radial directions, each an exact rational of
the record's floats, the leaves `revolvemesh.RevolveLift.ExactPointRound`
reads. A region the meridian separator reads on the side `ρ < 0` is mirrored,
which turns each direction to its opposite and leaves the reading below
unchanged. When `D` is parallel to the cylinder's axis and `A` lies on that
line, both exactly, the point's distance from the axis is
`ρ·|cos φ·P a₁ + sin φ·P a₂|`, `P` the exact projection across the axis,
because `P` removes `A`'s offset along the axis and `z·D`. Its square is
`ρ²·Q(φ)`, with `Q` the quadratic form of `M = [[|Pa₁|², Pa₁·Pa₂],
[Pa₁·Pa₂, |Pa₂|²]]` at `(cos φ, sin φ)`. Let `ρ_min > 0` be the region's
least radius (a vertex's, or a disc's centre less its radius) and
`κ = r²/ρ_min²`. Every point then lies at least `r` from the axis when
`Q − κ` is non-negative at every swept direction:

- a full turn sweeps every direction, so `M − κI` must be positive
  semidefinite: both diagonal terms and the determinant non-negative;
- a partial sweep `[φ₀, φ₁]`, whose width read from the two ends' angle
  enclosures (`revolveangle.Angle.EnclosureFor`) lies in `[0, π)`, sweeps only
  directions `w = α·e₀ + β·e₁` with `α, β ≥ 0`, `eᵢ` the end directions.
  There the form is `α²·q₀ + β²·q₁ + 2αβ·b`, with `qᵢ` its value at `eᵢ` and
  `b` its bilinear value at the pair. It is non-negative when `q₀, q₁ ≥ 0`
  and either `b ≥ 0` or `b² ≤ q₀·q₁`, since it is then at least
  `(α·√q₀ − β·√q₁)²`. The ends' sines and cosines are rational enclosures
  (`revolveangle.Angle.SinCosFor`), `q₀`, `q₁` and `b` are evaluated in
  interval arithmetic, and each test reads the bound that can only refuse
  more.

The Revolve then lies in the closed exterior of the cylinder, so its interior
misses the open interior of the other body, which lies in the closed
cylinder. Under an exactly orthonormal lift `Q` is 1 at every angle. Under a
lift that is a rotation by rounding, `Q` differs from 1 by about `1e-16`, and
unevenly with `φ`. A 0.13 rad label of inner radius 30.65 bonded on a pack of
that radius, turned by a rounded angle, has `Q − 1 ≥ 4.2e-16` over its sweep
and is decided. The same lift's least eigenvalue is `6.9e-17` below 1, so a
whole ring on it dips about `1e-15` mm into the pack: its denoted set overlaps
the pack's, and the reading refuses it.

A tangent contact designed through rounded numbers can overlap the same way,
and then no reading may call it apart. A brake cord of radius 0.6 swept half
round a bobbin's V groove of slope 2, at the float radius `42 + 0.6·√5`,
clears exact tangency by `6.5e-15` mm. Its arc's axis sits `7.1e-15` mm off
the bobbin's axis and its plane `5.3e-15` mm off the groove's centre, so the
cord's denoted set crosses a wall by about `2e-15` mm at one end of the arc
and `8e-15` mm at the other. The pair stays undecided until the design opens
a clearance.

**The history certificate** (`internal/verifyrun/csg.go`) answers a pair the
coaxial separator leaves open from the recorded Boolean history alone. A Cut
result's interior misses its tool's interior, so it misses the interior of any
body the tool contains: a rail glued into the socket its own longer stock cut,
whose faces coincide with the socket's, is proven non-overlapping however
many curved faces either body has. The walk reads a result's operation and
inputs where the result denotes that operation exactly: a mesh result the
chain admits (`boolchain.Expandable`), or an analytic result with no operation
warning and no private shift. An analytic result's record may hold its
section within a displacement of the set, but the set it denotes is the
operation's exact result, and only that set enters the walk:

- a Cut result misses a body that lies in its tool, or that its target
  misses; an Intersect misses what any operand misses; a Union, of any number
  of operands, misses what every operand misses;
- any two bodies the walk meets, a Boolean's operands included, miss each
  other when their boxes, each inflated by its own proven bound, have disjoint
  interiors, compared exactly (`measurement.BoxesDisjoint`): every body's
  interior lies in its inflated box's interior. A hook whose stem sits in a
  bore and whose ring stands clear of the drilled part is decided this way,
  the stem by its drilling tool and the ring by its box;
- a Cut result lies in its target, an Intersect in any operand, a Union in a
  body every operand lies in, and a body lies in a Union when it lies in any
  operand or in an Intersect when it lies in every one;
- a direct copy made by `WithCopyLiftTranslation` (`mirror-pattern-design.md`
  §4.5) reads as its source shifted by the copy's exact world vector
  (`historyProducers.ExactShiftProducer` authenticates the operation record,
  the referenced source and the lineage). The shift follows the source's
  Boolean history to its leaves. A box test adds each shift to its box
  exactly, and two prisms add each shift to their exact lift translation, so
  a pin copied between two seats sketched on one lift is read against the
  second seat. The end-cap and slab separators read the other body moved by
  the relative shift `d`: its reach gains `g·d` (`m·d`), and the end-cap slab
  is read with the prism moved by `−d` relative to the Revolve. Every other
  rule reads unshifted records and answers only when both bodies carry the
  same shift: translating both by one vector keeps their relation. A copy made by `WithCopyPlacement` or `WithCopyRotation`, a copy of
  a copy and a `Transform` of a copy stay leaves;
- a prism lies in another prism whose exact linear lift (frame axes carried
  by the placement basis, in rationals) is the same, with both sections and
  levels held exactly: the translation between the lifts solves exactly into
  a level shift and a section shift, the section shift must keep every
  recorded coordinate a float, and then its shifted levels lie within the
  other's and Sketch proves its shifted section inside the other's
  (`prismcells.SectionWithin`, the arrangement of
  `agent-docs/prism-boolean-design.md` §4.1-§4.2). A section whose outer loop
  is one whole circle exactly equal to the outer loop of a hole-free section
  lies in it without the arrangement, which cannot hold two coincident
  circles: the inner section lies in the closed disc, and closed containment
  is all the miss rule reads, because a body in the closed tool misses every
  interior the tool's Cut result keeps. A pin seated in the bore its own
  tool drilled is decided this way. A prism that is one axis-aligned
  rectangle of whole lines lies in another such prism, on frames whose axes are
  the other's signed and permuted, compared before one placement both carry bit
  for bit or in the world when both lifts are exactly orthonormal, when its box
  lies in the other's box, exactly. A plate swept along one axis seats in the
  recess a box swept along another milled. Any other leaf lies only in an
  analytically equal body;
- two such prisms, read through the same exact offset, miss each other when
  their levels share no interior or Sketch proves their sections share none (`prismcells.SectionsApart`), so a
  block glued flat on a rail is decided; a leaf against a Boolean result is
  read through the result's history;
- two leaves the prism rule leaves open miss each other through the meridian
  separator or the end-cap separator below.

**The meridian separator** (`internal/verifyrun/meridian.go`) reads a leaf as
a region of the (ρ, z) half-plane whose full revolution about an axis holds
the body. A solid Revolve with no section displacement, an exact unit axis,
no snap onto the axis, no material admitted across it, and a meridian of whole
chained lines, or of one whole circle, on one side of the axis is its own
region (its holes only remove material). A partial sweep lies in the full
revolution of its region, so its angles are never read; a band wrapped half
round a pulley is read as its disc. A prism with exact section and levels gives the rectangle ρ ≤ r over its levels for a whole-circle outer loop of
radius r, and the strip ρ ≥ r over its levels for each whole-circle hole. Each
frame must be exactly orthonormal, so frame coordinates measure true distance
before the placement. Two regions are compared in a's frame coordinates when
the two lifts share one linear part, b moving by the exact solve of the
translation between them; or before one placement both carry bit for bit,
which is one affine bijection on both and so keeps interiors apart, where their
frames are exactly orthonormal; or in the world when both lifts are exactly
orthonormal. The axes must be one line exactly; b's z then maps to
a's by an exact shift and sign. A strip's open side is closed at a radius past
every finite radius of the two regions, which loses no point the other region
can reach and keeps two strips overlapping exactly when their levels do. A disc
moves with its centre. Every coordinate must stay a float, and Sketch then proves the two regions'
interiors disjoint (`prismcells.SectionsApart`).

The step is sound because each body lies in the revolution of its region,
and the comparison map is an affine bijection, so it keeps interiors apart.
A point in both interiors would sit at one (ρ, z) about the shared axis,
interior to both revolutions. For ρ > 0 that (ρ, z) is interior to both
regions. For ρ = 0 the point's neighbourhood holds points of small positive ρ
in both interiors, and those are interior to both regions. Either way the
regions' interiors meet, which Sketch has ruled out. A turned spindle under a
bearing that sits flush on its collar, with its bore flush on the spindle, is
decided this way.

**The end-cap separator** (`internal/verifyrun/halfspace.go`), under bodies
carrying different exact copy shifts, reads the other body moved by the
relative shift: its support gains `m·d` and the slab moves with the prism. It
reads a prism a
with exact levels as lying between the planes of its two end caps: with m the
exact cross product of its first two lift columns, turned so that m·L₃ > 0,
a lies where m·x ≥ m·p(lo) and where m·x ≤ m·p(hi). It decides the pair when
the other body b lies wholly on the far side of one of those planes, proven
through b's support along m, with no unit normal:

- a prism with exact section and levels over a whole circle of centre c and
  radius r reaches m·t + μ·(c, z) + r·√(μ₁² + μ₂²) at one of its levels, with
  μ = Lᵀm; over whole lines it reaches its largest vertex, and an arc adds the support of the circle through its start about its centre, the record stating that start and the centre and never the radius;
- a Revolve the meridian separator reads over whole lines reaches m·t + μ·anchor + z·(μ·d) +
  ρ·|μ across d| at a vertex of its region, d its axis and μ again Lᵀm, the
  frame coordinates measuring true distance so the split across d is exact.

Each candidate lin + ρ·√q is held at most the cap's value k exactly, as
k − lin ≥ 0 and (k − lin)² ≥ ρ²·q. For a Revolve only the part of b in a's
slab across the axis enters: a's reach along the axial level's functional
g (Lᵀg = d), each square root rounded up to a float checked exactly, bounds
the slab, and the region is cut to the vertices inside it and its edges'
crossings of the two levels, whose hull holds the cut region. The step is
sound because a lies in the closed half-space beyond the cap and in the slab,
and the part of b in the slab lies in the opposite closed half-space, so the
interiors, which lie in the open half-spaces, share no point. A set screw
seated on a shaft is decided this way, away from a collar the screw's slab
would reach.

**Same-set producers.** Both walks read a Refine result as its source:
Refine moves no vertex and changes only the bound its mesh publishes, so the
true solid it denotes is the source's (`refine.go`). The root names such a
producer (`historyProducers.SameSetProducer`) only for a Refine whose one
consumed input is the recorded source, and the walks follow it before any other
reading, so a refined Union is opened as its Union. A Transform whose motion
is exactly the identity, unit axes and a zero translation bit for bit up to
the sign of zero, is read the same way: the identity moves no point, so the
placed body denotes its source's set whatever payload it holds. `Transform`
records each such result as it commits it (`Document.identityPlacements`),
since the operation record keeps no motion. A refined yarn chain that a
part's head tilt places by the identity at its rest pose is opened through
the placement, the Refine and the Union down to the span resting on the pack.
Any other motion, however close to the identity, keeps the placed body a leaf.

**The Cut-tool meridian** (`internal/verifyrun/cut_meridian.go`) reads a
Cut by a prism over one whole circle, or by a Union with such an operand, as a
region about that cylinder's axis. The target, whatever its shape, lies
between two levels `lo ≤ z ≤ hi` along the axis, `z` the tool frame's own
level, each end read from the target's reach along `±g` (`Lᵀg = e₃` for the
tool's lift), exactly where the reach is rational and from above where it holds
a square root; a Cut lies where its target lies, a Union where every operand
does. The tool is exactly the revolution of its rectangle `[0, r] × [z₀, z₁]`.
So the Cut result lies in the revolution of the band `[0, ∞) × [lo, hi]` less
the open rectangle `[0, r] × [z₀, z₁]`, and less any one operand of a Union
tool, since removing more only shrinks the result. The band's open side is
closed past every finite radius of the two regions compared, as for a hole's
strip. The other body's meridian is then compared with that region as the
meridian separator compares two (`sharedMeridians`, `SectionsApart`). A screw
whose shank sits in the pilot the tool bored and whose head rests on the
target's face is decided this way, though the target has no hole of its own.

**The slab separator** (`internal/verifyrun/slab.go`), under bodies carrying
different exact copy shifts, adds `g·d` to the other body's reach for the
relative shift `d`. It reads the world box
test of the history walk in a prism's own frame. With the prism's exact lift
`t + L·(x, y, z)`, its coordinate along local axis `k` at world point `p` is
`g_k·(p − t)` with `Lᵀg_k = e_k`, so the prism, with exact section and levels,
lies between `g_k·t` plus its section box's bounds along the section axes and
its levels along the extrusion. When another body's reach along `g_k`, or along
`−g_k`, stays at or below the near plane, the plane holds the bodies apart. A
reach is read exactly where it is rational and from above where it holds a
square root, each root rounded up to a float checked exactly: a prism's or a
lined Revolve's support as above, and a disc Revolve's as its centre's reach
plus `R·√(a² + q)` (with `a` the reach's part along the axis and `√q` across
it), which is the largest of `z·a + ρ·√q` over the disc. The step is sound
because each body lies in the closed half-space its reach bounds, and the
prism in the opposite closed one. A hook ring standing clear of a turned arm,
whose world boxes overlap, is decided this way.

**The axial section separator** (`internal/verifyrun/axial_section.go`) reads a
body a, less the prisms its Cut chain removed, against a prism b with holes. It
runs at a Cut result and at two leaves the other separators leave open. a is
read through its Cut chain (a Union tool's operands each) down to a target, a
solid Revolve the meridian separator reads or a prism, and b is a prism with
exact section and levels. The pair is decided when, over b's levels, the
target's cover lies in the union of b's holes and the cutting prisms' sections:

- a prism target's cover is its outer loop, moved into b's coordinates by the
  exact offset of two prisms on one linear lift (`liftOffset`), over the levels
  the two share;
  a prism over one whole circle on another lift is a cylinder about its axis and is
  read as a Revolve is, its meridian the rectangle ρ ≤ r over its levels, with the
  two unit directions across the axis its frame's own two axes;
- a Revolve's cover is a closed disc. The Revolve is round in its frame's
  coordinates, and b is an exact rational image of its recorded set, so a point
  of the Revolve sits in b's coordinates at c + z·g3 + ρ·(cos φ·g1 + sin φ·g2),
  with c, g3, g1 and g2 the exact solves, against b's lift, of the axis anchor,
  the axis direction and the two unit directions across it. The disc is about
  c + z_c·g3, of radius max(ρ·σ + |z − z_c|·|g3'|) over the vertices that bound
  the meridian's interior inside b's levels, σ the spectral norm of the section
  plane's 2×2 reading of g1 and g2, g3' the section plane's reading of g3, each
  square root rounded up to a float and checked exactly, and the centre
  rounded to a float and its distance charged. The levels are b's, widened by
  ρ·|(g1, g2) across the sweep| since a point off the axis leans out of its
  level. A lift that is exactly a rotation of b's has σ = 1 and no lean, so the
  cover is the exact disc of the largest ρ; one that is a rotation by rounding
  widens the disc by about 1e-15 relative, which a hole of the same radius does
  not survive;
- a vertex of the meridian that only touches b's level, such as the near face
  of a shoulder that starts on the knob's top level, bounds no interior inside
  the open levels and is left out;
- a cutting prism removes material from the cover when its levels, moved by the
  same offset, hold the levels the cover is read over: either it shares b's
  exact linear lift, or it is one axis-aligned rectangle of whole lines on a
  frame whose three axes are b's, signed and permuted, compared before one
  placement both carry bit for bit or in the world when both lifts are exactly
  orthonormal, where it is a box across b's sweep with a rectangle section.
  A cutter that does not hold the levels is left out, which only widens the
  cover.

prismcells' `SectionWithinUnion` then asks Sketch whether every cell of the
cover lies in the union of the outer regions: b's hole loops, each walked
counter-clockwise as a region, and the cutters' sections. The outer regions may
overlap, so membership is Sketch's containment verdicts against the declared
loops alone, and the edge-direction reading, which takes each outer edge to
bound the family, only falsifies the cover's side when it reads at all. A cover that is one whole circle equal, centre and radius bit
for bit, to a hole-free outer region lies in it without the arrangement, which
cannot hold two coincident circles.

The step is sound because a point p of a's interior, at a level of b's open
interval, lies in the target's cover, outside every cutter that holds the levels
(a Cut result's interior misses its closed tool), and has its section point
outside the closed hole of b if p is in b's interior. The cover's section lies
in the closure of the union of the holes and the cutters' sections, so p's
section point lies in the closure of one of them, which it cannot, being
outside the closed holes and the closed cutter sections. The interiors share no
point. Every number that enters the cover is an upper bound or an exact
rational, so the disc and the levels only widen.

A pair whose cover touches a hole on a lift that is not exactly a rotation of
b's stays open: the widened disc crosses the hole by rounding, as for a
shaft pressed into a bore of its own radius under two inexact rotations.

**The piece separator** (`internal/verifyrun/axial_section.go`) reads a Cut
chain down to a prism, whose target a turned screw sits across, against a Revolve
the meridian separator reads. The Revolve lies in the revolution of its meridian,
which splits at the meridian's vertex levels into slabs, each in a cylinder about
the axis of the largest ρ the slab holds. The pair is decided when every slab is
apart from the chain, a slab by either of:

- the cylinder lies in a whole-circle prism of the chain over the levels its
  target reaches along the axis, which is the Cut-tool meridian with the slab as
  the Revolve's region: the Cut result misses its tool's solid. A shank in the
  hole its own tool bored is decided this way;
- the same tool holds every point of the cylinder that lies in the target, read
  per exact prism part of the target (a Union's operands, whatever displacement
  the Union's own record holds). The tool's axis is the slab's exactly, its radius
  at least the slab's, and each part contributes the range of the slab's axial
  coordinate over its section's outline inside the slab's closed strip across the
  axis: whole lines clipped to the strip by their ends, and a circle or an arc,
  whose box leaves the strip or does not, by its box, an arc's circle the one
  through its start about its centre. The
  tool's levels must hold the union of those ranges. A plate across a shank is
  held by a hole that runs past it though a knuckle beside the plate stands
  higher than the hole;
- the cylinder's footprint on the target's section plane has no interior in
  common with the target's section. The axis must lie in that plane along one of
  its axes, the target sweep across it, and both lifts be exactly orthonormal,
  in the coordinates before one placement the two carry bit for bit or in the
  world. The footprint is the rectangle the cylinder projects to along the sweep,
  its edges rounded outward to floats; a point of both interiors would project
  into both sections' interiors, which Sketch has ruled out
  (`prismcells.SectionsApart`), for every exact prism part of the target. Two
  sections whose outline boxes share no interior along a section axis, touching
  allowed, are apart without the arrangement; a circle tangent to the rectangle's
  edge, which the arrangement cannot certify, is decided this way. A head flush on a bracket's face, though its
  revolution reaches a knuckle beside it, is decided this way, as is a cylinder
  whose reach along the sweep misses the target's levels.

The step is sound because the Revolve lies in the union of its slabs' cylinders
and every slab's cylinder, less the chain's tools where the first reading uses
them, misses the target's interior. The first reading needs the target's
axial range, which reads an arc of the target's section by the support of its
circle.

**The box algebra separator** (`internal/verifyrun/box_algebra.go`) reads two
Cut chains A = A0 less the tools T1 and B = B0 less the tools T2, each target a
prism or a Union's prism operands, each tool a prism. It runs at a Cut result.
The chains miss each other when every pair of target parts (A0ᵢ, B0ⱼ) shares
only a region that the tools cover. Each target part is bounded by the box of
its section outline over its levels in its own coordinates, and each tool must
be exactly a box in its own: one axis-aligned rectangle of whole lines over its
levels. A box is a convex solid in the world: its prism's lift is an exact
rational map L, so the box is the intersection of the six half-spaces
e_k·L⁻¹(x − t) ≤ hi_k and ≥ lo_k with L⁻¹ solved exactly, and L need not be
orthonormal. A turned box and a square one are then both convex polytopes. The
shared region A0ᵢ ∩ B0ⱼ is covered when it lies in the union of the tool
polytopes, decided by splitting it with the first tool's planes into the part
inside that tool and the pieces beyond each plane, each piece covered by the
remaining tools; a piece with no volume (its vertices, three planes meeting in a
point, lie in one plane) is covered, and the split is charged to the work
budget.

The step is sound because a point of both interiors lies in the interior of
A0ᵢ and of B0ⱼ, so inside their bounding boxes' intersection, and outside the
closed tools of both chains, a Cut result's interior missing its closed tool.
It cannot lie in the union of the tool polytopes. A tool that is no exact box,
or a tool the chain holds as a mesh Union carrying a geometry-moving warning (a
gap-shifted Union, whose set is not its operands' union), is left out, which
only shrinks the cover. Two parts half-lapped into each other, each cut where
the other stays, are decided when each is cut by its lap boxes one at a time,
and when a turned part and its lap tool share one lift, so that their faces
coincide exactly.

Like the coaxial separator, they prove no gap.

### 3.3 Full containment

A strict full-containment certificate proves both the relation and the overlap
volume without a boolean. §4 defines it.

### 3.4 Read-only intersection

Three evaluators answer here, tried in that order. The read-only analytic
`OpIntersect` dispatch of `agent-docs/prism-boolean-design.md` resolves an admitted
co-directional coplanar prism pair from the recorded sections and publishes
the resolved payload's own volume (§5.2). A pair it does not admit next
reaches that document's §4.5 overlap-area reading, which measures the same
recorded sections without building a body and so answers an overlap covering
any number of disjoint regions. Every pair neither admits reaches the
exact-predicate mesh boolean, which computes the held intersection; its exact
rational volume integral and symmetric-difference bound yield a proven
interval for the true overlap. §5–§6 define when any of the three intervals
proves a row.

## 4. Full-containment certificate

If one entire body lies in the other's material, their intersection is the
contained body. Reuse that body's existing `Volume` measurement; do not
tessellate it again and do not manufacture a new bound.

The certificate requires all of these:

1. Both operands are proven solids.
2. Their boundaries are proven disjoint with a **strictly positive** boundary
   clearance. A zero-distance or undecided contact cannot enter this path.
3. Both bodies are shipped analytic payloads represented by `bodyGeom`, and
   each has exactly one material lump. Multi-lump/faceted bodies skip this
   certificate and proceed to §5.
4. Choose one deterministic witness from **every** shell of both operands,
   void shells included.
5. `pointInBody` proves **both** directions: every witness of `inner` lies in
   `outer` material, **and** every witness of `outer` — its void shells among
   them — lies outside `inner` material. A witness proven in an `outer` void is
   outside `outer` material and fails full containment. A cast that grazes or
   exhausts its direction ladder is undecided.

With disjoint boundaries, membership is constant on each connected component of
one body minus the other body's shells — not throughout the material lump,
which the other body's shells can cut into several components. Each shell is
connected and misses the other boundary, so it lies wholly in one such
component and its single witness decides that whole shell.

Proving every `outer` witness outside `inner` is what makes `inner`'s own
witnesses speak for all of it: `outer`'s boundary then misses `inner`
entirely, so `inner` lies in one component and is wholly inside or wholly
outside `outer`. Without that direction, an `outer` void shell can sit inside
`inner`'s material and carve away a part of it that no `inner` witness reports
— and the reused volume would overstate the true overlap. `inner`'s own voids
need no separate treatment: they remove material from `inner`, and the
intersection with `outer` removes the same void.

If A's lump is inside B, report A's volume. If B's lump is inside A, report B's.
Strict boundary separation prevents both directions from holding for non-empty
solids.

**All multi-lump analytic containment is staged.** A multi-lump body is faceted,
such as a mesh Boolean result or a mesh solid of several pieces, or an analytic
prism group. `bodyGeom` builds no model of a body with more than one lump, so such
a body skips analytic nesting entirely and proceeds to read-only intersection; an
unsupported or unmeasurable intersection reads `Suspect`. Never reuse a whole
multi-lump body's volume from a partial witness set, and never sum only known
inside lumps.

Containment volume keeps the contained body's `Value`, `Exactness`, and
`Bound` unchanged. The containment proof itself establishes positive overlap,
so this path may emit the row even when that existing volume interval reaches
zero. The coarse measurement still fails §8's tolerance gate; the report
remains `Interfering`, which outranks `Suspect`.

### 4.1 Coincident analytic bodies

Strict containment cannot classify equal bodies because their boundaries
coincide. An exact set-equality certificate may reuse one operand's volume on
the same grounds: `A = B` proves `A ∩ B = A`.

The shipped certificate is analytic and exact. It compares the payload fields
that determine the represented point set using exact equality, never samples,
bounding boxes, tolerances, or topology counts. Derived metadata that
does not change that set — `cupPayload.Thickness`/`Sense` and
`prismPayload.BlendSegs`/`BlendKind` — is deliberately excluded. Equal
represented-set fields in `prismPayload`, `cupPayload`, or `revolvePayload`
prove identical represented sets.

For prisms, both endpoint levels must have equal exact denotations. For cups,
the open, outer, and cavity levels must each have equal exact denotations.
Missing or stale displaced-level evidence withholds the certificate.

The certificate additionally requires each operand's record to BE the set it
denotes. A `prismPayload` carrying a nonzero section displacement
(`agent-docs/prism-boolean-design.md` §7), or a `cupPayload` carrying a nonzero
offset displacement (`agent-docs/modify-design.md` §9), denotes a set its record is
only within that displacement of, so two equal such records say nothing about
the two sets: the certificate withholds and the pair stays undecided. That document's
Implementation notes own the per-consumer list this rule belongs to, and this
section does not restate its derivation.

Broader equality is staged. A future normalizer may prove harmless
representation choices equivalent:

- a closed loop's cyclic start point;
- hole order;
- an axis line's direction sign;
- a full revolution's angular start;
- a sweep interval's endpoint spelling.

That broader normalizer MUST compare by payload:

| Payload | Equality certificate |
|---|---|
| `prismPayload` | same world plane and sweep direction; same signed world sweep interval; same normalized line/arc material region |
| `cupPayload` | same outer region and interval; same cavity region and interval; same accumulated rigid placement |
| `revolvePayload` | same world axis line; same normalized meridian material region; same angular point set — equal partial interval or both full |

Current coordinates and curve parameters are compared as exact stored values.
A near match or structurally different record stays undecided. A future
normalizer may add adaptive-exact comparisons only when it proves set identity.

Do not apply this certificate to `facetedPayload`. Equal held polygons do not
prove the two bounded source parts equal. Their read-only intersection still
uses §5–§6.

When equality is certified, emit A's existing volume measurement, where A is
the first operand in stable pair order. Do not choose opportunistically by
bound width; stable pair order makes repeated reports byte-stable. The set
identity proves positive overlap independently of the volume interval, exactly
as strict containment does.

## 5. Read-only boolean evaluation

Factor the mesh boolean into geometry evaluation and document commit:

```go
func evaluateBoolean(
    ctx context.Context,
    op operationKind,
    a, b *Body,
) (booleanEvaluation, error) // no document write

func performBoolean(
    ctx context.Context,
    op operationKind,
    a, b *Body,
) (*Body, error) // public commit path
```

Names are illustrative; the split is normative.

`evaluateBoolean` performs only geometry work:

1. derive pair chord tolerance and pair diameter;
2. tessellate operands — at the pair's chord tolerance for a public boolean
   (raised to a restating operand's held floor), at each body's own §5.3
   chord under `Verify`;
3. map source faces;
4. prepare exact-predicate meshes;
5. run the hidden-tangency refusal;
6. classify and cut the mesh intersection;
7. stitch, conform, weld, and audit the held result;
8. compose chord and rounding bounds;
9. integrate held volume in exact rational arithmetic.

It MUST NOT call `nextProducerID`, advance provenance, retire an operand, register a
body, or expose a transient result through the document. Its result contains
the held facets and every bound input needed by either caller.

The public booleans keep consuming behavior:

1. gate nil, foreign, retired, and identical operands;
2. call `evaluateBoolean(ctx, op, a, b)`;
3. build and audit the public faceted body with `ctx` and the next real step reference;
4. append the boolean step and retire/register atomically.

Cancellation before step 4 returns
`ctx.Err()` unchanged and leaves the document and operands unchanged.

`Verify` calls `booleandispatch.AnalyticIntersect` — §5.2's read-only
analytic twin of the same `OpIntersect` dispatch `performBoolean` runs — then
`agent-docs/prism-boolean-design.md` §4.5's overlap-area reading for a pair that
twin does not admit, then `evaluateBoolean(ctx, OpIntersect, a, b)` for a pair
neither admits. It consumes only the volume result from whichever path
answers.

The twin reaches its own answer by building the admitted analytic payload, so
it reads `nextProducerID` to name that body, but it never commits: provenance
does not advance, no operand retires, no body registers, and the built body never
leaves the evaluator. The overlap-area reading builds one such payload per
measured arrangement cell and keeps the same promise for every one of them.
None of the three paths builds or registers a transient `Body` in the
document.

The order is normative. The twin runs first so that every pair it already
answers keeps its existing measurement unchanged; the overlap-area reading
picks up only pairs that previously reached the mesh path or no path at all.

### 5.1 One volume integrator

Extract the exact rational volume loop from faceted-body construction into one
private helper used by both callers. Given the stitched, oriented closed mesh
and the composed symmetric-difference allowance, it returns:

```go
Measurement{
    Value:     units.CubicMillimeters(Vheld),
    Exactness: exactnessOf(E),
    Bound:     units.CubicMillimeters(E),
}
```

`E` is the existing boolean volume allowance:

- operand A chord symmetric difference;
- operand B chord symmetric difference;
- volume swept by final float rounding over the **pre-round** stitched area;
- exact-rational-to-float rounding of `Vheld`.

The boolean rim displacement does not replace this allowance. The
symmetric-difference identity is the volume proof; the trim-amplified rim bound
continues to govern the public result's boundary measurements.

Both callers MUST use the same held facets, orientation audit, volume integral,
and bound helpers. A verification-only approximate volume formula is forbidden.

### 5.2 Positive-area coplanar patches

The current symmetric triangle classifier refuses a two-dimensional coplanar
intersection. That refusal stays correct until this complete replacement
lands; it must not be weakened one pair at a time.
`agent-docs/prism-boolean-design.md` §14 PR4 wires `measuredInterference` to the
same read-only analytic `OpIntersect` dispatch `performBoolean` uses
(`booleandispatch.AnalyticIntersect`), so an admitted coplanar,
co-directional prism pair — `Union`'s select-all path, `Cut`/`Intersect`'s
clean-nesting sub-case, or (since PR3) `Cut`/`Intersect`'s crossing sub-case,
§4.2 — is superseded here and no longer reaches the mesh path at all. PR5
extends that supersession to a coplanar pair whose overlap covers two or more
disjoint regions, through that document's §4.5 overlap-area reading: the
reading measures the arrangement cells both operands claim and sums their
volumes, so it needs no single assembled section and no body, and a pair of
meshing gears — whose overlap is always multi-region — is measured rather
than deferred to a mesh refusal. A crossing pair whose operands carry a
section displacement or walk charge, or whose re-expression is nonidentity (a
gear moved with `Transform`), is measured too: that document's §3.4 charges
each crossing the displacement it can amplify
(`agent-docs/general-boolean-design.md` §3 A6), and sends one too close to
tangent to bound to the mesh path below. A pair neither analytic
path admits still falls back to `evaluateBoolean`'s read-only mesh
intersection unchanged, so this section still governs every coplanar pair the
mesh path receives:
non-prism payloads, non-coplanar or non-co-directional prisms, and every pair
the entry gate or the region resolution declines.

Coplanar breadth support constructs one exact 2D arrangement per coplanar face
patch in the dominant-axis projection already used by the boolean's rational
cutting code:

1. Intersect projected triangle boundaries with exact rational predicates.
2. Split them into maximal positive-area cells. Edge-only and vertex-only cells
   remain contact cases.
3. For each cell, read both operands' outward material side from the oriented
   adjacent facets. Never infer it from triangle input order alone.
4. Opposing material sides describe boundary-only contact at that cell. Keep it
   out of the intersection volume and pass the complete contact set to the
   touching proof.
5. Matching material sides describe a coincident intersection boundary. Keep
   exactly one copy, choosing operand A in stable pair order, and connect its
   cell edges to the non-coplanar cut regions.
6. Audit every arrangement edge for two-sided adjacency after the merge. A
   branching edge, an unclassified neighboring cell, mixed side evidence, or a
   cell the rational arrangement cannot close is an expected undecided contact
   outcome, never a guessed keep/drop.

This section owes the hidden-tangency gate (`meshbool.FacesNearMiss`, `internal/meshbool/depth_witness.go`;
`agent-docs/evaluator-design.md` §9) a **constraint**, not a permission. That gate
returns `near = false` for the whole face pair on the first facet pair it
classifies as a positive-area coplanar overlap, deferring the pair to the mesh
pass's own refusal of a coplanar face-on-face tangency. The deferral is sound
only while that refusal happens. Once §11's PR4 classifies material sides over a
patch instead of refusing it, the same `near = false` rests on a refusal that no
longer runs, and the whole face pair clears the gate with no near-miss proof
behind it.

§11's PR4 MUST therefore settle the near-miss question for a coplanar carrier
pair before it removes that refusal, and carrier coplanarity ALONE does not
settle it. A planar face's carrier plane is exact, but its outline is not: near
a curved rim the held outline is inscribed, so two operands whose true rims
touch can leave their held outlines a chord apart, and the arrangement above —
projected triangle boundaries split into maximal positive-area cells — then has
no positive-area cell to classify at all. The cells carry no outline allowance
of their own that would cover that gap.

The patch relation does not decide the whole pair by itself. Opposing cells are
`boundaryaudit.PairTouching` only when the analytic/contact pass proves no overlap elsewhere.
Matching cells may bound a positive-volume intersection only after the complete
stitched result passes §5 and its volume clears §6.

### 5.3 One mesh per body in `Verify`

A public boolean meshes both operands at the pair's own chord tolerance:
`boolChordFactor` times the diameter of the union of the two operands'
bounds-inflated boxes, raised past each operand's own reserved floor (its
section displacement, a revolve's coordinate stages, a coil's or a mitred
sweep's held `delta`). `Verify` does not. Each
body is meshed **once per `Verify` call**, at one chord, and that one mesh — its
facet-contact audit included — serves every pair of the body that reaches the
mesh path. Meshing per pair would mesh and audit a body in a 20-body star 19
times, at 19 slightly different tolerances.

**The rule.** A body's chord is the least pair tolerance over its
*candidate pairs*: the solid pairs, in §2's job list, whose bounds-inflated
boxes are not separated. Those are the only pairs that can reach the mesh path,
and the set is known before any pair runs; the pairs that actually reach it are
known only after the analytic kernel has run on them, too late to fix a chord
that pairs running beside them share. The chord is computed once, before the
worker pool starts, from the same per-operand shares the pair tolerance is
built from. In a two-body document each body's chord is therefore the pair's
own tolerance, the one a public boolean meshes both operands at.

**Restatements keep the public booleans' rule.** A body whose tessellation
restates a held mesh instead of chording — a faceted Boolean result, a mitred
sweep — has nothing to share: it returns the same held facets at any tolerance
at or above its held floor. Such a body is meshed per pair exactly as a public
boolean meshes it, at the pair's tolerance raised to its held floor, and each
pair gates the held facets it touches against that pair's own tolerance
(`agent-docs/api-design.md` §8 "The chain depth",
`agent-docs/faceted-vertex-bounds-design.md` §5). A shared body is never gated: it
restates nothing.

**The bound stays sound.** The requested tolerance only steers how finely a
body is chorded. §5.1's allowance `E` reads each operand's symmetric-difference
bound from the mesh that was actually built, never from the tolerance asked
for: each operand's own proven `volSymDiff`, the rounding of the stitched
result, and the conversion of `Vheld`. The symmetric-difference identity
`|1_{A∩B} − 1_{A'∩B'}| ≤ |1_A − 1_A'| + |1_B − 1_B'|` holds for any two held
operands A' and B', whatever tolerances they were chorded at, so the interval
`[V − E, V + E]` encloses the true overlap whether or not the two meshes share
a tolerance. The hidden-tangency gate and `rimDelta` likewise read each mesh's
own boundary bound, and `rimDelta` keeps the pair's own diameter. Nothing in
the pipeline assumes the two operands were chorded alike.

**What the reading becomes.** A body's chord is never coarser than the
tolerance any of its pairs would have derived alone, because the least of a set
is at most each member. Each operand mesh of a pair is therefore at least as
fine as the pair would have asked for, and the published `Volume` and `Bound`
of an overlap can differ from the reading at the pair's own tolerance. Both
intervals enclose the true overlap, so the two values differ by at most the
sum of the two bounds. The bound composes each operand's own allowance at its
body chord, which is usually the smaller of the two.

The §8 tolerance gate reads no chord: its reference is the pair's diameter and
the operands' areas. A body's chord does depend on which other
bodies' boxes meet its own, so adding a body that touches it can change the
reading of its other pairs. For one document the reading is fixed, whatever
the worker count or the order the pairs finish in.

**The cache.** The per-call cache holds one entry per sharing body: the chord,
and the outcome of the one tessellation that builds the mesh, run under a
once-only guard. The entries are created before the pool starts and only read
afterwards, so workers that ask for the same body at once wait on one build
rather than each building their own. A failed build is kept like a mesh: the
build is deterministic, so every pair of the body would fail the same way, and
each pair still maps the error to its own operand index for §7.1's
diagnostics. The cache lives for one `Verify` call and is then dropped. The
body's own one-entry tessellation cache (`agent-docs/tessellation-design.md` §1.1)
also holds the mesh, keyed by the chord, so a later `Verify` of the same
document finds it there.

A pair whose chord share cannot be read — a body with no bounds, a pair with
no extent — contributes nothing to any chord. If it reaches the mesh path, it
derives its own pair tolerance there and fails exactly as it would without the
cache. The motion check (`agent-docs/motion-check-design.md`) measures one transient
pose against one static body at a time and keeps the pair's own tolerance.

## 6. Positive-volume gate

Let an intersection evaluation return `V = abs(Volume.Value)` and
`E = Volume.Bound`. The true overlap volume lies in the non-negative interval

```text
[max(0, V - E), V + E].
```

Emit an `Interference` row from this path only when `V - E > 0`. That strict
inequality proves the true intersection has positive volume. Copy the
measurement unchanged into the row.

These outcomes do **not** emit a row:

| Outcome | Why | Pair result |
|---|---|---|
| held intersection empty after a gap shift | the shift moved a held mesh off the true operand, so the empty result says nothing of the true pair | `Suspect` unless an earlier path proved disjoint/touching |
| `V - E <= 0` | interval admits zero; held overlap is below this evaluator's proof reach | `Suspect` |
| tangent/grazing/coplanar contact refused | topology classifier did not prove a positive-volume intersection | `Suspect` unless clearance certified touching |
| operand/result unsupported | evaluator lacks a complete path | `Suspect` |

An unshifted empty held intersection proves the interiors disjoint
(`boundaryaudit.InterferenceProvenDisjoint`, §6.1). The pair takes no row and
no diagnostic, or an unmeasured-clearance diagnostic under `WithClearances()`.

Do not turn a mesh witness point, an intersection segment, a nesting witness,
or a positive held volume alone into a row. Every row carries the bounded
overlap volume the public contract promises.

The set-identity paths are the only exceptions to `V - E > 0`: strict full
containment proves `inner ∩ outer = inner`, and the §4.1 analytic equality
certificate proves `A ∩ B = A`. Each independently proves positive overlap,
and the existing body volume is the quantity, not the proof of existence.

### 6.1 Why an empty gated intersection proves disjointness

The rule strengthens `Verify`: a pair it used to leave `Suspect` now passes. The
claim, its premises and its proof follow in full.

**Claim.** Let `A` and `B` be the true solids and `HA`, `HB` their held
solids, the closed polyhedra `Verify` meshes them as. Suppose the read-only
intersection `OpIntersect(A, B)` ran with no gap shift, passed the proximity
gate (`meshbool.RefuseUndecidableProximity`), and kept no facet
(`meshbool.BooleanExpectedEmpty` with no `AppliedShift`). Then the interiors of
`A` and `B` are disjoint.

**Premises.** The proof reads only what the evaluator already proves.

- P1, the operand bound. Every held facet `f` carries a bound `d(f)`: its
  face's `sourceBound`, or a finer per-facet or per-region figure where one is
  proven (`agent-docs/faceted-vertex-bounds-design.md` §2,
  `agent-docs/tessellation-design.md` §2's planar record). The true boundary
  `∂A` is the end of a homotopy that starts at the held boundary `∂HA` and
  moves each point of facet `f` by at most `d(f)`. The tessellation contract
  states every bound as such a displacement, two-sided: every true boundary
  point lies within `d(f)` of some held facet `f`, and every held point within
  `d(f)` of the true boundary (tessellation §2's `sourceBound`). The
  construction homotopies of tessellation §11 realize it, and every other
  bound-based proof in `Verify`, such as the clearance kernel's widening of a
  gap by both displacements, reads the same premise.
- P2, the gate. For every face pair the gate visits, it gathers the facet
  pairs `(a, b)` that come within the slack of their faces. It narrows that
  slack to the contacting facets' own bounds, `d(a) + d(b)` at most. Then
  either it proves an interpenetration of the held facets deeper than that
  slack, or it finds no facet pair within it, or it refuses. A face pair with
  zero slack is held exactly, and the mesh pass's exact predicates decide it.
  A coplanar face-on-face overlap is deferred to the mesh pass, which refuses
  it ("two operand facets overlap in one plane") instead of returning empty.
- P3, the mesh pass. `OpIntersect` over the held solids keeps every facet of
  `HA` inside `HB`, every facet of `HB` inside `HA`, and the cut pieces of
  every crossing. It keeps none exactly when `HA ∩ HB` has no interior and no
  boundary crossing.

**Step 1: no held facets are close.** A gate witness of interpenetration
deeper than the slack places held material of one solid strictly inside the
other, so P3 would keep facets. The result kept none, so the gate found no
such witness. It did not refuse either, and no coplanar overlap reached the
mesh pass (P2). So every held facet pair `(a, b)` stands farther apart than
`d(a) + d(b)`, or both facets are exact and the exact predicates saw them meet
in no interior point.

**Step 2: the held solids are apart.** By P3 and the empty result, `HA` and
`HB` share no interior point and their boundaries do not cross. With Step 1
the boundaries do not even touch, except where exact faces meet with opposed
material, which the exact predicates decided. So `HA` and `HB` are disjoint
closed sets, or meet only on exact faces from opposite sides. In particular
neither lies inside the other: nesting would keep every facet of the inner
one.

**Step 3: each true solid stays in its held solid's band.** Let `NA` be the
set of points within `d(f)` of some facet `f` of `HA`, and `NB` likewise. By
P1 the homotopy from `∂HA` to `∂A` sweeps only points of `NA`. A point outside
`NA` is never crossed by the moving boundary, so its winding number about the
boundary, and with it its membership, is the same in `A` as in `HA`. Hence
`A ⊆ HA ∪ NA` and `B ⊆ HB ∪ NB`.

**Step 4: the bands keep apart.** Take a point `x` in both `A` and `B`. By
Step 3 it lies in `HA ∪ NA` and in `HB ∪ NB`. Every case is impossible:

- `x ∈ HA ∩ HB` contradicts Step 2.
- `x ∈ NA ∩ NB` puts `x` within `d(a)` of a facet `a` and within `d(b)` of a
  facet `b`, so `a` and `b` are within `d(a) + d(b)` of each other, against
  Step 1.
- `x ∈ HA ∩ NB`: `x` lies within `d(b)` of a facet `b` of `HB`, at a point `y`
  of `∂HB`. `y` is not in `HA`, by Step 2, so the segment from `x` to `y`
  leaves `HA` through some point `z` of a facet `a` of `HA`. Then `a` and `b`
  are within `|xy| ≤ d(b)` of each other, against Step 1. `x ∈ NA ∩ HB` is the
  same with the roles swapped.

The exact-face exception of Step 1 adds only boundary points, where both
solids' exact faces meet with opposed material, and no interior point. So no
interior point lies in both `A` and `B`. ∎

**What the rule does not claim.** It proves no gap: the true boundaries may
touch along exact faces, and the margin Step 1 finds is not measured. A gap
shift moves a held mesh off its true operand, so P1 fails for the moved
operand and a shifted empty result proves nothing. Every other empty result,
from a point-cone record or a public Boolean's dispatch, never reaches this
path.

## 7. Errors and cancellation

### 7.1 Expected undecided outcomes

The read-only evaluator MUST distinguish expected geometric non-results from
internal failures with private typed outcomes, not sentinel identity alone and
never message matching:

- empty held intersection;
- contact/graze/coplanar arrangement the mesh boolean refuses;
- evaluator staging (`ErrUnsupported`);
- a valid operand whose chording is too coarse to prove its own topology —
  loops that close within their chord bounds of each other, a bridge that
  pinches, a chorded boundary whose ear clipping stalls. The evaluator owns the
  chord tolerance here, so the caller cannot act on the refusal;
- positive held intersection whose bound does not clear zero.

`performBoolean` maps these to its existing public errors (`ErrBooleanFailed`,
`ErrDegenerate`, or `ErrUnsupported`). `Verify` maps them to an undecided pair.
For diagnostics, preserve the private reason: pre-contact operand staging emits
`DiagUnsupportedPairPayload` and names the operand; contact policy emits
`DiagUnsupportedPairContact`; later pipeline reach emits
`DiagUnsupportedPairPipeline`. Each message states the matching corrective
action. Verify emits only the matching cause-specific code.

An invariant failure — inconsistent source mapping, an unclosed stitched mesh,
an impossible shell relation, a failed exact predicate, or a non-positive
signed volume on a supposedly non-empty oriented result — is not ordinary
uncertainty. `Verify` returns that error and no report. It MUST NOT hide a broken
evaluator as `Suspect`.

### 7.2 Context and work

`Verify` passes its context through the entire read-only path. After document
and option validation, return `ctx.Err()` unchanged once canceled.

Check cancellation:

- before and after every §5 phase;
- in every O(facet²), O(edge²), component, shell, and containment loop;
- at every branch-and-bound/refinement subdivision;
- while cutting facets and while building/stitching adjacency.

Leaf exact predicates remain context-free. Their callers use a shared bounded
work counter and check `ctx` at least once per 256 candidate operations. Phase
boundaries check unconditionally. This gives prompt cancellation without
threading context through arithmetic primitives or changing deterministic
geometry decisions.

`Union` / `Cut` / `Intersect` propagate their caller
context through evaluation and faceted-body construction.

The mesh fallback has no constant-work promise. `meshBoolean` checks every
pair of operand facets, using the facet boxes to skip exact predicates only
after each pair-box check. Its pair-box work is therefore the number of facets
in A multiplied by the number in B. `Verify` can pay that cost for every
proven-solid pair that box, analytic clearance, containment, equality, and the
§3.4 analytic `OpIntersect` dispatch leave unresolved. Verification §1.2 owns
the public deadline guidance. The existing
context is the public control for this work; do not add a separate work-limit
or progress API.

## 8. Interference tolerance gate

An emitted row makes the report `Interfering` regardless of measurement
coarseness. The row's measurement is still judged because nothing in a report
is exempt.

Use verification's ordinary scalar rule:

```text
Bound <= rel × Ref
Ref = max(abs(Volume.Value), Quantum)
Quantum = δ × (AreaA + AreaB)
δ = 1e-9 × Dpair
```

`Dpair` is the diameter of the union of the two operand point sets. `AreaA` and
`AreaB` are the operands' own surface-area readings. The overlap boundary lies
on operand skins, so their summed area bounds the surface displaced at the
noise floor.

Keep this reference pair-local. Do not use either body's volume, either body's
box volume, the document diameter, or the transient intersection's faceted
area. The transient area can omit or add facets exactly where the approximation
is uncertain; the operand skins are the proof reference.

The gate affects the pair's trust finding, but status precedence remains
`Interfering > Suspect`. A caller can inspect the row's `Exactness` and `Bound`
even though `Passed()` is already false because the bodies overlap.

## 9. Coverage and refusal

The proof paths use operand capabilities, with one history-backed Cut-tool
exception:

| Operand capability | Available path |
|---|---|
| bounds | box disjointness |
| analytic boundary model + certified casts | clearance/contact/full-containment proof |
| exact planar held boundary with a displacement `δ` (a mitred sweep, a faceted Boolean result, a coil) against an exact planar partner | §3.2's exact planar arm: disjoint with a gap, touching at `δ = 0`, or overlap through a deep vertex |
| unadjusted mesh Cut result and an exact same-set analytic copy of its consumed tool | operation history and bounded set equality prove non-overlap, without measuring a gap |
| tessellation accepted by mesh boolean | read-only intersection volume |
| neither boundary model nor tessellation | undecided → `Suspect` |

The other rows describe operand capabilities. The §3.4 analytic `OpIntersect`
dispatch and the overlap-area reading behind it sit on top of them as
pair-level paths: an admitted co-directional coplanar prism pair measures its
overlap from the two recorded sections and needs no tessellation row at all,
whether that overlap is one region or several.

A prism, a filleted/chamfered prism, and a tube share `prismPayload` and use the
same paths where their section displacement is zero. A `prismPayload` carrying a
nonzero displacement (`agent-docs/prism-boolean-design.md` §7) gets no `bodyGeom`
model, so no analytic clearance, contact or full-containment path is available
to it, and §4.1's equality certificate withholds separately. Such a body still
reaches a read-only intersection path: the §3.4 analytic `OpIntersect` dispatch
where the pair admits it — that path reads the recorded sections rather than a
`bodyGeom` model, and composes the displacement into the bound it publishes —
and otherwise the intersection its tessellation supports. A pair neither
measures reads `Suspect`. A cup uses box proofs and the read-only boolean once
its shipped cup tessellation is accepted; analytic clearance/full containment
remain staged until `bodyGeom` supports `cupPayload`. A faceted boolean result
uses its held mesh and carried bounds. A revolve uses analytic clearance/full
containment where supported; read-only intersection stays staged until revolve
tessellation lands.

Do not claim curved coverage from planar facets alone. The hidden-tangency gate
and every operand chord allowance remain mandatory. If a curved pair lies
inside those bounds and no analytic proof settles it, the pair is `Suspect`.

## 10. Required tests

Every increment asserts geometry and report state, not only successful return.

### 10.1 Non-mutation and order

- Snapshot the next producer identity, live body pointers/order, and retired state before
  `Verify`; assert exact equality afterward on overlap, disjoint, undecided,
  error, and cancellation paths.
- Build at least three bodies; assert `Interferences` and `Clearances` follow
  document pair order, independent of proof path.
- Call `Verify` repeatedly; assert byte-stable report ordering and identical
  measurements.
- Verify one document with one pool worker and with several; assert the two
  reports equal field for field. Assert every pair a parallel pool proves sees
  one contact worker, and the one-worker walk sees the caller's count.

### 10.2 Relation

- separated boxes → no interference, `Sound` without asked clearances;
- certified touching caps → no interference, optional `Exact` zero clearance;
- transversal overlap → one interference row with `Value - Bound > 0`;
- nested single-lump body → row volume equals contained body volume exactly as
  a `Measurement` value;
- a single-lump body whose **void** shell lies inside the other body, whose
  boundaries are disjoint → no containment reuse: the outer body's void
  witness is proven inside, so the pair reports only what the read-only
  Boolean proves, else `Suspect`. A hole-free fixture cannot exercise this;
- fully or partially contained multi-lump faceted body → no analytic
  containment reuse; read-only Boolean proves the complete overlap or the pair
  stays `Suspect`;
- held overlap at or below its bound → no fabricated row, `Suspect`;
- identical/coincident and broad coplanar pairs stay `Suspect` until their
  stated increments land, then report the proved volume;
- a coplanar prism pair whose 2D outlines overlap in two or more disjoint
  regions reports one row whose volume is the summed overlap, proving the
  overlap-area reading answered rather than the mesh path — which refuses a
  coplanar pair outright, so any row at all on such a pair is the proof;
- a coplanar prism pair whose sections meet only along a shared wall reports
  no row and stays `Suspect`, so the reading never turns contact into a
  zero-volume interference.

### 10.3 Bounds and errors

- perturb analytic fixtures within the carried operand chord allowances; true
  overlap remains inside `Value ± Bound`;
- all-planar exact-rational case reaches zero volume bound only when its float
  conversion is exact;
- an expected unsupported/contact/empty outcome yields a report with
  `Suspect`;
- a forced closure/source/invariant failure makes `Verify` return an error;
- cancellation during tessellation, facet-pair classification, cutting,
  stitching, and containment returns `context.Canceled` or
  `context.DeadlineExceeded` and leaves the document unchanged.

### 10.4 One mesh per body

- three mutually crossing cylinders whose pairs derive different tolerances:
  each body is tessellated exactly once per `Verify` call, counted by an
  internal hook on every build, and each body's chord is the least of its
  pairs' tolerances;
- the same document's three overlap volumes each enclose the pair's true
  volume, computed independently by quadrature of the crossed-cylinder
  integral, and each proves positive;
- many goroutines asking the cache for one body at once receive one mesh from
  one build, under `-race`.

### 10.5 Gate

- judge `Interference.Volume` against pair diameter and summed operand areas;
- never use document size or transient intersection area;
- a row beyond tolerance remains `Interfering` by precedence while preserving
  its approximate measurement.

## 11. Increments

Each row is a PR-sized stage. An unanswered verification question reads
`Suspect`; no row claims work that has not landed.

| PR | Lands | Still `Suspect` |
|---|---|---|
| 1 | four-way pair relation; read-only `evaluateBoolean`; shared shell audit and rational volume/bound helper; transversal intersections for operands the mesh boolean accepts; stable ordering; context propagation | containment without a measurable mesh intersection; coincident/broad-coplanar arrangements; operands without tessellation |
| 2 | strict full-containment certificate for shipped single-lump analytic payloads; whole-contained-body volume reuse | all multi-lump containment unless the Boolean measures the complete intersection |
| 3 | exact structural equality certificate for analytic payloads, reusing the first equal body's volume | harmless alternate record spellings; non-identical broad coplanar overlap |
| 4 | coplanar breadth in the mesh classifier: classify material sides over every positive-area coplanar patch, keep crossing/overlap patches, and retain pure opposite-side contact as touching; settle the near-miss question §5.2 states this increment owes a coplanar carrier pair before removing the refusal that pair is deferred to | unsupported curved operands and unresolved curved tangencies |
| 5 | curved read-only intersection coverage after revolve tessellation, with chord bounds and the hidden-tangency refusal intact | contact or overlap whose proven interval still admits both zero and positive volume |
| 6 | multi-region analytic overlap: `agent-docs/prism-boolean-design.md` §4.5's overlap-area reading, entered after the §5 twin declines, measuring an admitted coplanar prism pair whose overlap covers any number of disjoint regions | a coplanar pair the prism entry gate or the region classification declines, an exactly-tangent pair, and every faceted or non-coplanar operand |
| 7 | §3.2's exact planar arm: a mitred sweep or a faceted Boolean result, against a prism, a stitched solid or each other, and a coil against a prism or a stitched solid, partitioned by the exact planar relation under the displacement band | a held touch or a held gap or overlap within the summed displacement; every pair holding a cup, a loft, a composite sweep, a cap-loop chamfer or a curved body |

## 12. Decisions

- Keep the existing public `Interference{A, B, Volume}` shape. Add no option,
  witness, selector, or modeling operation.
- Keep public booleans consuming. Share only their read-only geometry
  evaluation — the mesh `evaluateBoolean` and the analytic intersection twin
  `booleandispatch.AnalyticIntersect` (§5, §5.2) — never their commit.
- Let a proof path answer with a volume alone where a body is not needed.
  `agent-docs/prism-boolean-design.md` §4.5's overlap-area reading measures a
  multi-region overlap `Intersect` itself still cannot build, and that
  asymmetry between what `Verify` proves and what a public boolean returns is
  accepted rather than resolved by widening `prismPayload`.
- Reuse the boolean's exact rational volume and symmetric-difference bounds.
- Mesh each body once per `Verify` call, at the least tolerance any of its
  candidate pairs derives (§5.3). Restatements keep the public booleans' rule.
  Public booleans keep the pair's tolerance for both operands.
- Run the contact batches inside a parallel pair pool on the pair's own worker
  (§2): one layer of parallel workers, never two.
- Require strict positive lower overlap volume except under a certified
  containment or equality set identity.
- Treat an unshifted empty result as proven disjoint (§6); treat a shifted
  empty, contact, unsupported, and coarse results as undecided; propagate
  internal failures.
- Preserve document pair order.
- Stage all multi-lump analytic containment; use read-only intersection and
  never report a partial sum as total overlap.
- Check cancellation at phase boundaries and every 256 expensive candidates.

No design decision remains open in this increment. Revolve tessellation,
analytic cup boundary support, and broader contact certificates are explicit
prerequisites or staged coverage, not unresolved contract choices.
