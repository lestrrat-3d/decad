# Gap tolerance

Companion to `agent-docs/api-design.md` §2.1 and §6, `agent-docs/evaluator-design.md` §9 and
`agent-docs/faceted-vertex-bounds-design.md`. Owner of the document's gap tolerance: what it permits, what it
never permits, how every use is charged, and which operations read it.

## Router

| Question | Section |
|---|---|
| What does the option do? | §1 |
| What is the default and why? | §2 |
| What is the tolerance never allowed to do? | §3 |
| How is a use of it charged? | §4 |
| Which operations read it? | §5 |
| How does a Boolean use it on a coplanar contact? | §5.2 |
| How does `Stitch` use it? | §5.3 |
| Which refusals does it leave in place? | §6 |

This router is navigation, not authority.

## 1. The option

A `Document` carries one gap tolerance, a length in millimetres. `decad.New` takes it as a `DocumentOption`:

- `decad.WithGapTolerance(tol units.Value)` sets it to `tol`.
- `decad.ExactOnly()` sets it to zero. `WithGapTolerance` of a zero length is the same thing.
- With no option the tolerance is `gaptol.Default`, 0.01 mm (§2).
- Several gap options in one `New` call: the last one wins.

The tolerance is the **reach** of two repairs, and each runs only after the exact path has refused:

- The held-mesh embedding repair (§5.1). A result whose exact vertices round to float64 so that facets fold through
  each other is first repaired among the float-box corners of each exact point
  (`agent-docs/evaluator-design.md` §9). A mesh that search leaves folded gets a second stage in which each movable
  vertex of a failing pair may also sit up to the reach from its exact point.
- The shifted Boolean retry (§5.2). A mesh Boolean whose operands meet in coplanar facets is run again with the
  second operand moved by up to the reach.

With reach zero neither runs, and the refusal is the one the exact path gives.

An invalid option is reported by every operation that reads the tolerance, because `New` returns no error:

| Option value | Error |
|---|---|
| Not a length | `ErrUnitKind` |
| Not finite | `ErrNotFinite` |
| Negative | `ErrNegativeMagnitude` |
| A nil option | `ErrDegenerate` |

A refused call leaves the document unchanged, as every refusal does (`agent-docs/api-design.md` §6).

The tolerance rides on the `context.Context` an operation passes down (`internal/gaptol`). Evaluator code that no root
operation called reads zero, so it is exact-only.

## 2. The default

0.01 mm is one tenth of the smallest feature the unaided eye resolves at reading distance (about 0.1 mm), so a
move within it changes nothing a viewer can see. The renderer in `solidlens` uses the same stance: a facet narrower
than half a pixel at the model's distance draws no line. A caller that proves a
part against a stricter standard than the eye chooses `ExactOnly` or a smaller value.

## 3. What the tolerance never permits

- **It never widens a predicate.** Every embedding, contact and coincidence decision stays exact over the held
  binary64 values the evaluator ends with. The tolerance chooses where a held vertex, or a whole operand, sits, and
  the decision is then made exactly about that position: two facets still count as meeting only where exact
  arithmetic says they meet.
- **It never admits an answer an upstream package called approximate.** A fragment `sketch` reports uncertified
  (`BoundaryEdge.TExact` false) is still rejected (`agent-docs/sketch-seam-design.md`). The tolerance moves decad's
  own held vertices, and only vertices that stand for exactly one exact point.
- **It never welds.** A position another held vertex occupies is skipped, as in the float-box stage. A vertex that
  stands for several exact points never moves.
- **It never moves a vertex beyond the reach.** Every candidate lies within the reach of the exact point, and the
  caller charges the distance the search actually left.

`agent-docs/api-design.md` §2.1 rejects a kernel's "tolerant topology" because it hides a geometry that does not
meet. This design keeps topology exact and states the displacement: the moved vertex is part of the result's bound
(§4), so `Verify` judges it like any other bound.

## 4. How a use is charged

The distance from every exact point to the float that finally holds it is MEASURED after the search, exactly, and
every consumer composes that maximum as the rounding bound (`StitchedMesh.Round`, the placement's `VertexBound`, the
similarity image's `VertexBound`). The three sites that run the search charge the final position and no
estimate, so a reach move needs no new term: its distance is larger than an ulp, and the same measured gap carries it
into the swept-volume and perturbed-area terms.

A result whose search used the second stage therefore reads `Approximate`, with a vertex `Bound` that covers the
moved vertices. The shifted Boolean retry charges its move by widening the bounds of the moved operand's copy that
the result's composition reads (§5.2). `Verify` compares those bounds with the caller's `WithTolerance`, so a
result the gap tolerance kept can still read `Suspect`.

## 5. Where the tolerance runs

The root reads the tolerance in `placePayload`, `booleanBodyWith`, `performUnionMany`, `ScaledAt` and
`Document.Verify`, and puts it on the context. `Verify` hands it to the read-only boolean that measures each pair's
overlap, so a pair the exact predicates could not classify reads the overlap it has, with a bound that covers the
shift, instead of `DiagUnsupportedPairContact`. Every other operation, such as `Fillet`, `Shell` and `Loft`, reads no tolerance of its own. Four evaluator
sites read it:

| Site | Operations that reach it |
|---|---|
| `facetproof.KeepPlacedEmbedded`, a rigid placement of a faceted mesh | `Transform`, `Copy`, `PatternCopies`, `Patterned`, `MirroredCopy` |
| `facetproof` similarity image | `ScaledAt` |
| `meshbool.keepRoundedEmbedded`, the stitched result of a mesh boolean | `Union`, `Cut`, `Intersect`, and the booleans inside `Patterned` and the tool combinations |
| `booleaneval.Evaluate`'s shifted retry | `Union`, `Cut`, `Intersect`, the same inner booleans, and the pair readings of `Verify` |

The first three rows are the embedding repair, a stage of `meshbool.RepairHeldEmbedding`: `HeldRounding.Reach` is the
reach, and zero keeps the float-box search alone.

### 5.1 The search

1. The float-box stage runs to the end exactly as it does with no reach. A mesh it clears is not touched again,
   and its grid, pad and moves are those of the reach-free search.
2. A mesh it leaves folded gets stage 2. The grid is rebuilt with a pad of twice the largest distance a vertex can
   lie from its first rounding, counting the reach: `2·(maxDev + Reach)` plus rounding slack, so no move
   leaves a facet's cells.
3. Each offending movable vertex tries positions at `step·Reach` from its exact point for step in 1/64, 1/16, 1/4 and 1,
   nearest first. For each step it tries both senses of the unit normal of every incident facet and of every facet
   they fail against, then both senses of the coordinate axes. A collapsed facet has no normal, so the axes
   always apply.
4. A candidate is taken when it leaves fewer failing pairs among the vertex's facets than where the vertex sits.
   A step that clears every pair ends the vertex's search, so the nearest clean position wins and the charge is the
   smallest this search can give. Ties go to the earlier candidate.
5. Passes repeat as in the float-box stage. Each move strictly lowers the mesh's failing-pair count, so the search
   cannot cycle, and its order is fixed, so equal inputs give equal held meshes.

### 5.2 The shifted Boolean retry

A mesh Boolean refuses four contacts the exact predicates cannot classify. Two operand facets can overlap in one
plane, so no side of the other operand is proven for the shared region. An edge of one operand can graze along a facet
of the other. The operands can touch at an isolated point. Facets of a curved face of one operand can come within the
chord tolerance of the other operand's facets without provably crossing deeper than it (the proximity gate,
`meshbool.RefuseUndecidableProximity`). `meshbool.TouchContactError` carries every such contact it met (up to 64) as
the first operand's outward unit normal at the contact, whether it is Decisive, a Slack, and the second operand's
facet it meets. A coplanar pair is Decisive when the second operand's facet faces the opposite way, which puts the
operands on opposite sides of the plane (a face-on-face touch). A graze and a point touch are always Decisive. A
gate contact is Decisive when the first operand's face is curved, and its Slack is the sum of the largest facet bound
among the close facets of each face (never more than the faces' own bounds, which cover their worst facet). Any
other contact has Slack zero. `booleaneval.Evaluate` reads the error and
runs the Boolean again with the second operand moved:

1. Directions come from the contacts (`shiftDirections`). Every contact must open: the shift's component along each
   contact's normal is at least 0.02 in magnitude. A Decisive contact also fixes the sense. A Union moves the second
   operand inward along the first operand's normal, so bodies that touch join into one, along a face, an edge or at a
   corner. A Cut and an Intersect move it outward, so a touching Cut leaves its target whole and a touching Intersect
   is empty. With no Decisive contact, each contact is a flush wall or a gate contact on a flat face, and a gate
   contact cannot tell a body resting on the face from one flush beneath it. A Cut and an Intersect still need every
   such contact to open outward, so a tool flush with its target's wall pokes through it and leaves no skin, and a tool
   resting on it stays apart. A Union needs only that they open (`weightedBase`). Each contact that is not wanted
   pulls in the sense the pull so far leans along its normal. A contact the pull does not lean along takes the tie
   sense: inward when the Union has no Decisive contact, outward otherwise. The lean is the float dot product of the
   pull and the normal, so only an exactly zero product is a tie, and a nearly orthogonal normal follows the sign
   rounding gives it. With no Decisive contact nothing else brings the operands into overlap, so a body resting on a
   flat face sinks into it and joins into one solid, and a body flush beneath it sinks inside, where an outward pull
   would leave the resting body as a second lump above the face. A Decisive contact already moves the operands into
   overlap, so the contacts beside it, such as the flush top and bottom caps of two ribs whose curved and flat walls
   touch, only open, and they open outward on a tie. An inward tie there gains no connection, and on the line, arc and
   line ribs of `TestWebPathJoinsALineArcLineChain` it leaves the joined ribs with a mesh bound that the next Union of
   the chain refuses at the chain-depth gate (`RefuseCoarseHeldContact`). A pair of contacts that no direction opens in the needed sense,
   such as a tool flush with both walls of a slot, is opened by growth (item 5).

   The rule follows from the tolerant reading `A∘(B+s)` with `s·n` the shift along a contact's outward normal `n`.
   `A∘(B+s)` is the solid the result states, never `A∘B` (see "What a result of the retry states" below). A
   Union with `s·n < 0` overlaps a resting B by `|s·n|` and leaves a flush B inside A. With `s·n > 0` it leaves a
   gap under a resting B and a sliver of a flush B outside A. A Union that also holds a Decisive contact overlaps B
   there whichever way a contact that is not Decisive opens, so the sense of that contact decides only on which side
   of its face a sliver or a gap of the shift's length lies, and the result is one solid either way. A Cut with
   `s·n > 0` leaves A whole under a resting tool and opens a flush tool's pocket. With `s·n < 0` it notches A under a
   resting tool and leaves a skin over a flush pocket. An Intersect with `s·n > 0` is empty for a resting B and B less a sliver for a flush B. With
   `s·n < 0` it leaves a sliver solid for a resting B. Union with no Decisive contact therefore pulls inward, and Cut
   and Intersect push outward: every contact is wanted for them when none is Decisive, and a tie beside a Decisive
   contact opens outward.
2. Each direction is the contacts' unit direction, each contact pulling in proportion to its Slack, plus a generic part
   of weight 0.2 that is parallel to no coordinate plane, so a shift also opens the walls that two bodies sharing a
   footprint leave coplanar.
3. Steps of 1/4096, 1/512, 1/64, 1/16, 1/4 and 1 of the reach are tried in order, three directions each. A step's length is the
   larger of that fraction of the reach and the least shift that clears every contact's Slack (the Slack over the
   component along its normal, with a margin of 5%). A length above the reach is skipped. Each component of the
   shift `s` is then truncated toward zero onto the grid `2^(⌊log₂|s|⌋−24)` (`snapShift`): the shift never
   lengthens, and its length changes by under `2^-23` of itself, inside the 5% margin. Every component of `s` is a
   multiple of the grid, so for a tag coordinate `o ≠ 0` the sum `o + s` is a multiple of `q`, the smaller of the
   grid and the lowest set bit of `o`, and it is a float, so `o` moves exactly, whenever `|o + s| ≤ 2^53·q`. Two
   cases follow. A coordinate whose last-place unit is no larger than the grid moves exactly unless `|o + s|`
   exceeds the power of two just above `|o|`. A coordinate that is a multiple of the grid moves exactly while
   `|o + s| ≤ 2^53·grid`, which is at least `2^28·|s|`. A tag whose coordinates all move exactly carries no rounding
   (the tag paragraph below). The first outcome that
   is not a contact refusal is the result, whether it is a body or another error, such as an empty Intersect. When a
   shifted attempt meets a contact the unmoved pair did not reach, the contacts are merged and the search starts
   again, up to four times. When every attempt refuses, the original refusal is returned.
5. When every translation has refused, and the exact predicates alone refused the pair with coplanar facet overlaps
   (`meshbool.TouchCoplanar`, every contact with zero Slack), each piece is grown about the mean of its vertices
   (`growShifted`, `pieceShift.Grow`): its farthest vertex moves by the step times the reach, outward for a Union and a
   Cut, inward for an Intersect, and every face moves by its distance from the centre. A tool flush with both walls of a
   slot, whose two faces need to move opposite ways, then pokes through both; a Union overlaps what only touched, and an
   Intersect leaves a gap where it only touched. The growth is charged as any displacement is, its length being the
   farthest vertex's. An edge graze or a point touch is not grown across, because the sign of the growth would decide a
   tangency that stays refused. A tool that only touches a Cut target, grown, notches the target by the growth, which the
   bound covers.
6. A Union whose contacts are all flush faces (none Decisive, none with Slack, the exact predicates' coplanar refusal)
   is tried first by shrinking the second operand about its centre (`growShifted` with `shrink`), steps from the least
   up: the second operand then lies strictly inside the first, so the union is the first operand's own faces and
   leaves no sliver for the next Boolean of a chain to meet, where a translation protrudes by the shift at one end. A
   held curved operand needs the shrink to pass its own chord bound, so the steps climb to it. When no shrink stands,
   the translations of items 1 to 4 run as before. Where this is the first step of a chain (`PatternJoin` of round pins
   into a mesh plate), the result keeps the plate's six faces and the volume of the plate.
4. Each connected piece of the second operand (the teeth of a gear tool, for one) is moved on its own, along the
   directions and by the length its own contacts ask for, and a piece no contact names stays where it is. Each
   piece's displacement is charged to its own vertices and faces, the mesh bound takes the largest, and the swept
   volume sums the pieces, so a piece is never charged for another's move.

The moved operand is the solid the Boolean decides about, so the proximity gate and the exact predicates read it
(`decisionMesh`). Its true solid is the exact move of the true operand, each piece translated by `s` and, when grown,
scaled about its float centre by its float factor `k`. A held vertex is the rounded image of that move, so the
decision copy keeps `m`'s bounds and adds, per vertex, an upper bound on the distance of the stored image `fl(v + s)`
(or the grown image) from the exact `v + s`, computed from the exact rounding error of each coordinate
(`moveRounding`). A face bound takes the largest addition among its corners, and the mesh
bound the largest overall. A grown piece's bounds also stretch by `1 + |k|`. A held vertex need not be a multiple of
the grid, so only the first case of item 3 makes its sum exact: the addition is zero for a coordinate whose last-place
unit is no larger than the grid (below about `2^28·|s|`) unless the sum passes the power of two above it, and a larger
coordinate generally rounds. The chain-depth gate (`RefuseCoarseHeldContact`) admits a mesh bound coarser than the chord
tolerance by the shift the retry applied (the charged copy's mesh bound less the unmoved operand's), because the result's bounds already carry it. The composition of the result's bounds reads a copy that is wider by the shift
(`shiftOperandMesh`): the vertex and face bounds of a moved piece grow by its displacement, the mesh bound by the
largest, the symmetric-difference volume grows by what each move sweeps over its piece's boundary, and the area allowance grows by the rounding of the new
coordinates. The ordinary trim amplification then charges the shift at every crossing. A shifted mesh that folds,
and any pair that uses a spiral-join certificate, is declined.

The moved operand's retained tags (`agent-docs/faceted-source-retention-design.md` §3.1) follow the move
(`movedGroup`, `internal/booleaneval/moved_tag.go`). A face of a piece translated by `s` lies on its tag translated
by `s`, never on the unmoved tag, which misses the true face by `|s·n|` for a plane, `|s − (s·â)â|` for a cylinder
and `|s|` for a sphere. The group carries the translated tag `T+s` (`surfaceplace.TransformSurface`), charged as a
placement by the translation (`facetedrecord.PlacedTagDeparture`, retention §3.4). Every position parameter of `T+s`
is compared with the exact rational sum `o + s`: where the float equals it, the charge is zero and the tag reads as
exact (retained-edge-blend §2), and otherwise the charge is the measured rounding of the moved origin, which the
placement path already charges soundly, and the tag states that departure. No residual against the held mesh admits
the tag. The plane's `CapRecord` is dropped, because it records the unmoved plane. The group keeps no tag for a face
of a grown piece, a face whose facets lie in several pieces, a tag that is not retained, a denoted patch and a `Cone`.
A face of a piece no shift names keeps its tag unchanged.

What a result of the retry states, and what it does not:

- Its true solid is `A∘(B+s)`, the Boolean with the second operand moved by the exact shift. Its retained tags,
  its edge curves and every operation built on it describe that solid.
- Its volume reads `Approximate`, and the bound covers the distance to the volume of the unmoved Boolean. The
  charge widens each moved piece's bounds by its whole displacement, so the bound covers `A∘(B+s)` as well.
- Its vertices lie within their bounds of the features of the unmoved Boolean that the move leaves in place.
- A later blend or Boolean states its bounds against `A∘(B+s)`. They need not cover the distance to the unmoved
  `A∘B`, from which the moved faces lie up to `|s|`, at most the reach.
- The Boolean it computes is the tolerant kernel's reading of a coincident contact, not the exact reading. A feature
  whose existence depends on a gap below the tolerance can be present or absent. A tool that stops one ulp short of a
  wall opens a through hole, because a flush Cut pokes through, and the wall's face area reads the pierced wall. A
  caller that needs the literal reading uses `ExactOnly` or a smaller tolerance.

### 5.3 Stitch

`Stitch` reads the tolerance in `stitchrecord.BuildTolerantPlan`. The exact plan (`agent-docs/surface-design.md`
Table J) is built first. With a positive reach, `mergeNear` then clusters the vertices of planar, straight-edged faces:

- A vertex takes part when its held bound is zero and every edge it meets is a `Line3` on a `Plane` face with no
  normal bound. A vertex on a curved edge or face, or with a bound, stays alone.
- Vertices are taken in walk order, and each joins the first earlier representative within reach, so a class holds
  no chain: the farthest member is within reach of the first.
- The class holds the first member's position. Its bound is the farthest member's distance, taken as a proven upper
  bound, and it is replayed by every placement.
- The merge is made only if no edge collapses and no face loop repeats a vertex the exact table did not repeat.
  Otherwise the stitch runs on the exact table.

The bounds that follow from a bounded class are the ones the shared-denotation weld already charges
(`agent-docs/surface-design.md` Table J amendment J4′): every vertex bound, the volume and centroid through
`massDelta`, and the box. A moved edge's length bound grows by the distances at its two ends. A moved face's area
bound grows by `d·P + n·d²` for its perimeter `P` and `n` vertices. The volume is never `Exact`. A face that no merge
touched keeps its own bounds. A stitch with no vertex within reach of another, and an `ExactOnly` stitch, are the
exact stitch.

## 6. What stays refused

The tolerance repairs one refusal class: a held mesh whose float rounding folds it through itself. Everything else
keeps its refusal:

- A mesh the second stage leaves folded, because a vertex is welded, no candidate within the reach clears it, or
  the reach is too small.
- `Stitch` of a curved edge, of a vertex that carries a bound, and of any pair under `ExactOnly`: the edges stay free (§5.3).
- Branching intersection curves, and a facet or subdivision region that touches the other operand's boundary. They
  carry no contact normal.
- A curved near contact no shift within the reach clears. The gate asks whether chorded surfaces touch, and it is
  decided only when the shift exceeds the contact's Slack, which is the chord error of the facets that meet there.
- A coplanar overlap, graze or point touch whose contacts no shift opens in the sense the operation needs (§5.2), and
  one on a pair that uses a spiral-join certificate.
- An uncertified `sketch` fragment (§3).

Each is a separate decision. Extending the tolerance to one of them changes this document and the document that
owns that refusal in the same change.

## 7. Tests

| Test | What it shows |
|---|---|
| `meshbool.TestEnforceHeldEmbeddingReachClearsWhatTheFloatBoxCannot` | The fixture the float box refuses is cleared at the nearest step, within the reach, with no fold left. |
| `meshbool.TestEnforceHeldEmbeddingReachLeavesACleanMeshAlone` | A mesh the float box clears moves identically with a reach set. |
| `meshbool.TestEnforceHeldEmbeddingReachCannotMoveAWeldedVertex` | A welded vertex stays and the fold is still refused. |
| `facetproof.TestKeepPlacedEmbeddedReach` | A 118-facet crop of a restored spiral cap refuses with no reach, clears at a reach of 1e-3 mm, and charges every moved vertex at least the distance it moved. |
| `apitest.TestDocumentGapToleranceOptions` | The option set, its last-wins rule and each invalid value's error. |
| `booleaneval.TestShiftNeed` | The least shift that clears a contact's Slack, along a normal and along a slanted direction. |
| `booleaneval.TestShiftPiecesChargeEachPieceOnItsOwn` | Two separate pieces: only the moved piece moves and is charged. |
| `booleaneval.TestShiftDecisionChargesRounding` | A tetrahedron at `2^30` mm moved by a snapped shift whose sums round: each decision vertex bound, its face bound and the mesh bound cover the exact distance of the stored vertex from `v + s`; a tetrahedron at 50 mm moves exactly and keeps zero bounds; a grown piece's bounds stretch by `1 + k`. Red legs: drop the addition, the face widening, the mesh-bound widening or the stretch. |
| `booleaneval.TestMoveRounding` | The rounding bound of a translated and of a grown vertex covers the exact distance from its exact image, an exact sum adds nothing, and a non-finite image answers `+Inf`. Red leg: a grown vertex answers 0. |
| `booleaneval.TestSnapShift` | A snapped shift never lengthens, keeps each component's sign, changes by under `2^-23` of its length, and moves whole and dyadic coordinates such as 50 exactly. |
| `booleaneval.TestMovedGroup` | A translated piece's Cylinder and Plane tags move by the exact shift with a zero bound and the plane's `CapRecord` dropped; an unmoved piece keeps its tag; a grown piece, a face spanning pieces, a face with no facet, a denoted patch, an unretained tag and a Cone keep none. |
| `booleaneval.TestFacePieces` | A face whose facets lie in two pieces is marked as spanning them. |
| `facetedrecord.TestPlacedTagDeparture` | An exact translation of a Plane, Cylinder, Sphere or Torus tag states zero departure, and an inexact one of a Cylinder at x = 0.1 states a positive one. |
| `meshbool.TestRefuseUndecidableProximityLocalSlack` | A pair the faces' bounds cannot decide is decided by its close facets' own bounds, and a refusal names its contacts with their Slack. |
| `booleaneval.TestShiftDirections` | Which sense each operation wants for each kind of contact, a Union's inward pull on contacts that are not Decisive when none is, the outward tie beside a Decisive contact, and when no direction exists. |
| `apitest.TestBooleanGapTolerance` | Union, Cut and Intersect of boxes that touch or sit flush: the result, the bound that covers the true volume, a tighter bound from a smaller tolerance, and the refusals `ExactOnly` and a slot keep. |
| `apitest.TestBooleanGapToleranceEdgeAndPointTouches` | A shared corner, a knife edge inside a wider wedge and two crossing edges: Union, Cut and Intersect, the volume bound, and the `ExactOnly` refusal. |
| `apitest.TestBooleanGapToleranceRestingBossJoinsPlate` | A `Transform`ed boss resting on a plate unions into one lump whose volume bound covers `60000 + 288π`, with the boss's top below z = 18, and `Verify` reads `Sound`. |
| `apitest.TestBooleanGapShiftKeepsTranslatedTags` | A `Transform`ed boss on a plate, and a second sketched boss on a plate and boss, join through the retry: the moved wall reads as a radius-6 `Cylinder` about the boss's axis moved by `s`, the top as a `Plane` at `18 + s.z`, and the root and rim as `Circle3` edges. |
| `apitest.TestBooleanGapShiftFilletsRestingBoss` | A radius-1 `Fillet` of every root and rim of the same bodies builds one `Torus` per edge, its volume bound covers the hand volume `60000 + n·(288π + 4π·A·d)` and the moved solid's, smaller by `36π·|s.z|`, and `Verify` reads `Sound`. |
| `apitest.TestBooleanGapToleranceStaysExactForCurvedContact` | A tangent cylinder on a plate stays refused under `ExactOnly`. |
| `apitest.TestVerifyGapToleranceDecidesAFlushPair` | `Verify` reads a flush pair's overlap with a covering bound, leaves it staged under `ExactOnly`, and reports an invalid tolerance. |
| `examples.Example_gap_tolerance` | The documented use. |
