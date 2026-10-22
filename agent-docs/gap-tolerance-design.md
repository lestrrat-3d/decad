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

The root reads the tolerance in `placePayload`, `booleanBodyWith`, `performUnionMany` and `ScaledAt` and puts it on the
context. Every other operation, such as `Fillet`, `Shell` and `Loft`, reads no tolerance of its own. Four evaluator
sites read it:

| Site | Operations that reach it |
|---|---|
| `facetproof.KeepPlacedEmbedded`, a rigid placement of a faceted mesh | `Transform`, `Copy`, `PatternCopies`, `Patterned`, `MirroredCopy` |
| `facetproof` similarity image | `ScaledAt` |
| `meshbool.keepRoundedEmbedded`, the stitched result of a mesh boolean | `Union`, `Cut`, `Intersect`, and the booleans inside `Patterned` and the tool combinations |
| `booleaneval.Evaluate`'s shifted retry | `Union`, `Cut`, `Intersect`, and the same inner booleans |

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

A mesh Boolean refuses three contacts the exact predicates cannot classify. Two operand facets can overlap in one
plane, so no side of the other operand is proven for the shared region. An edge of one operand can graze along a facet
of the other. The operands can touch at an isolated point. `meshbool.TouchContactError` carries every such contact it
met (up to 64) as the first operand's outward unit normal at the contact and whether it is Decisive. A coplanar pair is
Decisive when the second operand's facet faces the opposite way, which puts the operands on opposite sides of the plane
(a face-on-face touch). A graze and a point touch are always Decisive. `booleaneval.Evaluate` reads the error and
runs the Boolean again with the second operand moved:

1. Directions come from the contacts (`shiftDirections`). Every contact must open: the shift's component along each
   contact's normal is at least 0.02 in magnitude. A Decisive contact also fixes the sense. A Union moves the second
   operand inward along the first operand's normal, so bodies that touch join into one, along a face, an edge or at a
   corner. A Cut and an Intersect move it outward, so a touching Cut leaves its target whole and a touching Intersect
   is empty. With no Decisive contact, the contacts are flush walls. A Cut and an Intersect still need every flush contact to open
   outward, so a tool flush with its target's wall pokes through it and leaves no skin. A Union needs only that they
   open. A pair of contacts that no direction opens in the needed sense, such as a tool flush with both walls of a
   slot, is not retried and the refusal stands.
2. Each direction is the contacts' unit direction plus a generic part of weight 0.2 that is parallel to no coordinate
   plane, so a shift also opens the walls that two bodies sharing a footprint leave coplanar.
3. Steps of 1/16, 1/4 and 1 of the reach are tried in order, three directions each. The first outcome that is not a
   coplanar refusal is the result, whether it is a body or another error, such as an empty Intersect. When every
   attempt refuses, the original refusal is returned.

The moved operand is the solid the Boolean decides about, so the proximity gate and the exact predicates read it
with its own bounds. The composition of the result's bounds reads a copy that is wider by the shift
(`shiftOperandMesh`): every vertex, face and mesh bound grows by the displacement, the symmetric-difference volume
grows by what the move sweeps over the operand's boundary, and the area allowance grows by the rounding of the new
coordinates. The ordinary trim amplification then charges the shift at every crossing. The moved operand keeps no
retained surface tag, because its facets no longer lie on the surfaces they were cut from. A shifted mesh that folds,
and any pair that uses a spiral-join certificate, is declined.

What a result of the retry states, and what it does not:

- Its volume reads `Approximate`, and the bound covers the distance to the volume of the unmoved Boolean.
- Its vertices lie within their bounds of the features of the unmoved Boolean that the move leaves in place.
- The Boolean it computes is the tolerant kernel's reading of a coincident contact, not the exact reading. A feature
  whose existence depends on a gap below the tolerance can be present or absent. A tool that stops one ulp short of a
  wall opens a through hole, because a flush Cut pokes through, and the wall's face area reads the pierced wall. A
  caller that needs the literal reading uses `ExactOnly` or a smaller tolerance.

## 6. What stays refused

The tolerance repairs one refusal class: a held mesh whose float rounding folds it through itself. Everything else
keeps its refusal:

- A mesh the second stage leaves folded, because a vertex is welded, no candidate within the reach clears it, or
  the reach is too small.
- `Stitch` joins free edges only where coincidence is proven (`agent-docs/surface-design.md` §1.3).
- Branching intersection curves, a facet or subdivision region that touches the other operand's boundary, and a curved
  tangency or near contact the proximity gate cannot decide. The first two carry no contact normal. The gate asks
  whether chorded surfaces touch, and a shift far smaller than the chord error cannot answer that.
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
| `booleaneval.TestShiftDirections` | Which sense each operation wants for each kind of contact, and when no direction exists. |
| `apitest.TestBooleanGapTolerance` | Union, Cut and Intersect of boxes that touch or sit flush: the result, the bound that covers the true volume, a tighter bound from a smaller tolerance, and the refusals `ExactOnly` and a slot keep. |
| `apitest.TestBooleanGapToleranceEdgeAndPointTouches` | A shared corner, a knife edge inside a wider wedge and two crossing edges: Union, Cut and Intersect, the volume bound, and the `ExactOnly` refusal. |
| `apitest.TestBooleanGapToleranceStaysExactForCurvedContact` | A tangent cylinder on a plate stays refused. |
| `examples.Example_gap_tolerance` | The documented use. |
