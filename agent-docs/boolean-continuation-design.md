# Boolean continuation design

This document defines a read-only continuation of a refused Boolean through
certified source expressions. It adds no public operation or option. The module
boundary is owned by [Module boundary gate](module-boundary.md); public history
and computational evidence are owned by
[operation-history-design.md](operation-history-design.md).

This design is unimplemented. Existing part-named continuations do not establish
its admission or its supported input class.

## Accepted source expressions

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

## Producer relation certificate

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

## Set rewrites

MUST apply only the following equivalences over certified accepted expressions.
The symbols denote true solids; every constructed intermediate still passes its
ordinary boundary, contact, orientation and occupied-volume checks.

### Cut across Union

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

### Union after Cut

For an accepted `Q = X − G(Y)` and strict whole-solid separation of `G(Y)` and `Z`:

```text
Q ∪ Z = (X ∪ Z) − G(Y)
```

This is the reusable identity corresponding to the existing brake history.
The final Cut must use the exact accepted `G(Y)`, not an independently shifted
copy of nominal `Y`. The separation proof must read that accepted image.

### Images of expressions

For a certified bijective rigid placement or positive uniform similarity `T`:

```text
T(X op Y) = T(X) op T(Y)
```

MUST compose certified maps and their charges through the expression. This
permits a placed compound operand to be rebuilt from its immutable sources.
The planner must not require a particular child count, slot-and-pin recipe,
construction name or original model coordinates.

## Admission and refusal

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

## Bounds and lineage

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

## Declined replacement behavior

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

## Real end-to-end cases

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

## Staged implementation

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
