# Point containment of Boolean BReps

## Status and scope

Proposal; unimplemented.

Extend `Body.LocatePoint` to selected A7 crossing blind cuts, A8 opposed-pocket cuts and class-B Boolean results
by combining their original operands' exact point answers. Do not classify their trimmed face records directly.
The result may still be `PointUndecided` where an operand or the existing set-algebra table cannot decide it.

`point-containment-design.md` owns the public query contract, exact leaf predicates, set-algebra tables and mesh
fallback. `general-boolean-design.md` owns the A7, A8 and class-B constructions. Those owners win over this proposal
until implementation updates them. Public types and signatures do not change.

## Observed behavior

The following observations apply to base `5ed2045c6`.

| Site | Current behavior |
|---|---|
| `point_containment.go`, `locateExact` | Only a faceted payload enters `LocateBoolean`; an analytic BRep enters `LocateRecord`. |
| `internal/pointquery/record_location.go`, `LocateRecord` | A BRep is offered only to `LocateStackBrep`. |
| `internal/pointquery/stacked_location.go`, `LocateStackBrep` | A nil `Stack`, a displaced face or a modified band declines. |
| `internal/pointquery/history_location.go`, `historyWalk.locate` | An undecided analytic leaf cannot expand because `boolchain.Expandable` accepts faceted payloads only. |
| `internal/breprecord/payload.go`, `Payload` | Only an A1 result carries `Stack`; A7, A8 and class-B records have none. |
| `internal/boolchain/chain.go`, `Expandable` | The same faceted eligibility serves point queries, Boolean regrouping and Section. |

The gap affects a BRep queried directly and the same BRep encountered as an operand of a later mesh Boolean.
Public Boolean history already records the original target and tool through `historyProducers.BooleanProducer`
in `boolean_regroup.go`. The missing fact is permission to use that operation as this BRep's set identity.

A local public-API probe built real sketches, extrusions and cuts. At a chord tolerance of 0.05 mm:

| Real producer | Query | Observed answer |
|---|---|---|
| A7: 20 × 20 × 10 plate, radius-3 through hole, pocket `[-2,6] × [-2,2] × [6,10]` | `(0,3,5)` and radial offsets ±1e-9 mm | All three were `PointUndecided`. |
| A8: same plate, top pocket `[-6,0] × [-3,3] × [4,10]`, radius-3 bottom pocket centered at `(1,0)` over `[0,4]` | `(4,0,2)` | `PointUndecided`. |
| Class B: same plate, radius-3 cylinder along x, centered at `(y,z)=(0,5)`, spanning `[-20,20]` | `(0,3,5)` | `PointUndecided`. |

Each result's payload class was checked as `decad.brepPayload`. The probe did not substitute handwritten BRep
records for the real producers. Its source and output are local evidence under `.tmp/brep-point-design/`.

## Proposed construction evidence

Add a constant-size, immutable Boolean set-identity record on `topo.Body`, beside its existing computational
evidence. Define its plain-value representation in `internal/prooflineage/boolean_set.go`.

Record the operation kind and full provenance references of output, target and tool, in original caller order.
Use full `provenance.Ref` values, including roles, rather than producer numbers alone. The record contains no
topology pointers, context, cache or public operation object. The existing document history supplies the retained
operand bodies when the record's bindings match.

The record asserts exactly this: the output's denoted material set equals the regularized operation over the
two named input sets in world coordinates. It does not assert that the held face trims are exact, that all leaf
queries can decide, or that the result admits Boolean regrouping or an exact planar Section.

Attach the record only in `internal/booleandispatch/dispatch.go`, after a selected route has returned a successful
body and before the outer operation commits:

- `TryCrossingBlindCut` followed by `AnalyticBody` supplies A7 Cut evidence.
- `TryOpposedBlindCut` followed by `AnalyticBody` supplies A8 Cut evidence.
- A successful `classB` followed by `AnalyticBody` supplies class-B evidence when the result is a BRep.

These are construction-specific certificates. Do not attach one in the generic `breprecord.Build`, the shared
`found` helper, or every successful analytic Boolean. A private pattern step without a matching committed public
producer cannot expand through this route, even if it carries evaluator evidence.

First enable only unmodified outputs with zero face/level displacement, no loop or shell bands, and exact
source-to-result coordinate maps. This gate excludes rounded crossing trims even when the dispatch succeeded.
It permits the whole-line A7/A8 first slice and exactly restated class-B through/rooted faces after the
construction checks below. This first slice helps BReps used as history operands even when their own exact
mesh already decides the same point. It does not claim to solve the curved probe cases immediately.

Before enabling each attachment, prove the route's original-input identity from its owning construction:

1. A7 clips the tool's interval against the target. Removing the clipped portion equals removing the original
   tool because all discarded tool points lie outside the target. The section scene selects the original
   regions' difference; its certified cuts and displacement charges enclose the crossing, rather than moving it.
2. A8 partitions opposed pockets at their common level. The assembled floors, ceilings and walls bound the same
   plate minus the original pocket tools; the scene changes the representation of that difference.
3. Class B selects original operand faces or their certified trims. Restating a wall as a plane and reading an
   admitted Revolve as its equal prism or coaxial stack must preserve the original input set exactly.

The prerequisite for each proof is its existing admission contract, including Sketch certification and exact
re-expression. A small residual, zero reported bound or successful mesh audit cannot supply a missing set identity.
If a path snaps or moves accepted geometry, omit its certificate and keep the fallback. Future widening of any
producer must explicitly preserve this certificate's assertion or stop attaching it on the new branch.

| Construction | Proof status and attachment rule |
|---|---|
| Whole-line A7/A8, undisplaced levels and exact maps | The recorded exact cells select the original set difference. Confirm the real producer emits exactly those cells and no canonical vertex changes their geometry before attaching. |
| Curved A7/A8 with rounded junctions | The query gap is confirmed. Original-source identity is unproved; keep these paths uncertified until the source-defined record and consumer enclosures are established below. |
| Class-B through/rooted face replacement | Admit only the exact re-expression, unchanged-face and exact interval subpaths. Verify selection of each kept original face and clipping of the tool interval; an operation name alone supplies no evidence. |
| Class-B crossing reach | Canonicalized face trims carry displacement. Keep uncertified until its original-carrier definition and crossing charges establish the same set and all consumers' enclosures. |
| Revolve prism or stepped view | `CylinderSteps` is an existing equal-set reduction, but certificate attachment also needs the selected downstream class-B proof. A successful view does not certify a downstream crossing trim. |

For the displaced cases, the missing design obligation is a source-defined BRep set record: the original
carriers, certified selected regions and exact crossing limits define the material, while canonical coordinates
are held topology approximations with their existing charges. Audit `stackedbrep.Engine`, `StackedRecord`,
`classbgeom.CanonicalizeFace`, BRep measurements and tessellation against that definition. If their proofs cover
only the emitted approximating trims, add the source-defined record and revise those enclosures before enabling
history classification. A bound relating two sets does not prove the sets equal. Do not attach on those paths
while this obligation is unresolved.

Output face displacement alone does not refute the identity: a rounded intersection vertex may approximate a
true trim on the original surfaces. The certificate must distinguish that representation error from an actual
change to the accepted material set. Do not erase or reduce any existing displacement charge.

## Query admission

Keep `boolchain.Expandable` unchanged. Its existing regrouping and Section consumers require their own proofs.
Add point-query-specific eligibility beside `historyWalk`; use the existing faceted branch first, then the new
BRep certificate branch.

Admit a BRep certificate only when all of the following hold:

- The node is a solid BRep in the query's document.
- The certificate's output reference equals the node's full origin reference.
- The document names a committed public Cut, Union or Intersect with this body as output.
- Its consumed input references, operation kind and original input order exactly match the certificate.
- Every operation warning passes the existing fail-closed no-geometry-change classification.
- The record is present for the selected construction, not inferred from retained surface tags or face roles.

Missing records, unknown kinds, mismatched inputs, unknown warnings and any claimed geometry change decline.
Do not accept arbitrary BReps merely because their public operation says Cut or their faces are planes and cylinders.

The BRep's certificate is attached to the built node, not its payload. `Copy`, `Transform`, `ScaledAt`, Fillet,
Chamfer and Shell build fresh nodes and do not copy it. Their output operation kinds also fail the public-producer
check. A new Boolean may certify its own operation over such a body, but that operand still needs an exact leaf
answer or its own separately justified query route.

## Query algorithm and proof

Run the existing analytic leaf reader first. If it declines, try point-query Boolean eligibility on both a BRep
and a faceted result. This ordering preserves the exact stack reader where it can answer more than set algebra.
Change `locateExact` to enter this common walk for its admitted payload kinds instead of selecting history only
for faceted payloads.

For an eligible BRep, read the original operands and reuse `historyWalk.union`, `intersect`, `cut`, `join`, `meet`
and `complement` without changing their tables. Each exact leaf answer is about its original denoted set. The
certificate equates the BRep's set with their operation, so the existing table proof transfers every decided
answer to that BRep. This proof is independent of rounded trim vertices and of the tessellation chord tolerance.

Keep the world query point unchanged throughout this walk. Existing prism, stack and Revolve readers solve
their own recorded frames and placements. Never project the point onto an output face or round an inverse query
to binary64 to make a leaf applicable. Placed-result and geometry-shifted-result extensions need separate evidence;
the latter is scoped by `gap-shifted-query-design.md`.

Reuse existing exact leaf predicates for profiles, holes, arcs and multiple regions. Do not arrange output face
loops, recompute 2D intersections, infer crossing vertices or add a second winding implementation. A leaf's
uncertified fragment or displaced level stays undecided under the current leaf contract.

On a surviving face, a point inside the target and on the tool boundary gives `PointOnBoundary` for Cut. A point
on both operands' boundaries still gives `PointUndecided`; a bore rim or crossing seam needs local neighborhood
information the current table does not carry. Do not force those points onto the boundary from a face tag.

For A8, the first pocket result can be a stacked leaf and the second tool a prism leaf. On their shared level,
both-boundary cases may remain undecided. This increment does not require a new slab representation for A7 or A8.

If the common walk cannot decide the root query, call the existing verified-mesh fallback exactly once for the
requested result. Do not tessellate retired operands. The fallback and exact walk must refer to the same denoted
set; certification is omitted whenever that equality has not been established.

## Work, cancellation and concurrency

Keep one `historyWalk` budget for body visits and preserve each leaf's existing segment and region polling.
Reuse the finite consumed-input history walk; do not rebuild geometry or recursively retain new topology graphs.
The certificate adds constant storage per selected result and constant validation per expanded body.

Poll cancellation before eligibility and before returning a decided answer. Propagate context errors rather than
turning them into `PointUndecided`. The new record is finalized before publication and never mutated by a query;
per-query traversal data stays local. No background goroutine or shared query cache is needed.

## Implementation slices

| Slice | Files and result |
|---|---|
| A7 first | Add the identity record, selected attachment and common query entry described below. Prove one whole-line fixture end to end before widening. |
| A8 | Attach only the proven A8 route; cover both pocket orders and the exact interface level. |
| Class B | Attach the proven BRep-producing class-B paths; cover through and rooted reach, crossing trims and Revolve prism/stack views separately. |
| Contract and integration | Update query/history owners, the missing-features row and public API description. Register files and new test names under layout and shard rules. |

The first slice changes `internal/prooflineage/boolean_set.go`, `internal/topo/body.go`,
`internal/booleandispatch/dispatch.go`, `internal/pointquery/history_location.go` and `point_containment.go`.
Contract owners are `point-containment-design.md` and `operation-history-design.md`; update `docs/api.md` when
the new behavior ships. New exact-construction fixtures belong in `apitest/point_containment_brep_test.go`.

Do not change `internal/sectionhistory`, regrouping, leaf admission or geometry producers' refusal boundaries.
If a selected class-B subpath cannot prove original-input identity, leave it uncertified and record that specific
remaining gap instead of expanding this work into a general BRep point classifier.

## Required real fixtures

First run a whole-line version of A7 using the rectangular-hole fixture in `crossing_blind_pocket_test.go`.
Require zero face/level displacement and exact original-source identity before attaching its certificate.
Use that result as an operand of a later mesh Boolean so the fixture exercises the new history path.

Once the curved producer's source-defined identity and all enclosures are proved, use the exact A7 probe geometry
under Observed behavior. Require its BRep class, the three point answers
`OnBoundary`, `Inside`, `Outside` at radius 3 and the outward/inward offsets respectively, and an unchanged volume
enclosure. Here material is outside the through hole, so the outward radial offset is inside the solid.
Check the pocket floor at `(4,0,6)` and ±1e-9 mm along z: boundary, inside below and outside above.

Then use the A8 probe and require `(4,0,2)` on the bore wall with either pocket order. Check ±1e-9 mm in x,
and a surviving floor point `(-4,0,4)` and its z offsets. Include a shared pocket rim expected to stay undecided
when both operand answers are boundary. Check the same answers after retiring the queried result in another call.
`Body.LocatePoint` explicitly permits retired bodies; retain this public guarantee while also testing retired
operands through a live descendant.

Use the class-B probe and require `(0,3,5)` on the cross bore, with y offsets ±1e-9 mm deciding material/void.
Add one existing stepped-shaft Cut fixture from `revolve_stepped_shaft_class_b_test.go` as a certificate-binding
case, using the real Revolve producer and its original operand leaf instead of a hand-built equivalent prism.

Test an eligible BRep used as an operand in a later warning-free mesh Boolean, at a point away from the new tool.
The answer must require expanding the BRep; an unrelated operand that alone decides the table is insufficient.

## Negative and focused verification

- Unit-test missing, stale and mismatched certificates, reversed Cut operands, unknown warning codes and changed
  geometry. Verify `boolchain.Expandable` still declines certified BReps.
- Verify Copy, Transform and modify descendants carry no usable certificate; preserve their current fallback.
- Preserve leaf refusals for displaced levels, unsupported arc fragments and undecidable loop containment.
- Exercise pre-canceled and mid-walk contexts; inspect only scalar results and bounded error data.
- Run focused `internal/pointquery` and `internal/prooflineage` tests, the new public cases and existing producer
  regressions. Run race mode for concurrent queries. Let remote CI run the full suite after implementation.

Design-time checks passed:

```text
go test -v -count=1 ./.tmp/brep-point-design
go test ./internal/pointquery -run '^(TestSetAlgebraTables|TestLocateBooleanCombines|TestLocateBooleanWalksHistory|TestLocateBooleanDeclines|TestLocateBooleanStackedAndRevolveLeaves|TestLocateStackBrep|TestLocateStackBrepLeavesOpenRecords)$' -count=1
```

These checks confirm today's fallback and the existing algebra machinery. They do not test the proposed certificate.

## Limits and open work

The baseline gap and smallest A7/A8/class-B public fixtures are confirmed. No dependency change is needed for
the initial whole-line/circle operands. Establish the construction-specific set-identity proof in each producer
before attaching its certificate; existing successful topology tests alone do not prove it.

Exact answers at both-boundary seams, rational-level leaf admission, arbitrary modified BReps and placed Boolean
result queries are outside this increment. They need additional local evidence or separate query designs.

## Implementation task plan

Follow the [ordered implementation tasks](plans/brep-point-containment-implementation.md).
This design owns the proposed contract; the plan supplies execution steps.
