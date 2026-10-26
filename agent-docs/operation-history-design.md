# Operation history design

Navigation only; the named sections own their rules.

| Question | Section |
|---|---|
| What calculation improves? | Computational consumers; First calculation fixture |
| What survives each operation? | Computational evidence; Admission and bounds |
| What can callers inspect? | Public API proposal; Face correspondence and completeness |
| What is collected and published atomically? | Atomic publication and storage |
| What should an implementation agent do, in order? | Ordered implementation tasks |
| May internal code read public records? | Boolean chain regroup |

## Status and scope

Implementation records public body-producing routes and `Remove`, and includes
contained-Cut Volume, centroid, and mass-property calculations. Remove and Unstitch have
complete face maps; other routes report unknown coverage unless the builder
supplies an attribution map. Composite tools remain outside the live set and
public history until their outer call commits. This work changes no role,
geometry, liveness, or measurement contract.

Retain construction history as evaluator evidence so later calculations can use
analytic source geometry instead of accumulating avoidable mesh error. The new
calculations derive tighter Volume, centroid, and mass-property readings for a
certified contained Cut. The centroid uses source first moments, and the mass
properties use density-independent source volume, first, and second moments.
Every document records public operation history from creation for verification
and inspection of successful calls.
The caller's Go function remains the feature tree. Proof records describe evaluated
sets and constructions; they contain no replayable feature commands, rebuild mechanism,
undo, or file format. Internal code may read public records to re-run Boolean geometry
over the retained bodies they name; Boolean chain regroup below owns the rules.

The owning contracts are `api-design.md` §2, §3, §5.3, §6 and §12;
`evaluator-design.md` §3 and §8; `prism-boolean-design.md` §11;
`general-boolean-design.md` §4.2; and `param-binding-design.md` §3.
Use those documents for existing behavior. This proposal owns the additional
computational lineage, source-quantity consumer, and public history queries.

## Computational consumers

### Existing local Boolean bounds

Preserve the existing calculation in `faceted-source-retention-design.md` §11.
`booleaneval.retainedPlaneOf` supplies certified planes to
`BoolMesh.SetSlabBounds`; the slab certificate tightens the later held-contact gate
and `RimBound`. These are implemented calculations, not proposed history benefits.
Keep the separate full displacement for interior vertices and trim uncertainty.
The owning document supplies the formula and admission rules; do not duplicate them here.

Carry the immediate source patch and its valid plane certificate through real
Boolean assembly and modify rewrites. This lets a later nearby Cut consume the
same evidence after earlier operations. Source identity makes the certificate
traceable; identity alone never admits the plane or changes a bound.

### Contained Cut volume

Add a calculation for a successfully built `Cut(A, B)` when a typed certificate
proves the whole true solid B lies in A's material. Retain that set relation and
the immutable source volume readings. Compute `Volume(A) - Volume(B)` with outward
rounding, including both input bounds and arithmetic rounding. This reads the true
source solids rather than the tessellation's occupied-volume approximation.

Obtain the relation from the strict full-containment proof owned by
`interference-design.md` §4. Reuse its single-material-lump, positive boundary gap,
and both-directions shell-witness requirements, including void shells. The existing
`pairkernel.Result.Contained` is a per-call result; turn its proven statement into
an immutable certificate attached to this result, not a stored boolean verdict.
Do not use a nesting witness or bounding-box containment as that certificate.

Run the relation proof while both input records are available and before commit.
The public Boolean still builds and audits its real result. This calculation does
not make a previously invalid Boolean valid. For an unproved relation, keep the
current mesh volume. Select the source-derived reading only when its absolute
bound is strictly smaller; a tie keeps the current reading for stable output.
Keep both valid proof intervals available for consistency checks.
Disjoint candidate intervals are an internal consistency error before publication.
Limit the attempt to the containment kernel's supported analytic input classes.
Reuse a matching certificate obtained during evaluation when available; otherwise
call `pairkernel.Pair` once before commit under the operation's context. An
unsupported carrier or undecided relation returns no containment certificate and
selects the ordinary reading. `proofbound.WorkBudget` polls the context and has no
separate work-exhaustion result. Cancellation and other `Pair` errors follow the
existing error path and abort the operation before commit.

### Contained Cut centroid

For a successfully built mesh `Cut(A, B)` with the same strict whole-containment
certificate, derive the result centroid from the bounded source volume and
centroid readings. Compute each world-coordinate first moment about the target
centroid anchor `o`:

```text
o + [VA·(CA − o) − VB·(CB − o)] / [VA − VB]
```

Use the already-derived source-volume difference as the denominator. Propagate
both source volume bounds, both source centroid radii, and each arithmetic
rounding term with `proofbound` bounded arithmetic. Require the denominator's
proven lower endpoint to be positive. If its interval crosses zero, keep the
audited mesh centroid. If the interval is wholly nonpositive, the source
readings contradict strict containment; fail before commit. If arithmetic
overflows a finite reading, keep the mesh centroid.

Compare the derived and audited mesh centroid balls using exact held-center
distance. Disjoint certified balls are an internal consistency error before
commit. Select the derived centroid only when its radius is strictly smaller;
a tie or larger radius keeps the mesh result. This selection is independent of
whether the source-derived Volume reading wins its own bound comparison.
`Verify` reads the selected `Body.Centroid` through its existing centroid gate.
Derive `Exactness` from the selected centroid's own bound. Do not derive
`MassProperties` or inertia from this centroid record.

Store both candidate centroid readings and the selection result as plain
`CentroidLineage` values nested in `VolumeLineage`. Validate source producer
identities against the parent record. Do not retain topology, body, document,
context, or cache pointers. Public operation history continues to record only
the public call and its body relationships.

### Contained Cut mass properties

For the currently admitted rectangular-prism target and whole circular-prism
tool, integrate both source records into world-coordinate V/P/Q moments about
the target's held centroid. Freeze copied source moments and their difference
in the result's existing volume lineage before commit. The moment record keeps
producer identities and containment facts, and each query validates them
before reading independent copies.

The faceted `MassProperties` path always obtains its audited mesh tuple first.
It then publishes a source-derived candidate with the caller's density and
checks mass, all six inertia entries, and both center balls against the mesh
and the selected body centroid. Any disjoint evidence fails the query. A
consistent source tuple replaces the mesh tuple only when all eight bounds are
no larger and at least one is smaller. Unsupported integrations or source
publication limits keep the mesh tuple. One direct exact-rotation Copy of an
admitted original Cut transfers frozen V/P/Q evidence through
exact-copy-moments-design.md. Transform, default Copy, and later descendants
do not transfer the record.

### Local modify and later measurements

Preserve unchanged source-patch certificates across Fillet and Chamfer. Replace
affected patches with the modify builder's certified trimmed and blend records.
`internal/retainedblend` already preserves surface groups and creates blend tags;
extend its lineage alongside those real groups, not by joining old and new roles.
Existing retained-edge routes can consume the resulting surfaces, rims and local
bounds under `retained-edge-blend-design.md`. They gain no new admission from an
operation name, ancestry pointer, or complete public face mapping.

The contained-Cut mass-property slice is implemented in
`agent-docs/contained-cut-moments-design.md`. It retains only a rectangular
prism target and whole circular prism tool, consumes their V/P/Q moments in
the existing faceted query, and validates the full tuple against the audited
mesh. A direct exact-rotation Copy uses the retained source under
agent-docs/exact-copy-moments-design.md; it does not support moment chains.
Other operation classes still need their own moment evidence and selection
rules before they can improve `MassProperties`. A Volume or
centroid certificate alone cannot tighten area, wall thickness, or the mesh's
symmetric-difference bound.

Stage analytic reconstruction separately. Reusing a source payload requires a
set-equality certificate or a complete certified analytic trim boundary plus
closure, orientation and occupied-set proof. A carrier surface and held chord loop
do not provide that boundary. For admitted reconstruction, build a new immutable
result through the existing analytic builder and fresh-role policy; never replace
an already returned body's topology. Unsupported reconstruction keeps the proven
faceted result. Do not re-tessellate an unknown trim as if it were an analytic patch.

### Boolean chain regroup

`agent-docs/boolean-chain-regroup-design.md` owns the one designed reader of public
records. When a two-body Boolean refuses and an operand is a mesh Boolean result, it
walks producer records to the retained source bodies and evaluates a set-equal
regrouping of the chain. It is not replay:

- It re-runs Boolean geometry only, over immutable retained bodies.
- It replays no feature call, re-solves no sketch and reads no parameter.
- It publishes no record beyond the caller's own Boolean and consumes no retired body.
- A missing producer record or an ineligible body makes it decline; it never refuses.

## Computational evidence

Keep two distinct always-on records. Internal proof lineage contains immutable
construction data a calculation needs. Public operation history records every
successful modeling call and its caller-visible body relationships. A snapshot
exposes those public records; taking a snapshot does not enable their collection.

| Internal evidence | Required contents |
|---|---|
| Source patch | Immediate source identity, copied analytic record or immutable source-record reference, material orientation, placement, and its certified domain and bounds. |
| Boundary attribution | Operand patch to output patch map, actual arrangement spans or held facet groups, and completeness per region. Equal role sets are insufficient. |
| Relation certificate | Source-set identities and placements plus the typed whole-containment or equality proof and its eligibility checks. |
| Quantity certificate | The bounded source readings or source integrals, units, reference frame, and arithmetic or transformation charges needed to recompute the claimed quantity. |
| Modify certificate | The untouched patch map, the newly constructed trim and blend records, and the local displacement or occupied-volume terms required by its consumer. |

Store this evidence with the immutable result payload. Retain compact source
descriptions and certificates, not arbitrary caller objects or the entire document.
Share immutable source records where practical. A source's retirement does not
change its mathematical geometry; readers can use retained evidence without
reviving a body or changing any feature's live-input gates. Existing live-source
requirements such as `CertifyCopies` remain in force until their owning design changes.
Pending maps can key topology pointers. Committed computational links use private
source-record tokens and immutable descriptions, not topology nodes that retain
their owning body and document. Public records retain those body handles.

`Group.Origins`, `RetainedTag`, `CapRecord`, denoted patches, per-vertex bounds and
typed source records already prove different claims. Reuse their owning contracts;
do not replace them with a universal history-present flag. Add immediate source
identity at `booleaneval.Evaluate`'s existing face-to-group assignment and carry it
through final connected-patch assembly. Identity can key evidence, but cannot
manufacture a missing domain, exact curve, containment relation, or error bound.

## Admission and bounds

Require each computational consumer to validate the exact evidence it reads.
Keep a valid source surface when trim evidence is incomplete, but restrict it to
calculations its carrier and normal certificates prove. Missing optional evidence
selects the existing certified path. A contradictory stored certificate is an
internal error; never conceal it as an ordinary missing optimization.

Preserve the original `MeshBound`, vertex displacements, `AreaSlack` and
`VolSymDiff` unless a separate proof tightens that same quantity. A smaller Volume
bound does not shrink the mesh's geometric error. A local plane slab bound only
tightens the contact calculations named by its owner. Recompute placement and
rounding charges through r3 and the existing bound machinery; drop evidence whose
placement transfer is unproved. A small residual can reject nothing and admit nothing.

Derive Exactness per returned reading from its own proven absolute bound. Exact
source records plus an exactly representable subtraction can give Exact Volume;
a cylinder involving pi normally keeps a small positive bound and Approximate.
Retained surface tags, complete face history, and an exact containment relation do
not by themselves make a numerical reading Exact. Verify must read the selected
cached Volume with its new bound through the existing tolerance gate. A later
calculation can reuse that bound only with a certificate for its own operation.

## Concrete questions

| Caller question | Proposed answer |
|---|---|
| Can a contained curved cutter avoid mesh volume error? | Its certified set relation permits a source-volume difference with arithmetic and input bounds. |
| Can a later nearby Cut use a retained plane? | Its valid plane and local slab certificates tighten contact bounds through the existing Boolean path. |
| Which call made this retired body? | Its successful public operation and its actual input and output body handles. |
| Did a copy consume its source? | Its source input is recorded as a reference; a later consuming call is separate. |
| Why did a body leave `Bodies()`? | The operation consuming it, including an explicit `Remove` record. |
| Which faces of this Cut came from this particular tool face? | A one-operation face correspondence result with explicit completeness. |
| Did an analytic Union erase a source role? | Existing selectors follow existing role rules; history can still report body dependencies. |
| Is an empty face result proof that the source disappeared? | Only an exhaustive correspondence result proves no output boundary contribution. |

History records body dependencies, not parameter dependencies. A preceding
`Document.Param` read is not automatically attached to the next call. The binding
ledger continues to record reads independently, as `param-binding-design.md` §3 specifies.

## Public API proposal

Define caller-visible types in the root package, with unexported storage and
accessors. Put record construction, validation and lookup data in a focused
internal package. Names below are proposed signatures, not implemented symbols.

| Proposed symbol | Behavior |
|---|---|
| `(*Document).OperationHistory() *OperationHistory` | Returns an immutable snapshot. Every document collects history from creation; an empty document returns an empty snapshot. |
| `(*OperationHistory).Operations() []*Operation` | Returns successful public operations in commit order. |
| `(*OperationHistory).Producer(*Body) (*Operation, error)` | Returns the public operation that returned this body. It accepts retired bodies. |
| `(*OperationHistory).Consumers(*Body) ([]*Operation, error)` | Returns records that consumed this body within the snapshot. |
| `(*Operation).Kind() OperationKind` | Reports the public entry point, such as Extrude, Cut, Patterned or Remove. |
| `(*Operation).Inputs() []OperationInput` | Returns ordered body inputs with a role and use classification. |
| `(*Operation).Outputs() []*Body` | Returns bodies delivered by this public call, in its return order. |
| `(*Operation).FaceSuccessors(*Face) (FaceHistory, error)` | Returns certified boundary contributors among this operation's output faces. |
| `FaceHistory.Coverage() HistoryCoverage` | Reports `HistoryUnknown` or `HistoryComplete`. The zero value is unknown. |
| `FaceHistory.Faces() []*Face` | Returns known successors without duplicates, in output-body then face-traversal order. |

The public record does not expose or certify internal reconstruction evidence.
`OperationInput` is an inspectable value containing `Body *Body`, `Role string`,
and `Use InputUse`. `InputUse` distinguishes `InputConsumed` from `InputReferenced`.
Roles name argument purposes: `target`, `tool`, `source`, `operand`, `stop`, and
`throughAll`. Repeated argument purposes preserve caller order. A body read in
several roles has several entries; consuming membership is deduplicated by body.
These input roles are metadata and never become `FeatureRef.Role` values.

`*Operation` identity is scoped to the document and stable across its history
snapshots. Do not expose a numeric operation ID or reuse a `FeatureRef` as one.
An operation with several outputs can span several private producer identities;
an operation with no outputs can span none.

Every returned slice is a copy. Bodies, faces and operation records are immutable.
Changing the live model after a snapshot cannot change that snapshot's records,
consumer results, or face correspondence. It can change current body liveness;
history is never permission to consume a retired body. Reading a retired body's
immutable geometry inside a later call's private evaluation is not consumption: it
changes no liveness and publishes no record.

## Operation kinds

This section owns the shared root-package `OperationKind` vocabulary for history
and `operation-diagnostics-design.md`. Diagnostics can name failed calls and
selector resolution without requiring a committed public history record.

Reserve zero as `OperationUnknown`. Define one documented value per public modeling
or selector-resolution entry point, including `OperationSelectEdges` and
`OperationSelectFaces`. `OperationKind.String() string` returns stable lower-snake
tokens: `unknown`, `extrude`, `cut`, `patterned`, `remove`, `select_edges`, and
`select_faces` illustrate the spelling rule. Existing tokens never change meaning.
Clients must handle unknown and future values with a default branch. A kind reports
no evaluator route, payload kind, mesh index, or internal producer number.
Vocabulary membership does not imply a public record: the next section owns which calls
produce history records. Selector resolution produces none.

## Recorded calls and dependencies

Record one entry per successful public body-producing call, plus successful
`Remove`. Record the public entry point the caller used. A `Hole` is one Hole
record, and its private tool construction and cuts create no public history entries.
The same rule applies to Rib, Web, Emboss, mirror joins and pattern joins.
If the caller explicitly invokes Copy followed by Union, those are two entries.

| Call shape | Inputs | Outputs |
|---|---|---|
| Feature built from a sketch | No body inputs unless its extent or another argument reads a body. | The returned body. |
| Cut or Intersect | Consumed target and tool, in public argument order. | The returned body. |
| Union | Consumed operands, in public argument order. | The final returned body. |
| Transform or a modify operation | Consumed receiver, plus any distinct body dependencies. | The returned body. |
| Copy or MirroredCopy | Referenced source and any body used to define placement. | The new copy. |
| PatternCopies | Referenced source. | The returned new instances; the original is not an output. |
| Patterned or PatternJoin | The public consumed inputs. | The final returned body. |
| Unstitch | Consumed receiver. | All returned sheets in one operation. |
| Remove | Consumed body. | No outputs. |

Record body-relative extent and placement dependencies even when they stay live.
For `ToFace`, record the stop body's handle; a selector or face reference does not
replace the owning body dependency. For `ThroughAll`, record the live-body snapshot
whose bounds the evaluator used, including bodies excluded by its bounds tests.
Do not rescan the document after evaluation to reconstruct those dependencies.

Do not record read-only calls: selectors, tessellation, export, Section, measurements,
Verify, interference evaluation and motion checks create no modeling records.
In particular, the read-only boolean evaluator used by Verify must never emit an
Intersect record. A failed or canceled public call emits no record.

## Face correspondence and completeness

A successor is an output face whose boundary patch the builder derives from some
part of the input face, possibly trimmed, reoriented, approximated, or placed by
the operation's recorded construction. It need not occupy the input face's world
position. A Cut's tool wall may contribute with reversed material orientation.
The relationship states construction ancestry; it states neither area equality nor
whole-face survival.

Keep face correspondence separate from role provenance. In particular:

- Analytic section rewrites keep their fresh roles under `prism-boolean-design.md`
  §11 and `general-boolean-design.md` §4.2.
- Copy and Transform keep their existing per-payload provenance rules under
  `api-design.md` §8.
- Canonicalization keeps the existing union of contributing origin roles.
- Matching origins do not establish correspondence between two particular faces.
  Several disconnected source faces can carry identical origins.
- History faces are read-only inspection handles. A consuming operation still takes
  selectors, never a history output face pointer or its position in a returned slice.

`HistoryComplete` means the builder has accounted for every output patch derived
from that input face. With complete coverage, an empty result means this operation
returned no boundary contribution from that face. The input body may nevertheless
have contributed material, and the face remains readable on the retired input.
Do not name this outcome body retirement or infer material removal from it.

`HistoryUnknown` means coverage is not established. It can carry known positive
links, but an empty result proves nothing. Return unknown coverage for a route
without an attribution certificate; never infer a negative result from absent roles,
matching coordinates, small residuals, or a source group that was not tracked.

Allow one input face to lead to several output faces and several input faces to
lead to one output face. A trimmed face that splits into disconnected patches must
return all its certified patches. A canonicalization merge must retain every
certified incoming face relationship.

The initial correspondence scope is face-to-face only. It does not classify new
blend faces by their source edges or trace sketch entities, vertices and edges.
A face can therefore have no predecessor in this API. Do not require every output
face to have an input-face predecessor.

## Builder evidence

Record actual source-face identity while evaluating, before source information is
reduced to roles. Retain computational evidence in the committed result payload;
derive public face correspondence from the pending operation's map.

| Route | Evidence required for complete face correspondence |
|---|---|
| Copy or Transform | The payload rebuild's explicit original-face to rebuilt-face map, including canonicalization. Equal role strings alone are insufficient. |
| Unstitch | The original face passed to each real single-face sheet builder and that builder's result face. |
| Mesh boolean | Operand source-face identity carried through facet cutting, source grouping and final connected-patch assembly. |
| Analytic section rewrite or modify | Certified source spans or patches carried through the actual arrangement and reconstruction. Until available, report unknown. |
| Multi-step compound operation | Validated composition of the private correspondence from every internal step. A missing step makes affected coverage unknown. |
| Remove | Complete empty correspondence for every face of its consumed body. |

`internal/facetedrecord/Group.Origins` currently stores role provenance, not the
identity of the immediate operand face. Preserve the immediate face identity in
operation-local data before grouping; do not reconstruct it from `Origins`.
The final patch assembly can supply several output faces for one group.
Retained analytic surface tags are separate evidence and do not prove whole-face
survival or upgrade geometry exactness.

The first release must demonstrate the tighter calculation in First calculation
fixture. It need not make every public face query complete. Record every successful
public modeling call, and report unproved face coverage as unknown. Unproved
attribution never refuses a valid geometry route.

## Atomic publication and storage

Prepare a record alongside geometry evaluation. Validate its ownership, input uses,
output membership, face links, and coverage before the final cancellation check.
Publish it in the same critical section that changes live membership. A record must
never become visible before every result is ready or after only some inputs retire.

Extend the existing single-output, multi-output and producer-span commit paths;
do not equate commit count, output count and producer span. Keep public operation
grouping above internal boolean and placement helpers. Temporary pattern instances,
tools and intermediate Union bodies are not public outputs or producer lookup keys.
They can supply internal attribution evidence without becoming history entries.

`Remove` appends its record while holding the live-set lock and does not advance
the producer counter. No new operation number is required. Failed geometry still
obeys `evaluator-design.md` §8, including its distinct level and curve counter rules.

Every document collects public operation history from creation. The records hold
strong references to input and output bodies, so retired geometry stays reachable
for the document's lifetime and for any surviving snapshot. Memory grows with
recorded bodies and face links, including bodies no caller otherwise retains.
This retention is the memory cost of complete run history for verification.
Boolean chain regroup reads retired source bodies through these references; an eviction
API would also have to state how that reader declines.
No eviction or clearing API is proposed because it would invalidate producer
lookup and completeness claims. Computational proof lineage remains distinct
from these public records and supplies its consumers directly from result payloads.

History snapshot creation shares the membership lock with Remove and commit.
Snapshots may run beside Bodies, Remove and Verify. This adds no permission for
concurrent feature mutation. Once obtained, a snapshot and its records are safe
for concurrent reads. Snapshotting must not rerun geometry or tessellation.

## Exactness and failures

History coverage describes evidence about a relationship, not numeric exactness.
Do not add an Exactness field to an operation or label all analytic history exact.
Read geometric quantities through the existing bounded Body and Face methods.
Any future numerical history summary must use those measurement types and their
units and bounds; no unbounded scalar or guessed tolerance belongs in this API.

| Condition | Proposed result |
|---|---|
| Nil lookup receiver, nil query body or face, or an unowned handle | `ErrDegenerate`. |
| Body or face belongs to another document | `ErrForeignBody`. |
| Valid body was produced after this snapshot | Producer and consumer lookup return `ErrNoMatch`. |
| Known body has no consumers within this snapshot | Consumers returns an empty slice without error. |
| Face belongs to this document but is not on an operation input body | `FaceSuccessors` returns `ErrNoMatch`. |
| Face is a valid input but this route lacks correspondence evidence | The query succeeds with `HistoryUnknown`. |
| Input body is retired | Read-only history lookup succeeds; feature liveness gates still reject reuse. |
| Geometry fails or context is canceled before commit | No history entry, retirement, or output registration occurs. |
| Pending record links a foreign or non-output face, or claims conflicting completeness | Reject before commit as an internal consistency error; never publish a false complete result. |

Do not capture wall-clock timings, failed attempts, context values, file paths,
source locations, parameter tables, or caller-owned option objects. The initial
public record contains only the specified metadata and immutable handles.
Computational source records additionally contain the evidence listed above.

## First calculation fixture

Start with a 20 mm cube and a wholly enclosed cylinder of radius 1 mm and length
4 mm, centered at (10, 10, 10), placed with a non-cardinal 20 degree tilt. Build the
cube and cylinder with real Extrude and placement calls. Run the real strict
containment producer and public Cut, then read the returned body's Volume.
The intended nominal volume is `8000 - 4*pi` cubic millimetres; include the actual
source placement and input bounds when comparing to this nominal expression.

Require a mesh-built result, one material lump, a real void shell, and complete
closure and orientation audits. Compare two evaluations from the same immutable
source records: the ordinary mesh volume path and the new source-difference path.
Disable only the new quantity-certificate consumer in the internal control;
do not disable the normal mesh proof or invent a certificate in a fixture.
Require a strictly smaller public Volume bound and a reference enclosure contained
within both proven intervals. Check the actual values as well as the bounds.
Do not demand Exact for pi. Verify must consume the tighter returned reading.

Inspect the always-available public snapshot and confirm its Cut record names the
actual inputs and result. Confirm that Cut retired both original inputs and that
the returned body's calculation does not depend on their live membership.
`operation_history_volume_test.go` implements this real fixture and checks the
containment proof, mesh output, selected Volume, Verify reading and public Cut
record. It also confirms the inputs retire while the result remains readable.

`operation_history_centroid_test.go` implements the centroid fixture with the
same real cube, tilted cylinder, containment proof and public Cut. It checks a
certified pi interval and source volume and centroid bounds against both
candidate centroid balls in 3D. Both balls must enclose the reference, and the
selected source-derived centroid must have a strictly smaller radius than the
audited mesh centroid. `Body.Centroid` and `Verify` must return the selected
reading.

## Focused repository proof fixtures

After that path works, add the following small cases under the existing test-scope
rules in `testing.md`. Names here describe proposed fixtures, not existing tests.
The mesh split-face and duplicate-origin correspondence fixtures gate later
expansion beyond the complete public face routes in Implementation scope.

| Fixture | Required proof |
|---|---|
| Source planes through a mesh Cut and later nearby Cut | Preserve the retained-plane slab calculation. Compare to the full-displacement control and require a strictly tighter new rim bound plus bounded computed geometry. |
| Retained plane through modify and later Cut | Feed the modified result into the consumer. Unchanged patches keep source certificates; changed patches need new proof or fall back. Compare local bounds and geometry. |
| Unproved containment or outer void inside cutter | Withhold source-volume subtraction. The actual Boolean keeps its current certified result or refusal, never a falsely tight volume. |
| Incomplete source domain or invalid placement transfer | Surface-only evidence cannot reconstruct trimmed geometry. The bounded mesh path remains the calculation path. |
| Analytic Cut of nested prism sections | Check both consumed inputs, the real output, fresh cap roles, bounded section-difference volume, and unknown face coverage until source spans exist. |
| A supported mesh Cut splitting a face | Feed the real result into the query. Both patches link to the exact source face. Assert patch connectivity and bounded area or coordinates. |
| Two input faces with equal origin-role sets | A real split result used by a later operation must keep their immediate face identities distinct. A role-keyed implementation must fail this fixture. |
| Unstitch of one box | All six real sheet results have one public producer record. Query each original face and verify its sheet geometry, area and free-edge topology. |
| PatternCopies and Patterned on minimal prisms | Copy outputs omit the still-live original. Patterned has one public record and one final output despite internal producer spans. Verify actual placements and combined volume. |
| A body-relative stop | ToFace records the referenced stop body. ThroughAll records the actual scanned set. Verify the computed extent and that reference-only bodies stay live. |
| Remove after Copy | The source has a zero-output Remove consumer. The copy's producer record and volume stay readable. The next real feature keeps the existing producer numbering rule. |
| Unsupported contact and cancellation | Compare live handles, public history contents and the next real feature's provenance against a control run. Repeat with a failure after some Unstitch outputs or pattern instances have been built. |
| Snapshots before and after consumption | An old snapshot does not acquire later consumers or outputs. Mutating returned slices does not alter either snapshot. Concurrent Remove and snapshot reads observe complete commits under the race detector. |
| Read-only consumers | Run real Verify/interference and tessellation on supported bodies. Quantities still satisfy their bounds and history gains no modeling entries. |

Use existing minimal cases in `apitest/document_context_test.go`,
`apitest/document_remove_test.go`, `apitest/boolean_test.go`,
`apitest/prism_boolean_test.go`, `apitest/pattern_test.go`,
`apitest/unstitch_test.go` and `apitest/selector_test.go` as fixture sources.
Assert bounded fields and explicit handle relationships. Do not deeply dump topology
graphs on failure. Run focused affected tests locally and leave full suites to CI.

## Implementation scope

The first implementation returns complete public face correspondence only for
Unstitch and Remove. Unstitch supplies an explicit source-face to result-sheet map;
Remove returns no faces. Other routes report unknown coverage, even when their
internal computational certificates can already tighten a particular calculation.
This separates the contained-Cut calculation from broader face-attribution work.

Provide one-operation face queries. Callers can walk recorded body dependencies;
the initial API does not compose face correspondence transitively. A transitive
query would require a separate work budget, cancellation, and rules for an unknown
intermediate mapping.

Exclude normalized feature arguments and selector definitions from public records.
The records answer execution ancestry, while the caller's Go function defines
feature arguments and rebuild behavior. This avoids a second per-option contract.
Internal computational certificates still retain the normalized source geometry
and quantity evidence their owning calculations require.

The contained-Cut Volume path is the first new computational deliverable; complete
face inspection alone is insufficient to accept the feature.
This scope preserves current stable feature roles.

## Ordered implementation tasks

Execute these phases in order on one implementation branch. Phases 0-6 are
implemented (#2405). Phase 7 below is the open work, and its proposed new file
and test names are destinations, not existing symbols. The owning
sections above define behavior; each phase's acceptance checks apply those rules.
Do not merge a partially instrumented public history API.

### Phase 0: Establish contracts and prerequisites

- Dependencies: This proposal's settled scope and the repository's public API
  approval described in Integration and behavioral reference.
- Files: `agent-docs/api-design.md`, `agent-docs/evaluator-design.md`,
  `agent-docs/param-binding-design.md`, `agent-docs/interference-design.md`,
  `agent-docs/faceted-source-retention-design.md`, and
  `agent-docs/payload-verification-design.md`.
- Deliverable: Approve Public API proposal and Operation kinds, including every
  public modeling entry point and the role names needed by body-based axes,
  neutral planes, face sections and placements. Record the internal quantity
  certificate's owner, validity checks and placement policy in the owning designs.
  Update the immediate-mode bookkeeping and mesh-measurement descriptions to
  permit the additional records and independently certified Volume reading.
- Inspection: Trace `document.go`'s `commitOperation*` helpers and `Remove`; `internal/clearance/pairkernel/kernel.go`'s `Pair` and
  `nestingRelation`; `boolean.go`'s `booleanBodyWith`; and
  `internal/facetedrecord/build.go`'s `BuildWithProof` before selecting seams.
  Inventory all public body producers, including wrappers that delegate to another
  public entry point; direct commit searches alone miss those wrappers.
- Focused checks to add later: Assert the complete public kind/token inventory
  and the separation between kind membership and calls that publish history.
- Acceptance: Every proposed public symbol, argument role and changed numerical
  claim has an owning contract. The strict-containment producer and the mesh
  consumer have named code seams and a bounded invocation plan.
- Stop condition: An unresolved public signature or proof contract blocks its
  implementation. A defect in a prerequisite requires a scope report before
  repair; do not expand another feature's admission to make this fixture work.

### Phase 1: Add unconditional storage and the atomic publication seam

- Dependencies: Phase 0.
- Files: New root `operation_history.go` and `operation_kind.go`, new
  `internal/operationhistory/`, `document.go`, `internal/topo/document.go`, and
  `topology_handles_test.go`'s document write guard.
- Deliverable: Define the root-owned public types and internal pending-record,
  validation, committed-record and index data. Initialize collection in `New`
  without an option. Store the public bookkeeping on the root document under
  `n.LiveMu`, or use an equivalent arrangement that preserves the topology
  package's import direction and root-only write rule.
- Deliverable: Make pending record validation precede the last context check.
  Pass a validated record to each public commit path; append its record and
  producer/consumer indexes while holding the same lock as retirement and result
  registration. Preserve each path's producer-span arithmetic. `Remove` validates
  and publishes its zero-output record inside its existing locked liveness check.
  Never add a default that silently omits a public record.
- Deliverable: Instrument only the real Extrude, placement and Cut routes needed
  by First calculation fixture before expanding the producer inventory. Keep
  private geometry-only builders record-free. Provide the query subset needed
  to inspect that first path; Phase 6 completes the full query matrix.
- Focused tests: Add small root tests in `operation_history_test.go` for pending
  ownership, duplicate consumed membership, output membership, foreign face
  links and conflicting completeness. Add exported-API cases in proposed
  `apitest/operation_history_test.go` for an empty document and the first path's
  ordered inputs/outputs. Test atomic rejection before changing either index or
  membership; compare handles and scalar counters, never whole topology graphs.
- Acceptance: A snapshot cannot observe a record with missing outputs or partially
  retired inputs. Remove publishes no new producer identity. Failed calls leave
  public history and producer numbering unchanged; level and curve counters keep
  evaluator §8's separate failure behavior.

### Phase 2: Produce an immutable strict-containment certificate

- Dependencies: Phase 0; Phase 1 supplies the first path's publication seam.
- Files: `internal/clearance/pairkernel/kernel.go` and `geom.go`,
  `internal/verifyrun/interference.go`, new `internal/prooflineage/` records and
  tests, and the source-record adapters for `internal/prismrecord/` and
  `internal/revolveplan/` needed by the fixture's actual payload classes.
- Deliverable: Add a typed proof result issued where Pair proves strict whole
  containment. Preserve Verify's existing verdict, contained-source reading and
  refusal semantics. `Result.Contained` currently names a body; that pointer
  alone is not the new certificate. Capture the eligibility, positive boundary
  gap, both directions of shell membership and the source-set identities and
  placements while the proof's actual inputs are present.
- Deliverable: Freeze the computational certificate into plain immutable source
  records and tokens. Retain the required domains, orientation, bounded source
  Volume readings and placement charges. Do not retain `*topo.Body`, faces,
  public handles, the document, a pair cache, contexts or mutable caller options
  in the committed proof lineage.
- Deliverable: Thread a matching certificate through existing geometry work when
  that work actually performs the same relation proof. Public Cut currently does
  not call Pair merely by entering the Boolean dispatcher; add one optional
  bounded invocation when no matching proof exists. Share the operation's context
  and a bounded work allowance, including nested witness walks. Do not call
  document-wide Verify or repeat proof work after commit.
- Focused tests: Add pair-kernel tests using real built analytic sources for the
  tilted contained cylinder, reverse argument order, an outer void inside the
  cutter, unsupported payloads, grazing or undecided casts, optional budget
  exhaustion and cancellation. Extend Verify regression cases to compare its
  bounded interference reading before and after this refactoring.
- Acceptance: Only the existing full-containment argument issues the certificate.
  An unproved or budget-exhausted optional attempt supplies no certificate;
  cancellation remains an operation failure. Verify still performs no modeling
  publication. A certificate cannot be reused for a different source or placement.

### Phase 3: Consume the certificate in the real mesh-built Cut

- Dependencies: Phases 1 and 2.
- Files: `boolean.go`, `boolean_mesh.go`, `internal/booleandispatch/`,
  `internal/booleaneval/eval.go`, `internal/facetedrecord/payload.go`,
  `build.go` and `assemble.go`, plus a focused numerical consumer in
  `internal/prooflineage/`. Update `internal/facetedrecord/place.go` and
  `placed.go` where new payload fields must transfer or be dropped.
- Deliverable: Attach the matching relation and source-quantity certificate to
  the resulting immutable faceted payload. Build and audit the actual mesh,
  shells, orientation and ordinary mesh Volume first. Compute source subtraction
  with outward rounding and both source bounds. Validate that both intervals
  intersect before publication, and cache the strictly tighter reading on the
  result body. Keep the ordinary reading on equal bounds or absent optional proof.
- Deliverable: Retain both proven intervals and the evidence needed to check
  them. Preserve held-mesh integration, `VolSymDiff`, `MeshBound`, `AreaSlack`,
  vertex bounds and all topology audits. Do not let a tighter scalar substitute
  for mesh proof, closure or source-face attribution. Derive Exactness from the
  selected bound; do not label the cylinder's pi calculation Exact.
- Deliverable: Preserve evidence through placement only with a proved transfer
  and fresh charges. Otherwise discard the optional quantity certificate and
  recompute through the existing path. Do not tighten `MassProperties`, area,
  centroid or inertia from Volume alone. The separate contained-Cut centroid
  consumer uses bounded source first moments and its own placement and
  arithmetic charges. `internal/massdispatch/dispatch.go` continues to require
  its own moment proof for `MassProperties` and inertia.
- Focused tests: Add consumer arithmetic tests for representable subtraction,
  nonzero source bounds, rounding, equal-bound selection, mismatched identities,
  invalid units or non-finite data, and disjoint intervals. Use a test-only
  internal control that disables only certificate selection on the real
  producer's result. Never provide a fixture constructor that invents admission.
- Acceptance: `Body.Volume` and Verify read the selected cached measurement.
  Contradictory stored evidence fails before commit; missing evidence keeps the
  ordinary certified result. A retired source does not invalidate the reading.

### Phase 4: Gate expansion on the first real end-to-end calculation

- Dependencies: Phases 1 through 3; execute this gate before Phases 5 through 9.
- Files: Proposed root `operation_history_volume_test.go`, the small public
  snapshot case in `apitest/operation_history_test.go`, and
  `internal/booleaneval/export_test.go` or the narrowest existing test hook seam.
- Deliverable: Implement First calculation fixture using its real source
  builders, placement, strict-containment producer, public Cut, result Volume
  and Verify. Use one smallest input before adding a matrix. Inspect the actual
  mesh payload, one material lump, void shell, closure and orientation audits,
  actual retired inputs and the public Cut record.
- Focused proof: Compare the ordinary and source-derived intervals from the same
  immutable source records. Check nominal value, source placement charges, a
  reference enclosure contained in both intervals and a strictly smaller public
  Volume bound. Check Verify's reading and tolerance result through its real
  measurement path. Do not add a second consumer model or hand-written proof.
- Acceptance: The real producer's certificate reaches the real returned-body
  consumer, and the computed result satisfies every First calculation fixture
  requirement. Public face coverage may still be unknown on Cut.
- Stop condition: Stop expansion if the real containment kernel cannot certify
  the pair, the mesh path refuses it, a real void shell fails its audits, the
  bound does not improve strictly, or the intervals disagree. Report the failed
  seam, current reading/bounds, smallest reproduction, proposed prerequisite
  repair, scope and measured cost where available. Do not alter dimensions,
  replace the producer with a fixture verdict, relax proof gates, or implement
  general containment/reconstruction as an unapproved prerequisite repair.

### Phase 5: Record every public producer and its actual body dependencies

- Dependencies: Phase 4 passes. Re-read owning feature designs before changing
  each operation family.
- Deliverable: Complete the following route inventory. Record the public caller's
  entry point once, including successful calls with no new instances. Reuse
  private geometry helpers rather than publishing delegated public calls.
  Preserve actual consuming/reference behavior and public argument order.

- `extrude.go`, `draft.go`, `revolve.go`, `coil.go`, `stops.go`, `internal/extent/`:
  Carry body handles from actual extent, axis and neutral-plane resolution.
  Capture the entire ThroughAll scan before bounds exclusions; do not recover
  handles from producer roles or rescan after evaluation. Add real ToFace,
  ToFaceAngular, EdgeAxis and ThroughAll fixtures. Check computed extents and
  live reference bodies, including an excluded scanned body.
- `sweep.go`, `loft.go`, `loft_between.go`, `loft_point.go`, `loft_sections.go`,
  `loft_faces.go`, `cone_from_face.go`, `patch.go`: Cover profile and chain
  overloads, reductions and delegated routes. Capture owning bodies for face
  sections and tangent ends. Keep the caller's kind through private builders.
  Add minimal supported cases for each public entry and distinct publication
  path. Check bounded volume, area or coordinates.
- `boolean.go`, `boolean_many.go`, `internal/booleandispatch/`: Cover analytic,
  mesh, specialized and multi-operand routes. Retain caller operand order and
  one public Union despite a private chain or producer span. Add nested analytic
  Cut, real mesh Cut, three-input Union and supported private-chain fixtures.
  Check volume and retirement.
- `document.go`, `mirror.go`, `mirror_join.go`, `point_plane_scale.go`,
  `pattern.go`: Cover Transform, Copy, Mirrored, MirroredCopy, ScaledAt,
  PatternCopies, Patterned and PatternJoin. Capture placement bodies. Exclude
  the original from PatternCopies outputs and every transient from producer
  lookup. Add minimal prisms with measured placements and volume. Query each
  output's producer. Check fast/fallback joins and numbering after the span.
- `fillet.go`, `chamfer.go`, `shell.go`, `brep_modify.go`, `draft.go`: Pass the
  original public kind through every modify commit and span helper. Record all
  distinct body reads without changing fresh role policy. Use small supported
  receivers and existing modify fixture families. Check changed geometry and
  unchanged selector-role rules.
- `hole.go`, `rib.go`, `emboss.go`, `tool_combine.go`: Group Hole, Rib, Web and
  Emboss above private tool construction and repeated Boolean steps. Keep tools
  and intermediate bodies out of public outputs and producer indexes. Add one
  supported minimal call per entry, plus failure after private work. Check
  volume and exactly one committed public record.
- `thicken.go`, `offset.go`, `patch_body.go`, `stitch.go`, `unstitch.go`,
  `surface_trim.go`: Cover Thicken, Offset, PatchEdges, Stitch, Unstitch, Trim,
  Extend and Split. Register all returned bodies in one record and preserve
  each operation's current input use. Unstitch one box, Split a minimal
  supported source, and Stitch real sheets. Verify sheet areas/free edges
  and solid volume as applicable.
- `document.go` Remove path: Publish a consumed input with no outputs and no
  producer-counter advance. Copy and Remove the original, read both historical
  producer records, and build the next real feature.

- Acceptance: Every successful public body-producing call and Remove contributes
  exactly one record. A read-only call, failed call or canceled call contributes
  none. Inventory tests fail when a public entry or commit route is omitted;
  private helpers, temporary tools and internal producer identities are never
  mistaken for public operations.

### Phase 6: Complete snapshots, indexes and initial face queries

- Dependencies: Phases 1 and 5.
- Files: `operation_history.go`, `internal/operationhistory/`, `unstitch.go`,
  `internal/stitchrecord/`, proposed `apitest/operation_history_test.go` and
  root `operation_history_test.go`.
- Deliverable: Implement every Public API proposal method and Exactness and
  failures lookup outcome. Freeze snapshot indexes so an old snapshot gains no
  later consumers. Preserve stable operation pointer identity across snapshots.
  Copy every returned slice and order face results by actual output traversal.
- Deliverable: Build Unstitch's map from each original face passed to the real
  sheet builder and the resulting sheet face. Give Remove complete empty maps.
  Every other initial route returns unknown coverage, even when computational
  evidence exists. Validate input-face ownership and output membership, deduplicate
  face links, and allow one-to-many and many-to-one maps internally.
- Focused tests: Cover nil and unowned receivers/handles, foreign documents,
  retired inputs, bodies created after a snapshot, reference-only inputs,
  repeated roles, no consumers and a face outside the operation's inputs.
  Mutate returned slices and prove records unchanged. Check six real Unstitch
  sheets, Remove's empty complete results, and unknown Cut/Copy results.
- Acceptance: All query and snapshot semantics match Public API proposal and
  Exactness and failures. Empty unknown results never imply disappearance;
  inspection handles cannot bypass live-input or selector gates.

### Phase 7: Carry source patches through later real consumers

- Dependencies: Phase 4 and Phase 6. `operation_history_volume_test.go` and
  `operation_history_centroid_test.go` run First calculation fixture on the
  real producer. `Operation.FaceSuccessors` returns complete maps for Unstitch
  (`unstitch.go`) and Remove (`document.go`) and `HistoryUnknown` elsewhere.

The retained-plane certificate already travels with the faceted payload. Each
stage carries it as follows:

| Stage | Code | What it carries |
|---|---|---|
| Face-to-group assignment | `booleaneval/eval.go` `evaluate` | One group per operand face; `SourceTag` copies the admitted tag. |
| Facet cutting and weld | `meshbool` `SourceIDs`, `KeptFacet.Src`, `StitchFacetsContext` | Each kept or split facet's group index. |
| Patch assembly | `facetedrecord/assemble.go` | One face per connected patch of a group, with the group's tag. |
| Edge chains | `facetedtopology` | Per-face indices only; it never reads groups. |
| Later mesh Boolean | `booleaneval/retained_plane.go`, `meshbool/plane_slab.go` | The slab bound of retention §11 for an admitted plane facet. |
| Placement | `facetedrecord/placed.go`, `place.go`, `point_plane_scale.go` | Every group, with `NormalBound` recomputed by retention §3.4. |
| Retained blend | `retainedblend/route.go` and the five `*_rewrite.go` files | The copied groups, plus one new blend group. |

All paths are under `internal/`. `evaluate` orders groups as
`topobuild.Faces(a)` then `Faces(b)`, so a group index is the operand face's
identity within one operation. Its `sourceID` map from face pointer to group
index is local and dropped on return. A shifted retry keeps only `Origins` for
the moved operand. Assembly sets `RetainedTag` on a patch when retention A4
admits it, and no faceted path merges two groups. `facetedtopology` needs no
change in this phase.

`retainedPlaneOf` reads the operand mesh's `Source` face. It admits a
`RetainedTag` `Plane` with `NormalBound == 0` and no denoted patch.
`RefuseCoarseHeldContact` and `RimBound` then read the slab bound in place of
`FacetBound`. Any departure that placement states ends the admission.
`retainedblend` copies `fp.Groups`, clearing only the whole-group centroid of
the shrunk faces. It appends one blend group with `Retained`, `NormalBound == 0`
and the exact blend surface. An R1, R3 or R5 chamfer adds a `Plane`. The five
rewrite files are `line_rewrite.go`, `cyl_rewrite.go`, `cyl2_rewrite.go`,
`arc_rewrite.go` and `chain_rewrite.go`.

A retained plane therefore reaches a later mesh Cut through mesh Booleans,
exact-translation placements and retained blends. `apitest/faceted_plane_slab_test.go`
checks the Cut-then-Cut case against a fixed bound literal.
Root `faceted_source_patch_test.go` checks it against the full-displacement
control of Task 7.1. No test checks the slab after a blend. `Group` records no source identity, and
`Group.Origins` holds roles only, so no record says which earlier patch a group
descends from.

Phase 7 consists of four tasks. Each merges on its own, and the first PR also
carries this document.

| Task | Depends on | Can run beside |
|---|---|---|
| 7.1 Slab control and Cut-then-Cut fixture | none | 7.3 |
| 7.2 Retained blend then Cut fixture | 7.1 merged | 7.3, 7.4 |
| 7.3 Immediate source identity | none | 7.1, 7.2, 7.4 |
| 7.4 Fallback fixtures | 7.1 merged | 7.2, 7.3 |

Tasks 7.1, 7.2 and 7.4 add root-package tests in separate files, so their
branches do not conflict. Task 7.3 edits the face-to-group loop of
`booleaneval/eval.go`; Task 7.1 edits only the `PrepBoolMesh` path in the same
file. Each fixture builds bodies through the public API, then reads internal
fields from the root package. It compares two runs: the ordinary run, and a
control run that disables only `retainedPlaneOf`. Each run uses its own document.
The control refuses every second cut of the P3 scene, so a fixture compares the
real run's build against the control's refusal, and where both runs build it
compares vertex lists in a loop and reports the first differing index only.
Never dump a mesh or topology graph on failure. Pin no bound literal, because
float bounds differ between amd64 and arm64; assert relations between the two
runs, or between the real run and the bound the control's refusal quotes. Show every fixture red first by the red leg named in its task.
Add each new test name to shard 10 of `.github/test-shards.txt`.

#### Task 7.1: Slab control and Cut-then-Cut fixture

- Files: `internal/booleaneval/retained_plane.go`, `eval.go`'s `PrepBoolMesh`
  path, and new root `faceted_source_patch_test.go`.
- Deliverable: Add `booleaneval.WithoutRetainedPlaneSlab(ctx)`, a context option
  in the style of `meshbool.WithContactWorkers`. Under it, `PrepBoolMesh` passes
  a plane reader that admits no facet, so every facet keeps `FacetBound`.
  Production code never sets the option. The full-displacement control is this
  option on the real producer's result; never construct a payload by hand.
- Fixture: Build retention §1's P3 scene in the root package: a 40×20×20 box,
  four lateral edges filleted at 3 mm, and a Ø8 y-bore at x = 20 placed by
  `Transform`. Run each second y-cut with and without the option.
  - Measured: Across second cuts at x from 5 to 37 and radii from 0.25 to 9,
    the real run builds and the control refuses every one that is not itself
    unbuildable. The control's refusal is the chain-depth gate: the first cut's
    plane facets hold a `FacetBound` of about 0.002973 mm, coarser than the
    pair's chord tolerance of about 0.0015 mm. Neither x nor the radius lowers
    the bound, because every y-cut touches the first cut's y = 0 and y = 20
    faces. No second y-cut builds in both runs, so no vertex-position
    comparison exists in this scene.
  - Cases x = 30 r = 4, x = 33 r = 2 and x = 10 r = 1: The real run builds and
    the control refuses with a plain `ErrUnsupported` that quotes its touched
    bound and the chord tolerance, the bound being larger. Every new rim
    vertex lies within its own bound of the exact circle, and the largest
    rim bound is strictly smaller than the quoted touched bound. The volume
    matches the closed form within its bound.
- Red leg: Make `PrepBoolMesh` ignore the option. The control then builds, and
  the refusal assertion fails.
- Stop condition: If a probe finds a second cut that builds in both runs and
  the runs differ in any vertex position, stop and report. Retention §11 claims
  the slab changes bounds and gates only.

#### Task 7.2: Retained blend then Cut fixture

- Files: New root `faceted_source_patch_blend_test.go`. No production change is
  expected.
- Fixture: Bore the 40×20×20 plate of `apitest/faceted_retained_blend_test.go`
  with a Ø8 z-bore at (20, 10) placed by translation. Fillet the top mouth at
  r = 1 (R2). Then cut beside the mouth so the new wall crosses top-plane facets
  between the blend's setback ring and the plate edge. In a second case, chamfer
  a straight retained edge (R1) and cut across the chamfer face.
- Assertions: After the blend, the top face is still a `RetainedTag` `Plane`
  with `NormalBound == 0`, and its frame is bit-equal to the bored plate's.
  The control cannot be relied on to build: the later cut builds in the real
  run, and the control refuses it on the touched plane bound. The top-plane rim
  vertex bounds all lie below the bound the control's refusal quotes. A cut that
  touches only the torus rim refuses in both runs, because a curved group takes
  no slab. The chamfer face is a `RetainedTag` `Plane` with `NormalBound == 0`,
  and its rim vertex bounds lie below the bound the control's refusal quotes. Every rim vertex lies within its bound of the
  exact circle. The volume removed by the later cut matches its closed form
  within the published bounds.
- Red leg: In a scratch edit, clear `Retained` on the copied groups in
  `retainedblend` `work.build`. The top plane then loses its slab, and the
  strictness check fails.
- Stop condition: If the blend's output does not reach the slab consumer, stop
  and report the failing stage. Do not repair `retainedblend` inside this task.

#### Task 7.3: Immediate source identity

- Files: `internal/facetedrecord/group.go`, new `internal/facetedrecord/source_patch.go`,
  `internal/booleaneval/eval.go`'s face-to-group loop, and the blend-group
  appends in the six `retainedblend` files of the table above. Update
  retention §3.1's group table, `retained-edge-blend-design.md` §5.1 and
  `evaluator-design.md` §9's group fields.
- Deliverable: Add `Group.Source *SourcePatch`. A `SourcePatch` is immutable and
  holds the operand body's `Origin.Producer`, the operand position, the face's
  ordinal in `topobuild.Faces`, and `Parent`. `Parent` is the operand face's own
  group record when the operand is faceted, and nil for a built face. A blend
  group's record names the modify producer and the edge's ordinal in the call.
  The record holds no topology node, body, document, context or cache.
- Deliverable: `evaluate` sets `Source` for every group, including a moved
  operand's group on the shifted retry, where the tag is still dropped.
  Placement and similarity copy the group struct (`placed.go`, `place.go`,
  `point_plane_scale.go`), so they keep the pointer with no new code.
  `retainedblend` keeps copied groups' records and gives each blend group a new
  one. A future path that merges groups must keep every incoming record.
- Deliverable: Identity admits nothing. `retainedPlaneOf` keeps reading the
  face's tag and bound only, and no route derives coverage from `Source`.
- Fixtures: In root `faceted_source_patch_identity_test.go`, split a retained
  top plane into two patches with a second cut. Both patches carry one record
  that names the top face, and their areas sum to the closed-form remainder
  within bounds. Cut that result again. The two patches become operand faces
  with equal `Origins` and get two distinct records with equal `Parent`. An
  untouched plane two Cuts later has the first Cut's record as `Parent` and a
  bit-equal frame. A Transform keeps the pointers. The public Cut record's
  `FaceSuccessors` still reports `HistoryUnknown` with no faces.
- Red leg: Key the record map by `Origins`. The equal-origins case then merges
  the two records.

#### Task 7.4: Fallback fixtures

- Files: New root `faceted_source_patch_fallback_test.go`.
- Fixtures: Each case runs with and without the option of Task 7.1. The control
  refuses in the P3 scene, so a case asserts that the real run refuses with the
  same touched bound as the control, or builds where the control refuses, plus
  the volume's closed form within its bound when it builds.
  - Invalid placement transfer: Make a 20° exact-rotation `Copy` of the P3
    result, and separately translate it by X = 0.1. First assert that each
    plane the later cut touches carries a nonzero `NormalBound`. The later cut
    then matches the control.
  - Valid transfer: An integer translation keeps `NormalBound == 0`, and the
    later cut still builds where the control refuses.
  - Denoted plane: Run a revolved part's cap through a mesh Cut, then a later
    cut. First assert that the cap carries a denoted patch. The later cut then
    matches the control.
  - Curved rim: The cut through the first bore's wall in
    `TestFacetedPlaneSlab` refuses identically in both runs.
- Red leg: Drop the `NormalBound == 0` test in `retainedPlaneOf`. The placed
  cases then tighten.

- Acceptance: A later calculation consumes the real prior builder's evidence.
  Source identity alone upgrades neither geometry nor public completeness.
- Expansion gate: Complete mesh, Copy, Transform and analytic public face
  mapping is a separate increment. It needs a pending list from group index to
  operand face, returned beside `Evaluation` and never stored in the payload.
  It also needs the real split-face and duplicate-origin fixtures above. Do not
  use `facetedtopology.UniqueGroupFace`, `GroupFace` or `TrimGroupFace` for a
  source that can produce several connected patches. Analytic section maps
  require actual arrangement spans; compound maps require every private step.

### Phase 8: Verify rollback, concurrency and retained memory

- Dependencies: Phases 5 through 7.
- Files: Proposed history tests, `apitest/document_context_test.go`,
  `apitest/document_remove_test.go`, existing pattern/Unstitch/Verify fixtures,
  and a focused history allocation benchmark if needed.
- Deliverable: Add failure/cancellation checks after some outputs or temporary
  instances have been built and immediately before publication. Compare public
  history, live handles and next successful provenance with a control run.
  Exercise snapshots beside Remove, Bodies and Verify under the race detector.
  Do not run concurrent feature mutation outside the documented contract.
- Deliverable: Document and inspect strong-reference retention explicitly.
  Measure a small repeated Copy/Remove history and snapshot allocation cost;
  report actual record/body/face-link counts and allocation data without imposing
  an invented memory threshold. Share immutable source records, and ensure
  computational records do not retain topology/document graphs or mutable buffers.
  Do not add eviction, clearing, lazy collection or snapshot-triggered collection.
- Focused tests: Verify old snapshots remain fixed after consumption; all outputs
  of Unstitch/Split appear together; canceled pattern/private-tool work publishes
  nothing; selectors, tessellation, export, Section, Verify/interference and
  motion queries add no records. Keep failure diagnostics bounded to identities,
  scalar readings, counts and capped differences.
- Acceptance: Race checks observe whole public commits, rollback matches the
  control, and retention behavior matches Atomic publication and storage.

### Phase 9: Update public documentation and finish required verification

- Dependencies: Every prior phase's acceptance criteria, including Phase 4's
  demonstrated tighter calculation.
- Files: `docs/api.md`, `docs/data-structures.md`, executable `examples/`,
  the owning design sections listed in Phase 0, affected layout part files,
  `.github/test-shards.txt` and `.github/test-shards-apitest.txt`.
- Deliverable: Document shipped signatures, copied-slice/snapshot behavior,
  unconditional collection, retained-memory cost, input uses, coverage and lookup
  errors. Add a small executable history example with verified output. Keep
  replay, parameter binding, feature roles and geometric exactness contracts at
  their existing owners. Mark later moments/reconstruction/face-map expansion as
  staged rather than claiming them from this implementation.
- Focused verification: Run only the affected new tests and packages locally,
  including targeted race cases. Add new root/apitest test names to shard 10
  under `testing.md`, then run `go test . ./apitest/ -run '^TestCI'` for manifest
  changes and the affected documentation/layout guards. Run executable examples
  that changed. Leave the full suite to configured CI.
- Required Go lint before PR: After the last Go edit and before the first push
  or PR creation, run `golangci-lint run --timeout=5m` using CI's v2.12.2 and
  `go vet ./...`. Store expensive output in `.tmp/`. Fix changed-code failures
  and rerun the failed checks; stop if lint cannot run or unrelated errors block it.
  Follow `~/.claude/docs/git-operations.md` for later pushes and PR CI Watch.
- Acceptance: Focused checks and required lint pass, public examples match the
  implementation, and the PR's full CI has completed before claiming the suite
  passes. A feature with complete inspection but no proven Volume improvement
  does not satisfy this implementation plan.

## Integration and behavioral reference

Submit these API symbols and input-role names through repository API approval
before implementation. That review is repository process for this settled scope.
Before implementation, extend the execution-bookkeeping descriptions in
`api-design.md` §2 and §6, `evaluator-design.md` §8, and the no-history wording in
`param-binding-design.md` §1 to point here. Preserve their immediate-mode and
no-replay decisions. Update `docs/api.md`, `docs/data-structures.md`, executable
examples, layout rows and test-shard manifests when the public API lands.
Update evaluator §9's mesh-measurement path and the owning payload/measurement
designs before shipping source-derived quantities; their current proofs do not
grant a general exactness upgrade from retained analytic surfaces.

The local OCCT checkout was read only to inspect observable per-operation lineage,
result availability and chained-result queries. The inspected files are
`src/ModelingData/TKBRep/BRepTools/BRepTools_History.hxx`,
`src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderShape.hxx`,
`src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.hxx`, and
`src/ModelingAlgorithms/TKBO/GTests/BRepTools_History_Test.cxx` under
`../../Open-CASCADE-SAS/OCCT` from the repository root. No OCCT code, comments,
taxonomy rules, or test fixtures are copied or adapted. Decad's immutable bodies,
atomic commits, private producers, bounded readings and existing role policies
determine this proposal.
