# Operation history design

Navigation only; the named sections own their rules.

| Question | Section |
|---|---|
| What calculation improves? | Computational consumers; First calculation fixture |
| What survives each operation? | Computational evidence; Admission and bounds |
| What can callers inspect? | Public API proposal; Face correspondence and completeness |
| What is collected and published atomically? | Atomic publication and storage |
| What does each implementation phase cover, in order? | Ordered implementation tasks |
| May internal code read public records? | Boolean chain regroup; Point location through history; Section through history |

## Status and scope

The implementation records every public body-producing call and `Remove`, and computes contained-Cut Volume,
centroid and mass-property readings from retained source evidence. `Remove` and `Unstitch` have complete face
maps; every other route reports unknown coverage unless its builder supplies an attribution map. Composite tools
stay outside the live set and public history until their outer call commits. The history changes no role,
geometry, liveness or measurement contract.

Construction history is evaluator evidence. Later calculations use analytic source geometry instead of
accumulating avoidable mesh error. For a certified contained Cut, the Volume reading uses the source volumes, the
centroid uses source first moments, and the mass properties use density-independent source volume, first and
second moments. Every document records public operation history from creation, for verification and inspection
of successful calls.

The caller's Go function is the feature tree. Proof records describe evaluated sets and constructions; they hold
no replayable feature commands, rebuild mechanism, undo or file format. Internal code may read public records to
re-run Boolean geometry over the retained bodies they name, to combine their point classifications, or to
section a mesh Boolean result from them. Boolean chain regroup, Point location through history and Section
through history below own those rules.

The owning contracts are `api-design.md` §2, §3, §5.3, §6 and §12; `evaluator-design.md` §3 and §8;
`prism-boolean-design.md` §11; `agent-docs/prism-boolean-design.md` Part G §4.2; and `param-binding-design.md` §3.
Those documents own existing behavior. This document owns the computational lineage, the source-quantity
consumer and the public history queries.

## Computational consumers

### Existing local Boolean bounds

`faceted-source-retention-design.md` §11 owns the retained-plane slab calculation and its admission rules; this
document does not repeat the formula. `booleaneval.retainedPlaneOf` supplies certified planes to
`BoolMesh.SetChargedSlabBounds`, and the slab certificate tightens the later held-contact gate and `RimBound`.
The full displacement for interior vertices and trim uncertainty stays separate.

The immediate source patch and its valid plane certificate travel through Boolean assembly, placement and
retained blends (Phase 7), so a later nearby Cut consumes the same evidence after earlier operations. Source
identity makes the certificate traceable; identity alone never admits the plane or changes a bound.

### Contained Cut volume

For a successfully built `Cut(A, B)`, a typed certificate proves that the whole true solid B lies in A's
material. The result keeps that set relation and the immutable source volume readings, and computes
`Volume(A) - Volume(B)` with outward rounding that includes both input bounds and the arithmetic rounding. This
reads the true source solids, not the tessellation's occupied-volume approximation.

The relation comes from the strict full-containment proof owned by `interference-design.md` §4. That proof
requires a single material lump, a positive boundary gap and shell witnesses in both directions, including void
shells. `pairkernel.Result.Containment` (a `*ContainmentCertificate`) is the per-call proof. `prooflineage.Containment`
freezes its boundary gap and witness counts into an immutable record attached to the result; no stored boolean
verdict stands in for it. A nesting witness or bounding-box containment is never the certificate.

`massdispatch.CaptureContainedCutQuantities` runs after the Boolean builds its result and before commit. It calls
`pairkernel.Pair` once, under the operation's context, with the target as outer and the tool as inner. The public
Boolean still builds and audits its real result, so this calculation never makes an invalid Boolean valid.
When the relation is unproved, the carrier is unsupported or the relation is undecided, the result keeps the
current mesh volume. The source-derived reading is selected only when its absolute bound is strictly smaller; a
tie keeps the current reading for stable output. Both proven intervals stay available for consistency checks, and
disjoint intervals are an internal consistency error before publication. `proofbound.WorkBudget` polls the
context and has no separate work-exhaustion result. Cancellation and other `Pair` errors take the existing
error path and abort the operation before commit.

### Contained Cut centroid

For a successfully built mesh `Cut(A, B)` with the same strict whole-containment certificate, the result centroid
derives from the bounded source volume and centroid readings. Each world-coordinate first moment is taken about
the target centroid anchor `o`:

```text
o + [VA·(CA − o) − VB·(CB − o)] / [VA − VB]
```

The denominator is the already-derived source-volume difference. `proofbound` bounded arithmetic propagates both
source volume bounds, both source centroid radii and each arithmetic rounding term. The denominator's proven
lower endpoint must be positive. If its interval crosses zero, the audited mesh centroid stays. If the interval
is wholly nonpositive, the source readings contradict strict containment and the call fails before commit. If
arithmetic overflows a finite reading, the mesh centroid stays.

The derived and audited mesh centroid balls are compared by exact held-center distance. Disjoint certified balls
are an internal consistency error before commit. The derived centroid is selected only when its radius is
strictly smaller; a tie or larger radius keeps the mesh result. This selection is independent of whether the
source-derived Volume wins its own bound comparison. `Verify` reads the selected `Body.Centroid` through its
existing centroid gate, and `Exactness` derives from the selected centroid's own bound. The centroid record never
feeds `MassProperties` or inertia.

Both candidate centroid readings and the selection result are stored as plain `CentroidLineage` values nested in
`VolumeLineage`. Source producer identities validate against the parent record. The lineage keeps no topology,
body, document, context or cache pointers. Public operation history records only the public call and its body
relationships.

### Contained Cut mass properties

For a rectangular-prism target and a whole circular-prism tool, both source records integrate into
world-coordinate V/P/Q moments about the target's held centroid. The copied source moments and their difference
freeze in the result's volume lineage before commit. The moment record keeps producer identities and containment
facts, and each query validates them before reading independent copies.

The faceted `MassProperties` path obtains its audited mesh tuple first. It then publishes a source-derived
candidate with the caller's density and checks mass, all six inertia entries and both center balls against the
mesh and the selected body centroid. Any disjoint evidence fails the query. A consistent source tuple replaces
the mesh tuple only when all eight bounds are no larger and at least one is smaller. Unsupported integrations and
source publication limits keep the mesh tuple. One direct exact-rotation Copy of an admitted original Cut
transfers the frozen V/P/Q evidence (`contained-cut-moments-design.md` "Exact rotation copy"). Transform, default
Copy and later descendants do not transfer the record.

### Local modify and later measurements

Fillet and Chamfer keep unchanged source-patch certificates and replace affected patches with the modify
builder's certified trimmed and blend records. `internal/retainedblend` preserves surface groups and gives each
blend group a `BlendSource` record beside the copied groups; it never joins old and new roles. Retained-edge
routes can consume the resulting surfaces, rims and local bounds under `agent-docs/modify-design.md` Part E. They
gain no admission from an operation name, an ancestry pointer or a complete public face mapping.

`agent-docs/contained-cut-moments-design.md` owns the mass-property slice. Other operation classes need their own
moment evidence and selection rules before they can improve `MassProperties`. A Volume or centroid certificate
alone cannot tighten area, wall thickness or the mesh's symmetric-difference bound.

Analytic reconstruction from retained source is not implemented. Reusing a source payload would need a
set-equality certificate, or a complete certified analytic trim boundary plus closure, orientation and
occupied-set proof. A carrier surface and held chord loop do not provide that boundary. An admitted
reconstruction builds a new immutable result through the existing analytic builder and fresh-role policy and
never replaces a returned body's topology. Unsupported reconstruction keeps the proven faceted result, and an
unknown trim is never re-tessellated as if it were an analytic patch.

### Boolean chain regroup

`agent-docs/prism-boolean-design.md` Part R owns the first reader of public records. When a two-body Boolean
refuses and an operand is a mesh Boolean result, it walks producer records to the retained source bodies and
evaluates a set-equal regrouping of the chain. It is not replay:

- It re-runs Boolean geometry only, over immutable retained bodies.
- It replays no feature call, re-solves no sketch and reads no parameter.
- It publishes no record beyond the caller's own Boolean and consumes no retired body.
- A missing producer record or an ineligible body makes it decline; it never refuses.

### Point location through history

`agent-docs/point-containment-design.md` §8 owns the second reader. `Body.LocatePoint` on a mesh Boolean result
walks the same producer records, under the eligibility regroup uses, to the retained operand bodies. It combines
their exact point classifications by the set algebra of the recorded operation. It re-runs no geometry, builds
no body and publishes nothing. A missing record or an ineligible body reads undecided and leaves the point to the
verified mesh.

### Section through history

`agent-docs/section-design.md` ("Mesh Boolean result read through its operation history") owns the third reader.
`Body.Section` on a mesh Boolean result reads its producer record under the same eligibility, takes its two
retained operands and returns the 2D operation of their exact sections. It re-runs no 3D geometry, builds no body
and publishes nothing. A missing record or an ineligible body returns `ErrUnsupported`.

## Computational evidence

The document keeps two distinct always-on records. Internal proof lineage holds the immutable construction data a
calculation needs. Public operation history records every successful modeling call and its caller-visible body
relationships. A snapshot exposes those public records; taking a snapshot does not enable their collection.

| Internal evidence | Required contents |
|---|---|
| Source patch | Immediate source identity, copied analytic record or immutable source-record reference, material orientation, placement, and its certified domain and bounds. |
| Boundary attribution | Operand patch to output patch map, actual arrangement spans or held facet groups, and completeness per region. Equal role sets are insufficient. |
| Relation certificate | Source-set identities and placements plus the typed whole-containment or equality proof and its eligibility checks. |
| Quantity certificate | The bounded source readings or source integrals, units, reference frame, and arithmetic or transformation charges needed to recompute the claimed quantity. |
| Modify certificate | The untouched patch map, the newly constructed trim and blend records, and the local displacement or occupied-volume terms required by its consumer. |

The evidence is stored with the immutable result payload as compact source descriptions and certificates, never
arbitrary caller objects or the whole document. Immutable source records are shared where practical. A source's
retirement does not change its mathematical geometry, so readers use retained evidence without reviving a body
or changing any feature's live-input gates. Live-source requirements such as `CertifyCopies` stay in force until
their owning design changes. Pending maps can key topology pointers. Committed computational links use private
source-record tokens and immutable descriptions, not topology nodes that retain their owning body and document.
Public records retain body handles.

`Group.Origins`, `RetainedTag`, `CapRecord`, denoted patches, per-vertex bounds and typed source records each
prove a different claim. Their owning contracts stay in force, and no universal history-present flag replaces
them. Immediate source identity is assigned at `booleaneval.Evaluate`'s face-to-group assignment and carried
through final connected-patch assembly (Phase 7). Identity can key evidence, but it cannot manufacture a missing
domain, exact curve, containment relation or error bound.

## Admission and bounds

Each computational consumer validates the exact evidence it reads. A valid source surface is kept when trim
evidence is incomplete, but it serves only the calculations its carrier and normal certificates prove. Missing
optional evidence selects the existing certified path. A contradictory stored certificate is an internal error,
never concealed as an ordinary missing optimization.

The original `MeshBound`, vertex displacements, `AreaSlack` and `VolSymDiff` stay unless a separate proof
tightens that same quantity. A smaller Volume bound does not shrink the mesh's geometric error. A local plane
slab bound tightens only the contact calculations named by its owner. Placement and rounding charges are
recomputed through r3 and the existing bound machinery, and evidence whose placement transfer is unproved is
dropped. A small residual can reject nothing and admit nothing.

Exactness derives per returned reading from its own proven absolute bound. Exact source records plus an exactly
representable subtraction can give an Exact Volume; a cylinder involving pi normally keeps a small positive bound
and Approximate. Retained surface tags, complete face history and an exact containment relation do not by
themselves make a numerical reading Exact. `Verify` reads the selected cached Volume with its new bound through
the existing tolerance gate. A later calculation can reuse that bound only with a certificate for its own
operation.

## Concrete questions

| Caller question | Answer |
|---|---|
| Can a contained curved cutter avoid mesh volume error? | Its certified set relation permits a source-volume difference with arithmetic and input bounds. |
| Can a later nearby Cut use a retained plane? | Its valid plane and local slab certificates tighten contact bounds through the existing Boolean path. |
| Which call made this retired body? | Its successful public operation and its actual input and output body handles. |
| Did a copy consume its source? | Its source input is recorded as a reference; a later consuming call is separate. |
| Why did a body leave `Bodies()`? | The operation consuming it, including an explicit `Remove` record. |
| Which faces of this Cut came from this particular tool face? | A one-operation face correspondence result with explicit completeness. |
| Did an analytic Union erase a source role? | Existing selectors follow existing role rules; history can still report body dependencies. |
| Is an empty face result proof that the source disappeared? | Only an exhaustive correspondence result proves no output boundary contribution. |

History records body dependencies, not parameter dependencies. A preceding `Document.Param` read is not
attached to the next call. The binding ledger records reads independently, as `param-binding-design.md` §3
specifies.

## Public API proposal

The caller-visible types live in the root package with unexported storage and accessors (`operation_history.go`,
`operation_kind.go`). The record, validation and index code lives in `document.go` and `operation_history.go`;
no separate internal package exists.

| Symbol | Behavior |
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

The public record does not expose or certify internal reconstruction evidence. `OperationInput` is an
inspectable value containing `Body *Body`, `Role string` and `Use InputUse`. `InputUse` distinguishes
`InputConsumed` from `InputReferenced`. Roles name argument purposes: `target`, `tool`, `source`, `operand`,
`stop` and `throughAll`. Repeated argument purposes keep caller order. A body read in several roles has several
entries; consuming membership is deduplicated by body. These input roles are metadata and never become
`FeatureRef.Role` values.

`*Operation` identity is scoped to the document and stable across its history snapshots. No numeric operation ID
is exposed and no `FeatureRef` serves as one. An operation with several outputs can span several private
producer identities; an operation with no outputs can span none.

Every returned slice is a copy. Bodies, faces and operation records are immutable. Changing the live model after
a snapshot cannot change that snapshot's records, consumer results or face correspondence. It can change current
body liveness, and history is never permission to consume a retired body. Reading a retired body's immutable
geometry inside a later call's private evaluation is not consumption: it changes no liveness and publishes no
record.

## Operation kinds

This section owns the shared root-package `OperationKind` vocabulary for history and
`operation-diagnostics-design.md`. Diagnostics can name failed calls and selector resolution without a committed
public history record.

Zero is `OperationUnknown`. Each public modeling or selector-resolution entry point has one documented value,
including `OperationSelectEdges` and `OperationSelectFaces`. `OperationKind.String() string` returns stable
lower-snake tokens: `unknown`, `extrude`, `cut`, `patterned`, `remove`, `select_edges` and `select_faces`
illustrate the spelling rule. Existing tokens never change meaning. Clients must handle unknown and future values
with a default branch. A kind reports no evaluator route, payload kind, mesh index or internal producer number.
Vocabulary membership does not imply a public record: the next section owns which calls produce history records.
Selector resolution produces none.

## Recorded calls and dependencies

One entry records each successful public body-producing call, plus each successful `Remove`. The entry names the
public entry point the caller used. A `Hole` is one Hole record, and its private tool construction and cuts
create no public history entries. The same rule applies to WebPath, Emboss, mirror joins and pattern joins. If
the caller explicitly invokes Copy followed by Union, those are two entries.

| Call shape | Inputs | Outputs |
|---|---|---|
| Feature built from a sketch | No body inputs unless its extent or another argument reads a body. | The returned body. |
| Cut or Intersect | Consumed target and tool, in public argument order. | The returned body. |
| Union | Consumed operands, in public argument order. | The final returned body. |
| Transform or a modify operation | Consumed receiver, plus any distinct body dependencies. | The returned body. |
| Copy or MirroredCopy | Referenced source and any body used to define placement. | The new copy. |
| PatternCopies | Referenced source. | The returned new instances; the original is not an output. |
| Patterned | The public consumed inputs. | The final returned body. |
| Unstitch | Consumed receiver. | All returned sheets in one operation. |
| Remove | Consumed body. | No outputs. |

Body-relative extent and placement dependencies are recorded even when the body stays live. For `ToFace`, the
stop body's handle is recorded; a selector or face reference does not replace the owning body dependency. For
`ThroughAll`, the record holds the live-body snapshot whose bounds the evaluator used, including bodies excluded
by its bounds tests. The document is not rescanned after evaluation to reconstruct those dependencies.

Read-only calls record nothing: selectors, tessellation, export, Section, measurements, Verify, interference
evaluation and motion checks create no modeling records. In particular, the read-only boolean evaluator used by
Verify never emits an Intersect record. A failed or canceled public call emits no record.

## Face correspondence and completeness

A successor is an output face whose boundary patch the builder derives from some part of the input face,
possibly trimmed, reoriented, approximated, or placed by the operation's recorded construction. It need not
occupy the input face's world position. A Cut's tool wall may contribute with reversed material orientation. The
relationship states construction ancestry; it states neither area equality nor whole-face survival.

Face correspondence stays separate from role provenance:

- Analytic section rewrites keep their fresh roles, and the operand roles their planar faces inherit, under
  `prism-boolean-design.md` §11 and `agent-docs/prism-boolean-design.md` Part G §4.2.
- Copy and Transform keep their existing per-payload provenance rules under `api-design.md` §8.
- Canonicalization keeps the existing union of contributing origin roles.
- Matching origins do not establish correspondence between two particular faces. Several disconnected source
  faces can carry identical origins.
- History faces are read-only inspection handles. A consuming operation takes selectors, never a history output
  face pointer or its position in a returned slice.

`HistoryComplete` means the builder has accounted for every output patch derived from that input face. With
complete coverage, an empty result means this operation returned no boundary contribution from that face. The
input body may nevertheless have contributed material, and the face stays readable on the retired input. This
outcome is not body retirement, and material removal is not inferred from it.

`HistoryUnknown` means coverage is not established. It can carry known positive links, but an empty result proves
nothing. A route without an attribution certificate returns unknown coverage. A negative result is never
inferred from absent roles, matching coordinates, small residuals, or a source group that was not tracked.

One input face can lead to several output faces, and several input faces can lead to one output face. A trimmed
face that splits into disconnected patches returns all its certified patches. A canonicalization merge retains
every certified incoming face relationship.

The correspondence scope is face-to-face only. It does not classify new blend faces by their source edges or
trace sketch entities, vertices and edges. A face can therefore have no predecessor in this API, and no output
face is required to have an input-face predecessor.

## Builder evidence

Actual source-face identity is recorded while evaluating, before source information reduces to roles.
Computational evidence stays in the committed result payload; public face correspondence derives from the
pending operation's map.

| Route | Evidence required for complete face correspondence |
|---|---|
| Copy or Transform | The payload rebuild's explicit original-face to rebuilt-face map, including canonicalization. Equal role strings alone are insufficient. |
| Unstitch | The original face passed to each real single-face sheet builder and that builder's result face. |
| Mesh boolean | Operand source-face identity carried through facet cutting, source grouping and final connected-patch assembly. |
| Analytic section rewrite or modify | Certified source spans or patches carried through the actual arrangement and reconstruction. Until available, report unknown. |
| Multi-step compound operation | Validated composition of the private correspondence from every internal step. A missing step makes affected coverage unknown. |
| Remove | Complete empty correspondence for every face of its consumed body. |

`Group.Origins` stores role provenance, not the identity of the immediate operand face. `Group.Source` holds that
identity (Phase 7, Task 7.3); a public face mapping reads it, never `Origins`. The final patch assembly can supply
several output faces for one group. Retained analytic surface tags are separate evidence and do not prove
whole-face survival or upgrade geometry exactness.

The tighter calculation in First calculation fixture does not need every public face query complete. Every
successful public modeling call is recorded, and unproved face coverage is reported as unknown. Unproved
attribution never refuses a valid geometry route.

## Atomic publication and storage

A record is prepared alongside geometry evaluation. Its ownership, input uses, output membership, face links and
coverage validate (`validateOperation`) before the final cancellation check. The record publishes in the same
critical section that changes live membership, so it is never visible before every result is ready or after only
some inputs retire.

The single-output, multi-output and producer-span commit paths (`commitOperation*` in `document.go`) each take a
validated record. Commit count, output count and producer span are distinct. Public operation grouping sits above
internal boolean and placement helpers. Temporary pattern instances, tools and intermediate Union bodies are not
public outputs or producer lookup keys; they can supply internal attribution evidence without becoming history
entries.

`Remove` appends its record while holding the live-set lock and does not advance the producer counter, so it
needs no new operation number. Failed geometry obeys `evaluator-design.md` §8, including its distinct level and
curve counter rules.

Every document collects public operation history from creation. The records hold strong references to input and
output bodies, so retired geometry stays reachable for the document's lifetime and for any surviving snapshot.
Memory grows with recorded bodies and face links, including bodies no caller otherwise retains. This retention is
the memory cost of complete run history for verification. Boolean chain regroup and point location read retired
source bodies through these references; an eviction API would also have to state how those readers decline. No
eviction or clearing API exists, because it would invalidate producer lookup and completeness claims.
Computational proof lineage stays distinct from these public records and supplies its consumers directly from
result payloads.

History snapshot creation shares the membership lock with Remove and commit. Snapshots may run beside Bodies,
Remove and Verify; this adds no permission for concurrent feature mutation. A snapshot and its records, once
obtained, are safe for concurrent reads. Snapshotting never reruns geometry or tessellation.

## Exactness and failures

History coverage describes evidence about a relationship, not numeric exactness. Operations carry no Exactness
field, and no analytic history is labelled exact. Geometric quantities are read through the existing bounded Body
and Face methods. Any future numerical history summary must use those measurement types and their units and
bounds; no unbounded scalar or guessed tolerance belongs in this API.

| Condition | Result |
|---|---|
| Nil lookup receiver, nil query body or face, or an unowned handle | `ErrDegenerate`. |
| Body or face belongs to another document | `ErrForeignBody`. |
| Valid body was produced after this snapshot | Producer and consumer lookup return `ErrNoMatch`. |
| Known body has no consumers within this snapshot | Consumers returns an empty slice without error. |
| Face belongs to this document but is not on an operation input body | `FaceSuccessors` returns `ErrNoMatch`. |
| Face is a valid input but this route lacks correspondence evidence | The query succeeds with `HistoryUnknown`. |
| Input body is retired | Read-only history lookup succeeds; feature liveness gates still reject reuse. |
| Geometry fails or context is canceled before commit | No history entry, retirement, or output registration occurs. |
| Pending record links a foreign or non-output face, or claims conflicting completeness | Rejected before commit as an internal consistency error; a false complete result is never published. |

The public record captures no wall-clock timings, failed attempts, context values, file paths, source locations,
parameter tables or caller-owned option objects. It contains only the specified metadata and immutable handles.
Computational source records additionally contain the evidence listed above.

## First calculation fixture

The fixture is a 20 mm cube and a wholly enclosed cylinder of radius 1 mm and length 4 mm, centered at
(10, 10, 10) and placed with a non-cardinal 20 degree tilt. Real Extrude and placement calls build the cube and
cylinder. The real strict containment producer and public Cut run, and the returned body's Volume is read. The
nominal volume is `8000 - 4*pi` cubic millimetres; the comparison includes the actual source placement and input
bounds.

The result must be mesh-built, with one material lump, a real void shell, and complete closure and orientation
audits. Two evaluations from the same immutable source records are compared: the ordinary mesh volume path and
the source-difference path. The internal control disables only the quantity-certificate consumer; it leaves the
normal mesh proof in place and invents no certificate. The public Volume bound must be strictly smaller, and a
reference enclosure must lie within both proven intervals. The actual values are checked as well as the bounds.
Exact is not required for pi. `Verify` must consume the tighter returned reading.

The public snapshot's Cut record must name the actual inputs and result. The Cut retires both original inputs,
and the returned body's calculation does not depend on their live membership.
`TestContainedCutUsesCertifiedSourceVolumeAndRecordsHistory` (`operation_history_volume_test.go`) implements this
real fixture and checks the containment proof, mesh output, selected Volume, `Verify` reading and public Cut
record. It also confirms the inputs retire while the result stays readable.

`TestContainedCutUsesCertifiedSourceCentroid` (`operation_history_centroid_test.go`) implements the centroid
fixture with the same real cube, tilted cylinder, containment proof and public Cut. It checks a certified pi
interval and source volume and centroid bounds against both candidate centroid balls in 3D. Both balls must
enclose the reference, and the selected source-derived centroid must have a strictly smaller radius than the
audited mesh centroid. `Body.Centroid` and `Verify` must return the selected reading.

## Focused repository proof fixtures

The table below states the proof each focused fixture owes, under the test-scope rules in `testing.md`. The
mesh split-face and duplicate-origin correspondence fixtures gate expansion beyond the complete public face
routes in Implementation scope.

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

Fixture sources are the minimal cases in `apitest/document_context_test.go`, `apitest/document_remove_test.go`,
`apitest/boolean_test.go`, `apitest/prism_boolean_test.go`, `apitest/pattern_test.go`,
`apitest/unstitch_test.go` and `apitest/selector_test.go`. Assertions use bounded fields and explicit handle
relationships and never dump topology graphs on failure. Affected tests run focused locally; CI runs the full
suites.

## Implementation scope

Complete public face correspondence exists only for Unstitch and Remove. Unstitch supplies an explicit
source-face to result-sheet map; Remove returns no faces. Other routes report unknown coverage, even when their
internal computational certificates already tighten a particular calculation. This separates the contained-Cut
calculation from broader face-attribution work.

Face queries are one-operation. Callers can walk recorded body dependencies; face correspondence does not
compose transitively. A transitive query would need a separate work budget, cancellation, and rules for an
unknown intermediate mapping.

Normalized feature arguments and selector definitions stay out of public records. The records answer execution
ancestry, while the caller's Go function defines feature arguments and rebuild behavior, so no second per-option
contract exists. Internal computational certificates still retain the normalized source geometry and quantity
evidence their owning calculations require.

The contained-Cut Volume path is the first computational deliverable; complete face inspection alone does not
accept the feature. Current stable feature roles are unchanged.

## Ordered implementation tasks

The phases below partition the work in dependency order. Phases 0-7 are implemented, and each phase section
states what exists. Phase 8 lists the rollback, concurrency and retention checks the history must satisfy.
Phase 9 lists the public documentation. The owning sections above define behavior; each phase's acceptance
checks apply those rules.

### Phase 0: Establish contracts and prerequisites

- Dependencies: This document's scope and the repository's public API approval described in Integration and
  behavioral reference.
- Files: `agent-docs/api-design.md`, `agent-docs/evaluator-design.md`, `agent-docs/param-binding-design.md`,
  `agent-docs/interference-design.md`, `agent-docs/faceted-source-retention-design.md` and
  `agent-docs/payload-verification-design.md`.
- Contract: Public API proposal and Operation kinds cover every public modeling entry point and the role names
  for body-based axes, neutral planes, face sections and placements. The internal quantity certificate's owner,
  validity checks and placement policy live in the owning designs. The immediate-mode bookkeeping and
  mesh-measurement descriptions permit the additional records and the independently certified Volume reading.
- Seams: `document.go`'s `commitOperation*` helpers and `Remove`; `internal/clearance/pairkernel/kernel.go`'s
  `Pair` and `nestingRelation`; `boolean.go`'s `booleanBodyWith`; and `internal/facetedrecord/build.go`'s
  `BuildWithProof`. The inventory of public body producers includes wrappers that delegate to another public
  entry point; a search for direct commit calls alone misses those wrappers.
- Check: The complete public kind/token inventory is asserted, together with the separation between kind
  membership and calls that publish history.

### Phase 1: Add unconditional storage and the atomic publication seam

- Dependencies: Phase 0.
- Files: root `operation_history.go` and `operation_kind.go`, `document.go`, `internal/topo/document.go`, and
  `topology_handles_test.go`'s document write guard.
- Storage: The root `Document` holds the public bookkeeping (`operations`, `producers`, `consumers`) under
  `n.LiveMu`, which preserves the topology package's import direction and root-only write rule. Collection
  starts in `New` without an option and has no default that omits a record.
- Publication: Pending record validation precedes the last context check. A validated record passes to each
  public commit path, which appends it and its producer/consumer indexes while holding the same lock as
  retirement and result registration. Each path keeps its producer-span arithmetic. `Remove` validates and
  publishes its zero-output record inside its existing locked liveness check. Private geometry-only builders are
  record-free.
- Tests: Root `operation_history_test.go` covers the snapshot, face coverage, private construction and `Remove`
  (`TestOperationHistorySnapshotAndFaceCoverage`, `TestPrivateConstructionKeepsBodiesAndProducerNumbersLocal`,
  `TestOperationHistoryRemovePublishesZeroOutput`). Atomic-rejection tests compare handles and scalar counters,
  never whole topology graphs.
- Acceptance: A snapshot cannot observe a record with missing outputs or partially retired inputs. Remove
  publishes no new producer identity. Failed calls leave public history and producer numbering unchanged; level
  and curve counters keep evaluator §8's separate failure behavior.

### Phase 2: Produce an immutable strict-containment certificate

- Dependencies: Phase 0; Phase 1 supplies the first path's publication seam.
- Files: `internal/clearance/pairkernel/kernel.go` and `geom.go`, `internal/verifyrun/interference.go`,
  `internal/prooflineage/`, and the source-record adapters for `internal/prismrecord/` and
  `internal/revolveplan/` that the fixture's payload classes need.
- Certificate: `pairkernel.Pair` issues `Result.Containment` where it proves strict whole containment, and only
  there. The certificate captures the eligibility, the positive boundary gap, both directions of shell membership
  and the source-set identities and placements while the proof's actual inputs are present. `Verify`'s verdict,
  contained-source reading and refusal semantics are unchanged.
- Freezing: `prooflineage` records hold plain immutable source records and tokens, with the required domains,
  orientation, bounded source Volume readings and placement charges. The committed proof lineage retains no
  `*topo.Body`, face, public handle, document, pair cache, context or mutable caller option.
- Invocation: Public Cut does not call `Pair` by entering the Boolean dispatcher.
  `massdispatch.CaptureContainedCutQuantities` makes one bounded invocation after the result builds, under the
  operation's context. Document-wide `Verify` is never called, and no proof work repeats after commit.
- Tests: Pair-kernel tests use real built analytic sources for the tilted contained cylinder, reverse argument
  order, an outer void inside the cutter, unsupported payloads, grazing or undecided casts, budget exhaustion and
  cancellation. Verify regression cases compare its bounded interference reading before and after the
  refactoring.
- Acceptance: Only the existing full-containment argument issues the certificate. An unproved or
  budget-exhausted optional attempt supplies no certificate; cancellation is an operation failure. `Verify`
  performs no modeling publication. A certificate cannot be reused for a different source or placement.

### Phase 3: Consume the certificate in the real mesh-built Cut

- Dependencies: Phases 1 and 2.
- Files: `boolean.go`, `internal/booleandispatch/`, `internal/booleaneval/eval.go`,
  `internal/facetedrecord/payload.go`, `build.go` and `assemble.go`, `internal/massdispatch/`, and the numerical
  consumer in `internal/prooflineage/`. `internal/facetedrecord/place.go` and `placed.go` transfer or drop the
  new payload fields.
- Consumer: The relation and source-quantity certificate attach to the resulting immutable faceted payload. The
  actual mesh, shells, orientation and ordinary mesh Volume build and audit first. Source subtraction runs with
  outward rounding and both source bounds. Both intervals must intersect before publication, and the strictly
  tighter reading caches on the result body. Equal bounds or absent optional proof keep the ordinary reading.
- Retained evidence: Both proven intervals and the evidence needed to check them stay. Held-mesh integration,
  `VolSymDiff`, `MeshBound`, `AreaSlack`, vertex bounds and all topology audits are preserved. A tighter scalar
  never substitutes for mesh proof, closure or source-face attribution. Exactness derives from the selected
  bound, so the cylinder's pi calculation is never labelled Exact.
- Placement: Evidence survives placement only with a proved transfer and fresh charges; otherwise the optional
  quantity certificate is discarded and the existing path recomputes. Volume alone never tightens
  `MassProperties`, area, centroid or inertia. The contained-Cut centroid consumer uses bounded source first
  moments and its own placement and arithmetic charges. `internal/massdispatch/dispatch.go` requires its own
  moment proof for `MassProperties` and inertia.
- Tests: Consumer arithmetic tests cover representable subtraction, nonzero source bounds, rounding,
  equal-bound selection, mismatched identities, invalid units or non-finite data, and disjoint intervals. A
  test-only internal control disables only certificate selection on the real producer's result. No fixture
  constructor invents admission.
- Acceptance: `Body.Volume` and `Verify` read the selected cached measurement. Contradictory stored evidence
  fails before commit; missing evidence keeps the ordinary certified result. A retired source does not
  invalidate the reading.

### Phase 4: Gate expansion on the first real end-to-end calculation

- Dependencies: Phases 1 through 3.
- Files: root `operation_history_volume_test.go`, `operation_history_test.go`, and
  `internal/booleaneval/export_test.go` or the narrowest existing test hook seam.
- Fixture: First calculation fixture runs with its real source builders, placement, strict-containment producer,
  public Cut, result Volume and `Verify`. It inspects the actual mesh payload, one material lump, void shell,
  closure and orientation audits, actual retired inputs and the public Cut record.
- Proof: The ordinary and source-derived intervals from the same immutable source records are compared: nominal
  value, source placement charges, a reference enclosure inside both intervals and a strictly smaller public
  Volume bound. `Verify`'s reading and tolerance result come through its real measurement path. No second
  consumer model or hand-written proof exists.
- Acceptance: The real producer's certificate reaches the real returned-body consumer, and the computed result
  satisfies every First calculation fixture requirement. Public face coverage on Cut can stay unknown.

### Phase 5: Record every public producer and its actual body dependencies

- Dependencies: Phase 4.
- Rule: Each public caller's entry point is recorded once, including successful calls with no new instances.
  Private geometry helpers are reused rather than publishing delegated public calls. Actual consuming/reference
  behavior and public argument order are preserved.

Route inventory by file:

- `extrude.go`, `draft.go`, `revolve.go`, `coil.go`, `stops.go`, `internal/extent/`: Body handles come from
  actual extent, axis and neutral-plane resolution. The entire ThroughAll scan is captured before bounds
  exclusions; handles are never recovered from producer roles or by rescanning after evaluation. Required
  fixtures: ToFace, ToFaceAngular, EdgeAxis and ThroughAll, checking computed extents and live reference bodies,
  including an excluded scanned body.
- `sweep.go`, `loft.go`, `loft_point.go`, `loft_sections.go`, `loft_faces.go`, `cone_from_face.go`, `patch.go`:
  Profile and chain overloads and reductions are covered. Owning bodies are captured for face sections and
  tangent ends. Each private builder commits its own entry point's kind. Required fixtures: a minimal supported
  case for each public entry and distinct publication path, checking bounded volume, area or coordinates.
- `boolean.go`, `boolean_many.go`, `internal/booleandispatch/`: Analytic, mesh, specialized and multi-operand
  routes are covered, with caller operand order and one public Union despite a private chain or producer span.
  Required fixtures: nested analytic Cut, real mesh Cut, three-input Union and supported private-chain cases,
  checking volume and retirement.
- `document.go`, `mirror.go`, `mirror_join.go`, `point_plane_scale.go`, `pattern.go`: Transform, Copy, Mirrored,
  MirroredCopy, ScaledAt, PatternCopies and Patterned are covered, with placement bodies captured. The original
  is excluded from PatternCopies outputs, and every transient is excluded from producer lookup. Required
  fixtures: minimal prisms with measured placements and volume, each output's producer, fast/fallback joins and
  numbering after the span.
- `fillet.go`, `chamfer.go`, `shell.go`, `brep_modify.go`, `draft.go`: The original public kind passes through
  every modify commit and span helper. All distinct body reads are recorded without changing fresh role policy.
  Required fixtures: small supported receivers from the existing modify fixture families, checking changed
  geometry and unchanged selector-role rules.
- `hole.go`, `rib.go`, `emboss.go`, `tool_combine.go`: Hole, WebPath and Emboss group above private tool
  construction and repeated Boolean steps. Tools and intermediate bodies stay out of public outputs and producer
  indexes. Required fixtures: one supported minimal call per entry, plus a failure after private work, checking
  volume and exactly one committed public record.
- `thicken.go`, `offset.go`, `patch_body.go`, `stitch.go`, `unstitch.go`, `surface_trim.go`: Thicken, Offset,
  PatchEdges, Stitch, Unstitch, Trim, Extend and Split are covered. All returned bodies register in one record,
  and each operation's current input use is preserved. Required fixtures: Unstitch one box, Split a minimal
  supported source, and Stitch real sheets, verifying sheet areas/free edges and solid volume as applicable.
- `document.go` Remove path: A consumed input publishes with no outputs and no producer-counter advance. The
  fixture Copies and Removes the original, reads both historical producer records, and builds the next real
  feature.

Acceptance: Every successful public body-producing call and Remove contributes exactly one record. A read-only,
failed or canceled call contributes none. Inventory tests fail when a public entry or commit route is omitted;
private helpers, temporary tools and internal producer identities are never mistaken for public operations.

### Phase 6: Complete snapshots, indexes and initial face queries

- Dependencies: Phases 1 and 5.
- Files: `operation_history.go`, `unstitch.go`, `internal/stitchrecord/`, and root `operation_history_test.go`.
- Queries: Every Public API proposal method and every Exactness and failures lookup outcome is implemented.
  Snapshot indexes are frozen, so an old snapshot gains no later consumers. Operation pointer identity is stable
  across snapshots. Every returned slice is a copy, and face results follow actual output traversal order.
- Face maps: Unstitch's map pairs each original face passed to the real sheet builder with the resulting sheet
  face. Remove has complete empty maps. Every other route returns unknown coverage, even when computational
  evidence exists. Input-face ownership and output membership validate, face links deduplicate, and one-to-many
  and many-to-one maps are allowed internally.
- Tests: Required cases are nil and unowned receivers/handles, foreign documents, retired inputs, bodies created
  after a snapshot, reference-only inputs, repeated roles, no consumers, a face outside the operation's inputs,
  mutation of returned slices, six real Unstitch sheets, Remove's empty complete results, and unknown Cut/Copy
  results.
- Acceptance: All query and snapshot semantics match Public API proposal and Exactness and failures. Empty
  unknown results never imply disappearance; inspection handles cannot bypass live-input or selector gates.

### Phase 7: Carry source patches through later real consumers

- Dependencies: Phase 4 and Phase 6. `operation_history_volume_test.go` and `operation_history_centroid_test.go`
  run First calculation fixture on the real producer. `Operation.FaceSuccessors` returns complete maps for
  Unstitch (`unstitch.go`) and Remove (`document.go`) and `HistoryUnknown` elsewhere.

The retained-plane certificate travels with the faceted payload. Each stage carries it as follows:

| Stage | Code | What it carries |
|---|---|---|
| Face-to-group assignment | `booleaneval/eval.go` `evaluate`, `eval_mesh.go` `sourceGroup` | One group per operand face; `SourceTag` copies the admitted tag, and `Source` records the face's identity. |
| Facet cutting and weld | `meshbool` `SourceIDs`, `KeptFacet.Src`, `StitchFacetsContext` | Each kept or split facet's group index. |
| Patch assembly | `facetedrecord/assemble.go` | One face per connected patch of a group, with the group's tag. |
| Edge chains | `facetedtopology` | Per-face indices only; it never reads groups. |
| Later mesh Boolean | `booleaneval/retained_plane.go`, `meshbool/plane_slab.go` | The slab bound of retention §11 for an admitted plane facet. |
| Placement | `facetedrecord/placed.go`, `place.go`, `point_plane_scale.go` | Every group, with `NormalBound` recomputed by retention §3.4. |
| Retained blend | `retainedblend/route.go` and the `*_rewrite.go` files | The copied groups, plus a new blend group for each blended edge. |

All paths are under `internal/`. `evaluate` orders groups as `topobuild.Faces(a)` then `Faces(b)`, so a group
index is the operand face's identity within one operation. Its `sourceID` map from face pointer to group index
is local and dropped on return. A shifted retry keeps `Origins` and `Source` for every face of the moved
operand, and the tag translated by the shift for a face of a translated piece
(`agent-docs/gap-tolerance-design.md` §5.2). Assembly sets `RetainedTag` on a patch when retention A4 admits it.
A canonicalization merge joins groups and keeps each member's record in `Source.Members`
(`agent-docs/union-face-merge-design.md` §4.6). `facetedtopology` never reads groups.

`retainedPlaneOf` reads the operand mesh's `Source` face. It admits a `RetainedTag` `Plane` with
`NormalBound == 0` and no denoted patch, or a denoted revolve patch whose tag is exactly its plane (retention
§11). `RefuseCoarseHeldContact` and `RimBound` then read the slab bound in place of `FacetBound`. Any departure
that placement states ends the admission. `retainedblend` copies `fp.Groups`, clearing only the whole-group
centroid of the shrunk faces. It appends one blend group with `Retained`, `NormalBound == 0` and the exact blend
surface. An R1, R3 or R5 chamfer adds a `Plane`.

A retained plane therefore reaches a later mesh Cut through mesh Booleans, exact-translation placements and
retained blends. `apitest/faceted_plane_slab_test.go` (`TestFacetedPlaneSlab`) checks the Cut-then-Cut volume and
rim-vertex bounds. Root `faceted_source_patch_test.go` (`TestFacetedSourcePatchSlabControl`) checks the same
scene against the full-displacement control of Task 7.1. Root `faceted_source_patch_blend_test.go`
(`TestFacetedSourcePatchBlendThenCut`) checks the slab after a retained blend the same way. `Group.Source` names
the earlier patch a group descends from, and `Group.Origins` holds roles only.

Phase 7 has four tasks, all implemented:

| Task | Depends on | Can run beside |
|---|---|---|
| 7.1 Slab control and Cut-then-Cut fixture | none | 7.3 |
| 7.2 Retained blend then Cut fixture | 7.1 | 7.3, 7.4 |
| 7.3 Immediate source identity | none | 7.1, 7.2, 7.4 |
| 7.4 Fallback fixtures | 7.1 | 7.2, 7.3 |

Each fixture builds bodies through the public API, then reads internal fields from the root package. It compares
two runs, each in its own document: the ordinary run, and a control run that disables only `retainedPlaneOf`.
The control refuses every second cut of the P3 scene, so a fixture compares the real run's build against the
control's refusal, and where both runs build it compares vertex lists in a loop and reports the first differing
index only. A failure never dumps a mesh or topology graph. No bound literal is pinned, because float bounds
differ between amd64 and arm64; assertions state relations between the two runs, or between the real run and the
bound the control's refusal quotes. Each fixture goes red under the red leg named in its task. New test
names go in shard 10 of `.github/test-shards.txt`.

#### Task 7.1: Slab control and Cut-then-Cut fixture

- Files: `internal/booleaneval/retained_plane.go`, `eval_mesh.go`'s `PrepBoolMesh` path, and root
  `faceted_source_patch_test.go`.
- Control: `booleaneval.WithoutRetainedPlaneSlab(ctx)` is a context option in the style of
  `meshbool.WithContactWorkers`. Under it, `PrepBoolMesh` passes a plane reader that admits no facet, so every
  facet keeps `FacetBound`. Production code never sets the option. The full-displacement control is this option
  on the real producer's result; no payload is built by hand.
- Fixture: Retention §1's P3 scene in the root package is a 40×20×20 box, four lateral edges filleted at 3 mm,
  and a Ø8 y-bore at x = 20 placed by `Transform`. A `Copy` of the first cut's result ends the Boolean chain, so
  the second cut meets the sequential gate and no regroup rebuilds it. Each second y-cut runs with and without
  the option.
  - Every second y-cut that builds in the real run is refused by the control. The control's refusal is the
    chain-depth gate: the first cut's plane facets hold a `FacetBound` coarser than the pair's chord tolerance.
    Neither x nor the radius lowers the bound, because every y-cut touches the first cut's y = 0 and y = 20
    faces. No second y-cut builds in both runs, so this scene has no vertex-position comparison.
  - Cases x = 30 r = 4, x = 33 r = 2 and x = 10 r = 1: The real run builds and the control refuses with a plain
    `ErrUnsupported` that quotes its touched bound and the chord tolerance, the bound being larger. Every new rim
    vertex lies within its own bound of the exact circle, and the largest rim bound is strictly smaller than the
    quoted touched bound. The volume matches the closed form within its bound.
- Red leg: Make `PrepBoolMesh` ignore the option. The control then builds, and the refusal assertion fails.
- Invariant: Retention §11 states that the slab changes bounds and gates only. A second cut that builds in both
  runs with any differing vertex position contradicts it.

#### Task 7.2: Retained blend then Cut fixture

- Files: root `faceted_source_patch_blend_test.go`. No production change.
- Fixture: The 40×20×20 plate of `apitest/faceted_retained_blend_test.go` is bored with a Ø8 z-bore at (20, 10)
  placed by translation. The top mouth is filleted at r = 1 (R2). Then a cut beside the mouth sends the new wall
  across top-plane facets between the blend's setback ring and the plate edge. In a second case, a straight
  retained edge is chamfered (R1) and a cut crosses the chamfer face.
- Assertions: After the blend, the top face is still a `RetainedTag` `Plane` with `NormalBound == 0`, and its
  frame is bit-equal to the bored plate's. The chain is one level deep, so the control does not reach the
  chain-depth gate of Task 7.1: the later cut builds in both runs, and the two runs place every vertex at the
  same coordinates. The slab shows as bounds. Every top-plane rim vertex bound of the real run is strictly below
  the control's bound for the same vertex, and each vertex lies within its bound of the exact circle. The volume
  removed by the later cut matches its closed form within the published bounds. A cut through the torus rim, which
  also pierces the bottom plane, builds in both runs. The torus-rim vertices carry equal bounds in both runs,
  because a curved group takes no slab, and the bottom plane's vertices are strictly tighter in the real run.
  The chamfer face is a `RetainedTag` `Plane` with `NormalBound == 0`, and its rim vertex bounds are strictly
  below the control's for the same vertices.
- Red leg: In a scratch edit, clear `Retained` on the copied groups in `retainedblend` `work.build`. The top
  plane then loses its slab, and the strictness check fails.
- Invariant: A blend output that does not reach the slab consumer is a defect in `retainedblend`.

#### Task 7.3: Immediate source identity

- Files: `internal/facetedrecord/group.go`, `internal/facetedrecord/source_patch.go`,
  `internal/provenance/source_patch.go`, `internal/booleaneval/eval_mesh.go`'s `sourceGroup`, and the blend-group
  appends in `internal/retainedblend`. `agent-docs/faceted-source-retention-design.md` §3.1's group table,
  `agent-docs/modify-design.md` Part E §5.1 and `evaluator-design.md` §9 describe the group fields.
- Record: `Group.Source *SourcePatch` (an alias of `provenance.SourcePatch`) is immutable. It holds the operand
  body's `Origin.Producer`, the operand position, the face's ordinal in `topobuild.Faces`, and `Parent`. `Parent`
  is the operand face's own group record when the operand is faceted, and nil for a built face. A blend group's
  record (`facetedrecord.BlendSource`) names the modify producer and the edge's ordinal in the call. The record
  holds no topology node, body, document, context or cache.
- Assignment: `sourceGroup` sets `Source` for every group, including a moved operand's group on the shifted
  retry, whether it keeps a translated tag or none. Placement and similarity copy the group struct
  (`placed.go`, `place.go`, `point_plane_scale.go`), so they keep the pointer. `retainedblend` keeps copied
  groups' records and gives each blend group a new one. A path that merges groups keeps every incoming record
  (`Members`).
- Admission: Identity admits nothing. `retainedPlaneOf` reads the face's tag and bound only, and no route
  derives coverage from `Source`.
- Fixtures: Root `faceted_group_source_identity_test.go` (`TestSplitPlaneSharesOneSourceRecord`,
  `TestShiftedRetryGroupsKeepSource`) splits a retained top plane into two patches with a second cut. Both
  patches carry one record that names the top face, and their areas sum to the closed-form remainder within
  bounds. Cutting that result again turns the two patches into operand faces with equal `Origins` and gives them
  two distinct records with equal `Parent`. An untouched plane two Cuts later has the first Cut's record as
  `Parent` and a bit-equal frame. A Transform keeps the pointers. The public Cut record's `FaceSuccessors`
  reports `HistoryUnknown` with no faces.
- Red leg: Key the record map by `Origins`. The equal-origins case then merges the two records.

#### Task 7.4: Fallback fixtures

- Files: root `faceted_source_patch_fallback_test.go` (`TestFacetedSourcePatchFallback`).
- Fixtures: Each case runs with and without the option of Task 7.1. The control refuses in the P3 scene, so a
  case asserts that the real run refuses with the same touched bound and chord tolerance as the control, or
  builds where the control refuses, plus the volume's closed form within its bound when it builds. The later
  cut's tool is placed by the same `Copy` option as the target.
  - Invalid placement transfer: A 20° exact-rotation `Copy` of the P3 result about the z axis through
    (20, 10, 10). First assert that the two planes the later cut touches, the y = 0 and y = 20 faces, carry a
    `RetainedTag` and a nonzero `NormalBound`. The later cut then matches the control. A translation by Y = 0.1
    leaves the y = 0 face at the exact level 0.1 and rounds the y = 20 face's level 20.1 off its plane: only the
    back face states a departure, and the later cut refuses.
  - Valid transfer: An integer translation (X = 5), and a translation by X = 0.1 whose rounding stays within the
    touched planes (`agent-docs/faceted-source-retention-design.md` §3.4), keep `NormalBound == 0` on the touched
    planes, and the later cut still builds where the control refuses.
  - Denoted plane: A stepped profile revolved 270° has end faces that are caps. A z-axis pin of radius 1.5 cuts
    through the cap in the sketch plane, then a second pin 4 mm along x. Both pins end inside the material, so
    the second cut meets no wall of the part. First assert that the cap carries a `RetainedTag` and a denoted
    patch. The later cut then matches the control.
  - Curved rim: The cut through the first bore's wall in `TestFacetedPlaneSlab` refuses identically in both
    runs.
- Red leg: Drop the `NormalBound == 0` test in `retainedPlaneOf`. The placed cases then tighten. Drop the
  `Denoted` test and the denoted case builds. Set every facet's `SlabBound` to 0 in `SetSlabBounds` and the
  curved-rim case builds.

Phase 7 acceptance: A later calculation consumes the real prior builder's evidence. Source identity alone
upgrades neither geometry nor public completeness.

Expansion gate: Complete mesh, Copy, Transform and analytic public face mapping is not implemented. It needs a
pending list from group index to operand face, returned beside `Evaluation` and never stored in the payload. It
also needs the real split-face and duplicate-origin fixtures above. `facetedtopology.UniqueGroupFace`,
`GroupFace` and `TrimGroupFace` must not serve a source that can produce several connected patches. Analytic
section maps require actual arrangement spans; compound maps require every private step.

### Phase 8: Verify rollback, concurrency and retained memory

- Dependencies: Phases 5 through 7.
- Files: history tests, `apitest/document_context_test.go`, `apitest/document_remove_test.go`, existing
  pattern/Unstitch/Verify fixtures, and a focused history allocation benchmark if needed.
- Rollback: Failure and cancellation checks run after some outputs or temporary instances have been built and
  immediately before publication. Public history, live handles and next successful provenance compare against a
  control run. Snapshots run beside Remove, Bodies and Verify under the race detector. Concurrent feature
  mutation stays outside the documented contract.
- Retention: Strong-reference retention is documented and inspected. A small repeated Copy/Remove history
  reports actual record/body/face-link counts and allocation data, with no invented memory threshold. Immutable
  source records are shared, and computational records retain no topology/document graphs or mutable buffers.
  The history has no eviction, clearing, lazy collection or snapshot-triggered collection.
- Tests: Old snapshots stay fixed after consumption; all outputs of Unstitch/Split appear together; canceled
  pattern/private-tool work publishes nothing; selectors, tessellation, export, Section, Verify/interference and
  motion queries add no records. Failure diagnostics stay bounded to identities, scalar readings, counts and
  capped differences.
- Acceptance: Race checks observe whole public commits, rollback matches the control, and retention behavior
  matches Atomic publication and storage.

### Phase 9: Update public documentation and finish required verification

- Dependencies: Every prior phase's acceptance criteria, including Phase 4's demonstrated tighter calculation.
- Files: `docs/api.md`, `docs/data-structures.md`, executable `examples/`, the owning design sections listed in
  Phase 0, affected layout part files, `.github/test-shards.txt` and `.github/test-shards-apitest.txt`.
- Documentation: Signatures, copied-slice/snapshot behavior, unconditional collection, retained-memory cost,
  input uses, coverage and lookup errors are documented, with a small executable history example and verified
  output. Replay, parameter binding, feature roles and geometric exactness contracts stay at their existing
  owners. Later moments, reconstruction and face-map expansion are marked as staged.
- Verification: Only the affected new tests and packages run locally, including targeted race cases. New
  root/apitest test names go in shard 10 under `testing.md`, then `go test . ./apitest/ -run '^TestCI'` runs for
  manifest changes, with the affected documentation/layout guards and changed executable examples. Configured
  CI runs the full suite.
- Go lint: After the last Go edit and before the first push or PR creation, run `golangci-lint run --timeout=5m`
  with CI's v2.15.0 and `go vet ./...`. Store expensive output in `.tmp/`. Fix changed-code failures and rerun
  the failed checks; stop if lint cannot run or unrelated errors block it. `~/.claude/docs/git-operations.md`
  governs later pushes and PR CI Watch.
- Acceptance: Focused checks and required lint pass, public examples match the implementation, and the PR's
  full CI has completed before the suite is claimed to pass. A feature with complete inspection but no proven
  Volume improvement does not satisfy this plan.

## Integration and behavioral reference

The API symbols and input-role names passed the repository's API approval. The execution-bookkeeping
descriptions in `api-design.md` §2 and §6, `evaluator-design.md` §8, and the no-history wording in
`param-binding-design.md` §1 point here and keep their immediate-mode and no-replay decisions. `docs/api.md`,
`docs/data-structures.md`, executable examples, layout rows and test-shard manifests describe the public API.
`evaluator-design.md` §9's mesh-measurement path and the owning payload/measurement designs state that their
proofs grant no general exactness upgrade from retained analytic surfaces.

The local OCCT checkout was read only to inspect observable per-operation lineage, result availability and
chained-result queries. The inspected files are `src/ModelingData/TKBRep/BRepTools/BRepTools_History.hxx`,
`src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderShape.hxx`,
`src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.hxx`, and
`src/ModelingAlgorithms/TKBO/GTests/BRepTools_History_Test.cxx` under `../../Open-CASCADE-SAS/OCCT` from the
repository root. No OCCT code, comments, taxonomy rules, or test fixtures are copied or adapted. Decad's
immutable bodies, atomic commits, private producers, bounded readings and existing role policies determine this
design.
