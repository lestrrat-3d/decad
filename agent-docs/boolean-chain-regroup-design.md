# Boolean Chain Regroup Design

A two-body `Union`/`Cut`/`Intersect` that refuses, and that has a mesh Boolean result as an operand, is
re-evaluated from the source bodies the operation history names, under a set identity that leaves the result set
unchanged. Companion to `agent-docs/operation-history-design.md` ("history §N"), `agent-docs/evaluator-design.md`
§9, `agent-docs/faceted-vertex-bounds-design.md` §5 ("vertex-bounds §5"), `agent-docs/prism-boolean-design.md`
("prism-boolean §N") and `agent-docs/general-boolean-design.md` ("general-boolean §N"). Core §N is
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

## Problem

An operand of `Union`/`Cut`/`Intersect` that is a mesh Boolean result (`facetedrecord.Payload`) restates its held
mesh. The next pair's local gate refuses a contact on held facets coarser than the pair's chord tolerance
(vertex-bounds §5, `meshbool.RefuseCoarseHeldContact`), and the mesh path refuses coplanar or grazing contact against
the held facets. The operation history (`operation_history.go`: `Document.OperationHistory`, `Producer`,
`Operation.Inputs`) records which bodies produced the operand. No library code reads it today: outside
`operation_history.go`, `Document.producers`/`consumers`/`operations` are only initialized (`New`) and appended
(`appendOperationLocked` from the commit paths and `Remove`, `document.go`).

Success criteria:

- A two-body Boolean that refuses with an error wrapping `ErrUnsupported`, and whose chain behind a mesh operand
  can be regrouped into at most one mesh step, returns a body whose readings enclose the true quantities of the
  same set the sequential chain denotes.
- Every call that succeeds today returns the same body as today (no regroup runs).
- Every call regroup cannot rescue leaves the document unchanged and returns today's error, except two cases:
  cancellation during regroup returns `ctx.Err()`, and an invariant break inside a private step returns that
  error (Failure behavior).
- The fixtures in Test plan pass with assertions on computed volumes and coordinates.

**Out of scope:** `Union` of three or more bodies (`performUnionMany`), regrouping before a call that succeeds
today, walking through `Transform`/`Copy`/modify producers, chains mixing `Union` with `Cut`/`Intersect`, a public
option or diagnostic for regrouping, and `Verify`'s read-only evaluator.

## Value assessment

Regrouping cannot make an unsupported pair exact: a pair no analytic route admits stays a mesh step in every order.
What it buys is a chain with at most one mesh step, run last, over operands that hold no mesh. Whenever that one
mesh step remains, the result is still `Faceted`.

No missing-features row closes. Effect per row or claim:

- "Held mesh operand coarser than the pair tolerance after a Boolean chain": narrowed, not closed. A chain builds
  when its source bodies combine with every pair but one taken by an analytic route. Chains whose pairs are all
  mesh-only gain nothing: crossing cylinders (general-boolean §3 B.2), the mitred-sweep branch tree and cone chains
  of vertex-bounds §6, held primitives.
- "Contact on the mesh path other than a coplanar overlap, ...": narrowed for chained calls only. A coplanar contact
  between two source bodies that an analytic route joins (general-boolean §3 A3) no longer meets the held mesh. A
  coplanar contact that exists in the final set still refuses.
- "Multi-body Union ... a mesh Boolean result joins no such chain": not touched. `performUnionMany` keeps its
  refusal (`boolean_many.go`, "Union of three or more bodies needs ...").
- general-boolean §1 item 3 and §2 S11 (three cross holes): not needed. general-boolean §3 B.3 places S11's chain
  in class B's analytic reach.
- Bound tightening of chains that succeed today: none, by the Trigger decision.

The concrete gain is a set of ordinary refused chains: two overlapping parallel holes placed off the body's frame
(fixture F1), a straight hole that breaks into an earlier translated hole (F2), a box beside a plate that already
carries a mesh-only boss under `ExactOnly` (F3), and an analytic intersect after a mesh cut (F4).

## Assumptions

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
  producer, as in the many-body prism chain (`boolean_many.go` `tryPrismUnionChain`).

## Approach

Run today's two-body path first. When it fails with an error wrapping `ErrUnsupported` and an operand is an
expandable mesh Boolean result, root reads producer records, `internal/boolchain` flattens the chain into a normal
form of source-body literals, and `internal/booleandispatch` evaluates a plan: every pair an analytic route admits
first, then at most one full dispatch step. Any miss, failure or ineligibility returns today's error unchanged.
The public commit records the caller's call exactly as today and reserves one producer identity per private step.

Alternatives considered:

- Regroup before dispatch on every call with a mesh operand: rejected. It changes results and roles of calls that
  succeed today and costs analytic attempts on every chained call; per-vertex bounds already keep untouched
  geometry from compounding (vertex-bounds §5), so the gain on succeeding calls is small.
- Regroup before dispatch only when the plan is all analytic: rejected. It needs a mesh-only pair to vanish under
  reordering, which the fixtures surveyed never showed, and it costs analytic attempts on every chained call.
- Walk through `Transform`/`Copy` (a rigid motion distributes over Boolean operations): rejected for this
  increment. It needs placed leaves built inside the plan; nothing in the fixtures requires it.
- A public opt-in or opt-out option, or an `OpDiag` warning naming the regroup: rejected. It adds public API under
  core §6 review for behavior that only turns refusals into proven results.
- Distributive rewrites for mixed chains, e.g. `(A ∪ B) − C = (A − C) ∪ (B − C)`: rejected. They add steps and do
  not reduce the mesh-step count in any surveyed fixture.

## Design

### Trigger

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

### Eligibility of an expandable body

`boolchain` expands body `n` only when all hold; any failure makes `n` a leaf:

- `n.Payload` is a `facetedrecord.Payload` value.
- The `Source` reports a public producer of kind `Cut`, `Union` or `Intersect` with `n` as its output.
- `n.Payload.(facetedrecord.Payload).PointCone == nil`. A point-cone trim may record `MatchingFaces`
  (`internal/facetedrecord/point.go`), which history does not record.
- No `n.OperationWarnings` record has `Change != opdiagnostic.ChangeUnknown`. A gap shift or embedding reach moved
  accepted geometry (`internal/booleaneval/eval.go` `Diagnostics`, `internal/meshbool/held_embedding_diagnostics.go`),
  so `n` does not denote the operation over its inputs. `CodeChainAllowanceApplied` has no change mechanism and does
  not block.

A body whose producer is `Transform`, `Copy`, a modify operation, `Hole`/`Rib`/`Web`/`Emboss`/`Boss`, `Patterned`
or any non-Boolean kind is a leaf. An analytic (non-faceted) Boolean result is a leaf: it is already exact.

### Chain normal form

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

### Admitted identities

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

No mixed identity is admitted: `(A ∪ B) − C ≠ A ∪ (B − C)` in general, so a `Union` result inside a conjunction's
positive side, or a `Cut`/`Intersect` result inside a disjunction or a negative literal, stays a leaf.

### Plan

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
5. Otherwise → `ok == false`.
6. Merge (shared with disjunction): over `items` in order, find the first pair `i < j` with
   `Exact(OpUnion, items[i], items[j])` ok; replace `items[i]` with the result, delete `items[j]`, restart. Stop
   when no pair merges.

Disjunction: Merge all items. One item → return it. Two items → return `Full(OpUnion, items[0], items[1])`.
Otherwise → `ok == false`.

Rules:

- Keep a miss set keyed by `(op, x, y)`; never call `Exact` twice on one key. Bodies are immutable and routes are
  deterministic (Assumptions), so a cached miss stays valid.
- `Full` runs at most once per `Run`.
- Check `ctx.Err()` before every callback.
- With `MaxLiterals = 6`, `Exact` runs at most 90 times: each Absorb pass tries each pending literal once per `acc`
  version (at most 30 per Absorb, two Absorbs), and Merge tries at most 10 initial pairs plus 16 pairs with merged
  items.

### Execution in booleandispatch

`internal/booleandispatch/dispatch.go` splits `Body` without changing its behavior:

```go
// Analytic runs every route Body tries before the mesh pipeline, in Body's order.
// ok=false with a nil error is a miss. A non-nil error is the route's refusal, already
// mapped through hooks.AsBooleanError where Body maps it today.
func Analytic(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body,
    hooks Hooks) (*topo.Body, bool, error)
```

`Body` becomes `Analytic`, then the unchanged mesh tail (from `booleaneval.Evaluate` through the final
`ctx.Err()` check) when `Analytic` misses. `refuseSheetOperand` stays at its current position inside `Analytic`.

New file `internal/booleandispatch/regroup.go`:

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

- `envAt(k)` is `env` with `Ref = env.Ref + k`, where `k` counts successful steps so far. A missed attempt does
  not advance `k`.
- A cancellation is `ctx.Err() != nil` after the callback returns. Return `ctx.Err()`.
- A step error with `surface(err) == true` is an invariant break. Return it unchanged; regroup stops. Root's
  `surface` reports true for a `*BooleanError` with `Code == BooleanEvaluatorFailure`, and for an error wrapping
  `ErrBooleanFailed` that is not a `*BooleanError` with `Code == BooleanEmpty`. Root owns the predicate because
  `BooleanError` is a root type.
- "Restates a held mesh" is `booloperand.HeldFloor(result)` reporting `true`: a faceted, mitred-sweep, coil or
  twisted-sweep payload. Such an `Exact` result is a miss, so `acc` never holds a mesh and the final step's
  operands come from analytic steps or leaves. A route ahead of the mesh tail that returns a held mesh is still
  reachable through `Full`.
- The returned span is the final `k`. The result was built under `env.Ref + k − 1`.
- `Regroup` never writes the document. `Analytic` and `Body` write nothing today (`Body`'s doc comment); keep it so.

Soundness under CLAUDE.md "A decad-side check may only FALSIFY": every check here is a routing decision that can
only send the call back to today's error or surface an invariant break. Each step's result is admitted by its own route's proofs, exactly as a
public call would be. The set identity is a theorem over the recorded inputs, not a geometric test. No residual,
sampled distance or bound comparison admits anything.

### Root wrapper

New root file `boolean_regroup.go`. It stays in root because it reads `Document.producers`, which only root holds.

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
- Disjunction chains return `ok == false` until Task 5 enables them.
- `booleanHooks(cfg booleanConfig) booleandispatch.Hooks` is extracted from `booleanBodyWith` (`boolean.go`) by
  Task 3; `booleanBodyWith` and `regroupBoolean` both call it.

`performBoolean` change:

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

`containedCutQuantities` runs on the rescued body unchanged: its relation reads the public `a` and `b`, and the
rescued body denotes `a − b` (Admitted identities).

### Source reachability and retention

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

### Result identity and history

- Set: the result denotes the same regular set as the caller's call over its public operands (Admitted
  identities). Its readings are proven against that set by the routes that built it, so a test checks them against
  the closed-form value of that set.
- Representation: the result differs from any sequential result. It has at most one mesh step; `Faceted` whenever
  that step ran the mesh pipeline.
- Public history: one record, exactly as today: kind `Cut`/`Union`/`Intersect`, the caller's `a` and `b` consumed
  with today's roles (`operationInputsForBoolean`), and the result as the only output. No record names a leaf or a
  private step. The chain is recoverable from `Producer(a)`/`Producer(b)`. `FaceSuccessors` stays
  `HistoryUnknown`, as for every Boolean route.
- Producer identities: the commit reserves the span `Regroup` returns (`commitOperationSpanIn`), as
  `tryPrismUnionChain` does. `Body.Origin` names the final step's producer.
- Roles: faces a private analytic step mints carry fresh roles under that step's private producer
  (prism-boolean §11), which no caller can name. The final mesh step keeps its operands' face origins
  (evaluator §9). A selector naming a source feature's origin can therefore match fewer faces than it would on a
  sequential result. Select by geometric predicate instead, as prism-boolean §11 already advises.
- Diagnostics: the result carries no `OperationWarnings` (Execution in booleandispatch rejects any).
- Recognizers in the mesh tail (`obliqueboss.Recognize`, `facetedrecord.RecognizePrismUnionFillet`,
  `crossbore.RecognizeBore`, `loftfacestrim.RecordConeToe`) read the final step's operands. Their records are claims
  about the body built from those operands, so they hold.

### Failure behavior

| Condition | Result |
|---|---|
| Trigger not met | Today's `err`, document unchanged. |
| `Flatten` declines (no expandable operand, ineligible body, mixed chain, more than 6 literals) | Today's `err`. |
| Plan leaves two or more pending literals | Today's `err`. |
| A step fails with an error `surface` does not report, or a step result carries warnings | Today's `err`. |
| A step fails with an error `surface` reports (invariant break) | That error; no commit. |
| Context canceled during regroup | `ctx.Err()`; no commit. |
| Rescued body built | Commit as Root wrapper states. |

Every non-commit path leaves the live set, producer counter and history unchanged (evaluator §8). Level and curve
counters may advance, as for any failed build.

### Public behavior

No exported symbol changes. Callers observe: a refused two-body Boolean may now return a body; that body's selector
roles follow Result identity and history. Task 4 updates core §8 "The chain depth", evaluator §9's gate bullet,
`docs/api.md`, the `Union`/`Cut`/`Intersect` doc comments (`boolean.go`) and the two missing-features rows.

## Performance

- Calls that succeed today pay nothing.
- A refused call with an expandable operand pays one history map read under `RLock` per examined body (the walk
  stops past `MaxLiterals` literals), at most 90 `Exact` attempts (Plan), and one `Full` step.
- An `Exact` miss on a structural gate costs a payload type switch; a miss after a private `sketch` scene costs
  that scene.
- No timing threshold is set. Repository rule: measure on CI, not locally.

## Test plan

Every fixture asserts computed geometry. Use `require`, `t.Context()`, and bounded diagnostics (scalars, counts,
handles); never dump topology. Helpers come from `apitest`: `retentionScene` (`faceted_source_retention_test.go`),
`slabYTool` (`faceted_plane_slab_test.go`), `boxBody`/`boxBodyAtZ` (`clearance_test.go`), `translated`,
`volumeMM`, `requireBodyWatertight` (`boolean_test.go`), `tookMeshPath` (`prism_boolean_test.go`).

### Fixtures

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
- `tookMeshPath(result)` is true (the one mesh step is honest). Task 1 pins this premise on the hand-built final step.
- History: the last record has the caller's kind, the two public operands as consumed inputs with the roles
  `operationInputsForBoolean` gives (`target`/`tool` for `Cut`, `a`/`b` for `Intersect`, `first`/`others` for
  two-body `Union`), and outputs `[result]`. For each leaf behind an expanded operand (`box` and `t1` in F1, F2, F4;
  `plate` and `boss` in F3), `Consumers(leaf)` returns only the earlier Boolean that consumed it. `doc.Bodies()`
  contains `result` and neither public operand.

### Decline fixtures

| ID | Setup | Why regroup declines | Assert |
|---|---|---|---|
| D1 | F1, but `Cut(Copy(first), t2)` | `Copy` producer: leaf, no expansion | `ErrUnsupported`, one diagnostic `OpDiagOperandBoundExceeded`, history length and `doc.Bodies()` unchanged |
| D2 | F1's `first`, then `Union(first, t2)` | `Cut` result inside a disjunction: leaf | today's error unchanged |
| D3 | F1 with `Cut(first, t2, decad.WithMatchingFaces())` | Trigger excludes the option | same as D1 |
| D4 | F1 with `t2` rotated 0.01 rad about the x axis through (22, 10, 10) | `Union(t1, t2')` misses (prism-boolean §3.1 G3: the sweep normals disagree), two literals pending | same as D1 |

### Unit tests (`internal/boolchain`)

- `Flatten`: each expansion rule row; role-based `Cut` input reading; leaf on non-faceted payload, non-Boolean
  producer, `PointCone != nil`, gap-shift and embedding-reach warnings; expansion through a chain-allowance warning;
  a 7-literal chain declines; no expansion declines; mixed chains decline.
- `Run`: Absorb order and restart; Merge first-pair order and restart; one `Full` call at most; miss cache (no key
  repeated); two pending mixed literals decline; three disjunction items decline; a callback error returns unchanged;
  a pre-canceled context returns `ctx.Err()` before any callback. Fakes record calls; bodies are bare `topo.Body`
  values.

### Existing tests this change flips

`TestChainToleranceAdmitsAContactOnACoarserRim` (`apitest/chain_tolerance_test.go`) and the "a contact on a curved
rim still refuses" subtest of `TestFacetedPlaneSlab` (`apitest/faceted_plane_slab_test.go`) assert F1's sequential
refusal. Rule: insert D1's `Copy` so they keep testing the sequential gate, and read `held` from the copy. Any other
test whose refusal regroup rescues: if the test's purpose is the gate or contact refusal itself, insert the `Copy`;
otherwise assert the rescued geometry against a closed form. The same rule covers the sequential-premise subtests of
`TestRegroupPlansBuildByHand`: Task 4 inserts the `Copy` before the final sequential call of F1, F2 and F4, and
Task 5 does so for F3.

## Open questions

- Decide whether a regrouped result should carry a public warning (e.g. `OpDiagChainRegrouped`) because selector
  roles differ from a sequential result. Adding it costs a core §6/operation-diagnostics review, `docs/api.md` and
  `docs/data-structures.md` edits. Leaving it out keeps the public surface unchanged; the default here is to leave it
  out.

## Tasks

Run Task 1 first; its stop condition gates everything. Task 1 is a commit, not a PR: the repository ships no
design-only PR, so Task 1 and Task 3 form the first PR on this branch. Task 2 can run in parallel with Task 3 on its
own branch; its PR merges after the Task 1+3 PR. Tasks 4 and 5 are one PR each, in order, after Tasks 2 and 3.
Before the first push of any task with Go changes, run `golangci-lint run --timeout=5m` at CI's version and
`go vet ./...`, per `~/.claude/docs/git-operations.md` "Go Lint Before PR". Every task that edits
`agent-docs/layout/` runs `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'`; keep each layout cell at or under
300 characters by rewording, never by adding detail. The branch was cut at `b2b42d30b`; rebase onto `main` before
the first push (expect conflicts only in `agent-docs/missing-features.md` and `.github/test-shards-apitest.txt`).

### Task 1: Design doc and the hand-built plans

- **Files**: `agent-docs/boolean-chain-regroup-design.md` (this doc), `agent-docs/operation-history-design.md`
  (no-replay and retention edits, already on the branch), `agent-docs/layout/design-docs-gaps.md` (row, already on
  the branch), new `apitest/boolean_regroup_plan_test.go`, `.github/test-shards-apitest.txt`.
- **What**: Through public calls only, run each F fixture's sequential chain and its regrouped plan by hand. Pin the
  premises the design depends on.
- **Depends on**: none.
- **Pattern refs**: `apitest/faceted_plane_slab_test.go`, `apitest/boolean_test.go`
  `TestBooleanChainDepthGateReadsOnlyTheTouchedRims`.
- **Tests**: `TestRegroupPlansBuildByHand` with one subtest per F fixture asserting: each sequential chain's final call
  fails with `errors.Is(err, decad.ErrUnsupported)`; each plan step marked analytic in Fixtures has
  `tookMeshPath == false`; each plan's final `Full` step has `tookMeshPath == true`; each plan's final body meets the
  F assertions on volume, bound and watertightness. One subtest per D1 and D4 asserting the sequential refusal
  persists through the `Copy` and the tilted `t2'`, and that `Union(t1, t2')` takes the mesh path. Add the test name
  to shard 10.
- **Done when**: `go test ./apitest/ -run '^TestRegroupPlansBuildByHand$'` passes,
  `go test . ./apitest/ -run '^TestCI'` passes, and `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` passes.
- **Stop condition**: if any premise fails (a sequential chain builds, an analytic step takes the mesh path, a final
  step is admitted by an analytic route, a plan's final call refuses, or a volume misses its reference), stop. Report
  the fixture, the failing call, the error or readings, and do not change dimensions or the design.

### Task 2: `internal/boolchain` normal form and plan

- **Files**: new `internal/boolchain/chain.go`, `internal/boolchain/plan.go`, `internal/boolchain/chain_test.go`,
  `internal/boolchain/plan_test.go`; `agent-docs/layout/operation-observability.md` (row).
- **What**: `Literal`, `Chain`, `Source`, `Flatten`, `Step`, `Steps`, `Run`, `MaxLiterals` per Chain normal form,
  Eligibility of an expandable body and Plan. No caller yet.
- **Depends on**: Task 1.
- **Interface**:
  ```go
  const MaxLiterals = 6
  type Literal struct { Body *topo.Body; Negated bool }
  type Chain struct { Disjunction bool; Literals []Literal } // conjunction: Literals[0] positive
  type Source interface {
      BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool)
  }
  func Flatten(op meshbool.OperationKind, a, b *topo.Body, src Source) (Chain, bool)
  func Run(ctx context.Context, chain Chain, steps Steps) (*topo.Body, bool, error)
  ```
  Package doc names this design. Imports limited to `topo`, `meshbool`, `facetedrecord`, `opdiagnostic`.
- **Tests**: Unit tests (`internal/boolchain`) above.
- **Done when**: `go test ./internal/boolchain/` passes, `golangci-lint run ./internal/boolchain/...` is clean, and
  `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` passes.

### Task 3: Split `booleandispatch.Body` into `Analytic` plus the mesh tail

- **Files**: `internal/booleandispatch/dispatch.go`, `boolean.go` (extract `booleanHooks`), new root
  `boolean_dispatch_split_test.go`, `.github/test-shards.txt`, `agent-docs/layout/internal-checks.md`
  (`internal/booleandispatch/` row: add "`Analytic` route prefix").
- **What**: Move every route before `booleaneval.Evaluate` into `Analytic` with identical order, error mapping and
  sheet refusal. `Body` calls `Analytic`, then the unchanged mesh tail on a miss. Extract
  `booleanHooks(cfg booleanConfig) booleandispatch.Hooks` from `booleanBodyWith` unchanged. No behavior change.
- **Depends on**: Task 1.
- **Tests**: `TestBooleanDispatchAnalyticPrefix` (root package, hooks from `booleanHooks(booleanConfig{})`, context
  from `d.evalContext`): boxes x 0..10 and x 5..15 (y 0..10, z 0..10) on one XY frame → `Analytic(OpUnion)` ok with a
  `prismPayload` of volume 1500 mm³; F1's `box` and `t1`, rebuilt in the test with `Extrude`, `Fillet` and
  `Transform` as `apitest/faceted_source_retention_test.go` builds them (root tests cannot use `apitest` helpers) →
  `Analytic(OpCut)` misses with nil error and `Body` returns a `facetedPayload` whose volume is within its bound plus
  1e-6 of `Vbox − 320π`. Add the name to shard 10.
- **Done when**: `go test . -run '^TestBooleanDispatchAnalyticPrefix$'` passes, `go vet ./internal/booleandispatch/`
  is clean, `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` passes, and the PR's CI is green.

### Task 4: Conjunction regroup end to end

- **Files**: `boolean.go` (`performBoolean`, `booleanHooks`, `Union`/`Cut`/`Intersect` doc comments), new
  `boolean_regroup.go`, new `internal/booleandispatch/regroup.go`, new `apitest/boolean_regroup_test.go`,
  `apitest/boolean_regroup_plan_test.go` (insert the `Copy` before the final sequential call of F1, F2 and F4, per
  Existing tests this change flips), `apitest/chain_tolerance_test.go`, `apitest/faceted_plane_slab_test.go`,
  `.github/test-shards-apitest.txt`, `agent-docs/api-design.md` (§8 "The chain depth"), `agent-docs/evaluator-design.md`
  (§9 gate bullet, plus one rule: a route that moves accepted geometry MUST record an `opdiagnostic.Record` with a
  `Change` mechanism, because Eligibility of an expandable body relies on it),
  `agent-docs/missing-features.md` (the two narrowed Booleans rows), `agent-docs/layout/booleans.md`
  (`boolean_regroup.go` row), `agent-docs/layout/internal-checks.md` (`internal/booleandispatch/` row: add the chain
  regroup executor), `docs/api.md` (Booleans note), plus any `apitest/*_test.go` or root `*_test.go` file whose
  refusal assertion CI shows flipped, updated per Existing tests this change flips and listed in the PR body.
- **What**: Wire Trigger, Root wrapper and Execution in booleandispatch for conjunction chains (`Cut`, `Intersect`).
  `regroupBoolean` returns `ok == false` for a disjunction.
- **Depends on**: Tasks 2 and 3.
- **Pattern refs**: `boolean_many.go` `tryPrismUnionChain` (private steps, producer span, final commit).
- **Tests**: `TestBooleanRegroupConjunction` with F1, F2, F4 and D1, D3, D4 per Test plan; the flipped tests per
  Existing tests this change flips. A root test `TestRegroupSurfacesInvariantBreaks` checks `regroupSurfaces` on a
  `BooleanEvaluatorFailure`, a `BooleanEmpty`, a `BooleanUnsupportedContact` and a bare `ErrBooleanFailed` (true,
  false, false, true). Add names to shard 10.
- **Done when**: `go test ./apitest/ -run '^(TestBooleanRegroupConjunction|TestChainToleranceAdmitsAContactOnACoarserRim|TestFacetedPlaneSlab|TestRegroupPlansBuildByHand)$'`
  passes, `go test . ./apitest/ -run '^TestCI'` passes, `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` passes,
  and the PR's CI is green.

### Task 5: Disjunction regroup

- **Files**: `boolean_regroup.go` (remove the disjunction gate), `apitest/boolean_regroup_test.go`,
  `apitest/boolean_regroup_plan_test.go` (insert the `Copy` before F3's final sequential call),
  `.github/test-shards-apitest.txt`, `agent-docs/api-design.md` (§8 "The chain depth": name `Union`),
  `docs/api.md`, `agent-docs/missing-features.md` (contact row wording, if Task 4's names only `Cut`/`Intersect`),
  plus any `apitest/*_test.go` or root `*_test.go` file whose refusal assertion CI shows flipped, updated per Existing
  tests this change flips and listed in the PR body.
- **What**: Enable `Union` chains per Plan's disjunction.
- **Depends on**: Task 4.
- **Tests**: `TestBooleanRegroupDisjunction` with F3 and D2. Before writing D2's assertions, confirm the sequential
  `Union(first, t2)` refuses; if it builds, drop the D2 e2e case, keep the `Flatten` mixed-chain unit test, and say
  so in the PR body. Add names to shard 10.
- **Done when**: `go test ./apitest/ -run '^(TestBooleanRegroupDisjunction|TestRegroupPlansBuildByHand)$'` passes,
  `go test . ./apitest/ -run '^TestCI'` passes, and the PR's CI is green.
