# Build steps design

`RunSteps` runs independent build steps on one `Document` at the same time. Each step behaves as if it ran alone on
the document as it stood when `RunSteps` began, and the bodies, operations and identities that come out are the same
for every worker count and every run. The public contract is in `docs/api.md` (Concurrent build steps) and
`agent-docs/api-design.md` §6.3 and §12. This file owns the rules behind it.

## What a step is

A step is a `Step{Name, Build}`. `Build` receives a ctx that carries the step (`topo.WithStep`). Every decad call a
step makes goes through that ctx, one call at a time. Commits go straight into the shared document under `LiveMu`.
There is no journal and no finished body is rewritten. When every step has finished, `topo.Document.EndRun` sorts the
live set and `Document.sortOperationsByStepLocked` sorts the operation list into step order.

## Determinism invariants

- **I1 Blocks.** With `P`, `L`, `C` the producer, level and curve counters when the run begins and `B` the block
  size, step `k` mints producers in `[P+kB, P+(k+1)B)`, levels in `(L+kB, L+(k+1)B]` and curves in
  `(C+kB, C+(k+1)B]`. After the run the counters are `P+nB`, `L+nB`, `C+nB`.
- **I2 Isolation.** Every read a step makes of shared document state depends only on the pre-run state and on the
  step's own earlier calls. Keyed reads (liveness of a body, its producing operation, its container, its circle
  witness) satisfy this because no other step touches the step's bodies or a pre-run body. Aggregate reads go
  through the step view (below).
- **I3 Serial order inside a step.** A step's calls run one after another, so its cursors move the same way every
  run. A mint made on one of decad's own worker goroutines inside a call takes its id from a private transient
  source (below), never from the step's block.
- **I4 Order on return.** The live set and the operation list are stable-sorted by step, bodies live before the run
  first. Within a step they keep commit order. Containers list their bodies in live-set order.
- **I5 Numeric order.** Block bases grow with the step index, so the numeric order of ids across the document equals
  the order of the same commits in a serial build of the steps in step order. Step 0 of a run on a fresh document
  still gets producers 0 and 1 for its first two commits, which keeps the two numeric gates that read them
  (`internal/loftcontact/quarter_held_pair.go`, `internal/facetedrecord/exact_copy_moments.go`) as in a serial build.

The results of a run equal in geometry (mesh bits) those of each step run alone on a document holding the pre-run
bodies. They equal a plain serial build only where no operation in a step reads the whole live set or the history
size differently from that build (step views below).

## Identities

- The producer counter is read through `Document.producerAt(ctx)` and `topo.Document.ProducerAt(ctx)`. Inside a step
  they return the step's cursor. `Document.advanceProducer(ctx, n)` is the only writer and returns an error:
  `ErrStepIdentities` when the step's block cannot hold `n` more or an earlier level or curve mint ran past its block
  (the flag is sticky), `ErrStepScope` for a call without a step ctx while a run is active, and `ErrStepScope` for a
  finished step. A refused commit changes nothing, because `stepGate` and `advanceProducer` run before the commit
  takes `LiveMu`.
- `topo.Document.MintLevel` and `MintCurve` take the id, in order, from the ctx's transient source, the ctx's step
  block, or the document counter when no run is active. During a run a mint with none of the three marks the run
  failed and returns the zero token, which always declines admission.
- A producer read or mint during a run without a step ctx and without a transient source marks the run failed.
  `RunSteps` returns that fault as `ErrStepScope` even when every step succeeded.
- `CounterMu` guards the three counters. Readers (`SeedsAt`, `ProducerAt` outside a step) take it for reading and the
  writers (`advanceProducer`, `MintLevel`, `MintCurve` outside a run, `EndRun`, `restore`) for writing. It is a leaf
  lock taken after `LiveMu`, so a non-step `Verify` or `IntersectionVolume` never races `EndRun`'s counter write.
  `EndRun` writes the counters inside the `LiveMu` critical section that publishes the sorted live set. A query that
  reads the live set and then its transient seeds therefore never sees the new set with the old counters. The queries
  (`Verify`, `VerifyMotion`, `VerifyLinkage`, `VerifyJointBox`) read the live set before `queryContext` for this
  reason.

### Transient identities

`Verify` and the other ctx queries build transient bodies on worker goroutines. `queryContext` puts a private atomic
id source in ctx at entry, seeded from the caller's cursors: the step's cursors inside a step (never the document's,
which block 0 shares with step 0's committed ids), otherwise the document counters. It is never written back, so the
query moves no counter. `SeedsAt` reads the transient source when ctx carries one, so a nested transient document
starts above every id the query has handed out. A transient document built by `VerifyMotion` and the query's own
source both start at the same cursor, so their ids overlap. Nothing commits a transient id.

## Document state during a run

| State | Rule |
|---|---|
| `LiveSet`, `operations`, `producers`, `consumers` | Written under `LiveMu`. Each commit records the step in `topo.Document.StepTag` and `Operation.step`. |
| `StepTag` | Maps each body committed in the run to its step. A body absent from it was live before the run. Dropped when the run ends. |
| `circleBuilds`, `circlePlacements` | Read through `circleBuildOf` and `circlePlacementOf` under `LiveMu`. |
| `resultIn` container | Travels in ctx (`resultIn`, `resultInOf`), so two steps' Extrudes never share it. |
| Container tree | `NewChild`, `ChildNamed`, `NewUniqueChild`, `AddSketch`, `AddPlane` and `AddBody` return `ErrStepScope` while a run is active. `RunSteps` creates the root container first. |
| `Param`, `Expr`, `ParamInt` | Return `ErrStepScope` while a run is active. `RunSteps` creates the parameter table first. |
| `Params().Set` | A `sketch/param` call the document cannot guard. Do not change the table while steps run. |

`stepGate(ctx, inputs, consumed)` runs in every commit path before the document changes. It returns the step, or nil
outside a run, and refuses with `ErrStepScope`: a ctx that names no step of the active run (and marks the run
failed), a finished step, an input body tagged with another step, a consumed body that was live before the run, and a
consumed body tagged with another step. `TestStepCommitPathsGate` fails when a function that publishes does not call
it.

`Remove(b).Do(ctx)` passes through the same gate, so a step may retire a body it built and nothing else.

## Step views

An aggregate read of the live set or the history makes a step's result depend on what other steps have committed so
far. Each goes through a view: the bodies and operations from before the run, plus the step's own. Outside a run the
view is the whole document. `TestLiveSetReadsGoThroughViews` fails on a call of `Live()` outside `internal/topo`.

| Read | View |
|---|---|
| ThroughAll extents (`resolveLinearExtent`, `resolveLinearSide`) | `LiveIn(ctx)` |
| `Verify`, `VerifyMotion`, `VerifyLinkage`, `VerifyJointBox` bodies | `LiveIn(ctx)` |
| `loftcontact` gates (`quarter_held_pair.go`, `quarter_held_seam.go`) | `LiveIn(ctx)` |
| Blind group cut replay budget counts | `viewSizesLocked(ctx)`; the rebuild still copies every registry |
| Rim chamfer history snapshot and its size gate | `rimHistorySnapshot`, over `viewSizesLocked` and `historyAtLocked` |
| Paired cut ancestry | `historyAt(ctx)` |
| Public `Bodies`, `OperationHistory`, `VerifyContainers`, `Container.Bodies` (no ctx) | the view of a caller outside the steps: the bodies live before the run |

A call without a step ctx therefore sees no step's work, also when it is made from inside a step's `Build`.

## Rollback inside a step

Composites (`Arrange`, `Infill`, `PathPattern`) call `snapshot(ctx)` and `restore`. Inside a step the snapshot holds
only the step's own live bodies, operations, container, witness and identity-placement entries and its producer
cursor, and `restore` undoes only those. For a caller without a step while a run is active the snapshot is inert,
because none of that caller's commits can succeed.

## Block size

`B` is `1 << 32` per kind per step. `RunSteps` refuses with `ErrDegenerate` when `P + nB` would pass the largest
`int`. A step that runs out fails at its next commit with `ErrStepIdentities` and changes nothing. Producer advances
are checked before they happen. A level or curve mint past the block sets the sticky flag before any body carrying it
can commit. `WithStepIdentities` exists for tests and for a caller that wants a smaller range.

## Misuse and where it is caught

| Misuse | Caught at | Result |
|---|---|---|
| Operation without a step ctx during a run | `stepGate`, `advanceProducer`, and the mint and read sites | `ErrStepScope`; the run is marked failed |
| Input body tagged with another step | `stepGate` | `ErrStepScope` |
| Consuming a body live before the run | `stepGate` | `ErrStepScope` |
| Container mutation, `Param`, `Expr`, `ParamInt` during a run | `requireRunIdle` | `ErrStepScope`; the run is marked failed |
| Step ctx after its run | `stepGate`, `AdvanceProducer` | `ErrStepScope` |
| Nested `RunSteps` | `BeginRun` compare-and-swap on `Run` | `ErrStepScope` |
| Block used up | `AdvanceProducer`, the sticky flag | `ErrStepIdentities` |
| Two goroutines in one step; steps sharing Go variables | the race detector | CI runs every test under `-race` |

## Failure and cancellation

`RunSteps` runs every step, also when one fails, and returns the failures joined in step order, each wrapped with the
step's name. A canceled ctx stops steps that have not started. The document holds what the steps committed, in step
order; a caller discards it as it would a failed serial build.
