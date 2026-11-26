package decad

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"runtime"
	"runtime/debug"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-go/option/v3"
)

// defaultStepIdentities is the size of the block of identities each kind (producer, level, curve) of each step
// draws from. Measured gallery steps use a few thousand; the block is far larger than any step can need.
const defaultStepIdentities = 1 << 32

// Step is one independent part of a model build. [RunSteps] may run it beside the other steps of the same call.
type Step struct {
	// Name labels the step in errors. It need not be unique.
	Name string
	// Build makes the step's bodies. It MUST make every decad call on the document through ctx, one call at a
	// time, and MUST NOT share mutable state with another step's Build.
	Build func(ctx context.Context) error
}

// RunStepsOption configures [RunSteps]. [WithStepWorkers] and [WithStepIdentities] are its options. An option from
// another package cannot implement this sealed tier.
type RunStepsOption interface {
	option.Interface
	runStepsOption()
}

type stepsOption struct{ option.Interface }

func (stepsOption) runStepsOption() {}

type identStepWorkers struct{}
type identStepIdentities struct{}

// WithStepWorkers runs at most n steps at a time. The default is runtime.GOMAXPROCS(0). An n below 1 is
// ErrDegenerate when RunSteps evaluates the option. The worker count never changes a result.
func WithStepWorkers(n int) RunStepsOption {
	return stepsOption{option.New(identStepWorkers{}, n)}
}

// WithStepIdentities gives every step a block of n identities of each kind (producer, level, curve). The default
// is 1 << 32. An n below 1, or a block total past the identity range, is ErrDegenerate when RunSteps evaluates
// the option.
func WithStepIdentities(n int) RunStepsOption {
	return stepsOption{option.New(identStepIdentities{}, n)}
}

type stepsConfig struct {
	workers  int
	identity int
}

func stepsConfigOf(opts []RunStepsOption) (stepsConfig, error) {
	cfg := stepsConfig{workers: runtime.GOMAXPROCS(0), identity: defaultStepIdentities}
	for _, opt := range opts {
		o, ok := opt.(stepsOption)
		if !ok || o.Interface == nil {
			return cfg, fmt.Errorf(`%w: a nil or zero RunSteps option names nothing to apply`, ErrDegenerate)
		}
		n, _ := option.Get[int](o.Interface)
		switch o.Ident().(type) {
		case identStepWorkers:
			if n < 1 {
				return cfg, fmt.Errorf(`%w: the step worker count must be at least 1, got %d`, ErrDegenerate, n)
			}
			cfg.workers = n
		case identStepIdentities:
			if n < 1 {
				return cfg, fmt.Errorf(`%w: the step identity block must be at least 1, got %d`, ErrDegenerate, n)
			}
			cfg.identity = n
		}
	}
	return cfg, nil
}

// RunSteps runs steps on doc and returns when every step has finished.
//
// Each step runs as if alone on doc as it stood when RunSteps began: it sees the bodies live then and the bodies
// it built itself, never another step's. A step may run every operation and every query that takes a ctx (Verify,
// VerifyMotion, ContactPair and the like) on those bodies, but it may only consume (retire) the bodies it built.
// Breaking a rule is [ErrStepScope].
//
// Step k draws its producer, level and curve identities from block k, so results do not depend on the worker
// count or on timing. When RunSteps returns, Bodies(), OperationHistory() and every Container list the bodies
// live before RunSteps, then step 0's in commit order, then step 1's, and so on.
//
// While steps run, a call without a step's ctx reads doc as it stood when RunSteps began: Bodies,
// OperationHistory, VerifyContainers, Container.Bodies and a Verify from outside the steps see no step's work. An
// operation on doc without a step's ctx is refused with ErrStepScope and makes RunSteps fail too. Changing the
// parameter table (Params().Set) while steps run is a data race the document cannot see: do not.
//
// RunSteps runs every step, also when one fails, unless ctx is canceled. It returns the failures joined in step
// order (errors.Join), each wrapped with its step's Name. After a failure the document holds what the steps
// committed, in step order; discard it as a failed serial build would be discarded.
//
// ctx and doc MUST NOT be nil (ErrDegenerate), and neither may a step's Build. An empty steps runs nothing and
// returns nil.
func RunSteps(ctx context.Context, doc *Document, steps []Step, opts ...RunStepsOption) error {
	if ctx == nil {
		return fmt.Errorf(`%w: a nil context cannot control RunSteps`, ErrDegenerate)
	}
	if doc == nil {
		return fmt.Errorf(`%w: a nil document owns no steps to run`, ErrDegenerate)
	}
	cfg, err := stepsConfigOf(opts)
	if err != nil {
		return err
	}
	for i, s := range steps {
		if s.Build == nil {
			return fmt.Errorf(`%w: step %s has no Build`, ErrDegenerate, stepLabel(i, s))
		}
	}
	if len(steps) == 0 {
		return nil
	}
	// Create the lazily built state a step would otherwise create concurrently.
	doc.Params()
	doc.Root()
	run, err := doc.n.BeginRun(len(steps), cfg.identity)
	if err != nil {
		return err
	}
	errs := make([]error, len(steps))
	jobs := make(chan int)
	var wg sync.WaitGroup
	for range min(cfg.workers, len(steps)) {
		wg.Go(func() {
			for i := range jobs {
				errs[i] = runStep(ctx, run.Step(i), steps[i])
			}
		})
	}
	for _, i := range stepDispatchOrder(len(steps)) {
		jobs <- i
	}
	close(jobs)
	wg.Wait()
	doc.n.EndRun(run, doc.settleOperationsLocked)
	var failed []error
	for i, e := range errs {
		if e != nil {
			failed = append(failed, fmt.Errorf("decad: step %s: %w", stepLabel(i, steps[i]), e))
		}
	}
	if fault := run.Fault(); fault != "" {
		failed = append(failed, fmt.Errorf("%w: %s", ErrStepScope, fault))
	}
	return errors.Join(failed...)
}

// stepDispatchOrder returns the order in which RunSteps hands the n steps to its workers. The order never changes a
// result; tests replace it to prove that.
var stepDispatchOrder = func(n int) []int {
	order := make([]int, n)
	for i := range order {
		order[i] = i
	}
	return order
}

func stepLabel(i int, s Step) string {
	if s.Name == "" {
		return fmt.Sprintf("#%d", i)
	}
	return fmt.Sprintf("%q", s.Name)
}

// runStep runs one step's Build on a goroutine of its own, marks the step finished and returns Build's error. A
// panic in Build becomes an error, and so does a Build that ends its goroutine with runtime.Goexit (for example
// through t.FailNow), so the worker that waits here always returns and the dispatcher never blocks. A canceled ctx
// stops a step that has not started.
func runStep(ctx context.Context, s *topo.Step, step Step) error {
	done := make(chan error, 1)
	go func() {
		var err error
		returned := false
		defer func() {
			if r := recover(); r != nil {
				err = fmt.Errorf("decad: the build panicked: %v\n%s", r, debug.Stack())
			} else if !returned {
				err = errors.New("decad: the build ended its goroutine without returning (runtime.Goexit)")
			}
			s.Finish()
			done <- err
		}()
		if err = ctx.Err(); err != nil {
			returned = true
			return
		}
		err = step.Build(topo.WithStep(ctx, s))
		returned = true
	}()
	return <-done
}

// settleOperationsLocked ends a run's bookkeeping on the operation list: it puts the list in step order
// (operations committed before the run first, then step 0's in commit order, and so on) and then marks every
// operation as committed outside a step, so the next run sees them all as pre-run. The caller holds LiveMu.
func (d *Document) settleOperationsLocked() {
	slices.SortStableFunc(d.operations, func(a, b *Operation) int { return a.step - b.step })
	for _, op := range d.operations {
		op.step = -1
	}
}

func stepIndex(s *topo.Step) int {
	if s == nil {
		return -1
	}
	return s.Index()
}

// failRun marks run failed with reason and returns the ErrStepScope refusal for it, so a caller that swallows the
// refusal still gets an error from RunSteps.
func failRun(run *topo.Run, reason string) error {
	run.Fail(reason)
	return errStepScope("%s", reason)
}

func errStepScope(format string, args ...any) error {
	return fmt.Errorf("%w: "+format, append([]any{ErrStepScope}, args...)...)
}

// stepGate checks, before a commit changes d, that the call is inside the rules of the run on d and returns the
// step it belongs to, or nil outside a run. inputs are every body the operation reads and consumed the bodies it
// retires. It takes LiveMu for reading, so the caller MUST NOT hold it.
//
// A call without its step's ctx while a run is active, or with a step ctx used after its step finished, is
// ErrStepScope. A body another step built, as input of any kind, is ErrStepScope; so is a consumed body that was
// live before the run. Every one of these refusals also marks the run failed, so RunSteps returns ErrStepScope
// even when the step swallows the refusal.
func (d *Document) stepGate(ctx context.Context, inputs []OperationInput, consumed []*Body) (*topo.Step, error) {
	run := d.n.ActiveRun()
	s := d.n.StepOf(ctx)
	if run == nil {
		if s != nil {
			return nil, errStepScope("a step's context was used after its run ended")
		}
		return nil, nil //nolint:nilnil // a nil step is the outside-a-run case, which no caller treats as an error
	}
	if s != nil && s.Run() != run {
		run.Fail("a step's context from a previous run was used")
		return nil, errStepScope("an operation changed the document with a step's context from a previous run")
	}
	if s == nil {
		run.Fail("an operation changed the document without its step's context")
		return nil, errStepScope("an operation changed the document while steps run, without its step's context")
	}
	if s.Done() {
		return nil, failRun(run, "a step's context was used after its step finished")
	}
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	for _, in := range inputs {
		if in.Body == nil {
			continue
		}
		if i, ok := d.n.StepTag[bodyNode(in.Body)]; ok && i != s.Index() {
			return nil, failRun(run, "an input body was built by another step")
		}
	}
	for _, b := range consumed {
		if b == nil {
			continue
		}
		i, ok := d.n.StepTag[bodyNode(b)]
		if !ok {
			return nil, failRun(run, "a step consumed a body that was live before the steps began")
		}
		if i != s.Index() {
			return nil, failRun(run, "a consumed body was built by another step")
		}
	}
	return s, nil
}

// requireRunIdle refuses with ErrStepScope a call that cannot name its step while a run is active on d. It
// serves Container mutations, Param, Expr and ParamInt.
func (d *Document) requireRunIdle(what string) error {
	r := d.n.ActiveRun()
	if r == nil {
		return nil
	}
	r.Fail(what + " was called while steps run")
	return errStepScope("%s cannot name its step, so it is refused while steps run", what)
}

// historyAt is the operation history as ctx sees it: the whole history outside a run, and during a run the
// operations committed before it, plus the calling step's own inside a step.
func (d *Document) historyAt(ctx context.Context) *OperationHistory {
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	return d.historyAtLocked(ctx)
}

// historyOutside is historyAt for a caller outside every step, such as the public OperationHistory, which takes
// no ctx: during a run it is the history from before the run.
func (d *Document) historyOutside() *OperationHistory {
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	return d.historyForLocked(nil)
}

// historyAtLocked is historyAt for a caller that holds LiveMu.
func (d *Document) historyAtLocked(ctx context.Context) *OperationHistory {
	return d.historyForLocked(d.n.StepOf(ctx))
}

func (d *Document) historyForLocked(step *topo.Step) *OperationHistory {
	h := &OperationHistory{doc: &d.n}
	if d.n.ActiveRun() == nil {
		h.operations = slices.Clone(d.operations)
		h.producers = maps.Clone(d.producers)
		h.consumers = make(map[*Body][]*Operation, len(d.consumers))
		for body, operations := range d.consumers {
			h.consumers[body] = slices.Clone(operations)
		}
		return h
	}
	visible := d.operationFilterLocked(step)
	h.operations = make([]*Operation, 0, len(d.operations))
	for _, op := range d.operations {
		if visible(op) {
			h.operations = append(h.operations, op)
		}
	}
	h.producers = make(map[*Body]*Operation, len(d.producers))
	for body, op := range d.producers {
		if visible(op) {
			h.producers[body] = op
		}
	}
	h.consumers = make(map[*Body][]*Operation, len(d.consumers))
	for body, operations := range d.consumers {
		var kept []*Operation
		for _, op := range operations {
			if visible(op) {
				kept = append(kept, op)
			}
		}
		if len(kept) > 0 {
			h.consumers[body] = kept
		}
	}
	return h
}

// operationFilterLocked returns the predicate that tells whether an operation is part of a step's view of the
// history during a run; a nil step is the view of a caller outside every step. The caller holds LiveMu.
func (d *Document) operationFilterLocked(s *topo.Step) func(*Operation) bool {
	run := d.n.ActiveRun()
	own := -1
	if s != nil && s.Run() == run {
		own = s.Index()
	}
	return func(op *Operation) bool {
		return run == nil || op.step < 0 || op.step == own
	}
}

// viewSizes are the sizes of the registries as ctx sees them.
type viewSizes struct{ live, operations, producers, consumers, groups int }

// viewSizesLocked counts the live set, the history indexes and the container index as ctx sees them (the whole
// document outside a run). The caller holds LiveMu.
func (d *Document) viewSizesLocked(ctx context.Context) viewSizes {
	if d.n.ActiveRun() == nil {
		return viewSizes{len(d.n.LiveSet), len(d.operations), len(d.producers), len(d.consumers), len(d.groups.ofBody)}
	}
	visible := d.operationFilterLocked(d.n.StepOf(ctx))
	var v viewSizes
	for _, b := range d.n.LiveSet {
		if d.n.VisibleLocked(ctx, b) {
			v.live++
		}
	}
	for _, op := range d.operations {
		if visible(op) {
			v.operations++
		}
	}
	for _, op := range d.producers {
		if visible(op) {
			v.producers++
		}
	}
	for body := range d.consumers {
		if d.n.VisibleLocked(ctx, &body.n) {
			v.consumers++
		}
	}
	for node := range d.groups.ofBody {
		if d.n.VisibleLocked(ctx, node) {
			v.groups++
		}
	}
	return v
}

// stepSnapshot is the part of a documentSnapshot a build step saves: only the step's own publications, since
// other steps publish beside it and a rollback must not undo their work.
type stepSnapshot struct {
	step         *topo.Step
	producer     provenance.Producer
	ownOperation int
	live         []*topo.Body
	tagged       map[*topo.Body]struct{}
	ofBody       map[*topo.Body]*Container
	circleBuilds map[*Body]*prismrecord.CircleBuildWitness
	placements   map[*Body]*circlePlacementWitness
	identity     map[*Body]struct{}
}

// stepSnapshotLocked saves s's own publications. The caller holds LiveMu.
func (d *Document) stepSnapshotLocked(s *topo.Step) *stepSnapshot {
	idx := s.Index()
	ss := &stepSnapshot{step: s, producer: s.ProducerNext(), tagged: make(map[*topo.Body]struct{}),
		ofBody: make(map[*topo.Body]*Container), circleBuilds: make(map[*Body]*prismrecord.CircleBuildWitness),
		placements: make(map[*Body]*circlePlacementWitness), identity: make(map[*Body]struct{})}
	own := func(b *topo.Body) bool { i, ok := d.n.StepTag[b]; return ok && i == idx }
	for b, i := range d.n.StepTag {
		if i == idx {
			ss.tagged[b] = struct{}{}
		}
	}
	for _, b := range d.n.LiveSet {
		if own(b) {
			ss.live = append(ss.live, b)
		}
	}
	for _, op := range d.operations {
		if op.step == idx {
			ss.ownOperation++
		}
	}
	for b, c := range d.groups.ofBody {
		if own(b) {
			ss.ofBody[b] = c
		}
	}
	for b, w := range d.circleBuilds {
		if own(&b.n) {
			ss.circleBuilds[b] = w
		}
	}
	for b, w := range d.circlePlacements {
		if own(&b.n) {
			ss.placements[b] = w
		}
	}
	for b := range d.identityPlacements {
		if own(&b.n) {
			ss.identity[b] = struct{}{}
		}
	}
	return ss
}

// restoreStepLocked puts the step's own publications back as ss saved them and leaves every other step's alone.
// The caller holds LiveMu.
func (d *Document) restoreStepLocked(ss *stepSnapshot) {
	if ss.step.Done() || d.n.ActiveRun() != ss.step.Run() {
		return // the step's run ended: a goroutine that outlived its step has nothing left to roll back
	}
	idx := ss.step.Index()
	own := func(b *topo.Body) bool { i, ok := d.n.StepTag[b]; return ok && i == idx }
	var drop []*Operation
	owned := 0
	for _, op := range d.operations {
		if op.step == idx {
			owned++
		}
	}
	for i := len(d.operations) - 1; i >= 0 && len(drop) < owned-ss.ownOperation; i-- {
		if d.operations[i].step == idx {
			drop = append(drop, d.operations[i])
			d.unpublishOperationLocked(d.operations[i])
		}
	}
	d.operations = slices.DeleteFunc(d.operations, func(op *Operation) bool { return slices.Contains(drop, op) })
	d.n.LiveSet = slices.DeleteFunc(d.n.LiveSet, own)
	d.n.LiveSet = append(d.n.LiveSet, ss.live...)
	d.groups.ofBody = restoreOwned(d.groups.ofBody, own, ss.ofBody)
	ownHandle := func(b *Body) bool { return own(&b.n) }
	d.circleBuilds = restoreOwned(d.circleBuilds, ownHandle, ss.circleBuilds)
	d.circlePlacements = restoreOwned(d.circlePlacements, ownHandle, ss.placements)
	d.identityPlacements = restoreOwned(d.identityPlacements, ownHandle, ss.identity)
	for b, i := range d.n.StepTag {
		if _, kept := ss.tagged[b]; i == idx && !kept {
			d.n.UntagLocked(b)
		}
	}
	ss.step.RewindProducer(ss.producer)
}

// restoreOwned deletes the keys of m that own names and puts back the saved ones.
func restoreOwned[K comparable, V any](m map[K]V, own func(K) bool, saved map[K]V) map[K]V {
	for k := range m {
		if own(k) {
			delete(m, k)
		}
	}
	if m == nil && len(saved) > 0 {
		m = make(map[K]V, len(saved))
	}
	maps.Copy(m, saved)
	return m
}
