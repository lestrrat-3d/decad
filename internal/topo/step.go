package topo

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

// Run is the state of one RunSteps call on a document: the identity block of
// each step and the first scope fault a call made without a step's context.
// agent-docs/steps-design.md owns the rules.
type Run struct {
	doc   *Document
	steps []*Step
	fault atomic.Pointer[string]
	// base counters at the start of the run, and the block size.
	producer0, level0, curve0 int64
	block                     int64
}

// Step is the identity block and the done flag of one build step of a Run.
// Its cursors are atomic so that a mint made on one of decad's own worker
// goroutines inside a call of the step is not a data race; no committed
// body's mint happens off the step's goroutine (agent-docs/steps-design.md).
type Step struct {
	run   *Run
	index int
	// producer is the identity the step's next commit occupies, and
	// producerEnd the first identity past its block.
	producer    atomic.Int64
	producerEnd int64
	// level and curve are the last identity minted. The block ends at
	// levelEnd and curveEnd, inclusive.
	level, curve       atomic.Int64
	levelEnd, curveEnd int64
	exhausted          atomic.Bool
	done               atomic.Bool
}

type stepKey struct{}

// WithStep returns ctx carrying s, so every identity read and mint, and every
// view of the live set made with the result, belongs to s.
func WithStep(ctx context.Context, s *Step) context.Context {
	return context.WithValue(ctx, stepKey{}, s)
}

// stepOf is the step ctx carries for d, or nil.
func stepOf(ctx context.Context, d *Document) *Step {
	if ctx == nil {
		return nil
	}
	s, _ := ctx.Value(stepKey{}).(*Step)
	if s == nil || s.run.doc != d {
		return nil
	}
	return s
}

// StepOf is the step ctx carries for d, or nil.
func (d *Document) StepOf(ctx context.Context) *Step { return stepOf(ctx, d) }

// ActiveRun is the run in progress on d, or nil.
func (d *Document) ActiveRun() *Run { return d.Run.Load() }

// Fail records the first scope fault of the run. RunSteps returns it as
// ErrStepScope when the run ends.
func (r *Run) Fail(reason string) {
	r.fault.CompareAndSwap(nil, &reason)
}

// Fault is the first scope fault recorded, or "".
func (r *Run) Fault() string {
	if p := r.fault.Load(); p != nil {
		return *p
	}
	return ""
}

// Step returns step i of the run.
func (r *Run) Step(i int) *Step { return r.steps[i] }

// Index is the position of the step in the run.
func (s *Step) Index() int { return s.index }

// Run is the run the step belongs to.
func (s *Step) Run() *Run { return s.run }

// Done reports whether the step has finished.
func (s *Step) Done() bool { return s.done.Load() }

// Finish marks the step finished: its ctx is refused from now on.
func (s *Step) Finish() { s.done.Store(true) }

// Exhausted reports whether the step ran past its identity block.
func (s *Step) Exhausted() bool { return s.exhausted.Load() }

func (s *Step) producerNext() provenance.Producer {
	return provenance.Producer(s.producer.Load())
}

// ProducerNext is the identity the step's next commit occupies.
func (s *Step) ProducerNext() provenance.Producer { return s.producerNext() }

// RewindProducer puts the step's producer cursor back to p, an earlier value of ProducerNext. A composite
// operation that rolls back its own publications uses it.
func (s *Step) RewindProducer(p provenance.Producer) { s.producer.Store(int64(p)) }

// AdvanceProducer moves the step's producer cursor past the n identities an
// operation has just occupied. It returns ErrStepIdentities, and moves
// nothing, when the block cannot hold them or an earlier level or curve mint
// ran past its block, and ErrStepScope when the step has finished.
func (s *Step) AdvanceProducer(n provenance.Producer) error {
	if s.done.Load() || s.run.doc.Run.Load() != s.run {
		return fmt.Errorf("%w: the step's context was used after its step finished", decaderr.ErrStepScope)
	}
	if s.exhausted.Load() {
		return fmt.Errorf("%w: a level or curve mint ran past the step's block", decaderr.ErrStepIdentities)
	}
	next := s.producer.Load()
	if n < 0 || int64(n) > s.producerEnd-next {
		s.exhausted.Store(true)
		return fmt.Errorf("%w: the step's producer block is used up", decaderr.ErrStepIdentities)
	}
	s.producer.Store(next + int64(n))
	return nil
}

func (s *Step) mintLevel() (int64, bool) {
	if s.done.Load() {
		return 0, false
	}
	id := s.level.Add(1)
	if id > s.levelEnd {
		s.exhausted.Store(true)
	}
	return id, true
}

func (s *Step) mintCurve() (int64, bool) {
	if s.done.Load() {
		return 0, false
	}
	id := s.curve.Add(1)
	if id > s.curveEnd {
		s.exhausted.Store(true)
	}
	return id, true
}

// BeginRun starts a run of count steps with a block of block identities per
// kind per step. It is ErrStepScope when d already runs steps, and
// ErrDegenerate when count or block is below 1 or the blocks would run past
// the identity range. Every step's cursors start at the document's counters
// plus its own block offset (agent-docs/steps-design.md I1).
func (d *Document) BeginRun(count, block int) (*Run, error) {
	if count < 1 || block < 1 {
		return nil, fmt.Errorf("%w: a run needs at least one step and one identity per block", decaderr.ErrDegenerate)
	}
	d.CounterMu.RLock()
	p0, l0, c0 := int64(d.NextProducer), int64(d.NextLevel), int64(d.NextCurve)
	d.CounterMu.RUnlock()
	top := max(p0, l0, c0)
	if top < 0 || int64(block) > (math.MaxInt-top)/int64(count) {
		return nil, fmt.Errorf("%w: the step identity blocks run past the identity range", decaderr.ErrDegenerate)
	}
	r := &Run{doc: d, producer0: p0, level0: l0, curve0: c0, block: int64(block)}
	r.steps = make([]*Step, count)
	for k := range r.steps {
		off := int64(k) * int64(block)
		s := &Step{run: r, index: k, producerEnd: p0 + off + int64(block),
			levelEnd: l0 + off + int64(block), curveEnd: c0 + off + int64(block)}
		s.producer.Store(p0 + off)
		s.level.Store(l0 + off)
		s.curve.Store(c0 + off)
		r.steps[k] = s
	}
	if !d.Run.CompareAndSwap(nil, r) {
		return nil, fmt.Errorf("%w: the document is already running steps", decaderr.ErrStepScope)
	}
	return r, nil
}

// EndRun ends r: it sorts the live set into step order (the bodies live
// before the run first, then step 0's in commit order, and so on), calls
// whileLocked under LiveMu so root can sort its own indexes the same way,
// drops the step tags, advances the counters past every block and clears the
// run. The counters are written under CounterMu, in the same critical
// section of LiveMu, so a query that reads the live set and then its seeds
// never sees the new set with old seeds.
func (d *Document) EndRun(r *Run, whileLocked func()) {
	d.LiveMu.Lock()
	defer d.LiveMu.Unlock()
	slices.SortStableFunc(d.LiveSet, func(a, b *Body) int {
		return d.tagLocked(a) - d.tagLocked(b)
	})
	if whileLocked != nil {
		whileLocked()
	}
	d.CounterMu.Lock()
	n := int64(len(r.steps)) * r.block
	d.NextProducer = provenance.Producer(r.producer0 + n)
	d.NextLevel += denotation.LevelID(n)
	d.NextCurve += denotation.CurveID(n)
	d.CounterMu.Unlock()
	d.StepTag = nil
	d.Run.Store(nil)
}

// tagLocked is the step index of b, or -1 for a body live before the run. The
// caller holds LiveMu.
func (d *Document) tagLocked(b *Body) int {
	if i, ok := d.StepTag[b]; ok {
		return i
	}
	return -1
}

// TagOf is the step index that committed b during the active run, with ok
// false for a body live before the run (or when no run is active). It is read
// under LiveMu.
func (d *Document) TagOf(b *Body) (int, bool) {
	d.LiveMu.RLock()
	defer d.LiveMu.RUnlock()
	i, ok := d.StepTag[b]
	return i, ok
}

// TagLocked records that step s committed b. The caller holds LiveMu. A nil
// step records nothing.
func (d *Document) TagLocked(s *Step, bodies ...*Body) {
	if s == nil {
		return
	}
	if d.StepTag == nil {
		d.StepTag = make(map[*Body]int)
	}
	for _, b := range bodies {
		d.StepTag[b] = s.index
	}
}

// UntagLocked forgets the step tag of b. The caller holds LiveMu.
func (d *Document) UntagLocked(b *Body) { delete(d.StepTag, b) }

// VisibleLocked reports whether b is part of the view ctx has of the live
// set while a run is active: the bodies live before the run and, inside a
// step, the step's own. With no run active every body is visible. The caller
// holds LiveMu.
func (d *Document) VisibleLocked(ctx context.Context, b *Body) bool {
	return d.visibleToLocked(stepOf(ctx, d), b)
}

// VisibleOutsideLocked is VisibleLocked for a caller outside every step, such
// as a public reader that takes no ctx: it sees the bodies live before the
// run. The caller holds LiveMu.
func (d *Document) VisibleOutsideLocked(b *Body) bool {
	return d.visibleToLocked(nil, b)
}

func (d *Document) visibleToLocked(s *Step, b *Body) bool {
	r := d.Run.Load()
	if r == nil {
		return true
	}
	i, tagged := d.StepTag[b]
	if !tagged {
		return true
	}
	return s != nil && s.run == r && s.index == i
}

// LiveIn returns a copy of the live set as ctx sees it. Outside a run that is
// the whole set. During a run it is the bodies live before the run, plus the
// calling step's own bodies inside a step: a call never sees another step's
// work, so its result does not depend on how far another step has got.
func (d *Document) LiveIn(ctx context.Context) []*Body {
	return d.liveFor(stepOf(ctx, d))
}

// LiveOutside is LiveIn for a caller outside every step: during a run it is
// the bodies live before the run.
func (d *Document) LiveOutside() []*Body {
	return d.liveFor(nil)
}

func (d *Document) liveFor(s *Step) []*Body {
	d.LiveMu.RLock()
	defer d.LiveMu.RUnlock()
	if d.Run.Load() == nil {
		return append([]*Body(nil), d.LiveSet...)
	}
	out := make([]*Body, 0, len(d.LiveSet))
	for _, b := range d.LiveSet {
		if d.visibleToLocked(s, b) {
			out = append(out, b)
		}
	}
	return out
}
