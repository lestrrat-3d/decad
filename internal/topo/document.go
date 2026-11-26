package topo

import (
	"context"
	"slices"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// Document is the data of the root Document handle, which holds one by
// value: the live body set and the document's identity counters
// (agent-docs/evaluator-design.md §8). Internal code reads liveness through IsLive
// and Live and mints level and curve identities through MintLevel and
// MintCurve; only root writes LiveSet and NextProducer
// (TestTopoDocumentWriteGuard).
type Document struct {
	// LiveMu guards LiveSet alone. Root operation commit helpers and Remove take it to write; every reader takes it to read and works on a
	// copy, so a Remove racing a Verify never shifts the slice a reader is
	// walking. Internal code never takes the write lock.
	LiveMu sync.RWMutex
	// LiveSet holds the nodes of the live bodies in commit order. It is
	// written only by root operation commit helpers and retire, under
	// LiveMu.
	LiveSet []*Body
	// NextLevel is the LEVEL half of the shared-denotation certificate. It is
	// written only by MintLevel when ctx carries no transient source.
	NextLevel denotation.LevelID
	// NextCurve is the CURVE half of the shared-denotation certificate. It is
	// written only by MintCurve when ctx carries no transient source.
	NextCurve denotation.CurveID
	// NextProducer is the identity the next successful operation occupies. It
	// is written only by root operation commit helpers.
	NextProducer provenance.Producer
	// CounterMu guards the three counters against RunSteps' end-of-run write and
	// the readers (SeedsAt, ProducerAt) a non-step query makes beside it. It is a
	// leaf lock: nothing is acquired while it is held, and it is taken after
	// LiveMu when both are needed.
	CounterMu sync.RWMutex
	// Run is the build-step run in progress, or nil (step.go). StepTag names the
	// step that committed each live or retired body of the run, under LiveMu; a
	// body absent from it was live before the run began.
	Run     atomic.Pointer[Run]
	StepTag map[*Body]int
	// Owner is the root Document handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}

// transientIDs is the private id source a ctx query (Verify and the other
// ctx query methods of root Document) mints from, so the query never moves
// the document's counters and its worker goroutines never write them.
type transientIDs struct {
	doc   *Document
	level atomic.Int64
	curve atomic.Int64
}

type transientIDsKey struct{}

// WithTransientIDs returns ctx carrying a private id source for d, seeded from
// d's current counters, so the ids it hands out exceed every id any body of d
// holds. The source is never written back: bodies minted from it are
// transient and no commit may keep one. A ctx that already carries a source
// for d is returned unchanged, so a nested query shares its caller's source.
func WithTransientIDs(ctx context.Context, d *Document) context.Context {
	if d == nil || ctx == nil {
		return ctx
	}
	if t, _ := ctx.Value(transientIDsKey{}).(*transientIDs); t != nil && t.doc == d {
		return ctx
	}
	seeds := d.SeedsAt(ctx)
	t := &transientIDs{doc: d}
	t.level.Store(int64(seeds.Level))
	t.curve.Store(int64(seeds.Curve))
	return context.WithValue(ctx, transientIDsKey{}, t)
}

func transientOf(ctx context.Context, d *Document) *transientIDs {
	if ctx == nil {
		return nil
	}
	t, _ := ctx.Value(transientIDsKey{}).(*transientIDs)
	if t == nil || t.doc != d {
		return nil
	}
	return t
}

// ProducerAt is the identity the next successful operation called with ctx
// occupies. It is the only read of NextProducer outside root's commit
// helpers; ctx names the caller so the source of the counter lives here: a
// build step's own cursor inside a step. While a run is active, a read made
// without a step's ctx and without a transient source marks the run failed,
// because such a caller is an operation outside the steps.
func (d *Document) ProducerAt(ctx context.Context) provenance.Producer {
	if s := stepOf(ctx, d); s != nil {
		return s.producerNext()
	}
	if r := d.Run.Load(); r != nil && transientOf(ctx, d) == nil {
		r.Fail("a producer identity was read without a step's context")
	}
	d.CounterMu.RLock()
	defer d.CounterMu.RUnlock()
	return d.NextProducer
}

// Seeds are the three identity counters a transient document starts from.
type Seeds struct {
	Level    denotation.LevelID
	Curve    denotation.CurveID
	Producer provenance.Producer
}

// SeedsAt reads the counters a transient document called with ctx starts
// from: the identities the next operation would occupy. A ctx that carries a
// transient source for d reads that source, so a nested transient document
// starts above every id the query has handed out; inside a step the base is
// the step's cursors, never the document's.
func (d *Document) SeedsAt(ctx context.Context) Seeds {
	if t := transientOf(ctx, d); t != nil {
		return Seeds{Level: denotation.LevelID(t.level.Load()), Curve: denotation.CurveID(t.curve.Load()),
			Producer: d.ProducerAt(ctx)}
	}
	if s := stepOf(ctx, d); s != nil {
		return Seeds{Level: denotation.LevelID(s.level.Load()), Curve: denotation.CurveID(s.curve.Load()),
			Producer: s.producerNext()}
	}
	d.CounterMu.RLock()
	defer d.CounterMu.RUnlock()
	return Seeds{Level: d.NextLevel, Curve: d.NextCurve, Producer: d.NextProducer}
}

// MintLevel returns a token for a new level at origin with normal. It takes
// the id from the transient source ctx carries for d, else from the block of
// the build step ctx carries, else it advances the level counter. Identities
// only increase, so none is reused, even by a build that later fails. While a
// run is active, a mint with none of the three marks the run failed and
// returns the zero token, which always declines admission.
func (d *Document) MintLevel(ctx context.Context, origin, normal r3.Vec) denotation.LevelToken {
	if t := transientOf(ctx, d); t != nil {
		return denotation.NewLevel(denotation.LevelID(t.level.Add(1)), origin, normal)
	}
	if s := stepOf(ctx, d); s != nil {
		id, ok := s.mintLevel()
		if !ok {
			return denotation.LevelToken{}
		}
		return denotation.NewLevel(denotation.LevelID(id), origin, normal)
	}
	if r := d.Run.Load(); r != nil {
		r.Fail("a level identity was minted without a step's context")
		return denotation.LevelToken{}
	}
	d.CounterMu.Lock()
	defer d.CounterMu.Unlock()
	d.NextLevel++
	return denotation.NewLevel(d.NextLevel, origin, normal)
}

// MintCurve returns a token for a new curve. It takes the id from the
// transient source ctx carries for d, else from the block of the build step
// ctx carries, else it advances the curve counter. Identities only increase,
// so none is reused, even by a build that later fails. ctx names the caller,
// like ProducerAt's.
func (d *Document) MintCurve(ctx context.Context) denotation.CurveToken {
	if t := transientOf(ctx, d); t != nil {
		return denotation.NewCurve(denotation.CurveID(t.curve.Add(1)))
	}
	if s := stepOf(ctx, d); s != nil {
		id, ok := s.mintCurve()
		if !ok {
			return denotation.CurveToken{}
		}
		return denotation.NewCurve(denotation.CurveID(id))
	}
	if r := d.Run.Load(); r != nil {
		r.Fail("a curve identity was minted without a step's context")
		return denotation.CurveToken{}
	}
	d.CounterMu.Lock()
	defer d.CounterMu.Unlock()
	d.NextCurve++
	return denotation.NewCurve(d.NextCurve)
}

// IsLive reports whether b is in the live set, read under LiveMu.
func (d *Document) IsLive(b *Body) bool {
	d.LiveMu.RLock()
	defer d.LiveMu.RUnlock()
	return slices.Contains(d.LiveSet, b)
}

// Live returns a copy of the live set, read under LiveMu, or nil when no body
// is live.
func (d *Document) Live() []*Body {
	d.LiveMu.RLock()
	defer d.LiveMu.RUnlock()
	return append([]*Body(nil), d.LiveSet...)
}
