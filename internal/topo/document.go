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
// helpers; ctx names the caller so the source of the counter lives here.
func (d *Document) ProducerAt(_ context.Context) provenance.Producer {
	return d.NextProducer
}

// Seeds are the three identity counters a transient document starts from.
type Seeds struct {
	Level    denotation.LevelID
	Curve    denotation.CurveID
	Producer provenance.Producer
}

// SeedsAt reads the counters a transient document called with ctx starts
// from: the identities the next operation would occupy.
func (d *Document) SeedsAt(ctx context.Context) Seeds {
	return Seeds{Level: d.NextLevel, Curve: d.NextCurve, Producer: d.ProducerAt(ctx)}
}

// MintLevel returns a token for a new level at origin with normal. It takes
// the id from the transient source ctx carries for d, else it advances the
// level counter. Identities only increase, so none is reused, even by a build
// that later fails.
func (d *Document) MintLevel(ctx context.Context, origin, normal r3.Vec) denotation.LevelToken {
	if t := transientOf(ctx, d); t != nil {
		return denotation.NewLevel(denotation.LevelID(t.level.Add(1)), origin, normal)
	}
	d.NextLevel++
	return denotation.NewLevel(d.NextLevel, origin, normal)
}

// MintCurve returns a token for a new curve. It takes the id from the
// transient source ctx carries for d, else it advances the curve counter.
// Identities only increase, so none is reused, even by a build that later
// fails. ctx names the caller, like ProducerAt's.
func (d *Document) MintCurve(ctx context.Context) denotation.CurveToken {
	if t := transientOf(ctx, d); t != nil {
		return denotation.NewCurve(denotation.CurveID(t.curve.Add(1)))
	}
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
