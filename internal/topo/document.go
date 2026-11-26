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
	// NextLevel is the LEVEL half of the shared-denotation certificate: the
	// last LevelID minted. It is written only by MintLevel, atomically, so two
	// read-only builds on one document may mint at once.
	NextLevel atomic.Int64
	// NextCurve is the CURVE half of the shared-denotation certificate: the
	// last CurveID minted. It is written only by MintCurve, atomically.
	NextCurve atomic.Int64
	// NextProducer is the identity the next successful operation occupies. It
	// is written only by root operation commit helpers.
	NextProducer provenance.Producer
	// Owner is the root Document handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
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
	return Seeds{
		Level:    denotation.LevelID(d.NextLevel.Load()),
		Curve:    denotation.CurveID(d.NextCurve.Load()),
		Producer: d.ProducerAt(ctx),
	}
}

// StartFrom sets the level and curve counters of a transient document to the
// identities in seeds. It is called before the document is shared, so the
// stores are not racing a mint.
func (d *Document) StartFrom(seeds Seeds) {
	d.NextLevel.Store(int64(seeds.Level))
	d.NextCurve.Store(int64(seeds.Curve))
}

// MintLevel advances the level counter (atomically) and returns a token for the new level
// at origin with normal. Identities only increase, so none is reused, even by
// a build that later fails. ctx names the caller, like ProducerAt's.
func (d *Document) MintLevel(_ context.Context, origin, normal r3.Vec) denotation.LevelToken {
	return denotation.NewLevel(denotation.LevelID(d.NextLevel.Add(1)), origin, normal)
}

// MintCurve advances the curve counter (atomically) and returns a token for the new curve.
// Identities only increase, so none is reused, even by a build that later
// fails. ctx names the caller, like ProducerAt's.
func (d *Document) MintCurve(_ context.Context) denotation.CurveToken {
	return denotation.NewCurve(denotation.CurveID(d.NextCurve.Add(1)))
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
