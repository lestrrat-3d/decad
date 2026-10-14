package topo

import (
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// Document is the data of the root Document handle, which holds one by
// value: the live body set and the document's identity counters
// (docs/evaluator-design.md §8). Internal code reads liveness through IsLive
// and Live and mints level and curve identities through MintLevel and
// MintCurve; only root writes LiveSet and NextProducer
// (TestTopoDocumentWriteGuard).
type Document struct {
	// LiveMu guards LiveSet alone. Root commit, commitMany, commitSpan and
	// Remove take it to write; every reader takes it to read and works on a
	// copy, so a Remove racing a Verify never shifts the slice a reader is
	// walking. Internal code never takes the write lock.
	LiveMu sync.RWMutex
	// LiveSet holds the nodes of the live bodies in commit order. It is
	// written only by root commit, commitMany, commitSpan and retire, under
	// LiveMu.
	LiveSet []*Body
	// NextLevel is the LEVEL half of the shared-denotation certificate. It is
	// written only by MintLevel.
	NextLevel denotation.LevelID
	// NextCurve is the CURVE half of the shared-denotation certificate. It is
	// written only by MintCurve.
	NextCurve denotation.CurveID
	// NextProducer is the identity the next successful operation occupies. It
	// is written only by root commit, commitMany and commitSpan.
	NextProducer provenance.Producer
	// Owner is the root Document handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}

// MintLevel advances the level counter and returns a token for the new level
// at origin with normal. Identities only increase, so none is reused, even by
// a build that later fails.
func (d *Document) MintLevel(origin, normal r3.Vec) denotation.LevelToken {
	d.NextLevel++
	return denotation.NewLevel(d.NextLevel, origin, normal)
}

// MintCurve advances the curve counter and returns a token for the new curve.
// Identities only increase, so none is reused, even by a build that later
// fails.
func (d *Document) MintCurve() denotation.CurveToken {
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
