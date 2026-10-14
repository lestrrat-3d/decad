package topo

import (
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/r3"
)

// Document is the data of the root Document handle, which holds one by
// value.
type Document struct {
	// Mu guards Bodies alone. Every reader takes a snapshot under it
	// (liveBodies) and works on that copy, so a Remove racing a Verify never
	// shifts the slice a reader is walking.
	Mu sync.RWMutex
	// Bodies is the live body set. Only root's commit and retire write it;
	// internal code reads it through IsLive and Live.
	Bodies []*Body
	// NextLevel is denotation.go's own counter behind mintLevel: the LEVEL
	// half of the shared-denotation certificate.
	NextLevel denotation.LevelID
	// NextCurve is denotation.go's own counter behind mintCurve: the CURVE
	// half of the shared-denotation certificate.
	NextCurve denotation.CurveID
	// Owner is the root handle that holds this node (decad.Document); only
	// root sets it.
	Owner any
}

// MintLevel mints a fresh LEVEL certificate at origin with normal.
func (d *Document) MintLevel(origin, normal r3.Vec) denotation.LevelToken {
	d.NextLevel++
	return denotation.NewLevel(d.NextLevel, origin, normal)
}

// MintCurve mints a fresh CURVE certificate.
func (d *Document) MintCurve() denotation.CurveToken {
	d.NextCurve++
	return denotation.NewCurve(d.NextCurve)
}

// IsLive reports whether b is currently part of the model.
func (d *Document) IsLive(b *Body) bool {
	d.Mu.RLock()
	defer d.Mu.RUnlock()
	return slices.Contains(d.Bodies, b)
}

// Live returns a snapshot of the live body set, nil when it is empty.
func (d *Document) Live() []*Body {
	d.Mu.RLock()
	defer d.Mu.RUnlock()
	return append([]*Body(nil), d.Bodies...)
}
