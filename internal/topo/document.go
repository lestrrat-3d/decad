package topo

import (
	"sync"

	"github.com/lestrrat-3d/decad/internal/denotation"
)

// Document is the data of the root Document handle, which holds one by
// value.
type Document struct {
	// Mu guards the root Document's live body set alone. Every reader takes a
	// snapshot under it (liveBodies) and works on that copy, so a Remove racing a Verify never
	// shifts the slice a reader is walking.
	Mu sync.RWMutex
	// NextLevel is root denotation.go's own counter behind mintLevel: the LEVEL
	// half of the shared-denotation certificate.
	NextLevel denotation.LevelID
	// NextCurve is root denotation.go's own counter behind mintCurve: the CURVE
	// half of the shared-denotation certificate.
	NextCurve denotation.CurveID
}
