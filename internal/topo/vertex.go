// Package topo holds the internal topology nodes the root handle types wrap.
package topo

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Vertex is the data of a topological point. The root Vertex handle holds a
// pointer to one of these.
type Vertex struct {
	// Position is the vertex position in millimetres.
	Position r3.Vec
	// Bound is the proven displacement of Position.
	Bound units.Value
	// Level is the LEVEL half of the shared-denotation certificate.
	Level denotation.LevelToken
	// Denot is the CURVE half of the shared-denotation certificate.
	Denot denotation.CurveToken
}
