// Package topo holds the internal topology nodes that the root package's
// public handle types embed by value.
package topo

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/denotation"
)

// Vertex is the data of a topological point. The root Vertex handle holds
// one by value.
type Vertex struct {
	// Position is the vertex position in millimetres.
	Position r3.Vec
	// Bound is the proven displacement of Position.
	Bound units.Value
	// Level is the LEVEL half of the shared-denotation certificate: non-zero
	// only for a vertex a builder stamped at one denoted sweep level.
	Level denotation.LevelToken
	// Denot is the CURVE half of the shared-denotation certificate: non-zero
	// only for a vertex a builder minted one for, propagated by copiers.
	Denot denotation.CurveToken
}
