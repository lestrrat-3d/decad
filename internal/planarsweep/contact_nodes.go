package planarsweep

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/report"
)

// The contact vocabulary of the public report package, with topology nodes as
// the face, edge, vertex and body references.
type (
	contactFeature  = report.ContactFeatureOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactPoint    = report.ContactPointOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactManifold = report.ContactManifoldOf[*topo.Face, *topo.Edge, *topo.Vertex]
)
