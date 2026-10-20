package sweepproof

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
)

// The contact vocabulary of the public report package, with topology nodes as
// the face, edge, vertex and body references.
type (
	contactFeature  = vocab.ContactFeatureOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactPoint    = vocab.ContactPointOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactManifold = vocab.ContactManifoldOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactReport   = vocab.ContactReportOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]
)
