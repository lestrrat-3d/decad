package placed

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/report"
)

type (
	contactFeature  = report.ContactFeatureOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactPoint    = report.ContactPointOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactManifold = report.ContactManifoldOf[*topo.Face, *topo.Edge, *topo.Vertex]
	contactReport   = report.ContactReportOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]
)
