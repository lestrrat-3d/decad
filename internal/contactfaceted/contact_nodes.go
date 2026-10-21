package contactfaceted

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/report"
)

// The contact vocabulary of the public report package, with topology nodes as
// the face, edge, vertex and body references.
type (
	contactReport = report.ContactReportOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]
)
