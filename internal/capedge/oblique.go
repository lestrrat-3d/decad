package capedge

import (
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ObliqueProfile adapts the selected topology edge to its recorded
// section edge, then applies the cutter's convexity and clearance gates.
func ObliqueProfile(pp prismrecord.Payload, edge *topo.Edge, radius float64) bool {
	vertices, ok := ConvexVertices(pp.Profile)
	if !ok {
		return false
	}
	match := -1
	for i, a := range vertices {
		b := vertices[(i+1)%len(vertices)]
		for _, level := range []float64{pp.Z0, pp.Z1} {
			if !brepgeom.MatchEndpoints(edge.Start.Position, edge.End.Position,
				pp.Point(a.U, a.V, level), pp.Point(b.U, b.V, level), 1e-6) {
				continue
			}
			if match >= 0 {
				return false
			}
			match = i
		}
	}
	return Eligible(vertices, match, radius)
}
