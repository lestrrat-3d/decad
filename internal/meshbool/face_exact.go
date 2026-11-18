package meshbool

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// FaceGateDelta is the displacement the proximity gate charges source face f
// of m: the mesh's FaceBound for f, lowered to the face's surface bound in
// surface when that is smaller (agent-docs/tessellation-design.md §2). A zero
// reading is the claim that every held polygon of f IS the true trimmed patch.
// stated is false when m states no bound for f.
func FaceGateDelta(m *topo.Mesh, surface map[*topo.Face]float64, f *topo.Face) (float64, bool) {
	delta, stated := m.FaceBound[f]
	if !stated {
		return 0, false
	}
	if sigma, ok := surface[f]; ok && sigma < delta {
		delta = sigma
	}
	return delta, true
}

// SetExactFacets fills bm.Exact from m, the tessellation bm was prepared from:
// facet t is exact when FacetBound[t] is zero and the gate reading of its
// source face is zero (agent-docs/coplanar-contact-design.md §3.1). A facet
// whose face has no stated bound is a broken evaluator and returns
// ErrBooleanFailed, as the proximity gate does.
func (bm *BoolMesh) SetExactFacets(m *topo.Mesh) error {
	exact := make([]bool, len(bm.Tris))
	for t := range bm.Tris {
		delta, stated := FaceGateDelta(m, bm.FaceSurface, m.Source[t])
		if !stated {
			return fmt.Errorf(`%w: an operand's tessellation states no displacement bound for one of its own source faces`, decaderr.ErrBooleanFailed)
		}
		exact[t] = bm.FacetBound[t] == 0 && delta <= 0
	}
	bm.Exact = exact
	return nil
}
