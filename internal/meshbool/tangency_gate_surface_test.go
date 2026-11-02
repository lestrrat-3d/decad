package meshbool

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFacesOfMeshReadsSurfaceBound holds the proximity gate's face slack to
// the smaller of the mesh's face bound and a refined surface bound: a smaller
// surface bound lowers it, a larger one leaves it, and a face without one keeps
// its mesh bound.
//
// Leg shown to fail: ignoring the surface map leaves the first face at 0.3.
func TestFacesOfMeshReadsSurfaceBound(t *testing.T) {
	t.Parallel()
	refined, loose, plain := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Source:    []*topo.Face{refined, loose, plain},
		FaceBound: map[*topo.Face]float64{refined: 0.3, loose: 0.01, plain: 0.2},
	}
	boxes := make([][2]r3.Vec, len(m.Source))
	surface := map[*topo.Face]float64{refined: 0.04, loose: 0.5}
	faces, err := facesOfMesh(proofbound.NewWorkBudget(t.Context()), m, boxes, surface)
	require.NoError(t, err)
	require.Len(t, faces, 3)
	got := map[*topo.Face]float64{}
	for _, f := range faces {
		got[f.source] = f.delta
	}
	require.InDelta(t, 0.04, got[refined], 0)
	require.InDelta(t, 0.01, got[loose], 0)
	require.InDelta(t, 0.2, got[plain], 0)
}
