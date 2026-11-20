package meshbool

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFiniteRetainedTorusSlab(t *testing.T) {
	s := surfacegeom.Torus{Axis: r3.Vec{Z: 1}, Major: units.Millimeters(3), Minor: units.Millimeters(1)}
	point := func(a, b float64) r3.Vec {
		return r3.Vec{X: (3 + math.Cos(b)) * math.Cos(a), Y: (3 + math.Cos(b)) * math.Sin(a), Z: math.Sin(b)}
	}
	corners := [3]r3.Vec{point(0, 0), point(.005, 0), point(0, .02)}
	owner := &topo.Face{RetainedTag: true, Surface: s, FilletCarrier: &topo.FilletCarrier{Surface: s}}
	other := &topo.Face{RetainedTag: true, Surface: s}
	mesh := &topo.Mesh{Source: []*topo.Face{owner, other}}
	bm := &BoolMesh{Verts: corners[:], Tris: [][3]int{{0, 1, 2}, {0, 1, 2}}, FacetBound: []float64{.003, .003}, SlabBound: []float64{.003, .003}, Gate: .001}
	budget := proofbound.NewReplayTrialBudget().WorkBudget(t.Context())
	require.NoError(t, bm.TightenTorusSlabs(mesh, budget))
	require.LessOrEqual(t, bm.SlabBound[0], bm.Gate)
	// An unrelated face with the same held coordinates cannot borrow the torus tag.
	require.Equal(t, .003, bm.SlabBound[1])
	require.Equal(t, .003, bm.FacetBound[0])
	n := corners[1].Sub(corners[0]).Cross(corners[2].Sub(corners[0]))
	n = n.Scale(1 / n.Len())
	for i := range 11 {
		for j := 0; j <= 10-i; j++ {
			p := point(.005*float64(i)/10, .02*float64(j)/10)
			require.LessOrEqual(t, math.Abs(n.Dot(p.Sub(corners[0]))), bm.SlabBound[0])
		}
	}
	owner.FilletCarrier.Distance = .0011
	bm.SlabBound[0] = .003
	require.NoError(t, bm.TightenTorusSlabs(mesh, budget))
	require.Equal(t, .003, bm.SlabBound[0])
	owner.FilletCarrier.Distance = 0
	owner.AxialDelta = .01
	bm.SlabBound[0] = .003
	require.NoError(t, bm.TightenTorusSlabs(mesh, budget))
	require.Equal(t, .003, bm.SlabBound[0])
	owner.AxialDelta = 0
	// The same authenticated carrier on a displaced triangle may genuinely exceed the gate.
	for i := range corners {
		corners[i].X += .002
	}
	require.NoError(t, bm.TightenTorusSlabs(mesh, budget))
	require.Equal(t, .003, bm.SlabBound[0])
	limited, err := proofbound.NewReplayBudgetLimits(2, 10, 1024)
	require.NoError(t, err)
	var exhausted *proofbound.WorkLimitError
	require.ErrorAs(t, bm.TightenTorusSlabs(mesh, limited.WorkBudget(t.Context())), &exhausted)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, bm.TightenTorusSlabs(mesh, proofbound.NewReplayTrialBudget().WorkBudget(ctx)), context.Canceled)
}
