package diameter_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestVertexWitnesses(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	points := []r3.Vec{{X: 1}, {Y: 2}, {Z: 3}}

	g, ok, err := diameter.VertexWitnesses(budget, points, []float64{1e-6, 4e-6, 2e-6}, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, points, g.Points)
	require.GreaterOrEqual(t, g.Allow, 4e-6)
	require.Less(t, g.Allow, 4e-6*(1+1e-9))

	g, ok, err = diameter.VertexWitnesses(budget, points, []float64{1e-6, 4e-6, 2e-6}, 1e-6)
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, g.Allow, 5e-6)
	require.Less(t, g.Allow, 5e-6*(1+1e-9))

	_, ok, err = diameter.VertexWitnesses(budget, []r3.Vec{{X: math.NaN()}}, []float64{1e-6}, 0)
	require.NoError(t, err)
	require.False(t, ok)

	_, ok, err = diameter.VertexWitnesses(budget, points, []float64{1e-6, -1, 2e-6}, 0)
	require.NoError(t, err)
	require.False(t, ok)
}
