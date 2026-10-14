package motionbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestHeldRadiusUpperCoversFacetedCubeAndDeparture(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(-2, -3, -4), r3.NewVec(2, -3, -4),
		r3.NewVec(-2, 3, -4), r3.NewVec(2, 3, -4),
		r3.NewVec(-2, -3, 4), r3.NewVec(2, -3, 4),
		r3.NewVec(-2, 3, 4), r3.NewVec(2, 3, 4),
	}
	radius := motionbound.HeldRadiusUpper(verts, 0.5)
	require.GreaterOrEqual(t, radius, 9.5)
	require.Less(t, radius, 9.5+1e-12)
	require.True(t, math.IsInf(motionbound.HeldRadiusUpper(verts, math.NaN()), 1))
	require.True(t, math.IsInf(motionbound.HeldRadiusUpper([]r3.Vec{{X: math.NaN()}}, 0), 1))
}
