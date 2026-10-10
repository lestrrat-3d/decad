package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointConeTriangleBoundIncludesReciprocalCurvature(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(1, 5), slope: big.NewRat(1, 1)}
	tri := [3]r3.Vec{{X: -0.9, Y: 1.1}, {X: -0.9, Y: 1.1125},
		{X: -0.9, Y: 1.1, Z: 0.0125}}
	_, alphaBound, _, err := pointConeTriangleBound(tri, cone)
	require.NoError(t, err)
	alpha := func(p r3.Vec) float64 {
		return 0.2 / (p.X + math.Hypot(p.Y, p.Z))
	}
	mid := tri[0].Scale(0.5).Add(tri[1].Scale(0.5))
	departure := math.Abs(alpha(mid) - 0.5*(alpha(tri[0])+alpha(tri[1])))
	require.Greater(t, departure, 0.0008)
	require.GreaterOrEqual(t, alphaBound, departure)
}

func TestPointConeSourceMustClearFiniteFarCap(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{far: big.NewRat(-2, 1)}
	base := facetedPayload{verts: []r3.Vec{{}, {X: -3, Y: 10}}}
	require.False(t, pointConeSourceAboveFarCap(base, cone))
	base.verts[1].X = -1
	require.True(t, pointConeSourceAboveFarCap(base, cone))
	base.meshBound = 1
	require.False(t, pointConeSourceAboveFarCap(base, cone))
}

func TestPointConeOutsideWitnessRejectsUncertainContact(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(13, 1), slope: big.NewRat(1, 1)}
	base := facetedPayload{verts: []r3.Vec{{}, {X: 8, Y: 5.00001}}}
	require.True(t, pointConeHasOutsideWitness(base, cone))
	base.meshBound = 0.001
	require.False(t, pointConeHasOutsideWitness(base, cone))
}
