package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointConeUpperFarPlaneNodeRequiresCapCharge(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(10, 1), slope: big.NewRat(1, 1)}
	far := r3.Vec{X: 5, Y: 4.9}
	sample, err := pointConeFactor(far, cone)
	require.NoError(t, err)
	require.Less(t, sample.Q+sample.QError, 1.0)
	_, _, err = pointcone.UpperCapNode(far, sample, false)
	require.ErrorIs(t, err, ErrUnsupported)
	clipped, round, err := pointcone.UpperCapNode(far, sample, true)
	require.NoError(t, err)
	require.Equal(t, far, clipped)
	require.GreaterOrEqual(t, round, far.Scale(sample.Alpha).Sub(far).Len())
}

func TestPointConeOutsideWitnessRejectsUncertainContact(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(13, 1), slope: big.NewRat(1, 1)}
	base := facetedPayload{verts: []r3.Vec{{}, {X: 8, Y: 5.00001}}}
	require.True(t, pointConeHasOutsideWitness(base, cone))
	base.meshBound = 0.001
	require.False(t, pointConeHasOutsideWitness(base, cone))
}
