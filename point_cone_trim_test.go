package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPointConeTrimRejectsRoundTrippedPointLoft(t *testing.T) {
	for _, tc := range []struct {
		name string
		deg  float64
	}{
		{"30deg", 30},
		{"1deg", 1},
		{"0.001deg", 0.001},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			tooth := pointLoftTooth(t, doc)
			motion, err := r3.RotationAround(r3.Vec{X: 1000, Y: 1, Z: 1},
				r3.Vec{Z: 1}, units.Degrees(tc.deg))
			require.NoError(t, err)
			inverse, err := motion.Inverse()
			require.NoError(t, err)
			round, err := motion.Then(inverse)
			require.NoError(t, err)
			require.Equal(t, r3.Identity(), round)

			moved, err := Transform(t.Context(), tooth, motion)
			require.NoError(t, err)
			back, err := Transform(t.Context(), moved, inverse)
			require.NoError(t, err)
			fp, ok := back.n.Payload.(facetedPayload)
			require.True(t, ok)
			require.Equal(t, r3.Identity(), fp.Xform)
			require.True(t, fp.PointSection == nil, "a nonidentity delta must drop the source record")

			got, err := Cut(t.Context(), back, pointLoftConeTool(t, doc, 5))
			require.NoError(t, err)
			gfp, ok := got.n.Payload.(facetedPayload)
			require.True(t, ok)
			require.Nil(t, gfp.PointCone)
		})
	}

	t.Run("unplaced copy", func(t *testing.T) {
		doc := New()
		cloned, err := Copy(t.Context(), pointLoftTooth(t, doc))
		require.NoError(t, err)
		fp, ok := cloned.n.Payload.(facetedPayload)
		require.True(t, ok)
		require.NotNil(t, fp.PointSection)
		got, err := Cut(t.Context(), cloned, pointLoftConeTool(t, doc, 5))
		require.NoError(t, err)
		gfp, ok := got.n.Payload.(facetedPayload)
		require.True(t, ok)
		require.NotNil(t, gfp.PointCone)
	})
}

func TestPointConeUpperFarPlaneNodeRequiresCapCharge(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{Apex: big.NewRat(10, 1), Slope: big.NewRat(1, 1)}
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
	cone := &pointConeLimit{Apex: big.NewRat(13, 1), Slope: big.NewRat(1, 1)}
	base := facetedPayload{Verts: []r3.Vec{{}, {X: 8, Y: 5.00001}}}
	require.True(t, pointConeHasOutsideWitness(base, cone))
	base.MeshBound = 0.001
	require.False(t, pointConeHasOutsideWitness(base, cone))
}
