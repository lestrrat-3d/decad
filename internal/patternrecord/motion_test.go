package patternrecord

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLinearMotionUsesDenotedStep(t *testing.T) {
	rp, err := ResolveLinear(r3.NewVec(3, 4, 0), units.Millimeters(10), 2)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	motion, err := rp.LinearMotion(frame, r3.Identity(), 1)
	require.NoError(t, err)
	point, charge, err := motion(Point2{U: 1, V: 2})
	require.NoError(t, err)
	require.Equal(t, Point2{U: 7, V: 10}, point)
	require.Zero(t, charge)
	require.Zero(t, proofarith.RationalFloatError(big.NewRat(7, 1), point.U))
}

func TestWorldMotionKeepsExactAxialSimilarity(t *testing.T) {
	for _, axis := range []r3.Vec{{Z: 1}, {Z: -1}} {
		for _, count := range []int{12, 17, 22} {
			rp, err := ResolveCircular(r3.Vec{}, axis, count)
			require.NoError(t, err)
			for i := 1; i < count; i++ {
				motion, err := rp.WorldMotion(i)
				require.NoError(t, err)
				require.True(t, motion.IsValid(), "count=%d instance=%d", count, i)
				require.False(t, motion.IsReflection(), "count=%d instance=%d", count, i)
				require.Zero(t, motion.Translation(), "count=%d instance=%d", count, i)
				basis := motion.Basis()
				require.Equal(t, r3.Vec{Z: 1}, basis.EZ, "count=%d instance=%d", count, i)
				require.Zero(t, basis.EX.Z, "count=%d instance=%d", count, i)
				require.Zero(t, basis.EY.Z, "count=%d instance=%d", count, i)
				require.Equal(t, basis.EX.X, basis.EY.Y, "count=%d instance=%d", count, i)
				require.Equal(t, basis.EX.Y, -basis.EY.X, "count=%d instance=%d", count, i)
			}
		}
	}
}
