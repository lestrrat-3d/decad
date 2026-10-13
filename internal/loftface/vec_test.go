package loftface_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func requireMat(t *testing.T, want [3][3]int64, got loftface.RatMat) {
	t.Helper()
	for i := range want {
		for j := range want[i] {
			require.Zero(t, big.NewRat(want[i][j], 1).Cmp(got[i][j]), "entry (%d, %d) is %s", i, j, got[i][j])
		}
	}
}

func requireVec(t *testing.T, want [3]int64, got loftface.RatVec) {
	t.Helper()
	for i := range want {
		require.Zero(t, big.NewRat(want[i], 1).Cmp(got[i]), "entry %d is %s", i, got[i])
	}
}

func TestAffineCompose(t *testing.T) {
	translation, err := r3.Translation(r3.Vec{X: 1, Y: 2, Z: 3})
	require.NoError(t, err)

	motion := loftface.Motion(translation)
	requireMat(t, [3][3]int64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, motion.A)
	requireVec(t, [3]int64{1, 2, 3}, motion.B)

	scale := loftface.Scale(r3.Vec{X: 1}, 2)
	requireMat(t, [3][3]int64{{2, 0, 0}, {0, 2, 0}, {0, 0, 2}}, scale.A)
	requireVec(t, [3]int64{-1, 0, 0}, scale.B)

	composed := loftface.Compose(scale, motion)
	requireMat(t, [3][3]int64{{2, 0, 0}, {0, 2, 0}, {0, 0, 2}}, composed.A)
	requireVec(t, [3]int64{0, 2, 3}, composed.B)
}

func TestMatMulOrder(t *testing.T) {
	r := func(v int64) *big.Rat { return big.NewRat(v, 1) }
	a := loftface.RatMat{{r(1), r(2), r(0)}, {r(0), r(1), r(0)}, {r(0), r(0), r(1)}}
	b := loftface.RatMat{{r(1), r(0), r(0)}, {r(3), r(1), r(0)}, {r(0), r(0), r(1)}}
	requireMat(t, [3][3]int64{{7, 2, 0}, {3, 1, 0}, {0, 0, 1}}, loftface.MatMul(a, b))
}

// TestMotionColumns checks that Motion stores the basis vectors as matrix
// columns: a quarter turn about Z maps X to Y, so A[1][0] is +1 and A[0][1] is -1.
func TestMotionColumns(t *testing.T) {
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	a := loftface.Motion(turn).A
	entry := func(i, j int) float64 {
		f, _ := a[i][j].Float64()
		return f
	}
	require.InDelta(t, 1, entry(1, 0), 1e-12)
	require.InDelta(t, -1, entry(0, 1), 1e-12)
	require.InDelta(t, 1, entry(2, 2), 1e-12)
	require.InDelta(t, 0, entry(0, 0), 1e-12)
}
