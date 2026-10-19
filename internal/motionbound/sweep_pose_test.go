package motionbound

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestSweepPointDeviationSquaredSignedIntervals(t *testing.T) {
	zero := proofbound.PointInterval(new(big.Rat))
	one := proofbound.PointInterval(big.NewRat(1, 1))
	var matrix IvMat
	for row := range 3 {
		for col := range 3 {
			matrix[row][col] = zero
		}
		matrix[row][row] = one
	}
	matrix[0][0] = proofbound.IntervalOwned(big.NewRat(1, 1), big.NewRat(2, 1))
	rot := NewScaledIvMat(matrix)
	shift := IvVec{proofbound.IntervalOwned(big.NewRat(1, 3), big.NewRat(2, 3)), zero, zero}
	source := []proofarith.DyV3{
		{proofarith.MustDyOf(0.5), proofarith.MustDyOf(0.25), proofarith.DyZero()},
		{proofarith.MustDyOf(-0.5), proofarith.MustDyOf(0.25), proofarith.DyZero()},
	}
	actual := []proofarith.DyV3{
		{proofarith.DyZero(), proofarith.MustDyOf(0.25), proofarith.DyZero()},
		{proofarith.DyZero(), proofarith.MustDyOf(0.25), proofarith.DyZero()},
	}
	// The first point's farthest X endpoint is 5/3; the second's is 2/3.
	want := big.NewRat(25, 9)
	require.Zero(t, want.Cmp(SweepPointDeviationSquared(source, actual, rot, shift)))
}
