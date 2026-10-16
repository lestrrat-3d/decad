package obliqueboss

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBasisAdmittedRejectsExactTiltLimit(t *testing.T) {
	limit := 0.349
	atLimit := facetedrecord.ObliqueBossRecord{
		Axis: r3.Vec{X: limit, Z: 0.945},
		Ex:   r3.Vec{X: 0.945, Z: -limit},
		Ey:   r3.Vec{Y: 1},
	}
	inside := atLimit
	inside.Axis.X = math.Nextafter(limit, 0)
	inside.Ex.Z = -inside.Axis.X

	// Both sides lift the same float64 value to a rational, so equality does
	// not depend on whether decimal 0.349 has an exact binary representation.
	limitRat := proofarith.FloatRat(limit)
	limitSquared := new(big.Rat).Mul(limitRat, limitRat)
	tiltSquared := func(rec *facetedrecord.ObliqueBossRecord) *big.Rat {
		planar := r3.Vec{X: rec.Axis.X, Y: rec.Axis.Y}
		return ObliqueRatDot(planar, planar)
	}
	require.Zero(t, tiltSquared(&atLimit).Cmp(limitSquared))
	require.Less(t, tiltSquared(&inside).Cmp(limitSquared), 0)

	// The other guards accept the equality fixture, so only the tilt check
	// can distinguish >= from > at this boundary.
	require.Greater(t, atLimit.Axis.Z, 0.94)
	lo, hi := proofarith.FloatRat(0.999), proofarith.FloatRat(1.009)
	lo.Mul(lo, lo)
	hi.Mul(hi, hi)
	for _, basis := range []r3.Vec{atLimit.Axis, atLimit.Ex, atLimit.Ey} {
		square := ObliqueRatDot(basis, basis)
		require.Greater(t, square.Cmp(lo), 0)
		require.Less(t, square.Cmp(hi), 0)
	}
	for _, pair := range [][2]r3.Vec{
		{atLimit.Axis, atLimit.Ex}, {atLimit.Axis, atLimit.Ey}, {atLimit.Ex, atLimit.Ey},
	} {
		require.Zero(t, ObliqueRatDot(pair[0], pair[1]).Sign())
	}
	require.True(t, basisAdmitted(&inside))
	require.False(t, basisAdmitted(&atLimit))
}
