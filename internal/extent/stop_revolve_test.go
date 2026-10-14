package extent_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolvedPlaneStopRequiresPointCapDirection(t *testing.T) {
	t.Parallel()
	point := func(n int64) proofbound.RatInterval {
		return proofbound.Interval(big.NewRat(n, 1), big.NewRat(n, 1))
	}
	// At the lower endpoint of either widened direction, the cap plane is
	// exactly perpendicular to the sweep. Reading that endpoint as the
	// direction would therefore admit a plane the record does not prove.
	r := surfacenormal.Revolved{
		Origin: proofbound.IvVec3{point(0), point(5), point(0)},
		Basis: [3]proofbound.IvVec3{
			{point(1), point(0), point(0)},
			{point(0), point(1), point(0)},
			{point(0), point(0), point(1)},
		},
		Cap: true, Valid: true,
		CapDir: [2]proofbound.RatInterval{point(1), point(-1)},
	}
	sweep := mustFrame(t, r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(0, 0, 1))
	level, ok := extent.RevolvedPlaneStop(r, sweep)
	require.True(t, ok)
	require.Negative(t, level.Sign())

	r.CapDir[0] = proofbound.Interval(big.NewRat(1, 1), big.NewRat(101, 100))
	_, ok = extent.RevolvedPlaneStop(r, sweep)
	require.False(t, ok)

	r.CapDir[0] = point(1)
	r.CapDir[1] = proofbound.Interval(big.NewRat(-1, 1), big.NewRat(-99, 100))
	_, ok = extent.RevolvedPlaneStop(r, sweep)
	require.False(t, ok)
}
