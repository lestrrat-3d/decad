package surfacenormal_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolvedCircularAllowWithEitherBasisSign(t *testing.T) {
	point := func(x, y, z int64) proofbound.IvVec3 {
		return proofbound.IvVec3{
			proofbound.PointInterval(big.NewRat(x, 1)),
			proofbound.PointInterval(big.NewRat(y, 1)),
			proofbound.PointInterval(big.NewRat(z, 1)),
		}
	}
	centre := [2]proofbound.RatInterval{
		proofbound.PointInterval(big.NewRat(1, 1)),
		proofbound.PointInterval(big.NewRat(2, 1)),
	}
	p := r3.NewVec(5, 4, 3)
	radial := math.Sqrt(41)
	trueNormal := r3.NewVec(5-10/radial, 4-8/radial, 2)
	trueNormal = trueNormal.Scale(1 / trueNormal.Len())

	for _, xSign := range []int64{1, -1} {
		r := surfacenormal.Revolved{
			Origin:   point(0, 0, 0),
			Basis:    [3]proofbound.IvVec3{point(xSign, 0, 0), point(0, 1, 0), point(0, 0, 1)},
			Circular: true,
			Centre:   centre,
			Valid:    true,
		}
		for _, reversed := range []bool{false, true} {
			outward := trueNormal
			if reversed {
				outward = outward.Scale(-1)
			}
			held := outward.Add(r3.NewVec(1e-8, 0, 0))
			held = held.Scale(1 / held.Len())
			allow, status := r.Allow(p, held, reversed)
			require.Equal(t, surfacenormal.Proven, status, "basis X sign %d, reversed %v", xSign, reversed)
			require.GreaterOrEqual(t, allow, held.Sub(outward).Len(), "basis X sign %d, reversed %v", xSign, reversed)
			require.Less(t, allow, 1e-6, "basis X sign %d, reversed %v", xSign, reversed)
		}
	}
}
