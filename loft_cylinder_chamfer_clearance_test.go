package decad

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The old rational calculation is an independent oracle for the integer
// comparison, including exact equality at the strict clearance threshold.
func loftCylinderChamferAxisGapRational(a, b [3]proofarith.Xpt,
	axis proofarith.Xpt, bound *big.Rat) bool {
	norm2 := proofarith.XdotRat(axis, axis)
	if norm2.Sign() == 0 {
		return false
	}
	minA, maxA := proofarith.XdotRat(axis, a[0]), proofarith.XdotRat(axis, a[0])
	minB, maxB := proofarith.XdotRat(axis, b[0]), proofarith.XdotRat(axis, b[0])
	for i := 1; i < 3; i++ {
		p := proofarith.XdotRat(axis, a[i])
		if p.Cmp(minA) < 0 {
			minA = p
		}
		if p.Cmp(maxA) > 0 {
			maxA = p
		}
		p = proofarith.XdotRat(axis, b[i])
		if p.Cmp(minB) < 0 {
			minB = p
		}
		if p.Cmp(maxB) > 0 {
			maxB = p
		}
	}
	gap := new(big.Rat).Sub(minA, maxB)
	if reverse := new(big.Rat).Sub(minB, maxA); reverse.Cmp(gap) > 0 {
		gap = reverse
	}
	if gap.Sign() <= 0 {
		return false
	}
	gap2 := new(big.Rat).Mul(gap, gap)
	bound2 := new(big.Rat).Mul(bound, bound)
	return gap2.Cmp(new(big.Rat).Mul(bound2, norm2)) > 0
}

func TestLoftCylinderChamferAxisGapMatchesRational(t *testing.T) {
	toPoint := func(x, y, z float64) proofarith.Xpt {
		return proofarith.XptOf(r3.NewVec(x, y, z))
	}
	a := [3]proofarith.Xpt{toPoint(0, 0, 0), toPoint(0, 1, 0), toPoint(0, 0, 1)}
	b := [3]proofarith.Xpt{toPoint(1, 0, 0), toPoint(1, 1, 0), toPoint(1, 0, 1)}
	axis := toPoint(1, 0, 0)
	for _, distance := range []float64{0, math.Nextafter(1, 0), 1, math.Nextafter(1, math.Inf(1))} {
		bound := proofarith.FloatRat(distance)
		squared := loftCylinderChamferBoundSquare{
			num: new(big.Int).Mul(bound.Num(), bound.Num()),
			den: new(big.Int).Mul(bound.Denom(), bound.Denom()),
		}
		require.Equal(t, loftCylinderChamferAxisGapRational(a, b, axis, bound),
			loftCylinderChamferAxisGapAbove(a, b, axis, squared), "bound %g", distance)
	}

	rng := rand.New(rand.NewSource(17))
	coordinate := func() float64 { return float64(rng.Intn(65)-32) / 16 }
	for trial := range 128 {
		for corner := range 3 {
			a[corner] = toPoint(coordinate(), coordinate(), coordinate())
			b[corner] = toPoint(coordinate(), coordinate(), coordinate())
		}
		if trial%3 == 0 {
			a[0] = proofarith.Xlerp(a[0], a[1], big.NewInt(1), big.NewInt(3))
		}
		axis = toPoint(coordinate(), coordinate(), coordinate())
		if trial%4 == 0 {
			axis = proofarith.Xcross(proofarith.Xsub(a[1], a[0]), proofarith.Xsub(a[2], a[0]))
		}
		bound := proofarith.FloatRat([]float64{0, 0.125, 0.5, 2}[trial%4])
		squared := loftCylinderChamferBoundSquare{
			num: new(big.Int).Mul(bound.Num(), bound.Num()),
			den: new(big.Int).Mul(bound.Denom(), bound.Denom()),
		}
		require.Equal(t, loftCylinderChamferAxisGapRational(a, b, axis, bound),
			loftCylinderChamferAxisGapAbove(a, b, axis, squared), "trial %d", trial)
	}
}
