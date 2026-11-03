package facetedrecord

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// pointGapReference computes the exact image coordinate with big.Rat.Add and
// a fresh FloatRat per product, the straightforward form pointGap must match.
func pointGapReference(row [3]*big.Rat, t *big.Rat, p r3.Vec) *big.Rat {
	coords := [3]float64{p.X, p.Y, p.Z}
	exact := new(big.Rat).Set(t)
	for j := range coords {
		exact.Add(exact, pointPlaneMul(row[j], ratOfPointPlane(coords[j])))
	}
	return exact
}

func pointGapOf(linear [3][3]*big.Rat, translation [3]*big.Rat, p, moved r3.Vec) float64 {
	got := [3]float64{moved.X, moved.Y, moved.Z}
	gap := 0.0
	for i := range got {
		gap = max(gap, proofbound.RatAbsDiff(pointGapReference(linear[i], translation[i], p), got[i]))
	}
	return proofbound.Radius3D(gap)
}

func TestPointGapMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(5, 6))
	f := func() float64 {
		switch rng.IntN(5) {
		case 0:
			return 0
		case 1:
			return float64(rng.IntN(5) - 2)
		case 2:
			return math.Ldexp(rng.Float64()-0.5, rng.IntN(2000)-1000)
		default:
			return math.Ldexp(rng.Float64()-0.5, rng.IntN(30)-10)
		}
	}
	r := func() *big.Rat {
		if rng.IntN(4) == 0 {
			return big.NewRat(int64(rng.IntN(201)-100), int64(rng.IntN(30)+1))
		}
		return ratOfPointPlane(f())
	}
	for range 20000 {
		var linear [3][3]*big.Rat
		var translation [3]*big.Rat
		for i := range linear {
			translation[i] = r()
			for j := range linear[i] {
				linear[i][j] = r()
			}
		}
		p := r3.Vec{X: f(), Y: f(), Z: f()}
		moved := r3.Vec{X: f(), Y: f(), Z: f()}
		switch rng.IntN(6) {
		case 0:
			// An exact image gives a zero gap.
			for i := range linear {
				for j := range linear[i] {
					linear[i][j] = new(big.Rat)
				}
				translation[i] = new(big.Rat)
			}
			moved = r3.Vec{}
		case 1:
			moved = p
		}
		before := [3]string{translation[0].String(), linear[1][2].String(), linear[2][0].String()}
		coords := ratVecOf(p)
		var term big.Rat
		for i := range linear {
			got := imageCoord(linear[i], translation[i], coords, &term)
			want := pointGapReference(linear[i], translation[i], p)
			require.Zero(t, got.Cmp(want))
			require.Equal(t, want.Num().String(), got.Num().String())
			require.Equal(t, want.Denom().String(), got.Denom().String())
		}
		require.Equal(t, math.Float64bits(pointGapOf(linear, translation, p, moved)),
			math.Float64bits(pointGap(linear, translation, p, moved)))
		require.Equal(t, before, [3]string{translation[0].String(), linear[1][2].String(), linear[2][0].String()})
	}
}
