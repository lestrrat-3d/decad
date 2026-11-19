package occupancy

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// applyExactReference is M·p + T by term-by-term big.Rat arithmetic.
func applyExactReference(a Affine, p xvec) xvec {
	var out xvec
	for i := range 3 {
		s := ratOf(a.T[i])
		for j := range 3 {
			s.Add(s, new(big.Rat).Mul(ratOf(a.M[i][j]), p[j]))
		}
		out[i] = s
	}
	return out
}

func TestAffineApplyExactMatchesTermByTerm(t *testing.T) {
	c, s := math.Cos(0.7391), math.Sin(0.7391)
	affines := map[string]Affine{
		"identity": Identity(),
		"rotation": {
			M: [3][3]float64{{c, -s, 0}, {s, c, 0}, {0, 0, 1}},
			T: [3]float64{0.1, -2.3, 1.0 / 3.0},
		},
		"dense": {
			M: [3][3]float64{{c, -s * c, s * s}, {s, c * c, -c * s}, {0.3, -0.7, math.Pi}},
			T: [3]float64{math.E, -math.Sqrt2, 1e-3},
		},
		"zero rows and columns": {
			M: [3][3]float64{{0, 0, 0}, {0, 2.5, 0}, {0, 0, 0}},
			T: [3]float64{0, 0, 4},
		},
		"zero": {},
		"tiny and large": {
			M: [3][3]float64{{5e-324, 1e300, -1e-300}, {1e-310, -1e150, 7}, {2, 3, 1e-5}},
			T: [3]float64{1e-320, -1e290, 0.5},
		},
	}
	points := map[string]xvec{
		"origin":   xvecOf3(0, 0, 0),
		"axis":     xvecOf3(1, 0, 0),
		"negative": xvecOf3(-3.25, 0.125, -7),
		"mantissa": xvecOf3(0.1, math.Pi, -math.Sqrt2),
		"tiny":     xvecOf3(1e-300, -5e-324, 1e200),
		"fraction": {big.NewRat(-7, 3), big.NewRat(22, 7), big.NewRat(0, 1)},
	}
	for an, a := range affines {
		for pn, p := range points {
			t.Run(an+"/"+pn, func(t *testing.T) {
				got := a.applyExact(p)
				want := applyExactReference(a, p)
				for i := range 3 {
					require.Zero(t, got[i].Cmp(want[i]), "coordinate %d", i)
					require.Zero(t, got[i].Num().Cmp(want[i].Num()), "numerator %d", i)
					require.Zero(t, got[i].Denom().Cmp(want[i].Denom()), "denominator %d", i)
				}
			})
		}
	}
}

func xvecOf3(x, y, z float64) xvec { return xvec{ratOf(x), ratOf(y), ratOf(z)} }
