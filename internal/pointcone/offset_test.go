package pointcone_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// meridianSign is the exact sign of x + k*r - c at (x, r).
func meridianSign(x, r float64, k, c *big.Rat) int {
	v := new(big.Rat).Add(proofarith.FloatRat(x), new(big.Rat).Mul(k, proofarith.FloatRat(r)))
	return v.Sub(v, c).Sign()
}

func TestOffsetMeridianStrictSide(t *testing.T) {
	tests := []struct {
		name               string
		x0, r0, x1, r1     float64
		far                float64
		wantK, wantC       *big.Rat // nil: only check the sign facts
		dyadicApexMustMove bool
	}{
		{name: "dyadic", x0: 1, r0: 2, x1: 2, r1: 1, far: -1,
			wantK: big.NewRat(1, 1), wantC: big.NewRat(3, 1), dyadicApexMustMove: true},
		{name: "non-dyadic", x0: 31.0000000000000036, r0: 0.7, x1: 29.3, r1: 2.1, far: -3},
		{name: "steep", x0: 5.5, r0: 0.3, x1: 4.9, r1: 1.7, far: 0.25},
	}
	for _, tc := range tests {
		for _, side := range []int{1, -1} {
			t.Run(fmt.Sprintf("%s/side%+d", tc.name, side), func(t *testing.T) {
				apex, radius, k, c, err := pointcone.OffsetMeridian(tc.x0, tc.r0, tc.x1, tc.r1, tc.far, side)
				require.NoError(t, err)
				if tc.wantK != nil {
					require.Zero(t, tc.wantK.Cmp(k))
					require.Zero(t, tc.wantC.Cmp(c))
				}
				// The face's own endpoints lie on the line exactly.
				require.Zero(t, meridianSign(tc.x0, tc.r0, k, c))
				require.Zero(t, meridianSign(tc.x1, tc.r1, k, c))
				require.Equal(t, side, meridianSign(apex, 0, k, c), "apex side")
				require.Equal(t, side, meridianSign(tc.far, radius, k, c), "far rim side")
				require.Greater(t, apex, tc.far)
				require.Positive(t, radius)
				// Strict means no nearer float also lies strictly on the side.
				back := math.Nextafter(apex, math.Inf(-side))
				require.NotEqual(t, side, meridianSign(back, 0, k, c), "apex is the nearest strict float")
				if tc.dyadicApexMustMove {
					cf, _ := tc.wantC.Float64()
					require.NotEqual(t, cf, apex, "dyadic C must be stepped off")
					require.NotZero(t, meridianSign(apex, 0, k, c))
				}
			})
		}
	}
}

func TestOffsetMeridianRefuses(t *testing.T) {
	inf, nan := math.Inf(1), math.NaN()
	tests := []struct {
		name           string
		x0, r0, x1, r1 float64
		far            float64
		side           int
		want           error
	}{
		{"dx zero", 1, 2, 1, 3, -1, 1, pointcone.ErrUnsupportedMeridian},
		{"dr zero", 1, 2, 2, 2, -1, 1, pointcone.ErrUnsupportedMeridian},
		{"k negative", 1, 1, 2, 2, -1, 1, pointcone.ErrUnsupportedMeridian},
		{"far not below", 1, 2, 2, 1, 1, 1, pointcone.ErrUnsupportedMeridian},
		{"far above", 1, 2, 2, 1, 5, 1, pointcone.ErrUnsupportedMeridian},
		{"zero radius", 1, 0, 2, 1, -1, 1, pointcone.ErrUnsupportedMeridian},
		{"bad side", 1, 2, 2, 1, -1, 0, pointcone.ErrUnsupportedMeridian},
		{"infinite x", inf, 2, 2, 1, -1, 1, pointcone.ErrNonFiniteMeridian},
		{"nan r", 1, nan, 2, 1, -1, 1, pointcone.ErrNonFiniteMeridian},
		{"infinite far", 1, 2, 2, 1, -inf, 1, pointcone.ErrNonFiniteMeridian},
		{"overflowing radius", 1e300, 1e-300, 2e300, 2e-300, -1e300, -1, pointcone.ErrUnsupportedMeridian},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			_, _, _, _, err := pointcone.OffsetMeridian(tc.x0, tc.r0, tc.x1, tc.r1, tc.far, tc.side)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

func TestWalkOrientation(t *testing.T) {
	ccw := [][2]float64{{0, 0}, {1, 0}, {1, 1}, {0, 1}}
	require.Equal(t, 1, pointcone.WalkOrientation(ccw))
	cw := [][2]float64{ccw[3], ccw[2], ccw[1], ccw[0]}
	require.Equal(t, -1, pointcone.WalkOrientation(cw))
	require.Equal(t, 0, pointcone.WalkOrientation([][2]float64{{0, 0}, {1, 1}, {2, 2}}))
	require.Equal(t, 0, pointcone.WalkOrientation(ccw[:2]))
	require.Equal(t, 0, pointcone.WalkOrientation([][2]float64{{0, 0}, {math.Inf(1), 0}, {0, 1}}))
	// An area of 2^-80 is still decided exactly.
	sliver := [][2]float64{{0, 0}, {1, 0}, {1, math.Ldexp(1, -80)}}
	require.Equal(t, 1, pointcone.WalkOrientation(sliver))
}
