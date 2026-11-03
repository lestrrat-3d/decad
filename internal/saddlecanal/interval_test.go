package saddlecanal

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestIntervalMulMatchesFourProductTable(t *testing.T) {
	smallest := math.SmallestNonzeroFloat64
	tests := []struct {
		name string
		a, b iv
	}{
		{"positive-positive", iv{1, 3}, iv{2, 4}},
		{"negative-negative", iv{-3, -1}, iv{-4, -2}},
		{"positive-negative", iv{1, 3}, iv{-4, -2}},
		{"negative-positive", iv{-3, -1}, iv{2, 4}},
		{"cross-cross", iv{-3, 2}, iv{-4, 5}},
		{"cross-positive", iv{-3, 2}, iv{2, 4}},
		{"cross-negative", iv{-3, 2}, iv{-4, -2}},
		{"positive-cross", iv{1, 3}, iv{-4, 5}},
		{"negative-cross", iv{-3, -1}, iv{-4, 5}},
		{"signed-zero", iv{math.Copysign(0, -1), 0}, iv{math.Copysign(0, -1), 0}},
		{"subnormal", iv{smallest, 2 * smallest}, iv{-3, -1}},
		{"finite-overflow", iv{math.MaxFloat64 / 2, math.MaxFloat64}, iv{2, 3}},
		{"nan-fallback", iv{math.NaN(), 1}, iv{-2, 3}},
		{"infinity-fallback", iv{1, math.Inf(1)}, iv{-2, 3}},
		{"zero-times-infinity-fallback", iv{0, 1}, iv{0, math.Inf(1)}},
		{"inverted-fallback", iv{2, 1}, iv{-2, 3}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got, want := tc.a.mul(tc.b), tc.a.mulFour(tc.b)
			require.Equal(t, math.Float64bits(want.lo), math.Float64bits(got.lo))
			require.Equal(t, math.Float64bits(want.hi), math.Float64bits(got.hi))
		})
	}
}

// TestStepHelpersMatchNextafter compares down and up with math.Nextafter by
// exact bit pattern over the boundary values, every exponent and a seeded
// random sample. Both NaN results must be NaN.
func TestStepHelpersMatchNextafter(t *testing.T) {
	check := func(x float64) {
		bits := math.Float64bits(x)
		for _, c := range []struct {
			name string
			got  float64
			want float64
		}{
			{"down", down(x), math.Nextafter(x, math.Inf(-1))},
			{"up", up(x), math.Nextafter(x, math.Inf(1))},
		} {
			if math.IsNaN(c.want) {
				require.True(t, math.IsNaN(c.got), "%s(%#x) must be NaN", c.name, bits)
				continue
			}
			require.Equal(t, math.Float64bits(c.want), math.Float64bits(c.got),
				"%s(%#x)", c.name, bits)
		}
	}
	specials := []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), -math.NaN(),
		math.Float64frombits(0x7ff0000000000001), math.Float64frombits(0xfff8000000000123),
		math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		0x1p-1022, -0x1p-1022, 0x1p-969, -0x1p-969, 0x1p1023, 1, -1,
	}
	for _, x := range specials {
		check(x)
		check(math.Nextafter(x, 0))
		check(math.Nextafter(x, math.Inf(1)))
		check(math.Nextafter(x, math.Inf(-1)))
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		for _, x := range []float64{p, -p, math.Nextafter(p, 0), math.Nextafter(p, math.Inf(1)), -math.Nextafter(p, 0)} {
			check(x)
		}
	}
	r := rand.New(rand.NewPCG(3, 4))
	for range 200_000 {
		check(math.Float64frombits(r.Uint64()))
	}
}
