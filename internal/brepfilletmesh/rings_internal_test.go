package brepfilletmesh

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// twistRings is TwistRingCount's count once the band's spread is known. The
// expected counts are worked by hand: ⌈spread·π/(4·chord)⌉ against the
// sagitta count n, capped at 4096.
func TestTwistRings(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name          string
		n             int
		spread, chord float64
		want          int
	}{
		// 0.1·π/0.04 = 7.854.
		{"twist raises the count", 2, 0.1, 0.01, 8},
		{"sagitta count already covers the twist", 10, 0.1, 0.01, 10},
		{"no spread keeps the sagitta count", 4, 0, 0.1, 4},
		// 4·π/4e-9 ≈ 3.1e9 rings.
		{"cap", 2, 4, 1e-9, 4096},
		{"zero chord", 2, 4, 0, 2},
		{"negative chord", 2, 4, -0.1, 2},
		{"NaN chord", 2, 4, math.NaN(), 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, twistRings(tc.n, tc.spread, tc.chord))
		})
	}
}
