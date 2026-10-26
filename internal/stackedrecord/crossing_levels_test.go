package stackedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/stretchr/testify/require"
)

func oneSlab(z0, z1, d0, d1 float64) []stackedrecord.Slab {
	return []stackedrecord.Slab{{Z0: z0, Z1: z1, Z0Delta: d0, Z1Delta: d1}}
}

// TestCrossingBlindLevels reads a target 0..10 and a tool whose end levels
// move against the target's caps. Orders the displacements do not prove, and
// equal levels that carry a displacement, give no levels.
func TestCrossingBlindLevels(t *testing.T) {
	t.Parallel()
	const hair = 0x1p-50 // one ulp of 10 is 2*hair
	target := oneSlab(0, 10, 0, 0)
	for _, tc := range []struct {
		name     string
		tool     []stackedrecord.Slab
		target   []stackedrecord.Slab
		held     []float64
		reach    [][2]int
		admitted bool
	}{
		{"top-start-ends-at-cap", oneSlab(6, 10, 0, 0), target, []float64{0, 6, 10}, [][2]int{{0, -1}, {0, 0}}, true},
		{"top-start-passes-cap", oneSlab(6, 14, 0, 0), target, []float64{0, 6, 10}, [][2]int{{0, -1}, {0, 0}}, true},
		{"bottom-start-ends-inside", oneSlab(0, 4, 0, 0), target, []float64{0, 4, 10}, [][2]int{{0, 0}, {0, -1}}, true},
		{"bottom-start-passes-cap", oneSlab(-3, 4, 0, 0), target, []float64{0, 4, 10}, [][2]int{{0, 0}, {0, -1}}, true},
		{"through-tool", oneSlab(-1, 11, 0, 0), target, nil, nil, false},
		{"tool-inside-target", oneSlab(3, 7, 0, 0), target, nil, nil, false},
		{"tool-equals-target", oneSlab(0, 10, 0, 0), target, nil, nil, false},
		// A displacement leaves a clear order when the gap exceeds both.
		{"clear-margin-above-cap", oneSlab(6, 10+8*hair, 0, 2*hair), oneSlab(0, 10, 0, 2*hair), []float64{0, 6, 10}, [][2]int{{0, -1}, {0, 0}}, true},
		// Equal levels count as one only when both are undisplaced.
		{"equal-cap-tool-displaced", oneSlab(6, 10, 0, hair), target, nil, nil, false},
		{"equal-cap-target-displaced", oneSlab(6, 10, 0, 0), oneSlab(0, 10, 0, hair), nil, nil, false},
		{"equal-start-tool-displaced", oneSlab(0, 4, hair, 0), target, nil, nil, false},
		// A gap inside the combined displacement orders nothing.
		{"gap-inside-displacement-above-cap", oneSlab(6, 10+2*hair, 0, 0), oneSlab(0, 10, 0, 3*hair), nil, nil, false},
		{"gap-inside-displacement-below-start", oneSlab(-hair, 4, 0, 0), oneSlab(0, 10, 2*hair, 0), nil, nil, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			levels, reach, ok := stackedrecord.CrossingBlindLevels(tc.target, tc.tool, new(big.Rat))
			require.Equal(t, tc.admitted, ok)
			if !tc.admitted {
				return
			}
			held := make([]float64, len(levels))
			for i, level := range levels {
				held[i] = level.Held
			}
			require.Equal(t, tc.held, held)
			require.Equal(t, tc.reach, reach)
		})
	}
}
