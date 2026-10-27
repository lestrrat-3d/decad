package stackedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
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

// denotedSlab is oneSlab with the levels each end denotes; a nil rational
// records nothing at that end. Every given rational must read back, so a
// refusal below never comes from a record DenoteLevel dropped.
func denotedSlab(t *testing.T, z0, z1, d0, d1 float64, r0, r1 *big.Rat) []stackedrecord.Slab {
	t.Helper()
	slabs := oneSlab(z0, z1, d0, d1)
	slabs[0].Z0Denoted = prismplacement.DenoteLevel(r0, z0, d0)
	slabs[0].Z1Denoted = prismplacement.DenoteLevel(r1, z1, d1)
	if r0 != nil {
		require.Zero(t, slabs[0].DenotedZ0().Cmp(r0))
	}
	if r1 != nil {
		require.Zero(t, slabs[0].DenotedZ1().Cmp(r1))
	}
	return slabs
}

// near is z + k*hair as an exact rational.
func near(z, k float64) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).SetFloat64(z), new(big.Rat).SetFloat64(k*0x1p-50))
}

// TestCrossingBlindLevelsDenoted reads a target 0..10 whose displaced caps
// record the levels they denote. A tool end meets a cap when both record the
// same rational, even where their held floats differ by less than their
// displacements; a missing record or two rationals order nothing.
func TestCrossingBlindLevelsDenoted(t *testing.T) {
	t.Parallel()
	const hair = 0x1p-50 // one ulp of 10 is 2*hair
	below10 := 10 - 2*hair
	top := denotedSlab(t, 0, 10, 0, 2*hair, nil, near(10, -1))
	bottom := denotedSlab(t, -hair, 10, 2*hair, 0, near(0, 0.5), nil)
	topArm, bottomArm := [][2]int{{0, -1}, {0, 0}}, [][2]int{{0, 0}, {0, -1}}
	for _, tc := range []struct {
		name     string
		target   []stackedrecord.Slab
		tool     []stackedrecord.Slab
		shift    *big.Rat
		held     []float64
		reach    [][2]int
		admitted bool
	}{
		{"equal-held-equal-denoted", top, denotedSlab(t, 6, 10, 0, 2*hair, nil, near(10, -1)), new(big.Rat),
			[]float64{0, 6, 10}, topArm, true},
		// The held ends differ by one ulp, inside both displacements, and
		// still denote one level: the result keeps the target's cap.
		{"held-apart-equal-denoted", top, denotedSlab(t, 6, below10, 0, 2*hair, nil, near(10, -1)), new(big.Rat),
			[]float64{0, 6, 10}, topArm, true},
		{"bottom-arm-equal-denoted", bottom, denotedSlab(t, -hair, 4, 2*hair, 0, near(0, 0.5), nil), new(big.Rat),
			[]float64{-hair, 4, 10}, bottomArm, true},
		// The tool lies on an axis 4 above the target's: its denotation
		// lifts by the exact shift before the comparison.
		{"shifted-equal-denoted", top, denotedSlab(t, 2, 6, 0, 2*hair, nil, near(6, -1)), big.NewRat(4, 1),
			[]float64{0, 6, 10}, topArm, true},
		{"tool-end-unrecorded", top, oneSlab(6, 10, 0, 2*hair), new(big.Rat), nil, nil, false},
		{"target-cap-unrecorded", oneSlab(0, 10, 0, 2*hair), denotedSlab(t, 6, 10, 0, 2*hair, nil, near(10, -1)),
			new(big.Rat), nil, nil, false},
		{"two-rationals-one-held-float", top, denotedSlab(t, 6, 10, 0, 2*hair, nil, near(10, 1)), new(big.Rat),
			nil, nil, false},
		{"bottom-arm-two-rationals", bottom, denotedSlab(t, -hair, 4, 2*hair, 0, near(0, -0.5), nil), new(big.Rat),
			nil, nil, false},
		// A copy that rewrites the tool's displacement without restating the
		// denotation reads no denoted level there.
		{"stale-tool-record", top, func() []stackedrecord.Slab {
			slabs := denotedSlab(t, 6, 10, 0, 2*hair, nil, near(10, -1))
			slabs[0].Z1Delta = 3 * hair
			return slabs
		}(), new(big.Rat), nil, nil, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			levels, reach, ok := stackedrecord.CrossingBlindLevels(tc.target, tc.tool, tc.shift)
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
