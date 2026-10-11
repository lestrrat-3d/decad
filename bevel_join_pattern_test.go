package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestBevelPatternRingAnglesKeepCapStationsAcrossHalfTurn(t *testing.T) {
	sectors := [][2]float64{{-0.1, 0.1}, {math.Pi - 0.1, math.Pi + 0.1}, {5.0, 5.2}}
	plans := make([]bevelTwoPlan, len(sectors))
	for i, sector := range sectors {
		plans[i] = bevelTwoPlan{
			lo: sector[0], hi: sector[1],
			placed: []bevelPathNode{
				{angle: sector[0], rootBoundary: true},
				{},
				{angle: sector[1], rootBoundary: true},
			},
		}
	}
	angles, err := bevelPatternRingAngles(plans, -math.Pi/8)
	require.NoError(t, err)
	require.True(t, slices.IsSorted(angles))
	for i, angle := range angles {
		if i > 0 {
			require.Less(t, angles[i-1], angle)
		}
		for _, plan := range plans {
			require.False(t, plan.lo < angle && angle < plan.hi,
				"a blank station enters a cap-owned sector")
		}
	}
	for _, plan := range plans {
		require.Equal(t, -1, plan.rootNode[1])
		require.Equal(t, plan.lo, angles[plan.rootNode[0]])
		require.Equal(t, plan.hi, angles[plan.rootNode[2]])
	}
}

func TestBevelPatternAngularGapUsesCircularSeam(t *testing.T) {
	a := bevelTwoPlan{lo: -0.25, hi: -0.15}
	b := bevelTwoPlan{lo: 5.8, hi: 5.9}
	require.InDelta(t, 2*math.Pi-6.15, bevelPatternAngularGap(a, b), 1e-12)
	require.InDelta(t, 2*math.Pi-6.15, bevelPatternAngularGap(b, a), 1e-12)
	require.True(t, math.IsInf(bevelPatternAngularGap(a,
		bevelTwoPlan{lo: -0.2, hi: 0.1}), -1))
}
