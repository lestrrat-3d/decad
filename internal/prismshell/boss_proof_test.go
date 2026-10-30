package prismshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/stretchr/testify/require"
)

// The 20x20x4 plate carries an 8x6 boss over z in [3, 8], t = 1. The outer
// solid holds 1600 + 192; the cavity is the 18x18x2 lower prism, the 6x4x4
// upper prism and the root void at distance more than t from the root edge,
// 8·6·1 − 14π/2 + 8/3 = 152/3 − 7π.
func TestPrismUnionBossShellTrueVolume(t *testing.T) {
	t.Parallel()
	got := prismshell.PrismUnionBossShellTrueVolume(
		prismshell.BossRect{U0: -10, V0: -10, U1: 10, V1: 10},
		prismshell.BossRect{U0: -4, V0: -3, U1: 4, V1: 3},
		0, 4, 3, 8, 1, 1, 1)
	want := 1792 - (648 + 96 + 152.0/3 - 7*math.Pi)
	lo, _ := got.Lo.Float64()
	hi, _ := got.Hi.Float64()
	require.LessOrEqual(t, lo, hi)
	require.InDelta(t, want, (lo+hi)/2, (hi-lo)/2+1e-9*want)
}
