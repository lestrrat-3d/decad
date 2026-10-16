package prismshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/stretchr/testify/require"
)

func TestBossShellChecks(t *testing.T) {
	t.Run("exact rectangle", func(t *testing.T) {
		rect := prismshell.BossRect{U0: 1, V0: 2, U1: 5, V1: 8}
		require.True(t, prismshell.BossRectArithmeticExact(rect, 1))
		require.False(t, prismshell.BossRectArithmeticExact(rect, math.SmallestNonzeroFloat64))
	})

	t.Run("scaled wall room", func(t *testing.T) {
		require.True(t, prismshell.PrismUnionBossShellRoom(4, 1, 1))
		require.False(t, prismshell.PrismUnionBossShellRoom(2, 1, 1))
	})

	t.Run("strict contained bound", func(t *testing.T) {
		require.True(t, prismshell.PrismUnionBossShellInside(5, 1, 0, 10))
		require.False(t, prismshell.PrismUnionBossShellInside(1, 1, 0, 10))
		require.False(t, prismshell.PrismUnionBossShellInside(9, 1, 0, 10))
	})

	t.Run("complete cap", func(t *testing.T) {
		corners := [4]int{0, 1, 2, 3}
		require.True(t, prismshell.PrismUnionBossShellCompleteCap(corners,
			[][3]int{{0, 1, 2}, {0, 2, 3}}))
		require.False(t, prismshell.PrismUnionBossShellCompleteCap(corners,
			[][3]int{{0, 1, 2}, {0, 1, 3}}))
		require.False(t, prismshell.PrismUnionBossShellCompleteCap(corners,
			[][3]int{{0, 1, 2}, {0, 2, 4}}))
	})
}
