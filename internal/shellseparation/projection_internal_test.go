package shellseparation

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectionChargesMeshBound(t *testing.T) {
	t.Parallel()

	shell := Shell{
		Verts:     []r3.Vec{{X: 0, Y: 1, Z: 0}, {X: 0, Y: 3, Z: 2}},
		MeshBound: 0.5,
	}
	within := func(q *big.Rat, want *big.Rat, below bool) {
		t.Helper()
		slack := big.NewRat(1, 1_000_000_000)
		if below {
			// The lower end rounds outward: at most want, within 1e-9 of it.
			require.LessOrEqual(t, q.Cmp(want), 0)
			require.GreaterOrEqual(t, q.Cmp(new(big.Rat).Sub(want, slack)), 0)
			return
		}
		require.GreaterOrEqual(t, q.Cmp(want), 0)
		require.LessOrEqual(t, q.Cmp(new(big.Rat).Add(want, slack)), 0)
	}

	t.Run("y direction", func(t *testing.T) {
		t.Parallel()
		lo, hi, ok := projection(shell, ratOne(), ratZero(), 1)
		require.True(t, ok)
		within(lo, big.NewRat(1, 2), true)
		within(hi, big.NewRat(7, 2), false)
	})
	t.Run("z direction", func(t *testing.T) {
		t.Parallel()
		lo, hi, ok := projection(shell, ratZero(), ratOne(), 1)
		require.True(t, ok)
		require.LessOrEqual(t, lo.Cmp(big.NewRat(-1, 2)), 0)
		require.GreaterOrEqual(t, hi.Cmp(big.NewRat(5, 2)), 0)
	})
	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		for name, bad := range map[string]Shell{
			"empty":          {MeshBound: 0.5},
			"negative bound": {Verts: shell.Verts, MeshBound: -1},
			"nan vertex": {
				Verts:     []r3.Vec{{X: 0, Y: math.NaN(), Z: 0}},
				MeshBound: 0.5,
			},
		} {
			_, _, ok := projection(bad, ratOne(), ratZero(), 1)
			require.False(t, ok, name)
		}
	})
}
