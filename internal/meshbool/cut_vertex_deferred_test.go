package meshbool

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDeferredIndexPromotesRoundedCollisions(t *testing.T) {
	var scratch cutProjectionScratch
	tc := newTriCutter(t.Context(), &scratch)
	ref := &refCutter{}
	w := new(big.Int).Lsh(big.NewInt(1), 120)
	for round := range 2 {
		for i := range 64 {
			n := new(big.Int).Add(w, big.NewInt(int64(i)))
			weight := new(big.Int).Set(w)
			if round == 1 {
				n.Mul(n, big.NewInt(3))
				weight.Mul(weight, big.NewInt(3))
			}
			p := proof.Xpt{X: n, Y: new(big.Int), Z: new(big.Int), W: weight}
			boundary := round == 1
			p2 := tc.project(p, 0, 1)
			require.Equal(t, math.Float64bits(1), math.Float64bits(p2.Fu), "point %d", i)
			want := ref.addVert(NewXP2FromXpt(p, 0, 1), p, boundary)
			got := tc.AddVert(p2, p, boundary)
			require.Equal(t, i, want)
			require.Equal(t, want, got, "round %d point %d", round, i)
		}
	}
	require.Len(t, tc.Verts, 64)
	require.Len(t, tc.index.promoted, 1)
	for i := range ref.verts {
		requireSameVert(t, ref.verts[i], tc.Verts[i], fmt.Sprintf("point %d", i))
	}
	// A non-reduced manual rational keeps its textual identity even inside
	// a bucket that already contains the same mathematical coordinate.
	p2 := NewXP2(nonReducedRat(3, 3), new(big.Rat))
	p3 := proof.XptOf(r3.Vec{X: 1})
	want := ref.addVert(p2, p3, false)
	require.Equal(t, 64, want)
	require.Equal(t, want, tc.AddVert(p2, p3, false))
	require.Equal(t, want, tc.AddVert(NewXP2(nonReducedRat(3, 3), new(big.Rat)), p3, true))
	require.True(t, tc.Verts[want].Boundary)
	newTriCutter(t.Context(), &scratch)
	require.Empty(t, tc.index.promoted)
	require.Empty(t, tc.index.exactHead)
	require.Empty(t, tc.index.next)
}
