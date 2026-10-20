package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func TestCapOutsideHeelCone(t *testing.T) {
	t.Parallel()

	zero, one := new(big.Rat), big.NewRat(1, 1)
	near := big.NewRat(10000000000000001, 10000000000000000)
	m := loftface.Map{
		Constant: loftface.RatVec{zero, zero, zero},
		Q0:       loftface.RatVec{big.NewRat(10, 1), zero, zero},
		QU:       loftface.RatVec{one, near, zero},
		QV:       loftface.RatVec{zero, zero, big.NewRat(3, 1)},
		G0:       one, GU: zero, GV: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &facetedrecord.PointConeLimit{Apex: big.NewRat(10, 1), Far: zero, Slope: one}
	require.True(t, facetedrecord.CapOutsideHeelCone(m, box, cone),
		"the exact positive quadratic with a small radial margin was refused")
	changed := m
	changed.QU[1] = one
	require.False(t, facetedrecord.CapOutsideHeelCone(changed, box, cone), "a cap that touches the cone at v=0 was admitted")
	changed = m
	changed.QV[1] = one
	require.False(t, facetedrecord.CapOutsideHeelCone(changed, box, cone), "an uncertified cross term was admitted")
	box[1] = one
	require.False(t, facetedrecord.CapOutsideHeelCone(m, box, cone), "a source rectangle containing the quadratic zero was admitted")
	box = [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	box[0] = one
	require.True(t, facetedrecord.CapOutsideHeelCone(m, box, cone), "a rectangle wholly above zero may still be outside")

	onePast := [4]*big.Rat{zero, one, big.NewRat(-1, 1), one}
	require.False(t, facetedrecord.CapOutsideHeelCone(m, onePast, cone), "a rectangle starting at zero touches the zero")
	atZero := [4]*big.Rat{big.NewRat(-1, 1), zero, big.NewRat(-1, 1), one}
	require.False(t, facetedrecord.CapOutsideHeelCone(m, atZero, cone), "a rectangle ending at zero touches the zero")
	below := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	require.True(t, facetedrecord.CapOutsideHeelCone(m, below, cone), "a rectangle wholly below zero may be outside")
}

func TestCapOutsideToeCone(t *testing.T) {
	t.Parallel()

	zero, one := new(big.Rat), big.NewRat(1, 1)
	m := loftface.Map{
		Constant: loftface.RatVec{zero, zero, zero},
		Q0:       loftface.RatVec{big.NewRat(5, 1), big.NewRat(10, 1), zero},
		QU:       loftface.RatVec{zero, zero, zero},
		QV:       loftface.RatVec{zero, zero, zero},
		G0:       one, GU: zero, GV: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &facetedrecord.PointConeLimit{Apex: big.NewRat(8, 1), Far: zero, Slope: one}
	require.True(t, facetedrecord.CapOutsideToeCone(m, box, cone), "the cap strictly outside the toe cone was refused")

	inside := m
	inside.Q0[1] = big.NewRat(2, 1)
	require.False(t, facetedrecord.CapOutsideToeCone(inside, box, cone), "the cap inside the toe cone was admitted")

	atFar := m
	atFar.Q0[0] = zero
	require.False(t, facetedrecord.CapOutsideToeCone(atFar, box, cone), "a cap reaching the far plane was admitted")

	steep := &facetedrecord.PointConeLimit{Apex: big.NewRat(8, 1), Far: zero, Slope: big.NewRat(1, 100)}
	require.False(t, facetedrecord.CapOutsideToeCone(m, box, steep), "a thin cone leaves the margin below zero")

	exact := &facetedrecord.PointConeLimit{Apex: big.NewRat(5, 1), Far: zero, Slope: big.NewRat(1, 1)}
	touching := m
	touching.Q0[1] = zero
	require.False(t, facetedrecord.CapOutsideToeCone(touching, box, exact), "touching the cone is not strictly outside")
}

func TestCapOutsideToeConeMargin(t *testing.T) {
	t.Parallel()

	zero, one := new(big.Rat), big.NewRat(1, 1)
	box := [4]*big.Rat{big.NewRat(-3, 1), one, big.NewRat(-1, 1), one}
	mapOf := func(q0, qu loftface.RatVec) loftface.Map {
		return loftface.Map{Constant: loftface.RatVec{zero, zero, zero}, Q0: q0,
			QU: qu, QV: loftface.RatVec{zero, zero, zero}, G0: one, GU: zero, GV: zero}
	}
	cone := func(apex, far, slope int64) *facetedrecord.PointConeLimit {
		return &facetedrecord.PointConeLimit{Apex: big.NewRat(apex, 1), Far: big.NewRat(far, 1), Slope: big.NewRat(slope, 1)}
	}
	flat := loftface.RatVec{zero, zero, zero}

	t.Run("the slope enters squared", func(t *testing.T) {
		t.Parallel()
		m := mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(2, 1), zero}, flat)
		require.False(t, facetedrecord.CapOutsideToeCone(m, box, cone(8, 0, 1)), "radial 4 against distance 3")
		require.True(t, facetedrecord.CapOutsideToeCone(m, box, cone(8, 0, 2)), "16 against 9")
	})
	t.Run("the farthest distance along the axis limits it", func(t *testing.T) {
		t.Parallel()
		// x runs over [-1, 7] across the box, so the apex distance runs over [1, 9].
		m := mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(7, 1), zero}, loftface.RatVec{big.NewRat(2, 1), zero, zero})
		require.False(t, facetedrecord.CapOutsideToeCone(m, box, cone(8, -5, 1)), "49 against 81")
		m = mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(12, 1), zero}, loftface.RatVec{big.NewRat(2, 1), zero, zero})
		require.True(t, facetedrecord.CapOutsideToeCone(m, box, cone(8, -5, 1)), "144 against 81")
		require.False(t, facetedrecord.CapOutsideToeCone(m, box, cone(-5, -5, 1)),
			"the distance is measured from the apex, not the far plane")
	})
	t.Run("both radial axes count", func(t *testing.T) {
		t.Parallel()
		onlyY := mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(4, 1), zero}, flat)
		onlyZ := mapOf(loftface.RatVec{big.NewRat(5, 1), zero, big.NewRat(4, 1)}, flat)
		both := mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(2, 1), big.NewRat(2, 1)}, flat)
		c := cone(8, 0, 1)
		require.True(t, facetedrecord.CapOutsideToeCone(onlyY, box, c), "16 against 9 on y alone")
		require.True(t, facetedrecord.CapOutsideToeCone(onlyZ, box, c), "16 against 9 on z alone")
		require.False(t, facetedrecord.CapOutsideToeCone(both, box, c), "4 + 4 against 9")
		both = mapOf(loftface.RatVec{big.NewRat(5, 1), big.NewRat(3, 1), big.NewRat(3, 1)}, flat)
		require.True(t, facetedrecord.CapOutsideToeCone(both, box, c), "9 + 9 against 9")
	})
}

func TestUniqueSourcePatches(t *testing.T) {
	t.Parallel()

	payload := func(src ...int) facetedrecord.Payload {
		p := facetedrecord.Payload{Src: src, Tris: make([][3]int, len(src))}
		return p
	}
	// Facets 0 and 1 are neighbours, as are 2 and 3; facet 4 stands alone.
	adj := [][]int{{1}, {0}, {3}, {2}, {}}

	tests := []struct {
		name   string
		src    []int
		adj    [][]int
		groups []int
		want   bool
	}{
		{"each group is one connected patch", []int{0, 0, 1, 1, 2}, adj, []int{0, 1}, true},
		{"one group alone", []int{0, 0, 1, 1, 2}, adj, []int{2}, true},
		{"a group split across two patches", []int{0, 0, 0, 1, 2}, adj, []int{0}, false},
		{"a group with no facet", []int{0, 0, 1, 1, 2}, adj, []int{7}, false},
		{"a later group split", []int{0, 0, 1, 1, 1}, adj, []int{0, 1}, false},
		{"no groups asked", []int{0, 0, 1, 1, 2}, adj, nil, true},
		{"adjacency of the wrong length", []int{0, 0, 1, 1, 2}, adj[:4], []int{0}, false},
		{"sources of the wrong length", []int{0, 0, 1, 1}, adj, []int{0}, false},
	}
	for _, tc := range tests {
		p := payload(tc.src...)
		if tc.name == "sources of the wrong length" {
			p.Tris = make([][3]int, 5)
		}
		got, err := facetedrecord.UniqueSourcePatches(t.Context(), p, tc.adj, tc.groups...)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}
