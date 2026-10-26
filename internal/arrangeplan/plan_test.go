package arrangeplan_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/arrangeplan"
	"github.com/stretchr/testify/require"
)

func TestPackFillsAShelfThenStartsAnother(t *testing.T) {
	t.Parallel()
	items := []arrangeplan.Item{{Width: 10, Depth: 5}, {Width: 10, Depth: 8}, {Width: 10, Depth: 5}, {Width: 10, Depth: 3}}
	got, ok := arrangeplan.Pack(items, arrangeplan.Bed{Width: 36, Depth: 40, Gap: 2})
	require.True(t, ok)
	// Deepest first: the 8 deep item opens the shelf and the first 5 deep one
	// follows it. The second 5 deep one would cross x = 36, so it starts a
	// shelf 8 + 2 above the first, and the 3 deep one sits beside it.
	require.Equal(t, [][2]float64{{14, 2}, {2, 2}, {2, 12}, {14, 12}}, got)
}

func TestPackRefusesAnItemThatDoesNotFit(t *testing.T) {
	t.Parallel()
	cases := map[string]struct {
		items []arrangeplan.Item
		bed   arrangeplan.Bed
	}{
		"too wide":        {[]arrangeplan.Item{{Width: 20, Depth: 5}}, arrangeplan.Bed{Width: 22, Depth: 40, Gap: 2}},
		"too deep":        {[]arrangeplan.Item{{Width: 5, Depth: 20}}, arrangeplan.Bed{Width: 40, Depth: 22, Gap: 2}},
		"runs out of bed": {[]arrangeplan.Item{{Width: 10, Depth: 10}, {Width: 10, Depth: 10}}, arrangeplan.Bed{Width: 14, Depth: 24, Gap: 2}},
	}
	for name, tc := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			_, ok := arrangeplan.Pack(tc.items, tc.bed)
			require.False(t, ok)
		})
	}
}

func TestPackKeepsEveryGap(t *testing.T) {
	t.Parallel()
	items := make([]arrangeplan.Item, 0, 12)
	for i := range 12 {
		items = append(items, arrangeplan.Item{Width: float64(3 + i%4), Depth: float64(2 + i%5)})
	}
	bed := arrangeplan.Bed{Width: 40, Depth: 40, Gap: 1}
	got, ok := arrangeplan.Pack(items, bed)
	require.True(t, ok)
	for i, a := range items {
		require.GreaterOrEqual(t, got[i][0], bed.Gap)
		require.GreaterOrEqual(t, got[i][1], bed.Gap)
		require.LessOrEqual(t, got[i][0]+a.Width+bed.Gap, bed.Width)
		require.LessOrEqual(t, got[i][1]+a.Depth+bed.Gap, bed.Depth)
		for j := i + 1; j < len(items); j++ {
			b := items[j]
			apartX := got[i][0]+a.Width+bed.Gap <= got[j][0] || got[j][0]+b.Width+bed.Gap <= got[i][0]
			apartY := got[i][1]+a.Depth+bed.Gap <= got[j][1] || got[j][1]+b.Depth+bed.Gap <= got[i][1]
			require.True(t, apartX || apartY, "items %d and %d keep a gap", i, j)
		}
	}
}
