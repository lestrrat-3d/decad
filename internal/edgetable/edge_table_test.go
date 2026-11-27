package edgetable_test

// CI: isolated-tests

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/edgetable"
	"github.com/stretchr/testify/require"
)

// edgeKeyPool mixes ordinary indices with values at and beyond the flat
// table's range limits, so every edge class is exercised.
var edgeKeyPool = []int{
	0, 1, 2, 3, 7, 100, 1 << 20, 1<<31 - 2, 1<<31 - 1, 1 << 31, 1 << 32, math.MaxInt, -1, -5, math.MinInt,
}

func TestEdgeTableDifferential(t *testing.T) {
	for seed := range uint64(40) {
		r := rand.New(rand.NewPCG(seed, 0xed9e))
		ref := map[[2]int]int{}
		tab := edgetable.New(r.IntN(4) * 3) // often too small, forcing growth
		small := 8 + int(seed)*5            // vertex range, so duplicates are common
		pick := func() int {
			if r.IntN(10) == 0 {
				return edgeKeyPool[r.IntN(len(edgeKeyPool))]
			}
			return r.IntN(small)
		}
		for step := range 4000 {
			a, b := pick(), pick()
			switch op := r.IntN(10); {
			case op < 6:
				v := r.IntN(1000)
				switch r.IntN(20) {
				case 0:
					v = math.MaxInt32 + 1 + r.IntN(5) // does not fit int32
				case 1:
					v = -1 - r.IntN(5)
				}
				ref[[2]int{a, b}] = v
				tab.Set(a, b, v)
			case op < 8:
				_, had := ref[[2]int{a, b}]
				delete(ref, [2]int{a, b})
				require.Equal(t, had, tab.Delete(a, b), "seed %d step %d delete", seed, step)
			default:
				want, wok := ref[[2]int{a, b}]
				got, gok := tab.Get(a, b)
				require.Equal(t, wok, gok, "seed %d step %d presence", seed, step)
				require.Equal(t, want, got, "seed %d step %d value", seed, step)
			}
			require.Equal(t, len(ref), tab.Len(), "seed %d step %d len", seed, step)
		}
		for k, want := range ref {
			got, ok := tab.Get(k[0], k[1])
			require.True(t, ok)
			require.Equal(t, want, got)
		}
		// Absent edges stay absent, including the reverse of every present edge.
		for k := range ref {
			if _, ok := ref[[2]int{k[1], k[0]}]; ok {
				continue
			}
			_, ok := tab.Get(k[1], k[0])
			require.False(t, ok)
		}
	}
}

func TestEdgeTableGrowthAndDeleteAll(t *testing.T) {
	tab := edgetable.New(0)
	const n = 20000
	for i := range n {
		tab.Set(i, i+1, i)
		tab.Set(i, i+1, i+1) // overwrite: last write wins
	}
	require.Equal(t, n, tab.Len())
	for i := range n {
		v, ok := tab.Get(i, i+1)
		require.True(t, ok)
		require.Equal(t, i+1, v)
		_, ok = tab.Get(i+1, i)
		require.False(t, ok)
	}
	for i := range n {
		require.True(t, tab.Delete(i, i+1))
		require.False(t, tab.Delete(i, i+1))
	}
	require.Equal(t, 0, tab.Len())
	tab.Set(5, 6, 7)
	v, ok := tab.Get(5, 6)
	require.True(t, ok)
	require.Equal(t, 7, v)
}

func TestEdgeTableNilAndSentinel(t *testing.T) {
	var nilTab *edgetable.Table
	_, ok := nilTab.Get(0, 1)
	require.False(t, ok)
	require.Equal(t, 0, nilTab.Len())

	// The largest in-range edge packs to 0x7fffffff7fffffff, not the empty
	// marker, and the all-ones pattern a=-1,b=-1 goes to the side map.
	tab := edgetable.New(1)
	tab.Set(1<<31-1, 1<<31-1, 4)
	tab.Set(-1, -1, 5)
	v, ok := tab.Get(1<<31-1, 1<<31-1)
	require.True(t, ok)
	require.Equal(t, 4, v)
	v, ok = tab.Get(-1, -1)
	require.True(t, ok)
	require.Equal(t, 5, v)
	_, ok = tab.Get(0, 0)
	require.False(t, ok)
}

func TestEdgeTableClear(t *testing.T) {
	tab := edgetable.New(4)
	for i := range 100 {
		tab.Set(i, i+1, i)
	}
	tab.Set(-1, 2, 5) // side map entry
	tab.Clear()
	require.Equal(t, 0, tab.Len())
	_, ok := tab.Get(3, 4)
	require.False(t, ok)
	_, ok = tab.Get(-1, 2)
	require.False(t, ok)
	tab.Set(3, 4, 9)
	v, ok := tab.Get(3, 4)
	require.True(t, ok)
	require.Equal(t, 9, v)
	require.Equal(t, 1, tab.Len())
}
