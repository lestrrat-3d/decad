package planar

import (
	"math"
	"math/rand/v2"
	"slices"
	"strconv"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestBoxTreeNodeCount(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		n    int
		want int
	}{
		{n: 0, want: 0},
		{n: 1, want: 1},
		{n: boxTreeLeaf, want: 1},
		{n: boxTreeLeaf + 1, want: 3},
		{n: 41, want: 15},
	} {
		require.Equal(t, tc.want, boxTreeNodeCount(tc.n), "n=%d", tc.n)
	}
}

func TestBoxTreeCentersSanitizeNonFiniteValues(t *testing.T) {
	t.Parallel()
	boxes := []floatBox{
		{lo: [3]float64{-2, math.Inf(1), math.Inf(-1)}, hi: [3]float64{4, 3, 1}},
		{lo: [3]float64{math.NaN(), math.MaxFloat64, 2}, hi: [3]float64{5, math.MaxFloat64, 6}},
		{lo: [3]float64{-3, 6, -math.MaxFloat64}, hi: [3]float64{3, 10, math.MaxFloat64}},
	}
	tree := newBoxTree(boxes)
	require.Equal(t, [][3]float64{{1, 0, 0}, {0, 0, 4}, {0, 8, 0}}, tree.centres)
	for i, box := range boxes {
		for axis := range 3 {
			require.Equal(t, boxCentre(box, axis), tree.centres[i][axis])
		}
	}
}

// TestBoxTreeQueryMatchesFullScan holds every query to the full scan with the
// floatApart pre-test: the same indices, in ascending order. The boxes cover
// clustered, overlapping, degenerate and non-finite cases.
//
// Legs shown to fail: a leaf dropping its last box loses indices the full scan
// keeps, and a query that skips its final sort returns them out of order.
func TestBoxTreeQueryMatchesFullScan(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	randBox := func(spread, size float64) floatBox {
		var b floatBox
		for axis := range 3 {
			c := rng.Float64() * spread
			w := rng.Float64() * size
			b.lo[axis], b.hi[axis] = c-w, c+w
		}
		return b
	}
	for trial := range 40 {
		n := 1 + rng.IntN(400)
		boxes := make([]floatBox, n)
		for i := range boxes {
			boxes[i] = randBox(10, 0.1+float64(trial%5))
			switch rng.IntN(30) {
			case 0:
				boxes[i].lo[0], boxes[i].hi[1] = math.Inf(-1), math.Inf(1)
			case 1:
				boxes[i].lo[2] = math.NaN()
			}
		}
		tree := newBoxTree(boxes)
		var got []int
		hits := 0
		for range 60 {
			q := randBox(10, 2)
			got = tree.query(q, got)
			var want []int
			for i, b := range boxes {
				if !floatApart(b, q) {
					want = append(want, i)
				}
			}
			require.True(t, slices.Equal(want, got), "trial %d: want %d indices, got %d", trial, len(want), len(got))
			hits += len(want)
		}
		require.Positive(t, hits, "premise: trial %d has hits", trial)
	}
	require.Empty(t, newBoxTree(nil).query(floatBox{}, nil))
}

func BenchmarkNewBoxTreeCold(b *testing.B) {
	for _, n := range []int{64, 512, 4096} {
		b.Run(strconv.Itoa(n), func(b *testing.B) {
			boxes := make([]floatBox, n)
			for i := range boxes {
				x := float64((i * 37) % n)
				y := float64((i * 53) % n)
				z := float64((i * 97) % n)
				w := float64(i%7) / 10
				boxes[i] = floatBox{
					lo: [3]float64{x - w, y - w, z - w},
					hi: [3]float64{x + w, y + w, z + w},
				}
			}
			b.ReportAllocs()
			b.ResetTimer()
			for range b.N {
				_ = newBoxTree(boxes)
			}
		})
	}
}
