package planar

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

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
