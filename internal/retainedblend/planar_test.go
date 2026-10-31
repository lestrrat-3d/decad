package retainedblend

import (
	"math"
	"sort"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestTriangulateRegionKeepsEveryLoopVertex(t *testing.T) {
	t.Parallel()
	// A 10x10 square with a collinear vertex on each side, and a square hole.
	ps := []pt2{
		{0, 0}, {5, 0}, {10, 0}, {10, 5}, {10, 10}, {5, 10}, {0, 10}, {0, 5}, // outer, counter-clockwise
		{3, 3}, {3, 7}, {7, 7}, {7, 3}, // hole, clockwise
	}
	outer := []int{0, 1, 2, 3, 4, 5, 6, 7}
	hole := []int{8, 9, 10, 11}
	tris, err := triangulateRegion(ps, outer, [][]int{hole})
	require.NoError(t, err)

	used := map[int]bool{}
	area := 0.0
	for _, tri := range tris {
		for _, v := range tri {
			used[v] = true
		}
		require.Positive(t, orient(ps[tri[0]], ps[tri[1]], ps[tri[2]]), "triangle %v is counter-clockwise", tri)
		a, b, c := ps[tri[0]], ps[tri[1]], ps[tri[2]]
		area += math.Abs((b.u-a.u)*(c.w-a.w)-(b.w-a.w)*(c.u-a.u)) / 2
	}
	for v := range ps {
		require.True(t, used[v], "vertex %d appears in a triangle", v)
	}
	require.InDelta(t, 100-16, area, 1e-12)
}

func TestSortPlanarEarsUsesAtOrderForQualityTies(t *testing.T) {
	ears := []planarEar{
		{at: 3, quality: 1},
		{at: 1, quality: 2},
		{at: 0, quality: 1},
		{at: 2, quality: 2},
		{at: 5, quality: math.Inf(-1)},
		{at: 4, quality: math.Inf(1)},
		{at: 7, quality: math.Inf(-1)},
		{at: 6, quality: math.Inf(1)},
	}

	sortPlanarEars(ears)

	require.Equal(t, []planarEar{
		{at: 4, quality: math.Inf(1)},
		{at: 6, quality: math.Inf(1)},
		{at: 1, quality: 2},
		{at: 2, quality: 2},
		{at: 0, quality: 1},
		{at: 3, quality: 1},
		{at: 5, quality: math.Inf(-1)},
		{at: 7, quality: math.Inf(-1)},
	}, ears)
}

func TestSortPlanarEarsKeepsStableOrderWithNaN(t *testing.T) {
	ears := []planarEar{{at: 2, quality: 1}, {at: 1, quality: math.NaN()}, {at: 0, quality: 1}, {at: 3, quality: 2}}
	want := append([]planarEar(nil), ears...)
	sort.SliceStable(want, func(i, j int) bool { return want[i].quality > want[j].quality })

	sortPlanarEars(ears)

	for i := range ears {
		require.Equal(t, want[i].at, ears[i].at)
	}
}

func TestZipperJoinsRingsOfDifferentCounts(t *testing.T) {
	t.Parallel()
	ring := func(base, n int, offset float64) []ringPt {
		out := make([]ringPt, n)
		for k := range n {
			out[k] = ringPt{idx: base + k, ang: offset + 2*math.Pi*float64(k)/float64(n)}
		}
		return out
	}
	a, b := ring(0, 5, 0), ring(100, 8, 0.1)
	tris := zipper(a, b)
	require.Len(t, tris, 13, "one triangle per ring vertex")
	edges := map[[2]int]int{}
	for _, tri := range tris {
		for k := range 3 {
			edges[[2]int{min(tri[k], tri[(k+1)%3]), max(tri[k], tri[(k+1)%3])}]++
		}
	}
	// Every ring edge borders exactly one triangle; every other edge two.
	for i := range a {
		require.Equal(t, 1, edges[[2]int{min(a[i].idx, a[(i+1)%5].idx), max(a[i].idx, a[(i+1)%5].idx)}])
	}
	for i := range b {
		require.Equal(t, 1, edges[[2]int{min(b[i].idx, b[(i+1)%8].idx), max(b[i].idx, b[(i+1)%8].idx)}])
	}
}

func TestTriangulateRegionLeavesNoSliverOnACollinearRun(t *testing.T) {
	t.Parallel()
	// A triangle whose base holds six vertices that are collinear to within a
	// few ulps, the shape a Boolean rim leaves along a straight edge.
	const e = 3e-15
	ps := []pt2{
		{0, 0}, {2, e}, {4, -e}, {6, e}, {8, -e}, {10, 0}, {5, 8},
	}
	tris, err := triangulateRegion(ps, []int{2, 3, 4, 5, 6, 0, 1}, nil)
	require.NoError(t, err)
	used := map[int]bool{}
	for _, tri := range tris {
		for _, v := range tri {
			used[v] = true
		}
		require.GreaterOrEqual(t, earQuality(ps[tri[0]], ps[tri[1]], ps[tri[2]]), sliverQuality, "triangle %v", tri)
	}
	require.Len(t, used, len(ps), "every vertex appears")
}
