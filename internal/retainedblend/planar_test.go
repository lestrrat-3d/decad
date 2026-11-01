package retainedblend

import (
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestEarQualityMatchesLoopImplementation(t *testing.T) {
	t.Parallel()
	inputs := []struct {
		name    string
		a, b, c pt2
	}{
		{name: "ordinary", a: pt2{0, 0}, b: pt2{3, 0}, c: pt2{0, 4}},
		{name: "collinear", a: pt2{-2, 1}, b: pt2{0, 1}, c: pt2{5, 1}},
		{name: "coincident", a: pt2{2, -3}, b: pt2{2, -3}, c: pt2{2, -3}},
		{name: "signed zero", a: pt2{math.Copysign(0, -1), 0}, b: pt2{0, math.Copysign(0, -1)}, c: pt2{0, 0}},
		{name: "tiny", a: pt2{0, 0}, b: pt2{1e-150, 0}, c: pt2{0, 1e-150}},
		{name: "large", a: pt2{0, 0}, b: pt2{1e150, 0}, c: pt2{0, 1e150}},
		{name: "infinite", a: pt2{0, 0}, b: pt2{math.Inf(1), 1}, c: pt2{2, 3}},
		{name: "nan", a: pt2{0, 0}, b: pt2{math.NaN(), 1}, c: pt2{2, 3}},
		{name: "nan payload", a: pt2{0, 0}, b: pt2{math.Float64frombits(0x7ff8000000000001), 1}, c: pt2{2, 3}},
	}
	for _, input := range inputs {
		t.Run(input.name, func(t *testing.T) {
			got := earQuality(input.a, input.b, input.c)
			want := earQualityLoopImplementation(input.a, input.b, input.c)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
		})
	}
}

func earQualityLoopImplementation(a, b, c pt2) float64 {
	area2 := math.Abs((b.u-a.u)*(c.w-a.w) - (b.w-a.w)*(c.u-a.u))
	long := 0.0
	for _, d := range [][2]pt2{{a, b}, {b, c}, {c, a}} {
		du, dw := d[1].u-d[0].u, d[1].w-d[0].w
		long = math.Max(long, du*du+dw*dw)
	}
	if long == 0 {
		return 0
	}
	return area2 / long
}

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

func TestSortPlanarEarsPutsNaNLast(t *testing.T) {
	ears := []planarEar{
		{at: 2, quality: 1},
		{at: 5, quality: math.NaN()},
		{at: 1, quality: math.NaN()},
		{at: 0, quality: 1},
		{at: 3, quality: 2},
		{at: 4, quality: math.Inf(-1)},
	}

	sortPlanarEars(ears)

	got := make([]int, len(ears))
	for i, e := range ears {
		got[i] = e.at
	}
	require.Equal(t, []int{3, 0, 2, 4, 1, 5}, got)
}

func TestBestPlanarEarMatchesSortedFirst(t *testing.T) {
	t.Parallel()
	inputs := [][]planarEar{
		{{at: 3, quality: 1}, {at: 1, quality: 2}, {at: 0, quality: 1}, {at: 2, quality: 2}},
		{{at: 5, quality: math.Inf(-1)}, {at: 4, quality: math.Inf(1)}, {at: 7, quality: math.Inf(-1)}, {at: 6, quality: math.Inf(1)}},
		{{at: 2, quality: 1}, {at: 5, quality: math.NaN()}, {at: 1, quality: math.NaN()}, {at: 0, quality: 1}},
	}
	for _, input := range inputs {
		sorted := slices.Clone(input)
		sortPlanarEars(sorted)
		best, ok := bestPlanarEar(input)
		require.True(t, ok)
		require.Equal(t, sorted[0], best)
	}
	_, ok := bestPlanarEar(nil)
	require.False(t, ok)
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

func TestEarBlockedMatchesUnscreenedPredicate(t *testing.T) {
	t.Parallel()
	ps := []pt2{
		{0, 0}, {4, 0}, {0, 4},
		{1, 1}, {-1, 1}, {5, 1}, {1, -1}, {1, 5},
		{2, 0}, {2, 2}, {0, 2}, {0, 0},
		{2, math.Nextafter(2, 0)},
		{1, 0}, {2, 0}, {3, 0},
	}
	p := pt2{2, math.Nextafter(2, 0)}
	left := (ps[2].u - ps[1].u) * (p.w - ps[1].w)
	right := (ps[2].w - ps[1].w) * (p.u - ps[1].u)
	require.LessOrEqual(t, math.Abs(left-right), 8*0x1p-53*(math.Abs(left)+math.Abs(right)))
	idx := make([]int, len(ps))
	for i := range ps {
		idx[i] = i
	}
	require.Equal(t, earBlockedUnscreened(ps, idx, 0, 1, 2), earBlocked(ps, idx, 0, 1, 2))

	for _, point := range []pt2{
		{1, 1}, {-1, 1}, {5, 1}, {1, -1}, {1, 5},
		{2, 0}, {2, 2}, {0, 2}, {0, 0},
		{2, math.Nextafter(2, 0)},
	} {
		points := []pt2{{0, 0}, {4, 0}, {0, 4}, point}
		got := earBlocked(points, []int{0, 1, 2, 3}, 0, 1, 2)
		want := earBlockedUnscreened(points, []int{0, 1, 2, 3}, 0, 1, 2)
		require.Equal(t, want, got, "point (%g, %g)", point.u, point.w)
	}
}

func TestEarBlockedPreservesNonFiniteFallback(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		ps   []pt2
	}{
		{"NaN vertex", []pt2{{0, 0}, {4, 0}, {0, 4}, {math.NaN(), 1}}},
		{"infinite vertex", []pt2{{0, 0}, {4, 0}, {0, 4}, {math.Inf(1), 1}}},
		{"NaN corner", []pt2{{0, 0}, {math.NaN(), 0}, {0, 4}, {1, 1}}},
		{"infinite corner", []pt2{{0, 0}, {math.Inf(1), 0}, {0, 4}, {1, 1}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			idx := []int{0, 1, 2, 3}
			require.Panics(t, func() { earBlockedUnscreened(tc.ps, idx, 0, 1, 2) })
			require.Panics(t, func() { earBlocked(tc.ps, idx, 0, 1, 2) })
		})
	}
}

func TestTriangulateRegionMatchesUnscreenedEarClip(t *testing.T) {
	t.Parallel()
	fixtures := []struct {
		name  string
		ps    []pt2
		outer []int
		holes [][]int
	}{
		{
			name: "bridged hole with collinear runs",
			ps: []pt2{
				{0, 0}, {5, 0}, {10, 0}, {10, 5}, {10, 10}, {5, 10}, {0, 10}, {0, 5},
				{3, 3}, {3, 7}, {7, 7}, {7, 3},
			},
			outer: []int{0, 1, 2, 3, 4, 5, 6, 7}, holes: [][]int{{8, 9, 10, 11}},
		},
		{
			name:  "near-collinear boundary run",
			ps:    []pt2{{0, 0}, {2, 3e-15}, {4, -3e-15}, {6, 3e-15}, {8, -3e-15}, {10, 0}, {5, 8}},
			outer: []int{2, 3, 4, 5, 6, 0, 1},
		},
	}
	for _, fixture := range fixtures {
		t.Run(fixture.name, func(t *testing.T) {
			want, err := triangulateRegionUnscreened(fixture.ps, fixture.outer, fixture.holes)
			require.NoError(t, err)
			got, err := triangulateRegion(fixture.ps, fixture.outer, fixture.holes)
			require.NoError(t, err)
			require.Equal(t, want, got)
		})
	}
}

func earBlockedUnscreened(ps []pt2, idx []int, ia, ib, ic int) bool {
	a, b, c := ps[ia], ps[ib], ps[ic]
	for _, v := range idx {
		p := ps[v]
		if samePt(p, a) || samePt(p, b) || samePt(p, c) {
			continue
		}
		if orient(a, b, p) >= 0 && orient(b, c, p) >= 0 && orient(c, a, p) >= 0 {
			return true
		}
	}
	return false
}

func triangulateRegionUnscreened(ps []pt2, outer []int, holes [][]int) ([][3]int, error) {
	merged := slices.Clone(outer)
	remaining := slices.Clone(holes)
	for len(remaining) > 0 {
		var err error
		merged, remaining, err = bridgeNearest(ps, merged, remaining)
		if err != nil {
			return nil, err
		}
	}
	tris, err := earClipUnscreened(ps, merged)
	if err != nil {
		return nil, err
	}
	return flipSlivers(ps, tris), nil
}

func earClipUnscreened(ps []pt2, poly []int) ([][3]int, error) {
	idx := slices.Clone(poly)
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		n := len(idx)
		var ears []planarEar
		for i := range n {
			a, b, c := ps[idx[(i+n-1)%n]], ps[idx[i]], ps[idx[(i+1)%n]]
			if orient(a, b, c) > 0 {
				ears = append(ears, planarEar{i, earQuality(a, b, c)})
			}
		}
		best, ok := bestPlanarEar(ears)
		if !ok {
			return nil, fmt.Errorf("no ear")
		}
		ia, ib, ic := idx[(best.at+n-1)%n], idx[best.at], idx[(best.at+1)%n]
		if earBlockedUnscreened(ps, idx, ia, ib, ic) {
			sortPlanarEars(ears)
			found := false
			for _, e := range ears[1:] {
				ia, ib, ic = idx[(e.at+n-1)%n], idx[e.at], idx[(e.at+1)%n]
				if earBlockedUnscreened(ps, idx, ia, ib, ic) {
					continue
				}
				best, found = e, true
				break
			}
			if !found {
				return nil, fmt.Errorf("no ear")
			}
		}
		tris = append(tris, [3]int{ia, ib, ic})
		idx = append(idx[:best.at], idx[best.at+1:]...)
	}
	if orient(ps[idx[0]], ps[idx[1]], ps[idx[2]]) <= 0 {
		return nil, fmt.Errorf("degenerate final triangle")
	}
	return append(tris, [3]int{idx[0], idx[1], idx[2]}), nil
}
