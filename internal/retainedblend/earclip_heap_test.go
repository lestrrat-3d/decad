// CI: isolated-tests

package retainedblend

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestEarClipMatchesSortedOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	sizes := []struct{ count, maxN int }{{3000, 12}, {2000, 40}, {300, 150}, {30, 600}}
	for _, size := range sizes {
		for _, p := range earClipPolygons(rng, size.count, size.maxN) {
			want := runEarClip(earClipSorted, p.ps, p.poly)
			got := runEarClip(earClip, p.ps, p.poly)
			if !sameEarClipOutcome(want, got) {
				// Report scalars only: the outcomes hold whole triangle lists.
				require.FailNow(t, "earClip differs from the sorted oracle", "%s with %d vertices: oracle err=%q "+
					"triangles=%d panic=%v, heap err=%q triangles=%d panic=%v", p.name, len(p.poly), want.err,
					len(want.tris), want.panicked, got.err, len(got.tris), got.panicked)
			}
		}
	}
}

func TestEarClipMatchesSortedOracleOnBlockedRuns(t *testing.T) {
	t.Parallel()
	// A comb whose teeth sit under one flat roof blocks most ears for most of the run.
	for _, teeth := range []int{2, 5, 40, 200} {
		var ps []pt2
		for i := range teeth {
			ps = append(ps, pt2{float64(2 * i), 0})
		}
		ps = append(ps, pt2{float64(2 * teeth), 0})
		for i := teeth - 1; i >= 0; i-- {
			x := float64(2*i) + 1
			ps = append(ps, pt2{x + 1, 5}, pt2{x, 5}, pt2{x, 1 + float64(i%3)*0.5})
		}
		poly := identityLoop(len(ps))
		want := runEarClip(earClipSorted, ps, poly)
		got := runEarClip(earClip, ps, poly)
		require.True(t, sameEarClipOutcome(want, got), "comb with %d teeth", teeth)
	}
}

func TestEarClipMatchesSortedOracleOnNonFinitePoints(t *testing.T) {
	t.Parallel()
	nan, inf := math.NaN(), math.Inf(1)
	rng := rand.New(rand.NewPCG(3, 5))
	for range 400 {
		p := earClipPolygons(rng, 1, 14)[0]
		p.ps[rng.IntN(len(p.ps))] = []pt2{{nan, 0}, {0, inf}, {inf, -inf}, {nan, nan}}[rng.IntN(4)]
		want := runEarClip(earClipSorted, p.ps, p.poly)
		got := runEarClip(earClip, p.ps, p.poly)
		require.True(t, sameEarClipOutcome(want, got), "%s with %d vertices", p.name, len(p.poly))
	}
}

func TestEarClipRefusalsMatchSortedOracle(t *testing.T) {
	t.Parallel()
	cases := map[string]struct {
		ps   []pt2
		poly []int
	}{
		"clockwise square":  {[]pt2{{0, 0}, {0, 1}, {1, 1}, {1, 0}}, []int{0, 1, 2, 3}},
		"collinear":         {[]pt2{{0, 0}, {1, 0}, {2, 0}, {3, 0}}, []int{0, 1, 2, 3}},
		"triangle":          {[]pt2{{0, 0}, {1, 0}, {0, 1}}, []int{0, 1, 2}},
		"degenerate triple": {[]pt2{{0, 0}, {1, 0}, {2, 0}}, []int{0, 1, 2}},
		"too short":         {[]pt2{{0, 0}, {1, 0}}, []int{0, 1}},
		"repeated vertex":   {[]pt2{{0, 0}, {2, 0}, {1, 1}, {2, 2}, {0, 2}}, []int{0, 1, 2, 2, 3, 4}},
	}
	for name, c := range cases {
		want := runEarClip(earClipSorted, c.ps, c.poly)
		got := runEarClip(earClip, c.ps, c.poly)
		require.True(t, sameEarClipOutcome(want, got), name)
	}
}
