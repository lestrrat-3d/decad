// CI: isolated-tests

package retainedblend

import (
	"cmp"
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestBridgeFirstSortMatchesSortFunc checks that bridgeFirstSort leaves at index 0 exactly the candidate that
// slices.SortFunc leaves there, ties included.
//
// bridgeFirstSort is a copy of the Go 1.27.4 pdqsort steps. If a later Go toolchain changes slices.SortFunc so that
// it orders equal keys differently, this test fails. That failure means decad's bridge choice, and so the
// triangulation of faces with tied candidates, no longer matches the toolchain; keep the copy as it is, because the
// copy is what freezes today's results.
func TestBridgeFirstSortMatchesSortFunc(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(1, 2))
	for name, key := range bridgeKeyShapes(r) {
		for _, n := range sizesAroundPdqsortThresholds() {
			for rep := range 5 {
				data := bridgeKeyCandidates(n, key)
				if n == 0 {
					bridgeFirstSort(data)
					continue
				}
				want := sortedFirstInput(data)
				bridgeFirstSort(data)
				// Compare bounded scalars only; a failing run never dumps the slice.
				require.Equalf(t, want, data[0].m, "shape %q n=%d rep=%d", name, n, rep)
			}
		}
	}
}

// TestBridgeFirstSortMatchesSortFuncLarge covers sizes whose partitions recurse deeply.
func TestBridgeFirstSortMatchesSortFuncLarge(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(3, 4))
	shapes := bridgeKeyShapes(r)
	for _, name := range []string{"few values", "many ties", "equal", "sorted", "reversed", "sawtooth", "nan"} {
		for _, n := range []int{100003, 65536} {
			data := bridgeKeyCandidates(n, shapes[name])
			want := sortedFirstInput(data)
			bridgeFirstSort(data)
			require.Equalf(t, want, data[0].m, "shape %q n=%d", name, n)
		}
	}
}

// TestBridgeFirstSortKeepsMultiset checks that bridgeFirstSort only permutes its input.
func TestBridgeFirstSortKeepsMultiset(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(5, 6))
	shapes := bridgeKeyShapes(r)
	for _, name := range []string{"few values", "many ties", "nan", "reversed"} {
		data := bridgeKeyCandidates(5000, shapes[name])
		bridgeFirstSort(data)
		seen := make([]bool, len(data))
		for _, c := range data {
			require.Falsef(t, seen[c.m], "shape %q repeats input position %d", name, c.m)
			seen[c.m] = true
		}
	}
}

// ringScene returns an n-gon outer loop of radius 10 and a concentric n-gon hole of radius 4 turned by rot, so a
// rot of 0 puts every hole vertex on a ray through an outer vertex and ties n candidates.
func ringScene(n int, rot float64) ([]pt2, []int, [][]int) {
	var ps []pt2
	outer := make([]int, n)
	hole := make([]int, n)
	for i := range n {
		a := 2 * math.Pi * float64(i) / float64(n)
		outer[i] = len(ps)
		ps = append(ps, pt2{10 * math.Cos(a), 10 * math.Sin(a)})
	}
	// The hole runs clockwise, as a hole loop does.
	for i := range n {
		a := rot - 2*math.Pi*float64(i)/float64(n)
		hole[i] = len(ps)
		ps = append(ps, pt2{4 * math.Cos(a), 4 * math.Sin(a)})
	}
	return ps, outer, [][]int{hole}
}

// TestBridgeNearestTiedCandidatesMatchSortedSearch checks that bridgeNearest picks the bridge the all-candidate
// sort picks when 2, 3 or 4 candidates tie at the minimum.
func TestBridgeNearestTiedCandidatesMatchSortedSearch(t *testing.T) {
	t.Parallel()
	type scene struct {
		name  string
		ps    []pt2
		outer []int
		holes [][]int
	}
	var scenes []scene
	scenes = append(scenes, scene{
		"two ties", append([]pt2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, pt2{4, 8}, pt2{5, 5}, pt2{6, 8}),
		[]int{0, 1, 2, 3}, [][]int{{4, 5, 6}},
	})
	for _, n := range []int{3, 4} {
		ps, outer, holes := ringScene(n, 0)
		scenes = append(scenes, scene{fmt.Sprintf("ring %d", n), ps, outer, holes})
	}
	for _, sc := range scenes {
		want, wantRest, wantErr := sortedBridge(sc.ps, sc.outer, sc.holes)
		got, gotRest, gotErr := bridgeNearest(sc.ps, sc.outer, sc.holes)
		require.NoError(t, wantErr, sc.name)
		require.NoError(t, gotErr, sc.name)
		require.Equal(t, want, got, sc.name)
		require.Equal(t, wantRest, gotRest, sc.name)
	}
}

// bridgeRandomScene builds a 20 x 20 square with one or two holes of 3 to 5 vertices on a 2 mm grid, so many
// candidates tie. Some holes are concave or self-crossing; the sort reference sees the same scene.
func bridgeRandomScene(r *rand.Rand) ([]pt2, []int, [][]int) {
	ps := []pt2{{0, 0}, {20, 0}, {20, 20}, {0, 20}}
	outer := []int{0, 1, 2, 3}
	var holes [][]int
	for range 1 + r.IntN(2) {
		var hole []int
		for range 3 + r.IntN(3) {
			hole = append(hole, len(ps))
			ps = append(ps, pt2{float64(2 * (1 + r.IntN(9))), float64(2 * (1 + r.IntN(9)))})
		}
		holes = append(holes, hole)
	}
	return ps, outer, holes
}

// firstRejected reports whether the candidate the first-element selector picks fails the acceptance test, so
// bridgeNearest takes the full-sort fallback, and whether the minimum distance is tied.
func firstRejected(ps []pt2, merged []int, holes [][]int) (rejected, tied bool) {
	var cands []bridgeCandidate
	for hi, hole := range holes {
		for hk, hv := range hole {
			for mi, mv := range merged {
				du, dw := ps[hv].u-ps[mv].u, ps[hv].w-ps[mv].w
				cands = append(cands, bridgeCandidate{d: du*du + dw*dw, m: mi, h: hk, holeIdx: hi})
			}
		}
	}
	if len(cands) == 0 {
		return false, false
	}
	minD := slices.MinFunc(cands, func(a, b bridgeCandidate) int { return cmp.Compare(a.d, b.d) }).d
	minCount := 0
	for _, c := range cands {
		if c.d == minD {
			minCount++
		}
	}
	bridgeFirstSort(cands)
	c := cands[0]
	hole := holes[c.holeIdx]
	a, b := ps[merged[c.m]], ps[hole[c.h]]
	ok := bridgeClear(ps, merged, holes, c.holeIdx, c.m, c.h) &&
		insidePolygon(ps, merged, pt2{(a.u + b.u) / 2, (a.w + b.w) / 2})
	return !ok, minCount > 1
}

// TestBridgeNearestRandomTiesMatchSortedSearch compares bridgeNearest with the all-candidate sort on many scenes
// with tied candidates, including scenes where the first-element pick is rejected and the sort fallback runs.
func TestBridgeNearestRandomTiesMatchSortedSearch(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(7, 8))
	tied, fallback, tiedFallback := 0, 0, 0
	for range 3000 {
		ps, outer, holes := bridgeRandomScene(r)
		rejected, isTied := firstRejected(ps, outer, holes)
		if isTied {
			tied++
		}
		if rejected {
			fallback++
		}
		if rejected && isTied {
			tiedFallback++
		}
		want, wantRest, wantErr := sortedBridge(ps, outer, holes)
		got, gotRest, gotErr := bridgeNearest(ps, outer, holes)
		require.Equal(t, wantErr == nil, gotErr == nil)
		if wantErr != nil {
			continue
		}
		require.Equal(t, want, got)
		require.Equal(t, wantRest, gotRest)
	}
	require.Positive(t, tied, "no scene tied the nearest candidates")
	require.Positive(t, fallback, "no scene rejected the first-element pick")
	require.Positive(t, tiedFallback, "no tied scene rejected the first-element pick")
}
