// CI: isolated-tests

package retainedblend

import (
	"fmt"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestBridgeFirstSortMatchesSortFunc checks that bridgeFirstSort leaves at index 0 exactly the candidate that
// slices.SortFunc leaves there, ties included.
//
// bridgeFirstSort freezes the first-element pick to the order of Go 1.27.4's slices.SortFunc; the sort fallback in
// bridgeNearest still calls the toolchain's slices.SortFunc. This test is a tripwire: if a Go upgrade changes
// SortFunc's tie order, it fails. The fix is to update the copy from the new toolchain's slices/zsortanyfunc.go and
// compare the gallery swisslever fingerprint before accepting the change.
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

// TestBridgeFirstSortSweepMatchesSortFunc checks every size from 0 to 300 against slices.SortFunc for many key
// shapes, 20 repeats each, comparing which element ends at index 0. Together with the pdqsort thresholds this
// reaches the ninther, the decreasing-hint reversal and the partial insertion sort's shifting.
func TestBridgeFirstSortSweepMatchesSortFunc(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(9, 10))
	for _, shape := range bridgeSweepShapes() {
		for n := range 301 {
			for rep := range 20 {
				data := bridgeKeyCandidates(n, func(i, _ int) float64 { return 0 })
				for i, k := range shape.keys(r, n) {
					data[i].d = k
				}
				if n == 0 {
					bridgeFirstSort(data)
					continue
				}
				want := sortedFirstInput(data)
				bridgeFirstSort(data)
				// Compare bounded scalars only; a failing run never dumps the slice.
				require.Equalf(t, want, data[0].m, "shape %q n=%d rep=%d", shape.name, n, rep)
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

// TestBridgeNearestBoundaryScenesMatchSortedSearch compares bridgeNearest with the all-candidate sort on 20,000
// random scenes: a 20 x 20 square with a vertex every 2 units (40 boundary vertices) and one or two grid holes. It
// requires that enough scenes reject the first-element pick, so the sort fallback and its candidate rebuild run.
func TestBridgeNearestBoundaryScenesMatchSortedSearch(t *testing.T) {
	t.Parallel()
	const scenes, minFallback = 20000, 100
	r := rand.New(rand.NewPCG(11, 12))
	fallback, solved := 0, 0
	for i := range scenes {
		ps, outer, holes := bridgeBoundaryScene(r)
		if rejected, _ := firstRejected(ps, outer, holes); rejected {
			fallback++
		}
		want, wantRest, wantErr := sortedBridge(ps, outer, holes)
		got, gotRest, gotErr := bridgeNearest(ps, outer, holes)
		require.Equalf(t, wantErr == nil, gotErr == nil, "scene %d error mismatch", i)
		if wantErr != nil {
			continue
		}
		solved++
		// Compare bounded facts first so a failure names the scene and sizes, never the whole loops.
		require.Equalf(t, len(want), len(got), "scene %d merged length", i)
		require.Equalf(t, len(wantRest), len(gotRest), "scene %d remaining holes", i)
		require.Truef(t, slices.Equal(want, got), "scene %d merged loop differs (len %d)", i, len(want))
		require.Truef(t, slices.EqualFunc(wantRest, gotRest, slices.Equal[[]int]), "scene %d remaining holes differ", i)
	}
	t.Logf("scenes=%d solved=%d fallback=%d", scenes, solved, fallback)
	require.GreaterOrEqualf(t, fallback, minFallback, "only %d scenes rejected the first-element pick", fallback)
}
