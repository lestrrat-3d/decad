package retainedblend

import (
	"cmp"
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

// sortedBridge is the original all-candidate sort and bridge search.
func sortedBridge(ps []pt2, merged []int, holes [][]int) ([]int, [][]int, error) {
	type candidate struct {
		d             float64
		m, h, holeIdx int
	}
	var cands []candidate
	for hi, hole := range holes {
		for hk, hv := range hole {
			for mi, mv := range merged {
				du, dw := ps[hv].u-ps[mv].u, ps[hv].w-ps[mv].w
				cands = append(cands, candidate{du*du + dw*dw, mi, hk, hi})
			}
		}
	}
	slices.SortFunc(cands, func(a, b candidate) int { return cmp.Compare(a.d, b.d) })
	for _, c := range cands {
		hole := holes[c.holeIdx]
		a, b := ps[merged[c.m]], ps[hole[c.h]]
		if !bridgeClear(ps, merged, holes, c.holeIdx, c.m, c.h) ||
			!insidePolygon(ps, merged, pt2{(a.u + b.u) / 2, (a.w + b.w) / 2}) {
			continue
		}
		spliced := append([]int(nil), merged[:c.m+1]...)
		for k := range len(hole) + 1 {
			spliced = append(spliced, hole[(c.h+k)%len(hole)])
		}
		spliced = append(spliced, merged[c.m])
		spliced = append(spliced, merged[c.m+1:]...)
		rest := append([][]int(nil), holes[:c.holeIdx]...)
		rest = append(rest, holes[c.holeIdx+1:]...)
		return spliced, rest, nil
	}
	return nil, nil, fmt.Errorf("no bridge")
}

func TestBridgeNearestMatchesSortedSearch(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name             string
		hole             []pt2
		unique, accepted bool
	}{
		{"unique accepted", []pt2{{1, 1.2}, {1.3, 2}, {2, 1.3}}, true, true},
		{"equal-distance tie", []pt2{{2, 2}, {2, 8}, {8, 8}, {8, 2}}, false, true},
		{"rejected nearest", []pt2{{-1, 1}, {2, 2}, {2, 3}}, true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ps := append([]pt2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, tc.hole...)
			outer := []int{0, 1, 2, 3}
			hole := make([]int, len(tc.hole))
			for i := range hole {
				hole[i] = i + 4
			}
			holes := [][]int{hole}
			minDist := math.Inf(1)
			minCount, mi, hk := 0, 0, 0
			for h, hv := range hole {
				for m, mv := range outer {
					du, dw := ps[hv].u-ps[mv].u, ps[hv].w-ps[mv].w
					d := du*du + dw*dw
					if d < minDist {
						minDist, minCount, mi, hk = d, 1, m, h
					} else if d == minDist {
						minCount++
					}
				}
			}
			a, b := ps[outer[mi]], ps[hole[hk]]
			accepted := bridgeClear(ps, outer, holes, 0, mi, hk) && insidePolygon(ps, outer, pt2{(a.u + b.u) / 2, (a.w + b.w) / 2})
			require.Equal(t, tc.unique, minCount == 1)
			require.Equal(t, tc.accepted, accepted)
			want, wantRest, wantErr := sortedBridge(ps, outer, holes)
			got, gotRest, gotErr := bridgeNearest(ps, outer, holes)
			require.NoError(t, wantErr)
			require.NoError(t, gotErr)
			require.Equal(t, want, got)
			require.Equal(t, wantRest, gotRest)
		})
	}
}

func TestBridgeNearestMatchesSortedSearchWithMultipleHoles(t *testing.T) {
	t.Parallel()
	ps := []pt2{
		{0, 0}, {10, 0}, {10, 10}, {0, 10},
		{2, 2}, {2, 4}, {4, 4}, {4, 2},
		{6, 6}, {6, 8}, {8, 8}, {8, 6},
	}
	outer := []int{0, 1, 2, 3}
	holes := [][]int{{4, 5, 6, 7}, {8, 9, 10, 11}}
	want, wantRest, wantErr := sortedBridge(ps, outer, holes)
	got, gotRest, gotErr := bridgeNearest(ps, outer, holes)
	require.NoError(t, wantErr)
	require.NoError(t, gotErr)
	require.Equal(t, want, got)
	require.Equal(t, wantRest, gotRest)
}
