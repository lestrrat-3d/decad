package loftmesh

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestGroupedTreeScanKeepsExactlyUnprovedBoxPairs(t *testing.T) {
	verts := make([]r3.Vec, 0, 36)
	tris := make([][3]int, 0, 12)
	families := []int{-1, 0, 1, 0, -1, 1, 2, 2, -1, 0, 1, 2}
	for i := range families {
		x := float64(i%4) * 0.4
		y := float64(i/4) * 0.3
		start := len(verts)
		verts = append(verts, r3.Vec{X: x, Y: y},
			r3.Vec{X: x + 0.8, Y: y}, r3.Vec{X: x, Y: y + 0.8})
		tris = append(tris, [3]int{start, start + 1, start + 2})
	}
	boxes := make([][2]r3.Vec, len(tris))
	members := make([]int, len(tris))
	for i, tri := range tris {
		boxes[i], members[i] = meshbool.TriBox(verts, tri), i
	}
	var got [][2]int
	scan := newGroupedTreeScan(boxes, members, families)
	scanned, exceeded, err := scan.visit(proofbound.NewWorkBudget(t.Context()), math.MaxUint64,
		func(i, j int) { got = append(got, [2]int{i, j}) })
	require.NoError(t, err)
	require.False(t, exceeded)
	require.GreaterOrEqual(t, scanned, uint64(len(got)))
	slices.SortFunc(got, func(a, b [2]int) int {
		if a[0] != b[0] {
			return a[0] - b[0]
		}
		return a[1] - b[1]
	})
	var want [][2]int
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			if families[i] >= 0 && families[i] == families[j] {
				continue
			}
			if meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				want = append(want, [2]int{i, j})
			}
		}
	}
	require.Equal(t, want, got)
	for i := range tris {
		var row []int
		rowWork, rowExceeded, rowErr := scan.visitRow(proofbound.NewWorkBudget(t.Context()),
			i, math.MaxUint64, func(j int) { row = append(row, j) })
		require.NoError(t, rowErr)
		require.False(t, rowExceeded)
		require.GreaterOrEqual(t, rowWork, uint64(len(row)))
		slices.Sort(row)
		var expected []int
		for _, pair := range want {
			if pair[0] == i {
				expected = append(expected, pair[1])
			}
		}
		require.Equal(t, expected, row)
	}
	limited, exceeded, err := scan.visit(proofbound.NewWorkBudget(t.Context()), 1,
		func(i, j int) {})
	require.NoError(t, err)
	require.True(t, exceeded)
	require.Equal(t, uint64(2), limited)
}
