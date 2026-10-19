package revolveloftfaces_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/stretchr/testify/require"
)

// The ring's vertex 10 sits on the positive Y axis. Between endpoints 12
// and 16 the forward walk misses the axis and the backward walk crosses it.
func TestShellOccupiedRingPathFollowsPositiveYSelection(t *testing.T) {
	ring := []int{10, 11, 12, 13, 14, 15, 16, 17}
	path, ok := revolveloftfaces.OccupiedRingPath(ring, [2]int{12, 16}, 0, false)
	require.True(t, ok)
	require.Equal(t, []int{12, 13, 14, 15, 16}, path)
	path, ok = revolveloftfaces.OccupiedRingPath(ring, [2]int{12, 16}, 0, true)
	require.True(t, ok)
	require.Equal(t, []int{12, 11, 10, 17, 16}, path)

	_, ok = revolveloftfaces.OccupiedRingPath(ring, [2]int{10, 14}, 0, true)
	require.False(t, ok, "an endpoint on the axis ray cannot select a walk")
	_, ok = revolveloftfaces.OccupiedRingPath(ring, [2]int{12, 12}, 0, true)
	require.False(t, ok, "equal endpoints have no walk between them")
	_, ok = revolveloftfaces.OccupiedRingPath(ring, [2]int{12, 99}, 0, true)
	require.False(t, ok, "an endpoint off the ring refuses")
	_, ok = revolveloftfaces.OccupiedRingPath([]int{10, 11, 12}, [2]int{11, 12}, 0, true)
	require.False(t, ok, "a ring of fewer than four rays refuses")
}

// A fan band joins a one-vertex ring to a four-ray ring. Each cell names one
// triangle of that fan.
func TestShellCellMatchesReadsSectionRings(t *testing.T) {
	shell := revolveloftfaces.ShellMesh{
		SectionRings: [][][]int{{{0}, {1, 2, 3, 4}}},
		RayCount:     4,
	}
	cell := revolveloftfaces.ShellTriangleCell{Section: 0, Band: 0, Ray: 3}
	require.True(t, shell.CellMatches([3]int{1, 0, 4}, cell))
	require.False(t, shell.CellMatches([3]int{0, 4, 2}, cell))
	cell.Half = 1
	require.False(t, shell.CellMatches([3]int{1, 0, 4}, cell),
		"a fan triangle has no second half")
}
