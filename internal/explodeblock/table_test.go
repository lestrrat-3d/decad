package explodeblock_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTableCollarAssembly(t *testing.T) {
	const bound = 0.02
	meshes := []explodeblock.Mesh{
		collarShaft(), // 0 shaft
		ring(bound),   // 1 bearing
		box(r3.NewVec(20, 20, 0), r3.NewVec(24, 24, 3)), // 2 distant block
	}
	build := func(dir r3.Vec, workers int) *explodeblock.Table {
		var items []explodeblock.Instance
		for _, m := range meshes {
			items = append(items, place(t, m, dir))
		}
		return explodeblock.BuildTable(items, workers)
	}

	upTable := build(up, 1)
	require.Equal(t, 3, upTable.N())
	require.Equal(t, []int{1}, upTable.Blockers(0))
	require.Empty(t, upTable.Blockers(1))
	require.Empty(t, upTable.Blockers(2))
	require.Empty(t, upTable.Uncertain(0))

	downTable := build(down, 4)
	require.Empty(t, downTable.Blockers(0))
	require.Equal(t, []int{0}, downTable.Blockers(1))
	require.Equal(t, down, downTable.Dir())

	// Parallel and serial builds agree entry by entry.
	serial := build(down, 1)
	for a := range 3 {
		for b := range 3 {
			require.Equal(t, serial.At(a, b).Verdict, downTable.At(a, b).Verdict)
		}
	}
}

func TestDirections(t *testing.T) {
	got := explodeblock.Directions([]r3.Vec{r3.NewVec(0, 0, 5), {}, r3.NewVec(1, 1, 0)}, 1e-9)
	require.Len(t, got, 2+2+4) // +Z -Z, the diagonal both ways, then +X -X +Y -Y
	require.Equal(t, up, got[0])
	require.Equal(t, down, got[1])
	require.InDelta(t, 0.7071067811865476, got[2].X, 1e-12)
	require.InDelta(t, -0.7071067811865476, got[3].X, 1e-12)
	require.Equal(t, r3.NewVec(1, 0, 0), got[4])
}
