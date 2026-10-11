package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBevelBoreCapFanRequiresAdjacentRingVertices(t *testing.T) {
	positions := map[int]int{10: 0, 11: 1, 12: 2, 13: 3}
	sector, fan, err := bevelBoreCapFan([3]int{9, 13, 10}, 1, 9, 1, positions, 4)
	require.NoError(t, err)
	require.True(t, fan)
	require.Equal(t, 3, sector)
	_, fan, err = bevelBoreCapFan([3]int{10, 11, 12}, 1, 9, 1, positions, 4)
	require.NoError(t, err)
	require.False(t, fan)

	_, fan, err = bevelBoreCapFan([3]int{9, 10, 12}, 1, 9, 1, positions, 4)
	require.False(t, fan)
	require.ErrorIs(t, err, ErrUnsupported)
	_, fan, err = bevelBoreCapFan([3]int{9, 10, 11}, 2, 9, 1, positions, 4)
	require.False(t, fan)
	require.ErrorIs(t, err, ErrUnsupported)
	_, fan, err = bevelBoreCapFan([3]int{9, 10, 14}, 1, 9, 1, positions, 4)
	require.False(t, fan)
	require.ErrorIs(t, err, ErrUnsupported)
}

func TestBevelBoreTriangleClearKeepsOnlyOutsideFacets(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 2, -1), r3.NewVec(1, 3, 0), r3.NewVec(2, 2, 1),
		r3.NewVec(0, -2, -1), r3.NewVec(1, 0, 0), r3.NewVec(2, 1, 1),
		r3.NewVec(0, 1, 0),
	}
	require.True(t, bevelBoreTriangleClear([3]int{0, 1, 2}, verts, 1))
	require.False(t, bevelBoreTriangleClear([3]int{3, 4, 5}, verts, 1))
	require.False(t, bevelBoreTriangleClear([3]int{0, 1, 6}, verts, 1),
		"a facet touching the bound still needs the exact audit")
}

func TestBevelBorePatternKindRequiresMatchingToeTopology(t *testing.T) {
	for _, test := range []struct {
		placed     int
		toeClipped bool
		ok         bool
	}{
		{placed: 7, ok: true},
		{placed: 11, toeClipped: true, ok: true},
		{placed: 15, ok: true},
		{placed: 14},
	} {
		toeClipped, ok := bevelBorePatternKind(test.placed)
		require.Equal(t, test.ok, ok)
		require.Equal(t, test.toeClipped, toeClipped)
	}
}
