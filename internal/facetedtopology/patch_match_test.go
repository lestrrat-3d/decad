package facetedtopology_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestUnchangedGroupPatch(t *testing.T) {
	base := facetedtopology.GroupedMesh{
		Verts:  []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}},
		Tris:   [][3]int{{0, 1, 2}, {0, 2, 3}},
		Src:    []int{1, 1},
		FaceOf: []int{0, 0},
	}
	result := func() *facetedtopology.GroupedMesh {
		return &facetedtopology.GroupedMesh{
			Verts:  []r3.Vec{{X: 1, Y: 1}, {}, {Y: 1}, {X: 1}},
			Tris:   [][3]int{{1, 3, 0}, {1, 0, 2}, {0, 1, 2}},
			Src:    []int{0, 0, 5},
			FaceOf: []int{2, 2, 1},
		}
	}
	run := func(t *testing.T, group int, res *facetedtopology.GroupedMesh) (int, bool) {
		t.Helper()
		face, ok, err := facetedtopology.UnchangedGroupPatch(t.Context(), base, 2, group, 1, res, 3)
		require.NoError(t, err)
		return face, ok
	}

	face, ok := run(t, 1, result())
	require.True(t, ok)
	require.Equal(t, 2, face)

	dropped := result()
	dropped.Tris, dropped.Src, dropped.FaceOf = dropped.Tris[:1], dropped.Src[:1], dropped.FaceOf[:1]
	doubled := result()
	doubled.Tris[1] = [3]int{1, 3, 0}
	split := result()
	split.FaceOf[1] = 1
	tests := []struct {
		name  string
		group int
		res   *facetedtopology.GroupedMesh
	}{
		{"dropped triangle", 1, dropped},
		{"repeated triangle", 1, doubled},
		{"split across faces", 1, split},
		{"nil result", 1, nil},
		{"group out of range", 2, result()},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			face, ok := run(t, tc.group, tc.res)
			require.False(t, ok)
			require.Equal(t, -1, face)
		})
	}

	t.Run("cancelled context", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, ok, err := facetedtopology.UnchangedGroupPatch(ctx, base, 2, 1, 1, result(), 3)
		require.False(t, ok)
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestSortedTriangle(t *testing.T) {
	got, sign := facetedtopology.SortedTriangle([3]int{2, 0, 1})
	require.Equal(t, [3]int{0, 1, 2}, got)
	require.Equal(t, 1, sign)
	got, sign = facetedtopology.SortedTriangle([3]int{1, 0, 2})
	require.Equal(t, [3]int{0, 1, 2}, got)
	require.Equal(t, -1, sign)
}

func TestCapsOppose(t *testing.T) {
	lower, lowerSrc := [][3]int{{0, 1, 2}}, []int{4}
	require.True(t, facetedtopology.CapsOppose(lower, lowerSrc, [][3]int{{0, 2, 1}}, []int{4}, []int{0, 1, 2}, 4, 4))
	require.False(t, facetedtopology.CapsOppose(lower, lowerSrc, [][3]int{{0, 1, 2}}, []int{4}, []int{0, 1, 2}, 4, 4))
	require.False(t, facetedtopology.CapsOppose(lower, lowerSrc, [][3]int{{0, 2, 1}}, []int{4}, []int{0, 1}, 4, 4))
	require.False(t, facetedtopology.CapsOppose(lower, lowerSrc,
		[][3]int{{0, 2, 1}, {0, 2, 1}}, []int{4, 4}, []int{0, 1, 2}, 4, 4))

	quad, quadSrc := [][3]int{{0, 1, 2}, {0, 2, 3}}, []int{4, 4}
	upper, upperSrc := [][3]int{{2, 1, 0}, {3, 2, 0}}, []int{4, 4}
	identity := []int{0, 1, 2, 3}
	require.True(t, facetedtopology.CapsOppose(quad, quadSrc, upper, upperSrc, identity, 4, 4))
	require.False(t, facetedtopology.CapsOppose(quad, quadSrc, [][3]int{{2, 1, 0}, {0, 2, 3}}, upperSrc,
		identity, 4, 4), "one upper triangle keeps the lower winding")
	require.False(t, facetedtopology.CapsOppose(quad, quadSrc, upper, upperSrc, []int{0, 1, 2, 4}, 4, 4),
		"the map moves an upper vertex off the lower cap")
	require.False(t, facetedtopology.CapsOppose(quad, quadSrc, upper, []int{4, 3}, identity, 4, 4),
		"one upper triangle leaves the cap group")
}
