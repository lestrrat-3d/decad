package loftmesh_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireAudit(t *testing.T, err error, want tessellation.Sentinel) *tessellation.AuditError {
	t.Helper()
	var audit *tessellation.AuditError
	require.True(t, errors.As(err, &audit), "want an *AuditError, got %v", err)
	require.Equal(t, want, audit.Sentinel)
	return audit
}

// unitCubeInput is the unit cube as a loft payload holds it: eight wall
// triangles (two per cell), then a two-triangle start cap, then a
// two-triangle end cap.
func unitCubeInput() loftmesh.LoftInput {
	in := loftmesh.LoftInput{
		Vertices: []r3.Vec{
			{}, {X: 1}, {X: 1, Y: 1}, {Y: 1},
			{Z: 1}, {X: 1, Z: 1}, {X: 1, Y: 1, Z: 1}, {Y: 1, Z: 1},
		},
		WallCount:     8,
		StartCapCount: 2,
		FaceOfRole:    map[string]int{"capStart": 8, "capEnd": 9},
		StartCapRole:  "capStart",
		EndCapRole:    "capEnd",
		FreeChainCounts: map[int]int{
			0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1, 7: 1,
		},
		FacetDepartureMM: 0.25,
		AreaSlackMM2:     0.5,
		VolumeSymDiffMM3: 0.75,
	}
	for j := range 4 {
		n := (j + 1) % 4
		in.Triangles = append(in.Triangles, [3]int{j, n, j + 4}, [3]int{n, n + 4, j + 4})
		in.WallCell = append(in.WallCell, [2]int{0, j}, [2]int{0, j})
		in.WallSide = append(in.WallSide, 0, 1)
	}
	for j := range 4 {
		in.FaceOfRole[sideRole(j, 0)] = 2 * j
		in.FaceOfRole[sideRole(j, 1)] = 2*j + 1
	}
	in.Triangles = append(in.Triangles,
		[3]int{0, 2, 1}, [3]int{0, 3, 2},
		[3]int{4, 5, 6}, [3]int{4, 6, 7},
	)
	return in
}

func sideRole(j, k int) string {
	return "side(0," + string(rune('0'+j)) + "," + string(rune('0'+k)) + ")"
}

func TestRestateLoftCopiesASolidWithoutAliasing(t *testing.T) {
	t.Parallel()
	in := unitCubeInput()
	wantVerts := append([]r3.Vec(nil), in.Vertices...)
	wantTris := append([][3]int(nil), in.Triangles...)

	out, err := loftmesh.RestateLoft(t.Context(), in)
	require.NoError(t, err)
	require.Equal(t, wantTris, out.Triangles)
	require.Equal(t, wantVerts, out.Vertices)
	require.Equal(t, []int{0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9}, out.SourceFaces)
	require.Equal(t, 0.25, out.BoundMM)
	require.Equal(t, 0.5, out.AreaSlackMM2)
	require.Equal(t, 0.75, out.VolumeSymDiffMM3)
	require.True(t, out.VolumeProof)

	in.Vertices[0] = r3.Vec{X: 9}
	in.Triangles[0] = [3]int{7, 7, 7}
	require.Equal(t, wantVerts, out.Vertices)
	require.Equal(t, wantTris, out.Triangles)
	require.NotSame(t, &in.Vertices[0], &out.Vertices[0])
}

func TestRestateLoftSheetKeepsOnlyTheWallRange(t *testing.T) {
	t.Parallel()
	in := unitCubeInput()
	in.Sheet = true
	delete(in.FaceOfRole, "capStart")
	delete(in.FaceOfRole, "capEnd")
	in.VolumeSymDiffMM3 = math.NaN()

	out, err := loftmesh.RestateLoft(t.Context(), in)
	require.NoError(t, err)
	require.Len(t, out.Triangles, 8)
	require.Equal(t, []int{0, 1, 2, 3, 4, 5, 6, 7}, out.SourceFaces)
	require.False(t, out.VolumeProof)
	require.Zero(t, out.VolumeSymDiffMM3)

	in.FreeChainCounts[0] = 2
	_, err = loftmesh.RestateLoft(t.Context(), in)
	requireAudit(t, err, tessellation.Degenerate)
}

func TestRestateLoftRefusesAnUnrestatableInput(t *testing.T) {
	t.Parallel()
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	tests := []struct {
		name     string
		mutate   func(in *loftmesh.LoftInput)
		sentinel tessellation.Sentinel
		contains string
	}{
		{"no triangles", func(in *loftmesh.LoftInput) { in.Triangles = nil },
			tessellation.Degenerate, "holds no triangle set"},
		{"split does not partition", func(in *loftmesh.LoftInput) { in.WallCount = 13 },
			tessellation.Degenerate, "do not partition"},
		{"no wall cells", func(in *loftmesh.LoftInput) { in.WallCell = nil },
			tessellation.Degenerate, "names a cell for 0 of its 8"},
		{"missing wall role", func(in *loftmesh.LoftInput) { delete(in.FaceOfRole, "side(0,0,0)") },
			tessellation.Degenerate, `role "side(0,0,0)"`},
		{"missing end cap role", func(in *loftmesh.LoftInput) { delete(in.FaceOfRole, "capEnd") },
			tessellation.Degenerate, `role "capEnd"`},
		{"infinite facet departure", func(in *loftmesh.LoftInput) { in.FacetDepartureMM = math.Inf(1) },
			tessellation.Unsupported, "no finite proof"},
		{"infinite area slack", func(in *loftmesh.LoftInput) { in.AreaSlackMM2 = math.Inf(1) },
			tessellation.Unsupported, "no finite proof"},
		{"NaN volume proof", func(in *loftmesh.LoftInput) { in.VolumeSymDiffMM3 = math.NaN() },
			tessellation.Unsupported, "no finite proof"},
		{"open mesh", func(in *loftmesh.LoftInput) { in.Triangles = in.Triangles[:len(in.Triangles)-1] },
			tessellation.Unsupported, "not a closed mesh"},
		{"non-finite anchor", func(in *loftmesh.LoftInput) { in.Anchor.X = math.Inf(1) },
			tessellation.Unsupported, "no finite anchor"},
		{"reversed winding", func(in *loftmesh.LoftInput) {
			for i, tri := range in.Triangles {
				in.Triangles[i] = [3]int{tri[0], tri[2], tri[1]}
			}
		}, tessellation.Unsupported, "does not enclose a positive volume"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			in := unitCubeInput()
			tc.mutate(&in)
			_, err := loftmesh.RestateLoft(t.Context(), in)
			audit := requireAudit(t, err, tc.sentinel)
			require.Contains(t, audit.Detail, tc.contains)
		})
	}
	t.Run("cancelled context", func(t *testing.T) {
		t.Parallel()
		_, err := loftmesh.RestateLoft(cancelled, unitCubeInput())
		require.ErrorIs(t, err, context.Canceled)
	})
}
