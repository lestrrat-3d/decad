package tessellation_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func tetrahedron() [][3]int {
	return [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}}
}

func requireAudit(t *testing.T, err error, want tessellation.Sentinel) *tessellation.AuditError {
	t.Helper()
	var audit *tessellation.AuditError
	require.True(t, errors.As(err, &audit), "want an *AuditError, got %v", err)
	require.Equal(t, want, audit.Sentinel)
	return audit
}

func TestClosedMeshAdmitsATetrahedron(t *testing.T) {
	t.Parallel()
	require.NoError(t, tessellation.RequireClosedMesh(tetrahedron()))
}

func TestClosedMeshRefusesAnOpenFan(t *testing.T) {
	t.Parallel()
	err := tessellation.RequireClosedMesh([][3]int{{0, 1, 2}, {0, 2, 3}})
	audit := requireAudit(t, err, tessellation.Degenerate)
	require.Equal(t, "the chorded boundary could not be triangulated into a watertight mesh", audit.Detail)
}

// TestSheetVertexLinksAdmitsAnOpenPathFan is a fan of triangles around a
// center vertex, none of them closing the fan into a full disk: the center's
// link is an OPEN PATH, exactly the shape every boundary vertex of a sound
// open sheet has.
func TestSheetVertexLinksAdmitsAnOpenPathFan(t *testing.T) {
	t.Parallel()
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}}
	require.NoError(t, tessellation.RequireSheetVertexLinks(t.Context(), 5, tris))
}

// TestSheetVertexLinksRefusesAForkedLink adds one triangle to the fan above
// that gives link vertex 2 a THIRD link edge, neither a cycle nor a path.
func TestSheetVertexLinksRefusesAForkedLink(t *testing.T) {
	t.Parallel()
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 2, 5}}
	err := tessellation.RequireSheetVertexLinks(t.Context(), 6, tris)
	audit := requireAudit(t, err, tessellation.Unsupported)
	require.Contains(t, audit.Error(), "forked link")
}

// TestSheetVertexLinksRefusesTwoConesSharingAnApex has two tetrahedral cones
// meeting at vertex 0: each cone alone has a sound closed-cycle link, but
// together apex 0's link is two disjoint cycles.
func TestSheetVertexLinksRefusesTwoConesSharingAnApex(t *testing.T) {
	t.Parallel()
	tris := [][3]int{
		{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2},
		{0, 5, 4}, {0, 6, 5}, {0, 4, 6}, {4, 5, 6},
	}
	err := tessellation.RequireSheetVertexLinks(t.Context(), 7, tris)
	audit := requireAudit(t, err, tessellation.Unsupported)
	require.Contains(t, audit.Error(), "more than one connected component")
}

func TestSheetVertexLinksReturnsACancelledContext(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}}
	require.ErrorIs(t, tessellation.RequireSheetVertexLinks(ctx, 5, tris), context.Canceled)
}

func TestSheetBoundaryRefusesADuplicatedDirectedEdge(t *testing.T) {
	t.Parallel()
	err := tessellation.RequireSheetBoundary(t.Context(), tessellation.SheetBoundary{
		Triangles:   [][3]int{{0, 1, 2}, {0, 1, 2}},
		SourceFaces: []int{0, 0},
	})
	audit := requireAudit(t, err, tessellation.Degenerate)
	require.Contains(t, audit.Detail, "occurs 2 times")
}

func TestSheetBoundaryComparesChainCountsPerFace(t *testing.T) {
	t.Parallel()
	quad := func(counts map[int]int) error {
		return tessellation.RequireSheetBoundary(t.Context(), tessellation.SheetBoundary{
			Triangles:       [][3]int{{0, 1, 2}, {0, 2, 3}},
			SourceFaces:     []int{0, 0},
			FreeChainCounts: counts,
		})
	}
	require.NoError(t, quad(map[int]int{0: 1}))

	audit := requireAudit(t, quad(map[int]int{0: 2}), tessellation.Degenerate)
	require.Contains(t, audit.Detail, "1 free boundary chain(s) in the mesh but 2")

	audit = requireAudit(t, quad(map[int]int{1: 1}), tessellation.Degenerate)
	require.Contains(t, audit.Detail, "carries no recorded free edge for")

	audit = requireAudit(t, quad(map[int]int{}), tessellation.Degenerate)
	require.Contains(t, audit.Detail, "names 1 face(s)")
}

// TestSheetBoundaryAdmitsAClosedSetWithNoFreeBoundary is
// docs/tessellation-design.md §1.2's closed-sheet sentence: with no free
// edge, every directed edge has its reverse and the audit passes.
func TestSheetBoundaryAdmitsAClosedSetWithNoFreeBoundary(t *testing.T) {
	t.Parallel()
	require.NoError(t, tessellation.RequireSheetBoundary(t.Context(), tessellation.SheetBoundary{
		Triangles:   tetrahedron(),
		SourceFaces: []int{0, 1, 2, 3},
	}))
}

func TestOrientationSignReadsTheTetrahedronSum(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	outward := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	require.Equal(t, 1, tessellation.OrientationSign(verts, outward, r3.Vec{}))
	require.Equal(t, 1, tessellation.OrientationSign(verts, outward, r3.Vec{X: 100, Y: -7, Z: 3}),
		"a closed set's sum is anchor-independent")

	reversed := make([][3]int, len(outward))
	for i, tri := range outward {
		reversed[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	require.Equal(t, -1, tessellation.OrientationSign(verts, reversed, r3.Vec{}))

	flat := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 0, Y: 0, Z: 0}}
	require.Equal(t, 0, tessellation.OrientationSign(flat, outward, r3.Vec{}))
}
