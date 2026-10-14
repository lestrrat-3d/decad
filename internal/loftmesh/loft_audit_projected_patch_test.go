package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectedPatchAuditSkipsOnlyCertifiedInternalPairs(t *testing.T) {
	verts := []r3.Vec{{X: 0, Y: 0}, {X: 2, Y: 0},
		{X: 2, Y: 2}, {X: 0, Y: 2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	patch := []loftmesh.ProjectedPatch{{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}}
	require.Error(t, loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(t.Context()), verts, tris, len(tris), 0))
	require.NoError(t, loftmesh.LoftCrossingAuditBoundedProjectedPatches(
		proofbound.NewWorkBudget(t.Context()), verts, tris, patch, len(tris), 0))

	verts = append(verts, r3.Vec{X: 0.5, Y: 0.5, Z: -1},
		r3.Vec{X: 1.5, Y: 0.5, Z: 1}, r3.Vec{X: 1, Y: 1.5, Z: -1})
	tris = append(tris, [3]int{4, 5, 6})
	require.Error(t, loftmesh.LoftCrossingAuditBoundedProjectedPatches(
		proofbound.NewWorkBudget(t.Context()), verts, tris, patch, len(tris), 100))
}

// fanCylinder returns a closed cylinder of radius 1 and height 1 with fan caps
// and quad walls over n rim stations, plus its two cap patches over XY. The
// rim vertices are 0..n-1 (bottom) and n..2n-1 (top); the cap centres follow.
func fanCylinder(n int) ([]r3.Vec, [][3]int, []loftmesh.ProjectedPatch) {
	verts := make([]r3.Vec, 0, 2*n+2)
	for _, z := range []float64{0, 1} {
		for i := range n {
			s, c := math.Sincos(2 * math.Pi * float64(i) / float64(n))
			verts = append(verts, r3.Vec{X: c, Y: s, Z: z})
		}
	}
	verts = append(verts, r3.Vec{Z: 0}, r3.Vec{Z: 1})
	bc, tc := 2*n, 2*n+1
	patches := []loftmesh.ProjectedPatch{{Axes: [2]int{0, 1}}, {Axes: [2]int{0, 1}}}
	var tris [][3]int
	for i := range n {
		j := (i + 1) % n
		tris = append(tris, [3]int{bc, j, i}, [3]int{tc, n + i, n + j})
		patches[0].Triangles = append(patches[0].Triangles, len(tris)-2)
		patches[1].Triangles = append(patches[1].Triangles, len(tris)-1)
		tris = append(tris, [3]int{i, j, n + j}, [3]int{i, n + j, n + i})
	}
	return verts, tris, patches
}

func TestPatchAuditMatchesPairwiseOnFanCylinder(t *testing.T) {
	const n = 48
	verts, tris, patches := fanCylinder(n)
	const maxScans = 1_200_000_000
	pairwise := func() error {
		return loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(t.Context()),
			verts, tris, loftmesh.MaxLoftAuditTriangles, maxScans)
	}
	streaming := func() error {
		return loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(t.Context()),
			verts, tris, patches, loftmesh.MaxLoftAuditTriangles, maxScans)
	}
	for k, p := range patches {
		require.Equal(t, loftmesh.ProjectedPatchHolds,
			loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, p), "cap %d", k)
	}
	require.NoError(t, pairwise())
	require.NoError(t, streaming())

	// Moving one top rim vertex across the centre makes two top cap
	// triangles overlap. The broken cap's proof fails, so its pairs are
	// audited and both routes refuse.
	verts[n+10] = r3.Vec{X: -verts[n+10].X, Y: -verts[n+10].Y, Z: 1}
	require.NotEqual(t, loftmesh.ProjectedPatchHolds,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patches[1]))
	require.ErrorIs(t, pairwise(), decaderr.ErrDegenerate)
	require.ErrorIs(t, streaming(), decaderr.ErrDegenerate)
}

// TestPatchAuditFailedCapFallsBackToExactPairs has a bad contact only inside
// one cap: the mesh is that cap alone, with no wall. The cap's proof fails,
// so only the exact pair audit can find the overlap.
func TestPatchAuditFailedCapFallsBackToExactPairs(t *testing.T) {
	const n = 48
	verts, fullTris, patches := fanCylinder(n)
	top := patches[1].Triangles
	tris := make([][3]int, 0, len(top))
	capIdx := make([]int, 0, len(top))
	for _, ti := range top {
		capIdx = append(capIdx, len(tris))
		tris = append(tris, fullTris[ti])
	}
	patch := []loftmesh.ProjectedPatch{{Triangles: capIdx, Axes: [2]int{0, 1}}}
	audit := func() error {
		return loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(t.Context()),
			verts, tris, patch, loftmesh.MaxLoftAuditTriangles, 1_200_000_000)
	}
	require.Equal(t, loftmesh.ProjectedPatchHolds,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch[0]))
	require.NoError(t, audit())

	verts[n+10] = r3.Vec{X: -verts[n+10].X, Y: -verts[n+10].Y, Z: 1}
	require.NotEqual(t, loftmesh.ProjectedPatchHolds,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch[0]))
	require.ErrorIs(t, audit(), decaderr.ErrDegenerate)
	require.ErrorIs(t, loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(t.Context()),
		verts, tris, loftmesh.MaxLoftAuditTriangles, 1_200_000_000), decaderr.ErrDegenerate)
}
