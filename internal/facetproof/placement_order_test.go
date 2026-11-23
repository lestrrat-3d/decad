package facetproof_test

import (
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ulpChainMesh loads testdata/placement_ulp_chain.json: a closed solid of 50
// facets whose ridge holds seven vertices one to 23 ulps apart along X near
// x = -2.6. Two fans meet on that chain: needles from a far apex on the top
// face, and the facets of a sloped face, each chain vertex joined to several
// vertices below it. The ulp at x = 97.4 is 32 times coarser, so a translation
// by 100 along X rounds the chain into one or two floats.
func ulpChainMesh(t *testing.T) ([]r3.Vec, [][3]int) {
	t.Helper()
	raw, err := os.ReadFile("testdata/placement_ulp_chain.json")
	require.NoError(t, err)
	var d struct {
		V [][3]float64 `json:"V"`
		T [][3]int     `json:"T"`
	}
	require.NoError(t, json.Unmarshal(raw, &d))
	verts := make([]r3.Vec, len(d.V))
	for i, v := range d.V {
		verts[i] = r3.Vec{X: v[0], Y: v[1], Z: v[2]}
	}
	return verts, d.T
}

// ulpChain lists the chain vertices of ulpChainMesh in ascending X.
var ulpChain = []int{1, 3, 5, 7, 10, 20, 15}

func exactImages(verts []r3.Vec, move r3.Transform) []proof.Xpt {
	basis, tr := move.Basis(), move.Translation()
	exact := make([]proof.Xpt, len(verts))
	for i, v := range verts {
		exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v)
	}
	return exact
}

func TestOrderedRounding(t *testing.T) {
	t.Parallel()

	t.Run("keeps nearest rounding when it merges nothing", func(t *testing.T) {
		t.Parallel()
		exact := []proof.Xpt{
			proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 3}), proof.XptOf(r3.Vec{X: 1, Y: -2, Z: 0.5}),
			proof.XptOf(r3.Vec{X: 7, Y: 2, Z: 3}),
		}
		got, changed, err := facetproof.OrderedRounding(t.Context(), exact)
		require.NoError(t, err)
		require.False(t, changed)
		for i, p := range exact {
			require.Equal(t, p.Vec(), got[i])
		}
	})

	t.Run("spreads merged coordinates in their exact order", func(t *testing.T) {
		t.Parallel()
		verts, _ := ulpChainMesh(t)
		move, err := r3.Translation(r3.Vec{X: 100})
		require.NoError(t, err)
		exact := exactImages(verts, move)
		merged := map[float64]struct{}{}
		for _, v := range ulpChain {
			merged[exact[v].Vec().X] = struct{}{}
		}
		require.Less(t, len(merged), len(ulpChain), "nearest rounding merges the chain")
		got, changed, err := facetproof.OrderedRounding(t.Context(), exact)
		require.NoError(t, err)
		require.True(t, changed)
		for k := 1; k < len(ulpChain); k++ {
			require.Less(t, got[ulpChain[k-1]].X, got[ulpChain[k]].X, "chain vertex %d", ulpChain[k])
		}
		for i, p := range exact {
			near := p.Vec()
			// Only X merges; equal exact Y and Z coordinates stay equal.
			require.Equal(t, near.Y, got[i].Y)
			require.Equal(t, near.Z, got[i].Z)
			// A run of n merged values moves no coordinate more than n ulps.
			require.LessOrEqual(t, math.Abs(got[i].X-near.X), float64(len(ulpChain))*proofbound.UlpOf(near.X))
		}
	})
}

func TestPlaceMeshOrdersAMergedChain(t *testing.T) {
	t.Parallel()
	const reach = 0.01
	verts, tris := ulpChainMesh(t)
	move, err := r3.Translation(r3.Vec{X: 100})
	require.NoError(t, err)
	exact := exactImages(verts, move)
	zero := make([]float64, len(verts))

	t.Run("nearest rounding stays folded after the reach search", func(t *testing.T) {
		t.Parallel()
		held := make([]r3.Vec, len(exact))
		for i, p := range exact {
			held[i] = p.Vec()
		}
		_, _, err := facetproof.KeepPlacedEmbeddedWithSummary(gaptol.With(t.Context(), reach), exact, held, tris)
		var fold *meshbool.HeldFoldError
		require.ErrorAs(t, err, &fold)
	})

	t.Run("ordered rounding keeps it embedded and charges every move", func(t *testing.T) {
		t.Parallel()
		ctx := gaptol.With(t.Context(), reach)
		placed, err := facetproof.PlaceMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, zero, 0, 0, 0, 0, move)
		require.NoError(t, err)
		for k := 1; k < len(ulpChain); k++ {
			require.NotEqual(t, placed.Verts[ulpChain[k-1]], placed.Verts[ulpChain[k]])
		}
		for i, p := range exact {
			d, _ := meshbool.CoordDistance(p, placed.Verts[i]).Float64()
			require.GreaterOrEqual(t, placed.VertexBound[i], proofbound.Radius3D(d), "vertex %d", i)
		}
		require.NotNil(t, placed.EmbeddingReach)
		require.Equal(t, reach, placed.EmbeddingReach.AllowedReach)
		require.Positive(t, placed.EmbeddingReach.DisplacementBound)
		require.LessOrEqual(t, placed.EmbeddingReach.DisplacementBound, reach)
		require.LessOrEqual(t, placed.EmbeddingReach.DisplacementBound, placed.MeshBound)
	})

	t.Run("exact-only placement refuses", func(t *testing.T) {
		t.Parallel()
		_, err := facetproof.PlaceMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris, zero, 0, 0, 0, 0, move)
		var fold *meshbool.HeldFoldError
		require.ErrorAs(t, err, &fold)
	})

	t.Run("a reach too small for the spread refuses", func(t *testing.T) {
		t.Parallel()
		ctx := gaptol.With(t.Context(), 1e-15)
		_, err := facetproof.PlaceMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, zero, 0, 0, 0, 0, move)
		var fold *meshbool.HeldFoldError
		require.ErrorAs(t, err, &fold)
	})
}

// A mesh whose exact image itself folds stays refused: the ordered stage runs,
// because the chain still merges, and the exact check still finds the crossing.
func TestPlaceMeshRefusesAGenuineFold(t *testing.T) {
	t.Parallel()
	verts, tris := ulpChainMesh(t)
	// A triangle that pierces the top face's long needles near their middle,
	// a millimetre on either side of it.
	mid := verts[0].Add(verts[ulpChain[3]]).Scale(0.5)
	n := len(verts)
	verts = append(verts,
		mid.Add(r3.Vec{X: -1, Y: -1, Z: 1}), mid.Add(r3.Vec{X: 1, Y: -1, Z: 1}), mid.Add(r3.Vec{Y: 1, Z: -1}))
	tris = append(tris, [3]int{n, n + 1, n + 2})
	move, err := r3.Translation(r3.Vec{X: 100})
	require.NoError(t, err)
	ctx := gaptol.With(t.Context(), 0.01)
	_, err = facetproof.PlaceMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, make([]float64, len(verts)), 0, 0, 0, 0, move)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var fold *meshbool.HeldFoldError
	require.ErrorAs(t, err, &fold)
}
