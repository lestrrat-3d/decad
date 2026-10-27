package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §7's ExtremeFacesByCentroid tests on a mesh solid.

func prismSolid(t *testing.T) *decad.Body {
	t.Helper()
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 6, 0),
		r3.NewVec(0, 0, 4), r3.NewVec(10, 0, 4), r3.NewVec(0, 6, 4),
	}
	tris := [][3]int{{0, 2, 1}, {3, 4, 5}, {0, 1, 4}, {0, 4, 3}, {0, 3, 5}, {0, 5, 2}, {1, 2, 5}, {1, 5, 4}}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	return body
}

func TestExtremeFacesByCentroidRanksAMeshSolidsFaces(t *testing.T) {
	t.Parallel()
	body := prismSolid(t)
	// Along (1, 1, 0) the face centroids project to 3 (the x = 0 face), 16/3 (the bottom and top), 5 (the y = 0 face) and
	// 8 (the slanted face), so the least is the x = 0 face and the greatest the slanted one.
	least, greatest, err := body.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 1, 0))
	require.NoError(t, err)
	c, err := least.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0, c.Value.X, 1e-12)
	require.InDelta(t, 3, c.Value.Y, 1e-12)
	c, err = greatest.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5, c.Value.X, 1e-12)
	require.InDelta(t, 3, c.Value.Y, 1e-12)
}

func TestExtremeFacesByCentroidRefusesWhatTheReadingsLeaveTied(t *testing.T) {
	t.Parallel()
	body := prismSolid(t)
	// Along z the bottom (0) and top (4) are strictly extreme, the four side faces all project to 2.
	least, greatest, err := body.ExtremeFacesByCentroid(t.Context(), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	c, err := least.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0, c.Value.Z, 1e-12)
	c, err = greatest.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 4, c.Value.Z, 1e-12)
	// Along x the y = 0 and slanted faces tie at 5 for the greatest.
	least, greatest, err = body.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 0, 0))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, least)
	require.Nil(t, greatest)
}
