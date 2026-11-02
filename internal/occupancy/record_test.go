package occupancy_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLocalBulgeReadsTheHoleWall(t *testing.T) {
	t.Parallel()
	// A leg of radius 3 along Z from 0 to 40, bored across by a tool of radius
	// 2.2 along X at height 20, whose held mesh carries a bound of 0.001.
	leg, err := occupancy.CircleLeaf(occupancy.Chain{{M: [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}},
		0, 0, 3, 0, 40, 0, 0, 0, 0, 128, 0.001)
	require.NoError(t, err)
	acrossX := occupancy.Chain{{M: [3][3]float64{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}, T: [3]float64{-10, 0, 20}}}
	tool, err := occupancy.CircleLeaf(acrossX, 0, 0, 2.2, 0, 20, 0, 0, 0, 0, 128, 0.001)
	require.NoError(t, err)
	rec := occupancy.CutCompose(occupancy.LeafRecord(leg), occupancy.LeafRecord(tool), 0, 0, nil)
	require.NotNil(t, rec)

	// A wall facet of the leg, inside the leg's circle, near the bend end.
	facet := [3]r3.Vec{{X: 2.9, Z: 2}, {X: 2.9, Y: 0.1, Z: 2}, {X: 2.9, Z: 3}}
	far := [2]r3.Vec{{X: 2.8, Y: -0.1, Z: 1.9}, {X: 3, Y: 0.2, Z: 3.1}}
	require.Less(t, rec.LocalBulge(facet, 0.01, far), 1e-9, "a contact far from the bore's wall reads no hole bulge")

	// The same reading at a contact on the bore's wall reads the tool's bound.
	wall := [2]r3.Vec{{X: 2.5, Y: -0.1, Z: 22.1}, {X: 2.7, Y: 0.1, Z: 22.3}}
	require.GreaterOrEqual(t, rec.LocalBulge(facet, 0.01, wall), 0.001)
}

func TestPiecesNearBuildsOnlyNearCells(t *testing.T) {
	t.Parallel()
	const radius, rho = 2.4, 0.6
	o, w, e0, e1 := torusFrame()
	leaf, err := occupancy.ArcTubeLeaf(o, w, e0, e1, radius, rho, 0, 0, true, 0, 1e-4, 64, 0.01)
	require.NoError(t, err)
	all, err := leaf.Pieces()
	require.NoError(t, err)
	rec := occupancy.LeafRecord(leaf)
	at := torusPoint(o, w, e0, e1, rho, 0.5, 0)
	near, err := rec.PiecesNear([][2]r3.Vec{{at, at}}, 0.05)
	require.NoError(t, err)
	require.NotEmpty(t, near)
	require.Less(t, len(near), len(all)/10, "only the cells near the query are built")
	// Every piece that holds the query point is among them.
	for _, p := range all {
		ok, err := occupancy.Misses([3]r3.Vec{at, at, at}, r3.Vec{}, p, &occupancy.Budget{Limit: 1 << 40})
		require.NoError(t, err)
		if !ok {
			require.Contains(t, near, p)
		}
	}
	far, err := rec.PiecesNear([][2]r3.Vec{{o.Add(w.Scale(50)), o.Add(w.Scale(50))}}, 0.05)
	require.NoError(t, err)
	require.Empty(t, far)
}
