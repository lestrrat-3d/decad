package meshbool_test

// CI: isolated-tests

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFacetAdjacencyMatchesMapReference(t *testing.T) {
	for name, tris := range edgeTableMeshes() {
		got, err := meshbool.FacetAdjacencyContext(t.Context(), tris)
		require.NoError(t, err, name)
		want := referenceAdjacency(tris)
		require.Len(t, got, len(want), name)
		for i := range want {
			// A nil slice and an empty one both mean "no neighbors".
			if len(want[i]) == 0 && len(got[i]) == 0 {
				continue
			}
			require.True(t, slices.Equal(want[i], got[i]), "%s facet %d: want %v got %v", name, i, want[i], got[i])
		}
	}
}

func TestPrepOwnerMatchesMapReference(t *testing.T) {
	// A prepared operand's Owner table must answer exactly as a map built by
	// the old rule would, including TwinFacet.
	n := 6
	var verts []r3.Vec
	for iy := 0; iy <= n; iy++ {
		for ix := 0; ix <= n; ix++ {
			verts = append(verts, r3.NewVec(float64(ix), float64(iy), math.Sin(float64(ix+2*iy))))
		}
	}
	var tris [][3]int
	for iy := range n {
		for ix := range n {
			v00 := iy*(n+1) + ix
			v10, v01, v11 := v00+1, v00+n+1, v00+n+2
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	bm, err := meshbool.PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	ref := map[[2]int]int{}
	for i, tri := range tris {
		for k := range 3 {
			ref[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	require.Equal(t, len(ref), bm.Owner.Len())
	for e, want := range ref {
		got, ok := bm.Owner.Get(e[0], e[1])
		require.True(t, ok)
		require.Equal(t, want, got)
	}
	adj, err := meshbool.FacetAdjacencyContext(t.Context(), tris)
	require.NoError(t, err)
	for i := range tris {
		var twins []int
		for k := range 3 {
			if twin, ok := bm.TwinFacet(i, k); ok {
				twins = append(twins, twin)
			}
		}
		require.True(t, slices.Equal(adj[i], twins), "facet %d: adjacency %v twins %v", i, adj[i], twins)
	}
}

func TestMeshBooleanSameWithReusedOwner(t *testing.T) {
	// keepSideWith reuses the prepared Owner when it describes m.Tris and
	// rebuilds adjacency otherwise. Cloning Tris after prep forces the
	// rebuild; both paths must keep the same facets.
	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut, meshbool.OpIntersect} {
		requireSameKept(t, op)
	}
}

func requireSameKept(t *testing.T, op meshbool.OperationKind) {
	t.Helper()
	build := func(cloneTris bool) ([]meshbool.KeptFacet, float64) {
		a := tiltedCube(t, 2, 0.11, 0.07, r3.Vec{})
		b := tiltedCube(t, 1.5, 0.61, 0.43, r3.Vec{X: 0.37, Y: 0.21, Z: 0.53})
		if cloneTris {
			a.Tris = slices.Clone(a.Tris)
			b.Tris = slices.Clone(b.Tris)
		}
		kept, rim, err := meshbool.MeshBoolean(t.Context(), op, a, b, meshbool.NewContactMemo(a, b))
		require.NoError(t, err)
		return kept, rim
	}
	reused, rimR := build(false)
	rebuilt, rimB := build(true)
	require.NotEmpty(t, reused)
	require.Equal(t, rimB, rimR)
	require.Equal(t, len(rebuilt), len(reused))
	for i := range rebuilt {
		require.Equal(t, rebuilt[i].Facet, reused[i].Facet, "kept %d facet", i)
		require.Equal(t, rebuilt[i].Src, reused[i].Src, "kept %d src", i)
		require.Equal(t, rebuilt[i].Keys, reused[i].Keys, "kept %d keys", i)
	}
}
