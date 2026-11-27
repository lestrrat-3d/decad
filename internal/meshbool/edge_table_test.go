package meshbool_test

// CI: isolated-tests

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestEdgeTableDifferential(t *testing.T) {
	for seed := range uint64(40) {
		r := rand.New(rand.NewPCG(seed, 0xed9e))
		ref := map[[2]int]int{}
		tab := meshbool.NewEdgeTable(r.IntN(4) * 3) // often too small, forcing growth
		small := 8 + int(seed)*5                    // vertex range, so duplicates are common
		pick := func() int {
			if r.IntN(10) == 0 {
				return edgeKeyPool[r.IntN(len(edgeKeyPool))]
			}
			return r.IntN(small)
		}
		for step := range 4000 {
			a, b := pick(), pick()
			switch op := r.IntN(10); {
			case op < 6:
				v := r.IntN(1000)
				switch r.IntN(20) {
				case 0:
					v = math.MaxInt32 + 1 + r.IntN(5) // does not fit int32
				case 1:
					v = -1 - r.IntN(5)
				}
				ref[[2]int{a, b}] = v
				tab.Set(a, b, v)
			case op < 8:
				_, had := ref[[2]int{a, b}]
				delete(ref, [2]int{a, b})
				require.Equal(t, had, tab.Delete(a, b), "seed %d step %d delete", seed, step)
			default:
				want, wok := ref[[2]int{a, b}]
				got, gok := tab.Get(a, b)
				require.Equal(t, wok, gok, "seed %d step %d presence", seed, step)
				require.Equal(t, want, got, "seed %d step %d value", seed, step)
			}
			require.Equal(t, len(ref), tab.Len(), "seed %d step %d len", seed, step)
		}
		for k, want := range ref {
			got, ok := tab.Get(k[0], k[1])
			require.True(t, ok)
			require.Equal(t, want, got)
		}
		// Absent edges stay absent, including the reverse of every present edge.
		for k := range ref {
			if _, ok := ref[[2]int{k[1], k[0]}]; ok {
				continue
			}
			_, ok := tab.Get(k[1], k[0])
			require.False(t, ok)
		}
	}
}

func TestEdgeTableGrowthAndDeleteAll(t *testing.T) {
	tab := meshbool.NewEdgeTable(0)
	const n = 20000
	for i := range n {
		tab.Set(i, i+1, i)
		tab.Set(i, i+1, i+1) // overwrite: last write wins
	}
	require.Equal(t, n, tab.Len())
	for i := range n {
		v, ok := tab.Get(i, i+1)
		require.True(t, ok)
		require.Equal(t, i+1, v)
		_, ok = tab.Get(i+1, i)
		require.False(t, ok)
	}
	for i := range n {
		require.True(t, tab.Delete(i, i+1))
		require.False(t, tab.Delete(i, i+1))
	}
	require.Equal(t, 0, tab.Len())
	tab.Set(5, 6, 7)
	v, ok := tab.Get(5, 6)
	require.True(t, ok)
	require.Equal(t, 7, v)
}

func TestEdgeTableNilAndSentinel(t *testing.T) {
	var nilTab *meshbool.EdgeTable
	_, ok := nilTab.Get(0, 1)
	require.False(t, ok)
	require.Equal(t, 0, nilTab.Len())

	// The largest in-range edge packs to 0x7fffffff7fffffff, not the empty
	// marker, and the all-ones pattern a=-1,b=-1 goes to the side map.
	tab := meshbool.NewEdgeTable(1)
	tab.Set(1<<31-1, 1<<31-1, 4)
	tab.Set(-1, -1, 5)
	v, ok := tab.Get(1<<31-1, 1<<31-1)
	require.True(t, ok)
	require.Equal(t, 4, v)
	v, ok = tab.Get(-1, -1)
	require.True(t, ok)
	require.Equal(t, 5, v)
	_, ok = tab.Get(0, 0)
	require.False(t, ok)
}

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

func TestEdgeTableClear(t *testing.T) {
	tab := meshbool.NewEdgeTable(4)
	for i := range 100 {
		tab.Set(i, i+1, i)
	}
	tab.Set(-1, 2, 5) // side map entry
	tab.Clear()
	require.Equal(t, 0, tab.Len())
	_, ok := tab.Get(3, 4)
	require.False(t, ok)
	_, ok = tab.Get(-1, 2)
	require.False(t, ok)
	tab.Set(3, 4, 9)
	v, ok := tab.Get(3, 4)
	require.True(t, ok)
	require.Equal(t, 9, v)
	require.Equal(t, 1, tab.Len())
}
