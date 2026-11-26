package meshbool

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestParityIndexBuildsAfterDeferScans(t *testing.T) {
	t.Parallel()
	verts, tris := parityCylinder(64)
	all := parityAllFacets(len(tris))
	plain := NewParityMesh(verts, tris)
	plain.Unfiltered = true

	onMesh := proof.XptOf(r3.Vec{X: 100, Y: 0.125, Z: 0.25})
	want := answerOf(MeshParityPreparedContext(t.Context(), onMesh, plain, all))
	pm := NewParityMesh(verts, tris)
	for i := 1; i <= ParityIndexDeferScans; i++ {
		require.Equal(t, want, answerOf(MeshParityAllContext(t.Context(), onMesh, pm)), `query %d`, i)
		require.Equal(t, i, pm.plainScans[0], `query %d: the completed x query must be counted`, i)
		require.False(t, pm.indexBuilt[0], `query %d: x queries up to the threshold must use the plain scan`, i)
	}
	require.Equal(t, want, answerOf(MeshParityAllContext(t.Context(), onMesh, pm)))
	require.True(t, pm.indexBuilt[0], `the query after the threshold must build the index`)
	require.NotNil(t, pm.index[0])
	require.Equal(t, ParityIndexDeferScans, pm.plainScans[0], `an indexed query is not a plain scan`)

	pAmbiguous := ratXpt(big.NewRat(0, 1), big.NewRat(3, 1), big.NewRat(1, 2))
	ambiguousMesh := NewParityMesh(verts, tris)
	want = answerOf(MeshParityPreparedContext(t.Context(), pAmbiguous, plain, all))
	// Each ambiguous query sweeps both senses of every axis, so one query
	// completes two plain scans per axis.
	for i := 1; i <= ParityIndexDeferScans/2; i++ {
		got := answerOf(MeshParityAllContext(t.Context(), pAmbiguous, ambiguousMesh))
		require.Equal(t, want, got, `the ambiguous query %d must match the plain scan`, i)
		require.Contains(t, got.err, `every parity ray was ambiguous`)
		for axis := range 3 {
			require.Equal(t, 2*i, ambiguousMesh.plainScans[axis], `query %d axis %d`, i, axis)
			require.False(t, ambiguousMesh.indexBuilt[axis], `query %d axis %d must stay plain`, i, axis)
		}
	}
	require.Equal(t, want, answerOf(MeshParityAllContext(t.Context(), pAmbiguous, ambiguousMesh)))
	for axis := range 3 {
		require.True(t, ambiguousMesh.indexBuilt[axis], `axis %d must build after the threshold`, axis)
		require.NotNil(t, ambiguousMesh.index[axis], `axis %d index must be usable`, axis)
	}

	direct := NewParityMesh(verts, tris)
	require.NotNil(t, direct.parityIndex(0, 1, 2), `a direct index request must still build immediately`)
	require.Equal(t, answerOf(MeshParityPreparedContext(t.Context(), onMesh, plain, all)),
		answerOf(MeshParityAllContext(t.Context(), onMesh, direct)))
	require.Zero(t, direct.plainScans[0], `a prebuilt index must remain available on the first sweep`)
}

func BenchmarkParityIndexDeferredQuery(b *testing.B) {
	verts, tris := parityCylinder(64)
	p := proof.XptOf(r3.Vec{X: 100, Y: 0.125, Z: 0.25})
	ctx := b.Context()
	b.Run(`one-use`, func(b *testing.B) {
		b.ReportAllocs()
		for range b.N {
			b.StopTimer()
			pm := NewParityMesh(verts, tris)
			b.StartTimer()
			_, _, err := MeshParityAllContext(ctx, p, pm)
			b.StopTimer()
			if err != nil || pm.plainScans[0] != 1 || pm.indexBuilt[0] {
				b.Fatalf(`unexpected one-use result: err=%v scans=%d built=%v`, err,
					pm.plainScans[0], pm.indexBuilt[0])
			}
		}
	})
	b.Run(`second-use-build`, func(b *testing.B) {
		b.ReportAllocs()
		for range b.N {
			b.StopTimer()
			pm := NewParityMesh(verts, tris)
			for range ParityIndexDeferScans {
				if _, _, err := MeshParityAllContext(ctx, p, pm); err != nil {
					b.Fatal(err)
				}
			}
			b.StartTimer()
			_, _, err := MeshParityAllContext(ctx, p, pm)
			b.StopTimer()
			if err != nil || !pm.indexBuilt[0] || pm.index[0] == nil {
				b.Fatalf(`unexpected second-use result: err=%v built=%v index=%v`, err,
					pm.indexBuilt[0], pm.index[0] != nil)
			}
		}
	})
}
