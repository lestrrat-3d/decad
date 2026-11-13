package meshbool

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestParityIndexBuildsOnSecondEligibleSweep(t *testing.T) {
	t.Parallel()
	verts, tris := parityCylinder(64)
	all := parityAllFacets(len(tris))
	plain := NewParityMesh(verts, tris)
	plain.Unfiltered = true

	onMesh := proof.XptOf(r3.Vec{X: 100, Y: 0.125, Z: 0.25})
	want := answerOf(MeshParityPreparedContext(t.Context(), onMesh, plain, all))
	pm := NewParityMesh(verts, tris)
	got := answerOf(MeshParityAllContext(t.Context(), onMesh, pm))
	require.Equal(t, want, got)
	require.True(t, pm.indexDeferred[0], `the completed first x query must be recorded`)
	require.False(t, pm.indexBuilt[0], `the first x query must use the plain scan`)

	pAmbiguous := ratXpt(big.NewRat(0, 1), big.NewRat(3, 1), big.NewRat(1, 2))
	ambiguousMesh := NewParityMesh(verts, tris)
	want = answerOf(MeshParityPreparedContext(t.Context(), pAmbiguous, plain, all))
	got = answerOf(MeshParityAllContext(t.Context(), pAmbiguous, ambiguousMesh))
	require.Equal(t, want, got, `the ambiguous first ray must match the plain scan`)
	require.Contains(t, got.err, `every parity ray was ambiguous`)
	for axis := range 3 {
		require.True(t, ambiguousMesh.indexDeferred[axis], `axis %d must complete its first plain sweep`, axis)
		require.True(t, ambiguousMesh.indexBuilt[axis], `axis %d must build on its opposite sweep`, axis)
		require.NotNil(t, ambiguousMesh.index[axis], `axis %d index must be usable`, axis)
	}

	direct := NewParityMesh(verts, tris)
	require.NotNil(t, direct.parityIndex(0, 1, 2), `a direct index request must still build immediately`)
	require.Equal(t, answerOf(MeshParityPreparedContext(t.Context(), onMesh, plain, all)),
		answerOf(MeshParityAllContext(t.Context(), onMesh, direct)))
	require.False(t, direct.indexDeferred[0], `a prebuilt index must remain available on the first sweep`)
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
			if err != nil || !pm.indexDeferred[0] || pm.indexBuilt[0] {
				b.Fatalf(`unexpected one-use result: err=%v deferred=%v built=%v`, err,
					pm.indexDeferred[0], pm.indexBuilt[0])
			}
		}
	})
	b.Run(`second-use-build`, func(b *testing.B) {
		b.ReportAllocs()
		for range b.N {
			b.StopTimer()
			pm := NewParityMesh(verts, tris)
			if _, _, err := MeshParityAllContext(ctx, p, pm); err != nil {
				b.Fatal(err)
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
