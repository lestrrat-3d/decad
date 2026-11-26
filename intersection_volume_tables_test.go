package decad

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestIntersectionVolumeLeavesTablesAlone snapshots the identity counters and every document table before and
// after each query: the live set, the operations and their producer and consumer indexes, the circle and
// identity witnesses and the container membership. The pairs cover the span route, the exact grid route, mesh
// operands, a body in a child container and a disjoint pair. NextProducer, NextLevel and NextCurve must not move.
func TestIntersectionVolumeLeavesTablesAlone(t *testing.T) {
	t.Parallel()
	doc := New()
	a := ivBox(t, doc, 0, 0, 10, 10, 0)
	b := ivBox(t, doc, 5, 5, 15, 15, 5)
	c := ivCylinder(t, doc, 5, 5, 3, -2, 14)
	m1 := ivMeshBox(t, ivBox(t, doc, 20, 0, 30, 10, 0))
	m2 := ivMeshBox(t, ivBox(t, doc, 24, 4, 34, 14, 3))
	far := ivBox(t, doc, 100, 100, 110, 110, 0)
	g1 := ivGridBlock(t, doc, r3.NewVec(40, 0, 0), r3.NewVec(50, 10, 10))
	g2 := ivGridBlock(t, doc, r3.NewVec(44, 3, 2), r3.NewVec(54, 13, 12))
	child, err := doc.Root().NewChild("tools")
	require.NoError(t, err)
	require.NoError(t, child.AddBody(c))

	pairs := [][2]*Body{{a, b}, {b, a}, {a, c}, {c, a}, {b, c}, {m1, m2}, {m2, m1}, {a, far}, {m1, a}, {g1, g2},
		{g2, g1}, {g1, a}}
	for _, p := range pairs {
		before := ivTablesOf(doc)
		_, _ = doc.IntersectionVolume(t.Context(), p[0], p[1])
		require.Empty(t, ivDiff(before, ivTablesOf(doc)))
	}
}

// TestIntersectionVolumeOrthogonalRouteCommitsNothing runs the query on grid blocks, whose result the exact
// orthogonal route builds, and requires that route to leave the document as it was. Intersect on the same pair
// then commits one operation and answers the queried volume.
func TestIntersectionVolumeOrthogonalRouteCommitsNothing(t *testing.T) {
	t.Parallel()
	doc := New()
	a := ivGridBlock(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	b := ivGridBlock(t, doc, r3.NewVec(4, 3, 2), r3.NewVec(14, 13, 12))
	before := ivTablesOf(doc)
	queried, err := doc.IntersectionVolume(t.Context(), a, b)
	require.NoError(t, err)
	require.Empty(t, ivDiff(before, ivTablesOf(doc)))
	require.Equal(t, []*Body{a, b}, doc.Bodies())

	got, err := Intersect(a, b).Do(t.Context())
	require.NoError(t, err)
	require.Len(t, doc.OperationHistory().Operations(), len(before.operations)+1)
	require.Equal(t, before.nextProducer+1, ivTablesOf(doc).nextProducer)
	want, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, want, queried)
}
