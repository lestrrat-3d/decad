package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// classify runs the pair classifier on two float triangles, lifting them the
// way the mesh pass does.
func classify(t *testing.T, ta, tb [3]r3.Vec) triContact {
	t.Helper()
	xta := [3]xpt{xptOf(ta[0]), xptOf(ta[1]), xptOf(ta[2])}
	xtb := [3]xpt{xptOf(tb[0]), xptOf(tb[1]), xptOf(tb[2])}
	na := xcross(xsub(xta[1], xta[0]), xsub(xta[2], xta[0]))
	nb := xcross(xsub(xtb[1], xtb[0]), xsub(xtb[2], xtb[0]))
	c, err := triTriClassify(ta, tb, xta, xtb, na, nb)
	require.NoError(t, err)
	return c
}

func TestTriTriClassifyIsSymmetric(t *testing.T) {
	// The pair below is the one the old branch grid dropped: two of A's vertices
	// lie on B's plane, and B's vertex (0, 0, 0) sits strictly INSIDE the A edge
	// they span. The old code, having entered the "two of A's vertices are on the
	// plane" cell, only ever looked for a touch among A's OWN vertices — and
	// neither of them lies on B — so it reported no contact at all, dropping a
	// real point contact. Asking what the intersection IS, rather than whose
	// geometry to look on, cannot make that mistake: it is one point, whichever
	// way round the pair is handed in.
	a := [3]r3.Vec{{X: 0, Y: 0, Z: -5}, {X: 0, Y: 0, Z: 5}, {X: 12, Y: -4, Z: 5}}
	b := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: -12, Y: 0, Z: -6}, {X: -12, Y: 0, Z: 5}}

	fwd := classify(t, a, b)
	require.Equal(t, contactPoint, fwd.kind)
	require.Equal(t, r3.Vec{}, fwd.p0.vec(), `the contact is the origin`)
	// The point lies on A's boundary (inside its edge) AND on B's (its corner).
	require.True(t, fwd.p0OnA)
	require.True(t, fwd.p0OnB)

	rev := classify(t, b, a)
	require.Equal(t, contactPoint, rev.kind)
	require.Equal(t, fwd.p0.vec(), rev.p0.vec(), `the answer does not depend on the argument order`)
	require.Equal(t, fwd.p0OnA, rev.p0OnB)
	require.Equal(t, fwd.p0OnB, rev.p0OnA)
}

func TestTriTriClassifyNamesTheInPlaneEdge(t *testing.T) {
	// A's edge 0 lies exactly in B's plane (z = 0) and crosses B's interior: the
	// contact is a segment, and it runs ALONG that edge. The pair reports WHICH
	// edge and stops there — whether the edge grazes B or crosses it is decided
	// by the edge's two adjacent facets, which no pair can see.
	a := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 4, Y: 0, Z: 0}, {X: 2, Y: 1, Z: 3}}
	b := [3]r3.Vec{{X: -2, Y: -2, Z: 0}, {X: 6, Y: -2, Z: 0}, {X: 2, Y: 6, Z: 0}}

	c := classify(t, a, b)
	require.Equal(t, contactSegment, c.kind)
	require.Equal(t, 0, c.edgeA, `the contact runs along A's edge 0`)
	require.Equal(t, -1, c.edgeB, `it crosses B's interior, along no edge of B`)
	// Both endpoints are A's own corners, so both lie on A's boundary; both lie
	// strictly inside B, so neither is on B's.
	require.True(t, c.p0OnA)
	require.True(t, c.p1OnA)
	require.False(t, c.p0OnB)
	require.False(t, c.p1OnB)
}
