package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T11-shaped internal coverage: the
// two hand-built fixtures decad's public seam admits no way to author
// directly, because every real operand Stitch can be handed is already
// locally orientable and manifold by its own construction. Both fixtures
// call the package-private step they exercise directly, on a hand-built
// face set, rather than faking one through Stitch's public entry — the same
// treatment the design brief requires of the non-orientable case, extended
// here to its non-manifold sibling and to a self-overlapping closed
// assembly (R9).

// TestStitchOrientationRefusesMobiusAssembly is Table R row R7's
// orientation-contradiction path: three faces glued pairwise along three
// edges, each shared edge walked FORWARD by both its adjacent faces. That
// is the combinatorial shape of a Möbius strip's simplicial boundary: no
// assignment of a per-face flip can make every shared edge see one forward
// and one backward use, so deriveStitchOrientation must refuse rather than
// publish an inconsistent choice.
func TestStitchOrientationRefusesMobiusAssembly(t *testing.T) {
	t.Parallel()
	e12 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	e23 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	e31 := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}

	f1 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e12, forward: true}, {edge: e31, forward: true},
	}}}}
	f2 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e12, forward: true}, {edge: e23, forward: true},
	}}}}
	f3 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{
		{edge: e23, forward: true}, {edge: e31, forward: true},
	}}}}
	e12.faces = []*Face{f1, f2}
	e23.faces = []*Face{f2, f3}
	e31.faces = []*Face{f3, f1}

	err := deriveStitchOrientation([]*Face{f1, f2, f3})
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestStitchClosureRefusesNonManifoldEdge is Table R row R7's other path:
// one edge shared by three faces. Every pairwise contact test the reused
// crossing audit runs would pass for three triangles meeting only along one
// shared edge — the audit decides CONTACT, never adjacency counts — so this
// is exactly the gap the explicit directed-edge parity leg closes
// (correction 1, docs/surface-design.md §6.4).
func TestStitchClosureRefusesNonManifoldEdge(t *testing.T) {
	t.Parallel()
	e := &Edge{curve: Line3{}, start: &Vertex{}, end: &Vertex{}}
	f1 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: true}}}}}
	f2 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: false}}}}}
	f3 := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: e, forward: true}}}}}
	e.faces = []*Face{f1, f2, f3}

	err := checkStitchClosure([]*Face{f1, f2, f3})
	require.ErrorIs(t, err, ErrDegenerate)
}

// stitchTestSquareFace builds one free-standing, self-contained planar
// square face — its own four fresh vertices and edges, all zero-bound, one
// outer loop walked counter-clockwise as viewed from +Z with reversed
// false, exactly the invariant every real wall or patch face already
// satisfies (prism_build.go, patch.go): CCW-as-stored viewed from the
// face's own +frame.N(), reversed only ever toggled by a LATER derivation,
// never set true at first build. corner is the loop's first vertex, and the
// square runs corner -> (corner+10,0,0) -> (corner+10,10,0) -> (corner,10,0)
// in the z=corner.Z plane.
func stitchTestSquareFace(corner r3.Vec) *Face {
	c := [4]r3.Vec{
		corner,
		corner.Add(r3.NewVec(10, 0, 0)),
		corner.Add(r3.NewVec(10, 10, 0)),
		corner.Add(r3.NewVec(0, 10, 0)),
	}
	verts := make([]*Vertex, 4)
	for i, p := range c {
		verts[i] = &Vertex{position: p}
	}
	edges := make([]*Edge, 4)
	for i := range 4 {
		edges[i] = &Edge{curve: Line3{}, start: verts[i], end: verts[(i+1)%4]}
	}
	frame, err := r3.NewFrame(c[0], r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		panic(err)
	}
	f := &Face{
		surface: Plane{Frame: frame},
		loops: []*Loop{{outer: true, coedges: []coedge{
			{edge: edges[0], forward: true},
			{edge: edges[1], forward: true},
			{edge: edges[2], forward: true},
			{edge: edges[3], forward: true},
		}}},
		area: 100,
	}
	for _, e := range edges {
		e.faces = []*Face{f}
	}
	return f
}

// TestStitchAuditRefusesOverlappingAssembly is Table R row R9: two
// free-standing squares built at the identical location (stitchTestSquareFace
// twice over the same corner, so every one of the four vertex pairs is
// bit-identical and zero-bound) weld along all four edges into a
// combinatorially valid, consistently orientable closed shell — exactly the
// shape deriveStitchOrientation and checkStitchClosure both admit — whose
// two faces nonetheless occupy the SAME plane region. The reused crossing
// audit is what catches this: every pairwise contact it decides for these
// two faces' triangles is either a full vertex-set match or a coplanar
// overlap, neither of which is the pair's own expected contact, so
// evalStitchContext must surface the audit's ErrDegenerate unchanged rather
// than publish a zero-volume "solid". decad's public seam has no way to
// author two independent operand sheets at the identical location this
// directly, so this is a hand-built fixture fed straight to the evaluator.
func TestStitchAuditRefusesOverlappingAssembly(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(0, 0, 0)
	a := stitchTestSquareFace(origin)
	b := stitchTestSquareFace(origin)

	plan := buildStitchWeldPlan([]*Face{a, b})
	d := New()
	_, err := evalStitchContext(context.Background(), d, d.nextProducerID(), []*Face{a, b}, plan, r3.Identity())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Empty(t, d.Bodies())
}
