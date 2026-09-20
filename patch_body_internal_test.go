package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file pins the three Body.Patch cases decad's public seam admits no
// way to author directly (docs/surface-design.md §5.2), the same reason
// stitch_internal_test.go carries its own R7 (non-orientable) fixture: gate
// 3's non-planar refusal (every builder's own free-edge chains are planar by
// construction), R18's orientation disagreement (every builder leaves one
// shell consistently oriented, §2.3), and R19's no-payload receiver (every
// evaluator this package ships always sets one). Each is a hand-built
// fixture over the unexported topology types directly.

// TestProvePatchChainPlaneRejectsANonPlanarChain is docs/surface-design.md's
// T12 non-planar half: four vertices, one of them lifted out of the other
// three's plane, proven non-planar over dyadic.go's exact rational lift —
// never a residual against a fitted plane.
func TestProvePatchChainPlaneRejectsANonPlanarChain(t *testing.T) {
	t.Parallel()
	v0 := &Vertex{position: r3.NewVec(0, 0, 0)}
	v1 := &Vertex{position: r3.NewVec(1, 0, 0)}
	v2 := &Vertex{position: r3.NewVec(1, 1, 1)}
	v3 := &Vertex{position: r3.NewVec(0, 1, 0)}
	edges := []*Edge{
		{curve: Line3{}, start: v0, end: v1},
		{curve: Line3{}, start: v1, end: v2},
		{curve: Line3{}, start: v2, end: v3},
		{curve: Line3{}, start: v3, end: v0},
	}
	err := provePatchChainPlane(edges)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestProvePatchChainPlaneAcceptsAPlanarChain is
// TestProvePatchChainPlaneRejectsANonPlanarChain's control: the same four
// vertices with the fourth back in the z = 0 plane pass gate 3 cleanly,
// which is what proves the rejection above is about the lifted coordinate
// and nothing else about the fixture's shape.
func TestProvePatchChainPlaneAcceptsAPlanarChain(t *testing.T) {
	t.Parallel()
	v0 := &Vertex{position: r3.NewVec(0, 0, 0)}
	v1 := &Vertex{position: r3.NewVec(1, 0, 0)}
	v2 := &Vertex{position: r3.NewVec(1, 1, 0)}
	v3 := &Vertex{position: r3.NewVec(0, 1, 0)}
	edges := []*Edge{
		{curve: Line3{}, start: v0, end: v1},
		{curve: Line3{}, start: v1, end: v2},
		{curve: Line3{}, start: v2, end: v3},
		{curve: Line3{}, start: v3, end: v0},
	}
	require.NoError(t, provePatchChainPlane(edges))
}

// TestOrientPatchChainRejectsDisagreeingAdjacentFaces is Table R row R18: two
// free edges over the same vertex pair, each recorded in the SAME direction
// (v1 to v2) and each traversed forward by its own one adjacent face.
// Reversing each independently ("the opposite sense") then makes BOTH
// directed edges start at v2 — a duplicate orientPatchChain's own
// connectivity walk catches directly, by pigeonhole leaving v1 the start of
// none.
func TestOrientPatchChainRejectsDisagreeingAdjacentFaces(t *testing.T) {
	t.Parallel()
	v1 := &Vertex{position: r3.NewVec(0, 0, 0)}
	v2 := &Vertex{position: r3.NewVec(1, 0, 0)}
	a := &Edge{curve: Line3{}, start: v1, end: v2}
	b := &Edge{curve: Line3{}, start: v1, end: v2}
	faceA := &Face{loops: []*Loop{{coedges: []coedge{{edge: a, forward: true}}, outer: true}}}
	faceB := &Face{loops: []*Loop{{coedges: []coedge{{edge: b, forward: true}}, outer: true}}}
	a.faces = []*Face{faceA}
	b.faces = []*Face{faceB}

	_, err := orientPatchChain(bodyPatchChain{edges: []*Edge{a, b}})
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestBuildPatchFaceRemapsACrossingChainToErrDegenerate is gate 4 exercised
// directly: a bowtie quadrilateral (A, B, C, D in order) whose two diagonals
// — the two non-adjacent segment pairs a plain 4-cycle boundary walk never
// puts in contact — cross at its center. orientPatchChain's own "opposite
// sense" derivation succeeds (every builder here agrees, by construction: a
// single adjacent face per edge, each walked forward), so this reaches gate
// 4, whose ErrUnsupported crossing refusal (fillet_audit.go's
// crossingAuditBudget) is remapped to ErrDegenerate at Body.Patch's own
// boundary (docs/surface-design.md Table R row R5).
func TestBuildPatchFaceRemapsACrossingChainToErrDegenerate(t *testing.T) {
	t.Parallel()
	a := &Vertex{position: r3.NewVec(0, 0, 0)}
	b := &Vertex{position: r3.NewVec(1, 1, 0)}
	c := &Vertex{position: r3.NewVec(1, 0, 0)}
	d := &Vertex{position: r3.NewVec(0, 1, 0)}
	e1 := &Edge{curve: Line3{}, start: a, end: b}
	e2 := &Edge{curve: Line3{}, start: b, end: c}
	e3 := &Edge{curve: Line3{}, start: c, end: d}
	e4 := &Edge{curve: Line3{}, start: d, end: a}
	edges := []*Edge{e1, e2, e3, e4}
	edgeCopy := map[*Edge]*Edge{}
	for _, e := range edges {
		f := &Face{loops: []*Loop{{coedges: []coedge{{edge: e, forward: true}}, outer: true}}}
		e.faces = []*Face{f}
		edgeCopy[e] = e
	}

	_, err := buildPatchFace(context.Background(), 0, bodyPatchChain{edges: edges}, edgeCopy)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestBodyPatchRejectsAReceiverWithNoPayload is Table R row R19: a live body
// this evaluator did not build (payload nil) — unreachable through the
// public seam, since every builder here always sets one, on the same terms
// Placed/PlacedCopy/Duplicate already require.
func TestBodyPatchRejectsAReceiverWithNoPayload(t *testing.T) {
	t.Parallel()
	d := New()
	b := &Body{doc: d, origin: FeatureRef{Role: roleBody}, kind: BodySheet}
	d.bodies = append(d.bodies, b)

	_, err := b.Patch(Edges(Free()))
	require.ErrorIs(t, err, ErrUnsupported)
}
