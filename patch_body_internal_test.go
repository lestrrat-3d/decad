package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
	_, hasAxialBound, err := provePatchChainPlane(edges)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, hasAxialBound)
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
	_, hasAxialBound, err := provePatchChainPlane(edges)
	require.NoError(t, err)
	require.False(t, hasAxialBound, "the exact arm admitted this chain, so it publishes no axial bound")
}

// boundedSquareChain builds a 4-edge square chain at z = 10 with every vertex
// and every edge stamped lvl, and every vertex carrying bound — the shape a
// straight prism's own rim carries once gate 3's first (exact) arm has
// already been shown to refuse it. lvl == 0 reproduces what an untouched
// builder (a revolve seam, a cap-loop chamfer, a one-cap shell's open rim)
// leaves behind: a bounded chain with no certificate.
func boundedSquareChain(lvl levelID, bound units.Value) []*Edge {
	v0 := &Vertex{position: r3.NewVec(0, 0, 10), bound: bound, level: lvl}
	v1 := &Vertex{position: r3.NewVec(1, 0, 10), bound: bound, level: lvl}
	v2 := &Vertex{position: r3.NewVec(1, 1, 10), bound: bound, level: lvl}
	v3 := &Vertex{position: r3.NewVec(0, 1, 10), bound: bound, level: lvl}
	return []*Edge{
		{curve: Line3{}, start: v0, end: v1, level: lvl},
		{curve: Line3{}, start: v1, end: v2, level: lvl},
		{curve: Line3{}, start: v2, end: v3, level: lvl},
		{curve: Line3{}, start: v3, end: v0, level: lvl},
	}
}

// TestProvePatchChainPlaneAdmitsABoundedCoLevelChain is
// docs/surface-design.md §15's T27: gate 3's second (LEVEL) arm admits a
// bounded chain whose every vertex and edge shares one non-zero levelID —
// the shape a straight prism build (prism_build.go) stamps onto a rim at one
// swept end.
func TestProvePatchChainPlaneAdmitsABoundedCoLevelChain(t *testing.T) {
	t.Parallel()
	const lvl levelID = 1
	edges := boundedSquareChain(lvl, units.Millimeters(0.002))

	bound, hasAxialBound, err := provePatchChainPlane(edges)
	require.NoError(t, err)
	require.True(t, hasAxialBound, "the level arm admitted this chain, so it publishes the chain's own axial bound")
	require.Equal(t, 0.002, bound)
}

// TestProvePatchChainPlaneRefusesABoundedChainWithNoLevelToken is
// docs/surface-design.md §15's T27 mirror: the identical bounded shape with
// no level token at all — levelID's own zero value, "no certificate" — stays
// refused on gate 3's existing row, exactly as it was before this
// certificate existed. This is what pins T12's own "a bounded chain that
// carries no shared level token" half: a real revolve seam or a one-cap
// shell's open rim never carries a level token, since neither builder mints
// one, and neither is reachable through decad's public seam as a Body.Patch
// fixture without changing an untouched file.
func TestProvePatchChainPlaneRefusesABoundedChainWithNoLevelToken(t *testing.T) {
	t.Parallel()
	edges := boundedSquareChain(0, units.Millimeters(0.002))

	bound, hasAxialBound, err := provePatchChainPlane(edges)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, hasAxialBound)
	require.Zero(t, bound)
}

// TestProvePatchChainPlaneRefusesIdenticalBoundedVerticesUnderDifferentLevels
// is docs/surface-design.md §15's T28, and the proof this certificate is an
// IDENTITY, never a tolerance: v1 and v1Twin hold bit-identical coordinates
// and bit-identical, nonzero bounds — exactly what two independently built
// prisms' matching rim corners would hold, sweeping the same profile to the
// same extent — yet v1Twin was minted under a different levelID. Splicing it
// into an otherwise single-level chain still refuses: nothing here ever
// compares the coordinate or the bound, only the identity, so bit-identical
// held data is not what admits a chain.
//
// This shape is not reachable through Body.Patch's own public seam: a chain
// requires two consecutive edges to SHARE a vertex POINTER, which only one
// evaluator's own build or a zero-bound weld creates, and Table J refuses a
// zero-bound weld of a bounded pair (docs/surface-design.md §6.2) — so no
// selector over any two live bodies can ever hand Body.Patch one chain
// spanning two independently minted levels. Pinning it here, directly
// against provePatchChainPlane, is what the brief's own fallback asks for.
func TestProvePatchChainPlaneRefusesIdenticalBoundedVerticesUnderDifferentLevels(t *testing.T) {
	t.Parallel()
	const levelA, levelB levelID = 1, 2
	bound := units.Millimeters(0.002)

	v1 := &Vertex{position: r3.NewVec(1, 0, 10), bound: bound}
	v1Twin := &Vertex{position: r3.NewVec(1, 0, 10), bound: bound, level: levelB}
	require.Equal(t, v1.position, v1Twin.position, "the two vertices' held coordinates are bit-identical")
	require.Equal(t, v1.bound, v1Twin.bound, "the two vertices' held bounds are bit-identical")

	v0 := &Vertex{position: r3.NewVec(0, 0, 10), bound: bound, level: levelA}
	v2 := &Vertex{position: r3.NewVec(1, 1, 10), bound: bound, level: levelA}
	v3 := &Vertex{position: r3.NewVec(0, 1, 10), bound: bound, level: levelA}
	edges := []*Edge{
		{curve: Line3{}, start: v0, end: v1Twin, level: levelA},
		{curve: Line3{}, start: v1Twin, end: v2, level: levelA},
		{curve: Line3{}, start: v2, end: v3, level: levelA},
		{curve: Line3{}, start: v3, end: v0, level: levelA},
	}

	_, hasAxialBound, err := provePatchChainPlane(edges)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, hasAxialBound)
}

// TestBuildPatchFaceCarriesTheLevelBoundOntoTheNewFacesAxialDelta is
// docs/surface-design.md §15's T29: buildPatchFace over a chain gate 3's
// level arm admitted sets the new face's own axialDelta/hasAxialDelta from
// the chain's proven bound — the same fields a prism cap already carries
// (prism_build.go) — rather than leaving them at their zero value.
func TestBuildPatchFaceCarriesTheLevelBoundOntoTheNewFacesAxialDelta(t *testing.T) {
	t.Parallel()
	edges := boundedSquareChain(1, units.Millimeters(0.002))
	edgeCopy := map[*Edge]*Edge{}
	for _, e := range edges {
		f := &Face{loops: []*Loop{{coedges: []coedge{{edge: e, forward: true}}, outer: true}}}
		e.faces = []*Face{f}
		edgeCopy[e] = e
	}

	chain := bodyPatchChain{edges: edges, axialBound: 0.002, hasAxialBound: true}
	face, err := buildPatchFace(context.Background(), 0, chain, edgeCopy)
	require.NoError(t, err)
	require.True(t, face.hasAxialDelta, "the new face carries the level arm's own axial bound")
	require.Equal(t, 0.002, face.axialDelta)
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
