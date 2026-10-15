package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the three PatchEdges cases decad's public seam admits no
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
	v0 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 0, 0)}})
	v1 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 0)}})
	v2 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 1, 1)}})
	v3 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 1, 0)}})
	edges := []*Edge{
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v0), End: vertexNode(v1)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v1), End: vertexNode(v2)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v2), End: vertexNode(v3)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v3), End: vertexNode(v0)}}),
	}
	_, _, hasAxialBound, err := provePatchChainPlane(edges)
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
	v0 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 0, 0)}})
	v1 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 0)}})
	v2 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 1, 0)}})
	v3 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 1, 0)}})
	edges := []*Edge{
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v0), End: vertexNode(v1)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v1), End: vertexNode(v2)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v2), End: vertexNode(v3)}}),
		bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(v3), End: vertexNode(v0)}}),
	}
	_, _, hasAxialBound, err := provePatchChainPlane(edges)
	require.NoError(t, err)
	require.False(t, hasAxialBound, "the exact arm admitted this chain, so it publishes no axial bound")
}

// boundedSquareChain builds a 4-edge square chain at z = 10 with every
// vertex and every edge stamped a levelToken carrying id, and every vertex
// carrying bound — the shape a straight prism's own rim carries once gate
// 3's first (exact) arm has already been shown to refuse it. The token's
// own origin/normal match the fixture's own frame ((0,0,10), (0,0,1)),
// mirroring what internal/prismrecord/build.go's mint actually stamps. id == 0 reproduces
// what an untouched builder (a revolve seam, a cap-loop chamfer, a one-cap
// shell's open rim) leaves behind: a bounded chain with no certificate, and
// returns the zero levelToken to match.
func boundedSquareChain(id levelID, bound units.Value) ([]*Edge, levelToken) {
	tok := levelToken{}
	if id != 0 {
		tok = levelToken{ID: id, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	}
	v0 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 0, 10), Bound: bound, Level: tok}})
	v1 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 10), Bound: bound, Level: tok}})
	v2 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 1, 10), Bound: bound, Level: tok}})
	v3 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 1, 10), Bound: bound, Level: tok}})
	return []*Edge{
		bornEdge(&Edge{n: topo.Edge{Level: tok, Curve: surfacegeom.Line3{}, Start: vertexNode(v0), End: vertexNode(v1)}}),
		bornEdge(&Edge{n: topo.Edge{Level: tok, Curve: surfacegeom.Line3{}, Start: vertexNode(v1), End: vertexNode(v2)}}),
		bornEdge(&Edge{n: topo.Edge{Level: tok, Curve: surfacegeom.Line3{}, Start: vertexNode(v2), End: vertexNode(v3)}}),
		bornEdge(&Edge{n: topo.Edge{Level: tok, Curve: surfacegeom.Line3{}, Start: vertexNode(v3), End: vertexNode(v0)}}),
	}, tok
}

// TestProvePatchChainPlaneAdmitsABoundedCoLevelChain is
// docs/surface-design.md §15's T27: gate 3's second (LEVEL) arm admits a
// bounded chain whose every vertex and edge shares one non-zero levelID —
// the shape a straight prism build (internal/prismrecord/build.go) stamps onto a rim at one
// swept end — and returns the SAME token, never a value fitted to the
// chain's own held vertex coordinates.
func TestProvePatchChainPlaneAdmitsABoundedCoLevelChain(t *testing.T) {
	t.Parallel()
	edges, want := boundedSquareChain(1, units.Millimeters(0.002))

	lvl, bound, hasAxialBound, err := provePatchChainPlane(edges)
	require.NoError(t, err)
	require.True(t, hasAxialBound, "the level arm admitted this chain, so it publishes the chain's own axial bound")
	require.Equal(t, 0.002, bound)
	require.Equal(t, want, lvl, "the returned token is the chain's own recorded one, never a fitted value")
}

// TestProvePatchChainPlaneRefusesABoundedChainWithNoLevelToken is
// docs/surface-design.md §15's T27 mirror: the identical bounded shape with
// no level token at all — levelToken's own zero value, "no certificate" —
// stays refused on gate 3's existing row, exactly as it was before this
// certificate existed. This is what pins T12's own "a bounded chain that
// carries no shared level token" half: a real revolve seam or a one-cap
// shell's open rim never carries a level token, since neither builder mints
// one, and neither is reachable through decad's public seam as a PatchEdges
// fixture without changing an untouched file.
func TestProvePatchChainPlaneRefusesABoundedChainWithNoLevelToken(t *testing.T) {
	t.Parallel()
	edges, _ := boundedSquareChain(0, units.Millimeters(0.002))

	_, bound, hasAxialBound, err := provePatchChainPlane(edges)
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
// This shape is not reachable through PatchEdges's own public seam: a chain
// requires two consecutive edges to SHARE a vertex POINTER, which only one
// evaluator's own build or a zero-bound weld creates, and Table J refuses a
// zero-bound weld of a bounded pair (docs/surface-design.md §6.2) — so no
// selector over any two live bodies can ever hand PatchEdges one chain
// spanning two independently minted levels. Pinning it here, directly
// against provePatchChainPlane, is what the brief's own fallback asks for.
func TestProvePatchChainPlaneRefusesIdenticalBoundedVerticesUnderDifferentLevels(t *testing.T) {
	t.Parallel()
	levelA := levelToken{ID: 1, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	levelB := levelToken{ID: 2, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	bound := units.Millimeters(0.002)

	v1 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 10), Bound: bound}})
	v1Twin := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 10), Bound: bound, Level: levelB}})
	require.Equal(t, v1.n.Position, v1Twin.n.Position, "the two vertices' held coordinates are bit-identical")
	require.Equal(t, v1.n.Bound, v1Twin.n.Bound, "the two vertices' held bounds are bit-identical")

	v0 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 0, 10), Bound: bound, Level: levelA}})
	v2 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 1, 10), Bound: bound, Level: levelA}})
	v3 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 1, 10), Bound: bound, Level: levelA}})
	edges := []*Edge{
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v0), End: vertexNode(v1Twin)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v1Twin), End: vertexNode(v2)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v2), End: vertexNode(v3)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v3), End: vertexNode(v0)}}),
	}

	_, _, hasAxialBound, err := provePatchChainPlane(edges)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, hasAxialBound)
}

// TestBodyPatchRejectsAReceiverWithNoPayload is Table R row R19: a live body
// this evaluator did not build (payload nil) — unreachable through the
// public seam, since every builder here always sets one, on the same terms
// Transform/Copy already require.
func TestBodyPatchRejectsAReceiverWithNoPayload(t *testing.T) {
	t.Parallel()
	d := New()
	b := bornBody(&Body{n: topo.Body{Origin: provenance.Ref{Role: provenance.RoleBody}, Kind: topo.BodySheet, Doc: documentNode(d)}})
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(b))

	_, err := PatchEdges(t.Context(), b, Edges(Free()))
	require.ErrorIs(t, err, ErrUnsupported)
}

// patchTestReceiverWithOneChain builds a receiver *Body carrying one face
// bounded by edges, so receiver.Edges() returns exactly edges, each free.
// Rule P's three tests below each hand-build a receiver this shape rather
// than a real Extrude, since only one of the three conditions is ever the
// thing under test and the other two must hold cleanly for the isolation to
// mean anything.
func patchTestReceiverWithOneChain(payload topo.Payload, edges []*Edge) *Body {
	loop := bornLoop(&Loop{n: topo.Loop{Outer: true}})
	for _, e := range edges {
		loop.n.CoEdges = append(loop.n.CoEdges, topo.CoEdge{Edge: edgeNode(e), Forward: true})
	}
	face := bornFace(&Face{n: topo.Face{Loops: []*topo.Loop{loopNode(loop)}}})
	for _, e := range edges {
		e.n.Faces = []*topo.Face{faceNode(face)}
	}
	receiver := bornBody(&Body{n: topo.Body{Payload: payload}})
	receiver.n.Lumps = topobuild.SheetLumps(handleAlloc{}, faceNodes([]*Face{face}))
	face.n.Body = bodyNode(receiver)
	return receiver
}

// TestBodyPatchPayloadProvesSimpleRefusesNonAdmittingReceiver is
// docs/surface-design.md's T43, Rule P condition 1 isolated: the receiver's
// own payload is a prismPayload whose sectionDelta is nonzero, so
// payloadProvesSimple refuses it — exactly the shape a boolean-reduced
// prism reads (denotation.go's own doc comment: such a section is COMPUTED,
// never recorded). The chain itself is the receiver's own complete
// single-level end rim (conditions 2 and 3 both hold), isolating the
// refusal to condition 1 alone: deleting the receiver-admission check in
// bodyPatchPayloadProvesSimple is what this test is shown to catch.
func TestBodyPatchPayloadProvesSimpleRefusesNonAdmittingReceiver(t *testing.T) {
	t.Parallel()
	edges, _ := boundedSquareChain(1, units.Millimeters(0))
	receiver := patchTestReceiverWithOneChain(prismrecord.Payload{SurfaceResult: true, SectionDelta: 5}, edges)

	pp := bodyPatchPayload{Faces: faceNodes(receiver.Faces()), Chains: []bodyPatchChain{{Edges: edgeNodes(edges)}}}
	require.False(t, bodyPatchPayloadProvesSimple(context.Background(), pp))
}

// TestBodyPatchPayloadProvesSimpleRefusesIncompleteChain is
// docs/surface-design.md's T44, Rule P condition 2 isolated: the receiver's
// own end holds TWO disjoint free-edge loops sharing one level id — a
// four-edge outer rim and a one-edge inner (hole) rim, the annular-profile
// shape §6.4 names — and the new chain caps only the inner one. The
// receiver's own payload admits Rule S and the chain's own shared level id
// is valid (conditions 1 and 3 both hold), isolating the refusal to
// condition 2 alone: patching a proper subset of an end's own free edges
// proves nothing about the whole end's non-self-intersection.
func TestBodyPatchPayloadProvesSimpleRefusesIncompleteChain(t *testing.T) {
	t.Parallel()
	tok := levelToken{ID: 1, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	outer, _ := boundedSquareChain(1, units.Millimeters(0))
	cv := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(5, 5, 10), Level: tok}})
	inner := bornEdge(&Edge{n: topo.Edge{Level: tok, Curve: surfacegeom.Circle3{Center: r3.NewVec(5, 5, 10), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)}, Start: vertexNode(cv), End: vertexNode(cv)}})

	outerLoop := bornLoop(&Loop{n: topo.Loop{Outer: true}})
	for _, e := range outer {
		outerLoop.n.CoEdges = append(outerLoop.n.CoEdges, topo.CoEdge{Edge: edgeNode(e), Forward: true})
	}
	outerFace := bornFace(&Face{n: topo.Face{Loops: []*topo.Loop{loopNode(outerLoop)}}})
	for _, e := range outer {
		e.n.Faces = []*topo.Face{faceNode(outerFace)}
	}
	innerFace := bornFace(&Face{n: topo.Face{Loops: []*topo.Loop{loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(inner), Forward: true}}}}))}}})
	inner.n.Faces = []*topo.Face{faceNode(innerFace)}

	receiver := bornBody(&Body{n: topo.Body{Payload: prismrecord.Payload{SurfaceResult: true, SectionDelta: 0}}})
	receiver.n.Lumps = topobuild.SheetLumps(handleAlloc{}, faceNodes([]*Face{outerFace, innerFace}))
	outerFace.n.Body = bodyNode(receiver)
	innerFace.n.Body = bodyNode(receiver)

	pp := bodyPatchPayload{
		Faces:  faceNodes(receiver.Faces()),
		Chains: []bodyPatchChain{{Edges: edgeNodes([]*Edge{inner})}},
	}
	require.False(t, bodyPatchPayloadProvesSimple(context.Background(), pp))
}

// TestBodyPatchPayloadProvesSimpleRefusesVertexLevelMismatch is
// docs/surface-design.md's T45, Rule P condition 3 isolated: every one of
// the chain's four edges carries the SAME non-zero level id, but one
// vertex (v2) was stamped under a different one. The receiver's own end
// holds exactly these four edges under the edges' shared id (so
// completeness, condition 2, would hold), and the receiver's own payload
// admits Rule S (condition 1 holds), isolating the refusal to condition 3's
// own vertex check: an edge's id alone says its CURVE was recorded at one
// level, never that both its endpoints were too.
func TestBodyPatchPayloadProvesSimpleRefusesVertexLevelMismatch(t *testing.T) {
	t.Parallel()
	levelA := levelToken{ID: 1, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	levelB := levelToken{ID: 2, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	v0 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 0, 10), Level: levelA}})
	v1 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 10), Level: levelA}})
	v2 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 1, 10), Level: levelB}})
	v3 := bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(0, 1, 10), Level: levelA}})
	edges := []*Edge{
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v0), End: vertexNode(v1)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v1), End: vertexNode(v2)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v2), End: vertexNode(v3)}}),
		bornEdge(&Edge{n: topo.Edge{Level: levelA, Curve: surfacegeom.Line3{}, Start: vertexNode(v3), End: vertexNode(v0)}}),
	}
	receiver := patchTestReceiverWithOneChain(prismrecord.Payload{SurfaceResult: true, SectionDelta: 0}, edges)

	pp := bodyPatchPayload{Faces: faceNodes(receiver.Faces()), Chains: []bodyPatchChain{{Edges: edgeNodes(edges)}}}
	require.False(t, bodyPatchPayloadProvesSimple(context.Background(), pp))
}
