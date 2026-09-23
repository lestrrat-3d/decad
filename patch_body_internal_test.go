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
	_, _, hasAxialBound, err := provePatchChainPlane(edges)
	require.NoError(t, err)
	require.False(t, hasAxialBound, "the exact arm admitted this chain, so it publishes no axial bound")
}

// boundedSquareChain builds a 4-edge square chain at z = 10 with every
// vertex and every edge stamped a levelToken carrying id, and every vertex
// carrying bound — the shape a straight prism's own rim carries once gate
// 3's first (exact) arm has already been shown to refuse it. The token's
// own origin/normal match the fixture's own frame ((0,0,10), (0,0,1)),
// mirroring what prism_build.go's mint actually stamps. id == 0 reproduces
// what an untouched builder (a revolve seam, a cap-loop chamfer, a one-cap
// shell's open rim) leaves behind: a bounded chain with no certificate, and
// returns the zero levelToken to match.
func boundedSquareChain(id levelID, bound units.Value) ([]*Edge, levelToken) {
	tok := levelToken{}
	if id != 0 {
		tok = levelToken{id: id, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
	}
	v0 := &Vertex{position: r3.NewVec(0, 0, 10), bound: bound, level: tok}
	v1 := &Vertex{position: r3.NewVec(1, 0, 10), bound: bound, level: tok}
	v2 := &Vertex{position: r3.NewVec(1, 1, 10), bound: bound, level: tok}
	v3 := &Vertex{position: r3.NewVec(0, 1, 10), bound: bound, level: tok}
	return []*Edge{
		{curve: Line3{}, start: v0, end: v1, level: tok},
		{curve: Line3{}, start: v1, end: v2, level: tok},
		{curve: Line3{}, start: v2, end: v3, level: tok},
		{curve: Line3{}, start: v3, end: v0, level: tok},
	}, tok
}

// TestProvePatchChainPlaneAdmitsABoundedCoLevelChain is
// docs/surface-design.md §15's T27: gate 3's second (LEVEL) arm admits a
// bounded chain whose every vertex and edge shares one non-zero levelID —
// the shape a straight prism build (prism_build.go) stamps onto a rim at one
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
// one, and neither is reachable through decad's public seam as a Body.Patch
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
// This shape is not reachable through Body.Patch's own public seam: a chain
// requires two consecutive edges to SHARE a vertex POINTER, which only one
// evaluator's own build or a zero-bound weld creates, and Table J refuses a
// zero-bound weld of a bounded pair (docs/surface-design.md §6.2) — so no
// selector over any two live bodies can ever hand Body.Patch one chain
// spanning two independently minted levels. Pinning it here, directly
// against provePatchChainPlane, is what the brief's own fallback asks for.
func TestProvePatchChainPlaneRefusesIdenticalBoundedVerticesUnderDifferentLevels(t *testing.T) {
	t.Parallel()
	levelA := levelToken{id: 1, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
	levelB := levelToken{id: 2, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
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

	_, _, hasAxialBound, err := provePatchChainPlane(edges)
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
	edges, tok := boundedSquareChain(1, units.Millimeters(0.002))
	edgeCopy := map[*Edge]*Edge{}
	for _, e := range edges {
		f := &Face{loops: []*Loop{{coedges: []coedge{{edge: e, forward: true}}, outer: true}}}
		e.faces = []*Face{f}
		edgeCopy[e] = e
	}

	chain := bodyPatchChain{edges: edges, axialBound: 0.002, hasAxialBound: true, level: tok}
	face, err := buildPatchFace(context.Background(), 0, chain, edgeCopy, r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, face.hasAxialDelta, "the new face carries the level arm's own axial bound")
	require.Equal(t, 0.002, face.axialDelta)
	plane, ok := face.surface.(Plane)
	require.True(t, ok)
	// orientPatchChain reverses each edge relative to this fixture's own
	// dummy adjacent face, so the walk's own sense (and so the sign
	// patchChainLevelNormal settles on) is a fixture artifact; either sign
	// of the token's own EXACT direction is what "never fitted" claims.
	require.True(t, plane.Frame.N() == tok.normal || plane.Frame.N() == tok.normal.Scale(-1),
		"the published normal must be exactly the token's own direction (either sign), got %v", plane.Frame.N())
}

// TestBuildPatchFaceReadsTheLevelTokensNormalNotAFittedOneOnARotatedFrame is
// the coordinator's own concern, made concrete: for a chain admitted by the
// level arm, held vertex coordinates at a common recorded level are only
// APPROXIMATELY coplanar in float64 once the frame is not axis-aligned
// (frame.ToWorldUV(u, v) rounds differently per (u, v) — see denotation.go's
// own doc comment), so a normal FITTED to them (patchChainOrientedNormal's
// Newell sum) would carry that same rounding as an unbounded tilt no
// axialDelta covers. This fixture deliberately perturbs one vertex's held
// coordinate by far more than any float64 rounding could — 1e-6 mm, which a
// genuine rotated-frame build would never actually produce — to prove the
// PUBLISHED plane does not track that perturbation at all: it comes from the
// token's own recorded normal, never from these (or any) held positions.
func TestBuildPatchFaceReadsTheLevelTokensNormalNotAFittedOneOnARotatedFrame(t *testing.T) {
	t.Parallel()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	u, ok := r3.NewVec(1, 0, 0).Sub(axis.Scale(axis.Dot(r3.NewVec(1, 0, 0)))).Normalize()
	require.True(t, ok)
	v := axis.Cross(u)

	corner := func(s, t float64) r3.Vec {
		return r3.NewVec(7, -3, 11).Add(u.Scale(s)).Add(v.Scale(t)).Add(axis.Scale(5))
	}
	tok := levelToken{id: 1, origin: corner(0, 0), normal: axis}

	bound := units.Millimeters(0.002)
	v0 := &Vertex{position: corner(0, 0), bound: bound, level: tok}
	v1 := &Vertex{position: corner(10, 0), bound: bound, level: tok}
	v2 := &Vertex{position: corner(10, 10), bound: bound, level: tok}
	// v3's held position is perturbed off the true plane by 1e-6 mm along the
	// axis — far more than any float64 evaluation of corner() itself could
	// ever depart from it, standing in for "held data this arm never
	// verifies coplanar".
	v3 := &Vertex{position: corner(0, 10).Add(axis.Scale(1e-6)), bound: bound, level: tok}
	edges := []*Edge{
		{curve: Line3{}, start: v0, end: v1, level: tok},
		{curve: Line3{}, start: v1, end: v2, level: tok},
		{curve: Line3{}, start: v2, end: v3, level: tok},
		{curve: Line3{}, start: v3, end: v0, level: tok},
	}
	edgeCopy := map[*Edge]*Edge{}
	for _, e := range edges {
		f := &Face{loops: []*Loop{{coedges: []coedge{{edge: e, forward: true}}, outer: true}}}
		e.faces = []*Face{f}
		edgeCopy[e] = e
	}

	chain := bodyPatchChain{edges: edges, axialBound: 0.002, hasAxialBound: true, level: tok}
	face, err := buildPatchFace(context.Background(), 0, chain, edgeCopy, r3.Identity(), 0)
	require.NoError(t, err)
	plane, ok := face.surface.(Plane)
	require.True(t, ok)
	// See the sibling test's own comment: the walk's sense (and so which
	// sign patchChainLevelNormal settles on) is this fixture's own artifact.
	// planeFrameFromNormal's own Gram-Schmidt reprocesses whatever normal it
	// is handed, introducing the SAME ordinary float64 rounding
	// prism_build.go's own capFrame already carries for any analytic Plane
	// (a few ULPs, ~1e-16) — the claim under test is that the published
	// normal tracks the TOKEN's axis to THAT floor, not the far larger 1e-6
	// perturbation on v3's own held position: a fit to the held vertices
	// would show error near 1e-6, not near 1e-16.
	dPos := plane.Frame.N().Sub(axis).Len()
	dNeg := plane.Frame.N().Sub(axis.Scale(-1)).Len()
	require.Less(t, min(dPos, dNeg), 1e-9,
		"the published normal must track the token's own axis, not the perturbed vertex; got %v vs axis %v", plane.Frame.N(), axis)
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

	_, err := buildPatchFace(context.Background(), 0, bodyPatchChain{edges: edges}, edgeCopy, r3.Identity(), 0)
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

// patchTestReceiverWithOneChain builds a receiver *Body carrying one face
// bounded by edges, so receiver.Edges() returns exactly edges, each free.
// Rule P's three tests below each hand-build a receiver this shape rather
// than a real Extrude, since only one of the three conditions is ever the
// thing under test and the other two must hold cleanly for the isolation to
// mean anything.
func patchTestReceiverWithOneChain(payload featurePayload, edges []*Edge) *Body {
	loop := &Loop{outer: true}
	for _, e := range edges {
		loop.coedges = append(loop.coedges, coedge{edge: e, forward: true})
	}
	face := &Face{loops: []*Loop{loop}}
	for _, e := range edges {
		e.faces = []*Face{face}
	}
	receiver := &Body{payload: payload}
	receiver.lumps = sheetLumps([]*Face{face})
	face.body = receiver
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
	receiver := patchTestReceiverWithOneChain(prismPayload{surfaceResult: true, sectionDelta: 5}, edges)

	pp := bodyPatchPayload{faces: receiver.Faces(), chains: []bodyPatchChain{{edges: edges}}}
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
	tok := levelToken{id: 1, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
	outer, _ := boundedSquareChain(1, units.Millimeters(0))
	cv := &Vertex{position: r3.NewVec(5, 5, 10), level: tok}
	inner := &Edge{
		curve: Circle3{Center: r3.NewVec(5, 5, 10), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)},
		start: cv, end: cv, level: tok,
	}

	outerLoop := &Loop{outer: true}
	for _, e := range outer {
		outerLoop.coedges = append(outerLoop.coedges, coedge{edge: e, forward: true})
	}
	outerFace := &Face{loops: []*Loop{outerLoop}}
	for _, e := range outer {
		e.faces = []*Face{outerFace}
	}
	innerFace := &Face{loops: []*Loop{{outer: true, coedges: []coedge{{edge: inner, forward: true}}}}}
	inner.faces = []*Face{innerFace}

	receiver := &Body{payload: prismPayload{surfaceResult: true, sectionDelta: 0}}
	receiver.lumps = sheetLumps([]*Face{outerFace, innerFace})
	outerFace.body = receiver
	innerFace.body = receiver

	pp := bodyPatchPayload{
		faces:  receiver.Faces(),
		chains: []bodyPatchChain{{edges: []*Edge{inner}}},
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
	levelA := levelToken{id: 1, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
	levelB := levelToken{id: 2, origin: r3.NewVec(0, 0, 10), normal: r3.NewVec(0, 0, 1)}
	v0 := &Vertex{position: r3.NewVec(0, 0, 10), level: levelA}
	v1 := &Vertex{position: r3.NewVec(1, 0, 10), level: levelA}
	v2 := &Vertex{position: r3.NewVec(1, 1, 10), level: levelB}
	v3 := &Vertex{position: r3.NewVec(0, 1, 10), level: levelA}
	edges := []*Edge{
		{curve: Line3{}, start: v0, end: v1, level: levelA},
		{curve: Line3{}, start: v1, end: v2, level: levelA},
		{curve: Line3{}, start: v2, end: v3, level: levelA},
		{curve: Line3{}, start: v3, end: v0, level: levelA},
	}
	receiver := patchTestReceiverWithOneChain(prismPayload{surfaceResult: true, sectionDelta: 0}, edges)

	pp := bodyPatchPayload{faces: receiver.Faces(), chains: []bodyPatchChain{{edges: edges}}}
	require.False(t, bodyPatchPayloadProvesSimple(context.Background(), pp))
}
