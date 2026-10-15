package patchchain

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the face build of PatchEdges (docs/surface-design.md §5.2)
// over hand-built node fixtures: the root package's public seam admits no way
// to author a chain whose adjacent faces disagree, a chain the level arm
// admitted over a perturbed vertex, or a bowtie chain.

// freeEdgeFace gives e one adjacent face that walks it forward, so e is a
// free edge of a one-edge face.
func freeEdgeFace(e *topo.Edge) {
	f := &topo.Face{Loops: []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{{Edge: e, Forward: true}}}}}
	e.Faces = []*topo.Face{f}
}

// squareChain builds a 4-edge square chain at z = 10 with every vertex and
// every edge stamped a level token carrying id, and every vertex carrying
// bound — the shape a straight prism's own rim carries once gate 3's first
// (exact) arm has already been shown to refuse it. The token's own
// origin/normal match the fixture's own frame ((0,0,10), (0,0,1)).
func squareChain(id denotation.LevelID, bound units.Value) ([]*topo.Edge, denotation.LevelToken) {
	tok := denotation.LevelToken{ID: id, Origin: r3.NewVec(0, 0, 10), Normal: r3.NewVec(0, 0, 1)}
	v := func(x, y float64) *topo.Vertex {
		return &topo.Vertex{Position: r3.NewVec(x, y, 10), Bound: bound, Level: tok}
	}
	v0, v1, v2, v3 := v(0, 0), v(1, 0), v(1, 1), v(0, 1)
	return []*topo.Edge{
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v0, End: v1},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v1, End: v2},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v2, End: v3},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v3, End: v0},
	}, tok
}

// TestBuildFaceCarriesTheLevelBoundOntoTheNewFacesAxialDelta is
// docs/surface-design.md §15's T29: buildFace over a chain gate 3's level arm
// admitted sets the new face's own AxialDelta/HasAxialDelta from the chain's
// proven bound — the same fields a prism cap already carries
// (internal/prismrecord/build.go) — rather than leaving them at their zero
// value.
func TestBuildFaceCarriesTheLevelBoundOntoTheNewFacesAxialDelta(t *testing.T) {
	t.Parallel()
	edges, tok := squareChain(1, units.Millimeters(0.002))
	edgeCopy := map[*topo.Edge]*topo.Edge{}
	for _, e := range edges {
		freeEdgeFace(e)
		edgeCopy[e] = e
	}

	chain := Chain{Edges: edges, AxialBound: 0.002, HasAxialBound: true, Level: tok}
	face, err := buildFace(context.Background(), topotest.Alloc{}, 0, chain, edgeCopy, r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, face.HasAxialDelta, "the new face carries the level arm's own axial bound")
	require.Equal(t, 0.002, face.AxialDelta)
	plane, ok := face.Surface.(surfacegeom.Plane)
	require.True(t, ok)
	// orientChain reverses each edge relative to this fixture's own dummy
	// adjacent face, so the walk's own sense (and so the sign LevelNormal
	// settles on) is a fixture artifact; either sign of the token's own EXACT
	// direction is what "never fitted" claims.
	require.True(t, plane.Frame.N() == tok.Normal || plane.Frame.N() == tok.Normal.Scale(-1),
		"the published normal must be exactly the token's own direction (either sign), got %v", plane.Frame.N())
}

// TestBuildFaceReadsTheLevelTokensNormalNotAFittedOneOnARotatedFrame is the
// coordinator's own concern, made concrete: for a chain admitted by the level
// arm, held vertex coordinates at a common recorded level are only
// APPROXIMATELY coplanar in float64 once the frame is not axis-aligned
// (frame.ToWorldUV(u, v) rounds differently per (u, v) — see denotation.go's
// own doc comment), so a normal FITTED to them (OrientedNormal's Newell sum)
// would carry that same rounding as an unbounded tilt no AxialDelta covers.
// This fixture deliberately perturbs one vertex's held coordinate by far more
// than any float64 rounding could — 1e-6 mm, which a genuine rotated-frame
// build would never actually produce — to prove the PUBLISHED plane does not
// track that perturbation at all: it comes from the token's own recorded
// normal, never from these (or any) held positions.
func TestBuildFaceReadsTheLevelTokensNormalNotAFittedOneOnARotatedFrame(t *testing.T) {
	t.Parallel()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	u, ok := r3.NewVec(1, 0, 0).Sub(axis.Scale(axis.Dot(r3.NewVec(1, 0, 0)))).Normalize()
	require.True(t, ok)
	v := axis.Cross(u)

	corner := func(s, t float64) r3.Vec {
		return r3.NewVec(7, -3, 11).Add(u.Scale(s)).Add(v.Scale(t)).Add(axis.Scale(5))
	}
	tok := denotation.LevelToken{ID: 1, Origin: corner(0, 0), Normal: axis}

	bound := units.Millimeters(0.002)
	v0 := &topo.Vertex{Position: corner(0, 0), Bound: bound, Level: tok}
	v1 := &topo.Vertex{Position: corner(10, 0), Bound: bound, Level: tok}
	v2 := &topo.Vertex{Position: corner(10, 10), Bound: bound, Level: tok}
	// v3's held position is perturbed off the true plane by 1e-6 mm along the
	// axis — far more than any float64 evaluation of corner() itself could
	// ever depart from it, standing in for "held data this arm never
	// verifies coplanar".
	v3 := &topo.Vertex{Position: corner(0, 10).Add(axis.Scale(1e-6)), Bound: bound, Level: tok}
	edges := []*topo.Edge{
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v0, End: v1},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v1, End: v2},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v2, End: v3},
		{Level: tok, Curve: surfacegeom.Line3{}, Start: v3, End: v0},
	}
	edgeCopy := map[*topo.Edge]*topo.Edge{}
	for _, e := range edges {
		freeEdgeFace(e)
		edgeCopy[e] = e
	}

	chain := Chain{Edges: edges, AxialBound: 0.002, HasAxialBound: true, Level: tok}
	face, err := buildFace(context.Background(), topotest.Alloc{}, 0, chain, edgeCopy, r3.Identity(), 0)
	require.NoError(t, err)
	plane, ok := face.Surface.(surfacegeom.Plane)
	require.True(t, ok)
	// See the sibling test's own comment: the walk's sense (and so which sign
	// LevelNormal settles on) is this fixture's own artifact. PlaneFrameFromNormal's
	// own Gram-Schmidt reprocesses whatever normal it is handed, introducing the
	// SAME ordinary float64 rounding prismrecord.Payload.CapFrame already carries
	// for any analytic Plane (a few ULPs, ~1e-16) — the claim under test is that
	// the published normal tracks the TOKEN's axis to THAT floor, not the far
	// larger 1e-6 perturbation on v3's own held position: a fit to the held
	// vertices would show error near 1e-6, not near 1e-16.
	dPos := plane.Frame.N().Sub(axis).Len()
	dNeg := plane.Frame.N().Sub(axis.Scale(-1)).Len()
	require.Less(t, min(dPos, dNeg), 1e-9,
		"the published normal must track the token's own axis, not the perturbed vertex; got %v vs axis %v", plane.Frame.N(), axis)
}

// TestOrientChainRejectsDisagreeingAdjacentFaces is Table R row R18: two free
// edges over the same vertex pair, each recorded in the SAME direction (v1 to
// v2) and each traversed forward by its own one adjacent face. Reversing each
// independently ("the opposite sense") then makes BOTH directed edges start at
// v2 — a duplicate orientChain's own connectivity walk catches directly, by
// pigeonhole leaving v1 the start of none.
func TestOrientChainRejectsDisagreeingAdjacentFaces(t *testing.T) {
	t.Parallel()
	v1 := &topo.Vertex{Position: r3.NewVec(0, 0, 0)}
	v2 := &topo.Vertex{Position: r3.NewVec(1, 0, 0)}
	a := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v1, End: v2}
	b := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v1, End: v2}
	freeEdgeFace(a)
	freeEdgeFace(b)

	_, err := orientChain(Chain{Edges: []*topo.Edge{a, b}})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
}

// TestBuildFaceRemapsACrossingChainToErrDegenerate is gate 4 exercised
// directly: a bowtie quadrilateral (A, B, C, D in order) whose two diagonals
// — the two non-adjacent segment pairs a plain 4-cycle boundary walk never
// puts in contact — cross at its center. orientChain's own "opposite sense"
// derivation succeeds (every builder here agrees, by construction: a single
// adjacent face per edge, each walked forward), so this reaches gate 4, whose
// ErrUnsupported crossing refusal (sectionaudit.Crossing) is remapped to
// ErrDegenerate at PatchEdges's own boundary (docs/surface-design.md Table R
// row R5).
func TestBuildFaceRemapsACrossingChainToErrDegenerate(t *testing.T) {
	t.Parallel()
	a := &topo.Vertex{Position: r3.NewVec(0, 0, 0)}
	b := &topo.Vertex{Position: r3.NewVec(1, 1, 0)}
	c := &topo.Vertex{Position: r3.NewVec(1, 0, 0)}
	d := &topo.Vertex{Position: r3.NewVec(0, 1, 0)}
	edges := []*topo.Edge{
		{Curve: surfacegeom.Line3{}, Start: a, End: b},
		{Curve: surfacegeom.Line3{}, Start: b, End: c},
		{Curve: surfacegeom.Line3{}, Start: c, End: d},
		{Curve: surfacegeom.Line3{}, Start: d, End: a},
	}
	edgeCopy := map[*topo.Edge]*topo.Edge{}
	for _, e := range edges {
		freeEdgeFace(e)
		edgeCopy[e] = e
	}

	_, err := buildFace(context.Background(), topotest.Alloc{}, 0, Chain{Edges: edges}, edgeCopy, r3.Identity(), 0)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
}
