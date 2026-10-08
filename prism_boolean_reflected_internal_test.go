package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/general-boolean-design.md §3 A4's white-box suite: the
// reflected operand admitted in place of prism-boolean G2, the re-wound
// record buildPrismScene builds a reflected operand B from, the tags it
// derives from that record, its walk charge, and the shared-axis arm that
// admits two operands reflected by one placement with no re-expression.
// apitest/prism_boolean_reflected_test.go covers the public booleans and
// Verify's interference reading on the same shapes.

// prismMirrorAcrossX is the reflection across the plane x = x0, the world
// YZ plane moved along x. Its linear part is diag(−1, 1, 1) exactly.
func prismMirrorAcrossX(t *testing.T, x0 float64) r3.Transform {
	t.Helper()
	mirror, err := r3.NewFrame(r3.NewVec(x0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, refl.IsReflection())
	return refl
}

// prismReflectDiscBody extrudes a circle of radius r centred at (cx, cy)
// prismFixtureHeight mm.
func prismReflectDiscBody(t *testing.T, doc *Document, cx, cy, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestPrismBooleanGateG2AdmitsAReflectedOperand pins A4's replacement of G2:
// a reflected operand clears the gate, its relative map reports a
// reflection, and two operands reflected by one placement read as the
// identity. Shown to fail with admitPrismPairBudget's former
// `pa.reflected() || pb.reflected()` refusal restored.
func TestPrismBooleanGateG2AdmitsAReflectedOperand(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pp := prismPayload{
		profile: ProfileRecord{Outer: synthLineLoop()},
		frame:   frame, z0: 0, z1: 10, xform: r3.Identity(),
	}
	reflected := pp
	reflected.xform = prismMirrorAcrossX(t, 0)

	pa, pb, ok := admitPrismPair(&Body{payload: pp}, &Body{payload: reflected})
	require.True(t, ok, "A4: a reflected operand clears G1-G4")
	re, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.False(t, re.Identity)
	require.True(t, re.Reflected, "one reflected placement makes the relative map improper")

	pa, pb, ok = admitPrismPair(&Body{payload: reflected}, &Body{payload: reflected})
	require.True(t, ok)
	re, err = prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, re.Identity, "one shared reflected placement is the shared-axis arm's identity")
	require.False(t, re.Reflected)
}

// TestPrismReexpressionRewound pins every per-kind rule of
// prismcells.Reexpression.Rewind on one record holding a narrowed LineSeg, a
// whole LineSeg walked forward and one walked backward, a whole ArcSeg and a
// clockwise CircleSeg hole, reflected across x = 0 so every mapped
// coordinate is exact. Each re-wound loop must close walk to walk, the outer
// must wind counter-clockwise and the hole clockwise again, and the narrowed
// line's walk charge must be returned.
//
// Shown to fail, one change at a time: keeping each loop's segment order,
// leaving a line's Start/End unswapped, and building the ArcSeg as
// {m(C), m(S), m(E)} each broke the junctions; the CircleSeg with CCW
// flipped failed walkOf ("CCW flag contradicts its range order"); and
// dropping the narrowed line's charge returned zero.
func TestPrismReexpressionRewound(t *testing.T) {
	t.Parallel()
	narrowed := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 20, V: 0}, TStart: 0, TEnd: 0.5}
	profile := ProfileRecord{
		Outer: LoopRecord{Segments: []CurveSegment{
			narrowed, // walks (0,0) → (10,0)
			LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 5}, TStart: 0, TEnd: 1},
			ArcSeg{Center: Point2{U: 5, V: 5}, Start: Point2{U: 10, V: 5}, End: Point2{U: 0, V: 5}, TStart: 0, TEnd: 1},
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 5}, TStart: 1, TEnd: 0}, // walks (0,5) → (0,0)
		}},
		Holes: []LoopRecord{{Segments: []CurveSegment{
			CircleSeg{Center: Point2{U: 5, V: 4}, Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
		}}},
	}
	frame := canonicalPrismFrame(t)
	pa := prismPayload{profile: ProfileRecord{Outer: synthRectLoop(-20, -20, 20, 20)}, frame: frame, z0: 0, z1: 10, xform: r3.Identity()}
	pb := prismPayload{profile: profile, frame: frame, z0: 0, z1: 10, xform: prismMirrorAcrossX(t, 0)}
	re, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, re.Reflected)

	budget := proofbound.NewWorkBudget(t.Context())
	got, charge, err := re.Rewind(budget, prismcells.SceneProfile{Outer: profile.Outer, Holes: profile.Holes})
	require.NoError(t, err)

	// Every segment walks, and a loop of several segments closes walk to walk
	// exactly. A lone circle closes by its own kind; its walk ends are cos/sin
	// readings and are not compared.
	for _, loop := range append([]LoopRecord{got.Outer}, got.Holes...) {
		walks := make([]Point2, 0, 2*len(loop.Segments))
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, nil)
			require.NoError(t, err)
			walks = append(walks, Point2{U: w.StartU, V: w.StartV}, Point2{U: w.EndU, V: w.EndV})
		}
		if len(loop.Segments) == 1 {
			continue
		}
		for i := 1; i < len(walks); i += 2 {
			next := walks[(i+1)%len(walks)]
			require.Equal(t, next, walks[i], "segment %d's walk end must be the next segment's walk start", i/2)
		}
	}

	outerArea, err := loopSignedAreaBudget(budget, got.Outer)
	require.NoError(t, err)
	require.InDelta(t, 50+math.Pi*12.5, outerArea, 1e-9, "the re-wound outer winds counter-clockwise")
	holeArea, err := loopSignedAreaBudget(budget, got.Holes[0])
	require.NoError(t, err)
	require.InDelta(t, -math.Pi, holeArea, 1e-9, "the re-wound hole winds clockwise")

	w, err := boundarywalk.WalkOf(narrowed, nil)
	require.NoError(t, err)
	wantCharge, err := walkChargeOf(narrowed, w)
	require.NoError(t, err)
	require.Positive(t, wantCharge)
	require.Equal(t, wantCharge, charge, "the narrowed line's own walk charge")

	m := func(u, v float64) Point2 { return Point2{U: -u, V: v} }
	want := prismcells.SceneProfile{
		Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: m(0, 5), End: m(0, 0), TStart: 1, TEnd: 0},
			ArcSeg{Center: m(5, 5), Start: m(0, 5), End: m(10, 5), TStart: 0, TEnd: 1},
			LineSeg{Start: m(10, 5), End: m(10, 0), TStart: 0, TEnd: 1},
			LineSeg{Start: m(10, 0), End: m(0, 0), TStart: 0, TEnd: 1},
		}},
		Holes: []LoopRecord{{Segments: []CurveSegment{
			CircleSeg{Center: m(5, 4), Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
		}}},
	}
	require.Equal(t, want, got)
}

// TestPrismReflectedSceneClassifiesTheRewoundWinding builds the private scene
// for a box A and a box B whose reflected image [5, 15]² crosses A, then
// reads prismcells.Classify's cells straight off it: the cells A keeps
// without B, both keep, and B keeps without A have the exact areas 75, 25
// and 75. Shown to fail with buildPrismScene adding B through
// reexpress.point without rewound: B's walls then wind clockwise, B's
// membership flips, and the A-without-B selection measures less than 75.
func TestPrismReflectedSceneClassifiesTheRewoundWinding(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	pa := prismPayload{profile: ProfileRecord{Outer: synthRectLoop(0, 0, 10, 10)}, frame: frame, z0: 0, z1: 10, xform: r3.Identity()}
	pb := prismPayload{profile: ProfileRecord{Outer: synthRectLoop(-15, 5, -5, 15)}, frame: frame, z0: 0, z1: 10, xform: prismMirrorAcrossX(t, 0)}
	re, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, re.Reflected)

	budget := proofbound.NewWorkBudget(t.Context())
	s, tags, _, err := buildPrismScene(budget, pa, pb, re)
	require.NoError(t, err)
	profiles := s.Profiles()
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	require.NoError(t, err)
	require.True(t, resolved)

	for _, tc := range []struct {
		name string
		keep func(a, b bool) bool
		area int64
	}{
		{"A without B", func(a, b bool) bool { return a && !b }, 75},
		{"A and B", func(a, b bool) bool { return a && b }, 25},
		{"B without A", func(a, b bool) bool { return !a && b }, 75},
	} {
		selected, err := prismcells.Select(budget, profiles, matterA, matterB, tc.keep)
		require.NoError(t, err)
		merged, _, ok, err := mergePrismCells(budget, selected, tc.name)
		require.NoError(t, err)
		require.True(t, ok, tc.name)
		require.Zero(t, prismExactLineOnlyArea(t, merged).Cmp(big.NewRat(tc.area, 1)), "%s: area", tc.name)
	}
}

// TestPrismReflectedOperandChargesItsWalk pins the narrowed-line arm of A4's
// re-wound record on a live operand: the split-left-cell fixture, whose
// bottom and top walls are recorded over narrowed ranges, reflected across
// x = 0 and set strictly inside a box. buildPrismScene's walk charge for B
// must equal the largest walkChargeOf over B's own recorded segments, the
// same charge the unreflected path takes, and the union must publish a
// sectionDelta covering it. Shown to fail with buildPrismScene's fold of
// rewound's charge into sceneDelta.B deleted: sceneDelta.B read 0.
func TestPrismReflectedOperandChargesItsWalk(t *testing.T) {
	t.Parallel()
	doc := New()
	cell := prismSplitLeftCellBody(t, doc)
	image, err := cell.PlacedCopy(t.Context(), prismMirrorAcrossX(t, 0))
	require.NoError(t, err)
	box := prismRectBody(t, doc, -10, -2, 10, 12)

	pa, pb, ok := admitPrismPair(box, image)
	require.True(t, ok)
	want := 0.0
	for _, seg := range pb.profile.Outer.Segments {
		w, err := boundarywalk.WalkOf(seg, nil)
		require.NoError(t, err)
		c, err := walkChargeOf(seg, w)
		require.NoError(t, err)
		want = math.Max(want, c)
	}
	require.Positive(t, want, "the fixture must carry a narrowed segment, or it proves nothing")

	re, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, re.Reflected)
	_, _, sceneDelta, err := buildPrismScene(proofbound.NewWorkBudget(t.Context()), pa, pb, re)
	require.NoError(t, err)
	require.Zero(t, sceneDelta.A, "the box is drawn whole")
	require.Equal(t, want, sceneDelta.B)

	result, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, box, image)
	require.NoError(t, err)
	require.True(t, ok, "B nests inside A, so the union splits nothing")
	require.GreaterOrEqual(t, result.sectionDelta, want)
}

// TestPrismBooleanReflectedSharedAxisIsIdentity is T1 of
// docs/mirror-pattern-design.md §2 through tryPrismBoolean: an L and a
// cylinder inside its leg, both placed by one reflection, cut through G3's
// shared-axis arm with no re-expression, so the result's sectionDelta is
// exactly 0. The same pair with only the L reflected, the cylinder drawn at
// the image position, cuts through the reflected re-expression and carries
// a positive sectionDelta. Shown to fail with G2's refusal restored (neither
// pair admits).
func TestPrismBooleanReflectedSharedAxisIsIdentity(t *testing.T) {
	t.Parallel()
	lPts := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}
	mirror := prismMirrorAcrossX(t, 30)

	doc := New()
	l := internalPolyPrismBody(t, doc, lPts, prismFixtureHeight)
	disc := prismReflectDiscBody(t, doc, 2.5, 12, 1.5)
	ml, err := l.Placed(t.Context(), mirror)
	require.NoError(t, err)
	md, err := disc.Placed(t.Context(), mirror)
	require.NoError(t, err)
	pa, pb, ok := admitPrismPair(ml, md)
	require.True(t, ok)
	re, err := prismcells.NewReexpression(prismPlacementOf(pa), prismPlacementOf(pb))
	require.NoError(t, err)
	require.True(t, re.Identity)

	both, ok, err := tryPrismBoolean(t.Context(), meshbool.OpCut, ml, md)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, both.reflected())
	require.Zero(t, both.sectionDelta, "the shared-axis arm computes no coordinate")
	require.Len(t, both.profile.Holes, 1)

	doc2 := New()
	l2 := internalPolyPrismBody(t, doc2, lPts, prismFixtureHeight)
	ml2, err := l2.Placed(t.Context(), mirror)
	require.NoError(t, err)
	disc2 := prismReflectDiscBody(t, doc2, 57.5, 12, 1.5)
	one, ok, err := tryPrismBoolean(t.Context(), meshbool.OpCut, ml2, disc2)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, one.sectionDelta, "the reflected re-expression rounds the tool's coordinates")
	require.Len(t, one.profile.Holes, 1)
}

// prismToothBody extrudes a gear tooth over [z0, z0+h]: an arc of radius 20
// about the origin from −0.1 to 0.1 rad, closed by two radial flanks and an
// outer chord at radius 25. Its root arc lies on a radius-20 hub's circle.
func prismToothBody(t *testing.T, doc *Document, z0, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	p1 := s.CreatePoint(20*math.Cos(-0.1), 20*math.Sin(-0.1))
	p2 := s.CreatePoint(20*math.Cos(0.1), 20*math.Sin(0.1))
	p3 := s.CreatePoint(25*math.Cos(0.1), 25*math.Sin(0.1))
	p4 := s.CreatePoint(25*math.Cos(-0.1), 25*math.Sin(-0.1))
	s.Fix(p1)
	s.CreateArc(center, p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// prismOvershootQuadBody draws a quadrilateral as four lines overshooting
// each corner by over times the side, so every recorded boundary segment is
// a Partial fragment (apitest's overshootQuadBody, fu141's reproduction), and
// extrudes the largest cell prismFixtureHeight mm.
func prismOvershootQuadBody(t *testing.T, doc *Document, corners [][2]float64, over float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	n := len(corners)
	for i := range n {
		a, b := corners[i], corners[(i+1)%n]
		dx, dy := b[0]-a[0], b[1]-a[1]
		p := s.CreatePoint(a[0]-over*dx, a[1]-over*dy)
		q := s.CreatePoint(b[0]+over*dx, b[1]+over*dy)
		s.Fix(p)
		s.Fix(q)
		s.CreateLine(p, q)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	best := 0
	for i, p := range profs {
		if p.Area > profs[best].Area {
			best = i
		}
	}
	body, err := doc.Extrude(s, profs[best], Distance{D: units.Millimeters(prismFixtureHeight), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestPrismAmplifiedCutsFallBack pins A6's routing for a pair whose cuts
// carry an amplified input displacement: it never refuses on the analytic
// path.
//
//   - A tooth whose root arc lies on the hub circle, rotated 60°: the root
//     arc and the hub circle are one carrier (general-boolean §3 A3), the
//     rotation displaces the tooth, and Union keeps the shared arc inside
//     its result, where no displacement charge covers the two true walls
//     parting (prismSceneDelta.SharedSpansBounded). Union and a Cut whose
//     tool is taller than the hub both return ok=false with no error, so
//     the caller takes the mesh path.
//   - fu141's overshooting quadrilateral, whose fragments carry a walk
//     charge, restates one corner as two whole segments that do not
//     bit-match. Unioned with a box strictly inside it, nothing is cut, and
//     the merge refuses with RB9. Unioned with a box crossing its right
//     wall, the cuts carry the walk charge, the crossings are charged, and
//     the same RB9 sends the pair to the mesh path instead.
//
// Shown to fail with prismcells.AmplifiedFallback returning every error (the
// crossing quadrilateral returned RB9), and with sharedSpansBounded always
// reporting true (the tooth's Union built analytically).
func TestPrismAmplifiedCutsFallBack(t *testing.T) {
	t.Parallel()
	turn, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Radians(math.Pi/3))
	require.NoError(t, err)
	for _, tc := range []struct {
		name  string
		op    meshbool.OperationKind
		z0, h float64
	}{
		{"tangent root, union", meshbool.OpUnion, 0, prismFixtureHeight},
		{"tangent root, cut by a taller tool", meshbool.OpCut, -5, 20},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			hub := prismReflectDiscBody(t, doc, 0, 0, 20)
			tooth, err := prismToothBody(t, doc, tc.z0, tc.h).Placed(t.Context(), turn)
			require.NoError(t, err)
			_, ok, err := tryPrismBoolean(t.Context(), tc.op, hub, tooth)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}

	corners := [][2]float64{{-9.317, -5.731}, {10.29, -6.113}, {8.877, 7.219}, {-7.331, 6.407}}
	t.Run("a merge that cuts nothing refuses RB9", func(t *testing.T) {
		doc := New()
		quad := prismOvershootQuadBody(t, doc, corners, 0.13)
		_, _, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, quad, prismRectBody(t, doc, -1, -1, 1, 1))
		require.ErrorIs(t, err, ErrUnrecordableProfile)
	})
	t.Run("the same RB9 on an amplified cut falls back", func(t *testing.T) {
		doc := New()
		quad := prismOvershootQuadBody(t, doc, corners, 0.13)
		_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, quad, prismRectBody(t, doc, 8, -1, 12, 1))
		require.NoError(t, err)
		require.False(t, ok)
	})
}
