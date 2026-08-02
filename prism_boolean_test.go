package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md PR1's black-box test suite
// (§14, §15): the analytic reduction for Union over co-directional coplanar
// straight prisms. boxBody/ballBody (clearance_test.go) are the shared
// helpers. Per-gate (G1-G6) isolation lives in
// prism_boolean_internal_test.go, which reads the gate decisions directly —
// most of these gate misses share a sketch/cap plane with their pair (the
// ordinary shape this design targets), so a black-box fallback reaches the
// SAME pre-existing coplanar-contact refusal the mesh path has always had
// (evaluator-design §9's own limitation, and this design's whole reason to
// exist), not a clean mesh-path success; that refusal is itself the
// documented pre-PR1 behavior, not a new one.

// hubBody extrudes a radius-r circle centered at the origin to an h mm
// prism — the gear hub these tests union a tooth onto.
func hubBody(t *testing.T, doc *decad.Document, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// toothBody extrudes a single gear tooth: a wedge whose inner boundary is an
// arc of radius r about the origin (the hub's own carrier) from th1 to th2,
// and whose outer boundary sits at radius r2 — the coincident-carrier union
// case (§15).
func toothBody(t *testing.T, doc *decad.Document, r, r2, th1, th2, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	p1 := s.CreatePoint(r*math.Cos(th1), r*math.Sin(th1))
	p2 := s.CreatePoint(r*math.Cos(th2), r*math.Sin(th2))
	p3 := s.CreatePoint(r2*math.Cos(th2), r2*math.Sin(th2))
	p4 := s.CreatePoint(r2*math.Cos(th1), r2*math.Sin(th1))
	s.Fix(p1)
	s.CreateArc(center, p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// verticalConvexEdge picks one convex lateral edge, matching fillet_test.go's
// own selector idiom.
func verticalConvexEdge() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()).AtLeast(1)
}

// anyFaceIsFaceted reports whether any face of b is a Faceted surface — the
// mesh boolean's own output kind, never produced by the analytic reduction.
func anyFaceIsFaceted(b *decad.Body) bool {
	for _, f := range b.Faces() {
		if f.Surface().Kind() == decad.KindFaceted {
			return true
		}
	}
	return false
}

// --- Core PR1 capability tests (§14) ---

// TestPrismUnionTwoBoxesSharingCapPlaneBuildsAnalyticExact is the "control"
// case from the consumer's report (§14): two overlapping straight prisms
// sharing a cap plane combine through the analytic reduction, reporting a
// line-only Exact volume with a zero bound (§7) rather than the mesh
// boolean's Approximate faceted result.
func TestPrismUnionTwoBoxesSharingCapPlaneBuildsAnalyticExact(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	got, err := decad.Union(a, b)
	require.NoError(t, err)

	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, units.CubicMillimeters(0), vol.Bound)
	require.InDelta(t, 1750.0, volumeMM(t, vol), 1e-9) // 100 + 100 - 25 (the 5x5 overlap), times h=10

	// The result is a first-class prismPayload: every face is analytic
	// (Plane/Cylinder), never Faceted — the mesh boolean never ran.
	for _, f := range got.Faces() {
		require.NotEqual(t, decad.KindFaceted, f.Surface().Kind())
	}
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	start, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(got))).Exactly(1).SelectFaces(got)
	require.NoError(t, err)
	require.Contains(t, start[0].Origins(), decad.CapStart(got))
	require.NotContains(t, start[0].Origins(), decad.CapStart(a))
	require.NotContains(t, start[0].Origins(), decad.CapStart(b))
}

// TestPrismUnionGearToothOnHubSharedCarrier is §15's coincident-carrier
// case: a tooth whose inner boundary is an arc on the hub's own circle
// unions with the hub, keeping the hub's own circle for the surviving
// majority arc (not a fabricated third geometry — the surviving Cylinder
// face's own recorded area matches the closed-form majority-arc reading
// exactly) and reporting an Approximate volume within the closed-form
// Pappus/Green's-theorem sum of the two prisms' own volumes.
func TestPrismUnionGearToothOnHubSharedCarrier(t *testing.T) {
	const r, r2, th1, th2, h = 20.0, 25.0, 0.0, 0.2, 10.0
	doc := decad.New()
	hub := hubBody(t, doc, r, h)
	tooth := toothBody(t, doc, r, r2, th1, th2, h)

	hubVol, err := hub.Volume()
	require.NoError(t, err)
	toothVol, err := tooth.Volume()
	require.NoError(t, err)
	closedForm := volumeMM(t, hubVol) + volumeMM(t, toothVol)

	got, err := decad.Union(hub, tooth)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	bound := boundMM3(t, vol)
	require.Greater(t, bound, 0.0)
	require.InDelta(t, closedForm, volumeMM(t, vol), bound)

	// Region set: exactly one surviving Cylinder wall (the hub's own circle
	// minus the tooth's span), three surviving planar tooth walls, and two
	// caps — the shared span between the hub disk and the tooth wedge
	// dropped as an interior wall, never appearing in the result.
	var cylinders, planes int
	for _, f := range got.Faces() {
		switch f.Surface().Kind() {
		case decad.KindCylinder:
			cylinders++
			area, err := f.Area()
			require.NoError(t, err)
			// The majority arc spans 2*pi - (th2-th1) at radius r.
			wantArea := r * (2*math.Pi - (th2 - th1)) * h
			require.InDelta(t, wantArea, areaMM2(t, area), 1e-6)
		case decad.KindPlane:
			planes++
		}
	}
	require.Equal(t, 1, cylinders)
	require.Equal(t, 5, planes) // 3 tooth walls + capStart + capEnd
}

// TestPrismUnionG1FallsBackWithUnchangedBehavior demonstrates §3.4's "no new
// behavior" promise concretely: a G1 miss (a revolve body carries no
// prismPayload) reaches the exact same pre-existing mesh-path staging
// refusal it would have before this design existed (evaluator-design §9:
// revolve has no tessellator yet) — no new error, no new text.
func TestPrismUnionG1FallsBackWithUnchangedBehavior(t *testing.T) {
	doc := decad.New()
	ball := ballBody(t, doc, 5)
	box := boxBody(t, doc, -20, -20, 20, 20, 20)
	_, err := decad.Union(ball, box)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestPrismUnionRotatedToothG3 is §3.3/§15's rotated-tooth case, at the
// r3-probe-measured breadth: r3.RotationAround about (0,0,1) is inexact at
// MOST step counts, not merely 17 — the probe at
// r3/.tmp/decad-axis-exact-rotation-ask swept 3..60 and found 48 of 58
// counts inexact somewhere in the sweep, including every count from 19 up.
// n=17 (this test's case) fails at k=10 specifically; it is representative
// of the general shape, not a special or unusually unlucky count.
//
// Because the tooth's inner boundary sits on the hub's own cylinder
// (radius r about the shared rotation axis), a RotationAround-placed tooth
// falls back not to a clean mesh-path success but to the SAME coplanar
// (here, co-cylindrical) contact the mesh boolean's exact predicates cannot
// classify — the r3 probe's own README states the consequence directly: "a
// 12-tooth gear proves analytically end to end while a 17-tooth gear fails
// at the second tooth." That refusal is this design's whole motivation, not
// a defect introduced by it.
//
// A hand-built r3.FromBasis placement (literal-zero z components) stays
// exactly planar for the same n, k and builds analytically instead.
func TestPrismUnionRotatedToothG3(t *testing.T) {
	const r, r2, th1, th2, h = 20.0, 25.0, 0.0, 0.2, 10.0
	const n, k = 17, 10
	pivot := r3.Vec{}
	axis := r3.NewVec(0, 0, 1)

	build := func(t *testing.T, tr r3.Transform) (*decad.Body, error) {
		doc := decad.New()
		hub := hubBody(t, doc, r, h)
		toothSrc := toothBody(t, doc, r, r2, th1, th2, h)
		tooth, err := toothSrc.Placed(tr)
		require.NoError(t, err)
		return decad.Union(hub, tooth)
	}

	t.Run("RotationAround falls back to the mesh path's own coplanar-contact refusal", func(t *testing.T) {
		angle := units.Radians(2 * math.Pi * float64(k) / float64(n))
		tr, err := r3.RotationAround(pivot, axis, angle)
		require.NoError(t, err)
		_, err = build(t, tr)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("hand-built FromBasis builds analytically", func(t *testing.T) {
		theta := 2 * math.Pi * float64(k) / float64(n)
		cos, sin := math.Cos(theta), math.Sin(theta)
		basis := r3.Basis{
			EX: r3.Vec{X: cos, Y: sin, Z: 0},
			EY: r3.Vec{X: -sin, Y: cos, Z: 0},
			EZ: r3.Vec{X: 0, Y: 0, Z: 1},
		}
		tr, err := r3.FromBasis(basis, r3.Vec{})
		require.NoError(t, err)
		got, err := build(t, tr)
		require.NoError(t, err)
		for _, f := range got.Faces() {
			require.NotEqual(t, decad.KindFaceted, f.Surface().Kind())
		}
		vol, err := got.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness) // a circular contribution retires Exact
		require.Greater(t, boundMM3(t, vol), 0.0)
	})
}

// TestPrismUnionIndependentlyConstructedCoplanarFrames is the r3-probe's
// second finding: Frame.N() itself is inexact for an in-plane rotated frame
// (a -0 and a one-to-two-ulp-off-1 Z component appear in practice), and G3
// reads xform.ApplyDir(frame.N()) — so N() is an input to the gate, not just
// the placement. Two operands whose SKETCH PLANES (not merely their
// placements) were constructed independently — one the canonical XY plane,
// one an in-plane-rotated frame that denotes the identical geometric plane —
// can therefore read different worldNormal bit patterns and miss G3, even
// though the two planes are genuinely, geometrically coplanar. Because they
// really are coplanar, the fallback does not merely miss an optimization: it
// lands on the mesh path's own coplanar-contact refusal (the same shape
// TestPrismUnionRotatedToothG3 hits), a more serious cost than a
// non-coplanar miss. This is a finding for the design (recorded in the
// implementation report), not a bug this test papers over: G3 stays exact
// and unconditional (§3.1's own decision), so the pair correctly declines to
// risk admitting a residual.
func TestPrismUnionIndependentlyConstructedCoplanarFrames(t *testing.T) {
	theta := 2 * math.Pi * 2 / 17.0 // the r3 probe's own n=17,k=2 N()-inexact case
	cos, sin := math.Cos(theta), math.Sin(theta)
	rotated, err := r3.NewFrame(r3.Vec{}, r3.NewVec(cos, sin, 0), r3.NewVec(-sin, cos, 0))
	require.NoError(t, err)
	require.NotEqual(t, r3.NewVec(0, 0, 1), rotated.N(), "the probe's own finding: N() is not bit-exact here")

	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)

	wB := sketch.NewWorld()
	planeB, err := wB.CreatePlaneFromFrame(rotated)
	require.NoError(t, err)
	sB, err := wB.CreateSketch(planeB)
	require.NoError(t, err)
	rectB := sB.CreateRectangle(5, 5, 15, 15)
	sB.Fix(rectB.A)
	_, err = sB.Solve(t.Context())
	require.NoError(t, err)
	b, err := doc.Extrude(sB, sB.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	_, err = decad.Union(a, b)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// --- §3.4 post-admission refusal: once resolution finds a unique candidate,
// every further problem is a genuine, typed refusal — never a mesh result.

// TestPrismUnionAdmittedThenInvalidRegionRefuses is RB1 (§9): a pair that
// clears G1-G6 and whose chain closes into one simple loop, but whose
// private scene's arrangement reports an invalid cell (here, a razor-thin
// sliver overlap sketch's own arrangement flags as degenerate), refuses
// explicitly rather than rerouting to an Approximate mesh result.
func TestPrismUnionAdmittedThenInvalidRegionRefuses(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 9.9999999, 0, 20, 10, 10)
	beforeRecipe := doc.Recipe()
	beforeBodies := doc.Bodies()

	_, err := decad.Union(a, b)
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var booleanErr *decad.BooleanError
	require.ErrorAs(t, err, &booleanErr)
	require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
	require.Equal(t, beforeRecipe, doc.Recipe())
	require.Equal(t, beforeBodies, doc.Bodies())
}

// TestPrismUnionCancellationLeavesDocumentUnchanged is §10/§15's
// cancellation obligation: a context canceled before the call returns
// ctx.Err() unchanged, with the document and recipe untouched — the same
// contract every other modify-op audit honors.
func TestPrismUnionCancellationLeavesDocumentUnchanged(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	beforeRecipe := doc.Recipe()
	beforeBodies := doc.Bodies()

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := decad.UnionContext(ctx, a, b)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, beforeRecipe, doc.Recipe())
	require.Equal(t, beforeBodies, doc.Bodies())
}

// --- §8 downstream consequences: chaining and modify/survey reach.

// TestPrismUnionChainedBooleanCarriesNoAccumulatedBound is §8's "no
// chaining" consequence and §15's own required test: a second Union
// consuming the first's analytic result is itself admitted (a prismPayload
// operand, same plane), so its own Exactness/Bound are governed purely by
// §7's rule — never by an accumulated tessellation meshBound, which does not
// exist on this lineage at all.
//
// c's boundary genuinely CUTS one of first's own walls partway (rather than
// meeting it at a shared whole edge or missing it entirely), so the merged
// section carries a Partial LineSeg whose walked fraction is an ordinary
// (non-Pythagorean-exact) float — the moments engine's own, pre-existing
// rule for ANY such record, chained or not (moments.go, unmodified by this
// design): that segment's own length carries a proven, ordinary
// floating-point bound. The volume is still exactly correct (2540 mm^3, the
// closed-form area 254 mm^2 times the 10 mm height); what §7/§8 promise is
// that its Bound stays at this ordinary per-mechanism scale, never the large,
// accumulating meshBound a chained mesh boolean would compose.
func TestPrismUnionChainedBooleanCarriesNoAccumulatedBound(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	first, err := decad.Union(a, b)
	require.NoError(t, err)
	firstVol, err := first.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, firstVol.Exactness)

	c := boxBody(t, doc, 12, 8, 22, 18, 10)
	second, err := decad.Union(first, c)
	require.NoError(t, err)
	secondVol, err := second.Volume()
	require.NoError(t, err)
	require.InDelta(t, 2540.0, volumeMM(t, secondVol), 1e-9)
	// An ordinary moments-engine bound is many orders of magnitude below the
	// coarsest meshBound a chained mesh boolean could ever report (the
	// boolean chord tolerance floor alone is diameter * 2e-5).
	require.Less(t, boundMM3(t, secondVol), 1e-9)
}

// TestPrismUnionDownstreamFilletAndWallSurvey is §8's "analytic identity
// dies" consequence, removed for the admitted class: Fillet and
// MinWallThickness both succeed on the analytic union's own prismPayload,
// where a mesh-path union's facetedPayload permanently refuses both
// (modify-reach SX9, payload-verification's survey wiring).
func TestPrismUnionDownstreamFilletAndWallSurvey(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 5, 5, 15, 15, 10)
	got, err := decad.Union(a, b)
	require.NoError(t, err)

	filleted, err := got.Fillet(verticalConvexEdge(), units.Millimeters(1))
	require.NoError(t, err)
	filletVol, err := filleted.Volume()
	require.NoError(t, err)
	require.Less(t, volumeMM(t, filletVol), 1750.0) // the fillet removed material

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotNil(t, report.Bodies[0].MinWallThickness)

	// Contrast: a mesh-path union — two boxes with disjoint z ranges, so the
	// boolean itself succeeds as a 2-lump facetedPayload — refuses both
	// permanently.
	meshDoc := decad.New()
	meshA := boxBody(t, meshDoc, 0, 0, 10, 10, 10)
	meshBSrc := boxBody(t, meshDoc, 5, 5, 15, 15, 10)
	farAbove, err := r3.Translation(r3.Vec{Z: 30})
	require.NoError(t, err)
	meshB, err := meshBSrc.Placed(farAbove)
	require.NoError(t, err)
	meshGot, err := decad.Union(meshA, meshB)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(meshGot), "expected the mesh path to run and produce a Faceted body")

	_, err = meshGot.Fillet(decad.Edges().AtLeast(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	meshReport, err := meshDoc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, meshReport.Bodies, 1)
	require.Nil(t, meshReport.Bodies[0].MinWallThickness)
}
