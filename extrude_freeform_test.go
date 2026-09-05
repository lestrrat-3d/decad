package decad_test

import (
	"bytes"
	"encoding/json"
	"io"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/spline-design.md §10 P4b's build-level test obligation
// (§11): a Tier A free-form section now builds through the public Extrude
// call. Every test here asserts on computed geometry — a volume, a bound, a
// topology count, a bool the certificate decided — never merely "it ran"
// (CLAUDE.md's hard rules). spline_fit_test.go's
// TestExtrudeFitSplineProfileBuilds carries the headline Volume assertion;
// this file covers the rest of §11's obligations.

// fitSplineArchSketch builds the fit-spline profile the headline test uses:
// a shallow hump through (0,0), (4,3), (8,0) closed by a chord, whose region
// area is the exact rational 15 mm² (spline_fit_test.go).
func fitSplineArchSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	s.CreateLine(end, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// freeformWallFace picks the one built face whose surface is a NURBSSurface.
func freeformWallFace(t *testing.T, body *decad.Body) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.NURBSSurface); ok {
			require.Nil(t, found, "exactly one free-form wall in this fixture")
			found = f
		}
	}
	require.NotNil(t, found, "the fit-spline wall must build as a NURBSSurface")
	return found
}

// TestExtrudeFreeformAreaIsRegionPlusPerimeter asserts observable test 2: a
// free-form prism's Area is 2*(region area) + (wall lengths)*height, with a
// POSITIVE bound — an arc length is never exact in any tier
// (docs/spline-design.md §6.1), so a test pinning Exact here would pin a bug.
func TestExtrudeFreeformAreaIsRegionPlusPerimeter(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	record, _, err := decad.RecordProfile(s, p)
	require.NoError(t, err)
	regionArea, err := record.Area()
	require.NoError(t, err)

	d := decad.New()
	const height = 10.0
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)

	// Sum every SIDE face's own area (never a cap's, told apart by role) —
	// each wall face already carries length*height (extrude.go's faceArea).
	wallAreaSum := 0.0
	for _, f := range body.Faces() {
		origins := f.Origins()
		require.NotEmpty(t, origins)
		if len(origins[0].Role) < 5 || origins[0].Role[:5] != "side(" {
			continue // a cap
		}
		fa, err := f.Area()
		require.NoError(t, err)
		wallAreaSum += fa.Value.Mag()
	}

	bodyArea, err := body.Area()
	require.NoError(t, err)
	want := 2*regionArea.Value.Mag() + wallAreaSum
	require.InDelta(t, want, bodyArea.Value.Mag(), 1e-9,
		"Area is 2*region + the swept perimeter, exactly as a straight-walled prism's is")
	require.Greater(t, bodyArea.Bound.Mag(), 0.0,
		"an arc length is never exact in any tier, so this bound is never zero")
}

// TestExtrudeFreeformTopology asserts observable test 3: one side face per
// coalesced walk plus two caps, the free-form side face tagged KindNURBS,
// both its rim edges type-asserting NURBSCurve, every edge manifold (exactly
// two faces), and Face.Origins() carrying the side(0,j) role of the recorded
// segment.
func TestExtrudeFreeformTopology(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	record, _, err := decad.RecordProfile(s, p)
	require.NoError(t, err)
	require.Len(t, record.Outer.Segments, 2, "the fit spline plus its closing chord")

	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	faces := body.Faces()
	require.Len(t, faces, 4, "one free-form wall, one straight wall, two caps — no coalescing across kinds")

	wall := freeformWallFace(t, body)
	require.Equal(t, decad.KindNURBS, wall.Surface().Kind())

	nurbsEdges := 0
	for _, e := range wall.Edges() {
		if _, ok := e.Curve().(decad.NURBSCurve); ok {
			nurbsEdges++
		}
	}
	require.Equal(t, 2, nurbsEdges, "the free-form wall's own two rim edges, one per cap")

	for _, e := range body.Edges() {
		require.Len(t, e.Faces(), 2, "every edge of a closed solid bounds exactly two faces")
	}

	origins := wall.Origins()
	require.Len(t, origins, 1)
	require.Regexp(t, `^side\(0,[01]\)$`, origins[0].Role,
		"the free-form wall's role names the outer loop and its own recorded segment index")
}

// TestExtrudeFreeformRimLengthPublished asserts observable test 4:
// Edge.Length() on a free-form rim reports a POSITIVE bound and Approximate
// — never Exact 0 mm, which topology.go's own doc comment warns a rim left
// unset would silently publish.
func TestExtrudeFreeformRimLengthPublished(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	wall := freeformWallFace(t, body)
	seen := 0
	for _, e := range wall.Edges() {
		if _, ok := e.Curve().(decad.NURBSCurve); !ok {
			continue
		}
		seen++
		length, err := e.Length()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, length.Exactness)
		require.Greater(t, length.Bound.Mag(), 0.0)
		// The curve runs from (0,0) to (8,0) through (4,3), so its length is
		// strictly longer than the 8 mm chord and well under the 3+3+... loose
		// ceiling a hump this shallow cannot approach.
		require.Greater(t, length.Value.Mag(), 8.0)
		require.Less(t, length.Value.Mag(), 15.0)
	}
	require.Equal(t, 2, seen)
}

// TestExtrudeFreeformNormalAtRefuses asserts observable test 5: NormalAt on
// the built free-form face refuses ErrUnsupported through the public API —
// topology_internal_test.go's TestNormalAtRefusesNURBSSurface pins the same
// fact against a bare *Face built directly.
func TestExtrudeFreeformNormalAtRefuses(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	wall := freeformWallFace(t, body)
	_, err = wall.NormalAt(r3.NewVec(4, 1, 5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// rectangleWithBulgeSketch builds a 10x10 mm square whose three straight
// walls (A->B->C->D) are unchanged and whose fourth (D->A) is a cubic spline
// bulging either AWAY from the square (bulgeOut, an outward bump — convex)
// or INTO it (a bite — concave), through the same two interior control
// points reflected across the D-A chord. The rest of the loop stays
// identical between the two calls, so only the one wall's own curvature
// sign differs — never a re-walked or re-oriented loop.
func rectangleWithBulgeSketch(t *testing.T, bulgeOut bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	sign := 1.0
	if bulgeOut {
		sign = -1
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 10)
	d := s.CreatePoint(0, 10)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	m1 := s.CreatePoint(sign*3, 7)
	m2 := s.CreatePoint(sign*3, 3)
	_, err = s.CreateSpline(d, m1, m2, a)
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestExtrudeFreeformConvexityBothDirections asserts observable test 6: the
// SAME wall shape closes convex on one strict sign and concave on the other,
// and a test covering only one direction cannot tell the certificate from a
// constant (docs/spline-design.md §6.5).
func TestExtrudeFreeformConvexityBothDirections(t *testing.T) {
	t.Parallel()
	convexOf := func(bulgeOut bool) bool {
		s, p := rectangleWithBulgeSketch(t, bulgeOut)
		d := decad.New()
		body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		wall := freeformWallFace(t, body)
		var convex bool
		seen := 0
		for _, e := range wall.Edges() {
			if _, ok := e.Curve().(decad.NURBSCurve); !ok {
				continue
			}
			seen++
			convex = e.IsConvex()
		}
		require.Equal(t, 2, seen)
		return convex
	}

	out := convexOf(true)
	in := convexOf(false)
	require.True(t, out, "a bump bulging away from the square's material is convex")
	require.False(t, in, "a bite bulging into the square's material is concave")
}

// TestExtrudeFreeformStraightWalkBuilds asserts observable test 7: a
// degree-2 unit-weight NURBSSeg on three COLLINEAR points has K identically
// zero (Table K), so §6.5 routes it to evaluator §3's loop-role rule — the
// straight-wall rule — rather than refusing, and it publishes a bool rather
// than staying unresolved.
func TestExtrudeFreeformStraightWalkBuilds(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	p1 := s.CreatePoint(1, 0)
	p2 := s.CreatePoint(2, 0)
	_, err = s.CreateNURBS(2, []*sketch.Point{p0, p1, p2}, []float64{1, 1, 1}, []float64{0, 0, 0, 1, 1, 1})
	require.NoError(t, err)
	p3 := s.CreatePoint(2, 3)
	p4 := s.CreatePoint(0, 3)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p0)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	require.NotNil(t, body)

	wall := freeformWallFace(t, body)
	seen := 0
	for _, e := range wall.Edges() {
		if _, ok := e.Curve().(decad.NURBSCurve); ok {
			seen++
			// The loop-role rule reads !holeLoop for the outer loop: convex.
			require.True(t, e.IsConvex(), "the straight-walk case takes the loop-role rule (outer convex)")
		}
	}
	require.Equal(t, 2, seen)
}

// TestExtrudeFreeformJointCarriesChainVerdict asserts observable test 8:
// Table K's degree-1 row. A unit-weight NURBSSeg on (0,0), (1,0), (1,1),
// degree 1, knots [0,0,1,2,2] converts to two degree-1 spans (each verdict
// 0, since C” = 0 at degree 1), so the WHOLE chain's verdict is carried by
// the single interior joint between them — a left turn, so convex.
func TestExtrudeFreeformJointCarriesChainVerdict(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	p1 := s.CreatePoint(1, 0)
	p2 := s.CreatePoint(1, 1)
	_, err = s.CreateNURBS(1, []*sketch.Point{p0, p1, p2}, []float64{1, 1, 1}, []float64{0, 0, 1, 2, 2})
	require.NoError(t, err)
	p3 := s.CreatePoint(0, 1)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p0)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	wall := freeformWallFace(t, body)
	seen := 0
	for _, e := range wall.Edges() {
		if _, ok := e.Curve().(decad.NURBSCurve); ok {
			seen++
			require.True(t, e.IsConvex(), "the interior joint at (1,0) turns left: convex")
		}
	}
	require.Equal(t, 2, seen)
}

// r19Fixture builds a profile whose one wall is the named cubic spline
// control net, closed by a chord back to its start.
func r19Fixture(t *testing.T, control [4][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(control))
	for i, c := range control {
		points[i] = s.CreatePoint(c[0], c[1])
	}
	_, err = s.CreateSpline(points...)
	require.NoError(t, err)
	s.CreateLine(points[len(points)-1], points[0])
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestExtrudeFreeformR19RefusesTheBuild asserts observable test 9: a wall
// edge whose chain proves no single curvature sign is a BUILD refusal, not a
// Suspect reading — evaluator §3 decides convex at build and a bool has no
// Suspect to fall back on (docs/spline-design.md §6.5). Three shapes, three
// ways to reach it: two genuine curvature sign changes inside one span, an
// interior cusp the coefficient fold alone would call strictly signed, and a
// coincident first control pair whose cusp a half-open root count alone would
// miss (spline_convexity_internal_test.go pins all three at the certificate
// level; this pins them as BUILD refusals through the public Extrude).
func TestExtrudeFreeformR19RefusesTheBuild(t *testing.T) {
	t.Parallel()
	cases := map[string][4][2]float64{
		"two curvature sign changes":    {{0, 0}, {1, 0}, {-4, 1}, {0.9, 0}},
		"coincident first control pair": {{0, 0}, {0, 0}, {1.0 / 3, 0}, {1, 1}},
	}
	for name, control := range cases {
		t.Run(name, func(t *testing.T) {
			s, p := r19Fixture(t, control)
			d := decad.New()
			body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
			require.Error(t, err)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, d.Bodies(), "a refused extrude registers no body")
			require.Empty(t, d.Recipe().Steps, "a refused extrude records no step")
		})
	}

	// The interior-cusp net has no direction at t=1/2 (u=1/8, v=-1/12 twice in
	// a row is NOT the shape — it is the net whose SPEED, not its coefficients,
	// vanishes at the midpoint of the span).
	t.Run("interior cusp", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		control := [][2]float64{{-1.0 / 8, 1.0 / 4}, {1.0 / 8, -1.0 / 12}, {-1.0 / 8, -1.0 / 12}, {1.0 / 8, 1.0 / 4}}
		points := make([]*sketch.Point, len(control))
		for i, c := range control {
			points[i] = s.CreatePoint(c[0], c[1])
		}
		_, err = s.CreateSpline(points...)
		require.NoError(t, err)
		s.CreateLine(points[len(points)-1], points[0])
		profiles := s.Profiles()
		require.Len(t, profiles, 1)

		d := decad.New()
		body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		require.Error(t, err)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Empty(t, d.Bodies())
		require.Empty(t, d.Recipe().Steps)
	})
}

// TestExtrudeClosedSplineTwoLoopsAndSound extends TestExtrudeClosedSplineProfileBuilds
// (spline_moments_test.go) with observable test 10's topology and Verify
// obligations: a lone ClosedSplineSeg builds ONE side face carrying TWO
// boundary loops (one per cap), the seam vertex shared by both rim edges'
// start and end, and Verify reports Solid/Watertight/Manifold — never
// Unsound, which is what a loop-less closed face would report
// (verify.go's loop-less-face audit has no arm for KindNURBS).
func TestExtrudeClosedSplineTwoLoopsAndSound(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(closedSplineControls))
	for i, control := range closedSplineControls {
		points[i] = s.CreatePoint(control[0], control[1])
	}
	_, err = s.CreateClosedSpline(points...)
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	faces := body.Faces()
	require.Len(t, faces, 3, "one closed free-form wall plus two caps, no junction vertex")

	wall := freeformWallFace(t, body)
	loops := wall.Loops()
	require.Len(t, loops, 2, "a closed cylindrical band has two boundary loops, one per cap")

	bottomStart := loops[0].CoEdges()[0].Start()
	bottomEnd := loops[0].CoEdges()[0].End()
	require.Same(t, bottomStart, bottomEnd, "the seam vertex is shared by the rim edge's own start and end")

	report, err := d.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	br := report.Bodies[0]
	require.True(t, br.Solid)
	require.True(t, br.Watertight)
	require.True(t, br.Manifold)
	require.False(t, br.SelfIntersecting)
	require.NotEqual(t, decad.Unsound, br.Status,
		"a loop-less closed NURBSSurface face would report Unsound; this body has two loops")
}

// TestExtrudeFreeformVerifySound asserts observable test 13: Verify on the
// built fit-spline prism reports Solid/Watertight/Manifold, not
// SelfIntersecting, a Volume matching the direct build, and — with the
// tolerance-gate arm this increment depends on (#178) — no
// DiagMeasurementBeyondTolerance at the default tolerance.
func TestExtrudeFreeformVerifySound(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	directVolume, err := body.Volume()
	require.NoError(t, err)

	report, err := d.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	br := report.Bodies[0]
	require.True(t, br.Solid)
	require.True(t, br.Watertight)
	require.True(t, br.Manifold)
	require.False(t, br.SelfIntersecting)
	require.NotNil(t, br.Volume)
	require.InDelta(t, directVolume.Value.Mag(), br.Volume.Value.Mag(), 1e-9)

	for _, diag := range report.Diagnostics {
		require.NotEqual(t, decad.DiagMeasurementBeyondTolerance, diag.Code,
			"the free-form diameter-gate arm (#178) must give this body a reference")
	}
}

// TestExtrudeFreeformRecipeRoundTrips asserts observable test 14: the
// OpExtrude step carrying a free-form ProfileRecord encodes and decodes
// byte-stably and passes the recipe's own strict decode.
func TestExtrudeFreeformRecipeRoundTrips(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	_, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	recipe := d.Recipe()
	require.Len(t, recipe.Steps, 1)
	require.Equal(t, decad.OpExtrude, recipe.Steps[0].Op)

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, "the recorded recipe round-trips")

	buf2, err := json.Marshal(got)
	require.NoError(t, err)
	require.Equal(t, buf, buf2, "the round-tripped recipe re-encodes byte-stably")
}

// TestExtrudeFreeformPlacementReproducesVolume asserts observable test 15:
// Placed, Duplicate and PlacedCopy of a free-form prism each reproduce the
// unplaced volume. Each mints its own R7 work ceiling (prismPayload.placed
// carries no preflight counter across the call), so each re-spends the full
// budget — cheap for this fixture, but the reason a caller placing a large
// free-form body many times pays for it every time.
func TestExtrudeFreeformPlacementReproducesVolume(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	base, err := body.Volume()
	require.NoError(t, err)

	move, err := r3.Translation(r3.NewVec(5, -3, 2))
	require.NoError(t, err)

	// Duplicate and PlacedCopy leave the receiver LIVE; Placed retires it
	// (core §8), so it runs last.
	dup, err := body.Duplicate()
	require.NoError(t, err)
	dv, err := dup.Volume()
	require.NoError(t, err)
	require.InDelta(t, base.Value.Mag(), dv.Value.Mag(), 1e-9, "Duplicate reproduces the volume")

	copied, err := body.PlacedCopy(move)
	require.NoError(t, err)
	cv, err := copied.Volume()
	require.NoError(t, err)
	require.InDelta(t, base.Value.Mag(), cv.Value.Mag(), 1e-9, "PlacedCopy reproduces the volume")

	placed, err := body.Placed(move)
	require.NoError(t, err)
	pv, err := placed.Volume()
	require.NoError(t, err)
	require.InDelta(t, base.Value.Mag(), pv.Value.Mag(), 1e-9, "Placed reproduces the volume")
}

// TestExtrudeFreeformTierBCRefusesWithTierACounterpart asserts observable
// test 16's R10 row: a Tier B (whole ellipse) or Tier C (unequal-weight
// NURBS) section is ErrUnsupported at the build, with a Tier A counterpart
// of the SAME evaluator building in the same test — so a caller can tell the
// refusal is about the tier, not about free-form sections in general.
func TestExtrudeFreeformTierBCRefusesWithTierACounterpart(t *testing.T) {
	t.Parallel()
	// Tier A counterpart: the fit-spline arch builds.
	s, p := fitSplineArchSketch(t)
	d := decad.New()
	body, err := d.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.NotNil(t, body)

	// Tier C: an unequal-weight NURBS section is ErrUnsupported (R10).
	w2 := sketch.NewWorld()
	s2, err := w2.CreateSketch(w2.XY())
	require.NoError(t, err)
	p0 := s2.CreatePoint(0, 0)
	p1 := s2.CreatePoint(1, 2)
	p2 := s2.CreatePoint(2, 0)
	_, err = s2.CreateNURBS(2, []*sketch.Point{p0, p1, p2}, []float64{1, 2, 1}, []float64{0, 0, 0, 1, 1, 1})
	require.NoError(t, err)
	s2.CreateLine(p2, p0)
	profiles2 := s2.Profiles()
	require.Len(t, profiles2, 1)

	d2 := decad.New()
	body2, err := d2.Extrude(s2, profiles2[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.Error(t, err)
	require.Nil(t, body2)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, d2.Bodies())
}

// TestExtrudeFreeformCollapsedControlNetRefusesR14 asserts observable test
// 16's R14 row against a record built directly (record.go's own doc
// comments already state ProfileRecord validates its own fields): a
// free-form control net collapsed to a single point has no boundary at all,
// so ProfileRecord.Area — which Extrude's own falsifier calls before any
// build — refuses ErrDegenerate rather than integrate a zero-area region.
// A caller-drawn sketch never reaches this at all: CreateClosedSpline on
// three coincident points authenticates to zero live profiles, so the
// degenerate net never becomes a candidate Extrude could even be offered.
func TestExtrudeFreeformCollapsedControlNetRefusesR14(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	same := s.CreatePoint(3, 3)
	_, err = s.CreateClosedSpline(same, same, same)
	require.NoError(t, err, "sketch accepts the entity; its OWN arrangement drops the degenerate loop")
	require.Empty(t, s.Profiles(), "a collapsed closed spline authenticates to zero live profiles")

	same2 := decad.Point2{U: 3, V: 3}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.ClosedSplineSeg{Control: []decad.Point2{same2, same2, same2}, CCW: true, TStart: 0, TEnd: 1},
	}}}
	_, err = record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

// TestExtrudeFreeformFalsifierDenseSample asserts observable test 18: a
// dense sample of the true recorded curve is a FALSIFIER, never an admission
// gate. Every sampled point of the closed-spline fixture's own boundary must
// sit inside the built body's published Bounds, widened by its Bound.
func TestExtrudeFreeformFalsifierDenseSample(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(closedSplineControls))
	for i, control := range closedSplineControls {
		points[i] = s.CreatePoint(control[0], control[1])
	}
	_, err = s.CreateClosedSpline(points...)
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	box, err := body.Bounds()
	require.NoError(t, err)
	bound := box.Bound.Mag()

	coords := make([][2]float64, len(closedSplineControls))
	copy(coords, closedSplineControls)
	// A ClosedSplineSeg is a PERIODIC B-spline (record.go), so the reference
	// sampler must be too — spline_moments_test.go's own densePolylineArea
	// uses the same function for the same fixture.
	ring, err := geom.SamplePeriodicCubicBSpline(coords, 20000)
	require.NoError(t, err)
	require.NotEmpty(t, ring)

	for _, pt := range ring {
		require.GreaterOrEqual(t, pt[0], box.Min.X-bound, "a sampled X must sit inside the published box")
		require.LessOrEqual(t, pt[0], box.Max.X+bound)
		require.GreaterOrEqual(t, pt[1], box.Min.Y-bound)
		require.LessOrEqual(t, pt[1], box.Max.Y+bound)
	}
}

// The section below covers the OTHER capabilities a free-form-walled body
// reaches (docs/spline-design.md §8 Table C). Chording lands them in two
// groups. Tessellation, export, the booleans and the interference proof read
// the chorded mesh and are asserted on computed geometry — a watertight
// manifold, a volume within its own published bound, a decided pair. The
// analytic surveys and the clearance kernel read analytic faces, have no
// free-form arm, and stay undecided; each of those tests pins the exact
// sentinel or diagnostic so a later change cannot widen one silently.

// freeformArchBody extrudes the fit-spline arch profile ((0,0), (4,3), (8,0)
// closed by a chord) used throughout extrude_freeform_test.go, into doc, by
// 10 mm. Its one free-form wall is the fit-spline span.
func freeformArchBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := fitSplineArchSketch(t)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// straightNURBSPrismBody extrudes a 10x10 rectangle by 5 mm; its bottom edge
// is recorded as a degree-1 unit-weight NURBSSeg rather than a LineSeg — the
// straight-walk case (docs/spline-design.md §6.5's Table K, "K identically
// zero"). The refusal every capability below stages on is keyed to the
// RECORDED kind, never the degree or the geometric straightness, so this
// fixture must trip the identical refusal a curved free-form wall does.
func straightNURBSPrismBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	p1 := s.CreatePoint(10, 0)
	_, err = s.CreateNURBS(1, []*sketch.Point{p0, p1}, []float64{1, 1}, []float64{0, 0, 1, 1})
	require.NoError(t, err)
	p2 := s.CreatePoint(10, 10)
	p3 := s.CreatePoint(0, 10)
	s.CreateLine(p1, p2)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p0)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// overshootNetControls is docs/spline-design.md §6.2.1's own overshooting
// control net: a cubic whose two interior control points sit a hundredth of a
// millimetre off the chord through its ends while the curve itself swings
// roughly 0.76 mm past that chord's start. It is the shape that separates a
// chord-SEGMENT sagitta from a carrier-LINE one, so it is also the shape a
// chording arm must not under-chord.
var overshootNetControls = [4][2]float64{{0, 0}, {-3, 0.01}, {4, 0.01}, {1, 0}}

// overshootNetPoint evaluates that cubic Bézier at parameter s, over the same
// control net the sketch records, by de Casteljau — an independent reference
// the tessellation never consults.
func overshootNetPoint(s float64) (float64, float64) {
	pts := overshootNetControls
	work := pts[:]
	buf := make([][2]float64, len(work))
	copy(buf, work)
	for round := len(buf) - 1; round > 0; round-- {
		for i := range round {
			buf[i][0] = buf[i][0] + s*(buf[i+1][0]-buf[i][0])
			buf[i][1] = buf[i][1] + s*(buf[i+1][1]-buf[i][1])
		}
	}
	return buf[0][0], buf[0][1]
}

// overshootNetBody extrudes that net, closed back to its start by a straight
// chord, into a 5 mm prism.
func overshootNetBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(overshootNetControls))
	for i, c := range overshootNetControls {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	_, err = s.CreateNURBS(3, pts, []float64{1, 1, 1, 1}, []float64{0, 0, 0, 0, 1, 1, 1, 1})
	require.NoError(t, err)
	s.CreateLine(pts[len(pts)-1], pts[0])
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// segmentDistance is the distance from p to the segment ab, in the plane.
func segmentDistance(p, a, b [2]float64) float64 {
	dx, dy := b[0]-a[0], b[1]-a[1]
	den := dx*dx + dy*dy
	s := 0.0
	if den > 0 {
		s = math.Max(0, math.Min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
	}
	return math.Hypot(p[0]-(a[0]+s*dx), p[1]-(a[1]+s*dy))
}

// TestFreeformPrismTessellatesItsChordedWall is docs/tessellation-reach-design.md
// §5's headline: a Tier A free-form wall chords on the existing prism path.
// The mesh must be a closed, consistently oriented manifold, its facets must
// name the body's own faces, and the volume it encloses must land on the body's
// exactly measured volume from below — a chorded outline is inscribed, so it
// can only lose area to the curve it stands in for, never gain it.
func TestFreeformPrismTessellatesItsChordedWall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)

	const tol = 0.02
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	require.LessOrEqual(t, mesh.Bound().Base(), tol,
		"an unplaced body's whole bound is chording plus a coordinate rounding, and chording alone is capped by the requested tolerance")
	require.Positive(t, mesh.Bound().Base(), "a chorded free-form wall never publishes an exact mesh")

	faces := map[*decad.Face]struct{}{}
	for _, f := range body.Faces() {
		faces[f] = struct{}{}
	}
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()), "every facet names a source face")
	for _, f := range mesh.SourceFaces() {
		require.Contains(t, faces, f, "a facet's source face must be one of the body's own")
	}

	vol, err := body.Volume()
	require.NoError(t, err)
	held := meshVolume(mesh)
	require.Positive(t, held, "the facets must be wound outward")
	require.Less(t, held, vol.Value.Base(), "the inscribed chorded prism holds strictly less than the curved one")
	require.Greater(t, held, vol.Value.Base()*0.99, "a chording this fine loses under a percent of the volume")

	// The deficit is chording, not a fixed modelling error, so asking for a
	// four-times finer chord must shrink it.
	finer, err := body.Tessellate(units.Millimeters(tol / 4))
	require.NoError(t, err)
	require.Less(t, vol.Value.Base()-meshVolume(finer), vol.Value.Base()-held,
		"a finer chording must recover volume the coarser one lost")
}

// TestFreeformPrismChordingEnclosesTheCurveBothWays is §5's falsifier on
// docs/spline-design.md §6.2.1's own overshooting net, the shape whose true
// departure from its chord is two orders of magnitude past what a carrier-line
// reading would report. Both directions of the Hausdorff claim [Mesh.Bound]
// makes are checked: no densely sampled point of the recorded curve lies
// farther than Bound from the chorded polyline, and no chorded vertex lies
// farther than Bound from the curve. The vertex direction is checked far
// tighter than Bound — a station is an exact point ON the curve, so it may
// only miss the reference polyline by that polyline's own sampling error.
func TestFreeformPrismChordingEnclosesTheCurveBothWays(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := overshootNetBody(t, doc)

	const tol = 0.01
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	bound := mesh.Bound().Base()
	require.LessOrEqual(t, bound, tol)

	// The chorded outline, read off the mesh: every vertex at the base level,
	// in the order the walls chain them, is a boundary sample.
	verts := mesh.Vertices()
	outline := make([][2]float64, 0, len(verts)/2)
	for i := 0; i < len(verts); i += 2 {
		require.Zero(t, verts[i].Z, "the base ring sits exactly on the sketch plane")
		require.InDelta(t, 5.0, verts[i+1].Z, 1e-12, "the top ring sits exactly at the extrusion height")
		outline = append(outline, [2]float64{verts[i].X, verts[i].Y})
	}
	require.GreaterOrEqual(t, len(outline), 4, "this net needs several chords at this tolerance")

	const samples = 20000
	curve := make([][2]float64, samples+1)
	for i := range curve {
		x, y := overshootNetPoint(float64(i) / samples)
		curve[i] = [2]float64{x, y}
	}

	// Direction 1 — the falsifier: no point of the true curve is farther from
	// the chorded outline than the mesh claims.
	worstCurve := 0.0
	for _, p := range curve {
		best := math.Inf(1)
		for i := range outline {
			best = math.Min(best, segmentDistance(p, outline[i], outline[(i+1)%len(outline)]))
		}
		worstCurve = math.Max(worstCurve, best)
	}
	require.LessOrEqual(t, worstCurve, bound,
		"a point of the recorded curve sits farther from the mesh than the mesh's own published bound")
	require.Positive(t, worstCurve, "this net genuinely departs from its chords — the check is not vacuous")

	// Direction 2: every chorded station is a point ON the curve, so it may
	// only miss the dense reference polyline by that polyline's own sagitta.
	for j, p := range outline {
		best := math.Inf(1)
		for i := range curve[:len(curve)-1] {
			best = math.Min(best, segmentDistance(p, curve[i], curve[i+1]))
		}
		require.Less(t, best, 1e-9, "station %d must lie on the recorded curve, not merely near it", j)
	}
}

// TestFreeformPrismChordCountTracksTheTolerance pins the two ends of the
// chording budget: doubling the tolerance never asks for more chords, and a
// tolerance no bisection this evaluator admits can reach refuses outright
// rather than returning a coarser mesh under a bound it cannot prove.
//
// The refusal that binds at the fine end is the free-form work counter
// (docs/spline-design.md Table R row R7), not the chord cap (row R8): an exact
// dyadic bisection charges its own arithmetic, and that ceiling is reached long
// before 2^14 chords are. Both are ErrUnsupported and both are §5's refusal
// table, so the assertion is on the sentinel and on nothing being returned.
func TestFreeformPrismChordCountTracksTheTolerance(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)

	previous := 0
	for _, tol := range []float64{0.015625, 0.03125, 0.0625, 0.125, 0.25, 0.5, 1} {
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		require.LessOrEqual(t, mesh.Bound().Base(), tol, "tol %v: the published bound stays within the tolerance asked for", tol)
		count := len(mesh.Vertices())
		if previous > 0 {
			require.LessOrEqual(t, count, previous, "tol %v: doubling the tolerance must never ask for more chords", tol)
		}
		previous = count
	}

	mesh, err := body.Tessellate(units.Millimeters(1e-12))
	require.Nil(t, mesh, "a refused tessellation returns no mesh")
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestFreeformPrismExportsDeterministically pins the STL and OBJ rows: both
// write the chorded mesh, and both are byte-identical across calls (export.go's
// determinism contract) on the 15-point involute fit spline, the heaviest
// free-form fixture this package builds.
func TestFreeformPrismExportsDeterministically(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := involuteFlankSketch(t)
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)

	for _, tc := range []struct {
		name  string
		write func(w io.Writer) error
	}{
		{"STL", func(w io.Writer) error { return body.STL(w) }},
		{"OBJ", func(w io.Writer) error { return body.OBJ(w) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var first, second bytes.Buffer
			require.NoError(t, tc.write(&first))
			require.NoError(t, tc.write(&second))
			require.NotZero(t, first.Len(), "a free-form-walled body exports a non-empty mesh")
			require.Equal(t, first.Bytes(), second.Bytes(), "two exports of one body must be byte-identical")
		})
	}
}

// TestFreeformPrismCutsAnInteriorBox pins the boolean row: a free-form-walled
// operand now tessellates, so the mesh boolean can consume it. Cutting a box
// that lies STRICTLY inside the prism leaves a sealed void, and the result's
// volume must land within its own published bound of the exact difference of
// the two operands' volumes.
func TestFreeformPrismCutsAnInteriorBox(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)
	prismVolume, err := body.Volume()
	require.NoError(t, err)

	lift, err := r3.Translation(r3.NewVec(0, 0, 3))
	require.NoError(t, err)
	inner, err := boxBody(t, doc, 3, 0.5, 5, 1.5, 4).Placed(lift)
	require.NoError(t, err)
	innerVolume, err := inner.Volume()
	require.NoError(t, err)

	result, err := decad.Cut(body, inner)
	require.NoError(t, err)
	got, err := result.Volume()
	require.NoError(t, err)

	want := prismVolume.Value.Base() - innerVolume.Value.Base()
	require.InDelta(t, want, got.Value.Base(), got.Bound.Base(),
		"the cut result's volume must sit within its own published bound of the exact difference")
	require.Positive(t, got.Bound.Base(), "a chorded operand never yields an exact boolean volume")
}

// TestFreeformPrismInterferenceDecided pins Verify's default pairwise
// interference proof over two overlapping free-form-walled bodies: the pair is
// DECIDED — a proven Interference row with a positive overlap volume and its
// own bound — rather than left undecided under DiagUnsupportedPairPayload.
func TestFreeformPrismInterferenceDecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)
	// Shifted in all three axes, so the pair overlaps in a genuine volume and
	// shares no face plane with its twin.
	shift, err := r3.Translation(r3.NewVec(2, 0.2, 1))
	require.NoError(t, err)
	_, err = body.PlacedCopy(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.False(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload),
		"a chorded free-form operand is no longer an unsupported payload")
	require.Len(t, report.Interferences, 1, "the overlapping pair must publish one proven overlap")
	overlap := report.Interferences[0].Volume
	require.Positive(t, overlap.Value.Base(), "the two copies genuinely overlap")
	require.Less(t, overlap.Value.Base(), 150.0, "the overlap is strictly less than either body's own volume")
	require.Positive(t, overlap.Bound.Base(), "a chorded overlap is measured, never exact")
	require.Equal(t, decad.Interfering, report.Status)
}

// TestFreeformPrismClearanceUndecided pins WithClearances on a box-disjoint
// free-form pair: DiagUndecidedClearance, never a fabricated Gap and never a
// silent Sound pass (clearance.go / verify.go).
func TestFreeformPrismClearanceUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	freeformArchBody(t, doc)
	boxBody(t, doc, 100, 100, 110, 110, 10)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)

	require.True(t, hasDiagnostic(report, decad.DiagUndecidedClearance))
	require.Empty(t, report.Clearances, "an undecided gap never publishes a proven measurement")
	require.NotEqual(t, decad.Sound, report.Status)
}

// TestFreeformPrismMinWallThicknessUndecided pins WithMinWallThickness: a
// nil BodyReport.MinWallThickness with DiagUndecidedWall, never a silent
// pass (survey.go's errFreeformSection through survey.go's DiagUndecidedWall).
func TestFreeformPrismMinWallThicknessUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Nil(t, br.MinWallThickness)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedWall))
}

// TestFreeformPrismUndercutsUndecided pins WithPullDirection: an EMPTY
// BodyReport.Undercuts is the proven all-clear only inside a Sound report
// (verify.go's BodyReport doc comment); here it must come with
// DiagUndecidedUndercut and a non-Sound status, never read as a pass.
func TestFreeformPrismUndercutsUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Empty(t, br.Undercuts)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

// TestFreeformPrismMinRadiusUndecided pins WithMinRadius: a nil
// BodyReport.MinRadius with DiagUndecidedMinRadius, never a silent "no
// concave feature" pass (survey.go).
func TestFreeformPrismMinRadiusUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	freeformArchBody(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Nil(t, br.MinRadius)
	require.NotEqual(t, decad.Sound, br.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedMinRadius))
}

// TestFreeformPrismFilletChamferRefuse pins Fillet and Chamfer's per-corner
// rewrite (fillet.go's prismCornerLoopsBudget): it walks every segment of
// every loop regardless of which corner is selected, so a free-form section
// refuses even when the selected edges are themselves the analytic vertical
// junctions. Both a curved wall and a straight-walk (degree-1) NURBSSeg wall
// must trip the SAME refusal — it is keyed on the recorded kind, not the
// degree (docs/spline-design.md §5.1/§6.5).
func TestFreeformPrismFilletChamferRefuse(t *testing.T) {
	t.Parallel()
	for _, fixture := range []struct {
		name  string
		build func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{"curved fit-spline wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return freeformArchBody(t, doc)
		}},
		{"straight degree-1 NURBSSeg wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return straightNURBSPrismBody(t, doc)
		}},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			t.Run("Fillet", func(t *testing.T) {
				doc := decad.New()
				body := fixture.build(t, doc)
				_, err := body.Fillet(verticalEdges(), units.Millimeters(1))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "free-form boundary segment")
			})
			t.Run("Chamfer", func(t *testing.T) {
				doc := decad.New()
				body := fixture.build(t, doc)
				_, err := body.Chamfer(verticalEdges(), units.Millimeters(1))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "free-form boundary segment")
			})
		})
	}
}

// TestFreeformPrismShellRefuses pins Shell (fillet.go's audit and
// shell_offset.go's own offset loop reversal), over both a curved and a
// straight-walk (degree-1) free-form wall.
func TestFreeformPrismShellRefuses(t *testing.T) {
	t.Parallel()
	for _, fixture := range []struct {
		name  string
		build func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{"curved fit-spline wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return freeformArchBody(t, doc)
		}},
		{"straight degree-1 NURBSSeg wall", func(t *testing.T, doc *decad.Document) *decad.Body {
			return straightNURBSPrismBody(t, doc)
		}},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			doc := decad.New()
			body := fixture.build(t, doc)
			_, err := body.Shell(bothCaps(), units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "free-form boundary segment")
		})
	}
}

// TestFreeformPrismCapLoopChamferRefuses pins the complete-cap-loop chamfer
// path (capblend_geom.go's oneLoopCornerLoop), reached by selecting every
// edge of one cap rather than a single corner.
func TestFreeformPrismCapLoopChamferRefuses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)

	_, err := body.Chamfer(capLoopEdges(body), units.Millimeters(1))

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
}

// TestFreeformPrismRevolveRefuses pins Document.Revolve of a free-form
// section: rejectInteriorContact (revolve.go) walks every segment before
// deciding circularity, so it refuses on the free-form span regardless of
// where the axis sits relative to the profile.
func TestFreeformPrismRevolveRefuses(t *testing.T) {
	t.Parallel()
	s, p := fitSplineArchSketch(t)
	doc := decad.New()

	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})

	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "free-form boundary segment")
	require.Empty(t, doc.Bodies(), "a refused Revolve registers no body")
}

// TestFreeformPrismSelectorNeverMatchesFace pins that a type-keyed FacePredicate
// (Planar, whose match is an `ok` type assertion on Surface) never counts the
// free-form NURBSSurface wall among its matches: the arch profile's two caps
// AND its straight chord wall are all analytic Plane faces, so Exactly(3)
// fails the moment the free-form wall's NURBSSurface leaks in too.
func TestFreeformPrismSelectorNeverMatchesFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)
	require.Len(t, body.Faces(), 4, "two caps plus the spline wall plus the chord wall")

	faces, err := decad.Faces(decad.Planar()).Exactly(3).SelectFaces(body)
	require.NoError(t, err)
	for _, f := range faces {
		_, ok := f.Surface().(decad.NURBSSurface)
		require.False(t, ok, "the free-form wall must never satisfy a Surface type assertion it fails")
	}
}

// TestFreeformPrismSelectorNeverMatchesEdge pins that a type-keyed
// EdgePredicate (ParallelTo, whose match is an `ok` type assertion on Curve)
// never counts a free-form NURBSCurve rim: only the two analytic vertical
// junction edges are Line3, so Exactly(2) fails the moment a rim leaks in.
func TestFreeformPrismSelectorNeverMatchesEdge(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := freeformArchBody(t, doc)

	edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(2).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range edges {
		_, ok := e.Curve().(decad.NURBSCurve)
		require.False(t, ok, "a free-form rim must never satisfy a Curve type assertion it fails")
	}
}

// reversedArchSketch is fitSplineArchSketch's mirror image in RECORDING
// direction only: the same hump through the same three points and the same
// closing chord, but authored so the profile loop walks the spline against the
// curve's own parametrization. The region, and so every measurement of it, is
// identical; only walkFreeform's reversed flag differs.
func reversedArchSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(end, mid, start)
	require.NoError(t, err)
	s.CreateLine(start, end)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestFreeformPrismChordsBothWalkDirectionsIdentically pins the reversal arm of
// the chording (docs/tessellation-reach-design.md §5): a walk running against
// its curve emits the SAME dyadic cell boundaries the forward walk does, in the
// opposite order. Two bodies over the same region, recorded in opposite
// directions, must therefore carry the same chord count, the same published
// bound and the same vertex set — down to the last bit, since every station is
// one rounding of the same exact rational.
func TestFreeformPrismChordsBothWalkDirectionsIdentically(t *testing.T) {
	t.Parallel()
	const tol = 0.02
	meshOf := func(build func(t *testing.T) (*sketch.Sketch, *sketch.Profile)) *decad.Mesh {
		s, p := build(t)
		body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		return mesh
	}
	forward := meshOf(fitSplineArchSketch)
	backward := meshOf(reversedArchSketch)

	require.Equal(t, len(forward.Triangles()), len(backward.Triangles()),
		"the two recording directions must settle on the same chord count")
	require.Equal(t, forward.Bound().Base(), backward.Bound().Base(),
		"the two recording directions must publish the same bound, bit for bit")

	key := func(m *decad.Mesh) map[r3.Vec]int {
		out := map[r3.Vec]int{}
		for _, v := range m.Vertices() {
			out[v]++
		}
		return out
	}
	require.Equal(t, key(forward), key(backward),
		"the two recording directions must land on the identical vertex set")
}
