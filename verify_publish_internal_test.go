package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests verify_publish.go's assembler directly, in-package,
// because bodyResult and its fields are unexported (task-list §PR-1). Every
// fixture below is a real body built through the public Document/Body API —
// no hand-assembled result stands in for a producer's own output.

// publishBody folds opts through the real option resolver and effective
// request, then runs evaluateBody on body — the same path Document.Verify
// takes for one body — and returns the private bodyResult it publishes.
func publishBody(t *testing.T, body *Body, opts ...VerifyOption) *bodyResult {
	t.Helper()
	cfg, err := resolveVerifyOptions(opts)
	require.NoError(t, err)
	req := effectiveVerifyRequest(cfg)
	res, _, err := evaluateBody(t.Context(), body, cfg, req)
	require.NoError(t, err)
	return res
}

// rectangularPrism extrudes a solved w x d rectangle by h, the same fixture
// survey_test.go's rectPrism builds, returning the Body directly.
func rectangularPrism(t *testing.T, w, d, h float64) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, w, d)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestVerifyPublishRectangularPrismPath is the acceptance path (proposal
// §16): a sketch-created rectangle through the real extrusion producer, its
// certified readings, and no options, exercising the whole publication
// assembler end to end on a real 100x60x10 mm plate.
func TestVerifyPublishRectangularPrismPath(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 60, 10)
	res := publishBody(t, body)

	require.Equal(t, Sound, res.Status)
	require.True(t, res.Area.Value.Equal(units.SquareMillimeters(15200), 1e-9))
	require.Equal(t, r3.NewVec(0, 0, 0), res.Bounds.Min)
	require.Equal(t, r3.NewVec(100, 60, 10), res.Bounds.Max)
	require.Equal(t, scalarNotRequested, res.Wall.Outcome)
	require.Nil(t, res.Wall.Request)
	require.Equal(t, assessmentNotEvaluated, res.Wall.Assessment)
	require.Equal(t, scalarNotRequested, res.ConcaveRadius.Outcome)
	require.Nil(t, res.ConcaveRadius.Minimum)

	// Both core readings are exact on this evaluator's plate, so their
	// zero bound passes without a computed Limit.
	require.Equal(t, toleranceSatisfied, res.Area.Tolerance.State)
	require.Nil(t, res.Area.Tolerance.Limit)
	require.Equal(t, toleranceSatisfied, res.Bounds.Tolerance.State)
	require.Nil(t, res.Bounds.Tolerance.Limit)
}

// TestVerifyPublishMeasuredWall replays TestWallCubeSound's 100 mm cube: its
// only spanning ball is the center slab, exactly 100 mm, met at a 1 mm tool
// and at the exact threshold equality case (proposal §16 "Threshold
// equality").
func TestVerifyPublishMeasuredWall(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 100, 100)

	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, scalarMeasured, res.Wall.Outcome)
	require.NotNil(t, res.Wall.Minimum)
	require.True(t, res.Wall.Minimum.Value.Equal(units.Millimeters(100), 1e-9))
	require.Equal(t, Exact, res.Wall.Minimum.Exactness)
	require.Equal(t, assessmentMet, res.Wall.Assessment)
	require.NotNil(t, res.Wall.Request)
	require.True(t, res.Wall.Request.Minimum.Equal(units.Millimeters(1), 1e-9))

	equality := publishBody(t, body, WithMinWallThickness(units.Millimeters(100)))
	require.Equal(t, assessmentMet, equality.Wall.Assessment, "L >= T including equality is Met")
}

// TestVerifyPublishViolatedWall replays TestWallThinPlateViolating's
// 10x10x0.5 mm plate: the proven [0.5, 0.5] mm interval sits below a 1 mm
// tool at any coarseness.
func TestVerifyPublishViolatedWall(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 10, 10, 0.5)
	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, scalarMeasured, res.Wall.Outcome)
	require.NotNil(t, res.Wall.Minimum)
	require.True(t, res.Wall.Minimum.Value.Equal(units.Millimeters(0.5), 1e-9))
	require.Equal(t, assessmentViolated, res.Wall.Assessment)
}

// TestVerifyPublishExactZeroWall replays TestWallKnifeEdgeExactZero's pinch
// geometry: the chord meets the arc at a 10-degree tangent-chord angle, a
// wedge within the default allowance ground to a genuine Exact zero — Measured,
// never Absent, and Violated against every positive minimum (proposal §6,
// §16 "Exact zero wall").
func TestVerifyPublishExactZeroWall(t *testing.T) {
	t.Parallel()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	half := 20 * math.Sin(10*math.Pi/180)
	d := 20 * math.Cos(10*math.Pi/180)
	a := s.CreatePoint(-half, d)
	b := s.CreatePoint(half, d)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateArc(c, b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	doc := New()
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)

	for _, minimum := range []float64{1, 0.001} {
		res := publishBody(t, body, WithMinWallThickness(units.Millimeters(minimum)))
		require.Equal(t, scalarMeasured, res.Wall.Outcome)
		require.NotNil(t, res.Wall.Minimum)
		require.Equal(t, 0.0, res.Wall.Minimum.Value.Base())
		require.Equal(t, Exact, res.Wall.Minimum.Exactness)
		require.Equal(t, assessmentViolated, res.Wall.Assessment, "minimum=%v mm", minimum)
	}
}

// wedgePrismBody extrudes the equilateral wedge of TestWallWedgePrismNoWall:
// side pairs at 60 degrees, caps beyond any inscribed ball's reach.
func wedgePrismBody(t *testing.T) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 0}, {30, 0}, {15, 15 * math.Sqrt(3)}}
	pt := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		pt[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(pt[0])
	for i := range pt {
		s.CreateLine(pt[i], pt[(i+1)%len(pt)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	doc := New()
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(80), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestVerifyPublishAbsentWall replays TestWallWedgePrismNoWall: a supported
// analytic body with no spanning ball anywhere publishes the PROVEN absence,
// which meets every requested minimum (proposal §16 "Proven wall absence").
func TestVerifyPublishAbsentWall(t *testing.T) {
	t.Parallel()
	body := wedgePrismBody(t)
	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, scalarAbsent, res.Wall.Outcome)
	require.Nil(t, res.Wall.Minimum)
	require.Equal(t, assessmentMet, res.Wall.Assessment)
	require.Equal(t, Sound, res.Status)
}

// TestVerifyPublishAbsentConcaveRadius replays TestMinRadiusPlainBlockNil: an
// all-convex block has no concave feature, the proven best case for any
// endmill (proposal §16 "Proven radius absence").
func TestVerifyPublishAbsentConcaveRadius(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 60, 10)
	res := publishBody(t, body, WithMinRadius())
	require.Equal(t, scalarAbsent, res.ConcaveRadius.Outcome)
	require.Nil(t, res.ConcaveRadius.Minimum)
	require.Equal(t, Sound, res.Status)
}

// holePlateBody extrudes a 100x60x8 plate with a diameter-20 through hole at
// (70, 30), the same fixture survey_test.go's holePlate builds.
func holePlateBody(t *testing.T) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := New()
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestVerifyPublishMeasuredConcaveRadius replays TestMinRadiusHolePlate: the
// hole wall is the plate's one concave face, and the certified interval
// encloses its 10 mm radius.
func TestVerifyPublishMeasuredConcaveRadius(t *testing.T) {
	t.Parallel()
	body := holePlateBody(t)
	res := publishBody(t, body, WithMinRadius())
	require.Equal(t, scalarMeasured, res.ConcaveRadius.Outcome)
	require.NotNil(t, res.ConcaveRadius.Minimum)
	require.Equal(t, Exact, res.ConcaveRadius.Minimum.Exactness)
	lo := res.ConcaveRadius.Minimum.Value.Base() - res.ConcaveRadius.Minimum.Bound.Base()
	hi := res.ConcaveRadius.Minimum.Value.Base() + res.ConcaveRadius.Minimum.Bound.Base()
	require.LessOrEqual(t, lo, 10.0)
	require.GreaterOrEqual(t, hi, 10.0)
	require.Equal(t, Sound, res.Status)
}

// curvedWebPlateBody replays TestWallReadingBoundEnclosesCurvedWeb: the
// 100x60x20 mm plate with two r=1 holes whose web (3*sqrt(2) - 2) no float64
// holds exactly, so the reading is Approximate.
func curvedWebPlateBody(t *testing.T) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(48.5, 28.5), 1)
	s.CreateCircle(s.CreatePoint(51.5, 31.5), 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := New()
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestVerifyPublishThresholdStraddle picks a minimum strictly inside the
// certified [L, U] interval curvedWebPlateBody's Approximate wall reading
// publishes, and asserts the body stays Measured with an Undecided
// assessment rather than Met or Violated (proposal §6, §16 "Threshold
// straddle").
func TestVerifyPublishThresholdStraddle(t *testing.T) {
	t.Parallel()
	body := curvedWebPlateBody(t)
	probe := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, scalarMeasured, probe.Wall.Outcome)
	require.NotNil(t, probe.Wall.Minimum)
	require.Equal(t, Approximate, probe.Wall.Minimum.Exactness)
	require.Positive(t, probe.Wall.Minimum.Bound.Base())
	threshold := probe.Wall.Minimum.Value.Base()

	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(threshold)))
	require.Equal(t, scalarMeasured, res.Wall.Outcome)
	require.Equal(t, assessmentUndecided, res.Wall.Assessment)
}

// axisBoxBody extrudes an axis-aligned x0,y0 to x1,y1 rectangle into doc.
func axisBoxBody(t *testing.T, doc *Document, x0, y0, x1, y1, h float64) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// translatedBody moves b by (x, y, z).
func translatedBody(t *testing.T, b *Body, x, y, z float64) *Body {
	t.Helper()
	tr, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	moved, err := b.Placed(tr)
	require.NoError(t, err)
	return moved
}

// planarBooleanBody replays allPlanarBoolean: two overlapping axis-aligned
// cubes unioned through the mesh-boolean path, whose Area comes back
// Approximate (facetedPayload).
func planarBooleanBody(t *testing.T) *Body {
	t.Helper()
	doc := New()
	a := axisBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := translatedBody(t, axisBoxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	body, err := Union(a, b)
	require.NoError(t, err)
	return body
}

// TestVerifyPublishCoarseReadingKeepsGeometry replays
// TestVerifyAllPlanarBooleanGatesApproximateArea under a zero tolerance: the
// Area reading keeps its own computed geometry unchanged while its tolerance
// verdict flips to Exceeded with a computed Limit (proposal §16 "Coarse
// measured result").
func TestVerifyPublishCoarseReadingKeepsGeometry(t *testing.T) {
	t.Parallel()
	body := planarBooleanBody(t)
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())

	res := publishBody(t, body, WithTolerance(units.Scalar(0)))
	require.True(t, res.Area.Value.Equal(area.Value, 1e-9),
		"the published geometry is the one Area() computed, regardless of tolerance")
	require.Equal(t, area.Bound.Base(), res.Area.Bound.Base())
	require.Equal(t, toleranceExceeded, res.Area.Tolerance.State)
	require.NotNil(t, res.Area.Tolerance.Limit)
}

// TestVerifyPublishEffectiveRequest drives the real option resolver behind
// Document.Verify (defaults, a duplicate wall option where the last one
// wins, and the wall's own default draft allowance) and a real empty-document
// Verify call, asserting the recorded effective request (proposal §5, §16
// "Effective settings").
func TestVerifyPublishEffectiveRequest(t *testing.T) {
	t.Parallel()

	cfg, err := resolveVerifyOptions(nil)
	require.NoError(t, err)
	req := effectiveVerifyRequest(cfg)
	require.True(t, req.RelativeTolerance.Equal(units.Scalar(1e-3), 1e-12))
	require.Nil(t, req.Wall)
	require.Nil(t, req.Undercut)
	require.False(t, req.ConcaveRadius)
	require.False(t, req.Clearances)

	cfg, err = resolveVerifyOptions([]VerifyOption{
		WithMinWallThickness(units.Millimeters(1)),
		WithMinWallThickness(units.Millimeters(2), WithDraftAllowance(units.Degrees(10))),
	})
	require.NoError(t, err)
	req = effectiveVerifyRequest(cfg)
	require.NotNil(t, req.Wall)
	require.True(t, req.Wall.Minimum.Equal(units.Millimeters(2), 1e-9), "the later option wins")
	require.True(t, req.Wall.DraftAllowance.Equal(units.Radians(10*math.Pi/180), 1e-9))

	cfg, err = resolveVerifyOptions([]VerifyOption{WithMinWallThickness(units.Millimeters(1))})
	require.NoError(t, err)
	req = effectiveVerifyRequest(cfg)
	require.True(t, req.Wall.DraftAllowance.Equal(units.Radians(15*math.Pi/180), 1e-9),
		"WithDraftAllowance's own default is 15 degrees")

	pull := r3.NewVec(3, 0, 4)
	cfg, err = resolveVerifyOptions([]VerifyOption{WithPullDirection(pull)})
	require.NoError(t, err)
	req = effectiveVerifyRequest(cfg)
	require.NotNil(t, req.Undercut)
	require.Equal(t, pull, req.Undercut.PullDirection, "the exact accepted vector is kept, never normalized")
	require.InDelta(t, 5.0, req.Undercut.PullDirection.Len(), 1e-9)

	doc := New()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Sound, report.Status)
	emptyCfg, err := resolveVerifyOptions(nil)
	require.NoError(t, err)
	emptyReq := effectiveVerifyRequest(emptyCfg)
	require.True(t, emptyReq.RelativeTolerance.Equal(units.Scalar(1e-3), 1e-12),
		"the effective request is recorded even for an empty document")
}
