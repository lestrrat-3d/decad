package decad

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests verify_publish.go's assembler directly, in-package,
// because it drives verifyBody with a private verifyConfig — a fresh body
// (task-list §4 item 4's fixtures) that never reaches the public Document.
// Every other fixture below is a real body built through the public
// Document/Body API — no hand-assembled result stands in for a producer's
// own output.

// publishBody folds opts through the real option resolver and effective
// request, then runs verifyBody on body — the same path Document.Verify
// takes for one body — and returns the BodyReport it publishes.
func publishBody(t *testing.T, body *Body, opts ...VerifyOption) *BodyReport {
	t.Helper()
	cfg, err := resolveVerifyOptions(opts)
	require.NoError(t, err)
	req := effectiveVerifyRequest(cfg)
	res, err := verifyBody(t.Context(), body, cfg, req)
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
	require.Equal(t, ScalarNotRequested, res.Wall.Outcome)
	require.Nil(t, res.Wall.Request)
	require.Equal(t, AssessmentNotEvaluated, res.Wall.Assessment)
	require.Equal(t, ScalarNotRequested, res.ConcaveRadius.Outcome)
	require.Nil(t, res.ConcaveRadius.Minimum)

	// Both core readings are exact on this evaluator's plate, so their
	// zero bound passes without a computed Limit.
	require.Equal(t, ToleranceSatisfied, res.Area.Tolerance.State)
	require.Nil(t, res.Area.Tolerance.Limit)
	require.Equal(t, ToleranceSatisfied, res.Bounds.Tolerance.State)
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
	require.Equal(t, ScalarMeasured, res.Wall.Outcome)
	require.NotNil(t, res.Wall.Minimum)
	require.True(t, res.Wall.Minimum.Value.Equal(units.Millimeters(100), 1e-9))
	require.Equal(t, Exact, res.Wall.Minimum.Exactness)
	require.Equal(t, AssessmentMet, res.Wall.Assessment)
	require.NotNil(t, res.Wall.Request)
	require.True(t, res.Wall.Request.Minimum.Equal(units.Millimeters(1), 1e-9))

	equality := publishBody(t, body, WithMinWallThickness(units.Millimeters(100)))
	require.Equal(t, AssessmentMet, equality.Wall.Assessment, "L >= T including equality is Met")
}

// TestVerifyPublishViolatedWall replays TestWallThinPlateViolating's
// 10x10x0.5 mm plate: the proven [0.5, 0.5] mm interval sits below a 1 mm
// tool at any coarseness.
func TestVerifyPublishViolatedWall(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 10, 10, 0.5)
	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, ScalarMeasured, res.Wall.Outcome)
	require.NotNil(t, res.Wall.Minimum)
	require.True(t, res.Wall.Minimum.Value.Equal(units.Millimeters(0.5), 1e-9))
	require.Equal(t, AssessmentViolated, res.Wall.Assessment)
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
		require.Equal(t, ScalarMeasured, res.Wall.Outcome)
		require.NotNil(t, res.Wall.Minimum)
		require.Equal(t, 0.0, res.Wall.Minimum.Value.Base())
		require.Equal(t, Exact, res.Wall.Minimum.Exactness)
		require.Equal(t, AssessmentViolated, res.Wall.Assessment, "minimum=%v mm", minimum)
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
	require.Equal(t, ScalarAbsent, res.Wall.Outcome)
	require.Nil(t, res.Wall.Minimum)
	require.Equal(t, AssessmentMet, res.Wall.Assessment)
	require.Equal(t, Sound, res.Status)
}

// TestVerifyPublishAbsentConcaveRadius replays TestMinRadiusPlainBlockNil: an
// all-convex block has no concave feature, the proven best case for any
// endmill (proposal §16 "Proven radius absence").
func TestVerifyPublishAbsentConcaveRadius(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 60, 10)
	res := publishBody(t, body, WithConcaveRadius())
	require.Equal(t, ScalarAbsent, res.ConcaveRadius.Outcome)
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
	res := publishBody(t, body, WithConcaveRadius())
	require.Equal(t, ScalarMeasured, res.ConcaveRadius.Outcome)
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
	require.Equal(t, ScalarMeasured, probe.Wall.Outcome)
	require.NotNil(t, probe.Wall.Minimum)
	require.Equal(t, Approximate, probe.Wall.Minimum.Exactness)
	require.Positive(t, probe.Wall.Minimum.Bound.Base())
	threshold := probe.Wall.Minimum.Value.Base()

	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(threshold)))
	require.Equal(t, ScalarMeasured, res.Wall.Outcome)
	require.Equal(t, AssessmentUndecided, res.Wall.Assessment)
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
	moved, err := b.Placed(t.Context(), tr)
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
	require.Equal(t, ToleranceExceeded, res.Area.Tolerance.State)
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

// TestVerifyPublishUndercutCompleteAbsence replays TestUndercutsPrismClear
// and TestUndercutsVerticalHoleClear: every face is decided and none
// opposes, the proven all-clear (proposal §7, §16 "Complete undercut
// absence").
func TestVerifyPublishUndercutCompleteAbsence(t *testing.T) {
	t.Parallel()
	for _, body := range []*Body{rectangularPrism(t, 100, 60, 10), holePlateBody(t)} {
		res := publishBody(t, body, WithPullDirection(r3.NewVec(0, 0, 1)))
		require.Equal(t, CoverageComplete, res.Undercut.Coverage)
		require.Empty(t, res.Undercut.Faces)
		require.Equal(t, AssessmentMet, res.Undercut.Assessment)
		require.NotNil(t, res.Undercut.Request)
	}
}

// TestVerifyPublishUndercutCompleteViolation replays TestUndercutsTiltedPull:
// every face is decided and two confirmed faces oppose the tilted pull.
func TestVerifyPublishUndercutCompleteViolation(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 60, 10)
	res := publishBody(t, body, WithPullDirection(r3.NewVec(1, 0, 1)))
	require.Equal(t, CoverageComplete, res.Undercut.Coverage)
	require.Len(t, res.Undercut.Faces, 2)
	require.Equal(t, AssessmentViolated, res.Undercut.Assessment)
}

// chamferedQuarterDiskBody replays capblend_survey_test.go's
// chamferedQuarterDiskPatch: a quarter disk (0,0)->(r,0), a CCW arc to
// (0,r), and back to the origin, extruded by h and then chamfered by d on
// its end cap loop — the same real cap-blend construction reached through
// the public Body API alone, since chamferedQuarterDiskPatch itself lives in
// the external decad_test package and is unreachable from this in-package
// test.
func chamferedQuarterDiskBody(t *testing.T, r, h, d float64) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(0, r)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	doc := New()
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
	require.NoError(t, err)
	return chamfered
}

// TestVerifyPublishUndercutPartialCoverage replays
// TestCapBlendUndecidedPatchKeepsProvenUndercut, the real mixed cap-blend
// case: the circular chamfer patch's bounded range is undecided against a
// +x pull while the flat chamfer patch at the origin corner provenly
// opposes it. Coverage is Partial, carrying the confirmed face plus both the
// violation and the incomplete-survey diagnostic (proposal §7, §16 "Mixed
// undercuts").
func TestVerifyPublishUndercutPartialCoverage(t *testing.T) {
	t.Parallel()
	body := chamferedQuarterDiskBody(t, 100, 20, 0.5)
	var plane *Face
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if o.Role == "chamferCap(end,0,2)" {
				plane = f
			}
		}
	}
	require.NotNil(t, plane, "the origin-corner flat chamfer patch")
	require.Equal(t, KindPlane, plane.Surface().Kind())

	res := publishBody(t, body, WithPullDirection(r3.NewVec(1, 0, 0)))
	require.Equal(t, CoveragePartial, res.Undercut.Coverage)
	require.Contains(t, res.Undercut.Faces, plane)
	require.Equal(t, AssessmentViolated, res.Undercut.Assessment)

	var violating, undecided bool
	for _, d := range res.Undercut.Diagnostics {
		switch d.Code {
		case DiagUndercut:
			violating = true
		case DiagUndecidedUndercut:
			undecided = true
		}
	}
	require.True(t, violating, "the confirmed face's own DiagUndercut is carried")
	require.True(t, undecided, "the straddling patch's own DiagUndecidedUndercut is carried")
}

// freeformArchProfileBody extrudes a rectangle whose bottom edge is recorded
// as a degree-1 unit-weight NURBSSeg — the same free-form-recorded-kind
// refusal TestFreeformPrismUndercutsUndecided's freeformArchBody trips
// (survey.go's errFreeformSection), reached here through the public
// sketch/Body API alone since that fixture's own helpers live in the
// external decad_test package.
func freeformArchProfileBody(t *testing.T) *Body {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
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
	doc := New()
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestVerifyPublishUndercutUndecided replays
// TestFreeformPrismUndercutsUndecided: the free-form-recorded wall refuses
// the survey outright, so coverage is Undecided with an empty face list —
// never Complete, which would read as a proven absence the producer never
// certified (proposal §7, §16 "Undecided analytic survey").
func TestVerifyPublishUndercutUndecided(t *testing.T) {
	t.Parallel()
	body := freeformArchProfileBody(t)
	res := publishBody(t, body, WithPullDirection(r3.NewVec(0, 0, 1)))
	require.Equal(t, CoverageUndecided, res.Undercut.Coverage)
	require.Empty(t, res.Undercut.Faces)
	require.Equal(t, AssessmentUndecided, res.Undercut.Assessment)
}

// TestVerifyPublishUndercutUnavailable replays planarBooleanBody's
// facetedPayload: the pull survey has no implemented reader for a faceted
// body, so it is Unavailable rather than Undecided.
func TestVerifyPublishUndercutUnavailable(t *testing.T) {
	t.Parallel()
	body := planarBooleanBody(t)
	res := publishBody(t, body, WithPullDirection(r3.NewVec(0, 0, 1)))
	require.Equal(t, CoverageUnavailable, res.Undercut.Coverage)
	require.Empty(t, res.Undercut.Faces)
	require.Equal(t, AssessmentUndecided, res.Undercut.Assessment)
}

// capBlendPlateBody extrudes a 100×60 plate by 20 mm and chamfers its whole
// end-cap loop by 5 mm — the same cap-blend construction capblend_test.go's
// capBlendBox drives, reached here through the public Body API alone since
// that helper lives in the external decad_test package.
func capBlendPlateBody(t *testing.T) *Body {
	t.Helper()
	doc := New()
	body := axisBoxBody(t, doc, 0, 0, 100, 60, 20)
	chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(5))
	require.NoError(t, err)
	return chamfered
}

// TestVerifyPublishUndercutFacesOrderedByBody proves proposal §11's ordering
// guarantee: against a pull tilted off every face normal, the cap-blend
// chamfer's own producer confirms three faces out of its own build order, but
// publication must return them in the body's Faces() order regardless. The
// assertion is on computed order, not a pinned index triple, so it stays
// meaningful if the chamfer's build order ever changes.
func TestVerifyPublishUndercutFacesOrderedByBody(t *testing.T) {
	t.Parallel()
	body := capBlendPlateBody(t)
	res := publishBody(t, body, WithPullDirection(r3.NewVec(1, 0, 0.2)))
	require.Equal(t, CoverageComplete, res.Undercut.Coverage)
	require.Equal(t, AssessmentViolated, res.Undercut.Assessment)
	require.Len(t, res.Undercut.Faces, 3, "the chamfer's own producer confirms three opposing faces")

	indices := make(map[*Face]int, len(body.Faces()))
	for i, f := range body.Faces() {
		indices[f] = i
	}
	got := make([]int, len(res.Undercut.Faces))
	for i, f := range res.Undercut.Faces {
		idx, ok := indices[f]
		require.True(t, ok, "every confirmed face belongs to the body")
		got[i] = idx
	}
	require.True(t, sort.IntsAreSorted(got), "Faces come back in body.Faces() order, got indices %v", got)
}

// TestVerifyPublishCapBlendWallStaged proves DX9's deliberate cap-blend wall
// limit (survey.go, docs/modify-reach-design.md Table DX) is published as an
// explicit unsupported-payload dispatch rather than a generic undecided
// result (task-list §4 item 3): the real cap-blend body's wall survey
// reports Unavailable through DiagUnsupportedSurveyPayload, never
// DiagUndecidedWall — the same treatment a staged loft payload gets
// (TestLoftVerifySurveysStaySuspect).
func TestVerifyPublishCapBlendWallStaged(t *testing.T) {
	t.Parallel()
	body := capBlendPlateBody(t)
	res := publishBody(t, body, WithMinWallThickness(units.Millimeters(1)))
	require.Equal(t, ScalarUnavailable, res.Wall.Outcome)
	require.Nil(t, res.Wall.Minimum)
	require.Equal(t, AssessmentUndecided, res.Wall.Assessment)

	require.Len(t, res.Wall.Diagnostics, 1)
	diag := res.Wall.Diagnostics[0]
	require.Equal(t, DiagUnsupportedSurveyPayload, diag.Code)
	require.Equal(t, SurveyWall, diag.Survey)
	require.Equal(t, Suspect, diag.Status)
	require.Contains(t, diag.Message, "capBlendPayload")
	require.Contains(t, diag.Message, "wall survey")
}

// TestVerifyPublishToleranceReferenceUnavailable is proposal §16's "Missing
// reference" acceptance case: private-gate coverage of the tolerance verdict
// itself, not an end-to-end public path, since every shipped payload class
// forms a usable reference (requireDiagnosticInvariants, verify_test.go). A
// nil body gives bodyGateDiameter no usable diameter, so a nonzero-bound
// reading has no usable tolerance reference.
func TestVerifyPublishToleranceReferenceUnavailable(t *testing.T) {
	t.Parallel()
	in := &bodyToleranceInputs{ctx: t.Context(), body: nil, area: Measurement{Value: units.SquareMillimeters(100), Exactness: Exact}}
	m := Measurement{Value: units.Millimeters(5), Exactness: Approximate, Bound: units.Millimeters(0.001)}
	body := rectangularPrism(t, 10, 10, 10)

	tr, diag := scalarToleranceVerdict(ReadingWall, SurveyWall, body, m, 1e-3, in.lengthReference)
	require.Equal(t, ToleranceUndecided, tr.State)
	require.Nil(t, tr.Limit)
	require.NotNil(t, diag)
	require.Equal(t, DiagToleranceReferenceUnavailable, diag.Code)
	require.Equal(t, Suspect, diag.Status)
	require.Equal(t, SurveyWall, diag.Survey)
	require.Same(t, body, diag.Body)
	require.Nil(t, diag.Required)
}

// TestVerifyPublishDiagnosticFlattening drives planarBooleanBody's real
// facetedPayload union under a zero tolerance and a requested wall survey —
// a body that produces both a core reading diagnostic (Area's coarse
// measurement) and a survey diagnostic (the staged faceted wall refusal) at
// once — and asserts proposal §10's order: every core reading diagnostic
// (SurveyNone) precedes every wall diagnostic, and no finding repeats.
func TestVerifyPublishDiagnosticFlattening(t *testing.T) {
	t.Parallel()
	body := planarBooleanBody(t)
	res := publishBody(t, body, WithTolerance(units.Scalar(0)), WithMinWallThickness(units.Millimeters(1)))
	require.NotEmpty(t, res.Diagnostics)

	lastCoreIdx, firstWallIdx := -1, -1
	for i, d := range res.Diagnostics {
		switch d.Survey {
		case SurveyNone:
			lastCoreIdx = i
		case SurveyWall:
			if firstWallIdx == -1 {
				firstWallIdx = i
			}
		}
	}
	require.GreaterOrEqual(t, lastCoreIdx, 0, "the coarse Area reading emits a core diagnostic")
	require.GreaterOrEqual(t, firstWallIdx, 0, "the staged faceted wall refusal is present")
	require.Less(t, lastCoreIdx, firstWallIdx,
		"every core reading diagnostic precedes every wall diagnostic (proposal §10)")

	for i := range res.Diagnostics {
		for j := range res.Diagnostics {
			if i != j {
				require.NotEqual(t, res.Diagnostics[i], res.Diagnostics[j],
					"each finding appears exactly once in the flattened inventory (proposal §10)")
			}
		}
	}
}

// TestVerifyPublishValidBodyRegion replays the real 100x60x10 mm plate: a
// proven solid publishes both its Region readings and its held topology
// counts (proposal §9).
func TestVerifyPublishValidBodyRegion(t *testing.T) {
	t.Parallel()
	body := rectangularPrism(t, 100, 60, 10)
	res := publishBody(t, body)

	require.Equal(t, ValidityValid, res.Validity.Outcome)
	require.Empty(t, res.Validity.Diagnostics)
	require.NotNil(t, res.Region)
	require.True(t, res.Region.Volume.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	require.Equal(t, Exact, res.Region.Volume.Exactness)
	require.Equal(t, r3.NewVec(50, 30, 5), res.Region.Centroid.Value)
	require.Equal(t, 1, res.Topology.Lumps)
	require.Equal(t, 0, res.Topology.Voids)
}

// TestVerifyPublishPlateWithHoleTopology replays the through-hole plate
// behind TestVerifyPlateWithHole (verify_test.go): a through hole opens to
// the outside, so it walls off no cavity — one lump, no void (proposal §9).
func TestVerifyPublishPlateWithHoleTopology(t *testing.T) {
	t.Parallel()
	body := holePlateBody(t)
	res := publishBody(t, body)
	require.Equal(t, 1, res.Topology.Lumps)
	require.Equal(t, 0, res.Topology.Voids)
}

// TestVerifyPublishRevolveVoid replays TestRevolveFullTurnHoleIsVoidShell
// (revolve_test.go): a full-turn revolve of an annular rectangle with a
// circular hole closes the hole into a toroidal void shell — one void
// (proposal §9).
func TestVerifyPublishRevolveVoid(t *testing.T) {
	t.Parallel()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 10), 2)
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
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	body, err := doc.Revolve(s, prof, axis, FullRevolution{})
	require.NoError(t, err)

	res := publishBody(t, body)
	require.Equal(t, 1, res.Topology.Voids)
}

// requireSurveysBlockedByValidity asserts §9's blocking contract on a body
// whose validity is not ValidityValid: every requested survey publishes its
// Unavailable outcome plus exactly one local DiagSurveyPrerequisite naming
// its own survey, and the body's underlying validity diagnostic
// (validityCode) appears exactly once in the flattened inventory.
func requireSurveysBlockedByValidity(t *testing.T, res *BodyReport, validityCode DiagnosticCode) {
	t.Helper()
	require.Nil(t, res.Region)

	require.Equal(t, ScalarUnavailable, res.Wall.Outcome)
	require.Len(t, res.Wall.Diagnostics, 1)
	require.Equal(t, DiagSurveyPrerequisite, res.Wall.Diagnostics[0].Code)
	require.Equal(t, SurveyWall, res.Wall.Diagnostics[0].Survey)

	require.Equal(t, CoverageUnavailable, res.Undercut.Coverage)
	require.Len(t, res.Undercut.Diagnostics, 1)
	require.Equal(t, DiagSurveyPrerequisite, res.Undercut.Diagnostics[0].Code)
	require.Equal(t, SurveyUndercut, res.Undercut.Diagnostics[0].Survey)

	require.Equal(t, ScalarUnavailable, res.ConcaveRadius.Outcome)
	require.Len(t, res.ConcaveRadius.Diagnostics, 1)
	require.Equal(t, DiagSurveyPrerequisite, res.ConcaveRadius.Diagnostics[0].Code)
	require.Equal(t, SurveyConcaveRadius, res.ConcaveRadius.Diagnostics[0].Survey)

	count := 0
	for _, d := range res.Diagnostics {
		if d.Code == validityCode {
			count++
		}
	}
	require.Equal(t, 1, count, "the underlying validity diagnostic appears exactly once")
}

// TestVerifyPublishInvalidValidityBlocksSurveys is private-mapping coverage
// of publishBodyResult's validity gate, not an end-to-end public path:
// verifyBody's invalid branch is unreachable through a live document body
// (task-list §4 item 4), so this constructs &Body{} directly — no faces, so
// auditBoundary refuses it outright (proposal §16 "Validity
// invalid/undecided").
func TestVerifyPublishInvalidValidityBlocksSurveys(t *testing.T) {
	t.Parallel()
	body := &Body{}
	res := publishBody(t, body,
		WithMinWallThickness(units.Millimeters(1)),
		WithPullDirection(r3.NewVec(0, 0, 1)),
		WithConcaveRadius(),
	)

	require.Equal(t, Unsound, res.Status)
	require.Equal(t, ValidityInvalid, res.Validity.Outcome)
	require.Equal(t, ToleranceNotEvaluated, res.Area.Tolerance.State)
	require.Equal(t, ToleranceNotEvaluated, res.Bounds.Tolerance.State)
	requireSurveysBlockedByValidity(t, res, DiagInvalidBody)
}

// TestVerifyPublishUndecidedValidityBlocksSurveys is private-mapping
// coverage of publishBodyResult's validity gate, not an end-to-end public
// path: verifyBody's undecided branch is unreachable through a live
// document body (task-list §4 item 4), so this builds a second body over the
// same held topology and measurements without an evaluator payload, which
// keeps a clean boundary but no build proof (proposal §16 "Validity
// invalid/undecided").
func TestVerifyPublishUndecidedValidityBlocksSurveys(t *testing.T) {
	t.Parallel()
	built := rectangularPrism(t, 100, 60, 10)
	body := &Body{
		doc:      built.doc,
		origin:   built.origin,
		lumps:    built.lumps,
		volume:   built.volume,
		area:     built.area,
		centroid: built.centroid,
		bounds:   built.bounds,
		solid:    built.solid,
	}

	res := publishBody(t, body,
		WithMinWallThickness(units.Millimeters(1)),
		WithPullDirection(r3.NewVec(0, 0, 1)),
		WithConcaveRadius(),
	)

	require.Equal(t, Suspect, res.Status)
	require.Equal(t, ValidityUndecided, res.Validity.Outcome)
	requireSurveysBlockedByValidity(t, res, DiagUndecidedValidity)
}

// TestVerifyPublishUndercutRunsOnAProvenSheet drives publishUndercutResult
// directly on a BodySheet body at ValidityValid with a populated undercut
// outcome (docs/surface-design.md §2.3, §9.1): the outcome is published as
// given, not replaced by the DiagSurveyPrerequisite refusal
// requireSurveysBlockedByValidity pins for a non-valid body above — the
// widened kind gate is publishUndercutResult's own, so this drives it
// without the rest of verifyBody's plumbing.
func TestVerifyPublishUndercutRunsOnAProvenSheet(t *testing.T) {
	t.Parallel()
	body := &Body{kind: BodySheet}
	req := VerifyRequest{Undercut: &UndercutRequest{PullDirection: r3.NewVec(0, 0, 1)}}
	surveys := surveyResults{Undercut: undercutOutcome{ok: true, faces: []*Face{}}}

	res := publishUndercutResult(body, surveys, req, ValidityValid)

	require.Equal(t, CoverageComplete, res.Coverage)
	require.NotNil(t, res.Faces)
	require.Empty(t, res.Faces)
	require.Equal(t, AssessmentMet, res.Assessment)
	require.Empty(t, res.Diagnostics)
}
