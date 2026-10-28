package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The wall survey of a full Revolve whose meridian arc is centred exactly on
// the axis (agent-docs/verification-design.md §6): the arc and its mirror lie
// on one circle with the material on one side, so the pair generates no
// zero-width candidate and the survey reads the body's real wall.

// sphereMeridianSketch builds a solved closed profile through pts, every point
// fixed. Segment i runs from pts[i] to pts[i+1]; an entry in arcs makes it an
// arc about that fixed centre, counter-clockwise from pts[i] to pts[i+1]
// unless cw names i, which walks it clockwise.
func sphereMeridianSketch(t *testing.T, pts [][2]float64, arcs map[int][2]float64, cw map[int]struct{}) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		j := (i + 1) % len(sp)
		c, ok := arcs[i]
		if !ok {
			s.CreateLine(sp[i], sp[j])
			continue
		}
		cp := s.CreatePoint(c[0], c[1])
		s.Fix(cp)
		if _, rev := cw[i]; rev {
			s.CreateArc(cp, sp[j], sp[i])
			continue
		}
		s.CreateArc(cp, sp[i], sp[j])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ps := s.Profiles()
	require.Len(t, ps, 1)
	return s, ps[0]
}

// halfAnnulusSketch is a hollow sphere's meridian: the half annulus between
// radii ri and ro about the origin, its two flats on the u axis.
func halfAnnulusSketch(t *testing.T, ri, ro float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return sphereMeridianSketch(t, [][2]float64{{ro, 0}, {-ro, 0}, {-ri, 0}, {ri, 0}},
		map[int][2]float64{0: {0, 0}, 2: {0, 0}}, map[int]struct{}{2: {}})
}

// sphereWallReport revolves the profile a full turn about axis and verifies
// it against a tool of the given width.
func sphereWallReport(t *testing.T, s *sketch.Sketch, p *sketch.Profile, axis decad.SketchLine, tool float64) *decad.Report {
	t.Helper()
	doc := decad.New()
	_, err := decad.Revolve(t.Context(), doc, s, p, axis, decad.FullRevolution{})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(tool)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	return report
}

// requireWallTooThin asserts the report proves the wall below the tool.
func requireWallTooThin(t *testing.T, report *decad.Report) {
	t.Helper()
	require.Equal(t, decad.Violating, report.Status, "%+v", report.Diagnostics)
	_, thin := wallDiagnostic(report, decad.DiagWallTooThin)
	require.True(t, thin, "%+v", report.Diagnostics)
}

// requireWallUndecided asserts the survey refuses to decide.
func requireWallUndecided(t *testing.T, report *decad.Report) {
	t.Helper()
	br := report.Bodies[0]
	require.Equal(t, decad.ScalarUndecided, br.Wall.Outcome)
	require.Nil(t, br.Wall.Minimum)
	require.Equal(t, decad.Suspect, report.Status)
	count := 0
	for _, d := range report.Diagnostics {
		if d.Code == decad.DiagUndecidedWall {
			count++
		}
	}
	require.Equal(t, 1, count, "%+v", report.Diagnostics)
}

func TestWallHollowSphere(t *testing.T) {
	t.Parallel()
	// The shell between the r 8 and r 10 skins is r2 − r1 = 2 mm, a difference
	// of two recorded radii, halved and doubled: exact.
	s, p := halfAnnulusSketch(t, 8, 10)
	report := sphereWallReport(t, s, p, uAxis, 1.5)
	decadtest.MeasuresWallMinimum(t, report.Bodies[0], units.Millimeters(2), decadtest.Exactly())
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)

	report = sphereWallReport(t, s, p, uAxis, 2.5)
	decadtest.MeasuresWallMinimum(t, report.Bodies[0], units.Millimeters(2), decadtest.Exactly())
	requireWallTooThin(t, report)
}

func TestWallThinSphereShell(t *testing.T) {
	t.Parallel()
	s, p := halfAnnulusSketch(t, 9.5, 10)
	report := sphereWallReport(t, s, p, uAxis, 1)
	decadtest.MeasuresWallMinimum(t, report.Bodies[0], units.Millimeters(0.5), decadtest.Exactly())
	requireWallTooThin(t, report)
}

func TestWallSolidBallReadsDiameter(t *testing.T) {
	t.Parallel()
	// A ball of radius 10: its thinnest spanning ball is itself, 2r = 20 mm.
	// The reading is an interval, because the pole vertex's candidate rounds.
	s, p := sphereMeridianSketch(t, [][2]float64{{10, 0}, {-10, 0}}, map[int][2]float64{0: {0, 0}}, nil)
	report := sphereWallReport(t, s, p, uAxis, 1.5)
	br := report.Bodies[0]
	require.Equal(t, decad.ScalarMeasured, br.Wall.Outcome, "%+v", report.Diagnostics)
	require.NotNil(t, br.Wall.Minimum)
	value, err := br.Wall.Minimum.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.Wall.Minimum.Bound.In(units.Millimeter)
	require.NoError(t, err)
	requireContains(t, big.NewRat(20, 1), value, bound)
	require.LessOrEqual(t, bound, 1e-9, "a cap on the interval, not a pin")
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)

	report = sphereWallReport(t, s, p, uAxis, 25)
	requireWallTooThin(t, report)
}

func TestWallHemisphereReadsAngleLimit(t *testing.T) {
	t.Parallel()
	// A solid hemisphere of radius 10. Its thinnest spanning ball touches the
	// flat and the sphere at the 15° allowance limit: centre c = (r, y) with
	// |c| = 10 − r and c at 15° off the axis, so r = (10 − r)·cos15° and the
	// diameter is 20·cos15°/(1 + cos15°), cos15° = (√6 + √2)/4.
	s, p := sphereMeridianSketch(t, [][2]float64{{0, 0}, {10, 0}, {0, 10}}, map[int][2]float64{1: {0, 0}}, nil)
	report := sphereWallReport(t, s, p, uAxis, 9)
	br := report.Bodies[0]
	require.Equal(t, decad.ScalarMeasured, br.Wall.Outcome, "%+v", report.Diagnostics)
	require.NotNil(t, br.Wall.Minimum)
	value, err := br.Wall.Minimum.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.Wall.Minimum.Bound.In(units.Millimeter)
	require.NoError(t, err)

	const prec = 256
	f := func(v float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(v) }
	cos15 := new(big.Float).SetPrec(prec).Add(
		new(big.Float).SetPrec(prec).Sqrt(f(6)),
		new(big.Float).SetPrec(prec).Sqrt(f(2)))
	cos15.Quo(cos15, f(4))
	truth := new(big.Float).SetPrec(prec).Quo(
		new(big.Float).SetPrec(prec).Mul(f(20), cos15),
		new(big.Float).SetPrec(prec).Add(f(1), cos15))
	lo := new(big.Float).SetPrec(prec).Sub(f(value), f(bound))
	hi := new(big.Float).SetPrec(prec).Add(f(value), f(bound))
	require.LessOrEqual(t, lo.Cmp(truth), 0, "the published interval must reach down to the truth")
	require.GreaterOrEqual(t, hi.Cmp(truth), 0, "the published interval must reach up to the truth")
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)

	report = sphereWallReport(t, s, p, uAxis, 10)
	requireWallTooThin(t, report)
}

func TestWallSphericalZoneReadsFlats(t *testing.T) {
	t.Parallel()
	// A ball of radius 10 with both poles cut flat at u = ±6: the flats are
	// 12 mm apart, and the r 6 ball between them clears the r 10 sphere.
	s, p := sphereMeridianSketch(t, [][2]float64{{-6, 0}, {6, 0}, {6, 8}, {-6, 8}}, map[int][2]float64{2: {0, 0}}, nil)
	report := sphereWallReport(t, s, p, uAxis, 1.5)
	decadtest.MeasuresWallMinimum(t, report.Bodies[0], units.Millimeters(12), decadtest.Exactly())
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)
}

func TestWallSphereCentreOffAxisUndecided(t *testing.T) {
	t.Parallel()
	// The arc's recorded centre sits 1e-13 off the axis: the profile's flat
	// still reads as on the axis, but the centre is not proven on it, so the
	// arc and its mirror are two circles and the survey stays undecided.
	s, p := circleSketch(t, 5, 1e-13, 5)
	requireWallUndecided(t, sphereWallReport(t, s, p, uAxis, 1.5))
}

func TestWallSphereTiltedAxisUndecided(t *testing.T) {
	t.Parallel()
	// About the axis (0,0)→(1,1) the centre (5,5) re-expresses with a zero
	// radial coordinate but a nonzero bound on it: on the axis only within
	// rounding, so the survey stays undecided.
	s, p := sphereMeridianSketch(t, [][2]float64{{10, 10}, {0, 0}}, map[int][2]float64{0: {5, 5}}, nil)
	tilted := decad.SketchLine{End: decad.Point2{U: 1, V: 1}}
	requireWallUndecided(t, sphereWallReport(t, s, p, tilted, 1.5))
}
