package decad

import (
	"math"
	"os"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md's PR 1 (docs/loft-design.md's chord-target calibration,
// Part 2 Q2/Q3, Part 3 PR 1): it MEASURES, rather than assumes, where the future
// unexported constant loftChordFraction should land. That constant does not exist
// yet — no production code changes here — so this file builds the fixture the plan
// names for measuring it anyway: a hand-chorded LineSeg-only profile builds through
// today's public Loft with no code change and publishes Exact readings and a zero
// Bound (a straight polygon has no curve to fall short of). Only sectionDelta itself
// — the per-cell chord-vs-curve displacement the future evaluator will publish — is
// absent, and this file computes it (closed form for the circular arm, a dense
// numeric measurement for the free-form arm, since a fit spline has no closed-form
// sagitta) and adds it arithmetically to each reading's Bound before re-running
// verify.go's own tolerance gate (scalarToleranceRef/boundedToleranceRef) on the
// widened value by hand.

// --- shared wedge geometry ---

const (
	wedgeRadius = 5.0         // A10a/A10b's quarter-arc radius
	wedgeSweep  = math.Pi / 2 // the quarter-arc sweep
	wedgeHeight = 10.0        // the loft's Z extrusion distance

	// toleranceRel is verify.go's WithTolerance default (verify.go:390-394): the
	// relative tolerance every widened bound below is compared against.
	toleranceRel = 1e-3
)

// wedgeCirclePoints returns the m+1 chord vertices for the A10a arm: exact points on
// the radius-5 quarter circle from (5,0) to (0,5), k = 0..m at k*sweep/m.
func wedgeCirclePoints(m int) [][2]float64 {
	pts := make([][2]float64, m+1)
	for k := 0; k <= m; k++ {
		theta := wedgeSweep * float64(k) / float64(m)
		pts[k] = [2]float64{wedgeRadius * math.Cos(theta), wedgeRadius * math.Sin(theta)}
	}
	return pts
}

// wedgeFitSpline builds the A10b reference curve once: a 5-point fit spline through
// the same quarter circle, at k*pi/8 for k = 0..4 (angles 0, pi/8, pi/4, 3pi/8, pi/2).
// Every A10b fixture below samples THIS curve's own Eval, never the circle, since the
// task requires the hand-chorded stand-in to chord the spline's own curve.
func wedgeFitSpline(t *testing.T) *sketch.FitSpline {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := make([]*sketch.Point, 5)
	for k := range pts {
		theta := float64(k) * math.Pi / 8
		p := s.CreatePoint(wedgeRadius*math.Cos(theta), wedgeRadius*math.Sin(theta))
		s.Fix(p)
		pts[k] = p
	}
	fs, err := s.CreateFitSpline(pts...)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return fs
}

// wedgeSplinePoints returns the m+1 chord vertices for the A10b arm: points ON the
// fit spline's own curve (fs.Eval, normalized t in [0,1]) — never points on the
// circle, since the fit spline only touches the circle at its 5 defining points and
// departs from it in between.
func wedgeSplinePoints(fs *sketch.FitSpline, m int) [][2]float64 {
	pts := make([][2]float64, m+1)
	for k := 0; k <= m; k++ {
		u, v := fs.Eval(float64(k) / float64(m))
		pts[k] = [2]float64{u, v}
	}
	return pts
}

// wedgePlanes builds the two parallel planes both wedge fixtures loft between: z=0
// and its CreateOffsetPlane at wedgeHeight, sharing one U/V basis so a chord vertex
// at the same (u,v) on each plane is the natural (offset-0) correspondence
// loft_test.go's loftSquaresAt idiom also relies on.
func wedgePlanes(t *testing.T) (*sketch.World, *sketch.Plane, *sketch.Plane) {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := w.CreateOffsetPlane(base, wedgeHeight)
	require.NoError(t, err)
	return w, base, top
}

// chordedWedgeProfile is task 1's helper: it builds one hand-chorded LineSeg-only
// wedge outline on plane — origin -> pts[0] -> ... -> pts[last] -> origin — the only
// way to see the future chorded evaluator's published measurements before that
// evaluator exists.
func chordedWedgeProfile(t *testing.T, w *sketch.World, plane *sketch.Plane, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	prev := origin
	for _, p := range pts {
		next := s.CreatePoint(p[0], p[1])
		s.CreateLine(prev, next)
		prev = next
	}
	s.CreateLine(prev, origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1, "the chorded wedge outline is one closed loop")
	return s, profiles[0]
}

// buildChordedWedgeLoft builds both planes' chorded profiles from the same 2D vertex
// list and lofts them, returning the built body and the wall-clock the Loft call
// itself took (the quantity Q3's O(F^2) audit cost is measured against).
func buildChordedWedgeLoft(t *testing.T, pts [][2]float64) (*Body, time.Duration) {
	t.Helper()
	w, base, top := wedgePlanes(t)
	s0, p0 := chordedWedgeProfile(t, w, base, pts)
	s1, p1 := chordedWedgeProfile(t, w, top, pts)
	doc := New()
	start := time.Now()
	body, err := doc.Loft(s0, p0, s1, p1)
	elapsed := time.Since(start)
	require.NoError(t, err)
	return body, elapsed
}

// --- the TRUE (uncorded) wedge outlines, used only to read the chordTarget
// envelope through the real profileCoordinateUpper/profileCoordinateEnvelope helper
// (Q2), never to build a measured loft ---

// wedgeArcSketch builds the true A10a outline: two lines plus one radius-5, 90-degree
// arc.
func wedgeArcSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	px := s.CreatePoint(wedgeRadius, 0)
	py := s.CreatePoint(0, wedgeRadius)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// wedgeSplineSketch builds the true A10b outline: the same 5-point fit spline
// wedgeFitSpline samples, closed by a chord from its last point back to its first.
func wedgeSplineSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, 5)
	for k := range pts {
		theta := float64(k) * math.Pi / 8
		p := s.CreatePoint(wedgeRadius*math.Cos(theta), wedgeRadius*math.Sin(theta))
		s.Fix(p)
		pts[k] = p
	}
	_, err = s.CreateFitSpline(pts...)
	require.NoError(t, err)
	s.CreateLine(pts[len(pts)-1], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// wedgeArcEnvelope reads Q2's chordTarget envelope — max(profileCoordinateUpper(p0),
// profileCoordinateUpper(p1)) — off the TRUE (arc) wedge record on both planes. The
// arc walk is analytic, so profileCoordinateUpper (extrude.go:452) answers directly.
func wedgeArcEnvelope(t *testing.T) float64 {
	t.Helper()
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	rec0, _, err := RecordProfile(s0, p0)
	require.NoError(t, err)
	rec1, _, err := RecordProfile(s1, p1)
	require.NoError(t, err)
	work := newFreeformWork()
	u0, err := profileCoordinateUpper(rec0, work)
	require.NoError(t, err)
	u1, err := profileCoordinateUpper(rec1, work)
	require.NoError(t, err)
	return math.Max(u0, u1)
}

// wedgeSplineEnvelope is wedgeArcEnvelope's A10b twin. profileCoordinateUpper
// REFUSES a free-form walk outright (requireAnalyticWalk, extrude.go) — it exists for
// callers that need a placed cap frame, which a free-form wall genuinely cannot
// represent. This fixture's curved side is a FitSplineSeg, so it has no placed cap
// frame to ask for; profileCoordinateEnvelope is the twin extrude.go's own doc
// comment names for exactly this case — a caller that needs only a coordinate
// MAGNITUDE envelope reads it directly rather than refusing.
func wedgeSplineEnvelope(t *testing.T) float64 {
	t.Helper()
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeSplineSketch(t, w, base)
	s1, p1 := wedgeSplineSketch(t, w, top)
	rec0, _, err := RecordProfile(s0, p0)
	require.NoError(t, err)
	rec1, _, err := RecordProfile(s1, p1)
	require.NoError(t, err)
	work := newFreeformWork()
	u0, err := profileCoordinateEnvelope(rec0, work)
	require.NoError(t, err)
	u1, err := profileCoordinateEnvelope(rec1, work)
	require.NoError(t, err)
	return math.Max(u0, u1)
}

// --- sectionDelta and the wall Area excess, per arm ---

// arcSagitta is tessellate.go's chordCount sagitta (tessellate.go:807-810) in closed
// form, evaluated at a FORCED station count m rather than walked up to a tolerance:
// 2r*sin^2(sweep/m/4), the max per-cell displacement between an m-chord polygon and
// the true radius-r arc it approximates.
func arcSagitta(r, sweep float64, m int) float64 {
	s := math.Sin(sweep / float64(m) / 4)
	return 2 * r * s * s
}

// arcChordExcess is the wall Area term: how much longer the true arc is than its
// m-chord polygon, times the extrusion height.
func arcChordExcess(r, sweep float64, m int, height float64) float64 {
	arcLen := r * sweep
	chordLen := float64(m) * 2 * r * math.Sin(sweep/(2*float64(m)))
	return (arcLen - chordLen) * height
}

// splineCellSagitta measures the maximum perpendicular distance from the chord
// [fs.Eval(tA), fs.Eval(tB)] to the curve fs.Eval(t) for t strictly between, by dense
// sampling. This is a CALIBRATION PROXY for the future evaluator's own per-cell
// sagitta bound, not a certified one — the real bound will come from measure-then-
// bisect over the curve's own control polygon (docs/spline-design.md Sec 6.1), which
// this harness does not reimplement. Dense sampling is enough for calibration: the
// question here is where the constant lands, not whether the shipped bound is sound.
func splineCellSagitta(fs *sketch.FitSpline, tA, tB float64, samples int) float64 {
	ax, ay := fs.Eval(tA)
	bx, by := fs.Eval(tB)
	dx, dy := bx-ax, by-ay
	length := math.Hypot(dx, dy)
	worst := 0.0
	for i := 1; i < samples; i++ {
		tt := tA + (tB-tA)*float64(i)/float64(samples)
		px, py := fs.Eval(tt)
		var d float64
		if length == 0 {
			d = math.Hypot(px-ax, py-ay)
		} else {
			d = math.Abs(dx*(ay-py)-dy*(ax-px)) / length
		}
		if d > worst {
			worst = d
		}
	}
	return worst
}

// splineSagitta is splineCellSagitta maximized over the m chords wedgeSplinePoints(fs,
// m) draws — the free-form arm's sectionDelta at station count m.
func splineSagitta(fs *sketch.FitSpline, m, samplesPerCell int) float64 {
	worst := 0.0
	for k := range m {
		if s := splineCellSagitta(fs, float64(k)/float64(m), float64(k+1)/float64(m), samplesPerCell); s > worst {
			worst = s
		}
	}
	return worst
}

// splineDenseLength sums n tiny chords along fs's own curve as a close numeric
// estimate of its true arc length — the fit spline (unlike the arc) has no
// closed-form length, so every "true length" this file needs is this dense sample.
func splineDenseLength(fs *sketch.FitSpline, n int) float64 {
	total := 0.0
	px, py := fs.Eval(0)
	for i := 1; i <= n; i++ {
		qx, qy := fs.Eval(float64(i) / float64(n))
		total += math.Hypot(qx-px, qy-py)
		px, py = qx, qy
	}
	return total
}

// splineChordExcess is arcChordExcess's free-form twin: the densely measured true
// curve length minus the m-chord polygon's own length, times the extrusion height.
func splineChordExcess(fs *sketch.FitSpline, m int, height float64, denseN int) float64 {
	trueLen := splineDenseLength(fs, denseN)
	pts := wedgeSplinePoints(fs, m)
	chordLen := 0.0
	for i := 1; i < len(pts); i++ {
		chordLen += math.Hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
	}
	return (trueLen - chordLen) * height
}

// wedgeShoelaceRegion computes a closed polygon's area and centroid by the standard
// shoelace sums over vertices in order (the last implicitly reconnects to the
// first). It is used only to build the A10b "dense-sample reference" the pinned
// acceptance test compares against — decad's own Area/Centroid always come from the
// actual built body, never from this helper.
func wedgeShoelaceRegion(verts [][2]float64) (area, cx, cy float64) {
	var a, mx, my float64
	n := len(verts)
	for i := range n {
		x0, y0 := verts[i][0], verts[i][1]
		x1, y1 := verts[(i+1)%n][0], verts[(i+1)%n][1]
		cross := x0*y1 - x1*y0
		a += cross
		mx += (x0 + x1) * cross
		my += (y0 + y1) * cross
	}
	a /= 2
	mx /= 6 * a
	my /= 6 * a
	return a, mx, my
}

// --- the gate reproduction: verify.go's own scalarToleranceRef/boundedToleranceRef,
// run on the WIDENED bound each reading would carry once sectionDelta exists ---

// widenedGateRow is one reading's widened-bound gate comparison: widened is the
// production Bound plus this file's own sectionDelta term, ref is the
// diameter-anchored reference verify.go's own gate built, ratio = widened/ref is
// what the default 1e-3 tolerance compares against 1, and sound reports whether that
// comparison passes.
type widenedGateRow struct {
	reading string
	widened float64
	ref     float64
	ratio   float64
	sound   bool
}

// wedgeMeasurement is one m's full row: the closed-form/measured sectionDelta this
// file added to every Bound, the built body's own face count and wall-clock, and the
// four widened-bound gate rows.
type wedgeMeasurement struct {
	m            int
	sectionDelta float64
	f            int
	elapsed      time.Duration
	body         *Body // the already-built loft, reused so callers never rebuild it

	volume, area, bounds, centroid widenedGateRow
}

// binding returns the reading with the largest ratio — the one that decides whether
// this m reads Sound at the default tolerance, per the plan's "the BINDING reading is
// the one with the largest ratio".
func (m wedgeMeasurement) binding() widenedGateRow {
	worst := m.volume
	for _, r := range []widenedGateRow{m.area, m.bounds, m.centroid} {
		if r.ratio > worst.ratio {
			worst = r
		}
	}
	return worst
}

// measureWedgeReadings builds the chorded loft over pts, then widens each of the
// four readings' Bound by the term Part 2 Q2 states for it and re-runs verify.go's
// own tolerance gate on the widened value:
//   - Volume:   + sectionDelta * areaUpper                  (chordedBoundaryVolumeAllow)
//   - Area:     + areaExcess                                (ruled-vs-chord wall excess)
//   - Bounds:   + sectionDelta
//   - Centroid: + sectionDelta*(diameter/2+|centroid|)*areaUpper/volume (a calibration
//     ESTIMATE of chordedBoundaryMomentAllow's quotient-rule composition, not the
//     shipped bound — stated in the plan prompt as such)
//
// areaUpper is the body's own Area().Value widened by its own Area().Bound: the
// simplest sound-shaped ceiling on "surface area along the chord-to-curve path"
// available without a per-face area survey this calibration harness has no need to
// build.
func measureWedgeReadings(t *testing.T, pts [][2]float64, sectionDelta, areaExcess float64) wedgeMeasurement {
	t.Helper()
	body, elapsed := buildChordedWedgeLoft(t, pts)

	area, err := body.Area()
	require.NoError(t, err)
	vol, err := body.Volume()
	require.NoError(t, err)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	bounds, err := body.Bounds()
	require.NoError(t, err)

	br := &BodyReport{Body: body, Area: area}
	in := &bodyToleranceInputs{ctx: t.Context(), report: br}

	areaUpper := math.Abs(area.Value.Base()) + area.Bound.Base()

	widenedVol := Measurement{Value: vol.Value, Exactness: vol.Exactness, Bound: units.CubicMillimeters(vol.Bound.Base() + sectionDelta*areaUpper)}
	volPass, volRef, volHaveRef := scalarToleranceRef(widenedVol, toleranceRel, in.volumeReference)

	widenedArea := Measurement{Value: area.Value, Exactness: area.Exactness, Bound: units.SquareMillimeters(area.Bound.Base() + areaExcess)}
	areaPass, areaRef, areaHaveRef := scalarToleranceRef(widenedArea, toleranceRel, in.areaReference)

	widenedBoundsBound := bounds.Bound.Base() + sectionDelta
	boundsPass, boundsRef, boundsHaveRef := boundedToleranceRef(widenedBoundsBound, toleranceRel, in.diameterReference)

	diameter, diamOK, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, diamOK, "the chorded wedge prism must read a gate diameter")

	centroidMag := math.Sqrt(centroid.Value.X*centroid.Value.X + centroid.Value.Y*centroid.Value.Y + centroid.Value.Z*centroid.Value.Z)
	volValue := math.Abs(vol.Value.Base())
	centroidTerm := sectionDelta * (diameter/2 + centroidMag) * areaUpper / volValue
	widenedCentroidBound := centroid.Bound.Base() + centroidTerm
	centroidPass, centroidRef, centroidHaveRef := boundedToleranceRef(widenedCentroidBound, toleranceRel, in.diameterReference)

	row := func(name string, widened, ref float64, haveRef, pass bool) widenedGateRow {
		r := widenedGateRow{reading: name, widened: widened, ref: ref, sound: pass}
		if haveRef && ref != 0 {
			r.ratio = widened / ref
		}
		return r
	}

	return wedgeMeasurement{
		m:            len(pts) - 1,
		sectionDelta: sectionDelta,
		f:            len(body.Faces()),
		elapsed:      elapsed,
		body:         body,
		volume:       row("Volume", widenedVol.Bound.Base(), volRef, volHaveRef, volPass),
		area:         row("Area", widenedArea.Bound.Base(), areaRef, areaHaveRef, areaPass),
		bounds:       row("Bounds", widenedBoundsBound, boundsRef, boundsHaveRef, boundsPass),
		centroid:     row("Centroid", widenedCentroidBound, centroidRef, centroidHaveRef, centroidPass),
	}
}

func logWedgeMeasurement(t *testing.T, label string, m wedgeMeasurement) {
	t.Helper()
	binding := m.binding()
	margin := math.Inf(1)
	if binding.ratio > 0 {
		margin = toleranceRel / binding.ratio
	}
	t.Logf(
		"%-14s m=%-4d sectionDelta=%.6g F=%-4d elapsed=%-12s vol[w=%.4e ref=%.4e ratio=%.4e] area[w=%.4e ref=%.4e ratio=%.4e] bounds[w=%.4e ref=%.4e ratio=%.4e] centroid[w=%.4e ref=%.4e ratio=%.4e] binding=%s margin=%.3gx",
		label, m.m, m.sectionDelta, m.f, m.elapsed,
		m.volume.widened, m.volume.ref, m.volume.ratio,
		m.area.widened, m.area.ref, m.area.ratio,
		m.bounds.widened, m.bounds.ref, m.bounds.ratio,
		m.centroid.widened, m.centroid.ref, m.centroid.ratio,
		binding.reading, margin,
	)
}

// --- the sweep: TestLoftChordCalibrationSweep, opt-in only ---

// decadLoftCalibrationEnv is the explicit opt-in TestLoftChordCalibrationSweep
// requires. testing.Short() alone does not gate anything in the package's default
// `go test ./...` run — that flag is false unless a caller passes -short — so the
// sweep needs its own default-off switch to keep Q3's "at most three [2s] fixtures"
// budget out of the normal suite. Set it to any non-empty value to run the sweep.
const decadLoftCalibrationEnv = "DECAD_LOFT_CALIBRATION"

// TestLoftChordCalibrationSweep is a10-plan.md PR 1's calibration procedure (Part 2
// Q2, Part 3 PR 1 tasks 2-3): both wedges, hand-chorded at m = 4, 8, 16, 32, 64, 128,
// each row logging the four widened-bound gate readings, F, and wall-clock. It is a
// one-time measurement harness, not a regression fixture, and it is expensive (m=128
// drives F past 500, and loftCrossingAudit is O(F^2) — Q3: ~13s and 12 loft builds
// measured), so it costs the default `go test ./...` run nothing: it skips unless
// DECAD_LOFT_CALIBRATION is set (and still honors -short on top of that).
// TestLoftChordCalibrationPinsFraction below is the fast, always-run pin.
func TestLoftChordCalibrationSweep(t *testing.T) {
	if testing.Short() {
		t.Skip("the calibration sweep builds loft fixtures up to m=128 stations (F>500, O(F^2) audit); run without -short to measure it")
	}
	if os.Getenv(decadLoftCalibrationEnv) == "" {
		t.Skipf("the calibration sweep is a one-time measurement harness, not a regression fixture; set %s=1 to re-run it", decadLoftCalibrationEnv)
	}

	ms := []int{4, 8, 16, 32, 64, 128}
	fs := wedgeFitSpline(t)
	const splineSamplesPerCell = 200
	const splineDenseN = 20000

	arcEnvelope := wedgeArcEnvelope(t)
	splineEnvelope := wedgeSplineEnvelope(t)
	t.Logf("A10a envelope (profileCoordinateUpper) = %.10g mm; A10b envelope (profileCoordinateEnvelope) = %.10g mm", arcEnvelope, splineEnvelope)

	for _, m := range ms {
		pts := wedgeCirclePoints(m)
		sd := arcSagitta(wedgeRadius, wedgeSweep, m)
		ae := arcChordExcess(wedgeRadius, wedgeSweep, m, wedgeHeight)
		meas := measureWedgeReadings(t, pts, sd, ae)
		logWedgeMeasurement(t, "A10a(arc)", meas)
		t.Logf("  A10a m=%d implied loftChordFraction = sagitta/envelope = %.6g", m, sd/arcEnvelope)
	}

	for _, m := range ms {
		pts := wedgeSplinePoints(fs, m)
		sd := splineSagitta(fs, m, splineSamplesPerCell)
		ae := splineChordExcess(fs, m, wedgeHeight, splineDenseN)
		meas := measureWedgeReadings(t, pts, sd, ae)
		logWedgeMeasurement(t, "A10b(spline)", meas)
		t.Logf("  A10b m=%d implied loftChordFraction = sagitta/envelope = %.6g", m, sd/splineEnvelope)
	}
}

// --- the pin: fast, always-run ---

// loftChordFractionPinM is this PR's measured answer for the future
// loftChordFraction package constant (a10-plan.md Part 2 Q2's
// "chordTarget = loftChordFraction * envelope" rule), chosen from
// TestLoftChordCalibrationSweep's table over the mandated grid m = 4, 8, 16, 32,
// 64, 128. On that grid the 4x-margin requirement and the 2s wall-clock budget do
// NOT hold simultaneously (a10-plan.md's risk R2, confirmed by measurement rather
// than assumed):
//
//   - Volume is the binding reading throughout (not Centroid — this file's
//     deliberately generous areaUpper choice, the body's own total Area rather
//     than just the curved wall, makes the volume term dominate).
//   - The coarsest grid m at which BOTH fixtures clear 4x margin is m=128
//     (arc ratio=1.04e-4, margin=9.58x; spline ratio=1.32e-4, margin=7.55x) —
//     but F=262 there and the measured build is ~4.3s, more than double the 2s
//     budget.
//   - The finest grid m that still fits the 2s budget is m=64 (F=134,
//     ~1.1s measured for both fixtures) — Sound (ratio < 1e-3 for both) but only
//     ~2.4x (arc) / ~1.9x (spline) margin, short of 4x.
//
// Per the plan's named fallback (Q2, "Fallback if calibration does not close"),
// this pins m=64: Q3's 2s wall-clock ceiling is stated as a hard "any fixture that
// ships in go test ./... builds in 2 seconds or less", while the 4x margin is a
// target on top of Sound, not a second hard gate. A loft at this radius/aspect-ratio
// combination can therefore still read Suspect at a tighter-than-default tolerance;
// that is the plan's accepted, non-silent outcome, not a bug.
const loftChordFractionPinM = 64

// TestLoftChordCalibrationPinsFraction is the fast, always-run pin PR 1's acceptance
// line requires: it re-measures loftChordFractionPinM's fixtures (not the whole
// sweep) and asserts the closed-form enclosure and the achieved margin as numbers,
// never merely a Sound/Suspect verdict, and that both fixtures build within the 2s
// budget.
func TestLoftChordCalibrationPinsFraction(t *testing.T) {
	const wantVolume = math.Pi * 25 / 4 * wedgeHeight // 196.349540849...

	arcStart := time.Now()
	arcPts := wedgeCirclePoints(loftChordFractionPinM)
	arcSD := arcSagitta(wedgeRadius, wedgeSweep, loftChordFractionPinM)
	arcAE := arcChordExcess(wedgeRadius, wedgeSweep, loftChordFractionPinM, wedgeHeight)
	arcMeas := measureWedgeReadings(t, arcPts, arcSD, arcAE)
	arcElapsed := time.Since(arcStart)
	require.Less(t, arcElapsed, 2*time.Second, "PR 1's acceptance line: the arc wedge must build in under 2s")

	// Reuse the body measureWedgeReadings already built above rather than lofting
	// the same arc wedge a second time — Q3's "at most three [2s] fixtures" budget
	// counts loft builds, not assertions, so this pin stays at two builds total.
	vol, err := arcMeas.body.Volume()
	require.NoError(t, err)
	// The plan's acceptance line: |Volume.Value - (pi*25/4)*10| <= Volume.Bound +
	// sectionDeltaTerm. Volume.Base() is already in cubic millimetres (units'
	// documented base unit for Volume), so no conversion is needed before comparing
	// against the closed-form wantVolume mm^3.
	gotVolume := vol.Value.Base()
	sectionDeltaVolumeTerm := arcMeas.volume.widened - vol.Bound.Base() // isolate the added term for the assert message
	require.LessOrEqual(t, math.Abs(gotVolume-wantVolume), arcMeas.volume.widened,
		"|Volume.Value - (pi*25/4)*10| must be <= Volume.Bound + sectionDelta*areaUpper; got |%.10g-%.10g|=%.3e, allowed %.3e (raw Bound=%.3e + term=%.3e)",
		gotVolume, wantVolume, math.Abs(gotVolume-wantVolume), arcMeas.volume.widened, vol.Bound.Base(), sectionDeltaVolumeTerm)

	arcEnvelope := wedgeArcEnvelope(t)
	arcFraction := arcSD / arcEnvelope
	arcBinding := arcMeas.binding()
	arcMargin := toleranceRel / arcBinding.ratio
	t.Logf("A10a pin: m=%d F=%d elapsed=%s binding=%s ratio=%.6g margin=%.3gx impliedFraction=%.6g", loftChordFractionPinM, arcMeas.f, arcElapsed, arcBinding.reading, arcBinding.ratio, arcMargin, arcFraction)
	// The achieved margin, asserted numerically rather than the verdict alone
	// (PR 1's task 4): this fixture does NOT reach 4x at the budget-compliant m=64
	// (Volume is binding here — see loftChordFractionPinM's comment) — assert the
	// weaker margin actually measured (~2.4x), loudly, rather than silently
	// asserting 4x. require.InEpsilon pins the measured value against drift while
	// tolerating ordinary float rounding.
	require.Greater(t, arcBinding.ratio, 0.0, "the binding reading must have formed a usable reference")
	require.Less(t, arcBinding.ratio, toleranceRel, "the binding reading must still be Sound (ratio < 1e-3) at m=%d", loftChordFractionPinM)
	require.InEpsilon(t, 2.394, arcMargin, 0.05, "the achieved arc-wedge margin at m=%d, pinned so a future change to the widening formula is caught", loftChordFractionPinM)

	// --- A10b: the fit-spline wedge, checked against a dense-sample reference
	// since the spline has no closed form to compare against ---
	fs := wedgeFitSpline(t)
	const splineSamplesPerCell = 200
	const splineDenseN = 20000

	splineStart := time.Now()
	splinePts := wedgeSplinePoints(fs, loftChordFractionPinM)
	splineSD := splineSagitta(fs, loftChordFractionPinM, splineSamplesPerCell)
	splineAE := splineChordExcess(fs, loftChordFractionPinM, wedgeHeight, splineDenseN)
	splineMeas := measureWedgeReadings(t, splinePts, splineSD, splineAE)
	splineElapsed := time.Since(splineStart)
	require.Less(t, splineElapsed, 2*time.Second, "PR 1's acceptance line: the spline wedge must build in under 2s")

	denseVerts := append([][2]float64{{0, 0}}, wedgeSplinePoints(fs, splineDenseN)...)
	denseArea, _, _ := wedgeShoelaceRegion(denseVerts)
	denseVolume := denseArea * wedgeHeight

	// Reuse the body measureWedgeReadings already built above — the same
	// second-build avoidance as the arc wedge.
	splineVol, err := splineMeas.body.Volume()
	require.NoError(t, err)
	gotSplineVolume := splineVol.Value.Base()
	require.LessOrEqual(t, math.Abs(gotSplineVolume-denseVolume), splineMeas.volume.widened,
		"the analogous enclosure for the spline wedge: |Volume.Value - denseSampleVolume| must be <= Volume.Bound + sectionDelta*areaUpper; got |%.10g-%.10g|=%.3e, allowed %.3e",
		gotSplineVolume, denseVolume, math.Abs(gotSplineVolume-denseVolume), splineMeas.volume.widened)

	splineEnvelope := wedgeSplineEnvelope(t)
	splineFraction := splineSD / splineEnvelope
	splineBinding := splineMeas.binding()
	splineMargin := toleranceRel / splineBinding.ratio
	t.Logf("A10b pin: m=%d F=%d elapsed=%s binding=%s ratio=%.6g margin=%.3gx impliedFraction=%.6g", loftChordFractionPinM, splineMeas.f, splineElapsed, splineBinding.reading, splineBinding.ratio, splineMargin, splineFraction)
	require.Greater(t, splineBinding.ratio, 0.0, "the binding reading must have formed a usable reference")
	require.Less(t, splineBinding.ratio, toleranceRel, "the binding reading must still be Sound (ratio < 1e-3) at m=%d", loftChordFractionPinM)
	require.InEpsilon(t, 1.898, splineMargin, 0.05, "the achieved spline-wedge margin at m=%d, pinned so a future change to the widening formula is caught", loftChordFractionPinM)

	// The recommended shared loftChordFraction (Q2's tie-break: "the constant takes
	// the finer of the two") is the SMALLER of the two implied fractions — a smaller
	// fraction only tightens the future evaluator's own chordTarget for both arms,
	// so using the finer one guarantees at least m=64 stations on whichever arm a
	// caller's own section resembles. Report it rather than assert it: it is a
	// derived float from two independently-measured envelopes, not a value this
	// harness can pin bit-for-bit without coupling to profileCoordinateUpper's own
	// internals.
	loftChordFraction := min(arcFraction, splineFraction)
	t.Logf("recommended loftChordFraction (finer of the two) = %.6g, from arc=%.6g spline=%.6g", loftChordFraction, arcFraction, splineFraction)
	require.Positive(t, loftChordFraction)
}
