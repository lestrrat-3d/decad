package decad

import (
	"fmt"
	"math"
	"os"
	"strings"
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
// — the per-cell chord-vs-curve displacement the evaluator publishes — is absent
// from such a build, and this file supplies it and adds it arithmetically to each
// reading's Bound before re-running verify.go's own tolerance gate
// (scalarToleranceRef/boundedToleranceRef) on the widened value by hand.
//
// The two arms supply it differently, and deliberately so. The A10a (circular) arm
// takes BOTH its chord vertices and its sectionDelta from the SHIPPED generator
// (wedgeArcChords), so every margin below is a reading of the geometry decad
// actually builds and moves when that generator moves. The A10b (fit-spline) arm
// stays a local stand-in — dense numeric sampling for its sagitta, since a fit
// spline has no closed form and no free-form pairing arm ships to ask.

// --- shared wedge geometry ---

const (
	wedgeRadius = 5.0         // A10a/A10b's quarter-arc radius
	wedgeSweep  = math.Pi / 2 // the quarter-arc sweep
	wedgeHeight = 10.0        // the loft's Z extrusion distance

	// toleranceRel is verify.go's WithTolerance default (verify.go:390-394): the
	// relative tolerance every widened bound below is compared against.
	toleranceRel = 1e-3
)

// wedgeArcChords returns the m+1 chord vertices for the A10a arm, and the
// sectionDelta a real build publishes for them — both read off the SHIPPED
// production generator, never off a local cos/sin loop.
//
// This is what makes every A10a reading in this file a measurement of the
// geometry decad actually builds. The vertices are circularStationChain's own
// output for the recorded quarter-arc and the walk walkOf resolves for it
// (loft_build.go), closed by that walk's own end point — the station the NEXT
// segment of a real loop would contribute, which is the arc's recorded End.
// The delta is chordCellDeltaUpper over that same generator's two terms: the
// certified per-cell sagitta and the generated stations' own displacement. A
// local reimplementation of either would leave the fixture measuring itself:
// it would keep reporting the same margins however far production's own
// stations drifted from the curve they chord.
func wedgeArcChords(t *testing.T, m int) ([][2]float64, float64) {
	t.Helper()
	seg, w := wedgeArcRecord(t)
	stations, stationDelta := circularStationChain(w, seg, m)
	require.False(t, isNonFinite(stationDelta), "the shipped generator must state its stations' own displacement at m=%d", m)
	pts := make([][2]float64, 0, m+1)
	for _, p := range stations {
		pts = append(pts, [2]float64{p.U, p.V})
	}
	pts = append(pts, [2]float64{w.endU, w.endV})

	sd := chordCellDeltaUpper(loftCertifiedSagittaUpper(seg, m), stationDelta)
	require.False(t, isNonFinite(sd), "the shipped generator must state a chord bound at m=%d", m)
	return pts, sd
}

// wedgeFitSpline builds the A10b reference curve once: a 5-point fit spline through
// the same quarter circle, at k*pi/8 for k = 0..4 (angles 0, pi/8, pi/4, 3pi/8, pi/2).
// Every A10b fixture below samples THIS curve's own Eval, never the circle, since the
// task requires the hand-chorded stand-in to chord the spline's own curve.
func wedgeFitSpline(t testing.TB) *sketch.FitSpline {
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
func wedgePlanes(t testing.TB) (*sketch.World, *sketch.Plane, *sketch.Plane) {
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
func chordedWedgeProfile(t testing.TB, w *sketch.World, plane *sketch.Plane, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
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
// itself took — the quantity the audit cost docs/loft-design.md §13's build cost
// model paragraph states, over the F §7 owns, is measured against (Q3).
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
// wedgeFitSpline samples, closed back through the origin by the two radial lines
// chordedWedgeProfile also draws. It is therefore the uncorded twin of the chorded
// A10b body — same wedge region, the chord chain replaced by the curve itself —
// exactly as wedgeArcSketch is the uncorded twin of the chorded A10a body.
func wedgeSplineSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	pts := make([]*sketch.Point, 5)
	for k := range pts {
		theta := float64(k) * math.Pi / 8
		p := s.CreatePoint(wedgeRadius*math.Cos(theta), wedgeRadius*math.Sin(theta))
		s.Fix(p)
		pts[k] = p
	}
	_, err = s.CreateFitSpline(pts...)
	require.NoError(t, err)
	s.CreateLine(origin, pts[0])
	s.CreateLine(pts[len(pts)-1], origin)
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
	u0, err := profileCoordinateUpper(rec0, work, nil)
	require.NoError(t, err)
	u1, err := profileCoordinateUpper(rec1, work, nil)
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
	u0, err := profileCoordinateEnvelope(rec0, work, nil)
	require.NoError(t, err)
	u1, err := profileCoordinateEnvelope(rec1, work, nil)
	require.NoError(t, err)
	return math.Max(u0, u1)
}

// requireFeetMatchChordEnds asserts that the two radial feet of a true outline are
// the same two points the chorded stand-in's own chain starts and ends at. Both
// lists are ordered by descending u first, so the wedge's (r,0) corner is compared
// against (r,0) and its (0,r) corner against (0,r).
func requireFeetMatchChordEnds(t *testing.T, feet [][2]float64, chorded [][2]float64) {
	t.Helper()
	require.Len(t, feet, 2, "a true wedge outline closes through the origin on exactly two radial lines")
	if feet[0][0] < feet[1][0] {
		feet[0], feet[1] = feet[1], feet[0]
	}
	want := [][2]float64{chorded[0], chorded[len(chorded)-1]}
	if want[0][0] < want[1][0] {
		want[0], want[1] = want[1], want[0]
	}
	for i := range want {
		require.InDelta(t, want[i][0], feet[i][0], 1e-12,
			"the radial lines reach the same two corners the chorded stand-in starts and ends at")
		require.InDelta(t, want[i][1], feet[i][1], 1e-12,
			"the radial lines reach the same two corners the chorded stand-in starts and ends at")
	}
}

// TestLoftChordCalibrationTrueOutlineIsTheChordedTwin pins the fidelity both
// envelope readings rest on: each TRUE outline encloses the same wedge region its
// chorded stand-in does. chordedWedgeProfile walks origin -> pts[0] -> ... ->
// pts[last] -> origin, so the uncorded twin is origin -> the curve's first defining
// point -> the curve -> its last defining point -> origin, with the chord chain and
// nothing else replaced by the curve. An outline closed straight from the curve's
// last point back to its first would enclose a lens rather than a wedge, and its
// envelope would then be read off a different region than every other reading here
// is measured on. It records profiles only and builds no loft, so it always runs.
func TestLoftChordCalibrationTrueOutlineIsTheChordedTwin(t *testing.T) {
	// radialFeet returns, for each straight segment of the recorded loop, the
	// endpoint away from the origin — and fails the test outright on a straight
	// segment that touches the origin at neither end, which is exactly the lens
	// closure this fixture must not have.
	radialFeet := func(t *testing.T, rec ProfileRecord) [][2]float64 {
		t.Helper()
		var feet [][2]float64
		for _, seg := range rec.Outer.Segments {
			line, ok := seg.(LineSeg)
			if !ok {
				continue
			}
			switch {
			case line.Start == (Point2{}):
				feet = append(feet, [2]float64{line.End.U, line.End.V})
			case line.End == (Point2{}):
				feet = append(feet, [2]float64{line.Start.U, line.Start.V})
			default:
				t.Fatalf("a true wedge outline's straight segments are radial, but %v reaches the origin at neither end", line)
			}
		}
		return feet
	}

	t.Run("A10a: the arc outline is the chorded circle wedge's twin", func(t *testing.T) {
		w, base, _ := wedgePlanes(t)
		s, p := wedgeArcSketch(t, w, base)
		rec, _, err := RecordProfile(s, p)
		require.NoError(t, err)
		require.Len(t, rec.Outer.Segments, 3, "two radial lines plus the arc")

		curved := 0
		for _, seg := range rec.Outer.Segments {
			if _, ok := seg.(ArcSeg); ok {
				curved++
			}
		}
		require.Equal(t, 1, curved, "the curved side is the arc itself, never a chord")

		chords, _ := wedgeArcChords(t, wedgePinStations(t))
		requireFeetMatchChordEnds(t, radialFeet(t, rec), chords)
	})

	t.Run("A10b: the spline outline is the chorded spline wedge's twin", func(t *testing.T) {
		w, base, _ := wedgePlanes(t)
		s, p := wedgeSplineSketch(t, w, base)
		rec, _, err := RecordProfile(s, p)
		require.NoError(t, err)
		require.Len(t, rec.Outer.Segments, 3, "two radial lines plus the fit spline")

		curved := 0
		for _, seg := range rec.Outer.Segments {
			if _, ok := seg.(FitSplineSeg); ok {
				curved++
			}
		}
		require.Equal(t, 1, curved, "the curved side is the fit spline itself, never a chord")

		fs := wedgeFitSpline(t)
		requireFeetMatchChordEnds(t, radialFeet(t, rec), wedgeSplinePoints(fs, wedgePinStations(t)))
	})
}

// --- sectionDelta and the Area excesses, per arm ---

// wedgeAreaExcess is how much more surface the curved wedge carries than the
// hand-chorded stand-in this file actually builds, split by where it lives: wall is
// the lateral ruled-surface-versus-chord excess summed over the m cells, and cap is
// the curved-region-versus-chorded-polygon excess over BOTH caps. Each arm's own
// helper (arcChordExcess, splineChordExcess) is the single owner of its two values.
//
// The split matters because the two terms are consumed differently: the Area reading
// is widened by the wall term alone (a10-plan.md Part 2 Q4: "the Area bound gains one
// wall term and no cap term"), while the area-along-the-path ceiling measureWedgeReadings
// hands the Volume and Centroid widenings takes both, since it must bound a whole
// closed intermediate surface rather than its walls.
type wedgeAreaExcess struct {
	wall float64
	cap  float64
}

// total is the excess over the WHOLE boundary — walls and both caps — which is what
// the area-along-the-path ceiling adds to the held chord surface.
func (e wedgeAreaExcess) total() float64 { return e.wall + e.cap }

// arcSagitta is the EXACT closed-form per-cell sagitta of the reference quarter
// arc at a forced station count m: 2r*sin^2(sweep/m/4) over the fixture's own
// wedgeRadius/wedgeSweep. It is NOT this file's sectionDelta stand-in — that is
// wedgeArcChords' own production reading — and has exactly one caller left:
// wedgePinStations' seed-straddle assertion, which contrasts chordCount's
// deliberately conservative r*sweep^2/(8n^2) against this exact value to show
// WHY the joint walk-up seeds one station above the grid point.
// tessellate.go publishes the proven upper bound on that displacement
// (docs/tessellation-design.md Sec 3); this value is the displacement itself.
func arcSagitta(m int) float64 {
	s := math.Sin(wedgeSweep / float64(m) / 4)
	return 2 * wedgeRadius * s * s
}

// arcChordExcess is the A10a arm's wedgeAreaExcess: how much more surface the
// TRUE quarter-arc wedge carries than the chorded stand-in this file builds.
//
// The true half of each term is closed form over the fixture's own constants —
// the arc's length r*sweep and the sector's area r^2*sweep/2. The chorded half
// is measured on the SHIPPED chord vertices (wedgeArcChords), never on the
// closed-form m-chord polygon, so a production generator whose stations drift
// off the circle moves this excess rather than leaving it untouched.
func arcChordExcess(t *testing.T, m int) wedgeAreaExcess {
	t.Helper()
	const r, sweep = wedgeRadius, wedgeSweep
	pts, _ := wedgeArcChords(t, m)

	chordLen := 0.0
	for i := 1; i < len(pts); i++ {
		chordLen += math.Hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
	}
	polygon := wedgeShoelaceArea(append([][2]float64{{0, 0}}, pts...))
	return wedgeAreaExcess{
		wall: (r*sweep - chordLen) * wedgeHeight,
		cap:  2 * (r*r*sweep/2 - math.Abs(polygon)),
	}
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

// splineRegionArea is the area of the wedge region the n-chord stand-in encloses:
// the closed polygon origin -> fs.Eval(0) -> ... -> fs.Eval(1) -> origin. At the
// fixture's own m it is the built cap's area; at a dense n it is this file's
// stand-in for the true curved cap, which the fit spline has no closed form for.
func splineRegionArea(fs *sketch.FitSpline, n int) float64 {
	verts := append([][2]float64{{0, 0}}, wedgeSplinePoints(fs, n)...)
	area := wedgeShoelaceArea(verts)
	return math.Abs(area)
}

// splineChordExcess is arcChordExcess's free-form twin, measured densely rather than
// in closed form. The wall term is the densely measured true curve length minus the
// m-chord polygon's own length, times the extrusion height. The cap term is the
// densely measured true region area minus the m-chord region's own area, doubled for
// the two caps.
func splineChordExcess(fs *sketch.FitSpline, m int, height float64, denseN int) wedgeAreaExcess {
	trueLen := splineDenseLength(fs, denseN)
	pts := wedgeSplinePoints(fs, m)
	chordLen := 0.0
	for i := 1; i < len(pts); i++ {
		chordLen += math.Hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
	}
	return wedgeAreaExcess{
		wall: (trueLen - chordLen) * height,
		cap:  2 * (splineRegionArea(fs, denseN) - splineRegionArea(fs, m)),
	}
}

// wedgeShoelaceArea is a closed polygon's SIGNED area by the standard shoelace sum
// over its vertices in order (the last implicitly reconnects to the first). Both
// arms' cap regions are measured through it: the A10b arm's dense-sample reference
// and its chorded cap (splineRegionArea), and the A10a arm's own chorded cap over
// the SHIPPED station vertices (arcChordExcess). decad's own Area/Centroid always
// come from the actual built body, never from this helper.
func wedgeShoelaceArea(verts [][2]float64) float64 {
	var a float64
	n := len(verts)
	for i := range n {
		x0, y0 := verts[i][0], verts[i][1]
		x1, y1 := verts[(i+1)%n][0], verts[(i+1)%n][1]
		a += x0*y1 - x1*y0
	}
	return a / 2
}

// --- the gate reproduction: verify.go's own scalarToleranceRef/boundedToleranceRef,
// run on the WIDENED bound each reading would carry once sectionDelta exists ---

// widenedGateRow is one reading's widened-bound gate comparison: value is the
// measurement the built body published, widened is the production Bound plus this
// file's own sectionDelta term, ref is the diameter-anchored reference verify.go's
// own gate built, ratio = widened/ref is what the default 1e-3 tolerance compares
// against 1, haveRef reports whether the gate formed that reference at all, and
// sound reports whether the comparison passes.
//
// value is carried as already-rendered text because the four readings are not one
// shape: Volume and Area are scalars, Centroid is a position and Bounds is a box.
// a10-plan.md Part 2 Q2 step 2 requires each swept m to record the four
// measurements themselves and not only their bounds, and no other column carries
// them: Volume and Area are readable off ref only because volumeReference and
// areaReference return max(value, tiny) (verify.go:1152-1174), while Centroid and
// Bounds are compared against the body diameter instead.
type widenedGateRow struct {
	reading string
	value   string
	widened float64
	ref     float64
	ratio   float64
	haveRef bool
	sound   bool
}

// verdict is the row's Sound/Suspect word at the default tolerance, and it never
// reads the same for a pass and a fail. Both scalarToleranceRef and
// boundedToleranceRef (verify.go:1046-1080) can decline to form a reference, for two
// opposite reasons: a bound of exactly zero passes with no reference needed, while
// an unusable magnitude or a reference the body cannot supply fails. Both leave ref
// and ratio at 0, so the verdict word is the only column that separates them, and it
// names which of the two happened.
func (r widenedGateRow) verdict() string {
	switch {
	case r.haveRef && r.sound:
		return "Sound"
	case r.haveRef:
		return "Suspect"
	case r.sound:
		return "Sound(zero-bound,no-ref)"
	default:
		return "Suspect(no-ref)"
	}
}

// wedgeMeasurement is one m's full row: the closed-form/measured sectionDelta this
// file added to every Bound, the area-along-the-path ceiling the Volume and Centroid
// widenings multiplied it by, the built body's own face count (the F
// docs/loft-design.md §7 owns) and wall-clock, and the four widened-bound gate rows.
type wedgeMeasurement struct {
	m            int
	sectionDelta float64
	areaUpper    float64 // the ceiling measureWedgeReadings' own doc comment derives
	f            int
	elapsed      time.Duration
	body         *Body // the already-built loft, reused so callers never rebuild it

	volume, area, bounds, centroid widenedGateRow
}

// rows returns the four gate rows in the order every logged sweep row prints them.
func (m wedgeMeasurement) rows() []widenedGateRow {
	return []widenedGateRow{m.volume, m.area, m.bounds, m.centroid}
}

// binding returns the reading with the largest ratio — the one that decides whether
// this m reads Sound at the default tolerance, per the plan's "the BINDING reading is
// the one with the largest ratio".
func (m wedgeMeasurement) binding() widenedGateRow {
	worst := m.volume
	for _, r := range m.rows()[1:] {
		if r.ratio > worst.ratio {
			worst = r
		}
	}
	return worst
}

// verdict is the whole measurement's Sound/Suspect word: this m reads Sound only
// when all four widened readings clear the default tolerance. It is computed over
// every row rather than read off binding(), because a reading the gate could form no
// reference for carries ratio 0 and so is never the binding row even when it fails.
func (m wedgeMeasurement) verdict() string {
	for _, r := range m.rows() {
		if !r.sound {
			return "Suspect"
		}
	}
	return "Sound"
}

// marginText renders the achieved margin on the binding reading, or names the reason
// there is none. A binding ratio of 0 means the gate formed no usable reference, and
// then toleranceRel/ratio would print +Inf for a pass and for a fail alike.
func (m wedgeMeasurement) marginText() string {
	binding := m.binding()
	if binding.ratio <= 0 {
		return "n/a(no-ref)"
	}
	return fmt.Sprintf("%.3gx", toleranceRel/binding.ratio)
}

// measureWedgeReadings builds the chorded loft over pts, then widens each of the
// four readings' Bound by the term Part 2 Q2 states for it and re-runs verify.go's
// own tolerance gate on the widened value:
//   - Volume:   + sectionDelta * areaUpper                  (chordedBoundaryVolumeAllow)
//   - Area:     + excess.wall                               (ruled-vs-chord wall excess)
//   - Bounds:   + sectionDelta
//   - Centroid: + sectionDelta*(diameter/2+|centroid|)*areaUpper/volume (a calibration
//     ESTIMATE of chordedBoundaryMomentAllow's quotient-rule composition, not the
//     shipped bound — stated in the plan prompt as such)
//
// areaUpper is the area-along-the-path ceiling a10-plan.md Part 2 Q4 states the
// obligation for: it must bound the area of EVERY intermediate surface on the
// chord-to-curve homotopy, "the held chord area plus each cell's own ruled-area
// excess". It is therefore the sum of three terms:
//
//   - the built body's own Area().Value — the held chord surface, walls and both
//     caps;
//   - that reading's own Area().Bound, so a rounding-level error in the held term
//     cannot make the ceiling fall under the surface it holds; and
//   - excess.total() — the wall excess each cell's ruled surface carries over its
//     chord, plus the cap excess each curved cap region carries over its chorded
//     polygon.
//
// Every intermediate surface replaces some cells' chords by curves lying inside the
// sectionDelta-neighbourhood of those chords and leaves the rest held, so its wall
// and cap areas are each at most the fully curved arm's, and the sum above covers
// it. The Area reading's own widening takes excess.wall alone, per Q4's "the Area
// bound gains one wall term and no cap term".
func measureWedgeReadings(t *testing.T, pts [][2]float64, sectionDelta float64, excess wedgeAreaExcess) wedgeMeasurement {
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

	areaUpper := math.Abs(area.Value.Base()) + area.Bound.Base() + excess.total()

	widenedVol := Measurement{Value: vol.Value, Exactness: vol.Exactness, Bound: units.CubicMillimeters(vol.Bound.Base() + sectionDelta*areaUpper)}
	volPass, volRef, volHaveRef := scalarToleranceRef(widenedVol, toleranceRel, in.volumeReference)

	widenedArea := Measurement{Value: area.Value, Exactness: area.Exactness, Bound: units.SquareMillimeters(area.Bound.Base() + excess.wall)}
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

	row := func(name, value string, widened, ref float64, haveRef, pass bool) widenedGateRow {
		r := widenedGateRow{reading: name, value: value, widened: widened, ref: ref, haveRef: haveRef, sound: pass}
		if haveRef && ref != 0 {
			r.ratio = widened / ref
		}
		return r
	}

	// The four VALUES, each in units' own documented base unit for its Kind
	// (mm^3, mm^2, mm), so a recorded row states the measurement itself and not
	// only how far it can be trusted.
	volText := fmt.Sprintf("%.10g", vol.Value.Base())
	areaText := fmt.Sprintf("%.10g", area.Value.Base())
	boundsText := fmt.Sprintf("min(%.6g,%.6g,%.6g)max(%.6g,%.6g,%.6g)",
		bounds.Min.X, bounds.Min.Y, bounds.Min.Z, bounds.Max.X, bounds.Max.Y, bounds.Max.Z)
	centroidText := fmt.Sprintf("(%.6g,%.6g,%.6g)", centroid.Value.X, centroid.Value.Y, centroid.Value.Z)

	return wedgeMeasurement{
		m:            len(pts) - 1,
		sectionDelta: sectionDelta,
		areaUpper:    areaUpper,
		f:            len(body.Faces()),
		elapsed:      elapsed,
		body:         body,
		volume:       row("Volume", volText, widenedVol.Bound.Base(), volRef, volHaveRef, volPass),
		area:         row("Area", areaText, widenedArea.Bound.Base(), areaRef, areaHaveRef, areaPass),
		bounds:       row("Bounds", boundsText, widenedBoundsBound, boundsRef, boundsHaveRef, boundsPass),
		centroid:     row("Centroid", centroidText, widenedCentroidBound, centroidRef, centroidHaveRef, centroidPass),
	}
}

// formatWedgeMeasurement renders one row of the calibration table: everything
// a10-plan.md Part 2 Q2 step 2 requires each swept m to record — the four
// measurements with their VALUES and their widened bounds, the gate reference each
// was compared against, each one's Verify verdict at the default 1e-3, the
// sectionDelta this file added and the areaUpper ceiling it multiplied that by, the
// face count F (§7) and the loft's wall-clock.
func formatWedgeMeasurement(label string, m wedgeMeasurement) string {
	var b strings.Builder
	fmt.Fprintf(&b, "%-14s m=%-4d sectionDelta=%.6g areaUpper=%.6g F=%-4d elapsed=%-12s", label, m.m, m.sectionDelta, m.areaUpper, m.f, m.elapsed)
	for _, c := range []struct {
		tag string
		row widenedGateRow
	}{{"vol", m.volume}, {"area", m.area}, {"bounds", m.bounds}, {"centroid", m.centroid}} {
		fmt.Fprintf(&b, " %s[v=%s w=%.4e ref=%.4e ratio=%.4e %s]",
			c.tag, c.row.value, c.row.widened, c.row.ref, c.row.ratio, c.row.verdict())
	}
	fmt.Fprintf(&b, " verdict=%s binding=%s margin=%s", m.verdict(), m.binding().reading, m.marginText())
	return b.String()
}

func logWedgeMeasurement(t *testing.T, label string, m wedgeMeasurement) {
	t.Helper()
	t.Log(formatWedgeMeasurement(label, m))
}

// TestLoftChordCalibrationRowRecordsValueAndVerdict pins the recording contract
// a10-plan.md Part 2 Q2 step 2 states for the sweep: every row carries each of the
// four measurements' own VALUE and its Verify verdict, on top of the widened bound,
// the reference and the ratio. It builds one m=8 loft, whose F is §7's and whose
// cost sits inside the budget docs/loft-design.md §13's build cost model paragraph
// owns, and it always runs, so the contract holds even though the sweep itself is
// opt-in behind DECAD_LOFT_CALIBRATION.
func TestLoftChordCalibrationRowRecordsValueAndVerdict(t *testing.T) {
	t.Run("a no-reference row reads differently for a pass and a fail", func(t *testing.T) {
		require.Equal(t, "Sound", widenedGateRow{haveRef: true, sound: true}.verdict())
		require.Equal(t, "Suspect", widenedGateRow{haveRef: true, sound: false}.verdict())
		require.Equal(t, "Sound(zero-bound,no-ref)", widenedGateRow{sound: true}.verdict())
		require.Equal(t, "Suspect(no-ref)", widenedGateRow{}.verdict())

		// A row the gate formed no reference for carries ratio 0 either way, so it is
		// never the binding row and the margin column cannot separate the two. The
		// measurement's own verdict is what makes the failure visible.
		ok := widenedGateRow{reading: "Volume", sound: true}
		bad := widenedGateRow{reading: "Centroid"}
		passing := wedgeMeasurement{volume: ok, area: ok, bounds: ok, centroid: ok}
		failing := wedgeMeasurement{volume: ok, area: ok, bounds: ok, centroid: bad}
		require.Equal(t, "Sound", passing.verdict())
		require.Equal(t, "Suspect", failing.verdict())
		require.Equal(t, passing.binding().reading, failing.binding().reading)
		require.Equal(t, "n/a(no-ref)", passing.marginText())
		require.Equal(t, "n/a(no-ref)", failing.marginText())
		require.NotEqual(t, formatWedgeMeasurement("A10a(arc)", passing), formatWedgeMeasurement("A10a(arc)", failing))
	})

	t.Run("every reading records the value the built body published", func(t *testing.T) {
		const m = 8
		pts, sd := wedgeArcChords(t, m)
		meas := measureWedgeReadings(t, pts, sd, arcChordExcess(t, m))

		vol, err := meas.body.Volume()
		require.NoError(t, err)
		area, err := meas.body.Area()
		require.NoError(t, err)
		centroid, err := meas.body.Centroid()
		require.NoError(t, err)
		bounds, err := meas.body.Bounds()
		require.NoError(t, err)

		// The recorded values are the geometry, not placeholders: the m-chord wedge is
		// m isosceles triangles of area (1/2)r^2*sin(sweep/m) extruded wedgeHeight, its
		// centroid sits at mid-height, and its box spans the full extrusion in Z.
		wantVolume := 0.5 * float64(m) * wedgeRadius * wedgeRadius * math.Sin(wedgeSweep/float64(m)) * wedgeHeight
		require.InDelta(t, wantVolume, vol.Value.Base(), 1e-9)
		require.InDelta(t, wedgeHeight/2, centroid.Value.Z, 1e-9)
		require.InDelta(t, 0.0, bounds.Min.Z, 1e-12)
		require.InDelta(t, wedgeHeight, bounds.Max.Z, 1e-12)

		require.Equal(t, fmt.Sprintf("%.10g", vol.Value.Base()), meas.volume.value)
		require.Equal(t, fmt.Sprintf("%.10g", area.Value.Base()), meas.area.value)
		require.Equal(t, fmt.Sprintf("(%.6g,%.6g,%.6g)", centroid.Value.X, centroid.Value.Y, centroid.Value.Z), meas.centroid.value)
		require.Equal(t, fmt.Sprintf("min(%.6g,%.6g,%.6g)max(%.6g,%.6g,%.6g)",
			bounds.Min.X, bounds.Min.Y, bounds.Min.Z, bounds.Max.X, bounds.Max.Y, bounds.Max.Z), meas.bounds.value)

		// Centroid is the reading no other column can recover: it is compared against
		// the body diameter, so its value reaches the table only through v=.
		line := formatWedgeMeasurement("A10a(arc)", meas)
		for _, c := range []struct {
			tag string
			row widenedGateRow
		}{{"vol", meas.volume}, {"area", meas.area}, {"bounds", meas.bounds}, {"centroid", meas.centroid}} {
			require.Contains(t, line, fmt.Sprintf("%s[v=%s ", c.tag, c.row.value))
			require.Contains(t, line, fmt.Sprintf("ratio=%.4e %s]", c.row.ratio, c.row.verdict()))
			require.True(t, c.row.haveRef, "the m=8 wedge forms a reference for every reading, so no verdict here is the degenerate one")
		}
		require.Contains(t, line, " verdict="+meas.verdict()+" ")
		require.Contains(t, line, " margin="+meas.marginText())
		t.Log(line)
	})
}

// TestLoftChordCalibrationCeilingCoversRuledExcess discharges the obligation
// a10-plan.md Part 2 Q4 places on the areaUpper argument of a chord-to-curve
// homotopy: the area-along-the-path ceiling must bound "the held chord area plus
// each cell's own ruled-area excess", so the Volume and Centroid widenings built on
// it cannot understate the chord-to-curve gap. It asserts the COMPUTED ceiling
// against an independently summed per-cell excess, and asserts that the two
// widenings actually consumed that ceiling. It builds one m=8 loft, whose F is §7's
// and whose cost sits inside the budget §13's build cost model paragraph owns, and
// it always runs, since the sweep it protects is opt-in.
func TestLoftChordCalibrationCeilingCoversRuledExcess(t *testing.T) {
	const m = 8
	excess := arcChordExcess(t, m)
	pts, sd := wedgeArcChords(t, m)
	meas := measureWedgeReadings(t, pts, sd, excess)

	// The per-cell ruled-wall excess, summed independently of arcChordExcess's own
	// whole-arc-minus-whole-polygon form: each of the m cells spans sweep/m of the
	// true arc, so the ruled wall over that cell carries
	// (r*sweep/m - 2r*sin(sweep/2m)) * height more area than its chord's flat wall.
	cellArc := wedgeRadius * wedgeSweep / float64(m)
	cellChord := 2 * wedgeRadius * math.Sin(wedgeSweep/(2*float64(m)))
	perCellWall := float64(m) * (cellArc - cellChord) * wedgeHeight
	require.Positive(t, perCellWall, "the m=8 chord polygon is strictly shorter than the arc it stands in for")
	require.InDelta(t, perCellWall, excess.wall, 1e-12)

	// The cap excess, likewise summed independently: the true circular sector minus
	// the m-chord polygon it is chorded by, over both caps.
	perCap := 0.5*wedgeRadius*wedgeRadius*wedgeSweep - 0.5*float64(m)*wedgeRadius*wedgeRadius*math.Sin(wedgeSweep/float64(m))
	require.Positive(t, perCap, "the m=8 chord polygon encloses strictly less than the sector it stands in for")
	require.InDelta(t, 2*perCap, excess.cap, 1e-12)

	area, err := meas.body.Area()
	require.NoError(t, err)
	held := math.Abs(area.Value.Base())

	// The ceiling COVERS the held surface plus the per-cell ruled-wall excess — the
	// obligation itself — and also the cap excess, so it bounds a whole intermediate
	// surface rather than its walls alone.
	require.GreaterOrEqual(t, meas.areaUpper, held+perCellWall,
		"the area-along-the-path ceiling must cover the held chord surface plus each cell's own ruled-wall excess")
	require.GreaterOrEqual(t, meas.areaUpper, held+perCellWall+2*perCap,
		"the ceiling must also cover the curved caps' excess over their chorded polygons")

	// Both widenings consume that ceiling: Volume multiplies it by sectionDelta, and
	// Centroid divides the same product by the body's own volume after scaling it by
	// the coordinate reach.
	vol, err := meas.body.Volume()
	require.NoError(t, err)
	require.InDelta(t, vol.Bound.Base()+meas.sectionDelta*meas.areaUpper, meas.volume.widened, 1e-15)
	require.Greater(t, meas.volume.widened, vol.Bound.Base()+meas.sectionDelta*(held+area.Bound.Base()),
		"a ceiling that omitted the excess would widen Volume by strictly less")

	centroid, err := meas.body.Centroid()
	require.NoError(t, err)
	diameter, diamOK, err := bodyGateDiameter(t.Context(), meas.body)
	require.NoError(t, err)
	require.True(t, diamOK)
	centroidMag := math.Sqrt(centroid.Value.X*centroid.Value.X + centroid.Value.Y*centroid.Value.Y + centroid.Value.Z*centroid.Value.Z)
	wantCentroid := centroid.Bound.Base() + meas.sectionDelta*(diameter/2+centroidMag)*meas.areaUpper/math.Abs(vol.Value.Base())
	require.InDelta(t, wantCentroid, meas.centroid.widened, 1e-15)

	// The Area reading keeps the wall term alone, per Q4's "the Area bound gains one
	// wall term and no cap term".
	require.InDelta(t, area.Bound.Base()+excess.wall, meas.area.widened, 1e-15)

	t.Log(formatWedgeMeasurement("A10a(arc)", meas))
}

// --- the sweep: TestLoftChordCalibrationSweep, opt-in only ---

// decadLoftCalibrationEnv is the explicit opt-in TestLoftChordCalibrationSweep
// requires. testing.Short() alone does not gate anything in the package's default
// `go test ./...` run — that flag is false unless a caller passes -short — so the
// sweep needs its own default-off switch to keep Q3's "at most three fixtures"
// budget — the one docs/loft-design.md §13's build cost model paragraph owns — out
// of the normal suite. Set it to any non-empty value to run the sweep.
const decadLoftCalibrationEnv = "DECAD_LOFT_CALIBRATION"

// TestLoftChordCalibrationSweep is a10-plan.md PR 1's calibration procedure (Part 2
// Q2, Part 3 PR 1 tasks 2-3): both wedges, hand-chorded at m = 4, 8, 16, 32, 64, 128,
// each row logging the column set formatWedgeMeasurement owns. It is a
// one-time measurement harness, not a regression fixture, and its m=128 rows carry
// an F (§7) and a build cost well past the budget docs/loft-design.md §13's build
// cost model paragraph owns (Q3), so it stays out of the default `go test ./...`
// run entirely and costs it nothing: it skips unless
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
		pts, sd := wedgeArcChords(t, m)
		ex := arcChordExcess(t, m)
		meas := measureWedgeReadings(t, pts, sd, ex)
		logWedgeMeasurement(t, "A10a(arc)", meas)
		t.Logf("  A10a m=%d implied loftChordFraction = sagitta/envelope = %.6g", m, sd/arcEnvelope)
	}

	for _, m := range ms {
		pts := wedgeSplinePoints(fs, m)
		sd := splineSagitta(fs, m, splineSamplesPerCell)
		ex := splineChordExcess(fs, m, wedgeHeight, splineDenseN)
		meas := measureWedgeReadings(t, pts, sd, ex)
		logWedgeMeasurement(t, "A10b(spline)", meas)
		t.Logf("  A10b m=%d implied loftChordFraction = sagitta/envelope = %.6g", m, sd/splineEnvelope)
	}
}

// --- the pin: fast, always-run ---

// loftChordBuildCeiling is a RUNAWAY guard, deliberately far above the
// per-fixture wall-clock budget docs/loft-design.md §13's build cost model
// paragraph owns (a10-plan.md Q3). That budget is a design constraint measured
// on a reference machine, NOT a portable property
// of any one run: the same build measures about 1.4s on the development host and
// about 2.3s on a CI Windows runner, and about 9.7s on a CI Linux runner under
// the race detector, so asserting the budget itself makes the
// suite fail on the slower host while proving nothing about the code. What a
// test CAN assert portably is that the build has not regressed by orders of
// magnitude — §13 states how the audit's cost grows with F (§7), so a station-count
// or cap-count regression shows up as a 10x blowup, not a 1.6x one. The
// achieved time is logged at every run so the budget stays observable. NEVER
// tighten this toward that budget: it reintroduces a host-dependent failure.
const loftChordBuildCeiling = 60 * time.Second

// loftChordFractionPinM is the station count the SHIPPED generator settles the
// reference arc wedge on at the shipped loftChordFraction:
// loftCircularCellStations (loft_build.go), asked for loftChordFraction *
// wedgeArcEnvelope, settles its joint walk-up at 65 chords. wedgePinStations
// re-derives it from that generator at every run and requires the two to
// agree, so every fixture below is chorded at a count production actually
// produces and this literal is a pin on the generator's own answer, never a
// hand-forced count.
//
// The constant that count is measured at was itself read off
// TestLoftChordCalibrationSweep's table over the mandated grid m = 4, 8, 16,
// 32, 64, 128 (a10-plan.md Part 2 Q2's "chordTarget = loftChordFraction *
// envelope" rule). On that grid the 4x-margin requirement and the wall-clock
// budget §13 owns do NOT hold simultaneously (a10-plan.md's risk R2,
// confirmed by measurement rather than assumed):
//
//   - Volume is the binding reading throughout (not Centroid — the areaUpper
//     ceiling covers the body's whole boundary, walls and both caps, rather than
//     the curved wall alone, which makes the volume term dominate).
//   - The coarsest grid m at which BOTH fixtures clear 4x margin is m=128
//     (arc ratio=1.04e-4, margin=9.58x; spline ratio=1.32e-4, margin=7.55x) —
//     but its assembled F (§7) and its measured build both land outside the
//     budget docs/loft-design.md §13's build cost model paragraph owns.
//   - The finest grid m that still fits that budget is m=64 — Sound (ratio <
//     1e-3 for both) but only ~2.4x (arc) / ~1.9x (spline) margin, short of
//     4x. The shipped constant is that grid point's own implied fraction.
//
// Per the plan's named fallback (Q2, "Fallback if calibration does not close"),
// the coarser, in-budget value ships: Q3's wall-clock ceiling is stated as a
// hard "any fixture that ships in go test ./... builds in 2 seconds or less",
// while the 4x margin is a target on top of Sound, not a second hard gate. A
// loft at this radius/aspect-ratio combination can therefore still read Suspect
// at a tighter-than-default tolerance; that is the plan's accepted, non-silent
// outcome, not a bug.
//
// The pin sits one station ABOVE that grid point because the walk-up's SEED
// proves its bound differently from the way the sweep measures one:
// chordCount asks chordSagitta, whose outward-rounded r*sweep^2/(8n^2) is
// conservative against the exact 2r*sin^2(dtheta/4) arcSagitta evaluates, and
// at m=64 on this fixture the two straddle the target. The joint walk-up only
// ever increments from that seed, so the count stands at 65 even though the
// certified sagitta at 64 already clears. wedgePinStations asserts the
// straddle directly. The production chording is therefore strictly FINER than
// the grid point the constant was read off, and the margins measured here are
// correspondingly wider than that grid row's.
const loftChordFractionPinM = 65

// wedgePinStations asks the PRODUCTION generator — loftCircularCellStations
// (loft_build.go), the same call a real build makes — how many stations the
// reference arc wedge takes at the shipped loftChordFraction, and requires the
// answer to be loftChordFractionPinM. Every fixture in this file is chorded at
// THIS returned count, so no reading here can belong to a chording production
// never produces.
//
// It also ties wedgeArcChords to that same call: the vertices and the
// sectionDelta every A10a reading below is measured on must be the ones
// loftCircularCellStations itself hands a real build at the settled count. The
// count alone would not do it — a fixture that took the count from production
// and its geometry from a local generator would keep reporting these margins
// however far the shipped stations drifted.
func wedgePinStations(t *testing.T) int {
	t.Helper()
	target := loftChordFraction * wedgeArcEnvelope(t)
	seg, w := wedgeArcRecord(t)
	stations, _, sagitta, err := loftCircularCellStations(w, w, seg, seg, target)
	require.NoError(t, err)
	m := len(stations)
	require.LessOrEqual(t, sagitta, target, "the generator's own published bound must meet the target it was asked for")

	// The published bound is the certified sagitta composed with the generated
	// stations' own displacement — both halves, since the certified sagitta
	// alone bounds a chord between the EXACT recorded points, not the chord
	// this build actually draws between two rounded stations.
	certified := loftCertifiedSagittaUpper(seg, m)
	chordPts, chordDelta := wedgeArcChords(t, m)
	require.Equal(t, chordDelta, sagitta, "the published bound is the certified reading composed with the station displacement, at the settled count")
	require.Greater(t, sagitta, certified, "the station displacement is a real term on this fixture, not a rounding that vanishes")
	require.Len(t, chordPts, m+1, "the fixture's vertex list is the m stations plus the walk's own end point")
	for k, p := range stations {
		require.Equal(t, [2]float64{p.U, p.V}, chordPts[k], "every A10a vertex must BE the station the shipped generator produced")
	}

	// The seed that decides this count, asserted rather than described: the
	// held chooser's conservative bound at one station BELOW the pin still
	// reads over target, so chordCount seeds the joint walk-up one station
	// higher, while the exact sagitta formula the sweep measures with is
	// already under target there.
	require.Greater(t, chordSagitta(wedgeRadius, wedgeSweep, m-1), target,
		"the held chooser's own conservative bound at m=%d must exceed the target, or it would not have seeded m=%d", m-1, m)
	require.Less(t, arcSagitta(m-1), target,
		"the exact sagitta at m=%d is already under target, which is why the sweep's own grid row sits one station coarser", m-1)

	t.Logf("the shipped generator on the reference arc wedge: m=%d target=%.10g published=%.17g (certified=%.17g + stations=%.4g); chordSagitta at m=%d is %.10g against an exact sagitta of %.10g",
		m, target, sagitta, certified, sagitta-certified, m-1, chordSagitta(wedgeRadius, wedgeSweep, m-1), arcSagitta(m-1))
	require.Equal(t, loftChordFractionPinM, m, "the pinned station count must be the one the shipped generator produces at the shipped constant")
	return m
}

// wedgeArcRecord is the reference quarter arc as a RECORDED ArcSeg plus the
// walk walkOf resolves for it. The generator reads both — the stations off the
// walk, the certified sagitta off the record — so the pin is measured on the
// pair a real build hands it, never on a hand-built walk with no record behind
// it.
func wedgeArcRecord(t *testing.T) (ArcSeg, segmentWalk) {
	t.Helper()
	seg := ArcSeg{Center: pt(0, 0), Start: pt(wedgeRadius, 0), End: pt(0, wedgeRadius), TStart: 0, TEnd: 1}
	w, err := walkOf(seg, nil)
	require.NoError(t, err)
	require.Equal(t, wedgeRadius, w.radius, "the recorded arc must resolve to the fixture's own radius")
	require.Equal(t, wedgeSweep, w.th1-w.th0, "the recorded arc must resolve to the fixture's own sweep")
	return seg, w
}

// TestLoftChordCalibrationPinsFraction is the fast, always-run pin PR 1's acceptance
// line requires: it re-measures both fixtures at the station count the SHIPPED
// chooser settles the reference wedge on (wedgePinStations, not the whole sweep)
// and asserts the closed-form enclosure and the achieved margin as numbers, never
// merely a Sound/Suspect verdict, and that both fixtures build within the budget
// docs/loft-design.md §13's build cost model paragraph owns.
//
// The fit-spline wedge is chorded at that SAME count. There is no shipped
// generator arm to ask for a free-form pairing — loftCellStations has no
// free-form arm (loft_build.go) — so the two fixtures share the arc arm's own
// production count, which is what makes their two margins comparable readings
// of one constant.
func TestLoftChordCalibrationPinsFraction(t *testing.T) {
	const wantVolume = math.Pi * 25 / 4 * wedgeHeight // 196.349540849...

	m := wedgePinStations(t)

	arcStart := time.Now()
	arcPts, arcSD := wedgeArcChords(t, m)
	arcExcess := arcChordExcess(t, m)
	arcMeas := measureWedgeReadings(t, arcPts, arcSD, arcExcess)
	arcElapsed := time.Since(arcStart)
	t.Logf("A10a pin build wall-clock: %s (a10-plan.md Q3 budget is 2s on the reference host)", arcElapsed)
	require.Less(t, arcElapsed, loftChordBuildCeiling, "the arc wedge build has regressed by orders of magnitude")

	// Reuse the body measureWedgeReadings already built above rather than lofting
	// the same arc wedge a second time — Q3's "at most three fixtures" budget, the
	// one §13's build cost model paragraph owns, counts loft builds and not
	// assertions, so this pin stays at two builds total.
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
	t.Logf("A10a pin: m=%d F=%d elapsed=%s binding=%s ratio=%.6g margin=%.3gx impliedFraction=%.6g", m, arcMeas.f, arcElapsed, arcBinding.reading, arcBinding.ratio, arcMargin, arcFraction)
	// The achieved margin, asserted numerically rather than the verdict alone
	// (PR 1's task 4): this fixture does NOT reach 4x at the production count
	// (Volume is binding here — see loftChordFractionPinM's comment) — assert the
	// weaker margin actually measured (~2.5x), loudly, rather than silently
	// asserting 4x. require.InEpsilon pins the measured value against drift while
	// tolerating ordinary float rounding.
	require.Greater(t, arcBinding.ratio, 0.0, "the binding reading must have formed a usable reference")
	require.Less(t, arcBinding.ratio, toleranceRel, "the binding reading must still be Sound (ratio < 1e-3) at m=%d", m)
	require.InEpsilon(t, 2.4696, arcMargin, 0.05, "the achieved arc-wedge margin at m=%d, pinned so a future change to the widening formula is caught", m)

	// --- A10b: the fit-spline wedge, checked against a dense-sample reference
	// since the spline has no closed form to compare against ---
	fs := wedgeFitSpline(t)
	const splineSamplesPerCell = 200
	const splineDenseN = 20000

	splineStart := time.Now()
	splinePts := wedgeSplinePoints(fs, m)
	splineSD := splineSagitta(fs, m, splineSamplesPerCell)
	splineExcess := splineChordExcess(fs, m, wedgeHeight, splineDenseN)
	splineMeas := measureWedgeReadings(t, splinePts, splineSD, splineExcess)
	splineElapsed := time.Since(splineStart)
	t.Logf("A10b pin build wall-clock: %s (a10-plan.md Q3 budget is 2s on the reference host)", splineElapsed)
	require.Less(t, splineElapsed, loftChordBuildCeiling, "the spline wedge build has regressed by orders of magnitude")

	denseVolume := splineRegionArea(fs, splineDenseN) * wedgeHeight

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
	t.Logf("A10b pin: m=%d F=%d elapsed=%s binding=%s ratio=%.6g margin=%.3gx impliedFraction=%.6g", m, splineMeas.f, splineElapsed, splineBinding.reading, splineBinding.ratio, splineMargin, splineFraction)
	require.Greater(t, splineBinding.ratio, 0.0, "the binding reading must have formed a usable reference")
	require.Less(t, splineBinding.ratio, toleranceRel, "the binding reading must still be Sound (ratio < 1e-3) at m=%d", m)
	require.InEpsilon(t, 1.9483, splineMargin, 0.05, "the achieved spline-wedge margin at m=%d, pinned so a future change to the widening formula is caught", m)

	// The tie-break Q2 states for the shared constant ("the constant takes the
	// finer of the two") is the SMALLER of the two implied fractions — a smaller
	// fraction only tightens chordTarget for both arms, so using the finer one
	// keeps the chooser at this fixture's own station count or above on whichever
	// arm a caller's own section resembles. Report it rather than assert it: it is
	// a derived float from two independently-measured envelopes, not a value this
	// harness can pin bit-for-bit without coupling to profileCoordinateUpper's own
	// internals. The shipped loftChordFraction (loft_build.go) is that tie-break's
	// own answer, read off the sweep grid rather than off this pin.
	finerFraction := min(arcFraction, splineFraction)
	t.Logf("finer of the two implied fractions = %.6g, from arc=%.6g spline=%.6g", finerFraction, arcFraction, splineFraction)
	require.Positive(t, finerFraction)
}
