package decad

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md Part 3 PR 6's own acceptance tests: the
// same-kind circular pairing S3's arc form now admits, S14's station-
// rounding gate, the CCW structural gate, and sectionDelta's composition
// into all four measurements (loft_moments.go's computeLoftChordedAllow).
// wedgePlanes/wedgeArcSketch/wedgeRadius/wedgeSweep/wedgeHeight/toleranceRel
// are loft_chord_calibration_internal_test.go's own PR 1 fixtures (same
// package, same reference wedge).

// --- the A10a wedge, built through the real evaluator ---

// TestLoftArcWedgeBuildsAndMatchesClosedForm is the ask's own "done when"
// line: a two-plane loft whose corresponding segments are arc-to-arc builds,
// and its published Volume encloses the closed-form quarter-cylinder wedge
// volume (pi*r^2/4)*h.
func TestLoftArcWedgeBuildsAndMatchesClosedForm(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	const wantVolume = math.Pi * 25 / 4 * wedgeHeight // 196.349540849...
	vol, err := body.Volume()
	require.NoError(t, err)
	residual := math.Abs(vol.Value.Base() - wantVolume)
	t.Logf("A10a wedge: Volume.Value=%.10g Volume.Bound=%.6e residual=%.6e", vol.Value.Base(), vol.Bound.Base(), residual)
	require.LessOrEqual(t, residual, vol.Bound.Base(),
		"|Volume.Value - (pi*25/4)*10| must be <= Volume.Bound; got value=%.10g bound=%.3e", vol.Value.Base(), vol.Bound.Base())
}

// TestLoftArcWedgeMatchesExtrudeOracle is the ask's own independent-oracle
// line: Document.Extrude on the SAME untwisted congruent section (the base
// plane's own arc-wedge profile, swept the identical wedgeHeight) is an
// INDEPENDENT evaluation path, and the two measurement intervals must
// overlap.
func TestLoftArcWedgeMatchesExtrudeOracle(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	loftDoc := New()
	loftBody, err := loftDoc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	extrudeDoc := New()
	extrudeBody, err := extrudeDoc.Extrude(s0, p0, Distance{D: units.Millimeters(wedgeHeight), Dir: Along})
	require.NoError(t, err)

	loftVol, err := loftBody.Volume()
	require.NoError(t, err)
	extVol, err := extrudeBody.Volume()
	require.NoError(t, err)
	gap := math.Abs(loftVol.Value.Base() - extVol.Value.Base())
	allowed := loftVol.Bound.Base() + extVol.Bound.Base()
	t.Logf("Volume oracle: loft=%.10g+/-%.3e extrude=%.10g+/-%.3e gap=%.3e allowed=%.3e",
		loftVol.Value.Base(), loftVol.Bound.Base(), extVol.Value.Base(), extVol.Bound.Base(), gap, allowed)
	require.LessOrEqual(t, gap, allowed, "the loft and extrude volume intervals must overlap")

	loftCen, err := loftBody.Centroid()
	require.NoError(t, err)
	extCen, err := extrudeBody.Centroid()
	require.NoError(t, err)
	dist := loftCen.Value.Sub(extCen.Value).Len()
	allowedC := loftCen.Bound.Base() + extCen.Bound.Base()
	t.Logf("Centroid oracle: loft=%v+/-%.3e extrude=%v+/-%.3e dist=%.3e allowed=%.3e",
		loftCen.Value, loftCen.Bound.Base(), extCen.Value, extCen.Bound.Base(), dist, allowedC)
	require.LessOrEqual(t, dist, allowedC, "the loft and extrude centroid intervals must overlap")
}

// loftBodyBindingRatio replays verify.go's own scalar/bounded tolerance gate
// (bodyToleranceInputs, scalarToleranceRef, boundedToleranceRef) over one
// body's own four readings and returns the WORST (largest) ratio — the
// reading that decides Sound/Suspect — and its name, so a test can assert
// the achieved margin as a number rather than merely reading Verify's own
// verdict.
func loftBodyBindingRatio(t *testing.T, ctx context.Context, body *Body) (float64, string) {
	t.Helper()
	area, err := body.Area()
	require.NoError(t, err)
	bounds, err := body.Bounds()
	require.NoError(t, err)
	vol, err := body.Volume()
	require.NoError(t, err)
	cen, err := body.Centroid()
	require.NoError(t, err)

	br := &BodyReport{Body: body, Area: area, Bounds: bounds, Volume: &vol, Centroid: &cen}
	in := &bodyToleranceInputs{ctx: ctx, report: br}

	type row struct {
		name  string
		ratio float64
	}
	var rows []row
	if _, ref, have := scalarToleranceRef(area, toleranceRel, in.areaReference); have {
		rows = append(rows, row{"Area", area.Bound.Base() / ref})
	}
	if _, ref, have := boundedToleranceRef(bounds.Bound.Base(), toleranceRel, in.diameterReference); have {
		rows = append(rows, row{"Bounds", bounds.Bound.Base() / ref})
	}
	if _, ref, have := scalarToleranceRef(vol, toleranceRel, in.volumeReference); have {
		rows = append(rows, row{"Volume", vol.Bound.Base() / ref})
	}
	if _, ref, have := boundedToleranceRef(cen.Bound.Base(), toleranceRel, in.diameterReference); have {
		rows = append(rows, row{"Centroid", cen.Bound.Base() / ref})
	}
	require.NotEmpty(t, rows, "at least one reading must form a usable tolerance reference")
	worst := rows[0]
	for _, r := range rows[1:] {
		if r.ratio > worst.ratio {
			worst = r
		}
	}
	return worst.ratio, worst.name
}

// TestLoftArcWedgeVerifiesSound is the ask's own Verify line: Sound at the
// default tolerance, with the achieved margin asserted numerically.
func TestLoftArcWedgeVerifiesSound(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Sound, report.Status, "diagnostics: %+v", report.Diagnostics)

	ratio, reading := loftBodyBindingRatio(t, t.Context(), body)
	require.Greater(t, ratio, 0.0, "the binding reading must have formed a usable reference")
	margin := toleranceRel / ratio
	t.Logf("A10a wedge Verify margin: binding=%s ratio=%.6g margin=%.3gx", reading, ratio, margin)
	require.Greater(t, margin, 1.0, "the achieved margin must exceed 1x for a Sound verdict")
	// Centroid is the binding reading (chordedBoundaryMomentAllow's own
	// widened-coordUpper composition, dominated by the loft's own true
	// 10mm height rather than any looseness left in the derivation) at a
	// measured ~1.34x — thinner than the hand-estimated calibration's own
	// ~2.39x (that estimate approximated chordedBoundaryMomentAllow's
	// quotient-rule composition rather than running it, and never modeled
	// the seam leg at all), but genuinely Sound. Pinned with generous slack
	// since a fraction-of-a-ulp difference in composed bound arithmetic
	// between hosts must never flip this assertion (never a wall-clock or
	// exact-bit pin — CLAUDE.md's own host-portability rule).
	require.InEpsilon(t, 1.34, margin, 0.25,
		"the achieved margin at the shipped constant, pinned so a future change to the widening formula is caught")
}

// TestLoftArcWedgeReadingsApproximateWithPositiveBounds is the ask's own
// line: all four readings are Approximate with positive bounds; Volume is
// never Exact when any pair is circular.
func TestLoftArcWedgeReadingsApproximateWithPositiveBounds(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	cen, err := body.Centroid()
	require.NoError(t, err)
	bnd, err := body.Bounds()
	require.NoError(t, err)
	ar, err := body.Area()
	require.NoError(t, err)

	require.Equal(t, Approximate, vol.Exactness, "Volume must never be Exact when any pair is circular")
	require.Equal(t, Approximate, cen.Exactness)
	require.Equal(t, Approximate, bnd.Exactness)
	require.Equal(t, Approximate, ar.Exactness)
	require.Greater(t, vol.Bound.Base(), 0.0)
	require.Greater(t, cen.Bound.Base(), 0.0)
	require.Greater(t, bnd.Bound.Base(), 0.0)
	require.Greater(t, ar.Bound.Base(), 0.0)
}

// arcWedgeLoopStraddling is a two-line-plus-one-arc wedge loop about the
// origin, centered on angle 0 with a total sweep of 2*halfSweep — chosen,
// unlike arcWedgeLoop's own axis-pinned Start, so the arc's own bulge past
// its chord lands STRICTLY INSIDE the swept angular range rather than at
// either endpoint. That distinction is why TestLoftArcWedgeBoxSoundness
// needs this shape rather than the A10a reference wedge (Start at angle 0,
// End at angle 90): a sub-90-degree circular arc is monotone in both cos and
// sin, so its own axis-aligned bounding box is realized EXACTLY at its two
// endpoints — held, recorded vertices, never displaced — and a box taken
// over that fixture would contain a dense arc sample with ZERO widening,
// making a soundness check against it vacuous whatever Bounds.Bound
// actually is. Straddling angle 0 puts the TRUE x-maximum (cos 0 = 1) at an
// INTERIOR, computed station instead, so the box only contains it when
// Bounds.Bound is wide enough to reach past the chord polygon's own
// (strictly smaller) x-extreme.
func arcWedgeLoopStraddling(radius, halfSweep float64) LoopRecord {
	origin := pt(0, 0)
	start := pt(radius*math.Cos(-halfSweep), radius*math.Sin(-halfSweep))
	end := pt(radius*math.Cos(halfSweep), radius*math.Sin(halfSweep))
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: origin, End: start, TStart: 0, TEnd: 1},
		ArcSeg{Center: origin, Start: start, End: end, TStart: 0, TEnd: 1},
		LineSeg{Start: end, End: origin, TStart: 0, TEnd: 1},
	}}
}

// TestLoftArcWedgeBoxSoundness is the ask's own box-soundness line: the
// Bounds box widened by its own Bound must contain a dense sample of BOTH
// true recorded arcs lifted through their planes. The report's own final
// message records the observed RED result of temporarily reverting
// bounds()'s Bound to bare delta and re-running this test, per the ask's
// own instruction — that revert is not part of the shipped diff.
func TestLoftArcWedgeBoxSoundness(t *testing.T) {
	const radius = 5.0
	const halfSweep = math.Pi / 6 // 30 degrees either side of angle 0: 60 degrees total
	p := ProfileRecord{Outer: arcWedgeLoopStraddling(radius, halfSweep)}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	budget := newWorkBudget(t.Context())
	body, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)

	bnd, err := body.Bounds()
	require.NoError(t, err)
	b := bnd.Bound.Base()
	require.Greater(t, b, 0.0, "a chorded curved loft's Bounds must carry a positive Bound")

	// The true x-maximum (radius, at theta=0) sits strictly inside the
	// sweep and past the chord polygon's own x-extreme: confirm the
	// fixture actually exercises the widening this test means to check,
	// rather than silently degrading into the vacuous case
	// arcWedgeLoopStraddling's own doc comment names.
	require.Greater(t, radius, bnd.Max.X, "the fixture must place the true x-extreme PAST the held polygon's own for this test to be non-vacuous")

	const samples = 4000
	for k := 0; k <= samples; k++ {
		theta := -halfSweep + 2*halfSweep*float64(k)/float64(samples)
		x, y := radius*math.Cos(theta), radius*math.Sin(theta)
		for _, z := range []float64{0, 10} {
			require.GreaterOrEqualf(t, x, bnd.Min.X-b, "theta=%v z=%v x=%v", theta, z, x)
			require.LessOrEqualf(t, x, bnd.Max.X+b, "theta=%v z=%v x=%v", theta, z, x)
			require.GreaterOrEqualf(t, y, bnd.Min.Y-b, "theta=%v z=%v y=%v", theta, z, y)
			require.LessOrEqualf(t, y, bnd.Max.Y+b, "theta=%v z=%v y=%v", theta, z, y)
			require.GreaterOrEqualf(t, z, bnd.Min.Z-b, "theta=%v z=%v", theta, z)
			require.LessOrEqualf(t, z, bnd.Max.Z+b, "theta=%v z=%v", theta, z)
		}
	}
}

// --- refusals ---

// TestLoftArcToFitSplineStillRefusesS3 is the ask's own line: an
// arc-to-fit-spline pair still refuses S3 — a circular walk and a free-form
// walk are never the same admitted kind.
func TestLoftArcToFitSplineStillRefusesS3(t *testing.T) {
	p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(ArcSeg{
		Center: pt(0.5, -1), Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1,
	})}
	p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(FitSplineSeg{
		Fit:    []Point2{pt(0, 0), pt(0.3, 0.2), pt(0.6, -0.1), pt(1, 0)},
		TStart: 0, TEnd: 1,
	})}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3: an arc-to-fit-spline pair is a mixed kind")
}

// TestLoftCircleSegOppositeCCWRefusesStructuralGateNotS7 is the ask's own
// line: two paired CircleSegs with opposite CCW refuse at the STRUCTURAL
// gate (loftSameKindGate), never at S7 (the build-time crossing audit) —
// asserted on the SENTINEL, which is what keeps the two refusals
// distinguishable: ErrUnsupported here, never ErrDegenerate (S7's own).
func TestLoftCircleSegOppositeCCWRefusesStructuralGateNotS7(t *testing.T) {
	ccw := CircleSeg{Center: pt(0.5, 0.5), Radius: units.Millimeters(0.5), CCW: true, TStart: 0, TEnd: 1}
	cw := CircleSeg{Center: pt(0.5, 0.5), Radius: units.Millimeters(0.5), CCW: false, TStart: 1, TEnd: 0}
	p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(ccw)}
	p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(cw)}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported, "the CCW-disagreement gate's own sentinel")
	require.False(t, errors.Is(err, ErrDegenerate), "must NOT be S7's own sentinel (ErrDegenerate)")
	require.Contains(t, err.Error(), "opposite directions")
}

// --- the m=1 edge case ---

// arcWedgeLoop is a two-line-plus-one-arc wedge loop about the origin, at
// the given radius and sweep (radians), Start pinned at angle 0 (so a
// natural, untrimmed ArcSeg endpoint reads a proven-zero rounding —
// circularStationChain's own doc comment).
func arcWedgeLoop(radius, sweep float64) LoopRecord {
	origin := pt(0, 0)
	start := pt(radius, 0)
	end := pt(radius*math.Cos(sweep), radius*math.Sin(sweep))
	return LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: origin, End: start, TStart: 0, TEnd: 1},
		ArcSeg{Center: origin, Start: start, End: end, TStart: 0, TEnd: 1},
		LineSeg{Start: end, End: origin, TStart: 0, TEnd: 1},
	}}
}

// TestLoftArcPairM1PublishesZeroDeltaWithPositiveSectionDelta is the ask's
// own m=1 line: a curved pair chorded at ONE station (no interior computed
// station at all — chordCount picks m=1 for a sweep tiny enough that even
// one chord meets the target) publishes delta == 0 and an UNSHRUNK
// bodyGateDiameter reference, with sectionDelta > 0 (the chord still
// departs from the arc it denotes).
func TestLoftArcPairM1PublishesZeroDeltaWithPositiveSectionDelta(t *testing.T) {
	const radius = 5.0
	const tinySweep = 1e-4 // sagitta ~= r*sweep^2/8 ~= 6.25e-10, far under any target
	p := ProfileRecord{Outer: arcWedgeLoop(radius, tinySweep)}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	budget := newWorkBudget(t.Context())
	body, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)

	loaded, ok := body.payload.(loftPayload)
	require.True(t, ok)
	require.Equal(t, 0.0, loaded.delta, "m=1: no interior computed station, so delta must be exactly 0")
	require.Greater(t, loaded.sectionDelta, 0.0, "m=1: the single chord still departs from the arc it denotes")

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	held, ok, err := pointSetDiameterContext(t.Context(), loaded.verts)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, held, d, "delta == 0 takes the unshrunk fast path: the shared reader's own answer, unchanged")
}

// --- placement: both displacements, and sectionDelta invariant under motion ---

// smallSweepWedgeSketch builds a two-line-plus-one-arc wedge (radius 5, a
// 30-degree sweep — about 21 stations at the shipped loftChordFraction, per
// a10-plan.md's own Fixture sizing note: station count scales with sweep,
// so a small-sweep arc keeps the repeated-PlacedCopy build cheap) on plane.
func smallSweepWedgeSketch(t *testing.T, w *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const sweep = math.Pi / 6 // 30 degrees
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	px := s.CreatePoint(wedgeRadius, 0)
	py := s.CreatePoint(wedgeRadius*math.Cos(sweep), wedgeRadius*math.Sin(sweep))
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestLoftPlacedArcWedgeCarriesBothDisplacements is the ask's own line: a
// PLACED arc loft carries both delta and sectionDelta.
func TestLoftPlacedArcWedgeCarriesBothDisplacements(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := smallSweepWedgeSketch(t, w, base)
	s1, p1 := smallSweepWedgeSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	moved, err := r3.Translation(r3.NewVec(3, -2, 1))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(moved)
	require.NoError(t, err)

	loaded, ok := placed.payload.(loftPayload)
	require.True(t, ok)
	require.Greater(t, loaded.delta, 0.0, "a placed body's delta must be positive")
	require.Greater(t, loaded.sectionDelta, 0.0, "a curved pair's sectionDelta must be positive whether or not the body is placed")
}

// TestLoftPlacedCopyTenTimesSectionDeltaUnchanged is the ask's own line: ten
// successive PlacedCopy motions leave sectionDelta UNCHANGED, proving it is
// a record property and not an accumulating one.
func TestLoftPlacedCopyTenTimesSectionDeltaUnchanged(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := smallSweepWedgeSketch(t, w, base)
	s1, p1 := smallSweepWedgeSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	loaded0, ok := body.payload.(loftPayload)
	require.True(t, ok)
	want := loaded0.sectionDelta
	require.Greater(t, want, 0.0)

	for i := range 10 {
		motion, err := r3.Translation(r3.NewVec(float64(i+1), 0, 0))
		require.NoError(t, err)
		body, err = body.PlacedCopy(motion)
		require.NoError(t, err)
		loaded, ok := body.payload.(loftPayload)
		require.True(t, ok)
		require.Equal(t, want, loaded.sectionDelta, "sectionDelta must not drift across repeated placements (iteration %d)", i)
	}
}
