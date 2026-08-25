package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md Part 3 PR 9 Task 1c: an END-TO-END fixture for
// the wall's own ruled-versus-chord AREA excess (loft_moments.go's
// m.chorded.areaExcess, area()'s own composition line
// `bound = absSumUpper(bound, m.chorded.areaExcess, m.chorded.capAreaExcess)`)
// — the leg zeroing left the ENTIRE repository suite green before this file
// existed. TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap
// (loft_arc_pairs_internal_test.go) already proves computeLoftChordedAllow's
// own RETURNED areaExcess value is sound; it does NOT prove area() actually
// SPENDS it — a regression deleting that composition term, or swapping which
// field it reads, is invisible to a test that calls computeLoftChordedAllow
// directly and never goes through the real Body.Area(). Every test below
// goes through Document.Loft and Body.Area(), the real published path, for
// the BODY UNDER TEST. The dense REFERENCE each test converges against is
// computed separately, described below.
//
// TWO REVIEW FINDINGS fixed the first version of this file:
//
//  1. THE CONVERGENCE CRITERION MUST NOT READ THE BOUND UNDER TEST. An
//     earlier version gated convergence on `diff <= 1% * area.Bound.Base()`
//     — the very quantity a leg-zeroing corrupts. Verified by hand: zeroing
//     the wall leg shrinks the published Area.Bound, which TIGHTENS that
//     convergence gate, which can fail the SWEEP itself (the reference
//     "does not converge") before the enclosure assertion ever runs — a red
//     test for the wrong reason, and a green one on a shape whose own cap
//     term happens to mask the wall leg regardless (the untwisted wedge:
//     confirmed by hand that zeroing the wall leg still leaves it PASSING,
//     residual 5.6408e-3 against a shrunk bound of 1.1521e-2 — cap slack
//     alone covers it, the identical masking
//     TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap's own doc
//     comment already reports for the cone frustum). So convergence below
//     is judged entirely on the REFERENCE's own successive-refinement
//     agreement, at a FIXED relative tolerance untied to anything this file
//     is checking, and the untwisted-wedge fixture is kept only as a
//     baseline — TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference
//     and TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference are the
//     two that are actually load-bearing for the wall leg.
//  2. THE REFERENCE MUST NOT PAY THE PRODUCTION AUDIT. Routing every
//     reference count through Document.Loft made this file cost 323s (the
//     package suite was about 65s before it), because each hand-chorded
//     build pays loft_audit.go's O(F^2) exact-rational crossing-audit cost
//     for a manifold proof the REFERENCE never needed — it is a plain
//     surface-area sum, not a solid this file certifies watertight.
//     denseChordedArea below computes the identical Table B geometry
//     (loft_build.go's assembleLoft: the diagonal-split lower/upper
//     triangle pair per cell, both caps' own shoelace) directly, in plain
//     float64 arithmetic, with no sketch and no Document.Loft call at all —
//     which is also a cleaner independence for a reference to have, and
//     removes the audit cost entirely, so the sweep can run to a much
//     finer, genuinely converged count for a small fraction of the
//     original cost. The BODY UNDER TEST is unaffected: every test still
//     builds it through the real Document.Loft and reads its Area from the
//     real Body.Area().
//
// FALSIFICATION LEDGER (verified by hand during review: temporarily zero
// the named leg in loft_moments.go's `area()`, rerun the named test,
// confirm RED, restore). Each entry quotes the ACTUAL failing assertion
// line, not merely "RED":
//
//   - m.chorded.areaExcess (the wall leg) zeroed
//     (`bound = absSumUpper(bound, 0, m.chorded.capAreaExcess)`):
//     TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.002446194848033656" is not less than or equal to "0.00046084914520616235"`
//     (residual=2.4462e-3, shrunk bound=4.6085e-4) — on "the loft's own
//     Area must enclose the converged densely-chorded reference on a
//     wall-dominated shape", the enclosure assertion itself.
//     TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.031970386054553046" is not less than or equal to "0.006083634427563824"`
//     (residual=3.1970e-2, shrunk bound=6.0836e-3) — on the same enclosure
//     assertion, "...under a large in-plane shear".
//     TestLoftArcWedgeAreaBoundEnclosesConvergedReference (the untwisted,
//     radius-5 wedge) stays GREEN under this same leg-zeroing (residual
//     5.7333e-3 against a shrunk bound of 1.1521e-2) — its own cap term is
//     large enough, relative to its short wall, to mask a missing wall term
//     entirely; it is kept as a baseline sanity check, never as the wall
//     leg's own falsifier.
//   - m.chorded.capAreaExcess (the cap leg) zeroed
//     (`bound = absSumUpper(bound, m.chorded.areaExcess, 0)`):
//     TestLoftArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.0057332610270179885" is not less than or equal to "0.0038222448743720373"`
//     (residual=5.7333e-3, shrunk bound=3.8222e-3) — on "the loft's own Area
//     must enclose the converged densely-chorded reference", the enclosure
//     assertion. TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference
//     and TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference both
//     stay GREEN under this same leg-zeroing — their own wall term
//     dominates and masks a missing cap term, the complementary shape to
//     the wall-leg case above. The PRE-EXISTING
//     TestLoftConeFrustumWallAreaEnclosed (loft_arc_pairs_internal_test.go)
//     also stays GREEN under this same leg-zeroing (gap 5.9387e-2 against
//     an untouched bound of 5.9587e+2) — CONTRARY to what an earlier
//     version of that test's own doc comment claimed about VIOLATION 1: at
//     its own r0=20 that fixture's bound is dominated by other slack (the
//     wall summation's own sumSlop term at that scale) by four orders of
//     magnitude, so it is not, in fact, a working falsifier for this leg —
//     a finding surfaced here rather than papered over. This file's own
//     untwisted-wedge fixture is the one that actually falsifies the cap
//     leg.

// triArea3 is a plain float64 triangle-area formula in 3D — the same
// (1/2)|cross| form triAreaFloat (loft_arc_pairs_internal_test.go) already
// uses, restated here so this file has no dependency on that one beyond the
// shared package.
func triArea3(a, b, c r3.Vec) float64 {
	return 0.5 * b.Sub(a).Cross(c.Sub(a)).Len()
}

// shoelace2DAbs is the plain (unsigned) shoelace area of a closed 2D
// polygon, vertices in either winding order.
func shoelace2DAbs(ring [][2]float64) float64 {
	sum := 0.0
	n := len(ring)
	for i := range n {
		x0, y0 := ring[i][0], ring[i][1]
		x1, y1 := ring[(i+1)%n][0], ring[(i+1)%n][1]
		sum += x0*y1 - x1*y0
	}
	return math.Abs(sum) / 2
}

// denseChordedArea computes the SAME wall-plus-cap area a hand-chorded
// LineSeg-only wedge loft (chordedWedgeProfile's own outline: origin ->
// botPts[0] -> ... -> botPts[m] -> origin, bottom and top) would publish
// through the real Document.Loft/Body.Area() — loft_build.go's assembleLoft
// own Table B construction (the diagonal-split lower/upper triangle pair
// per wall cell, `{vj,vjn,wjn}` then `{vj,wjn,wj}`) and both caps' own
// shoelace area — computed DIRECTLY, with no sketch and no Document.Loft
// call, so a reference sweep pays none of the O(F^2) crossing-audit cost
// the production build needs for its own manifold proof but a plain area
// reference does not.
//
// botPts/topPts are each the m+1 arc points (or radius/shear variant)
// wedgeCirclePoints(m) and its twins return; the ORIGIN each hand-chorded
// profile also draws through is prepended here, exactly matching
// chordedWedgeProfile's own vertex order. The bottom plane is z=0 with the
// identity (u,v)->(x,y) lift wedgePlanes' own base plane uses; the top
// plane is z=height with the SAME U/V basis (an offset plane, never a
// rotation — wedgePlanes' own CreateOffsetPlane), shifted in-plane by
// (dx, dy) — 0 for the untwisted fixtures, nonzero for the sheared one.
func denseChordedArea(botPts, topPts [][2]float64, height, dx, dy float64) float64 {
	botRing := append([][2]float64{{0, 0}}, botPts...)
	topRing := append([][2]float64{{0, 0}}, topPts...)
	n := len(botRing)

	bot := func(p [2]float64) r3.Vec { return r3.NewVec(p[0], p[1], 0) }
	top := func(p [2]float64) r3.Vec { return r3.NewVec(p[0]+dx, p[1]+dy, height) }

	wall := 0.0
	for j := range n {
		jn := (j + 1) % n
		vLo, vHi := bot(botRing[j]), bot(botRing[jn])
		wLo, wHi := top(topRing[j]), top(topRing[jn])
		wall += triArea3(vLo, vHi, wHi) + triArea3(vLo, wHi, wLo)
	}

	return wall + shoelace2DAbs(botRing) + shoelace2DAbs(topRing)
}

// convergedDenseArea sweeps denseChordedArea at increasing station counts
// and returns the value at the finest count once two SUCCESSIVE counts
// agree to within a FIXED relative tolerance of the reference's own value —
// never of refBound, the quantity a test built on this helper is checking
// (finding 1 in this file's own header comment). Because denseChordedArea
// pays no audit cost, the sweep can run to genuine convergence cheaply: at
// convergeRelTol = 1e-9 every fixture in this file converges by m <= 16384,
// in milliseconds.
func convergedDenseArea(t *testing.T, botPtsAt, topPtsAt func(m int) [][2]float64, height, dx, dy float64) float64 {
	t.Helper()
	const convergeRelTol = 1e-9
	var prev float64
	havePrev := false
	for _, m := range []int{256, 512, 1024, 2048, 4096, 8192, 16384, 32768} {
		cur := denseChordedArea(botPtsAt(m), topPtsAt(m), height, dx, dy)
		if havePrev {
			diff := math.Abs(cur - prev)
			if diff <= convergeRelTol*math.Abs(cur) {
				t.Logf("dense reference converged at m=%d: area=%.15g diff-from-previous=%.6e (relative %.3e)", m, cur, diff, diff/math.Abs(cur))
				return cur
			}
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("dense-chorded area reference did not converge to within relative %.3g", convergeRelTol)
	return 0
}

// TestLoftArcWedgeAreaBoundEnclosesConvergedReference is the untwisted
// A10a-shaped baseline: the real arc-paired loft's own Area must enclose
// the densely-converged reference — not merely the closed-form
// quarter-cylinder value TestLoftArcWedgeBuildsAndMatchesClosedForm already
// checks for Volume, and not merely computeLoftChordedAllow's own RETURNED
// value (TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap) — this
// goes through the real Document.Loft and Body.Area() end to end. Per this
// file's own ledger, this shape's own cap term masks a zeroed wall leg; it
// stands as a baseline sanity check and as the cap leg's own falsifier
// (alongside the pre-existing TestLoftConeFrustumWallAreaEnclosed).
func TestLoftArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePoints(m) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, wedgeHeight, 0, 0)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference")
}

// wedgeCirclePointsR is wedgeCirclePoints' own radius-parametrized twin: the
// m+1 chord vertices on a quarter circle of the given radius, k=0..m at
// k*sweep/m.
func wedgeCirclePointsR(m int, radius float64) [][2]float64 {
	pts := make([][2]float64, m+1)
	for k := 0; k <= m; k++ {
		theta := wedgeSweep * float64(k) / float64(m)
		pts[k] = [2]float64{radius * math.Cos(theta), radius * math.Sin(theta)}
	}
	return pts
}

// wedgePlanesH is wedgePlanes' own height-parametrized twin: two parallel
// planes sharing one U/V basis, offset by height rather than the fixed
// package constant wedgeHeight.
func wedgePlanesH(t *testing.T, height float64) (*sketch.World, *sketch.Plane, *sketch.Plane) {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := w.CreateOffsetPlane(base, height)
	require.NoError(t, err)
	return w, base, top
}

// TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference is this
// file's own primary Task 1c fixture: radius 1, height 60. Wall area scales
// with height and cap area with radius^2, so this shape's own cap excess is
// two orders of magnitude smaller than its wall excess — the wall leg is
// LOAD-BEARING here, unlike the untwisted baseline or the big-radius cone
// frustum fixture (both of whose cap slack masks a zeroed wall term, this
// file's own ledger).
func TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	const radius, height = 1.0, 60.0
	w, base, top := wedgePlanesH(t, height)
	s0, p0 := wedgeArcSketchR(t, w, base, radius)
	s1, p1 := wedgeArcSketchR(t, w, top, radius)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePointsR(m, radius) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, height, 0, 0)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("tall-thin arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference on a wall-dominated shape")
}

// wedgeArcSketchShiftedR is wedgeArcSketchR's own sheared twin: the true
// (uncorded) quarter-arc outline of the given radius, translated in-plane by
// (dx, dy).
func wedgeArcSketchShiftedR(t *testing.T, w *sketch.World, plane *sketch.Plane, radius, dx, dy float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(dx, dy)
	s.Fix(origin)
	px := s.CreatePoint(radius+dx, dy)
	py := s.CreatePoint(dx, radius+dy)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference is this file's
// own shear fixture: radius 1, height 60 — the SAME wall-dominated shape as
// the tall-thin fixture above, so the wall leg stays load-bearing (a
// radius-5 shear alone left the cap and other bound terms with enough slack
// to mask a zeroed wall leg too, verified by hand and corrected here) — with
// the top section's own construction points ALSO translated by (3, -2),
// several times the wedge's own radius, against the bottom section: the
// case where the ruling tilts hardest against the section tangent
// (cellTwistAreaAllow's own twist leg, a10-plan.md's own wording for this
// task).
func TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	const radius, height = 1.0, 60.0
	const dx, dy = 3.0, -2.0
	w, base, top := wedgePlanesH(t, height)
	s0, p0 := wedgeArcSketchR(t, w, base, radius)
	s1, p1 := wedgeArcSketchShiftedR(t, w, top, radius, dx, dy)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePointsR(m, radius) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, height, dx, dy)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("sheared arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference under a large in-plane shear")
}
