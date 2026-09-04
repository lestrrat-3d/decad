package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBodyGateDiameterCancellation(t *testing.T) {
	doc := New()
	// An analytic prism, not a faceted body: bodyGateDiameter builds its carrier
	// model, which is the path that must observe cancellation.
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 5)

	// A live context returns the diameter exactly, with no error — the path a
	// valid caller observes, byte-identical to before the fix.
	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, d)

	// A cancelled context is observed during the build rather than after the
	// whole carrier model finishes (the non-cancellable newBodyGeom gap).
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err = bodyGateDiameter(ctx, body)
	require.ErrorIs(t, err, context.Canceled)
}

// requireCertifiedDiameter judges a published gate diameter against the EXACT
// squared distance of the pair it is meant to report, in math/big and never in
// float64: the published value's own square must not exceed that square (the
// certification docs/verification-design.md §3 requires), and the next float
// UP must exceed it (so the reading is the tightest float64 carrying that
// certification, not an arbitrary smaller number).
func requireCertifiedDiameter(t *testing.T, published float64, exactSquare *big.Rat) {
	t.Helper()
	square := new(big.Rat).SetFloat64(published)
	require.NotNil(t, square)
	square.Mul(square, square)
	require.LessOrEqual(t, square.Cmp(exactSquare), 0,
		`a published gate diameter must sit at or below the exact witness maximum`)

	up := new(big.Rat).SetFloat64(math.Nextafter(published, math.Inf(1)))
	require.NotNil(t, up)
	up.Mul(up, up)
	require.Positive(t, up.Cmp(exactSquare),
		`the certified reading must be the tightest float64 at or below the exact maximum`)
}

// The shared witness-maximum reader publishes a DOWNWARD-rounded pair maximum,
// so an ordinary analytic body read through bodyGateDiameter's exact carrier
// arm never reports a diameter above its own. This test demonstrates the
// hazard the reader guards against using naiveNorm (bounds_internal_test.go),
// never r3.Vec.Len itself: naiveNorm's own comment explains why this file
// states its reference arithmetic itself rather than asserting on r3's, so
// the fixture below discriminates on every architecture regardless of r3's
// own implementation. naiveNorm's sum is exact whenever it fits under
// 2^53 — so a small box's farthest pair (its own squared distance well under
// that bound) can never overshoot this way, however cleanly its distance
// factors: an exhaustive search over box dimensions up to several hundred
// found no overshoot at that scale. The fixture below instead scales the
// primitive quadruple 4² + 8² + 19² = 21² (16 + 64 + 361 = 441) by
// k = 23,726,573, landing each leg past 94,906,266 (~2^26.5) — the magnitude
// at which squaring a float64 first loses bits — while the exact distance
// stays 21k = 498,258,033, still exactly representable: naiveNorm of that
// same farthest pair overshoots it once the intermediate squares round.
func TestBodyGateDiameterExactCarrierArmNeverOverstates(t *testing.T) {
	const (
		bx, by, bh    = 94_906_292.0, 189_812_584.0, 450_804_887.0
		exactDiameter = 498_258_033.0
	)
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, bx, by, bh)

	require.Greater(t, naiveNorm(r3.NewVec(bx, by, 0).Sub(r3.NewVec(0, 0, bh))), exactDiameter,
		`naiveNorm of the winning pair lands above the exact distance, which is why the reader may not publish it`)

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, exactDiameter, d, `sqrt(248261067449029089) is representable, so the certified reading is 498258033 exactly`)
	requireCertifiedDiameter(t, d, big.NewRat(248_261_067_449_029_089, 1))
}

// A prismPayload carrying BOTH displacements still earns a gate reference
// (docs/prism-boolean-design.md §12's "Verify's structural/tolerance gates"
// row): the exact carrier model refuses it, so gateWitnessPrism hands
// fallbackGateDiameter the body's own recorded section beside the displacement
// each witness carries. This test pins what that arm proves — the charge is
// twice the SUM of the section and axial displacements, and the value it is
// subtracted from is the shared reader's exact-rational pair distance, not
// naiveNorm's own scan.
//
// The same scaled 4-8-19-21 box (TestBodyGateDiameterExactCarrierArmNeverOverstates's
// own comment derives it) is the fixture, so the exact witness maximum is
// 498,258,033 while naiveNorm of that pair reads 498258033.0000000596 on
// every architecture (see naiveNorm's own comment for why this file states
// that arithmetic itself rather than asserting on r3's). Both displacements
// are 0.125, so twice their sum is 0.5 and the exact shrunken value is exactly
// 498,258,032.5 — every quantity here is representable, and the two routes
// land on opposite sides of it. At this magnitude a float64's own spacing
// near the shrunken value is around 6e-8, so the InDelta tolerance below is
// loosened from a small-box fixture's 1e-12 to 1e-4 — still three orders of
// magnitude tighter than the 0.125 gap a coarse envelope would leave, but
// wide enough to clear that float spacing.
func TestBodyGateDiameterDisplacedPrismShrinksByTwiceTheSum(t *testing.T) {
	const (
		bx, by, bh    = 94_906_292.0, 189_812_584.0, 450_804_887.0
		exactDiameter = 498_258_033.0
	)
	doc := New()
	pp, isPrism := internalBoxBody(t, doc, 0, 0, bx, by, bh).payload.(prismPayload)
	require.True(t, isPrism)

	const delta = 0.125
	pp.sectionDelta = delta
	pp.z1Delta = delta
	require.InDelta(t, delta, pp.axialDelta(), 0)

	witness, displacement, ok := gateWitnessPrism(pp)
	require.True(t, ok, `a displaced prism earns a witness prism instead of losing its gate reference`)
	require.Zero(t, witness.sectionDelta, `the witness prism's own section is read as held, never as displaced`)
	require.Equal(t, absSumUpper(delta, pp.axialDelta()), displacement,
		`the charged displacement is the SUM of the section and axial terms, since they are perpendicular`)

	d, ok, err := bodyGateDiameter(t.Context(), &Body{payload: pp})
	require.NoError(t, err)
	require.True(t, ok, `Verify forms a tolerance-gate reference for a displaced prism`)

	require.Less(t, d, exactDiameter-2*(delta+delta),
		`the reference sits at or below the exact 498258033 - 2*(sectionDelta + axialDelta), and downRound puts it strictly below`)
	require.InDelta(t, exactDiameter-2*(delta+delta), d, 1e-4,
		`the shrink is the two displacements themselves, not a coarse envelope`)
	require.Less(t, d, exactDiameter-2*delta-delta,
		`a shrink of twice the section term PLUS the axial term would leave the reference above 498258032.625`)

	// naiveNorm's own scan would publish a reference at or above the exact
	// shrunken value, which is the loosening direction §3 forbids. The
	// published reading is proven below it.
	scan := naiveNorm(r3.NewVec(bx, by, 0).Sub(r3.NewVec(0, 0, bh)))
	require.Greater(t, scan, exactDiameter)
	require.GreaterOrEqual(t, downRound(scan-2*displacement), exactDiameter-2*(delta+delta))
	require.Less(t, d, downRound(scan-2*displacement),
		`the shrink is applied to the exact-rational pair distance, never to naiveNorm's own scan`)
}

// This file pins docs/verification-design.md's free-form tolerance-gate
// diameter arm: freeformSectionGateDiameter, tried by
// bodyGateDiameter ahead of fallbackGateDiameter whenever the exact carrier
// model refuses a prismPayload. Every fixture is a synthetic prismPayload
// built directly — the pattern prism_boolean_internal_test.go already uses —
// since no evaluator path builds a live free-form-walled prismPayload yet
// (docs/spline-design.md §10 P4b's side-face build is a later increment).

// freeformTriangleProfile is a degree-1 (piecewise-linear) unit-weight NURBS
// curve through (0,0), (5,4), (10,0), closed by a chord back to (0,0). A
// degree-1 NURBS reproduces its own Control array exactly at every knot — no
// smoothing arithmetic sits between a recorded coordinate and its span joint
// — so this fixture's witness points are hand-verifiable, bit for bit.
func freeformTriangleProfile() ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		NURBSSeg{
			Degree:  1,
			Control: []Point2{{U: 0, V: 0}, {U: 5, V: 4}, {U: 10, V: 0}},
			Weights: []float64{1, 1, 1},
			Knots:   []float64{0, 0, 0.5, 1, 1},
			TStart:  0, TEnd: 1,
		},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
}

func freeformTrianglePayload(t *testing.T, z0, z1 float64) prismPayload {
	t.Helper()
	return prismPayload{
		profile: freeformTriangleProfile(),
		frame:   identityFrame(t),
		z0:      z0, z1: z1,
		xform: r3.Identity(),
	}
}

// sCurveFreeformProfile is a single cubic SplineSeg (one raw Bézier span, its
// four control points read verbatim) whose control polygon zigzags — (0,0),
// (0,20), (10,-20), (10,0) — closed by a chord back to (0,0). The curve
// interpolates only its two endpoints; its interior sweeps well away from the
// chord between them, so a witness set built from span endpoints alone
// understates the section's true extent.
func sCurveFreeformProfile() ProfileRecord {
	control := []Point2{{U: 0, V: 0}, {U: 0, V: 20}, {U: 10, V: -20}, {U: 10, V: 0}}
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		SplineSeg{Control: control, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
}

// denseFreeformCapPoints is an independent oracle: it samples every free-form
// walk densely through evalSpans (spline_bezier_internal_test.go's own
// de Casteljau, never freeformSectionGateDiameter's machinery) at both cap
// heights, so a diameter read off its output is a dense-sample approximation
// of the section's TRUE diameter, never the arm's own witness-only reading.
func denseFreeformCapPoints(t *testing.T, pp prismPayload, samples int) []r3.Vec {
	t.Helper()
	var pts []r3.Vec
	work := newFreeformWork()
	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			require.NoError(t, err)
			if w.kind != walkFreeform {
				pts = append(pts,
					pp.point(w.startU, w.startV, pp.z0), pp.point(w.startU, w.startV, pp.z1),
					pp.point(w.endU, w.endV, pp.z0), pp.point(w.endU, w.endV, pp.z1),
				)
				continue
			}
			floatSpans := floatBezierSpansOf(w.spans)
			for i := 0; i <= samples; i++ {
				at := float64(i) / float64(samples)
				u, v := evalFloatBezierSpans(floatSpans, at)
				pts = append(pts, pp.point(u, v, pp.z0), pp.point(u, v, pp.z1))
			}
		}
	}
	return pts
}

// 1. The arm answers, with a value equal to the hand-computed maximum over
// the section's own analytic vertex and free-form span endpoints at both cap
// heights: the six witnesses are (0,0,0), (0,0,10), (5,4,0), (5,4,10),
// (10,0,0), (10,0,10), and the farthest pair — (0,0,0) to (10,0,10), or its
// mirror — is sqrt(200) apart.
func TestBodyGateDiameterFreeformArmAnswersHandComputedMaximum(t *testing.T) {
	pp := freeformTrianglePayload(t, 0, 10)
	body := &Body{payload: pp}

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok, "a free-form-walled prismPayload must read a diameter through its own arm")
	require.InDelta(t, math.Sqrt(200), d, 1e-9)

	// sqrt(200) is not representable, so the arm's own certification is
	// decided against the exact 200 in math/big: the published value squares
	// to at most 200, and the next float up squares past it.
	requireCertifiedDiameter(t, d, big.NewRat(200, 1))
	require.Greater(t, math.Sqrt(200), d,
		`the nearest float64 to sqrt(200) sits above it, so the certified reading is the one below`)
}

// 2. The arm can only understate the true diameter, never overstate it: the
// S-curve fixture's interior sweeps away from the chord between its own two
// span endpoints, so a dense independent sample of the actual curve finds a
// pair farther apart than the arm's own witness-only reading.
func TestBodyGateDiameterFreeformArmUnderstatesNeverOverstates(t *testing.T) {
	pp := prismPayload{profile: sCurveFreeformProfile(), frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity()}
	body := &Body{payload: pp}

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, d)

	trueD, denseOK := pointSetDiameter(denseFreeformCapPoints(t, pp, 4000))
	require.True(t, denseOK)
	require.Less(t, d, trueD,
		"a span-endpoint-only witness set must understate a curve that bulges past them, never overstate it")
}

// 3. The arm is not a box diagonal: on the same bulging S-curve section, the
// reported diameter sits below the section's own computed bounding-box
// diagonal (prismBoundsContext, P4a's already-landed free-form Box).
func TestBodyGateDiameterFreeformArmBelowBoxDiagonal(t *testing.T) {
	pp := prismPayload{profile: sCurveFreeformProfile(), frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity()}
	body := &Body{payload: pp}

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)

	box, err := prismBoundsContext(t.Context(), pp, newFreeformWork(), nil)
	require.NoError(t, err)
	diagonal := box.Max.Sub(box.Min).Len()
	require.Less(t, d, diagonal, "the diameter arm must read tighter than the section's own bounding-box diagonal")
}

// 4. A FitSplineSeg reads the converted chain (Points), never the raw
// recorded Fit: a fixture whose last two Fit points coincide within 1e-12 —
// so geom.NewFitInterpolant collapses them, keeping the first — reports the
// identical diameter to the same section recorded without the duplicate.
func TestBodyGateDiameterFreeformArmFitSplineReadsConvertedChain(t *testing.T) {
	build := func(fit []Point2) prismPayload {
		return prismPayload{
			profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
				FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
			}}},
			frame: identityFrame(t), z0: 0, z1: 10, xform: r3.Identity(),
		}
	}
	deduped := build([]Point2{{U: 0, V: 0}, {U: 5, V: 4}, {U: 10, V: 0}, {U: 10 + 5e-13, V: 0}})
	clean := build([]Point2{{U: 0, V: 0}, {U: 5, V: 4}, {U: 10, V: 0}})

	dD, dOK, err := bodyGateDiameter(t.Context(), &Body{payload: deduped})
	require.NoError(t, err)
	require.True(t, dOK)

	cD, cOK, err := bodyGateDiameter(t.Context(), &Body{payload: clean})
	require.NoError(t, err)
	require.True(t, cOK)

	require.Equal(t, cD, dD,
		"the arm must read the converted chain's own Points, never the raw duplicate-carrying Fit")
}

// 5. Displacement shrinks the reported diameter, and a shrink that collapses
// to non-positive reports no diameter at all.
func TestBodyGateDiameterFreeformArmDisplacementShrinksDiameter(t *testing.T) {
	base := freeformTrianglePayload(t, 0, 10)
	baseD, ok, err := bodyGateDiameter(t.Context(), &Body{payload: base})
	require.NoError(t, err)
	require.True(t, ok)

	shrunk := base
	shrunk.z0Delta = 0.5
	shrunkD, ok, err := bodyGateDiameter(t.Context(), &Body{payload: shrunk})
	require.NoError(t, err)
	require.True(t, ok)
	require.Less(t, shrunkD, baseD)

	collapsed := base
	collapsed.z0Delta = baseD
	_, ok, err = bodyGateDiameter(t.Context(), &Body{payload: collapsed})
	require.NoError(t, err)
	require.False(t, ok, "a shrink that collapses to non-positive must report no diameter")
}

// Refusal: a revolvePayload has no arm here, free-form-walled or not — the
// arm bodyGateDiameter tries is keyed on prismPayload alone, so a free-form
// revolve section must still report no diameter, exactly as before this arm
// existed.
func TestBodyGateDiameterFreeformArmDoesNotWidenRevolvePayload(t *testing.T) {
	rp := revolvePayload{
		profile: freeformTriangleProfile(),
		frame:   identityFrame(t),
		ax:      axisFrame{dU: 1},
		phi0:    0, phi1: 2 * math.Pi, full: true,
		xform: r3.Identity(),
	}
	_, ok, err := bodyGateDiameter(t.Context(), &Body{payload: rp})
	require.NoError(t, err)
	require.False(t, ok, "the free-form arm must never answer for a non-prismPayload payload")
}

// Refusal: a curve this arm cannot certify — a free-form span whose control
// net has collapsed to a single point, R14 — must never publish a diameter.
func TestBodyGateDiameterFreeformArmDeclinesOnCollapsedSpan(t *testing.T) {
	collapsed := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		SplineSeg{
			Control: []Point2{{U: 5, V: 5}, {U: 5, V: 5}, {U: 5, V: 5}, {U: 5, V: 5}},
			TStart:  0, TEnd: 1,
		},
	}}}
	pp := prismPayload{profile: collapsed, frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity()}
	_, ok, err := bodyGateDiameter(t.Context(), &Body{payload: pp})
	require.NoError(t, err)
	require.False(t, ok, "a curve this arm cannot certify must never publish a diameter")
}

// Refusal: a withheld answer is NOT rescued downstream. Both refusal shapes —
// a free-form section the arm cannot certify (R14's collapsed control net) and
// a displacement shrink that collapses the reading to non-positive — reach
// fallbackGateDiameter, which has no arm for a prismPayload whose sectionDelta
// is zero, so the body ends with no gate diameter at all.
func TestBodyGateDiameterFreeformArmDeclineReachesFallbackWithNoArm(t *testing.T) {
	collapsedSpan := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			SplineSeg{
				Control: []Point2{{U: 5, V: 5}, {U: 5, V: 5}, {U: 5, V: 5}, {U: 5, V: 5}},
				TStart:  0, TEnd: 1,
			},
		}}},
		frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity(),
	}

	collapsedShrink := freeformTrianglePayload(t, 0, 10)
	baseD, ok, err := bodyGateDiameter(t.Context(), &Body{payload: collapsedShrink})
	require.NoError(t, err)
	require.True(t, ok)
	collapsedShrink.z0Delta = baseD

	for name, pp := range map[string]prismPayload{
		"uncertifiable curve": collapsedSpan,
		"collapsed shrink":    collapsedShrink,
	} {
		t.Run(name, func(t *testing.T) {
			_, armOK, err := freeformSectionGateDiameter(t.Context(), pp)
			require.NoError(t, err)
			require.False(t, armOK, "the arm must withhold on this path")

			body := &Body{payload: pp}
			_, fallbackOK, err := fallbackGateDiameter(newWorkBudget(t.Context()), body)
			require.NoError(t, err)
			require.False(t, fallbackOK,
				"the fallback a withheld answer falls through to has no arm for a zero-sectionDelta prism")

			_, ok, err := bodyGateDiameter(t.Context(), body)
			require.NoError(t, err)
			require.False(t, ok, "the body ends with no gate diameter at all")
		})
	}
}

// Refusal: walkEndBoundAllow (bounds.go) is the mechanism behind every
// witness's own displacement charge, and its own contract is that an
// underivable component reads +Inf, never a small number a build could
// silently spend in its place.
func TestWalkEndBoundAllowRefusesNonFiniteComponent(t *testing.T) {
	require.Equal(t, math.Inf(1), walkEndBoundAllow(walkEndBound{u: math.Inf(1), v: 0}))
	require.Equal(t, math.Inf(1), walkEndBoundAllow(walkEndBound{u: 0, v: math.Inf(1)}))

	got := walkEndBoundAllow(walkEndBound{u: 3, v: 4})
	require.Equal(t, radius3D(4), got, "the wider of the two components is the one radius3D reads")
}

// 6a. Regression: an analytic (non-free-form) box's diameter still reads off
// the exact carrier model unchanged — the new arm's own prismPayload type
// assertion is never reached when newBodyGeomBudget already answers ok=true.
func TestBodyGateDiameterFreeformArmSkipsAnalyticPrism(t *testing.T) {
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 5)

	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, math.Sqrt(10*10+10*10+5*5), d, 1e-9, "an axis-aligned box's diameter is its own space diagonal")
}

// 6b. Regression: a section-displaced analytic prism (sectionDelta != 0, the
// analytic Union/Cut/Intersect case) still reads gateWitnessPrism's own
// displaced-section arm through fallbackGateDiameter, bit for bit — the new
// free-form arm declines outright (no free-form segment present) and never
// changes this reading.
func TestBodyGateDiameterFreeformArmSkipsSectionDisplacedPrism(t *testing.T) {
	analytic := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 10}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 10}, End: Point2{U: 0, V: 10}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	pp := prismPayload{
		profile: analytic, frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity(),
		sectionDelta: 0.1,
	}

	armD, armOK, err := freeformSectionGateDiameter(t.Context(), pp)
	require.NoError(t, err)
	require.False(t, armOK, "no free-form segment is present, so the new arm must decline outright")
	require.Zero(t, armD)

	want, wantOK, err := fallbackGateDiameter(newWorkBudget(t.Context()), &Body{payload: pp})
	require.NoError(t, err)
	require.True(t, wantOK)

	got, ok, err := bodyGateDiameter(t.Context(), &Body{payload: pp})
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, want, got, "a section-displaced analytic prism's reading must be unchanged, bit for bit")
}

// freeformLargeProfile is the same free-form NURBS bump the triangle fixture
// records — so the arm answers — closed by a polyline of `lines` chords around
// the lower half of the circle of radius 5 centred at (5,0). It exists to make
// the arm's two phases differ in cost by orders of magnitude: the segment walk
// is linear in the segment count, while the witness maximum it feeds is
// quadratic in it (four witness points per segment), which is why the maximum
// has to poll the caller's context on its own account.
func freeformLargeProfile(lines int) ProfileRecord {
	segs := []CurveSegment{
		NURBSSeg{
			Degree:  1,
			Control: []Point2{{U: 0, V: 0}, {U: 5, V: 4}, {U: 10, V: 0}},
			Weights: []float64{1, 1, 1},
			Knots:   []float64{0, 0, 0.5, 1, 1},
			TStart:  0, TEnd: 1,
		},
	}
	at := func(i int) Point2 {
		if i == lines {
			return Point2{U: 0, V: 0}
		}
		th := math.Pi * float64(i) / float64(lines)
		return Point2{U: 5 + 5*math.Cos(th), V: -5 * math.Sin(th)}
	}
	for i := range lines {
		segs = append(segs, LineSeg{Start: at(i), End: at(i + 1), TStart: 0, TEnd: 1})
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

// cancelAfterPolls is a context whose cancellation lands at a CHOSEN poll: the
// first `pass` calls to Err report a live context, every later one reports
// cancellation. It makes "which phase observed the cancellation" a
// deterministic assertion instead of a race against a wall clock.
type cancelAfterPolls struct {
	//nolint:containedctx // a test double standing in for a context, not a struct carrying one through an API.
	context.Context

	pass  int
	polls int
}

func (c *cancelAfterPolls) Err() error {
	c.polls++
	if c.polls <= c.pass {
		return nil
	}
	return context.Canceled
}

// 7. Cancellation: the witness maximum is the arm's quadratic phase, and it
// polls the caller's context on its own account. Cancellation landing after
// the segment walk has finished — the one moment an unpolled maximum would sit
// through a multi-million-pair scan — is reported AS an error, never folded
// into the arm's structural "no arm" answer.
func TestBodyGateDiameterFreeformArmCancelsDuringWitnessMaximum(t *testing.T) {
	pp := prismPayload{
		profile: freeformLargeProfile(800),
		frame:   identityFrame(t),
		z0:      0, z1: 10,
		xform: r3.Identity(),
	}
	segments := len(pp.profile.Outer.Segments)

	// The fixture really does answer, so the cancelled run below cannot pass
	// by declining: its witnesses span (0,0,0) to (10,0,10), sqrt(200) apart.
	d, ok, err := freeformSectionGateDiameter(t.Context(), pp)
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, math.Sqrt(200), d, 1e-9)

	// One poll per segment, then the maximum's own first budget poll.
	ctx := &cancelAfterPolls{Context: t.Context(), pass: segments}
	d, ok, err = freeformSectionGateDiameter(ctx, pp)
	require.ErrorIs(t, err, context.Canceled, "cancellation must propagate as an error, not as (0, false, nil)")
	require.False(t, ok)
	require.Zero(t, d)
	require.Equal(t, segments+1, ctx.polls,
		"the witness maximum must abort at its first poll, not run its full pairwise scan")

	// The same answer reaches bodyGateDiameter's caller: a cancelled Verify
	// never reads this arm's cancellation as a body with no usable diameter.
	cancelled, stop := context.WithCancel(t.Context())
	stop()
	_, ok, err = bodyGateDiameter(cancelled, &Body{payload: pp})
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
