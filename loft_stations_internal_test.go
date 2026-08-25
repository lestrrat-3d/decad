package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md Part 3 PR 5's own test file: loftCellStations (the
// station generator), loftCircularCellStations (the ARC arm), the S15 station
// cap and S16 one-sided collapsed-cell refusal, and loftPairings' own
// station-chain and sectionDelta expansion. S3 still refuses every non-LineSeg
// pairing (loft_build.go's own header comment states the same shape for the
// original PR 1a landing), so every circular-arm test below calls the
// generator or loftPairings DIRECTLY, never through Document.Loft.

// --- loftLineCellStations / the LineSeg arm ---

// TestLoftLineCellStationsIsUnchanged pins the LineSeg arm's own contract: one
// station per side, at the segment's own recorded start, zero sagitta — the
// exact shape a LineSeg pairing already had before this generator existed. An
// arbitrary target must not perturb it: a straight wall's own chord IS the
// recorded segment, so this arm never reads target at all.
func TestLoftLineCellStationsIsUnchanged(t *testing.T) {
	w0 := segmentWalk{kind: walkLine, startU: 1, startV: 2}
	w1 := segmentWalk{kind: walkLine, startU: 3, startV: 4}
	stations0, stations1, sagitta, err := loftCellStations(w0, w1, 123.0, nil, nil)
	require.NoError(t, err)
	require.Equal(t, []Point2{{U: 1, V: 2}}, stations0)
	require.Equal(t, []Point2{{U: 3, V: 4}}, stations1)
	require.Zero(t, sagitta)
}

// --- loftCircularCellStations / the ARC arm: station count ---

// TestLoftCircularCellStationsHandDerivedStationCount is the plan's own
// acceptance line: "a 90-degree radius-5 arc at the calibrated constant
// yields the expected station count, computed BY HAND in the test... derive
// it in the test rather than hard-coding a magic number, and assert against
// the derivation." The hand derivation below independently re-implements
// chordCount's own conservative walk-up — s(n) = r*sweep^2/(8*n^2) <= tol,
// tessellate.go's chordSagitta formula, never the exact 2r*sin^2(dtheta/4)
// PR 1's calibration measured — so it is checking chordCount's own algorithm
// against an independent transcription of it, not merely echoing chordCount's
// return value back at itself.
//
// wedgeArcEnvelope and wedgeRadius/wedgeSweep are
// loft_chord_calibration_internal_test.go's own PR 1 fixtures (same package,
// same reference wedge): a 90-degree radius-5 quarter arc, envelope read
// through the real profileCoordinateUpper.
func TestLoftCircularCellStationsHandDerivedStationCount(t *testing.T) {
	envelope := wedgeArcEnvelope(t)
	target := loftChordFraction * envelope

	handN := 1
	for {
		s := handChordSagittaConservative(wedgeRadius, wedgeSweep, handN)
		if s <= target {
			break
		}
		handN++
	}

	w := circularWalk(0, 0, wedgeRadius, 0, wedgeSweep, wedgeRadius, wedgeSweep)
	m, achieved, err := chordCount(w, target)
	require.NoError(t, err)
	require.LessOrEqual(t, achieved, target)
	require.Equal(t, handN, m, "chordCount's own walk-up must match this test's own independent transcription of the same conservative bound")

	// The chooser's own walk-up settles this reference wedge at 65 stations,
	// the count loft_chord_calibration_internal_test.go's loftChordFractionPinM
	// pins and measures every calibration margin at. It sits one station above
	// the sweep grid row the shipped constant was read off because the two
	// prove the sagitta differently: the sweep measures the EXACT
	// 2r*sin^2(dtheta/4), while the PRODUCTION chooser proves its own bound
	// through chordSagitta's outward-rounded, provably conservative
	// r*sweep^2/(8n^2) (chordSagitta's own doc comment states the (x/sin x)^2
	// gap this opens). At m=64 on this exact wedge the two straddle the target:
	// chordSagitta(5, pi/2, 64) = 3.7649553e-4 against a target of
	// 3.7649100e-4, over by about 4.53e-9, so the walk-up commits one further
	// station. The straddle is host-independent — it holds computing target
	// from an exact envelope of 10.0 as well as from the live
	// profileCoordinateUpper reading — and wedgePinStations asserts both sides
	// of it directly.
	//
	// The literal below is deliberate here, where every other site names
	// loftChordFractionPinM instead: this test exists to check the chooser
	// against an independent transcription, so naming the pin the chooser
	// itself settles would make the two ends of the cross-check the same
	// value.
	require.Equal(t, 65, m)
}

// handChordSagittaConservative is TestLoftCircularCellStationsHandDerivedStationCount's
// own transcription of chordSagitta's proven bound (tessellate.go), written
// independently (ordinary math.Pow rather than chordSagitta's own
// single-operation outward-rounding chain) so the two are a genuine
// cross-check rather than one call site echoing the other's arithmetic.
func handChordSagittaConservative(radius, sweep float64, n int) float64 {
	dtheta := sweep / float64(n)
	return radius * dtheta * dtheta / 8
}

// TestLoftChordCountShippedFractionOnReferenceWedge is this PR's own
// additional acceptance line: drive the REAL chordCount at the SHIPPED
// loftChordFraction constant, envelope read through the real
// profileCoordinateUpper, on the plan's own reference wedge — proving the
// production chooser, not a hand re-derivation of it. See
// TestLoftCircularCellStationsHandDerivedStationCount's own comment for the
// straddle at m=64 that puts this count at 65, and
// loft_chord_calibration_internal_test.go's loftChordFractionPinM for the
// margins measured at it.
func TestLoftChordCountShippedFractionOnReferenceWedge(t *testing.T) {
	envelope := wedgeArcEnvelope(t)
	target := loftChordFraction * envelope
	w := circularWalk(0, 0, wedgeRadius, 0, wedgeSweep, wedgeRadius, wedgeSweep)
	m, achieved, err := chordCount(w, target)
	require.NoError(t, err)
	require.LessOrEqual(t, achieved, target)
	t.Logf("chordCount at the shipped loftChordFraction on the reference wedge: m=%d (target=%.10g, achieved=%.10g)", m, target, achieved)
	require.Equal(t, loftChordFractionPinM, m, "the calibration pin must name the count the production chooser produces at the shipped constant; see this test's sibling for the derivation of the straddle at m=64")
}

// --- loftCircularCellStations: the shared-m rule ---

// TestLoftCircularCellStationsSharedMTakesMax pins a10-plan.md Q2's
// shared-station rule directly: two sides of genuinely different radius and
// sweep settle on different OWN station counts, and the cell's shared count
// is their max — never either side's own smaller count.
func TestLoftCircularCellStationsSharedMTakesMax(t *testing.T) {
	w0 := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2) // radius 5, 90 degrees
	w1 := circularWalk(0, 0, 2, 0, math.Pi/6, 2, math.Pi/6) // radius 2, 30 degrees
	const target = 1e-3

	m0, _, err := chordCount(w0, target)
	require.NoError(t, err)
	m1, _, err := chordCount(w1, target)
	require.NoError(t, err)
	require.NotEqual(t, m0, m1, "the fixture must need different station counts on its two sides for this test to exercise the max rule")
	want := max(m0, m1)

	stations0, stations1, sagittaUpper, err := loftCircularCellStations(w0, w1, target)
	require.NoError(t, err)
	require.Len(t, stations0, want)
	require.Len(t, stations1, want)
	require.LessOrEqual(t, sagittaUpper, target)

	// Each side's achieved sagitta AT THE SHARED m is at or below its OWN
	// target — checked over math/big.Rat, the exact rational value of
	// chordSagitta's own formula r*sweep^2/(8*m^2), with no outward rounding
	// on either side of the comparison, so the check is host-independent.
	ratTarget := new(big.Rat).SetFloat64(target)
	for _, w := range []segmentWalk{w0, w1} {
		sweep := math.Abs(w.th1 - w.th0)
		num := new(big.Rat).Mul(
			new(big.Rat).SetFloat64(w.radius),
			new(big.Rat).Mul(new(big.Rat).SetFloat64(sweep), new(big.Rat).SetFloat64(sweep)),
		)
		den := new(big.Rat).SetInt64(8 * int64(want) * int64(want))
		ratSagitta := new(big.Rat).Quo(num, den)
		require.LessOrEqual(t, ratSagitta.Cmp(ratTarget), 0,
			"radius=%v sweep=%v must meet its own target at the shared m=%d", w.radius, sweep, want)
	}
}

// --- loftCircularCellStations: the parameter-matched discharge ---

// TestLoftCircularArcSagittaIsTheUniformParameterMatchedBound pins the claim
// loftCircularCellStations' own doc comment makes: under uniform-angle
// parametrization (t_k = th0 + (k/m)*(th1-th0)), sup_s |arc(s) - chord(s)|
// over one chord EQUALS the sagitta 2r*sin^2(dtheta/4) exactly, the maximum
// always landing at s = 1/2 — proven here by dense sampling across a spread
// of sweeps from 0.5 to 359.5 degrees, never merely one case.
func TestLoftCircularArcSagittaIsTheUniformParameterMatchedBound(t *testing.T) {
	const r = 5.0
	const samples = 20_000
	for degSweep := 0.5; degSweep <= 359.5; degSweep += 15 {
		sweep := degSweep * math.Pi / 180
		cx0, cy0 := r, 0.0
		cx1, cy1 := r*math.Cos(sweep), r*math.Sin(sweep)
		maxDev, argMax := 0.0, 0.0
		for i := 0; i <= samples; i++ {
			s := float64(i) / float64(samples)
			theta := s * sweep
			ax, ay := r*math.Cos(theta), r*math.Sin(theta)
			chx, chy := cx0+s*(cx1-cx0), cy0+s*(cy1-cy0)
			dev := math.Hypot(ax-chx, ay-chy)
			if dev > maxDev {
				maxDev, argMax = dev, s
			}
		}
		sagitta := 2 * r * math.Pow(math.Sin(sweep/4), 2)
		require.InDelta(t, sagitta, maxDev, sagitta*1e-3+1e-12,
			"sweep=%v degrees: dense-sample departure must equal the sagitta", degSweep)
		require.InDelta(t, 0.5, argMax, 1.0/samples*2,
			"sweep=%v degrees: the maximum departure must land at the chord's own midpoint", degSweep)
	}
}

// --- S15: the station cap ---

// TestLoftCellStationsStationCapFiresBeforeAuditCeiling is S15: a target this
// fine on a radius-5 quarter arc demands far more than maxChordsPerWalk
// (2^14) chords — chordSagitta ~ r*sweep^2/(8n^2), so meeting it needs n well
// past the cap — and chordCount refuses outright rather than build past it
// (errTooManyChords, reused from tessellate.go per spline design Table R row
// R8).
//
// This necessarily precedes S8's own audit-budget ceiling: evalLoft calls
// loftPairings (which reaches this refusal through loftCellStations)
// strictly BEFORE assembleLoft ever builds a triangle and before
// loftCrossingAudit ever runs (loft_build.go's own evalLoft body). Had the
// cap not fired, the station count alone — already past 2^14 per side — would
// publish orders of magnitude more wall faces than S8's own
// maxFacetPairTestsPerCall ceiling binds at (roughly F=4000,
// loft_audit.go), so this fixture is exactly the one the plan names: one
// that would otherwise reach the audit ceiling.
func TestLoftCellStationsStationCapFiresBeforeAuditCeiling(t *testing.T) {
	w0 := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2)
	w1 := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2)
	_, _, _, err := loftCellStations(w0, w1, 1e-12, nil, nil) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, errTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "more than 16384 chords")
}

// --- S16: the one-sided collapsed cell (defensive) ---

// TestLoftCircularCellStationsRefusesOneSidedCollapse is S16, a defensive
// gate: a chord cell whose two stations coincide on exactly ONE of the two
// sections has no case in the uniform two-faces-per-cell wall topology
// assembleLoft builds. It is reachable only from a degenerate walk (radius 0
// here) built directly on the record — a real sketch authentication does not
// produce one (this PR's own risk register, mirroring spline_fit.go's dedup
// carve-out for the free-form arm's analogous case) — so this fixture is
// built directly on a hand-constructed segmentWalk, bypassing sketch
// entirely, exactly as the plan's own fallback names.
func TestLoftCircularCellStationsRefusesOneSidedCollapse(t *testing.T) {
	degenerate := circularWalk(0, 0, 0, 0, math.Pi/2, 0, math.Pi/2) // radius 0: every station is the same point
	normal := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2)
	_, _, _, err := loftCircularCellStations(degenerate, normal, 1e-3) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "collapses to one point on only one of the two sections")
}

// TestLoftCircularCellStationsSymmetricCollapseIsFine proves S16's own
// boundary: a cell collapsed on BOTH sides — the m=1 case a curved pair
// chorded at one station always is — is sound (loftPayload's own doc comment,
// docs/loft-design.md §12's m=1 case), never refused.
func TestLoftCircularCellStationsSymmetricCollapseIsFine(t *testing.T) {
	w0 := circularWalk(0, 0, 0, 0, math.Pi/2, 0, math.Pi/2)
	w1 := circularWalk(0, 0, 0, 0, math.Pi/2, 0, math.Pi/2)
	stations0, stations1, sagitta, err := loftCircularCellStations(w0, w1, 1e-3)
	require.NoError(t, err)
	require.Len(t, stations0, 1)
	require.Len(t, stations1, 1)
	require.Zero(t, sagitta)
}

// --- loftPairings: station-chain expansion and sectionDelta ---

// TestLoftPairingsSectionDeltaIsMaxNotSum is the plan's own acceptance line:
// a loop with two curved pairs of different curvature publishes sectionDelta
// as the LARGER cell's own measured sagitta, never their sum — a boundary
// point lies in exactly one cell. The two segment pairs are built directly on
// hand-constructed walks (loftPairings only reads each loop's own Segments
// SLICE LENGTH from p0, never their content, once walks are already
// resolved), each independently proven to reach a DIFFERENT achieved
// sagitta at the shared target so the max-versus-sum distinction is
// observable.
func TestLoftPairingsSectionDeltaIsMaxNotSum(t *testing.T) {
	const target = 1e-3

	wA0 := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2)
	wA1 := circularWalk(0, 0, 5, 0, math.Pi/2, 5, math.Pi/2)
	_, _, sagA, err := loftCellStations(wA0, wA1, target, nil, nil)
	require.NoError(t, err)

	wB0 := circularWalk(0, 0, 1, 0, math.Pi/6, 1, math.Pi/6)
	wB1 := circularWalk(0, 0, 1, 0, math.Pi/6, 1, math.Pi/6)
	_, _, sagB, err := loftCellStations(wB0, wB1, target, nil, nil)
	require.NoError(t, err)

	require.NotEqual(t, sagA, sagB, "the two segment pairs must reach different sagittas for this test to distinguish max from sum")

	p0 := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{LineSeg{}, LineSeg{}}}}
	walks0 := [][]segmentWalk{{wA0, wB0}}
	walks1 := [][]segmentWalk{{wA1, wB1}}

	pairs, sectionDelta, err := loftPairings(p0, []int{0}, walks0, walks1, target, nil, nil)
	require.NoError(t, err)
	require.NotEmpty(t, pairs)

	ratDelta := new(big.Rat).SetFloat64(sectionDelta)
	ratMax := new(big.Rat).SetFloat64(math.Max(sagA, sagB))
	ratSum := new(big.Rat).SetFloat64(sagA + sagB)
	require.Zero(t, ratDelta.Cmp(ratMax), "sectionDelta must equal the larger cell's own sagitta exactly")
	require.NotZero(t, ratDelta.Cmp(ratSum), "sectionDelta must never equal the sum of the two cells")
}

// TestLoftPairingsLineSegOnlyStationChainUnchanged is the plan's own
// acceptance line: a LineSeg-only pairing is bit-identical to today's — one
// station per segment, at its own recorded start, sectionDelta exactly zero,
// unaffected by an arbitrary target the LineSeg arm never reads.
func TestLoftPairingsLineSegOnlyStationChainUnchanged(t *testing.T) {
	p := unitSquareProfile()
	offsets := []int{0}
	walks := resolveLoftLoopWalks(t, p)
	pairs, sectionDelta, err := loftPairings(p, offsets, walks, walks, 999.0, nil, nil)
	require.NoError(t, err)
	require.Len(t, pairs, 1)
	require.Len(t, pairs[0].v, 4)
	require.Len(t, pairs[0].w, 4)
	for j := range 4 {
		require.Equal(t, Point2{U: walks[0][j].startU, V: walks[0][j].startV}, pairs[0].v[j])
		require.Equal(t, Point2{U: walks[0][j].startU, V: walks[0][j].startV}, pairs[0].w[j])
	}
	require.Zero(t, sectionDelta)
}

// --- loftChordTarget ---

// TestLoftChordTargetUsesTheAnalyticEnvelope pins the deliberate choice
// loftChordTarget's own doc comment states: it reads profileCoordinateUpper,
// which refuses a free-form boundary segment outright, rather than its
// non-refusing twin profileCoordinateEnvelope — costless today because every
// kind this evaluator admits into a pairing (LineSeg) is analytic.
func TestLoftChordTargetUsesTheAnalyticEnvelope(t *testing.T) {
	fit := FitSplineSeg{
		Fit:    []Point2{pt(0, 0), pt(1, 1), pt(2, 0), pt(3, 1), pt(4, 0)},
		TStart: 0, TEnd: 1,
	}
	loop := LoopRecord{Segments: []CurveSegment{fit, fit, fit}}
	p := ProfileRecord{Outer: loop}
	work := newFreeformWork()
	walks := make([]segmentWalk, 3)
	var err error
	for i := range walks {
		walks[i], err = walkOf(fit, work)
		require.NoError(t, err)
	}
	walks0 := [][]segmentWalk{walks}

	_, err = loftChordTarget(p, p, walks0, walks0)
	require.ErrorIs(t, err, ErrUnsupported)
}
