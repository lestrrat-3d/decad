package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md Part 3 PR 5's own test file: loftCellStations (the
// station generator), loftCircularCellStations (the ARC arm), the S15 station
// cap and S16 one-sided collapsed-cell refusal, and loftPairings' own
// station-chain and sectionDelta expansion. S3 still refuses every non-LineSeg
// pairing (loft_build.go's own header comment states the same shape for the
// original PR 1a landing), so every circular-arm test below calls the
// generator or loftPairings DIRECTLY, never through Document.Loft.

// --- shared fixtures ---

// arcFixture builds one RECORDED ArcSeg about the origin — radius r, running
// sweep radians from base angle base, trimmed to [tStart, tEnd] — together
// with the walk walkOf resolves for it. The circular arm reads BOTH: the
// stations come off the walk, while the certified per-cell sagitta comes off
// the record's own radius and swept-angle enclosures (loft_build.go's
// loftCertifiedSagittaUpper). A fixture that stated only a hand-built walk
// could not exercise the arm at all, since no record stands behind it.
func arcFixture(t *testing.T, r, base, sweep, tStart, tEnd float64) (ArcSeg, segmentWalk) {
	t.Helper()
	seg := ArcSeg{
		Center: pt(0, 0),
		Start:  pt(r*math.Cos(base), r*math.Sin(base)),
		End:    pt(r*math.Cos(base+sweep), r*math.Sin(base+sweep)),
		TStart: tStart,
		TEnd:   tEnd,
	}
	w, err := walkOf(seg, nil)
	require.NoError(t, err)
	return seg, w
}

// degenerateArcFixture is arcFixture's zero-radius case, written out because
// it cannot be reached through an angle: every point of a zero-radius arc is
// its own centre, so Start, End and Center are one coordinate and the record
// states no angle at all. A real sketch authentication never produces it
// (spline_fit.go's own dedup plays the analogous role for the free-form arm),
// so S16's fixtures build it on the record directly.
func degenerateArcFixture(t *testing.T) (ArcSeg, segmentWalk) {
	t.Helper()
	seg := ArcSeg{Center: pt(0, 0), Start: pt(0, 0), End: pt(0, 0), TStart: 0, TEnd: 1}
	w, err := walkOf(seg, nil)
	require.NoError(t, err)
	return seg, w
}

// --- loftLineCellStations / the LineSeg arm ---

// TestLoftLineCellStationsIsUnchanged pins the LineSeg arm's own contract: one
// station per side, at the segment's own recorded start, zero sagitta — the
// exact shape a LineSeg pairing already had before this generator existed. An
// arbitrary target must not perturb it: a straight wall's own chord IS the
// recorded segment, so this arm never reads target at all.
func TestLoftLineCellStationsIsUnchanged(t *testing.T) {
	w0 := segmentWalk{kind: walkLine, startU: 1, startV: 2}
	w1 := segmentWalk{kind: walkLine, startU: 3, startV: 4}
	stations0, stations1, sagitta, err := loftCellStations(w0, w1, LineSeg{}, LineSeg{}, 123.0, nil, nil)
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
// the derivation." The count in question is the SEED the joint walk-up starts
// from (docs/loft-design.md §5.1). The hand derivation below independently
// re-implements chordCount's own conservative walk-up — s(n) =
// r*sweep^2/(8*n^2) <= tol, tessellate.go's chordSagitta formula, never the
// exact 2r*sin^2(dtheta/4) PR 1's calibration measured — so it is checking
// chordCount's own algorithm against an independent transcription of it, not
// merely echoing chordCount's return value back at itself.
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

	// The seed for this reference wedge is 65 stations, and the joint walk-up
	// only increments, so 65 is also the count
	// loft_chord_calibration_internal_test.go's loftChordFractionPinM pins and
	// measures every calibration margin at. It sits one station above the sweep
	// grid row the shipped constant was read off because the two prove the
	// sagitta differently: the sweep measures the EXACT 2r*sin^2(dtheta/4),
	// while chordCount proves its own bound through chordSagitta's
	// outward-rounded, provably conservative r*sweep^2/(8n^2) (chordSagitta's
	// own doc comment states the (x/sin x)^2 gap this opens). At m=64 on this
	// exact wedge the two straddle the target: chordSagitta(5, pi/2, 64) =
	// 3.7649553e-4 against a target of 3.7649100e-4, over by about 4.53e-9, so
	// the seed walk-up commits one further station. The straddle is
	// host-independent — it holds computing target from an exact envelope of
	// 10.0 as well as from the live profileCoordinateUpper reading — and
	// wedgePinStations asserts both sides of it directly.
	//
	// The literal below is deliberate here, where every other site names
	// loftChordFractionPinM instead: this test exists to check chordCount
	// against an independent transcription, so naming the pin the generator
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

// TestLoftShippedFractionOnReferenceWedge is this PR's own additional
// acceptance line: drive the REAL generator at the SHIPPED loftChordFraction
// constant, envelope read through the real profileCoordinateUpper, on the
// plan's own reference wedge — proving the production station rule, not a hand
// re-derivation of it. See TestLoftCircularCellStationsHandDerivedStationCount's
// own comment for the seed straddle at m=64 that puts this count at 65, and
// loft_chord_calibration_internal_test.go's loftChordFractionPinM for the
// margins measured at it.
func TestLoftShippedFractionOnReferenceWedge(t *testing.T) {
	envelope := wedgeArcEnvelope(t)
	target := loftChordFraction * envelope
	seg, w := wedgeArcRecord(t)
	stations, _, sagitta, err := loftCircularCellStations(w, w, seg, seg, target)
	require.NoError(t, err)
	require.LessOrEqual(t, sagitta, target)
	t.Logf("the generator at the shipped loftChordFraction on the reference wedge: m=%d (target=%.10g, certified=%.10g)", len(stations), target, sagitta)
	require.Equal(t, loftChordFractionPinM, len(stations), "the calibration pin must name the count the production generator produces at the shipped constant; see this test's sibling for the derivation of the seed straddle at m=64")
}

// --- loftCircularCellStations: the joint walk-up ---

// TestLoftCircularCellStationsJointWalkUpSharesOneCount pins
// docs/loft-design.md §5.1's shared-station rule: two sides of genuinely
// different radius and sweep settle on different OWN seed counts, and the
// cell walks BOTH at one count that is at least their max and at which BOTH
// sides' CERTIFIED sagittae clear the target. Neither side is ever walked at
// its own smaller count.
func TestLoftCircularCellStationsJointWalkUpSharesOneCount(t *testing.T) {
	const target = 1e-3
	seg0, w0 := arcFixture(t, 5, 0, math.Pi/2, 0, 1) // radius 5, 90 degrees
	seg1, w1 := arcFixture(t, 2, 0, math.Pi/6, 0, 1) // radius 2, 30 degrees

	m0, _, err := chordCount(w0, target)
	require.NoError(t, err)
	m1, _, err := chordCount(w1, target)
	require.NoError(t, err)
	require.NotEqual(t, m0, m1, "the fixture must seed different station counts on its two sides for this test to exercise the shared-count rule")
	seed := max(m0, m1)

	stations0, stations1, sagittaUpper, err := loftCircularCellStations(w0, w1, seg0, seg1, target)
	require.NoError(t, err)
	require.Len(t, stations1, len(stations0), "both sides must be walked at ONE count")
	m := len(stations0)
	require.GreaterOrEqual(t, m, seed, "the joint walk-up starts at the larger of the two seeds and only ever increments")
	require.LessOrEqual(t, sagittaUpper, target)

	// Both sides clear the target at the SETTLED count, recomputed there —
	// never inferred from how either side's own seed compared. This is the
	// property the joint walk-up buys, and it is asserted against the same
	// certified reading the arm itself publishes.
	for i, seg := range []CurveSegment{seg0, seg1} {
		s := loftCertifiedSagittaUpper(seg, m)
		require.False(t, isNonFinite(s), "side %d's certified sagitta must be derivable at the settled count", i)
		require.LessOrEqual(t, s, target, "side %d must meet the target at the settled m=%d", i, m)
		require.LessOrEqual(t, s, sagittaUpper, "the cell publishes the larger of its two sides' certified sagittae")
	}
	require.True(t,
		loftCertifiedSagittaUpper(seg0, m) == sagittaUpper || loftCertifiedSagittaUpper(seg1, m) == sagittaUpper,
		"the published bound must BE one of the two sides' own certified readings, never a blend of them")
}

// TestLoftCircularCellStationsJointWalkUpOutrunsTheSeed is the joint walk-up's
// own reason for existing, shown on a fixture that needs it: an arc whose HELD
// sagitta already reads under the target at chordCount's own seed count, while
// the CERTIFIED sagitta over the record's own enclosures still does not. A
// max-of-seeds rule would stop at the seed and publish a bound the record does
// not support; the joint walk-up increments until the certified reading clears.
func TestLoftCircularCellStationsJointWalkUpOutrunsTheSeed(t *testing.T) {
	const m = 3
	seg, w, held, _ := shortfallArc(t, m)

	// The target IS the held reading at m: chordCount is exactly satisfied
	// there and seeds the walk at m, which is what makes the certified
	// reading's own verdict at m the only thing that can move the count.
	seed, ach, err := chordCount(w, held)
	require.NoError(t, err)
	require.Equal(t, m, seed, "the held chooser must be exactly satisfied at the fixture's own count")
	require.LessOrEqual(t, ach, held)

	require.Greater(t, loftCertifiedSagittaUpper(seg, seed), held,
		"the certified sagitta at the seed must still exceed the target, or this fixture proves nothing about the walk-up")

	stations0, stations1, sagittaUpper, err := loftCircularCellStations(w, w, seg, seg, held)
	require.NoError(t, err)
	require.Len(t, stations1, len(stations0))
	require.Greater(t, len(stations0), seed, "the joint walk-up must commit at least one station past the seed")
	require.LessOrEqual(t, sagittaUpper, held)
	require.Equal(t, loftCertifiedSagittaUpper(seg, len(stations0)), sagittaUpper)
}

// --- the published sagitta is certified, never the held chooser's ---

// TestLoftCircularSagittaIsCertifiedNotHeld is finding B's own regression: on
// a fixture from the audit's own family the arm's published sagitta is
// STRICTLY ABOVE the held float chordSagitta returns for the same walk and
// count, and a certified LOWER bound on the true sagitta is above it too — so
// the held value bounds nothing there. Publishing the held value, the shape a
// max-of-chordCount rule ships, fails both assertions.
func TestLoftCircularSagittaIsCertifiedNotHeld(t *testing.T) {
	const m = 3
	seg, _, held, lower := shortfallArc(t, m)

	certified := loftCertifiedSagittaUpper(seg, m)
	require.False(t, isNonFinite(certified))
	require.Greater(t, lower, held,
		"a proven LOWER bound on the true sagitta exceeds the held value, so the held value bounds nothing")
	require.Greater(t, certified, held,
		"the arm must publish the certified enclosure, which on this fixture sits above the held reading")
	require.GreaterOrEqual(t, certified, lower)
}

// TestLoftCircularCellStationsPublishesTheCertifiedReading pins WHICH value the
// arm publishes, on an ordinary quarter arc rather than a cancellation fixture:
// the cell's bound is loftCertifiedSagittaUpper at the settled count, exactly,
// never chordSagitta's held float for the same walk and count. The two are
// distinct values, so substituting the held one fails the equality.
func TestLoftCircularCellStationsPublishesTheCertifiedReading(t *testing.T) {
	const target = 1e-3
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)

	stations, _, sagittaUpper, err := loftCircularCellStations(w, w, seg, seg, target)
	require.NoError(t, err)
	m := len(stations)
	require.Equal(t, loftCertifiedSagittaUpper(seg, m), sagittaUpper)
	require.NotEqual(t, chordSagitta(w.radius, math.Abs(w.th1-w.th0), m), sagittaUpper,
		"the held chooser's own float and the certified enclosure are different readings; the arm publishes the certified one")
}

// shortfallArc scans the family the finding's own audit measured — a radius-5
// arc trimmed to [0.3, 0.9] of a small sweep, on base angles a few radians
// round the circle — and returns the first row where a certified LOWER bound
// on the true per-cell sagitta at count m already exceeds the HELD value
// chordSagitta returns for the same walk.
//
// Such a row exists because the trimming cancels: the walk's two angles are
// both a0 + t*sweep at a magnitude of a few radians, so their float difference
// keeps only the digits the subtraction leaves, while chordSagitta's own slack
// over the true 2r*sin^2(x) is (x/sin x)^2 - 1 ~ x^2/3, which vanishes as the
// sweep shrinks and cannot absorb that loss. The scan is what makes the
// fixture portable: WHICH rows cancel low depends on the host's rounding of
// a0 + t*sweep, but that a sizeable share of them do is arithmetic, not
// accuracy — the audit measured 200 of 800 rows over this family.
func shortfallArc(t *testing.T, m int) (ArcSeg, segmentWalk, float64, float64) {
	t.Helper()
	scanned := 0
	for _, sweep := range []float64{1e-5, 3e-5, 1e-4, 3e-4, 1e-3} {
		for step := range 21 {
			base := 3.0 + 0.05*float64(step)
			seg, w := arcFixture(t, 5, base, sweep, 0.3, 0.9)
			scanned++
			held := chordSagitta(w.radius, math.Abs(w.th1-w.th0), m)
			lower := certifiedSagittaLower(t, seg, m)
			if lower > held {
				t.Logf("shortfall row after %d scanned: base=%v sweep=%v held=%.17g certifiedLower=%.17g (relative shortfall %.5g)",
					scanned, base, sweep, held, lower, (lower-held)/held)
				return seg, w, held, lower
			}
		}
	}
	t.Fatalf("no row of the audit's own arc family understated the sagitta at m=%d over %d fixtures; the fixture family no longer exercises the cancellation this test exists for", m, scanned)
	return ArcSeg{}, segmentWalk{}, 0, 0
}

// certifiedSagittaLower is loftCertifiedSagittaUpper's lower end, built here
// out of the same enclosures so a test can state what the held value is
// measured against. It belongs in the test rather than in the package: nothing
// production publishes a lower bound on a displacement term.
func certifiedSagittaLower(t *testing.T, seg CurveSegment, m int) float64 {
	t.Helper()
	radius, sweep, ok := circularWalkEnclosures(seg)
	require.True(t, ok)
	sin, _, ok := radSinCosSpan(intervalScale(sweep, big.NewRat(1, 4*int64(m))))
	require.True(t, ok)
	require.Positive(t, sin.lo.Sign(), "the cell half-angle must be enclosed strictly inside the first quadrant for its sine's lower end to be a bound")
	s := intervalMul(intervalScale(radius, big.NewRat(2, 1)), intervalMul(sin, sin))
	return ratFloatDown(s.lo)
}

// TestLoftCertifiedSagittaRefusesAnUnderivableRecord is Table S row S14: a
// record whose radius and sweep have no enclosure publishes no sagitta at all.
// A CircleSeg whose radius is not a length cannot state either, so the reading
// answers +Inf and the arm refuses rather than substituting a finite value.
func TestLoftCertifiedSagittaRefusesAnUnderivableRecord(t *testing.T) {
	seg := CircleSeg{Center: pt(0, 0), Radius: units.Radians(1), TStart: 0, TEnd: 1, CCW: true}
	require.True(t, isNonFinite(loftCertifiedSagittaUpper(seg, 8)))

	good, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	_, _, _, err := loftCircularCellStations(w, w, good, seg, 1e-3) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorIs(t, err, errLoftSagittaUnderivable)
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
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	_, _, _, err := loftCellStations(w, w, seg, seg, 1e-12, nil, nil) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, errTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "more than 16384 chords")
}

// --- S16: the one-sided collapsed cell (defensive) ---

// TestLoftCircularCellStationsRefusesOneSidedCollapse is S16, a defensive
// gate: a chord cell whose two stations coincide on exactly ONE of the two
// sections has no case in the uniform two-faces-per-cell wall topology
// assembleLoft builds. It is reachable only from a degenerate record (a
// zero-radius arc, whose Start, End and Center are one coordinate) — a real
// sketch authentication does not produce one (this PR's own risk register,
// mirroring spline_fit.go's dedup carve-out for the free-form arm's analogous
// case) — so this fixture is built on the record directly, bypassing sketch
// entirely, exactly as the plan's own fallback names.
func TestLoftCircularCellStationsRefusesOneSidedCollapse(t *testing.T) {
	degSeg, degWalk := degenerateArcFixture(t) // radius 0: every station is the same point
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	_, _, _, err := loftCircularCellStations(degWalk, w, degSeg, seg, 1e-3) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "collapses to one point on only one of the two sections")
}

// TestLoftCircularCellStationsSymmetricCollapseIsFine proves S16's own
// boundary: a cell collapsed on BOTH sides is sound (loftPayload's own doc
// comment, docs/loft-design.md §12), never refused, and its certified sagitta
// is exactly zero because the record encloses the radius at exactly zero.
func TestLoftCircularCellStationsSymmetricCollapseIsFine(t *testing.T) {
	seg, w := degenerateArcFixture(t)
	stations0, stations1, sagitta, err := loftCircularCellStations(w, w, seg, seg, 1e-3)
	require.NoError(t, err)
	require.NotEmpty(t, stations0)
	require.Len(t, stations1, len(stations0))
	require.Zero(t, sagitta)
	for _, p := range stations0 {
		require.Equal(t, Point2{}, p, "every station of a zero-radius arc is its own centre")
	}
}

// --- the chain's own pinned endpoint ---

// TestCircularStationChainStartsAtThePinnedEnd is the seam regression for the
// station chain's first station: walkOf pins an UNTRIMMED ArcSeg end to the
// recorded Start verbatim (pinArcWalkEnds -> arcWalkEnd, extrude.go), so
// station 0 must BE that recorded coordinate — bit for bit, not within a
// tolerance. Recomputing cU + radius*cos(th0) instead lands beside it, which
// is what this test measures and refuses: the recomputed point is displaced,
// while the reading that station carries says it is not.
func TestCircularStationChainStartsAtThePinnedEnd(t *testing.T) {
	seg, w := arcFixture(t, 5, 0.7, math.Pi/2, 0, 1)
	require.Equal(t, seg.Start.U, w.startU, "walkOf must pin an untrimmed arc start to the recorded coordinate")
	require.Equal(t, seg.Start.V, w.startV)
	require.Equal(t, walkEndBound{}, w.startBound, "the pinned end carries a zero displacement reading")

	stations := circularStationChain(w, 8)
	require.Equal(t, Point2{U: seg.Start.U, V: seg.Start.V}, stations[0],
		"station 0 IS the recorded coordinate the walk pinned, never a recomputed cos/sin at th0")

	// The recomputed point this chain must NOT use, measured: it differs from
	// the recorded coordinate, and circularWalkEndBound reports a nonzero
	// displacement for it, so a station placed there would carry the pinned
	// end's zero reading while sitting off the coordinate it names.
	sin, cos := math.Sincos(w.th0)
	recomputedU, recomputedV := w.cU+w.radius*cos, w.cV+w.radius*sin
	require.NotEqual(t, Point2{U: recomputedU, V: recomputedV}, stations[0],
		"the fixture must be one where the two readings differ, or it proves nothing")
	recomputedBound := circularWalkEndBound(seg, 0, recomputedU, recomputedV)
	require.Positive(t, recomputedBound.u+recomputedBound.v,
		"the recomputed station carries a positive displacement from the enclosure the record states")
	t.Logf("recomputed station 0 sits %.5g, %.5g off the recorded coordinate under a bound of {%.5g, %.5g}",
		recomputedU-seg.Start.U, recomputedV-seg.Start.V, recomputedBound.u, recomputedBound.v)
}

// TestCircularStationChainJunctionsMeetOnOneCoordinate is the terminal half of
// the same rule: the chain excludes its own end point, so a segment's terminal
// station is the NEXT segment's station 0 — and where both ends are untrimmed
// arc ends, that one station is the single recorded coordinate the two
// segments share. The junction therefore carries exactly one station, and it
// is a recorded coordinate.
func TestCircularStationChainJunctionsMeetOnOneCoordinate(t *testing.T) {
	first, wFirst := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	second, wSecond := arcFixture(t, 5, math.Pi/2, math.Pi/2, 0, 1)
	require.Equal(t, first.End, second.Start, "the fixture's two arcs must share one recorded junction coordinate")

	stations := append(circularStationChain(wFirst, 6), circularStationChain(wSecond, 6)...)
	require.Len(t, stations, 12, "neither segment contributes its own end point")
	require.Equal(t, Point2{U: second.Start.U, V: second.Start.V}, stations[6],
		"the junction station is the second segment's pinned start, which is the recorded coordinate")
}

// --- loftPairings: station-chain expansion and sectionDelta ---

// TestLoftPairingsSectionDeltaIsMaxNotSum is the plan's own acceptance line:
// a loop with two curved pairs of different curvature publishes sectionDelta
// as the LARGER cell's own measured sagitta, never their sum — a boundary
// point lies in exactly one cell. The two segment pairs are built directly on
// recorded arcs of different radius and sweep, each independently proven to
// reach a DIFFERENT certified sagitta at the shared target so the
// max-versus-sum distinction is observable.
func TestLoftPairingsSectionDeltaIsMaxNotSum(t *testing.T) {
	const target = 1e-3

	segA, wA := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	_, _, sagA, err := loftCellStations(wA, wA, segA, segA, target, nil, nil)
	require.NoError(t, err)

	segB, wB := arcFixture(t, 1, 0, math.Pi/6, 0, 1)
	_, _, sagB, err := loftCellStations(wB, wB, segB, segB, target, nil, nil)
	require.NoError(t, err)

	require.NotEqual(t, sagA, sagB, "the two segment pairs must reach different sagittas for this test to distinguish max from sum")

	p := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{segA, segB}}}
	walks0 := [][]segmentWalk{{wA, wB}}
	walks1 := [][]segmentWalk{{wA, wB}}

	pairs, sectionDelta, err := loftPairings(p, p, []int{0}, walks0, walks1, target, nil, nil)
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
	pairs, sectionDelta, err := loftPairings(p, p, offsets, walks, walks, 999.0, nil, nil)
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
