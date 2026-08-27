package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a10-plan.md Part 3 PR 5's own test file: loftCellStations (the
// station generator), loftCircularCellStations (the ARC arm), the S15 station
// cap and S16 one-sided collapsed-cell refusal, and loftPairings' own
// station-chain and sectionDelta expansion. Every circular-arm test below
// calls the generator or loftPairings DIRECTLY rather than through
// Document.Loft, so each one pins the arm's own published readings; the
// end-to-end arc build is pinned in loft_arc_pairs_internal_test.go.

// --- shared fixtures ---

// arcFixture builds one RECORDED ArcSeg about the origin — radius r, running
// sweep radians from base angle base, trimmed to [tStart, tEnd] — together
// with the walk walkOf resolves for it. The circular arm reads BOTH: the
// stations come off the walk, while both halves of the published chord bound —
// the certified per-cell sagitta (loft_build.go's loftCertifiedSagittaUpper)
// and the generated stations' own displacement (circularStationChain) — are
// read off the record's own enclosures. A fixture that stated only a
// hand-built walk could not exercise the arm at all, since no record stands
// behind it.
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
	stations0, stations1, sagitta, matchedDelta, stationRound, err := loftCellStations(w0, w1, LineSeg{}, LineSeg{}, 123.0, nil, nil)
	require.NoError(t, err)
	require.Equal(t, []Point2{{U: 1, V: 2}}, stations0)
	require.Equal(t, []Point2{{U: 3, V: 4}}, stations1)
	require.Zero(t, sagitta)
	require.Equal(t, []float64{0}, matchedDelta, "a LineSeg cell's own chord IS the curve it denotes")
	require.Zero(t, stationRound, "a LineSeg station is a recorded endpoint, never computed")
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
	stations, _, sagitta, _, _, err := loftCircularCellStations(w, w, seg, seg, target) //nolint:dogsled // only the stations and the sagitta matter here.
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

	stations0, stations1, sagittaUpper, _, stationUpper, err := loftCircularCellStations(w0, w1, seg0, seg1, target)
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
	sagittaHalf := math.Max(loftCertifiedSagittaUpper(seg0, m), loftCertifiedSagittaUpper(seg1, m))
	require.True(t,
		loftCertifiedSagittaUpper(seg0, m) == sagittaHalf || loftCertifiedSagittaUpper(seg1, m) == sagittaHalf,
		"the published bound's sagitta half must BE one of the two sides' own certified readings, never a blend of them")
	require.Equal(t, sagittaHalf, sagittaUpper,
		"the published sagitta IS one of the two sides' own certified readings")
	require.Equal(t, requireStationDelta(t, w0, w1, seg0, seg1, m), stationUpper,
		"the arm publishes the two sides' own station displacement as its own term, never folded into the sagitta")
}

// requireStationDelta is the station-displacement half of every composition
// assertion below: the larger of the two sides' own generated-station
// displacements at count m, read from the SAME production generator the arm
// itself reads it from.
func requireStationDelta(t *testing.T, w0, w1 segmentWalk, seg0, seg1 CurveSegment, m int) float64 {
	t.Helper()
	_, d0 := circularStationChain(w0, seg0, m)
	_, d1 := circularStationChain(w1, seg1, m)
	require.False(t, isNonFinite(d0), "side 0's station displacement must be derivable at m=%d", m)
	require.False(t, isNonFinite(d1), "side 1's station displacement must be derivable at m=%d", m)
	return math.Max(d0, d1)
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

	stations0, stations1, sagittaUpper, _, stationUpper, err := loftCircularCellStations(w, w, seg, seg, held)
	require.NoError(t, err)
	require.Len(t, stations1, len(stations0))
	require.Greater(t, len(stations0), seed, "the joint walk-up must commit at least one station past the seed")
	require.LessOrEqual(t, sagittaUpper, held)
	m2 := len(stations0)
	require.Equal(t, loftCertifiedSagittaUpper(seg, m2), sagittaUpper)
	require.Equal(t, requireStationDelta(t, w, w, seg, seg, m2), stationUpper)
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
// the cell's bound is loftCertifiedSagittaUpper at the settled count composed
// with the generated stations' own displacement, exactly, never chordSagitta's
// held float for the same walk and count. The two are distinct values, so
// substituting the held one fails the equality.
func TestLoftCircularCellStationsPublishesTheCertifiedReading(t *testing.T) {
	const target = 1e-3
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)

	stations, _, sagittaUpper, _, stationUpper, err := loftCircularCellStations(w, w, seg, seg, target)
	require.NoError(t, err)
	m := len(stations)
	certified := loftCertifiedSagittaUpper(seg, m)
	require.Equal(t, certified, sagittaUpper)
	require.Equal(t, requireStationDelta(t, w, w, seg, seg, m), stationUpper)
	require.Greater(t, stationUpper, 0.0,
		"the trimmed stations of this fixture carry a real displacement, so the cell publishes a positive station term beside the certified sagitta")
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
	_, _, _, _, _, err := loftCircularCellStations(w, w, good, seg, 1e-3) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorIs(t, err, errLoftSagittaUnderivable)
}

// --- loftCircularCellStations: the parameter-matched discharge ---

// TestLoftCircularArcSagittaIsTheUniformParameterMatchedBound corroborates
// the claim loftCircularCellStations' own doc comment makes: under
// uniform-angle parametrization (t_k = th0 + (k/m)*(th1-th0)),
// sup_s |arc(s) - chord(s)| over one chord EQUALS the sagitta
// 2r*sin^2(dtheta/4) exactly, the maximum always landing at s = 1/2. The
// PROOF of that claim is TestArcMatchedDeltaEqualsSagitta
// (bounds_chord_internal_test.go): an analytic derivation whose steps are
// checked over exact rationals, covering every cell angle up to 4 radians
// and so every cell angle chordCount can produce. What this test adds is
// numerical corroboration across a wider spread than the derivation covers —
// sweeps from 0.5 to 359.5 degrees, past the half turn no chord split ever
// hands one cell — sampled, and so evidence rather than proof.
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

// TestLoftCircularCellStationsMatchedDeltaIsItsSagitta pins the coupling the
// derivation above exists to license, on the production path itself: the
// per-cell matchedDelta loftCircularCellStations publishes IS that pairing's
// own sagittaUpper, for EVERY cell of the shared chain, never a separate or
// smaller reading. That per-cell value is the CHORD-TO-CURVE HALF of
// docs/loft-design.md §5.2's matchedDelta row, which every consumer
// (bounds.go's cellChordCurveAreaUpper through loft_moments.go's
// computeLoftChordedAllow) reads straight off this arm before composing it
// with the build's own delta, and loftPayload's own doc comment states the
// equality as fact — so it is asserted here rather than left to the
// assignment that implements it.
//
// The rows exercise every shape the arm can settle on: two identical sides,
// two sides whose own station counts differ (the shared max, where the
// coarser side's sagitta is re-derived at m), a single-cell pairing, and a
// degenerate radius-0 pair whose sagitta is exactly zero.
func TestLoftCircularCellStationsMatchedDeltaIsItsSagitta(t *testing.T) {
	for _, row := range []struct {
		name           string
		r0, sweep0     float64
		r1, sweep1     float64
		target         float64
		wantSharedOnly bool
	}{
		{name: "identical sides", r0: 5, sweep0: math.Pi / 2, r1: 5, sweep1: math.Pi / 2, target: 1e-3},
		{name: "different station counts", r0: 5, sweep0: math.Pi / 2, r1: 2, sweep1: math.Pi / 6, target: 1e-3, wantSharedOnly: true},
		{name: "single cell", r0: 5, sweep0: math.Pi / 2, r1: 4, sweep1: math.Pi / 2, target: 10},
		{name: "degenerate radius", r0: 0, sweep0: math.Pi / 2, r1: 0, sweep1: math.Pi / 2, target: 1e-3},
	} {
		t.Run(row.name, func(t *testing.T) {
			seg0, w0 := arcFixture(t, row.r0, 0, row.sweep0, 0, 1)
			seg1, w1 := arcFixture(t, row.r1, 0, row.sweep1, 0, 1)

			if row.wantSharedOnly {
				m0, _, err := chordCount(w0, row.target)
				require.NoError(t, err)
				m1, _, err := chordCount(w1, row.target)
				require.NoError(t, err)
				require.NotEqual(t, m0, m1, "this row must need different station counts on its two sides")
			}

			stations0, stations1, sagitta, matchedDelta, _, err := loftCircularCellStations(w0, w1, seg0, seg1, row.target)
			require.NoError(t, err)
			require.Len(t, stations1, len(stations0))
			require.Len(t, matchedDelta, len(stations0),
				"one matched-delta entry per cell, the count loftCellStations' own doc comment fixes")

			want := make([]float64, len(stations0))
			for i := range want {
				want[i] = sagitta
			}
			require.Equal(t, want, matchedDelta,
				"every circular cell's matchedDelta must equal the pairing's own sagitta %v exactly", sagitta)
		})
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
// This necessarily precedes S8's own audit-budget ceiling, each row decided at
// the phase docs/loft-design.md §4's gate-order paragraph assigns it: evalLoft
// calls loftPairings (which reaches this refusal through loftCellStations)
// before assembleLoft ever builds a triangle and before
// loftCrossingAudit ever runs (loft_build.go's own evalLoft body). Had the
// cap not fired, the station count alone — already past 2^14 per side — would
// publish orders of magnitude more wall faces than S8's own
// maxFacetPairTestsPerCall ceiling binds at, an F (§7) far past it
// (loft_audit.go), so this fixture is exactly the one the plan names: one
// that would otherwise reach the audit ceiling.
func TestLoftCellStationsStationCapFiresBeforeAuditCeiling(t *testing.T) {
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)
	_, _, _, _, _, err := loftCellStations(w, w, seg, seg, 1e-12, nil, nil) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, errTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "more than 16384 chords")
}

// quarterArcRingProfile is the S15 fixture: n radius-5 quarter arcs recorded in
// one loop, each a same-kind circular pair with itself. Every pair settles at
// the SAME station count against the shared chord target (the whole profile's
// own coordinate envelope decides that target, §5.1, so it does not move with
// n), which is what lets a fixture drive P and C alone and read the per-segment
// share loftStationShare allocates from them.
func quarterArcRingProfile(t *testing.T, n int) (ProfileRecord, [][]segmentWalk) {
	t.Helper()
	segs := make([]CurveSegment, n)
	for i := range n {
		base := float64(i) * math.Pi / 2
		segs[i] = ArcSeg{
			Center: pt(0, 0),
			Start:  pt(5*math.Cos(base), 5*math.Sin(base)),
			End:    pt(5*math.Cos(base+math.Pi/2), 5*math.Sin(base+math.Pi/2)),
			TStart: 0, TEnd: 1,
		}
	}
	p := ProfileRecord{Outer: LoopRecord{Segments: segs}}
	return p, resolveLoftLoopWalks(t, p)
}

// TestLoftStationCapClearsTheAuditPairCeiling pins the DERIVATION behind
// loftStationCap's value (docs/loft-design.md §5.1, §14): a build whose
// Σstations reaches the cap must assemble an F whose F*(F-1)/2 is STRICTLY
// below maxFacetPairTestsPerCall, the ceiling S8 enforces.
//
// The F formula the derivation rests on is measured here on a real assembled
// triangle set rather than assumed: a square-with-square-hole loft has
// Σstations = 8 over H = 1 hole, and assembleLoft emits exactly
// 4*8 + 4*1 - 4 = 32 triangles. A hole-free square gives 4*4 - 4 = 12.
func TestLoftStationCapClearsTheAuditPairCeiling(t *testing.T) {
	assembledTriangles := func(t *testing.T, p ProfileRecord) int {
		t.Helper()
		pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
		offsets, walks0, walks1, err := validateLoftRecords(p, p, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
		require.NoError(t, err)
		pairs, _, _, stationRound, err := loftPairings(p, p, offsets, walks0, walks1, 0, nil, nil)
		require.NoError(t, err)
		a, err := assembleLoft(t.Context(), pairs, mustFrame(t, pl0), mustFrame(t, pl1), pl0, r3.Identity(), stationRound)
		require.NoError(t, err)
		return len(a.tris)
	}

	// F = 4*Σstations + 4H - 4, measured on the built triangle set.
	require.Equal(t, 4*4-4, assembledTriangles(t, unitSquareProfile()),
		"a hole-free 4-station loft must assemble 4*Σ - 4 triangles")
	holed := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []LoopRecord{squareLoop(0.5, 0.5, 0.2, false)}}
	require.Equal(t, 4*8+4*1-4, assembledTriangles(t, holed),
		"an 8-station one-hole loft must assemble 4*Σ + 4H - 4 triangles")

	// H <= Σ - 1 (every loop holds at least one segment and every pair chords
	// at m >= 1), so the worst F a build AT the cap can assemble is 8*cap - 8.
	worstF := uint64(8*loftStationCap - 8)
	worstPairs, ok := wallChoose2(worstF)
	require.True(t, ok, "the worst-case pair count at the cap must not overflow")
	require.Less(t, worstPairs, uint64(maxFacetPairTestsPerCall),
		"a build at the station cap must stay STRICTLY below S8's own pair ceiling")

	// The cap is not arbitrarily conservative either: two stations further in
	// that same worst shape already breaks S8's ceiling, so the constant sits
	// against the bound it is derived from rather than far under it.
	overPairs, ok := wallChoose2(uint64(8*(loftStationCap+2) - 8))
	require.True(t, ok)
	require.Greater(t, overPairs, uint64(maxFacetPairTestsPerCall),
		"the cap must sit at the ceiling it is derived from, not far below it")

	// And it leaves room for every fixture docs/loft-design.md §13 requires:
	// that section's reference wedge forces 64 stations and its calibrated
	// twin settles at 65.
	require.Greater(t, loftStationCap, 65*4, "the cap must leave room for §13's own chorded fixtures")
}

// TestLoftStationCapRefusesPastItsShareBeforeTheAuditCeiling is Table S row
// S15 (docs/loft-design.md §5.1): a same-kind circular pair whose settled
// station count exceeds its own share of loftStationCap refuses with
// errTooManyChords, and the refusal NAMES the segment whose share it exceeded.
//
// It is also the fixture §13 requires for this row — one that fires BEFORE S8
// on a construction that would otherwise reach the audit ceiling. That is
// asserted, not asserted-by-narration: the settled count is read back, the F
// the build would have assembled from it is computed through §7's own formula,
// and its F*(F-1)/2 is shown to exceed maxFacetPairTestsPerCall.
//
// The refusal must NOT read as tessellate.go's own per-walk message. That text
// blames a "chord tolerance" for asking "more than 16384 chords on one curve",
// and a loft has no such caller knob (§5.1: "The target is not a caller
// option") — nor did any single curve here ask for 16384 of anything.
func TestLoftStationCapRefusesPastItsShareBeforeTheAuditCeiling(t *testing.T) {
	const n = 16
	p, walks := quarterArcRingProfile(t, n)

	target, err := loftChordTarget(p, p, walks, walks)
	require.NoError(t, err)
	m, _, _, err := loftSettleStationCount(walks[0][0], walks[0][0], p.Outer.Segments[0], p.Outer.Segments[0], target)
	require.NoError(t, err)

	mMax := loftStationShare(n, n)
	require.Greater(t, m, mMax, "the fixture must settle past its own share, or it proves nothing")
	require.Less(t, m, maxChordsPerWalk, "the fixture must stay inside the per-walk ceiling, so only the CAP can refuse it")

	// What this build would have assembled had the cap not fired: §7's
	// F = 4*Σstations - 4 over this hole-free loop, and S8's own pair count
	// over it (loft_audit.go).
	wouldBePairs, ok := wallChoose2(uint64(4*n*m - 4))
	require.True(t, ok)
	require.Greater(t, wouldBePairs, uint64(maxFacetPairTestsPerCall),
		"the fixture must be one that would otherwise reach S8's audit ceiling")

	err = loftStationCapGate(p, p, make([]int, 1), walks, walks)
	require.ErrorIs(t, err, errTooManyChords, "S15 carries chordCount's own sentinel (spline design Table R row R8)")
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "loop 0 segment 0", "the refusal must name the segment whose share it exceeded")
	require.Contains(t, err.Error(), fmt.Sprintf("%d chord cells", m))
	require.NotContains(t, err.Error(), "chord tolerance", "a loft's chord target is not a caller-supplied tolerance")
	require.NotContains(t, err.Error(), fmt.Sprintf("%d chords on one curve", maxChordsPerWalk),
		"the refusal must not report the per-walk ceiling it never reached")
}

// TestLoftStationCapAdmitsAPairInsideItsShare is S15's own boundary: the same
// arcs, few enough that each one's settled count fits the share the cap
// allocates it, are admitted. Without this the refusal above would prove only
// that the gate refuses everything.
func TestLoftStationCapAdmitsAPairInsideItsShare(t *testing.T) {
	const n = 4
	p, walks := quarterArcRingProfile(t, n)

	target, err := loftChordTarget(p, p, walks, walks)
	require.NoError(t, err)
	m, _, _, err := loftSettleStationCount(walks[0][0], walks[0][0], p.Outer.Segments[0], p.Outer.Segments[0], target)
	require.NoError(t, err)
	require.LessOrEqual(t, m, loftStationShare(n, n), "the fixture must settle inside its own share")

	require.NoError(t, loftStationCapGate(p, p, make([]int, 1), walks, walks))
}

// TestLoftStationShareAllocatesTheCap pins docs/loft-design.md §5.1's own
// allocation arithmetic, including the two cases that paragraph carves out by
// name.
func TestLoftStationShareAllocatesTheCap(t *testing.T) {
	for _, row := range []struct {
		name    string
		p, c    uint64
		want    int
		comment string
	}{
		{name: "one circular pair alone", p: 1, c: 1, want: 1 + (loftStationCap-1)/1},
		{name: "four circular pairs", p: 4, c: 4, want: 1 + (loftStationCap-4)/4},
		{name: "circular pairs among line pairs", p: 100, c: 4, want: 1 + (loftStationCap-100)/4},
		{
			name: "a record whose own P already exceeds the cap clamps to one",
			p:    loftStationCap + 1, c: 1, want: 1,
			comment: "such a record is past chording altogether and S8 is what refuses it",
		},
		{
			name: "integer division only ever under-allocates",
			p:    10, c: 7, want: 1 + (loftStationCap-10)/7,
		},
	} {
		t.Run(row.name, func(t *testing.T) {
			mMax := loftStationShare(row.p, row.c)
			require.Equal(t, row.want, mMax)

			// §5.1's own sum: because C counts the circular pairs AMONG P, a
			// circular pair's m stations SUBSUME the first-station
			// entitlement P already grants it, so the build's total is
			// (P - C)*1 + C*mMax — which can never pass the cap unless P
			// alone already did.
			require.LessOrEqual(t, row.c, row.p, "C counts the circular pairs among P, so it can never exceed it")
			total := (row.p - row.c) + row.c*uint64(mMax) //nolint:gosec // mMax is a positive share bounded by loftStationCap.
			if row.p <= loftStationCap {
				require.LessOrEqual(t, total, uint64(loftStationCap),
					"no build every pair of which passes S15 may exceed the cap")
			}
		})
	}
}

// TestLoftStationCapGateNeverConsultsTheCapWithNoCircularPair is §5.1's C == 0
// carve-out: an all-LineSeg build's Σstations is Σn_i exactly, the count the
// record itself states, so S8 is its only resource refusal and the gate does
// not even read the chord target. The fixture proves the second half by
// handing the gate a profile loftChordTarget itself would REFUSE — a free-form
// boundary profileCoordinateUpper has no envelope for — and requiring the gate
// to answer nil anyway.
func TestLoftStationCapGateNeverConsultsTheCapWithNoCircularPair(t *testing.T) {
	p := unitSquareProfile()
	walks := resolveLoftLoopWalks(t, p)
	require.NoError(t, loftStationCapGate(p, p, make([]int, 1), walks, walks))

	fit := FitSplineSeg{Fit: []Point2{pt(0, 0), pt(1, 1), pt(2, 0), pt(3, 1), pt(4, 0)}, TStart: 0, TEnd: 1}
	free := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{fit, fit, fit}}}
	freeWalks := resolveLoftLoopWalks(t, free)
	_, err := loftChordTarget(free, free, freeWalks, freeWalks)
	require.Error(t, err, "the fixture must be one whose chord target cannot be read at all")
	require.NoError(t, loftStationCapGate(free, free, make([]int, 1), freeWalks, freeWalks),
		"a build with no circular pair must never consult the cap, nor the target it is measured against")
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
	_, _, _, _, _, err := loftCircularCellStations(degWalk, w, degSeg, seg, 1e-3) //nolint:dogsled // only the error matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "collapses to one point on only one of the two sections")
}

// TestLoftCircularCellStationsSymmetricCollapseIsFine proves S16's own
// boundary: a cell collapsed on BOTH sides is sound (loftPayload's own doc
// comment, docs/loft-design.md §12), never refused, and its published chord
// bound is exactly zero: the record encloses the radius at exactly zero, which
// zeroes the certified sagitta and every station's own displacement alike.
func TestLoftCircularCellStationsSymmetricCollapseIsFine(t *testing.T) {
	seg, w := degenerateArcFixture(t)
	stations0, stations1, sagitta, _, _, err := loftCircularCellStations(w, w, seg, seg, 1e-3)
	require.NoError(t, err)
	require.NotEmpty(t, stations0)
	require.Len(t, stations1, len(stations0))
	require.Zero(t, sagitta)
	for _, p := range stations0 {
		require.Equal(t, Point2{}, p, "every station of a zero-radius arc is its own centre")
	}
}

// lineStationLoopFixture records one loop of LineSegs whose walk STARTS are
// exactly the points given, chained End-to-Start around the loop. The LineSeg
// arm publishes one station a side at its own recorded start
// (loftLineCellStations), so the points given ARE the station chain
// loftPairings assembles for that side — which is what lets an S16 fixture
// state a cell's collapse directly instead of hunting for a curve that
// produces one. A repeated point makes a zero-length LineSeg, which
// docs/loft-design.md §4 names as a recordable input rather than a
// hypothetical one (record.go's validateSegment checks finiteness alone).
func lineStationLoopFixture(t *testing.T, starts []Point2) (ProfileRecord, [][]segmentWalk) {
	t.Helper()
	n := len(starts)
	segs := make([]CurveSegment, n)
	for j := range n {
		segs[j] = LineSeg{Start: starts[j], End: starts[(j+1)%n], TStart: 0, TEnd: 1}
	}
	p := ProfileRecord{Outer: LoopRecord{Segments: segs}}
	return p, resolveLoftLoopWalks(t, p)
}

// requireOneSidedCellRefusal is S16's own reading: ErrUnsupported and never
// ErrDegenerate. The distinction is the whole point of the row — a one-sided
// collapse that escapes here falls through to S6 (loft_audit.go), whose
// collapse refusal claims no body exists under ANY evaluator, where this row
// owes only that THIS evaluator's uniform two-faces-per-cell topology has no
// case for a point-degenerate correspondence.
func requireOneSidedCellRefusal(t *testing.T, err error, cell int) {
	t.Helper()
	require.ErrorIs(t, err, ErrUnsupported)
	require.NotErrorIs(t, err, ErrDegenerate, "S16 owes ErrUnsupported; ErrDegenerate is S6's own claim")
	require.Contains(t, err.Error(), fmt.Sprintf("chord cell %d", cell))
	require.Contains(t, err.Error(), "collapses to one point on only one of the two sections")
}

// TestLoftPairingsRefusesAOneSidedTerminalCell is S16 over a class of cell no
// per-segment generator can see: a segment's TERMINAL cell, which pairs that
// segment's last station with the NEXT segment's first. Every LineSeg-pair
// cell is one of those — that arm publishes a single station a side — so this
// fixture is also the LineSeg pairing's only S16 reading.
//
// Side 0 repeats its first station, side 1 does not, so cell 0 collapses on
// exactly one of the two sections.
func TestLoftPairingsRefusesAOneSidedTerminalCell(t *testing.T) {
	p0, walks0 := lineStationLoopFixture(t, []Point2{pt(0, 0), pt(0, 0), pt(1, 1)})
	p1, walks1 := lineStationLoopFixture(t, []Point2{pt(0, 0), pt(2, 0), pt(1, 3)})

	_, _, _, _, err := loftPairings(p0, p1, make([]int, 1), walks0, walks1, 0, nil, nil) //nolint:dogsled // only the error matters here.
	requireOneSidedCellRefusal(t, err, 0)
}

// TestLoftPairingsRefusesAOneSidedWrapCell is the same row at the loop's own
// WRAP: a loop's last cell pairs its last station back to its first (§7's
// flattened chord-cell sequence), so that cell exists in no segment at all and
// is invisible to any walk that stops at the end of the chain. Cells 0 and 1
// here are sound on both sections; only the wrap collapses, and only on side 0.
func TestLoftPairingsRefusesAOneSidedWrapCell(t *testing.T) {
	p0, walks0 := lineStationLoopFixture(t, []Point2{pt(0, 0), pt(1, 1), pt(0, 0)})
	p1, walks1 := lineStationLoopFixture(t, []Point2{pt(0, 0), pt(2, 0), pt(3, 3)})

	_, _, _, _, err := loftPairings(p0, p1, make([]int, 1), walks0, walks1, 0, nil, nil) //nolint:dogsled // only the error matters here.
	requireOneSidedCellRefusal(t, err, 2)
}

// TestLoftPairingsRefusesAOneSidedCellAtOneChordCell is the third class: a
// circular pair settled at m = 1 holds no INTERIOR station at all (§5.1), so
// its single cell reaches into the next segment and no consecutive station
// pair exists inside it. The pair below settles at exactly one chord cell —
// asserted, since the fixture proves nothing otherwise — with side 0's station
// landing on the next segment's own start and side 1's not.
func TestLoftPairingsRefusesAOneSidedCellAtOneChordCell(t *testing.T) {
	const target = 1.0
	arc0, w0 := arcFixture(t, 1e-9, 0, 0.1, 0, 1)
	arc1, w1 := arcFixture(t, 1, 0, 0.1, 0, 1)

	stations0, stations1, _, _, _, err := loftCircularCellStations(w0, w1, arc0, arc1, target) //nolint:dogsled // only the two station chains matter here.
	require.NoError(t, err)
	require.Len(t, stations0, 1, "the fixture must settle at one chord cell, or it tests a different class")
	require.Len(t, stations1, 1)

	line0 := LineSeg{Start: stations0[0], End: pt(0, 1), TStart: 0, TEnd: 1}
	line1 := LineSeg{Start: pt(2, 0), End: pt(0, 1), TStart: 0, TEnd: 1}
	require.NotEqual(t, stations1[0], line1.Start, "side 1's cell must NOT collapse, or the cell is S6's row and not this one")

	p0 := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{arc0, line0}}}
	p1 := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{arc1, line1}}}

	_, _, _, _, err = loftPairings(p0, p1, make([]int, 1), resolveLoftLoopWalks(t, p0), resolveLoftLoopWalks(t, p1), target, nil, nil) //nolint:dogsled // only the error matters here.
	requireOneSidedCellRefusal(t, err, 0)
}

// TestLoftPairingsAdmitsABothSidedCollapsedCell is S16's own boundary, and the
// reason the gate compares the two sides rather than refusing on either: a
// cell collapsing on BOTH sections is S6's row, so loftPairings must let it
// through for the audit to answer it. Without this the three refusals above
// would prove only that the gate refuses every collapse.
func TestLoftPairingsAdmitsABothSidedCollapsedCell(t *testing.T) {
	p0, walks0 := lineStationLoopFixture(t, []Point2{pt(0, 0), pt(0, 0), pt(1, 1)})
	p1, walks1 := lineStationLoopFixture(t, []Point2{pt(5, 5), pt(5, 5), pt(9, 9)})

	pairs, _, _, _, err := loftPairings(p0, p1, make([]int, 1), walks0, walks1, 0, nil, nil) //nolint:dogsled // only the pairs matter here.
	require.NoError(t, err)
	require.Len(t, pairs, 1)
	require.Equal(t, pairs[0].v[0], pairs[0].v[1], "the fixture's cell 0 must collapse on side 0")
	require.Equal(t, pairs[0].w[0], pairs[0].w[1], "and on side 1 too, or it is not this boundary")
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

	stations, _ := circularStationChain(w, seg, 8)
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

	firstChain, _ := circularStationChain(wFirst, first, 6)
	secondChain, _ := circularStationChain(wSecond, second, 6)
	stations := append(append([]Point2{}, firstChain...), secondChain...)
	require.Len(t, stations, 12, "neither segment contributes its own end point")
	require.Equal(t, Point2{U: second.Start.U, V: second.Start.V}, stations[6],
		"the junction station is the second segment's pinned start, which is the recorded coordinate")
}

// --- the chain's own station displacement ---

// TestCircularStationChainDeltaBoundsEveryGeneratedStation is the regression
// for the chain's second return, and the assertion with power over the SHIPPED
// station geometry: every station the chain produces must sit within the
// returned delta of the point the RECORD denotes at that station's own exact
// parameter t_k = TStart + (k/m)·(TEnd − TStart), measured here against a
// fresh enclosure rather than against the chain's own bookkeeping.
//
// It is deliberately two-sided. The upper half proves the published delta
// covers the built geometry, so a generator that drifts off the recorded curve
// — by a scale factor on the radius, a wrong angle, a dropped pin — either
// moves the delta with it or fails outright. The lower half proves the delta is
// ATTAINED, so it is a measurement of this build's own stations rather than a
// constant large enough to cover anything.
func TestCircularStationChainDeltaBoundsEveryGeneratedStation(t *testing.T) {
	const m = 65
	seg, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)

	stations, delta := circularStationChain(w, seg, m)
	require.Len(t, stations, m)
	require.False(t, isNonFinite(delta), "the reference arc's stations must state a displacement")
	require.Positive(t, delta, "the interior stations are computed trigonometry, so their displacement is real, never zero")
	require.Less(t, delta, 1e-13,
		"a station displacement this far above rounding means the generator is no longer walking the recorded curve")

	tStart, dt, ok := circularSegmentRange(seg)
	require.True(t, ok)

	worst := math.Max(walkEndPlaneDelta(w.startBound), walkEndPlaneDelta(w.endBound))
	for k := 1; k < m; k++ {
		tk := new(big.Rat).Add(tStart, new(big.Rat).Mul(big.NewRat(int64(k), int64(m)), dt))
		uIv, vIv, ok := circularEndpointInterval(seg, tk)
		require.True(t, ok, "the record must enclose its own point at station %d's parameter", k)

		// The enclosure is a bracket, so the station's own worst-case gap from
		// the point inside it is the wider of its two ends' gaps — the same
		// reading intervalFloatError takes, spelled out here so this assertion
		// does not borrow the production helper it is checking.
		gapU := math.Max(rationalFloatError(uIv.lo, stations[k].U), rationalFloatError(uIv.hi, stations[k].U))
		gapV := math.Max(rationalFloatError(vIv.lo, stations[k].V), rationalFloatError(vIv.hi, stations[k].V))
		require.LessOrEqual(t, math.Hypot(gapU, gapV), delta,
			"station %d sits %g from the point the record denotes at its own parameter, past the published delta of %g", k, math.Hypot(gapU, gapV), delta)
		worst = math.Max(worst, radius2D(gapU, gapV))
	}
	require.Equal(t, worst, delta, "the published delta is the worst station's own reading, never a padded constant")
	t.Logf("the reference arc's %d stations sit within %.4g mm of the curve the record states", m, delta)
}

// TestCircularStationChainRefusesAnUnenclosableRecord pins the chain's own
// refusal: a segment kind the circular enclosures cannot state a parameter
// range for has no proven station displacement, so the chain answers +Inf and
// loftCircularCellStations refuses on it rather than publishing the certified
// sagitta as if it were the whole chord bound.
func TestCircularStationChainRefusesAnUnenclosableRecord(t *testing.T) {
	_, w := arcFixture(t, 5, 0, math.Pi/2, 0, 1)

	_, delta := circularStationChain(w, LineSeg{Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1}, 4)
	require.True(t, math.IsInf(delta, 1), "a record with no circular parameter range states no station displacement")
	require.True(t, math.IsInf(chordCellDeltaUpper(1e-3, delta), 1), "an underivable half makes the whole chord bound underivable")
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
	_, _, sagA, _, _, err := loftCellStations(wA, wA, segA, segA, target, nil, nil) //nolint:dogsled // only sagA and err matter here.
	require.NoError(t, err)

	segB, wB := arcFixture(t, 1, 0, math.Pi/6, 0, 1)
	_, _, sagB, _, _, err := loftCellStations(wB, wB, segB, segB, target, nil, nil) //nolint:dogsled // only sagB and err matter here.
	require.NoError(t, err)

	require.NotEqual(t, sagA, sagB, "the two segment pairs must reach different sagittas for this test to distinguish max from sum")

	p := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{segA, segB}}}
	walks0 := [][]segmentWalk{{wA, wB}}
	walks1 := [][]segmentWalk{{wA, wB}}

	pairs, sectionDelta, _, _, err := loftPairings(p, p, []int{0}, walks0, walks1, target, nil, nil)
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
	pairs, sectionDelta, _, stationRound, err := loftPairings(p, p, offsets, walks, walks, 999.0, nil, nil)
	require.NoError(t, err)
	require.Len(t, pairs, 1)
	require.Len(t, pairs[0].v, 4)
	require.Len(t, pairs[0].w, 4)
	for j := range 4 {
		require.Equal(t, Point2{U: walks[0][j].startU, V: walks[0][j].startV}, pairs[0].v[j])
		require.Equal(t, Point2{U: walks[0][j].startU, V: walks[0][j].startV}, pairs[0].w[j])
	}
	require.Zero(t, sectionDelta)
	require.Zero(t, stationRound)
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
