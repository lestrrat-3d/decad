package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/spline-design.md §10 P4b's build obligations that
// need a record built directly rather than through sketch's own arrangement
// — the same pattern extrude_work_test.go and prism_boolean_internal_test.go
// already use for a synthetic prismPayload.

// TestEvalPrismCollapsedSpanRunStillBuilds asserts observable test 11: a
// walk carrying a RUN of collapsed spans inside a longer clamped net still
// builds and reports its Volume — §6.5 SKIPS a collapsed span rather than
// refusing the body (Table K). This reuses
// TestConsecutiveCollapsedSpansPairAcrossTheWholeRun's own fixture
// (spline_convexity_internal_test.go), which already pins the certificate's
// own verdict on it (freeformConvexityPositive); this test pins the BUILD.
func TestEvalPrismCollapsedSpanRunStillBuilds(t *testing.T) {
	t.Parallel()
	seg := NURBSSeg{
		Degree: 1,
		Control: []Point2{
			{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 0}, {U: 1, V: 0}, {U: 1, V: 1},
		},
		Knots:   []float64{0, 0, 1, 2, 3, 4, 4},
		Weights: []float64{1, 1, 1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		seg,
		LineSeg{Start: Point2{U: 1, V: 1}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)

	body, err := evalPrism(New(), 0, prismPayload{
		profile: profile,
		frame:   frame,
		z1:      3,
		xform:   r3.Identity(),
	}, newFreeformWork())
	require.NoError(t, err)
	require.NotNil(t, body)
	require.Greater(t, body.volume.Value.Mag(), 0.0, "the walk's own length stays positive despite the collapsed run")
}

// A recorded segment's walk states the record's own endpoints at the two
// natural bounds. Each analytic kind's parameterization runs Start → End over
// [0, 1] (record.go), so its value at t = 0 is Start and at t = 1 is End,
// exactly — while the float routes to those points (a line's chord lerp, an
// arc's atan2 plus sweep) need not land back on them.
//
// Each arm states that premise before asserting the walk, and states it where
// it is decidable. The line's route is a chord formula over two exact float64
// constants, and its own multiply is by one — exact whether or not the build
// contracts the multiply-add — so whether that route misses follows from
// IEEE-754 arithmetic alone and one fixture can require it. The arc's route runs
// through atan2 and cos/sin, whose last bit does differ between platforms: a
// build that contracts a multiply-add rounds once where another rounds twice, so
// whether any single arc misses is a fact about the host rather than about this
// seam. The arc arm therefore measures the premise over the family the diagnosis
// covered, on the running platform, and requires that family to hold a miss;
// that is what keeps its walk assertions non-vacuous wherever the test runs.
//
// What depends on it: buildPrismScene (prism_boolean.go) creates one sketch
// point per walked endpoint, so a walk landing an ulp off the vertex two
// segments share would offer sketch two points where the record states one, and
// RecordProfile would then refuse the region the arrangement admits on its own
// proximity threshold.
func TestWholeSegmentWalkStatesTheRecordedEndpoints(t *testing.T) {
	t.Parallel()
	t.Run("line", func(t *testing.T) {
		// 4/7 → 10/3 is a pair the chord formula does not round-trip: the
		// difference needs a finer grid than its own exponent carries, so it
		// rounds, and start + (end − start) then lands one ulp short of end.
		start := Point2{U: 4.0 / 7.0, V: 2}
		end := Point2{U: 10.0 / 3.0, V: 2}
		require.NotEqual(t, end.U, start.U+1.0*(end.U-start.U),
			`premise: evaluating the chord formula at t = 1 misses this endpoint`)

		u0, v0 := lerp2(start, end, 0)
		require.Equal(t, start, Point2{U: u0, V: v0})
		u1, v1 := lerp2(start, end, 1)
		require.Equal(t, end, Point2{U: u1, V: v1})

		// An interior parameter names no recorded coordinate, so the formula
		// remains what states it.
		uq, vq := lerp2(start, end, 0.25)
		require.Equal(t, start.U+0.25*(end.U-start.U), uq)
		require.Equal(t, start.V+0.25*(end.V-start.V), vq)

		w, err := walkOf(LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{start.U, start.V, end.U, end.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		// A reversed whole edge records TStart = 1, TEnd = 0 (seam.go), so the
		// walk's own ends swap while each still states a recorded coordinate.
		rev, err := walkOf(LineSeg{Start: start, End: end, TStart: 1, TEnd: 0}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{end.U, end.V, start.U, start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})
	})

	t.Run("arc", func(t *testing.T) {
		const cu, cv, r = 10.3, 9.7, 4.7
		th0, th1 := 0.05, 0.75
		seg := ArcSeg{
			Center: Point2{U: cu, V: cv},
			Start:  Point2{U: cu + r*math.Cos(th0), V: cv + r*math.Sin(th0)},
			End:    Point2{U: cu + r*math.Cos(th1), V: cv + r*math.Sin(th1)},
			TStart: 0, TEnd: 1,
		}

		w, err := walkOf(seg, nil)
		require.NoError(t, err)

		// Observation, not a requirement: whether THIS arc's own end angle
		// lands back on its recorded endpoint depends on the host's arithmetic.
		// The family below is what states the premise.
		t.Logf("this fixture's recorded arc end is %v; its walk's own end angle reaches %v",
			seg.End, walkEndFromModel(w))

		require.Equal(t,
			[4]float64{seg.Start.U, seg.Start.V, seg.End.U, seg.End.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		rev, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 1, TEnd: 0,
		}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{seg.End.U, seg.End.V, seg.Start.U, seg.Start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})

		// A trimmed bound keeps the circular model's own value: the record
		// states no coordinate there, and this seam never invents one.
		part, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 0, TEnd: 0.5,
		}, nil)
		require.NoError(t, err)
		require.Equal(t, [2]float64{seg.Start.U, seg.Start.V}, [2]float64{part.startU, part.startV})
		require.Equal(t, walkEndFromModel(part), Point2{U: part.endU, V: part.endV})

		// The premise, stated over the family the diagnosis measured rather
		// than over one fixture: across these arcs at least one walk's own end
		// angle misses its recorded endpoint, so at least one of the endpoint
		// assertions this loop makes would fail if a whole segment's walk
		// answered its circular model at that bound instead of the record.
		var missed, total int
		for _, c := range []struct{ cu, cv, r float64 }{
			{10.3, 9.7, 4.7},
			{-3.35, 0.15, 1.9},
			{0.7, -12.25, 21.3},
			{1000 + 1.0/3.0, 2.0 / 7.0, 0.35},
		} {
			for i := range 5 {
				a0 := 0.05 + 0.71*float64(i)
				for j := range 4 {
					a1 := a0 + 0.13 + 0.79*float64(j)
					fam := ArcSeg{
						Center: Point2{U: c.cu, V: c.cv},
						Start:  Point2{U: c.cu + c.r*math.Cos(a0), V: c.cv + c.r*math.Sin(a0)},
						End:    Point2{U: c.cu + c.r*math.Cos(a1), V: c.cv + c.r*math.Sin(a1)},
						TStart: 0, TEnd: 1,
					}
					fw, err := walkOf(fam, nil)
					require.NoError(t, err)
					require.Equal(t,
						[4]float64{fam.Start.U, fam.Start.V, fam.End.U, fam.End.V},
						[4]float64{fw.startU, fw.startV, fw.endU, fw.endV},
						`the walk states this arc's own recorded endpoints`)
					total++
					if walkEndFromModel(fw) != fam.End {
						missed++
					}
				}
			}
		}
		t.Logf("%d of %d arcs in the family have a walk end angle that misses the recorded endpoint", missed, total)
		require.Positive(t, missed,
			`premise: this platform's arc family holds an end angle that misses its recorded endpoint`)
	})
}

// cosEighthPi and sinEighthPi are cos(π/8) and sin(π/8) to 60 significant
// digits — the values the trimmed fixtures below actually denote, computed
// outside this package from the closed forms √(2+√2)/2 and √(2−√2)/2. They are
// the TRUTH each assertion measures a published interval against, so they must
// come from arithmetic none of the code under test performs; a float64 constant
// could not state either one, since the whole defect lives in the last two bits
// of a float64.
var (
	cosEighthPi = mustRatDecimal("0.923879532511286756128183189396788286822416625863642486115097")
	sinEighthPi = mustRatDecimal("0.382683432365089771728459984030398866761344562485627041433800")
)

// requireEnclosesTruth proves a published reading covers the value the record
// denotes: the gap between the held float and the truth must not exceed the
// bound published beside it. A zero bound therefore demands an exactly held
// reading, which is the whole point of asserting it this way.
func requireEnclosesTruth(t *testing.T, held, bound float64, truth *big.Rat, what string) {
	t.Helper()
	gap := new(big.Rat).Sub(floatRat(held), truth)
	gap.Abs(gap)
	require.LessOrEqual(t, gap.Cmp(floatRat(bound)), 0,
		`%s: the held %.20f sits %s from the value the record denotes, past the published bound %g`,
		what, held, gap.FloatString(22), bound)
}

func oneSegmentProfile(seg CurveSegment) ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}
}

// A walk endpoint the record does not state verbatim is a coordinate this
// evaluator COMPUTED — a float lerp for a trimmed line, a math.Cos/math.Sin at
// a computed angle for a trimmed arc and for every circle — so the walk states
// what it is worth and the boundary-extreme scan publishes that width. Read
// along a direction the trimmed end itself attains, the scan's own interval must
// contain the value the record denotes; a zero bound there is a false claim,
// because the true endpoint sits outside the interval the scan reports.
func TestBoundaryExtremesChargeAComputedWalkEndpoint(t *testing.T) {
	t.Parallel()
	// The trimmed quarter of a quarter-circle arc: the walk sweeps θ ∈
	// [π/8, 3π/8], so along (1, 0) both extremes ARE endpoints — the interior
	// apex at θ = 0 is not swept and contributes nothing.
	trimmedArc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 1, V: 0},
		End:    Point2{U: 0, V: 1},
		TStart: 0.25, TEnd: 0.75,
	}
	// The same two angles read off a circle instead, where no endpoint is ever
	// pinned to a recorded coordinate: 2π·0.0625 = π/8 and 2π·0.1875 = 3π/8.
	trimmedCircle := CircleSeg{
		Center: Point2{U: 0, V: 0},
		Radius: units.Millimeters(1),
		CCW:    true,
		TStart: 0.0625, TEnd: 0.1875,
	}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{name: "arc", seg: trimmedArc},
		{name: "circle", seg: trimmedCircle},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := walkOf(tc.seg, nil)
			require.NoError(t, err)
			require.Positive(t, w.startBound.u, `a trimmed circular endpoint is not a recorded coordinate`)
			require.Positive(t, w.endBound.u, `a trimmed circular endpoint is not a recorded coordinate`)
			require.Less(t, w.startBound.u, 1e-15,
				`the bound is this endpoint's own displacement, not the circle's extent`)

			lo, hi, bound, err := boundaryExtremesBoundedContext(
				t.Context(), oneSegmentProfile(tc.seg), 1, 0, newFreeformWork(), nil)
			require.NoError(t, err)
			require.Positive(t, bound)
			requireEnclosesTruth(t, hi, bound, cosEighthPi, `the maximum along (1, 0)`)
			requireEnclosesTruth(t, lo, bound, sinEighthPi, `the minimum along (1, 0)`)
		})
	}

	// A trimmed LINE endpoint is the same class: lerp2 pins only t = 0 and
	// t = 1, and every other parameter is a float lerp whose exact value is the
	// rational one. 0.3·0.1 is not representable, so this endpoint is held short
	// of what the record denotes.
	t.Run("line", func(t *testing.T) {
		seg := LineSeg{
			Start:  Point2{U: 0, V: 0},
			End:    Point2{U: 0.1, V: 0.1},
			TStart: 0.3, TEnd: 1,
		}
		truth := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
		require.NotEqual(t, 0, truth.Cmp(floatRat(0.3*0.1)),
			`premise: this trimmed line endpoint is not exactly representable`)

		w, err := walkOf(seg, nil)
		require.NoError(t, err)
		require.Positive(t, w.startBound.u)
		require.Equal(t, walkEndBound{}, w.endBound, `t = 1 names the recorded End`)

		lo, hi, bound, err := boundaryExtremesBoundedContext(
			t.Context(), oneSegmentProfile(seg), 1, 0, newFreeformWork(), nil)
		require.NoError(t, err)
		require.Positive(t, bound)
		requireEnclosesTruth(t, lo, bound, truth, `the minimum along (1, 0)`)
		requireEnclosesTruth(t, hi, bound, floatRat(seg.End.U), `the maximum along (1, 0)`)
	})

	// An endpoint whose displacement no arithmetic here can state refuses the
	// whole scan. Folding its +Inf into the accumulators instead would publish
	// an infinite box bound, or read as the empty region's own ErrDegenerate.
	t.Run("underivable", func(t *testing.T) {
		seg := LineSeg{
			Start:  Point2{U: math.Inf(1), V: 0},
			End:    Point2{U: 1, V: 1},
			TStart: 0.5, TEnd: 1,
		}
		w, err := walkOf(seg, nil)
		require.NoError(t, err)
		require.False(t, w.startBound.derivable())

		_, _, _, err = boundaryExtremesBoundedContext(
			t.Context(), oneSegmentProfile(seg), 1, 0, newFreeformWork(), nil)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// The charge is levied on a COMPUTED endpoint only. Where the record states the
// endpoint verbatim — a line's own bounds, an arc's own bounds — and every other
// candidate is exactly representable, the scan keeps the zero bound and the
// section's box stays Exact.
func TestBoundaryExtremesKeepAProvenZero(t *testing.T) {
	t.Parallel()
	square := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 0}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	wholeArc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 1, V: 0},
		End:    Point2{U: 0, V: 1},
		TStart: 0, TEnd: 1,
	}
	wholeCircle := CircleSeg{
		Center: Point2{U: 0, V: 0},
		Radius: units.Millimeters(1),
		CCW:    true,
		TStart: 0, TEnd: 1,
	}

	for _, tc := range []struct {
		name    string
		profile ProfileRecord
		lo, hi  float64
	}{
		{name: "all straight", profile: square, lo: 0, hi: 2},
		{name: "whole arc", profile: oneSegmentProfile(wholeArc), lo: 0, hi: 1},
		{name: "whole circle", profile: oneSegmentProfile(wholeCircle), lo: -1, hi: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			lo, hi, bound, err := boundaryExtremesBoundedContext(
				t.Context(), tc.profile, 1, 0, newFreeformWork(), nil)
			require.NoError(t, err)
			require.Equal(t, 0.0, bound, `every candidate here is a value the record states exactly`)
			require.Equal(t, tc.lo, lo)
			require.Equal(t, tc.hi, hi)
		})
	}

	// The whole circle keeps its zero along (1, 0) for a reason the endpoint's
	// own components state: at t = 0 the record's own quarter-turn readings are
	// exact in both, and at t = 1 the walk lands exactly on the circle's u
	// extreme while missing v — math.Cos returns 1 at that angle and math.Sin
	// does not return 0 — so a direction reading u alone charges nothing while
	// the v error is still stated rather than dropped.
	w, err := walkOf(wholeCircle, nil)
	require.NoError(t, err)
	require.Equal(t, walkEndBound{}, w.startBound)
	require.Equal(t, 0.0, w.endBound.u)
	require.Positive(t, w.endBound.v)
	require.Less(t, w.endBound.v, 1e-15)
	require.Equal(t, 0.0, pointPerturbationAllow(w.endBound, 1, 0))
	require.Positive(t, pointPerturbationAllow(w.endBound, 0, 1))
}

// walkEndFromModel re-derives a circular walk's far endpoint from the walk's OWN
// published model — its centre, radius and end angle — rather than from a second
// copy of walkOf's formula. It is the coordinate the walk would carry at that
// bound if it answered its model there instead of the record.
func walkEndFromModel(w segmentWalk) Point2 {
	sin, cos := math.Sincos(w.th1)
	return Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin}
}

// This file pins the fix for docs/spline-design.md §5.2's own discipline:
// within ONE evalPrismContext call, buildLoopSidesAs,
// profileCoordinateEnvelope (via prismCentroidGeometryBound and, four times
// over, prismBoundsContext's per-axis extentBoundedAlong) and
// boundaryExtremesBoundedContext (three times, also via extentBoundedAlong)
// each used to call walkOf on the SAME recorded segment, so one free-form
// segment's conversion-and-bracket charge was spent eight times over instead
// of once. profileWalks (extrude.go) resolves every segment's walk exactly
// once per evaluation and lets every consumer read it back.

// involuteFitProfile is the requester's own reproduction fixture: 15
// endpoint-inclusive samples of one involute gear-tooth flank
// (involuteFitPoints, spline_convexity_internal_test.go), closed by a
// straight chord back to its start. The construction — a LineSeg from the
// first fit point to the last, followed by the FitSplineSeg walked in
// REVERSE (TStart=1, TEnd=0) — is the same reversed pairing
// spline_fit_test.go's own TestFitSplineTerminalDedupRefusesUnclosedLoopReversed
// uses to reproduce a real recorded record exactly, and it is what gives this
// loop positive net area: the forward pairing (spline first, chord back)
// winds the opposite way and evalPrismContext refuses it as ErrDegenerate.
func involuteFitProfile() ProfileRecord {
	fit := involuteFitPoints()
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: fit[0], End: fit[len(fit)-1], TStart: 0, TEnd: 1},
		FitSplineSeg{Fit: fit, TStart: 1, TEnd: 0},
	}}}
}

// involuteFitPrismPayload builds the payload evalPrism evaluates: an
// unplaced 5 mm prism over involuteFitProfile, the same synthetic-payload
// pattern extrude_freeform_internal_test.go, extrude_work_test.go and
// verify_freeform_internal_test.go already use for a free-form-walled prism,
// since no evaluator path builds one through a live sketch profile yet.
func involuteFitPrismPayload(t *testing.T) prismPayload {
	t.Helper()
	return prismPayload{
		profile: involuteFitProfile(),
		frame:   identityFrame(t),
		z1:      5,
		xform:   r3.Identity(),
	}
}

// TestPrismWalkOnceInvoluteRecordFitsWorkBudget is the observable fix itself:
// before profileWalks, this exact record's walk was resolved eight times
// over within one evalPrismContext call (230,168 units each), for
// 1,841,344 units against a 1,048,576 ceiling — refused as ErrUnsupported.
// With the walk resolved once and every consumer reading it back, the whole
// build's charge fits comfortably inside the ceiling and the body builds.
//
// 959,408 is this file's own pinned measurement, taken directly from this
// exact fixture (involuteFitPrismPayload evaluated through evalPrism, the
// same call every other free-form-prism test in this package uses). It is
// LOWER than the 975,600 the bug report measured on the requester's real
// record by exactly one moments-preflight charge (16,192,
// evaluatorIntegralsUncheckedContext's own — see evaluatorIntegralsUncheckedContext's
// call in evalPrismContext): the requester's number was measured through the
// public Document.Extrude path, whose RecordProfile seam runs its own
// independent-implementation area check (§5.2's "sketch computes its own
// free-form area... decad integrates its OWN records... the two agreeing is
// the §1 falsifier") before Extrude's own evalPrismContext runs its own
// preflight — a second whole-record preflight this synthetic-payload fixture
// never pays, since it calls evalPrism directly on a hand-built
// prismPayload/ProfileRecord rather than through RecordProfile. What both
// numbers agree on is the walk resolving exactly ONCE per evaluation instead
// of eight times: resolveProfileWalks's own charge is
// TestResolveProfileWalksChargesSegmentOnce's 230,168, one occurrence of
// which is common to both totals.
func TestPrismWalkOnceInvoluteRecordFitsWorkBudget(t *testing.T) {
	t.Parallel()
	pp := involuteFitPrismPayload(t)
	work := newFreeformWork()

	body, err := evalPrism(New(), 0, pp, work)
	require.NoError(t, err, "the deduplicated charge must fit inside the work budget")
	require.NotNil(t, body)
	require.Greater(t, body.volume.Value.Mag(), 0.0, "the built prism must enclose positive volume")

	require.Less(t, work.spent, freeformWorkLimit,
		"the whole build's deduplicated charge must sit strictly below the ceiling")
	// The charge is asserted as a BAND, not an exact equality. Most of it is
	// count-driven and identical everywhere, but §6.5's certificate subdivides
	// on a mixed Bernstein sign, and how deep it goes is decided by float
	// coefficients that sketch's own solve produces — Go may contract a*b+c
	// into a fused multiply-add on arm64 and not on amd64, so a coefficient
	// sitting near zero can cost one extra split on one host and not another.
	// Pinning the exact figure would pin the host's FMA contraction rather
	// than the deduplication this test is for. The band is tight enough to
	// fail loudly if the walk were resolved even twice: a second resolution
	// alone adds 230,168 units, far outside it.
	require.Greater(t, work.spent, uint64(900000),
		"the build must still do its real free-form work, not silently skip it")
	require.Less(t, work.spent, uint64(1000000),
		"measured 959,408 on amd64; a second walk resolution would add 230,168 and break this")
}

// TestResolveProfileWalksChargesSegmentOnce is the mechanism
// TestPrismWalkOnceInvoluteRecordFitsWorkBudget's fix rests on: resolving a
// profile's walks charges each segment's conversion and bracket cost exactly
// ONCE, never once per loop iteration or once per caller. 230,168 is the
// segment's own single-resolution cost (this file's involuteFitProfile doc
// comment, and the bug report's own measurement of walkOf on this exact
// segment); the closing LineSeg is analytic and charges nothing
// (walkOf's own doc comment: "An analytic segment charges nothing").
func TestResolveProfileWalksChargesSegmentOnce(t *testing.T) {
	t.Parallel()
	work := newFreeformWork()
	pw, err := resolveProfileWalks(involuteFitProfile(), work)
	require.NoError(t, err)
	require.NotNil(t, pw)
	require.Equal(t, uint64(230168), work.spent,
		"one resolution of one free-form segment must charge exactly its own single-walk cost")
}

// TestProfileWalksMismatchRefuses pins the coarsest half of constraint 5's
// reject-only guard: a *profileWalks resolved from one profile must never be
// read against a profile of a DIFFERENT shape, silently or otherwise. The
// same-shape half — a profile that differs only in its segment DATA — is
// TestProfileWalksSegmentDataMismatchRefuses below. Either set applied anyway
// would read another section's geometry as this one's, a plumbing bug a nil
// fallback would hide behind a correct-looking answer, so every consumer
// refuses instead.
func TestProfileWalksMismatchRefuses(t *testing.T) {
	t.Parallel()
	// A 4-segment square, deliberately a different outer segment count than
	// involuteFitProfile's 2 (a LineSeg and a FitSplineSeg).
	square := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 0}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	wrongWalks, err := resolveProfileWalks(square, newFreeformWork())
	require.NoError(t, err)

	involute := involuteFitProfile()
	require.NotEqual(t, len(square.Outer.Segments), len(involute.Outer.Segments),
		"premise: the two profiles must have different shapes for the guard to fire")
	require.False(t, wrongWalks.matches(involute), "matches must catch the shape mismatch")

	t.Run("profileCoordinateEnvelope", func(t *testing.T) {
		_, err := profileCoordinateEnvelope(involute, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("profileCoordinateUpper", func(t *testing.T) {
		_, err := profileCoordinateUpper(involute, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("boundaryExtremesBoundedContext", func(t *testing.T) {
		_, _, _, err := boundaryExtremesBoundedContext(t.Context(), involute, 1, 0, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// lineEndProfile is a one-segment profile whose only difference from another
// built the same way is WHERE its line ends: same variant, same start, same
// range, one differing coordinate. Two of them have identical shape — one
// outer segment, no holes — so shape alone cannot tell them apart.
func lineEndProfile(end Point2) ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: end, TStart: 0, TEnd: 1},
	}}}
}

// TestProfileWalksSegmentDataMismatchRefuses pins the guard on the reading it
// exists for: a *profileWalks resolved from one profile must not be read
// against a DIFFERENT profile of the same shape. Comparing counts alone let
// such a pair through, and the cached read then published the first section's
// geometry as the second's — profileCoordinateEnvelope reporting 1 for a
// section whose own envelope is 2, with no error anywhere. The comparison is
// over the recorded segments themselves, exactly: the one-ulp subtest below
// pins that it carries no tolerance and no "close enough" arm.
func TestProfileWalksSegmentDataMismatchRefuses(t *testing.T) {
	t.Parallel()
	near := lineEndProfile(Point2{U: 1, V: 0})
	far := lineEndProfile(Point2{U: 2, V: 0})
	require.Len(t, far.Outer.Segments, len(near.Outer.Segments),
		"premise: the two profiles must have the same shape for the data comparison to be the thing under test")
	require.Empty(t, near.Holes)
	require.Empty(t, far.Holes)

	pw, err := resolveProfileWalks(near, newFreeformWork())
	require.NoError(t, err)
	require.True(t, pw.matches(near), "premise: the resolved profile itself must still read back")
	require.False(t, pw.matches(far), "a same-shaped profile with different segment data is not the resolved profile")

	// The two readings the mismatch used to conflate: near's coordinate
	// envelope is 1, far's is 2.
	nearUpper, err := profileCoordinateEnvelope(near, newFreeformWork(), pw)
	require.NoError(t, err)
	require.Equal(t, 1.0, nearUpper, "near's own envelope, read through its own resolved walks")
	farUpper, err := profileCoordinateEnvelope(far, newFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, 2.0, farUpper, "far's true envelope, resolved from far's own segment")

	t.Run("profileCoordinateEnvelope", func(t *testing.T) {
		_, err := profileCoordinateEnvelope(far, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("profileCoordinateUpper", func(t *testing.T) {
		_, err := profileCoordinateUpper(far, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("boundaryExtremesBoundedContext", func(t *testing.T) {
		_, _, _, err := boundaryExtremesBoundedContext(t.Context(), far, 1, 0, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("buildLoopSidesAs", func(t *testing.T) {
		pp := prismPayload{profile: near, frame: identityFrame(t), z1: 5, xform: r3.Identity()}
		body := &Body{doc: New(), solid: true}
		_, _, _, _, err := buildLoopSidesAs(t.Context(), body, 0, pp, 0, false, far.Outer, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("one ulp apart", func(t *testing.T) {
		ulp := lineEndProfile(Point2{U: math.Nextafter(1, 2), V: 0})
		require.False(t, pw.matches(ulp), "the comparison is exact: one ulp of difference is a mismatch")
		_, err := profileCoordinateEnvelope(ulp, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("hole data", func(t *testing.T) {
		withHole := near
		withHole.Holes = []LoopRecord{lineEndProfile(Point2{U: 1, V: 1}).Outer}
		holed, err := resolveProfileWalks(withHole, newFreeformWork())
		require.NoError(t, err)
		other := near
		other.Holes = []LoopRecord{lineEndProfile(Point2{U: 1, V: 3}).Outer}
		require.False(t, holed.matches(other), "a hole loop's own segment data is compared too")
		_, err = profileCoordinateEnvelope(other, newFreeformWork(), holed)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
}

// TestProfileWalksReadBackMatchesFreshResolution is the other half of the
// guard: the profile a set WAS resolved from still reads back, and what it
// hands each consumer is the walk walkOf itself resolves for that segment,
// field for field. The refusal above must cost the matching path nothing.
func TestProfileWalksReadBackMatchesFreshResolution(t *testing.T) {
	t.Parallel()
	profile := involuteFitProfile()
	pw, err := resolveProfileWalks(profile, newFreeformWork())
	require.NoError(t, err)
	require.True(t, pw.matches(profile))
	require.True(t, pw.loopMatches(0, profile.Outer))

	for si, seg := range profile.Outer.Segments {
		fresh, err := walkOf(seg, newFreeformWork())
		require.NoError(t, err)
		require.Equal(t, fresh, pw.at(0, si),
			"segment %d's read-back walk must be exactly the walk walkOf resolves for it", si)
	}

	// The whole cached-read path still produces the reading it did before:
	// the involute section's own coordinate envelope, unchanged by the guard.
	cached, err := profileCoordinateEnvelope(profile, newFreeformWork(), pw)
	require.NoError(t, err)
	direct, err := profileCoordinateEnvelope(profile, newFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, direct, cached, "reading the cache must give the resolve-every-segment answer")
	require.Greater(t, cached, 0.0)
}

func TestEvalPrismContinuesCallerFreeformWork(t *testing.T) {
	t.Parallel()
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		SplineSeg{
			Control: []Point2{{U: 2}, {U: 2, V: 2}, {V: 2}, {}},
			TStart:  0,
			TEnd:    1,
		},
		LineSeg{Start: Point2{}, End: Point2{U: 2}, TStart: 0, TEnd: 1},
	}}}
	work := newFreeformWork()
	_, err := profile.evaluatorIntegrals(momentAreaOrder, work)
	require.NoError(t, err)
	spent := work.spent
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)

	body, err := evalPrism(New(), 0, prismPayload{
		profile: profile,
		frame:   frame,
		z1:      1,
		xform:   r3.Identity(),
	}, work)
	require.NoError(t, err, "a Tier A free-form side face now builds (§10 P4b)")
	require.NotNil(t, body)
	require.Greater(t, work.spent, spent)
}

// ------------------------------------------- reusing a published resolution

// prismPayloadOf reads back the payload a prism build published on its body.
func prismPayloadOf(t *testing.T, b *Body) prismPayload {
	t.Helper()
	pp, ok := b.payload.(prismPayload)
	require.True(t, ok, "a prism build must publish a prismPayload")
	return pp
}

// rebuiltUnder re-evaluates pp under motion on a counter the caller can read
// afterwards. It is prismPayload.placed's own body with the counter lifted out:
// placed mints one internally, which is the single thing about the placement
// path a test cannot otherwise observe. The payload it carries over and the
// build it runs are the same.
func rebuiltUnder(t *testing.T, pp prismPayload, motion r3.Transform, work *freeformWork) *Body {
	t.Helper()
	pp.xform = motion
	body, err := evalPrism(New(), 0, pp, work)
	require.NoError(t, err, "the re-evaluation under the motion must build")
	return body
}

// withoutWalks is the reuse switch this file tests against: the same payload
// with its published resolution dropped, so the build resolves every walk
// itself exactly as it did before a payload carried one.
func withoutWalks(pp prismPayload) prismPayload {
	pp.walks = nil
	return pp
}

// placementMotions are the rigid motions the reuse must survive: a pure
// translation, a rotation about the sweep axis, a reflection (which inverts
// face orientation and arc sense downstream of the plane-local walk, and so is
// the motion most likely to expose a walk that secretly carried placement), and
// a composition of a rotation with a translation.
func placementMotions(t *testing.T) []struct {
	name   string
	motion r3.Transform
} {
	t.Helper()
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(math.Pi/3))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(3, -4, 7))
	require.NoError(t, err)
	composed, err := turn.Then(shift)
	require.NoError(t, err)
	mirror, err := r3.Reflection(identityFrame(t))
	require.NoError(t, err)
	require.True(t, mirror.IsReflection(), "premise: the mirror must actually reverse orientation")
	return []struct {
		name   string
		motion r3.Transform
	}{
		{name: "identity", motion: r3.Identity()},
		{name: "translation", motion: shift},
		{name: "rotation", motion: turn},
		{name: "reflection", motion: mirror},
		{name: "composed", motion: composed},
	}
}

// requireSamePrismBuild asserts two prism builds published the same geometry:
// every measurement with its own exactness and proven bound, every face's
// surface, area and roles, and the same topology counts. It compares the
// measurements as whole values rather than within a tolerance, because a reused
// resolution that changed an answer AT ALL would have changed what the body
// claims about itself.
func requireSamePrismBuild(t *testing.T, want, got *Body) {
	t.Helper()
	require.Equal(t, want.solid, got.solid, "solidity")
	require.Equal(t, want.volume, got.volume, "volume, its exactness and its bound")
	require.Equal(t, want.area, got.area, "area, its exactness and its bound")
	require.Equal(t, want.centroid, got.centroid, "centroid, its exactness and its bound")
	require.Equal(t, want.bounds, got.bounds, "bounding box, its exactness and its bound")

	wantFaces, gotFaces := want.Faces(), got.Faces()
	require.Len(t, gotFaces, len(wantFaces), "face count")
	for i := range wantFaces {
		require.Equal(t, wantFaces[i].surface, gotFaces[i].surface, "face %d's surface", i)
		require.Equal(t, wantFaces[i].area, gotFaces[i].area, "face %d's area", i)
		require.Equal(t, wantFaces[i].areaBound, gotFaces[i].areaBound, "face %d's area bound", i)
		require.Equal(t, wantFaces[i].origins, gotFaces[i].origins, "face %d's roles", i)
		require.Equal(t, wantFaces[i].axialDelta, gotFaces[i].axialDelta, "face %d's axial displacement", i)
	}
	require.Len(t, got.Edges(), len(want.Edges()), "edge count")
	require.Len(t, got.Vertices(), len(want.Vertices()), "vertex count")
}

// TestPrismBuildPublishesItsOwnResolution pins the publication half of the
// reuse: a completed build hands its body the walk resolution it just made,
// metered with what that resolution charged, and matching the record the body
// was built from. Everything downstream reads it through that guard, so a build
// that published an unmetered or mismatched set would simply resolve again —
// which is why the assertions here are on the published set itself.
func TestPrismBuildPublishesItsOwnResolution(t *testing.T) {
	t.Parallel()
	source := involuteFitPrismPayload(t)
	require.Nil(t, source.walks, "premise: a payload a caller draws carries no resolution")

	built, err := evalPrism(New(), 0, source, newFreeformWork())
	require.NoError(t, err)

	published := prismPayloadOf(t, built)
	require.NotNil(t, published.walks, "a completed build must publish its own resolution")
	require.True(t, published.walks.metered, "the published resolution must know what it cost")
	require.True(t, published.walks.reusable(published.profile), "it must read back against its own record")
	require.Equal(t, uint64(230168), published.walks.spent,
		"the published charge is resolveProfileWalks' own single-resolution cost for this record")
	require.Equal(t, source.profile, published.profile, "publishing a resolution must not disturb the record")
}

// TestPlacedReusesPublishedWalks is the optimization itself: a rigid
// re-evaluation of a recorded section carries the SAME resolution rather than
// bracketing every free-form arc a second time, and it is charged exactly what
// that resolution cost, so the record's ceiling binds it as before.
//
// Pointer identity is the proof that the work was skipped, not merely that the
// answer came out the same: resolveProfileWalks allocates a new set on every
// call, so a placement that resolved again could not hand back this one.
func TestPlacedReusesPublishedWalks(t *testing.T) {
	t.Parallel()
	cold := newFreeformWork()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), cold)
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(math.Pi/3))
	require.NoError(t, err)

	placed, err := source.placed(t.Context(), New(), 0, turn)
	require.NoError(t, err)
	require.Same(t, source.walks, prismPayloadOf(t, placed).walks,
		"a rigid placement must carry the source's resolution, not allocate a second one")
	require.Equal(t, turn, prismPayloadOf(t, placed).xform, "the motion is the only thing the placement changes")

	// The charge is what it always was. The reused build and the resolve-again
	// build levy the same total on their own counters, so a record sitting near
	// the ceiling refuses in exactly the same place either way.
	warm := newFreeformWork()
	reused := rebuiltUnder(t, source, turn, warm)
	require.Same(t, source.walks, prismPayloadOf(t, reused).walks)

	uncached := newFreeformWork()
	plain := rebuiltUnder(t, withoutWalks(source), turn, uncached)
	require.NotSame(t, source.walks, prismPayloadOf(t, plain).walks,
		"premise: without a published resolution the build must allocate its own")

	require.Equal(t, uncached.spent, warm.spent,
		"replaying the recorded charge must levy exactly what doing the work levies")
	require.Equal(t, uncached.reconstructionSpent, warm.reconstructionSpent,
		"the reconstruction counter is replayed on the same terms")
	require.Equal(t, cold.spent, warm.spent,
		"and that is the same figure the original build spent on this record")
	require.Greater(t, warm.spent, published(t, source).spent,
		"premise: the whole build charges more than the walk resolution alone")
}

// published is the resolution a payload carries, asserted present.
func published(t *testing.T, pp prismPayload) *profileWalks {
	t.Helper()
	require.NotNil(t, pp.walks)
	return pp.walks
}

// TestPlacedWalksReuseMatchesUncachedBuild is the regression matrix's first
// row. For every rigid motion the payload admits, the body built from a reused
// resolution and the body built by resolving afresh publish the same geometry —
// the same measurements with the same exactness and the same proven bounds, the
// same faces with the same surfaces and roles, and the same topology counts.
//
// The reflection row is the one that earns its place: a reflected placement
// inverts face orientation and arc sense, and it does so downstream of the walk.
// A walk that had secretly carried any placement would disagree here first.
func TestPlacedWalksReuseMatchesUncachedBuild(t *testing.T) {
	t.Parallel()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), newFreeformWork())
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	for _, tc := range placementMotions(t) {
		t.Run(tc.name, func(t *testing.T) {
			reused := rebuiltUnder(t, source, tc.motion, newFreeformWork())
			plain := rebuiltUnder(t, withoutWalks(source), tc.motion, newFreeformWork())
			requireSamePrismBuild(t, plain, reused)
		})
	}
}

// placementChain places pp ten times over, one pi/5 turn about the sweep axis
// at a time, and returns the last body with the resolution each step carried.
// drop clears the published resolution before every step, which is how the same
// chain is run against the path that resolves its walks itself.
func placementChain(t *testing.T, pp prismPayload, drop bool) (*Body, []*profileWalks) {
	t.Helper()
	step, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(math.Pi/5))
	require.NoError(t, err)

	var carried []*profileWalks
	var body *Body
	for k := range 10 {
		if drop {
			pp = withoutWalks(pp)
		}
		composed, err := pp.xform.Then(step)
		require.NoError(t, err, "composing placement %d", k+1)
		body, err = pp.placed(t.Context(), New(), 0, composed)
		require.NoError(t, err, "placement %d", k+1)
		pp = prismPayloadOf(t, body)
		carried = append(carried, pp.walks)
	}
	return body, carried
}

// TestRepeatedPlacementAccumulatesNoError is the matrix's second row: placing
// the same section ten times over adds no error term and drops none. The chain
// is run twice — once reusing the published resolution throughout, once
// resolving afresh at every step — and the two must agree on every published
// reading, including each measurement's own proven bound.
//
// The bounds are compared between the two CHAINS rather than against the
// unplaced body, because a placement's rounding is a real term the centroid
// bound is supposed to carry (docs/evaluator-design.md §8): it differs from the
// unplaced body's by design. What must not differ is the reused chain from the
// resolve-every-time chain, and that is what this asserts.
func TestRepeatedPlacementAccumulatesNoError(t *testing.T) {
	t.Parallel()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), newFreeformWork())
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	reused, carried := placementChain(t, source, false)
	for k, walks := range carried {
		require.Same(t, source.walks, walks, "placement %d must still carry the original resolution", k+1)
	}

	plain, resolved := placementChain(t, source, true)
	for k, walks := range resolved {
		require.NotSame(t, source.walks, walks, "premise: placement %d resolved its own walks", k+1)
	}

	requireSamePrismBuild(t, plain, reused)

	// Rigid motion invariants the chain must also preserve outright, and the
	// premise that the copies genuinely moved rather than composing back.
	require.Equal(t, built.volume, reused.volume, "ten rotations about the sweep axis change no volume term")
	require.Equal(t, built.area, reused.area, "nor any area term")
	require.NotEqual(t, built.centroid.Value, reused.centroid.Value, "premise: the copies genuinely moved")
}

// TestChangedRecordRefusesPublishedWalks is the matrix's third and fourth rows
// together: a resolution is read back only for the record it was resolved from,
// and every other record — a differing coordinate, a differing segment count, a
// new hole — resolves afresh. The guard is matches, which compares the recorded
// segments by their bits, so there is no near-miss arm to fall through.
func TestChangedRecordRefusesPublishedWalks(t *testing.T) {
	t.Parallel()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), newFreeformWork())
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	nudged := involuteFitProfile()
	line, ok := nudged.Outer.Segments[0].(LineSeg)
	require.True(t, ok, "premise: this fixture's first segment is the closing line")
	line.End.U = math.Nextafter(line.End.U, math.Inf(1))
	nudged.Outer.Segments[0] = line

	t.Run("one ulp of a coordinate", func(t *testing.T) {
		require.False(t, source.walks.reusable(nudged), "one ulp of difference is a different record")
		stale := source
		stale.profile = nudged
		rebuilt, err := evalPrism(New(), 0, stale, newFreeformWork())
		require.NoError(t, err)
		require.NotSame(t, source.walks, prismPayloadOf(t, rebuilt).walks,
			"a changed record must resolve its own walks")
		require.True(t, prismPayloadOf(t, rebuilt).walks.reusable(nudged),
			"and publish them against the record it actually built")
	})

	t.Run("a new hole", func(t *testing.T) {
		holed := source
		holed.profile.Holes = []LoopRecord{lineEndProfile(Point2{U: 1, V: 1}).Outer}
		require.False(t, source.walks.reusable(holed.profile), "a record that gained a hole is a different record")
	})

	t.Run("a rescaled record", func(t *testing.T) {
		scaled := ProfileRecord{Outer: LoopRecord{Segments: make([]CurveSegment, 0, 1)}}
		scaled.Outer.Segments = append(scaled.Outer.Segments,
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1})
		require.False(t, source.walks.reusable(scaled),
			"a record in different coordinates is a different record, whatever produced it")
	})
}

// TestWalksChargeReplayRefusesAtTheCeiling is the matrix's last row: a record
// near its work ceiling must refuse on the reused path exactly as it does on
// the path that does the work, with the same error identity. Replaying the
// recorded charge is what makes that true — a reuse that charged nothing would
// have admitted a build the budget had already proved unaffordable.
func TestWalksChargeReplayRefusesAtTheCeiling(t *testing.T) {
	t.Parallel()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), newFreeformWork())
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	// Spend all but one unit less than this record's own build costs, so the
	// build refuses whichever way it gets its walks.
	whole := newFreeformWork()
	_, err = evalPrism(New(), 0, withoutWalks(source), whole)
	require.NoError(t, err)
	require.Greater(t, whole.spent, uint64(0), "premise: this record charges real work")

	starve := func() *freeformWork {
		w := newFreeformWork()
		require.NoError(t, w.step(freeformWorkLimit-whole.spent+1))
		return w
	}

	reusedWork := starve()
	_, reusedErr := evalPrism(New(), 0, source, reusedWork)
	require.ErrorIs(t, reusedErr, ErrUnsupported, "the reused build must refuse at the ceiling")

	plainWork := starve()
	_, plainErr := evalPrism(New(), 0, withoutWalks(source), plainWork)
	require.ErrorIs(t, plainErr, ErrUnsupported, "so must the build that does the work")
	require.Equal(t, plainErr.Error(), reusedErr.Error(),
		"and both must refuse with the same error, not merely the same class")
}

// TestUnmeteredWalksAreNeverReused pins the guard on a resolution that cannot
// state its own charge. loft_stations.go builds such a view over walks its own
// pairing already charged; a set like it must never be replayed, because
// replaying a charge it never measured would hand the build free work.
func TestUnmeteredWalksAreNeverReused(t *testing.T) {
	t.Parallel()
	profile := involuteFitProfile()
	metered, err := resolveProfileWalks(profile, newFreeformWork())
	require.NoError(t, err)

	view := &profileWalks{profile: profile, outer: metered.outer, holes: metered.holes}
	require.True(t, view.matches(profile), "premise: the view is over this very record")
	require.False(t, view.reusable(profile), "an unmetered set is never read back")
	require.ErrorIs(t, view.charge(newFreeformWork()), errUnmeteredWalksCharge)
	require.ErrorIs(t, view.charge(newFreeformWork()), ErrUnsupported)

	require.True(t, metered.reusable(profile), "the metered set it was built from still reads back")
	require.NoError(t, metered.charge(newFreeformWork()))
}

// TestPublishedWalksAreReadOnlyAcrossGoroutines runs two independent
// re-evaluations over one published resolution at the same time, in separate
// documents. The resolution is shared read-only by construction — every walk in
// it is plane-local and nothing downstream writes back through it — and this is
// the test that says so under -race.
func TestPublishedWalksAreReadOnlyAcrossGoroutines(t *testing.T) {
	t.Parallel()
	built, err := evalPrism(New(), 0, involuteFitPrismPayload(t), newFreeformWork())
	require.NoError(t, err)
	source := prismPayloadOf(t, built)

	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(math.Pi/3))
	require.NoError(t, err)

	type outcome struct {
		body *Body
		err  error
	}
	shift, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	results := make(chan outcome, 2)
	for _, motion := range []r3.Transform{turn, shift} {
		go func() {
			pp := source
			pp.xform = motion
			body, err := evalPrismContext(t.Context(), New(), 0, pp, newFreeformWork())
			results <- outcome{body: body, err: err}
		}()
	}
	for range 2 {
		got := <-results
		require.NoError(t, got.err)
		require.Same(t, source.walks, prismPayloadOf(t, got.body).walks,
			"both concurrent builds read the one shared resolution")
		require.Equal(t, built.volume, got.body.volume, "a rigid motion changes no volume term")
	}
}

// benchInvoluteFitPayload is involuteFitPrismPayload built against *testing.B.
func benchInvoluteFitPayload(b *testing.B) prismPayload {
	b.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(b, err)
	return prismPayload{profile: involuteFitProfile(), frame: frame, z1: 5, xform: r3.Identity()}
}

// BenchmarkPlacedFreeformPrism measures the placement this reuse is for: a
// rigid re-evaluation of an involute fit-spline section, once reading back the
// resolution the original build published and once resolving every walk again.
// The gap between the two arms is the arc-length bracketing the reuse skips,
// and it is the only difference between them — both build the same body, and
// both charge their counter the same total.
func BenchmarkPlacedFreeformPrism(b *testing.B) {
	built, err := evalPrism(New(), 0, benchInvoluteFitPayload(b), newFreeformWork())
	require.NoError(b, err)
	source, ok := built.payload.(prismPayload)
	require.True(b, ok)
	turn, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Radians(math.Pi/3))
	require.NoError(b, err)

	for _, arm := range []struct {
		name    string
		payload prismPayload
	}{
		{name: "reusing the published resolution", payload: source},
		{name: "resolving every walk again", payload: withoutWalks(source)},
	} {
		b.Run(arm.name, func(b *testing.B) {
			pp := arm.payload
			pp.xform = turn
			for b.Loop() {
				if _, err := evalPrism(New(), 0, pp, newFreeformWork()); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
