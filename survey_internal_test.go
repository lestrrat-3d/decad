package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is a deliberate internal-test exception (like
// selector_internal_test.go): the kernel's sub-resolution web semantic is
// unreachable through the public API — sketch's arrangement refuses the
// near-identical concentric circles that would carry a web under the
// kernel's candidate floor — yet the kernel must still never let such a
// candidate underwrite a pass if a future geometry source produces one.

func TestWallKernelFlagsOffJunctionSubTolerance(t *testing.T) {
	// Two concentric full circles 2e-8 apart at scale 10: the web disk is
	// under the candidate floor (4·1e-9·10) and no junction vertex is
	// supplied, so the kernel must flag it rather than silently treat the
	// boundary as web-free.
	outer, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := arcElem(0, 0, 10-2e-8, 2*math.Pi, 0, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{outer, inner}, nil, math.Inf(1))
	out := k.run()
	require.True(t, out.subTolFar, `an off-junction sub-tolerance candidate must be flagged`)
}

func TestWallKernelCleanProfileDoesNotFlag(t *testing.T) {
	// A plain 100×60 rectangle produces no off-junction sub-tolerance
	// candidates: the flag stays clear and the reading is decided.
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, 4)
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%4][0], pts[(i+1)%4][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, pts, math.Inf(1))
	out := k.run()
	require.True(t, out.ok)
	require.False(t, out.subTolFar)
	require.True(t, out.hasSpan)
	require.InDelta(t, 60.0, out.span, 1e-9)
}

func TestWallKernelGenerateStreamsCandidates(t *testing.T) {
	arc, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{arc}, nil, math.Inf(1))
	stop := errors.New("stop after the first candidate")
	seen := 0

	err := k.generate(nil, func(diskCand) error {
		seen++
		return stop
	})

	require.ErrorIs(t, err, stop)
	require.Equal(t, 1, seen)
}

func TestWallKernelGenerateCancellationIsBounded(t *testing.T) {
	arc, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		elems[i] = arc
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	seen := 0

	err := newWallKernel(elems, nil, math.Inf(1)).
		generate(newWorkBudget(ctx), func(diskCand) error {
			seen++
			return nil
		})

	require.ErrorIs(t, err, context.Canceled)
	require.LessOrEqual(t, seen, workPollInterval-1)
}

func TestWallKernelSetupCancellationIsBounded(t *testing.T) {
	arc, ok := arcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		elems[i] = arc
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "newWallKernelBudget"}

	_, err := newWallKernelBudget(newWorkBudget(ctx), elems, nil, nil, 15*math.Pi/180, exactScalar(0), false, math.Inf(1))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `wall-kernel boundary sizing must poll inside its scan`)
}

func TestWallKernelValidateCancellationIsBounded(t *testing.T) {
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		e, ok := lineElem(100, float64(i+1), 101, float64(i+1))
		require.True(t, ok)
		elems[i] = e
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "validate"}
	k := newWallKernel(elems, nil, math.Inf(1))

	spanning, empty, valid, err := k.validate(diskCand{x: 0, y: 0, r: 1}, newWorkBudget(ctx))
	_ = spanning
	_ = empty
	_ = valid

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestWallKernelContainsCancellationIsBounded(t *testing.T) {
	elems := make([]surveyElem, workPollInterval+64)
	for i := range elems {
		e, ok := arcElem(1000+float64(i), 1000, 1, 0, 2*math.Pi, true)
		require.True(t, ok)
		elems[i] = e
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "contains"}
	k := newWallKernel(elems, nil, math.Inf(1))

	_, _, err := k.contains(0, 0, newWorkBudget(ctx))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestWallKernelBudgetedRunKeepsNormalResult(t *testing.T) {
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, len(pts))
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, pts, math.Inf(1))

	out, err := k.runBudget(newWorkBudget(t.Context()))

	require.NoError(t, err)
	require.True(t, out.ok)
	require.True(t, out.hasSpan)
	require.InDelta(t, 60.0, out.span, 1e-9)
}

// TestWallKernelPublishesDiameterBounds pins the unit relation both wall-survey
// consumers inherit: prismWall and revolveWall each publish out.span, a
// DIAMETER, beside max(out.spanBound, out.maxCandBound), so those two fields
// must be bounds on that diameter and not on the candidate radius each
// generator actually bounded. The re-derivation below reads the generators
// directly and asserts the exact factor of two, which no consumer restates.
func TestWallKernelPublishesDiameterBounds(t *testing.T) {
	// A 100×60 outline with two r=1 circular holes 3√2 apart: the web between
	// them is the arcArcCands centreline candidate, whose division carries a
	// nonzero radius bound.
	elems := func() []surveyElem {
		pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
		out := make([]surveyElem, 0, len(pts)+2)
		for i := range pts {
			e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
			require.True(t, ok)
			out = append(out, e)
		}
		for _, c := range [][2]float64{{48.5, 28.5}, {51.5, 31.5}} {
			// Reversed angle order: a hole keeps the material on its left.
			e, ok := arcElem(c[0], c[1], 1, 2*math.Pi, 0, true)
			require.True(t, ok)
			out = append(out, e)
		}
		return out
	}

	out := newWallKernel(elems(), nil, math.Inf(1)).run()
	require.True(t, out.ok)
	require.True(t, out.hasSpan)
	require.Greater(t, out.spanBound, 0.0)

	// Re-derive the same population from the generators, whose diskCand.rBound
	// speaks for the RADIUS. A fresh kernel keeps the streamed run above from
	// colouring this pass.
	k := newWallKernel(elems(), nil, math.Inf(1))
	bestR, winnerRBound, maxRBound := math.Inf(1), 0.0, 0.0
	require.NoError(t, k.generate(nil, func(c diskCand) error {
		spanning, empty, ok, err := k.validate(c, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if !spanning || !empty {
			return nil
		}
		if c.rBound > maxRBound {
			maxRBound = c.rBound
		}
		if c.r < bestR {
			bestR, winnerRBound = c.r, c.rBound
		}
		return nil
	}))
	require.Greater(t, winnerRBound, 0.0)

	require.Equal(t, 2*bestR, out.span)
	require.Equal(t, 2*winnerRBound, out.spanBound)
	require.Equal(t, 2*maxRBound, out.maxCandBound)
}

// TestWallKernelConcentricSpanBoundsRadiusDifference pins the concentric
// annulus candidate at the kernel's own door, with the two radii handed
// straight in. R = 1e16 and r = 1 are both held exactly, yet their difference
// 9999999999999999 is not a float64 at all: the kernel's held span rounds up
// to 1e16, a full millimetre above the truth. A candidate that reported bound
// zero there published a zero-width interval that excluded its own answer.
func TestWallKernelConcentricSpanBoundsRadiusDifference(t *testing.T) {
	outer, ok := arcElem(0, 0, 1e16, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := arcElem(0, 0, 1, 2*math.Pi, 0, true)
	require.True(t, ok)

	out := newWallKernel([]surveyElem{outer, inner}, nil, math.Inf(1)).run()

	require.True(t, out.ok)
	require.True(t, out.hasSpan)
	truth := new(big.Rat).SetInt64(9999999999999999)
	require.NotEqual(t, 0, new(big.Rat).SetFloat64(out.span).Cmp(truth),
		`the held span must miss the truth, or this fixture proves nothing`)
	requireRatInInterval(t, truth, out.span, math.Max(out.spanBound, out.maxCandBound))
}

// requireRatInInterval asserts that truth lies in [value − bound,
// value + bound], compared over big.Rat so neither endpoint is formed by a
// float subtraction whose own rounding could swamp the bound under test.
func requireRatInInterval(t *testing.T, truth *big.Rat, value, bound float64) {
	t.Helper()
	v := new(big.Rat).SetFloat64(value)
	b := new(big.Rat).SetFloat64(bound)
	require.NotNil(t, v)
	require.NotNil(t, b)
	require.LessOrEqual(t, new(big.Rat).Sub(v, b).Cmp(truth), 0,
		`the published interval must reach down to the truth`)
	require.GreaterOrEqual(t, new(big.Rat).Add(v, b).Cmp(truth), 0,
		`the published interval must reach up to the truth`)
}

// TestWholeArcCandidateCarriesArcRadiusBound pins the concentric whole-arc
// disk, the one candidate whose radius IS the element's radius with no
// arithmetic between them. An ArcSeg states its Start and Center, never its
// radius, so the walk's radius is a math.Hypot evaluation and the candidate
// must publish the walk's bound on it rather than zero.
func TestWholeArcCandidateCarriesArcRadiusBound(t *testing.T) {
	// Centre (10, 9) one unit from start (9, 10) in each of u and v: the true
	// radius is √2, which no float64 holds.
	seg := ArcSeg{
		Center: Point2{U: 10, V: 9},
		Start:  Point2{U: 9, V: 10},
		End:    Point2{U: 11, V: 10},
		TStart: 0,
		TEnd:   1,
	}
	w, err := walkOf(seg, newFreeformWork())
	require.NoError(t, err)
	require.Greater(t, w.radiusBound, 0.0, `a hypot radius is never exact`)
	e, ok := walkElem(w)
	require.True(t, ok)
	require.Equal(t, w.radiusBound, e.rrBound, `the element must take the walk's own bound`)
	require.True(t, e.matInside, `the fixture must walk counter-clockwise, or no whole-arc disk is emitted`)

	// generate opens with the whole-arc disks, so the first candidate is the
	// one under test (TestWallKernelGenerateStreamsCandidates pins the order).
	k := newWallKernel([]surveyElem{e}, nil, math.Inf(1))
	var first *diskCand
	require.NoError(t, k.generate(nil, func(c diskCand) error {
		if first == nil {
			cand := c
			first = &cand
		}
		return nil
	}))
	require.NotNil(t, first)
	require.Equal(t, e.rr, first.r)
	require.Equal(t, e.rrBound, first.rBound)

	// √2 to 200 bits: the published interval must reach it.
	const prec = 200
	sqrt2 := new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).SetInt64(2))
	truth, _ := sqrt2.Rat(nil)
	requireRatInInterval(t, truth, first.r, first.rBound)
}

// TestWedgeCandidateCarriesCapSineBound pins the cap sine, one of the two
// kernel inputs that are not exact leaves (an element's own radius is the
// other, pinned by TestWholeArcCandidateCarriesArcRadiusBound). A 60° sweep's
// cap half-angle is 30°, whose TRUE sine is
// exactly 1/2, but the held float64 sine is a hair under it — so the wedge
// candidate at a meridian vertex y = 10 holds a radius a hair under the exact
// s·y/(1−s) = 10. A candidate that read the sine as an exact leaf published
// bound zero over three rounded operations and so published an interval that
// missed its own value.
func TestWedgeCandidateCarriesCapSineBound(t *testing.T) {
	sweep, err := units.Degrees(60).In(units.Radian)
	require.NoError(t, err)
	sinBS, _ := radianTrigBounds(sweep / 2)
	require.NotEqual(t, 0.5, sinBS.value, `the held sine is not the true sin(30°)`)
	require.Greater(t, sinBS.bound, 0.0)
	require.LessOrEqual(t, math.Abs(sinBS.value-0.5), sinBS.bound,
		`the certified trig interval must contain the true sine`)

	el, ok := lineElem(0, 10, 1, 10)
	require.True(t, ok)
	k, kerr := newWallKernelBudget(nil, []surveyElem{el}, nil, [][2]float64{{0, 10}},
		15*math.Pi/180, sinBS, false, math.Inf(1))
	require.NoError(t, kerr)

	var near *diskCand
	require.NoError(t, k.wedgeCands(nil, func(c diskCand) error {
		if math.Abs(c.r-10) < 1e-6 {
			cand := c
			near = &cand
		}
		return nil
	}))
	require.NotNil(t, near, `the y = 10 vertex must produce the r ≈ 10 wedge candidate`)
	require.Greater(t, near.rBound, 0.0, `a sine-derived radius is never exact`)
	require.LessOrEqual(t, math.Abs(near.r-10), near.rBound,
		`the published radius interval must contain the exact s·y/(1−s) = 10`)
}

// TestSolve3LinearBoundCoversCramerArithmetic pins that a three-linear
// Apollonius triple bounds the WHOLE of Cramer's rule and not just its final
// division. Each numerator rounds six products and three sums before the
// quotient runs, so a bound taken from the division alone can be an order of
// magnitude short of the error it must cover.
func TestSolve3LinearBoundCoversCramerArithmetic(t *testing.T) {
	// A non-axis-aligned trapezoid: four line elements, so every triple of
	// their tangency equations is three linears and lands in solve3Linear.
	pts := [][2]float64{{0, 0}, {20.3, 1.7}, {16.1, 9.4}, {3.7, 7.9}}
	elems := make([]surveyElem, 0, len(pts))
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, pts, math.Inf(1))
	eqs, err := k.tripleEquations(nil)
	require.NoError(t, err)
	lins := make([]circEq, 0, len(elems))
	for _, e := range eqs {
		if !e.quad {
			lins = append(lins, e)
		}
	}
	require.Len(t, lins, len(elems))

	seen, shortUnderOldRule := 0, 0
	for i := range lins {
		for j := i + 1; j < len(lins); j++ {
			for l := j + 1; l < len(lins); l++ {
				triple := []circEq{lins[i], lins[j], lins[l]}
				solve3Linear(triple, func(_, _, r, rBound float64) {
					seen++
					exact := exactCramerRadius(triple)
					require.NotNil(t, exact)
					gap := ratAbsDiff(exact, r)
					require.LessOrEqual(t, gap, rBound,
						`the published radius interval must contain the exact Cramer answer`)
					if gap > divisionOnlyRadiusBound(triple) {
						shortUnderOldRule++
					}
				})
			}
		}
	}
	require.Positive(t, seen, `the trapezoid must produce three-linear roots`)
	require.Positive(t, shortUnderOldRule,
		`at least one root's error must exceed what its final division alone accounts for`)
}

// exactCramerRadius evaluates dr/det over big.Rat from the SAME held
// coefficients solve3Linear reads, so the comparison is against the exact
// answer to the very system the kernel solved, never against another float64
// evaluation of it.
func exactCramerRadius(l []circEq) *big.Rat {
	r := func(v boundedScalar) *big.Rat { return floatRat(v.value) }
	minor := func(p, q, s, t *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(p, t), new(big.Rat).Mul(s, q))
	}
	a := [3]*big.Rat{r(l[0].a), r(l[1].a), r(l[2].a)}
	b := [3]*big.Rat{r(l[0].b), r(l[1].b), r(l[2].b)}
	e := [3]*big.Rat{r(l[0].e), r(l[1].e), r(l[2].e)}
	f := [3]*big.Rat{r(l[0].f), r(l[1].f), r(l[2].f)}
	for _, col := range [][3]*big.Rat{a, b, e, f} {
		for _, v := range col {
			if v == nil {
				return nil
			}
		}
	}
	be := minor(b[1], e[1], b[2], e[2])
	ae := minor(a[1], e[1], a[2], e[2])
	ab := minor(a[1], b[1], a[2], b[2])
	af := minor(a[1], f[1], a[2], f[2])
	bf := minor(b[1], f[1], b[2], f[2])
	det := new(big.Rat).Add(
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], be), new(big.Rat).Mul(b[0], ae)),
		new(big.Rat).Mul(e[0], ab))
	if det.Sign() == 0 {
		return nil
	}
	dr := new(big.Rat).Sub(
		new(big.Rat).Add(
			new(big.Rat).Mul(new(big.Rat).Neg(a[0]), bf),
			new(big.Rat).Mul(b[0], af)),
		new(big.Rat).Mul(f[0], ab))
	return new(big.Rat).Quo(dr, det)
}

// divisionOnlyRadiusBound is the bound a rule that read every Cramer
// determinant as an exact leaf would publish: the final division's own
// rounding and nothing above it. The test uses it only to prove the fixture
// discriminates — never as an assertion about the shipped rule.
func divisionOnlyRadiusBound(l []circEq) float64 {
	minor := func(p, q, s, t float64) float64 { return p*t - s*q }
	be := minor(l[1].b.value, l[1].e.value, l[2].b.value, l[2].e.value)
	ae := minor(l[1].a.value, l[1].e.value, l[2].a.value, l[2].e.value)
	ab := minor(l[1].a.value, l[1].b.value, l[2].a.value, l[2].b.value)
	af := minor(l[1].a.value, l[1].f.value, l[2].a.value, l[2].f.value)
	bf := minor(l[1].b.value, l[1].f.value, l[2].b.value, l[2].f.value)
	det := l[0].a.value*be - l[0].b.value*ae + l[0].e.value*ab
	dr := -l[0].a.value*bf + l[0].b.value*af - l[0].f.value*ab
	return boundedQuotient(dr, 0, det, 0).bound
}

func TestPrismWallSubToleranceWebIsUndecided(t *testing.T) {
	// The prism path must apply the same rule as the revolve path: a
	// near-concentric annular profile whose 2e-8 web sits under the kernel
	// floor reads undecided — never a proven absence or a positive wall.
	pp := prismPayload{
		profile: ProfileRecord{
			Outer: LoopRecord{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1},
			}},
			Holes: []LoopRecord{{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10 - 2e-8), CCW: false, TStart: 1, TEnd: 0},
			}}},
		},
		z0: 0, z1: 10,
	}
	out, err := prismWall(newWorkBudget(t.Context()), pp, 15*math.Pi/180)
	require.NoError(t, err)
	require.False(t, out.ok, `undecided, never a silent pass`)
}

func TestCupWallRequiresExactMorphology(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{
			Start:  Point2{U: u0, V: v0},
			End:    Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	outer := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		line(0, 0, 100, 0),
		line(100, 0, 100, 60),
		line(100, 60, 0, 60),
		line(0, 60, 0, 0),
	}}}
	cavity, err := offsetProfileBudget(newWorkBudget(t.Context()), outer, 1, 5)
	require.NoError(t, err)
	cp := cupPayload{
		outer:     outer,
		cavity:    cavity,
		zOuter:    0,
		zCav:      5,
		zOpen:     20,
		thickness: 5,
		sense:     Inward,
	}

	out, err := cupWall(newWorkBudget(t.Context()), cp, 15*math.Pi/180)
	require.NoError(t, err)
	require.True(t, out.ok)
	require.NotNil(t, out.reading)
	require.Equal(t, 5.0, *out.reading)

	// Move the stored cavity sideways from the exact five-millimetre offset.
	// The loop stays closed with the same positive area and loop count, but the
	// morphology certificate no longer holds, so the survey stays undecided.
	bad := cp
	bad.cavity.Outer.Segments = append([]CurveSegment(nil), cp.cavity.Outer.Segments...)
	for i, seg := range bad.cavity.Outer.Segments {
		moved := seg.(LineSeg)
		moved.Start.U += 0.25
		moved.End.U += 0.25
		bad.cavity.Outer.Segments[i] = moved
	}

	out, err = cupWall(newWorkBudget(t.Context()), bad, 15*math.Pi/180)
	require.NoError(t, err)
	require.False(t, out.ok, `a malformed offset relation must not return the recipe thickness`)

	body := &Body{payload: bad}
	br := BodyReport{Body: body, Solid: true}
	diags, err := runSurveys(newWorkBudget(t.Context()), &br, verifyConfig{
		wall:     &wallSpec{tool: units.Millimeters(1)},
		toolMM:   1,
		allowRad: 15 * math.Pi / 180,
	})
	require.NoError(t, err)
	require.Nil(t, br.MinWallThickness)
	require.Len(t, diags, 1)
	require.Equal(t, DiagUndecidedWall, diags[0].Code)
	require.NotEqual(t, DiagUnsupportedSurveyPayload, diags[0].Code)
	require.Equal(t, Suspect, diags[0].Status)
	require.NotContains(t, diags[0].Message, "facetedPayload",
		`an undecided analytic survey must not be reported as an unsupported payload`)
}

func manySegmentProfile(segmentCount int) ProfileRecord {
	segs := make([]CurveSegment, segmentCount)
	for i := range segmentCount {
		th0 := 2 * math.Pi * float64(i) / float64(segmentCount)
		th1 := 2 * math.Pi * float64(i+1) / float64(segmentCount)
		segs[i] = LineSeg{
			Start:  Point2{U: 100 * math.Cos(th0), V: 100 * math.Sin(th0)},
			End:    Point2{U: 100 * math.Cos(th1), V: 100 * math.Sin(th1)},
			TStart: 0,
			TEnd:   1,
		}
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

func newFrameWorkBudget(target string) (*workBudget, *bool) {
	entered := false
	cancelled := false
	inTarget := func() bool {
		pcs := make([]uintptr, 32)
		frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
		for {
			frame, more := frames.Next()
			if strings.HasSuffix(frame.Function, "."+target) {
				return true
			}
			if !more {
				return false
			}
		}
	}
	cancelInTarget := func() error {
		if cancelled {
			return context.Canceled
		}
		if !inTarget() {
			return nil
		}
		entered = true
		cancelled = true
		return context.Canceled
	}
	return &workBudget{stepFn: cancelInTarget, errFn: cancelInTarget}, &entered
}

func TestRecordLoopsCancellationIsBounded(t *testing.T) {
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "recordLoops"}
	_, err := recordLoops(newWorkBudget(ctx), manySegmentProfile(workPollInterval+64))
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `profile segment resolution must poll inside recordLoops`)
}

func TestRevolveLoopsCancellationIsBounded(t *testing.T) {
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "revolveLoops"}
	_, err := revolveLoops(newWorkBudget(ctx), revolvePayload{
		profile: manySegmentProfile(workPollInterval + 64),
		ax:      axisFrame{dU: 1},
	})
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `profile segment resolution must poll inside revolveLoops`)
}

func TestCupWallCancellationCoversOffsetAuditAndReverse(t *testing.T) {
	t.Parallel()
	outer := manySegmentProfile(workPollInterval + 64)
	cavity, err := offsetProfile(nil, outer, 1, 5)
	require.NoError(t, err)
	cp := cupPayload{
		outer:     outer,
		cavity:    cavity,
		zOuter:    0,
		zCav:      5,
		zOpen:     20,
		thickness: 5,
		sense:     Inward,
	}
	out, err := cupWall(newWorkBudget(t.Context()), cp, 15*math.Pi/180)
	require.NoError(t, err)
	require.True(t, out.ok)

	for _, target := range []string{"coalesceWalksBudget", "crossingAuditBudget", "reverseLoopRecordBudget", "loopRecordsEqual"} {
		t.Run(target, func(t *testing.T) {
			budget, entered := newFrameWorkBudget(target)
			_, err := cupWall(budget, cp, 15*math.Pi/180)
			require.ErrorIs(t, err, context.Canceled)
			require.True(t, *entered, `cup wall work must poll inside the named phase`)
		})
	}
}

func TestCupWallCancellationDuringProfileIntegrals(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{
			Start:  Point2{U: u0, V: v0},
			End:    Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	outer := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		line(0, 0, 100, 0),
		line(100, 0, 100, 60),
		line(100, 60, 0, 60),
		line(0, 60, 0, 0),
	}}}
	cavity, err := offsetProfile(newWorkBudget(t.Context()), outer, 1, 5)
	require.NoError(t, err)
	cp := cupPayload{
		outer:     outer,
		cavity:    cavity,
		zOuter:    0,
		zCav:      5,
		zOpen:     20,
		thickness: 5,
		sense:     Inward,
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "integralsBudget"}

	_, err = cupWall(newWorkBudget(ctx), cp, 15*math.Pi/180)

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}
