package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
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
// consumers inherit: prismWall and revolveWall each publish out.span beside
// out.spanBound as a DIAMETER and its half-width, so both must speak in the
// diameter and not in the candidate radius each generator actually bounded.
// The re-derivation below reads the generators directly, folds their radii
// through the same §9.2 aggregate at twice their value and twice their bound,
// and asserts the kernel published exactly that — the factor of two, which no
// consumer restates, and the aggregate, which no candidate decides alone.
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
	agg := minAggregate()
	winnerValue, winnerBound := math.Inf(1), 0.0
	sawBounded := false
	require.NoError(t, k.generate(nil, func(c diskCand) error {
		spanning, empty, ok, err := k.validate(c, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if !spanning || !empty {
			return nil
		}
		if c.rBound > 0 {
			sawBounded = true
		}
		// The doubling is the production one, so the assertion below is about
		// the aggregate and not about a re-spelling of boundedMul's own
		// outward rounding.
		diam := boundedMul(exactScalar(2), measuredScalar(c.r, c.rBound))
		require.Equal(t, 2*c.r, diam.value, `the answer is a diameter, not a radius`)
		require.GreaterOrEqual(t, diam.bound, 2*c.rBound, `the bound doubles with the value`)
		agg.take(diam.value, diam.bound)
		if diam.value < winnerValue {
			winnerValue, winnerBound = diam.value, diam.bound
		}
		return nil
	}))
	require.True(t, sawBounded, `the web candidate's own division must carry a nonzero bound`)

	wantSpan, wantBound, ok := agg.resolve()
	require.True(t, ok)
	require.Equal(t, wantSpan, out.span)
	require.Equal(t, wantBound, out.spanBound)

	// The fixture must discriminate: a winner-only reduction — the smallest
	// held diameter beside that one candidate's own bound — must publish a
	// DIFFERENT interval here, or the assertion above would hold under the
	// defective rule too.
	require.NotEqual(t, winnerBound, out.spanBound,
		`the population must contain a rival whose bound the winner's own does not cover`)
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
	requireRatInInterval(t, truth, out.span, out.spanBound)
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

// TestWallCandidateExactChainsStayExact pins the kernel's other two
// boundedHypot readings — the arc-arc centre separation and the vertex-arc
// one — at a separation a float64 holds exactly, where the whole chain into
// the candidate's radius is exact and the candidate must publish a zero
// bound. Neither can be pinned through a published READING: each family also
// emits its angle-limit siblings, whose radii come from the draft
// allowance's own certified trig enclosure, and the reading's bound is the
// largest over every spanning candidate (wallSurveyOut.maxCandBound), so an
// arc anywhere in the section keeps the reading Approximate.
func TestWallCandidateExactChainsStayExact(t *testing.T) {
	// Two r=1 hole walls whose centres sit a 3-4-5 distance apart: the web
	// between them is 5 − 1 − 1 = 3, so the centreline candidate's radius is
	// exactly 1.5. Reversed angle order keeps the material on each hole's left.
	holeA, ok := arcElem(48.5, 28.5, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	holeB, ok := arcElem(51.5, 32.5, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	k := newWallKernel([]surveyElem{holeA, holeB}, nil, math.Inf(1))

	collect := func(gen func(add func(x, y, r, rBound float64))) map[float64]float64 {
		out := map[float64]float64{}
		gen(func(_, _, r, rBound float64) {
			if prev, seen := out[r]; !seen || rBound > prev {
				out[r] = rBound
			}
		})
		return out
	}

	arcArc := collect(func(add func(x, y, r, rBound float64)) {
		k.arcArcCands(holeA, holeB, add)
	})
	arcArcBound, found := arcArc[1.5]
	require.True(t, found, `the 3-4-5 web must produce the r = 1.5 centreline candidate`)
	require.Equal(t, 0.0, arcArcBound, `an exactly representable web chain is exact`)

	// The same separation from a junction vertex to a hole centre: the neck is
	// 5 − 1 = 4, so the vertex-arc centreline candidate's radius is exactly 2.
	vertexArc := collect(func(add func(x, y, r, rBound float64)) {
		k.vertexElemCands([2]float64{40, 30}, holeAt(t, 37, 26), add)
	})
	vertexArcBound, found := vertexArc[2]
	require.True(t, found, `the 3-4-5 neck must produce the r = 2 centreline candidate`)
	require.Equal(t, 0.0, vertexArcBound, `an exactly representable neck chain is exact`)
}

// holeAt is a unit-radius hole wall centred at (qx, qy), walked so the
// material stays on its left.
func holeAt(t *testing.T, qx, qy float64) surveyElem {
	t.Helper()
	e, ok := arcElem(qx, qy, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	return e
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

// TestWallKernelValidateReadsTheCandidateInterval pins the three guards
// validate takes on a candidate's own proven radius interval rather than on
// its held radius: emptiness, contact, and the spanning angle that quantifies
// over the contact set. Each rejects only what the interval PROVES, so a
// candidate whose bound reaches across the guard is kept and its doubt travels
// into the reading — the resolution validate's own doc comment derives, and
// the one the held comparisons could not express at all.
//
// A held comparison decides both crafted candidates below the wrong way: the
// first is held blocked by 2 µm on a section whose candidate bound is 3 µm, and
// the second is held clear of contact by the same margin. Both really do sit
// inside the interval their own arithmetic admits.
func TestWallKernelValidateReadsTheCandidateInterval(t *testing.T) {
	// A 40×10 rectangle walked counter-clockwise: material inside, the disk at
	// its centre reaching 5 mm to the two long skins and 20 mm to the ends.
	pts := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {0, 10}}
	elems := make([]surveyElem, 0, len(pts))
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	k := newWallKernel(elems, pts, math.Inf(1))
	require.Less(t, k.tol, 1e-6, `the crafted bound must sit above the kernel's declared slack`)

	check := func(name string, c diskCand, wantSpanning, wantEmpty bool) {
		t.Helper()
		spanning, empty, ok, err := k.validate(c, nil)
		require.NoError(t, err, name)
		require.True(t, ok, name)
		require.Equal(t, wantEmpty, empty, "%s: emptiness", name)
		require.Equal(t, wantSpanning, spanning, "%s: spanning", name)
	}

	// Held blocked by 2 µm, but the candidate's own 3 µm bound reaches back
	// past the two skins: an empty disk really does sit at this centre.
	check("held blocked, interval straddles",
		diskCand{x: 20, y: 5, r: 5 + 2e-6, rBound: 3e-6}, true, true)
	// Held clear of contact by 2 µm under the same bound: the two skins really
	// can touch a disk in this interval, which is what makes it span.
	check("held clear of contact, interval straddles",
		diskCand{x: 20, y: 5, r: 5 - 2e-6, rBound: 3e-6}, true, true)

	// The controls: a candidate PROVEN to cut into the skins is still dropped,
	// and one PROVEN clear of them still spans nothing.
	check("proven blocked", diskCand{x: 20, y: 5, r: 6, rBound: 1e-12}, false, false)
	check("proven clear of every skin", diskCand{x: 20, y: 5, r: 1, rBound: 1e-12}, false, true)
}

// TestWallKernelInradiusAggregatesRivalCandidates pins the kernel's OTHER
// extremum on the same §9.2 rule as the span. The inradius is a maximum, so it
// reduces the other way — the greatest of the candidates' lower ends beside the
// greatest of their upper ends — and a rival whose interval reaches ABOVE the
// winning held value is exactly what a comparison of held values cannot see.
func TestWallKernelInradiusAggregatesRivalCandidates(t *testing.T) {
	// A 100×60 plate holding one r=22 circular hole: the trig-derived
	// angle-limit candidates around the hole carry bounds far above the
	// kernel's declared slack, so the population really does have rivals.
	pts := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]surveyElem, 0, len(pts)+1)
	for i := range pts {
		e, ok := lineElem(pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	hole, ok := arcElem(50, 30, 22, 2*math.Pi, 0, true)
	require.True(t, ok)
	elems = append(elems, hole)

	out := newWallKernel(elems, pts, math.Inf(1)).run()
	require.True(t, out.ok)

	k := newWallKernel(elems, pts, math.Inf(1))
	agg := maxAggregate()
	heldMax, heldMaxBound := math.Inf(-1), 0.0
	require.NoError(t, k.generate(nil, func(c diskCand) error {
		_, empty, ok, err := k.validate(c, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if !empty {
			return nil
		}
		agg.take(c.r, c.rBound)
		if c.r > heldMax {
			heldMax, heldMaxBound = c.r, c.rBound
		}
		return nil
	}))
	wantValue, wantBound, resolved := agg.resolve()
	require.True(t, resolved)
	require.Equal(t, wantValue, out.inradius)
	require.Equal(t, wantBound, out.inradiusBound)

	// The fixture must discriminate: reducing by held value alone — the
	// greatest held radius beside that one candidate's own bound — publishes a
	// different interval here.
	require.NotEqual(t, heldMaxBound, out.inradiusBound,
		`the population must contain a rival the winner's own bound does not cover`)
	require.Greater(t, out.inradiusBound, 0.0)
}

// TestWallKernelFitGateReadsTheCandidateInterval pins the third of runBudget's
// readings on the candidate's own interval: whether a spanning disk is narrow
// enough to lift out of the section into the sweep. The gate rejects only a
// disk PROVEN wider than the sweep, because dropping one whose interval still
// reaches under the height would drop a wall the body may really have — the
// one error a wall reading must never make.
//
// The fixture is an annulus whose outer radius arrives with a 2 µm bound of its
// own, as an ArcSeg-derived element does, against a sweep 1 µm under the
// annulus's held 10 mm web. A held comparison finds the only spanning disk too
// wide and reports the body wall-free.
func TestWallKernelFitGateReadsTheCandidateInterval(t *testing.T) {
	outer, ok := arcElem(0, 0, 20, 0, 2*math.Pi, true)
	require.True(t, ok)
	outer.rrBound = 2e-3
	inner, ok := arcElem(0, 0, 10, 2*math.Pi, 0, true)
	require.True(t, ok)

	fitMax := 10 - 1e-3
	k, err := newWallKernelBudget(nil, []surveyElem{outer, inner}, nil, nil,
		15*math.Pi/180, exactScalar(0), false, fitMax)
	require.NoError(t, err)
	out, err := k.runBudget(nil)
	require.NoError(t, err)

	require.True(t, out.ok)
	require.True(t, out.hasSpan,
		`the only spanning disk's interval still reaches under the sweep, so it is a wall`)
	require.Greater(t, out.span, fitMax, `the held diameter must exceed the sweep, or the gate is untested`)
	require.Less(t, out.span-out.spanBound, fitMax,
		`the interval must reach under the sweep, or the gate has nothing to straddle`)
}

// TestWallKernelStraddledLeadingCoefficientStaysDecided pins the one denominator
// that can silence a whole survey: an Apollonius triple whose 2A the arithmetic
// cannot separate from zero. boundedQuotient answers +Inf there, the aggregate
// refuses an unbounded candidate, and the survey that would have published a
// number reads undecided instead. quadRootsBounded answers such a triple with
// the degenerate linear root −C/B, whose own denominator is well separated, so
// the root a straddling A is meant to recover arrives bounded.
//
// The fixture is at KERNEL level deliberately. The family needs a determinant
// near enough to singular to throw the affine centre out to 1e19–1e21, and a
// centre that far away is outside every closed section a body can be built
// from, so the containment scan drops the candidate before it reaches the
// aggregate: the defect is unreachable through the public API and only the
// kernel's own door exposes it. Three material-inside circles with
// near-collinear centres put the triple's leading coefficient at
// 3.4e+24 ± 4.8e+24 while the pair determinant is still admitted, which is
// exactly that state.
func TestWallKernelStraddledLeadingCoefficientStaysDecided(t *testing.T) {
	elems := func() []surveyElem {
		out := make([]surveyElem, 0, 3)
		for _, c := range [][3]float64{
			{8255.0884454771622, 4195.7682375914947, 149.98644153683318},
			{2475.9000902096891, -11681.738286473166, 131.34705082306061},
			{-959.14779827151824, -21119.048840666175, 104.09453452133486},
		} {
			e, ok := arcElem(c[0], c[1], c[2], 0, 2*math.Pi, true)
			require.True(t, ok)
			out = append(out, e)
		}
		return out
	}

	k, err := newWallKernelBudget(nil, elems(), nil, nil,
		15*math.Pi/180, exactScalar(0), false, math.Inf(1))
	require.NoError(t, err)
	out, err := k.runBudget(nil)
	require.NoError(t, err)

	require.True(t, out.ok, `a straddling leading coefficient must not silence the survey`)
	require.True(t, out.hasSpan)
	// Each circle's own concentric disk is the reading: the smallest circle's
	// diameter is the least spanning one, the largest circle's radius the
	// greatest empty one. Both are stated radii, so both readings are exact.
	require.Equal(t, 2*104.09453452133486, out.span)
	require.Equal(t, 0.0, out.spanBound)
	require.Equal(t, 149.98644153683318, out.inradius)
	require.Equal(t, 0.0, out.inradiusBound)

	// The discriminating half: the defect is an admitted candidate carrying no
	// finite bound, which is what the aggregate refuses. Every candidate this
	// boundary admits must now arrive with one.
	k2, err := newWallKernelBudget(nil, elems(), nil, nil,
		15*math.Pi/180, exactScalar(0), false, math.Inf(1))
	require.NoError(t, err)
	require.NoError(t, k2.generate(nil, func(c diskCand) error {
		_, empty, ok, err := k2.validate(c, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if !empty {
			return nil
		}
		require.False(t, math.IsInf(c.rBound, 1),
			`an admitted candidate with no finite bound leaves the whole survey undecided`)
		return nil
	}))
}

// TestSolveTripleStraddledDeterminantDropsOnlyItsOwnCandidate pins the second
// place a vanishing denominator can silence a survey: the Apollonius triple
// whose two linear equations the arithmetic cannot separate from parallel. The
// straddle runs BOTH generators, which is the sound posture — the pair may
// really be parallel, and it may really not be — but the independent branch
// divides the affine centre by that same determinant, so boundedQuotient hands
// it +Inf and the candidate it builds would refuse the whole survey through
// runBudget's aggregate. That one candidate is dropped, and only it.
//
// The fixture is two facing lines 10 mm apart, the second a hair off parallel
// under a bound its own arithmetic cannot resolve, plus a circular hole between
// them. solveParallelPair's reading of the same triple — the 5 mm disk — must
// still arrive, which is what makes this a local refusal rather than a missing
// wall.
func TestSolveTripleStraddledDeterminantDropsOnlyItsOwnCandidate(t *testing.T) {
	l1 := circEq{a: exactScalar(1), b: exactScalar(0), e: exactScalar(-1), f: exactScalar(0)}
	l2 := circEq{
		a: exactScalar(-1),
		b: measuredScalar(1e-13, 1e-9),
		e: exactScalar(-1),
		f: exactScalar(10),
	}
	// A hole of radius 2 centred at (5, 5): cx² + cy² − r² − 10cx − 10cy − 4r + 46 = 0.
	hole := circEq{
		quad: true,
		g:    exactScalar(-10),
		h:    exactScalar(-10),
		kk:   exactScalar(-4),
		m:    exactScalar(46),
	}

	require.Equal(t, survStraddle,
		admitMagnitudeAbove(boundedSub(boundedMul(l1.a, l2.b), boundedMul(l2.a, l1.b)), 1e-12),
		`the pair's determinant must straddle, or the fall-through is untested`)

	var radii []float64
	solveTriple([3]circEq{l1, l2, hole}, 1, func(_, _, r, rBound float64) {
		require.False(t, math.IsInf(rBound, 1),
			`a candidate divided out of a straddled determinant carries no bound and must not be emitted`)
		radii = append(radii, r)
	})
	require.NotEmpty(t, radii,
		`the parallel generator's own reading of this triple must survive the refusal`)
	for _, r := range radii {
		require.InDelta(t, 5.0, r, 1e-9, `the pair fixes the disk at half its own separation`)
	}
}

// TestArcWalkRadiusBoundStaysUnderTheKernelSlack pins the headroom the
// surveyElem.rrBound doc comment's derivation rests on, so that derivation
// cannot silently rot. An ArcSeg records Start and Center, never the radius, so
// the walk's radius is a math.Hypot of coordinate differences and
// arcWalkRadiusBound brackets its error; the kernel's positions, ray casts and
// tolerance comparisons read that HELD radius rather than the bracket, and what
// makes them sound is that the bracket sits decades below every slack they
// concede. The two claims below are that chain, over a range of arc geometries
// and scales: the bracket is at most 2^-50 of the radius — a few ulp of it —
// and, since the kernel's scale is never smaller than an element's radius, that
// leaves it over six decades under k.tol, the weakest slack any of those
// readings declares.
func TestArcWalkRadiusBoundStaysUnderTheKernelSlack(t *testing.T) {
	for _, scale := range []float64{1e-3, 1, 7.5, 1e3, 1e6} {
		for _, d := range [][2]float64{
			{1, 1}, {3, 4}, {0.5, 0.125}, {2, 9}, {1, 1e-3}, {6.7, 0.29}, {1e-2, 1},
		} {
			du, dv := d[0]*scale, d[1]*scale
			cu, cv := 0.375*scale, -1.25*scale
			t.Run(fmt.Sprintf("scale=%g/d=%g,%g", scale, d[0], d[1]), func(t *testing.T) {
				// The whole production chain, so the bound under test is the
				// one an element really carries: the held radius is the walk's
				// own math.Hypot of recorded differences, not an ideal radius.
				w, err := walkOf(ArcSeg{
					Center: Point2{U: cu, V: cv},
					Start:  Point2{U: cu + du, V: cv + dv},
					End:    Point2{U: cu - du, V: cv - dv},
					TStart: 0,
					TEnd:   1,
				}, newFreeformWork())
				require.NoError(t, err)
				e, ok := walkElem(w)
				require.True(t, ok)
				require.Equal(t, w.radiusBound, e.rrBound)

				require.LessOrEqual(t, e.rrBound, math.Ldexp(e.rr, -50),
					`the hypot bracket must stay within a few ulp of the radius it brackets`)

				k := newWallKernel([]surveyElem{e}, nil, math.Inf(1))
				require.GreaterOrEqual(t, k.scale, e.rr,
					`the kernel's scale reaches every element radius, which is what carries the bound over`)
				require.Less(t, e.rrBound*1e6, k.tol,
					`the bound must stay decades under the slack every predicate concedes`)
			})
		}
	}
}

// TestRevolveMinRadiusNumeratorIsIntervalMinimum pins the numerator the
// non-circular arm of revolveMinRadius hands its quotient: the enclosure of
// the NEARER wall end's radial coordinate, which is the interval minimum of
// the two ends and never the interval of whichever HELD value compared
// smaller. The two ends carry independent proven bounds (axisFrame.walk's
// startVBound/endVBound), so a comparison of the held floats decides nothing
// while those intervals overlap.
//
// The wall below is that overlap made concrete. Its two v coordinates are
// adjacent float64s, so about the axis v = −1000 both ends hold the same
// radial coordinate 1000.5 — the start exactly, the end only after its own
// subtraction rounded up to it. The end is therefore the truly nearer one, by
// less than the ulp that hides it, and the interval a held comparison
// publishes excludes it.
func TestRevolveMinRadiusNumeratorIsIntervalMinimum(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	line, err := axisInPlane(SketchLine{
		Start: Point2{U: 0, V: -1000},
		End:   Point2{U: 1, V: -1000},
	}, frame)
	require.NoError(t, err)
	ax := axisFrame{
		aU: line.aU, aV: line.aV,
		aUBound: line.aUBound, aVBound: line.aVBound,
		dU: line.dU, dV: line.dV,
		dUBound: line.dUBound, dVBound: line.dVBound,
		snapTol: 1e-9,
	}

	// 0.49999999999999994 is the float64 immediately below 0.5.
	const nearV = 0.49999999999999994
	require.Equal(t, nearV, math.Nextafter(0.5, 0))
	w, err := walkOf(LineSeg{
		Start:  Point2{U: 0, V: 0.5},
		End:    Point2{U: 10, V: nearV},
		TStart: 0,
		TEnd:   1,
	}, nil)
	require.NoError(t, err)
	aw := ax.walk(w)
	require.Equal(t, aw.startV, aw.endV, `the two ends must hold the SAME radial coordinate for the tie to bite`)
	require.Equal(t, 0.0, aw.startVBound)
	require.Greater(t, aw.endVBound, 0.0)

	// The truth is the exact radial coordinate of the nearer end, computed
	// over the rationals: rounding it to a float64 would round it straight
	// back onto the held value and no assertion here could fail.
	toRat := func(x float64) *big.Rat {
		r := new(big.Rat)
		require.NotNil(t, r.SetFloat64(x), `float64 %v must be finite to convert exactly`, x)
		return r
	}
	truth := new(big.Rat).Sub(toRat(nearV), toRat(ax.aV))
	require.Equal(t, -1, truth.Cmp(new(big.Rat).Sub(toRat(0.5), toRat(ax.aV))),
		`the end the held comparison discards must be the truly nearer one`)

	encloses := func(q boundedScalar) bool {
		lo := new(big.Rat).Sub(toRat(q.value), toRat(q.bound))
		hi := new(big.Rat).Add(toRat(q.value), toRat(q.bound))
		return lo.Cmp(truth) <= 0 && hi.Cmp(truth) >= 0
	}
	start := measuredScalar(aw.startV, aw.startVBound)
	end := measuredScalar(aw.endV, aw.endVBound)
	require.False(t, encloses(start),
		`the assertion below is vacuous unless the held-selected end's own interval misses the truth`)
	require.True(t, encloses(boundedMin(start, end)),
		`the interval minimum must contain the nearer end's true radial coordinate %s (got %v +/- %v)`,
		truth.FloatString(20), boundedMin(start, end).value, boundedMin(start, end).bound)
}

// freeformWallSection is a fit-spline arc closed by a chord — the same shape
// docs/spline-design.md §10 P4b's own fixture uses — built as a raw record
// rather than through sketch, mirroring TestPrismWallSubToleranceWebIsUndecided
// above. prismWall never validates profile closure itself (§8.1), so a raw
// record is enough to exercise the wall kernel's free-form arm.
func freeformWallSection() ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		FitSplineSeg{
			Fit:    []Point2{{U: 0, V: 0}, {U: 5, V: 4}, {U: 10, V: 0}},
			TStart: 0, TEnd: 1,
		},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
}

// TestPrismWallFreeformSectionReadsUndecided pins PR 1
// (docs/spline-design.md §8.1, Table R R9): a free-form boundary segment must
// leave the wall survey undecided — Suspect through Verify — never return an
// error out of prismWall. Reaching this through the public surface now runs
// through Extrude (§10 P4b); this test still calls prismWall directly, as the
// sub-tolerance-web test above does, to isolate the survey from the build.
func TestPrismWallFreeformSectionReadsUndecided(t *testing.T) {
	pp := prismPayload{profile: freeformWallSection(), z0: 0, z1: 10}
	out, err := prismWall(newWorkBudget(t.Context()), pp, 15*math.Pi/180)
	require.NoError(t, err, `a free-form section must not error out of Verify`)
	require.False(t, out.ok, `undecided, never a silent pass`)
	require.Equal(t, surveyUndecided, out.reason)
}

// TestPrismWallFreeformSectionPropagatesCancellation pins that swallowing the
// free-form refusal does not also swallow genuine cancellation: a context
// cancelled ahead of the call must still surface as the context's own error,
// not as an undecided reading.
func TestPrismWallFreeformSectionPropagatesCancellation(t *testing.T) {
	pp := prismPayload{profile: freeformWallSection(), z0: 0, z1: 10}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := prismWall(newWorkBudget(ctx), pp, 15*math.Pi/180)
	require.ErrorIs(t, err, context.Canceled)
}

// TestPrismWallPropagatesNonFreeformRefusals pins the other half of the same
// rule: the undecided reading is keyed to the free-form refusal ALONE, so every
// other reason a section fails to decompose still reaches the caller as the
// error it is. Each row names a section recordLoops rejects for a reason that
// is not the free-form staging limit — one of walkOf's ErrDegenerate arms, a
// radius whose unit is not a length, and a free-form span the §6.1 length
// bracket itself refuses as R15's ErrUnsupported. A reading of "undecided" on
// any of them would report a proof that did not close where the survey never
// looked at the section at all.
func TestPrismWallPropagatesNonFreeformRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		seg     CurveSegment
		is      error
		message string
	}{
		{
			name: "a circle whose CCW flag contradicts its range order",
			seg: CircleSeg{
				Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(10),
				CCW: true, TStart: 1, TEnd: 0,
			},
			is:      ErrDegenerate,
			message: "CCW flag contradicts its range order",
		},
		{
			name:    "a nil segment pointer",
			seg:     (*LineSeg)(nil),
			is:      ErrDegenerate,
			message: "nil curve segment",
		},
		{
			name: "a circle whose radius is an angle",
			seg: CircleSeg{
				Center: Point2{U: 0, V: 0}, Radius: units.Degrees(10),
				CCW: true, TStart: 0, TEnd: 1,
			},
			message: "radius is not a length",
		},
		{
			// R15: the curve exists and this evaluator cannot state its length,
			// so the refusal is ErrUnsupported — the SAME sentinel the free-form
			// staging limit wraps, on a free-form segment, and still not the
			// undecided reading.
			seg: SplineSeg{Control: []Point2{
				{U: -math.MaxFloat64, V: -math.MaxFloat64},
				{U: -math.MaxFloat64, V: math.MaxFloat64},
				{U: math.MaxFloat64, V: -math.MaxFloat64},
				{U: math.MaxFloat64, V: math.MaxFloat64},
			}, TStart: 0, TEnd: 1},
			name:    "a free-form span whose length runs past the float64 range",
			is:      ErrUnsupported,
			message: "representable float64 range",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pp := prismPayload{
				profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{tc.seg}}},
				z0:      0, z1: 10,
			}
			out, err := prismWall(newWorkBudget(t.Context()), pp, 15*math.Pi/180)
			require.Error(t, err, `a section the survey never read is a failure, never an undecided reading`)
			if tc.is != nil {
				require.ErrorIs(t, err, tc.is)
			}
			require.Contains(t, err.Error(), tc.message)
			require.NotErrorIs(t, err, errFreeformSection,
				`only the free-form staging limit carries the survey's own sentinel`)
			require.False(t, out.ok)
		})
	}
}

// TestPrismWallFreeformRefusalKeepsItsSentinels pins the identity the undecided
// reading is keyed to: recordLoops' free-form refusal is the survey's own
// sentinel AND still the ErrUnsupported staging limit every other consumer of
// the same decomposition branches on.
func TestPrismWallFreeformRefusalKeepsItsSentinels(t *testing.T) {
	_, err := recordLoops(newWorkBudget(t.Context()), freeformWallSection())
	require.ErrorIs(t, err, errFreeformSection)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "the wall survey does not support a free-form boundary segment")
}

// TestPrismWallAnalyticSectionRegression pins that an all-analytic section's
// wall reading is unchanged by PR 1's swallow: a 10x10x10 box still reports
// its exact spanning diameter.
func TestPrismWallAnalyticSectionRegression(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
	}
	pp := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			line(0, 0, 10, 0),
			line(10, 0, 10, 10),
			line(10, 10, 0, 10),
			line(0, 10, 0, 0),
		}}},
		z0: 0, z1: 10,
	}
	out, err := prismWall(newWorkBudget(t.Context()), pp, 15*math.Pi/180)
	require.NoError(t, err)
	require.True(t, out.ok)
	require.NotNil(t, out.reading)
	require.InDelta(t, 10.0, *out.reading, 1e-9, `a 10mm square's spanning diameter is 10mm`)
	require.Equal(t, 0.0, out.bound, `an all-analytic square's spanning diameter is Exact`)
}
