package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file pins the three load-bearing facts docs/spline-design.md §6.5 rests
// on. First: a control polygon whose turns all share one sign does NOT bound
// the curve's own curvature sign, which is why a wall edge's convexity is
// certified from the curvature numerator K = u'v" - v'u" in the Bernstein
// basis and never from the polygon's turns. Second: that coefficient test does
// not stand alone either — a span whose speed vanishes carries no signed
// curvature at all while K stays perfectly well behaved, so §6.5 proves the
// speed nonzero over the CLOSED span before reading a single coefficient, and
// the two cusp fixtures here are the records that would otherwise publish a
// convexity bool for an edge the curve doubles back on. Third: a FitSplineSeg
// records no control points at all, and the fit points it does record neither
// are the converted Bezier chain nor contain the curve — so every rule §6.5
// states must be phrased over §5.1's converted chain.
//
// The certificate itself is staged (§10 P4b); what is testable today is the
// exact-rational geometry it will read, which these tests compute through the
// shipped conversion and the shipped ratPoly engine.

// curvatureNumerator is §6.2's K = u'v" - v'u" for one polynomial span: the
// quantity whose sign IS the curve's signed-curvature sign. It reads the
// shipped exact Bernstein-to-monomial restatement spline_moments.go already
// integrates through, so nothing here rounds and nothing forks a second basis
// conversion.
func curvatureNumerator(span bezierSpan) ratPoly {
	u, v := spanCoordinatePolys(span)
	du, dv := rpDeriv(u), rpDeriv(v)
	ddu, ddv := rpDeriv(du), rpDeriv(dv)
	return rpSub(rpMul(du, ddv), rpMul(dv, ddu))
}

// polygonTurns is the retired rule's own quantity: the cross product at each
// interior vertex of the control polygon, exactly.
func polygonTurns(span bezierSpan) []*big.Rat {
	out := make([]*big.Rat, 0, len(span)-2)
	for i := 1; i+1 < len(span); i++ {
		ax := new(big.Rat).Sub(span[i].u, span[i-1].u)
		ay := new(big.Rat).Sub(span[i].v, span[i-1].v)
		bx := new(big.Rat).Sub(span[i+1].u, span[i].u)
		by := new(big.Rat).Sub(span[i+1].v, span[i].v)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(ay, bx))
		out = append(out, cross)
	}
	return out
}

// squaredSpeed is §6.3's S = u'^2 + v'^2 for one polynomial span: the exact
// rational polynomial §6.5's regularity precondition proves has no root on the
// closed span before any curvature coefficient is read.
func squaredSpeed(span bezierSpan) ratPoly {
	u, v := spanCoordinatePolys(span)
	du, dv := rpDeriv(u), rpDeriv(v)
	return rpAdd(rpMul(du, du), rpMul(dv, dv))
}

// closedSpanRootCount is the precondition's own mechanical test, exactly as
// §6.5 states it: ratPoly's Sturm chain counts roots on the HALF-OPEN (0, 1],
// so the value at 0 is reported beside it and the closed span is covered only
// by reading both.
func closedSpanRootCount(s ratPoly) (halfOpen int, atZero *big.Rat) {
	chain := sturmChain(rpSquareFree(rpTrim(s)))
	return sturmCount(chain, big.NewRat(0, 1), big.NewRat(1, 1)), rpEval(s, big.NewRat(0, 1))
}

// bernsteinCoefficients restates a monomial ratPoly in the Bernstein basis of
// the given degree, exactly: b_i = sum_k C(i,k)/C(n,k) * a_k. §6.5 reads these
// coefficients' signs, and the sign of a Bernstein coefficient is basis
// business, never a rounding one.
func bernsteinCoefficients(p ratPoly, degree int) []*big.Rat {
	binom := func(n, k int) *big.Rat {
		return new(big.Rat).SetInt(new(big.Int).Binomial(int64(n), int64(k)))
	}
	out := make([]*big.Rat, degree+1)
	for i := range out {
		sum := new(big.Rat)
		for k := 0; k <= i && k < len(p); k++ {
			term := new(big.Rat).Quo(binom(i, k), binom(degree, k))
			sum.Add(sum, term.Mul(term, p[k]))
		}
		out[i] = sum
	}
	return out
}

// splitBernsteinAtMidpoint is §6.5's own subdivision: exact dyadic de
// Casteljau on the curvature numerator's Bernstein coefficients, returning the
// two children's coefficient sets.
func splitBernsteinAtMidpoint(b []*big.Rat) (left, right []*big.Rat) {
	half := big.NewRat(1, 2)
	work := make([]*big.Rat, len(b))
	for i, c := range b {
		work[i] = new(big.Rat).Set(c)
	}
	left = make([]*big.Rat, 0, len(b))
	right = make([]*big.Rat, len(b))
	for level := range b {
		left = append(left, new(big.Rat).Set(work[0]))
		right[len(b)-1-level] = new(big.Rat).Set(work[len(b)-1-level])
		for i := 0; i+1 < len(b)-level; i++ {
			work[i] = new(big.Rat).Mul(half, new(big.Rat).Add(work[i], work[i+1]))
		}
	}
	return left, right
}

// signsOf reads a coefficient set's signs, which is the whole of what §6.5's
// verdict rule consumes.
func signsOf(coeffs []*big.Rat) []int {
	out := make([]int, len(coeffs))
	for i, c := range coeffs {
		out[i] = c.Sign()
	}
	return out
}

func unitWeightCubic(control []Point2) NURBSSeg {
	return NURBSSeg{
		Degree:  3,
		Control: control,
		Knots:   []float64{0, 0, 0, 0, 1, 1, 1, 1},
		Weights: []float64{1, 1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
}

// TestSingleSignPolygonTurnsProveNoCurvatureSign is §6.5's falsifier for the
// polygon-turn rule: an admissible record whose every control-polygon turn is
// strictly positive while its own curvature takes BOTH signs. A rule reading
// the turns would publish convex for this wall.
func TestSingleSignPolygonTurnsProveNoCurvatureSign(t *testing.T) {
	seg := unitWeightCubic([]Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: -4, V: 1}, {U: 0.9, V: 0}})
	require.NoError(t, validateSegment(seg), "record.go admits the net: no distinctness or convexity gate exists")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err, "the record must convert: no gate rejects a net for its shape")
	require.False(t, reversed, "the recorded walk runs with the curve's natural sense")
	require.Len(t, spans, 1, "§5.1 converts this record to exactly one Bezier span")
	require.Len(t, spans[0], 4)

	turns := polygonTurns(spans[0])
	require.Len(t, turns, 2)
	for i, turn := range turns {
		require.Equal(t, 1, turn.Sign(), "polygon turn %d must be strictly positive", i)
	}
	// Both magnitudes are pinned, not just the signs, because §6.5 quotes them
	// ("1, and ~1/10"). The first is fixed a second time by the K(0) assertion
	// below — K(0) = 18*turn0 identically for a degree-3 Bezier over [0, 1] —
	// and the second is free otherwise. The doc marks only the second one
	// approximate, and each is asserted the way it is quoted: exactly, and at
	// the precision of the quoted figure with the exact lifted rational beside
	// it. NEVER assert an exact 1/10 here — the recorded U = 0.9 lifts to its
	// own binary float64, which is why the document writes that figure as
	// approximate.
	require.Equal(t, "1", turns[0].RatString(), "§6.5 quotes the first turn unmarked, so it must be exactly 1")
	require.Equal(t, "900719925474099/9007199254740992", turns[1].RatString())
	second, _ := turns[1].Float64()
	require.InDelta(t, 0.1, second, 1e-12, "the second turn is the doc's approximate 1/10")

	k := curvatureNumerator(spans[0])
	at0 := rpEval(k, big.NewRat(0, 1))
	require.Equal(t, 1, at0.Sign(), "K(0) must be positive")
	require.Equal(t, "18", at0.RatString())

	at57 := rpEval(k, big.NewRat(5, 7))
	require.Equal(t, -1, at57.Sign(), "K(5/7) must be negative — the curvature sign the polygon turns did not bound")

	at1 := rpEval(k, big.NewRat(1, 1))
	require.Equal(t, 1, at1.Sign(), "K(1) must be positive: two curvature sign changes, zero polygon-turn sign changes")
}

// TestInteriorCuspFoldsToAStrictSignWithoutRegularity is §6.5's sharpest
// falsifier for a coefficient test that skips the regularity precondition:
// §6.3's own interior-cusp net, whose curve has NO direction at t = 1/2, is
// mixed at the top level and one-signed with strict entries on both dyadic
// children, so the fold alone reaches a strict '-' and publishes convex for an
// edge that doubles back on itself. The speed is what refuses it.
func TestInteriorCuspFoldsToAStrictSignWithoutRegularity(t *testing.T) {
	seg := SplineSeg{
		Control: []Point2{
			{U: -1.0 / 8, V: 1.0 / 4}, {U: 1.0 / 8, V: -1.0 / 12},
			{U: -1.0 / 8, V: -1.0 / 12}, {U: 1.0 / 8, V: 1.0 / 4},
		},
		TStart: 0,
		TEnd:   1,
	}
	require.NoError(t, validateSegment(seg), "record.go admits the cusp net: no regularity gate exists at recording")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.False(t, reversed)
	require.Len(t, spans, 1, "a clamped 4-control SplineSeg converts to exactly one span")

	// §6.5 quotes K as approximately -3/2*(2t - 1)^2, and it is approximate for
	// the reason that section states once: the recorded -1/12 enters as its own
	// binary float64 a, so the true K is 18*(a - 1/4)*(t - 1/2)^2 and its
	// leading factor misses -6 by about 8.3e-17. The quoted figure is asserted
	// at the precision it states; the exact lifted rationals are pinned beside
	// it, so a conversion that rounded anywhere would fail here.
	k := rpTrim(curvatureNumerator(spans[0]))
	require.Len(t, k, 3)
	for i, want := range []float64{-1.5, 6, -6} {
		got, _ := k[i].Float64()
		require.InDelta(t, want, got, 1e-12, "K's monomial coefficient %d must match the doc's quoted figure", i)
	}
	require.Equal(t, []string{
		"-216172782113783805/144115188075855872",
		"216172782113783805/36028797018963968",
		"-216172782113783805/36028797018963968",
	}, []string{k[0].RatString(), k[1].RatString(), k[2].RatString()},
		"the exact lifted coefficients: 18*(a - 1/4) with a the float64 lift of -1/12")

	// Top level: MIXED, so §6.5 subdivides rather than publishing.
	top := bernsteinCoefficients(k, 3)
	require.Equal(t, []int{-1, 1, 1, -1}, signsOf(top), "the top-level coefficients must be mixed")

	// Both children: one-signed with a strict entry, which folds to a strict
	// '-' and would set the bool.
	left, right := splitBernsteinAtMidpoint(top)
	require.Equal(t, []int{-1, -1, 0, 0}, signsOf(left), "the left child folds to a strict '-'")
	require.Equal(t, []int{0, 0, -1, -1}, signsOf(right), "the right child folds to a strict '-'")

	// The precondition is what stops it: S vanishes at the cusp, interior to
	// the span, so the root count alone already refuses R19.
	s := squaredSpeed(spans[0])
	require.Equal(t, 0, rpEval(s, big.NewRat(1, 2)).Sign(), "the speed vanishes at t = 1/2 — an ordinary cusp")
	halfOpen, atZero := closedSpanRootCount(s)
	require.Equal(t, 1, halfOpen, "the Sturm chain must see that root on (0, 1]")
	require.Equal(t, 1, atZero.Sign(), "and the span's own start is regular, so only the count refuses it")
}

// TestEndpointCuspEscapesAHalfOpenRootCount is the precondition's other cusp
// shape and the reason §6.5 states the CLOSED span: two coincident leading
// controls put the only speed root exactly at t = 0, where a half-open Sturm
// count reports nothing, while the curvature coefficients are one-signed with
// strict entries and would publish convex outright.
func TestEndpointCuspEscapesAHalfOpenRootCount(t *testing.T) {
	seg := unitWeightCubic([]Point2{{U: 0, V: 0}, {U: 0, V: 0}, {U: 1.0 / 3, V: 0}, {U: 1, V: 1}})
	require.NoError(t, validateSegment(seg), "record.go admits coincident adjacent controls")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.False(t, reversed)
	require.Len(t, spans, 1)

	k := rpTrim(curvatureNumerator(spans[0]))
	top := bernsteinCoefficients(k, 3)
	require.Equal(t, []int{0, 0, 1, 1}, signsOf(top),
		"every coefficient >= 0 with strict entries: the coefficient test alone publishes a strict '+'")

	s := squaredSpeed(spans[0])
	halfOpen, atZero := closedSpanRootCount(s)
	require.Equal(t, 0, halfOpen, "the half-open count finds NO root — on its own it admits this span")
	require.Equal(t, 0, atZero.Sign(), "yet S(0) is zero: C'(0) = 3(P1 - P0) = (0, 0), so the span has no direction there")
}

// TestCollinearNetProvesTheZeroCurvatureNumerator is §6.5's straight-walk
// outcome: a collinear net — three DISTINCT control positions, so no
// two-position special case reaches it — has K identically zero, which is the
// verdict that routes the wall edge to evaluator §3's loop-role rule instead
// of to a sign or a refusal.
func TestCollinearNetProvesTheZeroCurvatureNumerator(t *testing.T) {
	seg := NURBSSeg{
		Degree:  2,
		Control: []Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: 2, V: 0}},
		Knots:   []float64{0, 0, 0, 1, 1, 1},
		Weights: []float64{1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}

	require.NoError(t, validateSegment(seg), "record.go admits a collinear net too")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.False(t, reversed)
	require.Len(t, spans, 1)
	require.Len(t, spans[0], 3, "three DISTINCT recorded positions, not a two-position net")

	turns := polygonTurns(spans[0])
	require.Len(t, turns, 1)
	require.Equal(t, 0, turns[0].Sign(), "the lone polygon turn is zero — neither a sign nor a disagreement")

	k := curvatureNumerator(spans[0])
	require.Empty(t, rpTrim(k), "K must be the zero polynomial: the span lies on one straight line")
}

// TestFitPointsAreNeitherTheChainNorItsHull is §6.5's FitSplineSeg clause: the
// converted chain's own controls leave the recorded fit points' hull, so a
// containment or convexity rule phrased over recorded points misses the curve.
func TestFitPointsAreNeitherTheChainNorItsHull(t *testing.T) {
	fit := []Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: 2, V: 1}, {U: 3, V: 0}}
	seg := FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}
	require.NoError(t, validateSegment(seg))

	spans, err := fitSplineBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 3)
	require.Len(t, spans[0], 4, "a FitSplineSeg records NO control points; these are §5.1.2's converted ones")

	floor := math.Inf(1)
	for _, p := range fit {
		floor = math.Min(floor, p.V)
	}
	require.Equal(t, 0.0, floor, "every recorded fit point sits at or above v = 0")

	// The two interior controls §5.1.2's closed form produces for the first
	// span sit BELOW that floor: the -h^2*m/18 terms are what push them out.
	for i := 1; i <= 2; i++ {
		control, _ := spans[0][i].v.Float64()
		require.Less(t, control, floor, "converted control %d must leave the recorded hull", i)
	}
	// §6.5 marks these two interior controls approximate and quotes them to four
	// decimals, so they are asserted at exactly that precision.
	b1, _ := spans[0][1].v.Float64()
	b2, _ := spans[0][2].v.Float64()
	require.InDelta(t, -0.0790, b1, 1e-4, "the doc's approximate -0.0790, at the precision it states")
	require.InDelta(t, -0.1580, b2, 1e-4, "the doc's approximate -0.1580, at the precision it states")

	// The two ends of that same quoted four-control figure carry no such mark,
	// so they are exact zeros, and they are pinned exactly rather than through
	// the sampled minimum: the minimum's own delta has enough slack that an
	// endpoint drift of order 1e-4 would leave every other assertion here true.
	require.Equal(t, "0", spans[0][0].v.RatString(), "the chain starts at the first fit point's own v")
	require.Equal(t, "0", spans[0][3].v.RatString(), "the first span's joint control sits at v = 0 too")

	// And so does the curve itself, sampled through the shipped evaluator over
	// the first span's own third of the converted parameter.
	minV := math.Inf(1)
	const samples = 2000
	for i := 0; i <= samples; i++ {
		_, v := evalSpans(t, spans, float64(i)/float64(samples)/3)
		minV = math.Min(minV, v)
	}
	require.Less(t, minV, floor, "the curve leaves the recorded fit points' hull")
	require.InDelta(t, -0.0912, minV, 1e-4, "the doc's approximate dip, at the precision it states")
}
