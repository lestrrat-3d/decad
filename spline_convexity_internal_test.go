package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file pins the load-bearing facts docs/spline-design.md §6.5 rests on.
// First: a control polygon whose turns all share one sign does NOT bound
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
// Beyond those three, the tests at the end of this file pin §6.5's Table K —
// the enumeration that makes the section TOTAL over what §5.1's conversion can
// produce. Each of them converts a record record.go admits and shows the shape
// the table names arriving: a degree-1 span, a degree-2 span whose true K sits
// below its stated degree, a RUN of consecutive collapsed spans, the joint a
// subdivision creates, and a reversed recorded range over an unreversed chain.
//
// The certificate itself is staged (§10 P4b); what is testable today is the
// exact-rational geometry it will read, which these tests compute through the
// shipped conversion and the shipped ratPoly engine.

// curvatureNumerator is now spline_convexity.go's own production function;
// this file no longer defines a duplicate. Every test below calls it exactly
// as before — the production version reproduces it verbatim (§6.2's
// K = u'v" - v'u").

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
func closedSpanRootCount(t *testing.T, s ratPoly) (halfOpen int, atZero *big.Rat) {
	t.Helper()
	chain := mustSturmChainInt(t, rpSquareFree(rpTrim(s)))
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

// splitSpanAtMidpoint is the same exact dyadic de Casteljau split applied to
// the SPAN itself rather than to K's coefficients. §6.5 states the subdivision
// over the span, so the two routes to a child's coefficients are pinned against
// each other below.
func splitSpanAtMidpoint(span bezierSpan) (left, right bezierSpan) {
	n := len(span)
	half := big.NewRat(1, 2)
	work := make([]ratPoint, n)
	for i, p := range span {
		work[i] = ratPoint{u: new(big.Rat).Set(p.u), v: new(big.Rat).Set(p.v)}
	}
	left = make(bezierSpan, 0, n)
	right = make(bezierSpan, n)
	for level := range n {
		left = append(left, ratPoint{u: new(big.Rat).Set(work[0].u), v: new(big.Rat).Set(work[0].v)})
		right[n-1-level] = ratPoint{u: new(big.Rat).Set(work[n-1-level].u), v: new(big.Rat).Set(work[n-1-level].v)}
		for i := 0; i+1 < n-level; i++ {
			work[i] = ratPoint{
				u: new(big.Rat).Mul(half, new(big.Rat).Add(work[i].u, work[i+1].u)),
				v: new(big.Rat).Mul(half, new(big.Rat).Add(work[i].v, work[i+1].v)),
			}
		}
	}
	return left, right
}

// controlEdge is one control edge of a span as an exact rational vector: the
// quantity §6.5's joint rule crosses, and the quantity a collapsed span has
// none of.
func controlEdge(from, to ratPoint) (u, v *big.Rat) {
	return new(big.Rat).Sub(to.u, from.u), new(big.Rat).Sub(to.v, from.v)
}

// crossOf is the exact cross product §6.5's joint verdict reads.
func crossOf(au, av, bu, bv *big.Rat) *big.Rat {
	return new(big.Rat).Sub(new(big.Rat).Mul(au, bv), new(big.Rat).Mul(av, bu))
}

// dotOf is the exact dot product that tells §6.5's "same way" from its
// "OPPOSITE ways" once a cross has come back zero.
func dotOf(au, av, bu, bv *big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(au, bu), new(big.Rat).Mul(av, bv))
}

// jointCross is §6.5's joint verdict between two spans of a chain: the incoming
// span's LAST control edge crossed with the outgoing span's FIRST.
func jointCross(incoming, outgoing bezierSpan) *big.Rat {
	inU, inV := controlEdge(incoming[len(incoming)-2], incoming[len(incoming)-1])
	outU, outV := controlEdge(outgoing[0], outgoing[1])
	return crossOf(inU, inV, outU, outV)
}

// spanIsCollapsed is §5.1's collapsed span: every control point of the span the
// same point, so the span has no nonzero control edge and no direction.
func spanIsCollapsed(span bezierSpan) bool {
	for i := 1; i < len(span); i++ {
		if span[i].u.Cmp(span[0].u) != 0 || span[i].v.Cmp(span[0].v) != 0 {
			return false
		}
	}
	return true
}

// ratStrings renders a coefficient set exactly, for pinning without a delta.
func ratStrings(coeffs []*big.Rat) []string {
	out := make([]string, len(coeffs))
	for i, c := range coeffs {
		out[i] = c.RatString()
	}
	return out
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

	// The production certificate must refuse this net rather than publish the
	// polygon rule's wrong "convex": K genuinely changes sign twice inside the
	// span, so no depth of subdivision resolves it to one strict sign.
	_, err = freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.ErrorIs(t, err, ErrUnsupported,
		"the certificate must refuse R19 rather than read the single-signed polygon turns")
}

// TestMixedCurvatureAtTheSubdivisionDepthCapRefusesR19 is the fixture bank's
// missing case: a span whose curvature numerator genuinely changes sign
// inside the span — not merely a hull over-estimate a deeper split would
// clear — never resolves its Bernstein form to one strict sign at any depth,
// so it must refuse Table R row R19 once the fixed depth cap
// (freeformLengthDepth, spline_length.go) is reached, rather than loop
// forever or publish a bool nothing proved.
//
// It reuses TestSingleSignPolygonTurnsProveNoCurvatureSign's own net, whose
// K is positive at t=0, negative at t=5/7 and positive again at t=1 — two
// genuine sign changes — and separately pins that this span's SPEED is
// regular, so the refusal below is provably the depth cap and not the
// regularity precondition.
func TestMixedCurvatureAtTheSubdivisionDepthCapRefusesR19(t *testing.T) {
	seg := unitWeightCubic([]Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: -4, V: 1}, {U: 0.9, V: 0}})
	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.False(t, reversed)
	require.Len(t, spans, 1)

	s := squaredSpeed(spans[0])
	halfOpen, atZero := closedSpanRootCount(t, s)
	require.Equal(t, 0, halfOpen, "this span's speed has no interior or end root")
	require.Equal(t, 1, atZero.Sign(), "and its speed is nonzero at its own start too — regularity holds")

	_, err = freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.ErrorIs(t, err, ErrUnsupported,
		"a genuine curvature sign change never resolves to one strict sign, so the depth cap must refuse R19")
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
	halfOpen, atZero := closedSpanRootCount(t, s)
	require.Equal(t, 1, halfOpen, "the Sturm chain must see that root on (0, 1]")
	require.Equal(t, 1, atZero.Sign(), "and the span's own start is regular, so only the count refuses it")

	// The production certificate must refuse this net too: the regularity
	// precondition, not the coefficient fold, is what stops it.
	_, err = freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.ErrorIs(t, err, ErrUnsupported,
		"the speed precondition must refuse R19 before the mixed-then-strict coefficient fold ever runs")
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
	halfOpen, atZero := closedSpanRootCount(t, s)
	require.Equal(t, 0, halfOpen, "the half-open count finds NO root — on its own it admits this span")
	require.Equal(t, 0, atZero.Sign(), "yet S(0) is zero: C'(0) = 3(P1 - P0) = (0, 0), so the span has no direction there")

	// The production certificate must still refuse: the CLOSED-span endpoint
	// check is what catches what the half-open root count alone would admit.
	_, err = freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.ErrorIs(t, err, ErrUnsupported,
		"the endpoint value must refuse R19 even though the half-open root count alone would admit this span")
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

	verdict, err := freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityStraight, verdict,
		"K is identically zero and the chain is a single span, so the chain's verdict is the straight-walk one")
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

	// §6.5's own clause: every rule it states, including the certificate
	// itself, reads §5.1.2's converted chain and never the recorded fit
	// points. This hump-then-dip curve's curvature genuinely changes sign
	// more than once, so the certificate refuses R19 rather than publish a
	// bool for it — the same outcome a hand count of the curve's inflections
	// would predict, not a certificate defect.
	_, err = freeformWallConvexityContext(t.Context(), spans, false, false)
	require.ErrorIs(t, err, ErrUnsupported,
		"a hump-then-dip fit curve's curvature changes sign more than once, so the certificate must refuse R19")
}

// degreeOneNURBS is §6.5's Table K degree-1 row as a record: three control
// points at degree 1 over a clamped knot vector with one interior knot, which
// record.go admits (validateNURBSSegmentSizes refuses only Degree < 1) and
// which §5.1 converts to TWO degree-1 spans.
func degreeOneNURBS(tStart, tEnd float64) NURBSSeg {
	return NURBSSeg{
		Degree:  1,
		Control: []Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}},
		Knots:   []float64{0, 0, 1, 2, 2},
		Weights: []float64{1, 1, 1},
		TStart:  tStart,
		TEnd:    tEnd,
	}
}

// TestDegreeOneSpansCarryAZeroCurvatureNumerator is §6.5's degree-1 row: the
// stated coefficient degree 2p-3 is -1 at p = 1 and names no Bernstein form at
// all, so the section states the degree-0 all-zero one instead. This pins that
// the shipped conversion really does hand §6.5 such spans, that their K is the
// zero polynomial, that the regularity precondition closes on them, and that
// the chain's whole verdict therefore comes from the JOINT between them.
func TestDegreeOneSpansCarryAZeroCurvatureNumerator(t *testing.T) {
	seg := degreeOneNURBS(0, 1)
	require.NoError(t, validateSegment(seg), "record.go admits a degree-1 NURBS segment")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.False(t, reversed)
	require.Len(t, spans, 2, "a 3-control degree-1 record converts to two spans")

	// Both spans are degree 1 — two control points — and their coordinates are
	// exact: a degree-1 conversion inserts no knot and divides by nothing.
	require.Equal(t, [][]string{{"0", "0"}, {"1", "0"}}, spanStrings(spans[0]))
	require.Equal(t, [][]string{{"1", "0"}, {"1", "1"}}, spanStrings(spans[1]))

	for i, span := range spans {
		require.Len(t, span, 2, "span %d must be degree 1", i)
		require.False(t, spanIsCollapsed(span), "span %d is a real segment, not a collapsed one", i)

		// K is the ZERO polynomial: C" is identically zero on a degree-1 span.
		k := rpTrim(curvatureNumerator(span))
		require.Empty(t, k, "span %d must have K identically zero", i)

		// §6.5 carries it as a degree-0 Bernstein form holding one zero
		// coefficient, which the all-zero rule reads as verdict 0. Asserting the
		// LENGTH is the point: a degree read off 2p-3 = -1 names no array.
		bern := bernsteinCoefficients(k, 0)
		require.Equal(t, []string{"0"}, ratStrings(bern),
			"span %d must carry exactly one Bernstein coefficient, and it must be zero", i)
		require.Equal(t, []int{0}, signsOf(bern))

		// The regularity precondition closes: S is the nonzero constant 1 here,
		// so the half-open count sees no root and the endpoint value is nonzero.
		s := squaredSpeed(span)
		require.Equal(t, []string{"1"}, ratStrings(rpTrim(s)), "span %d's S is the constant 1", i)
		halfOpen, atZero := closedSpanRootCount(t, s)
		require.Equal(t, 0, halfOpen, "span %d's speed has no root on (0, 1]", i)
		require.Equal(t, 1, atZero.Sign(), "span %d's speed is nonzero at its start too", i)
	}

	// Every span verdict is 0, so the chain's verdict is the joint's own: a
	// strictly positive turn where the walk leaves the u axis for the v one.
	cross := jointCross(spans[0], spans[1])
	require.Equal(t, "1", cross.RatString(), "the joint between the two degree-1 spans turns by exactly +1")
	require.Equal(t, 1, cross.Sign())

	verdict, err := freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityPositive, verdict,
		"both span verdicts are 0, so the chain's verdict is the joint's own strictly positive turn")
}

// TestDegreeTwoCurvatureNumeratorIsAConstantAtTheStatedDegree is §6.5's
// degree-2 row: 2p-3 = 1 is a LOOSE bound, the true K being a constant, and the
// coefficient array's length is fixed by the stated degree rather than by that
// true one. The constant is 4*cross(dP0, dP1), which is 4 times the span's lone
// control-polygon turn.
func TestDegreeTwoCurvatureNumeratorIsAConstantAtTheStatedDegree(t *testing.T) {
	seg := NURBSSeg{
		Degree:  2,
		Control: []Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}},
		Knots:   []float64{0, 0, 0, 1, 1, 1},
		Weights: []float64{1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
	require.NoError(t, validateSegment(seg))

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 1)

	k := rpTrim(curvatureNumerator(spans[0]))
	require.Equal(t, []string{"4"}, ratStrings(k), "K is the constant 4 — one degree BELOW the stated 2p-3 = 1")

	turns := polygonTurns(spans[0])
	require.Len(t, turns, 1)
	require.Equal(t, "1", turns[0].RatString())
	require.Equal(t, "4", new(big.Rat).Mul(big.NewRat(4, 1), turns[0]).RatString(),
		"K is exactly 4*cross(dP0, dP1) on a degree-2 span")

	// At the STATED degree the array holds two entries, both that constant, and
	// its signs are the constant's own. A rule sizing the array from K's true
	// degree would carry one entry instead — the verdict is the same, which is
	// exactly why no verdict may depend on the true degree.
	stated := bernsteinCoefficients(k, 1)
	require.Equal(t, []string{"4", "4"}, ratStrings(stated), "the stated degree 2p-3 = 1 fixes the array's length")
	require.Equal(t, []int{1, 1}, signsOf(stated))
	require.Equal(t, []int{1}, signsOf(bernsteinCoefficients(k, 0)), "the true degree reads the same verdict")

	verdict, err := freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityPositive, verdict,
		"K is the positive constant 4 across the whole span, and the chain is one span with an empty joint set")
}

// TestConsecutiveCollapsedSpansPairAcrossTheWholeRun is §6.5's collapsed-RUN
// row. Three coincident controls in a degree-1 net produce two ADJACENT
// collapsed spans, so a joint rule that skips one span at a time pairs a
// neighbour with a span that has no direction at all; pairing across the whole
// run is what leaves a turn to read.
func TestConsecutiveCollapsedSpansPairAcrossTheWholeRun(t *testing.T) {
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
	require.NoError(t, validateSegment(seg), "record.go gates a net's shape nowhere")

	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 4)

	require.False(t, spanIsCollapsed(spans[0]))
	require.True(t, spanIsCollapsed(spans[1]), "the run's first span is collapsed")
	require.True(t, spanIsCollapsed(spans[2]), "and so is the one immediately after it")
	require.False(t, spanIsCollapsed(spans[3]))

	// Skipping ONE span pairs span 0 with span 2, whose only control edge is the
	// zero vector: there is no direction to cross with, and the cross a rule
	// would read comes back zero for a walk that plainly turns.
	oneAtATimeU, oneAtATimeV := controlEdge(spans[2][0], spans[2][1])
	require.Equal(t, 0, oneAtATimeU.Sign(), "the next collapsed span's control edge is the zero vector")
	require.Equal(t, 0, oneAtATimeV.Sign())
	require.Equal(t, 0, jointCross(spans[0], spans[2]).Sign(),
		"so a one-span skip reads a zero cross where the walk turns")

	// Pairing across the WHOLE run reads the turn the walk actually has.
	cross := jointCross(spans[0], spans[3])
	require.Equal(t, "1", cross.RatString(), "the neighbours across the run turn by exactly +1")
	require.Equal(t, 1, cross.Sign())

	verdict, err := freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityPositive, verdict,
		"both live spans are degree-1 (verdict 0), so the chain's verdict is the joint that pairs across the whole run")
}

// TestMidpointSplitCreatesAKnownZeroJoint is §6.5's subdivision row. The joint
// a split CREATES is not folded: a midpoint de Casteljau leaves the left
// child's last control edge and the right child's first the IDENTICAL vector,
// so its cross is exactly zero with the tangents pointing the same way. The
// same test pins the other half of that row — splitting the SPAN and splitting
// K's own Bernstein coefficients differ by the positive factor 1/8 per level,
// so both routes read the same signs.
func TestMidpointSplitCreatesAKnownZeroJoint(t *testing.T) {
	seg := unitWeightCubic([]Point2{{U: 0, V: 0}, {U: 1, V: 0}, {U: -4, V: 1}, {U: 0.9, V: 0}})
	spans, reversed, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 1)

	left, right := splitSpanAtMidpoint(spans[0])
	require.Len(t, left, 4)
	require.Len(t, right, 4)

	inU, inV := controlEdge(left[len(left)-2], left[len(left)-1])
	outU, outV := controlEdge(right[0], right[1])
	require.Equal(t, 0, inU.Cmp(outU), "the two meeting control edges are the identical vector")
	require.Equal(t, 0, inV.Cmp(outV))
	require.Equal(t, 0, crossOf(inU, inV, outU, outV).Sign(), "so the created joint's cross is exactly zero")
	require.Equal(t, 1, dotOf(inU, inV, outU, outV).Sign(), "and the two tangents point the same way: verdict 0")

	// Route A: split the parent's Bernstein coefficients. Route B: split the
	// span and recompute K on each child. They differ by 1/8 per level exactly.
	parent := bernsteinCoefficients(rpTrim(curvatureNumerator(spans[0])), 3)
	splitLeft, splitRight := splitBernsteinAtMidpoint(parent)
	childLeft := bernsteinCoefficients(rpTrim(curvatureNumerator(left)), 3)
	childRight := bernsteinCoefficients(rpTrim(curvatureNumerator(right)), 3)

	eighth := big.NewRat(1, 8)
	for i := range parent {
		require.Equal(t, 0, childLeft[i].Cmp(new(big.Rat).Mul(eighth, splitLeft[i])),
			"left child coefficient %d must be the split one scaled by 1/8", i)
		require.Equal(t, 0, childRight[i].Cmp(new(big.Rat).Mul(eighth, splitRight[i])),
			"right child coefficient %d must be the split one scaled by 1/8", i)
	}
	require.Equal(t, signsOf(splitLeft), signsOf(childLeft), "1/8 is positive, so both routes read the same signs")
	require.Equal(t, signsOf(splitRight), signsOf(childRight))

	// The production certificate subdivides at this same Bernstein level
	// (bernsteinCurvatureSignContext), so it must reach the identical
	// refusal this net's genuine sign change forces on both routes above.
	_, err = freeformWallConvexityContext(t.Context(), spans, false, reversed)
	require.ErrorIs(t, err, ErrUnsupported,
		"the same genuinely mixed curvature must refuse R19 through the production entry point too")
}

// TestReversedRangeConvertsToTheIdenticalUnreversedChain is §6.5's reversal
// row: the negation is ONE operation at the end. The conversion returns the
// same spans in the same order whatever the recorded range order is, and
// reports the reversal beside them, so every span verdict and every joint cross
// is computed on the unreversed chain before the sign is flipped.
func TestReversedRangeConvertsToTheIdenticalUnreversedChain(t *testing.T) {
	forward := degreeOneNURBS(0, 1)
	backward := degreeOneNURBS(1, 0)
	require.NoError(t, validateSegment(backward), "record.go admits a reversed recorded range")

	forwardSpans, forwardReversed, err := freeformBezierSpans(forward, newFreeformWork())
	require.NoError(t, err)
	require.False(t, forwardReversed)

	backwardSpans, backwardReversed, err := freeformBezierSpans(backward, newFreeformWork())
	require.NoError(t, err)
	require.True(t, backwardReversed, "the reversal is REPORTED, not applied to the spans")

	require.Len(t, backwardSpans, len(forwardSpans))
	for i := range forwardSpans {
		require.Equal(t, spanStrings(forwardSpans[i]), spanStrings(backwardSpans[i]),
			"span %d must be identical under both range orders", i)
	}

	// So the joint cross is the same quantity either way, and only the final
	// sign differs. Negating the straight walk's own 0 leaves 0.
	require.Equal(t, jointCross(forwardSpans[0], forwardSpans[1]).RatString(),
		jointCross(backwardSpans[0], backwardSpans[1]).RatString())
	require.Equal(t, 0, new(big.Rat).Neg(rpEval(rpTrim(curvatureNumerator(backwardSpans[0])), big.NewRat(1, 2))).Sign(),
		"a degree-1 span's K is zero, and negating zero is zero")

	// The production certificate's own reversal negation: the forward chain's
	// positive joint (TestDegreeOneSpansCarryAZeroCurvatureNumerator) must
	// negate to negative under the identical unreversed spans, reported
	// reversed.
	forwardVerdict, err := freeformWallConvexityContext(t.Context(), forwardSpans, false, forwardReversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityPositive, forwardVerdict)

	backwardVerdict, err := freeformWallConvexityContext(t.Context(), backwardSpans, false, backwardReversed)
	require.NoError(t, err)
	require.Equal(t, freeformConvexityNegative, backwardVerdict,
		"the identical unreversed chain's positive verdict negates once, at the end, under the reported reversal")
}

// TestClosedChainAddsTheWrapJointAnOpenChainNeverReads is §6.5's closed-chain
// row: on a chain that closes on itself, the joint between the last live span
// and the first is INTERIOR to that one wall edge and folds like every
// other, so a closed chain can refuse R19 on a joint an OPEN chain over the
// identical spans never even reads.
//
// Two synthetic degree-1 spans — degreeOneNURBS's own converted shape, built
// directly via ratSpan rather than through a record — pin this: the one
// INTERNAL joint between them turns +1 (TestDegreeOneSpansCarryAZeroCurvatureNumerator's
// own turn), so the open chain's verdict is that turn's own sign. The joint
// that would CLOSE the loop, from the second span back to the first, turns
// -1 the other way — a conflict an open fold never reads and a closed fold
// must.
func TestClosedChainAddsTheWrapJointAnOpenChainNeverReads(t *testing.T) {
	spanA := ratSpan([][2]float64{{0, 0}, {1, 0}})
	spanB := ratSpan([][2]float64{{1, 0}, {1, 1}})
	spans := []bezierSpan{spanA, spanB}

	require.Equal(t, "1", jointCross(spanA, spanB).RatString(),
		"the internal joint turns by exactly +1, the identical shape and figure degreeOneNURBS's own conversion carries")
	require.Equal(t, "-1", jointCross(spanB, spanA).RatString(),
		"the joint that would close the loop turns the other way")

	openVerdict, err := freeformWallConvexityContext(t.Context(), spans, false, false)
	require.NoError(t, err, "an open chain never reads the closing joint")
	require.Equal(t, freeformConvexityPositive, openVerdict)

	_, err = freeformWallConvexityContext(t.Context(), spans, true, false)
	require.ErrorIs(t, err, ErrUnsupported,
		"a closed chain folds the closing joint in too, and it conflicts with the internal turn — refuse R19")
}

// spanStrings renders a span's control points exactly, so a conversion that
// rounded anywhere fails the comparison rather than passing within a delta.
func spanStrings(span bezierSpan) [][]string {
	out := make([][]string, len(span))
	for i, p := range span {
		out[i] = []string{p.u.RatString(), p.v.RatString()}
	}
	return out
}
