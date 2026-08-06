package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file pins the two load-bearing facts docs/spline-design.md §6.5 rests
// on. First: a control polygon whose turns all share one sign does NOT bound
// the curve's own curvature sign, which is why a wall edge's convexity is
// certified from the curvature numerator K = u'v" - v'u" in the Bernstein
// basis and never from the polygon's turns. Second: a FitSplineSeg records no
// control points at all, and the fit points it does record neither are the
// converted Bezier chain nor contain the curve — so every rule §6.5 states
// must be phrased over §5.1's converted chain.
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

	k := curvatureNumerator(spans[0])
	at0 := rpEval(k, big.NewRat(0, 1))
	require.Equal(t, 1, at0.Sign(), "K(0) must be positive")
	require.Equal(t, "18", at0.RatString())

	at57 := rpEval(k, big.NewRat(5, 7))
	require.Equal(t, -1, at57.Sign(), "K(5/7) must be negative — the curvature sign the polygon turns did not bound")

	at1 := rpEval(k, big.NewRat(1, 1))
	require.Equal(t, 1, at1.Sign(), "K(1) must be positive: two curvature sign changes, zero polygon-turn sign changes")
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
	b1, _ := spans[0][1].v.Float64()
	b2, _ := spans[0][2].v.Float64()
	require.InDelta(t, -0.0790, b1, 1e-4)
	require.InDelta(t, -0.1580, b2, 1e-4)

	// And so does the curve itself, sampled through the shipped evaluator over
	// the first span's own third of the converted parameter.
	minV := math.Inf(1)
	const samples = 2000
	for i := 0; i <= samples; i++ {
		_, v := evalSpans(t, spans, float64(i)/float64(samples)/3)
		minV = math.Min(minV, v)
	}
	require.Less(t, minV, floor, "the curve leaves the recorded fit points' hull")
	require.InDelta(t, -0.0912, minV, 1e-4)
}
