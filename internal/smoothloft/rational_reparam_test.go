package smoothloft

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func TestRationalReparameterizationMatchesBlendedTangents(t *testing.T) {
	center := twistTestPoint(0, 0)
	source := [2]rationalCurve{
		circularChart(rint(1), center, 0, rint(0), big.NewRat(1, 3)),
		circularChart(rint(1), center, 0, big.NewRat(1, 3), rint(1)),
	}
	point := func(x, y *big.Rat) freeform.RatPoint { return freeform.RatPoint{U: x, V: y} }
	unit := polynomial.RatPoly{rint(1)}
	fit := [2]rationalCurve{
		fittedOnDenominator(twistLineSpan(point(rint(1), rint(0)), point(big.NewRat(1, 2), big.NewRat(1, 2))), unit),
		fittedOnDenominator(twistLineSpan(point(big.NewRat(1, 2), big.NewRat(1, 2)), point(rint(0), rint(1))), unit),
	}
	blended := func(a, b rationalCurve, end int64) freeform.RatPoint {
		x, y := rationalEndpointDerivative(a, end), rationalEndpointDerivative(b, end)
		s := big.NewRat(44, 125) // smoothstep(2/5)
		return twistAdd(twistScale(x, sub(rint(1), s)), twistScale(y, s))
	}
	cross := func(a, b freeform.RatPoint) *big.Rat { return sub(mul(a.U, b.V), mul(a.V, b.U)) }
	beforeLeft, beforeRight := blended(source[0], fit[0], 1), blended(source[1], fit[1], 0)
	require.NotZero(t, cross(beforeLeft, beforeRight).Sign(), "premise: the unreparameterized blend has a crease")
	s0, s1, ok := rationalJoinSlopes(source[0], source[1])
	require.True(t, ok)
	f0, f1, ok := rationalJoinSlopes(fit[0], fit[1])
	require.True(t, ok)
	original := source[0]
	source[0], source[1] = rationalReparameterize(source[0], rint(1), s0), rationalReparameterize(source[1], s1, rint(1))
	fit[0], fit[1] = rationalReparameterize(fit[0], rint(1), f0), rationalReparameterize(fit[1], f1, rint(1))
	afterLeft, afterRight := blended(source[0], fit[0], 1), blended(source[1], fit[1], 0)
	require.Zero(t, afterLeft.U.Cmp(afterRight.U))
	require.Zero(t, afterLeft.V.Cmp(afterRight.V))
	u := big.NewRat(1, 3)
	q := polynomial.BernsteinValue([]*big.Rat{rint(0), big.NewRat(1, 3), sub(rint(1), div(s0, rint(3))), rint(1)}, u)
	for _, pair := range [][2]polynomial.RatPoly{{original.x, source[0].x}, {original.y, source[0].y}} {
		want := div(polynomial.RpEval(pair[0], q), polynomial.RpEval(original.w, q))
		got := div(polynomial.RpEval(pair[1], u), polynomial.RpEval(source[0].w, u))
		require.Zero(t, got.Cmp(want))
	}
}

func TestRationalSmoothPatchesRejectsUnequalRatios(t *testing.T) {
	center := twistTestPoint(0, 0)
	mid := freeform.RatPoint{U: big.NewRat(1, 2), V: big.NewRat(1, 2)}
	spans := [2]twistSpan{twistLineSpan(twistTestPoint(1, 0), mid), twistLineSpan(mid, twistTestPoint(0, 1))}
	patches := make([][]rationalTransition, 2)
	for i, interval := range [][2]*big.Rat{{rint(0), big.NewRat(1, 3)}, {big.NewRat(1, 3), rint(1)}} {
		source := circularChart(rint(1), center, 0, interval[0], interval[1])
		patches[i] = []rationalTransition{{a: source, b: fittedOnDenominator(spans[i], source.w)}}
	}
	_, err := rationalSmoothPatches(t.Context(), patches, []int{0, 1}, 1, true)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "unequal parameter-speed ratios")
}

func TestRationalSmoothPatchesKeepsExactJoinTangent(t *testing.T) {
	center := twistTestPoint(0, 0)
	spans := [2]twistSpan{
		twistLineSpan(twistTestPoint(1, 0), freeform.RatPoint{U: big.NewRat(1, 2), V: big.NewRat(1, 2)}),
		twistLineSpan(freeform.RatPoint{U: big.NewRat(1, 2), V: big.NewRat(1, 2)}, twistTestPoint(0, 1)),
	}
	patches := make([][]rationalTransition, 2)
	for i, interval := range [][2]*big.Rat{{rint(0), big.NewRat(1, 3)}, {big.NewRat(1, 3), rint(1)}} {
		source := circularChart(rint(1), center, 0, interval[0], interval[1])
		patches[i] = []rationalTransition{{a: source, b: fittedOnDenominator(spans[i], source.w)}}
	}
	got, err := rationalSmoothPatches(t.Context(), patches, []int{0, 1}, 1, false)
	require.NoError(t, err)
	// At t=2/5, exact one-sided derivatives must agree, not merely be close.
	blended := func(p rationalTransition) rationalCurve {
		s := big.NewRat(44, 125)
		return rationalCurve{
			x: polynomial.RpAdd(polynomial.RpScale(p.a.x, sub(rint(1), s)), polynomial.RpScale(p.b.x, s)),
			y: polynomial.RpAdd(polynomial.RpScale(p.a.y, sub(rint(1), s)), polynomial.RpScale(p.b.y, s)),
			w: p.a.w,
		}
	}
	left := rationalEndpointDerivative(blended(got[0][0]), 1)
	right := rationalEndpointDerivative(blended(got[1][0]), 0)
	require.Zero(t, left.U.Cmp(right.U))
	require.Zero(t, left.V.Cmp(right.V))
}

func TestRationalJoinSlopesRejectsExactTangentMismatch(t *testing.T) {
	unit := polynomial.RatPoly{rint(1)}
	left := fittedOnDenominator(twistLineSpan(twistTestPoint(0, 0), twistTestPoint(1, 0)), unit)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 54))
	right := fittedOnDenominator(twistLineSpan(twistTestPoint(1, 0), freeform.RatPoint{U: rint(2), V: tiny}), unit)
	_, _, ok := rationalJoinSlopes(left, right)
	require.False(t, ok, "a nonzero exact tangent determinant must not be rounded away")
	right = fittedOnDenominator(twistLineSpan(twistTestPoint(2, 0), twistTestPoint(3, 0)), unit)
	_, _, ok = rationalJoinSlopes(left, right)
	require.False(t, ok, "parallel tangents at different points do not form a join")
}
