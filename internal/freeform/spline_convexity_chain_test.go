package freeform

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func TestInteriorCuspFailsClosedSpanRegularity(t *testing.T) {
	t.Parallel()
	span := BezierSpan{
		{U: big.NewRat(-1, 8), V: big.NewRat(1, 4)},
		{U: big.NewRat(1, 8), V: big.NewRat(-1, 12)},
		{U: big.NewRat(-1, 8), V: big.NewRat(-1, 12)},
		{U: big.NewRat(1, 8), V: big.NewRat(1, 4)},
	}
	k := polynomial.RpTrim(CurvatureNumerator(span))
	require.Len(t, k, 3)
	for i, want := range []string{"-3/2", "6", "-6"} {
		require.Equal(t, want, k[i].RatString())
	}
	speed := spanSquaredSpeed(span)
	require.Zero(t, polynomial.RpEval(speed, big.NewRat(1, 2)).Sign())
	_, err := SpanConvexitySignContext(t.Context(), span, NewFreeformWork())
	require.ErrorIs(t, err, ErrFreeformConvexitySpeedInterior)
}

func TestEndpointCuspFailsClosedSpanRegularity(t *testing.T) {
	t.Parallel()
	span := BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 3), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(1, 1)},
	}
	speed := spanSquaredSpeed(span)
	require.Zero(t, polynomial.RpEval(speed, big.NewRat(0, 1)).Sign())
	_, err := SpanConvexitySignContext(t.Context(), span, NewFreeformWork())
	require.ErrorIs(t, err, ErrFreeformConvexitySpeedAtStart)
}

func TestConflictingSpanCurvatureCannotBeMaskedByFitInterpolation(t *testing.T) {
	t.Parallel()
	positive := quadraticTestSpan([][2]int{{0, 0}, {1, 0}, {2, 1}})
	negative := quadraticTestSpan([][2]int{{2, 1}, {3, 2}, {4, 2}})
	positiveSign, err := SpanConvexitySignContext(t.Context(), positive, NewFreeformWork())
	require.NoError(t, err)
	require.Equal(t, FreeformConvexityPositive, positiveSign)
	negativeSign, err := SpanConvexitySignContext(t.Context(), negative, NewFreeformWork())
	require.NoError(t, err)
	require.Equal(t, FreeformConvexityNegative, negativeSign)
	_, err = FreeformWallConvexityContext(t.Context(), []BezierSpan{positive, negative}, false, false, true, NewFreeformWork())
	require.ErrorIs(t, err, ErrFreeformConvexityConflict)
}

func quadraticTestSpan(points [][2]int) BezierSpan {
	span := make(BezierSpan, len(points))
	for i, point := range points {
		span[i] = RatPoint{U: big.NewRat(int64(point[0]), 1), V: big.NewRat(int64(point[1]), 1)}
	}
	return span
}

func spanSquaredSpeed(span BezierSpan) polynomial.RatPoly {
	u, v := SpanCoordinatePolys(span)
	du, dv := polynomial.RpDeriv(u), polynomial.RpDeriv(v)
	return polynomial.RpAdd(polynomial.RpMul(du, du), polynomial.RpMul(dv, dv))
}
