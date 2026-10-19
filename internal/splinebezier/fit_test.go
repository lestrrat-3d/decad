package splinebezier

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

func TestFitSplineBezierMatchesSketchInterpolant(t *testing.T) {
	t.Parallel()
	fit := []Point2{{}, {U: 4, V: 3}, {U: 9, V: -1}, {U: 12, V: 2}, {U: 15}}
	coords := make([][2]float64, len(fit))
	for i, point := range fit {
		coords[i] = [2]float64{point.U, point.V}
	}
	want, err := geom.NewFitInterpolant(coords)
	require.NoError(t, err)
	spans, err := FitSplineBezierSpans(FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, len(want.Spans()))
	params := want.Params
	total := params[len(params)-1]
	for i, span := range spans {
		for _, sample := range []float64{0, 0.25, 0.5, 0.75, 1} {
			u, v := evalFitBezierSpan(span, big.NewRat(int64(sample*4), 4))
			at := (params[i] + sample*(params[i+1]-params[i])) / total
			wantU, wantV := want.Eval(at)
			gotU, _ := u.Float64()
			gotV, _ := v.Float64()
			require.InDelta(t, wantU, gotU, 1e-12)
			require.InDelta(t, wantV, gotV, 1e-12)
		}
	}
}

func TestFitSplineBezierUsesActiveDeduplicatedEndpoint(t *testing.T) {
	t.Parallel()
	fit := []Point2{{}, {U: 10}, {U: 10, V: 10}, {U: 10 + 3e-13, V: 10}}
	spans, err := FitSplineBezierSpans(FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}, freeform.NewFreeformWork())
	require.NoError(t, err)
	start, end, err := FreeformEndpoints(spans, false)
	require.NoError(t, err)
	require.Equal(t, fit[0], start)
	require.Equal(t, fit[2], end)
}

func TestFitInterpolantChargeIsLinearInRecordedPoints(t *testing.T) {
	t.Parallel()
	require.Zero(t, freeform.FitInterpolantCost(0))
	require.Equal(t, uint64(64*10), freeform.FitInterpolantCost(10))
	require.Equal(t, uint64(64*1000), freeform.FitInterpolantCost(1000))
	require.Equal(t, 2*freeform.FitInterpolantCost(500), freeform.FitInterpolantCost(1000))
}

func TestFitSplineBezierTrimPreservesSource(t *testing.T) {
	t.Parallel()
	fit := []Point2{{}, {U: 1, V: 1}, {U: 2}}
	coords := [][2]float64{{0, 0}, {1, 1}, {2, 0}}
	want, err := geom.NewFitInterpolant(coords)
	require.NoError(t, err)
	spans, err := FitSplineBezierSpans(FitSplineSeg{Fit: fit, TStart: 0.25, TEnd: 0.75}, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 2)
	for i, span := range spans {
		for j, at := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 2), big.NewRat(1, 1)} {
			u, v := evalFitBezierSpan(span, at)
			wantU, wantV := want.Eval(0.25 + 0.25*float64(i) + 0.125*float64(j))
			gotU, _ := u.Float64()
			gotV, _ := v.Float64()
			require.InDelta(t, wantU, gotU, 1e-12)
			require.InDelta(t, wantV, gotV, 1e-12)
		}
	}
}

func evalFitBezierSpan(span freeform.BezierSpan, at *big.Rat) (*big.Rat, *big.Rat) {
	u, v := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
		u[i], v[i] = new(big.Rat).Set(point.U), new(big.Rat).Set(point.V)
	}
	oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), at)
	for remaining := len(span) - 1; remaining > 0; remaining-- {
		for i := range remaining {
			u[i] = new(big.Rat).Add(new(big.Rat).Mul(oneMinus, u[i]), new(big.Rat).Mul(at, u[i+1]))
			v[i] = new(big.Rat).Add(new(big.Rat).Mul(oneMinus, v[i]), new(big.Rat).Mul(at, v[i+1]))
		}
	}
	return u[0], v[0]
}
