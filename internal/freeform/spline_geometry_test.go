package freeform_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func TestSpanLengthAndSagittaBoundsEncloseCurve(t *testing.T) {
	t.Parallel()
	span := freeform.BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(3, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(4, 1), V: big.NewRat(0, 1)},
	}
	points := make([][2]float64, len(span))
	for i, point := range span {
		points[i][0], _ = point.U.Float64()
		points[i][1], _ = point.V.Float64()
	}
	denseLength := 0.0
	maxDeviation := 0.0
	var previous [2]float64
	for step := range 10000 + 1 {
		parameter := float64(step) / 10000
		work := append([][2]float64(nil), points...)
		for len(work) > 1 {
			for i := 0; i+1 < len(work); i++ {
				work[i][0] = (1-parameter)*work[i][0] + parameter*work[i+1][0]
				work[i][1] = (1-parameter)*work[i][1] + parameter*work[i+1][1]
			}
			work = work[:len(work)-1]
		}
		current := work[0]
		if step > 0 {
			denseLength += math.Hypot(current[0]-previous[0], current[1]-previous[1])
		}
		previous = current
		deviation := math.Abs(current[1])
		maxDeviation = math.Max(maxDeviation, deviation)
	}
	previousWidth := math.Inf(1)
	for _, depth := range []int{0, 2, 4, 6, 8} {
		lo, hi := freeform.SpanLengthBracket(span, depth)
		require.LessOrEqual(t, lo, denseLength)
		require.GreaterOrEqual(t, hi, denseLength)
		require.Less(t, hi-lo, previousWidth)
		previousWidth = hi - lo
	}
	sagitta, err := freeform.SpanSagittaUpper(nil, span)
	require.NoError(t, err)
	require.Greater(t, sagitta, 0.0)
	require.GreaterOrEqual(t, sagitta, maxDeviation)
}

func TestChainStationsRespectSagittaTarget(t *testing.T) {
	t.Parallel()
	span := freeform.BezierSpan{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(3, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(4, 1), V: big.NewRat(0, 1)},
	}
	result, err := freeform.ChainStations([]freeform.BezierSpan{span}, 0.1, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, result.Sagitta, 0.1)
	require.Positive(t, len(result.Stations))
}
