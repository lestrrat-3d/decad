package decad

import (
	"context"
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestMomentValidationCancellationIsBounded(t *testing.T) {
	segments := make([]CurveSegment, workPollInterval+64)
	for i := range segments {
		start := Point2{U: float64(i), V: 0}
		segments[i] = LineSeg{Start: start, End: Point2{U: start.U + 1, V: math.Sin(float64(i))}, TEnd: 1}
	}
	record := ProfileRecord{Outer: LoopRecord{Segments: segments}}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "validateMomentFieldsBudget"}

	_, err := record.integralsBudget(newWorkBudget(ctx))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `moment field validation must poll inside its segment scan`)
}
