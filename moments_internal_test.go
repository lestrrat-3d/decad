package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// The positive-area gate reads the region's own exact rational wherever there is
// one, and the float accumulator only where there is not. Underflow is why: a
// strictly positive rational can round to a float zero, and refusing it would
// deny a measurement the accumulator already holds. Every non-positive region
// must still refuse, on either arithmetic.
func TestPositiveAreaGateConsultsExactRational(t *testing.T) {
	withExact := func(area *big.Rat, held float64) *regionIntegrals {
		ig := &regionIntegrals{area: held, exact: newExactMoments()}
		ig.exact.area.Set(area)
		return ig
	}
	for _, tc := range []struct {
		name    string
		ig      *regionIntegrals
		refuses bool
	}{
		{
			name: "positive rational underflowing to zero",
			ig:   withExact(big.NewRat(1, 1<<62), 0),
		},
		{
			name: "positive rational and positive float",
			ig:   withExact(big.NewRat(3, 2), 1.5),
		},
		{
			name:    "zero rational",
			ig:      withExact(new(big.Rat), 0),
			refuses: true,
		},
		{
			name:    "negative rational held as a positive float",
			ig:      withExact(big.NewRat(-3, 2), 1.5),
			refuses: true,
		},
		{
			name:    "retired accumulator with a non-positive float",
			ig:      &regionIntegrals{area: 0, exactDead: true},
			refuses: true,
		},
		{
			name: "retired accumulator with a positive float",
			ig:   &regionIntegrals{area: 2, exactDead: true},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := tc.ig.requirePositiveArea()
			if !tc.refuses {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, ErrDegenerate)
			require.Contains(t, err.Error(), "no positive net area")
		})
	}
}

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
