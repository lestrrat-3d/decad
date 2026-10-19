package freeform_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func TestSpanExtremeEnclosureAtRationalStationAndCollapsedSpan(t *testing.T) {
	t.Parallel()
	span := freeform.BezierSpan{
		{U: polynomial.MustRatOf(0), V: polynomial.MustRatOf(0)},
		{U: polynomial.MustRatOf(2), V: polynomial.MustRatOf(0)},
		{U: polynomial.MustRatOf(0), V: polynomial.MustRatOf(0)},
	}
	_, maximum, err := freeform.SpanExtremeEnclosureContext(t.Context(), span, 1, 0, freeform.NewFreeformWork())
	require.NoError(t, err)
	truth := polynomial.MustRatOf(1)
	require.LessOrEqual(t, maximum.Lo.Cmp(truth), 0)
	require.GreaterOrEqual(t, maximum.Hi.Cmp(truth), 0)

	collapsed := freeform.BezierSpan{
		{U: polynomial.MustRatOf(5), V: polynomial.MustRatOf(-1)},
		{U: polynomial.MustRatOf(5), V: polynomial.MustRatOf(-1)},
		{U: polynomial.MustRatOf(5), V: polynomial.MustRatOf(-1)},
	}
	minimum, maximum, err := freeform.SpanExtremeEnclosureContext(t.Context(), collapsed, 1, 1, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Zero(t, minimum.Lo.Cmp(polynomial.MustRatOf(4)))
	require.Zero(t, maximum.Hi.Cmp(polynomial.MustRatOf(4)))
}
