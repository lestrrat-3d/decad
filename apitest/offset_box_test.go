package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func offsetBoxSketch(tb testing.TB, ctx context.Context, x0, y0, x1, y1, z0 float64) *sketch.Sketch {
	tb.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(ctx)
	require.NoError(tb, err)
	return s
}
