package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/path-pattern-design.md's public-surface tests for
// PathPattern with WithPathCount, on a 4 x 4 x 2 block at the origin.

func patternBlock(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	doc := decad.New()
	return doc, decadtest.NewBlock(tb, doc, 0, 0, 4, 4, units.Millimeters(2))
}

func requireBoundsNear(t *testing.T, b *decad.Body, minimum, maximum r3.Vec) {
	t.Helper()
	box, err := b.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(minimum, 1e-9), "min %v, want %v", box.Min, minimum)
	require.True(t, box.Max.Equal(maximum, 1e-9), "max %v, want %v", box.Max, maximum)
}

func TestPathPatternCountAlongALine(t *testing.T) {
	t.Parallel()
	doc, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathCount(4))
	require.NoError(t, err)
	require.Len(t, copies, 3)
	require.Len(t, doc.Bodies(), 4, "the receiver stays live beside its copies")
	for i, c := range copies {
		x := 10 * float64(i+1)
		requireBoundsNear(t, c, r3.Vec{X: x}, r3.Vec{X: x + 4, Y: 4, Z: 2})
		decadtest.MeasuresVolume(t, c, units.CubicMillimeters(32))
	}
	requireBoundsNear(t, block, r3.Vec{}, r3.Vec{X: 4, Y: 4, Z: 2})
}

func TestPathPatternCountAlongAnArc(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	mid := 10 / math.Sqrt2
	path, err := decad.NewPath(r3.Vec{X: 10},
		decad.ArcThrough{Through: r3.Vec{X: mid, Y: mid}, End: r3.Vec{Y: 10}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathCount(3))
	require.NoError(t, err)
	require.Len(t, copies, 2)
	// The copies keep the block's orientation and move by the chord from the
	// arc's start to its middle and end.
	requireBoundsNear(t, copies[0], r3.Vec{X: mid - 10, Y: mid}, r3.Vec{X: mid - 6, Y: mid + 4, Z: 2})
	requireBoundsNear(t, copies[1], r3.Vec{X: -10, Y: 10}, r3.Vec{X: -6, Y: 14, Z: 2})
}

func TestPathPatternCountSpacesByArcLength(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	// A 20 mm run along X then a 20 mm run along Y: the middle copy of three
	// sits at the corner, 20 mm along the path.
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 20}}, decad.LineTo{End: r3.Vec{X: 20, Y: 20}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathCount(3))
	require.NoError(t, err)
	requireBoundsNear(t, copies[0], r3.Vec{X: 20}, r3.Vec{X: 24, Y: 4, Z: 2})
	requireBoundsNear(t, copies[1], r3.Vec{X: 20, Y: 20}, r3.Vec{X: 24, Y: 24, Z: 2})
}

func TestPathPatternCountRefusals(t *testing.T) {
	t.Parallel()
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	cases := []struct {
		name  string
		path  *decad.Path
		count int
		want  error
	}{
		{"count of one", path, 1, decad.ErrDegenerate},
		{"count of zero", path, 0, decad.ErrDegenerate},
		{"negative count", path, -3, decad.ErrDegenerate},
		{"no path", nil, 3, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, block := patternBlock(t)
			_, err := decad.PathPattern(t.Context(), block, tc.path, decad.WithPathCount(tc.count))
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{block}, doc.Bodies(), "a refused pattern makes no copy")
		})
	}
	t.Run("retired body", func(t *testing.T) {
		t.Parallel()
		_, block := patternBlock(t)
		require.NoError(t, decad.Remove(block))
		_, err := decad.PathPattern(t.Context(), block, path, decad.WithPathCount(3))
		require.ErrorIs(t, err, decad.ErrRetiredBody)
	})
	t.Run("canceled context", func(t *testing.T) {
		t.Parallel()
		doc, block := patternBlock(t)
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := decad.PathPattern(ctx, block, path, decad.WithPathCount(3))
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, []*decad.Body{block}, doc.Bodies())
	})
}

func TestPathPatternStationOptionRefusals(t *testing.T) {
	t.Parallel()
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	ten := units.Millimeters(10)
	cases := []struct {
		name string
		opts []decad.PathPatternOption
	}{
		{"no option", nil},
		{"follow only", []decad.PathPatternOption{decad.WithPathFollow()}},
		{"count then spacing", []decad.PathPatternOption{decad.WithPathCount(3), decad.WithPathSpacing(ten)}},
		{"spacing then count", []decad.PathPatternOption{decad.WithPathSpacing(ten), decad.WithPathCount(3)}},
		{"count twice", []decad.PathPatternOption{decad.WithPathCount(3), decad.WithPathCount(3)}},
		{"spacing twice", []decad.PathPatternOption{decad.WithPathSpacing(ten), decad.WithPathSpacing(ten)}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, block := patternBlock(t)
			_, err := decad.PathPattern(t.Context(), block, path, tc.opts...)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Equal(t, []*decad.Body{block}, doc.Bodies())
			if tc.name != "no option" {
				return
			}
			var de decad.OperationDiagnosticError
			require.ErrorAs(t, err, &de)
			diags := de.OperationDiagnostics()
			require.NotEmpty(t, diags)
			require.Equal(t, decad.OperationCopy, diags[0].Operation)
		})
	}
}
