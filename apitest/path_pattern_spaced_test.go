package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/path-pattern-design.md §6's public-surface tests for
// PathPattern with WithPathSpacing, on the 4 x 4 x 2 block of path_pattern_test.go.

func TestPathPatternSpacingPlacesCopiesAtTheStatedDistance(t *testing.T) {
	t.Parallel()
	doc, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(units.Millimeters(12)))
	require.NoError(t, err)
	require.Len(t, copies, 2, "copies at 12 and 24 mm; 36 mm lies past the path's end")
	require.Len(t, doc.Bodies(), 3)
	for i, c := range copies {
		x := 12 * float64(i+1)
		requireBoundsNear(t, c, r3.Vec{X: x}, r3.Vec{X: x + 4, Y: 4, Z: 2})
		decadtest.MeasuresVolume(t, c, units.CubicMillimeters(32))
	}
}

func TestPathPatternSpacingPutsTheLastCopyAtTheEndWhenItDivides(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(units.Millimeters(10)))
	require.NoError(t, err)
	require.Len(t, copies, 3)
	requireBoundsNear(t, copies[2], r3.Vec{X: 30}, r3.Vec{X: 34, Y: 4, Z: 2})
}

func TestPathPatternSpacingMeasuresAlongAnArc(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	// A quarter of the radius-10 circle about the origin, from (10, 0): 5π mm
	// long, so a spacing of 2.5π gives one copy at its middle and one at its end.
	mid := 10 / math.Sqrt2
	path, err := decad.NewPath(r3.Vec{X: 10}, decad.ArcThrough{Through: r3.Vec{X: mid, Y: mid}, End: r3.Vec{Y: 10}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(units.Millimeters(2.5*math.Pi)))
	require.NoError(t, err)
	require.Len(t, copies, 2)
	requireBoundsNear(t, copies[0], r3.Vec{X: mid - 10, Y: mid}, r3.Vec{X: mid - 6, Y: mid + 4, Z: 2})
	requireBoundsNear(t, copies[1], r3.Vec{X: -10, Y: 10}, r3.Vec{X: -6, Y: 14, Z: 2})
}

func TestPathPatternSpacingFollowsThePath(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 20}}, decad.LineTo{End: r3.Vec{X: 20, Y: 20}})
	require.NoError(t, err)
	copies, err := decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(units.Millimeters(30)), decad.WithPathFollow())
	require.NoError(t, err)
	require.Len(t, copies, 1)
	// 30 mm along is 10 mm up the second leg, which runs +Y: a quarter turn.
	lo, hi := turnedBlockBounds(math.Pi/2, r3.Vec{X: 20, Y: 10})
	requireBoundsNear(t, copies[0], lo, hi)
}

func TestPathPatternSpacingRefusals(t *testing.T) {
	t.Parallel()
	doc, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	cases := []struct {
		name    string
		spacing units.Value
		want    error
	}{
		{"zero spacing", units.Millimeters(0), decad.ErrDegenerate},
		{"negative spacing", units.Millimeters(-1), decad.ErrNegativeMagnitude},
		{"spacing as an angle", units.Degrees(5), decad.ErrUnitKind},
		{"longer than the path", units.Millimeters(31), decad.ErrDegenerate},
		{"far too fine", units.Millimeters(1e-6), decad.ErrDegenerate},
	}
	for _, tc := range cases {
		_, err := decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(tc.spacing))
		require.ErrorIs(t, err, tc.want, tc.name)
	}
	require.Equal(t, []*decad.Body{block}, doc.Bodies(), "a refused pattern leaves no copy")
	_, err = decad.PathPattern(t.Context(), block, nil, decad.WithPathSpacing(units.Millimeters(5)))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.PathPattern(t.Context(), block, path, decad.WithPathSpacing(units.Millimeters(5)), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
