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

// This file is agent-docs/path-pattern-design.md §5's public-surface tests for
// WithPathFollow, on the 4 x 4 x 2 block of path_pattern_test.go.

// followPath runs 10 mm along +X, then turns a quarter circle of radius 10 about
// (10, 10) counter-clockwise, to end at (20, 10) heading +Y.
func followPath(t *testing.T) *decad.Path {
	t.Helper()
	mid := 10 / math.Sqrt2
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.Vec{X: 10}},
		decad.ArcThrough{Through: r3.Vec{X: 10 + mid, Y: 10 - mid}, End: r3.Vec{X: 20, Y: 10}})
	require.NoError(t, err)
	return path
}

// turnedBlockBounds returns the bounds of the 4 x 4 x 2 block turned by angle
// about Z through the origin and then moved to at.
func turnedBlockBounds(angle float64, at r3.Vec) (r3.Vec, r3.Vec) {
	sin, cos := math.Sincos(angle)
	lo, hi := r3.Vec{X: math.Inf(1), Y: math.Inf(1)}, r3.Vec{X: math.Inf(-1), Y: math.Inf(-1)}
	for _, c := range [][2]float64{{0, 0}, {4, 0}, {4, 4}, {0, 4}} {
		x, y := c[0]*cos-c[1]*sin+at.X, c[0]*sin+c[1]*cos+at.Y
		lo.X, lo.Y = min(lo.X, x), min(lo.Y, y)
		hi.X, hi.Y = max(hi.X, x), max(hi.Y, y)
	}
	lo.Z, hi.Z = 0, 2
	return lo, hi
}

func TestPathPatternFollowTurnsCopiesWithThePath(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	copies, err := decad.PathPatternCopies(t.Context(), block, followPath(t), 3, decad.WithPathFollow())
	require.NoError(t, err)
	require.Len(t, copies, 2)
	// The middle copy sits on the arc at the length (10 + 5π)/2, which is
	// 2.5π − 5 mm round it, so a turn of (2.5π − 5)/10 radians.
	turn := (2.5*math.Pi - 5) / 10
	lo, hi := turnedBlockBounds(turn, r3.Vec{X: 10 + 10*math.Sin(turn), Y: 10 - 10*math.Cos(turn)})
	requireBoundsNear(t, copies[0], lo, hi)
	// The last copy ends the path heading +Y, a quarter turn.
	lo, hi = turnedBlockBounds(math.Pi/2, r3.Vec{X: 20, Y: 10})
	requireBoundsNear(t, copies[1], lo, hi)
	for _, c := range copies {
		decadtest.MeasuresVolume(t, c, units.CubicMillimeters(32))
	}
}

func TestPathPatternFollowOnAStraightPathKeepsTheOrientation(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30, Y: 30}})
	require.NoError(t, err)
	copies, err := decad.PathPatternCopies(t.Context(), block, path, 2, decad.WithPathFollow())
	require.NoError(t, err)
	requireBoundsNear(t, copies[0], r3.Vec{X: 30, Y: 30}, r3.Vec{X: 34, Y: 34, Z: 2})
}

func TestPathPatternFollowRefusals(t *testing.T) {
	t.Parallel()
	_, block := patternBlock(t)
	doubled, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 10}}, decad.LineTo{End: r3.Vec{}})
	require.NoError(t, err)
	_, err = decad.PathPatternCopies(t.Context(), block, doubled, 3, decad.WithPathFollow())
	require.ErrorIs(t, err, decad.ErrDegenerate, "a path that ends heading back along its start has no shortest turn")
	_, err = decad.PathPatternCopies(t.Context(), block, doubled, 3)
	require.NoError(t, err, "without the option the same path is a plain pattern")
	_, err = decad.PathPatternCopies(t.Context(), block, followPath(t), 3, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
