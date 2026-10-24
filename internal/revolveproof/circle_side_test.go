package revolveproof_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// circleFragmentScene cuts a circle of radius r about the sketch origin with a
// vertical line at local u = cutU. It returns the frame (a plane normal to the
// cone axis, 10 mm from the apex) and the circle fragment left of the line.
func circleFragmentScene(t *testing.T, r, cutU float64) (r3.Frame, *sketch.Circle, sketch.BoundaryEdge) {
	t.Helper()
	return circleFragmentSceneOn(t, r3.Vec{X: 10}, r3.Vec{Y: 1}, r3.Vec{Z: 1}, r, cutU)
}

// circleFragmentSceneOn is circleFragmentScene on the plane with the given
// frame.
func circleFragmentSceneOn(t *testing.T, origin, u, v r3.Vec, r, cutU float64) (
	r3.Frame, *sketch.Circle, sketch.BoundaryEdge) {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	scene, err := world.CreateSketch(plane)
	require.NoError(t, err)
	circle := scene.CreateCircle(scene.CreatePoint(0, 0), r)
	scene.CreateLine(scene.CreatePoint(cutU, -2*r), scene.CreatePoint(cutU, 2*r))
	for _, profile := range scene.Profiles() {
		for _, edge := range profile.Outer {
			if edge.Entity == sketch.Entity(circle) && edge.Partial && edge.TStart < 0.5 && edge.TEnd > 0.5 {
				return frame, circle, edge
			}
		}
	}
	require.FailNow(t, "no circle fragment around the -u direction")
	return frame, circle, sketch.BoundaryEdge{}
}

func circleSegment(circle *sketch.Circle, edge sketch.BoundaryEdge) sectionrecord.CircleSeg {
	return sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: circle.Center.X(), V: circle.Center.Y()},
		Radius: units.Millimeters(circle.R()), CCW: true, TStart: edge.TStart, TEnd: edge.TEnd,
	}
}

func TestCircleFragmentSide(t *testing.T) {
	cone := sketch.ConeSection{Axis: r3.Vec{X: 1}, Slope: 1}

	t.Run("inside", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 3, -2.9)
		side, margin, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.True(t, ok)
		require.Equal(t, sketch.ConeInside, side)
		// |F| = 10^2 - 3^2 = 91 on the circle; the box widens it a little.
		require.Positive(t, margin.Cmp(big.NewRat(85, 1)))
		require.Negative(t, margin.Cmp(big.NewRat(91, 1)))
	})
	t.Run("outside", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 20, -19.5)
		side, margin, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.True(t, ok)
		require.Equal(t, sketch.ConeOutside, side)
		require.Positive(t, margin.Cmp(big.NewRat(270, 1))) // box edge u >= 19.5: 19.5^2 - 10^2 = 280
	})
	t.Run("touching the cone", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 10, -9.5)
		_, _, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.False(t, ok)
	})
	t.Run("whole edge", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 3, -2.9)
		edge.Partial = false
		_, _, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.False(t, ok)
	})
	t.Run("bounds not exact", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 3, -2.9)
		edge.TExact = false
		_, _, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.False(t, ok)
	})
	t.Run("record range differs", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 3, -2.9)
		segment := circleSegment(circle, edge)
		segment.TEnd = edge.TEnd + 0.01
		_, _, ok := revolveproof.CircleFragmentSide(frame, circle, edge, segment, cone)
		require.False(t, ok)
	})
	t.Run("wider than a quarter turn", func(t *testing.T) {
		frame, circle, edge := circleFragmentScene(t, 3, -0.5)
		_, _, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge), cone)
		require.False(t, ok)
	})
}

// A cone about another axis reads Sketch's own F = |q×A|² − s²(q·A)²
// (agent-docs/revolve-axis-design.md §5.3). The scene mapped by the exact
// rotation (x, y, z) → (z, x, y) about A = (0, 1, 0) proves the same side with
// the same margin; about A = (1, 2, 0) the margin scales by about A·A = 5.
func TestCircleFragmentSideAboutAxis(t *testing.T) {
	for _, tc := range []struct {
		name   string
		r, cut float64
		side   sketch.ConeSide
	}{{"inside", 3, -2.9, sketch.ConeInside}, {"outside", 20, -19.5, sketch.ConeOutside}} {
		t.Run(tc.name, func(t *testing.T) {
			frame, circle, edge := circleFragmentScene(t, tc.r, tc.cut)
			xSide, xMargin, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge),
				sketch.ConeSection{Axis: r3.Vec{X: 1}, Slope: 1})
			require.True(t, ok)
			require.Equal(t, tc.side, xSide)

			frame, circle, edge = circleFragmentSceneOn(t, r3.Vec{Y: 10}, r3.Vec{Z: 1}, r3.Vec{X: 1}, tc.r, tc.cut)
			side, margin, ok := revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge),
				sketch.ConeSection{Axis: r3.Vec{Y: 1}, Slope: 1})
			require.True(t, ok)
			require.Equal(t, tc.side, side)
			require.Zero(t, xMargin.Cmp(margin), "x margin %s, permuted %s", xMargin, margin)

			s5 := math.Sqrt(5)
			frame, circle, edge = circleFragmentSceneOn(t, r3.Vec{X: 10 / s5, Y: 20 / s5},
				r3.Vec{X: -2 / s5, Y: 1 / s5}, r3.Vec{Z: 1}, tc.r, tc.cut)
			side, margin, ok = revolveproof.CircleFragmentSide(frame, circle, edge, circleSegment(circle, edge),
				sketch.ConeSection{Axis: r3.Vec{X: 1, Y: 2}, Slope: 1})
			require.True(t, ok)
			require.Equal(t, tc.side, side)
			scaled := new(big.Rat).Mul(xMargin, big.NewRat(5, 1))
			ratio, _ := new(big.Rat).Quo(margin, scaled).Float64()
			// The world box of the oblique fragment is wider than its frame box,
			// so the margin is slightly smaller than five times the +X margin.
			require.InDelta(t, 1, ratio, 0.05, "the margin about (1, 2, 0) is five times the +X margin")
		})
	}
}

func TestFitRangeMatchesEdge(t *testing.T) {
	tests := map[string]struct {
		segment sectionrecord.FitSplineSeg
		edge    sketch.BoundaryEdge
		want    bool
	}{
		"whole forward": {sectionrecord.FitSplineSeg{TStart: 0, TEnd: 1}, sketch.BoundaryEdge{TStart: 0, TEnd: 1}, true},
		"whole reversed": {sectionrecord.FitSplineSeg{TStart: 1, TEnd: 0},
			sketch.BoundaryEdge{TStart: 0, TEnd: 1, Reversed: true}, true},
		"whole with a range": {sectionrecord.FitSplineSeg{TStart: 0.2, TEnd: 1}, sketch.BoundaryEdge{}, false},
		"partial forward": {sectionrecord.FitSplineSeg{TStart: 0.25, TEnd: 1},
			sketch.BoundaryEdge{Partial: true, TExact: true, TStart: 0.25, TEnd: 1}, true},
		"partial reversed": {sectionrecord.FitSplineSeg{TStart: 1, TEnd: 0.25},
			sketch.BoundaryEdge{Partial: true, TExact: true, Reversed: true, TStart: 0.25, TEnd: 1}, true},
		"partial not exact": {sectionrecord.FitSplineSeg{TStart: 0.25, TEnd: 1},
			sketch.BoundaryEdge{Partial: true, TStart: 0.25, TEnd: 1}, false},
		"partial record differs": {sectionrecord.FitSplineSeg{TStart: 0.3, TEnd: 1},
			sketch.BoundaryEdge{Partial: true, TExact: true, TStart: 0.25, TEnd: 1}, false},
		"partial record in the wrong direction": {sectionrecord.FitSplineSeg{TStart: 1, TEnd: 0.25},
			sketch.BoundaryEdge{Partial: true, TExact: true, TStart: 0.25, TEnd: 1}, false},
		"partial empty range": {sectionrecord.FitSplineSeg{TStart: 0.5, TEnd: 0.5},
			sketch.BoundaryEdge{Partial: true, TExact: true, TStart: 0.5, TEnd: 0.5}, false},
	}
	for name, test := range tests {
		t.Run(name, func(t *testing.T) {
			require.Equal(t, test.want, revolveproof.FitRangeMatchesEdge(test.segment, test.edge))
		})
	}
}
