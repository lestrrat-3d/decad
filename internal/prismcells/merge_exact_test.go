package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestExactLineLoopRefusesUnauthenticatedCutEdges(t *testing.T) {
	world := sketch.NewWorld()
	scene, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(a, b sectionrecord.Point2, t0, t1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: t0, TEnd: t1}
	}
	segments := []sectionrecord.CurveSegment{
		line(point(5, -5), point(5, 5), 0, 0.3),
		line(point(3, -2), point(7, -2), 0.5, 1),
		line(point(7, -2), point(5, -5), 0, 1),
	}
	// These fields mimic a certified cut, but they lack the live Sketch event.
	edges := []sketch.BoundaryEdge{
		{Partial: true, TExact: true, Polyline: [][2]float64{{5, -5}, {5, -2}}},
		{Partial: true, TExact: true, Polyline: [][2]float64{{5, -2}, {7, -2}}},
		{},
	}
	_, ok := reparameterizeExactLineLoop(scene, edges, segments)
	require.False(t, ok)
	_, ok = reparameterizeExactLineLoop(nil, edges, segments)
	require.False(t, ok)
}
