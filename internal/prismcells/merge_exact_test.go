package prismcells

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestReparameterizeExactLineLoopUsesCertifiedIntersectionNode(t *testing.T) {
	line := func(a, b sectionrecord.Point2, t0, t1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: t0, TEnd: t1}
	}
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	segments := []sectionrecord.CurveSegment{
		line(p(5, -5), p(5, 5), 0, 0.3),
		line(p(3, -2), p(7, -2), 0.5, 1),
		line(p(7, -2), p(5, -5), 0, 1),
	}
	edges := []sketch.BoundaryEdge{
		{Partial: true, TExact: true, Polyline: [][2]float64{{5, -5}, {5, -2}}},
		{Partial: true, TExact: true, Polyline: [][2]float64{{5, -2}, {7, -2}}},
		{},
	}
	exact, ok := reparameterizeExactLineLoop(edges, segments)
	require.True(t, ok)
	require.Equal(t, line(p(5, -5), p(5, -2), 0, 1), exact[0])
	require.Equal(t, line(p(5, -2), p(7, -2), 0, 1), exact[1])

	reversedSegments := []sectionrecord.CurveSegment{
		line(p(7, -2), p(5, -5), 1, 0),
		line(p(3, -2), p(7, -2), 1, 0.5),
		line(p(5, -5), p(5, 5), 0.3, 0),
	}
	reversedEdges := []sketch.BoundaryEdge{
		{},
		{Partial: true, TExact: true, Polyline: [][2]float64{{7, -2}, {5, -2}}},
		{Partial: true, TExact: true, Polyline: [][2]float64{{5, -2}, {5, -5}}},
	}
	reversed, ok := reparameterizeExactLineLoop(reversedEdges, reversedSegments)
	require.True(t, ok)
	require.Equal(t, line(p(7, -2), p(5, -2), 0, 1), reversed[1])
	require.Equal(t, line(p(5, -2), p(5, -5), 0, 1), reversed[2])

	wrongNode := append([]sketch.BoundaryEdge(nil), edges...)
	wrongNode[1].Polyline = [][2]float64{{5, math.Nextafter(-2, 0)}, {7, -2}}
	_, ok = reparameterizeExactLineLoop(wrongNode, segments)
	require.False(t, ok)

	wrongCarrier := append([]sketch.BoundaryEdge(nil), edges...)
	wrongCarrier[0].Polyline = [][2]float64{{5, -5}, {5, math.Nextafter(-2, 0)}}
	wrongCarrier[1].Polyline = [][2]float64{{5, math.Nextafter(-2, 0)}, {7, -2}}
	_, ok = reparameterizeExactLineLoop(wrongCarrier, segments)
	require.False(t, ok)

	uncertified := append([]sketch.BoundaryEdge(nil), edges...)
	uncertified[1].TExact = false
	_, ok = reparameterizeExactLineLoop(uncertified, segments)
	require.False(t, ok)

	parallel := append([]sectionrecord.CurveSegment(nil), segments...)
	parallel[1] = line(p(3, -2), p(7, -2), 0.5, 1)
	parallel[0] = line(p(3, -2), p(7, -2), 0, 0.5)
	parallelEdges := append([]sketch.BoundaryEdge(nil), edges...)
	parallelEdges[0].Polyline = [][2]float64{{3, -2}, {5, -2}}
	_, ok = reparameterizeExactLineLoop(parallelEdges, parallel)
	require.False(t, ok)
}
