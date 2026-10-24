package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func fixedPolygonProfile(t *testing.T, world *sketch.World, plane *sketch.Plane,
	vertices [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	sketchModel, err := world.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(vertices))
	for i, vertex := range vertices {
		points[i] = sketchModel.CreatePoint(vertex[0], vertex[1])
		sketchModel.Fix(points[i])
	}
	for i := range points {
		sketchModel.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = sketchModel.Solve(t.Context())
	require.NoError(t, err)
	profiles := sketchModel.Profiles()
	require.Len(t, profiles, 1)
	return sketchModel, profiles[0]
}

func meshPolygonSketch(t *testing.T, world *sketch.World, plane *sketch.Plane,
	vertices [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return fixedPolygonProfile(t, world, plane, vertices)
}

func splitPolygon(t *testing.T, world *sketch.World, plane *sketch.Plane,
	vertices [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return fixedPolygonProfile(t, world, plane, vertices)
}
