package booleaneval

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestClipOverlapDiameterStreamsTargetTriangles(t *testing.T) {
	t.Parallel()

	tool := overlapClipTestTool(t)
	t.Run("faceted target", func(t *testing.T) {
		target := &topo.Body{Payload: facetedrecord.Payload{
			Verts: []r3.Vec{{X: -0.5, Y: -0.5, Z: 1}, {X: 0.5, Y: -0.5, Z: 1}, {Y: 0.5, Z: 1}},
			Tris:  [][3]int{{0, 1, 2}},
		}}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(math.Sqrt(2)), math.Float64bits(diameter))
	})

	t.Run("faceted diagonal crossing", func(t *testing.T) {
		target := &topo.Body{
			Payload: facetedrecord.Payload{
				Verts: []r3.Vec{{X: -2, Y: -2, Z: 1}, {X: 2, Y: 2, Z: 1}, {Y: -3, Z: 1}},
				Tris:  [][3]int{{0, 1, 2}},
			},
		}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(math.Sqrt(2)), math.Float64bits(diameter))
	})

	t.Run("box fallback", func(t *testing.T) {
		target := &topo.Body{Bounds: boxForOverlapTest(-2, 2, -2, 2, 0.5, 1.5)}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(math.Sqrt(3)), math.Float64bits(diameter))
	})

	t.Run("empty facets use box fallback", func(t *testing.T) {
		target := &topo.Body{
			Payload: facetedrecord.Payload{},
			Bounds:  boxForOverlapTest(-2, 2, -2, 2, 0.5, 1.5),
		}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(math.Sqrt(3)), math.Float64bits(diameter))
	})

	t.Run("faceted target outside tool", func(t *testing.T) {
		target := &topo.Body{Payload: facetedrecord.Payload{
			Verts: []r3.Vec{{X: -0.5, Y: -0.5, Z: 3}, {X: 0.5, Y: -0.5, Z: 3}, {Y: 0.5, Z: 3}},
			Tris:  [][3]int{{0, 1, 2}},
		}}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.False(t, ok)
		require.Zero(t, diameter)
	})

	t.Run("no clipped target points", func(t *testing.T) {
		target := &topo.Body{Bounds: boxForOverlapTest(-2, 2, -2, 2, 3, 4)}
		diameter, ok := clipOverlapDiameter(target, tool)
		require.False(t, ok)
		require.Zero(t, diameter)
	})
}

func overlapClipTestTool(t *testing.T) *topo.Body {
	t.Helper()

	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: point(-0.5, -0.5), End: point(0.5, -0.5)},
		sectionrecord.LineSeg{Start: point(0.5, -0.5), End: point(0.5, 0.5)},
		sectionrecord.LineSeg{Start: point(0.5, 0.5), End: point(-0.5, 0.5)},
		sectionrecord.LineSeg{Start: point(-0.5, 0.5), End: point(-0.5, -0.5)},
	}}
	return &topo.Body{Payload: prismrecord.Payload{
		Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 2,
		Profile: momentinput.Profile{Outer: loop},
	}}
}

func boxForOverlapTest(x0, x1, y0, y1, z0, z1 float64) measurement.Box {
	return measurement.Box{Min: r3.Vec{X: x0, Y: y0, Z: z0}, Max: r3.Vec{X: x1, Y: y1, Z: z1}}
}
