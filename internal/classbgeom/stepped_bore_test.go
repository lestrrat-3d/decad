package classbgeom

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func boreRectangle(t *testing.T, size, z1 float64) prismrecord.Payload {
	t.Helper()
	points := []sectionrecord.Point2{
		{U: -size, V: -size}, {U: size, V: -size},
		{U: size, V: size}, {U: -size, V: size},
	}
	loop := sectionrecord.LoopRecord{}
	for i, start := range points {
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{
			Start: start, End: points[(i+1)%len(points)], TStart: 0, TEnd: 1,
		})
	}
	step := stepOf(t, 8, 0, z1)
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: loop}, Frame: step.Frame,
		Z0: 0, Z1: z1, Xform: r3.Identity(),
	}
}

func TestCutSteppedBoreView(t *testing.T) {
	t.Parallel()
	steps := []prismrecord.Payload{stepOf(t, 8, 0, 10), stepOf(t, 10, 10, 20), stepOf(t, 6, 20, 35)}
	tool := boreRectangle(t, 1, 35)
	got, ok, err := cutSteppedBore(t.Context(), steps, tool)
	require.NoError(t, err)
	require.True(t, ok)
	brep, ok := got.(breprecord.Payload)
	require.True(t, ok)
	require.Len(t, brep.Faces, 11)
}

func TestCutSteppedBoreAntiparallelTool(t *testing.T) {
	t.Parallel()
	steps := []prismrecord.Payload{stepOf(t, 8, 0, 10), stepOf(t, 10, 10, 20), stepOf(t, 6, 20, 35)}
	target := steps[0].Frame
	tool := boreRectangle(t, 1, 35)
	toolFrame, err := r3.NewFrame(target.Origin().Add(target.N().Scale(35)), target.U(), target.V().Scale(-1))
	require.NoError(t, err)
	tool.Frame = toolFrame
	got, ok, err := cutSteppedBore(t.Context(), steps, tool)
	require.NoError(t, err)
	require.True(t, ok)
	brep, ok := got.(breprecord.Payload)
	require.True(t, ok)
	require.Len(t, brep.Faces, 11)
}

func TestCutSteppedBoreMissesUnsupportedTools(t *testing.T) {
	t.Parallel()
	steps := []prismrecord.Payload{stepOf(t, 8, 0, 10), stepOf(t, 10, 10, 20), stepOf(t, 6, 20, 35)}
	for _, tc := range []struct {
		name string
		tool func(*testing.T) topo.Payload
	}{
		{name: "blind tool", tool: func(t *testing.T) topo.Payload { return boreRectangle(t, 1, 30) }},
		{name: "tool breaks outer wall", tool: func(t *testing.T) topo.Payload { return boreRectangle(t, 7, 35) }},
		{name: "different placement", tool: func(t *testing.T) topo.Payload {
			tool := boreRectangle(t, 1, 35)
			move, err := r3.Translation(r3.Vec{X: 1})
			require.NoError(t, err)
			tool.Xform = move
			return tool
		}},
		{name: "rotated section frame", tool: func(t *testing.T) topo.Payload {
			tool := boreRectangle(t, 1, 35)
			a := math.Sqrt(0.5)
			frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: a, Z: a}, r3.Vec{Y: -a, Z: a})
			require.NoError(t, err)
			tool.Frame = frame
			return tool
		}},
		{name: "tool has a hole", tool: func(t *testing.T) topo.Payload {
			tool := boreRectangle(t, 1, 35)
			tool.Profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(0.25), CCW: false, TStart: 1, TEnd: 0},
			}}}
			return tool
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok, err := cutSteppedBore(t.Context(), steps, tc.tool(t))
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
}
