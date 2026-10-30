package loftrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBoreHasExactAxisProof(t *testing.T) {
	frame := func(origin, u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		return f
	}
	u, v := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	bottom := frame(r3.NewVec(0, 0, 0), u, v)
	for _, tc := range []struct {
		name string
		top  r3.Frame
		want bool
	}{
		{"shared axis", frame(r3.NewVec(0, 0, 2), u, v), true},
		{"origin differs but circle centers agree", frame(r3.NewVec(2, 0, 2), u.Scale(-1), v.Scale(-1)), false},
		{"origin agrees but circle centers differ", frame(r3.NewVec(0, 0, 2), u.Scale(-1), v.Scale(-1)), false},
		{"axis direction differs", frame(r3.NewVec(0, 0, 2), u, v.Scale(-1)), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, BoreHasExactAxis(bottom, tc.top, 1, 0, 2))
		})
	}
	require.False(t, BoreHasExactAxis(bottom, bottom, 1, 0, 0))
}

func TestThroughBoreProfileCircleContainsArcSupports(t *testing.T) {
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: point(-1.5, -1), End: point(1.5, -1), TEnd: 1},
		sectionrecord.ArcSeg{Center: point(1.5, 0), Start: point(1.5, -1), End: point(1.5, 1), TEnd: 1},
		sectionrecord.LineSeg{Start: point(1.5, 1), End: point(-1.5, 1), TEnd: 1},
		sectionrecord.ArcSeg{Center: point(-1.5, 0), Start: point(-1.5, 1), End: point(-1.5, -1), TEnd: 1},
	}}
	container, ok, err := throughBoreProfileCircle(loop)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, point(0, 0), container.Center)
	// Each support circle reaches 2.5 mm from the bounding-box center.
	require.GreaterOrEqual(t, container.Radius.Base(), 2.5)
}

func TestThroughBoreProfileCircleRejectsUnsupportedLoops(t *testing.T) {
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	closed := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: point(-1, -1), End: point(1, -1), TEnd: 1},
		sectionrecord.LineSeg{Start: point(1, -1), End: point(1, 1), TEnd: 1},
		sectionrecord.LineSeg{Start: point(1, 1), End: point(-1, 1), TEnd: 1},
		sectionrecord.LineSeg{Start: point(-1, 1), End: point(-1, -1), TEnd: 1},
	}}
	open := closed
	open.Segments = append([]sectionrecord.CurveSegment(nil), closed.Segments...)
	open.Segments[3] = sectionrecord.LineSeg{Start: point(-1, 1), End: point(-1, 0), TEnd: 1}
	freeform := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{point(-1, -1), point(1, -1), point(1, 1)}},
	}}
	for _, loop := range []sectionrecord.LoopRecord{open, freeform} {
		_, ok, err := throughBoreProfileCircle(loop)
		require.NoError(t, err)
		require.False(t, ok)
	}
}

func TestBoreTopologyShapeRefusals(t *testing.T) {
	type fixture struct {
		pl    Payload
		a     loftmesh.Assembly
		start *topo.Face
		end   *topo.Face
		walls []*topo.Face
	}
	valid := func() fixture {
		return fixture{
			pl: Payload{Profile0: momentinput.Profile{Holes: []sectionrecord.LoopRecord{{
				Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{TStart: 1, TEnd: 0}},
			}}}},
			a:     loftmesh.Assembly{VIdx: [][]int{{}, {}}, Walls: 1},
			start: &topo.Face{Loops: []*topo.Loop{{}, {}}},
			end:   &topo.Face{Loops: []*topo.Loop{{}, {}}},
			walls: []*topo.Face{{}},
		}
	}
	baseline := valid()
	_, ok := boreTopologyCircle(baseline.pl, baseline.a, baseline.start, baseline.end, baseline.walls)
	require.True(t, ok)
	for _, tc := range []struct {
		name   string
		change func(*fixture)
	}{
		{"no hole", func(f *fixture) { f.pl.Profile0.Holes = nil }},
		{"extra hole", func(f *fixture) {
			f.pl.Profile0.Holes = append(f.pl.Profile0.Holes, sectionrecord.LoopRecord{})
		}},
		{"no segment", func(f *fixture) { f.pl.Profile0.Holes[0].Segments = nil }},
		{"extra segment", func(f *fixture) {
			f.pl.Profile0.Holes[0].Segments = append(f.pl.Profile0.Holes[0].Segments, sectionrecord.LineSeg{})
		}},
		{"wrong station count", func(f *fixture) { f.a.VIdx = f.a.VIdx[:1] }},
		{"wrong start loops", func(f *fixture) { f.start.Loops = f.start.Loops[:1] }},
		{"wrong end loops", func(f *fixture) { f.end.Loops = f.end.Loops[:1] }},
		{"wrong wall count", func(f *fixture) { f.walls = nil }},
		{"noncircle segment", func(f *fixture) { f.pl.Profile0.Holes[0].Segments[0] = sectionrecord.LineSeg{} }},
		{"counterclockwise circle", func(f *fixture) {
			f.pl.Profile0.Holes[0].Segments[0] = sectionrecord.CircleSeg{CCW: true, TStart: 1}
		}},
		{"wrong circle start", func(f *fixture) {
			f.pl.Profile0.Holes[0].Segments[0] = sectionrecord.CircleSeg{TStart: 0}
		}},
		{"wrong circle end", func(f *fixture) {
			f.pl.Profile0.Holes[0].Segments[0] = sectionrecord.CircleSeg{TStart: 1, TEnd: 1}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := valid()
			tc.change(&f)
			_, ok := boreTopologyCircle(f.pl, f.a, f.start, f.end, f.walls)
			require.False(t, ok)
		})
	}
}

// TestDirectCircularHoleNeedsBoundedCylinder keeps one far section pair and
// changes only the bore radius. Radius 1 publishes the bounded cylinder; at
// 1e-8 the center's lift rounding bounds no normal, so the wall stays faceted.
func TestDirectCircularHoleNeedsBoundedCylinder(t *testing.T) {
	bottom, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	top, err := r3.NewFrame(r3.NewVec(1e9, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	payload := func(radius float64) Payload {
		hole := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: .427}, Radius: units.Millimeters(radius), TStart: 1}
		profile := momentinput.Profile{Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{hole}}}}
		return Payload{
			AuthenticatedReconstruction: true,
			Profile0:                    profile, Profile1: profile,
			Frame0: bottom, Frame1: top, Xform: r3.Identity(),
		}
	}
	require.Equal(t, WallCylinder, DirectCircularHole(payload(1)))
	require.Equal(t, WallNone, DirectCircularHole(payload(1e-8)))
}
