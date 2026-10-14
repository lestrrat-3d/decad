package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftCircleBoreExactAxisProof(t *testing.T) {
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
			require.Equal(t, tc.want, loftBoreHasExactAxis(bottom, tc.top, 1, 0, 2))
		})
	}
	require.False(t, loftBoreHasExactAxis(bottom, bottom, 1, 0, 0))
}

func TestLoftCircleBoreTopologyShapeRefusals(t *testing.T) {
	type fixture struct {
		pl    loftPayload
		a     loftmesh.Assembly
		start *Face
		end   *Face
		walls []*Face
	}
	valid := func() fixture {
		return fixture{
			pl: loftPayload{profile0: profileRecord{Holes: []loopRecord{{
				Segments: []curveSegment{circleSeg{TStart: 1, TEnd: 0}},
			}}}},
			a:     loftmesh.Assembly{VIdx: [][]int{{}, {}}, Walls: 1},
			start: bornFace(&Face{loops: []*Loop{{}, {}}}),
			end:   bornFace(&Face{loops: []*Loop{{}, {}}}),
			walls: []*Face{bornFace(&Face{})},
		}
	}
	baseline := valid()
	_, ok := loftBoreTopologyCircle(baseline.pl, baseline.a, baseline.start, baseline.end, baseline.walls)
	require.True(t, ok)
	for _, tc := range []struct {
		name   string
		change func(*fixture)
	}{
		{"no hole", func(f *fixture) { f.pl.profile0.Holes = nil }},
		{"extra hole", func(f *fixture) { f.pl.profile0.Holes = append(f.pl.profile0.Holes, loopRecord{}) }},
		{"no segment", func(f *fixture) { f.pl.profile0.Holes[0].Segments = nil }},
		{"extra segment", func(f *fixture) {
			f.pl.profile0.Holes[0].Segments = append(f.pl.profile0.Holes[0].Segments, lineSeg{})
		}},
		{"wrong station count", func(f *fixture) { f.a.VIdx = f.a.VIdx[:1] }},
		{"wrong start loops", func(f *fixture) { f.start.loops = f.start.loops[:1] }},
		{"wrong end loops", func(f *fixture) { f.end.loops = f.end.loops[:1] }},
		{"wrong wall count", func(f *fixture) { f.walls = nil }},
		{"noncircle segment", func(f *fixture) { f.pl.profile0.Holes[0].Segments[0] = lineSeg{} }},
		{"counterclockwise circle", func(f *fixture) {
			f.pl.profile0.Holes[0].Segments[0] = circleSeg{CCW: true, TStart: 1}
		}},
		{"wrong circle start", func(f *fixture) { f.pl.profile0.Holes[0].Segments[0] = circleSeg{TStart: 0} }},
		{"wrong circle end", func(f *fixture) { f.pl.profile0.Holes[0].Segments[0] = circleSeg{TStart: 1, TEnd: 1} }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := valid()
			tc.change(&f)
			_, ok := loftBoreTopologyCircle(f.pl, f.a, f.start, f.end, f.walls)
			require.False(t, ok)
		})
	}
}
