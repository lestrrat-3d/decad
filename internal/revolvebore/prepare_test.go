package revolvebore_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrepareLinearFrustumAndCircularTool(t *testing.T) {
	meridianFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Z: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	toolFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	points := []sectionrecord.Point2{{}, {V: 10}, {U: 32, V: 8}, {U: 32}}
	loop := sectionrecord.LoopRecord{}
	for i, p := range points {
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{Start: p, End: points[(i+1)%len(points)], TEnd: 1})
	}
	rp := revolveplan.Payload{Frame: meridianFrame, Ax: revolveaxis.Frame{DU: 1}, Full: true,
		Xform: r3.Identity(), Profile: momentinput.Profile{Outer: loop}}
	pp := prismrecord.Payload{Frame: toolFrame, Xform: r3.Identity(), Z1: 32,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(2), CCW: true, TEnd: 1},
		}}}}
	target := &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: rp}
	tool := &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: pp}
	candidate, ok, err := revolvebore.Prepare(t.Context(), target, tool)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 8.0, candidate.Meridian.MinRadius)
	require.Len(t, candidate.Caps[0].Holes, 1)
	require.Len(t, candidate.Caps[1].Holes, 1)
	for _, tc := range []struct {
		name string
		edit func(*prismrecord.Payload)
	}{
		{"cap inside", func(p *prismrecord.Payload) { p.Z1 = math.Nextafter(32, 0) }},
		{"cap outside", func(p *prismrecord.Payload) { p.Z1 = math.Nextafter(32, 33) }},
		{"displaced section", func(p *prismrecord.Payload) { p.SectionDelta = 0.01 }},
		{"touches minimum radius", func(p *prismrecord.Payload) {
			p.Profile.Outer = sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(8), CCW: true, TEnd: 1},
			}}
		}},
		{"crosses exterior", func(p *prismrecord.Payload) {
			p.Profile.Outer = sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(9), CCW: true, TEnd: 1},
			}}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			altered := pp
			tc.edit(&altered)
			other := &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: altered}
			_, ok, err := revolvebore.Prepare(t.Context(), target, other)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
}
