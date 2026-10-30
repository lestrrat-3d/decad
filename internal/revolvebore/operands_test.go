package revolvebore_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestOperandsRejectsDisplacedOrNonSolidSources(t *testing.T) {
	rp := revolveplan.Payload{Full: true}
	pp := prismrecord.Payload{Z1: 1, Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{TEnd: 1}},
	}}}
	for _, tc := range []struct {
		name string
		edit func(*topo.Body, *topo.Body)
	}{
		{"solid records", func(_, _ *topo.Body) {}},
		{"sheet target", func(a, _ *topo.Body) { a.Kind = topo.BodySheet }},
		{"invalid tool", func(_, b *topo.Body) { b.Solid = false }},
		{"partial revolve", func(a, _ *topo.Body) { p := rp; p.Full = false; a.Payload = p }},
		{"displaced meridian", func(a, _ *topo.Body) { p := rp; p.SectionDelta = 1; a.Payload = p }},
		{"snapped axis", func(a, _ *topo.Body) { p := rp; p.Ax.Snap.Area = 1; a.Payload = p }},
		{"axis admission charge", func(a, _ *topo.Body) { p := rp; p.Ax.RadialAdmitAllow = 1; a.Payload = p }},
		{"displaced section", func(_, b *topo.Body) { p := pp; p.SectionDelta = 1; b.Payload = p }},
		{"displaced level", func(_, b *topo.Body) { p := pp; p.Z1Delta = 1; b.Payload = p }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: rp}
			b := &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: pp}
			tc.edit(a, b)
			_, _, ok := revolvebore.Operands(a, b)
			require.Equal(t, tc.name == "solid records", ok)
		})
	}
}
