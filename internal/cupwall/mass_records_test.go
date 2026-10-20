package cupwall_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestViewMassRecords(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	loop := func(u float64) momentinput.Profile {
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.LineSeg{End: sectionrecord.Point2{U: u}, TEnd: 1}}}}
	}
	view := cupwall.View{Outer: loop(10), Cavity: loop(6), Frame: frame, Xform: r3.Identity(),
		ZOpen: 20, ZOuter: 0, ZCav: 5, OffsetDelta: 0.125, Sense: cupwall.Inward}

	outer, cavity := view.MassRecords()
	wantOuter, wantCavity := view.OuterPrism(), view.CavityPrism()
	wantOuter.Profile, wantCavity.Profile = view.Outer, view.Cavity
	require.Equal(t, wantOuter.MassRecord(), outer)
	require.Equal(t, wantCavity.MassRecord(), cavity)
	require.Equal(t, view.Outer, outer.Profile)
	require.Equal(t, view.Cavity, cavity.Profile)
	require.Zero(t, outer.SectionDelta, "inward, only the cavity is the offset region")
	require.Equal(t, 0.125, cavity.SectionDelta)

	view.Sense = cupwall.Outward
	outer, cavity = view.MassRecords()
	require.Equal(t, 0.125, outer.SectionDelta, "outward, the outer region is the offset one")
	require.Zero(t, cavity.SectionDelta)
}
