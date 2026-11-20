package loftrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDirectSectionsBindBothNativeLoops(t *testing.T) {
	b := new(topo.Body)
	profile := func() momentinput.Profile {
		return momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{momentinput.LineSeg{End: momentinput.Point2{U: 1}, TEnd: 1}}}}
	}
	p := Payload{Profile0: profile(), Profile1: profile(), AuthenticatedReconstruction: true, Xform: r3.Identity(), Alignment: []int{0}}
	p.SourceIdentity = &SourceIdentity{direct: sealDirectSections(b, p)}
	require.True(t, p.MatchesDirectSections(b))
	require.False(t, p.MatchesDirectSections(new(topo.Body)))
	p.Alignment[0] = 1
	require.False(t, p.MatchesDirectSections(b))
	p.Alignment[0] = 0
	p.Profile1.Outer.Segments[0] = momentinput.LineSeg{End: momentinput.Point2{U: 2}, TEnd: 1}
	require.False(t, p.MatchesDirectSections(b))
}
