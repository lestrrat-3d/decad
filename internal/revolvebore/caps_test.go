package revolvebore_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCapProfilesCertifiesNestedSection(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	tool := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: 32,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(2), CCW: true, TEnd: 1},
		}}}}
	profiles, ok, err := revolvebore.CapProfiles(t.Context(), revolvebore.Meridian{LowRadius: 10, HighRadius: 8}, tool)
	require.NoError(t, err)
	require.True(t, ok)
	for _, profile := range profiles {
		require.Len(t, profile.Outer.Segments, 1)
		require.Len(t, profile.Holes, 1)
		require.Len(t, profile.Holes[0].Segments, 1)
	}
}
