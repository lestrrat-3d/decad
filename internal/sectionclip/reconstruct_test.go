package sectionclip_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestSectionBrepStackGates checks each refusal of a slab stack, the gates a
// stacked prism with a displaced profile or level cannot be built to reach
// through the public API.
func TestSectionBrepStackGates(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := r3.NewFrame(r3.Vec{Z: 5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	proxy := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 10}
	region := momentinput.Profile{}
	for name, tc := range map[string]struct {
		proxy prismrecord.Payload
		slabs []stackedrecord.Slab
		want  string
	}{
		"displaced profile": {
			proxy: func() prismrecord.Payload { p := proxy; p.SectionDelta = 0.25; return p }(),
			slabs: []stackedrecord.Slab{{Regions: []momentinput.Profile{region}, Z0: 0, Z1: 10}},
			want:  "displaced source profile",
		},
		"displaced level": {
			proxy: proxy,
			slabs: []stackedrecord.Slab{{Regions: []momentinput.Profile{region}, Z0: 0, Z1: 10, Z1Delta: 0.1}},
			want:  "one exact slab region",
		},
		"two regions": {
			proxy: proxy,
			slabs: []stackedrecord.Slab{{Regions: []momentinput.Profile{region, region}, Z0: 0, Z1: 10}},
			want:  "one exact slab region",
		},
		"interface level": {
			proxy: proxy,
			slabs: []stackedrecord.Slab{{Regions: []momentinput.Profile{region}, Z0: 0, Z1: 5},
				{Regions: []momentinput.Profile{region}, Z0: 5, Z1: 10}},
			want: "outside slab stack interiors",
		},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			_, _, err := sectionclip.SectionBrepStack(t.Context(), tc.proxy, tc.slabs, plane)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
