package classbgeom

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stepOf is one step of a stepped shaft along x: radius r over [z0, z1].
func stepOf(t *testing.T, r, z0, z1 float64) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Frame:   frame, Z0: z0, Z1: z1, Xform: r3.Identity(),
	}
}

func TestSteppedEnvelope(t *testing.T) {
	t.Parallel()

	steps := func() []prismrecord.Payload {
		return []prismrecord.Payload{stepOf(t, 8, 0, 10), stepOf(t, 10, 10, 20), stepOf(t, 6, 20, 35)}
	}
	got, ok := steppedEnvelope(steps())
	require.True(t, ok)
	require.Equal(t, 0.0, got.Z0)
	require.Equal(t, 35.0, got.Z1)
	require.Equal(t, steps()[1].Profile, got.Profile, `the envelope is the widest step's circle`)
	require.Equal(t, steps()[0].Frame, got.Frame)

	moved, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	// Each edit leaves a stack whose steps are not coaxial circular prisms
	// stacked end to start; each must miss.
	for name, edit := range map[string]func([]prismrecord.Payload) []prismrecord.Payload{
		"one step": func(s []prismrecord.Payload) []prismrecord.Payload { return s[:1] },
		"other frame": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].Frame = turned
			return s
		},
		"other placement": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[2].Xform = moved
			return s
		},
		"gap between steps": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].Z0 = 11
			return s
		},
		"empty span": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[2].Z1 = 20
			return s
		},
		"level displacement": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[0].Z1Delta = 1e-12
			return s
		},
		"start displacement": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].Z0Delta = 1e-12
			return s
		},
		"section displacement": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].SectionDelta = 1e-12
			return s
		},
		"sheet": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].SurfaceResult = true
			return s
		},
		"off-centre step": func(s []prismrecord.Payload) []prismrecord.Payload {
			c := s[2].Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
			c.Center = sectionrecord.Point2{U: 1}
			s[2].Profile.Outer.Segments[0] = c
			return s
		},
		"equal neighbouring radii": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[2] = stepOf(t, 10, 20, 35)
			return s
		},
		"clockwise circle": func(s []prismrecord.Payload) []prismrecord.Payload {
			c := s[0].Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
			c.CCW = false
			s[0].Profile.Outer.Segments[0] = c
			return s
		},
		"partial circle": func(s []prismrecord.Payload) []prismrecord.Payload {
			c := s[0].Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
			c.TEnd = 0.5
			s[0].Profile.Outer.Segments[0] = c
			return s
		},
		"holed step": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].Profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(2), CCW: false, TStart: 1, TEnd: 0}}}}
			return s
		},
		"line section": func(s []prismrecord.Payload) []prismrecord.Payload {
			s[1].Profile.Outer.Segments = []sectionrecord.CurveSegment{sectionrecord.LineSeg{
				Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 1}, TStart: 0, TEnd: 1}}
			return s
		},
	} {
		_, ok := steppedEnvelope(edit(steps()))
		require.False(t, ok, name)
	}
}
