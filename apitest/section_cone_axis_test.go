package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sectionConeSource(t *testing.T, height float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	base := s.CreatePoint(0, 0)
	rim := s.CreatePoint(5, 0)
	apex := s.CreatePoint(0, height)
	s.Fix(base)
	s.CreateLine(base, rim)
	s.CreateLine(rim, apex)
	s.CreateLine(apex, base)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{}, End: decad.Point2{V: 1},
	}, decad.FullRevolution{})
	require.NoError(t, err)
	return doc, body
}

func sectionConePlane(t *testing.T, z float64) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

func TestSectionCardinalConeFeedsVerifiedExtrude(t *testing.T) {
	doc, cone := sectionConeSource(t, 10)
	section, profiles, err := cone.Section(t.Context(), sectionConePlane(t, 5))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-6.25*math.Pi), area.Bound.Base())
	require.Less(t, area.Bound.Base(), 1e-9)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-12.5*math.Pi), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 1e-9)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, []*decad.Body{cone}, doc.Bodies())
}

func TestSectionCardinalConeInexactRadiusFeedsVerifiedExtrude(t *testing.T) {
	doc, cone := sectionConeSource(t, 3)
	section, profiles, err := cone.Section(t.Context(), sectionConePlane(t, 1))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-200*math.Pi/9), volume.Bound.Base())
	require.Greater(t, volume.Bound.Base(), 0.0)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, []*decad.Body{cone}, doc.Bodies())
}

func TestSectionCardinalConePlacementAndAxisSense(t *testing.T) {
	doc, cone := sectionConeSource(t, 10)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{X: 1},
	}, r3.NewVec(20, 30, 40))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), cone, motion)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(25, 27, 38), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	box, err := child.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(25, 27.5, 37.5), box.Min)
	require.Equal(t, r3.NewVec(27, 32.5, 42.5), box.Max)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())

	reversed, err := r3.NewFrame(r3.NewVec(25, 27, 38), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	_, reverseProfiles, err := placed.Section(t.Context(), reversed)
	require.NoError(t, err)
	require.Len(t, reverseProfiles, 1)
}

func TestSectionCardinalConeRefusesUnprovedCuts(t *testing.T) {
	doc, cone := sectionConeSource(t, 10)
	oblique, err := r3.NewFrame(r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 1))
	require.NoError(t, err)
	for _, tc := range []struct {
		name  string
		plane r3.Frame
	}{
		{name: "base", plane: sectionConePlane(t, 0)},
		{name: "apex", plane: sectionConePlane(t, 10)},
		{name: "exterior", plane: sectionConePlane(t, 11)},
		{name: "oblique", plane: oblique},
	} {
		t.Run(tc.name, func(t *testing.T) {
			section, profiles, err := cone.Section(t.Context(), tc.plane)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
			require.Equal(t, []*decad.Body{cone}, doc.Bodies())
		})
	}
}
