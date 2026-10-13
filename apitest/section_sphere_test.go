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

func sectionSphereSource(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	left := s.CreatePoint(0, 0)
	s.Fix(left)
	right := s.CreatePoint(10, 0)
	center := s.CreatePoint(5, 0)
	s.CreateLine(left, right)
	s.CreateArc(center, right, left)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func sectionSpherePlane(t *testing.T, z float64) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return frame
}

func TestSectionCardinalSphereFeedsVerifiedExtrude(t *testing.T) {
	sphere := sectionSphereSource(t)
	frame := sectionSpherePlane(t, 3)
	section, profiles, err := sphere.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	require.Equal(t, frame, sectionFrame)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-16*math.Pi), area.Bound.Base())
	require.Less(t, area.Bound.Base(), 1e-9)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-32*math.Pi), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 1e-9)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestSectionCardinalSphereCutsAcrossRevolveAxis(t *testing.T) {
	sphere := sectionSphereSource(t)
	frame, err := r3.NewFrame(r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	section, profiles, err := sphere.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-9*math.Pi), area.Bound.Base())
	require.Less(t, area.Bound.Base(), 1e-9)
}

func TestSectionCardinalSpherePlacementAndPlaneSense(t *testing.T) {
	sphere := sectionSphereSource(t)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 7, Y: 11, Z: 13})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), sphere, motion)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{X: 7, Y: 11, Z: 16}, r3.Vec{X: -1}, r3.Vec{Y: -1})
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-16*math.Pi), area.Bound.Base())
}

func TestSectionCardinalSphereRefusesUnprovedCuts(t *testing.T) {
	sphere := sectionSphereSource(t)
	oblique, err := r3.NewFrame(r3.Vec{Z: 3}, r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1})
	require.NoError(t, err)
	for _, tc := range []struct {
		name  string
		plane r3.Frame
	}{
		{name: "non-square radius", plane: sectionSpherePlane(t, 1)},
		{name: "tangent", plane: sectionSpherePlane(t, 5)},
		{name: "exterior", plane: sectionSpherePlane(t, 6)},
		{name: "oblique", plane: oblique},
	} {
		t.Run(tc.name, func(t *testing.T) {
			section, profiles, err := sphere.Section(t.Context(), tc.plane)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
}
