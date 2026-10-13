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

func TestSectionCardinalSphereNonSquareFeedsVerifiedExtrude(t *testing.T) {
	sphere := sectionSphereSource(t)
	section, profiles, err := sphere.Section(t.Context(), sectionSpherePlane(t, 2))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-42*math.Pi), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestSectionObliqueCenterSphereFeedsVerifiedExtrude(t *testing.T) {
	sphere := sectionSphereSource(t)
	frame, err := r3.NewFrame(r3.Vec{X: 5}, r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1})
	require.NoError(t, err)
	section, profiles, err := sphere.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-50*math.Pi), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestSectionObliqueOffsetSphereFeedsVerifiedExtrude(t *testing.T) {
	sphere := sectionSphereSource(t)
	frame, err := r3.NewFrame(r3.NewVec(5, 1.8, 2.4),
		r3.NewVec(1, 0, 0), r3.NewVec(0, 0.8, -0.6))
	require.NoError(t, err)
	section, profiles, err := sphere.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-32*math.Pi), volume.Bound.Base())
	capFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(child))).Exactly(1).SelectFaces(child)
	require.NoError(t, err)
	area, err := capFaces[0].Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-16*math.Pi), area.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueOffsetSpherePlacement(t *testing.T) {
	sphere := sectionSphereSource(t)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.NewVec(7, 11, 13))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), sphere, motion)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(7, 17.8, 15.4),
		r3.NewVec(-1, 0, 0), r3.NewVec(0, 0.8, -0.6))
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-32*math.Pi), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueOffsetSphereHeldGramCrossTerm(t *testing.T) {
	sphere := sectionSphereSource(t)
	frame, err := r3.NewFrame(r3.NewVec(5, 0, 2),
		r3.NewVec(1, 2, 3), r3.NewVec(2, -1, 4))
	require.NoError(t, err)
	require.NotZero(t, frame.U().Dot(frame.V()))
	section, profiles, err := sphere.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	// The requested axes span a plane whose squared center distance is 2/3 mm².
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-146*math.Pi/3), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueCenterSpherePlacementAndPlaneSense(t *testing.T) {
	sphere := sectionSphereSource(t)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 7, Y: 11, Z: 13})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), sphere, motion)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{X: 7, Y: 16, Z: 13},
		r3.Vec{X: -1}, r3.Vec{Y: -1, Z: -1})
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	require.Equal(t, frame, sectionFrame)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-50*math.Pi), volume.Bound.Base())
}

func TestSectionObliqueSphereShiftedOrigins(t *testing.T) {
	sphere := sectionSphereSource(t)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		volume float64
	}{
		{name: "same plane", origin: r3.Vec{}, volume: 50 * math.Pi},
		{name: "offset plane", origin: r3.Vec{X: 5, Y: 1}, volume: 49 * math.Pi},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1})
			require.NoError(t, err)
			section, profiles, err := sphere.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
				D: units.Millimeters(2), Dir: decad.Along,
			})
			require.NoError(t, err)
			volume, err := child.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.volume), volume.Bound.Base())
			mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestSectionObliqueSphereRefusesTangentExterior(t *testing.T) {
	sphere := sectionSphereSource(t)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
	}{
		{name: "tangent", origin: r3.NewVec(5, 3, 4)},
		{name: "exterior", origin: r3.NewVec(5, 3.6, 4.8)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 0.8, -0.6))
			require.NoError(t, err)
			section, profiles, err := sphere.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
			require.Equal(t, []*decad.Body{sphere}, sphere.Document().Bodies())
		})
	}
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

func TestSectionCardinalSphereNonSquarePlacementAndPlaneSense(t *testing.T) {
	sphere := sectionSphereSource(t)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 7, Y: 11, Z: 13})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), sphere, motion)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{X: 7, Y: 11, Z: 15}, r3.Vec{X: -1}, r3.Vec{Y: -1})
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	sectionFrame, err := section.Plane().Frame()
	require.NoError(t, err)
	require.Equal(t, frame, sectionFrame)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-42*math.Pi), volume.Bound.Base())
}

func TestSectionCardinalSphereRefusesUnprovedCuts(t *testing.T) {
	sphere := sectionSphereSource(t)
	for _, tc := range []struct {
		name  string
		plane r3.Frame
	}{
		{name: "tangent", plane: sectionSpherePlane(t, 5)},
		{name: "exterior", plane: sectionSpherePlane(t, 6)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			section, profiles, err := sphere.Section(t.Context(), tc.plane)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
}
