package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFullRevolveScaledAtPreservesSphereVolumeAndSourceFace(t *testing.T) {
	s, profile := semicircleSketch(t)
	doc := decad.New()
	sphere, err := decad.Revolve(t.Context(), doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	require.Len(t, sphere.Faces(), 1)
	role := sphere.Faces()[0].Origins()[0]
	anchor := r3.NewVec(1, 2, 3)
	scaled, err := decad.ScaledAt(t.Context(), sphere, anchor, units.Scalar(2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{scaled}, doc.Bodies())

	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8*4*math.Pi*125/3, volume.Value.Base(), volume.Bound.Base())
	centroid, err := scaled.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, r3.NewVec(9, -2, -3).Sub(centroid.Value).Len(), centroid.Bound.Base())
	faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	require.IsType(t, decad.Faceted{}, faces[0].Surface())
	require.Empty(t, faces[0].Loops())
	require.Empty(t, scaled.Edges())
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.NotEqual(t, decad.Unsound, report.Status)
}

func TestFullRevolveScaledAtPreservesPlanarFaceRoles(t *testing.T) {
	s, profile := solidSketch(t)
	doc := decad.New()
	cylinder, err := decad.Revolve(t.Context(), doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	scaled, err := decad.ScaledAt(t.Context(), cylinder, r3.NewVec(1, 2, 3), units.Scalar(1.25))
	require.NoError(t, err)
	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1250*math.Pi, volume.Value.Base(), volume.Bound.Base())
	for _, face := range cylinder.Faces() {
		role := face.Origins()[0]
		images, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
		require.NoError(t, err)
		require.Len(t, images, 1)
		if _, planar := face.Surface().(decad.Plane); planar {
			require.IsType(t, decad.Plane{}, images[0].Surface())
		} else {
			require.IsType(t, decad.Faceted{}, images[0].Surface())
		}
	}
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestPartialRevolveScaledAtPreservesSectorVolumeAndFaceRoles(t *testing.T) {
	s, profile := solidSketch(t)
	doc := decad.New()
	wedge, err := decad.Revolve(t.Context(), doc, s, profile, uAxis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	scaled, err := decad.ScaledAt(t.Context(), wedge, r3.NewVec(1, 2, 3), units.Scalar(2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{scaled}, doc.Bodies())

	// The source is a quarter of the cylinder of radius 8 and length 10, so
	// its volume is pi/4*8^2*10 = 160 pi. Scaling by 2 multiplies it by 8.
	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8*160*math.Pi, volume.Value.Base(), volume.Bound.Base())
	require.Greater(t, volume.Bound.Base(), 0.0)
	for _, face := range wedge.Faces() {
		images, err := decad.Faces(decad.FaceCreatedBy(face.Origins()[0])).SelectFaces(scaled)
		require.NoError(t, err)
		require.Len(t, images, 1)
		if _, planar := face.Surface().(decad.Plane); planar {
			require.IsType(t, decad.Plane{}, images[0].Surface())
		} else {
			require.IsType(t, decad.Faceted{}, images[0].Surface())
		}
	}
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.NotEqual(t, decad.Unsound, report.Status)
}

func TestScaledAtRefusesPartialSheetRevolveWithoutRetiringSource(t *testing.T) {
	s, profile := solidSketch(t)
	doc := decad.New()
	sheet, err := decad.Revolve(t.Context(), doc, s, profile, uAxis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = decad.ScaledAt(t.Context(), sheet, r3.Vec{}, units.Scalar(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{sheet}, doc.Bodies())
}

func TestFullRevolveScaledAtRepeatsAfterRigidPlacement(t *testing.T) {
	s, profile := semicircleSketch(t)
	doc := decad.New()
	sphere, err := decad.Revolve(t.Context(), doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	motion, err := r3.Translation(r3.NewVec(2, -3, 1))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), sphere, motion)
	require.NoError(t, err)
	first, err := decad.ScaledAt(t.Context(), placed, r3.Vec{}, units.Scalar(2))
	require.NoError(t, err)
	again, err := decad.ScaledAt(t.Context(), first, r3.NewVec(1, 2, 3), units.Scalar(0.5))
	require.NoError(t, err)
	volume, err := again.Volume()
	require.NoError(t, err)
	require.InDelta(t, 4*math.Pi*125/3, volume.Value.Base(), volume.Bound.Base())
	mesh, err := again.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestScaledAtRefusesSheetRevolveWithoutRetiringSource(t *testing.T) {
	s, profile := solidSketch(t)
	doc := decad.New()
	sheet, err := decad.Revolve(t.Context(), doc, s, profile, uAxis,
		decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = decad.ScaledAt(t.Context(), sheet, r3.Vec{}, units.Scalar(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{sheet}, doc.Bodies())
}
