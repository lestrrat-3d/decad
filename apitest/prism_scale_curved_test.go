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

func circularScaleSource(t *testing.T, quarter bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	if quarter {
		px := s.CreatePoint(3, 0)
		py := s.CreatePoint(0, 3)
		s.CreateLine(o, px)
		s.CreateLine(py, o)
		s.CreateArc(o, px, py)
	} else {
		s.CreateCircle(o, 3)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

func TestCircularPrismScaledAtKeepsMeasuredProofAndSourceRoles(t *testing.T) {
	for _, tc := range []struct {
		name    string
		quarter bool
		volume  float64
		area    float64
		center  r3.Vec
	}{
		{"circle", false, 18 * math.Pi, 30 * math.Pi, r3.NewVec(0, 0, 1)},
		{"quarter disk", true, 4.5 * math.Pi, 12 + 7.5*math.Pi, r3.NewVec(4/math.Pi, 4/math.Pi, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			source := circularScaleSource(t, tc.quarter)
			anchor := r3.NewVec(1, 2, 3)
			factor := 1.5
			scaled, err := source.ScaledAt(t.Context(), anchor, units.Scalar(factor))
			require.NoError(t, err)
			mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.LessOrEqual(t, mesh.Bound().Base(), 0.1)

			volume, err := scaled.Volume()
			require.NoError(t, err)
			require.InDelta(t, tc.volume*factor*factor*factor, volume.Value.Base(), volume.Bound.Base())
			area, err := scaled.Area()
			require.NoError(t, err)
			require.InDelta(t, tc.area*factor*factor, area.Value.Base(), area.Bound.Base())
			center, err := scaled.Centroid()
			require.NoError(t, err)
			want := anchor.Add(tc.center.Sub(anchor).Scale(factor))
			require.LessOrEqual(t, want.Sub(center.Value).Len(), center.Bound.Base())

			for _, role := range []decad.FeatureRef{decad.CapStart(source), decad.CapEnd(source)} {
				faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
				require.NoError(t, err)
				require.Len(t, faces, 1)
				require.IsType(t, decad.Plane{}, faces[0].Surface())
			}
			for _, face := range source.Faces() {
				if _, ok := face.Surface().(decad.Cylinder); !ok {
					continue
				}
				require.NotEmpty(t, face.Origins())
				faces, err := decad.Faces(decad.FaceCreatedBy(face.Origins()[0])).SelectFaces(scaled)
				require.NoError(t, err)
				require.Len(t, faces, 1)
				require.IsType(t, decad.Faceted{}, faces[0].Surface())
			}
		})
	}
}

func TestCircularPrismScaledAtAcceptsPlacedAndRepeatedImage(t *testing.T) {
	source := circularScaleSource(t, false)
	turn, err := r3.RotationAround(r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0), units.Radians(0.2))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, turn)
	require.NoError(t, err)
	first, err := placed.ScaledAt(t.Context(), r3.NewVec(1, 2, 3), units.Scalar(1.25))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(2, -1, 0.5))
	require.NoError(t, err)
	placedCopy, err := decad.Copy(t.Context(), first, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	again, err := placedCopy.ScaledAt(t.Context(), r3.NewVec(0, 0, 0), units.Scalar(0.8))
	require.NoError(t, err)
	mesh, err := again.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := again.Volume()
	require.NoError(t, err)
	require.InDelta(t, 18*math.Pi, volume.Value.Base(), volume.Bound.Base())
}
