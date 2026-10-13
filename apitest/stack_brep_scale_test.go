package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStackBrepScaledAtVerified(t *testing.T) {
	for _, tc := range []struct {
		name           string
		bossX0, bossX1 float64
		source, scaled float64
	}{
		{name: "nested stack", bossX0: -5, bossX1: 5, source: 2500, scaled: 8437.5},
		{name: "flush BRep", bossX0: 5, bossX1: 10, source: 2250, scaled: 7593.75},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 5)
			boss := boxBodyAtZ(t, doc, tc.bossX0, -5, tc.bossX1, 5, 5, 5)
			joined, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			before, err := joined.Volume()
			require.NoError(t, err)
			require.Equal(t, tc.source, before.Value.Base())
			_, err = joined.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			var sourceRoles []decad.FeatureRef
			for _, face := range joined.Faces() {
				sourceRoles = append(sourceRoles, face.Origins()...)
			}

			scaled, err := joined.ScaledAt(t.Context(), r3.NewVec(2, -3, 4), units.Scalar(1.5))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{scaled}, doc.Bodies())
			volume, err := scaled.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.scaled), volume.Bound.Base())
			mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Less(t, mesh.Bound().Base(), 0.1)
			for _, role := range sourceRoles {
				faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
				require.NoError(t, err)
				require.NotEmpty(t, faces)
				for _, face := range faces {
					require.IsType(t, decad.Plane{}, face.Surface())
				}
			}
		})
	}
}

func TestStackBrepScaledAtPlacedAndRepeated(t *testing.T) {
	for _, tc := range []struct {
		name           string
		bossX0, bossX1 float64
		want           float64
	}{
		{name: "nested stack", bossX0: -5, bossX1: 5, want: 1054.6875},
		{name: "flush BRep", bossX0: 5, bossX1: 10, want: 949.21875},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 5)
			boss := boxBodyAtZ(t, doc, tc.bossX0, -5, tc.bossX1, 5, 5, 5)
			joined, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			move, err := r3.Translation(r3.NewVec(3, -2, 7))
			require.NoError(t, err)
			placed, err := decad.Transform(t.Context(), joined, move)
			require.NoError(t, err)
			first, err := placed.ScaledAt(t.Context(), r3.NewVec(2, -3, 4), units.Scalar(1.5))
			require.NoError(t, err)
			again, err := first.ScaledAt(t.Context(), r3.NewVec(-1, 2, 3), units.Scalar(0.5))
			require.NoError(t, err)
			volume, err := again.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.want), volume.Bound.Base())
			mesh, err := again.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestStackBrepScaledAtRefusesCurvedStack(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 5)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	_, err = joined.ScaledAt(t.Context(), r3.NewVec(2, -3, 4), units.Scalar(1.5))
	require.True(t, errors.Is(err, decad.ErrUnsupported), "ScaledAt returned %v", err)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
}
