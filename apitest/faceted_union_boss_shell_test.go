package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFacetedUnionNestedBossShellVerified(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "plate first", true: "boss first"}[reverse], func(t *testing.T) {
			doc := decad.New()
			plate := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 4)
			boss := boxBodyAtZ(t, doc, -4, -3, 4, 3, 3, 5)
			turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 0.5}, r3.Vec{Z: 1}, units.Degrees(20))
			require.NoError(t, err)
			boss, err = decad.Transform(t.Context(), boss, turn)
			require.NoError(t, err)
			first, second := plate, boss
			if reverse {
				first, second = second, first
			}
			joined, err := decad.Union(t.Context(), first, second)
			require.NoError(t, err)
			shell, err := decad.Shell(t.Context(), joined,
				decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).Exactly(1), units.Millimeters(1))
			require.NoError(t, err)
			volume, err := shell.Volume()
			require.NoError(t, err)
			want := 1792.0 - (648 + 96 + 80.0/3 + 5*math.Pi)
			require.LessOrEqual(t, math.Abs(volume.Value.Mag()-want), volume.Bound.Mag())
			mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Less(t, mesh.Bound().Mag(), 0.1)
			if !reverse {
				move, err := r3.Translation(r3.Vec{X: 7, Y: -3, Z: 2})
				require.NoError(t, err)
				placed, err := decad.Transform(t.Context(), shell, move)
				require.NoError(t, err)
				placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, placedMesh.BoundaryVerified())
				require.True(t, placedMesh.VolumeVerified())
			}
		})
	}
}

func TestFacetedUnionNestedBossShellRefusesInsufficientClearance(t *testing.T) {
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 4)
	boss := boxBodyAtZ(t, doc, -9, -3, -1, 3, 3, 5)
	turn, err := r3.RotationAround(r3.Vec{X: -5}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	boss, err = decad.Transform(t.Context(), boss, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	_, err = decad.Shell(t.Context(), joined,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).Exactly(1), units.Millimeters(1))
	require.True(t, errors.Is(err, decad.ErrUnsupported), "Shell returned %v", err)
	_, err = joined.Volume()
	require.NoError(t, err)
}

func TestFacetedUnionNestedBossShellRefusesPlacedSourceRelation(t *testing.T) {
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 4)
	boss := boxBodyAtZ(t, doc, -4, -3, 4, 3, 3, 5)
	turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 0.5}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	boss, err = decad.Transform(t.Context(), boss, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 2})
	require.NoError(t, err)
	joined, err = decad.Transform(t.Context(), joined, move)
	require.NoError(t, err)
	_, err = decad.Shell(t.Context(), joined,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).Exactly(1), units.Millimeters(1))
	require.True(t, errors.Is(err, decad.ErrUnsupported), "Shell returned %v", err)
	_, err = joined.Volume()
	require.NoError(t, err)
}
