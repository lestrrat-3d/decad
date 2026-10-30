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
			// Plate 1600 and boss 192 above it, less the 18x18x2 lower cavity,
			// the 6x4x4 upper cavity and the 8x6 root band
			// wht - (w+h)πt²/2 + 8t³/3 = 152/3 - 7π.
			want := 1792.0 - (648 + 96 + 152.0/3 - 7*math.Pi)
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

// An inward shell of the faceted plate-and-boss Union keeps its wall thickness
// t = 1 at the concave boss root. The cavity stays at distance t from the root
// edge, the boss outline at the plate top z = 4, so in the boss's local frame
// a point 0.6 mm inside a side and 0.6 mm below the root, 0.85 mm from the
// edge, is material. A point 0.8 mm inside and below, 1.13 mm from the edge,
// is cavity. Near a root corner the cavity's sharp mitre puts the same
// readings on the diagonal.
//
// LocatePoint leaves these points undecided, so the probes read the verified
// mesh by ray parity. Each lies at least 0.13 mm from the analytic surface,
// and the 16-station root band departs from it by under 0.01 mm.
func TestFacetedUnionNestedBossShellKeepsThicknessAtRoot(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 4)
	boss := boxBodyAtZ(t, doc, -4, -3, 4, 3, 3, 5)
	turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 0.5}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	boss, err = decad.Transform(t.Context(), boss, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	shell, err := decad.Shell(t.Context(), joined,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	for _, p := range []r3.Vec{{X: 3.4, Z: 3.4}, {Y: 2.4, Z: 3.4}, {X: 3.4, Y: 2.4, Z: 3.4}} {
		require.True(t, meshEncloses(mesh, turn.Apply(p)), "local %v lies 0.85 mm from the root edge and is material", p)
	}
	for _, p := range []r3.Vec{{X: 3.2, Z: 3.2}, {Y: 2.2, Z: 3.2}, {X: 3.2, Y: 2.2, Z: 3.2}} {
		require.False(t, meshEncloses(mesh, turn.Apply(p)), "local %v lies 1.13 mm from the root edge and is cavity", p)
	}
	// The thinnest wall is t everywhere, the root included. The wall survey's
	// rays only bound the minimum from above, so a 1 mm tool stays undecided;
	// a root wall thinner than t would prove the wall too thin.
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br, err := report.ForBody(shell)
	require.NoError(t, err)
	require.NotEqual(t, decad.AssessmentViolated, br.Wall.Assessment, "the root wall is not below t")
	require.NotNil(t, br.Wall.Minimum)
	require.GreaterOrEqual(t, br.Wall.Minimum.Value.Base()+br.Wall.Minimum.Bound.Base(), 1.0,
		"the wall reading's upper end covers t")
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
