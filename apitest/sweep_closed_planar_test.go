package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func roundedRectangleSweepPath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(2, 0, 14), End: r3.NewVec(5, 0, 15)},
		decad.LineTo{End: r3.NewVec(15, 0, 15)},
		decad.ArcThrough{Through: r3.NewVec(19, 0, 13), End: r3.NewVec(20, 0, 10)},
		decad.LineTo{End: r3.NewVec(20, 0, 0)},
		decad.ArcThrough{Through: r3.NewVec(18, 0, -4), End: r3.NewVec(15, 0, -5)},
		decad.LineTo{End: r3.NewVec(5, 0, -5)},
		decad.ArcThrough{Through: r3.NewVec(1, 0, -3), End: r3.Vec{}},
	)
	require.NoError(t, err)
	return path
}

func TestSweepClosedPlanarTangentLoopBuildsVerifiedSolid(t *testing.T) {
	t.Parallel()
	const radius = 0.25
	section, profile := racetrackSection(t, 0, radius)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, roundedRectangleSweepPath(t))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Shells(), 1)
	require.Len(t, body.Faces(), 8)
	require.Len(t, body.Edges(), 8)
	for i, face := range body.Faces() {
		require.Equal(t, fmt.Sprintf("side(%d,0,0)", i), face.Origins()[0].Role)
		for _, edge := range face.Edges() {
			require.Len(t, edge.Faces(), 2)
		}
	}
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	length := 40 + 10*math.Pi
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, math.Pi*radius*radius*length, volume.Value.Base(), 1e-10+volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	require.InDelta(t, 2*math.Pi*radius*length, area.Value.Base(), 1e-10+area.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)

	translation, err := r3.Translation(r3.NewVec(3, 4, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, translation)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestSweepClosedPlanarTangentLoopRefusesAxisContact(t *testing.T) {
	t.Parallel()
	section, profile := racetrackSection(t, 0, 5)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, roundedRectangleSweepPath(t))
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}
