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

// embeddingReachSource has two narrow notches beside its left wall. At the
// translated placement their distinct X coordinates share float-box corners;
// the accepted repair needs positions beyond those corners to keep it embedded.
func embeddingReachSource(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	coords := [][2]float64{
		{0, 0}, {10, 0}, {10, 10}, {0.004, 10},
		{0.004, 9}, {0.003, 9}, {0.003, 10}, {0.002, 10},
		{0.002, 9}, {0.001, 9}, {0.001, 10}, {0, 10},
	}
	points := make([]*sketch.Point, len(coords))
	for i, p := range coords {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	solid, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	// Public similarity construction supplies the held mesh that placement
	// repairs; no private payload or handwritten warning enters the fixture.
	held, err := decad.ScaledAt(t.Context(), solid, r3.Vec{}, units.Scalar(1))
	require.NoError(t, err)
	require.Empty(t, held.OperationDiagnostics())
	return held
}

func TestCopyReportsAcceptedEmbeddingReach(t *testing.T) {
	t.Parallel()
	move, err := r3.Translation(r3.Vec{X: 1e13})
	require.NoError(t, err)

	t.Run("accepted repair publishes its actual change", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.WithGapTolerance(units.Millimeters(1)))
		source := embeddingReachSource(t, doc)
		image, err := decad.Copy(t.Context(), source, decad.WithCopyPlacement(move))
		require.NoError(t, err)
		require.True(t, image.IsSolid())
		require.Len(t, image.Lumps(), 1)
		require.Len(t, doc.Bodies(), 2)

		warnings := image.OperationDiagnostics()
		require.Len(t, warnings, 1)
		warning := warnings[0]
		require.Equal(t, decad.OpDiagEmbeddingReachApplied, warning.Code)
		require.Equal(t, decad.OperationDiagnosticWarning, warning.Severity)
		require.Equal(t, decad.OperationCopy, warning.Operation)
		require.Equal(t, decad.OperationStageResult, warning.Stage)
		require.Nil(t, warning.Subject.Body)
		require.NotNil(t, warning.Change)
		require.Equal(t, decad.OperationChangeEmbeddingReach, warning.Change.Mechanism)
		require.Equal(t, units.Length, warning.Change.DisplacementBound.Kind())
		require.Positive(t, warning.Change.DisplacementBound.Base())
		require.Equal(t, units.Millimeters(1), warning.Change.AllowedReach)
		require.LessOrEqual(t, warning.Change.DisplacementBound.Base(), warning.Change.AllowedReach.Base())

		volume, err := image.Volume()
		require.NoError(t, err)
		// The two 0.001×1 mm notches remove 0.02 mm³ from the 10 mm extrusion.
		const wantVolume = 999.98
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base())
		mesh, err := image.Tessellate(t.Context(), units.Millimeters(1),
			decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("float-box search alone refuses the same placement", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.ExactOnly())
		source := embeddingReachSource(t, doc)
		image, err := decad.Copy(t.Context(), source, decad.WithCopyPlacement(move))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Nil(t, image)
		live := doc.Bodies()
		require.Len(t, live, 1)
		require.True(t, live[0] == source)
		require.Empty(t, source.OperationDiagnostics())
	})
}
