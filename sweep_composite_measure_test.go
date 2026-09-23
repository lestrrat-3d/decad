package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepCompositeArcLineArcMeasurementsReplay(t *testing.T) {
	t.Parallel()

	sketch, profile := orthogonalSweepProfile(t)
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{
			Through: r3.NewVec(2, 0, 4),
			End:     r3.NewVec(5, 0, 5),
		},
		decad.LineTo{End: r3.NewVec(8, 0, 5)},
		decad.ArcThrough{
			Through: r3.NewVec(12, 2, 5),
			End:     r3.NewVec(13, 5, 5),
		},
	)
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Sweep(t.Context(), sketch, profile, path)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, body.Lumps(), 1)
	require.Len(t, body.Shells(), 1)
	require.Len(t, body.Faces(), 14)
	require.Len(t, body.Edges(), 28)
	require.Len(t, body.Vertices(), 16)
	for _, edge := range body.Edges() {
		require.Len(t, edge.Faces(), 2)
	}

	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(12+20*math.Pi))
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(32+40*math.Pi))
	decadtest.MeasuresBounds(t, body, r3.NewVec(-1, -1, 0), r3.NewVec(14, 5, 6))
	centroid, err := body.Centroid()
	require.NoError(t, err)

	move, err := r3.Translation(r3.NewVec(20, -10, 3))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, placed, units.CubicMillimeters(12+20*math.Pi))
	decadtest.MeasuresArea(t, placed, units.SquareMillimeters(32+40*math.Pi))
	decadtest.MeasuresCentroid(t, placed, move.Apply(centroid.Value))
	decadtest.MeasuresBounds(t, placed, r3.NewVec(19, -11, 3), r3.NewVec(34, -5, 9))

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestSweepCompositePlacedCopyKeepsSourceReplay(t *testing.T) {
	t.Parallel()

	sketch, profile, path, _, _ := orthogonalSweepFixture(t)
	doc := decad.New()
	source, err := doc.Sweep(t.Context(), sketch, profile, path)
	require.NoError(t, err)
	sourceCentroid, err := source.Centroid()
	require.NoError(t, err)

	move, err := r3.Translation(r3.NewVec(20, -10, 3))
	require.NoError(t, err)
	placedCopy, err := source.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	decadtest.MeasuresCentroid(t, placedCopy, move.Apply(sourceCentroid.Value))

	replayedSource, err := source.Duplicate(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresCentroid(t, replayedSource, sourceCentroid.Value)
	decadtest.MeasuresBounds(t, replayedSource, r3.NewVec(-1, -1, 0), r3.NewVec(21, 5, 6))
}

func TestSweepCompositeSpanBudgetRejectsBeforeCommit(t *testing.T) {
	t.Parallel()

	sketch, profile := orthogonalSweepProfile(t)
	segments := make([]decad.PathSegment, 257)
	for i := range segments {
		segments[i] = decad.LineTo{End: r3.NewVec(0, 0, float64(i+1))}
	}
	path, err := decad.NewPath(r3.Vec{}, segments...)
	require.NoError(t, err)

	doc := decad.New()
	_, err = doc.Sweep(t.Context(), sketch, profile, path)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}
