package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func allPlanarBoolean(t *testing.T, scale float64) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10*scale, 10*scale, 10*scale)
	b := translated(t, boxBody(t, doc, 0, 0, 10*scale, 10*scale, 10*scale), 5*scale, 5*scale, 5*scale)
	body, err := decad.Union(a, b)
	require.NoError(t, err)
	return doc, body
}

func TestVerifyAllPlanarBooleanGatesApproximateArea(t *testing.T) {
	doc, body := allPlanarBoolean(t, 1)

	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, report.Bodies[0].Exactness)
	require.Equal(t, decad.Sound, report.Bodies[0].Status)
	require.True(t, report.Trustworthy())

	zero, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(0)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, zero.Bodies[0].Status)
	require.False(t, zero.Trustworthy())
}

func TestVerifyToleranceBoundaryIsInclusive(t *testing.T) {
	doc, body := allPlanarBoolean(t, 1)
	required := requiredBodyTolerance(t, body)
	require.Positive(t, required)

	atBoundary, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(required)))
	require.NoError(t, err)
	require.Equal(t, decad.Sound, atBoundary.Status)
	require.True(t, atBoundary.Trustworthy())

	below := math.Nextafter(required, 0)
	tooStrict, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(below)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, tooStrict.Status)
	require.False(t, tooStrict.Trustworthy())
}

func TestVerifyToleranceGateTracksScaleAndPlacement(t *testing.T) {
	for _, scale := range []float64{1, 1000} {
		t.Run(units.Scalar(scale).String(), func(t *testing.T) {
			doc, body := allPlanarBoolean(t, scale)
			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			translation, err := r3.Translation(r3.NewVec(1000*scale, -250*scale, 80*scale))
			require.NoError(t, err)
			placement, err := rotation.Then(translation)
			require.NoError(t, err)
			placed, err := body.Placed(placement)
			require.NoError(t, err)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Len(t, report.Bodies, 1)
			require.Same(t, placed, report.Bodies[0].Body)
			require.Equal(t, decad.Approximate, report.Bodies[0].Exactness)
			require.Equal(t, decad.Sound, report.Status)
			require.True(t, report.Trustworthy())
		})
	}
}

func TestVerifyExactBodyPassesZeroTolerance(t *testing.T) {
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(0)))
	require.NoError(t, err)
	require.Equal(t, decad.Exact, report.Bodies[0].Exactness)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
}

func requiredBodyTolerance(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	mesh, err := body.Tessellate(units.Millimeters(1000))
	require.NoError(t, err)
	diameter := diameterOf(mesh.Vertices())
	require.Positive(t, diameter)

	area, err := body.Area()
	require.NoError(t, err)
	edgeLength := 0.0
	for _, edge := range body.Edges() {
		length, err := edge.Length()
		require.NoError(t, err)
		edgeLength += length.Value.Base()
	}
	areaRef := math.Max(math.Abs(area.Value.Base()), 1e-9*diameter*edgeLength)
	required := area.Bound.Base() / areaRef

	bounds, err := body.Bounds()
	require.NoError(t, err)
	required = math.Max(required, bounds.Bound.Base()/diameter)

	volume, err := body.Volume()
	require.NoError(t, err)
	volumeRef := math.Max(math.Abs(volume.Value.Base()), 1e-9*diameter*math.Abs(area.Value.Base()))
	required = math.Max(required, volume.Bound.Base()/volumeRef)

	centroid, err := body.Centroid()
	require.NoError(t, err)
	required = math.Max(required, centroid.Bound.Base()/diameter)
	return required
}

func diameterOf(points []r3.Vec) float64 {
	best := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			best = math.Max(best, points[i].Sub(points[j]).Len())
		}
	}
	return best
}
