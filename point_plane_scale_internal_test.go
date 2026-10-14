package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPointPlaneScaleCarriesSourceThroughLaterPlacement(t *testing.T) {
	doc := New()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	profileSketch, err := world.CreateSketch(plane)
	require.NoError(t, err)
	corners := [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
	points := make([]*sketch.Point, len(corners))
	for i, corner := range corners {
		points[i] = profileSketch.CreatePoint(corner[0], corner[1])
		profileSketch.Fix(points[i])
	}
	for i := range points {
		profileSketch.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = profileSketch.Solve(t.Context())
	require.NoError(t, err)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, profileSketch, profileSketch.Profiles()[0])
	require.NoError(t, err)
	sheetSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	a, b := sheetSketch.CreatePoint(5, -30), sheetSketch.CreatePoint(5, 30)
	sheetSketch.Fix(a)
	sheetSketch.Fix(b)
	sheetSketch.CreateLine(a, b)
	_, err = sheetSketch.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := Extrude(t.Context(), doc, sheetSketch, sheetSketch.Chains()[0], TwoSided{
		One: DistanceSide{D: units.Millimeters(30)},
		Two: DistanceSide{D: units.Millimeters(30)},
	})
	require.NoError(t, err)
	pieces, err := Split(t.Context(), tooth, sheet)
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(0.2))
	require.NoError(t, err)
	placed, err := Copy(t.Context(), pieces[1], WithCopyPlacement(turn))
	require.NoError(t, err)
	placedPayload := placed.n.Payload.(facetedPayload)
	require.NotNil(t, placedPayload.pointPlane)
	require.False(t, placedPayload.pointPlaneSourceEligible)
	scaled, err := ScaledAt(t.Context(), placed, r3.NewVec(5, 0, 0), units.Scalar(0.9))
	require.NoError(t, err)
	scaledPayload := scaled.n.Payload.(facetedPayload)
	require.Same(t, placedPayload.pointPlane, scaledPayload.pointPlane)
	require.False(t, scaledPayload.pointPlaneSourceEligible)
	require.NotNil(t, scaledPayload.pointSimilarity)
	require.Same(t, placedPayload.pointPlane, scaledPayload.pointSimilarity.slice)
	require.Len(t, scaledPayload.pointSimilarity.steps, 2)
	require.Equal(t, turn, scaledPayload.pointSimilarity.steps[0].motion)
	require.Equal(t, 0.9, scaledPayload.pointSimilarity.steps[1].factor)
	require.Greater(t, scaledPayload.pointPlaneCapDisplacement, 0.0)
	_, _, err = scaled.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	placedCopy, err := Copy(t.Context(), scaled, WithCopyPlacement(r3.Identity()))
	require.NoError(t, err)
	copyPayload := placedCopy.n.Payload.(facetedPayload)
	require.Same(t, scaledPayload.pointPlane, copyPayload.pointPlane)
	require.Len(t, copyPayload.pointSimilarity.steps, 3)
	require.Len(t, scaledPayload.pointSimilarity.steps, 2)
	require.False(t, copyPayload.pointPlaneSourceEligible)
}
