package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExtrudeFreeformReflectedCopyPreservesGeometry(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	s.CreateLine(end, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	doc := decad.New()
	base, err := decad.Extrude(t.Context(), doc, s, profiles[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	image, err := decad.Copy(t.Context(), base, decad.WithCopyPlacement(reflection))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2)

	baseVolume, err := base.Volume()
	require.NoError(t, err)
	imageVolume, err := image.Volume()
	require.NoError(t, err)
	require.InDelta(t, 150.0, baseVolume.Value.Mag(), 1e-9)
	require.InDelta(t, baseVolume.Value.Mag(), imageVolume.Value.Mag(), 1e-9)

	baseArea, err := base.Area()
	require.NoError(t, err)
	imageArea, err := image.Area()
	require.NoError(t, err)
	require.InDelta(t, baseArea.Value.Mag(), imageArea.Value.Mag(), 1e-9)

	baseCentroid, err := base.Centroid()
	require.NoError(t, err)
	imageCentroid, err := image.Centroid()
	require.NoError(t, err)
	wantCentroid := reflection.Apply(baseCentroid.Value)
	require.InDelta(t, wantCentroid.X, imageCentroid.Value.X, 1e-9)
	require.InDelta(t, wantCentroid.Y, imageCentroid.Value.Y, 1e-9)
	require.InDelta(t, wantCentroid.Z, imageCentroid.Value.Z, 1e-9)

	baseBox, err := base.Bounds()
	require.NoError(t, err)
	imageBox, err := image.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -baseBox.Max.X, imageBox.Min.X, 1e-9)
	require.InDelta(t, -baseBox.Min.X, imageBox.Max.X, 1e-9)
	require.InDelta(t, baseBox.Min.Y, imageBox.Min.Y, 1e-9)
	require.InDelta(t, baseBox.Max.Y, imageBox.Max.Y, 1e-9)

	nurbsFaces := 0
	for _, face := range image.Faces() {
		if _, ok := face.Surface().(decad.NURBSSurface); ok {
			nurbsFaces++
		}
	}
	require.Equal(t, 1, nurbsFaces)
}
