package apitest_test

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pointSlabCap(t *testing.T, slab *decad.Body, role decad.FeatureRef) *decad.Face {
	t.Helper()
	faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(slab)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	return faces[0]
}

func TestLoftFacesEightSourceSlabsKeepVolumeAndRejectBadCapOrder(t *testing.T) {
	doc := decad.New()
	tooth := pointLoftRadialTab(t, doc)
	span := pointLoftSliceSpan()
	inner := tooth
	kept := make([]*decad.Body, 0, 8)
	for k := 1; k <= 8; k++ {
		sheet := pointLoftSliceSheet(t, doc, span*float64(k)/6, 30)
		parts, err := decad.Split(t.Context(), inner, sheet)
		require.NoError(t, err)
		require.Len(t, parts, 2)
		inner = parts[0]
		kept = append(kept, parts[1])
	}
	sections := make([]*decad.Face, 0, 9)
	sections = append(sections, pointSlabCap(t, kept[7], decad.CapStart(kept[7])))
	for _, slab := range slices.Backward(kept) {
		sections = append(sections, pointSlabCap(t, slab, decad.CapEnd(slab)))
	}
	before := doc.Bodies()
	_, err := decad.LoftFaces(t.Context(), doc, sections[:1]...)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Bodies())
	duplicate := append([]*decad.Face(nil), sections...)
	duplicate[2] = duplicate[1]
	_, err = decad.LoftFaces(t.Context(), doc, duplicate...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	reordered := append([]*decad.Face(nil), sections...)
	reordered[3], reordered[4] = reordered[4], reordered[3]
	_, err = decad.LoftFaces(t.Context(), doc, reordered...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	start := pointSlabCap(t, kept[0], decad.CapStart(kept[0]))
	var side *decad.Face
	for _, face := range kept[0].Faces() {
		if face != sections[len(sections)-1] && face != start {
			side = face
			break
		}
	}
	require.NotNil(t, side)
	withSide := append([]*decad.Face(nil), sections...)
	withSide[len(withSide)-1] = side
	_, err = decad.LoftFaces(t.Context(), doc, withSide...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = decad.LoftFaces(canceled, doc, sections...)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, doc.Bodies())

	loft, err := decad.LoftFaces(t.Context(), doc, sections...)
	require.NoError(t, err)
	mesh, err := loft.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Positive(t, mesh.Bound().Base())
	volume, err := loft.Volume()
	require.NoError(t, err)
	sum, allowance := 0.0, volume.Bound.Base()
	for _, slab := range kept {
		reading, err := slab.Volume()
		require.NoError(t, err)
		sum += reading.Value.Base()
		allowance += reading.Bound.Base()
	}
	require.InDelta(t, sum, volume.Value.Base(), allowance)
	const toeApex = 5.3
	cut, err := decad.Cut(t.Context(), loft, pointConeTool(t, doc, toeApex))
	require.NoError(t, err)
	cutVolume, err := cut.Volume()
	require.NoError(t, err)
	require.Less(t, cutVolume.Value.Base(), volume.Value.Base())
	cutMesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, cutMesh.BoundaryVerified())
	require.True(t, cutMesh.VolumeVerified())
	wrongHeel := pointConeTool(t, doc, math.Nextafter(8, math.Inf(1)))
	beforeWrong := doc.Bodies()
	_, err = decad.Intersect(t.Context(), cut, wrongHeel)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, beforeWrong, doc.Bodies())
	heel, err := decad.Intersect(t.Context(), cut, pointConeTool(t, doc, 8))
	require.NoError(t, err)
	heelVolume, err := heel.Volume()
	require.NoError(t, err)
	require.Less(t, heelVolume.Value.Base(), cutVolume.Value.Base())
	heelMesh, err := heel.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, heelMesh.BoundaryVerified())
	require.True(t, heelMesh.VolumeVerified())
}
