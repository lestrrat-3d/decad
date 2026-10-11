package apitest_test

import (
	"context"
	"errors"
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPointLoftSplitAnchoredScalePreservesCapsAndVolume(t *testing.T) {
	doc := decad.New()
	tooth := bevelReferenceTooth(t, doc)
	span := pointBevelRootSpan()
	first := pointBevelSliceSheet(t, doc, span/6, 30)
	parts, err := doc.Split(t.Context(), tooth, first)
	require.NoError(t, err)
	require.Len(t, parts, 2)
	second := pointBevelSliceSheet(t, doc, span/3, 30)
	parts, err = doc.Split(t.Context(), parts[0], second)
	require.NoError(t, err)
	require.Len(t, parts, 2)
	slab := parts[1]
	turn, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 0, 0), units.Radians(0.1366860654293528))
	require.NoError(t, err)
	twisted, err := slab.Placed(t.Context(), turn)
	require.NoError(t, err)
	heelFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(slab))).SelectFaces(twisted)
	require.NoError(t, err)
	require.Len(t, heelFaces, 1)
	heel := heelFaces[0]
	heelCenter, err := heel.Centroid()
	require.NoError(t, err)
	require.Positive(t, heelCenter.Bound.Base())
	seen := map[*decad.Vertex]struct{}{}
	var corners []r3.Vec
	for _, edge := range heel.Edges() {
		for _, vertex := range []*decad.Vertex{edge.Start(), edge.End()} {
			if _, ok := seen[vertex]; ok {
				continue
			}
			seen[vertex] = struct{}{}
			corners = append(corners, vertex.Position().Value)
		}
	}
	require.GreaterOrEqual(t, len(corners), 2)
	sort.Slice(corners, func(i, j int) bool {
		return math.Hypot(corners[i].Y, corners[i].Z) < math.Hypot(corners[j].Y, corners[j].Z)
	})
	anchor := corners[0].Add(corners[1]).Scale(0.5)
	before, err := twisted.Volume()
	require.NoError(t, err)
	current := doc.Bodies()
	_, err = twisted.ScaledAt(t.Context(), anchor, units.Scalar(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, current, doc.Bodies())
	_, err = twisted.ScaledAt(t.Context(), anchor, units.Scalar(-1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, current, doc.Bodies())
	_, err = twisted.ScaledAt(t.Context(), anchor, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	require.Equal(t, current, doc.Bodies())
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = twisted.ScaledAt(canceled, anchor, units.Scalar(0.9776654155427462))
	require.True(t, errors.Is(err, context.Canceled))
	require.Equal(t, current, doc.Bodies())
	const factor = 0.9776654155427462
	scaled, err := twisted.ScaledAt(t.Context(), anchor, units.Scalar(factor))
	require.NoError(t, err)
	after, err := scaled.Volume()
	require.NoError(t, err)
	want := before.Value.Base() * factor * factor * factor
	allow := after.Bound.Base() + before.Bound.Base()*factor*factor*factor + 1e-8
	require.InDelta(t, want, after.Value.Base(), allow)
	scaledHeelFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(slab))).SelectFaces(scaled)
	require.NoError(t, err)
	require.Len(t, scaledHeelFaces, 1)
	scaledHeel := scaledHeelFaces[0]
	_, err = scaledHeel.Centroid()
	require.NoError(t, err)
	capPlane, ok := scaledHeel.Surface().(decad.Plane)
	require.True(t, ok)
	capNormal, err := scaledHeel.NormalAt(capPlane.Frame.Origin())
	require.NoError(t, err)
	require.GreaterOrEqual(t, capNormal.Bound.Base(), 2.0)
	require.False(t, math.IsInf(capNormal.Bound.Base(), 0))
	for _, role := range []decad.FeatureRef{decad.CapStart(slab), decad.CapEnd(slab)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
		require.NoError(t, err)
		require.Len(t, faces, 1)
	}
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	placedCopy, err := scaled.PlacedCopy(t.Context(), r3.Identity())
	require.NoError(t, err)
	copyFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(slab))).SelectFaces(placedCopy)
	require.NoError(t, err)
	require.Len(t, copyFaces, 1)
	_, err = copyFaces[0].Centroid()
	require.NoError(t, err)
}
