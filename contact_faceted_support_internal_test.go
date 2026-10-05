package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFacetedAxisSupportProvesRealUnionFloorFace(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 5, 5, 15, 15, 4,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	mesh, err := union.Tessellate(t.Context(), units.Millimeters(1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Zero(t, mesh.Bound().Base())
	unionPayload, ok := union.payload.(facetedPayload)
	require.True(t, ok)
	require.Zero(t, unionPayload.volSymDiff)

	proof, ok, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 0, dyCmp(proof.plane, mustDyOf(0)))
	require.Equal(t, 0, dyCmp(proof.footLo[0], mustDyOf(0)))
	require.Equal(t, 0, dyCmp(proof.footLo[1], mustDyOf(0)))
	require.Equal(t, 0, dyCmp(proof.footHi[0], mustDyOf(10)))
	require.Equal(t, 0, dyCmp(proof.footHi[1], mustDyOf(10)))
	require.Equal(t, r3.Vec{Z: -1}, proof.normal)
	require.Contains(t, union.Faces(), proof.face)
	for i, want := range []r3.Vec{{}, {X: 10}, {X: 10, Y: 10}, {Y: 10}} {
		require.Equal(t, dyVec(want), proof.corners[i])
	}
	require.Equal(t, 0, dyCmp(proof.outerHi[0], mustDyOf(15)))
	require.Equal(t, 0, dyCmp(proof.outerHi[1], mustDyOf(15)))
	require.Equal(t, 0, dyCmp(proof.outerHi[2], mustDyOf(12)))

	top, ok, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 1)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 0, dyCmp(top.plane, mustDyOf(12)))
	require.Equal(t, 0, dyCmp(top.footLo[0], mustDyOf(5)))
	require.Equal(t, 0, dyCmp(top.footHi[0], mustDyOf(15)))
	require.Equal(t, r3.Vec{Z: 1}, top.normal)
	require.Contains(t, union.Faces(), top.face)
	require.NotSame(t, proof.face, top.face)

	left, ok, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 0, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -1}, left.normal)
	require.Equal(t, 0, dyCmp(left.footHi[0], mustDyOf(10)))
	require.Equal(t, 0, dyCmp(left.footHi[1], mustDyOf(10)))
	require.Contains(t, union.Faces(), left.face)
	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	reflected, ok, err := sourceFacetedAxisSupport(t.Context(), union, reflection, 0, 1)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, left.face, reflected.face)
	require.Equal(t, r3.Vec{X: 1}, reflected.normal)

	shift, err := r3.Translation(r3.Vec{X: 2, Y: 3, Z: 20})
	require.NoError(t, err)
	moved, ok, err := sourceFacetedAxisSupport(t.Context(), union, shift, 2, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, proof.face, moved.face)
	require.Equal(t, 0, dyCmp(moved.plane, mustDyOf(20)))
	require.Equal(t, 0, dyCmp(moved.footLo[0], mustDyOf(2)))
	require.Equal(t, 0, dyCmp(moved.footLo[1], mustDyOf(3)))

	// Rebuilding a faceted body through an inexact translation widens its
	// source boundary. A triangle's plane cannot then certify the true normal.
	inexactShift, err := r3.Translation(r3.Vec{X: 0.1})
	require.NoError(t, err)
	widened, err := union.Placed(t.Context(), inexactShift)
	require.NoError(t, err)
	widenedPayload, ok := widened.payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, widenedPayload.meshBound)
	_, ok, err = sourceFacetedAxisSupport(t.Context(), widened, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.False(t, ok)
}

func TestFacetedAxisSupportRefusesTwoLowestFaces(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 20, 0, 30, 10, 0,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	payload, ok := union.payload.(facetedPayload)
	require.True(t, ok)
	require.Zero(t, payload.meshBound)
	require.Zero(t, payload.volSymDiff)
	_, ok, err = sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.False(t, ok)
}

func TestFacetedAxisSupportRefusesHoledFootprint(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 5, 5, 15, 15, 4,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	tool := internalOffsetBox(t, doc, 2, 2, 4, 4, -2,
		Distance{D: units.Millimeters(4), Dir: Along})
	holed, err := Cut(t.Context(), union, tool)
	require.NoError(t, err)
	mesh, err := holed.Tessellate(t.Context(), units.Millimeters(1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Zero(t, mesh.Bound().Base())
	payload, ok := holed.payload.(facetedPayload)
	require.True(t, ok)
	require.Zero(t, payload.volSymDiff)
	_, ok, err = sourceFacetedAxisSupport(t.Context(), holed, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.False(t, ok)
}
