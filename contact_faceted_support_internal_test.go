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

	// Rebuilding through an inexact translation widens the held mesh, while
	// the saved zero-bound source still certifies the true support plane.
	inexactShift, err := r3.Translation(r3.Vec{X: 0.1})
	require.NoError(t, err)
	widened, err := union.Placed(t.Context(), inexactShift)
	require.NoError(t, err)
	widenedPayload, ok := widened.payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, widenedPayload.meshBound)
	widenedProof, ok, err := sourceFacetedAxisSupport(t.Context(), widened, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 0, dyCmp(widenedProof.plane, mustDyOf(0)))
	require.Equal(t, 0, dyCmp(widenedProof.footLo[0], mustDyOf(.1)))
	require.Contains(t, widened.Faces(), widenedProof.face)
}

func TestFacetedAxisSupportRejectsMalformedPlacementProvenance(t *testing.T) {
	_, _, placed := boundedFacetedFloorFixture(t)
	valid := placed.payload.(facetedPayload)
	require.NotEmpty(t, valid.exactSourceVerts)
	require.NotEmpty(t, valid.exactSourceTris)
	missing := valid
	missing.exactSourceVerts = nil
	wrongTris := valid
	wrongTris.exactSourceTris = append([][3]int(nil), valid.exactSourceTris...)
	wrongTris.exactSourceTris[0][0] = -1
	wrongVerts := valid
	wrongVerts.exactSourceVerts = append([]r3.Vec(nil), valid.exactSourceVerts...)
	wrongVerts.exactSourceVerts[0].X += 1
	for _, corrupt := range []facetedPayload{missing, wrongTris, wrongVerts} {
		body := &Body{lumps: placed.lumps, solid: placed.solid, kind: placed.kind, payload: corrupt}
		_, ok, err := sourceFacetedAxisSupport(t.Context(), body, r3.Identity(), 2, 0)
		require.NoError(t, err)
		require.False(t, ok)
	}

	doc, _, source := facetedFloorSweepFixture(t)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	reflected, err := source.Placed(t.Context(), reflection)
	require.NoError(t, err)
	require.Same(t, doc, reflected.Document())
	require.Empty(t, reflected.payload.(facetedPayload).exactSourceVerts)
	_, ok, err := sourceFacetedAxisSupport(t.Context(), reflected, r3.Identity(), 2, 0)
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

func TestContactPairRealFacetedUnionOnFloor(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 5, 5, 15, 15, 4,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	before := doc.Bodies()
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}
	report, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactTouching, report.Relation, "reason=%v", report.Reason)
	require.Same(t, floor, report.A)
	require.Same(t, union, report.B)
	require.Equal(t, ContactNoReason, report.Reason)
	require.Equal(t, Measurement{Value: units.Millimeters(0), Exactness: Exact,
		Bound: units.Millimeters(0)}, *report.Gap)
	require.NotNil(t, report.Manifold)
	require.Len(t, report.Manifold.Points, 4)
	face, ok, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.True(t, ok)
	for i, want := range []r3.Vec{{}, {X: 10}, {X: 10, Y: 10}, {Y: 10}} {
		point := report.Manifold.Points[i]
		require.Equal(t, want, point.OnA.Value)
		require.Equal(t, want, point.OnB.Value)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Zero(t, point.Normal.Bound.Base())
		require.Zero(t, point.NormalAngle.Base())
		require.Zero(t, point.Separation.Value.Base())
		require.Zero(t, point.OnA.Bound.Base())
		require.Zero(t, point.OnB.Bound.Base())
		require.Same(t, face.face, point.FaceB)
		require.Same(t, face.face, point.FeatureB.Face)
		require.Contains(t, floor.Faces(), point.FaceA)
		require.Same(t, point.FaceA, point.FeatureA.Face)
	}
	reversed, err := doc.ContactPair(t.Context(), union, floor, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactTouching, reversed.Relation)
	require.Len(t, reversed.Manifold.Points, 4)
	for i, point := range reversed.Manifold.Points {
		require.Equal(t, report.Manifold.Points[i].OnB, point.OnA)
		require.Equal(t, report.Manifold.Points[i].OnA, point.OnB)
		require.Equal(t, r3.Vec{Z: -1}, point.Normal.Value)
		require.Same(t, face.face, point.FaceA)
		require.Contains(t, floor.Faces(), point.FaceB)
	}
	up, err := r3.Translation(r3.Vec{Z: 3})
	require.NoError(t, err)
	separated, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), up, req)
	require.NoError(t, err)
	require.Equal(t, ContactSeparated, separated.Relation)
	require.Equal(t, Measurement{Value: units.Millimeters(3), Exactness: Exact,
		Bound: units.Millimeters(0)}, *separated.Gap)
	require.Nil(t, separated.Manifold)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairFacetedFloorRefusesUnprovedPatches(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 5, 5, 15, 15, 4,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}
	for _, pose := range []r3.Vec{{X: 10}, {X: 25}, {Z: -1}} {
		placed, err := r3.Translation(pose)
		require.NoError(t, err)
		report, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), placed, req)
		require.NoError(t, err)
		require.Equal(t, ContactUndecided, report.Relation, "pose=%v", pose)
		require.Nil(t, report.Manifold)
		require.Nil(t, report.Gap)
	}
	shifted, err := r3.Translation(r3.Vec{X: 0.1})
	require.NoError(t, err)
	tight := req
	tight.PointResolution = units.Millimeters(1e-20)
	coarse, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), shifted, tight)
	require.NoError(t, err)
	require.Equal(t, ContactTouching, coarse.Relation)
	require.Nil(t, coarse.Manifold)
	require.Equal(t, ContactPointTooCoarse, coarse.Reason)
	shift, err := r3.Translation(r3.Vec{X: 0.1})
	require.NoError(t, err)
	widened, err := union.Placed(t.Context(), shift)
	require.NoError(t, err)
	report, err := doc.ContactPair(t.Context(), floor, widened, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactUndecided, report.Relation)
	require.Nil(t, report.Manifold)
}

func TestContactPairFacetedFloorRefusesMultipleSupportFaces(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalBoxBody(t, doc, 20, 0, 30, 10, 8)
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	floor := internalOffsetBox(t, doc, -20, -20, 40, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}
	report, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, ContactUndecided, report.Relation)
	require.Nil(t, report.Manifold)
	require.Nil(t, report.Gap)
}
