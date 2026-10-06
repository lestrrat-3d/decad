package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func revolvedCylinderZPose(t *testing.T, z, x float64) r3.Transform {
	t.Helper()
	pose, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Z: 1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Y: 1},
	}, r3.Vec{X: x, Z: z})
	require.NoError(t, err)
	return pose
}

func TestContactPairRevolvedCylinderAxialDiskFace(t *testing.T) {
	doc := decad.New()
	sketch, profile := solidSketch(t)
	cylinder, err := doc.Revolve(sketch, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	ceiling := boxBodyAtZ(t, doc, -20, -20, 20, 20, 10, 10)
	placed, err := cylinder.PlacedCopy(t.Context(), revolvedCylinderZPose(t, 0, 0))
	require.NoError(t, err)
	fractionalPlaced, err := cylinder.PlacedCopy(t.Context(), revolvedCylinderZPose(t, .1, 0))
	require.NoError(t, err)
	reflection, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Z: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Y: 1},
	}, r3.Vec{Z: 10})
	require.NoError(t, err)
	reflected, err := cylinder.PlacedCopy(t.Context(), reflection)
	require.NoError(t, err)
	before := doc.Bodies()
	req := contactRequest()
	for _, tc := range []struct {
		name     string
		z        float64
		relation decad.ContactRelation
		gap      float64
	}{
		{name: "separated", z: 1, relation: decad.ContactSeparated, gap: 1},
		{name: "touching", z: 0, relation: decad.ContactTouching},
		{name: "shallow overlap", z: -.5, relation: decad.ContactOverlapping},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pose := revolvedCylinderZPose(t, tc.z, 0)
			for _, reverse := range []bool{false, true} {
				a, b, poseA, poseB := floor, cylinder, r3.Identity(), pose
				if reverse {
					a, b, poseA, poseB = b, a, poseB, poseA
				}
				report, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, req)
				require.NoError(t, err)
				require.Equal(t, tc.relation, report.Relation, "reason=%v reverse=%v", report.Reason, reverse)
				if tc.relation == decad.ContactSeparated {
					require.Equal(t, tc.gap, report.Gap.Value.Base())
					require.Nil(t, report.Manifold)
					continue
				}
				require.NotNil(t, report.Manifold, "reason=%v reverse=%v", report.Reason, reverse)
				require.Len(t, report.Manifold.Points, 1)
				point := report.Manifold.Points[0]
				cylinderFace, cylinderWitness := point.FaceB, point.OnB
				wantNormal := r3.Vec{Z: 1}
				if reverse {
					cylinderFace, cylinderWitness = point.FaceA, point.OnA
					wantNormal.Z = -1
				}
				require.Contains(t, cylinder.Faces(), cylinderFace)
				require.IsType(t, decad.Plane{}, cylinderFace.Surface())
				require.Equal(t, r3.Vec{Z: tc.z}, cylinderWitness.Value)
				require.Equal(t, wantNormal, point.Normal.Value)
				require.Zero(t, point.Normal.Bound.Base())
				require.InDelta(t, tc.z, point.Separation.Value.Base(), 1e-12)
			}
		})
	}
	pose := revolvedCylinderZPose(t, 0, 0)
	lower, err := doc.ContactPair(t.Context(), floor, cylinder, r3.Identity(), pose, req)
	require.NoError(t, err)
	upper, err := doc.ContactPair(t.Context(), cylinder, ceiling, pose, r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, upper.Relation)
	require.Len(t, upper.Manifold.Points, 1)
	lowFace := lower.Manifold.Points[0].FaceB
	highFace := upper.Manifold.Points[0].FaceA
	require.NotSame(t, lowFace, highFace)
	for _, tc := range []struct {
		face   *decad.Face
		at     r3.Vec
		normal r3.Vec
	}{
		{face: lowFace, at: r3.Vec{}, normal: r3.Vec{Z: -1}},
		{face: highFace, at: r3.Vec{X: 10}, normal: r3.Vec{Z: 1}},
	} {
		reading, err := tc.face.NormalAt(tc.at)
		require.NoError(t, err)
		require.Equal(t, tc.normal, pose.ApplyDir(reading.Value))
		require.Zero(t, reading.Bound.Base())
	}
	require.Equal(t, r3.Vec{Z: 1}, upper.Manifold.Points[0].Normal.Value)
	require.Equal(t, r3.Vec{Z: 10}, upper.Manifold.Points[0].OnA.Value)
	placedReport, err := doc.ContactPair(t.Context(), floor, placed,
		r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, placedReport.Relation, "reason=%v", placedReport.Reason)
	require.Len(t, placedReport.Manifold.Points, 1)
	require.Contains(t, placed.Faces(), placedReport.Manifold.Points[0].FaceB)
	fractionalReport, err := doc.ContactPair(t.Context(), floor, fractionalPlaced,
		r3.Identity(), contactPose(t, r3.Vec{Z: -.1}), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, fractionalReport.Relation,
		"reason=%v", fractionalReport.Reason)
	require.Len(t, fractionalReport.Manifold.Points, 1)
	require.Contains(t, fractionalPlaced.Faces(), fractionalReport.Manifold.Points[0].FaceB)
	reflectedReport, err := doc.ContactPair(t.Context(), floor, reflected,
		r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, reflectedReport.Relation,
		"reason=%v", reflectedReport.Reason)
	require.Len(t, reflectedReport.Manifold.Points, 1)
	require.Contains(t, reflected.Faces(), reflectedReport.Manifold.Points[0].FaceB)
	require.Equal(t, r3.Vec{Z: 1}, reflectedReport.Manifold.Points[0].Normal.Value)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairRevolvedCylinderRefusesUnprovedDiskCorridor(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	sketch, profile := solidSketch(t)
	full, err := doc.Revolve(sketch, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	partial, err := doc.Revolve(sketch, profile, uAxis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	annularInput, annularProfile := annularSketch(t)
	annular, err := doc.Revolve(annularInput, annularProfile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		body *decad.Body
		pose r3.Transform
	}{
		{name: "partial", body: partial, pose: revolvedCylinderZPose(t, 1, 0)},
		{name: "annular", body: annular, pose: revolvedCylinderZPose(t, 1, 0)},
		{name: "floor edge", body: full, pose: revolvedCylinderZPose(t, 0, 12)},
		{name: "embedded", body: full, pose: revolvedCylinderZPose(t, -10, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			report, err := doc.ContactPair(t.Context(), floor, tc.body, r3.Identity(), tc.pose, contactRequest())
			require.NoError(t, err)
			require.Equal(t, decad.ContactUndecided, report.Relation)
			require.Nil(t, report.Manifold)
		})
	}
}
