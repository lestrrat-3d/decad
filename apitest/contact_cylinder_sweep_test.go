package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The disk-track certificate has two legs, each shown to fail by deleting it:
//   - the zero axial slope: with it and the final touching-sample check both
//     removed, the closing path publishes a track that overlaps at its end. The
//     final ideal sample alone also refuses it, since gap(1) equals the slope.
//   - the conversion envelope (sourceTrackPointsWithin): removed, the far-out
//     slide publishes a track whose ManifoldAt fails at fraction 2^-20.
//
// The disk corridor is the impact path's existing strict endpoint check.

func cylinderTrackRequest() decad.SweepRequest {
	req := sweepRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	return req
}

func TestSweepPairCylinderDiskTrack(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := verticalCylinder(t, doc, 2, 0, 5)
	before := doc.Bodies()
	still := sweepDrift(r3.Vec{}, 1)
	slide := sweepDrift(r3.Vec{X: 8, Y: -4}, 1)
	slide.From = contactPose(t, r3.Vec{Z: 5})
	req := cylinderTrackRequest()
	for _, reverse := range []bool{false, true} {
		a, b, pathA, pathB := floor, cylinder, decad.PairPath(still), decad.PairPath(slide)
		normal := r3.Vec{Z: 1}
		if reverse {
			a, b, pathA, pathB = b, a, pathB, pathA
			normal.Z = -1
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v reverse=%v", report.Cause, reverse)
		track := report.ContactTrack
		require.NotNil(t, track)
		require.Equal(t, 0.0, track.Start().Fraction.Base())
		require.Equal(t, 1.0, track.End().Fraction.Base())
		require.Equal(t, normal, track.Normal().Value)
		require.Zero(t, track.Normal().Bound.Base())
		require.Nil(t, track.Band())
		featureA, featureB := track.Features()
		floorFace, cylinderFace := featureA.Face, featureB.Face
		if reverse {
			floorFace, cylinderFace = cylinderFace, floorFace
		}
		require.Contains(t, floor.Faces(), floorFace)
		require.Contains(t, cylinder.Faces(), cylinderFace)
		require.IsType(t, decad.Plane{}, cylinderFace.Surface())
		for _, fraction := range []float64{0, 0.25, 0.5, 1} {
			manifold, err := track.ManifoldAt(units.Scalar(fraction))
			require.NoError(t, err)
			require.Len(t, manifold.Points, 1)
			point := manifold.Points[0]
			// The disk center, sketched at x = 2, is the witness; it slides
			// with the cylinder on the floor's top plane.
			center := r3.Vec{X: 2 + 8*fraction, Y: -4 * fraction}
			require.Equal(t, center, point.OnA.Value)
			require.Equal(t, center, point.OnB.Value)
			require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())
			require.LessOrEqual(t, point.OnB.Bound.Base(), req.PointResolution.Base())
			require.Equal(t, normal, point.Normal.Value)
			require.Zero(t, point.Separation.Value.Base())
			require.Zero(t, point.Separation.Bound.Base())
		}
		require.True(t, report.HasAffineReplayProof())
		poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(0.5))
		require.NoError(t, err)
		floorPose, cylinderPose := poseA, poseB
		if reverse {
			floorPose, cylinderPose = poseB, poseA
		}
		require.Equal(t, r3.Identity(), floorPose)
		require.Equal(t, r3.Vec{X: 4, Y: -2, Z: 5}, cylinderPose.Translation())
		_, err = track.ManifoldAt(units.Scalar(1.5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	}

	stillCylinder := sweepDrift(r3.Vec{}, 1)
	stillCylinder.From = slide.From
	resting, err := doc.SweepPair(t.Context(), floor, cylinder, still, stillCylinder, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, resting.Outcome, "cause=%v", resting.Cause)
	require.Equal(t, uint64(2), resting.PoseEvaluations)
	restPoint, err := resting.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Equal(t, r3.Vec{X: 2}, restPoint.Points[0].OnB.Value)

	rectangle, profile := solidSketch(t)
	revolved, err := doc.Revolve(rectangle, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	revolvedSlide := sweepDrift(r3.Vec{X: 4}, 1)
	revolvedSlide.From = revolvedCylinderZPose(t, 0, 0)
	revolvedReport, err := doc.SweepPair(t.Context(), floor, revolved, still, revolvedSlide, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, revolvedReport.Outcome, "cause=%v", revolvedReport.Cause)
	revolvedPoint, err := revolvedReport.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Equal(t, r3.Vec{X: 2}, revolvedPoint.Points[0].OnB.Value)
	_, revolvedPose, err := revolvedReport.CertifiedPosesAt(units.Seconds(0.25))
	require.NoError(t, err)
	require.Equal(t, r3.Vec{X: 1}, revolvedPose.Translation())
	require.Equal(t, append(before, revolved), doc.Bodies())
}

func TestSweepPairCylinderDiskDeparture(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := verticalCylinder(t, doc, 0, 0, 5)
	lift := sweepDrift(r3.Vec{X: 10, Z: 50}, 0.1)
	lift.From = contactPose(t, r3.Vec{Z: 5})
	report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 0.1), lift,
		cylinderTrackRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.Departure)
	require.Nil(t, report.ContactTrack)
	// 50 mm/s for 0.1 s lifts the disk 5 mm off the floor.
	require.InDelta(t, 5, report.Departure.GapAtUntil.Value.Base(), 1e-12)
	_, lifted, err := report.CertifiedPosesAt(units.Seconds(0.05))
	require.NoError(t, err)
	require.InDelta(t, 0.5, lifted.Translation().X, 1e-12)
	require.InDelta(t, 7.5, lifted.Translation().Z, 1e-12)
}

func TestSweepPairCylinderDiskTrackRefusals(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := verticalCylinder(t, doc, 0, 0, 5)
	still := sweepDrift(r3.Vec{}, 1)
	resting := func(v r3.Vec) decad.RigidDriftSegment {
		path := sweepDrift(v, 1)
		path.From = contactPose(t, r3.Vec{Z: 5})
		return path
	}
	for _, tc := range []struct {
		name   string
		path   decad.RigidDriftSegment
		policy decad.SweepStartPolicy
		cause  decad.SweepCause
	}{
		{name: "separating policy", path: resting(r3.Vec{}), policy: decad.ContinueSeparatingTouch,
			cause: decad.SweepDepartureUnproved},
		{name: "closing", path: resting(r3.Vec{Z: -1}), policy: decad.ContinueCertifiedTouch,
			cause: decad.SweepContactTrackUnproved},
		// The disk ends at x in [15, 25], past the floor's x = 20 edge.
		{name: "leaves face", path: resting(r3.Vec{X: 20}), policy: decad.ContinueCertifiedTouch,
			cause: decad.SweepContactUnsupported},
	} {
		t.Run(tc.name, func(t *testing.T) {
			req := sweepRequest()
			req.StartPolicy = tc.policy
			report, err := doc.SweepPair(t.Context(), floor, cylinder, still, tc.path, req)
			require.NoError(t, err)
			require.Equal(t, decad.SweepUndecided, report.Outcome)
			require.Equal(t, tc.cause, report.Cause)
			require.Nil(t, report.ContactTrack)
		})
	}

	// Far from the origin the float spacing exceeds PointResolution: the
	// endpoints at x = 2^33 and 2^33 + 1 read exactly, but the disk center at
	// fraction 2^-20 lies halfway between two floats.
	far := math.Ldexp(1, 33)
	farFloor := sweepDrift(r3.Vec{}, 1)
	farFloor.From = contactPose(t, r3.Vec{X: far})
	farSlide := sweepDrift(r3.Vec{X: 1}, 1)
	farSlide.From = contactPose(t, r3.Vec{X: far, Z: 5})
	report, err := doc.SweepPair(t.Context(), floor, cylinder, farFloor, farSlide, cylinderTrackRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, report.Outcome)
	require.Equal(t, decad.SweepContactTrackUnproved, report.Cause)
	require.NotNil(t, report.InitialEvent)
	require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
}
