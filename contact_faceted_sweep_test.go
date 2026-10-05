package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func facetedFloorSweepFixture(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalOffsetBox(t, doc, 5, 5, 15, 15, 4,
		Distance{D: units.Millimeters(8), Dir: Along})
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	_, faceted := union.payload.(facetedPayload)
	require.True(t, faceted)
	floor := internalOffsetBox(t, doc, -20, -20, 20, 20, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	return doc, floor, union
}

func facetedSweepRequest(policy SweepStartPolicy) SweepRequest {
	return SweepRequest{
		ContactRequest: ContactRequest{
			PointResolution:  units.Millimeters(1e-6),
			NormalResolution: units.Degrees(1),
		},
		TimeResolution: units.Seconds(1e-6), MaxPoseEvaluations: 8, StartPolicy: policy,
	}
}

func facetedSweepPose(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.Translation(v)
	require.NoError(t, err)
	return pose
}

func facetedSweepPath(from, to r3.Transform) PoseSegment {
	return PoseSegment{From: from, To: to, Duration: units.Seconds(1)}
}

func TestSweepPairRealFacetedUnionClearAndImpact(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	before := doc.Bodies()
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	start := facetedSweepPose(t, r3.Vec{Z: 10})
	contact, err := doc.ContactPair(t.Context(), floor, union, r3.Identity(), start,
		facetedSweepRequest(StopAtInitialContact).ContactRequest)
	require.NoError(t, err)
	require.Equal(t, ContactSeparated, contact.Relation)
	require.Equal(t, units.Millimeters(10), contact.Gap.Value)

	clearEnd := facetedSweepPose(t, r3.Vec{Z: 5})
	clearSweep, err := doc.SweepPair(t.Context(), floor, union, still,
		facetedSweepPath(start, clearEnd), facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepClear, clearSweep.Outcome, "cause=%v", clearSweep.Cause)
	require.True(t, clearSweep.HasAffineReplayProof())
	for _, elapsed := range []float64{0, 0.25, 0.5, 1} {
		poseA, poseB, err := clearSweep.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(StopAtInitialContact).ContactRequest)
		require.NoError(t, err)
		require.Equal(t, ContactSeparated, at.Relation)
	}

	end := facetedSweepPose(t, r3.Vec{Z: -10})
	impact, err := doc.SweepPair(t.Context(), floor, union, still,
		facetedSweepPath(start, end), facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepImpactBracket, impact.Outcome, "cause=%v", impact.Cause)
	require.NotNil(t, impact.Bracket)
	require.Equal(t, 0.5, impact.Bracket.To.Fraction.Base())
	require.Equal(t, ContactSeparated, impact.Samples[0].Ideal.Relation)
	require.Equal(t, ContactTouching, impact.Event.Relation)
	require.Len(t, impact.Event.Manifold.Points, 4)
	proof, ok, err := sourceFacetedAxisSupport(t.Context(), union, r3.Identity(), 2, 0)
	require.NoError(t, err)
	require.True(t, ok)
	for _, point := range impact.Event.Manifold.Points {
		require.Same(t, proof.face, point.FaceB)
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	}
	for _, elapsed := range []float64{0, 0.1, 0.25, 0.5} {
		poseA, poseB, err := impact.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(StopAtInitialContact).ContactRequest)
		require.NoError(t, err)
		if elapsed == 0.5 {
			require.Equal(t, ContactTouching, at.Relation)
		} else {
			require.Equal(t, ContactSeparated, at.Relation)
		}
	}
	_, _, err = impact.CertifiedPosesAt(units.Seconds(0.75))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairRealFacetedUnionTouchContinuations(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	up := facetedSweepPose(t, r3.Vec{Z: 5})
	depart, err := doc.SweepPair(t.Context(), union, floor,
		facetedSweepPath(r3.Identity(), up), still,
		facetedSweepRequest(ContinueSeparatingTouch))
	require.NoError(t, err)
	require.Equal(t, SweepDepartedClear, depart.Outcome, "cause=%v", depart.Cause)
	require.NotNil(t, depart.Departure)
	require.Equal(t, units.Millimeters(5), depart.Departure.GapAtUntil.Value)
	for _, elapsed := range []float64{0, 0.25, 1} {
		poseA, poseB, err := depart.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), union, floor, poseA, poseB,
			facetedSweepRequest(ContinueSeparatingTouch).ContactRequest)
		require.NoError(t, err)
		if elapsed == 0 {
			require.Equal(t, ContactTouching, at.Relation)
		} else {
			require.Equal(t, ContactSeparated, at.Relation)
		}
	}

	shift := facetedSweepPose(t, r3.Vec{X: 0.1})
	track, err := doc.SweepPair(t.Context(), floor, union,
		facetedSweepPath(r3.Identity(), shift),
		facetedSweepPath(r3.Identity(), shift),
		facetedSweepRequest(ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, SweepPersistentTouch, track.Outcome, "cause=%v", track.Cause)
	require.NotNil(t, track.ContactTrack)
	for _, fraction := range []float64{0, 0.25, 0.5, 1} {
		manifold, err := track.ContactTrack.ManifoldAt(units.Scalar(fraction))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 4)
		poseA, poseB, err := track.CertifiedPosesAt(units.Seconds(fraction))
		require.NoError(t, err)
		at, err := doc.ContactPair(t.Context(), floor, union, poseA, poseB,
			facetedSweepRequest(ContinueCertifiedTouch).ContactRequest)
		require.NoError(t, err)
		require.Equal(t, ContactTouching, at.Relation)
	}
}

func TestSweepPairFacetedFloorRefusesUnprovedPath(t *testing.T) {
	doc, floor, union := facetedFloorSweepFixture(t)
	still := facetedSweepPath(r3.Identity(), r3.Identity())
	start := facetedSweepPose(t, r3.Vec{Z: 10})
	for _, end := range []r3.Vec{{X: 1, Z: -10}, {Z: -7}} {
		pose := facetedSweepPose(t, end)
		report, err := doc.SweepPair(t.Context(), floor, union, still,
			facetedSweepPath(start, pose), facetedSweepRequest(StopAtInitialContact))
		require.NoError(t, err)
		require.Equal(t, SweepUndecided, report.Outcome, "end=%v", end)
		require.False(t, report.HasAffineReplayProof())
	}
	edgeFloor := internalOffsetBox(t, doc, 0, 0, 10, 10, -10,
		Distance{D: units.Millimeters(10), Dir: Along})
	edge, err := doc.SweepPair(t.Context(), edgeFloor, union, still, still,
		facetedSweepRequest(ContinueCertifiedTouch))
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, edge.Outcome)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	reflected, err := union.Placed(t.Context(), reflection)
	require.NoError(t, err)
	require.Positive(t, reflected.payload.(facetedPayload).meshBound)
	require.Empty(t, reflected.payload.(facetedPayload).exactSourceVerts)
	unproved, err := doc.SweepPair(t.Context(), floor, reflected, still,
		facetedSweepPath(start, facetedSweepPose(t, r3.Vec{Z: -10})),
		facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepUndecided, unproved.Outcome)
	require.False(t, unproved.HasAffineReplayProof())
	clear, err := doc.SweepPair(t.Context(), floor, reflected, still,
		facetedSweepPath(start, facetedSweepPose(t, r3.Vec{Z: 5})),
		facetedSweepRequest(StopAtInitialContact))
	require.NoError(t, err)
	require.Equal(t, SweepClear, clear.Outcome)
	require.True(t, clear.HasAffineReplayProof())
	_, _, err = clear.CertifiedPosesAt(units.Seconds(.5))
	require.NoError(t, err)
}
