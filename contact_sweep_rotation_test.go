package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContactPairRotatedBoxContainedHorizontalFace(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	box := boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 10)
	turn, err := r3.RotationAround(r3.Vec{Z: 5}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), turn, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.InDelta(t, 0, point.OnA.Value.Z, point.OnA.Bound.Base())
		require.InDelta(t, 0, point.OnB.Value.Z, point.OnB.Bound.Base())
		require.NotNil(t, point.FaceA)
		require.NotNil(t, point.FaceB)
	}
	reversed, err := doc.ContactPair(t.Context(), box, floor, turn, r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, reversed.Relation)
	require.Len(t, reversed.Manifold.Points, 4)
	require.Equal(t, r3.Vec{Z: -1}, reversed.Manifold.Points[0].Normal.Value)

	shift, err := r3.Translation(r3.Vec{Z: -0.125})
	require.NoError(t, err)
	overlapPose, err := turn.Then(shift)
	require.NoError(t, err)
	overlap, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), overlapPose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, overlap.Relation)
	require.Len(t, overlap.Manifold.Points, 4)
	require.InDelta(t, -0.125, overlap.Manifold.Points[0].Separation.Value.Base(),
		overlap.Manifold.Points[0].Separation.Bound.Base())
	reverseOverlap, err := doc.ContactPair(t.Context(), box, floor, overlapPose, r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, reverseOverlap.Relation)
	require.Len(t, reverseOverlap.Manifold.Points, 4)
	require.Equal(t, r3.Vec{Z: -1}, reverseOverlap.Manifold.Points[0].Normal.Value)

	narrow := boxBodyAtZ(t, doc, -5, -5, 5, 5, -10, 10)
	unresolved, err := doc.ContactPair(t.Context(), narrow, box, r3.Identity(), turn, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, unresolved.Relation)
	require.Nil(t, unresolved.Manifold)
}

func TestContactPairOrientedSourceBoxes(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 12, 0, 22, 10, 10)
	center := r3.Vec{X: 5, Y: 5, Z: 5}
	for _, test := range []struct {
		angle    float64
		relation decad.ContactRelation
		gap      float64
	}{
		{angle: 30, relation: decad.ContactSeparated, gap: .1698729810778068},
		{angle: 45, relation: decad.ContactOverlapping},
		{angle: 60, relation: decad.ContactSeparated, gap: .1698729810778068},
	} {
		pose, err := r3.RotationAround(center, r3.Vec{Z: 1}, units.Degrees(test.angle))
		require.NoError(t, err)
		report, err := doc.ContactPair(t.Context(), a, b, pose, r3.Identity(), contactRequest())
		require.NoError(t, err)
		require.Equal(t, test.relation, report.Relation)
		if test.relation == decad.ContactSeparated {
			require.NotNil(t, report.Gap)
			require.Greater(t, report.Gap.Value.Base()-report.Gap.Bound.Base(), 0.0)
			require.InDelta(t, test.gap, report.Gap.Value.Base(),
				report.Gap.Bound.Base()+1e-12)
		} else {
			require.Nil(t, report.Manifold)
			require.Equal(t, decad.ContactNoNormalProof, report.Reason)
		}
	}

	// Both coordinate bounding boxes overlap at 45 degrees, while the
	// complete rotated boxes are separated on a diagonal face axis.
	second := decad.New()
	c := boxBody(t, second, -5, -5, 5, 5, 10)
	d := boxBody(t, second, 5, 5, 15, 15, 10)
	pose, err := r3.RotationAround(r3.Vec{Z: 5}, r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	report, err := second.ContactPair(t.Context(), c, d, pose, r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, report.Relation)
	require.NotNil(t, report.Gap)
	require.Greater(t, report.Gap.Value.Base()-report.Gap.Bound.Base(), 0.0)
}

func TestContactPairOrientedAxisFaceWitness(t *testing.T) {
	doc := decad.New()
	driver := boxBody(t, doc, 0, 0, 10, 10, 10)
	box := boxBody(t, doc, 19, 0, 29, 10, 10)
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(45))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 9.0001})
	require.NoError(t, err)
	pose, err := turn.Then(shift)
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), driver, box, pose, r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 1)
	point := contact.Manifold.Points[0]
	require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
	require.InDelta(t, -.0001, point.Separation.Value.Base(), 1e-10)
	require.InDelta(t, 5, point.OnA.Value.Y, 1e-9)
	require.InDelta(t, 5, point.OnA.Value.Z, 1e-9)
	require.NotNil(t, point.FaceA)
	require.NotNil(t, point.FaceB)

	reverse, err := doc.ContactPair(t.Context(), box, driver, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, reverse.Relation)
	require.NotNil(t, reverse.Manifold)
	require.Equal(t, r3.Vec{X: -1}, reverse.Manifold.Points[0].Normal.Value)
}

func TestSweepPairRotatingPoseSegmentBracketsAxisFaceImpact(t *testing.T) {
	doc := decad.New()
	driver := boxBody(t, doc, 0, 0, 10, 10, 10)
	box := boxBody(t, doc, 19, 0, 29, 10, 10)
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	endpoint, err := turn.Then(shift)
	require.NoError(t, err)
	path := decad.PoseSegment{From: r3.Identity(), To: endpoint, Duration: units.Seconds(1)}
	request := sweepRequest()
	request.MaxPoseEvaluations = 128
	report, err := doc.SweepPair(t.Context(), driver, box, path, sweepDrift(r3.Vec{}, 1), request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	require.InDelta(t, .45, report.Bracket.To.Elapsed.Value.Base(), 1e-8)
	require.NotNil(t, report.Event)
	require.Nil(t, report.Event.Manifold)
	found := false
	for _, sample := range report.Samples {
		if sample.At.Fraction != report.Bracket.To.Fraction {
			continue
		}
		relation, err := doc.ContactPair(t.Context(), driver, box, sample.PoseA, sample.PoseB,
			contactRequest())
		require.NoError(t, err)
		require.Equal(t, decad.ContactOverlapping, relation.Relation)
		require.NotNil(t, relation.Manifold)
		require.Equal(t, r3.Vec{X: 1}, relation.Manifold.Points[0].Normal.Value)
		found = true
	}
	require.True(t, found)
}

func TestSweepPairRotatingBoxFindsHiddenImpact(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 12, 0, 22, 10, 10)
	before := doc.Bodies()
	turn := sweepDrift(r3.Vec{}, 1)
	turn.Center = r3.Vec{X: 5, Y: 5, Z: 5}
	turn.AngularVelocity.Z = units.RadiansPerSecond(1.5707963267948966)
	req := sweepRequest()
	req.TimeResolution = units.Seconds(1e-6)
	req.MaxPoseEvaluations = 128
	report, err := doc.SweepPair(t.Context(), a, b, turn, sweepDrift(r3.Vec{}, 1), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	require.NotNil(t, report.Bracket)
	require.False(t, report.BracketEndsAtDuration())
	require.NotNil(t, report.Event)
	require.Equal(t, decad.ContactOverlapping, report.Event.Relation)
	require.Nil(t, report.Event.Manifold)
	left, right := report.Bracket.From.Elapsed.Value.Base(), report.Bracket.To.Elapsed.Value.Base()
	require.Less(t, left, .409666)
	require.Greater(t, right, .409665)
	require.LessOrEqual(t, right-left, req.TimeResolution.Base())
	require.Equal(t, decad.ContactSeparated, report.Samples[0].Ideal.Relation)
	require.Equal(t, decad.ContactSeparated, report.Samples[len(report.Samples)-1].Ideal.Relation)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairEqualSpinDepartsFromSourceFace(t *testing.T) {
	doc := decad.New()
	lower := boxBodyAtZ(t, doc, 0, 0, 10, 10, -10, 10)
	upper := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	path := func(x, z, centerZ float64) decad.RigidDriftSegment {
		motion := sweepDrift(r3.Vec{X: x, Z: z}, .1)
		motion.Center = r3.Vec{X: 5, Y: 5, Z: centerZ}
		motion.AngularVelocity.Y = units.RadiansPerSecond(6)
		return motion
	}
	lowerPath, upperPath := path(20, 0, -5), path(80, 50, 5)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	req.TimeResolution = units.Seconds(1e-6)
	req.MaxPoseEvaluations = 128
	report, err := doc.SweepPair(t.Context(), lower, upper, lowerPath, upperPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, report.Outcome)
	require.NotNil(t, report.InitialEvent)
	require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
	require.NotNil(t, report.Departure)
	require.Equal(t, .025, report.Departure.Until.Elapsed.Value.Base())
	require.InDelta(t, 1.347831825490874, report.Departure.GapAtUntil.Value.Base(),
		report.Departure.GapAtUntil.Bound.Base()+1e-9)
	require.Greater(t, report.Departure.GapAtUntil.Value.Base()-
		report.Departure.GapAtUntil.Bound.Base(), 0.0)
	require.Equal(t, decad.ContactSeparated, report.Samples[len(report.Samples)-1].Ideal.Relation)

	upperPath.AngularVelocity.Y = units.RadiansPerSecond(-6)
	undecided, err := doc.SweepPair(t.Context(), lower, upper, lowerPath, upperPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, undecided.Outcome)
	require.Equal(t, decad.SweepDepartureUnproved, undecided.Cause)
}
