package decad_test

import (
	"math"
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
	clipped, err := doc.ContactPair(t.Context(), narrow, box, r3.Identity(), turn, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, clipped.Relation)
	require.NotNil(t, clipped.Manifold)
	require.Len(t, clipped.Manifold.Points, 8)
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
			// At 45 degrees the turned box pokes its vertical edge at
			// x = 5 + 5·√2 through b's x = 12 face: §9.3's shallow
			// penetration publishes that edge's two ends, each paired with
			// its foot on the face, at depth 5 + 5·√2 − 12.
			require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
			require.Len(t, report.Manifold.Points, 2)
			depth := 5 + 5*math.Sqrt2 - 12
			for _, point := range report.Manifold.Points {
				require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
				require.InDelta(t, 12+depth, point.OnA.Value.X, point.OnA.Bound.Base()+1e-12)
				require.Equal(t, 12.0, point.OnB.Value.X)
				require.Equal(t, point.OnA.Value.Z, point.OnB.Value.Z)
				require.InDelta(t, -depth, point.Separation.Value.Base(), point.Separation.Bound.Base()+1e-12)
				require.NotNil(t, point.FeatureA.Edge)
				require.NotNil(t, point.FeatureB.Face)
			}
			require.ElementsMatch(t, []float64{0, 10},
				[]float64{report.Manifold.Points[0].OnA.Value.Z, report.Manifold.Points[1].OnA.Value.Z})
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
	require.InDelta(t, 19.0001, point.OnA.Value.X, 1e-10)
	require.InDelta(t, 19, point.OnB.Value.X, 1e-10)
	require.InDelta(t, 5, point.OnA.Value.Y, 1e-9)
	require.InDelta(t, 5, point.OnA.Value.Z, 1e-9)
	require.NotNil(t, point.FaceA)
	require.NotNil(t, point.FaceB)

	reverse, err := doc.ContactPair(t.Context(), box, driver, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, reverse.Relation)
	require.NotNil(t, reverse.Manifold)
	require.Equal(t, r3.Vec{X: -1}, reverse.Manifold.Points[0].Normal.Value)
	require.InDelta(t, -.0001, reverse.Manifold.Points[0].Separation.Value.Base(), 1e-10)
	require.InDelta(t, 19, reverse.Manifold.Points[0].OnA.Value.X, 1e-10)
	require.InDelta(t, 19.0001, reverse.Manifold.Points[0].OnB.Value.X, 1e-10)

	// The first face center is outside this smaller face, so the witness
	// must come from the second face center and keep the same exact X gap.
	small := boxBodyAtZ(t, doc, 19, 7, 29, 9, 4, 2)
	fallback, err := doc.ContactPair(t.Context(), driver, small, pose, r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, fallback.Relation)
	require.NotNil(t, fallback.Manifold)
	require.Len(t, fallback.Manifold.Points, 1)
	require.InDelta(t, -.0001, fallback.Manifold.Points[0].Separation.Value.Base(), 1e-10)
	require.InDelta(t, 8, fallback.Manifold.Points[0].OnA.Value.Y, 1e-9)
	require.InDelta(t, 5, fallback.Manifold.Points[0].OnA.Value.Z, 1e-9)
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
	require.False(t, report.HasAffineReplayProof())
	replayedA, replayedB, replayErr := report.CertifiedPosesAtInterval(units.Seconds(.2),
		units.Seconds(0), units.Seconds(1))
	require.NoError(t, replayErr)
	contact, err := doc.ContactPair(t.Context(), a, b, replayedA, replayedB, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.NotNil(t, report.Bracket)
	bracketMiddle := (report.Bracket.From.Fraction.Base() + report.Bracket.To.Fraction.Base()) / 2
	_, _, replayErr = report.CertifiedPosesAt(units.Seconds(bracketMiddle))
	require.ErrorIs(t, replayErr, decad.ErrUnsupported)
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
	replayedA, replayedB, err := report.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	replayedContact, err := doc.ContactPair(t.Context(), lower, upper, replayedA, replayedB, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, replayedContact.Relation)

	upperPath.AngularVelocity.Y = units.RadiansPerSecond(-6)
	undecided, err := doc.SweepPair(t.Context(), lower, upper, lowerPath, upperPath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, undecided.Outcome)
	require.Equal(t, decad.SweepDepartureUnproved, undecided.Cause)
	_, _, err = undecided.CertifiedPosesAt(units.Seconds(.05))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSweepPairFixedFloorTangentialSpinDeparts(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, 0, -5, 10, 5, -10, 10)
	box := boxBodyAtZ(t, doc, 5, -5, 15, 5, 0, 10)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)

	path := sweepDrift(r3.Vec{Z: 100}, .1)
	path.Center = r3.Vec{X: 10, Z: 5}
	path.AngularVelocity.Y = units.RadiansPerSecond(1)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	req.MaxPoseEvaluations = 128
	report, err := doc.SweepPair(t.Context(), floor, box, sweepDrift(r3.Vec{}, .1), path, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.Departure)
	require.Equal(t, .1, report.Departure.Until.Elapsed.Value.Base())
	require.Greater(t, report.Departure.GapAtUntil.Value.Base()-report.Departure.GapAtUntil.Bound.Base(), 0.0)
	for _, elapsed := range []float64{1e-8, .025, .1} {
		poseFloor, poseBox, replayErr := report.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, replayErr, "time=%v", elapsed)
		separated, contactErr := doc.ContactPair(t.Context(), floor, box, poseFloor, poseBox, contactRequest())
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactSeparated, separated.Relation, "time=%v", elapsed)
	}

	reversed, err := doc.SweepPair(t.Context(), box, floor, path, sweepDrift(r3.Vec{}, .1), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, reversed.Outcome, "cause=%v", reversed.Cause)
	require.NotNil(t, reversed.Departure)
	_, _, err = reversed.CertifiedPosesAt(units.Seconds(.025))
	require.NoError(t, err)

	path.LinearVelocity.Z = units.MillimetersPerSecond(6)
	shortHorizon, err := doc.SweepPair(t.Context(), floor, box, sweepDrift(r3.Vec{}, .1), path, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, shortHorizon.Outcome, "cause=%v", shortHorizon.Cause)
	require.Equal(t, .05, shortHorizon.Departure.Until.Elapsed.Value.Base())
	_, _, err = shortHorizon.CertifiedPosesAt(units.Seconds(.075))
	require.NoError(t, err)

	// At 5 mm/s the corners over x = 15 hold their height, so the box proof
	// against the floor face fails. The general planar proof
	// (docs/multibody-dynamics-design.md §10.2) takes the box's lower face as
	// the support plane instead, and every floor vertex falls away from it.
	path.LinearVelocity.Z = units.MillimetersPerSecond(5)
	tooSlow, err := doc.SweepPair(t.Context(), floor, box, sweepDrift(r3.Vec{}, .1), path, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, tooSlow.Outcome, "cause=%v", tooSlow.Cause)
	path.LinearVelocity.Z = units.MillimetersPerSecond(100)
	path.AngularVelocity.Y = units.RadiansPerSecond(30)
	// A fast spin fails the box proof too; against the box's lower face the
	// general proof departs, over a horizon its 900 rad²/s² curvature keeps
	// short.
	fastSpin, err := doc.SweepPair(t.Context(), floor, box, sweepDrift(r3.Vec{}, .1), path, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, fastSpin.Outcome, "cause=%v", fastSpin.Cause)
	require.Less(t, fastSpin.Departure.Until.Elapsed.Value.Base(), .01)
	path.AngularVelocity.Y = units.RadiansPerSecond(1)
	path.Center.X = -100
	badPivot, err := doc.SweepPair(t.Context(), floor, box, sweepDrift(r3.Vec{}, .1), path, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, badPivot.Outcome)
	require.Equal(t, decad.SweepDepartureUnproved, badPivot.Cause)
}
