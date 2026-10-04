package decad_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sweepDrift(v r3.Vec, seconds float64) decad.RigidDriftSegment {
	return decad.RigidDriftSegment{
		From: r3.Identity(),
		LinearVelocity: decad.QuantityVec{
			X: units.MillimetersPerSecond(v.X),
			Y: units.MillimetersPerSecond(v.Y),
			Z: units.MillimetersPerSecond(v.Z),
		},
		AngularVelocity: decad.QuantityVec{
			X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0),
			Z: units.RadiansPerSecond(0),
		},
		Duration: units.Seconds(seconds),
	}
}

func sweepRequest() decad.SweepRequest {
	return decad.SweepRequest{
		ContactRequest:     contactRequest(),
		TimeResolution:     units.Seconds(1e-9),
		MaxPoseEvaluations: 8,
	}
}

func TestSweepPairTranslatedBoxesImpact(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 20, 0, 30, 10, 10)
	before := doc.Bodies()
	report, err := doc.SweepPair(t.Context(), a, b,
		sweepDrift(r3.Vec{X: 100}, 0.2), sweepDrift(r3.Vec{}, 0.2), sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	require.Same(t, a, report.A)
	require.Same(t, b, report.B)
	require.NotNil(t, report.Bracket)
	left := report.Bracket.From.Elapsed.Value.Base()
	right := report.Bracket.To.Elapsed.Value.Base()
	require.Less(t, left, 0.1)
	require.Greater(t, right, 0.1)
	require.LessOrEqual(t, right-left, 1e-9)
	require.Equal(t, decad.ContactSeparated, report.Samples[1].Ideal.Relation)
	require.NotNil(t, report.Event)
	require.NotNil(t, report.Event.Manifold)
	require.NotEmpty(t, report.Event.Manifold.Points)
	require.Equal(t, r3.Vec{X: 1}, report.Event.Manifold.Points[0].Normal.Value)
	replayA, replayB, err := report.CertifiedPosesAt(units.Seconds(0.05))
	require.NoError(t, err)
	replayContact, err := doc.ContactPair(t.Context(), a, b, replayA, replayB, sweepRequest().ContactRequest)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, replayContact.Relation)
	_, _, err = report.CertifiedPosesAt(units.Seconds(0.15))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	floatPoint := report.Samples[len(report.Samples)-1].FloatContact.Manifold.Points[0]
	idealPoint := report.Event.Manifold.Points[0]
	require.Equal(t, floatPoint.OnA.Value, idealPoint.OnA.Value)
	require.Equal(t, floatPoint.OnB.Value, idealPoint.OnB.Value)
	require.GreaterOrEqual(t, idealPoint.OnA.Bound.Base(), floatPoint.OnA.Bound.Base())
	require.GreaterOrEqual(t, idealPoint.OnB.Bound.Base(), floatPoint.OnB.Bound.Base())
	require.Equal(t, before, doc.Bodies())

	t.Run("rounded interior pose", func(t *testing.T) {
		large := decad.New()
		moving := boxBody(t, large, 0, 0, 10, 10, 10)
		far := boxBody(t, large, 100, 0, 110, 10, 10)
		from, err := r3.Translation(r3.Vec{X: 1e16})
		require.NoError(t, err)
		to, err := r3.Translation(r3.Vec{X: 1e16 + 2})
		require.NoError(t, err)
		request := sweepRequest()
		request.PointResolution = units.Millimeters(1e-6)
		clearReport, err := large.SweepPair(t.Context(), moving, far,
			decad.PoseSegment{From: from, To: to, Duration: units.Seconds(1)},
			decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}, request)
		require.NoError(t, err)
		require.Equal(t, decad.SweepClear, clearReport.Outcome)
		_, _, err = clearReport.CertifiedPosesAt(units.Seconds(.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("deep grid bracket", func(t *testing.T) {
		deep := decad.New()
		moving := boxBody(t, deep, 0, 0, 10, 10, 10)
		stationary := boxBody(t, deep, 20, 0, 30, 10, 10)
		request := sweepRequest()
		request.TimeResolution = units.Seconds(math.Ldexp(1, -55))
		report, err := deep.SweepPair(t.Context(), moving, stationary,
			sweepDrift(r3.Vec{X: 30}, 1), sweepDrift(r3.Vec{}, 1), request)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome)
		require.NotNil(t, report.Bracket)
		require.False(t, report.BracketEndsAtDuration())
		grid := new(big.Int).Lsh(big.NewInt(1), 56)
		floor := new(big.Int).Quo(grid, big.NewInt(3))
		trueRight := new(big.Rat).SetFrac(new(big.Int).Add(floor, big.NewInt(2)), grid)
		published := new(big.Rat).SetFloat64(report.Bracket.To.Fraction.Base())
		require.Positive(t, published.Cmp(trueRight))
		_, _, err = report.CertifiedPosesAt(report.Bracket.To.Elapsed.Value)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

func TestSweepPairOffCenterBoxImpactAtEndpoint(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -5, -5, 5-1e-6, 5, -10, 20)
	box := boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 10)
	from, err := r3.Translation(r3.Vec{Z: 20})
	require.NoError(t, err)
	fall := sweepDrift(r3.Vec{Z: -80}, 0.125)
	fall.From = from
	report, err := doc.SweepPair(t.Context(), floor, box,
		sweepDrift(r3.Vec{}, 0.125), fall, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	require.NotNil(t, report.Bracket)
	require.True(t, report.BracketEndsAtDuration())
	require.Less(t, report.Bracket.From.Fraction.Base(), 1.0)
	require.Equal(t, 1.0, report.Bracket.To.Fraction.Base())
	require.LessOrEqual(t, report.Bracket.To.Elapsed.Value.Base()-
		report.Bracket.From.Elapsed.Value.Base(), 1e-9)
	require.Equal(t, decad.ContactSeparated, report.Samples[0].Ideal.Relation)
	require.Equal(t, decad.ContactTouching, report.Event.Relation)
	require.NotNil(t, report.Event.Manifold)
	require.NotEmpty(t, report.Event.Manifold.Points)
	require.Equal(t, r3.Vec{Z: 1}, report.Event.Manifold.Points[0].Normal.Value)

	// The read 0.1 s duration makes the ideal drift overlap by less than one
	// pose ULP while ContactPair sees a touching rounded endpoint. The exact
	// source-box manifold still bounds the ideal penetration.
	fall = sweepDrift(r3.Vec{Z: -100}, 0.1)
	fall.From = from
	near, err := doc.SweepPair(t.Context(), floor, box,
		sweepDrift(r3.Vec{}, 0.1), fall, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, near.Outcome)
	require.Equal(t, 1.0, near.Bracket.To.Fraction.Base())
	require.Equal(t, decad.ContactOverlapping, near.Event.Relation)
	require.Equal(t, decad.ContactTouching, near.Samples[len(near.Samples)-1].FloatContact.Relation)
	require.NotNil(t, near.Event.Manifold)
	require.Len(t, near.Event.Manifold.Points, 4)
	point := near.Event.Manifold.Points[0]
	require.Less(t, point.Separation.Value.Base(), 0.0)
	require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
	depth := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.1), big.NewRat(-100, 1))
	depth.Add(depth, big.NewRat(10, 1))
	errorValue := new(big.Rat).Sub(depth, new(big.Rat).SetFloat64(point.Separation.Value.Base()))
	errorValue.Abs(errorValue)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(point.Separation.Bound.Base()).Cmp(errorValue), 0)
}

func TestSweepPairSourceSphereAndBox(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	ball := ballBody(t, doc, 5)
	before := doc.Bodies()
	still := sweepDrift(r3.Vec{}, 0.2)
	drop := sweepDrift(r3.Vec{Z: -100}, 0.2)
	drop.From = contactPose(t, r3.Vec{Z: 15})
	req := sweepRequest()
	report, err := doc.SweepPair(t.Context(), floor, ball, still, drop, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.Event)
	require.NotNil(t, report.Event.Manifold)
	require.Len(t, report.Event.Manifold.Points, 1)
	require.Equal(t, r3.Vec{Z: 1}, report.Event.Manifold.Points[0].Normal.Value)
	require.Less(t, report.Bracket.From.Elapsed.Value.Base(), 0.1)
	require.GreaterOrEqual(t, report.Bracket.To.Elapsed.Value.Base(), 0.1)
	require.LessOrEqual(t,
		report.Bracket.To.Elapsed.Value.Base()-report.Bracket.From.Elapsed.Value.Base(), 1e-9)
	require.True(t, report.HasAffineReplayProof())
	beforeA, beforeB, err := report.CertifiedPosesAtInterval(units.Milliseconds(50),
		units.Seconds(0), units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, r3.Identity(), beforeA)
	require.InDelta(t, 10, beforeB.Translation().Z, 1e-12)
	reversed, err := doc.SweepPair(t.Context(), ball, floor, drop, still, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, reversed.Outcome, "cause=%v", reversed.Cause)
	require.Equal(t, r3.Vec{Z: -1}, reversed.Event.Manifold.Points[0].Normal.Value)
	reverseA, reverseB, err := reversed.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	require.InDelta(t, 10, reverseA.Translation().Z, 1e-12)
	require.Equal(t, r3.Identity(), reverseB)
	slow := sweepDrift(r3.Vec{Z: -10}, .2)
	slow.From = drop.From
	clearReport, err := doc.SweepPair(t.Context(), floor, ball, still, slow, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, clearReport.Outcome)
	_, clearBall, err := clearReport.CertifiedPosesAt(units.Seconds(.1))
	require.NoError(t, err)
	require.InDelta(t, 14, clearBall.Translation().Z, 1e-12)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairSourceSpherePair(t *testing.T) {
	doc := decad.New()
	a, b := ballBody(t, doc, 5), ballBody(t, doc, 5)
	before := doc.Bodies()
	approachA := sweepDrift(r3.Vec{X: 50}, 0.2)
	approachB := sweepDrift(r3.Vec{X: -50}, 0.2)
	approachA.From = contactPose(t, r3.Vec{X: -10})
	approachB.From = contactPose(t, r3.Vec{X: 10})
	report, err := doc.SweepPair(t.Context(), a, b, approachA, approachB, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.Event)
	require.NotNil(t, report.Event.Manifold)
	require.Len(t, report.Event.Manifold.Points, 1)
	require.Equal(t, r3.Vec{X: 1}, report.Event.Manifold.Points[0].Normal.Value)
	require.Less(t, report.Bracket.From.Elapsed.Value.Base(), 0.1)
	require.GreaterOrEqual(t, report.Bracket.To.Elapsed.Value.Base(), 0.1)
	require.LessOrEqual(t, report.Bracket.To.Elapsed.Value.Base()-
		report.Bracket.From.Elapsed.Value.Base(), 1e-9)
	reversed, err := doc.SweepPair(t.Context(), b, a, approachB, approachA, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, reversed.Outcome)
	require.Equal(t, r3.Vec{X: -1}, reversed.Event.Manifold.Points[0].Normal.Value)
	passA, passB := approachA, approachB
	passA.Duration, passB.Duration = units.Seconds(0.4), units.Seconds(0.4)
	passThrough, err := doc.SweepPair(t.Context(), a, b, passA, passB, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, passThrough.Outcome, "cause=%v", passThrough.Cause)
	require.InDelta(t, 0.1, passThrough.Bracket.To.Elapsed.Value.Base(), 1e-9)
	poseA := decad.PoseSegment{From: approachA.From, To: contactPose(t, r3.Vec{}),
		Duration: units.Seconds(0.2)}
	poseB := decad.PoseSegment{From: approachB.From, To: contactPose(t, r3.Vec{}),
		Duration: units.Seconds(0.2)}
	posed, err := doc.SweepPair(t.Context(), a, b, poseA, poseB, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, posed.Outcome, "cause=%v", posed.Cause)
	endpointA, endpointB := approachA, approachB
	endpointA.Duration, endpointB.Duration = units.Seconds(0.1), units.Seconds(0.1)
	endpoint, err := doc.SweepPair(t.Context(), a, b, endpointA, endpointB, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, endpoint.Outcome, "cause=%v", endpoint.Cause)
	require.Equal(t, 1.0, endpoint.Bracket.To.Fraction.Base())
	departingA, departingB := sweepDrift(r3.Vec{X: -25}, 0.1), sweepDrift(r3.Vec{X: 25}, 0.1)
	departingA.From = contactPose(t, r3.Vec{X: -5})
	departingB.From = contactPose(t, r3.Vec{X: 5})
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	departed, err := doc.SweepPair(t.Context(), a, b, departingA, departingB, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, departed.Outcome, "cause=%v", departed.Cause)
	lateral := approachB
	lateral.LinearVelocity.Y = units.MillimetersPerSecond(1)
	unsupported, err := doc.SweepPair(t.Context(), a, b, approachA, lateral, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, unsupported.Outcome)
	require.Equal(t, decad.SweepContactUnsupported, unsupported.Cause)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairSourceSphereTouchContinuation(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	ball := ballBody(t, doc, 5)
	still := sweepDrift(r3.Vec{}, 0.1)
	touching := sweepDrift(r3.Vec{}, 0.1)
	touching.From = contactPose(t, r3.Vec{Z: 5})
	req := sweepRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	report, err := doc.SweepPair(t.Context(), floor, ball, still, touching, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.ContactTrack)
	manifold, err := report.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Len(t, manifold.Points, 1)
	require.Equal(t, r3.Vec{Z: 1}, manifold.Points[0].Normal.Value)
	require.Equal(t, 0.0, manifold.Points[0].Separation.Value.Base())
	_, touchBall, err := report.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: 5}, touchBall.Translation())
	departing := touching
	departing.LinearVelocity.Z = units.MillimetersPerSecond(50)
	req.StartPolicy = decad.ContinueSeparatingTouch
	report, err = doc.SweepPair(t.Context(), floor, ball, still, departing, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
	require.NotNil(t, report.Departure)
	_, departedBall, err := report.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	require.InDelta(t, 7.5, departedBall.Translation().Z, 1e-12)
}

func TestSweepPairSourceSphereInitialEdgeTouch(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -10, 10)
	ball := ballBody(t, doc, 5)
	pose := contactPose(t, r3.Vec{X: 25})
	contact, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), pose, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Nil(t, contact.Manifold)
	still := sweepDrift(r3.Vec{}, 0.1)
	ballPath := still
	ballPath.From = pose
	report, err := doc.SweepPair(t.Context(), floor, ball, still, ballPath, sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, report.Outcome)
	require.NotNil(t, report.InitialEvent)
	require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
	require.Nil(t, report.InitialEvent.Manifold)
	require.Len(t, report.Samples, 1)
	require.False(t, report.HasAffineReplayProof())
	_, _, err = report.CertifiedPosesAt(units.Seconds(.05))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSweepPairTwoMoversAndDeparture(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 30, 0, 40, 10, 10)
	report, err := doc.SweepPair(t.Context(), a, b,
		sweepDrift(r3.Vec{X: 100}, 0.2), sweepDrift(r3.Vec{X: -100}, 0.2), sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome)
	require.Less(t, report.Bracket.From.Elapsed.Value.Base(), 0.1)
	require.Greater(t, report.Bracket.To.Elapsed.Value.Base(), 0.1)

	stack := decad.New()
	floor := boxBody(t, stack, 0, 0, 10, 10, 10)
	top := boxBodyAtZ(t, stack, 0, 0, 10, 10, 10, 10)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch
	departed, err := stack.SweepPair(t.Context(), floor, top,
		sweepDrift(r3.Vec{}, 0.1), sweepDrift(r3.Vec{Z: 50}, 0.1), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, departed.Outcome)
	require.NotNil(t, departed.Departure)
	require.InDelta(t, 5, departed.Departure.GapAtUntil.Value.Base(), 1e-12)
}

func TestSweepPairRotatingPathClear(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 30, 0, 40, 10, 10)
	rotating := sweepDrift(r3.Vec{}, 0.1)
	rotating.AngularVelocity.Z = units.RadiansPerSecond(1)
	report, err := doc.SweepPair(t.Context(), a, b,
		rotating, sweepDrift(r3.Vec{}, 0.1), sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, report.Outcome)
	require.Equal(t, decad.SweepNoCause, report.Cause)
	require.GreaterOrEqual(t, report.PoseEvaluations, uint64(2))
}

func TestSweepPairSourceBoxPersistentSlide(t *testing.T) {
	doc := decad.New()
	floor := boxBody(t, doc, 0, -5, 20, 15, 10)
	top := boxBodyAtZ(t, doc, 5, 0, 15, 10, 10, 10)
	before := doc.Bodies()
	initial, err := doc.ContactPair(t.Context(), floor, top, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initial.Relation)
	require.Len(t, initial.Manifold.Points, 4)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	report, err := doc.SweepPair(t.Context(), floor, top,
		sweepDrift(r3.Vec{}, 1), sweepDrift(r3.Vec{X: 1}, 1), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome)
	require.NotNil(t, report.ContactTrack)
	require.Equal(t, 0.0, report.ContactTrack.Start().Fraction.Base())
	require.Equal(t, 1.0, report.ContactTrack.End().Fraction.Base())
	require.Equal(t, r3.Vec{Z: 1}, report.ContactTrack.Normal().Value)
	faceA, faceB := report.ContactTrack.Features()
	require.NotNil(t, faceA.Face)
	require.NotNil(t, faceB.Face)
	for _, sample := range []struct{ fraction, low, high float64 }{
		{0, 5, 15}, {0.5, 5.5, 15.5}, {1, 6, 16},
	} {
		manifold, err := report.ContactTrack.ManifoldAt(units.Scalar(sample.fraction))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 4)
		require.Equal(t, sample.low, manifold.Points[0].OnA.Value.X)
		require.Equal(t, sample.high, manifold.Points[1].OnA.Value.X)
		for _, point := range manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			require.Equal(t, 10.0, point.OnA.Value.Z)
			require.Equal(t, point.OnA.Value, point.OnB.Value)
			require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())
			require.NotNil(t, point.FaceA)
			require.NotNil(t, point.FaceB)
		}
	}
	changed, err := report.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	changed.Points[0].OnA.Value.X = -100
	again, err := report.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Equal(t, 5.5, again.Points[0].OnA.Value.X)
	_, err = report.ContactTrack.ManifoldAt(units.Seconds(0.5))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = report.ContactTrack.ManifoldAt(units.Scalar(1.5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairSourceBoxSlideEdgeTransition(t *testing.T) {
	doc := decad.New()
	floor := boxBody(t, doc, 0, 0, 10, 10, 10)
	top := boxBodyAtZ(t, doc, 0, 0, 10, 10, 10, 10)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueCertifiedTouch
	report, err := doc.SweepPair(t.Context(), floor, top,
		sweepDrift(r3.Vec{}, 3), sweepDrift(r3.Vec{X: 5}, 3), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, report.Outcome)
	require.NotNil(t, report.Bracket)
	require.Less(t, report.Bracket.From.Elapsed.Value.Base(), 2.0)
	require.Greater(t, report.Bracket.To.Elapsed.Value.Base(), 2.0)
	require.LessOrEqual(t, report.Bracket.To.Elapsed.Value.Base()-report.Bracket.From.Elapsed.Value.Base(),
		req.TimeResolution.Base())
	require.NotNil(t, report.ContactTrack)
	require.Equal(t, report.Bracket.From.Fraction, report.ContactTrack.End().Fraction)
	require.NotNil(t, report.Event)
	require.Equal(t, decad.ContactSeparated, report.Event.Relation)
	patch, err := report.ContactTrack.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Len(t, patch.Points, 4)
	require.Equal(t, 7.5, patch.Points[0].OnA.Value.X)
	require.Equal(t, 10.0, patch.Points[1].OnA.Value.X)
	require.Equal(t, r3.Vec{Z: 1}, patch.Points[0].Normal.Value)
}
