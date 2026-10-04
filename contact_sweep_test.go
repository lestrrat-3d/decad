package decad_test

import (
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
	floatPoint := report.Samples[len(report.Samples)-1].FloatContact.Manifold.Points[0]
	idealPoint := report.Event.Manifold.Points[0]
	require.Equal(t, floatPoint.OnA.Value, idealPoint.OnA.Value)
	require.Equal(t, floatPoint.OnB.Value, idealPoint.OnB.Value)
	require.GreaterOrEqual(t, idealPoint.OnA.Bound.Base(), floatPoint.OnA.Bound.Base())
	require.GreaterOrEqual(t, idealPoint.OnB.Bound.Base(), floatPoint.OnB.Bound.Base())
	require.Equal(t, before, doc.Bodies())
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

func TestSweepPairRotatingPathUndecided(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 30, 0, 40, 10, 10)
	rotating := sweepDrift(r3.Vec{}, 0.1)
	rotating.AngularVelocity.Z = units.RadiansPerSecond(1)
	report, err := doc.SweepPair(t.Context(), a, b,
		rotating, sweepDrift(r3.Vec{}, 0.1), sweepRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, report.Outcome)
	require.Equal(t, decad.SweepContactUnsupported, report.Cause)
}
