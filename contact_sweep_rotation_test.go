package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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
