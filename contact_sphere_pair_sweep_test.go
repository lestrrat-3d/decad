package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepPairRotatingSourceSpheresDepart(t *testing.T) {
	doc := decad.New()
	a, b := ballBody(t, doc, 5), ballBody(t, doc, 5)
	before := doc.Bodies()
	fromA := contactPose(t, r3.Vec{X: -5})
	fromB := contactPose(t, r3.Vec{X: 5})
	contact, err := doc.ContactPair(t.Context(), a, b, fromA, fromB, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)

	// These are the sliding-Coulomb response velocities for 1 kg spheres with
	// 10 kg·mm² inertia, e=0.5, μ=0.02, and initial velocities (50,20),(-50,0).
	pathA := sweepDrift(r3.Vec{X: -25, Y: 18.5}, .1)
	pathB := sweepDrift(r3.Vec{X: 25, Y: 1.5}, .1)
	pathA.From, pathB.From = fromA, fromB
	pathA.Center, pathB.Center = r3.Vec{X: -5}, r3.Vec{X: 5}
	pathA.AngularVelocity.Z = units.RadiansPerSecond(-.75)
	pathB.AngularVelocity.Z = units.RadiansPerSecond(-.75)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch

	for _, order := range []struct {
		name         string
		a, b         *decad.Body
		pathA, pathB decad.RigidDriftSegment
		normalX      float64
	}{{"forward", a, b, pathA, pathB, 1}, {"reverse", b, a, pathB, pathA, -1}} {
		t.Run(order.name, func(t *testing.T) {
			report, err := doc.SweepPair(t.Context(), order.a, order.b, order.pathA, order.pathB, req)
			require.NoError(t, err)
			require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
			require.True(t, report.HasAffineReplayProof())
			require.NotNil(t, report.InitialEvent)
			require.Equal(t, order.normalX, report.InitialEvent.Manifold.Points[0].Normal.Value.X)
			require.Positive(t, report.Departure.GapAtUntil.Value.Base())
			for _, elapsed := range []float64{.025, .05, .1} {
				poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
				require.NoError(t, err)
				require.NotEqual(t, order.pathA.From.Basis(), poseA.Basis())
				require.NotEqual(t, order.pathB.From.Basis(), poseB.Basis())
				observed, err := doc.ContactPair(t.Context(), order.a, order.b,
					poseA, poseB, contactRequest())
				require.NoError(t, err)
				require.Equal(t, decad.ContactSeparated, observed.Relation)
				require.Positive(t, observed.Gap.Value.Base()-observed.Gap.Bound.Base())
			}
		})
	}
	require.Equal(t, before, doc.Bodies())

	t.Run("off-center pivot", func(t *testing.T) {
		bad := pathA
		bad.Center.X = -4
		report, err := doc.SweepPair(t.Context(), a, b, bad, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
	t.Run("touching tangent departure", func(t *testing.T) {
		tangentA := sweepDrift(r3.Vec{}, 1)
		tangentB := sweepDrift(r3.Vec{Y: 1}, 1)
		tangentB.From = contactPose(t, r3.Vec{X: 10})
		tangentB.Center = r3.Vec{X: 10}
		tangentB.AngularVelocity.Z = units.RadiansPerSecond(-.75)
		report, err := doc.SweepPair(t.Context(), a, b, tangentA, tangentB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
		_, _, err = report.CertifiedPosesAt(units.Seconds(.5))
		require.NoError(t, err)
	})
	t.Run("closing touch", func(t *testing.T) {
		closingA := pathA
		closingA.LinearVelocity.X = units.MillimetersPerSecond(50)
		report, err := doc.SweepPair(t.Context(), a, b, closingA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
	t.Run("separated rotating graze", func(t *testing.T) {
		grazeA := sweepDrift(r3.Vec{}, 1)
		grazeB := sweepDrift(r3.Vec{X: -40}, 1)
		grazeB.From = contactPose(t, r3.Vec{X: 20, Y: 10})
		grazeB.Center = r3.Vec{X: 20, Y: 10}
		grazeB.AngularVelocity.Z = units.RadiansPerSecond(-.75)
		report, err := doc.SweepPair(t.Context(), a, b, grazeA, grazeB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
}

func TestSweepPairRotatingSourceSpheresClear(t *testing.T) {
	doc := decad.New()
	b, c := ballBody(t, doc, 5), ballBody(t, doc, 5)
	pathB := sweepDrift(r3.Vec{X: 7.1875, Y: .625}, .1)
	pathC := sweepDrift(r3.Vec{X: .625, Y: 7.1875}, .1)
	pathB.From = contactPose(t, r3.Vec{X: 10})
	pathC.From = contactPose(t, r3.Vec{Y: 10})
	pathB.Center = r3.Vec{X: 10}
	pathC.Center = r3.Vec{Y: 10}
	pathB.AngularVelocity.Z = units.RadiansPerSecond(-.3125)
	pathC.AngularVelocity.Z = units.RadiansPerSecond(.3125)
	req := sweepRequest()
	req.StartPolicy = decad.StopAtInitialContact

	initial, err := doc.ContactPair(t.Context(), b, c, pathB.From, pathC.From, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, initial.Relation)
	report, err := doc.SweepPair(t.Context(), b, c, pathB, pathC, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, report.Outcome, "cause=%v", report.Cause)
	require.True(t, report.HasAffineReplayProof())
	for _, elapsed := range []float64{0, .025, .1} {
		poseB, poseC, replayErr := report.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, replayErr, "time=%v", elapsed)
		observed, contactErr := doc.ContactPair(t.Context(), b, c, poseB, poseC, contactRequest())
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactSeparated, observed.Relation)
		require.Positive(t, observed.Gap.Value.Base()-observed.Gap.Bound.Base())
	}
	intervalB, intervalC, err := report.CertifiedPosesAtInterval(
		units.Seconds(.05), units.Seconds(0), units.Seconds(.1))
	require.NoError(t, err)
	intervalContact, err := doc.ContactPair(t.Context(), b, c, intervalB, intervalC, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, intervalContact.Relation)

	reversed, err := doc.SweepPair(t.Context(), c, b, pathC, pathB, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, reversed.Outcome)
	_, _, err = reversed.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)

	t.Run("off-center pivot", func(t *testing.T) {
		bad := pathB
		bad.Center.X = 11
		undecided, err := doc.SweepPair(t.Context(), b, c, bad, pathC, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, undecided.Outcome)
		require.False(t, undecided.HasAffineReplayProof())
	})
	t.Run("possible impact", func(t *testing.T) {
		closing := pathB
		closing.LinearVelocity.X = units.MillimetersPerSecond(-40)
		closing.LinearVelocity.Y = units.MillimetersPerSecond(40)
		undecided, err := doc.SweepPair(t.Context(), b, c, closing, pathC, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, undecided.Outcome)
		require.False(t, undecided.HasAffineReplayProof())
	})
}
