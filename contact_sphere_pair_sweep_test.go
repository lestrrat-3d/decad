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
