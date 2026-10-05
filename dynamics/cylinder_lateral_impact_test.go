package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCylinderLateralFloorImpactUsesProductionSweep(t *testing.T) {
	for _, revolved := range []bool{false, true} {
		for _, reverse := range []bool{false, true} {
			name := "extruded"
			if revolved {
				name = "revolved"
			}
			if reverse {
				name += " cylinder first"
			} else {
				name += " floor first"
			}
			t.Run(name, func(t *testing.T) {
				cylinderLateralFloorImpact(t, revolved, reverse)
			})
		}
	}
}

func cylinderLateralFloorImpact(t *testing.T, revolved, reverse bool) {
	t.Helper()
	doc := decad.New()
	var floor, cylinder *decad.Body
	var start r3.Vec
	var incoming decad.QuantityVec
	if revolved {
		floor = makeBox(t, doc, -10, -20, 0, 20, -20, 40)
		cylinder = makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 0)
		start = r3.Vec{X: 1}
		incoming = decad.QuantityVec{X: units.MillimetersPerSecond(-10),
			Y: units.MillimetersPerSecond(1), Z: units.MillimetersPerSecond(0)}
	} else {
		floor = makeBox(t, doc, -20, -20, 20, 20, -10, 10)
		cylinder = makeCylinder(t, doc)
		start = r3.Vec{Z: 1}
		incoming = decad.QuantityVec{X: units.MillimetersPerSecond(1),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-10)}
	}
	pose, err := r3.Translation(start)
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	zero := zeroAngular(t)
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.2)}
	moving := decad.RigidDriftSegment{From: pose, LinearVelocity: incoming,
		AngularVelocity: zero, Duration: units.Seconds(.2)}
	a, b := floor, cylinder
	poseA, poseB := r3.Identity(), pose
	pathA, pathB := decad.PairPath(still), decad.PairPath(moving)
	if reverse {
		a, b, poseA, poseB = b, a, poseB, poseA
		pathA, pathB = pathB, pathA
	}
	contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.True(t, sweep.HasAffineReplayProof())
	require.Len(t, sweep.Event.Manifold.Points, 1)
	require.InDelta(t, .1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	_, _, err = sweep.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	_, _, err = sweep.CertifiedPosesAt(units.Seconds(.15))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := cylinder.MassProperties(t.Context(), density)
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	defs := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: material}}
	states := []dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: cylinder, Pose: pose, LinearVelocity: incoming, AngularVelocity: zero}}
	if reverse {
		defs[0], defs[1] = defs[1], defs[0]
		states[0], states[1] = states[1], states[0]
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: defs,
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-5),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096}})
	require.NoError(t, err)
	state, err := world.NewState(states)
	require.NoError(t, err)
	step, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, mass.Mass.Value.Base()*15, step.Events[0].NormalImpulse.Base(), 1e-4)
	for _, sample := range []struct {
		time, axial, lateral, normalSpeed float64
	}{
		{time: .05, axial: .5, lateral: .05, normalSpeed: -10},
		{time: .15, axial: .25, lateral: .15, normalSpeed: 5},
		{time: .2, axial: .5, lateral: .2, normalSpeed: 5},
	} {
		at, err := step.Trace.Sample(units.Seconds(sample.time))
		require.NoError(t, err)
		body, ok := at.Body(cylinder)
		require.True(t, ok)
		if revolved {
			require.InDelta(t, sample.axial, body.Pose.Translation().X, 1e-6)
			require.InDelta(t, sample.lateral, body.Pose.Translation().Y, 1e-9)
			require.InDelta(t, sample.normalSpeed, body.LinearVelocity.X.Base(), 1e-6)
			require.Equal(t, 1.0, body.LinearVelocity.Y.Base())
		} else {
			require.InDelta(t, sample.axial, body.Pose.Translation().Z, 1e-6)
			require.InDelta(t, sample.lateral, body.Pose.Translation().X, 1e-9)
			require.InDelta(t, sample.normalSpeed, body.LinearVelocity.Z.Base(), 1e-6)
			require.Equal(t, 1.0, body.LinearVelocity.X.Base())
		}
	}
}
