package dynamics_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var (
	underflowForceUnit = units.Define("decad-test-force-underflow", units.Force, 1e-200)
	underflowAccelUnit = units.Define("decad-test-acceleration-underflow", units.Acceleration, 1e-200)
)

func testForce(x, z float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.KilogramMillimetersPerSecondSquared(x),
		Y: units.KilogramMillimetersPerSecondSquared(0),
		Z: units.KilogramMillimetersPerSecondSquared(z)}
}

func testTorque(z float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.KilogramSquareMillimetersPerSecondSquared(0),
		Y: units.KilogramSquareMillimetersPerSecondSquared(0),
		Z: units.KilogramSquareMillimetersPerSecondSquared(z)}
}

func TestStepValidatesCenterLoadsBeforeCancellation(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	foreign := makeBox(t, decad.New(), -5, -5, 5, 5, 0, 10)
	good := dynamics.BodyLoad{Body: box, Force: testForce(0, -500), Torque: testTorque(0)}
	for _, tc := range []struct {
		name  string
		loads []dynamics.BodyLoad
		want  error
	}{
		{"nil body", []dynamics.BodyLoad{{Force: good.Force, Torque: good.Torque}}, dynamics.ErrInvalidInput},
		{"foreign body", []dynamics.BodyLoad{{Body: foreign, Force: good.Force, Torque: good.Torque}},
			dynamics.ErrInvalidInput},
		{"fixed body", []dynamics.BodyLoad{{Body: floor, Force: good.Force, Torque: good.Torque}},
			dynamics.ErrInvalidInput},
		{"duplicate body", []dynamics.BodyLoad{good, good}, dynamics.ErrInvalidInput},
		{"wrong force kind", []dynamics.BodyLoad{{Body: box, Force: zeroVelocity(), Torque: good.Torque}},
			dynamics.ErrInvalidInput},
		{"wrong torque kind", []dynamics.BodyLoad{{Body: box, Force: good.Force, Torque: zeroVelocity()}},
			dynamics.ErrInvalidInput},
		{"nonfinite force", []dynamics.BodyLoad{{Body: box, Force: testForce(math.NaN(), 0),
			Torque: good.Torque}}, dynamics.ErrInvalidInput},
		{"nonfinite torque", []dynamics.BodyLoad{{Body: box, Force: good.Force,
			Torque: testTorque(math.Inf(1))}}, dynamics.ErrInvalidInput},
		{"nonzero torque", []dynamics.BodyLoad{{Body: box, Force: good.Force, Torque: testTorque(1)}},
			dynamics.ErrUnsupported},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			report, err := w.Step(ctx, start, dynamics.StepInput{
				Gravity: zeroAcceleration(), Loads: tc.loads}, units.Seconds(0.1))
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, report)
		})
	}
}

func TestCenterForceUsesMassIntervalForKick(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	mass.Mass.Bound = units.Kilograms(0.01)
	mass.Mass.Exactness = decad.Approximate
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	for _, force := range []float64{-500, 500} {
		report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
			Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, force), Torque: testTorque(0)}}},
			units.Seconds(0.2))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, report.Status, "force %g", force)
		require.Nil(t, report.Next)
		require.Contains(t, report.Diagnostics[0].Reason, "force kick")
	}
}

func TestUnderflowedForceAndGravityStillCheckKick(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-100),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	force := dynamics.QuantityVec{X: units.KilogramMillimetersPerSecondSquared(0),
		Y: units.KilogramMillimetersPerSecondSquared(0), Z: units.New(-1e-200, underflowForceUnit)}
	gravity := zeroAcceleration()
	gravity.Z = units.New(-1e-200, underflowAccelUnit)
	require.Zero(t, force.Z.Base())
	require.Zero(t, gravity.Z.Base())
	for _, input := range []dynamics.StepInput{
		{Gravity: zeroAcceleration(), Loads: []dynamics.BodyLoad{{Body: box, Force: force,
			Torque: testTorque(0)}}},
		{Gravity: gravity},
	} {
		report, err := w.Step(t.Context(), start, input, units.Seconds(1e308))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, report.Status)
		require.Nil(t, report.Next)
		require.Contains(t, report.Diagnostics[0].Reason, "force kick")
	}
}

func TestCenterForceClearDriftAndGravityCancellation(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(100, 0), Torque: testTorque(0)}}},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Impulse, report.Conservation.LoadImpulse.Value.X.Kind())
	require.Equal(t, units.Impulse, report.Conservation.LoadImpulse.Bound.X.Kind())
	require.InDelta(t, 10, report.Conservation.LoadImpulse.Value.X.Base(), 1e-9)
	require.InDelta(t, 50, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-9)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 10, final.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, 1, final.Pose.Translation().X, 1e-9)

	gravity := zeroAcceleration()
	gravity.Z = units.MillimetersPerSecondSquared(-1000)
	report, err = w.Step(t.Context(), start, dynamics.StepInput{Gravity: gravity,
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 1000), Torque: testTorque(0)}}},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.Equal(t, start.Entries(), report.Next.Entries())
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -100, report.Conservation.GravityImpulse.Value.Z.Base(), 1e-9)
	require.InDelta(t, 100, report.Conservation.LoadImpulse.Value.Z.Base(), 1e-9)
	require.Zero(t, report.Conservation.AfterKick.LinearMomentum.Value.Z.Base())
}
