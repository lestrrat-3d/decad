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
	underflowSpinUnit  = units.Define("decad-test-angular-underflow", units.AngularVelocity, 1e-200)
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

func TestTorqueKickRotatesClearSourceBox(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(100)}}},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	entry, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0.6, entry.AngularVelocity.Z.Base(), 1e-9)
	require.InDelta(t, math.Cos(0.06), entry.Pose.ApplyDir(r3.Vec{X: 1}).X, 1e-9)
	require.InDelta(t, math.Sin(0.06), entry.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
	middle, err := report.Trace.Sample(units.Seconds(0.05))
	require.NoError(t, err)
	middleBox, ok := middle.Body(box)
	require.True(t, ok)
	require.InDelta(t, math.Cos(0.03), middleBox.Pose.ApplyDir(r3.Vec{X: 1}).X, 1e-9)
	require.InDelta(t, math.Sin(0.03), middleBox.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
	require.InDelta(t, entry.AngularVelocity.Z.Base(), middleBox.AngularVelocity.Z.Base(), 1e-12)
	unchanged, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, entry.Pose, unchanged.Pose)
	end, err := report.Trace.Sample(units.Milliseconds(100))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), end.Entries())
	_, err = report.Trace.Sample(units.Seconds(-0.01))
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
	_, err = report.Trace.Sample(units.Seconds(0.11))
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.AngularMomentum, report.Conservation.TorqueImpulse.Value.Z.Kind())
	require.InDelta(t, 10, report.Conservation.TorqueImpulse.Value.Z.Base(), 1e-9)
	require.InDelta(t, 10, report.Conservation.AfterKick.AngularMomentum.Value.Z.Base(), 1e-8)
	require.InDelta(t, 3, report.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-8)
	require.InDelta(t, 10, report.Conservation.Completion.AngularMomentum.Value.Z.Base(), 1e-6)
	second, err := w.NewState(report.Next.Entries())
	require.NoError(t, err)
	report, err = w.Step(t.Context(), second, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	entry, ok = report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0.6, entry.AngularVelocity.Z.Base(), 1e-9)
	require.InDelta(t, math.Sin(0.12), entry.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
	secondMiddle, err := report.Trace.Sample(units.Milliseconds(50))
	require.NoError(t, err)
	secondBox, ok := secondMiddle.Body(box)
	require.True(t, ok)
	require.InDelta(t, math.Sin(0.09), secondBox.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
}

func TestTorqueTraceReplaysClearRotationInReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material}},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	travel := dynamics.QuantityVec{X: units.MillimetersPerSecond(10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: pose, LinearVelocity: travel, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(100)}}},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	middle, err := report.Trace.Sample(units.Milliseconds(50))
	require.NoError(t, err)
	entry, ok := middle.Body(box)
	require.True(t, ok)
	require.InDelta(t, math.Cos(0.03), entry.Pose.ApplyDir(r3.Vec{X: 1}).X, 1e-9)
	require.InDelta(t, math.Sin(0.03), entry.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
	require.InDelta(t, 0.5, entry.Pose.Apply(r3.Vec{X: 0, Y: 0, Z: 5}).X, 1e-9)
	fixed, ok := middle.Body(floor)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), fixed.Pose)
	endpoint, err := report.Trace.Sample(units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), endpoint.Entries())
}

func TestTorqueDrivenRotatingBoxReboundsFromFixedFloor(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-100)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	sweep, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(0.2)},
		decad.RigidDriftSegment{From: pose, Center: pose.Apply(mass.Center.Value),
			LinearVelocity: fall, AngularVelocity: dynamics.QuantityVec{
				X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
				Z: units.RadiansPerSecond(0.6)}, Duration: units.Seconds(0.2)},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128, StartPolicy: decad.StopAtInitialContact})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.NotNil(t, sweep.Event)
	require.NotNil(t, sweep.Event.Manifold)
	require.Len(t, sweep.Event.Manifold.Points, 4)
	require.InDelta(t, 0.1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(50)}}},
		units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.InDelta(t, 0.1, event.Time.Base(), 1e-9)
	require.InDelta(t, 50, event.PostVelocity.Z.Base(), 1e-6)
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	require.InDelta(t, 0.6, event.PreAngularVelocityB.Z.Base(), 1e-9)
	require.InDelta(t, 0.6, event.PostAngularVelocityB.Z.Base(), 1e-9)
	require.True(t, event.PoseA.IsValid())
	require.True(t, event.PoseB.IsValid())
	end, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0.6, end.AngularVelocity.Z.Base(), 1e-9)
	require.InDelta(t, 5, end.Pose.Translation().Z, 2e-6)
	require.InDelta(t, math.Sin(0.12), end.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 10, report.Conservation.TorqueImpulse.Value.Z.Base(), 1e-8)
	require.InDelta(t, 10, report.Conservation.Completion.AngularMomentum.Value.Z.Base(), 1e-6)
	replayed, err := report.Trace.Sample(units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	second, err := w.NewState(report.Next.Entries())
	require.NoError(t, err)
	next, err := w.Step(t.Context(), second, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, next.Status, "%+v", next.Diagnostics)
	continued, ok := next.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 10, continued.Pose.Translation().Z, 2e-6)
	require.InDelta(t, math.Sin(0.18), continued.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
}

func TestTorqueDrivenRotatingBoxRestingContactNeedsTrack(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-100)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(50)}}},
		units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
}

func TestTorqueDrivenRotatingImpactUsesCertifiedNonHalfPose(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	pose, err := r3.Translation(r3.Vec{Z: 7})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-100)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(50)}}},
		units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.07, report.Events[0].Time.Base(), 1e-9)
	require.NotEqual(t, 0.5, report.Events[0].Bracket.To.Fraction.Base())
	end, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 6.5, end.Pose.Translation().Z, 2e-6)
	require.InDelta(t, math.Sin(0.12), end.Pose.ApplyDir(r3.Vec{X: 1}).Y, 1e-9)
}

func TestTorqueKickRejectsWideInertiaInterval(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	mass.Inertia.ZZ.Bound = units.KilogramSquareMillimeters(1)
	mass.Inertia.ZZ.Exactness = decad.Approximate
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: testTorque(100)}}},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Contains(t, report.Diagnostics[0].Reason, "kick")
}

func TestTorqueKickUsesWorldFrameInertia(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -10, 5, 10, 0, 30)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	shift, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	rotated, err := turn.Then(shift)
	require.NoError(t, err)
	torque := dynamics.QuantityVec{X: units.KilogramSquareMillimetersPerSecondSquared(100),
		Y: units.KilogramSquareMillimetersPerSecondSquared(0),
		Z: units.KilogramSquareMillimetersPerSecondSquared(0)}
	for _, tc := range []struct {
		name string
		pose r3.Transform
		spin float64
	}{{"source axes", shift, 10.0 / 650}, {"rotated axes", rotated, 10.0 / 500}} {
		t.Run(tc.name, func(t *testing.T) {
			start, err := w.NewState([]dynamics.BodyState{
				{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
				{Body: box, Pose: tc.pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
			})
			require.NoError(t, err)
			report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
				Loads: []dynamics.BodyLoad{{Body: box, Force: testForce(0, 0), Torque: torque}}},
				units.Seconds(0.1))
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			entry, ok := report.Next.Body(box)
			require.True(t, ok)
			require.InDelta(t, tc.spin, entry.AngularVelocity.X.Base(), 1e-8)
		})
	}
}

func TestFreeAsymmetricSpinIncludesGyroscopicKick(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -10, 5, 10, 0, 30)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	spin := dynamics.QuantityVec{X: units.RadiansPerSecond(1), Y: units.RadiansPerSecond(1),
		Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: spin},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	entry, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0.06, entry.AngularVelocity.Z.Base(), 1e-8)
	require.InDelta(t, 0, report.Conservation.TorqueImpulse.Value.Z.Base(), 1e-12)
}

func TestUnderflowedAngularDriftIsNotCertifiedClear(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	spin := zeroAngular(t)
	spin.Z = units.New(1e-200, underflowSpinUnit)
	require.Zero(t, spin.Z.Base())
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: spin},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Contains(t, report.Diagnostics[0].Reason, "kick")
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
