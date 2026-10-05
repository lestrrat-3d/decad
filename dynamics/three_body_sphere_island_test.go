package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodySphereFloorSphereCoupledInitialResponse(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	ball, upper := makeBall(t, doc), makeBall(t, doc)
	ballPose, err := r3.Translation(r3.Vec{Z: 5})
	require.NoError(t, err)
	upperPose, err := r3.Translation(r3.Vec{X: 6, Z: 13})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	for _, pair := range []struct {
		a, b   *decad.Body
		pa, pb r3.Transform
	}{
		{floor, ball, r3.Identity(), ballPose},
		{ball, upper, ballPose, upperPose},
	} {
		contact, contactErr := doc.ContactPair(t.Context(), pair.a, pair.b, pair.pa, pair.pb, request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, contact.Relation)
		require.NotEmpty(t, contact.Manifold.Points)
		still := decad.QuantityVec{X: units.MillimetersPerSecond(0),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
		zeroSpin := decad.QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
		sweep, sweepErr := doc.SweepPair(t.Context(), pair.a, pair.b,
			decad.RigidDriftSegment{From: pair.pa, LinearVelocity: still,
				AngularVelocity: zeroSpin, Duration: units.Seconds(.1)},
			decad.RigidDriftSegment{From: pair.pb, LinearVelocity: still,
				AngularVelocity: zeroSpin, Duration: units.Seconds(.1)},
			decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
				MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch})
		require.NoError(t, sweepErr)
		require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome, "cause=%v", sweep.Cause)
	}
	fixedPair, err := doc.ContactPair(t.Context(), floor, upper, r3.Identity(), upperPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, fixedPair.Relation)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 4}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: upper, Role: dynamics.Fixed, Material: material},
		},
		Step: step,
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: ballPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(200), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: upper, Pose: upperPose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: ball}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: ball, B: upper}, report.Events[1].Pair)
	require.InDelta(t, 1100.0/3, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 1000.0/3, report.Events[1].NormalImpulse.Base(), 1e-6)
	for _, event := range report.Events {
		require.Len(t, event.Manifold.Points, 1)
		require.NotNil(t, event.Solver)
	}
	finalBall, ok := report.Next.Body(ball)
	require.True(t, ok)
	require.Equal(t, ballPose, finalBall.Pose)
	require.Equal(t, zeroVelocity(), finalBall.LinearVelocity)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -200, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, 0, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-9)
	for _, at := range []units.Value{units.Seconds(0), units.Seconds(.05), units.Seconds(.1)} {
		replayed, sampleErr := report.Trace.Sample(at)
		require.NoError(t, sampleErr)
		require.Equal(t, report.Next.Entries(), replayed.Entries())
	}

	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: upper, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material},
		},
		Step: step,
	})
	require.NoError(t, err)
	reversedStart, err := reversed.NewState(start.Entries())
	require.NoError(t, err)
	reversedReport, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedReport.Status, "%+v", reversedReport.Diagnostics)
	require.Len(t, reversedReport.Events, 2)
	reversedBall, ok := reversedReport.Next.Body(ball)
	require.True(t, ok)
	require.Equal(t, finalBall, reversedBall)
	replayed, err := reversedReport.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	require.Equal(t, reversedReport.Next.Entries(), replayed.Entries())
	density := units.KilogramsPerCubicMillimeter(.001)
	densityWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: upper, Role: dynamics.Fixed, Material: material},
		}, Step: step,
	})
	require.NoError(t, err)
	densityStart, err := densityWorld.NewState(start.Entries())
	require.NoError(t, err)
	densityReport, err := densityWorld.Step(t.Context(), densityStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, densityReport.Status, "%+v", densityReport.Diagnostics)
	require.Len(t, densityReport.Events, 2)
	properties, err := ball.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, properties.Mass.Value.Base()*1100/3,
		densityReport.Events[0].NormalImpulse.Base(), 1e-6)
	require.NotNil(t, densityReport.Conservation)
	uncertainMass := mass
	uncertainMass.Mass.Bound = units.Kilograms(.01)
	uncertainMass.Mass.Exactness = decad.Approximate
	uncertainWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &uncertainMass, Material: material},
			{Body: upper, Role: dynamics.Fixed, Material: material},
		}, Step: step,
	})
	require.NoError(t, err)
	uncertainStart, err := uncertainWorld.NewState(start.Entries())
	require.NoError(t, err)
	uncertainReport, err := uncertainWorld.Step(t.Context(), uncertainStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, uncertainReport.Status)
	require.Nil(t, uncertainReport.Next)
	offsetMass := mass
	offsetMass.Center.Value = r3.Vec{X: 2}
	offsetWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &offsetMass, Material: material},
			{Body: upper, Role: dynamics.Fixed, Material: material},
		}, Step: step,
	})
	require.NoError(t, err)
	offsetStart, err := offsetWorld.NewState(start.Entries())
	require.NoError(t, err)
	offsetReport, err := offsetWorld.Step(t.Context(), offsetStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, offsetReport.Status)
	require.Nil(t, offsetReport.Next)

	resting, err := w.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, resting.Status, "%+v", resting.Diagnostics)
	require.Empty(t, resting.Events)
	require.NotNil(t, resting.Conservation)
	replayed, err = resting.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	require.Equal(t, resting.Next.Entries(), replayed.Entries())

	departingStart, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: ballPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: upper, Pose: upperPose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	departing, err := w.Step(t.Context(), departingStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, departing.Status, "%+v", departing.Diagnostics)
	require.Len(t, departing.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: floor, B: ball}, departing.Events[0].Pair)
	require.InDelta(t, 100, departing.Events[0].NormalImpulse.Base(), 1e-6)
	departedBall, ok := departing.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, -10, departedBall.Pose.Translation().X, 1e-12)
	require.InDelta(t, 5, departedBall.Pose.Translation().Z, 1e-12)
	require.Equal(t, 0.0, departedBall.LinearVelocity.Z.Base())
	endSphereContact, err := doc.ContactPair(t.Context(), ball, upper,
		departedBall.Pose, upperPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, endSphereContact.Relation)
	replayed, err = departing.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	midBall, ok := replayed.Body(ball)
	require.True(t, ok)
	require.InDelta(t, -5, midBall.Pose.Translation().X, 1e-12)

	tangentStart, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: ballPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(100),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: upper, Pose: upperPose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	tangent, err := w.Step(t.Context(), tangentStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, tangent.Status, "%+v", tangent.Diagnostics)
	require.Len(t, tangent.Events, 1)
	tangentBall, ok := tangent.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 10, tangentBall.Pose.Translation().Y, 1e-12)
	replayed, err = tangent.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	midBall, ok = replayed.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 5, midBall.Pose.Translation().Y, 1e-12)

	gravity := zeroAcceleration()
	gravity.Y = units.MillimetersPerSecondSquared(100)
	noImpact, err := w.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, noImpact.Status, "%+v", noImpact.Diagnostics)
	require.Empty(t, noImpact.Events)
	noImpactBall, ok := noImpact.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 10, noImpactBall.LinearVelocity.Y.Base(), 1e-12)
	replayed, err = noImpact.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	midBall, ok = replayed.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 10, midBall.LinearVelocity.Y.Base(), 1e-12)
	require.InDelta(t, .5, midBall.Pose.Translation().Y, 1e-12)
}
