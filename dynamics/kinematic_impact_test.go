package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestKinematicDriverInteriorImpactUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	beforeDriver, err := driver.Bounds()
	require.NoError(t, err)
	beforeBox, err := box.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	duration := units.Seconds(.25)
	endDriver, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	path := decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	first, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, first.Outcome)
	require.NotNil(t, first.Bracket)
	require.NotNil(t, first.Event)
	require.NotNil(t, first.Event.Manifold)
	require.InDelta(t, .125, first.Bracket.To.Elapsed.Value.Base(), 1e-8)
	contactDriver, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), driver, box, contactDriver, r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	endBox, err := r3.Translation(r3.Vec{X: 15})
	require.NoError(t, err)
	postVelocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(120),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	remainder := units.Seconds(.125)
	continuationRequest := decad.SweepRequest{ContactRequest: request,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128,
		StartPolicy: decad.ContinueSeparatingTouch}
	slicedDriver := decad.PoseSegment{From: contactDriver, To: endDriver, Duration: remainder}
	ideal, err := doc.SweepPair(t.Context(), driver, box, slicedDriver,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: postVelocity, AngularVelocity: zeroAngular(t), Duration: remainder}, continuationRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, ideal.Outcome)
	rounded, err := doc.SweepPair(t.Context(), driver, box, slicedDriver,
		decad.PoseSegment{From: r3.Identity(), To: endBox, Duration: remainder}, continuationRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, rounded.Outcome)

	w := kinematicImpactWorld(t, doc, driver, box, false, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Impulse, report.Conservation.ContactImpulse.Value.X.Kind())
	require.Equal(t, units.Impulse, report.Conservation.ContactImpulse.Bound.X.Kind())
	require.InDelta(t, 120, report.Conservation.ContactImpulse.Value.X.Base(), 1e-5)
	require.InDelta(t, 120, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-5)
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Value.Kind())
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Bound.Kind())
	require.InDelta(t, 9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	require.Less(t, report.Conservation.KinematicWork.Bound.Base(), 1e-6)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Equal(t, first.Bracket.To.Elapsed.Value, event.Time)
	require.InDelta(t, .125, event.Time.Base(), 1e-8)
	require.InDelta(t, 120, event.NormalImpulse.Base(), 1e-5)
	driverSpeed := dynamics.QuantityVec{X: units.MillimetersPerSecond(80),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	require.Equal(t, driverSpeed, event.PreVelocityA)
	require.Equal(t, driverSpeed, event.PostVelocityA)
	require.Equal(t, zeroVelocity(), event.PreVelocityB)
	require.Equal(t, postVelocity, event.PostVelocityB)
	require.Equal(t, r3.Vec{}, event.PositionChangeA)
	require.InDelta(t, 0, event.PositionChangeB.Y, 1e-12)
	require.InDelta(t, 0, event.PositionChangeB.Z, 1e-12)
	checkpoint, err := report.Trace.Sample(event.Time)
	require.NoError(t, err)
	checkpointDriver, ok := checkpoint.Body(driver)
	require.True(t, ok)
	checkpointBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.InDelta(t, 80*event.Time.Base(), checkpointDriver.Pose.Translation().X, 1e-6)
	require.Equal(t, zeroVelocity(), checkpointDriver.LinearVelocity)
	require.InDelta(t, event.PositionChangeB.X, checkpointBox.Pose.Translation().X, 1e-6)
	require.Equal(t, postVelocity, checkpointBox.LinearVelocity)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 20}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	require.InDelta(t, 15, finalBox.Pose.Translation().X, 1e-5)
	require.Equal(t, postVelocity, finalBox.LinearVelocity)
	require.Equal(t, []*decad.Body{driver, box}, doc.Bodies())
	afterDriver, err := driver.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, beforeDriver, afterDriver)
	require.Equal(t, beforeBox, afterBox)
}

func kinematicImpactWorld(t *testing.T, doc *decad.Document, driver, box *decad.Body,
	dynamicFirst bool, maxEvents int) *dynamics.World {
	return kinematicImpactWorldWithRestitution(t, doc, driver, box, dynamicFirst, maxEvents, .5)
}

func kinematicImpactWorldWithRestitution(t *testing.T, doc *decad.Document, driver, box *decad.Body,
	dynamicFirst bool, maxEvents int, restitution float64) *dynamics.World {
	t.Helper()
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(0)}
	bodies := []dynamics.RigidBody{
		{Body: driver, Role: dynamics.Kinematic, Material: material},
		{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
	}
	if dynamicFirst {
		bodies[0], bodies[1] = bodies[1], bodies[0]
	}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: bodies,
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: maxEvents,
		},
	})
	require.NoError(t, err)
	return w
}

func TestKinematicInteriorImpactRespectsEventLimit(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	w := kinematicImpactWorld(t, doc, driver, box, false, 1)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endDriver, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Empty(t, report.Events)
}

func TestKinematicInteriorImpactSupportsReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	box := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	driver := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	w := kinematicImpactWorld(t, doc, driver, box, true, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endDriver, err := r3.Translation(r3.Vec{X: -20})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -120, report.Conservation.ContactImpulse.Value.X.Base(), 1e-5)
	require.InDelta(t, 9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 120, report.Events[0].NormalImpulse.Base(), 1e-5)
	require.Equal(t, units.MillimetersPerSecond(-80), report.Events[0].PreVelocityB.X)
	require.Equal(t, units.MillimetersPerSecond(-120), report.Events[0].PostVelocityA.X)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, -15, finalBox.Pose.Translation().X, 1e-5)
	require.Equal(t, units.MillimetersPerSecond(-120), finalBox.LinearVelocity.X)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -20}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
}
