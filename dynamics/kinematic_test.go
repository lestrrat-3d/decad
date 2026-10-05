package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func kinematicBoxWorld(t *testing.T, doc *decad.Document, driver, box *decad.Body) *dynamics.World {
	return kinematicBoxWorldWithLimit(t, doc, driver, box, false, 2)
}

func kinematicBoxWorldOrdered(t *testing.T, doc *decad.Document, driver, box *decad.Body,
	dynamicFirst bool) *dynamics.World {
	return kinematicBoxWorldWithLimit(t, doc, driver, box, dynamicFirst, 2)
}

func kinematicBoxWorldWithLimit(t *testing.T, doc *decad.Document, driver, box *decad.Body,
	dynamicFirst bool, maxEvents int) *dynamics.World {
	t.Helper()
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	bodies := []dynamics.RigidBody{
		{Body: driver, Role: dynamics.Kinematic, Material: material},
		{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
	}
	if dynamicFirst {
		bodies[0], bodies[1] = bodies[1], bodies[0]
	}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: bodies,
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: maxEvents, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	return w
}

func TestKinematicPushRespectsEventLimit(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	w := kinematicBoxWorldWithLimit(t, doc, driver, box, false, 1)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endPose, err := r3.Translation(r3.Vec{X: 1.25})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	// §12: the impact reaches MaxEvents 1 with time remaining; the certified
	// prefix keeps its event and stops there.
	require.Len(t, report.Events, 1)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepEventBudget, report.Diagnostics[0].Code)
	require.Equal(t, units.Scalar(1), report.Diagnostics[0].Limit)
	require.Equal(t, report.Events[0].Time, report.Diagnostics[0].From)
	require.Equal(t, []*decad.Body{driver, box}, doc.Bodies())
}

func TestKinematicDriverPushesDynamicBodyInReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	box := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	driver := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	w := kinematicBoxWorldOrdered(t, doc, driver, box, true)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endPose, err := r3.Translation(r3.Vec{X: -1.25})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 10, report.Events[0].NormalImpulse.Base(), 1e-6)
	backward := dynamics.QuantityVec{X: units.MillimetersPerSecond(-10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	require.Equal(t, backward, report.Events[0].PreVelocityB)
	require.Equal(t, backward, report.Events[0].PostVelocityA)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, endPose.Translation(), finalBox.Pose.Translation())
	require.Equal(t, backward, finalBox.LinearVelocity)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, endPose.Translation(), finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
}

func TestKinematicDriverMovesClearPair(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 100, 0, 110, 10, 0, 10)
	w := kinematicBoxWorld(t, doc, driver, box)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endPose, err := r3.Translation(r3.Vec{X: 1.25})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Value.Kind())
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Bound.Kind())
	require.Zero(t, report.Conservation.KinematicWork.Value.Base())
	require.Zero(t, report.Conservation.KinematicWork.Bound.Base())
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, endPose.Translation(), finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	interior, err := report.Trace.Sample(units.Seconds(.0625))
	require.NoError(t, err)
	interiorDriver, ok := interior.Body(driver)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: .625}, interiorDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), interiorDriver.LinearVelocity)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{}, finalBox.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalBox.LinearVelocity)
}

func TestKinematicDriverDepartsInitiallyTouchingBox(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	beforeDriver, err := driver.Bounds()
	require.NoError(t, err)
	beforeBox, err := box.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	initial, err := doc.ContactPair(t.Context(), driver, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initial.Relation)
	require.NotNil(t, initial.Manifold)
	duration := units.Seconds(.125)
	endPose, err := r3.Translation(r3.Vec{X: -5})
	require.NoError(t, err)
	path := decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}
	still := decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
		LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: duration}
	sweepRequest := decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128}
	first, err := doc.SweepPair(t.Context(), driver, box, path, still, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, first.Outcome)
	require.NotNil(t, first.Event)
	require.NotNil(t, first.Event.Manifold)
	sweepRequest.StartPolicy = decad.ContinueSeparatingTouch
	ideal, err := doc.SweepPair(t.Context(), driver, box, path, still, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, ideal.Outcome)
	rounded, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, rounded.Outcome)
	endpoint, err := doc.ContactPair(t.Context(), driver, box, endPose, r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, endpoint.Relation)

	w := kinematicBoxWorldWithLimit(t, doc, driver, box, false, 1)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Next)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -5}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	require.Equal(t, r3.Vec{}, finalBox.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalBox.LinearVelocity)
	traceStart, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	traceEnd, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, start, traceStart)
	require.Equal(t, *report.Next, traceEnd)
	require.Equal(t, []*decad.Body{driver, box}, doc.Bodies())
	afterDriver, err := driver.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, beforeDriver, afterDriver)
	require.Equal(t, beforeBox, afterBox)
}

func TestKinematicDriverDepartureSupportsReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	box := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	driver := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	w := kinematicBoxWorldWithLimit(t, doc, driver, box, true, 1)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	endPose, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{}, finalBox.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalBox.LinearVelocity)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 5}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	endpoint, err := doc.ContactPair(t.Context(), box, driver, finalBox.Pose, finalDriver.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, endpoint.Relation)
}

func TestKinematicDriverRejectsInvalidPaths(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	w := kinematicBoxWorld(t, doc, driver, box)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	endPose, err := r3.Translation(r3.Vec{X: 1.25})
	require.NoError(t, err)
	badStart, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	rotated, err := r3.Rotation(r3.Vec{X: 1, Y: 1}, units.Degrees(90))
	require.NoError(t, err)
	valid := dynamics.KinematicDriver{Body: driver,
		Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}}
	for _, tc := range []struct {
		name    string
		drivers []dynamics.KinematicDriver
		dt      units.Value
		want    error
	}{
		{"missing", nil, duration, dynamics.ErrInvalidInput},
		{"duplicate", []dynamics.KinematicDriver{valid, valid}, duration, dynamics.ErrInvalidInput},
		{"wrong body", []dynamics.KinematicDriver{{Body: box, Path: valid.Path}}, duration,
			dynamics.ErrInvalidInput},
		{"nil path", []dynamics.KinematicDriver{{Body: driver}}, duration, dynamics.ErrInvalidInput},
		{"wrong duration", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: units.Seconds(.2)}}},
			duration, dynamics.ErrInvalidInput},
		{"wrong start", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: badStart, To: endPose, Duration: duration}}},
			duration, dynamics.ErrInvalidInput},
		{"invalid endpoint", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: r3.Transform{}, Duration: duration}}},
			duration, dynamics.ErrInvalidInput},
		{"unsupported path", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.RigidDriftSegment{From: r3.Identity(), Center: r3.Vec{},
				LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: duration}}},
			duration, dynamics.ErrUnsupported},
		{"inexact derivative", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: badStart, Duration: units.Seconds(.3)}}},
			units.Seconds(.3), dynamics.ErrUnsupported},
		{"unrepresentable derivative", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: badStart,
				Duration: units.Seconds(math.SmallestNonzeroFloat64)}}},
			units.Seconds(math.SmallestNonzeroFloat64), dynamics.ErrUnsupported},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := w.Step(t.Context(), start, dynamics.StepInput{
				Gravity: zeroAcceleration(), Drivers: tc.drivers}, tc.dt)
			require.ErrorIs(t, err, tc.want)
		})
	}
	// A driver turning about (1, 1, 0) is a valid input; the touching box's
	// sweep cannot follow its screw, so the step stops at its start.
	turning, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: rotated, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, turning.Status)
	require.Nil(t, turning.Next)
	require.Len(t, turning.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, turning.Diagnostics[0].Code)
	require.Equal(t, dynamics.BodyPair{A: driver, B: box}, turning.Diagnostics[0].Pair)
	require.Equal(t, units.Seconds(0), turning.Diagnostics[0].From)
	require.Equal(t, duration, turning.Diagnostics[0].To)
	_, err = w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(1), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
	underflowAngular := units.Define("decad-test-kinematic-angular-underflow", units.AngularVelocity, 1e-200)
	tinySpin := zeroAngular(t)
	tinySpin.Z = units.New(1e-200, underflowAngular)
	require.Zero(t, tinySpin.Z.Base())
	_, err = w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: tinySpin},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}
