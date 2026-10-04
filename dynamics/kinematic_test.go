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
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: maxEvents,
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
	require.Empty(t, report.Events)
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
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, endPose.Translation(), finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{}, finalBox.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalBox.LinearVelocity)
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
	rotated, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
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
		{"rotating endpoint", []dynamics.KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: rotated, Duration: duration}}},
			duration, dynamics.ErrUnsupported},
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
