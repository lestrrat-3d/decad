package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func rotatingNormalDriver(t *testing.T, translation float64) decad.PoseSegment {
	t.Helper()
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: translation})
	require.NoError(t, err)
	end, err := turn.Then(shift)
	require.NoError(t, err)
	return decad.PoseSegment{From: r3.Identity(), To: end, Duration: units.Seconds(1)}
}

func TestKinematicRotatingDriverInteriorImpactUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 19, 0, 29, 10, 0, 10)
	before := doc.Bodies()
	path := rotatingNormalDriver(t, 20)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	sweep, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: path.Duration},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.Nil(t, sweep.Event.Manifold)
	require.InDelta(t, .45, sweep.Bracket.To.Elapsed.Value.Base(), 1e-8)
	for _, sample := range sweep.Samples {
		if sample.At.Fraction != sweep.Bracket.To.Fraction {
			continue
		}
		full, readErr := path.To.Screw()
		require.NoError(t, readErr)
		inverse, readErr := sample.PoseA.Inverse()
		require.NoError(t, readErr)
		relative, readErr := inverse.Then(path.To)
		require.NoError(t, readErr)
		sliced, readErr := relative.Screw()
		require.NoError(t, readErr)
		require.Equal(t, full.Axis, sliced.Axis)
		require.NotEqual(t, full.Point.Z, sliced.Point.Z)
		require.InDelta(t, full.Point.Z, sliced.Point.Z, 1e-12)
		middle := (1 + sample.At.Fraction.Base()) / 2
		fullTurn, readErr := full.At(middle)
		require.NoError(t, readErr)
		originalMiddle, readErr := path.From.Then(fullTurn)
		require.NoError(t, readErr)
		slicedTurn, readErr := sliced.At(.5)
		require.NoError(t, readErr)
		slicedMiddle, readErr := sample.PoseA.Then(slicedTurn)
		require.NoError(t, readErr)
		require.InDelta(t, originalMiddle.Translation().X, slicedMiddle.Translation().X, 1e-8)
	}

	w := kinematicImpactWorld(t, doc, driver, box, false, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	require.Equal(t, r3.Vec{X: 1}, report.Events[0].Manifold.Points[0].Normal.Value)
	require.InDelta(t, 30, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, .45, report.Events[0].Time.Base(), 1e-8)
	require.InDelta(t, 600, report.Conservation.KinematicWork.Value.Base(), 1e-6)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 30, finalBox.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 16.5, finalBox.Pose.Translation().X, 1e-5)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	checkpoint, err := report.Trace.Sample(report.Events[0].Time)
	require.NoError(t, err)
	postBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, report.Events[0].PostVelocity, postBox.LinearVelocity)
	require.Equal(t, before, doc.Bodies())
}

func TestKinematicRotatingDriverMovesClearPair(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 100, 0, 110, 10, 0, 10)
	path := rotatingNormalDriver(t, 20)
	w := kinematicBoxWorld(t, doc, driver, box)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), finalBox.Pose)
}

func TestKinematicRotatingDriverInteriorImpactReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 19, 0, 29, 10, 0, 10)
	path := rotatingNormalDriver(t, 20)
	w := kinematicImpactWorld(t, doc, driver, box, true, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, r3.Vec{X: -1}, report.Events[0].Manifold.Points[0].Normal.Value)
	require.InDelta(t, 30, report.Events[0].NormalImpulse.Base(), 1e-6)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 30, finalBox.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 16.5, finalBox.Pose.Translation().X, 1e-5)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
}
