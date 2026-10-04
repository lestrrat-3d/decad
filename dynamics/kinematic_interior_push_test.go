package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestKinematicInteriorZeroRestitutionPushUsesProductionGeometry(t *testing.T) {
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
	driverPath := decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	first, err := doc.SweepPair(t.Context(), driver, box, driverPath,
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
	var right *decad.SweepSample
	for i := range first.Samples {
		if first.Samples[i].At.Fraction == first.Bracket.To.Fraction {
			right = &first.Samples[i]
			break
		}
	}
	require.NotNil(t, right)
	require.Equal(t, decad.ContactOverlapping, right.FloatContact.Relation)
	separation := first.Event.Manifold.Points[0].Separation.Value.Base()
	require.Less(t, separation, 0.0)
	correction, err := r3.Translation(r3.Vec{X: -separation})
	require.NoError(t, err)
	correctedBox, err := right.PoseB.Then(correction)
	require.NoError(t, err)
	postContact, err := doc.ContactPair(t.Context(), driver, box, right.PoseA, correctedBox, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, postContact.Relation)
	remainder := units.Seconds(duration.Base() - first.Bracket.To.Elapsed.Value.Base())
	slicedDriver := decad.PoseSegment{From: right.PoseA, To: endDriver, Duration: remainder}
	postVelocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(80),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	continuationRequest := decad.SweepRequest{ContactRequest: request,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128,
		StartPolicy: decad.ContinueCertifiedTouch}
	ideal, err := doc.SweepPair(t.Context(), driver, box, slicedDriver,
		decad.RigidDriftSegment{From: correctedBox, Center: correctedBox.Apply(mass.Center.Value),
			LinearVelocity: postVelocity, AngularVelocity: zeroAngular(t), Duration: remainder}, continuationRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, ideal.Outcome)
	require.NotNil(t, ideal.ContactTrack)
	endBoxMotion, err := r3.Translation(r3.Vec{X: 80 * remainder.Base()})
	require.NoError(t, err)
	endBox, err := correctedBox.Then(endBoxMotion)
	require.NoError(t, err)
	rounded, err := doc.SweepPair(t.Context(), driver, box, slicedDriver,
		decad.PoseSegment{From: correctedBox, To: endBox, Duration: remainder}, continuationRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, rounded.Outcome)
	require.NotNil(t, rounded.ContactTrack)

	w := kinematicImpactWorldWithRestitution(t, doc, driver, box, false, 2, 0)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: driverPath}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Equal(t, first.Bracket.To.Elapsed.Value, event.Time)
	require.InDelta(t, 80, event.NormalImpulse.Base(), 1e-5)
	require.Equal(t, postVelocity, event.PreVelocityA)
	require.Equal(t, postVelocity, event.PostVelocityA)
	require.Equal(t, zeroVelocity(), event.PreVelocityB)
	require.Equal(t, postVelocity, event.PostVelocityB)
	checkpoint, err := report.Trace.Sample(event.Time)
	require.NoError(t, err)
	checkpointDriver, ok := checkpoint.Body(driver)
	require.True(t, ok)
	checkpointBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, right.PoseA, checkpointDriver.Pose)
	require.Equal(t, zeroVelocity(), checkpointDriver.LinearVelocity)
	require.InDelta(t, correctedBox.Translation().X, checkpointBox.Pose.Translation().X, 1e-6)
	require.Equal(t, postVelocity, checkpointBox.LinearVelocity)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 20}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	require.InDelta(t, 10, finalBox.Pose.Translation().X, 1e-5)
	require.Equal(t, postVelocity, finalBox.LinearVelocity)
	endContact, err := doc.ContactPair(t.Context(), driver, box, finalDriver.Pose, finalBox.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endContact.Relation)
	require.Equal(t, []*decad.Body{driver, box}, doc.Bodies())
	afterDriver, err := driver.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, beforeDriver, afterDriver)
	require.Equal(t, beforeBox, afterBox)
}

func TestKinematicInteriorZeroRestitutionPushSupportsReverseWorldOrder(t *testing.T) {
	doc := decad.New()
	box := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	driver := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	w := kinematicImpactWorldWithRestitution(t, doc, driver, box, true, 2, 0)
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
	require.Len(t, report.Events, 1)
	require.InDelta(t, 80, report.Events[0].NormalImpulse.Base(), 1e-5)
	require.Equal(t, units.MillimetersPerSecond(-80), report.Events[0].PreVelocityB.X)
	require.Equal(t, units.MillimetersPerSecond(-80), report.Events[0].PostVelocityA.X)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, -10, finalBox.Pose.Translation().X, 1e-5)
	require.Equal(t, units.MillimetersPerSecond(-80), finalBox.LinearVelocity.X)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -20}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	endContact, err := doc.ContactPair(t.Context(), box, driver, finalBox.Pose, finalDriver.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endContact.Relation)
}

func TestKinematicInteriorZeroRestitutionPushRespectsEventLimit(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	w := kinematicImpactWorldWithRestitution(t, doc, driver, box, false, 1, 0)
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
