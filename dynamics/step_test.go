package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func makeBox(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(height), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

func zeroAngular(t *testing.T) dynamics.QuantityVec {
	t.Helper()
	zero := units.RadiansPerSecond(0)
	return dynamics.QuantityVec{X: zero, Y: zero, Z: zero}
}

func zeroVelocity() dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
}

func zeroAcceleration() dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
}

func fixedBoxContactWorld(t *testing.T, doc *decad.Document, floor, box *decad.Body,
	restitution float64) *dynamics.World {
	return fixedBoxContactWorldWithMaxEvents(t, doc, floor, box, restitution, 2)
}

func fixedBoxContactWorldWithMaxEvents(t *testing.T, doc *decad.Document, floor, box *decad.Body,
	restitution float64, maxEvents int) *dynamics.World {
	t.Helper()
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: maxEvents, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	return w
}

// The geometry producers and response solver all run here; no contact or mass fixture is fabricated.
func TestVerticalBoxReboundUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	originalFloor, err := floor.Bounds()
	require.NoError(t, err)
	originalBox, err := box.Bounds()
	require.NoError(t, err)

	density := units.KilogramsPerCubicMillimeter(0.001)
	impulseLimit := units.KilogramMillimetersPerSecond(1e-6)
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(2e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual: units.MillimetersPerSecond(1e-6), ImpulseResidual: impulseLimit,
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100),
		}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Torque, report.Conservation.Input.KineticEnergy.Value.Kind())
	require.Equal(t, units.Impulse, report.Conservation.Input.LinearMomentum.Value.Z.Kind())
	require.InDelta(t, 5000, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 5000, report.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 1250, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
	require.InDelta(t, -100, report.Conservation.Input.LinearMomentum.Value.Z.Base(), 1e-9)
	require.InDelta(t, 50, report.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	require.Zero(t, report.Conservation.GravityImpulse.Value.Z.Base())
	require.Zero(t, report.Conservation.LoadImpulse.Value.Z.Base())
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-4)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.InDelta(t, 0.1, event.Time.Base(), 1e-9)
	require.InDelta(t, 150, event.NormalImpulse.Base(), 1e-4)
	require.NotEmpty(t, event.Manifold.Points)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 5, final.Pose.Apply(r3.Vec{}).Z, 2e-6)
	replayed, err := report.Trace.Sample(units.Seconds(0.2))
	require.NoError(t, err)
	replayedBox, ok := replayed.Body(box)
	require.True(t, ok)
	require.InDelta(t, final.Pose.Apply(r3.Vec{}).Z, replayedBox.Pose.Apply(r3.Vec{}).Z, 1e-12)
	postSample, err := report.Trace.Sample(units.Seconds(0.15))
	require.NoError(t, err)
	postBox, ok := postSample.Body(box)
	require.True(t, ok)
	require.InDelta(t, 2.5, postBox.Pose.Translation().Z, 1e-6)
	require.Equal(t, units.MillimetersPerSecond(50), postBox.LinearVelocity.Z)
	postContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), postBox.Pose, config.Step.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, postContact.Relation)
	preSample, err := report.Trace.Sample(units.Seconds(0.05))
	require.NoError(t, err)
	preBox, ok := preSample.Body(box)
	require.True(t, ok)
	require.InDelta(t, 5, preBox.Pose.Translation().Z, 1e-9)
	require.Equal(t, units.MillimetersPerSecond(-100), preBox.LinearVelocity.Z)
	preContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), preBox.Pose, config.Step.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, preContact.Relation)
	unitShifted := units.Milliseconds(event.Time.Base() * 1000)
	require.Equal(t, event.Time.Base(), unitShifted.Base())
	unitSample, err := report.Trace.Sample(unitShifted)
	require.NoError(t, err)
	unitBox, ok := unitSample.Body(box)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(-100), unitBox.LinearVelocity.Z)
	beyondEnd := units.Minutes(.2 / 60)
	require.Equal(t, units.Seconds(.2).Base(), beyondEnd.Base())
	_, err = report.Trace.Sample(beyondEnd)
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, originalFloor, afterFloor)
	require.Equal(t, originalBox, afterBox)
	endpoint, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, endpoint.Status, "%+v", endpoint.Diagnostics)
	require.Len(t, endpoint.Events, 1)
	require.Equal(t, 1.0, endpoint.Events[0].Bracket.To.Fraction.Base())
	require.InDelta(t, 0.1, endpoint.Events[0].Time.Base(), 1e-12)
	endBox, ok := endpoint.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0, endBox.Pose.Translation().Z, 2e-6)
	require.InDelta(t, 50, endBox.LinearVelocity.Z.Base(), 1e-6)
	endContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), endBox.Pose, config.Step.Contact)
	require.NoError(t, err)
	// The impact at the step end is corrected into exact touch (§6.6); the
	// box leaves it at 50 mm/s on the next step.
	require.Equal(t, decad.ContactTouching, endContact.Relation)
	require.Zero(t, endBox.Pose.Translation().Z)
	beforeEndpoint, err := endpoint.Trace.Sample(units.Seconds(.1 - 1e-9))
	require.NoError(t, err)
	beforeBox, ok := beforeEndpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(-100), beforeBox.LinearVelocity.Z)
	beforeContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), beforeBox.Pose,
		config.Step.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, beforeContact.Relation)
}

func TestTraceSampleEarlyImpactNearEnd(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, .5)
	pose, err := r3.Translation(r3.Vec{Z: 1.25})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(.2)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, .0125, report.Events[0].Time.Base(), 1e-9)
	nearEnd := units.Milliseconds(200)
	require.Equal(t, duration.Base(), nearEnd.Base())
	sample, err := report.Trace.Sample(nearEnd)
	require.NoError(t, err)
	sampleBox, ok := sample.Body(box)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(50), sampleBox.LinearVelocity.Z)
	require.InDelta(t, 9.375, sampleBox.Pose.Translation().Z, 1e-5)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), sampleBox.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
}

func TestTraceSampleMillisecondEndpointImpact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, .5)
	gap := math.Nextafter(10, 0)
	pose, err := r3.Translation(r3.Vec{Z: gap})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Milliseconds(100)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, units.Scalar(1), report.Events[0].Bracket.To.Fraction)
	require.Equal(t, duration, report.Events[0].Time)
	post, err := report.Trace.Sample(report.Events[0].Time)
	require.NoError(t, err)
	postBox, ok := post.Body(box)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(50), postBox.LinearVelocity.Z)
}

func TestObliqueBoxReboundUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	floorBounds, err := floor.Bounds()
	require.NoError(t, err)
	boxBounds, err := box.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	contactPose, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	duration := units.Seconds(0.2)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)}
	sweep, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: startPose, Center: startPose.Apply(mass.Center.Value),
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), contactPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-4)
	require.Equal(t, report.Events[0].PreVelocityB.X, report.Events[0].PostVelocityB.X)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 10, final.Pose.Translation().X, 2e-6)
	require.InDelta(t, 5, final.Pose.Translation().Z, 2e-6)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, floorBounds, afterFloor)
	require.Equal(t, boxBounds, afterBox)
}

func TestTwoAxisBoxDriftStaysClear(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(50), Y: units.MillimetersPerSecond(20),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 10, Y: 4, Z: 10}, final.Pose.Translation())
	interior, err := report.Trace.Sample(units.Seconds(0.125))
	require.NoError(t, err)
	interiorBox, ok := interior.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 6.25, Y: 2.5, Z: 10}, interiorBox.Pose.Translation())
	interiorContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), interiorBox.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, interiorContact.Relation)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestObliqueInitialTouchContinuesAsPersistentContact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(50), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.ContactImpact, report.Events[0].Kind)
	// An initial touch takes its restitution target like any contact
	// (docs/multibody-dynamics-design.md §6.5): restitution 0.5 sends the box
	// up at 50 mm/s with 1.5·100 kg·mm/s, and it keeps its 50 mm/s slide.
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-6)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-6)
	require.InDelta(t, 5, final.Pose.Translation().Z, 1e-6)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestInitiallyTouchingTangentialSlideUsesProductionTrack(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	floorBounds, err := floor.Bounds()
	require.NoError(t, err)
	boxBounds, err := box.Bounds()
	require.NoError(t, err)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	initial, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initial.Relation)
	require.NotNil(t, initial.Manifold)
	duration := units.Seconds(0.1)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(50),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	endPose, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	sweepRequest := decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch}
	ideal, err := doc.SweepPair(t.Context(), floor, box, stationary,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: duration}, sweepRequest)
	require.NoError(t, err)
	rounded, err := doc.SweepPair(t.Context(), floor, box, stationary,
		decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}, sweepRequest)
	require.NoError(t, err)
	for _, sweep := range []*decad.SweepReport{ideal, rounded} {
		require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
		require.NotNil(t, sweep.ContactTrack)
		for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(.5), units.Scalar(1)} {
			manifold, err := sweep.ContactTrack.ManifoldAt(fraction)
			require.NoError(t, err)
			require.NotNil(t, manifold)
		}
	}
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Next)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, velocity, final.LinearVelocity)
	require.Equal(t, endPose.Translation(), final.Pose.Translation())
	interior, err := report.Trace.Sample(units.Seconds(0.0375))
	require.NoError(t, err)
	interiorBox, ok := interior.Body(box)
	require.True(t, ok)
	require.InDelta(t, 1.875, interiorBox.Pose.Translation().X, 1e-12)
	require.Equal(t, velocity, interiorBox.LinearVelocity)
	endpoint, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), final.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endpoint.Relation)
	require.NotNil(t, endpoint.Manifold)
	replayed, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, floorBounds, afterFloor)
	require.Equal(t, boxBounds, afterBox)
}

func TestSlideEdgeTransitionRespectsEventLimit(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 10, 10)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	duration := units.Seconds(3)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(5),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	sweep, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128,
			StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, sweep.Outcome)
	w := fixedBoxContactWorldWithMaxEvents(t, doc, floor, box, 0, 1)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
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
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestSlideEdgeTransitionContinuesClearWithProductionGeometry(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 10, 10)
	floorBounds, err := floor.Bounds()
	require.NoError(t, err)
	boxBounds, err := box.Bounds()
	require.NoError(t, err)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	duration := units.Seconds(3)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(5),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	initial, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initial.Relation)
	sweepRequest := decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch}
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	first, err := doc.SweepPair(t.Context(), floor, box, still,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: duration}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, first.Outcome)
	require.NotNil(t, first.Bracket)
	require.NotNil(t, first.ContactTrack)
	require.NotNil(t, first.Event)
	require.Equal(t, decad.ContactSeparated, first.Event.Relation)
	require.InDelta(t, 2, first.Bracket.To.Elapsed.Value.Base(), 1e-9)
	require.Equal(t, first.Bracket.From.Fraction, first.ContactTrack.End().Fraction)
	for _, fraction := range []units.Value{units.Scalar(0),
		units.Scalar(first.ContactTrack.End().Fraction.Base() / 2), first.ContactTrack.End().Fraction} {
		manifold, err := first.ContactTrack.ManifoldAt(fraction)
		require.NoError(t, err)
		require.NotNil(t, manifold)
	}
	endPose, err := r3.Translation(r3.Vec{X: 15})
	require.NoError(t, err)
	roundedFirst, err := doc.SweepPair(t.Context(), floor, box, still,
		decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, roundedFirst.Outcome)
	require.NotNil(t, roundedFirst.ContactTrack)
	require.NotNil(t, roundedFirst.Bracket)
	chosen := duration.Base() * first.Bracket.To.Fraction.Base()
	right, err := r3.Translation(r3.Vec{X: 5 * chosen})
	require.NoError(t, err)
	rightContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), right, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, rightContact.Relation)
	remainder := units.Seconds(duration.Base() - chosen)
	clearRequest := sweepRequest
	clearRequest.StartPolicy = decad.StopAtInitialContact
	ideal, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: remainder},
		decad.RigidDriftSegment{From: right, Center: right.Apply(mass.Center.Value),
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: remainder}, clearRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, ideal.Outcome)
	rounded, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: remainder},
		decad.PoseSegment{From: right, To: endPose, Duration: remainder}, clearRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, rounded.Outcome)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.ContactTransition, report.Events[0].Kind)
	require.InDelta(t, 2, report.Events[0].Time.Base(), 1e-9)
	require.Equal(t, units.KilogramMillimetersPerSecond(0), report.Events[0].NormalImpulse)
	require.Empty(t, report.Events[0].Manifold.Points)
	require.Equal(t, velocity, report.Events[0].PreVelocityB)
	require.Equal(t, velocity, report.Events[0].PostVelocityB)
	require.Equal(t, *first.Bracket, report.Events[0].Bracket)
	preInterior, err := report.Trace.Sample(units.Seconds(1))
	require.NoError(t, err)
	preBox, ok := preInterior.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 5}, preBox.Pose.Translation())
	preContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), preBox.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, preContact.Relation)
	checkpoint, err := report.Trace.Sample(report.Events[0].Time)
	require.NoError(t, err)
	checkpointBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, right.Translation(), checkpointBox.Pose.Translation())
	require.Equal(t, velocity, checkpointBox.LinearVelocity)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, velocity, final.LinearVelocity)
	require.Equal(t, endPose.Translation(), final.Pose.Translation())
	postInterior, err := report.Trace.Sample(units.Seconds(2.5))
	require.NoError(t, err)
	postBox, ok := postInterior.Body(box)
	require.True(t, ok)
	require.InDelta(t, 12.5, postBox.Pose.Translation().X, 1e-8)
	postContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), postBox.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, postContact.Relation)
	replayed, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, floorBounds, afterFloor)
	require.Equal(t, boxBounds, afterBox)
}

func TestSlideEdgeTransitionDoesNotRepeatForceKick(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 10, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	startVelocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(2),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: startVelocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	zeroTorque := units.KilogramSquareMillimetersPerSecondSquared(0)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{
		Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: dynamics.QuantityVec{
			X: units.KilogramMillimetersPerSecondSquared(1),
			Y: units.KilogramMillimetersPerSecondSquared(0),
			Z: units.KilogramMillimetersPerSecondSquared(0)},
			Torque: dynamics.QuantityVec{X: zeroTorque, Y: zeroTorque, Z: zeroTorque}}},
	}, units.Seconds(3))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.ContactTransition, report.Events[0].Kind)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(5), final.LinearVelocity.X)
	require.Equal(t, r3.Vec{X: 15}, final.Pose.Translation())
}

func TestSlideEdgeTransitionCertifiesRoundedRightPose(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 10, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	speed := 28.900856797231235
	duration := units.Seconds(9.530447383534144)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(speed),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	request := decad.SweepRequest{ContactRequest: decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128,
		StartPolicy: decad.ContinueCertifiedTouch}
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	ideal, err := doc.SweepPair(t.Context(), floor, box, still,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: velocity, AngularVelocity: zeroAngular(t), Duration: duration}, request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, ideal.Outcome)
	require.NotNil(t, ideal.Bracket)
	wholeEnd, err := r3.Translation(r3.Vec{X: speed * duration.Base()})
	require.NoError(t, err)
	fullRounded, err := doc.SweepPair(t.Context(), floor, box, still,
		decad.PoseSegment{From: r3.Identity(), To: wholeEnd, Duration: duration}, request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, fullRounded.Outcome)
	chosen := duration.Base() * ideal.Bracket.To.Fraction.Base()
	right, err := r3.Translation(r3.Vec{X: speed * chosen})
	require.NoError(t, err)
	require.NotEqual(t, wholeEnd.Translation().X*ideal.Bracket.To.Fraction.Base(), right.Translation().X)
	prefix, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(chosen)},
		decad.PoseSegment{From: r3.Identity(), To: right, Duration: units.Seconds(chosen)}, request)
	require.NoError(t, err)
	require.Equal(t, decad.SweepContactTransitionBracket, prefix.Outcome)
	require.NotNil(t, prefix.ContactTrack)
	require.NotNil(t, prefix.Bracket)
	require.Equal(t, units.Scalar(1), prefix.Bracket.To.Fraction)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.ContactTransition, report.Events[0].Kind)
	checkpoint, err := report.Trace.Sample(report.Events[0].Time)
	require.NoError(t, err)
	checkpointBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, right.Translation(), checkpointBox.Pose.Translation())
}

func TestSuppliedMassBoxReboundUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	duration := units.Seconds(0.2)
	sweep, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: startPose, Center: startPose.Apply(mass.Center.Value),
			LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)},
			AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
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
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	// World must retain the supplied readings it admitted, not the caller's pointer.
	mass.Mass.Value = units.Kilograms(2)
	mass.Center.Value = r3.Vec{X: 100}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-4)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 5, final.Pose.Translation().Z, 2e-6)
}

func TestCenteredForceBoxReboundUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	originalFloor, err := floor.Bounds()
	require.NoError(t, err)
	originalBox, err := box.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	duration := units.Seconds(0.2)
	fall := dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)}
	sweep, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: pose, Center: pose.Apply(mass.Center.Value),
			LinearVelocity: fall, AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	w := fixedBoxContactWorld(t, doc, floor, box, 0.5)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	zeroTorque := units.KilogramSquareMillimetersPerSecondSquared(0)
	force := dynamics.QuantityVec{X: units.KilogramMillimetersPerSecondSquared(0),
		Y: units.KilogramMillimetersPerSecondSquared(0),
		Z: units.KilogramMillimetersPerSecondSquared(-500)}
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: box, Force: force, Torque: dynamics.QuantityVec{
			X: zeroTorque, Y: zeroTorque, Z: zeroTorque}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Conservation)
	require.Zero(t, report.Conservation.Input.KineticEnergy.Value.Base())
	require.InDelta(t, 5000, report.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 1250, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
	require.InDelta(t, -100, report.Conservation.LoadImpulse.Value.Z.Base(), 1e-9)
	require.Zero(t, report.Conservation.GravityImpulse.Value.Z.Base())
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-4)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, -100, report.Events[0].PreVelocityB.Z.Base(), 1e-6)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-4)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 5, final.Pose.Translation().Z, 2e-6)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, originalFloor, afterFloor)
	require.Equal(t, originalBox, afterBox)
}

func TestRestingBoxUsesPersistentContactTrack(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	originalFloor, err := floor.Bounds()
	require.NoError(t, err)
	originalBox, err := box.Bounds()
	require.NoError(t, err)
	contactRequest := decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6),
	}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), contactRequest)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	duration := units.Seconds(0.1)
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	sweep, err := doc.SweepPair(t.Context(), floor, box, still, still, decad.SweepRequest{
		ContactRequest: contactRequest, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128, StartPolicy: decad.ContinueCertifiedTouch,
	})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
	require.NotNil(t, sweep.ContactTrack)
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(0.5), units.Scalar(1)} {
		manifold, err := sweep.ContactTrack.ManifoldAt(fraction)
		require.NoError(t, err)
		require.Len(t, manifold.Points, 4)
	}
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: contactRequest, TimeResolution: units.Seconds(1e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	gravity := zeroAcceleration()
	gravity.Z = units.MillimetersPerSecondSquared(-1000)
	// The initial touch takes its restitution 0.5 like any contact
	// (docs/multibody-dynamics-design.md §6.5): the first step's kick of
	// −100 mm/s bounces the box up at 50 mm/s, 5 mm by the step end; the
	// second step's kick turns that into −50 mm/s, which lands it back on the
	// floor exactly at the step end, where it bounces at 25 mm/s.
	for step, want := range []struct {
		at, impulse, pre, post, z float64
	}{{0, 150, -100, 50, 5}, {.1, 75, -50, 25, 0}} {
		report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: gravity}, duration)
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
		require.NotNil(t, report.Next)
		require.Len(t, report.Events, 1)
		require.InDelta(t, want.at, report.Events[0].Time.Base(), 1e-9)
		require.InDelta(t, want.impulse, report.Events[0].NormalImpulse.Base(), 1e-6)
		require.InDelta(t, want.pre, report.Events[0].PreVelocityB.Z.Base(), 1e-6)
		require.InDelta(t, want.post, report.Events[0].PostVelocityB.Z.Base(), 1e-6)
		final, ok := report.Next.Body(box)
		require.True(t, ok)
		require.InDelta(t, want.post, final.LinearVelocity.Z.Base(), 1e-6)
		require.InDelta(t, want.z, final.Pose.Translation().Z, 1e-9)
		replayed, err := report.Trace.Sample(duration)
		require.NoError(t, err)
		require.Equal(t, report.Next.Entries(), replayed.Entries())
		state = *report.Next
	}
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, originalFloor, afterFloor)
	require.Equal(t, originalBox, afterBox)
}

func TestZeroRestitutionImpactContinuesAsTouching(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, 100, report.Events[0].NormalImpulse.Base(), 1e-6)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 0, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 0, final.Pose.Translation().Z, 1e-6)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), final.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestStationaryTouchAdvancesWithoutImpulse(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.Equal(t, start.Entries(), report.Next.Entries())
	replayed, err := report.Trace.Sample(units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestStationarySeparatedBoxesAdvanceClear(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := fixedBoxContactWorld(t, doc, floor, box, 0)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(0.1)
	stillFloor := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	stillBox := decad.PoseSegment{From: pose, To: pose, Duration: duration}
	sweep, err := doc.SweepPair(t.Context(), floor, box, stillFloor, stillBox, decad.SweepRequest{
		ContactRequest: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128,
		StartPolicy: decad.ContinueCertifiedTouch,
	})
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, sweep.Outcome)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.Equal(t, start.Entries(), report.Next.Entries())
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), final.Pose,
		decad.ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)})
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}

func TestTwoDynamicBoxesExchangeMomentum(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, -5, 10, 5, 0, 10)
	b := makeBox(t, doc, 35, -5, 45, 5, 0, 10)
	originalA, err := a.Bounds()
	require.NoError(t, err)
	originalB, err := b.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.25))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 10000, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 2500, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.Zero(t, report.Conservation.Input.LinearMomentum.Value.X.Base())
	require.Zero(t, report.Conservation.Completion.LinearMomentum.Value.X.Base())
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
	require.Len(t, report.Events, 1)
	require.InDelta(t, 0.125, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-6)
	aFinal, ok := report.Next.Body(a)
	require.True(t, ok)
	bFinal, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -50, aFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50, bFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 6.25, aFinal.Pose.Translation().X, 1e-6)
	require.InDelta(t, -6.25, bFinal.Pose.Translation().X, 1e-6)
	require.InDelta(t, 100, report.Events[0].PreVelocityA.X.Base(), 1e-6)
	require.InDelta(t, -100, report.Events[0].PreVelocityB.X.Base(), 1e-6)
	require.InDelta(t, -50, report.Events[0].PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, 50, report.Events[0].PostVelocityB.X.Base(), 1e-6)
	replayed, err := report.Trace.Sample(units.Seconds(0.25))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	require.Equal(t, []*decad.Body{a, b}, doc.Bodies())
	afterA, err := a.Bounds()
	require.NoError(t, err)
	afterB, err := b.Bounds()
	require.NoError(t, err)
	require.Equal(t, originalA, afterA)
	require.Equal(t, originalB, afterB)
}

func TestKinematicBoxPushUsesProductionGeometry(t *testing.T) {
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 10, 0, 20, 10, 0, 10)
	beforeDriver, err := driver.Bounds()
	require.NoError(t, err)
	beforeBox, err := box.Bounds()
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	duration := units.Seconds(0.125)
	endPose, err := r3.Translation(r3.Vec{X: 1.25})
	require.NoError(t, err)
	path := decad.PoseSegment{From: r3.Identity(), To: endPose, Duration: duration}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	initial, err := doc.ContactPair(t.Context(), driver, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initial.Relation)
	require.NotNil(t, initial.Manifold)
	sweepRequest := decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
		MaxPoseEvaluations: 128}
	first, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: duration}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, first.Outcome)
	require.NotNil(t, first.Event)
	require.NotNil(t, first.Event.Manifold)
	sweepRequest.StartPolicy = decad.ContinueCertifiedTouch
	postVelocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	ideal, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: postVelocity, AngularVelocity: zeroAngular(t), Duration: duration}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, ideal.Outcome)
	require.NotNil(t, ideal.ContactTrack)
	rounded, err := doc.SweepPair(t.Context(), driver, box, path, path, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, rounded.Outcome)
	require.NotNil(t, rounded.ContactTrack)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: driver, Role: dynamics.Kinematic, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{
		Gravity: zeroAcceleration(), Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}},
	}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 10, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.Equal(t, dynamics.ContactImpact, report.Events[0].Kind)
	require.Equal(t, postVelocity, report.Events[0].PreVelocityA)
	require.Equal(t, postVelocity, report.Events[0].PostVelocityA)
	require.Equal(t, zeroVelocity(), report.Events[0].PreVelocityB)
	require.Equal(t, postVelocity, report.Events[0].PostVelocityB)
	checkpoint, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	checkpointDriver, ok := checkpoint.Body(driver)
	require.True(t, ok)
	require.Equal(t, zeroVelocity(), checkpointDriver.LinearVelocity)
	checkpointBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, postVelocity, checkpointBox.LinearVelocity)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 1.25}, finalDriver.Pose.Translation())
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	require.Equal(t, r3.Vec{X: 1.25}, finalBox.Pose.Translation())
	require.Equal(t, postVelocity, finalBox.LinearVelocity)
	endpoint, err := doc.ContactPair(t.Context(), driver, box, finalDriver.Pose, finalBox.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endpoint.Relation)
	require.Equal(t, []*decad.Body{driver, box}, doc.Bodies())
	afterDriver, err := driver.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, beforeDriver, afterDriver)
	require.Equal(t, beforeBox, afterBox)
}

func TestTwoDynamicBoxesUseBothMasses(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, -5, 10, 5, 0, 10)
	b := makeBox(t, doc, 35, -5, 45, 5, 0, 10)
	densityA := units.KilogramsPerCubicMillimeter(0.001)
	densityB := units.KilogramsPerCubicMillimeter(0.002)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &densityA, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &densityB, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.25))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 200, report.Events[0].NormalImpulse.Base(), 1e-6)
	aFinal, ok := report.Next.Body(a)
	require.True(t, ok)
	bFinal, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -100, aFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 0, bFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 0, aFinal.Pose.Translation().X, 1e-6)
	require.InDelta(t, -12.5, bFinal.Pose.Translation().X, 1e-6)
}

func TestClearIdealPathRejectsRoundedOverlappingEndpoint(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, 10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, math.Nextafter(1, 2))
	density := units.KilogramsPerCubicMillimeter(0.001)
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(30),
		}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	duration := units.Seconds(0.3)
	ideal, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(30)},
			AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: config.Step.Contact,
			TimeResolution: config.Step.TimeResolution, MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, ideal.Outcome)
	roundedPose, err := r3.Translation(r3.Vec{Z: 30 * duration.Base()})
	require.NoError(t, err)
	actual, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), roundedPose, config.Step.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, actual.Relation)

	report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
}

func TestOffCenterBoxImpactRefusesOmittedSpin(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -5, -5, 5-1e-6, 5, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(1e6)
	contactReq := decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6),
	}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), contactReq)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	meanX := 0.0
	for _, point := range contact.Manifold.Points {
		meanX += point.OnB.Value.X
	}
	meanX /= float64(len(contact.Manifold.Points))
	require.InDelta(t, -5e-7, meanX, 1e-9)

	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
				Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}},
		},
		Step: dynamics.StepConfig{
			Contact: contactReq, TimeResolution: units.Seconds(1e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-8),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-2),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100),
		}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	// The patch's corners sit 1e-6 mm off-center; the solve splits the
	// impulse unevenly so its pressure center stays under the mass center,
	// and the 1e9 kg box leaves at 0.5·100 mm/s with no spin.
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.InDelta(t, .1, event.Time.Base(), config.Step.TimeResolution.Base())
	require.InDelta(t, 1.5e11, event.NormalImpulse.Base(), 1e-2)
	moment := 0.0
	for i, point := range event.PointImpulses {
		moment += point.Normal.Base() * event.Manifold.Points[i].OnB.Value.X
	}
	require.InDelta(t, 0, moment, 1e-1)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.Equal(t, zeroAngular(t), final.AngularVelocity)
}
