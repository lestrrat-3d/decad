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
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual: units.MillimetersPerSecond(1e-6), ImpulseResidual: impulseLimit,
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
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
	require.Len(t, report.Events, 1)
	event := report.Events[0]
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
	_, err = report.Trace.Sample(units.Seconds(0.05))
	require.ErrorIs(t, err, dynamics.ErrUnsupported)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	afterFloor, err := floor.Bounds()
	require.NoError(t, err)
	afterBox, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, originalFloor, afterFloor)
	require.Equal(t, originalBox, afterBox)
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
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
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
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Contains(t, report.Diagnostics[0].Reason, "off-center impulse")
}
