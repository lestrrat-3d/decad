package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFixedFloorInteriorSlidingImpactUsesRealGeometry(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	velocity := QuantityVec{X: units.MillimetersPerSecond(200), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(-160)}
	dt := units.Seconds(.125)
	floorPath := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroV,
		AngularVelocity: zeroW, Duration: dt}
	boxPath := decad.RigidDriftSegment{From: pose, Center: pose.Apply(mass.Center.Value),
		LinearVelocity: velocity, AngularVelocity: zeroW, Duration: dt}
	sweep, err := doc.SweepPair(t.Context(), floor, box, floorPath, boxPath,
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.NotNil(t, sweep.Event)
	require.Len(t, sweep.Event.Manifold.Points, 4)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.25)}
	cfg := StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2}
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, dt)
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, ContactImpact, event.Kind)
	require.InDelta(t, .0625, event.Time.Base(), 1e-9)
	require.InDelta(t, 160, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -40, event.TangentImpulse.X.Base(), 1e-6)
	require.Len(t, event.PointImpulses, 4)
	require.NotNil(t, event.Solver)
	var summedNormal, summedTangent float64
	for _, point := range event.PointImpulses {
		require.Positive(t, point.Normal.Base())
		require.LessOrEqual(t, math.Hypot(point.Tangent.X.Base(), point.Tangent.Y.Base()),
			.25*point.Normal.Base()+1e-6)
		require.LessOrEqual(t, math.Abs(point.Tangent.Y.Base()), 1e-6)
		require.InDelta(t, -.25*point.Normal.Base(), point.Tangent.X.Base(), 1e-6)
		summedNormal += point.Normal.Base()
		summedTangent += point.Tangent.X.Base()
	}
	require.InDelta(t, event.NormalImpulse.Base(), summedNormal, 1e-6)
	require.InDelta(t, event.TangentImpulse.X.Base(), summedTangent, 1e-6)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Torque, report.Conservation.Input.KineticEnergy.Value.Kind())
	require.Equal(t, units.Torque, report.Conservation.Completion.KineticEnergy.Value.Kind())
	require.GreaterOrEqual(t, report.Conservation.Input.KineticEnergy.Bound.Base(), 0.0)
	require.GreaterOrEqual(t, report.Conservation.Completion.KineticEnergy.Bound.Base(), 0.0)
	require.InDelta(t, 32800, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 12800, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, -40, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 160, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 160, final.LinearVelocity.X.Base(), 1e-6)
	require.Zero(t, final.LinearVelocity.Z.Base())
	require.Equal(t, zeroW, final.AngularVelocity)
	require.InDelta(t, 22.5, final.Pose.Translation().X, 1e-6)
	require.Zero(t, final.Pose.Translation().Z)
	endContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), final.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, endContact.Relation)
	// The certified bracket can place the event slightly after the analytic touch.
	for _, sampleCase := range []struct {
		at, x, z, speed float64
	}{
		{at: .03125, x: 6.25, z: 5, speed: 200},
		{at: event.Time.Base(), x: 200 * event.Time.Base(), z: 0, speed: 160},
		{at: .1, x: 18.5, z: 0, speed: 160},
		{at: .125, x: 22.5, z: 0, speed: 160},
	} {
		sample, sampleErr := report.Trace.Sample(units.Seconds(sampleCase.at))
		require.NoError(t, sampleErr)
		sampledBox, exists := sample.Body(box)
		require.True(t, exists)
		require.InDelta(t, sampleCase.x, sampledBox.Pose.Translation().X, 1e-6)
		require.InDelta(t, sampleCase.z, sampledBox.Pose.Translation().Z, 1e-6)
		require.InDelta(t, sampleCase.speed, sampledBox.LinearVelocity.X.Base(), 1e-6)
	}
	limited := cfg
	limited.MaxEvents = 1
	limitedWorld, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: material},
	}, Step: limited})
	require.NoError(t, err)
	limitedStart, err := limitedWorld.NewState(start.Entries())
	require.NoError(t, err)
	limitedStep, err := limitedWorld.Step(t.Context(), limitedStart, StepInput{Gravity: gravity}, dt)
	require.NoError(t, err)
	require.Equal(t, Undecided, limitedStep.Status)
	require.Nil(t, limitedStep.Next)
}
