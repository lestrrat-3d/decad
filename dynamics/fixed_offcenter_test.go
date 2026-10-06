package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFixedOffcenterSuppliedMassReboundsWithCertifiedSpin(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -5, -5, 5, 5, -10, 10)
	box := makeBox(t, doc, 0, -5, 10, 5, 0, 10)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(.001))
	require.NoError(t, err)
	mass.Inertia.XX.Value = units.KilogramSquareMillimeters(25)
	mass.Inertia.YY.Value = units.KilogramSquareMillimeters(40)
	mass.Inertia.ZZ.Value = units.KilogramSquareMillimeters(25)
	mass.Center.Value.X = 7
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	initial := dynamics.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(-100)}
	idle := zeroVelocity()
	duration := units.Seconds(.01)
	first, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: initial, AngularVelocity: zeroAngular(t), Duration: duration},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, first.Outcome)
	material := dynamics.Material{Restitution: units.Scalar(1), Friction: units.Scalar(0)}
	config := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: box, Role: dynamics.Dynamic, Supplied: &mass, Material: material}}, Step: config})
	require.NoError(t, err)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: idle, AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: initial, AngularVelocity: zeroAngular(t)}})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Conservation)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.InDelta(t, 2000.0/11, event.NormalImpulse.Base(), 1e-6)
	require.Len(t, event.PointImpulses, 4)
	for i, impulse := range event.PointImpulses {
		if contact.Manifold.Points[i].OnB.Value.X == 5 {
			require.InDelta(t, 1000.0/11, impulse.Normal.Base(), 1e-6)
		} else {
			require.Zero(t, impulse.Normal.Base())
		}
	}
	require.LessOrEqual(t, event.Solver.NormalResidual.Base(), config.VelocityResidual.Base())
	require.GreaterOrEqual(t, event.Solver.AngularUpper.Base(), event.PostAngularVelocityB.Y.Base())
	require.InDelta(t, 900.0/11, event.PostVelocityB.Z.Base(), 1e-6)
	require.InDelta(t, 100.0/11, event.PostAngularVelocityB.Y.Base(), 1e-6)
	require.InDelta(t, report.Conservation.Input.KineticEnergy.Value.Base(),
		report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	post := dynamics.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(900.0 / 11)}
	spin := dynamics.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(100.0 / 11),
		Z: units.RadiansPerSecond(0)}
	departure, err := doc.SweepPair(t.Context(), floor, box,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: post, AngularVelocity: spin, Duration: duration},
		decad.SweepRequest{ContactRequest: request, StartPolicy: decad.ContinueSeparatingTouch,
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, departure.Outcome, "cause=%v", departure.Cause)
	for _, elapsed := range []float64{1e-8, .005, .01} {
		replay, replayErr := report.Trace.Sample(units.Seconds(elapsed))
		require.NoError(t, replayErr)
		fixedState, ok := replay.Body(floor)
		require.True(t, ok)
		movingState, ok := replay.Body(box)
		require.True(t, ok)
		relation, contactErr := doc.ContactPair(t.Context(), floor, box,
			fixedState.Pose, movingState.Pose, request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactSeparated, relation.Relation, "time=%v", elapsed)
	}
	endpoint, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, *report.Next, endpoint)

	uncertain := mass
	uncertain.Inertia.YY.Bound = units.KilogramSquareMillimeters(1)
	uncertain.Inertia.YY.Exactness = decad.Approximate
	uncertainWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &uncertain, Material: material}}, Step: config})
	require.NoError(t, err)
	uncertainStart, err := uncertainWorld.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: idle, AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: initial, AngularVelocity: zeroAngular(t)}})
	require.NoError(t, err)
	refused, err := uncertainWorld.Step(t.Context(), uncertainStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, refused.Status)
	require.Nil(t, refused.Next)

	centered := mass
	centered.Center.Value.X = 5
	centeredWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &centered, Material: material}}, Step: config})
	require.NoError(t, err)
	centeredStart, err := centeredWorld.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: idle, AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: initial, AngularVelocity: zeroAngular(t)}})
	require.NoError(t, err)
	centeredStep, err := centeredWorld.Step(t.Context(), centeredStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	// With the mass center over the patch edge x = 5 the edge corners carry
	// the whole elastic impulse, 200 kg·mm/s, through the center: the box
	// leaves at +100 mm/s with no spin.
	require.Equal(t, dynamics.Advanced, centeredStep.Status, "%+v", centeredStep.Diagnostics)
	require.Len(t, centeredStep.Events, 1)
	require.InDelta(t, 200, centeredStep.Events[0].NormalImpulse.Base(), 1e-6)
	for i, impulse := range centeredStep.Events[0].PointImpulses {
		edge := 0.0
		if centeredStep.Events[0].Manifold.Points[i].OnB.Value.X == 5 {
			edge = 100
		}
		require.InDelta(t, edge, impulse.Normal.Base(), 1e-6, "point %d", i)
	}
	rebound, ok := centeredStep.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 100, rebound.LinearVelocity.Z.Base(), 1e-6)
	require.Equal(t, zeroAngular(t), rebound.AngularVelocity)
}
