package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodyTwoDynamicSphereImpact(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	remote := makeBox(t, doc, -100, -100, 100, 100, -100, 10)
	originalBodies := doc.Bodies()
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	velocityA := dynamics.QuantityVec{X: units.MillimetersPerSecond(30),
		Y: units.MillimetersPerSecond(40), Z: units.MillimetersPerSecond(0)}
	velocityB := dynamics.QuantityVec{X: units.MillimetersPerSecond(-30),
		Y: units.MillimetersPerSecond(-40), Z: units.MillimetersPerSecond(0)}
	zeroSpin := zeroAngular(t)
	contactA, err := r3.Translation(r3.Vec{X: -3, Y: -4})
	require.NoError(t, err)
	contactB, err := r3.Translation(r3.Vec{X: 3, Y: 4})
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), a, b, contactA, contactB, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)
	sweep, err := doc.SweepPair(t.Context(), a, b,
		decad.RigidDriftSegment{From: pa, LinearVelocity: velocityA,
			AngularVelocity: zeroSpin, Duration: units.Seconds(.2)},
		decad.RigidDriftSegment{From: pb, LinearVelocity: velocityB,
			AngularVelocity: zeroSpin, Duration: units.Seconds(.2)},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.InDelta(t, .1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	for _, moving := range []struct {
		body     *decad.Body
		pose     r3.Transform
		velocity dynamics.QuantityVec
	}{{a, pa, velocityA}, {b, pb, velocityB}} {
		clearSweep, clearErr := doc.SweepPair(t.Context(), moving.body, remote,
			decad.RigidDriftSegment{From: moving.pose, LinearVelocity: moving.velocity,
				AngularVelocity: zeroSpin, Duration: units.Seconds(.2)},
			decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.2)},
			decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
				MaxPoseEvaluations: 128})
		require.NoError(t, clearErr)
		require.Equal(t, decad.SweepClear, clearSweep.Outcome, "cause=%v", clearSweep.Cause)
	}
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
		MaxIterations: 8, MaxEvents: 2}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: remote, Role: dynamics.Fixed, Material: material},
		},
		Step: step,
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: velocityA, AngularVelocity: zeroSpin},
		{Body: b, Pose: pb, LinearVelocity: velocityB, AngularVelocity: zeroSpin},
		{Body: remote, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroSpin},
	})
	require.NoError(t, err)
	result, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, result.Status, "%+v", result.Diagnostics)
	require.Len(t, result.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: a, B: b}, result.Events[0].Pair)
	require.InDelta(t, 75, result.Events[0].NormalImpulse.Base(), 1e-5)
	require.NotNil(t, result.Conservation)
	require.InDelta(t, 2500, result.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 625, result.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 0, result.Conservation.ContactImpulse.Value.X.Base(), 1e-9)
	require.InDelta(t, 0, result.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-9)
	endA, ok := result.Next.Body(a)
	require.True(t, ok)
	endB, ok := result.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -4.5, endA.Pose.Translation().X, 2e-6)
	require.InDelta(t, -6, endA.Pose.Translation().Y, 2e-6)
	require.InDelta(t, 4.5, endB.Pose.Translation().X, 2e-6)
	require.InDelta(t, 6, endB.Pose.Translation().Y, 2e-6)
	require.InDelta(t, -15, endA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 20, endB.LinearVelocity.Y.Base(), 1e-6)
	for _, sample := range []struct {
		at, ax, bx, avx float64
	}{{.05, -4.5, 4.5, 30}, {result.Events[0].Time.Base(), -3, 3, -15},
		{.15, -3.75, 3.75, -15}, {.2, -4.5, 4.5, -15}} {
		sampled, sampleErr := result.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, sampleErr)
		require.Len(t, sampled.Entries(), 3)
		movingA, _ := sampled.Body(a)
		movingB, _ := sampled.Body(b)
		require.InDelta(t, sample.ax, movingA.Pose.Translation().X, 2e-6)
		require.InDelta(t, sample.bx, movingB.Pose.Translation().X, 2e-6)
		require.InDelta(t, sample.avx, movingA.LinearVelocity.X.Base(), 1e-6, "time=%g", sample.at)
		fixed, found := sampled.Body(remote)
		require.True(t, found)
		require.Equal(t, r3.Identity(), fixed.Pose)
	}
	require.Equal(t, originalBodies, doc.Bodies())

	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: remote, Role: dynamics.Fixed, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		}, Step: step,
	})
	require.NoError(t, err)
	reversedStart, err := reversed.NewState(start.Entries())
	require.NoError(t, err)
	reversedResult, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedResult.Status, "%+v", reversedResult.Diagnostics)
	require.Equal(t, dynamics.BodyPair{A: b, B: a}, reversedResult.Events[0].Pair)
	replayed, err := reversedResult.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	reversedA, _ := replayed.Body(a)
	require.InDelta(t, -3.75, reversedA.Pose.Translation().X, 2e-6)

	step.MaxEvents = 1
	limited, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: remote, Role: dynamics.Fixed, Material: material},
		}, Step: step,
	})
	require.NoError(t, err)
	limitedStart, err := limited.NewState(start.Entries())
	require.NoError(t, err)
	limitedResult, err := limited.Step(t.Context(), limitedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, limitedResult.Status)
	require.Nil(t, limitedResult.Next)
}

func TestThreeBodyTwoDynamicLoadsKickBothBodiesOnce(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	floor := makeBox(t, doc, -100, -100, 100, 100, -100, 10)
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	mass := exactSphereMass()
	heavy := exactSphereMass()
	heavy.Mass.Value = units.Kilograms(2)
	heavy.Inertia.XX.Value = units.KilogramSquareMillimeters(20)
	heavy.Inertia.YY.Value = units.KilogramSquareMillimeters(20)
	heavy.Inertia.ZZ.Value = units.KilogramSquareMillimeters(20)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &heavy, Material: material},
		},
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: pb, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	force := func(x float64) dynamics.QuantityVec {
		return dynamics.QuantityVec{X: units.KilogramMillimetersPerSecondSquared(x),
			Y: units.KilogramMillimetersPerSecondSquared(0),
			Z: units.KilogramMillimetersPerSecondSquared(0)}
	}
	zeroTorque := dynamics.QuantityVec{X: units.KilogramSquareMillimetersPerSecondSquared(0),
		Y: units.KilogramSquareMillimetersPerSecondSquared(0),
		Z: units.KilogramSquareMillimetersPerSecondSquared(0)}
	input := dynamics.StepInput{Gravity: zeroAcceleration(), Loads: []dynamics.BodyLoad{
		{Body: a, Force: force(-100), Torque: zeroTorque},
		{Body: b, Force: force(100), Torque: zeroTorque},
	}}
	report, err := w.Step(t.Context(), start, input, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	endA, _ := report.Next.Body(a)
	endB, _ := report.Next.Body(b)
	require.InDelta(t, -7, endA.Pose.Translation().X, 1e-9)
	require.InDelta(t, 6.5, endB.Pose.Translation().X, 1e-9)
	require.InDelta(t, -10, endA.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, 5, endB.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, 75, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 0, report.Conservation.LoadImpulse.Value.X.Base(), 1e-9)
	sampled, err := report.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	sampleA, _ := sampled.Body(a)
	sampleB, _ := sampled.Body(b)
	require.InDelta(t, -6.5, sampleA.Pose.Translation().X, 1e-9)
	require.InDelta(t, 6.25, sampleB.Pose.Translation().X, 1e-9)
}

func TestThreeBodyTwoDynamicFixedImpactKeepsOtherBodyMoving(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	ball := makeBall(t, doc)
	boxPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	ballPose, err := r3.Translation(r3.Vec{X: 50, Z: 50})
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	density := units.KilogramsPerCubicMillimeter(.001)
	ballMass := exactSphereMass()
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &ballMass, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: box, B: ball}},
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: ballPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(10), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: boxPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, report.Events[0].Pair)
	require.Equal(t, []dynamics.BodyPair{{A: ball, B: box}}, report.Excluded)
	endBall, _ := report.Next.Body(ball)
	endBox, _ := report.Next.Body(box)
	require.InDelta(t, 52, endBall.Pose.Translation().X, 1e-9)
	require.InDelta(t, 5, endBox.Pose.Translation().Z, 2e-6)
	require.InDelta(t, 1300, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	for _, sample := range []struct{ at, ballX, boxZ float64 }{
		{.05, 50.5, 5}, {.15, 51.5, 2.5}, {.2, 52, 5},
	} {
		replayed, sampleErr := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, sampleErr)
		movingBall, _ := replayed.Body(ball)
		movingBox, _ := replayed.Body(box)
		require.InDelta(t, sample.ballX, movingBall.Pose.Translation().X, 1e-9)
		require.InDelta(t, sample.boxZ, movingBox.Pose.Translation().Z, 2e-6)
	}
}

func TestThreeBodyTwoDynamicAllExcludedDriftsIndependently(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	fixed := makeBox(t, doc, -5, -5, 5, 5, -5, 10)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: fixed, Role: dynamics.Fixed, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: b, B: a}, {A: fixed, B: a}, {A: b, B: fixed}},
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 1},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(10), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-10), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: fixed, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.Len(t, report.Excluded, 3)
	endA, _ := report.Next.Body(a)
	endB, _ := report.Next.Body(b)
	require.InDelta(t, 1, endA.Pose.Translation().X, 1e-9)
	require.InDelta(t, -1, endB.Pose.Translation().X, 1e-9)
	require.InDelta(t, 100, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 0, report.Conservation.ContactImpulse.Value.X.Base(), 1e-9)
	interior, err := report.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	sampleA, _ := interior.Body(a)
	sampleB, _ := interior.Body(b)
	require.InDelta(t, .5, sampleA.Pose.Translation().X, 1e-9)
	require.InDelta(t, -.5, sampleB.Pose.Translation().X, 1e-9)
}

func TestThreeBodyTwoDynamicOverlappingPairEventsRemainUndecided(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	floor := makeBox(t, doc, -100, -100, 100, 100, -100, 100)
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8, Z: 15})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8, Z: 15})
	require.NoError(t, err)
	velocityA := dynamics.QuantityVec{X: units.MillimetersPerSecond(30),
		Y: units.MillimetersPerSecond(40), Z: units.MillimetersPerSecond(-100)}
	velocityB := dynamics.QuantityVec{X: units.MillimetersPerSecond(-30),
		Y: units.MillimetersPerSecond(-40), Z: units.MillimetersPerSecond(-100)}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	sweepRequest := decad.SweepRequest{ContactRequest: request,
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128}
	segmentA := decad.RigidDriftSegment{From: pa, LinearVelocity: velocityA,
		AngularVelocity: zeroAngular(t), Duration: units.Seconds(.2)}
	segmentB := decad.RigidDriftSegment{From: pb, LinearVelocity: velocityB,
		AngularVelocity: zeroAngular(t), Duration: units.Seconds(.2)}
	sweep, sweepErr := doc.SweepPair(t.Context(), a, b, segmentA, segmentB, sweepRequest)
	require.NoError(t, sweepErr)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.InDelta(t, .1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	fixedSegment := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(),
		Duration: units.Seconds(.2)}
	for _, moving := range []struct {
		body    *decad.Body
		segment decad.RigidDriftSegment
	}{{a, segmentA}, {b, segmentB}} {
		sweep, sweepErr := doc.SweepPair(t.Context(), moving.body, floor, moving.segment,
			fixedSegment, sweepRequest)
		require.NoError(t, sweepErr)
		require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
		require.InDelta(t, .1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	}
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material},
		},
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 3},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: velocityA, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: pb, LinearVelocity: velocityB, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Empty(t, report.Events)
}

func TestThreeBodyTwoDynamicSequentialFloorImpacts(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	pa, err := r3.Translation(r3.Vec{X: -40, Z: 15})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 40, Z: 25})
	require.NoError(t, err)
	falling := dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)}
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	mass := exactSphereMass()
	step := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
		MaxIterations: 8}
	makeWorld := func(maxEvents int) (*dynamics.World, dynamics.State) {
		step.MaxEvents = maxEvents
		world, worldErr := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
			Bodies: []dynamics.RigidBody{
				{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
				{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
				{Body: floor, Role: dynamics.Fixed, Material: material},
			}, Step: step,
		})
		require.NoError(t, worldErr)
		state, stateErr := world.NewState([]dynamics.BodyState{
			{Body: a, Pose: pa, LinearVelocity: falling, AngularVelocity: zeroAngular(t)},
			{Body: b, Pose: pb, LinearVelocity: falling, AngularVelocity: zeroAngular(t)},
			{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
				AngularVelocity: zeroAngular(t)},
		})
		require.NoError(t, stateErr)
		return world, state
	}
	for _, target := range []struct {
		body  *decad.Body
		x, at float64
	}{{a, -40, .1}, {b, 40, .2}} {
		atContact, poseErr := r3.Translation(r3.Vec{X: target.x, Z: 5})
		require.NoError(t, poseErr)
		contact, contactErr := doc.ContactPair(t.Context(), target.body, floor,
			atContact, r3.Identity(), request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, contact.Relation)
		startPose := pa
		if target.body == b {
			startPose = pb
		}
		sweep, sweepErr := doc.SweepPair(t.Context(), target.body, floor,
			decad.RigidDriftSegment{From: startPose, LinearVelocity: falling,
				AngularVelocity: zeroAngular(t), Duration: units.Seconds(.3)},
			decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.3)},
			decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
				MaxPoseEvaluations: 128})
		require.NoError(t, sweepErr)
		require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
		require.InDelta(t, target.at, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	}
	separate, err := doc.SweepPair(t.Context(), a, b,
		decad.RigidDriftSegment{From: pa, LinearVelocity: falling,
			AngularVelocity: zeroAngular(t), Duration: units.Seconds(.3)},
		decad.RigidDriftSegment{From: pb, LinearVelocity: falling,
			AngularVelocity: zeroAngular(t), Duration: units.Seconds(.3)},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, separate.Outcome)
	limited, limitedStart := makeWorld(2)
	refused, err := limited.Step(t.Context(), limitedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.3))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, refused.Status)
	require.Nil(t, refused.Next)
	require.Empty(t, refused.Events)
	require.Len(t, refused.Diagnostics, 1)
	require.Contains(t, refused.Diagnostics[0].Reason, "maximum contact events reached")

	world, start := makeWorld(3)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.3))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: a, B: floor}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: b, B: floor}, report.Events[1].Pair)
	require.InDelta(t, .1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, .2, report.Events[1].Time.Base(), 1e-9)
	for _, sample := range []struct{ at, az, bz, avz, bvz float64 }{
		{.05, 10, 20, -100, -100}, {.15, 7.5, 10, 50, -100},
		{.25, 12.5, 7.5, 50, 50}, {.3, 15, 10, 50, 50},
	} {
		replayed, replayErr := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, replayErr)
		stateA, _ := replayed.Body(a)
		stateB, _ := replayed.Body(b)
		fixed, _ := replayed.Body(floor)
		require.InDelta(t, sample.az, stateA.Pose.Translation().Z, 2e-6)
		require.InDelta(t, sample.bz, stateB.Pose.Translation().Z, 2e-6)
		require.InDelta(t, sample.avz, stateA.LinearVelocity.Z.Base(), 1e-6)
		require.InDelta(t, sample.bvz, stateB.LinearVelocity.Z.Base(), 1e-6)
		require.Equal(t, r3.Identity(), fixed.Pose)
	}
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 10000, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 2500, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 300, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-5)
}
