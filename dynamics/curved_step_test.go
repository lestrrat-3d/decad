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

func exactSphereMass() decad.MassProperties {
	reading := func(value units.Value) decad.Measurement {
		return decad.Measurement{Value: value, Bound: units.New(0, value.Unit()), Exactness: decad.Exact}
	}
	inertia := reading(units.KilogramSquareMillimeters(10))
	zeroInertia := reading(units.KilogramSquareMillimeters(0))
	return decad.MassProperties{
		Mass:   reading(units.Kilograms(1)),
		Center: decad.VecMeasurement{Value: r3.Vec{}, Bound: units.Millimeters(0), Exactness: decad.Exact},
		Inertia: decad.InertiaReading{
			XX: inertia, YY: inertia, ZZ: inertia,
			XY: zeroInertia, XZ: zeroInertia, YZ: zeroInertia,
		},
	}
}

func makeBall(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	left := s.CreatePoint(-5, 0)
	s.Fix(left)
	right := s.CreatePoint(5, 0)
	center := s.CreatePoint(0, 0)
	s.CreateLine(left, right)
	s.CreateArc(center, right, left)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ball, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return ball
}

func TestSourceSphereRotatedBoxFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 20)
	ball := makeBall(t, doc)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	normal := turn.ApplyDir(r3.Vec{Z: 1})
	center := normal.Scale(20)
	startPose, err := r3.Translation(center)
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	pair, err := doc.ContactPair(t.Context(), floor, ball, turn, startPose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, pair.Relation)
	require.NotNil(t, pair.Gap)
	require.InDelta(t, 5, pair.Gap.Value.Base(), 1e-12)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-100 * normal.X),
		Y: units.MillimetersPerSecond(-100 * normal.Y),
		Z: units.MillimetersPerSecond(-100 * normal.Z)}
	endPose, err := r3.Translation(center.Add(normal.Scale(-6)))
	require.NoError(t, err)
	sweep, err := doc.SweepPair(t.Context(), floor, ball,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(.06)},
		decad.PoseSegment{From: startPose, To: endPose, Duration: units.Seconds(.06)},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.NotNil(t, sweep.Event.Manifold)
	require.Len(t, sweep.Event.Manifold.Points, 1)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	mass := exactSphereMass()
	cfg := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
			MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	final, ok := step.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 50*normal.X, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50*normal.Z, final.LinearVelocity.Z.Base(), 1e-6)
	before, err := step.Trace.Sample(units.Seconds(.03))
	require.NoError(t, err)
	beforeBall, ok := before.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 17, beforeBall.Pose.Translation().Dot(normal), 1e-6)
	require.Equal(t, velocity, beforeBall.LinearVelocity)
	after, err := step.Trace.Sample(units.Seconds(.055))
	require.NoError(t, err)
	afterBall, ok := after.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 15.25, afterBall.Pose.Translation().Dot(normal), 1e-5)
	require.Equal(t, final.LinearVelocity, afterBall.LinearVelocity)
	reversed, err := doc.ContactPair(t.Context(), ball, floor, endPose, turn, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, reversed.Relation)
	require.NotNil(t, reversed.Manifold)
	require.Len(t, reversed.Manifold.Points, 1)
	require.Less(t, reversed.Manifold.Points[0].Normal.Value.Z, 0.0)
	cornerPose, err := r3.Translation(turn.Apply(r3.Vec{X: 18, Z: 14}))
	require.NoError(t, err)
	corner, err := doc.ContactPair(t.Context(), floor, ball, turn, cornerPose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, corner.Relation)
	require.Nil(t, corner.Manifold)
	require.Equal(t, decad.ContactAmbiguousFeature, corner.Reason)
	edgeStart, err := r3.Translation(turn.Apply(r3.Vec{X: 18, Z: 20}))
	require.NoError(t, err)
	edgeEnd, err := r3.Translation(turn.Apply(r3.Vec{X: 18, Z: 14}))
	require.NoError(t, err)
	edgeSweep, err := doc.SweepPair(t.Context(), floor, ball,
		decad.PoseSegment{From: turn, To: turn, Duration: units.Seconds(.06)},
		decad.PoseSegment{From: edgeStart, To: edgeEnd, Duration: units.Seconds(.06)},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, edgeSweep.Outcome)
	tight := req
	tight.PointResolution = units.Millimeters(1e-17)
	tooTight, err := doc.ContactPair(t.Context(), floor, ball, turn, endPose, tight)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, tooTight.Relation)
	require.Nil(t, tooTight.Manifold)
	require.Equal(t, decad.ContactPointTooCoarse, tooTight.Reason)
	tangent := velocity
	tangent.Y = units.MillimetersPerSecond(10)
	tangentStart, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: tangent, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	tangentStep, err := w.Step(t.Context(), tangentStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, tangentStep.Status, "%+v", tangentStep.Diagnostics)
	require.Len(t, tangentStep.Events, 1)
	tangentFinal, found := tangentStep.Next.Body(ball)
	require.True(t, found)
	require.Equal(t, 10.0, tangentFinal.LinearVelocity.Y.Base())
	cfg.Bodies[0], cfg.Bodies[1] = cfg.Bodies[1], cfg.Bodies[0]
	reverseWorld, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	reverseStart, err := reverseWorld.NewState([]dynamics.BodyState{
		{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	reverseStep, err := reverseWorld.Step(t.Context(), reverseStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reverseStep.Status, "%+v", reverseStep.Diagnostics)
	require.Len(t, reverseStep.Events, 1)
	reverseFinal, found := reverseStep.Next.Body(ball)
	require.True(t, found)
	require.InDelta(t, 50*normal.X, reverseFinal.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50*normal.Z, reverseFinal.LinearVelocity.Z.Base(), 1e-6)
	second, err := w.Step(t.Context(), *step.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.01))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
	skewTurn, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(45))
	require.NoError(t, err)
	unsupported, err := doc.ContactPair(t.Context(), floor, ball, skewTurn, startPose, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactUndecided, unsupported.Relation)
	require.Equal(t, decad.ContactPayloadUnsupported, unsupported.Reason)
	t.Run("impact speed straddles threshold", func(t *testing.T) {
		cfg.Bodies[0], cfg.Bodies[1] = cfg.Bodies[1], cfg.Bodies[0]
		cfg.Step.ImpactSpeed = units.MillimetersPerSecond(math.Nextafter(100, math.Inf(-1)))
		thresholdWorld, worldErr := dynamics.NewWorld(t.Context(), doc, cfg)
		require.NoError(t, worldErr)
		thresholdStart, stateErr := thresholdWorld.NewState([]dynamics.BodyState{
			{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
			{Body: ball, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		})
		require.NoError(t, stateErr)
		thresholdStep, stepErr := thresholdWorld.Step(t.Context(), thresholdStart,
			dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
		require.NoError(t, stepErr)
		require.Equal(t, dynamics.Undecided, thresholdStep.Status)
		require.Nil(t, thresholdStep.Next)
		// §12: the island's restitution-target gate refuses the speed whose
		// enclosure straddles −ImpactSpeed, at the impact time, with
		// ImpactSpeed as its limit.
		require.Len(t, thresholdStep.Diagnostics, 1)
		require.Equal(t, dynamics.StepIslandResidual, thresholdStep.Diagnostics[0].Code)
		require.Contains(t, thresholdStep.Diagnostics[0].Reason, "restitution target")
		require.Equal(t, cfg.Step.ImpactSpeed, thresholdStep.Diagnostics[0].Limit)
		require.InDelta(t, .05, thresholdStep.Diagnostics[0].From.Base(), cfg.Step.TimeResolution.Base())
	})
}

func TestRotatedSphereBoxOffCenterMassNeedsPointMotionProof(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 20)
	ball := makeBall(t, doc)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	normal := turn.ApplyDir(r3.Vec{Z: 1})
	pose, err := r3.Translation(normal.Scale(20))
	require.NoError(t, err)
	mass := exactSphereMass()
	mass.Center.Value = r3.Vec{Y: 2}
	mass.Inertia.XX.Value = units.KilogramSquareMillimeters(5e8)
	mass.Inertia.YY.Value = units.KilogramSquareMillimeters(5e8)
	mass.Inertia.ZZ.Value = units.KilogramSquareMillimeters(5e8)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-100 * normal.X),
			Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100 * normal.Z)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.06))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, step.Status)
	require.Nil(t, step.Next)
	// §12: the off-center sphere's spinning departure has no certified sweep
	// after its impact at 0.05 s.
	require.Len(t, step.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, step.Diagnostics[0].Code)
	require.InDelta(t, .05, step.Diagnostics[0].From.Base(), 1e-8)
}

func TestSphereDensityMassRebound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	ball := makeBall(t, doc)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := ball.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 4*math.Pi*125*0.001/3, mass.Mass.Value.Base(), 1e-12)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Density: &density, Material: material},
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 15})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: pose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, mass.Mass.Value.Base()*150, report.Events[0].NormalImpulse.Base(), 1e-4)
	final, ok := report.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
}

func TestSourceSpherePairDensityImpact(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	density := units.KilogramsPerCubicMillimeter(0.001)
	mass, err := a.MassProperties(t.Context(), density)
	require.NoError(t, err)
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
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	left, err := r3.Translation(r3.Vec{X: -10})
	require.NoError(t, err)
	right, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	velocity := func(x float64) dynamics.QuantityVec {
		return dynamics.QuantityVec{X: units.MillimetersPerSecond(x),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: left, LinearVelocity: velocity(50), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: right, LinearVelocity: velocity(-50), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 75*mass.Mass.Value.Base(), report.Events[0].NormalImpulse.Base(), 1e-4)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -25, finalA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 25, finalB.LinearVelocity.X.Base(), 1e-6)
}

func TestSphereReboundUsesProductionContactAndSweep(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	ball := makeBall(t, doc)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	contactRequest := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact:        contactRequest,
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 15})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Len(t, report.Events[0].Manifold.Points, 1)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-4)
	final, ok := report.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 10, final.Pose.Translation().Z, 2e-6)
	before, err := report.Trace.Sample(units.Milliseconds(50))
	require.NoError(t, err)
	beforeBall, ok := before.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 10, beforeBall.Pose.Translation().Z, 1e-9)
	require.Equal(t, units.MillimetersPerSecond(-100), beforeBall.LinearVelocity.Z)
	beforeContact, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), beforeBall.Pose,
		contactRequest)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, beforeContact.Relation)
	after, err := report.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	afterBall, ok := after.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 7.5, afterBall.Pose.Translation().Z, 2e-6)
	require.Equal(t, units.MillimetersPerSecond(50), afterBall.LinearVelocity.Z)
	afterContact, err := doc.ContactPair(t.Context(), floor, ball, r3.Identity(), afterBall.Pose,
		contactRequest)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, afterContact.Relation)
	endpointDuration := units.Seconds(.1000000002)
	endpoint, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, endpointDuration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, endpoint.Status, "%+v", endpoint.Diagnostics)
	require.Len(t, endpoint.Events, 1)
	require.Equal(t, units.Scalar(1), endpoint.Events[0].Bracket.To.Fraction)
	require.Equal(t, endpointDuration, endpoint.Events[0].Time)
	beforeEndpoint, err := endpoint.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	beforeEndpointBall, ok := beforeEndpoint.Body(ball)
	require.True(t, ok)
	require.Equal(t, units.MillimetersPerSecond(-100), beforeEndpointBall.LinearVelocity.Z)
	post, err := endpoint.Trace.Sample(endpoint.Events[0].Time)
	require.NoError(t, err)
	postBall, ok := post.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 50, postBall.LinearVelocity.Z.Base(), 1e-6)
}

func TestSourceSpherePairCenteredImpact(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	left, err := r3.Translation(r3.Vec{X: -10})
	require.NoError(t, err)
	right, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	velocity := func(x float64) dynamics.QuantityVec {
		return dynamics.QuantityVec{X: units.MillimetersPerSecond(x),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	}
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: left, LinearVelocity: velocity(50), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: right, LinearVelocity: velocity(-50), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 75, report.Events[0].NormalImpulse.Base(), 1e-5)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -25, finalA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 25, finalB.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, -7.5, finalA.Pose.Translation().X, 2e-6)
	require.InDelta(t, 7.5, finalB.Pose.Translation().X, 2e-6)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 0, report.Conservation.Input.LinearMomentum.Value.X.Base(), 1e-9)
	require.InDelta(t, 0, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-6)
	require.InDelta(t, 2500, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 625, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
	endpoint, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, endpoint.Status, "%+v", endpoint.Diagnostics)
	require.Len(t, endpoint.Events, 1)
	endpointA, ok := endpoint.Next.Body(a)
	require.True(t, ok)
	endpointB, ok := endpoint.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -5, endpointA.Pose.Translation().X, 2e-6)
	require.InDelta(t, 5, endpointB.Pose.Translation().X, 2e-6)
}
