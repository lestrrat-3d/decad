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

func makeCylinder(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, 5)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cylinder, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return cylinder
}

func cylinderMass() decad.MassProperties {
	reading := func(value units.Value) decad.Measurement {
		return decad.Measurement{Value: value, Bound: units.New(0, value.Unit()), Exactness: decad.Exact}
	}
	inertia := reading(units.KilogramSquareMillimeters(10))
	zero := reading(units.KilogramSquareMillimeters(0))
	return decad.MassProperties{
		Mass: reading(units.Kilograms(1)),
		Center: decad.VecMeasurement{Value: r3.Vec{Z: 5}, Bound: units.Millimeters(0),
			Exactness: decad.Exact},
		Inertia: decad.InertiaReading{XX: inertia, YY: inertia, ZZ: inertia,
			XY: zero, XZ: zero, YZ: zero},
	}
}

func TestCylinderClearStepUsesProductionSweep(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			cylinder := makeCylinder(t, doc)
			pose, err := r3.Translation(r3.Vec{Z: 1})
			require.NoError(t, err)
			request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)}
			a, b := floor, cylinder
			pa, pb := r3.Identity(), pose
			if reverse {
				a, b, pa, pb = b, a, pb, pa
			}
			contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, request)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, contact.Relation)
			zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
				Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
			zeroLinear := zeroVelocity()
			up := decad.QuantityVec{X: units.MillimetersPerSecond(0),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(1)}
			floorPath := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroLinear,
				AngularVelocity: zeroAngular, Duration: units.Seconds(1)}
			cylinderPath := decad.RigidDriftSegment{From: pose, LinearVelocity: up,
				AngularVelocity: zeroAngular, Duration: units.Seconds(1)}
			pathA, pathB := decad.PairPath(floorPath), decad.PairPath(cylinderPath)
			if reverse {
				pathA, pathB = pathB, pathA
			}
			sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
				ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
			require.NoError(t, err)
			require.Equal(t, decad.SweepClear, sweep.Outcome)
			require.True(t, sweep.HasAffineReplayProof())
			replayA, replayB, err := sweep.CertifiedPosesAt(units.Seconds(.5))
			require.NoError(t, err)
			cylinderReplay := replayB
			if reverse {
				cylinderReplay = replayA
			}
			require.InDelta(t, 1.5, cylinderReplay.Translation().Z, 1e-9)
			material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
			density := units.KilogramsPerCubicMillimeter(0.001)
			mass, err := cylinder.MassProperties(t.Context(), density)
			require.NoError(t, err)
			require.InDelta(t, math.Pi*25*10*density.Base(), mass.Mass.Value.Base(), 1e-12)
			require.InDelta(t, 0.5*mass.Mass.Value.Base()*25, mass.Inertia.ZZ.Value.Base(), 1e-12)
			require.InDelta(t, mass.Mass.Value.Base()*175/12, mass.Inertia.XX.Value.Base(), 1e-12)
			require.InDelta(t, mass.Inertia.XX.Value.Base(), mass.Inertia.YY.Value.Base(), 1e-12)
			require.LessOrEqual(t, math.Abs(mass.Inertia.XY.Value.Base()), mass.Inertia.XY.Bound.Base())
			require.Less(t, mass.Mass.Bound.Base(), mass.Mass.Value.Base())
			require.InDelta(t, 5, mass.Center.Value.Z, 1e-12)
			defs := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
				{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: material}}
			states := []dynamics.BodyState{{Body: floor, Pose: r3.Identity(),
				LinearVelocity: zeroLinear, AngularVelocity: zeroAngular},
				{Body: cylinder, Pose: pose, LinearVelocity: up, AngularVelocity: zeroAngular}}
			if reverse {
				defs[0], defs[1] = defs[1], defs[0]
				states[0], states[1] = states[1], states[0]
			}
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: defs,
				Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
					ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
					AngularVelocityResidual: units.RadiansPerSecond(1e-6),
					ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
					PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
					MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
			require.NoError(t, err)
			start, err := world.NewState(states)
			require.NoError(t, err)
			step, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()},
				units.Seconds(1))
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Empty(t, step.Events)
			require.NotNil(t, step.Conservation)
			require.InDelta(t, mass.Mass.Value.Base(),
				step.Conservation.Input.LinearMomentum.Value.Z.Base(), 1e-12)
			require.InDelta(t, mass.Mass.Value.Base()/2,
				step.Conservation.Input.KineticEnergy.Value.Base(), 1e-12)
			final, ok := step.Next.Body(cylinder)
			require.True(t, ok)
			require.InDelta(t, 2, final.Pose.Translation().Z, 1e-9)
			mid, err := step.Trace.Sample(units.Seconds(.5))
			require.NoError(t, err)
			middle, ok := mid.Body(cylinder)
			require.True(t, ok)
			require.InDelta(t, 1.5, middle.Pose.Translation().Z, 1e-9)
		})
	}
}

func TestCylinderSweepRefusesUnprovedCorridors(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	zero := decad.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	for _, tc := range []struct {
		name     string
		x        float64
		start    float64
		velocity float64
		lateral  float64
	}{
		{name: "below point resolution", start: 1e-7, velocity: 1},
		{name: "lateral edge exit", start: 1, velocity: -2, lateral: 16},
		{name: "outside floor face", x: 30, start: 1, velocity: 1},
		{name: "disk reaches face edge", x: 15, start: 1, velocity: -2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pose, err := r3.Translation(r3.Vec{X: tc.x, Z: tc.start})
			require.NoError(t, err)
			if tc.x != 0 {
				contact, err := doc.ContactPair(t.Context(), floor, cylinder, r3.Identity(), pose, request)
				require.NoError(t, err)
				require.Equal(t, decad.ContactUndecided, contact.Relation)
			}
			velocity := decad.QuantityVec{X: units.MillimetersPerSecond(tc.lateral),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(tc.velocity)}
			moving := decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
				AngularVelocity: zero, Duration: units.Seconds(1)}
			for _, reverse := range []bool{false, true} {
				a, b := floor, cylinder
				pathA, pathB := decad.PairPath(stationary), decad.PairPath(moving)
				if reverse {
					a, b, pathA, pathB = b, a, pathB, pathA
				}
				sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
					ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
				require.NoError(t, err)
				require.Equal(t, decad.SweepUndecided, sweep.Outcome)
				require.False(t, sweep.HasAffineReplayProof())
			}
		})
	}
}

func TestCylinderSweepRefusesTiltAndSpin(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	request := decad.SweepRequest{ContactRequest: decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128}
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	start, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	turn, err := r3.RotationAround(r3.Vec{Z: 5}, r3.Vec{Y: 1}, units.Degrees(30))
	require.NoError(t, err)
	tilted, err := turn.Then(start)
	require.NoError(t, err)
	zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	down := decad.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-20)}
	for _, tc := range []struct {
		name    string
		pose    r3.Transform
		angular decad.QuantityVec
	}{
		{name: "tilted", pose: tilted, angular: zeroAngular},
		{name: "spinning", pose: start, angular: decad.QuantityVec{
			X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			moving := decad.RigidDriftSegment{From: tc.pose, LinearVelocity: down,
				AngularVelocity: tc.angular, Duration: units.Seconds(1)}
			for _, reverse := range []bool{false, true} {
				a, b := floor, cylinder
				pathA, pathB := decad.PairPath(stationary), decad.PairPath(moving)
				if reverse {
					a, b, pathA, pathB = b, a, pathB, pathA
				}
				sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, request)
				require.NoError(t, err)
				require.Equal(t, decad.SweepUndecided, sweep.Outcome)
				require.False(t, sweep.HasAffineReplayProof())
			}
		})
	}
}

func TestCylinderAxialFaceContact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	for _, tc := range []struct {
		name     string
		z        float64
		relation decad.ContactRelation
	}{
		{name: "touch", z: 0, relation: decad.ContactTouching},
		{name: "shallow overlap", z: -.25, relation: decad.ContactOverlapping},
		{name: "deep overlap", z: -10, relation: decad.ContactUndecided},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pose, err := r3.Translation(r3.Vec{Z: tc.z})
			require.NoError(t, err)
			for _, reverse := range []bool{false, true} {
				a, b := floor, cylinder
				pa, pb := r3.Identity(), pose
				if reverse {
					a, b, pa, pb = b, a, pb, pa
				}
				contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, request)
				require.NoError(t, err)
				require.Equal(t, tc.relation, contact.Relation)
				if tc.relation == decad.ContactUndecided {
					require.Nil(t, contact.Manifold)
					continue
				}
				require.NotNil(t, contact.Manifold)
				require.Len(t, contact.Manifold.Points, 1)
				point := contact.Manifold.Points[0]
				normal := r3.Vec{Z: 1}
				if reverse {
					normal.Z = -1
				}
				require.Equal(t, normal, point.Normal.Value)
				require.NotNil(t, point.FaceA)
				require.NotNil(t, point.FaceB)
				require.InDelta(t, tc.z, point.Separation.Value.Base(), 1e-12)
			}
		})
	}
}

func TestCylinderImpactPoseBudgetIsUndecided(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	pose, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	zero := decad.QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	down := decad.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-10)}
	fixed := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.2)}
	moving := decad.RigidDriftSegment{From: pose, LinearVelocity: down,
		AngularVelocity: zero, Duration: units.Seconds(.2)}
	request := decad.SweepRequest{ContactRequest: decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 2}
	for _, reverse := range []bool{false, true} {
		a, b := floor, cylinder
		pa, pb := decad.PairPath(fixed), decad.PairPath(moving)
		if reverse {
			a, b, pa, pb = b, a, pb, pa
		}
		sweep, err := doc.SweepPair(t.Context(), a, b, pa, pb, request)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, sweep.Outcome)
		require.Equal(t, decad.SweepPoseBudget, sweep.Cause)
		require.EqualValues(t, 2, sweep.PoseEvaluations)
		require.NotNil(t, sweep.Unresolved)
		require.Less(t, sweep.Unresolved.From.Elapsed.Value.Base(), .1)
		require.Greater(t, sweep.Unresolved.To.Elapsed.Value.Base(), .1)
		require.False(t, sweep.HasAffineReplayProof())
	}
}

func TestCylinderAxialFloorImpact(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			cylinderAxialFloorImpact(t, reverse, .2, 10)
		})
	}
}

func TestCylinderAxialEndpointImpact(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			cylinderAxialFloorImpact(t, reverse, .125, 8)
		})
	}
}

func cylinderAxialFloorImpact(t *testing.T, reverse bool, duration, speed float64) {
	t.Helper()
	impactAt := 1 / speed
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	pose, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	a, b := floor, cylinder
	poseA, poseB := r3.Identity(), pose
	if reverse {
		a, b, poseA, poseB = b, a, poseB, poseA
	}
	initial, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, initial.Relation)
	zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	down := decad.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-speed)}
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(duration)}
	moving := decad.RigidDriftSegment{From: pose, LinearVelocity: down,
		AngularVelocity: zeroAngular, Duration: units.Seconds(duration)}
	pathA, pathB := decad.PairPath(stationary), decad.PairPath(moving)
	if reverse {
		pathA, pathB = pathB, pathA
	}
	sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v samples=%+v", sweep.Cause, sweep.Samples)
	require.NotNil(t, sweep.Event.Manifold)
	require.Less(t, sweep.Bracket.From.Elapsed.Value.Base(), impactAt)
	if duration == impactAt {
		require.True(t, sweep.BracketEndsAtDuration())
		require.Equal(t, duration, sweep.Bracket.To.Elapsed.Value.Base())
	} else {
		require.Greater(t, sweep.Bracket.To.Elapsed.Value.Base(), impactAt)
	}
	require.LessOrEqual(t, sweep.Bracket.To.Elapsed.Value.Base()-sweep.Bracket.From.Elapsed.Value.Base(), 1e-9)
	normal := r3.Vec{Z: 1}
	if reverse {
		normal.Z = -1
	}
	require.Equal(t, normal, sweep.Event.Manifold.Points[0].Normal.Value)
	require.True(t, sweep.HasAffineReplayProof())
	replayA, replayB, err := sweep.CertifiedPosesAt(units.Seconds(impactAt / 2))
	require.NoError(t, err)
	replayCylinder := replayB
	if reverse {
		replayCylinder = replayA
	}
	require.InDelta(t, .5, replayCylinder.Translation().Z, 1e-9)
	if duration > impactAt {
		_, _, err = sweep.CertifiedPosesAt(units.Seconds((impactAt + duration) / 2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
	mass := cylinderMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	defs := []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: cylinder, Role: dynamics.Dynamic, Supplied: &mass, Material: material}}
	states := []dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular},
		{Body: cylinder, Pose: pose, LinearVelocity: down, AngularVelocity: zeroAngular}}
	if reverse {
		defs[0], defs[1] = defs[1], defs[0]
		states[0], states[1] = states[1], states[0]
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: defs,
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
	require.NoError(t, err)
	start, err := world.NewState(states)
	require.NoError(t, err)
	step, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(duration))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, 1.5*speed, step.Events[0].NormalImpulse.Base(), 1e-6)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, .5*speed*speed, step.Conservation.Input.KineticEnergy.Value.Base(), 1e-8)
	require.InDelta(t, .125*speed*speed, step.Conservation.Completion.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, -speed, step.Conservation.Input.LinearMomentum.Value.Z.Base(), 1e-8)
	require.InDelta(t, .5*speed, step.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	require.InDelta(t, 1.5*speed, step.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	final, ok := step.Next.Body(cylinder)
	require.True(t, ok)
	finalZ := .5 * speed * (duration - impactAt)
	require.InDelta(t, finalZ, final.Pose.Translation().Z, 1e-6)
	require.InDelta(t, .5*speed, final.LinearVelocity.Z.Base(), 1e-6)
	samples := []struct{ at, z float64 }{{impactAt / 2, .5}}
	if duration > impactAt {
		samples = append(samples, struct{ at, z float64 }{
			(impactAt + duration) / 2, finalZ / 2})
	}
	samples = append(samples, struct{ at, z float64 }{duration, finalZ})
	for _, sample := range samples {
		state, err := step.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, err)
		entry, ok := state.Body(cylinder)
		require.True(t, ok)
		require.InDelta(t, sample.z, entry.Pose.Translation().Z, 1e-6)
	}
}

func TestCylinderSlowOffCenterSpinUndecided(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	pose, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	mass := cylinderMass()
	mass.Center.Value.X = 5
	mass.Inertia.XX.Value = units.KilogramSquareMillimeters(1e8)
	mass.Inertia.YY.Value = units.KilogramSquareMillimeters(1e8)
	mass.Inertia.ZZ.Value = units.KilogramSquareMillimeters(1e8)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	down := decad.QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-10)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: cylinder, Role: dynamics.Dynamic, Supplied: &mass, Material: material}},
		Step: dynamics.StepConfig{Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
	require.NoError(t, err)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular},
		{Body: cylinder, Pose: pose, LinearVelocity: down, AngularVelocity: zeroAngular}})
	require.NoError(t, err)
	step, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, step.Status)
	require.Nil(t, step.Next)
	require.Contains(t, step.Diagnostics[0].Reason, "omitted cylinder point motion")
}
