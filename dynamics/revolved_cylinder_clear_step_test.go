package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func makeRevolvedCylinder(t *testing.T, doc *decad.Document, extent decad.AngularExtent,
	innerRadius float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, innerRadius, 10, 5)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}, extent)
	require.NoError(t, err)
	return body
}

func makeTorus(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, 3)
	s.CreateCircle(center, 1)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestRevolvedCylinderClearStepUsesProductionSweep(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			runRevolvedCylinderClearStep(t, reverse)
		})
	}
}

func runRevolvedCylinderClearStep(t *testing.T, reverse bool) {
	t.Helper()
	doc := decad.New()
	floor := makeBox(t, doc, -10, -20, 0, 20, -20, 40)
	cylinder := makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 0)
	pose, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	a, b := floor, cylinder
	pa, pb := r3.Identity(), pose
	if reverse {
		a, b, pa, pb = b, a, pb, pa
	}
	contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.InDelta(t, 1, contact.Gap.Value.Base(), 1e-12)
	zero := zeroAngular(t)
	up := dynamics.QuantityVec{X: units.MillimetersPerSecond(1),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	still := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroVelocity(),
		AngularVelocity: zero, Duration: units.Seconds(1)}
	moving := decad.RigidDriftSegment{From: pose, LinearVelocity: up,
		AngularVelocity: zero, Duration: units.Seconds(1)}
	pathA, pathB := decad.PairPath(still), decad.PairPath(moving)
	if reverse {
		pathA, pathB = pathB, pathA
	}
	sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB,
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, sweep.Outcome)
	require.True(t, sweep.HasAffineReplayProof())
	replayA, replayB, err := sweep.CertifiedPosesAt(units.Seconds(.5))
	require.NoError(t, err)
	replay := replayB
	if reverse {
		replay = replayA
	}
	require.InDelta(t, 1.5, replay.Translation().X, 1e-9)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := cylinder.MassProperties(t.Context(), density)
	require.NoError(t, err)
	mat := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	defs := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: mat},
		{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: mat}}
	states := []dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: cylinder, Pose: pose, LinearVelocity: up, AngularVelocity: zero}}
	if reverse {
		defs[0], defs[1] = defs[1], defs[0]
		states[0], states[1] = states[1], states[0]
	}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: defs,
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
	require.NoError(t, err)
	state, err := w.NewState(states)
	require.NoError(t, err)
	step, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Empty(t, step.Events)
	require.NotNil(t, step.Conservation)
	require.InDelta(t, mass.Mass.Value.Base(),
		step.Conservation.Input.LinearMomentum.Value.X.Base(), 1e-12)
	require.InDelta(t, mass.Mass.Value.Base()/2,
		step.Conservation.Input.KineticEnergy.Value.Base(), 1e-12)
	require.InDelta(t, mass.Mass.Value.Base(),
		step.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-12)
	require.InDelta(t, mass.Mass.Value.Base()/2,
		step.Conservation.Completion.KineticEnergy.Value.Base(), 1e-12)
	require.Zero(t, step.Conservation.ContactImpulse.Value.X.Base())
	final, ok := step.Next.Body(cylinder)
	require.True(t, ok)
	require.InDelta(t, 2, final.Pose.Translation().X, 1e-9)
	mid, err := step.Trace.Sample(units.Seconds(.5))
	require.NoError(t, err)
	middle, ok := mid.Body(cylinder)
	require.True(t, ok)
	require.InDelta(t, 1.5, middle.Pose.Translation().X, 1e-9)
}

func TestRevolvedCylinderAxialFloorImpact(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			revolvedCylinderAxialFloorImpact(t, reverse, .2, 10)
		})
	}
}

func TestRevolvedCylinderAxialEndpointImpact(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			revolvedCylinderAxialFloorImpact(t, reverse, .125, 8)
		})
	}
}

func revolvedCylinderAxialFloorImpact(t *testing.T, reverse bool, duration, speed float64) {
	t.Helper()
	impactAt := 1 / speed
	doc := decad.New()
	floor := makeBox(t, doc, -10, -20, 0, 20, -20, 40)
	cylinder := makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 0)
	pose, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := cylinder.MassProperties(t.Context(), density)
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
	zero := zeroAngular(t)
	incoming := decad.QuantityVec{X: units.MillimetersPerSecond(-speed),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(duration)}
	moving := decad.RigidDriftSegment{From: pose, LinearVelocity: incoming,
		AngularVelocity: zero, Duration: units.Seconds(duration)}
	pathA, pathB := decad.PairPath(stationary), decad.PairPath(moving)
	if reverse {
		pathA, pathB = pathB, pathA
	}
	sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.NotNil(t, sweep.Event.Manifold)
	require.Less(t, sweep.Bracket.From.Elapsed.Value.Base(), impactAt)
	require.GreaterOrEqual(t, sweep.Bracket.To.Elapsed.Value.Base(), impactAt)
	if duration == impactAt {
		require.True(t, sweep.BracketEndsAtDuration())
	}
	require.LessOrEqual(t,
		sweep.Bracket.To.Elapsed.Value.Base()-sweep.Bracket.From.Elapsed.Value.Base(), 1e-9)
	point := sweep.Event.Manifold.Points[0]
	wantNormal := r3.Vec{X: 1}
	if reverse {
		wantNormal.X = -1
	}
	require.Equal(t, wantNormal, point.Normal.Value)
	cylinderFace := point.FaceB
	if reverse {
		cylinderFace = point.FaceA
	}
	require.Contains(t, cylinder.Faces(), cylinderFace)
	require.True(t, sweep.HasAffineReplayProof())
	replayA, replayB, err := sweep.CertifiedPosesAt(units.Seconds(impactAt / 2))
	require.NoError(t, err)
	replay := replayB
	if reverse {
		replay = replayA
	}
	require.InDelta(t, .5, replay.Translation().X, 1e-9)
	if duration > impactAt {
		_, _, err = sweep.CertifiedPosesAt(units.Seconds((impactAt + duration) / 2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
	mat := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	defs := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: mat},
		{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: mat}}
	states := []dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: cylinder, Pose: pose, LinearVelocity: incoming, AngularVelocity: zero}}
	if reverse {
		defs[0], defs[1] = defs[1], defs[0]
		states[0], states[1] = states[1], states[0]
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: defs,
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-5),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
	require.NoError(t, err)
	start, err := world.NewState(states)
	require.NoError(t, err)
	step, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(duration))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, mass.Mass.Value.Base()*1.5*speed, step.Events[0].NormalImpulse.Base(), 1e-4)
	final, ok := step.Next.Body(cylinder)
	require.True(t, ok)
	finalX := .5 * speed * (duration - impactAt)
	require.InDelta(t, finalX, final.Pose.Translation().X, 1e-6)
	require.InDelta(t, .5*speed, final.LinearVelocity.X.Base(), 1e-6)
	samples := []struct{ at, x float64 }{{impactAt / 2, .5}}
	if duration > impactAt {
		samples = append(samples, struct{ at, x float64 }{
			(impactAt + duration) / 2, finalX / 2})
	}
	samples = append(samples, struct{ at, x float64 }{duration, finalX})
	for _, tc := range samples {
		sample, err := step.Trace.Sample(units.Seconds(tc.at))
		require.NoError(t, err)
		entry, ok := sample.Body(cylinder)
		require.True(t, ok)
		require.InDelta(t, tc.x, entry.Pose.Translation().X, 1e-6)
	}
}

func TestRevolvedCylinderClearSweepReverseAndRefusals(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -10, -20, 0, 20, -20, 40)
	full := makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 0)
	partial := makeRevolvedCylinder(t, doc,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}, 0)
	annular := makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 2)
	torus := makeTorus(t, doc)
	placement, err := r3.Translation(r3.Vec{Y: 3})
	require.NoError(t, err)
	placed, err := full.PlacedCopy(t.Context(), placement)
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	for _, tc := range []struct {
		name            string
		body            *decad.Body
		y               float64
		start           float64
		velocity        float64
		lateralVelocity float64
		relation        decad.ContactRelation
		outcome         decad.SweepOutcome
	}{
		{name: "clear full turn", body: full, start: 1, velocity: 1,
			relation: decad.ContactSeparated, outcome: decad.SweepClear},
		{name: "clear placed full turn", body: placed, start: 1, velocity: 1,
			relation: decad.ContactSeparated, outcome: decad.SweepClear},
		{name: "near contact", body: full, start: 1e-7, velocity: 1,
			relation: decad.ContactSeparated, outcome: decad.SweepUndecided},
		{name: "crossing", body: full, start: 1, velocity: -2,
			relation: decad.ContactSeparated, outcome: decad.SweepImpactBracket},
		{name: "lateral crossing", body: full, start: 1, velocity: -2, lateralVelocity: .25,
			relation: decad.ContactSeparated, outcome: decad.SweepUndecided},
		{name: "floor edge", body: full, y: 15, start: 1, velocity: -2,
			relation: decad.ContactUndecided, outcome: decad.SweepUndecided},
		{name: "lateral outside floor face", body: full, y: 30, start: 1, velocity: 1,
			relation: decad.ContactUndecided, outcome: decad.SweepUndecided},
		{name: "partial turn", body: partial, start: 1, velocity: 1,
			relation: decad.ContactUndecided, outcome: decad.SweepUndecided},
		{name: "annular turn", body: annular, start: 1, velocity: 1,
			relation: decad.ContactUndecided, outcome: decad.SweepUndecided},
		{name: "torus", body: torus, start: 1, velocity: 1,
			relation: decad.ContactUndecided, outcome: decad.SweepUndecided},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pose, err := r3.Translation(r3.Vec{X: tc.start, Y: tc.y})
			require.NoError(t, err)
			velocity := decad.QuantityVec{X: units.MillimetersPerSecond(tc.velocity),
				Y: units.MillimetersPerSecond(tc.lateralVelocity), Z: units.MillimetersPerSecond(0)}
			moving := decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
				AngularVelocity: zeroAngular(t), Duration: units.Seconds(1)}
			for _, reverse := range []bool{false, true} {
				a, b := floor, tc.body
				pa, pb := r3.Identity(), pose
				pathA, pathB := decad.PairPath(still), decad.PairPath(moving)
				if reverse {
					a, b, pa, pb = b, a, pb, pa
					pathA, pathB = pathB, pathA
				}
				contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, req)
				require.NoError(t, err)
				require.Equal(t, tc.relation, contact.Relation)
				sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB,
					decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
						MaxPoseEvaluations: 128})
				require.NoError(t, err)
				require.Equal(t, tc.outcome, sweep.Outcome)
				require.Equal(t, tc.outcome == decad.SweepClear || tc.outcome == decad.SweepImpactBracket,
					sweep.HasAffineReplayProof())
			}
		})
	}
}
