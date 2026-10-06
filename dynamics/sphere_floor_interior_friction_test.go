package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSphereFloorInteriorFrictionUsesRealBracketAndTrace(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name            string
		mu              float64
		reverse         bool
		wantTangent     float64
		wantX, wantSpin float64
		boundedMass     bool
		restitution     float64
		maxEvents       int
		refuse          bool
	}{
		{name: "stick", mu: .5, wantTangent: -100.0 / 7, wantX: 250.0 / 7, wantSpin: 50.0 / 7},
		{name: "slip", mu: .1, wantTangent: -10, wantX: 40, wantSpin: 5},
		{name: "reverse stick", mu: .5, reverse: true, wantTangent: 100.0 / 7,
			wantX: 250.0 / 7, wantSpin: 50.0 / 7},
		{name: "reverse slip", mu: .1, reverse: true, wantTangent: 10, wantX: 40, wantSpin: 5},
		{name: "bounded mass", mu: .5, boundedMass: true, refuse: true},
		{name: "positive restitution", mu: .5, restitution: .5, wantTangent: -100.0 / 7, wantX: 250.0 / 7,
			wantSpin: 50.0 / 7},
		{name: "insufficient events", mu: .5, maxEvents: 1, refuse: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			ball := makeBall(t, doc)
			startPose, err := r3.Translation(r3.Vec{Z: 15})
			require.NoError(t, err)
			endPose, err := r3.Translation(r3.Vec{X: 7.5})
			require.NoError(t, err)
			req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)}
			contact, err := doc.ContactPair(t.Context(), floor, ball,
				r3.Identity(), startPose, req)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, contact.Relation)
			fixedPath := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.15)}
			movingPath := decad.PoseSegment{From: startPose, To: endPose, Duration: units.Seconds(.15)}
			sweep, err := doc.SweepPair(t.Context(), floor, ball, fixedPath, movingPath,
				decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
					MaxPoseEvaluations: 128})
			require.NoError(t, err)
			require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
			require.Len(t, sweep.Event.Manifold.Points, 1)
			material := dynamics.Material{Restitution: units.Scalar(tc.restitution),
				Friction: units.Scalar(tc.mu)}
			mass := exactSphereMass()
			if tc.boundedMass {
				mass.Mass.Exactness = decad.Approximate
				mass.Mass.Bound = units.Kilograms(.01)
			}
			maxEvents := 2
			if tc.maxEvents != 0 {
				maxEvents = tc.maxEvents
			}
			bodies := []dynamics.RigidBody{
				{Body: floor, Role: dynamics.Fixed, Material: material},
				{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			}
			states := []dynamics.BodyState{
				{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
				{Body: ball, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
					X: units.MillimetersPerSecond(50), Y: units.MillimetersPerSecond(0),
					Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
			}
			if tc.reverse {
				bodies[0], bodies[1] = bodies[1], bodies[0]
				states[0], states[1] = states[1], states[0]
			}
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: bodies, Step: dynamics.StepConfig{Contact: req,
					TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
					VelocityResidual:        units.MillimetersPerSecond(1e-6),
					AngularVelocityResidual: units.RadiansPerSecond(1e-6),
					ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
					PenetrationResidual:     units.Millimeters(1e-6),
					ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
					MaxIterations: 64, MaxEvents: maxEvents, MaxPairSweeps: 4096},
			})
			require.NoError(t, err)
			start, err := world.NewState(states)
			require.NoError(t, err)
			report, err := world.Step(t.Context(), start,
				dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.15))
			require.NoError(t, err)
			if tc.refuse {
				require.Equal(t, dynamics.Undecided, report.Status)
				require.Nil(t, report.Next)
				if tc.maxEvents == 0 {
					require.Empty(t, report.Events)
					return
				}
				// The impact at 0.1 s reaches MaxEvents with time remaining: the
				// certified prefix keeps its event and stops there (§12).
				require.Len(t, report.Events, 1)
				require.InDelta(t, .1, report.Events[0].Time.Base(), 1e-8)
				require.InDelta(t, 100, report.Events[0].NormalImpulse.Base(), 1e-6)
				require.Len(t, report.Diagnostics, 1)
				require.Equal(t, dynamics.StepEventBudget, report.Diagnostics[0].Code)
				require.InDelta(t, .1, report.Diagnostics[0].From.Base(), 1e-8)
				return
			}
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 1)
			event := report.Events[0]
			require.InDelta(t, .1, event.Time.Base(), 1e-8)
			if tc.restitution > 0 {
				// The ball bounces at 0.5·100 mm/s, 150 kg·mm/s, and friction
				// −100/7 kg·mm/s inside the 75 kg·mm/s cone sets it rolling.
				require.InDelta(t, 150, event.NormalImpulse.Base(), 1e-6)
				require.InDelta(t, tc.wantTangent, event.TangentImpulse.X.Base(), 1e-6)
				bounced, ok := report.Next.Body(ball)
				require.True(t, ok)
				require.InDelta(t, tc.wantX, bounced.LinearVelocity.X.Base(), 1e-6)
				require.InDelta(t, 50, bounced.LinearVelocity.Z.Base(), 1e-6)
				require.InDelta(t, tc.wantSpin, bounced.AngularVelocity.Y.Base(), 1e-6)
				return
			}
			require.Len(t, event.Manifold.Points, 1)
			require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-6)
			require.InDelta(t, tc.wantTangent, event.TangentImpulse.X.Base(), 1e-6)
			require.LessOrEqual(t, math.Abs(event.TangentImpulse.X.Base()),
				tc.mu*event.NormalImpulse.Base()+1e-6)
			require.NotNil(t, event.Solver)
			require.LessOrEqual(t, event.Solver.PenetrationResidual.Base(), 1e-6)
			require.NotNil(t, report.Conservation)
			require.InDelta(t, -math.Abs(tc.wantTangent),
				report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
			require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
			require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
				report.Conservation.AfterKick.KineticEnergy.Value.Base())
			final, ok := report.Next.Body(ball)
			require.True(t, ok)
			require.InDelta(t, tc.wantX, final.LinearVelocity.X.Base(), 1e-6)
			require.InDelta(t, tc.wantSpin, final.AngularVelocity.Y.Base(), 1e-6)
			require.InDelta(t, 0, final.LinearVelocity.Z.Base(), 1e-6)
			require.InDelta(t, 5+tc.wantX*.05, final.Pose.Translation().X, 1e-6)
			require.InDelta(t, 5, final.Pose.Translation().Z, 1e-6)
			for _, at := range []units.Value{units.Seconds(.05), event.Time,
				units.Seconds(.125), units.Seconds(.15)} {
				sample, sampleErr := report.Trace.Sample(at)
				require.NoError(t, sampleErr)
				ballState, exists := sample.Body(ball)
				require.True(t, exists)
				relation, contactErr := doc.ContactPair(t.Context(), floor, ball,
					r3.Identity(), ballState.Pose, req)
				require.NoError(t, contactErr)
				if at.Base() < event.Time.Base() {
					require.Equal(t, decad.ContactSeparated, relation.Relation)
				} else {
					require.Equal(t, decad.ContactTouching, relation.Relation)
				}
			}
			require.Len(t, doc.Bodies(), 2)
		})
	}
}
