package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func facetedFloorStepFixture(t *testing.T) (*decad.Document, *decad.Body, *decad.Body,
	decad.MassProperties) {
	t.Helper()
	doc := decad.New()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	cap := makeBox(t, doc, -2, -2, 2, 2, 8, 4)
	faceted, err := decad.Union(t.Context(), base, cap)
	require.NoError(t, err)
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	center, err := faceted.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0, center.Value.X, center.Bound.Base())
	require.InDelta(t, 0, center.Value.Y, center.Bound.Base())
	reading := func(value units.Value) decad.Measurement {
		return decad.Measurement{Value: value, Bound: units.New(0, value.Unit()), Exactness: decad.Exact}
	}
	inertia := reading(units.KilogramSquareMillimeters(10))
	offDiagonal := reading(units.KilogramSquareMillimeters(0))
	mass := decad.MassProperties{
		Mass: reading(units.Kilograms(1)), Center: center,
		Inertia: decad.InertiaReading{XX: inertia, YY: inertia, ZZ: inertia,
			XY: offDiagonal, XZ: offDiagonal, YZ: offDiagonal},
	}
	return doc, floor, faceted, mass
}

func facetedFloorStepConfig() dynamics.StepConfig {
	return dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution:          units.Seconds(1e-9),
		ContactSlop:             units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0),
		MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
	}
}

func facetedFloorWorld(t *testing.T, doc *decad.Document, floor, faceted *decad.Body,
	mass decad.MassProperties, restitution float64, reversed bool,
	startHeight, velocity float64) (*dynamics.World, dynamics.State) {
	t.Helper()
	material := dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(0)}
	definitions := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: faceted, Role: dynamics.Dynamic, Supplied: &mass, Material: material}}
	pose, err := r3.Translation(r3.Vec{Z: startHeight})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(velocity)
	states := []dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: faceted, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	}
	if reversed {
		definitions[0], definitions[1] = definitions[1], definitions[0]
		states[0], states[1] = states[1], states[0]
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: definitions, Step: facetedFloorStepConfig(),
	})
	require.NoError(t, err)
	start, err := world.NewState(states)
	require.NoError(t, err)
	return world, start
}

func TestFacetedFloorImpactUsesRealUnionSweepAndTrace(t *testing.T) {
	for _, reversed := range []bool{false, true} {
		for _, restitution := range []float64{0, .5} {
			t.Run(map[bool]string{false: "floor first", true: "faceted first"}[reversed]+
				map[bool]string{false: " resting", true: " rebounding"}[restitution > 0], func(t *testing.T) {
				doc, floor, faceted, mass := facetedFloorStepFixture(t)
				world, start := facetedFloorWorld(t, doc, floor, faceted, mass, restitution,
					reversed, 10, -160)
				request := facetedFloorStepConfig().Contact
				first := start.Entries()[0]
				second := start.Entries()[1]
				contact, err := doc.ContactPair(t.Context(), first.Body, second.Body,
					first.Pose, second.Pose, request)
				require.NoError(t, err)
				require.Equal(t, decad.ContactSeparated, contact.Relation)
				require.Equal(t, units.Millimeters(10), contact.Gap.Value)

				duration := units.Seconds(.125)
				still := decad.PairPath(decad.PoseSegment{From: r3.Identity(), To: r3.Identity(),
					Duration: duration})
				movingState, found := start.Body(faceted)
				require.True(t, found)
				moving := decad.PairPath(decad.RigidDriftSegment{From: movingState.Pose,
					Center: movingState.Pose.Apply(mass.Center.Value),
					LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(0),
						Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-160)},
					AngularVelocity: zeroAngular(t), Duration: duration})
				pathA, pathB := still, moving
				if reversed {
					pathA, pathB = pathB, pathA
				}
				sweep, err := doc.SweepPair(t.Context(), first.Body, second.Body, pathA, pathB,
					decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
						MaxPoseEvaluations: 128})
				require.NoError(t, err)
				require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
				require.True(t, sweep.HasAffineReplayProof())
				require.Equal(t, decad.ContactTouching, sweep.Event.Relation)
				require.Len(t, sweep.Event.Manifold.Points, 4)

				report, err := world.Step(t.Context(), start,
					dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
				require.NoError(t, err)
				require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
				require.Len(t, report.Events, 1)
				require.NotNil(t, report.Next)
				require.NotNil(t, report.Conservation)
				event := report.Events[0]
				require.Equal(t, dynamics.ContactImpact, event.Kind)
				require.InDelta(t, .0625, event.Time.Base(), 1e-12)
				require.Len(t, event.Manifold.Points, 4)
				for i, point := range event.Manifold.Points {
					require.Same(t, sweep.Event.Manifold.Points[i].FaceA, point.FaceA)
					require.Same(t, sweep.Event.Manifold.Points[i].FaceB, point.FaceB)
					require.Equal(t, sweep.Event.Manifold.Points[i].Normal.Value, point.Normal.Value)
				}
				require.InDelta(t, 160*(1+restitution), event.NormalImpulse.Base(), 1e-6)
				require.InDelta(t, -160, event.PreVelocity.Z.Base(), 1e-9)
				require.InDelta(t, 160*restitution, event.PostVelocity.Z.Base(), 1e-6)
				require.InDelta(t, 12800, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
				require.InDelta(t, 12800*restitution*restitution,
					report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
				require.InDelta(t, 160*(1+restitution),
					report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)

				for _, sample := range []struct {
					at, height, speed float64
					relation          decad.ContactRelation
				}{{0, 10, -160, decad.ContactSeparated},
					{.03125, 5, -160, decad.ContactSeparated},
					{.09375, 5 * restitution, 160 * restitution,
						map[bool]decad.ContactRelation{false: decad.ContactTouching,
							true: decad.ContactSeparated}[restitution > 0]},
					{.125, 10 * restitution, 160 * restitution,
						map[bool]decad.ContactRelation{false: decad.ContactTouching,
							true: decad.ContactSeparated}[restitution > 0]}} {
					replayed, sampleErr := report.Trace.Sample(units.Seconds(sample.at))
					require.NoError(t, sampleErr)
					body, found := replayed.Body(faceted)
					require.True(t, found)
					require.InDelta(t, sample.height, body.Pose.Translation().Z, 1e-6)
					require.InDelta(t, sample.speed, body.LinearVelocity.Z.Base(), 1e-6)
					floorState, found := replayed.Body(floor)
					require.True(t, found)
					at, sampleErr := doc.ContactPair(t.Context(), floor, faceted,
						floorState.Pose, body.Pose, request)
					require.NoError(t, sampleErr)
					require.Equal(t, sample.relation, at.Relation)
				}
				final, found := report.Next.Body(faceted)
				require.True(t, found)
				require.InDelta(t, 10*restitution, final.Pose.Translation().Z, 1e-6)
				require.Equal(t, []*decad.Body{faceted, floor}, doc.Bodies())
			})
		}
	}
}

func TestFacetedFloorImpactRefusesUncertifiedResponse(t *testing.T) {
	doc, floor, faceted, mass := facetedFloorStepFixture(t)
	for _, tc := range []struct {
		name, reason    string
		bound, duration float64
		velocity        float64
	}{
		{name: "mass center uncertainty", bound: .1, duration: .125, velocity: -160,
			reason: "off-center impulse"},
		{name: "unrepresented impact time", duration: .2, velocity: -100,
			reason: "first sweep returned"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			uncertain := mass
			if tc.bound > 0 {
				uncertain.Center.Bound = units.Millimeters(tc.bound)
				uncertain.Center.Exactness = decad.Approximate
			}
			world, start := facetedFloorWorld(t, doc, floor, faceted, uncertain,
				.5, false, 10, tc.velocity)
			report, err := world.Step(t.Context(), start,
				dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(tc.duration))
			require.NoError(t, err)
			require.Equal(t, dynamics.Undecided, report.Status)
			require.Nil(t, report.Next)
			require.Empty(t, report.Events)
			require.Len(t, report.Diagnostics, 1)
			require.Contains(t, report.Diagnostics[0].Reason, tc.reason)
		})
	}
}
