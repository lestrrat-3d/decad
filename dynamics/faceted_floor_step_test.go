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

func positiveBoundFacetedFloorFixture(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	upper, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(4), Dir: decad.Along,
	})
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{Z: 8})
	require.NoError(t, err)
	upper, err = upper.Placed(t.Context(), shift)
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), base, upper)
	require.NoError(t, err)
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	return doc, floor, union
}

func positiveBoundFacetedStepConfig() dynamics.StepConfig {
	config := facetedFloorStepConfig()
	config.ImpulseResidual = units.KilogramMillimetersPerSecond(.01)
	config.AngularVelocityResidual = units.RadiansPerSecond(.03)
	return config
}

func TestPositiveBoundFacetedFloorDensityImpactAndTrace(t *testing.T) {
	t.Parallel()
	doc, floor, body := positiveBoundFacetedFloorFixture(t)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	require.Positive(t, mass.Center.Bound.Base())
	request := facetedFloorStepConfig().Contact
	contact, err := doc.ContactPair(t.Context(), floor, body, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: body, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: positiveBoundFacetedStepConfig()})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-160)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: body, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(.125)
	still := decad.PairPath(decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration})
	moving := decad.PairPath(decad.RigidDriftSegment{From: pose, Center: pose.Apply(mass.Center.Value),
		LinearVelocity: fall, AngularVelocity: zeroAngular(t), Duration: duration})
	sweep, err := doc.SweepPair(t.Context(), floor, body, still, moving, decad.SweepRequest{
		ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, .0625, report.Events[0].Time.Base(), 1e-12)
	require.InDelta(t, 80, report.Events[0].PostVelocity.Z.Base(), 1e-6)
	for _, sample := range []struct{ time, z, speed float64 }{
		{0, 10, -160}, {.03125, 5, -160}, {.0625, 0, 80}, {.09375, 2.5, 80}, {.125, 5, 80},
	} {
		replayed, sampleErr := report.Trace.Sample(units.Seconds(sample.time))
		require.NoError(t, sampleErr)
		state, ok := replayed.Body(body)
		require.True(t, ok)
		require.InDelta(t, sample.z, state.Pose.Translation().Z, 1e-6)
		require.InDelta(t, sample.speed, state.LinearVelocity.Z.Base(), 1e-6)
	}
}

func TestPositiveBoundFacetedFloorDensityRestAndTrace(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		density float64
	}{{"original density", .001}, {"alternate density", .0015612}} {
		t.Run(tc.name, func(t *testing.T) {
			positiveBoundFacetedFloorDensityRestAndTrace(t, tc.density)
		})
	}
}

func positiveBoundFacetedFloorDensityRestAndTrace(t *testing.T, densityValue float64) {
	t.Helper()
	doc, floor, body := positiveBoundFacetedFloorFixture(t)
	density := units.KilogramsPerCubicMillimeter(densityValue)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: body, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: positiveBoundFacetedStepConfig()})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-160)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: body, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.125))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, .0625, report.Events[0].Time.Base(), 1e-12)
	require.Zero(t, report.Events[0].PostVelocity.Z.Base())
	for _, time := range []float64{.0625, .09375, .125} {
		replayed, sampleErr := report.Trace.Sample(units.Seconds(time))
		require.NoError(t, sampleErr)
		state, ok := replayed.Body(body)
		require.True(t, ok)
		require.InDelta(t, 0, state.Pose.Translation().Z, 1e-6)
		require.Zero(t, state.LinearVelocity.Z.Base())
		contact, contactErr := doc.ContactPair(t.Context(), floor, body,
			r3.Identity(), state.Pose, positiveBoundFacetedStepConfig().Contact)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, contact.Relation)
	}
}

func facetedFloorStepFixture(t *testing.T) (*decad.Document, *decad.Body, *decad.Body,
	decad.MassProperties) {
	t.Helper()
	doc := decad.New()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	// Placed up to z = 8 so the analytic union's shared-axis gate declines
	// the pair and the union stays faceted.
	upperCap, err := makeBox(t, doc, -2, -2, 2, 2, 0, 4).Placed(t.Context(), translation(t, r3.Vec{Z: 8}))
	require.NoError(t, err)
	faceted, err := decad.Union(t.Context(), base, upperCap)
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
		MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
	}
}

func TestBoundedFacetedMassAdmitsDynamicWorld(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	// Placed up to z = 8 so the union stays faceted (see facetedFloorStepFixture).
	upper, err := makeBox(t, doc, -2, -2, 2, 2, 0, 4).Placed(t.Context(), translation(t, r3.Vec{Z: 8}))
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), base, upper)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	placed, err := union.Placed(t.Context(), shift)
	require.NoError(t, err)
	floor := makeBox(t, doc, -20, -20, 20, 20, -100, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: placed, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: facetedFloorStepConfig(),
	})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: placed, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	require.Len(t, state.Entries(), 2)
}

func TestBoundedFacetedMassClearStepAndTrace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	// Placed up to z = 8 so the union stays faceted (see facetedFloorStepFixture).
	upper, err := makeBox(t, doc, -2, -2, 2, 2, 0, 4).Placed(t.Context(), translation(t, r3.Vec{Z: 8}))
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), base, upper)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	placed, err := union.Placed(t.Context(), shift)
	require.NoError(t, err)
	floor := makeBox(t, doc, -20, -20, 20, 20, -100, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: placed, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: facetedFloorStepConfig(),
	})
	require.NoError(t, err)
	velocity := zeroVelocity()
	velocity.Z = units.MillimetersPerSecond(-1)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: placed, Pose: r3.Identity(), LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, placed, r3.Identity(), r3.Identity(),
		facetedFloorStepConfig().Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Next)
	for _, sample := range []struct{ time, z float64 }{{0, 0}, {.05, -.05}, {.1, -.1}} {
		state, sampleErr := report.Trace.Sample(units.Seconds(sample.time))
		require.NoError(t, sampleErr)
		body, ok := state.Body(placed)
		require.True(t, ok)
		require.InDelta(t, sample.z, body.Pose.Translation().Z, 1e-10)
		require.Equal(t, velocity, body.LinearVelocity)
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
	t.Parallel()
	for _, reversed := range []bool{false, true} {
		for _, restitution := range []float64{0, .5} {
			t.Run(map[bool]string{false: "fixed floor before union", true: "union before fixed floor"}[reversed]+
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

func TestFacetedFloorImpactUsesDensityMass(t *testing.T) {
	t.Parallel()
	doc, floor, faceted, _ := facetedFloorStepFixture(t)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := faceted.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1.032, mass.Mass.Value.Base(), mass.Mass.Bound.Base()+1e-15)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: faceted, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: facetedFloorStepConfig(),
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-160)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: faceted, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.125))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, .0625, report.Events[0].Time.Base(), 1e-12)
	require.InDelta(t, 80, report.Events[0].PostVelocity.Z.Base(), 1e-6)
}

func TestPlacedFacetedFloorDensityImpactAndTrace(t *testing.T) {
	t.Parallel()
	doc, floor, source, _ := facetedFloorStepFixture(t)
	shift, err := r3.Translation(r3.Vec{X: .1})
	require.NoError(t, err)
	faceted, err := source.Placed(t.Context(), shift)
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := faceted.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	require.Positive(t, mass.Center.Bound.Base())
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: faceted, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: facetedFloorStepConfig(),
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	velocity := zeroVelocity()
	velocity.Z = units.MillimetersPerSecond(-160)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: faceted, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	request := facetedFloorStepConfig().Contact
	initial, err := doc.ContactPair(t.Context(), floor, faceted, r3.Identity(), pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, initial.Relation)
	atFloor, err := doc.ContactPair(t.Context(), floor, faceted, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, atFloor.Relation)
	require.Len(t, atFloor.Manifold.Points, 4)
	duration := units.Seconds(.125)
	still := decad.PairPath(decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration})
	moving := decad.PairPath(decad.RigidDriftSegment{From: pose,
		Center: pose.Apply(mass.Center.Value), LinearVelocity: velocity,
		AngularVelocity: zeroAngular(t), Duration: duration})
	sweep, err := doc.SweepPair(t.Context(), floor, faceted, still, moving,
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.Equal(t, decad.ContactTouching, sweep.Event.Relation)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, .0625, report.Events[0].Time.Base(), 1e-12)
	require.InDelta(t, 80, report.Events[0].PostVelocity.Z.Base(), 1e-6)
	require.Len(t, report.Events[0].Manifold.Points, 4)
	for _, sample := range []struct {
		at, z, speed float64
		relation     decad.ContactRelation
	}{
		{0, 10, -160, decad.ContactSeparated},
		{.03125, 5, -160, decad.ContactSeparated},
		{.0625, 0, 80, decad.ContactTouching},
		{.09375, 2.5, 80, decad.ContactSeparated},
		{.125, 5, 80, decad.ContactSeparated},
	} {
		replayed, sampleErr := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, sampleErr)
		body, ok := replayed.Body(faceted)
		require.True(t, ok)
		require.InDelta(t, sample.z, body.Pose.Translation().Z, 1e-6)
		require.InDelta(t, sample.speed, body.LinearVelocity.Z.Base(), 1e-6)
		floorState, ok := replayed.Body(floor)
		require.True(t, ok)
		contact, contactErr := doc.ContactPair(t.Context(), floor, faceted,
			floorState.Pose, body.Pose, request)
		require.NoError(t, contactErr)
		require.Equal(t, sample.relation, contact.Relation)
	}
}

func TestFacetedFloorImpactRefusesUncertifiedResponse(t *testing.T) {
	t.Parallel()
	doc, floor, faceted, mass := facetedFloorStepFixture(t)
	for _, tc := range []struct {
		name, reason    string
		bound, duration float64
		velocity, from  float64
		code            dynamics.StepReason
	}{
		{name: "mass center uncertainty", bound: .1, duration: .125, velocity: -160,
			code: dynamics.StepIslandResidual, reason: "angular law", from: .0625},
		{name: "unrepresented impact time", duration: .2, velocity: -100,
			code: dynamics.StepPairUndecided, from: 0},
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
			require.Equal(t, tc.code, report.Diagnostics[0].Code)
			require.Contains(t, report.Diagnostics[0].Reason, tc.reason)
			require.InDelta(t, tc.from, report.Diagnostics[0].From.Base(), 1e-8)
		})
	}
}
