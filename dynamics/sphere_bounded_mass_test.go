package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestDiagonalSphereImpactWithDensityMass(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := a.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Positive(t, mass.Mass.Bound.Base())
	require.Positive(t, mass.Center.Bound.Base())
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	va := dynamics.QuantityVec{X: units.MillimetersPerSecond(30), Y: units.MillimetersPerSecond(40),
		Z: units.MillimetersPerSecond(0)}
	vb := dynamics.QuantityVec{X: units.MillimetersPerSecond(-30), Y: units.MillimetersPerSecond(-40),
		Z: units.MillimetersPerSecond(0)}
	zero := dynamics.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	sweep, err := doc.SweepPair(t.Context(), a, b,
		decad.RigidDriftSegment{From: pa, LinearVelocity: va, AngularVelocity: zero,
			Duration: units.Seconds(.2)},
		decad.RigidDriftSegment{From: pb, LinearVelocity: vb, AngularVelocity: zero,
			Duration: units.Seconds(.2)},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.Positive(t, sweep.Event.Manifold.Points[0].Normal.Bound.Base())
	mat := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	cfg := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: mat},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: mat},
		},
		Step: dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: zero},
		{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 75*mass.Mass.Value.Base(), report.Events[0].NormalImpulse.Base(), 1e-5)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, -15, finalA.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, -20, finalA.LinearVelocity.Y.Base(), 1e-6)
	require.InDelta(t, 15, finalB.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 20, finalB.LinearVelocity.Y.Base(), 1e-6)
	require.NotNil(t, report.Conservation)
	require.LessOrEqual(t, report.Conservation.Completion.LinearMomentum.Value.X.Base(),
		report.Conservation.Completion.LinearMomentum.Bound.X.Base())
	before, err := report.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	beforeA, ok := before.Body(a)
	require.True(t, ok)
	require.Equal(t, va, beforeA.LinearVelocity)
	after, err := report.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	afterA, ok := after.Body(a)
	require.True(t, ok)
	require.Equal(t, finalA.LinearVelocity, afterA.LinearVelocity)
	continued, err := w.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, continued.Status, "%+v", continued.Diagnostics)
	require.Empty(t, continued.Events)
	spinning := zero
	spinning.Z = units.RadiansPerSecond(1)
	spinStart, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: spinning},
		{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero},
	})
	require.NoError(t, err)
	spinReport, err := w.Step(t.Context(), spinStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, spinReport.Status)
	require.Nil(t, spinReport.Next)
	cfg.Bodies[0], cfg.Bodies[1] = cfg.Bodies[1], cfg.Bodies[0]
	reverseWorld, err := dynamics.NewWorld(t.Context(), doc, cfg)
	require.NoError(t, err)
	reverseStart, err := reverseWorld.NewState([]dynamics.BodyState{
		{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero},
		{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: zero},
	})
	require.NoError(t, err)
	reversed, err := reverseWorld.Step(t.Context(), reverseStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversed.Status, "%+v", reversed.Diagnostics)
	require.Len(t, reversed.Events, 1)
	require.InDelta(t, report.Events[0].NormalImpulse.Base(), reversed.Events[0].NormalImpulse.Base(), 1e-6)
	reverseA, ok := reversed.Next.Body(a)
	require.True(t, ok)
	require.InDelta(t, finalA.Pose.Translation().X, reverseA.Pose.Translation().X, 1e-8)
	for _, test := range []struct {
		name   string
		change func(*decad.MassProperties)
	}{
		{"wide mass", func(m *decad.MassProperties) { m.Mass.Bound = units.Kilograms(.1) }},
		{"wide center", func(m *decad.MassProperties) { m.Center.Bound = units.Millimeters(.01) }},
		{"off-center with large inertia", func(m *decad.MassProperties) {
			m.Center.Value.X = .25
			m.Inertia.XX.Value = units.KilogramSquareMillimeters(4e7)
			m.Inertia.YY.Value = units.KilogramSquareMillimeters(4e7)
			m.Inertia.ZZ.Value = units.KilogramSquareMillimeters(4e7)
		}},
	} {
		t.Run(test.name, func(t *testing.T) {
			uncertain := mass
			test.change(&uncertain)
			cfg.Bodies = []dynamics.RigidBody{
				{Body: a, Role: dynamics.Dynamic, Supplied: &uncertain, Material: mat},
				{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: mat},
			}
			unsupportedWorld, worldErr := dynamics.NewWorld(t.Context(), doc, cfg)
			require.NoError(t, worldErr)
			unsupportedStart, stateErr := unsupportedWorld.NewState([]dynamics.BodyState{
				{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: zero},
				{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero},
			})
			require.NoError(t, stateErr)
			unsupported, stepErr := unsupportedWorld.Step(t.Context(), unsupportedStart,
				dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Undecided, unsupported.Status)
			require.Nil(t, unsupported.Next)
		})
	}
	t.Run("combined omitted point speed", func(t *testing.T) {
		offset := mass
		offset.Center.Value.X = .25
		offset.Inertia.XX.Value = units.KilogramSquareMillimeters(8e7)
		offset.Inertia.YY.Value = units.KilogramSquareMillimeters(8e7)
		offset.Inertia.ZZ.Value = units.KilogramSquareMillimeters(8e7)
		cfg.Bodies = []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &offset, Material: mat},
			{Body: b, Role: dynamics.Dynamic, Supplied: &offset, Material: mat},
		}
		pairWorld, worldErr := dynamics.NewWorld(t.Context(), doc, cfg)
		require.NoError(t, worldErr)
		pairStart, stateErr := pairWorld.NewState([]dynamics.BodyState{
			{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: zero},
			{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: zero},
		})
		require.NoError(t, stateErr)
		pairReport, stepErr := pairWorld.Step(t.Context(), pairStart,
			dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
		require.NoError(t, stepErr)
		require.Equal(t, dynamics.Undecided, pairReport.Status, "%+v", pairReport.Diagnostics)
		require.Nil(t, pairReport.Next)
	})
}
