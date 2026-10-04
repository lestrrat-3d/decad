package dynamics_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSuppliedMassAdmission(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	source, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Supplied: &source, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
		},
	}
	_, err = dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = dynamics.NewWorld(ctx, doc, config)
	require.ErrorIs(t, err, context.Canceled)

	for _, tc := range []struct {
		name   string
		change func(*decad.MassProperties)
	}{
		{"wrong mass kind", func(m *decad.MassProperties) { m.Mass.Value = units.Millimeters(1) }},
		{"wrong center kind", func(m *decad.MassProperties) { m.Center.Bound = units.Kilograms(0) }},
		{"wrong inertia kind", func(m *decad.MassProperties) { m.Inertia.XX.Value = units.Kilograms(1) }},
		{"nonfinite mass", func(m *decad.MassProperties) { m.Mass.Value = units.Kilograms(math.Inf(1)) }},
		{"nonfinite center", func(m *decad.MassProperties) { m.Center.Value = r3.Vec{X: math.NaN()} }},
		{"nonfinite inertia", func(m *decad.MassProperties) {
			m.Inertia.XX.Value = units.KilogramSquareMillimeters(math.Inf(1))
		}},
		{"unbounded mass", func(m *decad.MassProperties) { m.Mass.Bound = units.Kilograms(1) }},
		{"negative center bound", func(m *decad.MassProperties) { m.Center.Bound = units.Millimeters(-1) }},
		{"negative inertia bound", func(m *decad.MassProperties) {
			m.Inertia.XX.Bound = units.KilogramSquareMillimeters(-1)
		}},
		{"unproved tensor", func(m *decad.MassProperties) {
			m.Inertia.XX.Value = units.KilogramSquareMillimeters(0)
		}},
		{"exact mass with bound", func(m *decad.MassProperties) {
			m.Mass.Exactness = decad.Exact
			m.Mass.Bound = units.Kilograms(0.1)
		}},
		{"exact center with bound", func(m *decad.MassProperties) {
			m.Center.Exactness = decad.Exact
			m.Center.Bound = units.Millimeters(0.1)
		}},
		{"exact inertia with bound", func(m *decad.MassProperties) {
			m.Inertia.XX.Exactness = decad.Exact
			m.Inertia.XX.Bound = units.KilogramSquareMillimeters(0.1)
		}},
		{"invalid mass exactness", func(m *decad.MassProperties) { m.Mass.Exactness = decad.Exactness(-1) }},
		{"invalid center exactness", func(m *decad.MassProperties) { m.Center.Exactness = decad.Exactness(-1) }},
		{"invalid inertia exactness", func(m *decad.MassProperties) { m.Inertia.XX.Exactness = decad.Exactness(-1) }},
		{"mass inverse overflow", func(m *decad.MassProperties) {
			m.Mass.Value = units.Kilograms(1e-320)
			m.Mass.Bound = units.Kilograms(0)
		}},
		{"inertia inverse overflow", func(m *decad.MassProperties) {
			m.Inertia.XX.Value = units.KilogramSquareMillimeters(1e-320)
			m.Inertia.XX.Bound = units.KilogramSquareMillimeters(0)
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			changed := source
			tc.change(&changed)
			cfg := config
			cfg.Bodies = append([]dynamics.RigidBody(nil), config.Bodies...)
			cfg.Bodies[1].Supplied = &changed
			_, err := dynamics.NewWorld(t.Context(), doc, cfg)
			require.ErrorIs(t, err, dynamics.ErrInvalidMassProperties)
		})
	}

	for _, tc := range []struct {
		name    string
		density *units.Value
		mass    *decad.MassProperties
	}{
		{"no source", nil, nil},
		{"both sources", &density, &source},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := config
			cfg.Bodies = append([]dynamics.RigidBody(nil), config.Bodies...)
			cfg.Bodies[1].Density = tc.density
			cfg.Bodies[1].Supplied = tc.mass
			_, err := dynamics.NewWorld(t.Context(), doc, cfg)
			require.ErrorIs(t, err, dynamics.ErrInvalidInput)
		})
	}
}
