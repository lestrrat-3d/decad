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

// fiveBodyConfig places five separated source boxes along X: a fixed floor,
// two dynamic boxes, a fixed wall and a kinematic pusher.
func fiveBodyConfig(t *testing.T, doc *decad.Document) ([5]*decad.Body, dynamics.WorldConfig) {
	t.Helper()
	var bodies [5]*decad.Body
	for i := range bodies {
		x := float64(40 * i)
		bodies[i] = makeBox(t, doc, x, 0, x+10, 10, 0, 10)
	}
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := func(restitution, friction float64) dynamics.Material {
		return dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(friction)}
	}
	return bodies, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: bodies[0], Role: dynamics.Fixed, Material: material(.5, .0625)},
			{Body: bodies[1], Role: dynamics.Dynamic, Density: &density, Material: material(.25, .25)},
			{Body: bodies[2], Role: dynamics.Dynamic, Density: &density, Material: material(.75, 1)},
			{Body: bodies[3], Role: dynamics.Fixed, Material: material(.5, .5)},
			{Body: bodies[4], Role: dynamics.Kinematic, Material: material(1, 0)},
		},
		Step: pairMaterialStepConfig(),
	}
}

func TestFiveBodyWorldPairTable(t *testing.T) {
	doc := decad.New()
	b, config := fiveBodyConfig(t, doc)
	config.Excluded = []dynamics.BodyPair{{A: b[4], B: b[2]}, {A: b[3], B: b[0]}}
	config.Overrides = []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: b[2], B: b[1]},
		Restitution: units.Scalar(.125), Friction: units.Scalar(.75)}}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)

	require.Equal(t, []dynamics.BodyPair{
		{A: b[0], B: b[1]}, {A: b[0], B: b[2]}, {A: b[0], B: b[3]}, {A: b[0], B: b[4]},
		{A: b[1], B: b[2]}, {A: b[1], B: b[3]}, {A: b[1], B: b[4]},
		{A: b[2], B: b[3]}, {A: b[2], B: b[4]},
		{A: b[3], B: b[4]},
	}, w.Pairs(), "pairs follow canonical world order, excluded pairs included")
	require.Equal(t, []dynamics.BodyPair{{A: b[0], B: b[3]}, {A: b[2], B: b[4]}}, w.Excluded(),
		"exclusions are canonicalized and listed in pair order")

	definitions := w.Bodies()
	require.Len(t, definitions, 5)
	for i, definition := range definitions {
		require.Equal(t, b[i], definition.Body)
		require.Equal(t, config.Bodies[i].Role, definition.Role)
	}
	*definitions[1].Density = units.KilogramsPerCubicMillimeter(1)
	definitions[0].Body = nil
	w.Pairs()[0] = dynamics.BodyPair{}
	w.Excluded()[0] = dynamics.BodyPair{}
	require.Equal(t, units.KilogramsPerCubicMillimeter(0.001), *w.Bodies()[1].Density)
	require.Equal(t, b[0], w.Bodies()[0].Body)
	require.Equal(t, dynamics.BodyPair{A: b[0], B: b[1]}, w.Pairs()[0])
	require.Equal(t, dynamics.BodyPair{A: b[0], B: b[3]}, w.Excluded()[0])

	// NewState takes one entry per body in any order and stores world order.
	entries := make([]dynamics.BodyState, 0, 5)
	for _, i := range []int{3, 1, 4, 0, 2} {
		pose, poseErr := r3.Translation(r3.Vec{Z: float64(i)})
		require.NoError(t, poseErr)
		if config.Bodies[i].Role != dynamics.Dynamic {
			pose = r3.Identity()
		}
		entries = append(entries, dynamics.BodyState{Body: b[i], Pose: pose,
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)})
	}
	state, err := w.NewState(entries)
	require.NoError(t, err)
	stored := state.Entries()
	require.Len(t, stored, 5)
	for i, entry := range stored {
		require.Equal(t, b[i], entry.Body)
	}
	require.InDelta(t, 2, stored[2].Pose.Translation().Z, 0)
	for name, bad := range map[string][]dynamics.BodyState{
		"missing entry":   entries[:4],
		"duplicate entry": append(append([]dynamics.BodyState(nil), entries[:4]...), entries[0]),
		"foreign body": append(append([]dynamics.BodyState(nil), entries[:4]...), dynamics.BodyState{
			Body: makeBox(t, doc, 300, 0, 310, 10, 0, 10), Pose: r3.Identity(),
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)}),
	} {
		t.Run(name, func(t *testing.T) {
			_, err := w.NewState(bad)
			require.ErrorIs(t, err, dynamics.ErrInvalidInput)
		})
	}

	// Every body is still and 30 mm from its neighbors along X, so the broad
	// phase excludes every scheduled pair and the step advances in place.
	dt := units.Seconds(.125)
	drivers := []dynamics.KinematicDriver{{Body: b[4],
		Path: decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: dt}}}
	report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: drivers}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Empty(t, report.Events)
	require.Empty(t, report.Diagnostics)
	require.Equal(t, []dynamics.BodyPair{{A: b[0], B: b[3]}, {A: b[2], B: b[4]}}, report.Excluded)
	for i, entry := range report.Next.Entries() {
		require.Equal(t, stored[i].Pose, entry.Pose)
	}
	for name, input := range map[string]dynamics.StepInput{
		"missing driver": {Gravity: zeroAcceleration()},
		"fixed load": {Gravity: zeroAcceleration(), Drivers: drivers, Loads: []dynamics.BodyLoad{{
			Body: b[0], Force: testForce(1, 0), Torque: testTorque(0)}}},
	} {
		t.Run(name, func(t *testing.T) {
			_, err := w.Step(t.Context(), state, input, dt)
			require.ErrorIs(t, err, dynamics.ErrInvalidInput)
		})
	}
}

func TestFiveBodyWorldExclusionAndOverrideRules(t *testing.T) {
	doc := decad.New()
	b, base := fiveBodyConfig(t, doc)
	outside := makeBox(t, doc, 300, 0, 310, 10, 0, 10)
	pair := func(i, j int) dynamics.BodyPair { return dynamics.BodyPair{A: b[i], B: b[j]} }
	override := func(p dynamics.BodyPair, restitution float64) dynamics.PairMaterial {
		return dynamics.PairMaterial{Pair: p, Restitution: units.Scalar(restitution), Friction: units.Scalar(0)}
	}
	frictionOn := func(cfg *dynamics.WorldConfig, i int, friction units.Value) {
		cfg.Bodies = append([]dynamics.RigidBody(nil), cfg.Bodies...)
		cfg.Bodies[i].Material.Friction = friction
	}
	for _, tc := range []struct {
		name   string
		change func(*dynamics.WorldConfig)
		want   error
	}{
		{name: "exclusion with nil body", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Excluded = []dynamics.BodyPair{{A: nil, B: b[1]}}
		}},
		{name: "exclusion of one body with itself", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Excluded = []dynamics.BodyPair{pair(2, 2)}
		}},
		{name: "exclusion outside the world", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Excluded = []dynamics.BodyPair{{A: b[3], B: outside}}
		}},
		{name: "duplicate exclusion in reverse order", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Excluded = []dynamics.BodyPair{pair(1, 3), pair(3, 1)}
		}},
		{name: "override outside the world", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Overrides = []dynamics.PairMaterial{override(dynamics.BodyPair{A: outside, B: b[2]}, .5)}
		}},
		{name: "duplicate override in reverse order", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Overrides = []dynamics.PairMaterial{override(pair(1, 2), .5), override(pair(2, 1), .5)}
		}},
		{name: "override of an excluded pair", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Excluded = []dynamics.BodyPair{pair(0, 4)}
			c.Overrides = []dynamics.PairMaterial{override(pair(4, 0), .5)}
		}},
		{name: "override restitution above one", want: dynamics.ErrInvalidInput, change: func(c *dynamics.WorldConfig) {
			c.Overrides = []dynamics.PairMaterial{override(pair(0, 1), 2)}
		}},
		{name: "positive friction on a kinematic pair", want: dynamics.ErrUnsupported, change: func(c *dynamics.WorldConfig) {
			frictionOn(c, 4, units.Scalar(.25))
		}},
		{name: "kinematic friction with its moving pairs excluded or zeroed", change: func(c *dynamics.WorldConfig) {
			frictionOn(c, 4, units.Scalar(.25))
			c.Excluded = []dynamics.BodyPair{pair(0, 4), pair(1, 4), pair(2, 4)}
			c.Overrides = []dynamics.PairMaterial{override(pair(4, 3), .5)}
		}},
		{name: "unrepresentable friction mean on a moving pair", want: dynamics.ErrUnsupported,
			change: func(c *dynamics.WorldConfig) {
				frictionOn(c, 1, overflowFriction)
				frictionOn(c, 2, overflowFriction)
			}},
		{name: "unrepresentable friction mean on a fixed pair", change: func(c *dynamics.WorldConfig) {
			frictionOn(c, 0, overflowFriction)
			frictionOn(c, 3, overflowFriction)
			for _, dynamic := range []int{1, 2} {
				frictionOn(c, dynamic, units.Scalar(0))
			}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := base
			tc.change(&cfg)
			_, err := dynamics.NewWorld(t.Context(), doc, cfg)
			if tc.want == nil {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, tc.want)
		})
	}

	single := base
	single.Bodies = base.Bodies[:1]
	_, err := dynamics.NewWorld(t.Context(), doc, single)
	require.ErrorIs(t, err, dynamics.ErrUnsupported)
}

// The three-body step resolves a pair through the parent table, so a pair
// override replaces the body coefficients in the published rebound.
func TestThreeBodyStepReadsPairOverride(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	remote := makeBox(t, doc, -5, -5, 5, 5, -50, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: remote, Role: dynamics.Fixed, Material: material},
		},
		Overrides: []dynamics.PairMaterial{
			{Pair: dynamics.BodyPair{A: box, B: floor}, Restitution: units.Scalar(.25), Friction: units.Scalar(0)},
			{Pair: dynamics.BodyPair{A: remote, B: floor}, Restitution: units.Scalar(1), Friction: units.Scalar(2)},
		},
		Step: pairMaterialStepConfig(),
	})
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: remote, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, report.Events[0].Pair)
	// The box is 1 kg; restitution 1/4 turns -100 mm/s into +25 mm/s, so the
	// impulse is 125 kg·mm/s and the box rises 2.5 mm in the remaining 0.1 s.
	// The slack covers the rounded contact bracket only.
	require.InDelta(t, 125, report.Events[0].NormalImpulse.Base(), 1e-6)
	endBox, found := report.Next.Body(box)
	require.True(t, found)
	require.InDelta(t, 25, endBox.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 2.5, endBox.Pose.Translation().Z, 2e-6)
}

// TestNewWorldAdmitsRotatedDensityBox feeds a box placed 30° about (1,1,1)
// through density-derived mass into NewWorld, whose own validation re-proves
// the published tensor positive (docs/multibody-dynamics-design.md §8.1).
func TestNewWorldAdmitsRotatedDensityBox(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -50, -50, 50, 50, -10, 10)
	box := makeBox(t, doc, 0, 0, 20, 10, 40, 30)
	pose, err := r3.RotationAround(r3.NewVec(10, 5, 55), r3.NewVec(1, 1, 1), units.Degrees(30))
	require.NoError(t, err)
	turned, err := box.Placed(t.Context(), pose)
	require.NoError(t, err)
	mass, err := turned.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	// The rotation mixes the axes, so the consumer receives nonzero products.
	require.Greater(t, math.Abs(mass.Inertia.XY.Value.Base()), mass.Inertia.XY.Bound.Base())
	w := fixedBoxContactWorld(t, doc, floor, turned, 0)
	require.Len(t, w.Bodies(), 2)
	require.Equal(t, dynamics.Dynamic, w.Bodies()[1].Role)
}

// elongatedObliqueBox builds a 60×10×10 mm box turned 45° about (1,1,0)
// through its center and reads its density-derived mass from decad.
func elongatedObliqueBox(t *testing.T, doc *decad.Document) (*decad.Body, decad.MassProperties) {
	t.Helper()
	box := makeBox(t, doc, -30, -5, 30, 5, 95, 10)
	pose, err := r3.RotationAround(r3.NewVec(0, 0, 100), r3.NewVec(1, 1, 0), units.Degrees(45))
	require.NoError(t, err)
	turned, err := box.Placed(t.Context(), pose)
	require.NoError(t, err)
	mass, err := turned.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	return turned, mass
}

// TestNewWorldAdmitsElongatedObliqueBox feeds a body whose products of
// inertia outweigh a diagonal entry through the real decad producer into
// NewWorld. Row dominance cannot prove this tensor positive; the leading
// principal minors over the published intervals can.
func TestNewWorldAdmitsElongatedObliqueBox(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	turned, mass := elongatedObliqueBox(t, doc)

	// The 6 kg box has principal moments 100 (long axis) and 1850 kg·mm².
	// With u the long axis after the turn, the world tensor is
	// 1850·I − 1750·u·uᵀ, and Rodrigues' formula gives u = (½+√2/4, ½−√2/4, −½).
	// The slack covers decad's published bounds and float rounding of the
	// expectation.
	u := r3.NewVec(0.5+math.Sqrt2/4, 0.5-math.Sqrt2/4, -0.5)
	expected := map[string]struct {
		got  decad.Measurement
		want float64
	}{
		"XX": {mass.Inertia.XX, 1850 - 1750*u.X*u.X},
		"YY": {mass.Inertia.YY, 1850 - 1750*u.Y*u.Y},
		"ZZ": {mass.Inertia.ZZ, 1850 - 1750*u.Z*u.Z},
		"XY": {mass.Inertia.XY, -1750 * u.X * u.Y},
		"XZ": {mass.Inertia.XZ, -1750 * u.X * u.Z},
		"YZ": {mass.Inertia.YZ, -1750 * u.Y * u.Z},
	}
	for name, entry := range expected {
		got, err := entry.got.Value.In(units.KilogramSquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, entry.want, got, 1e-6, name)
	}
	// Row X fails dominance by hundreds of kg·mm², far beyond every bound.
	dominance := mass.Inertia.XX.Value.Base() - mass.Inertia.XX.Bound.Base() -
		math.Abs(mass.Inertia.XY.Value.Base()) - mass.Inertia.XY.Bound.Base() -
		math.Abs(mass.Inertia.XZ.Value.Base()) - mass.Inertia.XZ.Bound.Base()
	require.Negative(t, dominance)

	w := fixedBoxContactWorld(t, doc, floor, turned, 0)
	require.Equal(t, dynamics.Dynamic, w.Bodies()[1].Role)

	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: turned, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		},
		Step: pairMaterialStepConfig(),
	}
	_, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err, "the supplied record takes the same proof")
}

// TestNewWorldInertiaPositivityProof drives supplied records derived from
// the elongated oblique box through NewWorld. None of the refused fixtures
// passes row dominance, so each refusal rests on the leading-minor proof.
//
// Each leg below was weakened in world.go's principalMinorFloor or
// validateMass, watched to turn its fixture's case red (wrongly admitted),
// and restored:
//   - Building each entry from its value instead of value ± bound admits
//     "interval reaches a singular tensor".
//   - Dropping the 3×3 determinant check admits "interval reaches a singular
//     tensor".
//   - Dropping the 2×2 minor check admits "negative second minor".
//   - Dropping the 1×1 minor check admits "negative first minor".
//   - Dropping the finite-inverse check on the floor, or taking the smallest
//     diagonal lower end as the floor, admits "eigenvalue floor inverse
//     overflows".
//
// The positive-trace guard is redundant: three positive minors already prove
// every member positive definite, so its trace is positive. It only keeps the
// floor's division defined. Using the trace's lower end in place of its upper
// end is not separately exercised: these fixtures' trace intervals are too
// narrow for the two ends to reach different verdicts.
func TestNewWorldInertiaPositivityProof(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	turned, source := elongatedObliqueBox(t, doc)
	mm2 := func(m decad.Measurement) float64 {
		value, err := m.Value.In(units.KilogramSquareMillimeter)
		require.NoError(t, err)
		return value
	}
	in := source.Inertia
	xx, yy, zz := mm2(in.XX), mm2(in.YY), mm2(in.ZZ)
	xy, xz, yz := mm2(in.XY), mm2(in.XZ), mm2(in.YZ)
	// The determinant is linear in XX with slope cofYZ, so widening XX's
	// bound past det/cofYZ puts a singular tensor inside the intervals while
	// the published values stay positive definite.
	cofYZ := yy*zz - yz*yz
	det := xx*cofYZ - xy*(xy*zz-yz*xz) + xz*(xy*yz-yy*xz)
	require.Positive(t, det)
	singularReach := det / cofYZ
	require.Less(t, singularReach, xx/2, "XX's lower end and the 2×2 minor stay positive")

	widenXX := func(factor float64) func(*decad.InertiaReading) {
		return func(r *decad.InertiaReading) {
			r.XX.Exactness = decad.Approximate
			r.XX.Bound = units.KilogramSquareMillimeters(factor * singularReach)
		}
	}
	diagonal := func(x, y, z float64) func(*decad.InertiaReading) {
		return func(r *decad.InertiaReading) {
			exact := func(v float64) decad.Measurement {
				return decad.Measurement{Value: units.KilogramSquareMillimeters(v), Exactness: decad.Exact,
					Bound: units.KilogramSquareMillimeters(0)}
			}
			*r = decad.InertiaReading{XX: exact(x), YY: exact(y), ZZ: exact(z),
				XY: exact(0), XZ: exact(0), YZ: exact(0)}
		}
	}
	// Scaling by a power of two keeps the tensor positive definite: values
	// below float64's normal range lose at most a few parts in 10^16, far
	// inside the 100:1850 eigenvalue spread. The floor 4·det/trace² is about
	// 94.8 kg·mm² before scaling, against a smallest eigenvalue of 100 and a
	// smallest diagonal near 575. At 2^-1032 the floor's reciprocal is near
	// 4e308, past float64's 1.8e308, while the smallest diagonal's is near
	// 6.5e307. At 2^-1028 the floor's reciprocal is near 2.5e307 and fits.
	scale := func(exponent int) func(*decad.InertiaReading) {
		return func(r *decad.InertiaReading) {
			for _, m := range []*decad.Measurement{&r.XX, &r.YY, &r.ZZ, &r.XY, &r.XZ, &r.YZ} {
				m.Value = units.KilogramSquareMillimeters(math.Ldexp(mm2(*m), exponent))
				m.Bound = units.KilogramSquareMillimeters(math.Ldexp(m.Bound.Mag(), exponent))
			}
		}
	}
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	for _, tc := range []struct {
		name   string
		change func(*decad.InertiaReading)
		admit  bool
	}{
		// 0.99 leaves det·0.01 of margin, far above the other components'
		// published bounds, so the proof admits just short of singular.
		{"interval stops short of a singular tensor", widenXX(0.99), true},
		{"interval reaches a singular tensor", widenXX(1.01), false},
		// Both diagonals keep a positive determinant (10) and trace (8), so
		// only the named minor can refuse them.
		{"negative second minor", diagonal(10, -1, -1), false},
		{"negative first minor", diagonal(-1, -1, 10), false},
		{"eigenvalue floor inverse fits", scale(-1028), true},
		{"eigenvalue floor inverse overflows", scale(-1032), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			supplied := source
			tc.change(&supplied.Inertia)
			_, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: []dynamics.RigidBody{
					{Body: floor, Role: dynamics.Fixed, Material: material},
					{Body: turned, Role: dynamics.Dynamic, Supplied: &supplied, Material: material},
				},
				Step: pairMaterialStepConfig(),
			})
			if tc.admit {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, dynamics.ErrInvalidMassProperties)
		})
	}
}
