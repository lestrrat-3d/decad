package dynamics

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sourceBoxForFriction(t *testing.T, doc *decad.Document,
	x0, y0, x1, y1, z0 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rectangle := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rectangle.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

// frictionPatchConfig is the step configuration of the patch fixtures.
func frictionPatchConfig() StepConfig {
	return StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096}
}

// stepPatch steps two bodies for 0.1 s without gravity: a at rest, b moving
// at vb, both at their poses.
func stepPatch(t *testing.T, doc *decad.Document, a, b RigidBody, poseA, poseB r3.Transform,
	vb r3.Vec) (*World, *StepReport) {
	t.Helper()
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{a, b}, Step: frictionPatchConfig()})
	require.NoError(t, err)
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	velocity := func(v r3.Vec) QuantityVec {
		return QuantityVec{X: units.MillimetersPerSecond(v.X), Y: units.MillimetersPerSecond(v.Y),
			Z: units.MillimetersPerSecond(v.Z)}
	}
	start, err := w.NewState([]BodyState{
		{Body: a.Body, Pose: poseA, LinearVelocity: velocity(r3.Vec{}), AngularVelocity: zeroW},
		{Body: b.Body, Pose: poseB, LinearVelocity: velocity(vb), AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	return w, report
}

// The box's patch witnesses sit off its mass center by amounts whose float
// difference rounds, so the lever the island reads must be the exact
// producer lever: the certified spin stays within AngularVelocityResidual.
func TestFixedFloorFrictionPatchKeepsExactProducerLever(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -4.9, -5, 5.3, 5, 0)
	request := frictionPatchConfig().Contact
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	var roundedLeverDiffers bool
	for _, witness := range contact.Manifold.Points {
		exact := new(big.Rat).Sub(ratFloat(witness.OnB.Value.X), ratFloat(mass.Center.Value.X))
		if exact.Cmp(ratFloat(witness.OnB.Value.X-mass.Center.Value.X)) != 0 {
			roundedLeverDiffers = true
		}
	}
	require.True(t, roundedLeverDiffers)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	_, report := stepPatch(t, doc, RigidBody{Body: floor, Role: Fixed, Material: material},
		RigidBody{Body: box, Role: Dynamic, Density: &density, Material: material},
		r3.Identity(), r3.Identity(), r3.Vec{X: 100, Z: -100})
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.NotNil(t, report.Events[0].Solver)
	require.LessOrEqual(t, report.Events[0].Solver.AngularUpper.Base(),
		frictionPatchConfig().AngularVelocityResidual.Base())
}

// A centered 1 kg box strikes the floor at (100, 0, −100) mm/s with zero
// restitution and friction 0.5: the floor stops its fall with 100 kg·mm/s
// and its friction takes 50 kg·mm/s of the slide, at the cone, leaving
// (50, 0, 0) mm/s and no spin.
func TestFixedFloorFrictionPatchUsesRealManifoldAndMass(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	require.Equal(t, r3.Vec{Z: 5}, mass.Center.Value)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	cfg := frictionPatchConfig()
	_, report := stepPatch(t, doc, RigidBody{Body: floor, Role: Fixed, Material: material},
		RigidBody{Body: box, Role: Dynamic, Density: &density, Material: material},
		r3.Identity(), r3.Identity(), r3.Vec{X: 100, Z: -100})
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Len(t, event.PointImpulses, 4)
	require.InDelta(t, 50, event.PostVelocityB.X.Base(), 1e-6)
	require.InDelta(t, 0, event.PostVelocityB.Y.Base(), 1e-6)
	require.InDelta(t, 0, event.PostVelocityB.Z.Base(), 1e-6)
	require.LessOrEqual(t, event.Solver.AngularUpper.Base(), cfg.AngularVelocityResidual.Base())
	var normal, tangentX, tangentY float64
	var torque r3.Vec
	for i, impulse := range event.PointImpulses {
		normal += impulse.Normal.Base()
		tangentX += impulse.Tangent.X.Base()
		tangentY += impulse.Tangent.Y.Base()
		arm := event.Manifold.Points[i].OnB.Value.Sub(mass.Center.Value)
		torque = torque.Add(arm.Cross(r3.Vec{X: impulse.Tangent.X.Base(),
			Y: impulse.Tangent.Y.Base(), Z: impulse.Normal.Base()}))
		require.GreaterOrEqual(t, impulse.Normal.Base(), 0.0)
		require.LessOrEqual(t, math.Hypot(impulse.Tangent.X.Base(), impulse.Tangent.Y.Base()),
			.5*impulse.Normal.Base()+cfg.ImpulseResidual.Base())
	}
	require.InDelta(t, 100, normal, 1e-6)
	require.InDelta(t, -50, tangentX, 1e-6)
	require.InDelta(t, 0, tangentY, 1e-6)
	require.LessOrEqual(t, math.Abs(torque.X)+math.Abs(torque.Y)+math.Abs(torque.Z),
		mass.Inertia.XX.Value.Base()*cfg.AngularVelocityResidual.Base())
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}
