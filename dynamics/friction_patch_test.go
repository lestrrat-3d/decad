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

func TestFixedFloorFrictionPatchKeepsExactProducerLever(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -4.9, -5, 5.3, 5, 0)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	mass, err := box.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(.001))
	require.NoError(t, err)
	var roundedLeverDiffers bool
	for _, witness := range contact.Manifold.Points {
		exact := new(big.Rat).Sub(ratFloat(witness.OnB.Value.X), ratFloat(mass.Center.Value.X))
		rounded := ratFloat(witness.OnB.Value.X - mass.Center.Value.X)
		lever := exactPatchLever(witness.OnB.Value, mass.Center.Value)
		require.Zero(t, exact.Cmp(lever[0]))
		if lever[0].Cmp(rounded) != 0 {
			roundedLeverDiffers = true
		}
	}
	require.True(t, roundedLeverDiffers)
	pre := QuantityVec{X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(-100)}
	cfg := StepConfig{Contact: request, VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), MaxIterations: 64}
	response, ok := solveFixedFloorFrictionPatch(contact.Manifold, mass, r3.Identity(), pre,
		units.Scalar(.5), cfg)
	require.True(t, ok)
	require.LessOrEqual(t, response.AngularUpper.Base(), cfg.AngularVelocityResidual.Base())
}

func TestFixedFloorFrictionPatchUsesRealManifoldAndMass(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, 1, mass.Mass.Value.Base(), 1e-12)
	require.Equal(t, r3.Vec{Z: 5}, mass.Center.Value)
	pre := QuantityVec{X: units.MillimetersPerSecond(100),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(-100)}
	cfg := StepConfig{Contact: request, VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), MaxIterations: 64}
	response, ok := solveFixedFloorFrictionPatch(contact.Manifold, mass, r3.Identity(), pre, units.Scalar(.5), cfg)
	require.True(t, ok)
	require.Len(t, response.Points, 4)
	require.InDelta(t, 50, response.Post.X.Base(), 1e-6)
	require.Zero(t, response.Post.Y.Base())
	require.Zero(t, response.Post.Z.Base())
	require.LessOrEqual(t, response.AngularUpper.Base(), cfg.AngularVelocityResidual.Base())
	var normal, tangentX, tangentY float64
	var torque r3.Vec
	for i, impulse := range response.Points {
		normal += impulse.Normal.Base()
		tangentX += impulse.TangentX.Base()
		tangentY += impulse.TangentY.Base()
		arm := contact.Manifold.Points[i].OnB.Value.Sub(mass.Center.Value)
		torque = torque.Add(arm.Cross(r3.Vec{X: impulse.TangentX.Base(),
			Y: impulse.TangentY.Base(), Z: impulse.Normal.Base()}))
		require.GreaterOrEqual(t, impulse.Normal.Base(), 0.0)
		require.LessOrEqual(t, math.Hypot(impulse.TangentX.Base(), impulse.TangentY.Base()),
			.5*impulse.Normal.Base()+cfg.ImpulseResidual.Base())
	}
	require.InDelta(t, 100, normal, 1e-6)
	require.InDelta(t, -50, tangentX, 1e-6)
	require.InDelta(t, 0, tangentY, 1e-6)
	require.LessOrEqual(t, math.Abs(torque.X)+math.Abs(torque.Y)+math.Abs(torque.Z),
		mass.Inertia.XX.Value.Base()*cfg.AngularVelocityResidual.Base())
	cfg.MaxIterations = 8
	_, ok = solveFixedFloorFrictionPatch(contact.Manifold, mass, r3.Identity(), pre, units.Scalar(.5), cfg)
	require.False(t, ok)
	cfg.MaxIterations = 64
	widened := mass
	widened.Mass.Bound = units.Kilograms(.01)
	widened.Mass.Exactness = decad.Approximate
	_, ok = solveFixedFloorFrictionPatch(contact.Manifold, widened, r3.Identity(), pre, units.Scalar(.5), cfg)
	require.False(t, ok)
	translated, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	_, ok = solveFixedFloorFrictionPatch(contact.Manifold, mass, translated, pre, units.Scalar(.5), cfg)
	require.False(t, ok)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
}
