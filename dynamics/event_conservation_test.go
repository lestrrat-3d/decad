package dynamics

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func conservationBox(t *testing.T, doc *decad.Document, x0, x1 float64) *decad.Body {
	t.Helper()
	scene := sketch.NewWorld()
	profile, err := scene.CreateSketch(scene.XY())
	require.NoError(t, err)
	rectangle := profile.CreateRectangle(x0, -5, x1, 5)
	profile.Fix(rectangle.A)
	_, err = profile.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(profile, profile.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

func TestEventConservationRejectsEnergyGainWithBalancedMomentum(t *testing.T) {
	doc := decad.New()
	a := conservationBox(t, doc, 0, 10)
	b := conservationBox(t, doc, 35, 45)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: a, Role: Dynamic, Density: &density, Material: material},
			{Body: b, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	velocity := func(x float64) QuantityVec {
		return QuantityVec{X: units.MillimetersPerSecond(x),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	}
	angular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: velocity(100), AngularVelocity: angular},
		{Body: b, Pose: r3.Identity(), LinearVelocity: velocity(-100), AngularVelocity: angular},
	})
	require.NoError(t, err)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: zeroGravity}, units.Seconds(.25))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Empty(t, w.eventConservationFailure(event))

	// This changed pair retains total momentum and each body's stated impulse.
	// Its post-event energy exceeds the real event's pre-event energy.
	gaining := event
	gaining.NormalImpulse = units.KilogramMillimetersPerSecond(250)
	gaining.PostVelocityA = velocity(-150)
	gaining.PostVelocityB = velocity(150)
	require.Equal(t, "contact event increases kinetic energy beyond numerical residual",
		w.eventConservationFailure(gaining))

	// A wider held mass interval must not excuse the same numerical impulse.
	w.parts[0].mass.Mass.Bound = units.Kilograms(.1)
	require.Equal(t, "contact event linear momentum exceeds impulse residual",
		w.eventConservationFailure(event))
}
