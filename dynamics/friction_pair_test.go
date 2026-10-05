package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Two 1 kg boxes stack face to face; the upper one strikes at
// (100, 0, −100) mm/s with friction 0.5. Zero restitution gives them a
// common normal speed of −50 mm/s from a 50 kg·mm/s normal impulse, and the
// patch sticks: a 20 kg·mm/s tangent impulse leaves the lower box at 20 and
// the upper at 80 mm/s, both turning at 6 rad/s about Y so the patch's
// points move together. A lower box of twice the density takes the normal
// impulse 100/1.5, and restitution 0.5 separates the pair at 50 mm/s.
// Each case asserts the step's certified impact.
func TestTwoDynamicPatchUsesBothBodiesMassAndInertia(t *testing.T) {
	doc := decad.New()
	a := sourceBoxForFriction(t, doc, -5, -5, 5, 5, -10)
	b := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	density := units.KilogramsPerCubicMillimeter(.001)
	cfg := frictionPatchConfig()
	sums := func(event ContactEvent) (float64, float64) {
		t.Helper()
		var normal, tangent float64
		for _, point := range event.PointImpulses {
			normal += point.Normal.Base()
			tangent += point.Tangent.X.Base()
			require.LessOrEqual(t, math.Hypot(point.Tangent.X.Base(), point.Tangent.Y.Base()),
				.5*point.Normal.Base()+cfg.ImpulseResidual.Base())
		}
		return normal, tangent
	}
	run := func(densityA units.Value, restitution float64) (ContactEvent, decad.MassProperties) {
		t.Helper()
		material := Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(.5)}
		_, report := stepPatch(t, doc, RigidBody{Body: a, Role: Dynamic, Density: &densityA, Material: material},
			RigidBody{Body: b, Role: Dynamic, Density: &density, Material: material},
			r3.Identity(), r3.Identity(), r3.Vec{X: 100, Z: -100})
		// The certified impact is the step's first event. A pair left
		// spinning in touch has no certified touch track in the root
		// package, so such a step stops after it with StepPairUndecided.
		require.NotEmpty(t, report.Events, "%+v", report.Diagnostics)
		if report.Status != Advanced {
			require.Len(t, report.Diagnostics, 1)
			require.Equal(t, StepPairUndecided, report.Diagnostics[0].Code)
			require.Equal(t, report.Events[0].Time, report.Diagnostics[0].From)
		}
		ma, err := a.MassProperties(t.Context(), densityA)
		require.NoError(t, err)
		return report.Events[0], ma
	}
	mb, err := b.MassProperties(t.Context(), density)
	require.NoError(t, err)

	event, ma := run(density, 0)
	require.Len(t, event.PointImpulses, 4)
	require.InDelta(t, 20, event.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, 80, event.PostVelocityB.X.Base(), 1e-6)
	require.InDelta(t, -50, event.PostVelocityA.Z.Base(), 1e-6)
	require.InDelta(t, -50, event.PostVelocityB.Z.Base(), 1e-6)
	require.InDelta(t, 6, event.PostAngularVelocityA.Y.Base(), 1e-6)
	require.InDelta(t, 6, event.PostAngularVelocityB.Y.Base(), 1e-6)
	normal, tangent := sums(event)
	require.InDelta(t, 50, normal, 1e-6)
	require.InDelta(t, -20, tangent, 1e-6)
	require.InDelta(t, -tangent, ma.Mass.Value.Base()*event.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, tangent, mb.Mass.Value.Base()*(event.PostVelocityB.X.Base()-100), 1e-6)
	// AngularUpper bounds the published 6 rad/s from above.
	require.GreaterOrEqual(t, event.Solver.AngularUpper.Base(), event.PostAngularVelocityB.Y.Base())

	unequal, heavyA := run(units.KilogramsPerCubicMillimeter(.002), 0)
	unequalNormal, unequalTangent := sums(unequal)
	require.InDelta(t, 100.0/(.5+1), unequalNormal, 1e-6)
	require.InDelta(t, -100.0/3, unequal.PostVelocityA.Z.Base(), 1e-6)
	require.InDelta(t, -100.0/3, unequal.PostVelocityB.Z.Base(), 1e-6)
	require.InDelta(t, -unequalTangent, heavyA.Mass.Value.Base()*unequal.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, unequalTangent, mb.Mass.Value.Base()*(unequal.PostVelocityB.X.Base()-100), 1e-6)

	bouncing, _ := run(density, .5)
	require.InDelta(t, 60, bouncing.PostVelocityB.X.Base()-bouncing.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, 50, bouncing.PostVelocityB.Z.Base()-bouncing.PostVelocityA.Z.Base(), 1e-6)
	require.InDelta(t, 6, bouncing.PostAngularVelocityA.Y.Base(), 1e-6)
	require.InDelta(t, bouncing.PostAngularVelocityA.Y.Base(), bouncing.PostAngularVelocityB.Y.Base(), 1e-6)
	require.LessOrEqual(t, bouncing.Solver.NormalResidual.Base(), cfg.VelocityResidual.Base())
}
