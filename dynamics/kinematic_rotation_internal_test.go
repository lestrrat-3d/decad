package dynamics

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSliceKinematicMotionRejectsDisplacedAxis(t *testing.T) {
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	end, err := turn.Then(shift)
	require.NoError(t, err)
	screw, err := end.Screw()
	require.NoError(t, err)
	linear, angular, ok := screwDriverRates(screw, units.Seconds(1))
	require.True(t, ok)
	motion := kinematicMotion{path: decad.PoseSegment{From: r3.Identity(), To: end,
		Duration: units.Seconds(1)}, effective: linear, angular: angular, screw: &screw}
	w := &World{step: StepConfig{
		Contact:                 decad.ContactRequest{PointResolution: units.Millimeters(1e-6)},
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
	}}
	from, err := screw.At(.5)
	require.NoError(t, err)
	manifold := &decad.ContactManifold{Points: []decad.ContactPoint{{
		OnA: decad.VecMeasurement{Value: r3.Vec{X: 10, Y: 5, Z: 5},
			Bound: units.Millimeters(0)},
	}}}
	_, ok = w.sliceKinematicMotion(motion, from, units.Seconds(.5), r3.Vec{X: 1}, manifold)
	require.True(t, ok)
	transverse, err := r3.Translation(r3.Vec{Y: .01})
	require.NoError(t, err)
	displaced, err := from.Then(transverse)
	require.NoError(t, err)
	_, ok = w.sliceKinematicMotion(motion, displaced, units.Seconds(.5), r3.Vec{X: 1}, manifold)
	require.False(t, ok)
}
