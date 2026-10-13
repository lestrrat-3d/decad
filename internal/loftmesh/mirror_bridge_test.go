package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// nearAxialMotion is the rotation about Z whose basis EZ the library rounds
// to the float just below one.
func nearAxialMotion(t *testing.T) r3.Transform {
	t.Helper()
	tr, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(10*2*math.Pi/17))
	require.NoError(t, err)
	require.Equal(t, math.Nextafter(1, 0), tr.Basis().EZ.Z)
	return tr
}

func TestNearAxialPlacement(t *testing.T) {
	require.True(t, loftmesh.NearAxialPlacement(nearAxialMotion(t)))
	require.False(t, loftmesh.NearAxialPlacement(r3.Identity()))
}

func TestMirrorUpperBridgeShift(t *testing.T) {
	motion := nearAxialMotion(t)
	shift, ok := loftmesh.MirrorUpperBridgeShift(motion, 3)
	require.True(t, ok)
	// 2*middle*(1-EZ.Z) is 6 * 2^-53, rounded up.
	require.InDelta(t, 6*math.Ldexp(1, -53), shift, 1e-30)
	require.GreaterOrEqual(t, shift, 6*math.Ldexp(1, -53))

	for _, middle := range []float64{0, -1, math.Inf(1), math.NaN()} {
		_, ok = loftmesh.MirrorUpperBridgeShift(motion, middle)
		require.False(t, ok)
	}
	_, ok = loftmesh.MirrorUpperBridgeShift(r3.Identity(), 3)
	require.False(t, ok)
}

func TestMirrorUpperBridgeSwept(t *testing.T) {
	swept, ok := loftmesh.MirrorUpperBridgeSwept(0.5, 10, 2)
	require.True(t, ok)
	require.GreaterOrEqual(t, swept, 0.5*12)
	require.InDelta(t, 6, swept, 1e-6)

	_, ok = loftmesh.MirrorUpperBridgeSwept(0.5, math.Inf(1), 0)
	require.False(t, ok)
	_, ok = loftmesh.MirrorUpperBridgeSwept(0.5, 1, math.NaN())
	require.False(t, ok)
}

func TestMirrorBridgeReserve(t *testing.T) {
	verts := []r3.Vec{{X: 1, Y: -2, Z: 5}}
	t.Run("zero shift needs no reserve", func(t *testing.T) {
		got, err := loftmesh.MirrorBridgeReserve(verts, 0.01, 0, 0.5)
		require.NoError(t, err)
		require.Zero(t, got)
	})
	t.Run("reserve covers vertical and lateral motion", func(t *testing.T) {
		got, err := loftmesh.MirrorBridgeReserve(verts, 0.01, 0.001, 0.5)
		require.NoError(t, err)
		require.InDelta(t, 0.04116, got, 1e-12)
		// Z is left out of the span: a large Z must not change the reserve.
		far, err := loftmesh.MirrorBridgeReserve([]r3.Vec{{X: 1, Y: -2, Z: 500}}, 0.01, 0.001, 0.5)
		require.NoError(t, err)
		require.Equal(t, got, far)
	})
	t.Run("setback must exceed the shift", func(t *testing.T) {
		_, err := loftmesh.MirrorBridgeReserve(verts, 0.01, 0.001, 0.001)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "mirrored cap bridge reaches its side station")
	})
	t.Run("reserve of 0.1 mm or more is refused", func(t *testing.T) {
		_, err := loftmesh.MirrorBridgeReserve([]r3.Vec{{X: 2}}, 0.01, 0.01, 0.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "mirrored cap bridge has no finite allowance")
	})
}
