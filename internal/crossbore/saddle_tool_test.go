package crossbore_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestProveToolBand proves the saddle chamfer's band for a radius-10 shaft
// and a radius-3 bore under a placed pose, and shows that the held-sample
// comparison reads a moved band end.
func TestProveToolBand(t *testing.T) {
	t.Parallel()
	const R, r, d = 10.0, 3.0, 0.5
	pose := crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}}
	held := func(j int) (r3.Vec, r3.Vec) {
		o, b := crossbore.BandEnds(R, r, d, j)
		return pose.World(o), pose.World(b)
	}
	proof, err := crossbore.ProveToolBand(t.Context(), R, r, d, pose, held)
	require.NoError(t, err)
	require.Positive(t, proof.Chord)
	require.Less(t, proof.SampleError, 1e-12, "the held ends are the exact ends rounded")
	require.Less(t, proof.Chord, d/10, "the chord bound is a small part of the setback")
	require.Less(t, proof.AreaLo, proof.AreaHi)
	require.Positive(t, proof.AreaLo)

	const moved = 1e-4
	shifted := func(j int) (r3.Vec, r3.Vec) {
		o, b := held(j)
		if j == 77 {
			o.X += moved
		}
		return o, b
	}
	off, err := crossbore.ProveToolBand(t.Context(), R, r, d, pose, shifted)
	require.NoError(t, err)
	require.GreaterOrEqual(t, off.SampleError, moved, "the comparison reads the moved end")
	require.Greater(t, off.Chord, proof.Chord+moved/2)
}
