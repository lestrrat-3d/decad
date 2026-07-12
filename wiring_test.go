package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// TestWiring exercises the seam decad is built on: a solved 2D sketch profile,
// lifted into world space through the r3 frame of the plane it was drawn on.
// It asserts nothing about decad itself — the kernel API is not designed yet.
// It exists to keep both dependencies honest and to prove the layering
// compiles. Delete it once real decad code imports them.
func TestWiring(t *testing.T) {
	w := sketch.NewWorld()

	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err, `w.CreateSketch should succeed`)

	// A 100 x 60 rectangle, grounded at the origin corner.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)

	_, err = s.Solve(t.Context())
	require.NoError(t, err, `s.Solve should succeed`)

	// sketch answers the 2D question: does this close into an extrudable region?
	profiles := s.Profiles()
	require.Len(t, profiles, 1, `rectangle should yield exactly one profile`)
	require.True(t, profiles[0].Valid, `profile should be extrudable`)
	require.InDelta(t, 6000.0, profiles[0].Area, 1e-9, `profile area should be 100 x 60`)

	// r3 answers the 3D question: where does that region sit in world space?
	f, err := w.XY().Frame()
	require.NoError(t, err, `XY datum should have a valid frame`)

	corner := f.ToWorldUV(100, 60)
	require.Equal(t, r3.NewVec(100, 60, 0), corner, `XY-local (100, 60) is world (100, 60, 0)`)
}
