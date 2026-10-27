package decad

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// wideRectTwist sweeps a w x t centered rectangle straight up by length with a
// twist of deg degrees, from the origin XY sketch.
func wideRectTwist(t *testing.T, w, th, length, deg float64) (*Body, error) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-w/2, -th/2, w/2, th/2)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, length)})
	require.NoError(t, err)
	return Sweep(t.Context(), New(), s, s.Profiles()[0], path, WithSweepTwist(units.Degrees(deg)))
}

func TestTwistWideThinRectangleAngles(t *testing.T) {
	const w, th, length = 15.4, 4.75, 6.0
	angles := []float64{43.6}
	for a := 2.5; a <= 90; a += 2.5 {
		angles = append(angles, a)
	}
	for _, deg := range angles {
		// A single flat Loft cell (about 30 degrees and below) certifies a
		// coarse boundary; the stationed build (the folded 35-55 degree
		// band, and above) is finer. 43.6 degrees is the screw gear's.
		tol := 5.0
		if deg == 43.6 {
			tol = 0.5
		}
		t.Run(fmt.Sprintf("%gdeg", deg), func(t *testing.T) {
			body, err := wideRectTwist(t, w, th, length, deg)
			require.NoError(t, err)
			vol, err := body.Volume()
			require.NoError(t, err)
			want := w * th * length
			require.InDelta(t, want, vol.Value.Base(), 1e-6*want+vol.Bound.Base())
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(tol), WithVerification(VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// A full-turn scan of the same section keeps the stationed build inside its
// 64-cell ceiling, in coarse steps to bound the test's cost.
func TestTwistWideThinRectangleFullTurn(t *testing.T) {
	const w, th, length = 15.4, 4.75, 24.0
	for _, deg := range []float64{100, 135, 180, 225, 270, 315, 360} {
		t.Run(fmt.Sprintf("%gdeg", deg), func(t *testing.T) {
			body, err := wideRectTwist(t, w, th, length, deg)
			require.NoError(t, err)
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5), WithVerification(VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// The screw gear builds its twisted bore at the origin and moves it onto a
// tilted, offset axis: Transform keeps the stationed twist usable.
func TestTwistWideThinRectangleTransform(t *testing.T) {
	const w, th, length = 15.4, 4.75, 6.0
	body, err := wideRectTwist(t, w, th, length, 43.6)
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 0.3, 0), units.Degrees(23))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(30, -12, 7))
	require.NoError(t, err)
	both, err := rot.Then(move)
	require.NoError(t, err)
	placed, err := Transform(t.Context(), body, both)
	require.NoError(t, err)
	vol, err := placed.Volume()
	require.NoError(t, err)
	want := w * th * length
	require.InDelta(t, want, vol.Value.Base(), 1e-6*want+vol.Bound.Base())
	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.5), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
