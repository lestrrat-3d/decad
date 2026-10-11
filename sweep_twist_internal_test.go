package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A short twisted extrusion translated far along Z makes the held caps move
// axially by more than the horizontal-section tube alone can charge. The
// Loft proof's own occupied-volume term must cover that second displacement.
func TestTwistPlacedAxialRoundVolumeProof(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 1e-4)})
	require.NoError(t, err)
	body, err := New().Sweep(t.Context(), s, s.Profiles()[0], path, WithSweepTwist(units.Radians(1e-6)))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 1e8))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(1e-4), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	vertices := mesh.Vertices()
	for i := range vertices {
		vertices[i].Z -= 1e8
	}
	heldVolume := 0.0
	for _, tri := range mesh.Triangles() {
		heldVolume += vertices[tri[0]].Dot(vertices[tri[1]].Cross(vertices[tri[2]])) / 6
	}
	trueVolume, err := placed.Volume()
	require.NoError(t, err)
	gap := math.Abs(heldVolume - trueVolume.Value.Base())
	require.Greater(t, gap, 5e-9)
	require.LessOrEqual(t, gap, mesh.volSymDiff+trueVolume.Bound.Base())
}
