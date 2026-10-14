package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStationedTwistOccupiedVolumeProof(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	body, err := Sweep(t.Context(), New(), s, s.Profiles()[0], path,
		WithSweepTwist(units.Radians(2)))
	require.NoError(t, err)
	payload, ok := body.n.Payload.(twistedSweepPayload)
	require.True(t, ok)
	require.NotNil(t, payload.mesh)
	volume, err := body.Volume()
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	t.Logf("cells=%d boundary=%g occupied_volume=%g volume=%g volume_bound=%g volume_exactness=%v area=%g area_bound=%g",
		len(payload.mesh.n.Vertices)/4-1, payload.delta, payload.volSymDiff, volume.Value.Base(),
		volume.Bound.Base(), volume.Exactness, area.Value.Base(), area.Bound.Base())
	require.LessOrEqual(t, payload.delta, 0.1)
	require.Less(t, payload.volSymDiff, 20.0)
	heldVolume := exactMeshVolume(payload.mesh)
	heldFloat, _ := heldVolume.Float64()
	t.Logf("held_volume=%g", heldFloat)
	gap := new(big.Rat).Abs(new(big.Rat).Sub(heldVolume, big.NewRat(20, 1)))
	require.LessOrEqual(t, gap.Cmp(new(big.Rat).SetFloat64(payload.volSymDiff)), 0)
}

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
	body, err := Sweep(t.Context(), New(), s, s.Profiles()[0], path, WithSweepTwist(units.Radians(1e-6)))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 1e8))
	require.NoError(t, err)
	placed, err := Transform(t.Context(), body, move)
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
	require.LessOrEqual(t, gap, mesh.n.VolSymDiff+trueVolume.Bound.Base())
}
