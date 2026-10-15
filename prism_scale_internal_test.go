package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestScaledAtBoundsFacetedCentroidDiameterAfterVertexRounding(t *testing.T) {
	world := sketch.NewWorld()
	profile, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := profile.CreatePoint(0, 0)
	profile.Fix(center)
	profile.CreateCircle(center, 3)
	_, err = profile.Solve(t.Context())
	require.NoError(t, err)
	doc := New()
	source, err := Extrude(t.Context(), doc, profile, profile.Profiles()[0],
		Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	scaled, err := ScaledAt(t.Context(), source, r3.NewVec(1e6, -2e6, 3e6), units.Scalar(1.3))
	require.NoError(t, err)
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	payload := scaled.n.Payload.(facetedPayload)
	require.GreaterOrEqual(t, payload.dPair, facetedSourceDiameterUpper(payload.Verts, payload.meshBound))
	centroid, err := scaled.Centroid()
	require.NoError(t, err)
	want := r3.NewVec(-3e5, 6e5, -9e5+1.3)
	require.LessOrEqual(t, want.Sub(centroid.Value).Len(), centroid.Bound.Base())
	require.False(t, math.IsInf(centroid.Bound.Base(), 0))

	shift, err := r3.Translation(r3.NewVec(1.25, -2.5, 3.75))
	require.NoError(t, err)
	placed, err := Copy(t.Context(), scaled, WithCopyPlacement(shift))
	require.NoError(t, err)
	placedPayload := placed.n.Payload.(facetedPayload)
	require.GreaterOrEqual(t, placedPayload.dPair,
		facetedSourceDiameterUpper(placedPayload.Verts, placedPayload.meshBound))
	again, err := ScaledAt(t.Context(), placed, r3.NewVec(-2e6, 1e6, 5e5), units.Scalar(0.8))
	require.NoError(t, err)
	againPayload := again.n.Payload.(facetedPayload)
	require.GreaterOrEqual(t, againPayload.dPair,
		facetedSourceDiameterUpper(againPayload.Verts, againPayload.meshBound))
}
