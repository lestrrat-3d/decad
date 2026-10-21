package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// TestUnionHubAndManyRadialLugs joins a hub with more lugs than any fixed tool
// count once allowed. Each lug adds the same volume, so the joined volume
// grows by N times the single-lug gain, within the proven bounds.
func TestUnionHubAndManyRadialLugs(t *testing.T) {
	if testing.Short() {
		t.Skip("joins 40 sectors")
	}
	doc, hub, lug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.1, false)
	require.Contains(t, doc.Bodies(), hub)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)

	const count = 40
	placed := make([]*decad.Body, 0, count)
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/count))
		require.NoError(t, err)
		copied, err := decad.Copy(t.Context(), lug, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		placed = append(placed, copied)
	}
	// The single-lug gain comes from a second, independent lug so that the
	// first one stays live for the many-lug join.
	singleDoc, singleHub, singleLug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.1, false)
	single, err := decad.Union(t.Context(), singleHub, singleLug)
	require.Contains(t, singleDoc.Bodies(), single)
	require.NoError(t, err)
	singleVolume, err := single.Volume()
	require.NoError(t, err)

	joined, err := decad.Union(t.Context(), hub, append([]*decad.Body{lug}, placed...)...)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))

	volume, err := joined.Volume()
	require.NoError(t, err)
	gain := singleVolume.Value.Base() - hubVolume.Value.Base()
	require.Greater(t, gain, 0.0)
	got := volume.Value.Base() - hubVolume.Value.Base()
	slack := volume.Bound.Base() + hubVolume.Bound.Base() +
		count*(singleVolume.Bound.Base()+hubVolume.Bound.Base())
	require.InDelta(t, count*gain, got, slack)
}
