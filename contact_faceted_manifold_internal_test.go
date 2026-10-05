package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlanarManifoldMatchesClippedBoxPatch is docs/multibody-dynamics-design.md
// §9.4's parity check: a box turned 30° about z resting on a narrower box
// publishes eight clipped corners through contact_clipped_patch.go, and the
// general exact clip publishes the same eight points, bit for bit, with the
// same faces, normals and separations.
func TestPlanarManifoldMatchesClippedBoxPatch(t *testing.T) {
	doc := New()
	narrow := internalOffsetBox(t, doc, -5, -5, 5, 5, -10, Distance{D: units.Millimeters(10), Dir: Along})
	box := internalOffsetBox(t, doc, -5, -5, 5, 5, 0, Distance{D: units.Millimeters(10), Dir: Along})
	turn, err := r3.RotationAround(r3.Vec{Z: 5}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	req := ContactRequest{PointResolution: units.Millimeters(1e-6), NormalResolution: units.Degrees(1)}

	shipped, err := doc.ContactPair(t.Context(), narrow, box, r3.Identity(), turn, req)
	require.NoError(t, err)
	require.Equal(t, ContactTouching, shipped.Relation)
	require.NotNil(t, shipped.Manifold)
	require.Len(t, shipped.Manifold.Points, 8)

	general := &ContactReport{A: narrow, B: box, PoseA: r3.Identity(), PoseB: turn, Request: req}
	planar, err := classifyExactPlanarPair(t.Context(), general)
	require.NoError(t, err)
	require.True(t, planar)
	require.Equal(t, ContactTouching, general.Relation)
	require.NotNil(t, general.Manifold, "reason=%v", general.Reason)
	require.Len(t, general.Manifold.Points, 8)
	for _, want := range shipped.Manifold.Points {
		require.Contains(t, general.Manifold.Points, want)
	}
}
