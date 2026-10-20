package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlacedAxisPrismBooleans passes real sketch extrusions through Copy
// and the public Boolean operations. The matching section coordinates and
// exact axial displacement let the existing prism and stacked builders prove
// the result without falling to a coplanar mesh contact.
func TestPlacedAxisPrismBooleans(t *testing.T) {
	t.Parallel()
	placed := func(t *testing.T, source *decad.Body, x, y, z float64) *decad.Body {
		t.Helper()
		motion, err := r3.Translation(r3.NewVec(x, y, z))
		require.NoError(t, err)
		placedBody, err := decad.Copy(t.Context(), source, decad.WithCopyPlacement(motion))
		require.NoError(t, err)
		return placedBody
	}
	verify := func(t *testing.T, body *decad.Body, wantVolume float64) {
		t.Helper()
		require.False(t, tookMeshPath(body))
		volume, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, volume.Exactness)
		require.Equal(t, wantVolume, volumeMM(t, volume))
		require.Zero(t, boundMM3(t, volume))
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
		requireWatertight(t, mesh)
	}

	t.Run("union overlapping", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := placed(t, a, 0, 0, 5)
		got, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		verify(t, got, 1500)
	})
	t.Run("union touching caps", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := placed(t, a, 0, 0, 10)
		got, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		verify(t, got, 2000)
	})
	t.Run("intersection", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := placed(t, a, 0, 0, 5)
		got, err := decad.Intersect(t.Context(), a, b)
		require.NoError(t, err)
		verify(t, got, 500)
	})
	t.Run("cut through tool", func(t *testing.T) {
		doc := decad.New()
		target := boxBody(t, doc, 0, 0, 10, 10, 10)
		tool := boxBody(t, doc, 2, 2, 8, 8, 20)
		placedTool := placed(t, tool, 0, 0, -5)
		got, err := decad.Cut(t.Context(), target, placedTool)
		require.NoError(t, err)
		verify(t, got, 640)
	})
	t.Run("blind cut", func(t *testing.T) {
		doc := decad.New()
		target := boxBody(t, doc, 0, 0, 10, 10, 10)
		tool := boxBody(t, doc, 2, 2, 8, 8, 5)
		placedTool := placed(t, tool, 0, 0, 5)
		got, err := decad.Cut(t.Context(), target, placedTool)
		require.NoError(t, err)
		verify(t, got, 820)
	})
	t.Run("off-axis translation refuses", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := placed(t, a, 0.5, 0, 5)
		_, err := decad.Union(t.Context(), a, b)
		requireCoplanarFaceRefusal(t, err)
	})
	t.Run("rotated basis refuses", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		motion, err := r3.FromBasis(r3.Basis{
			EX: r3.NewVec(0, 0, -1),
			EY: r3.NewVec(0, 1, 0),
			EZ: r3.NewVec(1, 0, 0),
		}, r3.NewVec(10, 0, 10))
		require.NoError(t, err)
		b, err := decad.Copy(t.Context(), a, decad.WithCopyPlacement(motion))
		require.NoError(t, err)
		_, err = decad.Union(t.Context(), a, b)
		requireCoplanarFaceRefusal(t, err)
	})
}
