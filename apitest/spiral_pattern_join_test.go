package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// spiralPatternVolume is the volume in mm^3 of the blank joined with the
// first tooth and one copy at tooth 23 of 31, in 64 faces.
const (
	spiralPatternVolume = 3412.2099
	spiralPatternFaces  = 64
)

// TestSpiralCopiedToothUnionNeedsExactRotation joins the blank, the first tooth
// and a copy at 2*pi*23/31. A copy placed by a float matrix denotes a slightly
// scaled solid, so the patterned join refuses it at the entrance. A copy made
// with WithCopyRotation denotes an exact rotation about X, which leaves the
// blank invariant, and the join succeeds. One fixture build serves both steps.
func TestSpiralCopiedToothUnionNeedsExactRotation(t *testing.T) {
	doc := decad.New()
	blank, tooth := spiralBlankAndTooth(t, doc)
	angle := 2 * math.Pi * 23 / 31

	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(angle))
	require.NoError(t, err)
	bad, err := decad.Copy(t.Context(), tooth, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	_, err = decad.Union(t.Context(), blank, tooth, bad)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "carries no exact X-axis rotation")
	require.Len(t, doc.Bodies(), 3, "a refused Union leaves every input live")
	for _, body := range doc.Bodies() {
		_, err := body.Volume()
		require.NoError(t, err)
	}

	good, err := decad.Copy(t.Context(), tooth, decad.WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Radians(angle)))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), blank, tooth, good)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	require.Len(t, joined.Faces(), spiralPatternFaces)

	// A lost tooth is 17.5 mm^3 away, so the first-join test's 5e-4 holds it.
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.InEpsilon(t, spiralPatternVolume, volume.Value.Base(), 5e-4)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
