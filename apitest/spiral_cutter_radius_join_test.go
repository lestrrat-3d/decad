package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// TestSpiralToothWithAShorterCutterJoinsTheBlank joins the blank with spiral
// teeth cut on a cutter circle of 30 mm radius, not the mean cone distance. The
// seam proofs read their X and slope limits from the tooth's own root arc, so
// the first tooth and its rotated copy each add the same volume.
func TestSpiralToothWithAShorterCutterJoinsTheBlank(t *testing.T) {
	const cutter = 30.0
	angle := 2 * math.Pi * 23 / 31

	doc := decad.New()
	blank, tooth := spiralBlankAndToothCutter(t, doc, cutter)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	oneVolume, err := joined.Volume()
	require.NoError(t, err)
	added := oneVolume.Value.Base() - blankVolume.Value.Base()
	require.Greater(t, added, 10.0, "the tooth must add volume")

	doc2 := decad.New()
	blank2, tooth2 := spiralBlankAndToothCutter(t, doc2, cutter)
	copied, err := decad.Copy(t.Context(), tooth2, decad.WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Radians(angle)))
	require.NoError(t, err)
	both, err := decad.Union(t.Context(), blank2, tooth2, copied)
	require.NoError(t, err)
	require.True(t, both.IsSolid())
	twoVolume, err := both.Volume()
	require.NoError(t, err)

	slack := oneVolume.Bound.Base() + twoVolume.Bound.Base() + blankVolume.Bound.Base()
	require.LessOrEqual(t, math.Abs(twoVolume.Value.Base()-blankVolume.Value.Base()-2*added), slack+1e-3*added,
		"two teeth: %v, one tooth: %v, blank: %v", twoVolume.Value, oneVolume.Value, blankVolume.Value)

	mesh, err := both.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
