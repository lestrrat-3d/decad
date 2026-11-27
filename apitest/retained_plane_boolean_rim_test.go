package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFilletRetainedRevolvePlaneWithCutRim keeps the caps of a turned cylinder,
// clips its sides to a square, and cuts a bore whose radius is not a recorded
// meridian endpoint. The corner fillets end on those retained caps.
func TestFilletRetainedRevolvePlaneWithCutRim(t *testing.T) {
	sc := newRetentionScene()
	host := revolvedCylinder(t, sc.doc, 0, 20, 0, 10)
	clip := placed(t, rectBox(t, sc.w, sc.doc, -5, -7, 25, 7, 14), -7)
	clipped, err := decad.Intersect(host, clip).Do(t.Context())
	require.NoError(t, err)
	bore := revolvedCylinder(t, sc.doc, -5, 25, 0, 2)
	bored, err := decad.Cut(clipped, bore).Do(t.Context())
	require.NoError(t, err)
	before, err := bored.Volume()
	require.NoError(t, err)
	selection := decad.Edges(decad.Convex(), decad.ParallelTo(r3.Vec{X: 1})).Exactly(4)
	result, err := decad.Fillet(bored, selection, units.Millimeters(.73)).Do(t.Context())
	require.NoError(t, err)
	require.True(t, result.IsSolid())
	after, err := result.Volume()
	require.NoError(t, err)
	requireVolumeChange(t, before, after, (4-math.Pi)*.73*.73*20)
	_, err = result.Tessellate(t.Context(), units.Millimeters(.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
}
