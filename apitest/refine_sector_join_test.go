package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// refineSectorHubAndLugs joins a revolved hub and count copies of one wide
// point-Loft lug (six in all), trimmed by toe and heel cones. The wide lug makes the cone
// cap interpolation the largest term of the join's bound, so the join holds
// a boundary bound above 0.05 mm.
func refineSectorHubAndLugs(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	const count = 6
	doc, hub, lug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.8, false)
	lugs := []*decad.Body{lug}
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/float64(count)))
		require.NoError(t, err)
		copied, err := decad.Copy(lug).Place(decad.CopyTransform{Transform: turn}).Do(t.Context())
		require.NoError(t, err)
		lugs = append(lugs, copied)
	}
	joined, err := decad.Union(append([]*decad.Body{hub}, lugs...)...).Do(t.Context())
	require.NoError(t, err)
	return doc, joined
}

// refineSectorFloor reads the held boundary bound a body publishes.
func refineSectorFloor(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	_, err := body.Tessellate(t.Context(), units.Millimeters(0.0001))
	var bound *decad.MeshBoundError
	require.ErrorAs(t, err, &bound)
	return bound.Floor().Base()
}

// TestRefineSectorJoinReachesTargetBound rebuilds a straight-bevel style
// sector join with finer cone caps. The default join holds more than 0.05 mm
// and cannot be tessellated at 0.05 mm. After Refine it can, at each target,
// and the volume agrees with the source's within both bounds.
func TestRefineSectorJoinReachesTargetBound(t *testing.T) {
	for _, target := range []float64{0.05, 0.03, 0.02} {
		doc, joined := refineSectorHubAndLugs(t)
		held := refineSectorFloor(t, joined)
		require.Greater(t, held, 0.05)
		_, err := joined.Tessellate(t.Context(), units.Millimeters(0.05))
		require.Error(t, err)
		before, err := joined.Volume()
		require.NoError(t, err)

		refined, err := decad.Refine(joined, units.Millimeters(target)).Do(t.Context())
		require.NoError(t, err, "target %g", target)
		require.NotContains(t, doc.Bodies(), joined)
		require.Contains(t, doc.Bodies(), refined)
		floor := refineSectorFloor(t, refined)
		require.LessOrEqual(t, floor, target)
		mesh, err := refined.Tessellate(t.Context(), units.Millimeters(target),
			decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
		require.Equal(t, 2, revolveLugEuler(mesh))
		after, err := refined.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(after.Value.Base()-before.Value.Base()),
			after.Bound.Base()+before.Bound.Base())
	}
}

// TestRefineSectorJoinKeepsLaterOperations checks that a refined join still
// takes the coaxial bore Cut at the refined bound, and that a serial Union of
// one more lug keeps the refined bound instead of falling back to the default.
func TestRefineSectorJoinKeepsLaterOperations(t *testing.T) {
	doc, joined := refineSectorHubAndLugs(t)
	refined, err := decad.Refine(joined, units.Millimeters(0.03)).Do(t.Context())
	require.NoError(t, err)
	tool := revolveSectorCylinderTool(t, doc, 1, 4, 6, 0)
	cut, err := decad.Cut(refined, tool).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, cut), 0.05)
	_, err = cut.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)

	// Four lugs of a six-lug ring, refined, then joined one at a time with
	// the remaining copies: the serial union keeps the refined ceiling.
	doc2, hub, lug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.8, false)
	copies := make([]*decad.Body, 0, 5)
	for i := 1; i < 6; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/6))
		require.NoError(t, err)
		copied, err := decad.Copy(lug).Place(decad.CopyTransform{Transform: turn}).Do(t.Context())
		require.NoError(t, err)
		copies = append(copies, copied)
	}
	four, err := decad.Union(hub, lug, copies[0], copies[1], copies[2]).Do(t.Context())
	require.NoError(t, err)
	held := refineSectorFloor(t, four)
	require.Greater(t, held, 0.03)
	refinedFour, err := decad.Refine(four, units.Millimeters(0.03)).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, refinedFour), 0.03)
	five, err := decad.Union(refinedFour, copies[3]).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, five), 0.03)
	require.Contains(t, doc2.Bodies(), five)
}

// TestRefineSectorJoinRefusals checks that a target the join cannot meet is
// ErrUnsupported naming the bound, and leaves the source live and unchanged.
func TestRefineSectorJoinRefusals(t *testing.T) {
	t.Parallel()
	doc, joined := refineSectorHubAndLugs(t)
	held := refineSectorFloor(t, joined)
	bodies := doc.Bodies()
	refined, err := decad.Refine(joined, units.Millimeters(0.001)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "mm boundary proof")
	require.True(t, refined == nil)
	require.Equal(t, bodies, doc.Bodies())
	require.Equal(t, held, refineSectorFloor(t, joined))
}

// TestRefineSectorJoinNeverLoosens checks that a target above the held bound
// leaves a join at least as fine as the source.
func TestRefineSectorJoinNeverLoosens(t *testing.T) {
	_, joined := refineSectorHubAndLugs(t)
	held := refineSectorFloor(t, joined)
	before, err := joined.Volume()
	require.NoError(t, err)
	refined, err := decad.Refine(joined, units.Millimeters(0.09)).Do(t.Context())
	require.NoError(t, err)
	require.LessOrEqual(t, refineSectorFloor(t, refined), held)
	after, err := refined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(after.Value.Base()-before.Value.Base()),
		after.Bound.Base()+before.Bound.Base())
}
