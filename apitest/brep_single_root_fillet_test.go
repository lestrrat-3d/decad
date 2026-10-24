package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func squareBossOnPlate(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 5)
	boss := boxBodyAtZ(t, doc, -5, -5, 5, 5, 5, 5)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return joined
}

// A single straight edge of a boss root is concave in the solid's transverse
// section, even though its two endpoint faces have convex corners.
func TestBrepSingleBossRootFilletAddsMaterial(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	before, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, 2500.0, before.Value.Base())

	root := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 5))).Exactly(1)
	got, err := decad.Fillet(t.Context(), joined, root, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	want := 2500 + 10*(1-math.Pi/4)
	require.InDelta(t, want, after.Value.Base(), after.Bound.Base()+1e-12)
	require.Positive(t, countRolePrefix(got, "filletLoop("))
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestBrepSingleBossTopEdgeFilletKeepsRouteE(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	top := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 10))).Exactly(1)
	got, err := decad.Fillet(t.Context(), joined, top, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	want := 2500 - 10*(1-math.Pi/4)
	require.InDelta(t, want, after.Value.Base(), after.Bound.Base()+1e-12)
	require.Equal(t, 1, countRolePrefix(got, "fillet("))
	require.Zero(t, countRolePrefix(got, "filletLoop("))
}

func TestBrepIndependentBossRootEdgesRefuseRouteEWedge(t *testing.T) {
	testIndependentBossRootRefusal(t, func(ctx context.Context, body *decad.Body, selector decad.EdgeSelector,
		amount units.Value) (*decad.Body, error) {
		return decad.Fillet(ctx, body, selector, amount)
	})
}
