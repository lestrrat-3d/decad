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

func TestBrepAdjacentBossRootChamferAddsCornerMaterial(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	beforeMesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)

	corner := r3.NewVec(5, -5, 5)
	roots := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(corner)).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(corner)).Exactly(2)
	got, err := decad.Chamfer(t.Context(), joined, roots, units.Millimeters(1))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Equal(t, 2, countRolePrefix(got, "chamferLoop("))

	// At height t in [0,1], the two 10 mm strips each have width 1-t;
	// their outside corner adds a square of area (1-t)^2.
	wantVolume := 2500 + 10 + 1.0/3
	volume, err := got.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base()+1e-12)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, 10+1.0/3, meshVolume(mesh)-meshVolume(beforeMesh), 1e-8)

	cornerFill := r3.NewVec(5.25, -5.25, 5.25)
	before, err := joined.LocatePoint(t.Context(), cornerFill, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, before)
	after, err := got.LocatePoint(t.Context(), cornerFill, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, after)
}

func TestBrepAdjacentUnderBossRootChamferAddsCornerMaterial(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 5)
	boss := boxBodyAtZ(t, doc, -5, -5, 5, 5, -5, 5)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	corner := r3.NewVec(5, -5, 0)
	roots := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(corner)).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(corner)).Exactly(2)
	got, err := decad.Chamfer(t.Context(), joined, roots, units.Millimeters(1))
	require.NoError(t, err)
	wantVolume := 2500 + 10 + 1.0/3
	volume, err := got.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base()+1e-12)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	cornerFill := r3.NewVec(5.25, -5.25, -0.25)
	before, err := joined.LocatePoint(t.Context(), cornerFill, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, before)
	after, err := got.LocatePoint(t.Context(), cornerFill, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, after)
}

func TestBrepSingleBossRootChamferAddsMaterial(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	before, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, 2500.0, before.Value.Base())

	root := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 5))).Exactly(1)
	got, err := decad.Chamfer(t.Context(), joined, root, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 2505.0, after.Value.Base())
	require.Zero(t, after.Bound.Base())
	require.Positive(t, countRolePrefix(got, "chamferLoop("))

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, 2505.0, meshVolume(mesh), 1e-8)

	added := r3.NewVec(0, -5.25, 5.25)
	previous, err := joined.LocatePoint(t.Context(), added, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, previous)
	now, err := got.LocatePoint(t.Context(), added, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, now)
}

func TestBrepSingleBossPerpendicularRootChamferAddsMaterial(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	root := decad.Edges(decad.ParallelTo(r3.NewVec(0, 1, 0)),
		decad.EndpointAt(r3.NewVec(5, -5, 5))).Exactly(1)
	got, err := decad.Chamfer(t.Context(), joined, root, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 2505.0, after.Value.Base())
	require.Zero(t, after.Bound.Base())
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestBrepSingleUnderBossRootChamferAddsMaterial(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 5)
	boss := boxBodyAtZ(t, doc, -5, -5, 5, 5, -5, 5)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	root := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 0))).Exactly(1)
	got, err := decad.Chamfer(t.Context(), joined, root, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 2505.0, after.Value.Base())
	require.Zero(t, after.Bound.Base())
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, 2505.0, meshVolume(mesh), 1e-8)
}

func TestBrepSingleBossTopEdgeChamferKeepsRouteE(t *testing.T) {
	t.Parallel()
	joined := squareBossOnPlate(t)
	top := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 10))).Exactly(1)
	got, err := decad.Chamfer(t.Context(), joined, top, units.Millimeters(1))
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 2495.0, after.Value.Base())
	require.Zero(t, after.Bound.Base())
	require.Equal(t, 1, countRolePrefix(got, "chamfer("))
	require.Zero(t, countRolePrefix(got, "chamferLoop("))
}

func TestBrepIndependentBossRootEdgesRefuseChamferRouteE(t *testing.T) {
	testIndependentBossRootRefusal(t, func(ctx context.Context, body *decad.Body, selector decad.EdgeSelector,
		amount units.Value) (*decad.Body, error) {
		return decad.Chamfer(ctx, body, selector, amount)
	})
}
