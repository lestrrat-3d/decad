package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFacetedPrismUnionUntouchedOuterEdgeFillet(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 6)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(lug, turn).Do(t.Context())
	require.NoError(t, err)
	joined, err := decad.Union(base, lug).Do(t.Context())
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 10}),
	).Exactly(1)
	filleted, err := decad.Fillet(joined, selector, units.Millimeters(1)).Do(t.Context())
	require.NoError(t, err)
	mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	before, err := joined.Volume()
	require.NoError(t, err)
	after, err := filleted.Volume()
	require.NoError(t, err)
	want := 10 * (1 - math.Pi/4)
	delta := before.Value.Mag() - after.Value.Mag()
	bound := before.Bound.Mag() + after.Bound.Mag()
	require.LessOrEqual(t, math.Abs(delta-want), bound)
}

func TestFacetedPrismUnionFilletReversedSources(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -7, -5, 11, 9, 1, 8)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 6)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(lug, turn).Do(t.Context())
	require.NoError(t, err)
	joined, err := decad.Union(lug, base).Do(t.Context())
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -7, Y: 9, Z: 1}),
		decad.EndpointAt(r3.Vec{X: -7, Y: 9, Z: 9}),
	).Exactly(1)
	filleted, err := decad.Fillet(joined, selector, units.Millimeters(1)).Do(t.Context())
	require.NoError(t, err)
	mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestFacetedPrismUnionFilletRefusesToolInCornerBand(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, -9.8, -9.8, -8.5, -8.5, 2, 6)
	turn, err := r3.RotationAround(r3.Vec{X: -9, Y: -9}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	tool, err = decad.Transform(tool, turn).Do(t.Context())
	require.NoError(t, err)
	joined, err := decad.Union(base, tool).Do(t.Context())
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 10}),
	).Exactly(1)
	_, err = decad.Fillet(joined, selector, units.Millimeters(1)).Do(t.Context())
	require.True(t, errors.Is(err, decad.ErrUnsupported), "fillet returned %v", err)
}

func TestFacetedPrismUnionFilletOfPlacedUnionTakesTheRetainedRoute(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 6)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(lug, turn).Do(t.Context())
	require.NoError(t, err)
	joined, err := decad.Union(base, lug).Do(t.Context())
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	joined, err = decad.Transform(joined, move).Do(t.Context())
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 1}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 11}),
	).Exactly(1)
	before, err := joined.Volume()
	require.NoError(t, err)
	// The placed Union keeps no source relation, so the prism route declines;
	// the edge is a straight edge between two retained planes, which
	// agent-docs/modify-design.md Part E blends from the tags.
	filleted, err := decad.Fillet(joined, selector, units.Millimeters(1)).Do(t.Context())
	require.NoError(t, err)
	after, err := filleted.Volume()
	require.NoError(t, err)
	removed := before.Value.Mag() - after.Value.Mag()
	growth := after.Bound.Mag() - before.Bound.Mag()
	require.InDelta(t, 10*(1-math.Pi/4), removed, math.Max(growth, 1e-6))
}
