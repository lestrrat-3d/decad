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

func TestFacetedPrismUnionUntouchedOuterEdgeChamfer(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 4)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(t.Context(), lug, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), base, lug)
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 10}),
	).Exactly(1)
	chamfered, err := decad.Chamfer(t.Context(), joined, selector, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{chamfered}, doc.Bodies())
	require.Equal(t, 1, countRolePrefix(chamfered, "chamfer("))
	before, err := joined.Volume()
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)
	removed := before.Value.Mag() - after.Value.Mag()
	bound := before.Bound.Mag() + after.Bound.Mag()
	require.LessOrEqual(t, math.Abs(removed-5), bound)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestFacetedPrismUnionChamferReversedSources(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 4)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(t.Context(), lug, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), lug, base)
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: 10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: 10, Z: 10}),
	).Exactly(1)
	chamfered, err := decad.Chamfer(t.Context(), joined, selector, units.Millimeters(1))
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(before.Value.Mag()-after.Value.Mag()-5),
		before.Bound.Mag()+after.Bound.Mag())
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestFacetedPrismUnionChamferRefusesToolInCornerBand(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, -9.8, -9.8, -8.5, -8.5, 2, 4)
	turn, err := r3.RotationAround(r3.Vec{X: -9, Y: -9}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), base, tool)
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 10}),
	).Exactly(1)
	chamfered, err := decad.Chamfer(t.Context(), joined, selector, units.Millimeters(1))
	require.True(t, errors.Is(err, decad.ErrUnsupported), "chamfer returned %v", err)
	require.Nil(t, chamfered)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
}

func TestFacetedPrismUnionChamferOfPlacedUnionTakesTheRetainedRoute(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 4)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(t.Context(), lug, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), base, lug)
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), joined, move)
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 1}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 11}),
	).Exactly(1)
	before, err := placed.Volume()
	require.NoError(t, err)
	// The placed Union keeps no source relation, so the prism route declines;
	// the edge is a straight edge between two retained planes, which
	// agent-docs/retained-edge-blend-design.md blends from the tags.
	chamfered, err := decad.Chamfer(t.Context(), placed, selector, units.Millimeters(1))
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)
	removed := before.Value.Mag() - after.Value.Mag()
	growth := after.Bound.Mag() - before.Bound.Mag()
	require.InDelta(t, 5, removed, math.Max(growth, 1e-6), "a 1 mm bevel along a 10 mm edge removes 5 mm³")
	require.Equal(t, []*decad.Body{chamfered}, doc.Bodies())
}

func TestFacetedPrismUnionChamferRefusesInexactFoot(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	lug := boxBodyAtZ(t, doc, 8, -2, 14, 2, 2, 4)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	lug, err = decad.Transform(t.Context(), lug, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), base, lug)
	require.NoError(t, err)
	selector := decad.Edges(
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 0}),
		decad.EndpointAt(r3.Vec{X: -10, Y: -10, Z: 10}),
	).Exactly(1)
	chamfered, err := decad.Chamfer(t.Context(), joined, selector, units.Millimeters(0.3))
	require.True(t, errors.Is(err, decad.ErrUnsupported), "chamfer returned %v", err)
	require.Nil(t, chamfered)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
}
