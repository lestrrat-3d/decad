package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers LocatePoint on mesh Boolean results through the operation
// history (agent-docs/point-containment-design.md §8). Every result here is
// a real mesh Boolean of prism operands. Each expected answer follows from
// the operands' closed-form geometry: which operand interiors and faces the
// point lies in, combined by the operation.

// turnAboutX turns points 45° about the x axis through (15, 15, 2), the
// plates' common centre line.
func turnAboutX(t *testing.T) r3.Transform {
	t.Helper()
	rotation, err := r3.RotationAround(r3.NewVec(15, 15, 2), r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	return rotation
}

// crossedPlateUnion unions a 30 × 30 × 4 plate with a copy of the plate
// x ∈ [5, 25] turned 45° about the x axis through (15, 15, 2). The turned
// plate is narrower than the flat one, so no face of one lies in a face of
// the other and the mesh Boolean needs no shifted retry.
func crossedPlateUnion(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := boxBody(t, doc, 0, 0, 30, 30, 4)
	narrow := boxBody(t, doc, 5, 0, 25, 30, 4)
	turned, err := decad.Copy(t.Context(), narrow,
		decad.WithCopyRotation(r3.NewVec(15, 15, 2), r3.NewVec(1, 0, 0), units.Degrees(45)))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, turned)
	require.NoError(t, err)
	require.True(t, tookMeshPath(joined))
	require.Empty(t, joined.OperationDiagnostics())
	return joined
}

// TestPointLocationHistoryCrossedPlates reads the crossed plates. A point
// 0.001 mm under the flat plate's top face is inside the flat plate, so it
// is inside the union whatever the turned plate holds there. An identity
// Copy of the result is a mesh body with no Boolean producer: it reads the
// verified mesh, which leaves the face and the points 1e-9 mm from it
// undecided.
func TestPointLocationHistoryCrossedPlates(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	joined := crossedPlateUnion(t, doc)
	requireLocations(t, joined, []locateCase{
		{name: "0.001 under the top face", at: r3.NewVec(1, 1, 3.999), want: decad.PointInside},
		// (1, 1) lies 14 mm from the turn axis in y, outside the turned plate.
		{name: "top face", at: r3.NewVec(1, 1, 4), want: decad.PointOnBoundary},
		{name: "0.001 over the top face", at: r3.NewVec(1, 1, 4.001), want: decad.PointOutside},
		{name: "1e-9 under the top face", at: r3.NewVec(1, 1, 4-1e-9), want: decad.PointInside},
		{name: "1e-9 over the top face", at: r3.NewVec(1, 1, 4+1e-9), want: decad.PointOutside},
		// On the turn axis the turned plate holds every |z − 2| < 2.
		{name: "top face inside the turned plate", at: r3.NewVec(10, 15, 4), want: decad.PointInside},
		// x = 5 is the turned plate's end face, so both operands read boundary.
		{name: "top face on the turned plate's end", at: r3.NewVec(5, 15, 4), want: decad.PointUndecided},
	})

	copied, err := decad.Copy(t.Context(), joined)
	require.NoError(t, err)
	requireLocations(t, copied, []locateCase{
		{name: "mesh: 1e-9 under the top face", at: r3.NewVec(1, 1, 4-1e-9), want: decad.PointUndecided},
		{name: "mesh: 1e-9 over the top face", at: r3.NewVec(1, 1, 4+1e-9), want: decad.PointUndecided},
		{name: "mesh: top face", at: r3.NewVec(1, 1, 4), want: decad.PointUndecided},
	})
}

// TestPointLocationHistoryNestedCut cuts a radius-2 pin through the crossed
// plates' centre, so the classification walks two levels of history. The pin
// wall point (17, 15, 1) is inside both plates.
func TestPointLocationHistoryNestedCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	joined := crossedPlateUnion(t, doc)
	pin := translated(t, diskBody(t, doc, 15, 15, 2), 0, 0, -10)
	part, err := decad.Cut(t.Context(), joined, pin)
	require.NoError(t, err)
	require.True(t, tookMeshPath(part))
	requireLocations(t, part, []locateCase{
		{name: "pin wall", at: r3.NewVec(17, 15, 1), want: decad.PointOnBoundary},
		{name: "1e-9 into the material", at: r3.NewVec(17+1e-9, 15, 1), want: decad.PointInside},
		{name: "1e-9 into the bore", at: r3.NewVec(17-1e-9, 15, 1), want: decad.PointOutside},
		{name: "0.001 under the top face", at: r3.NewVec(1, 1, 3.999), want: decad.PointInside},
	})
}

// pinPlate returns a 20 × 20 × 8 plate and a radius-2 pin about (10, 10)
// spanning z ∈ [−6, 14], which a translation places, so the pair takes the
// mesh path.
func pinPlate(t *testing.T) (*decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	return boxBody(t, doc, 0, 0, 20, 20, 8), translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
}

// TestPointLocationHistoryPinCut reads the plate less the pin. The bore's
// rim lies on both operands' boundaries, which the set algebra leaves open.
func TestPointLocationHistoryPinCut(t *testing.T) {
	t.Parallel()
	plate, pin := pinPlate(t)
	part, err := decad.Cut(t.Context(), plate, pin)
	require.NoError(t, err)
	require.True(t, tookMeshPath(part))
	requireLocations(t, part, []locateCase{
		{name: "bore wall", at: r3.NewVec(12, 10, 4), want: decad.PointOnBoundary},
		{name: "bore wall at −y", at: r3.NewVec(10, 8, 4), want: decad.PointOnBoundary},
		{name: "1e-9 into the material", at: r3.NewVec(12+1e-9, 10, 4), want: decad.PointInside},
		{name: "1e-9 into the bore", at: r3.NewVec(12-1e-9, 10, 4), want: decad.PointOutside},
		{name: "bore axis", at: r3.NewVec(10, 10, 4), want: decad.PointOutside},
		{name: "top face", at: r3.NewVec(3, 3, 8), want: decad.PointOnBoundary},
		{name: "1e-9 under the top face", at: r3.NewVec(3, 3, 8-1e-9), want: decad.PointInside},
		{name: "beside the plate", at: r3.NewVec(-1e-9, 3, 4), want: decad.PointOutside},
		{name: "bore rim", at: r3.NewVec(12, 10, 8), want: decad.PointUndecided},
	})
}

// TestPointLocationHistoryPinIntersect reads the plate met with the pin: the
// pin's length inside the plate.
func TestPointLocationHistoryPinIntersect(t *testing.T) {
	t.Parallel()
	plate, pin := pinPlate(t)
	common, err := decad.Intersect(t.Context(), plate, pin)
	require.NoError(t, err)
	require.True(t, tookMeshPath(common))
	requireLocations(t, common, []locateCase{
		{name: "pin wall", at: r3.NewVec(12, 10, 4), want: decad.PointOnBoundary},
		{name: "1e-9 inside the wall", at: r3.NewVec(12-1e-9, 10, 4), want: decad.PointInside},
		{name: "1e-9 outside the wall", at: r3.NewVec(12+1e-9, 10, 4), want: decad.PointOutside},
		{name: "top face inside the pin", at: r3.NewVec(10, 10, 8), want: decad.PointOnBoundary},
		{name: "1e-9 under the top face", at: r3.NewVec(10, 10, 8-1e-9), want: decad.PointInside},
		{name: "1e-9 over the top face", at: r3.NewVec(10, 10, 8+1e-9), want: decad.PointOutside},
		{name: "pin beyond the plate", at: r3.NewVec(10, 10, 10), want: decad.PointOutside},
	})
}

// TestPointLocationHistoryUndecidedOperand unites a box with a quarter pie
// whose arc end lies 500 ulps off its radius, which §7 does not read, so the
// pie's answer is undecided everywhere. Only the box decides the result.
func TestPointLocationHistoryUndecidedOperand(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pie := pieBody(t, doc, offRadiusEnd(20))
	box := translated(t, boxBody(t, doc, -5, -5, 5, 5, 4), 0, 0, 3)
	joined, err := decad.Union(t.Context(), pie, box)
	require.NoError(t, err)
	require.True(t, tookMeshPath(joined))
	requireLocations(t, joined, []locateCase{
		{name: "1e-9 inside the box face", at: r3.NewVec(-5+1e-9, -1, 5), want: decad.PointInside},
		{name: "box face beyond the pie", at: r3.NewVec(-5, -1, 5), want: decad.PointUndecided},
		{name: "1e-9 inside the arc beyond the box", at: r3.NewVec(12, 16-1e-9, 5), want: decad.PointUndecided},
	})
}

// TestPointLocationHistoryRefusals keeps results whose history is not a
// set-equality proof on the verified mesh. The full-width turned plate shares
// the flat plate's end faces, so the Union takes a gap-shifted retry: the
// result is not the union of its recorded inputs. A Transform's producer is
// not a Boolean. Both leave points on and near a flat face undecided.
func TestPointLocationHistoryRefusals(t *testing.T) {
	t.Parallel()
	t.Run("gap-shifted union", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 30, 30, 4)
		turned, err := decad.Copy(t.Context(), plate,
			decad.WithCopyRotation(r3.NewVec(15, 15, 2), r3.NewVec(1, 0, 0), units.Degrees(45)))
		require.NoError(t, err)
		joined, err := decad.Union(t.Context(), plate, turned)
		require.NoError(t, err)
		require.True(t, tookMeshPath(joined))
		diagnostics := joined.OperationDiagnostics()
		require.Len(t, diagnostics, 1)
		require.Equal(t, decad.OpDiagGapShiftApplied, diagnostics[0].Code)
		// This pins the conservative design choice, not the true answer: the
		// first point is inside the flat plate, which the diagnostic reports
		// unshifted, so the true answer is Inside however far the turned plate
		// moved. The history route still
		// refuses every gap-shifted result rather than prove which operand
		// moved and by how much (agent-docs/point-containment-design.md §8.1).
		requireLocations(t, joined, []locateCase{
			{name: "0.001 under the top face", at: r3.NewVec(1, 1, 3.999), want: decad.PointUndecided},
			{name: "top face", at: r3.NewVec(1, 1, 4), want: decad.PointUndecided},
		})
	})
	t.Run("transformed union", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rotation := turnAboutX(t)
		placed, err := decad.Transform(t.Context(), crossedPlateUnion(t, doc), rotation)
		require.NoError(t, err)
		require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(placed))
		requireLocations(t, placed, []locateCase{
			{name: "1e-9 under the top face", at: rotation.Apply(r3.NewVec(1, 1, 4-1e-9)), want: decad.PointUndecided},
			{name: "top face", at: rotation.Apply(r3.NewVec(1, 1, 4)), want: decad.PointUndecided},
		})
	})
}
