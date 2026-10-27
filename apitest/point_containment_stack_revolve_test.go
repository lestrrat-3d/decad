package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers LocatePoint's exact stacked, stack BRep and full Revolve
// routes (agent-docs/point-containment-design.md §9, §10) and their reading
// as Boolean operands (§8). Each expected answer is stated from the drawn
// geometry: points on faces, edges and levels, 1e-9 mm either side of them,
// and points on the revolve axis.

// pocketedCube is Example_cut_blind_pocket's part: a 10 mm cube with a
// 4 × 4 pocket over x, y ∈ [3, 7] from z = 6 to its top.
func pocketedCube(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocketed, err := decad.Cut(t.Context(), cube, tool)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(pocketed))
	return pocketed
}

// TestPointLocationExactStackedRoundPocket reads a 10 mm cube over
// [−5, 5]² with a radius-2.5 pocket about the z axis from z = 6 to its top: the
// pocket's floor, curved wall and rim, the interface level z = 6 under the
// material, and points 1e-9 mm either side of the floor and the wall. The
// mesh alone leaves the wall and every 1e-9 mm point undecided.
func TestPointLocationExactStackedRoundPocket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, -5, -5, 5, 5, 10)
	tool := circleBodyAtZ(t, doc, 0, 2.5, 6, 4)
	pocketed, err := decad.Cut(t.Context(), cube, tool)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(pocketed))
	requireLocations(t, pocketed, []locateCase{
		{name: "pocket floor", at: r3.NewVec(0, 0, 6), want: decad.PointOnBoundary},
		{name: "1e-9 under the floor", at: r3.NewVec(1, 1, 6-1e-9), want: decad.PointInside},
		{name: "1e-9 over the floor", at: r3.NewVec(1, 1, 6+1e-9), want: decad.PointOutside},
		{name: "interface level in the material", at: r3.NewVec(4, 0, 6), want: decad.PointInside},
		{name: "foot of the pocket wall", at: r3.NewVec(2.5, 0, 6), want: decad.PointOnBoundary},
		{name: "pocket wall", at: r3.NewVec(1.5, -2, 8), want: decad.PointOnBoundary},
		{name: "1e-9 into the wall", at: r3.NewVec(0, 2.5+1e-9, 8), want: decad.PointInside},
		{name: "1e-9 into the pocket", at: r3.NewVec(0, 2.5-1e-9, 8), want: decad.PointOutside},
		{name: "pocket mouth", at: r3.NewVec(0, 0, 10), want: decad.PointOutside},
		{name: "pocket rim", at: r3.NewVec(-2.5, 0, 10), want: decad.PointOnBoundary},
		{name: "top beside the pocket", at: r3.NewVec(4, 4, 10), want: decad.PointOnBoundary},
		{name: "outer wall", at: r3.NewVec(5, 1, 8), want: decad.PointOnBoundary},
		{name: "bottom face under the pocket", at: r3.NewVec(0, 0, 0), want: decad.PointOnBoundary},
		{name: "1e-9 under the bottom", at: r3.NewVec(0, 0, -1e-9), want: decad.PointOutside},
	})
}

// TestPointLocationExactStackedBoss reads a union-built stack: a 40 × 40 × 10
// plate under a radius-5 boss from z = 10 to 25 about the origin. At the
// interface the boss's footprint is material on both sides.
func TestPointLocationExactStackedBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := circleBodyAtZ(t, doc, 0, 5, 10, 15)
	got, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(got))
	requireLocations(t, got, []locateCase{
		{name: "interface under the boss", at: r3.NewVec(0, 0, 10), want: decad.PointInside},
		{name: "boss foot", at: r3.NewVec(3, 4, 10), want: decad.PointOnBoundary},
		{name: "plate top beside the boss", at: r3.NewVec(10, 0, 10), want: decad.PointOnBoundary},
		{name: "1e-9 over the plate top", at: r3.NewVec(10, 0, 10+1e-9), want: decad.PointOutside},
		{name: "boss wall", at: r3.NewVec(-3, 4, 20), want: decad.PointOnBoundary},
		{name: "1e-9 inside the boss wall", at: r3.NewVec(0, 5-1e-9, 20), want: decad.PointInside},
		{name: "1e-9 outside the boss wall", at: r3.NewVec(0, 5+1e-9, 20), want: decad.PointOutside},
		{name: "boss top", at: r3.NewVec(1, 1, 25), want: decad.PointOnBoundary},
	})
}

// surdSide is the exact sign of x² + y² − r², stated over rationals from
// the held coordinates, for a point whose radius about the z axis is
// irrational.
func surdSide(x, y, r float64) decad.PointLocation {
	rx, ry, rr := new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y), new(big.Rat).SetFloat64(r)
	lhs := new(big.Rat).Add(new(big.Rat).Mul(rx, rx), new(big.Rat).Mul(ry, ry))
	switch lhs.Cmp(new(big.Rat).Mul(rr, rr)) {
	case -1:
		return decad.PointInside
	case 1:
		return decad.PointOutside
	}
	return decad.PointOnBoundary
}

// TestPointLocationExactRevolveStepped reads the stepped shaft, r = 10 for
// z ∈ [0, 20] and r = 6 for z ∈ [20, 40] about world Z: its walls at several
// angles, the shoulder, the caps, and points on the axis.
func TestPointLocationExactRevolveStepped(t *testing.T) {
	t.Parallel()
	_, shaft := steppedShaft(t)
	require.Equal(t, "decad.revolvePayload", decadtest.PayloadClass(shaft))
	diagonal := 5 * math.Sqrt2
	requireLocations(t, shaft, []locateCase{
		{name: "wall", at: r3.NewVec(10, 0, 5), want: decad.PointOnBoundary},
		{name: "wall at 90°", at: r3.NewVec(0, 10, 5), want: decad.PointOnBoundary},
		{name: "wall at (6, 8)", at: r3.NewVec(-6, 8, 5), want: decad.PointOnBoundary},
		{name: "1e-9 inside the wall", at: r3.NewVec(10-1e-9, 0, 5), want: decad.PointInside},
		{name: "1e-9 outside the wall", at: r3.NewVec(10+1e-9, 0, 5), want: decad.PointOutside},
		{name: "irrational radius near the wall", at: r3.NewVec(diagonal, diagonal, 5), want: surdSide(diagonal, diagonal, 10)},
		{name: "(7, 7) under the shoulder", at: r3.NewVec(7, 7, 5), want: decad.PointInside},
		{name: "(7, 7) over the shoulder", at: r3.NewVec(7, 7, 25), want: decad.PointOutside},
		{name: "shoulder", at: r3.NewVec(0, -8, 20), want: decad.PointOnBoundary},
		{name: "1e-9 under the shoulder", at: r3.NewVec(0, -8, 20-1e-9), want: decad.PointInside},
		{name: "1e-9 over the shoulder", at: r3.NewVec(0, -8, 20+1e-9), want: decad.PointOutside},
		{name: "shoulder level inside the neck", at: r3.NewVec(3, 0, 20), want: decad.PointInside},
		{name: "top rim", at: r3.NewVec(6, 0, 40), want: decad.PointOnBoundary},
		{name: "axis", at: r3.NewVec(0, 0, 5), want: decad.PointInside},
		{name: "axis at the shoulder", at: r3.NewVec(0, 0, 20), want: decad.PointInside},
		{name: "bottom cap centre", at: r3.NewVec(0, 0, 0), want: decad.PointOnBoundary},
		{name: "top cap centre", at: r3.NewVec(0, 0, 40), want: decad.PointOnBoundary},
		{name: "axis 1e-9 into the bottom", at: r3.NewVec(0, 0, 1e-9), want: decad.PointInside},
		{name: "axis 1e-9 under the bottom", at: r3.NewVec(0, 0, -1e-9), want: decad.PointOutside},
	})
}

// TestPointLocationExactRevolveCapsule reads a cylinder r = 5 over
// z ∈ [0, 10] under a hemisphere of radius 5 about (0, 0, 10). The pole is
// an axis point on an arc, so it is on the boundary.
func TestPointLocationExactRevolveCapsule(t *testing.T) {
	t.Parallel()
	_, capsule := meridianCapsule(t)
	requireLocations(t, capsule, []locateCase{
		{name: "pole", at: r3.NewVec(0, 0, 15), want: decad.PointOnBoundary},
		{name: "1e-9 under the pole", at: r3.NewVec(0, 0, 15-1e-9), want: decad.PointInside},
		{name: "1e-9 over the pole", at: r3.NewVec(0, 0, 15+1e-9), want: decad.PointOutside},
		{name: "sphere", at: r3.NewVec(0, 3, 14), want: decad.PointOnBoundary},
		{name: "1e-9 inside the sphere", at: r3.NewVec(0, 3, 14-1e-9), want: decad.PointInside},
		{name: "1e-9 outside the sphere", at: r3.NewVec(0, 3, 14+1e-9), want: decad.PointOutside},
		{name: "equator", at: r3.NewVec(3, 4, 10), want: decad.PointOnBoundary},
		{name: "cylinder below the equator", at: r3.NewVec(0, 4.5, 7), want: decad.PointInside},
	})
}

// TestPointLocationExactRevolveTube reads a tube r ∈ [4, 10] over
// z ∈ [0, 20]: its meridian is clear of the axis, so an axis point is
// outside.
func TestPointLocationExactRevolveTube(t *testing.T) {
	t.Parallel()
	_, tube := meridianTube(t)
	requireLocations(t, tube, []locateCase{
		{name: "axis", at: r3.NewVec(0, 0, 10), want: decad.PointOutside},
		{name: "bore wall", at: r3.NewVec(0, -4, 10), want: decad.PointOnBoundary},
		{name: "1e-9 into the bore", at: r3.NewVec(0, -4+1e-9, 10), want: decad.PointOutside},
		{name: "1e-9 into the wall", at: r3.NewVec(0, -4-1e-9, 10), want: decad.PointInside},
		{name: "end face", at: r3.NewVec(7, 0, 20), want: decad.PointOnBoundary},
	})
}

// TestPointLocationExactRevolvePlaced translates the stepped shaft by
// (100, 0, 0) and turns a copy 45° about the x axis; both answer through the
// exact placement solve.
func TestPointLocationExactRevolvePlaced(t *testing.T) {
	t.Parallel()
	_, shaft := steppedShaft(t)
	moved := translated(t, shaft, 100, 0, 0)
	requireLocations(t, moved, []locateCase{
		{name: "wall", at: r3.NewVec(110, 0, 5), want: decad.PointOnBoundary},
		{name: "1e-9 outside the wall", at: r3.NewVec(110+1e-9, 0, 5), want: decad.PointOutside},
		{name: "bottom cap centre", at: r3.NewVec(100, 0, 0), want: decad.PointOnBoundary},
		{name: "axis", at: r3.NewVec(100, 0, 30), want: decad.PointInside},
	})
	_, second := steppedShaft(t)
	turned, err := decad.Copy(t.Context(), second,
		decad.WithCopyRotation(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), units.Degrees(45)))
	require.NoError(t, err)
	rotation, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	world := func(x, y, z float64) r3.Vec { return rotation.Apply(r3.NewVec(x, y, z)) }
	requireLocations(t, turned, []locateCase{
		{name: "0.001 inside the wall", at: world(9.999, 0, 5), want: decad.PointInside},
		{name: "0.001 outside the wall", at: world(10.001, 0, 5), want: decad.PointOutside},
		{name: "0.001 into the bottom on the axis", at: world(0, 0, 0.001), want: decad.PointInside},
	})
}

// TestPointLocationThroughHistoryRevolveLeaf unites the stepped shaft with
// a box crossing its wall, x ∈ [8, 20], y ∈ [−2, 2], z ∈ [2, 8]. The mesh
// result reads the shaft through §10 and the box through §7.
func TestPointLocationThroughHistoryRevolveLeaf(t *testing.T) {
	t.Parallel()
	doc, shaft := steppedShaft(t)
	box := boxBodyAtZ(t, doc, 8, -2, 20, 2, 2, 6)
	got, err := decad.Union(t.Context(), shaft, box)
	require.NoError(t, err)
	require.True(t, tookMeshPath(got))
	requireLocations(t, got, []locateCase{
		{name: "shaft wall beside the box", at: r3.NewVec(0, 10, 5), want: decad.PointOnBoundary},
		{name: "1e-9 inside the shaft wall", at: r3.NewVec(0, 10-1e-9, 5), want: decad.PointInside},
		{name: "1e-9 outside the shaft wall", at: r3.NewVec(0, 10+1e-9, 5), want: decad.PointOutside},
		{name: "shaft wall inside the box", at: r3.NewVec(10, 0, 5), want: decad.PointInside},
		{name: "box end face", at: r3.NewVec(20, 0, 5), want: decad.PointOnBoundary},
		{name: "bottom cap centre", at: r3.NewVec(0, 0, 0), want: decad.PointOnBoundary},
		{name: "axis", at: r3.NewVec(0, 0, 30), want: decad.PointInside},
	})
}

// TestPointLocationThroughHistoryStackedLeaf cuts the pocketed cube with a
// radius-1 pin through it at (8, 8), turned 45° about the z axis through
// that centre so the cut takes the mesh path. The pocket floor and walls
// read through §9.
func TestPointLocationThroughHistoryStackedLeaf(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pocketed := pocketedCube(t, doc)
	pin := circleBodyAtZ(t, doc, 8, 1, -5, 20)
	turned, err := decad.Copy(t.Context(), pin,
		decad.WithCopyRotation(r3.NewVec(8, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(45)))
	require.NoError(t, err)
	got, err := decad.Cut(t.Context(), pocketed, turned)
	require.NoError(t, err)
	require.True(t, tookMeshPath(got))
	requireLocations(t, got, []locateCase{
		{name: "pocket floor", at: r3.NewVec(5, 5, 6), want: decad.PointOnBoundary},
		{name: "1e-9 under the floor", at: r3.NewVec(5, 5, 6-1e-9), want: decad.PointInside},
		{name: "1e-9 over the floor", at: r3.NewVec(5, 5, 6+1e-9), want: decad.PointOutside},
		{name: "interface level in the material", at: r3.NewVec(1, 1, 6), want: decad.PointInside},
		{name: "pocket wall", at: r3.NewVec(3, 5, 8), want: decad.PointOnBoundary},
	})
}

// TestPointLocationThroughHistoryStackBrepLeaf cuts the flush corner boss of
// TestStackedUnionFlushCornerBoss, a BRep that keeps its two slabs (the plate
// over [−20, 20]² × [0, 10] and the boss over [10, 20] × [−20, −10] ×
// [10, 25]), with a radius-1 pin at (−10, 0) turned 45° about its own axis,
// so the cut takes the mesh path. The BRep reads through §9 as an operand.
func TestPointLocationThroughHistoryStackBrepLeaf(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 10)
	boss := boxBodyAtZ(t, doc, 10, -20, 20, -10, 10, 15)
	brep, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(brep))
	pin := circleBodyAtZ(t, doc, -10, 1, -5, 40)
	turned, err := decad.Copy(t.Context(), pin,
		decad.WithCopyRotation(r3.NewVec(-10, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(45)))
	require.NoError(t, err)
	got, err := decad.Cut(t.Context(), brep, turned)
	require.NoError(t, err)
	require.True(t, tookMeshPath(got))
	requireLocations(t, got, []locateCase{
		{name: "shared wall", at: r3.NewVec(20, -15, 15), want: decad.PointOnBoundary},
		{name: "shared wall at the interface", at: r3.NewVec(20, -15, 10), want: decad.PointOnBoundary},
		{name: "1e-9 inside the shared wall", at: r3.NewVec(20-1e-9, -15, 15), want: decad.PointInside},
		{name: "1e-9 outside the shared wall", at: r3.NewVec(20+1e-9, -15, 15), want: decad.PointOutside},
		{name: "interface under the boss", at: r3.NewVec(15, -15, 10), want: decad.PointInside},
		{name: "plate top", at: r3.NewVec(0, 0, 10), want: decad.PointOnBoundary},
		{name: "1e-9 under the plate top", at: r3.NewVec(0, 0, 10-1e-9), want: decad.PointInside},
		{name: "boss top", at: r3.NewVec(15, -15, 25), want: decad.PointOnBoundary},
		{name: "1e-9 over the boss top", at: r3.NewVec(15, -15, 25+1e-9), want: decad.PointOutside},
	})
}
