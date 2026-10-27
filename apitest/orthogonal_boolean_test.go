package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md's public-surface tests for the Booleans of two
// faceted solids every face of which squares to a world axis.

func blockIn(t *testing.T, doc *decad.Document, lo, hi r3.Vec) *decad.Body {
	t.Helper()
	v, tris := cubeMesh(lo, hi)
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	return body
}

func requireVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base()+1e-9, "volume %v, want %v", v.Value.Base(), want)
	require.LessOrEqual(t, v.Bound.Base(), 1e-9, "the grid result carries no mesh bound")
}

func TestOrthogonalUnionOfBlocksTouchingAtAFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	left := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	right := blockIn(t, doc, r3.NewVec(10, 0, 0), r3.NewVec(20, 10, 10))
	out, err := decad.Union(t.Context(), left, right)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationUnion, operations[len(operations)-1].Kind())
	requireVolume(t, out, 2000)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.True(t, box.Max.Equal(r3.NewVec(20, 10, 10), 1e-9))
	require.Len(t, out.Faces(), 6, "the joined faces merge into one bar's six")
	loc, err := out.LocatePoint(t.Context(), r3.NewVec(10, 5, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, loc, "the seam is inside")
}

func TestOrthogonalBooleansWithCoplanarFaces(t *testing.T) {
	t.Parallel()
	// A 10 cube and a 20 x 10 x 5 slab share the bottom face's plane and overlap in a 10 x 10 x 5 block.
	build := func(doc *decad.Document) (*decad.Body, *decad.Body) {
		return blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10)),
			blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 5))
	}
	doc := decad.New()
	a, b := build(doc)
	union, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	requireVolume(t, union, 1500)
	require.Len(t, union.Faces(), 8, "an L-shaped bar has eight faces")
	doc = decad.New()
	a, b = build(doc)
	common, err := decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	requireVolume(t, common, 500)
	doc = decad.New()
	a, b = build(doc)
	rest, err := decad.Cut(t.Context(), a, b)
	require.NoError(t, err)
	requireVolume(t, rest, 500)
	box, err := rest.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 5, box.Min.Z, 1e-9)
}

func TestOrthogonalCutOpensABlindPocketWithACoplanarFloorAndWalls(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(30, 30, 10))
	// The tool shares the plate's top plane and one side plane.
	tool := blockIn(t, doc, r3.NewVec(0, 10, 4), r3.NewVec(12, 20, 10))
	out, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	requireVolume(t, out, 9000-12*10*6)
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(6, 15, 7), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "the pocket is empty")
	floor, err := out.LocatePoint(t.Context(), r3.NewVec(6, 15, 2), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, floor)
}

func TestOrthogonalBooleanResultsChain(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	b := blockIn(t, doc, r3.NewVec(10, 0, 0), r3.NewVec(20, 10, 10))
	c := blockIn(t, doc, r3.NewVec(0, 10, 0), r3.NewVec(20, 20, 10))
	ab, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	abc, err := decad.Union(t.Context(), ab, c)
	require.NoError(t, err)
	requireVolume(t, abc, 4000)
	hole := blockIn(t, doc, r3.NewVec(5, 5, 0), r3.NewVec(15, 15, 10))
	out, err := decad.Cut(t.Context(), abc, hole)
	require.NoError(t, err)
	requireVolume(t, out, 4000-1000)
	require.Len(t, doc.Bodies(), 1)
}

func TestOrthogonalBooleanDisjointIntersectionKeepsItsError(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	b := blockIn(t, doc, r3.NewVec(30, 0, 0), r3.NewVec(40, 10, 10))
	_, err := decad.Intersect(t.Context(), a, b)
	require.ErrorIs(t, err, decad.ErrBooleanFailed)
	require.Len(t, doc.Bodies(), 2, "a refusal leaves the operands live")
}
