package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-offset-design.md's public-surface tests for Offset of a solid.

func TestOffsetGrowsAndShrinksABlock(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	grown, err := decad.Offset(t.Context(), block, units.Millimeters(2))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2, "the source stays live beside its offset")
	requireVolume(t, grown, 14*12*10)
	box, err := grown.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(r3.NewVec(-2, -2, -2), 1e-12), "%v", box.Min)
	require.True(t, box.Max.Equal(r3.NewVec(12, 10, 8), 1e-12), "%v", box.Max)
	require.Len(t, grown.Faces(), 6)
	shrunk, err := decad.Offset(t.Context(), block, units.Millimeters(1), decad.WithOffsetSide(decad.OffsetNegative))
	require.NoError(t, err)
	requireVolume(t, shrunk, 8*6*4)
	box, err = shrunk.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(r3.NewVec(1, 1, 1), 1e-12))
	require.True(t, box.Max.Equal(r3.NewVec(9, 7, 5), 1e-12))
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationOffset, operations[len(operations)-1].Kind())
}

func TestOffsetMitresAnLShapedSolidAtItsCorners(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(4, 10, 4))
	l, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	// The L's plan is a 10 x 4 bar and a 4 x 10 bar. Grown by 1 mm each way, 12 x 6 and 6 x 12 overlapping in a
	// 6 x 6 square: 108 mm² over a height of 6; the reflex corner stays sharp, at (5, 5).
	grown, err := decad.Offset(t.Context(), l, units.Millimeters(1))
	require.NoError(t, err)
	requireVolume(t, grown, 108*6)
	reflex, err := grown.LocatePoint(t.Context(), r3.NewVec(5.5, 5.5, 3), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, reflex, "past the sharp reflex corner")
	inside, err := grown.LocatePoint(t.Context(), r3.NewVec(4.5, 4.5, 3), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside, "the grown corner fills")
	// Shrunk by 1 mm: legs of 2 mm, 1..9 by 1..3 and 1..3 by 1..9 over 1..3 in z: 28 mm² for 2 mm.
	shrunk, err := decad.Offset(t.Context(), l, units.Millimeters(1), decad.WithOffsetSide(decad.OffsetNegative))
	require.NoError(t, err)
	requireVolume(t, shrunk, 28*2)
}

func TestOffsetClosesANarrowSlotAndDropsAThinPart(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 4))
	slot := blockIn(t, doc, r3.NewVec(9, -1, -1), r3.NewVec(10, 11, 5))
	slotted, err := decad.Cut(t.Context(), plate, slot)
	require.NoError(t, err)
	// The two halves grown by 1 mm each way meet and overlap across the 1 mm slot: one 22 x 12 x 6 block.
	grown, err := decad.Offset(t.Context(), slotted, units.Millimeters(1))
	require.NoError(t, err)
	requireVolume(t, grown, 22*12*6)
	require.Len(t, grown.Faces(), 6, "the slot is gone")
	// A 2 mm sheet shrunk by 1 mm is nothing; by 0.5 mm it is a 1 mm sheet.
	sheet := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 2))
	_, err = decad.Offset(t.Context(), sheet, units.Millimeters(1), decad.WithOffsetSide(decad.OffsetNegative))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	half, err := decad.Offset(t.Context(), sheet, units.Millimeters(0.5), decad.WithOffsetSide(decad.OffsetNegative))
	require.NoError(t, err)
	requireVolume(t, half, 9*9*1)
}

func TestOffsetOfASolidRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	prism := boxBody(t, doc, 0, 0, 10, 8, 6)
	_, err := decad.Offset(t.Context(), prism, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported, "a solid that is not faceted")
	block := blockIn(t, doc, r3.NewVec(20, 0, 0), r3.NewVec(30, 8, 6))
	_, err = decad.Offset(t.Context(), block, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Offset(t.Context(), block, units.Degrees(3))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = decad.Offset(t.Context(), block, units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	require.Len(t, doc.Bodies(), 2, "every refusal leaves the document alone")
}
