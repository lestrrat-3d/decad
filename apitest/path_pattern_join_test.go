package apitest_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/path-pattern-design.md §7's tests: a 4 x 4 x 2 peg at the
// origin, copied along a path and joined by one Union to a 30 x 4 x 1 bar under it.

func joinFixture(t *testing.T) (*decad.Document, *decad.Body, *decad.Body, *decad.Path) {
	t.Helper()
	doc, peg := patternBlock(t)
	bar := decadtest.NewBlock(t, doc, 0, 0, 30, 4, units.Millimeters(1))
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 20}})
	require.NoError(t, err)
	return doc, bar, peg, path
}

func TestPathPatternUnionUnitesTheInstancesWithTheTarget(t *testing.T) {
	t.Parallel()
	doc, bar, peg, path := joinFixture(t)
	copies, err := decad.PathPattern(t.Context(), peg, path, decad.WithPathCount(3))
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), bar, append([]*decad.Body{peg}, copies...)...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the target, the feature and the copies are all retired")
	require.True(t, doc.Bodies()[0] == out)
	// The pegs stand at x 0, 10 and 20: the bar is 120 and each peg adds the 16 above it.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(120+3*16))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 30, Y: 4, Z: 2})
}

func TestPathPatternSpacingUnionPlacesTheCopiesByDistance(t *testing.T) {
	t.Parallel()
	doc, bar, peg, path := joinFixture(t)
	copies, err := decad.PathPattern(t.Context(), peg, path, decad.WithPathSpacing(units.Millimeters(10)))
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), bar, append([]*decad.Body{peg}, copies...)...)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(120+3*16))
}

func TestPathPatternUnionRefusalLeavesEveryOperandLive(t *testing.T) {
	t.Parallel()
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 20}})
	require.NoError(t, err)
	// A sheet is no solid to join to: Union refuses it and leaves the sheet, the
	// feature and every copy live.
	sheetDoc, sheet := offsetPrismSheet(t)
	feature := decadtest.NewBlock(t, sheetDoc, 0, 0, 4, 4, units.Millimeters(2))
	copies, err := decad.PathPattern(t.Context(), feature, path, decad.WithPathCount(3))
	require.NoError(t, err)
	_, err = decad.Union(t.Context(), sheet, append([]*decad.Body{feature}, copies...)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, slices.Equal(append([]*decad.Body{sheet, feature}, copies...), sheetDoc.Bodies()))
	for _, c := range copies {
		require.NoError(t, decad.Remove(c))
	}
	require.Len(t, sheetDoc.Bodies(), 2)
}
