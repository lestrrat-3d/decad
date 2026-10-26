package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/path-pattern-design.md §7's public-surface tests for
// PathPatternJoin and PathPatternJoinSpaced: a 4 x 4 x 2 peg at the origin
// joined, along a path, to a 30 x 4 x 1 bar under it.

func joinFixture(t *testing.T) (*decad.Document, *decad.Body, *decad.Body, *decad.Path) {
	t.Helper()
	doc, peg := patternBlock(t)
	bar := decadtest.NewBlock(t, doc, 0, 0, 30, 4, units.Millimeters(1))
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 20}})
	require.NoError(t, err)
	return doc, bar, peg, path
}

func TestPathPatternJoinUnitesTheInstancesWithTheTarget(t *testing.T) {
	t.Parallel()
	doc, bar, peg, path := joinFixture(t)
	out, err := decad.PathPatternJoin(t.Context(), bar, peg, path, 3)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the target, the feature and the copies are all retired")
	require.True(t, doc.Bodies()[0] == out)
	// The pegs stand at x 0, 10 and 20: the bar is 120 and each peg adds the 16 above it.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(120+3*16))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 30, Y: 4, Z: 2})
}

func TestPathPatternJoinSpacedPlacesTheCopiesByDistance(t *testing.T) {
	t.Parallel()
	doc, bar, peg, path := joinFixture(t)
	out, err := decad.PathPatternJoinSpaced(t.Context(), bar, peg, path, units.Millimeters(10))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(120+3*16))
}

func TestPathPatternJoinRefusalsLeaveBothBodiesLive(t *testing.T) {
	t.Parallel()
	doc, bar, peg, path := joinFixture(t)
	_, err := decad.PathPatternJoin(t.Context(), bar, bar, path, 3)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a target is not its own feature")
	_, err = decad.PathPatternJoin(t.Context(), nil, peg, path, 3)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.PathPatternJoin(t.Context(), bar, peg, path, 1)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a count of one")
	_, err = decad.PathPatternJoinSpaced(t.Context(), bar, peg, path, units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	// A sheet is no solid to join to: Union refuses it after the copies exist, and the document
	// ends with the target and the feature only, no stray copy.
	sheetDoc, sheet := offsetPrismSheet(t)
	feature := decadtest.NewBlock(t, sheetDoc, 0, 0, 4, 4, units.Millimeters(2))
	_, err = decad.PathPatternJoin(t.Context(), sheet, feature, path, 3)
	require.Error(t, err)
	require.Len(t, sheetDoc.Bodies(), 2, "a refused join leaves the sheet and the feature, and no copy")
	require.Len(t, doc.Bodies(), 2, "the first document is as it was")
}
