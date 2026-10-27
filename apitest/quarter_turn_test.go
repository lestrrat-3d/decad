package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B10's public-surface tests for QuarterTurn.

func TestQuarterTurnExchangesCoordinatesExactly(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	for name, tc := range map[string]struct {
		axis, about r3.Vec
		turns       int
		min, max    r3.Vec
	}{
		// The block 0..10 by 0..8 by 0..6. About +Z through the origin a quarter turn sends (x, y) to (-y, x).
		"z once":        {z, r3.NewVec(0, 0, 0), 1, r3.NewVec(-8, 0, 0), r3.NewVec(0, 10, 6)},
		"z twice":       {z, r3.NewVec(0, 0, 0), 2, r3.NewVec(-10, -8, 0), r3.NewVec(0, 0, 6)},
		"z backwards":   {z, r3.NewVec(0, 0, 0), -1, r3.NewVec(0, -10, 0), r3.NewVec(8, 0, 6)},
		"-z once":       {z.Scale(-1), r3.NewVec(0, 0, 0), 1, r3.NewVec(0, -10, 0), r3.NewVec(8, 0, 6)},
		"x once":        {x, r3.NewVec(0, 0, 0), 1, r3.NewVec(0, -6, 0), r3.NewVec(10, 0, 8)},
		"y once":        {y, r3.NewVec(0, 0, 0), 1, r3.NewVec(0, 0, -10), r3.NewVec(6, 8, 0)},
		"five is one":   {z, r3.NewVec(0, 0, 0), 5, r3.NewVec(-8, 0, 0), r3.NewVec(0, 10, 6)},
		"about a point": {z, r3.NewVec(5, 4, 0), 1, r3.NewVec(1, -1, 0), r3.NewVec(9, 9, 6)},
	} {
		doc := decad.New()
		block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
		out, err := decad.QuarterTurn(t.Context(), block, tc.axis, tc.about, tc.turns)
		require.NoError(t, err, name)
		require.Equal(t, []*decad.Body{out}, doc.Bodies(), name)
		requireVolume(t, out, 480)
		box, err := out.Bounds()
		require.NoError(t, err)
		require.True(t, box.Min.Equal(tc.min, 1e-12), "%s min %v", name, box.Min)
		require.True(t, box.Max.Equal(tc.max, 1e-12), "%s max %v", name, box.Max)
		operations := doc.OperationHistory().Operations()
		require.Equal(t, decad.OperationQuarterTurn, operations[len(operations)-1].Kind())
	}
}

func TestQuarterTurnedSolidsJoinExactly(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	bar := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	other := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 4, 4))
	// A copy turned a quarter about +Z through (0, 0, 0) lies along -Y from x in -4..0; moved to meet the bar.
	turned, err := decad.QuarterTurn(t.Context(), other, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 0), 1)
	require.NoError(t, err)
	require.Equal(t, 2, len(doc.Bodies()))
	// It now spans x -4..0 and y 0..10 against the bar's x 0..10, y 0..4: they touch along x = 0 over y 0..4.
	joined, err := decad.Union(t.Context(), bar, turned)
	require.NoError(t, err)
	requireVolume(t, joined, 160+160)
}

func TestQuarterTurnRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	z := r3.NewVec(0, 0, 1)
	_, err := decad.QuarterTurn(t.Context(), block, z, r3.NewVec(0, 0, 0), 4)
	require.ErrorIs(t, err, decad.ErrDegenerate, "whole turns change nothing")
	_, err = decad.QuarterTurn(t.Context(), block, r3.NewVec(1, 1, 0), r3.NewVec(0, 0, 0), 1)
	require.ErrorIs(t, err, decad.ErrUnsupported, "an axis that is not a world axis")
	_, err = decad.QuarterTurn(t.Context(), block, z, r3.NewVec(1.0/3, 0, 0), 1)
	require.ErrorIs(t, err, decad.ErrUnsupported, "a centre the sums round on")
	_, err = decad.QuarterTurn(t.Context(), block, z, r3.NewVec(math.NaN(), 0, 0), 1)
	require.ErrorIs(t, err, decad.ErrNotFinite)
	prism := boxBody(t, doc, 30, 0, 40, 8, 6)
	_, err = decad.QuarterTurn(t.Context(), prism, z, r3.NewVec(0, 0, 0), 1)
	require.ErrorIs(t, err, decad.ErrUnsupported, "a solid that is not faceted")
	_, err = decad.QuarterTurn(nil, block, z, r3.NewVec(0, 0, 0), 1) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.QuarterTurn(t.Context(), nil, z, r3.NewVec(0, 0, 0), 1)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 2, "every refusal leaves the document alone")
}
