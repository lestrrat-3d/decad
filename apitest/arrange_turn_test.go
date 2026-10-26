package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/arrange-design.md §5's public-surface tests for
// WithArrangeTurns, on the bed of arrange_test.go.

func TestArrangeTurnsBodiesToFitTheBed(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	// Three 4 x 20 blocks need 22 mm of depth as they stand, and the bed has 20.
	blocks := arrangeBlocks(t, doc, [][2]float64{{4, 20}, {4, 20}, {4, 20}})
	_, err := decad.Arrange(t.Context(), blocks, arrangeBed(30, 20, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	out, err := decad.Arrange(t.Context(), blocks, arrangeBed(30, 20, 1), decad.WithArrangeTurns())
	require.NoError(t, err)
	require.Len(t, out, 3)
	require.Len(t, doc.Bodies(), 3, "the inputs and the turned bodies are retired")
	// Each block lies 20 along X and 4 along Y, on a shelf 5 mm above the last.
	for i, b := range out {
		y := 201 + 5*float64(i)
		decadtest.MeasuresBounds(t, b, r3.Vec{X: 101, Y: y, Z: 5}, r3.Vec{X: 121, Y: y + 4, Z: 8}, decadtest.Within(units.Millimeters(1e-6)))
		decadtest.MeasuresVolume(t, b, units.CubicMillimeters(240), decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

func TestArrangeTurnsLeavesAFlatBodyAlone(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}})
	history := len(doc.OperationHistory().Operations())
	out, err := decad.Arrange(t.Context(), blocks, arrangeBed(30, 20, 2), decad.WithArrangeTurns())
	require.NoError(t, err)
	decadtest.MeasuresBounds(t, out[0], r3.Vec{X: 102, Y: 202, Z: 5}, r3.Vec{X: 112, Y: 207, Z: 8}, decadtest.Within(units.Millimeters(1e-6)))
	require.Len(t, doc.OperationHistory().Operations(), history+1, "an unturned body takes one move")
}

func TestArrangeTurnsRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{30, 30}})
	_, err := decad.Arrange(t.Context(), blocks, arrangeBed(20, 20, 1), decad.WithArrangeTurns())
	require.ErrorIs(t, err, decad.ErrDegenerate, "a body too big either way stays refused")
	require.Equal(t, blocks, doc.Bodies(), "a refused arrangement moves nothing")
	_, err = decad.Arrange(t.Context(), blocks, arrangeBed(40, 40, 1), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
