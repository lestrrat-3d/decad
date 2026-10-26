package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/arrange-design.md §7's public-surface tests for
// ArrangeBed.Height, on the bed of arrange_test.go.

func layeredBed(height float64) decad.ArrangeBed {
	bed := arrangeBed(20, 10, 1)
	bed.Height = units.Millimeters(height)
	return bed
}

// layeredBlocks are three 8 x 8 blocks 5, 3 and 4 mm tall, built far apart.
func layeredBlocks(t *testing.T, doc *decad.Document) []*decad.Body {
	t.Helper()
	var out []*decad.Body
	for i, h := range []float64{5, 3, 4} {
		x := float64(i) * 1000
		out = append(out, decadtest.NewBlock(t, doc, x, -x, x+8, -x+8, units.Millimeters(h)))
	}
	return out
}

func TestArrangeStacksWhatTheBedCannotHoldInLayers(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := layeredBlocks(t, doc)
	// The bed holds two 8 x 8 blocks side by side, so the third goes above, a gap over the taller
	// of the first two: its base at the bed's surface z = 5 plus 5 + 1.
	_, err := decad.Arrange(t.Context(), blocks, arrangeBed(20, 10, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "without a height there is one layer, and three blocks do not fit")
	out, err := decad.Arrange(t.Context(), blocks, layeredBed(100))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 3)
	decadtest.MeasuresBounds(t, out[0], r3.Vec{X: 101, Y: 201, Z: 5}, r3.Vec{X: 109, Y: 209, Z: 10}, decadtest.Within(units.Millimeters(1e-6)))
	decadtest.MeasuresBounds(t, out[1], r3.Vec{X: 110, Y: 201, Z: 5}, r3.Vec{X: 118, Y: 209, Z: 8}, decadtest.Within(units.Millimeters(1e-6)))
	decadtest.MeasuresBounds(t, out[2], r3.Vec{X: 101, Y: 201, Z: 11}, r3.Vec{X: 109, Y: 209, Z: 15}, decadtest.Within(units.Millimeters(1e-6)))
	for i, h := range []float64{5, 3, 4} {
		decadtest.MeasuresVolume(t, out[i], units.CubicMillimeters(64*h), decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

func TestArrangeLayersKeepToTheHeightLimit(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := layeredBlocks(t, doc)
	// Two layers, 5 and 4 tall with a gap of 1, stand 10 high.
	_, err := decad.Arrange(t.Context(), blocks, layeredBed(9.5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, blocks, doc.Bodies(), "a refused arrangement moves nothing")
	_, err = decad.Arrange(t.Context(), blocks, layeredBed(10.5))
	require.NoError(t, err)
}

func TestArrangeLayersRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := layeredBlocks(t, doc)
	bed := layeredBed(100)
	bed.Height = units.Degrees(5)
	_, err := decad.Arrange(t.Context(), blocks, bed)
	require.ErrorIs(t, err, decad.ErrUnitKind)
	bed.Height = units.Millimeters(-5)
	_, err = decad.Arrange(t.Context(), blocks, bed)
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	// A block wider than the bed fits no layer, however high the bed.
	wide := decadtest.NewBlock(t, doc, 5000, 0, 5030, 8, units.Millimeters(2))
	_, err = decad.Arrange(t.Context(), []*decad.Body{wide}, layeredBed(100))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
