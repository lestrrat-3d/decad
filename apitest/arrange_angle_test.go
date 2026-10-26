package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/arrange-design.md §6's public-surface tests for
// WithArrangeAngle, on the bed of arrange_test.go.

// diagonalBar is a 4 x 30 x 3 block turned 45 degrees about +Z, so its footprint
// is about 24 by 24 mm although its long side is 30.
func diagonalBar(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	bar := decadtest.NewBlock(t, doc, 0, 0, 4, 30, units.Millimeters(3))
	turn, err := r3.RotationAround(r3.Vec{X: 2, Y: 15}, r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	turned, err := decad.Transform(t.Context(), bar, turn)
	require.NoError(t, err)
	return turned
}

func TestArrangeAngleLaysADiagonalBarSquareToTheBed(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	bar := diagonalBar(t, doc)
	_, err := decad.Arrange(t.Context(), []*decad.Body{bar}, arrangeBed(40, 10, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "turned 45 degrees it is about 24 mm deep")
	out, err := decad.Arrange(t.Context(), []*decad.Body{bar}, arrangeBed(40, 10, 1), decad.WithArrangeAngle(units.Degrees(5)))
	require.NoError(t, err)
	// A further 45 degrees puts its long side along X: 30 by 4, at the gap from the corner.
	decadtest.MeasuresBounds(t, out[0], r3.Vec{X: 101, Y: 201, Z: 5}, r3.Vec{X: 131, Y: 205, Z: 8}, decadtest.Within(units.Millimeters(1e-6)))
	decadtest.MeasuresVolume(t, out[0], units.CubicMillimeters(360), decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestArrangeAngleKeepsABodyThatIsAlreadySquare(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}})
	history := len(doc.OperationHistory().Operations())
	out, err := decad.Arrange(t.Context(), blocks, arrangeBed(30, 20, 2), decad.WithArrangeAngle(units.Degrees(15)))
	require.NoError(t, err)
	decadtest.MeasuresBounds(t, out[0], r3.Vec{X: 102, Y: 202, Z: 5}, r3.Vec{X: 112, Y: 207, Z: 8}, decadtest.Within(units.Millimeters(1e-6)))
	var moves int
	for _, op := range doc.OperationHistory().Operations()[history:] {
		if op.Kind() == decad.OperationTransform {
			moves++
		}
	}
	require.Equal(t, 1, moves, "a body already square to the axes takes only its move")
}

func TestArrangeAngleCoarseStepMissesTheBestAngle(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	bar := diagonalBar(t, doc)
	// The multiples of 30 degrees leave the bar 15 degrees off the X axis, about 11.7 mm deep.
	_, err := decad.Arrange(t.Context(), []*decad.Body{bar}, arrangeBed(40, 10, 1), decad.WithArrangeAngle(units.Degrees(30)))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestArrangeAngleRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{4, 4}})
	_, err := decad.Arrange(t.Context(), blocks, arrangeBed(40, 40, 1), decad.WithArrangeAngle(units.Millimeters(5)))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = decad.Arrange(t.Context(), blocks, arrangeBed(40, 40, 1), decad.WithArrangeAngle(units.Degrees(-5)))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, err = decad.Arrange(t.Context(), blocks, arrangeBed(40, 40, 1), decad.WithArrangeAngle(units.Degrees(0)))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Arrange(t.Context(), blocks, arrangeBed(40, 40, 1), decad.WithArrangeAngle(units.Degrees(0.1)))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, blocks, doc.Bodies(), "a refused arrangement moves nothing")
}
