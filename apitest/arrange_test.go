package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/arrange-design.md's public-surface tests for Arrange.

func arrangeBed(width, depth, gap float64) decad.ArrangeBed {
	return decad.ArrangeBed{
		Origin: r3.Vec{X: 100, Y: 200, Z: 5},
		Width:  units.Millimeters(width),
		Depth:  units.Millimeters(depth),
		Gap:    units.Millimeters(gap),
	}
}

// arrangeBlocks returns blocks of the given footprints, each 3 mm tall and
// built at a different place, so the arrangement has to bring them together.
func arrangeBlocks(tb testing.TB, doc *decad.Document, sizes [][2]float64) []*decad.Body {
	tb.Helper()
	out := make([]*decad.Body, len(sizes))
	for i, sz := range sizes {
		x := float64(i) * 1000
		out[i] = decadtest.NewBlock(tb, doc, x, -x, x+sz[0], -x+sz[1], units.Millimeters(3))
	}
	return out
}

func TestArrangePlacesBodiesInShelves(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}, {10, 8}, {10, 5}, {10, 3}})
	out, err := decad.Arrange(t.Context(), blocks, arrangeBed(36, 40, 2))
	require.NoError(t, err)
	require.Len(t, out, 4)
	require.Len(t, doc.Bodies(), 4, "the inputs are retired and the moved bodies are live")
	// The bed's corner is (100, 200) at z = 5. The deepest block opens the
	// first shelf at one gap in, and the wrapped blocks start a second shelf.
	want := [][2]float64{{114, 202}, {102, 202}, {102, 212}, {114, 212}}
	sizes := [][2]float64{{10, 5}, {10, 8}, {10, 5}, {10, 3}}
	for i, b := range out {
		decadtest.MeasuresBounds(t, b,
			r3.Vec{X: want[i][0], Y: want[i][1], Z: 5},
			r3.Vec{X: want[i][0] + sizes[i][0], Y: want[i][1] + sizes[i][1], Z: 8})
		decadtest.MeasuresVolume(t, b, units.CubicMillimeters(sizes[i][0]*sizes[i][1]*3))
	}
}

func TestArrangeRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		sizes [][2]float64
		bed   decad.ArrangeBed
		want  error
	}{
		{"does not fit", [][2]float64{{10, 10}, {10, 10}}, arrangeBed(14, 24, 2), decad.ErrDegenerate},
		{"zero width", [][2]float64{{10, 10}}, arrangeBed(0, 24, 2), decad.ErrDegenerate},
		{"negative gap", [][2]float64{{10, 10}}, arrangeBed(40, 40, -1), decad.ErrNegativeMagnitude},
		{"depth as an angle", [][2]float64{{10, 10}}, decad.ArrangeBed{Width: units.Millimeters(40), Depth: units.Degrees(40), Gap: units.Millimeters(1)}, decad.ErrUnitKind},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			blocks := arrangeBlocks(t, doc, tc.sizes)
			_, err := decad.Arrange(t.Context(), blocks, tc.bed)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, blocks, doc.Bodies(), "a refused arrangement moves nothing")
		})
	}
}

func TestArrangeRefusesBadBodyLists(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := arrangeBlocks(t, doc, [][2]float64{{10, 10}})
	_, err := decad.Arrange(t.Context(), nil, arrangeBed(40, 40, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "an empty list")
	_, err = decad.Arrange(t.Context(), []*decad.Body{blocks[0], blocks[0]}, arrangeBed(40, 40, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a body twice")
	_, err = decad.Arrange(t.Context(), []*decad.Body{blocks[0], nil}, arrangeBed(40, 40, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil body")
	other := decad.New()
	foreign := decadtest.NewBlock(t, other, 0, 0, 5, 5, units.Millimeters(1))
	_, err = decad.Arrange(t.Context(), []*decad.Body{blocks[0], foreign}, arrangeBed(40, 40, 1))
	require.ErrorIs(t, err, decad.ErrForeignBody)
	require.Equal(t, blocks, doc.Bodies())
}
