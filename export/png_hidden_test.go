package export_test

import (
	"image"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §9's tests for View.HiddenEdges.

// countGrey counts the pixels of img within box that are exactly the given grey.
func countGrey(img *image.RGBA, box image.Rectangle, grey uint8) int {
	n := 0
	for y := box.Min.Y; y < box.Max.Y; y++ {
		for x := box.Min.X; x < box.Max.X; x++ {
			if c := img.RGBAAt(x, y); c.R == grey && c.G == grey && c.B == grey {
				n++
			}
		}
	}
	return n
}

func TestPNGHiddenEdgesDrawsTheFarCornerOfABlock(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	plain := export.IsometricView(300, 300)
	hidden := plain
	hidden.HiddenEdges = true
	both := hidden
	both.Outlines = true
	middle := image.Rect(100, 100, 200, 200)
	require.Zero(t, countGrey(renderPNG(t, body, plain), middle, 150), "no hidden edges unless asked")
	// The three hidden edges of a cube meet at the far corner, at the middle of its isometric
	// image, so dashed grey pixels fill the middle of the body's silhouette.
	img := renderPNG(t, body, hidden)
	require.Greater(t, countGrey(img, middle, 150), 60)
	require.Zero(t, countGrey(img, middle, 32), "no outline is drawn without Outlines")
	// The dashes are broken: far fewer grey pixels than the edges' full length would give.
	require.Less(t, countGrey(img, middle, 150), 3*50)
	withOutlines := renderPNG(t, body, both)
	// The near and far corners of a cube look the same in an isometric view, so the visible
	// edges cross the hidden ones there and cover a few of their pixels, and no more.
	require.InDelta(t, countGrey(img, middle, 150), countGrey(withOutlines, middle, 150), 6,
		"the outlines are drawn over the dashes, covering only where the edges cross")
	require.Greater(t, countGrey(withOutlines, image.Rect(0, 0, 300, 300), 32), 100, "the visible outlines are drawn")
}
