package export_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §6's tests for View.Outlines.

func TestPNGOutlinesTraceTheEdgesOfABlock(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	plain := topDown(200)
	outlined := plain
	outlined.Outlines = true
	off := renderPNG(t, body, plain)
	on := renderPNG(t, body, outlined)
	// The 20 mm top face spans pixels 10 to 190. Without outlines the first pixel
	// along a row through the middle that is not white is the blue face; with
	// them it is the dark outline.
	first := func(row int) (int, [4]uint8) {
		for x := range 200 {
			if c := on.RGBAAt(x, row); c != pngWhite {
				return x, [4]uint8{c.R, c.G, c.B, c.A}
			}
		}
		return -1, [4]uint8{}
	}
	x, c := first(100)
	require.InDelta(t, 10, x, 1)
	require.Equal(t, [4]uint8{32, 32, 32, 255}, c)
	require.Equal(t, off.RGBAAt(100, 100), on.RGBAAt(100, 100), "the inside of a face is not outlined")
	require.NotEqual(t, [3]uint8{32, 32, 32}, [3]uint8{off.RGBAAt(10, 100).R, off.RGBAAt(10, 100).G, off.RGBAAt(10, 100).B}, "the plain image has no outline")
	// The four sides are all drawn.
	for _, p := range [][2]int{{10, 100}, {190, 100}, {100, 10}, {100, 190}} {
		got := on.RGBAAt(p[0], p[1])
		require.Equal(t, [3]uint8{32, 32, 32}, [3]uint8{got.R, got.G, got.B}, "an outline at %v", p)
	}
}

func TestPNGOutlinesLeaveAHiddenEdgeOut(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	view := topDown(200)
	view.Outlines = true
	img := renderPNG(t, body, view)
	// The block's bottom face has the same outline as its top seen from above
	// and lies behind it, so no dark pixel appears inside the top face.
	for _, p := range [][2]int{{60, 60}, {140, 60}, {60, 140}, {140, 140}, {100, 100}} {
		got := img.RGBAAt(p[0], p[1])
		require.NotEqual(t, [3]uint8{32, 32, 32}, [3]uint8{got.R, got.G, got.B}, "inside the face at %v", p)
	}
}
