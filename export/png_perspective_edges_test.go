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

// This file is agent-docs/png-export-design.md §12's tests for View.Outlines and
// View.HiddenEdges with a perspective camera.

// perspectiveBlock is a 20 x 20 x 10 mm block seen from straight above through a
// 60 degree field on a 200 pixel image. The scene's bounding sphere has radius
// 15 mm, so the eye sits 30 mm above its centre, 35 mm above the block's bottom
// and 25 mm above its top. A half side of 10 mm then lies 10/25 = 0.4 from the
// axis in the plane at distance 1 on top and 10/35 on the bottom. The top face
// fills the 0.8 span less a 10 pixel margin, 225 pixels per unit, so the bottom
// face's edges lie 100 +- 64.3 pixels from the image's middle.
func perspectiveBlock(t *testing.T) (*decad.Body, export.View) {
	t.Helper()
	view := topDown(200)
	view.FieldOfView = units.Degrees(60)
	return decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10)), view
}

func TestPNGPerspectiveOutlinesTraceTheNearFace(t *testing.T) {
	t.Parallel()
	body, view := perspectiveBlock(t)
	view.Outlines = true
	img := renderPNG(t, body, view)
	for _, p := range [][2]int{{10, 100}, {190, 100}, {100, 10}, {100, 190}} {
		got := img.RGBAAt(p[0], p[1])
		require.Equal(t, [3]uint8{32, 32, 32}, [3]uint8{got.R, got.G, got.B}, "the top face's outline at %v", p)
	}
	// The bottom face lies behind the top one, so its edges are hidden and not outlined.
	require.Zero(t, countGrey(img, image.Rect(30, 40, 42, 160), 32), "no outline where the bottom edge is hidden")
}

func TestPNGPerspectiveHiddenEdgesShowTheFarFace(t *testing.T) {
	t.Parallel()
	body, view := perspectiveBlock(t)
	left := image.Rect(34, 44, 39, 156) // the bottom face's left edge, 36 pixels in
	require.Zero(t, countGrey(renderPNG(t, body, view), left, 150), "no hidden edges unless asked")
	view.HiddenEdges = true
	img := renderPNG(t, body, view)
	// Dashes of 4 pixels in 7 along 128 pixels.
	require.Greater(t, countGrey(img, left, 150), 40, "the bottom face's left edge is drawn dashed")
	require.Less(t, countGrey(img, left, 150), 5*128, "and broken")
	right := image.Rect(161, 44, 166, 156)
	require.Greater(t, countGrey(img, right, 150), 40, "its right edge too")
	// The four vertical edges run from each top corner to the bottom's, 26 pixels inward.
	require.Greater(t, countGrey(img, image.Rect(12, 12, 34, 34), 150), 8, "a vertical edge runs to the bottom corner")
	// Where an orthographic camera puts the bottom edge under the top one, this one puts it inside.
	ortho := view
	ortho.FieldOfView = units.Value{}
	require.Zero(t, countGrey(renderPNG(t, body, ortho), left, 150), "an orthographic view has no edge there")
}

func TestPNGPerspectiveEdgesOnAnObliqueView(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	view := export.IsometricView(300, 300)
	view.FieldOfView = units.Degrees(40)
	view.Outlines = true
	view.HiddenEdges = true
	img := renderPNG(t, body, view)
	whole := image.Rect(0, 0, 300, 300)
	require.Greater(t, countGrey(img, whole, 32), 400, "the visible edges are outlined")
	require.Greater(t, countGrey(img, whole, 150), 60, "the hidden edges are dashed")
	again := renderPNG(t, body, view)
	require.Equal(t, img.Pix, again.Pix, "equal views give equal pixels")
}
