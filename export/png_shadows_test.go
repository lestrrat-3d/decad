package export_test

import (
	"bytes"
	"image"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §10's tests for View.Shadows.

// shadowScene is a 60 x 40 x 2 plate with a 10 x 10 x 20 pillar on it, drawn from above.
func shadowScene(t *testing.T) []*decad.Body {
	t.Helper()
	doc := decad.New()
	plate := decadtest.NewBlock(t, doc, 0, 0, 60, 40, units.Millimeters(2))
	pillar := decadtest.NewBlock(t, doc, 25, 15, 35, 25, units.Millimeters(20))
	return []*decad.Body{plate, pillar}
}

func renderShadowScene(t *testing.T, bodies []*decad.Body, view export.View) *image.RGBA {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &out, bodies, units.Millimeters(0.1), view))
	img, err := png.Decode(&out)
	require.NoError(t, err)
	rgba := image.NewRGBA(img.Bounds())
	for y := range img.Bounds().Dy() {
		for x := range img.Bounds().Dx() {
			rgba.Set(x, y, img.At(x, y))
		}
	}
	return rgba
}

func TestPNGShadowsDarkenTheGroundBesideAPillar(t *testing.T) {
	t.Parallel()
	bodies := shadowScene(t)
	plain := topDown(400)
	lit := plain
	lit.Shadows = true
	off := renderShadowScene(t, bodies, plain)
	on := renderShadowScene(t, bodies, lit)
	// The key light is raised and to the left of the camera, so the pillar's shadow falls on
	// the plate to its right and below it in the image. Find the pixels that shadows darkened.
	darker, same := 0, 0
	var darkest [2]int
	for y := range 400 {
		for x := range 400 {
			a, b := off.RGBAAt(x, y), on.RGBAAt(x, y)
			switch {
			case a == b:
				same++
			case int(b.R)+int(b.G)+int(b.B) < int(a.R)+int(a.G)+int(a.B):
				darker++
				darkest = [2]int{x, y}
			}
		}
	}
	require.Greater(t, darker, 200, "a pillar's shadow covers a patch of the plate")
	require.Greater(t, same, 400*400/2, "most of the image is as lit as before")
	_ = darkest
	// The plate's far corner, away from the pillar, is as lit as ever, and so is the pillar's top.
	require.Equal(t, off.RGBAAt(30, 200), on.RGBAAt(30, 200))
	require.Equal(t, off.RGBAAt(200, 200), on.RGBAAt(200, 200), "the pillar's top faces the light")
}

func TestPNGShadowsLeaveALoneBlockAlone(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	plain := export.IsometricView(200, 200)
	lit := plain
	lit.Shadows = true
	// A convex block shadows none of itself: a face turned from the light is at the ambient level
	// already, and every other face sees the light.
	require.Equal(t, renderPNG(t, body, plain), renderPNG(t, body, lit))
}

func TestPNGShadowsLeaveARoundRodAlone(t *testing.T) {
	t.Parallel()
	// A rod's wall is a ring of narrow facets at every angle to the light, which a map with no
	// bias would speckle with their own shadows.
	rod := circleBodyForDXF(t, decad.New())
	plain := export.IsometricView(240, 240)
	lit := plain
	lit.Shadows = true
	require.Equal(t, renderPNG(t, rod, plain), renderPNG(t, rod, lit))
}

func TestPNGShadowsFallWhereAPerspectiveCameraPutsThem(t *testing.T) {
	t.Parallel()
	bodies := shadowScene(t)
	view := topDown(400)
	view.FieldOfView = units.Degrees(60)
	plain := view
	view.Shadows = true
	off := renderShadowScene(t, bodies, plain)
	on := renderShadowScene(t, bodies, view)
	// The scene spans 60 x 40 x 22 mm, so its bounding sphere has radius √5684/2 and the eye
	// sits twice that above its middle, at z = 11 + √5684, 86.39 mm. The plate's top, at z = 2,
	// is 84.39 mm from the eye, and each millimetre of it is 1/84.39 of the plane at distance 1;
	// the plate's 60 mm width fits 360 pixels, 506.3 per unit of that plane. A plate point at
	// (x, y) mm is so at pixel (200 + (x-30)·506.3/84.39, 200 - (y-20)·506.3/84.39).
	pixel := func(x, y float64) (int, int) {
		k := 506.3 / 84.39
		return int(200 + (x-30)*k), int(200 - (y-20)*k)
	}
	// The key light is above and to the left of the camera: it falls 0.35 to the right and 0.5
	// down the image per unit it drops, so the pillar's top, 18 mm over the plate, throws its
	// shadow 6.3 mm right and 9 mm down, onto x 31.3 to 41.3 and y 6 to 16.
	x, y := pixel(38, 10)
	require.Equal(t, [3]uint8{36, 48, 60}, [3]uint8{on.RGBAAt(x, y).R, on.RGBAAt(x, y).G, on.RGBAAt(x, y).B},
		"the shadow is the plate's blue at the ambient level, 0.3")
	require.NotEqual(t, off.RGBAAt(x, y), on.RGBAAt(x, y))
	// The plate across the pillar, and the pillar's top, are as lit as without shadows.
	x, y = pixel(20, 30)
	require.Equal(t, off.RGBAAt(x, y), on.RGBAAt(x, y), "the lit side of the plate")
	require.Equal(t, off.RGBAAt(200, 200), on.RGBAAt(200, 200), "the pillar's top faces the light")
	// A convex block shadows none of itself under a perspective camera either.
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	one := export.IsometricView(200, 200)
	one.FieldOfView = units.Degrees(60)
	lit := one
	lit.Shadows = true
	require.Equal(t, renderPNG(t, block, one), renderPNG(t, block, lit))
}
