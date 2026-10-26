package export_test

import (
	"bytes"
	"context"
	"image"
	"image/color"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var pngWhite = color.RGBA{R: 255, G: 255, B: 255, A: 255}

// topDown looks down the -Z axis with +Y up the image.
func topDown(size int) export.View {
	return export.View{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Width: size, Height: size}
}

func renderPNG(t *testing.T, body *decad.Body, view export.View) *image.RGBA {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.PNG(t.Context(), &out, body, units.Millimeters(0.1), view))
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

func TestPNGDrawsABlockFromAbove(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	img := renderPNG(t, body, topDown(200))
	require.Equal(t, image.Rect(0, 0, 200, 200), img.Bounds())
	// The 20 mm top face scales to 180 px with a 10 px margin.
	face := img.RGBAAt(100, 100)
	require.NotEqual(t, pngWhite, face)
	require.Greater(t, face.B, face.R, "a face toward the camera is bluish")
	require.Equal(t, face, img.RGBAAt(12, 12), "just inside the corner")
	require.Equal(t, face, img.RGBAAt(187, 187))
	require.Equal(t, pngWhite, img.RGBAAt(5, 5), "in the margin")
	require.Equal(t, pngWhite, img.RGBAAt(195, 100))
}

func TestPNGSeesThroughAHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(10))
	drilled, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.NewVec(10, 10, 10),
		Diameter: units.Millimeters(8),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	img := renderPNG(t, drilled, topDown(200))
	require.Equal(t, pngWhite, img.RGBAAt(100, 100), "the through hole shows the background")
	face := img.RGBAAt(100, 30)
	require.NotEqual(t, pngWhite, face, "the top face beside it")
	// The 8 mm hole is 72 px across at 9 px per mm: its edge is 36 px from the centre.
	require.Equal(t, pngWhite, img.RGBAAt(100+34, 100))
	require.Equal(t, face, img.RGBAAt(100+38, 100))
}

func TestPNGIsometricShowsThreeShades(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	img := renderPNG(t, body, export.IsometricView(300, 300))
	require.Equal(t, pngWhite, img.RGBAAt(2, 2), "the corners are empty")
	require.Equal(t, pngWhite, img.RGBAAt(297, 2))
	top := img.RGBAAt(150, 90)
	left := img.RGBAAt(100, 190)
	right := img.RGBAAt(200, 190)
	require.NotEqual(t, pngWhite, top)
	require.NotEqual(t, top, left)
	require.NotEqual(t, left, right, "the two side faces are lit differently")
}

func TestPNGIsDeterministic(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	var a, b bytes.Buffer
	require.NoError(t, export.PNG(t.Context(), &a, body, units.Millimeters(0.1), export.IsometricView(120, 90)))
	require.NoError(t, export.PNG(t.Context(), &b, body, units.Millimeters(0.1), export.IsometricView(120, 90)))
	require.Equal(t, a.Bytes(), b.Bytes())
}

func TestPNGRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	good := export.IsometricView(100, 100)
	var nilContext context.Context
	require.ErrorIs(t, export.PNG(nilContext, &bytes.Buffer{}, body, units.Millimeters(0.1), good), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNG(t.Context(), nil, body, units.Millimeters(0.1), good), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNG(t.Context(), &bytes.Buffer{}, nil, units.Millimeters(0.1), good), decad.ErrDegenerate)
	for name, view := range map[string]export.View{
		"no direction":    {Up: r3.NewVec(0, 0, 1), Width: 100, Height: 100},
		"up along view":   {Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 0, 1), Width: 100, Height: 100},
		"tiny image":      {Direction: r3.NewVec(-1, -1, -1), Up: r3.NewVec(0, 0, 1), Width: 4, Height: 100},
		"enormous height": {Direction: r3.NewVec(-1, -1, -1), Up: r3.NewVec(0, 0, 1), Width: 100, Height: 100000},
	} {
		var out bytes.Buffer
		require.ErrorIs(t, export.PNG(t.Context(), &out, body, units.Millimeters(0.1), view), decad.ErrDegenerate, name)
		require.Zero(t, out.Len(), "%s writes nothing", name)
	}
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	var out bytes.Buffer
	require.ErrorIs(t, export.PNG(canceled, &out, body, units.Millimeters(0.1), good), context.Canceled)
	require.Zero(t, out.Len())
	require.ErrorIs(t, export.PNG(t.Context(), &bytes.Buffer{}, body, units.Degrees(1), good), decad.ErrUnitKind)
}
