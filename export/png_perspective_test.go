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

// This file is agent-docs/png-export-design.md §7's tests for View.FieldOfView.

func renderBodies(t *testing.T, bodies []*decad.Body, view export.View) *image.RGBA {
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

func covered(img *image.RGBA, from, to int) int {
	n := 0
	for x := from; x < to; x++ {
		if img.RGBAAt(x, img.Bounds().Dy()/2) != pngWhite {
			n++
		}
	}
	return n
}

func TestPNGPerspectiveMakesANearerBlockLarger(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	// Two 4 x 4 mm blocks side by side along X: one 10 mm tall, one 2 mm. Looking
	// down, the taller block's top is nearer the camera.
	tall := decadtest.NewBlock(t, doc, -6, -2, -2, 2, units.Millimeters(10))
	flat := decadtest.NewBlock(t, doc, 2, -2, 6, 2, units.Millimeters(2))
	view := topDown(200)
	ortho := renderBodies(t, []*decad.Body{tall, flat}, view)
	require.InDelta(t, covered(ortho, 0, 100), covered(ortho, 100, 200), 1, "orthographic: equal footprints")
	view.FieldOfView = units.Degrees(60)
	persp := renderBodies(t, []*decad.Body{tall, flat}, view)
	require.Greater(t, covered(persp, 0, 100), covered(persp, 100, 200)+10, "the nearer, taller block looks wider")
}

func TestPNGPerspectiveZeroFieldOfViewIsOrthographic(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	var a, b bytes.Buffer
	require.NoError(t, export.PNG(t.Context(), &a, body, units.Millimeters(0.1), topDown(120)))
	view := topDown(120)
	view.FieldOfView = units.Value{}
	require.NoError(t, export.PNG(t.Context(), &b, body, units.Millimeters(0.1), view))
	require.Equal(t, a.Bytes(), b.Bytes())
}

func TestPNGPerspectiveRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	tol := units.Millimeters(0.1)
	for name, tc := range map[string]struct {
		edit func(*export.View)
		want error
	}{
		"180 degrees":     {func(v *export.View) { v.FieldOfView = units.Degrees(180) }, decad.ErrDegenerate},
		"negative":        {func(v *export.View) { v.FieldOfView = units.Degrees(-10) }, decad.ErrNegativeMagnitude},
		"a length":        {func(v *export.View) { v.FieldOfView = units.Millimeters(5) }, decad.ErrUnitKind},
		"with shadows":    {func(v *export.View) { v.FieldOfView = units.Degrees(60); v.Shadows = true }, decad.ErrDegenerate},
		"zero as degrees": {func(v *export.View) { v.FieldOfView = units.Degrees(0) }, decad.ErrDegenerate},
	} {
		view := topDown(64)
		tc.edit(&view)
		var out bytes.Buffer
		require.ErrorIs(t, export.PNG(t.Context(), &out, body, tol, view), tc.want, name)
		require.Zero(t, out.Len(), name)
	}
}
