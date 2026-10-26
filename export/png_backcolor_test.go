package export_test

import (
	"bytes"
	"image/color"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §11's tests for View.BackColor,
// on a unit cube wound inside out, so that every face it draws looks away.

// cubeMesh is a 10 mm cube mesh wound outward, or inside out when inverted.
func cubeMesh(t *testing.T, inverted bool) *decad.MeshBody {
	t.Helper()
	v := []r3.Vec{{}, {X: 10}, {X: 10, Y: 10}, {Y: 10}, {Z: 10}, {X: 10, Z: 10}, {X: 10, Y: 10, Z: 10}, {Y: 10, Z: 10}}
	tri := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	if inverted {
		for i, f := range tri {
			tri[i] = [3]int{f[0], f[2], f[1]}
		}
	}
	m, err := decad.NewMeshBody(t.Context(), v, tri)
	require.NoError(t, err)
	return m
}

func meshPixel(t *testing.T, m *decad.MeshBody, view export.View) color.RGBA {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.MeshPNG(t.Context(), &out, m, view))
	img, err := png.Decode(&out)
	require.NoError(t, err)
	return color.RGBAModel.Convert(img.At(view.Width/2, view.Height/2)).(color.RGBA)
}

func TestPNGBackColorReplacesTheOrangeOfAFaceLookingAway(t *testing.T) {
	t.Parallel()
	m := cubeMesh(t, true)
	view := topDown(200)
	orange := meshPixel(t, m, view)
	require.Greater(t, int(orange.R), int(orange.B)+40, "the default is orange: red well above blue")
	view.BackColor = &color.RGBA{R: 30, G: 200, B: 60, A: 9}
	green := meshPixel(t, m, view)
	require.Greater(t, int(green.G), 2*int(green.R), "the chosen colour, at the same shade")
	require.Equal(t, uint8(255), green.A, "the alpha is ignored")
	// Shaded alike: the same level of light, so the two colours are in the ratio of their bases.
	require.InDelta(t, float64(orange.R)/220, float64(green.G)/200, 0.02)
}

func TestPNGBackColorLeavesTheFrontFacesAlone(t *testing.T) {
	t.Parallel()
	cube := cubeMesh(t, false)
	view := topDown(200)
	plain := meshPixel(t, cube, view)
	require.Greater(t, int(plain.B), int(plain.R), "a face toward the camera is blue")
	view.BackColor = &color.RGBA{R: 30, G: 200, B: 60}
	require.Equal(t, plain, meshPixel(t, cube, view))
}
