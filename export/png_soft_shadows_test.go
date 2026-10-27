package export_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §14's tests for View.ShadowSoftness.

func brightness(c [4]uint8) int { return int(c[0]) + int(c[1]) + int(c[2]) }

func TestPNGSoftShadowsGradeAcrossAPenumbra(t *testing.T) {
	t.Parallel()
	bodies := shadowScene(t)
	hardView := topDown(400)
	hardView.Shadows = true
	softView := hardView
	softView.ShadowSoftness = units.Degrees(5)
	plain := topDown(400)
	off := renderShadowScene(t, bodies, plain)
	hard := renderShadowScene(t, bodies, hardView)
	soft := renderShadowScene(t, bodies, softView)
	// The lit plate and the full shadow are the two levels a pixel of the hard render takes. Count the pixels
	// strictly between them: a hard shadow has none, a soft one a band of them.
	var lit, dark int
	for y := range 400 {
		for x := range 400 {
			a, h := off.RGBAAt(x, y), hard.RGBAAt(x, y)
			if a != h && brightness([4]uint8{h.R, h.G, h.B}) < brightness([4]uint8{a.R, a.G, a.B}) {
				dark = brightness([4]uint8{h.R, h.G, h.B})
				lit = brightness([4]uint8{a.R, a.G, a.B})
			}
		}
	}
	require.Greater(t, lit, dark)
	count := func(get func(x, y int) int) int {
		n := 0
		for y := range 400 {
			for x := range 400 {
				if b := get(x, y); b > dark && b < lit {
					n++
				}
			}
		}
		return n
	}
	hardMid := count(func(x, y int) int { c := hard.RGBAAt(x, y); return brightness([4]uint8{c.R, c.G, c.B}) })
	softMid := count(func(x, y int) int { c := soft.RGBAAt(x, y); return brightness([4]uint8{c.R, c.G, c.B}) })
	// The plate is flat and headlit, so its lit and shadowed levels are single values, and the hard render
	// has only a few edge pixels between them.
	require.Less(t, hardMid, 40)
	require.Greater(t, softMid, 300, "the penumbra is a band of intermediate shades")
	// The pillar's top, and the plate far from the pillar, are as lit as ever.
	require.Equal(t, off.RGBAAt(200, 200), soft.RGBAAt(200, 200))
	require.Equal(t, off.RGBAAt(30, 200), soft.RGBAAt(30, 200))
	// A wider light gives a wider penumbra.
	wideView := softView
	wideView.ShadowSoftness = units.Degrees(12)
	wide := renderShadowScene(t, bodies, wideView)
	wideMid := count(func(x, y int) int { c := wide.RGBAAt(x, y); return brightness([4]uint8{c.R, c.G, c.B}) })
	require.Greater(t, wideMid, softMid)
}

func TestPNGSoftShadowsRefusals(t *testing.T) {
	t.Parallel()
	body := shadowScene(t)[1]
	render := func(v export.View) error {
		return export.PNG(t.Context(), &bytes.Buffer{}, body, units.Millimeters(0.1), v)
	}
	view := topDown(100)
	view.ShadowSoftness = units.Degrees(5)
	require.ErrorIs(t, render(view), decad.ErrDegenerate, "softness without shadows")
	view.Shadows = true
	require.NoError(t, render(view))
	view.ShadowSoftness = units.Degrees(60)
	require.ErrorIs(t, render(view), decad.ErrDegenerate, "wider than 45 degrees")
	view.ShadowSoftness = units.Millimeters(5)
	require.ErrorIs(t, render(view), decad.ErrUnitKind)
}
