package main

import (
	"context"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"io"

	"github.com/lestrrat-3d/solidlens"
)

// renderVerifiedFit is called only after verifyFitGap has checked the scene's
// two live bodies and certified that the measured gap rounds to 7 mm.
func renderVerifiedFit(ctx context.Context, writer io.Writer, scene solidlens.Scene, settings solidlens.Settings) error {
	img, err := solidlens.Render(ctx, scene, settings)
	if err != nil {
		return err
	}
	drawVerifyBadge(img)
	return png.Encode(writer, img)
}

// The glyphs use five columns and seven rows, so the badge needs no font
// dependency or platform font to render identically in CI.
var verifyGlyphs = map[byte][7]uint8{
	'7': {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000},
	'M': {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001},
	'G': {0b01111, 0b10000, 0b10000, 0b10111, 0b10001, 0b10001, 0b01111},
	'A': {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001},
	'P': {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000},
}

func drawVerifyBadge(img *image.RGBA) {
	scale := max(1, (img.Bounds().Dx()+80)/160)
	x, y := 6*scale, 2*scale
	green := color.RGBA{R: 82, G: 237, B: 175, A: 255}
	navy := color.RGBA{R: 15, G: 33, B: 52, A: 255}
	white := color.RGBA{R: 244, G: 250, B: 255, A: 255}
	badgeRect(img, x, y, x+74*scale, y+18*scale, navy)
	badgeRect(img, x, y, x+74*scale, y+scale, green)
	badgeRect(img, x, y+17*scale, x+74*scale, y+18*scale, green)
	badgeRect(img, x, y, x+scale, y+18*scale, green)
	badgeRect(img, x+73*scale, y, x+74*scale, y+18*scale, green)

	check := [7]uint8{0b00001, 0b00010, 0b00100, 0b10100, 0b01000}
	badgeGlyph(img, x+4*scale, y+2*scale, 2*scale, check, green)
	pen := x + 19*scale
	for _, letter := range []byte("7 MM GAP") {
		badgeGlyph(img, pen, y+6*scale, scale, verifyGlyphs[letter], white)
		pen += 6 * scale
	}
}

func badgeGlyph(img *image.RGBA, x, y, scale int, glyph [7]uint8, ink color.RGBA) {
	for row, bits := range glyph {
		for column := range 5 {
			if bits&(1<<(4-column)) != 0 {
				badgeRect(img, x+column*scale, y+row*scale, x+(column+1)*scale, y+(row+1)*scale, ink)
			}
		}
	}
}

func badgeRect(img *image.RGBA, minX, minY, maxX, maxY int, ink color.RGBA) {
	draw.Draw(img, image.Rect(minX, minY, maxX, maxY), image.NewUniform(ink), image.Point{}, draw.Src)
}
