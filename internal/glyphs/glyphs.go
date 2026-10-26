// Package glyphs lays a line of text out in a TrueType font and returns the
// closed polygons of its glyph outlines (agent-docs/text-design.md). It works
// on plain millimetre data and touches no sketch, body or document.
package glyphs

import (
	"errors"
	"fmt"
	"math"
	"strings"

	"golang.org/x/image/font"
	"golang.org/x/image/font/gofont/goregular"
	"golang.org/x/image/font/sfnt"
	"golang.org/x/image/math/fixed"
)

// ErrDegenerate marks a font, size or text that draws nothing, or a rune the
// font has no glyph for.
var ErrDegenerate = errors.New("glyphs: degenerate text")

// maxContourPoints bounds the polygon points one call may produce.
const maxContourPoints = 200000

// Contour is one closed polygon in plane-local millimetres, the first point not
// repeated at the end.
type Contour [][2]float64

// Result is the laid-out text.
type Result struct {
	// Contours are the glyph outlines, in glyph order, each closed.
	Contours []Contour
	// Advances are the pen positions after each rune of the text, along the
	// baseline of its line, in millimetres from the line's start. A newline's is
	// zero.
	Advances []float64
}

// DefaultFont returns the bytes of the font used when none is given: Go Regular.
func DefaultFont() []byte { return goregular.TTF }

// Layout sets text at the baseline origin (0, 0) in the font ttf, size
// millimetres to the em, and flattens each outline curve until it lies within
// tolerance millimetres of its polygon. Lines break at "\\n" and are a line
// height apart, the first on the baseline y = 0 and later ones below it. There
// is no kerning, and a rune with no glyph is ErrDegenerate. size and tolerance
// are positive and finite.
func Layout(ttf []byte, text string, size, tolerance float64) (Result, error) {
	if !(size > 0) || math.IsInf(size, 0) || !(tolerance > 0) || math.IsInf(tolerance, 0) {
		return Result{}, fmt.Errorf("%w: the size and the flattening tolerance must be positive and finite", ErrDegenerate)
	}
	f, err := sfnt.Parse(ttf)
	if err != nil {
		return Result{}, fmt.Errorf("%w: the font does not parse: %s", ErrDegenerate, err)
	}
	var buf sfnt.Buffer
	units := int(f.UnitsPerEm())
	if units <= 0 {
		return Result{}, fmt.Errorf("%w: the font has no units per em", ErrDegenerate)
	}
	// One pixel is one font unit, so a coordinate is a font coordinate in 1/64 unit.
	ppem := fixed.I(units)
	metrics, err := f.Metrics(&buf, ppem, font.HintingNone)
	if err != nil {
		return Result{}, fmt.Errorf("%w: the font has no metrics: %s", ErrDegenerate, err)
	}
	scale := size / float64(units)
	lineHeight := float64(metrics.Height) / 64 * scale
	var out Result
	points := 0
	for lineNo, line := range strings.Split(text, "\n") {
		pen := 0.0
		for _, r := range line {
			index, err := f.GlyphIndex(&buf, r)
			if err != nil {
				return Result{}, fmt.Errorf("%w: rune %q: %s", ErrDegenerate, r, err)
			}
			if index == 0 && r != 0 {
				return Result{}, fmt.Errorf("%w: the font has no glyph for %q", ErrDegenerate, r)
			}
			segments, err := f.LoadGlyph(&buf, index, ppem, nil)
			if err != nil {
				return Result{}, fmt.Errorf("%w: rune %q: %s", ErrDegenerate, r, err)
			}
			offset := [2]float64{pen, -float64(lineNo) * lineHeight}
			contours := flatten(segments, scale, offset, tolerance)
			for _, c := range contours {
				points += len(c)
			}
			if points > maxContourPoints {
				return Result{}, fmt.Errorf("%w: the text needs more than %d outline points", ErrDegenerate, maxContourPoints)
			}
			out.Contours = append(out.Contours, contours...)
			advance, err := f.GlyphAdvance(&buf, index, ppem, font.HintingNone)
			if err != nil {
				return Result{}, fmt.Errorf("%w: rune %q: %s", ErrDegenerate, r, err)
			}
			pen += float64(advance) / 64 * scale
			out.Advances = append(out.Advances, pen)
		}
		out.Advances = append(out.Advances, 0)
	}
	if len(out.Contours) == 0 {
		return Result{}, fmt.Errorf("%w: the text has no outline", ErrDegenerate)
	}
	return out, nil
}

// flatten turns one glyph's segments into closed polygons in plane-local
// millimetres: font units scaled, the y axis turned up and the glyph moved by
// offset.
func flatten(segments sfnt.Segments, scale float64, offset [2]float64, tolerance float64) []Contour {
	var contours []Contour
	var current Contour
	var last [2]float64
	point := func(p fixed.Point26_6) [2]float64 {
		return [2]float64{float64(p.X)/64*scale + offset[0], -float64(p.Y)/64*scale + offset[1]}
	}
	add := func(p [2]float64) {
		if len(current) == 0 || current[len(current)-1] != p {
			current = append(current, p)
		}
		last = p
	}
	closeContour := func() {
		if len(current) > 1 && current[0] == current[len(current)-1] {
			current = current[:len(current)-1]
		}
		if len(current) >= 3 {
			contours = append(contours, current)
		}
		current = nil
	}
	for _, seg := range segments {
		switch seg.Op {
		case sfnt.SegmentOpMoveTo:
			closeContour()
			add(point(seg.Args[0]))
		case sfnt.SegmentOpLineTo:
			add(point(seg.Args[0]))
		case sfnt.SegmentOpQuadTo:
			c, e := point(seg.Args[0]), point(seg.Args[1])
			deviation := math.Hypot(last[0]-2*c[0]+e[0], last[1]-2*c[1]+e[1]) / 4
			n := pieces(deviation, tolerance)
			p0 := last
			for i := 1; i <= n; i++ {
				t := float64(i) / float64(n)
				u := 1 - t
				add([2]float64{u*u*p0[0] + 2*u*t*c[0] + t*t*e[0], u*u*p0[1] + 2*u*t*c[1] + t*t*e[1]})
			}
		case sfnt.SegmentOpCubeTo:
			c1, c2, e := point(seg.Args[0]), point(seg.Args[1]), point(seg.Args[2])
			second := math.Hypot(last[0]-2*c1[0]+c2[0], last[1]-2*c1[1]+c2[1])
			third := math.Hypot(c1[0]-2*c2[0]+e[0], c1[1]-2*c2[1]+e[1])
			n := pieces(0.75*math.Max(second, third), tolerance)
			p0 := last
			for i := 1; i <= n; i++ {
				t := float64(i) / float64(n)
				u := 1 - t
				b0, b1, b2, b3 := u*u*u, 3*u*u*t, 3*u*t*t, t*t*t
				add([2]float64{b0*p0[0] + b1*c1[0] + b2*c2[0] + b3*e[0], b0*p0[1] + b1*c1[1] + b2*c2[1] + b3*e[1]})
			}
		}
	}
	closeContour()
	return contours
}

// pieces returns how many straight pieces bring a curve whose chord deviation
// is deviation within tolerance: the deviation falls with the square of the
// piece count.
func pieces(deviation, tolerance float64) int {
	if !(deviation > tolerance) {
		return 1
	}
	return min(int(math.Ceil(math.Sqrt(deviation/tolerance))), 256)
}
