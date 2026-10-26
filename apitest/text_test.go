package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
	"golang.org/x/image/font"
	"golang.org/x/image/font/gofont/goregular"
	"golang.org/x/image/font/sfnt"
	"golang.org/x/image/math/fixed"
)

// This file is agent-docs/text-design.md's public-surface tests for
// TextProfiles. Expected sizes come straight from the Go Regular font's own
// metrics, read with the sfnt package at one pixel to the font unit.

// goFont reads Go Regular and its units per em.
func goFont(t *testing.T) (*sfnt.Font, float64) {
	t.Helper()
	f, err := sfnt.Parse(goregular.TTF)
	require.NoError(t, err)
	return f, float64(f.UnitsPerEm())
}

// box is an extent with y up, in font units or millimetres.
type box struct{ x0, y0, x1, y1 float64 }

// glyphBox returns the bounds of r in font units, with y up, and its advance.
func glyphBox(t *testing.T, r rune) (box, float64) {
	t.Helper()
	f, units := goFont(t)
	var buf sfnt.Buffer
	index, err := f.GlyphIndex(&buf, r)
	require.NoError(t, err)
	bounds, adv, err := f.GlyphBounds(&buf, index, fixed.I(int(units)), font.HintingNone)
	require.NoError(t, err)
	return box{float64(bounds.Min.X) / 64, -float64(bounds.Max.Y) / 64, float64(bounds.Max.X) / 64, -float64(bounds.Min.Y) / 64}, float64(adv) / 64
}

// straightGlyphArea is the area, in squared font units, of a glyph the font draws
// with straight segments alone, from the shoelace formula over each contour.
func straightGlyphArea(t *testing.T, r rune) float64 {
	t.Helper()
	f, units := goFont(t)
	var buf sfnt.Buffer
	index, err := f.GlyphIndex(&buf, r)
	require.NoError(t, err)
	segments, err := f.LoadGlyph(&buf, index, fixed.I(int(units)), nil)
	require.NoError(t, err)
	total := 0.0
	var first, prev [2]float64
	have := false
	closeContour := func() {
		if have {
			total += prev[0]*first[1] - first[0]*prev[1]
		}
	}
	for _, seg := range segments {
		p := [2]float64{float64(seg.Args[0].X) / 64, float64(seg.Args[0].Y) / 64}
		switch seg.Op {
		case sfnt.SegmentOpMoveTo:
			closeContour()
			first, prev, have = p, p, true
		case sfnt.SegmentOpLineTo:
			total += prev[0]*p[1] - p[0]*prev[1]
			prev = p
		default:
			require.Fail(t, "the glyph is not drawn with straight segments alone")
		}
	}
	closeContour()
	return math.Abs(total) / 2
}

// profileBounds is the extent of a profile's outer boundary.
func profileBounds(p *sketch.Profile) box {
	b := box{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	for _, e := range p.Outer {
		for _, pt := range e.Polyline {
			b.x0, b.x1 = math.Min(b.x0, pt[0]), math.Max(b.x1, pt[0])
			b.y0, b.y1 = math.Min(b.y0, pt[1]), math.Max(b.y1, pt[1])
		}
	}
	return b
}

func textSpec(text string, height float64) decad.TextSpec {
	return decad.TextSpec{Text: text, Height: units.Millimeters(height)}
}

func TestTextProfilesOfACapitalIIsItsGlyphBox(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), textSpec("I", 20))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	glyph, _ := glyphBox(t, 'I')
	_, upm := goFont(t)
	scale := 20 / upm
	got := profileBounds(profiles[0])
	require.InDelta(t, glyph.x0*scale, got.x0, 1e-9)
	require.InDelta(t, glyph.y0*scale, got.y0, 1e-9)
	require.InDelta(t, glyph.x1*scale, got.x1, 1e-9)
	require.InDelta(t, glyph.y1*scale, got.y1, 1e-9)
	// Go Regular's I is drawn with straight segments alone, so its area is the
	// shoelace area of the font's own polygon.
	require.InDelta(t, straightGlyphArea(t, 'I')*scale*scale, profiles[0].Area, 1e-6)
	require.Empty(t, profiles[0].Holes)
}

func TestTextProfilesKeepsALettersCounterAsAHole(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), textSpec("O", 20))
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 2, "the sketch holds the ring and the counter inside it")
	require.Len(t, profiles, 1, "only the ring is a filled region")
	require.Len(t, profiles[0].Holes, 1)
	glyph, _ := glyphBox(t, 'O')
	_, upm := goFont(t)
	scale := 20 / upm
	got := profileBounds(profiles[0])
	require.InDelta(t, (glyph.x1-glyph.x0)*scale, got.x1-got.x0, 0.02, "the polygon spans the glyph's width within the tolerance")
	require.InDelta(t, (glyph.y1-glyph.y0)*scale, got.y1-got.y0, 0.02)
}

func TestTextProfilesSetsLettersAdvanceApart(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), textSpec("II", 20))
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	_, upm := goFont(t)
	_, advance := glyphBox(t, 'I')
	a, b := profileBounds(profiles[0]).x0, profileBounds(profiles[1]).x0
	require.InDelta(t, advance*20/upm, math.Abs(a-b), 1e-9, "the second I starts one advance after the first")
}

func TestTextProfilesBreaksLinesAndPlacesTheOrigin(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	spec := textSpec("I\nI", 20)
	spec.At = decad.Point2{U: 100, V: 50}
	_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), spec)
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	f, upm := goFont(t)
	var buf sfnt.Buffer
	metrics, err := f.Metrics(&buf, fixed.I(int(upm)), font.HintingNone)
	require.NoError(t, err)
	lineHeight := float64(metrics.Height) / 64 * 20 / upm
	first, second := profileBounds(profiles[0]), profileBounds(profiles[1])
	require.InDelta(t, lineHeight, math.Abs(first.y0-second.y0), 1e-9, "the lines are one line height apart")
	glyph, _ := glyphBox(t, 'I')
	require.InDelta(t, 100+glyph.x0*20/upm, math.Min(first.x0, second.x0), 1e-9, "the origin moves the baseline start")
	require.InDelta(t, 50+glyph.y0*20/upm, math.Max(first.y0, second.y0), 1e-9, "the first line sits on the origin's baseline")
}

func TestTextProfilesSkipsSpaces(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), textSpec("I I", 20))
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	_, upm := goFont(t)
	_, ia := glyphBox(t, 'I')
	_, sa := glyphBox(t, ' ')
	a, b := profileBounds(profiles[0]).x0, profileBounds(profiles[1]).x0
	require.InDelta(t, (ia+sa)*20/upm, math.Abs(a-b), 1e-9, "a space moves the pen by its advance")
}

func TestTextProfilesEmbossesAndEngraves(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := decadtest.NewBlock(t, doc, 0, 0, 60, 30, units.Millimeters(10))
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	spec := textSpec("HI", 12)
	spec.At = decad.Point2{U: 8, V: 10}
	s, profiles, err := decad.TextProfiles(t.Context(), w, plane, spec)
	require.NoError(t, err)
	area := 0.0
	for _, p := range profiles {
		area += p.Area
	}
	require.Positive(t, area)
	raised, err := decad.Emboss(t.Context(), plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, units.Millimeters(2))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, raised, units.CubicMillimeters(60*30*10+2*area), decadtest.WithinRel(units.Scalar(1e-6)))
	// An engraving of the same letters in a second plate removes the same volume.
	plate2 := decadtest.NewBlock(t, decad.New(), 0, 0, 60, 30, units.Millimeters(10))
	w2 := sketch.NewWorld()
	plane2, err := w2.CreateOffsetPlane(w2.XY(), 10)
	require.NoError(t, err)
	s2, profiles2, err := decad.TextProfiles(t.Context(), w2, plane2, spec)
	require.NoError(t, err)
	engraved, err := decad.Emboss(t.Context(), plate2, s2, profiles2, embossFace(true, plate2), decad.EmbossEngraved, units.Millimeters(2))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, engraved, units.CubicMillimeters(60*30*10-2*area), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestTextProfilesRefusals(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	good := textSpec("I", 10)
	for name, tc := range map[string]struct {
		edit func(*decad.TextSpec)
		want error
	}{
		"empty text":       {func(s *decad.TextSpec) { s.Text = "" }, decad.ErrDegenerate},
		"blank text":       {func(s *decad.TextSpec) { s.Text = "  \n " }, decad.ErrDegenerate},
		"zero height":      {func(s *decad.TextSpec) { s.Height = units.Millimeters(0) }, decad.ErrDegenerate},
		"negative height":  {func(s *decad.TextSpec) { s.Height = units.Millimeters(-1) }, decad.ErrNegativeMagnitude},
		"height as angle":  {func(s *decad.TextSpec) { s.Height = units.Degrees(1) }, decad.ErrUnitKind},
		"negative tol":     {func(s *decad.TextSpec) { s.Tolerance = units.Millimeters(-1) }, decad.ErrNegativeMagnitude},
		"a rune with none": {func(s *decad.TextSpec) { s.Text = "I\U0001F600" }, decad.ErrDegenerate},
		"font that is not": {func(s *decad.TextSpec) { s.Font = []byte("not a font") }, decad.ErrDegenerate},
	} {
		spec := good
		tc.edit(&spec)
		_, _, err := decad.TextProfiles(t.Context(), w, w.XY(), spec)
		require.ErrorIs(t, err, tc.want, name)
	}
	_, _, err := decad.TextProfiles(t.Context(), nil, w.XY(), good)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.TextProfiles(t.Context(), w, nil, good)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
