package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"sort"
	"strings"

	"github.com/lestrrat-3d/decad/internal/glyphs"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// TextSpec describes a line of lettering for [TextProfiles].
//
// Text is set in the font as written: runes of a line follow one another by the
// font's advance widths with no kerning, and "\n" starts the next line one line
// height below. Height is the font size, the length of one em, a positive
// length: a capital letter of the default font is about 0.7 of it tall. At is the
// baseline's start of the first line in the sketch plane's coordinates. Tolerance
// is optional: it is how far a flattened curve may lie from its polygon, a
// positive length, and the zero units.Value means a thousandth of Height. Font is
// optional TrueType font data, and nil sets the text in Go Regular.
type TextSpec struct {
	Text      string
	Height    units.Value
	At        Point2
	Tolerance units.Value
	Font      []byte
	// Arc, when set, sets the first line along a circular arc instead of a
	// straight baseline (agent-docs/text-design.md §6). At is then unused.
	Arc *TextArc
}

// TextArc is a circle to set lettering along. The baseline runs along the circle
// of the given Radius about Center from the angle Start, which is measured
// counter-clockwise from the plane's U axis. With Clockwise set the letters run
// clockwise with their tops outward, as text reads along the top of a badge; with
// it clear they run counter-clockwise with their tops inward, as it reads along
// the bottom. Each letter keeps its own shape and is turned as a whole to stand
// on the circle at the middle of its advance, so letters are not bent: a letter
// much wider than the circle is large leaves a gap or a wedge between its
// neighbours. Radius is a positive length and Start an angle. The lettering must
// be one line.
type TextArc struct {
	Center    Point2
	Radius    units.Value
	Start     units.Value
	Clockwise bool
}

// TextProfiles draws the outlines of a line of lettering into a new sketch on
// plane of w and returns the sketch with the closed profiles of its filled
// regions, so that [Emboss] can raise or engrave them
// (agent-docs/text-design.md).
//
// Each glyph's curves are flattened into straight lines within the spec's
// tolerance, so a round letter is a polygon. A letter with a counter, as 'o' has,
// is one profile with a hole, and the counter itself is not returned: only the
// regions the font fills are. Draw the lettering on the plane of the face it is
// to mark, which Emboss requires of its sketch.
//
// ctx, w and plane MUST NOT be nil, each otherwise ErrDegenerate. A Height or
// Tolerance that is not a positive length is ErrDegenerate, ErrNegativeMagnitude
// or ErrUnitKind. Text that is empty, draws no outline, holds a rune the font
// has no glyph for, needs more than 200000 outline points or whose glyphs touch
// so that a region is not a valid profile is ErrDegenerate, as is a font that
// does not parse.
func TextProfiles(ctx context.Context, w *sketch.World, plane *sketch.Plane, spec TextSpec) (*sketch.Sketch, []*sketch.Profile, error) {
	if ctx == nil {
		return nil, nil, fmt.Errorf(`%w: a nil context cannot control lettering`, ErrDegenerate)
	}
	if w == nil || plane == nil {
		return nil, nil, fmt.Errorf(`%w: lettering needs a world and a plane`, ErrDegenerate)
	}
	height, err := sectionrecord.MagnitudeIn(spec.Height, units.Length, units.Millimeter, "the text height")
	if err != nil {
		return nil, nil, err
	}
	tolerance := height / 1000
	if spec.Tolerance != (units.Value{}) {
		if tolerance, err = sectionrecord.MagnitudeIn(spec.Tolerance, units.Length, units.Millimeter, "the text tolerance"); err != nil {
			return nil, nil, err
		}
	}
	ttf := spec.Font
	if ttf == nil {
		ttf = glyphs.DefaultFont()
	}
	if strings.TrimSpace(spec.Text) == "" {
		return nil, nil, fmt.Errorf(`%w: there is no text to set`, ErrDegenerate)
	}
	laid, err := glyphs.Layout(ttf, spec.Text, height, tolerance)
	if errors.Is(err, glyphs.ErrDegenerate) {
		return nil, nil, fmt.Errorf(`%w: %s`, ErrDegenerate, err)
	}
	if err != nil {
		return nil, nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, nil, err
	}
	place := func(i int, p [2]float64) (float64, float64) { return spec.At.U + p[0], spec.At.V + p[1] }
	if spec.Arc != nil {
		if place, err = arcPlacement(spec.Arc, laid); err != nil {
			return nil, nil, err
		}
	}
	for i, contour := range laid.Contours {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, err
			}
		}
		points := make([]*sketch.Point, len(contour))
		for k, p := range contour {
			u, v := place(i, p)
			points[k] = s.CreatePoint(u, v)
			s.Fix(points[k])
		}
		for k := range points {
			s.CreateLine(points[k], points[(k+1)%len(points)])
		}
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, nil, err
	}
	profiles, err := filledProfiles(s.Profiles())
	if err != nil {
		return nil, nil, err
	}
	return s, profiles, nil
}

// filledProfiles keeps the profiles the font fills. A region that lies in
// another's hole, as the counter of an 'o' does, is empty, and one in the hole of
// an empty region is filled again: a region is filled when an even number of
// regions enclose it.
func filledProfiles(all []*sketch.Profile) ([]*sketch.Profile, error) {
	parent := map[string]int{}
	for i, p := range all {
		for _, hole := range p.Holes {
			parent[boundaryKey(hole)] = i
		}
	}
	depth := func(i int) int {
		d := 0
		for {
			up, ok := parent[boundaryKey(all[i].Outer)]
			if !ok || d > len(all) {
				return d
			}
			d++
			i = up
		}
	}
	var out []*sketch.Profile
	for i, p := range all {
		if depth(i)%2 != 0 {
			continue
		}
		if !p.Valid || p.SelfIntersecting {
			return nil, fmt.Errorf(`%w: letters touch or overlap, so a region of the lettering is not a valid profile`, ErrDegenerate)
		}
		out = append(out, p)
	}
	if len(out) == 0 {
		return nil, fmt.Errorf(`%w: the lettering has no filled region`, ErrDegenerate)
	}
	return out, nil
}

// boundaryKey names a boundary loop by the sketch entities on it, so the same
// loop reads the same from the region inside it and from the region around it.
func boundaryKey(loop []sketch.BoundaryEdge) string {
	names := make([]string, len(loop))
	for i, e := range loop {
		names[i] = fmt.Sprintf("%p", e.Entity)
	}
	sort.Strings(names)
	return strings.Join(slices.Compact(names), ",")
}

// arcPlacement returns the map from contour i's laid-out point to the plane
// along arc: the glyph the contour belongs to is turned to stand on the circle at
// the middle of its advance, its baseline tangent to the circle there.
func arcPlacement(arc *TextArc, laid glyphs.Result) (func(int, [2]float64) (float64, float64), error) {
	radius, err := sectionrecord.MagnitudeIn(arc.Radius, units.Length, units.Millimeter, "the text arc radius")
	if err != nil {
		return nil, err
	}
	if radius == 0 {
		return nil, fmt.Errorf(`%w: the text arc radius must be positive`, ErrDegenerate)
	}
	start, err := sectionrecord.MagnitudeIn(arc.Start, units.Angle, units.Radian, "the text arc start")
	if err != nil {
		return nil, err
	}
	for _, g := range laid.Glyphs {
		if g.Line != 0 {
			return nil, fmt.Errorf(`%w: lettering along an arc is one line`, ErrDegenerate)
		}
	}
	sign := -1.0 // clockwise runs the angle down
	if !arc.Clockwise {
		sign = 1
	}
	return func(i int, p [2]float64) (float64, float64) {
		g := laid.Glyphs[laid.ContourGlyph[i]]
		mid := (g.Start + g.End) / 2
		theta := start + sign*mid/radius
		sin, cos := math.Sincos(theta)
		// The tangent along the text and the way up: outward for clockwise text, inward for
		// counter-clockwise, so the pair is right-handed and no letter is mirrored.
		tx, ty := sign*-sin, sign*cos
		ux, uy := cos, sin
		if !arc.Clockwise {
			ux, uy = -cos, -sin
		}
		lx, ly := p[0]-mid, p[1]
		return arc.Center.U + radius*cos + lx*tx + ly*ux, arc.Center.V + radius*sin + lx*ty + ly*uy
	}, nil
}
