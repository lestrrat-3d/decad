package decad

import (
	"context"
	"errors"
	"fmt"
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
	for i, contour := range laid.Contours {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, err
			}
		}
		points := make([]*sketch.Point, len(contour))
		for k, p := range contour {
			points[k] = s.CreatePoint(spec.At.U+p[0], spec.At.V+p[1])
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
