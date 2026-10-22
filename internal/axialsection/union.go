// Package axialsection takes the 2D Boolean of recorded sections, so a Boolean of
// axial extrusions of those sections is the extrusion of the result
// (agent-docs/axial-section-boolean-design.md).
package axialsection

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/sketch"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Union returns the union of the hole-free sections as one recorded profile. Every
// section is a closed outer loop of whole edges. ok is false, with a nil error, for
// sections it does not take (a hole, a trimmed segment, a kind the scene builder does
// not create) and for a union sketch does not certify (a region whose containment is
// unproven, a union sketch declines to join, or a fragment it reports inexact), so the
// caller takes its next route. An error is a failure of the scene itself.
func Union(ctx context.Context, sections []momentinput.Profile) (momentinput.Profile, bool, error) {
	profile, _, ok, err := unionArea(ctx, sections)
	return profile, ok, err
}

// unionArea is Union with the area sketch reports for the joined region.
func unionArea(ctx context.Context, sections []momentinput.Profile) (momentinput.Profile, float64, bool, error) {
	if len(sections) < 2 {
		return momentinput.Profile{}, 0, false, nil
	}
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return momentinput.Profile{}, 0, false, err
	}
	b := newScene(s)
	loops := make([][]sketch.Entity, len(sections))
	for i, sec := range sections {
		if len(sec.Holes) != 0 {
			return momentinput.Profile{}, 0, false, nil
		}
		for _, seg := range sec.Outer.Segments {
			if !whole(seg) {
				return momentinput.Profile{}, 0, false, nil
			}
			e, ok, err := b.entity(seg)
			if err != nil || !ok {
				return momentinput.Profile{}, 0, false, err
			}
			loops[i] = append(loops[i], e)
		}
	}
	for i, es := range loops {
		if _, err := s.DeclareLoop(es...); err != nil {
			return momentinput.Profile{}, 0, false, fmt.Errorf("%w: section %d is not a loop of distinct entities: %v",
				decaderr.ErrUnsupported, i, err)
		}
	}
	if err := ctx.Err(); err != nil {
		return momentinput.Profile{}, 0, false, err
	}
	var chosen []int
	for i, p := range s.Profiles() {
		if !p.Valid {
			continue
		}
		inside, unproven := false, ""
		for _, c := range p.Containments() {
			switch c.State {
			case sketch.ContainmentInside:
				inside = true
			case sketch.ContainmentUnproven:
				unproven = fmt.Sprint(c.Reason)
			}
		}
		switch {
		case inside:
			chosen = append(chosen, i)
		case unproven != "":
			// Sketch cannot say whether this region is inside any operand.
			return momentinput.Profile{}, 0, false, nil
		}
	}
	if len(chosen) == 0 {
		return momentinput.Profile{}, 0, false, nil
	}
	u, err := s.UnionProfiles(chosen...)
	if err != nil {
		// Sketch declines to join these regions, such as where an edge it would
		// have to split is only partly shared (a coincident overlap).
		return momentinput.Profile{}, 0, false, nil //nolint:nilerr // a union sketch declines is a miss, not a failure
	}
	for _, e := range u.Outer {
		if e.Partial && !e.TExact {
			return momentinput.Profile{}, 0, false, nil
		}
	}
	for _, h := range u.Holes {
		for _, e := range h {
			if e.Partial && !e.TExact {
				return momentinput.Profile{}, 0, false, nil
			}
		}
	}
	// The seam gate: a fragment sketch left uncertified is refused here too.
	record, _, area, err := sectioncert.RecordProfile(s, u)
	if err != nil {
		return momentinput.Profile{}, 0, false, err
	}
	return record, area, true, nil
}

// whole reports whether seg is a whole edge walked either way.
func whole(seg sectionrecord.CurveSegment) bool {
	var t0, t1 float64
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		t0, t1 = s.TStart, s.TEnd
	case sectionrecord.CircleSeg:
		t0, t1 = s.TStart, s.TEnd
	case sectionrecord.ArcSeg:
		t0, t1 = s.TStart, s.TEnd
	case sectionrecord.FitSplineSeg:
		t0, t1 = s.TStart, s.TEnd
	default:
		return false
	}
	return math.Min(t0, t1) == 0 && math.Max(t0, t1) == 1
}
