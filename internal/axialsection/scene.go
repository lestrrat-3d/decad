package axialsection

import (
	"fmt"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// scene builds sketch entities from recorded segments, one entity per distinct
// recorded curve across every section, so two sections that draw the same edge
// share it.
type scene struct {
	s        *sketch.Sketch
	points   map[sectionrecord.Point2]*sketch.Point
	entities map[string]sketch.Entity
}

func newScene(s *sketch.Sketch) *scene {
	return &scene{s: s, points: map[sectionrecord.Point2]*sketch.Point{}, entities: map[string]sketch.Entity{}}
}

func (b *scene) point(p sectionrecord.Point2) *sketch.Point {
	if q, ok := b.points[p]; ok {
		return q
	}
	q := b.s.CreatePoint(p.U, p.V)
	b.s.Fix(q)
	b.points[p] = q
	return q
}

// entity returns the sketch entity a whole recorded segment denotes, creating it
// on first use. ok is false for a kind this builder does not create.
func (b *scene) entity(seg sectionrecord.CurveSegment) (sketch.Entity, bool, error) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		key := fmt.Sprintf("L%v|%v", s.Start, s.End)
		if e, ok := b.entities[key]; ok {
			return e, true, nil
		}
		e := b.s.CreateLine(b.point(s.Start), b.point(s.End))
		b.entities[key] = e
		return e, true, nil
	case sectionrecord.ArcSeg:
		key := fmt.Sprintf("A%v|%v|%v", s.Center, s.Start, s.End)
		if e, ok := b.entities[key]; ok {
			return e, true, nil
		}
		e := b.s.CreateArc(b.point(s.Center), b.point(s.Start), b.point(s.End))
		b.entities[key] = e
		return e, true, nil
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false, err
		}
		key := fmt.Sprintf("C%v|%v", s.Center, r)
		if e, ok := b.entities[key]; ok {
			return e, true, nil
		}
		e := b.s.CreateCircle(b.point(s.Center), r)
		b.entities[key] = e
		return e, true, nil
	case sectionrecord.FitSplineSeg:
		key := fmt.Sprintf("F%v", s.Fit)
		if e, ok := b.entities[key]; ok {
			return e, true, nil
		}
		pts := make([]*sketch.Point, len(s.Fit))
		for i, p := range s.Fit {
			pts[i] = b.point(p)
		}
		e, err := b.s.CreateFitSpline(pts...)
		if err != nil {
			return nil, false, err
		}
		b.entities[key] = e
		return e, true, nil
	}
	return nil, false, nil
}
