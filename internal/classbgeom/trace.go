package classbgeom

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// TraceChord is one certified fragment of a section's trace line. Crossed
// names the section carrier at each end, in the trace's directed order.
type TraceChord[C comparable] struct {
	Ends    [2]sectionrecord.Point2
	Bounds  [2]proofbound.WalkEndBound
	Crossed [2]C
}

// TraceChords arranges a section and one line through its box. It accepts a
// partial trace fragment only when sketch certifies its exact parameter range.
// miss is the caller's refusal for a scene this crossing reach cannot record.
func TraceChords[C comparable](ctx context.Context, section []CarrierSegment[C], fixedLocal int,
	fixedValue float64, carrier C, miss error) ([]TraceChord[C], sectionrecord.LineSeg, error) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, cs := range section {
		box, err := SegmentBox(cs.Seg)
		if err != nil {
			return nil, sectionrecord.LineSeg{}, err
		}
		l, _ := box.Lo[1-fixedLocal].Float64()
		h, _ := box.Hi[1-fixedLocal].Float64()
		lo, hi = math.Min(lo, l), math.Max(hi, h)
	}
	span := hi - lo + 1
	lo, hi = lo-span, hi+span
	var a, z sectionrecord.Point2
	if fixedLocal == 0 {
		a, z = sectionrecord.Point2{U: fixedValue, V: lo}, sectionrecord.Point2{U: fixedValue, V: hi}
	} else {
		a, z = sectionrecord.Point2{U: lo, V: fixedValue}, sectionrecord.Point2{U: hi, V: fixedValue}
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, sectionrecord.LineSeg{}, err
	}
	points := map[sectionrecord.Point2]*sketch.Point{}
	point := func(q sectionrecord.Point2) *sketch.Point {
		if pt, ok := points[q]; ok {
			return pt
		}
		pt := s.CreatePoint(q.U, q.V)
		s.Fix(pt)
		points[q] = pt
		return pt
	}
	entity := map[sketch.Entity]CarrierSegment[C]{}
	for _, cs := range section {
		ent, err := CreateNaturalSegment(s, point, cs.Seg)
		if err != nil {
			return nil, sectionrecord.LineSeg{}, err
		}
		entity[ent] = cs
	}
	trace := sectionrecord.LineSeg{Start: a, End: z, TStart: 0, TEnd: 1}
	traceEnt := s.CreateLine(point(a), point(z))
	entity[traceEnt] = CarrierSegment[C]{Seg: trace, Carrier: carrier}
	profiles, err := prismcells.ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, sectionrecord.LineSeg{}, err
	}
	seen := map[[2]float64]struct{}{}
	var out []TraceChord[C]
	for _, prof := range profiles {
		if !prof.Valid || len(prof.Holes) != 0 {
			return nil, sectionrecord.LineSeg{}, miss
		}
		edges := prof.Outer
		for i, e := range edges {
			if e.Entity != traceEnt {
				continue
			}
			if e.Partial && !e.TExact {
				return nil, sectionrecord.LineSeg{}, miss
			}
			t0, t1 := math.Min(e.TStart, e.TEnd), math.Max(e.TStart, e.TEnd)
			if _, dup := seen[[2]float64{t0, t1}]; dup {
				continue
			}
			seen[[2]float64{t0, t1}] = struct{}{}
			prev, next := edges[(i+len(edges)-1)%len(edges)], edges[(i+1)%len(edges)]
			cPrev, okPrev := entity[prev.Entity]
			cNext, okNext := entity[next.Entity]
			if !okPrev || !okNext {
				return nil, sectionrecord.LineSeg{}, miss
			}
			piece, err := boundarywalk.WalkOf(sectionrecord.LineSeg{
				Start: a, End: z, TStart: e.TStart, TEnd: e.TEnd,
			}, nil)
			if err != nil {
				return nil, sectionrecord.LineSeg{}, err
			}
			out = append(out, TraceChord[C]{
				Ends: [2]sectionrecord.Point2{{U: piece.StartU, V: piece.StartV},
					{U: piece.EndU, V: piece.EndV}},
				Bounds:  [2]proofbound.WalkEndBound{piece.StartBound, piece.EndBound},
				Crossed: [2]C{cPrev.Carrier, cNext.Carrier},
			})
		}
	}
	return out, trace, nil
}
