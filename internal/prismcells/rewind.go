package prismcells

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RewindLoop reverses one recorded loop under a point map. Narrowed lines
// enter as whole segments between their walked endpoints, with the same
// walk charge the prism scene records. The point map owns its rounding charge.
func RewindLoop(budget *proofbound.WorkBudget, loop LoopRecord,
	mapPoint func(sectionrecord.Point2) (sectionrecord.Point2, error)) (LoopRecord, float64, error) {
	charge := 0.0
	n := len(loop.Segments)
	out := make([]CurveSegment, n)
	for i, seg := range loop.Segments {
		if err := budget.Step(); err != nil {
			return LoopRecord{}, 0, err
		}
		pts := func(in ...sectionrecord.Point2) ([]sectionrecord.Point2, error) {
			mapped := make([]sectionrecord.Point2, len(in))
			for k, p := range in {
				m, err := mapPoint(p)
				if err != nil {
					return nil, err
				}
				mapped[k] = m
			}
			return mapped, nil
		}
		var mapped CurveSegment
		switch s := seg.(type) {
		case LineSeg:
			start, end, tStart, tEnd := s.Start, s.End, s.TStart, s.TEnd
			if !WholeSegmentRange(tStart, tEnd) {
				w, err := boundarywalk.WalkOf(s, nil)
				if err != nil {
					return LoopRecord{}, 0, err
				}
				c, err := WalkChargeOf(s, w)
				if err != nil {
					return LoopRecord{}, 0, err
				}
				charge = math.Max(charge, c)
				start, end = sectionrecord.Point2{U: w.StartU, V: w.StartV},
					sectionrecord.Point2{U: w.EndU, V: w.EndV}
				tStart, tEnd = 0, 1
			}
			p, err := pts(end, start)
			if err != nil {
				return LoopRecord{}, 0, err
			}
			mapped = LineSeg{Start: p[0], End: p[1], TStart: tStart, TEnd: tEnd}
		case ArcSeg:
			if !WholeSegmentRange(s.TStart, s.TEnd) {
				return LoopRecord{}, 0, fmt.Errorf(`%w: a reflected operand's trimmed arc cannot be re-wound`, decaderr.ErrUnsupported)
			}
			p, err := pts(s.Center, s.End, s.Start)
			if err != nil {
				return LoopRecord{}, 0, err
			}
			mapped = ArcSeg{Center: p[0], Start: p[1], End: p[2], TStart: s.TStart, TEnd: s.TEnd}
		case CircleSeg:
			if !WholeSegmentRange(s.TStart, s.TEnd) {
				return LoopRecord{}, 0, fmt.Errorf(`%w: a reflected operand's trimmed circle cannot be re-wound`, decaderr.ErrUnsupported)
			}
			p, err := pts(s.Center)
			if err != nil {
				return LoopRecord{}, 0, err
			}
			mapped = CircleSeg{Center: p[0], Radius: s.Radius, CCW: s.CCW, TStart: s.TStart, TEnd: s.TEnd}
		default:
			return LoopRecord{}, 0, fmt.Errorf(`%w: a %T segment is not part of the admitted class`, decaderr.ErrUnsupported, seg)
		}
		out[n-1-i] = mapped
	}
	return LoopRecord{Segments: out}, charge, nil
}
