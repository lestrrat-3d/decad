package patternrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type (
	LoopRecord   = sectionrecord.LoopRecord
	CurveSegment = sectionrecord.CurveSegment
	LineSeg      = sectionrecord.LineSeg
	ArcSeg       = sectionrecord.ArcSeg
	CircleSeg    = sectionrecord.CircleSeg
)

// Region is the recorded outer loop and holes before payload construction.
type Region struct {
	Outer LoopRecord
	Holes []LoopRecord
}

// MoveRegion moves every segment of a region by mv, keeping each segment's
// kind, sense and range: a translation or a proper rotation changes neither.
// It returns the moved region beside the largest segment charge: a line's
// larger endpoint charge, a circle's centre charge, and an arc's largest
// point charge tripled, since its centre and radius both move with its three
// points' rounding (the argument offsetSectionDelta states for a recorded
// arc).
func MoveRegion(budget *proofbound.WorkBudget, region Region, mv PointMotion) (Region, float64, error) {
	delta := 0.0
	moveLoop := func(loop LoopRecord) (LoopRecord, error) {
		out := make([]CurveSegment, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return LoopRecord{}, err
			}
			pts := func(in ...Point2) ([]Point2, float64, error) {
				moved := make([]Point2, len(in))
				worst := 0.0
				for k, p := range in {
					m, e, err := mv(p)
					if err != nil {
						return nil, 0, err
					}
					moved[k], worst = m, math.Max(worst, e)
				}
				return moved, worst, nil
			}
			switch s := seg.(type) {
			case LineSeg:
				p, e, err := pts(s.Start, s.End)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Start, s.End = p[0], p[1]
				out[i], delta = s, math.Max(delta, e)
			case ArcSeg:
				p, e, err := pts(s.Center, s.Start, s.End)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Center, s.Start, s.End = p[0], p[1], p[2]
				out[i], delta = s, math.Max(delta, proofbound.ProductUpper(3, e))
			case CircleSeg:
				p, e, err := pts(s.Center)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Center = p[0]
				out[i], delta = s, math.Max(delta, e)
			default:
				return LoopRecord{}, fmt.Errorf(`%w: a %T segment has no exact pattern motion`, decaderr.ErrUnsupported, seg)
			}
		}
		return LoopRecord{Segments: out}, nil
	}
	outer, err := moveLoop(region.Outer)
	if err != nil {
		return Region{}, 0, err
	}
	out := Region{Outer: outer}
	for _, hole := range region.Holes {
		moved, err := moveLoop(hole)
		if err != nil {
			return Region{}, 0, err
		}
		out.Holes = append(out.Holes, moved)
	}
	return out, delta, nil
}

// WithDelta adds an instance's motion charge to the receiver's own
// section displacement; a zero charge keeps it bit for bit.
func WithDelta(sectionDelta, delta float64) float64 {
	if delta == 0 {
		return sectionDelta
	}
	return proofbound.AbsSumUpper(sectionDelta, delta)
}
