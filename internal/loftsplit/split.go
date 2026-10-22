// Package loftsplit splits recorded profile segments at stated parameters, so two
// sections of different segment counts can be paired one to one
// (agent-docs/loft-design.md, "Splits").
package loftsplit

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Split names one recorded segment of one loop and the fractions of its
// parameter range at which to cut it. Loop 0 is the outer loop and loop 1+h is
// Holes[h]. Every fraction is strictly inside (0, 1), and they increase.
type Split struct {
	Loop    int
	Segment int
	At      []float64
}

// Apply returns p with every named segment replaced by its pieces. A piece is
// the same recorded curve over a sub-range of the segment's own parameter range,
// which is how a trimmed fragment is already recorded, so a split changes no
// curve: a fraction f resolves to the parameter TStart + f·(TEnd−TStart), and
// the point the pieces share is the point the record denotes at that float
// parameter. The walk order of the loop is kept: the pieces follow each other
// from the segment's start to its end, whichever way the range runs. Apply never
// modifies p.
func Apply(p momentinput.Profile, splits []Split) (momentinput.Profile, error) {
	loops := 1 + len(p.Holes)
	byLoop := make([]map[int][]float64, loops)
	for _, s := range splits {
		if s.Loop < 0 || s.Loop >= loops {
			return momentinput.Profile{}, fmt.Errorf(`%w: a split names loop %d of a profile with %d loops`,
				decaderr.ErrDegenerate, s.Loop, loops)
		}
		n := len(loopOf(p, s.Loop).Segments)
		if s.Segment < 0 || s.Segment >= n {
			return momentinput.Profile{}, fmt.Errorf(`%w: a split names segment %d of loop %d, which has %d segments`,
				decaderr.ErrDegenerate, s.Segment, s.Loop, n)
		}
		if len(s.At) == 0 {
			return momentinput.Profile{}, fmt.Errorf(`%w: the split of segment %d in loop %d names no parameter`,
				decaderr.ErrDegenerate, s.Segment, s.Loop)
		}
		prev := 0.0
		for _, f := range s.At {
			if !(f > prev && f < 1) {
				return momentinput.Profile{}, fmt.Errorf(`%w: the split fractions of segment %d in loop %d must increase strictly inside (0, 1); got %v`,
					decaderr.ErrDegenerate, s.Segment, s.Loop, s.At)
			}
			prev = f
		}
		if byLoop[s.Loop] == nil {
			byLoop[s.Loop] = map[int][]float64{}
		}
		if _, dup := byLoop[s.Loop][s.Segment]; dup {
			return momentinput.Profile{}, fmt.Errorf(`%w: segment %d of loop %d is split twice`,
				decaderr.ErrDegenerate, s.Segment, s.Loop)
		}
		byLoop[s.Loop][s.Segment] = s.At
	}
	out := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(p.Outer)}
	for _, h := range p.Holes {
		out.Holes = append(out.Holes, sectionrecord.CloneLoopRecord(h))
	}
	for i := range loops {
		if len(byLoop[i]) == 0 {
			continue
		}
		src := loopOf(p, i)
		var pieces []sectionrecord.CurveSegment
		for j, seg := range src.Segments {
			at, ok := byLoop[i][j]
			if !ok {
				pieces = append(pieces, seg)
				continue
			}
			cut, err := cutSegment(seg, at)
			if err != nil {
				return momentinput.Profile{}, fmt.Errorf("loop %d segment %d: %w", i, j, err)
			}
			pieces = append(pieces, cut...)
		}
		if i == 0 {
			out.Outer = sectionrecord.LoopRecord{Segments: pieces}
		} else {
			out.Holes[i-1] = sectionrecord.LoopRecord{Segments: pieces}
		}
	}
	return out, nil
}

func loopOf(p momentinput.Profile, i int) sectionrecord.LoopRecord {
	if i == 0 {
		return p.Outer
	}
	return p.Holes[i-1]
}

// cutSegment returns seg split at the given fractions of its parameter range.
func cutSegment(seg sectionrecord.CurveSegment, at []float64) ([]sectionrecord.CurveSegment, error) {
	t0, t1, ok := rangeOf(seg)
	if !ok {
		return nil, fmt.Errorf(`%w: a segment of type %T has no splittable range`, decaderr.ErrUnsupported, seg)
	}
	bounds := make([]float64, 0, len(at)+2)
	bounds = append(bounds, t0)
	for _, f := range at {
		t := t0 + f*(t1-t0)
		// A fraction that rounds onto its neighbour would leave an empty piece.
		if t == bounds[len(bounds)-1] || t == t1 {
			return nil, fmt.Errorf(`%w: the split fraction %v leaves an empty piece of the range [%v, %v]`,
				decaderr.ErrDegenerate, f, t0, t1)
		}
		bounds = append(bounds, t)
	}
	bounds = append(bounds, t1)
	out := make([]sectionrecord.CurveSegment, 0, len(bounds)-1)
	for i := 0; i+1 < len(bounds); i++ {
		out = append(out, withRange(seg, bounds[i], bounds[i+1]))
	}
	return out, nil
}

func rangeOf(seg sectionrecord.CurveSegment) (float64, float64, bool) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.CircleSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.ArcSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.EllipseSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.EllipticalArcSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.SplineSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.NURBSSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.ClosedSplineSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.FitSplineSeg:
		return s.TStart, s.TEnd, true
	case sectionrecord.ConicSeg:
		return s.TStart, s.TEnd, true
	}
	return 0, 0, false
}

// withRange returns a copy of seg over [t0, t1]. Every slice the copy holds is
// the segment's own, never shared with seg.
func withRange(seg sectionrecord.CurveSegment, t0, t1 float64) sectionrecord.CurveSegment {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.CircleSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.ArcSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.EllipseSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.EllipticalArcSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.SplineSeg:
		s.Control = append([]sectionrecord.Point2(nil), s.Control...)
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.NURBSSeg:
		s.Control = append([]sectionrecord.Point2(nil), s.Control...)
		s.Knots = append([]float64(nil), s.Knots...)
		s.Weights = append([]float64(nil), s.Weights...)
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.ClosedSplineSeg:
		s.Control = append([]sectionrecord.Point2(nil), s.Control...)
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.FitSplineSeg:
		s.Fit = append([]sectionrecord.Point2(nil), s.Fit...)
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.ConicSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	}
	return seg
}
