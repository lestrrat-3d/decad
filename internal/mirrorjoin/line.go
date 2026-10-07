package mirrorjoin

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

type (
	Point2       = sectionrecord.Point2
	CurveSegment = sectionrecord.CurveSegment
	LineSeg      = sectionrecord.LineSeg
	ArcSeg       = sectionrecord.ArcSeg
	CircleSeg    = sectionrecord.CircleSeg
	LoopRecord   = sectionrecord.LoopRecord
)

// Region names the selected walls in each recorded loop.
type Region struct {
	Loops []LoopRecord
	Sel   []map[int]struct{}
}

// Line is the join's mirror line in the section's plane: the carrier of
// the first selected wall, oriented along that wall's walk, so the material
// lies on its left. Every coordinate is held as an exact rational.
type Line struct {
	pu, pv *big.Rat // the first wall's walk start
	du, dv *big.Rat // its walk direction, end minus start
	dd     *big.Rat // du² + dv²
}

func ratOf(v float64) *big.Rat { return proofarith.FloatRat(v) }

// lineWalkEnds is a whole line's walk: Start to End, or End to Start when its
// range runs backwards.
func lineWalkEnds(s LineSeg) (Point2, Point2) {
	if s.TStart > s.TEnd {
		return s.End, s.Start
	}
	return s.Start, s.End
}

// AdmitLine runs J3 and J4 over the selected walls and returns the
// line they lie on. J3: every wall's two recorded endpoints lie on the first
// wall's carrier, decided by exact rational cross products. J4: every wall is
// whole.
func AdmitLine(walls []LineSeg) (Line, error) {
	first := walls[0]
	start, end := lineWalkEnds(first)
	l := Line{pu: ratOf(start.U), pv: ratOf(start.V)}
	eu, ev := ratOf(end.U), ratOf(end.V)
	l.du = new(big.Rat).Sub(eu, l.pu)
	l.dv = new(big.Rat).Sub(ev, l.pv)
	l.dd = proofbound.RatAdd(proofbound.RatMul(l.du, l.du), proofbound.RatMul(l.dv, l.dv))
	if l.dd.Sign() == 0 {
		return Line{}, fmt.Errorf(`%w: the selected wall has zero length and names no mirror line`, decaderr.ErrDegenerate)
	}
	for i, s := range walls {
		if l.Side(s.Start) != 0 || l.Side(s.End) != 0 {
			return Line{}, fmt.Errorf(`%w: selected wall %d does not lie on the first selected wall's line, so the selection names no single mirror plane (J3)`, decaderr.ErrDegenerate, i)
		}
	}
	for i, s := range walls {
		if !prismcells.WholeSegmentRange(s.TStart, s.TEnd) {
			return Line{}, fmt.Errorf(`%w: selected wall %d is a fragment of a longer recorded line, and the join mirrors only a wall the record states whole (J4)`, decaderr.ErrUnsupported, i)
		}
	}
	return l, nil
}

// cross is D × (X − P), exactly: positive on the material side, zero on the
// line.
func (l Line) cross(p Point2) *big.Rat {
	xu, xv := ratOf(p.U), ratOf(p.V)
	ru := new(big.Rat).Sub(xu, l.pu)
	rv := new(big.Rat).Sub(xv, l.pv)
	return new(big.Rat).Sub(proofbound.RatMul(l.du, rv), proofbound.RatMul(l.dv, ru))
}

// side is the sign of cross.
func (l Line) Side(p Point2) int { return l.cross(p).Sign() }

// circleOnSide reports whether the whole circle about c of squared radius r2
// lies in the closed material half-plane: c on the material side, and its
// distance to the line, cross/|D|, at least the radius. Both sides are
// squared, so the test is exact.
func (l Line) circleOnSide(c Point2, r2 *big.Rat) bool {
	x := l.cross(c)
	if x.Sign() < 0 {
		return false
	}
	return proofbound.RatMul(x, x).Cmp(proofbound.RatMul(r2, l.dd)) >= 0
}

// reflect returns the image of p across the line, each coordinate computed
// exactly and rounded to the nearest float once, beside the rounding it
// committed: the sum of the two coordinates' errors, which bounds the
// distance from the held image to the exact one. m(X) = X − 2·c/(D·D)·D⊥
// with c = D × (X − P) and D⊥ = (−dv, du); no square root is taken.
func (l Line) Reflect(p Point2) (Point2, float64, error) {
	c := l.cross(p)
	k := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), c), l.dd)
	u := new(big.Rat).Add(ratOf(p.U), new(big.Rat).Mul(k, l.dv))
	v := new(big.Rat).Sub(ratOf(p.V), new(big.Rat).Mul(k, l.du))
	hu, _ := u.Float64()
	hv, _ := v.Float64()
	if math.IsInf(hu, 0) || math.IsInf(hv, 0) {
		return Point2{}, 0, fmt.Errorf(`%w: a mirrored coordinate overflows a float`, decaderr.ErrNotFinite)
	}
	held := Point2{U: hu, V: hv}
	eu, ev := proofarith.RationalFloatError(u, hu), proofarith.RationalFloatError(v, hv)
	switch {
	case eu == 0:
		// A coordinate that rounded nothing adds nothing, so a line parallel
		// to an axis charges the other coordinate's rounding exactly.
		return held, ev, nil
	case ev == 0:
		return held, eu, nil
	default:
		return held, proofbound.AbsSumUpper(eu, ev), nil
	}
}

// admitRegions runs J5 and J6 over every region, in that order. J5: every
// selected wall walks the first wall's way (so the material lies on one
// side), every region's outer loop holds a selected wall, every other segment
// is a line, arc or circle in the closed material half-plane (an arc or
// circle with its whole circle there), and a segment end on the line is the
// junction with a selected wall. J6: no arc or circle is recorded over a
// narrowed range.
func (l Line) AdmitRegions(budget *proofbound.WorkBudget, regions []Region) error {
	work := freeform.NewFreeformWork()
	for _, region := range regions {
		if len(region.Sel[0]) == 0 {
			return fmt.Errorf(`%w: the mirror line bounds no part of a region's outer loop, so the region lies across or away from it (J5)`, decaderr.ErrUnsupported)
		}
		for li, loop := range region.Loops {
			n := len(loop.Segments)
			for si, raw := range loop.Segments {
				if err := budget.Step(); err != nil {
					return err
				}
				seg, err := sectionrecord.NormalizeSegment(raw)
				if err != nil {
					return err
				}
				if _, ok := region.Sel[li][si]; ok {
					s, ok := seg.(LineSeg)
					if !ok {
						return fmt.Errorf(`%w: a selected wall is a %T, and a curved wall names no mirror plane`, decaderr.ErrDegenerate, seg)
					}
					start, end := lineWalkEnds(s)
					d := proofbound.RatAdd(
						proofbound.RatMul(new(big.Rat).Sub(ratOf(end.U), ratOf(start.U)), l.du),
						proofbound.RatMul(new(big.Rat).Sub(ratOf(end.V), ratOf(start.V)), l.dv))
					if d.Sign() <= 0 {
						return fmt.Errorf(`%w: selected walls on one line bound material on both of its sides (J5)`, decaderr.ErrUnsupported)
					}
					continue
				}
				ends, err := l.AdmitSegment(seg, work)
				if err != nil {
					return fmt.Errorf(`loop %d segment %d: %w`, li, si, err)
				}
				_, prevSel := region.Sel[li][(si+n-1)%n]
				_, nextSel := region.Sel[li][(si+1)%n]
				if ends.Closed {
					continue
				}
				if l.Side(ends.Start) == 0 && !prevSel {
					return fmt.Errorf(`%w: loop %d segment %d starts on the mirror line away from a selected wall (J5)`, decaderr.ErrUnsupported, li, si)
				}
				if l.Side(ends.End) == 0 && !nextSel {
					return fmt.Errorf(`%w: loop %d segment %d ends on the mirror line away from a selected wall (J5)`, decaderr.ErrUnsupported, li, si)
				}
			}
		}
	}
	for _, region := range regions {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, region.Loops[0], region.Loops[1:])
		if err != nil {
			return err
		}
		if trimmed {
			return fmt.Errorf(`%w: an arc or circle of the receiver is recorded over a narrowed range (J6)`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// Ends is a segment's walk ends as the join reads them; ComputedStart
// and ComputedEnd mark an end the evaluator computed (a narrowed line's
// walked end) rather than one the record states.
type Ends struct {
	Start, End                 Point2
	ComputedStart, ComputedEnd bool
	Closed                     bool
}

// SegmentEnds reads seg's walk ends. A whole line's or arc's are its own
// recorded points; a narrowed line's come from walkOf, which evaluates the
// carrier at the recorded parameter.
func SegmentEnds(seg CurveSegment, work *freeform.FreeformWork) (Ends, error) {
	switch s := seg.(type) {
	case LineSeg:
		if prismcells.WholeSegmentRange(s.TStart, s.TEnd) {
			start, end := lineWalkEnds(s)
			return Ends{Start: start, End: end}, nil
		}
		w, err := boundarywalk.WalkOf(s, work)
		if err != nil {
			return Ends{}, err
		}
		natural := func(t float64) bool { return t == 0 || t == 1 }
		return Ends{
			Start:         Point2{U: w.StartU, V: w.StartV},
			End:           Point2{U: w.EndU, V: w.EndV},
			ComputedStart: !natural(s.TStart),
			ComputedEnd:   !natural(s.TEnd),
		}, nil
	case ArcSeg:
		if s.TStart > s.TEnd {
			return Ends{Start: s.End, End: s.Start}, nil
		}
		return Ends{Start: s.Start, End: s.End}, nil
	case CircleSeg:
		return Ends{Closed: true}, nil
	default:
		return Ends{}, fmt.Errorf(`%w: a %T segment has no exact mirror image (J5)`, decaderr.ErrUnsupported, seg)
	}
}

// AdmitSegment is J5's side test for one unselected segment: every point the
// test reads must have a non-negative exact cross product. A line's two
// recorded carrier ends bound its walk by convexity, and a narrowed line's
// walked ends are read too, since the build uses them. An arc or circle needs
// its whole circle on the side; an arc's squared radius is the larger of its
// two recorded radii, so a three-point arc whose ends sit at slightly
// different radii is tested at the farther one.
func (l Line) AdmitSegment(seg CurveSegment, work *freeform.FreeformWork) (Ends, error) {
	ends, err := SegmentEnds(seg, work)
	if err != nil {
		return Ends{}, err
	}
	wrongSide := fmt.Errorf(`%w: a boundary segment reaches across the mirror line, which the join would have to arrange (J5)`, decaderr.ErrUnsupported)
	switch s := seg.(type) {
	case LineSeg:
		for _, p := range []Point2{s.Start, s.End, ends.Start, ends.End} {
			if l.Side(p) < 0 {
				return Ends{}, wrongSide
			}
		}
	case ArcSeg:
		if l.Side(s.Start) < 0 || l.Side(s.End) < 0 {
			return Ends{}, wrongSide
		}
		r2 := slices.MaxFunc([]*big.Rat{sqDist(s.Start, s.Center), sqDist(s.End, s.Center)}, func(a, b *big.Rat) int { return a.Cmp(b) })
		if !l.circleOnSide(s.Center, r2) {
			return Ends{}, wrongSide
		}
	case CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return Ends{}, fmt.Errorf(`%w: a circle radius is not a length: %s`, decaderr.ErrUnsupported, err)
		}
		rr := ratOf(r)
		if rr == nil || !l.circleOnSide(s.Center, proofbound.RatMul(rr, rr)) {
			return Ends{}, wrongSide
		}
	}
	return ends, nil
}

// sqDist is |a − b|², exactly.
func sqDist(a, b Point2) *big.Rat {
	du := new(big.Rat).Sub(ratOf(a.U), ratOf(b.U))
	dv := new(big.Rat).Sub(ratOf(a.V), ratOf(b.V))
	return proofbound.RatAdd(proofbound.RatMul(du, du), proofbound.RatMul(dv, dv))
}

// ReverseRun is a run of segments' image across the line walked back:
// rewindLoop's rule (internal/prismcells/rewind.go) under the exact reflection, so the
// image of the run's last segment comes first and walks from the run's end
// back to its start. Per kind, a line becomes the whole line from m(walk end)
// to m(walk start), an arc {C, S, E} becomes {m(C), m(E), m(S)} over the same
// range, and a circle keeps its radius, CCW flag and range about m(C): the
// reflection reverses its winding and the reversed walk reverses it back.
//
// It returns the images beside how far any point of a held image can sit from
// the exact image of the segment's denoted walk: the largest point rounding,
// tripled when the run holds an arc (its centre and radius both move with the
// rounding of its three points, the argument offsetSectionDelta states for a
// recorded arc), plus the walk charge of any narrowed line, whose walked
// endpoints were computed.
func (l Line) ReverseRun(budget *proofbound.WorkBudget, run []CurveSegment) ([]CurveSegment, float64, error) {
	pointMax := 0.0
	reflect := func(p Point2) (Point2, error) {
		m, e, err := l.Reflect(p)
		pointMax = math.Max(pointMax, e)
		return m, err
	}
	img, walk, err := prismcells.RewindLoop(budget, LoopRecord{Segments: run}, reflect)
	if err != nil {
		return nil, 0, err
	}
	for _, seg := range run {
		if _, ok := seg.(ArcSeg); ok {
			pointMax = proofbound.ProductUpper(3, pointMax)
			break
		}
	}
	if walk == 0 {
		return img.Segments, pointMax, nil
	}
	return img.Segments, proofbound.AbsSumUpper(pointMax, walk), nil
}
