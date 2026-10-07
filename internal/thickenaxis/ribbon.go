package thickenaxis

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This section is the OPEN-walk counterpart of the closed offsets above: the
// ribbon and the uncapped chain shell of docs/surface-design.md §16.6 and
// §16.7. An open walk has no interior to erode, so there is no P ⊖ t / P ⊕ t
// to take. What a thickened ribbon sweeps is one CLOSED section assembled in
// closed form: the walk's two offset copies — or the walk and one copy —
// joined by one cap line at each free end, walked right copy forward, end cap,
// left copy backward, start cap. Every coordinate of that section is a
// recorded walk coordinate plus an INTEGER multiple of the offset parameter,
// so the whole section is affine in τ and the interval scan above reads it
// unchanged.

// thickenRibbonSide names one side of the walk and how far its copy sits from
// it: normal is the left-normal sign (−1 the walk's right-hand side, +1 its
// left), and steps is 0 where that copy IS the recorded walk and 1 where it is
// an offset copy.
type thickenRibbonSide struct{ normal, steps int }

// thickenOpenDirections is AxisDirections without wraparound: it reads
// one open walk's per-segment axis direction, refusing an inexact endpoint, a
// junction the two walks do not share exactly, a segment that is not
// axis-parallel, and an interior corner that is not a right angle.
func thickenOpenDirections(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]AxisDir, error) {
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: an open walk holds no segment`, decaderr.ErrUnsupported)
	}
	dirs := make([]AxisDir, n)
	for i, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.StartBound.U != 0 || w.StartBound.V != 0 || w.EndBound.U != 0 || w.EndBound.V != 0 {
			return nil, fmt.Errorf(`%w: an open walk endpoint has an unresolved coordinate bound`, decaderr.ErrUnsupported)
		}
		if i+1 < n {
			next := walks[i+1]
			if w.EndU != next.StartU || w.EndV != next.StartV {
				return nil, fmt.Errorf(`%w: the open walk has no exact interior joins`, decaderr.ErrUnsupported)
			}
		}
		switch {
		case w.StartU == w.EndU && w.StartV < w.EndV:
			dirs[i] = AxisDir{v: 1}
		case w.StartU == w.EndU && w.StartV > w.EndV:
			dirs[i] = AxisDir{v: -1}
		case w.StartV == w.EndV && w.StartU < w.EndU:
			dirs[i] = AxisDir{u: 1}
		case w.StartV == w.EndV && w.StartU > w.EndU:
			dirs[i] = AxisDir{u: -1}
		default:
			return nil, fmt.Errorf(`%w: the open walk is not axis-parallel`, decaderr.ErrUnsupported)
		}
		if proofarith.FloatRat(w.StartU) == nil || proofarith.FloatRat(w.StartV) == nil ||
			proofarith.FloatRat(w.EndU) == nil || proofarith.FloatRat(w.EndV) == nil {
			return nil, fmt.Errorf(`%w: an open walk coordinate is not finite`, decaderr.ErrUnsupported)
		}
	}
	for i := 1; i < n; i++ {
		p, q := dirs[i-1], dirs[i]
		if p.u*q.v-p.v*q.u == 0 {
			return nil, fmt.Errorf(`%w: an open walk corner is not a right angle`, decaderr.ErrUnsupported)
		}
	}
	return dirs, nil
}

// thickenRibbonCopy is one side's moving copy of the walk, in walk order: the
// offset lines and the corner arcs between them, plus the two free-end points
// the caps attach to.
type thickenRibbonCopy struct {
	pieces     []thickenMovingPiece
	head, tail thickenMovingPoint
}

// thickenRibbonCopyOf builds one copy in closed form. A corner arc appears
// exactly where the walk turns AWAY from this copy's side, the same
// sign(cross) == −s rule the closed offset takes (shell_offset.go); the other
// corner miters. A copy with steps == 0 is the recorded walk itself and takes
// neither.
func thickenRibbonCopyOf(walks []survey2d.SideWalk, dirs []AxisDir, c thickenRibbonSide) thickenRibbonCopy {
	n := len(dirs)
	k := c.steps * c.normal
	offset := func(u, v float64, d AxisDir) thickenMovingPoint {
		return thickenMovingOffset(u, v, k*-d.v, k*d.u)
	}
	type join struct {
		arc              bool
		m, before, after thickenMovingPoint
	}
	joins := make([]join, n)
	for i := 1; i < n; i++ {
		prev, cur := dirs[i-1], dirs[i]
		v := walks[i]
		joins[i] = join{
			arc:    c.steps != 0 && prev.u*cur.v-prev.v*cur.u == -c.normal,
			before: offset(v.StartU, v.StartV, prev),
			after:  offset(v.StartU, v.StartV, cur),
			m: thickenMovingOffset(v.StartU, v.StartV,
				k*(-prev.v-cur.v), k*(prev.u+cur.u)),
		}
	}
	out := thickenRibbonCopy{
		head: offset(walks[0].StartU, walks[0].StartV, dirs[0]),
		tail: offset(walks[n-1].EndU, walks[n-1].EndV, dirs[n-1]),
	}
	for i, dir := range dirs {
		start, end := out.head, out.tail
		if i > 0 {
			start = joins[i].m
			if joins[i].arc {
				start = joins[i].after
			}
		}
		if i+1 < n {
			end = joins[i+1].m
			if joins[i+1].arc {
				end = joins[i+1].before
			}
		}
		out.pieces = append(out.pieces, thickenMovingPiece{line: true, horizontal: dir.u != 0,
			start: start, end: end})
		if i+1 < n && joins[i+1].arc {
			v := walks[i+1]
			out.pieces = append(out.pieces, thickenMovingPiece{
				start: joins[i+1].before, end: joins[i+1].after,
				center: thickenExactPoint{u: proofarith.FloatRat(v.StartU), v: proofarith.FloatRat(v.StartV)},
			})
		}
	}
	return out
}

// thickenReverseMovingPiece walks one piece the other way. A line swaps its
// two ends; an arc swaps them about the same fixed centre.
func thickenReverseMovingPiece(p thickenMovingPiece) thickenMovingPiece {
	p.start, p.end = p.end, p.start
	return p
}

// thickenRibbonLoop assembles the two copies into ONE closed moving boundary
// in loop order: the right copy forward, the cap at the walk's far end, the
// left copy backward, and the cap at its near end. The order is what makes the
// interval scan's own adjacency exclusion (consecutive pieces of one loop)
// correct with no second rule.
func thickenRibbonLoop(right, left thickenRibbonCopy) []thickenMovingPiece {
	pieces := append([]thickenMovingPiece{}, right.pieces...)
	pieces = append(pieces, thickenCapPiece(right.tail, left.tail))
	for _, piece := range slices.Backward(left.pieces) {
		pieces = append(pieces, thickenReverseMovingPiece(piece))
	}
	return append(pieces, thickenCapPiece(left.head, right.head))
}

// thickenCapPiece is one free end's cap: the straight join between the two
// copies' own endpoints there. Both endpoints offset along the SAME normal
// line, so the cap is axis-parallel wherever the walk's end segment is.
func thickenCapPiece(from, to thickenMovingPoint) thickenMovingPiece {
	horizontal := from.v.a.Cmp(to.v.a) == 0 && from.v.b.Cmp(to.v.b) == 0
	return thickenMovingPiece{line: true, horizontal: horizontal, start: from, end: to}
}

// thickenExactPointAt evaluates one moving point at τ and refuses unless every
// held float64 equals its closed-form value exactly (R27).
func thickenExactPointAt(p thickenMovingPoint, at *big.Rat) (sectionrecord.Point2, error) {
	u, v := thickenAffineAt(p.u, at), thickenAffineAt(p.v, at)
	held := sectionrecord.Point2{U: ratToFloat(u), V: ratToFloat(v)}
	if proofarith.RationalFloatError(u, held.U) != 0 || proofarith.RationalFloatError(v, held.V) != 0 {
		return sectionrecord.Point2{}, fmt.Errorf(`%w: a generated ribbon coordinate is rounded`, decaderr.ErrUnsupported)
	}
	return held, nil
}

func ratToFloat(r *big.Rat) float64 {
	f, _ := r.Float64()
	return f
}

// thickenRibbonSection evaluates the assembled moving boundary at the
// requested offset, refusing any coordinate that does not land exactly.
func thickenRibbonSection(pieces []thickenMovingPiece, at *big.Rat) (sectionrecord.LoopRecord, error) {
	segs := make([]sectionrecord.CurveSegment, 0, len(pieces))
	for _, p := range pieces {
		start, err := thickenExactPointAt(p.start, at)
		if err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		end, err := thickenExactPointAt(p.end, at)
		if err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		if p.line {
			if start == end {
				return sectionrecord.LoopRecord{}, fmt.Errorf(`%w: a generated ribbon segment has no length`, decaderr.ErrUnsupported)
			}
			segs = append(segs, sectionrecord.LineSeg{Start: start, End: end, TStart: 0, TEnd: 1})
			continue
		}
		center := sectionrecord.Point2{U: ratToFloat(p.center.u), V: ratToFloat(p.center.v)}
		if proofarith.RationalFloatError(p.center.u, center.U) != 0 || proofarith.RationalFloatError(p.center.v, center.V) != 0 {
			return sectionrecord.LoopRecord{}, fmt.Errorf(`%w: a generated ribbon corner centre is rounded`, decaderr.ErrUnsupported)
		}
		segs = append(segs, thickenArcSegment(center, start, end, thickenArcIsCCW(start, end, center)))
	}
	return sectionrecord.LoopRecord{Segments: segs}, nil
}

// thickenArcIsCCW reads a right-angle corner arc's own sense from the exact
// integer cross product of its two radii — never an angle.
func thickenArcIsCCW(start, end, center sectionrecord.Point2) bool {
	return (start.U-center.U)*(end.V-center.V)-(start.V-center.V)*(end.U-center.U) > 0
}

// RibbonSection builds and checks one thickened open walk's closed section.
func RibbonSection(ctx context.Context, walks []survey2d.SideWalk, rightSteps, leftSteps int,
	amount float64, budget *proofbound.WorkBudget, radial *Radial) (sectionrecord.LoopRecord, error) {
	dirs, err := thickenOpenDirections(walks, budget)
	if err != nil {
		return sectionrecord.LoopRecord{}, err
	}
	pieces := thickenRibbonLoop(
		thickenRibbonCopyOf(walks, dirs, thickenRibbonSide{normal: -1, steps: rightSteps}),
		thickenRibbonCopyOf(walks, dirs, thickenRibbonSide{normal: +1, steps: leftSteps}),
	)
	limit := proofarith.FloatRat(amount)
	if limit == nil {
		return sectionrecord.LoopRecord{}, fmt.Errorf(`%w: the thicken offset is not finite`, decaderr.ErrUnsupported)
	}
	if err := thickenPiecesIntervalClear(ctx, pieces, limit, budget, radial); err != nil {
		return sectionrecord.LoopRecord{}, err
	}
	return thickenRibbonSection(pieces, limit)
}

// thickenArcSegment records a generated corner in its walking direction.
func thickenArcSegment(center, start, end sectionrecord.Point2, ccw bool) sectionrecord.CurveSegment {
	if ccw {
		return sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return sectionrecord.ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}
