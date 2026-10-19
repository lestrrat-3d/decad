package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/sketch"
)

// FitSideCrossing describes the exact source crossing before root binds live faces.
type FitSideCrossing struct {
	Segment int
	Span    freeform.BezierSpan
	Index   int
	First   int
	Last    int
	Caps    []loftface.RootCrossing
	X       []loftface.Interval
}

// FitSide classifies signed sides over the entire true mapped Bezier span in
// every band. A crossing has a strict Q_s sign and a positive Bernstein bound
// on the root's X movement.
func FitSide(trim *loftFacesTrimRecord, m, b *big.Rat,
	segmentIndex int, segment fitSplineSeg) (*FitSideCrossing, int, bool) {
	if _, ok := trim.Source.Source.Entities[segmentIndex].(*sketch.FitSpline); !ok ||
		!((segment.TStart == 0 && segment.TEnd == 1) ||
			(segment.TStart == 1 && segment.TEnd == 0)) {
		return nil, 0, false
	}
	work := freeform.NewFreeformWork()
	work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	spans, err := splinebezier.FitSplineBezierSpans(segment, work)
	if err != nil || len(spans) == 0 || len(spans) > 128 {
		return nil, 0, false
	}
	sides := make([]int, len(spans))
	var crossing *FitSideCrossing
	for index, span := range spans {
		var sign int
		seenBand := false
		for band := 0; band+1 < len(trim.Maps); band++ {
			q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
			kind, signed := loftface.ClassifyRows(q)
			if kind == 2 || seenBand && kind != sign {
				return nil, 0, false
			}
			sign = kind
			seenBand = true
			if kind == loftface.SideCrossed &&
				!polynomial.BernsteinRowsPositive(loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, signed)) {
				return nil, 0, false
			}
		}
		sides[index] = sign
		if sign == loftface.SideCrossed {
			if crossing != nil {
				return nil, 0, false
			}
			crossing = &FitSideCrossing{Segment: segmentIndex, Span: span,
				Index: index, Caps: make([]loftface.RootCrossing, len(trim.Maps)),
				X: make([]loftface.Interval, len(trim.Maps))}
		}
	}
	if crossing == nil {
		for _, side := range sides {
			if side != sides[0] || side == 0 {
				return nil, 0, false
			}
		}
		return nil, sides[0], true
	}
	q := loftface.MeridianQRows(trim.Maps[0], trim.Maps[1], crossing.Span, m, b)
	before, after := loftface.SideInside, loftface.SideOutside
	if q[0][0].Sign() > 0 {
		before, after = after, before
	}
	for i, side := range sides {
		switch {
		case i < crossing.Index && side != before:
			return nil, 0, false
		case i > crossing.Index && side != after:
			return nil, 0, false
		}
	}
	for capIndex, mapped := range trim.Maps {
		capQ := loftface.MeridianQRows(mapped, mapped, crossing.Span, m, b)
		kind, signedCap := loftface.ClassifyRows(capQ)
		if kind != loftface.SideCrossed {
			return nil, 0, false
		}
		root := loftface.IsolateRoot(signedCap[0])
		root.Span = crossing.Index
		direction := loftface.CapXDirection(mapped, crossing.Span)
		if direction == 2 {
			return nil, 0, false
		}
		crossing.Caps[capIndex] = root
		crossing.X[capIndex] = loftface.DirectedCapXRange(mapped, crossing.Span, root, direction)
	}
	for i := 0; i+1 < len(crossing.X); i++ {
		if crossing.X[i].Hi.Cmp(crossing.X[i+1].Lo) >= 0 {
			return nil, 0, false
		}
	}
	if segment.TStart == 1 {
		before, after = after, before
	}
	crossing.First, crossing.Last = before, after
	return crossing, 0, true
}
