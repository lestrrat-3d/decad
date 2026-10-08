package offset2d

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ReverseLoopRecord walks a loop in the opposite sense, re-emitting each segment
// reversed — what turns an offset outer loop into a tube's hole (a hole is
// walked clockwise, so its wall's material lies outside it). It reads the loop
// through walkOf, so it handles line, arc and full-circle segments alike.
func ReverseLoopRecord(l LoopRecord) (LoopRecord, error) {
	return ReverseLoopRecordBudget(nil, l)
}

func ReverseLoopRecordBudget(budget *proofbound.WorkBudget, l LoopRecord) (LoopRecord, error) {
	return reverseLoopRecordWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, l)
}

func ReverseLoopRecordContext(ctx context.Context, l LoopRecord) (LoopRecord, error) {
	return reverseLoopRecordWithPoll(ctx.Err, l)
}

func reverseLoopRecordWithPoll(poll func() error, l LoopRecord) (LoopRecord, error) {
	// One free-form counter for this loop's walk: reversal is reached from the
	// offset construction and the cup build, neither of which holds a preflight
	// counter for the loop it hands over.
	work := freeform.NewFreeformWork()
	n := len(l.Segments)
	walks := make([]survey2d.SegmentWalk, n)
	for i, seg := range l.Segments {
		if poll != nil {
			if err := poll(); err != nil {
				return LoopRecord{}, err
			}
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return LoopRecord{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "the shell section offset"); err != nil {
			return LoopRecord{}, err
		}
		walks[i] = w
	}
	segs := make([]CurveSegment, 0, n)
	for i := n - 1; i >= 0; i-- {
		if poll != nil {
			if err := poll(); err != nil {
				return LoopRecord{}, err
			}
		}
		w := walks[i]
		switch {
		case w.Closed:
			segs = append(segs, CircleSegment(w.CU, w.CV, w.Radius, !(w.Th1 > w.Th0)))
		case w.IsCircular():
			segs = append(segs, ArcSegment(Point2{U: w.CU, V: w.CV}, Point2{U: w.EndU, V: w.EndV}, Point2{U: w.StartU, V: w.StartV}, !(w.Th1 > w.Th0)))
		default:
			segs = append(segs, LineSeg{Start: Point2{U: w.EndU, V: w.EndV}, End: Point2{U: w.StartU, V: w.StartV}, TStart: 0, TEnd: 1})
		}
	}
	return LoopRecord{Segments: segs}, nil
}
