package mirrorjoin

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SpliceResult is the joined region before the root's modify audit.
type SpliceResult struct {
	Outer LoopRecord
	Holes []LoopRecord
	Delta float64
}

// Splice cuts each loop at selected walls and closes each run with its
// reflected reverse. The largest positive closed loop becomes the outer.
func (l Line) Splice(budget *proofbound.WorkBudget, region Region) (SpliceResult, error) {
	var closed, kept, images []LoopRecord
	delta := 0.0
	for li, loop := range region.Loops {
		if err := budget.Step(); err != nil {
			return SpliceResult{}, err
		}
		segs := make([]CurveSegment, len(loop.Segments))
		for i, raw := range loop.Segments {
			seg, err := sectionrecord.NormalizeSegment(raw)
			if err != nil {
				return SpliceResult{}, err
			}
			segs[i] = seg
		}
		if len(region.Sel[li]) == 0 {
			img, charge, err := l.ReverseRun(budget, segs)
			if err != nil {
				return SpliceResult{}, err
			}
			kept = append(kept, LoopRecord{Segments: segs})
			images = append(images, LoopRecord{Segments: img})
			delta = math.Max(delta, charge)
			continue
		}
		cuts := make([]int, 0, len(region.Sel[li]))
		for si := range region.Sel[li] {
			cuts = append(cuts, si)
		}
		slices.Sort(cuts)
		n := len(segs)
		for t, a := range cuts {
			b := cuts[(t+1)%len(cuts)]
			var run []CurveSegment
			for i := (a + 1) % n; i != b; i = (i + 1) % n {
				run = append(run, segs[i])
			}
			if len(run) == 0 {
				continue
			}
			img, charge, err := l.ReverseRun(budget, run)
			if err != nil {
				return SpliceResult{}, err
			}
			closed = append(closed, LoopRecord{Segments: append(slices.Clone(run), img...)})
			delta = math.Max(delta, charge)
		}
	}
	outer := -1
	best := 0.0
	for i, loop := range closed {
		area, err := sectionaudit.LoopSignedArea(budget, loop)
		if err != nil {
			return SpliceResult{}, err
		}
		if area > best {
			outer, best = i, area
		}
	}
	if outer < 0 {
		return SpliceResult{}, fmt.Errorf(`%w: the join's spliced loops enclose no material`, decaderr.ErrDegenerate)
	}
	out := SpliceResult{Outer: closed[outer], Delta: delta}
	for i, loop := range closed {
		if i != outer {
			out.Holes = append(out.Holes, loop)
		}
	}
	out.Holes = append(out.Holes, kept...)
	out.Holes = append(out.Holes, images...)
	return out, nil
}
