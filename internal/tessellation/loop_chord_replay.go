package tessellation

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type replayLoopWalk struct {
	n      int
	sag    float64
	start  *big.Rat
	span   *big.Rat
	circle *circularbounds.ReplayCircleEndpointPlan
	arc    *circularbounds.ReplayArcEndpointPlan
}

// ReplayChordLoop chords an analytic loop from separately authenticated
// source walks. The caller must bind each walk to the sealed producer record
// and the loop to the body before this private consumer runs.
func ReplayChordLoop(ctx context.Context, loop sectionrecord.LoopRecord,
	sourceWalks []survey2d.SegmentWalk, chord, height float64,
	budget *proofbound.ReplayBudget) (ChordSamples[int], error) {
	if budget == nil || len(loop.Segments) == 0 || len(sourceWalks) != len(loop.Segments) ||
		len(loop.Segments) > 4096 || proofbound.IsNonFinite(chord) || chord <= 0 ||
		proofbound.IsNonFinite(height) {
		return ChordSamples[int]{}, decaderr.ErrUnsupported
	}
	count := uint64(len(loop.Segments))
	if err := budget.ReserveBundle(ctx, 8*count, 2*count,
		2*count*(uint64(unsafe.Sizeof(survey2d.SideWalk{}))+8)+512); err != nil {
		return ChordSamples[int]{}, err
	}
	indices := make([]int, len(loop.Segments))
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, w := range sourceWalks {
		if w.Kind != survey2d.WalkLine && w.Kind != survey2d.WalkCircular ||
			len(w.Spans) != 0 {
			return ChordSamples[int]{}, decaderr.ErrUnsupported
		}
		indices[i] = i
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: indices[i : i+1 : i+1]}
	}
	walks, err := boundarywalk.CoalesceWalksNoMergeReplay(ctx, raw, budget)
	if err != nil {
		return ChordSamples[int]{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(walks)), uint64(unsafe.Sizeof(replayLoopWalk{}))); err != nil {
		return ChordSamples[int]{}, err
	}
	plans := make([]replayLoopWalk, len(walks))
	total := 0
	for i, w := range walks {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return ChordSamples[int]{}, err
		}
		if len(w.Segs) != 1 || w.Segs[0] < 0 || w.Segs[0] >= len(loop.Segments) {
			return ChordSamples[int]{}, decaderr.ErrUnsupported
		}
		plans[i].n = 1
		if w.Kind == survey2d.WalkCircular {
			n, sag, err := ReplayChordCount(ctx, w.SegmentWalk, chord,
				ChordWalkMin(w.SegmentWalk), budget)
			if err != nil {
				return ChordSamples[int]{}, err
			}
			plans[i].n, plans[i].sag = n, sag
			seg := loop.Segments[w.Segs[0]]
			var t0, t1 float64
			switch s := seg.(type) {
			case sectionrecord.CircleSeg:
				t0, t1 = s.TStart, s.TEnd
				plans[i].circle, err = circularbounds.NewReplayCircleEndpointPlan(ctx,
					circularbounds.CircleSeg{Center: circularbounds.Point2(s.Center),
						Radius: s.Radius, CCW: s.CCW, TStart: s.TStart, TEnd: s.TEnd}, budget)
			case sectionrecord.ArcSeg:
				t0, t1 = s.TStart, s.TEnd
				plans[i].arc, err = circularbounds.NewReplayArcEndpointPlan(ctx,
					circularbounds.ArcSeg{Center: circularbounds.Point2(s.Center),
						Start: circularbounds.Point2(s.Start), End: circularbounds.Point2(s.End),
						TStart: s.TStart, TEnd: s.TEnd}, budget)
			default:
				return ChordSamples[int]{}, decaderr.ErrUnsupported
			}
			if err != nil {
				return ChordSamples[int]{}, err
			}
			plans[i].start, err = proofarith.ReplayFloatRat(ctx, t0, budget)
			if err != nil {
				return ChordSamples[int]{}, err
			}
			end, err := proofarith.ReplayFloatRat(ctx, t1, budget)
			if err != nil {
				return ChordSamples[int]{}, err
			}
			plans[i].span, err = proofarith.ReplayRatArithmetic(ctx, end, plans[i].start, '-', budget)
			if err != nil {
				return ChordSamples[int]{}, err
			}
		}
		if plans[i].n <= 0 || total > 4096-plans[i].n {
			return ChordSamples[int]{}, decaderr.ErrUnsupported
		}
		total += plans[i].n
	}
	if err := budget.ReserveBundle(ctx, uint64(total), uint64(4*total), 512); err != nil {
		return ChordSamples[int]{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(total), uint64(unsafe.Sizeof(sectionrecord.Point2{}))); err != nil {
		return ChordSamples[int]{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(total), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return ChordSamples[int]{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(total), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ChordSamples[int]{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(total), uint64(unsafe.Sizeof(proofbound.WalkEndBound{}))); err != nil {
		return ChordSamples[int]{}, err
	}
	out := ChordSamples[int]{Samples: make([]sectionrecord.Point2, 0, total),
		FaceOf: make([]int, 0, total), SagOf: make([]float64, 0, total),
		BoundOf: make([]proofbound.WalkEndBound, 0, total), Walks: len(walks)}
	for i, w := range walks {
		if err := budget.SpendWork(ctx, 64); err != nil {
			return ChordSamples[int]{}, err
		}
		out.PerimeterUpper = proofbound.AbsSumUpper(out.PerimeterUpper, w.Length, w.LengthBound)
		if len(walks) == 1 && !w.Closed {
			return ChordSamples[int]{}, decaderr.ErrUnsupported
		}
		prev := walks[(i+len(walks)-1)%len(walks)]
		start, err := boundarywalk.JunctionStartBoundReplay(ctx,
			loop.Segments, prev, w, budget)
		if err != nil {
			return ChordSamples[int]{}, err
		}
		if w.Kind == survey2d.WalkCircular {
			out.MaxSag = math.Max(out.MaxSag, plans[i].sag)
			out.WallSlack = proofbound.AbsSumUpper(out.WallSlack,
				proofbound.ProductUpper(WalkWallSlack(w.SegmentWalk, plans[i].n, height), 1+1e-9))
			area := WalkSegmentArea(w.SegmentWalk, plans[i].n)
			out.CapSlack = proofbound.AbsSumUpper(out.CapSlack, proofbound.ProductUpper(area, 1+1e-9))
			out.SegmentArea = proofbound.AbsSumUpper(out.SegmentArea, area)
		}
		for k := range plans[i].n {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return ChordSamples[int]{}, err
			}
			p := sectionrecord.Point2{U: w.StartU, V: w.StartV}
			bound := start
			if k > 0 {
				dth := (w.Th1 - w.Th0) / float64(plans[i].n)
				th := w.Th0 + float64(k)*dth
				p = sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th),
					V: w.CV + w.Radius*math.Sin(th)}
				rt, err := replayChordParameter(ctx, plans[i], k, budget)
				if err != nil {
					return ChordSamples[int]{}, err
				}
				var u, v proofbound.RatInterval
				if plans[i].circle != nil {
					u, v, err = plans[i].circle.Endpoint(ctx, rt, budget)
				} else {
					u, v, err = plans[i].arc.Endpoint(ctx, rt, budget)
				}
				if err != nil {
					return ChordSamples[int]{}, err
				}
				bound.U, err = replayIntervalFloatError(ctx, u, p.U, budget)
				if err != nil {
					return ChordSamples[int]{}, err
				}
				bound.V, err = replayIntervalFloatError(ctx, v, p.V, budget)
				if err != nil {
					return ChordSamples[int]{}, err
				}
			}
			out.Samples = append(out.Samples, p)
			out.FaceOf = append(out.FaceOf, w.Segs[0])
			out.SagOf = append(out.SagOf, plans[i].sag)
			out.BoundOf = append(out.BoundOf, bound)
		}
	}
	return out, nil
}

func replayChordParameter(ctx context.Context, plan replayLoopWalk, k int,
	budget *proofbound.ReplayBudget) (*big.Rat, error) {
	if err := budget.ReserveBundle(ctx, 1, 2, 128); err != nil {
		return nil, err
	}
	frac, err := proofarith.ReplayReducedFraction(ctx,
		big.NewInt(int64(k)), big.NewInt(int64(plan.n)), budget)
	if err != nil {
		return nil, err
	}
	step, err := proofarith.ReplayRatArithmetic(ctx, frac, plan.span, '*', budget)
	if err != nil {
		return nil, err
	}
	return proofarith.ReplayRatArithmetic(ctx, plan.start, step, '+', budget)
}

func replayIntervalFloatError(ctx context.Context, iv proofbound.RatInterval,
	held float64, budget *proofbound.ReplayBudget) (float64, error) {
	lo, err := proofarith.ReplayRationalFloatError(ctx, iv.Lo, held, budget)
	if err != nil {
		return 0, err
	}
	hi, err := proofarith.ReplayRationalFloatError(ctx, iv.Hi, held, budget)
	if err != nil {
		return 0, err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	return math.Max(lo, hi), nil
}
