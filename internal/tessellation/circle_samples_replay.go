package tessellation

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// CircleSamplesReplay samples a natural whole circle using a newly reconstructed
// walk. The caller authenticates the record and its associated face separately.
func CircleSamplesReplay[F any](ctx context.Context, seg sectionrecord.CircleSeg, face F,
	chord, height float64, budget *proofbound.ReplayBudget) (ChordSamples[F], error) {
	if budget == nil {
		return ChordSamples[F]{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ChordSamples[F]{}, err
	}
	w, err := boundarywalk.CircleWalkReplay(ctx, seg, budget)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	n, sag, capArea, wallArea, err := CircleScheduleReplay(ctx, seg, w, chord, height, budget)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	if err = budget.SpendWork(ctx, uint64(n)*64); err != nil {
		return ChordSamples[F]{}, err
	}
	if err = budget.ReserveRecords(ctx, uint64(4*n+4)); err != nil {
		return ChordSamples[F]{}, err
	}
	for _, size := range [...]uint64{uint64(unsafe.Sizeof(sectionrecord.Point2{})), uint64(unsafe.Sizeof(*new(F))), uint64(unsafe.Sizeof(float64(0))), uint64(unsafe.Sizeof(proofbound.WalkEndBound{}))} {
		if err = budget.ReserveArray(ctx, uint64(n), size); err != nil {
			return ChordSamples[F]{}, err
		}
	}
	out := ChordSamples[F]{Samples: make([]sectionrecord.Point2, n), FaceOf: make([]F, n), SagOf: make([]float64, n), BoundOf: make([]proofbound.WalkEndBound, n), MaxSag: sag, WallSlack: wallArea, CapSlack: capArea, SegmentArea: capArea, Walks: 1, PerimeterUpper: proofbound.AbsSumUpper(0, w.Length, w.LengthBound)}
	cache, err := circularbounds.NewCircleStationReplayCache(ctx, seg, budget)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	turns, err := newCircleTurnCache(ctx, n, budget)
	if err != nil {
		return ChordSamples[F]{}, err
	}
	dth := (w.Th1 - w.Th0) / float64(n)
	for k := range n {
		if err := ctx.Err(); err != nil {
			return ChordSamples[F]{}, err
		}
		p := sectionrecord.Point2{U: w.StartU, V: w.StartV}
		bound := boundarywalk.JunctionStartBound(seg, w, seg, w)
		if k > 0 {
			th := w.Th0 + float64(k)*dth
			p = sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th), V: w.CV + w.Radius*math.Sin(th)}
			sin, cos, trigErr := turns.station(ctx, k, budget)
			if trigErr != nil {
				return ChordSamples[F]{}, trigErr
			}
			bound, err = cache.Bound(ctx, p.U, p.V, sin, cos, budget)
			if err != nil {
				return ChordSamples[F]{}, err
			}
		}
		out.Samples[k], out.FaceOf[k], out.SagOf[k], out.BoundOf[k] = p, face, sag, bound
	}
	return out, nil
}
