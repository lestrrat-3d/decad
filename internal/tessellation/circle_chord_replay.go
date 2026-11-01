package tessellation

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// ChordCircleReplay chords one recorded whole circle under the brake budget.
func ChordCircleReplay[F any](ctx context.Context, loop sectionrecord.LoopRecord, chord, height float64,
	resolved *momentinput.ProfileWalks, wallFace func(survey2d.SideWalk) (F, error),
	b *proofbound.ReplayBudget) (ChordSamples[F], error) {
	if err := ctx.Err(); err != nil {
		return ChordSamples[F]{}, err
	}
	if b == nil || !b.IsBrakeReplay() || wallFace == nil || len(loop.Segments) != 1 || resolved == nil ||
		!resolved.Metered || resolved.Spent != 0 || resolved.ReconstructionSpent != 0 ||
		len(resolved.Profile.Holes) != 0 || len(resolved.Profile.Outer.Segments) != 1 ||
		len(resolved.Outer) != 1 || len(resolved.Holes) != 0 {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded circle requires a recorded whole walk", decaderr.ErrUnsupported)
	}
	seg, ok := loop.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded circle requires a counterclockwise whole circle", decaderr.ErrUnsupported)
	}
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(radius >= math.Ldexp(1, -100)) || radius > math.Ldexp(1, 100) {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded circle radius exceeds its arithmetic envelope", decaderr.ErrUnsupported)
	}
	w := resolved.Outer[0]
	if w.Kind != survey2d.WalkCircular || !w.Closed || len(w.Spans) != 0 || w.CU != seg.Center.U ||
		w.CV != seg.Center.V || w.Radius != radius || w.Th0 != 0 || w.Th1 != 2*math.Pi {
		return ChordSamples[F]{}, fmt.Errorf("%w: bounded circle walk does not match its record", decaderr.ErrUnsupported)
	}
	if err := b.SpendWork(ctx, 128); err != nil {
		return ChordSamples[F]{}, err
	}
	if err := b.ReserveRecords(ctx, 2); err != nil {
		return ChordSamples[F]{}, err
	}
	if err := b.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(sectionrecord.LoopRecord{}))); err != nil {
		return ChordSamples[F]{}, err
	}
	if !resolved.LoopMatches(0, loop) {
		return ChordSamples[F]{}, momentinput.ErrResolvedWalksMismatch
	}
	n, sag, err := ChordCountWithBudget(w, chord, ChordWalkMin(w), b.WorkBudget(ctx))
	if err != nil {
		return ChordSamples[F]{}, err
	}
	if err := b.SpendWork(ctx, uint64(n+8)); err != nil {
		return ChordSamples[F]{}, err
	}
	if err := b.ReserveRecords(ctx, uint64(4*n+4)); err != nil {
		return ChordSamples[F]{}, err
	}
	for _, a := range [][2]uint64{
		{1, uint64(unsafe.Sizeof(int(0)))},
		{uint64(n), uint64(unsafe.Sizeof(sectionrecord.Point2{}))},
		{uint64(n), uint64(unsafe.Sizeof(*new(F)))},
		{uint64(n), uint64(unsafe.Sizeof(float64(0)))},
		{uint64(n), uint64(unsafe.Sizeof(proofbound.WalkEndBound{}))},
	} {
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return ChordSamples[F]{}, err
		}
	}
	face, err := wallFace(survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}})
	if err != nil {
		return ChordSamples[F]{}, err
	}
	out := ChordSamples[F]{Samples: make([]sectionrecord.Point2, n), FaceOf: make([]F, n), SagOf: make([]float64, n),
		BoundOf: make([]proofbound.WalkEndBound, n), MaxSag: math.Max(0, sag), Walks: 1,
		WallSlack:      proofbound.AbsSumUpper(0, proofbound.ProductUpper(WalkWallSlack(w, n, height), 1+1e-9)),
		CapSlack:       proofbound.AbsSumUpper(0, proofbound.ProductUpper(WalkSegmentArea(w, n), 1+1e-9)),
		SegmentArea:    proofbound.AbsSumUpper(0, WalkSegmentArea(w, n)),
		PerimeterUpper: proofbound.AbsSumUpper(0, w.Length, w.LengthBound)}
	dth := (w.Th1 - w.Th0) / float64(n)
	start := boundarywalk.JunctionStartBound(seg, w, seg, w)
	for k := range n {
		p := sectionrecord.Point2{U: w.StartU, V: w.StartV}
		bound := start
		if k > 0 {
			th := w.Th0 + float64(k)*dth
			p = sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th), V: w.CV + w.Radius*math.Sin(th)}
			bound, err = circularbounds.CircleStationReplay(ctx, seg, k, n, p.U, p.V, b)
			if err != nil {
				return ChordSamples[F]{}, err
			}
		}
		out.Samples[k], out.FaceOf[k], out.SagOf[k], out.BoundOf[k] = p, face, sag, bound
	}
	return out, nil
}
