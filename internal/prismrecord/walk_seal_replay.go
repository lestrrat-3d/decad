package prismrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// SealPrismBuildWalksReplay converts the producer's analytic walks into a
// floor-safe snapshot, binding the original ProfileWalks field address.
func SealPrismBuildWalksReplay(ctx context.Context, body *topo.Body, pw *momentinput.ProfileWalks, budget *proofbound.ReplayBudget) (bool, error) {
	if body == nil || pw == nil || budget == nil || !pw.Metered || pw.Spent != 0 || pw.ReconstructionSpent != 0 || len(pw.ReadCharges) != 0 || len(pw.Holes) != len(pw.Profile.Holes) {
		return false, nil
	}
	count := uint64(len(pw.Outer))
	if count > 4096 {
		return false, nil
	}
	for _, loop := range pw.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if uint64(len(loop)) > 4096-count {
			return false, nil
		}
		count += uint64(len(loop))
	}
	if count == 0 || count > 4096 || len(pw.Holes) > 4096 {
		return false, nil
	}
	bytes := 2*count*uint64(unsafe.Sizeof(sectionrecord.AnalyticWalkView{})) + 2*uint64(len(pw.Holes))*uint64(unsafe.Sizeof([]sectionrecord.AnalyticWalkView{})) + 256
	if err := budget.ReserveBundle(ctx, 64*count+64, count+uint64(len(pw.Holes))+2, bytes); err != nil {
		return false, err
	}
	source := sectionrecord.AnalyticWalkSource{Anchor: &pw.Profile.Outer, Profile: pw.Profile.AnalyticSealView(), Outer: make([]sectionrecord.AnalyticWalkView, len(pw.Outer)), Holes: make([][]sectionrecord.AnalyticWalkView, len(pw.Holes))}
	for i, w := range pw.Outer {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		v, ok := w.AnalyticSealView()
		if !ok {
			return false, nil
		}
		source.Outer[i] = v
	}
	for i, loop := range pw.Holes {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		source.Holes[i] = make([]sectionrecord.AnalyticWalkView, len(loop))
		for j, w := range loop {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			v, ok := w.AnalyticSealView()
			if !ok {
				return false, nil
			}
			source.Holes[i][j] = v
		}
	}
	return body.SealPrismBuildWalksReplay(ctx, source, budget)
}

// ReplayPrismBuiltWalk verifies the original source identity and one walk.
func ReplayPrismBuiltWalk(ctx context.Context, body *topo.Body, pw *momentinput.ProfileWalks, loopIndex, segmentIndex int, budget *proofbound.ReplayBudget) (survey2d.SegmentWalk, bool, error) {
	if body == nil || pw == nil || budget == nil || !pw.Metered || pw.Spent != 0 || pw.ReconstructionSpent != 0 || len(pw.ReadCharges) != 0 {
		return survey2d.SegmentWalk{}, false, nil
	}
	var loop sectionrecord.LoopRecord
	var walks []survey2d.SegmentWalk
	if len(pw.Holes) != len(pw.Profile.Holes) {
		return survey2d.SegmentWalk{}, false, nil
	}
	if loopIndex == 0 {
		loop, walks = pw.Profile.Outer, pw.Outer
	} else if loopIndex > 0 && loopIndex <= len(pw.Profile.Holes) && loopIndex <= len(pw.Holes) {
		loop, walks = pw.Profile.Holes[loopIndex-1], pw.Holes[loopIndex-1]
	} else {
		return survey2d.SegmentWalk{}, false, nil
	}
	if segmentIndex < 0 || segmentIndex >= len(walks) {
		return survey2d.SegmentWalk{}, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return survey2d.SegmentWalk{}, false, err
	}
	w := walks[segmentIndex]
	view, ok := w.AnalyticSealView()
	if !ok {
		return survey2d.SegmentWalk{}, false, nil
	}
	ok, err := body.ReplayPrismBuiltWalk(ctx, &pw.Profile.Outer, loopIndex, segmentIndex, loop, len(walks), view, budget)
	if err != nil || !ok {
		return survey2d.SegmentWalk{}, false, err
	}
	return w, true, nil
}
