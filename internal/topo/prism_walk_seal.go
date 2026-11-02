package topo

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type prismWalkSeal struct {
	anchor *sectionrecord.LoopRecord
	outer  []sectionrecord.AnalyticWalkView
	holes  [][]sectionrecord.AnalyticWalkView
}

// SealPrismBuildWalksReplay captures the producer's bounded analytic walks.
func (b *Body) SealPrismBuildWalksReplay(ctx context.Context, source sectionrecord.AnalyticWalkSource, budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil || source.Anchor == nil {
		return false, nil
	}
	count := uint64(len(source.Outer))
	if count > prismSealLimit {
		return false, nil
	}
	for _, loop := range source.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if uint64(len(loop)) > prismSealLimit-count {
			return false, nil
		}
		count += uint64(len(loop))
	}
	if count == 0 || count > prismSealLimit || len(source.Holes) > prismSealLimit {
		return false, nil
	}
	if len(source.Profile.Outer.Segments) > prismSealLimit {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 64*uint64(len(source.Profile.Outer.Segments))+count+1); err != nil {
		return false, err
	}
	bytes := uint64(unsafe.Sizeof(prismWalkSeal{})) + 2*count*uint64(unsafe.Sizeof(sectionrecord.AnalyticWalkView{})) + 2*uint64(len(source.Holes))*uint64(unsafe.Sizeof([]sectionrecord.AnalyticWalkView{})) + 1024
	if err := budget.ReserveBundle(ctx, 128*count+64, count+2, bytes); err != nil {
		return false, err
	}
	ok := b.sealPrismBuildWalks(source, func(uint64) error { return ctx.Err() })
	if err := ctx.Err(); err != nil {
		if b != nil && b.prismBuildSeal != nil {
			b.prismBuildSeal.walks = nil
		}
		return false, err
	}
	return ok, nil
}

func (b *Body) sealPrismBuildWalks(source sectionrecord.AnalyticWalkSource, spend func(uint64) error) bool {
	if b == nil || b.prismBuildSeal == nil || b.prismBuildSeal.payload == nil || b.prismBuildSeal.walks != nil || source.Anchor == nil || spend == nil || len(b.prismBuildSeal.payload.profiles) != 1 || len(source.Holes) != len(source.Profile.Holes) || len(source.Holes) != len(b.prismBuildSeal.payload.profiles[0].Holes) || len(source.Outer) != len(source.Profile.Outer.Segments) {
		return false
	}
	profile := b.prismBuildSeal.payload.profiles[0]
	if !prismSameAnalyticLoop(*source.Anchor, source.Profile.Outer) ||
		!prismSameAnalyticLoop(source.Profile.Outer, profile.Outer) || len(source.Profile.Holes) != len(profile.Holes) {
		return false
	}
	count := len(source.Outer)
	for i, loop := range profile.Holes {
		if len(source.Holes[i]) != len(loop.Segments) || !prismSameAnalyticLoop(source.Profile.Holes[i], loop) {
			return false
		}
		count += len(source.Holes[i])
	}
	if count == 0 || count > prismSealLimit || spend(uint64(count)*64+64) != nil {
		return false
	}
	for _, walk := range source.Outer {
		if walk.Kind > 1 {
			return false
		}
	}
	for _, loop := range source.Holes {
		for _, walk := range loop {
			if walk.Kind > 1 {
				return false
			}
		}
	}
	seal := &prismWalkSeal{anchor: source.Anchor, outer: append([]sectionrecord.AnalyticWalkView(nil), source.Outer...), holes: make([][]sectionrecord.AnalyticWalkView, len(source.Holes))}
	for i, loop := range source.Holes {
		seal.holes[i] = append([]sectionrecord.AnalyticWalkView(nil), loop...)
	}
	b.prismBuildSeal.walks = seal
	return true
}

func prismSameAnalyticLoop(a, b sectionrecord.LoopRecord) bool {
	if len(a.Segments) != len(b.Segments) {
		return false
	}
	for i, segment := range a.Segments {
		if !sectionrecord.IdenticalAnalyticSegment(segment, b.Segments[i]) {
			return false
		}
	}
	return true
}

// HasStrictPrismWalkSeal reports whether the original analytic walks were captured.
func (b *Body) HasStrictPrismWalkSeal() bool {
	return b != nil && b.HasStrictPrismBuildSeal() && b.prismBuildSeal.walks != nil
}

// ReplayPrismBuiltWalk checks one analytic walk and the original source anchor.
func (b *Body) ReplayPrismBuiltWalk(ctx context.Context, anchor *sectionrecord.LoopRecord, loopIndex, segmentIndex int, currentLoop sectionrecord.LoopRecord, walkCount int, current sectionrecord.AnalyticWalkView, budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil || !b.HasStrictPrismWalkSeal() || anchor == nil || anchor != b.prismBuildSeal.walks.anchor {
		return false, nil
	}
	seal := b.prismBuildSeal.walks
	var original []sectionrecord.AnalyticWalkView
	var recorded sectionrecord.LoopRecord
	if loopIndex == 0 {
		original = seal.outer
		recorded = b.prismBuildSeal.payload.profiles[0].Outer
	} else if loopIndex > 0 && loopIndex <= len(seal.holes) {
		original = seal.holes[loopIndex-1]
		recorded = b.prismBuildSeal.payload.profiles[0].Holes[loopIndex-1]
	} else {
		return false, nil
	}
	if segmentIndex < 0 || segmentIndex >= len(original) || walkCount != len(original) || len(currentLoop.Segments) != len(recorded.Segments) || walkCount != len(recorded.Segments) {
		return false, nil
	}
	for i, segment := range currentLoop.Segments {
		if err := budget.SpendWork(ctx, 64); err != nil {
			return false, err
		}
		if !sectionrecord.IdenticalAnalyticSegment(segment, recorded.Segments[i]) {
			return false, nil
		}
	}
	if err := budget.ReserveBundle(ctx, 64, 2, 512); err != nil {
		return false, err
	}
	return current == original[segmentIndex], nil
}
