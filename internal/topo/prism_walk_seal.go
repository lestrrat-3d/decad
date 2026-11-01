package topo

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type prismWalkSeal struct {
	source *momentinput.ProfileWalks
	outer  []survey2d.SegmentWalk
	holes  [][]survey2d.SegmentWalk
}

// SealPrismBuildWalksReplay reserves the complete analytic walk snapshot
// under the same per-producer witness meter as face and payload snapshots.
func (b *Body) SealPrismBuildWalksReplay(ctx context.Context, pw *momentinput.ProfileWalks,
	budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil || pw == nil {
		return false, nil
	}
	count := uint64(len(pw.Outer))
	for _, loop := range pw.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		count += uint64(len(loop))
		if count > prismSealLimit {
			return false, nil
		}
	}
	if count == 0 || count > prismSealLimit || len(pw.Holes) > prismSealLimit {
		return false, nil
	}
	bytes := uint64(unsafe.Sizeof(prismWalkSeal{})) +
		2*count*uint64(unsafe.Sizeof(survey2d.SegmentWalk{})) +
		2*uint64(len(pw.Holes))*uint64(unsafe.Sizeof([]survey2d.SegmentWalk{})) + 1024
	if err := budget.ReserveBundle(ctx, 128*count+64, count+2, bytes); err != nil {
		return false, err
	}
	ok := b.sealPrismBuildWalks(pw, func(uint64) error { return ctx.Err() })
	if err := ctx.Err(); err != nil {
		if b != nil && b.prismBuildSeal != nil {
			b.prismBuildSeal.walks = nil
		}
		return false, err
	}
	return ok, nil
}

// sealPrismBuildWalks captures only complete analytic walks from the prism
// producer, after its face and payload seals have been recorded.
func (b *Body) sealPrismBuildWalks(pw *momentinput.ProfileWalks, spend func(uint64) error) bool {
	if b == nil || b.prismBuildSeal == nil || b.prismBuildSeal.payload == nil ||
		b.prismBuildSeal.walks != nil || pw == nil || spend == nil || !pw.Metered ||
		pw.Spent != 0 || pw.ReconstructionSpent != 0 || len(pw.ReadCharges) != 0 ||
		len(b.prismBuildSeal.payload.profiles) != 1 ||
		len(pw.Holes) != len(pw.Profile.Holes) || len(pw.Holes) != len(b.prismBuildSeal.payload.profiles[0].Holes) ||
		len(pw.Outer) != len(pw.Profile.Outer.Segments) {
		return false
	}
	profile := b.prismBuildSeal.payload.profiles[0]
	if !sectionrecord.IdenticalRecord(pw.Profile.Outer, profile.Outer) ||
		len(pw.Profile.Holes) != len(profile.Holes) {
		return false
	}
	count := len(pw.Outer)
	for i, loop := range profile.Holes {
		if len(pw.Holes[i]) != len(loop.Segments) ||
			!sectionrecord.IdenticalRecord(pw.Profile.Holes[i], loop) {
			return false
		}
		count += len(pw.Holes[i])
	}
	if count == 0 || count > prismSealLimit || spend(uint64(count)*64+64) != nil {
		return false
	}
	for _, walk := range pw.Outer {
		if !prismAnalyticWalk(walk) {
			return false
		}
	}
	for _, loop := range pw.Holes {
		for _, walk := range loop {
			if !prismAnalyticWalk(walk) {
				return false
			}
		}
	}
	seal := &prismWalkSeal{source: pw,
		outer: append([]survey2d.SegmentWalk(nil), pw.Outer...),
		holes: make([][]survey2d.SegmentWalk, len(pw.Holes))}
	for i, loop := range pw.Holes {
		seal.holes[i] = append([]survey2d.SegmentWalk(nil), loop...)
	}
	b.prismBuildSeal.walks = seal
	return true
}

func prismAnalyticWalk(w survey2d.SegmentWalk) bool {
	return (w.Kind == survey2d.WalkLine || w.Kind == survey2d.WalkCircular) &&
		len(w.Spans) == 0 && !w.FitInterpolated && !w.Reversed
}

// HasStrictPrismWalkSeal reports whether this body captured its original
// producer's analytic walk values at Build time.
func (b *Body) HasStrictPrismWalkSeal() bool {
	return b != nil && b.HasStrictPrismBuildSeal() && b.prismBuildSeal.walks != nil
}

// ReplayPrismBuiltWalk binds one current source walk to its original Build
// pointer, analytic segment, role index and bitwise construction snapshot.
func (b *Body) ReplayPrismBuiltWalk(ctx context.Context, pw *momentinput.ProfileWalks,
	loopIndex, segmentIndex int, budget *proofbound.ReplayBudget) (survey2d.SegmentWalk, bool, error) {
	if budget == nil || !b.HasStrictPrismWalkSeal() || pw == nil ||
		pw != b.prismBuildSeal.walks.source || !pw.Metered ||
		pw.Spent != 0 || pw.ReconstructionSpent != 0 || len(pw.ReadCharges) != 0 {
		return survey2d.SegmentWalk{}, false, nil
	}
	seal := b.prismBuildSeal.walks
	var source sectionrecord.LoopRecord
	var original, current []survey2d.SegmentWalk
	if loopIndex == 0 {
		source = b.prismBuildSeal.payload.profiles[0].Outer
		original, current = seal.outer, pw.Outer
	} else if loopIndex > 0 && loopIndex <= len(seal.holes) && loopIndex <= len(pw.Holes) {
		source = b.prismBuildSeal.payload.profiles[0].Holes[loopIndex-1]
		original, current = seal.holes[loopIndex-1], pw.Holes[loopIndex-1]
	} else {
		return survey2d.SegmentWalk{}, false, nil
	}
	if len(current) != len(original) || segmentIndex < 0 || segmentIndex >= len(current) {
		return survey2d.SegmentWalk{}, false, nil
	}
	same, err := pw.LoopMatchesReplay(ctx, loopIndex, source, budget)
	if err != nil || !same {
		return survey2d.SegmentWalk{}, false, err
	}
	if err := budget.ReserveBundle(ctx, 64, 2, 512); err != nil {
		return survey2d.SegmentWalk{}, false, err
	}
	w := current[segmentIndex]
	if !prismAnalyticWalk(w) || !prismSameWalkBits(w, original[segmentIndex]) {
		return survey2d.SegmentWalk{}, false, nil
	}
	return w, true, nil
}

func prismSameWalkBits(a, b survey2d.SegmentWalk) bool {
	if a.Kind != b.Kind || a.Closed != b.Closed || a.Reversed != b.Reversed ||
		a.FitInterpolated != b.FitInterpolated || len(a.Spans) != len(b.Spans) {
		return false
	}
	x := [...]float64{a.StartU, a.StartV, a.EndU, a.EndV,
		a.StartBound.U, a.StartBound.V, a.EndBound.U, a.EndBound.V,
		a.TanInU, a.TanInV, a.TanOutU, a.TanOutV,
		a.TanInBound, a.TanOutBound, a.Length, a.LengthBound,
		a.LengthUpper, a.CoordUpper, a.AxisRadiusUpper, a.AxisMomentUpper,
		a.StartVBound, a.EndVBound, a.CVBound, a.CU, a.CV,
		a.Radius, a.RadiusBound, a.Th0, a.Th1}
	y := [...]float64{b.StartU, b.StartV, b.EndU, b.EndV,
		b.StartBound.U, b.StartBound.V, b.EndBound.U, b.EndBound.V,
		b.TanInU, b.TanInV, b.TanOutU, b.TanOutV,
		b.TanInBound, b.TanOutBound, b.Length, b.LengthBound,
		b.LengthUpper, b.CoordUpper, b.AxisRadiusUpper, b.AxisMomentUpper,
		b.StartVBound, b.EndVBound, b.CVBound, b.CU, b.CV,
		b.Radius, b.RadiusBound, b.Th0, b.Th1}
	for i := range x {
		if math.Float64bits(x[i]) != math.Float64bits(y[i]) {
			return false
		}
	}
	return true
}
