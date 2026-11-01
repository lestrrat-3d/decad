package meshbool

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The envelopes count a vector add, subtract or scale as three operations, a
// dot as five, a cross as nine, a length as six, and a clamp as four. A segment
// candidate has three vector differences, five dots, at most four clamps,
// two scales, two sums, one final difference and length, plus fewer than 32
// scalar arithmetic and branch operations: fewer than 128 of the 192 charged.
// A point/triangle foot has at most three edge tests, each with two vector
// differences, a cross and a dot, plus fewer than 64 other operations: fewer
// than 128 of the 256 charged. An axis has seven dots, one length, and fewer
// than 64 scalar operations and comparisons: fewer than 128 of 256. Setup
// scans 18 scalar coordinates, makes two normals and a threshold; 192 covers
// those operations and their branches. A point/segment fallback is bounded by
// the segment envelope, including its degenerate-segment path.
const (
	replayDistanceSetupWork   = 192
	replayDistanceAxisWork    = 256
	replayDistanceFootWork    = 256
	replayDistanceSegmentWork = 192
	replayDistanceNormalWork  = 64
	replayDistanceCrossWork   = 32
)

func triTriDistanceExceedsReplay(ctx context.Context, ta, tb [3]r3.Vec, slack float64, b *proofbound.ReplayBudget) (bool, error) {
	if err := b.SpendWork(ctx, replayDistanceSetupWork); err != nil {
		return false, err
	}
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		return triTriDistanceReplay(ctx, ta, tb, slack, false, b)
	}
	thr := slack + (slack+m)*0x1p-26 + 0x1p-199
	na, nb := triNormal(ta), triNormal(tb)
	if err := b.SpendWork(ctx, replayDistanceAxisWork); err != nil {
		return false, err
	}
	clearA := axisGapExceeds(na, ta, tb, m, thr)
	if err := b.SpendWork(ctx, replayDistanceAxisWork); err != nil {
		return false, err
	}
	clearB := axisGapExceeds(nb, ta, tb, m, thr)
	if !clearA && !clearB {
		// Any separating axis can prove the edge candidates far. Unlike
		// the face normals, an edge-cross axis does not exclude either set
		// of perpendicular-foot candidates, so both sets remain below.
		separated := false
		for i := range 3 {
			for j := range 3 {
				if err := b.SpendWork(ctx, replayDistanceCrossWork+replayDistanceAxisWork); err != nil {
					return false, err
				}
				axis := ta[(i+1)%3].Sub(ta[i]).Cross(tb[(j+1)%3].Sub(tb[j]))
				if axisGapExceeds(axis, ta, tb, m, thr) {
					separated = true
					break
				}
			}
			if separated {
				break
			}
		}
		if !separated {
			return triTriDistanceReplay(ctx, ta, tb, slack, true, b)
		}
	}
	for i := range 3 {
		if !clearB {
			if err := b.SpendWork(ctx, replayDistanceFootWork); err != nil {
				return false, err
			}
			if d, foot := pointTriFoot(ta[i], tb, nb); foot && !distanceCandidateExceeds(d, slack) {
				return false, nil
			}
		}
		if !clearA {
			if err := b.SpendWork(ctx, replayDistanceFootWork); err != nil {
				return false, err
			}
			if d, foot := pointTriFoot(tb[i], ta, na); foot && !distanceCandidateExceeds(d, slack) {
				return false, nil
			}
		}
	}
	return true, nil
}

func triTriDistanceReplay(ctx context.Context, ta, tb [3]r3.Vec, slack float64, rejectEarly bool, b *proofbound.ReplayBudget) (bool, error) {
	best := posInf
	for i := range 3 {
		for j := range 3 {
			if err := b.SpendWork(ctx, replayDistanceSegmentWork); err != nil {
				return false, err
			}
			d := SegSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3])
			if rejectEarly && !distanceCandidateExceeds(d, slack) {
				return false, nil
			}
			best = minFloat64(best, d)
		}
	}
	point := func(p r3.Vec, t [3]r3.Vec) (float64, error) {
		if err := b.SpendWork(ctx, replayDistanceNormalWork+replayDistanceFootWork); err != nil {
			return 0, err
		}
		if d, ok := pointTriFoot(p, t, triNormal(t)); ok {
			return d, nil
		}
		minimum := posInf
		for i := range 3 {
			if err := b.SpendWork(ctx, replayDistanceSegmentWork); err != nil {
				return 0, err
			}
			minimum = minFloat64(minimum, pointSegDist3(p, t[i], t[(i+1)%3]))
		}
		return minimum, nil
	}
	for i := range 3 {
		d, err := point(ta[i], tb)
		if err != nil {
			return false, err
		}
		if rejectEarly && !distanceCandidateExceeds(d, slack) {
			return false, nil
		}
		best = minFloat64(best, d)
		d, err = point(tb[i], ta)
		if err != nil {
			return false, err
		}
		if rejectEarly && !distanceCandidateExceeds(d, slack) {
			return false, nil
		}
		best = minFloat64(best, d)
	}
	if best <= 0 || proofbound.IsNonFinite(best) {
		best = 0
	} else {
		best *= 1 - 1e-12
	}
	return best > slack, nil
}
