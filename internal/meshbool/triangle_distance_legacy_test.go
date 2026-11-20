package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The functions below are TriTriDistance, TriTriDistanceExceeds and the
// replay distance as they were before the nearest edge pair's offset was
// carried out of the edge loop: the closest-edge axis recomputes the nine
// segment offsets. TestTriTriDistanceEdgeOffsetMatchesLegacy holds the
// current code to them bit for bit.

func legacyTriTriDistanceExceedsReplay(ctx context.Context, ta, tb [3]r3.Vec, slack float64, b *proofbound.ReplayBudget) (bool, error) {
	if err := b.SpendWork(ctx, replayDistanceSetupWork); err != nil {
		return false, err
	}
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		far, err := legacyTriTriDistanceReplay(ctx, ta, tb, slack, false, b)
		if err != nil || !far {
			return false, err
		}
		return replayTriDistanceExactAbove(ctx, ta, tb, slack, b)
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
			far, err := legacyTriTriDistanceReplay(ctx, ta, tb, slack, true, b)
			if err != nil || !far {
				return false, err
			}
			return legacyReplayRemainingAxesOrExact(ctx, ta, tb, na, nb, m, thr, slack, b)
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
func legacyReplayRemainingAxesOrExact(ctx context.Context, ta, tb [3]r3.Vec, na, nb r3.Vec, m, thr, slack float64,
	b *proofbound.ReplayBudget) (bool, error) {
	if err := b.SpendWork(ctx, 9*replayDistanceSegmentWork+replayDistanceAxisWork); err != nil {
		return false, err
	}
	if legacyClosestEdgeAxisGapExceeds(ta, tb, m, thr) {
		return true, nil
	}
	for i := range 3 {
		if err := b.SpendWork(ctx, 2*(replayDistanceCrossWork+replayDistanceAxisWork)); err != nil {
			return false, err
		}
		for _, axis := range [2]r3.Vec{ta[(i+1)%3].Sub(ta[i]).Cross(na), tb[(i+1)%3].Sub(tb[i]).Cross(nb)} {
			if axisGapExceeds(axis, ta, tb, m, thr) {
				return true, nil
			}
		}
	}
	return replayTriDistanceExactAbove(ctx, ta, tb, slack, b)
}
func legacyTriTriDistanceReplay(ctx context.Context, ta, tb [3]r3.Vec, slack float64, rejectEarly bool, b *proofbound.ReplayBudget) (bool, error) {
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

func legacyTriTriDistanceExceeds(ta, tb [3]r3.Vec, slack float64) bool {
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		return legacyTriTriDistance(ta, tb) > slack && triDistanceExactAbove(ta, tb, slack)
	}
	thr := slack + (slack+m)*0x1p-26 + 0x1p-199
	na, nb := triNormal(ta), triNormal(tb)
	clearA := axisGapExceeds(na, ta, tb, m, thr)
	clearB := axisGapExceeds(nb, ta, tb, m, thr)
	if !clearA && !clearB {
		return legacyTriTriDistance(ta, tb) > slack && (legacyClosestEdgeAxisGapExceeds(ta, tb, m, thr) ||
			edgeAxesGapExceeds(ta, tb, na, nb, m, thr) || triDistanceExactAbove(ta, tb, slack))
	}
	for i := range 3 {
		if !clearB {
			if d, foot := pointTriFoot(ta[i], tb, nb); foot && !distanceCandidateExceeds(d, slack) {
				return false
			}
		}
		if !clearA {
			if d, foot := pointTriFoot(tb[i], ta, na); foot && !distanceCandidateExceeds(d, slack) {
				return false
			}
		}
	}
	return true
}
func legacyClosestEdgeAxisGapExceeds(ta, tb [3]r3.Vec, m, thr float64) bool {
	best := posInf
	var axis r3.Vec
	for i := range 3 {
		for j := range 3 {
			off := segSegOffset(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3])
			if d := off.Len(); d < best {
				best, axis = d, off
			}
		}
	}
	return axisGapExceeds(axis, ta, tb, m, thr)
}

func legacyTriTriDistance(ta, tb [3]r3.Vec) float64 {
	best := posInf
	for i := range 3 {
		for j := range 3 {
			best = minFloat64(best, SegSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]))
		}
	}
	for i := range 3 {
		best = minFloat64(best, PointTriDistance(ta[i], tb))
		best = minFloat64(best, PointTriDistance(tb[i], ta))
	}
	if best <= 0 || proofbound.IsNonFinite(best) {
		return 0
	}
	return best * (1 - 1e-12)
}

// legacyClosestEdgeOffset is the axis legacyClosestEdgeAxisGapExceeds builds.
func legacyClosestEdgeOffset(ta, tb [3]r3.Vec) r3.Vec {
	best := posInf
	var axis r3.Vec
	for i := range 3 {
		for j := range 3 {
			off := segSegOffset(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3])
			if d := off.Len(); d < best {
				best, axis = d, off
			}
		}
	}
	return axis
}

// requireRawBitsf requires got to carry want's exact bits, NaN payload and
// sign included. The current and legacy code run the same float operations
// on one machine, so even a NaN must match bit for bit; requireSameBitsf
// lets any two NaNs match because math.Min's payload differs across
// architectures, which this comparison does not involve.
func requireRawBitsf(t *testing.T, want, got float64, format string, args ...any) {
	t.Helper()
	if math.Float64bits(want) != math.Float64bits(got) {
		require.FailNow(t, "bit mismatch", "%s: want %v (%#x) got %v (%#x)", fmt.Sprintf(format, args...),
			want, math.Float64bits(want), got, math.Float64bits(got))
	}
}

// requireEdgeOffsetMatchesLegacy holds one pair at one slack to the legacy
// code: the float reading and the nearest-edge offset bit for bit, the
// replay distance's full-scan offset, the plain answer, and the replay's
// answer, error and charged work, on a trial budget and, when tight is set,
// on budgets cut to the legacy work less one and to half of it.
func requireEdgeOffsetMatchesLegacy(t *testing.T, ta, tb [3]r3.Vec, slack float64, tight bool, label string) {
	t.Helper()
	d, offset := triTriDistanceEdgeOffset(ta, tb)
	requireRawBitsf(t, legacyTriTriDistance(ta, tb), d, "%s distance", label)
	want := legacyClosestEdgeOffset(ta, tb)
	requireRawBitsf(t, want.X, offset.X, "%s offset X", label)
	requireRawBitsf(t, want.Y, offset.Y, "%s offset Y", label)
	requireRawBitsf(t, want.Z, offset.Z, "%s offset Z", label)
	_, replayOffset, err := triTriDistanceReplay(t.Context(), ta, tb, slack, false, proofbound.NewReplayTrialBudget())
	require.NoError(t, err, label)
	requireRawBitsf(t, want.X, replayOffset.X, "%s replay offset X", label)
	requireRawBitsf(t, want.Y, replayOffset.Y, "%s replay offset Y", label)
	requireRawBitsf(t, want.Z, replayOffset.Z, "%s replay offset Z", label)
	require.Equal(t, legacyTriTriDistanceExceeds(ta, tb, slack), TriTriDistanceExceeds(ta, tb, slack), label)

	legacyBudget, budget := proofbound.NewReplayTrialBudget(), proofbound.NewReplayTrialBudget()
	wantFar, wantErr := legacyTriTriDistanceExceedsReplay(t.Context(), ta, tb, slack, legacyBudget)
	gotFar, gotErr := triTriDistanceExceedsReplay(t.Context(), ta, tb, slack, budget)
	require.Equal(t, wantErr, gotErr, label)
	require.Equal(t, wantFar, gotFar, label)
	wantWork, wantRecords := legacyBudget.Usage()
	gotWork, gotRecords := budget.Usage()
	require.Equal(t, wantWork, gotWork, "%s replay work", label)
	require.Equal(t, wantRecords, gotRecords, "%s replay records", label)
	if !tight || wantWork < 2 {
		return
	}
	for _, limit := range []uint64{wantWork - 1, wantWork / 2} {
		legacyCut, err := proofbound.NewReplayBudgetLimits(limit, proofbound.ReplayTrialRecords, proofbound.ReplayTrialBytes)
		require.NoError(t, err)
		cut, err := proofbound.NewReplayBudgetLimits(limit, proofbound.ReplayTrialRecords, proofbound.ReplayTrialBytes)
		require.NoError(t, err)
		wantFar, wantErr := legacyTriTriDistanceExceedsReplay(t.Context(), ta, tb, slack, legacyCut)
		gotFar, gotErr := triTriDistanceExceedsReplay(t.Context(), ta, tb, slack, cut)
		require.Error(t, wantErr, "%s limit %d", label, limit)
		require.Equal(t, wantErr.Error(), gotErr.Error(), "%s limit %d", label, limit)
		require.Equal(t, wantFar, gotFar, "%s limit %d", label, limit)
		wantWork, _ := legacyCut.Usage()
		gotWork, _ := cut.Usage()
		require.Equal(t, wantWork, gotWork, "%s limit %d work", label, limit)
	}
}

// TestTriTriDistanceEdgeOffsetMatchesLegacy holds the distance, the
// nearest-edge offset, both answers and the replay's charges to the legacy
// code, which recomputes the nine segment offsets for the offset axis. It
// sweeps 60,000 exceedsPair pairs (3,000 under the race detector), each at
// one slack from exceedsSlacks cycling through the list, and the near-miss
// tables' pairs at slacks around each pair's exact distance. Every 16th sweep pair and every table pair
// also runs the replay on budgets cut below its need.
func TestTriTriDistanceEdgeOffsetMatchesLegacy(t *testing.T) {
	t.Parallel()
	n, samples := 60_000, 4
	if raceDetector {
		n, samples = 3_000, 1
	}
	rng := rand.New(rand.NewPCG(41, 42))
	ties := 0
	for i := range n {
		mode := i % exceedsPairModes
		ta, tb := exceedsPair(rng, mode)
		slacks := exceedsSlacks(TriTriDistance(ta, tb))
		slack := slacks[(i/exceedsPairModes)%len(slacks)]
		requireEdgeOffsetMatchesLegacy(t, ta, tb, slack, i%16 == 0, fmt.Sprintf("pair %d mode %d slack %v", i, mode, slack))
		if edgeLengthTie(ta, tb) {
			ties++
		}
	}
	require.Positive(t, ties, "the sweep must reach two nearest edge pairs of equal length")
	rng = rand.New(rand.NewPCG(21, 22))
	for _, c := range nearMissConfigs {
		for _, scale := range []float64{1, 100} {
			for _, gap := range []float64{1e-9, 1e-6, 1e-3} {
				for k := range samples {
					ta, tb := nearMissPair(rng, c.kind, scale, gap, c.theta)
					d := TriTriDistance(ta, tb)
					if d2, ok := refTriDistance2(ta, tb); ok && d2.n.Sign() > 0 {
						d = refDistanceFloat(d2)
					}
					for _, slack := range []float64{d, math.Nextafter(d, math.Inf(-1)), d * (1 + 1e-9), d * (1 - 1e-9),
						d * (1 + 1e-6), d * (1 - 1e-6), d / 2} {
						requireEdgeOffsetMatchesLegacy(t, ta, tb, slack, true,
							fmt.Sprintf("%s theta %v scale %v gap %v sample %d slack %v", c.kind, c.theta, scale, gap, k, slack))
					}
				}
			}
		}
	}
}

// edgeLengthTie reports whether the smallest of the nine edge-pair offset
// lengths is reached by two edge pairs.
func edgeLengthTie(ta, tb [3]r3.Vec) bool {
	best, count := posInf, 0
	for i := range 3 {
		for j := range 3 {
			d := segSegOffset(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]).Len()
			switch {
			case d < best:
				best, count = d, 1
			case d == best:
				count++
			}
		}
	}
	return count > 1
}
