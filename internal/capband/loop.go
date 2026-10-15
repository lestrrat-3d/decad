package capband

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file builds a cap loop's coalesced walks, offset joins, contour
// displacement and offset boundary from one recorded loop, and states the
// cap-loop chamfer's offset refusals (docs/modify-reach-design.md §8.3).

// LoopWalks decomposes a single recorded loop into its coalesced
// corner walk, the same decomposition offset2d.ProfileCornerLoopsBudget applies to
// every loop of a section.
func LoopWalks(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, work *freeform.FreeformWork) ([]survey2d.SideWalk, error) {
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return nil, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "a cap-loop chamfer"); err != nil {
			return nil, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceWalksBudget(raw, budget)
	if err != nil {
		return nil, err
	}
	return walks, nil
}

// OffsetJoins returns the shared Shell offset joins after checking the
// cap band's circular walls. The caller pairs each join with its source wall.
func OffsetJoins(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, d, tol float64) ([]offset2d.Join, error) {
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: a cap loop holds no walks`, decaderr.ErrDegenerate)
	}
	for _, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.IsCircular() {
			if _, err := BandRadius(w, d, tol); err != nil {
				return nil, err
			}
		}
	}
	return offset2d.SectionJoinsBudget(budget, walks, 1, d, tol)
}

// ContourJoins converts offset joins into the capcontour join readings the contour and closure proofs take.
func ContourJoins(joins []offset2d.Join) []capcontour.Join {
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{
			Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M: j.M, PA: j.PA, PB: j.PB,
		}
	}
	return readings
}

// LoopContourDelta re-derives one loop's contour displacement from its record
// for readings that hold no built band. It uses the same walks and joins as
// capblend.BuildBand.
func LoopContourDelta(ctx context.Context, loop sectionrecord.LoopRecord, d, dDelta, tol float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	walks, err := LoopWalks(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(walks) == 1 && walks[0].Closed {
		return WholeCircleDisplacement(walks[0], d, dDelta, tol)
	}
	joins, err := OffsetJoins(budget, walks, d, tol)
	if err != nil {
		return 0, err
	}
	return ContourDisplacement(walks, ContourJoins(joins), d, dDelta, tol)
}

// LoopBoundary returns loop_li's OWN offset-by-d boundary (d the cap's dc)
// as a standalone
// LoopRecord, used to compute a chamfered cap's per-loop enclosed area and
// the band's closing disk at the cap level.
func LoopBoundary(ctx context.Context, loop sectionrecord.LoopRecord, d, tol float64) (sectionrecord.LoopRecord, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	walks, err := LoopWalks(budget, loop, work)
	if err != nil {
		return sectionrecord.LoopRecord{}, err
	}
	segs, err := offset2d.BuildLoop(budget, walks, 1, d, tol)
	if err != nil {
		return sectionrecord.LoopRecord{}, err
	}
	return sectionrecord.LoopRecord{Segments: segs}, nil
}

// ChamferDropError re-sentinels the shared offset's own drop refusal as
// SX6 (docs/modify-reach-design.md Table SX). The offset code is Shell's, and
// there its drop is S11a — ErrUnsupported, because the shell body exists and
// only a trimmed-offset kernel is missing. A CAP-LOOP CHAMFER asks a different
// question of the same machinery: the drop says the selected loop has no
// regular radius-d envelope — the contour is empty (offsetting a radius-4
// circle inward by 4 leaves nothing) or merely irregular (a fillet arc whose
// offset radius reaches zero collapses to a sharp corner) — so the body the
// caller named does not exist as a cap-loop chamfer. §4's existence test puts
// that in stage 5 with ErrDegenerate. The translation lives here, at the one
// call site that asks the existence question, never on offset2d.ErrDrop itself —
// Shell's own sentinel is correct for Shell. The result does not wrap
// offset2d.ErrDrop: carrying its ErrUnsupported along would leave the refusal
// answering to both sentinels, and a caller branching on either would be
// right, which is the ambiguity §4's one-sentinel rule exists to prevent.
func ChamferDropError(err error) error {
	if !errors.Is(err, offset2d.ErrDrop) {
		return err
	}
	return fmt.Errorf(`%w: the cap-loop offset drops a section feature (a circular wall's offset radius reaches zero, or a walk is consumed), so the selected loop has no regular cap contour at this setback`, decaderr.ErrDegenerate)
}

// ChamferAuditError relabels the shared offset audit's refusal with the
// wording of the stage-6 row it came from, and preserves the sentinel that row
// already decided. SX6 (drop) is settled by capblend.MixedOffsetProfile's own call to
// offset2d.BuildLoop before the audit runs, so a refusal reaching here is one of
// the audit's own: the base S8/S9 nesting rows, which say the offset section is
// decidably broken and therefore that no such body exists (ErrDegenerate), or
// SX7/SX12, which say the body exists and this evaluator cannot trim or merge
// it (ErrUnsupported). Those are opposite existence claims (§4's one-sentinel
// rule), so an S9 refusal keeps ErrDegenerate through %w and errors.Is keeps
// branching on it; only the ErrUnsupported family is restated as SX7/SX12.
func ChamferAuditError(err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	if errors.Is(err, decaderr.ErrDegenerate) {
		return fmt.Errorf(`%w; the section that broke is the cap-loop chamfer's own offset of the selected loop(s) by the cap setback`, err)
	}
	return fmt.Errorf(`%w: the cap-loop chamfer's ruled patches cannot be certified disjoint from a non-adjacent boundary (%v); a trimming kernel is not available`, decaderr.ErrUnsupported, err)
}
