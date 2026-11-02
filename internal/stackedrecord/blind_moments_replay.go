package stackedrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// replayPiScalar proves that the ordinary float for pi is enclosed by its
// next float. The comparison is charged against the same meter as the circle.
func replayPiScalar(ctx context.Context, budget *proofbound.ReplayBudget) (proofbound.BoundedScalar, error) {
	if budget == nil {
		return proofbound.BoundedScalar{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return proofbound.BoundedScalar{}, err
	}
	next := math.Nextafter(math.Pi, math.Inf(1))
	lo, err := proof.ReplayFloatRat(ctx, math.Pi, budget)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	hi, err := proof.ReplayFloatRat(ctx, next, budget)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	first, err := proof.ReplayRatCompare(ctx, lo, proofbound.PiLower, budget)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	last, err := proof.ReplayRatCompare(ctx, proofbound.PiUpper, hi, budget)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	if first > 0 || last > 0 {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: pi enclosure is wider than one float step", decaderr.ErrUnsupported)
	}
	return proofbound.MeasuredScalar(math.Pi, next-math.Pi), nil
}

// replayWholeCircleMoments integrates an entire recorded circle from its
// closed form. Its radius and centre are exact millimetre float denotations.
// The source record has already proved the loop's orientation and placement.
func replayWholeCircleMoments(ctx context.Context, loop sectionrecord.LoopRecord,
	budget *proofbound.ReplayBudget) (area, mu, mv proofbound.BoundedScalar, ok bool, err error) {
	if budget == nil {
		return area, mu, mv, false, decaderr.ErrUnsupported
	}
	if err = budget.SpendWork(ctx, 1); err != nil {
		return
	}
	if len(loop.Segments) != 1 {
		return
	}
	c, circle := loop.Segments[0].(sectionrecord.CircleSeg)
	if !circle || c.Radius.Unit() != units.Millimeter ||
		proofbound.IsNonFinite(c.Radius.Mag()) || c.Radius.Mag() <= 0 ||
		proofbound.IsNonFinite(c.Center.U) || proofbound.IsNonFinite(c.Center.V) ||
		!(c.CCW && c.TStart == 0 && c.TEnd == 1 ||
			!c.CCW && c.TStart == 1 && c.TEnd == 0) {
		return
	}
	pi, e := replayPiScalar(ctx, budget)
	if e != nil {
		err = e
		return
	}
	r := proofbound.ExactScalar(c.Radius.Mag())
	r2, e := proofbound.BoundedMulReplay(ctx, r, r, budget)
	if e != nil {
		err = e
		return
	}
	area, e = proofbound.BoundedMulReplay(ctx, pi, r2, budget)
	if e != nil {
		err = e
		return
	}
	mu, e = proofbound.BoundedMulReplay(ctx, area, proofbound.ExactScalar(c.Center.U), budget)
	if e != nil {
		err = e
		return
	}
	mv, e = proofbound.BoundedMulReplay(ctx, area, proofbound.ExactScalar(c.Center.V), budget)
	if e != nil {
		err = e
		return
	}
	if !c.CCW {
		area, mu, mv = proofbound.BoundedNeg(area), proofbound.BoundedNeg(mu), proofbound.BoundedNeg(mv)
	}
	if proofbound.IsNonFinite(area.Value) || proofbound.IsNonFinite(area.Bound) ||
		proofbound.IsNonFinite(mu.Value) || proofbound.IsNonFinite(mu.Bound) ||
		proofbound.IsNonFinite(mv.Value) || proofbound.IsNonFinite(mv.Bound) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, false, nil
	}
	return area, mu, mv, true, nil
}

// replayBlindRegionMoments starts with the producer's measured region and adds
// only the new, source-bound circular hole loops. Original holes remain part
// of the producer's first moments and are never integrated twice.
func replayBlindRegionMoments(ctx context.Context, region momentinput.Profile,
	target prismrecord.Payload, budget *proofbound.ReplayBudget) (momentinput.Integrals, bool, error) {
	if budget == nil {
		return momentinput.Integrals{}, false, decaderr.ErrUnsupported
	}
	if target.Walks == nil || len(region.Holes) < len(target.Profile.Holes) {
		return momentinput.Integrals{}, false, nil
	}
	base, ok, err := target.Walks.FirstMomentsReplay(ctx, target.Profile, budget)
	if err != nil || !ok {
		return momentinput.Integrals{}, false, err
	}
	same, err := replayEqualBlindLoop(ctx, region.Outer, target.Profile.Outer, budget)
	if err != nil || !same {
		return momentinput.Integrals{}, false, err
	}
	for i, hole := range target.Profile.Holes {
		same, err = replayEqualBlindLoop(ctx, region.Holes[i], hole, budget)
		if err != nil || !same {
			return momentinput.Integrals{}, false, err
		}
	}
	area := proofbound.MeasuredScalar(base.Area, base.AreaBound)
	mu := proofbound.MeasuredScalar(base.Mu, base.MuBound)
	mv := proofbound.MeasuredScalar(base.Mv, base.MvBound)
	coordUpper := base.CoordUpper
	for _, hole := range region.Holes[len(target.Profile.Holes):] {
		a, u, v, circle, e := replayWholeCircleMoments(ctx, hole, budget)
		if e != nil || !circle || a.Value >= 0 {
			return momentinput.Integrals{}, false, e
		}
		if err := budget.SpendWork(ctx, 2); err != nil {
			return momentinput.Integrals{}, false, err
		}
		c, ok := hole.Segments[0].(sectionrecord.CircleSeg)
		if !ok {
			return momentinput.Integrals{}, false, nil
		}
		coordUpper = max(coordUpper, proofbound.AbsSumUpper(c.Center.U, c.Center.V,
			proofbound.ProductUpper(math.Sqrt2, c.Radius.Mag())))
		area, err = proofbound.BoundedAddReplay(ctx, area, a, budget)
		if err != nil {
			return momentinput.Integrals{}, false, err
		}
		mu, err = proofbound.BoundedAddReplay(ctx, mu, u, budget)
		if err != nil {
			return momentinput.Integrals{}, false, err
		}
		mv, err = proofbound.BoundedAddReplay(ctx, mv, v, budget)
		if err != nil {
			return momentinput.Integrals{}, false, err
		}
	}
	if area.Value <= 0 || proofbound.IsNonFinite(area.Bound) ||
		proofbound.IsNonFinite(mu.Bound) || proofbound.IsNonFinite(mv.Bound) {
		return momentinput.Integrals{}, false, nil
	}
	return momentinput.Integrals{CoordUpper: coordUpper, Area: area.Value, AreaBound: area.Bound,
		Mu: mu.Value, MuBound: mu.Bound, Mv: mv.Value, MvBound: mv.Bound}, true, nil
}

// patchAreaReplay reads the certified single-circle floor or ceiling and
// carries the section displacement through the same area bound as patchArea.
func patchAreaReplay(ctx context.Context, sectionDelta float64, colDelta []float64,
	columns []WallColumn, patch Patch, budget *proofbound.ReplayBudget) (proofbound.BoundedScalar, error) {
	if budget == nil {
		return proofbound.BoundedScalar{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return proofbound.BoundedScalar{}, err
	}
	if len(colDelta) != len(columns) || len(patch.Record.Holes) != 0 || len(patch.Loops) != 1 ||
		patch.Loops[0].Column < 0 || patch.Loops[0].Column >= len(columns) {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: unsupported blind patch", decaderr.ErrUnsupported)
	}
	a, _, _, ok, err := replayWholeCircleMoments(ctx, patch.Record.Outer, budget)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	if !ok || a.Value <= 0 {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: unsupported blind patch area", decaderr.ErrUnsupported)
	}
	col := columns[patch.Loops[0].Column]
	if len(col.Loop.Segments) != 1 || colDelta[patch.Loops[0].Column] != 0 ||
		proofbound.IsNonFinite(sectionDelta) || sectionDelta < 0 {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: unsupported blind patch displacement", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 3); err != nil {
		return proofbound.BoundedScalar{}, err
	}
	perimeter := proofbound.AbsSumUpper(col.Perimeter.Value, col.Perimeter.Bound)
	a.Bound = proofbound.AbsSumUpper(a.Bound, proofbound.SectionDisplacementArea(sectionDelta, 1, perimeter))
	return a, nil
}
