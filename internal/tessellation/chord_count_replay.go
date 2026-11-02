package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ReplayChordCount charges every sagitta trial. It preserves ChordCount's
// answer when the float bound is positive and refuses the rare underflow
// branch whose exact-rational fallback has no replay meter yet.
func ReplayChordCount(ctx context.Context, walk survey2d.SegmentWalk, tol float64,
	nMin int, budget *proofbound.ReplayBudget) (int, float64, error) {
	if budget == nil || proofbound.IsNonFinite(tol) || tol <= 0 ||
		proofbound.IsNonFinite(walk.Radius) || walk.Radius < 0 ||
		proofbound.IsNonFinite(walk.Th0) || proofbound.IsNonFinite(walk.Th1) {
		return 0, 0, decaderr.ErrUnsupported
	}
	if err := budget.ReserveBundle(ctx, 16, 2, 128); err != nil {
		return 0, 0, err
	}
	sweep := math.Abs(walk.Th1 - walk.Th0)
	if proofbound.IsNonFinite(sweep) {
		return 0, 0, decaderr.ErrUnsupported
	}
	if nMin < 1 {
		nMin = 1
	}
	if nMin > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	sag, err := replayChordSagitta(ctx, walk.Radius, sweep, nMin, budget)
	if err != nil {
		return 0, 0, err
	}
	if sag <= tol {
		return nMin, sag, nil
	}
	maxD := math.Pi
	if tol < walk.Radius {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return 0, 0, err
		}
		maxD = 4 * math.Asin(math.Sqrt(tol/(2*walk.Radius)))
	}
	if maxD == 0 || sweep/maxD > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	n := max(int(math.Ceil(sweep/maxD)), nMin)
	if n > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	sag, err = replayChordSagitta(ctx, walk.Radius, sweep, n, budget)
	if err != nil {
		return 0, 0, err
	}
	for sag > tol && sweep > 0 {
		if n == freeform.MaxChordsPerWalk {
			return 0, 0, freeform.ErrTooManyChords
		}
		n++
		sag, err = replayChordSagitta(ctx, walk.Radius, sweep, n, budget)
		if err != nil {
			return 0, 0, err
		}
	}
	return n, sag, nil
}

func replayChordSagitta(ctx context.Context, radius, sweep float64, n int,
	budget *proofbound.ReplayBudget) (float64, error) {
	if err := budget.SpendWork(ctx, 16); err != nil {
		return 0, err
	}
	if n <= 0 || sweep < 0 || radius < 0 || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(sweep) {
		return 0, decaderr.ErrUnsupported
	}
	if radius == 0 || sweep == 0 {
		return 0, nil
	}
	denom := 8 * float64(n) * float64(n)
	sag := proofbound.UpRound(proofbound.ProductUpper(radius,
		proofbound.ProductUpper(sweep, sweep)) / denom)
	if sag <= 0 || proofbound.IsNonFinite(sag) {
		return 0, decaderr.ErrUnsupported
	}
	return sag, nil
}
