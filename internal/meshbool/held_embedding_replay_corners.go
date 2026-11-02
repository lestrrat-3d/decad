package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// replayFloatBoxCorners preserves FloatBoxCorners' order without unmetered
// rational division or comparison. The returned array has at most eight points.
func replayFloatBoxCorners(ctx context.Context, p proof.Xpt, budget *proofbound.ReplayBudget) ([8]r3.Vec, int, error) {
	var out [8]r3.Vec
	if budget == nil || p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 {
		return out, 0, errReplayInvalidReplayFloatBox
	}
	var brackets [3][2]float64
	var counts [3]int
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		f, err := proof.ReplayFractionFloat64(ctx, num, p.W, budget)
		if err != nil {
			return out, 0, err
		}
		if math.IsInf(f, 0) || math.IsNaN(f) {
			return out, 0, errReplayNonFiniteReplayFloatBox
		}
		q, err := proof.ReplayFloatRat(ctx, f, budget)
		if err != nil {
			return out, 0, err
		}
		d, err := proof.ReplayRatProductDifference(ctx, q, p.W, num, budget)
		if err != nil {
			return out, 0, err
		}
		counts[i] = 1
		brackets[i][0] = f
		switch d.Sign() {
		case -1:
			counts[i], brackets[i][1] = 2, math.Nextafter(f, math.Inf(1))
		case 1:
			counts[i], brackets[i][0], brackets[i][1] = 2, math.Nextafter(f, math.Inf(-1)), f
		}
	}
	if err := budget.SpendWork(ctx, uint64(counts[0]*counts[1]*counts[2])); err != nil {
		return out, 0, err
	}
	n := 0
	for i := range counts[0] {
		for j := range counts[1] {
			for k := range counts[2] {
				out[n] = r3.Vec{X: brackets[0][i], Y: brackets[1][j], Z: brackets[2][k]}
				n++
			}
		}
	}
	return out, n, nil
}
