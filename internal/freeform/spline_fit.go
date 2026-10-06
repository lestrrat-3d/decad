package freeform

import (
	"math/big"
)

// FitInterpolantCostPerPoint is the conservative per-fit-point charge behind
// FitInterpolantCost. Linear, with NO quadratic term — unlike a knot
// insertion's ClampedConversionCost, a natural cubic interpolant gives one
// span per interval directly and there is no insertion pass to charge for.
// The ~40 units the interpolant solve, the rational lift and this file's own
// closed form actually cost (dedup + chord accumulation, the tridiagonal
// solve's two Thomas passes, the interpolant export, decad's rational lift of
// the three returned slices, and the per-span closed form) round up to 64 as
// the conservative figure, per §5.2's rule that the constant is backed by a
// measured boundary regression rather than by this accounting alone.
//
// This charge is a FLOOR, not the binding constraint: for a record holding a
// lone fit spline, the reconstruction charge (FreeformChords, already keyed on
// FitSplineSeg in spline_bezier.go's reconstructionChords) dominates well
// before this one does.
const FitInterpolantCostPerPoint = 64

// FitInterpolantCost is the size-derived charge for building and lifting one
// fit spline's interpolant, read from the recorded fit-point count alone —
// before geom.NewFitInterpolant allocates anything.
func FitInterpolantCost(n int) uint64 {
	return CostMul(FitInterpolantCostPerPoint, uint64(n))
}

// ChargeFitInterpolant levies FitInterpolantCost against the record's counter.
func ChargeFitInterpolant(work *FreeformWork, n int) error {
	return work.Step(FitInterpolantCost(n))
}

// FitSpanControls returns one coordinate's two INTERIOR Bézier control values
// for a fit-spline span, docs/spline-design.md §1.3's closed form:
//
//	b1 = (2·vi + vi1)/3 − hSq·(2·mi + mi1)/18
//	b2 = (vi + 2·vi1)/3 − hSq·(mi + 2·mi1)/18
//
// with vi/vi1 the span's endpoint values (its Bézier b0/b3, interpolated
// exactly), mi/mi1 the natural-cubic second derivatives at those same ends,
// and hSq the span width squared, all exact rationals. Every operand is a
// big.Rat, so nothing here rounds — cross-checked against the monomial route
// in spline_fit_internal_test.go, and against FitSpan's own independent
// conversion as an oracle.
func FitSpanControls(vi, vi1, mi, mi1, hSq *big.Rat) (b1, b2 *big.Rat) {
	three := big.NewRat(3, 1)
	eighteen := big.NewRat(18, 1)

	b1 = new(big.Rat).Add(new(big.Rat).Add(vi, vi), vi1)
	b1.Quo(b1, three)
	t1 := new(big.Rat).Add(new(big.Rat).Add(mi, mi), mi1)
	t1.Mul(t1, hSq)
	t1.Quo(t1, eighteen)
	b1.Sub(b1, t1)

	b2 = new(big.Rat).Add(vi, new(big.Rat).Add(vi1, vi1))
	b2.Quo(b2, three)
	t2 := new(big.Rat).Add(mi, new(big.Rat).Add(mi1, mi1))
	t2.Mul(t2, hSq)
	t2.Quo(t2, eighteen)
	b2.Sub(b2, t2)

	return b1, b2
}
