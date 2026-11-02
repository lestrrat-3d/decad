package meshbool

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

var replayExactZero = big.NewInt(0)
var replayExactNegativeOne = big.NewInt(-1)

func replayCommonLineAxis(ctx context.Context, a, b proof.Xpt, budget *proofbound.ReplayBudget) (int, bool, bool, error) {
	left := [3][4]*big.Int{{a.Y, b.Z, a.Z, b.Y}, {a.Z, b.X, a.X, b.Z}, {a.X, b.Y, a.Y, b.X}}
	for i, v := range left {
		d, err := proof.ReplayProductDifference(ctx, v[0], v[1], v[2], v[3], budget)
		if err != nil {
			return -1, false, false, err
		}
		if d.Sign() != 0 {
			return i, d.Sign() < 0, true, nil
		}
	}
	return -1, false, false, nil
}

func replaySumSquares(ctx context.Context, v [3]*big.Int, budget *proofbound.ReplayBudget) (*big.Int, error) {
	var sum *big.Int
	for _, x := range v {
		var err error
		if sum == nil {
			sum, err = proof.ReplayProductDifference(ctx, x, x, replayExactZero, replayExactZero, budget)
		} else {
			sum, err = proof.ReplayProductDifference(ctx, x, x, replayExactNegativeOne, sum, budget)
		}
		if err != nil {
			return nil, err
		}
	}
	return sum, nil
}

// replaySinSquared preserves SinSquared's exact unreduced numerator and denominator.
func replaySinSquared(ctx context.Context, a, b proof.Xpt, budget *proofbound.ReplayBudget) (*SinSquare, error) {
	var cross [3]*big.Int
	terms := [3][4]*big.Int{{a.Y, b.Z, a.Z, b.Y}, {a.Z, b.X, a.X, b.Z}, {a.X, b.Y, a.Y, b.X}}
	for i, v := range terms {
		var err error
		cross[i], err = proof.ReplayProductDifference(ctx, v[0], v[1], v[2], v[3], budget)
		if err != nil {
			return nil, err
		}
	}
	num, err := replaySumSquares(ctx, cross, budget)
	if err != nil {
		return nil, err
	}
	aa, err := replaySumSquares(ctx, [3]*big.Int{a.X, a.Y, a.Z}, budget)
	if err != nil {
		return nil, err
	}
	bb, err := replaySumSquares(ctx, [3]*big.Int{b.X, b.Y, b.Z}, budget)
	if err != nil {
		return nil, err
	}
	den, err := proof.ReplayProductDifference(ctx, aa, bb, replayExactZero, replayExactZero, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(SinSquare{}))); err != nil {
		return nil, err
	}
	if den.Sign() == 0 {
		if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(big.Int{})+unsafe.Sizeof(big.Word(0)))); err != nil {
			return nil, err
		}
		return &SinSquare{Num: new(big.Int), Den: big.NewInt(1)}, nil
	}
	return &SinSquare{Num: num, Den: den}, nil
}

func replayDotNumerator(ctx context.Context, n, p proof.Xpt, budget *proofbound.ReplayBudget) (*big.Int, error) {
	var sum *big.Int
	for i, x := range [3]*big.Int{n.X, n.Y, n.Z} {
		y := [3]*big.Int{p.X, p.Y, p.Z}[i]
		var err error
		if sum == nil {
			sum, err = proof.ReplayProductDifference(ctx, x, y, replayExactZero, replayExactZero, budget)
		} else {
			sum, err = proof.ReplayProductDifference(ctx, x, y, replayExactNegativeOne, sum, budget)
		}
		if err != nil {
			return nil, err
		}
	}
	return sum, nil
}

func replayPlaneCrossings(ctx context.Context, xt [3]proof.Xpt, origin, normal proof.Xpt, signs [3]int,
	budget *proofbound.ReplayBudget) ([3]proof.Xpt, int, error) {
	var out [3]proof.Xpt
	var values [3]*big.Int
	var base *big.Int
	value := func(i int) (*big.Int, error) {
		if values[i] != nil {
			return values[i], nil
		}
		var err error
		if base == nil {
			base, err = replayDotNumerator(ctx, normal, origin, budget)
			if err != nil {
				return nil, err
			}
		}
		dot, err := replayDotNumerator(ctx, normal, xt[i], budget)
		if err != nil {
			return nil, err
		}
		values[i], err = proof.ReplayProductDifference(ctx, dot, origin.W, base, xt[i].W, budget)
		return values[i], err
	}
	n := 0
	for i := range 3 {
		if signs[i] == 0 {
			out[n] = xt[i]
			n++
		}
		j := (i + 1) % 3
		if signs[i]*signs[j] < 0 {
			a, err := value(i)
			if err != nil {
				return out, 0, err
			}
			b, err := value(j)
			if err != nil {
				return out, 0, err
			}
			out[n], err = proof.ReplayPlaneCrossing(ctx, xt[i], xt[j], a, b, budget)
			if err != nil {
				return out, 0, err
			}
			n++
		}
	}
	return out, n, nil
}

// replayCrossingContactHeld may omit the float filter only when the caller
// proved the two facets of the same held mesh share exactly one vertex.
func replayCrossingContactHeld(ctx context.Context, ta, tb [3]r3.Vec, xa, xb [3]proof.Xpt, na, nb proof.Xpt,
	sa, sb [3]int, fna, fnb r3.Vec, budget *proofbound.ReplayBudget, heldSharedOne bool) (TriContact, error) {
	out := TriContact{EdgeA: -1, EdgeB: -1}
	// The shared point belongs to both intersection spans, so the reject-only
	// float filter cannot prove them disjoint. The 256-unit local reservation
	// covers the fixed crossing loops, endpoint ordering and contact bookkeeping;
	// every exact product and allocation remains separately metered.
	charge := uint64(1024)
	if heldSharedOne {
		charge = 256
	}
	if err := budget.SpendWork(ctx, charge); err != nil {
		return out, err
	}
	if !heldSharedOne && TriTriMissesFilter(ta, tb, fna, fnb, sa, sb) {
		return out, nil
	}
	pa, an, err := replayPlaneCrossings(ctx, xa, xb[0], nb, sa, budget)
	if err != nil {
		return out, err
	}
	pb, bn, err := replayPlaneCrossings(ctx, xb, xa[0], na, sb, budget)
	if err != nil {
		return out, err
	}
	if an == 0 || bn == 0 {
		return out, nil
	}
	if an > 2 || bn > 2 {
		return out, errReplayAFacetCrossesAPlaneMoreThan
	}
	axis, reverse, ok, err := replayCommonLineAxis(ctx, na, nb, budget)
	if err != nil {
		return out, err
	}
	if !ok {
		return out, errReplayNoncoplanarContactHasNoIntersectionLine
	}
	compare := func(a, b proof.Xpt) (int, error) {
		d, err := proof.ReplayProductDifference(ctx, Coord(a, axis), b.W, Coord(b, axis), a.W, budget)
		if err != nil {
			return 0, err
		}
		value := d.Sign()
		if reverse {
			value = -value
		}
		return value, nil
	}
	sorted := func(points [3]proof.Xpt, count int) (proof.Xpt, proof.Xpt, error) {
		first, second := points[0], points[0]
		if count > 1 {
			second = points[1]
			cmp, err := compare(first, second)
			if err != nil {
				return proof.Xpt{}, proof.Xpt{}, err
			}
			if cmp > 0 {
				first, second = second, first
			}
		}
		return first, second, nil
	}
	loA, hiA, err := sorted(pa, an)
	if err != nil {
		return out, err
	}
	loB, hiB, err := sorted(pb, bn)
	if err != nil {
		return out, err
	}
	loCmp, err := compare(loB, loA)
	if err != nil {
		return out, err
	}
	hiCmp, err := compare(hiB, hiA)
	if err != nil {
		return out, err
	}
	lo, hi := loA, hiA
	if loCmp > 0 {
		lo = loB
	}
	if hiCmp < 0 {
		hi = hiB
	}
	cmp, err := compare(lo, hi)
	if err != nil {
		return out, err
	}
	if cmp > 0 {
		return out, nil
	}
	if cmp == 0 {
		out.Kind = ContactPoint
		out.P0 = lo
		out.P1 = lo
	} else {
		out.Kind = ContactSegment
		out.P0 = lo
		out.P1 = hi
	}
	edgeA, edgeB := inPlaneEdge(sa), inPlaneEdge(sb)
	out.P0OnA, out.P1OnA = edgeA >= 0 || loCmp <= 0, edgeA >= 0 || hiCmp >= 0
	out.P0OnB, out.P1OnB = edgeB >= 0 || loCmp >= 0, edgeB >= 0 || hiCmp <= 0
	if out.Kind == ContactPoint {
		out.P0OnA = out.P0OnA || out.P1OnA
		out.P0OnB = out.P0OnB || out.P1OnB
		out.P1OnA, out.P1OnB = out.P0OnA, out.P0OnB
	} else {
		out.EdgeA, out.EdgeB = edgeA, edgeB
	}
	out.Sin2, err = replaySinSquared(ctx, na, nb, budget)
	if err != nil {
		return TriContact{}, err
	}
	return out, nil
}

func replayRimBound(ctx context.Context, a, b float64, sin2 *SinSquare, budget *proofbound.ReplayBudget) (float64, error) {
	d := proofbound.AbsSumUpper(a, b)
	if d <= 0 {
		return 0, nil
	}
	if sin2 == nil {
		return math.Inf(1), nil
	}
	f, err := proof.ReplayFractionFloat64(ctx, sin2.Num, sin2.Den, budget)
	if err != nil {
		return 0, err
	}
	if f <= 0 {
		return proofbound.DivUpper(d, 0), nil
	}
	s := math.Sqrt(f)
	s = math.Nextafter(s, 0)
	s = math.Nextafter(s, 0)
	return proofbound.DivUpper(d, s), nil
}
