package meshbool

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

var replayPolygonZero = big.NewRat(0, 1)

// replayPointInPoly2 uses the ordinary half-open rule with metered predicates.
func replayPointInPoly2(ctx context.Context, budget *proofbound.WorkBudget, poly []Xp2, p Xp2) (bool, bool, error) {
	for i := range poly {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		on, err := replayOnSegment2(ctx, budget, poly[i], poly[(i+1)%len(poly)], p)
		if err != nil {
			return false, false, err
		}
		if on {
			return false, true, nil
		}
	}
	inside := false
	for i := range poly {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		a, b := poly[i], poly[(i+1)%len(poly)]
		ca, err := replayCompare2(ctx, budget, a, p, 1)
		if err != nil {
			return false, false, err
		}
		cb, err := replayCompare2(ctx, budget, b, p, 1)
		if err != nil {
			return false, false, err
		}
		belowA := ca <= 0
		if belowA == (cb <= 0) {
			continue
		}
		side, err := replayCross2(ctx, budget, a, b, p)
		if err != nil {
			return false, false, err
		}
		if (belowA && side > 0) || (!belowA && side < 0) {
			inside = !inside
		}
	}
	return inside, false, nil
}

// replayPolyArea2Sign retains the ordinary floating filter and meters exact work.
func replayPolyArea2Sign(ctx context.Context, budget *proofbound.WorkBudget, poly []Xp2) (int, error) {
	if _, err := budget.StepN(uint64(len(poly))); err != nil {
		return 0, err
	}
	if sign := polyArea2FloatSign(poly); sign != 0 {
		return sign, nil
	}
	meter := replayExactWork{budget}
	sum := replayPolygonZero
	for i, a := range poly {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		b := poly[(i+1)%len(poly)]
		if a.deferredRats || b.deferredRats {
			return 0, errBrakeDeferredVertex
		}
		left, err := proof.ReplayRatArithmetic(ctx, a.U, b.V, '*', meter)
		if err != nil {
			return 0, err
		}
		right, err := proof.ReplayRatArithmetic(ctx, b.U, a.V, '*', meter)
		if err != nil {
			return 0, err
		}
		term, err := proof.ReplayRatArithmetic(ctx, left, right, '-', meter)
		if err != nil {
			return 0, err
		}
		sum, err = proof.ReplayRatArithmetic(ctx, sum, term, '+', meter)
		if err != nil {
			return 0, err
		}
	}
	return sum.Sign(), nil
}

// replayEarClip mirrors EarClipX while bounding the buffers and exact predicates.
func replayEarClip(ctx context.Context, budget *proofbound.WorkBudget, pts []Xp2, poly []int) ([][3]int, error) {
	if len(poly) < 3 {
		return nil, errBrakeInvalidEarPolygon
	}
	if err := budget.Reserve(2 * uint64(len(poly))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(uint64(len(poly)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(uint64(len(poly)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	if _, err := budget.StepN(2 * uint64(len(poly))); err != nil {
		return nil, err
	}
	idx := make([]int, len(poly))
	copy(idx, poly)
	tris := make([][3]int, 0, len(poly))
	for len(idx) > 3 {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		n, clipped := len(idx), false
		for i := range n {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			sign, err := replayCross2(ctx, budget, pts[ia], pts[ib], pts[ic])
			if err != nil {
				return nil, err
			}
			if sign <= 0 {
				continue
			}
			blocked, err := replayEarBlocked(ctx, budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			for j := i; j+1 < len(idx); j++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				idx[j] = idx[j+1]
			}
			idx = idx[:len(idx)-1]
			clipped = true
			break
		}
		if !clipped {
			return nil, errBrakeEarStalled
		}
	}
	sign, err := replayCross2(ctx, budget, pts[idx[0]], pts[idx[1]], pts[idx[2]])
	if err != nil {
		return nil, err
	}
	if sign > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	} else if sign < 0 {
		return nil, errBrakeEarClockwise
	}
	return tris, nil
}

func replayEarBlocked(ctx context.Context, budget *proofbound.WorkBudget, pts []Xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	minU, maxU := math.Min(a.Fu, math.Min(b.Fu, c.Fu)), math.Max(a.Fu, math.Max(b.Fu, c.Fu))
	minV, maxV := math.Min(a.Fv, math.Min(b.Fv, c.Fv)), math.Max(a.Fv, math.Max(b.Fv, c.Fv))
	finiteBox, finiteA, finiteC := a.FloatFinite && b.FloatFinite && c.FloatFinite, finiteRoundedCache(a), finiteRoundedCache(c)
	for j := range n {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		if finiteBox && p.FloatFinite && (p.Fu < minU || p.Fu > maxU || p.Fv < minV || p.Fv > maxV) {
			continue
		}
		finiteP := finiteRoundedCache(p)
		if !finiteP || !finiteA || (p.Fu == a.Fu && p.Fv == a.Fv) {
			x, err := replayCompare2(ctx, budget, p, a, 0)
			if err != nil {
				return false, err
			}
			if x == 0 {
				y, err := replayCompare2(ctx, budget, p, a, 1)
				if err != nil {
					return false, err
				}
				if y == 0 {
					continue
				}
			}
		}
		if !finiteP || !finiteC || (p.Fu == c.Fu && p.Fv == c.Fv) {
			x, err := replayCompare2(ctx, budget, p, c, 0)
			if err != nil {
				return false, err
			}
			if x == 0 {
				y, err := replayCompare2(ctx, budget, p, c, 1)
				if err != nil {
					return false, err
				}
				if y == 0 {
					continue
				}
			}
		}
		d1, err := replayCross2(ctx, budget, a, b, p)
		if err != nil {
			return false, err
		}
		d2, err := replayCross2(ctx, budget, b, c, p)
		if err != nil {
			return false, err
		}
		d3, err := replayCross2(ctx, budget, c, a, p)
		if err != nil {
			return false, err
		}
		if (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0) {
			return true, nil
		}
	}
	return false, nil
}
