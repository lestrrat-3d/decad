package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DerivativeBound is B_ij of docs/linkage-check-design.md §5.8 for
// the joints at positions m ≤ n of the path, m the shallower: a proven upper
// bound on |∂²x_c/∂q_i∂q_j| for every corner c of the link at every
// configuration the drive reaches. The shallower joint turns the deeper
// one's velocity, whose length is at most w_j — ρ_jk for a revolute joint j,
// 1 for a prismatic one — so B_ij = w_j when the shallower joint is a
// revolute; a prismatic shallower joint turns nothing, and B_ij is 0, nil
// here.
func DerivativeBound(rho []*big.Rat, m, n int) *big.Rat {
	if rho[m] == nil {
		return nil
	}
	if w := rho[n]; w != nil {
		return w
	}
	return big.NewRat(1, 1)
}

// Remainder is Rem(h) = ½·Σ_{i,j} B_ij·h_i·h_j of
// docs/linkage-check-design.md §5.8 over the relative path. h holds each
// joint's travel bound in path order. Taylor's theorem along the straight
// segment in joint space bounds how
// far a corner's position departs from its first-order expansion.
func Remainder(rho, h []*big.Rat) *big.Rat {
	sum := new(big.Rat)
	half := big.NewRat(1, 2)
	for m := range h {
		for n := m; n < len(h); n++ {
			w := DerivativeBound(rho, m, n)
			if w == nil {
				continue
			}
			term := new(big.Rat).Mul(h[m], h[n])
			term.Mul(term, w)
			if n == m {
				term.Mul(term, half)
			}
			sum.Add(sum, term)
		}
	}
	return sum
}

// AddHullShares is addProjectionShares along any direction n with |n| at
// most norm: at the point attaining the body's extent along sense·n, each
// joint i on its relative path takes |n·v_{i,c}|·h_i/norm + Σ_j B_ij·h_i·h_j,
// charged to axes[i]. The shares decide cost alone, never soundness.
func AddHullShares(shares map[int]*big.Rat, s Side, rho []*big.Rat, axes []int, n motionbound.RatVec, norm *big.Rat, sense int) {
	if len(s.H) == 0 {
		return
	}
	dot := func(p [3]proofbound.RatInterval) proofbound.RatInterval {
		sum := proofbound.PointInterval(new(big.Rat))
		for d := range 3 {
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(p[d], n[d]))
		}
		return sum
	}
	attained, top := -1, (*big.Rat)(nil)
	for c := range s.Corners.Hi {
		pos := dot([3]proofbound.RatInterval{
			proofbound.IntervalOwned(s.Corners.Lo[c][0], s.Corners.Hi[c][0]),
			proofbound.IntervalOwned(s.Corners.Lo[c][1], s.Corners.Hi[c][1]),
			proofbound.IntervalOwned(s.Corners.Lo[c][2], s.Corners.Hi[c][2]),
		})
		v := new(big.Rat).Set(pos.Hi)
		if sense < 0 {
			v.Neg(pos.Lo)
		}
		for j, vel := range s.Corners.Vel[c] {
			term := ivAbsUpper(dot(vel))
			v.Add(v, term.Mul(term, s.H[j]))
		}
		if top == nil || v.Cmp(top) > 0 {
			attained, top = c, v
		}
	}
	for j := range s.H {
		share := ivAbsUpper(dot(s.Corners.Vel[attained][j]))
		share.Mul(share, s.H[j])
		share.Quo(share, norm)
		for m := range s.H {
			lo, hi := min(j, m), max(j, m)
			if w := DerivativeBound(rho, lo, hi); w != nil {
				term := new(big.Rat).Mul(w, s.H[j])
				share.Add(share, term.Mul(term, s.H[m]))
			}
		}
		joint := axes[j]
		if cur, ok := shares[joint]; ok {
			cur.Add(cur, share)
			continue
		}
		shares[joint] = share
	}
}

// AttainedDirection is the coordinate axis and sense, ±1, of the direction
// n = sense·e_axis along which projectionLower attained bound: the first, in
// projectionLower's own order, whose separation equals it.
func AttainedDirection(a, p Side, bound *big.Rat) (int, int) {
	aUp, aDown := a.Extents()
	pUp, pDown := p.Extents()
	for d := range 3 {
		if new(big.Rat).Neg(new(big.Rat).Add(pDown[d], aUp[d])).Cmp(bound) == 0 {
			return d, 1
		}
		if new(big.Rat).Neg(new(big.Rat).Add(pUp[d], aDown[d])).Cmp(bound) == 0 {
			return d, -1
		}
	}
	return 0, 1
}

// AddAxisShares adds one body's shares of a projection bound's defect
// along sense·e_axis (docs/linkage-check-design.md §5.8, the split axis): at
// the corner attaining the body's extent along that direction, each joint i
// on its relative path takes |n·v_{i,c}|·h_i + Σ_j B_ij·h_i·h_j, the part of
// the first-order term and the remainder that halving h_i removes, charged
// to axes[i]: the joint itself, or a loop dependent's driver. A body with
// no joint, a static partner, takes nothing.
func AddAxisShares(shares map[int]*big.Rat, s Side, rho []*big.Rat, axes []int, axis, sense int) {
	if len(s.H) == 0 {
		return
	}
	attained, top := -1, (*big.Rat)(nil)
	for c := range s.Corners.Hi {
		up, down := s.FirstOrder(c, axis)
		v := up.Add(up, s.Corners.Hi[c][axis])
		if sense < 0 {
			v = down.Sub(down, s.Corners.Lo[c][axis])
		}
		if top == nil || v.Cmp(top) > 0 {
			attained, top = c, v
		}
	}
	for n := range s.H {
		share := ivAbsUpper(s.Corners.Vel[attained][n][axis])
		share.Mul(share, s.H[n])
		for m := range s.H {
			lo, hi := min(n, m), max(n, m)
			if w := DerivativeBound(rho, lo, hi); w != nil {
				term := new(big.Rat).Mul(w, s.H[n])
				share.Add(share, term.Mul(term, s.H[m]))
			}
		}
		joint := axes[n]
		if cur, ok := shares[joint]; ok {
			cur.Add(cur, share)
			continue
		}
		shares[joint] = share
	}
}
