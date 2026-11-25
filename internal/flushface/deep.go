package flushface

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Deep reports whether point p of the outline's plane lies inside the region the outline bounds and farther than
// margin from every edge, both read in the plane that drops o.Axis (agent-docs/coplanar-contact-design.md §19.4). The
// projection never lengthens a distance, so a projected distance above margin is an in-plane distance above it too.
// Every acceptance is exact: the squared distance of p from each edge, and the side of p for each edge the ray toward
// +u crosses, are exact rationals over the held floats. A float distance below margin rejects at once, since rejecting
// is always sound. It steps budget once per edge.
func Deep(budget *proofbound.WorkBudget, o Outline, p r3.Vec, margin float64) (bool, error) {
	if len(o.Edges) == 0 || !finiteBound(margin) || !proofbound.FiniteVec(p) {
		return false, nil
	}
	m2 := new(big.Rat).SetFloat64(margin)
	m2.Mul(m2, m2)
	q := rat2(p, o.Axis)
	pu, pv := proj(p, o.Axis)
	crossings := 0
	for _, e := range o.Edges {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !proofbound.FiniteVec(e.A) || !proofbound.FiniteVec(e.B) {
			return false, nil
		}
		au, av := proj(e.A, o.Axis)
		bu, bv := proj(e.B, o.Axis)
		if segDistFloat(pu, pv, au, av, bu, bv) < margin*(1-1e-6) {
			return false, nil
		}
		a, b := rat2(e.A, o.Axis), rat2(e.B, o.Axis)
		if segDist2(q, a, b).Cmp(m2) <= 0 {
			return false, nil
		}
		// Half-open rule: an edge counts when exactly one end lies above p, and then it crosses the ray toward +u
		// when p lies on the side of the edge's line that its direction along v names. p is off the edge, so the
		// orientation is not zero.
		aAbove, bAbove := a[1].Cmp(q[1]) > 0, b[1].Cmp(q[1]) > 0
		if aAbove == bAbove {
			continue
		}
		up := b[1].Cmp(a[1])
		if up*orient(a, b, q).Sign() > 0 {
			crossings++
		}
	}
	return crossings%2 == 1, nil
}

// proj returns p's two coordinates in the plane that drops axis, in rat2's order.
func proj(p r3.Vec, axis int) (float64, float64) {
	c := [3]float64{p.X, p.Y, p.Z}
	switch axis {
	case 0:
		return c[1], c[2]
	case 1:
		return c[0], c[2]
	}
	return c[0], c[1]
}

// segDistFloat is the float distance from (pu, pv) to the segment from (au, av) to (bu, bv); it only screens.
func segDistFloat(pu, pv, au, av, bu, bv float64) float64 {
	du, dv := bu-au, bv-av
	wu, wv := pu-au, pv-av
	l2 := du*du + dv*dv
	t := 0.0
	if l2 > 0 {
		t = math.Max(0, math.Min(1, (wu*du+wv*dv)/l2))
	}
	return math.Hypot(wu-t*du, wv-t*dv)
}

// segDist2 is the exact squared distance from q to the segment from a to b.
func segDist2(q, a, b rv) *big.Rat {
	d, w := sub(b, a), sub(q, a)
	dot := func(x, y rv) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(x[0], y[0]), new(big.Rat).Mul(x[1], y[1]))
	}
	l2 := dot(d, d)
	t := dot(w, d)
	if t.Sign() <= 0 || l2.Sign() == 0 {
		return dot(w, w)
	}
	if t.Cmp(l2) >= 0 {
		qb := sub(q, b)
		return dot(qb, qb)
	}
	c := cross(d, w)
	c.Mul(c, c)
	return c.Quo(c, l2)
}
