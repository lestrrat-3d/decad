package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type slabReplayOps struct {
	ctx context.Context //nolint:containedctx // The value lives only for one bounded replay call.
	b   *proofbound.ReplayBudget
}

func (m slabReplayOps) op(a, b proof.Dyadic, k byte) (proof.Dyadic, error) {
	return proof.ReplayDyadicArithmetic(m.ctx, a, b, k, m.b)
}
func (m slabReplayOps) sub(a, b proof.DyV3) (proof.DyV3, error) {
	var r proof.DyV3
	for i := range 3 {
		v, e := m.op(a[i], b[i], '-')
		if e != nil {
			return r, e
		}
		r[i] = v
	}
	return r, nil
}
func (m slabReplayOps) cross(a, b proof.DyV3) (proof.DyV3, error) {
	var r proof.DyV3
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		x, e := m.op(a[j], b[k], '*')
		if e != nil {
			return r, e
		}
		y, e := m.op(a[k], b[j], '*')
		if e != nil {
			return r, e
		}
		r[i], e = m.op(x, y, '-')
		if e != nil {
			return r, e
		}
	}
	return r, nil
}
func (m slabReplayOps) dot(a, b proof.DyV3) (proof.Dyadic, error) {
	var s proof.Dyadic
	for i := range 3 {
		x, e := m.op(a[i], b[i], '*')
		if e != nil {
			return s, e
		}
		s, e = m.op(s, x, '+')
		if e != nil {
			return s, e
		}
	}
	return s, nil
}
func (m slabReplayOps) rat(d proof.Dyadic) (*big.Rat, error) {
	return proof.ReplayDyadicRat(m.ctx, d, m.b)
}

// sqrt returns false when the fixed-cost seed cannot establish the ordinary seed.
func (m slabReplayOps) sqrt(q *big.Rat) (float64, bool, error) {
	if q.Sign() <= 0 {
		return 0, true, nil
	}
	if err := m.b.SpendWork(m.ctx, 4); err != nil {
		return 0, false, err
	}
	f, ok := proof.RatSqrtSeed(q)
	if !ok {
		return 0, false, nil
	}
	if math.IsInf(f, 0) || math.IsNaN(f) {
		f = math.MaxFloat64
	}
	for range proofbound.SqrtAdjustLimit {
		if math.IsInf(f, 1) {
			return f, true, nil
		}
		v, e := proof.ReplayFloatRat(m.ctx, f, m.b)
		if e != nil {
			return 0, false, e
		}
		sq, e := proof.ReplayRatArithmetic(m.ctx, v, v, '*', m.b)
		if e != nil {
			return 0, false, e
		}
		c, e := proof.ReplayRatCompare(m.ctx, sq, q, m.b)
		if e != nil {
			return 0, false, e
		}
		if c >= 0 {
			return f, true, nil
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1), true, nil
}

// SetChargedSlabBoundsReplay tightens only when the bounded proof completes.
func (bm *BoolMesh) SetChargedSlabBoundsReplay(ctx context.Context, planeOf func(int) (r3.Frame, float64, bool), budget *proofbound.ReplayBudget) error {
	for i, tri := range bm.Tris {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		frame, delta, ok := planeOf(i)
		if !ok || !(delta >= 0) || math.IsInf(delta, 0) {
			continue
		}
		m := slabReplayOps{ctx, budget}
		o := proof.DyVec(frame.Origin())
		n, e := m.cross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
		if e != nil {
			return e
		}
		nn, e := m.dot(n, n)
		if e != nil {
			return e
		}
		if nn.IsZero() {
			continue
		}
		c := [3]proof.DyV3{proof.DyVec(bm.Verts[tri[0]]), proof.DyVec(bm.Verts[tri[1]]), proof.DyVec(bm.Verts[tri[2]])}
		far, e := proof.ReplayFloatRat(ctx, 0, budget)
		if e != nil {
			return e
		}
		for _, p := range c {
			d, e := m.sub(p, o)
			if e != nil {
				return e
			}
			v, e := m.dot(n, d)
			if e != nil {
				return e
			}
			v, e = m.op(v, v, '*')
			if e != nil {
				return e
			}
			q, e := m.rat(v)
			if e != nil {
				return e
			}
			comp, e := proof.ReplayRatCompare(ctx, q, far, budget)
			if e != nil {
				return e
			}
			if comp > 0 {
				far = q
			}
		}
		nr, e := m.rat(nn)
		if e != nil {
			return e
		}
		q, e := proof.ReplayRatArithmetic(ctx, far, nr, '/', budget)
		if e != nil {
			return e
		}
		h, ok, e := m.sqrt(q)
		if e != nil {
			return e
		}
		if !ok {
			continue
		}
		u, e := m.sub(c[1], c[0])
		if e != nil {
			return e
		}
		v, e := m.sub(c[2], c[0])
		if e != nil {
			return e
		}
		nt, e := m.cross(u, v)
		if e != nil {
			return e
		}
		ntnt, e := m.dot(nt, nt)
		if e != nil {
			return e
		}
		if ntnt.IsZero() {
			continue
		}
		x, e := m.cross(nt, n)
		if e != nil {
			return e
		}
		xs, e := m.dot(x, x)
		if e != nil {
			return e
		}
		den, e := m.op(ntnt, nn, '*')
		if e != nil {
			return e
		}
		xr, e := m.rat(xs)
		if e != nil {
			return e
		}
		dr, e := m.rat(den)
		if e != nil {
			return e
		}
		q, e = proof.ReplayRatArithmetic(ctx, xr, dr, '/', budget)
		if e != nil {
			return e
		}
		slope, ok, e := m.sqrt(q)
		if e != nil {
			return e
		}
		if !ok {
			continue
		}
		slab := proofbound.AbsSumUpper(h, proofbound.ProductUpper(bm.FacetBound[i], slope))
		if delta != 0 {
			slab = proofbound.AbsSumUpper(slab, 2*delta)
		}
		if !proofbound.IsNonFinite(slab) && slab < bm.SlabBound[i] {
			bm.SlabBound[i] = slab
		}
	}
	return nil
}
