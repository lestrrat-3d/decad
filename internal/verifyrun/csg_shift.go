package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the exact-translation reading of the history walk
// (agent-docs/interference-design.md §3.2). A direct copy made by an exact
// lift translation denotes its source's true solid moved by an exact rational
// vector. The walk reads such a copy as its source plus that shift and carries
// the shift down the source's Boolean history. Only the rules that add the
// shift exactly may read two bodies with different shifts: the box test, the
// prism lift, and the end-cap and slab separators, which read the other body
// moved by the relative shift (planesApart). Translating both bodies by one vector keeps every relation, so
// every other rule reads the raw nodes only when both shifts are equal.

// exactShiftSource is a history that also names a direct exact-translation
// copy's source and the exact world vector the copy denotes.
type exactShiftSource interface {
	ExactShiftProducer(n *topo.Body) (*topo.Body, [3]*big.Rat, bool)
}

// shift is an exact world translation; the zero value is no shift.
type shift struct{ v *[3]*big.Rat }

func (s shift) at(i int) *big.Rat {
	if s.v == nil {
		return new(big.Rat)
	}
	return s.v[i]
}

// add returns the shift s followed by t.
func (s shift) add(t [3]*big.Rat) shift {
	var sum [3]*big.Rat
	for i := range sum {
		sum[i] = new(big.Rat).Add(s.at(i), t[i])
	}
	return shift{v: &sum}
}

// equal reports whether s and o are the same vector.
func (s shift) equal(o shift) bool {
	for i := range 3 {
		if s.at(i).Cmp(o.at(i)) != 0 {
			return false
		}
	}
	return true
}

// vec returns s as a rational vector.
func (s shift) vec() [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Set(s.at(0)), new(big.Rat).Set(s.at(1)), new(big.Rat).Set(s.at(2))}
}

// walkBody is a recorded body read as moved by an exact shift.
type walkBody struct {
	n *topo.Body
	s shift
}

// wrap reads each of nodes under the shift s.
func wrap(nodes []*topo.Body, s shift) []walkBody {
	out := make([]walkBody, len(nodes))
	for i, n := range nodes {
		out[i] = walkBody{n: n, s: s}
	}
	return out
}

// resolve follows b through the producers that denote the same true solid
// (a Refine, an identity Transform) and through direct exact-translation
// copies, adding each copy's vector to the shift. It takes a bounded number of
// steps.
func (w historyWalk) resolve(b walkBody) walkBody {
	same, _ := w.history.(sameSetSource)
	moved, _ := w.history.(exactShiftSource)
	for range historyMaxDepth {
		if same != nil {
			if src, ok := same.SameSetProducer(b.n); ok && src != nil {
				b.n = src
				continue
			}
		}
		if moved != nil {
			if src, t, ok := moved.ExactShiftProducer(b.n); ok && src != nil {
				b = walkBody{n: src, s: b.s.add(t)}
				continue
			}
		}
		break
	}
	return b
}

// boxesDisjoint reports whether the bounds-inflated boxes of a and b, each
// moved by its shift, have disjoint interiors. With equal shifts it is
// measurement.BoxesDisjoint on the raw bounds.
func boxesDisjoint(a, b walkBody) bool {
	if a.s.equal(b.s) {
		return measurement.BoxesDisjoint(a.n.Bounds, b.n.Bounds)
	}
	return boxesDisjointShifted(a.n.Bounds, a.s, b.n.Bounds, b.s)
}

// boxesDisjointShifted is measurement.BoxesDisjoint for boxes moved by exact
// shifts: a body's interior lies in the interior of its box inflated by the
// box's own bound, so on some axis one box's upper end meeting or passing the
// other's lower end proves the interiors disjoint. Every end is computed
// exactly in rationals.
func boxesDisjointShifted(a measurement.Box, sa shift, b measurement.Box, sb shift) bool {
	ia, ib := ratOf(a.Bound.Base()), ratOf(b.Bound.Base())
	amax, amin := [3]float64{a.Max.X, a.Max.Y, a.Max.Z}, [3]float64{a.Min.X, a.Min.Y, a.Min.Z}
	bmax, bmin := [3]float64{b.Max.X, b.Max.Y, b.Max.Z}, [3]float64{b.Min.X, b.Min.Y, b.Min.Z}
	upper := func(top float64, bound *big.Rat, s shift, axis int) *big.Rat {
		u := new(big.Rat).Add(ratOf(top), bound)
		return u.Add(u, s.at(axis))
	}
	lower := func(bottom float64, bound *big.Rat, s shift, axis int) *big.Rat {
		l := new(big.Rat).Sub(ratOf(bottom), bound)
		return l.Add(l, s.at(axis))
	}
	for axis := range 3 {
		if upper(amax[axis], ia, sa, axis).Cmp(lower(bmin[axis], ib, sb, axis)) <= 0 ||
			upper(bmax[axis], ib, sb, axis).Cmp(lower(amin[axis], ia, sa, axis)) <= 0 {
			return true
		}
	}
	return false
}

// liftOffsetShifted is liftOffset for prisms a and b read under shifts sa and
// sb: each shift is added to its prism's exact lift translation before the
// offset is solved.
func liftOffsetShifted(a prismrecord.Payload, sa shift, b prismrecord.Payload, sb shift) (prismOffset, bool) {
	return liftOffsetBy(a, sa.vec(), b, sb.vec())
}

// prismWithinShifted is prismWithin for prisms read under shifts. Unlike
// prismWithin it has no box fallback: the box rule reads unshifted records.
func prismWithinShifted(ctx context.Context, budget *proofbound.WorkBudget,
	inner prismrecord.Payload, si shift, outer prismrecord.Payload, so shift) (bool, error) {
	off, ok := liftOffsetShifted(outer, so, inner, si)
	if !ok {
		return false, nil
	}
	ilo, ihi := levels(inner, off.dz)
	olo, ohi := levels(outer, nil)
	if ilo.Cmp(olo) < 0 || ihi.Cmp(ohi) > 0 {
		return false, nil
	}
	moved, ok := off.profile(inner.Profile)
	if !ok {
		return false, nil
	}
	return prismcells.SectionWithin(ctx, budget, []momentinput.Profile{moved}, []momentinput.Profile{outer.Profile})
}

// zeroVec is the zero world vector, the shift of a body that is not moved.
func zeroVec() [3]*big.Rat { return [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)} }

// planesApart is the end-cap and slab separators for a and b, with b read as
// moved by the exact world vector d relative to a.
func planesApart(a, b *topo.Body, d [3]*big.Rat) bool {
	back := scaleVec(d, big.NewRat(-1, 1))
	return capApartBy(a, b, d) || capApartBy(b, a, back) || sideApartBy(a, b, d) || sideApartBy(b, a, back)
}
