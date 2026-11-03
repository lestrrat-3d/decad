package verifyrun

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// This file is the end-cap separator (agent-docs/interference-design.md §3.2,
// "The end-cap separator"). A prism with exact levels lies between the planes
// of its two end caps. When another body lies wholly on the far side of one of
// those planes, the plane separates the interiors: a set screw seated on a
// shaft's surface is decided by the plane of its seat. The far side is proven
// exactly, through the other body's support along the cap normal, with no
// unit normal and each square root compared squared. For a Revolve only the
// part within the prism's slab across the axis enters.

// supportTerm is one candidate for a support value: lin + rho·√q.
type supportTerm struct {
	lin, rho *big.Rat
}

// support bounds a body's reach along a direction: no point of the body
// reaches past the largest term, all sharing one q.
type support struct {
	q     *big.Rat
	terms []supportTerm
}

// within reports whether every term is at most k, exactly.
func (s support) within(k *big.Rat) bool {
	for _, term := range s.terms {
		d := new(big.Rat).Sub(k, term.lin)
		if d.Sign() < 0 {
			return false
		}
		if term.rho.Sign() == 0 || s.q.Sign() == 0 {
			continue
		}
		reach := new(big.Rat).Mul(new(big.Rat).Mul(term.rho, term.rho), s.q)
		if new(big.Rat).Mul(d, d).Cmp(reach) < 0 {
			return false
		}
	}
	return true
}

// upper is a rational at least every term's value, rounding each square root
// up; it fails when no float square root bounds q.
func (s support) upper() (*big.Rat, bool) {
	root, ok := sqrtUpper(s.q)
	if !ok {
		return nil, false
	}
	var top *big.Rat
	for _, term := range s.terms {
		v := new(big.Rat).Add(term.lin, new(big.Rat).Mul(term.rho, root))
		if top == nil || v.Cmp(top) > 0 {
			top = v
		}
	}
	return top, top != nil
}

// sqrtUpper is a float at least √q, checked exactly.
func sqrtUpper(q *big.Rat) (*big.Rat, bool) {
	if q.Sign() == 0 {
		return new(big.Rat), true
	}
	f, _ := q.Float64()
	u := math.Sqrt(f)
	for range 64 {
		if math.IsInf(u, 0) || math.IsNaN(u) {
			return nil, false
		}
		r := ratOf(u)
		if new(big.Rat).Mul(r, r).Cmp(q) >= 0 {
			return r, true
		}
		u = math.Nextafter(u, math.Inf(1))
	}
	return nil, false
}

// capApart reports whether the plane of one of a's end caps holds b, within
// the slab a occupies across b's axis, wholly on the side away from a.
func capApart(a, b *topo.Body) bool {
	pa, ok := a.Payload.(prismrecord.Payload)
	if !ok || pa.SurfaceResult || pa.Z0Delta != 0 || pa.Z1Delta != 0 {
		return false
	}
	l, t, _ := frameLift(pa.Frame, pa.Xform)
	m := crossVec(l[0], l[1])
	switch dotVec(m, l[2]).Sign() {
	case 0:
		return false
	case -1:
		m = scaleVec(m, big.NewRat(-1, 1))
	}
	lo, hi := ratOf(min(pa.Z0, pa.Z1)), ratOf(max(pa.Z0, pa.Z1))
	// a's points t + L·(x, y, z) reach m·t + z·(m·L₃) along m, and m·L₃ > 0,
	// so a lies where m·x ≥ m·p(lo) and where m·x ≤ m·p(hi).
	at := func(z *big.Rat) *big.Rat { return dotVec(m, addVec(t, scaleVec(l[2], z))) }
	if s, ok := supportOf(b, m, &pa); ok && s.within(at(lo)) {
		return true
	}
	neg := scaleVec(m, big.NewRat(-1, 1))
	if s, ok := supportOf(b, neg, &pa); ok && s.within(new(big.Rat).Neg(at(hi))) {
		return true
	}
	return false
}

// supportOf bounds how far along m a body reaches: a prism with an exact
// section and levels over a whole circle or whole lines, or a Revolve the
// meridian separator reads. Given near, a Revolve's reach is that of its part
// in near's slab across its axis; nil reads the whole body.
func supportOf(body *topo.Body, m [3]*big.Rat, near *prismrecord.Payload) (support, bool) {
	switch p := body.Payload.(type) {
	case prismrecord.Payload:
		return prismSupport(p, m)
	case revolveplan.Payload:
		r, ok := revolveMeridian(p)
		if !ok {
			return support{}, false
		}
		return revolveSupport(r, m, near), true
	}
	return support{}, false
}

// prismSupport reads a prism's points t + L·(x, y, z) along m as
// m·t + μ·(x, y, z), with μ = Lᵀm. Over a circle of centre c and radius r in
// (x, y) that peaks at μ·(c, z) + r·√(μ₁² + μ₂²); over whole lines at a
// vertex; over the levels at an end.
func prismSupport(p prismrecord.Payload, m [3]*big.Rat) (support, bool) {
	if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 {
		return support{}, false
	}
	l, t, _ := frameLift(p.Frame, p.Xform)
	mu := [3]*big.Rat{dotVec(l[0], m), dotVec(l[1], m), dotVec(l[2], m)}
	base := dotVec(m, t)
	levels := []*big.Rat{ratOf(p.Z0), ratOf(p.Z1)}
	reach := func(x, y, z *big.Rat) *big.Rat {
		return new(big.Rat).Add(base, dotVec(mu, [3]*big.Rat{x, y, z}))
	}
	if c, ok := wholeCircle(p.Profile.Outer); ok {
		r, err := c.Radius.In(units.Millimeter)
		if err != nil || !(r > 0) {
			return support{}, false
		}
		q := new(big.Rat).Add(new(big.Rat).Mul(mu[0], mu[0]), new(big.Rat).Mul(mu[1], mu[1]))
		s := support{q: q}
		for _, z := range levels {
			s.terms = append(s.terms, supportTerm{lin: reach(ratOf(c.Center.U), ratOf(c.Center.V), z), rho: ratOf(r)})
		}
		return s, true
	}
	s := support{q: new(big.Rat)}
	zero := new(big.Rat)
	for _, seg := range p.Profile.Outer.Segments {
		start, end, ok := wholeLine(seg)
		if !ok {
			return support{}, false
		}
		for _, z := range levels {
			for _, p := range []sectionrecord.Point2{start, end} {
				s.terms = append(s.terms, supportTerm{lin: reach(ratOf(p.U), ratOf(p.V), z), rho: zero})
			}
		}
	}
	return s, len(s.terms) > 0
}

// revolveSupport reads a revolve's points t + L·(anchor + z·dir + ρ·e), e a
// unit vector across the axis, along m. With μ = Lᵀm split along the axis,
// that peaks at m·t + μ·anchor + z·(μ·dir) + ρ·|μ across the axis| over the
// vertices of the meridian cut to near's slab; the frame coordinates measure
// true distance, so the split is exact.
func revolveSupport(r meridian, m [3]*big.Rat, near *prismrecord.Payload) support {
	mu := [3]*big.Rat{dotVec(r.l[0], m), dotVec(r.l[1], m), dotVec(r.l[2], m)}
	along := dotVec(mu, r.dir)
	q := new(big.Rat).Sub(dotVec(mu, mu), new(big.Rat).Mul(along, along))
	base := new(big.Rat).Add(dotVec(m, r.t), dotVec(mu, r.anchor))
	s := support{q: q}
	region := r.region
	if near != nil {
		region = slabCut(r, *near)
	}
	for _, v := range region {
		s.terms = append(s.terms, supportTerm{lin: new(big.Rat).Add(base, new(big.Rat).Mul(along, v[1])), rho: v[0]})
	}
	return s
}

// slabCut lists points of r's region whose hull holds the part of the region
// at axial levels near can reach: the vertices within the slab and the edges'
// crossings of its two levels. It returns every vertex when near's reach along
// the axis is not bounded.
func slabCut(r meridian, near prismrecord.Payload) [][2]*big.Rat {
	cols := [3][3]*big.Rat{}
	for i := range 3 {
		cols[i] = [3]*big.Rat{r.l[0][i], r.l[1][i], r.l[2][i]}
	}
	// A world point x sits at axial level g·x − g·t − dir·anchor, with Lᵀg = dir.
	g, ok := solve(cols, r.dir)
	if !ok {
		return r.region
	}
	off := new(big.Rat).Add(dotVec(g, r.t), dotVec(r.dir, r.anchor))
	hiSup, okHi := prismSupport(near, g)
	loSup, okLo := prismSupport(near, scaleVec(g, big.NewRat(-1, 1)))
	if !okHi || !okLo {
		return r.region
	}
	hi, okHi := hiSup.upper()
	lo, okLo := loSup.upper()
	if !okHi || !okLo {
		return r.region
	}
	hi.Sub(hi, off)
	lo.Neg(lo).Sub(lo, off)
	var out [][2]*big.Rat
	for i, p := range r.region {
		if p[1].Cmp(lo) >= 0 && p[1].Cmp(hi) <= 0 {
			out = append(out, p)
		}
		q := r.region[(i+1)%len(r.region)]
		for _, c := range []*big.Rat{lo, hi} {
			dp, dq := new(big.Rat).Sub(p[1], c), new(big.Rat).Sub(q[1], c)
			if dp.Sign()*dq.Sign() >= 0 {
				continue
			}
			f := new(big.Rat).Quo(dp, new(big.Rat).Sub(p[1], q[1]))
			rho := new(big.Rat).Add(p[0], new(big.Rat).Mul(f, new(big.Rat).Sub(q[0], p[0])))
			out = append(out, [2]*big.Rat{rho, c})
		}
	}
	return out
}
