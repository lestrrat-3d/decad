package verifyrun

import (
	"math/big"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the oriented slab separator (agent-docs/interference-design.md
// §3.2, "The slab separator"). A prism lies, along each axis of its own frame,
// between two planes: its section's box across the section axes and its
// levels along the extrusion. With L the prism's exact lift, the coordinate of
// a world point x along local axis k is g_k·(x − t), where Lᵀg_k = e_k, so the
// prism lies where g_k·x is between g_k·t plus its two bounds. When another
// body's reach along g_k, or along −g_k, stays on the far side of one of those
// planes, the plane separates the interiors. This is the world box test of the
// history walk read in the prism's own coordinates, so a part turned off the
// world axes is still decided by the box its own frame gives it.

// sideApart reports whether one of a's six frame planes holds b wholly on
// the side away from a.
func sideApart(a, b *topo.Body) bool {
	pa, ok := a.Payload.(prismrecord.Payload)
	if !ok || pa.SurfaceResult || pa.Z0Delta != 0 || pa.Z1Delta != 0 || pa.SectionDelta != 0 {
		return false
	}
	lo, hi, ok := loopBox(pa.Profile.Outer)
	if !ok {
		return false
	}
	lo = append(lo, ratOf(min(pa.Z0, pa.Z1)))
	hi = append(hi, ratOf(max(pa.Z0, pa.Z1)))
	l, t, _ := frameLift(pa.Frame, pa.Xform)
	var cols [3][3]*big.Rat
	for i := range 3 {
		cols[i] = [3]*big.Rat{l[0][i], l[1][i], l[2][i]}
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	for k := range 3 {
		e := [3]*big.Rat{zero, zero, zero}
		e[k] = one
		g, ok := solve(cols, e)
		if !ok {
			return false
		}
		base := dotVec(g, t)
		if up, ok := reachUpper(b, g); ok && up.Cmp(new(big.Rat).Add(base, lo[k])) <= 0 {
			return true
		}
		if down, ok := reachUpper(b, scaleVec(g, big.NewRat(-1, 1))); ok &&
			down.Cmp(new(big.Rat).Neg(new(big.Rat).Add(base, hi[k]))) <= 0 {
			return true
		}
	}
	return false
}

// loopBox bounds a section loop in its own coordinates, holding every point of
// the region it encloses: whole or trimmed lines by their full segments'
// ends, circles and arcs by their whole circle.
func loopBox(loop sectionrecord.LoopRecord) ([]*big.Rat, []*big.Rat, bool) {
	var lo, hi []*big.Rat
	add := func(u, v *big.Rat) {
		if lo == nil {
			lo, hi = []*big.Rat{u, v}, []*big.Rat{new(big.Rat).Set(u), new(big.Rat).Set(v)}
			return
		}
		for k, x := range []*big.Rat{u, v} {
			if x.Cmp(lo[k]) < 0 {
				lo[k] = x
			}
			if x.Cmp(hi[k]) > 0 {
				hi[k] = x
			}
		}
	}
	for _, seg := range loop.Segments {
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			add(ratOf(s.Start.U), ratOf(s.Start.V))
			add(ratOf(s.End.U), ratOf(s.End.V))
		case sectionrecord.CircleSeg:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil || !(r > 0) {
				return nil, nil, false
			}
			rr := ratOf(r)
			add(new(big.Rat).Sub(ratOf(s.Center.U), rr), new(big.Rat).Sub(ratOf(s.Center.V), rr))
			add(new(big.Rat).Add(ratOf(s.Center.U), rr), new(big.Rat).Add(ratOf(s.Center.V), rr))
		default:
			return nil, nil, false
		}
	}
	return lo, hi, lo != nil
}

// reachUpper is a rational at least body's reach along m: a prism's or a
// lined Revolve's support (supportOf), each square root rounded up, or a
// disc Revolve's, read by discReachUpper.
func reachUpper(body *topo.Body, m [3]*big.Rat) (*big.Rat, bool) {
	if p, ok := body.Payload.(revolveplan.Payload); ok {
		r, ok := revolveMeridian(p)
		if !ok {
			return nil, false
		}
		if r.disc != nil {
			return discReachUpper(r, m)
		}
		return revolveSupport(r, m, nil).upper()
	}
	s, ok := supportOf(body, m, nil)
	if !ok {
		return nil, false
	}
	return s.upper()
}

// discReachUpper bounds a disc Revolve's reach along m from above. Its points
// are t + L·(anchor + z·dir + ρ·e) with (ρ, z) in the disc of centre (ρc, zc)
// and radius R, ρ ≥ 0, and e a unit vector across dir in frame coordinates,
// which measure true distance. With μ = Lᵀm split into a = μ·dir along the
// axis and |μ⊥| = √q across it, m·x ≤ m·t + μ·anchor + z·a + ρ·√q, whose
// largest value over the disc is at its centre plus R·√(a² + q).
func discReachUpper(r meridian, m [3]*big.Rat) (*big.Rat, bool) {
	mu := [3]*big.Rat{dotVec(r.l[0], m), dotVec(r.l[1], m), dotVec(r.l[2], m)}
	along := dotVec(mu, r.dir)
	norm := dotVec(mu, mu)
	q := new(big.Rat).Sub(norm, new(big.Rat).Mul(along, along))
	across, ok := sqrtUpper(q)
	if !ok {
		return nil, false
	}
	whole, ok := sqrtUpper(norm)
	if !ok {
		return nil, false
	}
	out := new(big.Rat).Add(dotVec(r.t, m), dotVec(mu, r.anchor))
	out.Add(out, new(big.Rat).Mul(r.disc[1], along))
	out.Add(out, new(big.Rat).Mul(r.disc[0], across))
	out.Add(out, new(big.Rat).Mul(r.disc[2], whole))
	return out, true
}
