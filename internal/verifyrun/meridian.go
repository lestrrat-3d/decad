package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the coaxial meridian separator
// (agent-docs/interference-design.md §3.2, "The meridian separator"). A body
// that lies in the full revolution of a region of the (ρ, z) half-plane about
// an axis is read as that region. Two bodies on one axis then have disjoint
// interiors when their regions do: the revolutions of interior-disjoint
// regions about one axis share no interior point. A full Revolve is its own
// meridian; a prism over a whole circle centred on the axis lies in the
// rectangle of that radius over its levels; a prism with such a circular hole
// lies in the strip outside the hole's radius over its levels. Every frame is
// checked exactly orthonormal and every axis exactly unit, so each body's
// frame coordinates measure true distance. The two bodies are compared in
// a's frame coordinates when their lifts share one linear part, b's moving
// by the exact preimage of the translation between them, or in the world when
// both lifts are exactly orthonormal.

// meridian is a region of the (ρ, z) half-plane about an axis through anchor
// along the exact unit direction dir, both in the body's frame coordinates,
// which its lift t + L·x carries to the world. A nil ρ stands for a radius
// beyond every finite radius of the two regions compared.
type meridian struct {
	l           [3][3]*big.Rat
	t           [3]*big.Rat
	anchor, dir [3]*big.Rat
	region      [][2]*big.Rat // polygon vertices (ρ, z), in order
	// xform is the placement the lift composes, and fl, ft the frame's own
	// exactly orthonormal lift before it.
	xform r3.Transform
	fl    [3][3]*big.Rat
	ft    [3]*big.Rat
	// disc, when set, makes the region the closed disc of centre
	// (disc[0], disc[1]) and radius disc[2] in place of a polygon.
	disc *[3]*big.Rat
}

// meridiansOf lists the meridian regions body lies in.
func meridiansOf(body *topo.Body) []meridian {
	switch p := body.Payload.(type) {
	case revolveplan.Payload:
		if m, ok := revolveMeridian(p); ok {
			return []meridian{m}
		}
	case prismrecord.Payload:
		return prismMeridians(p)
	}
	return nil
}

// frameLift is the exact lift of a frame under a placement: frame point
// (x, y, z) sits at t + L·(x, y, z) in the world. It also reports whether the
// frame itself is exactly orthonormal.
func frameLift(f r3.Frame, xf r3.Transform) ([3][3]*big.Rat, [3]*big.Rat, bool) {
	u, v, n, o := ratVec(f.U()), ratVec(f.V()), ratVec(f.N()), ratVec(f.Origin())
	basis := xf.Basis()
	ex, ey, ez := ratVec(basis.EX), ratVec(basis.EY), ratVec(basis.EZ)
	apply := func(p [3]*big.Rat) [3]*big.Rat {
		return addVec(addVec(scaleVec(ex, p[0]), scaleVec(ey, p[1])), scaleVec(ez, p[2]))
	}
	return [3][3]*big.Rat{apply(u), apply(v), apply(n)}, addVec(apply(o), ratVec(xf.Translation())), orthonormal(u, v, n)
}

// revolveMeridian reads a solid, undisplaced Revolve, with no snap onto the
// axis and no material admitted across it, whose meridian is one chained loop
// of whole lines, or one whole circle, on one side of an exactly unit axis.
// Its holes only remove material, so the outer loop's region holds the body,
// and a partial sweep lies in the full revolution of that region.
func revolveMeridian(p revolveplan.Payload) (meridian, bool) {
	ax := p.Ax
	if p.SurfaceResult || p.SectionDelta != 0 || ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 ||
		ax.RadialAdmitAllow != 0 || ax.Snap != (revolveaxis.SnapAllow{}) {
		return meridian{}, false
	}
	l, t, ok := frameLift(p.Frame, p.Xform)
	if !ok {
		return meridian{}, false
	}
	du, dv := ratOf(ax.DU), ratOf(ax.DV)
	if new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)).Cmp(big.NewRat(1, 1)) != 0 {
		return meridian{}, false
	}
	au, av := ratOf(ax.AU), ratOf(ax.AV)
	zero := new(big.Rat)
	toAxis := func(pt sectionrecord.Point2) (*big.Rat, *big.Rat) {
		pu := new(big.Rat).Sub(ratOf(pt.U), au)
		pv := new(big.Rat).Sub(ratOf(pt.V), av)
		z := new(big.Rat).Add(new(big.Rat).Mul(pu, du), new(big.Rat).Mul(pv, dv))
		rho := new(big.Rat).Sub(new(big.Rat).Mul(pv, du), new(big.Rat).Mul(pu, dv))
		return rho, z
	}
	if c, ok := wholeCircle(p.Profile.Outer); ok {
		r, err := c.Radius.In(units.Millimeter)
		if err != nil || !(r > 0) {
			return meridian{}, false
		}
		rho, z := toAxis(c.Center)
		rr := ratOf(r)
		// The disc lies on one side of the axis when |ρ| ≥ r.
		if new(big.Rat).Abs(rho).Cmp(rr) < 0 {
			return meridian{}, false
		}
		rho.Abs(rho)
		fl, ft, _ := frameLift(p.Frame, r3.Identity())
		return meridian{l: l, t: t, xform: p.Xform, fl: fl, ft: ft, anchor: [3]*big.Rat{au, av, zero}, dir: [3]*big.Rat{du, dv, zero},
			disc: &[3]*big.Rat{rho, z, rr}}, true
	}
	segs := p.Profile.Outer.Segments
	var region [][2]*big.Rat
	side := 0
	for i, seg := range segs {
		start, end, ok := wholeLine(seg)
		if !ok {
			return meridian{}, false
		}
		if next, _, ok := wholeLine(segs[(i+1)%len(segs)]); !ok || next != end {
			return meridian{}, false
		}
		rho, z := toAxis(start)
		if s := rho.Sign(); s != 0 {
			if side != 0 && s != side {
				return meridian{}, false
			}
			side = s
		}
		region = append(region, [2]*big.Rat{rho, z})
	}
	if side == 0 || len(region) < 3 {
		return meridian{}, false
	}
	if side < 0 {
		for _, q := range region {
			q[0].Neg(q[0])
		}
	}
	fl, ft, _ := frameLift(p.Frame, r3.Identity())
	return meridian{l: l, t: t, xform: p.Xform, fl: fl, ft: ft, anchor: [3]*big.Rat{au, av, zero}, dir: [3]*big.Rat{du, dv, zero}, region: region}, true
}

// wholeLine returns a whole line's ends in walk order.
func wholeLine(seg sectionrecord.CurveSegment) (sectionrecord.Point2, sectionrecord.Point2, bool) {
	line, ok := seg.(sectionrecord.LineSeg)
	switch {
	case !ok:
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	case line.TStart == 0 && line.TEnd == 1:
		return line.Start, line.End, true
	case line.TStart == 1 && line.TEnd == 0:
		return line.End, line.Start, true
	}
	return sectionrecord.Point2{}, sectionrecord.Point2{}, false
}

// prismMeridians reads a prism with an exact section and levels on an exactly
// orthonormal frame: the rectangle under its outer circle, and the strip
// outside each circular hole, each over its levels.
func prismMeridians(p prismrecord.Payload) []meridian {
	if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 {
		return nil
	}
	l, t, ok := frameLift(p.Frame, p.Xform)
	if !ok {
		return nil
	}
	fl, ft, _ := frameLift(p.Frame, r3.Identity())
	z0, z1 := ratOf(min(p.Z0, p.Z1)), ratOf(max(p.Z0, p.Z1))
	zero, one := new(big.Rat), big.NewRat(1, 1)
	cylinder := func(c sectionrecord.CircleSeg, r0, r1 *big.Rat) meridian {
		return meridian{
			l: l, t: t, xform: p.Xform, fl: fl, ft: ft,
			anchor: [3]*big.Rat{ratOf(c.Center.U), ratOf(c.Center.V), zero},
			dir:    [3]*big.Rat{zero, zero, one},
			region: [][2]*big.Rat{{r0, z0}, {r1, z0}, {r1, z1}, {r0, z1}},
		}
	}
	var out []meridian
	if c, ok := wholeCircle(p.Profile.Outer); ok {
		if r, err := c.Radius.In(units.Millimeter); err == nil && r > 0 {
			out = append(out, cylinder(c, zero, ratOf(r)))
		}
	}
	for _, h := range p.Profile.Holes {
		c, ok := wholeCircle(h)
		if !ok {
			continue
		}
		if r, err := c.Radius.In(units.Millimeter); err == nil && r > 0 {
			out = append(out, cylinder(c, ratOf(r), nil))
		}
	}
	return out
}

// meridianApart reports whether some meridian of a and some meridian of b
// lie on one axis line with interior-disjoint regions.
func meridianApart(ctx context.Context, budget *proofbound.WorkBudget, a, b *topo.Body) (bool, error) {
	for _, ma := range meridiansOf(a) {
		for _, mb := range meridiansOf(b) {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			pa, pb, ok := sharedMeridians(ma, mb)
			if !ok {
				continue
			}
			apart, err := prismcells.SectionsApart(ctx, budget, []momentinput.Profile{pa}, []momentinput.Profile{pb})
			if err != nil || apart {
				return apart, err
			}
		}
	}
	return false, nil
}

// sharedMeridians returns both regions as profiles in a's (ρ, z) coordinates
// when the two meridians lie on one axis line, and every coordinate is
// exactly a float.
func sharedMeridians(a, b meridian) (momentinput.Profile, momentinput.Profile, bool) {
	aAnchor, aDir, bAnchor, bDir, ok := commonAxes(a, b)
	if !ok || !isZero(crossVec(aDir, bDir)) || !isZero(crossVec(subVec(bAnchor, aAnchor), aDir)) {
		return momentinput.Profile{}, momentinput.Profile{}, false
	}
	norm := dotVec(aDir, aDir)
	shift := new(big.Rat).Quo(dotVec(subVec(bAnchor, aAnchor), aDir), norm)
	sign := new(big.Rat).Quo(dotVec(aDir, bDir), norm)
	moved := meridian{region: make([][2]*big.Rat, len(b.region))}
	for i, q := range b.region {
		moved.region[i] = [2]*big.Rat{q[0], new(big.Rat).Add(shift, new(big.Rat).Mul(sign, q[1]))}
	}
	if b.disc != nil {
		moved.disc = &[3]*big.Rat{b.disc[0], new(big.Rat).Add(shift, new(big.Rat).Mul(sign, b.disc[1])), b.disc[2]}
	}
	far := big.NewRat(1, 1)
	for _, m := range []meridian{a, moved} {
		reach := []*big.Rat{}
		for _, q := range m.region {
			reach = append(reach, q[0])
		}
		if m.disc != nil {
			reach = append(reach, new(big.Rat).Add(m.disc[0], m.disc[2]))
		}
		for _, r := range reach {
			if r != nil && r.Cmp(far) >= 0 {
				far = new(big.Rat).Add(r, big.NewRat(1, 1))
			}
		}
	}
	pa, okA := meridianProfile(a, far)
	pb, okB := meridianProfile(moved, far)
	return pa, pb, okA && okB
}

// meridianProfile is m's region as a profile: its disc as one whole circle,
// else its polygon (regionProfile).
func meridianProfile(m meridian, far *big.Rat) (momentinput.Profile, bool) {
	if m.disc == nil {
		return regionProfile(m.region, far)
	}
	var f [3]float64
	for i, v := range m.disc {
		x, _ := v.Float64()
		if ratOf(x).Cmp(v) != 0 {
			return momentinput.Profile{}, false
		}
		f[i] = x
	}
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: f[0], V: f[1]}, Radius: units.Millimeters(f[2]),
		CCW: true, TStart: 0, TEnd: 1}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}, true
}

// commonAxes places both axes in one exactly Euclidean coordinate system: a's
// frame coordinates when the lifts share a linear part, b's moving by the
// exact preimage of the translation between them; else the coordinates before
// one placement both carry bit for bit; else the world when both lifts are
// exactly orthonormal.
func commonAxes(a, b meridian) (aAnchor, aDir, bAnchor, bDir [3]*big.Rat, ok bool) {
	same := true
	for i := range 3 {
		for k := range 3 {
			same = same && a.l[i][k].Cmp(b.l[i][k]) == 0
		}
	}
	if same {
		w, ok := solve(a.l, subVec(b.t, a.t))
		if !ok {
			return aAnchor, aDir, bAnchor, bDir, false
		}
		return a.anchor, a.dir, addVec(b.anchor, w), b.dir, true
	}
	// One placement, bit for bit, is one affine bijection on both bodies, so
	// they are compared before it, where their frames are exactly
	// orthonormal and measure true distance.
	sharedPlacement := a.xform == b.xform && a.fl[0][0] != nil && b.fl[0][0] != nil
	if !sharedPlacement && (!orthonormal(a.l[0], a.l[1], a.l[2]) || !orthonormal(b.l[0], b.l[1], b.l[2])) {
		return aAnchor, aDir, bAnchor, bDir, false
	}
	world := func(m meridian, p [3]*big.Rat, point bool) [3]*big.Rat {
		l, t := m.l, m.t
		if sharedPlacement {
			l, t = m.fl, m.ft
		}
		out := addVec(addVec(scaleVec(l[0], p[0]), scaleVec(l[1], p[1])), scaleVec(l[2], p[2]))
		if point {
			out = addVec(out, t)
		}
		return out
	}
	return world(a, a.anchor, true), world(a, a.dir, false), world(b, b.anchor, true), world(b, b.dir, false), true
}

// regionProfile turns a polygon into a counter-clockwise profile of whole
// lines, reading a nil ρ as far and refusing a coordinate that is not exactly
// a float.
func regionProfile(region [][2]*big.Rat, far *big.Rat) (momentinput.Profile, bool) {
	pts := make([][2]*big.Rat, len(region))
	for i, p := range region {
		pts[i] = p
		if p[0] == nil {
			pts[i] = [2]*big.Rat{far, p[1]}
		}
	}
	area := new(big.Rat)
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(p[0], q[1]), new(big.Rat).Mul(q[0], p[1])))
	}
	if area.Sign() == 0 {
		return momentinput.Profile{}, false
	}
	points := make([]sectionrecord.Point2, len(pts))
	for i, p := range pts {
		u, _ := p[0].Float64()
		v, _ := p[1].Float64()
		if ratOf(u).Cmp(p[0]) != 0 || ratOf(v).Cmp(p[1]) != 0 {
			return momentinput.Profile{}, false
		}
		points[i] = sectionrecord.Point2{U: u, V: v}
	}
	if area.Sign() < 0 {
		for i, j := 0, len(points)-1; i < j; i, j = i+1, j-1 {
			points[i], points[j] = points[j], points[i]
		}
	}
	var loop sectionrecord.LoopRecord
	for i := range points {
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{Start: points[i], End: points[(i+1)%len(points)], TEnd: 1})
	}
	return momentinput.Profile{Outer: loop}, true
}
