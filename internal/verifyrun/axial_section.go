package verifyrun

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the axial section separator
// (agent-docs/interference-design.md §3.2, "The axial section separator"). A
// body a that lies in a cylinder about the sweep axis of a prism b, less the
// prisms its history cut from it, misses b's interior over b's levels when
// the cylinder's cross-section lies in the union of b's holes and the cut
// prisms' sections. A turned screw with a flat cut along its D-shaft sits in
// the D-shaped hole of its knob this way, though no circle of the hole is the
// shaft's own.

// axialApart reports whether a's interior misses b's through the axial
// section separator. b is the prism with holes; a is read through its Cut
// chain down to a Revolve or a prism, every cutting prism of the chain a tool
// that may remove material from the cover.
func axialApart(ctx context.Context, budget *proofbound.WorkBudget, history boolchain.Source,
	a, b *topo.Body) (bool, error) {
	b = sameSetOf(history, b)
	pb, ok := b.Payload.(prismrecord.Payload)
	if !ok || len(pb.Profile.Holes) == 0 || !prismExact(pb) {
		return false, nil
	}
	target, cutters := cutChain(history, a)
	blo, bhi := levels(pb, nil)
	var cover momentinput.Profile
	var lo, hi *big.Rat
	switch p := target.Payload.(type) {
	case revolveplan.Payload:
		m, ok := revolveMeridian(p)
		if !ok || m.disc != nil {
			return false, nil
		}
		var apart bool
		cover, lo, hi, apart, ok = revolveCover(m, pb, blo, bhi)
		if apart {
			return true, nil
		}
		if !ok {
			return false, nil
		}
	case prismrecord.Payload:
		if !prismExact(p) {
			return false, nil
		}
		off, ok := liftOffset(pb, p)
		if !ok {
			return false, nil
		}
		plo, phi := levels(p, off.dz)
		lo, hi = maxRat(plo, blo), minRat(phi, bhi)
		if hi.Cmp(lo) <= 0 {
			return true, nil
		}
		moved, ok := off.profile(momentinput.Profile{Outer: p.Profile.Outer})
		if !ok {
			return false, nil
		}
		cover = moved
	default:
		return false, nil
	}
	outer := make([]momentinput.Profile, 0, len(pb.Profile.Holes)+len(cutters))
	for _, hole := range pb.Profile.Holes {
		// A hole is walked clockwise in its section; as a region of its own it
		// is walked counter-clockwise.
		area, err := classbgeom.LoopSignedArea(hole)
		if err != nil {
			return false, nil //nolint:nilerr // a hole with no area reading gives no region, so no proof
		}
		if area < 0 {
			if hole, err = offset2d.ReverseLoopRecord(hole); err != nil {
				return false, nil //nolint:nilerr // a hole that will not reverse gives no region, so no proof
			}
		}
		outer = append(outer, momentinput.Profile{Outer: hole})
	}
	for _, c := range cutters {
		pt, ok := c.Payload.(prismrecord.Payload)
		if !ok || !prismExact(pt) {
			continue
		}
		off, ok := liftOffset(pb, pt)
		if !ok {
			if box, ok := boxCutter(pb, pt); ok && box.h0.Cmp(lo) <= 0 && box.h1.Cmp(hi) >= 0 {
				outer = append(outer, box.profile)
			}
			continue
		}
		tlo, thi := levels(pt, off.dz)
		if tlo.Cmp(lo) > 0 || thi.Cmp(hi) < 0 {
			continue
		}
		moved, ok := off.profile(pt.Profile)
		if !ok {
			continue
		}
		outer = append(outer, moved)
	}
	return prismcells.SectionWithinUnion(ctx, budget, []momentinput.Profile{cover}, outer)
}

// cutChain returns the body a Cut chain starts from and every prism it cut,
// a Union tool's operands each; a body that is no Cut is its own target.
func cutChain(history boolchain.Source, body *topo.Body) (*topo.Body, []*topo.Body) {
	var tools []*topo.Body
	for range historyMaxDepth {
		body = sameSetOf(history, body)
		op, inputs, ok := expandRecorded(history, body)
		if !ok || op != meshbool.OpCut {
			break
		}
		tools = append(tools, unionParts(history, inputs[1], 0)...)
		body = inputs[0]
	}
	return body, tools
}

// prismExact reports whether a prism holds its section and levels exactly.
func prismExact(p prismrecord.Payload) bool {
	if p.SurfaceResult {
		return false
	}
	for _, d := range []float64{p.SectionDelta, p.Z0Delta, p.Z1Delta} {
		if d != 0 {
			return false
		}
	}
	return true
}

// revolveCover reads a Revolve's cover in b's section coordinates: a closed
// disc holding every point of the revolve that lies at one of b's levels, and
// those levels [lo, hi] in b's level coordinates. The revolve is round in its
// frame's coordinates, and both bodies are exact rational images of their
// recorded sets, so a point of the revolve sits in b's coordinates at
//
//	c + z·g3 + ρ·(cos φ·g1 + sin φ·g2)
//
// with c the image of the axis anchor and g3, g1, g2 the images of the axis
// direction and of the two unit directions across it, all solved exactly
// against b's lift. The images need not be orthonormal: where the two lifts
// are exactly a rotation of one another, g1 and g2 are orthonormal across b's
// sweep, g3 lies along it, and the cover is the exact disc of the largest ρ.
// Otherwise the section coordinates of g1 and g2 are bounded by their
// spectral norm and the lean of g3 by its length, each square root rounded up
// and checked exactly, and the disc widens by them. apart reports that no
// volume of the revolve lies at b's levels.
func revolveCover(m meridian, pb prismrecord.Payload, blo, bhi *big.Rat) (
	cover momentinput.Profile, lo, hi *big.Rat, apart, ok bool) {
	fail := func() (momentinput.Profile, *big.Rat, *big.Rat, bool, bool) {
		return momentinput.Profile{}, nil, nil, false, false
	}
	lb, tb := prismLift(pb)
	vec := func(v [3]*big.Rat) ([3]*big.Rat, bool) {
		return solve(lb, addVec(addVec(scaleVec(m.l[0], v[0]), scaleVec(m.l[1], v[1])), scaleVec(m.l[2], v[2])))
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	// up is √x rounded up to a float; a root it cannot bound fails the cover.
	failed := false
	up := func(x *big.Rat) *big.Rat {
		r, ok := sqrtUpper(x)
		failed = failed || !ok
		if !ok {
			return new(big.Rat)
		}
		return r
	}
	centre, ok1 := solve(lb, subVec(addVec(addVec(addVec(scaleVec(m.l[0], m.anchor[0]),
		scaleVec(m.l[1], m.anchor[1])), scaleVec(m.l[2], m.anchor[2])), m.t), tb))
	g3, ok2 := vec(m.dir)
	g1, ok3 := vec([3]*big.Rat{new(big.Rat).Neg(m.dir[1]), m.dir[0], zero})
	g2, ok4 := vec([3]*big.Rat{zero, zero, one})
	if !ok1 || !ok2 || !ok3 || !ok4 || new(big.Rat).Abs(g3[2]).Cmp(big.NewRat(1, 2)) < 0 {
		return fail()
	}
	rhoMax := m.region[0][0]
	for _, q := range m.region {
		rhoMax = maxRat(rhoMax, q[0])
	}
	// Every point at one of b's levels sits within ρ·η of the axis's level.
	eta := up(new(big.Rat).Add(new(big.Rat).Mul(g1[2], g1[2]), new(big.Rat).Mul(g2[2], g2[2])))
	slack := new(big.Rat).Mul(rhoMax, eta)
	zA := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Sub(blo, centre[2]), slack), g3[2])
	zB := new(big.Rat).Quo(new(big.Rat).Add(new(big.Rat).Sub(bhi, centre[2]), slack), g3[2])
	if zA.Cmp(zB) > 0 {
		zA, zB = zB, zA
	}
	pts := slabPoints(m.region, zA, zB)
	if len(pts) == 0 {
		return momentinput.Profile{}, nil, nil, true, true
	}
	zMin, zMax := pts[0][1], pts[0][1]
	for _, q := range pts {
		zMin, zMax = minRat(zMin, q[1]), maxRat(zMax, q[1])
	}
	zc := new(big.Rat).Quo(new(big.Rat).Add(zMin, zMax), big.NewRat(2, 1))
	// The section plane's reading of the unit directions across the axis, as
	// the spectral norm of the 2x2 matrix [g1 g2], and of the axis's lean.
	a := new(big.Rat).Add(new(big.Rat).Mul(g1[0], g1[0]), new(big.Rat).Mul(g1[1], g1[1]))
	b := new(big.Rat).Add(new(big.Rat).Mul(g1[0], g2[0]), new(big.Rat).Mul(g1[1], g2[1]))
	c := new(big.Rat).Add(new(big.Rat).Mul(g2[0], g2[0]), new(big.Rat).Mul(g2[1], g2[1]))
	half := new(big.Rat).Quo(new(big.Rat).Sub(a, c), big.NewRat(2, 1))
	root := up(new(big.Rat).Add(new(big.Rat).Mul(half, half), new(big.Rat).Mul(b, b)))
	sigma := up(new(big.Rat).Add(new(big.Rat).Quo(new(big.Rat).Add(a, c), big.NewRat(2, 1)), root))
	lean := up(new(big.Rat).Add(new(big.Rat).Mul(g3[0], g3[0]), new(big.Rat).Mul(g3[1], g3[1])))
	radius := new(big.Rat)
	for _, q := range pts {
		reach := new(big.Rat).Add(new(big.Rat).Mul(q[0], sigma),
			new(big.Rat).Mul(new(big.Rat).Abs(new(big.Rat).Sub(q[1], zc)), lean))
		radius = maxRat(radius, reach)
	}
	cu := new(big.Rat).Add(centre[0], new(big.Rat).Mul(zc, g3[0]))
	cv := new(big.Rat).Add(centre[1], new(big.Rat).Mul(zc, g3[1]))
	cuf, _ := cu.Float64()
	cvf, _ := cv.Float64()
	du, dv := new(big.Rat).Sub(cu, ratOf(cuf)), new(big.Rat).Sub(cv, ratOf(cvf))
	// A centre rounded to a float widens the disc by its distance from it.
	radius.Add(radius, up(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))))
	r, _ := radius.Float64()
	if ratOf(r).Cmp(radius) < 0 {
		r = math.Nextafter(r, math.Inf(1))
	}
	reachH := func(z *big.Rat) *big.Rat { return new(big.Rat).Add(centre[2], new(big.Rat).Mul(z, g3[2])) }
	lo, hi = reachH(zMin), reachH(zMax)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	lo, hi = maxRat(new(big.Rat).Sub(lo, slack), blo), minRat(new(big.Rat).Add(hi, slack), bhi)
	if hi.Cmp(lo) <= 0 {
		return momentinput.Profile{}, nil, nil, true, true
	}
	if failed || math.IsInf(r, 0) {
		return fail()
	}
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: cuf, V: cvf}, Radius: units.Millimeters(r),
		CCW: true, TStart: 0, TEnd: 1}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		lo, hi, false, true
}

// slabPoints lists the vertices of the (ρ, z) polygon that bound its interior
// inside the open slab zLo < z < zHi: each vertex strictly inside, and each end
// of a slanted or vertical edge's piece in the closed slab when that piece
// rises through it. A horizontal edge on the slab's plane, or a vertex that
// only touches it, bounds no interior inside the slab and is left out.
func slabPoints(poly [][2]*big.Rat, zLo, zHi *big.Rat) [][2]*big.Rat {
	var out [][2]*big.Rat
	for i, p := range poly {
		q := poly[(i+1)%len(poly)]
		if p[1].Cmp(zLo) > 0 && p[1].Cmp(zHi) < 0 {
			out = append(out, p)
		}
		if p[1].Cmp(q[1]) == 0 {
			continue
		}
		// The edge's piece in the closed slab, by parameter t in [0, 1].
		t0, t1 := new(big.Rat), big.NewRat(1, 1)
		at := func(z *big.Rat) *big.Rat {
			return new(big.Rat).Quo(new(big.Rat).Sub(z, p[1]), new(big.Rat).Sub(q[1], p[1]))
		}
		tLo, tHi := at(zLo), at(zHi)
		if tLo.Cmp(tHi) > 0 {
			tLo, tHi = tHi, tLo
		}
		t0, t1 = maxRat(t0, tLo), minRat(t1, tHi)
		if t0.Cmp(t1) >= 0 {
			continue
		}
		for _, t := range []*big.Rat{t0, t1} {
			out = append(out, [2]*big.Rat{
				new(big.Rat).Add(p[0], new(big.Rat).Mul(t, new(big.Rat).Sub(q[0], p[0]))),
				new(big.Rat).Add(p[1], new(big.Rat).Mul(t, new(big.Rat).Sub(q[1], p[1]))),
			})
		}
	}
	return out
}

func maxRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

func minRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

// cutterBox is a box-shaped cutter read in b's coordinates: its section in
// b's section plane, and the levels h0..h1 along b's sweep over which that
// section is the cutter's.
type cutterBox struct {
	profile momentinput.Profile
	h0, h1  *big.Rat
}

// boxCutter reads a prism whose section is one axis-aligned rectangle of whole
// lines, on a frame whose three axes are b's three, signed and permuted, as a
// box in b's coordinates. Both frames are compared in the coordinates before
// one placement when they carry it bit for bit, else in the world when both
// lifts are exactly orthonormal. A flat milled across a turned shaft is such a
// box beside the prism sweeping along the shaft.
func boxCutter(pb, pt prismrecord.Payload) (cutterBox, bool) {
	u0, v0, u1, v1, ok := axisRectangle(pt.Profile)
	if !ok {
		return cutterBox{}, false
	}
	lb, tb := prismLift(pb)
	lt, tt := prismLift(pt)
	if pb.Xform == pt.Xform {
		var okB, okT bool
		lb, tb, okB = frameLift(pb.Frame, r3.Identity())
		lt, tt, okT = frameLift(pt.Frame, r3.Identity())
		if !okB || !okT {
			return cutterBox{}, false
		}
	} else if !orthonormal(lb[0], lb[1], lb[2]) || !orthonormal(lt[0], lt[1], lt[2]) {
		return cutterBox{}, false
	}
	z0, z1 := levels(pt, nil)
	own := [3][2]*big.Rat{{u0, u1}, {v0, v1}, {z0, z1}}
	rel := subVec(tt, tb)
	var lo, hi [3]*big.Rat
	for k := range 3 {
		axis := -1
		var sign *big.Rat
		for c := range 3 {
			d := dotVec(lt[k], lb[c])
			switch {
			case d.Sign() == 0:
			case d.Cmp(big.NewRat(1, 1)) == 0 || d.Cmp(big.NewRat(-1, 1)) == 0:
				if axis >= 0 {
					return cutterBox{}, false
				}
				axis, sign = c, d
			default:
				return cutterBox{}, false
			}
		}
		if axis < 0 || lo[axis] != nil {
			return cutterBox{}, false
		}
		base := dotVec(rel, lb[axis])
		a := new(big.Rat).Add(base, new(big.Rat).Mul(sign, own[k][0]))
		b := new(big.Rat).Add(base, new(big.Rat).Mul(sign, own[k][1]))
		lo[axis], hi[axis] = minRat(a, b), maxRat(a, b)
	}
	region := [][2]*big.Rat{{lo[0], lo[1]}, {hi[0], lo[1]}, {hi[0], hi[1]}, {lo[0], hi[1]}}
	profile, ok := regionProfile(region, nil)
	if !ok {
		return cutterBox{}, false
	}
	return cutterBox{profile: profile, h0: lo[2], h1: hi[2]}, true
}

// axisRectangle reads a hole-free section that is one rectangle of four whole
// lines parallel to the section's axes.
func axisRectangle(p momentinput.Profile) (u0, v0, u1, v1 *big.Rat, ok bool) {
	segs := p.Outer.Segments
	if len(p.Holes) != 0 || len(segs) != 4 {
		return nil, nil, nil, nil, false
	}
	var us, vs []*big.Rat
	for i, seg := range segs {
		start, end, whole := wholeLine(seg)
		if !whole {
			return nil, nil, nil, nil, false
		}
		if next, _, whole := wholeLine(segs[(i+1)%4]); !whole || next != end {
			return nil, nil, nil, nil, false
		}
		if start.U != end.U && start.V != end.V {
			return nil, nil, nil, nil, false
		}
		us, vs = append(us, ratOf(start.U)), append(vs, ratOf(start.V))
	}
	u0, u1, v0, v1 = us[0], us[0], vs[0], vs[0]
	for i := range 4 {
		u0, u1 = minRat(u0, us[i]), maxRat(u1, us[i])
		v0, v1 = minRat(v0, vs[i]), maxRat(v1, vs[i])
	}
	return u0, v0, u1, v1, u0.Cmp(u1) != 0 && v0.Cmp(v1) != 0
}
