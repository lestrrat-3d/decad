package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the coaxial separator certificate
// (agent-docs/interference-design.md §3.2, "The coaxial separator"): two
// solids whose interiors one circular cylinder separates. One lies in the
// closed solid cylinder of axis L and radius r, the other outside its open
// interior, so their interiors are disjoint whatever their boundaries share.
// A shaft pressed into a bore of its own radius is the case: the two cylinder
// faces coincide, which no held mesh can decide, but the separating cylinder
// needs no mesh at all.
//
// Each side is proven from records alone, exactly:
//
//   - a prism lies inside the cylinder over its outer loop when that loop is
//     one whole circle, and outside the cylinder over any hole that is one
//     whole circle; two such cylinders are the same set when exactly
//     orthonormal lifts give them one axis and radius, or when one affine lift
//     carries both;
//   - a Revolve on the axis of an exactly orthonormal cylinder lies outside
//     it when its least radius, scaled by its lift across the axis over its
//     sweep, reaches the cylinder's radius (coaxial_reach.go);
//   - a Cut result lies outside the cylinder of its tool, or of a Union
//     operand of its tool, that is a prism over one whole circle, when its
//     target lies within the tool's levels: the target's part inside the
//     cylinder then lies in the tool, which the Cut removes;
//   - a mesh Boolean result whose history denotes its operation exactly
//     (boolchain.Expandable) inherits containment: a Cut lies where its target
//     lies, an Intersect where either operand lies, a Union where both lie.

// coaxialCylinder is the cylinder a prism's lateral face over one whole
// circle lies on, kept as the prism's exact lift: the world point of section
// point (x, y) at level z is t + L·(x, y, z), and the circle has centre
// (cx, cy) and radius r in section coordinates. iso marks a lift whose
// linear part is exactly orthonormal, so the set is a circular cylinder of
// radius r about the lifted centre along L's third column.
type coaxialCylinder struct {
	l      [3][3]*big.Rat // columns: lifted u, v, n
	t      [3]*big.Rat
	cx, cy *big.Rat
	radius *big.Rat
	iso    bool
	// xform is the placement the lift composes, fl and ft the frame's own lift
	// before it, and frameIso whether that is exactly orthonormal.
	xform    r3.Transform
	fl       [3][3]*big.Rat
	ft       [3]*big.Rat
	frameIso bool
}

// coaxialMaxDepth bounds the history walk; a deeper chain stays undecided.
const coaxialMaxDepth = 64

// coaxialSeparated reports whether a circular cylinder separates a's and b's
// interiors. history may be nil, which leaves every mesh Boolean result a leaf.
func coaxialSeparated(ctx context.Context, history boolchain.Source, a, b *topo.Body) (bool, error) {
	for _, pair := range [][2]*topo.Body{{a, b}, {b, a}} {
		for _, c := range insideCylinders(history, pair[0], 0) {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			if outsideCylinder(history, pair[1], c, 0) {
				return true, nil
			}
		}
	}
	return false, ctx.Err()
}

// insideCylinders lists cylinders whose closed solid holds body.
func insideCylinders(history boolchain.Source, body *topo.Body, depth int) []coaxialCylinder {
	if depth > coaxialMaxDepth {
		return nil
	}
	body = sameSetOf(history, body)
	if pp, ok := body.Payload.(prismrecord.Payload); ok {
		if circle, ok := wholeCircle(pp.Profile.Outer); ok {
			if c, ok := prismCylinder(pp, circle); ok {
				return []coaxialCylinder{c}
			}
		}
		return nil
	}
	op, inputs, ok := expandRecorded(history, body)
	if !ok {
		return nil
	}
	switch op {
	case meshbool.OpCut:
		return insideCylinders(history, inputs[0], depth+1)
	case meshbool.OpIntersect:
		var out []coaxialCylinder
		for _, part := range inputs {
			out = append(out, insideCylinders(history, part, depth+1)...)
		}
		return out
	case meshbool.OpUnion:
		var out []coaxialCylinder
		for _, c := range insideCylinders(history, inputs[0], depth+1) {
			all := true
			for _, part := range inputs[1:] {
				if !insideCylinder(history, part, c, depth+1) {
					all = false
					break
				}
			}
			if all {
				out = append(out, c)
			}
		}
		return out
	}
	return nil
}

// insideCylinder reports whether c's closed solid holds body.
func insideCylinder(history boolchain.Source, body *topo.Body, c coaxialCylinder, depth int) bool {
	for _, own := range insideCylinders(history, body, depth) {
		if own.equal(c) {
			return true
		}
	}
	return false
}

// outsideCylinder reports whether body misses c's open interior.
func outsideCylinder(history boolchain.Source, body *topo.Body, c coaxialCylinder, depth int) bool {
	if depth > coaxialMaxDepth {
		return false
	}
	body = sameSetOf(history, body)
	if pp, ok := body.Payload.(prismrecord.Payload); ok {
		for _, hole := range pp.Profile.Holes {
			circle, ok := wholeCircle(hole)
			if !ok {
				continue
			}
			if own, ok := prismCylinder(pp, circle); ok && own.equal(c) {
				return true
			}
		}
		return false
	}
	if rp, ok := body.Payload.(revolveplan.Payload); ok {
		return revolveClearsCylinder(rp, c)
	}
	op, inputs, ok := expandRecorded(history, body)
	if !ok {
		return false
	}
	switch op {
	case meshbool.OpCut:
		return outsideCylinder(history, inputs[0], c, depth+1) || toolHolds(history, inputs[1], inputs[0], c, depth+1)
	case meshbool.OpIntersect:
		for _, part := range inputs {
			if outsideCylinder(history, part, c, depth+1) {
				return true
			}
		}
		return false
	case meshbool.OpUnion:
		for _, part := range inputs {
			if !outsideCylinder(history, part, c, depth+1) {
				return false
			}
		}
		return true
	}
	return false
}

func expandable(history boolchain.Source, body *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	if history == nil {
		return 0, nil, false
	}
	op, inputs, ok := boolchain.Expandable(history, body)
	if !ok || !arityHolds(op, inputs) {
		return 0, nil, false
	}
	return op, inputs, true
}

// arityHolds reports whether a recorded producer's inputs are the ones the
// walks read: a Cut's target and tool, or two or more operands of a Union or
// an Intersect.
func arityHolds(op meshbool.OperationKind, inputs []*topo.Body) bool {
	if op == meshbool.OpCut {
		return len(inputs) == 2
	}
	return len(inputs) >= 2
}

// wholeCircle returns loop's circle when the loop is one whole circle.
func wholeCircle(loop sectionrecord.LoopRecord) (sectionrecord.CircleSeg, bool) {
	if len(loop.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := loop.Segments[0].(sectionrecord.CircleSeg)
	whole := c.CCW && c.TStart == 0 && c.TEnd == 1 || !c.CCW && c.TStart == 1 && c.TEnd == 0
	return c, ok && whole
}

// prismCylinder is the cylinder a prism's lateral face over circle lies on.
// It refuses unless the section is held exactly.
func prismCylinder(pp prismrecord.Payload, circle sectionrecord.CircleSeg) (coaxialCylinder, bool) {
	if pp.SectionDelta != 0 || pp.SurfaceResult {
		return coaxialCylinder{}, false
	}
	r, err := circle.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) {
		return coaxialCylinder{}, false
	}
	l, t := prismLift(pp)
	fl, ft, frameIso := frameLift(pp.Frame, r3.Identity())
	return coaxialCylinder{
		l:        l,
		t:        t,
		cx:       ratOf(circle.Center.U),
		cy:       ratOf(circle.Center.V),
		radius:   ratOf(r),
		iso:      orthonormal(l[0], l[1], l[2]),
		xform:    pp.Xform,
		fl:       fl,
		ft:       ft,
		frameIso: frameIso,
	}, true
}

// prismLift is a prism's exact lift: the world point of section point (x, y)
// at level z is t + L·(x, y, z), with L's columns the frame axes carried by
// the placement basis, in rationals.
func prismLift(pp prismrecord.Payload) ([3][3]*big.Rat, [3]*big.Rat) {
	u, v, n, o := ratVec(pp.Frame.U()), ratVec(pp.Frame.V()), ratVec(pp.Frame.N()), ratVec(pp.Frame.Origin())
	basis := pp.Xform.Basis()
	ex, ey, ez := ratVec(basis.EX), ratVec(basis.EY), ratVec(basis.EZ)
	apply := func(p [3]*big.Rat) [3]*big.Rat {
		return addVec(addVec(scaleVec(ex, p[0]), scaleVec(ey, p[1])), scaleVec(ez, p[2]))
	}
	return [3][3]*big.Rat{apply(u), apply(v), apply(n)}, addVec(apply(o), ratVec(pp.Xform.Translation()))
}

// equal reports whether two cylinders are the same set. Two exactly
// orthonormal lifts are compared in the world: equal radius and the same axis
// line. Two lifts with the same linear part are compared in either's section
// coordinates, where the other's circle moves by the lift's own preimage of the
// translation between them; the same affine map then carries one circle's
// cylinder onto the other's, round or not.
func (c coaxialCylinder) equal(o coaxialCylinder) bool {
	if c.radius.Cmp(o.radius) != 0 {
		return false
	}
	if c.iso && o.iso {
		dc, do := c.l[2], o.l[2]
		if !isZero(crossVec(dc, do)) {
			return false
		}
		return isZero(crossVec(subVec(o.anchor(), c.anchor()), dc))
	}
	// One placement, bit for bit, is one affine bijection on both sets, so
	// they are the same set when their frames' exactly orthonormal lifts give
	// one axis line and radius before it.
	if c.xform == o.xform && c.frameIso && o.frameIso {
		dc, do := c.fl[2], o.fl[2]
		if !isZero(crossVec(dc, do)) {
			return false
		}
		frameAnchor := func(x coaxialCylinder) [3]*big.Rat {
			return addVec(x.ft, addVec(scaleVec(x.fl[0], x.cx), scaleVec(x.fl[1], x.cy)))
		}
		return isZero(crossVec(subVec(frameAnchor(o), frameAnchor(c)), dc))
	}
	for i := range 3 {
		for k := range 3 {
			if c.l[i][k].Cmp(o.l[i][k]) != 0 {
				return false
			}
		}
	}
	w, ok := solve(c.l, subVec(o.t, c.t))
	if !ok {
		return false
	}
	return new(big.Rat).Add(o.cx, w[0]).Cmp(c.cx) == 0 && new(big.Rat).Add(o.cy, w[1]).Cmp(c.cy) == 0
}

// anchor is the lifted circle centre at level zero.
func (c coaxialCylinder) anchor() [3]*big.Rat {
	return addVec(c.t, addVec(scaleVec(c.l[0], c.cx), scaleVec(c.l[1], c.cy)))
}

// solve returns x with cols·x = b, by Cramer's rule, and false when the
// columns are singular.
func solve(cols [3][3]*big.Rat, b [3]*big.Rat) ([3]*big.Rat, bool) {
	det := dotVec(cols[0], crossVec(cols[1], cols[2]))
	if det.Sign() == 0 {
		return [3]*big.Rat{}, false
	}
	x := [3]*big.Rat{
		dotVec(b, crossVec(cols[1], cols[2])),
		dotVec(cols[0], crossVec(b, cols[2])),
		dotVec(cols[0], crossVec(cols[1], b)),
	}
	for i := range x {
		x[i].Quo(x[i], det)
	}
	return x, true
}

func ratOf(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

func ratVec(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func addVec(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat).Add(a[i], b[i])
	}
	return out
}

func subVec(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func scaleVec(a [3]*big.Rat, s *big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat).Mul(a[i], s)
	}
	return out
}

func dotVec(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func crossVec(a, b [3]*big.Rat) [3]*big.Rat {
	mul := func(x, y *big.Rat) *big.Rat { return new(big.Rat).Mul(x, y) }
	return [3]*big.Rat{
		new(big.Rat).Sub(mul(a[1], b[2]), mul(a[2], b[1])),
		new(big.Rat).Sub(mul(a[2], b[0]), mul(a[0], b[2])),
		new(big.Rat).Sub(mul(a[0], b[1]), mul(a[1], b[0])),
	}
}

func isZero(a [3]*big.Rat) bool { return a[0].Sign() == 0 && a[1].Sign() == 0 && a[2].Sign() == 0 }

// orthonormal reports whether a, b, c are exactly unit length and pairwise
// orthogonal.
func orthonormal(a, b, c [3]*big.Rat) bool {
	one := big.NewRat(1, 1)
	for _, v := range [][3]*big.Rat{a, b, c} {
		if dotVec(v, v).Cmp(one) != 0 {
			return false
		}
	}
	return dotVec(a, b).Sign() == 0 && dotVec(b, c).Sign() == 0 && dotVec(a, c).Sign() == 0
}

// toolHolds reports whether tool, or an operand of a Union tool, is a prism
// over one whole circle whose cylinder is c and whose levels hold target along
// the cylinder's axis. Every point of target inside c then lies in that prism,
// so target less the tool misses c's open interior.
func toolHolds(history boolchain.Source, tool, target *topo.Body, c coaxialCylinder, depth int) bool {
	if depth > coaxialMaxDepth {
		return false
	}
	if pp, ok := tool.Payload.(prismrecord.Payload); ok {
		circle, ok := wholeCircle(pp.Profile.Outer)
		if !ok || len(pp.Profile.Holes) != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
			return false
		}
		own, ok := prismCylinder(pp, circle)
		if !ok || !own.equal(c) {
			return false
		}
		return withinLevels(history, target, pp, depth+1)
	}
	op, inputs, ok := expandRecorded(history, tool)
	if !ok || op != meshbool.OpUnion {
		return false
	}
	for _, part := range inputs {
		if toolHolds(history, part, target, c, depth+1) {
			return true
		}
	}
	return false
}

// withinLevels reports whether body lies between the levels of prism pp: with
// pp's lift t + L·(x, y, z), every point x of body has z = g·(x − t) in
// [z0, z1], where Lᵀg = e₃. The reach along ±g is read exactly; a Cut lies
// where its target lies, an Intersect where any operand lies, a Union where
// all lie.
func withinLevels(history boolchain.Source, body *topo.Body, pp prismrecord.Payload, depth int) bool {
	if depth > coaxialMaxDepth {
		return false
	}
	l, t := prismLift(pp)
	var cols [3][3]*big.Rat
	for i := range 3 {
		cols[i] = [3]*big.Rat{l[0][i], l[1][i], l[2][i]}
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	g, ok := solve(cols, [3]*big.Rat{zero, zero, one})
	if !ok {
		return false
	}
	base := dotVec(g, t)
	lo := new(big.Rat).Add(base, ratOf(min(pp.Z0, pp.Z1)))
	hi := new(big.Rat).Add(base, ratOf(max(pp.Z0, pp.Z1)))
	var leaf func(*topo.Body, int) bool
	leaf = func(b *topo.Body, depth int) bool {
		if depth > coaxialMaxDepth {
			return false
		}
		if up, ok := supportOf(b, g, nil); ok {
			down, ok := supportOf(b, scaleVec(g, big.NewRat(-1, 1)), nil)
			return ok && up.within(hi) && down.within(new(big.Rat).Neg(lo))
		}
		op, inputs, ok := expandRecorded(history, b)
		if !ok {
			return false
		}
		switch op {
		case meshbool.OpCut:
			return leaf(inputs[0], depth+1)
		case meshbool.OpIntersect:
			for _, part := range inputs {
				if leaf(part, depth+1) {
					return true
				}
			}
			return false
		case meshbool.OpUnion:
			for _, part := range inputs {
				if !leaf(part, depth+1) {
					return false
				}
			}
			return true
		}
		return false
	}
	return leaf(body, depth)
}
