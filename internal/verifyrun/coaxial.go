package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
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
//   - a prism whose frame and placement are exactly orthonormal lies inside
//     the cylinder over its outer loop when that loop is one whole circle, and
//     outside the cylinder over any hole that is one whole circle;
//   - a mesh Boolean result whose history denotes its operation exactly
//     (boolchain.Expandable) inherits containment: a Cut lies where its target
//     lies, an Intersect where either operand lies, a Union where both lie.

// coaxialCylinder is a circular cylinder in exact world coordinates: the line
// through Anchor along Dir, at radius Radius.
type coaxialCylinder struct {
	anchor, dir [3]*big.Rat
	radius      *big.Rat
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
	if pp, ok := body.Payload.(prismrecord.Payload); ok {
		if circle, ok := wholeCircle(pp.Profile.Outer); ok {
			if c, ok := prismCylinder(pp, circle); ok {
				return []coaxialCylinder{c}
			}
		}
		return nil
	}
	op, inputs, ok := expandable(history, body)
	if !ok {
		return nil
	}
	switch op {
	case meshbool.OpCut:
		return insideCylinders(history, inputs[0], depth+1)
	case meshbool.OpIntersect:
		return append(insideCylinders(history, inputs[0], depth+1), insideCylinders(history, inputs[1], depth+1)...)
	case meshbool.OpUnion:
		var out []coaxialCylinder
		for _, c := range insideCylinders(history, inputs[0], depth+1) {
			if insideCylinder(history, inputs[1], c, depth+1) {
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
	op, inputs, ok := expandable(history, body)
	if !ok {
		return false
	}
	switch op {
	case meshbool.OpCut:
		return outsideCylinder(history, inputs[0], c, depth+1)
	case meshbool.OpIntersect:
		return outsideCylinder(history, inputs[0], c, depth+1) || outsideCylinder(history, inputs[1], c, depth+1)
	case meshbool.OpUnion:
		return outsideCylinder(history, inputs[0], c, depth+1) && outsideCylinder(history, inputs[1], c, depth+1)
	}
	return false
}

func expandable(history boolchain.Source, body *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	if history == nil {
		return 0, nil, false
	}
	op, inputs, ok := boolchain.Expandable(history, body)
	if !ok || len(inputs) != 2 {
		return 0, nil, false
	}
	return op, inputs, true
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

// prismCylinder is the world cylinder a prism's lateral face over circle
// lies on: the section circle's centre lifted through the frame and the
// placement, along the frame normal. It refuses unless the section is held
// exactly and the frame axes and the placement basis are exactly
// orthonormal, so the lift is an exact isometry and every section point's
// distance from the centre is its world distance from the axis.
func prismCylinder(pp prismrecord.Payload, circle sectionrecord.CircleSeg) (coaxialCylinder, bool) {
	if pp.SectionDelta != 0 || pp.SurfaceResult {
		return coaxialCylinder{}, false
	}
	r, err := circle.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) {
		return coaxialCylinder{}, false
	}
	u, v, n, o := ratVec(pp.Frame.U()), ratVec(pp.Frame.V()), ratVec(pp.Frame.N()), ratVec(pp.Frame.Origin())
	if !orthonormal(u, v, n) {
		return coaxialCylinder{}, false
	}
	basis := pp.Xform.Basis()
	ex, ey, ez := ratVec(basis.EX), ratVec(basis.EY), ratVec(basis.EZ)
	if !orthonormal(ex, ey, ez) {
		return coaxialCylinder{}, false
	}
	cx, cy := ratOf(circle.Center.U), ratOf(circle.Center.V)
	local := addVec(o, addVec(scaleVec(u, cx), scaleVec(v, cy)))
	apply := func(p [3]*big.Rat) [3]*big.Rat {
		return addVec(addVec(scaleVec(ex, p[0]), scaleVec(ey, p[1])), scaleVec(ez, p[2]))
	}
	anchor := addVec(apply(local), ratVec(pp.Xform.Translation()))
	return coaxialCylinder{anchor: anchor, dir: apply(n), radius: ratOf(r)}, true
}

// equal reports whether two cylinders are the same set: equal radius, and
// the same axis line.
func (c coaxialCylinder) equal(o coaxialCylinder) bool {
	if c.radius.Cmp(o.radius) != 0 {
		return false
	}
	if !isZero(crossVec(c.dir, o.dir)) {
		return false
	}
	return isZero(crossVec(subVec(o.anchor, c.anchor), c.dir))
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
