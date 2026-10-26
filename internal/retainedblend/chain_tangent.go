package retainedblend

import (
	"math"
	"math/big"

	clr "github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file decides the tangent junction of agent-docs/retained-modify-reach-design.md
// §5.2 from the retained tags alone. Every "yes" is an identity over the tags'
// floats read as rationals. A "no" is a clear structural or numeric miss, and a
// gap inside the angular noise band is undecided: a tolerance never proves
// "tangent".

// ratVec is a vector of rationals.
type ratVec [3]*big.Rat

func ratOf(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

func ratVecOf(v r3.Vec) ratVec { return ratVec{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func (a ratVec) sub(b ratVec) ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func (a ratVec) add(b ratVec) ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Add(a[i], b[i])
	}
	return out
}

func (a ratVec) scale(s *big.Rat) ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Mul(a[i], s)
	}
	return out
}

func (a ratVec) dot(b ratVec) *big.Rat {
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}

func (a ratVec) cross(b ratVec) ratVec {
	mul := func(x, y *big.Rat) *big.Rat { return new(big.Rat).Mul(x, y) }
	return ratVec{
		new(big.Rat).Sub(mul(a[1], b[2]), mul(a[2], b[1])),
		new(big.Rat).Sub(mul(a[2], b[0]), mul(a[0], b[2])),
		new(big.Rat).Sub(mul(a[0], b[1]), mul(a[1], b[0])),
	}
}

func (a ratVec) isZero() bool {
	return a[0].Sign() == 0 && a[1].Sign() == 0 && a[2].Sign() == 0
}

func (a ratVec) float() r3.Vec {
	x, _ := a[0].Float64()
	y, _ := a[1].Float64()
	z, _ := a[2].Float64()
	return r3.NewVec(x, y, z)
}

// chainWall is a chain wall's retained tag read as exact rationals.
type chainWall struct {
	face     *topo.Face
	isPlane  bool
	n, o     ratVec // plane: the frame normal and origin as held
	a, c     ratVec // cylinder: axis and a point on it
	radius   *big.Rat
	reversed bool
	// fn and fa are the held normal or axis, for the noise band.
	fn, fa r3.Vec
	fo, fc r3.Vec
	fr     float64
}

// readChainTag reads a retained plane or cylinder face. It returns false for
// any other face and for a face with a stated tag departure or denoted patch.
func readChainTag(f *topo.Face) (chainWall, bool) {
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || f.Denoted != nil {
		return chainWall{}, false
	}
	w := chainWall{face: f, reversed: f.Reversed}
	switch s := f.Surface.(type) {
	case surfacegeom.Plane:
		n := s.Frame.N()
		if !proofbound.FiniteVec(n) || !proofbound.FiniteVec(s.Frame.Origin()) || n.Len() == 0 {
			return chainWall{}, false
		}
		w.isPlane = true
		w.fn, w.fo = n, s.Frame.Origin()
		w.n, w.o = ratVecOf(n), ratVecOf(w.fo)
	case surfacegeom.Cylinder:
		radius, err := s.Radius.In(units.Millimeter)
		if err != nil || !(radius > 0) || math.IsInf(radius, 0) || !proofbound.FiniteVec(s.Axis) ||
			!proofbound.FiniteVec(s.Origin) || s.Axis.Len() == 0 {
			return chainWall{}, false
		}
		w.fa, w.fc, w.fr = s.Axis, s.Origin, radius
		w.a, w.c, w.radius = ratVecOf(s.Axis), ratVecOf(s.Origin), ratOf(radius)
	default:
		return chainWall{}, false
	}
	return w, true
}

// Chainable reports whether e is an edge that tangent-chain expansion on a
// retained body reads (§5.3): a retained Line3 or Arc3 between two faces that
// carry a retained plane or cylinder tag with no stated departure.
func Chainable(e *topo.Edge) bool {
	if e == nil || e.Start == nil || e.End == nil || e.Start == e.End || !e.RetainedCurve || len(e.Faces) != 2 {
		return false
	}
	switch e.Curve.(type) {
	case surfacegeom.Line3, surfacegeom.Arc3:
	default:
		return false
	}
	for _, f := range e.Faces {
		if _, ok := readChainTag(f); !ok {
			return false
		}
	}
	return true
}

// junction is a proven tangent junction of two chain edges at a vertex.
type junction struct {
	point r3.Vec // V = g ∩ P, rounded once from its rational value
	base  *topo.Face
	// wall1 and wall2 are the walls of the first and second edge.
	wall1, wall2 *topo.Face
}

// Tangent decides whether c continues e through their shared vertex v by the
// exact identities of §5.2. A junction that cannot be read from the tags is
// undecided, as is a gap inside the noise band.
func Tangent(e, c *topo.Edge, v *topo.Vertex) clr.DegState {
	if c == nil || c.Start == c.End || (c.Start != v && c.End != v) || len(e.Faces) != 2 || len(c.Faces) != 2 {
		return clr.DegNo
	}
	_, state, _ := tangentState(e, c, v)
	return state
}

// tangentState decides the junction and reports whether an undecided answer
// is a near tangent: two readable chain edges whose tags nearly meet.
func tangentState(e, c *topo.Edge, v *topo.Vertex) (junction, clr.DegState, bool) {
	var base *topo.Face
	shared := 0
	for _, f := range e.Faces {
		for _, g := range c.Faces {
			if f == g {
				base = f
				shared++
			}
		}
	}
	if shared != 1 {
		return junction{}, clr.DegNo, false
	}
	other := func(x *topo.Edge) *topo.Face {
		if x.Faces[0] == base {
			return x.Faces[1]
		}
		return x.Faces[0]
	}
	w1, w2 := other(e), other(c)
	if w1 == w2 {
		return junction{}, clr.DegNo, false
	}
	if !Chainable(e) || !Chainable(c) {
		return junction{}, clr.DegUnknown, false
	}
	bt, _ := readChainTag(base)
	if !bt.isPlane {
		return junction{}, clr.DegNo, false
	}
	t1, _ := readChainTag(w1)
	t2, _ := readChainTag(w2)
	if t1.isPlane && t2.isPlane {
		return junction{}, clr.DegNo, false
	}
	state, near := clr.DegYes, false
	for _, t := range []chainWall{t1, t2} {
		if t.isPlane {
			continue
		}
		// A cylinder wall's axis is normal to the base exactly.
		s, nr := bandOf(ratVec.cross(t.a, bt.n), ratCross(t.fa, bt.fn), t.fa, bt.fn)
		state, near = meet(state, s), near || nr
	}
	if state == clr.DegNo {
		return junction{}, clr.DegNo, false
	}
	var g ratVec // a point on the common generator g
	var dirr ratVec
	switch {
	case t1.isPlane || t2.isPlane:
		pl, cy := t1, t2
		if t2.isPlane {
			pl, cy = t2, t1
		}
		s, nr := bandOf(ratVec{cy.a.dot(pl.n), new(big.Rat), new(big.Rat)}, r3.Vec{X: cy.fa.Dot(pl.fn)}, cy.fa, pl.fn)
		state, near = meet(state, s), near || nr
		if state == clr.DegNo {
			return junction{}, clr.DegNo, false
		}
		d := cy.c.sub(pl.o).dot(pl.n)
		lhs := new(big.Rat).Mul(d, d)
		rhs := new(big.Rat).Mul(new(big.Rat).Mul(cy.radius, cy.radius), pl.n.dot(pl.n))
		s, nr = gapState(lhs, rhs)
		state, near = meet(state, s), near || nr
		if state != clr.DegYes {
			return junction{}, state, near
		}
		// The generator through the axis's foot on the plane wall.
		g = cy.c.sub(pl.n.scale(new(big.Rat).Quo(d, pl.n.dot(pl.n))))
		dirr = cy.a
	default:
		s, nr := bandOf(t1.a.cross(t2.a), ratCross(t1.fa, t2.fa), t1.fa, t2.fa)
		state, near = meet(state, s), near || nr
		if state == clr.DegNo {
			return junction{}, clr.DegNo, false
		}
		dd := t2.c.sub(t1.c)
		aa := t1.a.dot(t1.a)
		perp := dd.sub(t1.a.scale(new(big.Rat).Quo(dd.dot(t1.a), aa)))
		sum := new(big.Rat).Add(t1.radius, t2.radius)
		diff := new(big.Rat).Sub(t1.radius, t2.radius)
		d2 := perp.dot(perp)
		ext, nrExt := gapState(d2, new(big.Rat).Mul(sum, sum))
		inn, nrInn := gapState(d2, new(big.Rat).Mul(diff, diff))
		switch {
		case ext == clr.DegYes:
			g = t1.c.add(perp.scale(new(big.Rat).Quo(t1.radius, sum)))
		case inn == clr.DegYes && diff.Sign() != 0:
			g = t1.c.add(perp.scale(new(big.Rat).Quo(t1.radius, diff)))
		default:
			near = near || nrExt || nrInn
			state = meet(state, clr.DegUnknown)
			if ext == clr.DegNo && inn == clr.DegNo {
				state = clr.DegNo
			}
			return junction{}, state, near
		}
		dirr = t1.a
	}
	if state != clr.DegYes {
		return junction{}, state, near
	}
	// Both walls' outward normals along g agree.
	n1, ok1 := chainNormal(t1, g)
	n2, ok2 := chainNormal(t2, g)
	if !ok1 || !ok2 {
		return junction{}, clr.DegUnknown, false
	}
	if n1.dot(n2).Sign() <= 0 {
		return junction{}, clr.DegNo, false
	}
	// V is the meeting of g with the base plane.
	den := dirr.dot(bt.n)
	if den.Sign() == 0 {
		return junction{}, clr.DegNo, false
	}
	k := new(big.Rat).Quo(bt.o.sub(g).dot(bt.n), den)
	point := g.add(dirr.scale(k)).float()
	if !proofbound.FiniteVec(point) {
		return junction{}, clr.DegUnknown, false
	}
	// The falsifier: the held vertex lies near V; a larger distance
	// disproves the pairing. A smaller one proves nothing.
	if v != nil {
		scale := max(1, math.Abs(point.X), math.Abs(point.Y), math.Abs(point.Z))
		if point.Sub(v.Position).Len() > v.Bound.Base()+64*ulp*scale {
			return junction{}, clr.DegUnknown, false
		}
	}
	return junction{point: point, base: base, wall1: w1, wall2: w2}, clr.DegYes, false
}

// ratCross is the float cross product, for the noise band.
func ratCross(a, b r3.Vec) r3.Vec { return a.Cross(b) }

// bandOf decides that a rational vector is zero: yes when it is exactly zero,
// no when its float form is clearly above the angular noise, undecided between.
func bandOf(exact ratVec, f, a, b r3.Vec) (clr.DegState, bool) {
	if exact.isZero() {
		return clr.DegYes, false
	}
	if f.Len() > clr.ClrAngTol*a.Len()*b.Len() {
		return clr.DegNo, false
	}
	return clr.DegUnknown, true
}

// gapState decides lhs == rhs: yes when equal, no when they differ by more than
// the noise band relative to rhs, undecided between.
func gapState(lhs, rhs *big.Rat) (clr.DegState, bool) {
	diff := new(big.Rat).Sub(lhs, rhs)
	if diff.Sign() == 0 {
		return clr.DegYes, false
	}
	rel := new(big.Rat).Quo(diff, rhs)
	f, _ := rel.Float64()
	if math.Abs(f) > 4*clr.ClrAngTol {
		return clr.DegNo, false
	}
	return clr.DegUnknown, true
}

// meet is three-valued conjunction.
func meet(x, y clr.DegState) clr.DegState {
	switch {
	case x == clr.DegNo || y == clr.DegNo:
		return clr.DegNo
	case x == clr.DegYes && y == clr.DegYes:
		return clr.DegYes
	}
	return clr.DegUnknown
}

// chainNormal is the wall's outward normal at the point p of its tangent
// generator, before normalisation.
func chainNormal(t chainWall, p ratVec) (ratVec, bool) {
	var n ratVec
	if t.isPlane {
		n = t.n
	} else {
		rel := p.sub(t.c)
		aa := t.a.dot(t.a)
		n = rel.sub(t.a.scale(new(big.Rat).Quo(rel.dot(t.a), aa)))
	}
	if t.reversed {
		for i := range n {
			n[i] = new(big.Rat).Neg(n[i])
		}
	}
	return n, true
}
