package tangentchain

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Continues decides whether c continues e through their shared vertex
// v (§5 tests 1-5). Every "yes" is exact; a "no" may come from the oracle's
// noise floor or from an exact sign.
func Continues(e, c *topo.Edge, v *topo.Vertex) clearance.DegState {
	if c.Start == c.End || (c.Start != v && c.End != v) {
		return clearance.DegNo
	}
	te, okE := rayAway(e, v)
	tc, okC := rayAway(c, v)
	if !okE || !okC {
		return clearance.DegUnknown
	}
	var oracle clearance.Oracle
	switch oracle.ParallelExact(te.exact, tc.exact, te.f, tc.f) {
	case clearance.DegNo:
		return clearance.DegNo
	case clearance.DegUnknown:
		// An exactly non-negative dot product disproves opposite rays even
		// where the cross product sits in the oracle's undecided band.
		if proofarith.DvDot(te.exact, tc.exact).Sign() >= 0 {
			return clearance.DegNo
		}
		return clearance.DegUnknown
	}
	if proofarith.DvDot(te.exact, tc.exact).Sign() >= 0 {
		return clearance.DegNo
	}
	if len(e.Faces) != 2 || len(c.Faces) != 2 {
		return clearance.DegUnknown
	}
	a, bb := e.Faces[0], e.Faces[1]
	cc, dd := c.Faces[0], c.Faces[1]
	p := v.Position
	straight := degAnd(sameOutwardNormal(a, cc, p), sameOutwardNormal(bb, dd, p))
	crossed := degAnd(sameOutwardNormal(a, dd, p), sameOutwardNormal(bb, cc, p))
	return degOr(straight, crossed)
}

// Apart reports whether the rays of e and c away from their shared vertex v are proven not opposite (§5 test 1):
// the oracle reads them as not parallel beyond its noise band, or their exact dot product is non-negative. Then c
// does not continue e whatever its faces are. A curve the oracle does not read is never apart.
func Apart(e, c *topo.Edge, v *topo.Vertex) bool {
	if c.Start == c.End || (c.Start != v && c.End != v) {
		return false
	}
	te, okE := rayAway(e, v)
	tc, okC := rayAway(c, v)
	if !okE || !okC {
		return false
	}
	var oracle clearance.Oracle
	if oracle.ParallelExact(te.exact, tc.exact, te.f, tc.f) == clearance.DegNo {
		return true
	}
	return proofarith.DvDot(te.exact, tc.exact).Sign() >= 0
}

// HeldMesh is a faceted body's held mesh: vertices with their bounds, triangles, and the index of each triangle's face.
type HeldMesh struct {
	Verts  []r3.Vec
	Bounds []float64
	Tris   [][3]int
	FaceOf []int
}

// heldApartSine is the least sine, beyond a chord's own uncertainty, at which a held chord's direction is read as
// apart from a ray.
const heldApartSine = 1e-3

// HeldApart is Apart for a candidate c whose curve is a Boolean's chord chain, which carries no direction: its ray at
// v is read from the held mesh as the chord from v that a facet of each of c's two faces holds, whose face indices are
// fa and fb. Every such chord must be proven apart from e's ray: its angle to the reverse of that ray exceeds
// heldApartSine plus the chord's uncertainty, the angle whose sine is 4(δ + β_v + β_w)/L for the chain's bound δ, the
// two vertex bounds and the chord's length L. A curve whose points lie within that much of the chord turns from it at
// v by less than that angle. It reports false when the mesh holds no such chord.
func HeldApart(m HeldMesh, fa, fb int, chainBound float64, e *topo.Edge, v *topo.Vertex) bool {
	te, ok := rayAway(e, v)
	if !ok || len(m.Bounds) != len(m.Verts) || len(m.FaceOf) != len(m.Tris) {
		return false
	}
	hv := -1
	for i, p := range m.Verts {
		if p == v.Position {
			hv = i
			break
		}
	}
	if hv < 0 {
		return false
	}
	// The chord's far ends: vertices that share a triangle with hv on face fa and another on face fb.
	onA, onB := map[int]struct{}{}, map[int]struct{}{}
	for ti, t := range m.Tris {
		if t[0] != hv && t[1] != hv && t[2] != hv {
			continue
		}
		for _, w := range t {
			if w == hv {
				continue
			}
			switch m.FaceOf[ti] {
			case fa:
				onA[w] = struct{}{}
			case fb:
				onB[w] = struct{}{}
			}
		}
	}
	back, ok := te.f.Scale(-1).Normalize()
	if !ok {
		return false
	}
	found := false
	for w := range onA {
		if _, both := onB[w]; !both {
			continue
		}
		found = true
		chord := m.Verts[w].Sub(v.Position)
		l := chord.Len()
		if !(l > 0) {
			return false
		}
		u := 4 * (chainBound + m.Bounds[hv] + m.Bounds[w]) / l
		if u >= 1 {
			return false
		}
		limit := math.Asin(u) + math.Asin(heldApartSine)
		angle := math.Acos(max(-1, min(1, back.Dot(chord.Scale(1/l)))))
		if angle <= limit {
			return false
		}
	}
	return found
}

// degAnd is three-valued conjunction: no when either is no, yes when both
// are yes, undecided otherwise.
func degAnd(x, y clearance.DegState) clearance.DegState {
	switch {
	case x == clearance.DegNo || y == clearance.DegNo:
		return clearance.DegNo
	case x == clearance.DegYes && y == clearance.DegYes:
		return clearance.DegYes
	default:
		return clearance.DegUnknown
	}
}

// degOr is three-valued disjunction: yes when either is yes, no when both
// are no, undecided otherwise.
func degOr(x, y clearance.DegState) clearance.DegState {
	switch {
	case x == clearance.DegYes || y == clearance.DegYes:
		return clearance.DegYes
	case x == clearance.DegNo && y == clearance.DegNo:
		return clearance.DegNo
	default:
		return clearance.DegUnknown
	}
}

// chainDir is one unnormalised analytic direction: exact is its value over
// dyadic rationals, f its float form, and held whether exact is the true
// direction rather than a float stand-in (a cone or torus normal needs a
// square root or a trigonometric value, so its exact form is only the float
// one read exactly and can never prove "yes").
type chainDir struct {
	exact proofarith.DyV3
	f     r3.Vec
	held  bool
}

func newChainDir(f r3.Vec) (chainDir, bool) {
	if !proofbound.FiniteVec(f) {
		return chainDir{}, false
	}
	return chainDir{exact: proofarith.DyVec(f), f: f, held: true}, true
}

// rayAway is the tangent ray of e at its endpoint v pointing away from v
// (§5): a line's endpoint difference, and for an arc axis × (v − centre),
// reversed at the arc's end vertex since an Arc3 sweeps counter-clockwise
// about its axis from start to end. Any other curve is not an analytic
// variant the oracle reads.
func rayAway(e *topo.Edge, v *topo.Vertex) (chainDir, bool) {
	if e.Start == nil || e.End == nil {
		return chainDir{}, false
	}
	other := e.End
	if v == e.End {
		other = e.Start
	}
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		if !proofbound.FiniteVec(v.Position) || !proofbound.FiniteVec(other.Position) {
			return chainDir{}, false
		}
		return chainDir{
			exact: proofarith.DvSub(proofarith.DyVec(other.Position), proofarith.DyVec(v.Position)),
			f:     other.Position.Sub(v.Position),
			held:  true,
		}, true
	case surfacegeom.Arc3:
		if !proofbound.FiniteVec(v.Position) || !proofbound.FiniteVec(c.Center) || !proofbound.FiniteVec(c.Axis) {
			return chainDir{}, false
		}
		exact := proofarith.DvCross(proofarith.DyVec(c.Axis), proofarith.DvSub(proofarith.DyVec(v.Position), proofarith.DyVec(c.Center)))
		f := c.Axis.Cross(v.Position.Sub(c.Center))
		if v == e.End && v != e.Start {
			exact = dvNeg(exact)
			f = f.Scale(-1)
		}
		return chainDir{exact: exact, f: f, held: true}, true
	default:
		return chainDir{}, false
	}
}

// sameOutwardNormal decides whether faces f and g have the same outward
// normal at p (§5 test 4): one face is its own match, and otherwise the two
// carrier normals must be exactly parallel with a positive dot product.
func sameOutwardNormal(f, g *topo.Face, p r3.Vec) clearance.DegState {
	if f == g {
		return clearance.DegYes
	}
	nf, okF := outwardNumerator(f, p)
	ng, okG := outwardNumerator(g, p)
	if !okF || !okG {
		return clearance.DegUnknown
	}
	var oracle clearance.Oracle
	par := oracle.ParallelExact(nf.exact, ng.exact, nf.f, ng.f)
	if par == clearance.DegNo {
		return clearance.DegNo
	}
	if nf.held && ng.held {
		if proofarith.DvDot(nf.exact, ng.exact).Sign() <= 0 {
			return clearance.DegNo
		}
		return par
	}
	// A float stand-in proves only a clear "no": an antiparallel pair.
	if nf.f.Dot(ng.f) < -0.5*nf.f.Len()*ng.f.Len() {
		return clearance.DegNo
	}
	return clearance.DegUnknown
}

// outwardNumerator is the face's outward normal at p before normalisation
// (§5): a plane's frame normal, a cylinder's radial offset of p from its
// axis, a sphere's offset of p from its centre — each exact over the held
// floats — and a cone's or torus's normal as a float stand-in. The face's
// own reversed flag turns the carrier normal outward. Any other surface is
// not an analytic variant the oracle reads.
func outwardNumerator(f *topo.Face, p r3.Vec) (chainDir, bool) {
	var n chainDir
	var ok bool
	switch s := f.Surface.(type) {
	case surfacegeom.Plane:
		n, ok = newChainDir(s.Frame.N())
	case surfacegeom.Cylinder:
		if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(s.Origin) || !proofbound.FiniteVec(s.Axis) {
			return chainDir{}, false
		}
		// rel·|a|² − a·(rel·a) is the radial offset scaled by |a|² > 0, so it
		// holds the direction exactly even where the held axis is not of
		// exactly unit length.
		rel := proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(s.Origin))
		axis := proofarith.DyVec(s.Axis)
		exact := proofarith.DvSub(dvScale(rel, proofarith.DvDot(axis, axis)), dvScale(axis, proofarith.DvDot(rel, axis)))
		relF := p.Sub(s.Origin)
		n, ok = chainDir{exact: exact, f: relF.Sub(s.Axis.Scale(relF.Dot(s.Axis))), held: true}, true
	case surfacegeom.Sphere:
		if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(s.Center) {
			return chainDir{}, false
		}
		n, ok = chainDir{
			exact: proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(s.Center)),
			f:     p.Sub(s.Center),
			held:  true,
		}, true
	case surfacegeom.Cone, surfacegeom.Torus:
		m, err := topobuild.NormalAt(f, p)
		if err != nil {
			return chainDir{}, false
		}
		// NormalAt already applied the reversed flag.
		n, ok = newChainDir(m.Value)
		n.held = false
		return n, ok
	default:
		return chainDir{}, false
	}
	if !ok {
		return chainDir{}, false
	}
	if f.Reversed {
		n.exact = dvNeg(n.exact)
		n.f = n.f.Scale(-1)
	}
	if n.f.Len() == 0 || math.IsNaN(n.f.Len()) {
		return chainDir{}, false
	}
	return n, true
}

func dvNeg(a proofarith.DyV3) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyNeg(a[0]), proofarith.DyNeg(a[1]), proofarith.DyNeg(a[2])}
}

func dvScale(a proofarith.DyV3, s proofarith.Dyadic) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyMul(a[0], s), proofarith.DyMul(a[1], s), proofarith.DyMul(a[2], s)}
}
