package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is WithTangentChain's expansion (docs/modify-reach-design.md §5):
// a fixed-point walk from every seed edge across the edges that continue it
// with proven G1 continuity. Each continuation question goes to the
// clearance kernel's exact-float degeneracy oracle (internal/clearance's
// Oracle.ParallelExact) over unnormalised analytic directions: an exact zero
// cross product proves a direction pair parallel, a cross clearly above the
// kernel's angular noise disproves it, and the band between is undecided.
// A tolerance may prove "no" and never "yes". An undecided continuation is
// SX2, never a chain stop: stopping would under-blend the caller's rule.

// expandTangentChain expands seeds (stage 2 of the reach gate order, §4) and
// returns the expanded set in body edge order. A faceted receiver is SX9
// before any expansion runs: its curves and surfaces never enter the oracle.
func expandTangentChain(ctx context.Context, b *Body, sel EdgeSelector, seeds []*Edge) ([]*Edge, error) {
	if _, ok := b.payload.(facetedPayload); ok {
		return nil, fmt.Errorf(`%w: a faceted boolean result carries no analytic carrier to prove a tangent continuation over; this evaluator modifies no faceted body (modify-reach SX9)`, ErrUnsupported)
	}
	bodyEdges := b.Edges()
	incident := make(map[*Vertex][]*Edge)
	for _, e := range bodyEdges {
		if e.start != nil {
			incident[e.start] = append(incident[e.start], e)
		}
		if e.end != nil && e.end != e.start {
			incident[e.end] = append(incident[e.end], e)
		}
	}
	in := make(map[*Edge]struct{}, len(seeds))
	queue := make([]*Edge, 0, len(seeds))
	for _, e := range seeds {
		if _, ok := in[e]; ok {
			continue
		}
		in[e] = struct{}{}
		queue = append(queue, e)
	}
	for len(queue) > 0 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		e := queue[0]
		queue = queue[1:]
		// A closed curve has no endpoint corner; its seam vertex does not make
		// it a partial chain (§3's geometric edge).
		if e.start == nil || e.end == nil || e.start == e.end {
			continue
		}
		for _, v := range [2]*Vertex{e.start, e.end} {
			cands := incident[v]
			states := make([]clearance.DegState, len(cands))
			for i, c := range cands {
				if c == e {
					states[i] = clearance.DegNo
					continue
				}
				states[i] = tangentContinues(e, c, v)
			}
			next, err := chainStep(states)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, the chain through (%s)`, err, sel, renderVec(v.position))
			}
			if next < 0 {
				continue
			}
			c := cands[next]
			if _, ok := in[c]; ok {
				continue
			}
			in[c] = struct{}{}
			queue = append(queue, c)
		}
	}
	out := make([]*Edge, 0, len(in))
	for _, e := range bodyEdges {
		if _, ok := in[e]; ok {
			out = append(out, e)
		}
	}
	return out, nil
}

// chainStep is §5's per-endpoint outcome table over the candidates' oracle
// answers: no proven candidate and none undecided stops the chain (-1); one
// proven and none undecided continues through that candidate's index; more
// than one proven, or any undecided, is SX2. Candidate order breaks no tie.
func chainStep(states []clearance.DegState) (int, error) {
	proven, undecided, next := 0, 0, -1
	for i, s := range states {
		switch s {
		case clearance.DegYes:
			proven++
			next = i
		case clearance.DegUnknown:
			undecided++
		}
	}
	if proven > 1 || undecided > 0 {
		return -1, fmt.Errorf(`%w: the tangent chain branches or cannot be decided here: %d edges continue it and %d cannot be decided; this evaluator does not choose a branch or stop early (modify-reach SX2)`,
			ErrUnsupported, proven, undecided)
	}
	return next, nil
}

// tangentContinues decides whether c continues e through their shared vertex
// v (§5 tests 1-5). Every "yes" is exact; a "no" may come from the oracle's
// noise floor or from an exact sign.
func tangentContinues(e, c *Edge, v *Vertex) clearance.DegState {
	if c.start == c.end || (c.start != v && c.end != v) {
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
	if len(e.faces) != 2 || len(c.faces) != 2 {
		return clearance.DegUnknown
	}
	a, bb := e.faces[0], e.faces[1]
	cc, dd := c.faces[0], c.faces[1]
	p := v.position
	straight := degAnd(sameOutwardNormal(a, cc, p), sameOutwardNormal(bb, dd, p))
	crossed := degAnd(sameOutwardNormal(a, dd, p), sameOutwardNormal(bb, cc, p))
	return degOr(straight, crossed)
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
func rayAway(e *Edge, v *Vertex) (chainDir, bool) {
	if e.start == nil || e.end == nil {
		return chainDir{}, false
	}
	other := e.end
	if v == e.end {
		other = e.start
	}
	switch c := e.curve.(type) {
	case Line3:
		if !proofbound.FiniteVec(v.position) || !proofbound.FiniteVec(other.position) {
			return chainDir{}, false
		}
		return chainDir{
			exact: proofarith.DvSub(proofarith.DyVec(other.position), proofarith.DyVec(v.position)),
			f:     other.position.Sub(v.position),
			held:  true,
		}, true
	case Arc3:
		if !proofbound.FiniteVec(v.position) || !proofbound.FiniteVec(c.Center) || !proofbound.FiniteVec(c.Axis) {
			return chainDir{}, false
		}
		exact := proofarith.DvCross(proofarith.DyVec(c.Axis), proofarith.DvSub(proofarith.DyVec(v.position), proofarith.DyVec(c.Center)))
		f := c.Axis.Cross(v.position.Sub(c.Center))
		if v == e.end && v != e.start {
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
func sameOutwardNormal(f, g *Face, p r3.Vec) clearance.DegState {
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
func outwardNumerator(f *Face, p r3.Vec) (chainDir, bool) {
	var n chainDir
	var ok bool
	switch s := f.surface.(type) {
	case Plane:
		n, ok = newChainDir(s.Frame.N())
	case Cylinder:
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
	case Sphere:
		if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(s.Center) {
			return chainDir{}, false
		}
		n, ok = chainDir{
			exact: proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(s.Center)),
			f:     p.Sub(s.Center),
			held:  true,
		}, true
	case Cone, Torus:
		m, err := f.NormalAt(p)
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
	if f.reversed {
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
