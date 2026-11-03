package verifyrun

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the box algebra separator
// (agent-docs/interference-design.md §3.2, "The box algebra separator"). Two
// Cut chains A = A0 less the tools T1 and B = B0 less the tools T2 miss each
// other when A0 ∩ B0 lies in the union of every tool: a point of both
// interiors lies in the interior of A0 and of B0 and outside the closed
// tools, so it cannot lie in the union. Each target part is bounded by a box
// in its own coordinates and each tool must be exactly a box in its own; the
// boxes are convex polytopes in the world through their prisms' exact lifts.
// A treadle and the rails lapped into it, each cut where the other stays, are
// decided this way.

// boxApart reports whether a's and b's Cut chains miss each other by the box
// algebra.
func boxApart(budget *proofbound.WorkBudget, history boolchain.Source, a, b *topo.Body) (bool, error) {
	targetA, toolsA := cutChain(history, a)
	targetB, toolsB := cutChain(history, b)
	partsA, ok := exactPrismParts(history, targetA)
	if !ok {
		return false, nil
	}
	partsB, ok := exactPrismParts(history, targetB)
	if !ok {
		return false, nil
	}
	var tools []polytope
	for _, tool := range append(toolsA, toolsB...) {
		for _, part := range unionParts(history, tool, 0) {
			pt, ok := part.Payload.(prismrecord.Payload)
			if !ok || !prismExact(pt) {
				continue
			}
			if poly, ok := toolPolytope(pt); ok {
				tools = append(tools, poly)
			}
		}
	}
	for _, pa := range partsA {
		polyA, ok := boundingPolytope(pa)
		if !ok {
			return false, nil
		}
		for _, pb := range partsB {
			polyB, ok := boundingPolytope(pb)
			if !ok {
				return false, nil
			}
			covered, err := polyA.intersect(polyB).coveredBy(budget, tools)
			if err != nil || !covered {
				return false, err
			}
		}
	}
	return true, nil
}

// exactPrismParts lists the exact prisms body is: itself, or the operands of
// the Unions it was built from.
func exactPrismParts(history boolchain.Source, body *topo.Body) ([]prismrecord.Payload, bool) {
	var out []prismrecord.Payload
	for _, part := range unionParts(history, body, 0) {
		pp, ok := part.Payload.(prismrecord.Payload)
		if !ok || !prismExact(pp) {
			return nil, false
		}
		out = append(out, pp)
	}
	return out, len(out) > 0
}

// plane is the closed half-space n·x ≤ d.
type plane struct {
	n [3]*big.Rat
	d *big.Rat
}

// polytope is a convex solid held as the intersection of its half-spaces.
type polytope []plane

// polytopeOf is the box lo..hi of a prism's own coordinates, as the exact image
// of that box under the prism's lift: its lift L is an exact rational map, so
// the set is the intersection of the half-spaces e_k·L⁻¹(x − t) ≤ hi_k and
// ≥ lo_k, with L⁻¹ solved exactly. The lift need not be orthonormal.
func polytopeOf(pp prismrecord.Payload, lo, hi [3]*big.Rat) (polytope, bool) {
	l, t := prismLift(pp)
	var cols [3][3]*big.Rat
	for i := range 3 {
		cols[i] = [3]*big.Rat{l[0][i], l[1][i], l[2][i]}
	}
	var out polytope
	for k := range 3 {
		unit := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
		unit[k] = big.NewRat(1, 1)
		n, ok := solve(cols, unit)
		if !ok {
			return nil, false
		}
		shift := dotVec(n, t)
		out = append(out,
			plane{n: n, d: new(big.Rat).Add(hi[k], shift)},
			plane{n: scaleVec(n, big.NewRat(-1, 1)), d: new(big.Rat).Sub(new(big.Rat).Neg(lo[k]), shift)})
	}
	return out, true
}

// toolPolytope is a cutting prism that is one axis-aligned rectangle of whole
// lines over its levels, in the lift's world.
func toolPolytope(pt prismrecord.Payload) (polytope, bool) {
	u0, v0, u1, v1, ok := axisRectangle(pt.Profile)
	if !ok {
		return nil, false
	}
	z0, z1 := levels(pt, nil)
	return polytopeOf(pt, [3]*big.Rat{u0, v0, z0}, [3]*big.Rat{u1, v1, z1})
}

// boundingPolytope is a polytope holding every point of pp: the box of its
// section outline over its levels, in its own coordinates.
func boundingPolytope(pp prismrecord.Payload) (polytope, bool) {
	uLo, uHi, vLo, vHi, ok := sectionBox(pp.Profile)
	if !ok {
		return nil, false
	}
	z0, z1 := levels(pp, nil)
	return polytopeOf(pp, [3]*big.Rat{uLo, vLo, z0}, [3]*big.Rat{uHi, vHi, z1})
}

func (p polytope) intersect(o polytope) polytope { return append(slices.Clone(p), o...) }

// vertices lists the polytope's vertices: the points where three planes meet
// that lie in every half-space.
func (p polytope) vertices() [][3]*big.Rat {
	var out [][3]*big.Rat
	for i := range p {
		for j := i + 1; j < len(p); j++ {
			for k := j + 1; k < len(p); k++ {
				cols := [3][3]*big.Rat{
					{p[i].n[0], p[j].n[0], p[k].n[0]},
					{p[i].n[1], p[j].n[1], p[k].n[1]},
					{p[i].n[2], p[j].n[2], p[k].n[2]},
				}
				x, ok := solve(cols, [3]*big.Rat{p[i].d, p[j].d, p[k].d})
				if !ok || !p.contains(x) || hasPoint(out, x) {
					continue
				}
				out = append(out, x)
			}
		}
	}
	return out
}

func (p polytope) contains(x [3]*big.Rat) bool {
	for _, h := range p {
		if dotVec(h.n, x).Cmp(h.d) > 0 {
			return false
		}
	}
	return true
}

func hasPoint(pts [][3]*big.Rat, x [3]*big.Rat) bool {
	for _, q := range pts {
		if q[0].Cmp(x[0]) == 0 && q[1].Cmp(x[1]) == 0 && q[2].Cmp(x[2]) == 0 {
			return true
		}
	}
	return false
}

// hasVolume reports whether the polytope holds a ball: its vertices do not lie
// in one plane.
func (p polytope) hasVolume() bool {
	v := p.vertices()
	if len(v) < 4 {
		return false
	}
	for i := 3; i < len(v); i++ {
		d := dotVec(subVec(v[i], v[0]), crossVec(subVec(v[1], v[0]), subVec(v[2], v[0])))
		if d.Sign() != 0 {
			return true
		}
	}
	// The first three may be collinear; look for any non-coplanar quadruple.
	for a := range v {
		for b := a + 1; b < len(v); b++ {
			for c := b + 1; c < len(v); c++ {
				n := crossVec(subVec(v[b], v[a]), subVec(v[c], v[a]))
				if isZero(n) {
					continue
				}
				for d := c + 1; d < len(v); d++ {
					if dotVec(subVec(v[d], v[a]), n).Sign() != 0 {
						return true
					}
				}
			}
		}
	}
	return false
}

// coveredBy reports whether every point of p lies in some tool: p is split by
// the first tool's planes into that tool's part and the pieces outside it, each
// piece then covered by the remaining tools. A piece without volume is covered.
// The work is charged to budget and ends in a refusal when it runs out.
func (p polytope) coveredBy(budget *proofbound.WorkBudget, tools []polytope) (bool, error) {
	if err := budget.Step(); err != nil {
		return false, err
	}
	if !p.hasVolume() {
		return true, nil
	}
	if len(tools) == 0 {
		return false, nil
	}
	tool, rest := tools[0], tools[1:]
	inside := slices.Clone(p)
	for _, h := range tool {
		// The piece beyond this plane, within the planes before it.
		outside := append(slices.Clone(inside), plane{n: scaleVec(h.n, big.NewRat(-1, 1)), d: new(big.Rat).Neg(h.d)})
		covered, err := outside.coveredBy(budget, rest)
		if err != nil || !covered {
			return false, err
		}
		inside = append(inside, h)
	}
	return true, nil
}
