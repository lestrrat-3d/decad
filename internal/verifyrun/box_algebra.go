package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file is the box algebra separator
// (agent-docs/interference-design.md §3.2, "The box algebra separator"). Two
// Cut chains A = A0 less the tools T1 and B = B0 less the tools T2 miss each
// other when A0 ∩ B0 lies in the union of every tool: a point of both
// interiors lies in the interior of A0 and of B0 and outside the closed
// tools, so it cannot lie in the union. Each target part is bounded by a box
// in one reference prism's coordinates, and each tool must be exactly a box
// there. A treadle and the heel rail lapped into it, each cut where the other
// stays, are decided this way.

// boxApart reports whether a's and b's Cut chains miss each other by the box
// algebra.
func boxApart(ctx context.Context, budget *proofbound.WorkBudget, history boolchain.Source, a, b *topo.Body) (bool, error) {
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
	ref := partsA[0]
	var tools []box
	for _, tool := range append(toolsA, toolsB...) {
		for _, part := range unionParts(history, tool, 0) {
			pt, ok := part.Payload.(prismrecord.Payload)
			if !ok || !prismExact(pt) {
				continue
			}
			if bx, ok := exactBox(ref, pt); ok {
				tools = append(tools, bx)
			}
		}
	}
	for _, pa := range partsA {
		boxA, ok := boundingBox(ref, pa)
		if !ok {
			return false, nil
		}
		for _, pb := range partsB {
			boxB, ok := boundingBox(ref, pb)
			if !ok {
				return false, nil
			}
			overlap, ok := boxA.intersect(boxB)
			if ok && !overlap.coveredBy(tools) {
				return false, ctx.Err()
			}
			if err := budget.Step(); err != nil {
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

// box is a closed axis-aligned box in a reference prism's coordinates: its
// section axes and its sweep.
type box struct{ lo, hi [3]*big.Rat }

// intersect is the box two boxes share and whether it has volume.
func (b box) intersect(o box) (box, bool) {
	var out box
	for k := range 3 {
		out.lo[k], out.hi[k] = maxRat(b.lo[k], o.lo[k]), minRat(b.hi[k], o.hi[k])
		if out.lo[k].Cmp(out.hi[k]) >= 0 {
			return box{}, false
		}
	}
	return out, true
}

// coveredBy reports whether every point of b lies in some box of tools, by the
// cells their coordinates cut b into: each cell lies in one tool or in none.
func (b box) coveredBy(tools []box) bool {
	var cuts [3][]*big.Rat
	for k := range 3 {
		cuts[k] = []*big.Rat{b.lo[k], b.hi[k]}
		for _, t := range tools {
			for _, c := range []*big.Rat{t.lo[k], t.hi[k]} {
				if c.Cmp(b.lo[k]) > 0 && c.Cmp(b.hi[k]) < 0 {
					cuts[k] = append(cuts[k], c)
				}
			}
		}
		sortRats(cuts[k])
	}
	for i := 0; i+1 < len(cuts[0]); i++ {
		for j := 0; j+1 < len(cuts[1]); j++ {
			for l := 0; l+1 < len(cuts[2]); l++ {
				cell := box{
					lo: [3]*big.Rat{cuts[0][i], cuts[1][j], cuts[2][l]},
					hi: [3]*big.Rat{cuts[0][i+1], cuts[1][j+1], cuts[2][l+1]},
				}
				if cell.lo[0].Cmp(cell.hi[0]) == 0 || cell.lo[1].Cmp(cell.hi[1]) == 0 || cell.lo[2].Cmp(cell.hi[2]) == 0 {
					continue
				}
				if !cell.inSome(tools) {
					return false
				}
			}
		}
	}
	return true
}

func (b box) inSome(tools []box) bool {
	for _, t := range tools {
		in := true
		for k := range 3 {
			in = in && t.lo[k].Cmp(b.lo[k]) <= 0 && t.hi[k].Cmp(b.hi[k]) >= 0
		}
		if in {
			return true
		}
	}
	return false
}

func sortRats(v []*big.Rat) {
	for i := 1; i < len(v); i++ {
		for j := i; j > 0 && v[j].Cmp(v[j-1]) < 0; j-- {
			v[j], v[j-1] = v[j-1], v[j]
		}
	}
}

// exactBox is boxExtent as a box.
func exactBox(ref, pt prismrecord.Payload) (box, bool) {
	lo, hi, ok := boxExtent(ref, pt)
	return box{lo: lo, hi: hi}, ok
}

// boundingBox is a box in ref's coordinates holding every point of pp: the box
// of pp's section outline over its levels, its corners carried to ref's
// coordinates, both frames compared in the coordinates before one placement
// they carry bit for bit or in the world when both lifts are exactly
// orthonormal.
func boundingBox(ref, pp prismrecord.Payload) (box, bool) {
	uLo, uHi, vLo, vHi, ok := sectionBox(pp.Profile)
	if !ok {
		return box{}, false
	}
	lr, tr := prismLift(ref)
	lp, tp := prismLift(pp)
	if ref.Xform == pp.Xform {
		var okR, okP bool
		lr, tr, okR = frameLift(ref.Frame, r3.Identity())
		lp, tp, okP = frameLift(pp.Frame, r3.Identity())
		if !okR || !okP {
			return box{}, false
		}
	} else if !orthonormal(lr[0], lr[1], lr[2]) || !orthonormal(lp[0], lp[1], lp[2]) {
		return box{}, false
	}
	z0, z1 := levels(pp, nil)
	var out box
	for _, u := range []*big.Rat{uLo, uHi} {
		for _, v := range []*big.Rat{vLo, vHi} {
			for _, z := range []*big.Rat{z0, z1} {
				corner := addVec(tp, addVec(addVec(scaleVec(lp[0], u), scaleVec(lp[1], v)), scaleVec(lp[2], z)))
				rel := subVec(corner, tr)
				for k := range 3 {
					c := dotVec(rel, lr[k])
					if out.lo[k] == nil {
						out.lo[k], out.hi[k] = c, c
						continue
					}
					out.lo[k], out.hi[k] = minRat(out.lo[k], c), maxRat(out.hi[k], c)
				}
			}
		}
	}
	return out, true
}
