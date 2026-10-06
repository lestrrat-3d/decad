package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is the clearance kernel's degeneracy oracle — the one place every
// cell asks whether a configuration is degenerate. Every answer is a
// clearance.DegState, whose doc comment states the discipline that keeps a
// certificate honest (docs/clearance-design.md §4/§5).

// parallelExact decides a ∥ b from the vectors taken exactly (ra, rb) with their
// float forms (fa, fb) supplying the disproof threshold: an exactly zero cross
// product proves parallelism outright, a cross clearly above the kernel's
// angular noise disproves it, and the band between is undecided.
func (k *pairKernel) parallelExact(ra, rb proofarith.DyV3, fa, fb r3.Vec) clearance.DegState {
	la, lb := fa.Len(), fb.Len()
	if !proofbound.FiniteVec(fa) || !proofbound.FiniteVec(fb) || la == 0 || lb == 0 {
		return clearance.DegUnknown
	}
	if proofarith.DvIsZero(proofarith.DvCross(ra, rb)) {
		return clearance.DegYes
	}
	if fa.Cross(fb).Len() > clearance.ClrAngTol*la*lb {
		return clearance.DegNo
	}
	return clearance.DegUnknown
}

// parallel decides a ∥ b.
func (k *pairKernel) parallel(a, b r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) {
		return clearance.DegUnknown
	}
	return k.parallelExact(proofarith.DyVec(a), proofarith.DyVec(b), a, b)
}

// parallelSeg decides (b − a) ∥ d, with the difference taken exactly (a float
// subtraction of the endpoints would round away the very residual the proof
// rests on).
func (k *pairKernel) parallelSeg(a, b, d r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(d) {
		return clearance.DegUnknown
	}
	return k.parallelExact(proofarith.DvSub(proofarith.DyVec(b), proofarith.DyVec(a)), proofarith.DyVec(d), b.Sub(a), d)
}

// parallelSegs decides (b1 − a1) ∥ (b2 − a2).
func (k *pairKernel) parallelSegs(a1, b1, a2, b2 r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(a1) || !proofbound.FiniteVec(b1) || !proofbound.FiniteVec(a2) || !proofbound.FiniteVec(b2) {
		return clearance.DegUnknown
	}
	return k.parallelExact(proofarith.DvSub(proofarith.DyVec(b1), proofarith.DyVec(a1)), proofarith.DvSub(proofarith.DyVec(b2), proofarith.DyVec(a2)),
		b1.Sub(a1), b2.Sub(a2))
}

// perpendicularSeg decides (b − a) ⟂ n — the plane-plateau question of the
// curve tiers, taken exactly.
func (k *pairKernel) perpendicularSeg(a, b, n r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(n) {
		return clearance.DegUnknown
	}
	fRel := b.Sub(a)
	lr, ln := fRel.Len(), n.Len()
	if !proofbound.FiniteVec(fRel) || !proofbound.FiniteVec(n) || lr == 0 || ln == 0 {
		return clearance.DegUnknown
	}
	rel := proofarith.DvSub(proofarith.DyVec(b), proofarith.DyVec(a))
	if proofarith.DvDot(rel, proofarith.DyVec(n)).Sign() == 0 {
		return clearance.DegYes
	}
	if math.Abs(fRel.Dot(n)) > clearance.ClrAngTol*lr*ln {
		return clearance.DegNo
	}
	return clearance.DegUnknown
}

// onAxis decides whether p lies on the line (anchor, unit axis): the offset is
// EXACTLY zero, provenly beyond the kernel's length noise, or undecided. Every
// cell that needs a radial direction off that offset asks here first — an
// offset in the undecided band normalizes to a garbage direction, and a garbage
// direction decides a trim admission.
func (k *pairKernel) onAxis(p, anchor, axis r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(anchor) || !proofbound.FiniteVec(axis) {
		return clearance.DegUnknown
	}
	rel := proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(anchor))
	if proofarith.DvIsZero(rel) || proofarith.DvIsZero(proofarith.DvCross(rel, proofarith.DyVec(axis))) {
		return clearance.DegYes
	}
	fRel := p.Sub(anchor)
	if fRel.Sub(axis.Scale(fRel.Dot(axis))).Len() > k.tol {
		return clearance.DegNo
	}
	return clearance.DegUnknown
}

// coincident decides whether two points are the same point.
func (k *pairKernel) coincident(p, q r3.Vec) clearance.DegState {
	if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(q) {
		return clearance.DegUnknown
	}
	if proofarith.DvIsZero(proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(q))) {
		return clearance.DegYes
	}
	if p.Sub(q).Len() > k.tol {
		return clearance.DegNo
	}
	return clearance.DegUnknown
}

// coaxial decides whether two axis-symmetric carriers share an axis LINE.
func (k *pairKernel) coaxial(f, g *clearance.CFace) clearance.DegState {
	return clearance.DegAnd(k.parallel(f.Axis, g.Axis), k.onAxis(g.Anchor, f.Anchor, f.Axis))
}

// ringFamily decides §4's coincident-spine configurations — the ones whose
// annular gap really is attained all the way around a ring: concentric point
// spines, a point spine ON a line spine, and coaxial line spines (the
// peg-in-hole reading). A circle spine is never in the family, and a
// near-coincidence the oracle cannot decide is not either.
func (k *pairKernel) ringFamily(f, g *clearance.CFace) clearance.DegState {
	switch sf, sg := clearance.SpineOf(f), clearance.SpineOf(g); {
	case sf == 0 && sg == 0:
		return k.coincident(f.Anchor, g.Anchor)
	case sf == 0 && sg == 1:
		return k.onAxis(f.Anchor, g.Anchor, g.Axis)
	case sf == 1 && sg == 0:
		return k.onAxis(g.Anchor, f.Anchor, f.Axis)
	case sf == 1 && sg == 1:
		return k.coaxial(f, g)
	default:
		return clearance.DegNo
	}
}

// spineSup is §4's `d_sup` for the nested branch: the SUPREMUM over f's spine
// of the distance to g's spine — never a minimum, and finite only where the
// supremum is PROVEN finite.
//
// This is the boundedness gate the containment certificate turns on. A
// cylinder's spine is an infinite LINE: its supremum against any bounded
// partner is +∞, so no containment claim about a cylinder's carrier can ever
// rest on a foot distance. Only the doc's certified list keeps it finite — an
// inner POINT spine (the supremum over one point is that point's distance),
// exactly parallel line spines, and exactly coaxial spines, all of which have a
// constant spine distance. ok is false for everything else: unbounded,
// non-constant, or merely undecided.
func (k *pairKernel) spineSup(f, g *clearance.CFace) (float64, bool) {
	switch clearance.SpineOf(f) {
	case 0:
		return clearance.PointSpineDist(f.Anchor, g), true
	case 1:
		// An infinite line: bounded only against an exactly parallel line.
		if clearance.SpineOf(g) != 1 || k.parallel(f.Axis, g.Axis) != clearance.DegYes {
			return math.Inf(1), false
		}
		return clearance.PointSpineDist(f.Anchor, g), true
	default:
		// A circle spine is bounded, but a constant supremum needs exact
		// coaxiality — a non-coaxial circle inside another curved carrier has
		// no constant supremum and never borrows this branch (§4).
		switch clearance.SpineOf(g) {
		case 1:
			if k.coaxial(g, f) != clearance.DegYes {
				return math.Inf(1), false
			}
			return f.Major, true // every spine point sits f.major off the shared axis
		case 2:
			if k.coaxial(f, g) != clearance.DegYes {
				return math.Inf(1), false
			}
			rel := f.Anchor.Sub(g.Anchor)
			return math.Hypot(rel.Dot(g.Axis), f.Major-g.Major), true
		default:
			return math.Inf(1), false
		}
	}
}

// certifiedContainment is §4's nested branch, restricted to the doc's certified
// list through spineSup: f's carrier is strictly inside g's when the supremum of
// f's spine over g's spine, grown by f's offset, still clears g's offset —
// strictly, since equality is an internal tangency and routes to §6. Any
// configuration whose supremum is not proven finite never borrows the branch,
// which is what keeps a cylinder crossing a ball from certifying as nested.
func (k *pairKernel) certifiedContainment(f, g *clearance.CFace) bool {
	rf, rg := clearance.SpineOffset(f), clearance.SpineOffset(g)
	if sup, ok := k.spineSup(f, g); ok && rg > sup+rf+k.tol {
		return true
	}
	sup, ok := k.spineSup(g, f)
	return ok && rf > sup+rg+k.tol
}
