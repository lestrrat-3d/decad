package clearance

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Oracle decides whether clearance carrier configurations are degenerate.
// Tol is the pair's length tolerance and can only disprove a degeneracy.
type Oracle struct{ Tol float64 }

// This file is the clearance kernel's degeneracy oracle — the one place every
// cell asks whether a configuration is degenerate. Every answer is a
// DegState, whose doc comment states the discipline that keeps a
// certificate honest (docs/clearance-design.md §4/§5).

// ParallelExact decides a ∥ b from the vectors taken exactly (ra, rb) with their
// float forms (fa, fb) supplying the disproof threshold: an exactly zero cross
// product proves parallelism outright, a cross clearly above the kernel's
// angular noise disproves it, and the band between is undecided.
func (o Oracle) ParallelExact(ra, rb proofarith.DyV3, fa, fb r3.Vec) DegState {
	la, lb := fa.Len(), fb.Len()
	if !proofbound.FiniteVec(fa) || !proofbound.FiniteVec(fb) || la == 0 || lb == 0 {
		return DegUnknown
	}
	if proofarith.DvIsZero(proofarith.DvCross(ra, rb)) {
		return DegYes
	}
	if fa.Cross(fb).Len() > ClrAngTol*la*lb {
		return DegNo
	}
	return DegUnknown
}

// Parallel decides a ∥ b.
func (o Oracle) Parallel(a, b r3.Vec) DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) {
		return DegUnknown
	}
	return o.ParallelExact(proofarith.DyVec(a), proofarith.DyVec(b), a, b)
}

// ParallelSeg decides (b − a) ∥ d, with the difference taken exactly (a float
// subtraction of the endpoints would round away the very residual the proof
// rests on).
func (o Oracle) ParallelSeg(a, b, d r3.Vec) DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(d) {
		return DegUnknown
	}
	return o.ParallelExact(proofarith.DvSub(proofarith.DyVec(b), proofarith.DyVec(a)), proofarith.DyVec(d), b.Sub(a), d)
}

// ParallelSegs decides (b1 − a1) ∥ (b2 − a2).
func (o Oracle) ParallelSegs(a1, b1, a2, b2 r3.Vec) DegState {
	if !proofbound.FiniteVec(a1) || !proofbound.FiniteVec(b1) || !proofbound.FiniteVec(a2) || !proofbound.FiniteVec(b2) {
		return DegUnknown
	}
	return o.ParallelExact(proofarith.DvSub(proofarith.DyVec(b1), proofarith.DyVec(a1)), proofarith.DvSub(proofarith.DyVec(b2), proofarith.DyVec(a2)),
		b1.Sub(a1), b2.Sub(a2))
}

// PerpendicularSeg decides (b − a) ⟂ n — the plane-plateau question of the
// curve tiers, taken exactly.
func (o Oracle) PerpendicularSeg(a, b, n r3.Vec) DegState {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(n) {
		return DegUnknown
	}
	fRel := b.Sub(a)
	lr, ln := fRel.Len(), n.Len()
	if !proofbound.FiniteVec(fRel) || !proofbound.FiniteVec(n) || lr == 0 || ln == 0 {
		return DegUnknown
	}
	rel := proofarith.DvSub(proofarith.DyVec(b), proofarith.DyVec(a))
	if proofarith.DvDot(rel, proofarith.DyVec(n)).Sign() == 0 {
		return DegYes
	}
	if math.Abs(fRel.Dot(n)) > ClrAngTol*lr*ln {
		return DegNo
	}
	return DegUnknown
}

// OnAxis decides whether p lies on the line (anchor, unit axis): the offset is
// EXACTLY zero, provenly beyond the kernel's length noise, or undecided. Every
// cell that needs a radial direction off that offset asks here first — an
// offset in the undecided band normalizes to a garbage direction, and a garbage
// direction decides a trim admission.
func (o Oracle) OnAxis(p, anchor, axis r3.Vec) DegState {
	if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(anchor) || !proofbound.FiniteVec(axis) {
		return DegUnknown
	}
	rel := proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(anchor))
	if proofarith.DvIsZero(rel) || proofarith.DvIsZero(proofarith.DvCross(rel, proofarith.DyVec(axis))) {
		return DegYes
	}
	fRel := p.Sub(anchor)
	if fRel.Sub(axis.Scale(fRel.Dot(axis))).Len() > o.Tol {
		return DegNo
	}
	return DegUnknown
}

// Coincident decides whether two points are the same point.
func (o Oracle) Coincident(p, q r3.Vec) DegState {
	if !proofbound.FiniteVec(p) || !proofbound.FiniteVec(q) {
		return DegUnknown
	}
	if proofarith.DvIsZero(proofarith.DvSub(proofarith.DyVec(p), proofarith.DyVec(q))) {
		return DegYes
	}
	if p.Sub(q).Len() > o.Tol {
		return DegNo
	}
	return DegUnknown
}

// Coaxial decides whether two axis-symmetric carriers share an axis LINE.
func (o Oracle) Coaxial(f, g *CFace) DegState {
	return DegAnd(o.Parallel(f.Axis, g.Axis), o.OnAxis(g.Anchor, f.Anchor, f.Axis))
}

// RingFamily decides §4's Coincident-spine configurations — the ones whose
// annular gap really is attained all the way around a ring: concentric point
// spines, a point spine ON a line spine, and Coaxial line spines (the
// peg-in-hole reading). A circle spine is never in the family, and a
// near-coincidence the oracle cannot decide is not either.
func (o Oracle) RingFamily(f, g *CFace) DegState {
	switch sf, sg := SpineOf(f), SpineOf(g); {
	case sf == 0 && sg == 0:
		return o.Coincident(f.Anchor, g.Anchor)
	case sf == 0 && sg == 1:
		return o.OnAxis(f.Anchor, g.Anchor, g.Axis)
	case sf == 1 && sg == 0:
		return o.OnAxis(g.Anchor, f.Anchor, f.Axis)
	case sf == 1 && sg == 1:
		return o.Coaxial(f, g)
	default:
		return DegNo
	}
}

// SpineSup is §4's `d_sup` for the nested branch: the SUPREMUM over f's spine
// of the distance to g's spine — never a minimum, and finite only where the
// supremum is PROVEN finite.
//
// This is the boundedness gate the containment certificate turns on. A
// cylinder's spine is an infinite LINE: its supremum against any bounded
// partner is +∞, so no containment claim about a cylinder's carrier can ever
// rest on a foot distance. Only the doc's certified list keeps it finite — an
// inner POINT spine (the supremum over one point is that point's distance),
// exactly Parallel line spines, and exactly Coaxial spines, all of which have a
// constant spine distance. ok is false for everything else: unbounded,
// non-constant, or merely undecided.
func (o Oracle) SpineSup(f, g *CFace) (float64, bool) {
	switch SpineOf(f) {
	case 0:
		return PointSpineDist(f.Anchor, g), true
	case 1:
		// An infinite line: bounded only against an exactly Parallel line.
		if SpineOf(g) != 1 || o.Parallel(f.Axis, g.Axis) != DegYes {
			return math.Inf(1), false
		}
		return PointSpineDist(f.Anchor, g), true
	default:
		// A circle spine is bounded, but a constant supremum needs exact
		// coaxiality — a non-Coaxial circle inside another curved carrier has
		// no constant supremum and never borrows this branch (§4).
		switch SpineOf(g) {
		case 1:
			if o.Coaxial(g, f) != DegYes {
				return math.Inf(1), false
			}
			return f.Major, true // every spine point sits f.major off the shared axis
		case 2:
			if o.Coaxial(f, g) != DegYes {
				return math.Inf(1), false
			}
			rel := f.Anchor.Sub(g.Anchor)
			return math.Hypot(rel.Dot(g.Axis), f.Major-g.Major), true
		default:
			return math.Inf(1), false
		}
	}
}

// CertifiedContainment is §4's nested branch, restricted to the doc's certified
// list through SpineSup: f's carrier is strictly inside g's when the supremum of
// f's spine over g's spine, grown by f's offset, still clears g's offset —
// strictly, since equality is an internal tangency and routes to §6. Any
// configuration whose supremum is not proven finite never borrows the branch,
// which is what keeps a cylinder crossing a ball from certifying as nested.
func (o Oracle) CertifiedContainment(f, g *CFace) bool {
	rf, rg := SpineOffset(f), SpineOffset(g)
	if sup, ok := o.SpineSup(f, g); ok && rg > sup+rf+o.Tol {
		return true
	}
	sup, ok := o.SpineSup(g, f)
	return ok && rf > sup+rg+o.Tol
}
