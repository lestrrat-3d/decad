package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the clearance kernel's degeneracy oracle — the one place every
// cell asks whether a configuration is degenerate, and the one discipline that
// keeps a certificate honest (docs/clearance-design.md §4/§5).
//
// A closed-form cell is exact only where the configuration it assumes actually
// holds: a plateau needs EXACT parallelism, a constant-distance critical needs
// EXACT coaxiality, the nested branch needs a supremum that is actually finite.
// Deciding any of those with a tolerance mints an Exact reading the true answer
// undercuts by up to the tolerance — a lie the report then certifies. So every
// such question routes through here and comes back THREE-valued:
//
//   - degYes — proven degenerate by EXACT arithmetic on the payload's own
//     floats (rational cross/dot products over math/big.Rat, the same
//     take-the-floats-exactly discipline as clearance_poly.go's Sturm
//     brackets). The closed form IS the answer, and the candidate is Exact.
//   - degNo — proven NOT degenerate, by a residual clearly above the kernel's
//     own noise. The general (non-degenerate) closed form applies, and no
//     constant candidate may be emitted.
//   - degUnknown — neither: a residual too small to disprove degeneracy and too
//     large to prove it. The caller owes an honest lower bound, a coarse
//     enclosure or `unsure` — NEVER a certificate.
//
// A tolerance may therefore route a question to `unsure`; it may never route
// one to a certificate. The cost is real and accepted (clearance §4): a rigid
// motion can land a meant-to-be-coaxial pair a few ulps off true, and the
// honest answer there is Suspect. Bodies built on a shared axis or sketch plane
// produce bit-identical floats, so the common configurations still prove out.
type degState int

const (
	// degUnknown: undecidable at the kernel's noise — never a certificate.
	degUnknown degState = iota
	// degYes: proven degenerate by exact arithmetic.
	degYes
	// degNo: proven not degenerate.
	degNo
)

// degAnd is the conjunction of two oracle answers: a single degNo disproves
// the conjunction, and any doubt keeps it undecided.
func degAnd(a, b degState) degState {
	switch {
	case a == degNo || b == degNo:
		return degNo
	case a == degUnknown || b == degUnknown:
		return degUnknown
	default:
		return degYes
	}
}

// ratV3 is a vector of the payload's own floats taken EXACTLY — the only
// arithmetic allowed to prove a degeneracy.
type ratV3 [3]*big.Rat

func ratVec(v r3.Vec) ratV3 { return ratV3{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func rvSub(a, b ratV3) ratV3 {
	var out ratV3
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func rvCross(a, b ratV3) ratV3 {
	mul := func(x, y *big.Rat) *big.Rat { return new(big.Rat).Mul(x, y) }
	return ratV3{
		new(big.Rat).Sub(mul(a[1], b[2]), mul(a[2], b[1])),
		new(big.Rat).Sub(mul(a[2], b[0]), mul(a[0], b[2])),
		new(big.Rat).Sub(mul(a[0], b[1]), mul(a[1], b[0])),
	}
}

func rvDot(a, b ratV3) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func rvIsZero(a ratV3) bool {
	return a[0].Sign() == 0 && a[1].Sign() == 0 && a[2].Sign() == 0
}

// finiteVec guards the exact tests: ratOf maps a non-finite float to zero, so
// a non-finite coordinate could otherwise forge an exactly-zero cross product.
func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}

// parallelRat decides a ∥ b from the vectors taken exactly (ra, rb) with their
// float forms (fa, fb) supplying the disproof threshold: an exactly zero cross
// product proves parallelism outright, a cross clearly above the kernel's
// angular noise disproves it, and the band between is undecided.
func (k *pairKernel) parallelRat(ra, rb ratV3, fa, fb r3.Vec) degState {
	la, lb := fa.Len(), fb.Len()
	if !finiteVec(fa) || !finiteVec(fb) || la == 0 || lb == 0 {
		return degUnknown
	}
	if rvIsZero(rvCross(ra, rb)) {
		return degYes
	}
	if fa.Cross(fb).Len() > clrAngTol*la*lb {
		return degNo
	}
	return degUnknown
}

// parallel decides a ∥ b.
func (k *pairKernel) parallel(a, b r3.Vec) degState {
	return k.parallelRat(ratVec(a), ratVec(b), a, b)
}

// parallelSeg decides (b − a) ∥ d, with the difference taken exactly (a float
// subtraction of the endpoints would round away the very residual the proof
// rests on).
func (k *pairKernel) parallelSeg(a, b, d r3.Vec) degState {
	return k.parallelRat(rvSub(ratVec(b), ratVec(a)), ratVec(d), b.Sub(a), d)
}

// parallelSegs decides (b1 − a1) ∥ (b2 − a2).
func (k *pairKernel) parallelSegs(a1, b1, a2, b2 r3.Vec) degState {
	return k.parallelRat(rvSub(ratVec(b1), ratVec(a1)), rvSub(ratVec(b2), ratVec(a2)),
		b1.Sub(a1), b2.Sub(a2))
}

// perpendicularSeg decides (b − a) ⟂ n — the plane-plateau question of the
// curve tiers, taken exactly.
func (k *pairKernel) perpendicularSeg(a, b, n r3.Vec) degState {
	rel := rvSub(ratVec(b), ratVec(a))
	fRel := b.Sub(a)
	lr, ln := fRel.Len(), n.Len()
	if !finiteVec(fRel) || !finiteVec(n) || lr == 0 || ln == 0 {
		return degUnknown
	}
	if rvDot(rel, ratVec(n)).Sign() == 0 {
		return degYes
	}
	if math.Abs(fRel.Dot(n)) > clrAngTol*lr*ln {
		return degNo
	}
	return degUnknown
}

// onAxis decides whether p lies on the line (anchor, unit axis): the offset is
// EXACTLY zero, provenly beyond the kernel's length noise, or undecided. Every
// cell that needs a radial direction off that offset asks here first — an
// offset in the undecided band normalizes to a garbage direction, and a garbage
// direction decides a trim admission.
func (k *pairKernel) onAxis(p, anchor, axis r3.Vec) degState {
	if !finiteVec(p) || !finiteVec(anchor) || !finiteVec(axis) {
		return degUnknown
	}
	rel := rvSub(ratVec(p), ratVec(anchor))
	if rvIsZero(rel) || rvIsZero(rvCross(rel, ratVec(axis))) {
		return degYes
	}
	fRel := p.Sub(anchor)
	if fRel.Sub(axis.Scale(fRel.Dot(axis))).Len() > k.tol {
		return degNo
	}
	return degUnknown
}

// coincident decides whether two points are the same point.
func (k *pairKernel) coincident(p, q r3.Vec) degState {
	if !finiteVec(p) || !finiteVec(q) {
		return degUnknown
	}
	if rvIsZero(rvSub(ratVec(p), ratVec(q))) {
		return degYes
	}
	if p.Sub(q).Len() > k.tol {
		return degNo
	}
	return degUnknown
}

// coaxial decides whether two axis-symmetric carriers share an axis LINE.
func (k *pairKernel) coaxial(f, g *cFace) degState {
	return degAnd(k.parallel(f.axis, g.axis), k.onAxis(g.anchor, f.anchor, f.axis))
}

// ringFamily decides §4's coincident-spine configurations — the ones whose
// annular gap really is attained all the way around a ring: concentric point
// spines, a point spine ON a line spine, and coaxial line spines (the
// peg-in-hole reading). A circle spine is never in the family, and a
// near-coincidence the oracle cannot decide is not either.
func (k *pairKernel) ringFamily(f, g *cFace) degState {
	switch sf, sg := spineOf(f), spineOf(g); {
	case sf == 0 && sg == 0:
		return k.coincident(f.anchor, g.anchor)
	case sf == 0 && sg == 1:
		return k.onAxis(f.anchor, g.anchor, g.axis)
	case sf == 1 && sg == 0:
		return k.onAxis(g.anchor, f.anchor, f.axis)
	case sf == 1 && sg == 1:
		return k.coaxial(f, g)
	default:
		return degNo
	}
}

// pointSpineDist is the distance from a point to a spine — closed form for all
// three spine kinds, and finite by construction (a point's distance to a set is
// a number, never a supremum over an unbounded carrier).
func pointSpineDist(p r3.Vec, g *cFace) float64 {
	rel := p.Sub(g.anchor)
	switch spineOf(g) {
	case 0:
		return rel.Len()
	case 1:
		return rel.Sub(g.axis.Scale(rel.Dot(g.axis))).Len()
	default:
		z := rel.Dot(g.axis)
		rho := rel.Sub(g.axis.Scale(z)).Len()
		return math.Hypot(z, math.Abs(rho-g.major))
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
func (k *pairKernel) spineSup(f, g *cFace) (float64, bool) {
	switch spineOf(f) {
	case 0:
		return pointSpineDist(f.anchor, g), true
	case 1:
		// An infinite line: bounded only against an exactly parallel line.
		if spineOf(g) != 1 || k.parallel(f.axis, g.axis) != degYes {
			return math.Inf(1), false
		}
		return pointSpineDist(f.anchor, g), true
	default:
		// A circle spine is bounded, but a constant supremum needs exact
		// coaxiality — a non-coaxial circle inside another curved carrier has
		// no constant supremum and never borrows this branch (§4).
		switch spineOf(g) {
		case 1:
			if k.coaxial(g, f) != degYes {
				return math.Inf(1), false
			}
			return f.major, true // every spine point sits f.major off the shared axis
		case 2:
			if k.coaxial(f, g) != degYes {
				return math.Inf(1), false
			}
			rel := f.anchor.Sub(g.anchor)
			return math.Hypot(rel.Dot(g.axis), f.major-g.major), true
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
func (k *pairKernel) certifiedContainment(f, g *cFace) bool {
	rf, rg := spineOffset(f), spineOffset(g)
	if sup, ok := k.spineSup(f, g); ok && rg > sup+rf+k.tol {
		return true
	}
	sup, ok := k.spineSup(g, f)
	return ok && rf > sup+rg+k.tol
}
