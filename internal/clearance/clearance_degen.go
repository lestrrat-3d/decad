package clearance

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// DegState is the three-valued answer of the clearance kernel's degeneracy
// oracle (the root package's clearance_degen.go), and the discipline that
// keeps a certificate honest (docs/clearance-design.md §4/§5).
//
// A closed-form cell is exact only where the configuration it assumes actually
// holds: a plateau needs EXACT parallelism, a constant-distance critical needs
// EXACT coaxiality, the nested branch needs a supremum that is actually finite.
// Deciding any of those with a tolerance mints an Exact reading the true answer
// undercuts by up to the tolerance — a lie the report then certifies. So every
// such question routes through here and comes back THREE-valued:
//
//   - DegYes — proven degenerate by EXACT arithmetic on the payload's own
//     floats (rational cross/dot products over math/big.Rat, the same
//     take-the-floats-exactly discipline as internal/freeform/clearance_poly.go's Sturm
//     brackets). The closed form IS the answer, and the candidate is Exact.
//   - DegNo — proven NOT degenerate, by a residual clearly above the kernel's
//     own noise. The general (non-degenerate) closed form applies, and no
//     constant candidate may be emitted.
//   - DegUnknown — neither: a residual too small to disprove degeneracy and too
//     large to prove it. The caller owes an honest lower bound, a coarse
//     enclosure or `unsure` — NEVER a certificate.
//
// A tolerance may therefore route a question to `unsure`; it may never route
// one to a certificate. The cost is real and accepted (clearance §4): a rigid
// motion can land a meant-to-be-coaxial pair a few ulps off true, and the
// honest answer there is Suspect. Bodies built on a shared axis or sketch plane
// produce bit-identical floats, so the common configurations still prove out.
type DegState int

const (
	// DegUnknown: undecidable at the kernel's noise — never a certificate.
	DegUnknown DegState = iota
	// DegYes: proven degenerate by exact arithmetic.
	DegYes
	// DegNo: proven not degenerate.
	DegNo
)

// DegAnd is the conjunction of two oracle answers: a single DegNo disproves
// the conjunction, and any doubt keeps it undecided.
func DegAnd(a, b DegState) DegState {
	switch {
	case a == DegNo || b == DegNo:
		return DegNo
	case a == DegUnknown || b == DegUnknown:
		return DegUnknown
	default:
		return DegYes
	}
}

// PointSpineDist is the distance from a point to a spine — closed form for all
// three spine kinds, and finite by construction (a point's distance to a set is
// a number, never a supremum over an unbounded carrier).
func PointSpineDist(p r3.Vec, g *CFace) float64 {
	rel := p.Sub(g.Anchor)
	switch SpineOf(g) {
	case 0:
		return rel.Len()
	case 1:
		return rel.Sub(g.Axis.Scale(rel.Dot(g.Axis))).Len()
	default:
		z := rel.Dot(g.Axis)
		rho := rel.Sub(g.Axis.Scale(z)).Len()
		return math.Hypot(z, math.Abs(rho-g.Major))
	}
}
