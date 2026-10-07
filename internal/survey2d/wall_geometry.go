package survey2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// SurvTiny is the relative floor for degeneracy checks inside the kernel.
const SurvTiny = 1e-12

// SurvAngTol is the inclusive angular slack: a wall drafted at exactly the
// allowance spans (verification §6 — within is inclusive), so every angular
// comparison concedes this much in the spanning direction only.
const SurvAngTol = 1e-9

// PiRoundGuard bounds how far a held radian value built by composing Go's
// float64 math.Pi constant (the draft allowance's own aStar = π − α, or a
// literal ±π/2 wedge-tangent angle) can sit from the same expression
// evaluated at the TRUE mathematical π: at most the constant's own half-ulp
// rounding (docs/verification-design.md §6 never assumes better), well under
// a nanometre once composed through any reading the kernel publishes. It is
// folded once into RadianTrigBounds rather than re-derived at every call
// site that composes an angle through math.Pi.
const PiRoundGuard = 1e-15

// RadianTrigBounds proves sin(x) and cos(x) for a held RADIAN value x built
// from Go's math.Pi constant, via the same rational bracket
// (proofbound.RadSinCosInterval in internal/proofbound/interval_trig.go) a
// Cone's half-angle normal reads,
// widened by PiRoundGuard. It never trusts math.Sin/math.Cos's own accuracy —
// only that the enclosure it returns contains the true sine/cosine of x.
func RadianTrigBounds(x float64) (proofbound.BoundedScalar, proofbound.BoundedScalar) {
	heldSin, heldCos := math.Sin(x), math.Cos(x)
	xR := proofarith.FloatRat(x)
	if xR == nil {
		return proofbound.MeasuredScalar(heldSin, math.Inf(1)), proofbound.MeasuredScalar(heldCos, math.Inf(1))
	}
	sinIv, cosIv, ok := proofbound.RadSinCosInterval(xR)
	if !ok {
		return proofbound.MeasuredScalar(heldSin, math.Inf(1)), proofbound.MeasuredScalar(heldCos, math.Inf(1))
	}
	guard := proofarith.FloatRat(PiRoundGuard)
	sinIv, cosIv = proofbound.IntervalWiden(sinIv, guard), proofbound.IntervalWiden(cosIv, guard)
	return proofbound.MeasuredScalar(heldSin, proofbound.IntervalFloatError(sinIv, heldSin)),
		proofbound.MeasuredScalar(heldCos, proofbound.IntervalFloatError(cosIv, heldCos))
}

// QuadRootsBounded is QuadRoots' bound-carrying twin, over the same
// A·x²+B·x+C=0 closed form (both the degenerate linear branch and the full
// quadratic), each returned root's bound derived from ITS OWN denominator —
// 2A, or B in the linear branch — and discriminant via proofbound.BoundedQuotient's own
// clearance gate, never a global relative constant: a denominator near zero
// amplifies a fixed numerator error without limit, and proofbound.BoundedQuotient
// answers +Inf exactly where that amplification can no longer be bounded.
//
// Every branch here is taken on the coefficient's own PROVEN interval
// (proofbound.AdmitMagnitudeAbove/proofbound.AdmitBelow), never on its held value. A DISCRIMINANT
// that straddles resolves toward emitting the root: it discards the pair only
// when its whole interval is proven negative, and an interval crossing zero has
// real roots this kernel may not throw away, so it goes to proofbound.BoundedSqrt, which
// clamps the held operand at zero for the evaluation while keeping the
// interval's own upper end.
//
// A DENOMINATOR that straddles resolves the other way, and it has to. The
// quadratic form divides by 2A, so an A whose interval spans zero leaves
// proofbound.BoundedQuotient no positive clearance and it answers +Inf — not a wide bound
// but no bound at all, and a candidate carrying one silences the whole survey
// through runBudget's aggregate. What a straddling A really says is that the
// equation is linear to within its own error, so the root this kernel wants is
// the degenerate one, −C/B, whose own denominator is generally well separated:
// that root is emitted here instead of the pair the vanishing denominator would
// have produced. Where neither 2A nor B can be separated from zero the triple is
// dropped, on the dependency argument Solve3Linear's doc comment already writes
// down — coefficients whose arithmetic cannot separate a denominator from zero
// name no isolated disk, and the family they describe reaches the sink through
// the pair criticals and SolveParallelPair. The refusal stays local to the
// triple: runBudget's aggregate still refuses any unbounded candidate it is
// handed, which is what stops a survey publishing an interval it cannot prove.
func QuadRootsBounded(A, B, C proofbound.BoundedScalar) []proofbound.BoundedScalar {
	// The degenerate linear root, emitted wherever the quadratic form's own
	// denominator cannot be separated from zero.
	linearRoot := func() []proofbound.BoundedScalar {
		if proofbound.AdmitMagnitudeAbove(B, SurvTiny) != proofbound.SurvAdmit {
			return nil
		}
		return []proofbound.BoundedScalar{proofbound.BoundedQuotient(-C.Value, C.Bound, B.Value, B.Bound)}
	}
	if proofbound.AdmitMagnitudeAbove(A, SurvTiny) == proofbound.SurvReject {
		return linearRoot()
	}
	disc := proofbound.BoundedSub(proofbound.BoundedMul(B, B), proofbound.BoundedMul(proofbound.ExactScalar(4), proofbound.BoundedMul(A, C)))
	if proofbound.AdmitBelow(disc, 0) == proofbound.SurvAdmit {
		return nil
	}
	twoA := proofbound.BoundedMul(proofbound.ExactScalar(2), A)
	if proofbound.AdmitMagnitudeAbove(twoA, SurvTiny) != proofbound.SurvAdmit {
		return linearRoot()
	}
	s := proofbound.BoundedSqrt(disc)
	negB := proofbound.MeasuredScalar(-B.Value, B.Bound)
	num1 := proofbound.BoundedSub(negB, s)
	num2 := proofbound.BoundedAdd(negB, s)
	return []proofbound.BoundedScalar{
		proofbound.BoundedQuotient(num1.Value, num1.Bound, twoA.Value, twoA.Bound),
		proofbound.BoundedQuotient(num2.Value, num2.Bound, twoA.Value, twoA.Bound),
	}
}

// DirArc is a closed set of unit directions {(cos t, sin t) : t ∈ [lo, hi]},
// lo ≤ hi ≤ lo + 2π. A single direction has lo == hi. Contact directions are
// dirArcs because a disk can touch an arc element along a whole angular
// range (the concentric contact).
type DirArc struct{ Lo, Hi float64 }

// Mod2pi maps x into [0, 2π).
func Mod2pi(x float64) float64 {
	m := math.Mod(x, 2*math.Pi)
	if m < 0 {
		m += 2 * math.Pi
	}
	return m
}

// AngDist is the angular distance between two directions, in [0, π].
func AngDist(a, b float64) float64 {
	return math.Pi - math.Abs(Mod2pi(a-b)-math.Pi)
}

// ArcsIntersect reports whether two direction arcs share a direction,
// modulo 2π.
func ArcsIntersect(a, b DirArc) bool {
	aext := a.Hi - a.Lo
	bext := b.Hi - b.Lo
	if aext >= 2*math.Pi || bext >= 2*math.Pi {
		return true
	}
	d := Mod2pi(b.Lo - a.Lo)
	return d <= aext || d+bext >= 2*math.Pi
}

// MaxAngleBetween is the largest angle (≤ π) between a direction of a and a
// direction of b. It peaks at π exactly when some direction of a has its
// antipode in b; otherwise the maximum sits at interval endpoints.
func MaxAngleBetween(a, b DirArc) float64 {
	if ArcsIntersect(DirArc{Lo: a.Lo + math.Pi, Hi: a.Hi + math.Pi}, b) {
		return math.Pi
	}
	m := 0.0
	for _, ta := range []float64{a.Lo, a.Hi} {
		for _, tb := range []float64{b.Lo, b.Hi} {
			if d := AngDist(ta, tb); d > m {
				m = d
			}
		}
	}
	return m
}

// SurveyElemKind discriminates the kernel's boundary elements.
type SurveyElemKind int

const (
	SurveyLine SurveyElemKind = iota
	SurveyArc
)

// SurveyElem is one 2D boundary element, walked with the material on its
// left.
type SurveyElem struct {
	Kind SurveyElemKind

	// line: (ax, ay) → (bx, by); (nx, ny) is the unit inward (material-side)
	// normal.
	Ax, Ay, Bx, By float64
	Nx, Ny         float64

	// arc: center (qx, qy), radius rr, walked from th0 to th1. th1 > th0 is a
	// counter-clockwise walk, which puts the material inside the circle
	// (matInside); a clockwise walk puts it outside (a hole wall, a notch).
	//
	// rrBound is the PROVEN error bound on rr. Unlike the coordinates around
	// it, rr is NOT always an exact leaf: a walk built from a recorded ArcSeg
	// carries a math.Hypot radius (extrude.go's arcWalkRadiusBound). It is
	// zero for a caller that states the radius outright and for a recorded
	// CircleSeg. Every candidate radius the WALL kernel below derives from rr
	// composes this bound through its own arithmetic rather than reading rr as
	// exact.
	//
	// The readings that publish no interval of their own read the HELD rr
	// instead — nearest's distance and contact direction, and the tolerance
	// comparisons validate comes to make against them — and the headroom is the
	// argument for that, not convenience. arcWalkRadiusBound brackets a hypot of
	// recorded coordinate differences, so rrBound is at most 2^-50 of the
	// radius, a few ulp of it; the kernel's own scale is never smaller than an
	// element radius, so rrBound is at most 2^-50·scale ≈ 8.9e-16·scale, while
	// the weakest slack any of those readings concedes is k.tol = 1e-9·scale.
	// That is six decades of headroom, seven against contactTol's 8·k.tol, so
	// widening the readings by rrBound would move nothing any of them decides.
	// TestArcWalkRadiusBoundStaysUnderTheKernelSlack pins both halves of that
	// chain so the argument cannot rot unnoticed.
	//
	// contains/RayCrossings is a different kind of reading again: a held-float
	// PARITY predicate, whose soundness rests on its own ambiguity guards and
	// the 16-direction retry — a crossing too close to an endpoint, a tangency
	// or the ray start is refused outright and the next direction tried, never
	// counted — and not on any operand bound. The arc radius is neither the
	// weakest term there nor a distinguishable one: the same predicate rounds
	// fx*fx + fy*fy − rr*rr on its own account, by more than a radius shift of
	// this size moves it, so an interval carrying rrBound alone could not prove
	// the count either.
	Qx, Qy, Rr float64
	RrBound    float64
	Th0, Th1   float64
	Closed     bool
	MatInside  bool
}

// rrBS is the element's radius with its own proven bound — the one spelling
// every candidate derivation reads the radius through.
func (e SurveyElem) RrBS() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(e.Rr, e.RrBound)
}

// LineElem builds a line element from a walk-ordered segment.
func LineElem(ax, ay, bx, by float64) (SurveyElem, bool) {
	tx, ty := bx-ax, by-ay
	l := math.Hypot(tx, ty)
	if l == 0 {
		return SurveyElem{}, false
	}
	// Left of the walk direction is the material side.
	return SurveyElem{
		Kind: SurveyLine,
		Ax:   ax, Ay: ay, Bx: bx, By: by,
		Nx: -ty / l, Ny: tx / l,
	}, true
}

// ArcElem builds an arc element from walk geometry.
func ArcElem(qx, qy, rr, th0, th1 float64, closed bool) (SurveyElem, bool) {
	if rr <= 0 {
		return SurveyElem{}, false
	}
	return SurveyElem{
		Kind: SurveyArc,
		Qx:   qx, Qy: qy, Rr: rr,
		Th0: th0, Th1: th1, Closed: closed,
		MatInside: th1 > th0,
	}, true
}

// arcRange is the walked angular range in ascending order.
func (e SurveyElem) ArcRange() (float64, float64) {
	if e.Th1 >= e.Th0 {
		return e.Th0, e.Th1
	}
	return e.Th1, e.Th0
}

// arcContains reports whether angle θ lies on the walked range, modulo 2π.
func (e SurveyElem) ArcContains(th float64) bool {
	if e.Closed {
		return true
	}
	lo, hi := e.ArcRange()
	if hi-lo >= 2*math.Pi {
		return true
	}
	return Mod2pi(th-lo) <= hi-lo
}

// matSign is +1 for a material-inside arc (an inscribed disk sits inside the
// circle, |c−q| = R − r) and −1 for material-outside (|c−q| = R + r).
func (e SurveyElem) MatSign() float64 {
	if e.MatInside {
		return 1
	}
	return -1
}

// nearest returns the distance from p to the element and the direction set
// from p toward its nearest point(s). dirOK is false when the direction is
// undefined (p on the element, or exactly at an arc's center where the
// contact is the whole walked range — reported as that range).
func (e SurveyElem) Nearest(px, py, tiny float64) (float64, DirArc, bool) {
	switch e.Kind {
	case SurveyLine:
		ex, ey := e.Bx-e.Ax, e.By-e.Ay
		l2 := ex*ex + ey*ey
		u := ((px-e.Ax)*ex + (py-e.Ay)*ey) / l2
		u = math.Max(0, math.Min(1, u))
		nx, ny := e.Ax+u*ex-px, e.Ay+u*ey-py
		d := math.Hypot(nx, ny)
		if d <= tiny {
			return d, DirArc{}, false
		}
		th := math.Atan2(ny, nx)
		return d, DirArc{Lo: th, Hi: th}, true
	default: // SurveyArc
		vx, vy := px-e.Qx, py-e.Qy
		rho := math.Hypot(vx, vy)
		if rho <= tiny {
			// The concentric query: every point of the walked range is
			// nearest, and the contact direction set is the range itself.
			lo, hi := e.ArcRange()
			return e.Rr, DirArc{Lo: lo, Hi: hi}, true
		}
		thp := math.Atan2(vy, vx)
		if e.ArcContains(thp) {
			d := math.Abs(rho - e.Rr)
			if d <= tiny {
				return d, DirArc{}, false
			}
			cx := e.Qx + e.Rr*math.Cos(thp) - px
			cy := e.Qy + e.Rr*math.Sin(thp) - py
			th := math.Atan2(cy, cx)
			return d, DirArc{Lo: th, Hi: th}, true
		}
		// Off the walked range: the nearest point is an endpoint.
		lo, hi := e.ArcRange()
		d := math.Inf(1)
		var bx, by float64
		for _, th := range []float64{lo, hi} {
			x := e.Qx + e.Rr*math.Cos(th)
			y := e.Qy + e.Rr*math.Sin(th)
			if dd := math.Hypot(x-px, y-py); dd < d {
				d, bx, by = dd, x, y
			}
		}
		if d <= tiny {
			return d, DirArc{}, false
		}
		th := math.Atan2(by-py, bx-px)
		return d, DirArc{Lo: th, Hi: th}, true
	}
}
