package survey2d

import (
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the 2D kernel behind the analytic wall survey
// (docs/verification-design.md §6, docs/evaluator-design.md §10): the
// spanning-ball reading of a prism or revolved body reduces exactly to
// empty-disk analysis in a 2D section — the profile plane for a prism, the
// meridian plane through the ball's center for a solid of revolution — so the
// survey enumerates a closed-form candidate set of critical inscribed disks
// and validates each against the whole boundary. The candidate set is
// complete for the attained infimum/supremum over line/arc boundaries: a
// critical disk has three contacts (an Apollonius solution), two contacts at
// an antipodal configuration (a pair critical), two contacts exactly at the
// allowance boundary (an angle-limit disk — the closed-under-limits family of
// §6), or a whole-arc contact (a concentric disk). Corner pinches — dihedrals
// within the allowance, which span at every size a ball is starved to — are
// the caller's to report; they read zero and need no disk.
//
// Every candidate carries a PROVEN bound beside its radius (DiskCand.rBound):
// each family derives it from its own closed form over the bounded-scalar
// arithmetic moments.go already owns (proofbound.BoundedAdd/proofbound.BoundedSub/proofbound.BoundedMul/
// proofbound.BoundedQuotient/proofbound.BoundedSqrt), reading every element COORDINATE (line
// endpoints and normals, arc centers, junction vertices) as an EXACT leaf —
// the same convention prismPayload's own directional readings use — so for
// those the bound speaks only for the arithmetic THIS file performs, never for
// the record's own numbers or for the draft allowance angle itself. Every step
// of that arithmetic is charged, coefficient construction and Cramer
// determinants included: reading a coordinate as exact never licenses reading
// a PRODUCT of two of them as exact. A division's bound comes from
// proofbound.BoundedQuotient's own denominator-and-numerator composition, so a small
// denominator amplifies it without a separate case; nothing here loosens a
// candidate SET, a containment scan, or a tolerance — the bound only ever
// widens the interval the winning candidate publishes.
//
// The kernel takes exactly TWO inputs that are not exact leaves, and both
// arrive already bounded rather than being trusted:
//
//   - SurveyElem.rrBound, an arc element's own radius error. A caller that
//     states a radius outright, and a recorded CircleSeg, both give zero; a
//     walk built from a recorded ArcSeg holds a math.Hypot radius and gives
//     extrude.go's arcWalkRadiusBound. Every candidate radius derived from an
//     element radius reads it through SurveyElem.rrBS, never as a leaf.
//   - WallKernel's wedgeS, the partial-revolve cap half-angle's SINE, which
//     its caller proves from a certified trig interval.
//
// Every candidate derived from either composes its bound like any other, so an
// input the caller could not enclose never reaches a published reading.
//
// A candidate's own bound is on its RADIUS, while the kernel's answer is a
// spanning DIAMETER (WallSurveyOut.span = 2r), so the bound is doubled with
// the value it accompanies. runBudget performs that conversion at the single
// point where a candidate enters the answer, so every field of
// WallSurveyOut that carries a bound already speaks in the answer's own
// units and no consumer restates the factor.

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
// a nanometre once composed through any reading this file publishes. It is
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

// DiskCand is a candidate inscribed disk. rBound is the PROVEN bound its own
// generator derived — zero only where that generator's arithmetic is exact
// (a literal coordinate difference, never a division or a square root of a
// non-perfect value).
type DiskCand struct{ X, Y, R, RBound float64 }

// WallKernel is one 2D section survey: elements (material on the left),
// junction vertices, the draft allowance, and — for a partial revolve — the
// wedge the sweep caps cut, encoded by wedgeS = sin(min(Δφ/2, π/2)): a ball
// of radius r centered at height y fits the wedge iff y·wedgeS ≥ r.
//
// wedgeS is one of the two kernel inputs that are NOT exact leaves (the file
// comment names both; the other is an element's own radius): it is a sine, so
// its caller (survey.go's revolveWall) proves it from a certified trig
// interval and hands it over as a proofbound.BoundedScalar. Every wedge-derived radius
// composes that bound through the same bounded arithmetic the rest of the file
// uses, so a candidate the wedge produced publishes an interval that contains
// the sine's own error.
type WallKernel struct {
	Elems       []SurveyElem
	ContainOnly []SurveyElem // boundary for containment only (on-axis chords)
	Verts       [][2]float64
	Alpha       float64
	// draftTrig holds certified ±(π−alpha) values in candidate order.
	DraftTrig  [2]struct{ sin, cos proofbound.BoundedScalar }
	WedgeS     proofbound.BoundedScalar // value 0 = no wedge
	WedgeSpans bool                     // two wedge-cap contacts count as a spanning pair
	FitMax     float64                  // spanning disks wider than this cannot lift to 3D
	Scale, Tol float64
	SubTolFar  bool         // a sub-tolerance candidate away from every junction was dropped
	Boundary   []SurveyElem // elems + containOnly, built lazily for contains
}

// NewWallKernel sizes the tolerances from the geometry with the default draft allowance.
func NewWallKernel(elems []SurveyElem, verts [][2]float64, fitMax float64) *WallKernel {
	k, _ := NewWallKernelBudget(nil, elems, nil, verts, 15*math.Pi/180, proofbound.ExactScalar(0), false, fitMax)
	return k
}

// NewWallKernelBudget sizes the tolerances from the geometry while charging
// the boundary scan to the caller's shared budget.
func NewWallKernelBudget(budget *proofbound.WorkBudget, elems, containOnly []SurveyElem, verts [][2]float64, alpha float64, wedgeS proofbound.BoundedScalar, wedgeSpans bool, fitMax float64) (*WallKernel, error) {
	if err := WallBudgetErr(budget); err != nil {
		return nil, err
	}
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, boundary := range [][]SurveyElem{elems, containOnly} {
		for _, e := range boundary {
			if err := WallBudgetStep(budget); err != nil {
				return nil, err
			}
			if e.Kind == SurveyLine {
				grow(e.Ax, e.Ay, e.Bx, e.By)
				continue
			}
			grow(e.Qx-e.Rr, e.Qx+e.Rr, e.Qy-e.Rr, e.Qy+e.Rr)
		}
	}
	aStar := math.Pi - alpha
	positiveSin, positiveCos := RadianTrigBounds(aStar)
	negativeSin, negativeCos := RadianTrigBounds(-aStar)
	return &WallKernel{
		Elems:       elems,
		ContainOnly: containOnly,
		Verts:       verts,
		Alpha:       alpha,
		DraftTrig: [2]struct{ sin, cos proofbound.BoundedScalar }{
			{sin: positiveSin, cos: positiveCos},
			{sin: negativeSin, cos: negativeCos},
		},
		WedgeS:     wedgeS,
		WedgeSpans: wedgeSpans,
		FitMax:     fitMax,
		Scale:      scale,
		Tol:        1e-9 * scale,
	}, nil
}

// WallSurveyOut is the kernel's answer over its candidate set. Both of its
// numbers are extrema over a population of candidates that each carry their
// OWN bound, so each is published as survey.go's §9.2 ExtremeAggregate proves
// it — the aggregate interval's midpoint beside its half-width — and never as
// the figures of whichever candidate won a comparison of held values. Two
// candidates whose held values order one way can have true values that order
// the other, and the aggregate is what stops the reading from following the
// held order off the truth.
//
// span and spanBound speak in the spanning DIAMETER, not in the candidate
// radius each generator actually bounded: runBudget doubles a candidate's
// value and its bound together as it feeds the aggregate, at the single point
// where a candidate enters the answer, so no consumer restates the factor.
type WallSurveyOut struct {
	Ok            bool    // false: the survey could not decide (never a silent pass)
	HasSpan       bool    // some empty spanning disk exists
	Span          float64 // the smallest spanning DIAMETER, as the aggregate proves it
	SpanBound     float64 // that aggregate interval's half-width, also on the DIAMETER
	Inradius      float64 // the largest empty disk radius found (the 2D inradius)
	InradiusBound float64 // that aggregate interval's half-width
	SubTolFar     bool    // an off-junction sub-tolerance candidate was dropped
}

var ErrWallSurveyUndecided = errors.New("wall survey undecided")

// WallBudgetStep counts one unit of wall work when Verify supplied a budget.
// The context-free shell-admission caller uses nil.
func WallBudgetStep(budget *proofbound.WorkBudget) error {
	if budget == nil {
		return nil
	}
	return budget.Step()
}

func WallBudgetErr(budget *proofbound.WorkBudget) error {
	if budget == nil {
		return nil
	}
	return budget.Err()
}

// run preserves the context-free kernel API used by shell admission.
func (k *WallKernel) Run() WallSurveyOut {
	out, _ := k.RunBudget(nil)
	return out
}

// runBudget streams and validates candidates under one shared Verify budget,
// folding each admitted candidate into the two §9.2 aggregates the kernel
// publishes — the least spanning diameter and the greatest empty radius.
// Neither extremum is taken by comparing held values: a candidate's own bound
// is its own, so the candidate that holds the smaller number need not be the
// one whose truth is smaller, and reducing by held order publishes an interval
// a rival's truth can sit outside of. The aggregates reach to whichever
// candidate's interval reaches furthest, which is also what keeps an exact
// reading exact: an inexact rival that does not reach the extremum moves
// neither end.
//
// A candidate whose own generator could not derive a finite bound never
// widens the published interval silently: it makes the whole survey
// undecided instead (the same refusal prismWall already takes for a
// displaced section), since a reading with no proven bound is not one this
// evaluator may stand behind. The aggregate itself carries that refusal, so a
// candidate that loses the extremum cannot smuggle an unbounded figure past
// it either.
func (k *WallKernel) RunBudget(budget *proofbound.WorkBudget) (WallSurveyOut, error) {
	out := WallSurveyOut{Ok: true, Span: math.Inf(1)}
	spans, radii := MinAggregate(), MaxAggregate()
	err := k.Generate(budget, func(c DiskCand) error {
		spanning, empty, ok, err := k.Validate(c, budget)
		if err != nil {
			return err
		}
		if !ok {
			return ErrWallSurveyUndecided
		}
		if !empty {
			return nil
		}
		radii.Take(c.R, c.RBound)
		if !spanning {
			return nil
		}
		// The candidate's generator bounded its RADIUS; the answer is the
		// DIAMETER 2r, so the bound doubles with the value. Multiplying a
		// float64 by two only scales the exponent, so the doubled bound is
		// exact and needs no outward rounding — the sole failure is an
		// overflow to +Inf, which the aggregate's own finiteness refusal
		// catches with every other underivable bound.
		//
		// The fit gate is read on the candidate's own proven interval, not on
		// its held diameter, and it is reject-only like validate's own guards:
		// a disk PROVEN wider than the sweep cannot lift to 3D and is dropped,
		// while one whose interval still reaches under the height is kept and
		// carries its doubt into the reading. Deciding a straddle the other way
		// would drop a wall the body may really have, which is the one error a
		// wall reading must never make.
		diam := proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.MeasuredScalar(c.R, c.RBound))
		if proofbound.AdmitAbove(diam, k.FitMax+k.Tol) == proofbound.SurvAdmit {
			return nil
		}
		out.HasSpan = true
		spans.Take(diam.Value, diam.Bound)
		return nil
	})
	if errors.Is(err, ErrWallSurveyUndecided) {
		return WallSurveyOut{}, nil
	}
	if err != nil {
		return WallSurveyOut{}, err
	}
	if out.HasSpan {
		span, bound, ok := spans.Resolve()
		if !ok {
			return WallSurveyOut{}, nil
		}
		out.Span, out.SpanBound = span, bound
	}
	if !radii.Empty() {
		inradius, bound, ok := radii.Resolve()
		if !ok {
			return WallSurveyOut{}, nil
		}
		out.Inradius, out.InradiusBound = inradius, bound
	} else if radii.Unbounded {
		return WallSurveyOut{}, nil
	}
	out.SubTolFar = k.SubTolFar
	return out, nil
}

// validate checks one candidate disk against the whole boundary: it must fit
// the wedge (if any), sit inside the material, and clear every element; its
// contact set decides whether it spans — two contact directions within the
// allowance of antipodal (inclusive), a single whole-arc contact wide
// enough, or (when the wedge's two cap contacts span) an active wedge.
//
// Every one of those readings is taken on the candidate's OWN proven interval
// rather than on its held radius, and each REJECTS only what that interval
// proves: an element proven to cut into the disk blocks it, an element proven
// clear of the rim is not a contact, a contact pair proven short of the
// allowance does not span. A straddle resolves the same way every generator's
// does — toward keeping the candidate and counting the contact — and the
// candidate's own interval carries the doubt into the reading.
//
// Resolving that way, rather than into the undecided channel, is a decision
// the numbers force and it is worth stating why. These guards compare against
// slacks the survey DECLARES — k.tol and contactTol() are 1e-9 and 8e-9 of the
// section's own scale — but the intervals read against them are NOT ulp-scale:
// a T3 angle-limit candidate composes the draft allowance's certified trig
// enclosure through a division, and on a 100 mm section its radius bound comes
// out near 1e-6 mm, two decades ABOVE k.tol. Straddles here are the ordinary
// case, not an exotic one, so an undecided cell would read Suspect on
// commonplace bodies — the wrong answer, since the doubt is a micron of
// arithmetic and the reading already has an interval to put it in.
//
// Keeping a straddling candidate is sound because the straddle bounds how far
// the true geometry can be from it. Where an element cannot be proven to cut
// into the disk, its distance d satisfies d >= r − b − k.tol, so an empty disk
// of radius min_j d_j really does sit at this centre, within the candidate's
// own published interval up to the declared slack — the same slack the held
// comparison already conceded. Where an element cannot be proven clear of the
// rim, a disk inside that interval really does touch it. What the interval
// cannot absorb is a bound that is not a number at all: runBudget's aggregate
// refuses a candidate whose generator could derive no finite bound, and that
// is the reading's undecided channel.
func (k *WallKernel) Validate(c DiskCand, budget *proofbound.WorkBudget) (bool, bool, bool, error) {
	if err := WallBudgetStep(budget); err != nil {
		return false, false, false, err
	}
	if math.IsNaN(c.X) || math.IsNaN(c.Y) || math.IsNaN(c.R) || math.IsInf(c.R, 0) {
		return false, false, true, nil
	}
	// The candidate floor: a disk this small is indistinguishable from the
	// numeric noise of a degenerate solve. Dropping it is safe ONLY where a
	// junction owns the spot — the junction-pinch rule reports the true
	// dihedral there exactly. Away from every junction the same tiny disk
	// can be a REAL near-tangent web (two boundary elements almost touching
	// mid-span), and treating it as absent would bless a body whose wall is
	// thinner than the kernel can resolve: that is an undecided survey,
	// never a silent pass.
	if c.R <= 4*k.Tol {
		reach := c.R + 8*k.Tol
		for _, v := range k.Verts {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			if math.Hypot(c.X-v[0], c.Y-v[1]) <= reach {
				return false, false, true, nil
			}
		}
		// Not junction-owned: remember it. The caller decides — an exact
		// zero elsewhere still stands (nothing sits below zero), but a
		// positive reading or an absence claim would rest on a candidate
		// the kernel could not resolve, and reads undecided instead.
		k.SubTolFar = true
		return false, false, true, nil
	}
	wedgeActive := false
	if k.HasWedge() {
		// A fit/contact test against k.tol, not a published reading: the sine's
		// own bound reaches the answer through the candidate radii the wedge
		// generates. The FIT half still refuses only what the sine's own
		// interval proves does not fit, so a disk the wedge might admit is
		// never discarded on a held product. The CONTACT half below stays a
		// held comparison: its slack is 8·k.tol = 8e-9 of the section's own
		// scale while the product's bound is that scale times the sine's
		// last-ulp figure, so the interval cannot reach across the slack and
		// the straddle is unreachable.
		room := proofbound.BoundedMul(proofbound.ExactScalar(c.Y), k.WedgeS)
		if proofbound.AdmitBelow(proofbound.BoundedSub(room, k.ClearRadius(c)), 0) == proofbound.SurvAdmit {
			return false, false, true, nil
		}
		// The contact half reads the same two intervals: the wedge counts as a
		// contact unless the room it leaves is PROVEN past the slack.
		wedgeActive = proofbound.AdmitAbove(proofbound.BoundedSub(room, k.ContactRadius(c)), 0) != proofbound.SurvAdmit
	}
	var contacts []DirArc
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return false, false, false, err
		}
		d, dir, dirOK := e.Nearest(c.X, c.Y, SurvTiny*k.Scale)
		// Emptiness: drop the candidate only where the element is PROVEN to cut
		// into the disk.
		if proofbound.AdmitBelow(proofbound.BoundedSub(proofbound.ExactScalar(d), k.ClearRadius(c)), 0) == proofbound.SurvAdmit {
			return false, false, true, nil
		}
		// Contact: count the element unless it is PROVEN clear of the rim.
		if proofbound.AdmitAbove(proofbound.BoundedSub(proofbound.ExactScalar(d), k.ContactRadius(c)), 0) == proofbound.SurvAdmit {
			continue
		}
		if !dirOK {
			// A contact whose direction is undefined: the disk is
			// degenerate against this element; do not decide on it.
			return false, false, true, nil
		}
		contacts = append(contacts, dir)
	}
	inside, ok, err := k.Contains(c.X, c.Y, budget)
	if err != nil {
		return false, false, false, err
	}
	if !ok {
		return false, false, false, nil
	}
	if !inside {
		return false, false, true, nil
	}
	spanning := k.WedgeSpans && wedgeActive
	// The spanning threshold composes Go's float64 math.Pi, so it is not the
	// exact π − α − SurvAngTol it denotes and carries PiRoundGuard; the angle
	// beside it is this file's own atan2 composition and carries
	// AngleReadGuard. Both are decades under SurvAngTol, the inclusive slack
	// the survey DECLARES for a wall drafted at exactly the allowance, so the
	// declared line — not the arithmetic — is what decides that wall, and a
	// straddle needs a dihedral within a femtoradian of the declared line.
	need := proofbound.BoundedSub(
		proofbound.BoundedSub(proofbound.MeasuredScalar(math.Pi, PiRoundGuard), proofbound.ExactScalar(k.Alpha)),
		proofbound.ExactScalar(SurvAngTol),
	)
	for i := 0; i < len(contacts) && !spanning; i++ {
		for j := i; j < len(contacts); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			ang := proofbound.MeasuredScalar(MaxAngleBetween(contacts[i], contacts[j]), AngleReadGuard)
			// A pair spans unless its own interval is PROVEN short of the
			// threshold, the same reject-only posture the two guards above
			// take.
			if proofbound.AdmitBelow(proofbound.BoundedSub(ang, need), 0) != proofbound.SurvAdmit {
				spanning = true
				break
			}
		}
	}
	return spanning, true, true, nil
}

// clearRadius and contactRadius are the two bands validate reads an element's
// distance against, each carrying the candidate's own proven radius bound: an
// element nearer than clearRadius cuts into the disk, one no further than
// contactRadius touches it. The slacks themselves are the survey's declared
// lines, not measurements, so they enter as exact leaves.
func (k *WallKernel) ClearRadius(c DiskCand) proofbound.BoundedScalar {
	return proofbound.BoundedSub(proofbound.MeasuredScalar(c.R, c.RBound), proofbound.ExactScalar(k.Tol))
}

func (k *WallKernel) ContactRadius(c DiskCand) proofbound.BoundedScalar {
	return proofbound.BoundedAdd(proofbound.MeasuredScalar(c.R, c.RBound), proofbound.ExactScalar(k.ContactTol()))
}

// contactTol is the slack for counting an element as a contact.
func (k *WallKernel) ContactTol() float64 { return 8 * k.Tol }

// AngleReadGuard is the survey's DECLARED conservative allowance on one
// contact-angle reading: MaxAngleBetween composes a handful of math.Atan2
// evaluations, differences and reductions over values no larger than 2π, and
// this file assumes no ulp contract from any of them (the same posture
// RadianTrigBounds takes toward math.Sin). Ten femtoradians is decades above
// the last ulp of 2π and five decades below SurvAngTol, the inclusive slack
// the spanning comparison is actually about, so declaring it costs the
// comparison nothing while keeping the reading from asserting an accuracy the
// library never promised.
const AngleReadGuard = 1e-14

// hasWedge reports whether this kernel carries a partial revolve's cap wedge at
// all. It is a STRUCTURAL question, not a numeric one: a full turn's caller
// states proofbound.ExactScalar(0), a proven zero, and a partial sweep's caller
// (survey.go's revolveWall) proves its own half-angle sine positive before
// handing it over — so the reading is never taken on a sine that cannot be
// told from zero, and the proofbound.SurvAdmit answer below is the only one a built
// kernel reaches.
func (k *WallKernel) HasWedge() bool {
	return proofbound.AdmitAbove(k.WedgeS, 0) == proofbound.SurvAdmit
}

// contains is the material-membership test: crossing parity of a ray against
// every boundary element, retried across directions when a crossing is
// ambiguous (near an endpoint, near tangency, or grazing the start). All
// directions ambiguous → undecided, never guessed.
func (k *WallKernel) Contains(px, py float64, budget *proofbound.WorkBudget) (bool, bool, error) {
	if k.Boundary == nil {
		boundary := make([]SurveyElem, 0, len(k.Elems)+len(k.ContainOnly))
		for _, elems := range [][]SurveyElem{k.Elems, k.ContainOnly} {
			for _, e := range elems {
				if err := WallBudgetStep(budget); err != nil {
					return false, false, err
				}
				boundary = append(boundary, e)
			}
		}
		k.Boundary = boundary
	}
	all := k.Boundary
	for i := range 16 {
		if err := WallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range all {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, good := RayCrossings(e, px, py, dx, dy, k.Tol)
			if !good {
				ok = false
				break
			}
			crossings += n
		}
		if ok {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}

// RayCrossings counts proper crossings of the ray p + t·d (t > 0) with one
// element; good is false when a crossing is too ambiguous to count.
func RayCrossings(e SurveyElem, px, py, dx, dy, tol float64) (int, bool) {
	if e.Kind == SurveyLine {
		ex, ey := e.Bx-e.Ax, e.By-e.Ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) - (-ex)*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			// Parallel: an overlap along the ray line is ambiguous; a clean
			// miss (the segment's line offset from the ray's) is a zero.
			if math.Abs((e.Ax-px)*dy-(e.Ay-py)*dx) <= tol {
				return 0, false
			}
			return 0, true
		}
		rx, ry := e.Ax-px, e.Ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return 0, true
		}
		if t <= tol {
			if t > -tol {
				return 0, false // the boundary passes through the ray start
			}
			return 0, true
		}
		if u <= uTol || u >= 1-uTol {
			return 0, false // the ray grazes a segment endpoint
		}
		return 1, true
	}
	// Arc: |p + t·d − q|² = R².
	fx, fy := px-e.Qx, py-e.Qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.Rr*e.Rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.Rr {
			// Grazing tangency: ambiguous only when it happens ahead of us.
			if -b > 0 {
				return 0, false
			}
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	n := 0
	for _, t := range []float64{-b - s, -b + s} {
		if t <= tol {
			if t > -tol {
				return 0, false
			}
			continue
		}
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.Qy, x-e.Qx)
		if e.Closed {
			n++
			continue
		}
		lo, hi := e.ArcRange()
		off := Mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.Rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			return 0, false
		}
		if off < ext {
			n++
		}
	}
	if math.Abs(s) <= tol {
		return 0, false
	}
	return n, true
}

// generate emits the closed-form candidate set: pair criticals (T2),
// angle-limit disks (T3), Apollonius triples (T4), concentric whole-arc
// disks, and — under a wedge — the wedge-tangent minima. Candidates are
// generated liberally; validate is what admits them. Each candidate is sent
// directly to visit so the cubic triple set is never materialized.
func (k *WallKernel) Generate(budget *proofbound.WorkBudget, visit func(DiskCand) error) error {
	var visitErr error
	add := func(x, y, r, rBound float64) {
		if visitErr == nil {
			visitErr = visit(DiskCand{X: x, Y: y, R: r, RBound: rBound})
		}
	}

	// Concentric whole-arc disks: the element's own radius, under the
	// element's own bound on it — zero for a stated or recorded radius, the
	// hypot bracket for an ArcSeg-derived one.
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if e.Kind == SurveyArc && e.MatInside {
			add(e.Qx, e.Qy, e.Rr, e.RrBound)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Pair criticals and angle limits.
	for i := range k.Elems {
		for j := i + 1; j < len(k.Elems); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.PairCands(k.Elems[i], k.Elems[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for _, e := range k.Elems {
		for _, v := range k.Verts {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.VertexElemCands(v, e, add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for i := range k.Verts {
		for j := i + 1; j < len(k.Verts); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.VertexVertexCands(k.Verts[i], k.Verts[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Wedge-tangent minima (partial revolve only).
	if k.HasWedge() {
		if err := k.WedgeCands(budget, visit); err != nil {
			return err
		}
	}

	// Apollonius triples, the wedge included as an item.
	eqs, err := k.TripleEquations(budget)
	if err != nil {
		return err
	}
	for i := range eqs {
		for j := i + 1; j < len(eqs); j++ {
			for l := j + 1; l < len(eqs); l++ {
				if err := WallBudgetStep(budget); err != nil {
					return err
				}
				SolveTriple([3]CircEq{eqs[i], eqs[j], eqs[l]}, k.Scale, add)
				if visitErr != nil {
					return visitErr
				}
			}
		}
	}
	return nil
}

// pairCands emits the antipodal pair criticals (T2) and the angle-limit
// disks (T3) for one element pair.
func (k *WallKernel) PairCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	switch {
	case a.Kind == SurveyLine && b.Kind == SurveyLine:
		k.LineLineCands(a, b, add)
	case a.Kind == SurveyArc && b.Kind == SurveyArc:
		k.ArcArcCands(a, b, add)
	case a.Kind == SurveyLine:
		k.LineArcCands(a, b, add)
	default:
		k.LineArcCands(b, a, add)
	}
}

// lineLineCands: parallel facing lines carry a constant-diameter family;
// the overlap midpoint is its representative (blocked positions surface via
// the triples). Non-parallel lines have no interior critical: their corners
// are junctions, their extent limits vertices, their blockers triples.
//
// The parallel and facing tests read the two elements' own unit normals under
// a 1e-9 slack. Those normals are element coordinates, exact leaves by this
// file's stated convention, and the slack is seven orders above their last
// ulp, so neither test can be straddled by the arithmetic it performs.
func (k *WallKernel) LineLineCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	cross := a.Nx*b.Ny - a.Ny*b.Nx
	if math.Abs(cross) > 1e-9 {
		return
	}
	if a.Nx*b.Nx+a.Ny*b.Ny > -1+1e-9 {
		return // same-facing parallels never oppose across material
	}
	dBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(a.Nx), proofbound.BoundedSub(proofbound.ExactScalar(b.Ax), proofbound.ExactScalar(a.Ax))),
		proofbound.BoundedMul(proofbound.ExactScalar(a.Ny), proofbound.BoundedSub(proofbound.ExactScalar(b.Ay), proofbound.ExactScalar(a.Ay))),
	)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, 0) == proofbound.SurvReject {
		return // b is PROVEN not on a's material side
	}
	// Overlap of the two segments along a's tangent.
	tx, ty := -a.Ny, a.Nx
	a0, a1 := tx*a.Ax+ty*a.Ay, tx*a.Bx+ty*a.By
	b0, b1 := tx*b.Ax+ty*b.Ay, tx*b.Bx+ty*b.By
	lo := math.Max(math.Min(a0, a1), math.Min(b0, b1))
	hi := math.Min(math.Max(a0, a1), math.Max(b0, b1))
	if lo > hi {
		return
	}
	m := (lo + hi) / 2
	// Base point on a at parameter m, then half-way toward b.
	base := tx*a.Ax + ty*a.Ay
	px := a.Ax + (m-base)*tx + a.Nx*d/2
	py := a.Ay + (m-base)*ty + a.Ny*d/2
	rBS := proofbound.BoundedQuotient(d, dBS.Bound, 2, 0)
	add(px, py, d/2, rBS.Bound)
}

// lineArcCands: the critical disks sit on the perpendicular from the arc's
// center to the line; the angle-limit disks sit where the contact pair
// reaches exactly the allowance boundary.
//
// The T2 denominator sgn·s − 1 is built from two SIGNS, so it takes one of the
// exactly representable values {0, −2, 2} and its degeneracy reading can never
// straddle. The T3 denominator is 1 + cos α for the caller's own draft
// allowance α, and reaches zero only at α = π; a straddle there needs an
// allowance within a micro-radian of a half turn, which is no draft angle.
func (k *WallKernel) LineArcCands(l, a SurveyElem, add func(x, y, r, rBound float64)) {
	s := a.MatSign()
	e := l.Nx*(a.Qx-l.Ax) + l.Ny*(a.Qy-l.Ay) // signed height of q, material side positive
	eBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), proofbound.BoundedSub(proofbound.ExactScalar(a.Qx), proofbound.ExactScalar(l.Ax))),
		proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), proofbound.BoundedSub(proofbound.ExactScalar(a.Qy), proofbound.ExactScalar(l.Ay))),
	)
	fx := a.Qx - e*l.Nx
	fy := a.Qy - e*l.Ny
	// T2 on the perpendicular from q to the line, t = r, center at f + t·n̂:
	// |e − t| = R − s·t, enumerated as e − t = sgn·(R − s·t).
	for _, sgn := range []float64{1, -1} {
		den := sgn*s - 1
		if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
			continue
		}
		t := (sgn*a.Rr - e) / den
		numBS := proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sgn), a.RrBS()), eBS)
		tBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
		if proofbound.AdmitAbove(tBS, 0) != proofbound.SurvReject {
			add(fx+t*l.Nx, fy+t*l.Ny, t, tBS.Bound)
		}
	}
	// T3: contact directions exactly π − α apart. The line contact direction
	// is −n̂; the arc contact direction is s·(ĉ−q̂); c = q + (s·R − r)·u2.
	for _, trig := range k.DraftTrig {
		snBS, csBS := trig.sin, trig.cos
		cs, sn := csBS.Value, snBS.Value
		// u2 = rotate(−n̂, ±aStar)
		ux := -(l.Nx*cs - l.Ny*sn)
		uy := -(l.Nx*sn + l.Ny*cs)
		uxBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), csBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), snBS)))
		uyBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), snBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), csBS)))
		ndot := l.Nx*ux + l.Ny*uy // = −cos(aStar)
		ndotBS := proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), uxBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), uyBS))
		denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), ndotBS)
		if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
			continue
		}
		den := denBS.Value
		r := (l.Nx*(a.Qx-l.Ax) + l.Ny*(a.Qy-l.Ay) + s*a.Rr*ndot) / den
		numBS := proofbound.BoundedAdd(eBS, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(s), a.RrBS()), ndotBS))
		rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
		if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
			add(a.Qx+(s*a.Rr-r)*ux, a.Qy+(s*a.Rr-r)*uy, r, rBS.Bound)
		}
	}
}

// arcArcCands: centerline criticals (T2, the concentric family included) and
// the law-of-cosines angle-limit disks (T3).
//
// The concentric test is the one branch here that is NOT resolved toward
// generating both sides: a centre separation the interval cannot prove positive
// leaves the centreline direction dx/d undefined, so the general branch has no
// candidate to build and the concentric family is the only reading of that
// pair. The T2 denominator −εa·sa − εb·sb is built from four SIGNS and so takes
// one of {0, ±2} exactly, which no interval can straddle.
func (k *WallKernel) ArcArcCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	dx, dy := b.Qx-a.Qx, b.Qy-a.Qy
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	sa, sb := a.MatSign(), b.MatSign()
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		// Concentric: the family disk at each arc's own angular midpoint. The
		// annulus half-width is |Ra − Rb|/2, and NEITHER step is exact — the
		// difference of two radii rounds outside the Sterbenz range, and each
		// radius carries whatever bound its own element states — so the whole
		// chain runs through the bounded arithmetic. The halving is exact, but
		// proofbound.BoundedQuotient carries the difference's bound through it.
		diffBS := proofbound.BoundedAbs(proofbound.BoundedSub(a.RrBS(), b.RrBS()))
		rBS := proofbound.BoundedQuotient(diffBS.Value, diffBS.Bound, 2, 0)
		r := rBS.Value
		m := (a.Rr + b.Rr) / 2
		for _, e := range []SurveyElem{a, b} {
			lo, hi := e.ArcRange()
			th := (lo + hi) / 2
			add(a.Qx+m*math.Cos(th), a.Qy+m*math.Sin(th), r, rBS.Bound)
		}
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the centerline: εa(Ra − sa·r) + εb(Rb − sb·r) = d.
	for _, ea := range []float64{1, -1} {
		for _, eb := range []float64{1, -1} {
			den := -ea*sa - eb*sb
			if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (d - ea*a.Rr - eb*b.Rr) / den
			numBS := proofbound.BoundedSub(dBS, proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.ExactScalar(ea), a.RrBS()),
				proofbound.BoundedMul(proofbound.ExactScalar(eb), b.RrBS()),
			))
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
			if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
				continue
			}
			t := ea * (a.Rr - sa*r)
			add(a.Qx+t*ux, a.Qy+t*uy, r, rBS.Bound)
		}
	}
	// T3: angle at the center between (c−qa) and (c−qb) fixed by the
	// allowance boundary; law of cosines in r, then the two mirror centers.
	cosAStarBS := k.DraftTrig[0].cos
	cosThBS := proofbound.BoundedMul(proofbound.ExactScalar(sa*sb), cosAStarBS)
	// d² = Da² + Db² − 2·Da·Db·cosθ with Da = Ra − sa·r, Db = Rb − sb·r.
	raBS, rbBS := a.RrBS(), b.RrBS()
	ABS := proofbound.BoundedSub(proofbound.ExactScalar(2), proofbound.BoundedMul(proofbound.ExactScalar(2*sa*sb), cosThBS))
	BBS := proofbound.BoundedAdd(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(-2*sa), raBS), proofbound.BoundedMul(proofbound.ExactScalar(-2*sb), rbBS)),
		proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(sb), raBS),
			proofbound.BoundedMul(proofbound.ExactScalar(sa), rbBS),
		)), cosThBS),
	)
	CBS := proofbound.BoundedSub(
		proofbound.BoundedAdd(
			proofbound.BoundedAdd(proofbound.BoundedMul(raBS, raBS), proofbound.BoundedMul(rbBS, rbBS)),
			proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(-2), proofbound.BoundedMul(raBS, rbBS)), cosThBS),
		),
		proofbound.BoundedMul(dBS, dBS),
	)
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
			continue
		}
		r := rBS.Value
		da := a.Rr - sa*r
		db := b.Rr - sb*r
		PlaceCircleCircle(a.Qx, a.Qy, da, b.Qx, b.Qy, db, add, r, rBS.Bound)
	}
}

// vertexElemCands: the vertex-line foot and vertex-arc centerline criticals
// (T2) plus their angle-limit disks (T3).
//
// A vertex sitting ON the element it is read against — the ordinary case, since
// a junction vertex IS an endpoint of its own two walks — gives an exactly zero
// height or separation with an exactly zero bound, so the side and degeneracy
// readings below decide it outright. The arc T2 denominator sgn·s − ev is built
// from three SIGNS and takes one of {0, ±2} exactly.
func (k *WallKernel) VertexElemCands(v [2]float64, e SurveyElem, add func(x, y, r, rBound float64)) {
	if e.Kind == SurveyLine {
		// T2: the foot midpoint.
		h := e.Nx*(v[0]-e.Ax) + e.Ny*(v[1]-e.Ay)
		hBS := proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), proofbound.BoundedSub(proofbound.ExactScalar(v[0]), proofbound.ExactScalar(e.Ax))),
			proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), proofbound.BoundedSub(proofbound.ExactScalar(v[1]), proofbound.ExactScalar(e.Ay))),
		)
		if proofbound.AdmitAbove(hBS, 0) != proofbound.SurvReject {
			rBS := proofbound.BoundedQuotient(hBS.Value, hBS.Bound, 2, 0)
			add(v[0]-h/2*e.Nx, v[1]-h/2*e.Ny, h/2, rBS.Bound)
		}
		// T3: u_v = rotate(−n̂, ±A*), c = v − r·u_v, tangency fixes r.
		for _, trig := range k.DraftTrig {
			snBS, csBS := trig.sin, trig.cos
			cs, sn := csBS.Value, snBS.Value
			ux := -(e.Nx*cs - e.Ny*sn)
			uy := -(e.Nx*sn + e.Ny*cs)
			uxBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), csBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), snBS)))
			uyBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), snBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), csBS)))
			den := 1 + (e.Nx*ux + e.Ny*uy)
			denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), uxBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), uyBS)))
			if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (e.Nx*(v[0]-e.Ax) + e.Ny*(v[1]-e.Ay)) / den
			rBS := proofbound.BoundedQuotient(hBS.Value, hBS.Bound, denBS.Value, denBS.Bound)
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				add(v[0]-r*ux, v[1]-r*uy, r, rBS.Bound)
			}
		}
		return
	}
	s := e.MatSign()
	dx, dy := e.Qx-v[0], e.Qy-v[1]
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the line through v and q: |c−v| = r with c = v ± r·û, and
	// |c−q| = R − s·r.
	for _, ev := range []float64{1, -1} {
		// c = v + ev·r·û: |c − q| = |d − ev·r| = R − s·r.
		for _, sgn := range []float64{1, -1} {
			den := sgn*s - ev
			if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (sgn*e.Rr - d) / den
			numBS := proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sgn), e.RrBS()), dBS)
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				add(v[0]+ev*r*ux, v[1]+ev*r*uy, r, rBS.Bound)
			}
		}
	}
	// T3: law of cosines with sides r and R − s·r.
	cosAStarBS := k.DraftTrig[0].cos
	cosThBS := proofbound.BoundedMul(proofbound.ExactScalar(-s), cosAStarBS)
	ABS := proofbound.BoundedAdd(proofbound.ExactScalar(2), proofbound.BoundedMul(proofbound.ExactScalar(2*s), cosThBS))
	reBS := e.RrBS()
	BBS := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(-2), reBS), proofbound.BoundedAdd(proofbound.ExactScalar(s), cosThBS))
	CBS := proofbound.BoundedSub(proofbound.BoundedMul(reBS, reBS), proofbound.BoundedMul(dBS, dBS))
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
			continue
		}
		r := rBS.Value
		PlaceCircleCircle(v[0], v[1], r, e.Qx, e.Qy, e.Rr-s*r, add, r, rBS.Bound)
	}
}

// vertexVertexCands: the midpoint disk (T2) and the isoceles angle-limit
// disks (T3).
//
// Two coincident vertices give an exactly zero separation with an exactly zero
// bound, so the degeneracy reading decides them. The T3 denominator is
// 2·(1 − cos(π − α)) = 2·(1 + cos α) for the caller's own draft allowance, and
// vanishes only at α = π — an allowance of a half turn, which is no draft.
func (k *WallKernel) VertexVertexCands(a, b [2]float64, add func(x, y, r, rBound float64)) {
	dx, dy := b[0]-a[0], b[1]-a[1]
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		return
	}
	rBS := proofbound.BoundedQuotient(dBS.Value, dBS.Bound, 2, 0)
	add((a[0]+b[0])/2, (a[1]+b[1])/2, d/2, rBS.Bound)
	cosAStarBS := k.DraftTrig[0].cos
	denBS := proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedSub(proofbound.ExactScalar(1), cosAStarBS))
	if proofbound.AdmitAbove(denBS, SurvTiny) != proofbound.SurvReject {
		sqrtDenBS := proofbound.BoundedSqrt(denBS)
		rr := proofbound.BoundedQuotient(dBS.Value, dBS.Bound, sqrtDenBS.Value, sqrtDenBS.Bound)
		PlaceCircleCircle(a[0], a[1], rr.Value, b[0], b[1], rr.Value, add, rr.Value, rr.Bound)
	}
}

// wedgeCands: the wedge-tangent minima for a partial revolve's cap-cap
// reading — the disk tangent to one element with the wedge constraint
// active (r = wedgeS·y), at its own closed-form ρ-critical. wedgeS carries its
// own bound (a sine its caller proved from a certified trig interval), so BOTH
// quotient paths below compose that bound through their numerator and
// denominator rather than reading the sine as an exact leaf, and the arc path
// reads the element's radius through rrBS for the same reason.
//
// A vertex at or below the axis (v[1] <= 0) is an exact recorded coordinate
// against an exact zero, so that test needs no interval. Every other reading
// here is taken on the proven interval and resolved toward emitting: the wedge
// denominators 1 ∓ sin(Δφ/2) reach zero only for a half sweep at exactly a
// right angle, where the sine is proven 1 and the reading is a proven reject,
// never a straddle.
func (k *WallKernel) WedgeCands(budget *proofbound.WorkBudget, visit func(DiskCand) error) error {
	sBS := k.WedgeS
	s := sBS.Value
	for _, v := range k.Verts {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if v[1] <= 0 {
			continue
		}
		for _, sgn := range []float64{1, -1} {
			denBS := proofbound.BoundedSub(proofbound.ExactScalar(1), proofbound.BoundedMul(proofbound.ExactScalar(sgn), sBS))
			if proofbound.AdmitAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			numBS := proofbound.BoundedMul(sBS, proofbound.ExactScalar(v[1]))
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
			r := rBS.Value
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				if err := visit(DiskCand{X: v[0], Y: r / s, R: r, RBound: rBS.Bound}); err != nil {
					return err
				}
			}
		}
	}
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if e.Kind != SurveyArc {
			continue // a line's wedge-tangent family has no interior minimum
		}
		se := e.MatSign()
		for _, th := range []float64{math.Pi / 2, -math.Pi / 2} {
			if !e.ArcContains(th) {
				continue
			}
			sinBS, _ := RadianTrigBounds(th)
			numBS := proofbound.BoundedMul(sBS, proofbound.BoundedAdd(proofbound.ExactScalar(e.Qy), proofbound.BoundedMul(e.RrBS(), sinBS)))
			denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), proofbound.BoundedMul(proofbound.BoundedMul(sBS, proofbound.ExactScalar(se)), sinBS))
			if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
			if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
				continue
			}
			r := rBS.Value
			d := e.Rr - se*r
			if err := visit(DiskCand{X: e.Qx + d*math.Cos(th), Y: e.Qy + d*math.Sin(th), R: r, RBound: rBS.Bound}); err != nil {
				return err
			}
		}
	}
	return nil
}

// CircEq is one tangency equation for the Apollonius triples: either linear
// (a·cx + b·cy + e·r + f = 0) or quadratic
// (cx² + cy² − r² + g·cx + h·cy + kk·r + m = 0). Every coefficient is a
// proofbound.BoundedScalar, not a bare float: the element, vertex and wedge coordinates
// the equations are built from are exact leaves, but forming a coefficient
// from them rounds (a product of two coordinates, a sum of three such
// products); an arc element's radius is not even a leaf, since it arrives with
// its own bound (SurveyElem.rrBound); and the wedge's own coefficient is a
// sine that arrives already bounded.
// SolveTriple/Solve3Linear/SolveParallelPair compose those bounds through
// every determinant, numerator and division they perform, so a triple's
// published radius interval speaks for the WHOLE chain from coefficient
// construction down to the final quotient.
type CircEq struct {
	Quad        bool
	G, H, Kk, M proofbound.BoundedScalar
	A, B, E, F  proofbound.BoundedScalar
}

// tripleEquations builds the material-side-pinned tangency equation of every
// item: elements, vertices, and the wedge.
func (k *WallKernel) TripleEquations(budget *proofbound.WorkBudget) ([]CircEq, error) {
	var eqs []CircEq
	for _, el := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if el.Kind == SurveyLine {
			nx, ny := proofbound.ExactScalar(el.Nx), proofbound.ExactScalar(el.Ny)
			eqs = append(eqs, CircEq{
				A: nx, B: ny, E: proofbound.ExactScalar(-1),
				F: proofbound.BoundedNeg(proofbound.BoundedAdd(
					proofbound.BoundedMul(nx, proofbound.ExactScalar(el.Ax)),
					proofbound.BoundedMul(ny, proofbound.ExactScalar(el.Ay)),
				)),
			})
			continue
		}
		s := el.MatSign()
		qx, qy, rr := proofbound.ExactScalar(el.Qx), proofbound.ExactScalar(el.Qy), el.RrBS()
		eqs = append(eqs, CircEq{
			Quad: true,
			G:    proofbound.BoundedMul(proofbound.ExactScalar(-2), qx),
			H:    proofbound.BoundedMul(proofbound.ExactScalar(-2), qy),
			Kk:   proofbound.BoundedMul(proofbound.ExactScalar(2*s), rr),
			M: proofbound.BoundedSub(
				proofbound.BoundedAdd(proofbound.BoundedMul(qx, qx), proofbound.BoundedMul(qy, qy)),
				proofbound.BoundedMul(rr, rr),
			),
		})
	}
	for _, v := range k.Verts {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		vx, vy := proofbound.ExactScalar(v[0]), proofbound.ExactScalar(v[1])
		eqs = append(eqs, CircEq{
			Quad: true,
			G:    proofbound.BoundedMul(proofbound.ExactScalar(-2), vx),
			H:    proofbound.BoundedMul(proofbound.ExactScalar(-2), vy),
			M:    proofbound.BoundedAdd(proofbound.BoundedMul(vx, vx), proofbound.BoundedMul(vy, vy)),
		})
	}
	if k.HasWedge() {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		// The wedge's coefficient is the caller's already-bounded sine, so a
		// triple that includes the wedge inherits the sine's own error.
		eqs = append(eqs, CircEq{B: k.WedgeS, E: proofbound.ExactScalar(-1)})
	}
	return eqs, nil
}

// SolveTriple solves one triple of tangency equations in closed form:
// quadratic pairs subtract to linear, leaving at most one quadratic; two
// independent linears express the center affinely in r, and substitution
// yields a quadratic in r. Parallel linear pairs pin r instead and leave the
// position as the unknown.
//
// The parallel/independent split is the one reading here that decides between
// two GENERATORS rather than between a candidate and nothing, so an interval
// that cannot separate the pair's determinant from zero runs both: a
// superfluous candidate costs a validate pass and can never reach a reading
// it does not genuinely satisfy, while a missing one is a wall this survey
// would not see.
func SolveTriple(eqs [3]CircEq, scale float64, add func(x, y, r, rBound float64)) {
	var lins []CircEq
	var quad *CircEq
	for i, e := range eqs {
		if !e.Quad {
			lins = append(lins, e)
			continue
		}
		if quad == nil {
			q := eqs[i]
			quad = &q
			continue
		}
		// Subtract: the c·c − r² terms cancel.
		lins = append(lins, CircEq{
			A: proofbound.BoundedSub(e.G, quad.G), B: proofbound.BoundedSub(e.H, quad.H),
			E: proofbound.BoundedSub(e.Kk, quad.Kk), F: proofbound.BoundedSub(e.M, quad.M),
		})
	}
	if len(lins) == 3 {
		Solve3Linear(lins, add)
		return
	}
	if len(lins) != 2 || quad == nil {
		return
	}
	l1, l2 := lins[0], lins[1]
	detBS := proofbound.BoundedSub(proofbound.BoundedMul(l1.A, l2.B), proofbound.BoundedMul(l2.A, l1.B))
	parallel := proofbound.AdmitMagnitudeAbove(detBS, 1e-12*math.Max(1, scale))
	if parallel != proofbound.SurvAdmit {
		SolveParallelPair(l1, l2, *quad, add)
		if parallel == proofbound.SurvReject {
			return
		}
	}
	// (cx, cy) = P + r·Q from the two linears. The triple's OWN division (by
	// det) composes through proofbound.BoundedQuotient like every other division here, so
	// P and Q reach the quadratic below carrying their own error rather than
	// as exact leaves.
	pxBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.F), l2.B), proofbound.BoundedMul(l2.F, l1.B)), detBS)
	pyBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.A), l2.F), proofbound.BoundedMul(l2.A, l1.F)), detBS)
	qxBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.E), l2.B), proofbound.BoundedMul(l2.E, l1.B)), detBS)
	qyBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.A), l2.E), proofbound.BoundedMul(l2.A, l1.E)), detBS)
	// The same denominator guard QuadRootsBounded takes, read off the four
	// divisions themselves: proofbound.BoundedQuotient answers +Inf exactly where the
	// determinant's own interval left it no positive clearance, so a P or Q with
	// no finite bound says this pair is not separated from parallel and the
	// affine centre it would carry into the quadratic is not a disk anyone can
	// state. Dropping THIS generator's candidate here is local — the pair's own
	// reading came from SolveParallelPair a few lines above, which is why the
	// straddle runs both — and it is what keeps one straddled triple from
	// handing runBudget's aggregate an unbounded candidate and leaving the whole
	// survey undecided.
	if proofbound.IsNonFinite(pxBS.Bound) || proofbound.IsNonFinite(pyBS.Bound) || proofbound.IsNonFinite(qxBS.Bound) || proofbound.IsNonFinite(qyBS.Bound) {
		return
	}
	ABS := proofbound.BoundedSub(proofbound.BoundedAdd(proofbound.BoundedMul(qxBS, qxBS), proofbound.BoundedMul(qyBS, qyBS)), proofbound.ExactScalar(1))
	BBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedAdd(proofbound.BoundedMul(pxBS, qxBS), proofbound.BoundedMul(pyBS, qyBS))),
		proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(quad.G, qxBS), proofbound.BoundedMul(quad.H, qyBS)), quad.Kk),
	)
	CBS := proofbound.BoundedAdd(
		proofbound.BoundedAdd(proofbound.BoundedMul(pxBS, pxBS), proofbound.BoundedMul(pyBS, pyBS)),
		proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(quad.G, pxBS), proofbound.BoundedMul(quad.H, pyBS)), quad.M),
	)
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		r := rBS.Value
		if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
			add(pxBS.Value+r*qxBS.Value, pyBS.Value+r*qyBS.Value, r, rBS.Bound)
		}
	}
}

// Solve3Linear solves three linear tangency equations by Cramer's rule. Every
// 2×2 minor, every expansion of it, and the final division compose through the
// bounded arithmetic, so the radius the candidate publishes carries the error
// of the WHOLE rule and not just its last division: a well-conditioned triple
// still rounds six products and three sums into each numerator before the
// quotient ever runs.
//
// A determinant this rule cannot separate from zero is the one reading here
// that stops rather than emits, and it drops no attained extremum: three
// tangency equations whose determinant the arithmetic cannot prove nonzero are
// dependent to within their own error, so they name no ISOLATED disk at all —
// the family they describe reaches the sink through the pair criticals and
// SolveParallelPair, which is what those generators are for. Emitting instead
// would publish a position and radius Cramer's rule divides out of a vanishing
// denominator, whose own +Inf bound would then leave every survey containing
// such a triple undecided — which for a rotated or placed section is most of
// them.
func Solve3Linear(l []CircEq, add func(x, y, r, rBound float64)) {
	// The 2×2 minors of rows 1 and 2 over each column pair, named for the
	// columns they keep.
	minor := func(p, q, s, t proofbound.BoundedScalar) proofbound.BoundedScalar {
		return proofbound.BoundedSub(proofbound.BoundedMul(p, t), proofbound.BoundedMul(s, q))
	}
	be := minor(l[1].B, l[1].E, l[2].B, l[2].E)
	ae := minor(l[1].A, l[1].E, l[2].A, l[2].E)
	ab := minor(l[1].A, l[1].B, l[2].A, l[2].B)
	fe := minor(l[1].F, l[1].E, l[2].F, l[2].E)
	fb := minor(l[1].F, l[1].B, l[2].F, l[2].B)
	af := minor(l[1].A, l[1].F, l[2].A, l[2].F)
	bf := minor(l[1].B, l[1].F, l[2].B, l[2].F)
	detBS := proofbound.BoundedAdd(
		proofbound.BoundedSub(proofbound.BoundedMul(l[0].A, be), proofbound.BoundedMul(l[0].B, ae)),
		proofbound.BoundedMul(l[0].E, ab),
	)
	if proofbound.AdmitMagnitudeAbove(detBS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	drBS := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].A), bf), proofbound.BoundedMul(l[0].B, af)),
		proofbound.BoundedMul(l[0].F, ab),
	)
	rBS := proofbound.BoundedDiv(drBS, detBS)
	if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
		return
	}
	// The center is a position, never a published reading (see
	// PlaceCircleCircle), so it stays a plain quotient.
	dx := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].F), be), proofbound.BoundedMul(l[0].B, fe)),
		proofbound.BoundedMul(l[0].E, fb),
	).Value
	dy := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].A), fe), proofbound.BoundedMul(l[0].F, ae)),
		proofbound.BoundedMul(l[0].E, af),
	).Value
	add(dx/detBS.Value, dy/detBS.Value, rBS.Value, rBS.Bound)
}

// SolveParallelPair handles two parallel linear tangency equations plus a
// quadratic: the pair fixes r and a line of centers; the quadratic picks the
// positions along it. The normalization, the 2×2 determinant and the final
// division all compose their bounds, so the radius every emitted position
// shares carries the error of the whole reduction; the positions themselves
// never feed a published reading and stay plain floats.
//
// A normal this rule cannot separate from zero is not a line: the equation
// carries no direction to normalize by, so there is nothing to solve and the
// reading stops, the same way Solve3Linear's dependent triple does. The
// orientation sign σ is the one reading here whose straddle is unreachable
// from its own algebra — the two normals are already unit vectors and the
// caller only reaches this function for a pair whose cross product is at or
// below its own parallelism threshold, so their dot product sits within a few
// ulps of ±1 and no interval this arithmetic produces spans zero.
func SolveParallelPair(l1, l2, q CircEq, add func(x, y, r, rBound float64)) {
	nBS := proofbound.BoundedNorm2(l1.A, l1.B)
	if proofbound.AdmitAbove(nBS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	// Normalize both to unit normals; solve the 2×2 system in (h, r) where
	// h = n̂·c along l1's normal.
	a1, b1 := proofbound.BoundedDiv(l1.A, nBS), proofbound.BoundedDiv(l1.B, nBS)
	e1, f1 := proofbound.BoundedDiv(l1.E, nBS), proofbound.BoundedDiv(l1.F, nBS)
	n2BS := proofbound.BoundedNorm2(l2.A, l2.B)
	if proofbound.AdmitAbove(n2BS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	a2, b2 := proofbound.BoundedDiv(l2.A, n2BS), proofbound.BoundedDiv(l2.B, n2BS)
	e2, f2 := proofbound.BoundedDiv(l2.E, n2BS), proofbound.BoundedDiv(l2.F, n2BS)
	// n̂2 = ±n̂1: sign σ.
	sigma := -1.0
	if proofbound.AdmitAbove(proofbound.BoundedAdd(proofbound.BoundedMul(a1, a2), proofbound.BoundedMul(b1, b2)), 0) == proofbound.SurvAdmit {
		sigma = 1
	}
	// eq1: h + e1·r + f1 = 0; eq2: σ·h + e2·r + f2 = 0.
	detBS := proofbound.BoundedSub(e2, proofbound.BoundedMul(proofbound.ExactScalar(sigma), e1))
	if proofbound.AdmitMagnitudeAbove(detBS, SurvTiny) == proofbound.SurvReject {
		return
	}
	rBS := proofbound.BoundedDiv(proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sigma), f1), f2), detBS)
	r := rBS.Value
	if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
		return
	}
	h := -e1.Value*r - f1.Value
	// Centers: c = h·n̂1 + t·t̂1. Substitute into the quadratic.
	tx, ty := -b1.Value, a1.Value
	bx, by := h*a1.Value, h*b1.Value
	A := 1.0
	B := 2*(bx*tx+by*ty) + q.G.Value*tx + q.H.Value*ty
	C := bx*bx + by*by - r*r + q.G.Value*bx + q.H.Value*by + q.Kk.Value*r + q.M.Value
	for _, t := range QuadRoots(A, B, C) {
		add(bx+t*tx, by+t*ty, r, rBS.Bound)
	}
}

// QuadRoots returns the real roots of A·x² + B·x + C = 0 (both when A ≈ 0 —
// the linear case — and the full quadratic).
func QuadRoots(A, B, C float64) []float64 {
	if math.Abs(A) < SurvTiny {
		if math.Abs(B) < SurvTiny {
			return nil
		}
		return []float64{-C / B}
	}
	disc := B*B - 4*A*C
	if disc < 0 {
		return nil
	}
	s := math.Sqrt(disc)
	return []float64{(-B - s) / (2 * A), (-B + s) / (2 * A)}
}

// PlaceCircleCircle emits the (up to two) points at distance da from
// (ax, ay) and db from (bx, by), as candidate centers of radius r (with its
// own proven bound rBound, unaffected by the position solve below — the
// position never feeds a published reading).
func PlaceCircleCircle(ax, ay, da, bx, by, db float64, add func(x, y, r, rBound float64), r, rBound float64) {
	dx, dy := bx-ax, by-ay
	d := math.Hypot(dx, dy)
	if d < SurvTiny || da < 0 || db < 0 {
		return
	}
	a := (da*da - db*db + d*d) / (2 * d)
	h2 := da*da - a*a
	if h2 < 0 {
		return
	}
	h := math.Sqrt(h2)
	mx, my := ax+a*dx/d, ay+a*dy/d
	px, py := -dy/d, dx/d
	add(mx+h*px, my+h*py, r, rBound)
	if h > 0 {
		add(mx-h*px, my-h*py, r, rBound)
	}
}
