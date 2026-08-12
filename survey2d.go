package decad

import (
	"errors"
	"math"
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
// Every candidate carries a PROVEN bound beside its radius (diskCand.rBound):
// each family derives it from its own closed form over the bounded-scalar
// arithmetic moments.go already owns (boundedAdd/boundedSub/boundedMul/
// boundedQuotient/boundedSqrt), reading every element COORDINATE (line
// endpoints and normals, arc centers, junction vertices) as an EXACT leaf —
// the same convention prismPayload's own directional readings use — so for
// those the bound speaks only for the arithmetic THIS file performs, never for
// the record's own numbers or for the draft allowance angle itself. Every step
// of that arithmetic is charged, coefficient construction and Cramer
// determinants included: reading a coordinate as exact never licenses reading
// a PRODUCT of two of them as exact. A division's bound comes from
// boundedQuotient's own denominator-and-numerator composition, so a small
// denominator amplifies it without a separate case; nothing here loosens a
// candidate SET, a containment scan, or a tolerance — the bound only ever
// widens the interval the winning candidate publishes.
//
// The kernel takes exactly TWO inputs that are not exact leaves, and both
// arrive already bounded rather than being trusted:
//
//   - surveyElem.rrBound, an arc element's own radius error. A caller that
//     states a radius outright, and a recorded CircleSeg, both give zero; a
//     walk built from a recorded ArcSeg holds a math.Hypot radius and gives
//     extrude.go's arcWalkRadiusBound. Every candidate radius derived from an
//     element radius reads it through surveyElem.rrBS, never as a leaf.
//   - wallKernel's wedgeS, the partial-revolve cap half-angle's SINE, which
//     its caller proves from a certified trig interval.
//
// Every candidate derived from either composes its bound like any other, so an
// input the caller could not enclose never reaches a published reading.
//
// A candidate's own bound is on its RADIUS, while the kernel's answer is a
// spanning DIAMETER (wallSurveyOut.span = 2r), so the bound is doubled with
// the value it accompanies. runBudget performs that conversion at the single
// point where a candidate enters the answer, so every field of
// wallSurveyOut that carries a bound already speaks in the answer's own
// units and no consumer restates the factor.

// survTiny is the relative floor for degeneracy checks inside the kernel.
const survTiny = 1e-12

// survAngTol is the inclusive angular slack: a wall drafted at exactly the
// allowance spans (verification §6 — within is inclusive), so every angular
// comparison concedes this much in the spanning direction only.
const survAngTol = 1e-9

// piRoundGuard bounds how far a held radian value built by composing Go's
// float64 math.Pi constant (the draft allowance's own aStar = π − α, or a
// literal ±π/2 wedge-tangent angle) can sit from the same expression
// evaluated at the TRUE mathematical π: at most the constant's own half-ulp
// rounding (docs/verification-design.md §6 never assumes better), well under
// a nanometre once composed through any reading this file publishes. It is
// folded once into radianTrigBounds rather than re-derived at every call
// site that composes an angle through math.Pi.
const piRoundGuard = 1e-15

// radianTrigBounds proves sin(x) and cos(x) for a held RADIAN value x built
// from Go's math.Pi constant, via the same rational bracket
// (radSinCosInterval, normal_bound.go) a Cone's half-angle normal reads,
// widened by piRoundGuard. It never trusts math.Sin/math.Cos's own accuracy —
// only that the enclosure it returns contains the true sine/cosine of x.
func radianTrigBounds(x float64) (boundedScalar, boundedScalar) {
	heldSin, heldCos := math.Sin(x), math.Cos(x)
	xR := floatRat(x)
	if xR == nil {
		return measuredScalar(heldSin, math.Inf(1)), measuredScalar(heldCos, math.Inf(1))
	}
	sinIv, cosIv, ok := radSinCosInterval(xR)
	if !ok {
		return measuredScalar(heldSin, math.Inf(1)), measuredScalar(heldCos, math.Inf(1))
	}
	guard := floatRat(piRoundGuard)
	sinIv, cosIv = intervalWiden(sinIv, guard), intervalWiden(cosIv, guard)
	return measuredScalar(heldSin, intervalFloatError(sinIv, heldSin)),
		measuredScalar(heldCos, intervalFloatError(cosIv, heldCos))
}

// boundedNorm2 is boundedSqrt's own reduction of a 2D length, over two
// components that already carry their own proven bounds. It never routes
// through math.Hypot's undocumented accuracy: the sum of squares composes
// through the bounded arithmetic, and the square root's own rounding comes
// from boundedSqrt.
func boundedNorm2(x, y boundedScalar) boundedScalar {
	return boundedSqrt(boundedAdd(boundedMul(x, x), boundedMul(y, y)))
}

// boundedHypot is boundedNorm2 over two EXACT leaves — the same convention
// every element coordinate in this file takes (line endpoints and normals, arc
// centers, junction vertices). An element's radius is not one of them: it
// carries its own bound and is read through surveyElem.rrBS.
func boundedHypot(dx, dy float64) boundedScalar {
	return boundedNorm2(exactScalar(dx), exactScalar(dy))
}

// quadRootsBounded is quadRoots' bound-carrying twin, over the same
// A·x²+B·x+C=0 closed form (both the degenerate linear branch and the full
// quadratic), each returned root's bound derived from ITS OWN denominator —
// 2A, or B in the linear branch — and discriminant via boundedQuotient's own
// clearance gate, never a global relative constant: a denominator near zero
// amplifies a fixed numerator error without limit, and boundedQuotient
// answers +Inf exactly where that amplification can no longer be bounded.
//
// Every branch here is taken on the coefficient's own PROVEN interval
// (admitMagnitudeAbove/admitBelow), never on its held value, and every
// three-valued answer resolves the same way: toward emitting the root. The
// linear branch is taken only for a leading coefficient PROVEN degenerate, so
// a straddling A goes to the quadratic form, which still recovers the linear
// root — under a wide bound its own cancelling numerator earns. The
// discriminant likewise discards the pair only when its whole interval is
// proven negative; an interval crossing zero has real roots this kernel may not
// throw away, so it goes to boundedSqrt, which clamps the held operand at zero
// for the evaluation while keeping the interval's own upper end.
func quadRootsBounded(A, B, C boundedScalar) []boundedScalar {
	if admitMagnitudeAbove(A, survTiny) == survReject {
		if admitMagnitudeAbove(B, survTiny) == survReject {
			return nil
		}
		return []boundedScalar{boundedQuotient(-C.value, C.bound, B.value, B.bound)}
	}
	disc := boundedSub(boundedMul(B, B), boundedMul(exactScalar(4), boundedMul(A, C)))
	if admitBelow(disc, 0) == survAdmit {
		return nil
	}
	s := boundedSqrt(disc)
	twoA := boundedMul(exactScalar(2), A)
	negB := measuredScalar(-B.value, B.bound)
	num1 := boundedSub(negB, s)
	num2 := boundedAdd(negB, s)
	return []boundedScalar{
		boundedQuotient(num1.value, num1.bound, twoA.value, twoA.bound),
		boundedQuotient(num2.value, num2.bound, twoA.value, twoA.bound),
	}
}

// dirArc is a closed set of unit directions {(cos t, sin t) : t ∈ [lo, hi]},
// lo ≤ hi ≤ lo + 2π. A single direction has lo == hi. Contact directions are
// dirArcs because a disk can touch an arc element along a whole angular
// range (the concentric contact).
type dirArc struct{ lo, hi float64 }

// mod2pi maps x into [0, 2π).
func mod2pi(x float64) float64 {
	m := math.Mod(x, 2*math.Pi)
	if m < 0 {
		m += 2 * math.Pi
	}
	return m
}

// angDist is the angular distance between two directions, in [0, π].
func angDist(a, b float64) float64 {
	return math.Pi - math.Abs(mod2pi(a-b)-math.Pi)
}

// arcsIntersect reports whether two direction arcs share a direction,
// modulo 2π.
func arcsIntersect(a, b dirArc) bool {
	aext := a.hi - a.lo
	bext := b.hi - b.lo
	if aext >= 2*math.Pi || bext >= 2*math.Pi {
		return true
	}
	d := mod2pi(b.lo - a.lo)
	return d <= aext || d+bext >= 2*math.Pi
}

// maxAngleBetween is the largest angle (≤ π) between a direction of a and a
// direction of b. It peaks at π exactly when some direction of a has its
// antipode in b; otherwise the maximum sits at interval endpoints.
func maxAngleBetween(a, b dirArc) float64 {
	if arcsIntersect(dirArc{lo: a.lo + math.Pi, hi: a.hi + math.Pi}, b) {
		return math.Pi
	}
	m := 0.0
	for _, ta := range []float64{a.lo, a.hi} {
		for _, tb := range []float64{b.lo, b.hi} {
			if d := angDist(ta, tb); d > m {
				m = d
			}
		}
	}
	return m
}

// surveyElemKind discriminates the kernel's boundary elements.
type surveyElemKind int

const (
	surveyLine surveyElemKind = iota
	surveyArc
)

// surveyElem is one 2D boundary element, walked with the material on its
// left.
type surveyElem struct {
	kind surveyElemKind

	// line: (ax, ay) → (bx, by); (nx, ny) is the unit inward (material-side)
	// normal.
	ax, ay, bx, by float64
	nx, ny         float64

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
	// exact; the held rr alone still serves the positions, ray casts and
	// tolerances that publish no interval of their own.
	qx, qy, rr float64
	rrBound    float64
	th0, th1   float64
	closed     bool
	matInside  bool
}

// rrBS is the element's radius with its own proven bound — the one spelling
// every candidate derivation reads the radius through.
func (e surveyElem) rrBS() boundedScalar { return measuredScalar(e.rr, e.rrBound) }

// lineElem builds a line element from a walk-ordered segment.
func lineElem(ax, ay, bx, by float64) (surveyElem, bool) {
	tx, ty := bx-ax, by-ay
	l := math.Hypot(tx, ty)
	if l == 0 {
		return surveyElem{}, false
	}
	// Left of the walk direction is the material side.
	return surveyElem{
		kind: surveyLine,
		ax:   ax, ay: ay, bx: bx, by: by,
		nx: -ty / l, ny: tx / l,
	}, true
}

// arcElem builds an arc element from walk geometry.
func arcElem(qx, qy, rr, th0, th1 float64, closed bool) (surveyElem, bool) {
	if rr <= 0 {
		return surveyElem{}, false
	}
	return surveyElem{
		kind: surveyArc,
		qx:   qx, qy: qy, rr: rr,
		th0: th0, th1: th1, closed: closed,
		matInside: th1 > th0,
	}, true
}

// arcRange is the walked angular range in ascending order.
func (e surveyElem) arcRange() (float64, float64) {
	if e.th1 >= e.th0 {
		return e.th0, e.th1
	}
	return e.th1, e.th0
}

// arcContains reports whether angle θ lies on the walked range, modulo 2π.
func (e surveyElem) arcContains(th float64) bool {
	if e.closed {
		return true
	}
	lo, hi := e.arcRange()
	if hi-lo >= 2*math.Pi {
		return true
	}
	return mod2pi(th-lo) <= hi-lo
}

// matSign is +1 for a material-inside arc (an inscribed disk sits inside the
// circle, |c−q| = R − r) and −1 for material-outside (|c−q| = R + r).
func (e surveyElem) matSign() float64 {
	if e.matInside {
		return 1
	}
	return -1
}

// nearest returns the distance from p to the element and the direction set
// from p toward its nearest point(s). dirOK is false when the direction is
// undefined (p on the element, or exactly at an arc's center where the
// contact is the whole walked range — reported as that range).
func (e surveyElem) nearest(px, py, tiny float64) (float64, dirArc, bool) {
	switch e.kind {
	case surveyLine:
		ex, ey := e.bx-e.ax, e.by-e.ay
		l2 := ex*ex + ey*ey
		u := ((px-e.ax)*ex + (py-e.ay)*ey) / l2
		u = math.Max(0, math.Min(1, u))
		nx, ny := e.ax+u*ex-px, e.ay+u*ey-py
		d := math.Hypot(nx, ny)
		if d <= tiny {
			return d, dirArc{}, false
		}
		th := math.Atan2(ny, nx)
		return d, dirArc{lo: th, hi: th}, true
	default: // surveyArc
		vx, vy := px-e.qx, py-e.qy
		rho := math.Hypot(vx, vy)
		if rho <= tiny {
			// The concentric query: every point of the walked range is
			// nearest, and the contact direction set is the range itself.
			lo, hi := e.arcRange()
			return e.rr, dirArc{lo: lo, hi: hi}, true
		}
		thp := math.Atan2(vy, vx)
		if e.arcContains(thp) {
			d := math.Abs(rho - e.rr)
			if d <= tiny {
				return d, dirArc{}, false
			}
			cx := e.qx + e.rr*math.Cos(thp) - px
			cy := e.qy + e.rr*math.Sin(thp) - py
			th := math.Atan2(cy, cx)
			return d, dirArc{lo: th, hi: th}, true
		}
		// Off the walked range: the nearest point is an endpoint.
		lo, hi := e.arcRange()
		d := math.Inf(1)
		var bx, by float64
		for _, th := range []float64{lo, hi} {
			x := e.qx + e.rr*math.Cos(th)
			y := e.qy + e.rr*math.Sin(th)
			if dd := math.Hypot(x-px, y-py); dd < d {
				d, bx, by = dd, x, y
			}
		}
		if d <= tiny {
			return d, dirArc{}, false
		}
		th := math.Atan2(by-py, bx-px)
		return d, dirArc{lo: th, hi: th}, true
	}
}

// diskCand is a candidate inscribed disk. rBound is the PROVEN bound its own
// generator derived — zero only where that generator's arithmetic is exact
// (a literal coordinate difference, never a division or a square root of a
// non-perfect value).
type diskCand struct{ x, y, r, rBound float64 }

// wallKernel is one 2D section survey: elements (material on the left),
// junction vertices, the draft allowance, and — for a partial revolve — the
// wedge the sweep caps cut, encoded by wedgeS = sin(min(Δφ/2, π/2)): a ball
// of radius r centered at height y fits the wedge iff y·wedgeS ≥ r.
//
// wedgeS is one of the two kernel inputs that are NOT exact leaves (the file
// comment names both; the other is an element's own radius): it is a sine, so
// its caller (survey.go's revolveWall) proves it from a certified trig
// interval and hands it over as a boundedScalar. Every wedge-derived radius
// composes that bound through the same bounded arithmetic the rest of the file
// uses, so a candidate the wedge produced publishes an interval that contains
// the sine's own error.
type wallKernel struct {
	elems       []surveyElem
	containOnly []surveyElem // boundary for containment only (on-axis chords)
	verts       [][2]float64
	alpha       float64
	wedgeS      boundedScalar // value 0 = no wedge
	wedgeSpans  bool          // two wedge-cap contacts count as a spanning pair
	fitMax      float64       // spanning disks wider than this cannot lift to 3D
	scale, tol  float64
	subTolFar   bool         // a sub-tolerance candidate away from every junction was dropped
	boundary    []surveyElem // elems + containOnly, built lazily for contains
}

// newWallKernel sizes the tolerances from the geometry with the default draft allowance.
func newWallKernel(elems []surveyElem, verts [][2]float64, fitMax float64) *wallKernel {
	k, _ := newWallKernelBudget(nil, elems, nil, verts, 15*math.Pi/180, exactScalar(0), false, fitMax)
	return k
}

// newWallKernelBudget sizes the tolerances from the geometry while charging
// the boundary scan to the caller's shared budget.
func newWallKernelBudget(budget *workBudget, elems, containOnly []surveyElem, verts [][2]float64, alpha float64, wedgeS boundedScalar, wedgeSpans bool, fitMax float64) (*wallKernel, error) {
	if err := wallBudgetErr(budget); err != nil {
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
	for _, boundary := range [][]surveyElem{elems, containOnly} {
		for _, e := range boundary {
			if err := wallBudgetStep(budget); err != nil {
				return nil, err
			}
			if e.kind == surveyLine {
				grow(e.ax, e.ay, e.bx, e.by)
				continue
			}
			grow(e.qx-e.rr, e.qx+e.rr, e.qy-e.rr, e.qy+e.rr)
		}
	}
	return &wallKernel{
		elems:       elems,
		containOnly: containOnly,
		verts:       verts,
		alpha:       alpha,
		wedgeS:      wedgeS,
		wedgeSpans:  wedgeSpans,
		fitMax:      fitMax,
		scale:       scale,
		tol:         1e-9 * scale,
	}, nil
}

// wallSurveyOut is the kernel's answer over its candidate set. spanBound is
// the winning span candidate's own bound and maxCandBound the largest bound
// over every spanning, empty candidate the search admitted (the same
// population out.span's own minimum is taken over). Both are bounds on the
// spanning DIAMETER, the same quantity span reports, because runBudget
// doubles each candidate's radius bound as it doubles the radius itself.
// With m̂ = min v̂ᵢ and
// each true vᵢ ∈ [v̂ᵢ − bᵢ, v̂ᵢ + bᵢ], the true minimum is at most m̂ + b_winner
// (it is at most the winning candidate's own true value) and at least
// m̂ − maxᵢ bᵢ (every candidate's true value, the winner included, is at
// least v̂ᵢ − bᵢ ≥ m̂ − maxᵢ bᵢ), so only the larger of the two radii is a
// SYMMETRIC bound around m̂ — max(spanBound, maxCandBound) below.
type wallSurveyOut struct {
	ok           bool    // false: the survey could not decide (never a silent pass)
	hasSpan      bool    // some empty spanning disk exists
	span         float64 // the smallest spanning diameter found
	spanBound    float64 // the winning span candidate's own bound, on that DIAMETER
	maxCandBound float64 // the largest such DIAMETER bound over every spanning, empty candidate
	inradius     float64 // the largest empty disk radius found (the 2D inradius)
	subTolFar    bool    // an off-junction sub-tolerance candidate was dropped
}

var errWallSurveyUndecided = errors.New("wall survey undecided")

// wallBudgetStep counts one unit of wall work when Verify supplied a budget.
// The context-free shell-admission caller uses nil.
func wallBudgetStep(budget *workBudget) error {
	if budget == nil {
		return nil
	}
	return budget.step()
}

func wallBudgetErr(budget *workBudget) error {
	if budget == nil {
		return nil
	}
	return budget.err()
}

// run preserves the context-free kernel API used by shell admission.
func (k *wallKernel) run() wallSurveyOut {
	out, _ := k.runBudget(nil)
	return out
}

// runBudget streams and validates candidates under one shared Verify budget.
// A candidate whose own generator could not derive a finite bound never
// widens the published interval silently: it makes the whole survey
// undecided instead (the same refusal prismWall already takes for a
// displaced section), since a reading with no proven bound is not one this
// evaluator may stand behind.
func (k *wallKernel) runBudget(budget *workBudget) (wallSurveyOut, error) {
	out := wallSurveyOut{ok: true, span: math.Inf(1)}
	undecided := false
	err := k.generate(budget, func(c diskCand) error {
		spanning, empty, ok, err := k.validate(c, budget)
		if err != nil {
			return err
		}
		if !ok {
			return errWallSurveyUndecided
		}
		if !empty {
			return nil
		}
		if c.r > out.inradius {
			out.inradius = c.r
		}
		if spanning && 2*c.r <= k.fitMax+k.tol {
			// The candidate's generator bounded its RADIUS; the answer is the
			// DIAMETER 2r, so the bound doubles with the value. Multiplying a
			// float64 by two only scales the exponent, so the doubled bound is
			// exact and needs no outward rounding — the sole failure is an
			// overflow to +Inf, which the finiteness gate below catches with
			// every other underivable bound.
			dBound := 2 * c.rBound
			if isNonFinite(dBound) {
				undecided = true
				return nil
			}
			out.hasSpan = true
			if dBound > out.maxCandBound {
				out.maxCandBound = dBound
			}
			if 2*c.r < out.span {
				out.span = 2 * c.r
				out.spanBound = dBound
			}
		}
		return nil
	})
	if errors.Is(err, errWallSurveyUndecided) {
		return wallSurveyOut{}, nil
	}
	if err != nil {
		return wallSurveyOut{}, err
	}
	if undecided {
		return wallSurveyOut{}, nil
	}
	out.subTolFar = k.subTolFar
	return out, nil
}

// validate checks one candidate disk against the whole boundary: it must fit
// the wedge (if any), sit inside the material, and clear every element; its
// contact set decides whether it spans — two contact directions within the
// allowance of antipodal (inclusive), a single whole-arc contact wide
// enough, or (when the wedge's two cap contacts span) an active wedge.
func (k *wallKernel) validate(c diskCand, budget *workBudget) (bool, bool, bool, error) {
	if err := wallBudgetStep(budget); err != nil {
		return false, false, false, err
	}
	if math.IsNaN(c.x) || math.IsNaN(c.y) || math.IsNaN(c.r) || math.IsInf(c.r, 0) {
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
	if c.r <= 4*k.tol {
		reach := c.r + 8*k.tol
		for _, v := range k.verts {
			if err := wallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			if math.Hypot(c.x-v[0], c.y-v[1]) <= reach {
				return false, false, true, nil
			}
		}
		// Not junction-owned: remember it. The caller decides — an exact
		// zero elsewhere still stands (nothing sits below zero), but a
		// positive reading or an absence claim would rest on a candidate
		// the kernel could not resolve, and reads undecided instead.
		k.subTolFar = true
		return false, false, true, nil
	}
	wedgeActive := false
	if k.hasWedge() {
		// A fit/contact test against k.tol, not a published reading: the sine's
		// own bound reaches the answer through the candidate radii the wedge
		// generates. The FIT half still refuses only what the sine's own
		// interval proves does not fit, so a disk the wedge might admit is
		// never discarded on a held product. The CONTACT half below stays a
		// held comparison: its slack is 8·k.tol = 8e-9 of the section's own
		// scale while the product's bound is that scale times the sine's
		// last-ulp figure, so the interval cannot reach across the slack and
		// the straddle is unreachable.
		room := boundedMul(exactScalar(c.y), k.wedgeS)
		if admitBelow(room, c.r-k.tol) == survAdmit {
			return false, false, true, nil
		}
		wedgeActive = room.value <= c.r+k.contactTol()
	}
	var contacts []dirArc
	for _, e := range k.elems {
		if err := wallBudgetStep(budget); err != nil {
			return false, false, false, err
		}
		d, dir, dirOK := e.nearest(c.x, c.y, survTiny*k.scale)
		if d < c.r-k.tol {
			return false, false, true, nil
		}
		if d <= c.r+k.contactTol() {
			if !dirOK {
				// A contact whose direction is undefined: the disk is
				// degenerate against this element; do not decide on it.
				return false, false, true, nil
			}
			contacts = append(contacts, dir)
		}
	}
	inside, ok, err := k.contains(c.x, c.y, budget)
	if err != nil {
		return false, false, false, err
	}
	if !ok {
		return false, false, false, nil
	}
	if !inside {
		return false, false, true, nil
	}
	spanning := k.wedgeSpans && wedgeActive
	need := math.Pi - k.alpha - survAngTol
	for i := 0; i < len(contacts) && !spanning; i++ {
		for j := i; j < len(contacts); j++ {
			if err := wallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			if maxAngleBetween(contacts[i], contacts[j]) >= need {
				spanning = true
				break
			}
		}
	}
	return spanning, true, true, nil
}

// contactTol is the slack for counting an element as a contact.
func (k *wallKernel) contactTol() float64 { return 8 * k.tol }

// hasWedge reports whether this kernel carries a partial revolve's cap wedge at
// all. It is a STRUCTURAL question, not a numeric one: a full turn's caller
// states exactScalar(0), a proven zero, and a partial sweep's caller
// (survey.go's revolveWall) proves its own half-angle sine positive before
// handing it over — so the reading is never taken on a sine that cannot be
// told from zero, and the survAdmit answer below is the only one a built
// kernel reaches.
func (k *wallKernel) hasWedge() bool { return admitAbove(k.wedgeS, 0) == survAdmit }

// contains is the material-membership test: crossing parity of a ray against
// every boundary element, retried across directions when a crossing is
// ambiguous (near an endpoint, near tangency, or grazing the start). All
// directions ambiguous → undecided, never guessed.
func (k *wallKernel) contains(px, py float64, budget *workBudget) (bool, bool, error) {
	if k.boundary == nil {
		boundary := make([]surveyElem, 0, len(k.elems)+len(k.containOnly))
		for _, elems := range [][]surveyElem{k.elems, k.containOnly} {
			for _, e := range elems {
				if err := wallBudgetStep(budget); err != nil {
					return false, false, err
				}
				boundary = append(boundary, e)
			}
		}
		k.boundary = boundary
	}
	all := k.boundary
	for i := range 16 {
		if err := wallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range all {
			if err := wallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, good := rayCrossings(e, px, py, dx, dy, k.tol)
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

// rayCrossings counts proper crossings of the ray p + t·d (t > 0) with one
// element; good is false when a crossing is too ambiguous to count.
func rayCrossings(e surveyElem, px, py, dx, dy, tol float64) (int, bool) {
	if e.kind == surveyLine {
		ex, ey := e.bx-e.ax, e.by-e.ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) - (-ex)*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			// Parallel: an overlap along the ray line is ambiguous; a clean
			// miss (the segment's line offset from the ray's) is a zero.
			if math.Abs((e.ax-px)*dy-(e.ay-py)*dx) <= tol {
				return 0, false
			}
			return 0, true
		}
		rx, ry := e.ax-px, e.ay-py
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
	fx, fy := px-e.qx, py-e.qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.rr*e.rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.rr {
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
		th := math.Atan2(y-e.qy, x-e.qx)
		if e.closed {
			n++
			continue
		}
		lo, hi := e.arcRange()
		off := mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.rr
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
func (k *wallKernel) generate(budget *workBudget, visit func(diskCand) error) error {
	var visitErr error
	add := func(x, y, r, rBound float64) {
		if visitErr == nil {
			visitErr = visit(diskCand{x: x, y: y, r: r, rBound: rBound})
		}
	}

	// Concentric whole-arc disks: the element's own radius, under the
	// element's own bound on it — zero for a stated or recorded radius, the
	// hypot bracket for an ArcSeg-derived one.
	for _, e := range k.elems {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if e.kind == surveyArc && e.matInside {
			add(e.qx, e.qy, e.rr, e.rrBound)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Pair criticals and angle limits.
	for i := range k.elems {
		for j := i + 1; j < len(k.elems); j++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			k.pairCands(k.elems[i], k.elems[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for _, e := range k.elems {
		for _, v := range k.verts {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			k.vertexElemCands(v, e, add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for i := range k.verts {
		for j := i + 1; j < len(k.verts); j++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			k.vertexVertexCands(k.verts[i], k.verts[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Wedge-tangent minima (partial revolve only).
	if k.hasWedge() {
		if err := k.wedgeCands(budget, visit); err != nil {
			return err
		}
	}

	// Apollonius triples, the wedge included as an item.
	eqs, err := k.tripleEquations(budget)
	if err != nil {
		return err
	}
	for i := range eqs {
		for j := i + 1; j < len(eqs); j++ {
			for l := j + 1; l < len(eqs); l++ {
				if err := wallBudgetStep(budget); err != nil {
					return err
				}
				solveTriple([3]circEq{eqs[i], eqs[j], eqs[l]}, k.scale, add)
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
func (k *wallKernel) pairCands(a, b surveyElem, add func(x, y, r, rBound float64)) {
	switch {
	case a.kind == surveyLine && b.kind == surveyLine:
		k.lineLineCands(a, b, add)
	case a.kind == surveyArc && b.kind == surveyArc:
		k.arcArcCands(a, b, add)
	case a.kind == surveyLine:
		k.lineArcCands(a, b, add)
	default:
		k.lineArcCands(b, a, add)
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
func (k *wallKernel) lineLineCands(a, b surveyElem, add func(x, y, r, rBound float64)) {
	cross := a.nx*b.ny - a.ny*b.nx
	if math.Abs(cross) > 1e-9 {
		return
	}
	if a.nx*b.nx+a.ny*b.ny > -1+1e-9 {
		return // same-facing parallels never oppose across material
	}
	dBS := boundedAdd(
		boundedMul(exactScalar(a.nx), boundedSub(exactScalar(b.ax), exactScalar(a.ax))),
		boundedMul(exactScalar(a.ny), boundedSub(exactScalar(b.ay), exactScalar(a.ay))),
	)
	d := dBS.value
	if admitAbove(dBS, 0) == survReject {
		return // b is PROVEN not on a's material side
	}
	// Overlap of the two segments along a's tangent.
	tx, ty := -a.ny, a.nx
	a0, a1 := tx*a.ax+ty*a.ay, tx*a.bx+ty*a.by
	b0, b1 := tx*b.ax+ty*b.ay, tx*b.bx+ty*b.by
	lo := math.Max(math.Min(a0, a1), math.Min(b0, b1))
	hi := math.Min(math.Max(a0, a1), math.Max(b0, b1))
	if lo > hi {
		return
	}
	m := (lo + hi) / 2
	// Base point on a at parameter m, then half-way toward b.
	base := tx*a.ax + ty*a.ay
	px := a.ax + (m-base)*tx + a.nx*d/2
	py := a.ay + (m-base)*ty + a.ny*d/2
	rBS := boundedQuotient(d, dBS.bound, 2, 0)
	add(px, py, d/2, rBS.bound)
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
func (k *wallKernel) lineArcCands(l, a surveyElem, add func(x, y, r, rBound float64)) {
	s := a.matSign()
	e := l.nx*(a.qx-l.ax) + l.ny*(a.qy-l.ay) // signed height of q, material side positive
	eBS := boundedAdd(
		boundedMul(exactScalar(l.nx), boundedSub(exactScalar(a.qx), exactScalar(l.ax))),
		boundedMul(exactScalar(l.ny), boundedSub(exactScalar(a.qy), exactScalar(l.ay))),
	)
	fx := a.qx - e*l.nx
	fy := a.qy - e*l.ny
	// T2 on the perpendicular from q to the line, t = r, center at f + t·n̂:
	// |e − t| = R − s·t, enumerated as e − t = sgn·(R − s·t).
	for _, sgn := range []float64{1, -1} {
		den := sgn*s - 1
		if admitMagnitudeAbove(exactScalar(den), survTiny) == survReject {
			continue
		}
		t := (sgn*a.rr - e) / den
		numBS := boundedSub(boundedMul(exactScalar(sgn), a.rrBS()), eBS)
		tBS := boundedQuotient(numBS.value, numBS.bound, den, 0)
		if admitAbove(tBS, 0) != survReject {
			add(fx+t*l.nx, fy+t*l.ny, t, tBS.bound)
		}
	}
	// T3: contact directions exactly π − α apart. The line contact direction
	// is −n̂; the arc contact direction is s·(ĉ−q̂); c = q + (s·R − r)·u2.
	aStar := math.Pi - k.alpha
	for _, rot := range []float64{aStar, -aStar} {
		snBS, csBS := radianTrigBounds(rot)
		cs, sn := csBS.value, snBS.value
		// u2 = rotate(−n̂, rot)
		ux := -(l.nx*cs - l.ny*sn)
		uy := -(l.nx*sn + l.ny*cs)
		uxBS := boundedMul(exactScalar(-1), boundedSub(boundedMul(exactScalar(l.nx), csBS), boundedMul(exactScalar(l.ny), snBS)))
		uyBS := boundedMul(exactScalar(-1), boundedAdd(boundedMul(exactScalar(l.nx), snBS), boundedMul(exactScalar(l.ny), csBS)))
		ndot := l.nx*ux + l.ny*uy // = −cos(aStar)
		ndotBS := boundedAdd(boundedMul(exactScalar(l.nx), uxBS), boundedMul(exactScalar(l.ny), uyBS))
		denBS := boundedAdd(exactScalar(1), ndotBS)
		if admitMagnitudeAbove(denBS, survTiny) == survReject {
			continue
		}
		den := denBS.value
		r := (l.nx*(a.qx-l.ax) + l.ny*(a.qy-l.ay) + s*a.rr*ndot) / den
		numBS := boundedAdd(eBS, boundedMul(boundedMul(exactScalar(s), a.rrBS()), ndotBS))
		rBS := boundedQuotient(numBS.value, numBS.bound, denBS.value, denBS.bound)
		if admitAbove(rBS, 0) != survReject {
			add(a.qx+(s*a.rr-r)*ux, a.qy+(s*a.rr-r)*uy, r, rBS.bound)
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
func (k *wallKernel) arcArcCands(a, b surveyElem, add func(x, y, r, rBound float64)) {
	dx, dy := b.qx-a.qx, b.qy-a.qy
	dBS := boundedHypot(dx, dy)
	d := dBS.value
	sa, sb := a.matSign(), b.matSign()
	if admitAbove(dBS, survTiny*k.scale) != survAdmit {
		// Concentric: the family disk at each arc's own angular midpoint. The
		// annulus half-width is |Ra − Rb|/2, and NEITHER step is exact — the
		// difference of two radii rounds outside the Sterbenz range, and each
		// radius carries whatever bound its own element states — so the whole
		// chain runs through the bounded arithmetic. The halving is exact, but
		// boundedQuotient carries the difference's bound through it.
		diffBS := boundedAbs(boundedSub(a.rrBS(), b.rrBS()))
		rBS := boundedQuotient(diffBS.value, diffBS.bound, 2, 0)
		r := rBS.value
		m := (a.rr + b.rr) / 2
		for _, e := range []surveyElem{a, b} {
			lo, hi := e.arcRange()
			th := (lo + hi) / 2
			add(a.qx+m*math.Cos(th), a.qy+m*math.Sin(th), r, rBS.bound)
		}
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the centerline: εa(Ra − sa·r) + εb(Rb − sb·r) = d.
	for _, ea := range []float64{1, -1} {
		for _, eb := range []float64{1, -1} {
			den := -ea*sa - eb*sb
			if admitMagnitudeAbove(exactScalar(den), survTiny) == survReject {
				continue
			}
			r := (d - ea*a.rr - eb*b.rr) / den
			numBS := boundedSub(dBS, boundedAdd(
				boundedMul(exactScalar(ea), a.rrBS()),
				boundedMul(exactScalar(eb), b.rrBS()),
			))
			rBS := boundedQuotient(numBS.value, numBS.bound, den, 0)
			if admitAbove(rBS, 0) == survReject {
				continue
			}
			t := ea * (a.rr - sa*r)
			add(a.qx+t*ux, a.qy+t*uy, r, rBS.bound)
		}
	}
	// T3: angle at the center between (c−qa) and (c−qb) fixed by the
	// allowance boundary; law of cosines in r, then the two mirror centers.
	_, cosAStarBS := radianTrigBounds(math.Pi - k.alpha)
	cosThBS := boundedMul(exactScalar(sa*sb), cosAStarBS)
	// d² = Da² + Db² − 2·Da·Db·cosθ with Da = Ra − sa·r, Db = Rb − sb·r.
	raBS, rbBS := a.rrBS(), b.rrBS()
	ABS := boundedSub(exactScalar(2), boundedMul(exactScalar(2*sa*sb), cosThBS))
	BBS := boundedAdd(
		boundedAdd(boundedMul(exactScalar(-2*sa), raBS), boundedMul(exactScalar(-2*sb), rbBS)),
		boundedMul(boundedMul(exactScalar(2), boundedAdd(
			boundedMul(exactScalar(sb), raBS),
			boundedMul(exactScalar(sa), rbBS),
		)), cosThBS),
	)
	CBS := boundedSub(
		boundedAdd(
			boundedAdd(boundedMul(raBS, raBS), boundedMul(rbBS, rbBS)),
			boundedMul(boundedMul(exactScalar(-2), boundedMul(raBS, rbBS)), cosThBS),
		),
		boundedMul(dBS, dBS),
	)
	for _, rBS := range quadRootsBounded(ABS, BBS, CBS) {
		if admitAbove(rBS, 0) == survReject {
			continue
		}
		r := rBS.value
		da := a.rr - sa*r
		db := b.rr - sb*r
		placeCircleCircle(a.qx, a.qy, da, b.qx, b.qy, db, add, r, rBS.bound)
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
func (k *wallKernel) vertexElemCands(v [2]float64, e surveyElem, add func(x, y, r, rBound float64)) {
	aStar := math.Pi - k.alpha
	if e.kind == surveyLine {
		// T2: the foot midpoint.
		h := e.nx*(v[0]-e.ax) + e.ny*(v[1]-e.ay)
		hBS := boundedAdd(
			boundedMul(exactScalar(e.nx), boundedSub(exactScalar(v[0]), exactScalar(e.ax))),
			boundedMul(exactScalar(e.ny), boundedSub(exactScalar(v[1]), exactScalar(e.ay))),
		)
		if admitAbove(hBS, 0) != survReject {
			rBS := boundedQuotient(hBS.value, hBS.bound, 2, 0)
			add(v[0]-h/2*e.nx, v[1]-h/2*e.ny, h/2, rBS.bound)
		}
		// T3: u_v = rotate(−n̂, ±A*), c = v − r·u_v, tangency fixes r.
		for _, rot := range []float64{aStar, -aStar} {
			snBS, csBS := radianTrigBounds(rot)
			cs, sn := csBS.value, snBS.value
			ux := -(e.nx*cs - e.ny*sn)
			uy := -(e.nx*sn + e.ny*cs)
			uxBS := boundedMul(exactScalar(-1), boundedSub(boundedMul(exactScalar(e.nx), csBS), boundedMul(exactScalar(e.ny), snBS)))
			uyBS := boundedMul(exactScalar(-1), boundedAdd(boundedMul(exactScalar(e.nx), snBS), boundedMul(exactScalar(e.ny), csBS)))
			den := 1 + (e.nx*ux + e.ny*uy)
			denBS := boundedAdd(exactScalar(1), boundedAdd(boundedMul(exactScalar(e.nx), uxBS), boundedMul(exactScalar(e.ny), uyBS)))
			if admitMagnitudeAbove(denBS, survTiny) == survReject {
				continue
			}
			r := (e.nx*(v[0]-e.ax) + e.ny*(v[1]-e.ay)) / den
			rBS := boundedQuotient(hBS.value, hBS.bound, denBS.value, denBS.bound)
			if admitAbove(rBS, 0) != survReject {
				add(v[0]-r*ux, v[1]-r*uy, r, rBS.bound)
			}
		}
		return
	}
	s := e.matSign()
	dx, dy := e.qx-v[0], e.qy-v[1]
	dBS := boundedHypot(dx, dy)
	d := dBS.value
	if admitAbove(dBS, survTiny*k.scale) != survAdmit {
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the line through v and q: |c−v| = r with c = v ± r·û, and
	// |c−q| = R − s·r.
	for _, ev := range []float64{1, -1} {
		// c = v + ev·r·û: |c − q| = |d − ev·r| = R − s·r.
		for _, sgn := range []float64{1, -1} {
			den := sgn*s - ev
			if admitMagnitudeAbove(exactScalar(den), survTiny) == survReject {
				continue
			}
			r := (sgn*e.rr - d) / den
			numBS := boundedSub(boundedMul(exactScalar(sgn), e.rrBS()), dBS)
			rBS := boundedQuotient(numBS.value, numBS.bound, den, 0)
			if admitAbove(rBS, 0) != survReject {
				add(v[0]+ev*r*ux, v[1]+ev*r*uy, r, rBS.bound)
			}
		}
	}
	// T3: law of cosines with sides r and R − s·r.
	_, cosAStarBS := radianTrigBounds(aStar)
	cosThBS := boundedMul(exactScalar(-s), cosAStarBS)
	ABS := boundedAdd(exactScalar(2), boundedMul(exactScalar(2*s), cosThBS))
	reBS := e.rrBS()
	BBS := boundedMul(boundedMul(exactScalar(-2), reBS), boundedAdd(exactScalar(s), cosThBS))
	CBS := boundedSub(boundedMul(reBS, reBS), boundedMul(dBS, dBS))
	for _, rBS := range quadRootsBounded(ABS, BBS, CBS) {
		if admitAbove(rBS, 0) == survReject {
			continue
		}
		r := rBS.value
		placeCircleCircle(v[0], v[1], r, e.qx, e.qy, e.rr-s*r, add, r, rBS.bound)
	}
}

// vertexVertexCands: the midpoint disk (T2) and the isoceles angle-limit
// disks (T3).
//
// Two coincident vertices give an exactly zero separation with an exactly zero
// bound, so the degeneracy reading decides them. The T3 denominator is
// 2·(1 − cos(π − α)) = 2·(1 + cos α) for the caller's own draft allowance, and
// vanishes only at α = π — an allowance of a half turn, which is no draft.
func (k *wallKernel) vertexVertexCands(a, b [2]float64, add func(x, y, r, rBound float64)) {
	dx, dy := b[0]-a[0], b[1]-a[1]
	dBS := boundedHypot(dx, dy)
	d := dBS.value
	if admitAbove(dBS, survTiny*k.scale) != survAdmit {
		return
	}
	rBS := boundedQuotient(dBS.value, dBS.bound, 2, 0)
	add((a[0]+b[0])/2, (a[1]+b[1])/2, d/2, rBS.bound)
	_, cosAStarBS := radianTrigBounds(math.Pi - k.alpha)
	denBS := boundedMul(exactScalar(2), boundedSub(exactScalar(1), cosAStarBS))
	if admitAbove(denBS, survTiny) != survReject {
		sqrtDenBS := boundedSqrt(denBS)
		rr := boundedQuotient(dBS.value, dBS.bound, sqrtDenBS.value, sqrtDenBS.bound)
		placeCircleCircle(a[0], a[1], rr.value, b[0], b[1], rr.value, add, rr.value, rr.bound)
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
func (k *wallKernel) wedgeCands(budget *workBudget, visit func(diskCand) error) error {
	sBS := k.wedgeS
	s := sBS.value
	for _, v := range k.verts {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if v[1] <= 0 {
			continue
		}
		for _, sgn := range []float64{1, -1} {
			denBS := boundedSub(exactScalar(1), boundedMul(exactScalar(sgn), sBS))
			if admitAbove(denBS, survTiny) == survReject {
				continue
			}
			numBS := boundedMul(sBS, exactScalar(v[1]))
			rBS := boundedQuotient(numBS.value, numBS.bound, denBS.value, denBS.bound)
			r := rBS.value
			if admitAbove(rBS, 0) != survReject {
				if err := visit(diskCand{x: v[0], y: r / s, r: r, rBound: rBS.bound}); err != nil {
					return err
				}
			}
		}
	}
	for _, e := range k.elems {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		if e.kind != surveyArc {
			continue // a line's wedge-tangent family has no interior minimum
		}
		se := e.matSign()
		for _, th := range []float64{math.Pi / 2, -math.Pi / 2} {
			if !e.arcContains(th) {
				continue
			}
			sinBS, _ := radianTrigBounds(th)
			numBS := boundedMul(sBS, boundedAdd(exactScalar(e.qy), boundedMul(e.rrBS(), sinBS)))
			denBS := boundedAdd(exactScalar(1), boundedMul(boundedMul(sBS, exactScalar(se)), sinBS))
			if admitMagnitudeAbove(denBS, survTiny) == survReject {
				continue
			}
			rBS := boundedQuotient(numBS.value, numBS.bound, denBS.value, denBS.bound)
			if admitAbove(rBS, 0) == survReject {
				continue
			}
			r := rBS.value
			d := e.rr - se*r
			if err := visit(diskCand{x: e.qx + d*math.Cos(th), y: e.qy + d*math.Sin(th), r: r, rBound: rBS.bound}); err != nil {
				return err
			}
		}
	}
	return nil
}

// circEq is one tangency equation for the Apollonius triples: either linear
// (a·cx + b·cy + e·r + f = 0) or quadratic
// (cx² + cy² − r² + g·cx + h·cy + kk·r + m = 0). Every coefficient is a
// boundedScalar, not a bare float: the element, vertex and wedge coordinates
// the equations are built from are exact leaves, but forming a coefficient
// from them rounds (a product of two coordinates, a sum of three such
// products); an arc element's radius is not even a leaf, since it arrives with
// its own bound (surveyElem.rrBound); and the wedge's own coefficient is a
// sine that arrives already bounded.
// solveTriple/solve3Linear/solveParallelPair compose those bounds through
// every determinant, numerator and division they perform, so a triple's
// published radius interval speaks for the WHOLE chain from coefficient
// construction down to the final quotient.
type circEq struct {
	quad        bool
	g, h, kk, m boundedScalar
	a, b, e, f  boundedScalar
}

// tripleEquations builds the material-side-pinned tangency equation of every
// item: elements, vertices, and the wedge.
func (k *wallKernel) tripleEquations(budget *workBudget) ([]circEq, error) {
	var eqs []circEq
	for _, el := range k.elems {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		if el.kind == surveyLine {
			nx, ny := exactScalar(el.nx), exactScalar(el.ny)
			eqs = append(eqs, circEq{
				a: nx, b: ny, e: exactScalar(-1),
				f: boundedNeg(boundedAdd(
					boundedMul(nx, exactScalar(el.ax)),
					boundedMul(ny, exactScalar(el.ay)),
				)),
			})
			continue
		}
		s := el.matSign()
		qx, qy, rr := exactScalar(el.qx), exactScalar(el.qy), el.rrBS()
		eqs = append(eqs, circEq{
			quad: true,
			g:    boundedMul(exactScalar(-2), qx),
			h:    boundedMul(exactScalar(-2), qy),
			kk:   boundedMul(exactScalar(2*s), rr),
			m: boundedSub(
				boundedAdd(boundedMul(qx, qx), boundedMul(qy, qy)),
				boundedMul(rr, rr),
			),
		})
	}
	for _, v := range k.verts {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		vx, vy := exactScalar(v[0]), exactScalar(v[1])
		eqs = append(eqs, circEq{
			quad: true,
			g:    boundedMul(exactScalar(-2), vx),
			h:    boundedMul(exactScalar(-2), vy),
			m:    boundedAdd(boundedMul(vx, vx), boundedMul(vy, vy)),
		})
	}
	if k.hasWedge() {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		// The wedge's coefficient is the caller's already-bounded sine, so a
		// triple that includes the wedge inherits the sine's own error.
		eqs = append(eqs, circEq{b: k.wedgeS, e: exactScalar(-1)})
	}
	return eqs, nil
}

// solveTriple solves one triple of tangency equations in closed form:
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
func solveTriple(eqs [3]circEq, scale float64, add func(x, y, r, rBound float64)) {
	var lins []circEq
	var quad *circEq
	for i, e := range eqs {
		if !e.quad {
			lins = append(lins, e)
			continue
		}
		if quad == nil {
			q := eqs[i]
			quad = &q
			continue
		}
		// Subtract: the c·c − r² terms cancel.
		lins = append(lins, circEq{
			a: boundedSub(e.g, quad.g), b: boundedSub(e.h, quad.h),
			e: boundedSub(e.kk, quad.kk), f: boundedSub(e.m, quad.m),
		})
	}
	if len(lins) == 3 {
		solve3Linear(lins, add)
		return
	}
	if len(lins) != 2 || quad == nil {
		return
	}
	l1, l2 := lins[0], lins[1]
	detBS := boundedSub(boundedMul(l1.a, l2.b), boundedMul(l2.a, l1.b))
	parallel := admitMagnitudeAbove(detBS, 1e-12*math.Max(1, scale))
	if parallel != survAdmit {
		solveParallelPair(l1, l2, *quad, add)
		if parallel == survReject {
			return
		}
	}
	// (cx, cy) = P + r·Q from the two linears. The triple's OWN division (by
	// det) composes through boundedQuotient like every other division here, so
	// P and Q reach the quadratic below carrying their own error rather than
	// as exact leaves.
	pxBS := boundedDiv(boundedAdd(boundedMul(boundedNeg(l1.f), l2.b), boundedMul(l2.f, l1.b)), detBS)
	pyBS := boundedDiv(boundedAdd(boundedMul(boundedNeg(l1.a), l2.f), boundedMul(l2.a, l1.f)), detBS)
	qxBS := boundedDiv(boundedAdd(boundedMul(boundedNeg(l1.e), l2.b), boundedMul(l2.e, l1.b)), detBS)
	qyBS := boundedDiv(boundedAdd(boundedMul(boundedNeg(l1.a), l2.e), boundedMul(l2.a, l1.e)), detBS)
	ABS := boundedSub(boundedAdd(boundedMul(qxBS, qxBS), boundedMul(qyBS, qyBS)), exactScalar(1))
	BBS := boundedAdd(
		boundedMul(exactScalar(2), boundedAdd(boundedMul(pxBS, qxBS), boundedMul(pyBS, qyBS))),
		boundedAdd(boundedAdd(boundedMul(quad.g, qxBS), boundedMul(quad.h, qyBS)), quad.kk),
	)
	CBS := boundedAdd(
		boundedAdd(boundedMul(pxBS, pxBS), boundedMul(pyBS, pyBS)),
		boundedAdd(boundedAdd(boundedMul(quad.g, pxBS), boundedMul(quad.h, pyBS)), quad.m),
	)
	for _, rBS := range quadRootsBounded(ABS, BBS, CBS) {
		r := rBS.value
		if admitAbove(rBS, 0) != survReject {
			add(pxBS.value+r*qxBS.value, pyBS.value+r*qyBS.value, r, rBS.bound)
		}
	}
}

// solve3Linear solves three linear tangency equations by Cramer's rule. Every
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
// solveParallelPair, which is what those generators are for. Emitting instead
// would publish a position and radius Cramer's rule divides out of a vanishing
// denominator, whose own +Inf bound would then leave every survey containing
// such a triple undecided — which for a rotated or placed section is most of
// them.
func solve3Linear(l []circEq, add func(x, y, r, rBound float64)) {
	// The 2×2 minors of rows 1 and 2 over each column pair, named for the
	// columns they keep.
	minor := func(p, q, s, t boundedScalar) boundedScalar {
		return boundedSub(boundedMul(p, t), boundedMul(s, q))
	}
	be := minor(l[1].b, l[1].e, l[2].b, l[2].e)
	ae := minor(l[1].a, l[1].e, l[2].a, l[2].e)
	ab := minor(l[1].a, l[1].b, l[2].a, l[2].b)
	fe := minor(l[1].f, l[1].e, l[2].f, l[2].e)
	fb := minor(l[1].f, l[1].b, l[2].f, l[2].b)
	af := minor(l[1].a, l[1].f, l[2].a, l[2].f)
	bf := minor(l[1].b, l[1].f, l[2].b, l[2].f)
	detBS := boundedAdd(
		boundedSub(boundedMul(l[0].a, be), boundedMul(l[0].b, ae)),
		boundedMul(l[0].e, ab),
	)
	if admitMagnitudeAbove(detBS, survTiny) != survAdmit {
		return
	}
	drBS := boundedSub(
		boundedAdd(boundedMul(boundedNeg(l[0].a), bf), boundedMul(l[0].b, af)),
		boundedMul(l[0].f, ab),
	)
	rBS := boundedDiv(drBS, detBS)
	if admitAbove(rBS, 0) == survReject {
		return
	}
	// The center is a position, never a published reading (see
	// placeCircleCircle), so it stays a plain quotient.
	dx := boundedSub(
		boundedAdd(boundedMul(boundedNeg(l[0].f), be), boundedMul(l[0].b, fe)),
		boundedMul(l[0].e, fb),
	).value
	dy := boundedSub(
		boundedAdd(boundedMul(boundedNeg(l[0].a), fe), boundedMul(l[0].f, ae)),
		boundedMul(l[0].e, af),
	).value
	add(dx/detBS.value, dy/detBS.value, rBS.value, rBS.bound)
}

// solveParallelPair handles two parallel linear tangency equations plus a
// quadratic: the pair fixes r and a line of centers; the quadratic picks the
// positions along it. The normalization, the 2×2 determinant and the final
// division all compose their bounds, so the radius every emitted position
// shares carries the error of the whole reduction; the positions themselves
// never feed a published reading and stay plain floats.
//
// A normal this rule cannot separate from zero is not a line: the equation
// carries no direction to normalize by, so there is nothing to solve and the
// reading stops, the same way solve3Linear's dependent triple does. The
// orientation sign σ is the one reading here whose straddle is unreachable
// from its own algebra — the two normals are already unit vectors and the
// caller only reaches this function for a pair whose cross product is at or
// below its own parallelism threshold, so their dot product sits within a few
// ulps of ±1 and no interval this arithmetic produces spans zero.
func solveParallelPair(l1, l2, q circEq, add func(x, y, r, rBound float64)) {
	nBS := boundedNorm2(l1.a, l1.b)
	if admitAbove(nBS, survTiny) != survAdmit {
		return
	}
	// Normalize both to unit normals; solve the 2×2 system in (h, r) where
	// h = n̂·c along l1's normal.
	a1, b1 := boundedDiv(l1.a, nBS), boundedDiv(l1.b, nBS)
	e1, f1 := boundedDiv(l1.e, nBS), boundedDiv(l1.f, nBS)
	n2BS := boundedNorm2(l2.a, l2.b)
	if admitAbove(n2BS, survTiny) != survAdmit {
		return
	}
	a2, b2 := boundedDiv(l2.a, n2BS), boundedDiv(l2.b, n2BS)
	e2, f2 := boundedDiv(l2.e, n2BS), boundedDiv(l2.f, n2BS)
	// n̂2 = ±n̂1: sign σ.
	sigma := -1.0
	if admitAbove(boundedAdd(boundedMul(a1, a2), boundedMul(b1, b2)), 0) == survAdmit {
		sigma = 1
	}
	// eq1: h + e1·r + f1 = 0; eq2: σ·h + e2·r + f2 = 0.
	detBS := boundedSub(e2, boundedMul(exactScalar(sigma), e1))
	if admitMagnitudeAbove(detBS, survTiny) == survReject {
		return
	}
	rBS := boundedDiv(boundedSub(boundedMul(exactScalar(sigma), f1), f2), detBS)
	r := rBS.value
	if admitAbove(rBS, 0) == survReject {
		return
	}
	h := -e1.value*r - f1.value
	// Centers: c = h·n̂1 + t·t̂1. Substitute into the quadratic.
	tx, ty := -b1.value, a1.value
	bx, by := h*a1.value, h*b1.value
	A := 1.0
	B := 2*(bx*tx+by*ty) + q.g.value*tx + q.h.value*ty
	C := bx*bx + by*by - r*r + q.g.value*bx + q.h.value*by + q.kk.value*r + q.m.value
	for _, t := range quadRoots(A, B, C) {
		add(bx+t*tx, by+t*ty, r, rBS.bound)
	}
}

// quadRoots returns the real roots of A·x² + B·x + C = 0 (both when A ≈ 0 —
// the linear case — and the full quadratic).
func quadRoots(A, B, C float64) []float64 {
	if math.Abs(A) < survTiny {
		if math.Abs(B) < survTiny {
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

// placeCircleCircle emits the (up to two) points at distance da from
// (ax, ay) and db from (bx, by), as candidate centers of radius r (with its
// own proven bound rBound, unaffected by the position solve below — the
// position never feeds a published reading).
func placeCircleCircle(ax, ay, da, bx, by, db float64, add func(x, y, r, rBound float64), r, rBound float64) {
	dx, dy := bx-ax, by-ay
	d := math.Hypot(dx, dy)
	if d < survTiny || da < 0 || db < 0 {
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
