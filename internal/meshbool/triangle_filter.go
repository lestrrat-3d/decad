package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// FloatInterval is a proven float64 enclosure [lo, hi] of an exact rational
// value: the reject-only pre-filter in front of TriTriClassify's non-coplanar
// arm (TriTriMissesFilter, below) is built entirely on this type, the same
// "outward-rounded float arithmetic, exact fallback for anything it cannot
// prove" shape SegFilter uses for the conforming pass's own segment test. It
// is float64's own enclosure, deliberately apart from proofbound.RatInterval
// (moments.go) and internal/capcontour's Point/Carrier, which enclose in
// big.Rat and serve a different proof (a rational bound, not a float filter).
//
// Every method below returns EITHER a proper enclosure — lo and hi both
// finite, lo ≤ hi, and the true value proven to lie in [lo, hi] — OR the
// sentinel FivAbstain ({-Inf, +Inf}), which encloses every value trivially
// and is this type's uniform "cannot prove anything here" answer. No method
// ever returns a half-finite interval, so a caller need only test one
// sentinel to know whether an answer was reached.
//
// ROUNDING. Each arithmetic method computes its result with plain float64
// operations (round-to-nearest) and then widens the low end down and the high
// end up by exactly one representable value. Round-to-nearest error is at
// most half an ulp of the computed value, so a full ulp of outward widening
// covers it with room to spare — the same margin SegAdmissionRadius2's
// derivation leans on. A binary operation with two interval operands has up
// to four products or quotients at its corners. The result selects their
// extrema before the same one-ulp widening; multiplication uses each
// operand's sign to skip corners that cannot attain an endpoint.
//
// NON-FINITE INPUTS AND OUTPUTS ABSTAIN, NEVER PROPAGATE. Every method checks
// every intermediate it forms for finiteness before trusting it, and returns
// FivAbstain the moment one fails — mirroring SegFilter.tooFar's own
// "saturation is tested, not argued" discipline. A saturated intermediate
// does not merely widen an answer, it can flip which branch a later decision
// takes (Inf compares false against everything finite), so nothing downstream
// may read a value this type has not first proven finite.
type FloatInterval struct{ Lo, Hi float64 }

// FivAbstain is the everywhere-abstaining interval.
var FivAbstain = FloatInterval{Lo: math.Inf(-1), Hi: math.Inf(1)}

// FivNextDown and FivNextUp implement Nextafter for finite inputs. Every
// caller rejects non-finite values before widening them.
func FivNextDown(x float64) float64 {
	if x == 0 {
		return -math.SmallestNonzeroFloat64
	}
	bits := math.Float64bits(x)
	if x > 0 {
		bits--
	} else {
		bits++
	}
	return math.Float64frombits(bits)
}

func FivNextUp(x float64) float64 {
	if x == 0 {
		return math.SmallestNonzeroFloat64
	}
	bits := math.Float64bits(x)
	if x > 0 {
		bits++
	} else {
		bits--
	}
	return math.Float64frombits(bits)
}

// abstains reports whether iv is the abstain sentinel.
func (a FloatInterval) Abstains() bool { return a == FivAbstain }

// contains reports whether x lies within the closed interval.
func (a FloatInterval) Contains(x float64) bool { return a.Lo <= x && x <= a.Hi }

// disjoint reports whether a and b, as closed intervals, share no point.
func (a FloatInterval) Disjoint(b FloatInterval) bool { return a.Hi < b.Lo || b.Hi < a.Lo }

// FivPoint lifts a float64 that is itself an EXACT value (never itself the
// product of a rounding) into a degenerate interval: lo = hi = x needs no
// widening, because there is no rounding error to cover. Every triangle
// vertex coordinate TriTriMissesFilter reads is such a value — a float64 IS
// an exact rational — so this is the
// leaf constructor for every vertex coordinate the filter touches.
func FivPoint(x float64) FloatInterval {
	if proofbound.IsNonFinite(x) {
		return FivAbstain
	}
	return FloatInterval{Lo: x, Hi: x}
}

// FivRounded lifts a float64 that is itself a CORRECTLY-ROUNDED conversion of
// some exact value — at most half an ulp off truth, the guarantee
// big.Rat.Float64 makes — into an interval with one full ulp of margin on
// each side, which covers that half-ulp with the same spare-half-ulp margin
// every other widening in this type carries. TriTriMissesFilter's na/nb
// arguments are proof.Xpt.Vec() results, so they are exactly this case.
func FivRounded(x float64) FloatInterval {
	if proofbound.IsNonFinite(x) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(x), Hi: FivNextUp(x)}
}

func (a FloatInterval) Add(b FloatInterval) FloatInterval {
	lo, hi := a.Lo+b.Lo, a.Hi+b.Hi
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

func (a FloatInterval) Sub(b FloatInterval) FloatInterval {
	lo, hi := a.Lo-b.Hi, a.Hi-b.Lo
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

func (a FloatInterval) Mul(b FloatInterval) FloatInterval {
	var lo, hi float64
	// On sign-definite intervals multiplication is monotone, so only the two
	// corners that attain the range endpoints need to be evaluated.
	switch {
	case a.Lo >= 0 && b.Lo >= 0:
		lo, hi = a.Lo*b.Lo, a.Hi*b.Hi
	case a.Lo >= 0 && b.Hi <= 0:
		lo, hi = a.Hi*b.Lo, a.Lo*b.Hi
	case a.Lo >= 0:
		lo, hi = a.Hi*b.Lo, a.Hi*b.Hi
	case a.Hi <= 0 && b.Lo >= 0:
		lo, hi = a.Lo*b.Hi, a.Hi*b.Lo
	case a.Hi <= 0 && b.Hi <= 0:
		lo, hi = a.Hi*b.Hi, a.Lo*b.Lo
	case a.Hi <= 0:
		lo, hi = a.Lo*b.Hi, a.Lo*b.Lo
	case b.Lo >= 0:
		lo, hi = a.Lo*b.Hi, a.Hi*b.Hi
	case b.Hi <= 0:
		lo, hi = a.Hi*b.Lo, a.Lo*b.Lo
	default:
		lo = math.Min(a.Lo*b.Hi, a.Hi*b.Lo)
		hi = math.Max(a.Lo*b.Lo, a.Hi*b.Hi)
	}
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

// div guards the one case the other three operations do not have: a
// denominator interval containing zero makes the quotient unbounded (or
// undefined, at zero itself), so it returns FivAbstain outright rather than
// forming a division that could saturate or, worse, land on a finite value
// that means nothing.
func (a FloatInterval) Div(b FloatInterval) FloatInterval {
	if proofbound.IsNonFinite(a.Lo) || proofbound.IsNonFinite(a.Hi) || proofbound.IsNonFinite(b.Lo) || proofbound.IsNonFinite(b.Hi) {
		return FivAbstain
	}
	if b.Lo <= 0 && b.Hi >= 0 {
		return FivAbstain
	}
	q0, q1, q2, q3 := a.Lo/b.Lo, a.Lo/b.Hi, a.Hi/b.Lo, a.Hi/b.Hi
	if proofbound.IsNonFinite(q0) || proofbound.IsNonFinite(q1) || proofbound.IsNonFinite(q2) || proofbound.IsNonFinite(q3) {
		return FivAbstain
	}
	lo := math.Min(math.Min(q0, q1), math.Min(q2, q3))
	hi := math.Max(math.Max(q0, q1), math.Max(q2, q3))
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

// FivVec is a 3-vector of floatIntervals: the interval mirror of xpt, over
// float64 rather than big.Rat.
type FivVec struct{ X, Y, Z FloatInterval }

func FivVecOf(v r3.Vec) FivVec { return FivVec{FivPoint(v.X), FivPoint(v.Y), FivPoint(v.Z)} }

func FivSub(a, b FivVec) FivVec { return FivVec{a.X.Sub(b.X), a.Y.Sub(b.Y), a.Z.Sub(b.Z)} }

func FivDot(a, b FivVec) FloatInterval { return a.X.Mul(b.X).Add(a.Y.Mul(b.Y)).Add(a.Z.Mul(b.Z)) }

func FivCross(a, b FivVec) FivVec {
	return FivVec{
		a.Y.Mul(b.Z).Sub(a.Z.Mul(b.Y)),
		a.Z.Mul(b.X).Sub(a.X.Mul(b.Z)),
		a.X.Mul(b.Y).Sub(a.Y.Mul(b.X)),
	}
}

// TriSpanOnLine is TriTriMissesFilter's per-triangle half: a proven [lo, hi]
// enclosure of the range triangle t occupies when its plane crossings against
// the OTHER triangle o are projected onto dir — the float mirror of
// OrderOnLine over PlaneCrossings' own two cases (a zero-sign vertex sits
// exactly on the crossing; a sign-changing edge crosses it at t). signs is
// the caller's own already-decided sign triple (OrientSign, possibly via its
// exact fallback) for t's vertices against o's plane — this filter never
// re-derives a sign, only asks which of PlaneCrossings' branches it selects,
// exactly as the exact code does.
//
// The bound is deliberately a SUPERSET of the true occupied range: it takes
// every candidate PlaneCrossings would have considered — including one a
// wide interval later drops from contention exactly the way dedupePoints or
// the ≤2-points invariant would — and folds in its own projected interval, so
// the returned span can only be wider than the truth, never narrower. A
// wider span makes the filter LESS likely to prove disjointness, never more:
// the reject-only direction is unaffected by the extra slack. ok is false —
// the uniform abstain signal — the moment any step could not be bounded, or
// no candidate was found at all (which AllOneSide's own gate, run before this
// filter is ever called, has already ruled out for a real non-coplanar pair).
func TriSpanOnLine(t, o [3]FivVec, signs [3]int, dir FivVec) (FloatInterval, bool) {
	planeNormal := FivCross(FivSub(o[1], o[0]), FivSub(o[2], o[0]))
	return triSpanOnLineWithNormal(t, o[0], signs, dir, planeNormal)
}

// triSpanOnLineWithNormal uses an enclosure of the other triangle's exact
// normal. The caller already has that normal when classifying a facet pair.
func triSpanOnLineWithNormal(t [3]FivVec, origin FivVec, signs [3]int, dir, planeNormal FivVec) (FloatInterval, bool) {
	lo, hi := math.Inf(1), math.Inf(-1)
	found := false
	values := [3]FloatInterval{}
	valueSet := [3]bool{}
	value := func(i int) FloatInterval {
		if !valueSet[i] {
			values[i] = FivDot(planeNormal, FivSub(t[i], origin))
			valueSet[i] = true
		}
		return values[i]
	}
	widen := func(p FivVec) bool {
		proj := FivDot(p, dir)
		if proj.Abstains() {
			return false
		}
		lo = math.Min(lo, proj.Lo)
		hi = math.Max(hi, proj.Hi)
		found = true
		return true
	}
	for i := range 3 {
		j := (i + 1) % 3
		if signs[i] == 0 && !widen(t[i]) {
			return FloatInterval{}, false
		}
		if signs[i]*signs[j] >= 0 {
			continue
		}
		vi := value(i)
		vj := value(j)
		frac := vi.Div(vi.Sub(vj))
		if frac.Abstains() {
			return FloatInterval{}, false
		}
		p := FivVec{
			X: t[i].X.Add(frac.Mul(t[j].X.Sub(t[i].X))),
			Y: t[i].Y.Add(frac.Mul(t[j].Y.Sub(t[i].Y))),
			Z: t[i].Z.Add(frac.Mul(t[j].Z.Sub(t[i].Z))),
		}
		if !widen(p) {
			return FloatInterval{}, false
		}
	}
	if !found {
		return FloatInterval{}, false
	}
	return FloatInterval{Lo: lo, Hi: hi}, true
}

// TriTriMissesFilter is the reject-only float pre-filter in front of
// TriTriClassify's non-coplanar rational arm (docs/evaluator-design.md §9),
// dispatched ahead of PlaneCrossings for exactly the reason SegFilter sits
// ahead of OnSegmentInterior3: the circular fixture that motivated it enumerates
// 66,008 facet pairs into that arm, and only 1,860 of them (fu158) have any
// real contact, so almost every call pays a full math/big.Rat cross product to
// establish what a dozen float operations already establish — the pair's
// planes cross too far outside both triangles to meet at all.
//
// It reproduces TriTriClassify's own non-coplanar computation — PlaneCrossings
// per triangle, then the projection onto dir = na × nb that OrderOnLine
// compares — entirely in interval arithmetic (TriSpanOnLine, above), and
// returns true — PROVEN no contact — only when the two triangles' projected
// spans are proven disjoint. Every other outcome, abstention included,
// returns false: the pair still goes to the exact predicate, which decides it
// with no help from this filter. That asymmetry is the whole soundness
// argument, in SegFilter's own words: a filter that could ADMIT would be an
// admission gate on a residual, which this package forbids outright, and a
// filter that could reject a true contact would hand back a non-conforming
// subdivision — a WRONG boolean, not a slow one.
//
// ta, tb are the operands' own float corners, read as exact point intervals
// (FivPoint — a float64 vertex coordinate is exact, never itself a
// rounding). na, nb are proof.Xpt.Vec() — the correctly-rounded float64 conversion
// of the pair's exact rational normals — read with FivRounded's extra ulp of
// margin for that rounding. sa, sb are the vertex-against-the-other-plane
// sign triples OrientSign already decided; see TriSpanOnLine for how they
// steer the reconstruction.
//
// THE SHALLOW-ANGLE CASE is where this filter is expected to abstain most
// often, and correctly so: when na and nb are nearly parallel, dir = na × nb
// is near zero, so every FivDot projection carries an interval wide relative
// to its own magnitude, and frac's denominator interval is far more likely to
// straddle zero. Both drive TriSpanOnLine to its abstain return, sending the
// pair to the exact predicate — costing one rational classification, never a
// wrong answer.
func TriTriMissesFilter(ta, tb [3]r3.Vec, na, nb r3.Vec, sa, sb [3]int) bool {
	a := [3]FivVec{FivVecOf(ta[0]), FivVecOf(ta[1]), FivVecOf(ta[2])}
	b := [3]FivVec{FivVecOf(tb[0]), FivVecOf(tb[1]), FivVecOf(tb[2])}
	normalA := FivVec{FivRounded(na.X), FivRounded(na.Y), FivRounded(na.Z)}
	normalB := FivVec{FivRounded(nb.X), FivRounded(nb.Y), FivRounded(nb.Z)}
	dir := FivCross(normalA, normalB)

	spanA, ok := triSpanOnLineWithNormal(a, b[0], sa, dir, normalB)
	if !ok {
		return false
	}
	spanB, ok := triSpanOnLineWithNormal(b, a[0], sb, dir, normalA)
	if !ok {
		return false
	}
	return spanA.Disjoint(spanB)
}
