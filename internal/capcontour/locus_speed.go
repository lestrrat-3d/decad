package capcontour

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// miterConstraintRow is one carrier's own linear constraint on the miter
// locus's velocity P'(t), read at a foot enclosed within the box the
// intersection of the corner's two carriers over the offset range proves
// (carrierOverRange, Intersect, Nearest).
//
// A line carrier's offset satisfies n·X = n·v0 + t for its own fixed unit
// normal n (the construction's own p0(t) = v0 + t·n, and every point of the
// offset LINE shares n's component since the line runs perpendicular to it),
// so differentiating in t gives n·P'(t) = 1 — a row that needs no widening at
// all, since n does not depend on t.
//
// A circle carrier's offset satisfies |X-c| = R - inside·t (offsetRadius's
// own closed form), so differentiating gives û·P'(t) = -inside, with
// û = (X-c)/|X-c| the unit radial direction AT THE FOOT — which does depend
// on t, so it is enclosed from the box foot proves rather than read at one
// point.
func miterConstraintRow(w survey2d.SideWalk, c Carrier, foot Point) (Point, *big.Rat, bool) {
	if c.IsLine {
		n := Point{U: proofbound.IntervalNeg(c.Dir.V), V: c.Dir.U}
		return n, big.NewRat(1, 1), true
	}
	dx := proofbound.IntervalSub(foot.U, c.C.U)
	dy := proofbound.IntervalSub(foot.V, c.C.V)
	distSq := proofbound.IntervalAdd(proofbound.IntervalSquare(dx), proofbound.IntervalSquare(dy))
	dist, ok := proofbound.IntervalSqrt(distSq)
	if !ok || dist.Lo.Sign() <= 0 {
		return Point{}, nil, false
	}
	ux, okU := proofbound.IntervalQuo(dx, dist)
	uy, okV := proofbound.IntervalQuo(dy, dist)
	if !okU || !okV {
		return Point{}, nil, false
	}
	rhs := big.NewRat(-1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		rhs = big.NewRat(1, 1)
	}
	return Point{U: ux, V: uy}, rhs, true
}

// CircleCircleLocusSpeedUpper bounds |dP/dt| for a corner where TWO circular
// walls' own offset carriers meet, by enclosing the corner foot over the
// whole offset range at once (carrierOverRange, Intersect, Nearest)
// and solving the 2x2 linear system miterConstraintRow builds from each
// carrier — Cramer's rule, carried out in interval arithmetic. ok is false
// whenever the determinant's own enclosure straddles zero (the two carriers'
// rows are nearly parallel there) or any enclosure along the way fails.
//
// This decorrelates the two carriers' own offset amount — each is enclosed
// over [t0, t1] independently, rather than tracked as the SAME shared
// parameter through both — which is sound (the true trajectory is always
// inside the resulting box) but loose exactly where a corner is TANGENT: two
// circles tangent at the corner stay tangent under a consistent offset (the
// same identity LineCircleLocusSpeedUpper's own doc comment derives for a
// line and a circle), so the true velocity is finite there, but this
// decorrelated enclosure cannot tell that persistent tangency from a
// momentary one and refuses on both. LineCircleLocusSpeedUpper below carries
// the exact closed form that tells them apart for a line meeting a circle —
// the common case, reached at every tangent-filleted corner in this
// codebase's own test fixtures — and this generic solve is kept for the
// circle-circle miter it does not cover, on the same reject-only footing
// every other refusal here stands on: never a wrong bound, only a
// conservative one at a tangent circle-circle corner.
func CircleCircleLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	ca, okA := carrierOverRange(prev, t0, t1)
	cb, okB := carrierOverRange(cur, t0, t1)
	if !okA || !okB {
		return 0, false
	}
	cands, okI := Intersect(ca, cb)
	if !okI {
		return 0, false
	}
	foot, okN := Nearest(cands, vU, vV)
	if !okN {
		return 0, false
	}
	rowA, rhsA, okRA := miterConstraintRow(prev, ca, foot)
	rowB, rhsB, okRB := miterConstraintRow(cur, cb, foot)
	if !okRA || !okRB {
		return 0, false
	}
	det := proofbound.IntervalSub(proofbound.IntervalMul(rowA.U, rowB.V), proofbound.IntervalMul(rowB.U, rowA.V))
	if det.Lo.Sign() <= 0 && det.Hi.Sign() >= 0 {
		return 0, false
	}
	pu, okU := proofbound.IntervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowB.V, rhsA), proofbound.IntervalScale(rowA.V, rhsB)), det)
	pv, okV := proofbound.IntervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowA.U, rhsB), proofbound.IntervalScale(rowB.U, rhsA)), det)
	if !okU || !okV {
		return 0, false
	}
	mag, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(proofbound.IntervalSquare(pu), proofbound.IntervalSquare(pv)))
	if !ok {
		return 0, false
	}
	upper, exact := mag.Hi.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
		return 0, false
	}
	return upper, true
}

// lineWallFrame is a straight wall's own EXACT local frame: n is the
// enclosed unit MATERIAL-SIDE normal (rot90 of the enclosed unit tangent)
// and e is the enclosed unit tangent itself, both read once, since a line's
// own direction never moves as its offset amount varies — only its anchor
// point does, along n.
type lineWallFrame struct {
	anchorU, anchorV float64
	n, e             Point
}

// The interval e below deliberately EXCLUDES normalize2(w.tanInU, w.tanInV).
// That is not a hole in the enclosure, and widening e to cover normalize2's
// output would be wrong. UnitVec encloses the EXACT unit vector of the float
// pair — the value normalize2 rounds — and §8.4 requires this frame to hold the
// direction the construction DENOTES "whatever the platform's sqrt and hypot
// did", so anchoring it on a rounded float would replace the denoted direction
// with one particular platform's approximation of it. A tangent of (3, 4) is
// the clearest case: e is the exact [3/5, 3/5] × [4/5, 4/5], 3/5 is not a
// float, and normalize2 lands 2.22e-17 below the interval it is not supposed
// to be in. To falsify this claim, exhibit an admitted corner whose published
// Edge.Length interval fails to enclose the true locus length — not merely one
// where normalize2's output falls outside e, which is every rotated wall.
func lineWallFrameOf(w survey2d.SideWalk) (lineWallFrame, bool) {
	e, ok := UnitVec(w.TanInU, w.TanInV)
	if !ok {
		return lineWallFrame{}, false
	}
	n := Point{U: proofbound.IntervalNeg(e.V), V: e.U}
	return lineWallFrame{anchorU: w.StartU, anchorV: w.StartV, n: n, e: e}, true
}

// LineCircleLocusSpeedUpper bounds |dP/dt| for the corner foot where a
// STRAIGHT wall's own offset carrier meets a CIRCULAR wall's, over the
// offset range [t0, t1], by an EXACT closed form rather than
// CircleCircleLocusSpeedUpper's decorrelated enclosure — which matters
// because this is the common case, reached at every straight-to-arc
// tangent-filleted corner this codebase builds (a rounded rectangle's own
// corners among them), and that decorrelated method refuses on every one of
// them (see CircleCircleLocusSpeedUpper's own doc comment).
//
// Parametrise a point on the offset line as anchor + t·n + s·e (n, e the
// line's own fixed unit normal and tangent — offsetCarrier's own
// construction), and substitute into the offset circle's own
// |X−c| = R − inside·t. Writing w(t) = (anchor + t·n) − c, the quadratic in
// s is s² + 2(w(t)·e)s + (w(t)·w(t) − (R−inside·t)²) = 0, whose
// discriminant is Δ(t) = (w(t)·e)² − w(t)·w(t) + (R−inside·t)². Because
// n·e = 0 and |n| = 1, w(t)·e is CONSTANT (call it β) and w(t)·w(t) is
// R² − α² + ... — expanding fully, Δ(t)'s own t² coefficient is exactly
// 1 − inside² = 0 (inside is ±1), so Δ is EXACTLY AFFINE in t:
// Δ(t) = (R² − α²) − 2(α + inside·R)·t, α = w0·n, w0 = anchor − c.
//
// That affine form is what tells a TANGENT join's own PERSISTENT tangency
// (Δ(t) ≡ 0, both coefficients zero) from a momentary one (Δ0 = 0 but
// Δ1 ≠ 0, a true fold where the speed is genuinely unbounded right at the
// corner): both of s's two roots collapse to the SAME value −β for every t
// when Δ1 = 0, so X(t) = anchor + t·n − β·e is itself affine in t and
// X'(t) = n exactly, a closed-form, finite velocity that never needed a
// square root at all. Where Δ1 ≠ 0, |s'(t)| = |Δ1| / (2·sqrt(Δ(t))), whose
// magnitude does not depend on which of the two roots s(t) actually is — so
// this never needs to pick the branch nearest the corner the way
// intersectOffsets' own POSITION solve does; both roots move at the same
// speed. Δ affine means its minimum over [t0, t1] is at one of the two
// ends, so bounding it needs no search.
//
// ok is false whenever Δ's own enclosure reaches or crosses zero somewhere
// in [t0, t1], WITHOUT Δ1's own enclosure being exactly zero (a momentary
// fold within this range), or any enclosure fails.
//
// A TANGENT join this evaluator itself built — Fillet's own corner rewrite
// (fillet.go), which is exactly what a rounded-rectangle wall's corner is —
// lands Δ1 at EXACTLY zero, bit for bit: the tangent condition
// α = −inside·R the construction holds by is stated in the SAME floats this
// derivation reads, with no residual from a numerical solve to round away.
// A corner recorded through sketch's own solver need not be so exact, and
// Δ1's enclosure straddling (rather than sitting AT) zero there still falls
// through to the general bound below, which is sound but can refuse on a
// near-tangent corner no public fixture reaches today — the PR body for
// this change names that as a known limitation.
func LineCircleLocusSpeedUpper(line, circle survey2d.SideWalk, t0, t1 float64) (float64, bool) {
	frame, ok := lineWallFrameOf(line)
	if !ok {
		return 0, false
	}
	cx, cy := proofarith.FloatRat(circle.CU), proofarith.FloatRat(circle.CV)
	radius := proofarith.FloatRat(circle.Radius)
	anchorU, anchorV := proofarith.FloatRat(frame.anchorU), proofarith.FloatRat(frame.anchorV)
	if cx == nil || cy == nil || radius == nil || anchorU == nil || anchorV == nil {
		return 0, false
	}
	w0u := new(big.Rat).Sub(anchorU, cx)
	w0v := new(big.Rat).Sub(anchorV, cy)
	alpha := proofbound.IntervalAdd(proofbound.IntervalMul(proofbound.PointInterval(w0u), frame.n.U), proofbound.IntervalMul(proofbound.PointInterval(w0v), frame.n.V))
	inside := InsideSignOf(circle)

	// Delta(t) = (R^2 - alpha^2) - 2*(alpha + inside*R)*t = delta0 + delta1*t.
	delta0 := proofbound.IntervalSub(proofbound.IntervalSquare(proofbound.PointInterval(radius)), proofbound.IntervalSquare(alpha))
	delta1 := proofbound.IntervalScale(proofbound.IntervalAdd(alpha, proofbound.IntervalScale(proofbound.PointInterval(radius), inside)), big.NewRat(-2, 1))

	// Delta1 == 0 EXACTLY (both ends of its own enclosure) is the persistent-
	// tangency closed form this function's own doc comment derives: s(t) is
	// then constant, so X'(t) = n exactly and no square root — nor Delta0's
	// own sign, which this branch never even reads — enters the answer. The
	// bound is |n|'s own enclosed magnitude (n is unit by construction, so
	// this is 1 up to UnitVec's own tiny sqrt rounding) rather than
	// proofbound.Radius2D's √2-scaled one, since this specific case is common enough —
	// every tangent-filleted corner in this codebase's own test fixtures —
	// to be worth the tighter bound.
	if delta1.Lo.Sign() == 0 && delta1.Hi.Sign() == 0 {
		nMagUpper := proofbound.RatSqrtUp(proofbound.IntervalAbsUpper(proofbound.IntervalAdd(proofbound.IntervalSquare(frame.n.U), proofbound.IntervalSquare(frame.n.V))))
		if proofbound.IsNonFinite(nMagUpper) {
			return 0, false
		}
		return nMagUpper, true
	}

	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return 0, false
	}
	deltaAt := func(t *big.Rat) proofbound.RatInterval {
		return proofbound.IntervalAdd(delta0, proofbound.IntervalMul(delta1, proofbound.PointInterval(t)))
	}
	d0, d1 := deltaAt(rt0), deltaAt(rt1)
	// Delta is affine, so its minimum over [t0, t1] is at one of the two
	// ends — no interior point needs checking.
	deltaMinLo := d0.Lo
	if d1.Lo.Cmp(deltaMinLo) < 0 {
		deltaMinLo = d1.Lo
	}
	if deltaMinLo.Sign() <= 0 {
		// The discriminant reaches or crosses zero somewhere this enclosure
		// cannot rule out: a genuine fold inside this range, where the
		// position solve's own branch can turn without bound.
		return 0, false
	}
	sqrtLower := proofbound.RatSqrtDown(deltaMinLo)
	if sqrtLower <= 0 || proofbound.IsNonFinite(sqrtLower) {
		return 0, false
	}
	delta1Upper := proofbound.IntervalAbsUpper(delta1)
	delta1UpperF, exact := delta1Upper.Float64()
	if !exact {
		delta1UpperF = math.Nextafter(delta1UpperF, math.Inf(1))
	}
	sPrimeUpper := proofbound.UpRound(delta1UpperF / (2 * sqrtLower))
	if proofbound.IsNonFinite(sPrimeUpper) {
		return 0, false
	}
	// |dP/dt| = sqrt(1 + s'(t)^2), since n and e are orthonormal.
	upper := proofbound.Radius2D(1, sPrimeUpper)
	if proofbound.IsNonFinite(upper) {
		return 0, false
	}
	return upper, true
}

// MiterLocusSpeedUpper bounds |dP/dt| — the in-plane speed of the corner
// foot's own denoted locus (docs/modify-reach-design.md §8.3's exact offset
// family) — over the offset range [t0, t1], for the corner where prev's own
// offset carrier meets cur's, dispatching to the exact closed form
// (LineCircleLocusSpeedUpper) where one carrier is straight and the other
// circular, or the generic interval solve (CircleCircleLocusSpeedUpper)
// where both are circular. At least one of prev, cur must be circular — the
// caller (capSlantEdge) never reaches here for a line-line miter, whose
// locus is already exact.
func MiterLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	switch {
	case !prev.IsCircular() && cur.IsCircular():
		return LineCircleLocusSpeedUpper(prev, cur, t0, t1)
	case prev.IsCircular() && !cur.IsCircular():
		return LineCircleLocusSpeedUpper(cur, prev, t0, t1)
	default:
		return CircleCircleLocusSpeedUpper(prev, cur, t0, t1, vU, vV)
	}
}
