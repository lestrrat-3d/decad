package clearance

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file holds the closed-form readings every clearance cell publishes
// (docs/clearance-design.md §5): each returns a proven enclosure of one
// distance or signed height read off the carriers' own floats. Every product,
// sum and difference is taken exactly over the dyadic rationals, and every
// square root and quotient is rounded outward by an exact comparison, so no
// float operation of the reading itself goes uncharged. An enclosure is a
// single point only where that float is the value itself, which is what lets
// an integer part keep an Exact row.

// Dist is a proven enclosure [Lo, Hi] of one closed-form reading: a distance,
// or a signed height above a plane.
type Dist struct{ Lo, Hi float64 }

// PointDist is the enclosure of a value already known exactly.
func PointDist(v float64) Dist { return Dist{Lo: v, Hi: v} }

// Exact reports whether the enclosure is the single finite point it reads.
func (d Dist) Exact() bool { return d.Lo == d.Hi && !proofbound.IsNonFinite(d.Lo) }

// Sub returns d − r, rounded outward.
func (d Dist) Sub(r float64) Dist { return Dist{Lo: sumDown(d.Lo, -r), Hi: sumUp(d.Hi, -r)} }

// Add returns d + r, rounded outward.
func (d Dist) Add(r float64) Dist { return Dist{Lo: sumDown(d.Lo, r), Hi: sumUp(d.Hi, r)} }

// Plus returns d + e, rounded outward.
func (d Dist) Plus(e Dist) Dist { return Dist{Lo: sumDown(d.Lo, e.Lo), Hi: sumUp(d.Hi, e.Hi)} }

// Minus returns d − e, rounded outward.
func (d Dist) Minus(e Dist) Dist { return Dist{Lo: sumDown(d.Lo, -e.Hi), Hi: sumUp(d.Hi, -e.Lo)} }

// Neg returns −d.
func (d Dist) Neg() Dist { return Dist{Lo: -d.Hi, Hi: -d.Lo} }

// Abs returns the enclosure of |x| over every x in d.
func (d Dist) Abs() Dist {
	switch {
	case d.Lo >= 0:
		return d
	case d.Hi <= 0:
		return d.Neg()
	default:
		return Dist{Lo: 0, Hi: math.Max(-d.Lo, d.Hi)}
	}
}

// Widen grows d by a non-negative charge on both sides.
func (d Dist) Widen(charge float64) Dist {
	if charge == 0 {
		return d
	}
	return Dist{Lo: sumDown(d.Lo, -charge), Hi: sumUp(d.Hi, charge)}
}

// Axial reports whether every direction is exactly a signed coordinate axis,
// the only exactly unit directions with dyadic components.
func Axial(dirs ...r3.Vec) bool {
	for _, d := range dirs {
		if _, _, ok := SignedAxis(d); !ok {
			return false
		}
	}
	return true
}

// DirCharge is the charge a reading owes when one of its directions is not a
// signed coordinate axis, and zero when every one is (Axial). The readings
// below are exact over the stated floats, but a float unit direction other
// than an axis is itself a few roundings off the direction its carrier was
// built from, and that tilt moves a reading by at most the lever arm times a
// few units of roundoff. AnalyticRoundBound over an envelope of every point
// and radius the reading takes covers it many times over: every lever arm
// lies inside that envelope.
func DirCharge(dirs []r3.Vec, pts []r3.Vec, lens ...float64) float64 {
	if Axial(dirs...) {
		return 0
	}
	return EnvelopeCharge(pts, lens...)
}

// EnvelopeCharge is AnalyticRoundBound over the sum of every point's largest
// coordinate magnitude and every length given.
func EnvelopeCharge(pts []r3.Vec, lens ...float64) float64 {
	env := 0.0
	for _, p := range pts {
		env = proofbound.AbsSumUpper(env, proofbound.VecMaxAbs(p))
	}
	for _, l := range lens {
		env = proofbound.AbsSumUpper(env, l)
	}
	return proofbound.AnalyticRoundBound(env)
}

// PointPointDist encloses |a − b|.
func PointPointDist(a, b r3.Vec) Dist {
	da, okA := DyVecOf(a)
	db, okB := DyVecOf(b)
	if !okA || !okB {
		return unbounded()
	}
	r := proofarith.DvSub(da, db)
	return rootQuo(proofarith.DvDot(r, r), dyOne)
}

// PointLineDist encloses the distance from p to the line through a along
// dir, which may have any nonzero length.
func PointLineDist(p, a r3.Vec, dir proofarith.DyV3) Dist {
	dp, okP := DyVecOf(p)
	da, okA := DyVecOf(a)
	if !okP || !okA {
		return unbounded()
	}
	return DyPointLineDist(dp, da, dir)
}

// DyPointLineDist is PointLineDist over a point and anchor already held
// exactly.
func DyPointLineDist(p, a, dir proofarith.DyV3) Dist {
	den := proofarith.DvDot(dir, dir)
	if den.Sign() <= 0 {
		return unbounded()
	}
	c := proofarith.DvCross(proofarith.DvSub(p, a), dir)
	return rootQuo(proofarith.DvDot(c, c), den)
}

// SegDir is the exact direction b − a of a segment between two float points;
// ok is false for a non-finite end.
func SegDir(a, b r3.Vec) (proofarith.DyV3, bool) {
	da, okA := DyVecOf(a)
	db, okB := DyVecOf(b)
	return proofarith.DvSub(db, da), okA && okB
}

// LineLineDist encloses the distance between the line through a along u and
// the line through b along v, which must not be parallel.
func LineLineDist(a r3.Vec, u proofarith.DyV3, b r3.Vec, v proofarith.DyV3) Dist {
	da, okA := DyVecOf(a)
	db, okB := DyVecOf(b)
	if !okA || !okB {
		return unbounded()
	}
	w := proofarith.DvCross(u, v)
	den := proofarith.DvDot(w, w)
	if den.Sign() <= 0 {
		return unbounded()
	}
	h := proofarith.DvDot(proofarith.DvSub(db, da), w)
	return rootQuo(proofarith.DyMul(h, h), den)
}

// Height encloses the signed height (p − o)·n/|n| of p above the plane
// through o with normal n, which may have any nonzero length. It is also the
// signed axial offset of p from o along an axis n.
func Height(p, o, n r3.Vec) Dist {
	dp, okP := DyVecOf(p)
	do, okO := DyVecOf(o)
	dn, okN := DyVecOf(n)
	if !okP || !okO || !okN {
		return unbounded()
	}
	den := proofarith.DvDot(dn, dn)
	if den.Sign() <= 0 {
		return unbounded()
	}
	h := proofarith.DvDot(proofarith.DvSub(dp, do), dn)
	mag := rootQuo(proofarith.DyMul(h, h), den)
	if h.Sign() < 0 {
		return mag.Neg()
	}
	return mag
}

// PointCircleDist encloses the distance from p to the circle of radius r
// about c in the plane normal to axis: √(z² + (ρ − s·r)²), z the axial offset
// and ρ the distance from the axis line, read on the near side (s = 1) or
// the far side (s = −1). A point on the axis reads √(z² + r²) on both.
func PointCircleDist(p, c, axis r3.Vec, r, s float64) Dist {
	dp, okP := DyVecOf(p)
	dc, okC := DyVecOf(c)
	da, okA := DyVecOf(axis)
	if !okP || !okC || !okA {
		return unbounded()
	}
	den := proofarith.DvDot(da, da)
	if den.Sign() <= 0 {
		return unbounded()
	}
	rel := proofarith.DvSub(dp, dc)
	zn := proofarith.DvDot(rel, da)
	cr := proofarith.DvCross(rel, da)
	rho := rootQuo(proofarith.DvDot(cr, cr), den)
	t := rho.Sub(s * r)
	t2Lo, t2Hi, ok := squareRange(t)
	if !ok {
		return unbounded()
	}
	zz := proofarith.DyMul(zn, zn)
	return Dist{
		Lo: rootQuoDown(proofarith.DyAdd(zz, proofarith.DyMul(den, t2Lo)), den),
		Hi: rootQuoUp(proofarith.DyAdd(zz, proofarith.DyMul(den, t2Hi)), den),
	}
}

// CircleCircleCoaxialDist encloses the constant distance √(dz² + (ra − rb)²)
// between two circles of radii ra and rb about one axis line, dz the axial
// offset of centre a from centre b along axis.
func CircleCircleCoaxialDist(ca, cb, axis r3.Vec, ra, rb float64) Dist {
	dz := Height(ca, cb, axis)
	dr := PointDist(ra).Sub(rb).Abs()
	z2Lo, z2Hi, okZ := squareRange(dz)
	r2Lo, r2Hi, okR := squareRange(dr)
	if !okZ || !okR {
		return unbounded()
	}
	return Dist{
		Lo: rootQuoDown(proofarith.DyAdd(z2Lo, r2Lo), dyOne),
		Hi: rootQuoUp(proofarith.DyAdd(z2Hi, r2Hi), dyOne),
	}
}

// Amplitude encloses r·|n × axis|/(|n|·|axis|): the amplitude of a circle of
// radius r about axis in its height above a plane of normal n.
func Amplitude(n, axis r3.Vec, r float64) Dist {
	dn, okN := DyVecOf(n)
	da, okA := DyVecOf(axis)
	dr, okR := proofarith.DyOf(r)
	if !okN || !okA || !okR {
		return unbounded()
	}
	den := proofarith.DyMul(proofarith.DvDot(dn, dn), proofarith.DvDot(da, da))
	if den.Sign() <= 0 {
		return unbounded()
	}
	c := proofarith.DvCross(dn, da)
	return rootQuo(proofarith.DyMul(proofarith.DyMul(dr, dr), proofarith.DvDot(c, c)), den)
}

// ConeDist encloses the distance from p to a cone carrier's generating ray in
// p's own meridian half-plane, |ρ·Run − z·Rise|/√(Rise² + Run²), z the axial
// offset of p from the apex and ρ its distance from the axis.
func (f *CFace) ConeDist(p r3.Vec) Dist {
	z := Height(p, f.Anchor, f.Axis)
	da, okA := DyVecOf(f.Axis)
	rise, okRise := proofarith.DyOf(f.Rise)
	run, okRun := proofarith.DyOf(f.Run)
	if !okA || !okRise || !okRun || rise.Sign() < 0 || run.Sign() < 0 {
		return unbounded()
	}
	rho := PointLineDist(p, f.Anchor, da)
	rhoLo, ok1 := proofarith.DyOf(rho.Lo)
	rhoHi, ok2 := proofarith.DyOf(rho.Hi)
	zLo, ok3 := proofarith.DyOf(z.Lo)
	zHi, ok4 := proofarith.DyOf(z.Hi)
	if !ok1 || !ok2 || !ok3 || !ok4 {
		return unbounded()
	}
	// ρ·Run − z·Rise rises with ρ and falls with z, so its range over the two
	// enclosures sits at their opposite ends.
	eLo := proofarith.DySubScalar(proofarith.DyMul(rhoLo, run), proofarith.DyMul(zHi, rise))
	eHi := proofarith.DySubScalar(proofarith.DyMul(rhoHi, run), proofarith.DyMul(zLo, rise))
	var aLo, aHi proofarith.Dyadic
	switch {
	case eLo.Sign() >= 0:
		aLo, aHi = eLo, eHi
	case eHi.Sign() <= 0:
		aLo, aHi = proofarith.DyNeg(eHi), proofarith.DyNeg(eLo)
	default:
		aLo, aHi = proofarith.DyZero(), proofarith.DyAbs(eLo)
		if proofarith.DyCmp(eHi, aHi) > 0 {
			aHi = eHi
		}
	}
	den := proofarith.DyAdd(proofarith.DyMul(rise, rise), proofarith.DyMul(run, run))
	if den.Sign() <= 0 {
		return unbounded()
	}
	return Dist{
		Lo: rootQuoDown(proofarith.DyMul(aLo, aLo), den),
		Hi: rootQuoUp(proofarith.DyMul(aHi, aHi), den),
	}
}

var dyOne = proofarith.DyInt(1)

func unbounded() Dist { return Dist{Lo: 0, Hi: math.Inf(1)} }

// squareRange returns the exact range of x² over the enclosure d.
func squareRange(d Dist) (proofarith.Dyadic, proofarith.Dyadic, bool) {
	lo, okLo := proofarith.DyOf(d.Lo)
	hi, okHi := proofarith.DyOf(d.Hi)
	if !okLo || !okHi {
		return proofarith.Dyadic{}, proofarith.Dyadic{}, false
	}
	l2, h2 := proofarith.DyMul(lo, lo), proofarith.DyMul(hi, hi)
	switch {
	case d.Lo >= 0:
		return l2, h2, true
	case d.Hi <= 0:
		return h2, l2, true
	case proofarith.DyCmp(l2, h2) > 0:
		return proofarith.DyZero(), l2, true
	default:
		return proofarith.DyZero(), h2, true
	}
}

// sumDown and sumUp return a + b rounded down and up, exactly: a sum the
// float itself holds comes back unchanged. A non-finite operand answers the
// infinity on the safe side.
func sumDown(a, b float64) float64 {
	da, okA := proofarith.DyOf(a)
	db, okB := proofarith.DyOf(b)
	if !okA || !okB {
		return math.Inf(-1)
	}
	return proofarith.DyFloatDown(proofarith.DyAdd(da, db))
}

func sumUp(a, b float64) float64 {
	da, okA := proofarith.DyOf(a)
	db, okB := proofarith.DyOf(b)
	if !okA || !okB {
		return math.Inf(1)
	}
	return proofarith.DyFloatUp(proofarith.DyAdd(da, db))
}

func rootQuo(num, den proofarith.Dyadic) Dist {
	return Dist{Lo: rootQuoDown(num, den), Hi: rootQuoUp(num, den)}
}

// rootQuoDown returns the largest float f ≥ 0 within the walk's reach with
// f²·den ≤ num, decided exactly; den must be positive. A root the float holds
// exactly comes back as itself.
func rootQuoDown(num, den proofarith.Dyadic) float64 {
	if num.Sign() <= 0 {
		return 0
	}
	f := rootQuoSeed(num, den)
	if proofbound.IsNonFinite(f) {
		return 0
	}
	if quoSquareCmp(f, num, den) <= 0 {
		for range proofarith.SqrtAdjustLimit {
			next := math.Nextafter(f, math.Inf(1))
			if quoSquareCmp(next, num, den) > 0 {
				return f
			}
			f = next
		}
		return f
	}
	for range proofarith.SqrtAdjustLimit {
		f = math.Nextafter(f, 0)
		if quoSquareCmp(f, num, den) <= 0 {
			return f
		}
	}
	return 0
}

// rootQuoUp returns the smallest float f within the walk's reach with
// f²·den ≥ num, decided exactly, or +Inf.
func rootQuoUp(num, den proofarith.Dyadic) float64 {
	if num.Sign() <= 0 {
		return 0
	}
	f := rootQuoSeed(num, den)
	if proofbound.IsNonFinite(f) {
		return math.Inf(1)
	}
	if quoSquareCmp(f, num, den) >= 0 {
		for range proofarith.SqrtAdjustLimit {
			next := math.Nextafter(f, 0)
			if quoSquareCmp(next, num, den) < 0 {
				return f
			}
			f = next
		}
		return f
	}
	for range proofarith.SqrtAdjustLimit {
		f = math.Nextafter(f, math.Inf(1))
		if quoSquareCmp(f, num, den) >= 0 {
			return f
		}
	}
	return math.Inf(1)
}

// quoSquareCmp compares f²·den with num exactly.
func quoSquareCmp(f float64, num, den proofarith.Dyadic) int {
	df := proofarith.DyOfFinite(f)
	return proofarith.DyCmp(proofarith.DyMul(proofarith.DyMul(df, df), den), num)
}

// rootQuoSeed is a float near √(num/den). It only starts the walks above,
// never decides them.
func rootQuoSeed(num, den proofarith.Dyadic) float64 {
	if proofarith.DyCmp(den, dyOne) == 0 {
		return proofarith.DySqrtSeed(num)
	}
	n, _ := num.Float64()
	d, _ := den.Float64()
	if q := n / d; q > 0 && !proofbound.IsNonFinite(q) && q >= 0x1p-1000 && q <= 0x1p1000 {
		return math.Sqrt(q)
	}
	return proofbound.RatSqrtSeed(new(big.Rat).Quo(num.Rat(), den.Rat()))
}
