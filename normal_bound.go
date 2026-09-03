package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file owns the proof behind the bound every `Face.NormalAt` arm
// publishes (topology.go).
//
// The exact answer a normal reading is judged against is the outward UNIT
// normal of the surface the face's own tag names, evaluated in exact
// arithmetic on that tag's held numbers. Those numbers are the surface the
// record denotes: a placement re-evaluates the record and stores its own
// coordinates, so the record IS what it denotes — the same rule
// prismPayload's sectionDelta states for a section
// (docs/prism-boolean-design.md §7).
//
// Two independent things separate that exact direction from the float triple
// an arm hands back, and the bound below covers both:
//
//   - The ARITHMETIC the arm runs. A difference of two points, a projection
//     onto the axis, a normalization, the cross product an r3.Frame derives a
//     Plane's normal from on every call — it stores its two in-plane axes and
//     no normal at all — and, for a Cone, a cosine and a sine of the held half
//     angle are each rounded, and none of them is charged anywhere else.
//   - The held tag's own departure from the UNIT length a direction must
//     have. A placed cap's frame normal is the rounded image of a rotation,
//     so its length is not exactly one; a vector of length 1+e is not the
//     exact unit normal of any surface at all, and the distance to the
//     nearest unit vector is exactly |e|.
//
// Face.NormalAt separately composes this arm bound with Face.normalBound where
// the surface actually carried is only bounded-close to its published tag.
//
// Every enclosure here is a rational interval, so the answer never depends on
// what the platform's own libm did — the same discipline moments.go and
// capblend_contour.go already run on. Where an enclosure cannot separate the
// direction from zero the reading REFUSES, rather than publish a bound
// nothing proves.
//
// None of this is a residual gate, and no small number here admits anything.
// The enclosure IS the exact answer, built from the tag's own numbers in
// rational arithmetic, so a zero bound records that the two agree EXACTLY —
// never that they agree closely. That is what separates it from the residual
// CLAUDE.md's reject-only rule bans an admission from: there is no upstream
// claim being blessed, only a closed form being evaluated twice.

// normalStatus is the three-valued outcome of enclosing a face's own exact
// normal, the same shape clearance_degen.go's oracle uses: a proven answer, a
// proven degeneracy, and an undecided case that is never quietly folded into
// either.
type normalStatus int

const (
	// normalProven means the exact unit normal is enclosed and its bound
	// stated.
	normalProven normalStatus = iota
	// normalZero means the direction is EXACTLY zero, so the surface gives
	// this point no normal at all.
	normalZero
	// normalUnproven means the enclosure could not separate the direction
	// from zero, or a held number is not finite. Nothing is claimed.
	normalUnproven
)

// ivVec3 encloses a 3D vector coordinate-wise. A vector built from held
// float64s alone encloses it EXACTLY — every interval is a point — and only a
// square root or a sine widens one.
type ivVec3 [3]ratInterval

// ivVec3Of encloses a held vector exactly, one point interval per coordinate.
func ivVec3Of(v r3.Vec) (ivVec3, bool) {
	x, y, z := floatRat(v.X), floatRat(v.Y), floatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return ivVec3{}, false
	}
	return ivVec3{pointInterval(x), pointInterval(y), pointInterval(z)}, true
}

func ivVec3Add(a, b ivVec3) ivVec3 {
	return ivVec3{intervalAdd(a[0], b[0]), intervalAdd(a[1], b[1]), intervalAdd(a[2], b[2])}
}

func ivVec3Sub(a, b ivVec3) ivVec3 {
	return ivVec3{intervalSub(a[0], b[0]), intervalSub(a[1], b[1]), intervalSub(a[2], b[2])}
}

func ivVec3Mul(a ivVec3, s ratInterval) ivVec3 {
	return ivVec3{intervalMul(a[0], s), intervalMul(a[1], s), intervalMul(a[2], s)}
}

func ivVec3Cross(a, b ivVec3) ivVec3 {
	return ivVec3{
		intervalSub(intervalMul(a[1], b[2]), intervalMul(a[2], b[1])),
		intervalSub(intervalMul(a[2], b[0]), intervalMul(a[0], b[2])),
		intervalSub(intervalMul(a[0], b[1]), intervalMul(a[1], b[0])),
	}
}

func ivVec3Dot(a, b ivVec3) ratInterval {
	return intervalAdd(intervalAdd(intervalMul(a[0], b[0]), intervalMul(a[1], b[1])), intervalMul(a[2], b[2]))
}

// ivVec3NormSq is |v|² — intervalSquare per coordinate, never intervalMul with
// itself, so a coordinate straddling zero cannot contribute a negative low
// end.
func ivVec3NormSq(a ivVec3) ratInterval {
	return intervalAdd(intervalAdd(intervalSquare(a[0]), intervalSquare(a[1])), intervalSquare(a[2]))
}

// ivVec3Unit encloses the exact unit vector of an enclosed direction. It is
// the 3D sibling of capblend_contour.go's ivUnitVec, and like it the only
// widening a held-float input suffers is the length's own outward-rounded
// square root.
func ivVec3Unit(a ivVec3) (ivVec3, normalStatus) {
	n2 := ivVec3NormSq(a)
	if n2.hi.Sign() <= 0 {
		return ivVec3{}, normalZero
	}
	if n2.lo.Sign() <= 0 {
		return ivVec3{}, normalUnproven
	}
	length, ok := intervalSqrt(n2)
	if !ok || length.lo.Sign() <= 0 {
		return ivVec3{}, normalUnproven
	}
	var out ivVec3
	for i, comp := range a {
		q, ok := intervalQuo(comp, length)
		if !ok {
			return ivVec3{}, normalUnproven
		}
		out[i] = q
	}
	return out, normalProven
}

// unitDirAllow is the whole file's answer: an upper bound on the distance
// between the direction an arm computed and the exact unit normal its own
// enclosure names.
//
// The sign a reversed face applies is a float negation, which is exact, so
// the bound is the same for the outward direction and the geometric one and
// the caller may pass either — as long as both arguments carry the same sign.
func unitDirAllow(exact ivVec3, held r3.Vec) (float64, normalStatus) {
	unit, st := ivVec3Unit(exact)
	if st != normalProven {
		return 0, st
	}
	ex := intervalFloatError(unit[0], held.X)
	ey := intervalFloatError(unit[1], held.Y)
	ez := intervalFloatError(unit[2], held.Z)
	if isNonFinite(ex) || isNonFinite(ey) || isNonFinite(ez) {
		return 0, normalUnproven
	}
	// The three coordinate errors are known separately, so the distance is
	// their own root sum of squares — radius3D's √3 factor is the answer for
	// ONE per-coordinate number, and would only loosen this one.
	sum := absSumUpper(productUpper(ex, ex), productUpper(ey, ey), productUpper(ez, ez))
	allow := upRound(math.Sqrt(sum))
	if isNonFinite(allow) {
		return 0, normalUnproven
	}
	return allow, normalProven
}

// axialRadialExact encloses the exact vector from a surface's axis to p,
// perpendicular to that axis: rel − â(rel·â) with â the axis's OWN exact unit
// direction. Written as rel − a(rel·a)/(a·a) it needs no square root at all,
// so a held-float tag encloses it exactly.
//
// Writing it with the unit axis matters: the arms spell the projection with
// the tag's held Axis, which a placement leaves only near-unit, and the
// difference between the two spellings is part of what this file is charging.
func axialRadialExact(p, origin, axis r3.Vec) (ivVec3, bool) {
	pi, okP := ivVec3Of(p)
	oi, okO := ivVec3Of(origin)
	ai, okA := ivVec3Of(axis)
	if !okP || !okO || !okA {
		return ivVec3{}, false
	}
	rel := ivVec3Sub(pi, oi)
	share, ok := intervalQuo(ivVec3Dot(rel, ai), ivVec3NormSq(ai))
	if !ok {
		return ivVec3{}, false
	}
	return ivVec3Sub(rel, ivVec3Mul(ai, share)), true
}

// planeNormalAllow bounds the Plane arm's own reading. An r3.Frame stores no
// normal — it holds its origin and its two in-plane axes, and derives the
// normal as their cross product on every call — so the arm computes six
// products and three differences, each rounded, and the exact answer is the
// unit vector of the EXACT cross of those two held axes. The bound covers both
// halves of the distance to it: the cross product's own rounding and the
// resulting triple's departure from unit length. It is zero exactly where the
// held axes cross exactly and give a unit result, which is every axis-aligned
// frame.
func planeNormalAllow(fr r3.Frame, held r3.Vec) (float64, normalStatus) {
	u, okU := ivVec3Of(fr.U())
	v, okV := ivVec3Of(fr.V())
	if !okU || !okV {
		return 0, normalUnproven
	}
	return unitDirAllow(ivVec3Cross(u, v), held)
}

// axialNormalAllow bounds the Cylinder arm's reading: its exact normal is the
// axis-to-point direction the projection above names.
func axialNormalAllow(p, origin, axis r3.Vec, held r3.Vec) (float64, normalStatus) {
	exact, ok := axialRadialExact(p, origin, axis)
	if !ok {
		return 0, normalUnproven
	}
	return unitDirAllow(exact, held)
}

// radialNormalAllow bounds a reading whose exact direction is one held
// difference — the Sphere arm's centre-to-point.
func radialNormalAllow(p, center r3.Vec, held r3.Vec) (float64, normalStatus) {
	pi, okP := ivVec3Of(p)
	ci, okC := ivVec3Of(center)
	if !okP || !okC {
		return 0, normalUnproven
	}
	return unitDirAllow(ivVec3Sub(pi, ci), held)
}

// coneNormalAllow bounds the Cone arm's reading. The exact normal is
// r̂·cos(h) − â·sin(h) with r̂ and â the surface's own exact unit radial and
// axis, which is why this arm is inexact even on a body no placement ever
// touched: the arm reads cos and sin off float64 libm, and at every integer
// degree of half angle the pair it gets back does not satisfy cos² + sin² = 1.
func coneNormalAllow(p r3.Vec, s Cone, half float64, held r3.Vec) (float64, normalStatus) {
	radial, ok := axialRadialExact(p, s.Origin, s.Axis)
	if !ok {
		return 0, normalUnproven
	}
	rdir, st := ivVec3Unit(radial)
	if st != normalProven {
		return 0, st
	}
	axis, okA := ivVec3Of(s.Axis)
	if !okA {
		return 0, normalUnproven
	}
	adir, st := ivVec3Unit(axis)
	if st != normalProven {
		return 0, normalUnproven
	}
	rHalf := floatRat(half)
	if rHalf == nil {
		return 0, normalUnproven
	}
	sin, cos, okT := radSinCosInterval(rHalf)
	if !okT {
		return 0, normalUnproven
	}
	return unitDirAllow(ivVec3Sub(ivVec3Mul(rdir, cos), ivVec3Mul(adir, sin)), held)
}

// torusNormalAllow bounds the Torus arm's reading: the exact direction runs
// from the tube centre at p's own azimuth to p, and the tube centre itself is
// already an enclosure, since it rides the unit radial.
func torusNormalAllow(p r3.Vec, s Torus, major float64, held r3.Vec) (float64, normalStatus) {
	radial, ok := axialRadialExact(p, s.Center, s.Axis)
	if !ok {
		return 0, normalUnproven
	}
	rdir, st := ivVec3Unit(radial)
	if st != normalProven {
		return 0, st
	}
	rMajor := floatRat(major)
	pi, okP := ivVec3Of(p)
	ci, okC := ivVec3Of(s.Center)
	if rMajor == nil || !okP || !okC {
		return 0, normalUnproven
	}
	rel := ivVec3Sub(pi, ci)
	return unitDirAllow(ivVec3Sub(rel, ivVec3Mul(rdir, pointInterval(rMajor))), held)
}

// turnGridShift is the dyadic grid the radian-to-turn conversion lands on
// before moments_trig.go's series runs. π's own in-tree bounds carry
// seventy-odd digits, so the quotient by 2π is a rational nothing needs to
// square that wide; rounding it down to a 2⁻⁹⁶ grid and charging the whole
// gap back through the sine's own Lipschitz constant keeps the series input
// small while leaving the enclosure valid.
const turnGridShift = 96

// radSinCosInterval encloses sin(x) and cos(x) for an exact rational RADIAN
// value x, which is what a Cone's half angle is once it has been read out in
// radians.
//
// moments_trig.go's turnSinCosInterval answers for a rational TURN, and a
// radian value is not one: dividing by 2π lands on an interval, since π is
// itself only enclosed. So the turn is rounded DOWN onto a dyadic grid, the
// series is evaluated at that one point, and both readings are widened by
// 2π·w with w the whole remaining gap — sound because |sin(2πt) − sin(2πt₀)|
// ≤ 2π|t − t₀| and the same for the cosine, so no monotonicity over the gap
// need be argued.
func radSinCosInterval(x *big.Rat) (ratInterval, ratInterval, bool) {
	if x.Sign() == 0 {
		zero, one := new(big.Rat), big.NewRat(1, 1)
		return interval(zero, zero), interval(one, one), true
	}
	twoPi := twoPiInterval()
	turn, ok := intervalQuo(pointInterval(x), twoPi)
	if !ok {
		return ratInterval{}, ratInterval{}, false
	}
	base := ratFloorGrid(turn.lo, turnGridShift)
	gap := new(big.Rat).Sub(turn.hi, base)
	if gap.Sign() < 0 {
		return ratInterval{}, ratInterval{}, false
	}
	sin, cos := turnSinCosInterval(base)
	slop := new(big.Rat).Mul(twoPi.hi, gap)
	return intervalWiden(sin, slop), intervalWiden(cos, slop), true
}

// radSinCosSpan is radSinCosInterval over a whole radian INTERVAL rather than
// one exact radian value — what a caller holds when the angle itself is only
// enclosed, as an arc's own a0 + t·sweep is (both terms come from
// atan2Interval).
//
// It evaluates the point enclosure at the span's lower end and widens both
// readings by the span's own width. That is sound because sine and cosine are
// 1-Lipschitz, so no monotonicity over the span need be argued — the same
// argument radSinCosInterval makes for its own grid gap, one level up.
func radSinCosSpan(x ratInterval) (ratInterval, ratInterval, bool) {
	width := new(big.Rat).Sub(x.hi, x.lo)
	if width.Sign() < 0 {
		return ratInterval{}, ratInterval{}, false
	}
	sin, cos, ok := radSinCosInterval(x.lo)
	if !ok {
		return ratInterval{}, ratInterval{}, false
	}
	return intervalWiden(sin, width), intervalWiden(cos, width), true
}

// intervalWiden grows an enclosure by a non-negative rational on both ends.
func intervalWiden(a ratInterval, w *big.Rat) ratInterval {
	return interval(new(big.Rat).Sub(a.lo, w), new(big.Rat).Add(a.hi, w))
}

// ratFloorGrid rounds a rational DOWN onto the 2⁻ˢʰⁱᶠᵗ grid, so the result is
// never above the input and sits within 2⁻ˢʰⁱᶠᵗ of it. Rounding down in one
// direction only is what lets the caller charge the whole gap from one end.
func ratFloorGrid(x *big.Rat, shift uint) *big.Rat {
	scale := new(big.Int).Lsh(big.NewInt(1), shift)
	num := new(big.Int).Mul(x.Num(), scale)
	// Denom is positive for every big.Rat, so Div is the floor.
	q := new(big.Int).Div(num, x.Denom())
	return new(big.Rat).SetFrac(q, scale)
}
