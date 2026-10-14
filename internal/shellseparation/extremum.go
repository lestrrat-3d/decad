package shellseparation

import (
	"math"
	"math/big"
)

// slabProjection and projection return the exact least and greatest of many
// exact rational candidates. Each candidate first gets an outward float
// enclosure [lo, hi] of its exact value. The exact least value is at most every
// candidate's hi, so a candidate whose lo exceeds the smallest hi can never be
// the least value, and symmetrically for the greatest. Only the remaining
// candidates are evaluated exactly, by the same exact expressions as before,
// so the returned extrema are the same rationals. The enclosures never decide
// a separation: they only choose which exact values to compute.
//
// Enclosure argument. Every float operation below is one IEEE binary64
// operation (+, -, *, /) on float operands, rounded to nearest, followed by a
// step of one ulp outward (math.Nextafter). The exact result lies within half
// an ulp of the rounded one, so the step lands on the far side of it. This
// holds for every normal and subnormal result. An overflow to an infinity steps
// to MaxFloat64 toward zero, which the exact result also passes; an infinite
// outward end only widens the enclosure. A NaN end compares false everywhere,
// which keeps the candidate for exact evaluation.
//
// Fused multiply-add. A product is always converted with an explicit float64
// conversion before it is used, and the Go specification forbids fusing across
// an explicit conversion, so no compiler on any architecture (amd64 or arm64)
// folds a product into a later add. Each operation therefore rounds exactly
// once, as the argument above assumes. Even a fused result would be one
// correctly rounded value of the exact expression, which the one-ulp step also
// encloses; the conversion only keeps the rounding count fixed.
//
// Widening is charged operation by operation: an input that is itself an
// enclosure [aLo, aHi] enters a monotone operation through the end that
// produces the lower result for the lower end and the end that produces the
// higher result for the upper end (all four end products for a product of two
// enclosures), and each result is stepped outward again, so every operation
// adds its own rounding on top of the width it inherited. Nothing narrows an
// enclosure except intersection with another enclosure that is itself proven
// to hold the exact value (the hull of a crossing's two end values).

func fDown(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }
func fUp(x float64) float64   { return math.Nextafter(x, math.Inf(1)) }

func fAddDown(a, b float64) float64 { return fDown(a + b) }
func fAddUp(a, b float64) float64   { return fUp(a + b) }
func fSubDown(a, b float64) float64 { return fDown(a - b) }
func fSubUp(a, b float64) float64   { return fUp(a - b) }
func fMulDown(a, b float64) float64 { return fDown(float64(a * b)) }
func fMulUp(a, b float64) float64   { return fUp(float64(a * b)) }
func fQuoDown(a, b float64) float64 { return fDown(a / b) }
func fQuoUp(a, b float64) float64   { return fUp(a / b) }

// ratEnclosure returns floats lo <= r <= hi. A conversion that is not exact
// rounds to the nearest float, so r lies strictly between its neighbours; an
// overflow to an infinity has MaxFloat64 as its neighbour toward zero.
func ratEnclosure(r *big.Rat) (float64, float64) {
	f, exact := r.Float64()
	if exact {
		return f, f
	}
	return fDown(f), fUp(f)
}

// scaleEnclosure encloses n*y for every n in [nLo, nHi] and an exact float y.
// The product is linear in n, so its ends are the two end products.
func scaleEnclosure(nLo, nHi, y float64) (float64, float64) {
	return min(fMulDown(nLo, y), fMulDown(nHi, y)), max(fMulUp(nLo, y), fMulUp(nHi, y))
}

// direction is an exact projection direction (ny, nz) with float enclosures
// of both components.
type direction struct {
	ny, nz                 *big.Rat
	nyLo, nyHi, nzLo, nzHi float64
}

func newDirection(ny, nz *big.Rat) direction {
	d := direction{ny: ny, nz: nz}
	d.nyLo, d.nyHi = ratEnclosure(ny)
	d.nzLo, d.nzHi = ratEnclosure(nz)
	return d
}

// value encloses ny*y + nz*z for exact floats y and z.
func (d direction) value(y, z float64) (float64, float64) {
	aLo, aHi := scaleEnclosure(d.nyLo, d.nyHi, y)
	bLo, bHi := scaleEnclosure(d.nzLo, d.nzHi, z)
	return fAddDown(aLo, bLo), fAddUp(aHi, bHi)
}

// exact is ny*y + nz*z, the exact expression the projections use.
func (d direction) exact(y, z float64) *big.Rat {
	return ratAdd(ratMul(d.ny, ratF(y)), ratMul(d.nz, ratF(z)))
}

// crossingParamEnclosure encloses t = (limit - xk)/(xn - xk) for exact floats
// with limit strictly between xk and xn, so 0 < t < 1.
func crossingParamEnclosure(xk, xn, limit float64) (float64, float64) {
	// Both differences have the sign of xn - xk; the reversed edge uses the
	// negated differences, so a and d are positive.
	aLo, aHi := fSubDown(limit, xk), fSubUp(limit, xk)
	dLo, dHi := fSubDown(xn, xk), fSubUp(xn, xk)
	if xn < xk {
		aLo, aHi = fSubDown(xk, limit), fSubUp(xk, limit)
		dLo, dHi = fSubDown(xk, xn), fSubUp(xk, xn)
	}
	tLo, tHi := fQuoDown(aLo, dHi), fQuoUp(aHi, dLo)
	if !(tLo >= 0) {
		tLo = 0
	}
	if !(tHi <= 1) {
		tHi = 1
	}
	return tLo, tHi
}

// crossingEnclosure encloses vk + t*(vn - vk), t = (limit - xk)/(xn - xk),
// for exact floats xk, xn and limit with limit strictly between xk and xn, so
// 0 < t < 1, and exact vk in [kLo, kHi], vn in [nLo, nHi]. The result is also
// clipped to the hull of the two end enclosures, which holds the exact value
// because it is a convex combination of vk and vn.
func crossingEnclosure(xk, xn, limit, kLo, kHi, nLo, nHi float64) (float64, float64) {
	tLo, tHi := crossingParamEnclosure(xk, xn, limit)
	eLo, eHi := fSubDown(nLo, kHi), fSubUp(nHi, kLo)
	pLo := min(fMulDown(tLo, eLo), fMulDown(tHi, eLo), fMulDown(tLo, eHi), fMulDown(tHi, eHi))
	pHi := max(fMulUp(tLo, eLo), fMulUp(tHi, eLo), fMulUp(tLo, eHi), fMulUp(tHi, eHi))
	lo, hi := fAddDown(kLo, pLo), fAddUp(kHi, pHi)
	// math.Max and math.Min return NaN for a NaN operand, which keeps the
	// candidate.
	return math.Max(lo, math.Min(kLo, nLo)), math.Min(hi, math.Max(kHi, nHi))
}
