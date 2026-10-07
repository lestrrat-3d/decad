package revolvemesh

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// RevolveCellAreaSlack is docs/tessellation-design.md §10.2's Ecell for one
// wall cell of a LINE generator, in closed form by COMPLETE SIGN
// DECOMPOSITION — tess §15's first admissible path, chosen here because the
// decomposition turns out to need no root isolation at all.
//
// The reason is that both densities collapse. Over the common domain
// (t, u) ∈ [0,1]², with t along the meridian chord and u across one angular
// interval, the true patch is
//
//	Ftrue(t, u) = a3 + z(t)·w + ρ(t)·e(φ0 + u·dφ)
//
// with z and ρ AFFINE in t, so ∂Ftrue/∂t × ∂Ftrue/∂u = ρ(t)·dφ·(ρ'·w − z'·e),
// and w ⊥ e are both unit, giving
//
//	Jtrue(t, u) = L·dφ·ρ(t),   L = √(z'² + ρ'²)
//
// which does not depend on u at all. The held facet is FLAT, so its own
// parameterisation is affine and Jheld is the constant 2·area over each half of
// the domain — the half the fixed diagonal cuts. Their difference is therefore
// LINEAR in t on each half, its single zero is an exact rational quotient, and
// each sign-fixed piece integrates in closed form. No polynomial root
// isolation, no interval subdivision, and internal/polynomial/sturm.go's Sturm engine is
// not reached: the certified enclosures of cos dφ and sin dφ enter only through
// the ideal triangle's own area, never inside a root isolation, so the
// widening tess §9's open question worried about cannot lose a sign here.
//
// The two halves carry the weights their domains give them: the diagonal splits
// the unit square into {0 ≤ u ≤ t ≤ 1}, whose u-measure at t is t, and
// {0 ≤ t ≤ u ≤ 1}, whose u-measure is 1 − t.
//
// Every input is an enclosure, and the answer is an upper bound on
// ∫|Jtrue − Jheld| for EVERY member of it: with f_lo ≤ f ≤ f_hi pointwise,
// |f| ≤ |f_lo| + (f_hi − f_lo), and both of those integrate exactly.
//
// The answer is a BOUND on the local density difference, never an estimate of
// the two areas' own gap, and §10.2 forbids it from cancelling. A planar
// annulus cell shows the difference plainly: its true density is linear in the
// meridian parameter while its flat facet's is constant, so the non-cancelling
// integral reads well above the near-agreement of the two total areas. That is
// the reading a later boolean needs, since a boolean can retain one sign lobe
// of an error whose whole-cell sum vanishes.
//
// rho0/rho1 are the cell's two meridian radii, meridian encloses L, step
// encloses dφ, and twoArea encloses twice the ideal triangle's area — one entry
// per half, in the order (diagonal-low half, diagonal-high half).
func RevolveCellAreaSlack(rho0, rho1 float64, meridian, step proofbound.RatInterval, twoArea [2]proofbound.RatInterval) float64 {
	r0, r1 := proofarith.FloatRat(rho0), proofarith.FloatRat(rho1)
	if r0 == nil || r1 == nil {
		return math.Inf(1)
	}
	dRho := new(big.Rat).Sub(r1, r0)
	scale := proofbound.IntervalMul(meridian, step)
	total := new(big.Rat)
	for half, weight := range [2]int{RevolveWeightT, RevolveWeightOneMinusT} {
		total.Add(total, RevolveHalfCellSlack(r0, dRho, scale, twoArea[half], weight))
	}
	return proofbound.RatFloatUp(total)
}

// RevolveFanAreaSlack is RevolveCellAreaSlack for a cell with ONE ring on the
// axis: the held facets are a fan of single triangles rather than quads, and
// the domain is the whole unit square with the pole edge collapsed.
//
// With the pole at t = 0 the held map is P + t·((1−u)·A + u·B), whose Jacobian
// is exactly t·|A × B|, while Jtrue is L·dφ·ρ1·t — both LINEAR in t through the
// origin, so the difference is again linear and the same closed form answers.
// The pole at t = 1 is the mirror image, in 1 − t.
func RevolveFanAreaSlack(rhoOff float64, poleFirst bool, meridian, step, twoArea proofbound.RatInterval) float64 {
	r := proofarith.FloatRat(rhoOff)
	if r == nil {
		return math.Inf(1)
	}
	// f(t) = (L·dφ·ρ_off − 2A)·t for a pole at t = 0, and the same constant
	// times (1 − t) for a pole at t = 1.
	c := proofbound.IntervalSub(proofbound.IntervalScale(proofbound.IntervalMul(meridian, step), r), twoArea)
	lo, hi := c.Lo, c.Hi
	alpha, beta := lo, new(big.Rat)
	if !poleFirst {
		alpha, beta = new(big.Rat).Neg(lo), new(big.Rat).Set(lo)
	}
	width := new(big.Rat).Sub(hi, lo)
	widthAlpha, widthBeta := width, new(big.Rat)
	if !poleFirst {
		widthAlpha, widthBeta = new(big.Rat).Neg(width), new(big.Rat).Set(width)
	}
	total := new(big.Rat).Add(
		AbsLinearIntegral(alpha, beta, RevolveWeightOne),
		AbsLinearIntegral(widthAlpha, widthBeta, RevolveWeightOne),
	)
	return proofbound.RatFloatUp(total)
}

// RevolveHalfCellSlack is one half-domain's contribution to Ecell: the exact
// ∫|f_lo|·w plus the exact ∫(f_hi − f_lo)·w, with
// f(t) = L·dφ·(ρ0 + t·Δρ) − 2A.
func RevolveHalfCellSlack(rho0, dRho *big.Rat, scale, twoArea proofbound.RatInterval, weight int) *big.Rat {
	alphaLo := new(big.Rat).Mul(scale.Lo, dRho)
	betaLo := new(big.Rat).Sub(new(big.Rat).Mul(scale.Lo, rho0), twoArea.Hi)
	spread := new(big.Rat).Sub(scale.Hi, scale.Lo)
	alphaGap := new(big.Rat).Mul(spread, dRho)
	betaGap := new(big.Rat).Add(
		new(big.Rat).Mul(spread, rho0),
		new(big.Rat).Sub(twoArea.Hi, twoArea.Lo),
	)
	return new(big.Rat).Add(
		AbsLinearIntegral(alphaLo, betaLo, weight),
		AbsLinearIntegral(alphaGap, betaGap, weight),
	)
}

// The three weights AbsLinearIntegral integrates against: the whole unit
// interval, and the two halves the fixed cell diagonal cuts the unit square
// into.
const (
	RevolveWeightOne = iota
	RevolveWeightT
	RevolveWeightOneMinusT
)

// AbsLinearIntegral is the exact ∫₀¹ |α·t + β|·w(t) dt over the rationals. The
// integrand's single zero −β/α is isolated exactly, the unit interval is split
// there when the zero falls strictly inside it, and each sign-fixed piece is
// integrated through its own polynomial primitive. This is the whole of
// docs/tessellation-design.md §10.2's "isolate every zero ... then integrate
// each sign-fixed region in closed form" for a straight generator.
func AbsLinearIntegral(alpha, beta *big.Rat, weight int) *big.Rat {
	one := big.NewRat(1, 1)
	bounds := []*big.Rat{new(big.Rat), one}
	if alpha.Sign() != 0 {
		root := new(big.Rat).Quo(new(big.Rat).Neg(beta), alpha)
		if root.Sign() > 0 && root.Cmp(one) < 0 {
			bounds = []*big.Rat{new(big.Rat), root, one}
		}
	}
	total := new(big.Rat)
	for i := 0; i+1 < len(bounds); i++ {
		piece := new(big.Rat).Sub(
			LinearWeightPrimitive(alpha, beta, bounds[i+1], weight),
			LinearWeightPrimitive(alpha, beta, bounds[i], weight),
		)
		total.Add(total, piece.Abs(piece))
	}
	return total
}

// LinearWeightPrimitive evaluates the antiderivative of (α·t + β)·w(t) at t.
func LinearWeightPrimitive(alpha, beta, t *big.Rat, weight int) *big.Rat {
	t2 := new(big.Rat).Mul(t, t)
	t3 := new(big.Rat).Mul(t2, t)
	switch weight {
	case RevolveWeightT:
		// (αt + β)·t = αt² + βt.
		return new(big.Rat).Add(
			new(big.Rat).Mul(alpha, new(big.Rat).Mul(t3, big.NewRat(1, 3))),
			new(big.Rat).Mul(beta, new(big.Rat).Mul(t2, big.NewRat(1, 2))),
		)
	case RevolveWeightOneMinusT:
		// (αt + β)(1 − t) = −αt² + (α − β)t + β.
		return new(big.Rat).Add(
			new(big.Rat).Add(
				new(big.Rat).Mul(new(big.Rat).Neg(alpha), new(big.Rat).Mul(t3, big.NewRat(1, 3))),
				new(big.Rat).Mul(new(big.Rat).Sub(alpha, beta), new(big.Rat).Mul(t2, big.NewRat(1, 2))),
			),
			new(big.Rat).Mul(beta, t),
		)
	default:
		// (αt + β)·1.
		return new(big.Rat).Add(
			new(big.Rat).Mul(alpha, new(big.Rat).Mul(t2, big.NewRat(1, 2))),
			new(big.Rat).Mul(beta, t),
		)
	}
}

// IvTwoTriangleArea encloses twice the area of a triangle whose corners are
// themselves enclosed — the |A × B| the held facet's own Jacobian is.
func IvTwoTriangleArea(p0, p1, p2 proofbound.IvVec3) (proofbound.RatInterval, bool) {
	n := proofbound.IvVec3Cross(proofbound.IvVec3Sub(p1, p0), proofbound.IvVec3Sub(p2, p0))
	return proofbound.IntervalSqrt(proofbound.IvVec3NormSq(n))
}
