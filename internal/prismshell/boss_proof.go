package prismshell

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PrismUnionBossShellSimilarity checks the boss transform and brackets its
// horizontal similarity scale before the local offset is mapped to the boss.
func PrismUnionBossShellSimilarity(xform r3.Transform) (float64, float64, bool) {
	basis := xform.Basis()
	c, s := basis.EX.X, basis.EX.Y
	if basis.EX.Z != 0 || basis.EY != (r3.Vec{X: -s, Y: c}) ||
		basis.EZ != (r3.Vec{Z: 1}) || !proofbound.FiniteVec(basis.EX) ||
		!proofbound.FiniteVec(xform.Translation()) {
		return 0, 0, false
	}
	length2 := new(big.Rat).Add(
		new(big.Rat).Mul(proofarith.FloatRat(c), proofarith.FloatRat(c)),
		new(big.Rat).Mul(proofarith.FloatRat(s), proofarith.FloatRat(s)))
	lo, hi := proofbound.RatSqrtDown(length2), proofbound.RatSqrtUp(length2)
	return lo, hi, lo > 0.99 && hi < 1.01
}

// PrismUnionBossShellLevels checks the exact order and root thickness of the
// plate and boss levels.
func PrismUnionBossShellLevels(plateBottom, bossBottom, interfaceZ, bossTop, t float64) bool {
	pt := func(v float64) *big.Rat { return proofarith.FloatRat(v) }
	return new(big.Rat).Add(pt(plateBottom), pt(t)).Cmp(pt(bossBottom)) < 0 &&
		pt(bossBottom).Cmp(pt(interfaceZ)) < 0 &&
		pt(interfaceZ).Cmp(pt(bossTop)) < 0 &&
		new(big.Rat).Sub(pt(interfaceZ), pt(bossBottom)).Cmp(pt(t)) == 0
}

// PrismUnionBossShellCurveBound encloses the root's quarter-circle chord gap.
func PrismUnionBossShellCurveBound(t float64, stations int) float64 {
	// The planar quarter circle (q(w), s(w)) has second-derivative norm at
	// most 4t on 0 <= w <= 1. Linear interpolation at matching w across a
	// cell of width 1/N differs by at most 4t/(8N²) = t/(2N²). Insetting
	// both rectangle coordinates stretches this gap by at most sqrt(2),
	// rounded safely to 3/2.
	n := float64(stations)
	return proofbound.DivUpper(proofbound.ProductUpper(1.5, t), 2*n*n)
}

// PrismUnionBossShellTrueVolume encloses the outer solid less its three
// cavity regions, including the rounded root band.
func PrismUnionBossShellTrueVolume(plateRect, bossRect BossRect,
	plateBottom, interfaceZ, bossBottom, bossTop, unitLo, unitHi, t float64) proofbound.RatInterval {
	point := func(v float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(v)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := proofbound.Interval(proofarith.FloatRat(unitLo), proofarith.FloatRat(unitHi))
	w := mul(point(bossRect.Width()), scale)
	h := mul(point(bossRect.Height()), scale)
	tv := point(t)
	t2, t3 := mul(tv, tv), mul(mul(tv, tv), tv)
	plateArea := mul(point(plateRect.Width()), point(plateRect.Height()))
	lowerRect := plateRect.Inset(t)
	lowerArea := mul(point(lowerRect.Width()), point(lowerRect.Height()))
	outer := add(mul(plateArea, point(interfaceZ-plateBottom)),
		mul(mul(w, h), point(bossTop-interfaceZ)))
	lower := mul(lowerArea, point(bossBottom-plateBottom-t))
	upper := mul(mul(sub(w, mul(point(2), tv)), sub(h, mul(point(2), tv))),
		point(bossTop-interfaceZ))
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	widths := add(w, h)
	band := add(sub(mul(mul(w, h), tv), mul(mul(point(2), widths), t2)),
		add(mul(mul(mul(widths, pi), t2), point(0.5)),
			sub(mul(t3, proofbound.Interval(big.NewRat(20, 3), big.NewRat(20, 3))),
				mul(mul(point(2), pi), t3))))
	return sub(outer, add(add(lower, upper), band))
}
