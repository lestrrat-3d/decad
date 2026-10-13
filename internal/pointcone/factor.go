package pointcone

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// FactorDyadic is FactorRat's enclosures formed as Dyadic
// numerators over Dyadic denominators. With a = an/ad > 0, k = kn/kd > 0,
// rho = [lo, hi] the exact-rounded root bracket of y²+z² and
// G_lo = x·kd + kn·lo, G_hi = x·kd + kn·hi (so g = [G_lo, G_hi]/kd):
//
//	alpha = a/g = [an·kd/(ad·G_hi), an·kd/(ad·G_lo)]
//	q     = g/a = [ad·G_lo/(an·kd), ad·G_hi/(an·kd)]
//
// Each coordinate error reads both alpha endpoints, so the sign of the
// coordinate (which only orders the scaled interval) never matters. A cone
// with a non-positive apex or slope must use FactorRat.
func FactorDyadic(p r3.Vec, c *Consts) (Sample, error) {
	if !proofbound.FiniteVec(p) {
		return Sample{}, fmt.Errorf("%w: a point-cone cap vertex is non-finite", decaderr.ErrNotFinite)
	}
	x, y, z := proofarith.MustDyOf(p.X), proofarith.MustDyOf(p.Y), proofarith.MustDyOf(p.Z)
	rho2 := proofarith.DyAdd(proofarith.DyMul(y, y), proofarith.DyMul(z, z))
	lo := proofarith.MustDyOf(proofarith.DySqrtDown(rho2))
	hiF := proofarith.DySqrtUp(rho2)
	if math.IsInf(hiF, 0) {
		return Sample{}, fmt.Errorf("%w: the cone radius has no finite enclosure", decaderr.ErrUnsupported)
	}
	hi := proofarith.MustDyOf(hiF)
	xkd := proofarith.DyMul(x, c.kd)
	gLo := proofarith.DyAdd(xkd, proofarith.DyMul(c.kn, lo))
	gHi := proofarith.DyAdd(xkd, proofarith.DyMul(c.kn, hi))
	if gLo.Sign() <= 0 {
		return Sample{}, fmt.Errorf("%w: a point-cone ray is not forward", decaderr.ErrUnsupported)
	}
	q := (p.X + c.slopeF*math.Hypot(p.Y, p.Z)) / c.apexF
	if !proofbound.FiniteVec(r3.Vec{X: q}) || q <= 0 {
		return Sample{}, fmt.Errorf("%w: the held point-cone ray has no finite crossing", decaderr.ErrUnsupported)
	}
	alpha := 1 / q
	if !proofbound.FiniteVec(r3.Vec{X: alpha}) {
		return Sample{}, fmt.Errorf("%w: the held point-cone crossing is non-finite", decaderr.ErrUnsupported)
	}
	held := p.Scale(alpha)
	adGLo, adGHi := proofarith.DyMul(c.ad, gLo), proofarith.DyMul(c.ad, gHi)
	coordError := func(coord proofarith.Dyadic, heldCoord float64) float64 {
		n := proofarith.DyMul(c.cAlpha, coord)
		return math.Max(proofarith.DyQuoFloatError(n, adGHi, heldCoord),
			proofarith.DyQuoFloatError(n, adGLo, heldCoord))
	}
	bound := proofbound.Radius3D(max(coordError(x, held.X), coordError(y, held.Y), coordError(z, held.Z)))
	qError := math.Max(
		proofarith.DyQuoFloatError(proofarith.DyMul(c.ad, gLo), c.cAlpha, q),
		proofarith.DyQuoFloatError(proofarith.DyMul(c.ad, gHi), c.cAlpha, q))
	return Sample{Q: q, Alpha: alpha, QError: qError, PointError: bound}, nil
}

// FactorRat is the big.Rat form of the crossing enclosure of FactorDyadic.
func FactorRat(p r3.Vec, apex, slope *big.Rat) (Sample, error) {
	if !proofbound.FiniteVec(p) {
		return Sample{}, fmt.Errorf("%w: a point-cone cap vertex is non-finite", decaderr.ErrNotFinite)
	}
	x, y, z := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
	rho2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
	rho, ok := proofbound.IntervalSqrt(proofbound.PointInterval(rho2))
	if !ok {
		return Sample{}, fmt.Errorf("%w: the cone radius has no finite enclosure", decaderr.ErrUnsupported)
	}
	g := proofbound.IntervalAdd(proofbound.PointInterval(x),
		proofbound.IntervalScale(rho, slope))
	if g.Lo.Sign() <= 0 {
		return Sample{}, fmt.Errorf("%w: a point-cone ray is not forward", decaderr.ErrUnsupported)
	}
	alphaIv, ok := proofbound.IntervalQuo(proofbound.PointInterval(apex), g)
	if !ok {
		return Sample{}, fmt.Errorf("%w: a point-cone ray has no crossing", decaderr.ErrUnsupported)
	}
	qIv, ok := proofbound.IntervalQuo(g, proofbound.PointInterval(apex))
	if !ok {
		return Sample{}, fmt.Errorf("%w: a point-cone ray has no inverse crossing", decaderr.ErrUnsupported)
	}
	a, _ := apex.Float64()
	k, _ := slope.Float64()
	q := (p.X + k*math.Hypot(p.Y, p.Z)) / a
	if !proofbound.FiniteVec(r3.Vec{X: q}) || q <= 0 {
		return Sample{}, fmt.Errorf("%w: the held point-cone ray has no finite crossing", decaderr.ErrUnsupported)
	}
	alpha := 1 / q
	if !proofbound.FiniteVec(r3.Vec{X: alpha}) {
		return Sample{}, fmt.Errorf("%w: the held point-cone crossing is non-finite", decaderr.ErrUnsupported)
	}
	held := p.Scale(alpha)
	bound := proofbound.Radius3D(max(
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, x), held.X),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, y), held.Y),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, z), held.Z)))
	return Sample{Q: q, Alpha: alpha,
		QError: proofbound.IntervalFloatError(qIv, q), PointError: bound}, nil
}
