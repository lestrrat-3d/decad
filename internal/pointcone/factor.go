package pointcone

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
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
//
// For a general axis, x is the exact s(p) = p·A, rho the bracket of
// σ(p)² = |p×A|², and a the A-scaled apex of the sample constants c
// (ConstantsOf). The crossing c/(s + k·σ) equals the millimetre a/(x + k·rho)
// of the same geometry, so the point and Q errors do not depend on the axis
// (agent-docs/revolve-axis-design.md §5.4).
func FactorDyadic(p r3.Vec, axis coneaxis.Axis, c *Consts) (Sample, error) {
	if !proofbound.FiniteVec(p) {
		return Sample{}, fmt.Errorf("%w: a point-cone cap vertex is non-finite", decaderr.ErrNotFinite)
	}
	if !axis.Canonical() {
		heldS, heldSigma := axisHeld(p, axis)
		return factorDyadic(p, axis.Axial(p), axis.RadialSq(p), heldS, heldSigma, c)
	}
	x, y, z := proofarith.MustDyOf(p.X), proofarith.MustDyOf(p.Y), proofarith.MustDyOf(p.Z)
	rho2 := proofarith.DyAdd(proofarith.DyMul(y, y), proofarith.DyMul(z, z))
	return factorDyadic(p, x, rho2, p.X, math.Hypot(p.Y, p.Z), c)
}

// axisHeld is the float reading of s(p) and σ(p) the held crossing parameter
// starts from. Only the exact enclosures bound its error.
func axisHeld(p r3.Vec, axis coneaxis.Axis) (float64, float64) {
	a := axis.Vec()
	return p.Dot(a), p.Cross(a).Len()
}

// heldCrossing is the held crossing parameter fl(fl(x + fl(k·rho))/a) that
// both factor forms publish and their errors are measured against. The
// float64() conversion rounds the product before the add, so a target free to
// fuse the multiply and the add (arm64, or amd64 built with GOAMD64=v3) holds
// the same q, alpha and cone point as every other target, and FactorDyadic
// and FactorRat stay bit-identical within one build.
func heldCrossing(x, rho, k, a float64) float64 {
	return (x + float64(k*rho)) / a
}

// factorDyadic is FactorDyadic over the exact axial reading x and squared
// radial reading rho2, with heldX and heldRho their float readings.
func factorDyadic(p r3.Vec, x, rho2 proofarith.Dyadic, heldX, heldRho float64, c *Consts) (Sample, error) {
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
	q := heldCrossing(heldX, heldRho, c.slopeF, c.apexF)
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
	px, py, pz := proofarith.MustDyOf(p.X), proofarith.MustDyOf(p.Y), proofarith.MustDyOf(p.Z)
	bound := proofbound.Radius3D(max(coordError(px, held.X), coordError(py, held.Y), coordError(pz, held.Z)))
	qError := math.Max(
		proofarith.DyQuoFloatError(proofarith.DyMul(c.ad, gLo), c.cAlpha, q),
		proofarith.DyQuoFloatError(proofarith.DyMul(c.ad, gHi), c.cAlpha, q))
	return Sample{Q: q, Alpha: alpha, QError: qError, PointError: bound}, nil
}

// FactorRat is the big.Rat form of the crossing enclosure of FactorDyadic.
// For a general axis, apex is the A-scaled apex c.
func FactorRat(p r3.Vec, axis coneaxis.Axis, apex, slope *big.Rat) (Sample, error) {
	if !proofbound.FiniteVec(p) {
		return Sample{}, fmt.Errorf("%w: a point-cone cap vertex is non-finite", decaderr.ErrNotFinite)
	}
	if !axis.Canonical() {
		heldS, heldSigma := axisHeld(p, axis)
		return factorRat(p, axis.Axial(p).Rat(), axis.RadialSq(p).Rat(), heldS, heldSigma, apex, slope)
	}
	x, y, z := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
	rho2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
	return factorRat(p, x, rho2, p.X, math.Hypot(p.Y, p.Z), apex, slope)
}

// factorRat is FactorRat over the exact axial reading x and squared radial
// reading rho2, with heldX and heldRho their float readings.
func factorRat(p r3.Vec, x, rho2 *big.Rat, heldX, heldRho float64, apex, slope *big.Rat) (Sample, error) {
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
	q := heldCrossing(heldX, heldRho, k, a)
	if !proofbound.FiniteVec(r3.Vec{X: q}) || q <= 0 {
		return Sample{}, fmt.Errorf("%w: the held point-cone ray has no finite crossing", decaderr.ErrUnsupported)
	}
	alpha := 1 / q
	if !proofbound.FiniteVec(r3.Vec{X: alpha}) {
		return Sample{}, fmt.Errorf("%w: the held point-cone crossing is non-finite", decaderr.ErrUnsupported)
	}
	held := p.Scale(alpha)
	px, py, pz := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
	bound := proofbound.Radius3D(max(
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, px), held.X),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, py), held.Y),
		proofbound.IntervalFloatError(proofbound.IntervalScale(alphaIv, pz), held.Z)))
	return Sample{Q: q, Alpha: alpha,
		QError: proofbound.IntervalFloatError(qIv, q), PointError: bound}, nil
}
