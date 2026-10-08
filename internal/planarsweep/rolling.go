package planarsweep

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/placedruling"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
)

// RollingCoefficients bound the contact depth and lateral rim drift as
// constant + rate*t + quadratic*t² and lateralBase + lateral*t.
type RollingCoefficients struct {
	constant, rate, quadratic *big.Rat
	lateralBase, lateral      *big.Rat
	alpha, beta               *big.Rat
	gated                     bool
}

// RollingInput contains the exact source geometry and path readings for a
// cylinder rolling on a stationary-or-translating planar support.
type RollingInput struct {
	Cylinder        placedruling.Cylinder
	Moving, Support Motion
	Points          []proofarith.DyV3
	Normal          proofarith.DyV3
	Heights         [2]proofarith.Dyadic
	Alpha           proofarith.Dyadic
}

// RollingCoefficientsOf bounds both end-disk heights and the rim's lateral
// drift from the same exact motion and staged cylinder readings.
func RollingCoefficientsOf(in RollingInput) (RollingCoefficients, bool) {
	radius := in.Cylinder.Radius.Rat()
	normal := ratOfDyV3(in.Normal)
	relative := ratSub3(in.Moving.Velocity, in.Support.Velocity)
	out := RollingCoefficients{rate: new(big.Rat), quadratic: new(big.Rat), constant: new(big.Rat),
		alpha: new(big.Rat).Abs(in.Alpha.Rat())}
	for i, c := range in.Points {
		lever := ratCross3(in.Moving.Omega, ratSub3(ratOfDyV3(c), in.Moving.Center))
		rate := ratDot3(normal, ratAdd3(relative, lever))
		if rate.Abs(rate).Cmp(out.rate) > 0 {
			out.rate = rate
		}
		curvature, ok := ratSqrtUpRat(proofbound.RatMul(in.Moving.OmegaSq, ratDot3(lever, lever)))
		if !ok {
			return RollingCoefficients{}, false
		}
		if k := proofbound.RatMul(curvature, big.NewRat(1, 2)); k.Cmp(out.quadratic) > 0 {
			out.quadratic = k
		}
		if h := new(big.Rat).Abs(in.Heights[i].Rat()); h.Cmp(out.constant) > 0 {
			out.constant = h
		}
	}
	// β² = |ω|²·|ã|² − (ω·ã)², exact.
	axis := ratOfDyV3(in.Cylinder.Columns[in.Cylinder.Axis])
	along := ratDot3(in.Moving.Omega, axis)
	tiltSq := new(big.Rat).Sub(proofbound.RatMul(in.Moving.OmegaSq, ratDot3(axis, axis)), proofbound.RatMul(along, along))
	tilt, ok := ratSqrtUpRat(tiltSq)
	if !ok {
		return RollingCoefficients{}, false
	}
	out.beta = tilt
	out.constant = proofbound.RatAdd(out.constant, in.Cylinder.SectionDrift(in.Alpha).Rat())
	out.rate = proofbound.RatAdd(out.rate, proofbound.RatMul(big.NewRat(2, 1), radius, tilt, out.alpha))
	out.quadratic = proofbound.RatAdd(out.quadratic, proofbound.RatMul(radius, tiltSq))
	out.lateral = proofbound.RatMul(big.NewRat(3, 2), radius, tilt)
	out.lateralBase = new(big.Rat)
	if in.Cylinder.Gram.Sign() != 0 || in.Alpha.Sign() != 0 {
		out.gated = true
		out.lateralBase = in.Cylinder.RimDrift(in.Alpha.Rat())
	}
	return out, true
}

// At returns the band depth and lateral rim drift at t seconds.
func (c RollingCoefficients) At(t *big.Rat) (*big.Rat, *big.Rat) {
	return proofbound.RatAdd(c.constant, proofbound.RatMul(c.rate, t), proofbound.RatMul(c.quadratic, t, t)),
		proofbound.RatAdd(c.lateralBase, proofbound.RatMul(c.lateral, t))
}

// DriftAdmitted reports whether the rim drift's gate holds through t.
func (c RollingCoefficients) DriftAdmitted(t *big.Rat) bool {
	return !c.gated || proofbound.RatAdd(c.alpha, proofbound.RatMul(c.beta, t)).Cmp(big.NewRat(1, 4)) <= 0
}

// RollingColumnBox holds the staged cylinder corners and the largest section
// offset from its axis, including the start pose's Gram error.
type RollingColumnBox struct {
	Corners []proofarith.DyV3
	Reach   *big.Rat
}

// RollingColumnBoxOf builds the two exact bounds the column test intersects.
func RollingColumnBoxOf(c placedruling.Cylinder) (RollingColumnBox, bool) {
	corners := c.StagedCorners()
	stretch, ok := ratSqrtUpRat(proofbound.RatAdd(big.NewRat(1, 1), c.Gram.Rat()))
	if !ok {
		return RollingColumnBox{}, false
	}
	return RollingColumnBox{Corners: corners[:], Reach: proofbound.RatMul(c.Radius.Rat(), stretch)}, true
}

// RollingColumnBounds intersects the whole-corner span with the end-center
// span grown by reach, then subtracts the support body's translation.
func RollingColumnBounds(centers, corners sweepmemo.CornerSpans, reach *big.Rat,
	delta [3]proofarith.Dyadic, f *big.Rat) (lo, hi [3]*big.Rat) {
	zero := new(big.Rat)
	for axis := range 3 {
		low, high := corners.Hull(axis)
		axisLow, axisHigh := centers.Hull(axis)
		low = proofbound.RatMax(low, new(big.Rat).Sub(axisLow, reach))
		high = proofbound.RatMin(high, new(big.Rat).Add(axisHigh, reach))
		shift := new(big.Rat).Mul(delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(low, proofbound.RatMax(shift, zero))
		hi[axis] = new(big.Rat).Sub(high, proofbound.RatMin(shift, zero))
	}
	return lo, hi
}

// RollingFootBounds encloses the ruling's lateral footprint in the support
// body's start frame, widened by the depth and rim drift.
func RollingFootBounds(spans sweepmemo.CornerSpans, delta [3]proofarith.Dyadic,
	axis int, f, growth *big.Rat) (lo, hi [2]*big.Rat) {
	for slot, direction := range [2]int{(axis + 1) % 3, (axis + 2) % 3} {
		low, high := spans.Hull(direction)
		shift := new(big.Rat).Mul(delta[direction].Rat(), f)
		shiftLo, shiftHi := proofbound.RatMin(shift, new(big.Rat)), proofbound.RatMax(shift, new(big.Rat))
		lo[slot] = proofbound.RatAdd(low, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(growth))
		hi[slot] = proofbound.RatAdd(high, new(big.Rat).Neg(shiftLo), growth)
	}
	return lo, hi
}
