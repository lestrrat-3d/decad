package capband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CircularNormalRange encloses a patch's normal component over its own
// azimuth window. sample reads the published face normal at a given azimuth;
// model encloses the face tag's exact harmonic coefficients and departure.
// Three samples recover a*cos(phi)+b*sin(phi)+c at phi=0, pi/2, pi. The
// window is read over phi=theta-th0, so a window starting away from zero
// still uses the coefficients' own origin.
//
// The allowance charges the built surface's departure from its published
// tag, the recovered coefficients' distance from the exact model, the
// model's nonharmonic slop, and each extreme's interval and float rounding.
// A flat patch uses one Face.NormalAt reading instead; its published bound
// already includes the departure, so that branch must not add it again.
func CircularNormalRange(th0, th1 float64, wholeTurn bool, departure float64,
	model NormalModel, sample func(float64) (float64, bool)) (float64, float64, float64, bool) {
	f0, ok0 := sample(th0)
	f90, ok90 := sample(th0 + math.Pi/2)
	f180, ok180 := sample(th0 + math.Pi)
	if !ok0 || !ok90 || !ok180 {
		return 0, 0, 0, false
	}
	c := (f0 + f180) / 2
	a := f0 - c
	b := f90 - c
	ra, rb, rc := proofarith.FloatRat(a), proofarith.FloatRat(b), proofarith.FloatRat(c)
	rth0, rth1 := proofarith.FloatRat(th0), proofarith.FloatRat(th1)
	if ra == nil || rb == nil || rc == nil || rth0 == nil || rth1 == nil {
		return 0, 0, 0, false
	}
	ext, ok := HarmonicWindowRange(ra, rb, rc, new(big.Rat).Sub(rth1, rth0), wholeTurn)
	if !ok {
		return 0, 0, 0, false
	}
	lo, hi := proofbound.RatFloatDown(ext.MinLo), proofbound.RatFloatUp(ext.MaxHi)
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return 0, 0, 0, false
	}
	allow := proofbound.AbsSumUpper(
		departure,
		proofbound.IntervalFloatError(model.A, a),
		proofbound.IntervalFloatError(model.B, b),
		proofbound.IntervalFloatError(model.C, c),
		proofbound.RatFloatUp(model.Slop),
		proofbound.RatFloatUp(new(big.Rat).Sub(ext.MinHi, ext.MinLo)),
		proofbound.RatFloatUp(new(big.Rat).Sub(ext.MinLo, rlo)),
		proofbound.RatFloatUp(new(big.Rat).Sub(ext.MaxHi, ext.MaxLo)),
		proofbound.RatFloatUp(new(big.Rat).Sub(rhi, ext.MaxHi)),
	)
	if proofbound.IsNonFinite(allow) || proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return 0, 0, 0, false
	}
	return lo, hi, allow, true
}

// PullComponent returns a bounded normal component against the held pull.
// The normal's published bound is scaled by the pull's proven length, and
// the held dot product's rounding is charged against its exact interval.
func PullComponent(n measurement.VecMeasurement, p r3.Vec, pLen float64) (float64, float64, bool) {
	v := n.Value.Dot(p)
	bound, err := sectionrecord.MagnitudeIn(n.Bound, units.Dimensionless, units.One, "a normal's own bound")
	if err != nil || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(v) {
		return 0, 0, false
	}
	nv, okN := proofbound.IvVec3Of(n.Value)
	pv, okP := proofbound.IvVec3Of(p)
	if !okN || !okP {
		return 0, 0, false
	}
	allow := proofbound.AbsSumUpper(proofbound.ProductUpper(bound, pLen),
		proofbound.IntervalFloatError(proofbound.IvVec3Dot(nv, pv), v))
	if proofbound.IsNonFinite(allow) {
		return 0, 0, false
	}
	return v, allow, true
}

// PullLengthUpper bounds the length of a rounded unit pull. Normalization
// can leave its held length slightly above one, so a bound scaled by that
// length must use this upper bound rather than an assumed exact one.
func PullLengthUpper(p r3.Vec) (float64, bool) {
	pv, ok := proofbound.IvVec3Of(p)
	if !ok {
		return 0, false
	}
	length, okSqrt := proofbound.IntervalSqrt(proofbound.IvVec3NormSq(pv))
	if !okSqrt {
		return 0, false
	}
	up := proofbound.RatFloatUp(length.Hi)
	if proofbound.IsNonFinite(up) || up <= 0 {
		return 0, false
	}
	return up, true
}
