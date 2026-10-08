package revolvemesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
)

// AngularSequence encloses the sine and cosine stored at every angular sample.
// IdealBasis encloses the revolve axis basis before coordinate rounding.
// These checks implement docs/tessellation-design.md §8.

// AngularInput carries the revolve payload's held angles and denotation.
type AngularInput struct {
	Phi0, Phi1 float64
	Full       bool
	Den        revolveangle.Sweep
}

// AngularSequence builds the angular sequence for n chords, over the
// payload's own denotation (docs/evaluator-design.md §6): sample l's angle is
// enclosed as enc(phi0) + (l/n)·(enc(phi1) − enc(phi0)), so the stored
// cosine/sine is checked against the angle the RECORD denotes, not merely the
// held float the resolver rounded to. Wherever the payload's denotation
// cannot state an end exactly (den.Phi0/den.Phi1 invalid — a ToFaceAngular
// stop, a payload literal with none, or an angle unit this evaluator does not
// denote), this falls back to the prior reading over the held floats alone,
// which reproduces today's construction exactly: a partial sweep's angle
// φ0 + l·(φ1 − φ0)/n as an exact rational in the payload's own two floats
// (proofbound.RadSinCosInterval), a full turn starting at zero as l/n of a TURN
// (proofbound.TurnSinCosInterval, no π entering at all), and a full turn starting
// elsewhere as the same radian enclosure over φ0 + 2π·l/n, widened by the 2π
// enclosure's own (sub-2⁻²⁴⁰) width.
func AngularSequence(input AngularInput, n int) (RevolveAngular, error) {
	if n <= 0 {
		return RevolveAngular{}, fmt.Errorf(`%w: a revolve mesh needs at least one angular chord`, decaderr.ErrDegenerate)
	}
	phi0, phi1, full := input.Phi0, input.Phi1, input.Full
	r0, r1 := proofarith.FloatRat(phi0), proofarith.FloatRat(phi1)
	if r0 == nil || r1 == nil {
		return RevolveAngular{}, fmt.Errorf(`%w: the sweep interval is not finite, so no angular sample can be enclosed`, decaderr.ErrUnsupported)
	}
	enc0, ok0 := input.Den.Phi0.Enclosure()
	enc1, ok1 := input.Den.Phi1.Enclosure()
	haveDen := ok0 && ok1
	var diff proofbound.RatInterval
	if haveDen {
		diff = proofbound.IntervalSub(enc1, enc0)
	}
	out := RevolveAngular{N: n, Samples: n + 1}
	if full {
		out.Samples = n
	}
	nRat := new(big.Rat).SetInt64(int64(n))
	if full {
		out.Step = proofbound.IntervalScale(proofbound.TwoPiInterval(), new(big.Rat).Inv(nRat))
	} else {
		out.Step = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), nRat))
	}
	for l := range out.Samples {
		frac := new(big.Rat).SetFrac64(int64(l), int64(n))
		var cosIv, sinIv proofbound.RatInterval
		switch {
		case haveDen:
			angle := proofbound.IntervalAdd(enc0, proofbound.IntervalScale(diff, frac))
			var ok bool
			sinIv, cosIv, ok = proofbound.RadSinCosSpan(angle)
			if !ok {
				return RevolveAngular{}, ErrRevolveAngleEnclosure
			}
		case full && r0.Sign() == 0:
			sinIv, cosIv = proofbound.TurnSinCosInterval(frac)
		case full:
			angle := proofbound.IntervalAdd(proofbound.PointInterval(r0), proofbound.IntervalScale(proofbound.TwoPiInterval(), frac))
			var ok bool
			sinIv, cosIv, ok = proofbound.RadSinCosSpan(angle)
			if !ok {
				return RevolveAngular{}, ErrRevolveAngleEnclosure
			}
			// A full turn's last interval closes onto its first sample, so the
			// sequence never states φ1 and no seam ring is emitted.
		default:
			angle := new(big.Rat).Add(r0, new(big.Rat).Mul(frac, new(big.Rat).Sub(r1, r0)))
			var ok bool
			sinIv, cosIv, ok = proofbound.RadSinCosInterval(angle)
			if !ok {
				return RevolveAngular{}, ErrRevolveAngleEnclosure
			}
		}
		cosHeld, _ := new(big.Rat).Mul(new(big.Rat).Add(cosIv.Lo, cosIv.Hi), big.NewRat(1, 2)).Float64()
		sinHeld, _ := new(big.Rat).Mul(new(big.Rat).Add(sinIv.Lo, sinIv.Hi), big.NewRat(1, 2)).Float64()
		if proofbound.IsNonFinite(cosHeld) || proofbound.IsNonFinite(sinHeld) {
			return RevolveAngular{}, ErrRevolveAngleEnclosure
		}
		gap := math.Max(proofbound.IntervalFloatError(cosIv, cosHeld), proofbound.IntervalFloatError(sinIv, sinHeld))
		if proofbound.IsNonFinite(gap) || gap > RevolveTrigGapPrior {
			return RevolveAngular{}, fmt.Errorf(`%w: an angular sample's stored cosine and sine sit farther from the angle they denote than this mesh reserved for them`, decaderr.ErrUnsupported)
		}
		out.Cos = append(out.Cos, cosHeld)
		out.Sin = append(out.Sin, sinHeld)
		out.CosIv = append(out.CosIv, cosIv)
		out.SinIv = append(out.SinIv, sinIv)
		out.Gap = math.Max(out.Gap, gap)
	}
	return out, nil
}

// IdealBasis is docs/tessellation-design.md §8's axis basis as the EXACT
// expression the payload's own floats denote, rather than the float64 triple
// the build stores for it: a3 = O + aU·U + aV·V, w = dU·U + dV·V,
// e0 = −dV·U + dU·V and e1 = w × e0. The gap between this and the stored basis
// is one of the terms deltaC measures.
func IdealBasis(frame r3.Frame, aUHeld, aVHeld, dUHeld, dVHeld float64) (RevolveBasis3Iv, bool) {
	origin, ok0 := proofbound.IvVec3Of(frame.Origin())
	fu, ok1 := proofbound.IvVec3Of(frame.U())
	fv, ok2 := proofbound.IvVec3Of(frame.V())
	aU, aV := proofarith.FloatRat(aUHeld), proofarith.FloatRat(aVHeld)
	dU, dV := proofarith.FloatRat(dUHeld), proofarith.FloatRat(dVHeld)
	if !ok0 || !ok1 || !ok2 || aU == nil || aV == nil || dU == nil || dV == nil {
		return RevolveBasis3Iv{}, false
	}
	scale := func(v proofbound.IvVec3, s *big.Rat) proofbound.IvVec3 {
		return proofbound.IvVec3Mul(v, proofbound.PointInterval(s))
	}
	a3 := proofbound.IvVec3Add(origin, proofbound.IvVec3Add(scale(fu, aU), scale(fv, aV)))
	w := proofbound.IvVec3Add(scale(fu, dU), scale(fv, dV))
	e0 := proofbound.IvVec3Add(scale(fu, new(big.Rat).Neg(dV)), scale(fv, dU))
	return RevolveBasis3Iv{A3: a3, W: w, E0: e0, E1: proofbound.IvVec3Cross(w, e0)}, true
}
