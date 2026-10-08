package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file states how far a circular patch's held numbers sit from the values
// the band's closed surface reads there (HeldAllow), and encloses the patch's
// closed forms over every value those allowances admit.

// AngleAllow is a proven upper bound on |held − θ|, where θ is the exact angle
// about (cU, cV) of a point within reach of p, on the branch nearest held.
//
// p and the centre are float64s, so the direction p − c is an exact rational
// and proofbound.Atan2Interval encloses its angle with no libm accuracy
// assumed. A point within reach of p turns that angle by at most
// arcsin(reach/|p − c|) ≤ (π/2)·reach/|p − c|, which widens the enclosure; a
// reach at or past |p − c| says nothing about the angle and answers false, as
// does p on the centre.
//
// The branch is the one nearest held. Every held angle a cap band holds is a
// float Atan2 of the same direction, or such an angle unwrapped by whole turns,
// so it lies within a few ulps of its own branch and more than π from any
// other.
func AngleAllow(cU, cV float64, p Point, reach, held float64) (float64, bool) {
	dU, dV, ok := exactDelta(p, cU, cV)
	if !ok || (dU.Sign() == 0 && dV.Sign() == 0) || !(reach >= 0) || proofbound.IsNonFinite(reach) || proofbound.IsNonFinite(held) {
		return 0, false
	}
	iv := proofbound.Atan2Interval(dV, dU, false)
	mid, _ := intervalMid(iv).Float64()
	if turns := math.Round((held - mid) / (2 * math.Pi)); turns != 0 {
		iv = proofbound.IntervalAdd(iv, proofbound.IntervalScale(proofbound.TwoPiInterval(), new(big.Rat).SetFloat64(turns)))
	}
	if reach > 0 {
		rhoLower := proofbound.RatSqrtDown(new(big.Rat).Add(new(big.Rat).Mul(dU, dU), new(big.Rat).Mul(dV, dV)))
		if !(reach < rhoLower) {
			return 0, false
		}
		turn := new(big.Rat).Quo(
			proofbound.RatMul(proofbound.PiUpper, proofarith.FloatRat(reach)),
			proofbound.RatMul(big.NewRat(2, 1), proofarith.FloatRat(rhoLower)),
		)
		iv = proofbound.IntervalWiden(iv, turn)
	}
	return proofbound.IntervalFloatError(iv, held), true
}

// RadiusAllow encloses the distances from (cU, cV) to the points pts, each
// read exactly from its float64 coordinates with outward-rounded square roots.
// reach is a proven upper bound on how far any of those distances sits from
// held, and spread one on how far any two of them sit apart. ok is false where
// a coordinate does not lift or no point is given.
func RadiusAllow(cU, cV, held float64, pts ...Point) (float64, float64, bool) {
	if len(pts) == 0 || proofbound.IsNonFinite(held) {
		return 0, 0, false
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, p := range pts {
		dU, dV, ok := exactDelta(p, cU, cV)
		if !ok {
			return 0, 0, false
		}
		sq := new(big.Rat).Add(new(big.Rat).Mul(dU, dU), new(big.Rat).Mul(dV, dV))
		lo = math.Min(lo, proofbound.RatSqrtDown(sq))
		hi = math.Max(hi, proofbound.RatSqrtUp(sq))
	}
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return 0, 0, false
	}
	reach := math.Max(proofbound.UpRound(hi-held), proofbound.UpRound(held-lo))
	return math.Max(reach, 0), math.Max(proofbound.UpRound(hi-lo), 0), true
}

func exactDelta(p Point, cU, cV float64) (*big.Rat, *big.Rat, bool) {
	pu, pv := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	ru, rv := proofarith.FloatRat(cU), proofarith.FloatRat(cV)
	if pu == nil || pv == nil || ru == nil || rv == nil {
		return nil, nil, false
	}
	return new(big.Rat).Sub(pu, ru), new(big.Rat).Sub(pv, rv), true
}

// heldBox is the interval [x − e, x + e] over exact rationals. ok is false
// where x does not lift or e is not a finite non-negative float.
func heldBox(x, e float64) (proofbound.RatInterval, bool) {
	rx := proofarith.FloatRat(x)
	if rx == nil || !(e >= 0) || proofbound.IsNonFinite(e) {
		return proofbound.RatInterval{}, false
	}
	if e == 0 {
		return proofbound.PointInterval(rx), true
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(rx), proofarith.FloatRat(e)), true
}

// widenBy grows an enclosure by a finite non-negative float on both ends.
func widenBy(iv proofbound.RatInterval, e float64) proofbound.RatInterval {
	if e == 0 {
		return iv
	}
	return proofbound.IntervalWiden(iv, proofarith.FloatRat(e))
}

// finite reports whether every field of h is a finite non-negative float.
func (h HeldAllow) finite() bool {
	for _, e := range [...]float64{h.Th0, h.Th1, h.CapTh0, h.CapTh1, h.SideRadius, h.CapRadius} {
		if !(e >= 0) || proofbound.IsNonFinite(e) {
			return false
		}
	}
	return true
}

// phaseAllow bounds how far the phase k·θS + m·θC moves at either end of the
// window when each angle moves by its own allowance.
func (h HeldAllow) phaseAllow(k, m int) float64 {
	ak, am := math.Abs(float64(k)), math.Abs(float64(m))
	return math.Max(
		proofbound.AbsSumUpper(proofbound.ProductUpper(ak, h.Th0), proofbound.ProductUpper(am, h.CapTh0)),
		proofbound.AbsSumUpper(proofbound.ProductUpper(ak, h.Th1), proofbound.ProductUpper(am, h.CapTh1)),
	)
}
