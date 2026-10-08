package capcontour

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// LineCircleLocusSliverMoment bounds the in-plane sliver moment of a corner
// where a straight wall meets a circular one, for every setback dc in
// [lo, hi]: |(v − c) × W|, where v is the corner, c the circle's centre and
// W = ∫₀^dc (P(t) − Q(t)) dt. P is the corner foot's denoted locus (the two
// offset carriers' intersection at offset amount t) and Q is the chord from
// P(0) = v to P(dc), ridden at the same rate t. A Cone patch's chord-locus
// volume term charges this moment (docs/modify-reach-design.md §8.3).
//
// The derivation reads LineCircleLocusSpeedUpper's frame. The foot is
// P(t) = anchor + t·n + s(t)·e with s(t) = −β ± √Δ(t), and Δ(t) = Δ0 + Δ1·t
// is affine. The chord has the same n component t, so P − Q runs along e and
// |W| = ∫₀^dc (y − y_chord) dt for y = √Δ on one branch, a concave function.
// With y0 = y(0) and y1 = y(dc), ∫y = (2dc/3)·(y0² + y0·y1 + y1²)/(y0 + y1)
// and the chord integrates to dc·(y0 + y1)/2, so
// |W| = dc·(y0 − y1)²/(6·(y0 + y1)) = dc³·Δ1²/(6·(y0 + y1)³), since
// y0² − y1² = −Δ1·dc. The corner v lies on the line, so
// |(v − c) × e| = |(anchor − c)·n| = |alpha|, the centre's distance from the
// line, and the moment is |alpha|·|W|.
//
// The bound takes Δ1² and |alpha| at the top of their enclosures and hi³ for
// dc³. Δ is affine and positive over [0, hi], so y is monotone in t, and for
// every dc in [lo, hi] the root y1 is at least min(y(lo), y(hi)), each root
// rounded down. A persistent tangency (Δ1 exactly zero) has an affine locus
// and answers zero. ok is false where the enclosure of Δ reaches zero at an
// end of [0, hi] (a fold, the same refusal LineCircleLocusSpeedUpper makes),
// where 0 < lo <= hi fails, or where a number does not lift.
func LineCircleLocusSliverMoment(line, circle survey2d.SideWalk, lo, hi float64) (float64, bool) {
	k, ok := lineCircleCornerOf(line, circle)
	if !ok {
		return 0, false
	}
	if k.delta1.Lo.Sign() == 0 && k.delta1.Hi.Sign() == 0 {
		return 0, true
	}
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil || rlo.Sign() <= 0 || rlo.Cmp(rhi) > 0 {
		return 0, false
	}
	d0, dLo, dHi := k.delta0, k.deltaAt(rlo), k.deltaAt(rhi)
	if d0.Lo.Sign() <= 0 || dLo.Lo.Sign() <= 0 || dHi.Lo.Sign() <= 0 {
		return 0, false
	}
	y0 := proofarith.FloatRat(proofbound.RatSqrtDown(d0.Lo))
	yLo := proofarith.FloatRat(proofbound.RatSqrtDown(dLo.Lo))
	yHi := proofarith.FloatRat(proofbound.RatSqrtDown(dHi.Lo))
	if y0 == nil || yLo == nil || yHi == nil || y0.Sign() <= 0 || yLo.Sign() <= 0 || yHi.Sign() <= 0 {
		return 0, false
	}
	y1 := yLo
	if yHi.Cmp(y1) < 0 {
		y1 = yHi
	}
	sum := new(big.Rat).Add(y0, y1)
	d1 := proofbound.IntervalAbsUpper(k.delta1)
	num := new(big.Rat).Mul(new(big.Rat).Mul(rhi, rhi), rhi)
	num.Mul(num, new(big.Rat).Mul(d1, d1))
	num.Mul(num, proofbound.IntervalAbsUpper(k.alpha))
	den := new(big.Rat).Mul(new(big.Rat).Mul(sum, sum), sum)
	den.Mul(den, big.NewRat(6, 1))
	moment := proofbound.RatFloatUp(new(big.Rat).Quo(num, den))
	if proofbound.IsNonFinite(moment) {
		return 0, false
	}
	return moment, true
}

// LocusVelocityHull is the smallest axis-aligned box holding every velocity
// box it has been given. The zero value holds nothing.
type LocusVelocityHull struct {
	loU, hiU, loV, hiV *big.Rat
}

// Add grows the hull to hold box.
func (h *LocusVelocityHull) Add(box Point) {
	if h.loU == nil {
		h.loU, h.hiU = new(big.Rat).Set(box.U.Lo), new(big.Rat).Set(box.U.Hi)
		h.loV, h.hiV = new(big.Rat).Set(box.V.Lo), new(big.Rat).Set(box.V.Hi)
		return
	}
	if box.U.Lo.Cmp(h.loU) < 0 {
		h.loU.Set(box.U.Lo)
	}
	if box.U.Hi.Cmp(h.hiU) > 0 {
		h.hiU.Set(box.U.Hi)
	}
	if box.V.Lo.Cmp(h.loV) < 0 {
		h.loV.Set(box.V.Lo)
	}
	if box.V.Hi.Cmp(h.hiV) > 0 {
		h.hiV.Set(box.V.Hi)
	}
}

// SliverUpper bounds |W| = |∫₀^dc (P(t) − Q(t)) dt| for every dc in
// (0, span], given that the hull holds every velocity P'(t) the locus has for
// t in [0, span]. Q is the chord from P(0) to P(dc) ridden at the same rate t,
// so P(t) − Q(t) = ∫₀^t (P'(τ) − m) dτ = −∫_t^dc (P'(τ) − m) dτ, where m is
// the chord's own velocity, the mean of P' over [0, dc]. Both P'(τ) and m
// lie in the hull, so |P(t) − Q(t)| is at most min(t, dc − t) times the
// hull's diagonal D, and |W| is at most D·dc²/4 ≤ D·span²/4. An empty hull
// answers false.
func (h LocusVelocityHull) SliverUpper(span float64) (float64, bool) {
	rspan := proofarith.FloatRat(span)
	if h.loU == nil || rspan == nil || rspan.Sign() <= 0 {
		return 0, false
	}
	du := new(big.Rat).Sub(h.hiU, h.loU)
	dv := new(big.Rat).Sub(h.hiV, h.loV)
	diag := proofarith.FloatRat(proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))))
	if diag == nil {
		return 0, false
	}
	w := new(big.Rat).Mul(diag, new(big.Rat).Mul(rspan, rspan))
	w.Quo(w, big.NewRat(4, 1))
	upper := proofbound.RatFloatUp(w)
	if proofbound.IsNonFinite(upper) {
		return 0, false
	}
	return upper, true
}
