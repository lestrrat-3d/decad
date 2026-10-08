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

// LocusVelocityHull holds, for each offset sub-range [t0, t1] of a
// corner-foot locus in the order the sub-ranges tile [0, span], the
// enclosures of the locus position at t0 and at t1 and the box holding every
// velocity the locus has on the sub-range, together with the smallest
// axis-aligned box holding every velocity box. The zero value holds nothing.
type LocusVelocityHull struct {
	ranges             []locusVelocityRange
	loU, hiU, loV, hiV *big.Rat
}

// locusVelocityRange is one offset sub-range [t0, t1], the locus position
// enclosures at its two ends, and its velocity box.
type locusVelocityRange struct {
	t0, t1     *big.Rat
	start, end Point
	box        Point
}

// Add records one sub-range: start and end enclose the locus position at
// offsets t0 and t1, and box holds every velocity the locus has between them.
// It grows the hull to hold box. Each enclosure is first widened outward to
// float64 ends, which keeps the exact sums SliverEnclosure forms from growing
// their denominators range by range. A bound that does not lift is recorded
// as a gap, which SliverEnclosure refuses.
func (h *LocusVelocityHull) Add(t0, t1 float64, start, end, box Point) {
	start, end, box = floatOutward(start), floatOutward(end), floatOutward(box)
	h.ranges = append(h.ranges, locusVelocityRange{
		t0: proofarith.FloatRat(t0), t1: proofarith.FloatRat(t1), start: start, end: end, box: box,
	})
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
// so ∫₀^dc Q dt = dc·(P(0) + P(dc))/2, and integrating ∫₀^dc P dt by parts
// gives W = ∫₀^dc (dc/2 − τ)·P'(τ) dτ. The weight dc/2 − τ integrates to
// zero, so W = ∫₀^dc (dc/2 − τ)·(P'(τ) − m) dτ for the hull's centre m too.
// Every P'(τ) lies within half the hull's diagonal D of m, and |dc/2 − τ|
// integrates to dc²/4, so |W| is at most D·dc²/8 ≤ D·span²/8. An empty hull
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
	w.Quo(w, big.NewRat(8, 1))
	upper := proofbound.RatFloatUp(w)
	if proofbound.IsNonFinite(upper) {
		return 0, false
	}
	return upper, true
}

// SliverEnclosure encloses each component of W(dc) = ∫₀^dc (P(t) − Q(t)) dt
// for every dc in [lo, hi], from the per-range positions and velocity boxes
// rather than their hull. The recorded sub-ranges must tile [0, hi] in order,
// with no gap and no overlap.
//
// As SliverUpper derives, W(dc) = ∫₀^dc (dc/2 − τ)·P'(τ) dτ. A sub-range
// [t0, t1] that ends at or before lo lies inside [0, dc] for every dc. With
// mid its midpoint, its share splits as
//
//	(dc/2 − mid)·(P(t1) − P(t0)) + ∫ (mid − τ)·(P'(τ) − m) dτ,
//
// the second integral taken over the sub-range, where m is the centre of its
// velocity box: mid − τ integrates to zero there, so subtracting m changes
// nothing. The first term reads the two position enclosures, with the weight
// dc/2 − mid boxed over [lo, hi]. In the second, each component of P' − m is
// at most the box's half-width h, and |mid − τ| integrates to (t1 − t0)²/4,
// so each component is at most (t1 − t0)²·h/4. That error is cubic in the
// sub-range's width once the box's own width shrinks with it, so it falls
// as the square of the number of sub-ranges.
//
// The sub-ranges past lo tile [L, hi], with L the end of the last one that
// ended at or before lo, and dc may stop anywhere in them. On [L, dc],
// dc/2 − τ = −L/2 + ((L + dc)/2 − τ), so the remainder is
// −(L/2)·(P(dc) − P(L)) plus ∫ ((L + dc)/2 − τ)·(P'(τ) − m') dτ, with m' the
// centre of the hull of those sub-ranges' velocity boxes. P(dc) − P(L) is
// (dc − L) times an average velocity, which lies in that hull, and dc − L
// lies in [0, hi − L]; the integral is at most (hi − L)²/4 times the hull's
// half-width per component. With lo = hi no sub-range is past lo and the
// remainder is empty.
//
// Everything is exact rational interval arithmetic. ok is false for an empty
// hull, a bound that does not lift, sub-ranges that do not tile [0, hi], or
// 0 < lo <= hi failing.
func (h LocusVelocityHull) SliverEnclosure(lo, hi float64) (Point, bool) {
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if len(h.ranges) == 0 || rlo == nil || rhi == nil || rlo.Sign() <= 0 || rlo.Cmp(rhi) > 0 {
		return Point{}, false
	}
	at := new(big.Rat)
	for _, r := range h.ranges {
		if r.t0 == nil || r.t1 == nil || r.t0.Cmp(at) != 0 || r.t1.Cmp(r.t0) < 0 {
			return Point{}, false
		}
		at = r.t1
	}
	if at.Cmp(rhi) != 0 {
		return Point{}, false
	}
	half, quarter := big.NewRat(1, 2), big.NewRat(1, 4)
	cLo, cHi := new(big.Rat).Mul(rlo, half), new(big.Rat).Mul(rhi, half)
	zero := proofbound.PointInterval(new(big.Rat))
	sumU, sumV := zero, zero
	radU, radV := new(big.Rat), new(big.Rat)
	split := new(big.Rat)
	var past []Point
	for _, r := range h.ranges {
		if r.t1.Cmp(rlo) > 0 {
			past = append(past, r.box)
			continue
		}
		split = r.t1
		mid := new(big.Rat).Mul(new(big.Rat).Add(r.t0, r.t1), half)
		weight := proofbound.Interval(new(big.Rat).Sub(cLo, mid), new(big.Rat).Sub(cHi, mid))
		sumU = proofbound.IntervalAdd(sumU, proofbound.IntervalMul(weight, proofbound.IntervalSub(r.end.U, r.start.U)))
		sumV = proofbound.IntervalAdd(sumV, proofbound.IntervalMul(weight, proofbound.IntervalSub(r.end.V, r.start.V)))
		length := new(big.Rat).Sub(r.t1, r.t0)
		spread := new(big.Rat).Mul(new(big.Rat).Mul(length, length), quarter)
		radU.Add(radU, new(big.Rat).Mul(spread, intervalHalfWidth(r.box.U)))
		radV.Add(radV, new(big.Rat).Mul(spread, intervalHalfWidth(r.box.V)))
	}
	if len(past) > 0 {
		hullU, hullV := past[0].U, past[0].V
		for _, b := range past[1:] {
			hullU, hullV = IntervalHull(hullU, b.U), IntervalHull(hullV, b.V)
		}
		reach := new(big.Rat).Sub(rhi, split)
		run := proofbound.Interval(new(big.Rat), reach)
		lead := new(big.Rat).Neg(new(big.Rat).Mul(split, half))
		sumU = proofbound.IntervalAdd(sumU, proofbound.IntervalScale(proofbound.IntervalMul(run, hullU), lead))
		sumV = proofbound.IntervalAdd(sumV, proofbound.IntervalScale(proofbound.IntervalMul(run, hullV), lead))
		spread := new(big.Rat).Mul(new(big.Rat).Mul(reach, reach), quarter)
		radU.Add(radU, new(big.Rat).Mul(spread, intervalHalfWidth(hullU)))
		radV.Add(radV, new(big.Rat).Mul(spread, intervalHalfWidth(hullV)))
	}
	return Point{U: proofbound.IntervalWiden(sumU, radU), V: proofbound.IntervalWiden(sumV, radV)}, true
}

// CircleCircleLocusFoot encloses the corner foot where two circular walls'
// offset carriers meet at the single offset amount t: the root nearest the
// corner (vU, vV), the same root CircleCircleLocusVelocity encloses over a
// range. That root is the denoted locus. The two roots are mirror images
// across the line through the two centres, so the one nearer the corner is
// the one on the corner's side of that line; the locus starts at the corner
// and moves continuously, so it stays on that side until the two roots meet
// on the line, where the carriers touch. CircleCircleLocusVelocity refuses
// any offset range whose enclosed constraint determinant, the sine of the
// angle between the two radii to the root, reaches zero, which it does only
// for a root on that line, so wherever every range over [0, t] was enclosed
// the nearest root is the locus. ok is false where a carrier does not lift or no root is decided.
func CircleCircleLocusFoot(prev, cur survey2d.SideWalk, t, vU, vV float64) (Point, bool) {
	ca, okA := carrierOverRange(prev, t, t)
	cb, okB := carrierOverRange(cur, t, t)
	if !okA || !okB {
		return Point{}, false
	}
	cands, ok := Intersect(ca, cb)
	if !ok {
		return Point{}, false
	}
	return Nearest(cands, vU, vV)
}

func intervalHalfWidth(a proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Sub(a.Hi, a.Lo), big.NewRat(1, 2))
}

// floatOutward widens each component of p to the nearest float64 ends at or
// beyond it. A component past the float64 range is kept exact.
func floatOutward(p Point) Point {
	out := func(a proofbound.RatInterval) proofbound.RatInterval {
		lo, hi := proofbound.RatFloatDown(a.Lo), proofbound.RatFloatUp(a.Hi)
		if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
			return a
		}
		return proofbound.Interval(new(big.Rat).SetFloat64(lo), new(big.Rat).SetFloat64(hi))
	}
	return Point{U: out(p.U), V: out(p.V)}
}
