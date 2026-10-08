package capband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// MiterLocusSubdivisions bounds the work and enclosure width for a mitered
// corner's offset path. Thirty-two ranges enclose a quarter-disk chamfer near
// its line-circle tangency without swamping its held chord.
const MiterLocusSubdivisions = 32

// MiterSliverSubdivisions is the number of offset sub-ranges a corner
// between two circular walls reads to bound its sliver
// (capcontour.LocusVelocityHull's SliverEnclosure). That bound's error falls
// as the square of the count; at 128 the asymmetric lens's two corner shares
// are charged within a few percent.
const MiterSliverSubdivisions = 128

// MiterLocusUpper bounds the length of the conic corner-foot path from the
// original corner to the cap-level foot. Each range charges one work step.
// axialUpper encloses the stated side setback, not the rounded level gap.
// A false result means that no finite enclosure could be built.
func MiterLocusUpper(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk,
	apexU, apexV, axialUpper, dc, dcDelta float64) (float64, bool, error) {
	if dc <= 0 || !(axialUpper >= 0) || proofbound.IsNonFinite(axialUpper) {
		return 0, false, nil
	}
	// The offset span includes the setback's unit-conversion rounding.
	// Its lower endpoint gives a conservative axial rate.
	span, rate := dc, dc
	if dcDelta > 0 {
		span = proofbound.UpRound(dc + dcDelta)
		rate = freeform.DownRound(dc - dcDelta)
		if rate <= 0 {
			return 0, false, nil
		}
	}
	total := 0.0
	for _, r := range MiterLocusRanges(span) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		t0, t1 := r[0], r[1]
		speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, t0, t1, apexU, apexV)
		if !ok {
			return 0, false, nil
		}
		// Width encloses the sub-range's exact width. The axial rise is
		// rounded up at the conservative rate over the same sub-range.
		width := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(t1), proofarith.FloatRat(t0)))
		axial := proofbound.DivUpper(proofbound.ProductUpper(axialUpper, width), rate)
		total = proofbound.AbsSumUpper(total, proofbound.ChordLocusLengthAllow(speed, width, axial, 0))
	}
	return total, true, nil
}

// MiterLocusSliverFlux bounds the flux one mitered corner's sliver adds to the
// chord-locus term of the circular wall patch centred at (cU, cV)
// (proofbound.ChordLocusVolumeAllow's corner term): the axial rate times the
// sliver's in-plane moment about that centre. prev and cur are the corner's
// two walls, and (apexU, apexV) is the corner itself. axialUpper encloses the
// stated side setback; dc and dcDelta are the cap setback and its
// unit-conversion rounding, and the moment is taken over every setback that
// span holds. A line-line corner has an affine locus and answers zero.
//
// A corner with one straight wall reads the closed form
// (capcontour.LineCircleLocusSliverMoment), where the straight wall's
// distance from the centre is the moment arm. A corner between two circular
// walls encloses the locus velocity over each of the MiterSliverSubdivisions
// ranges (capcontour.CircleCircleLocusVelocity) and the corner foot at each
// range end (capcontour.LocusFoot), encloses the sliver from them
// (capcontour.LocusVelocityHull), and crosses it with the corner's own arm
// from the centre (circleCircleSliverMoment). Each range, and the
// closed form, charges one work step. A false result means no finite bound
// could be built.
func MiterLocusSliverFlux(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk,
	cU, cV, apexU, apexV, axialUpper, dc, dcDelta float64) (float64, bool, error) {
	if dc <= 0 || !(axialUpper >= 0) || proofbound.IsNonFinite(axialUpper) {
		return 0, false, nil
	}
	if !prev.IsCircular() && !cur.IsCircular() {
		return 0, true, nil
	}
	span, rate := dc, dc
	if dcDelta > 0 {
		span = proofbound.UpRound(dc + dcDelta)
		rate = freeform.DownRound(dc - dcDelta)
		if rate <= 0 {
			return 0, false, nil
		}
	}
	var moment float64
	var ok bool
	switch {
	case !prev.IsCircular() || !cur.IsCircular():
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		line, circle := prev, cur
		if prev.IsCircular() {
			line, circle = cur, prev
		}
		moment, ok = capcontour.LineCircleLocusSliverMoment(line, circle, rate, span)
	default:
		var err error
		moment, ok, err = circleCircleSliverMoment(budget, prev, cur, cU, cV, apexU, apexV, rate, span)
		if err != nil {
			return 0, false, err
		}
	}
	if !ok {
		return 0, false, nil
	}
	flux := proofbound.ChordLocusCornerFlux(axialUpper, rate, moment)
	if proofbound.IsNonFinite(flux) {
		return 0, false, nil
	}
	return flux, true, nil
}

// circleCircleSliverMoment bounds |(v − c) × W| for a corner between two
// circular walls, for every setback in [lo, span], as the smaller of two
// proven bounds. The first crosses the exact arm v − c with the per-range
// enclosure of W's two components (capcontour.LocusVelocityHull's
// SliverEnclosure), in exact interval arithmetic. The second is |v − c|,
// rounded up from the exact squared distance, times the hull's bound on |W|
// (SliverUpper), which reads only the hull of the boxes.
func circleCircleSliverMoment(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk,
	cU, cV, apexU, apexV, lo, span float64) (float64, bool, error) {
	var hull capcontour.LocusVelocityHull
	start, ok := capcontour.LocusFoot(prev, cur, 0, apexU, apexV)
	if !ok {
		return 0, false, nil
	}
	for _, r := range locusRanges(span, MiterSliverSubdivisions) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		box, ok := capcontour.CircleCircleLocusVelocity(prev, cur, r[0], r[1], apexU, apexV)
		if !ok {
			return 0, false, nil
		}
		end, ok := capcontour.LocusFoot(prev, cur, r[1], apexU, apexV)
		if !ok {
			return 0, false, nil
		}
		hull.Add(r[0], r[1], start, end, box)
		start = end
	}
	sliver, ok := hull.SliverUpper(span)
	if !ok {
		return 0, false, nil
	}
	squared, ok := proofarith.DySquaredDistance3(apexU, apexV, 0, cU, cV, 0)
	if !ok {
		return 0, false, nil
	}
	moment := proofbound.ProductUpper(proofbound.RatSqrtUp(squared.Rat()), sliver)
	w, ok := hull.SliverEnclosure(lo, span)
	if !ok {
		return moment, true, nil
	}
	armU := new(big.Rat).Sub(proofarith.FloatRat(apexU), proofarith.FloatRat(cU))
	armV := new(big.Rat).Sub(proofarith.FloatRat(apexV), proofarith.FloatRat(cV))
	cross := proofbound.IntervalSub(proofbound.IntervalScale(w.V, armU), proofbound.IntervalScale(w.U, armV))
	return math.Min(moment, proofbound.RatFloatUp(proofbound.IntervalAbsUpper(cross))), true, nil
}

// MiterLocusRanges tiles [0, span] using shared float endpoints. The last
// endpoint is span, so rounding cannot leave a gap between ranges.
func MiterLocusRanges(span float64) [MiterLocusSubdivisions][2]float64 {
	var ranges [MiterLocusSubdivisions][2]float64
	copy(ranges[:], locusRanges(span, MiterLocusSubdivisions))
	return ranges
}

// locusRanges tiles [0, span] into n ranges with shared float endpoints, the
// last ending at span exactly.
func locusRanges(span float64, n int) [][2]float64 {
	ranges := make([][2]float64, n)
	step := span / float64(n)
	t0 := 0.0
	for k := range n {
		t1 := float64(k+1) * step
		if k == n-1 {
			t1 = span
		}
		ranges[k] = [2]float64{t0, t1}
		t0 = t1
	}
	return ranges
}

// CornerLocusSpans encloses the corner-foot locus of the corner between the
// walls prev and cur, at (apexU, apexV), over each of the
// MiterLocusSubdivisions ranges of [0, dc]: the locus's angle about the
// circular patch's centre (cU, cV), measured counter-clockwise from the ray
// through (sideU, sideV), the patch's own side directrix end at this corner.
//
// The foot at each range end is enclosed by capcontour.LocusFoot, whose
// nearest root is the locus wherever the corner's sliver was bounded
// (MiterLocusSliverFlux), and the angle of a box is the hull of its four
// corners' angles: the directions a convex box subtends from a point outside
// it run between two of its vertices. Every corner must lie less than a
// quarter turn from the ray, which also keeps the centre out of the box. The
// locus's angle is monotone in the offset amount while the carriers do not
// touch (proofbound.ChordLocusVolumeAllow derives it), so over each range it
// lies between its two ends' angles. Each foot charges one work step. ok is
// false where a foot is not enclosed or a corner leaves the quarter turn.
func CornerLocusSpans(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk,
	cU, cV, sideU, sideV, apexU, apexV, dc float64) ([]LocusSpan, bool, error) {
	if !(dc > 0) || proofbound.IsNonFinite(dc) {
		return nil, false, nil
	}
	rc := func(x float64) *big.Rat { return proofarith.FloatRat(x) }
	ru, rv, rsu, rsv := rc(cU), rc(cV), rc(sideU), rc(sideV)
	if ru == nil || rv == nil || rsu == nil || rsv == nil {
		return nil, false, nil
	}
	au, av := new(big.Rat).Sub(rsu, ru), new(big.Rat).Sub(rsv, rv)
	angle := func(t float64) (*big.Rat, *big.Rat, bool, error) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, false, err
		}
		foot, ok := capcontour.LocusFoot(prev, cur, t, apexU, apexV)
		if !ok {
			return nil, nil, false, nil
		}
		var lo, hi *big.Rat
		for _, pu := range []*big.Rat{foot.U.Lo, foot.U.Hi} {
			for _, pv := range []*big.Rat{foot.V.Lo, foot.V.Hi} {
				du, dv := new(big.Rat).Sub(pu, ru), new(big.Rat).Sub(pv, rv)
				cross := new(big.Rat).Sub(new(big.Rat).Mul(au, dv), new(big.Rat).Mul(av, du))
				dot := new(big.Rat).Add(new(big.Rat).Mul(au, du), new(big.Rat).Mul(av, dv))
				if dot.Sign() <= 0 {
					return nil, nil, false, nil
				}
				a := proofbound.Atan2Interval(cross, dot, false)
				if lo == nil || a.Lo.Cmp(lo) < 0 {
					lo = a.Lo
				}
				if hi == nil || a.Hi.Cmp(hi) > 0 {
					hi = a.Hi
				}
			}
		}
		return lo, hi, true, nil
	}
	ranges := MiterLocusRanges(dc)
	spans := make([]LocusSpan, 0, len(ranges))
	prevLo, prevHi, ok, err := angle(0)
	if err != nil || !ok {
		return nil, false, err
	}
	for _, r := range ranges {
		lo, hi, ok, err := angle(r[1])
		if err != nil || !ok {
			return nil, false, err
		}
		spanLo, spanHi := prevLo, prevHi
		if lo.Cmp(spanLo) < 0 {
			spanLo = lo
		}
		if hi.Cmp(spanHi) > 0 {
			spanHi = hi
		}
		spans = append(spans, LocusSpan{T0: r[0], T1: r[1],
			Lo: proofbound.RatFloatDown(spanLo), Hi: proofbound.RatFloatUp(spanHi)})
		prevLo, prevHi = lo, hi
	}
	return spans, true, nil
}

// StraightLocusSpans is the span list of a corner whose locus runs along the
// circular wall's own radial through the corner, a reflex foot or a G1 join:
// its angle from that ray is zero over the whole range [0, dc].
func StraightLocusSpans(dc float64) []LocusSpan {
	return []LocusSpan{{T0: 0, T1: dc}}
}
