package capband

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// This file is agent-docs/tessellation-reach-design.md §7.1: a chamfer band's
// whole arc whose recorded End is off the circle its Start states. The record
// denotes Start's radius at End's angle and closes the junction with the
// radial chord to End (agent-docs/evaluator-design.md §4), so its section X
// holds a step a sketch solve's rounding leaves there. The proof restates the
// arc on the centre C′ of the perpendicular bisector of Start and End nearest
// the recorded Centre: both recorded ends lie on that circle exactly, so the
// restated section X′ has no step, and its corners are read as exactly G1 or
// near-tangent joins. The body X denotes and the body X′ denotes differ by a
// volume capblend.OffRadiusVolume charges, and the
// ideal polyhedron's stations on X′ sit within OffRadiusShift of those on X.
// Every test here is exact rational arithmetic over the record's floats.

// restateRefusal is the reason an off-radius arc admits no restatement: it
// shares its walk with another recorded segment, its recorded ends are too
// close for the bisector centre to stay within the bound OffRadiusShift
// states, or a number does not lift.
const restateRefusal = `an arc whose recorded end is not on the circle its start states, and whose restatement on one circle through both ends this evaluator does not bound`

// offRadius reports whether arc is a whole arc whose recorded End is off the
// circle its Start states: its two exact squared radii about the recorded
// Centre differ.
func offRadius(arc sectionrecord.ArcSeg) bool {
	if !naturalRange(arc.TStart, arc.TEnd) {
		return false
	}
	start := squaredRadius(arc.Start, arc.Center)
	end := squaredRadius(arc.End, arc.Center)
	return start != nil && end != nil && start.Cmp(end) != 0
}

// restatedCentre is the centre a corner reads arc about: the recorded Centre
// for an arc whose two ends lie on one circle about it, and otherwise
// C′ = C + μ·(End − Start), μ = (|End − C|² − |Start − C|²) / (2·|End − Start|²),
// the point of the two ends' perpendicular bisector nearest C. ok is false
// for a coordinate that denotes no rational or two equal ends.
func restatedCentre(arc sectionrecord.ArcSeg) ([2]*big.Rat, bool) {
	c, ok := ratPoint2(arc.Center)
	if !ok {
		return [2]*big.Rat{}, false
	}
	if !offRadius(arc) {
		return c, true
	}
	s, okS := ratPoint2(arc.Start)
	e, okE := ratPoint2(arc.End)
	if !okS || !okE {
		return [2]*big.Rat{}, false
	}
	chord := [2]*big.Rat{new(big.Rat).Sub(e[0], s[0]), new(big.Rat).Sub(e[1], s[1])}
	chordSq := ratDot(chord, chord)
	if chordSq.Sign() == 0 {
		return [2]*big.Rat{}, false
	}
	mu := new(big.Rat).Sub(squaredRadius(arc.End, arc.Center), squaredRadius(arc.Start, arc.Center))
	mu.Quo(mu, new(big.Rat).Mul(big.NewRat(2, 1), chordSq))
	return [2]*big.Rat{
		new(big.Rat).Add(c[0], new(big.Rat).Mul(mu, chord[0])),
		new(big.Rat).Add(c[1], new(big.Rat).Mul(mu, chord[1])),
	}, true
}

// OffRadiusShift bounds, for a whole arc whose recorded End is off its
// Start's circle, the distance between the point the record denotes and the
// point the restated arc denotes at every fraction φ of the two windows and
// every in-plane offset τ with |τ| <= reach:
//
//	P(φ, τ)  = C  + (r  − ιτ)·e(θ_S  + φ·W)
//	P′(φ, τ) = C′ + (r′ − ιτ)·e(θ′_S + φ·W′)
//
// With c = |C − C′|, |r − r′| <= c (both radii reach Start), and each end's
// angle about C and about C′ differs by at most (π/2)·c/r′ (the sine of the
// angle at Start between Start − C and Start − C′ is
// |(Start − C) × (C − C′)| / (r·r′) <= c/r′, and at End likewise), so the
// angle at every fraction does too, and
//
//	|P − P′| <= c + c + (R_hi + reach)·(π/2)·c/r′_lo
//
// with R_hi the larger recorded radius. It is zero for an arc whose ends lie
// on one circle, and ok is false for an arc this bound does not cover: c must
// stay below r′/2, and π·c/r′ below |End − Start|/R_hi, the least window a
// chord of that length subtends from either side, so both windows run the
// same way round. c reads |μ|·|End − Start| = |r_E² − r_S²| / (2·|End − Start|).
func OffRadiusShift(arc sectionrecord.ArcSeg, reach float64) (float64, bool) {
	if !offRadius(arc) {
		return 0, true
	}
	reachRat := proofarith.FloatRat(math.Abs(reach))
	s, okS := ratPoint2(arc.Start)
	e, okE := ratPoint2(arc.End)
	if reachRat == nil || !okS || !okE {
		return 0, false
	}
	centre, ok := restatedCentre(arc)
	if !ok {
		return 0, false
	}
	chord := [2]*big.Rat{new(big.Rat).Sub(e[0], s[0]), new(big.Rat).Sub(e[1], s[1])}
	chordLo := proofarith.FloatRat(proofbound.RatSqrtDown(ratDot(chord, chord)))
	startSq := squaredRadius(arc.Start, arc.Center)
	endSq := squaredRadius(arc.End, arc.Center)
	ds := [2]*big.Rat{new(big.Rat).Sub(s[0], centre[0]), new(big.Rat).Sub(s[1], centre[1])}
	restatedLo := proofarith.FloatRat(proofbound.RatSqrtDown(ratDot(ds, ds)))
	rHi := proofarith.FloatRat(proofbound.RatSqrtUp(startSq))
	if endHi := proofarith.FloatRat(proofbound.RatSqrtUp(endSq)); rHi != nil && endHi != nil && endHi.Cmp(rHi) > 0 {
		rHi = endHi
	}
	if chordLo == nil || chordLo.Sign() <= 0 || restatedLo == nil || restatedLo.Sign() <= 0 || rHi == nil {
		return 0, false
	}
	diff := new(big.Rat).Abs(new(big.Rat).Sub(endSq, startSq))
	c := new(big.Rat).Quo(diff, new(big.Rat).Mul(big.NewRat(2, 1), chordLo))
	if new(big.Rat).Mul(big.NewRat(2, 1), c).Cmp(restatedLo) >= 0 {
		return 0, false
	}
	// π < 3.1416: π·c/r′_lo < chord_lo/R_hi.
	if new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(31416, 10000), c), rHi).Cmp(new(big.Rat).Mul(chordLo, restatedLo)) >= 0 {
		return 0, false
	}
	// π/2 < 1.5708.
	turn := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(15708, 10000), c), restatedLo)
	shift := new(big.Rat).Mul(big.NewRat(2, 1), c)
	shift.Add(shift, new(big.Rat).Mul(new(big.Rat).Add(rHi, reachRat), turn))
	up := proofbound.RatFloatUp(shift)
	if proofbound.IsNonFinite(up) {
		return 0, false
	}
	return up, true
}

// cornerSegment is one normalized recorded segment as a corner reads it: an
// ArcSeg carries the centre the corner reads it about, its recorded Centre or,
// on a band that restates, its restatedCentre.
type cornerSegment struct {
	seg    sectionrecord.CurveSegment
	center [2]*big.Rat
}

// recordedCorner reads seg about its recorded centre.
func recordedCorner(seg sectionrecord.CurveSegment) (cornerSegment, bool) {
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !ok {
		return cornerSegment{seg: seg}, true
	}
	c, ok := ratPoint2(arc.Center)
	return cornerSegment{seg: seg, center: c}, ok
}

// readCorner reads seg about its recorded centre, or about its restated one
// when restate is set.
func readCorner(seg sectionrecord.CurveSegment, restate bool) (cornerSegment, bool) {
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !restate || !ok {
		return recordedCorner(seg)
	}
	c, ok := restatedCentre(arc)
	return cornerSegment{seg: seg, center: c}, ok
}

// OffRadiusTerms states one loop's restatement terms
// (agent-docs/tessellation-reach-design.md §7.1), for a loop whose every
// in-plane offset stays within reach:
//
//   - Shift, the largest OffRadiusShift of the loop's restated arcs;
//   - Perimeter, an upper bound on the length of every level set of the
//     restated loop region's signed distance at |τ| <= reach + Shift: a line
//     walk's own length, 2π·(R + reach + Shift) for each circular segment, and
//     π·(reach + Shift) for each recorded junction, the connector a level set
//     can take about it (a CircleSeg fragment never shares a loop with an
//     admitted restated arc, so none is read);
//   - Arcs, how many arcs the loop restates, and RadiusHi, the largest recorded
//     radius among them.
//
// Every field is zero for a loop that restates nothing.
type OffRadiusTerms struct {
	Shift, Perimeter, RadiusHi float64
	Arcs                       int
}

// OffRadiusLoop reads loop's OffRadiusTerms. ok is false for a number that
// does not lift, or a restated arc OffRadiusShift does not bound.
func OffRadiusLoop(loop sectionrecord.LoopRecord, reach float64) (OffRadiusTerms, bool) {
	var out OffRadiusTerms
	for _, raw := range loop.Segments {
		seg, err := sectionrecord.NormalizeSegment(raw)
		if err != nil {
			return OffRadiusTerms{}, false
		}
		arc, ok := seg.(sectionrecord.ArcSeg)
		if !ok || !offRadius(arc) {
			continue
		}
		shift, ok := OffRadiusShift(arc, reach)
		if !ok {
			return OffRadiusTerms{}, false
		}
		out.Arcs++
		out.Shift = math.Max(out.Shift, shift)
		for _, sq := range []*big.Rat{squaredRadius(arc.Start, arc.Center), squaredRadius(arc.End, arc.Center)} {
			out.RadiusHi = math.Max(out.RadiusHi, proofbound.RatSqrtUp(sq))
		}
	}
	if out.Arcs == 0 {
		return OffRadiusTerms{}, true
	}
	level := proofbound.AbsSumUpper(math.Abs(reach), out.Shift)
	// 3.1416 > π and 6.2832 > 2π.
	const piUpper, twoPiUpper = 3.1416, 6.2832
	perimeter := 0.0
	for _, raw := range loop.Segments {
		seg, err := sectionrecord.NormalizeSegment(raw)
		if err != nil {
			return OffRadiusTerms{}, false
		}
		var length float64
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			a, okA := ratPoint2(s.Start)
			b, okB := ratPoint2(s.End)
			if !okA || !okB {
				return OffRadiusTerms{}, false
			}
			d := [2]*big.Rat{new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(b[1], a[1])}
			length = proofbound.RatSqrtUp(ratDot(d, d))
		case sectionrecord.ArcSeg:
			radius := math.Max(proofbound.RatSqrtUp(squaredRadius(s.Start, s.Center)), proofbound.RatSqrtUp(squaredRadius(s.End, s.Center)))
			length = proofbound.ProductUpper(twoPiUpper, proofbound.AbsSumUpper(radius, level))
		default:
			// A circle fragment refuses the admission in a loop of several
			// walks (SegmentRefusal), so no loop that restates holds one.
			return OffRadiusTerms{}, false
		}
		perimeter = proofbound.AbsSumUpper(perimeter, length, proofbound.ProductUpper(piUpper, level))
	}
	out.Perimeter = perimeter
	if proofbound.IsNonFinite(out.Perimeter) || proofbound.IsNonFinite(out.RadiusHi) {
		return OffRadiusTerms{}, false
	}
	return out, true
}
