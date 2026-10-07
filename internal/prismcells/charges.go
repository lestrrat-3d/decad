package prismcells

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type (
	LoopRecord   = sectionrecord.LoopRecord
	CurveSegment = sectionrecord.CurveSegment
	LineSeg      = sectionrecord.LineSeg
	ArcSeg       = sectionrecord.ArcSeg
	CircleSeg    = sectionrecord.CircleSeg
)

// ProfileHasTrimmedCircularSource reports whether a section carries an ArcSeg or
// CircleSeg whose recorded range is narrower than its own natural domain
// (task fu143, §3.4's obligations). Such a segment enters buildPrismScene's
// private scene through entities built from TWO cos/sin-computed points, so
// its rebuilt carrier's radius and sweep both move, not merely its
// endpoints — the mechanism WalkChargeOf's plain coordinate charge does not
// state. This evaluator refuses the pair rather than under-charge it.
//
// "Whole" is decided the same way WalkChargeOf decides it, through the same
// WholeSegmentRange test: exact float equality against 0 and 1, never a
// tolerance. A CircleSeg is read from its OWN recorded range, never from its
// walk's closed-ness: circularWalk (extrude.go) calls a walk closed whenever
// its swept angle lands within a fixed tolerance of a full turn, so a range
// short of 1 by an ulp walks as closed while still being a trimmed segment
// this refusal owes an answer for — and a decad-side tolerance that can
// ACCEPT is the admission gate CLAUDE.md's reject-only rule forbids.
func ProfileHasTrimmedCircularSource(budget *proofbound.WorkBudget, outer LoopRecord, holes []LoopRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{outer}, holes...) {
		for _, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return false, err
			}
			switch s := seg.(type) {
			case ArcSeg:
				if !WholeSegmentRange(s.TStart, s.TEnd) {
					return true, nil
				}
			case CircleSeg:
				if !WholeSegmentRange(s.TStart, s.TEnd) {
					return true, nil
				}
			}
		}
	}
	return false, nil
}

// WholeSegmentRange reports whether a recorded parameter range names the two
// natural bounds of the segment's own domain, so that the segment's walk
// restates the entity's own defining data rather than a coordinate this
// evaluator computed. It is the single owner of that test for this file:
// ProfileHasTrimmedCircularSource's refusal and WalkChargeOf's zero
// charge are the same question asked twice, and they must not drift apart.
//
// The comparison is exact float equality against 0 and 1, never a tolerance
// in either direction — a range one ulp short of a bound is a trimmed
// segment, and admitting it on nearness would be a decad-side check that
// ACCEPTS, which CLAUDE.md's reject-only rule forbids. Which bound sits in
// which field does not matter: a Reversed whole edge records TStart=1,
// TEnd=0 (recordEdge's own "TStart and TEnd swapped" comment) and both
// values still name a natural bound, while validateSegmentRange (record.go)
// already rejects an empty TStart == TEnd range.
func WholeSegmentRange(tStart, tEnd float64) bool {
	return (tStart == 0 || tStart == 1) && (tEnd == 0 || tEnd == 1)
}

// WalkChargeOf is §7's δ_walk mechanism (task fu143): the charge ONE consumed
// source segment owes for entering buildPrismScene's private scene at its own
// WALKED endpoint(s) rather than at the coordinates its record states
// verbatim.
//
// A WHOLE segment's walk restates the entity's own defining data exactly —
// lerp2 (moments.go) and pinArcWalkEnds (extrude.go) both special-case the
// natural bounds t=0/t=1 to return the record's own Point2 verbatim
// regardless of which field holds which (a Reversed whole edge records
// TStart=1, TEnd=0 — recordEdge's own "TStart and TEnd swapped" comment —
// and both still name a natural bound), and a CircleSeg recorded over those
// bounds walks the recorded centre and radius directly (circularWalk never
// touches Center/Radius) — so a whole segment charges nothing. A narrowed range
// evaluates the carrier at a COMPUTED parameter instead (lerp2's else arm, or
// circularWalk's cos/sin at the walk's own angle), which proofbound.WalkEndpointAllow
// charges.
//
// proofbound.WalkEndpointAllow is charged at the SOURCE operands the walk's own
// arithmetic touches, never at the endpoint it produced, and the envelope each
// kind passes is the kind's own: a line passes lineWalkOperandUpper, because
// lerp2's b−a cancels and leaves the walked endpoint no witness at all to the
// carrier magnitude that rounding happened at; a circular walk passes
// survey2d.SegmentWalk.coordUpper, whose |cu|+|cv|+r+r L1 form (circularWalk) already
// bounds the centre and radius its cos/sin arithmetic works on, so it IS the
// source envelope for that kind rather than an answer standing in for one.
//
// Only the line arm is reachable through tryPrismBoolean: a trimmed circular
// carrier is refused before the scene is built
// (ProfileHasTrimmedCircularSource), because a coordinate envelope does
// not state the movement of its rebuilt radius and sweep, so every ArcSeg or
// CircleSeg that reaches this function through the boolean is WHOLE and
// charges zero. The narrowed circular arm stands anyway, so that a widening
// of that refusal meets a charge rather than a silent zero, and it is
// exercised directly by this file's own unit test rather than through a
// boolean.
//
// "Whole" is decided by WholeSegmentRange for every kind — exact float
// equality against 0 and 1, never a tolerance: a range short of a natural
// bound is a trimmed segment and must be charged, however near that bound it
// lies. A CircleSeg is decided from its recorded range too, not from its
// walk's tolerance-decided closed-ness, so that this charge and
// ProfileHasTrimmedCircularSource's refusal answer one question the same
// way.
func WalkChargeOf(seg CurveSegment, w survey2d.SegmentWalk) (float64, error) {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return 0, err
	}
	switch s := seg.(type) {
	case LineSeg:
		if WholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
		return proofbound.WalkEndpointAllow(lineWalkOperandUpper(s, w)), nil
	case ArcSeg:
		if WholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
	case CircleSeg:
		if WholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
	default:
		return 0, fmt.Errorf(`%w: a %T segment has no walk charge this evaluator states`, decaderr.ErrUnsupported, seg)
	}
	return proofbound.WalkEndpointAllow(w.CoordUpper), nil
}

// lineWalkOperandUpper is the envelope proofbound.WalkEndpointAllow requires for a
// trimmed LineSeg: an upper bound on every operand lerp2's general arm
// touches when it computes fl(a + fl(t·fl(b−a))) for that segment.
//
// Those operands are the carrier's own RECORDED Start and End coordinates —
// a Partial line fragment records its source sketch.Line's full Start/End
// with a narrowed range (recordEdge, seam.go), so the carrier can reach far
// past the fragment — together with the walked endpoint the outer sum
// produces. Folding the walk's own coordinate envelope in beside the recorded
// four costs nothing when the recorded range lies in [0, 1] (the lerp is then
// inside the carrier's own hull, which the recorded coordinates already
// bound) and keeps the envelope proven for any parameter at all, so this
// helper never leans on a range check it does not perform.
//
// A NaN coordinate propagates through math.Max, and an infinite one arrives as
// +Inf, so an absent envelope reaches proofbound.WalkEndpointAllow as the non-finite it
// is rather than as a small number.
func lineWalkOperandUpper(s LineSeg, w survey2d.SegmentWalk) float64 {
	upper := w.CoordUpper
	for _, c := range [...]float64{s.Start.U, s.Start.V, s.End.U, s.End.V} {
		upper = math.Max(upper, math.Abs(c))
	}
	return upper
}
