package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// TrimBoundsWalks resolves a trimmed profile in recorded outer-then-hole
// order and charges each cut endpoint's own coordinate allowance.
func TrimBoundsWalks(outerLoop sectionrecord.LoopRecord, holeLoops []sectionrecord.LoopRecord,
	walk func(sectionrecord.CurveSegment) (survey2d.SegmentWalk, error)) (
	[]survey2d.SegmentWalk, [][]survey2d.SegmentWalk, error) {
	walkCharged := func(seg sectionrecord.CurveSegment) (survey2d.SegmentWalk, error) {
		w, err := walk(seg)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		t0, t1, err := SegmentParamRange(seg)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		chargeU, chargeV, err := TrimCutChargeUV(seg)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		if t0 != 0 && t0 != 1 {
			w.StartBound = proofbound.WalkEndBound{
				U: proofbound.AbsSumUpper(w.StartBound.U, chargeU),
				V: proofbound.AbsSumUpper(w.StartBound.V, chargeV),
			}
		}
		if t1 != 0 && t1 != 1 {
			w.EndBound = proofbound.WalkEndBound{
				U: proofbound.AbsSumUpper(w.EndBound.U, chargeU),
				V: proofbound.AbsSumUpper(w.EndBound.V, chargeV),
			}
		}
		return w, nil
	}
	outer := make([]survey2d.SegmentWalk, len(outerLoop.Segments))
	for i, seg := range outerLoop.Segments {
		w, err := walkCharged(seg)
		if err != nil {
			return nil, nil, err
		}
		outer[i] = w
	}
	holes := make([][]survey2d.SegmentWalk, len(holeLoops))
	for hi, hole := range holeLoops {
		hw := make([]survey2d.SegmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := walkCharged(seg)
			if err != nil {
				return nil, nil, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	return outer, holes, nil
}

// FullExtendSegment copies the entity's defining data and names its natural
// parameter bounds, ALWAYS ascending — t from 0 to 1 — whichever way the
// receiver's own recorded range runs. No coordinate is computed, even for an
// arc or circle.
//
// The ascending order is what makes the scene entity's parameterisation the
// receiver's own, so §3.2's nearest-cut reading and the bound it stores live in
// ONE space and no map runs between them. This is not decad re-deriving a 2D
// answer: the cut parameter stays sketch's, and the only thing settled here is
// which of decad's own two parameterisations of decad's own recorded entity the
// scene is built in. The derivation is per kind, off buildPrismScene's own
// entity creation (prism_boolean.go), which reads walkOf's walked geometry of
// exactly the segment handed to it:
//
//   - LineSeg. buildPrismScene creates CreateLine(P(TStart), P(TEnd)). At
//     (0, 1) those are the record's own Start and End, so the scene line runs
//     Start to End and sketch's parameter is the record's own lerp parameter:
//     the identity. At (1, 0) it would run End to Start and the scene parameter
//     would be 1 − t, which is why the order is fixed here rather than mapped
//     back later.
//   - CircleSeg. buildPrismScene creates CreateCircle(centre, radius) from the
//     record's own two fields, so no walk direction reaches the entity at all.
//     walkOf indexes the record's t as the angle 2πt and sketch indexes its own
//     the same way, so the map is the identity in either sense; CCW is set true
//     only because walkOf refuses a CCW flag contradicting an ascending range.
//   - ArcSeg. buildPrismScene creates CreateArc(centre, lo, hi) with lo and hi
//     the walked endpoints put in ascending-angle order. At (0, 1)
//     pinArcWalkEnds (internal/boundarywalk/walk.go) pins them to the record's own Start and
//     End verbatim and th1 > th0 leaves them unswapped, so the scene arc sweeps
//     CCW from Start to End — the very angle interval a0 → a0 + sweep the
//     record's own t indexes. The identity.
//
// No arithmetic runs on any parameter here, so no bound is charged: every field
// written is either a recorded field copied verbatim or one of the two literal
// natural bounds.
func FullExtendSegment(seg CurveSegment) (CurveSegment, error) {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return nil, err
	}
	switch s := seg.(type) {
	case LineSeg:
		s.TStart, s.TEnd = 0, 1
		return s, nil
	case CircleSeg:
		s.CCW, s.TStart, s.TEnd = true, 0, 1
		return s, nil
	case ArcSeg:
		s.TStart, s.TEnd = 0, 1
		return s, nil
	default:
		return nil, fmt.Errorf(`%w: a %T segment has no admitted full domain`, decaderr.ErrUnsupported, seg)
	}
}

// ExtendCarrierDomain names the carrier a refusal is about — its kind, its own
// defining coordinates and its natural parameter range — so RS4's message
// states the domain the tool missed rather than only that it missed one
// (docs/surface-design.md §15's T179).
func ExtendCarrierDomain(seg CurveSegment) string {
	switch s := seg.(type) {
	case LineSeg:
		return fmt.Sprintf("the line from (%v, %v) to (%v, %v) over t in [0, 1]",
			s.Start.U, s.Start.V, s.End.U, s.End.V)
	case CircleSeg:
		return fmt.Sprintf("the circle of radius %v about (%v, %v) over t in [0, 1]",
			s.Radius, s.Center.U, s.Center.V)
	case ArcSeg:
		return fmt.Sprintf("the arc about (%v, %v) from (%v, %v) to (%v, %v) over t in [0, 1]",
			s.Center.U, s.Center.V, s.Start.U, s.Start.V, s.End.U, s.End.V)
	default:
		return fmt.Sprintf("a %T carrier over t in [0, 1]", seg)
	}
}

func ExtendSetBound(seg CurveSegment, atStart bool, bound float64) CurveSegment {
	switch s := seg.(type) {
	case LineSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	case CircleSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	case ArcSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	}
	return seg
}

// TrimProfileFullyWhole reports whether every segment of the supplied loops
// spans its entity's own natural domain (S7's third clause) — read off the
// recorded range through wholeSegmentRange, the identical prism-boolean
// reading (prism_boolean.go), and never off any walk's closed-ness. Stating
// it this way is a record property, decidable before the arrangement runs,
// which is what lets every miss here be one refusal at the call
// (docs/surface-intersection-design.md §2.1 S7).
func TrimProfileFullyWhole(budget *proofbound.WorkBudget, outer LoopRecord, holes []LoopRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{outer}, holes...) {
		for _, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return false, err
			}
			whole, err := trimSegmentIsWhole(seg)
			if err != nil {
				return false, err
			}
			if !whole {
				return false, nil
			}
		}
	}
	return true, nil
}

// trimSegmentIsWhole reads one S3-admitted segment's own recorded range
// through wholeSegmentRange (prism_boolean.go) — exact float equality
// against 0 and 1, never a tolerance in either direction.
func trimSegmentIsWhole(seg CurveSegment) (bool, error) {
	t0, t1, err := SegmentParamRange(seg)
	if err != nil {
		return false, err
	}
	return WholeSegmentRange(t0, t1), nil
}

// SegmentParamRange reads an S3-admitted segment's own recorded TStart/
// TEnd — the one place decad names, per field, whether that end is the
// entity's own natural bound or a coordinate the arrangement computed. Every
// admitted kind (LineSeg, CircleSeg, ArcSeg) carries the two fields directly.
func SegmentParamRange(seg CurveSegment) (t0, t1 float64, err error) {
	seg, err = sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return 0, 0, err
	}
	switch s := seg.(type) {
	case LineSeg:
		return s.TStart, s.TEnd, nil
	case CircleSeg:
		return s.TStart, s.TEnd, nil
	case ArcSeg:
		return s.TStart, s.TEnd, nil
	default:
		return 0, 0, fmt.Errorf(`%w: a %T segment is not part of the admitted class`, decaderr.ErrUnsupported, seg)
	}
}

// TrimCutChargeUV is trimBoundsWalks's own per-component reading of §7's
// δ_cut: how far a cut endpoint's u and v coordinates can EACH sit from the
// crossing they denote, given proofbound.CutDisplacementAllow's own parameter-to-
// coordinate scaling (internal/proofbound/bounds.go). A CircleSeg/ArcSeg's position varies with
// BOTH components under a cos/sin walk, so both take prismcells.CarrierSpeedUpper's
// existing isotropic reading (internal/prismcells/merge.go) unchanged. A LineSeg's does
// not: its walk is Start + t·(End−Start), so a coordinate whose OWN
// End−Start difference is exactly zero — a horizontal line's v, a vertical
// line's u — carries no displacement AT ALL as t moves, however uncertain t
// itself is, and charging it anyway would smear a widened cut candidate's
// slop onto an axis the cut never touches (T170's own top and bottom walls,
// whose v never moves at any parameter). Each component's own exact
// End−Start difference, taken over the recorded floats and rounded outward
// (ratL1Upper), is what prismcells.CarrierSpeedUpper already reduces to a single L1
// figure for the isotropic case; reading it per component instead is the
// same mechanism, not a new one.
func TrimCutChargeUV(seg CurveSegment) (chargeU, chargeV float64, err error) {
	seg, err = sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return 0, 0, err
	}
	if line, ok := seg.(LineSeg); ok {
		du := boundarywalk.RatL1Upper(circularbounds.ExactCoordinateDelta(line.End.U, line.Start.U))
		dv := boundarywalk.RatL1Upper(circularbounds.ExactCoordinateDelta(line.End.V, line.Start.V))
		return proofbound.CutDisplacementAllow(du), proofbound.CutDisplacementAllow(dv), nil
	}
	speed, err := CarrierSpeedUpper(seg)
	if err != nil {
		return 0, 0, err
	}
	charge := proofbound.CutDisplacementAllow(speed)
	return charge, charge, nil
}

// TrimRevolveSegmentCharges is trimBoundsWalks's own per-endpoint rule, read
// for the revolve family (docs/surface-intersection-design.md §7.1): the
// per-component charge trimCutChargeUV states lands at exactly the endpoint
// whose OWN recorded parameter is not a natural bound (0 or 1, per field —
// never per segment, since a segment cut on only one side keeps its natural
// end exact), and at NEITHER endpoint of a segment the arrangement did not
// cut. The decision reads only the two recorded floats.
//
// delta is the payload's own sectionDelta, read as a gate and never as the
// charge: a payload no construction displaced has every recorded parameter
// natural anyway, so the two answers would be absent regardless, and gating
// keeps an ordinary RevolveChain off this path entirely.
func TrimRevolveSegmentCharges(seg CurveSegment, delta float64) (proofbound.WalkEndBound, proofbound.WalkEndBound, error) {
	if delta == 0 {
		return proofbound.WalkEndBound{}, proofbound.WalkEndBound{}, nil
	}
	t0, t1, err := SegmentParamRange(seg)
	if err != nil {
		return proofbound.WalkEndBound{}, proofbound.WalkEndBound{}, err
	}
	chargeU, chargeV, err := TrimCutChargeUV(seg)
	if err != nil {
		return proofbound.WalkEndBound{}, proofbound.WalkEndBound{}, err
	}
	var start, end proofbound.WalkEndBound
	if t0 != 0 && t0 != 1 {
		start = proofbound.WalkEndBound{U: chargeU, V: chargeV}
	}
	if t1 != 0 && t1 != 1 {
		end = proofbound.WalkEndBound{U: chargeU, V: chargeV}
	}
	return start, end, nil
}
