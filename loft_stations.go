package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file sets a loft's chord target and station cap. Internal loftmesh
// computes the station chains and the bounds they carry.
//
// One chord target is chosen for the whole loft, and every cell derives its
// own station count from that target, so the two paired curves are sampled at
// matched parameters rather than at independently chosen ones. The certified
// sagitta and chord bounds are what turn that departure into the section
// displacement the payload publishes; a cell whose departure cannot be
// bounded refuses through errLoftSagittaUnderivable. The station cap bounds
// the work before any of it is done. See docs/loft-design.md §5.2.

const loftStationCap = loftmesh.LoftStationCap

// loftStationCapError is docs/loft-design.md Table S row S15's refusal: a
// same-kind circular pair whose settled station count `m` (§5.1's joint
// walk-up) exceeds the per-segment share loftStationShare allocates it.
//
// It is a type rather than an fmt.Errorf wrapper because the refusal must NAME
// the segment whose own share it exceeded (§5.1) while still answering
// errors.Is for freeform.ErrTooManyChords, the sentinel §5.1 assigns this row (spline
// design Table R row R8). Wrapping freeform.ErrTooManyChords with %w would prepend that
// sentinel's own text — "the chord tolerance asks for more than 16384 chords
// on one curve" — which names no segment and describes a caller-supplied
// tessellation tolerance a loft has no such knob for (§5.1: "The target is not
// a caller option"). Unwrap keeps errors.Is answering for both freeform.ErrTooManyChords
// and, through it, ErrUnsupported.
type loftStationCapError struct {
	loop, seg int
	m, mMax   int
}

func (e *loftStationCapError) Error() string {
	return fmt.Sprintf(
		`%s: loop %d segment %d needs %d chord cells to meet the loft chord target, past the %d its share of the %d-station cap allows`,
		ErrUnsupported.Error(), e.loop, e.seg, e.m, e.mMax, loftStationCap,
	)
}

func (e *loftStationCapError) Unwrap() error { return freeform.ErrTooManyChords }

func loftPairCounts(loops0 []LoopRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (uint64, uint64, bool) {
	return loftmesh.PairCounts(loops0, offsets, walks0, walks1)
}

func loftStationShare(p, c uint64) int { return loftmesh.StationShare(p, c) }

// loftStationCapGate decides docs/loft-design.md Table S row S15 from the two
// RECORDS alone, at the phase §4's gate-order paragraph assigns it — among the
// shape gates, beside S14's DERIVATION arm, with no station built and no
// triangle assembled. §5.1's "Deciding S15 from the record" paragraph is what
// makes that possible: m and mMax are each a function of the two records, so
// the construction phase settles the identical m this gate reads.
//
// A build with no same-kind circular pair (C == 0) never consults the cap and
// never even reads the chord target: its Σstations is Σn_i exactly, the count
// the record itself states, and S8 is its only resource refusal. That early
// return is why an all-LineSeg build — every build this evaluator admits
// today, S3 refusing every other kind — pays nothing for this gate.
//
// The refusal NAMES the segment whose own share was exceeded, since the share
// is that segment's (loftStationCapError). A walk-up that cannot settle at all
// propagates its own refusal instead: errLoftSagittaUnderivable is S14's
// DERIVATION arm, which §5.1 places beside this row precisely because the
// walk-up that settles m is what asks for that term, and freeform.ErrTooManyChords bare
// is chordCount's own per-walk ceiling.
func loftStationCapGate(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) error {
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)

	p, c, ok := loftPairCounts(loops0, offsets, walks0, walks1)
	if !ok {
		return fmt.Errorf(`%w: this loft's paired-segment count overflows the station-cap arithmetic`, ErrUnsupported)
	}
	if c == 0 {
		return nil
	}

	target, err := loftChordTarget(p0, p1, walks0, walks1)
	if err != nil {
		return err
	}
	mMax := loftStationShare(p, c)

	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		for j := range n {
			k := (j + off) % n
			w0, w1 := walks0[i][j], walks1[i][k]
			if w0.Kind != survey2d.WalkCircular || w1.Kind != survey2d.WalkCircular {
				continue
			}
			m, _, _, err := loftSettleStationCount(w0, w1, loops0[i].Segments[j], loops1[i].Segments[k], target)
			if err != nil {
				return err
			}
			if m > mMax {
				return &loftStationCapError{loop: i, seg: j, m: m, mMax: mMax}
			}
		}
	}
	return nil
}

// loftChordFraction is the coefficient a10-plan.md Part 2 Q2's chord-target
// rule applies to a whole section's own coordinate envelope:
//
//	chordTarget = loftChordFraction * max(profileCoordinateUpper(p0), profileCoordinateUpper(p1))
//
// It is calibrated by measurement, never assumed (merged PR #188,
// loft_chord_calibration_internal_test.go): against two hand-chorded
// reference wedges — a 90-degree radius-5 quarter-arc and a 5-point
// fit-spline approximation of the same arc, both lofted between z=0 and
// z=10 — it is the coarsest value at which both fixtures still read Sound at
// the default 1e-3 relative tolerance inside the per-fixture wall-clock budget
// (a10-plan.md Q3), which docs/loft-design.md §13's build cost model paragraph
// owns and this comment states no cost of its own for.
//
// Driven through the SHIPPED generator — loftCircularCellStations below, whose
// joint walk-up settles the count against the CERTIFIED per-cell sagitta — this
// constant settles the arc wedge at m=65 stations, whose assembled face count
// is the F §7 owns, with Volume the binding reading at a measured 2.47x margin
// (gate ratio 4.04928e-4), and the fit-spline wedge chorded at that same count
// at 1.95x (5.1326e-4). Both builds land inside the budget §13 owns. The
// calibration pins those two margins at that production count and re-derives
// the count from the generator at every run (loftChordFractionPinM), so no
// published margin here belongs to a chording this evaluator does not produce.
//
// A finer grid point (m=128) clears the plan's separate 4x-margin target but
// falls outside that budget, so the plan's
// own named fallback governs (a10-plan.md Q2's "Fallback if calibration does
// not close"): ship the coarser, in-budget value and accept that an extreme
// aspect ratio can read Suspect at a tight tolerance — a correct non-silent
// outcome, not a wrong answer.
//
// It is NOT a caller option: a loft's chording is topology, and nothing is
// added to the public API for it (a10-plan.md Q2).
const loftChordFraction = 3.76491e-05

// loftChordTarget is one loft build's own chord target (a10-plan.md Q2): the
// coordinate envelope is a WHOLE-PROFILE quantity, so it is read once here,
// never re-derived per paired segment.
//
// It reads profileCoordinateUpper, never its non-refusing twin
// profileCoordinateEnvelope, deliberately: every segment kind this evaluator
// admits into a pairing today (LineSeg; ArcSeg/CircleSeg once the arc
// correspondence lands) is analytic, so the placed-cap-frame requirement
// profileCoordinateUpper carries costs nothing here. A future free-form
// pairing has no such frame to ask for, so its own caller switches this
// reading to profileCoordinateEnvelope instead — extrude.go's own doc
// comment names it as exactly that twin.
//
// walks0/walks1 are validateLoftRecords' own already-resolved walks
// (outer at index 0, each hole at index i+1): wrapping them in a
// *profileWalks view here, rather than passing nil and letting
// profileCoordinateUpper resolve again, is what keeps this reading inside
// Task 1's resolve-once rule. The two views are deliberately UNMETERED —
// validateLoftRecords charged this work against its own counters, and a view
// that restated the charge as its own would let a later replay levy it twice.
// Neither leaves this function, so neither can reach a payload that replays it.
func loftChordTarget(p0, p1 ProfileRecord, walks0, walks1 [][]survey2d.SegmentWalk) (float64, error) {
	pw0 := &profileWalks{profile: p0, outer: walks0[0], holes: walks0[1:]}
	pw1 := &profileWalks{profile: p1, outer: walks1[0], holes: walks1[1:]}
	u0, err := profileCoordinateUpper(p0, nil, pw0)
	if err != nil {
		return 0, err
	}
	u1, err := profileCoordinateUpper(p1, nil, pw1)
	if err != nil {
		return 0, err
	}
	return loftChordFraction * math.Max(u0, u1), nil
}

func loftLineCellStations(w0, w1 survey2d.SegmentWalk) ([]Point2, []Point2, float64, []float64, float64, error) {
	return loftmesh.LineCellPoints(w0, w1)
}

func loftSettleStationCount(w0, w1 survey2d.SegmentWalk, seg0, seg1 CurveSegment, target float64) (int, float64, float64, error) {
	return loftmesh.SettleRecordStationCount(w0, w1, seg0, seg1, target)
}

func loftCircularCellStations(w0, w1 survey2d.SegmentWalk, seg0, seg1 CurveSegment, target float64) ([]Point2, []Point2, float64, []float64, float64, error) {
	return loftmesh.CircularCellPoints(w0, w1, seg0, seg1, target)
}

func perCellArcUpper(seg CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	return loftmesh.PerCellArcUpper(seg, w, m)
}

// chordCellDeltaUpper keeps the root loft callers on the shared bound.
func chordCellDeltaUpper(sagittaUpper, deltaUpper float64) float64 {
	return loftmesh.ChordCellDeltaUpper(sagittaUpper, deltaUpper)
}

// errLoftStationDisplacementUnderivable is the sentinel docs/loft-design.md
// Table S row S14 carries for the station-displacement term, raised in the arm
// §4's gate-order paragraph assigns that term: a pair whose generated stations
// have no proven displacement from the recorded points they stand for. BOTH
// station arms raise it, since both can generate a station — the circular arm
// for its walked chord chain, the LineSeg arm for a station sitting at a
// TRIMMED parameter — and neither may publish a finite bound in place of a
// term it could not derive. Like its certified-sagitta twin the shape itself is
// fine and the chord set is buildable; only one of the terms the published
// bound is composed from cannot be stated, so the sentinel is ErrUnsupported
// and no finite value — least of all the sagitta alone — is published in its
// place.
var errLoftStationDisplacementUnderivable = loftmesh.ErrLoftStationDisplacementUnderivable

func perCellTangentEnergy(seg CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	return loftmesh.PerCellTangentEnergy(seg, w, m)
}

func circularStationChain(w survey2d.SegmentWalk, seg CurveSegment, m int) ([]Point2, float64) {
	return loftmesh.CircularStationChain(w, seg, m)
}

// walkEndPlaneDelta delegates the station proof to internal/tessellation.
func walkEndPlaneDelta(bound proofbound.WalkEndBound) float64 {
	return loftmesh.WalkEndPlaneDelta(bound)
}

func arcNaturalEndRadialUpper(seg CurveSegment) float64 {
	return loftmesh.ArcNaturalEndRadialUpper(seg)
}

func circularSegmentRange(seg CurveSegment) (*big.Rat, *big.Rat, bool) {
	return loftmesh.CircularSegmentRange(seg)
}

func loftCertifiedSagittaUpper(seg CurveSegment, m int) float64 {
	return loftmesh.CertifiedSagittaUpper(seg, m)
}

func loftCertifiedChordLower(seg CurveSegment, m int) float64 {
	return loftmesh.CertifiedChordLower(seg, m)
}

// errLoftSagittaUnderivable is docs/loft-design.md Table S row S14's refusal:
// a chorded circular pair for which the certified per-cell sagitta has no
// derivation from the record. The body exists and the chord set is buildable;
// only one of its proven displacement terms cannot be stated, which is a
// derivation gap in this evaluator's certified circular enclosures rather than
// a shape rule — so the sentinel is ErrUnsupported, and no finite value is
// published in its place.
var errLoftSagittaUnderivable = loftmesh.ErrLoftSagittaUnderivable
