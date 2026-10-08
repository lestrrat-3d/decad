package decad

import (
	"fmt"
	"math"
	"math/big"

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

// loftStationCapError is docs/loft-design.md Table S row S15's refusal: a
// chorded pair whose station count `m` exceeds the per-segment share
// loftStationShare allocates it. loftmesh.StationCapError owns its message and
// its errors.Is answer for freeform.ErrTooManyChords.
type loftStationCapError = loftmesh.StationCapError

func loftPairCounts(loops0 []LoopRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (uint64, uint64, bool) {
	return loftmesh.PairCounts(loops0, offsets, walks0, walks1)
}

func loftStationShare(p, c uint64) int { return loftmesh.StationShare(p, c) }

func loftStationCap(p uint64) int { return loftmesh.StationCap(p) }

// loftStationCapGate decides docs/loft-design.md Table S row S15 from the two
// RECORDS alone, at the phase §4's gate-order paragraph assigns it — among the
// shape gates, beside S14's DERIVATION arm, with no station built and no
// triangle assembled. §5.1's "Deciding S15 from the record" paragraph is what
// makes that possible: m and mMax are each a function of the two records, so
// the construction phase settles the identical m this gate reads.
//
// It is the build's one reader of the chord target: it computes it from
// recordArea and the resolved walks (loftChordTarget) and returns it, so the
// station generators chord at exactly the target S15 was decided against.
//
// A build with no chorded pair (C == 0) never consults the cap and never even
// reads the chord target: its Σstations is Σn_i exactly, the count the record
// itself states, and S8 is its only resource refusal. It returns a target of
// 0 there, which no LineSeg cell reads. That early return is why an
// all-LineSeg build pays nothing for this gate.
//
// Only a circular pair settles its count here. A same-kind free-form pair's
// count is what its dyadic walk settles, so that walk carries the same share as
// its own ceiling and refuses S15 as it runs (loftmesh.FreeformCellPoints),
// with no second walk spent here to learn the count first.
//
// The refusal NAMES the segment whose own share was exceeded, since the share
// is that segment's (loftStationCapError). A walk-up that cannot settle at all
// propagates its own refusal instead: errLoftSagittaUnderivable is S14's
// DERIVATION arm, which §5.1 places beside this row precisely because the
// walk-up that settles m is what asks for that term, and freeform.ErrTooManyChords bare
// is chordCount's own per-walk ceiling.
func loftStationCapGate(p0, p1 ProfileRecord, recordArea [2]float64, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (float64, error) {
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)

	p, c, ok := loftPairCounts(loops0, offsets, walks0, walks1)
	if !ok {
		return 0, fmt.Errorf(`%w: this loft's paired-segment count overflows the station-cap arithmetic`, ErrUnsupported)
	}
	if c == 0 {
		return 0, nil
	}

	target, err := loftChordTarget(recordArea[0], loftPerimeterUpper(p0, walks0), recordArea[1], loftPerimeterUpper(p1, walks1))
	if err != nil {
		return 0, err
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
				return 0, err
			}
			if m > mMax {
				return 0, &loftStationCapError{Loop: i, Seg: j, M: m, MMax: mMax}
			}
		}
	}
	return target, nil
}

// loftChordFraction is the coefficient docs/loft-gear-bounds-design.md §5's
// chord-target rule applies to the sections' own feature size:
//
//	chordTarget = loftChordFraction * min(|area(p0)| / perimeterUpper(p0), |area(p1)| / perimeterUpper(p1))
//
// The target therefore scales with the section's own area over its own
// perimeter, and does not grow with the section's distance from the sketch
// origin or with the radius of the gear one tooth belongs to. That design's §5
// states why the volume residual tracks this size.
//
// 2.5e-4 is the finest value whose full z = 40 gear build that design measured
// keeps its audit near 5 s; the coarser 5e-4 still reads Sound on every gear
// case the design measured but leaves Area a 1.5x margin at z = 8. The
// reference arc wedge settles at the count wedgePinStations pins
// (loft_chord_calibration_internal_test.go), which re-derives it from the
// generator at every run.
//
// It is NOT a caller option: a loft's chording is topology, and nothing is
// added to the public API for it (docs/loft-design.md §5.1).
const loftChordFraction = 2.5e-4

// loftFeatureSize is one section's own feature size: the magnitude of its
// exact region area over a proven upper bound on its perimeter. Both inputs
// come from the record alone, so the same record gives the same size on every
// platform.
func loftFeatureSize(area, perimeterUpper float64) float64 {
	return math.Abs(area) / perimeterUpper
}

// loftPerimeterUpper is a proven upper bound on a section's whole boundary
// length, every loop included: the sum over the record's segments of
// perCellArcUpper at one cell, which is the exact circularLengthInterval
// bracket for a circular segment and the walk's own LengthUpper otherwise.
// walks is validateLoftRecords' own per-loop list (outer at index 0, each hole
// at index i+1), in the record's own segment order.
func loftPerimeterUpper(p ProfileRecord, walks [][]survey2d.SegmentWalk) float64 {
	loops := append([]LoopRecord{p.Outer}, p.Holes...)
	perimeter := 0.0
	for i, loop := range loops {
		for j, seg := range loop.Segments {
			perimeter = proofbound.AbsSumUpper(perimeter, perCellArcUpper(seg, walks[i][j], 1))
		}
	}
	return perimeter
}

// loftChordTarget is one loft build's own chord target
// (docs/loft-gear-bounds-design.md §5): loftChordFraction times the smaller of
// the two sections' feature sizes. area0/area1 are the records' own exact
// region integrals falsifyRecordedArea computed in Loft, never sketch's claimed
// areas, and perim0/perim1 are loftPerimeterUpper's bounds.
//
// A target that is not a positive finite number — a perimeter bound that
// overflowed, or an area that is zero — has no chord depth that meets it, so
// it refuses ErrUnsupported (Table S row S14's derivation arm) instead of
// sending the station walk to its ceiling.
func loftChordTarget(area0, perim0, area1, perim1 float64) (float64, error) {
	target := loftChordFraction * math.Min(loftFeatureSize(area0, perim0), loftFeatureSize(area1, perim1))
	if !(target > 0) || math.IsInf(target, 0) {
		return 0, fmt.Errorf(`%w: this loft's chord target is %v: a section's area (%v, %v) over its perimeter bound (%v, %v) gives no positive finite feature size`,
			ErrUnsupported, target, area0, area1, perim0, perim1)
	}
	return target, nil
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
