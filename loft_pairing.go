package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file adapts loftmesh's record gates and station pairs for root consumers.

// validateLoftRecords keeps the station cap after the record-only pairing
// gates, preserving the refusal order in docs/loft-design.md §4.
func validateLoftRecords(p0, p1 ProfileRecord, pl0, pl1 PlaneRecord, alignment []int, work0, work1 *freeform.FreeformWork) ([]int, [][]survey2d.SegmentWalk, [][]survey2d.SegmentWalk, error) {
	offsets, walks0, walks1, err := loftmesh.ValidateRecordWalks(
		loftmesh.RecordProfile{Outer: p0.Outer, Holes: p0.Holes},
		loftmesh.RecordProfile{Outer: p1.Outer, Holes: p1.Holes},
		pl0, pl1, alignment, work0, work1,
	)
	if err != nil {
		return nil, nil, nil, err
	}
	if err := loftStationCapGate(p0, p1, offsets, walks0, walks1); err != nil {
		return nil, nil, nil, err
	}
	return offsets, walks0, walks1, nil
}

// loftLoopPair is Table P's correspondence for one loop: the two walk-ordered
// STATION-chain lists, v from loop0's own segment order and w from loop1's,
// already rotated by that loop's own alignment offset (P4). Each paired
// segment contributes its own station count of entries — one per LineSeg, or
// the shared chord count a circular or free-form pair's own generator settles
// on — and every list still carries only each
// segment's OWN interior stations, never its shared end point, exactly as
// the one-point-per-LineSeg convention already did: the next segment's own
// first station (or the loop's wrap) supplies it.
//
// arcUpperV/arcUpperW and matchedDelta are parallel to v/w, one entry per
// station: arcUpperV[j]/arcUpperW[j] is that station's own OUTGOING cell's
// per-side arc-length upper bound (perCellArcUpper), and matchedDelta[j] is
// that cell's own PARAMETER-MATCHED bound on |curve(s) - idealChord(s)| at the
// same s: the CHORD-TO-CURVE HALF of docs/loft-design.md §5.2's matchedDelta
// row, stated for the ideal chord joining the two points the record denotes.
// The consumer composes it with the build's own delta through
// chordCellDeltaUpper to reach the bound internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper
// obligates for the chord the build actually DREW (computeLoftChordedAllow,
// loft_moments.go); this field is never that composed bound on its own, and
// never the SET-distance sagitta sectionDelta names either.
// A LineSeg cell's own chord IS the curve it denotes, so its entry is
// exactly 0; a circular cell's own sagitta discharges this half exactly
// (loftCircularCellStations' own doc comment), so its entry equals its
// sagitta; a free-form cell's entry is freeform.SpanMatchedDeltaUpper's own
// per-cell reading (freeform.PairChainStations), which can differ cell to cell
// within one paired segment where the bisection settled at different depths.
//
// computeLoftChordedAllow (loft_moments.go) reads all three to charge
// docs/loft-design.md §5/§8's chorded volume/centroid/area terms only where a
// genuine chord-to-curve departure exists (matchedDelta[j] > 0), never on an
// exact LineSeg cell. THIS GATE IS NEVER KEYED ON A SEGMENT KIND: a same-kind
// pairing this evaluator later admits that carries a positive matchedDelta
// must be charged regardless of which arm produced it, so the gate reads the
// proven quantity itself rather than an enum a future arm could be silently
// exempted from (a10-plan.md Part 3 PR 9 Task 1a).
type loftLoopPair struct {
	v, w                 []Point2
	arcUpperV, arcUpperW []float64
	matchedDelta         []float64
	// tangentEnergyV/tangentEnergyW are parallel to v/w too:
	// the per-side reading for that station's OUTGOING cell —
	// perCellTangentEnergy's for a line or circular arm,
	// freeform.SpanTangentEnergyUpper's for a free-form one —
	// internal/proofbound/bounds.go's proofbound.CellChordCurveAreaAllow tangentEnergyUpper obligation.
	// +Inf where the arm that placed the stations proves no such bound, which
	// costs that helper its sharper arm and never its soundness.
	tangentEnergyV, tangentEnergyW []float64
}

// loftPairings adapts the internal paired station chains for root consumers.
func loftPairings(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk, target float64, work0, work1 *freeform.FreeformWork) ([]loftLoopPair, float64, float64, float64, error) {
	internalPairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftmesh.PairRecords(
		loftmesh.RecordProfile{Outer: p0.Outer, Holes: p0.Holes},
		loftmesh.RecordProfile{Outer: p1.Outer, Holes: p1.Holes},
		offsets, walks0, walks1, target, work0, work1,
	)
	if err != nil {
		return nil, 0, 0, 0, err
	}
	pairs := make([]loftLoopPair, len(internalPairs))
	for i, pair := range internalPairs {
		pairs[i] = loftLoopPair{
			v: pair.V, w: pair.W,
			arcUpperV: pair.ArcUpperV, arcUpperW: pair.ArcUpperW,
			matchedDelta:   pair.MatchedDelta,
			tangentEnergyV: pair.TangentEnergyV, tangentEnergyW: pair.TangentEnergyW,
		}
	}
	return pairs, sectionDelta, sectionMatchedDelta, stationRound, nil
}
