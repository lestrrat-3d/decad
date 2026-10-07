package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file assembles loft station pairs after loftmesh checks the recorded
// correspondence under docs/loft-design.md Table P.

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
// segment contributes its own station count of entries — one per LineSeg or
// the shared chord count a circular pair's own generator settles on
// (loftCircularCellStations) — and every list still carries only each
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
// per-cell reading (internal/freeform/spline_sagitta.go's freeform.PairStations), which can differ cell
// to cell within one paired segment where the bisection settled at different
// depths.
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
	// perCellTangentEnergy's own per-side reading for that station's OUTGOING
	// cell, internal/proofbound/bounds.go's proofbound.CellChordCurveAreaAllow tangentEnergyUpper obligation.
	// +Inf where the arm that placed the stations proves no such bound, which
	// costs that helper its sharper arm and never its soundness.
	tangentEnergyV, tangentEnergyW []float64
}

// loftPairings resolves Table P into one flat correspondence per loop, from
// validateLoftRecords' own already-resolved walks — it spends no further
// walkOf call, and so no further free-form work (A10 plan Task 1). P1 pairs
// by position in Holes, never by area or proximity; P6 is satisfied by
// construction because each list is read in its own loop's own recorded walk
// order and nothing reinterprets it. The alignment offset rotates walks1's
// own natural order into correspondence here, at the point of use, exactly
// as validateLoftRecords' own S3 check already does.
//
// Each paired segment's own station chain now comes from loftCellStations
// (a10-plan.md Part 3 PR 5), so a loop's v/w lists carry more than one entry
// per segment exactly when that segment's own arm does — a LineSeg pairing
// stays exactly one entry per segment, bit-identical to before. sectionDelta
// is the MAX of every cell's own SAGITTA across the whole build, never a sum:
// a boundary point lies in exactly one cell, so only the widest cell's own
// departure bounds the whole section. sectionMatchedDelta is the analogous
// MAX of every cell's own PARAMETER-MATCHED chord-to-curve departure (F1's
// rule) — a DIFFERENT quantity, never interchangeable with sectionDelta. It
// is the CHORD-TO-CURVE HALF of docs/loft-design.md §5.2's matchedDelta row
// and never that whole row: evalLoft composes it with the build's own delta
// (chordCellDeltaUpper) before any caller of internal/proofbound/bounds.go's
// proofbound.ChordedBoundaryVolumeAllow/proofbound.ChordedBoundaryMomentAllow/
// proofbound.ChordedBoundarySeamAllow (each of whose own doc comments name a
// parameter-matched matchedDelta obligation, never "the sagitta alone")
// reads it. The two accumulators here coincide bit-for-bit
// on a circular-only build (every circular cell's own departure equals
// its own sagitta exactly, loftCircularCellStations' own doc comment) and on
// a LineSeg-only build (both exactly 0). stationRound is the analogous
// MAX of every cell's own station displacement (Table S row S14, delta's own
// component, never sectionDelta's or sectionMatchedDelta's) — the terms are
// accumulated apart and never added into one another here, which is the rule
// §5.2's table states for them.
//
// Both records are read, never p0 alone: a curved arm's own bound is stated by
// the RECORDED segment behind each side's walk (loftCellStations' own doc
// comment), so each side's segment is handed to the generator alongside its
// walk, under the same alignment offset the walk itself is read at.
func loftPairings(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk, target float64, work0, work1 *freeform.FreeformWork) ([]loftLoopPair, float64, float64, float64, error) {
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)
	pairs := make([]loftLoopPair, len(loops0))
	sectionDelta := 0.0
	sectionMatchedDelta := 0.0
	stationRound := 0.0
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		var v, w []Point2
		var arcUpperV, arcUpperW []float64
		var tangentEnergyV, tangentEnergyW []float64
		var matchedDelta []float64
		for j := range n {
			w0 := walks0[i][j]
			k := (j + off) % n
			w1 := walks1[i][k]
			seg0 := loops0[i].Segments[j]
			seg1 := loops1[i].Segments[k]
			stations0, stations1, sagitta, cellMatchedDelta, round, err := loftCellStations(w0, w1, seg0, seg1, target, work0, work1)
			if err != nil {
				return nil, 0, 0, 0, err
			}
			m := len(stations0)
			cellArcV := perCellArcUpper(seg0, w0, m)
			cellArcW := perCellArcUpper(seg1, w1, m)
			cellEnergyV := perCellTangentEnergy(seg0, w0, m)
			cellEnergyW := perCellTangentEnergy(seg1, w1, m)
			for range m {
				arcUpperV = append(arcUpperV, cellArcV)
				arcUpperW = append(arcUpperW, cellArcW)
				tangentEnergyV = append(tangentEnergyV, cellEnergyV)
				tangentEnergyW = append(tangentEnergyW, cellEnergyW)
			}
			matchedDelta = append(matchedDelta, cellMatchedDelta...)
			v = append(v, stations0...)
			w = append(w, stations1...)
			sectionDelta = math.Max(sectionDelta, sagitta)
			for _, d := range cellMatchedDelta {
				sectionMatchedDelta = math.Max(sectionMatchedDelta, d)
			}
			stationRound = math.Max(stationRound, round)
		}
		if err := loftOneSidedCellGate(i, v, w); err != nil {
			return nil, 0, 0, 0, err
		}
		pairs[i] = loftLoopPair{
			v: v, w: w,
			arcUpperV: arcUpperV, arcUpperW: arcUpperW,
			matchedDelta:   matchedDelta,
			tangentEnergyV: tangentEnergyV, tangentEnergyW: tangentEnergyW,
		}
	}
	return pairs, sectionDelta, sectionMatchedDelta, stationRound, nil
}

// loftOneSidedCellGate is docs/loft-design.md Table S row S16, decided over one
// loop's own ASSEMBLED station chains at the phase §4's gate-order paragraph
// assigns it — as stations are paired into chord cells, which is here and not
// inside the per-segment generator.
//
// A cell whose two stations coincide on exactly ONE of the two sections has no
// case in the uniform two-faces-per-cell wall topology assembleLoft builds, so
// the row refuses with ErrUnsupported. A cell collapsing on BOTH sections, and
// a collapsed cap triangle, are S6's two arms rather than this row, which is
// why the test below compares the two sides' equality rather than refusing on
// either alone — that is what makes every collapse covered exactly once.
//
// The walk is CYCLIC, over the whole loop, because §7's j indexes that loop's
// flattened chord-cell sequence and each chain carries only each segment's OWN
// stations, never its shared end point (loftLoopPair): cell j pairs station j
// to station (j+1) mod len, so the loop's last cell pairs its last station back
// to its first. Three whole classes of cell exist only in that reading and
// cannot be seen one segment at a time:
//
//   - a segment's TERMINAL cell, which reaches into the NEXT segment's first
//     station (or wraps to the loop's own first) — at every m, including the m
//     the generator itself settled;
//   - every cell of a pair settled at m = 1, which has no interior station at
//     all (§5.1) and so no consecutive pair inside its own segment;
//   - every LineSeg-pair cell, since that arm generates one station a side and
//     its cell is by definition a terminal one.
//
// Missing any of them lets a one-sided collapse fall through to S6
// (internal/loftmesh/loft_audit.go), whose collapse refusal is ErrDegenerate — a claim that no
// body exists under ANY evaluator — where this row owes ErrUnsupported, the
// weaker claim that a point-degenerate correspondence is a body a smarter
// kernel could still loft.
func loftOneSidedCellGate(loop int, v, w []Point2) error {
	n := len(v)
	if n != len(w) {
		// Unreachable from any build: both chains take the SAME per-segment
		// station count from loftCellStations, appended in the same order. A
		// refusal rather than a silent skip, since skipping would drop the
		// gate for the whole loop.
		return fmt.Errorf(`%w: loop %d's two station chains carry %d and %d stations; a chord cell has no pairing across unequal chains`,
			ErrUnsupported, loop, n, len(w))
	}
	for j := range n {
		k := (j + 1) % n
		if (v[j] == v[k]) != (w[j] == w[k]) {
			return fmt.Errorf(`%w: loop %d's chord cell %d collapses to one point on only one of the two sections`, ErrUnsupported, loop, j)
		}
	}
	return nil
}
