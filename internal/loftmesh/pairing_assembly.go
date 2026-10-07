package loftmesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// LoopPair holds the paired station chains and per-cell bounds for one loop.
type LoopPair struct {
	V, W                           []sectionrecord.Point2
	ArcUpperV, ArcUpperW           []float64
	MatchedDelta                   []float64
	TangentEnergyV, TangentEnergyW []float64
}

// PairRecords resolves Table P into one flat correspondence per loop, from
// validateLoftRecords' own already-resolved walks — it spends no further
// walkOf call, and so no further free-form work (A10 plan Task 1). P1 pairs
// by position in Holes, never by area or proximity; P6 is satisfied by
// construction because each list is read in its own loop's own recorded walk
// order and nothing reinterprets it. The alignment offset rotates walks1's
// own natural order into correspondence here, at the point of use, exactly
// as validateLoftRecords' own S3 check already does.
//
// Each paired segment's own station chain now comes from RecordCellStations
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
// the RECORDED segment behind each side's walk (RecordCellStations' own doc
// comment), so each side's segment is handed to the generator alongside its
// walk, under the same alignment offset the walk itself is read at.
func PairRecords(p0, p1 RecordProfile, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk, target float64, work0, work1 *freeform.FreeformWork) ([]LoopPair, float64, float64, float64, error) {
	loops0 := append([]sectionrecord.LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]sectionrecord.LoopRecord{p1.Outer}, p1.Holes...)
	pairs := make([]LoopPair, len(loops0))
	sectionDelta := 0.0
	sectionMatchedDelta := 0.0
	stationRound := 0.0
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		var v, w []sectionrecord.Point2
		var arcUpperV, arcUpperW []float64
		var tangentEnergyV, tangentEnergyW []float64
		var matchedDelta []float64
		for j := range n {
			w0 := walks0[i][j]
			k := (j + off) % n
			w1 := walks1[i][k]
			seg0 := loops0[i].Segments[j]
			seg1 := loops1[i].Segments[k]
			stations0, stations1, sagitta, cellMatchedDelta, round, err := RecordCellStations(w0, w1, seg0, seg1, target, work0, work1)
			if err != nil {
				return nil, 0, 0, 0, err
			}
			m := len(stations0)
			cellArcV := PerCellArcUpper(seg0, w0, m)
			cellArcW := PerCellArcUpper(seg1, w1, m)
			cellEnergyV := PerCellTangentEnergy(seg0, w0, m)
			cellEnergyW := PerCellTangentEnergy(seg1, w1, m)
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
		if err := oneSidedCellGate(i, v, w); err != nil {
			return nil, 0, 0, 0, err
		}
		pairs[i] = LoopPair{
			V: v, W: w,
			ArcUpperV: arcUpperV, ArcUpperW: arcUpperW,
			MatchedDelta:   matchedDelta,
			TangentEnergyV: tangentEnergyV, TangentEnergyW: tangentEnergyW,
		}
	}
	return pairs, sectionDelta, sectionMatchedDelta, stationRound, nil
}

// oneSidedCellGate is docs/loft-design.md Table S row S16, decided over one
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
// stations, never its shared end point (LoopPair): cell j pairs station j
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
// (loft_audit.go), whose collapse refusal is ErrDegenerate — a claim that no
// body exists under ANY evaluator — where this row owes ErrUnsupported, the
// weaker claim that a point-degenerate correspondence is a body a smarter
// kernel could still loft.
func oneSidedCellGate(loop int, v, w []sectionrecord.Point2) error {
	n := len(v)
	if n != len(w) {
		// Unreachable from any build: both chains take the SAME per-segment
		// station count from RecordCellStations, appended in the same order. A
		// refusal rather than a silent skip, since skipping would drop the
		// gate for the whole loop.
		return fmt.Errorf(`%w: loop %d's two station chains carry %d and %d stations; a chord cell has no pairing across unequal chains`,
			decaderr.ErrUnsupported, loop, n, len(w))
	}
	for j := range n {
		k := (j + 1) % n
		if (v[j] == v[k]) != (w[j] == w[k]) {
			return fmt.Errorf(`%w: loop %d's chord cell %d collapses to one point on only one of the two sections`, decaderr.ErrUnsupported, loop, j)
		}
	}
	return nil
}
