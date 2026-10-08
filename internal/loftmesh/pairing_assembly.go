package loftmesh

import (
	"errors"
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
// Each paired segment's own station chain comes from RecordCellStations for a
// LineSeg or circular pair and from FreeformCellPoints for a same-kind Tier A
// free-form pair, so a loop's v/w lists carry more than one entry per segment
// exactly when that segment's own arm does — a LineSeg pairing stays exactly
// one entry per segment. sectionDelta
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
	shareMax, err := freeformShare(loops0, offsets, walks0, walks1)
	if err != nil {
		return nil, 0, 0, 0, err
	}
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		var pair LoopPair
		for j := range n {
			w0 := walks0[i][j]
			k := (j + off) % n
			w1 := walks1[i][k]
			seg0 := loops0[i].Segments[j]
			seg1 := loops1[i].Segments[k]
			if w0.Kind == survey2d.WalkFreeform && w1.Kind == survey2d.WalkFreeform {
				cell, err := FreeformCellPoints(w0, w1, target, shareMax, work0, work1)
				if err != nil {
					var capErr *StationCapError
					if errors.As(err, &capErr) {
						capErr.Loop, capErr.Seg = i, j
					}
					return nil, 0, 0, 0, err
				}
				pair.appendFreeform(cell)
				sectionDelta = math.Max(sectionDelta, cell.Sagitta)
				for _, d := range cell.MatchedDelta {
					sectionMatchedDelta = math.Max(sectionMatchedDelta, d)
				}
				stationRound = math.Max(stationRound, cell.Round)
				continue
			}
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
				pair.ArcUpperV = append(pair.ArcUpperV, cellArcV)
				pair.ArcUpperW = append(pair.ArcUpperW, cellArcW)
				pair.TangentEnergyV = append(pair.TangentEnergyV, cellEnergyV)
				pair.TangentEnergyW = append(pair.TangentEnergyW, cellEnergyW)
			}
			pair.MatchedDelta = append(pair.MatchedDelta, cellMatchedDelta...)
			pair.V = append(pair.V, stations0...)
			pair.W = append(pair.W, stations1...)
			sectionDelta = math.Max(sectionDelta, sagitta)
			for _, d := range cellMatchedDelta {
				sectionMatchedDelta = math.Max(sectionMatchedDelta, d)
			}
			stationRound = math.Max(stationRound, round)
		}
		pairs[i] = pair
	}
	// S16 runs after every loop's stations exist, so a free-form pair's S15
	// and S14 refusals, decided as its stations are generated, come before
	// any loop's collapsed-cell refusal (docs/loft-design.md §4's gate order).
	for i, pair := range pairs {
		if err := oneSidedCellGate(i, pair.V, pair.W); err != nil {
			return nil, 0, 0, 0, err
		}
	}
	return pairs, sectionDelta, sectionMatchedDelta, stationRound, nil
}

// appendFreeform appends one free-form pair's cells to the loop. A free-form
// cell's native parameter is not constant speed, so neither side proves a
// tangent energy and both entries are +Inf, which costs
// proofbound.CellChordCurveAreaAllow its sharper arm and never its soundness
// (docs/loft-design.md §5.2's tangentEnergy_k row).
func (p *LoopPair) appendFreeform(cell FreeformCell) {
	p.V = append(p.V, cell.Stations0...)
	p.W = append(p.W, cell.Stations1...)
	p.ArcUpperV = append(p.ArcUpperV, cell.ArcUpper0...)
	p.ArcUpperW = append(p.ArcUpperW, cell.ArcUpper1...)
	p.MatchedDelta = append(p.MatchedDelta, cell.MatchedDelta...)
	for range cell.MatchedDelta {
		p.TangentEnergyV = append(p.TangentEnergyV, math.Inf(1))
		p.TangentEnergyW = append(p.TangentEnergyW, math.Inf(1))
	}
}

// freeformShare is the per-segment station share docs/loft-design.md §5.1
// allocates a chorded pair, read for the free-form arm, whose dyadic walk
// stops at it: the walk itself is what settles a free-form pair's count, so
// S15 is decided as that walk runs rather than by a separate settle step.
// A build with no free-form pair answers 0 and reads nothing.
func freeformShare(loops0 []sectionrecord.LoopRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (int, error) {
	found := false
	for i := range loops0 {
		n := len(loops0[i].Segments)
		for j := range n {
			if walks0[i][j].Kind == survey2d.WalkFreeform && walks1[i][(j+offsets[i])%n].Kind == survey2d.WalkFreeform {
				found = true
			}
		}
	}
	if !found {
		return 0, nil
	}
	p, c, ok := PairCounts(loops0, offsets, walks0, walks1)
	if !ok {
		return 0, fmt.Errorf(`%w: this loft's paired-segment count overflows the station-cap arithmetic`, decaderr.ErrUnsupported)
	}
	return StationShare(p, c), nil
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
