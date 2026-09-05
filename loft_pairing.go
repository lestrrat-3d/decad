package decad

import (
	"fmt"
	"math"
)

// This file decides WHICH segment of the from-profile a loft walls to which
// segment of the to-profile, and refuses every pairing it cannot decide.
//
// docs/loft-design.md's Table P is the pairing rule and this file is its
// implementation: loops pair by offset, segments pair in walk order, and a
// pair whose two kinds differ, whose circular senses oppose, or whose planes
// coincide is refused outright. Nothing here estimates a correspondence; a
// profile pair the table does not decide returns an error rather than a
// nearest match. See docs/loft-design.md §5 and §5.1.

// validateLoftRecords applies docs/loft-design.md Table S rows S1, S2, S4, S3,
// S7's STRUCTURAL arm, S5 and S15, in §4's stated gate order, from the two
// authenticated records alone — no triangle is built. It returns the
// normalized per-loop alignment
// offsets (a nil alignment becomes every offset 0, §2) alongside every
// segment's own resolved walk, one slice per loop, in that loop's own
// recorded segment order — NOT rotated by the alignment offset, which stays
// applied at loftPairings' own point of use, exactly as it is here for S3's
// own check.
//
// Each segment is walked exactly ONCE, at the SAME point in the SAME
// interleaved per-segment order this gate has always used — walk p0's
// segment j, walk p1's segment k=(j+off)%n, test S3 over the pair, then move
// to j+1 — never batched a whole loop ahead of that order. walkOf is neither
// memoized nor free to call twice (it charges the free-form work budget on
// every call, extrude.go's own doc comment), so resolving here and never
// again (loftPairings reads this function's own output) is what Task 1
// exists for; keeping the interleaving is what keeps S3's own refusal
// PRECEDENCE unchanged — a record whose p0 fails S3 at an early segment
// must still report that refusal even when p1 carries a later segment
// walkOf itself cannot resolve at all (a malformed CircleSeg, say), a
// combination sketch's own authentication never produces but a decoded
// recipe can (docs/recipe-replay-design.md).
//
// S3's admission test is a SAME-KIND test over the two RECORDED SEGMENT
// TYPES, exactly the three-way enumeration docs/loft-design.md §1 and Table P
// row P5 spell — both LineSeg, both ArcSeg, or both CircleSeg — never merely
// because one side is a LineSeg. Mixed-kind and free-form pairs keep today's
// refusal and today's sentinel (loftSameKindGate). Testing only after BOTH
// sides are resolved (rather than each side against its own kind test, as the
// LineSeg-only form did) is unavoidable once the admitted set has three
// types, and it does not relax PRECEDENCE: the first (i, j) whose pair fails
// is still the first refusal reported, in the same walk order as before.
func validateLoftRecords(p0, p1 ProfileRecord, pl0, pl1 PlaneRecord, alignment []int, work0, work1 *freeformWork) ([]int, [][]segmentWalk, [][]segmentWalk, error) {
	if len(p0.Holes) != len(p1.Holes) {
		return nil, nil, nil, fmt.Errorf(`%w: the two profiles have %d and %d holes; a loft has no positional pairing for a hole-count mismatch`,
			ErrUnsupported, len(p0.Holes), len(p1.Holes))
	}
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)
	loopCount := len(loops0)

	for i := range loops0 {
		if len(loops0[i].Segments) != len(loops1[i].Segments) {
			return nil, nil, nil, fmt.Errorf(`%w: loop %d has %d segments on the first profile and %d on the second; a loft has no one-to-one pairing for a segment-count mismatch`,
				ErrUnsupported, i, len(loops0[i].Segments), len(loops1[i].Segments))
		}
	}

	offsets := make([]int, loopCount)
	if alignment != nil {
		if len(alignment) != loopCount {
			return nil, nil, nil, fmt.Errorf(`%w: WithLoftAlignment carries %d offsets for %d loops`,
				ErrDegenerate, len(alignment), loopCount)
		}
		for i, off := range alignment {
			n := len(loops0[i].Segments)
			if off < 0 || off >= n {
				return nil, nil, nil, fmt.Errorf(`%w: loop %d's alignment offset %d is outside [0, %d)`,
					ErrDegenerate, i, off, n)
			}
			offsets[i] = off
		}
	}

	walks0 := make([][]segmentWalk, loopCount)
	walks1 := make([][]segmentWalk, loopCount)
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		walks0[i] = make([]segmentWalk, n)
		walks1[i] = make([]segmentWalk, n)
		for j := range n {
			w0, err := walkOf(loops0[i].Segments[j], work0)
			if err != nil {
				return nil, nil, nil, err
			}

			k := (j + off) % n
			w1, err := walkOf(loops1[i].Segments[k], work1)
			if err != nil {
				return nil, nil, nil, err
			}

			if err := loftSameKindGate(loops0[i].Segments[j], loops1[i].Segments[k], i, j, k); err != nil {
				return nil, nil, nil, err
			}
			walks0[i][j] = w0
			walks1[i][k] = w1
		}
	}

	if loftPlanesCoincide(pl0, pl1) {
		return nil, nil, nil, fmt.Errorf(`%w: the two profiles lie in the same geometric plane; the loft has zero volume by construction`, ErrDegenerate)
	}

	// S15, last among the shape gates, exactly where §4's gate-order paragraph
	// puts it: after S5 and beside S14's DERIVATION arm, decided from the two
	// records with no station built. It belongs HERE and not in the per-segment
	// station generator, because the share it enforces is allocated over the
	// WHOLE build's paired-segment counts (loftStationShare) and a generator
	// handed one pair can never see them.
	//
	// S3 above admits a same-kind ArcSeg or CircleSeg pair, so a record with a
	// circular pair to measure does reach this gate from a real build; it is
	// exercised there and at its own entry point both
	// (loft_stations_internal_test.go's own header).
	if err := loftStationCapGate(p0, p1, offsets, walks0, walks1); err != nil {
		return nil, nil, nil, err
	}

	return offsets, walks0, walks1, nil
}

// loftSameKindGate is docs/loft-design.md Table S row S3 and Table P row P5
// in their arc form (a10-plan.md Part 3 PR 6): a pairing is admitted only
// when the two RECORDED SEGMENT TYPES are the same one of the three §1 and
// P5 enumerate — both LineSeg, both ArcSeg, or both CircleSeg — so a
// mixed-kind or free-form pair still refuses under today's sentinel.
//
// The test is on the CONCRETE recorded type, never on the resolved walk's
// own walkKind. walkCircular is one kind for a circle and an arc alike
// (extrude.go), so a walkKind test would admit an ArcSeg paired against a
// CircleSeg — a pairing §1 names mixed-kind and refuses, and one §5.1 has no
// station correspondence for, since it classifies an ArcSeg side as OPEN
// with m+1 station points and a full-turn CircleSeg side as CLOSED with m
// cyclic stations and states no rule for one of each. Admitting by concrete
// type is what keeps the code inside the contract the document carries.
//
// Beside S3 sits S7's STRUCTURAL arm (docs/loft-design.md Table S row S7,
// Table P row P5, and §4's gate-order paragraph, which places both arms):
// two paired circular segments whose own EFFECTIVE walk directions disagree
// walk in opposite directions, and that correspondence walls each side
// against the other's reversed walk — the very crossing §6's build-time
// audit proves in its AUDIT arm. The two arms answer one existence question
// and therefore carry one sentinel, S7's own ErrDegenerate (loft_audit.go's
// errLoftContact is the audit arm's spelling of it): a self-crossing shell
// bounds no solid under any evaluator, so this is never a staging refusal.
// What this arm buys is POSITION, not a different answer — it is decided
// from the two records alone, before a single station or triangle is built,
// where the audit would only reach the same verdict three build phases
// later.
//
// An ArcSeg's sweep is NOT structurally fixed CCW: walkOf's own ArcSeg arm
// (extrude.go) reads th0 = a0 + TStart*sweep, th1 = a0 + TEnd*sweep with
// sweep always forced positive, so the walk's own angle is monotonic in t and
// its EFFECTIVE direction is CCW exactly when TEnd > TStart — the identical
// formula validateSegmentWinding already enforces a CircleSeg's own CCW field
// must equal (record.go). loftCircularSegmentCCW reads that one shared
// formula, so an ArcSeg pair whose two recorded ranges run opposite ways is
// caught here beside the CircleSeg pair P5 names, rather than left to surface
// as S7's own crossing refusal three build phases later.
func loftSameKindGate(seg0, seg1 CurveSegment, loop, j, k int) error {
	t0, t1 := loftPairTypeOf(seg0), loftPairTypeOf(seg1)
	if t0 == loftPairUnadmitted || t0 != t1 {
		return fmt.Errorf(`%w: loop %d segment %d of the first profile and segment %d of the second are not the same admitted segment type; this evaluator pairs two LineSegs, two ArcSegs or two CircleSegs only`,
			ErrUnsupported, loop, j, k)
	}
	if t0 == loftPairLine {
		return nil
	}
	ccw0, ok0 := loftCircularSegmentCCW(seg0)
	ccw1, ok1 := loftCircularSegmentCCW(seg1)
	if ok0 && ok1 && ccw0 != ccw1 {
		return fmt.Errorf(`%w: loop %d's paired circular segments at segment %d/%d walk in opposite directions; the correspondence walls each side against the other's reversed walk, so the shell self-crosses and bounds no solid`,
			ErrDegenerate, loop, j, k)
	}
	return nil
}

// loftPairType is the three-way enumeration docs/loft-design.md §1 and Table
// P row P5 admit a loft pairing over, plus loftPairUnadmitted for every other
// recorded type. It reads the CONCRETE recorded segment type — the walk kind
// resolved from it is coarser (walkCircular covers a circle and an arc alike,
// extrude.go) and cannot state this contract.
type loftPairType uint8

const (
	// loftPairUnadmitted is every recorded type outside the three below —
	// each free-form kind, and each conic kind the seam records. A pair is
	// refused when either side reads this value, even when both do: §1
	// admits three enumerated types and nothing else.
	loftPairUnadmitted loftPairType = iota
	loftPairLine
	loftPairArc
	loftPairCircle
)

// loftPairTypeOf classifies one recorded segment into loftPairType. A segment
// normalizeSegment itself refuses — a nil typed pointer a decoded recipe can
// carry — reads loftPairUnadmitted rather than panicking; walkOf resolves
// each side ahead of this gate and reports that refusal first
// (validateLoftRecords' own precedence note), so the value is never the one a
// caller sees.
func loftPairTypeOf(seg CurveSegment) loftPairType {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return loftPairUnadmitted
	}
	switch seg.(type) {
	case LineSeg:
		return loftPairLine
	case ArcSeg:
		return loftPairArc
	case CircleSeg:
		return loftPairCircle
	default:
		return loftPairUnadmitted
	}
}

// loftCircularSegmentCCW reads a circular segment's own EFFECTIVE walk
// direction structurally, from ONE shared formula rather than trusting a
// per-kind field (loftSameKindGate's own doc comment): a CircleSeg's CCW
// flag is required to already equal TStart < TEnd (record.go's
// validateSegmentWinding), and an ArcSeg's own walkOf arm forces its sweep
// positive (extrude.go), so its angle is a STRICTLY INCREASING function of
// t and its own walk visits increasing angle exactly when TEnd > TStart —
// the identical formula. The false return is defensive, unreached from any
// real build today since loftSameKindGate has already proven both sides
// record the SAME circular type, which only CircleSeg and ArcSeg are.
func loftCircularSegmentCCW(seg CurveSegment) (bool, bool) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return false, false
	}
	switch s := seg.(type) {
	case CircleSeg:
		return s.CCW, true
	case ArcSeg:
		return s.TEnd > s.TStart, true
	default:
		return false, false
	}
}

// loftPlanesCoincide decides S5 over exact rationals on the recorded U/V/
// Origin floats (boolean_exact.go's xptOf/xcross/xdot, the take-the-floats-
// exactly discipline): the two planes coincide when their normals (U×V) are
// exactly parallel and the displacement between their origins lies in that
// plane. A tolerance here would refuse a legitimately thin loft, and the
// existence claim S5 makes is a structural zero volume, not a small one.
func loftPlanesCoincide(a, b PlaneRecord) bool {
	na := xcross(xptOf(a.U), xptOf(a.V))
	nb := xcross(xptOf(b.U), xptOf(b.V))
	cr := xcross(na, nb)
	if cr.x.Sign() != 0 || cr.y.Sign() != 0 || cr.z.Sign() != 0 {
		return false
	}
	d := xsub(xptOf(b.Origin), xptOf(a.Origin))
	return xdotSign(na, d) == 0
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
// chordCellDeltaUpper to reach the bound bounds.go's cellChordCurveAreaUpper
// obligates for the chord the build actually DREW (computeLoftChordedAllow,
// loft_moments.go); this field is never that composed bound on its own, and
// never the SET-distance sagitta sectionDelta names either.
// A LineSeg cell's own chord IS the curve it denotes, so its entry is
// exactly 0; a circular cell's own sagitta discharges this half exactly
// (loftCircularCellStations' own doc comment), so its entry equals its
// sagitta; a free-form cell's entry is spanMatchedDeltaUpper's own
// per-cell reading (spline_sagitta.go's pairStations), which can differ cell
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
	// cell, bounds.go's cellChordCurveAreaAllow tangentEnergyUpper obligation.
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
// (chordCellDeltaUpper) before any caller of bounds.go's
// chordedBoundaryVolumeAllow/chordedBoundaryMomentAllow/
// chordedBoundarySeamAllow (each of whose own doc comments name a
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
func loftPairings(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]segmentWalk, target float64, work0, work1 *freeformWork) ([]loftLoopPair, float64, float64, float64, error) {
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
// (loft_audit.go), whose collapse refusal is ErrDegenerate — a claim that no
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
