package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type loftCircularBounder struct{ seg CurveSegment }

func (b loftCircularBounder) BoundAt(t *big.Rat, u, v float64) proofbound.WalkEndBound {
	return circularPointBound(b.seg, t, u, v)
}

func loftCircularSide(w survey2d.SegmentWalk, seg CurveSegment) loftmesh.LoftCircularSide {
	radius, sweep, enclosed := circularWalkEnclosures(seg)
	tStart, dt, rangeOK := circularSegmentRange(seg)
	return loftmesh.LoftCircularSide{
		Walk: w, Radius: radius, Sweep: sweep, Enclosed: enclosed,
		TStart: tStart, DT: dt, RangeOK: rangeOK,
		EndRadialUpper: arcNaturalEndRadialUpper(seg), Bounder: loftCircularBounder{seg: seg},
	}
}

func loftStationPoints(stations []loftmesh.LoftStation) []Point2 {
	points := make([]Point2, len(stations))
	for i, station := range stations {
		points[i] = Point2{U: station.U, V: station.V}
	}
	return points
}

// This file places the stations a loft's wall chords run between, and proves
// how far each chord chain departs from the curve it approximates.
//
// One chord target is chosen for the whole loft, and every cell derives its
// own station count from that target, so the two paired curves are sampled at
// matched parameters rather than at independently chosen ones. The certified
// sagitta and chord bounds are what turn that departure into the section
// displacement the payload publishes; a cell whose departure cannot be
// bounded refuses through errLoftSagittaUnderivable. The station cap bounds
// the work before any of it is done. See docs/loft-design.md §5.2.

// loftStationCap is docs/loft-design.md §5.1's ceiling on a build's TOTAL
// station count Σstations (§7) — the soft limit that keeps the chord chain
// from being what carries §6's audit past the pair-test ceiling S8 owns. §14
// points here for the value; the derivation follows.
//
// §7 fixes the assembled triangle count: 2·Σstations wall triangles, plus each
// of the two caps' own polygon-with-holes triangulation, which triangulate.go
// bridges into a simple polygon and so answers Σstations + 2H − 2 triangles
// over H hole loops. So
//
//	F = 2·Σstations + 2·(Σstations + 2H − 2) = 4·Σstations + 4H − 4
//
// S8 (internal/loftmesh/loft_audit.go) refuses unless F*(F−1)/2 is at or below
// proofbound.MaxFacetPairTestsPerCall (8_000_000, internal/proofbound/budget.go), which admits F ≤ 4000:
// 4000·3999/2 = 7_998_000 passes and 4001·4000/2 = 8_002_000 does not.
//
// H is bounded by Σstations itself. Every loop holds at least one segment and
// every paired segment chords at m ≥ 1 (§5.1), so a build of L loops has
// Σstations ≥ L and therefore H = L − 1 ≤ Σstations − 1. Taking that worst
// case,
//
//	F ≤ 4·Σstations + 4·(Σstations − 1) − 4 = 8·Σstations − 8
//
// and at Σstations = 500 that is F ≤ 3992, whose 3992·3991/2 = 7_966_036 is
// STRICTLY below the ceiling — which is the property §5.1 requires of this
// constant. The hole-free shape §5.1's own "F ≈ 4Σ − 4" names is far smaller
// still: F = 1996 at the cap, 1_991_010 pair tests.
//
// It also leaves room for every fixture §13 requires: that section's reference
// wedge forces 64 stations and its calibrated twin settles at 65, so the cap
// sits more than seven times above the largest fixture that ships.
//
// The cap is deliberately NOT freeform.MaxChordsPerWalk (tessellate.go). That constant
// bounds how finely ONE curve may be chorded and knows nothing of how many
// curves a build holds; this one bounds the build.
const loftStationCap = 500

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

// loftPairCounts reads docs/loft-design.md §5.1's two build-wide counts off
// Table P over both records: P, the total paired-segment count, and C, the
// number of same-kind circular pairs among them. Both are decided from the two
// authenticated records alone — no station is generated to read either.
//
// A pair is circular when BOTH sides' walks are circular; a mixed-kind pair is
// S3's refusal (validateLoftRecords) and is counted in P like any other, since
// P's own entitlement is one station per paired segment whatever its kind. Only
// loop0's segment counts are read: S2 has already proved loop1 carries the same
// count, which is what makes one loop's shape the pair count for both.
//
// The accumulation is checked (proofbound.WallCheckedAdd, internal/proofbound/budget.go) and answers false on
// overflow rather than wrapping, the discipline §5.1 states for every sum the
// mMax comparison reads.
func loftPairCounts(loops0 []LoopRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (uint64, uint64, bool) {
	var p, c uint64
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		for j := range n {
			var ok bool
			if p, ok = proofbound.WallCheckedAdd(p, 1); !ok {
				return 0, 0, false
			}
			k := (j + off) % n
			if walks0[i][j].Kind == survey2d.WalkCircular && walks1[i][k].Kind == survey2d.WalkCircular {
				if c, ok = proofbound.WallCheckedAdd(c, 1); !ok {
					return 0, 0, false
				}
			}
		}
	}
	return p, c, true
}

// loftStationShare allocates docs/loft-design.md §5.1's per-segment share of
// the station cap:
//
//	mMax = 1 + max(0, (loftStationCap - P) / C)      // integer division
//
// Every paired segment is entitled to its first station — a LineSeg pair's
// whole entitlement (m = 1, §7) — and each of the C circular pairs may take at
// most mMax. Because C counts the circular pairs AMONG P, a circular pair's m
// stations SUBSUME that first-station entitlement rather than adding to it, so
// §5.1's own sum shows no build every pair of which passes S15 can exceed the
// cap.
//
// The caller must not reach here with C == 0: §5.1 states a build with no
// circular pair never consults the cap at all, and dividing by C would be
// undefined besides.
//
// A record whose own P already exceeds the cap clamps to mMax = 1, which §5.1
// carves out deliberately: such a record is past chording altogether and S8 is
// what refuses it, over the assembled triangle count §6's own preflight
// computes. Refusing it here instead would refuse a mixed build while
// admitting an all-LineSeg build of the identical triangle count.
func loftStationShare(p, c uint64) int {
	q := max(int64(0), (int64(loftStationCap)-int64(p))/int64(c)) //nolint:gosec // p and c are paired-segment counts proofbound.WallCheckedAdd already proved do not overflow, and a record large enough to pass int64 cannot be built from the process's memory limits.
	return 1 + int(q)
}

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

// loftCellStations generates one paired loft segment's shared chord stations:
// a kind switch on w0/w1's own survey2d.WalkKind, fixed here for every future arm
// (a10-plan.md Part 3 PR 5's own constraint). Every arm publishes the
// identical contract — two per-plane station chains at a SHARED count, plus
// the sagitta this cell's own chording commits — so a later Tier A free-form
// arm (docs/spline-design.md §6.2.1) is an added case, never a rewrite of
// this one: an arc-shaped signature naming a radius or a sweep would force
// exactly the rewrite this ordering exists to avoid.
//
// stations0/stations1 each carry ONLY this segment's own interior stations,
// never its shared end point — the next segment's own first station (or the
// loop's wrap) supplies it, the convention loftLoopPair's own doc comment
// states, and what makes a loop's own chain total the count
// docs/loft-design.md §7 states for it rather than one stated here.
//
// sagittaUpper is the proven upper bound on max_s |curve(s) - chord(s)|
// under the SAME parameter this cell's own uniform stations walk — a
// PARAMETER-MATCHED bound, never a set distance from some chord point to the
// curve (internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper doc comment states the
// distinction this field's name exists to keep visible). The LineSeg arm's
// chord IS the recorded segment, so its bound is exactly zero; the circular
// arm composes the two terms docs/loft-design.md §5.2's table lists for a
// chorded cell — that table's certified per-cell sagitta and the displacement
// its stations carry, each with the derivation and rounding direction that
// table states and the provenance mark §5.1's Table C gives those stations —
// and answers the refusal §5.2's table assigns those terms.
//
// seg0/seg1 are the two RECORDED segments w0/w1 were resolved from. An arm
// whose bound is a proof rather than a held float needs them: every enclosure
// docs/loft-design.md §5.2 names is stated by the record, never by the walk,
// whose radius is a math.Hypot and whose angles are a math.Atan2 the walk
// itself declares it cannot enclose (extrude.go's circularWalk).
//
// target is loftChordTarget's own single per-build reading, never
// recomputed per cell. work0/work1 are the two records' own free-form work
// counters (docs/spline-design.md §5.2): unused by both arms below, carried
// through so a future free-form arm never needs a second counter — the same
// pass-through shape evalLoft's own doc comment already states for its own
// work0/work1 parameters, and this generator's own interface constraint
// (a10-plan.md Part 3 PR 5) fixes them into the signature ahead of that arm
// existing to consume them.
//
// stationRoundUpper is docs/loft-design.md Table S row S14 (a10-plan.md Part
// 3 PR 6): the proven rounding a COMPUTED station commits, taken as a MAX
// over this cell's own stations on both sides — a component of delta, never
// sectionDelta. NEITHER arm is exempt, and the LineSeg arm is not the
// zero it would be if a kind could grant one: §5.2 PINS a station by its own
// NATURAL parameter, never by the kind of segment it sits on, so this arm
// charges exactly zero where its two stations are UNTRIMMED recorded
// endpoints and charges its own certified lineWalkEndBound wherever a TRIMMED
// parameter made lerp2 compute one. Each arm's own doc comment states its
// mechanism.
//
// matchedDelta is the CHORD-TO-CURVE HALF of docs/loft-design.md §5.2's
// matchedDelta row — the half a consumer composes with the build's own delta
// (chordCellDeltaUpper) to reach internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper own
// matchedDeltaUpper obligation (F1's rule) — ONE ENTRY PER CELL, never a single per-segment
// scalar, since a bisected free-form arm can settle cells of that one paired
// segment at different depths and so at different matched-delta readings.
// len(matchedDelta) always equals len(stations0), the per-cell count every
// arm below publishes. It is read per cell rather than from sagittaUpper: the
// LineSeg arm's chord IS the curve, so every entry is exactly 0; the circular
// arm's own sagitta discharges that half exactly, so every entry equals
// the segment's own sagittaUpper (loftCircularCellStations' own doc comment);
// a future free-form arm's own per-cell reading can vary within these two
// extremes cell to cell.
func loftCellStations(w0, w1 survey2d.SegmentWalk, seg0, seg1 CurveSegment, target float64, work0, work1 *freeform.FreeformWork) ([]Point2, []Point2, float64, []float64, float64, error) { //nolint:unparam // work0/work1 are part of the fixed kind-switch interface every future arm shares; the ARC and LineSeg arms below are the two that do not need them yet.
	switch {
	case w0.Kind == survey2d.WalkLine && w1.Kind == survey2d.WalkLine:
		return loftLineCellStations(w0, w1)
	case w0.Kind == survey2d.WalkCircular && w1.Kind == survey2d.WalkCircular:
		return loftCircularCellStations(w0, w1, seg0, seg1, target)
	default:
		// Unreached from any real build today: validateLoftRecords' own S3
		// gate refuses every mixed-kind pair before loftPairings ever calls
		// this function (loftSameKindGate). A defensive refusal, not a dead
		// branch a caller could reach silently: a future kind this switch
		// has no case for yet must still fail loud rather than fall through
		// into either analytic arm's own assumptions.
		return nil, nil, 0, nil, 0, fmt.Errorf(`%w: this loft evaluator has no chord station rule for this segment-kind pairing`, ErrUnsupported)
	}
}

// loftLineCellStations delegates the station proof to internal/tessellation.
func loftLineCellStations(w0, w1 survey2d.SegmentWalk) ([]Point2, []Point2, float64, []float64, float64, error) {
	a, b, sagitta, matched, round, err := loftmesh.LoftLineCellStations(w0, w1)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	return loftStationPoints(a), loftStationPoints(b), sagitta, matched, round, nil
}

// loftSettleStationCount delegates the station proof to internal/tessellation.
func loftSettleStationCount(w0, w1 survey2d.SegmentWalk, seg0, seg1 CurveSegment, target float64) (int, float64, float64, error) {
	return loftmesh.LoftSettleStationCount(loftCircularSide(w0, seg0), loftCircularSide(w1, seg1), target)
}

// loftCircularCellStations delegates the station proof to internal/tessellation.
func loftCircularCellStations(w0, w1 survey2d.SegmentWalk, seg0, seg1 CurveSegment, target float64) ([]Point2, []Point2, float64, []float64, float64, error) {
	a, b, sagitta, matched, round, err := loftmesh.LoftCircularCellStations(
		loftCircularSide(w0, seg0), loftCircularSide(w1, seg1), target,
	)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	return loftStationPoints(a), loftStationPoints(b), sagitta, matched, round, nil
}

// perCellArcUpper is one paired segment's own per-cell arc-length upper
// bound, shared by every one of its m uniformly-stepped cells
// (computeLoftChordedAllow, loft_moments.go). Uniform angular stepping means
// each of the m cells carries the SAME true share of the whole sweep, so
// dividing a proven upper bound on the WHOLE segment's length by m stays an
// upper bound on each share.
//
// For a circular segment (CircleSeg/ArcSeg) that whole-length bound is
// moments.go's circularLengthInterval — an EXACT rational bracket on the
// segment's true length — never survey2d.SegmentWalk.lengthUpper: that field's own
// bound is deliberately loose (proofbound.CircularSweepUpper bounds any ArcSeg's sweep
// by the full 2*pi it could reach, never the sweep THIS record states, per
// its own doc comment), so a quarter-turn arc's lengthUpper overstates its
// true length by roughly 4x — a slack that would flow straight through this
// division into proofbound.CellChordCurveAreaUpper's own arcLenUpper argument and
// quadruple the wall/seam/cap terms it feeds (an earlier version of this
// function did exactly that, measured Suspect on the calibrated reference
// wedge before this fix). The tight bracket is what keeps the per-cell share
// close to the true one.
//
// For the LineSeg arm (m=1, and every other kind circularLengthInterval
// declines) this falls back to survey2d.SegmentWalk.lengthUpper exactly, unaffected —
// a straight chord's own recorded length bound was never the loose one. A
// non-finite whole-length bound propagates rather than silently shrinking
// under the division.
func perCellArcUpper(seg CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	if ns, err := normalizeSegment(seg); err == nil {
		if iv, ok := circularLengthInterval(ns); ok {
			return proofbound.UpRound(proofbound.RatFloatUp(iv.Hi) / float64(m))
		}
	}
	if proofbound.IsNonFinite(w.LengthUpper) {
		return math.Inf(1)
	}
	return proofbound.UpRound(w.LengthUpper / float64(m))
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

// perCellTangentEnergy is internal/proofbound/bounds.go's proofbound.CellChordCurveAreaAllow own
// tangentEnergyUpper obligation for ONE cell of this walk: a proven upper bound
// on the integral of |curve'(s) - chord|^2 over the cell's own shared
// parameter, or +Inf where this evaluator cannot prove one.
//
// Its two operands are BOTH read from the RECORD's own certified enclosures —
// perCellArcUpper over circularLengthInterval, and loftCertifiedChordLower over
// circularWalkEnclosures — never from the walk's own held math.Hypot radius and
// math.Atan2 angles, neither of which the walk can enclose (extrude.go's
// circularWalk). proofbound.UniformSpeedTangentEnergyUpper's published energy DECREASES in
// its chord operand, so a chord read off those floats can overstate the true
// chord and understate the energy every consumer downstream spends: a held
// value wearing a proof's clothes, which circularWalkEnclosures' own doc
// comment forbids.
//
// It is dispatched on the WALK KIND rather than shared across every arm,
// because the obligation proofbound.UniformSpeedTangentEnergyUpper discharges rests on the
// shared parametrization having CONSTANT SPEED — a property of the arm that
// placed the stations, not of the cell's geometry. The circular arm's
// uniform-ANGLE stations (loftCircularCellStations) are constant speed on a
// circle, which is what discharges it; a straight walk's chord IS its curve, so
// its deviation is identically zero. Any FUTURE kind — the free-form arm's own
// span-uniform native fraction above all, which is NOT constant speed — answers
// +Inf here until it carries a proof of its own, so it degrades
// proofbound.CellChordCurveAreaAllow to that helper's premise-free arm rather than being
// silently handed a bound whose premise it does not meet.
func perCellTangentEnergy(seg CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	switch w.Kind {
	case survey2d.WalkLine:
		return 0
	case survey2d.WalkCircular:
		return proofbound.UniformSpeedTangentEnergyUpper(perCellArcUpper(seg, w, m), loftCertifiedChordLower(seg, m))
	default:
		return math.Inf(1)
	}
}

// circularStationChain delegates the station proof to internal/tessellation.
func circularStationChain(w survey2d.SegmentWalk, seg CurveSegment, m int) ([]Point2, float64) {
	stations, delta := loftmesh.LoftCircularStationChain(loftCircularSide(w, seg), m)
	return loftStationPoints(stations), delta
}

// walkEndPlaneDelta delegates the station proof to internal/tessellation.
func walkEndPlaneDelta(bound proofbound.WalkEndBound) float64 {
	return loftmesh.WalkEndPlaneDelta(bound)
}

// arcNaturalEndRadialUpper charges docs/loft-design.md §5.2's ARC-END RADIAL
// RESIDUAL: an upper bound on | ‖End − Center‖ − ‖Start − Center‖ | for a
// recorded ArcSeg whose walk reaches the natural bound t == 1, and exactly
// zero for every other segment and every other bound.
//
// The term exists because a pin is a statement about a POSITION and not about
// the point the record denotes at that parameter. An ArcSeg records three
// points and the curve it denotes takes its radius from Start ALONE —
// circularEndpointInterval and circularWalkEnclosures (moments.go) both read
// |Start − Center|, the reading docs/sketch-seam-design.md states outright —
// so the denoted point at t == 1 lies at THAT radius and End's own angle. The
// walk holds the recorded End there (arcWalkEnd), and the two coincide only
// where the two radii are equal. Nothing certifies that: validateSegment's
// ArcSeg arm (record.go) tests point finiteness and the parameter range,
// seam.go records geom.Arc's three points verbatim, and sketch's own arc
// radius constraint is solved to solver tolerance rather than proven. So the
// residual is CHARGED. It is not a gate: a record whose radii differ is
// measured, never admitted or refused on the measurement, which is CLAUDE.md's
// reject-only rule read correctly — this term bounds a record and can never
// bless one.
//
// It is charged at t == 1 ALONE. At t == 0 the denoted point IS Start, by the
// definition of the denoted radius, so the true displacement there is zero and
// a generator's enclosure WIDTH read at that bound would publish a positive
// displacement for a station that has none. This is why the charge lives here
// rather than in arcWalkEnd, whose zero every other consumer of a pinned
// endpoint POSITION already relies on (pinArcWalkEnds' own doc comment).
//
// The bound is exact-rational throughout and never a float subtraction of two
// square roots. |r1 − r0| is |r1² − r0²| / (r1 + r0); the numerator is the
// exact rational difference of the two recorded squared distances, and the
// denominator is replaced by a rounded-DOWN sum of the two radii
// (proofbound.RatSqrtDown), which can only enlarge the quotient. proofbound.RatFloatUp rounds the
// result out once. Equal squared radii answer exactly zero, so a record that
// does state an exact circle keeps the zero delta §5.2 grants it.
//
// A denominator that cannot be shown positive answers +Inf, which the caller
// refuses on rather than publishing a substitute — the S14 discipline §5.2's
// table states for every term in it. It is defensive: it needs both recorded
// radii to round down to zero while their exact squares differ.
func arcNaturalEndRadialUpper(seg CurveSegment) float64 {
	arc, ok := seg.(ArcSeg)
	if !ok || (arc.TStart != 1 && arc.TEnd != 1) {
		return 0
	}
	dx0 := exactCoordinateDelta(arc.Start.U, arc.Center.U)
	dy0 := exactCoordinateDelta(arc.Start.V, arc.Center.V)
	dx1 := exactCoordinateDelta(arc.End.U, arc.Center.U)
	dy1 := exactCoordinateDelta(arc.End.V, arc.Center.V)
	r0 := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
	r1 := new(big.Rat).Add(new(big.Rat).Mul(dx1, dx1), new(big.Rat).Mul(dy1, dy1))

	diff := new(big.Rat).Sub(r1, r0)
	if diff.Sign() == 0 {
		return 0
	}
	diff.Abs(diff)

	den := new(big.Rat).Add(proofarith.FloatRat(proofbound.RatSqrtDown(r0)), proofarith.FloatRat(proofbound.RatSqrtDown(r1)))
	if den.Sign() <= 0 {
		return math.Inf(1)
	}
	up := proofbound.RatFloatUp(new(big.Rat).Quo(diff, den))
	if proofbound.IsNonFinite(up) {
		return math.Inf(1)
	}
	return up
}

// circularSegmentRange states a recorded circular segment's own parameter
// range exactly: the start parameter and the signed width TEnd − TStart, both
// over the rationals. circularStationChain divides that width into m equal
// parts, so the division has to happen where no rounding can enter it — a
// station parameter rounded to a float would name a point that divides the
// sweep slightly unevenly, and the per-cell sagitta the caller publishes is
// derived from the EVEN division alone (circularPointBound's own doc comment).
//
// A kind with no circular parameter range answers false, and the caller
// refuses.
func circularSegmentRange(seg CurveSegment) (*big.Rat, *big.Rat, bool) {
	var tStart, tEnd float64
	switch seg := seg.(type) {
	case CircleSeg:
		tStart, tEnd = seg.TStart, seg.TEnd
	case ArcSeg:
		tStart, tEnd = seg.TStart, seg.TEnd
	default:
		return nil, nil, false
	}
	start := proofarith.FloatRat(tStart)
	if start == nil {
		return nil, nil, false
	}
	return start, exactCoordinateDelta(tEnd, tStart), true
}

// loftCertifiedSagittaUpper delegates the station proof to internal/tessellation.
func loftCertifiedSagittaUpper(seg CurveSegment, m int) float64 {
	if m <= 0 {
		return math.Inf(1)
	}
	radius, sweep, enclosed := circularWalkEnclosures(seg)
	return loftmesh.LoftCertifiedSagittaUpper(radius, sweep, enclosed, m)
}

// loftCertifiedChordLower delegates the station proof to internal/tessellation.
func loftCertifiedChordLower(seg CurveSegment, m int) float64 {
	if m <= 0 {
		return 0
	}
	radius, sweep, enclosed := circularWalkEnclosures(seg)
	return loftmesh.LoftCertifiedChordLower(radius, sweep, enclosed, m)
}

// errLoftSagittaUnderivable is docs/loft-design.md Table S row S14's refusal:
// a chorded circular pair for which the certified per-cell sagitta has no
// derivation from the record. The body exists and the chord set is buildable;
// only one of its proven displacement terms cannot be stated, which is a
// derivation gap in this evaluator's certified circular enclosures rather than
// a shape rule — so the sentinel is ErrUnsupported, and no finite value is
// published in its place.
var errLoftSagittaUnderivable = loftmesh.ErrLoftSagittaUnderivable
