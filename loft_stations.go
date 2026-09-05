package decad

import (
	"fmt"
	"math"
	"math/big"
)

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
// S8 (loft_audit.go) refuses unless F*(F−1)/2 is at or below
// maxFacetPairTestsPerCall (8_000_000, budget.go), which admits F ≤ 4000:
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
// The cap is deliberately NOT maxChordsPerWalk (tessellate.go). That constant
// bounds how finely ONE curve may be chorded and knows nothing of how many
// curves a build holds; this one bounds the build.
const loftStationCap = 500

// loftStationCapError is docs/loft-design.md Table S row S15's refusal: a
// same-kind circular pair whose settled station count `m` (§5.1's joint
// walk-up) exceeds the per-segment share loftStationShare allocates it.
//
// It is a type rather than an fmt.Errorf wrapper because the refusal must NAME
// the segment whose own share it exceeded (§5.1) while still answering
// errors.Is for errTooManyChords, the sentinel §5.1 assigns this row (spline
// design Table R row R8). Wrapping errTooManyChords with %w would prepend that
// sentinel's own text — "the chord tolerance asks for more than 16384 chords
// on one curve" — which names no segment and describes a caller-supplied
// tessellation tolerance a loft has no such knob for (§5.1: "The target is not
// a caller option"). Unwrap keeps errors.Is answering for both errTooManyChords
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

func (e *loftStationCapError) Unwrap() error { return errTooManyChords }

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
// The accumulation is checked (wallCheckedAdd, budget.go) and answers false on
// overflow rather than wrapping, the discipline §5.1 states for every sum the
// mMax comparison reads.
func loftPairCounts(loops0 []LoopRecord, offsets []int, walks0, walks1 [][]segmentWalk) (uint64, uint64, bool) {
	var p, c uint64
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		for j := range n {
			var ok bool
			if p, ok = wallCheckedAdd(p, 1); !ok {
				return 0, 0, false
			}
			k := (j + off) % n
			if walks0[i][j].kind == walkCircular && walks1[i][k].kind == walkCircular {
				if c, ok = wallCheckedAdd(c, 1); !ok {
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
	q := max(int64(0), (int64(loftStationCap)-int64(p))/int64(c)) //nolint:gosec // p and c are paired-segment counts wallCheckedAdd already proved do not overflow, and a record large enough to pass int64 cannot be built from a decoded recipe's own resource limits.
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
// walk-up that settles m is what asks for that term, and errTooManyChords bare
// is chordCount's own per-walk ceiling.
func loftStationCapGate(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]segmentWalk) error {
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
			if w0.kind != walkCircular || w1.kind != walkCircular {
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
// added to the recipe wire format for it (a10-plan.md Q2).
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
// Task 1's resolve-once rule.
func loftChordTarget(p0, p1 ProfileRecord, walks0, walks1 [][]segmentWalk) (float64, error) {
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
// a kind switch on w0/w1's own walkKind, fixed here for every future arm
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
// curve (bounds.go's cellChordCurveAreaUpper doc comment states the
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
// (chordCellDeltaUpper) to reach bounds.go's cellChordCurveAreaUpper own
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
func loftCellStations(w0, w1 segmentWalk, seg0, seg1 CurveSegment, target float64, work0, work1 *freeformWork) ([]Point2, []Point2, float64, []float64, float64, error) { //nolint:unparam // work0/work1 are part of the fixed kind-switch interface every future arm shares; the ARC and LineSeg arms below are the two that do not need them yet.
	switch {
	case w0.kind == walkLine && w1.kind == walkLine:
		return loftLineCellStations(w0, w1)
	case w0.kind == walkCircular && w1.kind == walkCircular:
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

// loftLineCellStations is the LineSeg arm: one station per side, at the
// segment's own recorded start, with zero sagitta and zero matchedDelta — a
// straight wall's own chord IS the recorded segment, so there is no curve
// for it to depart from. m is fixed at 1, so this arm's STATION SET is
// bit-identical to every LineSeg pairing this evaluator built before the
// station generator existed.
//
// Its stationRoundUpper is NOT unconditionally zero, and that is the one
// thing this arm does not inherit from that earlier shape. docs/loft-design.md
// §5.2 pins a station by its own NATURAL parameter — t == 0 or t == 1 — and
// never by the kind of segment it sits on, and §5.1's Table C marks a TRIMMED
// LineSeg end GENERATED for exactly that reason. So this arm reads what its
// two walks actually prove rather than asserting a zero its kind does not
// grant.
//
// It charges each side's startBound and NEVER its endBound. Each station this
// arm emits is its own walk's START, and a cell's terminal station is the NEXT
// segment's own start — or the loop's wrap back to the first segment's — which
// that segment charges when its own cell is generated. Charging endBound here
// would charge a point this build never holds, and would double-count the
// junction the two segments share.
//
// At an UNTRIMMED start the term is exactly zero, proven rather than assumed:
// lerp2 and moments.go's ratLerp both special-case t == 0 and t == 1 to the
// recorded Point2 verbatim, so lineWalkEndBound's two rationalFloatError calls
// measure no gap at all. An untrimmed LineSeg-only pairing therefore still
// publishes the bit-identical zero delta — and the Exact readings §8 gives it
// — that it always did.
//
// At a TRIMMED start the term is whatever lineWalkEndBound proves. walkOf
// fills such a walk's start from lerp2 in FLOAT (extrude.go's LineSeg arm),
// while the point the record denotes is the exact rational ratLerp, and
// lineWalkEndBound stamps the outward-rounded gap between them. That gap is a
// real displacement of a held vertex from the point the record denotes, so
// leaving it uncharged would publish a bound smaller than the displacement the
// body actually carries. Such a record is caller-reachable rather than a
// corner case: seam.go's recordEdge records a certified Partial line fragment
// over a non-natural range verbatim, and no Table S row excludes one.
//
// The refusal is DEFENSIVE and no admitted record reaches it. walkEndPlaneDelta
// answers +Inf only where lineWalkEndBound could not state the denoted point as
// a rational, and ratLerp fails solely on a non-finite coordinate — which the
// record gates exclude long before any walk is resolved. It stands so that an
// underivable term can never be published as a finite bound, which is the S14
// discipline §5.2's table states for every term in it.
func loftLineCellStations(w0, w1 segmentWalk) ([]Point2, []Point2, float64, []float64, float64, error) {
	round := math.Max(walkEndPlaneDelta(w0.startBound), walkEndPlaneDelta(w1.startBound))
	if isNonFinite(round) {
		return nil, nil, 0, nil, 0, errLoftStationDisplacementUnderivable
	}
	return []Point2{{U: w0.startU, V: w0.startV}}, []Point2{{U: w1.startU, V: w1.startV}}, 0, []float64{0}, round, nil
}

// loftSettleStationCount runs docs/loft-design.md §5.1's JOINT WALK-UP for one
// same-kind circular pair and answers the count it settles on, together with
// each side's own certified per-cell sagitta AT that count.
//
// chordCount picks each side's OWN count against target independently, and
// max(m0, m1) only SEEDS the walk: from there BOTH sides' certified sagittae
// are recomputed at each candidate and the count increments until both are at
// or below the target. Settling that way needs no monotonicity claim about the
// certified sagitta as the count grows.
//
// It is a pure function of the two walks, the two RECORDED segments and the
// target, and it charges no work budget — which is what lets the record-only
// station-cap gate (loftStationCapGate, S15) settle the same m the
// construction phase will, with no station built and no triangle assembled.
//
// The two refusals it raises are the two docs/loft-design.md §4's gate-order
// paragraph assigns to this walk-up: errLoftSagittaUnderivable is S14's
// DERIVATION arm — a candidate count whose certified sagitta has no derivation
// from the record — and errTooManyChords is the per-walk ceiling chordCount
// itself enforces, raised again here because the certified sagitta shrinks
// with m but is floored by its own enclosure width, so a target below that
// floor would otherwise walk forever. That per-walk ceiling is NOT the station
// cap: it bounds one curve's own chording and knows nothing of how many curves
// the build holds, while loftStationCap bounds the build's station total.
func loftSettleStationCount(w0, w1 segmentWalk, seg0, seg1 CurveSegment, target float64) (int, float64, float64, error) {
	m0, _, err := chordCount(w0, target, chordWalkMin(w0))
	if err != nil {
		return 0, 0, 0, err
	}
	m1, _, err := chordCount(w1, target, chordWalkMin(w1))
	if err != nil {
		return 0, 0, 0, err
	}

	m := max(m0, m1)
	for {
		s0 := loftCertifiedSagittaUpper(seg0, m)
		s1 := loftCertifiedSagittaUpper(seg1, m)
		if isNonFinite(s0) || isNonFinite(s1) {
			return 0, 0, 0, errLoftSagittaUnderivable
		}
		if s0 <= target && s1 <= target {
			return m, s0, s1, nil
		}
		if m >= maxChordsPerWalk {
			return 0, 0, 0, errTooManyChords
		}
		m++
	}
}

// loftCircularCellStations is the circular arm. It settles the pair's shared
// station count through loftSettleStationCount — docs/loft-design.md §5.1's
// JOINT WALK-UP, whose own doc comment owns that rule — and then walks BOTH
// sides at the count it settles on, never at either side's own smaller count,
// which is the correspondence a loft wall needs.
//
// What the walk-up compares against the target, and what this arm publishes,
// is loftCertifiedSagittaUpper's enclosure over the RECORD's own radius and
// sweep brackets — never the held float chordCount itself returns. That float
// is computed from the walk's math.Hypot radius and its math.Atan2 sweep, with
// no enclosure and no outward rounding, so it can decide a COUNT and nothing
// more (docs/loft-design.md §5.2's third rule). A candidate count whose
// certified sagitta has no derivation refuses at Table S row S14, in the arm
// and at the phase §4's gate-order paragraph assigns it, and the refusal never
// degrades into a published zero or a held substitute.
//
// The arm publishes the two terms §5.2's table lists for a chorded cell
// SEPARATELY, each read across the two sides under the max-versus-sum rule
// that table states for it rather than one stated here: sagittaUpper is the
// per-cell sagitta, which the caller accumulates into sectionDelta, and
// stationUpper is the stations' own displacement, which the caller
// accumulates into stationRound and thence into delta. The table's own
// sectionDelta and delta rows own that split, and the two terms are never
// added into one another here. Only the sagitta half is walked up against
// the target: the target names the chord DEPTH the chording commits to,
// while the station displacement is a rounding term of the generator's own
// arithmetic, read once the count has settled.
//
// It also discharges loftCellStations' own
// PARAMETER-MATCHED obligation: stations are placed at uniform PARAMETER t_k =
// TStart + (k/m)*(TEnd-TStart), which for a circular walk is uniform in ANGLE
// because t is the walk's own sweep fraction — and under that uniform-angle
// parametrization the exact departure sup_s |arc(s) - chord(s)| over one chord
// between the EXACT recorded points at t_k and t_(k+1) IS the cell's sagitta
// 2r·sin²(Δθ/4m), attained at s = 1/2. The certified half encloses THAT
// quantity from above.
//
// The chord this arm actually draws runs between two stations carrying the
// provenance §5.1's Table C marks them with rather than between those two
// exact points, which is precisely the gap the station-displacement half
// closes: the sagitta alone bounds a chord the build never
// drew, and a built chord can and does depart from the curve by more than it.
// A SET distance from an arbitrary chord point to the curve would bound
// something else entirely, and the uniform-angle station rule — read at an
// EXACT rational parameter, never a float rounding of one (circularPointBound)
// — is what keeps both halves readings of the same quantity.
//
// matchedDelta is loftCellStations' own per-cell obligation — the CHORD-TO-
// CURVE half of docs/loft-design.md §5.2's matchedDelta row, which the
// consumer composes with the build's own delta (chordCellDeltaUpper), never
// the whole row: this arm's sagitta discharges that half EXACTLY (the
// paragraph above), and every cell of one uniformly-stepped circular segment
// shares the same true angular width, so the same value — math.Max(s0, s1) —
// is the correct, exact per-cell reading for all m cells, not merely a safe
// upper bound repeated m times.
func loftCircularCellStations(w0, w1 segmentWalk, seg0, seg1 CurveSegment, target float64) ([]Point2, []Point2, float64, []float64, float64, error) {
	m, s0, s1, err := loftSettleStationCount(w0, w1, seg0, seg1, target)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}

	stations0, d0 := circularStationChain(w0, seg0, m)
	stations1, d1 := circularStationChain(w1, seg1, m)

	// S14, in the arm and at the phase §4's gate-order paragraph assigns the
	// station-displacement term: a side whose stations have no proven
	// displacement from the recorded points they stand for cannot state this
	// cell's own chord bound at all, so the cell refuses rather than
	// publishing the certified sagitta alone — which would be a bound on a
	// chord this build did not draw.
	stationUpper := math.Max(d0, d1)
	if isNonFinite(stationUpper) {
		return nil, nil, 0, nil, 0, errLoftStationDisplacementUnderivable
	}

	// S16 over this segment's own INTERIOR cells — an early local guard that
	// refuses at the arm's own seam, so a caller reaching the generator
	// directly gets the row rather than a chain it has no case for.
	// loftOneSidedCellGate owns the COMPLETE row, cyclically over the whole
	// loop's assembled chains, and is the only site that can see this
	// segment's terminal cell, a pair settled at m = 1, or a LineSeg-pair
	// cell — none of which has a consecutive pair inside one segment. The two
	// sites agree by construction: both compare the two sides' station
	// equality at the same cell, and this one reads a strict subset of the
	// cells that one does.
	//
	// A same-kind pair with a genuinely positive sweep on both sides never
	// reaches this — distinct k give distinct angles give distinct (cos, sin)
	// pairs — so it is reachable only from a degenerate walk (a zero radius)
	// that a real sketch authentication does not produce (spline_fit.go's own
	// dedup plays the analogous role for the free-form arm's own R3 risk).
	for k := range m - 1 {
		eq0 := stations0[k] == stations0[k+1]
		eq1 := stations1[k] == stations1[k+1]
		if eq0 != eq1 {
			return nil, nil, 0, nil, 0, fmt.Errorf(`%w: chord cell %d of this paired segment collapses to one point on only one of the two sections`, ErrUnsupported, k)
		}
	}

	sagitta := math.Max(s0, s1)
	matchedDelta := make([]float64, m)
	for i := range matchedDelta {
		matchedDelta[i] = sagitta
	}
	return stations0, stations1, sagitta, matchedDelta, stationUpper, nil
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
// segment's true length — never segmentWalk.lengthUpper: that field's own
// bound is deliberately loose (circularSweepUpper bounds any ArcSeg's sweep
// by the full 2*pi it could reach, never the sweep THIS record states, per
// its own doc comment), so a quarter-turn arc's lengthUpper overstates its
// true length by roughly 4x — a slack that would flow straight through this
// division into cellChordCurveAreaUpper's own arcLenUpper argument and
// quadruple the wall/seam/cap terms it feeds (an earlier version of this
// function did exactly that, measured Suspect on the calibrated reference
// wedge before this fix). The tight bracket is what keeps the per-cell share
// close to the true one.
//
// For the LineSeg arm (m=1, and every other kind circularLengthInterval
// declines) this falls back to segmentWalk.lengthUpper exactly, unaffected —
// a straight chord's own recorded length bound was never the loose one. A
// non-finite whole-length bound propagates rather than silently shrinking
// under the division.
func perCellArcUpper(seg CurveSegment, w segmentWalk, m int) float64 {
	if ns, err := normalizeSegment(seg); err == nil {
		if iv, ok := circularLengthInterval(ns); ok {
			return upRound(ratFloatUp(iv.hi) / float64(m))
		}
	}
	if isNonFinite(w.lengthUpper) {
		return math.Inf(1)
	}
	return upRound(w.lengthUpper / float64(m))
}

// chordCellDeltaUpper is docs/loft-design.md §5.2's matchedDelta row: it
// composes a chord's own departure from the curve it chords (the certified
// sagitta, that table's sectionDelta row, read per cell or build-wide) with the
// displacement of the two held stations that chord actually joins (that table's
// delta row) into the single PARAMETER-MATCHED bound every chorded leg charges.
// Both terms are listed there with the quantity each bounds and the certified
// source each is read from.
//
// The two are accumulated apart — the generator publishes them apart and the
// payload carries them apart (loftPayload.sectionDelta and loftPayload.delta),
// which is the rule §5.2's table states for them — and this helper is only ever
// a consumer's own composition. Composing them is not optional for such a
// consumer: the sagitta bounds the IDEAL chord between the two points the record
// denotes, and the chord the build DREW joins two stations each displaced by
// delta from those points, so a leg charging the sagitta alone leaves the
// computed station's own displacement uncharged.
//
// That table owns the composition's derivation and its rounding direction, and
// this helper adds no mechanism of its own to either. What the code here does
// state is that both terms are read at the same s, which is what keeps the
// published bound PARAMETER-MATCHED (loftCellStations' own doc comment).
//
// Either term underivable answers +Inf, the answer §5.2's table assigns those
// rows, and the caller refuses on it.
func chordCellDeltaUpper(sagittaUpper, deltaUpper float64) float64 {
	if isNonFinite(sagittaUpper) || isNonFinite(deltaUpper) {
		return math.Inf(1)
	}
	return absSumUpper(sagittaUpper, deltaUpper)
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
var errLoftStationDisplacementUnderivable = fmt.Errorf(
	`%w: a loft pair's generated stations have no proven displacement from the recorded curve`, ErrUnsupported,
)

// perCellTangentEnergy is bounds.go's cellChordCurveAreaAllow own
// tangentEnergyUpper obligation for ONE cell of this walk: a proven upper bound
// on the integral of |curve'(s) - chord|^2 over the cell's own shared
// parameter, or +Inf where this evaluator cannot prove one.
//
// Its two operands are BOTH read from the RECORD's own certified enclosures —
// perCellArcUpper over circularLengthInterval, and loftCertifiedChordLower over
// circularWalkEnclosures — never from the walk's own held math.Hypot radius and
// math.Atan2 angles, neither of which the walk can enclose (extrude.go's
// circularWalk). uniformSpeedTangentEnergyUpper's published energy DECREASES in
// its chord operand, so a chord read off those floats can overstate the true
// chord and understate the energy every consumer downstream spends: a held
// value wearing a proof's clothes, which circularWalkEnclosures' own doc
// comment forbids.
//
// It is dispatched on the WALK KIND rather than shared across every arm,
// because the obligation uniformSpeedTangentEnergyUpper discharges rests on the
// shared parametrization having CONSTANT SPEED — a property of the arm that
// placed the stations, not of the cell's geometry. The circular arm's
// uniform-ANGLE stations (loftCircularCellStations) are constant speed on a
// circle, which is what discharges it; a straight walk's chord IS its curve, so
// its deviation is identically zero. Any FUTURE kind — the free-form arm's own
// span-uniform native fraction above all, which is NOT constant speed — answers
// +Inf here until it carries a proof of its own, so it degrades
// cellChordCurveAreaAllow to that helper's premise-free arm rather than being
// silently handed a bound whose premise it does not meet.
func perCellTangentEnergy(seg CurveSegment, w segmentWalk, m int) float64 {
	switch w.kind {
	case walkLine:
		return 0
	case walkCircular:
		return uniformSpeedTangentEnergyUpper(perCellArcUpper(seg, w, m), loftCertifiedChordLower(seg, m))
	default:
		return math.Inf(1)
	}
}

// circularStationChain walks this segment's own uniform-angle stations of w,
// at parameter t_k = k/m — its OWN interior stations, excluding its shared end
// point (loftCellStations' own doc comment states why), so the chain holds the
// entries a loop's own count under docs/loft-design.md §7 needs from it rather
// than a count stated here.
//
// Station 0 is the WALK's own start point, never a recomputed cos/sin at th0.
// The two are not the same reading: walkOf runs pinArcWalkEnds over every arc
// walk, and arcWalkEnd restates an UNTRIMMED end as the recorded Start / End
// verbatim under a zero bound (extrude.go), so at that kind of end the walk's
// point IS a recorded coordinate while circularWalk's own cU + r·cos(th0) is a
// float that misses it. Recomputing here would displace such a station off the
// coordinate the record states — measurably, by an ulp of the coordinate — and
// would leave every reading that reads a pinned station's own published
// displacement (docs/loft-design.md §5.2's pinned station kinds) stating it of
// a vertex the build does not hold. Reading the walk keeps the pin and the station
// the same point.
//
// The chain's own terminal station is the next segment's station 0, or the
// loop's wrap back to the first segment's, so it takes the identical pin from
// that segment's own walk: a loop's segments are contiguous on the record, and
// each junction carries exactly one station.
//
// The second return is this segment's own reading of the stationRound term
// docs/loft-design.md §5.2's table lists, as an in-section-plane distance;
// that table owns the quantity it bounds, the certified source it is read from
// and its max-versus-sum rule. Only station 0 is a coordinate reading
// alone: every interior station is a math.Sincos evaluation at an angle
// composed from the walk's own held math.Atan2 / math.Hypot floats, none of
// which the walk can enclose (circularWalk's own doc comment), so the built
// point can miss the recorded curve's own point by a rounding the certified
// sagitta says nothing about. circularPointBound (extrude.go) encloses that
// recorded point from the RECORD, at the station's own EXACT rational
// parameter t_k = TStart + (k/m)·(TEnd − TStart), and radius2D turns the two
// componentwise gaps into the plane distance the caller's chord bound is
// stated in.
//
// The chain's own two pinned ends are charged from the walk's own
// startBound/endBound rather than recomputed: those are the same readings
// arcWalkEnd already decided (zero at a natural bound, circularPointBound's
// enclosure at a trimmed one), and this segment's LAST cell reaches the walk
// end, so both bounds belong to cells this chain draws.
//
// Those two readings are not the whole charge at an UNTRIMMED ArcSeg end.
// arcWalkEnd's zero states that the held station IS the recorded coordinate,
// which is a statement about the POSITION and not about the point the record
// DENOTES there: circularEndpointInterval (moments.go) takes the denoted
// curve's radius from Start alone, so at t == 1 the denoted point sits at
// Start's radius and End's own angle and misses the recorded End by the
// arc-end radial residual. arcNaturalEndRadialUpper charges exactly that
// residual, at t == 1 alone, and docs/loft-design.md §5.2 owns the term.
//
// A station the record cannot enclose answers +Inf, which the caller refuses
// on rather than publishing the certified sagitta as if it were the whole
// bound.
func circularStationChain(w segmentWalk, seg CurveSegment, m int) ([]Point2, float64) {
	pts := make([]Point2, m)
	pts[0] = Point2{U: w.startU, V: w.startV}
	delta := math.Max(walkEndPlaneDelta(w.startBound), walkEndPlaneDelta(w.endBound))
	delta = math.Max(delta, arcNaturalEndRadialUpper(seg))
	tStart, dt, ok := circularSegmentRange(seg)
	if !ok {
		return pts, math.Inf(1)
	}
	for k := 1; k < m; k++ {
		t := float64(k) / float64(m)
		theta := w.th0 + t*(w.th1-w.th0)
		sin, cos := math.Sincos(theta)
		pts[k] = Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin}

		tk := new(big.Rat).Add(tStart, new(big.Rat).Mul(big.NewRat(int64(k), int64(m)), dt))
		delta = math.Max(delta, walkEndPlaneDelta(circularPointBound(seg, tk, pts[k].U, pts[k].V)))
	}
	return pts, delta
}

// walkEndPlaneDelta reads a walk endpoint's two componentwise bounds as one
// in-section-plane distance: the two components are along the section frame's
// own orthogonal U and V, so radius2D's √2 factor over the wider of them is an
// upper bound on the displacement's own length. An underivable component
// answers +Inf, never a small number spent in its place.
func walkEndPlaneDelta(bound walkEndBound) float64 {
	if !bound.derivable() {
		return math.Inf(1)
	}
	return radius2D(math.Abs(bound.u), math.Abs(bound.v))
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
// (ratSqrtDown), which can only enlarge the quotient. ratFloatUp rounds the
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

	den := new(big.Rat).Add(floatRat(ratSqrtDown(r0)), floatRat(ratSqrtDown(r1)))
	if den.Sign() <= 0 {
		return math.Inf(1)
	}
	up := ratFloatUp(new(big.Rat).Quo(diff, den))
	if isNonFinite(up) {
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
	start := floatRat(tStart)
	if start == nil {
		return nil, nil, false
	}
	return start, exactCoordinateDelta(tEnd, tStart), true
}

// loftCertifiedSagittaUpper is docs/loft-design.md §5.2's certified per-cell
// sagitta for one side of a chorded circular pair, at a candidate station
// count m: the in-section-plane distance from one chord to the recorded curve
// piece it chords, published as ONE outward rounding of an enclosure's upper
// end.
//
// The quantity is 2·r·sin²(Δθ/4m) with r the segment's radius and Δθ the angle
// its walk sweeps. Both come from circularWalkEnclosures (moments.go), which
// states them from the RECORD — an ArcSeg's ratSqrtDown/ratSqrtUp radius and
// its atan2Interval swept angle, a CircleSeg's recorded radius and exact
// rational turn — never from the walk's held math.Hypot radius and math.Atan2
// angles, neither of which the walk can enclose (extrude.go's circularWalk).
// radSinCosSpan supplies the sine of the enclosed cell half-angle, and the
// squaring goes through intervalMul, whose four-corner upper end dominates
// max x² over the span whatever the span's sign.
//
// The derivation of the form itself belongs to §5.2's table, on its per-cell
// sagitta row, and this comment restates none of it.
//
// An enclosure the record cannot state answers +Inf — the refusal that row
// assigns this term — and the caller refuses at Table S row S14 rather than
// publishing a finite substitute or a zero.
func loftCertifiedSagittaUpper(seg CurveSegment, m int) float64 {
	if m <= 0 {
		return math.Inf(1)
	}
	radius, sweep, ok := circularWalkEnclosures(seg)
	if !ok {
		return math.Inf(1)
	}
	half := intervalScale(sweep, big.NewRat(1, 4*int64(m)))
	sin, _, ok := radSinCosSpan(half)
	if !ok {
		return math.Inf(1)
	}
	s := intervalMul(intervalScale(radius, big.NewRat(2, 1)), intervalMul(sin, sin))
	up := ratFloatUp(s.hi)
	if isNonFinite(up) {
		return math.Inf(1)
	}
	return up
}

// loftCertifiedChordLower is a PROVEN LOWER bound on ONE uniform-angle cell's
// own true chord length at a station count m — the chord between the two points
// the RECORD denotes at the cell's own two parameters, which is
// uniformSpeedTangentEnergyUpper's own chordLower obligation and must never
// overstate that chord.
//
// The quantity is 2·r·sin(Δθ/2m), r the segment's radius and Δθ the angle its
// walk sweeps: the same two enclosures loftCertifiedSagittaUpper reads, from the
// same owner (moments.go's circularWalkEnclosures), taken at the cell's own HALF
// angle rather than its quarter. Reading them here rather than the walk's held
// w.radius/w.th0/w.th1 is not a preference: those floats carry no enclosure
// (circularWalkEnclosures' own doc comment), a math.Atan2 endpoint's own error
// is amplified by 1/Δθ on a short arc, and the published energy DECREASES in
// this operand — so a float-derived "lower" bound that lands above the true
// chord understates the energy and every area allowance composed from it.
//
// The product runs through intervalMul, whose four-corner LOWER end is a bound
// on 2·r·sin over the whole enclosure and so on the true chord wherever in it
// the true radius and sweep lie. A record this bracket cannot state, or a
// bracket whose own lower end is not positive, answers 0 — a valid, if empty,
// lower bound on any chord, which costs uniformSpeedTangentEnergyUpper its
// sharpness and never its soundness.
func loftCertifiedChordLower(seg CurveSegment, m int) float64 {
	if m <= 0 {
		return 0
	}
	radius, sweep, ok := circularWalkEnclosures(seg)
	if !ok {
		return 0
	}
	half := intervalScale(sweep, big.NewRat(1, 2*int64(m)))
	sin, _, ok := radSinCosSpan(half)
	if !ok {
		return 0
	}
	c := intervalMul(intervalScale(radius, big.NewRat(2, 1)), sin)
	lo := ratFloatDown(c.lo)
	if isNonFinite(lo) || lo <= 0 {
		return 0
	}
	return lo
}

// errLoftSagittaUnderivable is docs/loft-design.md Table S row S14's refusal:
// a chorded circular pair for which the certified per-cell sagitta has no
// derivation from the record. The body exists and the chord set is buildable;
// only one of its proven displacement terms cannot be stated, which is a
// derivation gap in this evaluator's certified circular enclosures rather than
// a shape rule — so the sentinel is ErrUnsupported, and no finite value is
// published in its place.
var errLoftSagittaUnderivable = fmt.Errorf(
	`%w: a chorded circular pair's certified per-cell sagitta has no derivation from this record`, ErrUnsupported,
)
