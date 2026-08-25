package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md PR 1a: the evaluator half of Loft — the
// loftPayload, its Table P pairing, Table S gates S1-S5 and S13's
// coordinate-range gate (S9-S11 are the public
// entry point's job, docs/loft-design.md §2/§4), the flat-triangle wall
// construction (§5), the wiring of the already-landed §6 audit
// (loft_audit.go) and §8 mass kernel (loft_moments.go), and the four
// measurements. Document.Loft/LoftContext are PR 1b; nothing here is called
// from outside this file's own tests, the same shape #114 (loft_audit.go/
// loft_moments.go) already shipped.
//
// loftPayload.placed and its delta field are §12 PR 2a (Table D, D7): a
// placement re-lifts both records under the composed motion and re-runs
// §5-§8 from scratch — every record-only Table S gate (S1-S8) and S13's
// coordinate-range gate, plus the placement-only S12, while S9-S11 and S4's
// arity half judge the original
// call's own arguments and never re-run (§4) — so delta is the ONE new term
// this PR adds, composed into every vertex, edge length, face area and body
// measurement §8 already derives.

// loftPayload is the evaluator's own record of a lofted body: the two
// authenticated sections, their planes and frames, the per-loop alignment,
// the accumulated rigid placement, and the assembled triangle set the
// construction produced (docs/loft-design.md §5/§7). verts/tris/walls are the
// same globally oriented triangle set §6's audit classified and §8's
// accumulator integrated — tris[:walls] the wall triangles (side(i,j,k)), the
// rest the two caps' own triangulations — kept on the payload so a later
// Tessellate (PR 2) restates it rather than rebuilding it.
type loftPayload struct {
	profile0, profile1 ProfileRecord
	plane0, plane1     PlaneRecord
	frame0, frame1     r3.Frame
	alignment          []int
	xform              r3.Transform

	// delta is the proven displacement of every held vertex from the exact
	// point the record denotes for it (docs/loft-design.md §5, §12 PR 2a,
	// a10-plan.md Part 3 PR 6): absSumUpper(stationRound, placeAllow).
	// placeAllow is zero for an unplaced body — pl.xform == r3.Identity(), an
	// exact struct comparison, never a tolerance — and otherwise
	// bounds.go's rigidRoundAllow, read at the pre-transform lifted point's
	// own magnitude and the composed translation's magnitude. stationRound is
	// the rounding a COMPUTED circular station commits (an exact-rational
	// trig enclosure rounded once into a Point2), zero for a LineSeg pairing
	// (every station is a recorded endpoint) and positive for a curved
	// pairing whenever a cell has an interior station — so delta is zero
	// exactly when BOTH terms are, no longer merely when the body is
	// unplaced. Every measurement this payload publishes composes it.
	delta float64

	// sectionDelta is the proven upper bound on how far any BUILT CHORD point
	// of a wall cell sits from the recorded curve it chords, AS A SET — the
	// curve's own sagitta, taken as a MAXIMUM over cells rather than a sum
	// (docs/loft-design.md §5, a10-plan.md Part 3 PR 5/PR 6). It is zero for
	// a LineSeg pairing — a straight wall's own chord IS the recorded
	// segment, so there is no curve for it to depart from — and positive for
	// a same-kind circular pairing, to the sagitta its station chording
	// commits.
	//
	// It is NEVER delta and never stands in for it, the identical
	// independence prismPayload's own sectionDelta/z0Delta pair states one
	// mechanism over (extrude.go): delta bounds a HELD VERTEX's own
	// displacement from the exact point the record denotes for it, while
	// sectionDelta bounds a BUILT CHORD's own displacement, in the section
	// plane, from the curve it chords. A reading that needs both sums them
	// into its own bound; neither is ever substituted for the other.
	//
	// It is ALSO never bounds.go's cellChordCurveAreaUpper's own
	// matchedDeltaUpper obligation, a STRONGER, DIFFERENT quantity that
	// helper's own doc comment defines: sectionDelta is a SET-distance (some
	// chord point, not necessarily at the matching parameter), while
	// matchedDeltaUpper must be a PARAMETER-MATCHED bound (the SAME s under
	// the wall homotopy's own constant-arc-length parametrization). The two
	// coincide only for a LINE (trivially, both zero) or an ARC under its own
	// uniform-angle parametrization (bounds.go's own doc comment on
	// cellChordCurveAreaUpper, F1) — the coincidence loftCircularCellStations'
	// own doc comment proves.
	//
	// Every matched-delta obligation is discharged by a SEPARATE quantity,
	// sectionMatchedDelta: loftPairings accumulates it as its own MAX over
	// each cell's matchedDelta and returns it beside sectionDelta, and
	// evalLoft passes it — never this field — to newLoftMassAccumulator and
	// computeLoftChordedAllow (loft_moments.go), which is where every
	// chordedBoundaryVolumeAllow, chordedBoundaryMomentAllow and
	// chordedBoundarySeamAllow matched argument comes from;
	// cellChordCurveAreaUpper reads the per-cell matchedDelta[j] that same
	// MAX is taken over, one cell at a time. sectionMatchedDelta is a
	// PER-BUILD LOCAL of evalLoft and deliberately NOT a loftPayload field:
	// nothing this payload stores needs it, because placed re-evaluates
	// through evalLoft, which recomputes both quantities from the records,
	// and those two consumers have exactly one production call site each —
	// evalLoft's own. So there is no stored copy of the matched quantity that
	// could disagree with the records, and no caller that could reach those
	// helpers with the other quantity.
	//
	// This field's OWN remaining spends are the cap SET-distance tube
	// (sectionDisplacementArea, loft_moments.go) and Bounds.Bound. Both are
	// set-distance readings, so both correctly read sectionDelta.
	sectionDelta float64

	verts []r3.Vec
	tris  [][3]int
	walls int
}

// transform is the accumulated rigid placement.
func (pl loftPayload) transform() r3.Transform { return pl.xform }

// axialDelta reports the displacement of every held loft vertex. Its planar
// caps are built from those vertices, so a ToFace stop against either cap
// inherits this bound.
func (pl loftPayload) axialDelta() float64 { return pl.delta }

// placed re-evaluates the same two records under the composed motion
// (docs/loft-design.md §7, §12 PR 2a): it re-lifts every vertex from the
// record under the FULL composed transform rather than moving the held mesh
// incrementally, so delta does not accumulate across repeated placements —
// one rounding, not one per placement, an advantage over
// facetedPayload.placed's move-the-held-mesh path (boolean_body.go). It is a
// re-evaluation path: no moments preflight has run on either record within
// this call, so the build opens two fresh counters, one per record, exactly
// as prismPayload.placed opens its own (extrude.go). §5's whole-shell
// orientation step re-decides the sign from the placed triangle set on its
// own, so a mirror flips `reversed` with no separate winding-flip case
// needed here.
func (pl loftPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	next := pl
	next.xform = composed
	next.verts, next.tris, next.walls = nil, nil, 0
	return evalLoft(ctx, d, ref, next, newWorkBudget(ctx), newFreeformWork(), newFreeformWork())
}

// validateLoftRecords applies docs/loft-design.md Table S rows S1, S2, S4, S3
// and S5, in §4's stated gate order, from the two authenticated records
// alone — no triangle is built. It returns the normalized per-loop alignment
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
// S3's admission test is now a SAME-KIND test (a10-plan.md Part 3 PR 6): a
// pair is admitted when both walks share the identical admitted kind —
// walkLine or walkCircular — never merely because one side is a LineSeg.
// Mixed-kind and free-form pairs keep today's refusal and today's sentinel
// (loftSameKindGate). Testing only after BOTH sides are resolved (rather
// than each side against its own kind test, as the LineSeg-only form did) is
// unavoidable once the admitted set has two kinds, and it does not relax
// PRECEDENCE: the first (i, j) whose pair fails is still the first refusal
// reported, in the same walk order as before.
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

			if err := loftSameKindGate(w0, w1, loops0[i].Segments[j], loops1[i].Segments[k], i, j, k); err != nil {
				return nil, nil, nil, err
			}
			walks0[i][j] = w0
			walks1[i][k] = w1
		}
	}

	if loftPlanesCoincide(pl0, pl1) {
		return nil, nil, nil, fmt.Errorf(`%w: the two profiles lie in the same geometric plane; the loft has zero volume by construction`, ErrDegenerate)
	}

	return offsets, walks0, walks1, nil
}

// loftSameKindGate is docs/loft-design.md Table S row S3 and Table P row P5
// in their arc form (a10-plan.md Part 3 PR 6): a pairing is admitted only
// when both walks share the SAME admitted kind — walkLine or walkCircular —
// so a mixed-kind or free-form pair still refuses under today's sentinel.
//
// Beside S3, a CHEAP STRUCTURAL gate rather than an expensive proof: two
// paired circular segments whose own EFFECTIVE walk directions disagree walk
// in opposite directions and would twist the station correspondence into a
// self-crossing wall. §6's build-time audit would eventually catch the
// resulting crossing as S7 (ErrDegenerate, loft_audit.go), but this test
// runs before a single station or triangle is built, and its own sentinel
// (ErrUnsupported) is what keeps the two refusals distinguishable — a
// caller can tell "this evaluator does not admit the pairing" from "the
// pairing self-crosses" without inspecting the message.
//
// An ArcSeg's sweep is NOT structurally fixed CCW the way an earlier version
// of this comment claimed: walkOf's own ArcSeg arm (extrude.go) reads
// th0 = a0 + TStart*sweep, th1 = a0 + TEnd*sweep with sweep always forced
// positive, so the walk's own angle is monotonic in t and its EFFECTIVE
// direction is CCW exactly when TEnd > TStart — the identical formula
// validateSegmentWinding already enforces a CircleSeg's own CCW field must
// equal (record.go). loftCircularSegmentCCW reads that one shared formula
// for either kind, so a CircleSeg{CCW: false} paired with an ArcSeg whose
// own TStart < TEnd — genuinely opposite directions — is caught here rather
// than silently admitted and left to surface as S7's own crossing refusal
// three build phases later.
func loftSameKindGate(w0, w1 segmentWalk, seg0, seg1 CurveSegment, loop, j, k int) error {
	if w0.kind != w1.kind || (w0.kind != walkLine && w0.kind != walkCircular) {
		return fmt.Errorf(`%w: loop %d segment %d of the first profile and segment %d of the second are not the same admitted kind; this evaluator pairs same-kind LineSeg or circular segments only`,
			ErrUnsupported, loop, j, k)
	}
	if w0.kind != walkCircular {
		return nil
	}
	ccw0, ok0 := loftCircularSegmentCCW(seg0)
	ccw1, ok1 := loftCircularSegmentCCW(seg1)
	if ok0 && ok1 && ccw0 != ccw1 {
		return fmt.Errorf(`%w: loop %d's paired circular segment at segment %d/%d walk in opposite directions; this evaluator refuses rather than twist the correspondence`,
			ErrUnsupported, loop, j, k)
	}
	return nil
}

// loftCircularSegmentCCW reads a circular segment's own EFFECTIVE walk
// direction structurally, from ONE shared formula rather than trusting a
// per-kind field (loftSameKindGate's own doc comment): a CircleSeg's CCW
// flag is required to already equal TStart < TEnd (record.go's
// validateSegmentWinding), and an ArcSeg's own walkOf arm forces its sweep
// positive (extrude.go), so its angle is a STRICTLY INCREASING function of
// t and its own walk visits increasing angle exactly when TEnd > TStart —
// the identical formula. The false return is defensive, unreached from any
// real build today since loftSameKindGate has already proven w.kind ==
// walkCircular for both sides, which only CircleSeg and ArcSeg produce.
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
// that cell's own PARAMETER-MATCHED bound on |curve(s) - chord(s)| at the
// same s — bounds.go's cellChordCurveAreaUpper own matchedDeltaUpper
// obligation, F1's rule — NEVER the SET-distance sagitta sectionDelta names.
// A LineSeg cell's own chord IS the curve it denotes, so its matchedDelta is
// exactly 0; a circular cell's own sagitta discharges the obligation exactly
// (loftCircularCellStations' own doc comment), so its matchedDelta equals its
// sagitta; a free-form cell's matchedDelta is spanMatchedDeltaUpper's own
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
// the default 1e-3 relative tolerance within the plan's 2-second
// per-fixture wall-clock budget (Q3): m=64 stations on the arc wedge (F=134
// wall+cap triangles), with Volume the binding reading at a measured 2.39x
// margin. A finer grid point (m=128) clears the plan's separate 4x-margin
// target but costs roughly 4.3s, more than double the wall-clock budget, so
// the plan's own named fallback governs (a10-plan.md Q2's "Fallback if
// calibration does not close"): ship the coarser, in-budget value and accept
// that an extreme aspect ratio can read Suspect at a tight tolerance — a
// correct non-silent outcome, not a wrong answer.
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
// states.
//
// sagittaUpper is this cell's own SAGITTA: the proven upper bound on how far
// a BUILT CHORD point sits from the curve it chords, AS A SET — the
// SET-distance reading loftPairings maxes into sectionDelta (loftPayload's
// own doc comment), never bounds.go's cellChordCurveAreaUpper own
// PARAMETER-MATCHED matchedDeltaUpper obligation. The two arms shipping
// today happen to discharge that matched obligation with this same number:
// the LineSeg arm's chord IS the recorded segment, so its bound is exactly
// zero, and the circular arm's own doc comment states why its uniform-angle
// sagitta discharges the matched obligation exactly rather than merely
// bounding it. That coincidence is a property of those two arms, not of this
// return — which is why the matched reading is returned separately as
// matchedDelta below, and why matchedDelta, never this value, is what the
// matched helpers read.
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
// sectionDelta. The LineSeg arm's stations are recorded endpoints, never
// computed, so its own contribution is exactly zero; the circular arm's own
// doc comment states the mechanism.
//
// matchedDelta is bounds.go's cellChordCurveAreaUpper own matchedDeltaUpper
// obligation (F1's rule), ONE ENTRY PER CELL — never a single per-segment
// scalar, since a bisected free-form arm can settle cells of that one paired
// segment at different depths and so at different matched-delta readings.
// len(matchedDelta) always equals len(stations0), the per-cell count every
// arm below publishes. It is NEVER sagittaUpper: the LineSeg arm's chord IS
// the curve, so every entry is exactly 0; the circular arm's own sagitta
// discharges the obligation exactly, so every entry equals the segment's own
// sagittaUpper (loftCircularCellStations' own doc comment); a future
// free-form arm's own per-cell reading can vary within these two extremes
// cell to cell.
func loftCellStations(seg0, seg1 CurveSegment, w0, w1 segmentWalk, target float64, work0, work1 *freeformWork) ([]Point2, []Point2, float64, []float64, float64, error) { //nolint:unparam // work0/work1 are part of the fixed kind-switch interface every future arm shares; the ARC and LineSeg arms below are the two that do not need them yet.
	switch {
	case w0.kind == walkLine && w1.kind == walkLine:
		stations0, stations1, sagitta, matchedDelta, err := loftLineCellStations(w0, w1)
		return stations0, stations1, sagitta, matchedDelta, 0, err
	case w0.kind == walkCircular && w1.kind == walkCircular:
		return loftCircularCellStations(seg0, seg1, w0, w1, target)
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
// for it to depart from. m is fixed at 1, so this arm's output is
// bit-identical to every LineSeg pairing this evaluator built before the
// station generator existed.
func loftLineCellStations(w0, w1 segmentWalk) ([]Point2, []Point2, float64, []float64, error) { //nolint:unparam // the error return matches loftCircularCellStations' own arm shape; a straight chord never fails to state its own recorded endpoint.
	return []Point2{{U: w0.startU, V: w0.startV}}, []Point2{{U: w1.startU, V: w1.startV}}, 0, []float64{0}, nil
}

// loftCircularCellStations is the circular arm: tessellate.go's chordCount
// picks each side's OWN station count against target independently, the
// shared count is m = max(m0, m1) (a10-plan.md Q2's shared-station rule),
// and each side's stations are then walked at that SHARED m — never at its
// own smaller count — so both sides carry the same station count by
// construction, the correspondence a loft wall needs.
//
// Because chordSagitta strictly decreases in n, m >= m_side already proves
// each side's own achieved sagitta at the shared m is at or below the SAME
// target its own independent count met; only the side whose own count was
// smaller than m needs its sagitta recomputed at the shared m; the side that
// already IS m keeps chordCount's own returned value unchanged.
//
// sagittaUpper is the MAX of the two sides' achieved sagittas — this cell's
// own single bound, covering whichever side departs further from its chord.
// It is also this arm's discharge of loftCellStations' own PARAMETER-MATCHED
// obligation: stations are placed at uniform PARAMETER t_k = TStart +
// (k/m)*(TEnd-TStart), which for a circular walk is uniform in ANGLE because
// t is the walk's own sweep fraction — and under that uniform-angle
// parametrization, sup_s |arc(s) - chord(s)| over one chord EQUALS the
// sagitta exactly (an audit verified this across sweeps from 0.5 to 359.5
// degrees, the maximum departure always landing at s = 1/2 of the chord),
// never merely bounding it. A SET distance from an arbitrary chord point to
// the curve would not carry that guarantee; the uniform-angle station rule
// is what buys it.
//
// matchedDelta is loftCellStations' own per-cell obligation: this arm's
// sagitta discharges it EXACTLY (the paragraph above), and every cell of one
// uniformly-stepped circular segment shares the same true angular width, so
// the same value — math.Max(s0, s1) — is the correct, exact per-cell reading
// for all m cells, not merely a safe upper bound repeated m times.
func loftCircularCellStations(seg0, seg1 CurveSegment, w0, w1 segmentWalk, target float64) ([]Point2, []Point2, float64, []float64, float64, error) {
	// S15: chordCount itself already refuses (errTooManyChords, reused per
	// spline design Table R row R8) a side whose target cannot be met inside
	// maxChordsPerWalk — this arm mints no separate cap of its own, and
	// propagates that refusal verbatim from whichever side hits it first.
	m0, s0, err := chordCount(w0, target)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	m1, s1, err := chordCount(w1, target)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	m := max(m0, m1)
	if m > m0 {
		s0 = chordSagitta(w0.radius, math.Abs(w0.th1-w0.th0), m)
	}
	if m > m1 {
		s1 = chordSagitta(w1.radius, math.Abs(w1.th1-w1.th0), m)
	}

	stations0 := circularStationChain(w0, m)
	stations1 := circularStationChain(w1, m)

	// S16, defensive: a chord cell whose two stations coincide on exactly
	// ONE of the two sections has no case in the uniform two-faces-per-cell
	// wall topology assembleLoft builds. A same-kind pair with a genuinely
	// positive sweep on both sides never reaches this — distinct k give
	// distinct angles give distinct (cos, sin) pairs — so it is reachable
	// only from a degenerate walk (a zero radius) that a real sketch
	// authentication does not produce (spline_fit.go's own dedup plays the
	// analogous role for the free-form arm's own R3 risk). It still ships:
	// deleting a gate for want of a caller-reachable fixture is not this
	// evaluator's call to make.
	for k := range m - 1 {
		eq0 := stations0[k] == stations0[k+1]
		eq1 := stations1[k] == stations1[k+1]
		if eq0 != eq1 {
			return nil, nil, 0, nil, 0, fmt.Errorf(`%w: chord cell %d of this paired segment collapses to one point on only one of the two sections`, ErrUnsupported, k)
		}
	}

	stationRound, err := loftCircularStationRound(seg0, seg1, stations0, stations1, m)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}

	sagitta := math.Max(s0, s1)
	matchedDelta := make([]float64, m)
	for i := range matchedDelta {
		matchedDelta[i] = sagitta
	}
	return stations0, stations1, sagitta, matchedDelta, stationRound, nil
}

// circularSegRange reads a recorded circular segment's own TStart/TEnd —
// the SAME normalized parameter domain circularWalkEndBound's own t argument
// reads (extrude.go's walkOf: a CircleSeg's t is a turn fraction, an ArcSeg's
// is a fraction of its own Start-to-End sweep; either way seg.TStart/seg.TEnd
// name it directly). Only CircleSeg and ArcSeg carry it in this evaluator's
// admitted circular kind; the false return is defensive, unreached from any
// real build today since loftSameKindGate has already proven w.kind ==
// walkCircular for both sides.
func circularSegRange(seg CurveSegment) (float64, float64, bool) {
	switch seg := seg.(type) {
	case CircleSeg:
		return seg.TStart, seg.TEnd, true
	case ArcSeg:
		return seg.TStart, seg.TEnd, true
	default:
		return 0, 0, false
	}
}

// loftCircularStationRound is docs/loft-design.md Table S row S14
// (a10-plan.md Part 3 PR 6): the proven rounding a COMPUTED chord station
// commits — an exact-rational trig enclosure rounded once into a Point2 —
// taken as a MAX over this cell's own m stations on BOTH sides, through
// extrude.go's circularWalkEndBound and bounds.go's walkEndBoundAllow. It is
// a component of delta (a HELD VERTEX's own displacement), never
// sectionDelta (a BUILT CHORD's own departure from the curve it chords):
// loftPayload's own doc comment states why the two are never interchanged.
//
// t_k = seg.TStart + (k/m)*(seg.TEnd-seg.TStart) is the identical parameter
// circularStationChain walks to place stations0[k]/stations1[k] (w.th0 is a
// float restatement of seg.TStart under the same affine map, so the two
// agree exactly), which is what lets circularWalkEndBound's own enclosure
// speak for the SAME computed point this function's caller already holds.
//
// A station whose rounding this evaluator cannot enclose — circularWalkEndBound
// answers +Inf for either side, at any k — refuses ErrUnsupported rather than
// publish a silent zero: the row's own text is "no derivation from this
// record", not "the derivation is exactly zero".
func loftCircularStationRound(seg0, seg1 CurveSegment, stations0, stations1 []Point2, m int) (float64, error) {
	t0Start, t0End, ok0 := circularSegRange(seg0)
	t1Start, t1End, ok1 := circularSegRange(seg1)
	if !ok0 || !ok1 {
		return 0, fmt.Errorf(`%w: this evaluator has no station-rounding rule for this circular segment's own representation`, ErrUnsupported)
	}
	round := 0.0
	for k := range m {
		frac := float64(k) / float64(m)
		t0 := t0Start + frac*(t0End-t0Start)
		t1 := t1Start + frac*(t1End-t1Start)
		b0 := circularWalkEndBound(seg0, t0, stations0[k].U, stations0[k].V)
		b1 := circularWalkEndBound(seg1, t1, stations1[k].U, stations1[k].V)
		round = math.Max(round, walkEndBoundAllow(b0))
		round = math.Max(round, walkEndBoundAllow(b1))
	}
	if isNonFinite(round) {
		return 0, fmt.Errorf(`%w: this evaluator cannot derive a proven rounding bound for a computed chord station on this circular pairing`, ErrUnsupported)
	}
	return round, nil
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

// circularStationChain walks m uniform-angle stations of w, at parameter
// t_k = k/m for k = 0..m-1 — this segment's OWN interior stations, excluding
// its shared end point (loftCellStations' own doc comment states why).
func circularStationChain(w segmentWalk, m int) []Point2 {
	pts := make([]Point2, m)
	for k := range m {
		t := float64(k) / float64(m)
		theta := w.th0 + t*(w.th1-w.th0)
		sin, cos := math.Sincos(theta)
		pts[k] = Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin}
	}
	return pts
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
// is the MAX of every cell's own SAGITTA (a SET-distance) across the whole
// build, never a sum: a boundary point lies in exactly one cell, so only the
// widest cell's own departure bounds the whole section. sectionMatchedDelta
// is the analogous MAX of every cell's own matchedDelta (a PARAMETER-MATCHED
// bound, F1's rule) — a DIFFERENT quantity, never interchangeable with
// sectionDelta, and every caller composing bounds.go's
// chordedBoundaryVolumeAllow/chordedBoundaryMomentAllow/
// chordedBoundarySeamAllow (each of whose own doc comments name a
// parameter-matched matchedDelta obligation, never "the sagitta alone")
// reads sectionMatchedDelta, never sectionDelta. The two coincide bit-for-bit
// on a circular-only build (every circular cell's own matchedDelta equals
// its own sagitta exactly, loftCircularCellStations' own doc comment) and on
// a LineSeg-only build (both exactly 0), which is why every existing arc
// fixture stays bit-identical under this split. stationRound is the
// analogous MAX of every cell's own loftCircularStationRound (Table S row
// S14, delta's own component, never sectionDelta's or
// sectionMatchedDelta's).
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
		var matchedDelta []float64
		for j := range n {
			w0 := walks0[i][j]
			k := (j + off) % n
			w1 := walks1[i][k]
			seg0 := loops0[i].Segments[j]
			seg1 := loops1[i].Segments[k]
			stations0, stations1, sagitta, cellMatchedDelta, round, err := loftCellStations(seg0, seg1, w0, w1, target, work0, work1)
			if err != nil {
				return nil, 0, 0, 0, err
			}
			m := len(stations0)
			cellArcV := perCellArcUpper(seg0, w0, m)
			cellArcW := perCellArcUpper(seg1, w1, m)
			for range m {
				arcUpperV = append(arcUpperV, cellArcV)
				arcUpperW = append(arcUpperW, cellArcW)
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
		pairs[i] = loftLoopPair{v: v, w: w, arcUpperV: arcUpperV, arcUpperW: arcUpperW, matchedDelta: matchedDelta}
	}
	return pairs, sectionDelta, sectionMatchedDelta, stationRound, nil
}

// loftAssembly is the built triangle set plus the index bookkeeping the
// topology needs.
type loftAssembly struct {
	// verts is the shared vertex table; tris is the complete, globally
	// oriented triangle set. tris[:walls] are the wall triangles (Table B's
	// side(i,j,k)); tris[walls:walls+capStartCount] are capStart's own
	// triangles; the rest are capEnd's.
	verts         []r3.Vec
	tris          [][3]int
	walls         int
	capStartCount int
	// reversed records whether §5's whole-shell orientation step flipped
	// every triangle's winding. buildLoftTopology fixes each face's directed
	// boundary from the LOCAL (pre-flip) index convention, so it must reverse
	// every walk it emits by exactly this flag or publish loops that run the
	// material on the wrong side of their own face normal.
	reversed bool
	// cell/side parallel tris[:walls]: cell[k] is {loop index i, cell index
	// j}, side[k] is 0 for lower_j and 1 for upper_j.
	cell [][2]int
	side []uint8
	// vIdx/wIdx are, per loop, the vertex-table index of V[i][j] and W[i][j].
	vIdx, wIdx [][]int
	// pts0/pts1 and loopIdx0/loopIdx1 are the plane-local (U, V) points and
	// per-loop index arrays this construction ACTUALLY triangulated each cap
	// from (§5's cap seeding) — the same arrays capPolygonAreaRat sums, so
	// the published cap area can never disagree with the built cap
	// triangles (docs/loft-design.md §8).
	pts0, pts1         []Point2
	loopIdx0, loopIdx1 [][]int
	// delta is the proven displacement every held vertex carries from the
	// exact placed image of the recorded sections (docs/loft-design.md §5,
	// §12 PR 2a) — absSumUpper(stationRound, placeAllow): zero exactly when
	// xform is r3.Identity() AND every station is a recorded endpoint (a10-
	// plan.md Part 3 PR 6), never zero merely because the body is unplaced,
	// since a curved pair with interior COMPUTED stations commits its own
	// rounding whether or not the body is later placed.
	delta float64
}

// assembleLoft lifts every recorded point once, emits the 2*sum(n_i) wall
// triangles in Table B's order and winding, triangulates both caps through
// triangulate.go's existing polygon-with-holes triangulator with capStart's
// triples reversed and capEnd's retained (§5's cap seeding), and orients the
// complete shell once from the signed tetrahedron sum anchored at the placed
// p0 origin (§5's whole-shell rule). It also owns Table S row S13: every
// placed coordinate it emits, the anchor among them, is proven finite before
// any of them is lifted into an exact rational.
//
// stationRound is loftPairings' own accumulated Table S row S14 term
// (a10-plan.md Part 3 PR 6): the proven rounding every COMPUTED circular
// station commits, composed into delta beside the placement's own
// rigidRoundAllow term.
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform, stationRound float64) (loftAssembly, error) {
	// S13, decided before the first coordinate is lifted into an exact
	// rational: the orientation anchor is the first point loftOrientationSign
	// hands to xptOf, so its own finiteness is the gate's first question.
	anchor := xform.Apply(plane0.Origin)
	if !finiteVec(anchor) {
		return loftAssembly{}, errLoftPointUnrepresentable("placed plane origin")
	}

	vIdx := make([][]int, len(pairs))
	wIdx := make([][]int, len(pairs))
	var verts []r3.Vec
	// maxInputAbs tracks the largest |coordinate| over the frame-lifted,
	// PRE-transform points — the magnitude bounds.go's rigidRoundAllow reads
	// the rounding at, never the placed result's (docs/loft-design.md §5,
	// §12 PR 2a).
	maxInputAbs := 0.0
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return loftAssembly{}, err
		}
		vIdx[i] = make([]int, len(p.v))
		for j, pt := range p.v {
			vIdx[i][j] = len(verts)
			lifted := f0.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, vecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !finiteVec(placed) {
				return loftAssembly{}, errLoftPointUnrepresentable(fmt.Sprintf("placed vertex %d of loop %d on the first profile", j, i))
			}
			verts = append(verts, placed)
		}
		wIdx[i] = make([]int, len(p.w))
		for j, pt := range p.w {
			wIdx[i][j] = len(verts)
			lifted := f1.ToWorldUV(pt.U, pt.V)
			maxInputAbs = max(maxInputAbs, vecMaxAbs(lifted))
			placed := xform.Apply(lifted)
			if !finiteVec(placed) {
				return loftAssembly{}, errLoftPointUnrepresentable(fmt.Sprintf("placed vertex %d of loop %d on the second profile", j, i))
			}
			verts = append(verts, placed)
		}
	}

	var tris [][3]int
	var cell [][2]int
	var side []uint8
	for i, p := range pairs {
		if err := ctx.Err(); err != nil {
			return loftAssembly{}, err
		}
		n := len(p.v)
		for j := range n {
			jn := (j + 1) % n
			vj, vjn := vIdx[i][j], vIdx[i][jn]
			wj, wjn := wIdx[i][j], wIdx[i][jn]
			tris = append(tris, [3]int{vj, vjn, wjn})
			cell = append(cell, [2]int{i, j})
			side = append(side, 0)
			tris = append(tris, [3]int{vj, wjn, wj})
			cell = append(cell, [2]int{i, j})
			side = append(side, 1)
		}
	}
	walls := len(tris)

	// Both caps' own triangulation, over each profile's own (u, v) points
	// and loop index arrays — a fresh index space, mapped back to the shared
	// vertex table as each triangle comes back.
	var pts0, pts1 []Point2
	var loopIdx0, loopIdx1 [][]int
	var pts0ToV, pts1ToV []int
	for i, p := range pairs {
		idx0 := make([]int, len(p.v))
		for j, pt := range p.v {
			idx0[j] = len(pts0)
			pts0 = append(pts0, pt)
			pts0ToV = append(pts0ToV, vIdx[i][j])
		}
		loopIdx0 = append(loopIdx0, idx0)

		idx1 := make([]int, len(p.w))
		for j, pt := range p.w {
			idx1[j] = len(pts1)
			pts1 = append(pts1, pt)
			pts1ToV = append(pts1ToV, wIdx[i][j])
		}
		loopIdx1 = append(loopIdx1, idx1)
	}

	tris0, err := triangulate2DContext(ctx, pts0, loopIdx0)
	if err != nil {
		return loftAssembly{}, wrapLoftTriangulationError(err)
	}
	tris1, err := triangulate2DContext(ctx, pts1, loopIdx1)
	if err != nil {
		return loftAssembly{}, wrapLoftTriangulationError(err)
	}

	// capStart reverses each p0 triple (swap 2nd and 3rd); capEnd retains
	// p1's own triples (§5's cap seeding).
	for _, t := range tris0 {
		tris = append(tris, [3]int{pts0ToV[t[0]], pts0ToV[t[2]], pts0ToV[t[1]]})
	}
	capStartCount := len(tris0)
	for _, t := range tris1 {
		tris = append(tris, [3]int{pts1ToV[t[0]], pts1ToV[t[1]], pts1ToV[t[2]]})
	}

	reversed := loftOrientationSign(verts, tris, anchor) < 0
	if reversed {
		for i, t := range tris {
			tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}

	// placeAllow is zero exactly when xform is the identity transform — an
	// exact struct comparison, never a tolerance. This fast path is
	// REQUIRED: without it, every directly-built (unplaced) LineSeg-only
	// loft would lose the Exact readings §8/§12 PR 1 publishes
	// (docs/loft-design.md §5, §12 PR 2a). delta = absSumUpper(stationRound,
	// placeAllow) (a10-plan.md Part 3 PR 6) is NO LONGER zero exactly when
	// xform is the identity: a curved pair with interior computed stations
	// carries a positive stationRound whether or not the body is placed, so
	// the fast path this comment used to state is now placeAllow's own,
	// while stationRound is absSumUpper's other, independent leg —
	// absSumUpper(0, 0) is exactly 0.0 (upRound never nudges a non-positive
	// value), which is what keeps an unplaced LineSeg-only loft's delta bit-
	// identical to before.
	placeAllow := 0.0
	if xform != r3.Identity() {
		placeAllow = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}
	delta := absSumUpper(stationRound, placeAllow)

	return loftAssembly{
		verts: verts, tris: tris, walls: walls, capStartCount: capStartCount,
		reversed: reversed, cell: cell, side: side, vIdx: vIdx, wIdx: wIdx,
		pts0: pts0, pts1: pts1, loopIdx0: loopIdx0, loopIdx1: loopIdx1,
		delta: delta,
	}, nil
}

// errLoftPointUnrepresentable is docs/loft-design.md Table S row S13: a
// coordinate this build emits — a recorded section point lifted through its
// own frame and carried by the composed placement, or the orientation anchor
// — runs past the representable float64 range.
//
// The sentinel is ErrUnsupported and never ErrNotFinite. Every INPUT is
// finite: both records' coordinates cleared the seam gates, the plane origins
// are recorded floats, and r3 validates a Transform's own composed
// translation before it ever reaches this evaluator. What runs off float64 is
// decad's OWN evaluation of the lift, and the body EXISTS — it is the rigid
// image of a body this evaluator already built — so modify §1's existence
// test reads "a body this evaluator cannot build". spline_length.go's R15 and
// spline_fit.go's R16 draw the identical line for a finite input whose
// derived magnitude runs off float64; errors.go scopes ErrNotFinite to a
// non-finite PARAMETER or a derived non-finite MEASUREMENT, and
// validateLoftBodyMeasurements already owns that second case.
//
// The gate runs BEFORE the first exact-rational lift, never after it:
// loftOrientationSign lifts the anchor and every vertex through xptOf, whose
// mustRatOf PANICS on a non-finite float, so a check placed any later is a
// panic out of a public method rather than a returned error.
func errLoftPointUnrepresentable(what string) error {
	return fmt.Errorf(`%w: the loft's %s runs past the representable float64 range`, ErrUnsupported, what)
}

// wrapLoftTriangulationError re-sentinels triangulate.go's cap refusal as
// ErrUnsupported (design O8): the caller's two profiles are each individually
// valid per sketch (S9 authenticated them at the original Document.Loft
// call, before any record reached evalLoft; a placement rebuilds from those
// same authenticated records and re-runs no seam gate, §4),
// so a triangulation refusal here is this evaluator's own triangulator
// failing to state the body, never a claim that no such body exists — modify
// §1's existence test applied verbatim. Cancellation is never relabeled.
func wrapLoftTriangulationError(err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%w: the loft cap triangulator could not state this profile: %s`, ErrUnsupported, err)
}

// loftOrientationSign is the sign of the signed tetrahedron sum §8 defines,
// over the complete triangle set anchored at anchor — the same identity §5's
// whole-shell orientation rule reads, computed once directly over exact
// rationals rather than through the full loftMassAccumulator (which also
// folds in the area/bounds bookkeeping this sign check does not need).
func loftOrientationSign(verts []r3.Vec, tris [][3]int, anchor r3.Vec) int {
	xa := xptOf(anchor)
	sum := new(big.Rat)
	for _, t := range tris {
		a := xsub(xptOf(verts[t[0]]), xa)
		b := xsub(xptOf(verts[t[1]]), xa)
		c := xsub(xptOf(verts[t[2]]), xa)
		sum.Add(sum, xdotRat(a, xcross(b, c)))
	}
	return sum.Sign()
}

// loftVertex builds a vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so an unplaced vertex (delta 0) carries the
// same zero-bound standing; a placed vertex carries the payload's own
// delta (§12 PR 2a).
func loftVertex(p r3.Vec, delta float64) *Vertex {
	return &Vertex{position: p, bound: units.Millimeters(delta)}
}

// loftEdgeLength is the proven bound on a straight loft edge's held length:
// the square root's own committed error against the exact rational squared
// length (capblend_contour.go's straightEdgeBound/ratSquaredDistance3), no
// new mechanism for an unplaced edge. A placed edge (delta > 0, §12 PR 2a)
// composes that with bounds.go's chainLengthBound(1, delta, held) — both
// endpoints displaced by delta is exactly that helper's own one-chord case —
// through absSumUpper.
func loftEdgeLength(a, b r3.Vec, delta float64) (float64, float64) {
	held := a.Sub(b).Len()
	sq := ratSquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	bound := straightEdgeBound(held, sq)
	if delta > 0 {
		bound = absSumUpper(bound, chainLengthBound(1, delta, held))
	}
	return held, bound
}

// loftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func loftEdge(vertexObjs []*Vertex, positions []r3.Vec, a, b int, convex bool, delta float64) *Edge {
	held, bound := loftEdgeLength(positions[a], positions[b], delta)
	return &Edge{curve: Line3{}, start: vertexObjs[a], end: vertexObjs[b], convex: convex, length: held, lengthBound: bound}
}

// junctionApex returns tri's one vertex index that is not in the shared pair
// (a, b) — the OTHER incident triangle's own apex, §5's D.
func junctionApex(tri [3]int, a, b int) int {
	for _, v := range tri {
		if v != a && v != b {
			return v
		}
	}
	return tri[0]
}

// junctionConvex decides a rung or diagonal edge's convexity: orientSign(A,
// B, C, D) < 0, where (A, B, C) is primary's own outward-wound vertex order
// and D is other's apex — design O3, pinned against the box fixture: a
// standard box's vertical edge (a rung) is a genuine convex corner, and this
// is the sign that reads it as one. A zero result is a decided non-convex
// (flat) edge: docs/loft-design.md §5's rule for a flat rung or diagonal.
func junctionConvex(verts []r3.Vec, primary, other [3]int, a, b int) bool {
	apex := junctionApex(other, a, b)
	return orientSign(verts[primary[0]], verts[primary[1]], verts[primary[2]], verts[apex]) < 0
}

// planeFromTriangle builds a face's Plane surface directly from one of its
// own (already outward-oriented) triangles: origin at its first vertex, U and
// V its two edge vectors. r3.NewFrame orthonormalizes them (Gram-Schmidt in
// effect), and the resulting normal U×V is (B-A)x(C-A) up to positive
// scaling — the outward normal of an outward-wound triangle, so the face's
// `reversed` flag stays false (§5's wall-face row: every wall face is a Plane
// wound outward). The Frame is the exact answer for the three vertices handed
// to it whatever their own standing — §5's surface-parameter carve-out — while
// the face's own area and its vertices' positions carry the payload's delta.
func planeFromTriangle(verts []r3.Vec, tri [3]int) (Plane, error) {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	f, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
	if err != nil {
		return Plane{}, fmt.Errorf(`%w: a loft triangle has no plane: %s`, ErrDegenerate, err)
	}
	return Plane{Frame: f}, nil
}

// buildLoftWallFace builds one wall triangle's Face (§7's lower/upper wall
// triangle row): its own Plane, its own proven area bracket
// (loft_moments.go's wallTriangleArea, the identical bracket the mass
// accumulator sums), and its side(i,j,k) role. A placed triangle (delta > 0,
// §12 PR 2a) widens that bracket by bounds.go's perturbedTriangleAreaAllow,
// the same per-triangle correction the mass accumulator sums into Area's own
// bound.
func buildLoftWallFace(body *Body, ref StepRef, verts []r3.Vec, tri [3]int, i, j, side int, delta float64) (*Face, error) {
	surf, err := planeFromTriangle(verts, tri)
	if err != nil {
		return nil, err
	}
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	u := xsub(xptOf(b), xptOf(a))
	v := xsub(xptOf(c), xptOf(a))
	lo, hi := wallTriangleArea(u, v)
	areaBound := upRound(hi - lo)
	if delta > 0 {
		areaBound = absSumUpper(areaBound, perturbedTriangleAreaAllow(a, b, c, delta))
	}
	return &Face{
		surface:   surf,
		origins:   []FeatureRef{{Step: ref, Role: fmt.Sprintf("side(%d,%d,%d)", i, j, side)}},
		body:      body,
		area:      lo,
		areaBound: areaBound,
	}, nil
}

// loftLoopCoedges carries §5's whole-shell reversal into one face's directed
// boundary. Every walk buildLoftTopology emits is written from the LOCAL
// vertex order of §5's construction table, which is the order the triangles
// had BEFORE the whole-shell step; a face's Plane, by contrast, is rebuilt
// from its own
// already-flipped triple (planeFromTriangle), so on a reversed shell the two
// disagree and the published boundary — Loop.CoEdges, CoEdge.Start/End/
// IsForward — walks the material on the RIGHT of the face's own outward
// normal, the opposite of decad's material-on-the-left convention.
//
// Reversing a walk is reversing its coedge order and negating each use's
// sense; nothing but the direction changes. The edge identities and their
// count are untouched, so every edge still bounds exactly the same two faces
// and Loop.Edges' undirected view is merely re-ordered.
func loftLoopCoedges(co []coedge, reversed bool) []coedge {
	if !reversed {
		return co
	}
	out := make([]coedge, len(co))
	for i, ce := range co {
		out[len(co)-1-i] = coedge{edge: ce.edge, forward: !ce.forward}
	}
	return out
}

// buildLoftTopology builds the B-rep topology from the assembled, globally
// oriented triangle set (docs/loft-design.md §5/§7): real Vertex/Edge/Loop/
// Face objects sharing indices with the assembly's own vertex table. Every
// edge bounds exactly two faces by construction (§5's four edge families:
// bottom rim, top rim, diagonal, rung), and every cap-boundary edge opposes
// its incident wall edge, the standard two-manifold convention.
//
// Every loop this builds is stated in §5's LOCAL vertex order and then passed
// through loftLoopCoedges, which is what carries the assembly's own
// whole-shell reversal into the directed boundary each face publishes. A walk
// emitted without it agrees with its face's Plane on one axial spelling of a
// section pair and opposes it on the mirror.
func buildLoftTopology(ctx context.Context, body *Body, ref StepRef, a loftAssembly, cap0Rat, cap1Rat *big.Rat) (*Face, *Face, []*Face, error) {
	vertexObjs := make([]*Vertex, len(a.verts))
	for i, p := range a.verts {
		vertexObjs[i] = loftVertex(p, a.delta)
	}

	loopCount := len(a.vIdx)
	lowerTri := make([][][3]int, loopCount)
	upperTri := make([][][3]int, loopCount)
	for i := range a.vIdx {
		lowerTri[i] = make([][3]int, len(a.vIdx[i]))
		upperTri[i] = make([][3]int, len(a.vIdx[i]))
	}
	for k := range a.walls {
		i, j := a.cell[k][0], a.cell[k][1]
		if a.side[k] == 0 {
			lowerTri[i][j] = a.tris[k]
		} else {
			upperTri[i][j] = a.tris[k]
		}
	}

	var walls []*Face
	var capStartLoops, capEndLoops []*Loop
	for i := range loopCount {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		n := len(a.vIdx[i])
		isOuter := i == 0
		vIdx, wIdx := a.vIdx[i], a.wIdx[i]

		rimBottom := make([]*Edge, n)
		rimTop := make([]*Edge, n)
		diagE := make([]*Edge, n)
		rungE := make([]*Edge, n)
		for j := range n {
			jn := (j + 1) % n
			rimBottom[j] = loftEdge(vertexObjs, a.verts, vIdx[j], vIdx[jn], isOuter, a.delta)
			rimTop[j] = loftEdge(vertexObjs, a.verts, wIdx[j], wIdx[jn], isOuter, a.delta)
		}
		for j := range n {
			jn := (j + 1) % n
			jp := (j - 1 + n) % n
			rungConvex := junctionConvex(a.verts, lowerTri[i][jp], upperTri[i][j], vIdx[j], wIdx[j])
			rungE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[j], rungConvex, a.delta)
			diagConvex := junctionConvex(a.verts, lowerTri[i][j], upperTri[i][j], vIdx[j], wIdx[jn])
			diagE[j] = loftEdge(vertexObjs, a.verts, vIdx[j], wIdx[jn], diagConvex, a.delta)
		}

		capStartCo := make([]coedge, n)
		capEndCo := make([]coedge, n)
		for j := range n {
			jn := (j + 1) % n

			lowerFace, err := buildLoftWallFace(body, ref, a.verts, lowerTri[i][j], i, j, 0, a.delta)
			if err != nil {
				return nil, nil, nil, err
			}
			lowerFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: rimBottom[j], forward: true},
				{edge: rungE[jn], forward: true},
				{edge: diagE[j], forward: false},
			}, a.reversed)}}
			walls = append(walls, lowerFace)

			upperFace, err := buildLoftWallFace(body, ref, a.verts, upperTri[i][j], i, j, 1, a.delta)
			if err != nil {
				return nil, nil, nil, err
			}
			upperFace.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: diagE[j], forward: true},
				{edge: rimTop[j], forward: false},
				{edge: rungE[j], forward: false},
			}, a.reversed)}}
			walls = append(walls, upperFace)

			capStartCo[n-1-j] = coedge{edge: rimBottom[j], forward: false}
			capEndCo[j] = coedge{edge: rimTop[j], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capStartCo, a.reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: isOuter, coedges: loftLoopCoedges(capEndCo, a.reversed)})
	}

	capStartSurf, err := planeFromTriangle(a.verts, a.tris[a.walls])
	if err != nil {
		return nil, nil, nil, err
	}
	capEndSurf, err := planeFromTriangle(a.verts, a.tris[a.walls+a.capStartCount])
	if err != nil {
		return nil, nil, nil, err
	}
	cap0Val, _ := cap0Rat.Float64()
	cap1Val, _ := cap1Rat.Float64()
	capStartBound := rationalFloatError(cap0Rat, cap0Val)
	capEndBound := rationalFloatError(cap1Rat, cap1Val)
	if a.delta > 0 {
		capStartTris := a.tris[a.walls : a.walls+a.capStartCount]
		capEndTris := a.tris[a.walls+a.capStartCount:]
		capStartBound = absSumUpper(capStartBound, capTriangleAreaAllow(a.verts, capStartTris, a.delta))
		capEndBound = absSumUpper(capEndBound, capTriangleAreaAllow(a.verts, capEndTris, a.delta))
	}
	capStart := &Face{
		surface:       capStartSurf,
		loops:         capStartLoops,
		origins:       []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:          body,
		area:          cap0Val,
		areaBound:     capStartBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       capEndSurf,
		loops:         capEndLoops,
		origins:       []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:          body,
		area:          cap1Val,
		areaBound:     capEndBound,
		axialDelta:    a.delta,
		hasAxialDelta: true,
	}

	return capStart, capEnd, walls, nil
}

// capTriangleAreaAllow sums bounds.go's perturbedTriangleAreaAllow over one
// cap's own triangulation triangles (docs/loft-design.md §12 PR 2a) — the
// extra area a placement's delta can add to a cap's own exact rational area
// (capPolygonAreaRat), summed the same way loft_moments.go's accumulator
// sums it for the wall triangles.
func capTriangleAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		total = upRound(total + perturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], delta))
	}
	return total
}

// capPolygonAreaRat returns the exact rational shoelace area of the cap
// polygon this construction ACTUALLY assembled: pts in that plane's own
// local (U, V) coordinates, walked per loop in loopIdx's own recorded walk
// order — assembleLoft's own pts0/loopIdx0 or pts1/loopIdx1, the identical
// arrays triangulate2DContext consumed to build that cap's own triangles.
// Reading the SAME points the triangles came from, rather than
// re-deriving the region's area from the record (moments.go), is what
// keeps the published cap Area and the built cap triangles in lockstep by
// construction: whatever assembleLoft walked into a triangle is exactly
// what this sums. On an untrimmed LineSeg profile the two are the same
// points and so agree exactly; on a TRIMMED LineSeg profile they did not
// before this function existed, because assembleLoft's own walk lands on
// walkOf's float lerp2 endpoint while moments.go's region-level
// accumulator integrates the exact rational ratLerp — a pre-existing gap
// this closes, not a regression (moments.go's ratLerp/lerp2 doc comments).
//
// The outer loop walks CCW and each hole walks CW
// (docs/sketch-seam-design.md), and a per-loop shoelace sum already nets a
// hole's area out with no special-casing — the identical convention
// moments.go's own Green's-theorem accumulator relies on (ProfileRecord.
// Area's own doc comment: "a hole's clockwise walk subtracts without a
// special case").
//
// Every coordinate is taken exactly as a math/big.Rat off its own float64
// (clearance_poly.go's mustRatOf, the package's take-the-floats-exactly
// discipline) — no float arithmetic anywhere in this sum. mustRatOf's
// finiteness precondition is already proven here: every pts entry is one
// of the SAME (U, V) pairs assembleLoft already lifted through its plane
// frame and checked with finiteVec before this function is ever reached
// (errLoftPointUnrepresentable, S13), so a non-finite U or V would have
// refused the build already.
//
// pts/loopIdx carry no assumption about segment kind, so admitting a
// curved same-kind pairing later needs no rework here: whatever stations
// assembleLoft chords a curve into become more pts entries this same
// shoelace sums unchanged.
func capPolygonAreaRat(pts []Point2, loopIdx [][]int) *big.Rat {
	sum := new(big.Rat)
	for _, idx := range loopIdx {
		n := len(idx)
		for j := range n {
			p, q := pts[idx[j]], pts[idx[(j+1)%n]]
			term := new(big.Rat).Mul(mustRatOf(p.U), mustRatOf(q.V))
			term.Sub(term, new(big.Rat).Mul(mustRatOf(q.U), mustRatOf(p.V)))
			sum.Add(sum, term)
		}
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}

// validateLoftBodyMeasurements is evalLoft's own finiteness gate (design O2).
// Volume, Centroid and Bounds must be fully finite, exactly as every other
// analytic payload's validateAnalyticBodyMeasurements requires — but Area's
// Bound is deliberately NOT checked: §8 requires a saturated Area bound to
// publish +Inf as a proof term (a wall set whose areas approach float64's own
// ceiling), and checking it here would refuse the very body that fixture
// constructs. Only Area's Value is checked for finiteness.
func validateLoftBodyMeasurements(body *Body) error {
	if !finiteMeasurementValues(body.volume.Value.Base(), body.volume.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's volume measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.area.Value.Base()) {
		return fmt.Errorf(`%w: the loft's area value is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.centroid.Value.X, body.centroid.Value.Y, body.centroid.Value.Z, body.centroid.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's centroid measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(
		body.bounds.Min.X, body.bounds.Min.Y, body.bounds.Min.Z,
		body.bounds.Max.X, body.bounds.Max.Y, body.bounds.Max.Z,
		body.bounds.Bound.Base(),
	) {
		return fmt.Errorf(`%w: the loft's bounds measurement is not finite`, ErrNotFinite)
	}
	return nil
}

// evalLoft builds the lofted body from the payload's own records
// (docs/loft-design.md §5-§8): pairing, assembly, the §6 audit, topology, and
// the four measurements — all four published at build, never staged (§12).
// budget is shared with the rest of the pre-commit cancellation path exactly
// as modify §5's audits already share one; the caller (LoftContext, PR 1b)
// mints it once for the whole build.
//
// work0/work1 are the per-profile free-form work counters (spline design
// §5.2): the R7 ceiling is one record's across a whole OPERATION, and
// LoftContext also runs falsifyRecordedArea on both records before evalLoft
// is called, so those counters — not two fresh ones minted here — must be
// the ones every walkOf call site in this build spends against. S3 admits
// only same-kind LineSeg or circular pairs, neither of which is a free-form
// kind, so nothing here charges them yet — but the counters are still
// threaded through so a future free-form correspondence does not silently
// open a second ceiling per record.
func evalLoft(ctx context.Context, d *Document, ref StepRef, pl loftPayload, budget *workBudget, work0, work1 *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	offsets, walks0, walks1, err := validateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, work0, work1)
	if err != nil {
		return nil, err
	}

	target, err := loftChordTarget(pl.profile0, pl.profile1, walks0, walks1)
	if err != nil {
		return nil, err
	}

	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftPairings(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	if err != nil {
		return nil, err
	}

	a, err := assembleLoft(ctx, pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	if err != nil {
		return nil, err
	}

	if err := loftCrossingAudit(budget, a.verts, a.tris); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cap0Rat := capPolygonAreaRat(a.pts0, a.loopIdx0)
	cap1Rat := capPolygonAreaRat(a.pts1, a.loopIdx1)

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	capStart, capEnd, walls, err := buildLoftTopology(ctx, body, ref, a, cap0Rat, cap1Rat)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	faces := append([]*Face{capStart, capEnd}, walls...)
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}

	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	anchor := pl.xform.Apply(pl.plane0.Origin)
	mass := newLoftMassAccumulator(anchor, a.delta, sectionDelta, sectionMatchedDelta)
	for k, t := range a.tris {
		mass.add(a.verts[t[0]], a.verts[t[1]], a.verts[t[2]], k < a.walls)
	}
	// The chorded correction terms (docs/loft-design.md §5/§8, a10-plan.md
	// Part 3 PR 6) read the mass accumulator's own coordUpper, which is only
	// complete once every triangle has folded into it above — so this runs
	// after the add loop, gated on EITHER sectionDelta or sectionMatchedDelta
	// being positive rather than on sectionDelta alone: a free-form cell can
	// carry a positive matchedDelta at an exactly-zero sagitta
	// (spline_sagitta.go's own counterexample), and skipping the computation
	// there would silently drop a genuine chord-to-curve area/volume
	// obligation. Left at its zero value (every field of loftChordedAllow)
	// for a LineSeg-only build, where both are zero.
	if sectionDelta > 0 || sectionMatchedDelta > 0 {
		mass.chorded = computeLoftChordedAllow(pairs, a.vIdx, a.wIdx, a.verts, anchor, sectionDelta, sectionMatchedDelta, mass.distUpper)
	}
	body.volume = mass.volume(a.verts, a.tris)
	centroid, err := mass.centroid(a.verts, a.tris)
	if err != nil {
		return nil, err
	}
	body.centroid = centroid
	bounds, ok := mass.bounds()
	if !ok {
		return nil, fmt.Errorf(`%w: the loft has no vertices to bound`, ErrDegenerate)
	}
	body.bounds = bounds
	body.area = mass.area(cap0Rat, cap1Rat)

	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}

	pl.verts, pl.tris, pl.walls = a.verts, a.tris, a.walls
	pl.delta = a.delta
	// sectionDelta is loftPairings' own accumulated MAX over cells
	// (loftPayload's own doc comment): zero for a LineSeg-only pairing,
	// positive for a same-kind circular one, to the sagitta its station
	// chording commits.
	pl.sectionDelta = sectionDelta
	body.payload = pl
	return body, nil
}
