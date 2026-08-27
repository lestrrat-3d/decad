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
	// placed image of the recorded sections (docs/loft-design.md §5, §12 PR
	// 2a): zero for an unplaced body — pl.xform == r3.Identity(), an exact
	// struct comparison, never a tolerance — and otherwise
	// bounds.go's rigidRoundAllow, read at the pre-transform lifted point's
	// own magnitude and the composed translation's magnitude. Every
	// measurement this payload publishes composes it.
	delta float64

	// sectionDelta holds the term of that name in docs/loft-design.md §5.2's
	// table, which owns the quantity it bounds, the row it is derived from and
	// its maximum-versus-sum rule; this comment states none of them. It is
	// zero for every pairing this evaluator admits today — S3 admits only
	// same-kind LineSeg pairs, and a straight wall's own chord IS the recorded
	// segment, so there is no curve for it to depart from. A same-kind curved
	// pairing (reach, not yet admitted) is the
	// construction that sets it, to the two-term bound its station chording
	// commits (chordCellDeltaUpper): the CERTIFIED sagitta
	// (loftCertifiedSagittaUpper) composed with the stations' own displacement
	// (circularStationChain) under the provenance mark §5.1's Table C gives
	// them, never to a held float and never to either term alone.
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
	// helper's own doc comment defines. §5.2's table owns both terms and the
	// relation between them: its matchedDelta row states what that
	// parameter-matched quantity is, what it is composed from and what it
	// refuses on. A consumer that needs it reads that row and composes the
	// terms the row names, never this field on its own.
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

// validateLoftRecords applies docs/loft-design.md Table S rows S1, S2, S4, S3,
// S5 and S15, in §4's stated gate order, from the two authenticated records
// alone — no triangle is built. It returns the normalized per-loop alignment
// offsets (a nil alignment becomes every offset 0, §2) alongside every
// segment's own resolved walk, one slice per loop, in that loop's own
// recorded segment order — NOT rotated by the alignment offset, which stays
// applied at loftPairings' own point of use, exactly as it is here for S3's
// own check.
//
// Each segment is walked exactly ONCE, at the SAME point in the SAME
// interleaved per-segment order this gate has always used — walk p0's
// segment j, test S3, walk p1's segment k=(j+off)%n, test S3, then move to
// j+1 — never batched a whole loop ahead of that order. walkOf is neither
// memoized nor free to call twice (it charges the free-form work budget on
// every call, extrude.go's own doc comment), so resolving here and never
// again (loftPairings reads this function's own output) is what Task 1
// exists for; keeping the interleaving is what keeps S3's own refusal
// PRECEDENCE unchanged — a record whose p0 fails S3 at an early segment
// must still report that refusal even when p1 carries a later segment
// walkOf itself cannot resolve at all (a malformed CircleSeg, say), a
// combination sketch's own authentication never produces but a decoded
// recipe can (docs/recipe-replay-design.md).
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
			if !w0.isLine() {
				return nil, nil, nil, fmt.Errorf(`%w: loop %d segment %d of the first profile is not a LineSeg; this evaluator rules straight lines only`,
					ErrUnsupported, i, j)
			}
			walks0[i][j] = w0

			k := (j + off) % n
			w1, err := walkOf(loops1[i].Segments[k], work1)
			if err != nil {
				return nil, nil, nil, err
			}
			if !w1.isLine() {
				return nil, nil, nil, fmt.Errorf(`%w: loop %d segment %d of the second profile is not a LineSeg; this evaluator rules straight lines only`,
					ErrUnsupported, i, k)
			}
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
	// S3 above refuses every non-LineSeg pair, so no record reaches this gate
	// with a circular pair to measure until the arc correspondence lands
	// (docs/loft-design.md §12 PR 3). The gate is exercised at its own entry
	// point meanwhile, the same way the circular station generator itself is
	// (loft_stations_internal_test.go's own header).
	if err := loftStationCapGate(p0, p1, offsets, walks0, walks1); err != nil {
		return nil, nil, nil, err
	}

	return offsets, walks0, walks1, nil
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

// loftStationCap is docs/loft-design.md §5.1's ceiling on a build's TOTAL
// station count Σstations (§7) — the soft limit that keeps the chord chain
// from being what carries §6's audit past the pair-test ceiling S8 owns. §14
// left its value to this increment; the derivation is here.
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

// loftLoopPair is Table P's correspondence for one loop: the two walk-ordered
// STATION-chain lists, v from loop0's own segment order and w from loop1's,
// already rotated by that loop's own alignment offset (P4). Each paired
// segment contributes the entries its own generator arm produces
// (a10-plan.md Part 3 PR 5), and every list still
// carries only each segment's OWN interior stations, never its shared end
// point: the next segment's own first station (or the loop's wrap) supplies
// it, which is what makes a loop's list total the count
// docs/loft-design.md §7 states for that loop rather than one stated here.
type loftLoopPair struct {
	v, w []Point2
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
func loftCellStations(w0, w1 segmentWalk, seg0, seg1 CurveSegment, target float64, work0, work1 *freeformWork) ([]Point2, []Point2, float64, error) { //nolint:unparam // work0/work1 are part of the fixed kind-switch interface every future arm shares; the ARC and LineSeg arms below are the two that do not need them yet.
	switch {
	case w0.kind == walkLine && w1.kind == walkLine:
		return loftLineCellStations(w0, w1)
	case w0.kind == walkCircular && w1.kind == walkCircular:
		return loftCircularCellStations(w0, w1, seg0, seg1, target)
	default:
		// Unreached from any real build today: validateLoftRecords' own S3
		// gate refuses every non-LineSeg pair before loftPairings ever calls
		// this function (loft_build.go's own header comment states the same
		// shape for the original PR 1a landing). A defensive refusal, not a
		// dead branch a caller could reach silently: a future kind this
		// switch has no case for yet must still fail loud rather than fall
		// through into either analytic arm's own assumptions.
		return nil, nil, 0, fmt.Errorf(`%w: this loft evaluator has no chord station rule for this segment-kind pairing`, ErrUnsupported)
	}
}

// loftLineCellStations is the LineSeg arm: one station per side, at the
// segment's own recorded start, with zero sagitta — a straight wall's own
// chord IS the recorded segment, so there is no curve for it to depart from.
// m is fixed at 1, so this arm's output is bit-identical to every LineSeg
// pairing this evaluator built before the station generator existed.
func loftLineCellStations(w0, w1 segmentWalk) ([]Point2, []Point2, float64, error) {
	return []Point2{{U: w0.startU, V: w0.startV}}, []Point2{{U: w1.startU, V: w1.startV}}, 0, nil
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
	m0, _, err := chordCount(w0, target)
	if err != nil {
		return 0, 0, 0, err
	}
	m1, _, err := chordCount(w1, target)
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
// sagittaUpper covers both sides of the pair at the settled count:
// chordCellDeltaUpper composes the two terms §5.2's table lists for a chorded
// cell, each read across the two sides under the max-versus-sum rule that
// table states for it rather than one stated here. Only the sagitta half is
// walked up against the target: the target names the chord DEPTH the chording
// commits to, while the station displacement is a rounding term of the
// generator's own arithmetic, composed into the published bound once the count
// has settled.
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
func loftCircularCellStations(w0, w1 segmentWalk, seg0, seg1 CurveSegment, target float64) ([]Point2, []Point2, float64, error) {
	m, s0, s1, err := loftSettleStationCount(w0, w1, seg0, seg1, target)
	if err != nil {
		return nil, nil, 0, err
	}

	stations0, d0 := circularStationChain(w0, seg0, m)
	stations1, d1 := circularStationChain(w1, seg1, m)

	// S14, in the arm and at the phase §4's gate-order paragraph assigns the
	// station-displacement term: a side whose stations have no proven
	// displacement from the recorded points they stand for cannot state this
	// cell's own chord bound at all, so the cell refuses rather than
	// publishing the certified sagitta alone — which would be a bound on a
	// chord this build did not draw.
	chordDelta := chordCellDeltaUpper(math.Max(s0, s1), math.Max(d0, d1))
	if isNonFinite(chordDelta) {
		return nil, nil, 0, errLoftStationDisplacementUnderivable
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
			return nil, nil, 0, fmt.Errorf(`%w: chord cell %d of this paired segment collapses to one point on only one of the two sections`, ErrUnsupported, k)
		}
	}

	return stations0, stations1, chordDelta, nil
}

// chordCellDeltaUpper composes one chord cell's two independent displacement
// terms into the single bound loftCellStations publishes: the certified
// per-cell sagitta and the station displacement, both of which
// docs/loft-design.md §5.2's table lists with the quantity each bounds and the
// certified source each is read from.
//
// That table owns the composition's derivation and its rounding direction, and
// this helper adds no mechanism of its own to either. What the code here does
// state is that both terms are read at the same s, which is what keeps the
// published bound PARAMETER-MATCHED (loftCellStations' own doc comment).
//
// Either term underivable answers +Inf, the answer §5.2's table assigns those
// rows, and the caller refuses on it.
func chordCellDeltaUpper(sagittaUpper, stationUpper float64) float64 {
	if isNonFinite(sagittaUpper) || isNonFinite(stationUpper) {
		return math.Inf(1)
	}
	return absSumUpper(sagittaUpper, stationUpper)
}

// errLoftStationDisplacementUnderivable is the sentinel docs/loft-design.md
// Table S row S14 carries for the station-displacement term, raised in the arm
// §4's gate-order paragraph assigns that term: a chorded circular pair whose
// stations have no proven displacement from the recorded points they stand
// for. Like its certified-sagitta twin the shape itself is fine and the chord
// set is buildable; only one of the two terms the published chord bound is
// composed from cannot be stated, so the sentinel is ErrUnsupported and no
// finite value — least of all the sagitta alone — is published in its place.
var errLoftStationDisplacementUnderivable = fmt.Errorf(
	`%w: a chorded circular pair's generated stations have no proven displacement from the recorded curve`, ErrUnsupported,
)

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
// would leave every reading that reads a pinned station's zero displacement
// (docs/loft-design.md §5.2's two pinned station kinds) stating a zero the
// built vertex does not have. Reading the walk keeps the pin and the station
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
// A station the record cannot enclose answers +Inf, which the caller refuses
// on rather than publishing the certified sagitta as if it were the whole
// bound.
func circularStationChain(w segmentWalk, seg CurveSegment, m int) ([]Point2, float64) {
	pts := make([]Point2, m)
	pts[0] = Point2{U: w.startU, V: w.startV}
	delta := math.Max(walkEndPlaneDelta(w.startBound), walkEndPlaneDelta(w.endBound))
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
// (a10-plan.md Part 3 PR 5), so a loop's v/w lists carry whatever entries that
// segment's own arm generates and total the count docs/loft-design.md §7
// states for the loop; a LineSeg pairing's own lists stay bit-identical to
// what this evaluator built before the generator existed. sectionDelta
// accumulates every cell's own published chord bound under the max-versus-sum
// rule §5.2's table states for that term, never one stated here. Within ONE
// cell the two terms that bound ARE summed (chordCellDeltaUpper) — they are
// displacements of two different things, not two cells' readings of one.
//
// Both records are read, never p0 alone: a curved arm's own bound is stated by
// the RECORDED segment behind each side's walk (loftCellStations' own doc
// comment), so each side's segment is handed to the generator alongside its
// walk, under the same alignment offset the walk itself is read at.
func loftPairings(p0, p1 ProfileRecord, offsets []int, walks0, walks1 [][]segmentWalk, target float64, work0, work1 *freeformWork) ([]loftLoopPair, float64, error) {
	loops0 := append([]LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]LoopRecord{p1.Outer}, p1.Holes...)
	pairs := make([]loftLoopPair, len(loops0))
	sectionDelta := 0.0
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		var v, w []Point2
		for j := range n {
			w0 := walks0[i][j]
			k := (j + off) % n
			w1 := walks1[i][k]
			stations0, stations1, sagitta, err := loftCellStations(w0, w1, loops0[i].Segments[j], loops1[i].Segments[k], target, work0, work1)
			if err != nil {
				return nil, 0, err
			}
			v = append(v, stations0...)
			w = append(w, stations1...)
			sectionDelta = math.Max(sectionDelta, sagitta)
		}
		if err := loftOneSidedCellGate(i, v, w); err != nil {
			return nil, 0, err
		}
		pairs[i] = loftLoopPair{v: v, w: w}
	}
	return pairs, sectionDelta, nil
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
	// §12 PR 2a) — zero exactly when xform is r3.Identity().
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
func assembleLoft(ctx context.Context, pairs []loftLoopPair, f0, f1 r3.Frame, plane0 PlaneRecord, xform r3.Transform) (loftAssembly, error) {
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

	// delta is zero exactly when xform is the identity transform — an exact
	// struct comparison, never a tolerance. This fast path is REQUIRED:
	// without it, every directly-built (unplaced) loft would lose the Exact
	// readings §8/§12 PR 1 publishes (docs/loft-design.md §5, §12 PR 2a).
	delta := 0.0
	if xform != r3.Identity() {
		delta = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}

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
// what this sums. On an untrimmed LineSeg profile that walked point IS the
// record's own endpoint, so this sum and moments.go's region-level integral
// are the same rational. On a TRIMMED LineSeg profile they are not: the walk
// lands on walkOf's float lerp2 endpoint while moments.go integrates the
// exact rational ratLerp (moments.go's ratLerp/lerp2 doc comments), and the
// cap reading follows the walked point, because that is the point the cap's
// own triangles have.
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
// the ones every walkOf call site in this build spends against. Increment 1
// admits only LineSeg pairs, so nothing here ever actually charges, but the
// counters are still threaded through so PR 3's curved correspondence does
// not silently open a second ceiling per record.
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

	pairs, sectionDelta, err := loftPairings(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	if err != nil {
		return nil, err
	}

	a, err := assembleLoft(ctx, pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform)
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
	mass := newLoftMassAccumulator(anchor, a.delta)
	for k, t := range a.tris {
		mass.add(a.verts[t[0]], a.verts[t[1]], a.verts[t[2]], k < a.walls)
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
	// sectionDelta is loftPairings' own accumulation, under the rule
	// docs/loft-design.md §5.2's table states for that term
	// (loftPayload's own doc comment): it is exactly zero today because S3
	// admits only same-kind LineSeg pairs, whose own chord IS the recorded
	// segment, so this is not yet observable through a real build — but the
	// wiring is live, so a same-kind curved pairing (reach, not yet
	// admitted) sets it with no further plumbing here.
	pl.sectionDelta = sectionDelta
	body.payload = pl
	return body, nil
}
