package loftmesh

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// RecordProfile contains the loop records needed for loft pairing gates.
type RecordProfile struct {
	Outer sectionrecord.LoopRecord
	Holes []sectionrecord.LoopRecord
}

// ValidateRecordWalks applies docs/loft-design.md Table S rows S1, S2, S4, S3,
// S7's STRUCTURAL arm and S5, in §4's stated gate order, from the two
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
// to j+1 — never batched a whole loop ahead of that order. boundarywalk.WalkOf is neither
// memoized nor free to call twice (it charges the free-form work budget on
// every call, extrude.go's own doc comment), so resolving here and never
// again (loftPairings reads this function's own output) is what Task 1
// exists for; keeping the interleaving is what keeps S3's own refusal
// PRECEDENCE unchanged — a record whose p0 fails S3 at an early segment
// must still report that refusal even when p1 carries a later segment
// boundarywalk.WalkOf itself cannot resolve at all (a malformed CircleSeg, say), a
// combination sketch's own authentication never produces but a decoded
// evaluator can.
//
// S3's admission test is a SAME-KIND test over the two RECORDED SEGMENT
// TYPES, exactly the three-way enumeration docs/loft-design.md §1 and Table P
// row P5 spell — both LineSeg, both ArcSeg, or both CircleSeg — never merely
// because one side is a LineSeg. Mixed-kind and free-form pairs keep today's
// refusal and today's sentinel (SameKindGate). Testing only after BOTH
// sides are resolved (rather than each side against its own kind test, as the
// LineSeg-only form did) is unavoidable once the admitted set has three
// types, and it does not relax PRECEDENCE: the first (i, j) whose pair fails
// is still the first refusal reported, in the same walk order as before.
func ValidateRecordWalks(p0, p1 RecordProfile, pl0, pl1 sectionrecord.PlaneRecord, alignment []int, work0, work1 *freeform.FreeformWork) ([]int, [][]survey2d.SegmentWalk, [][]survey2d.SegmentWalk, error) {
	if len(p0.Holes) != len(p1.Holes) {
		return nil, nil, nil, fmt.Errorf(`%w: the two profiles have %d and %d holes; a loft has no positional pairing for a hole-count mismatch`,
			decaderr.ErrUnsupported, len(p0.Holes), len(p1.Holes))
	}
	loops0 := append([]sectionrecord.LoopRecord{p0.Outer}, p0.Holes...)
	loops1 := append([]sectionrecord.LoopRecord{p1.Outer}, p1.Holes...)
	loopCount := len(loops0)

	for i := range loops0 {
		if len(loops0[i].Segments) != len(loops1[i].Segments) {
			return nil, nil, nil, fmt.Errorf(`%w: loop %d has %d segments on the first profile and %d on the second; a loft has no one-to-one pairing for a segment-count mismatch`,
				decaderr.ErrUnsupported, i, len(loops0[i].Segments), len(loops1[i].Segments))
		}
	}

	offsets := make([]int, loopCount)
	if alignment != nil {
		if len(alignment) != loopCount {
			return nil, nil, nil, fmt.Errorf(`%w: WithLoftAlignment carries %d offsets for %d loops`,
				decaderr.ErrDegenerate, len(alignment), loopCount)
		}
		for i, off := range alignment {
			n := len(loops0[i].Segments)
			if off < 0 || off >= n {
				return nil, nil, nil, fmt.Errorf(`%w: loop %d's alignment offset %d is outside [0, %d)`,
					decaderr.ErrDegenerate, i, off, n)
			}
			offsets[i] = off
		}
	}

	walks0 := make([][]survey2d.SegmentWalk, loopCount)
	walks1 := make([][]survey2d.SegmentWalk, loopCount)
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		walks0[i] = make([]survey2d.SegmentWalk, n)
		walks1[i] = make([]survey2d.SegmentWalk, n)
		for j := range n {
			w0, err := boundarywalk.WalkOf(loops0[i].Segments[j], work0)
			if err != nil {
				return nil, nil, nil, err
			}

			k := (j + off) % n
			w1, err := boundarywalk.WalkOf(loops1[i].Segments[k], work1)
			if err != nil {
				return nil, nil, nil, err
			}

			if err := SameKindGate(loops0[i].Segments[j], loops1[i].Segments[k], i, j, k); err != nil {
				return nil, nil, nil, err
			}
			walks0[i][j] = w0
			walks1[i][k] = w1
		}
	}

	if PlanesCoincide(pl0, pl1) {
		return nil, nil, nil, fmt.Errorf(`%w: the two profiles lie in the same geometric plane; the loft has zero volume by construction`, decaderr.ErrDegenerate)
	}

	return offsets, walks0, walks1, nil
}

// SameKindGate is docs/loft-design.md Table S row S3 and Table P row P5
// in their arc form (a10-plan.md Part 3 PR 6): a pairing is admitted only
// when the two RECORDED SEGMENT TYPES are the same one of the three §1 and
// P5 enumerate — both LineSeg, both ArcSeg, or both CircleSeg — so a
// mixed-kind or free-form pair still refuses under today's sentinel.
//
// The test is on the CONCRETE recorded type, never on the resolved walk's
// own survey2d.WalkKind. survey2d.WalkCircular is one kind for a circle and an arc alike
// (extrude.go), so a survey2d.WalkKind test would admit an ArcSeg paired against a
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
// ErrLoftContact is the audit arm's spelling of it): a self-crossing shell
// bounds no solid under any evaluator, so this is never a staging refusal.
// What this arm buys is POSITION, not a different answer — it is decided
// from the two records alone, before a single station or triangle is built,
// where the audit would only reach the same verdict three build phases
// later.
//
// An ArcSeg's sweep is NOT structurally fixed CCW: boundarywalk.WalkOf's own ArcSeg arm
// (extrude.go) reads th0 = a0 + TStart*sweep, th1 = a0 + TEnd*sweep with
// sweep always forced positive, so the walk's own angle is monotonic in t and
// its EFFECTIVE direction is CCW exactly when TEnd > TStart — the identical
// formula validateSegmentWinding already enforces a CircleSeg's own CCW field
// must equal (record.go). circularSegmentCCW reads that one shared
// formula, so an ArcSeg pair whose two recorded ranges run opposite ways is
// caught here beside the CircleSeg pair P5 names, rather than left to surface
// as S7's own crossing refusal three build phases later.
func SameKindGate(seg0, seg1 sectionrecord.CurveSegment, loop, j, k int) error {
	t0, t1 := PairTypeOf(seg0), PairTypeOf(seg1)
	if t0 == PairUnadmitted || t0 != t1 {
		return fmt.Errorf(`%w: loop %d segment %d of the first profile and segment %d of the second are not the same admitted segment type; this evaluator pairs two LineSegs, two ArcSegs or two CircleSegs only`,
			decaderr.ErrUnsupported, loop, j, k)
	}
	if t0 == PairLine {
		return nil
	}
	ccw0, ok0 := circularSegmentCCW(seg0)
	ccw1, ok1 := circularSegmentCCW(seg1)
	if ok0 && ok1 && ccw0 != ccw1 {
		return fmt.Errorf(`%w: loop %d's paired circular segments at segment %d/%d walk in opposite directions; the correspondence walls each side against the other's reversed walk, so the shell self-crosses and bounds no solid`,
			decaderr.ErrDegenerate, loop, j, k)
	}
	return nil
}

// PairType is the three-way enumeration docs/loft-design.md §1 and Table
// P row P5 admit a loft pairing over, plus PairUnadmitted for every other
// recorded type. It reads the CONCRETE recorded segment type — the walk kind
// resolved from it is coarser (survey2d.WalkCircular covers a circle and an arc alike,
// extrude.go) and cannot state this contract.
type PairType uint8

const (
	// PairUnadmitted is every recorded type outside the three below —
	// each free-form kind, and each conic kind the seam records. A pair is
	// refused when either side reads this value, even when both do: §1
	// admits three enumerated types and nothing else.
	PairUnadmitted PairType = iota
	PairLine
	PairArc
	PairCircle
)

// PairTypeOf classifies one recorded segment into PairType. A segment
// NormalizeSegment itself refuses — a nil typed pointer can
// carry — reads PairUnadmitted rather than panicking; boundarywalk.WalkOf resolves
// each side ahead of this gate and reports that refusal first
// (ValidateRecordWalks' own precedence note), so the value is never the one a
// caller sees.
func PairTypeOf(seg sectionrecord.CurveSegment) PairType {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return PairUnadmitted
	}
	switch seg.(type) {
	case sectionrecord.LineSeg:
		return PairLine
	case sectionrecord.ArcSeg:
		return PairArc
	case sectionrecord.CircleSeg:
		return PairCircle
	default:
		return PairUnadmitted
	}
}

// circularSegmentCCW reads a circular segment's own EFFECTIVE walk
// direction structurally, from ONE shared formula rather than trusting a
// per-kind field (SameKindGate's own doc comment): a CircleSeg's CCW
// flag is required to already equal TStart < TEnd (record.go's
// validateSegmentWinding), and an ArcSeg's own boundarywalk.WalkOf arm forces its sweep
// positive (extrude.go), so its angle is a STRICTLY INCREASING function of
// t and its own walk visits increasing angle exactly when TEnd > TStart —
// the identical formula. The false return is defensive, unreached from any
// real build today since SameKindGate has already proven both sides
// record the SAME circular type, which only CircleSeg and ArcSeg are.
func circularSegmentCCW(seg sectionrecord.CurveSegment) (bool, bool) {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return false, false
	}
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		return s.CCW, true
	case sectionrecord.ArcSeg:
		return s.TEnd > s.TStart, true
	default:
		return false, false
	}
}

// PlanesCoincide decides S5 over exact rationals on the recorded U/V/
// Origin floats (internal/meshbool/boolean_exact.go's proof.XptOf/xcross/xdot, the take-the-floats-
// exactly discipline): the two planes coincide when their normals (U×V) are
// exactly parallel and the displacement between their origins lies in that
// plane. A tolerance here would refuse a legitimately thin loft, and the
// existence claim S5 makes is a structural zero volume, not a small one.
func PlanesCoincide(a, b sectionrecord.PlaneRecord) bool {
	na := meshbool.Xcross(proof.XptOf(a.U), proof.XptOf(a.V))
	nb := meshbool.Xcross(proof.XptOf(b.U), proof.XptOf(b.V))
	cr := meshbool.Xcross(na, nb)
	if cr.X.Sign() != 0 || cr.Y.Sign() != 0 || cr.Z.Sign() != 0 {
		return false
	}
	d := proof.Xsub(proof.XptOf(b.Origin), proof.XptOf(a.Origin))
	return meshbool.XdotSign(na, d) == 0
}
