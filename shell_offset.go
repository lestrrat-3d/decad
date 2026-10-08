package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the exact section offset of docs/modify-design.md §7 and the §5
// audit wrapper for a shell. The offset is per-feature and topology-preserving:
// a line offsets to a parallel line, a circle to a concentric circle, a corner
// closes with a miter or an arc of radius t about the corner point or, at a G1
// join, with the corner moved t along the shared normal (§7). Every piece
// is a line or an arc, so P ⊖ t / P ⊕ t is again a ProfileRecord in the
// line-and-arc vocabulary (§2). The offset is decad's OWN synthesized geometry,
// proven by exact closed-form tests, never a residual (§5, CLAUDE.md's
// falsify-only rule governs only what sketch hands over).
//
// A feature the offset DROPS or a miter that does not close is a feature-set
// change the evaluator cannot build without a trimmed-offset kernel — S11a /
// S11, ErrUnsupported, caught here as the offset is constructed and so
// antecedent to the audit (§4). A drop takes two shapes, both caught here: a
// circular segment whose offset radius reaches zero (offsetRadius), and a whole
// loop whose erosion is empty — a polygonal hole narrower than 2t offset
// outward, whose corner joins overshoot every walk so each runs backward
// (walkOffsetConsumed). The eroded loop keeps its walk sense, so its signed
// area does not change sign and the §5 audit's S8 cannot see it — which is why
// the drop is decided per walk, here, before there is any section to audit. A
// merge the audit catches later is S11b.

// errOffsetDrop marks a feature the offset drops (S11a).
var errOffsetDrop = offset2d.ErrDrop

// errOffsetTopology marks a corner whose offset carriers do not close into a
// miter — a feature-set change this evaluator cannot resolve (S11).
var errOffsetTopology = offset2d.ErrTopology

// offsetProfile computes the topology-preserving offset of a section: P ⊖ t
// (inward, s = +1) or P ⊕ t (outward, s = −1), each loop offset in its own
// sense (docs/modify-design.md §7). A dropped feature is S11a (errOffsetDrop);
// a non-closing miter is S11 (errOffsetTopology). Both are ErrUnsupported.
func offsetProfile(budget *proofbound.WorkBudget, profile ProfileRecord, s, t float64) (ProfileRecord, error) {
	return offsetProfileBudget(budget, profile, s, t)
}

func offsetProfileBudget(budget *proofbound.WorkBudget, profile ProfileRecord, s, t float64) (ProfileRecord, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return ProfileRecord{}, err
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return ProfileRecord{}, err
	}
	out := make([]LoopRecord, len(loops))
	for i, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return ProfileRecord{}, err
		}
		segs, err := offsetLoopBudget(budget, loop, s, t)
		if err != nil {
			return ProfileRecord{}, err
		}
		out[i] = LoopRecord{Segments: segs}
	}
	return ProfileRecord{Outer: out[0], Holes: out[1:]}, nil
}

// offsetLoopBudget offsets one coalesced loop through the section offset owner.
func offsetLoopBudget(budget *proofbound.WorkBudget, loop cornerLoop, s, t float64) ([]CurveSegment, error) {
	return offset2d.BuildLoop(budget, loop.walks, s, t, shellTol)
}

// offsetJoinsBudget resolves every corner join of one coalesced loop of two or
// more walks, in walk order: corner i sits at walk i's start (== walk i−1's
// end). It is the one place the corner rule of docs/modify-design.md §7 is
// decided, so the offset build (offsetLoopBudget) and its displacement proof
// (offsetSectionDelta) always read the same joins.
func offsetJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64) ([]cornerJoin, error) {
	result, err := offset2d.SectionJoinsBudget(budget, walks, s, t, shellTol)
	if err != nil {
		return nil, err
	}
	joins := make([]cornerJoin, len(result))
	for i, j := range result {
		joins[i] = cornerJoin{
			arc: j.Arc, g1: j.G1, vU: j.VertU, vV: j.VertV,
			m:  Point2{U: j.M.U, V: j.M.V},
			pA: Point2{U: j.PA.U, V: j.PA.V},
			pB: Point2{U: j.PB.U, V: j.PB.V},
		}
	}
	return joins, nil
}

// cornerJoin is one corner's resolved offset join: a miter point m, or an arc
// of radius t about (vU, vV) from pA (the arriving walk's offset end) to pB (the
// leaving walk's offset start).
type cornerJoin struct {
	arc bool
	// g1 marks a G1 join (modify §7): m is the leaving walk's offset start
	// v + s·t·n̂, not a carrier intersection, and the corner ruling v→m is the
	// exact affine locus.
	g1     bool
	vU, vV float64
	m      Point2
	pA, pB Point2
}

// offsetRadius is a circular walk's offset radius, R − s·insideSign·t: R − t
// where the material is inside the circle (a CCW walk) and R + t where it is
// outside (a CW walk — a hole wall, a concave round), inward; the signs move the
// other way outward. ok is false when the radius reaches zero or goes negative —
// the segment drops (S11a).
func offsetRadius(w survey2d.SideWalk, s, t float64) (float64, bool) {
	return offset2d.OffsetRadius(w, s, t, shellTol)
}

// offsetWalkSegment re-emits a walk's trimmed offset curve.
func offsetWalkSegment(w survey2d.SideWalk, s, t float64, start, end Point2) (CurveSegment, error) {
	return offset2d.WalkSegment(w, s, t, offset2d.Point{U: start.U, V: start.V},
		offset2d.Point{U: end.U, V: end.V}, shellTol)
}

// walkOffsetConsumed reports whether an offset trimmed away a source walk.
func walkOffsetConsumed(w survey2d.SideWalk, start, end Point2) bool {
	return offset2d.WalkConsumed(w, offset2d.Point{U: start.U, V: start.V},
		offset2d.Point{U: end.U, V: end.V}, shellTol)
}

// circleSegConcentric records a full-circle walk in its own sense.
func circleSegConcentric(cu, cv, rr float64, ccw bool) CurveSegment {
	return offset2d.CircleSegment(cu, cv, rr, ccw)
}

// reverseLoopRecord walks a loop in the opposite sense, re-emitting each segment
// reversed — what turns an offset outer loop into a tube's hole (a hole is
// walked clockwise, so its wall's material lies outside it). It reads the loop
// through walkOf, so it handles line, arc and full-circle segments alike.
func reverseLoopRecord(l LoopRecord) (LoopRecord, error) {
	return reverseLoopRecordBudget(nil, l)
}

func reverseLoopRecordBudget(budget *proofbound.WorkBudget, l LoopRecord) (LoopRecord, error) {
	return reverseLoopRecordWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, l)
}

func reverseLoopRecordContext(ctx context.Context, l LoopRecord) (LoopRecord, error) {
	return reverseLoopRecordWithPoll(ctx.Err, l)
}

func reverseLoopRecordWithPoll(poll func() error, l LoopRecord) (LoopRecord, error) {
	// One free-form counter for this loop's walk: reversal is reached from the
	// offset construction and the cup build, neither of which holds a preflight
	// counter for the loop it hands over.
	work := freeform.NewFreeformWork()
	n := len(l.Segments)
	walks := make([]survey2d.SegmentWalk, n)
	for i, seg := range l.Segments {
		if poll != nil {
			if err := poll(); err != nil {
				return LoopRecord{}, err
			}
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return LoopRecord{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "the shell section offset"); err != nil {
			return LoopRecord{}, err
		}
		walks[i] = w
	}
	segs := make([]CurveSegment, 0, n)
	for i := n - 1; i >= 0; i-- {
		if poll != nil {
			if err := poll(); err != nil {
				return LoopRecord{}, err
			}
		}
		w := walks[i]
		switch {
		case w.Closed:
			segs = append(segs, circleSegConcentric(w.CU, w.CV, w.Radius, !(w.Th1 > w.Th0)))
		case w.IsCircular():
			segs = append(segs, arcSegment(Point2{U: w.CU, V: w.CV}, Point2{U: w.EndU, V: w.EndV}, Point2{U: w.StartU, V: w.StartV}, !(w.Th1 > w.Th0)))
		default:
			segs = append(segs, LineSeg{Start: Point2{U: w.EndU, V: w.EndV}, End: Point2{U: w.StartU, V: w.StartV}, TStart: 0, TEnd: 1})
		}
	}
	return LoopRecord{Segments: segs}, nil
}

// auditOffsetSectionBudget runs the shared §5 audit (fillet_audit.go) on a shell's
// offset section, in §4's order: S8 (orientation — an offset loop turned inside
// out is ErrDegenerate), the crossing test (a crossing or boundary contact of
// offset loops — S11b, ErrUnsupported), then S9 (nesting). A shell mints no
// cutback, so the empty fillet map makes the S6 trim test a no-op — the one test
// that cannot fire on an offset (§8).
func auditOffsetSectionBudget(budget *proofbound.WorkBudget, orig, offset ProfileRecord) error {
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: offset})
	if err != nil {
		return err
	}
	empty := make([]map[int]*cornerBlend, len(loops))
	for i := range empty {
		empty[i] = map[int]*cornerBlend{}
	}
	return auditRewriteBudget(budget, orig, offset, loops, empty)
}

// errOffsetUnbounded is the refusal for a cup whose offset section this
// evaluator cannot place within a proven distance of the offset it denotes: a
// walk kind with no closed-form carrier here, an endpoint with no stated
// bound, or carriers whose interval intersection is unbounded. The cup exists
// and only this evaluator cannot bound its readings, which is the
// ErrUnsupported side of docs/modify-design.md §1's existence test.
var errOffsetUnbounded = offset2d.ErrUnbounded

// offsetSectionDelta is the cup's offset displacement (docs/modify-design.md
// §9): a proven upper bound on how far any boundary point of the offset
// section offsetProfile records for (profile, s, t) sits from the boundary of
// the offset the shell DENOTES — P ⊖ t* inward, P ⊕ t* outward — where t* is
// the caller's thickness in exact millimetres, anywhere within tDelta of the
// held t (magnitudeInBounded's conversion bound).
//
// The denoted offset is §7's closed forms over the receiver's own walks taken
// exactly: a line walk is the segment between its recorded endpoints, a
// circular walk the circle about its recorded centre whose radius its walk
// brackets, each endpoint widened by the bound its walk states, and the
// corner rule (miter, arc, G1) is the construction's own (offsetJoinsBudget).
// The proof is an enclosure, the method capContourDelta
// (capblend_contour.go) states: the same closed forms are re-evaluated over
// rational intervals with outward-rounded square roots, over the whole
// thickness interval at once, and each recorded join point is charged its
// enclosure's greatest reach from the held float. A G1 join is enclosed by the
// hull of its two shared-normal feet, so a join the dead zone classified G1
// with a residual turn is charged that spread too.
//
// The figure is three times the largest reach. A recorded line segment and
// the denoted one are within the larger of their two endpoint reaches at
// every matching parameter. A recorded arc shares its centre with the denoted
// one, and its radius is within one reach of the denoted radius; a point the
// recorded arc sweeps past the denoted end lies on a chord no longer than two
// reaches from the recorded end, so it is within three reaches of the denoted
// end. Every recorded boundary point is therefore within the figure of the
// denoted boundary, and every denoted point within it of the record, which is
// the reading prismPayload.sectionDelta states for a whole section.
func offsetSectionDelta(budget *proofbound.WorkBudget, profile ProfileRecord, s, t, tDelta float64) (float64, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, err
	}
	rt, rd := proofarith.FloatRat(t), proofarith.FloatRat(tDelta)
	if rt == nil || rd == nil || rd.Sign() < 0 {
		return 0, errOffsetUnbounded
	}
	// amount is s·t* over every denoted thickness, the signed offset the
	// float build spells s*t.
	amount := proofbound.Interval(new(big.Rat).Sub(rt, rd), new(big.Rat).Add(rt, rd))
	if s < 0 {
		amount = proofbound.IntervalNeg(amount)
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return 0, err
	}
	reach := 0.0
	for _, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		r, err := offsetLoopReach(budget, loop.walks, s, t, amount)
		if err != nil {
			return 0, err
		}
		reach = math.Max(reach, r)
	}
	delta := proofbound.ProductUpper(3, reach)
	if proofbound.IsNonFinite(delta) {
		return 0, errOffsetUnbounded
	}
	return delta, nil
}

// offsetLoopReach reads the section offset displacement proof.
func offsetLoopReach(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64, amount proofbound.RatInterval) (float64, error) {
	return offset2d.LoopReach(budget, walks, s, t, amount, shellTol)
}
