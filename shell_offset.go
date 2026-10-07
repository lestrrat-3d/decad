package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
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
var errOffsetDrop = fmt.Errorf(`%w: the offset drops a section feature; a trimmed-offset kernel is not available`, ErrUnsupported)

// errOffsetTopology marks a corner whose offset carriers do not close into a
// miter — a feature-set change this evaluator cannot resolve (S11).
var errOffsetTopology = fmt.Errorf(`%w: the offset changes the section's topology; a trimmed-offset kernel is not available`, ErrUnsupported)

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

// offsetLoopBudget offsets one coalesced loop by s·t (docs/modify-design.md §7). The
// walk keeps its own sense, so the offset loop's orientation matches the
// original's (the S8 sign check reads that). Each corner closes with a miter
// (offset carriers meet) or an arc of radius t about the corner point; which,
// is decided by the corner turn and the sense: an arc appears exactly when
// sign(cross) == −s — the inward reflex and the outward convex cases (§7).
// Inside the shellTol dead zone (|cross| ≤ shellTol, dot > 0) the corner is a
// G1 join instead, closed at the leaving walk's offset start; the rule decides
// only which closed form is built, and every drop and audit gate still runs on
// the section it builds (modify §7). The cap-chamfer boolean operand's exact
// admission predicate capJoinIsG1 (exact zero cross over the record) implies
// this classification and stays separate from it.
func offsetLoopBudget(budget *proofbound.WorkBudget, loop cornerLoop, s, t float64) ([]CurveSegment, error) {
	walks := loop.walks
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: an offset loop holds no walks`, ErrDegenerate)
	}

	// Every circular walk's offset radius must stay positive; a non-positive one
	// is a dropped segment (S11a), caught before any join is computed.
	for _, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.IsCircular() {
			if _, ok := offsetRadius(w, s, t); !ok {
				return nil, errOffsetDrop
			}
		}
	}

	// A single closed circle offsets to a concentric circle — no corners.
	if n == 1 && walks[0].Closed {
		w := walks[0]
		rr, ok := offsetRadius(w, s, t)
		if !ok {
			return nil, errOffsetDrop
		}
		return []CurveSegment{circleSegConcentric(w.CU, w.CV, rr, w.Th1 > w.Th0)}, nil
	}

	joins, err := offsetJoinsBudget(budget, walks, s, t)
	if err != nil {
		return nil, err
	}

	// Emit each walk's offset segment trimmed to the joins at its two ends, then
	// the arc that closes the following corner.
	var segs []CurveSegment
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		w := walks[i]
		start := joins[i].m
		if joins[i].arc {
			start = joins[i].pB
		}
		j1 := joins[(i+1)%n]
		end := j1.m
		if j1.arc {
			end = j1.pA
		}
		// S11a: a walk the offset has consumed. When a loop's erosion is empty —
		// a hole narrower than 2t offset outward, a slot the offset over-eats —
		// the neighbouring corner joins overshoot the walk and its trimmed offset
		// segment no longer runs along the walk's own direction. offsetRadius
		// catches a circular segment collapsing to zero radius; this catches a
		// polygonal loop the joins turn inside out, which keeps its signed-area
		// sign (so S8 cannot see it) yet bounds no material. Caught here as the
		// offset is built — antecedent to the §5 audit (§4).
		if walkOffsetConsumed(w, start, end) {
			return nil, errOffsetDrop
		}
		seg, err := offsetWalkSegment(w, s, t, start, end)
		if err != nil {
			return nil, err
		}
		segs = append(segs, seg)
		if j1.arc {
			// The arc walks CCW outward (s < 0) and CW inward (s > 0): its tangent
			// continues the walk's travel direction at both feet (§7).
			segs = append(segs, arcSegment(Point2{U: j1.vU, V: j1.vV}, j1.pA, j1.pB, s < 0))
		}
	}
	return segs, nil
}

// offsetJoinsBudget resolves every corner join of one coalesced loop of two or
// more walks, in walk order: corner i sits at walk i's start (== walk i−1's
// end). It is the one place the corner rule of docs/modify-design.md §7 is
// decided, so the offset build (offsetLoopBudget) and its displacement proof
// (offsetSectionDelta) always read the same joins.
func offsetJoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64) ([]cornerJoin, error) {
	result, err := offset2d.JoinsBudget(budget, walks, s, t, shellTol)
	if errors.Is(err, offset2d.ErrNoDirection) {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
	}
	if errors.Is(err, offset2d.ErrNoIntersection) {
		return nil, errOffsetTopology
	}
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

// offsetWalkSegment re-emits a walk's offset curve trimmed to (start, end): a
// LineSeg for a straight walk, a concentric ArcSeg in the walk's own sense for a
// circular one. start and end lie on the offset curve by construction, so the
// emitted radius is the offset radius exactly.
func offsetWalkSegment(w survey2d.SideWalk, s, t float64, start, end Point2) (CurveSegment, error) {
	if !w.IsCircular() {
		return LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}, nil
	}
	if _, ok := offsetRadius(w, s, t); !ok {
		return nil, errOffsetDrop
	}
	return arcSegment(Point2{U: w.CU, V: w.CV}, start, end, w.Th1 > w.Th0), nil
}

// walkOffsetConsumed reports whether the offset dropped this walk (S11a): its
// trimmed offset segment, running from start (the join at its head) to end (the
// join at its tail), no longer advances along the walk's own direction. When a
// loop's erosion is empty — a hole narrower than 2t offset outward — every corner
// join overshoots and each straight walk's offset segment runs BACKWARD; an arc
// walk's offset sweeps the long way round, past its own span. The offset loop
// keeps its walk sense (its signed area does not change sign), so S8 cannot see
// this — the drop is a per-walk fact, decided here as the offset is built. An
// arc's overshoot past its own span is read as a length on the offset circle,
// against shellTol scaled by the walk's coordinate magnitude, so the rounding
// of feet held far from the origin never reads as a sweep past the span.
func walkOffsetConsumed(w survey2d.SideWalk, start, end Point2) bool {
	return offset2d.WalkConsumed(w, offset2d.Point{U: start.U, V: start.V},
		offset2d.Point{U: end.U, V: end.V}, shellTol)
}

// circleSegConcentric records a full-circle walk as a CircleSeg of radius rr in
// the given walk sense.
func circleSegConcentric(cu, cv, rr float64, ccw bool) CurveSegment {
	if ccw {
		return CircleSeg{Center: Point2{U: cu, V: cv}, Radius: units.Millimeters(rr), CCW: true, TStart: 0, TEnd: 1}
	}
	return CircleSeg{Center: Point2{U: cu, V: cv}, Radius: units.Millimeters(rr), CCW: false, TStart: 1, TEnd: 0}
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
		if err := requireAnalyticWalk(w, "the shell section offset"); err != nil {
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
var errOffsetUnbounded = fmt.Errorf(`%w: this evaluator cannot prove how far the shell's offset section sits from the offset it denotes, so the cup's readings would carry no bound`, ErrUnsupported)

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

// offsetLoopReach is the largest reach of one loop's recorded offset points
// from their enclosures (offsetSectionDelta).
func offsetLoopReach(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t float64, amount proofbound.RatInterval) (float64, error) {
	n := len(walks)
	if n == 0 {
		return 0, fmt.Errorf(`%w: an offset loop holds no walks`, ErrDegenerate)
	}
	if n == 1 && walks[0].Closed {
		// A concentric circle: every recorded point sits at the held radius
		// about the exact centre, so the radial gap is the whole displacement.
		w := walks[0]
		held, ok := offsetRadius(w, s, t)
		if !ok {
			return 0, errOffsetDrop
		}
		r, ok := capcontour.OffsetCircleRadius(w, amount)
		if !ok {
			return 0, errOffsetUnbounded
		}
		gap, ok := ivAxisSpread(r, held)
		if !ok {
			return 0, errOffsetUnbounded
		}
		return proofbound.RatFloatUp(gap), nil
	}
	joins, err := offsetJoinsBudget(budget, walks, s, t)
	if err != nil {
		return 0, err
	}
	reach := 0.0
	for _, w := range walks {
		// A recorded arc's end is pinned to its record and may sit off the
		// circle its start fixes; that radial gap moves the denoted foot off
		// the denoted carrier, so it joins the reach the arc argument reads.
		if !w.IsCircular() {
			continue
		}
		gap, ok := capcontour.CircularWalkEndGap(w)
		if !ok {
			return 0, errOffsetUnbounded
		}
		reach = math.Max(reach, gap)
	}
	for i, j := range joins {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		end, okE := capcontour.WalkPointEnclosure(prev.EndU, prev.EndV, prev.EndBound)
		start, okS := capcontour.WalkPointEnclosure(cur.StartU, cur.StartV, cur.StartBound)
		if !okE || !okS {
			return 0, errOffsetUnbounded
		}
		corner := ivUnion(end, start)
		a, okA := capcontour.OffsetFootEnclosure(corner, prev, true, amount)
		b, okB := capcontour.OffsetFootEnclosure(corner, cur, false, amount)
		if !okA || !okB {
			return 0, errOffsetUnbounded
		}
		switch {
		case j.arc:
			reach = math.Max(reach, math.Max(a.Reach(j.pA.U, j.pA.V), b.Reach(j.pB.U, j.pB.V)))
		case j.g1:
			reach = math.Max(reach, ivUnion(a, b).Reach(j.m.U, j.m.V))
		default:
			ca, okA := capcontour.OffsetCarrierEnclosure(prev, amount)
			cb, okB := capcontour.OffsetCarrierEnclosure(cur, amount)
			if !okA || !okB {
				return 0, errOffsetUnbounded
			}
			cands, ok := ivIntersect(ca, cb)
			if !ok {
				return 0, errOffsetUnbounded
			}
			m, ok := ivNearestTo(cands, corner)
			if !ok {
				return 0, errOffsetUnbounded
			}
			reach = math.Max(reach, m.Reach(j.m.U, j.m.V))
		}
	}
	return reach, nil
}
