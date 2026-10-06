package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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
	n := len(walks)
	joins := make([]cornerJoin, n)
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		prev := walks[(i+n-1)%n]
		cur := walks[i]
		vU, vV := cur.StartU, cur.StartV
		aox, aoy, la := normalize2(prev.TanOutU, prev.TanOutV)
		bix, biy, lb := normalize2(cur.TanInU, cur.TanInV)
		if la == 0 || lb == 0 {
			return nil, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
		}
		cross := aox*biy - aoy*bix
		// Left normals of the two tangents point into the material (the walk
		// keeps the material on its left); the offset endpoint is t along it.
		pA := Point2{U: vU + s*t*(-aoy), V: vV + s*t*aox}
		pB := Point2{U: vU + s*t*(-biy), V: vV + s*t*bix}
		if math.Abs(cross) > shellTol && (cross > 0) == (s < 0) {
			// arc corner: sign(cross) == −s.
			joins[i] = cornerJoin{arc: true, vU: vU, vV: vV, pA: pA, pB: pB}
			continue
		}
		if math.Abs(cross) <= shellTol && aox*bix+aoy*biy > 0 {
			// G1 join (modify §7's dead-zone rule): the two offset carriers are
			// tangent at the corner moved t along the shared normal, so that point
			// is their one common point. Intersecting them would solve a double
			// root the float discriminant cannot hold at zero (±1e-14 in practice),
			// which is the erratic S11/S11a refusal this branch replaces. A cusp
			// (dot <= 0) stays on the miter row below.
			joins[i] = cornerJoin{g1: true, vU: vU, vV: vV, m: pB}
			continue
		}
		// miter corner: intersect the two offset carriers, nearest the corner.
		offA := offsetCarrier(prev, s, t)
		offB := offsetCarrier(cur, s, t)
		mx, my, err := intersectOffsets(offA, offB, vU, vV)
		if err != nil {
			return nil, errOffsetTopology
		}
		joins[i] = cornerJoin{vU: vU, vV: vV, m: Point2{U: mx, V: my}}
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
	inside := 1.0
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr := w.Radius - s*inside*t
	if rr <= shellTol*math.Max(1, w.Radius) {
		return 0, false
	}
	return rr, true
}

// offsetCarrier builds a walk's offset carrier for a miter intersection: an
// offset line (a point on it plus the walk's unit tangent) or a concentric
// circle. It reuses the fillet's offCurve so the closed-form intersectOffsets
// serves both ops.
func offsetCarrier(w survey2d.SideWalk, s, t float64) offCurve {
	if !w.IsCircular() {
		tx, ty, _ := normalize2(w.TanInU, w.TanInV)
		return offCurve{isLine: true, px: w.StartU + s*t*(-ty), py: w.StartV + s*t*tx, dx: tx, dy: ty}
	}
	rr, _ := offsetRadius(w, s, t) // positivity already gated in offsetLoop
	return offCurve{cx: w.CU, cy: w.CV, rr: rr}
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
	du, dv := end.U-start.U, end.V-start.V
	if !w.IsCircular() {
		tx, ty, l := normalize2(w.TanInU, w.TanInV)
		if l == 0 {
			return false // no direction to compare against; left to other gates
		}
		adv := du*tx + dv*ty // signed advance along the walk's tangent
		return adv <= shellTol*math.Max(1, math.Hypot(du, dv))
	}
	// An arc's offset must not sweep further than the arc it descends from: a
	// consumed arc's trimmed feet swap sides, so its walk-sense sweep wraps the
	// long way round, exceeding the original span.
	a0 := math.Atan2(start.V-w.CV, start.U-w.CU)
	a1 := math.Atan2(end.V-w.CV, end.U-w.CU)
	span := a1 - a0
	if w.Th1 > w.Th0 { // CCW walk
		for span < 0 {
			span += 2 * math.Pi
		}
	} else { // CW walk
		for span > 0 {
			span -= 2 * math.Pi
		}
		span = -span
	}
	// The feet are held to coordinate rounding, so an overshoot is evidence only
	// once it exceeds that rounding AS A LENGTH on the offset circle — the same
	// scale-relative reading offsetRadius and the line branch above take. A
	// consumed arc overshoots by nearly a whole turn minus its own span, so the
	// gate's reject side is untouched; only a G1-joined arc far from the origin,
	// whose feet differ from its own endpoints' radials by ulp(coordinate)/radius,
	// stops reading as consumed.
	overshoot := span - math.Abs(w.Th1-w.Th0)
	rr := math.Hypot(start.U-w.CU, start.V-w.CV)
	return rr*overshoot > shellTol*math.Max(1, math.Abs(w.CU)+math.Abs(w.CV)+2*w.Radius)
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
	work := newFreeformWork()
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
		r, ok := offsetCircleRadius(w, amount)
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
		gap, ok := circularWalkEndGap(w)
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
		end, okE := walkPointEnclosure(prev.EndU, prev.EndV, prev.EndBound)
		start, okS := walkPointEnclosure(cur.StartU, cur.StartV, cur.StartBound)
		if !okE || !okS {
			return 0, errOffsetUnbounded
		}
		corner := ivUnion(end, start)
		a, okA := offsetFootEnclosure(corner, prev, true, amount)
		b, okB := offsetFootEnclosure(corner, cur, false, amount)
		if !okA || !okB {
			return 0, errOffsetUnbounded
		}
		switch {
		case j.arc:
			reach = math.Max(reach, math.Max(a.reach(j.pA), b.reach(j.pB)))
		case j.g1:
			reach = math.Max(reach, ivUnion(a, b).reach(j.m))
		default:
			ca, okA := offsetCarrierEnclosure(prev, amount)
			cb, okB := offsetCarrierEnclosure(cur, amount)
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
			reach = math.Max(reach, m.reach(j.m))
		}
	}
	return reach, nil
}

// walkPointEnclosure lifts a walk endpoint to the box its own stated bound
// allows. A recorded endpoint states zero and lifts to its exact point.
func walkPointEnclosure(u, v float64, bound proofbound.WalkEndBound) (ivPoint, bool) {
	p, ok := ivExactPoint(u, v)
	allow := proofbound.WalkEndBoundAllow(bound)
	if !ok || proofbound.IsNonFinite(allow) {
		return ivPoint{}, false
	}
	if allow == 0 {
		return p, true
	}
	ra := proofarith.FloatRat(allow)
	widen := func(c proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.Interval(new(big.Rat).Sub(c.Lo, ra), new(big.Rat).Add(c.Hi, ra))
	}
	return ivPoint{u: widen(p.u), v: widen(p.v)}, true
}

// ivUnitOf encloses the unit vector of every vector its argument encloses.
func ivUnitOf(p ivPoint) (ivPoint, bool) {
	l, ok := survey2d.IntervalSqrt(proofbound.IntervalAdd(survey2d.IntervalSquare(p.u), survey2d.IntervalSquare(p.v)))
	if !ok || l.Lo.Sign() <= 0 {
		return ivPoint{}, false
	}
	u, okU := survey2d.IntervalQuo(p.u, l)
	v, okV := survey2d.IntervalQuo(p.v, l)
	return ivPoint{u: u, v: v}, okU && okV
}

// walkTangentEnclosure encloses a walk's unit travel tangent at one end: a
// line's chord direction, or a circle's radius at that end turned a quarter in
// the walk's sense.
func walkTangentEnclosure(w survey2d.SideWalk, atEnd bool) (ivPoint, bool) {
	start, okS := walkPointEnclosure(w.StartU, w.StartV, w.StartBound)
	end, okE := walkPointEnclosure(w.EndU, w.EndV, w.EndBound)
	if !okS || !okE {
		return ivPoint{}, false
	}
	switch {
	case w.IsLine():
		return ivUnitOf(ivPoint{u: proofbound.IntervalSub(end.u, start.u), v: proofbound.IntervalSub(end.v, start.v)})
	case w.IsCircular():
		c, ok := ivExactPoint(w.CU, w.CV)
		if !ok {
			return ivPoint{}, false
		}
		p := start
		if atEnd {
			p = end
		}
		ru, rv := proofbound.IntervalSub(p.u, c.u), proofbound.IntervalSub(p.v, c.v)
		if w.Th1 > w.Th0 {
			return ivUnitOf(ivPoint{u: proofbound.IntervalNeg(rv), v: ru})
		}
		return ivUnitOf(ivPoint{u: rv, v: proofbound.IntervalNeg(ru)})
	default:
		return ivPoint{}, false
	}
}

// offsetFootEnclosure encloses corner + amount·n̂, n̂ the walk's left unit
// normal at that end — the point the float build spells v + s·t·(−ty, tx).
func offsetFootEnclosure(corner ivPoint, w survey2d.SideWalk, atEnd bool, amount proofbound.RatInterval) (ivPoint, bool) {
	tan, ok := walkTangentEnclosure(w, atEnd)
	if !ok {
		return ivPoint{}, false
	}
	return ivPoint{
		u: proofbound.IntervalAdd(corner.u, proofbound.IntervalMul(amount, proofbound.IntervalNeg(tan.v))),
		v: proofbound.IntervalAdd(corner.v, proofbound.IntervalMul(amount, tan.u)),
	}, true
}

// offsetCarrierEnclosure encloses a walk's offset carrier over every signed
// offset in amount: the line through the start moved along the left normal,
// or the concentric circle (offsetCarrier's two shapes).
func offsetCarrierEnclosure(w survey2d.SideWalk, amount proofbound.RatInterval) (ivCarrier, bool) {
	if w.IsCircular() {
		r, ok := offsetCircleRadius(w, amount)
		c, okC := ivExactPoint(w.CU, w.CV)
		return ivCarrier{c: c, r: r}, ok && okC
	}
	if !w.IsLine() {
		return ivCarrier{}, false
	}
	start, okS := walkPointEnclosure(w.StartU, w.StartV, w.StartBound)
	dir, okD := walkTangentEnclosure(w, false)
	if !okS || !okD {
		return ivCarrier{}, false
	}
	p := ivPoint{
		u: proofbound.IntervalAdd(start.u, proofbound.IntervalMul(amount, proofbound.IntervalNeg(dir.v))),
		v: proofbound.IntervalAdd(start.v, proofbound.IntervalMul(amount, dir.u)),
	}
	return ivCarrier{isLine: true, p: p, dir: dir}, true
}

// offsetCircleRadius encloses offsetRadius's R − insideSign·(s·t) over every
// signed offset in amount, R the walk's radius widened by its own bracket.
func offsetCircleRadius(w survey2d.SideWalk, amount proofbound.RatInterval) (proofbound.RatInterval, bool) {
	rr, rb := proofarith.FloatRat(w.Radius), proofarith.FloatRat(w.RadiusBound)
	if rr == nil || rb == nil || rb.Sign() < 0 {
		return proofbound.RatInterval{}, false
	}
	inside := big.NewRat(1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	base := proofbound.Interval(new(big.Rat).Sub(rr, rb), new(big.Rat).Add(rr, rb))
	r := proofbound.IntervalSub(base, proofbound.IntervalScale(amount, inside))
	if r.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, false
	}
	return r, true
}

// circularWalkEndGap bounds how far either end of a circular walk sits off
// the circle its walk radius brackets, measured radially.
func circularWalkEndGap(w survey2d.SideWalk) (float64, bool) {
	c, okC := ivExactPoint(w.CU, w.CV)
	r, okR := offsetCircleRadius(w, proofbound.PointInterval(new(big.Rat)))
	if !okC || !okR {
		return 0, false
	}
	gap := new(big.Rat)
	reach := func(u, v float64, bound proofbound.WalkEndBound) bool {
		p, ok := walkPointEnclosure(u, v, bound)
		if !ok {
			return false
		}
		d, ok := survey2d.IntervalSqrt(proofbound.IntervalAdd(survey2d.IntervalSquare(proofbound.IntervalSub(p.u, c.u)), survey2d.IntervalSquare(proofbound.IntervalSub(p.v, c.v))))
		if !ok {
			return false
		}
		for _, x := range []*big.Rat{new(big.Rat).Sub(d.Hi, r.Lo), new(big.Rat).Sub(r.Hi, d.Lo)} {
			if x.Cmp(gap) > 0 {
				gap = x
			}
		}
		return true
	}
	if !reach(w.StartU, w.StartV, w.StartBound) || !reach(w.EndU, w.EndV, w.EndBound) {
		return 0, false
	}
	return proofbound.RatFloatUp(gap), true
}
