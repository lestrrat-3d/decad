package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// This file is the exact section offset of docs/modify-design.md §7 and the §5
// audit wrapper for a shell. The offset is per-feature and topology-preserving:
// a line offsets to a parallel line, a circle to a concentric circle, a corner
// closes with a miter or an arc of radius t about the corner point. Every piece
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
func offsetProfile(budget *workBudget, profile ProfileRecord, s, t float64) (ProfileRecord, error) {
	return offsetProfileBudget(budget, profile, s, t)
}

func offsetProfileBudget(budget *workBudget, profile ProfileRecord, s, t float64) (ProfileRecord, error) {
	if err := wallBudgetErr(budget); err != nil {
		return ProfileRecord{}, err
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return ProfileRecord{}, err
	}
	out := make([]LoopRecord, len(loops))
	for i, loop := range loops {
		if err := wallBudgetStep(budget); err != nil {
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
func offsetLoopBudget(budget *workBudget, loop cornerLoop, s, t float64) ([]CurveSegment, error) {
	walks := loop.walks
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: an offset loop holds no walks`, ErrDegenerate)
	}

	// Every circular walk's offset radius must stay positive; a non-positive one
	// is a dropped segment (S11a), caught before any join is computed.
	for _, w := range walks {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.circular {
			if _, ok := offsetRadius(w, s, t); !ok {
				return nil, errOffsetDrop
			}
		}
	}

	// A single closed circle offsets to a concentric circle — no corners.
	if n == 1 && walks[0].closed {
		w := walks[0]
		rr, ok := offsetRadius(w, s, t)
		if !ok {
			return nil, errOffsetDrop
		}
		return []CurveSegment{circleSegConcentric(w.cU, w.cV, rr, w.th1 > w.th0)}, nil
	}

	// Per-corner joins: corner i sits at walk i's start (== walk i−1's end).
	joins := make([]cornerJoin, n)
	for i := range n {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		prev := walks[(i+n-1)%n]
		cur := walks[i]
		vU, vV := cur.startU, cur.startV
		aox, aoy, la := normalize2(prev.tanOutU, prev.tanOutV)
		bix, biy, lb := normalize2(cur.tanInU, cur.tanInV)
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
		// miter corner: intersect the two offset carriers, nearest the corner.
		offA := offsetCarrier(prev, s, t)
		offB := offsetCarrier(cur, s, t)
		mx, my, err := intersectOffsets(offA, offB, vU, vV)
		if err != nil {
			return nil, errOffsetTopology
		}
		joins[i] = cornerJoin{vU: vU, vV: vV, m: Point2{U: mx, V: my}}
	}

	// Emit each walk's offset segment trimmed to the joins at its two ends, then
	// the arc that closes the following corner.
	var segs []CurveSegment
	for i := range n {
		if err := wallBudgetStep(budget); err != nil {
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

// cornerJoin is one corner's resolved offset join: a miter point m, or an arc
// of radius t about (vU, vV) from pA (the arriving walk's offset end) to pB (the
// leaving walk's offset start).
type cornerJoin struct {
	arc    bool
	vU, vV float64
	m      Point2
	pA, pB Point2
}

// offsetRadius is a circular walk's offset radius, R − s·insideSign·t: R − t
// where the material is inside the circle (a CCW walk) and R + t where it is
// outside (a CW walk — a hole wall, a concave round), inward; the signs move the
// other way outward. ok is false when the radius reaches zero or goes negative —
// the segment drops (S11a).
func offsetRadius(w sideWalk, s, t float64) (float64, bool) {
	inside := 1.0
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr := w.radius - s*inside*t
	if rr <= shellTol*math.Max(1, w.radius) {
		return 0, false
	}
	return rr, true
}

// offsetCarrier builds a walk's offset carrier for a miter intersection: an
// offset line (a point on it plus the walk's unit tangent) or a concentric
// circle. It reuses the fillet's offCurve so the closed-form intersectOffsets
// serves both ops.
func offsetCarrier(w sideWalk, s, t float64) offCurve {
	if !w.circular {
		tx, ty, _ := normalize2(w.tanInU, w.tanInV)
		return offCurve{isLine: true, px: w.startU + s*t*(-ty), py: w.startV + s*t*tx, dx: tx, dy: ty}
	}
	rr, _ := offsetRadius(w, s, t) // positivity already gated in offsetLoop
	return offCurve{cx: w.cU, cy: w.cV, rr: rr}
}

// offsetWalkSegment re-emits a walk's offset curve trimmed to (start, end): a
// LineSeg for a straight walk, a concentric ArcSeg in the walk's own sense for a
// circular one. start and end lie on the offset curve by construction, so the
// emitted radius is the offset radius exactly.
func offsetWalkSegment(w sideWalk, s, t float64, start, end Point2) (CurveSegment, error) {
	if !w.circular {
		return LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}, nil
	}
	if _, ok := offsetRadius(w, s, t); !ok {
		return nil, errOffsetDrop
	}
	return arcSegment(Point2{U: w.cU, V: w.cV}, start, end, w.th1 > w.th0), nil
}

// walkOffsetConsumed reports whether the offset dropped this walk (S11a): its
// trimmed offset segment, running from start (the join at its head) to end (the
// join at its tail), no longer advances along the walk's own direction. When a
// loop's erosion is empty — a hole narrower than 2t offset outward — every corner
// join overshoots and each straight walk's offset segment runs BACKWARD; an arc
// walk's offset sweeps the long way round, past its own span. The offset loop
// keeps its walk sense (its signed area does not change sign), so S8 cannot see
// this — the drop is a per-walk fact, decided here as the offset is built.
func walkOffsetConsumed(w sideWalk, start, end Point2) bool {
	du, dv := end.U-start.U, end.V-start.V
	if !w.circular {
		tx, ty, l := normalize2(w.tanInU, w.tanInV)
		if l == 0 {
			return false // no direction to compare against; left to other gates
		}
		adv := du*tx + dv*ty // signed advance along the walk's tangent
		return adv <= shellTol*math.Max(1, math.Hypot(du, dv))
	}
	// An arc's offset must not sweep further than the arc it descends from: a
	// consumed arc's trimmed feet swap sides, so its walk-sense sweep wraps the
	// long way round, exceeding the original span.
	a0 := math.Atan2(start.V-w.cV, start.U-w.cU)
	a1 := math.Atan2(end.V-w.cV, end.U-w.cU)
	span := a1 - a0
	if w.th1 > w.th0 { // CCW walk
		for span < 0 {
			span += 2 * math.Pi
		}
	} else { // CW walk
		for span > 0 {
			span -= 2 * math.Pi
		}
		span = -span
	}
	return span > math.Abs(w.th1-w.th0)+shellTol
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

func reverseLoopRecordBudget(budget *workBudget, l LoopRecord) (LoopRecord, error) {
	n := len(l.Segments)
	walks := make([]segmentWalk, n)
	for i, seg := range l.Segments {
		if err := wallBudgetStep(budget); err != nil {
			return LoopRecord{}, err
		}
		w, err := walkOf(seg)
		if err != nil {
			return LoopRecord{}, err
		}
		walks[i] = w
	}
	segs := make([]CurveSegment, 0, n)
	for i := n - 1; i >= 0; i-- {
		if err := wallBudgetStep(budget); err != nil {
			return LoopRecord{}, err
		}
		w := walks[i]
		switch {
		case w.closed:
			segs = append(segs, circleSegConcentric(w.cU, w.cV, w.radius, !(w.th1 > w.th0)))
		case w.circular:
			segs = append(segs, arcSegment(Point2{U: w.cU, V: w.cV}, Point2{U: w.endU, V: w.endV}, Point2{U: w.startU, V: w.startV}, !(w.th1 > w.th0)))
		default:
			segs = append(segs, LineSeg{Start: Point2{U: w.endU, V: w.endV}, End: Point2{U: w.startU, V: w.startV}, TStart: 0, TEnd: 1})
		}
	}
	return LoopRecord{Segments: segs}, nil
}

// auditOffsetSection runs the shared §5 audit (fillet_audit.go) on a shell's
// offset section, in §4's order: S8 (orientation — an offset loop turned inside
// out is ErrDegenerate), the crossing test (a crossing or boundary contact of
// offset loops — S11b, ErrUnsupported), then S9 (nesting). A shell mints no
// cutback, so the empty fillet map makes the S6 trim test a no-op — the one test
// that cannot fire on an offset (§8).
func auditOffsetSection(orig, offset ProfileRecord) error {
	return auditOffsetSectionBudget(nil, orig, offset)
}

func auditOffsetSectionBudget(budget *workBudget, orig, offset ProfileRecord) error {
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
