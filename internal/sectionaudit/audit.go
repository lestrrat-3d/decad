// Package sectionaudit checks the area, separation, and nesting of rewritten
// section boundaries before a body is built.
package sectionaudit

import (
	"fmt"
	"math"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

type (
	CurveSegment = sectionrecord.CurveSegment
	LoopRecord   = sectionrecord.LoopRecord
	Point2       = sectionrecord.Point2
	LineSeg      = sectionrecord.LineSeg
	CircleSeg    = sectionrecord.CircleSeg
	ArcSeg       = sectionrecord.ArcSeg
)

var (
	ErrUnitKind    = decaderr.ErrUnitKind
	ErrNotFinite   = decaderr.ErrNotFinite
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrUnsupported = decaderr.ErrUnsupported
)

// Tolerance absorbs floating-point noise in the section rewrite.
const Tolerance = 1e-9

// Entry identifies one boundary walk and its position in a loop.
type Entry struct {
	loop int
	idx  int
	n    int
	w    survey2d.SegmentWalk
}

// NewEntry records one walk and its loop position.
func NewEntry(loop, idx, n int, walk survey2d.SegmentWalk) Entry {
	return Entry{loop: loop, idx: idx, n: n, w: walk}
}

// DiagnosticError preserves the shared audit error and its coordinate detail.
type DiagnosticError struct {
	legacy   error
	detailed string
}

func (e *DiagnosticError) Error() string    { return e.legacy.Error() }
func (e *DiagnosticError) Unwrap() error    { return e.legacy }
func (e *DiagnosticError) Detailed() string { return e.detailed }

// NewError keeps the legacy error while attaching a detailed rendering.
func NewError(legacy error, detailed string) error {
	return &DiagnosticError{legacy: legacy, detailed: detailed}
}

func auditError(legacy error, detailed string) error { return NewError(legacy, detailed) }

func renderCoord(c float64) string {
	if c == 0 {
		c = 0
	}
	return strconv.FormatFloat(c, 'g', -1, 64)
}

func shiftPoint(point Point2) Point2 {
	return Point2{U: point.U - 0, V: point.V - 0}
}

func lerp2(start, end Point2, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}

// LoopSignedArea is one loop's signed area (positive counter-clockwise): the
// Green's-theorem boundary integral of its own segments.
func LoopSignedArea(budget *proofbound.WorkBudget, loop LoopRecord) (float64, error) {
	var area float64
	for _, seg := range loop.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, err
		}
		term, err := analyticSignedArea(seg)
		if err != nil {
			return 0, err
		}
		area += term
	}
	return area, nil
}

// analyticSignedArea evaluates only the area term of the same Green's-theorem
// integral used by regionIntegrals.addAnalytic. Keep its float operations in
// the same order: S8 compares the held signed areas, including near zero.
func analyticSignedArea(segment CurveSegment) (float64, error) {
	segment, err := sectionrecord.NormalizeSegment(segment)
	if err != nil {
		return 0, err
	}
	switch segment := segment.(type) {
	case LineSeg:
		start := shiftPoint(segment.Start)
		end := shiftPoint(segment.End)
		u0, v0 := lerp2(start, end, segment.TStart)
		u1, v1 := lerp2(start, end, segment.TEnd)
		return 0.5 * (u0*v1 - u1*v0), nil
	case CircleSeg:
		if segment.Radius.Kind() != units.Length {
			return 0, fmt.Errorf(`%w: a circle segment's radius must be a %s, got %s`,
				ErrUnitKind, units.Length, segment.Radius.Kind())
		}
		radius, err := segment.Radius.In(units.Millimeter)
		if err != nil {
			return 0, fmt.Errorf(`%w: a circle segment's radius is not representable: %s`, ErrNotFinite, err)
		}
		if segment.CCW != (segment.TStart < segment.TEnd) {
			return 0, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		return circularSignedArea(shiftPoint(segment.Center), radius,
			2*math.Pi*segment.TStart, 2*math.Pi*segment.TEnd), nil
	case ArcSeg:
		center := shiftPoint(segment.Center)
		start := shiftPoint(segment.Start)
		end := shiftPoint(segment.End)
		radius := math.Hypot(start.U-center.U, start.V-center.V)
		a0 := math.Atan2(start.V-center.V, start.U-center.U)
		a1 := math.Atan2(end.V-center.V, end.U-center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		return circularSignedArea(center, radius,
			a0+segment.TStart*sweep, a0+segment.TEnd*sweep), nil
	default:
		return 0, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc, circle and Tier A free-form profile segments only; the profile has a %T segment`,
			ErrUnsupported, segment)
	}
}

func circularSignedArea(center Point2, radius, start, end float64) float64 {
	sin0, cos0 := math.Sincos(start)
	sin1, cos1 := math.Sincos(end)
	delta := end - start
	return 0.5 * (radius*radius*delta + center.U*radius*(sin1-sin0) - center.V*radius*(cos1-cos0))
}

// Crossing tests every pair of segments for an interior crossing OR a
// boundary contact — a tangency or a shared boundary point — skipping only
// pairs that legitimately share an endpoint (same-loop neighbours). A crossing
// and a contact both stop two Jordan loops being disjoint, so both must be
// rejected before S9's nesting can be tested: S9 classifies a point of one loop
// against another, and that classification is only defined once the loops are
// strictly disjoint. An interior-only crossing test would pass a large fillet
// whose rewritten loops merely pinch, and Body.Tessellate would then refuse the
// very body Fillet returned. Both are S7 (ErrUnsupported): the touch is the
// boundary case of a crossing, a body a resolving/trimmed-offset kernel could
// build but this evaluator cannot (§1 existence test — the body exists; §4
// Table S, S7).
func Crossing(budget *proofbound.WorkBudget, segs []Entry) error {
	touchFloor, err := ContactFloor(budget, segs)
	if err != nil {
		return err
	}
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return err
			}
			if adjacent(segs[i], segs[j]) {
				continue
			}
			if segCross(segs[i].w, segs[j].w) {
				legacy := fmt.Errorf(`%w: the rewrite crosses itself; a resolving kernel is not available`, ErrUnsupported)
				detailed := fmt.Sprintf(`%v: rewritten loop %d segment %d and loop %d segment %d cross; a resolving kernel is not available`,
					ErrUnsupported, segs[i].loop, segs[i].idx, segs[j].loop, segs[j].idx)
				return auditError(legacy, detailed)
			}
			if segMinDist(segs[i].w, segs[j].w) <= touchFloor {
				legacy := fmt.Errorf(`%w: the rewrite brings two boundaries into contact; a resolving kernel is not available`, ErrUnsupported)
				detailed := fmt.Sprintf(`%v: rewritten loop %d segment %d and loop %d segment %d are in contact; a resolving kernel is not available`,
					ErrUnsupported, segs[i].loop, segs[i].idx, segs[j].loop, segs[j].idx)
				return auditError(legacy, detailed)
			}
		}
	}
	return nil
}

// ContactEps is the noise-floor coefficient ε of the boundary-contact test,
// the SAME ε verification design §4 fixes for its diameter-anchored noise floor
// δ = ε·D (see appendClearance, the Clearance.Gap gate). It is not a
// distance; ContactFloor multiplies it by the section's own scale.
const ContactEps = 1e-9

const contactEps = ContactEps

// ContactFloor is the boundary-contact threshold for the §5 audit, anchored to
// the SECTION'S scale exactly as verification design §4 anchors a length's
// noise floor: δ = ε·D with ε = contactEps and D the section's diameter (its
// (u, v) bounding-box diagonal — the standard decad reading of D, as in
// export.STL and the boolean chord tolerance). Below δ two boundaries are
// indistinguishable from a pinch, so the test REFUSES; comfortably above it is
// a real positive gap that builds. The threshold is reject-only and SCALES with
// the section — a fixed absolute band mis-scales, rejecting a macroscopic gap
// on a sub-millimetre section and accepting a real pinch on a huge one.
func ContactFloor(budget *proofbound.WorkBudget, segs []Entry) (float64, error) {
	minU, minV, maxU, maxV, ok, err := sectionBBoxBudget(budget, segs)
	if err != nil {
		return 0, err
	}
	if !ok {
		return 0, nil
	}
	return contactEps * math.Hypot(maxU-minU, maxV-minV), nil
}

// sectionBBoxBudget is the TRUE (u, v) bounding box of a rewritten section — the box
// the §5 D reads. An arc bulges outside its endpoint chord (a semicircle from
// (−R,0) to (R,0) reaches (0,R); a full circle's endpoints collapse to a
// point), so a line contributes its two endpoints and an arc its endpoints AND
// every cardinal extremum (cU±R, cV±R) its own angular walk actually reaches —
// never the endpoint box alone, which understates D.
func sectionBBoxBudget(budget *proofbound.WorkBudget, segs []Entry) (minU, minV, maxU, maxV float64, ok bool, err error) {
	minU, minV = math.Inf(1), math.Inf(1)
	maxU, maxV = math.Inf(-1), math.Inf(-1)
	fold := func(x, y float64) {
		minU, maxU = math.Min(minU, x), math.Max(maxU, x)
		minV, maxV = math.Min(minV, y), math.Max(maxV, y)
	}
	for _, s := range segs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, 0, 0, 0, false, err
		}
		w := s.w
		fold(w.StartU, w.StartV)
		fold(w.EndU, w.EndV)
		if !w.IsCircular() {
			continue
		}
		lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
		for q := range 4 { // the four cardinal bearings 0, π/2, π, 3π/2
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, 0, 0, 0, false, err
			}
			base := float64(q) * (math.Pi / 2)
			th := base + 2*math.Pi*math.Ceil((lo-base)/(2*math.Pi))
			for ; th <= hi+1e-12; th += 2 * math.Pi {
				fold(w.CU+w.Radius*math.Cos(th), w.CV+w.Radius*math.Sin(th))
			}
		}
	}
	if math.IsInf(minU, 1) {
		return 0, 0, 0, 0, false, nil
	}
	return minU, minV, maxU, maxV, true, nil
}

// Nesting is the §5 test-4 containment audit (S9): with S7 having proven
// the rewritten loops strictly disjoint, each hole lies wholly inside or wholly
// outside the outer loop and every other hole. It classifies one point of each
// hole against the outer loop's boundary and against every other hole's, using
// the ray-parity walk with direction retries internal/survey2d/wall_kernel.go already runs
// (loopContains, over survey2d.RayCrossings). The audit passes only when the outer loop
// is PROVEN to contain each hole and the holes are proven mutually exterior.
//
// An undecided classification is S9 ErrUnsupported — a build-time audit has no
// Suspect to fall back on, so the evaluator declines rather than guess. A hole
// PROVEN outside the outer loop, or PROVEN inside another hole, is nesting
// decidably broken: the fillet consumed the region the caller's nested section
// lived in, so no such body exists (§1 existence test) — an S8-family
// ErrDegenerate, the same "modification consumed the region" verdict S8 gives an
// inverted loop.
func Nesting(budget *proofbound.WorkBudget, segs []Entry, nLoops int) error {
	if nLoops <= 1 { // no holes: nothing to contain
		return nil
	}
	bounds := make([][]survey2d.SurveyElem, nLoops)
	pts := make([][2]float64, nLoops)
	hasPt := make([]bool, nLoops)
	for _, s := range segs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		if e, ok := elemOf(s.w); ok {
			bounds[s.loop] = append(bounds[s.loop], e)
		}
		if !hasPt[s.loop] {
			pts[s.loop] = [2]float64{s.w.StartU, s.w.StartV}
			hasPt[s.loop] = true
		}
	}
	minU, minV, maxU, maxV, ok, err := sectionBBoxBudget(budget, segs)
	if err != nil {
		return err
	}
	if !ok {
		return nil
	}
	scale := math.Max(1, math.Max(math.Max(math.Abs(minU), math.Abs(maxU)), math.Max(math.Abs(minV), math.Abs(maxV))))
	tol := contactEps * scale

	undecidable := func(container, pointLoop int, point [2]float64) error {
		legacy := fmt.Errorf(`%w: the rewrite's nesting cannot be decided; a resolving kernel is not available`, ErrUnsupported)
		detailed := fmt.Sprintf(`%v: nesting of loop %d and loop %d at (u, v) = (%s, %s) cannot be decided; a resolving kernel is not available`,
			ErrUnsupported, container, pointLoop, renderCoord(point[0]), renderCoord(point[1]))
		return auditError(legacy, detailed)
	}
	// Every hole must sit inside the rewritten outer loop.
	for h := 1; h < nLoops; h++ {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		if !hasPt[h] {
			continue
		}
		inside, decided, err := loopContains(budget, bounds[0], pts[h][0], pts[h][1], tol)
		if err != nil {
			return err
		}
		if !decided {
			return undecidable(0, h, pts[h])
		}
		if !inside {
			legacy := fmt.Errorf(`%w: the rewrite left a hole outside the outer loop, consuming the region it lived in`, ErrDegenerate)
			detailed := fmt.Sprintf(`%v: hole loop %d at (u, v) = (%s, %s) lies outside outer loop 0, consuming the region it lived in`,
				ErrDegenerate, h, renderCoord(pts[h][0]), renderCoord(pts[h][1]))
			return auditError(legacy, detailed)
		}
	}
	// Holes must stay mutually exterior — neither nested in the other.
	for a := 1; a < nLoops; a++ {
		for b := a + 1; b < nLoops; b++ {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return err
			}
			if !hasPt[a] || !hasPt[b] {
				continue
			}
			for _, pr := range [2]struct {
				in int
				pt [2]float64
			}{{a, pts[b]}, {b, pts[a]}} {
				inside, decided, err := loopContains(budget, bounds[pr.in], pr.pt[0], pr.pt[1], tol)
				if err != nil {
					return err
				}
				if !decided {
					pointLoop := a
					if pr.in == a {
						pointLoop = b
					}
					return undecidable(pr.in, pointLoop, pr.pt)
				}
				if inside {
					pointLoop := a
					if pr.in == a {
						pointLoop = b
					}
					legacy := fmt.Errorf(`%w: the rewrite nested one hole inside another`, ErrDegenerate)
					detailed := fmt.Sprintf(`%v: hole loop %d at (u, v) = (%s, %s) lies inside hole loop %d`,
						ErrDegenerate, pointLoop, renderCoord(pr.pt[0]), renderCoord(pr.pt[1]), pr.in)
					return auditError(legacy, detailed)
				}
			}
		}
	}
	return nil
}

// elemOf converts a segment walk into a survey2d boundary element. It is
// walkElem under this file's own name; the conversion lives in one place so a
// new walk kind is decided once (survey.go).
func elemOf(w survey2d.SegmentWalk) (survey2d.SurveyElem, bool) { return survey2d.WalkElem(w) }

// loopContains is the named boundary-scan phase used by cancellation probes.
func loopContains(budget *proofbound.WorkBudget, boundary []survey2d.SurveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return false, false, err
	}
	return loopContainsBudget(budget, boundary, px, py, tol)
}

// loopContainsBudget classifies (px, py) against a loop's boundary by crossing parity
// of a ray, retried across the golden-angle direction sequence when a crossing
// is ambiguous — the same walk survey2d.WallKernel.contains runs. decided is false when
// every direction is ambiguous; the answer is never guessed.
func loopContainsBudget(budget *proofbound.WorkBudget, boundary []survey2d.SurveyElem, px, py, tol float64) (inside, decided bool, err error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return false, false, err
	}
	for i := range 16 {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, good := 0, true
		for _, e := range boundary {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, ok := survey2d.RayCrossings(e, px, py, dx, dy, tol)
			if !ok {
				good = false
				break
			}
			crossings += n
		}
		if good {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}

// adjacent reports whether two segments legitimately share an endpoint: the
// same loop and cyclically consecutive positions.
func adjacent(a, b Entry) bool {
	if a.loop != b.loop {
		return false
	}
	d := a.idx - b.idx
	if d < 0 {
		d = -d
	}
	return d == 1 || d == a.n-1
}

// segCrossEps is the interior margin for the crossing test: an intersection
// within it of a segment's endpoint is a shared vertex, not a crossing.
const segCrossEps = 1e-7

// segCross reports whether two segment primitives meet in both their interiors.
func segCross(a, b survey2d.SegmentWalk) bool {
	switch {
	case a.IsLine() && b.IsLine():
		return lineLineSegCross(a, b)
	case a.IsCircular() && b.IsCircular():
		return arcArcSegCross(a, b)
	case a.IsCircular():
		return lineArcSegCross(b, a)
	default:
		return lineArcSegCross(a, b)
	}
}

// lineLineSegCross reports an interior crossing of two line segments.
func lineLineSegCross(a, b survey2d.SegmentWalk) bool {
	rx, ry := a.EndU-a.StartU, a.EndV-a.StartV
	sx, sy := b.EndU-b.StartU, b.EndV-b.StartV
	den := rx*sy - ry*sx
	if math.Abs(den) <= Tolerance {
		return false // parallel: no transversal crossing
	}
	qpx, qpy := b.StartU-a.StartU, b.StartV-a.StartV
	t := (qpx*sy - qpy*sx) / den
	u := (qpx*ry - qpy*rx) / den
	return interior(t) && interior(u)
}

// lineArcSegCross reports an interior crossing of a line segment and an arc.
func lineArcSegCross(line, arc survey2d.SegmentWalk) bool {
	dx, dy := line.EndU-line.StartU, line.EndV-line.StartV
	dl := math.Hypot(dx, dy)
	if dl <= Tolerance {
		return false
	}
	ux, uy := dx/dl, dy/dl
	fx, fy := line.StartU-arc.CU, line.StartV-arc.CV
	bb := fx*ux + fy*uy
	cc := fx*fx + fy*fy - arc.Radius*arc.Radius
	disc := bb*bb - cc
	if disc < 0 {
		return false
	}
	sq := math.Sqrt(disc)
	for _, s := range []float64{-bb + sq, -bb - sq} {
		x, y := line.StartU+s*ux, line.StartV+s*uy
		t := s / dl
		if !interior(t) {
			continue
		}
		if angleInterior(arc, x, y) {
			return true
		}
	}
	return false
}

// arcArcSegCross reports an interior crossing of two arcs.
func arcArcSegCross(a, b survey2d.SegmentWalk) bool {
	pts := CircleCircle(a.CU, a.CV, a.Radius, b.CU, b.CV, b.Radius)
	for _, p := range pts {
		if angleInterior(a, p[0], p[1]) && angleInterior(b, p[0], p[1]) {
			return true
		}
	}
	return false
}

// interior reports whether a segment parameter is strictly inside (0, 1).
func interior(t float64) bool { return t > segCrossEps && t < 1-segCrossEps }

// angleInterior reports whether the point (x, y) lies strictly inside the arc's
// angular walk range.
func angleInterior(arc survey2d.SegmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.Th0, arc.Th1), math.Max(arc.Th0, arc.Th1)
	a := math.Atan2(y-arc.CV, x-arc.CU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th > lo+segCrossEps && th < hi-segCrossEps {
			return true
		}
	}
	return false
}

// angleWithin reports whether (x, y) lies within the arc's angular walk range,
// inclusive of its endpoints — the membership the minimum-distance candidates
// need (a nearest point may sit at an arc's own end).
func angleWithin(arc survey2d.SegmentWalk, x, y float64) bool {
	lo, hi := math.Min(arc.Th0, arc.Th1), math.Max(arc.Th0, arc.Th1)
	a := math.Atan2(y-arc.CV, x-arc.CU)
	for k := math.Floor((lo-a)/(2*math.Pi)) * 2 * math.Pi; a+k <= hi+segCrossEps; k += 2 * math.Pi {
		th := a + k
		if th >= lo-segCrossEps && th <= hi+segCrossEps {
			return true
		}
	}
	return false
}

// segMinDist is the minimum Euclidean distance between two closed segment
// primitives (line or arc), in closed form over their own line and arc data. It
// is the boundary-contact classifier behind S7: a value at or below the
// section-scaled ContactFloor is a tangency or a shared boundary point the
// interior-only crossing test misses. The candidate set is complete for the attained infimum over line/arc
// boundaries — the four endpoint-against-the-other distances, the interior
// radial/aligned criticals, and zero at any interior intersection.
func segMinDist(a, b survey2d.SegmentWalk) float64 {
	switch {
	case a.IsLine() && b.IsLine():
		return lineLineMinDist(a, b)
	case a.IsCircular() && b.IsCircular():
		return arcArcMinDist(a, b)
	case a.IsCircular():
		return lineArcMinDist(b, a)
	default:
		return lineArcMinDist(a, b)
	}
}

// pointLineSegDist is the distance from (px, py) to the closed line segment.
func pointLineSegDist(px, py float64, l survey2d.SegmentWalk) float64 {
	dx, dy := l.EndU-l.StartU, l.EndV-l.StartV
	l2 := dx*dx + dy*dy
	if l2 <= Tolerance*Tolerance {
		return math.Hypot(px-l.StartU, py-l.StartV)
	}
	t := ((px-l.StartU)*dx + (py-l.StartV)*dy) / l2
	t = math.Max(0, math.Min(1, t))
	return math.Hypot(px-(l.StartU+t*dx), py-(l.StartV+t*dy))
}

// pointArcDist is the distance from (px, py) to the closed arc: the radial gap
// when the point's bearing falls within the walk range, else the nearer of the
// two arc endpoints.
func pointArcDist(px, py float64, a survey2d.SegmentWalk) float64 {
	best := math.Min(math.Hypot(px-a.StartU, py-a.StartV), math.Hypot(px-a.EndU, py-a.EndV))
	dc := math.Hypot(px-a.CU, py-a.CV)
	if dc > Tolerance && angleWithin(a, px, py) {
		best = math.Min(best, math.Abs(dc-a.Radius))
	}
	return best
}

// lineLineMinDist is the minimum distance between two closed line segments:
// zero where their interiors cross, else the nearest endpoint-to-segment reach.
func lineLineMinDist(a, b survey2d.SegmentWalk) float64 {
	if lineLineSegCross(a, b) {
		return 0
	}
	return math.Min(
		math.Min(pointLineSegDist(a.StartU, a.StartV, b), pointLineSegDist(a.EndU, a.EndV, b)),
		math.Min(pointLineSegDist(b.StartU, b.StartV, a), pointLineSegDist(b.EndU, b.EndV, a)),
	)
}

// lineArcMinDist is the minimum distance between a closed line segment and a
// closed arc.
func lineArcMinDist(line, arc survey2d.SegmentWalk) float64 {
	if lineArcSegCross(line, arc) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(line.StartU, line.StartV, arc), pointArcDist(line.EndU, line.EndV, arc)),
		math.Min(pointLineSegDist(arc.StartU, arc.StartV, line), pointLineSegDist(arc.EndU, arc.EndV, line)),
	)
	// Interior critical: the arc point along the perpendicular from the centre
	// to the line, when its foot is interior to the segment and its bearing is
	// within the walk — the tangency/near-approach the endpoints miss.
	dx, dy := line.EndU-line.StartU, line.EndV-line.StartV
	l2 := dx*dx + dy*dy
	if l2 > Tolerance*Tolerance {
		s := ((arc.CU-line.StartU)*dx + (arc.CV-line.StartV)*dy) / l2
		if s > 0 && s < 1 {
			fx, fy := line.StartU+s*dx, line.StartV+s*dy
			ux, uy := fx-arc.CU, fy-arc.CV
			ul := math.Hypot(ux, uy)
			if ul > Tolerance {
				cpx, cpy := arc.CU+arc.Radius*ux/ul, arc.CV+arc.Radius*uy/ul
				if angleWithin(arc, cpx, cpy) {
					best = math.Min(best, pointLineSegDist(cpx, cpy, line))
				}
			}
		}
	}
	return best
}

// arcArcMinDist is the minimum distance between two closed arcs.
func arcArcMinDist(a, b survey2d.SegmentWalk) float64 {
	if arcArcSegCross(a, b) {
		return 0
	}
	best := math.Min(
		math.Min(pointArcDist(a.StartU, a.StartV, b), pointArcDist(a.EndU, a.EndV, b)),
		math.Min(pointArcDist(b.StartU, b.StartV, a), pointArcDist(b.EndU, b.EndV, a)),
	)
	dcx, dcy := b.CU-a.CU, b.CV-a.CV
	d := math.Hypot(dcx, dcy)
	if d > Tolerance {
		ux, uy := dcx/d, dcy/d
		// The mutually nearest points of two non-concentric circles lie on the
		// centre line; test each circle's two axis points against the other arc,
		// which captures external and internal tangency alike.
		for _, s := range []float64{1, -1} {
			pax, pay := a.CU+s*a.Radius*ux, a.CV+s*a.Radius*uy
			if angleWithin(a, pax, pay) {
				best = math.Min(best, pointArcDist(pax, pay, b))
			}
			pbx, pby := b.CU+s*b.Radius*ux, b.CV+s*b.Radius*uy
			if angleWithin(b, pbx, pby) {
				best = math.Min(best, pointArcDist(pbx, pby, a))
			}
		}
	} else if arcSpansOverlap(a, b) {
		// Concentric arcs whose walks overlap in bearing: the radial gap.
		best = math.Min(best, math.Abs(a.Radius-b.Radius))
	}
	return best
}

// arcSpansOverlap reports whether two concentric arcs' walk ranges share any
// bearing — an endpoint of one falling within the other's range.
func arcSpansOverlap(a, b survey2d.SegmentWalk) bool {
	return angleWithin(b, a.StartU, a.StartV) || angleWithin(b, a.EndU, a.EndV) ||
		angleWithin(a, b.StartU, b.StartV) || angleWithin(a, b.EndU, b.EndV)
}

// CircleCircle intersects two circles.
func CircleCircle(c0x, c0y, r0, c1x, c1y, r1 float64) [][2]float64 {
	dx, dy := c1x-c0x, c1y-c0y
	dsq := dx*dx + dy*dy
	d := math.Sqrt(dsq)
	if d <= Tolerance || d > r0+r1+Tolerance || d < math.Abs(r0-r1)-Tolerance {
		return nil
	}
	a := (dsq + r0*r0 - r1*r1) / (2 * d)
	h2 := r0*r0 - a*a
	if h2 < 0 {
		h2 = 0
	}
	h := math.Sqrt(h2)
	mx, my := c0x+a*dx/d, c0y+a*dy/d
	ox, oy := -dy/d*h, dx/d*h
	return [][2]float64{{mx + ox, my + oy}, {mx - ox, my - oy}}
}
