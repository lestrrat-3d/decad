package decad

import (
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the fillet of docs/modify-design.md: Body.Fillet rounds the
// convex or concave lateral edges of a straight prism (Table R row R1). The
// reduction is §2's — a lateral edge is a CORNER of the recorded 2D section,
// and rounding it is a rewrite of that section into a tangent arc of radius r,
// fed back through evalPrism so mass properties, tessellation and the surveys
// come for free (§10, Table D). The section rewrite is decad's OWN synthesized
// geometry (§5), so decad owns its validity with exact closed-form tests — the
// §5 audit (S8 orientation, S6 self-consuming trim, S7 crossing/contact, S9
// nesting) —
// never a residual. The blend surface is a Cylinder (§6); its wall carries the
// side(i,j) role AND a second fillet(i,j) role naming the same (loop, segment)
// of the rewritten record (Table B, B1).
//
// Fillet rounds lateral edges — line/line, line/arc and arc/arc corners,
// convex and concave — with B1's roles and the Step wiring (modify §13). A
// cap-edge selector is S1 (ErrUnsupported, the vertex blend §6); a non-prism
// receiver is S3 (ErrUnsupported).

// FilletOption configures Fillet. No options are currently supported: a fillet
// Step's Opts is nil (§6), and the option group exists so a variable-radius or
// setback option can be added without changing the signature.
type FilletOption interface {
	option.Interface
	filletOption()
}

// filletTol is the closed-form degeneracy tolerance for the section rewrite:
// two directions parallel within it are a smooth or cusped corner (S4), and an
// offset intersection is rejected below it (S5). The section is decad's own
// exact geometry, so this only absorbs float noise, never an admission on a
// residual.
const filletTol = 1e-9

// Fillet rounds the selected lateral edges of a straight prism with a tangent
// arc of radius r, returning the new body and retiring the receiver
// (docs/modify-design.md §6, core §8). sel is resolved against the live
// receiver; a query matching nothing is loud (ErrNoMatch / ErrCardinality,
// S16). r is a length magnitude, gated like every other (S15); a zero r is
// S13. A selected edge that is not a lateral edge — a cap edge — is S1
// (ErrUnsupported), and a receiver whose payload is not a prism is S3
// (ErrUnsupported). The rewritten section faces the §5 audit before anything
// is built, so no unproven body is ever made.
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	// Stage 1 pre-gates (§4): a live receiver (S17), a valid magnitude (S15),
	// a non-zero one (S13), then a selector that matches (S16).
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	rmm, err := magnitudeIn(r, units.Length, units.Millimeter, "the fillet radius")
	if err != nil {
		return nil, err
	}
	if rmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-radius fillet is the body the caller already holds`, ErrDegenerate)
	}
	// decad owns the selector vocabulary, so only the built-in query can be
	// resolved and recorded. Reject foreign implementations before invoking
	// their callback, and treat a typed nil query like an untyped nil.
	q, ok := sel.(*EdgeQuery)
	switch {
	case sel == nil:
		return nil, errNilSelector
	case !ok:
		return nil, fmt.Errorf(`%w: the fillet's edge selector is not a decad edge query (%T)`, ErrDegenerate, sel)
	case q == nil:
		return nil, errNilSelector
	}
	edges, err := q.SelectEdges(b)
	if err != nil {
		return nil, err
	}

	// Stage 2 (§4): the receiver's payload class (S3), then every selected
	// edge is a lateral edge mapped to a section corner (S1).
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator fillets a straight prism only`, ErrUnsupported)
	}

	loops, err := prismCornerLoops(pp)
	if err != nil {
		return nil, err
	}
	// Which corners each loop gets a fillet at, and the fillet's blend data.
	blendAt := make([]map[int]*cornerBlend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*cornerBlend{}
	}
	matched := make([]matchedCorner, 0, len(edges))
	for ei, e := range edges {
		li, ci, found := matchCorner(pp, loops, e)
		if !found {
			return nil, fmt.Errorf(`selector %s, %s: %w: a fillet of a cap edge is the vertex-blend problem, not yet supported`,
				sel, selectedEdgeContext(ei, e), ErrUnsupported)
		}
		matched = append(matched, newMatchedCorner(ei, e, li, ci, loops[li]))
		blendAt[li][ci] = nil // marked; the blend is computed in stage 3
	}

	// Stage 3 (§4): the construction's own gates, per corner — S4 (a corner
	// exists) then S5 (a blend of that radius exists).
	for _, corner := range matched {
		cb, err := computeFillet(loops[corner.loop], corner.corner, rmm)
		if err != nil {
			return nil, fmt.Errorf(`selector %s, %s: %w`, sel, corner, err)
		}
		blendAt[corner.loop][corner.corner] = cb
	}

	// The rewritten section, and the fillet arcs' (loop, segment) indices.
	profile, filletArcs := rewriteProfile(pp.profile, loops, blendAt)

	// Stage 4 (§4/§5): the audit of the rewritten profile — S8, S6, S7, S9.
	if err := auditRewrite(pp.profile, profile, loops, blendAt); err != nil {
		return nil, wrapModifyAuditError(sel, matched, err)
	}

	// Build through evalPrism (§2): same frame, interval and placement, only
	// the section changed.
	step := Step{
		Op:        OpFillet,
		Inputs:    []StepRef{b.originStep()},
		Selectors: cloneSelectors([]Selector{q}),
		Values:    []units.Value{r},
	}
	ref := d.nextStepRef()
	// The blend descriptors ride on the payload so a re-evaluation (a copy or a
	// placement) re-mints its own fillet(i,j) roles; evalPrism applies them.
	body, err := evalPrism(d, ref, prismPayload{
		profile:   profile,
		frame:     pp.frame,
		z0:        pp.z0,
		z1:        pp.z1,
		xform:     pp.xform,
		blendSegs: filletArcs,
		blendKind: "fillet",
	})
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with recipe liveness at the commit edge.
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	d.commit(step, body, b)
	return body, nil
}

// cornerLoop is one boundary loop resolved into its coalesced corner walk, the
// same decomposition buildLoopSides and the surveys use: a lateral edge is a
// junction between two consecutive walks.
type cornerLoop struct {
	walks []sideWalk
}

// matchedCorner retains one selector result beside the section coordinate it
// resolved to. The selected-edge ordinal follows SelectEdges' stable result
// order; the loop/corner coordinate and plane-local point identify the same
// junction in the recorded section.
type matchedCorner struct {
	edgeOrdinal int
	edge        *Edge
	loop        int
	corner      int
	point       Point2
}

func newMatchedCorner(edgeOrdinal int, edge *Edge, loop, corner int, cl cornerLoop) matchedCorner {
	w := cl.walks[corner]
	return matchedCorner{
		edgeOrdinal: edgeOrdinal,
		edge:        edge,
		loop:        loop,
		corner:      corner,
		point:       Point2{U: w.startU, V: w.startV},
	}
}

func (m matchedCorner) String() string {
	return fmt.Sprintf(`%s maps to loop %d corner %d at (u, v) = (%s, %s)`,
		selectedEdgeContext(m.edgeOrdinal, m.edge), m.loop, m.corner,
		renderCoord(m.point.U), renderCoord(m.point.V))
}

func selectedEdgeContext(ordinal int, edge *Edge) string {
	if edge == nil || edge.start == nil || edge.end == nil {
		return fmt.Sprintf(`selected edge[%d]`, ordinal)
	}
	return fmt.Sprintf(`selected edge[%d] from (%s) to (%s)`, ordinal,
		renderVec(edge.start.position), renderVec(edge.end.position))
}

func wrapModifyAuditError(sel EdgeSelector, matched []matchedCorner, err error) error {
	coordinates := make([]string, len(matched))
	for i, corner := range matched {
		coordinates[i] = corner.String()
	}
	err = renderAuditCoordinates(err)
	return fmt.Errorf(`selector %s matched [%s]: %w`, sel, strings.Join(coordinates, `; `), err)
}

// prismCornerLoops resolves every loop of the prism's section into its
// coalesced walks (outer first, then holes), so a lateral edge maps to a
// junction and the rewrite operates at the corner level.
func prismCornerLoops(pp prismPayload) ([]cornerLoop, error) {
	var out []cornerLoop
	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		}
		out = append(out, cornerLoop{walks: coalesceWalks(raw)})
	}
	return out, nil
}

// matchCorner maps a selected edge to a (loop, corner) of the section: a
// lateral edge is the vertical junction at a corner, so its two vertices are
// the corner point lifted to the two caps. A cap edge matches nothing (its
// endpoints share a cap plane), which is exactly S1's honest reading of the
// class.
func matchCorner(pp prismPayload, loops []cornerLoop, e *Edge) (int, int, bool) {
	if _, ok := e.curve.(Line3); !ok {
		return 0, 0, false
	}
	if e.start == nil || e.end == nil {
		return 0, 0, false
	}
	const tol = 1e-6
	for li, loop := range loops {
		n := len(loop.walks)
		if n == 1 && loop.walks[0].closed {
			continue
		}
		for ci, w := range loop.walks {
			j0 := pp.point(w.startU, w.startV, pp.z0)
			j1 := pp.point(w.startU, w.startV, pp.z1)
			if matchEndpoints(e.start.position, e.end.position, j0, j1, tol) {
				return li, ci, true
			}
		}
	}
	return 0, 0, false
}

// matchEndpoints reports whether {a, b} equals {p, q} as an unordered pair
// within tol.
func matchEndpoints(a, b, p, q r3.Vec, tol float64) bool {
	near := func(u, v r3.Vec) bool { return u.Sub(v).Len() <= tol }
	return (near(a, p) && near(b, q)) || (near(a, q) && near(b, p))
}

// cornerBlend is one corner's rewrite — the geometry Fillet and Chamfer share.
// Each op trims the two adjacent walks back to feet fA (on the arriving walk)
// and fB (on the leaving walk), losing cutbackA / cutbackB of arc length there
// (what the §5 S6 audit sums), then joins the feet with a connector segment: a
// tangent ArcSeg for a fillet (§6), a chord LineSeg for a chamfer (§7).
// Everything downstream — the §5 audit, the profile rewrite, evalPrism and the
// mass properties — reads only this shared shape, so the two ops fork no
// machinery.
type cornerBlend struct {
	fA, fB             Point2
	cutbackA, cutbackB float64 // arc length consumed on the arriving / leaving walk
	connector          CurveSegment
}

// carrier is one side of a corner as an offsettable curve: a line through the
// corner with unit travel tangent, or a circle with a signed material side
// (insideSign +1 when the material is inside the circle — a CCW walk).
type carrier struct {
	isLine     bool
	px, py     float64 // a point on the line (the corner)
	tx, ty     float64 // unit travel tangent
	cx, cy     float64 // circle center
	radius     float64 // circle radius
	insideSign float64 // +1 material inside, −1 outside
}

// offCurve is a carrier's material-relative offset: a line (point + unit dir)
// or a concentric circle.
type offCurve struct {
	isLine     bool
	px, py     float64
	dx, dy     float64
	cx, cy, rr float64
}

// carrierOf builds the carrier of a walk at a corner: (tx, ty) is the walk's
// unit travel tangent there.
func carrierOf(w sideWalk, tx, ty float64) carrier {
	if !w.circular {
		return carrier{isLine: true, px: w.startU, py: w.startV, tx: tx, ty: ty}
	}
	inside := 1.0
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	return carrier{cx: w.cU, cy: w.cV, radius: w.radius, insideSign: inside}
}

// offsetOf offsets a carrier by r, signed by offsetSign (+1 a convex corner
// offsets INTO the material, −1 a concave corner offsets away). A circular
// carrier whose offset radius is non-positive has no blend of that radius: S5.
func offsetOf(c carrier, offsetSign, r float64) (offCurve, error) {
	if c.isLine {
		// The left unit normal points into the material for a walk with the
		// material on its left; the offset shifts the line that way for a
		// convex corner and the other way for a concave one.
		nlx, nly := -c.ty, c.tx
		s := offsetSign * r
		return offCurve{isLine: true, px: c.px + s*nlx, py: c.py + s*nly, dx: c.tx, dy: c.ty}, nil
	}
	rr := c.radius - offsetSign*c.insideSign*r
	// The gate accepts a fillet exactly when the offset radius rr clears
	// filletTol (1e-9 mm). In the ordinary case, where the offset shrinks the
	// wall (offsetSign*insideSign = +1), that means the fillet radius must sit
	// strictly below the wall radius less the tolerance (c.radius − filletTol).
	// When that boundary is at or below zero the wall itself is within the
	// evaluator's tolerance, so NO positive fillet fits — the remedy is a larger
	// wall, not a smaller radius.
	if rr <= filletTol {
		bound := c.radius - filletTol
		if bound <= 0 {
			return offCurve{}, fmt.Errorf(`%w: the circular wall radius %s is at or below the evaluator's tolerance, so no fillet of radius %s fits; enlarge the wall`, ErrDegenerate, units.Millimeters(c.radius), units.Millimeters(r))
		}
		return offCurve{}, fmt.Errorf(`%w: no fillet of radius %s fits a circular wall of radius %s; the fillet radius must be a positive value strictly below %s`, ErrDegenerate, units.Millimeters(r), units.Millimeters(c.radius), units.Millimeters(bound))
	}
	return offCurve{cx: c.cx, cy: c.cy, rr: rr}, nil
}

// computeFillet builds a corner's blend from its two carriers (§6): S4 rejects
// a smooth or cusped corner, the two material-side offsets are intersected for
// the center nearest the corner (S5 when they never meet), and the tangent
// feet, cutbacks and arc sense follow.
func computeFillet(loop cornerLoop, ci int, r float64) (*cornerBlend, error) {
	n := len(loop.walks)
	arrive := loop.walks[(ci+n-1)%n] // walk A, arriving at the corner
	leave := loop.walks[ci]          // walk B, leaving the corner
	px, py := leave.startU, leave.startV

	// Unit travel tangents at the corner: A's outgoing, B's incoming.
	ax, ay, la := normalize2(arrive.tanOutU, arrive.tanOutV)
	bx, by, lb := normalize2(leave.tanInU, leave.tanInV)
	if la == 0 || lb == 0 {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
	}
	// S4: a smooth (tangent) or cusped (anti-tangent) corner is no corner.
	cross := ax*by - ay*bx
	if math.Abs(cross) <= filletTol {
		return nil, fmt.Errorf(`%w: the two walls meet smoothly — there is no corner to round`, ErrDegenerate)
	}
	offsetSign := 1.0 // convex: the walk turns left (material wedge < π)
	if cross < 0 {
		offsetSign = -1.0 // concave: fill material in
	}

	carA := carrierOf(arrive, ax, ay)
	carB := carrierOf(leave, bx, by)
	offA, err := offsetOf(carA, offsetSign, r)
	if err != nil {
		return nil, err
	}
	offB, err := offsetOf(carB, offsetSign, r)
	if err != nil {
		return nil, err
	}
	ox, oy, err := intersectOffsets(offA, offB, px, py)
	if err != nil {
		return nil, err
	}

	fax, fay := footOn(carA, ox, oy)
	fbx, fby := footOn(carB, ox, oy)
	cutA := cutbackOn(carA, px, py, fax, fay)
	cutB := cutbackOn(carB, px, py, fbx, fby)

	// The arc's walk sense: at fA the boundary continues in A's travel
	// direction, so the CCW tangent there (rotate(fA−O, +90°)) agrees with tA
	// exactly when the arc is a CCW walk.
	rx, ry := -(fay - oy), fax-ox
	ccw := rx*ax+ry*ay > 0

	fA := Point2{U: fax, V: fay}
	fB := Point2{U: fbx, V: fby}
	return &cornerBlend{
		fA:        fA,
		fB:        fB,
		cutbackA:  cutA,
		cutbackB:  cutB,
		connector: arcSegment(Point2{U: ox, V: oy}, fA, fB, ccw),
	}, nil
}

// normalize2 returns the unit vector and length of (x, y).
func normalize2(x, y float64) (float64, float64, float64) {
	l := math.Hypot(x, y)
	if l == 0 {
		return 0, 0, 0
	}
	return x / l, y / l, l
}

// intersectOffsets intersects two offset curves and returns the root nearest
// the corner (px, py). No intersection is S5 — no blend of that radius exists.
func intersectOffsets(a, b offCurve, px, py float64) (float64, float64, error) {
	var cands [][2]float64
	switch {
	case a.isLine && b.isLine:
		cands = lineLine(a, b)
	case a.isLine && !b.isLine:
		cands = lineCircle(a, b.cx, b.cy, b.rr)
	case !a.isLine && b.isLine:
		cands = lineCircle(b, a.cx, a.cy, a.rr)
	default:
		cands = circleCircle(a.cx, a.cy, a.rr, b.cx, b.cy, b.rr)
	}
	if len(cands) == 0 {
		return 0, 0, fmt.Errorf(`%w: no fillet of that radius fits this corner; try a smaller radius`, ErrDegenerate)
	}
	best, bestD := cands[0], math.Inf(1)
	for _, c := range cands {
		dd := (c[0]-px)*(c[0]-px) + (c[1]-py)*(c[1]-py)
		if dd < bestD {
			bestD, best = dd, c
		}
	}
	return best[0], best[1], nil
}

// lineLine intersects two lines (point + unit dir); parallel lines yield none.
func lineLine(a, b offCurve) [][2]float64 {
	den := a.dx*b.dy - a.dy*b.dx
	if math.Abs(den) <= filletTol {
		return nil
	}
	s := ((b.px-a.px)*b.dy - (b.py-a.py)*b.dx) / den
	return [][2]float64{{a.px + s*a.dx, a.py + s*a.dy}}
}

// lineCircle intersects a line (point + unit dir) with a circle.
func lineCircle(l offCurve, cx, cy, r float64) [][2]float64 {
	// |P + s·d − C|² = r², d unit: s² + 2 s (P−C)·d + |P−C|² − r² = 0.
	fx, fy := l.px-cx, l.py-cy
	bb := fx*l.dx + fy*l.dy
	cc := fx*fx + fy*fy - r*r
	disc := bb*bb - cc
	if disc < 0 {
		return nil
	}
	sq := math.Sqrt(math.Max(disc, 0))
	var out [][2]float64
	for _, s := range []float64{-bb + sq, -bb - sq} {
		out = append(out, [2]float64{l.px + s*l.dx, l.py + s*l.dy})
	}
	return out
}

// circleCircle intersects two circles.
func circleCircle(c0x, c0y, r0, c1x, c1y, r1 float64) [][2]float64 {
	dx, dy := c1x-c0x, c1y-c0y
	dsq := dx*dx + dy*dy
	d := math.Sqrt(dsq)
	if d <= filletTol || d > r0+r1+filletTol || d < math.Abs(r0-r1)-filletTol {
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

// footOn returns the tangent foot of the center on a carrier: the perpendicular
// projection onto a line, the nearest circle point along the center ray for a
// circle.
func footOn(c carrier, ox, oy float64) (float64, float64) {
	if c.isLine {
		s := (ox-c.px)*c.tx + (oy-c.py)*c.ty
		return c.px + s*c.tx, c.py + s*c.ty
	}
	ux, uy, l := normalize2(ox-c.cx, oy-c.cy)
	if l == 0 {
		return c.cx + c.radius, c.cy
	}
	return c.cx + c.radius*ux, c.cy + c.radius*uy
}

// cutbackOn is the arc length a carrier loses from the corner to its foot: a
// chord on a line, an arc on a circle.
func cutbackOn(c carrier, px, py, fx, fy float64) float64 {
	if c.isLine {
		return math.Hypot(fx-px, fy-py)
	}
	ap := math.Atan2(py-c.cy, px-c.cx)
	af := math.Atan2(fy-c.cy, fx-c.cx)
	d := math.Mod(af-ap, 2*math.Pi)
	if d > math.Pi {
		d -= 2 * math.Pi
	}
	if d < -math.Pi {
		d += 2 * math.Pi
	}
	return c.radius * math.Abs(d)
}

// rewriteProfile applies every corner's blend to the section, returning the new
// ProfileRecord and, per loop, the segment indices that are blend connectors
// (their faces carry the second fillet(i,j) / chamfer(i,j) role, Table B).
func rewriteProfile(orig ProfileRecord, loops []cornerLoop, blendAt []map[int]*cornerBlend) (ProfileRecord, []map[int]struct{}) {
	origLoops := append([]LoopRecord{orig.Outer}, orig.Holes...)
	newLoops := make([]LoopRecord, len(origLoops))
	blendSegs := make([]map[int]struct{}, len(origLoops))
	for li := range origLoops {
		blendSegs[li] = map[int]struct{}{}
		if len(blendAt[li]) == 0 {
			newLoops[li] = cloneLoopRecord(origLoops[li])
			continue
		}
		segs, connectors := rewriteLoop(loops[li], blendAt[li])
		newLoops[li] = LoopRecord{Segments: segs}
		blendSegs[li] = connectors
	}
	return ProfileRecord{Outer: newLoops[0], Holes: newLoops[1:]}, blendSegs
}

// rewriteLoop rebuilds one loop's segments with its corners blended: each walk
// is trimmed to the feet its two ends' blends pin, and each blend's connector —
// a fillet's tangent arc or a chamfer's chord — is inserted between the walls it
// joins.
func rewriteLoop(loop cornerLoop, blends map[int]*cornerBlend) ([]CurveSegment, map[int]struct{}) {
	n := len(loop.walks)
	var segs []CurveSegment
	connectors := map[int]struct{}{}
	for i := range n {
		w := loop.walks[i]
		startU, startV := w.startU, w.startV
		if cb := blends[i]; cb != nil { // corner i trims this walk's start
			startU, startV = cb.fB.U, cb.fB.V
		}
		endU, endV := w.endU, w.endV
		if cb := blends[(i+1)%n]; cb != nil { // corner i+1 trims this walk's end
			endU, endV = cb.fA.U, cb.fA.V
		}
		segs = append(segs, walkSegment(w, startU, startV, endU, endV))
		if cb := blends[(i+1)%n]; cb != nil {
			segs = append(segs, cb.connector)
			connectors[len(segs)-1] = struct{}{}
		}
	}
	return segs, connectors
}

// walkSegment re-emits a coalesced walk, trimmed to (start, end), as a
// LineSeg or an ArcSeg in the walk's own sense.
func walkSegment(w sideWalk, sU, sV, eU, eV float64) CurveSegment {
	if !w.circular {
		return LineSeg{Start: Point2{U: sU, V: sV}, End: Point2{U: eU, V: eV}, TStart: 0, TEnd: 1}
	}
	return arcSegment(Point2{U: w.cU, V: w.cV}, Point2{U: sU, V: sV}, Point2{U: eU, V: eV}, w.th1 > w.th0)
}

// arcSegment builds an ArcSeg walking from start to end about center: an
// ArcSeg walks CCW start→end by default (TStart<TEnd), so a clockwise walk is
// recorded with swapped endpoints and a reversed range — TStart>TEnd runs the
// CCW-defined arc backwards, which walkOf and the mass-property integrals both
// read exactly.
func arcSegment(center, start, end Point2, ccw bool) CurveSegment {
	if ccw {
		return ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}

// addBlendRoles gives every blend wall its second kind(i,j) role (Table B): the
// wall built from a blend connector already carries side(i,j); the second role —
// "fillet" for Fillet, "chamfer" for Chamfer — names the same (loop, segment) of
// the rewritten record.
func addBlendRoles(body *Body, ref StepRef, blendSegs []map[int]struct{}, kind string) {
	for li, segs := range blendSegs {
		for sj := range segs {
			side := fmt.Sprintf("side(%d,%d)", li, sj)
			blend := fmt.Sprintf("%s(%d,%d)", kind, li, sj)
			for _, f := range body.Faces() {
				for _, o := range f.origins {
					if o.Step == ref && o.Role == side {
						f.origins = append(f.origins, FeatureRef{Step: ref, Role: blend})
						break
					}
				}
			}
		}
	}
}
