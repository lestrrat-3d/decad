package sectionclip

import (
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type arcSeg = sectionrecord.ArcSeg

// meridianPoint is a recorded meridian point in exact axis coordinates: a
// along the oriented revolve axis from its anchor, rho across it on the
// material side.
type meridianPoint struct{ a, rho proofarith.Dyadic }

// Kinds of recorded meridian segment the revolve section classes read.
const (
	meridianLine = iota
	meridianArc
	meridianCircle
)

// meridianSegment is one whole recorded segment with its defining points in
// exact axis coordinates. For a line or arc, start and end are the entity's
// own points and forward says the loop walks from start to end. A circle
// keeps its record, whose winding and range a placement never changes.
type meridianSegment struct {
	kind               int
	start, end, center meridianPoint
	forward            bool
	radius2            proofarith.Dyadic
	onAxis             bool
	circle             circleSeg
}

// walkEnd is the point the loop walk reaches at the end of a line or arc.
func (seg meridianSegment) walkEnd() meridianPoint {
	if seg.forward {
		return seg.end
	}
	return seg.start
}

// revolveMeridian is the exact reading of a full solid Revolve that the two
// meridian section classes share (agent-docs/section-design.md, "Plane
// through or normal to the axis of a full Revolve"): the placed axis anchor
// and unit axial direction, and the meridian loops in axis coordinates.
type revolveMeridian struct {
	anchor, axial proofarith.DyV3
	outer         []meridianSegment
	holes         [][]meridianSegment
}

// readRevolveMeridian admits a full solid Revolve whose recorded meridian is
// the exact source of its material: zero section displacement, an exact
// resolved axis along one source-plane axis with zero bounds, zero radial
// admission allowance and zero snap allowance, a cardinal source frame and a
// signed-axis placement. Every loop must hold whole lines, arcs and circles
// with finite coordinates and no point across the axis. Every comparison is
// exact; a record outside these conditions is a silent miss.
func readRevolveMeridian(rp revolvePayload) (revolveMeridian, bool) {
	ax := rp.Ax
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 ||
		!box.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		!box.SignedAxisTransform(rp.Xform) || !proofbound.FiniteVec(rp.Frame.Origin()) ||
		ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 ||
		ax.RadialAdmitAllow != 0 || ax.Snap != (revolveaxis.SnapAllow{}) ||
		!((math.Abs(ax.DU) == 1 && ax.DV == 0) || (ax.DU == 0 && math.Abs(ax.DV) == 1)) ||
		!measurement.FiniteValues(ax.AU, ax.AV, ax.SnapTol) || ax.SnapTol < 0 {
		return revolveMeridian{}, false
	}
	au, av := proofarith.DyOfFinite(ax.AU), proofarith.DyOfFinite(ax.AV)
	du, dv := proofarith.DyOfFinite(ax.DU), proofarith.DyOfFinite(ax.DV)
	snap := proofarith.DyOfFinite(ax.SnapTol)
	world := func(u, v proofarith.Dyadic) proofarith.DyV3 {
		local := proofarith.DvAdd(proofarith.DyVec(rp.Frame.Origin()), proofarith.DvAdd(
			proofarith.DvScale(proofarith.DyVec(rp.Frame.U()), u),
			proofarith.DvScale(proofarith.DyVec(rp.Frame.V()), v)))
		return proofarith.DvTransform(rp.Xform, local)
	}
	anchor := world(au, av)
	axial := proofarith.DvSub(world(proofarith.DyAdd(au, du), proofarith.DyAdd(av, dv)), anchor)

	// (u, v) -> (a, rho) is the rotation [[dU, dV], [-dV, dU]] about the
	// anchor, so it keeps every loop's winding. The build reads a point or
	// centre within SnapTol of the axis as on it, so such a point must lie
	// exactly on the axis; a point across the axis is refused.
	point := func(p planePoint) (meridianPoint, bool) {
		if !measurement.FiniteValues(p.U, p.V) {
			return meridianPoint{}, false
		}
		u := proofarith.DySubScalar(proofarith.DyOfFinite(p.U), au)
		v := proofarith.DySubScalar(proofarith.DyOfFinite(p.V), av)
		m := meridianPoint{
			a:   proofarith.DyAdd(proofarith.DyMul(u, du), proofarith.DyMul(v, dv)),
			rho: proofarith.DySubScalar(proofarith.DyMul(v, du), proofarith.DyMul(u, dv)),
		}
		if m.rho.Sign() < 0 || (m.rho.Sign() > 0 && proofarith.DyCmp(m.rho, snap) <= 0) {
			return meridianPoint{}, false
		}
		return m, true
	}
	readLoop := func(loop loopRecord) ([]meridianSegment, bool) {
		out := make([]meridianSegment, 0, len(loop.Segments))
		for _, segment := range loop.Segments {
			read, ok := readMeridianSegment(segment, point)
			if !ok {
				return nil, false
			}
			out = append(out, read)
		}
		return out, len(out) > 0
	}
	outer, ok := readLoop(rp.Profile.Outer)
	if !ok {
		return revolveMeridian{}, false
	}
	holes := make([][]meridianSegment, 0, len(rp.Profile.Holes))
	for _, hole := range rp.Profile.Holes {
		read, ok := readLoop(hole)
		if !ok {
			return revolveMeridian{}, false
		}
		holes = append(holes, read)
	}
	return revolveMeridian{anchor: anchor, axial: axial, outer: outer, holes: holes}, true
}

// wholeRange reports whether a recorded range runs in the entity's own
// sense, and whether it is a whole edge.
func wholeRange(t0, t1 float64) (bool, bool) {
	switch {
	case t0 == 0 && t1 == 1:
		return true, true
	case t0 == 1 && t1 == 0:
		return false, true
	}
	return false, false
}

// readMeridianSegment reads one whole line, arc or circle. An arc's two
// pinned points must lie exactly equally far from its centre, so the record
// names one circular arc in every orientation.
func readMeridianSegment(segment curveSegment, point func(planePoint) (meridianPoint, bool)) (meridianSegment, bool) {
	switch segment := segment.(type) {
	case lineSeg:
		forward, whole := wholeRange(segment.TStart, segment.TEnd)
		start, okStart := point(segment.Start)
		end, okEnd := point(segment.End)
		if !whole || !okStart || !okEnd || segment.Start == segment.End {
			return meridianSegment{}, false
		}
		return meridianSegment{
			kind: meridianLine, start: start, end: end, forward: forward,
			onAxis: start.rho.IsZero() && end.rho.IsZero(),
		}, true
	case arcSeg:
		forward, whole := wholeRange(segment.TStart, segment.TEnd)
		start, okStart := point(segment.Start)
		end, okEnd := point(segment.End)
		center, okCenter := point(segment.Center)
		if !whole || !okStart || !okEnd || !okCenter || segment.Start == segment.End {
			return meridianSegment{}, false
		}
		r2 := squaredDistance(start, center)
		if proofarith.DyCmp(r2, squaredDistance(end, center)) != 0 || r2.Sign() <= 0 {
			return meridianSegment{}, false
		}
		return meridianSegment{
			kind: meridianArc, start: start, end: end, center: center, forward: forward, radius2: r2,
		}, true
	case circleSeg:
		radius, err := segment.Radius.In(units.Millimeter)
		_, whole := wholeRange(segment.TStart, segment.TEnd)
		center, okCenter := point(segment.Center)
		if err != nil || !whole || !okCenter || !measurement.FiniteValues(radius) || radius <= 0 {
			return meridianSegment{}, false
		}
		r := proofarith.DyOfFinite(radius)
		return meridianSegment{
			kind: meridianCircle, center: center, radius2: proofarith.DyMul(r, r), circle: segment,
		}, true
	}
	return meridianSegment{}, false
}

func squaredDistance(p, q meridianPoint) proofarith.Dyadic {
	da := proofarith.DySubScalar(p.a, q.a)
	dr := proofarith.DySubScalar(p.rho, q.rho)
	return proofarith.DyAdd(proofarith.DyMul(da, da), proofarith.DyMul(dr, dr))
}

// cross2 is the exact sign of (p − c) × (q − c) in (a, rho).
func cross2(c, p, q meridianPoint) int {
	pa, pr := proofarith.DySubScalar(p.a, c.a), proofarith.DySubScalar(p.rho, c.rho)
	qa, qr := proofarith.DySubScalar(q.a, c.a), proofarith.DySubScalar(q.rho, c.rho)
	return proofarith.DySubScalar(proofarith.DyMul(pa, qr), proofarith.DyMul(pr, qa)).Sign()
}

// onOpenArc reports whether x, a point off the arc's ends that is on its
// circle or any point in that direction from its centre, lies strictly inside
// the counter-clockwise sweep from start to end. Only signs of cross products
// about the centre decide it, so both readings agree.
func onOpenArc(seg meridianSegment, x meridianPoint) bool {
	c := seg.center
	switch sweep := cross2(c, seg.start, seg.end); {
	case sweep > 0:
		return cross2(c, seg.start, x) > 0 && cross2(c, x, seg.end) > 0
	case sweep < 0:
		return cross2(c, seg.start, x) > 0 || cross2(c, x, seg.end) > 0
	default:
		// Equal radii and distinct ends make this a half turn.
		return cross2(c, seg.start, x) > 0
	}
}

// clearOfAxis reports whether a segment that is not an axis line meets the
// axis at most at its ends: every point strictly inside it has rho > 0.
func clearOfAxis(seg meridianSegment) bool {
	switch seg.kind {
	case meridianLine:
		return !seg.onAxis
	case meridianArc:
		// On the circle, rho = rho_c + r sin(theta) is least at the point B
		// straight towards the axis, and the points with rho <= 0 form one
		// arc about B. Both ends have rho >= 0, so the sweep keeps rho > 0
		// inside it exactly when B is neither inside it nor one of its ends.
		c := seg.center
		if c.rho.Sign() > 0 && proofarith.DyCmp(proofarith.DyMul(c.rho, c.rho), seg.radius2) > 0 {
			return true
		}
		b := meridianPoint{a: c.a, rho: proofarith.DySubScalar(c.rho, proofarith.DyInt(1))}
		toward := func(p meridianPoint) bool {
			return proofarith.DyCmp(p.a, c.a) == 0 && proofarith.DyCmp(p.rho, c.rho) < 0
		}
		return !onOpenArc(seg, b) && !toward(seg.start) && !toward(seg.end)
	case meridianCircle:
	default:
		return false
	}
	// The whole circle clears the axis when its centre does by more than
	// its radius.
	return seg.center.rho.Sign() > 0 &&
		proofarith.DyCmp(proofarith.DyMul(seg.center.rho, seg.center.rho), seg.radius2) > 0
}

// liftedPlane is a requested frame's held axes, normal and origin taken
// exactly.
type liftedPlane struct{ u, v, n, origin proofarith.DyV3 }

// cardinalPlane lifts a cardinal requested frame whose stored normal is the
// exact cross product of its axes.
func cardinalPlane(plane r3.Frame) (liftedPlane, bool) {
	if !box.CardinalBasis(plane.U(), plane.V(), plane.N()) || !proofbound.FiniteVec(plane.Origin()) {
		return liftedPlane{}, false
	}
	p := liftedPlane{
		u: proofarith.DyVec(plane.U()), v: proofarith.DyVec(plane.V()), n: proofarith.DyVec(plane.N()),
		origin: proofarith.DyVec(plane.Origin()),
	}
	return p, proofarith.DvEqual(proofarith.DvCross(p.u, p.v), p.n)
}

// RevolveAxialSection is the exact section of a full solid Revolve by a
// cardinal plane holding its axis: the recorded meridian region and its
// mirror across the axis. A meridian clear of the axis gives two regions; one
// touching the axis along exactly one run of whole axis lines gives one
// region whose outer loop joins the off-axis chain to its mirror. Any other
// contact with the axis, an inexact mapped coordinate, or a plane missing the
// axis is a miss. See agent-docs/section-design.md.
func RevolveAxialSection(rp revolvePayload, plane r3.Frame) ([]profileRecord, bool) {
	m, ok := readRevolveMeridian(rp)
	if !ok {
		return nil, false
	}
	lifted, ok := cardinalPlane(plane)
	if !ok {
		return nil, false
	}
	u, v, n := lifted.u, lifted.v, lifted.n
	offset := proofarith.DvSub(m.anchor, lifted.origin)
	if !proofarith.DvDot(offset, n).IsZero() {
		return nil, false
	}
	// A point at axial a and signed offset t along e = n × axial maps into
	// the plane by a signed permutation of (a, t) plus the anchor's offset.
	// The axial direction and n are both signed world axes, so either they
	// are parallel, e is zero and det is zero, or the plane holds the axis
	// and det = (axial × e)·n = n·n = 1.
	e := proofarith.DvCross(n, m.axial)
	ax, ay := proofarith.DvDot(m.axial, u), proofarith.DvDot(m.axial, v)
	ex, ey := proofarith.DvDot(e, u), proofarith.DvDot(e, v)
	x0, y0 := proofarith.DvDot(offset, u), proofarith.DvDot(offset, v)
	det := proofarith.DySubScalar(proofarith.DyMul(ax, ey), proofarith.DyMul(ex, ay)).Sign()
	if det == 0 {
		return nil, false
	}
	place := func(mirror bool) func(meridianPoint) (planePoint, bool) {
		return func(p meridianPoint) (planePoint, bool) {
			t := p.rho
			if mirror {
				t = proofarith.DyNeg(t)
			}
			x, exactX := proofarith.DyAdd(x0, proofarith.DyAdd(proofarith.DyMul(ax, p.a), proofarith.DyMul(ex, t))).Float64()
			y, exactY := proofarith.DyAdd(y0, proofarith.DyAdd(proofarith.DyMul(ay, p.a), proofarith.DyMul(ey, t))).Float64()
			return planePoint{U: x, V: y}, exactX && exactY && measurement.FiniteValues(x, y)
		}
	}
	// The copy keeps the meridian's winding; the mirror reverses it, so its
	// image is walked backwards.
	copyPlace, mirrorPlace := place(false), place(true)

	for _, hole := range m.holes {
		for _, seg := range hole {
			if !clearOfAxis(seg) || seg.kind != meridianCircle && seg.walkEnd().rho.IsZero() {
				return nil, false
			}
		}
	}
	chain, touches, ok := axisChain(m.outer)
	if !ok {
		return nil, false
	}
	copyHoles := make([]loopRecord, 0, len(m.holes))
	mirrorHoles := make([]loopRecord, 0, len(m.holes))
	for _, hole := range m.holes {
		c, okCopy := placeLoop(hole, copyPlace, false)
		r, okMirror := placeLoop(hole, mirrorPlace, true)
		if !okCopy || !okMirror {
			return nil, false
		}
		copyHoles = append(copyHoles, loopRecord{Segments: c})
		mirrorHoles = append(mirrorHoles, loopRecord{Segments: r})
	}
	c, okCopy := placeLoop(chain, copyPlace, false)
	r, okMirror := placeLoop(chain, mirrorPlace, true)
	if !okCopy || !okMirror {
		return nil, false
	}
	if !touches {
		return []profileRecord{
			{Outer: loopRecord{Segments: c}, Holes: copyHoles},
			{Outer: loopRecord{Segments: r}, Holes: mirrorHoles},
		}, true
	}
	// The chain walks from the run's end Q back to its start P, and the
	// reversed mirror walks from P to Q, so the two close one loop.
	return []profileRecord{{
		Outer: loopRecord{Segments: append(c, r...)},
		Holes: append(copyHoles, mirrorHoles...),
	}}, true
}

// axisChain reads the outer loop's contact with the axis. A loop with no
// point on the axis is returned whole with touches false. Otherwise the loop
// must hold exactly one cyclically contiguous run of whole axis lines, every
// other segment must clear the axis away from its ends, and every vertex but
// the run's two ends must have rho > 0; it returns the off-axis chain in walk
// order, from the run's end back to its start, with touches true. A second
// run puts a chain junction other than the last on the axis, so the junction
// rule refuses it.
func axisChain(loop []meridianSegment) ([]meridianSegment, bool, bool) {
	n := len(loop)
	first := -1
	for i, seg := range loop {
		if !seg.onAxis {
			if !clearOfAxis(seg) {
				return nil, false, false
			}
			continue
		}
		if !loop[(i+n-1)%n].onAxis {
			first = i
		}
	}
	if first < 0 {
		for _, seg := range loop {
			if seg.kind != meridianCircle && seg.walkEnd().rho.IsZero() {
				return nil, false, false
			}
		}
		return loop, false, true
	}
	chain := make([]meridianSegment, 0, n)
	for k := range n {
		if seg := loop[(first+k)%n]; !seg.onAxis {
			chain = append(chain, seg)
		}
	}
	// Every chain junction but the last, which closes onto the run, must lie
	// off the axis; a second contact would pinch the mirrored region.
	for _, seg := range chain[:len(chain)-1] {
		if seg.walkEnd().rho.IsZero() {
			return nil, false, false
		}
	}
	return chain, true, true
}

// placeLoop maps a loop's segments into the requested plane. When reverse is
// set the image's winding is wrong, so the loop is walked backwards: a line
// swaps its range, an arc swaps its pinned ends, and a circle keeps its
// record because the reflection and the reversed walk cancel.
func placeLoop(loop []meridianSegment, place func(meridianPoint) (planePoint, bool), reverse bool) ([]curveSegment, bool) {
	out := make([]curveSegment, 0, len(loop))
	for _, seg := range loop {
		mapped, ok := placeSegment(seg, place, reverse)
		if !ok {
			return nil, false
		}
		out = append(out, mapped)
	}
	if reverse {
		slices.Reverse(out)
	}
	return out, true
}

func placeSegment(seg meridianSegment, place func(meridianPoint) (planePoint, bool), reverse bool) (curveSegment, bool) {
	switch seg.kind {
	case meridianLine:
		start, okStart := place(seg.start)
		end, okEnd := place(seg.end)
		t0, t1 := 0.0, 1.0
		if seg.forward == reverse {
			t0, t1 = 1, 0
		}
		return lineSeg{Start: start, End: end, TStart: t0, TEnd: t1}, okStart && okEnd
	case meridianArc:
		center, okCenter := place(seg.center)
		start, okStart := place(seg.start)
		end, okEnd := place(seg.end)
		t0, t1 := 0.0, 1.0
		if !seg.forward {
			t0, t1 = 1, 0
		}
		if reverse {
			start, end = end, start
		}
		return arcSeg{Center: center, Start: start, End: end, TStart: t0, TEnd: t1}, okCenter && okStart && okEnd
	case meridianCircle:
		center, ok := place(seg.center)
		circle := seg.circle
		circle.Center = center
		return circle, ok
	}
	return nil, false
}

// RevolveNormalSection is the exact section of a full solid Revolve by a
// cardinal plane normal to its axis at level h: one disk or annulus about the
// axis point per material interval of the meridian's crossings at h. Every
// vertex must lie off level h, every crossing must be transversal at a
// distinct radius held exactly as a float, and a crossing on the axis must
// come from an axis line. See agent-docs/section-design.md.
func RevolveNormalSection(rp revolvePayload, plane r3.Frame) ([]profileRecord, bool) {
	m, ok := readRevolveMeridian(rp)
	if !ok {
		return nil, false
	}
	lifted, ok := cardinalPlane(plane)
	u, v, n, origin := lifted.u, lifted.v, lifted.n, lifted.origin
	if !ok || !proofarith.DvEqual(n, m.axial) && !proofarith.DvEqual(n, proofarith.DvScale(m.axial, proofarith.DyInt(-1))) {
		return nil, false
	}
	h := proofarith.DvDot(proofarith.DvSub(origin, m.anchor), m.axial)
	offset := proofarith.DvSub(m.anchor, origin)
	cx, exactX := proofarith.DvDot(offset, u).Float64()
	cy, exactY := proofarith.DvDot(offset, v).Float64()
	if !exactX || !exactY || !measurement.FiniteValues(cx, cy) {
		return nil, false
	}
	var crossings []proofarith.Dyadic
	for _, loop := range append([][]meridianSegment{m.outer}, m.holes...) {
		for _, seg := range loop {
			found, ok := levelCrossings(seg, h)
			if !ok {
				return nil, false
			}
			crossings = append(crossings, found...)
		}
	}
	slices.SortFunc(crossings, proofarith.DyCmp)
	if len(crossings) == 0 || len(crossings)%2 != 0 {
		return nil, false
	}
	// Two loops meeting on the level would give one radius twice.
	for i := 1; i < len(crossings); i++ {
		if proofarith.DyCmp(crossings[i-1], crossings[i]) >= 0 {
			return nil, false
		}
	}
	center := planePoint{U: cx, V: cy}
	records := make([]profileRecord, 0, len(crossings)/2)
	for i := 0; i < len(crossings); i += 2 {
		inner, outer := crossings[i], crossings[i+1]
		outerMM, _ := outer.Float64()
		record := profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
			Center: center, Radius: units.Millimeters(outerMM), CCW: true, TStart: 0, TEnd: 1,
		}}}}
		if inner.Sign() > 0 {
			innerMM, _ := inner.Float64()
			record.Holes = []loopRecord{{Segments: []curveSegment{circleSeg{
				Center: center, Radius: units.Millimeters(innerMM), CCW: false, TStart: 1, TEnd: 0,
			}}}}
		}
		records = append(records, record)
	}
	return records, true
}

// levelCrossings returns the radii where one segment crosses the level
// a = h, each held exactly as a float. A segment end at level h, an inexact
// radius or square root, a radius across the axis, and an arc or circle
// crossing on the axis refuse.
func levelCrossings(seg meridianSegment, h proofarith.Dyadic) ([]proofarith.Dyadic, bool) {
	exact := func(r proofarith.Dyadic) (proofarith.Dyadic, bool) {
		f, ok := r.Float64()
		return r, ok && measurement.FiniteValues(f) && r.Sign() >= 0
	}
	if seg.kind != meridianCircle &&
		(proofarith.DyCmp(seg.start.a, h) == 0 || proofarith.DyCmp(seg.end.a, h) == 0) {
		return nil, false
	}
	if seg.kind == meridianLine {
		s0, s1 := proofarith.DyCmp(seg.start.a, h), proofarith.DyCmp(seg.end.a, h)
		if s0 == s1 {
			return nil, true
		}
		// rho = rho0 + (h - a0)(rho1 - rho0)/(a1 - a0), in exact rationals.
		q := new(big.Rat).Mul(proofarith.DySubScalar(h, seg.start.a).Rat(),
			proofarith.DySubScalar(seg.end.rho, seg.start.rho).Rat())
		q.Quo(q, proofarith.DySubScalar(seg.end.a, seg.start.a).Rat())
		q.Add(q, seg.start.rho.Rat())
		rho, ok := proofarith.DyOfRat(q)
		if !ok {
			return nil, false
		}
		// Both ends have rho >= 0, so a crossing on the axis is an axis
		// line's.
		rho, ok = exact(rho)
		if !ok {
			return nil, false
		}
		return []proofarith.Dyadic{rho}, true
	}
	da := proofarith.DySubScalar(h, seg.center.a)
	q := proofarith.DySubScalar(seg.radius2, proofarith.DyMul(da, da))
	if q.Sign() < 0 {
		return nil, true
	}
	// A tangent level (q = 0) gives one radius twice, which the caller's
	// distinct-radius rule refuses.
	w := proofarith.DySqrtDown(q)
	if !proofarith.DySquareEquals(w, q) {
		return nil, false
	}
	wd, _ := proofarith.DyOf(w)
	var out []proofarith.Dyadic
	for _, candidate := range [...]proofarith.Dyadic{
		proofarith.DySubScalar(seg.center.rho, wd), proofarith.DyAdd(seg.center.rho, wd),
	} {
		if seg.kind == meridianArc && !onOpenArc(seg, meridianPoint{a: h, rho: candidate}) {
			continue
		}
		rho, ok := exact(candidate)
		if !ok || rho.IsZero() {
			return nil, false
		}
		out = append(out, rho)
	}
	return out, true
}
