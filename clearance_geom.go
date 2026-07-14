package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the clearance kernel's boundary model (docs/clearance-design.md
// §2/§3): every proven-solid body reduces to trimmed carrier faces read off
// the evaluator's own payload (the same source the surveys read), plus the
// topology's exact edges and vertices. Trim membership on the shipped faces
// is closed form — an angular interval × a meridian/axial range on the
// revolution faces, loop containment with exact crossing parity on the planar
// ones — which is what makes §3's admission exact. It also carries the §2
// nesting-exclusion ray casts: crossings counted against every face,
// closed-form for plane, sphere, cylinder and cone, Sturm-certified for the
// torus quartic, with a deterministic direction ladder retried whenever a
// cast grazes an edge, a vertex or a tangency.

// clrAngTol is the dimensionless angular tolerance for parallelism and
// window-membership decisions, matching the evaluator's own 1e-9-relative
// classification style (revolve.go's snapTol).
const clrAngTol = 1e-9

// angWindow is a closed angular window [lo, hi] (hi − lo < 2π) or the full
// circle.
type angWindow struct {
	lo, hi float64
	full   bool
}

func newAngWindow(a, b float64) angWindow {
	lo, hi := math.Min(a, b), math.Max(a, b)
	if hi-lo >= 2*math.Pi-clrAngTol {
		return angWindow{full: true}
	}
	return angWindow{lo: lo, hi: hi}
}

// classify reports +1 when th lies in the window with more than margin to
// spare, −1 when it is out by more than margin, 0 in between.
func (w angWindow) classify(th, margin float64) int {
	if w.full {
		return 1
	}
	off := mod2pi(th - w.lo)
	ext := w.hi - w.lo
	if off <= ext {
		if off >= margin && ext-off >= margin {
			return 1
		}
		return 0
	}
	if off-ext > margin && 2*math.Pi-off > margin {
		return -1
	}
	return 0
}

// linWindow is a closed interval on a linear coordinate.
type linWindow struct{ lo, hi float64 }

func newLinWindow(a, b float64) linWindow {
	return linWindow{lo: math.Min(a, b), hi: math.Max(a, b)}
}

func (w linWindow) classify(z, margin float64) int {
	if z >= w.lo+margin && z <= w.hi-margin {
		return 1
	}
	if z < w.lo-margin || z > w.hi+margin {
		return -1
	}
	return 0
}

// region2 is a planar trim region: line/arc boundary loops in the face's own
// 2D frame, queried by exact crossing parity with a deterministic retry
// ladder — the same discipline as the wall survey's containment test.
type region2 struct {
	elems []surveyElem
	scale float64
}

func newRegion2(elems []surveyElem) region2 {
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, e := range elems {
		if e.kind == surveyLine {
			grow(e.ax, e.ay, e.bx, e.by)
			continue
		}
		grow(e.qx-e.rr, e.qx+e.rr, e.qy-e.rr, e.qy+e.rr)
	}
	return region2{elems: elems, scale: scale}
}

func (r region2) tol() float64 { return 1e-9 * r.scale }

// contains is exact parity membership; the second result is false when every
// ladder direction stays ambiguous.
func (r region2) contains(px, py float64) (bool, bool) {
	for i := range 16 {
		th := 0.5 + float64(i)*2.399963229728653
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range r.elems {
			n, good := rayCrossings(e, px, py, dx, dy, r.tol())
			if !good {
				ok = false
				break
			}
			crossings += n
		}
		if ok {
			return crossings%2 == 1, true
		}
	}
	return false, false
}

// boundaryDist is the distance from p to the region's boundary.
func (r region2) boundaryDist(px, py float64) float64 {
	best := math.Inf(1)
	for _, e := range r.elems {
		d, _, _ := e.nearest(px, py, survTiny*r.scale)
		if d < best {
			best = d
		}
	}
	return best
}

// classify reports +1 strictly inside with more than margin of boundary
// clearance, −1 strictly outside with more than margin, 0 ambiguous or on
// the boundary.
func (r region2) classify(px, py, margin float64) int {
	if r.boundaryDist(px, py) <= margin {
		return 0
	}
	in, ok := r.contains(px, py)
	if !ok {
		return 0
	}
	if in {
		return 1
	}
	return -1
}

// samples returns boundary sample points (element endpoints and midpoints) —
// points that lie ON the closed region by construction.
func (r region2) samples() [][2]float64 {
	var out [][2]float64
	for _, e := range r.elems {
		if e.kind == surveyLine {
			out = append(out, [2]float64{e.ax, e.ay}, [2]float64{(e.ax + e.bx) / 2, (e.ay + e.by) / 2})
			continue
		}
		lo, hi := e.arcRange()
		for _, th := range []float64{lo, (lo + hi) / 2} {
			out = append(out, [2]float64{e.qx + e.rr*math.Cos(th), e.qy + e.rr*math.Sin(th)})
		}
	}
	return out
}

// interiorPoint returns a point proven strictly inside the region, probing
// element-midpoint inward offsets at a few depths.
func (r region2) interiorPoint() ([2]float64, bool) {
	probe := func(x, y float64) bool { return r.classify(x, y, r.tol()) == 1 }
	for _, e := range r.elems {
		var px, py, nx, ny float64
		if e.kind == surveyLine {
			px, py = (e.ax+e.bx)/2, (e.ay+e.by)/2
			nx, ny = e.nx, e.ny
		} else {
			lo, hi := e.arcRange()
			th := (lo + hi) / 2
			px, py = e.qx+e.rr*math.Cos(th), e.qy+e.rr*math.Sin(th)
			s := e.matSign()
			nx, ny = -s*math.Cos(th), -s*math.Sin(th)
		}
		for _, f := range []float64{0.25, 0.03, 1e-4} {
			step := f * r.scale
			if e.kind == surveyArc && step > e.rr/2 {
				step = e.rr / 2
			}
			x, y := px+nx*step, py+ny*step
			if probe(x, y) {
				return [2]float64{x, y}, true
			}
		}
	}
	return [2]float64{}, false
}

// elemLineHits collects the crossing parameters of the (infinite) 2D line
// p + t·d with one boundary element; ok is false on an ambiguous geometry
// (near-parallel overlap, grazing an endpoint).
func elemLineHits(e surveyElem, px, py, dx, dy, tol float64) ([]float64, bool) {
	if e.kind == surveyLine {
		ex, ey := e.bx-e.ax, e.by-e.ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) + ex*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			if math.Abs((e.ax-px)*dy-(e.ay-py)*dx) <= tol {
				return nil, false
			}
			return nil, true
		}
		rx, ry := e.ax-px, e.ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return nil, true
		}
		if u <= uTol || u >= 1-uTol {
			return nil, false
		}
		return []float64{t}, true
	}
	fx, fy := px-e.qx, py-e.qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.rr*e.rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.rr {
			return nil, false
		}
		return nil, true
	}
	s := math.Sqrt(disc)
	if s <= tol {
		return nil, false
	}
	var out []float64
	for _, t := range []float64{-b - s, -b + s} {
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.qy, x-e.qx)
		if e.closed {
			out = append(out, t)
			continue
		}
		lo, hi := e.arcRange()
		off := mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			return nil, false
		}
		if off < ext {
			out = append(out, t)
		}
	}
	return out, true
}

// clrIv is a float interval.
type clrIv struct{ lo, hi float64 }

// lineIntervals returns the parameter intervals along the unit-direction 2D
// line p + t·d that lie inside the region; ok is false when the crossing
// structure is ambiguous.
func (r region2) lineIntervals(px, py, dx, dy float64) ([]clrIv, bool) {
	var ts []float64
	for _, e := range r.elems {
		hits, ok := elemLineHits(e, px, py, dx, dy, r.tol())
		if !ok {
			return nil, false
		}
		ts = append(ts, hits...)
	}
	if len(ts) == 0 {
		return nil, true
	}
	// Sort the crossings; between consecutive crossings membership is
	// constant, probed at the midpoint.
	for i := 1; i < len(ts); i++ {
		for j := i; j > 0 && ts[j] < ts[j-1]; j-- {
			ts[j], ts[j-1] = ts[j-1], ts[j]
		}
	}
	var out []clrIv
	for i := 0; i+1 < len(ts); i++ {
		mid := (ts[i] + ts[i+1]) / 2
		in, ok := r.contains(px+mid*dx, py+mid*dy)
		if !ok {
			return nil, false
		}
		if in {
			out = append(out, clrIv{lo: ts[i], hi: ts[i+1]})
		}
	}
	return out, true
}

// elemLineSuperset returns the proper crossing parameters of the unit 2D
// line p + t·d with one element, plus conservative spans covering every
// ambiguous contact (a parallel overlap, an endpoint graze, a tangency) —
// the ingredients of a SUPERSET of the region's intersection with the line,
// which may only ever exclude, never bless.
func elemLineSuperset(e surveyElem, px, py, dx, dy, tol float64) ([]float64, []clrIv) {
	if e.kind == surveyLine {
		ex, ey := e.bx-e.ax, e.by-e.ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) + ex*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			if math.Abs((e.ax-px)*dy-(e.ay-py)*dx) <= tol {
				ta := (e.ax-px)*dx + (e.ay-py)*dy
				tb := (e.bx-px)*dx + (e.by-py)*dy
				return nil, []clrIv{{lo: math.Min(ta, tb) - tol, hi: math.Max(ta, tb) + tol}}
			}
			return nil, nil
		}
		rx, ry := e.ax-px, e.ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return nil, nil
		}
		if u <= uTol || u >= 1-uTol {
			return nil, []clrIv{{lo: t - tol, hi: t + tol}}
		}
		return []float64{t}, nil
	}
	fx, fy := px-e.qx, py-e.qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.rr*e.rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.rr {
			w := math.Sqrt(math.Abs(disc)) + tol
			return nil, []clrIv{{lo: -b - w, hi: -b + w}}
		}
		return nil, nil
	}
	s := math.Sqrt(disc)
	if s <= tol {
		return nil, []clrIv{{lo: -b - s - tol, hi: -b + s + tol}}
	}
	var hits []float64
	var spans []clrIv
	for _, t := range []float64{-b - s, -b + s} {
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.qy, x-e.qx)
		if e.closed {
			hits = append(hits, t)
			continue
		}
		lo, hi := e.arcRange()
		off := mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			spans = append(spans, clrIv{lo: t - tol, hi: t + tol})
			continue
		}
		if off < ext {
			hits = append(hits, t)
		}
	}
	return hits, spans
}

// lineIntervalsSuperset returns a conservative superset of the region's
// intersection with the unit 2D line p + t·d: the parity-inside intervals
// between proper crossings plus every ambiguous contact's span. Sound for
// exclusion only — an empty result proves the line misses the closed region.
func (r region2) lineIntervalsSuperset(px, py, dx, dy float64) []clrIv {
	var ts []float64
	var spans []clrIv
	for _, e := range r.elems {
		hits, sp := elemLineSuperset(e, px, py, dx, dy, r.tol())
		ts = append(ts, hits...)
		spans = append(spans, sp...)
	}
	for i := 1; i < len(ts); i++ {
		for j := i; j > 0 && ts[j] < ts[j-1]; j-- {
			ts[j], ts[j-1] = ts[j-1], ts[j]
		}
	}
	for i := 0; i+1 < len(ts); i++ {
		mid := (ts[i] + ts[i+1]) / 2
		in, ok := r.contains(px+mid*dx, py+mid*dy)
		if !ok || in {
			// Undecidable membership is conservatively included.
			spans = append(spans, clrIv{lo: ts[i], hi: ts[i+1]})
		}
	}
	return spans
}

// segmentHits classifies a 2D segment against the region: +1 provably
// intersecting, −1 provably disjoint, 0 ambiguous. The +1 witness point is
// returned for admitted candidates.
func (r region2) segmentHits(ax, ay, bx, by float64) (int, [2]float64) {
	l := math.Hypot(bx-ax, by-ay)
	if l <= r.tol() {
		c := r.classify(ax, ay, r.tol())
		return c, [2]float64{ax, ay}
	}
	dx, dy := (bx-ax)/l, (by-ay)/l
	ivs, ok := r.lineIntervals(ax, ay, dx, dy)
	if ok {
		for _, iv := range ivs {
			lo := math.Max(iv.lo, 0)
			hi := math.Min(iv.hi, l)
			if hi-lo > 2*r.tol() {
				m := (lo + hi) / 2
				return 1, [2]float64{ax + m*dx, ay + m*dy}
			}
		}
	}
	// Endpoint membership decides containment with no crossing.
	for _, p := range [][2]float64{{ax, ay}, {bx, by}, {(ax + bx) / 2, (ay + by) / 2}} {
		if r.classify(p[0], p[1], r.tol()) == 1 {
			return 1, p
		}
	}
	// Provably disjoint: the segment clears every boundary element and one
	// endpoint is cleanly outside — a segment entering the region would have
	// to cross the boundary.
	clearing := math.Inf(1)
	for _, e := range r.elems {
		d := segElemDistLB(e, ax, ay, bx, by)
		if d < clearing {
			clearing = d
		}
	}
	if clearing > r.tol() && r.classify(ax, ay, r.tol()) == -1 {
		return -1, [2]float64{}
	}
	return 0, [2]float64{}
}

// segElemDistLB is a lower bound on the distance between a 2D segment and a
// boundary element (the arc bound goes through its full circle — an
// underestimate, which is the sound direction for exclusion proofs).
func segElemDistLB(e surveyElem, ax, ay, bx, by float64) float64 {
	if e.kind == surveyLine {
		return segSegDist(ax, ay, bx, by, e.ax, e.ay, e.bx, e.by)
	}
	lo, hi := segPointDistRange(ax, ay, bx, by, e.qx, e.qy)
	if hi < e.rr {
		return e.rr - hi
	}
	if lo > e.rr {
		return lo - e.rr
	}
	return 0
}

// segPointDistRange is the exact [min, max] of the distance from a point to
// a segment's points.
func segPointDistRange(ax, ay, bx, by, px, py float64) (float64, float64) {
	ex, ey := bx-ax, by-ay
	l2 := ex*ex + ey*ey
	da := math.Hypot(px-ax, py-ay)
	db := math.Hypot(px-bx, py-by)
	mx := math.Max(da, db)
	if l2 == 0 {
		return da, mx
	}
	u := ((px-ax)*ex + (py-ay)*ey) / l2
	if u <= 0 || u >= 1 {
		return math.Min(da, db), mx
	}
	fx, fy := ax+u*ex-px, ay+u*ey-py
	return math.Hypot(fx, fy), mx
}

// segSegDist is the exact distance between two 2D segments.
func segSegDist(ax, ay, bx, by, cx, cy, dx, dy float64) float64 {
	if segsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
		return 0
	}
	d1, _ := segPointDistRange(ax, ay, bx, by, cx, cy)
	d2, _ := segPointDistRange(ax, ay, bx, by, dx, dy)
	d3, _ := segPointDistRange(cx, cy, dx, dy, ax, ay)
	d4, _ := segPointDistRange(cx, cy, dx, dy, bx, by)
	return math.Min(math.Min(d1, d2), math.Min(d3, d4))
}

func segsIntersect(ax, ay, bx, by, cx, cy, dx, dy float64) bool {
	o := func(px, py, qx, qy, rx, ry float64) float64 {
		return (qx-px)*(ry-py) - (qy-py)*(rx-px)
	}
	o1 := o(ax, ay, bx, by, cx, cy)
	o2 := o(ax, ay, bx, by, dx, dy)
	o3 := o(cx, cy, dx, dy, ax, ay)
	o4 := o(cx, cy, dx, dy, bx, by)
	return o1*o2 < 0 && o3*o4 < 0
}

// cKind discriminates the kernel's carrier variants.
type cKind int

const (
	ckPlane cKind = iota
	ckCylinder
	ckCone
	ckSphere
	ckTorus
)

// cFace is one trimmed face in world coordinates: the carrier plus its
// closed-form trim, a containing box, and on-face witness points.
type cFace struct {
	kind cKind

	// plane: o + x·u + y·v, outward normal n; trim = region.
	o, u, v, n r3.Vec
	region     region2

	// axis-symmetric carriers: anchor + axis (unit) + azimuth frame.
	anchor     r3.Vec
	axis       r3.Vec
	refU, refV r3.Vec
	radius     float64 // cylinder/sphere radius, torus minor
	major      float64 // torus major
	half       float64 // cone half-angle
	sweep      angWindow
	zWin       linWindow // cylinder: axial range; cone: axial range from apex
	merid      angWindow // sphere/torus meridian window
	spindle    bool      // torus minor >= major: off the polynomial path (§4)

	box [2]r3.Vec
	wit []r3.Vec
}

// admitState folds two per-side admission classifications: any −1 rejects,
// else any 0 downgrades to a straddle.
func admitState(cs ...int) int {
	out := 1
	for _, c := range cs {
		if c == -1 {
			return -1
		}
		if c == 0 {
			out = 0
		}
	}
	return out
}

// planeCoords maps a world point into the plane face's 2D frame.
func (f *cFace) planeCoords(p r3.Vec) (float64, float64) {
	rel := p.Sub(f.o)
	return rel.Dot(f.u), rel.Dot(f.v)
}

// planeOffset is the face plane's offset along its unit normal.
func (f *cFace) planeOffset() float64 { return f.n.Dot(f.o) }

// axisCoords maps a world point into (z along axis from anchor, ρ, azimuth).
func (f *cFace) axisCoords(p r3.Vec) (float64, float64, float64) {
	rel := p.Sub(f.anchor)
	z := rel.Dot(f.axis)
	rad := rel.Sub(f.axis.Scale(z))
	rho := rad.Len()
	return z, rho, math.Atan2(rad.Dot(f.refV), rad.Dot(f.refU))
}

// admitPoint classifies a world point claimed to lie on the face's carrier
// against the trim: +1 admitted with margin, −1 rejected with margin, 0
// ambiguous. margin is a length.
func (f *cFace) admitPoint(p r3.Vec, margin float64) int {
	switch f.kind {
	case ckPlane:
		x, y := f.planeCoords(p)
		return f.region.classify(x, y, margin)
	case ckCylinder:
		z, _, phi := f.axisCoords(p)
		angMargin := margin / math.Max(f.radius, 1e-30)
		return admitState(f.zWin.classify(z, margin), f.sweep.classify(phi, angMargin))
	case ckCone:
		z, rho, phi := f.axisCoords(p)
		angMargin := margin / math.Max(rho, 1e-30)
		return admitState(f.zWin.classify(z, margin), f.sweep.classify(phi, angMargin))
	case ckSphere:
		z, rho, phi := f.axisCoords(p)
		th := math.Atan2(rho, z)
		angMargin := margin / math.Max(f.radius, 1e-30)
		c := f.merid.classify(th, angMargin)
		if rho <= margin {
			// A pole: every azimuth matches; only the meridian decides.
			return c
		}
		return admitState(c, f.sweep.classify(phi, margin/math.Max(rho, 1e-30)))
	default: // ckTorus
		z, rho, phi := f.axisCoords(p)
		mu := math.Atan2(rho-f.major, z)
		angMargin := margin / math.Max(f.radius, 1e-30)
		c := f.merid.classify(mu, angMargin)
		if rho <= margin {
			return c
		}
		return admitState(c, f.sweep.classify(phi, margin/math.Max(rho, 1e-30)))
	}
}

// boxOf grows a box over points.
func boxOf(pts ...r3.Vec) [2]r3.Vec {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, p := range pts {
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	return [2]r3.Vec{lo, hi}
}

func boxUnion(a, b [2]r3.Vec) [2]r3.Vec {
	return [2]r3.Vec{
		r3.NewVec(math.Min(a[0].X, b[0].X), math.Min(a[0].Y, b[0].Y), math.Min(a[0].Z, b[0].Z)),
		r3.NewVec(math.Max(a[1].X, b[1].X), math.Max(a[1].Y, b[1].Y), math.Max(a[1].Z, b[1].Z)),
	}
}

// circleBox is the exact box of a full 3D circle.
func circleBox(c, axis r3.Vec, r float64) [2]r3.Vec {
	ext := r3.NewVec(
		r*math.Sqrt(math.Max(0, 1-axis.X*axis.X)),
		r*math.Sqrt(math.Max(0, 1-axis.Y*axis.Y)),
		r*math.Sqrt(math.Max(0, 1-axis.Z*axis.Z)),
	)
	return [2]r3.Vec{c.Sub(ext), c.Add(ext)}
}

// clrBoxDist is the distance between two boxes (zero when they meet).
func clrBoxDist(a, b [2]r3.Vec) float64 {
	gap := func(alo, ahi, blo, bhi float64) float64 {
		if ahi < blo {
			return blo - ahi
		}
		if bhi < alo {
			return alo - bhi
		}
		return 0
	}
	dx := gap(a[0].X, a[1].X, b[0].X, b[1].X)
	dy := gap(a[0].Y, a[1].Y, b[0].Y, b[1].Y)
	dz := gap(a[0].Z, a[1].Z, b[0].Z, b[1].Z)
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// cEdge is one boundary edge: a segment or a circular arc/circle in world
// coordinates.
type cEdge struct {
	line   bool
	a, b   r3.Vec // segment endpoints
	center r3.Vec
	axis   r3.Vec
	refU   r3.Vec
	refV   r3.Vec
	radius float64
	ang    angWindow
	box    [2]r3.Vec
}

func (e *cEdge) at(th float64) r3.Vec {
	s, c := math.Sincos(th)
	return e.center.Add(e.refU.Scale(e.radius * c)).Add(e.refV.Scale(e.radius * s))
}

// bodyGeom is one body's kernel boundary model.
type bodyGeom struct {
	body     *Body
	faces    []*cFace
	edges    []*cEdge
	verts    []r3.Vec
	shellWit []r3.Vec // one witness point per shell (§2)
	supports []r3.Vec // support points for the pair-D reading (§7)
}

// perpTo returns a deterministic unit vector perpendicular to a unit vector.
func perpTo(a r3.Vec) r3.Vec {
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(a.X) > 0.9 {
		seed = r3.NewVec(0, 1, 0)
	}
	p, _ := a.Cross(seed).Normalize()
	return p
}

// newBodyGeom builds the kernel model; ok is false for a payload this
// evaluator did not build — the pair then stays undecided, never guessed.
func newBodyGeom(b *Body) (*bodyGeom, bool) {
	g := &bodyGeom{body: b}
	switch pl := b.payload.(type) {
	case prismPayload:
		if !g.addPrismFaces(pl) {
			return nil, false
		}
	case revolvePayload:
		if !g.addRevolveFaces(pl) {
			return nil, false
		}
	default:
		return nil, false
	}
	if !g.addTopology(b) {
		return nil, false
	}
	g.supports = append([]r3.Vec{}, g.verts...)
	for _, f := range g.faces {
		g.supports = append(g.supports, f.wit...)
	}
	return g, true
}

// addTopology reads the exact edges and vertices off the built topology and
// picks one witness point per shell for the §2 nesting casts.
func (g *bodyGeom) addTopology(b *Body) bool {
	for _, e := range b.Edges() {
		ce, ok := newCEdge(e)
		if !ok {
			return false
		}
		if ce != nil {
			g.edges = append(g.edges, ce)
		}
	}
	for _, v := range b.Vertices() {
		g.verts = append(g.verts, v.position)
	}
	for _, sh := range b.Shells() {
		w, ok := shellWitness(sh)
		if !ok {
			return false
		}
		g.shellWit = append(g.shellWit, w)
	}
	return true
}

func newCEdge(e *Edge) (*cEdge, bool) {
	switch c := e.curve.(type) {
	case Line3:
		if e.start.position.Sub(e.end.position).Len() == 0 {
			return nil, true
		}
		ce := &cEdge{line: true, a: e.start.position, b: e.end.position}
		ce.box = boxOf(ce.a, ce.b)
		return ce, true
	case Circle3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		u := perpTo(c.Axis)
		v := c.Axis.Cross(u)
		ce := &cEdge{center: c.Center, axis: c.Axis, refU: u, refV: v, radius: r, ang: angWindow{full: true}}
		ce.box = circleBox(c.Center, c.Axis, r)
		return ce, true
	case Arc3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		rel := e.start.position.Sub(c.Center)
		u, ok := rel.Normalize()
		if !ok {
			return nil, false
		}
		v := c.Axis.Cross(u)
		relEnd := e.end.position.Sub(c.Center)
		sweep := math.Atan2(relEnd.Dot(v), relEnd.Dot(u))
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		ce := &cEdge{center: c.Center, axis: c.Axis, refU: u, refV: v, radius: r, ang: newAngWindow(0, sweep)}
		ce.box = circleBox(c.Center, c.Axis, r)
		return ce, true
	default:
		return nil, false
	}
}

// shellWitness returns a point provenly on the shell: a vertex where one
// exists, else the loop-less closed face's own synthesized point off stored
// surface data (§2).
func shellWitness(sh *Shell) (r3.Vec, bool) {
	for _, f := range sh.faces {
		for _, e := range f.Edges() {
			if e.start != nil {
				return e.start.position, true
			}
		}
	}
	for _, f := range sh.faces {
		switch s := f.surface.(type) {
		case Sphere:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(perpTo(r3.NewVec(0, 0, 1)).Scale(r)), true
		case Torus:
			major, err1 := s.Major.In(units.Millimeter)
			minor, err2 := s.Minor.In(units.Millimeter)
			if err1 != nil || err2 != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(perpTo(s.Axis).Scale(major + minor)), true
		}
	}
	return r3.Vec{}, false
}

// addPrismFaces builds the prism's faces from its own payload, mirroring the
// walk decomposition the evaluator built the topology from.
func (g *bodyGeom) addPrismFaces(pp prismPayload) bool {
	loops, err := recordLoops(pp.profile)
	if err != nil {
		return false
	}
	nDir := pp.dir(0, 0, 1)
	h := pp.z1 - pp.z0

	var capElems []surveyElem
	for _, loop := range loops {
		for _, w := range loop {
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return false
			}
			capElems = append(capElems, el)

			if w.circular {
				f := &cFace{
					kind:   ckCylinder,
					anchor: pp.point(w.cU, w.cV, pp.z0),
					axis:   nDir,
					refU:   pp.dir(1, 0, 0),
					refV:   pp.dir(0, 1, 0),
					radius: w.radius,
					zWin:   newLinWindow(0, h),
					sweep:  angWindow{full: w.closed},
				}
				if !w.closed {
					f.sweep = newAngWindow(w.th0, w.th1)
				}
				top := pp.point(w.cU, w.cV, pp.z1)
				f.box = boxUnion(circleBox(f.anchor, nDir, w.radius), circleBox(top, nDir, w.radius))
				midTh := (w.th0 + w.th1) / 2
				f.wit = append(f.wit,
					pp.point(w.cU+w.radius*math.Cos(midTh), w.cV+w.radius*math.Sin(midTh), (pp.z0+pp.z1)/2),
					pp.point(w.cU+w.radius*math.Cos(w.th0), w.cV+w.radius*math.Sin(w.th0), pp.z0))
				g.faces = append(g.faces, f)
				continue
			}

			l := math.Hypot(w.endU-w.startU, w.endV-w.startV)
			if l == 0 {
				return false
			}
			e1, ok := pp.dir(w.endU-w.startU, w.endV-w.startV, 0).Normalize()
			if !ok {
				return false
			}
			tu, tv := w.tanInU, w.tanInV
			if pp.reflected() {
				tu, tv = -tu, -tv
			}
			nOut, ok := pp.dir(tu, tv, 0).Cross(nDir).Normalize()
			if !ok {
				return false
			}
			f := &cFace{
				kind: ckPlane,
				o:    pp.point(w.startU, w.startV, pp.z0),
				u:    e1,
				v:    nDir,
				n:    nOut,
			}
			le0, _ := lineElem(0, 0, l, 0)
			le1, _ := lineElem(l, 0, l, h)
			le2, _ := lineElem(l, h, 0, h)
			le3, _ := lineElem(0, h, 0, 0)
			f.region = newRegion2([]surveyElem{le0, le1, le2, le3})
			f.box = boxOf(f.o, f.o.Add(e1.Scale(l)), f.o.Add(nDir.Scale(h)), f.o.Add(e1.Scale(l)).Add(nDir.Scale(h)))
			f.wit = append(f.wit, f.o.Add(e1.Scale(l/2)).Add(nDir.Scale(h/2)), f.o)
			g.faces = append(g.faces, f)
		}
	}

	region := newRegion2(capElems)
	for _, cap := range []struct {
		z    float64
		sign float64
	}{{z: pp.z0, sign: -1}, {z: pp.z1, sign: 1}} {
		f := &cFace{
			kind:   ckPlane,
			o:      pp.point(0, 0, cap.z),
			u:      pp.dir(1, 0, 0),
			v:      pp.dir(0, 1, 0),
			n:      nDir.Scale(cap.sign),
			region: region,
		}
		f.box = capBox(f)
		f.wit = capWitnesses(f)
		g.faces = append(g.faces, f)
	}
	return true
}

// capBox is the world box of a planar face's region boundary (arcs taken as
// their full circles — conservative, and a box only needs to contain).
func capBox(f *cFace) [2]r3.Vec {
	at := func(x, y float64) r3.Vec { return f.o.Add(f.u.Scale(x)).Add(f.v.Scale(y)) }
	box := [2]r3.Vec{
		r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)),
		r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1)),
	}
	for _, e := range f.region.elems {
		if e.kind == surveyLine {
			box = boxUnion(box, boxOf(at(e.ax, e.ay), at(e.bx, e.by)))
			continue
		}
		axis := f.u.Cross(f.v)
		box = boxUnion(box, circleBox(at(e.qx, e.qy), axis, e.rr))
	}
	return box
}

// capWitnesses returns on-face points: boundary samples plus a verified
// interior point when one is found.
func capWitnesses(f *cFace) []r3.Vec {
	var out []r3.Vec
	for _, s := range f.region.samples() {
		out = append(out, f.o.Add(f.u.Scale(s[0])).Add(f.v.Scale(s[1])))
	}
	if p, ok := f.region.interiorPoint(); ok {
		out = append(out, f.o.Add(f.u.Scale(p[0])).Add(f.v.Scale(p[1])))
	}
	return out
}

// addRevolveFaces builds the revolved body's faces from its own payload.
func (g *bodyGeom) addRevolveFaces(rp revolvePayload) bool {
	loops, err := revolveLoops(rp)
	if err != nil {
		return false
	}
	b := rp.basis()
	a3p := rp.xform.Apply(b.a3)
	wp := rp.xform.ApplyDir(b.w)
	e0p := rp.xform.ApplyDir(b.e0)
	e1p := rp.xform.ApplyDir(b.e1)
	sweep := angWindow{full: rp.full}
	if !rp.full {
		sweep = newAngWindow(rp.phi0, rp.phi1)
	}
	midPhi := (rp.phi0 + rp.phi1) / 2
	onAxis := func(z float64) r3.Vec { return a3p.Add(wp.Scale(z)) }

	var capElems []surveyElem
	for _, loop := range loops {
		for _, w := range loop {
			kind := rp.ax.classify(w.segmentWalk)
			if el, ok := walkElem(w.segmentWalk); ok {
				capElems = append(capElems, el)
			}
			switch kind {
			case wallAxis:
				continue
			case wallCylinder:
				r := (w.startV + w.endV) / 2
				f := &cFace{
					kind: ckCylinder, anchor: a3p, axis: wp, refU: e0p, refV: e1p,
					radius: r, zWin: newLinWindow(w.startU, w.endU), sweep: sweep,
				}
				f.box = boxUnion(circleBox(onAxis(w.startU), wp, r), circleBox(onAxis(w.endU), wp, r))
				f.wit = append(f.wit, rp.point(b, (w.startU+w.endU)/2, r, midPhi), rp.point(b, w.startU, r, rp.phi0))
				g.faces = append(g.faces, f)
			case wallPlane:
				rlo := math.Min(w.startV, w.endV)
				rhi := math.Max(w.startV, w.endV)
				sign := 1.0
				if w.tanInV < 0 {
					sign = -1
				}
				f := &cFace{
					kind: ckPlane,
					o:    onAxis(w.startU),
					u:    e0p, v: e1p,
					n: wp.Scale(sign),
				}
				f.region = annularRegion(rlo, rhi, rp.phi0, rp.phi1, rp.full)
				f.box = capBox(f)
				f.wit = capWitnesses(f)
				g.faces = append(g.faces, f)
			case wallCone:
				dz, dr := w.endU-w.startU, w.endV-w.startV
				apexZ := w.startU - w.startV*dz/dr
				growth := 1.0
				if dz*dr < 0 {
					growth = -1
				}
				f := &cFace{
					kind:   ckCone,
					anchor: onAxis(apexZ),
					axis:   wp.Scale(growth),
					refU:   e0p, refV: e1p,
					half:  math.Atan2(math.Abs(dr), math.Abs(dz)),
					zWin:  newLinWindow(math.Abs(w.startU-apexZ), math.Abs(w.endU-apexZ)),
					sweep: sweep,
				}
				f.box = boxUnion(circleBox(onAxis(w.startU), wp, w.startV), circleBox(onAxis(w.endU), wp, w.endV))
				f.wit = append(f.wit, rp.point(b, (w.startU+w.endU)/2, (w.startV+w.endV)/2, midPhi))
				g.faces = append(g.faces, f)
				if w.startV <= 0 || w.endV <= 0 {
					// The apex sits on the trimmed face: a surface singular
					// point, synthesized as a vertex-like candidate (§3).
					g.verts = append(g.verts, onAxis(apexZ))
				}
			case wallSphere:
				f := &cFace{
					kind:   ckSphere,
					anchor: onAxis(w.cU),
					axis:   wp,
					refU:   e0p, refV: e1p,
					radius: w.radius,
					merid:  angWindow{full: w.closed},
					sweep:  sweep,
				}
				if !w.closed {
					f.merid = newAngWindow(w.th0, w.th1)
				}
				f.box = [2]r3.Vec{
					f.anchor.Sub(r3.NewVec(w.radius, w.radius, w.radius)),
					f.anchor.Add(r3.NewVec(w.radius, w.radius, w.radius)),
				}
				midTh := (w.th0 + w.th1) / 2
				f.wit = append(f.wit, rp.point(b, w.cU+w.radius*math.Cos(midTh), math.Max(0, w.radius*math.Sin(midTh)), midPhi))
				g.faces = append(g.faces, f)
			case wallTorus:
				f := &cFace{
					kind:   ckTorus,
					anchor: onAxis(w.cU),
					axis:   wp,
					refU:   e0p, refV: e1p,
					radius:  w.radius,
					major:   w.cV,
					merid:   angWindow{full: w.closed},
					sweep:   sweep,
					spindle: w.radius >= w.cV-clrAngTol*math.Max(1, w.cV),
				}
				if !w.closed {
					f.merid = newAngWindow(w.th0, w.th1)
				}
				spineBox := circleBox(f.anchor, wp, f.major)
				pad := r3.NewVec(w.radius, w.radius, w.radius)
				f.box = [2]r3.Vec{spineBox[0].Sub(pad), spineBox[1].Add(pad)}
				midTh := (w.th0 + w.th1) / 2
				f.wit = append(f.wit, rp.point(b, w.cU+w.radius*math.Cos(midTh), w.cV+w.radius*math.Sin(midTh), midPhi))
				g.faces = append(g.faces, f)
				// A spindle patch reaching the axis at a walk endpoint has a
				// singular axis-collapse point there, synthesized like a cone
				// apex (§3).
				if !w.closed && w.startV == 0 {
					g.verts = append(g.verts, onAxis(w.cU+w.radius*math.Cos(w.th0)))
				}
				if !w.closed && w.endV == 0 {
					g.verts = append(g.verts, onAxis(w.cU+w.radius*math.Cos(w.th1)))
				}
			}
		}
	}

	if !rp.full {
		region := newRegion2(capElems)
		for _, cap := range []struct {
			phi  float64
			sign float64
		}{{phi: rp.phi0, sign: -1}, {phi: rp.phi1, sign: 1}} {
			sin, cos := math.Sincos(cap.phi)
			radial := e0p.Scale(cos).Add(e1p.Scale(sin))
			vel := e0p.Scale(-sin).Add(e1p.Scale(cos))
			f := &cFace{
				kind:   ckPlane,
				o:      a3p,
				u:      wp,
				v:      radial,
				n:      vel.Scale(cap.sign),
				region: region,
			}
			f.box = capBox(f)
			f.wit = capWitnesses(f)
			g.faces = append(g.faces, f)
		}
	}
	return true
}

// annularRegion builds the 2D region of a revolve wallPlane face in its
// (e0, e1) frame: an annular sector, a disk sector, an annulus or a disk.
func annularRegion(rlo, rhi, phi0, phi1 float64, full bool) region2 {
	var elems []surveyElem
	if full {
		if e, ok := arcElem(0, 0, rhi, 0, 2*math.Pi, true); ok {
			elems = append(elems, e)
		}
		if rlo > 0 {
			if e, ok := arcElem(0, 0, rlo, 0, 2*math.Pi, true); ok {
				elems = append(elems, e)
			}
		}
		return newRegion2(elems)
	}
	if e, ok := arcElem(0, 0, rhi, phi0, phi1, false); ok {
		elems = append(elems, e)
	}
	if rlo > 0 {
		if e, ok := arcElem(0, 0, rlo, phi0, phi1, false); ok {
			elems = append(elems, e)
		}
	}
	s0, c0 := math.Sincos(phi0)
	s1, c1 := math.Sincos(phi1)
	if e, ok := lineElem(rlo*c0, rlo*s0, rhi*c0, rhi*s0); ok {
		elems = append(elems, e)
	}
	if e, ok := lineElem(rlo*c1, rlo*s1, rhi*c1, rhi*s1); ok {
		elems = append(elems, e)
	}
	return newRegion2(elems)
}

// clrLadder is the deterministic cast-direction ladder of §2: fixed,
// never random, so a replay resolves identically.
func clrLadder() []r3.Vec {
	out := make([]r3.Vec, 16)
	for i := range out {
		th := 0.5 + float64(i)*2.399963229728653
		z := 1 - 2*(float64(i)+0.5)/16
		r := math.Sqrt(math.Max(0, 1-z*z))
		out[i] = r3.NewVec(r*math.Cos(th), r*math.Sin(th), z)
	}
	return out
}

// pointInBody decides strict membership of a point in the body's material by
// a certified ray cast (§2): crossings counted against every face, each
// crossing certified transversal and interior to its trim; a grazing cast
// retries the next ladder direction. ok is false when every direction stays
// ambiguous.
func (g *bodyGeom) pointInBody(p r3.Vec, tol float64) (bool, bool) {
	for _, dir := range clrLadder() {
		total, ok := 0, true
		for _, f := range g.faces {
			n, good := f.rayCrossings(p, dir, tol)
			if !good {
				ok = false
				break
			}
			total += n
		}
		if ok {
			return total%2 == 1, true
		}
	}
	return false, false
}

// rayCrossings counts certified transversal crossings of the ray p + t·dir
// (t > tol) with the trimmed face; good is false on any ambiguity.
func (f *cFace) rayCrossings(p, dir r3.Vec, tol float64) (int, bool) {
	switch f.kind {
	case ckPlane:
		nd := f.n.Dot(dir)
		if math.Abs(nd) < 1e-7 {
			if math.Abs(f.n.Dot(p)-f.planeOffset()) > tol {
				return 0, true
			}
			return 0, false
		}
		t := (f.planeOffset() - f.n.Dot(p)) / nd
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start (a witness on a
			// shared construction plane): a crossing only if the start sits
			// on the trimmed face — cleanly off it, no crossing.
			if f.admitPoint(p.Add(dir.Scale(t)), tol) == -1 {
				return 0, true
			}
			return 0, false
		}
		if t < 0 {
			return 0, true
		}
		switch f.admitPoint(p.Add(dir.Scale(t)), tol) {
		case 1:
			return 1, true
		case -1:
			return 0, true
		default:
			return 0, false
		}
	case ckCylinder:
		rel := p.Sub(f.anchor)
		relP := rel.Sub(f.axis.Scale(rel.Dot(f.axis)))
		dirP := dir.Sub(f.axis.Scale(dir.Dot(f.axis)))
		a := dirP.Dot(dirP)
		b := relP.Dot(dirP)
		c := relP.Dot(relP) - f.radius*f.radius
		return f.quadraticCrossings(p, dir, a, b, c, tol)
	case ckSphere:
		rel := p.Sub(f.anchor)
		return f.quadraticCrossings(p, dir, 1, rel.Dot(dir), rel.Dot(rel)-f.radius*f.radius, tol)
	case ckCone:
		rel := p.Sub(f.anchor)
		k := math.Tan(f.half)
		az := rel.Dot(f.axis)
		dz := dir.Dot(f.axis)
		relP := rel.Sub(f.axis.Scale(az))
		dirP := dir.Sub(f.axis.Scale(dz))
		a := dirP.Dot(dirP) - k*k*dz*dz
		b := relP.Dot(dirP) - k*k*az*dz
		c := relP.Dot(relP) - k*k*az*az
		n, ok := f.quadraticCrossings(p, dir, a, b, c, tol)
		if !ok {
			return 0, false
		}
		return n, true
	default: // ckTorus
		return f.torusCrossings(p, dir, tol)
	}
}

// quadraticCrossings counts admitted roots of a·t² + 2b·t + c = 0 along the
// ray; the cone caller relies on admitPoint's z-window (from the apex,
// positive) to reject the wrong nappe.
func (f *cFace) quadraticCrossings(p, dir r3.Vec, a, b, c, tol float64) (int, bool) {
	if math.Abs(a) < 1e-14 {
		return 0, false
	}
	disc := b*b - a*c
	scale := math.Abs(a)*f.radius*f.radius + math.Abs(c) + 1
	if f.kind == ckCone {
		scale = math.Abs(c) + math.Abs(b) + 1
	}
	if disc <= 0 {
		if disc > -1e-9*scale {
			return 0, false
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	if s <= 1e-9*math.Sqrt(scale) {
		return 0, false
	}
	n := 0
	for _, t := range []float64{(-b - s) / a, (-b + s) / a} {
		q := p.Add(dir.Scale(t))
		if f.kind == ckCone && q.Sub(f.anchor).Dot(f.axis) < 0 {
			continue
		}
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start: cleanly off the
			// trimmed face means no crossing; anything else is ambiguous.
			if f.admitPoint(q, tol) == -1 {
				continue
			}
			return 0, false
		}
		if t < 0 {
			continue
		}
		switch f.admitPoint(q, tol) {
		case 1:
			n++
		case -1:
		default:
			return 0, false
		}
	}
	return n, true
}

// torusCrossings counts admitted ray roots of the torus quartic with Sturm
// certification (§2): a root count is a proof only when the isolation
// certifies it — a non-square-free quartic (a tangency) is ambiguous and the
// ladder retries.
func (f *cFace) torusCrossings(p, dir r3.Vec, tol float64) (int, bool) {
	rel := p.Sub(f.anchor)
	// |x−C|² = q2 t² + q1 t + q0; axial² = (a0 + a1 t)².
	q2 := dir.Dot(dir)
	q1 := 2 * rel.Dot(dir)
	q0 := rel.Dot(rel)
	a0 := rel.Dot(f.axis)
	a1 := dir.Dot(f.axis)
	k := f.major*f.major + q0 - f.radius*f.radius
	// f(t) = (|x−C|² + R² − r²)² − 4R²(|x−C|² − axial²)
	quad := ratPoly{ratOf(k), ratOf(q1), ratOf(q2)}
	sq := rpMul(quad, quad)
	perp := rpSub(ratPoly{ratOf(q0), ratOf(q1), ratOf(q2)}, rpMul(ratPoly{ratOf(a0), ratOf(a1)}, ratPoly{ratOf(a0), ratOf(a1)}))
	four := ratOf(4 * f.major * f.major)
	poly := rpTrim(rpSub(sq, rpScale(perp, four)))
	if rpDeg(poly) < 1 {
		return 0, false
	}
	sf := rpSquareFree(poly)
	if rpDeg(sf) != rpDeg(poly) {
		// A repeated root is a tangency somewhere on the line: ambiguous.
		return 0, false
	}
	chain := sturmChain(sf)
	n := 0
	for _, iv := range rpIsolateRoots(sf) {
		iv = rpRefineRoot(chain, iv, func(lo, hi float64) bool { return hi-lo <= 1e-11*math.Max(1, math.Abs(lo)) })
		tLo, _ := iv.lo.Float64()
		tHi, _ := iv.hi.Float64()
		if tHi <= -tol {
			continue // behind the start
		}
		if tLo <= tol {
			// At or straddling the start: a crossing there can only be the
			// carrier passing through the witness — cleanly off the trimmed
			// face means no crossing, anything else is ambiguous.
			q0 := p.Add(dir.Scale((tLo + tHi) / 2))
			if tHi-tLo <= tol && f.admitPoint(q0, tol+(tHi-tLo)) == -1 {
				continue
			}
			return 0, false
		}
		q := p.Add(dir.Scale((tLo + tHi) / 2))
		switch f.admitPoint(q, tol+(tHi-tLo)) {
		case 1:
			n++
		case -1:
		default:
			return 0, false
		}
	}
	return n, true
}
