package decad

import "math"

// This file is the 2D kernel behind the increment-5 analytic wall survey
// (docs/verification-design.md §6, docs/evaluator-design.md §10): the
// spanning-ball reading of a prism or revolved body reduces exactly to
// empty-disk analysis in a 2D section — the profile plane for a prism, the
// meridian plane through the ball's center for a solid of revolution — so the
// survey enumerates a closed-form candidate set of critical inscribed disks
// and validates each against the whole boundary. The candidate set is
// complete for the attained infimum/supremum over line/arc boundaries: a
// critical disk has three contacts (an Apollonius solution), two contacts at
// an antipodal configuration (a pair critical), two contacts exactly at the
// allowance boundary (an angle-limit disk — the closed-under-limits family of
// §6), or a whole-arc contact (a concentric disk). Corner pinches — dihedrals
// within the allowance, which span at every size a ball is starved to — are
// the caller's to report; they read zero and need no disk.

// survTiny is the relative floor for degeneracy checks inside the kernel.
const survTiny = 1e-12

// survAngTol is the inclusive angular slack: a wall drafted at exactly the
// allowance spans (verification §6 — within is inclusive), so every angular
// comparison concedes this much in the spanning direction only.
const survAngTol = 1e-9

// dirArc is a closed set of unit directions {(cos t, sin t) : t ∈ [lo, hi]},
// lo ≤ hi ≤ lo + 2π. A single direction has lo == hi. Contact directions are
// dirArcs because a disk can touch an arc element along a whole angular
// range (the concentric contact).
type dirArc struct{ lo, hi float64 }

// mod2pi maps x into [0, 2π).
func mod2pi(x float64) float64 {
	m := math.Mod(x, 2*math.Pi)
	if m < 0 {
		m += 2 * math.Pi
	}
	return m
}

// angDist is the angular distance between two directions, in [0, π].
func angDist(a, b float64) float64 {
	return math.Pi - math.Abs(mod2pi(a-b)-math.Pi)
}

// arcsIntersect reports whether two direction arcs share a direction,
// modulo 2π.
func arcsIntersect(a, b dirArc) bool {
	aext := a.hi - a.lo
	bext := b.hi - b.lo
	if aext >= 2*math.Pi || bext >= 2*math.Pi {
		return true
	}
	d := mod2pi(b.lo - a.lo)
	return d <= aext || d+bext >= 2*math.Pi
}

// maxAngleBetween is the largest angle (≤ π) between a direction of a and a
// direction of b. It peaks at π exactly when some direction of a has its
// antipode in b; otherwise the maximum sits at interval endpoints.
func maxAngleBetween(a, b dirArc) float64 {
	if arcsIntersect(dirArc{lo: a.lo + math.Pi, hi: a.hi + math.Pi}, b) {
		return math.Pi
	}
	m := 0.0
	for _, ta := range []float64{a.lo, a.hi} {
		for _, tb := range []float64{b.lo, b.hi} {
			if d := angDist(ta, tb); d > m {
				m = d
			}
		}
	}
	return m
}

// surveyElemKind discriminates the kernel's boundary elements.
type surveyElemKind int

const (
	surveyLine surveyElemKind = iota
	surveyArc
)

// surveyElem is one 2D boundary element, walked with the material on its
// left.
type surveyElem struct {
	kind surveyElemKind

	// line: (ax, ay) → (bx, by); (nx, ny) is the unit inward (material-side)
	// normal.
	ax, ay, bx, by float64
	nx, ny         float64

	// arc: center (qx, qy), radius rr, walked from th0 to th1. th1 > th0 is a
	// counter-clockwise walk, which puts the material inside the circle
	// (matInside); a clockwise walk puts it outside (a hole wall, a notch).
	qx, qy, rr float64
	th0, th1   float64
	closed     bool
	matInside  bool
}

// lineElem builds a line element from a walk-ordered segment.
func lineElem(ax, ay, bx, by float64) (surveyElem, bool) {
	tx, ty := bx-ax, by-ay
	l := math.Hypot(tx, ty)
	if l == 0 {
		return surveyElem{}, false
	}
	// Left of the walk direction is the material side.
	return surveyElem{
		kind: surveyLine,
		ax:   ax, ay: ay, bx: bx, by: by,
		nx: -ty / l, ny: tx / l,
	}, true
}

// arcElem builds an arc element from walk geometry.
func arcElem(qx, qy, rr, th0, th1 float64, closed bool) (surveyElem, bool) {
	if rr <= 0 {
		return surveyElem{}, false
	}
	return surveyElem{
		kind: surveyArc,
		qx:   qx, qy: qy, rr: rr,
		th0: th0, th1: th1, closed: closed,
		matInside: th1 > th0,
	}, true
}

// arcRange is the walked angular range in ascending order.
func (e surveyElem) arcRange() (float64, float64) {
	if e.th1 >= e.th0 {
		return e.th0, e.th1
	}
	return e.th1, e.th0
}

// arcContains reports whether angle θ lies on the walked range, modulo 2π.
func (e surveyElem) arcContains(th float64) bool {
	if e.closed {
		return true
	}
	lo, hi := e.arcRange()
	if hi-lo >= 2*math.Pi {
		return true
	}
	return mod2pi(th-lo) <= hi-lo
}

// matSign is +1 for a material-inside arc (an inscribed disk sits inside the
// circle, |c−q| = R − r) and −1 for material-outside (|c−q| = R + r).
func (e surveyElem) matSign() float64 {
	if e.matInside {
		return 1
	}
	return -1
}

// nearest returns the distance from p to the element and the direction set
// from p toward its nearest point(s). dirOK is false when the direction is
// undefined (p on the element, or exactly at an arc's center where the
// contact is the whole walked range — reported as that range).
func (e surveyElem) nearest(px, py, tiny float64) (float64, dirArc, bool) {
	switch e.kind {
	case surveyLine:
		ex, ey := e.bx-e.ax, e.by-e.ay
		l2 := ex*ex + ey*ey
		u := ((px-e.ax)*ex + (py-e.ay)*ey) / l2
		u = math.Max(0, math.Min(1, u))
		nx, ny := e.ax+u*ex-px, e.ay+u*ey-py
		d := math.Hypot(nx, ny)
		if d <= tiny {
			return d, dirArc{}, false
		}
		th := math.Atan2(ny, nx)
		return d, dirArc{lo: th, hi: th}, true
	default: // surveyArc
		vx, vy := px-e.qx, py-e.qy
		rho := math.Hypot(vx, vy)
		if rho <= tiny {
			// The concentric query: every point of the walked range is
			// nearest, and the contact direction set is the range itself.
			lo, hi := e.arcRange()
			return e.rr, dirArc{lo: lo, hi: hi}, true
		}
		thp := math.Atan2(vy, vx)
		if e.arcContains(thp) {
			d := math.Abs(rho - e.rr)
			if d <= tiny {
				return d, dirArc{}, false
			}
			cx := e.qx + e.rr*math.Cos(thp) - px
			cy := e.qy + e.rr*math.Sin(thp) - py
			th := math.Atan2(cy, cx)
			return d, dirArc{lo: th, hi: th}, true
		}
		// Off the walked range: the nearest point is an endpoint.
		lo, hi := e.arcRange()
		d := math.Inf(1)
		var bx, by float64
		for _, th := range []float64{lo, hi} {
			x := e.qx + e.rr*math.Cos(th)
			y := e.qy + e.rr*math.Sin(th)
			if dd := math.Hypot(x-px, y-py); dd < d {
				d, bx, by = dd, x, y
			}
		}
		if d <= tiny {
			return d, dirArc{}, false
		}
		th := math.Atan2(by-py, bx-px)
		return d, dirArc{lo: th, hi: th}, true
	}
}

// diskCand is a candidate inscribed disk.
type diskCand struct{ x, y, r float64 }

// wallKernel is one 2D section survey: elements (material on the left),
// junction vertices, the draft allowance, and — for a partial revolve — the
// wedge the sweep caps cut, encoded by wedgeS = sin(min(Δφ/2, π/2)): a ball
// of radius r centered at height y fits the wedge iff y·wedgeS ≥ r.
type wallKernel struct {
	elems       []surveyElem
	containOnly []surveyElem // boundary for containment only (on-axis chords)
	verts       [][2]float64
	alpha       float64
	wedgeS      float64 // 0 = no wedge
	wedgeSpans  bool    // two wedge-cap contacts count as a spanning pair
	fitMax      float64 // spanning disks wider than this cannot lift to 3D
	scale, tol  float64
	subTolFar   bool         // a sub-tolerance candidate away from every junction was dropped
	boundary    []surveyElem // elems + containOnly, built lazily for contains
}

// newWallKernel sizes the tolerances from the geometry.
func newWallKernel(elems, containOnly []surveyElem, verts [][2]float64, alpha, wedgeS float64, wedgeSpans bool, fitMax float64) *wallKernel {
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, e := range append(append([]surveyElem{}, elems...), containOnly...) {
		if e.kind == surveyLine {
			grow(e.ax, e.ay, e.bx, e.by)
			continue
		}
		grow(e.qx-e.rr, e.qx+e.rr, e.qy-e.rr, e.qy+e.rr)
	}
	return &wallKernel{
		elems:       elems,
		containOnly: containOnly,
		verts:       verts,
		alpha:       alpha,
		wedgeS:      wedgeS,
		wedgeSpans:  wedgeSpans,
		fitMax:      fitMax,
		scale:       scale,
		tol:         1e-9 * scale,
	}
}

// wallSurveyOut is the kernel's answer over its candidate set.
type wallSurveyOut struct {
	ok        bool    // false: the survey could not decide (never a silent pass)
	hasSpan   bool    // some empty spanning disk exists
	span      float64 // the smallest spanning diameter found
	inradius  float64 // the largest empty disk radius found (the 2D inradius)
	subTolFar bool    // an off-junction sub-tolerance candidate was dropped
}

// run enumerates and validates the candidate set.
func (k *wallKernel) run() wallSurveyOut {
	out := wallSurveyOut{ok: true, span: math.Inf(1)}
	for _, c := range k.generate() {
		spanning, empty, ok := k.validate(c)
		if !ok {
			return wallSurveyOut{}
		}
		if !empty {
			continue
		}
		if c.r > out.inradius {
			out.inradius = c.r
		}
		if spanning && 2*c.r <= k.fitMax+k.tol {
			out.hasSpan = true
			if 2*c.r < out.span {
				out.span = 2 * c.r
			}
		}
	}
	out.subTolFar = k.subTolFar
	return out
}

// validate checks one candidate disk against the whole boundary: it must fit
// the wedge (if any), sit inside the material, and clear every element; its
// contact set decides whether it spans — two contact directions within the
// allowance of antipodal (inclusive), a single whole-arc contact wide
// enough, or (when the wedge's two cap contacts span) an active wedge.
func (k *wallKernel) validate(c diskCand) (bool, bool, bool) {
	if math.IsNaN(c.x) || math.IsNaN(c.y) || math.IsNaN(c.r) || math.IsInf(c.r, 0) {
		return false, false, true
	}
	// The candidate floor: a disk this small is indistinguishable from the
	// numeric noise of a degenerate solve. Dropping it is safe ONLY where a
	// junction owns the spot — the junction-pinch rule reports the true
	// dihedral there exactly. Away from every junction the same tiny disk
	// can be a REAL near-tangent web (two boundary elements almost touching
	// mid-span), and treating it as absent would bless a body whose wall is
	// thinner than the kernel can resolve: that is an undecided survey,
	// never a silent pass.
	if c.r <= 4*k.tol {
		reach := c.r + 8*k.tol
		for _, v := range k.verts {
			if math.Hypot(c.x-v[0], c.y-v[1]) <= reach {
				return false, false, true
			}
		}
		// Not junction-owned: remember it. The caller decides — an exact
		// zero elsewhere still stands (nothing sits below zero), but a
		// positive reading or an absence claim would rest on a candidate
		// the kernel could not resolve, and reads undecided instead.
		k.subTolFar = true
		return false, false, true
	}
	wedgeActive := false
	if k.wedgeS > 0 {
		room := c.y * k.wedgeS
		if room < c.r-k.tol {
			return false, false, true
		}
		wedgeActive = room <= c.r+k.contactTol()
	}
	var contacts []dirArc
	for _, e := range k.elems {
		d, dir, dirOK := e.nearest(c.x, c.y, survTiny*k.scale)
		if d < c.r-k.tol {
			return false, false, true
		}
		if d <= c.r+k.contactTol() {
			if !dirOK {
				// A contact whose direction is undefined: the disk is
				// degenerate against this element; do not decide on it.
				return false, false, true
			}
			contacts = append(contacts, dir)
		}
	}
	inside, ok := k.contains(c.x, c.y)
	if !ok {
		return false, false, false
	}
	if !inside {
		return false, false, true
	}
	spanning := k.wedgeSpans && wedgeActive
	need := math.Pi - k.alpha - survAngTol
	for i := 0; i < len(contacts) && !spanning; i++ {
		for j := i; j < len(contacts); j++ {
			if maxAngleBetween(contacts[i], contacts[j]) >= need {
				spanning = true
				break
			}
		}
	}
	return spanning, true, true
}

// contactTol is the slack for counting an element as a contact.
func (k *wallKernel) contactTol() float64 { return 8 * k.tol }

// contains is the material-membership test: crossing parity of a ray against
// every boundary element, retried across directions when a crossing is
// ambiguous (near an endpoint, near tangency, or grazing the start). All
// directions ambiguous → undecided, never guessed.
func (k *wallKernel) contains(px, py float64) (bool, bool) {
	if k.boundary == nil {
		k.boundary = append(append([]surveyElem{}, k.elems...), k.containOnly...)
	}
	all := k.boundary
	for i := range 16 {
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range all {
			n, good := rayCrossings(e, px, py, dx, dy, k.tol)
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

// rayCrossings counts proper crossings of the ray p + t·d (t > 0) with one
// element; good is false when a crossing is too ambiguous to count.
func rayCrossings(e surveyElem, px, py, dx, dy, tol float64) (int, bool) {
	if e.kind == surveyLine {
		ex, ey := e.bx-e.ax, e.by-e.ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) - (-ex)*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			// Parallel: an overlap along the ray line is ambiguous; a clean
			// miss (the segment's line offset from the ray's) is a zero.
			if math.Abs((e.ax-px)*dy-(e.ay-py)*dx) <= tol {
				return 0, false
			}
			return 0, true
		}
		rx, ry := e.ax-px, e.ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return 0, true
		}
		if t <= tol {
			if t > -tol {
				return 0, false // the boundary passes through the ray start
			}
			return 0, true
		}
		if u <= uTol || u >= 1-uTol {
			return 0, false // the ray grazes a segment endpoint
		}
		return 1, true
	}
	// Arc: |p + t·d − q|² = R².
	fx, fy := px-e.qx, py-e.qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.rr*e.rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.rr {
			// Grazing tangency: ambiguous only when it happens ahead of us.
			if -b > 0 {
				return 0, false
			}
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	n := 0
	for _, t := range []float64{-b - s, -b + s} {
		if t <= tol {
			if t > -tol {
				return 0, false
			}
			continue
		}
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.qy, x-e.qx)
		if e.closed {
			n++
			continue
		}
		lo, hi := e.arcRange()
		off := mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			return 0, false
		}
		if off < ext {
			n++
		}
	}
	if math.Abs(s) <= tol {
		return 0, false
	}
	return n, true
}

// generate emits the closed-form candidate set: pair criticals (T2),
// angle-limit disks (T3), Apollonius triples (T4), concentric whole-arc
// disks, and — under a wedge — the wedge-tangent minima. Candidates are
// generated liberally; validate is what admits them.
func (k *wallKernel) generate() []diskCand {
	var out []diskCand
	add := func(x, y, r float64) { out = append(out, diskCand{x: x, y: y, r: r}) }

	// Concentric whole-arc disks.
	for _, e := range k.elems {
		if e.kind == surveyArc && e.matInside {
			add(e.qx, e.qy, e.rr)
		}
	}

	// Pair criticals and angle limits.
	for i := range k.elems {
		for j := i + 1; j < len(k.elems); j++ {
			k.pairCands(k.elems[i], k.elems[j], add)
		}
	}
	for _, e := range k.elems {
		for _, v := range k.verts {
			k.vertexElemCands(v, e, add)
		}
	}
	for i := range k.verts {
		for j := i + 1; j < len(k.verts); j++ {
			k.vertexVertexCands(k.verts[i], k.verts[j], add)
		}
	}

	// Wedge-tangent minima (partial revolve only).
	if k.wedgeS > 0 {
		k.wedgeCands(add)
	}

	// Apollonius triples, the wedge included as an item.
	eqs := k.tripleEquations()
	for i := range eqs {
		for j := i + 1; j < len(eqs); j++ {
			for l := j + 1; l < len(eqs); l++ {
				solveTriple([3]circEq{eqs[i], eqs[j], eqs[l]}, k.scale, add)
			}
		}
	}
	return out
}

// pairCands emits the antipodal pair criticals (T2) and the angle-limit
// disks (T3) for one element pair.
func (k *wallKernel) pairCands(a, b surveyElem, add func(x, y, r float64)) {
	switch {
	case a.kind == surveyLine && b.kind == surveyLine:
		k.lineLineCands(a, b, add)
	case a.kind == surveyArc && b.kind == surveyArc:
		k.arcArcCands(a, b, add)
	case a.kind == surveyLine:
		k.lineArcCands(a, b, add)
	default:
		k.lineArcCands(b, a, add)
	}
}

// lineLineCands: parallel facing lines carry a constant-diameter family;
// the overlap midpoint is its representative (blocked positions surface via
// the triples). Non-parallel lines have no interior critical: their corners
// are junctions, their extent limits vertices, their blockers triples.
func (k *wallKernel) lineLineCands(a, b surveyElem, add func(x, y, r float64)) {
	cross := a.nx*b.ny - a.ny*b.nx
	if math.Abs(cross) > 1e-9 {
		return
	}
	if a.nx*b.nx+a.ny*b.ny > -1+1e-9 {
		return // same-facing parallels never oppose across material
	}
	d := a.nx*(b.ax-a.ax) + a.ny*(b.ay-a.ay)
	if d <= 0 {
		return // b is not on a's material side
	}
	// Overlap of the two segments along a's tangent.
	tx, ty := -a.ny, a.nx
	a0, a1 := tx*a.ax+ty*a.ay, tx*a.bx+ty*a.by
	b0, b1 := tx*b.ax+ty*b.ay, tx*b.bx+ty*b.by
	lo := math.Max(math.Min(a0, a1), math.Min(b0, b1))
	hi := math.Min(math.Max(a0, a1), math.Max(b0, b1))
	if lo > hi {
		return
	}
	m := (lo + hi) / 2
	// Base point on a at parameter m, then half-way toward b.
	base := tx*a.ax + ty*a.ay
	px := a.ax + (m-base)*tx + a.nx*d/2
	py := a.ay + (m-base)*ty + a.ny*d/2
	add(px, py, d/2)
}

// lineArcCands: the critical disks sit on the perpendicular from the arc's
// center to the line; the angle-limit disks sit where the contact pair
// reaches exactly the allowance boundary.
func (k *wallKernel) lineArcCands(l, a surveyElem, add func(x, y, r float64)) {
	s := a.matSign()
	e := l.nx*(a.qx-l.ax) + l.ny*(a.qy-l.ay) // signed height of q, material side positive
	fx := a.qx - e*l.nx
	fy := a.qy - e*l.ny
	// T2 on the perpendicular from q to the line, t = r, center at f + t·n̂:
	// |e − t| = R − s·t, enumerated as e − t = sgn·(R − s·t).
	for _, sgn := range []float64{1, -1} {
		den := sgn*s - 1
		if math.Abs(den) < survTiny {
			continue
		}
		t := (sgn*a.rr - e) / den
		if t > 0 {
			add(fx+t*l.nx, fy+t*l.ny, t)
		}
	}
	// T3: contact directions exactly π − α apart. The line contact direction
	// is −n̂; the arc contact direction is s·(ĉ−q̂); c = q + (s·R − r)·u2.
	aStar := math.Pi - k.alpha
	for _, rot := range []float64{aStar, -aStar} {
		cs, sn := math.Cos(rot), math.Sin(rot)
		// u2 = rotate(−n̂, rot)
		ux := -(l.nx*cs - l.ny*sn)
		uy := -(l.nx*sn + l.ny*cs)
		ndot := l.nx*ux + l.ny*uy // = −cos(aStar)
		den := 1 + ndot
		if math.Abs(den) < survTiny {
			continue
		}
		r := (l.nx*(a.qx-l.ax) + l.ny*(a.qy-l.ay) + s*a.rr*ndot) / den
		if r > 0 {
			add(a.qx+(s*a.rr-r)*ux, a.qy+(s*a.rr-r)*uy, r)
		}
	}
}

// arcArcCands: centerline criticals (T2, the concentric family included) and
// the law-of-cosines angle-limit disks (T3).
func (k *wallKernel) arcArcCands(a, b surveyElem, add func(x, y, r float64)) {
	dx, dy := b.qx-a.qx, b.qy-a.qy
	d := math.Hypot(dx, dy)
	sa, sb := a.matSign(), b.matSign()
	if d <= survTiny*k.scale {
		// Concentric: the family disk at each arc's own angular midpoint.
		r := math.Abs(a.rr-b.rr) / 2
		m := (a.rr + b.rr) / 2
		for _, e := range []surveyElem{a, b} {
			lo, hi := e.arcRange()
			th := (lo + hi) / 2
			add(a.qx+m*math.Cos(th), a.qy+m*math.Sin(th), r)
		}
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the centerline: εa(Ra − sa·r) + εb(Rb − sb·r) = d.
	for _, ea := range []float64{1, -1} {
		for _, eb := range []float64{1, -1} {
			den := -ea*sa - eb*sb
			if math.Abs(den) < survTiny {
				continue
			}
			r := (d - ea*a.rr - eb*b.rr) / den
			if r <= 0 {
				continue
			}
			t := ea * (a.rr - sa*r)
			add(a.qx+t*ux, a.qy+t*uy, r)
		}
	}
	// T3: angle at the center between (c−qa) and (c−qb) fixed by the
	// allowance boundary; law of cosines in r, then the two mirror centers.
	cosTh := sa * sb * math.Cos(math.Pi-k.alpha)
	// d² = Da² + Db² − 2·Da·Db·cosθ with Da = Ra − sa·r, Db = Rb − sb·r.
	A := 2 - 2*cosTh*sa*sb
	B := -2*a.rr*sa - 2*b.rr*sb + 2*cosTh*(a.rr*sb+b.rr*sa)
	C := a.rr*a.rr + b.rr*b.rr - 2*a.rr*b.rr*cosTh - d*d
	for _, r := range quadRoots(A, B, C) {
		if r <= 0 {
			continue
		}
		da := a.rr - sa*r
		db := b.rr - sb*r
		placeCircleCircle(a.qx, a.qy, da, b.qx, b.qy, db, add, r)
	}
}

// vertexElemCands: the vertex-line foot and vertex-arc centerline criticals
// (T2) plus their angle-limit disks (T3).
func (k *wallKernel) vertexElemCands(v [2]float64, e surveyElem, add func(x, y, r float64)) {
	aStar := math.Pi - k.alpha
	if e.kind == surveyLine {
		// T2: the foot midpoint.
		h := e.nx*(v[0]-e.ax) + e.ny*(v[1]-e.ay)
		if h > 0 {
			add(v[0]-h/2*e.nx, v[1]-h/2*e.ny, h/2)
		}
		// T3: u_v = rotate(−n̂, ±A*), c = v − r·u_v, tangency fixes r.
		for _, rot := range []float64{aStar, -aStar} {
			cs, sn := math.Cos(rot), math.Sin(rot)
			ux := -(e.nx*cs - e.ny*sn)
			uy := -(e.nx*sn + e.ny*cs)
			den := 1 + (e.nx*ux + e.ny*uy)
			if math.Abs(den) < survTiny {
				continue
			}
			r := (e.nx*(v[0]-e.ax) + e.ny*(v[1]-e.ay)) / den
			if r > 0 {
				add(v[0]-r*ux, v[1]-r*uy, r)
			}
		}
		return
	}
	s := e.matSign()
	dx, dy := e.qx-v[0], e.qy-v[1]
	d := math.Hypot(dx, dy)
	if d <= survTiny*k.scale {
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the line through v and q: |c−v| = r with c = v ± r·û, and
	// |c−q| = R − s·r.
	for _, ev := range []float64{1, -1} {
		// c = v + ev·r·û: |c − q| = |d − ev·r| = R − s·r.
		for _, sgn := range []float64{1, -1} {
			den := sgn*s - ev
			if math.Abs(den) < survTiny {
				continue
			}
			r := (sgn*e.rr - d) / den
			if r > 0 {
				add(v[0]+ev*r*ux, v[1]+ev*r*uy, r)
			}
		}
	}
	// T3: law of cosines with sides r and R − s·r.
	cosTh := -s * math.Cos(aStar)
	A := 2 + 2*s*cosTh
	B := -2 * e.rr * (s + cosTh)
	C := e.rr*e.rr - d*d
	for _, r := range quadRoots(A, B, C) {
		if r <= 0 {
			continue
		}
		placeCircleCircle(v[0], v[1], r, e.qx, e.qy, e.rr-s*r, add, r)
	}
}

// vertexVertexCands: the midpoint disk (T2) and the isoceles angle-limit
// disks (T3).
func (k *wallKernel) vertexVertexCands(a, b [2]float64, add func(x, y, r float64)) {
	dx, dy := b[0]-a[0], b[1]-a[1]
	d := math.Hypot(dx, dy)
	if d <= survTiny*k.scale {
		return
	}
	add((a[0]+b[0])/2, (a[1]+b[1])/2, d/2)
	den := 2 * (1 - math.Cos(math.Pi-k.alpha))
	if den > survTiny {
		r := d / math.Sqrt(den)
		placeCircleCircle(a[0], a[1], r, b[0], b[1], r, add, r)
	}
}

// wedgeCands: the wedge-tangent minima for a partial revolve's cap-cap
// reading — the disk tangent to one element with the wedge constraint
// active (r = wedgeS·y), at its own closed-form ρ-critical.
func (k *wallKernel) wedgeCands(add func(x, y, r float64)) {
	s := k.wedgeS
	for _, v := range k.verts {
		if v[1] <= 0 {
			continue
		}
		for _, sgn := range []float64{1, -1} {
			den := 1 - sgn*s
			if den < survTiny {
				continue
			}
			r := s * v[1] / den
			if r > 0 {
				add(v[0], r/s, r)
			}
		}
	}
	for _, e := range k.elems {
		if e.kind != surveyArc {
			continue // a line's wedge-tangent family has no interior minimum
		}
		se := e.matSign()
		for _, th := range []float64{math.Pi / 2, -math.Pi / 2} {
			if !e.arcContains(th) {
				continue
			}
			sin := math.Sin(th)
			den := 1 + s*se*sin
			if math.Abs(den) < survTiny {
				continue
			}
			r := s * (e.qy + e.rr*sin) / den
			if r <= 0 {
				continue
			}
			d := e.rr - se*r
			add(e.qx+d*math.Cos(th), e.qy+d*math.Sin(th), r)
		}
	}
}

// circEq is one tangency equation for the Apollonius triples: either linear
// (a·cx + b·cy + e·r + f = 0) or quadratic
// (cx² + cy² − r² + g·cx + h·cy + kk·r + m = 0).
type circEq struct {
	quad        bool
	g, h, kk, m float64
	a, b, e, f  float64
}

// tripleEquations builds the material-side-pinned tangency equation of every
// item: elements, vertices, and the wedge.
func (k *wallKernel) tripleEquations() []circEq {
	var eqs []circEq
	for _, el := range k.elems {
		if el.kind == surveyLine {
			eqs = append(eqs, circEq{
				a: el.nx, b: el.ny, e: -1, f: -(el.nx*el.ax + el.ny*el.ay),
			})
			continue
		}
		s := el.matSign()
		eqs = append(eqs, circEq{
			quad: true,
			g:    -2 * el.qx, h: -2 * el.qy,
			kk: 2 * s * el.rr,
			m:  el.qx*el.qx + el.qy*el.qy - el.rr*el.rr,
		})
	}
	for _, v := range k.verts {
		eqs = append(eqs, circEq{
			quad: true,
			g:    -2 * v[0], h: -2 * v[1],
			m: v[0]*v[0] + v[1]*v[1],
		})
	}
	if k.wedgeS > 0 {
		eqs = append(eqs, circEq{b: k.wedgeS, e: -1})
	}
	return eqs
}

// solveTriple solves one triple of tangency equations in closed form:
// quadratic pairs subtract to linear, leaving at most one quadratic; two
// independent linears express the center affinely in r, and substitution
// yields a quadratic in r. Parallel linear pairs pin r instead and leave the
// position as the unknown.
func solveTriple(eqs [3]circEq, scale float64, add func(x, y, r float64)) {
	var lins []circEq
	var quad *circEq
	for i, e := range eqs {
		if !e.quad {
			lins = append(lins, e)
			continue
		}
		if quad == nil {
			q := eqs[i]
			quad = &q
			continue
		}
		// Subtract: the c·c − r² terms cancel.
		lins = append(lins, circEq{
			a: e.g - quad.g, b: e.h - quad.h, e: e.kk - quad.kk, f: e.m - quad.m,
		})
	}
	if len(lins) == 3 {
		solve3Linear(lins, add)
		return
	}
	if len(lins) != 2 || quad == nil {
		return
	}
	l1, l2 := lins[0], lins[1]
	det := l1.a*l2.b - l2.a*l1.b
	if math.Abs(det) <= 1e-12*math.Max(1, scale) {
		solveParallelPair(l1, l2, *quad, add)
		return
	}
	// (cx, cy) = P + r·Q from the two linears.
	px := (-l1.f*l2.b + l2.f*l1.b) / det
	py := (-l1.a*l2.f + l2.a*l1.f) / det
	qx := (-l1.e*l2.b + l2.e*l1.b) / det
	qy := (-l1.a*l2.e + l2.a*l1.e) / det
	A := qx*qx + qy*qy - 1
	B := 2*(px*qx+py*qy) + quad.g*qx + quad.h*qy + quad.kk
	C := px*px + py*py + quad.g*px + quad.h*py + quad.m
	for _, r := range quadRoots(A, B, C) {
		if r > 0 {
			add(px+r*qx, py+r*qy, r)
		}
	}
}

// solve3Linear solves three linear tangency equations by Cramer's rule.
func solve3Linear(l []circEq, add func(x, y, r float64)) {
	det := l[0].a*(l[1].b*l[2].e-l[2].b*l[1].e) -
		l[0].b*(l[1].a*l[2].e-l[2].a*l[1].e) +
		l[0].e*(l[1].a*l[2].b-l[2].a*l[1].b)
	if math.Abs(det) < survTiny {
		return
	}
	dx := -l[0].f*(l[1].b*l[2].e-l[2].b*l[1].e) +
		l[0].b*(l[1].f*l[2].e-l[2].f*l[1].e) -
		l[0].e*(l[1].f*l[2].b-l[2].f*l[1].b)
	dy := -l[0].a*(l[1].f*l[2].e-l[2].f*l[1].e) +
		l[0].f*(l[1].a*l[2].e-l[2].a*l[1].e) -
		l[0].e*(l[1].a*l[2].f-l[2].a*l[1].f)
	dr := -l[0].a*(l[1].b*l[2].f-l[2].b*l[1].f) +
		l[0].b*(l[1].a*l[2].f-l[2].a*l[1].f) -
		l[0].f*(l[1].a*l[2].b-l[2].a*l[1].b)
	r := dr / det
	if r > 0 {
		add(dx/det, dy/det, r)
	}
}

// solveParallelPair handles two parallel linear tangency equations plus a
// quadratic: the pair fixes r and a line of centers; the quadratic picks the
// positions along it.
func solveParallelPair(l1, l2, q circEq, add func(x, y, r float64)) {
	n := math.Hypot(l1.a, l1.b)
	if n < survTiny {
		return
	}
	// Normalize both to unit normals; solve the 2×2 system in (h, r) where
	// h = n̂·c along l1's normal.
	a1, b1, e1, f1 := l1.a/n, l1.b/n, l1.e/n, l1.f/n
	n2 := math.Hypot(l2.a, l2.b)
	if n2 < survTiny {
		return
	}
	a2, b2, e2, f2 := l2.a/n2, l2.b/n2, l2.e/n2, l2.f/n2
	// n̂2 = ±n̂1: sign σ.
	sigma := a1*a2 + b1*b2
	if sigma > 0 {
		sigma = 1
	} else {
		sigma = -1
	}
	// eq1: h + e1·r + f1 = 0; eq2: σ·h + e2·r + f2 = 0.
	det := e2 - sigma*e1
	if math.Abs(det) < survTiny {
		return
	}
	r := (sigma*f1 - f2) / det
	if r <= 0 {
		return
	}
	h := -e1*r - f1
	// Centers: c = h·n̂1 + t·t̂1. Substitute into the quadratic.
	tx, ty := -b1, a1
	bx, by := h*a1, h*b1
	A := 1.0
	B := 2*(bx*tx+by*ty) + q.g*tx + q.h*ty
	C := bx*bx + by*by - r*r + q.g*bx + q.h*by + q.kk*r + q.m
	for _, t := range quadRoots(A, B, C) {
		add(bx+t*tx, by+t*ty, r)
	}
}

// quadRoots returns the real roots of A·x² + B·x + C = 0 (both when A ≈ 0 —
// the linear case — and the full quadratic).
func quadRoots(A, B, C float64) []float64 {
	if math.Abs(A) < survTiny {
		if math.Abs(B) < survTiny {
			return nil
		}
		return []float64{-C / B}
	}
	disc := B*B - 4*A*C
	if disc < 0 {
		return nil
	}
	s := math.Sqrt(disc)
	return []float64{(-B - s) / (2 * A), (-B + s) / (2 * A)}
}

// placeCircleCircle emits the (up to two) points at distance da from
// (ax, ay) and db from (bx, by), as candidate centers of radius r.
func placeCircleCircle(ax, ay, da, bx, by, db float64, add func(x, y, r float64), r float64) {
	dx, dy := bx-ax, by-ay
	d := math.Hypot(dx, dy)
	if d < survTiny || da < 0 || db < 0 {
		return
	}
	a := (da*da - db*db + d*d) / (2 * d)
	h2 := da*da - a*a
	if h2 < 0 {
		return
	}
	h := math.Sqrt(h2)
	mx, my := ax+a*dx/d, ay+a*dy/d
	px, py := -dy/d, dx/d
	add(mx+h*px, my+h*py, r)
	if h > 0 {
		add(mx-h*px, my-h*py, r)
	}
}
