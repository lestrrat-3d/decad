package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is the curve and vertex half of the §3 tier enumeration: the
// face × edge-interior, edge × edge and vertex tiers, over the shipped
// Line3/Circle3/Arc3 edge carriers and the five face carriers — every cell
// closed form or a P4/P8 certified bracket per §4's curve-tier table, with
// Line3 × Cone and Circle3 × Cone taken by the coarse enclosure.
// Candidates are computed on the unbounded carriers and admitted by
// the edge's parameter range and the face's trim exactly as §3 demands.
//
// Every constant-distance family in this file (a line parallel to a plane or
// axis, a circle whose plane parallels the face's, a coaxial cap edge, a
// parallel segment pair) is emitted only on the degeneracy oracle's degYes:
// these objectives are affine along the carrier, so a degNo migrates the
// minimum to the trim boundary, where the endpoint/vertex tiers hold it
// exactly, and a degUnknown owes an honest lower bound, a coarse enclosure,
// or unsure — never a plateau.

// edgeWits returns on-edge sample points.
func edgeWits(e *cEdge) []r3.Vec {
	if e.line {
		return []r3.Vec{e.a, e.b, e.a.Add(e.b).Scale(0.5)}
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.ang.full {
		lo, hi = e.ang.lo, e.ang.hi
	}
	return []r3.Vec{e.at(lo), e.at((lo + hi) / 2)}
}

// lineParamAdmit classifies a point (assumed on the edge's carrier line)
// against the segment's parameter range.
func lineParamAdmit(e *cEdge, p r3.Vec, tol float64) int {
	dir := e.b.Sub(e.a)
	l := dir.Len()
	u, _ := dir.Normalize()
	return linWindow{lo: 0, hi: l}.classify(p.Sub(e.a).Dot(u), tol)
}

// circleAngleAdmit classifies a carrier angle against the arc's window.
func circleAngleAdmit(e *cEdge, th, tol float64) int {
	return e.ang.classify(th, tol/math.Max(e.radius, 1e-30))
}

// feCell dispatches one face × edge pair through §4's curve-tier table.
func (k *pairKernel) feCell(f *cFace, e *cEdge, sink *cellSink) {
	switch f.kind {
	case ckPlane:
		if e.line {
			k.linePlaneFE(f, e, sink)
			return
		}
		k.circlePlaneFE(f, e, sink)
	case ckCone:
		// Line3 × Cone and Circle3 × Cone take the coarse enclosure here.
		sink.coarse(f.box, e.box, f.wit, edgeWits(e))
	default:
		if f.kind == ckTorus && f.spindle {
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
		if e.line {
			k.lineOffsetFE(f, e, sink)
			return
		}
		k.circleOffsetFE(f, e, sink)
	}
}

// linePlaneFE: a line EXACTLY parallel to the plane carries a
// constant-distance family admitted through the projected segment; a line
// provenly crossing it is excluded through the trims or routed as a contact —
// its distance is affine along the segment, so the minimum sits at an endpoint
// and the vertex tier holds it exactly. A tilt the oracle cannot rule out gets
// neither reading: the plateau would be an Exact the true minimum undercuts.
func (k *pairKernel) linePlaneFE(f *cFace, e *cEdge, sink *cellSink) {
	dir := e.b.Sub(e.a)
	l := dir.Len()
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	s := f.n.Dot(u)
	switch k.perpendicularSeg(e.a, e.b, f.n) {
	case degYes:
		h := e.a.Sub(f.o).Dot(f.n)
		x0, y0 := f.planeCoords(e.a.Sub(f.n.Scale(h)))
		x1, y1 := f.planeCoords(e.b.Sub(f.n.Scale(h)))
		hit, w := f.region.segmentHits(x0, y0, x1, y1)
		if hit == -1 {
			return
		}
		if math.Abs(h) <= k.tol {
			sink.unsure = true // the edge meets the face's own plane inside the trim (or ambiguously)
			return
		}
		pa := f.o.Add(f.u.Scale(w[0])).Add(f.v.Scale(w[1]))
		sink.candidate(k, hit, math.Abs(h), math.Abs(h), true, pa, pa.Add(f.n.Scale(h)))
		return
	case degUnknown:
		if clrBoxDist(f.box, e.box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	// The crossing point of the carrier with the plane.
	t := (f.planeOffset() - f.n.Dot(e.a)) / s
	if (linWindow{lo: 0, hi: l}).classify(t, k.tol) == -1 {
		return
	}
	pt := e.a.Add(u.Scale(t))
	x, y := f.planeCoords(pt)
	if f.region.classify(x, y, k.tol) == -1 {
		return
	}
	sink.unsure = true
}

// circlePlaneFE: the circle's signed plane height is a first harmonic —
// closed-form criticals, closed-form crossing roots. The constant-height
// reading belongs to the circle whose own plane is EXACTLY parallel to the
// face's (its axis parallel to the normal); decided on an amplitude epsilon it
// would report abs(base) Exact where the true minimum is abs(base) − the
// amplitude.
func (k *pairKernel) circlePlaneFE(f *cFace, e *cEdge, sink *cellSink) {
	base := e.center.Sub(f.o).Dot(f.n)
	hu := e.radius * f.n.Dot(e.refU)
	hv := e.radius * f.n.Dot(e.refV)
	h := func(th float64) float64 { return base + hu*math.Cos(th) + hv*math.Sin(th) }
	parallel := k.parallel(f.n, e.axis)
	if parallel == degUnknown {
		// A tilt too small to prove or disprove: the extremal azimuth the
		// criticals below rest on is not resolvable, and the plateau is not
		// certifiable. Undecided, never a guess.
		if clrBoxDist(f.box, e.box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	if parallel == degYes {
		// The circle parallels the plane: constant height.
		if math.Abs(base) <= k.tol {
			// In the carrier plane itself: a contact exactly when the circle
			// meets the trim (the full circle is a sound superset of an arc).
			cx, cy := f.planeCoords(e.center)
			if circleRegionHits(f.region, cx, cy, e.radius) != -1 {
				sink.unsure = true
			}
			return
		}
		mid := 0.0
		if !e.ang.full {
			mid = (e.ang.lo + e.ang.hi) / 2
		}
		for _, th := range []float64{mid, mid + math.Pi/2, mid + math.Pi, mid + 3*math.Pi/2} {
			admit := circleAngleAdmit(e, th, k.tol)
			if admit == -1 {
				continue
			}
			pe := e.at(th)
			foot := pe.Sub(f.n.Scale(base))
			x, y := f.planeCoords(foot)
			sink.candidate(k, admitState(admit, f.region.classify(x, y, k.tol)), math.Abs(base), math.Abs(base), true, foot, pe)
		}
		return
	}
	star := math.Atan2(hv, hu)
	for _, th := range []float64{star, star + math.Pi} {
		admit := circleAngleAdmit(e, th, k.tol)
		if admit == -1 {
			continue
		}
		pe := e.at(th)
		hh := h(th)
		foot := pe.Sub(f.n.Scale(hh))
		x, y := f.planeCoords(foot)
		admit = admitState(admit, f.region.classify(x, y, k.tol))
		sink.candidate(k, admit, math.Abs(hh), math.Abs(hh), true, foot, pe)
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.ang.full {
		lo, hi = e.ang.lo, e.ang.hi
	}
	mn, mx := trigRange(hu, hv, lo, hi)
	if base+mn > k.tol || base+mx < -k.tol {
		return // the arc never meets the carrier plane
	}
	// Crossing roots h(θ) = 0: excluded only when every root leaves a trim.
	amp := math.Hypot(hu, hv)
	if amp < 1e-30 {
		sink.unsure = true
		return
	}
	dth := math.Acos(math.Max(-1, math.Min(1, -base/amp)))
	for _, th := range []float64{star + dth, star - dth} {
		if circleAngleAdmit(e, th, k.tol) == -1 {
			continue
		}
		pt := e.at(th)
		x, y := f.planeCoords(pt)
		if f.region.classify(x, y, k.tol) == -1 {
			continue
		}
		sink.unsure = true
		return
	}
}

// feOffsetEmit emits the ± offset combinations of one curve-to-spine
// critical against an offset face.
func (k *pairKernel) feOffsetEmit(sink *cellSink, f *cFace, pe, spineFoot r3.Vec, dLo, dHi float64, exact bool, eAdmit int) {
	sep := pe.Sub(spineFoot)
	d := sep.Len()
	if d <= k.tol {
		sink.unsure = true
		return
	}
	dir := sep.Scale(1 / d)
	margin := k.tol + (dHi - dLo)
	for _, sf := range []float64{1, -1} {
		pf := spineFoot.Add(dir.Scale(sf * f.radius))
		rawLo, rawHi := dLo-sf*f.radius, dHi-sf*f.radius
		admit := admitState(eAdmit, f.admitPoint(pf, margin))
		if rawLo <= k.tol && rawHi >= -k.tol {
			if admit != -1 {
				sink.unsure = true
			}
			continue
		}
		lo, hi := math.Abs(rawLo), math.Abs(rawHi)
		if lo > hi {
			lo, hi = hi, lo
		}
		sink.candidate(k, admit, lo, hi, exact, pf, pe)
	}
}

// spineDistOf is the distance from a point to an offset face's spine, with
// the spine foot.
func spineDistOf(f *cFace, p r3.Vec) (float64, r3.Vec) {
	switch spineOf(f) {
	case 0:
		return p.Sub(f.anchor).Len(), f.anchor
	case 1:
		foot := linePoint(f.anchor, f.axis, p)
		return p.Sub(foot).Len(), foot
	default:
		rel := p.Sub(f.anchor)
		perp := rel.Sub(f.axis.Scale(rel.Dot(f.axis)))
		dir, ok := perp.Normalize()
		if !ok {
			dir = f.refU
		}
		foot := f.anchor.Add(dir.Scale(f.major))
		return p.Sub(foot).Len(), foot
	}
}

// feCrossingExcluded proves the trimmed edge never meets the offset face's
// carrier, from a superset range of the edge's spine distance (criticals
// plus endpoints — conservative in the sound direction).
func (k *pairKernel) feCrossingExcluded(f *cFace, e *cEdge, minLo, maxHi float64) bool {
	var pts []r3.Vec
	if e.line {
		pts = []r3.Vec{e.a, e.b}
	} else if !e.ang.full {
		pts = []r3.Vec{e.at(e.ang.lo), e.at(e.ang.hi)}
	}
	for _, p := range pts {
		d, _ := spineDistOf(f, p)
		minLo = math.Min(minLo, d)
		maxHi = math.Max(maxHi, d)
	}
	if minLo > f.radius+k.tol || maxHi < f.radius-k.tol {
		return true
	}
	return clrBoxDist(f.box, e.box) > k.tol
}

// lineOffsetFE: a segment against a cylinder, sphere or torus face — the
// line × spine criticals per §4's curve tiers (CF for line/point spines, P4
// for the torus spine).
func (k *pairKernel) lineOffsetFE(f *cFace, e *cEdge, sink *cellSink) {
	dir := e.b.Sub(e.a)
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	var crits []spineCrit
	switch spineOf(f) {
	case 0:
		foot := linePoint(e.a, u, f.anchor)
		crits = []spineCrit{exactCrit(foot, f.anchor)}
	case 1:
		switch k.parallelSeg(e.a, e.b, f.axis) {
		case degYes:
			// EXACTLY parallel to the axis: constant distance, represented at
			// the axial-overlap midpoint.
			aOnF := e.a.Sub(f.anchor).Dot(f.axis)
			bOnF := e.b.Sub(f.anchor).Dot(f.axis)
			ew := newLinWindow(aOnF, bOnF)
			lo := math.Max(ew.lo, f.zWin.lo)
			hi := math.Min(ew.hi, f.zWin.hi)
			z := (lo + hi) / 2
			if lo > hi {
				z = math.Max(ew.lo, math.Min(ew.hi, (f.zWin.lo+f.zWin.hi)/2))
			}
			axPt := f.anchor.Add(f.axis.Scale(z))
			pe := linePoint(e.a, u, axPt)
			crits = []spineCrit{exactCrit(pe, linePoint(f.anchor, f.axis, pe))}
		case degNo:
			cs, okp := k.lineLinePerp(e.a, u, f.anchor, f.axis)
			if !okp {
				sink.coarse(f.box, e.box, f.wit, edgeWits(e))
				return
			}
			crits = []spineCrit{cs}
		default:
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
	default:
		cp := circleParam{
			c: [3]float64{f.anchor.X, f.anchor.Y, f.anchor.Z},
			u: [3]float64{f.refU.X, f.refU.Y, f.refU.Z},
			v: [3]float64{f.refV.X, f.refV.Y, f.refV.Z},
			r: f.major,
		}
		var okc bool
		// The bracket feet come back as (line foot, spine point) — exactly
		// the (edge point, spine foot) order the emit helper reads.
		crits, okc = k.lineCircleBracketCrits(cp, f.anchor, f.refU, f.refV, e.a, u)
		if !okc {
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
	}
	minLo, maxHi := math.Inf(1), math.Inf(-1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.lo)
		maxHi = math.Max(maxHi, c.hi)
		k.feOffsetEmit(sink, f, c.fa, c.fb, c.lo, c.hi, c.exact, lineParamAdmit(e, c.fa, k.tol))
	}
	if !k.feCrossingExcluded(f, e, minLo, maxHi) {
		sink.unsure = true
	}
}

// lineLinePerp is the common perpendicular critical between a segment carrier
// and a spine line — for lines the oracle has already proven NOT parallel. The
// solve still owes a guard: cancellation can drive the denominator to zero (or
// the feet to infinity) on a pair the oracle only just separated, and a
// non-finite foot is no critical at all. ok is false there — the caller owes
// an enclosure, never a fabricated critical.
func (k *pairKernel) lineLinePerp(a, u, b, v r3.Vec) (spineCrit, bool) {
	rel := b.Sub(a)
	uv := u.Dot(v)
	den := 1 - uv*uv
	if !(den > 0) {
		return spineCrit{}, false
	}
	ru := rel.Dot(u)
	rv := rel.Dot(v)
	s := (ru - uv*rv) / den
	t := (uv*ru - rv) / den
	fa := a.Add(u.Scale(s))
	fb := b.Add(v.Scale(t))
	if !finiteVec(fa) || !finiteVec(fb) {
		return spineCrit{}, false
	}
	return exactCrit(fa, fb), true
}

// circleOffsetFE: a circular edge against a cylinder, sphere or torus face —
// circle × point (CF), circle × axis (P4, with the perpendicular-plane and
// coaxial cases closed form), circle × spine circle (P8).
func (k *pairKernel) circleOffsetFE(f *cFace, e *cEdge, sink *cellSink) {
	var crits []spineCrit
	switch spineOf(f) {
	case 0:
		cs, ok := k.pointCircleCrits(f.anchor, e.center, e.axis, e.refU, e.refV, e.radius, e.ang)
		if !ok {
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
		for i := range cs {
			cs[i].fa, cs[i].fb = cs[i].fb, cs[i].fa // (edge point, spine foot)
		}
		crits = cs
	case 1:
		switch k.parallel(e.axis, f.axis) {
		case degYes:
			// The circle's plane is EXACTLY perpendicular to the axis:
			// in-plane point-to-circle geometry, closed form.
			switch k.onAxis(e.center, f.anchor, f.axis) {
			case degYes:
				// Coaxial: constant distance — the peg-in-hole cap edge.
				th := 0.0
				if !e.ang.full {
					th = (e.ang.lo + e.ang.hi) / 2
				}
				pe := e.at(th)
				crits = []spineCrit{exactCrit(pe, linePoint(f.anchor, f.axis, pe))}
			case degNo:
				rel := e.center.Sub(f.anchor)
				perp := rel.Sub(f.axis.Scale(rel.Dot(f.axis)))
				dir, okd := perp.Normalize()
				if !okd {
					sink.coarse(f.box, e.box, f.wit, edgeWits(e))
					return
				}
				near := angleOf(e, dir.Scale(-1))
				far := angleOf(e, dir)
				for _, th := range []float64{near, far} {
					pe := e.at(th)
					crits = append(crits, exactCrit(pe, linePoint(f.anchor, f.axis, pe)))
				}
			default:
				sink.coarse(f.box, e.box, f.wit, edgeWits(e))
				return
			}
		case degNo:
			cp := circleParam{
				c: [3]float64{e.center.X, e.center.Y, e.center.Z},
				u: [3]float64{e.refU.X, e.refU.Y, e.refU.Z},
				v: [3]float64{e.refV.X, e.refV.Y, e.refV.Z},
				r: e.radius,
			}
			cs, ok := k.lineCircleBracketCrits(cp, e.center, e.refU, e.refV, f.anchor, f.axis)
			if !ok {
				sink.coarse(f.box, e.box, f.wit, edgeWits(e))
				return
			}
			for i := range cs {
				cs[i].fa, cs[i].fb = cs[i].fb, cs[i].fa // (edge point, axis foot)
			}
			crits = cs
		default:
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
	default:
		ef := &cFace{kind: ckTorus, anchor: e.center, axis: e.axis, refU: e.refU, refV: e.refV, major: e.radius}
		cs, ok := k.circleCircleCrits(ef, f)
		if !ok {
			sink.coarse(f.box, e.box, f.wit, edgeWits(e))
			return
		}
		crits = cs
	}
	minLo, maxHi := math.Inf(1), math.Inf(-1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.lo)
		maxHi = math.Max(maxHi, c.hi)
		th := angleOf(e, c.fa.Sub(e.center))
		k.feOffsetEmit(sink, f, c.fa, c.fb, c.lo, c.hi, c.exact, circleAngleAdmit(e, th, k.tol+(c.hi-c.lo)))
	}
	if !k.feCrossingExcluded(f, e, minLo, maxHi) {
		sink.unsure = true
	}
}

// angleOf is the carrier angle of a direction in the edge's frame.
func angleOf(e *cEdge, dir r3.Vec) float64 {
	return math.Atan2(dir.Dot(e.refV), dir.Dot(e.refU))
}

// eeCell dispatches one edge pair through §4's curve tiers.
func (k *pairKernel) eeCell(ea, eb *cEdge, sink *cellSink) {
	switch {
	case ea.line && eb.line:
		k.lineLineEE(ea, eb, sink)
	case ea.line:
		k.lineCircleEE(ea, eb, sink)
	case eb.line:
		k.lineCircleEE(eb, ea, sink)
	default:
		k.circleCircleEE(ea, eb, sink)
	}
}

// lineLineEE: parallel segments carry a constant family represented at the
// overlap midpoint; otherwise the single common perpendicular.
func (k *pairKernel) lineLineEE(ea, eb *cEdge, sink *cellSink) {
	da := ea.b.Sub(ea.a)
	db := eb.b.Sub(eb.a)
	ua, oka := da.Normalize()
	ub, okb := db.Normalize()
	if !oka || !okb {
		return
	}
	switch k.parallelSegs(ea.a, ea.b, eb.a, eb.b) {
	case degYes:
		// EXACTLY parallel: the constant family over the overlap of eb's
		// parameter range projected onto ea.
		p0 := eb.a.Sub(ea.a).Dot(ua)
		p1 := eb.b.Sub(ea.a).Dot(ua)
		w := newLinWindow(p0, p1)
		lo := math.Max(0, w.lo)
		hi := math.Min(da.Len(), w.hi)
		if hi-lo <= k.tol {
			return // endpoint tiers hold the minimum
		}
		t := (lo + hi) / 2
		pa := ea.a.Add(ua.Scale(t))
		pb := linePoint(eb.a, ub, pa)
		d := pa.Sub(pb).Len()
		sink.candidate(k, 1, d, d, true, pa, pb)
		return
	case degUnknown:
		// A tilt too small to prove or disprove: the constant family is not
		// certifiable and the common perpendicular is not resolvable.
		if clrBoxDist(ea.box, eb.box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	c, ok := k.lineLinePerp(ea.a, ua, eb.a, ub)
	if !ok {
		sink.coarse(ea.box, eb.box, edgeWits(ea), edgeWits(eb))
		return
	}
	d := c.fa.Sub(c.fb).Len()
	admit := admitState(lineParamAdmit(ea, c.fa, k.tol), lineParamAdmit(eb, c.fb, k.tol))
	sink.candidate(k, admit, d, d, true, c.fa, c.fb)
}

// lineCircleEE: the axis-parallel case is closed form; the general case is
// the P4 bracket.
func (k *pairKernel) lineCircleEE(el, ec *cEdge, sink *cellSink) {
	dir := el.b.Sub(el.a)
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	switch k.parallelSeg(el.a, el.b, ec.axis) {
	case degYes:
		// The segment is EXACTLY parallel to the circle's axis: in-plane
		// point-to-circle geometry, closed form.
		var ths []float64
		switch k.onAxis(el.a, ec.center, ec.axis) {
		case degYes:
			th := 0.0
			if !ec.ang.full {
				th = (ec.ang.lo + ec.ang.hi) / 2
			}
			ths = []float64{th}
		case degNo:
			rel := el.a.Sub(ec.center)
			perp := rel.Sub(ec.axis.Scale(rel.Dot(ec.axis)))
			dirP, okd := perp.Normalize()
			if !okd {
				sink.coarse(el.box, ec.box, edgeWits(el), edgeWits(ec))
				return
			}
			ths = []float64{angleOf(ec, dirP), angleOf(ec, dirP.Scale(-1))}
		default:
			sink.coarse(el.box, ec.box, edgeWits(el), edgeWits(ec))
			return
		}
		for _, th := range ths {
			pc := ec.at(th)
			pl := linePoint(el.a, u, pc)
			d := pc.Sub(pl).Len()
			admit := admitState(circleAngleAdmit(ec, th, k.tol), lineParamAdmit(el, pl, k.tol))
			sink.candidate(k, admit, d, d, true, pl, pc)
		}
		return
	case degUnknown:
		sink.coarse(el.box, ec.box, edgeWits(el), edgeWits(ec))
		return
	}
	cp := circleParam{
		c: [3]float64{ec.center.X, ec.center.Y, ec.center.Z},
		u: [3]float64{ec.refU.X, ec.refU.Y, ec.refU.Z},
		v: [3]float64{ec.refV.X, ec.refV.Y, ec.refV.Z},
		r: ec.radius,
	}
	crits, ok := k.lineCircleBracketCrits(cp, ec.center, ec.refU, ec.refV, el.a, u)
	if !ok {
		sink.coarse(el.box, ec.box, edgeWits(el), edgeWits(ec))
		return
	}
	for _, c := range crits {
		th := angleOf(ec, c.fb.Sub(ec.center))
		admit := admitState(lineParamAdmit(el, c.fa, k.tol), circleAngleAdmit(ec, th, k.tol+(c.hi-c.lo)))
		sink.candidate(k, admit, c.lo, c.hi, c.exact, c.fa, c.fb)
	}
}

// circleCircleEE: coaxial circles are the constant closed form; otherwise
// the P8 bracket.
func (k *pairKernel) circleCircleEE(ea, eb *cEdge, sink *cellSink) {
	fa := &cFace{kind: ckTorus, anchor: ea.center, axis: ea.axis, refU: ea.refU, refV: ea.refV, major: ea.radius}
	fb := &cFace{kind: ckTorus, anchor: eb.center, axis: eb.axis, refU: eb.refU, refV: eb.refV, major: eb.radius}
	crits, ok := k.circleCircleCrits(fa, fb)
	if !ok {
		sink.coarse(ea.box, eb.box, edgeWits(ea), edgeWits(eb))
		return
	}
	for _, c := range crits {
		tha := angleOf(ea, c.fa.Sub(ea.center))
		thb := angleOf(eb, c.fb.Sub(eb.center))
		admit := admitState(circleAngleAdmit(ea, tha, k.tol+(c.hi-c.lo)), circleAngleAdmit(eb, thb, k.tol+(c.hi-c.lo)))
		sink.candidate(k, admit, c.lo, c.hi, c.exact, c.fa, c.fb)
	}
}

// vertexTier runs one vertex (a topological vertex, a synthesized cone apex
// or a spindle axis-collapse point) against the other body's faces and
// edges — every cell closed form (§3's vertex tiers). It continues the
// enumerator's budget through every face, edge, and planar trim scan.
func (k *pairKernel) vertexTier(budget *workBudget, v r3.Vec, other *bodyGeom, sink *cellSink) error {
	for _, f := range other.faces {
		if err := budget.step(); err != nil {
			return err
		}
		if err := k.vertexFace(budget, v, f, sink); err != nil {
			return err
		}
	}
	for _, e := range other.edges {
		if err := budget.step(); err != nil {
			return err
		}
		k.vertexEdge(v, e, sink)
	}
	return nil
}

func (k *pairKernel) vertexFace(budget *workBudget, v r3.Vec, f *cFace, sink *cellSink) error {
	switch f.kind {
	case ckPlane:
		h := v.Sub(f.o).Dot(f.n)
		foot := v.Sub(f.n.Scale(h))
		x, y := f.planeCoords(foot)
		admit, err := regionClassifyBudget(budget, f.region, x, y, k.tol)
		if err != nil {
			return err
		}
		sink.candidate(k, admit, math.Abs(h), math.Abs(h), true, foot, v)
	case ckCone:
		rel := v.Sub(f.anchor)
		z := rel.Dot(f.axis)
		perp := rel.Sub(f.axis.Scale(z))
		rho := perp.Len()
		sinA, cosA := math.Sincos(f.half)
		var radial r3.Vec
		switch k.onAxis(v, f.anchor, f.axis) {
		case degYes:
			// Provenly on the axis: every azimuth carries the same distance, so
			// the sweep window's midpoint represents the family.
			mid := 0.0
			if !f.sweep.full {
				mid = (f.sweep.lo + f.sweep.hi) / 2
			}
			radial = f.refU.Scale(math.Cos(mid)).Add(f.refV.Scale(math.Sin(mid)))
		case degNo:
			radial, _ = perp.Normalize()
		default:
			// The distance is azimuth-free, the admission foot is not. The
			// carrier distance is a proven lower bound; it stands as one.
			sink.loOnly(math.Abs(rho*cosA - z*sinA))
			return nil
		}
		t := z*cosA + rho*sinA
		if t <= k.tol {
			return nil // the apex holds the nearest point; the vertex tiers pair with it
		}
		pf := f.anchor.Add(f.axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		d := math.Abs(rho*cosA - z*sinA)
		sink.candidate(k, f.admitPoint(pf, k.tol), d, d, true, pf, v)
	default:
		d, foot := spineDistOf(f, v)
		if d <= k.tol {
			sink.unsure = true
			return nil
		}
		dir := v.Sub(foot).Scale(1 / d)
		for _, sf := range []float64{1, -1} {
			pf := foot.Add(dir.Scale(sf * f.radius))
			raw := d - sf*f.radius
			sink.candidate(k, f.admitPoint(pf, k.tol), math.Abs(raw), math.Abs(raw), true, pf, v)
		}
	}
	return nil
}

func (k *pairKernel) vertexEdge(v r3.Vec, e *cEdge, sink *cellSink) {
	if e.line {
		dir := e.b.Sub(e.a)
		u, ok := dir.Normalize()
		if !ok {
			return
		}
		foot := linePoint(e.a, u, v)
		d := v.Sub(foot).Len()
		sink.candidate(k, lineParamAdmit(e, foot, k.tol), d, d, true, foot, v)
		return
	}
	crits, ok := k.pointCircleCrits(v, e.center, e.axis, e.refU, e.refV, e.radius, e.ang)
	if !ok {
		// The radial direction is not resolvable, so the arc's own admission
		// cannot be decided — but the distance to the WHOLE circle bounds the
		// distance to any arc of it from below, and that is a proof.
		rel := v.Sub(e.center)
		z := rel.Dot(e.axis)
		rho := rel.Sub(e.axis.Scale(z)).Len()
		sink.loOnly(math.Hypot(z, math.Abs(rho-e.radius)))
		return
	}
	for _, c := range crits {
		th := angleOf(e, c.fb.Sub(e.center))
		d := c.fa.Sub(c.fb).Len()
		sink.candidate(k, circleAngleAdmit(e, th, k.tol), d, d, true, c.fb, v)
	}
}
