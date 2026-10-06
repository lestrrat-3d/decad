package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
		cp := freeform.CircleParam{
			C: [3]float64{f.anchor.X, f.anchor.Y, f.anchor.Z},
			U: [3]float64{f.refU.X, f.refU.Y, f.refU.Z},
			V: [3]float64{f.refV.X, f.refV.Y, f.refV.Z},
			R: f.major,
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
	if !proofbound.FiniteVec(fa) || !proofbound.FiniteVec(fb) {
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
			cp := freeform.CircleParam{
				C: [3]float64{e.center.X, e.center.Y, e.center.Z},
				U: [3]float64{e.refU.X, e.refU.Y, e.refU.Z},
				V: [3]float64{e.refV.X, e.refV.Y, e.refV.Z},
				R: e.radius,
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
	cp := freeform.CircleParam{
		C: [3]float64{ec.center.X, ec.center.Y, ec.center.Z},
		U: [3]float64{ec.refU.X, ec.refU.Y, ec.refU.Z},
		V: [3]float64{ec.refV.X, ec.refV.Y, ec.refV.Z},
		R: ec.radius,
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

// principalCircleEdgeGap proves the minimum over two complete circles whose
// axes are exactly vertical and whose centers differ along one horizontal
// principal axis. Their facing radial points attain the minimum. The points
// returned for the candidate are rounded representatives; the exact rational
// separation and directed square-root interval certify the distance.
func (k *pairKernel) principalCircleEdgeGap(ea, eb *cEdge, sink *cellSink) bool {
	if !ea.ang.full || !eb.ang.full || !proofbound.FiniteVec(ea.center) || !proofbound.FiniteVec(eb.center) ||
		proofbound.IsNonFinite(ea.radius) || proofbound.IsNonFinite(eb.radius) || ea.radius <= 0 || eb.radius <= 0 ||
		ea.axis.X != 0 || ea.axis.Y != 0 || math.Abs(ea.axis.Z) != 1 ||
		eb.axis.X != 0 || eb.axis.Y != 0 || math.Abs(eb.axis.Z) != 1 {
		return false
	}
	var offset big.Rat
	var radial r3.Vec
	switch {
	case ea.center.Y == eb.center.Y && ea.center.X != eb.center.X:
		offset.Sub(proofarith.FloatRat(eb.center.X), proofarith.FloatRat(ea.center.X))
		radial = r3.NewVec(1, 0, 0)
	case ea.center.X == eb.center.X && ea.center.Y != eb.center.Y:
		offset.Sub(proofarith.FloatRat(eb.center.Y), proofarith.FloatRat(ea.center.Y))
		radial = r3.NewVec(0, 1, 0)
	default:
		return false
	}
	sign := float64(offset.Sign())
	offset.Abs(&offset)
	gap := new(big.Rat).Sub(&offset, new(big.Rat).Add(proofarith.FloatRat(ea.radius), proofarith.FloatRat(eb.radius)))
	if gap.Sign() <= 0 {
		return false
	}
	dz := new(big.Rat).Sub(proofarith.FloatRat(ea.center.Z), proofarith.FloatRat(eb.center.Z))
	square := new(big.Rat).Mul(gap, gap)
	square.Add(square, new(big.Rat).Mul(dz, dz))
	lo, hi := proofbound.RatSqrtDown(square), proofbound.RatSqrtUp(square)
	if lo <= k.tol || proofbound.IsNonFinite(hi) {
		return false
	}
	pa := ea.center.Add(radial.Scale(sign * ea.radius))
	pb := eb.center.Sub(radial.Scale(sign * eb.radius))
	if !proofbound.FiniteVec(pa) || !proofbound.FiniteVec(pb) {
		return false
	}
	sink.candidate(k, 1, lo, hi, lo == hi, pa, pb)
	return true
}

// circleCircleEE: complete principal-axis exterior circles have a direct
// minimum; coaxial circles use their constant closed form; the rest use P8.
func (k *pairKernel) circleCircleEE(ea, eb *cEdge, sink *cellSink) {
	if k.principalCircleEdgeGap(ea, eb, sink) {
		return
	}
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
// enumerator's budget through every face, edge, and planar trim scan. A
// face or edge whose box lies beyond the sink's best upper bound is pruned
// (cellSink.pruned).
func (k *pairKernel) vertexTier(budget *proofbound.WorkBudget, v r3.Vec, other *bodyGeom, sink *cellSink) error {
	at := [2]r3.Vec{v, v}
	for _, f := range other.faces {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.pruned(clrBoxDist(at, f.box)) {
			continue
		}
		if err := k.vertexFace(budget, v, f, sink); err != nil {
			return err
		}
	}
	for _, e := range other.edges {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.pruned(clrBoxDist(at, e.box)) {
			continue
		}
		k.vertexEdge(v, e, sink)
	}
	return nil
}

func (k *pairKernel) vertexFace(budget *proofbound.WorkBudget, v r3.Vec, f *cFace, sink *cellSink) error {
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

// rulingContact is one §6 tangential line contact the kernel certified:
// Plane × Cylinder along the tangent ruling, or parallel external
// Cylinder × Cylinder along the common ruling. The plane at offset along
// normal separates the two bodies, and the ruling segment between ends lies
// on both trimmed faces. The kernel keeps the faces and the exact
// feet so the contact layer can publish the manifold of
// docs/contact-geometry-design.md §4.5; Verify reads only the verdict.
type rulingContact struct {
	faceA, faceB *cFace
	// normal is the exact unit A-to-B normal of the separating plane. An
	// exactly unit vector with dyadic components is a signed coordinate
	// axis, so this is one.
	normal proofarith.DyV3
	offset proofarith.Dyadic
	// ends are the exact ruling ends in lexicographic coordinate order.
	ends [2]proofarith.DyV3
}

// rulingContactCertified scans the plane-cylinder and cylinder-cylinder
// face pairs for a §6 ruling certificate. The caller runs it only when both
// bodies' bodyGeom.delta are exactly zero, so every carrier value is the
// boundary it names and every comparison below is exact.
func (k *pairKernel) rulingContactCertified(ctx context.Context) (*rulingContact, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			var ruling *rulingContact
			switch {
			case fa.kind == ckPlane && fb.kind == ckCylinder:
				ruling = k.planeCylinderRuling(ctx, fa, fb, k.a.body, k.b.body, false)
			case fa.kind == ckCylinder && fb.kind == ckPlane:
				ruling = k.planeCylinderRuling(ctx, fb, fa, k.b.body, k.a.body, true)
			case fa.kind == ckCylinder && fb.kind == ckCylinder:
				ruling = k.cylinderPairRuling(ctx, fa, fb)
			}
			if ruling != nil {
				return ruling, nil
			}
		}
	}
	return nil, ctx.Err()
}

// planeCylinderRuling certifies the cylinder's complete tangent ruling on the
// plane face. The cylinder axis must parallel the plane at exactly its radius
// on the outward side, the tangent azimuth and the whole axial window must be
// on the cylinder face, and the whole ruling must lie inside the plane trim.
// The plane must separate the bodies' complete extents. cylinderFirst names
// the cylinder's body as A.
func (k *pairKernel) planeCylinderRuling(ctx context.Context, plane, cyl *cFace,
	planeBody, cylBody *Body, cylinderFirst bool) *rulingContact {
	n, okN := exactSignedAxis(plane.n)
	axis, okAxis := exactSignedAxis(cyl.axis)
	o, okO := dyVecOf(plane.o)
	anchor, okAnchor := dyVecOf(cyl.anchor)
	radius, okR := proofarith.DyOf(cyl.radius)
	lo, okLo := proofarith.DyOf(cyl.zWin.lo)
	hi, okHi := proofarith.DyOf(cyl.zWin.hi)
	if !okN || !okAxis || !okO || !okAnchor || !okR || !okLo || !okHi ||
		radius.Sign() <= 0 || proofarith.DyCmp(lo, hi) >= 0 ||
		!proofarith.DvDot(n, axis).IsZero() {
		return nil
	}
	offset := proofarith.DvDot(n, o)
	if proofarith.DyCmp(proofarith.DySubScalar(proofarith.DvDot(n, anchor), offset), radius) != 0 {
		return nil
	}
	if !k.tangentAzimuthAdmitted(cyl, plane.n.Scale(-1)) {
		return nil
	}
	base := proofarith.DvSub(anchor, dyScaleVec(n, radius))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, dyScaleVec(axis, lo)),
		proofarith.DvAdd(base, dyScaleVec(axis, hi)),
	}
	if !k.rulingInsidePlaneTrim(plane, ends) {
		return nil
	}
	_, planeHi, okPlane := payloadExtent(ctx, planeBody, plane.n)
	cylLo, _, okCyl := payloadExtent(ctx, cylBody, plane.n)
	if !okPlane || !okCyl {
		return nil
	}
	planeHiDy, okPlaneHi := proofarith.DyOf(planeHi)
	cylLoDy, okCylLo := proofarith.DyOf(cylLo)
	if !okPlaneHi || !okCylLo || proofarith.DyCmp(planeHiDy, offset) > 0 ||
		proofarith.DyCmp(cylLoDy, offset) < 0 {
		return nil
	}
	ruling := &rulingContact{faceA: plane, faceB: cyl, normal: n, offset: offset, ends: orderedRulingEnds(ends)}
	if cylinderFirst {
		ruling.faceA, ruling.faceB = cyl, plane
		ruling.normal = proofarith.DvSub(proofarith.DyV3{}, n)
		ruling.offset = proofarith.DyNeg(offset)
	}
	return ruling
}

// cylinderPairRuling certifies the common ruling of two parallel external
// cylinders. Their axes must be exactly parallel, their center offset must be
// the radius sum along one signed coordinate axis, both tangent azimuths must
// be on their faces, and the axial windows must overlap with positive
// length. The tangent plane between them must separate the complete extents.
func (k *pairKernel) cylinderPairRuling(ctx context.Context, ca, cb *cFace) *rulingContact {
	axisA, okAxisA := exactSignedAxis(ca.axis)
	axisB, okAxisB := exactSignedAxis(cb.axis)
	anchorA, okAnchorA := dyVecOf(ca.anchor)
	anchorB, okAnchorB := dyVecOf(cb.anchor)
	rA, okRA := proofarith.DyOf(ca.radius)
	rB, okRB := proofarith.DyOf(cb.radius)
	loA, okLoA := proofarith.DyOf(ca.zWin.lo)
	hiA, okHiA := proofarith.DyOf(ca.zWin.hi)
	loB, okLoB := proofarith.DyOf(cb.zWin.lo)
	hiB, okHiB := proofarith.DyOf(cb.zWin.hi)
	if !okAxisA || !okAxisB || !okAnchorA || !okAnchorB || !okRA || !okRB ||
		!okLoA || !okHiA || !okLoB || !okHiB || rA.Sign() <= 0 || rB.Sign() <= 0 ||
		!proofarith.DvIsZero(proofarith.DvCross(axisA, axisB)) {
		return nil
	}
	delta := proofarith.DvSub(anchorB, anchorA)
	along := proofarith.DvDot(delta, axisA)
	perp := proofarith.DvSub(delta, dyScaleVec(axisA, along))
	normal, ok := axisOfLength(perp, proofarith.DyAdd(rA, rB))
	if !ok {
		return nil
	}
	normalVec := dyAxisVec(normal)
	if !k.tangentAzimuthAdmitted(ca, normalVec) || !k.tangentAzimuthAdmitted(cb, normalVec.Scale(-1)) {
		return nil
	}
	// Map B's axial window into A's axis parameter.
	bLo, bHi := proofarith.DyAdd(along, loB), proofarith.DyAdd(along, hiB)
	if proofarith.DvDot(axisA, axisB).Sign() < 0 {
		bLo, bHi = proofarith.DySubScalar(along, hiB), proofarith.DySubScalar(along, loB)
	}
	lo, hi := dyMax(loA, bLo), dyMin(hiA, bHi)
	if proofarith.DyCmp(lo, hi) >= 0 {
		return nil
	}
	offset := proofarith.DyAdd(proofarith.DvDot(normal, anchorA), rA)
	_, aHi, okA := payloadExtent(ctx, k.a.body, normalVec)
	bLoExtent, _, okB := payloadExtent(ctx, k.b.body, normalVec)
	if !okA || !okB {
		return nil
	}
	aHiDy, okAHi := proofarith.DyOf(aHi)
	bLoDy, okBLo := proofarith.DyOf(bLoExtent)
	if !okAHi || !okBLo || proofarith.DyCmp(aHiDy, offset) > 0 || proofarith.DyCmp(bLoDy, offset) < 0 {
		return nil
	}
	base := proofarith.DvAdd(anchorA, dyScaleVec(normal, rA))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, dyScaleVec(axisA, lo)),
		proofarith.DvAdd(base, dyScaleVec(axisA, hi)),
	}
	return &rulingContact{faceA: ca, faceB: cb, normal: normal, offset: offset, ends: orderedRulingEnds(ends)}
}

// tangentAzimuthAdmitted reports whether the cylinder face's angular trim
// holds the ruling whose outward radial direction is dir, with margin.
func (k *pairKernel) tangentAzimuthAdmitted(f *cFace, dir r3.Vec) bool {
	phi := math.Atan2(dir.Dot(f.refV), dir.Dot(f.refU))
	return f.sweep.classify(phi, k.tol/math.Max(f.radius, 1e-30)) == 1
}

// rulingInsidePlaneTrim admits the whole segment into the plane face's trim:
// one end strictly inside with margin and the segment clear of every trim
// boundary element by more than the margin, so it never leaves the region.
func (k *pairKernel) rulingInsidePlaneTrim(plane *cFace, ends [2]proofarith.DyV3) bool {
	var coords [2][2]float64
	for i, end := range ends {
		// The nearest float of each end is within an ulp; the trim margin
		// below is many orders wider.
		p := dyAxisVec(end)
		if !proofbound.FiniteVec(p) {
			return false
		}
		coords[i][0], coords[i][1] = plane.planeCoords(p)
	}
	if plane.region.classify(coords[0][0], coords[0][1], k.tol) != 1 {
		return false
	}
	for _, e := range plane.region.elems {
		if segElemDistLB(e, coords[0][0], coords[0][1], coords[1][0], coords[1][1]) <= k.tol {
			return false
		}
	}
	return true
}

// exactSignedAxis lifts a carrier direction that is exactly a signed
// coordinate axis.
func exactSignedAxis(v r3.Vec) (proofarith.DyV3, bool) {
	if _, _, ok := signedAxis(v); !ok {
		return proofarith.DyV3{}, false
	}
	return proofarith.DyVec(v), true
}

// axisOfLength returns v/length when v is exactly ±length along one
// coordinate axis and zero on the other two.
func axisOfLength(v proofarith.DyV3, length proofarith.Dyadic) (proofarith.DyV3, bool) {
	var unit proofarith.DyV3
	found := false
	for i := range 3 {
		switch {
		case v[i].IsZero():
			continue
		case found:
			return proofarith.DyV3{}, false
		case proofarith.DyCmp(v[i], length) == 0:
			unit[i] = proofarith.DyInt(1)
		case proofarith.DyCmp(v[i], proofarith.DyNeg(length)) == 0:
			unit[i] = proofarith.DyInt(-1)
		default:
			return proofarith.DyV3{}, false
		}
		found = true
	}
	return unit, found
}

// dyVecOf lifts a finite carrier vector exactly.
func dyVecOf(v r3.Vec) (proofarith.DyV3, bool) {
	if !proofbound.FiniteVec(v) {
		return proofarith.DyV3{}, false
	}
	return proofarith.DyVec(v), true
}

// dyAxisVec converts an exact vector to its nearest float vector; a signed
// coordinate axis converts exactly.
func dyAxisVec(v proofarith.DyV3) r3.Vec {
	var out r3.Vec
	out.X, _ = v[0].Float64()
	out.Y, _ = v[1].Float64()
	out.Z, _ = v[2].Float64()
	return out
}

func orderedRulingEnds(ends [2]proofarith.DyV3) [2]proofarith.DyV3 {
	for i := range 3 {
		switch proofarith.DyCmp(ends[0][i], ends[1][i]) {
		case -1:
			return ends
		case 1:
			return [2]proofarith.DyV3{ends[1], ends[0]}
		}
	}
	return ends
}
