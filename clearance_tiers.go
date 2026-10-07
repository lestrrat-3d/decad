package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/clearance"

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
// parallel segment pair) is emitted only on the degeneracy oracle's clearance.DegYes:
// these objectives are affine along the carrier, so a clearance.DegNo migrates the
// minimum to the trim boundary, where the endpoint/vertex tiers hold it
// exactly, and a clearance.DegUnknown owes an honest lower bound, a coarse enclosure,
// or unsure — never a plateau.

// feCell dispatches one face × edge pair through §4's curve-tier table.
func (k *pairKernel) feCell(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	switch f.Kind {
	case clearance.CkPlane:
		if e.Line {
			k.linePlaneFE(f, e, sink)
			return
		}
		k.circlePlaneFE(f, e, sink)
	case clearance.CkCone:
		// Line3 × Cone and Circle3 × Cone take the coarse enclosure here.
		sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
	default:
		if f.Kind == clearance.CkTorus && f.Spindle {
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
		if e.Line {
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
func (k *pairKernel) linePlaneFE(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	dir := e.B.Sub(e.A)
	l := dir.Len()
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	s := f.N.Dot(u)
	switch k.oracle().PerpendicularSeg(e.A, e.B, f.N) {
	case clearance.DegYes:
		h := e.A.Sub(f.O).Dot(f.N)
		x0, y0 := f.PlaneCoords(e.A.Sub(f.N.Scale(h)))
		x1, y1 := f.PlaneCoords(e.B.Sub(f.N.Scale(h)))
		hit, w := f.Region.SegmentHits(x0, y0, x1, y1)
		if hit == -1 {
			return
		}
		if math.Abs(h) <= k.tol {
			sink.unsure = true // the edge meets the face's own plane inside the trim (or ambiguously)
			return
		}
		pa := f.O.Add(f.U.Scale(w[0])).Add(f.V.Scale(w[1]))
		sink.candidate(k, hit, math.Abs(h), math.Abs(h), true, pa, pa.Add(f.N.Scale(h)))
		return
	case clearance.DegUnknown:
		if clearance.ClrBoxDist(f.Box, e.Box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	// The crossing point of the carrier with the plane.
	t := (f.PlaneOffset() - f.N.Dot(e.A)) / s
	if (clearance.LinWindow{Lo: 0, Hi: l}).Classify(t, k.tol) == -1 {
		return
	}
	pt := e.A.Add(u.Scale(t))
	x, y := f.PlaneCoords(pt)
	if f.Region.Classify(x, y, k.tol) == -1 {
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
func (k *pairKernel) circlePlaneFE(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	base := e.Center.Sub(f.O).Dot(f.N)
	hu := e.Radius * f.N.Dot(e.RefU)
	hv := e.Radius * f.N.Dot(e.RefV)
	h := func(th float64) float64 { return base + hu*math.Cos(th) + hv*math.Sin(th) }
	parallel := k.oracle().Parallel(f.N, e.Axis)
	if parallel == clearance.DegUnknown {
		// A tilt too small to prove or disprove: the extremal azimuth the
		// criticals below rest on is not resolvable, and the plateau is not
		// certifiable. Undecided, never a guess.
		if clearance.ClrBoxDist(f.Box, e.Box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	if parallel == clearance.DegYes {
		// The circle parallels the plane: constant height.
		if math.Abs(base) <= k.tol {
			// In the carrier plane itself: a contact exactly when the circle
			// meets the trim (the full circle is a sound superset of an arc).
			cx, cy := f.PlaneCoords(e.Center)
			if clearance.CircleRegionHits(f.Region, cx, cy, e.Radius) != -1 {
				sink.unsure = true
			}
			return
		}
		mid := 0.0
		if !e.Ang.Full {
			mid = (e.Ang.Lo + e.Ang.Hi) / 2
		}
		for _, th := range []float64{mid, mid + math.Pi/2, mid + math.Pi, mid + 3*math.Pi/2} {
			admit := clearance.CircleAngleAdmit(e, th, k.tol)
			if admit == -1 {
				continue
			}
			pe := e.At(th)
			foot := pe.Sub(f.N.Scale(base))
			x, y := f.PlaneCoords(foot)
			sink.candidate(k, clearance.AdmitState(admit, f.Region.Classify(x, y, k.tol)), math.Abs(base), math.Abs(base), true, foot, pe)
		}
		return
	}
	star := math.Atan2(hv, hu)
	for _, th := range []float64{star, star + math.Pi} {
		admit := clearance.CircleAngleAdmit(e, th, k.tol)
		if admit == -1 {
			continue
		}
		pe := e.At(th)
		hh := h(th)
		foot := pe.Sub(f.N.Scale(hh))
		x, y := f.PlaneCoords(foot)
		admit = clearance.AdmitState(admit, f.Region.Classify(x, y, k.tol))
		sink.candidate(k, admit, math.Abs(hh), math.Abs(hh), true, foot, pe)
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.Ang.Full {
		lo, hi = e.Ang.Lo, e.Ang.Hi
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
		if clearance.CircleAngleAdmit(e, th, k.tol) == -1 {
			continue
		}
		pt := e.At(th)
		x, y := f.PlaneCoords(pt)
		if f.Region.Classify(x, y, k.tol) == -1 {
			continue
		}
		sink.unsure = true
		return
	}
}

// feOffsetEmit emits the ± offset combinations of one curve-to-spine
// critical against an offset face.
func (k *pairKernel) feOffsetEmit(sink *cellSink, f *clearance.CFace, pe, spineFoot r3.Vec, dLo, dHi float64, exact bool, eAdmit int) {
	sep := pe.Sub(spineFoot)
	d := sep.Len()
	if d <= k.tol {
		sink.unsure = true
		return
	}
	dir := sep.Scale(1 / d)
	margin := k.tol + (dHi - dLo)
	for _, sf := range []float64{1, -1} {
		pf := spineFoot.Add(dir.Scale(sf * f.Radius))
		rawLo, rawHi := dLo-sf*f.Radius, dHi-sf*f.Radius
		admit := clearance.AdmitState(eAdmit, f.AdmitPoint(pf, margin))
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

// feCrossingExcluded proves the trimmed edge never meets the offset face's
// carrier, from a superset range of the edge's spine distance (criticals
// plus endpoints — conservative in the sound direction).
func (k *pairKernel) feCrossingExcluded(f *clearance.CFace, e *clearance.CEdge, minLo, maxHi float64) bool {
	var pts []r3.Vec
	if e.Line {
		pts = []r3.Vec{e.A, e.B}
	} else if !e.Ang.Full {
		pts = []r3.Vec{e.At(e.Ang.Lo), e.At(e.Ang.Hi)}
	}
	for _, p := range pts {
		d, _ := clearance.SpineDistOf(f, p)
		minLo = math.Min(minLo, d)
		maxHi = math.Max(maxHi, d)
	}
	if minLo > f.Radius+k.tol || maxHi < f.Radius-k.tol {
		return true
	}
	return clearance.ClrBoxDist(f.Box, e.Box) > k.tol
}

// lineOffsetFE: a segment against a cylinder, sphere or torus face — the
// line × spine criticals per §4's curve tiers (CF for line/point spines, P4
// for the torus spine).
func (k *pairKernel) lineOffsetFE(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	dir := e.B.Sub(e.A)
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	var crits []clearance.SpineCrit
	switch clearance.SpineOf(f) {
	case 0:
		foot := clearance.LinePoint(e.A, u, f.Anchor)
		crits = []clearance.SpineCrit{clearance.ExactCrit(foot, f.Anchor)}
	case 1:
		switch k.oracle().ParallelSeg(e.A, e.B, f.Axis) {
		case clearance.DegYes:
			// EXACTLY parallel to the axis: constant distance, represented at
			// the axial-overlap midpoint.
			aOnF := e.A.Sub(f.Anchor).Dot(f.Axis)
			bOnF := e.B.Sub(f.Anchor).Dot(f.Axis)
			ew := clearance.NewLinWindow(aOnF, bOnF)
			lo := math.Max(ew.Lo, f.ZWin.Lo)
			hi := math.Min(ew.Hi, f.ZWin.Hi)
			z := (lo + hi) / 2
			if lo > hi {
				z = math.Max(ew.Lo, math.Min(ew.Hi, (f.ZWin.Lo+f.ZWin.Hi)/2))
			}
			axPt := f.Anchor.Add(f.Axis.Scale(z))
			pe := clearance.LinePoint(e.A, u, axPt)
			crits = []clearance.SpineCrit{clearance.ExactCrit(pe, clearance.LinePoint(f.Anchor, f.Axis, pe))}
		case clearance.DegNo:
			cs, okp := k.lineLinePerp(e.A, u, f.Anchor, f.Axis)
			if !okp {
				sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
				return
			}
			crits = []clearance.SpineCrit{cs}
		default:
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
	default:
		cp := freeform.CircleParam{
			C: [3]float64{f.Anchor.X, f.Anchor.Y, f.Anchor.Z},
			U: [3]float64{f.RefU.X, f.RefU.Y, f.RefU.Z},
			V: [3]float64{f.RefV.X, f.RefV.Y, f.RefV.Z},
			R: f.Major,
		}
		var okc bool
		// The bracket feet come back as (line foot, spine point) — exactly
		// the (edge point, spine foot) order the emit helper reads.
		crits, okc = k.lineCircleBracketCrits(cp, f.Anchor, f.RefU, f.RefV, e.A, u)
		if !okc {
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
	}
	minLo, maxHi := math.Inf(1), math.Inf(-1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.Lo)
		maxHi = math.Max(maxHi, c.Hi)
		k.feOffsetEmit(sink, f, c.Fa, c.Fb, c.Lo, c.Hi, c.Exact, clearance.LineParamAdmit(e, c.Fa, k.tol))
	}
	if !k.feCrossingExcluded(f, e, minLo, maxHi) {
		sink.unsure = true
	}
}

func (k *pairKernel) lineLinePerp(a, u, b, v r3.Vec) (clearance.SpineCrit, bool) {
	e := k.spineEngine()
	return e.LineLinePerp(a, u, b, v)
}

// circleOffsetFE: a circular edge against a cylinder, sphere or torus face —
// circle × point (CF), circle × axis (P4, with the perpendicular-plane and
// coaxial cases closed form), circle × spine circle (P8).
func (k *pairKernel) circleOffsetFE(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	var crits []clearance.SpineCrit
	switch clearance.SpineOf(f) {
	case 0:
		cs, ok := k.pointCircleCrits(f.Anchor, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
		if !ok {
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
		for i := range cs {
			cs[i].Fa, cs[i].Fb = cs[i].Fb, cs[i].Fa // (edge point, spine foot)
		}
		crits = cs
	case 1:
		switch k.oracle().Parallel(e.Axis, f.Axis) {
		case clearance.DegYes:
			// The circle's plane is EXACTLY perpendicular to the axis:
			// in-plane point-to-circle geometry, closed form.
			switch k.oracle().OnAxis(e.Center, f.Anchor, f.Axis) {
			case clearance.DegYes:
				// Coaxial: constant distance — the peg-in-hole cap edge.
				th := 0.0
				if !e.Ang.Full {
					th = (e.Ang.Lo + e.Ang.Hi) / 2
				}
				pe := e.At(th)
				crits = []clearance.SpineCrit{clearance.ExactCrit(pe, clearance.LinePoint(f.Anchor, f.Axis, pe))}
			case clearance.DegNo:
				rel := e.Center.Sub(f.Anchor)
				perp := rel.Sub(f.Axis.Scale(rel.Dot(f.Axis)))
				dir, okd := perp.Normalize()
				if !okd {
					sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
					return
				}
				near := clearance.AngleOf(e, dir.Scale(-1))
				far := clearance.AngleOf(e, dir)
				for _, th := range []float64{near, far} {
					pe := e.At(th)
					crits = append(crits, clearance.ExactCrit(pe, clearance.LinePoint(f.Anchor, f.Axis, pe)))
				}
			default:
				sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
				return
			}
		case clearance.DegNo:
			cp := freeform.CircleParam{
				C: [3]float64{e.Center.X, e.Center.Y, e.Center.Z},
				U: [3]float64{e.RefU.X, e.RefU.Y, e.RefU.Z},
				V: [3]float64{e.RefV.X, e.RefV.Y, e.RefV.Z},
				R: e.Radius,
			}
			cs, ok := k.lineCircleBracketCrits(cp, e.Center, e.RefU, e.RefV, f.Anchor, f.Axis)
			if !ok {
				sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
				return
			}
			for i := range cs {
				cs[i].Fa, cs[i].Fb = cs[i].Fb, cs[i].Fa // (edge point, axis foot)
			}
			crits = cs
		default:
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
	default:
		ef := &clearance.CFace{Kind: clearance.CkTorus, Anchor: e.Center, Axis: e.Axis, RefU: e.RefU, RefV: e.RefV, Major: e.Radius}
		cs, ok := k.circleCircleCrits(ef, f)
		if !ok {
			sink.coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
		crits = cs
	}
	minLo, maxHi := math.Inf(1), math.Inf(-1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.Lo)
		maxHi = math.Max(maxHi, c.Hi)
		th := clearance.AngleOf(e, c.Fa.Sub(e.Center))
		k.feOffsetEmit(sink, f, c.Fa, c.Fb, c.Lo, c.Hi, c.Exact, clearance.CircleAngleAdmit(e, th, k.tol+(c.Hi-c.Lo)))
	}
	if !k.feCrossingExcluded(f, e, minLo, maxHi) {
		sink.unsure = true
	}
}

// eeCell dispatches one edge pair through §4's curve tiers.
func (k *pairKernel) eeCell(ea, eb *clearance.CEdge, sink *cellSink) {
	switch {
	case ea.Line && eb.Line:
		k.lineLineEE(ea, eb, sink)
	case ea.Line:
		k.lineCircleEE(ea, eb, sink)
	case eb.Line:
		k.lineCircleEE(eb, ea, sink)
	default:
		k.circleCircleEE(ea, eb, sink)
	}
}

// lineLineEE: parallel segments carry a constant family represented at the
// overlap midpoint; otherwise the single common perpendicular.
func (k *pairKernel) lineLineEE(ea, eb *clearance.CEdge, sink *cellSink) {
	da := ea.B.Sub(ea.A)
	db := eb.B.Sub(eb.A)
	ua, oka := da.Normalize()
	ub, okb := db.Normalize()
	if !oka || !okb {
		return
	}
	switch k.oracle().ParallelSegs(ea.A, ea.B, eb.A, eb.B) {
	case clearance.DegYes:
		// EXACTLY parallel: the constant family over the overlap of eb's
		// parameter range projected onto ea.
		p0 := eb.A.Sub(ea.A).Dot(ua)
		p1 := eb.B.Sub(ea.A).Dot(ua)
		w := clearance.NewLinWindow(p0, p1)
		lo := math.Max(0, w.Lo)
		hi := math.Min(da.Len(), w.Hi)
		if hi-lo <= k.tol {
			return // endpoint tiers hold the minimum
		}
		t := (lo + hi) / 2
		pa := ea.A.Add(ua.Scale(t))
		pb := clearance.LinePoint(eb.A, ub, pa)
		d := pa.Sub(pb).Len()
		sink.candidate(k, 1, d, d, true, pa, pb)
		return
	case clearance.DegUnknown:
		// A tilt too small to prove or disprove: the constant family is not
		// certifiable and the common perpendicular is not resolvable.
		if clearance.ClrBoxDist(ea.Box, eb.Box) > k.tol {
			return
		}
		sink.unsure = true
		return
	}
	c, ok := k.lineLinePerp(ea.A, ua, eb.A, ub)
	if !ok {
		sink.coarse(ea.Box, eb.Box, clearance.EdgeWits(ea), clearance.EdgeWits(eb))
		return
	}
	d := c.Fa.Sub(c.Fb).Len()
	admit := clearance.AdmitState(clearance.LineParamAdmit(ea, c.Fa, k.tol), clearance.LineParamAdmit(eb, c.Fb, k.tol))
	sink.candidate(k, admit, d, d, true, c.Fa, c.Fb)
}

// lineCircleEE: the axis-parallel case is closed form; the general case is
// the P4 bracket.
func (k *pairKernel) lineCircleEE(el, ec *clearance.CEdge, sink *cellSink) {
	dir := el.B.Sub(el.A)
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	switch k.oracle().ParallelSeg(el.A, el.B, ec.Axis) {
	case clearance.DegYes:
		// The segment is EXACTLY parallel to the circle's axis: in-plane
		// point-to-circle geometry, closed form.
		var ths []float64
		switch k.oracle().OnAxis(el.A, ec.Center, ec.Axis) {
		case clearance.DegYes:
			th := 0.0
			if !ec.Ang.Full {
				th = (ec.Ang.Lo + ec.Ang.Hi) / 2
			}
			ths = []float64{th}
		case clearance.DegNo:
			rel := el.A.Sub(ec.Center)
			perp := rel.Sub(ec.Axis.Scale(rel.Dot(ec.Axis)))
			dirP, okd := perp.Normalize()
			if !okd {
				sink.coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
				return
			}
			ths = []float64{clearance.AngleOf(ec, dirP), clearance.AngleOf(ec, dirP.Scale(-1))}
		default:
			sink.coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
			return
		}
		for _, th := range ths {
			pc := ec.At(th)
			pl := clearance.LinePoint(el.A, u, pc)
			d := pc.Sub(pl).Len()
			admit := clearance.AdmitState(clearance.CircleAngleAdmit(ec, th, k.tol), clearance.LineParamAdmit(el, pl, k.tol))
			sink.candidate(k, admit, d, d, true, pl, pc)
		}
		return
	case clearance.DegUnknown:
		sink.coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
		return
	}
	cp := freeform.CircleParam{
		C: [3]float64{ec.Center.X, ec.Center.Y, ec.Center.Z},
		U: [3]float64{ec.RefU.X, ec.RefU.Y, ec.RefU.Z},
		V: [3]float64{ec.RefV.X, ec.RefV.Y, ec.RefV.Z},
		R: ec.Radius,
	}
	crits, ok := k.lineCircleBracketCrits(cp, ec.Center, ec.RefU, ec.RefV, el.A, u)
	if !ok {
		sink.coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(ec, c.Fb.Sub(ec.Center))
		admit := clearance.AdmitState(clearance.LineParamAdmit(el, c.Fa, k.tol), clearance.CircleAngleAdmit(ec, th, k.tol+(c.Hi-c.Lo)))
		sink.candidate(k, admit, c.Lo, c.Hi, c.Exact, c.Fa, c.Fb)
	}
}

// principalCircleEdgeGap proves the minimum over two complete circles whose
// axes are exactly vertical and whose centers differ along one horizontal
// principal axis. Their facing radial points attain the minimum. The points
// returned for the candidate are rounded representatives; the exact rational
// separation and directed square-root interval certify the distance.
func (k *pairKernel) principalCircleEdgeGap(ea, eb *clearance.CEdge, sink *cellSink) bool {
	if !ea.Ang.Full || !eb.Ang.Full || !proofbound.FiniteVec(ea.Center) || !proofbound.FiniteVec(eb.Center) ||
		proofbound.IsNonFinite(ea.Radius) || proofbound.IsNonFinite(eb.Radius) || ea.Radius <= 0 || eb.Radius <= 0 ||
		ea.Axis.X != 0 || ea.Axis.Y != 0 || math.Abs(ea.Axis.Z) != 1 ||
		eb.Axis.X != 0 || eb.Axis.Y != 0 || math.Abs(eb.Axis.Z) != 1 {
		return false
	}
	var offset big.Rat
	var radial r3.Vec
	switch {
	case ea.Center.Y == eb.Center.Y && ea.Center.X != eb.Center.X:
		offset.Sub(proofarith.FloatRat(eb.Center.X), proofarith.FloatRat(ea.Center.X))
		radial = r3.NewVec(1, 0, 0)
	case ea.Center.X == eb.Center.X && ea.Center.Y != eb.Center.Y:
		offset.Sub(proofarith.FloatRat(eb.Center.Y), proofarith.FloatRat(ea.Center.Y))
		radial = r3.NewVec(0, 1, 0)
	default:
		return false
	}
	sign := float64(offset.Sign())
	offset.Abs(&offset)
	gap := new(big.Rat).Sub(&offset, new(big.Rat).Add(proofarith.FloatRat(ea.Radius), proofarith.FloatRat(eb.Radius)))
	if gap.Sign() <= 0 {
		return false
	}
	dz := new(big.Rat).Sub(proofarith.FloatRat(ea.Center.Z), proofarith.FloatRat(eb.Center.Z))
	square := new(big.Rat).Mul(gap, gap)
	square.Add(square, new(big.Rat).Mul(dz, dz))
	lo, hi := proofbound.RatSqrtDown(square), proofbound.RatSqrtUp(square)
	if lo <= k.tol || proofbound.IsNonFinite(hi) {
		return false
	}
	pa := ea.Center.Add(radial.Scale(sign * ea.Radius))
	pb := eb.Center.Sub(radial.Scale(sign * eb.Radius))
	if !proofbound.FiniteVec(pa) || !proofbound.FiniteVec(pb) {
		return false
	}
	sink.candidate(k, 1, lo, hi, lo == hi, pa, pb)
	return true
}

// circleCircleEE: complete principal-axis exterior circles have a direct
// minimum; coaxial circles use their constant closed form; the rest use P8.
func (k *pairKernel) circleCircleEE(ea, eb *clearance.CEdge, sink *cellSink) {
	if k.principalCircleEdgeGap(ea, eb, sink) {
		return
	}
	fa := &clearance.CFace{Kind: clearance.CkTorus, Anchor: ea.Center, Axis: ea.Axis, RefU: ea.RefU, RefV: ea.RefV, Major: ea.Radius}
	fb := &clearance.CFace{Kind: clearance.CkTorus, Anchor: eb.Center, Axis: eb.Axis, RefU: eb.RefU, RefV: eb.RefV, Major: eb.Radius}
	crits, ok := k.circleCircleCrits(fa, fb)
	if !ok {
		sink.coarse(ea.Box, eb.Box, clearance.EdgeWits(ea), clearance.EdgeWits(eb))
		return
	}
	for _, c := range crits {
		tha := clearance.AngleOf(ea, c.Fa.Sub(ea.Center))
		thb := clearance.AngleOf(eb, c.Fb.Sub(eb.Center))
		admit := clearance.AdmitState(clearance.CircleAngleAdmit(ea, tha, k.tol+(c.Hi-c.Lo)), clearance.CircleAngleAdmit(eb, thb, k.tol+(c.Hi-c.Lo)))
		sink.candidate(k, admit, c.Lo, c.Hi, c.Exact, c.Fa, c.Fb)
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
		if sink.pruned(clearance.ClrBoxDist(at, f.Box)) {
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
		if sink.pruned(clearance.ClrBoxDist(at, e.Box)) {
			continue
		}
		k.vertexEdge(v, e, sink)
	}
	return nil
}

func (k *pairKernel) vertexFace(budget *proofbound.WorkBudget, v r3.Vec, f *clearance.CFace, sink *cellSink) error {
	switch f.Kind {
	case clearance.CkPlane:
		h := v.Sub(f.O).Dot(f.N)
		foot := v.Sub(f.N.Scale(h))
		x, y := f.PlaneCoords(foot)
		admit, err := clearance.RegionClassifyBudget(budget, f.Region, x, y, k.tol)
		if err != nil {
			return err
		}
		sink.candidate(k, admit, math.Abs(h), math.Abs(h), true, foot, v)
	case clearance.CkCone:
		rel := v.Sub(f.Anchor)
		z := rel.Dot(f.Axis)
		perp := rel.Sub(f.Axis.Scale(z))
		rho := perp.Len()
		sinA, cosA := math.Sincos(f.Half)
		var radial r3.Vec
		switch k.oracle().OnAxis(v, f.Anchor, f.Axis) {
		case clearance.DegYes:
			// Provenly on the axis: every azimuth carries the same distance, so
			// the sweep window's midpoint represents the family.
			mid := 0.0
			if !f.Sweep.Full {
				mid = (f.Sweep.Lo + f.Sweep.Hi) / 2
			}
			radial = f.RefU.Scale(math.Cos(mid)).Add(f.RefV.Scale(math.Sin(mid)))
		case clearance.DegNo:
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
		pf := f.Anchor.Add(f.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		d := math.Abs(rho*cosA - z*sinA)
		sink.candidate(k, f.AdmitPoint(pf, k.tol), d, d, true, pf, v)
	default:
		d, foot := clearance.SpineDistOf(f, v)
		if d <= k.tol {
			sink.unsure = true
			return nil
		}
		dir := v.Sub(foot).Scale(1 / d)
		for _, sf := range []float64{1, -1} {
			pf := foot.Add(dir.Scale(sf * f.Radius))
			raw := d - sf*f.Radius
			sink.candidate(k, f.AdmitPoint(pf, k.tol), math.Abs(raw), math.Abs(raw), true, pf, v)
		}
	}
	return nil
}

func (k *pairKernel) vertexEdge(v r3.Vec, e *clearance.CEdge, sink *cellSink) {
	if e.Line {
		dir := e.B.Sub(e.A)
		u, ok := dir.Normalize()
		if !ok {
			return
		}
		foot := clearance.LinePoint(e.A, u, v)
		d := v.Sub(foot).Len()
		sink.candidate(k, clearance.LineParamAdmit(e, foot, k.tol), d, d, true, foot, v)
		return
	}
	crits, ok := k.pointCircleCrits(v, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
	if !ok {
		// The radial direction is not resolvable, so the arc's own admission
		// cannot be decided — but the distance to the WHOLE circle bounds the
		// distance to any arc of it from below, and that is a proof.
		rel := v.Sub(e.Center)
		z := rel.Dot(e.Axis)
		rho := rel.Sub(e.Axis.Scale(z)).Len()
		sink.loOnly(math.Hypot(z, math.Abs(rho-e.Radius)))
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(e, c.Fb.Sub(e.Center))
		d := c.Fa.Sub(c.Fb).Len()
		sink.candidate(k, clearance.CircleAngleAdmit(e, th, k.tol), d, d, true, c.Fb, v)
	}
}

// rulingContactCertified scans the plane-cylinder and cylinder-cylinder
// face pairs for a §6 ruling certificate. The caller runs it only when both
// bodies' bodyGeom.delta are exactly zero, so every carrier value is the
// boundary it names and every comparison below is exact.
func (k *pairKernel) rulingContactCertified(ctx context.Context) (*clearance.RulingContact, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			var ruling *clearance.RulingContact
			switch {
			case fa.Kind == clearance.CkPlane && fb.Kind == clearance.CkCylinder:
				ruling = k.planeCylinderRuling(ctx, fa, fb, k.a.body, k.b.body, false)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkPlane:
				ruling = k.planeCylinderRuling(ctx, fb, fa, k.b.body, k.a.body, true)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkCylinder:
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
func (k *pairKernel) planeCylinderRuling(ctx context.Context, plane, cyl *clearance.CFace,
	planeBody, cylBody *Body, cylinderFirst bool) *clearance.RulingContact {
	n, okN := clearance.ExactSignedAxis(plane.N)
	axis, okAxis := clearance.ExactSignedAxis(cyl.Axis)
	o, okO := clearance.DyVecOf(plane.O)
	anchor, okAnchor := clearance.DyVecOf(cyl.Anchor)
	radius, okR := proofarith.DyOf(cyl.Radius)
	lo, okLo := proofarith.DyOf(cyl.ZWin.Lo)
	hi, okHi := proofarith.DyOf(cyl.ZWin.Hi)
	if !okN || !okAxis || !okO || !okAnchor || !okR || !okLo || !okHi ||
		radius.Sign() <= 0 || proofarith.DyCmp(lo, hi) >= 0 ||
		!proofarith.DvDot(n, axis).IsZero() {
		return nil
	}
	offset := proofarith.DvDot(n, o)
	if proofarith.DyCmp(proofarith.DySubScalar(proofarith.DvDot(n, anchor), offset), radius) != 0 {
		return nil
	}
	if !k.tangentAzimuthAdmitted(cyl, plane.N.Scale(-1)) {
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
	_, planeHi, okPlane := payloadExtent(ctx, planeBody, plane.N)
	cylLo, _, okCyl := payloadExtent(ctx, cylBody, plane.N)
	if !okPlane || !okCyl {
		return nil
	}
	planeHiDy, okPlaneHi := proofarith.DyOf(planeHi)
	cylLoDy, okCylLo := proofarith.DyOf(cylLo)
	if !okPlaneHi || !okCylLo || proofarith.DyCmp(planeHiDy, offset) > 0 ||
		proofarith.DyCmp(cylLoDy, offset) < 0 {
		return nil
	}
	ruling := &clearance.RulingContact{FaceA: plane, FaceB: cyl, Normal: n, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
	if cylinderFirst {
		ruling.FaceA, ruling.FaceB = cyl, plane
		ruling.Normal = proofarith.DvSub(proofarith.DyV3{}, n)
		ruling.Offset = proofarith.DyNeg(offset)
	}
	return ruling
}

// cylinderPairRuling certifies the common ruling of two parallel external
// cylinders. Their axes must be exactly parallel, their center offset must be
// the radius sum along one signed coordinate axis, both tangent azimuths must
// be on their faces, and the axial windows must overlap with positive
// length. The tangent plane between them must separate the complete extents.
func (k *pairKernel) cylinderPairRuling(ctx context.Context, ca, cb *clearance.CFace) *clearance.RulingContact {
	axisA, okAxisA := clearance.ExactSignedAxis(ca.Axis)
	axisB, okAxisB := clearance.ExactSignedAxis(cb.Axis)
	anchorA, okAnchorA := clearance.DyVecOf(ca.Anchor)
	anchorB, okAnchorB := clearance.DyVecOf(cb.Anchor)
	rA, okRA := proofarith.DyOf(ca.Radius)
	rB, okRB := proofarith.DyOf(cb.Radius)
	loA, okLoA := proofarith.DyOf(ca.ZWin.Lo)
	hiA, okHiA := proofarith.DyOf(ca.ZWin.Hi)
	loB, okLoB := proofarith.DyOf(cb.ZWin.Lo)
	hiB, okHiB := proofarith.DyOf(cb.ZWin.Hi)
	if !okAxisA || !okAxisB || !okAnchorA || !okAnchorB || !okRA || !okRB ||
		!okLoA || !okHiA || !okLoB || !okHiB || rA.Sign() <= 0 || rB.Sign() <= 0 ||
		!proofarith.DvIsZero(proofarith.DvCross(axisA, axisB)) {
		return nil
	}
	delta := proofarith.DvSub(anchorB, anchorA)
	along := proofarith.DvDot(delta, axisA)
	perp := proofarith.DvSub(delta, dyScaleVec(axisA, along))
	normal, ok := clearance.AxisOfLength(perp, proofarith.DyAdd(rA, rB))
	if !ok {
		return nil
	}
	normalVec := clearance.DyAxisVec(normal)
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
	return &clearance.RulingContact{FaceA: ca, FaceB: cb, Normal: normal, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
}

// tangentAzimuthAdmitted reports whether the cylinder face's angular trim
// holds the ruling whose outward radial direction is dir, with margin.
func (k *pairKernel) tangentAzimuthAdmitted(f *clearance.CFace, dir r3.Vec) bool {
	phi := math.Atan2(dir.Dot(f.RefV), dir.Dot(f.RefU))
	return f.Sweep.Classify(phi, k.tol/math.Max(f.Radius, 1e-30)) == 1
}

// rulingInsidePlaneTrim admits the whole segment into the plane face's trim:
// one end strictly inside with margin and the segment clear of every trim
// boundary element by more than the margin, so it never leaves the region.
func (k *pairKernel) rulingInsidePlaneTrim(plane *clearance.CFace, ends [2]proofarith.DyV3) bool {
	var coords [2][2]float64
	for i, end := range ends {
		// The nearest float of each end is within an ulp; the trim margin
		// below is many orders wider.
		p := clearance.DyAxisVec(end)
		if !proofbound.FiniteVec(p) {
			return false
		}
		coords[i][0], coords[i][1] = plane.PlaneCoords(p)
	}
	if plane.Region.Classify(coords[0][0], coords[0][1], k.tol) != 1 {
		return false
	}
	for _, e := range plane.Region.Elems {
		if clearance.SegElemDistLB(e, coords[0][0], coords[0][1], coords[1][0], coords[1][1]) <= k.tol {
			return false
		}
	}
	return true
}
