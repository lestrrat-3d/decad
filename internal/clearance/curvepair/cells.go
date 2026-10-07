// Package curvepair evaluates face-edge and edge-edge clearance cells.
// Candidates come from unbounded Line3, Circle3 and Arc3 carriers and are
// admitted by edge parameters and face trims. Line3 × Cone and Circle3 × Cone
// use coarse enclosures. Constant-distance families require the degeneracy
// oracle's DegYes; uncertain degeneracy contributes a bound or marks the cell
// unsure.
package curvepair

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Kernel holds the per-call state shared by one curve cell.
type Kernel struct {
	ctx        context.Context //nolint:containedctx // A curve-cell kernel is per-call state.
	tol, slack float64
	err        error
	refused    bool
}

func New(ctx context.Context, tol, slack float64) *Kernel {
	return &Kernel{ctx: ctx, tol: tol, slack: slack}
}
func (k *Kernel) Err() error               { return k.err }
func (k *Kernel) Refused() bool            { return k.refused }
func (k *Kernel) oracle() clearance.Oracle { return clearance.Oracle{Tol: k.tol} }

func (k *Kernel) spineEngine() *spine.Engine {
	return &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
}

func (k *Kernel) captureSpine(e *spine.Engine) {
	if e.Err != nil {
		k.err = e.Err
	}
	k.refused = k.refused || e.Refused
}

func (k *Kernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.PointCircleCrits(p, c, axis, refU, refV, rad, win)
	k.captureSpine(e)
	return out, ok
}

func (k *Kernel) lineCircleBracketCrits(cp freeform.CircleParam, center, refU, refV, la, ld r3.Vec) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.LineCircleBracketCrits(cp, center, refU, refV, la, ld)
	k.captureSpine(e)
	return out, ok
}

func (k *Kernel) circleCircleCrits(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.CircleCircleCrits(f, g)
	k.captureSpine(e)
	return out, ok
}

// FaceEdge dispatches one face × edge pair through §4's curve-tier table.
func (k *Kernel) FaceEdge(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
	switch f.Kind {
	case clearance.CkPlane:
		if e.Line {
			k.linePlaneFE(f, e, sink)
			return
		}
		k.circlePlaneFE(f, e, sink)
	case clearance.CkCone:
		// Line3 × Cone and Circle3 × Cone take the coarse enclosure here.
		sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
	default:
		if f.Kind == clearance.CkTorus && f.Spindle {
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
func (k *Kernel) linePlaneFE(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
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
			sink.Unsure = true // the edge meets the face's own plane inside the trim (or ambiguously)
			return
		}
		pa := f.O.Add(f.U.Scale(w[0])).Add(f.V.Scale(w[1]))
		sink.Candidate(k.tol, hit, math.Abs(h), math.Abs(h), true, pa, pa.Add(f.N.Scale(h)))
		return
	case clearance.DegUnknown:
		if clearance.ClrBoxDist(f.Box, e.Box) > k.tol {
			return
		}
		sink.Unsure = true
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
	sink.Unsure = true
}

// circlePlaneFE: the circle's signed plane height is a first harmonic —
// closed-form criticals, closed-form crossing roots. The constant-height
// reading belongs to the circle whose own plane is EXACTLY parallel to the
// face's (its axis parallel to the normal); decided on an amplitude epsilon it
// would report abs(base) Exact where the true minimum is abs(base) − the
// amplitude.
func (k *Kernel) circlePlaneFE(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
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
		sink.Unsure = true
		return
	}
	if parallel == clearance.DegYes {
		// The circle parallels the plane: constant height.
		if math.Abs(base) <= k.tol {
			// In the carrier plane itself: a contact exactly when the circle
			// meets the trim (the full circle is a sound superset of an arc).
			cx, cy := f.PlaneCoords(e.Center)
			if clearance.CircleRegionHits(f.Region, cx, cy, e.Radius) != -1 {
				sink.Unsure = true
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
			sink.Candidate(k.tol, clearance.AdmitState(admit, f.Region.Classify(x, y, k.tol)), math.Abs(base), math.Abs(base), true, foot, pe)
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
		sink.Candidate(k.tol, admit, math.Abs(hh), math.Abs(hh), true, foot, pe)
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.Ang.Full {
		lo, hi = e.Ang.Lo, e.Ang.Hi
	}
	mn, mx := clearance.TrigRange(hu, hv, lo, hi)
	if base+mn > k.tol || base+mx < -k.tol {
		return // the arc never meets the carrier plane
	}
	// Crossing roots h(θ) = 0: excluded only when every root leaves a trim.
	amp := math.Hypot(hu, hv)
	if amp < 1e-30 {
		sink.Unsure = true
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
		sink.Unsure = true
		return
	}
}

// feOffsetEmit emits the ± offset combinations of one curve-to-spine
// critical against an offset face.
func (k *Kernel) feOffsetEmit(sink *clearance.CellSink, f *clearance.CFace, pe, spineFoot r3.Vec, dLo, dHi float64, exact bool, eAdmit int) {
	sep := pe.Sub(spineFoot)
	d := sep.Len()
	if d <= k.tol {
		sink.Unsure = true
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
				sink.Unsure = true
			}
			continue
		}
		lo, hi := math.Abs(rawLo), math.Abs(rawHi)
		if lo > hi {
			lo, hi = hi, lo
		}
		sink.Candidate(k.tol, admit, lo, hi, exact, pf, pe)
	}
}

// feCrossingExcluded proves the trimmed edge never meets the offset face's
// carrier, from a superset range of the edge's spine distance (criticals
// plus endpoints — conservative in the sound direction).
func (k *Kernel) feCrossingExcluded(f *clearance.CFace, e *clearance.CEdge, minLo, maxHi float64) bool {
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
func (k *Kernel) lineOffsetFE(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
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
				sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
				return
			}
			crits = []clearance.SpineCrit{cs}
		default:
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
		sink.Unsure = true
	}
}

func (k *Kernel) lineLinePerp(a, u, b, v r3.Vec) (clearance.SpineCrit, bool) {
	e := k.spineEngine()
	return e.LineLinePerp(a, u, b, v)
}

// circleOffsetFE: a circular edge against a cylinder, sphere or torus face —
// circle × point (CF), circle × axis (P4, with the perpendicular-plane and
// coaxial cases closed form), circle × spine circle (P8).
func (k *Kernel) circleOffsetFE(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
	var crits []clearance.SpineCrit
	switch clearance.SpineOf(f) {
	case 0:
		cs, ok := k.pointCircleCrits(f.Anchor, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
		if !ok {
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
					sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
					return
				}
				near := clearance.AngleOf(e, dir.Scale(-1))
				far := clearance.AngleOf(e, dir)
				for _, th := range []float64{near, far} {
					pe := e.At(th)
					crits = append(crits, clearance.ExactCrit(pe, clearance.LinePoint(f.Anchor, f.Axis, pe)))
				}
			default:
				sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
				sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
				return
			}
			for i := range cs {
				cs[i].Fa, cs[i].Fb = cs[i].Fb, cs[i].Fa // (edge point, axis foot)
			}
			crits = cs
		default:
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
			return
		}
	default:
		ef := &clearance.CFace{Kind: clearance.CkTorus, Anchor: e.Center, Axis: e.Axis, RefU: e.RefU, RefV: e.RefV, Major: e.Radius}
		cs, ok := k.circleCircleCrits(ef, f)
		if !ok {
			sink.Coarse(f.Box, e.Box, f.Wit, clearance.EdgeWits(e))
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
		sink.Unsure = true
	}
}

// EdgeEdge dispatches one edge pair through §4's curve tiers.
func (k *Kernel) EdgeEdge(ea, eb *clearance.CEdge, sink *clearance.CellSink) {
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
func (k *Kernel) lineLineEE(ea, eb *clearance.CEdge, sink *clearance.CellSink) {
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
		sink.Candidate(k.tol, 1, d, d, true, pa, pb)
		return
	case clearance.DegUnknown:
		// A tilt too small to prove or disprove: the constant family is not
		// certifiable and the common perpendicular is not resolvable.
		if clearance.ClrBoxDist(ea.Box, eb.Box) > k.tol {
			return
		}
		sink.Unsure = true
		return
	}
	c, ok := k.lineLinePerp(ea.A, ua, eb.A, ub)
	if !ok {
		sink.Coarse(ea.Box, eb.Box, clearance.EdgeWits(ea), clearance.EdgeWits(eb))
		return
	}
	d := c.Fa.Sub(c.Fb).Len()
	admit := clearance.AdmitState(clearance.LineParamAdmit(ea, c.Fa, k.tol), clearance.LineParamAdmit(eb, c.Fb, k.tol))
	sink.Candidate(k.tol, admit, d, d, true, c.Fa, c.Fb)
}

// lineCircleEE: the axis-parallel case is closed form; the general case is
// the P4 bracket.
func (k *Kernel) lineCircleEE(el, ec *clearance.CEdge, sink *clearance.CellSink) {
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
				sink.Coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
				return
			}
			ths = []float64{clearance.AngleOf(ec, dirP), clearance.AngleOf(ec, dirP.Scale(-1))}
		default:
			sink.Coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
			return
		}
		for _, th := range ths {
			pc := ec.At(th)
			pl := clearance.LinePoint(el.A, u, pc)
			d := pc.Sub(pl).Len()
			admit := clearance.AdmitState(clearance.CircleAngleAdmit(ec, th, k.tol), clearance.LineParamAdmit(el, pl, k.tol))
			sink.Candidate(k.tol, admit, d, d, true, pl, pc)
		}
		return
	case clearance.DegUnknown:
		sink.Coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
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
		sink.Coarse(el.Box, ec.Box, clearance.EdgeWits(el), clearance.EdgeWits(ec))
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(ec, c.Fb.Sub(ec.Center))
		admit := clearance.AdmitState(clearance.LineParamAdmit(el, c.Fa, k.tol), clearance.CircleAngleAdmit(ec, th, k.tol+(c.Hi-c.Lo)))
		sink.Candidate(k.tol, admit, c.Lo, c.Hi, c.Exact, c.Fa, c.Fb)
	}
}

// PrincipalCircleEdgeGap proves the minimum over two complete circles whose
// axes are exactly vertical and whose centers differ along one horizontal
// principal axis. Their facing radial points attain the minimum. The points
// returned for the candidate are rounded representatives; the exact rational
// separation and directed square-root interval certify the distance.
func (k *Kernel) PrincipalCircleEdgeGap(ea, eb *clearance.CEdge, sink *clearance.CellSink) bool {
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
	sink.Candidate(k.tol, 1, lo, hi, lo == hi, pa, pb)
	return true
}

// circleCircleEE: complete principal-axis exterior circles have a direct
// minimum; coaxial circles use their constant closed form; the rest use P8.
func (k *Kernel) circleCircleEE(ea, eb *clearance.CEdge, sink *clearance.CellSink) {
	if k.PrincipalCircleEdgeGap(ea, eb, sink) {
		return
	}
	fa := &clearance.CFace{Kind: clearance.CkTorus, Anchor: ea.Center, Axis: ea.Axis, RefU: ea.RefU, RefV: ea.RefV, Major: ea.Radius}
	fb := &clearance.CFace{Kind: clearance.CkTorus, Anchor: eb.Center, Axis: eb.Axis, RefU: eb.RefU, RefV: eb.RefV, Major: eb.Radius}
	crits, ok := k.circleCircleCrits(fa, fb)
	if !ok {
		sink.Coarse(ea.Box, eb.Box, clearance.EdgeWits(ea), clearance.EdgeWits(eb))
		return
	}
	for _, c := range crits {
		tha := clearance.AngleOf(ea, c.Fa.Sub(ea.Center))
		thb := clearance.AngleOf(eb, c.Fb.Sub(eb.Center))
		admit := clearance.AdmitState(clearance.CircleAngleAdmit(ea, tha, k.tol+(c.Hi-c.Lo)), clearance.CircleAngleAdmit(eb, thb, k.tol+(c.Hi-c.Lo)))
		sink.Candidate(k.tol, admit, c.Lo, c.Hi, c.Exact, c.Fa, c.Fb)
	}
}
