package rimextension

import (
	"context"
	"math"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// outerCapSupport bounds the original flange outside the selected side. The
// front arc has already been proved strictly beyond the open terminal plane.
func outerCapSupport(ctx context.Context, a OuterAdmission, n windowNative, start *big.Rat, meter *proofbound.ReplayBudget) (bool, error) {
	p, ok := a.Cap.Body.Payload.(prismrecord.Payload)
	if !ok {
		return false, nil
	}
	for i, seg := range p.Profile.Outer.Segments {
		if e := meter.ReserveBundle(ctx, 64, 0, 65536); e != nil {
			return false, e
		}
		role := "side(0," + strconv.Itoa(i) + ")"
		if windowRole(a.Round, role) || windowRole(a.Side, role) {
			continue
		}
		switch s := seg.(type) {
		case momentinput.LineSeg:
			if !wholeShoulderWalk(s.TStart, s.TEnd) {
				return false, nil
			}
			ax, az := outerCoordinate(p, a.Cap, s.Start, a.Wing)
			bx, bz := outerCoordinate(p, a.Cap, s.End, a.Wing)
			if wmin(ax, bx).Cmp(n.last) > 0 || wmax(ax, bx).Cmp(start) < 0 {
				continue
			}
			if az.Cmp(n.side(ax)) > 0 || bz.Cmp(n.side(bx)) > 0 {
				return false, nil
			}
		case momentinput.ArcSeg:
			maxU := windowSegmentUpper(s)
			if maxU == nil {
				return false, nil
			}
			r := minus(maxU, scalar(s.Center.U))
			x, z := outerCoordinate(p, a.Cap, s.Center, a.Wing)
			if minus(x, r).Cmp(n.last) > 0 || plus(x, r).Cmp(start) < 0 {
				continue
			}
			if plus(z, r).Cmp(n.side(start)) > 0 {
				return false, nil
			}
		default:
			return false, nil
		}
	}
	return true, nil
}

// AdmitOuterWindow certifies an exposed straight flange reaching its native
// rounded end beside an offset nose pad. The old cone can occur above the
// flange; throughout the new cut depth the guard is the finite cylinder.
func AdmitOuterWindow(ctx context.Context, a OuterAdmission, meter *proofbound.ReplayBudget) (WindowBounds, bool, error) {
	var none WindowBounds
	if meter == nil || a.Stock == nil || len(a.Stop) == 0 || len(a.Stop) > 16 || !windowFinite(a.Section.End) ||
		!windowFinite(a.Section.Stop) || !windowFinite(a.Section.Heel) || !windowFinite(a.Section.Toe) ||
		proofbound.IsNonFinite(a.Setback) || a.Setback <= 0 {
		return none, false, nil
	}
	if e := meter.ReserveBundle(ctx, 4096, 128, 3<<20); e != nil {
		return none, false, e
	}
	n, lip, ok, e := outerNative(ctx, a, meter)
	if !ok || e != nil {
		return none, false, e
	}
	h, ok := newChart(a.Section)
	if !ok || h.forward != 1 || h.rise != -1 || a.Section.Toe.Z >= a.Section.Stop.Z {
		return none, false, nil
	}
	s, ok, e := loadStock(ctx, a.Stock, meter)
	if !ok || e != nil {
		return none, false, e
	}
	accepted := map[*topo.Face]bool{a.Cap.Final: true, a.Side.Final: true, a.Boss.Final: true, a.Round.Final: true, a.Cone: true}
	for _, f := range a.RetainedDiskWalls {
		if f == nil || f.Body != a.Stock || a.Cone != a.Boss.Final {
			return none, false, nil
		}
		accepted[f] = true
	}
	delete(accepted, nil)
	base := a.Stop[0]
	if base.Body == nil || !base.Body.Solid || !windowFinite(base.Translation) {
		return none, false, nil
	}
	for _, f := range a.Stop {
		if f.Body != base.Body || f.Native == nil || f.Native.Body != f.Body || f.Translation != base.Translation || f.Final == nil || f.Final.Body != a.Stock {
			return none, false, nil
		}
		accepted[f.Final] = true
	}
	ids := map[int]bool{}
	g := Groups{Cap: -1, Boss: -1}
	for i, f := range s.fs {
		if accepted[f] {
			ids[i] = true
		}
		if f == a.Cap.Final {
			g.Cap = i
		}
		if f == a.Cone {
			g.Boss = i
		}
	}
	if len(ids) != len(accepted) || g.Cap < 0 || g.Boss < 0 {
		return none, false, nil
	}
	p, ok := base.Body.Payload.(loftrecord.Payload)
	if !ok || p.ConstructionDelta != 0 {
		return none, false, nil
	}
	loft, ok, e := shoulderLoft(ctx, &p, a.Wing, meter)
	if !ok || e != nil {
		return none, false, e
	}
	shift := ShoulderPoint(base.Translation, a.Wing)
	start, ok, e := LoftWindowXMax(ctx, &loft, shift.X, meter)
	if !ok || e != nil {
		return none, false, e
	}
	if start.Cmp(n.first) <= 0 || start.Cmp(n.last) >= 0 {
		return none, false, nil
	}
	ok, e = outerCapSupport(ctx, a, n, start, meter)
	if !ok || e != nil {
		return none, false, e
	}
	d := scalar(a.Setback)
	mag := plus(scalar(1), times(n.m, n.m))
	scale, scaleLo := scalar(proofbound.RatSqrtUp(mag)), scalar(proofbound.RatSqrtDown(mag))
	if scale == nil || scaleLo == nil || scaleLo.Sign() <= 0 {
		return none, false, nil
	}
	span := times(scale, d)
	left, right := wmin(start, h.start[0]), wmax(n.last, h.end[0])
	errSide := func(x *big.Rat) *big.Rat {
		return wabs(minus(n.side(x), plus(h.start[2], times(h.slope, minus(x, h.start[0])))))
	}
	ds := wmax(errSide(left), errSide(right))
	deep := plus(times(span, scalar(1.1)), ds)
	out := plus(divide(span, scalar(2)), ds)
	aligned := a.Boss.Final == a.Cone
	domain, ok, e := shoulderDiskRangeAt(ctx, n, start, plus(deep, ds), aligned, meter)
	if !ok || e != nil {
		return none, false, e
	}
	yLo, yHi := minus(n.capY, times(d, scalar(1.1))), plus(n.capY, divide(d, scalar(2)))
	if yLo.Cmp(a.PadBottom) <= 0 || yLo.Cmp(n.otherY) <= 0 || yHi.Cmp(a.PadTop) >= 0 || yHi.Cmp(a.Hinge) <= 0 {
		return none, false, nil
	}
	w := SourceWindow{Side0: a.Section.Stop, Side1: a.Section.End, XMin: proofbound.RatFloatDown(left), XMax: proofbound.RatFloatUp(right),
		YMin: proofbound.RatFloatDown(yLo), YMax: proofbound.RatFloatUp(yHi), OffsetMin: proofbound.RatFloatDown(new(big.Rat).Neg(deep)), OffsetMax: proofbound.RatFloatUp(out)}
	within := func(v, lo, hi *big.Rat) bool { return v.Cmp(lo) > 0 && v.Cmp(hi) < 0 }
	if !within(h.start[1], yLo, yHi) || !within(h.heel, yLo, yHi) || minus(h.start[2], scalar(a.Section.Toe.Z)).Cmp(deep) >= 0 {
		return none, false, nil
	}
	if e := meter.ReserveBundle(ctx, uint64(len(s.p.Verts))*4, uint64(len(s.p.Verts)), uint64(len(s.p.Verts))*24); e != nil {
		return none, false, e
	}
	vs := make([]r3.Vec, len(s.p.Verts))
	for i, v := range s.p.Verts {
		if !windowFinite(v) || proofbound.IsNonFinite(s.p.VertexBound[i]) || s.p.VertexBound[i] < 0 {
			return none, false, nil
		}
		vs[i] = ShoulderPoint(v, a.Wing)
	}
	ok, e = WindowSourcesCovered(ctx, vs, s.p.Tris, s.p.FaceOf, s.p.VertexBound, ids, w, meter)
	if !ok || e != nil {
		return none, false, e
	}
	support := w
	w.XMin = math.Nextafter(proofbound.RatFloatUp(start), math.Inf(1))
	w.XMax = proofbound.RatFloatDown(n.last)
	w.sideError = ds
	// The exterior witness belongs to the conical patch above its hinge.
	// Descending the cone grows the disk until the hinge, then holds its
	// radius throughout the cut. Its interior is convex. Its exterior
	// connects to the plane-only prefix along the outward side of the line.
	ok, e = windowWitnessesAt(ctx, Mesh{vs, s.p.Tris, s.p.FaceOf}, s.p.VertexBound, g, w, n.cx, n.cz, n.r, n.capY, a.Hinge, 1, -1, meter)
	if !ok || e != nil {
		return none, false, e
	}
	side := n.side(start)
	seam, ok, e := loftWindowSeam(ctx, &loft, shift, n.capY, side, minus(side, deep), plus(side, out), 1, meter)
	if !ok || e != nil {
		return none, false, e
	}
	if seam.toeZ.Cmp(side) >= 0 || minus(side, seam.toeZ).Cmp(deep) >= 0 || seam.heelY.Cmp(n.capY) >= 0 || !within(seam.heelY, yLo, yHi) {
		return none, false, nil
	}
	cross := n.last
	if aligned {
		cross = domain.crossing
	}
	b := windowMeasures(a.Admission, n, h, start, cross, seam, ds, span, scaleLo, times(domain.minOrdinate, domain.minOrdinate))
	// The terminating face is a finite section at the side/round seam. The
	// old and new guard projections enclose its two ends. Charge its whole
	// YZ rectangle, even if an output subdivision collapses on the rim.
	b.Area = proofbound.AbsSumUpper(b.Area, proofbound.RatFloatUp(times(d, span)))
	// Include both the radial width of the entire exposed crescent and its
	// fixed-u width at this operation's terminal plane.
	b.Deviation = proofbound.AbsSumUpper(b.Deviation, proofbound.RatFloatUp(lip), proofbound.RatFloatUp(domain.terminalGapUpper))
	b.support = support
	// Every point on the complete original rounded cheek is outside the
	// moved pad. Bound its radial lip in addition to the fixed-X terminal
	// width before accepting an endpoint comparison.
	if (!aligned && lip.Sign() <= 0) || domain.terminalGapUpper.Cmp(span) >= 0 || proofbound.IsNonFinite(b.Deviation) || proofbound.IsNonFinite(b.Area) || proofbound.IsNonFinite(b.Volume) {
		return none, false, nil
	}
	return b, true, nil
}
