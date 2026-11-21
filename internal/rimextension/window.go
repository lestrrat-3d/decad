package rimextension

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func windowFloat(x *big.Rat) (float64, bool) {
	v, exact := x.Float64()
	return v, exact && !proofbound.IsNonFinite(v)
}
func signed(x *big.Rat, s int) *big.Rat {
	if s < 0 {
		return new(big.Rat).Neg(x)
	}
	return x
}

// AdmitWindow certifies the occupied parts between the original stop and the
// supporting disk, using the caller's authenticated face history.
func AdmitWindow(ctx context.Context, a Admission, meter *proofbound.ReplayBudget) (WindowBounds, bool, error) {
	var none WindowBounds
	if meter == nil || a.Stock == nil || len(a.Stop) == 0 || len(a.Stop) > 16 || len(a.Allowed) > len(a.Stop)+4 || proofbound.IsNonFinite(a.Setback) || a.Setback <= 0 {
		return none, false, nil
	}
	if err := meter.ReserveBundle(ctx, 2048, 64, 1<<20); err != nil {
		return none, false, err
	}
	for _, p := range []r3.Vec{a.Section.Stop, a.Section.End, a.Section.Toe, a.Section.Heel, a.Cap.Translation, a.Boss.Translation, a.Stop[0].Translation} {
		if !windowFinite(p) {
			return none, false, nil
		}
	}
	h, ok := newChart(a.Section)
	if !ok || h.forward != 1 {
		return none, false, nil
	}
	n, ok, e := windowNativeFaces(ctx, a, meter)
	if !ok || e != nil {
		return none, false, e
	}
	if n.materialY != h.rise || signed(minus(h.start[2], scalar(a.Section.Toe.Z)), n.out).Sign() <= 0 {
		return none, false, nil
	}
	s, ok, e := loadStock(ctx, a.Stock, meter)
	if !ok || e != nil {
		return none, false, e
	}
	if err := meter.SpendWork(ctx, uint64(len(s.p.Verts))); err != nil {
		return none, false, err
	}
	for j, v := range s.p.Verts {
		if !windowFinite(v) || proofbound.IsNonFinite(s.p.VertexBound[j]) || s.p.VertexBound[j] < 0 {
			return none, false, nil
		}
	}
	g := Groups{Cap: -1, Boss: -1}
	verifiedFaces := map[*topo.Face]bool{}
	faceIndices := map[int]bool{}
	valid := map[int]bool{}
	admit := func(f *topo.Face) bool {
		id, exists := s.groups[f]
		if !exists || id < 0 {
			return false
		}
		verifiedFaces[f] = true
		valid[id] = true
		return true
	}
	for _, f := range []*topo.Face{a.Cap.Final, a.Side.Final, a.Boss.Final, a.Round.Final} {
		if !admit(f) {
			return none, false, nil
		}
	}
	base := a.Stop[0]
	if base.Body == nil || !base.Body.Solid {
		return none, false, nil
	}
	for _, f := range a.Stop {
		if f.Body != base.Body || f.Native == nil || f.Native.Body != f.Body || f.Translation != base.Translation || !admit(f.Final) {
			return none, false, nil
		}
	}
	for group, yes := range a.Allowed {
		if yes && !valid[group] {
			return none, false, nil
		}
	}
	for i, f := range s.fs {
		if verifiedFaces[f] {
			faceIndices[i] = true
		}
		if f == a.Cap.Final {
			g.Cap = i
		}
		if f == a.Boss.Final {
			g.Boss = i
		}
	}
	if g.Cap < 0 || g.Boss < 0 {
		return none, false, nil
	}
	loft, ok := base.Body.Payload.(loftrecord.Payload)
	if !ok || loft.ConstructionDelta != 0 {
		return none, false, nil
	}
	start, ok, e := LoftWindowXMax(ctx, &loft, base.Translation.X, meter)
	if !ok || e != nil {
		return none, false, e
	}
	if start.Cmp(n.first) <= 0 || start.Cmp(n.last) >= 0 {
		return none, false, nil
	}
	ok, e = windowCapSupport(ctx, a, n, start, meter)
	if !ok || e != nil {
		return none, false, e
	}
	d := scalar(a.Setback)
	mag := plus(scalar(1), times(n.m, n.m))
	scale := scalar(proofbound.RatSqrtUp(mag))
	scaleLo := scalar(proofbound.RatSqrtDown(mag))
	if scale == nil || scaleLo == nil || scaleLo.Sign() <= 0 {
		return none, false, nil
	}
	span := times(d, scale)
	// The true side segment terminates in the disk. The outside points on
	// this initial segment form one interval because a disk is convex.
	// The selected disk branch is continuous throughout the interval.
	l, bottom, ok, e := windowCrossing(ctx, n, start, meter)
	if !ok || e != nil {
		return none, false, e
	}
	minOrd := scalar(proofbound.RatSqrtDown(bottom))
	held0, held1 := h.start[0], h.end[0]
	left := wmin(start, held0)
	right := wmax(n.last, held1)
	heldSide := func(x *big.Rat) *big.Rat { return plus(h.start[2], times(h.slope, minus(x, held0))) }
	ds := wmax(wabs(minus(heldSide(left), n.side(left))), wabs(minus(heldSide(right), n.side(right))))
	// The wider exterior margin supplies a sub-cap witness and makes the
	// transverse cells open even where the circle reaches the side line.
	yLow, yHigh := minus(n.capY, divide(d, scalar(2))), plus(n.capY, times(d, scalar(1.1)))
	if n.materialY < 0 {
		yLow, yHigh = minus(n.capY, times(d, scalar(1.1))), plus(n.capY, divide(d, scalar(2)))
	}
	if signed(minus(n.otherY, n.capY), n.materialY).Cmp(times(d, scalar(1.1))) <= 0 {
		return none, false, nil
	}
	disk, ok := a.Boss.Body.Payload.(prismrecord.Payload)
	if !ok {
		return none, false, nil
	}
	z0, z1 := disk.DenotedZ0(), disk.DenotedZ1()
	if z0 == nil || z1 == nil {
		return none, false, nil
	}
	y := plus(scalar(disk.Frame.Origin().Y), scalar(a.Boss.Translation.Y))
	y0, y1 := minus(y, z0), minus(y, z1)
	if wmin(y0, y1).Cmp(yLow) >= 0 || wmax(y0, y1).Cmp(yHigh) <= 0 {
		return none, false, nil
	}
	deep, outer := plus(times(span, scalar(1.1)), ds), plus(divide(span, scalar(2)), ds)
	// Outside the crossing, the disk graph can pass the side by at most
	// its relative slope times the remaining interval. Keep that generator
	// inside the certified band and keep the opposite circle branch out.
	outsideSlope := plus(divide(n.r, minOrd), wabs(n.m))
	if times(minus(n.last, l), outsideSlope).Cmp(minus(outer, ds)) >= 0 ||
		wmin(signed(minus(n.side(start), n.cz), n.out), signed(minus(n.side(n.last), n.cz), n.out)).Cmp(plus(deep, ds)) <= 0 {
		return none, false, nil
	}
	offLo, offHi := new(big.Rat).Neg(deep), outer
	if n.out < 0 {
		offLo, offHi = new(big.Rat).Neg(outer), deep
	}
	within := func(q, lo, hi *big.Rat) bool { return q.Cmp(lo) > 0 && q.Cmp(hi) < 0 }
	if !within(h.start[1], yLow, yHigh) || !within(h.heel, yLow, yHigh) ||
		!within(minus(scalar(a.Section.Toe.Z), h.start[2]), offLo, offHi) {
		return none, false, nil
	}
	w := SourceWindow{
		Side0: a.Section.Stop, Side1: a.Section.End,
		XMin: proofbound.RatFloatDown(left), XMax: proofbound.RatFloatUp(right),
		YMin: proofbound.RatFloatDown(yLow), YMax: proofbound.RatFloatUp(yHigh),
		OffsetMin: proofbound.RatFloatDown(offLo), OffsetMax: proofbound.RatFloatUp(offHi),
	}
	ok, e = WindowSourcesCovered(ctx, s.p.Verts, s.p.Tris, s.p.FaceOf, s.p.VertexBound, faceIndices, w, meter)
	// A neighboring cap island can reach the wider witness margin while
	// staying clear of the true crossing, the held toe and a narrower witness.
	if !ok && e == nil {
		margin := divide(span, scalar(16))
		if times(minus(n.last, l), outsideSlope).Cmp(margin) >= 0 {
			return none, false, nil
		}
		outer = plus(margin, ds)
		if n.out > 0 {
			offHi = outer
			w.OffsetMax = proofbound.RatFloatUp(offHi)
		} else {
			offLo = new(big.Rat).Neg(outer)
			w.OffsetMin = proofbound.RatFloatDown(offLo)
		}
		if !within(minus(scalar(a.Section.Toe.Z), h.start[2]), offLo, offHi) {
			return none, false, nil
		}
		ok, e = WindowSourcesCovered(ctx, s.p.Verts, s.p.Tris, s.p.FaceOf, s.p.VertexBound, faceIndices, w, meter)
	}
	if !ok || e != nil {
		return none, false, e
	}
	// Restrict witnesses to the connected part strictly outside the disk.
	support := w
	w.XMin = math.Nextafter(proofbound.RatFloatUp(start), math.Inf(1))
	w.XMax = proofbound.RatFloatDown(l)
	w.sideError = ds
	cx, ok1 := windowFloat(n.cx)
	cz, ok2 := windowFloat(n.cz)
	cy, ok3 := windowFloat(n.capY)
	radius, ok4 := windowFloat(n.r)
	if !ok1 || !ok2 || !ok3 || !ok4 {
		return none, false, nil
	}
	ok, e = WindowWitnesses(ctx, Mesh{s.p.Verts, s.p.Tris, s.p.FaceOf}, s.p.VertexBound, g, w, r3.Vec{X: cx, Z: cz}, radius, cy, n.out, n.materialY, meter)
	if !ok || e != nil {
		return none, false, e
	}
	ss := n.side(start)
	seam, ok, e := loftWindowSeam(ctx, &loft, base.Translation, n.capY, ss, plus(ss, offLo), plus(ss, offHi), n.out, meter)
	if !ok || e != nil {
		return none, false, e
	}
	seamOffset := minus(seam.toeZ, ss)
	if signed(seamOffset, n.out).Sign() >= 0 || signed(minus(seam.heelY, n.capY), n.materialY).Sign() <= 0 ||
		!within(seamOffset, offLo, offHi) || !within(seam.heelY, yLow, yHigh) {
		return none, false, nil
	}
	bounds := windowMeasures(a, n, h, start, l, seam, ds, span, scaleLo, bottom)
	bounds.support = support
	bounds.coveredFaces = faceIndices
	if proofbound.IsNonFinite(bounds.Deviation) || proofbound.IsNonFinite(bounds.Area) || proofbound.IsNonFinite(bounds.Volume) {
		return none, false, nil
	}
	return bounds, true, nil
}

// At matched fractional X and axial height the straight toe and heel
// interpolate on each bevel. Clip the transverse interval to the disk;
// taking the min or max of two bounds is nonexpansive. The plane's
// inverse slope maps any discrepancy of that clipped endpoint into Y.
func windowMeasures(a Admission, n windowNative, h chart, start, cross *big.Rat, seam nativeSeam, sideError, span, scaleLo, minRadial2 *big.Rat) WindowBounds {
	d := scalar(a.Setback)
	sqrtHi := func(x *big.Rat) *big.Rat { return scalar(proofbound.RatSqrtUp(x)) }
	scale := divide(span, d)
	invZ := divide(n.r, scalar(proofbound.RatSqrtDown(minRadial2)))
	terminal := plus(wabs(minus(h.end[0], cross)), wabs(minus(n.last, cross)))
	dx := wmax(wabs(minus(h.start[0], start)), terminal)
	dy := wmax(wabs(minus(h.start[1], n.capY)), wabs(minus(h.heel, plus(n.capY, signed(d, n.materialY)))))
	dy = wmax(dy, wabs(minus(h.heel, seam.heelY)))
	toe := scalar(a.Section.Toe.Z)
	heldOffset := minus(toe, h.start[2])
	idealHi, idealLo := signed(span, -n.out), signed(times(scaleLo, d), -n.out)
	dz := wmax(wabs(minus(heldOffset, idealHi)), wabs(minus(heldOffset, idealLo)))
	dz = plus(sideError, wmax(dz, wabs(minus(toe, seam.toeZ))))
	dz = plus(dz, times(dx, wmax(invZ, wabs(n.m))))
	gradient := wmax(wabs(h.gradient), divide(scalar(1), scaleLo))
	dev := plus(plus(dx, dy), times(dz, plus(scalar(1), gradient)))
	width := minus(n.last, start)
	// The old and new horizontal, side, cylindrical and bevel patches are
	// subdomains of their untrimmed carriers. The old stop face has a
	// separate finite YZ rectangle at the left endpoint.
	projected := times(width, span)
	horizontal := projected
	side := times(times(width, scale), d)
	disk := times(times(width, invZ), d)
	bevel := times(projected, sqrtHi(scalar(2)))
	stop := times(span, d)
	area := plus(stop, times(scalar(2), plus(plus(horizontal, side), plus(disk, bevel))))
	return WindowBounds{Deviation: proofbound.RatFloatUp(dev), Area: proofbound.RatFloatUp(area), Volume: proofbound.RatFloatUp(times(projected, d))}
}

func windowFinite(v r3.Vec) bool {
	return !proofbound.IsNonFinite(v.X) && !proofbound.IsNonFinite(v.Y) && !proofbound.IsNonFinite(v.Z)
}
