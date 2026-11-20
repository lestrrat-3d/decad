package rimextension

import (
	"context"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// OuterAdmission binds the original circular nose chamfer to its current
// cylinder and cone and to its native pad. The prior producer certifies
// the exact world center, finite axial levels, radius and setback.
// Section uses the signed shoulder chart; all face identities are native.
type OuterAdmission struct {
	Admission
	Wing                                                            int
	Cone                                                            *topo.Face
	CenterX, CenterZ, Hinge, PadBottom, PadTop, Radius, NoseSetback *big.Rat
}

func outerCoordinate(p prismrecord.Payload, f BoundFace, pt momentinput.Point2, wing int) (*big.Rat, *big.Rat) {
	u := plus(plus(scalar(p.Frame.Origin().Z), scalar(f.Translation.Z)), scalar(pt.V))
	v := plus(plus(scalar(p.Frame.Origin().X), scalar(f.Translation.X)), scalar(pt.U))
	return signed(u, -wing), signed(v, -1)
}

func outerNative(ctx context.Context, a OuterAdmission, meter *proofbound.ReplayBudget) (windowNative, *big.Rat, bool, error) {
	var n windowNative
	for _, q := range []*big.Rat{a.CenterX, a.CenterZ, a.Hinge, a.PadBottom, a.PadTop, a.Radius, a.NoseSetback} {
		if q == nil || q.Num().BitLen() > 256 || q.Denom().BitLen() > 256 {
			return n, nil, false, nil
		}
	}
	if a.Wing != 1 && a.Wing != -1 || a.Boss.Final == nil || a.Boss.Final.Reversed || a.Cone == nil ||
		a.Cap.Body != a.Side.Body || a.Side.Body != a.Round.Body || a.Cap.Translation != a.Side.Translation || a.Side.Translation != a.Round.Translation {
		return n, nil, false, nil
	}
	for _, f := range []BoundFace{a.Cap, a.Side, a.Round, a.Boss} {
		if f.Final == nil || f.Final.Body != a.Stock || f.Native == nil || f.Native.Body != f.Body || !windowFinite(f.Translation) {
			return n, nil, false, nil
		}
	}
	if a.Cone.Body != a.Stock || !windowRole(a.Boss, "side(0,0)") || !windowRole(a.Cap, provenance.RoleCapStart) {
		return n, nil, false, nil
	}
	p, ok, err := windowPrism(ctx, a.Cap, meter)
	if !ok || err != nil {
		return n, nil, false, err
	}
	disk, ok, err := windowPrism(ctx, a.Boss, meter)
	if !ok || err != nil {
		return n, nil, false, err
	}
	if len(disk.Profile.Outer.Segments) != 1 {
		return n, nil, false, nil
	}
	circle, ok := disk.Profile.Outer.Segments[0].(momentinput.CircleSeg)
	if !ok || circle.Radius.Unit() != units.Millimeter {
		return n, nil, false, nil
	}
	r, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return n, nil, false, err
	}
	n.cx, n.cz = outerCoordinate(disk, a.Boss, circle.Center, a.Wing)
	n.r = scalar(r)
	if n.r.Sign() <= 0 || n.r.Cmp(a.Radius) != 0 || n.cx.Cmp(signed(a.CenterZ, -a.Wing)) != 0 || n.cz.Cmp(signed(a.CenterX, -1)) != 0 {
		return n, nil, false, nil
	}
	d0, d1 := disk.DenotedZ0(), disk.DenotedZ1()
	if d0 == nil || d1 == nil {
		return n, nil, false, nil
	}
	base := plus(scalar(disk.Frame.Origin().Y), scalar(a.Boss.Translation.Y))
	d0, d1 = minus(base, d0), minus(base, d1)
	if wmin(d0, d1).Cmp(a.PadBottom) != 0 || wmax(d0, d1).Cmp(a.PadTop) != 0 || a.NoseSetback.Sign() <= 0 || minus(a.PadTop, a.NoseSetback).Cmp(a.Hinge) != 0 {
		return n, nil, false, nil
	}
	p0, p1 := p.DenotedZ0(), p.DenotedZ1()
	if p0 == nil || p1 == nil {
		return n, nil, false, nil
	}
	base = plus(scalar(p.Frame.Origin().Y), scalar(a.Cap.Translation.Y))
	n.capY, n.otherY = minus(base, p0), minus(base, p1)
	n.out, n.materialY = 1, -1
	if n.otherY.Cmp(n.capY) >= 0 || n.capY.Cmp(a.Hinge) > 0 {
		return n, nil, false, nil
	}
	capPlane, ok := a.Cap.Final.Surface.(surfacegeom.Plane)
	if !ok {
		return n, nil, false, nil
	}
	normal := capPlane.Frame.N()
	if a.Cap.Final.Reversed {
		normal = normal.Scale(-1)
	}
	if normal.Y != 1 || normal.X != 0 || normal.Z != 0 {
		return n, nil, false, nil
	}
	side, ok := a.Side.Final.Surface.(surfacegeom.Plane)
	if !ok {
		return n, nil, false, nil
	}
	normal = side.Frame.N()
	if a.Side.Final.Reversed {
		normal = normal.Scale(-1)
	}
	if normal.Y != 0 || normal.X >= 0 {
		return n, nil, false, nil
	}
	si, ri := -1, -1
	for i := range p.Profile.Outer.Segments {
		role := "side(0," + strconv.Itoa(i) + ")"
		if windowRole(a.Round, role) {
			ri = i
		}
		if windowRole(a.Side, role) {
			si = i
		}
	}
	if si < 0 || ri < 0 {
		return n, nil, false, nil
	}
	line, ok := p.Profile.Outer.Segments[si].(momentinput.LineSeg)
	if !ok || line.TStart != 0 || line.TEnd != 1 {
		return n, nil, false, nil
	}
	arc, ok := p.Profile.Outer.Segments[ri].(momentinput.ArcSeg)
	if !ok {
		return n, nil, false, nil
	}
	n.x0, n.z0 = outerCoordinate(p, a.Cap, line.Start, a.Wing)
	x, z := outerCoordinate(p, a.Cap, line.End, a.Wing)
	dx := minus(x, n.x0)
	if dx.Sign() == 0 {
		return n, nil, false, nil
	}
	n.m = divide(minus(z, n.z0), dx)
	n.first, n.last = wmin(x, n.x0), wmax(x, n.x0)
	pt := line.End
	if dx.Sign() < 0 {
		pt = line.Start
	}
	if n.m.Sign() <= 0 || !ShoulderRoundBeyondEnd(arc, pt, a.Wing) {
		return n, nil, false, nil
	}
	// Along the entire native left arc, X is at most the larger endpoint
	// X and |Z| is at most the larger endpoint |Z| (relative to its center).
	// The largest dot product with the shifted center gives a lower bound
	// on squared radial clearance. Center offset plus the radius difference
	// bounds the width of the entire exposed lip by radial projection.
	ox, oz := outerCoordinate(p, a.Cap, arc.Center, a.Wing)
	u, v := outerCoordinate(p, a.Cap, arc.Start, a.Wing)
	uk, vk := outerCoordinate(p, a.Cap, arc.End, a.Wing)
	u, v, uk, vk = minus(u, ox), minus(v, oz), minus(uk, ox), minus(vk, oz)
	cr2 := plus(times(u, u), times(v, v))
	du, dv := minus(n.cx, ox), minus(n.cz, oz)
	if dv.Sign() >= 0 {
		return n, nil, false, nil
	}
	maxDot := plus(wmax(times(u, du), times(uk, du)), times(wmin(v, vk), dv))
	dist2 := plus(times(du, du), times(dv, dv))
	if minus(plus(cr2, dist2), times(scalar(2), maxDot)).Cmp(times(n.r, n.r)) <= 0 {
		return n, nil, false, nil
	}
	hi, lo := scalar(proofbound.RatSqrtUp(cr2)), scalar(proofbound.RatSqrtDown(cr2))
	lip := plus(scalar(proofbound.RatSqrtUp(dist2)), wmax(wabs(minus(hi, n.r)), wabs(minus(lo, n.r))))
	return n, lip, true, nil
}
