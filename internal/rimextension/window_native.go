package rimextension

import (
	"context"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type windowNative struct {
	first, last, x0, z0, m, cx, cz, r, capY, otherY *big.Rat
	out, materialY                                  int
}

func windowPrism(ctx context.Context, f BoundFace, meter *proofbound.ReplayBudget) (prismrecord.Payload, bool, error) {
	var none prismrecord.Payload
	if f.Body == nil || f.Native == nil || f.Final == nil || f.Native.Body != f.Body || !f.Body.Solid {
		return none, false, nil
	}
	p, ok := f.Body.Payload.(prismrecord.Payload)
	if !ok || p.SurfaceResult || p.SectionDelta != 0 || p.Xform != r3.Identity() || p.Frame.U() != (r3.Vec{X: 1}) || p.Frame.V() != (r3.Vec{Z: 1}) ||
		len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) == 0 || len(p.Profile.Outer.Segments) > 64 {
		return none, false, nil
	}
	if e := meter.ReserveBundle(ctx, 128, 1, 16384); e != nil {
		return none, false, e
	}
	ok, e := f.Body.ReplayPrismBuildPayload(ctx, p.Frame, p.Xform, p.Z0, p.Z1, p.Z0Delta, p.Z1Delta, p.SectionDelta, p.Z0Denoted, p.Z1Denoted, []sectionrecord.AnalyticProfileView{p.Profile.AnalyticSealView()}, meter)
	return p, ok, e
}

func windowRole(f BoundFace, role string) bool {
	return f.Native != nil && f.Body != nil && len(f.Native.Origins) == 1 && f.Native.Origins[0] == (provenance.Ref{Producer: f.Body.Origin.Producer, Role: role})
}
func wabs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }
func wmax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}
func wmin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

func windowNativeFaces(ctx context.Context, a Admission, meter *proofbound.ReplayBudget) (windowNative, bool, error) {
	var n windowNative
	if a.Side.Body != a.Cap.Body || a.Round.Body != a.Cap.Body || a.Side.Translation != a.Cap.Translation || a.Round.Translation != a.Cap.Translation ||
		a.Side.Native == nil || a.Round.Native == nil || a.Side.Final == nil || a.Round.Final == nil || a.Round.Final.Reversed || a.Boss.Final == nil || a.Boss.Final.Reversed {
		return n, false, nil
	}
	p, ok, e := windowPrism(ctx, a.Cap, meter)
	if !ok || e != nil {
		return n, false, e
	}
	disk, ok, e := windowPrism(ctx, a.Boss, meter)
	if !ok || e != nil {
		return n, false, e
	}
	if !windowRole(a.Boss, "side(0,0)") || len(disk.Profile.Outer.Segments) != 1 {
		return n, false, nil
	}
	c, ok := disk.Profile.Outer.Segments[0].(momentinput.CircleSeg)
	if !ok || c.Radius.Unit() != units.Millimeter {
		return n, false, nil
	}
	radius, err := c.Radius.In(units.Millimeter)
	if err != nil {
		return n, false, err
	}
	n.r = scalar(radius)
	if n.r == nil || n.r.Sign() <= 0 {
		return n, false, nil
	}
	world := func(p prismrecord.Payload, move r3.Vec, pt momentinput.Point2) (*big.Rat, *big.Rat) {
		return plus(plus(scalar(p.Frame.Origin().X), scalar(move.X)), scalar(pt.U)), plus(plus(scalar(p.Frame.Origin().Z), scalar(move.Z)), scalar(pt.V))
	}
	n.cx, n.cz = world(disk, a.Boss.Translation, c.Center)
	for _, f := range []BoundFace{a.Cap, a.Side, a.Boss, a.Round} {
		if f.Native.Body != f.Body || f.Final.Body != a.Stock {
			return n, false, nil
		}
	}
	side, round := -1, -1
	for i := range p.Profile.Outer.Segments {
		role := "side(0," + strconv.Itoa(i) + ")"
		if windowRole(a.Side, role) {
			side = i
		}
		if windowRole(a.Round, role) {
			round = i
		}
	}
	if side < 0 || round < 0 {
		return n, false, nil
	}
	line, ok := p.Profile.Outer.Segments[side].(momentinput.LineSeg)
	if !ok || line.TStart != 0 || line.TEnd != 1 {
		return n, false, nil
	}
	arc, ok := p.Profile.Outer.Segments[round].(momentinput.ArcSeg)
	if !ok || arc.TStart != 0 || arc.TEnd != 1 {
		return n, false, nil
	}
	x, z := world(p, a.Cap.Translation, arc.Center)
	if x.Cmp(n.cx) != 0 || z.Cmp(n.cz) != 0 {
		return n, false, nil
	}
	r2 := times(n.r, n.r)
	for _, pt := range [2]momentinput.Point2{arc.Start, arc.End} {
		x, z = world(p, a.Cap.Translation, pt)
		dx, dz := minus(x, n.cx), minus(z, n.cz)
		if plus(times(dx, dx), times(dz, dz)).Cmp(r2) > 0 {
			return n, false, nil
		}
	}
	n.x0, n.z0 = world(p, a.Cap.Translation, line.Start)
	x, z = world(p, a.Cap.Translation, line.End)
	dx := minus(x, n.x0)
	if dx.Sign() == 0 {
		return n, false, nil
	}
	n.m = divide(minus(z, n.z0), dx)
	n.first, n.last = wmin(x, n.x0), wmax(x, n.x0)
	n.out = -dx.Sign() // outer loop is counterclockwise in X,Z
	last := line.End
	if dx.Sign() < 0 {
		last = line.Start
	}
	arcFollows := dx.Sign() > 0 && last == arc.Start && (side+1)%len(p.Profile.Outer.Segments) == round
	arcPrecedes := dx.Sign() < 0 && last == arc.End && (round+1)%len(p.Profile.Outer.Segments) == side
	if !arcFollows && !arcPrecedes {
		return n, false, nil
	}
	z0, z1 := p.DenotedZ0(), p.DenotedZ1()
	if z0 == nil || z1 == nil || z0.Cmp(z1) == 0 {
		return n, false, nil
	}
	if windowRole(a.Cap, provenance.RoleCapStart) {
		z0, z1 = z1, z0
	} else if !windowRole(a.Cap, provenance.RoleCapEnd) {
		return n, false, nil
	}
	y := plus(scalar(p.Frame.Origin().Y), scalar(a.Cap.Translation.Y))
	n.capY, n.otherY = minus(y, z1), minus(y, z0)
	n.materialY = n.otherY.Cmp(n.capY)
	pl, ok := a.Cap.Final.Surface.(surfacegeom.Plane)
	if !ok {
		return n, false, nil
	}
	normal := pl.Frame.N()
	if a.Cap.Final.Reversed {
		normal.Y = -normal.Y
	}
	if normal.X != 0 || normal.Z != 0 || normal.Y != float64(-n.materialY) {
		return n, false, nil
	}
	sidePlane, ok := a.Side.Final.Surface.(surfacegeom.Plane)
	if !ok {
		return n, false, nil
	}
	normal = sidePlane.Frame.N()
	if a.Side.Final.Reversed {
		normal.Z = -normal.Z
	}
	if normal.Y != 0 || normal.Z*float64(n.out) <= 0 {
		return n, false, nil
	}
	return n, true, nil
}
func (n windowNative) side(x *big.Rat) *big.Rat { return plus(n.z0, times(n.m, minus(x, n.x0))) }

// Other portions of the cap boundary must stay below the selected line,
// miss the X interval, or belong to the rounded end enclosed by the boss.
// Every interior point of a bounded plane profile then satisfies the same
// outer support outside the disk, by following its outward vertical ray.
func windowCapSupport(ctx context.Context, a Admission, n windowNative, xmin *big.Rat, meter *proofbound.ReplayBudget) (bool, error) {
	p, ok := a.Cap.Body.Payload.(prismrecord.Payload)
	if !ok {
		return false, nil
	}
	xo := plus(scalar(p.Frame.Origin().X), scalar(a.Cap.Translation.X))
	zo := plus(scalar(p.Frame.Origin().Z), scalar(a.Cap.Translation.Z))
	ordinate := func(z *big.Rat) *big.Rat {
		if n.out < 0 {
			return new(big.Rat).Neg(z)
		}
		return z
	}
	sideMin := wmin(ordinate(n.side(xmin)), ordinate(n.side(n.last)))
	for i, seg := range p.Profile.Outer.Segments {
		if err := meter.ReserveBundle(ctx, 48, 0, 65536); err != nil {
			return false, err
		}
		role := "side(0," + strconv.Itoa(i) + ")"
		if windowRole(a.Round, role) || windowRole(a.Side, role) {
			continue
		}
		switch s := seg.(type) {
		case momentinput.LineSeg:
			if !(s.TStart >= 0 && s.TStart <= 1 && s.TEnd >= 0 && s.TEnd <= 1) {
				return false, nil
			}
			ax, bx := plus(xo, scalar(s.Start.U)), plus(xo, scalar(s.End.U))
			if wmax(ax, bx).Cmp(xmin) < 0 || wmin(ax, bx).Cmp(n.last) > 0 {
				continue
			}
			az, bz := plus(zo, scalar(s.Start.V)), plus(zo, scalar(s.End.V))
			if ordinate(minus(az, n.side(ax))).Sign() > 0 || ordinate(minus(bz, n.side(bx))).Sign() > 0 {
				return false, nil
			}
		case momentinput.ArcSeg:
			u := windowSegmentUpper(s)
			if u == nil {
				return false, nil
			}
			x := plus(xo, scalar(s.Center.U))
			r := minus(u, scalar(s.Center.U))
			if plus(x, r).Cmp(xmin) < 0 || minus(x, r).Cmp(n.last) > 0 {
				continue
			}
			z := plus(zo, scalar(s.Center.V))
			if plus(ordinate(z), r).Cmp(sideMin) > 0 {
				return false, nil
			}
		default:
			return false, nil
		}
	}
	return true, nil
}
