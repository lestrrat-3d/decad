package stackedshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// rectBoss handles only the exact two-slab, nested rectangular case, open at
// its cap end or, with no removed face, closed. An unrelated stack falls
// through to the ordinary Shell route.
func rectBoss(ctx context.Context, env topo.Env, sp stackedrecord.Payload, view breprecord.Payload,
	call Call) (*topo.Body, bool, error) {
	if !twoSlabs(sp) {
		return nil, false, nil
	}
	plate, pOK := prismshell.RecordedBossRect(sp.Slabs[0].Regions[0])
	boss, bOK := prismshell.RecordedBossRect(sp.Slabs[1].Regions[0])
	if !pOK || !bOK || !(boss.U0 > plate.U0 && boss.V0 > plate.V0 && boss.U1 < plate.U1 && boss.V1 < plate.V1) {
		return nil, false, nil
	}
	closed := call.Removed == 0
	if !call.Inward || (!closed && !call.CapEnd) {
		return nil, false, nil
	}
	refuse := func(why string) (*topo.Body, bool, error) {
		return nil, true, fmt.Errorf("%w: stacked boss shell %s", decaderr.ErrUnsupported, why)
	}
	if !exactSlabs(sp, call.TDelta) {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	t := call.T
	if !(a.Z1-a.Z0 > 2*t && boss.Width() > 2*t && boss.Height() > 2*t &&
		boss.U0 > plate.U0+t && boss.V0 > plate.V0+t && boss.U1 < plate.U1-t && boss.V1 < plate.V1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	if closed && b.Z1-b.Z0 <= t {
		return refuse("has no cavity below the kept boss roof")
	}
	lower, upper := plate.Inset(t), boss.Inset(t)
	lowerZ := a.Z1 - t
	cavityTopZ := b.Z1
	if closed {
		cavityTopZ = b.Z1 - t
		if proofbound.ExactSumRound(cavityTopZ, b.Z1, -t) != 0 {
			return refuse("needs an exactly represented cavity ceiling")
		}
	}
	if proofbound.ExactSumRound(lowerZ, a.Z1, -t) != 0 ||
		proofbound.ExactSumRound(a.Z0+t, a.Z0, t) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0, a.Z1, -a.Z0) != 0 ||
		proofbound.ExactSumRound(b.Z1-b.Z0, b.Z1, -b.Z0) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0-2*t, a.Z1, -a.Z0, -t, -t) != 0 ||
		!prismshell.BossRectArithmeticExact(plate, t) || !prismshell.BossRectArithmeticExact(boss, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	// The source view has a single top cap. All its other faces keep their
	// recorded geometry; the result gets new face(k)/wall(k) roles on rebuild.
	topCapIdx, why := soleCap(view.Faces, b.Z1, true, "top cap")
	if why != "" {
		return refuse(why)
	}
	if closed {
		topCapIdx = -1
	}
	r := newRecord(sp, view, topCapIdx, 16)
	outerFaceCount := len(r.p.Faces)
	r.plane(lower.Profile(), a.Z0+t, true)
	r.walls(lower.Profile(), a.Z0+t, lowerZ, true)
	ledge, err := annulus(ctx, lower.Profile(), boss.Profile())
	if err != nil {
		return nil, true, err
	}
	ledgeFace := r.plane(ledge, lowerZ, false)
	r.walls(upper.Profile(), b.Z0, cavityTopZ, true)
	if closed {
		r.plane(upper.Profile(), cavityTopZ, false)
	} else {
		rim, err := annulus(ctx, boss.Profile(), upper.Profile())
		if err != nil {
			return nil, true, err
		}
		r.plane(rim, b.Z1, true)
	}
	r.p.BossShell = &prismshell.BossShellBand{Plate: plate, Boss: boss, Lower: lower, Upper: upper,
		Z0: a.Z0, InterfaceZ: a.Z1, TopZ: b.Z1, CavityTopZ: cavityTopZ,
		LowerZ: lowerZ, T: t, LedgeFace: ledgeFace, Frame: sp.Frame,
		OuterFaceCount: outerFaceCount, Closed: closed}
	return r.build(ctx, env)
}

// roundBoss admits one nested, concentric whole-circle boss over a
// rectangular plate, open at its cap end. Every offset and level must be
// exactly represented.
func roundBoss(ctx context.Context, env topo.Env, sp stackedrecord.Payload, view breprecord.Payload,
	call Call) (*topo.Body, bool, error) {
	if !twoSlabs(sp) {
		return nil, false, nil
	}
	plate, pOK := prismshell.RecordedBossRect(sp.Slabs[0].Regions[0])
	circle, cOK := prismshell.RecordedBossCircle(sp.Slabs[1].Regions[0])
	if !pOK || !cOK {
		return nil, false, nil
	}
	if !call.Inward || !call.CapEnd {
		return nil, false, nil
	}
	refuse := func(why string) (*topo.Body, bool, error) {
		return nil, true, fmt.Errorf("%w: round stacked-boss shell %s", decaderr.ErrUnsupported, why)
	}
	if !exactSlabs(sp, call.TDelta) {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	t, rad, c := call.T, circle.Radius, circle.Center
	if !(t > 0 && a.Z1-a.Z0 > 2*t && rad > t &&
		c.U-rad > plate.U0+t && c.U+rad < plate.U1-t &&
		c.V-rad > plate.V0+t && c.V+rad < plate.V1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	lowerZ, innerR := a.Z1-t, rad-t
	if proofbound.ExactSumRound(lowerZ, a.Z1, -t) != 0 ||
		proofbound.ExactSumRound(a.Z0+t, a.Z0, t) != 0 ||
		proofbound.ExactSumRound(innerR, rad, -t) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0, a.Z1, -a.Z0) != 0 ||
		proofbound.ExactSumRound(b.Z1-b.Z0, b.Z1, -b.Z0) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0-2*t, a.Z1, -a.Z0, -t, -t) != 0 ||
		!prismshell.BossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, x := range []float64{c.U, c.V} {
		if proofbound.ExactSumRound(x-rad, x, -rad) != 0 ||
			proofbound.ExactSumRound(x+rad, x, rad) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	inner, ok := resizedCircle(circle.Outer, innerR)
	if !ok {
		return refuse("has no recorded whole-circle boundary")
	}
	circle.Inner = inner
	topCapIdx, why := soleCap(view.Faces, b.Z1, true, "top cap")
	if why != "" {
		return refuse(why)
	}
	r := newRecord(sp, view, topCapIdx, 12)
	lower := plate.Inset(t)
	r.plane(lower.Profile(), a.Z0+t, true)
	r.walls(lower.Profile(), a.Z0+t, lowerZ, true)
	ledge, err := annulus(ctx, lower.Profile(), circle.Outer)
	if err != nil {
		return nil, true, err
	}
	ledgeFace := r.plane(ledge, lowerZ, false)
	r.walls(circle.Inner, b.Z0, b.Z1, true)
	rim, err := annulus(ctx, circle.Outer, circle.Inner)
	if err != nil {
		return nil, true, err
	}
	r.plane(rim, b.Z1, true)
	r.p.BossShell = &prismshell.BossShellBand{Plate: plate, Lower: lower, Round: &circle,
		Z0: a.Z0, InterfaceZ: a.Z1, TopZ: b.Z1, LowerZ: lowerZ,
		T: t, LedgeFace: ledgeFace, Frame: sp.Frame}
	return r.build(ctx, env)
}
