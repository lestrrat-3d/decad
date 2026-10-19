package stackedshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// blindPocket takes only the exact two-slab rectangular blind Cut whose top
// cap is removed inward. A different stack retains route S's SG3 refusal.
func blindPocket(ctx context.Context, env topo.Env, sp stackedrecord.Payload, view breprecord.Payload,
	call Call) (*topo.Body, bool, error) {
	if !twoSlabs(sp) {
		return nil, false, nil
	}
	lower, upper := sp.Slabs[0], sp.Slabs[1]
	if len(lower.Regions[0].Holes) != 0 || len(upper.Regions[0].Holes) != 1 ||
		len(sp.Interfaces[0].LowerExposed) != 1 || len(sp.Interfaces[0].UpperExposed) != 0 ||
		!brepgeom.LoopsEqual(lower.Regions[0].Outer, upper.Regions[0].Outer) {
		return nil, false, nil
	}
	outer, oOK := prismshell.RecordedBossRect(lower.Regions[0])
	pocketLoop, err := offset2d.ReverseLoopRecordContext(ctx, upper.Regions[0].Holes[0])
	if err != nil {
		return nil, true, err
	}
	pocket, pOK := prismshell.RecordedBossRect(momentinput.Profile{Outer: pocketLoop})
	if !oOK || !pOK || !(pocket.U0 > outer.U0 && pocket.V0 > outer.V0 &&
		pocket.U1 < outer.U1 && pocket.V1 < outer.V1) {
		return nil, false, nil
	}
	if !call.Inward || !call.CapEnd {
		return nil, false, nil
	}
	refuse := func(why string) (*topo.Body, bool, error) {
		return nil, true, fmt.Errorf("%w: rectangular blind-pocket shell %s", decaderr.ErrUnsupported, why)
	}
	if !exactSlabs(sp, call.TDelta) {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.T
	if !(lower.Z1-lower.Z0 > 2*t && outer.Width() > 2*t && outer.Height() > 2*t &&
		pocket.Width() > 2*t && pocket.Height() > 2*t &&
		pocket.U0 > outer.U0+2*t && pocket.V0 > outer.V0+2*t &&
		pocket.U1 < outer.U1-2*t && pocket.V1 < outer.V1-2*t) {
		return refuse("has insufficient floor, outer-wall, pocket, or cavity clearance")
	}
	inner := outer.Inset(t)
	lowerZ := lower.Z1 - t
	if proofbound.ExactSumRound(lowerZ, lower.Z1, -t) != 0 ||
		proofbound.ExactSumRound(lower.Z0+t, lower.Z0, t) != 0 ||
		proofbound.ExactSumRound(lower.Z1-lower.Z0, lower.Z1, -lower.Z0) != 0 ||
		proofbound.ExactSumRound(upper.Z1-upper.Z0, upper.Z1, -upper.Z0) != 0 ||
		!prismshell.BossRectArithmeticExact(outer, t) || !prismshell.BossRectArithmeticExact(pocket, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	budget := proofbound.NewWorkBudget(ctx)
	rounded, err := offset2d.OffsetProfile(budget, pocket.Profile(), -1, t, offset2d.ShellTol)
	if err != nil {
		return nil, true, err
	}
	delta, err := offset2d.OffsetSectionDelta(budget, pocket.Profile(), -1, t, 0, offset2d.ShellTol)
	if err != nil {
		return nil, true, err
	}
	if delta != 0 {
		return refuse("needs an exact rounded-pocket offset")
	}
	read, err := breprecord.FilletLoopOf(budget, rounded.Outer, t, "pocket floor", freeform.NewFreeformWork())
	if err != nil {
		return nil, true, err
	}
	joins, err := filletband.OffsetJoins(budget, read.Walks, t, 0, offset2d.ShellTol)
	if err != nil {
		return nil, true, err
	}
	collapsed, err := filletband.OffsetLoop(budget, read.Walks, joins, t, offset2d.ShellTol)
	if err != nil {
		return nil, true, err
	}
	if len(read.Walks) != 8 ||
		!brepgeom.LoopsEqual(sectionrecord.LoopRecord{Segments: collapsed}, pocket.Profile().Outer) {
		return refuse("does not collapse to the source pocket floor")
	}
	for _, walk := range read.Walks {
		if walk.IsCircular() && !filletband.SphereWalk(walk, t) {
			return refuse("has a non-spherical floor corner")
		}
	}
	topCapIdx, why := soleCap(view.Faces, upper.Z1, true, "top cap")
	if why != "" {
		return refuse(why)
	}
	r := newRecord(sp, view, topCapIdx, 20)
	r.plane(inner.Profile(), lower.Z0+t, true)
	r.walls(inner.Profile(), lower.Z0+t, upper.Z1, true)
	outerRim, err := annulus(ctx, outer.Profile(), inner.Profile())
	if err != nil {
		return nil, true, err
	}
	r.plane(outerRim, upper.Z1, true)
	pocketRim, err := annulus(ctx, rounded, pocket.Profile())
	if err != nil {
		return nil, true, err
	}
	r.plane(pocketRim, upper.Z1, true)
	r.walls(rounded, lower.Z1, upper.Z1, false)
	floorFace := r.plane(pocket.Profile(), lowerZ, false)
	r.p.PocketShell = &prismshell.PocketShellBand{Outer: outer, Pocket: pocket, Inner: inner, Rounded: rounded,
		Z0: lower.Z0, FloorZ: lower.Z1, TopZ: upper.Z1, LowerZ: lowerZ, T: t,
		FloorFace: floorFace, Frame: sp.Frame}
	return r.build(ctx, env)
}

// counterbore accepts exactly two coaxial whole-circle holes in one
// rectangular plate, with the larger hole at the removed cap. It reads only
// the stacked Cut record and refuses any offset it cannot represent exactly.
func counterbore(ctx context.Context, env topo.Env, sp stackedrecord.Payload, view breprecord.Payload,
	call Call) (*topo.Body, bool, error) {
	if !twoSlabs(sp) {
		return nil, false, nil
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	lo, hi := a.Regions[0], b.Regions[0]
	if !brepgeom.LoopsEqual(lo.Outer, hi.Outer) {
		return nil, false, nil
	}
	plate, ok := prismshell.RecordedBossRect(momentinput.Profile{Outer: lo.Outer})
	if !ok {
		return nil, false, nil
	}
	lower, ok, err := prismshell.CounterboreHoleCircle(ctx, lo)
	if err != nil || !ok {
		return nil, false, err
	}
	upper, ok, err := prismshell.CounterboreHoleCircle(ctx, hi)
	if err != nil || !ok {
		return nil, false, err
	}
	if lower.Center != upper.Center || lower.Radius == upper.Radius || !call.Inward || call.Removed != 1 {
		return nil, false, nil
	}
	top := upper.Radius > lower.Radius
	if top && !call.CapEnd || !top && !call.CapStart {
		return nil, false, nil
	}
	refuse := func(why string) (*topo.Body, bool, error) {
		return nil, true, fmt.Errorf("%w: coaxial counterbore shell %s", decaderr.ErrUnsupported, why)
	}
	if !exactSlabs(sp, call.TDelta) {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	small, large := lower, upper
	farZ, nearZ, sign := a.Z0, b.Z1, 1.0
	if !top {
		small, large = upper, lower
		farZ, nearZ, sign = b.Z1, a.Z0, -1
	}
	stepZ, t := a.Z1, call.T
	farH, nearH := sign*(stepZ-farZ), sign*(nearZ-stepZ)
	if !(t > 0 && farH > 2*t && nearH > 0 && plate.Width() > 2*t && plate.Height() > 2*t) {
		return refuse("has insufficient floor, counterbore depth, or outer wall")
	}
	inset := plate.Inset(t)
	smallR, largeR := small.Radius+t, large.Radius+t
	c := small.Center
	if !(smallR < large.Radius &&
		c.U-largeR > inset.U0 && c.U+largeR < inset.U1 &&
		c.V-largeR > inset.V0 && c.V+largeR < inset.V1) {
		return refuse("has insufficient shoulder or bore-to-wall clearance")
	}
	farInnerZ, capZ := farZ+sign*t, stepZ-sign*t
	if proofbound.ExactSumRound(smallR, small.Radius, t) != 0 ||
		proofbound.ExactSumRound(largeR, large.Radius, t) != 0 ||
		proofbound.ExactSumRound(farInnerZ, farZ, sign*t) != 0 ||
		proofbound.ExactSumRound(capZ, stepZ, -sign*t) != 0 ||
		proofbound.ExactSumRound(farH, stepZ*sign, -farZ*sign) != 0 ||
		proofbound.ExactSumRound(nearH, nearZ*sign, -stepZ*sign) != 0 ||
		!prismshell.BossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, radius := range []float64{small.Radius, large.Radius, smallR, largeR} {
		if proofbound.ExactSumRound(c.U-radius, c.U, -radius) != 0 ||
			proofbound.ExactSumRound(c.U+radius, c.U, radius) != 0 ||
			proofbound.ExactSumRound(c.V-radius, c.V, -radius) != 0 ||
			proofbound.ExactSumRound(c.V+radius, c.V, radius) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	openingOutward := sign > 0
	openingIndex, why := soleCap(view.Faces, nearZ, openingOutward, "opening cap")
	if why != "" {
		return refuse(why)
	}
	smallExpanded, ok := resizedCircle(small.Outer, smallR)
	if !ok {
		return refuse("has no authenticated smaller circle")
	}
	largeExpanded, ok := resizedCircle(large.Outer, largeR)
	if !ok {
		return refuse("has no authenticated larger circle")
	}
	r := newRecord(sp, view, openingIndex, 15)
	floor, err := annulus(ctx, inset.Profile(), smallExpanded)
	if err != nil {
		return nil, true, err
	}
	r.plane(floor, farInnerZ, openingOutward)
	r.walls(inset.Profile(), farInnerZ, nearZ, true)
	outerRim, err := annulus(ctx, plate.Profile(), inset.Profile())
	if err != nil {
		return nil, true, err
	}
	r.plane(outerRim, nearZ, openingOutward)
	r.walls(smallExpanded, farInnerZ, capZ, false)
	under, err := annulus(ctx, large.Outer, smallExpanded)
	if err != nil {
		return nil, true, err
	}
	floorFace := r.plane(under, capZ, !openingOutward)
	r.walls(largeExpanded, stepZ, nearZ, false)
	innerRim, err := annulus(ctx, largeExpanded, large.Outer)
	if err != nil {
		return nil, true, err
	}
	r.plane(innerRim, nearZ, openingOutward)
	r.p.PocketShell = &prismshell.PocketShellBand{Outer: plate, Inner: inset, Rounded: largeExpanded,
		FloorZ: stepZ, LowerZ: capZ, TopZ: nearZ, Z0: farZ, T: t, FloorFace: floorFace, Frame: sp.Frame,
		Round: &prismshell.CounterboreShellRound{Small: small, Large: large, Plate: plate, Inset: inset,
			FarZ: farZ, StepZ: stepZ, NearZ: nearZ, Sign: sign, T: t, Center: c, Frame: sp.Frame}}
	return r.build(ctx, env)
}
