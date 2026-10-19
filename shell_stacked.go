package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/units"
)

// shellBlindPocket takes only the exact two-slab rectangular blind Cut whose
// top cap is removed inward. A different stack retains route S's SG3 refusal.
func shellBlindPocket(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
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
	pocket, pOK := prismshell.RecordedBossRect(profileRecord{Outer: pocketLoop})
	if !oOK || !pOK || !(pocket.U0 > outer.U0 && pocket.V0 > outer.V0 &&
		pocket.U1 < outer.U1 && pocket.V1 < outer.V1) {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 ||
		call.removed[0] != facesByRole(source)[provenance.RoleCapEnd] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: rectangular blind-pocket shell %s", ErrUnsupported, why)
	}
	if sp.SectionDelta != 0 || lower.Z0Delta != 0 || lower.Z1Delta != 0 ||
		upper.Z0Delta != 0 || upper.Z1Delta != 0 || lower.Z1 != upper.Z0 ||
		!(lower.Z0 < lower.Z1 && upper.Z0 < upper.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.tmm
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
	rounded, err := offsetProfile(budget, pocket.Profile(), -1, t)
	if err != nil {
		return nil, true, err
	}
	delta, err := offsetSectionDelta(budget, pocket.Profile(), -1, t, 0)
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
	joins, err := filletband.OffsetJoins(budget, read.Walks, t, 0, shellTol)
	if err != nil {
		return nil, true, err
	}
	collapsed, err := filletband.OffsetLoop(budget, read.Walks, joins, t, shellTol)
	if err != nil {
		return nil, true, err
	}
	if len(read.Walks) != 8 || !brepgeom.LoopsEqual(loopRecord{Segments: collapsed}, pocket.Profile().Outer) {
		return refuse("does not collapse to the source pocket floor")
	}
	for _, walk := range read.Walks {
		if walk.IsCircular() && !filletband.SphereWalk(walk, t) {
			return refuse("has a non-spherical floor corner")
		}
	}
	topCapIdx := -1
	for i, face := range view.Faces {
		if face.Planar() && face.Outward && face.Z0 == upper.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{Xform: view.Xform, Faces: make([]brepFace, 0, len(view.Faces)+20)}
	result.Faces = append(result.Faces, view.Faces[:topCapIdx]...)
	result.Faces = append(result.Faces, view.Faces[topCapIdx+1:]...)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Region: &p, Z0: z, Z1: z, Outward: outward})
		return len(result.Faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reversed bool) {
		for _, seg := range p.Outer.Segments {
			if reversed {
				seg = brepgeom.ReverseSegment(seg)
			}
			result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Wall: seg, Z0: z0, Z1: z1})
		}
	}
	addPlane(inner.Profile(), lower.Z0+t, true)
	addWalls(inner.Profile(), lower.Z0+t, upper.Z1, true)
	outerRim := outer.Profile()
	outerRim.Holes = []loopRecord{}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, inner.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	outerRim.Holes = append(outerRim.Holes, hole)
	addPlane(outerRim, upper.Z1, true)
	pocketRim := rounded
	hole, err = offset2d.ReverseLoopRecordContext(ctx, pocket.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	pocketRim.Holes = []loopRecord{hole}
	addPlane(pocketRim, upper.Z1, true)
	addWalls(rounded, lower.Z1, upper.Z1, false)
	floorFace := addPlane(pocket.Profile(), lowerZ, false)
	result.PocketShell = &prismshell.PocketShellBand{Outer: outer, Pocket: pocket, Inner: inner, Rounded: rounded,
		Z0: lower.Z0, FloorZ: lower.Z1, TopZ: upper.Z1, LowerZ: lowerZ, T: t,
		FloorFace: floorFace, Frame: sp.Frame}
	result.AssignRoles()
	body, err := bodyResult(breprecord.Build(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), result))
	return body, true, err
}

// shellCounterbore accepts exactly two coaxial whole-circle holes in one
// rectangular plate, with the larger hole at the removed end. It reads only
// the stacked Cut record and refuses any offset it cannot represent exactly.
func shellCounterbore(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	lo, hi := a.Regions[0], b.Regions[0]
	if !brepgeom.LoopsEqual(lo.Outer, hi.Outer) {
		return nil, false, nil
	}
	plate, ok := prismshell.RecordedBossRect(profileRecord{Outer: lo.Outer})
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
	if lower.Center != upper.Center || lower.Radius == upper.Radius ||
		call.sense != Inward || len(call.removed) != 1 {
		return nil, false, nil
	}
	top := upper.Radius > lower.Radius
	roles := facesByRole(source)
	if top && call.removed[0] != roles[provenance.RoleCapEnd] ||
		!top && call.removed[0] != roles[provenance.RoleCapStart] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: coaxial counterbore shell %s", ErrUnsupported, why)
	}
	if sp.SectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 ||
		b.Z0Delta != 0 || b.Z1Delta != 0 || a.Z1 != b.Z0 ||
		!(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	small, large := lower, upper
	farZ, nearZ, sign := a.Z0, b.Z1, 1.0
	if !top {
		small, large = upper, lower
		farZ, nearZ, sign = b.Z1, a.Z0, -1
	}
	stepZ, t := a.Z1, call.tmm
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
	openingZ := nearZ
	openingOutward := sign > 0
	openingIndex := -1
	for i, face := range view.Faces {
		if face.Planar() && face.Outward == openingOutward && face.Z0 == openingZ {
			if openingIndex >= 0 {
				return refuse("has more than one opening cap")
			}
			openingIndex = i
		}
	}
	if openingIndex < 0 {
		return refuse("has no opening cap")
	}
	makeCircle := func(orig prismshell.RoundBossCircle, radius float64) (profileRecord, bool) {
		circle, ok := orig.Outer.Outer.Segments[0].(circleSeg)
		if !ok {
			return profileRecord{}, false
		}
		circle.Radius = units.Millimeters(radius)
		return profileRecord{Outer: loopRecord{Segments: []curveSegment{circle}}}, true
	}
	smallExpanded, ok := makeCircle(small, smallR)
	if !ok {
		return refuse("has no authenticated smaller circle")
	}
	largeExpanded, ok := makeCircle(large, largeR)
	if !ok {
		return refuse("has no authenticated larger circle")
	}
	result := brepPayload{Xform: view.Xform, Faces: make([]brepFace, 0, len(view.Faces)+15)}
	result.Faces = append(result.Faces, view.Faces[:openingIndex]...)
	result.Faces = append(result.Faces, view.Faces[openingIndex+1:]...)
	annulus := func(outer, hole profileRecord) (profileRecord, error) {
		p := outer
		reversed, err := offset2d.ReverseLoopRecordContext(ctx, hole.Outer)
		if err != nil {
			return profileRecord{}, err
		}
		p.Holes = []loopRecord{reversed}
		return p, nil
	}
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Region: &p, Z0: z, Z1: z, Outward: outward})
		return len(result.Faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reverse bool) {
		if z0 > z1 {
			z0, z1 = z1, z0
		}
		for _, seg := range p.Outer.Segments {
			if reverse {
				seg = brepgeom.ReverseSegment(seg)
			}
			result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Wall: seg, Z0: z0, Z1: z1})
		}
	}
	floor, err := annulus(inset.Profile(), smallExpanded)
	if err != nil {
		return nil, true, err
	}
	addPlane(floor, farInnerZ, openingOutward)
	addWalls(inset.Profile(), farInnerZ, nearZ, true)
	outerRim, err := annulus(plate.Profile(), inset.Profile())
	if err != nil {
		return nil, true, err
	}
	addPlane(outerRim, nearZ, openingOutward)
	addWalls(smallExpanded, farInnerZ, capZ, false)
	under, err := annulus(large.Outer, smallExpanded)
	if err != nil {
		return nil, true, err
	}
	floorFace := addPlane(under, capZ, !openingOutward)
	addWalls(largeExpanded, stepZ, nearZ, false)
	innerRim, err := annulus(largeExpanded, large.Outer)
	if err != nil {
		return nil, true, err
	}
	addPlane(innerRim, nearZ, openingOutward)
	result.PocketShell = &prismshell.PocketShellBand{Outer: plate, Inner: inset, Rounded: largeExpanded,
		FloorZ: stepZ, LowerZ: capZ, TopZ: nearZ, Z0: farZ, T: t, FloorFace: floorFace, Frame: sp.Frame,
		Round: &prismshell.CounterboreShellRound{Small: small, Large: large, Plate: plate, Inset: inset,
			FarZ: farZ, StepZ: stepZ, NearZ: nearZ, Sign: sign, T: t, Center: c, Frame: sp.Frame}}
	result.AssignRoles()
	body, err := bodyResult(breprecord.Build(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), result))
	return body, true, err
}

// shellRoundBoss admits one nested, concentric whole-circle boss over a
// rectangular plate. Every offset and level must be exactly represented.
func shellRoundBoss(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	plate, pOK := prismshell.RecordedBossRect(sp.Slabs[0].Regions[0])
	circle, cOK := prismshell.RecordedBossCircle(sp.Slabs[1].Regions[0])
	if !pOK || !cOK {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 ||
		call.removed[0] != facesByRole(source)[provenance.RoleCapEnd] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: round stacked-boss shell %s", ErrUnsupported, why)
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	if sp.SectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 ||
		a.Z1 != b.Z0 || !(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t, r, c := call.tmm, circle.Radius, circle.Center
	if !(t > 0 && a.Z1-a.Z0 > 2*t && r > t &&
		c.U-r > plate.U0+t && c.U+r < plate.U1-t &&
		c.V-r > plate.V0+t && c.V+r < plate.V1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	lowerZ, innerR := a.Z1-t, r-t
	if proofbound.ExactSumRound(lowerZ, a.Z1, -t) != 0 ||
		proofbound.ExactSumRound(a.Z0+t, a.Z0, t) != 0 ||
		proofbound.ExactSumRound(innerR, r, -t) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0, a.Z1, -a.Z0) != 0 ||
		proofbound.ExactSumRound(b.Z1-b.Z0, b.Z1, -b.Z0) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0-2*t, a.Z1, -a.Z0, -t, -t) != 0 ||
		!prismshell.BossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, x := range []float64{c.U, c.V} {
		if proofbound.ExactSumRound(x-r, x, -r) != 0 ||
			proofbound.ExactSumRound(x+r, x, r) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	inner, ok := circle.Outer.Outer.Segments[0].(circleSeg)
	if !ok {
		return refuse("has no recorded whole-circle boundary")
	}
	inner.Radius = units.Millimeters(innerR)
	circle.Inner = profileRecord{Outer: loopRecord{Segments: []curveSegment{inner}}}
	topCapIdx := -1
	for i, face := range view.Faces {
		if face.Planar() && face.Outward && face.Z0 == b.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{Xform: view.Xform, Faces: make([]brepFace, 0, len(view.Faces)+12)}
	result.Faces = append(result.Faces, view.Faces[:topCapIdx]...)
	result.Faces = append(result.Faces, view.Faces[topCapIdx+1:]...)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Region: &p, Z0: z, Z1: z, Outward: outward})
		return len(result.Faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64) {
		for _, seg := range p.Outer.Segments {
			result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Wall: brepgeom.ReverseSegment(seg), Z0: z0, Z1: z1})
		}
	}
	lower := plate.Inset(t)
	addPlane(lower.Profile(), a.Z0+t, true)
	addWalls(lower.Profile(), a.Z0+t, lowerZ)
	ledge := lower.Profile()
	hole, err := offset2d.ReverseLoopRecordContext(ctx, circle.Outer.Outer)
	if err != nil {
		return nil, true, err
	}
	ledge.Holes = []loopRecord{hole}
	ledgeFace := addPlane(ledge, lowerZ, false)
	addWalls(circle.Inner, b.Z0, b.Z1)
	rim := circle.Outer
	hole, err = offset2d.ReverseLoopRecordContext(ctx, circle.Inner.Outer)
	if err != nil {
		return nil, true, err
	}
	rim.Holes = []loopRecord{hole}
	addPlane(rim, b.Z1, true)
	result.BossShell = &prismshell.BossShellBand{Plate: plate, Lower: lower, Round: &circle,
		Z0: a.Z0, InterfaceZ: a.Z1, TopZ: b.Z1, LowerZ: lowerZ,
		T: t, LedgeFace: ledgeFace, Frame: sp.Frame}
	result.AssignRoles()
	body, err := bodyResult(breprecord.Build(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), result))
	return body, true, err
}

// shellStackedBoss handles only the exact two-slab, nested rectangular case.
// An unrelated stack falls through to the ordinary Shell route.
func shellStackedBoss(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 || len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	plate, pOK := prismshell.RecordedBossRect(sp.Slabs[0].Regions[0])
	boss, bOK := prismshell.RecordedBossRect(sp.Slabs[1].Regions[0])
	if !pOK || !bOK || !(boss.U0 > plate.U0 && boss.V0 > plate.V0 && boss.U1 < plate.U1 && boss.V1 < plate.V1) {
		return nil, false, nil
	}
	closed := len(call.removed) == 0
	if call.sense != Inward || (!closed &&
		(len(call.removed) != 1 || call.removed[0] != facesByRole(source)[provenance.RoleCapEnd])) {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: stacked boss shell %s", ErrUnsupported, why)
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	if sp.SectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 ||
		a.Z1 != b.Z0 || !(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.tmm
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
	topCapIdx := -1
	for i, f := range view.Faces {
		if f.Planar() && f.Outward && f.Z0 == b.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{Xform: view.Xform, Faces: make([]brepFace, 0, len(view.Faces)+16)}
	if closed {
		result.Faces = append(result.Faces, view.Faces...)
	} else {
		result.Faces = append(result.Faces, view.Faces[:topCapIdx]...)
		result.Faces = append(result.Faces, view.Faces[topCapIdx+1:]...)
	}
	outerFaceCount := len(result.Faces)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Region: &p, Z0: z, Z1: z, Outward: outward})
		return len(result.Faces) - 1
	}
	addWalls := func(r prismshell.BossRect, z0, z1 float64) {
		for _, seg := range r.Profile().Outer.Segments {
			result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Wall: brepgeom.ReverseSegment(seg), Z0: z0, Z1: z1})
		}
	}
	addPlane(lower.Profile(), a.Z0+t, true)
	addWalls(lower, a.Z0+t, lowerZ)
	ledge := profileRecord{Outer: lower.Profile().Outer}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, boss.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	ledge.Holes = []loopRecord{hole}
	ledgeFace := addPlane(ledge, lowerZ, false)
	addWalls(upper, b.Z0, cavityTopZ)
	if closed {
		addPlane(upper.Profile(), cavityTopZ, false)
	} else {
		rim := profileRecord{Outer: boss.Profile().Outer}
		hole, err = offset2d.ReverseLoopRecordContext(ctx, upper.Profile().Outer)
		if err != nil {
			return nil, true, err
		}
		rim.Holes = []loopRecord{hole}
		addPlane(rim, b.Z1, true)
	}
	result.BossShell = &prismshell.BossShellBand{Plate: plate, Boss: boss, Lower: lower, Upper: upper,
		Z0: a.Z0, InterfaceZ: a.Z1, TopZ: b.Z1, CavityTopZ: cavityTopZ,
		LowerZ: lowerZ, T: t, LedgeFace: ledgeFace, Frame: sp.Frame,
		OuterFaceCount: outerFaceCount, Closed: closed}
	result.AssignRoles()
	body, err := bodyResult(breprecord.Build(ctx, envOf(source.n.Doc, source.n.Doc.NextProducer), result))
	return body, true, err
}
