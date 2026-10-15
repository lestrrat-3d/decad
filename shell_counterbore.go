package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/units"
)

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
		!bossRectArithmeticExact(plate, t) {
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
