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
