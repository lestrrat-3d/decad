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
)

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
