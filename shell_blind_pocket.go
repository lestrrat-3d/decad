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
		!bossRectArithmeticExact(outer, t) || !bossRectArithmeticExact(pocket, t) {
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
