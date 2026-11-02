package stackedrecord

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindTessStructure rechecks a built two-slab blind result and derives
// its columns and exposed patches without running unmetered stack builders.
// This is structural preparation only; no chord or mesh proof is supplied.
func ReplayBlindTessStructure(ctx context.Context, body *topo.Body,
	budget *proofbound.ReplayBudget) ([]WallColumn, [][]SlabLoop, []Patch, bool, error) {
	if budget == nil || body == nil || !body.HasStrictPrismBuildSeal() {
		return nil, nil, nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, nil, nil, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok || len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 || sp.ClosedShell ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, nil, nil, false, nil
	}
	if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(momentinput.Profile{}))); err != nil {
		return nil, nil, nil, false, err
	}
	first, second := sp.Slabs[0].Regions[0], sp.Slabs[1].Regions[0]
	views := []sectionrecord.AnalyticProfileView{first.AnalyticSealView(), second.AnalyticSealView()}
	sealed, err := body.ReplayPrismBuildPayload(ctx, sp.Frame, sp.Xform,
		sp.Slabs[0].Z0, sp.Slabs[1].Z1, sp.Slabs[0].Z0Delta, sp.Slabs[1].Z1Delta,
		sp.SectionDelta, sp.Slabs[0].Z0Denoted, sp.Slabs[1].Z1Denoted, views, budget)
	if err != nil || !sealed {
		return nil, nil, nil, false, err
	}
	if _, sealed, err := body.ReplayPrismBuiltBounds(ctx, budget); err != nil || !sealed {
		return nil, nil, nil, false, err
	}
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, nil, nil, false, err
		}
		if lump == nil {
			return nil, nil, nil, false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, nil, nil, false, err
			}
			if shell == nil {
				return nil, nil, nil, false, nil
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, nil, nil, false, err
				}
				sealed, err := body.ReplayPrismBuiltFace(ctx, face, budget)
				if err != nil || !sealed {
					return nil, nil, nil, false, err
				}
			}
		}
	}
	top := len(sp.Interfaces[0].LowerExposed) > 0
	floors := sp.Interfaces[0].LowerExposed
	uncut, cut := first, second
	if !top {
		floors = sp.Interfaces[0].UpperExposed
		uncut, cut = second, first
	}
	if len(floors) == 0 || len(floors) > 4096 || len(cut.Holes) <= len(uncut.Holes) {
		return nil, nil, nil, false, nil
	}
	if err := budget.ReserveArray(ctx, uint64(len(floors)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, nil, nil, false, err
	}
	regions := make([]int, len(floors))
	for i := range regions {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, nil, nil, false, err
		}
		regions[i] = i
	}
	target := prismrecord.Payload{Profile: uncut, Frame: sp.Frame, Xform: sp.Xform,
		Z0: sp.Slabs[0].Z0, Z1: sp.Slabs[1].Z1,
		Z0Delta: sp.Slabs[0].Z0Delta, Z1Delta: sp.Slabs[1].Z1Delta,
		Z0Denoted: sp.Slabs[0].Z0Denoted, Z1Denoted: sp.Slabs[1].Z1Denoted,
		SectionDelta: sp.SectionDelta}
	section := prismcells.ReplayBlindSection{Profile: cut, HoleRegions: regions, SectionDelta: sp.SectionDelta}
	inner := sp.Slabs[0].Z1
	valid, err := replayValidateBlindGroup(ctx, sp, target, section, inner, top, budget)
	if err != nil || !valid {
		return nil, nil, nil, false, err
	}
	columns, rows, valid, err := replayBlindColumns(ctx, sp, budget)
	if err != nil || !valid {
		return nil, nil, nil, false, err
	}
	patches, valid, err := replayBlindPatches(ctx, sp, target, section, inner, top, columns, rows, budget)
	if err != nil || !valid {
		return nil, nil, nil, false, err
	}
	denoted, valid, err := prismplacement.ReplayDenotedLevel(ctx, inner,
		sp.Slabs[0].Z1Delta, sp.Slabs[0].Z1Denoted, budget)
	if err != nil || !valid {
		return nil, nil, nil, false, err
	}
	for _, patch := range patches {
		matches := 0
		for _, lump := range body.Lumps {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, nil, nil, false, err
			}
			for _, shell := range lump.Shells {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, nil, nil, false, err
				}
				for _, face := range shell.Faces {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, nil, nil, false, err
					}
					if len(face.Origins) != 1 || face.Origins[0].Role != patch.Role {
						continue
					}
					matches++
					if face.CapRecord == nil || math.Float64bits(face.CapRecord.Z) != math.Float64bits(inner) ||
						face.CapRecord.ZDenoted == nil {
						return nil, nil, nil, false, nil
					}
					cmp, err := proof.ReplayRatCompare(ctx, face.CapRecord.ZDenoted, denoted, budget)
					if err != nil || cmp != 0 {
						return nil, nil, nil, false, err
					}
				}
			}
		}
		if matches != 1 {
			return nil, nil, nil, false, nil
		}
	}
	return columns, rows, patches, true, nil
}
