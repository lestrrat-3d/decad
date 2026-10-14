package decad

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

// tryCrossingBlindCut builds the exact rectangular case where a blind tool
// crosses one existing through hole. Its section and interface scenes are
// Sketch's; stackedbrep splits the partial hole walls at the pocket floor.
func tryCrossingBlindCut(ctx context.Context, a, b *Body) (brepPayload, bool, error) {
	target, targetOK := a.n.Payload.(prismPayload)
	tool, toolOK := b.n.Payload.(prismPayload)
	if !targetOK || !toolOK || len(target.Profile.Holes) != 1 || len(tool.Profile.Holes) != 0 {
		return brepPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	pa, pb, admitted, err := admitPrismPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !admitted {
		return brepPayload{}, false, err
	}
	if pa.SectionDelta != 0 || pb.SectionDelta != 0 ||
		pa.Z0Delta != 0 || pa.Z1Delta != 0 || pb.Z0Delta != 0 || pb.Z1Delta != 0 ||
		!crossingBlindAxisLines(pa.Profile) || !crossingBlindAxisLines(pb.Profile) {
		return brepPayload{}, false, nil
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil || !reexpress.Identity {
		return brepPayload{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(pa.PlacementOperand(), pb.PlacementOperand())
	if !ok || z0.Cmp(proofarith.FloatRat(pa.Z0)) <= 0 ||
		z0.Cmp(proofarith.FloatRat(pa.Z1)) >= 0 || z1.Cmp(proofarith.FloatRat(pa.Z1)) != 0 {
		return brepPayload{}, false, nil
	}
	va, _ := stackedUnionOperandOf(a)
	vb, _ := stackedUnionOperandOf(b)
	shift := prismplacement.ZShift(pa.PlacementOperand(), pb.PlacementOperand())
	levels, ok := stackedrecord.UnionLevels(va.slabs, vb.slabs, shift)
	if !ok || len(levels) != 3 || levels[0].Delta != 0 || levels[1].Delta != 0 || levels[2].Delta != 0 {
		return brepPayload{}, false, nil
	}
	zero := new(big.Rat)
	reach := make([][2]int, 2)
	for k := range reach {
		reach[k][0] = stackedrecord.UnionSlabOf(va.slabs, zero, levels[k].Exact, levels[k+1].Exact)
		reach[k][1] = stackedrecord.UnionSlabOf(vb.slabs, shift, levels[k].Exact, levels[k+1].Exact)
	}
	if reach[0] != [2]int{0, -1} || reach[1] != [2]int{0, 0} {
		return brepPayload{}, false, nil
	}
	state := &stackedUnionState{budget: budget, va: va, vb: vb, reexpress: reexpress}
	build := &ubBuild{st: state, cut: true, levels: levels, reach: reach,
		scenes: map[string]*ubScene{}, geom: stackedbrep.NewEngine([]float64{
			levels[0].Held, levels[1].Held, levels[2].Held,
		})}
	result, err := build.run(ctx)
	if errors.Is(err, errUBMiss) || errors.Is(err, ErrUnsupported) || errors.Is(err, brepgeom.ErrStackedWallMiss) {
		return brepPayload{}, false, nil
	}
	if err != nil {
		return brepPayload{}, false, err
	}
	return result, true, nil
}

func crossingBlindAxisLines(p profileRecord) bool {
	for _, loop := range append([]loopRecord{p.Outer}, p.Holes...) {
		if len(loop.Segments) != 4 {
			return false
		}
		for _, segment := range loop.Segments {
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok || (line.Start.U != line.End.U && line.Start.V != line.End.V) {
				return false
			}
		}
	}
	return true
}
