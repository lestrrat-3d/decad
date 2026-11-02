package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// GroupWholeCircleOrigins records the disjoint whole-circle Union loops and the
// scene source of each resulting segment. It refuses partial or multi-edge
// cells. Its caller must authenticate the scene and carry the origins through
// each private Union step before treating them as source lineage.
func GroupWholeCircleOrigins(budget *proofbound.WorkBudget, scene GroupScene) ([]sectionrecord.LoopRecord, [][]Origin, float64, bool, error) {
	if err := budget.Err(); err != nil {
		return nil, nil, 0, false, err
	}
	// Every admitted cell has exactly one whole circular boundary. Each debit
	// precedes the map and slice growth in mergeSurvivors and ChainClosedLoops.
	for i := range scene.Profiles {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, false, err
		}
		cell := scene.Profiles[i]
		if cell == nil || !cell.Valid || len(cell.Holes) != 0 || len(cell.Outer) != 1 {
			return nil, nil, 0, false, nil
		}
		edge := cell.Outer[0]
		if _, ok := edge.Entity.(*sketch.Circle); !ok || edge.Partial {
			return nil, nil, 0, false, nil
		}
		if _, ok := scene.Tags[edge.Entity]; !ok {
			return nil, nil, 0, false, nil
		}
		if err := budget.Reserve(64); err != nil {
			return nil, nil, 0, false, err
		}
		if err := budget.ReserveBytes(4096); err != nil {
			return nil, nil, 0, false, err
		}
	}
	if scene.Delta.sharedDisplaced {
		return nil, nil, 0, false, nil
	}
	if ok, err := CellsHaveNoVoid(budget, scene.Tags, scene.Profiles); err != nil || !ok {
		return nil, nil, 0, false, err
	}
	survivors, ok, err := mergeSurvivors(budget, scene.Profiles, "union")
	if err != nil || !ok {
		return nil, nil, 0, false, err
	}
	chains, ok, err := ChainClosedLoops(budget, survivors)
	if err != nil || !ok {
		return nil, nil, 0, false, err
	}
	if len(chains) != len(scene.Profiles) {
		return nil, nil, 0, false, nil
	}
	if err := budget.ReserveMul(uint64(len(chains)), 2); err != nil {
		return nil, nil, 0, false, err
	}
	if err := budget.ReserveBytesMul(uint64(len(chains)), 128); err != nil {
		return nil, nil, 0, false, err
	}
	loops := make([]sectionrecord.LoopRecord, 0, len(chains))
	origins := make([][]Origin, 0, len(chains))
	var cutDelta float64
	for i := range chains {
		if err := budget.Step(); err != nil {
			return nil, nil, 0, false, err
		}
		chain := chains[i]
		if len(chain) != 1 || chain[0].Partial {
			return nil, nil, 0, false, nil
		}
		origin, ok := scene.Tags[chain[0].Entity]
		if !ok {
			return nil, nil, 0, false, nil
		}
		if err := budget.Reserve(1); err != nil {
			return nil, nil, 0, false, err
		}
		if err := budget.ReserveBytes(256); err != nil {
			return nil, nil, 0, false, err
		}
		loop, delta, err := recordChain(budget, chain, nil)
		if err != nil {
			return nil, nil, 0, false, err
		}
		loops = append(loops, loop)
		origins = append(origins, []Origin{origin})
		cutDelta = math.Max(cutDelta, delta)
	}
	return loops, origins, cutDelta, true, nil
}
