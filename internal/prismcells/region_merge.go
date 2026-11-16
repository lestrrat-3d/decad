package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// MergeLoopsWithChains is MergeLoops, also returning each loop's edges in
// record order.
func MergeLoopsWithChains(budget *proofbound.WorkBudget, selected []*sketch.Profile,
	opName string) ([]sectionrecord.LoopRecord, [][]sketch.BoundaryEdge, float64, bool, error) {
	survivors, resolved, err := mergeSurvivors(budget, selected, opName)
	if err != nil || !resolved {
		return nil, nil, 0, false, err
	}
	chains, resolved, err := ChainClosedLoops(budget, survivors)
	if err != nil || !resolved {
		return nil, nil, 0, false, err
	}
	loops := make([]sectionrecord.LoopRecord, 0, len(chains))
	cutDelta := 0.0
	for _, chain := range chains {
		loop, d, err := recordChain(budget, chain, nil)
		if err != nil {
			return nil, nil, 0, false, err
		}
		loops = append(loops, loop)
		cutDelta = math.Max(cutDelta, d)
	}
	return loops, chains, cutDelta, true, nil
}

// CutEnd reports whether the arrangement computed the parameter at one end of
// a recorded edge, and the edge's cut allowance.
func CutEnd(e sketch.BoundaryEdge, seg sectionrecord.CurveSegment, atStart bool) (bool, float64, error) {
	return cutEnd(e, seg, atStart)
}

// ChainEnd is the arranged vertex a merged chain edge ends at: its last
// polyline point.
func ChainEnd(e sketch.BoundaryEdge) ([2]float64, bool) {
	if len(e.Polyline) == 0 {
		return [2]float64{}, false
	}
	return e.Polyline[len(e.Polyline)-1], true
}
