package modifyselect

import (
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MatchCorner maps a selected edge to a (loop, corner) of the section: a
// lateral edge is the vertical junction at a corner, so its two vertices are
// the corner point lifted to the two caps. A cap edge matches nothing (its
// endpoints share a cap plane), which is exactly S1's honest reading of the
// class.
func MatchCorner(budget *proofbound.WorkBudget, pp prismrecord.Payload, loops []survey2d.CornerLoop,
	e *topo.Edge) (int, int, bool, error) {
	if _, ok := e.Curve.(surfacegeom.Line3); !ok {
		return 0, 0, false, nil
	}
	if e.Start == nil || e.End == nil {
		return 0, 0, false, nil
	}
	const tol = 1e-6
	for li, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, 0, false, err
		}
		n := len(loop.Walks)
		if n == 1 && loop.Walks[0].Closed {
			continue
		}
		for ci, w := range loop.Walks {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, 0, false, err
			}
			j0 := pp.Point(w.StartU, w.StartV, pp.Z0)
			j1 := pp.Point(w.StartU, w.StartV, pp.Z1)
			if brepgeom.MatchEndpoints(e.Start.Position, e.End.Position, j0, j1, tol) {
				return li, ci, true, nil
			}
		}
	}
	return 0, 0, false, nil
}
