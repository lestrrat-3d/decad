package filletband

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ReadLoop classifies the recorded loop and builds its patch sequence. A
// nonnegative badCorner names a join that disagrees with Table LF; the caller
// can attach its face role and recorded segment indices to that refusal.
func ReadLoop(budget *proofbound.WorkBudget, record sectionrecord.LoopRecord,
	walks []survey2d.SideWalk, radius, tol float64) (loop Loop, joins []offset2d.Join,
	pieces []Piece, badCorner int, err error) {
	for _, w := range walks {
		loop.Walks = append(loop.Walks, Walk{
			Circular: w.IsCircular(), Closed: w.Closed, CCW: w.Th1 > w.Th0,
			Start:  Point{U: w.StartU, V: w.StartV},
			End:    Point{U: w.EndU, V: w.EndV},
			Center: Point{U: w.CU, V: w.CV}, Radius: w.Radius,
		})
	}
	if !loop.WholeTurn() {
		joins, err = OffsetJoins(budget, walks, radius, 0, tol)
		if err != nil {
			return Loop{}, nil, nil, -1, err
		}
		for k := range walks {
			if err = survey2d.WallBudgetStep(budget); err != nil {
				return Loop{}, nil, nil, -1, err
			}
			var class Corner
			class, err = CornerClass(record, walks, loop.Walks, joins, k)
			if err != nil {
				return Loop{}, nil, nil, -1, err
			}
			if class == 0 {
				return Loop{}, nil, nil, k, nil
			}
			loop.Corners = append(loop.Corners, class)
		}
	}
	pieces, err = loop.Pieces()
	if err != nil {
		return Loop{}, nil, nil, -1, err
	}
	return loop, joins, pieces, -1, nil
}

// ContourDelta bounds the contour after sphere poles consume their inward
// arcs. The surviving carriers meet at each recorded pole centre.
func ContourDelta(budget *proofbound.WorkBudget, walks []survey2d.SideWalk,
	radius, radiusDelta, tol float64) (float64, error) {
	joins, err := OffsetJoins(budget, walks, radius, radiusDelta, tol)
	if err != nil {
		return 0, err
	}
	var kept []survey2d.SideWalk
	var keptJoins []capcontour.Join
	n := len(walks)
	for i, w := range walks {
		if SphereWalk(w, radius) {
			continue
		}
		j := joins[i]
		prev := walks[(i+n-1)%n]
		if SphereWalk(prev, radius) {
			j = offset2d.Join{M: sectionrecord.Point2{U: prev.CU, V: prev.CV}, VertU: prev.CU, VertV: prev.CV}
		}
		kept = append(kept, w)
		keptJoins = append(keptJoins, capcontour.Join{
			Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M: j.M, PA: j.PA, PB: j.PB,
		})
	}
	if len(kept) < 2 {
		return 0, offset2d.ErrDrop
	}
	return capband.ContourDisplacement(kept, keptJoins, radius, radiusDelta, tol)
}
