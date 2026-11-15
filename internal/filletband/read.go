package filletband

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ReadLoop classifies the recorded loop for a fillet of radius r with
// conversion bound rDelta and builds its patch sequence. A nonnegative
// badCorner names a join that disagrees with Table LF, and badWhy the
// near-tangent rule's reason when one refused it ("" otherwise); the caller
// can attach its face role and recorded segment indices to that refusal.
//
// Every wall with an LF5′ end must pass capband.NearTangentWallRefusal, the
// survival gate §5.5's corner terms assume: its true edge runs from its start
// corner to its end corner at every level. A wall whose other end is LF8 or
// LF6 refuses, since that gate does not enclose such an end. A wall that fails
// names the LF5′ corner at its start, or at its end when only that one is LF5′.
func ReadLoop(budget *proofbound.WorkBudget, record sectionrecord.LoopRecord,
	walks []survey2d.SideWalk, radius, radiusDelta, tol float64) (loop Loop, joins []offset2d.Join,
	pieces []Piece, badCorner int, badWhy string, err error) {
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
			return Loop{}, nil, nil, -1, "", err
		}
		rho := make([]float64, len(walks))
		near := false
		for k := range walks {
			if err = survey2d.WallBudgetStep(budget); err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			class, reach, why, err := CornerClass(record, walks, loop.Walks, joins, k, radius, radiusDelta)
			if err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			if class == 0 {
				return Loop{}, nil, nil, k, why, nil
			}
			loop.Corners = append(loop.Corners, class)
			rho[k] = reach
			near = near || class == NearTangent
		}
		if near {
			loop.Rho = rho
			k, why, err := nearTangentWalls(budget, record, walks, loop.Corners, rho, radius, radiusDelta)
			if err != nil {
				return Loop{}, nil, nil, -1, "", err
			}
			if why != "" {
				return Loop{}, nil, nil, k, why, nil
			}
		}
	}
	pieces, err = loop.Pieces()
	if err != nil {
		return Loop{}, nil, nil, -1, "", err
	}
	return loop, joins, pieces, -1, "", nil
}

// nearTangentWalls runs capband.NearTangentWallRefusal on every wall with an
// LF5′ end, every walk moving by radius over radiusDelta. It returns the LF5′
// corner a failing wall names and the refusal's reason, or "" when every such
// wall survives; err is a budget or context error.
func nearTangentWalls(budget *proofbound.WorkBudget, record sectionrecord.LoopRecord, walks []survey2d.SideWalk,
	corners []Corner, rho []float64, radius, radiusDelta float64) (int, string, error) {
	n := len(walks)
	amounts := make([]float64, n)
	for i := range amounts {
		amounts[i] = radius
	}
	for i := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return -1, "", err
		}
		next := (i + 1) % n
		if corners[i] != NearTangent && corners[next] != NearTangent {
			continue
		}
		// The gate reads a far end as the carriers' miter or a circular
		// neighbour's radial foot, which an LF8 or LF6 end is not, so such a
		// wall is refused rather than gated on the wrong end.
		for _, k := range [2]int{i, next} {
			if corners[k] == CurvedMiter || corners[k] == Reflex {
				end := i
				if corners[i] != NearTangent {
					end = next
				}
				return end, nearTangentOtherEndWhy, nil
			}
		}
		if why := capband.NearTangentWallRefusal(record, walks, i, amounts, radiusDelta, rho[i], rho[next], false); why != "" {
			if corners[i] == NearTangent {
				return i, why, nil
			}
			return next, why, nil
		}
	}
	return -1, "", nil
}

// nearTangentOtherEndWhy refuses a wall with an LF5′ end whose other end is an
// LF8 or LF6 corner, which the near-tangent survival gate does not enclose.
const nearTangentOtherEndWhy = `a wall at a near-tangent join whose other end is a curved miter or a reflex corner, which the near-tangent survival gate does not enclose`

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
