package loftcontact

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SourceRoles names the original Loft roles an output face's triangle carries:
// a cap role for a cap face, side(0,i) for the first operand's wall, and for
// the second operand's wall the one or two side(0,i,k) halves of the wall
// cell the triangle lies in. owner is the face's operand, found false when the
// face maps to no source role.
func SourceRoles(face OutputFace, triangle [3]int,
	shell OutputShell, lower, upper []sectionrecord.LineSeg, z0, z1 *big.Rat,
	budget *proofbound.WorkBudget) ([2]string, int, bool, error) {
	if face.Kind == 0 {
		switch {
		case face.Owner == 0 && face.Level == 0, face.Owner == 1 && face.Level == 0:
			return [2]string{provenance.RoleCapStart}, face.Owner, true, nil
		case face.Owner == 1 && face.Level == 1, face.Owner == 0 && face.Level == 2:
			return [2]string{provenance.RoleCapEnd}, face.Owner, true, nil
		}
		return [2]string{}, 0, false, nil
	}
	if face.Kind != 1 || face.Segment < 0 {
		return [2]string{}, 0, false, nil
	}
	if face.Owner == 0 {
		return [2]string{fmt.Sprintf("side(0,%d)", face.Segment)}, 0, true, nil
	}
	if face.Owner != 1 || face.Segment >= len(lower) || face.Segment >= len(upper) {
		return [2]string{}, 0, false, nil
	}
	sides, ok, err := wallTriangleSides(triangle, shell, lower[face.Segment],
		upper[face.Segment], z0, z1, budget)
	if !ok {
		return [2]string{}, 0, false, err
	}
	var roles [2]string
	for side := range 2 {
		if sides&(1<<side) != 0 {
			roles[side] = fmt.Sprintf("side(0,%d,%d)", face.Segment, side)
		}
	}
	if roles[0] == "" {
		roles[0], roles[1] = roles[1], ""
	}
	return roles, 1, true, nil
}

// wallTriangleSides: the original Loft gives each wall cell two faces. An output triangle that
// crosses their exact diagonal carries both contributing source roles.
func wallTriangleSides(triangle [3]int, shell OutputShell,
	lower, upper sectionrecord.LineSeg, z0, z1 *big.Rat,
	budget *proofbound.WorkBudget) (uint8, bool, error) {
	// The height and four source corners create nine rationals before the
	// per-vertex visits below. Reserve those operations before allocating.
	if _, err := budget.StepN(9); err != nil {
		return 0, false, err
	}
	c := &Arithmetic{OK: true}
	height := c.Sub(z1, z0)
	if !c.OK || height.Sign() <= 0 {
		return 0, false, nil
	}
	lowStart, ok0 := PointOf(lower.Start)
	lowEnd, ok1 := PointOf(lower.End)
	highStart, ok2 := PointOf(upper.Start)
	highEnd, ok3 := PointOf(upper.End)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return 0, false, nil
	}
	positive, negative := false, false
	for _, id := range triangle {
		// Each point can perform at most 45 rational operations: four
		// two-coordinate interpolations (24), parameter arithmetic (6),
		// four constant comparisons (8), equality (2), and the diagonal
		// side expression (5). Reserve those and the vertex read before any
		// operation allocates its result.
		if _, err := budget.StepN(46); err != nil {
			return 0, false, err
		}
		if id < 0 || id >= len(shell.Vertices) {
			return 0, false, nil
		}
		point := shell.Vertices[id].Point
		s := c.Quo(c.Sub(point.Z, z0), height)
		start := c.Interpolate(lowStart, highStart, s)
		end := c.Interpolate(lowEnd, highEnd, s)
		du, dv := c.Sub(end.U, start.U), c.Sub(end.V, start.V)
		var t *big.Rat
		switch {
		case du.Sign() != 0:
			t = c.Quo(c.Sub(point.U, start.U), du)
		case dv.Sign() != 0:
			t = c.Quo(c.Sub(point.V, start.V), dv)
		default:
			return 0, false, nil
		}
		reconstructed := c.Interpolate(start, end, t)
		if !c.OK || c.Cmp(s, big.NewRat(0, 1)) < 0 ||
			c.Cmp(s, big.NewRat(1, 1)) > 0 || c.Cmp(t, big.NewRat(0, 1)) < 0 ||
			c.Cmp(t, big.NewRat(1, 1)) > 0 ||
			!c.SamePoint(Point{U: point.U, V: point.V}, reconstructed) {
			return 0, false, nil
		}
		diagonal := c.Interpolate(lowStart, highEnd, s)
		// The admitted wall is planar and convex. Its horizontal sections
		// stay parallel, so this sign is the side of the original straight
		// source diagonal throughout each output triangle.
		parallel := c.Add(c.Mul(c.Sub(point.U, diagonal.U), du),
			c.Mul(c.Sub(point.V, diagonal.V), dv))
		if !c.OK {
			return 0, false, nil
		}
		sign := parallel.Sign()
		positive = positive || sign > 0
		negative = negative || sign < 0
	}
	if !c.OK || !positive && !negative {
		return 0, false, nil
	}
	var sides uint8
	if positive {
		sides |= 1
	}
	if negative {
		sides |= 2
	}
	return sides, true, nil
}
