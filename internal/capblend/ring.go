package capblend

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SideRing writes one loop's side ring at both trimmed wall levels. The
// caller owns vertex storage; motion receives the bounds in vertex order.
func SideRing(budget *proofbound.WorkBudget, points []sectionrecord.Point2,
	bounds []proofbound.WalkEndBound, lo, hi proofbound.BoundedScalar,
	addVertex func(sectionrecord.Point2, float64, float64) (int, float64),
	motion *[]float64, proveVolume bool) ([]int, []int, error) {
	sideLo := make([]int, len(points))
	sideHi := make([]int, len(points))
	for j, p := range points {
		if err := budget.Step(); err != nil {
			return nil, nil, err
		}
		plane := proofbound.WalkEndBoundAllow(bounds[j])
		var round float64
		sideLo[j], round = addVertex(p, lo.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, lo.Bound))
		}
		sideHi[j], round = addVertex(p, hi.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, hi.Bound))
		}
	}
	return sideLo, sideHi, nil
}

// DraftNearRing writes the near cap's rim once. Both returned slices name
// that same ring, so the cap and wall use the same vertices.
func DraftNearRing(budget *proofbound.WorkBudget, points []sectionrecord.Point2,
	bounds []proofbound.WalkEndBound, near proofbound.BoundedScalar,
	addVertex func(sectionrecord.Point2, float64, float64) (int, float64),
	motion *[]float64, proveVolume bool) ([]int, []int, error) {
	side := make([]int, len(points))
	for j, p := range points {
		if err := budget.Step(); err != nil {
			return nil, nil, err
		}
		plane := proofbound.WalkEndBoundAllow(bounds[j])
		var round float64
		side[j], round = addVertex(p, near.Value, plane)
		if proveVolume {
			*motion = append(*motion, proofbound.AbsSumUpper(plane, round, near.Bound))
		}
	}
	return side, side, nil
}
