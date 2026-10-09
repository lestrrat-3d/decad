package box

import (
	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ContainedHorizontalGeometry is the exact four-corner patch of a rotated
// horizontal face strictly inside a signed-axis box face.
type ContainedHorizontalGeometry struct {
	Corners               [4]proofarith.DyV3
	SupportZ, Separation  proofarith.Dyadic
	BaseSide, RotatedSide int
	BaseBelow             bool
}

// ContainedHorizontalPatch proves a complete face patch at touch or shallow
// penetration. The rotated box must have one vertical edge and a horizontal
// face; its caller checks the recorded source pose and maps face identities.
func ContainedHorizontalPatch(base AxisBox, rotated OrientedBox, relation pair.Relation) (ContainedHorizontalGeometry, bool) {
	if relation != pair.Touching && relation != pair.Overlapping {
		return ContainedHorizontalGeometry{}, false
	}
	minZ, maxZ := OrientedProjection(rotated,
		proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)})
	baseBelow := proofarith.DyCmp(base.Lo[2], minZ) < 0 && proofarith.DyCmp(base.Hi[2], maxZ) < 0
	baseAbove := proofarith.DyCmp(base.Lo[2], minZ) > 0 && proofarith.DyCmp(base.Hi[2], maxZ) > 0
	if !baseBelow && !baseAbove {
		return ContainedHorizontalGeometry{}, false
	}
	var faceZ, supportZ, separation proofarith.Dyadic
	var baseSide, rotatedSide int
	if baseBelow {
		faceZ, supportZ = minZ, base.Hi[2]
		separation = proofarith.DySubScalar(faceZ, supportZ)
		baseSide, rotatedSide = 1, 0
	} else {
		faceZ, supportZ = maxZ, base.Lo[2]
		separation = proofarith.DySubScalar(supportZ, faceZ)
		baseSide, rotatedSide = 0, 1
	}
	if separation.Sign() > 0 ||
		(relation == pair.Touching && separation.Sign() != 0) ||
		(relation == pair.Overlapping && separation.Sign() >= 0) {
		return ContainedHorizontalGeometry{}, false
	}
	depth := proofarith.DyNeg(separation)
	vertical := -1
	for axis, edge := range rotated.Edge {
		if edge[0].Sign() == 0 && edge[1].Sign() == 0 && edge[2].Sign() != 0 {
			if vertical >= 0 {
				return ContainedHorizontalGeometry{}, false
			}
			vertical = axis
		}
	}
	if vertical < 0 {
		return ContainedHorizontalGeometry{}, false
	}
	start := 0
	if proofarith.DyCmp(rotated.Corner[0][2], faceZ) != 0 {
		start = 1 << vertical
	}
	i, j := (vertical+1)%3, (vertical+2)%3
	indices := [4]int{start, start | (1 << i), start | (1 << i) | (1 << j), start | (1 << j)}
	var corners [4]proofarith.DyV3
	for n, index := range indices {
		corner := rotated.Corner[index]
		if proofarith.DyCmp(corner[2], faceZ) != 0 {
			return ContainedHorizontalGeometry{}, false
		}
		for axis := range 2 {
			if proofarith.DyCmp(corner[axis], proofarith.DyAdd(base.Lo[axis], depth)) <= 0 ||
				proofarith.DyCmp(corner[axis], proofarith.DySubScalar(base.Hi[axis], depth)) >= 0 {
				return ContainedHorizontalGeometry{}, false
			}
		}
		corners[n] = corner
	}
	if relation == pair.Overlapping &&
		((baseBelow && proofarith.DyCmp(maxZ, base.Hi[2]) <= 0) ||
			(baseAbove && proofarith.DyCmp(minZ, base.Lo[2]) >= 0)) {
		return ContainedHorizontalGeometry{}, false
	}
	return ContainedHorizontalGeometry{
		Corners: corners, SupportZ: supportZ, Separation: separation,
		BaseSide: baseSide, RotatedSide: rotatedSide, BaseBelow: baseBelow,
	}, true
}
