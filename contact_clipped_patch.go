package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// publishClippedHorizontalPatch certifies the complete intersection of two
// horizontal source faces when one box is rotated in the world XY plane.
func publishClippedHorizontalPatch(report *ContactReport, a, b orientedSourceBox) {
	if base, ok := axisAlignedOrientedBox(a); ok &&
		publishClippedHorizontalPatchOrder(report, base, b, true) {
		return
	}
	if base, ok := axisAlignedOrientedBox(b); ok {
		publishClippedHorizontalPatchOrder(report, base, a, false)
	}
}

func axisAlignedOrientedBox(box orientedSourceBox) (sourceBoxContactProof, bool) {
	var aligned sourceBoxContactProof
	var used [3]bool
	for sourceAxis, edge := range box.edge {
		worldAxis := -1
		for axis := range 3 {
			if edge[axis].Sign() != 0 {
				if worldAxis >= 0 {
					return sourceBoxContactProof{}, false
				}
				worldAxis = axis
			}
		}
		if worldAxis < 0 || used[worldAxis] {
			return sourceBoxContactProof{}, false
		}
		used[worldAxis] = true
		for side := range 2 {
			worldSide := side
			if edge[worldAxis].Sign() < 0 {
				worldSide = 1 - side
			}
			aligned.faces[worldAxis][worldSide] = box.faces[sourceAxis][side]
		}
	}
	for axis := range 3 {
		aligned.lo[axis], aligned.hi[axis] = box.corner[0][axis], box.corner[0][axis]
		for _, corner := range box.corner[1:] {
			aligned.lo[axis] = dyMin(aligned.lo[axis], corner[axis])
			aligned.hi[axis] = dyMax(aligned.hi[axis], corner[axis])
		}
	}
	return aligned, true
}

func publishClippedHorizontalPatchOrder(report *ContactReport, base sourceBoxContactProof,
	rotated orientedSourceBox, baseIsA bool) bool {
	if report.Relation != ContactTouching {
		return false
	}
	vertical := -1
	for axis, edge := range rotated.edge {
		if edge[0].Sign() == 0 && edge[1].Sign() == 0 && edge[2].Sign() != 0 {
			if vertical >= 0 {
				return false
			}
			vertical = axis
		}
	}
	if vertical < 0 {
		return false
	}
	minZ, maxZ := orientedProjection(rotated, proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)})
	var faceZ proofarith.Dyadic
	baseSide := 0
	normalZ := -1.0
	switch {
	case proofarith.DyCmp(base.hi[2], minZ) == 0 && proofarith.DyCmp(base.lo[2], minZ) < 0:
		faceZ, baseSide, normalZ = minZ, 1, 1
	case proofarith.DyCmp(maxZ, base.lo[2]) == 0 && proofarith.DyCmp(base.hi[2], maxZ) > 0:
		faceZ, baseSide = maxZ, 0
	default:
		return false
	}
	start := 0
	rotatedSide := 0
	if proofarith.DyCmp(rotated.corner[0][2], faceZ) != 0 {
		start = 1 << vertical
		rotatedSide = 1
	}
	i, j := (vertical+1)%3, (vertical+2)%3
	indices := [4]int{start, start | (1 << i), start | (1 << i) | (1 << j), start | (1 << j)}
	polygon := make([][2]*big.Rat, 0, 4)
	for _, index := range indices {
		corner := rotated.corner[index]
		if proofarith.DyCmp(corner[2], faceZ) != 0 {
			return false
		}
		polygon = append(polygon, [2]*big.Rat{corner[0].Rat(), corner[1].Rat()})
	}
	for axis := range 2 {
		polygon = clipHorizontalPolygon(polygon, axis, base.lo[axis].Rat(), false)
		polygon = clipHorizontalPolygon(polygon, axis, base.hi[axis].Rat(), true)
	}
	polygon = deduplicateHorizontalPolygon(polygon)
	if len(polygon) < 3 || horizontalPolygonDoubleArea(polygon).Sign() == 0 {
		return false
	}
	if !baseIsA {
		normalZ = -normalZ
	}
	faceBase, faceRotated := base.faces[2][baseSide], rotated.faces[vertical][rotatedSide]
	if faceBase == nil || faceRotated == nil {
		return false
	}
	points := make([]ContactPoint, 0, len(polygon))
	for _, vertex := range polygon {
		world := [3]*big.Rat{vertex[0], vertex[1], faceZ.Rat()}
		witness, valid := orientedBoxPoint(world)
		if !valid || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return false
		}
		point := ContactPoint{OnA: witness, OnB: witness,
			Normal:      VecMeasurement{Value: r3.Vec{Z: normalZ}, Bound: units.Scalar(0), Exactness: Exact},
			NormalAngle: units.Radians(0),
			Separation:  Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}}
		if baseIsA {
			point.FaceA, point.FaceB = faceBase, faceRotated
		} else {
			point.FaceA, point.FaceB = faceRotated, faceBase
		}
		point.FeatureA, point.FeatureB = ContactFeature{Face: point.FaceA}, ContactFeature{Face: point.FaceB}
		points = append(points, point)
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
	return true
}

// clipHorizontalPolygon retains one closed half-plane using exact rational
// intersections. The clipping order fixes a stable source-feature order.
func clipHorizontalPolygon(polygon [][2]*big.Rat, axis int, limit *big.Rat, upper bool) [][2]*big.Rat {
	if len(polygon) == 0 {
		return nil
	}
	inside := func(vertex [2]*big.Rat) bool {
		cmp := vertex[axis].Cmp(limit)
		return upper && cmp <= 0 || !upper && cmp >= 0
	}
	intersection := func(from, to [2]*big.Rat) [2]*big.Rat {
		fraction := new(big.Rat).Quo(new(big.Rat).Sub(limit, from[axis]),
			new(big.Rat).Sub(to[axis], from[axis]))
		other := 1 - axis
		result := from
		result[axis] = new(big.Rat).Set(limit)
		result[other] = new(big.Rat).Add(from[other],
			new(big.Rat).Mul(fraction, new(big.Rat).Sub(to[other], from[other])))
		return result
	}
	output := make([][2]*big.Rat, 0, len(polygon)+2)
	from := polygon[len(polygon)-1]
	fromInside := inside(from)
	for _, to := range polygon {
		toInside := inside(to)
		if fromInside != toInside {
			output = append(output, intersection(from, to))
		}
		if toInside {
			output = append(output, to)
		}
		from, fromInside = to, toInside
	}
	return output
}

func deduplicateHorizontalPolygon(polygon [][2]*big.Rat) [][2]*big.Rat {
	unique := make([][2]*big.Rat, 0, len(polygon))
	for _, point := range polygon {
		if len(unique) == 0 || point[0].Cmp(unique[len(unique)-1][0]) != 0 ||
			point[1].Cmp(unique[len(unique)-1][1]) != 0 {
			unique = append(unique, point)
		}
	}
	if len(unique) > 1 && unique[0][0].Cmp(unique[len(unique)-1][0]) == 0 &&
		unique[0][1].Cmp(unique[len(unique)-1][1]) == 0 {
		unique = unique[:len(unique)-1]
	}
	return unique
}

func horizontalPolygonDoubleArea(polygon [][2]*big.Rat) *big.Rat {
	area := new(big.Rat)
	for i, point := range polygon {
		next := polygon[(i+1)%len(polygon)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(point[0], next[1]),
			new(big.Rat).Mul(point[1], next[0])))
	}
	return area
}
