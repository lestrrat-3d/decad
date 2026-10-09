package decad

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
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

// axisAlignedOrientedBox reads box as an axis-aligned box when every source
// edge lies along one world axis.
//
// It reads each Dyadic it compares into a named local before the comparison,
// never as an element of a local DyV3 indexed by a loop variable: Go 1.26.0
// to 1.26.5's stack-slot merging gave such a range copy the slot of an
// inlined dyMax argument while the indexed read still needed it, and the
// box's hi corner came back as its lo one.
func axisAlignedOrientedBox(box orientedSourceBox) (sourceBoxContactProof, bool) {
	var aligned sourceBoxContactProof
	var used [3]bool
	for sourceAxis := range box.edge {
		e0, e1, e2 := box.edge[sourceAxis][0], box.edge[sourceAxis][1], box.edge[sourceAxis][2]
		signs := [3]int{e0.Sign(), e1.Sign(), e2.Sign()}
		worldAxis := -1
		for axis, sign := range signs {
			if sign != 0 {
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
			if signs[worldAxis] < 0 {
				worldSide = 1 - side
			}
			aligned.faces[worldAxis][worldSide] = box.faces[sourceAxis][side]
		}
	}
	for axis := range 3 {
		lo, hi := box.corner[0][axis], box.corner[0][axis]
		for k := 1; k < len(box.corner); k++ {
			v := box.corner[k][axis]
			if proofarith.DyCmp(v, lo) < 0 {
				lo = v
			}
			if proofarith.DyCmp(v, hi) > 0 {
				hi = v
			}
		}
		aligned.lo[axis], aligned.hi[axis] = lo, hi
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
	minZ, maxZ := pairbox.OrientedProjection(rotated.pairBox(),
		proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)})
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
		polygon = pairbox.ClipHorizontalPolygon(polygon, axis, base.lo[axis].Rat(), false)
		polygon = pairbox.ClipHorizontalPolygon(polygon, axis, base.hi[axis].Rat(), true)
	}
	polygon = pairbox.DeduplicateHorizontalPolygon(polygon)
	if len(polygon) < 3 || pairbox.HorizontalPolygonDoubleArea(polygon).Sign() == 0 {
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
