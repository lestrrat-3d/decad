package box

import (
	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// AxisFaceWitness places one point strictly inside two opposed axis-normal
// source faces. The caller reads and publishes its rounded points and faces.
type AxisFaceWitness struct {
	PointA, PointB proofarith.DyV3
	Separation     proofarith.Dyadic
	Axis, Sign     int
	SideA, SideB   int
}

// AxisFaceWitnesses lists candidates in axis and sign order. A caller keeps
// the shallowest candidate whose point and face readings meet its request.
func AxisFaceWitnesses(a, b OrientedBox, relation pair.Relation) []AxisFaceWitness {
	var out []AxisFaceWitness
	for axis := range 3 {
		for _, sign := range []int{1, -1} {
			sideA, sideB := 1, 0
			if sign < 0 {
				sideA, sideB = 0, 1
			}
			var faceA, faceB OrientedFace
			if !OrientedAxisFace(&a, axis, sideA, &faceA) ||
				!OrientedAxisFace(&b, axis, sideB, &faceB) {
				continue
			}
			var candidate proofarith.DyV3
			OrientedFaceCenter(&faceA, &candidate)
			if !OrientedFaceContainsProjection(&faceB, &candidate, axis) {
				OrientedFaceCenter(&faceB, &candidate)
				if !OrientedFaceContainsProjection(&faceA, &candidate, axis) {
					continue
				}
			}
			separation := proofarith.DySubScalar(faceB.Origin[axis], faceA.Origin[axis])
			if sign < 0 {
				separation = proofarith.DyNeg(separation)
			}
			if separation.Sign() > 0 && relation != pair.Separated {
				continue
			}
			if relation == pair.Touching && separation.Sign() != 0 {
				continue
			}
			pointA, pointB := candidate, candidate
			pointA[axis] = faceA.Origin[axis]
			pointB[axis] = faceB.Origin[axis]
			out = append(out, AxisFaceWitness{
				PointA: pointA, PointB: pointB, Separation: separation,
				Axis: axis, Sign: sign, SideA: sideA, SideB: sideB,
			})
		}
	}
	return out
}
