package box

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// OrientedPatchFailure identifies the exact geometry gate that refused a patch.
type OrientedPatchFailure uint8

const (
	OrientedPatchReady OrientedPatchFailure = iota
	OrientedPatchNoNormal
	OrientedPatchAmbiguous
)

// OrientedPatchGeometry holds the face slots and exact corners of the
// intersection rectangle in A's dual basis.
type OrientedPatchGeometry struct {
	FaceA, FaceB FaceSlot
	Normal       proofarith.DyV3
	limits       [3][2]*big.Rat
	tangents     [2]int
}

// OrientedFacePatch proves one opposed face patch for parallel source edges.
func OrientedFacePatch(a, b OrientedBox) (OrientedPatchGeometry, OrientedPatchFailure) {
	var dual [3]proofarith.DyV3
	var denominator [3]*big.Rat
	var bAxis [3]int
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		dual[axis] = proofarith.DvCross(a.Edge[i], a.Edge[j])
		denominator[axis] = proofarith.DvDot(a.Edge[axis], dual[axis]).Rat()
		if denominator[axis].Sign() == 0 {
			return OrientedPatchGeometry{}, OrientedPatchNoNormal
		}
		if denominator[axis].Sign() < 0 {
			for k := range 3 {
				dual[axis][k] = proofarith.DyNeg(dual[axis][k])
			}
			denominator[axis].Neg(denominator[axis])
		}
		bAxis[axis] = -1
		for candidate := range 3 {
			if !proofarith.DvIsZero(proofarith.DvCross(a.Edge[axis], b.Edge[candidate])) {
				continue
			}
			if bAxis[axis] >= 0 {
				return OrientedPatchGeometry{}, OrientedPatchAmbiguous
			}
			bAxis[axis] = candidate
		}
		if bAxis[axis] < 0 {
			return OrientedPatchGeometry{}, OrientedPatchNoNormal
		}
	}
	if bAxis[0] == bAxis[1] || bAxis[1] == bAxis[2] || bAxis[0] == bAxis[2] {
		return OrientedPatchGeometry{}, OrientedPatchAmbiguous
	}
	var alo, ahi, blo, bhi [3]*big.Rat
	for axis := range 3 {
		alo[axis], ahi[axis] = big.NewRat(0, 1), big.NewRat(1, 1)
		for _, corner := range b.Corner {
			coordinate := new(big.Rat).Quo(proofarith.DvDot(proofarith.DvSub(corner, a.Corner[0]), dual[axis]).Rat(),
				denominator[axis])
			if blo[axis] == nil || coordinate.Cmp(blo[axis]) < 0 {
				blo[axis] = coordinate
			}
			if bhi[axis] == nil || coordinate.Cmp(bhi[axis]) > 0 {
				bhi[axis] = coordinate
			}
		}
	}
	contactAxis, sign := -1, 0
	for axis := range 3 {
		switch {
		case ahi[axis].Cmp(blo[axis]) == 0:
			if contactAxis >= 0 {
				return OrientedPatchGeometry{}, OrientedPatchAmbiguous
			}
			contactAxis, sign = axis, 1
		case bhi[axis].Cmp(alo[axis]) == 0:
			if contactAxis >= 0 {
				return OrientedPatchGeometry{}, OrientedPatchAmbiguous
			}
			contactAxis, sign = axis, -1
		default:
			if alo[axis].Cmp(bhi[axis]) >= 0 || blo[axis].Cmp(ahi[axis]) >= 0 {
				return OrientedPatchGeometry{}, OrientedPatchAmbiguous
			}
		}
	}
	if contactAxis < 0 {
		return OrientedPatchGeometry{}, OrientedPatchAmbiguous
	}
	var limits [3][2]*big.Rat
	var tangents [2]int
	n := 0
	for axis := range 3 {
		if axis == contactAxis {
			value := alo[axis]
			if sign > 0 {
				value = ahi[axis]
			}
			limits[axis] = [2]*big.Rat{value, value}
			continue
		}
		limits[axis] = [2]*big.Rat{proofbound.RatMax(alo[axis], blo[axis]), proofbound.RatMin(ahi[axis], bhi[axis])}
		if limits[axis][0].Cmp(limits[axis][1]) >= 0 {
			return OrientedPatchGeometry{}, OrientedPatchAmbiguous
		}
		tangents[n] = axis
		n++
	}
	aSide := 0
	if sign > 0 {
		aSide = 1
	}
	bSide := 0
	// Hold the dual axis in a named local. Go 1.24 through 1.27 can reuse an
	// indexed element pointer after DvDot and lose its big.Int mantissas.
	normalAxis := dual[contactAxis]
	if proofarith.DvDot(b.Edge[bAxis[contactAxis]], normalAxis).Sign()*sign > 0 {
		bSide = 0
	} else {
		bSide = 1
	}
	if sign < 0 {
		for k := range 3 {
			normalAxis[k] = proofarith.DyNeg(normalAxis[k])
		}
	}
	return OrientedPatchGeometry{
		FaceA:  FaceSlot{Axis: contactAxis, Side: aSide},
		FaceB:  FaceSlot{Axis: bAxis[contactAxis], Side: bSide},
		Normal: normalAxis, limits: limits, tangents: tangents,
	}, OrientedPatchReady
}

// Point returns one exact rectangle corner in the caller's chosen order.
func (g OrientedPatchGeometry) Point(a OrientedBox, corner [2]int) [3]*big.Rat {
	coordinates := [3]*big.Rat{}
	for axis := range 3 {
		coordinates[axis] = g.limits[axis][0]
	}
	for i, axis := range g.tangents {
		coordinates[axis] = g.limits[axis][corner[i]]
	}
	var point [3]*big.Rat
	for k := range 3 {
		point[k] = a.Corner[0][k].Rat()
		for axis := range 3 {
			point[k] = new(big.Rat).Add(point[k],
				new(big.Rat).Mul(a.Edge[axis][k].Rat(), coordinates[axis]))
		}
	}
	return point
}
