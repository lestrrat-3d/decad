package box

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ClosestOrthogonalBoxPoint minimizes exact squared distance for a box with
// pairwise orthogonal, nonzero edges. Callers must establish orthogonality.
func ClosestOrthogonalBoxPoint(center proofarith.DyV3, box OrientedBox) (*big.Rat, bool) {
	var value [3]*big.Rat
	delta := proofarith.DvSub(center, box.Corner[0])
	for axis := range 3 {
		length2 := proofarith.DvDot(box.Edge[axis], box.Edge[axis]).Rat()
		if length2.Sign() <= 0 {
			return nil, false
		}
		coefficient := new(big.Rat).Quo(proofarith.DvDot(delta, box.Edge[axis]).Rat(), length2)
		if coefficient.Sign() < 0 {
			coefficient.SetInt64(0)
		} else if coefficient.Cmp(big.NewRat(1, 1)) > 0 {
			coefficient.SetInt64(1)
		}
		value[axis] = coefficient
	}
	distance2 := new(big.Rat)
	for k := range 3 {
		coordinate := box.Corner[0][k].Rat()
		for axis := range 3 {
			coordinate.Add(coordinate, new(big.Rat).Mul(value[axis], box.Edge[axis][k].Rat()))
		}
		delta := new(big.Rat).Sub(center[k].Rat(), coordinate)
		distance2.Add(distance2, new(big.Rat).Mul(delta, delta))
	}
	return distance2, true
}

// OrientedSphereFace identifies one isolated exterior support. The strict
// dual-coordinate margins hold the entire projected ball inside the face.
func OrientedSphereFace(center proofarith.DyV3, radius proofarith.Dyadic, box OrientedBox) (
	int, int, proofarith.DyV3, [3]*big.Rat, *big.Rat, bool) {
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		dual := proofarith.DvCross(box.Edge[i], box.Edge[j])
		denom := proofarith.DvDot(box.Edge[axis], dual)
		if denom.Sign() == 0 {
			return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
		}
		if denom.Sign() < 0 {
			for k := range 3 {
				dual[k] = proofarith.DyNeg(dual[k])
			}
			denom = proofarith.DyNeg(denom)
		}
		var coordinate [3]*big.Rat
		for k := range 3 {
			denominator := OrientedDenominator(box, k).Rat()
			denominator.Abs(denominator)
			coordinate[k] = new(big.Rat).Quo(proofarith.DvDot(proofarith.DvSub(center, box.Corner[0]),
				OrientedDual(box, k)).Rat(), denominator)
		}
		side := -1
		if coordinate[axis].Sign() < 0 {
			side = 0
		} else if coordinate[axis].Cmp(big.NewRat(1, 1)) > 0 {
			side = 1
		}
		if side < 0 {
			continue
		}
		norm2 := proofarith.DvDot(dual, dual).Rat()
		plane := new(big.Rat).Mul(coordinate[axis], denom.Rat())
		if side == 1 {
			plane.Sub(plane, denom.Rat())
		} else {
			plane.Neg(plane)
		}
		if plane.Sign() <= 0 {
			continue
		}
		for k := range 3 {
			if k != axis && !orientedSphereMargin(radius, box, k, coordinate[k]) {
				return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
			}
		}
		if side == 0 {
			for k := range 3 {
				dual[k] = proofarith.DyNeg(dual[k])
			}
		}
		planeDistance2 := new(big.Rat).Quo(new(big.Rat).Mul(plane, plane), norm2)
		return axis, side, dual, coordinate, planeDistance2, true
	}
	return 0, 0, proofarith.DyV3{}, [3]*big.Rat{}, nil, false
}

func OrientedDual(box OrientedBox, axis int) proofarith.DyV3 {
	i, j := (axis+1)%3, (axis+2)%3
	dual := proofarith.DvCross(box.Edge[i], box.Edge[j])
	if OrientedDenominator(box, axis).Sign() < 0 {
		for k := range 3 {
			dual[k] = proofarith.DyNeg(dual[k])
		}
	}
	return dual
}

func OrientedDenominator(box OrientedBox, axis int) proofarith.Dyadic {
	i, j := (axis+1)%3, (axis+2)%3
	return proofarith.DvDot(box.Edge[axis], proofarith.DvCross(box.Edge[i], box.Edge[j]))
}

func orientedSphereMargin(radius proofarith.Dyadic, box OrientedBox, axis int, coordinate *big.Rat) bool {
	if coordinate.Sign() <= 0 || coordinate.Cmp(big.NewRat(1, 1)) >= 0 {
		return false
	}
	dual := OrientedDual(box, axis)
	denom := OrientedDenominator(box, axis).Rat()
	margin := new(big.Rat).Set(coordinate)
	remaining := new(big.Rat).Sub(big.NewRat(1, 1), coordinate)
	if remaining.Cmp(margin) < 0 {
		margin = remaining
	}
	left := new(big.Rat).Mul(margin, margin)
	left.Mul(left, new(big.Rat).Mul(denom, denom))
	right := new(big.Rat).Mul(radius.Rat(), radius.Rat())
	right.Mul(right, proofarith.DvDot(dual, dual).Rat())
	return left.Cmp(right) > 0
}

func OrthogonalSourceBox(box OrientedBox) bool {
	for i := range 3 {
		for j := i + 1; j < 3; j++ {
			if !proofarith.DvDot(box.Edge[i], box.Edge[j]).IsZero() {
				return false
			}
		}
	}
	return true
}
