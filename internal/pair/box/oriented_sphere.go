package box

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ClosestOrientedBoxPoint minimizes squared Euclidean distance over the exact
// parallelotope. Each coordinate is free or fixed at either endpoint; the
// twenty-seven resulting stationary candidates cover every face, edge and
// vertex of the compact convex box.
func ClosestOrientedBoxPoint(center proofarith.DyV3, box OrientedBox) (*big.Rat, bool) {
	var best *big.Rat
	for code := range 27 {
		status, value := code, [3]*big.Rat{}
		var free [3]int
		count := 0
		point := [3]*big.Rat{}
		for k := range 3 {
			point[k] = box.Corner[0][k].Rat()
		}
		for axis := range 3 {
			side := status % 3
			status /= 3
			if side == 2 {
				free[count], count = axis, count+1
				continue
			}
			value[axis] = big.NewRat(int64(side), 1)
			if side == 1 {
				for k := range 3 {
					point[k].Add(point[k], box.Edge[axis][k].Rat())
				}
			}
		}
		if count > 0 {
			matrix := make([][]*big.Rat, count)
			for i := range count {
				matrix[i] = make([]*big.Rat, count+1)
				for j := range count {
					matrix[i][j] = proofarith.DvDot(box.Edge[free[i]], box.Edge[free[j]]).Rat()
				}
				matrix[i][count] = new(big.Rat)
				for k := range 3 {
					term := new(big.Rat).Sub(center[k].Rat(), point[k])
					matrix[i][count].Add(matrix[i][count],
						new(big.Rat).Mul(box.Edge[free[i]][k].Rat(), term))
				}
			}
			if !solvePositiveGram(matrix) {
				return nil, false
			}
			valid := true
			for i := range count {
				candidate := matrix[i][count]
				if candidate.Sign() < 0 || candidate.Cmp(big.NewRat(1, 1)) > 0 {
					valid = false
					break
				}
				value[free[i]] = candidate
			}
			if !valid {
				continue
			}
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
		if best == nil || distance2.Cmp(best) < 0 {
			best = distance2
		}
	}
	return best, best != nil
}

func solvePositiveGram(matrix [][]*big.Rat) bool {
	n := len(matrix)
	for pivot := range n {
		if matrix[pivot][pivot].Sign() <= 0 {
			return false
		}
		denominator := new(big.Rat).Set(matrix[pivot][pivot])
		for column := pivot; column <= n; column++ {
			matrix[pivot][column] = new(big.Rat).Quo(matrix[pivot][column], denominator)
		}
		for row := pivot + 1; row < n; row++ {
			factor := new(big.Rat).Set(matrix[row][pivot])
			for column := pivot; column <= n; column++ {
				matrix[row][column] = new(big.Rat).Sub(matrix[row][column],
					new(big.Rat).Mul(factor, matrix[pivot][column]))
			}
		}
	}
	for row := n - 1; row >= 0; row-- {
		for column := row + 1; column < n; column++ {
			matrix[row][n] = new(big.Rat).Sub(matrix[row][n],
				new(big.Rat).Mul(matrix[row][column], matrix[column][n]))
		}
	}
	return true
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
