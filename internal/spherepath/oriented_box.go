package spherepath

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	proof "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// OrientedFaceCorridor proves a sphere stays opposite one isolated box face
// over an affine path. Its support gap is Start + f*Slope in the unnormalized
// outward direction.
func OrientedFaceCorridor(center proof.DyV3, radius proof.Dyadic, startBox box.OrientedBox,
	sphereDelta, boxDelta [3]proof.Dyadic, axis, side int, outward proof.DyV3) (
	proof.Dyadic, proof.Dyadic, bool) {
	for endpoint := range 2 {
		sphereCenter, placedBox := center, startBox
		if endpoint == 1 {
			sphereCenter = proof.DvAdd(center, sphereDelta)
			for corner := range placedBox.Corner {
				placedBox.Corner[corner] = proof.DvAdd(placedBox.Corner[corner], boxDelta)
			}
		}
		faceAxis, faceSide, faceNormal, _, distance2, ok := box.OrientedSphereFace(
			sphereCenter, radius, placedBox)
		if !ok || faceAxis != axis || faceSide != side || !proof.DvEqual(faceNormal, outward) ||
			distance2 == nil {
			return proof.Dyadic{}, proof.Dyadic{}, false
		}
		// The opposite support stays beyond the ball throughout the path.
		face := placedBox.Corner[0]
		if side == 1 {
			face = proof.DvAdd(face, placedBox.Edge[axis])
		}
		d := proof.DvDot(proof.DvSub(sphereCenter, face), outward)
		thickness := proof.DvDot(placedBox.Edge[axis], outward)
		if side == 0 {
			thickness = proof.DyNeg(thickness)
		}
		opposite := proof.DyAdd(d, thickness)
		if proof.DyCmp(proof.DyMul(opposite, opposite), proof.DyMul(proof.DyMul(radius, radius),
			proof.DvDot(outward, outward))) <= 0 {
			return proof.Dyadic{}, proof.Dyadic{}, false
		}
	}
	face := startBox.Corner[0]
	if side == 1 {
		face = proof.DvAdd(face, startBox.Edge[axis])
	}
	start := proof.DvDot(proof.DvSub(center, face), outward)
	slope := proof.DvDot(proof.DvSub(sphereDelta, boxDelta), outward)
	return start, slope, start.Sign() > 0
}

// OrientedFaceSign compares the squared exact center-to-face support with
// the squared radius times the squared face normal at fraction f.
func OrientedFaceSign(start, slope, radius proof.Dyadic, outward proof.DyV3, f *big.Rat) int {
	d := new(big.Rat).Add(start.Rat(), new(big.Rat).Mul(slope.Rat(), f))
	if d.Sign() <= 0 {
		return -1
	}
	radius2 := proof.DyMul(radius, radius).Rat()
	return new(big.Rat).Mul(d, d).Cmp(new(big.Rat).Mul(radius2,
		proof.DvDot(outward, outward).Rat()))
}

// OrientedFaceBracket leaves a positive support gap at the left endpoint
// and a touch or overlap at the right endpoint of a dyadic search grid.
func OrientedFaceBracket(start, slope, radius proof.Dyadic, outward proof.DyV3,
	duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	signAt := func(f *big.Rat) int { return OrientedFaceSign(start, slope, radius, outward, f) }
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if new(big.Rat).Mul(width, big.NewRat(4, 1)).Cmp(resolution) <= 0 {
			leftIndex, rightIndex := big.NewInt(0), new(big.Int).Set(grid)
			for new(big.Int).Sub(rightIndex, leftIndex).Cmp(big.NewInt(1)) > 0 {
				middle := new(big.Int).Add(leftIndex, rightIndex)
				middle.Rsh(middle, 1)
				f := new(big.Rat).SetFrac(middle, grid)
				if signAt(f) > 0 {
					leftIndex = middle
				} else {
					rightIndex = middle
				}
			}
			leftIndex.Sub(leftIndex, big.NewInt(1))
			rightIndex.Add(rightIndex, big.NewInt(2))
			left := new(big.Rat).SetFrac(leftIndex, grid)
			right := new(big.Rat).SetFrac(rightIndex, grid)
			if right.Cmp(big.NewRat(1, 1)) > 0 {
				right = big.NewRat(1, 1)
			}
			endSpan := new(big.Rat).Mul(new(big.Rat).Sub(big.NewRat(1, 1), left), duration)
			if endSpan.Cmp(resolution) <= 0 && signAt(big.NewRat(1, 1)) <= 0 {
				right = big.NewRat(1, 1)
			}
			span := new(big.Rat).Mul(new(big.Rat).Sub(right, left), duration)
			return left, right, left.Sign() > 0 && span.Cmp(resolution) <= 0 &&
				signAt(left) > 0 && signAt(right) <= 0
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

// TranslateObservedBox applies a rounded translation when its basis is
// unchanged from the source pose.
func TranslateObservedBox(source box.OrientedBox, from, at r3.Transform) (box.OrientedBox, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() ||
		!proofbound.FiniteVec(from.Translation()) || !proofbound.FiniteVec(at.Translation()) {
		return box.OrientedBox{}, false
	}
	a, b := from.Translation(), at.Translation()
	before, after := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	for axis := range 3 {
		move := proof.DySubScalar(proof.MustDyOf(after[axis]), proof.MustDyOf(before[axis]))
		for corner := range source.Corner {
			source.Corner[corner][axis] = proof.DyAdd(source.Corner[corner][axis], move)
		}
	}
	return source, true
}

// OrientedSpherePoseDeviation bounds each observed box corner and sphere
// center against its exact affine source path.
func OrientedSpherePoseDeviation(startBox, observedBox box.OrientedBox,
	boxDelta [3]proof.Dyadic, startCenter, observedCenter proof.DyV3,
	sphereDelta [3]proof.Dyadic, f *big.Rat) *big.Rat {
	maximum := new(big.Rat)
	for corner := range startBox.Corner {
		sum := new(big.Rat)
		for axis := range 3 {
			expected := new(big.Rat).Add(startBox.Corner[corner][axis].Rat(),
				new(big.Rat).Mul(boxDelta[axis].Rat(), f))
			difference := new(big.Rat).Sub(observedBox.Corner[corner][axis].Rat(), expected)
			sum.Add(sum, difference.Abs(difference))
		}
		if sum.Cmp(maximum) > 0 {
			maximum = sum
		}
	}
	for axis := range 3 {
		expected := new(big.Rat).Add(startCenter[axis].Rat(),
			new(big.Rat).Mul(sphereDelta[axis].Rat(), f))
		difference := new(big.Rat).Sub(observedCenter[axis].Rat(), expected)
		maximum.Add(maximum, difference.Abs(difference))
	}
	return maximum
}
