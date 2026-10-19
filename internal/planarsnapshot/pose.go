package planarsnapshot

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PoseScale is an exact upper bound, at least one, on how far the linear part
// L of an admitted transform stretches any length. The largest absolute row
// sum g of the Gram matrix LᵀL bounds its largest eigenvalue, which is |L|²,
// and (1 + g)/2 bounds √g from above. A valid r3 transform is orthonormal to
// rounding, so the bound sits a hair above one; it is charged all the same,
// since the held mesh and its displacement move through the exact float map,
// not through a rotation.
func PoseScale(t r3.Transform) proofarith.Dyadic {
	basis := t.Basis()
	columns := [3]proofarith.DyV3{proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)}
	one := proofarith.DyInt(1)
	g := one
	for i := range 3 {
		row := proofarith.DyZero()
		for j := range 3 {
			row = proofarith.DyAdd(row, proofarith.DyAbs(proofarith.DvDot(columns[i], columns[j])))
		}
		if proofarith.DyCmp(row, g) > 0 {
			g = row
		}
	}
	return proofarith.DyShift(proofarith.DyAdd(one, g), -1)
}

// PositiveAffine admits finite transforms with an exact positive determinant.
func PositiveAffine(t r3.Transform) bool {
	if !t.IsValid() || !proofbound.FiniteVec(t.Translation()) {
		return false
	}
	basis := t.Basis()
	if !proofbound.FiniteVec(basis.EX) || !proofbound.FiniteVec(basis.EY) || !proofbound.FiniteVec(basis.EZ) {
		return false
	}
	ex, ey, ez := proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)
	return proofarith.DvDot(ex, proofarith.DvCross(ey, ez)).Sign() > 0
}
