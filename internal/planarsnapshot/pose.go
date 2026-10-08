package planarsnapshot

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PoseScale is an exact upper bound of at least one on the stretch of a
// transform's linear part, read from its Gram matrix.
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
